///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License.                     //
///////////////////////////////////////////////////////////////////////////////////

#include "camerayololitert.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>

#include <QDebug>
#include <QStringList>

#ifdef CAMERA_LITERT_YOLO
#include <EGL/egl.h>
#include <GLES3/gl31.h>

#include <litert/c/litert_common.h>
#include <litert/c/litert_compiled_model.h>
#include <litert/c/litert_environment.h>
#include <litert/c/litert_model.h>
#include <litert/c/litert_model_types.h>
#include <litert/c/litert_options.h>
#include <litert/c/litert_tensor_buffer.h>
#include <litert/c/litert_tensor_buffer_requirements.h>
#endif

struct CameraYoloLiteRt::Impl
{
#ifdef CAMERA_LITERT_YOLO
    struct TensorInfo
    {
        LiteRtRankedTensorType m_type {};
        LiteRtQuantizationPerTensor m_quantization {1.0f, 0};
        size_t m_elements = 0;
        bool m_hasPerTensorQuantization = false;
    };

    LiteRtEnvironment m_environment = nullptr;
    LiteRtModel m_model = nullptr;
    LiteRtOptions m_options = nullptr;
    LiteRtCompiledModel m_compiledModel = nullptr;
    std::vector<LiteRtTensorBuffer> m_inputBuffers;
    std::vector<LiteRtTensorBuffer> m_outputBuffers;
    std::vector<GLuint> m_glBufferIds;
    TensorInfo m_inputInfo;
    std::vector<TensorInfo> m_outputInfo;
    EGLDisplay m_eglDisplay = EGL_NO_DISPLAY;
    EGLContext m_eglContext = EGL_NO_CONTEXT;
    EGLSurface m_eglSurface = EGL_NO_SURFACE;
    bool m_eglInitialized = false;
#endif
    QString m_modelPath;
    cv::Size m_inputSize;
    bool m_requestedGpu = false;
    bool m_gpuActive = false;
    bool m_loggedOutputRanges = false;
    QString m_gpuFallbackReason;

#ifdef CAMERA_LITERT_YOLO
    static QString eglError(const QString& operation)
    {
        return QStringLiteral("%1 (EGL error 0x%2)")
            .arg(operation)
            .arg(static_cast<unsigned int>(eglGetError()), 4, 16, QLatin1Char('0'));
    }

    bool makeEglCurrent(QString& error) const
    {
        if (m_eglContext == EGL_NO_CONTEXT) {
            error = QStringLiteral("LiteRT GPU EGL context is not initialized");
            return false;
        }
        if (eglGetCurrentContext() == m_eglContext) {
            return true;
        }
        if (eglMakeCurrent(m_eglDisplay, m_eglSurface, m_eglSurface, m_eglContext) != EGL_TRUE)
        {
            error = eglError(QStringLiteral("Cannot make the LiteRT GPU EGL context current"));
            return false;
        }
        return true;
    }

    void destroyEglContext()
    {
        if (m_eglDisplay == EGL_NO_DISPLAY) {
            return;
        }
        if (eglGetCurrentContext() == m_eglContext) {
            eglMakeCurrent(m_eglDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        }
        if (m_eglSurface != EGL_NO_SURFACE) {
            eglDestroySurface(m_eglDisplay, m_eglSurface);
        }
        if (m_eglContext != EGL_NO_CONTEXT) {
            eglDestroyContext(m_eglDisplay, m_eglContext);
        }
        if (m_eglInitialized) {
            eglTerminate(m_eglDisplay);
        }
        m_eglDisplay = EGL_NO_DISPLAY;
        m_eglContext = EGL_NO_CONTEXT;
        m_eglSurface = EGL_NO_SURFACE;
        m_eglInitialized = false;
    }

    bool createEglContext(QString& error)
    {
        m_eglDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (m_eglDisplay == EGL_NO_DISPLAY)
        {
            error = eglError(QStringLiteral("Cannot get an EGL display for LiteRT GPU"));
            return false;
        }

        EGLint major = 0;
        EGLint minor = 0;
        if (eglInitialize(m_eglDisplay, &major, &minor) != EGL_TRUE)
        {
            error = eglError(QStringLiteral("Cannot initialize EGL for LiteRT GPU"));
            destroyEglContext();
            return false;
        }
        m_eglInitialized = true;
        if (eglBindAPI(EGL_OPENGL_ES_API) != EGL_TRUE)
        {
            error = eglError(QStringLiteral("Cannot bind the OpenGL ES API for LiteRT GPU"));
            destroyEglContext();
            return false;
        }

        const EGLint configAttributes[] = {
            EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT_KHR,
            EGL_RED_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_BLUE_SIZE, 8,
            EGL_NONE
        };
        EGLConfig config = nullptr;
        EGLint configCount = 0;
        if ((eglChooseConfig(m_eglDisplay, configAttributes, &config, 1, &configCount) != EGL_TRUE)
            || (configCount == 0))
        {
            error = eglError(QStringLiteral("Cannot choose an OpenGL ES 3 EGL configuration for LiteRT GPU"));
            destroyEglContext();
            return false;
        }

        const EGLint contextAttributes[] = {
            EGL_CONTEXT_CLIENT_VERSION, 3,
            EGL_NONE
        };
        m_eglContext = eglCreateContext(
            m_eglDisplay, config, EGL_NO_CONTEXT, contextAttributes);
        if (m_eglContext == EGL_NO_CONTEXT)
        {
            error = eglError(QStringLiteral("Cannot create the LiteRT GPU EGL context"));
            destroyEglContext();
            return false;
        }

        const EGLint surfaceAttributes[] = {
            EGL_WIDTH, 1,
            EGL_HEIGHT, 1,
            EGL_NONE
        };
        m_eglSurface = eglCreatePbufferSurface(m_eglDisplay, config, surfaceAttributes);
        if (m_eglSurface == EGL_NO_SURFACE)
        {
            error = eglError(QStringLiteral("Cannot create the LiteRT GPU EGL pbuffer"));
            destroyEglContext();
            return false;
        }
        if (!makeEglCurrent(error))
        {
            destroyEglContext();
            return false;
        }

        qDebug() << "CameraYoloLiteRt: created dedicated EGL context"
                 << major << minor
                 << reinterpret_cast<const char *>(glGetString(GL_RENDERER))
                 << reinterpret_cast<const char *>(glGetString(GL_VERSION));
        return true;
    }
#endif

    void reset()
    {
#ifdef CAMERA_LITERT_YOLO
        QString eglCurrentError;
        if ((m_eglContext != EGL_NO_CONTEXT) && !makeEglCurrent(eglCurrentError)) {
            qWarning() << "CameraYoloLiteRt:" << eglCurrentError;
        }
        for (LiteRtTensorBuffer buffer : m_outputBuffers) {
            LiteRtDestroyTensorBuffer(buffer);
        }
        m_outputBuffers.clear();
        for (LiteRtTensorBuffer buffer : m_inputBuffers) {
            LiteRtDestroyTensorBuffer(buffer);
        }
        m_inputBuffers.clear();
        if (!m_glBufferIds.empty()) {
            glDeleteBuffers(static_cast<GLsizei>(m_glBufferIds.size()), m_glBufferIds.data());
            m_glBufferIds.clear();
        }
        m_outputInfo.clear();
        m_inputInfo = {};
        if (m_compiledModel) {
            LiteRtDestroyCompiledModel(m_compiledModel);
            m_compiledModel = nullptr;
        }
        if (m_options) {
            LiteRtDestroyOptions(m_options);
            m_options = nullptr;
        }
        if (m_model) {
            LiteRtDestroyModel(m_model);
            m_model = nullptr;
        }
        if (m_environment) {
            LiteRtDestroyEnvironment(m_environment);
            m_environment = nullptr;
        }
        destroyEglContext();
#endif
        m_modelPath.clear();
        m_inputSize = {};
        m_requestedGpu = false;
        m_gpuActive = false;
        m_loggedOutputRanges = false;
        m_gpuFallbackReason.clear();
    }

#ifdef CAMERA_LITERT_YOLO
    static QString statusError(const QString& operation, LiteRtStatus status)
    {
        const char *description = "unknown error";
        switch (status)
        {
        case kLiteRtStatusOk: description = "OK"; break;
        case kLiteRtStatusErrorInvalidArgument: description = "invalid argument"; break;
        case kLiteRtStatusErrorMemoryAllocationFailure: description = "memory allocation failure"; break;
        case kLiteRtStatusErrorRuntimeFailure: description = "runtime failure"; break;
        case kLiteRtStatusErrorMissingInputTensor: description = "missing input tensor"; break;
        case kLiteRtStatusErrorUnsupported: description = "unsupported"; break;
        case kLiteRtStatusErrorNotFound: description = "not found"; break;
        case kLiteRtStatusErrorTimeoutExpired: description = "timeout expired"; break;
        case kLiteRtStatusErrorWrongVersion: description = "wrong version"; break;
        case kLiteRtStatusErrorAlreadyExists: description = "already exists"; break;
        case kLiteRtStatusCancelled: description = "cancelled"; break;
        case kLiteRtStatusErrorFileIO: description = "file I/O error"; break;
        case kLiteRtStatusErrorInvalidFlatbuffer: description = "invalid FlatBuffer"; break;
        case kLiteRtStatusErrorDynamicLoading: description = "dynamic loading failure"; break;
        case kLiteRtStatusErrorSerialization: description = "serialization failure"; break;
        case kLiteRtStatusErrorCompilation: description = "compilation failure"; break;
        case kLiteRtStatusErrorShapeInferenceFailed: description = "shape inference failure"; break;
        default: break;
        }
        return QStringLiteral("LiteRT failed to %1: %2 (%3)")
            .arg(operation, QString::fromLatin1(description))
            .arg(static_cast<int>(status));
    }

    static bool checkStatus(LiteRtStatus status, const QString& operation, QString& error)
    {
        if (status == kLiteRtStatusOk) {
            return true;
        }
        error = statusError(operation, status);
        return false;
    }

    static bool readTensorInfo(LiteRtTensor tensor, TensorInfo& info, QString& error)
    {
        LiteRtTensorTypeId typeId;
        if (!checkStatus(LiteRtGetTensorTypeId(tensor, &typeId), QStringLiteral("query tensor type"), error)) {
            return false;
        }
        if (typeId != kLiteRtRankedTensorType)
        {
            error = QStringLiteral("YOLO LiteRT tensors must have a fixed rank");
            return false;
        }
        if (!checkStatus(LiteRtGetRankedTensorType(tensor, &info.m_type), QStringLiteral("query tensor dimensions"), error)) {
            return false;
        }
        if ((info.m_type.layout.rank == 0) || (info.m_type.layout.rank > LITERT_TENSOR_MAX_RANK))
        {
            error = QStringLiteral("YOLO LiteRT tensor has an invalid rank");
            return false;
        }

        size_t elements = 1;
        for (unsigned int dimension = 0; dimension < info.m_type.layout.rank; ++dimension)
        {
            const int32_t size = info.m_type.layout.dimensions[dimension];
            if ((size <= 0) || (elements > std::numeric_limits<size_t>::max() / static_cast<size_t>(size)))
            {
                error = QStringLiteral("YOLO LiteRT tensor has invalid or oversized dimensions");
                return false;
            }
            elements *= static_cast<size_t>(size);
        }
        info.m_elements = elements;

        LiteRtQuantizationTypeId quantizationType = kLiteRtQuantizationNone;
        if (LiteRtGetQuantizationTypeId(tensor, &quantizationType) == kLiteRtStatusOk
            && quantizationType == kLiteRtQuantizationPerTensor)
        {
            if (!checkStatus(
                    LiteRtGetPerTensorQuantization(tensor, &info.m_quantization),
                    QStringLiteral("query tensor quantization"), error)) {
                return false;
            }
            info.m_hasPerTensorQuantization = true;
        }
        return true;
    }

    static QString tensorShape(const TensorInfo& info)
    {
        QStringList dimensions;
        for (unsigned int dimension = 0; dimension < info.m_type.layout.rank; ++dimension) {
            dimensions.append(QString::number(info.m_type.layout.dimensions[dimension]));
        }
        return dimensions.join(QLatin1Char('x'));
    }

    static QString bufferTypeName(LiteRtTensorBufferType type)
    {
        switch (type)
        {
        case kLiteRtTensorBufferTypeHostMemory: return QStringLiteral("HostMemory");
        case kLiteRtTensorBufferTypeAhwb: return QStringLiteral("AHardwareBuffer");
        case kLiteRtTensorBufferTypeIon: return QStringLiteral("ION");
        case kLiteRtTensorBufferTypeDmaBuf: return QStringLiteral("DMA-BUF");
        case kLiteRtTensorBufferTypeFastRpc: return QStringLiteral("FastRPC");
        case kLiteRtTensorBufferTypeGlBuffer: return QStringLiteral("GL buffer");
        case kLiteRtTensorBufferTypeGlTexture: return QStringLiteral("GL texture");
        case kLiteRtTensorBufferTypeOpenClBuffer: return QStringLiteral("OpenCL buffer");
        case kLiteRtTensorBufferTypeOpenClBufferFp16: return QStringLiteral("OpenCL FP16 buffer");
        case kLiteRtTensorBufferTypeOpenClTexture: return QStringLiteral("OpenCL texture");
        case kLiteRtTensorBufferTypeOpenClTextureFp16: return QStringLiteral("OpenCL FP16 texture");
        default: return QString::number(static_cast<int>(type));
        }
    }

    bool createBuffer(
        const QString& description,
        const TensorInfo& info,
        LiteRtTensorBufferRequirements requirements,
        LiteRtTensorBuffer& buffer,
        QString& error)
    {
        int typeCount = 0;
        size_t bufferSize = 0;
        size_t alignment = 0;
        if (!checkStatus(LiteRtGetNumTensorBufferRequirementsSupportedBufferTypes(
                    requirements, &typeCount),
                QStringLiteral("query %1 buffer types").arg(description), error)
            || !checkStatus(LiteRtGetTensorBufferRequirementsBufferSize(
                    requirements, &bufferSize),
                QStringLiteral("query %1 buffer size").arg(description), error)
            || !checkStatus(LiteRtGetTensorBufferRequirementsAlignment(
                    requirements, &alignment),
                QStringLiteral("query %1 buffer alignment").arg(description), error)) {
            return false;
        }

        bool supportsHostMemory = false;
        bool supportsGlBuffer = false;
        QStringList typeNames;
        for (int typeIndex = 0; typeIndex < typeCount; ++typeIndex)
        {
            LiteRtTensorBufferType type = kLiteRtTensorBufferTypeUnknown;
            if (!checkStatus(LiteRtGetTensorBufferRequirementsSupportedTensorBufferType(
                        requirements, typeIndex, &type),
                    QStringLiteral("query %1 buffer type %2").arg(description).arg(typeIndex), error)) {
                return false;
            }
            supportsHostMemory |= type == kLiteRtTensorBufferTypeHostMemory;
            supportsGlBuffer |= type == kLiteRtTensorBufferTypeGlBuffer;
            typeNames.append(bufferTypeName(type));
        }

        qDebug() << "CameraYoloLiteRt:" << description << "buffer requirements"
                 << "types" << typeNames.join(QStringLiteral(", "))
                 << "bytes" << bufferSize << "alignment" << alignment;

        if (supportsHostMemory)
        {
            return checkStatus(LiteRtCreateManagedTensorBuffer(
                m_environment, kLiteRtTensorBufferTypeHostMemory,
                &info.m_type, bufferSize, &buffer),
                QStringLiteral("allocate the %1 tensor buffer as host memory").arg(description),
                error);
        }

        if (supportsGlBuffer)
        {
            if (!makeEglCurrent(error)) {
                return false;
            }
            while (glGetError() != GL_NO_ERROR) {}
            GLuint glBuffer = 0;
            glGenBuffers(1, &glBuffer);
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, glBuffer);
            glBufferData(GL_SHADER_STORAGE_BUFFER,
                static_cast<GLsizeiptr>(bufferSize), nullptr, GL_DYNAMIC_DRAW);
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
            const GLenum glError = glGetError();
            if ((glBuffer == 0) || (glError != GL_NO_ERROR))
            {
                if (glBuffer != 0) {
                    glDeleteBuffers(1, &glBuffer);
                }
                error = QStringLiteral("Cannot allocate the %1 LiteRT GL tensor buffer (GL error 0x%2)")
                    .arg(description)
                    .arg(static_cast<unsigned int>(glError), 4, 16, QLatin1Char('0'));
                return false;
            }

            const LiteRtStatus status = LiteRtCreateTensorBufferFromGlBuffer(
                m_environment, &info.m_type, GL_SHADER_STORAGE_BUFFER,
                glBuffer, bufferSize, 0, nullptr, &buffer);
            if (!checkStatus(status,
                    QStringLiteral("wrap the %1 OpenGL tensor buffer").arg(description), error))
            {
                glDeleteBuffers(1, &glBuffer);
                return false;
            }
            m_glBufferIds.push_back(glBuffer);
            return true;
        }

        return checkStatus(LiteRtCreateManagedTensorBufferFromRequirements(
                m_environment, &info.m_type, requirements, &buffer),
            QStringLiteral("allocate the %1 tensor buffer as an accelerator buffer").arg(description),
            error);
    }

    bool createCompiledModel(const QString& modelPath, bool useGpu, QString& error)
    {
        reset();
        m_modelPath = modelPath;
        m_requestedGpu = useGpu;

        LiteRtEnvOption environmentOptions[2] {};
        int environmentOptionCount = 0;
        if (useGpu)
        {
            if (!createEglContext(error)) {
                return false;
            }
            environmentOptions[0].tag = kLiteRtEnvOptionTagEglDisplay;
            environmentOptions[0].value.type = kLiteRtAnyTypeVoidPtr;
            environmentOptions[0].value.ptr_value = m_eglDisplay;
            environmentOptions[1].tag = kLiteRtEnvOptionTagEglContext;
            environmentOptions[1].value.type = kLiteRtAnyTypeVoidPtr;
            environmentOptions[1].value.ptr_value = m_eglContext;
            environmentOptionCount = 2;
        }

        if (!checkStatus(LiteRtCreateEnvironment(
                environmentOptionCount,
                environmentOptionCount > 0 ? environmentOptions : nullptr,
                &m_environment),
                QStringLiteral("create the CompiledModel environment"), error)) {
            return false;
        }
        if (!checkStatus(LiteRtCreateModelFromFile(
                m_environment, modelPath.toUtf8().constData(), &m_model),
                QStringLiteral("load model %1").arg(modelPath), error)) {
            return false;
        }

        LiteRtParamIndex signatureCount = 0;
        if (!checkStatus(LiteRtGetNumModelSignatures(m_model, &signatureCount),
                QStringLiteral("query model signatures"), error)
            || signatureCount == 0)
        {
            if (error.isEmpty()) {
                error = QStringLiteral("YOLO LiteRT model has no callable signature");
            }
            return false;
        }
        LiteRtSignature signature = nullptr;
        if (!checkStatus(LiteRtGetModelSignature(m_model, 0, &signature),
                QStringLiteral("query the default model signature"), error)) {
            return false;
        }

        LiteRtParamIndex inputCount = 0;
        if (!checkStatus(LiteRtGetNumSignatureInputs(signature, &inputCount),
                QStringLiteral("query model inputs"), error)) {
            return false;
        }
        if (inputCount != 1)
        {
            error = QStringLiteral("YOLO LiteRT models must have exactly one input tensor");
            return false;
        }
        LiteRtTensor inputTensor = nullptr;
        if (!checkStatus(LiteRtGetSignatureInputTensorByIndex(signature, 0, &inputTensor),
                QStringLiteral("query the model input tensor"), error)
            || !readTensorInfo(inputTensor, m_inputInfo, error)) {
            return false;
        }
        if ((m_inputInfo.m_type.layout.rank != 4)
            || (m_inputInfo.m_type.layout.dimensions[0] != 1)
            || (m_inputInfo.m_type.layout.dimensions[3] != 3))
        {
            error = QStringLiteral("YOLO LiteRT input must have NHWC shape [1, height, width, 3]");
            return false;
        }
        m_inputSize = cv::Size(
            m_inputInfo.m_type.layout.dimensions[2],
            m_inputInfo.m_type.layout.dimensions[1]);

        LiteRtParamIndex outputCount = 0;
        if (!checkStatus(LiteRtGetNumSignatureOutputs(signature, &outputCount),
                QStringLiteral("query model outputs"), error)
            || outputCount == 0)
        {
            if (error.isEmpty()) {
                error = QStringLiteral("YOLO LiteRT model has no output tensors");
            }
            return false;
        }
        m_outputInfo.resize(outputCount);
        for (LiteRtParamIndex outputIndex = 0; outputIndex < outputCount; ++outputIndex)
        {
            LiteRtTensor outputTensor = nullptr;
            if (!checkStatus(LiteRtGetSignatureOutputTensorByIndex(signature, outputIndex, &outputTensor),
                    QStringLiteral("query output tensor %1").arg(outputIndex), error)
                || !readTensorInfo(outputTensor, m_outputInfo[outputIndex], error)) {
                return false;
            }
        }

        if (!checkStatus(LiteRtCreateOptions(&m_options), QStringLiteral("create compilation options"), error)
            || !checkStatus(LiteRtSetOptionsHardwareAccelerators(
                    m_options, useGpu ? kLiteRtHwAcceleratorGpu : kLiteRtHwAcceleratorCpu),
                QStringLiteral("select the %1 accelerator").arg(useGpu ? QStringLiteral("GPU") : QStringLiteral("CPU")),
                error)
            || !checkStatus(LiteRtCreateCompiledModel(m_environment, m_model, m_options, &m_compiledModel),
                QStringLiteral("compile the model for %1").arg(useGpu ? QStringLiteral("GPU") : QStringLiteral("CPU")),
                error)) {
            return false;
        }

        LiteRtTensorBufferRequirements inputRequirements = nullptr;
        LiteRtTensorBuffer inputBuffer = nullptr;
        if (!checkStatus(LiteRtGetCompiledModelInputBufferRequirements(
                m_compiledModel, 0, 0, &inputRequirements),
                QStringLiteral("query input buffer requirements"), error)
            || !createBuffer(QStringLiteral("input"), m_inputInfo,
                inputRequirements, inputBuffer, error)) {
            return false;
        }
        m_inputBuffers.push_back(inputBuffer);

        m_outputBuffers.reserve(outputCount);
        for (LiteRtParamIndex outputIndex = 0; outputIndex < outputCount; ++outputIndex)
        {
            LiteRtTensorBufferRequirements outputRequirements = nullptr;
            LiteRtTensorBuffer outputBuffer = nullptr;
            if (!checkStatus(LiteRtGetCompiledModelOutputBufferRequirements(
                    m_compiledModel, 0, outputIndex, &outputRequirements),
                    QStringLiteral("query output buffer %1 requirements").arg(outputIndex), error)
                || !createBuffer(QStringLiteral("output %1").arg(outputIndex),
                    m_outputInfo[outputIndex], outputRequirements, outputBuffer, error)) {
                return false;
            }
            m_outputBuffers.push_back(outputBuffer);
        }

        bool fullyAccelerated = false;
        const LiteRtStatus accelerationStatus =
            LiteRtCompiledModelIsFullyAccelerated(m_compiledModel, &fullyAccelerated);
        m_gpuActive = useGpu;

        QStringList outputDescriptions;
        for (size_t outputIndex = 0; outputIndex < m_outputInfo.size(); ++outputIndex)
        {
            outputDescriptions.append(QStringLiteral("%1:[%2] type=%3")
                .arg(outputIndex)
                .arg(tensorShape(m_outputInfo[outputIndex]))
                .arg(static_cast<int>(m_outputInfo[outputIndex].m_type.element_type)));
        }
        qDebug() << "CameraYoloLiteRt: loaded CompiledModel" << modelPath
                 << "accelerator" << (useGpu ? "GPU" : "CPU")
                 << "fullyAccelerated" << (accelerationStatus == kLiteRtStatusOk && fullyAccelerated)
                 << "input" << tensorShape(m_inputInfo)
                 << "type" << static_cast<int>(m_inputInfo.m_type.element_type)
                 << "outputs" << outputDescriptions.join(QStringLiteral(", "));
        return true;
    }

    bool ensureLoaded(const QString& modelPath, bool useGpu, QString& error)
    {
        if ((m_modelPath == modelPath) && (m_requestedGpu == useGpu) && m_compiledModel) {
            return true;
        }
        if (createCompiledModel(modelPath, useGpu, error)) {
            return true;
        }

        if (useGpu)
        {
            const QString gpuError = error;
            qWarning() << "CameraYoloLiteRt: modern GPU CompiledModel failed; retrying CPU:" << gpuError;
            if (createCompiledModel(modelPath, false, error))
            {
                m_requestedGpu = true;
                m_gpuFallbackReason = gpuError;
                error = QStringLiteral("%1; using LiteRT CPU").arg(gpuError);
                return true;
            }
        }

        reset();
        return false;
    }

    static bool lockBuffer(
        LiteRtTensorBuffer buffer,
        LiteRtTensorBufferLockMode mode,
        void*& address,
        size_t& packedSize,
        QString& error)
    {
        if (!checkStatus(LiteRtGetTensorBufferPackedSize(buffer, &packedSize),
                QStringLiteral("query tensor buffer size"), error)
            || !checkStatus(LiteRtLockTensorBuffer(buffer, &address, mode),
                QStringLiteral("map tensor buffer"), error)) {
            return false;
        }
        return true;
    }

    bool copyInput(const cv::Mat& letterbox, QString& error)
    {
        if (letterbox.type() != CV_8UC3 || letterbox.size() != m_inputSize)
        {
            error = QStringLiteral("LiteRT input image does not match the model input");
            return false;
        }

        void *address = nullptr;
        size_t packedSize = 0;
        if (!lockBuffer(m_inputBuffers[0], kLiteRtTensorBufferLockModeWrite,
                address, packedSize, error)) {
            return false;
        }

        const size_t elements = m_inputInfo.m_elements;
        const LiteRtElementType elementType = m_inputInfo.m_type.element_type;
        const size_t requiredBytes = elements * (elementType == kLiteRtElementTypeFloat32 ? sizeof(float) : 1U);
        bool success = packedSize >= requiredBytes;
        if (!success) {
            error = QStringLiteral("LiteRT input buffer is smaller than the model input");
        }
        else if (elementType == kLiteRtElementTypeFloat32)
        {
            float *destination = static_cast<float *>(address);
            size_t index = 0;
            for (int y = 0; y < letterbox.rows; ++y)
            {
                const cv::Vec3b *source = letterbox.ptr<cv::Vec3b>(y);
                for (int x = 0; x < letterbox.cols; ++x)
                {
                    destination[index++] = source[x][2] / 255.0f;
                    destination[index++] = source[x][1] / 255.0f;
                    destination[index++] = source[x][0] / 255.0f;
                }
            }
        }
        else if ((elementType == kLiteRtElementTypeUInt8) || (elementType == kLiteRtElementTypeInt8))
        {
            if (!m_inputInfo.m_hasPerTensorQuantization || !(m_inputInfo.m_quantization.scale > 0.0f))
            {
                error = QStringLiteral("Quantized LiteRT input has no valid per-tensor scale");
                success = false;
            }
            else
            {
                uint8_t *destination = static_cast<uint8_t *>(address);
                size_t index = 0;
                const int minimum = elementType == kLiteRtElementTypeUInt8 ? 0 : -128;
                const int maximum = elementType == kLiteRtElementTypeUInt8 ? 255 : 127;
                for (int y = 0; y < letterbox.rows; ++y)
                {
                    const cv::Vec3b *source = letterbox.ptr<cv::Vec3b>(y);
                    for (int x = 0; x < letterbox.cols; ++x)
                    {
                        for (int channel : {2, 1, 0})
                        {
                            const float realValue = source[x][channel] / 255.0f;
                            const int quantized = std::clamp(
                                static_cast<int>(std::lround(realValue / m_inputInfo.m_quantization.scale))
                                    + static_cast<int>(m_inputInfo.m_quantization.zero_point),
                                minimum, maximum);
                            destination[index++] = static_cast<uint8_t>(quantized & 0xff);
                        }
                    }
                }
            }
        }
        else
        {
            error = QStringLiteral("LiteRT input datatype %1 is unsupported")
                .arg(static_cast<int>(elementType));
            success = false;
        }

        const LiteRtStatus unlockStatus = LiteRtUnlockTensorBuffer(m_inputBuffers[0]);
        if (success && unlockStatus != kLiteRtStatusOk)
        {
            error = statusError(QStringLiteral("unmap the input tensor buffer"), unlockStatus);
            success = false;
        }
        return success;
    }

    bool copyOutputs(std::vector<cv::Mat>& outputs, QString& error)
    {
        outputs.clear();
        outputs.reserve(m_outputBuffers.size());
        for (size_t outputIndex = 0; outputIndex < m_outputBuffers.size(); ++outputIndex)
        {
            const TensorInfo& info = m_outputInfo[outputIndex];
            std::vector<int> sizes(info.m_type.layout.rank);
            for (unsigned int dimension = 0; dimension < info.m_type.layout.rank; ++dimension) {
                sizes[dimension] = info.m_type.layout.dimensions[dimension];
            }
            cv::Mat output(static_cast<int>(sizes.size()), sizes.data(), CV_32F);

            void *address = nullptr;
            size_t packedSize = 0;
            if (!lockBuffer(m_outputBuffers[outputIndex], kLiteRtTensorBufferLockModeRead,
                    address, packedSize, error)) {
                return false;
            }

            const LiteRtElementType elementType = info.m_type.element_type;
            const size_t requiredBytes = info.m_elements
                * (elementType == kLiteRtElementTypeFloat32 ? sizeof(float) : 1U);
            bool success = packedSize >= requiredBytes;
            if (!success) {
                error = QStringLiteral("LiteRT output tensor %1 buffer is too small").arg(outputIndex);
            }
            else if (elementType == kLiteRtElementTypeFloat32) {
                std::copy_n(static_cast<const float *>(address), info.m_elements, output.ptr<float>());
            }
            else if ((elementType == kLiteRtElementTypeUInt8) || (elementType == kLiteRtElementTypeInt8))
            {
                if (!info.m_hasPerTensorQuantization)
                {
                    error = QStringLiteral("Quantized LiteRT output tensor %1 has no per-tensor scale")
                        .arg(outputIndex);
                    success = false;
                }
                else
                {
                    const uint8_t *source = static_cast<const uint8_t *>(address);
                    float *destination = output.ptr<float>();
                    for (size_t element = 0; element < info.m_elements; ++element)
                    {
                        const int value = elementType == kLiteRtElementTypeUInt8
                            ? static_cast<int>(source[element])
                            : static_cast<int>(static_cast<int8_t>(source[element]));
                        destination[element] =
                            (value - info.m_quantization.zero_point) * info.m_quantization.scale;
                    }
                }
            }
            else
            {
                error = QStringLiteral("LiteRT output datatype %1 is unsupported")
                    .arg(static_cast<int>(elementType));
                success = false;
            }

            const LiteRtStatus unlockStatus = LiteRtUnlockTensorBuffer(m_outputBuffers[outputIndex]);
            if (success && unlockStatus != kLiteRtStatusOk)
            {
                error = statusError(QStringLiteral("unmap output tensor %1").arg(outputIndex), unlockStatus);
                success = false;
            }
            if (!success) {
                return false;
            }
            outputs.push_back(std::move(output));
        }

        if (!m_loggedOutputRanges)
        {
            QStringList outputRanges;
            for (size_t outputIndex = 0; outputIndex < outputs.size(); ++outputIndex)
            {
                const cv::Mat flattened = outputs[outputIndex].reshape(1, 1);
                double minimum = 0.0;
                double maximum = 0.0;
                cv::minMaxLoc(flattened, &minimum, &maximum);
                outputRanges.append(QStringLiteral("%1:%2..%3")
                    .arg(outputIndex)
                    .arg(minimum, 0, 'g', 6)
                    .arg(maximum, 0, 'g', 6));
            }
            qDebug() << "CameraYoloLiteRt: first inference output ranges"
                     << outputRanges.join(QStringLiteral(", "));
            m_loggedOutputRanges = true;
        }
        return !outputs.empty();
    }

    bool infer(const cv::Mat& letterbox, std::vector<cv::Mat>& outputs, QString& error)
    {
        if (!m_compiledModel)
        {
            error = QStringLiteral("LiteRT CompiledModel is not loaded");
            return false;
        }
        if (m_gpuActive && !makeEglCurrent(error)) {
            return false;
        }
        if (!copyInput(letterbox, error)) {
            return false;
        }
        const LiteRtStatus runStatus = LiteRtRunCompiledModel(
            m_compiledModel, 0,
            m_inputBuffers.size(), m_inputBuffers.data(),
            m_outputBuffers.size(), m_outputBuffers.data());
        if (!checkStatus(runStatus, QStringLiteral("run the CompiledModel"), error)) {
            return false;
        }
        return copyOutputs(outputs, error);
    }
#endif
};

CameraYoloLiteRt::CameraYoloLiteRt() : m_impl(new Impl) {}
CameraYoloLiteRt::~CameraYoloLiteRt() = default;
void CameraYoloLiteRt::reset() { m_impl->reset(); }

bool CameraYoloLiteRt::ensureLoaded(const QString& modelPath, bool useGpu, QString& error)
{
#ifdef CAMERA_LITERT_YOLO
    return m_impl->ensureLoaded(modelPath, useGpu, error);
#else
    Q_UNUSED(modelPath)
    Q_UNUSED(useGpu)
    error = QStringLiteral("LiteRT support is not available in this build");
    return false;
#endif
}

bool CameraYoloLiteRt::infer(const cv::Mat& letterbox, std::vector<cv::Mat>& outputs, QString& error)
{
#ifdef CAMERA_LITERT_YOLO
    return m_impl->infer(letterbox, outputs, error);
#else
    Q_UNUSED(letterbox)
    Q_UNUSED(outputs)
    error = QStringLiteral("LiteRT support is not available in this build");
    return false;
#endif
}

cv::Size CameraYoloLiteRt::inputSize() const { return m_impl->m_inputSize; }
bool CameraYoloLiteRt::gpuActive() const { return m_impl->m_gpuActive; }
QString CameraYoloLiteRt::gpuFallbackReason() const { return m_impl->m_gpuFallbackReason; }
