///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                  //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#include "camerayolotensorrt.h"

#ifdef CAMERA_TENSORRT_YOLO

#include <algorithm>
#include <cstring>
#include <limits>
#include <numeric>
#include <utility>

#include <QCryptographicHash>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <opencv2/dnn/dnn.hpp>

namespace
{

class TensorRtLogger : public nvinfer1::ILogger
{
public:
    void log(Severity severity, const char *message) noexcept override
    {
        if (severity <= Severity::kWARNING) {
            qWarning() << "CameraYoloTensorRt:" << message;
        }
    }
};

TensorRtLogger s_logger;

size_t dataTypeSize(nvinfer1::DataType type)
{
    switch (type)
    {
    case nvinfer1::DataType::kFLOAT:
        return sizeof(float);
    case nvinfer1::DataType::kHALF:
        return 2;
    case nvinfer1::DataType::kINT8:
    case nvinfer1::DataType::kBOOL:
    case nvinfer1::DataType::kUINT8:
        return 1;
    case nvinfer1::DataType::kINT32:
        return sizeof(qint32);
    case nvinfer1::DataType::kINT64:
        return sizeof(qint64);
    case nvinfer1::DataType::kFP8:
    case nvinfer1::DataType::kBF16:
    case nvinfer1::DataType::kINT4:
    case nvinfer1::DataType::kFP4:
        return 0;
    }

    return 0;
}

QString dataTypeName(nvinfer1::DataType type)
{
    switch (type)
    {
    case nvinfer1::DataType::kFLOAT:
        return QStringLiteral("float32");
    case nvinfer1::DataType::kHALF:
        return QStringLiteral("float16");
    case nvinfer1::DataType::kINT8:
        return QStringLiteral("int8");
    case nvinfer1::DataType::kINT32:
        return QStringLiteral("int32");
    case nvinfer1::DataType::kBOOL:
        return QStringLiteral("bool");
    case nvinfer1::DataType::kUINT8:
        return QStringLiteral("uint8");
    case nvinfer1::DataType::kFP8:
        return QStringLiteral("float8");
    case nvinfer1::DataType::kBF16:
        return QStringLiteral("bfloat16");
    case nvinfer1::DataType::kINT64:
        return QStringLiteral("int64");
    case nvinfer1::DataType::kINT4:
        return QStringLiteral("int4");
    case nvinfer1::DataType::kFP4:
        return QStringLiteral("float4");
    }

    return QStringLiteral("unknown");
}

qint64 dimsVolume(const nvinfer1::Dims& dims)
{
    qint64 volume = 1;
    for (int i = 0; i < dims.nbDims; ++i)
    {
        if (dims.d[i] <= 0) {
            return 0;
        }
        volume *= dims.d[i];
    }
    return volume;
}

bool hasDynamicDim(const nvinfer1::Dims& dims)
{
    for (int i = 0; i < dims.nbDims; ++i)
    {
        if (dims.d[i] < 0) {
            return true;
        }
    }
    return false;
}

nvinfer1::Dims makeInputDims(int batch, const cv::Size& inputSize, const nvinfer1::Dims& modelDims)
{
    nvinfer1::Dims dims = modelDims;
    if (dims.nbDims != 4)
    {
        dims.nbDims = 4;
        dims.d[1] = 3;
    }

    dims.d[0] = batch;
    dims.d[1] = dims.d[1] > 0 ? dims.d[1] : 3;
    dims.d[2] = inputSize.height;
    dims.d[3] = inputSize.width;
    return dims;
}

QString makeEnginePath(const QString& modelPath, const cv::Size& inputSize, int maxBatch, bool fp16)
{
    QFileInfo modelInfo(modelPath);
    QCryptographicHash hash(QCryptographicHash::Sha1);
    hash.addData(QDir::toNativeSeparators(modelInfo.absoluteFilePath()).toUtf8());
    hash.addData(QByteArray::number(modelInfo.size()));
    hash.addData(QByteArray::number(modelInfo.lastModified().toMSecsSinceEpoch()));
    hash.addData(QByteArray::number(NV_TENSORRT_MAJOR));
    hash.addData(QByteArray::number(NV_TENSORRT_MINOR));
    hash.addData(QByteArray::number(NV_TENSORRT_PATCH));
    hash.addData(QByteArray::number(inputSize.width));
    hash.addData(QByteArray::number(inputSize.height));
    hash.addData(QByteArray::number(maxBatch));
    hash.addData(fp16 ? "fp16" : "fp32");

    const QString suffix = QString::fromLatin1(hash.result().toHex().left(12));
    return modelInfo.absoluteDir().absoluteFilePath(QStringLiteral("%1.trt%2-%3x%4-b%5-%6-%7.engine")
        .arg(modelInfo.completeBaseName())
        .arg(NV_TENSORRT_MAJOR)
        .arg(inputSize.width)
        .arg(inputSize.height)
        .arg(maxBatch)
        .arg(fp16 ? QStringLiteral("fp16") : QStringLiteral("fp32"))
        .arg(suffix));
}

bool readFile(const QString& path, QByteArray& data, QString& error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        error = QStringLiteral("Cannot open TensorRT engine %1: %2").arg(path, file.errorString());
        return false;
    }

    data = file.readAll();
    return !data.isEmpty();
}

bool writeFile(const QString& path, const void *data, qint64 size, QString& error)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly))
    {
        error = QStringLiteral("Cannot write TensorRT engine %1: %2").arg(path, file.errorString());
        return false;
    }

    if (file.write(static_cast<const char *>(data), size) != size)
    {
        error = QStringLiteral("Failed to write complete TensorRT engine %1").arg(path);
        return false;
    }

    return true;
}

QString parserErrors(nvonnxparser::IParser& parser)
{
    QStringList errors;
    const int count = parser.getNbErrors();
    for (int i = 0; i < count; ++i)
    {
        const nvonnxparser::IParserError *parserError = parser.getError(i);
        if (parserError) {
            errors.append(QString::fromUtf8(parserError->desc()));
        }
    }
    return errors.join(QStringLiteral("; "));
}

}

struct CameraYoloTensorRt::Impl
{
    QString m_modelPath;
    QString m_enginePath;
    cv::Size m_inputSize = cv::Size(0, 0);
    int m_requestedMaxBatch = 1;
    int m_maxBatch = 1;
    bool m_fixedInputBatch = false;
    bool m_fp16 = false;
    nvinfer1::Dims m_modelInputDims {};
    std::unique_ptr<nvinfer1::IRuntime> m_runtime;
    std::unique_ptr<nvinfer1::ICudaEngine> m_engine;
    std::unique_ptr<nvinfer1::IExecutionContext> m_context;
    std::string m_inputName;
    std::string m_outputName;
    cudaStream_t m_stream = nullptr;
    void *m_inputDevice = nullptr;
    size_t m_inputDeviceBytes = 0;
    void *m_outputDevice = nullptr;
    size_t m_outputDeviceBytes = 0;
    CameraYoloTensorRt::ProgressCallback m_progressCallback;

    ~Impl()
    {
        releaseDeviceBuffers();
        if (m_stream) {
            cudaStreamDestroy(m_stream);
        }
    }

    void releaseDeviceBuffers()
    {
        if (m_stream) {
            cudaStreamSynchronize(m_stream);
        }
        if (m_inputDevice) {
            cudaFree(m_inputDevice);
            m_inputDevice = nullptr;
            m_inputDeviceBytes = 0;
        }
        if (m_outputDevice) {
            cudaFree(m_outputDevice);
            m_outputDevice = nullptr;
            m_outputDeviceBytes = 0;
        }
    }

    bool ensureDeviceBuffer(void **buffer, size_t& bufferBytes, size_t requiredBytes, const QString& name, QString& error)
    {
        if (requiredBytes == 0)
        {
            error = QStringLiteral("TensorRT %1 allocation requested zero bytes").arg(name);
            return false;
        }

        if (*buffer && (bufferBytes >= requiredBytes)) {
            return true;
        }

        void *newBuffer = nullptr;
        const cudaError_t cudaError = cudaMalloc(&newBuffer, requiredBytes);
        if (cudaError != cudaSuccess)
        {
            error = QStringLiteral("TensorRT %1 allocation failed: %2")
                .arg(name, QString::fromUtf8(cudaGetErrorString(cudaError)));
            return false;
        }

        if (*buffer)
        {
            if (m_stream) {
                cudaStreamSynchronize(m_stream);
            }
            cudaFree(*buffer);
        }
        *buffer = newBuffer;
        bufferBytes = requiredBytes;
        return true;
    }

    void reset()
    {
        releaseDeviceBuffers();
        m_context.reset();
        m_engine.reset();
        m_runtime.reset();
        m_modelPath.clear();
        m_enginePath.clear();
        m_inputSize = cv::Size(0, 0);
        m_requestedMaxBatch = 1;
        m_maxBatch = 1;
        m_fixedInputBatch = false;
        m_fp16 = false;
        m_modelInputDims = nvinfer1::Dims {};
        m_inputName.clear();
        m_outputName.clear();
    }

    bool loadSerializedEngine(const QByteArray& engineBytes, QString& error)
    {
        m_runtime.reset(nvinfer1::createInferRuntime(s_logger));
        if (!m_runtime)
        {
            error = QStringLiteral("Failed to create TensorRT runtime");
            return false;
        }

        m_engine.reset(m_runtime->deserializeCudaEngine(engineBytes.constData(), static_cast<size_t>(engineBytes.size())));
        if (!m_engine)
        {
            error = QStringLiteral("Failed to deserialize TensorRT engine");
            return false;
        }

        m_context.reset(m_engine->createExecutionContext());
        if (!m_context)
        {
            error = QStringLiteral("Failed to create TensorRT execution context");
            return false;
        }

        const int tensorCount = m_engine->getNbIOTensors();
        for (int i = 0; i < tensorCount; ++i)
        {
            const char *name = m_engine->getIOTensorName(i);
            if (!name) {
                continue;
            }

            const nvinfer1::TensorIOMode mode = m_engine->getTensorIOMode(name);
            if ((mode == nvinfer1::TensorIOMode::kINPUT) && m_inputName.empty()) {
                m_inputName = name;
            } else if ((mode == nvinfer1::TensorIOMode::kOUTPUT) && m_outputName.empty()) {
                m_outputName = name;
            }
        }

        if (m_inputName.empty() || m_outputName.empty())
        {
            error = QStringLiteral("TensorRT engine does not expose a single usable input/output tensor");
            return false;
        }

        m_modelInputDims = m_engine->getTensorShape(m_inputName.c_str());
        m_maxBatch = std::max(1, m_requestedMaxBatch);
        m_fixedInputBatch = (m_modelInputDims.nbDims == 4) && (m_modelInputDims.d[0] > 0);
        if (m_fixedInputBatch) {
            m_maxBatch = m_modelInputDims.d[0];
        }
        if (!m_stream && (cudaStreamCreate(&m_stream) != cudaSuccess))
        {
            error = QStringLiteral("Failed to create CUDA stream for TensorRT inference");
            return false;
        }

        return true;
    }

    bool buildEngine(const QString& modelPath, const cv::Size& inputSize, int maxBatch, bool fp16, QString& error)
    {
        std::unique_ptr<nvinfer1::IBuilder> builder(nvinfer1::createInferBuilder(s_logger));
        if (!builder)
        {
            error = QStringLiteral("Failed to create TensorRT builder");
            return false;
        }

        const uint32_t networkFlags = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        std::unique_ptr<nvinfer1::INetworkDefinition> network(builder->createNetworkV2(networkFlags));
        if (!network)
        {
            error = QStringLiteral("Failed to create TensorRT network");
            return false;
        }

        std::unique_ptr<nvonnxparser::IParser> parser(nvonnxparser::createParser(*network, s_logger));
        if (!parser)
        {
            error = QStringLiteral("Failed to create TensorRT ONNX parser");
            return false;
        }

        const QByteArray localPath = QFile::encodeName(modelPath);
        if (!parser->parseFromFile(localPath.constData(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)))
        {
            error = QStringLiteral("TensorRT ONNX parse failed: %1").arg(parserErrors(*parser));
            return false;
        }

        std::unique_ptr<nvinfer1::IBuilderConfig> config(builder->createBuilderConfig());
        if (!config)
        {
            error = QStringLiteral("Failed to create TensorRT builder config");
            return false;
        }

        config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1024ULL * 1024ULL * 1024ULL);
        if (fp16 && builder->platformHasFastFp16()) {
            config->setFlag(nvinfer1::BuilderFlag::kFP16);
        }

        nvinfer1::ITensor *inputTensor = network->getNbInputs() > 0 ? network->getInput(0) : nullptr;
        if (!inputTensor)
        {
            error = QStringLiteral("TensorRT network has no input tensor");
            return false;
        }

        const nvinfer1::Dims inputDims = inputTensor->getDimensions();
        if ((inputDims.nbDims != 4) || ((inputDims.d[1] > 0) && (inputDims.d[1] != 3)))
        {
            error = QStringLiteral("Unsupported YOLO ONNX input dimensions for TensorRT");
            return false;
        }

        if (hasDynamicDim(inputDims))
        {
            nvinfer1::IOptimizationProfile *profile = builder->createOptimizationProfile();
            if (!profile)
            {
                error = QStringLiteral("Failed to create TensorRT optimization profile");
                return false;
            }

            const int optBatch = std::max(1, std::min(maxBatch, 4));
            const char *inputName = inputTensor->getName();
            profile->setDimensions(inputName, nvinfer1::OptProfileSelector::kMIN, makeInputDims(1, inputSize, inputDims));
            profile->setDimensions(inputName, nvinfer1::OptProfileSelector::kOPT, makeInputDims(optBatch, inputSize, inputDims));
            profile->setDimensions(inputName, nvinfer1::OptProfileSelector::kMAX, makeInputDims(maxBatch, inputSize, inputDims));
            config->addOptimizationProfile(profile);
        }

        std::unique_ptr<nvinfer1::IHostMemory> serialized(builder->buildSerializedNetwork(*network, *config));
        if (!serialized)
        {
            error = QStringLiteral("TensorRT engine build failed");
            return false;
        }

        if (!writeFile(m_enginePath, serialized->data(), static_cast<qint64>(serialized->size()), error)) {
            qWarning() << "CameraYoloTensorRt:" << error;
        }

        const QByteArray engineBytes(static_cast<const char *>(serialized->data()), static_cast<int>(serialized->size()));
        return loadSerializedEngine(engineBytes, error);
    }

    bool ensureLoaded(const QString& modelPath, const cv::Size& inputSize, int maxBatch, bool fp16, QString& error)
    {
        const int boundedBatch = std::max(1, std::min(maxBatch, 32));
        const QString enginePath = makeEnginePath(modelPath, inputSize, boundedBatch, fp16);
        if (m_context && (m_modelPath == modelPath) && (m_enginePath == enginePath)
            && (m_inputSize == inputSize) && (m_requestedMaxBatch >= boundedBatch) && (m_fp16 == fp16))
        {
            return true;
        }

        reset();
        m_modelPath = modelPath;
        m_enginePath = enginePath;
        m_inputSize = inputSize;
        m_requestedMaxBatch = boundedBatch;
        m_maxBatch = boundedBatch;
        m_fp16 = fp16;

        QByteArray engineBytes;
        if (QFileInfo::exists(enginePath) && readFile(enginePath, engineBytes, error))
        {
            if (loadSerializedEngine(engineBytes, error))
            {
                qDebug() << "CameraYoloTensorRt: loaded cached engine" << enginePath;
                return true;
            }

            qWarning() << "CameraYoloTensorRt: cached engine unusable, rebuilding:" << error;
        }

        qDebug() << "CameraYoloTensorRt: building engine" << enginePath;
        if (m_progressCallback) {
            m_progressCallback(true, modelPath, enginePath);
        }

        const bool built = buildEngine(modelPath, inputSize, boundedBatch, fp16, error);

        if (m_progressCallback) {
            m_progressCallback(false, modelPath, enginePath);
        }

        return built;
    }

    bool inferChunk(const std::vector<cv::Mat>& letterboxes, int firstTile, int tileCount, std::vector<cv::Mat>& outputs, QString& error)
    {
        if (!m_context || m_inputName.empty() || m_outputName.empty())
        {
            error = QStringLiteral("TensorRT engine is not loaded");
            return false;
        }

        const int inferenceBatch = m_fixedInputBatch ? m_maxBatch : tileCount;
        const nvinfer1::Dims inputDims = makeInputDims(inferenceBatch, m_inputSize, m_modelInputDims);
        if (!m_context->setInputShape(m_inputName.c_str(), inputDims))
        {
            error = QStringLiteral("TensorRT failed to set input shape");
            return false;
        }

        cv::Mat blob;
        std::vector<cv::Mat> chunk;
        chunk.reserve(static_cast<size_t>(inferenceBatch));
        for (int i = 0; i < inferenceBatch; ++i)
        {
            const int tileOffset = std::min(i, tileCount - 1);
            chunk.push_back(letterboxes[static_cast<size_t>(firstTile + tileOffset)]);
        }
        cv::dnn::blobFromImages(chunk, blob, 1.0 / 255.0, m_inputSize, cv::Scalar(), true, false);
        if (!blob.isContinuous()) {
            blob = blob.clone();
        }

        const nvinfer1::DataType inputDataType = m_engine->getTensorDataType(m_inputName.c_str());
        const size_t inputElementSize = dataTypeSize(inputDataType);
        const qint64 inputElements = dimsVolume(inputDims);
        if ((inputElements <= 0) || (static_cast<size_t>(inputElements) != blob.total()) || (inputElementSize == 0))
        {
            error = QStringLiteral("TensorRT input tensor is unsupported");
            return false;
        }

        std::vector<quint16> inputHalfHost;
        const void *inputHostData = blob.ptr<float>();
        if (inputDataType == nvinfer1::DataType::kHALF)
        {
            const float *floatData = blob.ptr<float>();
            inputHalfHost.resize(static_cast<size_t>(inputElements));
            for (size_t i = 0; i < inputHalfHost.size(); ++i)
            {
                const __half halfValue = __float2half(floatData[i]);
                std::memcpy(&inputHalfHost[i], &halfValue, sizeof(inputHalfHost[i]));
            }
            inputHostData = inputHalfHost.data();
        }
        else if (inputDataType != nvinfer1::DataType::kFLOAT)
        {
            error = QStringLiteral("TensorRT input tensor datatype %1 is unsupported").arg(dataTypeName(inputDataType));
            return false;
        }

        const size_t inputBytes = static_cast<size_t>(inputElements) * inputElementSize;
        if (!ensureDeviceBuffer(&m_inputDevice, m_inputDeviceBytes, inputBytes, QStringLiteral("input"), error)) {
            return false;
        }

        cudaError_t cudaError = cudaMemcpyAsync(m_inputDevice, inputHostData, inputBytes, cudaMemcpyHostToDevice, m_stream);
        if (cudaError != cudaSuccess)
        {
            error = QStringLiteral("TensorRT input upload failed: %1").arg(QString::fromUtf8(cudaGetErrorString(cudaError)));
            return false;
        }

        if (!m_context->setTensorAddress(m_inputName.c_str(), m_inputDevice))
        {
            error = QStringLiteral("TensorRT failed to bind input tensor");
            return false;
        }

        const char *missingTensorNames[8] = {};
        const int missingShapes = m_context->inferShapes(8, missingTensorNames);
        if (missingShapes != 0)
        {
            if (missingShapes < 0)
            {
                error = QStringLiteral("TensorRT shape inference failed");
            }
            else if ((missingShapes > 0) && missingTensorNames[0])
            {
                error = QStringLiteral("TensorRT shape inference has unresolved tensor shape: %1")
                    .arg(QString::fromUtf8(missingTensorNames[0]));
            }
            else
            {
                error = QStringLiteral("TensorRT shape inference has %1 unresolved tensor shape(s)").arg(missingShapes);
            }
            return false;
        }

        const nvinfer1::Dims outputDims = m_context->getTensorShape(m_outputName.c_str());
        const qint64 outputElements = dimsVolume(outputDims);
        const size_t outputElementSize = dataTypeSize(m_engine->getTensorDataType(m_outputName.c_str()));
        if ((outputElements <= 0) || (outputElementSize != sizeof(float)))
        {
            error = QStringLiteral("TensorRT output tensor is unsupported");
            return false;
        }

        const size_t outputBytes = static_cast<size_t>(outputElements) * outputElementSize;
        if (!ensureDeviceBuffer(&m_outputDevice, m_outputDeviceBytes, outputBytes, QStringLiteral("output"), error)) {
            return false;
        }

        if (!m_context->setTensorAddress(m_outputName.c_str(), m_outputDevice))
        {
            error = QStringLiteral("TensorRT failed to bind output tensor");
            return false;
        }

        if (!m_context->enqueueV3(m_stream))
        {
            error = QStringLiteral("TensorRT enqueue failed");
            return false;
        }

        std::vector<float> outputHost(static_cast<size_t>(outputElements));
        cudaError = cudaMemcpyAsync(outputHost.data(), m_outputDevice, outputBytes, cudaMemcpyDeviceToHost, m_stream);
        if (cudaError == cudaSuccess) {
            cudaError = cudaStreamSynchronize(m_stream);
        }
        if (cudaError != cudaSuccess)
        {
            error = QStringLiteral("TensorRT output download failed: %1").arg(QString::fromUtf8(cudaGetErrorString(cudaError)));
            return false;
        }

        if (outputDims.nbDims == 3)
        {
            const int batch = outputDims.d[0];
            if (batch != inferenceBatch)
            {
                error = QStringLiteral("TensorRT output batch size mismatch");
                return false;
            }

            const int rows = outputDims.d[1];
            const int cols = outputDims.d[2];
            const qint64 stride = static_cast<qint64>(rows) * cols;
            for (int i = 0; i < tileCount; ++i)
            {
                cv::Mat det(rows, cols, CV_32F, outputHost.data() + static_cast<size_t>(i * stride));
                outputs.push_back(det.clone());
            }
            return true;
        }

        if ((outputDims.nbDims == 2) && (tileCount == 1))
        {
            cv::Mat det(outputDims.d[0], outputDims.d[1], CV_32F, outputHost.data());
            outputs.push_back(det.clone());
            return true;
        }

        error = QStringLiteral("TensorRT output dimensions are unsupported");
        return false;
    }

    bool infer(const std::vector<cv::Mat>& letterboxes, std::vector<cv::Mat>& outputs, QString& error)
    {
        outputs.clear();
        outputs.reserve(letterboxes.size());

        for (int firstTile = 0; firstTile < static_cast<int>(letterboxes.size()); firstTile += m_maxBatch)
        {
            const int tileCount = std::min(m_maxBatch, static_cast<int>(letterboxes.size()) - firstTile);
            if (!inferChunk(letterboxes, firstTile, tileCount, outputs, error)) {
                return false;
            }
        }

        return outputs.size() == letterboxes.size();
    }
};

CameraYoloTensorRt::CameraYoloTensorRt() :
    m_impl(std::make_unique<Impl>())
{
}

CameraYoloTensorRt::~CameraYoloTensorRt() = default;

void CameraYoloTensorRt::reset()
{
    m_impl->reset();
}

void CameraYoloTensorRt::setProgressCallback(ProgressCallback callback)
{
    m_impl->m_progressCallback = std::move(callback);
}

bool CameraYoloTensorRt::ensureLoaded(const QString& modelPath, const cv::Size& inputSize, int maxBatch, bool fp16, QString& error)
{
    return m_impl->ensureLoaded(modelPath, inputSize, maxBatch, fp16, error);
}

bool CameraYoloTensorRt::infer(const std::vector<cv::Mat>& letterboxes, std::vector<cv::Mat>& outputs, QString& error)
{
    return m_impl->infer(letterboxes, outputs, error);
}

int CameraYoloTensorRt::maxBatch() const
{
    return m_impl->m_maxBatch;
}

QString CameraYoloTensorRt::enginePath() const
{
    return m_impl->m_enginePath;
}

#endif // CAMERA_TENSORRT_YOLO
