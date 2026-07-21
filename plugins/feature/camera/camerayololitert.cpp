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
#include <cstring>

#include <QDebug>
#include <QStringList>

#ifdef CAMERA_LITERT_YOLO
#include <tflite/c/c_api.h>
#include <tflite/delegates/gpu/delegate.h>
#endif

struct CameraYoloLiteRt::Impl
{
#ifdef CAMERA_LITERT_YOLO
    struct ModelDeleter { void operator()(TfLiteModel *p) const { TfLiteModelDelete(p); } };
    struct OptionsDeleter { void operator()(TfLiteInterpreterOptions *p) const { TfLiteInterpreterOptionsDelete(p); } };
    struct InterpreterDeleter { void operator()(TfLiteInterpreter *p) const { TfLiteInterpreterDelete(p); } };
    struct DelegateDeleter { void operator()(TfLiteDelegate *p) const { TfLiteGpuDelegateV2Delete(p); } };

    std::unique_ptr<TfLiteModel, ModelDeleter> m_model;
    std::unique_ptr<TfLiteInterpreterOptions, OptionsDeleter> m_options;
    std::unique_ptr<TfLiteInterpreter, InterpreterDeleter> m_interpreter;
    std::unique_ptr<TfLiteDelegate, DelegateDeleter> m_gpuDelegate;
    std::vector<float> m_floatInputBuffer;
    std::vector<quint8> m_quantizedBuffer;
#endif
    QString m_modelPath;
    cv::Size m_inputSize;
    bool m_requestedGpu = false;
    bool m_gpuActive = false;
    bool m_loggedOutputRanges = false;

    void reset()
    {
#ifdef CAMERA_LITERT_YOLO
        // The interpreter must be destroyed before its delegate and model.
        m_interpreter.reset();
        m_gpuDelegate.reset();
        m_options.reset();
        m_model.reset();
        m_floatInputBuffer.clear();
        m_quantizedBuffer.clear();
#endif
        m_modelPath.clear();
        m_inputSize = {};
        m_requestedGpu = false;
        m_gpuActive = false;
        m_loggedOutputRanges = false;
    }

#ifdef CAMERA_LITERT_YOLO
    static QString statusError(const QString& operation)
    {
        return QStringLiteral("LiteRT failed to %1").arg(operation);
    }

    bool createInterpreter(bool useGpu, QString& error)
    {
        m_options.reset(TfLiteInterpreterOptionsCreate());
        if (!m_options)
        {
            error = QStringLiteral("Cannot create LiteRT interpreter options");
            return false;
        }

        if (useGpu)
        {
            TfLiteGpuDelegateOptionsV2 gpuOptions = TfLiteGpuDelegateOptionsV2Default();
            gpuOptions.is_precision_loss_allowed = 1;
            gpuOptions.inference_preference = TFLITE_GPU_INFERENCE_PREFERENCE_SUSTAINED_SPEED;
            m_gpuDelegate.reset(TfLiteGpuDelegateV2Create(&gpuOptions));
            if (!m_gpuDelegate)
            {
                error = QStringLiteral("Cannot create the Android LiteRT GPU delegate");
                return false;
            }
            TfLiteInterpreterOptionsAddDelegate(m_options.get(), m_gpuDelegate.get());
        }

        m_interpreter.reset(TfLiteInterpreterCreate(m_model.get(), m_options.get()));
        if (!m_interpreter)
        {
            error = useGpu
                ? QStringLiteral("Cannot create a LiteRT interpreter with the GPU delegate; the model may contain unsupported GPU operations")
                : QStringLiteral("Cannot create a LiteRT CPU interpreter");
            return false;
        }
        if (TfLiteInterpreterAllocateTensors(m_interpreter.get()) != kTfLiteOk)
        {
            error = statusError(QStringLiteral("allocate tensors"));
            return false;
        }
        if (TfLiteInterpreterGetInputTensorCount(m_interpreter.get()) != 1)
        {
            error = QStringLiteral("YOLO LiteRT models must have exactly one input tensor");
            return false;
        }

        const TfLiteTensor *input = TfLiteInterpreterGetInputTensor(m_interpreter.get(), 0);
        if (!input || (TfLiteTensorNumDims(input) != 4)
            || (TfLiteTensorDim(input, 0) != 1) || (TfLiteTensorDim(input, 3) != 3))
        {
            error = QStringLiteral("YOLO LiteRT input must have NHWC shape [1, height, width, 3]");
            return false;
        }
        m_inputSize = cv::Size(TfLiteTensorDim(input, 2), TfLiteTensorDim(input, 1));
        if ((m_inputSize.width <= 0) || (m_inputSize.height <= 0))
        {
            error = QStringLiteral("YOLO LiteRT input has invalid dimensions");
            return false;
        }
        m_gpuActive = useGpu;

        QStringList inputShape;
        for (int dim = 0; dim < TfLiteTensorNumDims(input); ++dim) {
            inputShape.append(QString::number(TfLiteTensorDim(input, dim)));
        }
        QStringList outputDescriptions;
        const int outputCount = TfLiteInterpreterGetOutputTensorCount(m_interpreter.get());
        for (int outputIndex = 0; outputIndex < outputCount; ++outputIndex)
        {
            const TfLiteTensor *output = TfLiteInterpreterGetOutputTensor(m_interpreter.get(), outputIndex);
            QStringList outputShape;
            for (int dim = 0; output && (dim < TfLiteTensorNumDims(output)); ++dim) {
                outputShape.append(QString::number(TfLiteTensorDim(output, dim)));
            }
            outputDescriptions.append(QStringLiteral("%1:[%2] type=%3")
                .arg(outputIndex)
                .arg(outputShape.join(QLatin1Char('x')))
                .arg(output ? static_cast<int>(TfLiteTensorType(output)) : -1));
        }
        qDebug() << "CameraYoloLiteRt: loaded model" << m_modelPath
                 << "input" << inputShape.join(QLatin1Char('x'))
                 << "type" << static_cast<int>(TfLiteTensorType(input))
                 << "quantization" << TfLiteTensorQuantizationParams(input).scale
                 << TfLiteTensorQuantizationParams(input).zero_point
                 << "outputs" << outputDescriptions.join(QStringLiteral(", "));
        return true;
    }

    bool ensureLoaded(const QString& modelPath, bool useGpu, QString& error)
    {
        if ((m_modelPath == modelPath) && (m_requestedGpu == useGpu) && m_interpreter) {
            return true;
        }

        reset();
        m_model.reset(TfLiteModelCreateFromFile(modelPath.toUtf8().constData()));
        if (!m_model)
        {
            error = QStringLiteral("Cannot load LiteRT model %1").arg(modelPath);
            return false;
        }

        m_modelPath = modelPath;
        m_requestedGpu = useGpu;
        if (createInterpreter(useGpu, error)) {
            return true;
        }

        // Keep .tflite support useful on devices/models not supported by the GPU
        // delegate. This remains LiteRT inference; it never falls through to OpenCV.
        if (useGpu)
        {
            const QString gpuError = error;
            m_interpreter.reset();
            m_gpuDelegate.reset();
            m_options.reset();
            if (createInterpreter(false, error))
            {
                error = QStringLiteral("%1; using LiteRT CPU").arg(gpuError);
                return true;
            }
        }

        reset();
        return false;
    }

    bool copyInput(const cv::Mat& letterbox, QString& error)
    {
        if (letterbox.type() != CV_8UC3 || letterbox.size() != m_inputSize)
        {
            error = QStringLiteral("LiteRT input image does not match the model input");
            return false;
        }
        TfLiteTensor *input = TfLiteInterpreterGetInputTensor(m_interpreter.get(), 0);
        const size_t elements = static_cast<size_t>(m_inputSize.area()) * 3U;
        const TfLiteQuantizationParams quant = TfLiteTensorQuantizationParams(input);

        if (TfLiteTensorType(input) == kTfLiteFloat32)
        {
            m_floatInputBuffer.resize(elements);
            size_t dst = 0;
            for (int y = 0; y < letterbox.rows; ++y)
            {
                const cv::Vec3b *src = letterbox.ptr<cv::Vec3b>(y);
                for (int x = 0; x < letterbox.cols; ++x)
                {
                    m_floatInputBuffer[dst++] = src[x][2] / 255.0f;
                    m_floatInputBuffer[dst++] = src[x][1] / 255.0f;
                    m_floatInputBuffer[dst++] = src[x][0] / 255.0f;
                }
            }
            if (TfLiteTensorCopyFromBuffer(input, m_floatInputBuffer.data(), m_floatInputBuffer.size() * sizeof(float)) != kTfLiteOk)
            {
                error = statusError(QStringLiteral("copy the float input tensor"));
                return false;
            }
            return true;
        }

        if ((TfLiteTensorType(input) == kTfLiteUInt8) || (TfLiteTensorType(input) == kTfLiteInt8))
        {
            if (!(quant.scale > 0.0f))
            {
                error = QStringLiteral("Quantized LiteRT input has no valid scale");
                return false;
            }
            m_quantizedBuffer.resize(elements);
            size_t dst = 0;
            const int minValue = TfLiteTensorType(input) == kTfLiteUInt8 ? 0 : -128;
            const int maxValue = TfLiteTensorType(input) == kTfLiteUInt8 ? 255 : 127;
            for (int y = 0; y < letterbox.rows; ++y)
            {
                const cv::Vec3b *src = letterbox.ptr<cv::Vec3b>(y);
                for (int x = 0; x < letterbox.cols; ++x)
                {
                    for (int channel : {2, 1, 0})
                    {
                        const float realValue = src[x][channel] / 255.0f;
                        const int value = std::clamp(
                            static_cast<int>(std::lround(realValue / quant.scale)) + quant.zero_point,
                            minValue, maxValue);
                        m_quantizedBuffer[dst++] = static_cast<quint8>(value & 0xff);
                    }
                }
            }
            if (TfLiteTensorCopyFromBuffer(input, m_quantizedBuffer.data(), m_quantizedBuffer.size()) != kTfLiteOk)
            {
                error = statusError(QStringLiteral("copy the quantized input tensor"));
                return false;
            }
            return true;
        }

        error = QStringLiteral("LiteRT input datatype %1 is unsupported").arg(TfLiteTensorType(input));
        return false;
    }

    bool copyOutputs(std::vector<cv::Mat>& outputs, QString& error)
    {
        outputs.clear();
        const int outputCount = TfLiteInterpreterGetOutputTensorCount(m_interpreter.get());
        for (int outputIndex = 0; outputIndex < outputCount; ++outputIndex)
        {
            const TfLiteTensor *tensor = TfLiteInterpreterGetOutputTensor(m_interpreter.get(), outputIndex);
            const int dims = tensor ? TfLiteTensorNumDims(tensor) : 0;
            if (!tensor || (dims <= 0))
            {
                error = QStringLiteral("LiteRT output tensor %1 has invalid dimensions").arg(outputIndex);
                return false;
            }
            std::vector<int> sizes(static_cast<size_t>(dims));
            size_t elements = 1;
            for (int dim = 0; dim < dims; ++dim)
            {
                sizes[static_cast<size_t>(dim)] = TfLiteTensorDim(tensor, dim);
                if (sizes[static_cast<size_t>(dim)] <= 0)
                {
                    error = QStringLiteral("LiteRT output tensor %1 has an unresolved dimension").arg(outputIndex);
                    return false;
                }
                elements *= static_cast<size_t>(sizes[static_cast<size_t>(dim)]);
            }

            cv::Mat output(dims, sizes.data(), CV_32F);
            if (TfLiteTensorType(tensor) == kTfLiteFloat32)
            {
                if (TfLiteTensorCopyToBuffer(tensor, output.ptr<float>(), elements * sizeof(float)) != kTfLiteOk)
                {
                    error = statusError(QStringLiteral("copy output tensor %1").arg(outputIndex));
                    return false;
                }
            }
            else if ((TfLiteTensorType(tensor) == kTfLiteUInt8) || (TfLiteTensorType(tensor) == kTfLiteInt8))
            {
                m_quantizedBuffer.resize(elements);
                if (TfLiteTensorCopyToBuffer(tensor, m_quantizedBuffer.data(), m_quantizedBuffer.size()) != kTfLiteOk)
                {
                    error = statusError(QStringLiteral("copy quantized output tensor %1").arg(outputIndex));
                    return false;
                }
                const TfLiteQuantizationParams quant = TfLiteTensorQuantizationParams(tensor);
                float *dst = output.ptr<float>();
                for (size_t i = 0; i < elements; ++i)
                {
                    const int value = TfLiteTensorType(tensor) == kTfLiteUInt8
                        ? static_cast<int>(m_quantizedBuffer[i])
                        : static_cast<int>(static_cast<qint8>(m_quantizedBuffer[i]));
                    dst[i] = (value - quant.zero_point) * quant.scale;
                }
            }
            else
            {
                error = QStringLiteral("LiteRT output datatype %1 is unsupported").arg(TfLiteTensorType(tensor));
                return false;
            }
            outputs.push_back(std::move(output));
        }
        if (!m_loggedOutputRanges)
        {
            QStringList outputRanges;
            for (int outputIndex = 0; outputIndex < static_cast<int>(outputs.size()); ++outputIndex)
            {
                const cv::Mat flattened = outputs[static_cast<size_t>(outputIndex)].reshape(1, 1);
                double minimum = 0.0;
                double maximum = 0.0;
                cv::minMaxLoc(flattened, &minimum, &maximum);
                outputRanges.append(QStringLiteral("%1:%2..%3")
                    .arg(outputIndex)
                    .arg(minimum, 0, 'g', 6)
                    .arg(maximum, 0, 'g', 6));
            }
            qDebug() << "CameraYoloLiteRt: first inference output ranges" << outputRanges.join(QStringLiteral(", "));
            m_loggedOutputRanges = true;
        }
        return !outputs.empty();
    }

    bool infer(const cv::Mat& letterbox, std::vector<cv::Mat>& outputs, QString& error)
    {
        if (!m_interpreter)
        {
            error = QStringLiteral("LiteRT model is not loaded");
            return false;
        }
        if (!copyInput(letterbox, error)) {
            return false;
        }
        if (TfLiteInterpreterInvoke(m_interpreter.get()) != kTfLiteOk)
        {
            error = statusError(QStringLiteral("invoke the model"));
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
