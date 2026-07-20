///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
///////////////////////////////////////////////////////////////////////////////////

#include "camerathermalprocessor.h"

#include <algorithm>
#include <cmath>

#include <QDebug>
#include <QFile>

#include <opencv2/imgproc.hpp>

#include "camera.h"
#include "cameraframealigner.h"

CameraThermalProcessor::CameraThermalProcessor() = default;
CameraThermalProcessor::~CameraThermalProcessor() = default;

void CameraThermalProcessor::startWork()
{
    connect(&m_inputMessageQueue, &MessageQueue::messageEnqueued,
        this, &CameraThermalProcessor::handleInputMessages);
    handleInputMessages();
}

void CameraThermalProcessor::stopWork()
{
    disconnect(&m_inputMessageQueue, &MessageQueue::messageEnqueued,
        this, &CameraThermalProcessor::handleInputMessages);
}

void CameraThermalProcessor::submitFrame(const CameraPipelineFramePtr& frame)
{
    m_inputMessageQueue.push(Camera::MsgProcessFrame::create(frame));
}

void CameraThermalProcessor::handleInputMessages()
{
    Camera::discardQueuedProcessFramesOnCaptureActive(m_inputMessageQueue);

    Message *message = nullptr;
    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        handleMessage(*message);
        delete message;
    }
}

bool CameraThermalProcessor::handleMessage(const Message& message)
{
    if (Camera::MsgConfigureCamera::match(message))
    {
        const auto& configure = static_cast<const Camera::MsgConfigureCamera&>(message);
        applySettings(configure.getSettings(), configure.getSettingsKeys(), configure.getForce());
        return true;
    }
    if (Camera::MsgProcessFrame::match(message))
    {
        const auto& process = static_cast<const Camera::MsgProcessFrame&>(message);
        const CameraPipelineFramePtr frame = process.getFrame();
        if (!Camera::acceptsPipelineFrame(frame, m_captureActive, m_captureEpoch)) {
            return true;
        }
        if (!frame->m_thermal.m_rawFrame.m_bytes.isEmpty()) {
            m_lastThermalInput.reset(new CameraPipelineFrame(*frame));
        }
        const bool thermal = processThermalFrame(*frame);
        if (thermal || (!frame->m_thermal.m_calibrationFrame && !frame->m_image.isNull())) {
            forward(frame);
        }
        return true;
    }
    if (Camera::MsgCaptureActive::match(message))
    {
        const auto& active = static_cast<const Camera::MsgCaptureActive&>(message);
        m_captureActive = active.isActive();
        m_captureEpoch = active.getCaptureEpoch();
        if (m_captureActive) {
            m_lastThermalInput.clear();
        }
        return true;
    }
    return false;
}

void CameraThermalProcessor::applySettings(const CameraSettings& settings, const QList<QString>& keys, bool force)
{
    const bool sourceChanged = force
        || keys.contains(QStringLiteral("cameraId"))
        || keys.contains(QStringLiteral("cameraProtocol"))
        || keys.contains(QStringLiteral("resolutionWidth"))
        || keys.contains(QStringLiteral("resolutionHeight"));
    const bool decoderChanged = sourceChanged || keys.contains(QStringLiteral("thermalDecoder"));
    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(keys, settings);
    }
    if (decoderChanged)
    {
        m_decoder.reset();
        m_haveSmoothedRange = false;
        m_rawDumpAttempted = false;
        m_lastRawSignature.clear();
    }
    if (sourceChanged) {
        m_lastThermalInput.clear();
    }
    const bool renderingChanged = force || std::any_of(keys.cbegin(), keys.cend(), [](const QString& key) {
        return key.startsWith(QStringLiteral("thermal"));
    });
    if (renderingChanged && m_lastThermalInput)
    {
        CameraPipelineFramePtr preview(new CameraPipelineFrame(*m_lastThermalInput));
        preview->m_manualPreviewFrame = true;
        const bool thermal = processThermalFrame(*preview);
        if (thermal || (!preview->m_thermal.m_calibrationFrame && !preview->m_image.isNull())) {
            forward(preview);
        }
    }
}

bool CameraThermalProcessor::processThermalFrame(CameraPipelineFrame& frame)
{
    frame.m_thermal.clearDecoded();
    if ((m_settings.m_thermalDecoder == CameraSettings::ThermalDecoderOff)
        || frame.m_thermal.m_rawFrame.m_bytes.isEmpty()) {
        return false;
    }

    const CameraThermalRawFrame& raw = frame.m_thermal.m_rawFrame;
    const QString signature = QStringLiteral("%1x%2 stride=%3 bytes=%4 format=%5 (%6)")
        .arg(raw.m_width).arg(raw.m_height).arg(raw.m_bytesPerLine).arg(raw.m_bytes.size())
        .arg(raw.m_pixelFormat).arg(raw.m_pixelFormatName);
    if (signature != m_lastRawSignature)
    {
        m_lastRawSignature = signature;
        qDebug() << "CameraThermalProcessor: raw Qt video frame" << signature;
    }
    if (!m_rawDumpAttempted)
    {
        m_rawDumpAttempted = true;
        const QString dumpPath = qEnvironmentVariable("SDRANGEL_CAMERA_THERMAL_RAW_DUMP");
        if (!dumpPath.isEmpty())
        {
            QFile dumpFile(dumpPath);
            if (dumpFile.open(QIODevice::WriteOnly) && (dumpFile.write(raw.m_bytes) == raw.m_bytes.size())) {
                qDebug() << "CameraThermalProcessor: wrote raw radiometric diagnostic frame to" << dumpPath;
            } else {
                qWarning() << "CameraThermalProcessor: failed to write raw radiometric diagnostic frame to" << dumpPath
                    << dumpFile.errorString();
            }
        }
    }

    if (!m_decoder)
    {
        m_decoder = m_settings.m_thermalDecoder == CameraSettings::ThermalDecoderAuto
            ? CameraThermalDecoder::autoDetect(raw)
            : CameraThermalDecoder::create(m_settings.m_thermalDecoder);
    }
    if (!m_decoder) {
        frame.m_thermal.m_status = QStringLiteral("No supported radiometric plane in Qt video frame");
        return false;
    }

    CameraThermalDecodeResult decoded;
    if (!m_decoder->decode(raw, decoded))
    {
        frame.m_thermal.m_calibrationFrame = decoded.m_calibrationFrame;
        frame.m_thermal.m_status = decoded.m_calibrationFrame
            ? QStringLiteral("Camera calibration frame")
            : QStringLiteral("Radiometric frame decode failed");
        return false;
    }

    frame.m_thermal.m_temperatureC = decoded.m_temperatureC;
    frame.m_thermal.m_decoderName = decoded.m_decoderName;
    frame.m_thermal.m_status = decoded.m_diagnostic;
    frame.m_thermal.m_valid = true;
    updateStatistics(frame);

    double lowC = m_settings.m_thermalMinimumC;
    double highC = m_settings.m_thermalMaximumC;
    if (m_settings.m_thermalAutoRange)
    {
        cv::Mat flattened = decoded.m_temperatureC.reshape(1, 1).clone();
        cv::sort(flattened, flattened, cv::SORT_ASCENDING);
        const int count = flattened.cols;
        const int lowIndex = qBound(0, qRound((count - 1) * m_settings.m_thermalAutoLowPercentile / 100.0), count - 1);
        const int highIndex = qBound(0, qRound((count - 1) * m_settings.m_thermalAutoHighPercentile / 100.0), count - 1);
        const double measuredLow = flattened.at<float>(0, lowIndex);
        const double measuredHigh = flattened.at<float>(0, std::max(lowIndex + 1, highIndex));
        const double alpha = qBound(0.01, m_settings.m_thermalAutoRangeSmoothing, 1.0);
        if (!m_haveSmoothedRange)
        {
            m_smoothedLowC = measuredLow;
            m_smoothedHighC = measuredHigh;
            m_haveSmoothedRange = true;
        }
        else
        {
            m_smoothedLowC += alpha * (measuredLow - m_smoothedLowC);
            m_smoothedHighC += alpha * (measuredHigh - m_smoothedHighC);
        }
        lowC = m_smoothedLowC;
        highC = m_smoothedHighC;
    }
    if (highC <= lowC + 0.01) {
        highC = lowC + 0.01;
    }

    frame.m_image = colorize(decoded.m_temperatureC, lowC, highC);
    frame.m_unprocessedImage = frame.m_image;
    // The packed radiometric plane is not an image and must not be exposed as a
    // raw RGB FITS frame. Calibrated/post-processed recording uses m_unprocessedImage.
    frame.m_rawInputImage = QImage();
    frame.m_imageTransform.clear();
    frame.clearCudaCache();
    return !frame.m_image.isNull();
}

void CameraThermalProcessor::updateStatistics(CameraPipelineFrame& frame) const
{
    double minValue = 0.0;
    double maxValue = 0.0;
    cv::Point minPoint;
    cv::Point maxPoint;
    cv::minMaxLoc(frame.m_thermal.m_temperatureC, &minValue, &maxValue, &minPoint, &maxPoint);
    frame.m_thermal.m_minimumC = static_cast<float>(minValue);
    frame.m_thermal.m_maximumC = static_cast<float>(maxValue);
    frame.m_thermal.m_meanC = static_cast<float>(cv::mean(frame.m_thermal.m_temperatureC)[0]);
    frame.m_thermal.m_minimumPosition = QPoint(minPoint.x, minPoint.y);
    frame.m_thermal.m_maximumPosition = QPoint(maxPoint.x, maxPoint.y);

    const int x = qBound(0, qRound(m_settings.m_thermalMarkerX * (frame.m_thermal.m_temperatureC.cols - 1)),
        frame.m_thermal.m_temperatureC.cols - 1);
    const int y = qBound(0, qRound(m_settings.m_thermalMarkerY * (frame.m_thermal.m_temperatureC.rows - 1)),
        frame.m_thermal.m_temperatureC.rows - 1);
    frame.m_thermal.m_markerPosition = QPoint(x, y);
    frame.m_thermal.m_markerTemperatureC = frame.m_thermal.m_temperatureC.at<float>(y, x);
}

QImage CameraThermalProcessor::colorize(const cv::Mat& temperatureC, double lowC, double highC) const
{
    cv::Mat normalized;
    temperatureC.convertTo(normalized, CV_8UC1, 255.0 / (highC - lowC), -lowC * 255.0 / (highC - lowC));
    cv::Mat bgr;
    switch (m_settings.m_thermalPalette)
    {
    case CameraSettings::ThermalPaletteBlackHot:
        cv::bitwise_not(normalized, normalized);
        cv::cvtColor(normalized, bgr, cv::COLOR_GRAY2BGR);
        break;
    case CameraSettings::ThermalPaletteIron:
        cv::applyColorMap(normalized, bgr, cv::COLORMAP_HOT);
        break;
    case CameraSettings::ThermalPaletteInferno:
        cv::applyColorMap(normalized, bgr, cv::COLORMAP_INFERNO);
        break;
    case CameraSettings::ThermalPaletteTurbo:
        cv::applyColorMap(normalized, bgr, cv::COLORMAP_TURBO);
        break;
    case CameraSettings::ThermalPaletteViridis:
        cv::applyColorMap(normalized, bgr, cv::COLORMAP_VIRIDIS);
        break;
    case CameraSettings::ThermalPaletteWhiteHot:
    default:
        cv::cvtColor(normalized, bgr, cv::COLOR_GRAY2BGR);
        break;
    }
    return CameraPipelineFrame::bgrMatToRgbImage(bgr);
}

void CameraThermalProcessor::forward(const CameraPipelineFramePtr& frame)
{
    if (m_nextStage && Camera::acceptsPipelineFrame(frame, m_captureActive, m_captureEpoch)) {
        m_nextStage->submitFrame(frame);
    }
}
