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

#include <cmath>

#include <QDebug>

#include "util/profiler.h"
#include "cameraimageprocessor.h"
#include "camerapostprocessor.h"

MESSAGE_CLASS_DEFINITION(CameraImageProcessor::MsgConfigureCameraImageProcessor, Message)
MESSAGE_CLASS_DEFINITION(CameraImageProcessor::MsgProcessFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraImageProcessor::MsgCaptureActive, Message)

CameraImageProcessor::CameraImageProcessor() :
    m_nextStageInputMessageQueue(nullptr),
    m_captureActive(false),
    m_autoWhiteBalanceGains(1.0, 1.0, 1.0),
    m_autoWhiteBalanceInitialized(false)
{
}

CameraImageProcessor::~CameraImageProcessor() = default;

void CameraImageProcessor::startWork()
{
    QObject::connect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraImageProcessor::handleInputMessages);
    handleInputMessages();
}

void CameraImageProcessor::stopWork()
{
    QObject::disconnect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraImageProcessor::handleInputMessages);
}

bool CameraImageProcessor::handleMessage(const Message& cmd)
{
    if (MsgConfigureCameraImageProcessor::match(cmd))
    {
        const MsgConfigureCameraImageProcessor& cfg = (const MsgConfigureCameraImageProcessor&) cmd;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (MsgProcessFrame::match(cmd))
    {
        const MsgProcessFrame& frameMsg = (const MsgProcessFrame&) cmd;
        processNewFrame(frameMsg.getFrame());
        return true;
    }
    else if (MsgCaptureActive::match(cmd))
    {
        const MsgCaptureActive& activeMsg = (const MsgCaptureActive&) cmd;
        m_captureActive = activeMsg.isActive();
        if (m_captureActive)
        {
            m_lastInputFrame = CameraPipelineFrame();
            m_autoWhiteBalanceGains = cv::Vec3d(1.0, 1.0, 1.0);
            m_autoWhiteBalanceInitialized = false;
        }
        return true;
    }

    return false;
}

void CameraImageProcessor::handleInputMessages()
{
    Message *message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }

        if (m_inputMessageQueue.size() > 20)
        {
            while (m_inputMessageQueue.size() > 0)
            {
                message = m_inputMessageQueue.pop();
                if (MsgProcessFrame::match(*message))
                {
                    qDebug() << "CameraImageProcessor: Dropping frame to catch up";
                    delete message;
                }
                else
                {
                    if (handleMessage(*message)) {
                        delete message;
                    }
                }
            }
        }
    }
}

void CameraImageProcessor::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraImageProcessor::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    static const QStringList kImageProcessingKeys = {
        "postProcessWhiteBalanceMode",
        "postProcessWhiteBalanceRedGain",
        "postProcessWhiteBalanceGreenGain",
        "postProcessWhiteBalanceBlueGain",
        "saturation", "gamma", "gaussianBlur", "medianBlur", "sharpen", "sobelEdge", "flipX", "flipY",
        "brightness", "contrast", "invertColors"
    };
    const bool imageProcessingChanged = force || std::any_of(kImageProcessingKeys.cbegin(), kImageProcessingKeys.cend(),
        [&settingsKeys](const QString& k) { return settingsKeys.contains(k); });
    const bool sourceChanged = force
        || settingsKeys.contains("cameraId")
        || settingsKeys.contains("cameraProtocol")
        || settingsKeys.contains("resolutionWidth")
        || settingsKeys.contains("resolutionHeight")
        || settingsKeys.contains("cameraBinX")
        || settingsKeys.contains("cameraBinY")
        || settingsKeys.contains("cameraNumX")
        || settingsKeys.contains("cameraNumY")
        || settingsKeys.contains("cameraStartX")
        || settingsKeys.contains("cameraStartY")
        || settingsKeys.contains("cameraGain")
        || settingsKeys.contains("cameraOffset")
        || settingsKeys.contains("cameraReadoutMode")
        || settingsKeys.contains("exposureTimeMs")
        || settingsKeys.contains("stackEnabled")
        || settingsKeys.contains("stackFrameCount")
        || settingsKeys.contains("stackMethod")
        || settingsKeys.contains("stackAlignmentMethod");

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (sourceChanged) {
        m_lastInputFrame = CameraPipelineFrame();
    }

    if (force
        || settingsKeys.contains("postProcessWhiteBalanceMode")
        || settingsKeys.contains("postProcessWhiteBalanceRedGain")
        || settingsKeys.contains("postProcessWhiteBalanceGreenGain")
        || settingsKeys.contains("postProcessWhiteBalanceBlueGain"))
    {
        m_autoWhiteBalanceGains = cv::Vec3d(1.0, 1.0, 1.0);
        m_autoWhiteBalanceInitialized = false;
    }

    if (imageProcessingChanged && !m_lastInputFrame.m_image.isNull()) {
        processNewFrame(m_lastInputFrame);
    }
}

void CameraImageProcessor::processNewFrame(const CameraPipelineFrame& frame)
{
    if (frame.m_image.isNull()) {
        return;
    }

    m_lastInputFrame = frame;

    CameraPipelineFrame outputFrame = frame;
    if (outputFrame.m_unprocessedImage.isNull()) {
        outputFrame.m_unprocessedImage = frame.m_image;
    }
    outputFrame.m_image = applyImageProcessing(frame.m_image);

    if (m_nextStageInputMessageQueue) {
        m_nextStageInputMessageQueue->push(CameraPostProcessor::MsgProcessFrame::create(outputFrame));
    }
}

QImage CameraImageProcessor::applyImageProcessing(const QImage& input)
{
    PROFILER_START();

    const bool needsWhiteBalance = m_settings.m_postProcessWhiteBalanceMode != 0;
    const bool needsSaturation = std::abs(m_settings.m_saturation - 1.0) > 1e-4;
    const bool needsGamma = std::abs(m_settings.m_gamma - 1.0) > 1e-4;
    const bool needsGaussianBlur = m_settings.m_gaussianBlur > 0;
    const bool needsMedianBlur = m_settings.m_medianBlur > 0;
    const bool needsSharpen = m_settings.m_sharpen > 1e-4;
    const bool needsSobelEdge = m_settings.m_sobelEdge > 1e-4;
    const bool needsFlip = m_settings.m_flipX || m_settings.m_flipY;
    const bool needsBrightContrast = (m_settings.m_brightness != 0.0 || m_settings.m_contrast != 1.0);
    const bool needsAny = needsWhiteBalance
        || needsSaturation
        || needsGamma
        || needsGaussianBlur
        || needsMedianBlur
        || needsSharpen
        || needsSobelEdge
        || needsFlip
        || needsBrightContrast
        || m_settings.m_invertColors;

    if (!needsAny) {
        return input;
    }

    const QImage rgb = input.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(rgb.height(), rgb.width(), CV_8UC3,
                const_cast<uchar*>(rgb.bits()),
                static_cast<size_t>(rgb.bytesPerLine()));
    cv::Mat bgrMat;
    cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);

    if (needsWhiteBalance) { applyWhiteBalance(bgrMat); }
    if (needsSaturation) { applySaturation(bgrMat); }
    if (needsGamma) { applyGamma(bgrMat); }
    if (needsGaussianBlur) { applyGaussianBlur(bgrMat); }
    if (needsMedianBlur) { applyMedianBlur(bgrMat); }
    if (needsSharpen) { applySharpen(bgrMat); }
    if (needsSobelEdge) { applySobelEdge(bgrMat); }
    if (needsFlip) { applyFlip(bgrMat); }
    if (needsBrightContrast) { applyBrightnessContrast(bgrMat); }
    if (m_settings.m_invertColors) { applyInvertColors(bgrMat); }

    QImage result = convertBgrToRgbImage(bgrMat);
    PROFILER_STOP("CameraImageProcessor::applyImageProcessing");
    return result;
}

void CameraImageProcessor::applyWhiteBalance(cv::Mat& bgrMat)
{
    PROFILER_START();
    cv::Vec3d gains(
        m_settings.m_postProcessWhiteBalanceBlueGain,
        m_settings.m_postProcessWhiteBalanceGreenGain,
        m_settings.m_postProcessWhiteBalanceRedGain);

    if (m_settings.m_postProcessWhiteBalanceMode == 1)
    {
        const cv::Scalar means = cv::mean(bgrMat);
        const double blueMean = std::max(1.0, means[0]);
        const double greenMean = std::max(1.0, means[1]);
        const double redMean = std::max(1.0, means[2]);
        const double targetMean = (blueMean + greenMean + redMean) / 3.0;

        cv::Vec3d targetGains(
            qBound(0.25, targetMean / blueMean, 4.0),
            qBound(0.25, targetMean / greenMean, 4.0),
            qBound(0.25, targetMean / redMean, 4.0));

        if (!m_autoWhiteBalanceInitialized)
        {
            m_autoWhiteBalanceGains = targetGains;
            m_autoWhiteBalanceInitialized = true;
        }
        else
        {
            static constexpr double kAutoWhiteBalanceSmoothing = 0.2;
            m_autoWhiteBalanceGains =
                (1.0 - kAutoWhiteBalanceSmoothing) * m_autoWhiteBalanceGains
                + kAutoWhiteBalanceSmoothing * targetGains;
        }

        gains = m_autoWhiteBalanceGains;
    }

    std::vector<cv::Mat> channels;
    cv::split(bgrMat, channels);
    channels[0].convertTo(channels[0], -1, gains[0], 0.0);
    channels[1].convertTo(channels[1], -1, gains[1], 0.0);
    channels[2].convertTo(channels[2], -1, gains[2], 0.0);
    cv::merge(channels, bgrMat);
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applySaturation(cv::Mat& bgrMat)
{
    PROFILER_START();
    cv::Mat hsvMat;
    cv::cvtColor(bgrMat, hsvMat, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsvChannels;
    cv::split(hsvMat, hsvChannels);
    hsvChannels[1].convertTo(hsvChannels[1], -1, m_settings.m_saturation, 0.0);
    cv::merge(hsvChannels, hsvMat);
    cv::cvtColor(hsvMat, bgrMat, cv::COLOR_HSV2BGR);
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyGamma(cv::Mat& bgrMat) const
{
    PROFILER_START();
    cv::Mat lut(1, 256, CV_8U);
    uchar* lutData = lut.ptr<uchar>();
    for (int i = 0; i < 256; ++i) {
        lutData[i] = static_cast<uchar>(qBound(0, static_cast<int>(std::pow(i / 255.0, m_settings.m_gamma) * 255.0 + 0.5), 255));
    }
    cv::LUT(bgrMat, lut, bgrMat);
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyGaussianBlur(cv::Mat& bgrMat) const
{
    PROFILER_START();
    const int kernelSize = 2 * m_settings.m_gaussianBlur + 1;
    cv::GaussianBlur(bgrMat, bgrMat, cv::Size(kernelSize, kernelSize), 0);
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyMedianBlur(cv::Mat& bgrMat) const
{
    PROFILER_START();
    const int kernelSize = 2 * m_settings.m_medianBlur + 1;
    cv::medianBlur(bgrMat, bgrMat, kernelSize);
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applySharpen(cv::Mat& bgrMat) const
{
    PROFILER_START();
    cv::Mat blurred;
    cv::GaussianBlur(bgrMat, blurred, cv::Size(0, 0), 1.0);
    cv::addWeighted(bgrMat, 1.0 + m_settings.m_sharpen, blurred, -m_settings.m_sharpen, 0.0, bgrMat);
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applySobelEdge(cv::Mat& bgrMat) const
{
    PROFILER_START();
    cv::Mat grayMat;
    cv::cvtColor(bgrMat, grayMat, cv::COLOR_BGR2GRAY);

    cv::Mat gradX;
    cv::Mat gradY;
    cv::Sobel(grayMat, gradX, CV_16S, 1, 0, 3);
    cv::Sobel(grayMat, gradY, CV_16S, 0, 1, 3);

    cv::Mat absGradX;
    cv::Mat absGradY;
    cv::convertScaleAbs(gradX, absGradX);
    cv::convertScaleAbs(gradY, absGradY);

    cv::Mat edgesGray;
    cv::addWeighted(absGradX, 0.5, absGradY, 0.5, 0.0, edgesGray);

    cv::Mat edgesBgr;
    cv::cvtColor(edgesGray, edgesBgr, cv::COLOR_GRAY2BGR);
    cv::addWeighted(bgrMat, 1.0, edgesBgr, m_settings.m_sobelEdge, 0.0, bgrMat);
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyFlip(cv::Mat& bgrMat) const
{
    PROFILER_START();
    const int flipCode = m_settings.m_flipX && m_settings.m_flipY ? -1 : (m_settings.m_flipX ? 1 : 0);
    cv::flip(bgrMat, bgrMat, flipCode);
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyBrightnessContrast(cv::Mat& bgrMat) const
{
    PROFILER_START();
    cv::Mat adjusted;
    cv::convertScaleAbs(bgrMat, adjusted, m_settings.m_contrast, m_settings.m_brightness);
    bgrMat = adjusted;
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyInvertColors(cv::Mat& bgrMat) const
{
    PROFILER_START();
    cv::bitwise_not(bgrMat, bgrMat);
    PROFILER_STOP(__FUNCTION__);
}

QImage CameraImageProcessor::convertBgrToRgbImage(cv::Mat& bgrMat) const
{
    PROFILER_START();
    cv::cvtColor(bgrMat, bgrMat, cv::COLOR_BGR2RGB);
    const QImage rawResult(bgrMat.data, bgrMat.cols, bgrMat.rows,
                           static_cast<qsizetype>(bgrMat.step[0]),
                           QImage::Format_RGB888);
    return rawResult.copy();
}
