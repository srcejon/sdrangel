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

#include <algorithm>
#include <cmath>
#include <cstring>

#include <QDebug>
#include <QTimer>

#include "util/profiler.h"
#include "cameraframestacker.h"
#include "cameraimageprocessor.h"

MESSAGE_CLASS_DEFINITION(CameraFrameStacker::MsgConfigureCameraFrameStacker, Message)
MESSAGE_CLASS_DEFINITION(CameraFrameStacker::MsgProcessFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraFrameStacker::MsgCaptureActive, Message)

CameraFrameStacker::CameraFrameStacker() :
    m_nextStageInputMessageQueue(nullptr),
    m_captureActive(false)
{
}

CameraFrameStacker::~CameraFrameStacker() = default;

void CameraFrameStacker::startWork()
{
    connect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()), Qt::QueuedConnection);
}

void CameraFrameStacker::stopWork()
{
    disconnect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));
}

void CameraFrameStacker::resetFrameHistoryState()
{
    m_stackFrameHistory.clear();
    m_stackAccumulator.release();
}

bool CameraFrameStacker::handleMessage(const Message& cmd)
{
    if (MsgConfigureCameraFrameStacker::match(cmd))
    {
        const MsgConfigureCameraFrameStacker& cfg = (const MsgConfigureCameraFrameStacker&) cmd;
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
        if (m_captureActive) {
            resetFrameHistoryState();
        }
        return true;
    }

    return false;
}

void CameraFrameStacker::handleInputMessages()
{
    Message* message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void CameraFrameStacker::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraFrameStacker::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

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
        resetFrameHistoryState();
    }
}

void CameraFrameStacker::processNewFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || frame->m_image.isNull()) {
        return;
    }

    frame->m_image = applyFrameStacking(frame->m_image);
    frame->m_unprocessedImage = frame->m_image;

    if (m_nextStageInputMessageQueue) {
        m_nextStageInputMessageQueue->push(CameraImageProcessor::MsgProcessFrame::create(frame));
    }
}

QImage CameraFrameStacker::applyFrameStacking(const QImage& input)
{
    PROFILER_START();
    const bool highBitDepthInput = (input.format() == QImage::Format_RGBA64) || (input.format() == QImage::Format_RGBX64);

    auto convertToRgb888 = [](const QImage& source) -> QImage {
        if ((source.format() != QImage::Format_RGBA64) && (source.format() != QImage::Format_RGBX64)) {
            return source.convertToFormat(QImage::Format_RGB888);
        }

        QImage rgb8(source.width(), source.height(), QImage::Format_RGB888);
        for (int y = 0; y < source.height(); ++y)
        {
            const QRgba64 *inputLine = reinterpret_cast<const QRgba64*>(source.constScanLine(y));
            uchar *outputLine = rgb8.scanLine(y);

            for (int x = 0; x < source.width(); ++x)
            {
                outputLine[x * 3 + 0] = static_cast<uchar>(std::lround((inputLine[x].red() * 255.0) / 65535.0));
                outputLine[x * 3 + 1] = static_cast<uchar>(std::lround((inputLine[x].green() * 255.0) / 65535.0));
                outputLine[x * 3 + 2] = static_cast<uchar>(std::lround((inputLine[x].blue() * 255.0) / 65535.0));
            }
        }

        return rgb8;
    };

    if (!m_settings.m_stackEnabled || (m_settings.m_stackFrameCount <= 1)) {
        return highBitDepthInput ? convertToRgb888(input) : input;
    }

    cv::Mat frameMat;
    if (highBitDepthInput)
    {
        frameMat = cv::Mat(input.height(), input.width(), CV_16UC3);
        for (int y = 0; y < input.height(); ++y)
        {
            const QRgba64 *inputLine = reinterpret_cast<const QRgba64*>(input.constScanLine(y));
            cv::Vec<uint16_t, 3> *outputLine = frameMat.ptr<cv::Vec<uint16_t, 3>>(y);

            for (int x = 0; x < input.width(); ++x)
            {
                outputLine[x][0] = inputLine[x].red();
                outputLine[x][1] = inputLine[x].green();
                outputLine[x][2] = inputLine[x].blue();
            }
        }
    }
    else
    {
        const QImage rgb = input.convertToFormat(QImage::Format_RGB888);
        cv::Mat rgbMat(rgb.height(), rgb.width(), CV_8UC3,
                       const_cast<uchar*>(rgb.bits()),
                       static_cast<size_t>(rgb.bytesPerLine()));
        frameMat = rgbMat.clone();
    }

    cv::Mat alignedFrameMat = frameMat;

    if (m_stackFrameHistory.empty()
        || m_stackFrameHistory.front().size() != alignedFrameMat.size()
        || m_stackFrameHistory.front().type() != alignedFrameMat.type())
    {
        m_stackFrameHistory.clear();
        m_stackAccumulator.release();
    }

    m_stackFrameHistory.push_back(alignedFrameMat.clone());

    const int maxFrames = qBound(1, m_settings.m_stackFrameCount, 256);
    while (static_cast<int>(m_stackFrameHistory.size()) > maxFrames)
    {
        if (m_settings.m_stackMethod == CameraSettings::StackMethodAverage)
        {
            cv::Mat oldestFloatFrame;
            m_stackFrameHistory.front().convertTo(oldestFloatFrame, CV_32FC3);
            m_stackAccumulator -= oldestFloatFrame;
        }
        m_stackFrameHistory.pop_front();
    }

    const double scaleTo8Bit = highBitDepthInput ? (255.0 / 65535.0) : 1.0;

    if (m_settings.m_stackMethod == CameraSettings::StackMethodAverage)
    {
        if (m_stackAccumulator.empty()) {
            m_stackAccumulator = cv::Mat::zeros(alignedFrameMat.size(), CV_32FC3);
        }

        cv::Mat floatFrame;
        alignedFrameMat.convertTo(floatFrame, CV_32FC3);
        m_stackAccumulator += floatFrame;

        cv::Mat averagedFloat;
        m_stackAccumulator.convertTo(averagedFloat, CV_32FC3, 1.0 / static_cast<double>(m_stackFrameHistory.size()));
        cv::Mat averaged8u;
        averagedFloat.convertTo(averaged8u, CV_8UC3, scaleTo8Bit);

        QImage stackedImage(averaged8u.cols, averaged8u.rows, QImage::Format_RGB888);
        for (int row = 0; row < averaged8u.rows; ++row) {
            std::memcpy(stackedImage.scanLine(row), averaged8u.ptr(row), static_cast<size_t>(averaged8u.cols * 3));
        }

        return stackedImage;
    }

    m_stackAccumulator.release();

    QImage stackedImage(alignedFrameMat.cols, alignedFrameMat.rows, QImage::Format_RGB888);
    const size_t frameCount = m_stackFrameHistory.size();
    constexpr double sigmaThreshold = 2.0;

    std::vector<int> channelSamples[3];
    for (std::vector<int>& samples : channelSamples) {
        samples.reserve(frameCount);
    }

    for (int row = 0; row < alignedFrameMat.rows; ++row)
    {
        uchar *output = stackedImage.scanLine(row);

        for (int col = 0; col < alignedFrameMat.cols; ++col)
        {
            for (std::vector<int>& samples : channelSamples) {
                samples.clear();
            }

            for (const cv::Mat& frame : m_stackFrameHistory)
            {
                if (highBitDepthInput)
                {
                    const cv::Vec<uint16_t, 3>& pixel = frame.at<cv::Vec<uint16_t, 3>>(row, col);
                    channelSamples[0].push_back(pixel[0]);
                    channelSamples[1].push_back(pixel[1]);
                    channelSamples[2].push_back(pixel[2]);
                }
                else
                {
                    const cv::Vec3b& pixel = frame.at<cv::Vec3b>(row, col);
                    channelSamples[0].push_back(pixel[0]);
                    channelSamples[1].push_back(pixel[1]);
                    channelSamples[2].push_back(pixel[2]);
                }
            }

            for (int channel = 0; channel < 3; ++channel)
            {
                int channelValue = 0;

                if (m_settings.m_stackMethod == CameraSettings::StackMethodMedian)
                {
                    std::vector<int>& samples = channelSamples[channel];
                    const size_t medianIndex = samples.size() / 2;
                    std::nth_element(samples.begin(), samples.begin() + static_cast<std::ptrdiff_t>(medianIndex), samples.end());
                    channelValue = samples[medianIndex];
                }
                else
                {
                    const std::vector<int>& samples = channelSamples[channel];
                    double sum = 0.0;
                    double sumSquares = 0.0;

                    for (int sample : samples)
                    {
                        sum += sample;
                        sumSquares += static_cast<double>(sample) * sample;
                    }

                    const double mean = sum / static_cast<double>(samples.size());
                    const double variance = std::max(0.0, (sumSquares / static_cast<double>(samples.size())) - (mean * mean));
                    const double sigma = std::sqrt(variance);
                    const double minValue = mean - sigmaThreshold * sigma;
                    const double maxValue = mean + sigmaThreshold * sigma;

                    double clippedSum = 0.0;
                    int clippedCount = 0;
                    for (int sample : samples)
                    {
                        if ((sample >= minValue) && (sample <= maxValue))
                        {
                            clippedSum += sample;
                            ++clippedCount;
                        }
                    }

                    channelValue = clippedCount > 0
                        ? static_cast<int>(std::lround(clippedSum / static_cast<double>(clippedCount)))
                        : static_cast<int>(std::lround(mean));
                }

                const int outputValue = highBitDepthInput
                    ? static_cast<int>(std::lround((channelValue * 255.0) / 65535.0))
                    : channelValue;
                output[col * 3 + channel] = static_cast<uchar>(qBound(0, outputValue, 255));
            }
        }
    }

    return stackedImage;
}
