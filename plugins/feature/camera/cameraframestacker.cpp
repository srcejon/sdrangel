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

    cv::Mat alignedFrameMat = alignStackFrame(frameMat);

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

cv::Mat CameraFrameStacker::alignStackFrame(const cv::Mat& frameMat) const
{
    if (m_stackFrameHistory.empty()) {
        return frameMat.clone();
    }

    const cv::Mat& referenceFrame = m_stackFrameHistory.front();
    if (referenceFrame.size() != frameMat.size() || referenceFrame.type() != frameMat.type()) {
        return frameMat.clone();
    }

    switch (m_settings.m_stackAlignmentMethod)
    {
    case CameraSettings::StackAlignmentPhaseCorrelation:
        return alignWithPhaseCorrelation(referenceFrame, frameMat);
    case CameraSettings::StackAlignmentStarCentroidMatching:
        return alignWithStarCentroids(referenceFrame, frameMat);
    case CameraSettings::StackAlignmentNone:
    default:
        return frameMat.clone();
    }
}

cv::Mat CameraFrameStacker::frameToAlignmentGray(const cv::Mat& frameMat) const
{
    cv::Mat gray;
    if (frameMat.channels() == 1)
    {
        gray = frameMat;
    }
    else
    {
        cv::cvtColor(frameMat, gray, cv::COLOR_RGB2GRAY);
    }

    if (gray.depth() == CV_8U) {
        return gray;
    }

    cv::Mat gray8;
    if (gray.depth() == CV_16U) {
        gray.convertTo(gray8, CV_8U, 255.0 / 65535.0);
    } else {
        gray.convertTo(gray8, CV_8U);
    }

    return gray8;
}

cv::Mat CameraFrameStacker::warpFrameAffine(const cv::Mat& frameMat, const cv::Mat& transform) const
{
    if (transform.empty()) {
        return frameMat.clone();
    }

    cv::Mat aligned;
    cv::warpAffine(frameMat, aligned, transform, frameMat.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    return aligned;
}

cv::Mat CameraFrameStacker::alignWithPhaseCorrelation(const cv::Mat& referenceFrame, const cv::Mat& targetFrame) const
{
    const cv::Mat referenceGray = frameToAlignmentGray(referenceFrame);
    const cv::Mat targetGray = frameToAlignmentGray(targetFrame);

    cv::Mat referenceFloat;
    cv::Mat targetFloat;
    referenceGray.convertTo(referenceFloat, CV_32F);
    targetGray.convertTo(targetFloat, CV_32F);

    const cv::Point2d shift = cv::phaseCorrelate(targetFloat, referenceFloat);
    cv::Mat transform = (cv::Mat_<double>(2, 3) << 1.0, 0.0, shift.x, 0.0, 1.0, shift.y);
    return warpFrameAffine(targetFrame, transform);
}

std::vector<cv::Point2f> CameraFrameStacker::detectStarCentroids(const cv::Mat& grayFrame) const
{
    std::vector<cv::Point2f> stars;
    if (grayFrame.empty()) {
        return stars;
    }

    cv::Mat blurred;
    cv::GaussianBlur(grayFrame, blurred, cv::Size(0, 0), 1.2);

    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(blurred, mean, stddev);

    double maxValue = 0.0;
    cv::minMaxLoc(blurred, nullptr, &maxValue);
    const double thresholdValue = std::max(mean[0] + (2.5 * stddev[0]), maxValue * 0.55);
    if (thresholdValue <= 0.0) {
        return stars;
    }

    cv::Mat binary;
    cv::threshold(blurred, binary, thresholdValue, 255, cv::THRESH_BINARY);

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int componentCount = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 8, CV_32S);

    struct StarCandidate {
        cv::Point2f centroid;
        double brightness;
    };

    std::vector<StarCandidate> candidates;
    for (int component = 1; component < componentCount; ++component)
    {
        const int area = stats.at<int>(component, cv::CC_STAT_AREA);
        if (area < 1 || area > 200) {
            continue;
        }

        const int left = stats.at<int>(component, cv::CC_STAT_LEFT);
        const int top = stats.at<int>(component, cv::CC_STAT_TOP);
        const int width = stats.at<int>(component, cv::CC_STAT_WIDTH);
        const int height = stats.at<int>(component, cv::CC_STAT_HEIGHT);
        const cv::Rect roi(left, top, width, height);

        cv::Mat componentMask = (labels(roi) == component);
        const double brightness = cv::mean(blurred(roi), componentMask)[0] * area;
        candidates.push_back({
            cv::Point2f(static_cast<float>(centroids.at<double>(component, 0)),
                        static_cast<float>(centroids.at<double>(component, 1))),
            brightness
        });
    }

    std::sort(candidates.begin(), candidates.end(),
        [](const StarCandidate& lhs, const StarCandidate& rhs) { return lhs.brightness > rhs.brightness; });

    const size_t maxStars = std::min<size_t>(candidates.size(), 64);
    stars.reserve(maxStars);
    for (size_t i = 0; i < maxStars; ++i) {
        stars.push_back(candidates[i].centroid);
    }

    return stars;
}

cv::Mat CameraFrameStacker::alignWithStarCentroids(const cv::Mat& referenceFrame, const cv::Mat& targetFrame) const
{
    const cv::Mat referenceGray = frameToAlignmentGray(referenceFrame);
    const cv::Mat targetGray = frameToAlignmentGray(targetFrame);

    const std::vector<cv::Point2f> referenceStars = detectStarCentroids(referenceGray);
    const std::vector<cv::Point2f> targetStars = detectStarCentroids(targetGray);
    if (referenceStars.size() < 2 || targetStars.size() < 2) {
        return alignWithPhaseCorrelation(referenceFrame, targetFrame);
    }

    cv::Mat referenceFloat;
    cv::Mat targetFloat;
    referenceGray.convertTo(referenceFloat, CV_32F);
    targetGray.convertTo(targetFloat, CV_32F);
    const cv::Point2d shift = cv::phaseCorrelate(targetFloat, referenceFloat);

    std::vector<cv::Point2f> matchedTargetPoints;
    std::vector<cv::Point2f> matchedReferencePoints;
    std::vector<bool> targetUsed(targetStars.size(), false);
    constexpr float maxMatchDistance = 12.0f;

    for (const cv::Point2f& referenceStar : referenceStars)
    {
        int bestIndex = -1;
        float bestDistance = maxMatchDistance;

        for (size_t i = 0; i < targetStars.size(); ++i)
        {
            if (targetUsed[i]) {
                continue;
            }

            const cv::Point2f shiftedTarget(targetStars[i].x + static_cast<float>(shift.x),
                                            targetStars[i].y + static_cast<float>(shift.y));
            const float dx = shiftedTarget.x - referenceStar.x;
            const float dy = shiftedTarget.y - referenceStar.y;
            const float distance = std::sqrt((dx * dx) + (dy * dy));

            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestIndex = static_cast<int>(i);
            }
        }

        if (bestIndex >= 0)
        {
            targetUsed[bestIndex] = true;
            matchedTargetPoints.push_back(targetStars[bestIndex]);
            matchedReferencePoints.push_back(referenceStar);
        }
    }

    if (matchedTargetPoints.size() < 2) {
        return alignWithPhaseCorrelation(referenceFrame, targetFrame);
    }

    cv::Mat inliers;
    cv::Mat transform = cv::estimateAffinePartial2D(
        matchedTargetPoints, matchedReferencePoints, inliers, cv::RANSAC, 3.0);

    if (transform.empty()) {
        return alignWithPhaseCorrelation(referenceFrame, targetFrame);
    }

    return warpFrameAffine(targetFrame, transform);
}
