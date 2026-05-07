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

#include "util/profiler.h"
#include "cameraframealigner.h"
#include "cameraframestacker.h"

MESSAGE_CLASS_DEFINITION(CameraFrameAligner::MsgConfigureCameraFrameAligner, Message)
MESSAGE_CLASS_DEFINITION(CameraFrameAligner::MsgProcessFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraFrameAligner::MsgCaptureActive, Message)

CameraFrameAligner::CameraFrameAligner() :
    m_nextStage(nullptr),
    m_captureActive(false),
    m_processingFrame(false)
{
}

CameraFrameAligner::~CameraFrameAligner() = default;

void CameraFrameAligner::startWork()
{
    connect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()), Qt::QueuedConnection);
}

void CameraFrameAligner::stopWork()
{
    disconnect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));
}

void CameraFrameAligner::resetAlignmentState()
{
    m_alignmentReferenceHistory.clear();
}

bool CameraFrameAligner::handleMessage(const Message& cmd)
{
    if (MsgConfigureCameraFrameAligner::match(cmd))
    {
        const MsgConfigureCameraFrameAligner& cfg = (const MsgConfigureCameraFrameAligner&) cmd;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (MsgProcessFrame::match(cmd))
    {
        const MsgProcessFrame& frameMsg = (const MsgProcessFrame&) cmd;
        submitFrame(frameMsg.getFrame());
        return true;
    }
    else if (MsgCaptureActive::match(cmd))
    {
        const MsgCaptureActive& activeMsg = (const MsgCaptureActive&) cmd;
        m_captureActive = activeMsg.isActive();
        if (m_captureActive) {
            resetAlignmentState();
        }
        QMutexLocker locker(&m_frameMutex);
        m_pendingFrame.reset();
        if (!m_captureActive) {
            m_processingFrame = false;
        }
        return true;
    }

    return false;
}

void CameraFrameAligner::handleInputMessages()
{
    Message *message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void CameraFrameAligner::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraFrameAligner::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

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
        || settingsKeys.contains("stackAlignmentMethod");

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (sourceChanged) {
        resetAlignmentState();
    }
}

void CameraFrameAligner::submitFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame) {
        return;
    }

    bool schedule = false;
    {
        QMutexLocker locker(&m_frameMutex);
        m_pendingFrame = frame;
        if (!m_processingFrame)
        {
            m_processingFrame = true;
            schedule = true;
        }
    }

    if (schedule) {
        QMetaObject::invokeMethod(this, &CameraFrameAligner::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraFrameAligner::processNextFrame()
{
    CameraPipelineFramePtr frame;

    {
        QMutexLocker locker(&m_frameMutex);
        frame = m_pendingFrame;
        m_pendingFrame.reset();

        if (!frame)
        {
            m_processingFrame = false;
            return;
        }
    }

    processNewFrame(frame);

    bool schedule = false;
    {
        QMutexLocker locker(&m_frameMutex);
        if (m_pendingFrame) {
            schedule = true;
        } else {
            m_processingFrame = false;
        }
    }

    if (schedule) {
        QMetaObject::invokeMethod(this, &CameraFrameAligner::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraFrameAligner::processNewFrame(const CameraPipelineFramePtr& frame)
{
    PROFILER_START();
    if (!frame || frame->m_image.isNull()) {
        return;
    }

    if (m_settings.m_stackEnabled
        && (m_settings.m_stackFrameCount > 1)
        && (m_settings.m_stackAlignmentMethod != CameraSettings::StackAlignmentNone))
    {
        frame->m_image = applyAlignment(frame->m_image);
    }

    if (m_nextStage) {
        m_nextStage->submitFrame(frame);
    }
    PROFILER_STOP(__FUNCTION__);
}

cv::Mat CameraFrameAligner::imageToWorkingMat(const QImage& input, bool& highBitDepthInput)
{
    highBitDepthInput = (input.format() == QImage::Format_RGBA64) || (input.format() == QImage::Format_RGBX64);

    if (highBitDepthInput)
    {
        cv::Mat frameMat(input.height(), input.width(), CV_16UC3);
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

        return frameMat;
    }

    const QImage rgb = (input.format() == QImage::Format_RGB888)
        ? input
        : input.convertToFormat(QImage::Format_RGB888);
    cv::Mat rgbMat(rgb.height(), rgb.width(), CV_8UC3,
                   const_cast<uchar*>(rgb.bits()),
                   static_cast<size_t>(rgb.bytesPerLine()));
    return rgbMat.clone();
}

QImage CameraFrameAligner::workingMatToImage(const cv::Mat& frameMat, bool highBitDepthInput)
{
    if (!highBitDepthInput)
    {
        QImage result(frameMat.cols, frameMat.rows, QImage::Format_RGB888);
        for (int row = 0; row < frameMat.rows; ++row) {
            std::memcpy(result.scanLine(row), frameMat.ptr(row), static_cast<size_t>(frameMat.cols * 3));
        }
        return result;
    }

    QImage result(frameMat.cols, frameMat.rows, QImage::Format_RGBA64);
    for (int y = 0; y < frameMat.rows; ++y)
    {
        QRgba64 *outputLine = reinterpret_cast<QRgba64*>(result.scanLine(y));
        const cv::Vec<uint16_t, 3> *inputLine = frameMat.ptr<cv::Vec<uint16_t, 3>>(y);

        for (int x = 0; x < frameMat.cols; ++x) {
            outputLine[x] = qRgba64(inputLine[x][0], inputLine[x][1], inputLine[x][2], 65535);
        }
    }

    return result;
}

QImage CameraFrameAligner::applyAlignment(const QImage& input)
{
    PROFILER_START();

    bool highBitDepthInput = false;
    cv::Mat frameMat = imageToWorkingMat(input, highBitDepthInput);

    if (!m_alignmentReferenceHistory.empty()
        && (m_alignmentReferenceHistory.front().size() != frameMat.size()
            || m_alignmentReferenceHistory.front().type() != frameMat.type()))
    {
        m_alignmentReferenceHistory.clear();
    }

    cv::Mat alignedFrameMat = alignFrame(frameMat);

    m_alignmentReferenceHistory.push_back(alignedFrameMat.clone());
    const int maxFrames = qBound(1, m_settings.m_stackFrameCount, 256);
    while (static_cast<int>(m_alignmentReferenceHistory.size()) > maxFrames) {
        m_alignmentReferenceHistory.pop_front();
    }

    return workingMatToImage(alignedFrameMat, highBitDepthInput);
}

cv::Mat CameraFrameAligner::alignFrame(const cv::Mat& frameMat) const
{
    if (m_alignmentReferenceHistory.empty()) {
        return frameMat.clone();
    }

    const cv::Mat& referenceFrame = m_alignmentReferenceHistory.front();
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

cv::Mat CameraFrameAligner::frameToAlignmentGray(const cv::Mat& frameMat) const
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

cv::Mat CameraFrameAligner::warpFrameAffine(const cv::Mat& frameMat, const cv::Mat& transform) const
{
    if (transform.empty()) {
        return frameMat.clone();
    }

    cv::Mat aligned;
    cv::warpAffine(frameMat, aligned, transform, frameMat.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    return aligned;
}

cv::Mat CameraFrameAligner::alignWithPhaseCorrelation(const cv::Mat& referenceFrame, const cv::Mat& targetFrame) const
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

std::vector<cv::Point2f> CameraFrameAligner::detectStarCentroids(const cv::Mat& grayFrame) const
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

cv::Mat CameraFrameAligner::alignWithStarCentroids(const cv::Mat& referenceFrame, const cv::Mat& targetFrame) const
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
