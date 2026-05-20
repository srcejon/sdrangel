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

#include <QDebug>

#include "util/profiler.h"
#include "cameradetector.h"

MESSAGE_CLASS_DEFINITION(CameraDetectionStage::MsgConfigureCameraDetectionStage, Message)
MESSAGE_CLASS_DEFINITION(CameraDetectionStage::MsgProcessFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraDetectionStage::MsgCaptureActive, Message)
CameraDetectionStage::CameraDetectionStage() :
    m_nextStageQueue(nullptr),
    m_captureActive(false),
    m_processingFrame(false)
{
}

CameraDetectionStage::~CameraDetectionStage() = default;

void CameraDetectionStage::startWork()
{
    QObject::connect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraDetectionStage::handleInputMessages);
    handleInputMessages();
}

void CameraDetectionStage::stopWork()
{
    QObject::disconnect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraDetectionStage::handleInputMessages);
}

bool CameraDetectionStage::handleMessage(const Message& cmd)
{
    if (MsgConfigureCameraDetectionStage::match(cmd))
    {
        const MsgConfigureCameraDetectionStage& cfg = (const MsgConfigureCameraDetectionStage&) cmd;
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
        captureActiveChanged(m_captureActive);
        QMutexLocker locker(&m_frameMutex);
        m_pendingFrame.reset();
        if (!m_captureActive) {
            m_processingFrame = false;
        }
        return true;
    }

    return handleStageMessage(cmd);
}

bool CameraDetectionStage::handleStageMessage(const Message& cmd)
{
    (void) cmd;
    return false;
}

void CameraDetectionStage::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }
}

void CameraDetectionStage::captureActiveChanged(bool active)
{
    (void) active;
}

void CameraDetectionStage::handleInputMessages()
{
    Message *message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void CameraDetectionStage::forwardFrame(const CameraPipelineFramePtr& frame)
{
    if (m_nextStageQueue) {
        m_nextStageQueue->push(MsgProcessFrame::create(frame));
    }
}


void CameraDetectionStage::submitFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame) {
        return;
    }

    bool schedule = false;
    {
        QMutexLocker locker(&m_frameMutex);
        if (m_pendingFrame) {
            qDebug() << "CameraDetectionStage: Dropping pending frame in favor of new frame";
        }
        m_pendingFrame = frame;
        if (!m_processingFrame)
        {
            m_processingFrame = true;
            schedule = true;
        }
    }

    if (schedule) {
        QMetaObject::invokeMethod(this, &CameraDetectionStage::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraDetectionStage::processNextFrame()
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
        QMetaObject::invokeMethod(this, &CameraDetectionStage::processNextFrame, Qt::QueuedConnection);
    }
}

cv::Rect CameraDetectionStage::resolveDetectionRoi(const cv::Size& frameSize) const
{
    const int frameWidth = std::max(1, frameSize.width);
    const int frameHeight = std::max(1, frameSize.height);
    const int x = qBound(0, m_settings.m_detectionRoiX, frameWidth - 1);
    const int y = qBound(0, m_settings.m_detectionRoiY, frameHeight - 1);
    const int width = (m_settings.m_detectionRoiWidth <= 0)
        ? (frameWidth - x)
        : qBound(1, m_settings.m_detectionRoiWidth, frameWidth - x);
    const int height = (m_settings.m_detectionRoiHeight <= 0)
        ? (frameHeight - y)
        : qBound(1, m_settings.m_detectionRoiHeight, frameHeight - y);
    return cv::Rect(x, y, width, height);
}


cv::Mat CameraDetectionStage::buildExclusionMask(const cv::Rect& roi, const cv::Size& workSize) const
{
    cv::Mat mask(workSize, CV_8UC1, cv::Scalar(255));

    if (m_settings.m_motionExclusionRects.isEmpty()) {
        return mask;
    }

    const double scaleX = static_cast<double>(workSize.width) / std::max(1, roi.width);
    const double scaleY = static_cast<double>(workSize.height) / std::max(1, roi.height);

    for (const QRect& rect : m_settings.m_motionExclusionRects)
    {
        const QRect intersected = rect.intersected(QRect(roi.x, roi.y, roi.width, roi.height));
        if (intersected.isEmpty()) {
            continue;
        }

        const int x0 = std::clamp(static_cast<int>(std::floor((intersected.left() - roi.x) * scaleX)), 0, std::max(0, workSize.width - 1));
        const int y0 = std::clamp(static_cast<int>(std::floor((intersected.top() - roi.y) * scaleY)), 0, std::max(0, workSize.height - 1));
        const int x1 = std::clamp(static_cast<int>(std::ceil((intersected.right() + 1 - roi.x) * scaleX)), x0 + 1, workSize.width);
        const int y1 = std::clamp(static_cast<int>(std::ceil((intersected.bottom() + 1 - roi.y) * scaleY)), y0 + 1, workSize.height);

        cv::rectangle(mask, cv::Rect(x0, y0, x1 - x0, y1 - y0), cv::Scalar(0), cv::FILLED);
    }

    return mask;
}

const cv::Mat& CameraDetectionStage::cachedExclusionMask(const cv::Rect& roi, const cv::Size& workSize) const
{
    if (m_exclusionMask.empty()
        || (m_exclusionMaskRoi != roi)
        || (m_exclusionMaskWorkSize != workSize)
        || (m_exclusionMaskRects != m_settings.m_motionExclusionRects))
    {
        m_exclusionMask = buildExclusionMask(roi, workSize);
        m_exclusionMaskRoi = roi;
        m_exclusionMaskWorkSize = workSize;
        m_exclusionMaskRects = m_settings.m_motionExclusionRects;
    }

    return m_exclusionMask;
}

bool CameraDetectionStage::intersectsExclusionRects(const QRect& rect) const
{
    for (const QRect& exclusionRect : m_settings.m_motionExclusionRects)
    {
        if (rect.intersects(exclusionRect)) {
            return true;
        }
    }

    return false;
}

const QImage& CameraDetectionStage::ensureRgb888(const QImage& image, QImage& convertedImage)
{
    if (image.format() == QImage::Format_RGB888) {
        return image;
    }

    convertedImage = image.convertToFormat(QImage::Format_RGB888);
    return convertedImage;
}

cv::Mat CameraDetectionStage::wrapRgb888Image(const QImage& image)
{
    return cv::Mat(image.height(), image.width(), CV_8UC3,
                   const_cast<uchar*>(image.constBits()),
                   static_cast<size_t>(image.bytesPerLine()));
}

QImage CameraDetectionStage::convertBgrToRgbImage(const cv::Mat& bgrMat)
{
    PROFILER_START();
    QImage result(bgrMat.cols, bgrMat.rows, QImage::Format_RGB888);
    cv::Mat rgbMat(result.height(), result.width(), CV_8UC3,
                   result.bits(),
                   static_cast<size_t>(result.bytesPerLine()));
    cv::cvtColor(bgrMat, rgbMat, cv::COLOR_BGR2RGB);
    return result;
}

