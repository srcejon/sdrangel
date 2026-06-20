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

#ifndef INCLUDE_FEATURE_CAMERADETECTOR_H_
#define INCLUDE_FEATURE_CAMERADETECTOR_H_

#include <deque>

#include <QObject>
#include <QMutex>

#include <opencv2/core/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "util/message.h"
#include "util/messagequeue.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"

/**
 * \brief Base class for a detection stage in the camera processing pipeline.
 *
 * Each concrete detector (motion/star/object/diff) derives from this and implements
 * processNewFrame() to analyse a CameraPipelineFramePtr, then calls forwardFrame() to pass it
 * to the next stage. The base class provides the plumbing common to all stages: an input
 * MessageQueue, settings handling, capture-active gating, a bounded backlog of pending frames,
 * and shared helpers for resolving the detection ROI, building/caching the exclusion mask and
 * converting between QImage and OpenCV BGR/RGB mats.
 *
 * \note A stage is a QObject that lives on its own QThread; frames arrive as queued messages
 *       and are dispatched to processNewFrame() one at a time via processNextFrame(). The
 *       pending-frame deque is bounded to m_maxPendingFrames and protected by m_frameMutex;
 *       under sustained overrun the oldest frame is dropped to keep latency bounded.
 * \note Stages are chained with setNextStage()/setNextStageInputMessageQueue(); frames are only
 *       forwarded while capture is active and the frame's capture epoch matches the current one.
 * \warning Subclasses must override the pure-virtual processNewFrame(); applySettings(),
 *          captureActiveChanged() and handleStageMessage() may be overridden but should call the
 *          base implementation where appropriate.
 */
class CameraDetectionStage : public QObject
{
    Q_OBJECT
public:


    CameraDetectionStage();
    ~CameraDetectionStage() override;

    void startWork();
    void stopWork();
    void submitFrame(const CameraPipelineFramePtr& frame);
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setNextStage(CameraDetectionStage *nextStage) { m_nextStageQueue = nextStage ? nextStage->getInputMessageQueue() : nullptr; }
    void setNextStageInputMessageQueue(MessageQueue *messageQueue) { m_nextStageQueue = messageQueue; }

protected:
    MessageQueue m_inputMessageQueue;
    MessageQueue *m_nextStageQueue;
    CameraSettings m_settings;
    bool m_captureActive;
    quint64 m_captureEpoch = 0;
    QMutex m_frameMutex;
    // Small bounded backlog of frames waiting to be processed, so an occasional
    // processing spike is absorbed instead of costing a frame; a sustained
    // overrun trims the oldest frame so latency stays bounded.
    static constexpr int m_maxPendingFrames = 3;
    std::deque<CameraPipelineFramePtr> m_pendingFrames;
    bool m_processingFrame;

    virtual bool handleStageMessage(const Message& cmd);
    virtual void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    virtual void captureActiveChanged(bool active);
    virtual void processNewFrame(const CameraPipelineFramePtr& frame) = 0;
    void forwardFrame(const CameraPipelineFramePtr& frame);
    [[nodiscard]] cv::Rect resolveDetectionRoi(const cv::Size& frameSize) const;
    [[nodiscard]] cv::Mat buildExclusionMask(const cv::Rect& roi, const cv::Size& workSize) const;
    [[nodiscard]] const cv::Mat& cachedExclusionMask(const cv::Rect& roi, const cv::Size& workSize);
    [[nodiscard]] bool intersectsExclusionRects(const QRect& rect) const;
    [[nodiscard]] static const QImage& ensureRgb888(const QImage& image, QImage& convertedImage);
    [[nodiscard]] static cv::Mat wrapRgb888Image(const QImage& image);
    [[nodiscard]] static QImage convertBgrToRgbImage(const cv::Mat& bgrMat);

private:
    cv::Mat m_exclusionMask;
    cv::Rect m_exclusionMaskRoi;
    cv::Size m_exclusionMaskWorkSize;
    QList<QRect> m_exclusionMaskRects;

    bool handleMessage(const Message& cmd);

private slots:
    void handleInputMessages();
    void processNextFrame();
};

#endif // INCLUDE_FEATURE_CAMERADETECTOR_H_
