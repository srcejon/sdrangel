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

#include <QObject>
#include <QMutex>

#include <opencv2/core/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "util/message.h"
#include "util/messagequeue.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"

class CameraDetectionStage : public QObject
{
    Q_OBJECT
public:
    class MsgConfigureCameraDetectionStage : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const CameraSettings& getSettings() const { return m_settings; }
        const QList<QString>& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureCameraDetectionStage* create(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
        {
            return new MsgConfigureCameraDetectionStage(settings, settingsKeys, force);
        }

    private:
        CameraSettings m_settings;
        QList<QString> m_settingsKeys;
        bool m_force;

        MsgConfigureCameraDetectionStage(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force) :
            Message(),
            m_settings(settings),
            m_settingsKeys(settingsKeys),
            m_force(force)
        { }
    };

    class MsgProcessFrame : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const CameraPipelineFramePtr& getFrame() const { return m_frame; }

        static MsgProcessFrame* create(const CameraPipelineFramePtr& frame)
        {
            return new MsgProcessFrame(frame);
        }

    private:
        CameraPipelineFramePtr m_frame;

        MsgProcessFrame(const CameraPipelineFramePtr& frame) :
            Message(),
            m_frame(frame)
        { }
    };

    class MsgCaptureActive : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        bool isActive() const { return m_active; }

        static MsgCaptureActive* create(bool active)
        {
            return new MsgCaptureActive(active);
        }

    private:
        bool m_active;

        MsgCaptureActive(bool active) :
            Message(),
            m_active(active)
        { }
    };

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
    QMutex m_frameMutex;
    CameraPipelineFramePtr m_pendingFrame;
    bool m_processingFrame;

    virtual bool handleStageMessage(const Message& cmd);
    virtual void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    virtual void captureActiveChanged(bool active);
    virtual void processNewFrame(const CameraPipelineFramePtr& frame) = 0;
    void forwardFrame(const CameraPipelineFramePtr& frame);
    [[nodiscard]] cv::Rect resolveDetectionRoi(const cv::Size& frameSize) const;
    [[nodiscard]] cv::Mat buildExclusionMask(const cv::Rect& roi, const cv::Size& workSize) const;
    [[nodiscard]] const cv::Mat& cachedExclusionMask(const cv::Rect& roi, const cv::Size& workSize) const;
    [[nodiscard]] bool intersectsExclusionRects(const QRect& rect) const;
    [[nodiscard]] static const QImage& ensureRgb888(const QImage& image, QImage& convertedImage);
    [[nodiscard]] static cv::Mat wrapRgb888Image(const QImage& image);
    [[nodiscard]] static QImage convertBgrToRgbImage(const cv::Mat& bgrMat);

private:
    mutable cv::Mat m_exclusionMask;
    mutable cv::Rect m_exclusionMaskRoi;
    mutable cv::Size m_exclusionMaskWorkSize;
    mutable QVector<QRect> m_exclusionMaskRects;

    bool handleMessage(const Message& cmd);

private slots:
    void handleInputMessages();
    void processNextFrame();
};

#endif // INCLUDE_FEATURE_CAMERADETECTOR_H_
