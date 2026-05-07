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

#ifndef INCLUDE_FEATURE_CAMERAFRAMESTACKER_H_
#define INCLUDE_FEATURE_CAMERAFRAMESTACKER_H_

#include <QObject>
#include <deque>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>

#include "util/message.h"
#include "util/messagequeue.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"

class CameraFrameStacker : public QObject
{
    Q_OBJECT
public:
    class MsgConfigureCameraFrameStacker : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const CameraSettings& getSettings() const { return m_settings; }
        const QList<QString>& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureCameraFrameStacker* create(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
        {
            return new MsgConfigureCameraFrameStacker(settings, settingsKeys, force);
        }

    private:
        CameraSettings m_settings;
        QList<QString> m_settingsKeys;
        bool m_force;

        MsgConfigureCameraFrameStacker(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force) :
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

    CameraFrameStacker();
    ~CameraFrameStacker();

    void startWork();
    void stopWork();
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setNextStageInputMessageQueue(MessageQueue *messageQueue) { m_nextStageInputMessageQueue = messageQueue; }

private:
    MessageQueue m_inputMessageQueue;
    MessageQueue *m_nextStageInputMessageQueue;
    CameraSettings m_settings;
    bool m_captureActive;
    std::deque<cv::Mat> m_stackFrameHistory;
    cv::Mat m_stackAccumulator;

    bool handleMessage(const Message& cmd);
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    void processNewFrame(const CameraPipelineFramePtr& frame);
    void resetFrameHistoryState();
    [[nodiscard]] QImage applyFrameStacking(const QImage& input);
    [[nodiscard]] cv::Mat alignStackFrame(const cv::Mat& frameMat) const;
    [[nodiscard]] cv::Mat alignWithPhaseCorrelation(const cv::Mat& referenceFrame, const cv::Mat& targetFrame) const;
    [[nodiscard]] cv::Mat alignWithStarCentroids(const cv::Mat& referenceFrame, const cv::Mat& targetFrame) const;
    [[nodiscard]] cv::Mat warpFrameAffine(const cv::Mat& frameMat, const cv::Mat& transform) const;
    [[nodiscard]] cv::Mat frameToAlignmentGray(const cv::Mat& frameMat) const;
    [[nodiscard]] std::vector<cv::Point2f> detectStarCentroids(const cv::Mat& grayFrame) const;

private slots:
    void handleInputMessages();
};

#endif // INCLUDE_FEATURE_CAMERAFRAMESTACKER_H_
