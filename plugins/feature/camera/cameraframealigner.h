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

#ifndef INCLUDE_FEATURE_CAMERAFRAMEALIGNER_H_
#define INCLUDE_FEATURE_CAMERAFRAMEALIGNER_H_

#include <QObject>
#include <QMutex>
#include <deque>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "util/message.h"
#include "util/messagequeue.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"

class CameraFrameStacker;

class CameraFrameAligner : public QObject
{
    Q_OBJECT
public:
    class MsgConfigureCameraFrameAligner : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const CameraSettings& getSettings() const { return m_settings; }
        const QList<QString>& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureCameraFrameAligner* create(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
        {
            return new MsgConfigureCameraFrameAligner(settings, settingsKeys, force);
        }

    private:
        CameraSettings m_settings;
        QList<QString> m_settingsKeys;
        bool m_force;

        MsgConfigureCameraFrameAligner(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force) :
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

    CameraFrameAligner();
    ~CameraFrameAligner();

    void startWork();
    void stopWork();
    void submitFrame(const CameraPipelineFramePtr& frame);
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setNextStage(CameraFrameStacker *nextStage) { m_nextStage = nextStage; }

private:
    MessageQueue m_inputMessageQueue;
    CameraFrameStacker *m_nextStage;
    CameraSettings m_settings;
    bool m_captureActive;
    std::deque<cv::Mat> m_alignmentReferenceHistory;
    QMutex m_frameMutex;
    std::deque<CameraPipelineFramePtr> m_pendingFrames;
    bool m_processingFrame;

    bool handleMessage(const Message& cmd);
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    void processNewFrame(const CameraPipelineFramePtr& frame);
    bool preserveFrameOrder() const;
    int pendingFrameLimit() const;
    void resetAlignmentState();
    void trimAlignmentHistoryToCurrentLimit();
    [[nodiscard]] QImage applyAlignment(const QImage& input);
    [[nodiscard]] static cv::Mat imageToWorkingMat(const QImage& input, bool& highBitDepthInput);
    [[nodiscard]] static QImage workingMatToImage(const cv::Mat& frameMat, bool highBitDepthInput);
    [[nodiscard]] cv::Mat alignFrame(const cv::Mat& frameMat) const;
    [[nodiscard]] cv::Mat alignWithPhaseCorrelation(const cv::Mat& referenceFrame, const cv::Mat& targetFrame) const;
    [[nodiscard]] cv::Mat alignWithStarCentroids(const cv::Mat& referenceFrame, const cv::Mat& targetFrame) const;
    [[nodiscard]] cv::Mat warpFrameAffine(const cv::Mat& frameMat, const cv::Mat& transform) const;
    [[nodiscard]] cv::Mat frameToAlignmentGray(const cv::Mat& frameMat) const;
    [[nodiscard]] std::vector<cv::Point2f> detectStarCentroids(const cv::Mat& grayFrame) const;

private slots:
    void handleInputMessages();
    void processNextFrame();
};

#endif // INCLUDE_FEATURE_CAMERAFRAMEALIGNER_H_
