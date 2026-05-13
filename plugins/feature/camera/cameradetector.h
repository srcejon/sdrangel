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
#include <deque>
#include <QHash>
#include <QSet>
#ifdef QT_TEXTTOSPEECH_FOUND
#include <QTextToSpeech>
#endif

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/dnn/dnn.hpp>

#include "util/message.h"
#include "util/messagequeue.h"
#include "cameradetectionhistoryentry.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"

class CameraPostProcessor;

class CameraDetector : public QObject
{
    Q_OBJECT
public:
    class MsgConfigureCameraDetector : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const CameraSettings& getSettings() const { return m_settings; }
        const QList<QString>& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureCameraDetector* create(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
        {
            return new MsgConfigureCameraDetector(settings, settingsKeys, force);
        }

    private:
        CameraSettings m_settings;
        QList<QString> m_settingsKeys;
        bool m_force;

        MsgConfigureCameraDetector(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force) :
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

    class MsgReportObjectDetectionHistory : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QList<CameraDetectionHistoryEntry>& getHistory() const { return m_history; }

        static MsgReportObjectDetectionHistory* create(const QList<CameraDetectionHistoryEntry>& history)
        {
            return new MsgReportObjectDetectionHistory(history);
        }

    private:
        QList<CameraDetectionHistoryEntry> m_history;

        MsgReportObjectDetectionHistory(const QList<CameraDetectionHistoryEntry>& history) :
            Message(),
            m_history(history)
        { }
    };

    class MsgClearObjectDetectionHistory : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        static MsgClearObjectDetectionHistory* create()
        {
            return new MsgClearObjectDetectionHistory();
        }

    private:
        MsgClearObjectDetectionHistory() : Message() {}
    };

    CameraDetector();
    ~CameraDetector();

    void startWork();
    void stopWork();
    void submitFrame(const CameraPipelineFramePtr& frame);
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setNextStage(CameraPostProcessor *nextStage) { m_nextStage = nextStage; }
    void setMessageQueueToGUI(MessageQueue *messageQueue) { m_msgQueueToGUI = messageQueue; }
    void setMessageQueueToFeature(MessageQueue *messageQueue) { m_msgQueueToFeature = messageQueue; }

private:
    struct PendingDisappearState
    {
        QDateTime m_firstMissing;
        QDateTime m_deadline;
    };

    MessageQueue m_inputMessageQueue;
    MessageQueue *m_msgQueueToGUI;
    MessageQueue *m_msgQueueToFeature;
    CameraPostProcessor *m_nextStage;
    CameraSettings m_settings;
    bool m_captureActive;
    CameraPipelineFrame m_previousInputFrame;
    CameraPipelineFrame m_lastInputFrame;
    std::deque<cv::Mat> m_diffMaskHistory;
    cv::Ptr<cv::BackgroundSubtractor> m_bgSubtractor;
    cv::Ptr<cv::BackgroundSubtractor> m_streakBgSubtractor;
    cv::Mat m_motionLastFgMaskRaw;
    cv::Mat m_streakLastBackgroundGray;
    cv::Mat m_streakLastForegroundMask;
    QVector<QRect> m_lastMotionBoxes;
    int m_motionPersistenceRemaining;
    int m_motionConfirmCount;
    QVector<CameraPipelineStreakDetection> m_lastStreakDetections;
    int m_streakPersistenceRemaining;
    cv::dnn::Net m_yoloNet;
    cv::Size m_yoloInputSize;
    QString m_yoloLoadedModelPath;
    QSet<QString> m_reportedErrorKeys;
    QStringList m_yoloLabels;
    QString m_yoloLoadedLabelsPath;
    QSet<QString> m_detectedObjectClasses;
    QHash<QString, PendingDisappearState> m_pendingDisappearStates;
    QHash<QString, CameraDetectionHistoryEntry> m_activeObjectDetectionHistory;
    QList<CameraDetectionHistoryEntry> m_completedObjectDetectionHistory;
#ifdef QT_TEXTTOSPEECH_FOUND
    QTextToSpeech *m_speech;
#endif
    QMutex m_frameMutex;
    CameraPipelineFramePtr m_pendingFrame;
    bool m_processingFrame;

    bool handleMessage(const Message& cmd);
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    void reprocessLastFrame();
    void processNewFrame(const CameraPipelineFramePtr& frame);
    void processFrame(const CameraPipelineFramePtr& frame, const CameraPipelineFrame& diffReferenceFrame, bool updateInputHistory);
    [[nodiscard]] cv::Rect resolveDetectionRoi(const cv::Size& frameSize) const;
    [[nodiscard]] cv::Ptr<cv::BackgroundSubtractor> createBackgroundSubtractor() const;
    [[nodiscard]] cv::Ptr<cv::BackgroundSubtractor> createStreakBackgroundSubtractor() const;
    [[nodiscard]] cv::Mat applyStreakLineEnhancement(const cv::Mat& grayMat) const;
    void applyDiffMask(cv::Mat& bgrMat, const cv::Rect& roi, const CameraPipelineFrame& diffReferenceFrame);
    void applyMotionDetection(const cv::Mat& bgrMat, const cv::Rect& roi, QVector<QRect>& motionBoxes, bool updateBackgroundModel, cv::Mat* debugMask = nullptr);
    void applyStreakDetection(const cv::Mat& bgrMat, const cv::Rect& roi, QVector<CameraPipelineStreakDetection>& streakDetections, bool updateBackgroundModel, cv::Mat* debugMask = nullptr);
    void applyStarDetection(const cv::Mat& bgrMat, const cv::Rect& roi, QVector<CameraPipelineStarDetection>& starDetections, cv::Mat* debugMask = nullptr) const;
    [[nodiscard]] cv::Mat buildExclusionMask(const cv::Rect& roi, const cv::Size& workSize) const;
    [[nodiscard]] bool intersectsExclusionRects(const QRect& rect) const;
    void runYoloDetections(const cv::Mat& bgrMat, const cv::Rect& roi, QVector<CameraPipelineDetection>& detections);
    void processObjectDetections(const QVector<CameraPipelineDetection>& detections, const QDateTime& now, CameraPipelineFrame& frame);
    void clearObjectDetectionState();
    void clearObjectDetectionHistory();
    void reportObjectDetectionHistoryToGUI() const;
    void reportErrorToFeature(const QString& errorKey, const QString& title, const QString& errorMessage);
    [[nodiscard]] QList<CameraDetectionHistoryEntry> getObjectDetectionHistorySnapshot() const;
    bool applyObjectDetectedSettings(const QString& className);
    void applyObjectDisappearedSettings(const QString& className);
    void executeCommand(const QString& command, const QString& className);
    void saySpeech(const QString& speech, const QString& className);
    bool shouldRecordVideoForDetectedObjects() const;
    void setVideoRecordingEnabled(bool enabled);
    [[nodiscard]] static const QImage& ensureRgb888(const QImage& image, QImage& convertedImage);
    [[nodiscard]] static cv::Mat wrapRgb888Image(const QImage& image);
    [[nodiscard]] static QImage convertBgrToRgbImage(const cv::Mat& bgrMat);

private slots:
    void handleInputMessages();
    void processNextFrame();
};

#endif // INCLUDE_FEATURE_CAMERADETECTOR_H_
