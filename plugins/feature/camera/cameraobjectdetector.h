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

#ifndef INCLUDE_FEATURE_CAMERAOBJECTDETECTOR_H_
#define INCLUDE_FEATURE_CAMERAOBJECTDETECTOR_H_

#include <QHash>
#include <QSet>
#include <vector>

#include <opencv2/dnn/dnn.hpp>

#include "cameradetectionhistoryentry.h"
#include "cameradetector.h"
#ifdef CAMERA_TENSORRT_YOLO
#include "camerayolotensorrt.h"
#endif

class Camera;

class CameraObjectDetector : public CameraDetectionStage
{
    Q_OBJECT
public:
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

    class MsgReportTensorRtConversion : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        bool isActive() const { return m_active; }
        const QString& getModelPath() const { return m_modelPath; }
        const QString& getEnginePath() const { return m_enginePath; }

        static MsgReportTensorRtConversion* create(bool active, const QString& modelPath, const QString& enginePath)
        {
            return new MsgReportTensorRtConversion(active, modelPath, enginePath);
        }

    private:
        bool m_active;
        QString m_modelPath;
        QString m_enginePath;

        MsgReportTensorRtConversion(bool active, const QString& modelPath, const QString& enginePath) :
            Message(),
            m_active(active),
            m_modelPath(modelPath),
            m_enginePath(enginePath)
        { }
    };

    explicit CameraObjectDetector(Camera *camera);
    ~CameraObjectDetector() override;
    void setMessageQueueToGUI(MessageQueue *messageQueue) { m_msgQueueToGUI = messageQueue; }
    void setMessageQueueToFeature(MessageQueue *messageQueue) { m_msgQueueToFeature = messageQueue; }

protected:
    bool handleStageMessage(const Message& cmd) override;
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false) override;
    void captureActiveChanged(bool active) override;
    void processNewFrame(const CameraPipelineFramePtr& frame) override;

private:
    struct PendingDisappearState
    {
        QDateTime m_firstMissing;
        QDateTime m_deadline;
    };

    Camera *m_camera;
    MessageQueue *m_msgQueueToGUI;
    MessageQueue *m_msgQueueToFeature;
    cv::dnn::Net m_yoloNet;
    cv::Size m_yoloInputSize;
#ifdef CAMERA_TENSORRT_YOLO
    CameraYoloTensorRt m_yoloTensorRt;
#endif
    // Last YOLO DNN target (as CameraSettings::YoloDnnTarget value cast to int) that we
    // successfully applied to m_yoloNet via setPreferable{Backend,Target}. -1 = not yet
    // applied (e.g. just after the net was loaded). We re-apply only when this differs
    // from the current setting, avoiding a per-frame call that can rebuild the network's
    // compute graph when targets change.
    int m_appliedYoloDnnTarget = -1;
    bool m_yoloBatchedInferenceSupported = true;
    QString m_yoloLoadedModelPath;
    QSet<QString> m_reportedErrorKeys;
    QStringList m_yoloLabels;
    QString m_yoloLoadedLabelsPath;
    QSet<QString> m_yoloIgnoredClassNames;
    QSet<QString> m_detectedObjectClasses;
    QHash<QString, PendingDisappearState> m_pendingDisappearStates;
    QHash<QString, CameraDetectionHistoryEntry> m_activeObjectDetectionHistory;
    QList<CameraDetectionHistoryEntry> m_completedObjectDetectionHistory;
    void runYoloDetections(const cv::Mat& bgrMat, const cv::Rect& roi, QVector<CameraPipelineDetection>& detections);
    void decodeYoloDetections(const cv::Mat& det, const cv::Rect& tileRect, int padX, int padY, float invScale,
        std::vector<cv::Rect>& boxes, std::vector<float>& scores, std::vector<int>& classIds) const;
    QVector<cv::Rect> makeYoloTiles(const cv::Rect& roi) const;
    void processObjectDetections(const QVector<CameraPipelineDetection>& detections, const QDateTime& now, CameraPipelineFrame& frame);
    void sendFirstObjectDetectionTarget(const QVector<CameraPipelineDetection>& detections, const CameraPipelineFrame& frame) const;
    [[nodiscard]] int findCompletedPlaybackHistoryIndex(const QString& className, const CameraPipelineFrame& frame) const;
    [[nodiscard]] int findCompletedPlaybackHistoryIndex(const CameraDetectionHistoryEntry& entry) const;
    void updateHistoryEntryPlayback(CameraDetectionHistoryEntry& entry, const CameraPipelineFrame& frame) const;
    void mergeCompletedHistoryEntry(const CameraDetectionHistoryEntry& entry);
    void clearObjectDetectionState();
    void clearObjectDetectionHistory();
    void reportObjectDetectionHistoryToGUI() const;
    void reportErrorToFeature(const QString& errorKey, const QString& title, const QString& errorMessage);
    [[nodiscard]] QList<CameraDetectionHistoryEntry> getObjectDetectionHistorySnapshot() const;
};

#endif // INCLUDE_FEATURE_CAMERAOBJECTDETECTOR_H_
