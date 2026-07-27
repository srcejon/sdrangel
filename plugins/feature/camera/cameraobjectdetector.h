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
#include "camerameteorphotometer.h"
#include "camerayololitert.h"
#ifdef CAMERA_TENSORRT_YOLO
#include "camerayolotensorrt.h"
#endif

class Camera;

/**
 * \brief Detection stage that runs YOLO object detection and tracks detected objects.
 *
 * Runs a YOLO model over the detection ROI (tiling large frames), decoding and NMS-filtering
 * the raw outputs into CameraPipelineFrame::m_detections. Scale and tile inference can use
 * separate model states; ONNX models use OpenCV DNN or TensorRT, while Android .tflite models
 * use LiteRT CPU or its GPU delegate. Detections are then turned into a
 * per-class appearance/disappearance history (with debounce deadlines) reported to the GUI, and
 * the first detection can be forwarded to the feature as an azimuth/elevation target.
 *
 * \note Derives from CameraDetectionStage and runs on its own QThread; see that base class for
 *       threading, frame-backlog and ROI/exclusion handling.
 * \note Holds a back-pointer to the owning Camera plus message queues to the GUI and feature.
 *       Runtime state is loaded/rebuilt lazily when the model, labels or target settings
 *       change; ignored class names are filtered out of results.
 */
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
    [[nodiscard]] static bool isVulkanDnnAvailable();
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
    };

    Camera *m_camera;
    MessageQueue *m_msgQueueToGUI;
    MessageQueue *m_msgQueueToFeature;

    struct YoloModelState
    {
        cv::dnn::Net m_net;
        cv::Size m_inputSize = cv::Size(640, 640);
        CameraYoloLiteRt m_liteRt;
#ifdef CAMERA_TENSORRT_YOLO
        CameraYoloTensorRt m_tensorRt;
#endif
        // Last YOLO DNN target (as CameraSettings::YoloDnnTarget value cast to int) that
        // was applied via setPreferable{Backend,Target}. -1 = not yet applied.
        int m_appliedDnnTarget = -1;
        int m_failedDnnTarget = -1;
        bool m_batchedInferenceSupported = false;
        QString m_loadedModelPath;
    };

    YoloModelState m_yoloScaleModel;
    YoloModelState m_yoloTileModel;
    QSet<QString> m_reportedErrorKeys;
    QStringList m_yoloLabels;
    QString m_yoloLoadedLabelsPath;
    QSet<QString> m_yoloIgnoredClassNames;
    QSet<QString> m_detectedObjectClasses;
    QHash<QString, PendingDisappearState> m_pendingDisappearStates;
    QHash<QString, CameraDetectionHistoryEntry> m_activeObjectDetectionHistory;
    QList<CameraDetectionHistoryEntry> m_completedObjectDetectionHistory;
    CameraPipelineFrame m_lastInputFrame;
    CameraMeteorPhotometer m_meteorPhotometer;
    enum class YoloBackend {
        OpenCv,
        TensorRt,
        LiteRtCpu,
        LiteRtGpu
    };
    void runYoloDetections(const cv::Mat& bgrMat, const cv::Rect& roi, QVector<CameraPipelineDetection>& detections);
    void resetYoloModelState(YoloModelState& modelState);
    [[nodiscard]] YoloBackend yoloBackendForModel(const QString& modelPath) const;
    bool ensureYoloModelLoaded(YoloModelState& modelState, const QString& modelPath, YoloBackend backend);
    bool forwardYoloOpenCv(YoloModelState& modelState, const cv::Mat& blob, std::vector<cv::Mat>& outputs, QString& errorMessage);
    bool runYoloModelDetections(YoloModelState& modelState, const QString& modelPath, const cv::Mat& bgrMat,
        const QVector<cv::Rect>& inferenceRects, std::vector<cv::Rect>& boxes, std::vector<float>& scores, std::vector<int>& classIds,
        YoloBackend backend);
    void decodeYoloDetections(const cv::Mat& det, const cv::Rect& tileRect, const cv::Size& modelInputSize, int padX, int padY, float invScale,
        std::vector<cv::Rect>& boxes, std::vector<float>& scores, std::vector<int>& classIds) const;
    QVector<cv::Rect> makeYoloTiles(const cv::Rect& roi, const cv::Size& inputSize) const;
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
