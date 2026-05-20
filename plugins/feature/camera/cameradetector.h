///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3, or (at your option) later.         //
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
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/dnn/dnn.hpp>
#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
#include <opencv2/cudabgsegm.hpp>
#endif
#if defined(CAMERA_OPENCV_CUDA_DETECTION) || defined(CAMERA_OPENCV_CUDA_MOTION_DETECTION)
#include <opencv2/cudafilters.hpp>
#endif

#include "util/message.h"
#include "util/messagequeue.h"
#include "cameradetectionhistoryentry.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"

class Camera;

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

class CameraMotionDiffDetector : public CameraDetectionStage
{
    Q_OBJECT
public:
    CameraMotionDiffDetector();
    ~CameraMotionDiffDetector() override;

protected:
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false) override;
    void captureActiveChanged(bool active) override;
    void processNewFrame(const CameraPipelineFramePtr& frame) override;

private:
    CameraPipelineFrame m_previousInputFrame;
    CameraPipelineFrame m_lastInputFrame;
    std::deque<cv::Mat> m_diffMaskHistory;
#ifdef CAMERA_OPENCV_CUDA_DETECTION
    std::deque<cv::cuda::GpuMat> m_cudaDiffMaskHistory;
    cv::cuda::Stream m_cudaDetectionStream;
    cv::Ptr<cv::cuda::Filter> m_cudaDiffOpenFilter;
    cv::Ptr<cv::cuda::Filter> m_cudaDiffDilationFilter;
    cv::Ptr<cv::cuda::Filter> m_cudaDiffCloseFilter;
    int m_cudaDiffOpenFilterSize;
    int m_cudaDiffOpenFilterType;
    int m_cudaDiffDilationFilterSize;
    int m_cudaDiffDilationFilterType;
    int m_cudaDiffCloseFilterSize;
    int m_cudaDiffCloseFilterType;
    cv::cuda::GpuMat m_cudaDiffExclusionMask;
    cv::Rect m_cudaDiffExclusionRoi;
    cv::Size m_cudaDiffExclusionWorkSize;
    QVector<QRect> m_cudaDiffExclusionRects;
#endif
    cv::Ptr<cv::BackgroundSubtractor> m_bgSubtractor;
    cv::Mat m_motionLastFgMaskRaw;
#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
    cv::Ptr<cv::cuda::BackgroundSubtractorMOG2> m_cudaBgSubtractor;
    cv::cuda::Stream m_cudaMotionStream;
    cv::cuda::GpuMat m_cudaMotionLastFgMaskRaw;
    cv::Ptr<cv::cuda::Filter> m_cudaMotionOpenFilter;
    cv::Ptr<cv::cuda::Filter> m_cudaMotionCloseFilter;
    int m_cudaMotionOpenFilterSize;
    int m_cudaMotionOpenFilterType;
    int m_cudaMotionCloseFilterSize;
    int m_cudaMotionCloseFilterType;
    cv::cuda::GpuMat m_cudaMotionExclusionMask;
    cv::Rect m_cudaMotionExclusionRoi;
    cv::Size m_cudaMotionExclusionWorkSize;
    QVector<QRect> m_cudaMotionExclusionRects;
#endif
    QVector<QRect> m_lastMotionBoxes;
    int m_motionPersistenceRemaining;
    int m_motionConfirmCount;

    void resetDetectionState();
    [[nodiscard]] cv::Ptr<cv::BackgroundSubtractor> createBackgroundSubtractor() const;
#ifdef CAMERA_OPENCV_CUDA_DETECTION
    [[nodiscard]] bool canUseCudaDetection() const;
    bool applyDiffMaskCuda(cv::Mat& bgrMat, const cv::cuda::GpuMat* bgrGpu, const cv::Rect& roi, const CameraPipelineFrame& diffReferenceFrame);
    [[nodiscard]] cv::Ptr<cv::cuda::Filter> cudaDiffOpenFilter(int inputType, int kernelSize);
    [[nodiscard]] cv::Ptr<cv::cuda::Filter> cudaDiffDilationFilter(int inputType, int kernelSize);
    [[nodiscard]] cv::Ptr<cv::cuda::Filter> cudaDiffCloseFilter(int inputType, int kernelSize);
    [[nodiscard]] const cv::cuda::GpuMat& cudaDiffExclusionMask(const cv::Rect& roi, const cv::Size& workSize);
#endif
#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
    [[nodiscard]] cv::Ptr<cv::cuda::BackgroundSubtractorMOG2> createCudaBackgroundSubtractor() const;
    [[nodiscard]] bool canUseCudaMotionDetection() const;
    void invalidateCudaMotionCaches();
    [[nodiscard]] const cv::cuda::GpuMat& cudaMotionExclusionMask(const cv::Rect& roi, const cv::Size& workSize);
    bool applyMotionDetectionCuda(const cv::Mat& bgrMat, const cv::cuda::GpuMat* bgrGpu, const cv::Rect& roi, QVector<QRect>& motionBoxes, bool updateBackgroundModel, cv::Mat* debugMask = nullptr);
#endif
    void applyDiffMask(CameraPipelineFrame& frame, cv::Mat& bgrMat, const cv::Rect& roi, const CameraPipelineFrame& diffReferenceFrame);
    void applyMotionDetection(const cv::Mat& bgrMat, const cv::cuda::GpuMat* bgrGpu, const cv::Rect& roi, QVector<QRect>& motionBoxes, bool updateBackgroundModel, cv::Mat* debugMask = nullptr);
};

class CameraStarDetector : public CameraDetectionStage
{
    Q_OBJECT
public:
    CameraStarDetector();
    ~CameraStarDetector() override;

protected:
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false) override;
    void captureActiveChanged(bool active) override;
    void processNewFrame(const CameraPipelineFramePtr& frame) override;

private:
#ifdef CAMERA_OPENCV_CUDA_DETECTION
    mutable cv::cuda::Stream m_cudaDetectionStream;
    mutable cv::Ptr<cv::cuda::Filter> m_cudaStarSmallBlurFilter;
    mutable cv::Ptr<cv::cuda::Filter> m_cudaStarBackgroundBlurFilter;
    mutable int m_cudaStarSmallBlurFilterType;
    mutable int m_cudaStarBackgroundBlurFilterType;
    mutable int m_cudaStarBackgroundBlurFilterSize;
    mutable cv::cuda::GpuMat m_cudaStarExclusionMask;
    mutable cv::Rect m_cudaStarExclusionRoi;
    mutable cv::Size m_cudaStarExclusionWorkSize;
    mutable QVector<QRect> m_cudaStarExclusionRects;

    [[nodiscard]] bool canUseCudaDetection() const;
    [[nodiscard]] cv::Ptr<cv::cuda::Filter> cudaStarSmallBlurFilter(int inputType) const;
    [[nodiscard]] cv::Ptr<cv::cuda::Filter> cudaStarBackgroundBlurFilter(int inputType, int kernelSize) const;
    [[nodiscard]] const cv::cuda::GpuMat& cudaStarExclusionMask(const cv::Rect& roi, const cv::Size& workSize) const;
    bool applyStarPreprocessingCuda(const cv::Mat& bgrMat, const cv::cuda::GpuMat* bgrGpu, const cv::Rect& roi, cv::Mat& gray, cv::Mat& residual, cv::Mat& thresholdMask, cv::Mat* debugMask) const;
#endif
    void applyStarDetection(const cv::Mat& bgrMat, const cv::cuda::GpuMat* bgrGpu, const cv::Rect& roi, QVector<CameraPipelineStarDetection>& starDetections, cv::Mat* debugMask = nullptr) const;
    void applyStarPreprocessing(const cv::Mat& bgrMat, const cv::cuda::GpuMat* bgrGpu, const cv::Rect& roi, cv::Mat& gray, cv::Mat& residual, cv::Mat& thresholdMask, cv::Mat* debugMask) const;
};

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

    void runYoloDetections(const cv::Mat& bgrMat, const cv::Rect& roi, QVector<CameraPipelineDetection>& detections);
    void processObjectDetections(const QVector<CameraPipelineDetection>& detections, const QDateTime& now, CameraPipelineFrame& frame);
    void clearObjectDetectionState(bool clearHistory = true);
    void clearObjectDetectionHistory();
    void reportObjectDetectionHistoryToGUI() const;
    void reportErrorToFeature(const QString& errorKey, const QString& title, const QString& errorMessage);
    [[nodiscard]] QList<CameraDetectionHistoryEntry> getObjectDetectionHistorySnapshot() const;
    bool applyObjectDetectedSettings(const QString& className, const QDateTime& now);
    void applyObjectDisappearedSettings(const QString& className, const QDateTime& now);
    void sendEvent(const QString& className, bool detected, const QDateTime& eventTime);
    void executeCommand(const QString& command, const QString& className);
    void saySpeech(const QString& speech, const QString& className);
    bool shouldRecordVideoForDetectedObjects() const;
    void setVideoRecordingEnabled(bool enabled);
};

#endif // INCLUDE_FEATURE_CAMERADETECTOR_H_
