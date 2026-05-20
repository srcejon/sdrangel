///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3, or (at your option) later.         //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_FEATURE_CAMERAMOTIONDETECTOR_H_
#define INCLUDE_FEATURE_CAMERAMOTIONDETECTOR_H_

#include <opencv2/video/background_segm.hpp>
#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
#include <opencv2/cudabgsegm.hpp>
#include <opencv2/cudafilters.hpp>
#endif

#include "cameradetector.h"

class CameraMotionDetector : public CameraDetectionStage
{
    Q_OBJECT
public:
    CameraMotionDetector();
    ~CameraMotionDetector() override;

protected:
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false) override;
    void captureActiveChanged(bool active) override;
    void processNewFrame(const CameraPipelineFramePtr& frame) override;

private:
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

    [[nodiscard]] cv::Ptr<cv::BackgroundSubtractor> createBackgroundSubtractor() const;
#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
    [[nodiscard]] cv::Ptr<cv::cuda::BackgroundSubtractorMOG2> createCudaBackgroundSubtractor() const;
    [[nodiscard]] bool canUseCudaMotionDetection() const;
    void invalidateCudaMotionCaches();
    [[nodiscard]] const cv::cuda::GpuMat& cudaMotionExclusionMask(const cv::Rect& roi, const cv::Size& workSize);
    bool applyMotionDetectionCuda(const cv::Mat& bgrMat, const cv::cuda::GpuMat* bgrGpu, const cv::Rect& roi, QVector<QRect>& motionBoxes, bool updateBackgroundModel, cv::Mat* debugMask = nullptr);
#endif
    void applyMotionDetection(const cv::Mat& bgrMat, const cv::cuda::GpuMat* bgrGpu, const cv::Rect& roi, QVector<QRect>& motionBoxes, bool updateBackgroundModel, cv::Mat* debugMask = nullptr);
};

#endif // INCLUDE_FEATURE_CAMERAMOTIONDETECTOR_H_
