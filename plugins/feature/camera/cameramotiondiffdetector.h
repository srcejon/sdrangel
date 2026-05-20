///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3, or (at your option) later.         //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_FEATURE_CAMERAMOTIONDIFFDETECTOR_H_
#define INCLUDE_FEATURE_CAMERAMOTIONDIFFDETECTOR_H_

#include <deque>

#include <opencv2/video/background_segm.hpp>
#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
#include <opencv2/cudabgsegm.hpp>
#endif
#if defined(CAMERA_OPENCV_CUDA_DETECTION) || defined(CAMERA_OPENCV_CUDA_MOTION_DETECTION)
#include <opencv2/cudafilters.hpp>
#endif

#include "cameradetector.h"

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

#endif // INCLUDE_FEATURE_CAMERAMOTIONDIFFDETECTOR_H_
