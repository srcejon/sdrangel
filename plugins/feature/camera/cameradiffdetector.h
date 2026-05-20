///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3, or (at your option) later.         //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_FEATURE_CAMERADIFFDETECTOR_H_
#define INCLUDE_FEATURE_CAMERADIFFDETECTOR_H_

#include <deque>

#ifdef CAMERA_OPENCV_CUDA_DETECTION
#include <opencv2/cudafilters.hpp>
#endif

#include "cameradetector.h"

class CameraDiffDetector : public CameraDetectionStage
{
    Q_OBJECT
public:
    CameraDiffDetector();
    ~CameraDiffDetector() override;

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

#ifdef CAMERA_OPENCV_CUDA_DETECTION
    [[nodiscard]] bool canUseCudaDetection() const;
    bool applyDiffMaskCuda(cv::Mat& bgrMat, const cv::cuda::GpuMat* bgrGpu, const cv::Rect& roi, const CameraPipelineFrame& diffReferenceFrame);
    [[nodiscard]] cv::Ptr<cv::cuda::Filter> cudaDiffOpenFilter(int inputType, int kernelSize);
    [[nodiscard]] cv::Ptr<cv::cuda::Filter> cudaDiffDilationFilter(int inputType, int kernelSize);
    [[nodiscard]] cv::Ptr<cv::cuda::Filter> cudaDiffCloseFilter(int inputType, int kernelSize);
    [[nodiscard]] const cv::cuda::GpuMat& cudaDiffExclusionMask(const cv::Rect& roi, const cv::Size& workSize);
#endif
    void applyDiffMask(CameraPipelineFrame& frame, cv::Mat& bgrMat, const cv::Rect& roi, const CameraPipelineFrame& diffReferenceFrame);
};

#endif // INCLUDE_FEATURE_CAMERADIFFDETECTOR_H_
