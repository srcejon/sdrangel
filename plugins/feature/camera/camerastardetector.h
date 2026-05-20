///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3, or (at your option) later.         //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_FEATURE_CAMERASTARDETECTOR_H_
#define INCLUDE_FEATURE_CAMERASTARDETECTOR_H_

#ifdef CAMERA_OPENCV_CUDA_DETECTION
#include <opencv2/cudafilters.hpp>
#endif

#include "cameradetector.h"

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
    // highBitDepthGray, when non-empty, is a CV_16UC1 luminance image (at full image size,
    // *not* roi-cropped) extracted directly from the original QImage. When supplied the
    // detector preserves 16-bit precision in the residual and centroid weighting; the
    // saturation cutoff is scaled accordingly. When empty the detector falls back to the
    // legacy 8-bit pipeline derived from bgrMat.
    void applyStarDetection(const cv::Mat& bgrMat, const cv::cuda::GpuMat* bgrGpu, const cv::Rect& roi, const cv::Mat& highBitDepthGray, QVector<CameraPipelineStarDetection>& starDetections, cv::Mat* debugMask = nullptr) const;
    void applyStarPreprocessing(const cv::Mat& bgrMat, const cv::cuda::GpuMat* bgrGpu, const cv::Rect& roi, const cv::Mat& highBitDepthGray, cv::Mat& gray, cv::Mat& residual, cv::Mat& thresholdMask, cv::Mat* debugMask) const;
};

#endif // INCLUDE_FEATURE_CAMERASTARDETECTOR_H_
