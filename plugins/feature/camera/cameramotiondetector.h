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

#ifndef INCLUDE_FEATURE_CAMERAMOTIONDETECTOR_H_
#define INCLUDE_FEATURE_CAMERAMOTIONDETECTOR_H_

#include <opencv2/video/background_segm.hpp>
#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
#include <opencv2/cudabgsegm.hpp>
#include <opencv2/cudafilters.hpp>
#endif

#include "cameradetector.h"

class Camera;

class CameraMotionDetector : public CameraDetectionStage
{
    Q_OBJECT
public:
    explicit CameraMotionDetector(Camera *camera);
    ~CameraMotionDetector() override;

protected:
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false) override;
    void captureActiveChanged(bool active) override;
    void processNewFrame(const CameraPipelineFramePtr& frame) override;

private:
    Camera *m_camera;
    cv::Ptr<cv::BackgroundSubtractor> m_bgSubtractor;
#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
    cv::Ptr<cv::cuda::BackgroundSubtractorMOG2> m_cudaBgSubtractor;
    cv::cuda::Stream m_cudaMotionStream;
    cv::Ptr<cv::cuda::Filter> m_cudaMotionOpenFilter;
    cv::Ptr<cv::cuda::Filter> m_cudaMotionCloseFilter;
    int m_cudaMotionOpenFilterSize;
    int m_cudaMotionOpenFilterType;
    int m_cudaMotionCloseFilterSize;
    int m_cudaMotionCloseFilterType;
    cv::cuda::GpuMat m_cudaMotionExclusionMask;
    cv::Rect m_cudaMotionExclusionRoi;
    cv::Size m_cudaMotionExclusionWorkSize;
    QList<QRect> m_cudaMotionExclusionRects;
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
    bool applyMotionDetectionCuda(const cv::Mat& bgrMat, const cv::cuda::GpuMat* bgrGpu, const cv::Rect& roi, QVector<QRect>& motionBoxes, cv::Mat* debugMask = nullptr);
#endif
    void applyMotionDetection(const cv::Mat& bgrMat, const cv::Rect& roi, QVector<QRect>& motionBoxes, cv::Mat* debugMask = nullptr);
};

#endif // INCLUDE_FEATURE_CAMERAMOTIONDETECTOR_H_
