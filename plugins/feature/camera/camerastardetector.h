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

#ifndef INCLUDE_FEATURE_CAMERASTARDETECTOR_H_
#define INCLUDE_FEATURE_CAMERASTARDETECTOR_H_

#ifdef CAMERA_OPENCV_CUDA_DETECTION
#include <opencv2/cudafilters.hpp>
#endif

#include "cameradetector.h"
#include "cameraplatesolver.h"

class CameraStarDetector : public CameraDetectionStage
{
    Q_OBJECT
public:
    class MsgReportPlateSolveStatus : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        bool isSolving() const { return m_solving; }

        static MsgReportPlateSolveStatus* create(bool solving)
        {
            return new MsgReportPlateSolveStatus(solving);
        }

    private:
        bool m_solving;

        MsgReportPlateSolveStatus(bool solving) :
            Message(),
            m_solving(solving)
        { }
    };

    CameraStarDetector();
    ~CameraStarDetector() override;
    void setMessageQueueToGUI(MessageQueue *messageQueue) { m_msgQueueToGUI = messageQueue; }
    void requestPlateSolveCancellation();
    [[nodiscard]] static bool plateSolveInputSettingsChanged(const QList<QString>& settingsKeys);

protected:
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false) override;
    void captureActiveChanged(bool active) override;
    void processNewFrame(const CameraPipelineFramePtr& frame) override;

private:
    CameraPipelineFramePtr m_lastInputFrame;
    CameraPlateSolver m_plateSolver;
    MessageQueue *m_msgQueueToGUI;

    [[nodiscard]] static bool starDisplaySettingsChanged(const QList<QString>& settingsKeys);
    void reportPlateSolveStatus(bool solving) const;

#ifdef CAMERA_OPENCV_CUDA_DETECTION
    cv::cuda::Stream m_cudaDetectionStream;
    cv::Ptr<cv::cuda::Filter> m_cudaStarSmallBlurFilter;
    cv::Ptr<cv::cuda::Filter> m_cudaStarBackgroundBlurFilter;
    int m_cudaStarSmallBlurFilterType;
    int m_cudaStarBackgroundBlurFilterType;
    int m_cudaStarBackgroundBlurFilterSize;
    cv::cuda::GpuMat m_cudaStarExclusionMask;
    cv::Rect m_cudaStarExclusionRoi;
    cv::Size m_cudaStarExclusionWorkSize;
    QVector<QRect> m_cudaStarExclusionRects;

    [[nodiscard]] bool canUseCudaDetection() const;
    [[nodiscard]] cv::Ptr<cv::cuda::Filter> cudaStarSmallBlurFilter(int inputType);
    [[nodiscard]] cv::Ptr<cv::cuda::Filter> cudaStarBackgroundBlurFilter(int inputType, int kernelSize);
    [[nodiscard]] const cv::cuda::GpuMat& cudaStarExclusionMask(const cv::Rect& roi, const cv::Size& workSize);
    bool applyStarPreprocessingCuda(const cv::Mat& bgrMat, const cv::cuda::GpuMat* bgrGpu, const cv::Rect& roi, cv::Mat& gray, cv::Mat& residual, cv::Mat& thresholdMask, double& residualNoiseSigma, cv::Mat* debugMask);
#endif
    // highBitDepthGray, when non-empty, is a CV_16UC1 luminance image (at full image size,
    // *not* roi-cropped) extracted directly from the original QImage. When supplied the
    // detector preserves 16-bit precision in the residual and centroid weighting; the
    // saturation cutoff is scaled accordingly. When empty the detector falls back to the
    // legacy 8-bit pipeline derived from bgrMat.
    void applyStarDetection(
        const cv::Mat& bgrMat,
#ifdef CAMERA_OPENCV_CUDA_DETECTION
        const cv::cuda::GpuMat* bgrGpu,
#endif
        const cv::Rect& roi,
        const cv::Mat& highBitDepthGray,
        QVector<CameraPipelineStarDetection>& starDetections,
        cv::Mat* debugMask = nullptr);
    void applyStarPreprocessing(const cv::Mat& bgrMat, const cv::Rect& roi, const cv::Mat& highBitDepthGray, cv::Mat& gray, cv::Mat& residual, cv::Mat& thresholdMask, double& residualNoiseSigma, cv::Mat* debugMask);
};

#endif // INCLUDE_FEATURE_CAMERASTARDETECTOR_H_
