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

#ifndef INCLUDE_FEATURE_CAMERAIMAGEPROCESSOR_H_
#define INCLUDE_FEATURE_CAMERAIMAGEPROCESSOR_H_

#include <deque>

#include <QObject>
#include <QMutex>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif

#include "util/message.h"
#include "util/messagequeue.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"

class CameraDetectionStage;

/**
 * \brief Pipeline stage that applies the user-configured image adjustments to each frame.
 *
 * Final processing stage of the camera pipeline: takes the (preprocessed/aligned/stacked)
 * frame and applies the configured chain of adjustments — lens unwarp, white balance,
 * histogram stretch, greyscale, saturation, gamma, blur, sharpen, edge detection, flip,
 * rotation, brightness/contrast and colour inversion — then forwards the result to the
 * detection stage. It also computes the histogram data shown in the GUI.
 *
 * \note Runs as a QObject moved onto its own worker thread; frames arrive via
 *       submitFrame()/the input message queue and are processed in processNextFrame()
 *       on that thread. A small bounded backlog (m_maxPendingFrames) absorbs occasional
 *       processing spikes; sustained overruns trim the oldest pending frame to keep
 *       latency bounded.
 * \note Two interchangeable processing paths exist: a CPU path (applyImageProcessingCpu)
 *       and, when built with CAMERA_OPENCV_CUDA_IMAGE_PROCESSING and a CUDA device is
 *       available, an OpenCV-CUDA path (applyImageProcessingCuda). The CUDA path caches
 *       device-side filters/lookup tables and unwarp maps that are invalidated on the
 *       relevant settings changes.
 * \note Frames are reference-counted (CameraPipelineFramePtr); ownership of submitted
 *       frames is shared and the stage does not retain frames beyond processing apart
 *       from the cached last-input frame used to re-emit on settings changes.
 */
class CameraImageProcessor : public QObject
{
    Q_OBJECT
public:


    CameraImageProcessor();
    ~CameraImageProcessor();

    void startWork();
    void stopWork();
    void submitFrame(const CameraPipelineFramePtr& frame);
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setNextStage(CameraDetectionStage *nextStage) { m_nextStage = nextStage; }

private:
    struct LastInputFrame
    {
        QImage m_image;
        QImage m_unprocessedImage;
        QImage m_rawInputImage;
        QDateTime m_captureDateTime;
        CameraPipelineDirection m_captureDirection;
        quint64 m_captureEpoch = 0;
        qint64 m_pipelineInputWallClockMs = 0;
        qint64 m_playbackPositionMs = -1;
        int m_playbackFrameNumber = -1;
        double m_playbackFrameRate = 0.0;
        double m_exposureTimeMs = 0.0;
        int m_hdrExposureIndex = -1;
        int m_hdrExposureCount = 0;
        CameraPipelineStacking m_stack;
        CameraPipelineImageTransform m_imageTransform;
        CameraPipelineFrame::BayerPattern m_bayerPattern = CameraPipelineFrame::BayerNone;
        CameraPipelineFrame::BayerPattern m_rawInputBayerPattern = CameraPipelineFrame::BayerNone;
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
        cv::cuda::GpuMat m_cudaBgrImage;
        cv::cuda::GpuMat m_cudaGrayImage;
#endif

        void clear();
        [[nodiscard]] bool hasImageData() const;
    };

    MessageQueue m_inputMessageQueue;
    CameraDetectionStage *m_nextStage;
    CameraSettings m_settings;
    bool m_captureActive;
    quint64 m_captureEpoch = 0;
    LastInputFrame m_lastInputFrame;
    cv::Vec3d m_autoWhiteBalanceGains;
    bool m_autoWhiteBalanceInitialized;
    cv::Mat m_unwarpMapX;
    cv::Mat m_unwarpMapY;
    cv::Size m_unwarpMapSize;
    CameraSettings::LensProjection m_unwarpSourceProjection;
    double m_unwarpSourceFov;
    QMutex m_frameMutex;
    // Small bounded backlog of frames waiting to be processed, so an occasional
    // processing spike is absorbed instead of costing a frame; a sustained
    // overrun trims the oldest frame so latency stays bounded.
    static constexpr int m_maxPendingFrames = 3;
    std::deque<CameraPipelineFramePtr> m_pendingFrames;
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
    cv::cuda::Stream m_cudaStream;
    cv::cuda::GpuMat m_cudaUnwarpMapX;
    cv::cuda::GpuMat m_cudaUnwarpMapY;
    cv::Size m_cudaUnwarpMapSize;
    cv::Ptr<cv::cuda::CLAHE> m_cudaClahe;
    cv::Ptr<cv::cuda::LookUpTable> m_cudaHistogramStretchLookup;
    CameraSettings::HistogramStretch m_cudaHistogramStretchMode;
    double m_cudaHistogramStretchBlackPoint;
    double m_cudaHistogramStretchWhitePoint;
    double m_cudaHistogramStretchGamma;
    double m_cudaHistogramStretchAsinhStrength;
    double m_cudaHistogramStretchLogStrength;
    cv::Ptr<cv::cuda::LookUpTable> m_cudaGammaLookup;
    double m_cudaGamma;
    cv::Ptr<cv::cuda::Filter> m_cudaGaussianBlurFilter;
    int m_cudaGaussianBlurKernelSize;
    int m_cudaGaussianBlurType;
    cv::Ptr<cv::cuda::Filter> m_cudaMedianBlurFilter;
    int m_cudaMedianBlurKernelSize;
    int m_cudaMedianBlurChannelType;
    cv::Ptr<cv::cuda::Filter> m_cudaSharpenBlurFilter;
    int m_cudaSharpenBlurType;
    cv::Ptr<cv::cuda::Filter> m_cudaSobelXFilter;
    cv::Ptr<cv::cuda::Filter> m_cudaSobelYFilter;
    int m_cudaSobelInputType;
    cv::Ptr<cv::cuda::CannyEdgeDetector> m_cudaCannyDetector;
#endif
    bool m_processingFrame;

    bool handleMessage(const Message& cmd);
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    void processNewFrame(const CameraPipelineFramePtr& frame);
    void storeLastInputFrame(const CameraPipelineFrame& frame);
    [[nodiscard]] CameraPipelineFramePtr createFrameFromLastInput() const;
    void applyImageProcessing(CameraPipelineFrame& frame);
    void applyImageProcessingCpu(CameraPipelineFrame& frame);
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
    void applyImageProcessingCuda(CameraPipelineFrame& frame);
    [[nodiscard]] bool canUseCudaImageProcessing() const;
    void applyLensUnwarpCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream);
    void applyWhiteBalanceCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream);
    void applyHistogramStretchCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream);
    void applyGreyscaleCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream);
    void applySaturationCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream);
    void applyGammaCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream);
    void applyGaussianBlurCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream);
    void applyMedianBlurCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream);
    void invalidateMedianBlurCudaFilter();
    void invalidateCudaProcessingCaches();
    void applySharpenCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream);
    void applySobelEdgeCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream);
    void applyCannyEdgeCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream);
    void applyRotationCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream);
#endif
    void applyWhiteBalance(cv::Mat& bgrMat);
    void applyLensUnwarp(cv::Mat& bgrMat);
    void applyHistogramStretch(cv::Mat& bgrMat) const;
    void applyGreyscale(cv::Mat& bgrMat) const;
    void applySaturation(cv::Mat& bgrMat);
    void applyGamma(cv::Mat& bgrMat) const;
    void applyGaussianBlur(cv::Mat& bgrMat) const;
    void applyMedianBlur(cv::Mat& bgrMat) const;
    void applySharpen(cv::Mat& bgrMat) const;
    void applySobelEdge(cv::Mat& bgrMat) const;
    void applyCannyEdge(cv::Mat& bgrMat) const;
    void applyFlip(cv::Mat& bgrMat) const;
    void applyRotation(cv::Mat& bgrMat) const;
    void applyBrightnessContrast(cv::Mat& bgrMat) const;
    void applyInvertColors(cv::Mat& bgrMat) const;
    [[nodiscard]] CameraHistogramData computeHistogramData(const CameraPipelineFrame& frame);
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
    [[nodiscard]] CameraHistogramData computeHistogramDataCuda(const CameraPipelineFrame& frame);
    static void fillHistogramBinsFromCudaMat(const cv::Mat& hist, QVector<float>& bins);
#endif
    [[nodiscard]] static CameraHistogramData computeHistogramDataCpu(const QImage& image);
    [[nodiscard]] static const QImage& ensureRgb888(const QImage& image, QImage& convertedImage);
    [[nodiscard]] static cv::Mat wrapRgb888Image(const QImage& image);
    [[nodiscard]] static QImage convertBgrToRgbImage(const cv::Mat& bgrMat);
    static double degreesToRadians(double degrees);
    static double sourceRadiusForTheta(double thetaRadians, CameraSettings::LensProjection projection, double focalPixels);
    void invalidateUnwarpMaps();
    void ensureUnwarpMaps(const cv::Size& frameSize);

private slots:
    void handleInputMessages();
    void processNextFrame();
};

#endif // INCLUDE_FEATURE_CAMERAIMAGEPROCESSOR_H_
