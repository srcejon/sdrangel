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

class CameraImageProcessor : public QObject
{
    Q_OBJECT
public:
    class MsgConfigureCameraImageProcessor : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const CameraSettings& getSettings() const { return m_settings; }
        const QList<QString>& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureCameraImageProcessor* create(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
        {
            return new MsgConfigureCameraImageProcessor(settings, settingsKeys, force);
        }

    private:
        CameraSettings m_settings;
        QList<QString> m_settingsKeys;
        bool m_force;

        MsgConfigureCameraImageProcessor(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force) :
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

    CameraImageProcessor();
    ~CameraImageProcessor();

    void startWork();
    void stopWork();
    void submitFrame(const CameraPipelineFramePtr& frame);
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setNextStage(CameraDetectionStage *nextStage) { m_nextStage = nextStage; }

private:
    MessageQueue m_inputMessageQueue;
    CameraDetectionStage *m_nextStage;
    CameraSettings m_settings;
    bool m_captureActive;
    CameraPipelineFrame m_lastInputFrame;
    cv::Vec3d m_autoWhiteBalanceGains;
    bool m_autoWhiteBalanceInitialized;
    cv::Mat m_unwarpMapX;
    cv::Mat m_unwarpMapY;
    cv::Size m_unwarpMapSize;
    CameraSettings::LensProjection m_unwarpSourceProjection;
    double m_unwarpSourceFov;
    QMutex m_frameMutex;
    CameraPipelineFramePtr m_pendingFrame;
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
    cv::cuda::Stream m_cudaStream;
    mutable cv::cuda::GpuMat m_cudaUnwarpMapX;
    mutable cv::cuda::GpuMat m_cudaUnwarpMapY;
    mutable cv::Size m_cudaUnwarpMapSize;
    mutable cv::Ptr<cv::cuda::CLAHE> m_cudaClahe;
    mutable cv::Ptr<cv::cuda::LookUpTable> m_cudaHistogramStretchLookup;
    mutable CameraSettings::HistogramStretch m_cudaHistogramStretchMode;
    mutable double m_cudaHistogramStretchBlackPoint;
    mutable double m_cudaHistogramStretchWhitePoint;
    mutable double m_cudaHistogramStretchGamma;
    mutable double m_cudaHistogramStretchAsinhStrength;
    mutable double m_cudaHistogramStretchLogStrength;
    mutable cv::Ptr<cv::cuda::LookUpTable> m_cudaGammaLookup;
    mutable double m_cudaGamma;
    mutable cv::Ptr<cv::cuda::Filter> m_cudaGaussianBlurFilter;
    mutable int m_cudaGaussianBlurKernelSize;
    mutable int m_cudaGaussianBlurType;
    mutable cv::Ptr<cv::cuda::Filter> m_cudaMedianBlurFilter;
    mutable int m_cudaMedianBlurKernelSize;
    mutable int m_cudaMedianBlurChannelType;
    mutable cv::Ptr<cv::cuda::Filter> m_cudaSharpenBlurFilter;
    mutable int m_cudaSharpenBlurType;
    mutable cv::Ptr<cv::cuda::Filter> m_cudaSobelXFilter;
    mutable cv::Ptr<cv::cuda::Filter> m_cudaSobelYFilter;
    mutable int m_cudaSobelInputType;
    mutable cv::Ptr<cv::cuda::CannyEdgeDetector> m_cudaCannyDetector;
#endif
    bool m_processingFrame;

    bool handleMessage(const Message& cmd);
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    void processNewFrame(const CameraPipelineFramePtr& frame);
    void storeLastInputFrame(const CameraPipelineFrame& frame);
    void applyImageProcessing(CameraPipelineFrame& frame);
    void applyImageProcessingCpu(CameraPipelineFrame& frame);
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
    void applyImageProcessingCuda(CameraPipelineFrame& frame);
    [[nodiscard]] bool canUseCudaImageProcessing() const;
    void applyLensUnwarpCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream);
    void applyWhiteBalanceCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream);
    void applyHistogramStretchCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const;
    void applyGreyscaleCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const;
    void applySaturationCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const;
    void applyGammaCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const;
    void applyGaussianBlurCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const;
    void applyMedianBlurCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const;
    void invalidateMedianBlurCudaFilter() const;
    void invalidateCudaProcessingCaches() const;
    void applySharpenCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const;
    void applySobelEdgeCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const;
    void applyCannyEdgeCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const;
    void applyRotationCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const;
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
