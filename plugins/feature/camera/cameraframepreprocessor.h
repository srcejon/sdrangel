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

#ifndef INCLUDE_FEATURE_CAMERAFRAMEPREPROCESSOR_H_
#define INCLUDE_FEATURE_CAMERAFRAMEPREPROCESSOR_H_

#include <QObject>
#include <QMutex>
#include <deque>

#include <opencv2/core/core.hpp>
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafilters.hpp>
#endif

#include "util/message.h"
#include "util/messagequeue.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"

class CameraFrameAligner;

/**
 * \brief First pipeline stage: debayers raw frames and applies dark/flat/bias calibration.
 *
 * Turns an incoming raw/captured frame into a usable image: debayers raw single-channel
 * frames according to their Bayer pattern and applies the configured master calibration
 * frames (dark, flat, bias) loaded from FITS files. It may also materialise the
 * unprocessed and raw-input images that downstream stages and the GUI need, then forwards
 * the result to the frame aligner.
 *
 * \note Runs as a QObject on its own worker thread; frames arrive via submitFrame()/the
 *       input message queue and are processed in processNextFrame(). A bounded pending
 *       deque is used, with frame-order preservation and oldest-frame dropping on overrun.
 * \note Calibration is performed on the CPU (applyCalibration) or, when built with
 *       CAMERA_OPENCV_CUDA_IMAGE_PROCESSING and a CUDA device is available, on the GPU
 *       (applyCalibrationCuda). The CUDA path caches uploaded calibration frames keyed by
 *       source size/type/channels; these caches are invalidated when the calibration
 *       frames are reloaded.
 * \warning Calibration frames are validated against the expected frame geometry; a size
 *          mismatch disables that calibration term rather than corrupting the frame.
 */
class CameraFramePreprocessor : public QObject
{
    Q_OBJECT
public:


    CameraFramePreprocessor();
    ~CameraFramePreprocessor();

    void startWork();
    void stopWork();
    void submitFrame(const CameraPipelineFramePtr& frame);
    bool wouldReplacePendingFrame();
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setNextStage(CameraFrameAligner *nextStage) { m_nextStage = nextStage; }

private:
    MessageQueue m_inputMessageQueue;
    CameraFrameAligner *m_nextStage;
    CameraSettings m_settings;
    bool m_captureActive;
    quint64 m_captureEpoch = 0;
    cv::Mat m_darkCalibrationFrame;
    cv::Mat m_darkHotPixelMask;
    bool m_darkHotPixelRepairLogPending = false;
    cv::Mat m_flatCalibrationFrame;
    cv::Mat m_biasCalibrationFrame;
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
    cv::cuda::Stream m_cudaStream;

    struct CudaCalibrationFrame
    {
        cv::cuda::GpuMat m_frame;
        cv::Size m_sourceSize;
        int m_sourceType = -1;
        int m_channels = 0;
    };

    CudaCalibrationFrame m_cudaDarkCalibrationFrame;
    CudaCalibrationFrame m_cudaFlatCalibrationFrame;
    CudaCalibrationFrame m_cudaBiasCalibrationFrame;
    cv::cuda::GpuMat m_cudaDarkHotPixelMask;
    cv::Size m_cudaDarkHotPixelMaskSize;
    cv::Ptr<cv::cuda::Filter> m_cudaMonoHotPixelRepairFilter;
#endif
    QMutex m_frameMutex;
    std::deque<CameraPipelineFramePtr> m_pendingFrames;
    bool m_processingFrame;
    int m_droppedFrameCount;

    bool handleMessage(const Message& cmd);
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    void processNewFrame(const CameraPipelineFramePtr& frame);
    void applyPlaybackProjectionTransform(CameraPipelineFrame& frame) const;
    void preprocessFrame(const CameraPipelineFramePtr& frame, cv::Mat& inputMat);
    bool preserveFrameOrder() const;
    int pendingFrameLimit() const;
    void reloadCalibrationFrames();
    cv::Mat loadFitsCalibrationFrame(const QString& fileName, const QString& calibrationType, bool normalizeFlat) const;
    cv::Mat buildHotPixelMask(const cv::Mat& darkFrame, const QString& fileName) const;
    void validateCalibrationFrame(cv::Mat& calibrationFrame, const cv::Size& expectedSize, const QString& calibrationType, const QString& fileName);
    cv::Mat applyCalibration(const cv::Mat& input);
    int repairHotPixels(cv::Mat& calibratedFrame, const cv::Mat& hotPixelMask) const;
    bool shouldMaterializeUnprocessedImage(const CameraPipelineFrame& frame) const;
    bool shouldMaterializeRawInputImage(const CameraPipelineFrame& frame) const;
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
    bool canUseCudaPreprocessing() const;
    void invalidateCudaCalibrationFrames();
    cv::cuda::GpuMat uploadCalibrationFrameCuda(CudaCalibrationFrame& cachedFrame, const cv::Mat& calibrationFrame, int channels);
    cv::cuda::GpuMat uploadHotPixelMaskCuda();
    cv::Ptr<cv::cuda::Filter> cudaHotPixelRepairFilter(int channels);
    int repairHotPixelsCuda(cv::cuda::GpuMat& calibratedGpu, int channels);
    bool applyCalibrationCuda(cv::cuda::GpuMat& frameGpu, const cv::Size& inputSize, int inputType);
    bool preprocessFrameCuda(CameraPipelineFrame& frame, const cv::Mat& inputMat);
    QImage downloadCudaBgrImage(const cv::cuda::GpuMat& bgrGpu, bool preserveBitDepth);
#endif
    static int bayerPatternToOpenCvCode(CameraPipelineFrame::BayerPattern bayerPattern);
    static cv::Mat imageToWorkingMat(const QImage& input);
    static QImage workingMatToImage(const cv::Mat& frameMat);
    static cv::Mat debayerRawMat(const cv::Mat& input, CameraPipelineFrame::BayerPattern bayerPattern);

private slots:
    void handleInputMessages();
    void processNextFrame();
};

#endif // INCLUDE_FEATURE_CAMERAFRAMEPREPROCESSOR_H_
