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
#endif

#include "util/message.h"
#include "util/messagequeue.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"

class CameraFrameAligner;

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
#endif
    QMutex m_frameMutex;
    std::deque<CameraPipelineFramePtr> m_pendingFrames;
    bool m_processingFrame;
    int m_droppedFrameCount;

    bool handleMessage(const Message& cmd);
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    void processNewFrame(const CameraPipelineFramePtr& frame);
    void preprocessFrame(const CameraPipelineFramePtr& frame, cv::Mat& inputMat);
    bool preserveFrameOrder() const;
    int pendingFrameLimit() const;
    void reloadCalibrationFrames();
    cv::Mat loadFitsCalibrationFrame(const QString& fileName, const QString& calibrationType, bool normalizeFlat) const;
    void validateCalibrationFrame(cv::Mat& calibrationFrame, const cv::Size& expectedSize, const QString& calibrationType, const QString& fileName);
    cv::Mat applyCalibration(const cv::Mat& input);
    bool shouldMaterializeUnprocessedImage(const CameraPipelineFrame& frame) const;
    bool shouldMaterializeRawInputImage(const CameraPipelineFrame& frame) const;
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
    bool canUseCudaPreprocessing() const;
    void invalidateCudaCalibrationFrames();
    cv::cuda::GpuMat uploadCalibrationFrameCuda(CudaCalibrationFrame& cachedFrame, const cv::Mat& calibrationFrame, int channels);
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
