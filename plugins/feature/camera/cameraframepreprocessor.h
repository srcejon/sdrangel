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
    class MsgConfigureCameraFramePreprocessor : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const CameraSettings& getSettings() const { return m_settings; }
        const QList<QString>& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureCameraFramePreprocessor* create(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
        {
            return new MsgConfigureCameraFramePreprocessor(settings, settingsKeys, force);
        }

    private:
        CameraSettings m_settings;
        QList<QString> m_settingsKeys;
        bool m_force;

        MsgConfigureCameraFramePreprocessor(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force) :
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

    CameraFramePreprocessor();
    ~CameraFramePreprocessor();

    void startWork();
    void stopWork();
    void submitFrame(const CameraPipelineFramePtr& frame);
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setNextStage(CameraFrameAligner *nextStage) { m_nextStage = nextStage; }

private:
    MessageQueue m_inputMessageQueue;
    CameraFrameAligner *m_nextStage;
    CameraSettings m_settings;
    bool m_captureActive;
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
    bool preserveFrameOrder() const;
    int pendingFrameLimit() const;
    void reloadCalibrationFrames();
    cv::Mat loadFitsCalibrationFrame(const QString& fileName, const QString& calibrationType, bool normalizeFlat) const;
    void validateCalibrationFrame(cv::Mat& calibrationFrame, const cv::Size& expectedSize, const QString& calibrationType, const QString& fileName);
    cv::Mat applyCalibration(const cv::Mat& input);
    bool shouldMaterializeUnprocessedImage(const CameraPipelineFrame& frame) const;
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
    bool canUseCudaPreprocessing() const;
    void invalidateCudaCalibrationFrames();
    cv::cuda::GpuMat uploadCalibrationFrameCuda(CudaCalibrationFrame& cachedFrame, const cv::Mat& calibrationFrame, int channels);
    bool applyCalibrationCuda(cv::cuda::GpuMat& frameGpu, const cv::Size& inputSize, int inputType);
    bool preprocessFrameCuda(CameraPipelineFrame& frame, const cv::Mat& inputMat);
    QImage downloadCudaBgrImage(const cv::cuda::GpuMat& bgrGpu);
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
