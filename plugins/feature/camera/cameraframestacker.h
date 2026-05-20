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

#ifndef INCLUDE_FEATURE_CAMERAFRAMESTACKER_H_
#define INCLUDE_FEATURE_CAMERAFRAMESTACKER_H_

#include <QObject>
#include <QMutex>
#include <QString>
#include <deque>
#include <vector>

#include <opencv2/core/core.hpp>
#ifdef CAMERA_OPENCV_CUDA_STACKING
#include <opencv2/core/cuda.hpp>
#endif

#include "util/message.h"
#include "util/messagequeue.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"

class CameraImageProcessor;

class CameraFrameStacker : public QObject
{
    Q_OBJECT
public:
    class MsgConfigureCameraFrameStacker : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const CameraSettings& getSettings() const { return m_settings; }
        const QList<QString>& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureCameraFrameStacker* create(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
        {
            return new MsgConfigureCameraFrameStacker(settings, settingsKeys, force);
        }

    private:
        CameraSettings m_settings;
        QList<QString> m_settingsKeys;
        bool m_force;

        MsgConfigureCameraFrameStacker(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force) :
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

    CameraFrameStacker();
    ~CameraFrameStacker();

    void startWork();
    void stopWork();
    void submitFrame(const CameraPipelineFramePtr& frame);
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setNextStage(CameraImageProcessor *nextStage) { m_nextStage = nextStage; }

private:
    struct HdrFrameSample
    {
        cv::Mat m_frameMat;
        double m_exposureTimeMs = 0.0;
        int m_exposureIndex = -1;
    };

    MessageQueue m_inputMessageQueue;
    CameraImageProcessor *m_nextStage;
    CameraSettings m_settings;
    bool m_captureActive;
    std::deque<cv::Mat> m_stackFrameHistory;
    std::vector<HdrFrameSample> m_hdrFrameSamples;
    cv::Mat m_stackAccumulator;
#ifdef CAMERA_OPENCV_CUDA_STACKING
    cv::cuda::Stream m_cudaStackingStream;
    cv::cuda::GpuMat m_cudaStackAccumulator;
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
    void resetFrameHistoryState();
    void trimFrameHistoryToCurrentLimit();
#ifdef CAMERA_OPENCV_CUDA_STACKING
    [[nodiscard]] bool canUseCudaStacking() const;
    void subtractFromCudaAccumulator(const cv::Mat& frameMat);
    [[nodiscard]] bool applyAverageStackingCuda(const cv::Mat& frameMat, const cv::cuda::GpuMat* frameGpu, double scaleTo8Bit, QImage& outputImage);
#endif
    static cv::Mat imageToWorkingMat(const QImage& input, bool& highBitDepthInput);
    static QImage workingMatToImage(const cv::Mat& frameMat);
    bool canPassThroughFrame(const CameraPipelineFrame& inputFrame) const;
    [[nodiscard]] bool applyFrameStacking(const CameraPipelineFrame& inputFrame, QImage& outputImage, int& stackCount);

private slots:
    void handleInputMessages();
    void processNextFrame();
};

#endif // INCLUDE_FEATURE_CAMERAFRAMESTACKER_H_
