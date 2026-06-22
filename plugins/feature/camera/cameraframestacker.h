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
#include <opencv2/cudafilters.hpp>
#endif

#include "util/message.h"
#include "util/messagequeue.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"

class CameraImageProcessor;

/**
 * \brief Pipeline stage that accumulates/averages aligned frames (and performs HDR fusion).
 *
 * Combines the stream of aligned frames into a single output image. It maintains a rolling
 * history of recent frames and a running accumulator to produce an averaged stack,
 * assesses per-frame quality (mean, std-dev, Laplacian variance, black/saturated fraction)
 * to reject poor or mis-aligned frames, builds the history thumbnail tiles shown in the
 * GUI, and supports HDR exposure fusion (Mertens / Debevec) over a set of differently
 * exposed samples. The combined image is forwarded to the image processor.
 *
 * \note Runs as a QObject on its own worker thread; frames arrive via submitFrame()/the
 *       input message queue and are processed in processNextFrame(). A bounded pending
 *       deque is used, with frame-order preservation and oldest-frame dropping on overrun.
 * \note Stacking and HDR fusion run on the CPU or, when built with
 *       CAMERA_OPENCV_CUDA_STACKING and a CUDA device is available, on the GPU (the
 *       m_cuda* accumulator, filters and fusion helpers). The CUDA accumulator is rebuilt
 *       when the stack history changes in ways the incremental add/subtract cannot track.
 * \note MsgDeleteStackFrame lets the GUI remove a specific frame from the current stack,
 *       which forces the accumulator/history to be recomputed.
 */
class CameraFrameStacker : public QObject
{
    Q_OBJECT
public:


    class MsgDeleteStackFrame : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        int getFrameIndex() const { return m_frameIndex; }

        static MsgDeleteStackFrame* create(int frameIndex)
        {
            return new MsgDeleteStackFrame(frameIndex);
        }

    private:
        int m_frameIndex;

        MsgDeleteStackFrame(int frameIndex) :
            Message(),
            m_frameIndex(frameIndex)
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
#ifdef CAMERA_OPENCV_CUDA_STACKING
        cv::cuda::GpuMat m_frameGpu;
#endif
        double m_exposureTimeMs = 0.0;
    };

    struct StackFrameQuality
    {
        double m_mean = 0.0;
        double m_stdDev = 0.0;
        double m_laplacianVariance = 0.0;
        double m_blackFraction = 0.0;
        double m_saturatedFraction = 0.0;
        bool m_valid = false;
    };

    MessageQueue m_inputMessageQueue;
    CameraImageProcessor *m_nextStage;
    CameraSettings m_settings;
    bool m_captureActive;
    quint64 m_captureEpoch = 0;
    std::deque<cv::Mat> m_stackFrameHistory;
    std::deque<StackFrameQuality> m_stackFrameQualityHistory;
    std::deque<QImage> m_stackFrameThumbnails;
    std::vector<HdrFrameSample> m_hdrFrameSamples;
    cv::Mat m_stackAccumulator;
#ifdef CAMERA_OPENCV_CUDA_STACKING
    cv::cuda::Stream m_cudaStackingStream;
    cv::cuda::GpuMat m_cudaStackAccumulator;
    int m_cudaStackAccumulatorInputType;
    cv::Ptr<cv::cuda::Filter> m_cudaQualityLaplacianFilter;
    int m_cudaQualityLaplacianFilterType;
    cv::Ptr<cv::cuda::Filter> m_cudaHdrLaplacianFilter;
#endif
    QMutex m_frameMutex;
    std::deque<CameraPipelineFramePtr> m_pendingFrames;
    CameraPipelineFramePtr m_lastFrameTemplate;
    QImage m_lastStackedImage;
    bool m_processingFrame;
    int m_droppedFrameCount;
    int m_rejectedFrameCount;

    bool handleMessage(const Message& cmd);
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    void processNewFrame(const CameraPipelineFramePtr& frame);
    bool preserveFrameOrder() const;
    int pendingFrameLimit() const;
    int dropOldestPendingFramesForOverflow();
    void resetFrameHistoryState();
    void trimFrameHistoryToCurrentLimit();
#ifdef CAMERA_OPENCV_CUDA_STACKING
    [[nodiscard]] bool canUseCudaStacking() const;
    void subtractFromCudaAccumulator(const cv::Mat& frameMat);
    bool rebuildCudaAverageAccumulator();
    [[nodiscard]] bool applyAverageStackingCuda(const cv::Mat& frameMat, const cv::cuda::GpuMat* frameGpu, cv::cuda::GpuMat& outputRgbGpu);
    [[nodiscard]] bool applyMertensFusionCuda(const std::vector<const HdrFrameSample *>& sortedSamples, cv::cuda::GpuMat& tonemappedRgbGpu);
    [[nodiscard]] bool applyDebevecFusionCuda(const std::vector<const HdrFrameSample *>& sortedSamples, const std::vector<float>& exposureTimesSeconds, cv::cuda::GpuMat& tonemappedRgbGpu);
    void setCudaBgrOutputFromRgbGpu(CameraPipelineFrame& inputFrame, const cv::cuda::GpuMat& rgbGpu, int outputDepth = CV_8U);
    bool setCudaBgrOutputFromRgbMat(CameraPipelineFrame& inputFrame, const cv::Mat& rgbMat);
    [[nodiscard]] bool applyOutputScalingCuda(CameraPipelineFrame& frame, const QSize& targetSize);
#endif
    static cv::Mat imageToWorkingMat(const QImage& input, bool& highBitDepthInput);
    static QImage workingMatToImage(const cv::Mat& frameMat);
    static QImage makeHistoryThumbnail(const cv::Mat& frameMat);
    static QImage makeHistoryTilesImage(const std::deque<QImage>& thumbnails, const std::deque<StackFrameQuality>& qualities);
    static StackFrameQuality computeStackFrameQuality(const cv::Mat& frameMat);
#ifdef CAMERA_OPENCV_CUDA_STACKING
    StackFrameQuality computeStackFrameQualityCuda(const CameraPipelineFrame& inputFrame);
#endif
    StackFrameQuality computeStackFrameQualityForFrame(const CameraPipelineFrame& inputFrame, const cv::Mat& frameMat);
    void ensureStackFrameQualityHistory();
    std::vector<size_t> selectedSharpFrameIndices() const;
    static double medianQualityValue(const std::deque<StackFrameQuality>& qualities, double StackFrameQuality::*member);
    bool canPassThroughFrame(const CameraPipelineFrame& inputFrame) const;
    [[nodiscard]] QSize scaledOutputSize(const QSize& inputSize) const;
    [[nodiscard]] bool applyOutputScaling(CameraPipelineFrame& frame);
    [[nodiscard]] bool applyFrameStacking(CameraPipelineFrame& inputFrame, QImage& outputImage, int& stackCount);
    [[nodiscard]] bool shouldRejectStackFrame(const StackFrameQuality& quality, QString& reason) const;
    [[nodiscard]] bool shouldRejectStackAlignment(const CameraPipelineFrame& inputFrame, QString& reason) const;
    bool renderStackDisplayImage(const QImage& stackedImage, QImage& outputImage);
    void ensureHistoryThumbnails();
    void deleteStackFrame(int frameIndex);
    void emitHistoryPreviewFrame();

private slots:
    void handleInputMessages();
    void processNextFrame();
};

#endif // INCLUDE_FEATURE_CAMERAFRAMESTACKER_H_
