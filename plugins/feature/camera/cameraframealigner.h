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

#ifndef INCLUDE_FEATURE_CAMERAFRAMEALIGNER_H_
#define INCLUDE_FEATURE_CAMERAFRAMEALIGNER_H_

#include <QObject>
#include <QMutex>
#include <deque>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#ifdef CAMERA_OPENCV_CUDA_STACKING
#include <opencv2/core/cuda.hpp>
#endif

#include "util/message.h"
#include "util/messagequeue.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"

class CameraFrameStacker;

class CameraFrameAligner : public QObject
{
    Q_OBJECT
public:


    CameraFrameAligner();
    ~CameraFrameAligner();

    void startWork();
    void stopWork();
    void submitFrame(const CameraPipelineFramePtr& frame);
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setNextStage(CameraFrameStacker *nextStage) { m_nextStage = nextStage; }

private:
    MessageQueue m_inputMessageQueue;
    CameraFrameStacker *m_nextStage;
    CameraSettings m_settings;
    bool m_captureActive;
    quint64 m_captureEpoch = 0;
    // Single fixed alignment reference, holding the UNALIGNED first frame of the current
    // sequence. Storing one frame (instead of a rolling deque of warped output frames)
    // avoids accumulating registration error and the discontinuity that occurred when the
    // deque rolled past stackFrameCount and the "reference" jumped to a previously-warped
    // frame. Reset on capture start, on settings changes that invalidate the source, or
    // when the frame geometry changes.
    cv::Mat m_alignmentReference;
    cv::Mat m_lastStarAlignmentTransform;
#ifdef CAMERA_OPENCV_CUDA_STACKING
    cv::cuda::Stream m_cudaAlignmentStream;
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
    void resetAlignmentState();
    [[nodiscard]] QImage applyAlignment(CameraPipelineFrame& frame);
    [[nodiscard]] static cv::Mat imageToWorkingMat(const QImage& input, bool& highBitDepthInput);
    [[nodiscard]] static QImage workingMatToImage(const cv::Mat& frameMat, bool highBitDepthInput);
    [[nodiscard]] cv::Mat alignFrame(const cv::Mat& frameMat, CameraPipelineFrame& frame, cv::Mat *appliedTransform = nullptr, bool deferWarp = false);
    [[nodiscard]] cv::Mat alignWithPhaseCorrelation(const cv::Mat& referenceFrame, const cv::Mat& targetFrame, CameraPipelineFrame& frame, cv::Mat *appliedTransform = nullptr, bool deferWarp = false);
    [[nodiscard]] cv::Mat alignWithStarCentroids(const cv::Mat& referenceFrame, const cv::Mat& targetFrame, CameraPipelineFrame& frame, cv::Mat *appliedTransform = nullptr, bool deferWarp = false);
    [[nodiscard]] cv::Mat warpFrameAffine(const cv::Mat& frameMat, const cv::Mat& transform);
#ifdef CAMERA_OPENCV_CUDA_STACKING
    [[nodiscard]] bool warpFrameAffineCuda(const cv::cuda::GpuMat& inputGpu, const cv::Mat& transform, cv::cuda::GpuMat& outputGpu);
#endif
    [[nodiscard]] cv::Mat frameToAlignmentGray(const cv::Mat& frameMat) const;
    [[nodiscard]] std::vector<cv::Point2f> detectStarCentroids(const cv::Mat& grayFrame) const;

private slots:
    void handleInputMessages();
    void processNextFrame();
};

#endif // INCLUDE_FEATURE_CAMERAFRAMEALIGNER_H_
