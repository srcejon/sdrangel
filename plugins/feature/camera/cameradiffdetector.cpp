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

#include <algorithm>

#include <QDebug>
#ifdef CAMERA_OPENCV_CUDA_DETECTION
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include "util/profiler.h"
#include "cameradiffdetector.h"
#include "camerapostprocessor.h"

CameraDiffDetector::CameraDiffDetector()
#ifdef CAMERA_OPENCV_CUDA_DETECTION
    :
    m_cudaDiffOpenFilterSize(0),
    m_cudaDiffOpenFilterType(-1),
    m_cudaDiffDilationFilterSize(0),
    m_cudaDiffDilationFilterType(-1),
    m_cudaDiffCloseFilterSize(0),
    m_cudaDiffCloseFilterType(-1)
#endif
{
}

CameraDiffDetector::~CameraDiffDetector() = default;

void CameraDiffDetector::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraDiffDetector::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    CameraDetectionStage::applySettings(settings, settingsKeys, force);

    if (force
        || settingsKeys.contains("diffMask")
        || settingsKeys.contains("diffThreshold")
        || settingsKeys.contains("diffMaskOpenSize")
        || settingsKeys.contains("dilationSize")
        || settingsKeys.contains("diffMaskHistoryFrames")
        || settingsKeys.contains("diffMaskCloseSize")
        || settingsKeys.contains("motionExclusionRects")
        || settingsKeys.contains("detectionRoiX")
        || settingsKeys.contains("detectionRoiY")
        || settingsKeys.contains("detectionRoiWidth")
        || settingsKeys.contains("detectionRoiHeight"))
    {
        m_diffMaskHistory.clear();
#ifdef CAMERA_OPENCV_CUDA_DETECTION
        m_cudaDiffMaskHistory.clear();
#endif
    }

    if ((force && !m_settings.m_diffMask)
        || (settingsKeys.contains("diffMask") && !m_settings.m_diffMask)
        || settingsKeys.contains("detectionRoiX")
        || settingsKeys.contains("detectionRoiY")
        || settingsKeys.contains("detectionRoiWidth")
        || settingsKeys.contains("detectionRoiHeight")) {
        m_lastInputFrame = CameraPipelineFrame();
    }
}

void CameraDiffDetector::captureActiveChanged(bool active)
{
    if (!active) {
        return;
    }

    m_lastInputFrame = CameraPipelineFrame();
    m_diffMaskHistory.clear();
#ifdef CAMERA_OPENCV_CUDA_DETECTION
    m_cudaDiffMaskHistory.clear();
#endif
}

void CameraDiffDetector::processNewFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || !frame->hasImageData()) {
        return;
    }

    CameraPipelineFrame inputFrameSnapshot(*frame);

    if (m_settings.m_diffMask
        && m_lastInputFrame.hasImageData()
        && (m_lastInputFrame.imageSize() == frame->imageSize()))
    {
        if (!frame->ensureCpuImageFromCuda()) {
            return;
        }
        m_lastInputFrame.ensureCpuImageFromCuda();

        QImage convertedRgb;
        const QImage& rgb = ensureRgb888(frame->m_image, convertedRgb);
        cv::Mat mat = wrapRgb888Image(rgb);
        cv::Mat bgrMat;
        cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);
        const cv::Rect detectionRoi = resolveDetectionRoi(bgrMat.size());
        bool useCPUDiffMask = true;

#ifdef CAMERA_OPENCV_CUDA_DETECTION
        if (canUseCudaDetection() && applyDiffMaskCuda(bgrMat, frame->hasCudaBgrImage() ? &frame->m_cudaBgrImage : nullptr, detectionRoi, m_lastInputFrame)){
            useCPUDiffMask = false;
        }
#endif
        if (useCPUDiffMask) {
            applyDiffMask(*frame, bgrMat, detectionRoi, m_lastInputFrame);
        }

        frame->m_image = convertBgrToRgbImage(bgrMat);
        frame->clearCudaCache();
    }

    m_lastInputFrame = inputFrameSnapshot;

    if (m_nextStageQueue) {
        m_nextStageQueue->push(CameraPostProcessor::MsgProcessFrame::create(frame));
    }
}

void CameraDiffDetector::applyDiffMask(CameraPipelineFrame& frame, cv::Mat& bgrMat, const cv::Rect& roi, const CameraPipelineFrame& diffReferenceFrame)
{
    PROFILER_START();

    QImage convertedPrevRgb;
    const QImage& prevRgb = ensureRgb888(diffReferenceFrame.m_image, convertedPrevRgb);
    cv::Mat prevMat = wrapRgb888Image(prevRgb);
    cv::Mat prevBgr;
    cv::cvtColor(prevMat, prevBgr, cv::COLOR_RGB2BGR);

    // Convert only the ROI to gray. Previously this converted the entire frame even though
    // the absdiff below only consumed the ROI — for a 25% sky-region ROI on a 4K frame
    // that's 4x more pixel work per call than necessary.
    cv::Mat gray, prevGray;
    cv::cvtColor(bgrMat(roi), gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(prevBgr(roi), prevGray, cv::COLOR_BGR2GRAY);
    cv::Mat diff;
    cv::absdiff(gray, prevGray, diff);
    cv::Mat mask;
    cv::threshold(diff, mask, m_settings.m_diffThreshold, 255, cv::THRESH_BINARY);

    if (m_settings.m_diffMaskOpenSize > 0)
    {
        const int openKsize = 2 * m_settings.m_diffMaskOpenSize + 1;
        const cv::Mat openKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(openKsize, openKsize));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, openKernel);
    }

    if (m_settings.m_dilationSize > 0)
    {
        const int ksize = 2 * m_settings.m_dilationSize + 1;
        const cv::Mat dilationKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
        cv::dilate(mask, mask, dilationKernel);
    }

    if (!m_diffMaskHistory.empty() &&
        (m_diffMaskHistory.front().size() != mask.size() || m_diffMaskHistory.front().type() != mask.type()))
    {
        m_diffMaskHistory.clear();
#ifdef CAMERA_OPENCV_CUDA_DETECTION
        m_cudaDiffMaskHistory.clear();
#endif
    }

    m_diffMaskHistory.push_back(mask.clone());

    const size_t historyFrames = static_cast<size_t>(std::max(1, m_settings.m_diffMaskHistoryFrames));
    while (m_diffMaskHistory.size() > historyFrames) {
        m_diffMaskHistory.pop_front();
    }

    cv::Mat combinedMask = m_diffMaskHistory.front().clone();
    for (size_t i = 1; i < m_diffMaskHistory.size(); ++i) {
        cv::bitwise_or(combinedMask, m_diffMaskHistory[i], combinedMask);
    }
    cv::bitwise_and(combinedMask, cachedExclusionMask(roi, combinedMask.size()), combinedMask);
    if (m_settings.m_diffMaskCloseSize > 0)
    {
        const int closeKsize = 2 * m_settings.m_diffMaskCloseSize + 1;
        const cv::Mat closeKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(closeKsize, closeKsize));
        cv::morphologyEx(combinedMask, combinedMask, cv::MORPH_CLOSE, closeKernel);
    }

    cv::Mat result = cv::Mat::zeros(bgrMat.size(), bgrMat.type());
    cv::Mat fullMask = cv::Mat::zeros(bgrMat.rows, bgrMat.cols, combinedMask.type());
    combinedMask.copyTo(fullMask(roi));
    cv::bitwise_and(bgrMat, bgrMat, result, fullMask);
    bgrMat = result;
    PROFILER_STOP(__FUNCTION__);
}

#ifdef CAMERA_OPENCV_CUDA_DETECTION
bool CameraDiffDetector::applyDiffMaskCuda(cv::Mat& bgrMat, const cv::cuda::GpuMat* sourceBgrGpu, const cv::Rect& roi, const CameraPipelineFrame& diffReferenceFrame)
{
    PROFILER_START();
    try
    {
        cv::cuda::GpuMat bgrGpu;
        cv::cuda::GpuMat prevBgrGpu;
        cv::cuda::GpuMat grayGpu;
        cv::cuda::GpuMat prevGrayGpu;

        if (sourceBgrGpu
            && !sourceBgrGpu->empty()
            && (sourceBgrGpu->size() == bgrMat.size())
            && (sourceBgrGpu->type() == bgrMat.type())) {
            bgrGpu = *sourceBgrGpu;
        } else {
            bgrGpu.upload(bgrMat, m_cudaDetectionStream);
        }

        if (diffReferenceFrame.hasCudaBgrImage()
            && (diffReferenceFrame.m_cudaBgrImage.size() == bgrGpu.size())
            && (diffReferenceFrame.m_cudaBgrImage.type() == bgrGpu.type()))
        {
            // Only reuse the cached GPU reference when it matches the current frame's
            // geometry; otherwise prevBgrGpu(roi) below could index out of bounds (e.g.
            // after a resolution change) and throw on every frame.
            prevBgrGpu = diffReferenceFrame.m_cudaBgrImage;
        }
        else
        {
            QImage convertedPrevRgb;
            const QImage& prevRgb = ensureRgb888(diffReferenceFrame.m_image, convertedPrevRgb);
            cv::Mat prevMat = wrapRgb888Image(prevRgb);
            cv::Mat prevBgr;
            cv::cvtColor(prevMat, prevBgr, cv::COLOR_RGB2BGR);
            prevBgrGpu.upload(prevBgr, m_cudaDetectionStream);
        }

        // See CPU comment: convert only the ROI rather than the full frame, since absdiff
        // only consumes the ROI.
        cv::cuda::cvtColor(bgrGpu(roi), grayGpu, cv::COLOR_BGR2GRAY, 0, m_cudaDetectionStream);
        cv::cuda::cvtColor(prevBgrGpu(roi), prevGrayGpu, cv::COLOR_BGR2GRAY, 0, m_cudaDetectionStream);

        cv::cuda::GpuMat diffGpu;
        cv::cuda::GpuMat maskGpu;
        cv::cuda::absdiff(grayGpu, prevGrayGpu, diffGpu, m_cudaDetectionStream);
        cv::cuda::threshold(diffGpu, maskGpu, m_settings.m_diffThreshold, 255.0, cv::THRESH_BINARY, m_cudaDetectionStream);

        if (m_settings.m_diffMaskOpenSize > 0)
        {
            const int openKsize = 2 * m_settings.m_diffMaskOpenSize + 1;
            cv::cuda::GpuMat openedGpu;
            cudaDiffOpenFilter(maskGpu.type(), openKsize)->apply(maskGpu, openedGpu, m_cudaDetectionStream);
            maskGpu = openedGpu;
        }

        if (m_settings.m_dilationSize > 0)
        {
            const int ksize = 2 * m_settings.m_dilationSize + 1;
            cv::cuda::GpuMat dilatedGpu;
            cudaDiffDilationFilter(maskGpu.type(), ksize)->apply(maskGpu, dilatedGpu, m_cudaDetectionStream);
            maskGpu = dilatedGpu;
        }

        if (!m_cudaDiffMaskHistory.empty() &&
            (m_cudaDiffMaskHistory.front().size() != maskGpu.size() || m_cudaDiffMaskHistory.front().type() != maskGpu.type()))
        {
            m_cudaDiffMaskHistory.clear();
        }

        m_cudaDiffMaskHistory.push_back(maskGpu.clone());

        const size_t historyFrames = static_cast<size_t>(std::max(1, m_settings.m_diffMaskHistoryFrames));
        while (m_cudaDiffMaskHistory.size() > historyFrames) {
            m_cudaDiffMaskHistory.pop_front();
        }

        cv::cuda::GpuMat combinedMaskGpu = m_cudaDiffMaskHistory.front().clone();
        for (size_t i = 1; i < m_cudaDiffMaskHistory.size(); ++i) {
            cv::cuda::bitwise_or(combinedMaskGpu, m_cudaDiffMaskHistory[i], combinedMaskGpu, cv::noArray(), m_cudaDetectionStream);
        }

        cv::cuda::bitwise_and(combinedMaskGpu, cudaDiffExclusionMask(roi, combinedMaskGpu.size()), combinedMaskGpu, cv::noArray(), m_cudaDetectionStream);

        if (m_settings.m_diffMaskCloseSize > 0)
        {
            const int closeKsize = 2 * m_settings.m_diffMaskCloseSize + 1;
            cv::cuda::GpuMat closedGpu;
            cudaDiffCloseFilter(combinedMaskGpu.type(), closeKsize)->apply(combinedMaskGpu, closedGpu, m_cudaDetectionStream);
            combinedMaskGpu = closedGpu;
        }

        cv::cuda::GpuMat fullMaskGpu(bgrGpu.size(), combinedMaskGpu.type());
        fullMaskGpu.setTo(cv::Scalar::all(0), m_cudaDetectionStream);
        combinedMaskGpu.copyTo(fullMaskGpu(roi), m_cudaDetectionStream);

        cv::cuda::GpuMat resultGpu;
        cv::cuda::bitwise_and(bgrGpu, bgrGpu, resultGpu, fullMaskGpu, m_cudaDetectionStream);
        resultGpu.download(bgrMat, m_cudaDetectionStream);
        m_cudaDetectionStream.waitForCompletion();
        PROFILER_STOP(__FUNCTION__);
        return true;
    }
    catch (const cv::Exception& error)
    {
        qWarning() << "CameraDiffDetector: CUDA diff mask failed; falling back to CPU:" << error.what();
    }

    PROFILER_STOP(__FUNCTION__);
    return false;
}
#endif

#ifdef CAMERA_OPENCV_CUDA_DETECTION
bool CameraDiffDetector::canUseCudaDetection() const
{
    static bool warnedNoDevice = false;

    if (!m_settings.m_postProcessUseCuda) {
        return false;
    }

    if (cv::cuda::getCudaEnabledDeviceCount() <= 0)
    {
        if (!warnedNoDevice)
        {
            qWarning() << "CameraDiffDetector: CUDA detection requested, but no CUDA-enabled OpenCV device is available";
            warnedNoDevice = true;
        }
        return false;
    }

    return true;
}

cv::Ptr<cv::cuda::Filter> CameraDiffDetector::cudaDiffOpenFilter(int inputType, int kernelSize)
{
    if (!m_cudaDiffOpenFilter
        || (m_cudaDiffOpenFilterType != inputType)
        || (m_cudaDiffOpenFilterSize != kernelSize))
    {
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
        m_cudaDiffOpenFilter = cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, inputType, kernel);
        m_cudaDiffOpenFilterType = inputType;
        m_cudaDiffOpenFilterSize = kernelSize;
    }

    return m_cudaDiffOpenFilter;
}

cv::Ptr<cv::cuda::Filter> CameraDiffDetector::cudaDiffDilationFilter(int inputType, int kernelSize)
{
    if (!m_cudaDiffDilationFilter
        || (m_cudaDiffDilationFilterType != inputType)
        || (m_cudaDiffDilationFilterSize != kernelSize))
    {
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
        m_cudaDiffDilationFilter = cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, inputType, kernel);
        m_cudaDiffDilationFilterType = inputType;
        m_cudaDiffDilationFilterSize = kernelSize;
    }

    return m_cudaDiffDilationFilter;
}

cv::Ptr<cv::cuda::Filter> CameraDiffDetector::cudaDiffCloseFilter(int inputType, int kernelSize)
{
    if (!m_cudaDiffCloseFilter
        || (m_cudaDiffCloseFilterType != inputType)
        || (m_cudaDiffCloseFilterSize != kernelSize))
    {
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
        m_cudaDiffCloseFilter = cv::cuda::createMorphologyFilter(cv::MORPH_CLOSE, inputType, kernel);
        m_cudaDiffCloseFilterType = inputType;
        m_cudaDiffCloseFilterSize = kernelSize;
    }

    return m_cudaDiffCloseFilter;
}

const cv::cuda::GpuMat& CameraDiffDetector::cudaDiffExclusionMask(const cv::Rect& roi, const cv::Size& workSize)
{
    if (m_cudaDiffExclusionMask.empty()
        || (m_cudaDiffExclusionRoi != roi)
        || (m_cudaDiffExclusionWorkSize != workSize)
        || (m_cudaDiffExclusionRects != m_settings.m_motionExclusionRects))
    {
        const cv::Mat exclusionMask = buildExclusionMask(roi, workSize);
        m_cudaDiffExclusionMask.upload(exclusionMask, m_cudaDetectionStream);
        m_cudaDiffExclusionRoi = roi;
        m_cudaDiffExclusionWorkSize = workSize;
        m_cudaDiffExclusionRects = m_settings.m_motionExclusionRects;
    }

    return m_cudaDiffExclusionMask;
}

#endif
