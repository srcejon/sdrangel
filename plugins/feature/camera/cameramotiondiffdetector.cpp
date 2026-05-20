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
#include <cmath>

#include <QDebug>
#ifdef CAMERA_OPENCV_CUDA_DETECTION
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include "util/profiler.h"
#include "cameramotiondiffdetector.h"
CameraMotionDiffDetector::CameraMotionDiffDetector() :
#ifdef CAMERA_OPENCV_CUDA_DETECTION
    m_cudaDiffOpenFilterSize(0),
    m_cudaDiffOpenFilterType(-1),
    m_cudaDiffDilationFilterSize(0),
    m_cudaDiffDilationFilterType(-1),
    m_cudaDiffCloseFilterSize(0),
    m_cudaDiffCloseFilterType(-1),
#endif
#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
    m_cudaMotionOpenFilterSize(0),
    m_cudaMotionOpenFilterType(-1),
    m_cudaMotionCloseFilterSize(0),
    m_cudaMotionCloseFilterType(-1),
#endif
    m_motionPersistenceRemaining(0),
    m_motionConfirmCount(0)
{
}

CameraMotionDiffDetector::~CameraMotionDiffDetector() = default;

void CameraMotionDiffDetector::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraMotionDiffDetector::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    CameraDetectionStage::applySettings(settings, settingsKeys, force);

    if (force
        || settingsKeys.contains("diffMask")
        || settingsKeys.contains("diffThreshold")
        || settingsKeys.contains("diffMaskOpenSize")
        || settingsKeys.contains("dilationSize")
        || settingsKeys.contains("diffMaskHistoryFrames")
        || settingsKeys.contains("diffMaskCloseSize")
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
        m_previousInputFrame = CameraPipelineFrame();
    }

    if (force
        || settingsKeys.contains("motionDetect")
        || settingsKeys.contains("postProcessUseCuda")
        || settingsKeys.contains("motionBackgroundSubtractor")
        || settingsKeys.contains("motionHistory")
        || settingsKeys.contains("motionVarThreshold")
        || settingsKeys.contains("motionLearningRate")
        || settingsKeys.contains("motionDownscale")
        || settingsKeys.contains("motionDetectShadows")
        || settingsKeys.contains("detectionRoiX")
        || settingsKeys.contains("detectionRoiY")
        || settingsKeys.contains("detectionRoiWidth")
        || settingsKeys.contains("detectionRoiHeight"))
    {
        m_bgSubtractor = cv::Ptr<cv::BackgroundSubtractor>();
        m_motionLastFgMaskRaw.release();
#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
        m_cudaBgSubtractor = cv::Ptr<cv::cuda::BackgroundSubtractorMOG2>();
        m_cudaMotionLastFgMaskRaw.release();
        invalidateCudaMotionCaches();
#endif
    }

    if (force
        || settingsKeys.contains("motionDetect")
        || settingsKeys.contains("postProcessUseCuda")
        || settingsKeys.contains("motionBackgroundSubtractor")
        || settingsKeys.contains("motionHistory")
        || settingsKeys.contains("motionVarThreshold")
        || settingsKeys.contains("motionLearningRate")
        || settingsKeys.contains("motionConfirmFrames")
        || settingsKeys.contains("motionDownscale")
        || settingsKeys.contains("motionDetectShadows")
        || settingsKeys.contains("motionOpenSize")
        || settingsKeys.contains("motionCloseSize")
        || settingsKeys.contains("motionPersistenceFrames")
        || settingsKeys.contains("minContourArea")
        || settingsKeys.contains("motionExclusionRects")
        || settingsKeys.contains("detectionRoiX")
        || settingsKeys.contains("detectionRoiY")
        || settingsKeys.contains("detectionRoiWidth")
        || settingsKeys.contains("detectionRoiHeight"))
    {
        m_lastMotionBoxes.clear();
        m_motionPersistenceRemaining = 0;
        m_motionConfirmCount = 0;
#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
        invalidateCudaMotionCaches();
#endif
    }
}

void CameraMotionDiffDetector::captureActiveChanged(bool active)
{
    if (!active) {
        return;
    }

    m_previousInputFrame = CameraPipelineFrame();
    m_lastInputFrame = CameraPipelineFrame();
    m_diffMaskHistory.clear();
#ifdef CAMERA_OPENCV_CUDA_DETECTION
    m_cudaDiffMaskHistory.clear();
#endif
    m_bgSubtractor = cv::Ptr<cv::BackgroundSubtractor>();
    m_motionLastFgMaskRaw.release();
#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
    m_cudaBgSubtractor = cv::Ptr<cv::cuda::BackgroundSubtractorMOG2>();
    m_cudaMotionLastFgMaskRaw.release();
    invalidateCudaMotionCaches();
#endif
    m_lastMotionBoxes.clear();
    m_motionPersistenceRemaining = 0;
    m_motionConfirmCount = 0;
}

void CameraMotionDiffDetector::processNewFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || frame->m_image.isNull()) {
        return;
    }

    CameraPipelineFrame inputFrameSnapshot(*frame);
    frame->m_motionBoxes.clear();

    QImage convertedRgb;
    const QImage& rgb = ensureRgb888(frame->m_image, convertedRgb);
    cv::Mat mat = wrapRgb888Image(rgb);
    cv::Mat bgrMat;
    cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);
    const cv::Rect detectionRoi = resolveDetectionRoi(bgrMat.size());
#if defined(CAMERA_OPENCV_CUDA_DETECTION) || defined(CAMERA_OPENCV_CUDA_MOTION_DETECTION)
    const cv::cuda::GpuMat* cachedBgrGpu = frame->hasCudaBgrImage() ? &frame->m_cudaBgrImage : nullptr;
#else
    const cv::cuda::GpuMat* cachedBgrGpu = nullptr;
#endif

    if (m_settings.m_diffMask && !m_lastInputFrame.m_image.isNull()
        && m_lastInputFrame.m_image.width() == frame->m_image.width()
        && m_lastInputFrame.m_image.height() == frame->m_image.height())
    {
        bool useCPUDiffMask = true;

#ifdef CAMERA_OPENCV_CUDA_DETECTION
        if (canUseCudaDetection() && applyDiffMaskCuda(bgrMat, frame->hasCudaBgrImage() ? &frame->m_cudaBgrImage : nullptr, detectionRoi, m_lastInputFrame)){
            useCPUDiffMask = false;
        }
#endif
        if (useCPUDiffMask) {
            applyDiffMask(*frame, bgrMat, detectionRoi, m_lastInputFrame);
        }
        cachedBgrGpu = nullptr;
    }

    if (m_settings.m_motionDetect)
    {
        cv::Mat motionDebugMask;
        bool useCPUMotionDetection = true;

#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
        if (canUseCudaMotionDetection()
            && applyMotionDetectionCuda(
                bgrMat,
                cachedBgrGpu,
                detectionRoi,
                frame->m_motionBoxes,
                true,
                (m_settings.m_motionMaskView != CameraSettings::MotionMaskViewOff) ? &motionDebugMask : nullptr))
        {
            useCPUMotionDetection = false;
        }
#endif
        if (useCPUMotionDetection)
        {
            applyMotionDetection(
                bgrMat,
                cachedBgrGpu,
                detectionRoi,
                frame->m_motionBoxes,
                true,
                (m_settings.m_motionMaskView != CameraSettings::MotionMaskViewOff) ? &motionDebugMask : nullptr);
        }

        if (!motionDebugMask.empty())
        {
            cv::Mat maskCanvas = cv::Mat::zeros(bgrMat.size(), CV_8UC1);
            cv::Mat roiMask = motionDebugMask;
            if (motionDebugMask.size() != detectionRoi.size()) {
                cv::resize(motionDebugMask, roiMask, detectionRoi.size(), 0.0, 0.0, cv::INTER_NEAREST);
            }
            roiMask.copyTo(maskCanvas(detectionRoi));
            cv::cvtColor(maskCanvas, bgrMat, cv::COLOR_GRAY2BGR);
        }
    }

    frame->m_image = convertBgrToRgbImage(bgrMat);
    frame->clearCudaCache();
    m_previousInputFrame = m_lastInputFrame;
    m_lastInputFrame = inputFrameSnapshot;

    forwardFrame(frame);
}


void CameraMotionDiffDetector::applyDiffMask(CameraPipelineFrame& frame, cv::Mat& bgrMat, const cv::Rect& roi, const CameraPipelineFrame& diffReferenceFrame)
{
    PROFILER_START();

    QImage convertedPrevRgb;
    const QImage& prevRgb = ensureRgb888(diffReferenceFrame.m_image, convertedPrevRgb);
    cv::Mat prevMat = wrapRgb888Image(prevRgb);
    cv::Mat prevBgr;
    cv::cvtColor(prevMat, prevBgr, cv::COLOR_RGB2BGR);

    cv::Mat gray, prevGray;
    cv::cvtColor(bgrMat, gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(prevBgr, prevGray, cv::COLOR_BGR2GRAY);
    cv::Mat diff;
    cv::absdiff(gray(roi), prevGray(roi), diff);
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
bool CameraMotionDiffDetector::applyDiffMaskCuda(cv::Mat& bgrMat, const cv::cuda::GpuMat* sourceBgrGpu, const cv::Rect& roi, const CameraPipelineFrame& diffReferenceFrame)
{
    PROFILER_START();
    try
    {
        cv::cuda::GpuMat bgrGpu;
        cv::cuda::GpuMat prevBgrGpu;
        cv::cuda::GpuMat grayGpu;
        cv::cuda::GpuMat prevGrayGpu;

        if (sourceBgrGpu && !sourceBgrGpu->empty()) {
            bgrGpu = *sourceBgrGpu;
        } else {
            bgrGpu.upload(bgrMat, m_cudaDetectionStream);
        }

        if (diffReferenceFrame.hasCudaBgrImage())
        {
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

        cv::cuda::cvtColor(bgrGpu, grayGpu, cv::COLOR_BGR2GRAY, 0, m_cudaDetectionStream);
        cv::cuda::cvtColor(prevBgrGpu, prevGrayGpu, cv::COLOR_BGR2GRAY, 0, m_cudaDetectionStream);

        cv::cuda::GpuMat diffGpu;
        cv::cuda::GpuMat maskGpu;
        cv::cuda::absdiff(grayGpu(roi), prevGrayGpu(roi), diffGpu, m_cudaDetectionStream);
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

        cv::cuda::GpuMat fullMaskGpu(bgrMat.size(), combinedMaskGpu.type());
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
        qWarning() << "CameraMotionDiffDetector: CUDA diff mask failed; falling back to CPU:" << error.what();
    }

    PROFILER_STOP(__FUNCTION__);
    return false;
}
#endif

cv::Ptr<cv::BackgroundSubtractor> CameraMotionDiffDetector::createBackgroundSubtractor() const
{
    if (m_settings.m_motionBackgroundSubtractor == CameraSettings::MotionBackgroundSubtractorKNN)
    {
        return cv::createBackgroundSubtractorKNN(
            m_settings.m_motionHistory,
            m_settings.m_motionVarThreshold,
            m_settings.m_motionDetectShadows);
    }

    return cv::createBackgroundSubtractorMOG2(
        m_settings.m_motionHistory,
        m_settings.m_motionVarThreshold,
        m_settings.m_motionDetectShadows);
}

#ifdef CAMERA_OPENCV_CUDA_DETECTION
bool CameraMotionDiffDetector::canUseCudaDetection() const
{
    static bool warnedNoDevice = false;

    if (!m_settings.m_postProcessUseCuda) {
        return false;
    }

    if (cv::cuda::getCudaEnabledDeviceCount() <= 0)
    {
        if (!warnedNoDevice)
        {
            qWarning() << "CameraMotionDiffDetector: CUDA detection requested, but no CUDA-enabled OpenCV device is available";
            warnedNoDevice = true;
        }
        return false;
    }

    return true;
}

cv::Ptr<cv::cuda::Filter> CameraMotionDiffDetector::cudaDiffOpenFilter(int inputType, int kernelSize)
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

cv::Ptr<cv::cuda::Filter> CameraMotionDiffDetector::cudaDiffDilationFilter(int inputType, int kernelSize)
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

cv::Ptr<cv::cuda::Filter> CameraMotionDiffDetector::cudaDiffCloseFilter(int inputType, int kernelSize)
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

const cv::cuda::GpuMat& CameraMotionDiffDetector::cudaDiffExclusionMask(const cv::Rect& roi, const cv::Size& workSize)
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

cv::Ptr<cv::cuda::BackgroundSubtractorMOG2> CameraMotionDiffDetector::createCudaBackgroundSubtractor() const
{
    return cv::cuda::createBackgroundSubtractorMOG2(
        m_settings.m_motionHistory,
        m_settings.m_motionVarThreshold,
        m_settings.m_motionDetectShadows);
}

bool CameraMotionDiffDetector::canUseCudaMotionDetection() const
{
    static bool warnedNoDevice = false;
    static bool warnedUnsupportedSettings = false;

    if (!m_settings.m_postProcessUseCuda) {
        return false;
    }

    if (m_settings.m_motionBackgroundSubtractor != CameraSettings::MotionBackgroundSubtractorMOG2)
    {
        if (!warnedUnsupportedSettings)
        {
            qDebug() << "CameraMotionDiffDetector: CUDA motion detection requested, but only MOG2 is supported; using CPU path";
            warnedUnsupportedSettings = true;
        }
        return false;
    }

    if (cv::cuda::getCudaEnabledDeviceCount() <= 0)
    {
        if (!warnedNoDevice)
        {
            qWarning() << "CameraMotionDiffDetector: CUDA motion detection requested, but no CUDA-enabled OpenCV device is available";
            warnedNoDevice = true;
        }
        return false;
    }

    return true;
}

void CameraMotionDiffDetector::invalidateCudaMotionCaches()
{
    m_cudaMotionOpenFilter.release();
    m_cudaMotionCloseFilter.release();
    m_cudaMotionOpenFilterSize = 0;
    m_cudaMotionOpenFilterType = -1;
    m_cudaMotionCloseFilterSize = 0;
    m_cudaMotionCloseFilterType = -1;
    m_cudaMotionExclusionMask.release();
    m_cudaMotionExclusionRoi = cv::Rect();
    m_cudaMotionExclusionWorkSize = cv::Size();
    m_cudaMotionExclusionRects.clear();
}

const cv::cuda::GpuMat& CameraMotionDiffDetector::cudaMotionExclusionMask(const cv::Rect& roi, const cv::Size& workSize)
{
    if (m_cudaMotionExclusionMask.empty()
        || (m_cudaMotionExclusionRoi != roi)
        || (m_cudaMotionExclusionWorkSize != workSize)
        || (m_cudaMotionExclusionRects != m_settings.m_motionExclusionRects))
    {
        const cv::Mat exclusionMask = buildExclusionMask(roi, workSize);
        m_cudaMotionExclusionMask.upload(exclusionMask, m_cudaMotionStream);
        m_cudaMotionExclusionRoi = roi;
        m_cudaMotionExclusionWorkSize = workSize;
        m_cudaMotionExclusionRects = m_settings.m_motionExclusionRects;
    }

    return m_cudaMotionExclusionMask;
}

bool CameraMotionDiffDetector::applyMotionDetectionCuda(const cv::Mat& bgrMat, const cv::cuda::GpuMat* sourceBgrGpu, const cv::Rect& roi, QVector<QRect>& motionBoxes, bool updateBackgroundModel, cv::Mat* debugMask)
{
    PROFILER_START();

    try
    {
        if (!m_cudaBgSubtractor) {
            m_cudaBgSubtractor = createCudaBackgroundSubtractor();
        }

        const double downscale = m_settings.m_motionDownscale;
        cv::cuda::GpuMat motionInputGpu;
        if (sourceBgrGpu && !sourceBgrGpu->empty())
        {
            motionInputGpu = (*sourceBgrGpu)(roi);
        }
        else
        {
            cv::Mat motionInput = bgrMat(roi);
            motionInputGpu.upload(motionInput, m_cudaMotionStream);
        }

        if (downscale < 0.999)
        {
            const cv::Size downscaledSize(
                std::max(1, static_cast<int>(std::lround(roi.width * downscale))),
                std::max(1, static_cast<int>(std::lround(roi.height * downscale))));
            cv::cuda::GpuMat downscaledInputGpu;
            cv::cuda::resize(motionInputGpu, downscaledInputGpu, downscaledSize, 0.0, 0.0, cv::INTER_LINEAR, m_cudaMotionStream);
            motionInputGpu = downscaledInputGpu;
        }

        cv::cuda::GpuMat fgMaskGpu;
        if (updateBackgroundModel)
        {
            m_cudaBgSubtractor->apply(motionInputGpu, fgMaskGpu, m_settings.m_motionLearningRate, m_cudaMotionStream);
            fgMaskGpu.copyTo(m_cudaMotionLastFgMaskRaw, m_cudaMotionStream);
        }
        else
        {
            fgMaskGpu = m_cudaMotionLastFgMaskRaw;
        }

        if (fgMaskGpu.empty())
        {
            m_cudaBgSubtractor->apply(motionInputGpu, fgMaskGpu, 0.0, m_cudaMotionStream);
        }

        if (debugMask && (m_settings.m_motionMaskView == CameraSettings::MotionMaskViewRaw))
        {
            fgMaskGpu.download(*debugMask, m_cudaMotionStream);
        }

        cv::cuda::GpuMat thresholdMaskGpu;
        cv::cuda::threshold(fgMaskGpu, thresholdMaskGpu, 200.0, 255.0, cv::THRESH_BINARY, m_cudaMotionStream);

        cv::cuda::bitwise_and(thresholdMaskGpu, cudaMotionExclusionMask(roi, thresholdMaskGpu.size()), thresholdMaskGpu, cv::noArray(), m_cudaMotionStream);

        if (debugMask && (m_settings.m_motionMaskView == CameraSettings::MotionMaskViewThresholded))
        {
            thresholdMaskGpu.download(*debugMask, m_cudaMotionStream);
        }

        if (m_settings.m_motionOpenSize > 0)
        {
            const int ksize = 2 * m_settings.m_motionOpenSize + 1;
            if (!m_cudaMotionOpenFilter
                || (m_cudaMotionOpenFilterSize != ksize)
                || (m_cudaMotionOpenFilterType != thresholdMaskGpu.type()))
            {
                const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
                m_cudaMotionOpenFilter = cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, thresholdMaskGpu.type(), kernel);
                m_cudaMotionOpenFilterSize = ksize;
                m_cudaMotionOpenFilterType = thresholdMaskGpu.type();
            }
            cv::cuda::GpuMat openedGpu;
            m_cudaMotionOpenFilter->apply(thresholdMaskGpu, openedGpu, m_cudaMotionStream);
            thresholdMaskGpu = openedGpu;
        }
        if (debugMask && (m_settings.m_motionMaskView == CameraSettings::MotionMaskViewOpened))
        {
            thresholdMaskGpu.download(*debugMask, m_cudaMotionStream);
        }

        if (m_settings.m_motionCloseSize > 0)
        {
            const int ksize = 2 * m_settings.m_motionCloseSize + 1;
            if (!m_cudaMotionCloseFilter
                || (m_cudaMotionCloseFilterSize != ksize)
                || (m_cudaMotionCloseFilterType != thresholdMaskGpu.type()))
            {
                const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
                m_cudaMotionCloseFilter = cv::cuda::createMorphologyFilter(cv::MORPH_CLOSE, thresholdMaskGpu.type(), kernel);
                m_cudaMotionCloseFilterSize = ksize;
                m_cudaMotionCloseFilterType = thresholdMaskGpu.type();
            }
            cv::cuda::GpuMat closedGpu;
            m_cudaMotionCloseFilter->apply(thresholdMaskGpu, closedGpu, m_cudaMotionStream);
            thresholdMaskGpu = closedGpu;
        }
        if (debugMask && (m_settings.m_motionMaskView == CameraSettings::MotionMaskViewClosed)) {
            thresholdMaskGpu.download(*debugMask, m_cudaMotionStream);
        }

        cv::Mat fgMask;
        thresholdMaskGpu.download(fgMask, m_cudaMotionStream);
        m_cudaMotionStream.waitForCompletion();
        if (debugMask && (m_settings.m_motionMaskView == CameraSettings::MotionMaskViewFinal)) {
            *debugMask = fgMask.clone();
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(fgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        QVector<QRect> boxes;
        boxes.reserve(static_cast<qsizetype>(contours.size()));
        const double scaledMinArea = static_cast<double>(m_settings.m_minContourArea) * downscale * downscale;
        for (const auto& contour : contours)
        {
            if (cv::contourArea(contour) >= scaledMinArea)
            {
                cv::Rect box = cv::boundingRect(contour);
                if (downscale < 0.999)
                {
                    box.x = static_cast<int>(std::floor(box.x / downscale));
                    box.y = static_cast<int>(std::floor(box.y / downscale));
                    box.width = std::max(1, static_cast<int>(std::ceil(box.width / downscale)));
                    box.height = std::max(1, static_cast<int>(std::ceil(box.height / downscale)));
                }
                box.x += roi.x;
                box.y += roi.y;
                boxes.append(QRect(box.x, box.y, box.width, box.height));
            }
        }

        if (!boxes.isEmpty())
        {
            m_motionConfirmCount = std::min(m_settings.m_motionConfirmFrames, m_motionConfirmCount + 1);
            if (m_motionConfirmCount < m_settings.m_motionConfirmFrames) {
                boxes.clear();
            }
        }
        else
        {
            m_motionConfirmCount = 0;
        }

        if (!boxes.isEmpty())
        {
            m_lastMotionBoxes = boxes;
            m_motionPersistenceRemaining = m_settings.m_motionPersistenceFrames;
        }
        else if ((m_motionPersistenceRemaining > 0) && !m_lastMotionBoxes.isEmpty())
        {
            boxes = m_lastMotionBoxes;
            --m_motionPersistenceRemaining;
        }
        else
        {
            m_lastMotionBoxes.clear();
            m_motionPersistenceRemaining = 0;
        }

        motionBoxes = boxes;
        PROFILER_STOP(__FUNCTION__);
        return true;
    }
    catch (const cv::Exception& error)
    {
        qWarning() << "CameraMotionDiffDetector: CUDA motion detection failed; falling back to CPU:" << error.what();
    }

    PROFILER_STOP(__FUNCTION__);
    return false;
}
#endif

void CameraMotionDiffDetector::applyMotionDetection(const cv::Mat& bgrMat, const cv::cuda::GpuMat* sourceBgrGpu, const cv::Rect& roi, QVector<QRect>& motionBoxes, bool updateBackgroundModel, cv::Mat* debugMask)
{
    PROFILER_START();

    if (!m_bgSubtractor) {
        m_bgSubtractor = createBackgroundSubtractor();
    }

    const double downscale = m_settings.m_motionDownscale;
    cv::Mat motionInput = bgrMat(roi);
    cv::Mat downscaledInput;
    if (downscale < 0.999)
    {
        const cv::Size downscaledSize(
            std::max(1, static_cast<int>(std::lround(roi.width * downscale))),
            std::max(1, static_cast<int>(std::lround(roi.height * downscale))));
        cv::resize(motionInput, downscaledInput, downscaledSize, 0.0, 0.0, cv::INTER_AREA);
        motionInput = downscaledInput;
    }

    cv::Mat fgMask;
    if (updateBackgroundModel)
    {
        m_bgSubtractor->apply(motionInput, fgMask, m_settings.m_motionLearningRate);
        m_motionLastFgMaskRaw = fgMask.clone();
    }
    else
    {
        fgMask = m_motionLastFgMaskRaw.clone();
    }

    if (fgMask.empty()) {
        m_bgSubtractor->apply(motionInput, fgMask, 0.0);
    }
    if (debugMask && (m_settings.m_motionMaskView == CameraSettings::MotionMaskViewRaw)) {
        *debugMask = fgMask.clone();
    }
    cv::threshold(fgMask, fgMask, 200, 255, cv::THRESH_BINARY);
    cv::bitwise_and(fgMask, cachedExclusionMask(roi, fgMask.size()), fgMask);
    if (debugMask && (m_settings.m_motionMaskView == CameraSettings::MotionMaskViewThresholded)) {
        *debugMask = fgMask.clone();
    }

    if (m_settings.m_motionOpenSize > 0)
    {
        const int ksize = 2 * m_settings.m_motionOpenSize + 1;
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
        cv::morphologyEx(fgMask, fgMask, cv::MORPH_OPEN, kernel);
    }
    if (debugMask && (m_settings.m_motionMaskView == CameraSettings::MotionMaskViewOpened)) {
        *debugMask = fgMask.clone();
    }

    if (m_settings.m_motionCloseSize > 0)
    {
        const int ksize = 2 * m_settings.m_motionCloseSize + 1;
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
        cv::morphologyEx(fgMask, fgMask, cv::MORPH_CLOSE, kernel);
    }
    if (debugMask && (m_settings.m_motionMaskView == CameraSettings::MotionMaskViewClosed)) {
        *debugMask = fgMask.clone();
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (debugMask && (m_settings.m_motionMaskView == CameraSettings::MotionMaskViewFinal)) {
        *debugMask = fgMask.clone();
    }

    QVector<QRect> boxes;
    boxes.reserve(static_cast<qsizetype>(contours.size()));
    const double scaledMinArea = static_cast<double>(m_settings.m_minContourArea) * downscale * downscale;
    for (const auto& contour : contours)
    {
        if (cv::contourArea(contour) >= scaledMinArea) {
            cv::Rect box = cv::boundingRect(contour);
            if (downscale < 0.999)
            {
                box.x = static_cast<int>(std::floor(box.x / downscale));
                box.y = static_cast<int>(std::floor(box.y / downscale));
                box.width = std::max(1, static_cast<int>(std::ceil(box.width / downscale)));
                box.height = std::max(1, static_cast<int>(std::ceil(box.height / downscale)));
            }
            box.x += roi.x;
            box.y += roi.y;
            boxes.append(QRect(box.x, box.y, box.width, box.height));
        }
    }

    if (!boxes.isEmpty()) {
        m_motionConfirmCount = std::min(m_settings.m_motionConfirmFrames, m_motionConfirmCount + 1);
        if (m_motionConfirmCount < m_settings.m_motionConfirmFrames) {
            boxes.clear();
        }
    } else {
        m_motionConfirmCount = 0;
    }

    if (!boxes.isEmpty()) {
        m_lastMotionBoxes = boxes;
        m_motionPersistenceRemaining = m_settings.m_motionPersistenceFrames;
    } else if ((m_motionPersistenceRemaining > 0) && !m_lastMotionBoxes.isEmpty()) {
        boxes = m_lastMotionBoxes;
        --m_motionPersistenceRemaining;
    } else {
        m_lastMotionBoxes.clear();
        m_motionPersistenceRemaining = 0;
    }

    motionBoxes = boxes;
    PROFILER_STOP(__FUNCTION__);
}

