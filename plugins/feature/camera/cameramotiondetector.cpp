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
#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include "util/profiler.h"
#include "camera.h"
#include "cameramotiondetector.h"

namespace {

// Fraction of the full-image box covered by cloud in the (possibly downscaled) cloud mask
double cloudOverlapFraction(const CameraPipelineCloud& cloud, const cv::Rect& box)
{
    if (!cloud.m_valid || cloud.m_mask.empty() || (cloud.m_roi.width <= 0) || (cloud.m_roi.height <= 0)) {
        return 0.0;
    }

    const double scaleX = static_cast<double>(cloud.m_mask.cols) / cloud.m_roi.width;
    const double scaleY = static_cast<double>(cloud.m_mask.rows) / cloud.m_roi.height;
    const int x0 = std::max(0, static_cast<int>(std::floor((box.x - cloud.m_roi.x) * scaleX)));
    const int y0 = std::max(0, static_cast<int>(std::floor((box.y - cloud.m_roi.y) * scaleY)));
    const int x1 = std::min(cloud.m_mask.cols, static_cast<int>(std::ceil((box.x + box.width - cloud.m_roi.x) * scaleX)));
    const int y1 = std::min(cloud.m_mask.rows, static_cast<int>(std::ceil((box.y + box.height - cloud.m_roi.y) * scaleY)));
    if ((x1 <= x0) || (y1 <= y0)) {
        return 0.0;
    }

    const cv::Rect maskRect(x0, y0, x1 - x0, y1 - y0);
    return static_cast<double>(cv::countNonZero(cloud.m_mask(maskRect))) / maskRect.area();
}

} // namespace

CameraMotionDetector::CameraMotionDetector(Camera *camera) :
    m_camera(camera),
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

CameraMotionDetector::~CameraMotionDetector() = default;

void CameraMotionDetector::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraMotionDetector::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    CameraDetectionStage::applySettings(settings, settingsKeys, force);

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
#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
        m_cudaBgSubtractor = cv::Ptr<cv::cuda::BackgroundSubtractorMOG2>();
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

void CameraMotionDetector::captureActiveChanged(bool active)
{
    if (!active)
    {
        return;
    }

    m_bgSubtractor = cv::Ptr<cv::BackgroundSubtractor>();
#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
    m_cudaBgSubtractor = cv::Ptr<cv::cuda::BackgroundSubtractorMOG2>();
    invalidateCudaMotionCaches();
#endif
    m_lastMotionBoxes.clear();
    m_motionPersistenceRemaining = 0;
    m_motionConfirmCount = 0;
}

void CameraMotionDetector::processNewFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || !frame->hasImageData()) {
        return;
    }

    frame->m_motionBoxes.clear();

    if (!m_settings.m_motionDetect)
    {
        forwardFrame(frame);
        return;
    }

    const QSize frameSize = frame->imageSize();
    const cv::Size frameCvSize(frameSize.width(), frameSize.height());
    const cv::Rect detectionRoi = resolveDetectionRoi(frameCvSize);
#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
    const cv::cuda::GpuMat* cachedBgrGpu = frame->hasCudaBgrImage() ? &frame->m_cudaBgrImage : nullptr;
#endif

    cv::Mat bgrMat;
    cv::Mat motionDebugMask;
    bool useCPUMotionDetection = true;
    const CameraPipelineCloud* cloud =
        (m_settings.m_cloudFilterMotion && frame->m_cloud.m_valid) ? &frame->m_cloud : nullptr;

#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
    if (cachedBgrGpu
        && canUseCudaMotionDetection()
        && applyMotionDetectionCuda(
            bgrMat,
            cachedBgrGpu,
            detectionRoi,
            cloud,
            frame->m_motionBoxes,
            (m_settings.m_motionMaskView != CameraSettings::MotionMaskViewOff) ? &motionDebugMask : nullptr))
    {
        useCPUMotionDetection = false;
    }
#endif
    if (useCPUMotionDetection)
    {
        if (!frame->ensureCpuImageFromCuda()) {
            return;
        }

        QImage convertedRgb;
        const QImage& rgb = ensureRgb888(frame->m_image, convertedRgb);
        cv::Mat mat = wrapRgb888Image(rgb);
        cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);
        applyMotionDetection(
            bgrMat,
            detectionRoi,
            cloud,
            frame->m_motionBoxes,
            (m_settings.m_motionMaskView != CameraSettings::MotionMaskViewOff) ? &motionDebugMask : nullptr);
    }

    if (!motionDebugMask.empty())
    {
        cv::Mat maskCanvas = cv::Mat::zeros(frameCvSize, CV_8UC1);
        cv::Mat roiMask = motionDebugMask;
        if (motionDebugMask.size() != detectionRoi.size()) {
            cv::resize(motionDebugMask, roiMask, detectionRoi.size(), 0.0, 0.0, cv::INTER_NEAREST);
        }
        roiMask.copyTo(maskCanvas(detectionRoi));
        cv::cvtColor(maskCanvas, bgrMat, cv::COLOR_GRAY2BGR);
        frame->m_image = convertBgrToRgbImage(bgrMat);
        frame->clearCudaCache();
    }

    forwardFrame(frame);
}

cv::Ptr<cv::BackgroundSubtractor> CameraMotionDetector::createBackgroundSubtractor() const
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

#ifdef CAMERA_OPENCV_CUDA_MOTION_DETECTION
cv::Ptr<cv::cuda::BackgroundSubtractorMOG2> CameraMotionDetector::createCudaBackgroundSubtractor() const
{
    return cv::cuda::createBackgroundSubtractorMOG2(
        m_settings.m_motionHistory,
        m_settings.m_motionVarThreshold,
        m_settings.m_motionDetectShadows);
}

bool CameraMotionDetector::canUseCudaMotionDetection() const
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
            qDebug() << "CameraMotionDetector: CUDA motion detection requested, but only MOG2 is supported; using CPU path";
            warnedUnsupportedSettings = true;
        }
        return false;
    }

    if (cv::cuda::getCudaEnabledDeviceCount() <= 0)
    {
        if (!warnedNoDevice)
        {
            qWarning() << "CameraMotionDetector: CUDA motion detection requested, but no CUDA-enabled OpenCV device is available";
            warnedNoDevice = true;
        }
        return false;
    }

    return true;
}

void CameraMotionDetector::invalidateCudaMotionCaches()
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

const cv::cuda::GpuMat& CameraMotionDetector::cudaMotionExclusionMask(const cv::Rect& roi, const cv::Size& workSize)
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

bool CameraMotionDetector::applyMotionDetectionCuda(const cv::Mat& bgrMat, const cv::cuda::GpuMat* sourceBgrGpu, const cv::Rect& roi, const CameraPipelineCloud* cloud, QVector<QRect>& motionBoxes, cv::Mat* debugMask)
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
        m_cudaBgSubtractor->apply(motionInputGpu, fgMaskGpu, m_settings.m_motionLearningRate, m_cudaMotionStream);

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
                // Drop boxes that are mostly drifting cloud before they can seed the
                // confirmation/persistence counters
                if (cloud && (cloudOverlapFraction(*cloud, box) >= m_settings.m_cloudMotionOverlapThreshold)) {
                    continue;
                }
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
        qWarning() << "CameraMotionDetector: CUDA motion detection failed; falling back to CPU:" << error.what();
    }

    PROFILER_STOP(__FUNCTION__);
    return false;
}
#endif

void CameraMotionDetector::applyMotionDetection(const cv::Mat& bgrMat, const cv::Rect& roi, const CameraPipelineCloud* cloud, QVector<QRect>& motionBoxes, cv::Mat* debugMask)
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
    m_bgSubtractor->apply(motionInput, fgMask, m_settings.m_motionLearningRate);

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
            // Drop boxes that are mostly drifting cloud before they can seed the
            // confirmation/persistence counters
            if (cloud && (cloudOverlapFraction(*cloud, box) >= m_settings.m_cloudMotionOverlapThreshold)) {
                continue;
            }
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
