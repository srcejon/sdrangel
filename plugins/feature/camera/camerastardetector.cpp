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
#include "cameraplatesolver.h"
#include "camerastardetector.h"
CameraStarDetector::CameraStarDetector()
#ifdef CAMERA_OPENCV_CUDA_DETECTION
    :
    m_cudaStarSmallBlurFilterType(-1),
    m_cudaStarBackgroundBlurFilterType(-1),
    m_cudaStarBackgroundBlurFilterSize(0)
#endif
{
}

CameraStarDetector::~CameraStarDetector() = default;

#ifdef CAMERA_OPENCV_CUDA_DETECTION
bool CameraStarDetector::canUseCudaDetection() const
{
    static bool warnedNoDevice = false;

    if (!m_settings.m_postProcessUseCuda) {
        return false;
    }

    if (cv::cuda::getCudaEnabledDeviceCount() <= 0)
    {
        if (!warnedNoDevice)
        {
            qWarning() << "CameraStarDetector: CUDA detection requested, but no CUDA-enabled OpenCV device is available";
            warnedNoDevice = true;
        }
        return false;
    }

    return true;
}

cv::Ptr<cv::cuda::Filter> CameraStarDetector::cudaStarSmallBlurFilter(int inputType) const
{
    if (!m_cudaStarSmallBlurFilter || (m_cudaStarSmallBlurFilterType != inputType))
    {
        m_cudaStarSmallBlurFilter = cv::cuda::createGaussianFilter(
            inputType, inputType, cv::Size(3, 3), 0.0, 0.0);
        m_cudaStarSmallBlurFilterType = inputType;
    }

    return m_cudaStarSmallBlurFilter;
}

cv::Ptr<cv::cuda::Filter> CameraStarDetector::cudaStarBackgroundBlurFilter(int inputType, int kernelSize) const
{
    if (!m_cudaStarBackgroundBlurFilter
        || (m_cudaStarBackgroundBlurFilterType != inputType)
        || (m_cudaStarBackgroundBlurFilterSize != kernelSize))
    {
        m_cudaStarBackgroundBlurFilter = cv::cuda::createGaussianFilter(
            inputType, inputType, cv::Size(kernelSize, kernelSize), 0.0, 0.0);
        m_cudaStarBackgroundBlurFilterType = inputType;
        m_cudaStarBackgroundBlurFilterSize = kernelSize;
    }

    return m_cudaStarBackgroundBlurFilter;
}

const cv::cuda::GpuMat& CameraStarDetector::cudaStarExclusionMask(const cv::Rect& roi, const cv::Size& workSize) const
{
    if (m_cudaStarExclusionMask.empty()
        || (m_cudaStarExclusionRoi != roi)
        || (m_cudaStarExclusionWorkSize != workSize)
        || (m_cudaStarExclusionRects != m_settings.m_motionExclusionRects))
    {
        const cv::Mat exclusionMask = buildExclusionMask(roi, workSize);
        m_cudaStarExclusionMask.upload(exclusionMask, m_cudaDetectionStream);
        m_cudaStarExclusionRoi = roi;
        m_cudaStarExclusionWorkSize = workSize;
        m_cudaStarExclusionRects = m_settings.m_motionExclusionRects;
    }

    return m_cudaStarExclusionMask;
}
#endif


void CameraStarDetector::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraStarDetector::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;
    CameraDetectionStage::applySettings(settings, settingsKeys, force);
}

void CameraStarDetector::captureActiveChanged(bool active)
{
    (void) active;
}

void CameraStarDetector::processNewFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || frame->m_image.isNull()) {
        return;
    }

    frame->m_starDetections.clear();
    frame->m_plateSolved = false;
    frame->m_plateSolvedMatches = 0;
    frame->m_plateSolveDetectedStarsConsidered = 0;
    frame->m_plateSolveCatalogStarsLoaded = 0;
    frame->m_plateSolveCatalogCandidateStars = 0;
    frame->m_plateSolveOutlierStars = 0;
    frame->m_plateSolveRmsError = 0.0f;
    frame->m_plateSolveMaxError = 0.0f;
    frame->m_plateSolveAzimuth = 0.0f;
    frame->m_plateSolveElevation = 0.0f;
    frame->m_plateSolveRoll = 0.0f;
    frame->m_plateSolveFov = 0.0f;
    frame->m_plateSolveCenterOffsetX = 0.0f;
    frame->m_plateSolveCenterOffsetY = 0.0f;
    frame->m_plateSolveDistortionK1 = 0.0f;
    frame->m_plateSolveCatalogSource.clear();

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

    if (m_settings.m_starDetect)
    {
        cv::Mat starDebugMask;
        applyStarDetection(
            bgrMat,
            cachedBgrGpu,
            detectionRoi,
            frame->m_starDetections,
            (m_settings.m_starDebugView != CameraSettings::StarDebugViewOff) ? &starDebugMask : nullptr);

        if (!starDebugMask.empty())
        {
            cv::Mat maskCanvas = cv::Mat::zeros(bgrMat.size(), starDebugMask.type());
            cv::Mat roiMask = starDebugMask;
            if (starDebugMask.size() != detectionRoi.size()) {
                cv::resize(starDebugMask, roiMask, detectionRoi.size(), 0.0, 0.0, cv::INTER_NEAREST);
            }
            roiMask.copyTo(maskCanvas(detectionRoi));
            if (maskCanvas.channels() == 1) {
                cv::cvtColor(maskCanvas, bgrMat, cv::COLOR_GRAY2BGR);
            } else {
                bgrMat = maskCanvas;
            }
            frame->m_image = convertBgrToRgbImage(bgrMat);
            frame->clearCudaCache();
        }
    }

    if (!frame->m_starDetections.isEmpty())
    {
        const CameraPlateSolveResult plateSolveResult = CameraPlateSolver::solve(
            m_settings,
            frame->m_image.size(),
            frame->m_captureDateTime,
            frame->m_starDetections);
        frame->m_plateSolved = plateSolveResult.m_solved;
        frame->m_plateSolvedMatches = plateSolveResult.m_matchedStars;
        frame->m_plateSolveDetectedStarsConsidered = plateSolveResult.m_detectedStarsConsidered;
        frame->m_plateSolveCatalogStarsLoaded = plateSolveResult.m_catalogStarsLoaded;
        frame->m_plateSolveCatalogCandidateStars = plateSolveResult.m_catalogCandidateStars;
        frame->m_plateSolveOutlierStars = plateSolveResult.m_outlierStars;
        frame->m_plateSolveRmsError = static_cast<float>(plateSolveResult.m_rmsErrorPixels);
        frame->m_plateSolveMaxError = static_cast<float>(plateSolveResult.m_maxErrorPixels);
        frame->m_plateSolveAzimuth = static_cast<float>(plateSolveResult.m_azimuthDegrees);
        frame->m_plateSolveElevation = static_cast<float>(plateSolveResult.m_elevationDegrees);
        frame->m_plateSolveRoll = static_cast<float>(plateSolveResult.m_rollDegrees);
        frame->m_plateSolveFov = static_cast<float>(plateSolveResult.m_fovDegrees);
        frame->m_plateSolveCenterOffsetX = static_cast<float>(plateSolveResult.m_centerOffsetXPixels);
        frame->m_plateSolveCenterOffsetY = static_cast<float>(plateSolveResult.m_centerOffsetYPixels);
        frame->m_plateSolveDistortionK1 = static_cast<float>(plateSolveResult.m_distortionK1);
        frame->m_plateSolveCatalogSource = plateSolveResult.m_catalogSource;
    }

    forwardFrame(frame);
}

void CameraStarDetector::applyStarPreprocessing(const cv::Mat& bgrMat, const cv::cuda::GpuMat* sourceBgrGpu, const cv::Rect& roi, cv::Mat& gray, cv::Mat& residual, cv::Mat& thresholdMask, cv::Mat* debugMask) const
{
    PROFILER_START();

    cv::cvtColor(bgrMat(roi), gray, cv::COLOR_BGR2GRAY);

    cv::Mat blurredGray;
    cv::GaussianBlur(gray, blurredGray, cv::Size(3, 3), 0.0, 0.0);

    int maxKernelDimension = std::min(gray.cols, gray.rows);
    if ((maxKernelDimension % 2) == 0) {
        --maxKernelDimension;
    }
    maxKernelDimension = std::max(1, maxKernelDimension);
    const int backgroundKernelSize = std::min(2 * m_settings.m_starBackgroundBlur + 1, maxKernelDimension);
    cv::Mat background;
    cv::GaussianBlur(blurredGray, background, cv::Size(backgroundKernelSize, backgroundKernelSize), 0.0, 0.0);
    if (debugMask && (m_settings.m_starDebugView == CameraSettings::StarDebugViewBackground)) {
        *debugMask = background.clone();
    }

    cv::subtract(blurredGray, background, residual);
    if (debugMask && (m_settings.m_starDebugView == CameraSettings::StarDebugViewResidual))
    {
        double minValue = 0.0;
        double maxValue = 0.0;
        cv::minMaxLoc(residual, &minValue, &maxValue);
        if (maxValue > minValue) {
            cv::normalize(residual, *debugMask, 0, 255, cv::NORM_MINMAX);
            debugMask->convertTo(*debugMask, CV_8UC1);
        } else {
            *debugMask = residual.clone();
        }
    }

    cv::threshold(residual, thresholdMask, m_settings.m_starThreshold, 255, cv::THRESH_BINARY);
    cv::bitwise_and(thresholdMask, cachedExclusionMask(roi, thresholdMask.size()), thresholdMask);
    if (debugMask && (m_settings.m_starDebugView == CameraSettings::StarDebugViewThresholded)) {
        *debugMask = thresholdMask.clone();
    }
    PROFILER_STOP(__FUNCTION__);
}

#ifdef CAMERA_OPENCV_CUDA_DETECTION
bool CameraStarDetector::applyStarPreprocessingCuda(const cv::Mat& bgrMat, const cv::cuda::GpuMat* sourceBgrGpu, const cv::Rect& roi, cv::Mat& gray, cv::Mat& residual, cv::Mat& thresholdMask, cv::Mat* debugMask) const
{
    PROFILER_START();

    try
    {
        cv::cuda::GpuMat bgrGpu;
        cv::cuda::GpuMat grayGpu;
        cv::cuda::GpuMat blurredGrayGpu;
        cv::cuda::GpuMat backgroundGpu;
        cv::cuda::GpuMat residualGpu;
        cv::cuda::GpuMat thresholdMaskGpu;

        if (sourceBgrGpu && !sourceBgrGpu->empty()) {
            bgrGpu = (*sourceBgrGpu)(roi);
        } else {
            bgrGpu.upload(bgrMat(roi), m_cudaDetectionStream);
        }
        cv::cuda::cvtColor(bgrGpu, grayGpu, cv::COLOR_BGR2GRAY, 0, m_cudaDetectionStream);

        cudaStarSmallBlurFilter(grayGpu.type())->apply(grayGpu, blurredGrayGpu, m_cudaDetectionStream);

        int maxKernelDimension = std::min(grayGpu.cols, grayGpu.rows);
        if ((maxKernelDimension % 2) == 0) {
            --maxKernelDimension;
        }
        maxKernelDimension = std::max(1, maxKernelDimension);
        const int backgroundKernelSize = std::min(2 * m_settings.m_starBackgroundBlur + 1, maxKernelDimension);
        cudaStarBackgroundBlurFilter(blurredGrayGpu.type(), backgroundKernelSize)->apply(blurredGrayGpu, backgroundGpu, m_cudaDetectionStream);

        if (debugMask && (m_settings.m_starDebugView == CameraSettings::StarDebugViewBackground)) {
            backgroundGpu.download(*debugMask, m_cudaDetectionStream);
        }

        cv::cuda::subtract(blurredGrayGpu, backgroundGpu, residualGpu, cv::noArray(), -1, m_cudaDetectionStream);
        residualGpu.download(residual, m_cudaDetectionStream);

        cv::cuda::threshold(residualGpu, thresholdMaskGpu, m_settings.m_starThreshold, 255.0, cv::THRESH_BINARY, m_cudaDetectionStream);
        cv::cuda::bitwise_and(thresholdMaskGpu, cudaStarExclusionMask(roi, thresholdMaskGpu.size()), thresholdMaskGpu, cv::noArray(), m_cudaDetectionStream);
        thresholdMaskGpu.download(thresholdMask, m_cudaDetectionStream);

        grayGpu.download(gray, m_cudaDetectionStream);
        m_cudaDetectionStream.waitForCompletion();

        if (debugMask && (m_settings.m_starDebugView == CameraSettings::StarDebugViewResidual))
        {
            double minValue = 0.0;
            double maxValue = 0.0;
            cv::minMaxLoc(residual, &minValue, &maxValue);
            if (maxValue > minValue)
            {
                cv::normalize(residual, *debugMask, 0, 255, cv::NORM_MINMAX);
                debugMask->convertTo(*debugMask, CV_8UC1);
            }
            else
            {
                *debugMask = residual.clone();
            }
        }
        else if (debugMask && (m_settings.m_starDebugView == CameraSettings::StarDebugViewThresholded))
        {
            *debugMask = thresholdMask.clone();
        }

        PROFILER_STOP(__FUNCTION__);
        return true;
    }
    catch (const cv::Exception& error)
    {
        qWarning() << "CameraStarDetector: CUDA star preprocessing failed; falling back to CPU:" << error.what();
    }
    PROFILER_STOP(__FUNCTION__);
    return false;
}
#endif

void CameraStarDetector::applyStarDetection(const cv::Mat& bgrMat, const cv::cuda::GpuMat* sourceBgrGpu, const cv::Rect& roi, QVector<CameraPipelineStarDetection>& starDetections, cv::Mat* debugMask) const
{
    PROFILER_START();

    cv::Mat gray;
    cv::Mat residual;
    cv::Mat thresholdMask;
    bool useCPUStarPreprocessing = true;

#ifdef CAMERA_OPENCV_CUDA_DETECTION
    if (canUseCudaDetection() && applyStarPreprocessingCuda(bgrMat, sourceBgrGpu, roi, gray, residual, thresholdMask, debugMask)) {
        useCPUStarPreprocessing = false;
    }
#endif
    if (useCPUStarPreprocessing) {
        applyStarPreprocessing(bgrMat, sourceBgrGpu, roi, gray, residual, thresholdMask, debugMask);
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresholdMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat finalMask;
    if (debugMask && (m_settings.m_starDebugView == CameraSettings::StarDebugViewFinal)) {
        finalMask = cv::Mat::zeros(thresholdMask.size(), CV_8UC1);
    }

    starDetections.clear();
    starDetections.reserve(static_cast<qsizetype>(contours.size()));

    for (const std::vector<cv::Point>& contour : contours)
    {
        const double area = cv::contourArea(contour);
        if ((area < m_settings.m_starMinArea) || (area > m_settings.m_starMaxArea)) {
            continue;
        }

        const cv::Rect box = cv::boundingRect(contour);
        const double width = std::max(1, box.width);
        const double height = std::max(1, box.height);
        const double aspectRatio = std::max(width / height, height / width);
        if (aspectRatio > m_settings.m_starMaxAspectRatio) {
            continue;
        }

        const double boundingArea = std::max(1.0, width * height);
        const double fillRatio = area / boundingArea;
        if (fillRatio < 0.2) {
            continue;
        }

        const double perimeter = std::max(1.0, cv::arcLength(contour, true));
        const double roundness = std::clamp((4.0 * CV_PI * area) / (perimeter * perimeter), 0.0, 1.0);
        if (roundness < 0.2) {
            continue;
        }

        cv::Mat contourMask = cv::Mat::zeros(box.height, box.width, CV_8UC1);
        std::vector<std::vector<cv::Point>> singleContour{contour};
        cv::drawContours(contourMask, singleContour, 0, cv::Scalar(255), cv::FILLED, cv::LINE_8, cv::noArray(), INT_MAX, -box.tl());

        const cv::Mat residualRoi = residual(box);
        const cv::Mat grayRoi = gray(box);
        double totalWeight = 0.0;
        double weightedX = 0.0;
        double weightedY = 0.0;
        double peakValue = 0.0;
        double grayPeak = 0.0;

        for (int row = 0; row < box.height; ++row)
        {
            const uchar* maskRow = contourMask.ptr<uchar>(row);
            const uchar* residualRow = residualRoi.ptr<uchar>(row);
            const uchar* grayRow = grayRoi.ptr<uchar>(row);

            for (int col = 0; col < box.width; ++col)
            {
                if (maskRow[col] == 0) {
                    continue;
                }

                const double weight = residualRow[col];
                if (weight > 0.0)
                {
                    totalWeight += weight;
                    weightedX += static_cast<double>(box.x + col) * weight;
                    weightedY += static_cast<double>(box.y + row) * weight;
                }

                if (weight > peakValue) {
                    peakValue = weight;
                }
                if (grayRow[col] > grayPeak) {
                    grayPeak = grayRow[col];
                }
            }
        }

        if (totalWeight <= 0.0) {
            continue;
        }

        const double centerX = weightedX / totalWeight;
        const double centerY = weightedY / totalWeight;
        const bool saturated = grayPeak >= 250.0;

        const double qualityScore = peakValue
            * std::max(0.25, roundness)
            * std::max(0.25, fillRatio)
            / std::max(1.0, aspectRatio)
            * (saturated ? 0.85 : 1.0);

        CameraPipelineStarDetection detection;
        detection.m_center = QPointF(centerX + roi.x, centerY + roi.y);
        detection.m_peakValue = static_cast<float>(peakValue);
        detection.m_radius = static_cast<float>(std::max(1.0, std::sqrt(area / CV_PI)));
        detection.m_qualityScore = static_cast<float>(qualityScore);
        detection.m_roundness = static_cast<float>(roundness);
        detection.m_fillRatio = static_cast<float>(fillRatio);
        detection.m_aspectRatio = static_cast<float>(aspectRatio);
        detection.m_saturated = saturated;
        starDetections.append(detection);

        if (!finalMask.empty()) {
            cv::rectangle(finalMask, box, cv::Scalar(255), 1);
        }
    }

    if (debugMask && (m_settings.m_starDebugView == CameraSettings::StarDebugViewFinal)) {
        *debugMask = finalMask;
    }

    PROFILER_STOP(__FUNCTION__);
}

