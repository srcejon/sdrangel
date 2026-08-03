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
#include <cstdint>
#include <vector>

#include <QDebug>
#include <opencv2/imgproc.hpp>
#ifdef CAMERA_OPENCV_CUDA_DETECTION
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include "util/profiler.h"
#include "cameraimageutils.h"
#include "cameraplatesolver.h"
#include "camerastardetector.h"

MESSAGE_CLASS_DEFINITION(CameraStarDetector::MsgReportPlateSolveStatus, Message)
MESSAGE_CLASS_DEFINITION(CameraStarDetector::MsgReportPointingError, Message)

namespace {

// Returns true and fills outGray with a CV_16UC1 view (with backing memory cloned) if the
// source QImage natively carries 16-bit dynamic range. Returns false otherwise — callers
// then fall through to the existing 8-bit pipeline. We preserve the original 16-bit
// precision because plate-solve centroid accuracy on faint stars depends on the per-pixel
// weight subtlety that a 16->8 truncation would destroy.
bool extractGrayMat16(const QImage& image, cv::Mat& outGray)
{
    if (image.format() == QImage::Format_Grayscale16)
    {
        cv::Mat grayView(image.height(), image.width(), CV_16UC1,
            const_cast<uchar*>(image.constBits()),
            static_cast<size_t>(image.bytesPerLine()));
        outGray = grayView.clone();
        return true;
    }

    if ((image.format() == QImage::Format_RGBA64) || (image.format() == QImage::Format_RGBX64))
    {
        // Convert 16-bit RGBA64 to 16-bit luminance with the standard BT.601 weights.
        // We keep the result in CV_16UC1 so subsequent residual/threshold/centroid work
        // sees the full dynamic range.
        outGray = cv::Mat(image.height(), image.width(), CV_16UC1);
        for (int y = 0; y < image.height(); ++y)
        {
            const QRgba64* in = reinterpret_cast<const QRgba64*>(image.constScanLine(y));
            uint16_t* out = outGray.ptr<uint16_t>(y);
            for (int x = 0; x < image.width(); ++x)
            {
                const uint32_t r = in[x].red();
                const uint32_t g = in[x].green();
                const uint32_t b = in[x].blue();
                // 0.299 R + 0.587 G + 0.114 B in fixed-point, with rounding.
                const uint32_t y16 = (19595u * r + 38470u * g + 7471u * b + 32768u) >> 16;
                out[x] = static_cast<uint16_t>(std::min<uint32_t>(65535u, y16));
            }
        }
        return true;
    }

    return false;
}

cv::Mat debugMaskTo8Bit(const cv::Mat& mask)
{
    if (mask.empty() || (mask.depth() == CV_8U)) {
        return mask.clone();
    }

    double minValue = 0.0;
    double maxValue = 0.0;
    cv::minMaxLoc(mask, &minValue, &maxValue);

    if (maxValue > minValue)
    {
        cv::Mat normalized;
        cv::normalize(mask, normalized, 0, 255, cv::NORM_MINMAX);
        normalized.convertTo(normalized, CV_8UC1);
        return normalized;
    }

    cv::Mat converted;
    mask.convertTo(converted, CV_8UC1);
    return converted;
}

double estimateResidualNoiseSigma(const cv::Mat& residual)
{
    if (residual.empty()) {
        return 1.0;
    }

    constexpr int kTargetSamples = 4096;
    const double totalPixels = static_cast<double>(residual.rows) * static_cast<double>(residual.cols);
    const int sampleStep = std::max(1, static_cast<int>(std::sqrt(std::max(1.0, totalPixels / kTargetSamples))));
    std::vector<double> samples;
    samples.reserve(static_cast<size_t>(std::min(static_cast<double>(kTargetSamples * 2), totalPixels)));

    for (int y = 0; y < residual.rows; y += sampleStep)
    {
        if (residual.depth() == CV_16U)
        {
            const uint16_t* row = residual.ptr<uint16_t>(y);
            for (int x = 0; x < residual.cols; x += sampleStep) {
                samples.push_back(static_cast<double>(row[x]));
            }
        }
        else
        {
            const uchar* row = residual.ptr<uchar>(y);
            for (int x = 0; x < residual.cols; x += sampleStep) {
                samples.push_back(static_cast<double>(row[x]));
            }
        }
    }

    if (samples.empty()) {
        return 1.0;
    }

    std::sort(samples.begin(), samples.end());
    auto percentile = [&samples](double fraction) {
        if (samples.size() == 1) {
            return samples.front();
        }

        const double position = std::clamp(fraction, 0.0, 1.0) * static_cast<double>(samples.size() - 1);
        const size_t lower = static_cast<size_t>(std::floor(position));
        const size_t upper = static_cast<size_t>(std::ceil(position));
        const double t = position - static_cast<double>(lower);
        return samples[lower] * (1.0 - t) + samples[upper] * t;
    };
    const double median = percentile(0.50);
    std::vector<double> deviations;
    deviations.reserve(samples.size());
    for (double sample : samples) {
        deviations.push_back(std::fabs(sample - median));
    }
    std::sort(deviations.begin(), deviations.end());
    auto deviationPercentile = [&deviations](double fraction) {
        if (deviations.size() == 1) {
            return deviations.front();
        }

        const double position = std::clamp(fraction, 0.0, 1.0) * static_cast<double>(deviations.size() - 1);
        const size_t lower = static_cast<size_t>(std::floor(position));
        const size_t upper = static_cast<size_t>(std::ceil(position));
        const double t = position - static_cast<double>(lower);
        return deviations[lower] * (1.0 - t) + deviations[upper] * t;
    };
    double sigma = 1.4826 * deviationPercentile(0.50);
    if (sigma < 1.0)
    {
        const double p90 = percentile(0.90);
        sigma = (p90 > median) ? (p90 - median) / 1.28155 : 1.0;
    }
    return std::max(1.0, sigma);
}

void thresholdResidualWithRobustTiles(const cv::Mat& residual,
                                      double userThreshold,
                                      double maxValue,
                                      cv::Mat& thresholdMask,
                                      double *meanNoiseSigma = nullptr)
{
    constexpr int kTileSize = 512;
    thresholdMask.create(residual.size(), residual.type());
    double weightedSigmaSum = 0.0;
    qint64 weightedPixelCount = 0;

    for (int y = 0; y < residual.rows; y += kTileSize)
    {
        const int height = std::min(kTileSize, residual.rows - y);
        for (int x = 0; x < residual.cols; x += kTileSize)
        {
            const int width = std::min(kTileSize, residual.cols - x);
            const cv::Rect tileRect(x, y, width, height);
            const cv::Mat residualTile = residual(tileRect);
            cv::Mat thresholdTile = thresholdMask(tileRect);
            const double tileNoiseSigma = estimateResidualNoiseSigma(residualTile);
            const double adaptiveThreshold = tileNoiseSigma * 4.0;
            cv::threshold(
                residualTile,
                thresholdTile,
                std::max(userThreshold, adaptiveThreshold),
                maxValue,
                cv::THRESH_BINARY);
            const qint64 tilePixels = static_cast<qint64>(width) * height;
            weightedSigmaSum += tileNoiseSigma * static_cast<double>(tilePixels);
            weightedPixelCount += tilePixels;
        }
    }

    if (meanNoiseSigma) {
        *meanNoiseSigma = (weightedPixelCount > 0)
            ? std::max(1.0, weightedSigmaSum / static_cast<double>(weightedPixelCount))
            : 1.0;
    }
}

cv::Mat detectSatelliteTrails(const cv::Mat& thresholdMask)
{
    PROFILER_START();

    if (thresholdMask.empty() || (thresholdMask.type() != CV_8UC1)) {
        PROFILER_STOP(__FUNCTION__);
        return {};
    }

    const int minDimension = std::min(thresholdMask.cols, thresholdMask.rows);
    if (minDimension < 128) {
        PROFILER_STOP(__FUNCTION__);
        return {};
    }

    const double minLineLength = std::max(160.0, minDimension * 0.20);
    const double maxLineGap = std::max(8.0, minDimension * 0.02);
    const int houghThreshold = std::max(30, static_cast<int>(std::round(minLineLength * 0.35)));

    cv::Mat lineMask;
    cv::dilate(
        thresholdMask,
        lineMask,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(lineMask, lines, 1.0, CV_PI / 180.0, houghThreshold, minLineLength, maxLineGap);
    if (lines.empty()) {
        PROFILER_STOP(__FUNCTION__);
        return {};
    }

    cv::Mat trailMask = cv::Mat::zeros(thresholdMask.size(), CV_8UC1);
    const int eraseThickness = std::clamp(static_cast<int>(std::round(minDimension * 0.008)), 5, 17);
    for (const cv::Vec4i& line : lines)
    {
        const cv::Point p1(line[0], line[1]);
        const cv::Point p2(line[2], line[3]);
        const double dx = static_cast<double>(p1.x - p2.x);
        const double dy = static_cast<double>(p1.y - p2.y);
        if (std::sqrt(dx * dx + dy * dy) < minLineLength) {
            continue;
        }

        cv::line(trailMask, p1, p2, cv::Scalar(255), eraseThickness, cv::LINE_8);
    }

    PROFILER_STOP(__FUNCTION__);
    return trailMask;
}

} // namespace
CameraStarDetector::CameraStarDetector()
#ifdef CAMERA_OPENCV_CUDA_DETECTION
    :
    m_plateSolver(this),
    m_msgQueueToGUI(nullptr),
    m_msgQueueToFeature(nullptr),
    m_cudaStarSmallBlurFilterType(-1),
    m_cudaStarBackgroundBlurFilterType(-1),
    m_cudaStarBackgroundBlurFilterSize(0)
#else
    :
    m_plateSolver(this),
    m_msgQueueToGUI(nullptr),
    m_msgQueueToFeature(nullptr)
#endif
{
}

CameraStarDetector::~CameraStarDetector() = default;

void CameraStarDetector::requestPlateSolveCancellation()
{
    m_plateSolver.requestCancellation();
}

void CameraStarDetector::reportPlateSolveStatus(bool solving) const
{
    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportPlateSolveStatus::create(solving));
    }
}

bool CameraStarDetector::plateSolveInputSettingsChanged(const QList<QString>& settingsKeys, bool applyDirectionChanges)
{
    return settingsKeys.contains("plateSolve")
        || settingsKeys.contains("plateSolveMaxMagnitude")
        || settingsKeys.contains("plateSolveMinMatches")
        || settingsKeys.contains("plateSolveMatchRadius")
        || settingsKeys.contains("plateSolveFinalMatchRadius")
        || settingsKeys.contains("plateSolveSearchRadius")
        || settingsKeys.contains("plateSolveStartMode")
        || settingsKeys.contains("plateSolveUseCaptureDateTime")
        || settingsKeys.contains("plateSolveDateTime")
        || settingsKeys.contains("plateSolveDateTimeUtc")
        || settingsKeys.contains("plateSolveUseDownloadedCatalog")
        || settingsKeys.contains("plateSolveCatalogSource")
        || settingsKeys.contains("latitude")
        || settingsKeys.contains("longitude")
        || settingsKeys.contains("altitude")
        || settingsKeys.contains("positionSync")
        || (applyDirectionChanges && settingsKeys.contains("azimuth"))
        || (applyDirectionChanges && settingsKeys.contains("elevation"))
        || (applyDirectionChanges && settingsKeys.contains("roll"))
        || settingsKeys.contains("directionApplyToCurrentImage")
        || settingsKeys.contains("rotator")
        || settingsKeys.contains("fov")
        || settingsKeys.contains("fovMode")
        || settingsKeys.contains("fovSensorWidthMm")
        || settingsKeys.contains("fovSensorHeightMm")
        || settingsKeys.contains("fovFocalLengthMm")
        || settingsKeys.contains("lensProjection")
        || settingsKeys.contains("lensCenterOffsetX")
        || settingsKeys.contains("lensCenterOffsetY")
        || settingsKeys.contains("lensDistortionK1")
        || settingsKeys.contains("lensMirror")
        || settingsKeys.contains("playbackProjectionEnabled")
        || settingsKeys.contains("playbackProjectionX")
        || settingsKeys.contains("playbackProjectionY")
        || settingsKeys.contains("playbackProjectionWidth")
        || settingsKeys.contains("playbackProjectionHeight");
}

bool CameraStarDetector::starDisplaySettingsChanged(const QList<QString>& settingsKeys, bool applyDirectionChanges)
{
    return settingsKeys.contains("starDetect")
        || settingsKeys.contains("starDebugView")
        || settingsKeys.contains("starThreshold")
        || settingsKeys.contains("starBackgroundBlur")
        || settingsKeys.contains("starMinArea")
        || settingsKeys.contains("starMaxArea")
        || settingsKeys.contains("starMaxAspectRatio")
        || settingsKeys.contains("starColor")
        || settingsKeys.contains("plateSolveLabelMode")
        || plateSolveInputSettingsChanged(settingsKeys, applyDirectionChanges);
}

bool CameraStarDetector::plateSolveInputSettingsChanged(
    const CameraSettings& previousSettings,
    const CameraSettings& newSettings,
    const QList<QString>& settingsKeys)
{
    // Anything that is not the pointing direction invalidates an in-flight solve outright.
    if (plateSolveInputSettingsChanged(settingsKeys, /*applyDirectionChanges=*/false)) {
        return true;
    }

    const bool directionChanged = settingsKeys.contains("azimuth")
        || settingsKeys.contains("elevation")
        || settingsKeys.contains("roll");
    if (!directionChanged) {
        return false;
    }

    // With "apply direction changes to current image" off, the solve is seeded from the
    // frame's own captured direction (see CameraImageUtils::projectionSettingsForFrame),
    // so the live direction is not a solve input at all and cannot invalidate anything.
    if (!newSettings.m_directionApplyToCurrentImage) {
        return false;
    }

    // Otherwise the solve does re-seed from the live direction, but only a move large
    // enough to be a different pointing is worth restarting for. A rotator tracking a
    // target re-broadcasts the direction about once a second while drifting ~0.01 deg/s -
    // a hundredth of a 1.27 deg field - and treating that as a re-point aborted every
    // solve before it could finish (measured in the field: 19 attempts, 19 cancelled, 0
    // completed, each killed ~1 s into a multi-second solve).
    constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
    const double thresholdDegrees = std::max(0.5 * static_cast<double>(newSettings.m_fov), 0.25);
    const double previousElevation = static_cast<double>(previousSettings.m_elevation) * kDegreesToRadians;
    const double newElevation = static_cast<double>(newSettings.m_elevation) * kDegreesToRadians;
    const double azimuthDelta =
        (static_cast<double>(newSettings.m_azimuth) - static_cast<double>(previousSettings.m_azimuth)) * kDegreesToRadians;
    const double cosSeparation = std::clamp(
        (std::sin(previousElevation) * std::sin(newElevation))
            + (std::cos(previousElevation) * std::cos(newElevation) * std::cos(azimuthDelta)),
        -1.0,
        1.0);
    const double separationDegrees = std::acos(cosSeparation) / kDegreesToRadians;

    double rollDelta = std::fmod(
        static_cast<double>(newSettings.m_roll) - static_cast<double>(previousSettings.m_roll) + 180.0, 360.0);
    if (rollDelta < 0.0) {
        rollDelta += 360.0;
    }
    rollDelta = std::fabs(rollDelta - 180.0);

    return (separationDegrees > thresholdDegrees) || (rollDelta > thresholdDegrees);
}

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

cv::Ptr<cv::cuda::Filter> CameraStarDetector::cudaStarSmallBlurFilter(int inputType)
{
    if (!m_cudaStarSmallBlurFilter || (m_cudaStarSmallBlurFilterType != inputType))
    {
        m_cudaStarSmallBlurFilter = cv::cuda::createGaussianFilter(
            inputType, inputType, cv::Size(3, 3), 0.0, 0.0);
        m_cudaStarSmallBlurFilterType = inputType;
    }

    return m_cudaStarSmallBlurFilter;
}

cv::Ptr<cv::cuda::Filter> CameraStarDetector::cudaStarBackgroundBlurFilter(int inputType, int kernelSize)
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

const cv::cuda::GpuMat& CameraStarDetector::cudaStarExclusionMask(const cv::Rect& roi, const cv::Size& workSize)
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


void CameraStarDetector::captureActiveChanged(bool active)
{
    if (!active) {
        requestPlateSolveCancellation();
    }
}

void CameraStarDetector::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    cameraLogSettingsChange("CameraStarDetector::applySettings:", settings, settingsKeys, force);
    if ((force && !settings.m_plateSolve)
        || (!force && settingsKeys.contains("plateSolve") && !settings.m_plateSolve)
        || (!force && settings.m_plateSolve && plateSolveInputSettingsChanged(m_settings, settings, settingsKeys)))
    {
        requestPlateSolveCancellation();
    }
    CameraDetectionStage::applySettings(settings, settingsKeys, force);

    if (!force && starDisplaySettingsChanged(settingsKeys, settings.m_directionApplyToCurrentImage) && m_lastInputFrame)
    {
        CameraPipelineFramePtr frame(new CameraPipelineFrame(*m_lastInputFrame));
        CameraImageUtils::applyPlaybackProjectionTransform(*frame, m_settings, true);
        frame->m_manualPreviewFrame = true;
        submitFrame(frame);
    }
}

void CameraStarDetector::processNewFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || !frame->hasImageData()) {
        return;
    }

    m_lastInputFrame.reset(new CameraPipelineFrame(*frame));

    frame->m_starDetections.clear();
    frame->m_plateSolve.m_solved = false;
    frame->m_plateSolve.m_matchedStars = 0;
    frame->m_plateSolve.m_detectedStarsConsidered = 0;
    frame->m_plateSolve.m_catalogStarsLoaded = 0;
    frame->m_plateSolve.m_catalogCandidateStars = 0;
    frame->m_plateSolve.m_outlierStars = 0;
    frame->m_plateSolve.m_rmsError = 0.0f;
    frame->m_plateSolve.m_maxError = 0.0f;
    frame->m_plateSolve.m_azimuth = 0.0f;
    frame->m_plateSolve.m_elevation = 0.0f;
    frame->m_plateSolve.m_roll = 0.0f;
    frame->m_plateSolve.m_fov = 0.0f;
    frame->m_plateSolve.m_centerOffsetX = 0.0f;
    frame->m_plateSolve.m_centerOffsetY = 0.0f;
    frame->m_plateSolve.m_distortionK1 = 0.0f;
    frame->m_plateSolve.m_catalogSource.clear();
    frame->m_plateSolve.m_failureReason.clear();
    frame->m_plateSolve.m_matchSummary.clear();
    frame->m_plateSolve.m_profileSummary.clear();
    frame->m_plateSolve.m_requiredMatches = 0;

    cv::Mat bgrMat;
    cv::Rect detectionRoi;
    cv::Mat highBitDepthGray;
#ifdef CAMERA_OPENCV_CUDA_DETECTION
    const cv::cuda::GpuMat* cachedBgrGpu = nullptr;
#endif

    // Declared mirrored camera (m_lensMirror): the image is horizontally mirrored relative to
    // the sky (up-looking all-sky camera, or a telescope star diagonal), which the plate
    // solver's orientation-preserving projector cannot fit. Fix: flip the DETECTION INPUT
    // IMAGE here so star detection and the solve both run on a sky-true view, then reflect
    // the detection coordinates back to the displayed (original) image at the end. Flipping
    // the image (rather than coordinate-flipping the original image's detections) is the
    // measured-correct form: the marginal all-sky solves are sensitive to the sub-pixel/order
    // differences of re-detected centroids (TREx recovered 10/20 with the image flip vs 4/20
    // with a coordinate flip of the same frames).
    const bool lensMirror = m_settings.m_lensMirror;

    auto materializeStarCpuInput = [&]() -> bool
    {
        if (!bgrMat.empty()) {
            return true;
        }

        if (!frame->ensureCpuImageFromCuda()) {
            return false;
        }

        // Capture a 16-bit luminance Mat *before* the RGB888 conversion truncates the
        // source. Plate-solve centroid accuracy on faint stars depends on full 16-bit
        // precision being preserved through residual computation and weighted centroid.
        extractGrayMat16(frame->m_image, highBitDepthGray);

        QImage convertedRgb;
        const QImage& rgb = ensureRgb888(frame->m_image, convertedRgb);
        cv::Mat mat = wrapRgb888Image(rgb);
        cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);
        if (lensMirror)
        {
            cv::flip(bgrMat, bgrMat, 1);
            if (!highBitDepthGray.empty()) {
                cv::flip(highBitDepthGray, highBitDepthGray, 1);
            }
        }
        detectionRoi = resolveDetectionRoi(bgrMat.size());
        if (lensMirror) {
            // The user's ROI is specified on the displayed image; mirror it into the flipped frame.
            detectionRoi.x = bgrMat.cols - detectionRoi.x - detectionRoi.width;
        }
#ifdef CAMERA_OPENCV_CUDA_DETECTION
        // The cached GPU image is the UNFLIPPED frame, so it must not feed detection when the
        // mirror flip is active — force the (flipped) CPU input instead.
        cachedBgrGpu = (frame->hasCudaBgrImage() && !lensMirror) ? &frame->m_cudaBgrImage : nullptr;
#endif
        return true;
    };

    if (m_settings.m_starDetect)
    {
        const QSize frameSize = frame->imageSize();
        if (frameSize.isEmpty()) {
            return;
        }
        detectionRoi = resolveDetectionRoi(cv::Size(frameSize.width(), frameSize.height()));

#ifdef CAMERA_OPENCV_CUDA_DETECTION
        const bool canUseGpuOnlyStarPreprocessing = frame->hasCudaBgrImage()
            && (frame->m_cudaBgrImage.depth() == CV_8U)
            && (m_settings.m_starDebugView == CameraSettings::StarDebugViewOff)
            && !lensMirror // GPU image is unflipped; the mirror flip needs the CPU input path
            && canUseCudaDetection();
        if (canUseGpuOnlyStarPreprocessing) {
            cachedBgrGpu = &frame->m_cudaBgrImage;
        } else
#endif
        if (!materializeStarCpuInput()) {
            return;
        }
    }

    if (m_settings.m_starDetect)
    {
        cv::Mat starDebugMask;
        bool detectedStars = applyStarDetection(
            bgrMat,
#ifdef CAMERA_OPENCV_CUDA_DETECTION
            cachedBgrGpu,
#endif
            detectionRoi,
            highBitDepthGray,
            frame->m_starDetections,
            (m_settings.m_starDebugView != CameraSettings::StarDebugViewOff) ? &starDebugMask : nullptr);
        if (!detectedStars)
        {
            if (!materializeStarCpuInput()) {
                return;
            }
            detectedStars = applyStarDetection(
                bgrMat,
#ifdef CAMERA_OPENCV_CUDA_DETECTION
                cachedBgrGpu,
#endif
                detectionRoi,
                highBitDepthGray,
                frame->m_starDetections,
                (m_settings.m_starDebugView != CameraSettings::StarDebugViewOff) ? &starDebugMask : nullptr);
            if (!detectedStars) {
                return;
            }
        }

        if (!starDebugMask.empty())
        {
            if (!materializeStarCpuInput()) {
                return;
            }

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
            if (lensMirror) {
                // The debug mask was built in the flipped detection frame; flip it back so the
                // debug view displays in the original image orientation.
                cv::flip(bgrMat, bgrMat, 1);
            }
            frame->m_image = convertBgrToRgbImage(bgrMat);
            frame->clearCudaCache();
        }
    }

    // Cloud structure can break up into star-like blobs; drop detections inside the cloud
    // mask before they reach the plate solver
    if (m_settings.m_cloudFilterStars && frame->m_cloud.m_valid && !frame->m_starDetections.isEmpty())
    {
        const CameraPipelineCloud& cloud = frame->m_cloud;
        // With the mirror flip active, detections are (still) in the flipped detection frame,
        // but the cloud mask was computed on the original image — reflect the test point.
        const double cloudMirrorMaxX = static_cast<double>(frame->imageSize().width() - 1);
        frame->m_starDetections.erase(
            std::remove_if(
                frame->m_starDetections.begin(),
                frame->m_starDetections.end(),
                [&cloud, lensMirror, cloudMirrorMaxX](const CameraPipelineStarDetection& detection) {
                    QPointF point = detection.m_center;
                    if (lensMirror) {
                        point.setX(cloudMirrorMaxX - point.x());
                    }
                    return cloud.isCloudAtImagePoint(point);
                }),
            frame->m_starDetections.end());
    }

    if (m_settings.m_plateSolve
        && frame->m_stack.m_projectionValid
        && !frame->m_starDetections.isEmpty())
    {
        reportPlateSolveStatus(true);
        // With the mirror flip active the detections are (still) in the flipped detection
        // frame — exactly what the solve should see. The solver receives a settings copy with
        // m_lensMirror cleared: the flip has already been applied to its input here, and the
        // solver itself intentionally has no mirror branch (see the note in
        // CameraPlateSolver::solve).
        QVector<CameraPipelineStarDetection> solveDetections = frame->m_starDetections;
        const bool solveInOpticalCoordinates = frame->m_imageTransform.isValid();
        const double mirrorImageMaxX = static_cast<double>(frame->imageSize().width() - 1);
        const double mirrorOpticalMaxX = static_cast<double>(frame->opticalImageSize().width() - 1);
        if (solveInOpticalCoordinates)
        {
            for (CameraPipelineStarDetection& detection : solveDetections)
            {
                QPointF point = detection.m_center;
                if (lensMirror) {
                    // mapImageToOptical expects original-image coordinates: unflip, map, then
                    // re-apply the mirror in optical space so the solve stays sky-true.
                    point.setX(mirrorImageMaxX - point.x());
                }
                point = frame->mapImageToOptical(point);
                if (lensMirror) {
                    point.setX(mirrorOpticalMaxX - point.x());
                }
                detection.m_center = point;
            }
        }

        CameraSettings solveSettings = CameraImageUtils::projectionSettingsForFrame(m_settings, *frame);
        solveSettings.m_lensMirror = false; // input already flipped above
        const QSize solveImageSize = frame->opticalImageSize();
        QElapsedTimer solveTimer;
        solveTimer.start();
        const CameraPlateSolveResult plateSolveResult = m_plateSolver.solve(
            solveSettings,
            solveImageSize,
            frame->m_captureDateTime,
            solveDetections);
        frame->m_plateSolve.m_solveTimeMs = static_cast<float>(solveTimer.elapsed());
        reportPlateSolveStatus(false);

        if (solveInOpticalCoordinates)
        {
            const int count = std::min(frame->m_starDetections.size(), solveDetections.size());
            for (int i = 0; i < count; ++i)
            {
                const QPointF imageCenter = frame->m_starDetections[i].m_center;
                frame->m_starDetections[i] = solveDetections[i];
                frame->m_starDetections[i].m_center = imageCenter;
                if (frame->m_starDetections[i].m_solved && !solveDetections[i].m_projectedCenter.isNull())
                {
                    QPointF projected = solveDetections[i].m_projectedCenter;
                    if (lensMirror) {
                        // Projected centres come back in the mirrored optical frame; unflip in
                        // optical space BEFORE the optical->image mapping (the mapping does not
                        // commute with the mirror), yielding original-image coordinates.
                        projected.setX(mirrorOpticalMaxX - projected.x());
                    }
                    frame->m_starDetections[i].m_projectedCenter = frame->mapOpticalToImage(projected);
                }
            }
        }
        else
        {
            frame->m_starDetections = solveDetections;
        }

        frame->m_plateSolve.m_solved = plateSolveResult.m_solved;
        // The pose of a mirrored camera's solve is expressed in the mirrored frame; overlay
        // projection onto the displayed image reflects pixel x (see CameraPostProcessor's
        // SkyProjector::mirrorX, driven by the same m_lensMirror setting).
        frame->m_plateSolve.m_mirrored = lensMirror || plateSolveResult.m_mirrored;
        frame->m_plateSolve.m_matchedStars = plateSolveResult.m_matchedStars;
        frame->m_plateSolve.m_detectedStarsConsidered = plateSolveResult.m_detectedStarsConsidered;
        frame->m_plateSolve.m_catalogStarsLoaded = plateSolveResult.m_catalogStarsLoaded;
        frame->m_plateSolve.m_catalogCandidateStars = plateSolveResult.m_catalogCandidateStars;
        frame->m_plateSolve.m_outlierStars = plateSolveResult.m_outlierStars;
        frame->m_plateSolve.m_rmsError = static_cast<float>(plateSolveResult.m_rmsErrorPixels);
        frame->m_plateSolve.m_maxError = static_cast<float>(plateSolveResult.m_maxErrorPixels);
        frame->m_plateSolve.m_azimuth = static_cast<float>(plateSolveResult.m_azimuthDegrees);
        frame->m_plateSolve.m_elevation = static_cast<float>(plateSolveResult.m_elevationDegrees);
        frame->m_plateSolve.m_roll = static_cast<float>(plateSolveResult.m_rollDegrees);
        frame->m_plateSolve.m_fov = static_cast<float>(plateSolveResult.m_fovDegrees);
        frame->m_plateSolve.m_centerOffsetX = static_cast<float>(plateSolveResult.m_centerOffsetXPixels);
        frame->m_plateSolve.m_centerOffsetY = static_cast<float>(plateSolveResult.m_centerOffsetYPixels);
        frame->m_plateSolve.m_distortionK1 = static_cast<float>(plateSolveResult.m_distortionK1);
        frame->m_plateSolve.m_catalogSource = plateSolveResult.m_catalogSource;
        frame->m_plateSolve.m_failureReason = plateSolveResult.m_failureReason;
        frame->m_plateSolve.m_matchSummary = plateSolveResult.m_matchSummary;
        frame->m_plateSolve.m_profileSummary = plateSolveResult.m_profileSummary;
        frame->m_plateSolve.m_requiredMatches = plateSolveResult.m_requiredMatches;

        // One line per solve, always on (uncategorised qInfo, so it survives the
        // camera.platesolver trace category being off). This is the operational summary:
        // it names the inputs the solve actually got - how many stars were detected, which
        // catalogue was used and how deep it turned out to be, how many of its stars fell in
        // the field - alongside the outcome and, on failure, the reason. Those catalogue
        // counts are the ones that matter in practice: an empty or shallow catalogue looks
        // exactly like a bad frame from the outside, and without them the only way to tell
        // was to turn the full trace on and reproduce.
        qInfo().noquote().nospace()
            << "CameraPlateSolve: solved=" << (plateSolveResult.m_solved ? 1 : 0)
            << " detections=" << frame->m_starDetections.size()
            << " considered=" << plateSolveResult.m_detectedStarsConsidered
            << " matched=" << plateSolveResult.m_matchedStars
            << " required=" << plateSolveResult.m_requiredMatches
            << " catalog=" << (plateSolveResult.m_catalogSource.isEmpty()
                    ? QStringLiteral("none") : plateSolveResult.m_catalogSource)
            << " catalogStars=" << plateSolveResult.m_catalogStarsLoaded
            << " inFieldCandidates=" << plateSolveResult.m_catalogCandidateStars
            << " maxMag=" << QString::number(solveSettings.m_plateSolveMaxMagnitude, 'f', 1)
            << " startMode=" << static_cast<int>(solveSettings.m_plateSolveStartMode)
            << " seedAz=" << QString::number(solveSettings.m_azimuth, 'f', 3)
            << " seedEl=" << QString::number(solveSettings.m_elevation, 'f', 3)
            << " seedFov=" << QString::number(solveSettings.m_fov, 'f', 3)
            << " solveMs=" << QString::number(frame->m_plateSolve.m_solveTimeMs, 'f', 0);
        // Bright-star agreement, on the accepted line as well as the rejected one: it is what
        // tells a true pose from a wrong-roll alias (a guiding session measured 5-9 of 12
        // bright projected stars matched for correct poses against 0-2 for wrong ones), and
        // without it an accepted pose could not be second-guessed from an operational log.
        const auto brightSupportSummary = [&plateSolveResult]() {
            return QStringLiteral(" brightDet=%1/%2 brightProj=%3/%4 magErr=%5 rankErr=%6")
                .arg(plateSolveResult.m_matchedBrightDetections)
                .arg(plateSolveResult.m_brightDetections)
                .arg(plateSolveResult.m_matchedBrightProjectedStars)
                .arg(plateSolveResult.m_brightProjectedStars)
                .arg(plateSolveResult.m_brightDetectionMagnitudeError, 0, 'f', 2)
                .arg(plateSolveResult.m_brightnessRankError, 0, 'f', 2);
        };
        if (plateSolveResult.m_solved)
        {
            qInfo().noquote().nospace()
                << "CameraPlateSolve: pose az=" << QString::number(plateSolveResult.m_azimuthDegrees, 'f', 4)
                << " el=" << QString::number(plateSolveResult.m_elevationDegrees, 'f', 4)
                << " roll=" << QString::number(plateSolveResult.m_rollDegrees, 'f', 3)
                << " fov=" << QString::number(plateSolveResult.m_fovDegrees, 'f', 4)
                << " rms=" << QString::number(plateSolveResult.m_rmsErrorPixels, 'f', 2)
                << " max=" << QString::number(plateSolveResult.m_maxErrorPixels, 'f', 2)
                << " outliers=" << plateSolveResult.m_outlierStars
                << brightSupportSummary();
        }
        else
        {
            // The rejected candidate's own numbers, so a rejection can be judged from the log
            // instead of having to reproduce the frame offline: whether the pose looked right
            // (compare its roll against neighbouring solves) and how tight the fit was.
            qInfo().noquote().nospace()
                << "CameraPlateSolve: rejected pose az=" << QString::number(plateSolveResult.m_azimuthDegrees, 'f', 4)
                << " el=" << QString::number(plateSolveResult.m_elevationDegrees, 'f', 4)
                << " roll=" << QString::number(plateSolveResult.m_rollDegrees, 'f', 3)
                << " fov=" << QString::number(plateSolveResult.m_fovDegrees, 'f', 4)
                << " rms=" << QString::number(plateSolveResult.m_rmsErrorPixels, 'f', 2)
                << " max=" << QString::number(plateSolveResult.m_maxErrorPixels, 'f', 2)
                << brightSupportSummary();
            qInfo().noquote().nospace()
                << "CameraPlateSolve: failed reason="
                << (plateSolveResult.m_failureReason.isEmpty()
                        ? QStringLiteral("(none given)") : plateSolveResult.m_failureReason);
        }

        // Autoguide Phase 0: measure (do not act on) the mount's pointing error. The solve was
        // seeded with the commanded direction in solveSettings, so commanded − solved is the
        // error a future guiding loop would trim out via the rotator controller's offsets.
        // Logged per solve so a night's run gives the drift rate and noise floor that pick the
        // loop's gain, deadband and cadence.
        if (plateSolveResult.m_solved
            && (static_cast<int>(solveSettings.m_plateSolveStartMode)
                >= static_cast<int>(CameraSettings::PlateSolveStartFovAzEl)))
        {
            const auto wrapDegrees = [](double degrees) {
                degrees = std::fmod(degrees + 180.0, 360.0);
                if (degrees < 0.0) {
                    degrees += 360.0;
                }
                return degrees - 180.0;
            };
            const double cosElevation = std::cos(plateSolveResult.m_elevationDegrees * (3.14159265358979323846 / 180.0));
            const double errorAzOnSky =
                wrapDegrees(static_cast<double>(solveSettings.m_azimuth) - plateSolveResult.m_azimuthDegrees) * cosElevation;
            const double errorEl = static_cast<double>(solveSettings.m_elevation) - plateSolveResult.m_elevationDegrees;
            const double errorRoll = wrapDegrees(static_cast<double>(solveSettings.m_roll) - plateSolveResult.m_rollDegrees);
            frame->m_plateSolve.m_pointingErrorValid = true;
            frame->m_plateSolve.m_pointingErrorAzDeg = static_cast<float>(errorAzOnSky);
            frame->m_plateSolve.m_pointingErrorElDeg = static_cast<float>(errorEl);
            frame->m_plateSolve.m_pointingErrorRollDeg = static_cast<float>(errorRoll);
            qInfo().noquote().nospace()
                << "CameraPointingError: t=" << frame->m_captureDateTime.toUTC().toString(Qt::ISODateWithMs)
                << " errAzOnSkyDeg=" << QString::number(errorAzOnSky, 'f', 5)
                << " errElDeg=" << QString::number(errorEl, 'f', 5)
                << " errRollDeg=" << QString::number(errorRoll, 'f', 3)
                << " commandedAz=" << QString::number(solveSettings.m_azimuth, 'f', 4)
                << " commandedEl=" << QString::number(solveSettings.m_elevation, 'f', 4)
                << " solvedAz=" << QString::number(plateSolveResult.m_azimuthDegrees, 'f', 4)
                << " solvedEl=" << QString::number(plateSolveResult.m_elevationDegrees, 'f', 4)
                << " matched=" << plateSolveResult.m_matchedStars
                << " rms=" << QString::number(plateSolveResult.m_rmsErrorPixels, 'f', 2)
                << " solveMs=" << QString::number(frame->m_plateSolve.m_solveTimeMs, 'f', 0);
            if (m_msgQueueToFeature)
            {
                // Autoguide Phase 1: hand the raw (not cos-elevation scaled) errors to the
                // Camera feature, which closes the loop on the rotator controller's offsets.
                const double errorAzRaw =
                    wrapDegrees(static_cast<double>(solveSettings.m_azimuth) - plateSolveResult.m_azimuthDegrees);
                m_msgQueueToFeature->push(MsgReportPointingError::create(
                    errorAzRaw,
                    errorEl,
                    plateSolveResult.m_elevationDegrees,
                    plateSolveResult.m_fovDegrees,
                    plateSolveResult.m_matchedStars,
                    static_cast<float>(plateSolveResult.m_rmsErrorPixels),
                    frame->m_plateSolve.m_solveTimeMs,
                    frame->m_captureDateTime,
                    plateSolveResult.m_brightProjectedStars,
                    plateSolveResult.m_matchedBrightProjectedStars,
                    plateSolveResult.m_brightDetectionMagnitudeError));
            }
        }
    }

    if (lensMirror && !frame->m_starDetections.isEmpty())
    {
        // Detection centroids were produced in the flipped detection frame; reflect them back
        // so every downstream consumer (display, harness validation, cloud overlays) sees
        // original-image coordinates. Projected centres are likewise reflected, EXCEPT when an
        // image transform is active — the optical merge above already restored those to
        // original-image coordinates via the unflip-then-map path.
        const double mirrorMaxX = static_cast<double>(frame->imageSize().width() - 1);
        const bool projectedAlreadyOriginal = frame->m_imageTransform.isValid();
        for (CameraPipelineStarDetection& detection : frame->m_starDetections)
        {
            detection.m_center.setX(mirrorMaxX - detection.m_center.x());
            if (!projectedAlreadyOriginal && !detection.m_projectedCenter.isNull()) {
                detection.m_projectedCenter.setX(mirrorMaxX - detection.m_projectedCenter.x());
            }
        }
    }

    forwardFrame(frame);
}

void CameraStarDetector::applyStarPreprocessing(const cv::Mat& bgrMat, const cv::Rect& roi, const cv::Mat& highBitDepthGray, cv::Mat& gray, cv::Mat& residual, cv::Mat& thresholdMask, double& residualNoiseSigma, cv::Mat* debugMask)
{
    PROFILER_START();

    // Use the 16-bit luminance Mat when supplied; otherwise fall back to the 8-bit
    // BGR->gray conversion. The threshold and the residual subtraction operate at the
    // native depth, so the centroid loop later sees full 16-bit weights for faint stars.
    const bool useHighBitDepth = !highBitDepthGray.empty()
        && (highBitDepthGray.depth() == CV_16U)
        && (highBitDepthGray.size() == bgrMat.size());
    if (useHighBitDepth) {
        gray = highBitDepthGray(roi).clone();
    } else {
        cv::cvtColor(bgrMat(roi), gray, cv::COLOR_BGR2GRAY);
    }

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
        *debugMask = debugMaskTo8Bit(background);
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
            *debugMask = debugMaskTo8Bit(residual);
        }
    }

    // Threshold scaled to the native bit depth. The user-facing m_starThreshold remains
    // expressed on the 0-255 scale; we shift left by 8 when running 16-bit so the same
    // slider value gives consistent visual behaviour. Use the residual noise as an
    // adaptive floor so moonlight, gradients or sensor noise do not flood the contour pass.
    const double userThresholdScaled = useHighBitDepth
        ? static_cast<double>(m_settings.m_starThreshold) * 256.0
        : static_cast<double>(m_settings.m_starThreshold);
    const double maxValueScaled = useHighBitDepth ? 65535.0 : 255.0;
    thresholdResidualWithRobustTiles(residual, userThresholdScaled, maxValueScaled, thresholdMask, &residualNoiseSigma);
    // findContours only accepts CV_8U; downscale the binary mask (it's still a binary
    // mask, no precision lost).
    if (thresholdMask.depth() != CV_8U) {
        cv::Mat thresholdMask8;
        thresholdMask.convertTo(thresholdMask8, CV_8U, 255.0 / 65535.0);
        thresholdMask = thresholdMask8;
    }
    cv::bitwise_and(thresholdMask, cachedExclusionMask(roi, thresholdMask.size()), thresholdMask);
    if (debugMask && (m_settings.m_starDebugView == CameraSettings::StarDebugViewThresholded)) {
        *debugMask = thresholdMask.clone();
    }
    PROFILER_STOP(__FUNCTION__);
}

#ifdef CAMERA_OPENCV_CUDA_DETECTION
bool CameraStarDetector::applyStarPreprocessingCuda(const cv::Mat& bgrMat, const cv::cuda::GpuMat* sourceBgrGpu, const cv::Rect& roi, cv::Mat& gray, cv::Mat& residual, cv::Mat& thresholdMask, double& residualNoiseSigma, cv::Mat* debugMask)
{
    PROFILER_START();

    try
    {
        cv::cuda::GpuMat bgrGpu;
        cv::cuda::GpuMat grayGpu;
        cv::cuda::GpuMat blurredGrayGpu;
        cv::cuda::GpuMat backgroundGpu;
        cv::cuda::GpuMat residualGpu;

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

        // grayGpu is intentionally not downloaded here — the full-frame transfer saves
        // nothing useful because gray is only needed for the per-blob saturation check,
        // which is instead approximated from the already-downloaded residual peak in
        // applyStarDetection (see hasGray / saturationThreshold logic there).
        m_cudaDetectionStream.waitForCompletion();

        thresholdResidualWithRobustTiles(residual, static_cast<double>(m_settings.m_starThreshold), 255.0, thresholdMask, &residualNoiseSigma);
        cv::bitwise_and(thresholdMask, cachedExclusionMask(roi, thresholdMask.size()), thresholdMask);

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
                *debugMask = debugMaskTo8Bit(residual);
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

bool CameraStarDetector::applyStarDetection(
    const cv::Mat& bgrMat,
#ifdef CAMERA_OPENCV_CUDA_DETECTION
    const cv::cuda::GpuMat* sourceBgrGpu,
#endif
    const cv::Rect& roi,
    const cv::Mat& highBitDepthGray,
    QVector<CameraPipelineStarDetection>& starDetections,
    cv::Mat* debugMask)
{
    PROFILER_START();

    cv::Mat gray;
    cv::Mat residual;
    cv::Mat thresholdMask;
    double residualNoiseSigma = 1.0;
    bool useCPUStarPreprocessing = true;

#ifdef CAMERA_OPENCV_CUDA_DETECTION
    // The CUDA preprocessing path operates only on 8-bit inputs today, so only consult it
    // when no 16-bit luminance was supplied. Otherwise fall through to the CPU path which
    // honours full 16-bit precision.
    if (highBitDepthGray.empty()
        && canUseCudaDetection()
        && applyStarPreprocessingCuda(bgrMat, sourceBgrGpu, roi, gray, residual, thresholdMask, residualNoiseSigma, debugMask))
    {
        useCPUStarPreprocessing = false;
    }
#endif
    if (useCPUStarPreprocessing)
    {
        if (bgrMat.empty())
        {
            qWarning() << "CameraStarDetector: CUDA star preprocessing failed and no CPU image is available";
            PROFILER_STOP(__FUNCTION__);
            return false;
        }
        applyStarPreprocessing(bgrMat, roi, highBitDepthGray, gray, residual, thresholdMask, residualNoiseSigma, debugMask);
    }

    cv::Mat satelliteTrailMask;
    if (m_settings.m_plateSolve
        && (m_settings.m_lensProjection == CameraSettings::LensProjectionRectilinear)
        && (m_settings.m_fov <= 30.0))
    {
        satelliteTrailMask = detectSatelliteTrails(thresholdMask);
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresholdMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat finalMask;
    if (debugMask && (m_settings.m_starDebugView == CameraSettings::StarDebugViewFinal)) {
        finalMask = cv::Mat::zeros(thresholdMask.size(), CV_8UC1);
    }

    starDetections.clear();
    starDetections.reserve(static_cast<qsizetype>(contours.size()));

    const bool hasGray = !gray.empty();
    const bool is16Bit = (residual.depth() == CV_16U);
    const double saturationThreshold = is16Bit ? 64000.0 : (hasGray ? 250.0 : 200.0);

    // Phase 3 (star-detector v2): DEFAULT ON. Applies the true pixel-count blob area in wide/fisheye
    // contexts (see v2Active below) so ~1px stars are recovered. Validated to keep REAL at 47 (after
    // the wide-9 mode-2 rot-vec fix) while improving synthetic fisheye (mode1 +3, mode4 +5) and
    // getting the real all-sky fisheye corpus solving. Kill-switch SDRANGEL_CAMERA_STAR_DETECTOR_DISABLE_V2
    // reverts to the legacy detector (byte-identical) for A/B. Read once.
    static const bool detectorV2 = !qEnvironmentVariableIsSet("SDRANGEL_CAMERA_STAR_DETECTOR_DISABLE_V2");
    // V2 quality gate (A2): the minimum SNR a star recovered ONLY by the true pixel-count area
    // (one the legacy polygon-area would have rejected at the minArea gate) must have to be admitted.
    // This keeps confident faint point sources -- which help wide/fisheye recall -- while blocking
    // the low-SNR noise / faint galaxy-structure blobs that regressed dense narrow REAL fields when
    // small-star recovery was unconditional. Env-tunable so the bar can be swept without rebuilding.
    static const double v2RecoveredMinSnr = []() {
        bool ok = false;
        const double v = qEnvironmentVariable("SDRANGEL_CAMERA_STAR_DETECTOR_V2_MINSNR").toDouble(&ok);
        return ok ? v : 0.0;   // 0 = off; context-gating (v2Active) protects narrow REAL now
    }();

    for (const std::vector<cv::Point>& contour : contours)
    {
        const cv::Rect box = cv::boundingRect(contour);
        const double polygonArea = cv::contourArea(contour);

        // V2 uses the true filled-pixel count as the blob area (cv::contourArea gives the
        // pixel-CENTRE polygon area -- ~1 for a 2x2 star, 0 for a 1-px-wide blob -- which rejects
        // real ~1px stars at the minArea gate and biases their fillRatio/SNR/hot-pixel). This is the
        // "more correct" area and it delivers the real wide/fisheye recall gain (validated on the
        // GMN/TREx real corpus), but the pixel count and the polygon count differ ~20% for EVERY
        // blob, and the dense narrow-field solver heuristics are co-tuned around the legacy polygon
        // area -- feeding them the corrected area regressed real narrow/galaxy fields. Since the
        // small-star problem is specifically a wide/fisheye phenomenon (tiny PSFs), apply V2 ONLY in
        // wide or fisheye contexts and leave narrow rectilinear fields on the legacy area untouched.
        const bool v2Active = detectorV2
            && ((m_settings.m_lensProjection != CameraSettings::LensProjectionRectilinear)
                || (m_settings.m_fov >= 30.0f));
        double area = polygonArea;
        cv::Mat contourMask;
        if (v2Active)
        {
            contourMask = cv::Mat::zeros(box.height, box.width, CV_8UC1);
            std::vector<std::vector<cv::Point>> pixelAreaContour{contour};
            cv::drawContours(contourMask, pixelAreaContour, 0, cv::Scalar(255), cv::FILLED, cv::LINE_8, cv::noArray(), INT_MAX, -box.tl());
            area = static_cast<double>(cv::countNonZero(contourMask));
        }
        if ((area < m_settings.m_starMinArea) || (area > m_settings.m_starMaxArea)) {
            continue;
        }

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
        const double roundness = std::clamp((4.0 * CV_PI * polygonArea) / (perimeter * perimeter), 0.0, 1.0);
        bool saturatedContourCandidate = false;
        if ((roundness < 0.2) && hasGray && (area >= 6.0))
        {
            double grayPeakInBox = 0.0;
            cv::minMaxLoc(gray(box), nullptr, &grayPeakInBox);
            saturatedContourCandidate = grayPeakInBox >= saturationThreshold;
        }
        if ((roundness < 0.2)
            && !(saturatedContourCandidate && (roundness >= 0.12)))
        {
            continue;
        }

        if (contourMask.empty())  // already built above under detectorV2
        {
            contourMask = cv::Mat::zeros(box.height, box.width, CV_8UC1);
            std::vector<std::vector<cv::Point>> singleContour{contour};
            cv::drawContours(contourMask, singleContour, 0, cv::Scalar(255), cv::FILLED, cv::LINE_8, cv::noArray(), INT_MAX, -box.tl());
        }

        const cv::Mat residualRoi = residual(box);
        // gray is empty when the CUDA preprocessing path skipped the download to save a
        // full-frame GPU→CPU transfer.  CPU paths always populate it.
        const cv::Mat grayRoi = hasGray ? gray(box) : cv::Mat();
        double totalWeight = 0.0;
        double weightedX = 0.0;
        double weightedY = 0.0;
        double weightedXX = 0.0;
        double weightedYY = 0.0;
        double peakValue = 0.0;
        double grayPeak = 0.0;

        // Branch on depth so 16-bit inputs preserve per-pixel precision in the weighted
        // centroid (faint stars benefit substantially from the extra bits).
        for (int row = 0; row < box.height; ++row)
        {
            const uchar* maskRow = contourMask.ptr<uchar>(row);
            const uchar* residualRow8 = is16Bit ? nullptr : residualRoi.ptr<uchar>(row);
            const uint16_t* residualRow16 = is16Bit ? residualRoi.ptr<uint16_t>(row) : nullptr;
            // Gray row pointers are null when gray was not downloaded (CUDA fast-path).
            const uchar* grayRow8 = (hasGray && !is16Bit) ? grayRoi.ptr<uchar>(row) : nullptr;
            const uint16_t* grayRow16 = (hasGray && is16Bit) ? grayRoi.ptr<uint16_t>(row) : nullptr;

            for (int col = 0; col < box.width; ++col)
            {
                if (maskRow[col] == 0) {
                    continue;
                }

                const double weight = is16Bit
                    ? static_cast<double>(residualRow16[col])
                    : static_cast<double>(residualRow8[col]);
                if (weight > 0.0)
                {
                    const double x = static_cast<double>(box.x + col);
                    const double y = static_cast<double>(box.y + row);
                    totalWeight += weight;
                    weightedX += x * weight;
                    weightedY += y * weight;
                    weightedXX += x * x * weight;
                    weightedYY += y * y * weight;
                }

                if (weight > peakValue) {
                    peakValue = weight;
                }
                if (grayRow8 || grayRow16) {
                    const double grayValue = grayRow16
                        ? static_cast<double>(grayRow16[col])
                        : static_cast<double>(grayRow8[col]);
                    if (grayValue > grayPeak) {
                        grayPeak = grayValue;
                    }
                }
            }
        }

        if (totalWeight <= 0.0) {
            continue;
        }

        const double centerX = weightedX / totalWeight;
        const double centerY = weightedY / totalWeight;
        const double varianceX = std::max(0.0, weightedXX / totalWeight - centerX * centerX);
        const double varianceY = std::max(0.0, weightedYY / totalWeight - centerY * centerY);
        const double rmsRadius = std::sqrt(0.5 * (varianceX + varianceY));
        const double fwhm = 2.354820045 * rmsRadius;
        const double snr = totalWeight / std::max(1.0, residualNoiseSigma * std::sqrt(std::max(1.0, area)));
        const double centroidUncertainty = (snr > 0.0)
            ? std::max(0.05, fwhm / std::max(2.354820045, 2.354820045 * snr))
            : 999.0;
        // Saturation threshold scales with the input depth — 250/255 on 8-bit translates
        // to ~64000/65535 on 16-bit. We keep the m_saturated flag depth-agnostic for the
        // plate solver's downstream quality scoring.
        //
        // When gray was not downloaded (CUDA fast-path), approximate grayPeak from the
        // residual peak instead: residual = blurredGray − background, so a saturated
        // 8-bit pixel (gray ≈ 255) typically yields residual ≈ 225–240 against dark
        // sky.  A proxy threshold of 200 gives comfortable headroom below that level.
        if (!hasGray) { grayPeak = peakValue; }
        const bool saturated = saturatedContourCandidate || (grayPeak >= saturationThreshold);

        const double qualityScore = peakValue
            * std::max(0.25, roundness)
            * std::max(0.25, fillRatio)
            / std::max(1.0, aspectRatio)
            * (saturated ? 0.85 : 1.0);
        const bool hotPixelSuspect = (area <= 2.0)
            || ((fwhm > 0.0) && (fwhm < 0.85) && (fillRatio > 0.75))
            || ((width <= 2.0) && (height <= 2.0) && (peakValue > residualNoiseSigma * 12.0));

        if (!satelliteTrailMask.empty()
            && !saturated
            && (area <= 30.0)
            && (fwhm <= 4.0))
        {
            const int trailX = std::clamp(static_cast<int>(std::round(centerX)), 0, satelliteTrailMask.cols - 1);
            const int trailY = std::clamp(static_cast<int>(std::round(centerY)), 0, satelliteTrailMask.rows - 1);
            if (satelliteTrailMask.at<uchar>(trailY, trailX) != 0) {
                continue;
            }
        }

        CameraPipelineStarDetection detection;
        detection.m_center = QPointF(centerX + roi.x, centerY + roi.y);
        detection.m_peakValue = static_cast<float>(peakValue);
        detection.m_radius = static_cast<float>(std::max(1.0, std::sqrt(area / CV_PI)));
        detection.m_flux = static_cast<float>(totalWeight);
        detection.m_snr = static_cast<float>(snr);
        detection.m_fwhm = static_cast<float>(fwhm);
        detection.m_centroidUncertainty = static_cast<float>(centroidUncertainty);
        detection.m_qualityScore = static_cast<float>(qualityScore);
        detection.m_roundness = static_cast<float>(roundness);
        detection.m_fillRatio = static_cast<float>(fillRatio);
        detection.m_aspectRatio = static_cast<float>(aspectRatio);
        detection.m_saturated = saturated;
        detection.m_hotPixelSuspect = hotPixelSuspect;

        // Optional V2 quality gate (A2, off by default): in a V2 (wide/fisheye) context, reject a
        // star recovered only by the pixel-count area (legacy polygon-area would have rejected it at
        // minArea) unless it clears an SNR bar. Off by default because context-gating already
        // protects narrow REAL; retained env-tunable in case wide/fisheye REAL needs noise filtering.
        // Saturated cores are exempt (unreliable SNR but real bright stars).
        if (v2Active
            && (v2RecoveredMinSnr > 0.0)
            && !saturated
            && (polygonArea < static_cast<double>(m_settings.m_starMinArea))
            && (snr < v2RecoveredMinSnr))
        {
            continue;
        }
        starDetections.append(detection);

        if (!finalMask.empty()) {
            cv::rectangle(finalMask, box, cv::Scalar(255), 1);
        }
    }

    // --- Supplementary saturated-core recovery --------------------------------
    // Very bright stars bloom into large, flat-topped saturated regions. The
    // background estimator reads an elevated local background under such a region,
    // so its residual ≈ 0 and it never reaches the threshold mask above — the star
    // is silently lost (e.g. Jabbah in stars-narrow-8). Recover these by
    // thresholding the *gray* image directly at the saturation level and emitting a
    // detection for each saturated core that no existing detection already covers.
    // (Skipped on the CUDA fast-path where gray was not downloaded.)
    if (hasGray)
    {
        const cv::Mat saturatedMask = gray >= saturationThreshold;   // CV_8U 0/255
        std::vector<std::vector<cv::Point>> saturatedContours;
        cv::findContours(saturatedMask, saturatedContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const std::vector<cv::Point>& contour : saturatedContours)
        {
            const double area = cv::contourArea(contour);
            // Skip single-pixel specks (saturated hot pixels) and pathologically
            // large regions (overexposed frames, not point sources).
            if ((area < std::max(6.0, static_cast<double>(m_settings.m_starMinArea)))
                || (area > static_cast<double>(CameraSettings::m_maxContourAreaBound))) {
                continue;
            }

            const cv::Moments mu = cv::moments(contour);
            if (mu.m00 <= 0.0) {
                continue;
            }
            const double centerX = mu.m10 / mu.m00;
            const double centerY = mu.m01 / mu.m00;
            const cv::Rect box = cv::boundingRect(contour);

            // Dedupe: skip only when an existing detection actually REPRESENTS this core.
            // Falling inside the core's radius is not sufficient — the erased rim of a big
            // bloomed star leaves tiny specks around its edge, and such a speck would
            // otherwise suppress the very star it is an artifact of (measured on Vega: a
            // 1.1px speck 26px from a 50px-wide core, inside the old 29px radius, hid the
            // star entirely). So a detection near the rim only counts when its own extent is
            // a meaningful fraction of the core; anything sitting essentially on the centroid
            // counts regardless of size, since that is a usable position for the star.
            const double coreRadius = std::sqrt(area / CV_PI);
            const double dedupeRadius = 0.5 * std::max(box.width, box.height) + 4.0;
            const double dedupeRadiusSq = dedupeRadius * dedupeRadius;
            const double centreRadius = std::max(3.0, 0.25 * coreRadius);
            const double centreRadiusSq = centreRadius * centreRadius;
            const double representativeRadius = 0.35 * coreRadius;
            const QPointF center(centerX + roi.x, centerY + roi.y);
            bool alreadyDetected = false;
            for (const CameraPipelineStarDetection& existing : starDetections) {
                const double dx = existing.m_center.x() - center.x();
                const double dy = existing.m_center.y() - center.y();
                const double distanceSq = dx * dx + dy * dy;
                if (distanceSq <= centreRadiusSq) {
                    alreadyDetected = true;
                    break;
                }
                if ((distanceSq <= dedupeRadiusSq)
                    && (static_cast<double>(existing.m_radius) >= representativeRadius)) {
                    alreadyDetected = true;
                    break;
                }
            }
            if (alreadyDetected) {
                continue;
            }

            const double width = std::max(1, box.width);
            const double height = std::max(1, box.height);
            const double aspectRatio = std::max(width / height, height / width);
            if (aspectRatio > m_settings.m_starMaxAspectRatio) {
                continue;
            }
            const double boundingArea = std::max(1.0, width * height);
            const double fillRatio = area / boundingArea;
            const double perimeter = std::max(1.0, cv::arcLength(contour, true));
            const double roundness = std::clamp((4.0 * CV_PI * area) / (perimeter * perimeter), 0.0, 1.0);

            // Spatial RMS radius from central moments → FWHM.
            const double varX = std::max(0.0, mu.mu20 / mu.m00);
            const double varY = std::max(0.0, mu.mu02 / mu.m00);
            const double rmsRadius = std::sqrt(0.5 * (varX + varY));
            const double fwhm = 2.354820045 * rmsRadius;

            double grayPeak = 0.0;
            cv::minMaxLoc(gray(box), nullptr, &grayPeak);
            const double flux = grayPeak * area;
            const double snr = flux / std::max(1.0, residualNoiseSigma * std::sqrt(area));

            CameraPipelineStarDetection detection;
            detection.m_center = center;
            detection.m_peakValue = static_cast<float>(grayPeak);
            detection.m_radius = static_cast<float>(std::max(1.0, std::sqrt(area / CV_PI)));
            detection.m_flux = static_cast<float>(flux);
            detection.m_snr = static_cast<float>(snr);
            detection.m_fwhm = static_cast<float>(fwhm);
            detection.m_centroidUncertainty = static_cast<float>(std::max(0.1, 0.5 * rmsRadius));
            detection.m_qualityScore = static_cast<float>(grayPeak
                * std::max(0.25, roundness)
                * std::max(0.25, fillRatio)
                / std::max(1.0, aspectRatio)
                * 0.85);
            detection.m_roundness = static_cast<float>(roundness);
            detection.m_fillRatio = static_cast<float>(fillRatio);
            detection.m_aspectRatio = static_cast<float>(aspectRatio);
            detection.m_saturated = true;
            detection.m_hotPixelSuspect = false;
            starDetections.append(detection);

            if (!finalMask.empty()) {
                cv::rectangle(finalMask, box, cv::Scalar(255), 1);
            }
        }
    }

    if (debugMask && (m_settings.m_starDebugView == CameraSettings::StarDebugViewFinal)) {
        *debugMask = finalMask;
    }

    PROFILER_STOP(__FUNCTION__);
    return true;
}
