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
#include "cameraplatesolver.h"
#include "camerastardetector.h"

MESSAGE_CLASS_DEFINITION(CameraStarDetector::MsgReportPlateSolveStatus, Message)

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
    samples.reserve(static_cast<size_t>(std::min(kTargetSamples * 2, residual.rows * residual.cols)));

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
    m_cudaStarSmallBlurFilterType(-1),
    m_cudaStarBackgroundBlurFilterType(-1),
    m_cudaStarBackgroundBlurFilterSize(0)
#else
    :
    m_plateSolver(this),
    m_msgQueueToGUI(nullptr)
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

bool CameraStarDetector::plateSolveInputSettingsChanged(const QList<QString>& settingsKeys)
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
        || settingsKeys.contains("azimuth")
        || settingsKeys.contains("elevation")
        || settingsKeys.contains("roll")
        || settingsKeys.contains("rotator")
        || settingsKeys.contains("fov")
        || settingsKeys.contains("fovMode")
        || settingsKeys.contains("fovSensorWidthMm")
        || settingsKeys.contains("fovSensorHeightMm")
        || settingsKeys.contains("fovFocalLengthMm")
        || settingsKeys.contains("lensProjection")
        || settingsKeys.contains("lensCenterOffsetX")
        || settingsKeys.contains("lensCenterOffsetY")
        || settingsKeys.contains("lensDistortionK1");
}

bool CameraStarDetector::starDisplaySettingsChanged(const QList<QString>& settingsKeys)
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
        || plateSolveInputSettingsChanged(settingsKeys);
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
    qDebug() << "CameraStarDetector::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;
    if ((force && !settings.m_plateSolve)
        || (!force && settingsKeys.contains("plateSolve") && !settings.m_plateSolve)
        || (!force && settings.m_plateSolve && plateSolveInputSettingsChanged(settingsKeys)))
    {
        requestPlateSolveCancellation();
    }
    CameraDetectionStage::applySettings(settings, settingsKeys, force);

    if (!force && starDisplaySettingsChanged(settingsKeys) && m_lastInputFrame)
    {
        CameraPipelineFramePtr frame(new CameraPipelineFrame(*m_lastInputFrame));
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
        detectionRoi = resolveDetectionRoi(bgrMat.size());
#ifdef CAMERA_OPENCV_CUDA_DETECTION
        cachedBgrGpu = frame->hasCudaBgrImage() ? &frame->m_cudaBgrImage : nullptr;
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
        applyStarDetection(
            bgrMat,
#ifdef CAMERA_OPENCV_CUDA_DETECTION
            cachedBgrGpu,
#endif
            detectionRoi,
            highBitDepthGray,
            frame->m_starDetections,
            (m_settings.m_starDebugView != CameraSettings::StarDebugViewOff) ? &starDebugMask : nullptr);

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
            frame->m_image = convertBgrToRgbImage(bgrMat);
            frame->clearCudaCache();
        }
    }

    if (m_settings.m_plateSolve && !frame->m_starDetections.isEmpty())
    {
        reportPlateSolveStatus(true);
        const QSize solveImageSize = frame->imageSize();
        const CameraPlateSolveResult plateSolveResult = m_plateSolver.solve(
            m_settings,
            solveImageSize,
            frame->m_captureDateTime,
            frame->m_starDetections);
        reportPlateSolveStatus(false);
        frame->m_plateSolve.m_solved = plateSolveResult.m_solved;
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

void CameraStarDetector::applyStarDetection(
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
            return;
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

        cv::Mat contourMask = cv::Mat::zeros(box.height, box.width, CV_8UC1);
        std::vector<std::vector<cv::Point>> singleContour{contour};
        cv::drawContours(contourMask, singleContour, 0, cv::Scalar(255), cv::FILLED, cv::LINE_8, cv::noArray(), INT_MAX, -box.tl());

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

            // Dedupe: if an existing detection already lies inside this saturated core
            // (its centroid within roughly the core radius), the star is already
            // represented by the normal pipeline — skip it.
            const double dedupeRadius = 0.5 * std::max(box.width, box.height) + 4.0;
            const double dedupeRadiusSq = dedupeRadius * dedupeRadius;
            const QPointF center(centerX + roi.x, centerY + roi.y);
            bool alreadyDetected = false;
            for (const CameraPipelineStarDetection& existing : starDetections) {
                const double dx = existing.m_center.x() - center.x();
                const double dy = existing.m_center.y() - center.y();
                if ((dx * dx + dy * dy) <= dedupeRadiusSq) {
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
}
