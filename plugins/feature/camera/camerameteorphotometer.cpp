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
#include <numeric>

#include <QDebug>

#include <opencv2/imgproc.hpp>

#include "cameraimageutils.h"
#include "camerameteorphotometer.h"
#include "util/profiler.h"

namespace {

constexpr double kSigmaFromMad = 1.4826;
constexpr double kMinMeteorThresholdSigma = 3.0;
constexpr int kMinReferenceStars = 2;

bool isMeteorDetection(const CameraPipelineDetection& detection)
{
    return detection.m_label.trimmed().compare(QStringLiteral("meteor"), Qt::CaseInsensitive) == 0;
}

double median(QVector<double> values)
{
    if (values.isEmpty()) {
        return 0.0;
    }

    const int middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    double result = values[middle];

    if ((values.size() % 2) == 0)
    {
        std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
        result = 0.5 * (result + values[middle - 1]);
    }

    return result;
}

double robustSigma(const QVector<double>& values, double valueMedian)
{
    if (values.size() < 2) {
        return 0.0;
    }

    QVector<double> deviations;
    deviations.reserve(values.size());
    for (double value : values) {
        deviations.append(std::fabs(value - valueMedian));
    }
    return kSigmaFromMad * median(deviations);
}

cv::Mat luminanceMat(const QImage& image, double& saturationValue)
{
    bool highBitDepth = false;
    cv::Mat working = CameraImageUtils::imageToWorkingMat(image, &highBitDepth);
    saturationValue = highBitDepth ? 65535.0 : 255.0;

    if (working.empty()) {
        return {};
    }

    cv::Mat gray;
    if (working.channels() == 1) {
        working.convertTo(gray, CV_64F);
    } else {
        cv::Mat rgb64;
        working.convertTo(rgb64, CV_64F);
        std::vector<cv::Mat> channels;
        cv::split(rgb64, channels);
        if (channels.size() >= 3) {
            gray = channels[0] * 0.2126 + channels[1] * 0.7152 + channels[2] * 0.0722;
        }
    }

    return gray;
}

QRect clippedRect(const QRect& rect, const QSize& imageSize)
{
    return rect.normalized().intersected(QRect(QPoint(0, 0), imageSize));
}

bool pointNearStar(const QPoint& point, const QVector<CameraPipelineStarDetection>& stars, double radiusScale)
{
    for (const CameraPipelineStarDetection& star : stars)
    {
        const double radius = std::max(3.0, radiusScale * std::max<double>(star.m_fwhm, star.m_radius));
        const double dx = point.x() + 0.5 - star.m_center.x();
        const double dy = point.y() + 0.5 - star.m_center.y();
        if ((dx * dx + dy * dy) <= (radius * radius)) {
            return true;
        }
    }
    return false;
}

bool pointInsideOtherObject(const QPoint& point, const QVector<CameraPipelineDetection>& detections, const QRect& currentBox)
{
    for (const CameraPipelineDetection& detection : detections)
    {
        if (detection.m_box == currentBox) {
            continue;
        }
        if (detection.m_box.adjusted(-2, -2, 2, 2).contains(point)) {
            return true;
        }
    }
    return false;
}

QVector<double> sampleBackground(const cv::Mat& gray, const QRect& signalBox, const QVector<CameraPipelineStarDetection>& stars, const QVector<CameraPipelineDetection>& detections)
{
    const QSize imageSize(gray.cols, gray.rows);
    const int pad = std::clamp(std::max(signalBox.width(), signalBox.height()) / 2, 12, 80);
    const QRect backgroundRect = clippedRect(signalBox.adjusted(-pad, -pad, pad, pad), imageSize);
    if (backgroundRect.isEmpty()) {
        return {};
    }

    const int area = backgroundRect.width() * backgroundRect.height();
    const int stride = std::max(1, static_cast<int>(std::sqrt(static_cast<double>(area) / 40000.0)));
    QVector<double> samples;
    samples.reserve(std::min(area, 40000));

    for (int y = backgroundRect.top(); y <= backgroundRect.bottom(); y += stride)
    {
        const double *line = gray.ptr<double>(y);
        for (int x = backgroundRect.left(); x <= backgroundRect.right(); x += stride)
        {
            const QPoint point(x, y);
            if (signalBox.contains(point)
                || pointNearStar(point, stars, 2.5)
                || pointInsideOtherObject(point, detections, signalBox))
            {
                continue;
            }
            samples.append(line[x]);
        }
    }

    return samples;
}

double apertureFlux(const cv::Mat& gray, const QPointF& center, double radius, double& background, double& backgroundSigma)
{
    const int apertureRadius = static_cast<int>(std::ceil(radius));
    const QRect apertureRect = QRect(
        static_cast<int>(std::floor(center.x())) - apertureRadius,
        static_cast<int>(std::floor(center.y())) - apertureRadius,
        apertureRadius * 2 + 1,
        apertureRadius * 2 + 1);
    const QSize imageSize(gray.cols, gray.rows);
    if (clippedRect(apertureRect, imageSize) != apertureRect) {
        return 0.0;
    }

    const double annulusInner = std::max(radius + 2.0, radius * 1.8);
    const double annulusOuter = std::max(annulusInner + 3.0, radius * 3.0);
    const int annulusRadius = static_cast<int>(std::ceil(annulusOuter));
    const QRect annulusRect = QRect(
        static_cast<int>(std::floor(center.x())) - annulusRadius,
        static_cast<int>(std::floor(center.y())) - annulusRadius,
        annulusRadius * 2 + 1,
        annulusRadius * 2 + 1);
    if (clippedRect(annulusRect, imageSize) != annulusRect) {
        return 0.0;
    }

    QVector<double> backgroundSamples;
    for (int y = annulusRect.top(); y <= annulusRect.bottom(); ++y)
    {
        const double *line = gray.ptr<double>(y);
        for (int x = annulusRect.left(); x <= annulusRect.right(); ++x)
        {
            const double dx = x + 0.5 - center.x();
            const double dy = y + 0.5 - center.y();
            const double distanceSquared = dx * dx + dy * dy;
            if ((distanceSquared >= annulusInner * annulusInner) && (distanceSquared <= annulusOuter * annulusOuter)) {
                backgroundSamples.append(line[x]);
            }
        }
    }

    if (backgroundSamples.size() < 8) {
        return 0.0;
    }

    background = median(backgroundSamples);
    backgroundSigma = robustSigma(backgroundSamples, background);

    double flux = 0.0;
    for (int y = apertureRect.top(); y <= apertureRect.bottom(); ++y)
    {
        const double *line = gray.ptr<double>(y);
        for (int x = apertureRect.left(); x <= apertureRect.right(); ++x)
        {
            const double dx = x + 0.5 - center.x();
            const double dy = y + 0.5 - center.y();
            if ((dx * dx + dy * dy) <= radius * radius) {
                flux += std::max(0.0, line[x] - background);
            }
        }
    }

    return flux;
}

struct ZeroPointCalibration
{
    double zeroPoint = 0.0;
    double rms = 0.0;
    int referenceStars = 0;
};

ZeroPointCalibration calibrateZeroPoint(const CameraPipelineFrame& frame, const cv::Mat& gray)
{
    QVector<double> zeroPoints;
    zeroPoints.reserve(frame.m_starDetections.size());
    const double exposureSeconds = frame.m_exposureTimeMs > 0.0 ? frame.m_exposureTimeMs / 1000.0 : 1.0;

    for (const CameraPipelineStarDetection& star : frame.m_starDetections)
    {
        if (!star.m_solved
            || star.m_saturated
            || star.m_hotPixelSuspect
            || !std::isfinite(star.m_catalogMagnitude)
            || (star.m_snr > 0.0f && star.m_snr < 5.0f))
        {
            continue;
        }

        bool insideMeteorBox = false;
        for (const CameraPipelineDetection& detection : frame.m_detections)
        {
            if (isMeteorDetection(detection) && detection.m_box.adjusted(-8, -8, 8, 8).contains(star.m_center.toPoint()))
            {
                insideMeteorBox = true;
                break;
            }
        }
        if (insideMeteorBox) {
            continue;
        }

        const double radius = std::clamp(
            std::max(2.5, std::max<double>(star.m_fwhm * 1.6, star.m_radius * 1.8)),
            2.5,
            12.0);
        double background = 0.0;
        double backgroundSigma = 0.0;
        const double flux = apertureFlux(gray, star.m_center, radius, background, backgroundSigma);
        if (flux <= 0.0 || !std::isfinite(flux)) {
            continue;
        }

        zeroPoints.append(static_cast<double>(star.m_catalogMagnitude) + 2.5 * std::log10(flux / exposureSeconds));
    }

    ZeroPointCalibration calibration;
    if (zeroPoints.size() < kMinReferenceStars) {
        calibration.referenceStars = zeroPoints.size();
        return calibration;
    }

    const double firstMedian = median(zeroPoints);
    const double firstSigma = std::max(0.05, robustSigma(zeroPoints, firstMedian));
    QVector<double> clipped;
    clipped.reserve(zeroPoints.size());
    for (double zp : zeroPoints)
    {
        if (std::fabs(zp - firstMedian) <= std::max(0.5, firstSigma * 2.5)) {
            clipped.append(zp);
        }
    }

    if (clipped.size() < kMinReferenceStars) {
        clipped = zeroPoints;
    }

    calibration.zeroPoint = median(clipped);
    double sumSquared = 0.0;
    for (double zp : clipped) {
        sumSquared += (zp - calibration.zeroPoint) * (zp - calibration.zeroPoint);
    }
    calibration.referenceStars = clipped.size();
    calibration.rms = clipped.size() > 1 ? std::sqrt(sumSquared / static_cast<double>(clipped.size() - 1)) : 0.0;
    return calibration;
}

CameraPipelineMeteorPhotometry measureMeteor(const CameraPipelineFrame& frame, const cv::Mat& gray, const CameraPipelineDetection& detection, const ZeroPointCalibration& calibration, double saturationValue)
{
    CameraPipelineMeteorPhotometry result;
    result.m_box = clippedRect(detection.m_box, QSize(gray.cols, gray.rows));
    result.m_center = result.m_box.center();

    if (result.m_box.isEmpty())
    {
        result.m_failureReason = QStringLiteral("empty meteor box");
        return result;
    }

    QVector<double> backgroundSamples = sampleBackground(gray, result.m_box, frame.m_starDetections, frame.m_detections);
    if (backgroundSamples.size() < 16)
    {
        result.m_failureReason = QStringLiteral("insufficient local background samples");
        return result;
    }

    result.m_background = median(backgroundSamples);
    result.m_backgroundSigma = robustSigma(backgroundSamples, result.m_background);
    const double threshold = result.m_background + std::max(1.0, kMinMeteorThresholdSigma * result.m_backgroundSigma);

    double flux = 0.0;
    int signalPixels = 0;
    bool saturated = false;

    for (int y = result.m_box.top(); y <= result.m_box.bottom(); ++y)
    {
        const double *line = gray.ptr<double>(y);
        for (int x = result.m_box.left(); x <= result.m_box.right(); ++x)
        {
            const QPoint point(x, y);
            if (pointNearStar(point, frame.m_starDetections, 2.0)) {
                continue;
            }

            const double value = line[x];
            saturated = saturated || (value >= saturationValue - 0.5);
            if (value >= threshold)
            {
                flux += std::max(0.0, value - result.m_background);
                ++signalPixels;
            }
        }
    }

    if ((signalPixels == 0) || (flux <= 0.0))
    {
        for (int y = result.m_box.top(); y <= result.m_box.bottom(); ++y)
        {
            const double *line = gray.ptr<double>(y);
            for (int x = result.m_box.left(); x <= result.m_box.right(); ++x)
            {
                const QPoint point(x, y);
                if (pointNearStar(point, frame.m_starDetections, 2.0)) {
                    continue;
                }
                const double positive = std::max(0.0, line[x] - result.m_background);
                if (positive > 0.0)
                {
                    flux += positive;
                    ++signalPixels;
                }
            }
        }
    }

    result.m_flux = flux;
    result.m_signalPixels = signalPixels;
    result.m_saturated = saturated;

    if (flux <= 0.0)
    {
        result.m_failureReason = QStringLiteral("no positive meteor flux");
        return result;
    }

    if (!frame.m_plateSolve.m_solved)
    {
        result.m_failureReason = QStringLiteral("plate solve unavailable");
        return result;
    }

    if (calibration.referenceStars < kMinReferenceStars)
    {
        result.m_referenceStars = calibration.referenceStars;
        result.m_failureReason = QStringLiteral("insufficient reference stars");
        return result;
    }

    const double exposureSeconds = frame.m_exposureTimeMs > 0.0 ? frame.m_exposureTimeMs / 1000.0 : 1.0;
    result.m_zeroPoint = calibration.zeroPoint;
    result.m_zeroPointRms = calibration.rms;
    result.m_referenceStars = calibration.referenceStars;
    result.m_magnitude = -2.5 * std::log10(flux / exposureSeconds) + calibration.zeroPoint;
    const double fluxErrorMagnitude = result.m_backgroundSigma > 0.0
        ? 1.085736205 * (std::sqrt(static_cast<double>(std::max(1, signalPixels))) * result.m_backgroundSigma) / flux
        : 0.0;
    result.m_magnitudeError = std::sqrt(calibration.rms * calibration.rms + fluxErrorMagnitude * fluxErrorMagnitude);
    result.m_validMagnitude = std::isfinite(result.m_magnitude) && std::isfinite(result.m_magnitudeError);
    if (result.m_saturated) {
        result.m_failureReason = QStringLiteral("meteor saturated; magnitude is a lower bound");
    }
    return result;
}

}

void CameraMeteorPhotometer::processFrame(CameraPipelineFrame& frame) const
{
    PROFILER_START();

    frame.m_meteorPhotometry.clear();

    bool hasMeteor = false;
    for (const CameraPipelineDetection& detection : frame.m_detections)
    {
        if (isMeteorDetection(detection))
        {
            hasMeteor = true;
            break;
        }
    }
    const QImage& photometryImage = !frame.m_unprocessedImage.isNull() ? frame.m_unprocessedImage : frame.m_image;
    if (!hasMeteor || photometryImage.isNull()) {
        return;
    }

    double saturationValue = 255.0;
    const cv::Mat gray = luminanceMat(photometryImage, saturationValue);
    if (gray.empty()) {
        return;
    }

    const ZeroPointCalibration calibration = frame.m_plateSolve.m_solved
        ? calibrateZeroPoint(frame, gray)
        : ZeroPointCalibration();

    for (const CameraPipelineDetection& detection : frame.m_detections)
    {
        if (!isMeteorDetection(detection)) {
            continue;
        }

        CameraPipelineMeteorPhotometry result = measureMeteor(frame, gray, detection, calibration, saturationValue);
        frame.m_meteorPhotometry.append(result);
        qDebug() << "CameraMeteorPhotometer: meteor"
                 << "box" << result.m_box
                 << "flux" << result.m_flux
                 << "magValid" << result.m_validMagnitude
                 << "mag" << result.m_magnitude
                 << "magErr" << result.m_magnitudeError
                 << "refs" << result.m_referenceStars
                 << "zp" << result.m_zeroPoint
                 << "zpRms" << result.m_zeroPointRms
                 << "background" << result.m_background
                 << "sigma" << result.m_backgroundSigma
                 << "signalPixels" << result.m_signalPixels
                 << "saturated" << result.m_saturated
                 << "reason" << result.m_failureReason;
    }

    PROFILER_STOP(__FUNCTION__);
}
