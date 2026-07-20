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
#include <limits>

#include <QtGlobal>
#include <QRect>
#include <QRgba64>
#include <QStringList>

#include "cameraopticalspectrum.h"

namespace {

// Maximum background (sky) rows sampled on each side of the aperture for the
// per-column median, bounding memory for large RoIs while staying robust to
// stars and hot pixels in the margin.
constexpr int kMaxBackgroundRowsPerSide = 16;

// A pixel at or above this level (on the 0..255 working scale) has clipped, so it
// carries no colour information.
constexpr float kSaturationLevel = 254.0f;

// Background rows are only sky if they are meaningfully darker than the trace. When a
// source fills the RoI -- a discharge tube whose emission lines span the whole frame,
// say -- the "background" rows contain the signal, and subtracting them would cancel
// the spectrum out. If the background reaches this fraction of the aperture level for
// most of the axis, treat it as having no usable background rather than destroy the data.
constexpr double kBackgroundSignalRatio = 0.9;
constexpr double kBackgroundSignalColumnFraction = 0.5;

struct RgbF
{
    float m_r;
    float m_g;
    float m_b;
};

inline float luminanceOf(const RgbF& rgb)
{
    return 0.299f * rgb.m_r + 0.587f * rgb.m_g + 0.114f * rgb.m_b;
}

// Pixel layouts sampled directly from the source image. Anything else falls back to a
// converted copy of the RoI; the direct paths cover the pipeline's usual formats so the
// per-frame full-RoI copy + format conversion (the dominant cost for large RoIs) is
// avoided.
enum class PixelLayout
{
    Argb32,   // ARGB32 / RGB32 / ARGB32_Premultiplied
    Rgb888,
    Rgba64,   // RGBX64 / RGBA64 / RGBA64_Premultiplied
    Grey8,
    Grey16
};

bool layoutForFormat(QImage::Format format, PixelLayout& layout)
{
    switch (format)
    {
    case QImage::Format_ARGB32:
    case QImage::Format_RGB32:
    case QImage::Format_ARGB32_Premultiplied:
        layout = PixelLayout::Argb32;
        return true;
    case QImage::Format_RGB888:
        layout = PixelLayout::Rgb888;
        return true;
    case QImage::Format_RGBX64:
    case QImage::Format_RGBA64:
    case QImage::Format_RGBA64_Premultiplied:
        layout = PixelLayout::Rgba64;
        return true;
    case QImage::Format_Grayscale8:
        layout = PixelLayout::Grey8;
        return true;
    case QImage::Format_Grayscale16:
        layout = PixelLayout::Grey16;
        return true;
    default:
        return false;
    }
}

// Uniform pixel access over the source image, offset by the RoI, with the dispersion
// axis mapped to the first coordinate. 16-bit values are scaled to the 8-bit range so
// profile magnitudes are comparable between sources.
class RoiSampler
{
public:
    RoiSampler(const QImage& image, PixelLayout layout, int roiX, int roiY, int roiWidth, int roiHeight, bool vertical) :
        m_image(image),
        m_layout(layout),
        m_roiX(roiX),
        m_roiY(roiY),
        m_vertical(vertical),
        m_along(vertical ? roiHeight : roiWidth),
        m_across(vertical ? roiWidth : roiHeight)
    {
    }

    int alongSize() const { return m_along; }
    int acrossSize() const { return m_across; }

    RgbF sample(int along, int across) const
    {
        const int x = m_roiX + (m_vertical ? across : along);
        const int y = m_roiY + (m_vertical ? along : across);
        const uchar* line = m_image.constScanLine(y);
        switch (m_layout)
        {
        case PixelLayout::Argb32:
        {
            const QRgb pixel = reinterpret_cast<const QRgb*>(line)[x];
            return { static_cast<float>(qRed(pixel)), static_cast<float>(qGreen(pixel)), static_cast<float>(qBlue(pixel)) };
        }
        case PixelLayout::Rgb888:
        {
            const uchar* pixel = line + 3 * x;
            return { static_cast<float>(pixel[0]), static_cast<float>(pixel[1]), static_cast<float>(pixel[2]) };
        }
        case PixelLayout::Rgba64:
        {
            const QRgba64 pixel = reinterpret_cast<const QRgba64*>(line)[x];
            return { pixel.red() / 257.0f, pixel.green() / 257.0f, pixel.blue() / 257.0f };
        }
        case PixelLayout::Grey8:
        {
            const float value = line[x];
            return { value, value, value };
        }
        case PixelLayout::Grey16:
        {
            const float value = reinterpret_cast<const quint16*>(line)[x] / 257.0f;
            return { value, value, value };
        }
        }
        return { 0.0f, 0.0f, 0.0f };
    }

private:
    const QImage& m_image;
    PixelLayout m_layout;
    int m_roiX;
    int m_roiY;
    bool m_vertical;
    int m_along;
    int m_across;
};

float medianInPlace(QVector<float>& values)
{
    if (values.isEmpty()) {
        return 0.0f;
    }
    const int mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    return values[mid];
}

float percentileInPlace(QVector<float>& values, double fraction)
{
    if (values.isEmpty()) {
        return 0.0f;
    }
    const int index = qBound(0, static_cast<int>(fraction * (values.size() - 1)), values.size() - 1);
    std::nth_element(values.begin(), values.begin() + index, values.end());
    return values[index];
}

// Sky estimator for the background rows. In a dense field of (defocused) stars a large
// fraction of the sampled rows are star-covered at any column, which pulls a median up
// everywhere - not as local spikes the along-axis smoothing could reject, but as a
// uniform overestimate that subtracts real signal. A low percentile tracks the dark sky
// BETWEEN the stars instead, at the cost of a small noise bias (a fraction of a count),
// and stays valid until stars cover most of the sampled rows.
constexpr double kSkyPercentile = 0.3;

// Moving median along a curve, window shrinking symmetrically at the edges. Used to
// smooth the per-column background estimates along the dispersion axis: the sky varies
// slowly along the axis, so smoothing rejects both the per-column estimation noise and
// localised contamination (a defocused star sitting in the background rows), either of
// which would otherwise be subtracted straight into the spectrum.
QVector<double> movingMedian(const QVector<double>& values, int halfWindow)
{
    const int n = values.size();
    QVector<double> smoothed(n);
    QVector<float> scratch;
    for (int i = 0; i < n; i++)
    {
        const int sideHalf = qMin(halfWindow, qMin(i, n - 1 - i));
        scratch.resize(2 * sideHalf + 1);
        for (int j = -sideHalf; j <= sideHalf; j++) {
            scratch[j + sideHalf] = static_cast<float>(values[i + j]);
        }
        smoothed[i] = medianInPlace(scratch);
    }
    return smoothed;
}

// Intensity-weighted centroid of the profile values above zero around the peak.
float refinePeakCentroid(const QVector<float>& profile, int peakIndex)
{
    constexpr int kWindow = 3;
    const int lo = qMax(0, peakIndex - kWindow);
    const int hi = qMin(profile.size() - 1, peakIndex + kWindow);
    // Weight relative to the window edge level so the broad first-order spectrum
    // under the peak does not drag the centroid.
    const float base = qMin(profile[lo], profile[hi]);
    double weightSum = 0.0;
    double positionSum = 0.0;
    for (int i = lo; i <= hi; i++)
    {
        const double w = qMax(0.0f, profile[i] - base);
        weightSum += w;
        positionSum += w * i;
    }
    return (weightSum > 0.0) ? static_cast<float>(positionSum / weightSum) : static_cast<float>(peakIndex);
}

float profileCentroid(const QVector<float>& profile)
{
    double weightSum = 0.0;
    double positionSum = 0.0;
    for (int i = 0; i < profile.size(); i++)
    {
        const double w = qMax(0.0f, profile[i]);
        weightSum += w;
        positionSum += w * i;
    }
    return (weightSum > 0.0) ? static_cast<float>(positionSum / weightSum) : -1.0f;
}

} // namespace

CameraOpticalSpectrumData CameraOpticalSpectrumExtractor::extract(
    const QImage& image,
    int roiX,
    int roiY,
    int roiWidth,
    int roiHeight,
    int apertureRows,
    bool backgroundSubtract)
{
    CameraOpticalSpectrumData data;

    if (image.isNull()) {
        return data;
    }

    // Same clamping semantics as CameraDetectionStage::resolveDetectionRoi:
    // width/height <= 0 selects the full frame.
    const int frameWidth = image.width();
    const int frameHeight = image.height();
    const int x = qBound(0, roiX, frameWidth - 1);
    const int y = qBound(0, roiY, frameHeight - 1);
    const int width = (roiWidth <= 0) ? (frameWidth - x) : qBound(1, roiWidth, frameWidth - x);
    const int height = (roiHeight <= 0) ? (frameHeight - y) : qBound(1, roiHeight, frameHeight - y);

    // Sample supported formats directly from the source image (a QImage read never
    // detaches); only unsupported formats pay for a converted copy of the RoI.
    PixelLayout layout = PixelLayout::Argb32;
    QImage converted;
    int samplerX = x;
    int samplerY = y;
    const QImage* sourceImage = &image;
    if (!layoutForFormat(image.format(), layout))
    {
        // Formats deeper than 8 bits per channel keep their depth through RGBA64
        const QImage::Format workFormat = (image.depth() > 32)
            ? QImage::Format_RGBA64
            : QImage::Format_ARGB32;
        converted = image.copy(QRect(x, y, width, height)).convertToFormat(workFormat);
        if (converted.isNull() || !layoutForFormat(converted.format(), layout)) {
            return data;
        }
        sourceImage = &converted;
        samplerX = 0;
        samplerY = 0;
    }

    const bool vertical = height > width;
    const RoiSampler sampler(*sourceImage, layout, samplerX, samplerY, width, height, vertical);
    const int along = sampler.alongSize();
    const int across = sampler.acrossSize();

    // Pass 1: per-across-row luminance totals to locate the trace centre. The centre
    // does not need every column, so wide profiles are subsampled to bound the cost of
    // this full-RoI scan.
    const int alongStep = qMax(1, along / 512);
    QVector<double> rowLuminance(across, 0.0);
    for (int b = 0; b < across; b++)
    {
        double sum = 0.0;
        for (int a = 0; a < along; a += alongStep) {
            sum += luminanceOf(sampler.sample(a, b));
        }
        rowLuminance[b] = sum;
    }

    // Weight rows against the faintest row so a uniform sky level does not bias
    // the trace centre toward the geometric middle.
    const double minRowLuminance = *std::min_element(rowLuminance.cbegin(), rowLuminance.cend());
    double traceWeightSum = 0.0;
    double tracePositionSum = 0.0;
    for (int b = 0; b < across; b++)
    {
        const double w = rowLuminance[b] - minRowLuminance;
        traceWeightSum += w;
        tracePositionSum += w * b;
    }
    const double traceCentre = (traceWeightSum > 0.0) ? (tracePositionSum / traceWeightSum) : (across - 1) / 2.0;

    // Aperture across the trace; <= 0 or wider than the RoI selects all rows.
    int aperture = (apertureRows <= 0) ? across : qMin(apertureRows, across);
    int apertureStart = qBound(0, static_cast<int>(std::lround(traceCentre - aperture / 2.0)), across - aperture);
    const int apertureEnd = apertureStart + aperture;

    // Background rows: margin rows on each side of the trace, capped. Two guards keep
    // the trace's own light out of the estimate: a gap beyond the aperture (standard
    // aperture-photometry practice), and the MEASURED extent of the trace's glow from
    // the row-luminance profile - a defocused end of the spectrum (chromatic focus)
    // spreads well beyond the aperture, and sampling that halo as "sky" subtracts real
    // signal, driving the continuum negative. If too few rows remain, skip subtraction
    // and say so.
    const int backgroundGap = qMax(2, aperture / 4);
    QVector<int> backgroundRows;
    bool insufficientBackgroundRows = false;
    if (backgroundSubtract && (aperture < across))
    {
        // Trace extent: rows whose (sky-relative) luminance exceeds 2% of the peak row
        const double maxRowLuminance = *std::max_element(rowLuminance.cbegin(), rowLuminance.cend());
        const double extentThreshold = minRowLuminance + 0.02 * (maxRowLuminance - minRowLuminance);
        const int centreRow = qBound(0, static_cast<int>(std::lround(traceCentre)), across - 1);
        int traceLow = centreRow;
        while ((traceLow - 1 >= 0) && (rowLuminance[traceLow - 1] > extentThreshold)) {
            traceLow--;
        }
        int traceHigh = centreRow;
        while ((traceHigh + 1 < across) && (rowLuminance[traceHigh + 1] > extentThreshold)) {
            traceHigh++;
        }

        const int belowStart = qMin(apertureStart - 1 - backgroundGap, traceLow - 1 - backgroundGap);
        for (int b = belowStart; (b >= 0) && (belowStart - b < kMaxBackgroundRowsPerSide); b--) {
            backgroundRows.append(b);
        }
        const int aboveStart = qMax(apertureEnd + backgroundGap, traceHigh + 1 + backgroundGap);
        for (int b = aboveStart; (b < across) && (b - aboveStart < kMaxBackgroundRowsPerSide); b++) {
            backgroundRows.append(b);
        }
        constexpr int kMinBackgroundRows = 6;
        if (backgroundRows.size() < kMinBackgroundRows)
        {
            insufficientBackgroundRows = true;
            backgroundRows.clear();
        }
    }
    data.m_backgroundInsufficientRows = insufficientBackgroundRows;

    data.m_luminance.resize(along);
    data.m_red.resize(along);
    data.m_green.resize(along);
    data.m_blue.resize(along);

    // Pass 2: raw aperture sums, per-column background medians and saturation count.
    QVector<double> rawR(along);
    QVector<double> rawG(along);
    QVector<double> rawB(along);
    QVector<double> backgroundR(along, 0.0);
    QVector<double> backgroundG(along, 0.0);
    QVector<double> backgroundB(along, 0.0);
    QVector<float> bgR(backgroundRows.size());
    QVector<float> bgG(backgroundRows.size());
    QVector<float> bgB(backgroundRows.size());
    qint64 clippedPixels = 0;
    for (int a = 0; a < along; a++)
    {
        double sumR = 0.0;
        double sumG = 0.0;
        double sumB = 0.0;
        for (int b = apertureStart; b < apertureEnd; b++)
        {
            const RgbF rgb = sampler.sample(a, b);
            sumR += rgb.m_r;
            sumG += rgb.m_g;
            sumB += rgb.m_b;
            if ((rgb.m_r >= kSaturationLevel) || (rgb.m_g >= kSaturationLevel) || (rgb.m_b >= kSaturationLevel)) {
                clippedPixels++;
            }
        }
        rawR[a] = sumR;
        rawG[a] = sumG;
        rawB[a] = sumB;

        if (!backgroundRows.isEmpty())
        {
            for (int i = 0; i < backgroundRows.size(); i++)
            {
                const RgbF rgb = sampler.sample(a, backgroundRows[i]);
                bgR[i] = rgb.m_r;
                bgG[i] = rgb.m_g;
                bgB[i] = rgb.m_b;
            }
            backgroundR[a] = percentileInPlace(bgR, kSkyPercentile);
            backgroundG[a] = percentileInPlace(bgG, kSkyPercentile);
            backgroundB[a] = percentileInPlace(bgB, kSkyPercentile);
        }
    }

    // Smooth the background estimates along the dispersion axis: the sky varies slowly
    // there, so this removes the per-column median noise and rejects stars/blobs that
    // happen to sit in the background rows (localised in x), both of which would
    // otherwise imprint on the spectrum as wiggle and negative spikes.
    if (!backgroundRows.isEmpty())
    {
        constexpr int kBackgroundSmoothHalfWindow = 25;
        backgroundR = movingMedian(backgroundR, kBackgroundSmoothHalfWindow);
        backgroundG = movingMedian(backgroundG, kBackgroundSmoothHalfWindow);
        backgroundB = movingMedian(backgroundB, kBackgroundSmoothHalfWindow);
    }

    // Only subtract a background that is actually darker than the trace. If the rows
    // outside the aperture are as bright as the aperture itself, the source fills the
    // RoI and those rows are signal, not background.
    bool backgroundUsable = !backgroundRows.isEmpty();
    if (backgroundUsable)
    {
        int consideredColumns = 0;
        int signalLikeColumns = 0;
        for (int a = 0; a < along; a++)
        {
            const double apertureLevel = (0.299 * rawR[a] + 0.587 * rawG[a] + 0.114 * rawB[a]) / aperture;
            if (apertureLevel < 1.0) {
                continue; // too dark to tell background from signal
            }
            consideredColumns++;
            const double backgroundLevel = 0.299 * backgroundR[a] + 0.587 * backgroundG[a] + 0.114 * backgroundB[a];
            if (backgroundLevel >= kBackgroundSignalRatio * apertureLevel) {
                signalLikeColumns++;
            }
        }
        if ((consideredColumns > 0)
            && (signalLikeColumns > kBackgroundSignalColumnFraction * consideredColumns)) {
            backgroundUsable = false;
        }
    }
    // Only flag when background rows were actually attempted and found to carry signal;
    // aperture = "All" leaves no rows by construction and is documented behaviour, not a
    // filled-RoI condition.
    data.m_backgroundUnavailable = backgroundSubtract && !backgroundRows.isEmpty() && !backgroundUsable;

    if (backgroundUsable) {
        data.m_background.resize(along);
    }
    for (int a = 0; a < along; a++)
    {
        double sumR = rawR[a];
        double sumG = rawG[a];
        double sumB = rawB[a];
        if (backgroundUsable)
        {
            sumR -= aperture * backgroundR[a];
            sumG -= aperture * backgroundG[a];
            sumB -= aperture * backgroundB[a];
            data.m_background[a] = static_cast<float>(aperture
                * (0.299 * backgroundR[a] + 0.587 * backgroundG[a] + 0.114 * backgroundB[a]));
        }
        data.m_red[a] = static_cast<float>(sumR);
        data.m_green[a] = static_cast<float>(sumG);
        data.m_blue[a] = static_cast<float>(sumB);
        data.m_luminance[a] = static_cast<float>(0.299 * sumR + 0.587 * sumG + 0.114 * sumB);
    }

    data.m_saturatedFraction = (along > 0)
        ? static_cast<float>(static_cast<double>(clippedPixels) / (static_cast<double>(along) * aperture))
        : 0.0f;

    data.m_axisOrigin = vertical ? y : x;
    data.m_verticalAxis = vertical;

    // Zero order: strongest peak of the luminance profile, centroid-refined. Only
    // meaningful when the undispersed star image lies inside the RoI; the GUI lets
    // the user override with a manual position.
    const auto peakIt = std::max_element(data.m_luminance.cbegin(), data.m_luminance.cend());
    if ((peakIt != data.m_luminance.cend()) && (*peakIt > 0.0f)) {
        data.m_zeroOrderPx = refinePeakCentroid(data.m_luminance, static_cast<int>(peakIt - data.m_luminance.cbegin()));
    }

    return data;
}

QRgb CameraOpticalSpectrumExtractor::wavelengthToColour(double nm, double intensity)
{
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
    if ((nm >= 380.0) && (nm < 440.0))
    {
        r = (440.0 - nm) / 60.0;
        b = 1.0;
    }
    else if ((nm >= 440.0) && (nm < 490.0))
    {
        g = (nm - 440.0) / 50.0;
        b = 1.0;
    }
    else if ((nm >= 490.0) && (nm < 510.0))
    {
        g = 1.0;
        b = (510.0 - nm) / 20.0;
    }
    else if ((nm >= 510.0) && (nm < 580.0))
    {
        r = (nm - 510.0) / 70.0;
        g = 1.0;
    }
    else if ((nm >= 580.0) && (nm < 645.0))
    {
        r = 1.0;
        g = (645.0 - nm) / 65.0;
    }
    else if ((nm >= 645.0) && (nm <= 780.0))
    {
        r = 1.0;
    }

    // Fade towards the limits of vision
    double falloff = 1.0;
    if ((nm >= 380.0) && (nm < 420.0)) {
        falloff = 0.3 + 0.7 * (nm - 380.0) / 40.0;
    } else if ((nm > 700.0) && (nm <= 780.0)) {
        falloff = 0.3 + 0.7 * (780.0 - nm) / 80.0;
    }

    const double scale = falloff * qBound(0.0, intensity, 1.0);
    const auto component = [scale](double c) {
        constexpr double gammaExponent = 0.8;
        return static_cast<int>(std::lround(255.0 * std::pow(c * scale, gammaExponent)));
    };
    return qRgb(component(r), component(g), component(b));
}

const QVector<CameraOpticalSpectrumRefLineSet>& CameraOpticalSpectrumExtractor::referenceLineSets()
{
    // Air wavelengths in nm. Set keys are stored in the opticalSpectrumRefLines setting,
    // so they (and the line labels within a set) must stay stable.
    static const QVector<CameraOpticalSpectrumRefLineSet> sets = {
        {QStringLiteral("balmer"), QStringLiteral("Balmer series (H)"), {
            {QStringLiteral("H-alpha"), 656.28},
            {QStringLiteral("H-beta"), 486.13},
            {QStringLiteral("H-gamma"), 434.05},
            {QStringLiteral("H-delta"), 410.17},
            {QStringLiteral("H-epsilon"), 397.01},
            {QStringLiteral("H-zeta"), 388.91},
        }},
        {QStringLiteral("hei"), QStringLiteral("Helium"), {
            {QStringLiteral("He I 403"), 402.62},
            {QStringLiteral("He I 447"), 447.15},
            {QStringLiteral("He II 469"), 468.57},
            {QStringLiteral("He I 471"), 471.31},
            {QStringLiteral("He I 492"), 492.19},
            {QStringLiteral("He I 502"), 501.57},
            {QStringLiteral("He I 588"), 587.56},
            {QStringLiteral("He I 668"), 667.82},
            {QStringLiteral("He I 707"), 706.52},
        }},
        {QStringLiteral("naca"), QStringLiteral("Na / Ca"), {
            {QStringLiteral("Ca II K"), 393.37},
            {QStringLiteral("Ca II H"), 396.85},
            {QStringLiteral("Ca I 423"), 422.67},
            {QStringLiteral("Na D"), 589.29},
        }},
        {QStringLiteral("metals"), QStringLiteral("Metals (Fe, Mg)"), {
            {QStringLiteral("Fe I 405"), 404.58},
            {QStringLiteral("Fe I 438"), 438.36},
            {QStringLiteral("Mg b4"), 516.73},
            {QStringLiteral("Mg b2"), 517.27},
            {QStringLiteral("Mg b1"), 518.36},
            {QStringLiteral("Fe E 527"), 526.95},
        }},
        {QStringLiteral("molecular"), QStringLiteral("Molecular bands (CH, C2, TiO)"), {
            {QStringLiteral("CH G band"), 430.40},
            {QStringLiteral("C2 Swan 474"), 473.71},
            {QStringLiteral("TiO 495"), 495.40},
            {QStringLiteral("C2 Swan 517"), 516.52},
            {QStringLiteral("TiO 545"), 544.80},
            {QStringLiteral("TiO 560"), 559.80},
            {QStringLiteral("C2 Swan 564"), 563.55},
            {QStringLiteral("TiO 616"), 615.90},
            {QStringLiteral("TiO 705"), 705.40},
        }},
        {QStringLiteral("nebular"), QStringLiteral("Nebular emission"), {
            {QStringLiteral("[O II] 373"), 372.71},
            {QStringLiteral("[O III] 496"), 495.89},
            {QStringLiteral("[O III] 501"), 500.68},
            {QStringLiteral("[N II] 655"), 654.81},
            {QStringLiteral("[N II] 658"), 658.35},
            {QStringLiteral("[S II] 672"), 671.64},
            {QStringLiteral("[S II] 673"), 673.08},
        }},
        {QStringLiteral("aurora"), QStringLiteral("Aurora / airglow / meteor"), {
            {QStringLiteral("N2+ 391"), 391.44},
            {QStringLiteral("N2+ 428"), 427.81},
            {QStringLiteral("[O I] 558"), 557.73},
            {QStringLiteral("[O I] 630"), 630.03},
            {QStringLiteral("[O I] 636"), 636.38},
            {QStringLiteral("O I 777"), 777.42},
        }, true},
        {QStringLiteral("o2"), QStringLiteral("Telluric O2 / H2O"), {
            {QStringLiteral("O2 B"), 686.72},
            {QStringLiteral("H2O 719"), 718.60},
            {QStringLiteral("O2 A"), 759.37},
        }, true},
    };
    return sets;
}

QVector<CameraOpticalSpectrumRefLine> CameraOpticalSpectrumExtractor::parseCustomLines(const QString& customLines)
{
    QVector<CameraOpticalSpectrumRefLine> lines;
    const QStringList entries = customLines.split(';', Qt::SkipEmptyParts);
    for (const QString& entry : entries)
    {
        const int sep = entry.lastIndexOf(':');
        bool ok = false;
        const double nm = entry.mid(sep + 1).trimmed().toDouble(&ok);
        if (ok && (nm > 0.0)) {
            lines.append({(sep > 0) ? entry.left(sep).trimmed() : QString::number(nm), nm});
        }
    }
    return lines;
}

QVector<CameraOpticalSpectrumRefLine> CameraOpticalSpectrumExtractor::selectedReferenceLines(const QString& refLines, const QString& customLines)
{
    const QStringList tokens = refLines.split(',', Qt::SkipEmptyParts);
    QVector<CameraOpticalSpectrumRefLine> selected;
    const auto selectFromSet = [&tokens, &selected](const QString& key, const QVector<CameraOpticalSpectrumRefLine>& lines, bool terrestrial) {
        const bool wholeSet = tokens.contains(key);
        for (const CameraOpticalSpectrumRefLine& line : lines)
        {
            if (wholeSet || tokens.contains(key + ':' + line.m_label))
            {
                selected.append(line);
                selected.last().m_terrestrial = terrestrial;
            }
        }
    };
    for (const CameraOpticalSpectrumRefLineSet& set : referenceLineSets()) {
        selectFromSet(set.m_key, set.m_lines, set.m_terrestrial);
    }
    selectFromSet(QStringLiteral("custom"), parseCustomLines(customLines), false);
    return selected;
}

CameraOpticalSpectrumCalibration CameraOpticalSpectrumExtractor::calibrateOnePoint(double pixel, double nm, double zeroOrderPixel)
{
    CameraOpticalSpectrumCalibration calibration;
    // Reject reference points too close to the zero order for a usable scale
    if ((nm <= 0.0) || (std::abs(pixel - zeroOrderPixel) < 1.0)) {
        return calibration;
    }
    const double slope = nm / (pixel - zeroOrderPixel);
    calibration.m_valid = true;
    calibration.m_dispersion = std::abs(slope);
    calibration.m_redPositive = slope > 0.0;
    calibration.m_zeroOrderPx = zeroOrderPixel;
    return calibration;
}

CameraOpticalSpectrumCalibration CameraOpticalSpectrumExtractor::calibrateTwoPoint(double pixel1, double nm1, double pixel2, double nm2)
{
    CameraOpticalSpectrumCalibration calibration;
    if ((nm1 <= 0.0) || (nm2 <= 0.0) || (std::abs(pixel2 - pixel1) < 1.0) || (nm1 == nm2)) {
        return calibration;
    }
    const double slope = (nm2 - nm1) / (pixel2 - pixel1);
    calibration.m_valid = true;
    calibration.m_dispersion = std::abs(slope);
    calibration.m_redPositive = slope > 0.0;
    calibration.m_zeroOrderPx = pixel1 - nm1 / slope;
    return calibration;
}

bool CameraOpticalSpectrumExtractor::autoDirectionRedPositive(const CameraOpticalSpectrumData& data)
{
    if (!data.isValid()) {
        return true;
    }
    const float redCentroid = profileCentroid(data.m_red);
    const float blueCentroid = profileCentroid(data.m_blue);
    if ((redCentroid < 0.0f) || (blueCentroid < 0.0f)) {
        return true;
    }
    return redCentroid >= blueCentroid;
}

namespace {

// Median of profile values in [i-far, i-near] and [i+near, i+far], clamped to the
// profile; NaN when too few samples remain to be meaningful
double localContinuum(const QVector<float>& profile, int index, int near, int far)
{
    QVector<float> values;
    values.reserve(2 * (far - near + 1));
    for (int side = -1; side <= 1; side += 2)
    {
        for (int d = near; d <= far; d++)
        {
            const int j = index + side * d;
            if ((j >= 0) && (j < profile.size())) {
                values.append(profile[j]);
            }
        }
    }
    if (values.size() < 10) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return medianInPlace(values);
}

} // namespace

int CameraOpticalSpectrumExtractor::snapToFeature(const QVector<float>& profile, int index, int window)
{
    if (profile.size() < 3) {
        return index;
    }
    const int lo = qBound(0, index - window, profile.size() - 1);
    const int hi = qBound(0, index + window, profile.size() - 1);
    QVector<float> values;
    values.reserve(hi - lo + 1);
    for (int j = lo; j <= hi; j++) {
        values.append(profile[j]);
    }
    const float median = medianInPlace(values);
    int best = index;
    float bestDeviation = 0.0f;
    for (int j = lo; j <= hi; j++)
    {
        const float deviation = std::abs(profile[j] - median);
        if (deviation > bestDeviation)
        {
            bestDeviation = deviation;
            best = j;
        }
    }
    return best;
}

CameraOpticalSpectrumLineMeasurement CameraOpticalSpectrumExtractor::measureLine(const QVector<float>& profile, int index)
{
    CameraOpticalSpectrumLineMeasurement measurement;
    const int n = profile.size();
    if ((n < 30) || (index < 0) || (index >= n)) {
        return measurement;
    }

    const double continuum = localContinuum(profile, index, 12, 45);
    if (std::isnan(continuum)) {
        return measurement;
    }

    // Extremum near the requested position
    int centre = index;
    double amplitude = profile[index] - continuum;
    for (int j = qMax(0, index - 7); j <= qMin(n - 1, index + 7); j++)
    {
        const double a = profile[j] - continuum;
        if (std::abs(a) > std::abs(amplitude))
        {
            amplitude = a;
            centre = j;
        }
    }

    // Require a feature that meaningfully stands out of the profile
    float profileMin = profile[0];
    float profileMax = profile[0];
    for (const float v : profile)
    {
        profileMin = qMin(profileMin, v);
        profileMax = qMax(profileMax, v);
    }
    if ((profileMax - profileMin <= 0.0f) || (std::abs(amplitude) < 0.02 * (profileMax - profileMin))) {
        return measurement;
    }

    const bool emission = amplitude > 0.0;
    const double half = continuum + amplitude / 2.0;

    // Interpolated half-maximum crossings either side of the extremum
    const auto halfCrossing = [&profile, n, half, emission](int from, int step, double& crossing) {
        constexpr int kMaxWalk = 200;
        for (int j = from, walked = 0; (j + step >= 0) && (j + step < n) && (walked < kMaxWalk); j += step, walked++)
        {
            const double v0 = profile[j];
            const double v1 = profile[j + step];
            const bool crossed = emission ? (v1 <= half) : (v1 >= half);
            if (crossed)
            {
                const double frac = (std::abs(v1 - v0) > 1e-12) ? (half - v0) / (v1 - v0) : 0.0;
                crossing = j + step * frac;
                return true;
            }
        }
        return false;
    };
    double left = 0.0;
    double right = 0.0;
    if (!halfCrossing(centre, -1, left) || !halfCrossing(centre, +1, right)) {
        return measurement;
    }

    // Equivalent width between the continuum crossings (where the feature merges back
    // into the continuum, taken as 5% of the amplitude)
    double equivalentWidth = 0.0;
    if (continuum > 0.0)
    {
        const double mergeLevel = 0.05 * std::abs(amplitude);
        constexpr int kMaxWalk = 300;
        for (int side = -1; side <= 1; side += 2)
        {
            for (int j = centre, walked = 0; (j >= 0) && (j < n) && (walked < kMaxWalk); j += side, walked++)
            {
                if ((side == 1) && (j == centre)) {
                    continue; // count the centre sample once
                }
                const double deviation = profile[j] - continuum;
                if (std::abs(deviation) < mergeLevel) {
                    break;
                }
                equivalentWidth += std::abs(deviation) / continuum;
            }
        }
    }

    measurement.m_valid = true;
    measurement.m_emission = emission;
    measurement.m_centreIndex = centre;
    measurement.m_continuum = continuum;
    measurement.m_fwhmSamples = right - left;
    measurement.m_equivalentWidthSamples = equivalentWidth;
    return measurement;
}

QVector<CameraOpticalSpectrumFeature> CameraOpticalSpectrumExtractor::detectFeatures(const QVector<float>& profile, int maxFeatures)
{
    QVector<CameraOpticalSpectrumFeature> features;
    const int n = profile.size();
    if (n < 50) {
        return features;
    }

    // Moving-median continuum: wide enough to bridge lines, narrow enough to follow
    // the real continuum shape
    constexpr int kContinuumHalfWindow = 30;
    QVector<float> continuum(n);
    QVector<float> scratch;
    for (int i = 0; i < n; i++)
    {
        const int lo = qMax(0, i - kContinuumHalfWindow);
        const int hi = qMin(n - 1, i + kContinuumHalfWindow);
        scratch.resize(hi - lo + 1);
        for (int j = lo; j <= hi; j++) {
            scratch[j - lo] = profile[j];
        }
        continuum[i] = medianInPlace(scratch);
    }

    // Robust noise estimate from the residual
    QVector<float> absResidual(n);
    for (int i = 0; i < n; i++) {
        absResidual[i] = std::abs(profile[i] - continuum[i]);
    }
    QVector<float> noiseScratch = absResidual;
    const double noise = 1.4826 * medianInPlace(noiseScratch);
    const double threshold = qMax(5.0 * noise, 1e-6);

    // Local maxima of the residual magnitude above threshold, non-max suppressed
    constexpr int kSuppressionRadius = 8;
    for (int i = 0; i < n; i++)
    {
        if (absResidual[i] < threshold) {
            continue;
        }
        bool isLocalMax = true;
        for (int j = qMax(0, i - kSuppressionRadius); j <= qMin(n - 1, i + kSuppressionRadius); j++)
        {
            if (j == i) {
                continue;
            }
            // Ties break towards the earlier index so a flat-topped feature yields one hit
            if ((absResidual[j] > absResidual[i]) || ((absResidual[j] == absResidual[i]) && (j < i)))
            {
                isLocalMax = false;
                break;
            }
        }
        if (!isLocalMax) {
            continue;
        }
        CameraOpticalSpectrumFeature feature;
        feature.m_index = i;
        feature.m_emission = profile[i] > continuum[i];
        feature.m_prominence = absResidual[i];
        features.append(feature);
    }

    std::sort(features.begin(), features.end(),
        [](const CameraOpticalSpectrumFeature& a, const CameraOpticalSpectrumFeature& b) { return a.m_prominence > b.m_prominence; });
    if (features.size() > maxFeatures) {
        features.resize(maxFeatures);
    }
    return features;
}

bool CameraOpticalSpectrumExtractor::autoDirectionDecisive(const CameraOpticalSpectrumData& data, double minSeparationSamples)
{
    if (!data.isValid()) {
        return false;
    }
    const float redCentroid = profileCentroid(data.m_red);
    const float blueCentroid = profileCentroid(data.m_blue);
    if ((redCentroid < 0.0f) || (blueCentroid < 0.0f)) {
        return false;
    }
    return std::abs(redCentroid - blueCentroid) >= minSeparationSamples;
}
