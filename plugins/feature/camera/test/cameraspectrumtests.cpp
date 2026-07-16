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

// Standalone tests for the optical spectrum extractor. Synthetic frames with known
// trace/background/dip parameters are extracted and the profiles checked against
// the analytic expectation. Run with no arguments; exits non-zero on failure.

#include <cmath>
#include <iostream>

#include <QImage>
#include <QRgba64>
#include <QString>

#include "cameraopticalspectrum.h"
#include "cameraopticalspectrumlibrary.h"

namespace {

int g_failures = 0;

void check(bool condition, const QString& description)
{
    if (condition)
    {
        std::cout << "PASS: " << description.toStdString() << std::endl;
    }
    else
    {
        std::cout << "FAIL: " << description.toStdString() << std::endl;
        g_failures++;
    }
}

// Synthetic scene parameters shared by the tests. The scene has a per-column sky
// background (constant down each column so median subtraction is exact), an 11-row
// spectrum trace whose intensity has a Gaussian absorption dip, and a compact
// bright zero-order blob.
constexpr int kImageWidth = 400;
constexpr int kImageHeight = 120;
constexpr int kRoiX = 50;
constexpr int kRoiY = 20;
constexpr int kRoiWidth = 300;
constexpr int kRoiHeight = 80;
constexpr int kTraceRowFirst = 55; // image coordinates, 11 rows
constexpr int kTraceRowLast = 65;
constexpr int kApertureRows = 11;
constexpr int kDipX = 200;         // image coordinates
constexpr double kDipDepth = 100.0;
constexpr double kDipSigma = 4.0;
constexpr int kZeroOrderX = 80;    // image coordinates, 3 columns wide

double skyBackground(int x)
{
    return 10.0 + 0.05 * x;
}

double traceIntensity(int x)
{
    double value = 150.0 - kDipDepth * std::exp(-std::pow((x - kDipX) / kDipSigma, 2.0) / 2.0);
    if (std::abs(x - kZeroOrderX) <= 1) {
        value += 90.0; // compact zero-order blob on top of the trace
    }
    return value;
}

int greyValue(int x, int y)
{
    double value = skyBackground(x);
    if ((y >= kTraceRowFirst) && (y <= kTraceRowLast)) {
        value += traceIntensity(x);
    }
    return qBound(0, static_cast<int>(std::lround(value)), 255);
}

QImage buildGreyScene(QImage::Format format)
{
    QImage image(kImageWidth, kImageHeight, format);
    for (int y = 0; y < kImageHeight; y++)
    {
        for (int x = 0; x < kImageWidth; x++)
        {
            const int v = greyValue(x, y);
            if (image.depth() > 32) {
                image.setPixelColor(x, y, QColor::fromRgba64(v * 257, v * 257, v * 257));
            } else {
                image.setPixel(x, y, qRgb(v, v, v));
            }
        }
    }
    return image;
}

QImage transposedScene()
{
    QImage image(kImageHeight, kImageWidth, QImage::Format_ARGB32);
    for (int y = 0; y < kImageWidth; y++)
    {
        for (int x = 0; x < kImageHeight; x++)
        {
            const int v = greyValue(y, x); // swap: dispersion axis along image Y
            image.setPixel(x, y, qRgb(v, v, v));
        }
    }
    return image;
}

void testHorizontalExtraction()
{
    const QImage image = buildGreyScene(QImage::Format_ARGB32);
    const CameraOpticalSpectrumData data = CameraOpticalSpectrumExtractor::extract(
        image, kRoiX, kRoiY, kRoiWidth, kRoiHeight, kApertureRows, true);

    check(data.isValid(), "horizontal: extraction produced a profile");
    check(data.m_luminance.size() == kRoiWidth, "horizontal: profile length matches RoI width");
    check(!data.m_verticalAxis, "horizontal: axis direction detected as horizontal");
    check(data.m_axisOrigin == kRoiX, "horizontal: axis origin is the RoI X");

    // Away from the dip and zero order the profile should be apertureRows * trace
    // intensity, with the sky background removed (rounding gives ~ +/- aperture/2).
    bool profileOk = data.isValid() && (data.m_luminance.size() == kRoiWidth);
    double worstError = 0.0;
    if (profileOk)
    {
        for (int a = 0; a < data.m_luminance.size(); a++)
        {
            const int x = kRoiX + a;
            const double expected = kApertureRows * traceIntensity(x);
            const double error = std::abs(data.m_luminance[a] - expected);
            worstError = std::max(worstError, error);
            if (error > kApertureRows * 1.0)
            {
                profileOk = false;
                break;
            }
        }
    }
    check(profileOk, QString("horizontal: background-subtracted profile matches trace (worst error %1)").arg(worstError));

    // Dip minimum in the expected place (excluding the zero-order region)
    if (data.isValid())
    {
        int minIndex = -1;
        float minValue = std::numeric_limits<float>::max();
        for (int a = 0; a < data.m_luminance.size(); a++)
        {
            if (data.m_luminance[a] < minValue)
            {
                minValue = data.m_luminance[a];
                minIndex = a;
            }
        }
        check(std::abs(minIndex - (kDipX - kRoiX)) <= 1, QString("horizontal: absorption dip at expected position (found %1, expected %2)").arg(minIndex).arg(kDipX - kRoiX));
    }

    check(data.m_zeroOrderPx >= 0.0f && std::abs(data.m_zeroOrderPx - (kZeroOrderX - kRoiX)) <= 1.5f,
          QString("horizontal: zero order auto-detected near expected position (found %1, expected %2)").arg(data.m_zeroOrderPx).arg(kZeroOrderX - kRoiX));
}

void testBackgroundSubtractionOff()
{
    const QImage image = buildGreyScene(QImage::Format_ARGB32);
    const CameraOpticalSpectrumData data = CameraOpticalSpectrumExtractor::extract(
        image, kRoiX, kRoiY, kRoiWidth, kRoiHeight, kApertureRows, false);

    // Without background subtraction the sky level remains in the profile
    bool skyPresent = data.isValid();
    if (skyPresent)
    {
        const int a = 20; // away from dip and zero order
        const int x = kRoiX + a;
        const double expected = kApertureRows * (traceIntensity(x) + skyBackground(x));
        skyPresent = std::abs(data.m_luminance[a] - expected) < kApertureRows * 2.0;
    }
    check(skyPresent, "background off: sky level retained in profile");
}

void testFullHeightEmissionSource()
{
    // A discharge tube imaged through a grating: the emission lines are images of the
    // tube, so they span the full frame height. There are no off-trace rows to measure a
    // background from - subtracting one would cancel the spectrum out (the real
    // HydrogenEmissionOrig.jpg case, where every column's background median equalled its
    // aperture mean and the profile collapsed to noise).
    QImage image(400, 200, QImage::Format_ARGB32);
    image.fill(qRgb(0, 0, 0));
    const int lineColumns[] = {100, 200, 300};
    const QRgb lineColours[] = {qRgb(200, 20, 15), qRgb(20, 180, 190), qRgb(90, 40, 200)};
    for (int y = 0; y < image.height(); y++) // full height, as the tube's image is
    {
        for (int i = 0; i < 3; i++)
        {
            for (int dx = -2; dx <= 2; dx++) {
                image.setPixel(lineColumns[i] + dx, y, lineColours[i]);
            }
        }
    }

    const CameraOpticalSpectrumData data = CameraOpticalSpectrumExtractor::extract(image, 0, 0, 0, 0, 15, true);

    // Narrow full-height lines vanish from the along-axis-smoothed background estimate
    // (a wide median over mostly-dark columns), so they are not self-subtracted and no
    // longer need the filled-RoI escape hatch. The lines must survive with their colour.
    bool linesSurvive = data.isValid();
    for (int i = 0; i < 3 && linesSurvive; i++) {
        linesSurvive = data.m_luminance[lineColumns[i]] > 100.0f;
    }
    check(linesSurvive, "full-height: emission lines are not subtracted away");
    check(data.isValid() && (data.m_red[100] > 5.0f * data.m_green[100]),
          "full-height: red line keeps its hue (not washed to white)");
    check(data.isValid() && (data.m_blue[200] > 5.0f * data.m_red[200]),
          "full-height: cyan line keeps its hue");
    check(data.isValid() && (data.m_luminance[50] < 1.0f), "full-height: dark gaps stay dark");

    // A BROAD source filling the RoI is a different matter: its background rows carry
    // the signal at every column and survive the smoothing, so subtraction would cancel
    // the spectrum - that is what the filled-RoI detection must catch
    QImage broad(400, 200, QImage::Format_ARGB32);
    broad.fill(qRgb(120, 120, 120));
    const CameraOpticalSpectrumData broadData = CameraOpticalSpectrumExtractor::extract(broad, 0, 0, 0, 0, 15, true);
    check(broadData.isValid() && broadData.m_backgroundUnavailable,
          "full-height: broad filling source reported unavailable");
    check(broadData.isValid() && (std::abs(broadData.m_luminance[200] - 15 * 120.0f) < 15.0f),
          "full-height: broad source not subtracted away");

    // A genuine trace with dark sky around it must still have its background subtracted
    QImage traced(400, 200, QImage::Format_ARGB32);
    traced.fill(qRgb(30, 30, 30)); // uniform sky pedestal
    for (int y = 95; y <= 105; y++) {
        for (int x = 0; x < 400; x++) {
            traced.setPixel(x, y, qRgb(150, 150, 150));
        }
    }
    const CameraOpticalSpectrumData tracedData = CameraOpticalSpectrumExtractor::extract(traced, 0, 0, 0, 0, 11, true);
    check(tracedData.isValid() && !tracedData.m_backgroundUnavailable,
          "trace: background still subtracted when real sky rows exist");
    check(tracedData.isValid() && (std::abs(tracedData.m_luminance[200] - 11 * 120.0f) < 11 * 2.0f),
          "trace: sky pedestal removed from a genuine trace");

    // Aperture "All" leaves no background rows by construction; that is documented
    // behaviour, not a filled-RoI condition, so it must not raise the warning flag
    const CameraOpticalSpectrumData allRows = CameraOpticalSpectrumExtractor::extract(traced, 0, 0, 0, 0, 0, true);
    check(allRows.isValid() && !allRows.m_backgroundUnavailable,
          "trace: aperture All does not raise the filled-RoI warning");

    // An RoI drawn tightly around the trace leaves no room for sky beyond the aperture
    // and its guard gap: subtraction must be skipped and reported, not fed by the
    // trace's own wings (the real-image case where subtraction "made the spectrum worse")
    const CameraOpticalSpectrumData tight = CameraOpticalSpectrumExtractor::extract(traced, 0, 90, 400, 22, 15, true);
    check(tight.isValid() && tight.m_backgroundInsufficientRows,
          "trace: tight RoI reports insufficient background rows");
    check(tight.isValid() && (std::abs(tight.m_luminance[200] - (11 * 150.0f + 4 * 30.0f)) < 40.0f),
          "trace: tight RoI spectrum not degraded by wing subtraction");
    check(!tracedData.m_backgroundInsufficientRows, "trace: roomy RoI does not report insufficient rows");

    // A defocused star sitting in the background rows must not be subtracted into the
    // spectrum (the real-image case: a blob in the margin swung the per-column median
    // and carved a deep negative spike into the profile at that wavelength)
    QImage starry = traced;
    for (int y = 108; y <= 122; y++) // inside the background rows above the aperture
    {
        for (int x = 194; x <= 206; x++) {
            starry.setPixel(x, y, qRgb(220, 220, 220));
        }
    }
    const CameraOpticalSpectrumData starryData = CameraOpticalSpectrumExtractor::extract(starry, 0, 0, 0, 0, 11, true);
    check(starryData.isValid() && (std::abs(starryData.m_luminance[200] - 11 * 120.0f) < 11 * 5.0f),
          QString("trace: star blob in background rows rejected (got %1, expected %2)")
              .arg(starryData.isValid() ? starryData.m_luminance[200] : -1.0f).arg(11 * 120.0f));
    check(starryData.isValid() && (starryData.m_luminance[200] > 0.0f),
          "trace: no negative spike from background contamination");

    // A defocused trace spreads glow far beyond the aperture (the real-image case:
    // chromatic focus broadened the red end, its halo was sampled as sky, and the
    // continuum went negative). Background rows must sit beyond the trace's measured
    // extent, not just beyond the aperture.
    QImage defocused(400, 200, QImage::Format_ARGB32);
    defocused.fill(qRgb(30, 30, 30)); // sky
    for (int y = 82; y <= 118; y++) // broad glow
    {
        for (int x = 0; x < 400; x++) {
            defocused.setPixel(x, y, qRgb(70, 70, 70));
        }
    }
    for (int y = 97; y <= 103; y++) // bright core
    {
        for (int x = 0; x < 400; x++) {
            defocused.setPixel(x, y, qRgb(180, 180, 180));
        }
    }
    const CameraOpticalSpectrumData defocusedData = CameraOpticalSpectrumExtractor::extract(defocused, 0, 0, 0, 0, 11, true);
    // Aperture (11 rows on the core) sums 7x180 + 4x70 = 1540; true sky is 30/row, so
    // the clean result is 1540 - 11x30 = 1210. Sampling the glow as sky would instead
    // subtract 11x70 = 770 and return ~770.
    check(defocusedData.isValid() && (std::abs(defocusedData.m_luminance[200] - 1210.0f) < 50.0f),
          QString("trace: glow beyond the aperture not sampled as sky (got %1, expected 1210)")
              .arg(defocusedData.isValid() ? defocusedData.m_luminance[200] : -1.0f));
    check(defocusedData.isValid()
              && (defocusedData.m_background.size() == defocusedData.m_luminance.size())
              && (std::abs(defocusedData.m_background[200] - 330.0f) < 25.0f),
          "trace: subtracted background reported for display");

    // A crowded field of defocused stars covers a large fraction of the background rows
    // at EVERY column (the real-image case: the estimate was uniformly inflated, not
    // spiked). The low-percentile sky estimator must track the dark sky between the
    // stars; a median would report the star level once coverage passes half.
    QImage crowded = traced;
    for (int x = 0; x < 400; x++)
    {
        for (int y = 82; y <= 92; y++) {
            crowded.setPixel(x, y, qRgb(100, 100, 100));
        }
        for (int y = 108; y <= 118; y++) {
            crowded.setPixel(x, y, qRgb(100, 100, 100));
        }
    }
    const CameraOpticalSpectrumData crowdedData = CameraOpticalSpectrumExtractor::extract(crowded, 0, 0, 0, 0, 11, true);
    check(crowdedData.isValid() && (std::abs(crowdedData.m_luminance[200] - 11 * 120.0f) < 11 * 5.0f),
          QString("trace: crowded background rows still yield the true sky (got %1, expected %2)")
              .arg(crowdedData.isValid() ? crowdedData.m_luminance[200] : -1.0f).arg(11 * 120.0f));
}

void testSaturationReporting()
{
    QImage image(200, 100, QImage::Format_ARGB32);
    image.fill(qRgb(0, 0, 0));
    for (int y = 40; y < 60; y++) {
        for (int x = 0; x < 200; x++) {
            image.setPixel(x, y, qRgb(255, 255, 255)); // fully blown trace
        }
    }
    const CameraOpticalSpectrumData blown = CameraOpticalSpectrumExtractor::extract(image, 0, 0, 0, 0, 15, false);
    check(blown.isValid() && (blown.m_saturatedFraction > 0.5f), "saturation: blown trace reported");

    QImage dim(200, 100, QImage::Format_ARGB32);
    dim.fill(qRgb(0, 0, 0));
    for (int y = 40; y < 60; y++) {
        for (int x = 0; x < 200; x++) {
            dim.setPixel(x, y, qRgb(120, 60, 40));
        }
    }
    const CameraOpticalSpectrumData ok = CameraOpticalSpectrumExtractor::extract(dim, 0, 0, 0, 0, 15, false);
    check(ok.isValid() && (ok.m_saturatedFraction == 0.0f), "saturation: unclipped trace reports none");
}

void testVerticalExtraction()
{
    const QImage image = transposedScene();
    const CameraOpticalSpectrumData data = CameraOpticalSpectrumExtractor::extract(
        image, kRoiY, kRoiX, kRoiHeight, kRoiWidth, kApertureRows, true);

    check(data.isValid() && data.m_verticalAxis, "vertical: axis direction detected as vertical");
    check(data.m_luminance.size() == kRoiWidth, "vertical: profile length matches RoI height");
    check(data.m_axisOrigin == kRoiX, "vertical: axis origin is the RoI Y");

    bool profileOk = data.isValid() && (data.m_luminance.size() == kRoiWidth);
    if (profileOk)
    {
        for (int a = 0; a < data.m_luminance.size(); a++)
        {
            const int x = kRoiX + a;
            const double expected = kApertureRows * traceIntensity(x);
            if (std::abs(data.m_luminance[a] - expected) > kApertureRows * 1.0)
            {
                profileOk = false;
                break;
            }
        }
    }
    check(profileOk, "vertical: profile matches trace");
}

void testDirectFormatPaths()
{
    // RGB888 is the pipeline's usual processed-image format and is now sampled without
    // a copy/convert; it must produce exactly the same profile as ARGB32
    const QImage argb = buildGreyScene(QImage::Format_ARGB32);
    QImage rgb888(kImageWidth, kImageHeight, QImage::Format_RGB888);
    for (int y = 0; y < kImageHeight; y++)
    {
        uchar* line = rgb888.scanLine(y);
        for (int x = 0; x < kImageWidth; x++)
        {
            const int v = greyValue(x, y);
            line[3 * x] = static_cast<uchar>(v);
            line[3 * x + 1] = static_cast<uchar>(v);
            line[3 * x + 2] = static_cast<uchar>(v);
        }
    }
    const CameraOpticalSpectrumData a = CameraOpticalSpectrumExtractor::extract(argb, kRoiX, kRoiY, kRoiWidth, kRoiHeight, kApertureRows, true);
    const CameraOpticalSpectrumData b = CameraOpticalSpectrumExtractor::extract(rgb888, kRoiX, kRoiY, kRoiWidth, kRoiHeight, kApertureRows, true);
    bool identical = a.isValid() && b.isValid() && (a.m_luminance.size() == b.m_luminance.size());
    for (int i = 0; identical && (i < a.m_luminance.size()); i++) {
        identical = a.m_luminance[i] == b.m_luminance[i];
    }
    check(identical, "formats: RGB888 direct path matches ARGB32 exactly");

    // Grayscale16 must keep its 16-bit precision (previously it was truncated to 8 bits
    // via an ARGB32 conversion). A value of 300/65535 is ~1.167 on the 8-bit scale;
    // truncation would floor it to 1.
    QImage grey16(60, 40, QImage::Format_Grayscale16);
    grey16.fill(Qt::black);
    for (int y = 0; y < 40; y++)
    {
        quint16* line = reinterpret_cast<quint16*>(grey16.scanLine(y));
        line[30] = 300;
    }
    const CameraOpticalSpectrumData g = CameraOpticalSpectrumExtractor::extract(grey16, 0, 0, 0, 0, 10, false);
    const double expected = 10 * (300.0 / 257.0);
    check(g.isValid() && (std::abs(g.m_luminance[30] - expected) < 0.5),
          QString("formats: Grayscale16 keeps 16-bit precision (got %1, expected %2)")
              .arg(g.isValid() ? g.m_luminance[30] : -1.0f).arg(expected));
}

void testDeepColourFormat()
{
    const QImage image8 = buildGreyScene(QImage::Format_ARGB32);
    const QImage image16 = buildGreyScene(QImage::Format_RGBA64);
    const CameraOpticalSpectrumData data8 = CameraOpticalSpectrumExtractor::extract(
        image8, kRoiX, kRoiY, kRoiWidth, kRoiHeight, kApertureRows, true);
    const CameraOpticalSpectrumData data16 = CameraOpticalSpectrumExtractor::extract(
        image16, kRoiX, kRoiY, kRoiWidth, kRoiHeight, kApertureRows, true);

    bool match = data8.isValid() && data16.isValid() && (data8.m_luminance.size() == data16.m_luminance.size());
    if (match)
    {
        for (int a = 0; a < data8.m_luminance.size(); a++)
        {
            if (std::abs(data8.m_luminance[a] - data16.m_luminance[a]) > kApertureRows * 1.0)
            {
                match = false;
                break;
            }
        }
    }
    check(match, "16-bit: profile matches the 8-bit extraction");
}

void testFullFrameRoi()
{
    const QImage image = buildGreyScene(QImage::Format_ARGB32);
    const CameraOpticalSpectrumData data = CameraOpticalSpectrumExtractor::extract(
        image, 0, 0, 0, 0, kApertureRows, true);
    check(data.isValid() && (data.m_luminance.size() == kImageWidth), "full frame: zero RoI selects the whole image");
}

void testAutoDirection()
{
    // Colour spectrum: red energy at high pixel index, blue at low
    QImage image(kImageWidth, 40, QImage::Format_ARGB32);
    image.fill(qRgb(0, 0, 0));
    for (int y = 15; y <= 25; y++)
    {
        for (int x = 0; x < kImageWidth; x++)
        {
            const int red = static_cast<int>(200.0 * std::exp(-std::pow((x - 300.0) / 30.0, 2.0) / 2.0));
            const int blue = static_cast<int>(200.0 * std::exp(-std::pow((x - 100.0) / 30.0, 2.0) / 2.0));
            image.setPixel(x, y, qRgb(red, 0, blue));
        }
    }

    const CameraOpticalSpectrumData data = CameraOpticalSpectrumExtractor::extract(image, 0, 0, 0, 0, kApertureRows, false);
    check(CameraOpticalSpectrumExtractor::autoDirectionRedPositive(data), "direction: red at +pixels detected");

    const QImage mirrored = image.mirrored(true, false);
    const CameraOpticalSpectrumData mirroredData = CameraOpticalSpectrumExtractor::extract(mirrored, 0, 0, 0, 0, kApertureRows, false);
    check(!CameraOpticalSpectrumExtractor::autoDirectionRedPositive(mirroredData), "direction: red at -pixels detected after mirror");

    // Widely separated colour centroids are decisive; a grey source is not, so a
    // latching caller keeps its previous answer instead of flipping with noise
    check(CameraOpticalSpectrumExtractor::autoDirectionDecisive(data), "direction: colour spectrum is decisive");
    const QImage grey = buildGreyScene(QImage::Format_ARGB32);
    const CameraOpticalSpectrumData greyData = CameraOpticalSpectrumExtractor::extract(grey, kRoiX, kRoiY, kRoiWidth, kRoiHeight, kApertureRows, false);
    check(!CameraOpticalSpectrumExtractor::autoDirectionDecisive(greyData), "direction: grey source is not decisive");
}

void testEmptyImage()
{
    const CameraOpticalSpectrumData data = CameraOpticalSpectrumExtractor::extract(QImage(), 0, 0, 0, 0, 15, true);
    check(!data.isValid(), "empty: null image yields invalid data");
}

void testCalibrationSolve()
{
    // Forward model: nm = D * s * (pixel - z) with D = 0.5 nm/px, z = 30
    const double dispersion = 0.5;
    const double zeroOrder = 30.0;
    const auto pixelOf = [dispersion, zeroOrder](double nm, bool redPositive) {
        return zeroOrder + (redPositive ? 1.0 : -1.0) * nm / dispersion;
    };

    const CameraOpticalSpectrumCalibration one = CameraOpticalSpectrumExtractor::calibrateOnePoint(
        pixelOf(656.28, true), 656.28, zeroOrder);
    check(one.m_valid && (std::abs(one.m_dispersion - dispersion) < 1e-9) && one.m_redPositive,
          "calibrate: one point recovers dispersion and direction");

    const CameraOpticalSpectrumCalibration oneFlipped = CameraOpticalSpectrumExtractor::calibrateOnePoint(
        pixelOf(656.28, false), 656.28, zeroOrder);
    check(oneFlipped.m_valid && (std::abs(oneFlipped.m_dispersion - dispersion) < 1e-9) && !oneFlipped.m_redPositive,
          "calibrate: one point recovers flipped direction");

    const CameraOpticalSpectrumCalibration two = CameraOpticalSpectrumExtractor::calibrateTwoPoint(
        pixelOf(486.13, true), 486.13, pixelOf(656.28, true), 656.28);
    check(two.m_valid
              && (std::abs(two.m_dispersion - dispersion) < 1e-9)
              && two.m_redPositive
              && (std::abs(two.m_zeroOrderPx - zeroOrder) < 1e-6),
          "calibrate: two points recover dispersion, direction and zero order");

    const CameraOpticalSpectrumCalibration twoFlipped = CameraOpticalSpectrumExtractor::calibrateTwoPoint(
        pixelOf(486.13, false), 486.13, pixelOf(656.28, false), 656.28);
    check(twoFlipped.m_valid
              && (std::abs(twoFlipped.m_dispersion - dispersion) < 1e-9)
              && !twoFlipped.m_redPositive
              && (std::abs(twoFlipped.m_zeroOrderPx - zeroOrder) < 1e-6),
          "calibrate: two flipped points recover zero order");

    // Two-point order independence
    const CameraOpticalSpectrumCalibration twoSwapped = CameraOpticalSpectrumExtractor::calibrateTwoPoint(
        pixelOf(656.28, true), 656.28, pixelOf(486.13, true), 486.13);
    check(twoSwapped.m_valid
              && (std::abs(twoSwapped.m_dispersion - dispersion) < 1e-9)
              && twoSwapped.m_redPositive
              && (std::abs(twoSwapped.m_zeroOrderPx - zeroOrder) < 1e-6),
          "calibrate: two points in either order give the same result");

    check(!CameraOpticalSpectrumExtractor::calibrateOnePoint(zeroOrder + 0.5, 656.28, zeroOrder).m_valid,
          "calibrate: point at the zero order rejected");
    check(!CameraOpticalSpectrumExtractor::calibrateTwoPoint(100.0, 486.13, 100.5, 656.28).m_valid,
          "calibrate: coincident points rejected");
    check(!CameraOpticalSpectrumExtractor::calibrateTwoPoint(100.0, 656.28, 400.0, 656.28).m_valid,
          "calibrate: equal wavelengths rejected");
}

void testReferenceLineSelection()
{
    const auto lines = [](const QString& refLines, const QString& customLines = QString()) {
        return CameraOpticalSpectrumExtractor::selectedReferenceLines(refLines, customLines);
    };

    check(lines(QStringLiteral("balmer")).size() == 6, "ref lines: whole set token selects every line");
    check(lines(QString()).isEmpty(), "ref lines: empty selection selects nothing");
    check(lines(QStringLiteral("bogus,balmer:Not-a-line")).isEmpty(), "ref lines: unknown tokens ignored");

    const QVector<CameraOpticalSpectrumRefLine> mixed = lines(QStringLiteral("balmer:H-alpha,o2"));
    check((mixed.size() == 4)
              && (mixed[0].m_label == QStringLiteral("H-alpha"))
              && (std::abs(mixed[0].m_nm - 656.28) < 1e-9),
          "ref lines: individual line plus whole set");

    const QString customLines = QStringLiteral("Fe:527.0;Mg:518.4");
    check(lines(QStringLiteral("custom"), customLines).size() == 2, "ref lines: custom set token selects all custom lines");
    const QVector<CameraOpticalSpectrumRefLine> customOne = lines(QStringLiteral("custom:Fe"), customLines);
    check((customOne.size() == 1) && (std::abs(customOne[0].m_nm - 527.0) < 1e-9), "ref lines: individual custom line");

    // Terrestrial (telluric/aurora) lines must be flagged so redshift never moves them
    bool terrestrialOk = true;
    for (const CameraOpticalSpectrumRefLine& line : lines(QStringLiteral("o2,aurora"))) {
        terrestrialOk = terrestrialOk && line.m_terrestrial;
    }
    for (const CameraOpticalSpectrumRefLine& line : lines(QStringLiteral("balmer,hei,naca,metals,molecular,nebular,custom"), customLines)) {
        terrestrialOk = terrestrialOk && !line.m_terrestrial;
    }
    check(terrestrialOk, "ref lines: only telluric and aurora lines are terrestrial");

    // The opticalSpectrumRefLines token format requires unique keys, unique labels within
    // each set, and no separator characters in either
    bool setsOk = true;
    QStringList keys;
    for (const CameraOpticalSpectrumRefLineSet& set : CameraOpticalSpectrumExtractor::referenceLineSets())
    {
        setsOk = setsOk && !set.m_key.isEmpty() && !keys.contains(set.m_key) && !set.m_lines.isEmpty()
            && (set.m_key != QStringLiteral("custom"))
            && !set.m_key.contains(':') && !set.m_key.contains(',');
        keys.append(set.m_key);
        QStringList labels;
        for (const CameraOpticalSpectrumRefLine& line : set.m_lines)
        {
            setsOk = setsOk && !line.m_label.isEmpty() && !labels.contains(line.m_label)
                && !line.m_label.contains(':') && !line.m_label.contains(',')
                && (line.m_nm > 0.0);
            labels.append(line.m_label);
        }
    }
    check(setsOk, "ref lines: set keys and labels are unique and token-safe");
}

void testSpectralTypeParsing()
{
    // Real SIMBAD sp_type strings for typical bright targets, which carry peculiarity
    // codes, ranges and uncertainty that the parser has to see past
    const auto parsed = [](const char* type) { return CameraOpticalSpectrumLibrary::parseSpectralType(QString(type)); };

    const CameraOpticalSpectrumType vega = parsed("A0V"); // Vega
    check(vega.m_valid && (vega.m_class == 'A') && (vega.m_subClass == 0.0) && (vega.m_lum == "V"),
          "sptype: A0V parsed");

    const CameraOpticalSpectrumType sirius = parsed("A0mA1Va"); // Sirius: metallic-line peculiarity
    check(sirius.m_valid && (sirius.m_class == 'A') && (sirius.m_subClass == 0.0) && (sirius.m_lum == "V"),
          "sptype: A0mA1Va parsed (lower-case peculiarity not read as a class)");

    const CameraOpticalSpectrumType betelgeuse = parsed("M1-M2Ia-Iab"); // Betelgeuse: range + Ia
    check(betelgeuse.m_valid && (betelgeuse.m_class == 'M') && (betelgeuse.m_subClass == 1.0) && (betelgeuse.m_lum == "I"),
          "sptype: M1-M2Ia-Iab parsed");

    const CameraOpticalSpectrumType arcturus = parsed("K1.5IIIFe-0.5"); // Arcturus: fractional + III
    check(arcturus.m_valid && (arcturus.m_class == 'K') && (std::abs(arcturus.m_subClass - 1.5) < 1e-9) && (arcturus.m_lum == "III"),
          "sptype: K1.5IIIFe-0.5 parsed (III not read as II)");

    const CameraOpticalSpectrumType rigel = parsed("B8Ia"); // Rigel
    check(rigel.m_valid && (rigel.m_class == 'B') && (rigel.m_subClass == 8.0) && (rigel.m_lum == "I"),
          "sptype: B8Ia parsed");

    const CameraOpticalSpectrumType subgiant = parsed("F5IV");
    check(subgiant.m_valid && (subgiant.m_lum == "IV"), "sptype: IV not read as I");

    check(parsed("A0").m_lum == "V", "sptype: missing luminosity class defaults to dwarf");
    // Types that embed a Harvard letter but are not MK types must not be mis-read
    check(!parsed("").m_valid, "sptype: empty rejected");
    check(!parsed("DA2").m_valid, "sptype: white dwarf DA2 not read as A2");
    check(!parsed("DB3").m_valid, "sptype: white dwarf DB3 not read as B3");
    check(!parsed("sdB").m_valid, "sptype: subdwarf sdB not read as B");
    check(!parsed("WN5").m_valid, "sptype: Wolf-Rayet WN5 rejected");
}

void testTemplateMatching()
{
    check(CameraOpticalSpectrumLibrary::templates().size() == 131, "library: 131 Pickles templates");

    const auto match = [](const char* type) { return CameraOpticalSpectrumLibrary::matchTemplate(QString(type)); };
    check(match("A0V") == "a0v", "match: Vega A0V -> a0v");
    check(match("A0mA1Va") == "a0v", "match: Sirius -> a0v");
    check(match("B8Ia") == "b8i", "match: Rigel B8Ia -> b8i");
    check(match("M1-M2Ia-Iab") == "m2i", "match: Betelgeuse -> m2i (only M supergiant template)");
    check(match("G2V") == "g2v", "match: Sun-like G2V -> g2v");
    check(match("K1.5IIIFe-0.5") == "k1iii", "match: Arcturus -> k1iii");
    check(match("") .isEmpty(), "match: unparsable type gives no template");

    // A dwarf must never match a giant template just because the subclass is closer
    check(CameraOpticalSpectrumLibrary::findTemplate(match("M4V"))->m_lum == "V", "match: M4V stays a dwarf");
    // Metal-rich/weak variants are only selectable manually, never auto-matched
    bool solarOnly = true;
    for (const char* type : {"G2V", "K0III", "F8V", "G5V"}) {
        const CameraOpticalSpectrumTemplate* t = CameraOpticalSpectrumLibrary::findTemplate(match(type));
        solarOnly = solarOnly && t && (t->m_metallicity == 0);
    }
    check(solarOnly, "match: only solar-abundance templates are auto-matched");

    check(CameraOpticalSpectrumLibrary::templateUrl("a0v").endsWith("J/PASP/110/863/a0v.dat.gz"), "library: template URL");
    check(CameraOpticalSpectrumLibrary::findTemplate("nope") == nullptr, "library: unknown key rejected");
}

void testSpectrumDataParsing()
{
    // Real lines from the Pickles a0v.dat file (wavelength in Angstrom, normalised flux)
    const QByteArray data =
        " 1150.0  0.181751  0.348680  0.181751  0.000000\n"
        " 5550.0  1.011062  0.005376  0.000000  0.000000\n"
        " 5555.0  1.012813  0.005018  0.000000  0.000000\n"
        "\n"
        "10620.0  0.145451  0.000000  0.000000  0.000000\n";
    const QVector<QPointF> points = CameraOpticalSpectrumLibrary::parseSpectrumData(data);
    check(points.size() == 4, "data: parsed all non-blank rows");
    check(std::abs(points[0].x() - 115.0) < 1e-9, "data: Angstrom converted to nm");
    check(std::abs(points[2].x() - 555.5) < 1e-9 && std::abs(points[2].y() - 1.012813) < 1e-9,
          "data: flux normalised to ~1.0 at 555.6 nm");
    check(std::abs(points[3].x() - 1062.0) < 1e-9, "data: covers into the near infrared");
    check(CameraOpticalSpectrumLibrary::parseSpectrumData("garbage\nnot data\n").isEmpty(), "data: junk rejected");
    check(CameraOpticalSpectrumLibrary::gunzip("not a gzip stream").isEmpty(), "data: invalid gzip rejected");
    check(CameraOpticalSpectrumLibrary::gunzip(QByteArray()).isEmpty(), "data: empty gzip rejected");

    QString mainId;
    QString spectralType;
    const bool ok = CameraOpticalSpectrumLibrary::parseSimbadResponse(
        "main_id,sp_type,ra,dec\n\"* alf Lyr\",\"A0V\",279.23,38.78\n", mainId, spectralType);
    check(ok && (mainId == "* alf Lyr") && (spectralType == "A0V"), "simbad: quoted CSV response parsed");
    check(!CameraOpticalSpectrumLibrary::parseSimbadResponse("main_id,sp_type\n", mainId, spectralType),
          "simbad: empty result rejected");
}

void testInstrumentResponse()
{
    // Flat true spectrum seen through a Gaussian instrument response: the observed
    // spectrum IS the response, so computeInstrumentResponse must recover it
    const auto trueResponse = [](double nm) { return std::exp(-std::pow((nm - 550.0) / 80.0, 2.0)); };
    QVector<double> wavelengths;
    QVector<float> observed;
    for (double nm = 400.0; nm <= 700.0; nm += 0.5)
    {
        wavelengths.append(nm);
        observed.append(static_cast<float>(1000.0 * trueResponse(nm)));
    }
    const QVector<QPointF> reference = {QPointF(380.0, 1.0), QPointF(760.0, 1.0)}; // flat template

    const QVector<QPointF> response = CameraOpticalSpectrumLibrary::computeInstrumentResponse(
        wavelengths, observed, reference, {}, 12.0, 10.0);
    check(!response.isEmpty(), "response: curve recovered from flat template");
    bool matches = !response.isEmpty();
    for (const double nm : {450.0, 500.0, 550.0, 600.0, 650.0})
    {
        const double got = CameraOpticalSpectrumLibrary::responseAt(response, nm);
        matches = matches && (std::abs(got - trueResponse(nm)) < 0.05);
    }
    check(matches, "response: recovered curve matches the true response");
    check(CameraOpticalSpectrumLibrary::responseAt(response, 200.0) == 0.0, "response: zero outside the measured range");

    // Edge behaviour: the smoothing window shrinks symmetrically, so the curve stays
    // unbiased at its ends rather than bending with the one-sided average
    bool edgesOk = !response.isEmpty();
    for (const double nm : {402.0, 698.0})
    {
        const double got = CameraOpticalSpectrumLibrary::responseAt(response, nm);
        edgesOk = edgesOk && (std::abs(got - trueResponse(nm)) < 0.02);
    }
    check(edgesOk, "response: curve unbiased at its ends");

    // Samples a fraction of a nm beyond the grid hold the end value instead of reading
    // as no-coverage (which blanked the ends of corrected spectra)
    if (!response.isEmpty())
    {
        const double firstNm = response.first().x();
        const double lastNm = response.last().x();
        check(CameraOpticalSpectrumLibrary::responseAt(response, firstNm - 1.0) == response.first().y(),
              "response: end value held just below the grid");
        check(CameraOpticalSpectrumLibrary::responseAt(response, lastNm + 1.5) == response.last().y(),
              "response: end value held just above the grid");
        check(CameraOpticalSpectrumLibrary::responseAt(response, firstNm - 3.0) == 0.0,
              "response: tolerance does not extend far beyond the grid");
    }

    // An absorption line in the observed spectrum must not dent the response when masked
    QVector<float> withLine = observed;
    for (int i = 0; i < wavelengths.size(); i++)
    {
        if (std::abs(wavelengths[i] - 486.13) < 4.0) {
            withLine[i] *= 0.3f; // H-beta absorption
        }
    }
    const QVector<QPointF> masked = CameraOpticalSpectrumLibrary::computeInstrumentResponse(
        wavelengths, withLine, reference, {486.13}, 12.0, 10.0);
    check(!masked.isEmpty()
              && (std::abs(CameraOpticalSpectrumLibrary::responseAt(masked, 486.0) - trueResponse(486.0)) < 0.06),
          "response: masked absorption line interpolated across, not imprinted");

    // Applying the response to the observation recovers the flat true spectrum
    bool flat = !response.isEmpty();
    for (const double nm : {460.0, 550.0, 640.0})
    {
        const int index = static_cast<int>((nm - 400.0) / 0.5);
        const double corrected = observed[index] / CameraOpticalSpectrumLibrary::responseAt(response, nm);
        flat = flat && (std::abs(corrected - 1000.0) < 60.0);
    }
    check(flat, "response: division flattens the observed continuum");

    check(CameraOpticalSpectrumLibrary::computeInstrumentResponse({500.0, 501.0}, {1.0f, 1.0f}, reference, {}, 12.0, 10.0).isEmpty(),
          "response: too little overlap rejected");

    // The Balmer series crowds towards the blue (H-zeta 388.9 to H-gamma 434.1 are only
    // 8-24 nm apart), so fixed +/-12 nm masks merge into one block covering ~377-446 nm
    // and the response used to START at 447 nm, blanking the whole blue end of corrected
    // spectra (the orig.csv/aapply.csv case). Masks must narrow where lines crowd so the
    // continuum slivers between them survive as anchors.
    QVector<double> blueWavelengths;
    QVector<float> blueObserved;
    for (double nm = 384.0; nm <= 700.0; nm += 0.12) // the real capture's sampling
    {
        blueWavelengths.append(nm);
        blueObserved.append(static_cast<float>(1000.0 * trueResponse(nm)));
    }
    const QVector<double> balmerAndTelluric = {656.28, 486.13, 434.05, 410.17, 397.01, 388.91, 686.72, 718.60, 759.37};
    const QVector<QPointF> blueResponse = CameraOpticalSpectrumLibrary::computeInstrumentResponse(
        blueWavelengths, blueObserved, reference, balmerAndTelluric, 12.0, 20.0);
    check(!blueResponse.isEmpty() && (blueResponse.first().x() < 390.0),
          QString("response: crowded Balmer masks do not swallow the blue end (starts at %1 nm)")
              .arg(blueResponse.isEmpty() ? 0.0 : blueResponse.first().x()));
    bool blueOk = !blueResponse.isEmpty();
    for (const double nm : {390.0, 405.0, 420.0, 440.0})
    {
        const double got = CameraOpticalSpectrumLibrary::responseAt(blueResponse, nm);
        blueOk = blueOk && (std::abs(got - trueResponse(nm)) < 0.06);
    }
    check(blueOk, "response: blue end recovered through the Balmer forest");
}

void testLineMeasurement()
{
    // Flat continuum with a Gaussian absorption line and a Gaussian emission line;
    // FWHM of a Gaussian is 2.3548 sigma, EW is (depth/continuum) * sigma * sqrt(2*pi)
    QVector<float> profile(400, 100.0f);
    const double dipSigma = 4.0;
    const double peakSigma = 3.0;
    for (int i = 0; i < profile.size(); i++)
    {
        profile[i] -= static_cast<float>(60.0 * std::exp(-std::pow((i - 200.0) / dipSigma, 2.0) / 2.0));
        profile[i] += static_cast<float>(80.0 * std::exp(-std::pow((i - 320.0) / peakSigma, 2.0) / 2.0));
    }

    const CameraOpticalSpectrumLineMeasurement dip = CameraOpticalSpectrumExtractor::measureLine(profile, 198);
    check(dip.m_valid && !dip.m_emission && (std::abs(dip.m_centreIndex - 200.0) <= 1.0),
          "measure: absorption dip found from a nearby click");
    check(dip.m_valid && (std::abs(dip.m_continuum - 100.0) < 2.0), "measure: continuum level recovered");
    check(dip.m_valid && (std::abs(dip.m_fwhmSamples - 2.3548 * dipSigma) < 1.0),
          QString("measure: dip FWHM correct (got %1, expected %2)").arg(dip.m_fwhmSamples).arg(2.3548 * dipSigma));
    const double expectedEw = 0.6 * dipSigma * std::sqrt(2.0 * 3.14159265358979323846);
    check(dip.m_valid && (std::abs(dip.m_equivalentWidthSamples - expectedEw) < 0.9),
          QString("measure: dip equivalent width correct (got %1, expected %2)").arg(dip.m_equivalentWidthSamples).arg(expectedEw));

    const CameraOpticalSpectrumLineMeasurement peak = CameraOpticalSpectrumExtractor::measureLine(profile, 321);
    check(peak.m_valid && peak.m_emission && (std::abs(peak.m_fwhmSamples - 2.3548 * peakSigma) < 1.0),
          "measure: emission peak FWHM correct");

    const QVector<float> flat(400, 100.0f);
    check(!CameraOpticalSpectrumExtractor::measureLine(flat, 200).m_valid, "measure: featureless profile rejected");

    check(CameraOpticalSpectrumExtractor::snapToFeature(profile, 192) == 200, "snap: click near the dip snaps to it");
    check(CameraOpticalSpectrumExtractor::snapToFeature(profile, 328) == 320, "snap: click near the peak snaps to it");
}

void testFeatureDetection()
{
    QVector<float> profile(400, 100.0f);
    for (int i = 0; i < profile.size(); i++)
    {
        profile[i] -= static_cast<float>(30.0 * std::exp(-std::pow((i - 150.0) / 2.0, 2.0) / 2.0));
        profile[i] += static_cast<float>(50.0 * std::exp(-std::pow((i - 300.0) / 2.0, 2.0) / 2.0));
    }
    const QVector<CameraOpticalSpectrumFeature> features = CameraOpticalSpectrumExtractor::detectFeatures(profile);
    check(features.size() == 2, QString("features: both features detected (got %1)").arg(features.size()));
    if (features.size() == 2)
    {
        check((std::abs(features[0].m_index - 300) <= 2) && features[0].m_emission,
              "features: strongest (emission) first, at the right place");
        check((std::abs(features[1].m_index - 150) <= 2) && !features[1].m_emission,
              "features: absorption dip found");
    }
    check(CameraOpticalSpectrumExtractor::detectFeatures(QVector<float>(400, 100.0f)).isEmpty(),
          "features: flat profile yields none");
}

void testOverlayCsvParse()
{
    const QByteArray csv =
        "pixel,wavelength_nm,luminance,red,green,blue,luminance_corrected\n"
        "100,500.00,1000,1,2,3,2000\n"
        "101,500.12,1100,1,2,3,2200\n";
    const QVector<QPointF> points = CameraOpticalSpectrumLibrary::parseExportedSpectrumCsv(csv);
    check((points.size() == 2) && (std::abs(points[0].y() - 2000.0) < 1e-9),
          "overlay: corrected column preferred");
    const QByteArray plain =
        "pixel,wavelength_nm,luminance,red,green,blue\n"
        "100,500.00,1000,1,2,3\n";
    const QVector<QPointF> plainPoints = CameraOpticalSpectrumLibrary::parseExportedSpectrumCsv(plain);
    check((plainPoints.size() == 1) && (std::abs(plainPoints[0].y() - 1000.0) < 1e-9),
          "overlay: falls back to raw luminance");
    const QByteArray uncalibrated =
        "pixel,wavelength_nm,luminance,red,green,blue\n"
        "100,,1000,1,2,3\n";
    check(CameraOpticalSpectrumLibrary::parseExportedSpectrumCsv(uncalibrated).isEmpty(),
          "overlay: uncalibrated export rejected");
}

void testSdssFitsParse()
{
    // Minimal spSpec-style FITS: one header block, BITPIX -32, log-linear wavelengths
    QByteArray fits;
    const auto card = [&fits](const QByteArray& text) {
        QByteArray padded = text;
        padded.resize(80, ' ');
        fits.append(padded);
    };
    card("SIMPLE  =                    T");
    card("BITPIX  =                  -32");
    card("NAXIS   =                    2");
    card("NAXIS1  =                  100");
    card("NAXIS2  =                    1");
    card("COEFF0  =                  3.0 / log10 Angstrom of first pixel");
    card("COEFF1  =                0.001");
    card("END");
    fits.resize(2880, ' ');
    for (int i = 0; i < 100; i++)
    {
        const float value = static_cast<float>(i + 1);
        quint32 raw = 0;
        std::memcpy(&raw, &value, sizeof(raw));
        fits.append(static_cast<char>((raw >> 24) & 0xFF));
        fits.append(static_cast<char>((raw >> 16) & 0xFF));
        fits.append(static_cast<char>((raw >> 8) & 0xFF));
        fits.append(static_cast<char>(raw & 0xFF));
    }

    const QVector<QPointF> points = CameraOpticalSpectrumLibrary::parseSdssTemplateFits(fits);
    check(points.size() == 100, QString("fits: all samples parsed (got %1)").arg(points.size()));
    if (points.size() == 100)
    {
        check(std::abs(points[0].x() - 100.0) < 1e-6, "fits: first wavelength 10^3.0 A = 100 nm");
        check(std::abs(points[0].y() - 1.0) < 1e-6, "fits: big-endian flux decoded");
        check(std::abs(points[99].x() - std::pow(10.0, 3.0 + 0.099) / 10.0) < 1e-6, "fits: log-linear wavelength scale");
    }
    check(CameraOpticalSpectrumLibrary::parseSdssTemplateFits("not a fits file").isEmpty(), "fits: junk rejected");

    check(CameraOpticalSpectrumLibrary::isEmissionTemplate("qso") && !CameraOpticalSpectrumLibrary::isEmissionTemplate("a0v"),
          "templates: emission keys distinct from stellar keys");
    check(CameraOpticalSpectrumLibrary::emissionTemplateUrl("qso").endsWith("spDR2-029.fit"), "templates: QSO URL");
    check(!CameraOpticalSpectrumLibrary::templateDisplayName("qso").isEmpty()
              && (CameraOpticalSpectrumLibrary::templateDisplayName("a0v") == QStringLiteral("A0 V")),
          "templates: display names for both kinds");
}

void testMaskDeduplication()
{
    // A duplicated mask centre must not cancel the mask (0.4 x zero gap would give a
    // zero-width mask); the dip at the masked wavelength must still be excluded
    const auto trueResponse = [](double nm) { return std::exp(-std::pow((nm - 550.0) / 80.0, 2.0)); };
    QVector<double> wavelengths;
    QVector<float> observed;
    for (double nm = 400.0; nm <= 700.0; nm += 0.5)
    {
        wavelengths.append(nm);
        double v = 1000.0 * trueResponse(nm);
        if (std::abs(nm - 486.13) < 4.0) {
            v *= 0.3;
        }
        observed.append(static_cast<float>(v));
    }
    const QVector<QPointF> reference = {QPointF(380.0, 1.0), QPointF(760.0, 1.0)};
    const QVector<QPointF> response = CameraOpticalSpectrumLibrary::computeInstrumentResponse(
        wavelengths, observed, reference, {486.13, 486.13, 486.13}, 12.0, 10.0);
    check(!response.isEmpty()
              && (std::abs(CameraOpticalSpectrumLibrary::responseAt(response, 486.0) - trueResponse(486.0)) < 0.06),
          "response: duplicated mask centres still mask the line");

    // rawPeakOut reports the pre-normalisation peak
    double rawPeak = 0.0;
    CameraOpticalSpectrumLibrary::computeInstrumentResponse(wavelengths, observed, reference, {486.13}, 12.0, 10.0, &rawPeak);
    check(std::abs(rawPeak - 1000.0) < 30.0, QString("response: raw peak reported (got %1)").arg(rawPeak));
}

void testWavelengthColour()
{
    const QRgb blue = CameraOpticalSpectrumExtractor::wavelengthToColour(460.0);
    check((qBlue(blue) > qRed(blue)) && (qBlue(blue) > qGreen(blue)), "colour: 460 nm is blue dominant");
    const QRgb green = CameraOpticalSpectrumExtractor::wavelengthToColour(530.0);
    check((qGreen(green) > qRed(green)) && (qGreen(green) > qBlue(green)), "colour: 530 nm is green dominant");
    const QRgb red = CameraOpticalSpectrumExtractor::wavelengthToColour(660.0);
    check((qRed(red) > qGreen(red)) && (qRed(red) > qBlue(red)), "colour: 660 nm is red dominant");
    check(CameraOpticalSpectrumExtractor::wavelengthToColour(300.0) == qRgb(0, 0, 0), "colour: UV is black");
    check(CameraOpticalSpectrumExtractor::wavelengthToColour(900.0) == qRgb(0, 0, 0), "colour: IR is black");
    const QRgb dim = CameraOpticalSpectrumExtractor::wavelengthToColour(660.0, 0.25);
    check((qRed(dim) > 0) && (qRed(dim) < qRed(red)), "colour: intensity dims the colour");
    check(CameraOpticalSpectrumExtractor::wavelengthToColour(660.0, 0.0) == qRgb(0, 0, 0), "colour: zero intensity is black");
}

} // namespace

int main(int argc, char* argv[])
{
    (void) argc;
    (void) argv;

    testHorizontalExtraction();
    testBackgroundSubtractionOff();
    testDirectFormatPaths();
    testFullHeightEmissionSource();
    testSaturationReporting();
    testVerticalExtraction();
    testDeepColourFormat();
    testFullFrameRoi();
    testAutoDirection();
    testEmptyImage();
    testCalibrationSolve();
    testReferenceLineSelection();
    testSpectralTypeParsing();
    testTemplateMatching();
    testSpectrumDataParsing();
    testInstrumentResponse();
    testLineMeasurement();
    testFeatureDetection();
    testOverlayCsvParse();
    testSdssFitsParse();
    testMaskDeduplication();
    testWavelengthColour();

    if (g_failures > 0)
    {
        std::cout << g_failures << " test(s) FAILED" << std::endl;
        return 1;
    }
    std::cout << "All tests passed" << std::endl;
    return 0;
}
