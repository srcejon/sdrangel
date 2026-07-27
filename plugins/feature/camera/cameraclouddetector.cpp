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
#include <cstring>

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#ifdef CAMERA_OPENCV_CUDA_CLOUD_DETECTION
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include "util/astronomy.h"
#include "util/profiler.h"
#include "cameraimageutils.h"
#include "camerainfo.h"
#include "cameraclouddetector.h"
#include "cameraplatesolver.h"
#include "cameraskyprojector.h"

MESSAGE_CLASS_DEFINITION(CameraCloudDetector::MsgReportCloudCoverage, Message)
MESSAGE_CLASS_DEFINITION(CameraCloudDetector::MsgSaveCloudTestCase, Message)
MESSAGE_CLASS_DEFINITION(CameraCloudDetector::MsgSaveClearSkyReference, Message)
MESSAGE_CLASS_DEFINITION(CameraCloudDetector::MsgClearClearSkyReference, Message)
MESSAGE_CLASS_DEFINITION(CameraCloudDetector::MsgReportClearSkyReference, Message)

namespace {

// Absolute lower bound on the day-path brightness floor; the effective floor also scales
// with the evaluated sky's median brightness so dark vignette/borders never classify
constexpr int dayMinBrightness = 30;
// Night path: the sky is treated as moonlit when its bright quartile reaches this level -
// i.e. at least a quarter of the evaluated sky is bright, so the colour cues have something
// to work with. Judging by the bright quartile rather than the median keeps half-overcast
// moonlit skies (bright cloud over a dark clear half) on the colour path.
constexpr int moonlitBrightness = 60;
constexpr double moonlitSkyFraction = 0.75;
// A low percentile of the red/blue ratio over the bright sky, taken as "this frame's own
// clear blue". Used where a single number for the whole frame is the right question - the
// day overcast gate (is any blue sky left at all?) and the sun/moon embedded-in-cloud test
// (is the ring around the body whiter than clear sky anywhere?) - never as a detection
// threshold, since clear sky is not one colour across an all-sky frame.
constexpr double moonlitClearSkyFraction = 0.05;
// Moonlit night: cloud is where the red/blue ratio stands this far above the clear sky AT
// THE SAME ELEVATION (the radial profile the day path uses). Relative rather than absolute
// because high gain and night white balance shift the whole ratio scale. A fully overcast
// sky, where the profile can only follow the cloud, is covered by the day threshold applied
// as an absolute backstop alongside it.
constexpr float moonlitRatioMargin = 0.10f;
// Moonlit night: cloud lit by the moon or ground light is also this much brighter than the
// clear sky where it sits - the fitted sky surface, floored at the frame median so the
// surface can only ever raise the bar. Catches grey/white cloud when cloud dominates the
// frame and the colour test has little clear sky to calibrate against.
constexpr double moonlitCloudBrightness = 1.3;
// Night colour is shot-noise limited: overhead the red and blue channels hold only tens of
// counts, so the per-pixel ratio scatters by more than the margin that separates cloud from
// clear sky (measured: a clear zenith ring spanning 0.31 to 0.47 while its true colour is
// 0.36). Averaging the channels over a neighbourhood this fraction of the frame collapses
// that scatter to a few hundredths, and cloud is far larger than the window so nothing that
// matters is blurred away.
constexpr double moonlitColourBlurFraction = 1.0 / 60.0;
// Moonlit night structure vote: fill unflagged sky as cloud where at least this fraction of
// the surrounding unflagged interior carries band-pass structure detections. Lumpy pastel
// cloud banks measure 10-25% detection density, genuinely clear sky under 3%. Only pixels
// at least this fraction of the sky median may vote: cloud lit by twilight or the moon is
// bright, while textured foreground (roofs, aerials) above the dark floor is dim.
constexpr float moonlitStructureFillDensity = 0.045f;
// Flip an entire connected unflagged region to cloud when its interior gated-speck density
// exceeds this; pastel overcast regions measure 4-8%, genuinely clear regions under 0.5%
constexpr float moonlitStructureRegionDensity = 0.02f;
constexpr double moonlitStructureBrightness = 0.5;
// The day structure vote runs only when the bluest part of the bright sky is itself this
// whitish - i.e. the frame holds no genuinely blue sky to lose, so flipping whole lumpy
// regions to cloud cannot swallow a clear gap. A clear or partly clear day sits well below it.
//
// This is an absolute red/blue ratio, deliberately NOT a fraction of the user's day cloud
// threshold. Scaling it with that setting coupled two unrelated decisions: lowering the day
// threshold (the right move on a camera whose cloud is bluer than the 0.85 default assumes)
// also lowered this gate, which silently switched the whole-region flip on and made a partly
// cloudy sky read as overcast. Keeping it fixed leaves the threshold doing one predictable
// job - how white a pixel must be to count as cloud - and leaves this gate answering the
// separate question of whether any clear sky remains.
constexpr double dayOvercastRatioGate = 0.612;
// Near-saturated pixels are exempt from the day texture veto: sunlit cloud tops saturate and
// carry hard bright edges the fine-detail measure reads as texture, while the roofs and
// foliage the veto exists to reject are far darker than a sunlit cloud top.
constexpr int daySaturatedCloud = 230;
// How far above the frame's own sky texture a region must be to count as foreground. The
// texture measure scales with sensor noise, which differs by an order of magnitude between a
// bright low-gain frame and a dark high-gain all-sky exposure, so the user's threshold is
// treated as a floor and raised to clear this frame's noise.
constexpr int dayTextureNoiseMargin = 2;
// Clear-sky colour profile: rings of constant distance from the sky centre (constant
// elevation on an all-sky lens), and the percentile within a ring taken as the clear sky.
// The percentile is low so the profile stays on the clear sky even when much of a ring is
// clouded; the ring count is a compromise between following the horizon ramp and keeping
// enough samples per ring to be robust.
constexpr int kDayProfileRings = 24;
constexpr double kDayProfilePercentile = 0.25;
// Samples the innermost (smallest) ring must keep once subsampled; see the subsampling
// note in dayRelativeCloudMask
constexpr double kDayProfileRingSamples = 400.0;
// Dark night: pixels below this brightness are border or foreground, not sky, and are kept
// out of the local background average so the dark fisheye surround cannot bleed into it
constexpr int darkSkyFloor = 12;
// Below this fraction of the working image there is no sky left to measure, and the result
// must be reported as no result rather than as 0 % cloud. A minimum elevation set above what
// the lens actually covers, an uncalibrated pose, or a detection ROI swallowed by exclusion
// rectangles all land here, and "0 % of nothing" reads as a clear sky to everything
// downstream: the coverage display, the Scheduler events - and the auto-learner, which would
// take the frame as verified clear and overwrite this camera's reference with an empty map.
constexpr double minEvaluatedFraction = 0.01;
// Auto-mode hysteresis band: day at or above the upper bound, night at or below the lower
constexpr double autoDayBrightness = 60.0;
constexpr double autoNightBrightness = 45.0;
// Auto-mode day/night sun-elevation boundary in degrees: at or above this the sky is bright
// enough for the day-path colour cues; below it (twilight and full night) the night path
// runs, since a high-gain camera makes twilight brightness an unreliable day/night signal.
// When no observation time is available the brightness heuristic decides instead.
constexpr double sunDayElevation = -4.0;

// Sun/moon dynamic mask: when a near-saturated glare sits at the projected sun/moon position,
// the connected component of the *final cloud mask* that touches that position is deleted -
// i.e. exactly the false positive the classifier produced, however large the bloom is at the
// current gain and exposure. Growing by brightness instead was tried and fails when the whole
// neighbourhood is washed out (no brightness step to stop at); the classifier's own blob edge
// is the right boundary by construction. The configured radius caps the removal so a genuine
// cloud sheet merely touching the glare cannot be swallowed whole, and a small disc at the
// body position is always excluded from evaluation (the body itself is neither clear nor cloud).
constexpr int sunMoonGlareFloor = 200;        // 0..255: minimum peak brightness to treat the body as visible glare
// When the sky immediately around the body (inside the max disc, outside the seed) is itself
// this fraction flagged as cloud, the body is behind an overcast sheet, not glinting through
// clear sky: its bloom is indistinguishable from the surrounding cloud, so removing it would
// punch a spurious clear hole. Keep the cloud in that case.
constexpr double sunMoonOvercastFraction = 0.80;
// A flagged region lying wholly inside the body's disc is treated as glare debris (a flare
// ghost, or a bloom fragment the morphology detached) only while it is small compared with
// the disc. The disc can be tens of degrees across, and without this bound a whole cumulus
// mass that happens to sit near the sun is deleted as "debris".
constexpr double sunMoonDebrisMaxDiscFraction = 0.05;
// How much whiter than this frame's own clear blue the ring of sky around the body must be
// before the body counts as embedded in cloud rather than glinting through clear sky
constexpr double sunMoonEmbeddedRatioMargin = 0.10;
// Only complain about a body the lens model cannot find when it is high enough that it really
// should be in an all-sky frame; near the horizon, out of frame is perfectly normal
constexpr double sunMoonReportMinElevation = 15.0;
constexpr double sunMoonMinRadiusDeg = 1.5;   // always exclude this disc at the body position from evaluation
constexpr double sunMoonSeedFraction = 0.25;  // seed search radius as a fraction of the max radius (absorbs pose error)

// Star-visibility sensing: predicted catalog stars are checked for a point-source peak in a
// small full-resolution patch around their projected position, and cloud-mask components
// most of whose expected stars are visible are vetoed - stars shining through prove the
// "cloud" is clear sky (or haze thin enough to see through). The check is night-only and
// abstains by construction when no stars are visible (bright twilight, genuine overcast).
constexpr double starSenseMinElevation = 15.0; // degrees: below this, extinction makes visibility unreliable
constexpr double starSenseAvoidBodyDeg = 10.0; // degrees: skip stars this close to the sun or moon (glare/daylight)
constexpr int starSensePatchHalf = 12;        // half-size in full-res pixels of the patch searched around each prediction
constexpr int starSenseMinStars = 2;          // minimum expected stars on a component before the veto may judge it
constexpr double starSenseVisibleFraction = 0.5; // veto a component when at least this fraction of its expected stars are visible
// How far from a visible star the veto may reach, as a fraction of the work image's long
// side. A star proves the line of sight clear where it shines, not across a whole connected
// region: morphological closing and the structure vote can merge everything flagged in a
// frame into one component, and without this bound a handful of stars in a false-positive
// area erases genuine cloud on the far side of the sky along with it (observed: one veto
// call taking a 175 000-pixel mask down to 19). Cloud below the star-sensing elevation
// floor is exactly the part no star can ever vouch for.
constexpr double starVetoReachFraction = 0.06;
// Incremental reference learning: how far around a visible predicted star the sky counts
// as confirmed clear (fraction of the work-image long side)
constexpr double starClearRadiusFraction = 0.04;

// Star-blank cue: the positive counterpart of the visibility veto - a cluster of RECENTLY
// SEEN stars that all vanish is blocked by cloud, whatever the brightness/colour cues say
// (thin or dark cloud can be invisible to both). The recently-seen requirement is the load-
// bearing guard: on real frames most predicted stars fail detection even under a clear sky
// (measured 27/101 on a clear twilight field frame), so absence alone means nothing - only
// the DISAPPEARANCE of a star this camera demonstrably detects is evidence of cloud. The
// frame must additionally prove detection still works (enough stars visible somewhere).
constexpr int kStarBlankMinNeighbours = 2;           // other vanished stars required around a vanished star
constexpr double kStarBlankNeighbourFraction = 0.08; // neighbourhood radius as a fraction of the image long side
constexpr int kStarBlankMinVisible = 5;              // frame sensitivity proof: stars visible somewhere
constexpr qint64 kStarBlankMemorySecs = 1200;        // how recently a star must have been seen for its absence to count

// Sun-visibility check (the day analogue of star sensing, used to gate day auto-learning):
// a near-saturated glare peak at the projected sun position proves that line of sight clear;
// its absence when the sun should be well up proves cloud in front of the sun. Below the
// minimum elevation the check abstains - horizon obstructions and extinction make a low sun
// unreliable evidence either way.
constexpr double sunVisibilitySeedDeg = 3.0;
constexpr double sunVisibilityMinElevation = 5.0;

// Distribution of the 8-bit values where mask is non-zero, built in one pass. Several
// percentiles of the same masked image are usually wanted together (a floor and a ceiling,
// a median and a quartile), and each one otherwise costs a full pass over the image.
struct MaskedHistogram
{
    int bins[256] = {0};
    int total = 0;

    // Robust level estimate that ignores excluded regions and is insensitive to outliers
    [[nodiscard]] int percentile(double fraction) const
    {
        int remaining = std::max(1, static_cast<int>(std::ceil(std::clamp(fraction, 0.0, 1.0) * total)));
        for (int bin = 0; bin < 256; ++bin)
        {
            remaining -= bins[bin];
            if (remaining <= 0) {
                return bin;
            }
        }
        return 0;
    }
};

MaskedHistogram maskedHistogram(const cv::Mat& values, const cv::Mat& mask)
{
    MaskedHistogram histogram;
    for (int row = 0; row < values.rows; ++row)
    {
        const uchar *valueLine = values.ptr<uchar>(row);
        const uchar *maskLine = mask.ptr<uchar>(row);
        for (int col = 0; col < values.cols; ++col)
        {
            if (maskLine[col])
            {
                ++histogram.bins[valueLine[col]];
                ++histogram.total;
            }
        }
    }
    return histogram;
}

// Single percentile of the masked values (0.5 = median)
int maskedPercentile(const cv::Mat& values, const cv::Mat& mask, double fraction)
{
    return maskedHistogram(values, mask).percentile(fraction);
}

// Percentile of the red/blue ratio over the masked pixels; a low percentile estimates the
// clear-sky colour, self-calibrating to the camera's white balance and gain. The pixels are
// spatially subsampled: a percentile over tens of thousands of samples is statistically
// indistinguishable from one over the full image at a fraction of the copying and sorting.
float maskedRatioPercentile(const cv::Mat& red, const cv::Mat& blue, const cv::Mat& mask, double fraction)
{
    constexpr size_t targetSamples = 65536;
    const int step = std::max(1, static_cast<int>(std::lround(std::sqrt(
        static_cast<double>(red.total()) / static_cast<double>(targetSamples)))));

    std::vector<float> samples;
    samples.reserve(static_cast<size_t>(red.total()) / static_cast<size_t>(step * step) + 1);
    for (int row = 0; row < red.rows; row += step)
    {
        const float *redLine = red.ptr<float>(row);
        const float *blueLine = blue.ptr<float>(row);
        const uchar *maskLine = mask.ptr<uchar>(row);
        for (int col = 0; col < red.cols; col += step)
        {
            if (maskLine[col]) {
                samples.push_back(redLine[col] / (blueLine[col] + 1.0f));
            }
        }
    }

    if (samples.empty()) {
        return 0.0f;
    }

    const size_t index = static_cast<size_t>(std::clamp(fraction, 0.0, 1.0) * (samples.size() - 1));
    std::nth_element(samples.begin(), samples.begin() + index, samples.end());
    return samples[index];
}

// Fits a smooth order-3 2D polynomial surface to the masked luminance and returns it as a
// CV_32F image. A dark night sky's large-scale brightness variation - horizon glow, light
// pollution, vignetting - is well described by such a surface; subtracting it isolates
// localized cloud while absorbing the smooth gradient a local box-average would otherwise
// misread as cloud near the bright rim.
cv::Mat polynomialBackground(const cv::Mat& gray, const cv::Mat& skyMask, double floorLevel, double ceilLevel)
{
    constexpr int kTerms = 10;   // 1, x, y, x^2, y^2, xy, x^3, y^3, x^2 y, x y^2
    auto features = [](double x, double y, double *f) {
        f[0] = 1.0; f[1] = x; f[2] = y; f[3] = x * x; f[4] = y * y; f[5] = x * y;
        f[6] = x * x * x; f[7] = y * y * y; f[8] = x * x * y; f[9] = x * y * y;
    };
    const double cx = gray.cols * 0.5;
    const double cy = gray.rows * 0.5;
    const double sx = 1.0 / std::max(1, gray.cols);
    const double sy = 1.0 / std::max(1, gray.rows);

    double ata[kTerms][kTerms] = {{0.0}};
    double atb[kTerms] = {0.0};
    double f[kTerms];
    int count = 0;
    for (int row = 0; row < gray.rows; ++row)
    {
        const uchar *grayLine = gray.ptr<uchar>(row);
        const uchar *maskLine = skyMask.ptr<uchar>(row);
        const double y = (row - cy) * sy;
        for (int col = 0; col < gray.cols; ++col)
        {
            if (!maskLine[col]) {
                continue;
            }
            features((col - cx) * sx, y, f);
            const double v = grayLine[col];
            for (int i = 0; i < kTerms; ++i)
            {
                atb[i] += f[i] * v;
                for (int j = i; j < kTerms; ++j) {
                    ata[i][j] += f[i] * f[j];
                }
            }
            ++count;
        }
    }
    for (int i = 0; i < kTerms; ++i) {
        for (int j = 0; j < i; ++j) {
            ata[i][j] = ata[j][i];
        }
    }

    cv::Mat surface(gray.size(), CV_32F);
    cv::Mat ataMat(kTerms, kTerms, CV_64F, ata);
    cv::Mat atbMat(kTerms, 1, CV_64F, atb);
    cv::Mat coef;
    // Too few sky pixels to fit, or a singular system: fall back to a flat mean level
    if ((count < kTerms * 8) || !cv::solve(ataMat, atbMat, coef, cv::DECOMP_SVD))
    {
        surface.setTo(cv::mean(gray, skyMask)[0]);
        return surface;
    }

    const double *c = coef.ptr<double>();
    for (int row = 0; row < gray.rows; ++row)
    {
        float *surfaceLine = surface.ptr<float>(row);
        const double y = (row - cy) * sy;
        for (int col = 0; col < gray.cols; ++col)
        {
            features((col - cx) * sx, y, f);
            double value = 0.0;
            for (int i = 0; i < kTerms; ++i) {
                value += c[i] * f[i];
            }
            // A cubic surface can overshoot (Runge-style) at the frame edges when a bright
            // cloud dominates the interior, dipping below the real sky level in the corners
            // and manufacturing false cloud there. Clamp the background to the observed sky
            // range so the fitted surface can never extrapolate outside it.
            surfaceLine[col] = static_cast<float>(std::clamp(value, floorLevel, ceilLevel));
        }
    }
    return surface;
}

} // namespace

CameraCloudDetector::CameraCloudDetector() :
    m_msgQueueToFeature(nullptr),
    m_framesSinceUpdate(0),
    m_autoNight(true),
    m_haveAutoModeState(false),
    m_saveReferencePending(false)
{
}

CameraCloudDetector::~CameraCloudDetector() = default;

// Storage key identifying which camera the clear-sky reference belongs to. Live cameras
// have a unique device id and video/stream playback uses the file path or URL as its id,
// but image-sequence playback uses a constant id - substitute the first image path so two
// different sequences never share a reference store. The protocol prefixes the key so ids
// from different backends cannot collide.
QString CameraCloudDetector::referenceStorageKey(const CameraSettings& settings)
{
    QString id = settings.m_cameraId;
    if ((settings.m_cameraProtocol == CameraProtocol::images()) && !settings.m_imageFileCameraPaths.isEmpty()) {
        id = settings.m_imageFileCameraPaths.first();
    }
    return settings.m_cameraProtocol + QLatin1Char('|') + id;
}

bool CameraCloudDetector::handleStageMessage(const Message& cmd)
{
    if (MsgSaveCloudTestCase::match(cmd))
    {
        const MsgSaveCloudTestCase& saveMsg = (const MsgSaveCloudTestCase&) cmd;
        // The request is only honoured on a detection recompute; refuse immediately when
        // that cannot happen rather than latching it to fire on some unrelated future frame
        if (!m_settings.m_cloudDetect)
        {
            if (m_msgQueueToFeature) {
                m_msgQueueToFeature->push(MsgReportClearSkyReference::create(
                    QStringLiteral("Cannot save test case: cloud detection is disabled")));
            }
            return true;
        }
        if (!m_lastInputFrame && !m_captureActive)
        {
            if (m_msgQueueToFeature) {
                m_msgQueueToFeature->push(MsgReportClearSkyReference::create(
                    QStringLiteral("Cannot save test case: no frame available")));
            }
            return true;
        }
        m_saveTestCaseDir = saveMsg.getDirectory();
        invalidateCache();
        if (m_lastInputFrame)
        {
            CameraPipelineFramePtr frame(new CameraPipelineFrame(*m_lastInputFrame));
            frame->m_manualPreviewFrame = true;
            submitFrame(frame);
        }
        else if (m_msgQueueToFeature)
        {
            // Nothing retained to re-run on; the save fires on the next captured frame,
            // which for an idle source may be a long wait - say so rather than nothing
            m_msgQueueToFeature->push(MsgReportClearSkyReference::create(
                QStringLiteral("Save test case pending: waiting for the next frame")));
        }
        return true;
    }
    if (MsgClearClearSkyReference::match(cmd))
    {
        m_clearSkyReference.ensureLoaded(referenceStorageKey(m_settings));
        m_clearSkyReference.clear();
        invalidateCache();
        if (m_msgQueueToFeature) {
            m_msgQueueToFeature->push(MsgReportClearSkyReference::create(
                QStringLiteral("Clear-sky references deleted for this camera")));
        }
        return true;
    }
    if (MsgSaveClearSkyReference::match(cmd))
    {
        if (!m_settings.m_cloudDetect)
        {
            if (m_msgQueueToFeature) {
                m_msgQueueToFeature->push(MsgReportClearSkyReference::create(
                    QStringLiteral("Cannot save reference: cloud detection is disabled")));
            }
            return true;
        }
        // The request is only honoured on a detection recompute; with no retained frame
        // and no running capture there is nothing to capture now, and latching it would
        // fire on the first frame of some later capture - whatever the sky is doing then
        if (!m_lastInputFrame && !m_captureActive)
        {
            if (m_msgQueueToFeature) {
                m_msgQueueToFeature->push(MsgReportClearSkyReference::create(
                    QStringLiteral("Cannot save reference: no frame available")));
            }
            return true;
        }
        // Captured on the next recompute; re-run on the last frame so a paused image or an
        // idle interval-capture camera saves immediately rather than at the next frame
        m_saveReferencePending = true;
        invalidateCache();
        if (m_lastInputFrame)
        {
            CameraPipelineFramePtr frame(new CameraPipelineFrame(*m_lastInputFrame));
            frame->m_manualPreviewFrame = true;
            submitFrame(frame);
        }
        else if (m_msgQueueToFeature)
        {
            m_msgQueueToFeature->push(MsgReportClearSkyReference::create(
                QStringLiteral("Save reference pending: waiting for the next frame")));
        }
        return true;
    }

    return false;
}

bool CameraCloudDetector::cloudSettingsChanged(const QList<QString>& settingsKeys)
{
    return settingsKeys.contains("cloudDetect")
        || settingsKeys.contains("cloudUseDetectionRoi")
        || settingsKeys.contains("cloudMode")
        || settingsKeys.contains("cloudDebugView")
        || settingsKeys.contains("cloudDayThreshold")
        || settingsKeys.contains("cloudTextureThreshold")
        || settingsKeys.contains("cloudNightThreshold")
        || settingsKeys.contains("cloudBackgroundBlur")
        || settingsKeys.contains("cloudOpenSize")
        || settingsKeys.contains("cloudCloseSize")
        || settingsKeys.contains("cloudDownscale")
        || settingsKeys.contains("cloudUpdateIntervalFrames")
        || settingsKeys.contains("detectionRoiX")
        || settingsKeys.contains("detectionRoiY")
        || settingsKeys.contains("detectionRoiWidth")
        || settingsKeys.contains("detectionRoiHeight")
        || settingsKeys.contains("motionExclusionRects")
        // Auto mode derives day/night from the sun elevation at the camera position
        || settingsKeys.contains("latitude")
        || settingsKeys.contains("longitude")
        || settingsKeys.contains("plateSolveUseCaptureDateTime")
        || settingsKeys.contains("plateSolveDateTime")
        || settingsKeys.contains("plateSolveDateTimeUtc")
        || settingsKeys.contains("cloudEdgeMarginPercent")
        || settingsKeys.contains("cloudMinElevation")
        || settingsKeys.contains("cloudDayRelativeMargin")
        || settingsKeys.contains("cloudMaskSunMoon")
        || settingsKeys.contains("cloudSunMoonRadiusDeg")
        || settingsKeys.contains("cloudStarSense")
        || settingsKeys.contains("cloudStarSenseMagnitude")
        || settingsKeys.contains("cloudUseReference")
        || settingsKeys.contains("cloudAutoReference")
        // Star sensing reuses the star detector's peak threshold
        || settingsKeys.contains("starThreshold")
        // The sun/moon mask and star sensing project through the lens model
        || settingsKeys.contains("fov")
        || settingsKeys.contains("azimuth")
        || settingsKeys.contains("elevation")
        || settingsKeys.contains("roll")
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

void CameraCloudDetector::invalidateCache()
{
    m_lastCloud = CameraPipelineCloud();
    m_lastDebugMask = cv::Mat();
    m_lastDebugImage = QImage();
    m_sensedStarCatalog.clear();
    m_sensedStarCatalogMagnitude = -1.0;
    m_sunProjectionReported = false;
    m_moonProjectionReported = false;
    m_noSkyReported = false;
    m_framesSinceUpdate = 0;
    m_lastFrameSize = QSize();
    m_lastContentRect = cv::Rect();
    m_haveAutoModeState = false;
    // Settings changes reach here, so lens-pose/FoV/min-elevation edits rebuild the mask
    m_elevationKeepMask = cv::Mat();
    // Sightings recorded under the old pose/settings do not transfer
    m_starLastVisible.clear();
}

void CameraCloudDetector::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraCloudDetector::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    CameraDetectionStage::applySettings(settings, settingsKeys, force);

    if (force || cloudSettingsChanged(settingsKeys))
    {
        invalidateCache();

        // Re-run on the last frame so tuning is live on a paused/static image
        if (!force && m_lastInputFrame)
        {
            CameraPipelineFramePtr frame(new CameraPipelineFrame(*m_lastInputFrame));
            CameraImageUtils::applyPlaybackProjectionTransform(*frame, m_settings, true);
            frame->m_manualPreviewFrame = true;
            submitFrame(frame);
        }
    }
}

void CameraCloudDetector::captureActiveChanged(bool active)
{
    if (!active)
    {
        // Keep the retained frame so a stopped camera can still save a test case or
        // reference of the frozen display, but downgrade it to CPU so no GPU buffers stay
        // pinned. Pending latched saves are dropped (they only wait when no frame ever
        // arrived, and firing on a later capture would save the wrong scene).
        if (m_lastInputFrame)
        {
            m_lastInputFrame->ensureCpuImageFromCuda();
            m_lastInputFrame->clearCudaCache();
        }
        if ((!m_saveTestCaseDir.isEmpty() || m_saveReferencePending) && m_msgQueueToFeature) {
            m_msgQueueToFeature->push(MsgReportClearSkyReference::create(
                QStringLiteral("Pending save cancelled: capture stopped before a frame arrived")));
        }
        m_saveTestCaseDir.clear();
        m_saveReferencePending = false;
        return;
    }

    // Fresh capture: drop the previous run's retained frame so a save request just after
    // a restart cannot resurrect the old scene before the first new frame arrives, and
    // drop any save still latched from the previous run - it was requested against a scene
    // this capture knows nothing about
    if ((!m_saveTestCaseDir.isEmpty() || m_saveReferencePending) && m_msgQueueToFeature) {
        m_msgQueueToFeature->push(MsgReportClearSkyReference::create(
            QStringLiteral("Pending save cancelled: capture restarted")));
    }
    m_saveTestCaseDir.clear();
    m_saveReferencePending = false;
    m_lastInputFrame.reset();
    invalidateCache();
}

void CameraCloudDetector::processNewFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || !frame->hasImageData()) {
        return;
    }

    frame->m_cloud = CameraPipelineCloud();

    // Retained even while detection is off, so enabling it (or requesting a save) on a
    // paused or finished source re-runs on the displayed frame instead of waiting for a
    // frame that may never come. The copy shares the underlying image buffers, and
    // captureActiveChanged(false) still releases it when capture stops.
    m_lastInputFrame.reset(new CameraPipelineFrame(*frame));

    if (!m_settings.m_cloudDetect)
    {
        forwardFrame(frame);
        return;
    }

    const QSize frameSize = frame->imageSize();
    if (frameSize.isEmpty())
    {
        // Cannot classify, but the frame still belongs to the pipeline like every other
        // early-out here
        forwardFrame(frame);
        return;
    }
    const cv::Size frameCvSize(frameSize.width(), frameSize.height());
    // The shared detection RoI is optional here: an all-sky camera often wants the whole
    // frame's cloud coverage even while motion/star detection is restricted to a region
    const cv::Rect detectionRoi = m_settings.m_cloudUseDetectionRoi
        ? resolveDetectionRoi(frameCvSize)
        : cv::Rect(0, 0, frameCvSize.width, frameCvSize.height);

    // When output scaling pads the image inside a larger canvas, the frame's image transform
    // records where the real content sits; everything outside it is border, not sky
    cv::Rect contentRect(0, 0, frameSize.width(), frameSize.height());
    if (frame->m_imageTransform.isValid())
    {
        const QRect mapped = frame->m_imageTransform.m_opticalToImage.mapRect(
                QRectF(QPointF(0.0, 0.0), QSizeF(frame->m_imageTransform.m_opticalSize)))
            .toAlignedRect()
            .intersected(QRect(0, 0, frameSize.width(), frameSize.height()));
        if (!mapped.isEmpty()) {
            contentRect = cv::Rect(mapped.x(), mapped.y(), mapped.width(), mapped.height());
        }
    }

    const bool debugViewActive = m_settings.m_cloudDebugView != CameraSettings::CloudDebugViewOff;
    ++m_framesSinceUpdate;
    const bool recompute = !m_lastCloud.m_valid
        || (m_framesSinceUpdate >= m_settings.m_cloudUpdateIntervalFrames)
        || (m_lastFrameSize != frameSize)
        || (m_lastCloud.m_roi != detectionRoi)
        || (m_lastContentRect != contentRect)
        || frame->m_manualPreviewFrame;

    if (!recompute)
    {
        // Clouds evolve slowly; stamp the cached result onto intermediate frames. The mask
        // cv::Mat is shared by refcount, so downstream stages must treat it as read-only.
        frame->m_cloud = m_lastCloud;
        // The debug view replaces the frame image, so every frame must carry it (a stale
        // image would flicker against the live feed) - but the render itself only has to
        // happen when the mask changes, so intermediate frames reuse the last one
        if (debugViewActive && !m_lastDebugImage.isNull())
        {
            frame->m_image = m_lastDebugImage;
            frame->clearCudaCache();
        }
        forwardFrame(frame);
        return;
    }

    cv::Mat workBgr;
    cv::Mat rawGray;
    cv::Mat gray;
    bool prepared = false;

#ifdef CAMERA_OPENCV_CUDA_CLOUD_DETECTION
    // When the frame already carries a GPU-resident BGR image, run the only full-resolution
    // work (crop/downscale/luminance/median) on the GPU and download just the small
    // downscaled work images, instead of downloading the whole frame
    if (frame->hasCudaBgrImage()
        && (frame->m_cudaBgrImage.depth() == CV_8U)
        && canUseCudaCloudDetection())
    {
        prepared = prepareWorkImagesCuda(frame->m_cudaBgrImage, detectionRoi, workBgr, rawGray, gray);
    }
#endif
    if (!prepared)
    {
        // Nothing can be classified without pixels. The frame is dropped rather than
        // forwarded, as the motion and object detectors also do on this failure: a GPU
        // download that fails has left the frame without usable image data for any later
        // stage either.
        if (!frame->ensureCpuImageFromCuda()) {
            return;
        }

        prepareWorkImages(frame->m_image, detectionRoi, workBgr, rawGray, gray);
    }

    // Star-visibility sensing reads full-resolution patches, so it runs on the frame
    // rather than the downscaled work images
    const CloudStarSense starSense = senseStarVisibility(frame, frameSize);

    cv::Mat cloudDebugMask;
    applyCloudDetection(workBgr, rawGray, gray, detectionRoi, contentRect, frameSize, frame->m_imageTransform, frame->m_captureDateTime, starSense, frame->m_cloud, debugViewActive ? &cloudDebugMask : nullptr);

    m_lastCloud = frame->m_cloud;
    m_lastFrameSize = frameSize;
    m_lastContentRect = contentRect;
    m_framesSinceUpdate = 0;

    if (!m_saveTestCaseDir.isEmpty()) {
        saveTestCaseBundle(frame);
    }

    if (m_msgQueueToFeature && frame->m_cloud.m_valid) {
        m_msgQueueToFeature->push(MsgReportCloudCoverage::create(frame->m_cloud.m_coveragePercent, frame->m_cloud.m_night, frame->m_captureDateTime));
    }

    m_lastDebugMask = cloudDebugMask;
    if (!m_lastDebugMask.empty()) {
        renderDebugView(frame, frameCvSize, detectionRoi);
    }

    forwardFrame(frame);
}

// Writes a standalone test-case bundle: the exact input image the detection ran on, the
// complete serialized settings (lens pose, position, thresholds - everything), the capture
// time and the measured result, plus the clear-sky reference store when one is in use. The
// bundle reproduces this detection offline via the test harness --run-case mode.
void CameraCloudDetector::saveTestCaseBundle(const CameraPipelineFramePtr& frame)
{
    const QString directory = m_saveTestCaseDir;
    m_saveTestCaseDir.clear();

    QString status;
    if (!QDir().mkpath(directory))
    {
        status = QStringLiteral("Cannot create test case directory %1").arg(directory);
    }
    else if (!frame->ensureCpuImageFromCuda() || frame->m_image.isNull())
    {
        status = QStringLiteral("Cannot save test case: no frame image");
    }
    else if (!frame->m_image.save(QDir(directory).filePath(QStringLiteral("image.png"))))
    {
        status = QStringLiteral("Cannot save test case image in %1").arg(directory);
    }
    else
    {
        bool filesOk = true;
        QFile settingsFile(QDir(directory).filePath(QStringLiteral("settings.dat")));
        if (settingsFile.open(QIODevice::WriteOnly))
        {
            filesOk = settingsFile.write(m_settings.serialize()) >= 0;
            settingsFile.close();
        }
        else
        {
            filesOk = false;
        }

        QJsonObject meta;
        meta.insert(QStringLiteral("formatVersion"), 2);
        // The optical-to-image transform records how output scaling placed the sensor image
        // inside this frame. Everything that maps the sky onto the frame - the sun/moon mask,
        // star sensing, the elevation floor - needs it, so a bundle without it cannot
        // reproduce those: the body would be looked for at the wrong place, or off the frame
        // entirely, and the mask would silently do nothing.
        const CameraPipelineImageTransform& transform = frame->m_imageTransform;
        if (transform.isValid())
        {
            QJsonObject transformMeta;
            transformMeta.insert(QStringLiteral("opticalWidth"), transform.m_opticalSize.width());
            transformMeta.insert(QStringLiteral("opticalHeight"), transform.m_opticalSize.height());
            transformMeta.insert(QStringLiteral("m11"), transform.m_opticalToImage.m11());
            transformMeta.insert(QStringLiteral("m12"), transform.m_opticalToImage.m12());
            transformMeta.insert(QStringLiteral("m21"), transform.m_opticalToImage.m21());
            transformMeta.insert(QStringLiteral("m22"), transform.m_opticalToImage.m22());
            transformMeta.insert(QStringLiteral("dx"), transform.m_opticalToImage.dx());
            transformMeta.insert(QStringLiteral("dy"), transform.m_opticalToImage.dy());
            meta.insert(QStringLiteral("imageTransform"), transformMeta);
        }
        meta.insert(QStringLiteral("captureDateTime"),
            frame->m_captureDateTime.isValid() ? frame->m_captureDateTime.toUTC().toString(Qt::ISODateWithMs) : QString());
        meta.insert(QStringLiteral("cameraProtocol"), m_settings.m_cameraProtocol);
        meta.insert(QStringLiteral("cameraId"), m_settings.m_cameraId);
        meta.insert(QStringLiteral("coveragePercent"), frame->m_cloud.m_valid ? frame->m_cloud.m_coveragePercent : -1.0);
        meta.insert(QStringLiteral("night"), frame->m_cloud.m_valid && frame->m_cloud.m_night);
        // Remove any reference file left by an earlier save into this directory, so the
        // bundle never carries a stale store the current run did not use
        const QStringList staleReferences = QDir(directory).entryList(QStringList{QStringLiteral("*.csr")}, QDir::Files);
        for (const QString& stale : staleReferences) {
            QFile::remove(QDir(directory).filePath(stale));
        }
        bool referenceIncluded = false;
        if (m_settings.m_cloudUseReference)
        {
            m_clearSkyReference.ensureLoaded(referenceStorageKey(m_settings));
            referenceIncluded = m_clearSkyReference.exportTo(directory);
        }
        meta.insert(QStringLiteral("referenceIncluded"), referenceIncluded);

        QFile metaFile(QDir(directory).filePath(QStringLiteral("testcase.json")));
        if (metaFile.open(QIODevice::WriteOnly))
        {
            filesOk = (metaFile.write(QJsonDocument(meta).toJson(QJsonDocument::Indented)) >= 0) && filesOk;
        }
        else
        {
            filesOk = false;
        }

        status = filesOk
            ? QStringLiteral("Saved test case to %1 (coverage %2 %)")
                .arg(directory)
                .arg(frame->m_cloud.m_valid ? frame->m_cloud.m_coveragePercent : -1.0f, 0, 'f', 1)
            : QStringLiteral("Test case in %1 is incomplete: file writes failed").arg(directory);
    }

    qInfo() << "CameraCloudDetector:" << status;
    if (m_msgQueueToFeature) {
        m_msgQueueToFeature->push(MsgReportClearSkyReference::create(status));
    }
}

// Paints the cached debug-view mask onto the frame in place of the camera image. Called on
// every frame while a debug view is active (a stale image would flicker against the live
// feed), while the mask itself is only recomputed on the normal update cadence.
void CameraCloudDetector::renderDebugView(const CameraPipelineFramePtr& frame, const cv::Size& frameCvSize, const cv::Rect& roi)
{
    cv::Mat maskCanvas = cv::Mat::zeros(frameCvSize, CV_8UC1);
    cv::Mat roiMask = m_lastDebugMask;
    if (m_lastDebugMask.size() != roi.size()) {
        cv::resize(m_lastDebugMask, roiMask, roi.size(), 0.0, 0.0, cv::INTER_NEAREST);
    }
    roiMask.copyTo(maskCanvas(roi));
    cv::Mat debugBgr;
    cv::cvtColor(maskCanvas, debugBgr, cv::COLOR_GRAY2BGR);
    m_lastDebugImage = convertBgrToRgbImage(debugBgr);
    frame->m_image = m_lastDebugImage;
    frame->clearCudaCache();
}

bool CameraCloudDetector::resolveNightMode(const cv::Mat& medianGray, const cv::Mat& evaluationMask, const QDateTime& captureDateTime)
{
    switch (m_settings.m_cloudMode)
    {
    case CameraSettings::CloudModeDay:
        return false;
    case CameraSettings::CloudModeNight:
        return true;
    case CameraSettings::CloudModeAuto:
    default:
        break;
    }

    // Prefer the sun elevation at the camera position and the frame's observation time over
    // frame brightness, which auto-exposure cameras distort. Live frames carry the wall
    // clock, video/image playback carries the capture time derived from the file name, and
    // the plate-solve date/time settings supply a manual override for recorded media without
    // one (the same policy the plate solver uses).
    const QDateTime observationTime = m_settings.m_plateSolveUseCaptureDateTime
        ? captureDateTime
        : m_settings.m_plateSolveDateTime;
    if (observationTime.isValid())
    {
        AzAlt sunAzAlt;
        RADec sunRaDec;
        Astronomy::sunPosition(sunAzAlt, sunRaDec, m_settings.m_latitude, m_settings.m_longitude, observationTime);
        // Day only when the sun is up or in early twilight; the whole rest of the
        // sub-horizon range routes to night. Through twilight a high-gain camera makes a
        // dark sky read bright, so frame brightness cannot be trusted to pick day/night;
        // and the night path's moonlit branch already handles bright twilight cloud (and
        // does so better than the day path, whose fixed daylight threshold under-detects
        // pastel pre-dawn cloud), so there is nothing to gain by second-guessing here.
        m_autoNight = sunAzAlt.alt < sunDayElevation;
        m_haveAutoModeState = true;
        return m_autoNight;
    }

    const double meanBrightness = cv::mean(medianGray, evaluationMask)[0];
    if (!m_haveAutoModeState)
    {
        m_autoNight = meanBrightness < (autoDayBrightness + autoNightBrightness) / 2.0;
        m_haveAutoModeState = true;
    }
    else if (meanBrightness >= autoDayBrightness)
    {
        m_autoNight = false;
    }
    else if (meanBrightness <= autoNightBrightness)
    {
        m_autoNight = true;
    }
    // Within the hysteresis band, keep the previous decision so twilight doesn't flap

    return m_autoNight;
}

// Erases the sun and moon from the final cloud mask. Both bodies (and the glare bloom around
// them) read as cloud to every appearance-based cue, so they are handled geometrically: their
// sky az/el comes from the camera location and the frame's observation time, and the same
// fisheye lens model the overlays use projects them onto the frame. When a near-saturated
// glare sits at the projected position, the connected component of the cloud mask touching it
// is deleted - the classifier's own false positive, sized exactly to the bloom at the current
// gain and exposure - clipped to the configured maximum radius. The removed region is also
// excluded from the evaluated area: under the bloom, cloud and clear sky are indistinguishable,
// and counting the removal as clear sky biased measured coverage down on sunlit cloud (a field
// case lost a whole flagged cloud sheet around the sun to the removal and read 5 % on an
// overcast sky). Runs after classification and morphology, before coverage is measured.
void CameraCloudDetector::applySunMoonMask(cv::Mat& mask, cv::Mat& evaluationMask, const cv::Mat& gray, const cv::Mat& workBgr, const cv::Rect& roi, const QSize& imageSize, const CameraPipelineImageTransform& imageTransform, const QDateTime& captureDateTime) const
{
    if (!m_settings.m_cloudMaskSunMoon || (m_settings.m_cloudSunMoonRadiusDeg <= 0.0)) {
        return;
    }

    // Same observation-time policy as the day/night decision: live wall clock, playback
    // capture time, or the plate-solve manual override for recorded media without one.
    const QDateTime observationTime = m_settings.m_plateSolveUseCaptureDateTime
        ? captureDateTime
        : m_settings.m_plateSolveDateTime;
    if (!observationTime.isValid() || (roi.width <= 0) || (roi.height <= 0)) {
        return;
    }

    const SkyProjector projector = SkyProjector::create(m_settings, imageSize, imageTransform);
    if (!projector.valid) {
        return;
    }

    const double maxRadiusDeg = m_settings.m_cloudSunMoonRadiusDeg;
    const double scaleX = static_cast<double>(evaluationMask.cols) / roi.width;
    const double scaleY = static_cast<double>(evaluationMask.rows) / roi.height;

    // This frame's own clear blue, for the embedded-in-cloud test below
    cv::Mat redF, blueF;
    bool haveColour = false;
    float clearAnchorRatio = 0.0f;
    if (!workBgr.empty() && (workBgr.channels() == 3) && (workBgr.size() == gray.size()))
    {
        std::vector<cv::Mat> channels;
        cv::split(workBgr, channels);
        channels[0].convertTo(blueF, CV_32F);
        channels[2].convertTo(redF, CV_32F);
        cv::Mat brightSky;
        cv::bitwise_and(evaluationMask, gray >= dayMinBrightness, brightSky);
        if (cv::countNonZero(brightSky) > 0)
        {
            clearAnchorRatio = maskedRatioPercentile(redF, blueF, brightSky, moonlitClearSkyFraction);
            haveColour = true;
        }
    }

    // A body well above the horizon that the lens model cannot place in the frame, or places
    // outside the sky region, means the pose is wrong - and the mask then does nothing at all,
    // silently, while the body's glare goes on being classified as cloud
    const auto reportUnprojectable = [&](bool moon, const AzAlt& body, const QString& detail)
    {
        bool& reported = moon ? m_moonProjectionReported : m_sunProjectionReported;
        if (reported || (body.alt < sunMoonReportMinElevation) || !m_msgQueueToFeature) {
            return;
        }
        reported = true;
        m_msgQueueToFeature->push(MsgReportClearSkyReference::create(
            QStringLiteral("Sun/moon mask inactive: the body is %1%2 above the horizon but the lens model %3 - check the camera azimuth/elevation/roll, FoV and projection")
                .arg(body.alt, 0, 'f', 0).arg(QChar(0x00b0)).arg(detail)));
    };

    const auto maskBody = [&](bool moon, const AzAlt& body)
    {
        // Skip a body fully below the horizon (its bloom would not reach into the sky region).
        if (body.alt < -maxRadiusDeg) {
            return;
        }

        QPointF centerImage;
        if (!projector.projectAltAz(body.az, body.alt, centerImage))
        {
            reportUnprojectable(moon, body, QStringLiteral("places it outside the image"));
            return; // Outside the frame / behind the camera
        }

        // Measure the angular-to-pixel scale locally by projecting a point one max radius away
        // in elevation; the fisheye scale varies across the frame, so a fixed ratio would be
        // wrong near the edge.
        double radiusImage = 0.0;
        QPointF edgeImage;
        const double offsetElevation = std::min(89.9, body.alt + maxRadiusDeg);
        if (projector.projectAltAz(body.az, offsetElevation, edgeImage)) {
            radiusImage = std::hypot(edgeImage.x() - centerImage.x(), edgeImage.y() - centerImage.y());
        }
        if (radiusImage <= 0.0) {
            // Edge point left the frame: fall back to the mean full-frame angular scale.
            radiusImage = maxRadiusDeg / std::max(1.0, static_cast<double>(m_settings.m_fov)) * imageSize.width();
        }

        const int cx = static_cast<int>(std::lround((centerImage.x() - roi.x) * scaleX));
        const int cy = static_cast<int>(std::lround((centerImage.y() - roi.y) * scaleY));
        const int maxRadiusPx = std::max(1, static_cast<int>(std::lround(radiusImage * scaleX)));

        // Work in a window covering the maximum disc, clipped to the image
        cv::Rect window(cx - maxRadiusPx, cy - maxRadiusPx, 2 * maxRadiusPx + 1, 2 * maxRadiusPx + 1);
        window &= cv::Rect(0, 0, gray.cols, gray.rows);
        if (window.area() <= 0)
        {
            reportUnprojectable(moon, body, QStringLiteral("places it off the edge of the image"));
            return;
        }
        if (cv::countNonZero(evaluationMask(window)) == 0)
        {
            reportUnprojectable(moon, body, QStringLiteral("places it outside the sky region"));
            return;
        }
        const cv::Point centreInWindow(cx - window.x, cy - window.y);
        const cv::Mat grayWindow = gray(window);

        // The body's own disc is neither clear sky nor cloud: exclude it from the evaluated
        // area (and from the mask), whatever the brightness shows.
        const int minRadiusPx = std::max(1, static_cast<int>(std::lround(
            maxRadiusPx * std::min(1.0, sunMoonMinRadiusDeg / maxRadiusDeg))));
        cv::Mat minDisc = cv::Mat::zeros(window.size(), CV_8UC1);
        cv::circle(minDisc, centreInWindow, minRadiusPx, cv::Scalar(255), cv::FILLED);
        evaluationMask(window).setTo(0, minDisc);
        mask(window).setTo(0, minDisc);

        // Glare gate: only remove flagged cloud when a near-saturated peak sits at the body
        // position (within a seed radius that absorbs a few degrees of pose error). A sun or
        // moon hidden behind thick cloud does not saturate, and the cloud in front of it must
        // keep counting as cloud. The gate is absolute, not sky-relative: near an over-exposed
        // sun the entire neighbourhood is washed out, so there is no local contrast to measure.
        const int seedRadiusPx = std::max(minRadiusPx, static_cast<int>(std::lround(maxRadiusPx * sunMoonSeedFraction)));
        cv::Mat seedMask = cv::Mat::zeros(window.size(), CV_8UC1);
        cv::circle(seedMask, centreInWindow, seedRadiusPx, cv::Scalar(255), cv::FILLED);
        double peak = 0.0;
        cv::minMaxLoc(grayWindow, nullptr, &peak, nullptr, nullptr, seedMask);
        if (peak < sunMoonGlareFloor) {
            return;
        }

        // Is the body embedded in cloud rather than glinting through clear sky? Compare the
        // ring of sky around it against this frame's own clear blue: a ring markedly whiter
        // than the clear sky is cloud, and then the body's bloom cannot be told apart from
        // the cloud it sits in, so removing anything would punch a clear hole in real cloud.
        // This reads the image rather than the mask, so it still decides correctly when the
        // classifier has under-detected the very cloud in question - which is exactly the
        // situation on a bright cumulus sky, where the colour test only catches the whitest
        // fragments and the body's disc covers the rest.
        if (haveColour)
        {
            cv::Mat ring = cv::Mat::zeros(window.size(), CV_8UC1);
            cv::circle(ring, centreInWindow, maxRadiusPx, cv::Scalar(255), cv::FILLED);
            cv::circle(ring, centreInWindow, seedRadiusPx, cv::Scalar(0), cv::FILLED);
            cv::bitwise_and(ring, evaluationMask(window), ring);
            cv::bitwise_and(ring, gray(window) >= dayMinBrightness, ring);
            if (cv::countNonZero(ring) > 0)
            {
                const float ringRatio = maskedRatioPercentile(redF(window), blueF(window), ring, 0.5);
                if (ringRatio > clearAnchorRatio + sunMoonEmbeddedRatioMargin) {
                    return;
                }
            }
        }

        // Delete the connected components of the final cloud mask that belong to the glare -
        // the classifier's own false positive, sized exactly to the bloom. Two kinds qualify:
        // components touching the seed (the bloom itself), and components lying entirely
        // inside the max-radius disc (flare ghosts and bloom fragments detached by the
        // morphology). A genuine cloud sheet reaching in from outside the disc is only
        // trimmed where it overlaps the disc, never removed wholesale.
        if (cv::countNonZero(mask(window)) == 0) {
            return;
        }
        cv::Mat discMask = cv::Mat::zeros(window.size(), CV_8UC1);
        cv::circle(discMask, centreInWindow, maxRadiusPx, cv::Scalar(255), cv::FILLED);

        // If the sky around the body (inside the disc, outside the seed) is itself mostly
        // flagged cloud, the body is behind an overcast sheet rather than glinting through
        // clear sky. Its bloom cannot be told from the surrounding cloud, and trimming the
        // disc would punch a spurious clear hole in the overcast - so keep the cloud (the
        // body's own min-disc is already excluded above).
        cv::Mat annulus;
        cv::subtract(discMask, seedMask, annulus);
        const int annulusArea = cv::countNonZero(annulus);
        if (annulusArea > 0)
        {
            cv::Mat flaggedAnnulus;
            cv::bitwise_and(mask(window), annulus, flaggedAnnulus);
            if (cv::countNonZero(flaggedAnnulus) >= sunMoonOvercastFraction * annulusArea) {
                return;
            }
        }

        cv::Mat labels;
        const int labelCount = cv::connectedComponents(mask(window), labels, 8);
        std::vector<uchar> touchesSeed(static_cast<size_t>(labelCount), 0);
        std::vector<uchar> leavesDisc(static_cast<size_t>(labelCount), 0);
        // A glare bloom is a contained blob around the body; a flagged region that runs out
        // of the analysed window is a cloud sheet that merely happens to lie over the body,
        // so it must not be trimmed at all. Window edges that sit on the image edge do not
        // count - there the region is cut by the frame, not by leaving the body's disc.
        std::vector<uchar> escapesWindow(static_cast<size_t>(labelCount), 0);
        const bool openTop = window.y > 0;
        const bool openBottom = (window.y + window.height) < gray.rows;
        const bool openLeft = window.x > 0;
        const bool openRight = (window.x + window.width) < gray.cols;
        for (int row = 0; row < labels.rows; ++row)
        {
            const int *labelLine = labels.ptr<int>(row);
            const uchar *seedLine = seedMask.ptr<uchar>(row);
            const uchar *discLine = discMask.ptr<uchar>(row);
            const bool edgeRow = (openTop && (row == 0)) || (openBottom && (row == labels.rows - 1));
            for (int col = 0; col < labels.cols; ++col)
            {
                if (labelLine[col] > 0)
                {
                    if (seedLine[col]) {
                        touchesSeed[static_cast<size_t>(labelLine[col])] = 1;
                    }
                    if (!discLine[col]) {
                        leavesDisc[static_cast<size_t>(labelLine[col])] = 1;
                    }
                    if (edgeRow || (openLeft && (col == 0)) || (openRight && (col == labels.cols - 1))) {
                        escapesWindow[static_cast<size_t>(labelLine[col])] = 1;
                    }
                }
            }
        }

        // Component areas, for the debris size bound below
        std::vector<int> componentArea(static_cast<size_t>(labelCount), 0);
        for (int row = 0; row < labels.rows; ++row)
        {
            const int *labelLine = labels.ptr<int>(row);
            for (int col = 0; col < labels.cols; ++col)
            {
                if (labelLine[col] > 0) {
                    ++componentArea[static_cast<size_t>(labelLine[col])];
                }
            }
        }
        const int discArea = std::max(1, cv::countNonZero(discMask));

        cv::Mat removal = cv::Mat::zeros(window.size(), CV_8UC1);
        for (int row = 0; row < labels.rows; ++row)
        {
            const int *labelLine = labels.ptr<int>(row);
            const uchar *discLine = discMask.ptr<uchar>(row);
            uchar *removalLine = removal.ptr<uchar>(row);
            for (int col = 0; col < labels.cols; ++col)
            {
                const int label = labelLine[col];
                if (label <= 0) {
                    continue;
                }
                // Seed-touching blobs are trimmed within the disc, unless they escape the
                // window entirely (a cloud sheet over the body, not its bloom); blobs wholly
                // inside the disc are glare debris and removed outright
                const bool bloomLike = touchesSeed[static_cast<size_t>(label)]
                    && !escapesWindow[static_cast<size_t>(label)];
                const bool debrisLike = !leavesDisc[static_cast<size_t>(label)]
                    && (componentArea[static_cast<size_t>(label)] <= discArea * sunMoonDebrisMaxDiscFraction);
                if ((bloomLike && discLine[col]) || debrisLike) {
                    removalLine[col] = 255;
                }
            }
        }
        mask(window).setTo(0, removal);
        evaluationMask(window).setTo(0, removal);
    };

    AzAlt azAlt;
    RADec raDec;
    Astronomy::sunPosition(azAlt, raDec, m_settings.m_latitude, m_settings.m_longitude, observationTime);
    maskBody(false, azAlt);
    Astronomy::moonPosition(azAlt, raDec, m_settings.m_latitude, m_settings.m_longitude, observationTime);
    maskBody(true, azAlt);
}

// Checks for the sun's glare at its projected position - the day analogue of star-visibility
// sensing. Visible glare proves that line of sight clear; a well-up sun with no glare proves
// cloud in front of it. Used to keep a day frame whose sun is visibly obscured from being
// auto-learned as the clear-sky reference: day learning otherwise has only the detector's own
// coverage reading to verify itself with, which is circular exactly on the cameras where dark
// overcast is colorimetrically invisible.
CameraCloudDetector::BodyVisibility CameraCloudDetector::sunVisibility(const cv::Mat& gray, const cv::Rect& roi, const QSize& imageSize, const CameraPipelineImageTransform& imageTransform, const QDateTime& captureDateTime) const
{
    const QDateTime observationTime = m_settings.m_plateSolveUseCaptureDateTime
        ? captureDateTime
        : m_settings.m_plateSolveDateTime;
    if (!observationTime.isValid() || (roi.width <= 0) || (roi.height <= 0)) {
        return BodyVisibility::Unknown;
    }

    AzAlt sunAzAlt;
    RADec sunRaDec;
    Astronomy::sunPosition(sunAzAlt, sunRaDec, m_settings.m_latitude, m_settings.m_longitude, observationTime);
    if (sunAzAlt.alt < sunVisibilityMinElevation) {
        return BodyVisibility::Unknown;
    }

    const SkyProjector projector = SkyProjector::create(m_settings, imageSize, imageTransform);
    if (!projector.valid) {
        return BodyVisibility::Unknown;
    }
    QPointF centerImage;
    if (!projector.projectAltAz(sunAzAlt.az, sunAzAlt.alt, centerImage)) {
        return BodyVisibility::Unknown;
    }

    // Local angular-to-pixel scale, as in applySunMoonMask: the fisheye scale varies across
    // the frame
    double radiusImage = 0.0;
    QPointF edgeImage;
    const double offsetElevation = std::min(89.9, sunAzAlt.alt + sunVisibilitySeedDeg);
    if (projector.projectAltAz(sunAzAlt.az, offsetElevation, edgeImage)) {
        radiusImage = std::hypot(edgeImage.x() - centerImage.x(), edgeImage.y() - centerImage.y());
    }
    if (radiusImage <= 0.0) {
        radiusImage = sunVisibilitySeedDeg / std::max(1.0, static_cast<double>(m_settings.m_fov)) * imageSize.width();
    }

    const double scaleX = static_cast<double>(gray.cols) / roi.width;
    const double scaleY = static_cast<double>(gray.rows) / roi.height;
    const int cx = static_cast<int>(std::lround((centerImage.x() - roi.x) * scaleX));
    const int cy = static_cast<int>(std::lround((centerImage.y() - roi.y) * scaleY));
    const int seedPx = std::max(2, static_cast<int>(std::lround(radiusImage * scaleX)));

    cv::Rect window(cx - seedPx, cy - seedPx, 2 * seedPx + 1, 2 * seedPx + 1);
    window &= cv::Rect(0, 0, gray.cols, gray.rows);
    if (window.area() <= 0) {
        return BodyVisibility::Unknown; // Projected outside the evaluated region
    }

    cv::Mat seedMask = cv::Mat::zeros(window.size(), CV_8UC1);
    cv::circle(seedMask, cv::Point(cx - window.x, cy - window.y), seedPx, cv::Scalar(255), cv::FILLED);
    double peak = 0.0;
    cv::minMaxLoc(gray(window), nullptr, &peak, nullptr, nullptr, seedMask);
    return (peak >= sunMoonGlareFloor) ? BodyVisibility::Visible : BodyVisibility::Obscured;
}

// Excludes the non-illuminated surround - the dark region connected to the frame border -
// from evaluation. On a fisheye the image circle fills only part of the frame, and letterbox
// bars pad scaled output; that black is not sky, yet it otherwise dilutes the coverage
// denominator (a fully overcast fisheye reading only ~40 %) and drags down the sky median the
// brightness floors key on. Only border-connected dark is removed, so interior foreground and
// dark cloud are kept, and the removal is skipped when it would take essentially the whole
// frame - a uniformly dark night, where the dark region IS the sky.
void CameraCloudDetector::excludeSurround(cv::Mat& evaluationMask, const cv::Mat& gray)
{
    cv::Mat dark;
    cv::compare(gray, darkSkyFloor, dark, cv::CMP_LT);
    cv::bitwise_and(dark, evaluationMask, dark);
    if (cv::countNonZero(dark) == 0) {
        return;
    }

    cv::Mat labels;
    const int labelCount = cv::connectedComponents(dark, labels, 8);
    if (labelCount <= 1) {
        return;
    }

    // A dark component is surround if it reaches the frame border
    std::vector<uchar> touchesBorder(static_cast<size_t>(labelCount), 0);
    const auto markRow = [&](int row) {
        const int *line = labels.ptr<int>(row);
        for (int col = 0; col < labels.cols; ++col) {
            if (line[col] > 0) { touchesBorder[static_cast<size_t>(line[col])] = 1; }
        }
    };
    markRow(0);
    markRow(labels.rows - 1);
    for (int row = 0; row < labels.rows; ++row)
    {
        const int *line = labels.ptr<int>(row);
        if (line[0] > 0) { touchesBorder[static_cast<size_t>(line[0])] = 1; }
        if (line[labels.cols - 1] > 0) { touchesBorder[static_cast<size_t>(line[labels.cols - 1])] = 1; }
    }

    cv::Mat surround = cv::Mat::zeros(gray.size(), CV_8UC1);
    for (int row = 0; row < labels.rows; ++row)
    {
        const int *labelLine = labels.ptr<int>(row);
        uchar *surroundLine = surround.ptr<uchar>(row);
        for (int col = 0; col < labels.cols; ++col)
        {
            if ((labelLine[col] > 0) && touchesBorder[static_cast<size_t>(labelLine[col])]) {
                surroundLine[col] = 255;
            }
        }
    }

    // Would removing it leave almost nothing? Then this is not a fisheye surround but a
    // genuinely dark frame - leave the evaluation mask untouched.
    cv::Mat remaining;
    cv::bitwise_and(evaluationMask, ~surround, remaining);
    if (cv::countNonZero(remaining) < (gray.rows * gray.cols) / 50) {
        return;
    }
    evaluationMask = remaining;
}

// Optional sky-elevation floor: excludes sky below the configured elevation from
// classification and the coverage denominator. On an all-sky lens the horizon band
// dominates the pixel count while mattering least for observation, so without this a clear
// zenith under horizon murk reads as heavily clouded. The per-pixel unprojection is pure
// geometry, so the mask is cached and rebuilt only when the geometry it was computed for
// changes (settings changes invalidate it via invalidateCache()).
void CameraCloudDetector::applyMinElevationMask(cv::Mat& evaluationMask, const cv::Rect& roi, const QSize& imageSize, const CameraPipelineImageTransform& imageTransform)
{
    const cv::Size workSize = evaluationMask.size();
    // The transform is part of the geometry, not incidental to it: output scaling can move
    // the content inside an unchanged frame (changing the justification alone does exactly
    // that), which changes where every elevation lands while the roi, work size and image
    // size all stay put. Leaving it out of the key kept a mask built for the old placement.
    const bool sameTransform = (m_elevationMaskTransform.m_enabled == imageTransform.m_enabled)
        && (m_elevationMaskTransform.m_opticalSize == imageTransform.m_opticalSize)
        && (m_elevationMaskTransform.m_opticalToImage == imageTransform.m_opticalToImage);
    const bool cacheValid = !m_elevationKeepMask.empty()
        && (m_elevationMaskRoi == roi)
        && (m_elevationMaskWorkSize == workSize)
        && (m_elevationMaskImageSize == imageSize)
        && sameTransform;
    if (!cacheValid)
    {
        m_elevationKeepMask = cv::Mat();
        m_elevationMaskRoi = roi;
        m_elevationMaskWorkSize = workSize;
        m_elevationMaskImageSize = imageSize;
        m_elevationMaskTransform = imageTransform;

        const SkyProjector projector = SkyProjector::create(m_settings, imageSize, imageTransform);
        if (!projector.valid) {
            return; // No usable lens pose: the floor cannot be evaluated, so nothing is excluded
        }
        const double minElevation = m_settings.m_cloudMinElevation;
        const double pxPerWorkX = static_cast<double>(roi.width) / workSize.width;
        const double pxPerWorkY = static_cast<double>(roi.height) / workSize.height;
        cv::Mat keep(workSize, CV_8UC1);
        for (int row = 0; row < workSize.height; ++row)
        {
            uchar *keepLine = keep.ptr<uchar>(row);
            const double imageY = roi.y + (row + 0.5) * pxPerWorkY;
            for (int col = 0; col < workSize.width; ++col)
            {
                const QPointF imagePoint(roi.x + (col + 0.5) * pxPerWorkX, imageY);
                double azimuth = 0.0;
                double elevation = -90.0;
                keepLine[col] = (projector.unprojectToAltAz(imagePoint, azimuth, elevation)
                    && (elevation >= minElevation)) ? 255 : 0;
            }
        }
        m_elevationKeepMask = keep;
    }

    if (!m_elevationKeepMask.empty()) {
        cv::bitwise_and(evaluationMask, m_elevationKeepMask, evaluationMask);
    }
}

// Extracts a full-resolution grayscale patch centred on a predicted star position. Works
// from the CPU image when present, otherwise downloads just the patch from the GPU frame.
bool CameraCloudDetector::samplePatchGray(const CameraPipelineFramePtr& frame, const QPoint& centre, int half, cv::Mat& patch,
                                          const cv::Mat& fullResBgr)
{
    const int size = 2 * half + 1;

    // A frame downloaded once by the caller: same qGray weights as both other paths
    if (!fullResBgr.empty())
    {
        if ((centre.x() < half) || (centre.y() < half)
            || (centre.x() + half >= fullResBgr.cols) || (centre.y() + half >= fullResBgr.rows)) {
            return false;
        }
        const cv::Mat bgr = fullResBgr(cv::Rect(centre.x() - half, centre.y() - half, size, size));
        patch.create(size, size, CV_8UC1);
        for (int row = 0; row < size; ++row)
        {
            const cv::Vec3b *bgrLine = bgr.ptr<cv::Vec3b>(row);
            uchar *line = patch.ptr<uchar>(row);
            for (int col = 0; col < size; ++col) {
                line[col] = static_cast<uchar>(qGray(bgrLine[col][2], bgrLine[col][1], bgrLine[col][0]));
            }
        }
        return true;
    }

    if (!frame->m_image.isNull())
    {
        const QImage& image = frame->m_image;
        if ((centre.x() < half) || (centre.y() < half)
            || (centre.x() + half >= image.width()) || (centre.y() + half >= image.height())) {
            return false;
        }
        patch.create(size, size, CV_8UC1);
        const int x0 = centre.x() - half;
        for (int row = 0; row < size; ++row)
        {
            uchar *line = patch.ptr<uchar>(row);
            const uchar *scan = image.constScanLine(centre.y() - half + row);
            // Direct scanline access for the common pipeline formats; QImage::pixel() does a
            // per-call format dispatch that dominates the patch cost otherwise
            switch (image.format())
            {
            case QImage::Format_RGB32:
            case QImage::Format_ARGB32:
            case QImage::Format_ARGB32_Premultiplied:
            {
                const QRgb *rgbLine = reinterpret_cast<const QRgb*>(scan) + x0;
                for (int col = 0; col < size; ++col) {
                    line[col] = static_cast<uchar>(qGray(rgbLine[col]));
                }
                break;
            }
            case QImage::Format_RGB888:
            {
                const uchar *rgbLine = scan + 3 * x0;
                for (int col = 0; col < size; ++col) {
                    line[col] = static_cast<uchar>(qGray(rgbLine[3 * col], rgbLine[3 * col + 1], rgbLine[3 * col + 2]));
                }
                break;
            }
            case QImage::Format_Grayscale8:
            {
                std::memcpy(line, scan + x0, static_cast<size_t>(size));
                break;
            }
            default:
                for (int col = 0; col < size; ++col) {
                    line[col] = static_cast<uchar>(qGray(image.pixel(x0 + col, centre.y() - half + row)));
                }
                break;
            }
        }
        return true;
    }

#ifdef CAMERA_OPENCV_CUDA_CLOUD_DETECTION
    if (frame->hasCudaBgrImage() && (frame->m_cudaBgrImage.depth() == CV_8U) && (frame->m_cudaBgrImage.channels() == 3))
    {
        const cv::cuda::GpuMat& gpu = frame->m_cudaBgrImage;
        if ((centre.x() < half) || (centre.y() < half)
            || (centre.x() + half >= gpu.cols) || (centre.y() + half >= gpu.rows)) {
            return false;
        }
        const cv::Rect rect(centre.x() - half, centre.y() - half, size, size);
        cv::Mat bgr;
        gpu(rect).download(bgr);
        // Same luminance weights as the CPU path (qGray), so whether a borderline star reads
        // as visible does not depend on which path supplied the frame
        patch.create(size, size, CV_8UC1);
        for (int row = 0; row < size; ++row)
        {
            const cv::Vec3b *bgrLine = bgr.ptr<cv::Vec3b>(row);
            uchar *line = patch.ptr<uchar>(row);
            for (int col = 0; col < size; ++col) {
                line[col] = static_cast<uchar>(qGray(bgrLine[col][2], bgrLine[col][1], bgrLine[col][0]));
            }
        }
        return true;
    }
#endif

    return false;
}

// The catalog entries star sensing can actually use, cached across recomputes: everything
// at or brighter than the configured sensing magnitude, in catalog order (the index into
// the full catalog is preserved as the key the last-seen record uses).
const QVector<CameraPlateSolver::BrightStar>& CameraCloudDetector::sensedStarCatalog() const
{
    if (m_sensedStarCatalog.isEmpty() || (m_sensedStarCatalogMagnitude != m_settings.m_cloudStarSenseMagnitude))
    {
        const QVector<CameraPlateSolver::BrightStar> catalog = CameraPlateSolver::brightStarCatalog(m_settings);
        m_sensedStarCatalog.clear();
        m_sensedStarCatalog.reserve(catalog.size() / 8);
        for (const CameraPlateSolver::BrightStar& star : catalog)
        {
            if (star.magnitude <= m_settings.m_cloudStarSenseMagnitude) {
                m_sensedStarCatalog.append(star);
            }
        }
        m_sensedStarCatalogMagnitude = m_settings.m_cloudStarSenseMagnitude;
    }
    return m_sensedStarCatalog;
}

// Predicts where bright catalog stars should appear (camera position, observation time and
// lens model - the same inputs the sun/moon mask uses) and checks each position for a
// point-source peak in the full-resolution frame. Stars near the horizon, the sun or the
// moon, inside exclusion rectangles or off the frame are skipped. The result feeds
// applyStarVisibilityVeto() after classification.
CameraCloudDetector::CloudStarSense CameraCloudDetector::senseStarVisibility(const CameraPipelineFramePtr& frame, const QSize& imageSize) const
{
    CloudStarSense sense;
    if (!m_settings.m_cloudStarSense) {
        return sense;
    }

    const QDateTime observationTime = m_settings.m_plateSolveUseCaptureDateTime
        ? frame->m_captureDateTime
        : m_settings.m_plateSolveDateTime;
    if (!observationTime.isValid()) {
        return sense;
    }

    const SkyProjector projector = SkyProjector::create(m_settings, imageSize, frame->m_imageTransform);
    if (!projector.valid) {
        return sense;
    }

    PROFILER_START();

    AzAlt bodyAzAlt;
    RADec bodyRaDec;
    Astronomy::sunPosition(bodyAzAlt, bodyRaDec, m_settings.m_latitude, m_settings.m_longitude, observationTime);
    // No stars are detectable in daylight or early twilight; skip the patch sampling
    // entirely rather than measure a few hundred patches the night-only veto will ignore
    if (bodyAzAlt.alt >= sunDayElevation)
    {
        PROFILER_STOP(__FUNCTION__);
        return sense;
    }

    // Stars near the sun (twilight glow) or the moon (glare) are unreliable either way. The
    // avoidance radius is deliberately independent of the sun/moon mask setting: that value
    // caps the glare-blob removal and users tune it for exposure, which should not silently
    // change which stars the visibility check trusts.
    const SkyVector sunVector = skyVectorFromAltAz(bodyAzAlt.az, bodyAzAlt.alt);
    Astronomy::moonPosition(bodyAzAlt, bodyRaDec, m_settings.m_latitude, m_settings.m_longitude, observationTime);
    const SkyVector moonVector = skyVectorFromAltAz(bodyAzAlt.az, bodyAzAlt.alt);
    const double avoidCos = std::cos(skyDegToRad(starSenseAvoidBodyDeg));

    const QVector<CameraPlateSolver::BrightStar>& catalog = sensedStarCatalog();

    // On the GPU path the frame has no CPU image, and sampling each star's patch straight
    // from the device costs a synchronous download apiece - hundreds of round trips whose
    // overhead dwarfs the few hundred KB they move. One download of the whole frame is
    // cheaper as soon as more than a handful of stars are in play. The patches are still
    // converted with qGray weights on the CPU, so which borderline star reads as visible
    // does not depend on the path.
    cv::Mat fullResBgr;
#ifdef CAMERA_OPENCV_CUDA_CLOUD_DETECTION
    if (frame->m_image.isNull() && frame->hasCudaBgrImage()
        && (frame->m_cudaBgrImage.depth() == CV_8U) && (frame->m_cudaBgrImage.channels() == 3)
        && (catalog.size() > 24)) {
        frame->m_cudaBgrImage.download(fullResBgr);
    }
#endif

    for (int catalogIndex = 0; catalogIndex < catalog.size(); ++catalogIndex)
    {
        const CameraPlateSolver::BrightStar& star = catalog[catalogIndex];

        const RADec raDec{star.rightAscensionDegrees / 15.0, star.declinationDegrees}; // ra in hours
        const AzAlt azAlt = Astronomy::raDecToAzAlt(raDec, m_settings.m_latitude, m_settings.m_longitude, observationTime);
        if (azAlt.alt < starSenseMinElevation) {
            continue;
        }
        const SkyVector starVector = skyVectorFromAltAz(azAlt.az, azAlt.alt);
        if ((skyDot(starVector, sunVector) > avoidCos) || (skyDot(starVector, moonVector) > avoidCos)) {
            continue;
        }

        QPointF position;
        if (!projector.projectAltAz(azAlt.az, azAlt.alt, position)) {
            continue;
        }
        const QPoint centre(qRound(position.x()), qRound(position.y()));

        bool excluded = false;
        for (const QRect& rect : m_settings.m_motionExclusionRects)
        {
            if (rect.contains(centre))
            {
                excluded = true;
                break;
            }
        }
        if (excluded) {
            continue;
        }

        cv::Mat patch;
        if (!samplePatchGray(frame, centre, starSensePatchHalf, patch, fullResBgr)) {
            continue;
        }

        // Point-source test: the peak must stand above the patch background (median), and
        // the patch must be mostly background (90th percentile near the median) so a bright
        // extended structure - a moonlit cloud edge crossing the patch - does not fake a star
        std::vector<uchar> values(patch.begin<uchar>(), patch.end<uchar>());
        const size_t medianIndex = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + medianIndex, values.end());
        const int background = values[medianIndex];
        const size_t p90Index = (values.size() * 9) / 10;
        std::nth_element(values.begin(), values.begin() + p90Index, values.end());
        const int p90 = values[p90Index];
        double peak = 0.0;
        cv::minMaxLoc(patch, nullptr, &peak);

        const bool visible = ((peak - background) >= m_settings.m_starThreshold)
            && (2 * (p90 - background) < m_settings.m_starThreshold);
        sense.stars.append({position, visible, static_cast<float>(star.magnitude), catalogIndex});
    }

    sense.valid = true;
    PROFILER_STOP(__FUNCTION__);
    return sense;
}

// Removes cloud-mask components that predicted stars shine through: when a component holds
// enough expected stars and at least half of them are visible, whatever was flagged there is
// clear sky (or haze thin enough to see stars through - not cloud in any useful sense). A
// genuinely overcast component blocks its stars, so the veto abstains on it, and it abstains
// frame-wide in bright twilight when no stars are detectable anywhere.
void CameraCloudDetector::applyStarVisibilityVeto(cv::Mat& mask, const CloudStarSense& starSense, const cv::Rect& roi) const
{
    if (!starSense.valid || starSense.stars.isEmpty() || (roi.width <= 0) || (roi.height <= 0)) {
        return;
    }
    if (cv::countNonZero(mask) == 0) {
        return;
    }

    const double scaleX = static_cast<double>(mask.cols) / roi.width;
    const double scaleY = static_cast<double>(mask.rows) / roi.height;

    cv::Mat labels;
    const int labelCount = cv::connectedComponents(mask, labels, 8);
    if (labelCount <= 1) {
        return;
    }

    std::vector<int> expected(static_cast<size_t>(labelCount), 0);
    std::vector<int> visible(static_cast<size_t>(labelCount), 0);
    for (const CloudStarSense::Star& star : starSense.stars)
    {
        const int col = static_cast<int>(std::lround((star.position.x() - roi.x) * scaleX));
        const int row = static_cast<int>(std::lround((star.position.y() - roi.y) * scaleY));
        if ((col < 0) || (row < 0) || (col >= labels.cols) || (row >= labels.rows)) {
            continue;
        }
        const int label = labels.at<int>(row, col);
        if (label > 0)
        {
            ++expected[static_cast<size_t>(label)];
            if (star.visible) {
                ++visible[static_cast<size_t>(label)];
            }
        }
    }

    std::vector<uchar> veto(static_cast<size_t>(labelCount), 0);
    bool any = false;
    for (int label = 1; label < labelCount; ++label)
    {
        if (expected[static_cast<size_t>(label)] >= starSenseMinStars)
        {
            const double fraction = static_cast<double>(visible[static_cast<size_t>(label)])
                / static_cast<double>(expected[static_cast<size_t>(label)]);
            if (fraction >= starSenseVisibleFraction)
            {
                veto[static_cast<size_t>(label)] = 1;
                any = true;
            }
        }
    }
    if (!any) {
        return;
    }

    // Where the visible stars actually looked. The component vote decides WHETHER a region
    // may be vetoed; this decides HOW FAR that verdict reaches, so a component that happens
    // to stretch across the frame cannot be cleared wholesale on the strength of stars at
    // one end of it (see starVetoReachFraction).
    const int reach = std::max(4, cvRound(starVetoReachFraction * std::max(mask.cols, mask.rows)));
    cv::Mat vouched = cv::Mat::zeros(mask.size(), CV_8UC1);
    for (const CloudStarSense::Star& star : starSense.stars)
    {
        if (!star.visible) {
            continue;
        }
        const cv::Point centre(cvRound((star.position.x() - roi.x) * scaleX),
                               cvRound((star.position.y() - roi.y) * scaleY));
        cv::circle(vouched, centre, reach, cv::Scalar(255), cv::FILLED);
    }

    for (int row = 0; row < labels.rows; ++row)
    {
        const int *labelLine = labels.ptr<int>(row);
        const uchar *vouchedLine = vouched.ptr<uchar>(row);
        uchar *maskLine = mask.ptr<uchar>(row);
        for (int col = 0; col < labels.cols; ++col)
        {
            if ((labelLine[col] > 0) && veto[static_cast<size_t>(labelLine[col])] && vouchedLine[col]) {
                maskLine[col] = 0;
            }
        }
    }
}

// Records when each predicted star was last actually detected, feeding the star-blank
// cue's recently-seen requirement. Runs on every star-sensed recompute, day or night.
void CameraCloudDetector::recordStarVisibility(const CloudStarSense& starSense, const QDateTime& observationTime)
{
    if (!starSense.valid || !observationTime.isValid()) {
        return;
    }
    for (const CloudStarSense::Star& star : starSense.stars)
    {
        if (star.visible) {
            m_starLastVisible.insert(star.catalogIndex, observationTime);
        }
    }
}

// The positive counterpart of the visibility veto: a cluster of recently seen stars that
// all VANISH is blocked by cloud, however invisible that cloud is to the brightness and
// colour cues (thin cirrus, dark cloud). Only stars this camera demonstrably detects
// participate - on real frames most predicted stars fail detection even under a clear sky,
// so plain absence proves nothing. Fires only when the frame proves star detection still
// works (enough stars visible somewhere); isolated disappearances never fire (a variable
// star, a bad patch), and a visible star inside the neighbourhood vetoes it.
void CameraCloudDetector::applyStarBlankCue(cv::Mat& mask, const cv::Mat& evaluationMask, const CloudStarSense& starSense, const cv::Rect& roi, const QSize& imageSize, const QDateTime& observationTime) const
{
    if (!starSense.valid || !observationTime.isValid() || (roi.width <= 0) || (roi.height <= 0)) {
        return;
    }

    int visibleCount = 0;
    for (const CloudStarSense::Star& star : starSense.stars) {
        visibleCount += star.visible ? 1 : 0;
    }
    if (visibleCount < kStarBlankMinVisible) {
        return;
    }

    // A star is "vanished" when it was detected recently but is not detected now
    const auto vanished = [&](const CloudStarSense::Star& star) {
        if (star.visible) {
            return false;
        }
        const QDateTime lastSeen = m_starLastVisible.value(star.catalogIndex);
        return lastSeen.isValid() && (lastSeen.secsTo(observationTime) <= kStarBlankMemorySecs)
            && (lastSeen.secsTo(observationTime) >= 0);
    };

    const double neighbourRadius = kStarBlankNeighbourFraction * std::max(imageSize.width(), imageSize.height());
    const double neighbourRadiusSq = neighbourRadius * neighbourRadius;
    const double scaleX = static_cast<double>(mask.cols) / roi.width;
    const double scaleY = static_cast<double>(mask.rows) / roi.height;
    const int discRadius = std::max(4, cvRound(starClearRadiusFraction * std::max(mask.cols, mask.rows)));

    cv::Mat blank = cv::Mat::zeros(mask.size(), CV_8UC1);
    int candidates = 0;
    int painted = 0;
    for (int i = 0; i < starSense.stars.size(); ++i)
    {
        const CloudStarSense::Star& star = starSense.stars[i];
        if (!vanished(star)) {
            continue;
        }
        ++candidates;
        int vanishedNeighbours = 0;
        bool visibleNeighbour = false;
        for (int j = 0; j < starSense.stars.size(); ++j)
        {
            if (j == i) {
                continue;
            }
            const double dx = starSense.stars[j].position.x() - star.position.x();
            const double dy = starSense.stars[j].position.y() - star.position.y();
            if (dx * dx + dy * dy > neighbourRadiusSq) {
                continue;
            }
            if (starSense.stars[j].visible)
            {
                visibleNeighbour = true;
                break;
            }
            if (vanished(starSense.stars[j])) {
                ++vanishedNeighbours;
            }
        }
        if (visibleNeighbour || (vanishedNeighbours < kStarBlankMinNeighbours)) {
            continue;
        }
        const cv::Point centre(cvRound((star.position.x() - roi.x) * scaleX),
                               cvRound((star.position.y() - roi.y) * scaleY));
        if ((centre.x >= 0) && (centre.y >= 0) && (centre.x < mask.cols) && (centre.y < mask.rows))
        {
            cv::circle(blank, centre, discRadius, cv::Scalar(255), cv::FILLED);
            ++painted;
        }
    }
    if (painted > 0)
    {
        qDebug() << "CameraCloudDetector: star-blank cue:" << starSense.stars.size() << "stars," << visibleCount
                 << "visible," << candidates << "vanished," << painted << "blanked discs";
        cv::bitwise_and(blank, evaluationMask, blank);
        cv::bitwise_or(mask, blank, mask);
    }
}

// Produces the downscaled work images the classification runs on: BGR (for the day-path
// colour ratio), raw luminance and median-blurred luminance. The median blur erases stars,
// hot pixels and other point sources so they never register as cloud texture.
// The pipeline image is wrapped in its native format and colour conversion happens after the
// crop and downscale: resizing is channel-order agnostic, so converting the small work image
// avoids one or two full-resolution passes (format conversion plus channel swap) per recompute.
void CameraCloudDetector::prepareWorkImages(const QImage& image, const cv::Rect& roi, cv::Mat& workBgr, cv::Mat& rawGray, cv::Mat& gray) const
{
    PROFILER_START();

    cv::Mat wrapped;
    int toBgrCode = cv::COLOR_RGB2BGR;
    QImage convertedRgb;
    switch (image.format())
    {
    case QImage::Format_RGB32:
    case QImage::Format_ARGB32:
    case QImage::Format_ARGB32_Premultiplied:
        // Qt xRGB32 memory layout is B,G,R,A on little-endian
        wrapped = cv::Mat(image.height(), image.width(), CV_8UC4,
            const_cast<uchar*>(image.constBits()), static_cast<size_t>(image.bytesPerLine()));
        toBgrCode = cv::COLOR_BGRA2BGR;
        break;
    case QImage::Format_RGB888:
        wrapped = wrapRgb888Image(image);
        toBgrCode = cv::COLOR_RGB2BGR;
        break;
    default:
        // Uncommon formats fall back to a full conversion
        wrapped = wrapRgb888Image(ensureRgb888(image, convertedRgb));
        toBgrCode = cv::COLOR_RGB2BGR;
        break;
    }

    cv::Mat cropped = wrapped(roi);
    const double downscale = m_settings.m_cloudDownscale;
    cv::Mat scaled = cropped;
    if (downscale < 0.999)
    {
        const cv::Size downscaledSize(
            std::max(1, static_cast<int>(std::lround(roi.width * downscale))),
            std::max(1, static_cast<int>(std::lround(roi.height * downscale))));
        cv::resize(cropped, scaled, downscaledSize, 0.0, 0.0, cv::INTER_AREA);
    }

    cv::cvtColor(scaled, workBgr, toBgrCode);
    cv::cvtColor(workBgr, rawGray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(rawGray, gray, 5);

    PROFILER_STOP(__FUNCTION__);
}

#ifdef CAMERA_OPENCV_CUDA_CLOUD_DETECTION
bool CameraCloudDetector::canUseCudaCloudDetection() const
{
    static bool warnedNoDevice = false;

    if (!m_settings.m_postProcessUseCuda) {
        return false;
    }

    if (cv::cuda::getCudaEnabledDeviceCount() <= 0)
    {
        if (!warnedNoDevice)
        {
            qWarning() << "CameraCloudDetector: CUDA cloud detection requested, but no CUDA-enabled OpenCV device is available";
            warnedNoDevice = true;
        }
        return false;
    }

    return true;
}

// GPU variant of prepareWorkImages(): the full-resolution BGR image stays on the GPU and
// only the downscaled work images are downloaded
bool CameraCloudDetector::prepareWorkImagesCuda(const cv::cuda::GpuMat& bgrGpu, const cv::Rect& roi, cv::Mat& workBgr, cv::Mat& rawGray, cv::Mat& gray)
{
    PROFILER_START();

    try
    {
        cv::cuda::GpuMat inputGpu = bgrGpu(roi);
        const double downscale = m_settings.m_cloudDownscale;
        if (downscale < 0.999)
        {
            const cv::Size downscaledSize(
                std::max(1, static_cast<int>(std::lround(roi.width * downscale))),
                std::max(1, static_cast<int>(std::lround(roi.height * downscale))));
            cv::cuda::GpuMat downscaledGpu;
            cv::cuda::resize(inputGpu, downscaledGpu, downscaledSize, 0.0, 0.0, cv::INTER_AREA, m_cudaCloudStream);
            inputGpu = downscaledGpu;
        }

        cv::cuda::GpuMat rawGrayGpu;
        cv::cuda::cvtColor(inputGpu, rawGrayGpu, cv::COLOR_BGR2GRAY, 0, m_cudaCloudStream);

        if (!m_cudaCloudMedianFilter) {
            m_cudaCloudMedianFilter = cv::cuda::createMedianFilter(CV_8UC1, 5);
        }
        cv::cuda::GpuMat grayGpu;
        m_cudaCloudMedianFilter->apply(rawGrayGpu, grayGpu, m_cudaCloudStream);

        inputGpu.download(workBgr, m_cudaCloudStream);
        rawGrayGpu.download(rawGray, m_cudaCloudStream);
        grayGpu.download(gray, m_cudaCloudStream);
        m_cudaCloudStream.waitForCompletion();

        PROFILER_STOP(__FUNCTION__);
        return true;
    }
    catch (const cv::Exception& error)
    {
        qWarning() << "CameraCloudDetector: CUDA cloud preprocessing failed; falling back to CPU:" << error.what();
    }

    PROFILER_STOP(__FUNCTION__);
    return false;
}
#endif

// Local-contrast structure detector, shared by the dark night path (as its sole cue) and the
// moonlit path (as a colour-independent extra cue), in two modes selected by `bandpass`.
//
// Dark mode (bandpass=false): fits a smooth low-order surface to the sky - absorbing horizon
// glow, light pollution and vignetting - and flags where the local sky average stands above
// it. Right for dark unimodal skies, where dim veils only clear the threshold in the wide
// average and any deviation from the global trend is cloud.
//
// Bandpass mode (bandpass=true): compares a small-radius sky average against the wide-radius
// one, flagging brightness structure at cloud-lump scale. Right for bright moonlit/twilight
// skies: those are bimodal (bright cloud and dark clear sky in one frame), where a global
// surface lands in between and would flag any smooth bright clear region wholesale. Lumpy
// cloud has band-pass signal; a smooth region - however bright - matches its own surround
// and reads zero.
//
// Returns the thresholded CV_8U mask. When debugMask is given (dark mode), the Background and
// Signal debug views are filled from the fitted surface and the positive contrast.
cv::Mat CameraCloudDetector::structureContrastMask(const cv::Mat& gray, const cv::Mat& evaluationMask, bool bandpass, cv::Mat* debugMask) const
{
    cv::Mat skyMask8;
    cv::bitwise_and(gray >= darkSkyFloor, evaluationMask, skyMask8);
    cv::Mat sky;
    skyMask8.convertTo(sky, CV_32F, 1.0 / 255.0);
    cv::Mat grayWeighted;
    gray.convertTo(grayWeighted, CV_32F);
    grayWeighted = grayWeighted.mul(sky);

    // Local sky level, normalized over sky pixels only so the dark border/foreground
    // cannot bleed in
    const auto normalizedBox = [&](int radius, cv::Mat& average, cv::Mat& density)
    {
        const cv::Size kernel(2 * radius + 1, 2 * radius + 1);
        cv::Mat sum;
        cv::boxFilter(grayWeighted, sum, -1, kernel);
        cv::boxFilter(sky, density, -1, kernel);
        cv::Mat safeDensity;
        cv::max(density, 1e-3, safeDensity);
        cv::divide(sum, safeDensity, average);
    };
    cv::Mat backgroundLocal, densityLocal;
    normalizedBox(m_settings.m_cloudBackgroundBlur, backgroundLocal, densityLocal);

    cv::Mat contrast;
    if (bandpass)
    {
        // A narrow band keyed just under the surround radius: wider bands were tried and
        // false-flag the broad glow bumps of clear twilight sky, which look like cloud
        // lumps at large scale. The narrow band only fires densely on genuinely lumpy
        // cloud; the caller closes the sparse detections into solid regions.
        cv::Mat detailLocal, densityDetail;
        normalizedBox(std::max(2, m_settings.m_cloudBackgroundBlur / 4), detailLocal, densityDetail);
        contrast = detailLocal - backgroundLocal;
    }
    else
    {
        // Bound the fitted surface to the observed sky brightness range so a cubic edge
        // overshoot cannot invent false cloud in the corners
        const MaskedHistogram skyRange = maskedHistogram(gray, skyMask8);
        const double skyFloor = skyRange.percentile(0.02);
        // The ceiling must sit at the very top of the sky range: clamping lower parks the
        // surface beneath the brightest horizon/rim glow, and the uncovered glow then reads
        // as false cloud (an order-3 surface cannot ride up into patchy cloud regardless)
        const double skyCeil = std::max(skyFloor + 1.0, static_cast<double>(skyRange.percentile(0.995)));
        const cv::Mat surface = polynomialBackground(gray, skyMask8, skyFloor, skyCeil);
        if (debugMask && (m_settings.m_cloudDebugView == CameraSettings::CloudDebugViewBackground)) {
            surface.convertTo(*debugMask, CV_8U);
        }
        contrast = backgroundLocal - surface;
    }

    if (debugMask && (m_settings.m_cloudDebugView == CameraSettings::CloudDebugViewSignal))
    {
        cv::Mat positiveContrast;
        cv::max(contrast, 0.0, positiveContrast);
        positiveContrast.convertTo(*debugMask, CV_8U);
    }

    // Only positive contrast is cloud: clear sky sits at or below the reference
    cv::Mat mask = contrast > static_cast<float>(m_settings.m_cloudNightThreshold);
    cv::bitwise_and(mask, skyMask8, mask);
    // Ignore rim pixels whose local window saw too little sky to be reliable: near foliage
    // and the image-circle edge the window average rests on a handful of pixels and the
    // comparison against the reference turns into noise. The band-pass mode is far
    // stricter: the bright vignette/glow arc hugging the image-circle rim reads as a
    // textbook band-pass lump, so only windows seeing almost pure sky may fire - cloud
    // banks are frame-interior and unaffected.
    const cv::Mat stableMask = densityLocal > (bandpass ? 0.85f : 0.5f);
    cv::bitwise_and(mask, stableMask, mask);
    return mask;
}

// Flags day cloud that is whiter than the clear sky AT ITS OWN ELEVATION. Clear sky is not
// one colour across an all-sky frame: it whitens steadily toward the horizon as the line of
// sight crosses more atmosphere, so a single global red/blue threshold has to be set high
// enough for the pale horizon and then misses the grey cloud higher up - which is why a
// broken cumulus sky can read as almost clear. The expected clear-sky ratio is measured
// instead as a robust radial profile (a low percentile per ring of constant radius from the
// sky centre) and cloud is what stands a margin above the profile at its own radius. The low
// percentile keeps the profile on the clear sky even when much of a ring is clouded; when a
// ring is entirely clouded the profile follows the cloud and that ring simply contributes
// nothing, which is the safe failure - the absolute threshold and the overcast structure
// vote still cover the fully overcast case.
cv::Mat CameraCloudDetector::dayRelativeCloudMask(const cv::Mat& red, const cv::Mat& blue, const cv::Mat& evaluationMask,
                                                  const cv::Mat& sampleMask, double margin)
{
    cv::Mat flagged = cv::Mat::zeros(evaluationMask.size(), CV_8UC1);
    if (margin <= 0.0) {
        return flagged;
    }

    const cv::Moments moments = cv::moments(evaluationMask, true);
    if (moments.m00 <= 0.0) {
        return flagged;
    }
    const double centreX = moments.m10 / moments.m00;
    const double centreY = moments.m01 / moments.m00;

    // Radius of each evaluated pixel, and the largest of them, so the rings span the sky.
    // Only evaluated pixels are ever read back, so the rest are not worth a square root.
    cv::Mat radius(evaluationMask.size(), CV_32F, cv::Scalar(0.0f));
    double maxRadius = 0.0;
    for (int row = 0; row < evaluationMask.rows; ++row)
    {
        const uchar *maskLine = evaluationMask.ptr<uchar>(row);
        float *radiusLine = radius.ptr<float>(row);
        const double dy = row - centreY;
        for (int col = 0; col < evaluationMask.cols; ++col)
        {
            if (!maskLine[col]) {
                continue;
            }
            const double dx = col - centreX;
            const double r = std::sqrt(dx * dx + dy * dy);
            radiusLine[col] = static_cast<float>(r);
            if (r > maxRadius) {
                maxRadius = r;
            }
        }
    }
    if (maxRadius <= 0.0) {
        return flagged;
    }

    // Gather ring samples from the sample mask, not from everything evaluated: a pixel too
    // dark to classify has no meaningful colour either (red and blue are both near zero, so
    // their ratio is noise), and a handful of such pixels leaking in at the rim is enough to
    // wreck the profile - see the monotonic step below, which propagates the outermost value
    // inward. The evaluated region still decides where the result may be flagged.
    //
    // The samples are spatially subsampled, as the whole-frame ratio percentile is: a
    // percentile over a few hundred samples is close enough to one over every pixel, at a
    // fraction of the copying and sorting - and this profile runs on both the day and the
    // night path now. The step is set by the SMALLEST ring, not by the frame: robustness
    // here is a per-ring property, and the innermost ring holds a 576th of the sky's area.
    // On a work image small enough that even that ring is thinly covered the step is 1 and
    // nothing is dropped; a full-resolution all-sky frame subsamples heavily.
    const double innermostRadius = maxRadius / kDayProfileRings;
    const int step = std::max(1, static_cast<int>(std::sqrt(
        CV_PI * innermostRadius * innermostRadius / kDayProfileRingSamples)));
    std::vector<std::vector<float>> ringSamples(kDayProfileRings);
    const double ringScale = kDayProfileRings / maxRadius;
    for (int row = 0; row < evaluationMask.rows; row += step)
    {
        const uchar *maskLine = sampleMask.ptr<uchar>(row);
        const float *radiusLine = radius.ptr<float>(row);
        const float *redLine = red.ptr<float>(row);
        const float *blueLine = blue.ptr<float>(row);
        for (int col = 0; col < evaluationMask.cols; col += step)
        {
            if (!maskLine[col]) {
                continue;
            }
            const int ring = std::clamp(static_cast<int>(radiusLine[col] * ringScale), 0, kDayProfileRings - 1);
            ringSamples[static_cast<size_t>(ring)].push_back(redLine[col] / (blueLine[col] + 1.0f));
        }
    }

    // Clear-sky ratio per ring, with empty rings carried from their neighbours
    std::vector<float> profile(kDayProfileRings, 0.0f);
    std::vector<uchar> haveRing(kDayProfileRings, 0);
    for (int ring = 0; ring < kDayProfileRings; ++ring)
    {
        std::vector<float>& samples = ringSamples[static_cast<size_t>(ring)];
        if (samples.size() < 50) {
            continue;
        }
        const size_t index = static_cast<size_t>(kDayProfilePercentile * (samples.size() - 1));
        std::nth_element(samples.begin(), samples.begin() + index, samples.end());
        profile[static_cast<size_t>(ring)] = samples[index];
        haveRing[static_cast<size_t>(ring)] = 1;
    }
    int filled = 0;
    for (int ring = 0; ring < kDayProfileRings; ++ring) {
        filled += haveRing[static_cast<size_t>(ring)] ? 1 : 0;
    }
    if (filled < 2) {
        return flagged;
    }
    for (int ring = 0; ring < kDayProfileRings; ++ring)
    {
        if (haveRing[static_cast<size_t>(ring)]) {
            continue;
        }
        int nearest = -1;
        for (int other = 0; other < kDayProfileRings; ++other)
        {
            if (haveRing[static_cast<size_t>(other)]
                && ((nearest < 0) || (std::abs(other - ring) < std::abs(nearest - ring)))) {
                nearest = other;
            }
        }
        profile[static_cast<size_t>(ring)] = profile[static_cast<size_t>(nearest)];
    }

    // Clear sky cannot be whiter overhead than it is at the horizon: the ratio rises toward
    // the horizon because the line of sight crosses more atmosphere, so the profile must not
    // decrease outward. Where it does, that ring holds too little clear sky to measure - its
    // percentile has landed on the cloud filling it - so carry the smallest value found
    // further out inward over it. This is what lets a bank of cloud sitting over the zenith
    // be seen at all: without it the profile follows that cloud and the cloud is compared
    // against itself, which is exactly how an overhead sheet with clear sky only around the
    // edges escapes detection.
    for (int ring = kDayProfileRings - 2; ring >= 0; --ring)
    {
        profile[static_cast<size_t>(ring)] = std::min(profile[static_cast<size_t>(ring)],
                                                      profile[static_cast<size_t>(ring + 1)]);
    }

    // Smooth the profile so ring boundaries do not print themselves onto the mask
    std::vector<float> smoothed(profile);
    for (int ring = 0; ring < kDayProfileRings; ++ring)
    {
        float sum = 0.0f;
        int count = 0;
        for (int offset = -2; offset <= 2; ++offset)
        {
            const int other = std::clamp(ring + offset, 0, kDayProfileRings - 1);
            sum += profile[static_cast<size_t>(other)];
            ++count;
        }
        smoothed[static_cast<size_t>(ring)] = sum / std::max(1, count);
    }

    for (int row = 0; row < evaluationMask.rows; ++row)
    {
        const uchar *maskLine = evaluationMask.ptr<uchar>(row);
        const float *radiusLine = radius.ptr<float>(row);
        const float *redLine = red.ptr<float>(row);
        const float *blueLine = blue.ptr<float>(row);
        uchar *flaggedLine = flagged.ptr<uchar>(row);
        for (int col = 0; col < evaluationMask.cols; ++col)
        {
            if (!maskLine[col]) {
                continue;
            }
            const int ring = std::clamp(static_cast<int>(radiusLine[col] * ringScale), 0, kDayProfileRings - 1);
            const float ratio = redLine[col] / (blueLine[col] + 1.0f);
            if (ratio - smoothed[static_cast<size_t>(ring)] >= static_cast<float>(margin)) {
                flaggedLine[col] = 255;
            }
        }
    }
    return flagged;
}

// Structure (band-pass local-contrast) vote, shared by the moonlit-night and day paths: on a
// pale overcast sky the colour anchor sits inside the palest cloud, so bluish-white cloud that
// is itself the bluest thing in frame reads as clear. Such banks are lumpy where clear sky is
// smooth, a separation colour cannot express. skyMedian sets the brightness floor below which
// textured foreground cannot vote.
//
// Each connected unflagged sky region is judged as a whole and flipped to cloud when its
// interior is lumpy; a clear gap between clouds has a smooth interior and survives - the
// separation is an order of magnitude (4-8 % detection density inside pastel overcast against
// under 0.5 % in genuinely clear regions), and that measurement is what protects clear sky
// here, on either path.
//
// The day caller adds a gate on top of it, and the moonlit caller deliberately does not. By
// day the frame's own bluest sky answers a question the density cannot: whether any genuinely
// blue sky remains at all. Where it does, the region flip is powerful enough to swallow the
// blue gaps of a partly cloudy sky, so it is not run. At night there is no equivalent test -
// the colour anchor moves with gain and white balance and says nothing about whether clear sky
// remains - so the lumpiness threshold stands alone, as it was measured to.
void CameraCloudDetector::applyStructureVote(cv::Mat& mask, const cv::Mat& gray, const cv::Mat& evaluationMask, int skyMedian) const
{
    // Structure cue: on a fully overcast pastel sky the colour anchor inevitably sits
    // inside the palest cloud, so bluish-white cloud within the margin of it reads as
    // clear - colour cannot flag cloud that is itself the bluest thing in frame. But
    // such cloud banks are lumpy where clear sky is smooth: a band-pass local contrast
    // fires on 10-25% of the pixels inside a cloud bank versus under ~3% of genuinely
    // clear sky (measured), a separation that exists at region level where no per-pixel
    // threshold works. So judge each connected unflagged sky region as a whole: erode
    // away its boundary strip (band-pass bleed from adjacent colour-detected cloud
    // edges lives there), measure the detection density over the interior, and flip
    // the entire region to cloud when its interior is lumpy. A clear gap between
    // clouds - whatever its size - has a smooth interior and survives.
    cv::Mat structureMask = structureContrastMask(gray, evaluationMask, true, nullptr);
    // Moonlit/twilight cloud is bright; dim textured foreground (roofs, aerials, trees
    // above the dark floor) also carries band-pass structure but must not vote
    const int structureBrightness = static_cast<int>(std::lround(moonlitStructureBrightness * skyMedian));
    cv::bitwise_and(structureMask, gray >= structureBrightness, structureMask);
    // Nor may the bright vignette/glow arc that hugs the image-circle rim: it is a
    // textbook band-pass lump, but unlike a cloud bank it borders near-dark pixels
    // (the dim fisheye surround), so require clearance from anything near-dark
    cv::Mat notDark;
    cv::compare(gray, 2 * darkSkyFloor, notDark, cv::CMP_GE);
    cv::Mat distToDark;
    cv::distanceTransform(notDark, distToDark, cv::DIST_L2, 5);
    cv::Mat darkClearance = distToDark >= static_cast<float>(m_settings.m_cloudBackgroundBlur);
    cv::bitwise_and(structureMask, darkClearance, structureMask);
    cv::Mat skyMask8;
    cv::bitwise_and(gray >= darkSkyFloor, evaluationMask, skyMask8);
    cv::Mat notCloud;
    cv::bitwise_and(skyMask8, ~mask, notCloud);

    // Only detections in the interior of the unflagged area vote: the strip along the
    // colour-detected cloud boundary carries band-pass bleed from the cloud side, and
    // detections inside already-flagged cloud say nothing about the unflagged sky (and
    // counting them would fill in every clear gap between clouds).
    const int boundaryStrip = std::max(2, m_settings.m_cloudBackgroundBlur / 4) + 2;
    const cv::Mat erodeKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * boundaryStrip + 1, 2 * boundaryStrip + 1));
    cv::Mat interior;
    cv::erode(notCloud, interior, erodeKernel, cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::Mat interiorSpecks;
    cv::bitwise_and(structureMask, interior, interiorSpecks);

    cv::Mat speckF, interiorF;
    interiorSpecks.convertTo(speckF, CV_32F, 1.0 / 255.0);
    interior.convertTo(interiorF, CV_32F, 1.0 / 255.0);
    const cv::Size voteKernel(2 * m_settings.m_cloudBackgroundBlur + 1, 2 * m_settings.m_cloudBackgroundBlur + 1);
    cv::Mat speckSum, interiorSum;
    cv::boxFilter(speckF, speckSum, -1, voteKernel);
    cv::boxFilter(interiorF, interiorSum, -1, voteKernel);
    cv::Mat safeInteriorSum, speckDensity;
    cv::max(interiorSum, 0.05, safeInteriorSum);
    cv::divide(speckSum, safeInteriorSum, speckDensity);

    cv::Mat fill = speckDensity > moonlitStructureFillDensity;
    cv::bitwise_and(fill, notCloud, fill);
    cv::bitwise_or(mask, fill, mask);

    // Region vote: the windowed fill cannot reach unflagged cloud near the frame edge,
    // where the dark-clearance gate silences the specks. But such cloud is connected to
    // the lumpy interior it surrounds, so judge each connected unflagged region as a
    // whole too: gated speck densities measure 4-8% inside pastel overcast regions and
    // under 0.5% in genuinely clear ones (dark starfields and blue gaps measure zero),
    // so a uniformly lumpy region flips wholesale, edge to edge.
    cv::Mat labels;
    const int labelCount = cv::connectedComponents(notCloud, labels, 8);
    if (labelCount > 1)
    {
        std::vector<int> interiorArea(static_cast<size_t>(labelCount), 0);
        std::vector<int> interiorHits(static_cast<size_t>(labelCount), 0);
        for (int row = 0; row < labels.rows; ++row)
        {
            const int *labelLine = labels.ptr<int>(row);
            const uchar *interiorLine = interior.ptr<uchar>(row);
            const uchar *speckLine = interiorSpecks.ptr<uchar>(row);
            for (int col = 0; col < labels.cols; ++col)
            {
                if ((labelLine[col] > 0) && interiorLine[col])
                {
                    ++interiorArea[static_cast<size_t>(labelLine[col])];
                    if (speckLine[col]) {
                        ++interiorHits[static_cast<size_t>(labelLine[col])];
                    }
                }
            }
        }

        std::vector<uchar> lumpy(static_cast<size_t>(labelCount), 0);
        for (int label = 1; label < labelCount; ++label)
        {
            // Regions with next to no interior are boundary slivers; leave them alone
            if (interiorArea[static_cast<size_t>(label)] >= 50)
            {
                const float density = static_cast<float>(interiorHits[static_cast<size_t>(label)])
                    / static_cast<float>(interiorArea[static_cast<size_t>(label)]);
                lumpy[static_cast<size_t>(label)] = density > moonlitStructureRegionDensity;
            }
        }

        for (int row = 0; row < labels.rows; ++row)
        {
            const int *labelLine = labels.ptr<int>(row);
            uchar *maskLine = mask.ptr<uchar>(row);
            for (int col = 0; col < labels.cols; ++col)
            {
                if ((labelLine[col] > 0) && lumpy[static_cast<size_t>(labelLine[col])]) {
                    maskLine[col] = 255;
                }
            }
        }
    }
}

void CameraCloudDetector::applyCloudDetection(const cv::Mat& workBgr, const cv::Mat& rawGray, const cv::Mat& gray, const cv::Rect& roi, const cv::Rect& contentRect, const QSize& imageSize, const CameraPipelineImageTransform& imageTransform, const QDateTime& captureDateTime, const CloudStarSense& starSense, CameraPipelineCloud& cloud, cv::Mat* debugMask)
{
    PROFILER_START();

    // Local fine-scale texture energy: what the median blur removed, averaged over a
    // neighbourhood. Cloud is smooth at the working resolution, while roofs, trees and
    // buildings retain dense fine detail, so this separates grey/white man-made surfaces
    // from cloud where the red/blue ratio alone cannot.
    cv::Mat textureEnergy;
    {
        cv::Mat detail;
        cv::absdiff(rawGray, gray, detail);
        cv::boxFilter(detail, textureEnergy, -1, cv::Size(11, 11));
    }
    if (debugMask && (m_settings.m_cloudDebugView == CameraSettings::CloudDebugViewTexture)) {
        *debugMask = textureEnergy.clone();
    }

    // Pixels considered sky: not in an exclusion rectangle, and inside the real image
    // content when output scaling has padded the frame with borders
    const cv::Mat& exclusionMask = cachedExclusionMask(roi, gray.size());
    // Deep copy: the rim-margin, learned-foreground and sun/moon masks all mutate the
    // evaluation mask in place, and a shallow share would burn those (per-frame, moving)
    // exclusions into the cached exclusion mask carried to later frames
    cv::Mat evaluationMask = exclusionMask.clone();
    const cv::Rect contentInRoi = contentRect & roi;
    if (contentInRoi != roi)
    {
        cv::Mat contentMask = cv::Mat::zeros(gray.size(), CV_8UC1);
        if (contentInRoi.area() > 0)
        {
            const double scaleX = static_cast<double>(gray.cols) / roi.width;
            const double scaleY = static_cast<double>(gray.rows) / roi.height;
            const int x0 = std::max(0, static_cast<int>(std::floor((contentInRoi.x - roi.x) * scaleX)));
            const int y0 = std::max(0, static_cast<int>(std::floor((contentInRoi.y - roi.y) * scaleY)));
            const int x1 = std::min(gray.cols, static_cast<int>(std::ceil((contentInRoi.x + contentInRoi.width - roi.x) * scaleX)));
            const int y1 = std::min(gray.rows, static_cast<int>(std::ceil((contentInRoi.y + contentInRoi.height - roi.y) * scaleY)));
            if ((x1 > x0) && (y1 > y0)) {
                contentMask(cv::Rect(x0, y0, x1 - x0, y1 - y0)).setTo(255);
            }
        }
        cv::Mat combinedMask;
        cv::bitwise_and(exclusionMask, contentMask, combinedMask);
        evaluationMask = combinedMask;
    }

    // Clear-sky reference: resolve the sky-state slot for this frame (used by the save
    // request, the learned foreground exclusion, the deviation cue/veto and auto-learning)
    const QDateTime observationTime = m_settings.m_plateSolveUseCaptureDateTime
        ? captureDateTime
        : m_settings.m_plateSolveDateTime;
    int referenceSlot = -1;
    double referenceSunElevation = 0.0;
    double referenceMoonElevation = 0.0;
    if ((m_settings.m_cloudUseReference || m_saveReferencePending) && observationTime.isValid())
    {
        AzAlt sunAzAlt, moonAzAlt;
        RADec bodyRaDec;
        Astronomy::sunPosition(sunAzAlt, bodyRaDec, m_settings.m_latitude, m_settings.m_longitude, observationTime);
        Astronomy::moonPosition(moonAzAlt, bodyRaDec, m_settings.m_latitude, m_settings.m_longitude, observationTime);
        referenceSunElevation = sunAzAlt.alt;
        referenceMoonElevation = moonAzAlt.alt;
        referenceSlot = CameraClearSkyReference::slotFor(sunAzAlt.alt, moonAzAlt.alt);
        m_clearSkyReference.ensureLoaded(referenceStorageKey(m_settings));
    }
    const QRectF roiNorm(
        static_cast<double>(roi.x) / std::max(1, imageSize.width()),
        static_cast<double>(roi.y) / std::max(1, imageSize.height()),
        static_cast<double>(roi.width) / std::max(1, imageSize.width()),
        static_cast<double>(roi.height) / std::max(1, imageSize.height()));

    // Learned foreground (trees, roofs, window frames) derived from the clear-sky
    // reference: neither clear sky nor cloud, so excluded from classification and from
    // the coverage denominator, like a hands-free exclusion rectangle
    if (m_settings.m_cloudUseReference && (referenceSlot >= 0))
    {
        const cv::Mat foreground = m_clearSkyReference.foregroundMask(gray.size(), roiNorm);
        if (!foreground.empty())
        {
            cv::Mat notForeground;
            cv::bitwise_not(foreground, notForeground);
            cv::bitwise_and(evaluationMask, notForeground, evaluationMask);
        }
    }

    // Optionally exclude a margin inward from the illuminated sky-region boundary. On
    // fisheye all-sky lenses the image circle is ringed by a vignetted rim, lens flare
    // and foreground obstructions that are neither clear sky nor cloud; eroding the
    // bright sky region inward removes that band from classification and from the
    // coverage denominator. The band is measured relative to the shorter frame edge.
    if (m_settings.m_cloudEdgeMarginPercent > 0.0)
    {
        const int minDim = std::min(gray.cols, gray.rows);
        const int marginPx = static_cast<int>(std::lround(m_settings.m_cloudEdgeMarginPercent / 100.0 * minDim));
        if (marginPx > 0)
        {
            cv::Mat illuminated;
            cv::bitwise_and(evaluationMask, gray >= darkSkyFloor, illuminated);
            // Zero the outermost pixel ring so the margin also bites in from the image edge,
            // not only from the interior boundary of the sky region
            cv::rectangle(illuminated, cv::Rect(0, 0, illuminated.cols, illuminated.rows), cv::Scalar(0), 1);
            // Erode via a distance transform: O(n) whatever the margin, where a structuring-
            // element erosion grows with the kernel area and stalls at large margins
            cv::Mat boundaryDistance;
            cv::distanceTransform(illuminated, boundaryDistance, cv::DIST_L2, 5);
            cv::Mat inner = boundaryDistance > static_cast<float>(marginPx);
            cv::bitwise_and(evaluationMask, inner, evaluationMask);
        }
    }

    // Optional sky-elevation floor: exclude sky below the configured elevation so coverage
    // tracks the usable observing sky rather than being dominated by the horizon band
    if (m_settings.m_cloudMinElevation > 0.0) {
        applyMinElevationMask(evaluationMask, roi, imageSize, imageTransform);
    }

    // Save-reference request: capture the evaluated sky as this sky state's clear
    // reference. Requires an observation time (to know the sky state); reports back so the
    // GUI can show the store status. The surround exclusion below deliberately runs after
    // this: the learned-foreground silhouettes are found by looking for dark regions in the
    // reference, so a reference that never recorded the dark band cannot teach anything
    // about it. The anchors are made independent of how much of that dark region is in the
    // frame instead - see buildMaps().
    if (m_saveReferencePending)
    {
        m_saveReferencePending = false;
        if (referenceSlot >= 0)
        {
            m_clearSkyReference.ensureLoaded(referenceStorageKey(m_settings));
            m_clearSkyReference.capture(referenceSlot, gray, workBgr, textureEnergy, evaluationMask, roiNorm, observationTime,
                                        referenceSunElevation, referenceMoonElevation);
            if (m_msgQueueToFeature) {
                m_msgQueueToFeature->push(MsgReportClearSkyReference::create(
                    QStringLiteral("Saved %1 - %2").arg(CameraClearSkyReference::slotName(referenceSlot),
                        m_clearSkyReference.statusSummary(referenceSlot))));
            }
        }
        else if (m_msgQueueToFeature)
        {
            m_msgQueueToFeature->push(MsgReportClearSkyReference::create(
                QStringLiteral("Cannot save reference: no observation time (set position and capture time)")));
        }
    }

    // The dark fisheye/letterbox surround is not sky: remove it before the sky-median and
    // day/night decision, which it otherwise biases, and before coverage is measured
    excludeSurround(evaluationMask, gray);

    const bool night = resolveNightMode(gray, evaluationMask, captureDateTime);

    cv::Mat mask;
    const MaskedHistogram skyLevels = maskedHistogram(gray, evaluationMask);
    const int nightSkyMedian = night ? skyLevels.percentile(0.5) : 0;
    const int nightSkyBrightQuartile = night ? skyLevels.percentile(moonlitSkyFraction) : 0;
    if (night && (nightSkyBrightQuartile >= moonlitBrightness))
    {
        // A bright night sky (moonlit, or high gain and long exposure) is Rayleigh-scattered
        // light and behaves like dim daylight: clear sky is blue, cloud is white/pink. The
        // luminance-deviation approach fails here because moonlight and vignette span a wide
        // brightness range, so classify by red/blue ratio instead - but neither the colour nor
        // the brightness of a clear night sky is one number across an all-sky frame, and a
        // single anchored threshold for either flags the whole outer sky as cloud.
        std::vector<cv::Mat> channels;
        cv::split(workBgr, channels);
        cv::Mat blue, red;
        channels[0].convertTo(blue, CV_32F);
        channels[2].convertTo(red, CV_32F);

        // Sky, for everything below that must not look outside it
        cv::Mat skyMask8;
        cv::bitwise_and(gray >= darkSkyFloor, evaluationMask, skyMask8);

        // Colour first has to be measurable at all: see moonlitColourBlurFraction. The
        // average is taken over sky pixels only - red and blue are both near zero in the
        // unlit surround and in roof and tree silhouettes, and a plain box blur would drag
        // that into the sky's own colour within half a kernel of every such boundary. That
        // is precisely the rim where the clear-sky profile's outermost rings are measured,
        // and where a wrong value does the most damage. Same normalisation as the local sky
        // level in structureContrastMask.
        const int colourBlur = std::max(3, static_cast<int>(std::min(gray.rows, gray.cols) * moonlitColourBlurFraction) | 1);
        {
            const cv::Size kernel(colourBlur, colourBlur);
            cv::Mat weight;
            skyMask8.convertTo(weight, CV_32F, 1.0 / 255.0);
            cv::Mat density;
            cv::blur(weight, density, kernel);
            cv::max(density, 1e-3, density);
            cv::blur(blue.mul(weight), blue, kernel);
            cv::blur(red.mul(weight), red, kernel);
            cv::divide(blue, density, blue);
            cv::divide(red, density, red);
        }

        const int brightnessFloor = std::max(dayMinBrightness, nightSkyMedian / 2);
        const cv::Mat brightMask = gray >= brightnessFloor;

        if (debugMask && (m_settings.m_cloudDebugView == CameraSettings::CloudDebugViewBackground)) {
            *debugMask = gray.clone();
        }
        if (debugMask && (m_settings.m_cloudDebugView == CameraSettings::CloudDebugViewSignal))
        {
            // Ratio image scaled so a ratio of 1.0 maps to mid-grey
            cv::Mat ratio = red / (blue + 1.0f);
            ratio.convertTo(*debugMask, CV_8U, 128.0);
        }

        // Light pollution and airmass warm the clear sky steadily toward the horizon: one
        // all-sky frame measured 0.39 overhead rising to 0.69 at the rim, so the bluest-sky
        // anchor lands on the zenith and everything from mid-elevation outward reads as cloud.
        // Judge the colour against the clear sky at the same elevation, as the day path does.
        cv::Mat skyMask;
        cv::bitwise_and(brightMask, evaluationMask, skyMask);
        cv::Mat ratioMask = dayRelativeCloudMask(red, blue, evaluationMask, skyMask, moonlitRatioMargin);

        // A relative test alone cannot see a sky that is cloud everywhere: the profile follows
        // the cloud and the cloud is compared against itself. Keep the day threshold as an
        // absolute backstop - nothing that white is ever clear sky, at any elevation.
        cv::Mat absoluteMask;
        cv::compare(red, (blue + 1.0f) * m_settings.m_cloudDayThreshold, absoluteMask, cv::CMP_GE);
        cv::bitwise_or(ratioMask, absoluteMask, ratioMask);

        // Brightness carries the same gradient - and it is not even radially symmetric, since
        // the light-pollution dome sits over one horizon - so a multiple of the frame median
        // marks the whole glowing side of the sky. Raise the bar where the sky itself is
        // bright, using the fitted sky surface that absorbs the glow whichever direction it
        // lies in, exactly as the dark path does. The surface is fitted through cloud as well
        // as clear sky, so it must only ever raise the bar, never lower it: taken alone it
        // dips over a dark clear region beside a cloud bank and then flags that clear sky.
        const MaskedHistogram skyRange = maskedHistogram(gray, skyMask8);
        const double skyFloor = skyRange.percentile(0.02);
        const double skyCeil = std::max(skyFloor + 1.0, static_cast<double>(skyRange.percentile(0.995)));
        cv::Mat cloudLevel;
        cv::max(polynomialBackground(gray, skyMask8, skyFloor, skyCeil), static_cast<float>(nightSkyMedian), cloudLevel);
        cv::Mat grayFloat;
        gray.convertTo(grayFloat, CV_32F);
        cv::Mat brightCloudMask;
        cv::compare(grayFloat, cloudLevel * moonlitCloudBrightness, brightCloudMask, cv::CMP_GE);
        cv::bitwise_or(ratioMask, brightCloudMask, ratioMask);
        cv::bitwise_and(ratioMask, brightMask, mask);

        applyStructureVote(mask, gray, evaluationMask, nightSkyMedian);
    }
    else if (night)
    {
        // A dark night sky varies smoothly across the frame (horizon glow, light pollution,
        // vignetting). A local box-average background cannot separate that smooth gradient
        // from cloud: near the bright horizon the sky is always "brighter than the darker
        // zenith" and would false-flag as cloud. Instead fit a smooth low-order surface to
        // the sky and compare the local sky level against it - the surface absorbs the glow
        // gradient, while cloud (locally bright, not a smooth trend) stands out above it.
        mask = structureContrastMask(gray, evaluationMask, false, debugMask);
    }
    else
    {
        // Clear blue sky has a red/blue ratio well below 1; white/grey cloud approaches or
        // exceeds it
        std::vector<cv::Mat> channels;
        cv::split(workBgr, channels);
        cv::Mat blue, red;
        channels[0].convertTo(blue, CV_32F);
        channels[2].convertTo(red, CV_32F);

        if (debugMask && (m_settings.m_cloudDebugView == CameraSettings::CloudDebugViewBackground)) {
            *debugMask = gray.clone();
        }
        if (debugMask && (m_settings.m_cloudDebugView == CameraSettings::CloudDebugViewSignal))
        {
            // Ratio image scaled so a ratio of 1.0 maps to mid-grey
            cv::Mat ratio = red / (blue + 1.0f);
            ratio.convertTo(*debugMask, CV_8U, 128.0);
        }

        // Dark regions must not classify as cloud, whatever their colour balance: lens
        // vignette, borders and shadowed structures are neutral and smooth, so they pass
        // the ratio and texture tests. By day cloud is at least comparably bright to the
        // sky, so anchor the floor to the evaluated sky's median brightness.
        const int daySkyLevel = skyLevels.percentile(0.5);
        const int brightnessFloor = std::max(dayMinBrightness, daySkyLevel / 2);
        const cv::Mat brightMask = gray >= brightnessFloor;

        // Grey/white but finely textured regions (roofs, trees, buildings) are not cloud.
        // Day path only: at night the fine-detail measure is dominated by sensor noise.
        // Near-saturated pixels are exempt: those are sunlit cloud tops, whose hard bright
        // edges read as texture, not roofs or trees.
        cv::Mat plausible = brightMask;
        if (m_settings.m_cloudTextureThreshold > 0)
        {
            // The threshold is a floor, not an absolute: the fine-detail measure scales with
            // the frame's noise, so a value that separates foliage from cloud on a bright
            // clean frame can sit below the noise floor of a dark high-gain exposure and veto
            // the sky itself (measured: 3 rejected 77 % of one clear-sky frame, whose own
            // median texture was also 3). Foreground has to be textured relative to the sky
            // it is seen against, so raise the bar to clear this frame's own texture.
            const int skyTexture = maskedPercentile(textureEnergy, evaluationMask, 0.90);
            const int effectiveTexture = std::max(m_settings.m_cloudTextureThreshold,
                                                  skyTexture + dayTextureNoiseMargin);
            cv::Mat smoothMask = textureEnergy < effectiveTexture;
            cv::Mat saturated;
            cv::compare(gray, daySaturatedCloud, saturated, cv::CMP_GE);
            cv::bitwise_or(smoothMask, saturated, smoothMask);
            cv::Mat gated;
            cv::bitwise_and(brightMask, smoothMask, gated);
            plausible = gated;
        }

        cv::Mat ratioMask;
        cv::compare(red, (blue + 1.0f) * m_settings.m_cloudDayThreshold, ratioMask, cv::CMP_GE);
        cv::bitwise_and(ratioMask, plausible, mask);

        // Overcast bluish-grey cloud shares the clear sky's red/blue ratio on this kind of
        // camera and so escapes the colour tests, exactly as pastel moonlit cloud does. When
        // the sky offers no genuine blue - the bluest quartile of the bright sky is itself
        // close to the cloud threshold - run the structure vote: it flips lumpy overcast and
        // leaves a smooth clear-blue sky (whose bluest quartile sits well below the gate)
        // untouched, so a clear or partly clear day never triggers it. The vote judges the
        // absolute-threshold mask, before the relative detections below are added: it counts
        // structure in what the colour test left unflagged, and detections placed there first
        // would consume the very evidence it weighs.
        cv::Mat brightSky;
        cv::bitwise_and(brightMask, evaluationMask, brightSky);
        const float clearSkyRatio = maskedRatioPercentile(red, blue, brightSky, moonlitClearSkyFraction);
        if (clearSkyRatio >= dayOvercastRatioGate) {
            applyStructureVote(mask, gray, evaluationMask, daySkyLevel);
        }

        // Grey cloud that never reaches the absolute threshold, but stands out against the
        // clear sky at its own elevation, is cloud too - without this a broken cumulus sky
        // reads as almost clear, since only the brightest sunlit tops pass the fixed bar.
        if (m_settings.m_cloudDayRelativeMargin > 0.0)
        {
            // Sample the profile over sky that has a colour to measure. Red and blue are
            // both near zero in the unlit surround and in silhouettes, so their ratio there
            // is noise - and a handful of such pixels in the outermost ring is enough to
            // drag the whole profile down through the monotonic step below it.
            cv::Mat colouredSky;
            cv::bitwise_and(evaluationMask, gray >= darkSkyFloor, colouredSky);
            cv::Mat relative = dayRelativeCloudMask(red, blue, evaluationMask, colouredSky, m_settings.m_cloudDayRelativeMargin);
            cv::bitwise_and(relative, plausible, relative);
            cv::bitwise_or(mask, relative, mask);
        }
    }

    cv::bitwise_and(mask, evaluationMask, mask);

    // Clear-sky reference comparison: pixels standing above this camera's known clear sky
    // are added as cloud (cue); pixels matching it within tolerances tighter than any
    // detection margin are removed (veto) - static quirks the in-frame heuristics misread
    // as cloud are thereby retired. Abstains when the slot is empty or the frame globally
    // disagrees with the reference (exposure regime change).
    if (m_settings.m_cloudUseReference && (referenceSlot >= 0)) {
        m_clearSkyReference.applyCueAndVeto(referenceSlot, mask, gray, workBgr, evaluationMask, roiNorm, m_settings.m_cloudNightThreshold);
    }

    if (debugMask && (m_settings.m_cloudDebugView == CameraSettings::CloudDebugViewThresholded)) {
        *debugMask = mask.clone();
    }

    if (m_settings.m_cloudOpenSize > 0)
    {
        const int ksize = 2 * m_settings.m_cloudOpenSize + 1;
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    }
    if (m_settings.m_cloudCloseSize > 0)
    {
        const int ksize = 2 * m_settings.m_cloudCloseSize + 1;
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    }
    // Close can bleed the mask back into excluded regions, so re-apply the evaluation mask
    cv::bitwise_and(mask, evaluationMask, mask);

    // Remove the sun/moon glare bloom from the final mask: the flagged blob at the projected
    // body position is the classifier seeing the body, not cloud. Runs on the final mask so
    // the removal is sized exactly to what was flagged, whatever the gain and exposure.
    applySunMoonMask(mask, evaluationMask, gray, workBgr, roi, imageSize, imageTransform, captureDateTime);

    // Night only: by day no stars are detectable, and a bright structure crossing a patch
    // could fake a "visible star" and veto genuine cloud. After the veto has cleared what
    // stars shine through, clusters of predicted stars that all fail to appear are added as
    // cloud - the cue for thin or dark cloud the brightness/colour paths cannot see.
    if (night)
    {
        applyStarVisibilityVeto(mask, starSense, roi);
        applyStarBlankCue(mask, evaluationMask, starSense, roi, imageSize, observationTime);
    }
    // Recorded after the cue so a sighting always reflects a completed classification pass
    recordStarVisibility(starSense, observationTime);

    if (debugMask && (m_settings.m_cloudDebugView == CameraSettings::CloudDebugViewFinal)) {
        *debugMask = mask.clone();
    }

    // No sky left to measure: report no result rather than a clear one (see
    // minEvaluatedFraction). The cloud result stays invalid, so nothing downstream consumes
    // it, no coverage event fires, and the next frame retries.
    const int evaluatedPixels = cv::countNonZero(evaluationMask);
    if (evaluatedPixels < std::max(1, static_cast<int>(std::lround(minEvaluatedFraction * gray.total()))))
    {
        if (!m_noSkyReported && m_msgQueueToFeature)
        {
            m_noSkyReported = true;
            m_msgQueueToFeature->push(MsgReportClearSkyReference::create(
                QStringLiteral("No cloud measurement: %1 of the frame is left to evaluate - check Min elevation, the rim margin, the exclusion rectangles and the lens pose")
                    .arg(100.0 * evaluatedPixels / std::max<double>(1.0, gray.total()), 0, 'f', 1)
                    .append(QStringLiteral(" %"))));
        }
        PROFILER_STOP(__FUNCTION__);
        return;
    }
    m_noSkyReported = false;

    cloud.m_mask = mask;
    cloud.m_roi = roi;
    cloud.m_coveragePercent = 100.0f * static_cast<float>(cv::countNonZero(mask)) / static_cast<float>(std::max(1, evaluatedPixels));
    cloud.m_night = night;
    cloud.m_valid = true;

    // Auto-learning: blend verified-clear frames into the reference so it tracks seasons,
    // lens dirt and slow drift, and fills slots the user never saved manually. At night,
    // when star sensing runs, the sky must additionally prove itself clear by showing its
    // predicted stars. When the frame as a whole is not verified clear, regions that are -
    // sky around visible predicted stars at night, confidently blue sky by day - still
    // accumulate into the reference, so it assembles patchwork-style at sites that never
    // get a wholly clear sky.
    if (m_settings.m_cloudUseReference && m_settings.m_cloudAutoReference && (referenceSlot >= 0)
        && m_clearSkyReference.learnDue(referenceSlot, roiNorm, observationTime))
    {
        int visibleStars = 0;
        for (const CloudStarSense::Star& star : starSense.stars) {
            visibleStars += star.visible ? 1 : 0;
        }

        // Day learning's only other verification is the detector's own coverage reading,
        // which is circular exactly on cameras where dark overcast is colorimetrically
        // invisible. The sun is the day analogue of the stars: when it projects into the
        // frame but shows no glare, cloud is in front of it and the frame must not become
        // the clear-sky reference (a sun outside the frame or too low leaves the existing
        // gates in charge; a sun behind a fixed obstruction should be covered by an
        // exclusion rectangle).
        const bool dayLearnBlocked = !night
            && (sunVisibility(gray, roi, imageSize, imageTransform, captureDateTime) == BodyVisibility::Obscured);
        if (dayLearnBlocked) {
            qDebug() << "CameraCloudDetector: day auto-learn skipped: sun obscured at its projected position";
        }

        // Patchwork confirmation is night-only: a visible predicted star is physical
        // proof the line of sight is clear. Day colour is NOT proof - on IR-sensitive
        // all-sky cameras dark cloud carries the same red/blue ratio as blue sky, and a
        // colour-based day rule was observed learning an overcast sky as the clear
        // reference within two updates. Day slots fill only from verified-clear whole
        // frames or a manual Save ref.
        cv::Mat confirmedClear;
        if (night)
        {
            // Sky around a visible predicted star is clear along that line of sight -
            // cloud would have hidden the star
            if (starSense.valid && (visibleStars > 0))
            {
                confirmedClear = cv::Mat::zeros(mask.size(), CV_8UC1);
                const double scaleX = static_cast<double>(mask.cols) / roi.width;
                const double scaleY = static_cast<double>(mask.rows) / roi.height;
                const int radius = std::max(4, cvRound(starClearRadiusFraction * std::max(mask.cols, mask.rows)));
                for (const CloudStarSense::Star& star : starSense.stars)
                {
                    if (!star.visible) {
                        continue;
                    }
                    const cv::Point centre(cvRound((star.position.x() - roi.x) * scaleX),
                                           cvRound((star.position.y() - roi.y) * scaleY));
                    if ((centre.x >= 0) && (centre.y >= 0) && (centre.x < mask.cols) && (centre.y < mask.rows)) {
                        cv::circle(confirmedClear, centre, radius, cv::Scalar(255), cv::FILLED);
                    }
                }
            }
        }
        if (!confirmedClear.empty())
        {
            // Stand clear of everything the final mask flagged, so a cloud edge (or its
            // undetected fringe) is never learned as clear sky. Measured as a distance to
            // the nearest flagged pixel rather than a dilation: the clearance is a large
            // radius (tens of pixels at the working resolution), and distanceTransform is
            // linear in the image where a dilation of that kernel is not - the same idiom
            // the rim margin uses.
            const int clearance = std::max(4, cvRound(starClearRadiusFraction * std::max(mask.cols, mask.rows)));
            cv::Mat notMask;
            cv::bitwise_not(mask, notMask);
            cv::Mat distanceToCloud;
            cv::distanceTransform(notMask, distanceToCloud, cv::DIST_L2, 5);
            const cv::Mat clearOfCloud = distanceToCloud >= static_cast<float>(clearance);
            cv::bitwise_and(confirmedClear, clearOfCloud, confirmedClear);
            cv::bitwise_and(confirmedClear, evaluationMask, confirmedClear);
        }

        const CameraClearSkyReference::LearnResult learned = dayLearnBlocked
            ? CameraClearSkyReference::LearnResult::None
            : m_clearSkyReference.autoLearn(
                referenceSlot, gray, workBgr, textureEnergy, evaluationMask, roiNorm, observationTime,
                cloud.m_coveragePercent, night, m_settings.m_cloudStarSense,
                starSense.valid ? starSense.stars.size() : 0, visibleStars, confirmedClear);
        if ((learned != CameraClearSkyReference::LearnResult::None) && m_msgQueueToFeature)
        {
            const QString what = (learned == CameraClearSkyReference::LearnResult::Frame)
                ? QStringLiteral("Auto-learned %1 - %2")
                : QStringLiteral("Learned clear patches in %1 - %2");
            m_msgQueueToFeature->push(MsgReportClearSkyReference::create(
                what.arg(CameraClearSkyReference::slotName(referenceSlot),
                    m_clearSkyReference.statusSummary(referenceSlot))));
        }
    }

    PROFILER_STOP(__FUNCTION__);
}
