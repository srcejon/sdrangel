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
#ifdef CAMERA_OPENCV_CUDA_CLOUD_DETECTION
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include "util/astronomy.h"
#include "util/profiler.h"
#include "camerainfo.h"
#include "cameraclouddetector.h"
#include "cameraplatesolver.h"
#include "cameraskyprojector.h"

MESSAGE_CLASS_DEFINITION(CameraCloudDetector::MsgReportCloudCoverage, Message)
MESSAGE_CLASS_DEFINITION(CameraCloudDetector::MsgSaveClearSkyReference, Message)
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
// Moonlit night: cloud is where the red/blue ratio exceeds the clear-sky ratio by this
// margin. Relative rather than absolute, because high gain and night white balance shift
// the whole ratio scale. The clear-sky ratio is anchored at a low percentile of the bright
// sky, so it still finds the clear gaps when cloud covers most of the frame. On a fully
// overcast sky the anchor inevitably lands inside cloud, so the resulting threshold is
// capped at the day threshold: anything at least as white as daytime cloud is cloud at
// night too, whatever the anchor says
constexpr double moonlitClearSkyFraction = 0.05;
constexpr float moonlitRatioMargin = 0.10f;
// Moonlit night: cloud lit by the moon or ground light is also much brighter than the
// median night sky; this catches grey/white cloud when cloud dominates the frame and the
// colour anchor has little clear sky to calibrate against
constexpr double moonlitCloudBrightness = 1.3;
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
// Dark night: pixels below this brightness are border or foreground, not sky, and are kept
// out of the local background average so the dark fisheye surround cannot bleed into it
constexpr int darkSkyFloor = 12;
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

// Percentile of the 8-bit values where mask is non-zero (0.5 = median), as a robust
// sky-level estimate that ignores excluded regions and is insensitive to outliers
int maskedPercentile(const cv::Mat& values, const cv::Mat& mask, double fraction)
{
    int histogram[256] = {0};
    int total = 0;
    for (int row = 0; row < values.rows; ++row)
    {
        const uchar *valueLine = values.ptr<uchar>(row);
        const uchar *maskLine = mask.ptr<uchar>(row);
        for (int col = 0; col < values.cols; ++col)
        {
            if (maskLine[col])
            {
                ++histogram[valueLine[col]];
                ++total;
            }
        }
    }

    int remaining = std::max(1, static_cast<int>(std::ceil(std::clamp(fraction, 0.0, 1.0) * total)));
    for (int bin = 0; bin < 256; ++bin)
    {
        remaining -= histogram[bin];
        if (remaining <= 0) {
            return bin;
        }
    }

    return 0;
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
    if (MsgSaveClearSkyReference::match(cmd))
    {
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
        return true;
    }

    return false;
}

bool CameraCloudDetector::cloudSettingsChanged(const QList<QString>& settingsKeys)
{
    return settingsKeys.contains("cloudDetect")
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
        || settingsKeys.contains("lensMirror");
}

void CameraCloudDetector::invalidateCache()
{
    m_lastCloud = CameraPipelineCloud();
    m_lastDebugMask = cv::Mat();
    m_framesSinceUpdate = 0;
    m_lastFrameSize = QSize();
    m_lastContentRect = cv::Rect();
    m_haveAutoModeState = false;
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
            frame->m_manualPreviewFrame = true;
            submitFrame(frame);
        }
    }
}

void CameraCloudDetector::captureActiveChanged(bool active)
{
    if (!active)
    {
        // Release the retained tuning frame so its image buffers (CPU and GPU) are not
        // pinned while capture is stopped
        m_lastInputFrame.reset();
        return;
    }

    invalidateCache();
}

void CameraCloudDetector::processNewFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || !frame->hasImageData()) {
        return;
    }

    frame->m_cloud = CameraPipelineCloud();

    if (!m_settings.m_cloudDetect)
    {
        forwardFrame(frame);
        return;
    }

    // Kept so applySettings() can re-run detection on a paused image while tuning
    m_lastInputFrame.reset(new CameraPipelineFrame(*frame));

    const QSize frameSize = frame->imageSize();
    if (frameSize.isEmpty()) {
        return;
    }
    const cv::Size frameCvSize(frameSize.width(), frameSize.height());
    const cv::Rect detectionRoi = resolveDetectionRoi(frameCvSize);

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
        // The debug view replaces the frame image, so it must be rendered on every frame to
        // avoid the display flickering between the mask and the live image; classification
        // itself still only runs on the recompute cadence
        if (debugViewActive && !m_lastDebugMask.empty()) {
            renderDebugView(frame, frameCvSize, m_lastCloud.m_roi);
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

    if (m_msgQueueToFeature && frame->m_cloud.m_valid) {
        m_msgQueueToFeature->push(MsgReportCloudCoverage::create(frame->m_cloud.m_coveragePercent, frame->m_cloud.m_night, frame->m_captureDateTime));
    }

    m_lastDebugMask = cloudDebugMask;
    if (!m_lastDebugMask.empty()) {
        renderDebugView(frame, frameCvSize, detectionRoi);
    }

    forwardFrame(frame);
}

// Paints the cached debug-view mask onto the frame in place of the camera image. Called on
// every frame while a debug view is active (a stale image would flicker against the live
// feed), while the mask itself is only recomputed on the normal update cadence.
void CameraCloudDetector::renderDebugView(const CameraPipelineFramePtr& frame, const cv::Size& frameCvSize, const cv::Rect& roi) const
{
    cv::Mat maskCanvas = cv::Mat::zeros(frameCvSize, CV_8UC1);
    cv::Mat roiMask = m_lastDebugMask;
    if (m_lastDebugMask.size() != roi.size()) {
        cv::resize(m_lastDebugMask, roiMask, roi.size(), 0.0, 0.0, cv::INTER_NEAREST);
    }
    roiMask.copyTo(maskCanvas(roi));
    cv::Mat debugBgr;
    cv::cvtColor(maskCanvas, debugBgr, cv::COLOR_GRAY2BGR);
    frame->m_image = convertBgrToRgbImage(debugBgr);
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
// gain and exposure - clipped to the configured maximum radius. Runs after classification and
// morphology, before coverage is measured, so the freed pixels count as the clear sky they are.
void CameraCloudDetector::applySunMoonMask(cv::Mat& mask, cv::Mat& evaluationMask, const cv::Mat& gray, const cv::Rect& roi, const QSize& imageSize, const CameraPipelineImageTransform& imageTransform, const QDateTime& captureDateTime) const
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

    const auto maskBody = [&](const AzAlt& body)
    {
        // Skip a body fully below the horizon (its bloom would not reach into the sky region).
        if (body.alt < -maxRadiusDeg) {
            return;
        }

        QPointF centerImage;
        if (!projector.projectAltAz(body.az, body.alt, centerImage)) {
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
        if (window.area() <= 0) {
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

        cv::Mat labels;
        const int labelCount = cv::connectedComponents(mask(window), labels, 8);
        std::vector<uchar> touchesSeed(static_cast<size_t>(labelCount), 0);
        std::vector<uchar> leavesDisc(static_cast<size_t>(labelCount), 0);
        for (int row = 0; row < labels.rows; ++row)
        {
            const int *labelLine = labels.ptr<int>(row);
            const uchar *seedLine = seedMask.ptr<uchar>(row);
            const uchar *discLine = discMask.ptr<uchar>(row);
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
                }
            }
        }

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
                // Seed-touching blobs are trimmed within the disc; blobs wholly inside the
                // disc are glare debris and removed outright
                if ((touchesSeed[static_cast<size_t>(label)] && discLine[col])
                    || !leavesDisc[static_cast<size_t>(label)]) {
                    removalLine[col] = 255;
                }
            }
        }
        mask(window).setTo(0, removal);
    };

    AzAlt azAlt;
    RADec raDec;
    Astronomy::sunPosition(azAlt, raDec, m_settings.m_latitude, m_settings.m_longitude, observationTime);
    maskBody(azAlt);
    Astronomy::moonPosition(azAlt, raDec, m_settings.m_latitude, m_settings.m_longitude, observationTime);
    maskBody(azAlt);
}

// Extracts a full-resolution grayscale patch centred on a predicted star position. Works
// from the CPU image when present, otherwise downloads just the patch from the GPU frame.
bool CameraCloudDetector::samplePatchGray(const CameraPipelineFramePtr& frame, const QPoint& centre, int half, cv::Mat& patch)
{
    const int size = 2 * half + 1;

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

    const QVector<CameraPlateSolver::BrightStar> catalog = CameraPlateSolver::brightStarCatalog(m_settings);
    for (const CameraPlateSolver::BrightStar& star : catalog)
    {
        if (star.magnitude > m_settings.m_cloudStarSenseMagnitude) {
            continue;
        }

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
        if (!samplePatchGray(frame, centre, starSensePatchHalf, patch)) {
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
        sense.stars.append({position, visible});
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

    for (int row = 0; row < labels.rows; ++row)
    {
        const int *labelLine = labels.ptr<int>(row);
        uchar *maskLine = mask.ptr<uchar>(row);
        for (int col = 0; col < labels.cols; ++col)
        {
            if ((labelLine[col] > 0) && veto[static_cast<size_t>(labelLine[col])]) {
                maskLine[col] = 0;
            }
        }
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
        const double skyFloor = maskedPercentile(gray, skyMask8, 0.02);
        // The ceiling must sit at the very top of the sky range: clamping lower parks the
        // surface beneath the brightest horizon/rim glow, and the uncovered glow then reads
        // as false cloud (an order-3 surface cannot ride up into patchy cloud regardless)
        const double skyCeil = std::max(skyFloor + 1.0, static_cast<double>(maskedPercentile(gray, skyMask8, 0.995)));
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
    cv::Mat evaluationMask = exclusionMask;
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
    if ((m_settings.m_cloudUseReference || m_saveReferencePending) && observationTime.isValid())
    {
        AzAlt sunAzAlt, moonAzAlt;
        RADec bodyRaDec;
        Astronomy::sunPosition(sunAzAlt, bodyRaDec, m_settings.m_latitude, m_settings.m_longitude, observationTime);
        Astronomy::moonPosition(moonAzAlt, bodyRaDec, m_settings.m_latitude, m_settings.m_longitude, observationTime);
        referenceSlot = CameraClearSkyReference::slotFor(sunAzAlt.alt, moonAzAlt.alt);
        m_clearSkyReference.ensureLoaded(referenceStorageKey(m_settings));
    }
    const QRectF roiNorm(
        static_cast<double>(roi.x) / std::max(1, imageSize.width()),
        static_cast<double>(roi.y) / std::max(1, imageSize.height()),
        static_cast<double>(roi.width) / std::max(1, imageSize.width()),
        static_cast<double>(roi.height) / std::max(1, imageSize.height()));

    // The rim-margin and sun/moon masks (and the learned foreground exclusion) mutate the
    // evaluation mask in place. In the common case it aliases the cached exclusion mask
    // (shallow cv::Mat copy), so take a private copy first to avoid corrupting the cache
    // carried to later frames.
    if ((m_settings.m_cloudEdgeMarginPercent > 0.0) || m_settings.m_cloudMaskSunMoon
        || (m_settings.m_cloudUseReference && (referenceSlot >= 0))) {
        evaluationMask = evaluationMask.clone();
    }

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

    // Save-reference request: capture the evaluated sky as this sky state's clear
    // reference. Requires an observation time (to know the sky state); reports back so the
    // GUI can show the store status.
    if (m_saveReferencePending)
    {
        m_saveReferencePending = false;
        if (referenceSlot >= 0)
        {
            m_clearSkyReference.ensureLoaded(referenceStorageKey(m_settings));
            m_clearSkyReference.capture(referenceSlot, gray, workBgr, textureEnergy, evaluationMask, roiNorm, observationTime);
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

    const bool night = resolveNightMode(gray, evaluationMask, captureDateTime);

    cv::Mat mask;
    const int nightSkyMedian = night ? maskedPercentile(gray, evaluationMask, 0.5) : 0;
    const int nightSkyBrightQuartile = night ? maskedPercentile(gray, evaluationMask, moonlitSkyFraction) : 0;
    if (night && (nightSkyBrightQuartile >= moonlitBrightness))
    {
        // A bright night sky (moonlit, or high gain and long exposure) is Rayleigh-scattered
        // light and behaves like dim daylight: clear sky is blue, cloud is white/pink. The
        // luminance-deviation approach fails here because moonlight and vignette span a wide
        // brightness range. Classify by red/blue ratio instead, but with the threshold
        // anchored to the bluest quartile of the bright sky, since gain and night white
        // balance shift the whole ratio scale.
        std::vector<cv::Mat> channels;
        cv::split(workBgr, channels);
        cv::Mat blue, red;
        channels[0].convertTo(blue, CV_32F);
        channels[2].convertTo(red, CV_32F);

        const int brightnessFloor = std::max(dayMinBrightness, nightSkyMedian / 2);
        const cv::Mat brightMask = gray >= brightnessFloor;
        cv::Mat skyMask;
        cv::bitwise_and(brightMask, evaluationMask, skyMask);
        const float clearSkyRatio = maskedRatioPercentile(red, blue, skyMask, moonlitClearSkyFraction);
        const float cloudRatio = std::min(clearSkyRatio + moonlitRatioMargin, static_cast<float>(m_settings.m_cloudDayThreshold));

        if (debugMask && (m_settings.m_cloudDebugView == CameraSettings::CloudDebugViewBackground)) {
            *debugMask = gray.clone();
        }
        if (debugMask && (m_settings.m_cloudDebugView == CameraSettings::CloudDebugViewSignal))
        {
            // Ratio image scaled so a ratio of 1.0 maps to mid-grey
            cv::Mat ratio = red / (blue + 1.0f);
            ratio.convertTo(*debugMask, CV_8U, 128.0);
        }

        cv::Mat ratioMask;
        cv::compare(red, (blue + 1.0f) * cloudRatio, ratioMask, cv::CMP_GE);

        const int cloudBrightness = std::min(255, static_cast<int>(std::lround(moonlitCloudBrightness * nightSkyMedian)));
        const cv::Mat brightCloudMask = gray >= cloudBrightness;
        cv::bitwise_or(ratioMask, brightCloudMask, ratioMask);
        cv::bitwise_and(ratioMask, brightMask, mask);

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
        const int structureBrightness = static_cast<int>(std::lround(moonlitStructureBrightness * nightSkyMedian));
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

        cv::Mat ratioMask;
        cv::compare(red, (blue + 1.0f) * m_settings.m_cloudDayThreshold, ratioMask, cv::CMP_GE);

        // Dark regions must not classify as cloud, whatever their colour balance: lens
        // vignette, borders and shadowed structures are neutral and smooth, so they pass
        // the ratio and texture tests. By day cloud is at least comparably bright to the
        // sky, so anchor the floor to the evaluated sky's median brightness.
        const int daySkyLevel = maskedPercentile(gray, evaluationMask, 0.5);
        const int brightnessFloor = std::max(dayMinBrightness, daySkyLevel / 2);
        const cv::Mat brightMask = gray >= brightnessFloor;
        cv::bitwise_and(ratioMask, brightMask, mask);

        // Grey/white but finely textured regions (roofs, trees, buildings) are not cloud.
        // Day path only: at night the fine-detail measure is dominated by sensor noise.
        if (m_settings.m_cloudTextureThreshold > 0)
        {
            const cv::Mat smoothMask = textureEnergy < m_settings.m_cloudTextureThreshold;
            cv::bitwise_and(mask, smoothMask, mask);
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
    applySunMoonMask(mask, evaluationMask, gray, roi, imageSize, imageTransform, captureDateTime);

    // Night only: by day no stars are detectable, and a bright structure crossing a patch
    // could fake a "visible star" and veto genuine cloud
    if (night) {
        applyStarVisibilityVeto(mask, starSense, roi);
    }

    if (debugMask && (m_settings.m_cloudDebugView == CameraSettings::CloudDebugViewFinal)) {
        *debugMask = mask.clone();
    }

    const int evaluatedPixels = cv::countNonZero(evaluationMask);
    cloud.m_mask = mask;
    cloud.m_roi = roi;
    cloud.m_coveragePercent = 100.0f * static_cast<float>(cv::countNonZero(mask)) / static_cast<float>(std::max(1, evaluatedPixels));
    cloud.m_night = night;
    cloud.m_valid = true;

    // Auto-learning: blend verified-clear frames into the reference so it tracks seasons,
    // lens dirt and slow drift, and fills slots the user never saved manually. At night,
    // when star sensing runs, the sky must additionally prove itself clear by showing its
    // predicted stars.
    if (m_settings.m_cloudUseReference && m_settings.m_cloudAutoReference && (referenceSlot >= 0))
    {
        int visibleStars = 0;
        for (const CloudStarSense::Star& star : starSense.stars) {
            visibleStars += star.visible ? 1 : 0;
        }
        const bool starConfirmed = starSense.valid && (visibleStars >= 5);
        if (m_clearSkyReference.autoLearn(referenceSlot, gray, workBgr, textureEnergy, evaluationMask, roiNorm, observationTime,
                                          cloud.m_coveragePercent, night, m_settings.m_cloudStarSense, starConfirmed)
            && m_msgQueueToFeature)
        {
            m_msgQueueToFeature->push(MsgReportClearSkyReference::create(
                QStringLiteral("Auto-learned %1 - %2").arg(CameraClearSkyReference::slotName(referenceSlot),
                    m_clearSkyReference.statusSummary(referenceSlot))));
        }
    }

    PROFILER_STOP(__FUNCTION__);
}
