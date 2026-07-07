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
#ifdef CAMERA_OPENCV_CUDA_CLOUD_DETECTION
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include "util/astronomy.h"
#include "util/profiler.h"
#include "cameraclouddetector.h"

MESSAGE_CLASS_DEFINITION(CameraCloudDetector::MsgReportCloudCoverage, Message)

namespace {

// Absolute lower bound on the day-path brightness floor; the effective floor also scales
// with the evaluated sky's median brightness so dark vignette/borders never classify
constexpr int dayMinBrightness = 30;
// Auto-mode hysteresis band: day at or above the upper bound, night at or below the lower
constexpr double autoDayBrightness = 60.0;
constexpr double autoNightBrightness = 45.0;
// Auto-mode sun-elevation bounds in degrees: the day-path colour cues need a bright sky and
// the night-path background deviation needs real darkness; in the twilight band between them
// (and when no observation time is available) the brightness heuristic decides
constexpr double sunDayElevation = -4.0;
constexpr double sunNightElevation = -10.0;

// Median of the 8-bit values where mask is non-zero, as a robust sky-level estimate that
// ignores excluded regions and is insensitive to cloud outliers
int maskedMedian(const cv::Mat& values, const cv::Mat& mask)
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

    int remaining = (total + 1) / 2;
    for (int bin = 0; bin < 256; ++bin)
    {
        remaining -= histogram[bin];
        if (remaining <= 0) {
            return bin;
        }
    }

    return 0;
}

} // namespace

CameraCloudDetector::CameraCloudDetector() :
    m_msgQueueToFeature(nullptr),
    m_framesSinceUpdate(0),
    m_autoNight(true),
    m_haveAutoModeState(false)
{
}

CameraCloudDetector::~CameraCloudDetector() = default;

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
        || settingsKeys.contains("plateSolveDateTimeUtc");
}

void CameraCloudDetector::invalidateCache()
{
    m_lastCloud = CameraPipelineCloud();
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
    if (!active) {
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
    m_lastInputFrame.reset(new CameraPipelineFrame(*frame));

    if (!m_settings.m_cloudDetect)
    {
        forwardFrame(frame);
        return;
    }

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

    // The debug view replaces the frame image, so it must be rendered on every frame to
    // avoid the display flickering between the mask and the live image
    const bool debugViewActive = m_settings.m_cloudDebugView != CameraSettings::CloudDebugViewOff;
    ++m_framesSinceUpdate;
    const bool recompute = !m_lastCloud.m_valid
        || (m_framesSinceUpdate >= m_settings.m_cloudUpdateIntervalFrames)
        || (m_lastFrameSize != frameSize)
        || (m_lastCloud.m_roi != detectionRoi)
        || (m_lastContentRect != contentRect)
        || frame->m_manualPreviewFrame
        || debugViewActive;

    if (!recompute)
    {
        // Clouds evolve slowly; stamp the cached result onto intermediate frames. The mask
        // cv::Mat is shared by refcount, so downstream stages must treat it as read-only.
        frame->m_cloud = m_lastCloud;
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

        QImage convertedRgb;
        const QImage& rgb = ensureRgb888(frame->m_image, convertedRgb);
        cv::Mat mat = wrapRgb888Image(rgb);
        cv::Mat bgrMat;
        cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);
        prepareWorkImages(bgrMat, detectionRoi, workBgr, rawGray, gray);
    }

    cv::Mat cloudDebugMask;
    applyCloudDetection(workBgr, rawGray, gray, detectionRoi, contentRect, frame->m_captureDateTime, frame->m_cloud, debugViewActive ? &cloudDebugMask : nullptr);

    m_lastCloud = frame->m_cloud;
    m_lastFrameSize = frameSize;
    m_lastContentRect = contentRect;
    m_framesSinceUpdate = 0;

    if (m_msgQueueToFeature && frame->m_cloud.m_valid) {
        m_msgQueueToFeature->push(MsgReportCloudCoverage::create(frame->m_cloud.m_coveragePercent, frame->m_cloud.m_night, frame->m_captureDateTime));
    }

    if (!cloudDebugMask.empty())
    {
        cv::Mat maskCanvas = cv::Mat::zeros(frameCvSize, CV_8UC1);
        cv::Mat roiMask = cloudDebugMask;
        if (cloudDebugMask.size() != detectionRoi.size()) {
            cv::resize(cloudDebugMask, roiMask, detectionRoi.size(), 0.0, 0.0, cv::INTER_NEAREST);
        }
        roiMask.copyTo(maskCanvas(detectionRoi));
        cv::Mat debugBgr;
        cv::cvtColor(maskCanvas, debugBgr, cv::COLOR_GRAY2BGR);
        frame->m_image = convertBgrToRgbImage(debugBgr);
        frame->clearCudaCache();
    }

    forwardFrame(frame);
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
        if (sunAzAlt.alt >= sunDayElevation)
        {
            m_autoNight = false;
            m_haveAutoModeState = true;
            return false;
        }
        if (sunAzAlt.alt <= sunNightElevation)
        {
            m_autoNight = true;
            m_haveAutoModeState = true;
            return true;
        }
        // Twilight: neither path is clearly right, so let the brightness heuristic decide
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

// Produces the downscaled work images the classification runs on: BGR (for the day-path
// colour ratio), raw luminance and median-blurred luminance. The median blur erases stars,
// hot pixels and other point sources so they never register as cloud texture.
void CameraCloudDetector::prepareWorkImages(const cv::Mat& bgrMat, const cv::Rect& roi, cv::Mat& workBgr, cv::Mat& rawGray, cv::Mat& gray) const
{
    PROFILER_START();

    workBgr = bgrMat(roi);
    const double downscale = m_settings.m_cloudDownscale;
    if (downscale < 0.999)
    {
        const cv::Size downscaledSize(
            std::max(1, static_cast<int>(std::lround(roi.width * downscale))),
            std::max(1, static_cast<int>(std::lround(roi.height * downscale))));
        cv::Mat downscaledInput;
        cv::resize(workBgr, downscaledInput, downscaledSize, 0.0, 0.0, cv::INTER_AREA);
        workBgr = downscaledInput;
    }

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

void CameraCloudDetector::applyCloudDetection(const cv::Mat& workBgr, const cv::Mat& rawGray, const cv::Mat& gray, const cv::Rect& roi, const cv::Rect& contentRect, const QDateTime& captureDateTime, CameraPipelineCloud& cloud, cv::Mat* debugMask)
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

    const bool night = resolveNightMode(gray, evaluationMask, captureDateTime);

    cv::Mat mask;
    if (night)
    {
        // A clear night sky is smooth apart from point sources, so clouds show up as
        // low-frequency structure: brighter under moonlight/light pollution, darker where
        // airglow is blocked. Compare a heavily blurred background against a robust global
        // sky level and threshold the absolute deviation.
        const int ksize = 2 * m_settings.m_cloudBackgroundBlur + 1;
        cv::Mat background;
        cv::boxFilter(gray, background, -1, cv::Size(ksize, ksize));
        if (debugMask && (m_settings.m_cloudDebugView == CameraSettings::CloudDebugViewBackground)) {
            *debugMask = background.clone();
        }

        const int skyLevel = maskedMedian(background, evaluationMask);
        cv::Mat deviation;
        cv::absdiff(background, cv::Scalar(skyLevel), deviation);
        if (debugMask && (m_settings.m_cloudDebugView == CameraSettings::CloudDebugViewSignal)) {
            *debugMask = deviation.clone();
        }

        cv::threshold(deviation, mask, m_settings.m_cloudNightThreshold, 255, cv::THRESH_BINARY);
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
        const int daySkyLevel = maskedMedian(gray, evaluationMask);
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
    if (debugMask && (m_settings.m_cloudDebugView == CameraSettings::CloudDebugViewFinal)) {
        *debugMask = mask.clone();
    }

    const int evaluatedPixels = cv::countNonZero(evaluationMask);
    cloud.m_mask = mask;
    cloud.m_roi = roi;
    cloud.m_coveragePercent = 100.0f * static_cast<float>(cv::countNonZero(mask)) / static_cast<float>(std::max(1, evaluatedPixels));
    cloud.m_night = night;
    cloud.m_valid = true;

    PROFILER_STOP(__FUNCTION__);
}
