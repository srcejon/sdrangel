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
#include <QColor>
#include <QDataStream>
#include <QIODevice>
#include <QJsonArray>
#include <QJsonDocument>
#include <sstream>

#include "util/simpleserializer.h"
#include "util/httpdownloadmanager.h"
#include "settings/serializable.h"
#include "maincore.h"
#include "camerainfo.h"
#include "camerasettings.h"

#define DEFAULT_OVERLAY_TEXT_STRING "<img src=\":/sdrangel_icon.png\"><h1 style=\"color:blue\">SDRangel</h1>\n<p>\nText overlay "

namespace {
float normalizePositiveDegrees(float value)
{
    value = std::fmod(value, CameraSettings::m_fullRotationDegrees);
    if (value < 0.0f) {
        value += CameraSettings::m_fullRotationDegrees;
    }
    return value;
}

float normalizeSignedDegrees(float value)
{
    value = std::fmod(value, CameraSettings::m_fullRotationDegrees);
    if (value <= CameraSettings::m_minRoll) {
        value += CameraSettings::m_fullRotationDegrees;
    } else if (value > CameraSettings::m_maxRoll) {
        value -= CameraSettings::m_fullRotationDegrees;
    }
    return value;
}

int normalizeImageRotation(int value)
{
    int normalized = value % 360;
    if (normalized < 0) {
        normalized += 360;
    }

    switch (normalized)
    {
    case 90:
    case 180:
    case 270:
        return normalized;
    case 0:
    default:
        return 0;
    }
}

float calculateLongEdgeFovDegrees(double sensorWidthMm, double sensorHeightMm, double focalLengthMm)
{
    const double sensorLongEdgeMm = std::max(sensorWidthMm, sensorHeightMm);
    if ((sensorLongEdgeMm <= 0.0) || (focalLengthMm <= 0.0)) {
        return CameraSettings::m_minFov;
    }

    constexpr double radiansToDegrees = 180.0 / 3.14159265358979323846;
    const double fovDegrees = 2.0 * std::atan(sensorLongEdgeMm / (2.0 * focalLengthMm)) * radiansToDegrees;
    return static_cast<float>(qBound(
        static_cast<double>(CameraSettings::m_minFov),
        fovDegrees,
        static_cast<double>(CameraSettings::m_maxFov)));
}

QString serializeStringList(const QStringList& values)
{
    QJsonArray array;
    for (const QString& value : values) {
        array.append(value);
    }
    return QString::fromUtf8(QJsonDocument(array).toJson(QJsonDocument::Compact));
}

QStringList deserializeStringList(const QString& json)
{
    QStringList values;
    const QJsonDocument document = QJsonDocument::fromJson(json.toUtf8());
    if (!document.isArray()) {
        return values;
    }

    const QJsonArray array = document.array();
    for (const QJsonValue& value : array)
    {
        if (value.isString()) {
            values.append(value.toString());
        }
    }
    return values;
}
}

CameraSettings::CameraSettings() :
    m_rollupState(nullptr)
{
    resetToDefaults();
}

void CameraSettings::resetToDefaults()
{
    m_title = "Camera";
    m_rgbColor = QColor(64, 128, 255).rgb();
    m_cameraProtocol.clear();
    m_cameraId.clear();
    m_cameraDescription.clear();
    m_resolutionWidth = 1280;
    m_resolutionHeight = 720;
    m_framesPerSecond = 10;
    m_captureMode = CaptureModeFrameRate;
    m_captureInterval = 1.0;
    m_captureIntervalUnits = CaptureIntervalSeconds;
    m_exposureTimeMs = 50;
    m_isoSensitivity = -1; // -1 is auto
    m_alpacaDiscoveryEnabled = true;
    m_alpacaApiLogEnabled = false;
    m_alpacaHost = "127.0.0.1";
    m_alpacaPort = 11111;
    m_alpacaFocuserEnabled = false;
    m_alpacaFocuserHost = "127.0.0.1";
    m_alpacaFocuserPort = 11111;
    m_alpacaFocuserDeviceNumber = 0;
    m_alpacaFocusPosition = 0;
    m_alpacaFocusStepSize = 100;
    m_alpacaFilterWheelEnabled = false;
    m_alpacaFilterWheelHost = "127.0.0.1";
    m_alpacaFilterWheelPort = 11111;
    m_alpacaFilterWheelDeviceNumber = 0;
    m_alpacaFilterWheelPosition = 0;
    m_cameraBinX = 1;
    m_cameraBinY = 1;
    m_cameraNumX = 0;
    m_cameraNumY = 0;
    m_cameraStartX = 0;
    m_cameraStartY = 0;
    m_cameraGain = 100;
    m_cameraOffset = 1;
    m_cameraReadoutMode = 0;
    m_asiCoolerOn = -1;
    m_asiTargetTemp = std::numeric_limits<int>::min();
    m_asiUsbBandwidth = -1;
    m_asiHighSpeedMode = -1;
    m_asiAutoExposureGain = false;
    m_autoExposureGainEnabled = false;
    m_autoExposureGainMode = AutoExposureGainExposureFirst;
    m_autoExposureTargetPercentile = 95.0;
    m_autoExposureTargetBrightness = 70.0;
    m_autoExposureMaxChangePercent = 25.0;
    m_autoExposureMinMs = m_minExposureTimeMs;
    m_autoExposureMaxMs = 60000.0;
    m_autoExposureMinGain = 0;
    m_autoExposureMaxGain = 100;
    m_asiColorImageType = AsiColorImageTypeRgb24;
    m_saveImage = false;
    m_imageFileName = "camera.jpg";
    m_saveVideo = false;
    m_videoFileCameraPath.clear();
    m_streamUrl.clear();
    m_imageFileCameraPaths.clear();
    m_streamUrlHistory.clear();
    m_streamBufferingSeconds = 1.0;
    m_videoFileName = "camera.mp4";
    m_recordRawFits = false;
    m_recordCalibratedMedia = true;
    m_recordFilteredMedia = false;
    m_recordPostProcessedMedia = false;
    m_keogramEnabled = false;
    m_keogramFileName = "keogram.png";
    m_keogramDirection = KeogramVertical;
    m_keogramDayMode = KeogramMidnightToMidnight;
    m_keogramSamplePeriodMinutes = 5;
    m_keogramShowPreview = false;
    m_youtubeStreamEnabled = false;
    m_youtubeStreamUrl = "rtmps://a.rtmps.youtube.com/live2";
    m_youtubeStreamKey.clear();
    m_youtubeStreamPostProcessed = true;
    m_youtubeStreamBitrateKbps = 6800;
    m_youtubeStreamFps = 25;
    m_youtubeStreamWidth = 0;
    m_youtubeStreamHeight = 0;
    m_videoCodec = VideoCodecH264;
    m_videoRecordBitrateKbps = 0;
    m_videoLoop = false;
    m_videoPlaybackRate = 1.0;
    m_videoPlaybackAudioOffsetMs = 0;
    m_videoHwAcceleration = true;
    m_videoPreRecordBufferSeconds = 0;
    m_imageRecordLimit = 0;
    m_videoRecordLimitSeconds = 0;
    m_stackEnabled = false;
    m_stackFrameCount = 4;
    m_stackMethod = StackMethodAverage;
    m_stackHdrAlgorithm = StackHdrAlgorithmDebevec;
    m_stackHdrExposureCount = 3;
    m_stackHdrExposureTimesMs = {{12.5, 50.0, 200.0, 800.0}};
    m_stackAlignmentMethod = StackAlignmentNone;
    m_stackDisplayMode = StackDisplayStacked;
    m_stackDisplayFrameIndex = 0;
    m_stackRejectBadFrames = false;
    m_scaleEnabled = false;
    m_scaleWidth = 0;
    m_scaleHeight = 0;
    m_scaleKeepAspectRatio = true;
    m_scaleJustification = ScaleJustifyCenter;
    m_stackDarkFileName.clear();
    m_stackFlatFileName.clear();
    m_stackBiasFileName.clear();
    m_latitude = MainCore::instance()->getSettings().getLatitude();
    m_longitude = MainCore::instance()->getSettings().getLongitude();
    m_altitude = MainCore::instance()->getSettings().getAltitude();
    m_positionSync = false;
    m_owmAPIKey.clear();
    m_azimuth = 0.0f;
    m_elevation = 0.0f;
    m_roll = 0.0f;
    m_rotator.clear();
    m_directionSensor.clear();
    m_azimuthOffset = 0.0f;
    m_elevationOffset = 0.0f;
    m_rollOffset = 0.0f;
    m_fov = 60.0f;
    m_fovMode = FovModeDirect;
    m_fovSensorWidthMm = 36.0;
    m_fovSensorHeightMm = 24.0;
    m_fovFocalLengthMm = 35.0;
    m_lensProjection = LensProjectionRectilinear;
    m_lensCenterOffsetX = 0.0;
    m_lensCenterOffsetY = 0.0;
    m_lensDistortionK1 = 0.0;
    m_workspaceIndex = 0;
    m_geometryBytes.clear();
    m_postProcessWhiteBalanceMode = 0;
    m_postProcessWhiteBalanceRedGain = 1.0;
    m_postProcessWhiteBalanceGreenGain = 1.0;
    m_postProcessWhiteBalanceBlueGain = 1.0;
    m_postProcessWhiteBalanceHighlightProtection = 0.0;
    m_postProcessUseCuda = false;
    m_postProcessUnwarp = false;
    m_histogramStretch = HistogramStretchOff;
    m_histogramStretchBlackPoint = 0.0;
    m_histogramStretchWhitePoint = 1.0;
    m_histogramStretchGamma = 1.0;
    m_histogramStretchAsinhStrength = 10.0;
    m_histogramStretchLogStrength = 10.0;
    m_histogramVisible = false;
    m_postProcessGreyscale = false;
    m_saturation = 1.0;
    m_gamma = 1.0;
    m_gaussianBlur = 0;
    m_medianBlur = 0;
    m_sharpen = 0.0;
    m_edgeDisplayMode = EdgeDisplayOverlay;
    m_sobelEdge = 0.0;
    m_cannyEdge = 0.0;
    m_flipX = false;
    m_flipY = false;
    m_imageRotation = 0;
    m_brightness = 0.0;
    m_contrast = 1.0;
    m_invertColors = false;
    m_overlayDateTime = false;
    m_dateTimeColor = Qt::white;
    m_dateTimeFormat = QStringLiteral("yyyy-MM-dd hh:mm:ss");
    m_dateTimePosX = 4;
    m_dateTimePosY = 0;
    m_equatorialGrid = false;
    m_equatorialGridColor = QColor(80, 170, 255);
    m_altAzGrid = false;
    m_altAzGridColor = QColor(255, 170, 80);
    m_constellation  = false;
    m_constellationColor = QColor(255, 255, 120);
    m_constellationOverlay = ConstellationOverlayUrsaMajor;
    m_trackObjects = false;
    m_trackObjectTrails = false;
    m_trackObjectHeatMap = false;
    m_trackObjectRange = false;
    m_trackObjectMinElevation = 0.0;
    m_trackObjectColor = QColor(80, 255, 80);
    m_trackObjectFontFamily.clear();
    m_trackObjectFontScale = 9.0;
    m_gridLabelFontFamily.clear();
    m_gridLabelFontScale = 9.0;
    m_overlayText = false;
    m_overlayTextString = QStringLiteral(DEFAULT_OVERLAY_TEXT_STRING);
    m_overlayTextColor = Qt::white;
    m_overlayTextFontFamily.clear();
    m_overlayTextFontScale = 12.0;
    m_overlayTextPosX = 4;
    m_overlayTextPosY = 0;
    m_diffMask = false;
    m_diffThreshold = 30;
    m_diffMaskOpenSize = 0;
    m_dilationSize = 3;
    m_diffMaskHistoryFrames = 1;
    m_diffMaskCloseSize = 0;
    m_overlayFontFamily.clear();
    m_overlayFontScale = 12.0;
    m_detectionRoiX = 0;
    m_detectionRoiY = 0;
    m_detectionRoiWidth = 0;
    m_detectionRoiHeight = 0;
    m_showDetectionRoi = true;
    m_motionDetect = false;
    m_motionBackgroundSubtractor = MotionBackgroundSubtractorMOG2;
    m_motionMaskView = MotionMaskViewOff;
    m_motionHistory = 500;
    m_motionVarThreshold = 16.0;
    m_motionLearningRate = -1.0;
    m_motionConfirmFrames = 1;
    m_motionDownscale = 1.0;
    m_motionDetectShadows = true;
    m_motionOpenSize = 0;
    m_motionCloseSize = 0;
    m_motionPersistenceFrames = 0;
    m_motionBoxColor = Qt::red;
    m_minContourArea = 100;
    m_showMotionExclusionRects = true;
    m_motionExclusionRects.clear();
    m_starDetect = false;
    m_starThreshold = 24;
    m_starBackgroundBlur = 12;
    m_starMinArea = 1;
    m_starMaxArea = 1000;
    m_starMaxAspectRatio = 2.5;
    m_starDebugView = StarDebugViewOff;
    m_starColor = QColor(120, 255, 255);
    m_plateSolve = false;
    m_plateSolveMaxMagnitude = 5.0;
    m_plateSolveMinMatches = 4;
    // NOTE (2026-06-08): investigated lowering this from 24.0 to fix a real misconvergence on
    // ngc-2403.jpg (dense field, ~1000+ detections) where a 24px radius let wrong-pose candidates
    // accidentally accumulate match counts/RMS similar to the correct pose. A global reduction to
    // 8px DID make ngc-2403.jpg converge correctly (217@RMS~17px unsolved -> 380@RMS~0.24px solved),
    // proving the correct pose sits in a separate optimisation basin only reachable by tightening
    // the search/seed stage itself (refinement-only fixes -- a separate discrimination radius, and
    // two progressive radius-annealing schedules -- could not cross into that basin). However, a
    // full 42-test regression at 8px showed this is NOT a viable global default: it causes severe
    // under-matching regressions across ~11 other image types (sparse/wide fields, galaxies,
    // clusters, nebulae -- e.g. stars-wide-1.jpg matched dropped to 0), flipping the suite from
    // PASS=36/FAIL=6 to PASS=19/FAIL=23. Reverted to the original 24.0 default; ngc-2403.jpg
    // remains a known edge-case limitation for unusually dense star fields pending a smarter
    // (e.g. detection-density-adaptive) radius strategy.
    m_plateSolveMatchRadius = 24.0;
    m_plateSolveFinalMatchRadius = 24.0;
    m_plateSolveAzElSearchRadius = 12.0;
    m_plateSolveFovTolerance = 3.0;
    m_plateSolveStartMode = PlateSolveStartFovAzElRoll;
    m_plateSolveLabelMode = PlateSolveLabelName;
    m_plateSolveLabelHideSyntheticNames = false;
    m_showStarDetectionBoxes = true;
    m_plateSolveUseCaptureDateTime = true;
    m_plateSolveDateTime = QDateTime::currentDateTime();
    m_plateSolveDateTimeUtc = false;
    m_plateSolveUseDownloadedCatalog = false;
    m_plateSolveCatalogSource = PlateSolveCatalogAuto;
    m_plateSolveApplyMode = PlateSolveApplyAzElRollFov;
    m_starCatalogDiskCacheSizeGb = 32;
    m_overlaySpectrum = false;
    m_spectrumDevice.clear();
    m_spectrumOffsetX = 0;
    m_spectrumOffsetY = 0;
    m_spectrumScale = 1.0;
    m_yoloEnabled = false;
    m_yoloModelPath.clear();
    m_yoloLabelsPath.clear();
    m_yoloConfThreshold = 0.5;
    m_yoloNmsThreshold = 0.45;
    m_yoloBoxColor = Qt::green;
    m_yoloTileLargeImages = true;
    m_yoloTileOverlapPercent = 20;
    m_yoloIgnoredClassNames.clear();
    m_yoloDnnTarget = CPU;
    m_audioMute = true;
    m_audioPreviewVolume = 100;
    m_audioDeviceName.clear();
    m_whiteBalanceMode = 0;
    m_exposureCompensation = 0.0;
    m_focusMode = 0;
    m_focusDistance = 1.0;
    m_zoomFactor = 1.0;
    m_useReverseAPI = false;
    m_reverseAPIAddress = "127.0.0.1";
    m_reverseAPIPort = 8888;
    m_reverseAPIFeatureSetIndex = 0;
    m_reverseAPIFeatureIndex = 0;
}

QByteArray CameraSettings::serialize() const
{
    SimpleSerializer s(1);

    s.writeString(1, m_title);
    s.writeU32(2, m_rgbColor);
    s.writeString(3, m_cameraProtocol);
    s.writeString(4, m_cameraId);
    s.writeString(5, m_cameraDescription);
    s.writeS32(6, m_resolutionWidth);
    s.writeS32(7, m_resolutionHeight);
    s.writeS32(8, m_framesPerSecond);
    s.writeS32(9, m_captureMode);
    s.writeDouble(10, m_exposureTimeMs);
    s.writeS32(11, m_isoSensitivity);
    s.writeString(12, m_alpacaHost);
    s.writeU32(13, m_alpacaPort);
    s.writeBool(14, m_saveImage);
    s.writeString(15, m_imageFileName);
    s.writeBool(16, m_saveVideo);
    s.writeString(17, m_videoFileName);
    s.writeBool(18, m_videoHwAcceleration);
    s.writeBool(19, m_videoLoop);
    s.writeDouble(20, m_videoPlaybackRate);
    s.writeBool(21, m_stackEnabled);
    s.writeS32(22, m_stackFrameCount);
    s.writeS32(23, m_stackMethod);
    s.writeS32(24, m_stackHdrAlgorithm);
    s.writeS32(25, m_stackHdrExposureCount);
    s.writeDouble(26, m_stackHdrExposureTimesMs[0]);
    s.writeDouble(27, m_stackHdrExposureTimesMs[1]);
    s.writeDouble(28, m_stackHdrExposureTimesMs[2]);
    s.writeDouble(29, m_stackHdrExposureTimesMs[3]);
    s.writeS32(30, m_stackAlignmentMethod);
    s.writeString(31, m_stackDarkFileName);
    s.writeString(32, m_stackFlatFileName);
    s.writeString(33, m_stackBiasFileName);
    s.writeFloat(34, m_latitude);
    s.writeFloat(35, m_longitude);
    s.writeFloat(36, m_altitude);
    s.writeBool(37, m_positionSync);
    s.writeString(38, m_owmAPIKey);
    s.writeFloat(39, m_azimuth);
    s.writeFloat(40, m_elevation);
    s.writeFloat(41, m_roll);
    s.writeString(42, m_rotator);
    s.writeFloat(43, m_fov);
    s.writeS32(44, m_lensProjection);
    s.writeDouble(45, m_lensCenterOffsetX);
    s.writeDouble(46, m_lensCenterOffsetY);
    s.writeDouble(47, m_lensDistortionK1);

    if (m_rollupState) {
        s.writeBlob(48, m_rollupState->serialize());
    }

    s.writeS32(49, m_workspaceIndex);
    s.writeBlob(50, m_geometryBytes);
    s.writeS32(51, m_cameraBinX);
    s.writeS32(52, m_cameraBinY);
    s.writeS32(53, m_cameraGain);
    s.writeS32(54, m_cameraReadoutMode);
    s.writeS32(55, m_cameraOffset);
    s.writeS32(56, m_cameraNumX);
    s.writeS32(57, m_cameraNumY);
    s.writeS32(58, m_cameraStartX);
    s.writeS32(59, m_cameraStartY);
    s.writeS32(60, m_postProcessWhiteBalanceMode);
    s.writeDouble(61, m_postProcessWhiteBalanceRedGain);
    s.writeDouble(62, m_postProcessWhiteBalanceGreenGain);
    s.writeDouble(63, m_postProcessWhiteBalanceBlueGain);
    s.writeBool(64, m_postProcessUnwarp);
    s.writeS32(65, static_cast<qint32>(m_histogramStretch));
    s.writeDouble(66, m_histogramStretchBlackPoint);
    s.writeDouble(67, m_histogramStretchWhitePoint);
    s.writeDouble(68, m_histogramStretchGamma);
    s.writeDouble(69, m_histogramStretchAsinhStrength);
    s.writeDouble(70, m_histogramStretchLogStrength);
    s.writeBool(71, m_postProcessGreyscale);
    s.writeDouble(72, m_saturation);
    s.writeDouble(73, m_gamma);
    s.writeS32(74, m_gaussianBlur);
    s.writeS32(75, m_medianBlur);
    s.writeDouble(76, m_sharpen);
    s.writeS32(77, static_cast<qint32>(m_edgeDisplayMode));
    s.writeDouble(78, m_sobelEdge);
    s.writeDouble(79, m_cannyEdge);
    s.writeBool(80, m_flipX);
    s.writeBool(81, m_flipY);
    s.writeDouble(82, m_brightness);
    s.writeDouble(83, m_contrast);
    s.writeBool(84, m_invertColors);
    s.writeBool(85, m_overlayDateTime);
    s.writeU32(86, m_dateTimeColor.rgba());
    s.writeBool(87, m_diffMask);
    s.writeS32(88, m_diffThreshold);
    s.writeS32(89, m_dilationSize);
    s.writeS32(90, m_diffMaskHistoryFrames);
    s.writeS32(91, m_diffMaskCloseSize);
    s.writeString(92, m_overlayFontFamily);
    s.writeDouble(93, m_overlayFontScale);
    s.writeS32(94, m_detectionRoiX);
    s.writeS32(95, m_detectionRoiY);
    s.writeS32(96, m_detectionRoiWidth);
    s.writeS32(97, m_detectionRoiHeight);
    s.writeBool(98, m_showDetectionRoi);
    s.writeBool(99, m_motionDetect);
    s.writeS32(100, static_cast<qint32>(m_motionBackgroundSubtractor));
    s.writeS32(101, static_cast<qint32>(m_motionMaskView));
    s.writeS32(102, m_motionHistory);
    s.writeDouble(103, m_motionVarThreshold);
    s.writeBool(104, m_motionDetectShadows);
    s.writeS32(105, m_motionOpenSize);
    s.writeS32(106, m_motionCloseSize);
    s.writeS32(107, m_motionPersistenceFrames);
    s.writeU32(108, m_motionBoxColor.rgba());
    s.writeS32(109, m_minContourArea);
    s.writeDouble(110, m_motionLearningRate);
    s.writeS32(111, m_motionConfirmFrames);
    s.writeDouble(112, m_motionDownscale);
    s.writeBlob(113, serializeMotionExclusionRects(m_motionExclusionRects));
    s.writeBool(114, m_starDetect);
    s.writeS32(115, m_starThreshold);
    s.writeS32(116, m_starBackgroundBlur);
    s.writeS32(117, m_starMinArea);
    s.writeS32(118, m_starMaxArea);
    s.writeDouble(119, m_starMaxAspectRatio);
    s.writeS32(120, static_cast<qint32>(m_starDebugView));
    s.writeU32(121, m_starColor.rgba());
    s.writeBool(122, m_plateSolve);
    s.writeDouble(123, m_plateSolveMaxMagnitude);
    s.writeS32(124, m_plateSolveMinMatches);
    s.writeDouble(125, m_plateSolveMatchRadius);
    s.writeDouble(126, m_plateSolveFinalMatchRadius);
    s.writeDouble(127, m_plateSolveAzElSearchRadius);
    s.writeDouble(261, m_plateSolveFovTolerance);
    s.writeS32(128, static_cast<qint32>(m_plateSolveStartMode));
    s.writeBool(129, m_plateSolveUseDownloadedCatalog);
    s.writeBool(130, m_plateSolveUseCaptureDateTime);
    s.writeS64(131, m_plateSolveDateTime.isValid() ? m_plateSolveDateTime.toMSecsSinceEpoch() : 0);
    s.writeS32(132, static_cast<qint32>(m_plateSolveApplyMode));
    s.writeS32(133, static_cast<qint32>(m_plateSolveLabelMode));
    s.writeBool(134, m_overlaySpectrum);
    s.writeString(135, m_spectrumDevice);
    s.writeS32(136, m_spectrumOffsetX);
    s.writeS32(137, m_spectrumOffsetY);
    s.writeDouble(138, m_spectrumScale);
    s.writeString(139, m_dateTimeFormat);
    s.writeS32(140, m_dateTimePosX);
    s.writeS32(141, m_dateTimePosY);
    s.writeBool(142, m_overlayText);
    s.writeString(143, m_overlayTextString);
    s.writeU32(144, m_overlayTextColor.rgba());
    s.writeString(145, m_overlayTextFontFamily);
    s.writeDouble(146, m_overlayTextFontScale);
    s.writeS32(147, m_overlayTextPosX);
    s.writeS32(148, m_overlayTextPosY);
    s.writeBool(149, m_equatorialGrid);
    s.writeU32(150, m_equatorialGridColor.rgba());
    s.writeBool(151, m_altAzGrid);
    s.writeU32(152, m_altAzGridColor.rgba());
    s.writeBool(153, m_constellation);
    s.writeU32(154, m_constellationColor.rgba());
    s.writeS32(155, static_cast<qint32>(m_constellationOverlay));
    s.writeBool(156, m_trackObjects);
    s.writeDouble(157, m_trackObjectMinElevation);
    s.writeU32(158, m_trackObjectColor.rgba());
    s.writeDouble(159, m_trackObjectFontScale);
    s.writeString(160, m_gridLabelFontFamily);
    s.writeDouble(161, m_gridLabelFontScale);
    s.writeBool(162, m_yoloEnabled);
    s.writeString(163, m_yoloModelPath);
    s.writeString(164, m_yoloLabelsPath);
    s.writeDouble(165, m_yoloConfThreshold);
    s.writeDouble(166, m_yoloNmsThreshold);
    s.writeU32(167, m_yoloBoxColor.rgba());
    s.writeS32(169, m_yoloDnnTarget);
    s.writeS32(170, m_audioPreviewVolume);
    s.writeBool(171, m_audioMute);
    s.writeString(172, m_audioDeviceName);
    s.writeS32(173, m_whiteBalanceMode);
    s.writeDouble(174, m_exposureCompensation);
    s.writeS32(175, m_focusMode);
    s.writeDouble(176, m_focusDistance);
    s.writeDouble(177, m_zoomFactor);
    s.writeDouble(178, m_captureInterval);
    s.writeS32(179, m_captureIntervalUnits);
    s.writeBool(180, m_alpacaDiscoveryEnabled);
    s.writeBool(181, m_alpacaApiLogEnabled);
    s.writeBool(182, m_alpacaFocuserEnabled);
    s.writeString(183, m_alpacaFocuserHost);
    s.writeU32(184, m_alpacaFocuserPort);
    s.writeS32(185, m_alpacaFocuserDeviceNumber);
    s.writeS32(186, m_alpacaFocusPosition);
    s.writeS32(187, m_alpacaFocusStepSize);
    s.writeBool(188, m_alpacaFilterWheelEnabled);
    s.writeString(189, m_alpacaFilterWheelHost);
    s.writeU32(190, m_alpacaFilterWheelPort);
    s.writeS32(191, m_alpacaFilterWheelDeviceNumber);
    s.writeS32(192, m_alpacaFilterWheelPosition);
    s.writeString(193, m_videoFileCameraPath);

    s.writeBool(194, m_useReverseAPI);
    s.writeString(195, m_reverseAPIAddress);
    s.writeU32(196, m_reverseAPIPort);
    s.writeU32(197, m_reverseAPIFeatureSetIndex);
    s.writeU32(198, m_reverseAPIFeatureIndex);
    s.writeS32(199, m_asiCoolerOn);
    s.writeS32(200, m_asiTargetTemp);
    s.writeS32(201, m_asiUsbBandwidth);
    s.writeS32(202, m_asiHighSpeedMode);
    s.writeS32(203, m_asiColorImageType);
    s.writeS32(204, m_diffMaskOpenSize);
    s.writeBool(206, m_asiAutoExposureGain);
    s.writeDouble(207, m_postProcessWhiteBalanceHighlightProtection);
    s.writeS32(208, m_videoPreRecordBufferSeconds);
    s.writeS32(209, m_imageRecordLimit);
    s.writeS32(210, m_videoRecordLimitSeconds);
    s.writeBool(211, m_postProcessUseCuda);
    s.writeBool(212, m_plateSolveDateTimeUtc);
    s.writeS32(213, static_cast<qint32>(m_plateSolveCatalogSource));
    s.writeS32(214, m_imageRotation);
    s.writeS32(215, static_cast<qint32>(m_fovMode));
    s.writeDouble(216, m_fovSensorWidthMm);
    s.writeDouble(217, m_fovSensorHeightMm);
    s.writeDouble(218, m_fovFocalLengthMm);
    s.writeBool(219, m_yoloTileLargeImages);
    s.writeS32(220, m_yoloTileOverlapPercent);
    s.writeS32(221, static_cast<qint32>(m_stackDisplayMode));
    s.writeS32(222, m_stackDisplayFrameIndex);
    s.writeBool(223, m_stackRejectBadFrames);
    s.writeBool(224, m_autoExposureGainEnabled);
    s.writeS32(225, static_cast<qint32>(m_autoExposureGainMode));
    s.writeDouble(226, m_autoExposureTargetPercentile);
    s.writeDouble(227, m_autoExposureTargetBrightness);
    s.writeDouble(228, m_autoExposureMaxChangePercent);
    s.writeDouble(229, m_autoExposureMinMs);
    s.writeDouble(230, m_autoExposureMaxMs);
    s.writeS32(231, m_autoExposureMinGain);
    s.writeS32(232, m_autoExposureMaxGain);
    s.writeString(233, serializeStringList(m_imageFileCameraPaths));
    s.writeBool(234, m_recordRawFits);
    s.writeBool(235, m_recordCalibratedMedia);
    s.writeBool(236, m_recordPostProcessedMedia);
    s.writeS32(237, m_starCatalogDiskCacheSizeGb);
    s.writeBool(238, m_trackObjectTrails);
    s.writeBool(239, m_trackObjectHeatMap);
    s.writeBool(240, m_keogramEnabled);
    s.writeString(241, m_keogramFileName);
    s.writeS32(242, static_cast<qint32>(m_keogramDirection));
    s.writeS32(243, static_cast<qint32>(m_keogramDayMode));
    s.writeS32(244, m_keogramSamplePeriodMinutes);
    s.writeBool(245, m_keogramShowPreview);
    s.writeBool(246, m_youtubeStreamEnabled);
    s.writeString(247, m_youtubeStreamUrl);
    s.writeString(248, m_youtubeStreamKey);
    s.writeBool(249, m_youtubeStreamPostProcessed);
    s.writeS32(250, m_youtubeStreamBitrateKbps);
    s.writeS32(251, m_youtubeStreamFps);
    s.writeS32(252, m_youtubeStreamWidth);
    s.writeS32(253, m_youtubeStreamHeight);
    s.writeS32(254, static_cast<qint32>(m_videoCodec));
    s.writeS32(255, m_videoPlaybackAudioOffsetMs);
    s.writeS32(256, m_videoRecordBitrateKbps);
    s.writeString(257, serializeStringList(m_yoloIgnoredClassNames));
    s.writeBool(258, m_recordFilteredMedia);
    s.writeString(259, serializeStringList(m_streamUrlHistory));
    s.writeString(260, m_streamUrl);
    s.writeDouble(262, m_streamBufferingSeconds);
    s.writeBool(263, m_showStarDetectionBoxes);
    s.writeBool(264, m_plateSolveLabelHideSyntheticNames);
    s.writeBool(265, m_scaleEnabled);
    s.writeS32(266, m_scaleWidth);
    s.writeS32(267, m_scaleHeight);
    s.writeBool(268, m_scaleKeepAspectRatio);
    s.writeString(269, m_trackObjectFontFamily);
    s.writeString(270, m_directionSensor);
    s.writeFloat(271, m_azimuthOffset);
    s.writeFloat(272, m_elevationOffset);
    s.writeFloat(273, m_rollOffset);
    s.writeS32(274, static_cast<qint32>(m_scaleJustification));
    s.writeBool(275, m_trackObjectRange);

    return s.final();
}

bool CameraSettings::deserialize(const QByteArray& data)
{
    SimpleDeserializer d(data);

    if (!d.isValid())
    {
        resetToDefaults();
        return false;
    }

    if (d.getVersion() == 1)
    {
        uint32_t utmp;
        QByteArray bytetmp;

        d.readString(1, &m_title, "Camera");
        d.readU32(2, &m_rgbColor, QColor(64, 128, 255).rgb());
        d.readString(3, &m_cameraProtocol, "");
        d.readString(4, &m_cameraId, "");
        d.readString(5, &m_cameraDescription, "");
        d.readS32(6, &m_resolutionWidth, 1280);
        d.readS32(7, &m_resolutionHeight, 720);
        d.readS32(8, &m_framesPerSecond, 10);
        d.readS32(9, (qint32 *) &m_captureMode, (qint32) CaptureModeFrameRate);
        d.readDouble(10, &m_exposureTimeMs, 50.0);
        d.readS32(11, &m_isoSensitivity, -1);
        m_resolutionWidth = std::max(m_minResolution, m_resolutionWidth);
        m_resolutionHeight = std::max(m_minResolution, m_resolutionHeight);
        m_framesPerSecond = std::max(m_minFramesPerSecond, m_framesPerSecond);
        d.readDouble(178, &m_captureInterval, 1.0);
        d.readS32(179, (qint32 *) &m_captureIntervalUnits, (qint32) CaptureIntervalSeconds);
        d.readBool(180, &m_alpacaDiscoveryEnabled, true);
        d.readBool(181, &m_alpacaApiLogEnabled, false);
        d.readBool(182, &m_alpacaFocuserEnabled, false);
        d.readString(183, &m_alpacaFocuserHost, "127.0.0.1");
        d.readU32(184, &utmp, 11111);
        m_alpacaFocuserPort = (utmp <= 65535) ? static_cast<uint16_t>(utmp) : 11111;
        d.readS32(185, &m_alpacaFocuserDeviceNumber, 0);
        d.readS32(186, &m_alpacaFocusPosition, 0);
        d.readS32(187, &m_alpacaFocusStepSize, 100);
        d.readBool(188, &m_alpacaFilterWheelEnabled, false);
        d.readString(189, &m_alpacaFilterWheelHost, "127.0.0.1");
        d.readU32(190, &utmp, 11111);
        m_alpacaFilterWheelPort = (utmp <= 65535) ? static_cast<uint16_t>(utmp) : 11111;
        d.readS32(191, &m_alpacaFilterWheelDeviceNumber, 0);
        d.readS32(192, &m_alpacaFilterWheelPosition, 0);
        m_alpacaFocuserDeviceNumber = std::max(m_minNonNegative, m_alpacaFocuserDeviceNumber);
        m_alpacaFocusPosition = std::max(m_minNonNegative, m_alpacaFocusPosition);
        m_alpacaFocusStepSize = std::max(m_minPositive, m_alpacaFocusStepSize);
        m_alpacaFilterWheelDeviceNumber = std::max(m_minNonNegative, m_alpacaFilterWheelDeviceNumber);
        m_alpacaFilterWheelPosition = std::max(m_minNonNegative, m_alpacaFilterWheelPosition);
        m_captureMode = qBound(CaptureModeFrameRate, m_captureMode, CaptureModeInterval);
        m_captureInterval = std::max(m_minCaptureInterval, m_captureInterval);
        m_captureIntervalUnits = qBound(CaptureIntervalSeconds, m_captureIntervalUnits, CaptureIntervalMinutes);
        m_exposureTimeMs = std::max(m_minExposureTimeMs, m_exposureTimeMs);
        m_isoSensitivity = std::max(m_minIsoSensitivity, m_isoSensitivity);
        d.readString(12, &m_alpacaHost, "127.0.0.1");
        d.readU32(13, &utmp, 11111);
        m_alpacaPort = (utmp <= 65535) ? static_cast<uint16_t>(utmp) : 11111;
        d.readBool(14, &m_saveImage, false);
        d.readString(15, &m_imageFileName, "camera.jpg");
        d.readBool(16, &m_saveVideo, false);
        d.readString(17, &m_videoFileName, "camera.mp4");
        d.readBool(18, &m_videoHwAcceleration, true);
        d.readS32(208, &m_videoPreRecordBufferSeconds, 0);
        m_videoPreRecordBufferSeconds = qBound(0, m_videoPreRecordBufferSeconds, 60);
        d.readS32(209, &m_imageRecordLimit, 0);
        m_imageRecordLimit = std::max(m_minNonNegative, m_imageRecordLimit);
        d.readS32(210, &m_videoRecordLimitSeconds, 0);
        m_videoRecordLimitSeconds = std::max(m_minNonNegative, m_videoRecordLimitSeconds);
        d.readBool(21, &m_stackEnabled, false);
        d.readS32(22, &m_stackFrameCount, 4);
        d.readS32(23, (qint32 *) &m_stackMethod, (qint32) StackMethodAverage);
        d.readS32(24, (qint32 *) &m_stackHdrAlgorithm, (qint32) StackHdrAlgorithmDebevec);
        d.readS32(25, &m_stackHdrExposureCount, 3);
        d.readDouble(26, &m_stackHdrExposureTimesMs[0], 12.5);
        d.readDouble(27, &m_stackHdrExposureTimesMs[1], 50.0);
        d.readDouble(28, &m_stackHdrExposureTimesMs[2], 200.0);
        d.readDouble(29, &m_stackHdrExposureTimesMs[3], 800.0);
        d.readS32(30, (qint32 *) &m_stackAlignmentMethod, (qint32) StackAlignmentNone);
        d.readString(31, &m_stackDarkFileName, "");
        d.readString(32, &m_stackFlatFileName, "");
        d.readString(33, &m_stackBiasFileName, "");
        d.readFloat(34, &m_latitude, MainCore::instance()->getSettings().getLatitude());
        d.readFloat(35, &m_longitude, MainCore::instance()->getSettings().getLongitude());
        d.readFloat(36, &m_altitude, MainCore::instance()->getSettings().getAltitude());
        d.readBool(37, &m_positionSync, false);
        d.readString(38, &m_owmAPIKey, "");
        d.readFloat(39, &m_azimuth, 0.0f);
        d.readFloat(40, &m_elevation, 0.0f);
        d.readFloat(41, &m_roll, 0.0f);
        d.readString(42, &m_rotator, "");
        d.readFloat(43, &m_fov, 60.0f);
        qint32 fovMode = static_cast<qint32>(FovModeDirect);
        d.readS32(215, &fovMode, fovMode);
        m_fovMode = static_cast<FovMode>(qBound(
            static_cast<qint32>(FovModeDirect),
            fovMode,
            static_cast<qint32>(FovModeSensorFocalLength)));
        d.readDouble(216, &m_fovSensorWidthMm, 36.0);
        d.readDouble(217, &m_fovSensorHeightMm, 24.0);
        d.readDouble(218, &m_fovFocalLengthMm, 35.0);
        m_fovSensorWidthMm = std::max(0.001, m_fovSensorWidthMm);
        m_fovSensorHeightMm = std::max(0.001, m_fovSensorHeightMm);
        m_fovFocalLengthMm = std::max(0.001, m_fovFocalLengthMm);
        d.readS32(44, (int *) &m_lensProjection, LensProjectionRectilinear);
        d.readDouble(45, &m_lensCenterOffsetX, 0.0);
        d.readDouble(46, &m_lensCenterOffsetY, 0.0);
        d.readDouble(47, &m_lensDistortionK1, 0.0);

        if (m_rollupState)
        {
            d.readBlob(48, &bytetmp);
            m_rollupState->deserialize(bytetmp);
        }

        d.readS32(49, &m_workspaceIndex, 0);
        d.readBlob(50, &m_geometryBytes);
        d.readS32(51, &m_cameraBinX, 1);
        d.readS32(52, &m_cameraBinY, 1);
        d.readS32(53, &m_cameraGain, 100);
        d.readS32(54, &m_cameraReadoutMode, 0);
        d.readS32(55, &m_cameraOffset, 1);
        d.readS32(56, &m_cameraNumX, 0);
        d.readS32(57, &m_cameraNumY, 0);
        d.readS32(58, &m_cameraStartX, 0);
        d.readS32(59, &m_cameraStartY, 0);
        m_cameraBinX = std::max(m_minPositive, m_cameraBinX);
        m_cameraBinY = std::max(m_minPositive, m_cameraBinY);
        m_cameraNumX = std::max(m_minNonNegative, m_cameraNumX);
        m_cameraNumY = std::max(m_minNonNegative, m_cameraNumY);
        m_cameraStartX = std::max(m_minNonNegative, m_cameraStartX);
        m_cameraStartY = std::max(m_minNonNegative, m_cameraStartY);
        m_cameraReadoutMode = std::max(m_minNonNegative, m_cameraReadoutMode);
        d.readS32(60, &m_postProcessWhiteBalanceMode, 0);
        d.readDouble(61, &m_postProcessWhiteBalanceRedGain, 1.0);
        d.readDouble(62, &m_postProcessWhiteBalanceGreenGain, 1.0);
        d.readDouble(63, &m_postProcessWhiteBalanceBlueGain, 1.0);
        d.readBool(64, &m_postProcessUnwarp, false);
        d.readS32(65, reinterpret_cast<qint32*>(&m_histogramStretch), static_cast<qint32>(HistogramStretchOff));
        d.readDouble(66, &m_histogramStretchBlackPoint, 0.0);
        d.readDouble(67, &m_histogramStretchWhitePoint, 1.0);
        d.readDouble(68, &m_histogramStretchGamma, 1.0);
        d.readDouble(69, &m_histogramStretchAsinhStrength, 10.0);
        d.readDouble(70, &m_histogramStretchLogStrength, 10.0);
        d.readBool(71, &m_postProcessGreyscale, false);
        d.readDouble(72, &m_saturation, 1.0);
        d.readDouble(73, &m_gamma, 1.0);
        d.readS32(74, &m_gaussianBlur, 0);
        d.readS32(75, &m_medianBlur, 0);
        d.readDouble(76, &m_sharpen, 0.0);
        qint32 edgeDisplayMode = static_cast<qint32>(EdgeDisplayOverlay);
        d.readS32(77, &edgeDisplayMode, static_cast<qint32>(EdgeDisplayOverlay));
        m_edgeDisplayMode = static_cast<EdgeDisplayMode>(qBound(
            static_cast<qint32>(EdgeDisplayOverlay),
            edgeDisplayMode,
            static_cast<qint32>(EdgeDisplayEdgesOnly)));
        d.readDouble(78, &m_sobelEdge, 0.0);
        d.readDouble(79, &m_cannyEdge, 0.0);
        d.readBool(80, &m_flipX, false);
        d.readBool(81, &m_flipY, false);
        d.readS32(214, &m_imageRotation, 0);
        m_imageRotation = normalizeImageRotation(m_imageRotation);
        m_postProcessWhiteBalanceMode = qBound(m_minNonNegative, m_postProcessWhiteBalanceMode, 2);
        m_postProcessWhiteBalanceRedGain = qBound(m_minWhiteBalanceGain, m_postProcessWhiteBalanceRedGain, m_maxWhiteBalanceGain);
        m_postProcessWhiteBalanceGreenGain = qBound(m_minWhiteBalanceGain, m_postProcessWhiteBalanceGreenGain, m_maxWhiteBalanceGain);
        m_postProcessWhiteBalanceBlueGain = qBound(m_minWhiteBalanceGain, m_postProcessWhiteBalanceBlueGain, m_maxWhiteBalanceGain);
        m_histogramStretch = qBound(HistogramStretchOff, m_histogramStretch, HistogramStretchCLAHE);
        m_histogramStretchBlackPoint = qBound(m_minNormalized, m_histogramStretchBlackPoint, m_maxNormalized);
        m_histogramStretchWhitePoint = qBound(m_histogramStretchBlackPoint + m_minHistogramWhitePointGap, m_histogramStretchWhitePoint, m_maxNormalized);
        m_histogramStretchGamma = qBound(m_minHistogramStrength, m_histogramStretchGamma, m_maxWhiteBalanceGain);
        m_histogramStretchAsinhStrength = qBound(m_minHistogramStrength, m_histogramStretchAsinhStrength, m_maxHistogramStrength);
        m_histogramStretchLogStrength = qBound(m_minHistogramStrength, m_histogramStretchLogStrength, m_maxHistogramStrength);
        m_saturation = qBound(m_minFilterAmount, m_saturation, m_maxFilterAmount);
        m_gamma = qBound(m_minHistogramStrength, m_gamma, m_maxFilterAmount);
        m_gaussianBlur = qBound(m_minBlurRadius, m_gaussianBlur, m_maxBlurRadius);
        m_medianBlur = qBound(m_minBlurRadius, m_medianBlur, m_maxBlurRadius);
        m_sharpen = qBound(m_minFilterAmount, m_sharpen, m_maxFilterAmount);
        m_edgeDisplayMode = static_cast<EdgeDisplayMode>(qBound(
            static_cast<qint32>(EdgeDisplayOverlay),
            static_cast<qint32>(m_edgeDisplayMode),
            static_cast<qint32>(EdgeDisplayEdgesOnly)));
        m_sobelEdge = qBound(m_minFilterAmount, m_sobelEdge, m_maxFilterAmount);
        m_cannyEdge = qBound(m_minFilterAmount, m_cannyEdge, m_maxFilterAmount);
        d.readDouble(82, &m_brightness, 0.0);
        d.readDouble(83, &m_contrast, 1.0);
        d.readBool(84, &m_invertColors, false);
        d.readBool(85, &m_overlayDateTime, false);
        uint32_t colorRgba = QColor(Qt::white).rgba();
        d.readU32(86, &colorRgba, QColor(Qt::white).rgba());
        m_dateTimeColor = QColor::fromRgba(colorRgba);
        d.readBool(87, &m_diffMask, false);
        d.readS32(88, &m_diffThreshold, 30);
        d.readS32(89, &m_dilationSize, 3);
        d.readS32(90, &m_diffMaskHistoryFrames, 1);
        d.readS32(91, &m_diffMaskCloseSize, 0);
        d.readS32(204, &m_diffMaskOpenSize, 0);
        m_brightness = qBound(m_minBrightness, m_brightness, m_maxBrightness);
        m_contrast = qBound(m_minWhiteBalanceGain, m_contrast, m_maxFilterAmount);
        m_diffThreshold = qBound(m_minThreshold8Bit, m_diffThreshold, m_maxThreshold8Bit);
        m_diffMaskOpenSize = qBound(m_minMorphologyKernel, m_diffMaskOpenSize, m_maxMorphologyKernel);
        m_dilationSize = qBound(m_minMorphologyKernel, m_dilationSize, m_maxMorphologyKernel);
        m_diffMaskHistoryFrames = qBound(m_minShortHistoryFrames, m_diffMaskHistoryFrames, m_maxShortHistoryFrames);
        m_diffMaskCloseSize = qBound(m_minMorphologyKernel, m_diffMaskCloseSize, m_maxMorphologyKernel);

        d.readString(92, &m_overlayFontFamily, "");
        d.readDouble(93, &m_overlayFontScale, 12.0);
        d.readS32(94, &m_detectionRoiX, 0);
        d.readS32(95, &m_detectionRoiY, 0);
        d.readS32(96, &m_detectionRoiWidth, 0);
        d.readS32(97, &m_detectionRoiHeight, 0);
        d.readBool(98, &m_showDetectionRoi, true);
        d.readBool(99, &m_motionDetect, false);
        qint32 motionBackgroundSubtractor = static_cast<qint32>(MotionBackgroundSubtractorMOG2);
        d.readS32(100, &motionBackgroundSubtractor, static_cast<qint32>(MotionBackgroundSubtractorMOG2));
        m_motionBackgroundSubtractor = static_cast<MotionBackgroundSubtractor>(qBound(
            static_cast<qint32>(MotionBackgroundSubtractorMOG2),
            motionBackgroundSubtractor,
            static_cast<qint32>(MotionBackgroundSubtractorKNN)));
        qint32 motionMaskView = static_cast<qint32>(MotionMaskViewOff);
        d.readS32(101, &motionMaskView, static_cast<qint32>(MotionMaskViewOff));
        m_motionMaskView = static_cast<MotionMaskView>(qBound(
            static_cast<qint32>(MotionMaskViewOff),
            motionMaskView,
            static_cast<qint32>(MotionMaskViewFinal)));
        d.readS32(102, &m_motionHistory, 500);
        d.readDouble(103, &m_motionVarThreshold, 16.0);
        d.readDouble(110, &m_motionLearningRate, -1.0);
        d.readS32(111, &m_motionConfirmFrames, 1);
        d.readDouble(112, &m_motionDownscale, 1.0);
        d.readBool(104, &m_motionDetectShadows, true);
        d.readS32(105, &m_motionOpenSize, 0);
        d.readS32(106, &m_motionCloseSize, 0);
        d.readS32(107, &m_motionPersistenceFrames, 0);
        uint32_t motionBoxColorRgba = QColor(Qt::red).rgba();
        d.readU32(108, &motionBoxColorRgba, QColor(Qt::red).rgba());
        m_motionBoxColor = QColor::fromRgba(motionBoxColorRgba);
        d.readS32(109, &m_minContourArea, 100);
        d.readBlob(113, &bytetmp);
        deserializeMotionExclusionRects(bytetmp, m_motionExclusionRects);
        d.readBool(114, &m_starDetect, false);
        d.readS32(115, &m_starThreshold, 24);
        d.readS32(116, &m_starBackgroundBlur, 12);
        d.readS32(117, &m_starMinArea, 1);
        d.readS32(118, &m_starMaxArea, 1000);
        d.readDouble(119, &m_starMaxAspectRatio, 2.5);
        qint32 starDebugView = static_cast<qint32>(StarDebugViewOff);
        d.readS32(120, &starDebugView, static_cast<qint32>(StarDebugViewOff));
        m_starDebugView = static_cast<StarDebugView>(qBound(
            static_cast<qint32>(StarDebugViewOff),
            starDebugView,
            static_cast<qint32>(StarDebugViewFinal)));
        uint32_t starColorRgba = QColor(120, 255, 255).rgba();
        d.readU32(121, &starColorRgba, QColor(120, 255, 255).rgba());
        m_starColor = QColor::fromRgba(starColorRgba);
        d.readBool(122, &m_plateSolve, false);
        d.readDouble(123, &m_plateSolveMaxMagnitude, 5.0);
        d.readS32(124, &m_plateSolveMinMatches, 4);
        d.readDouble(125, &m_plateSolveMatchRadius, 24.0);
        d.readDouble(126, &m_plateSolveFinalMatchRadius, m_plateSolveMatchRadius);
        d.readDouble(127, &m_plateSolveAzElSearchRadius, 12.0);
        d.readDouble(261, &m_plateSolveFovTolerance, 3.0);
        d.readBool(129, &m_plateSolveUseDownloadedCatalog, false);
        d.readBool(130, &m_plateSolveUseCaptureDateTime, true);
        d.readBool(212, &m_plateSolveDateTimeUtc, false);
        qint32 plateSolveCatalogSource = static_cast<qint32>(PlateSolveCatalogAuto);
        d.readS32(213, &plateSolveCatalogSource, plateSolveCatalogSource);
        m_plateSolveCatalogSource = static_cast<PlateSolveCatalogSource>(plateSolveCatalogSource);
        qint64 plateSolveDateTimeMs = QDateTime::currentDateTime().toMSecsSinceEpoch();
        d.readS64(131, &plateSolveDateTimeMs, plateSolveDateTimeMs);
        m_plateSolveDateTime = QDateTime::fromMSecsSinceEpoch(
            std::max(plateSolveDateTimeMs, m_minPlateSolveDateTimeMs),
            m_plateSolveDateTimeUtc ? Qt::UTC : Qt::LocalTime);
        qint32 plateSolveApplyMode = static_cast<qint32>(PlateSolveApplyAzElRollFov);
        d.readS32(132, &plateSolveApplyMode, static_cast<qint32>(PlateSolveApplyAzElRollFov));
        m_plateSolveApplyMode = static_cast<PlateSolveApplyMode>(plateSolveApplyMode);
        qint32 plateSolveStartMode = static_cast<qint32>(PlateSolveStartFovAzElRoll);
        d.readS32(128, &plateSolveStartMode, plateSolveStartMode);
        m_plateSolveStartMode = static_cast<PlateSolveStartMode>(plateSolveStartMode);
        qint32 plateSolveLabelMode = static_cast<qint32>(PlateSolveLabelName);
        d.readS32(133, &plateSolveLabelMode, static_cast<qint32>(PlateSolveLabelName));
        m_plateSolveLabelMode = static_cast<PlateSolveLabelMode>(plateSolveLabelMode);
        m_overlayFontScale = qBound(m_minOverlayFontScale, m_overlayFontScale, m_maxOverlayFontScale);
        m_detectionRoiX = qBound(m_minUiPixelOffset, m_detectionRoiX, m_maxUiPixelOffset);
        m_detectionRoiY = qBound(m_minUiPixelOffset, m_detectionRoiY, m_maxUiPixelOffset);
        m_detectionRoiWidth = qBound(m_minUiPixelOffset, m_detectionRoiWidth, m_maxUiPixelOffset);
        m_detectionRoiHeight = qBound(m_minUiPixelOffset, m_detectionRoiHeight, m_maxUiPixelOffset);
        m_starThreshold = qBound(m_minThreshold8Bit, m_starThreshold, m_maxThreshold8Bit);
        m_starBackgroundBlur = qBound(m_minStarBackgroundBlur, m_starBackgroundBlur, m_maxStarBackgroundBlur);
        m_starMinArea = qBound(m_minContourAreaBound, m_starMinArea, m_maxContourAreaBound);
        m_starMaxArea = qBound(m_starMinArea, m_starMaxArea, m_maxContourAreaBound);
        m_starMaxAspectRatio = qBound(m_minStarAspectRatio, m_starMaxAspectRatio, m_maxStarAspectRatio);
        m_plateSolveMaxMagnitude = qBound(m_minPlateSolveMagnitude, m_plateSolveMaxMagnitude, m_maxPlateSolveMagnitude);
        m_plateSolveMinMatches = qBound(m_minPlateSolveMatches, m_plateSolveMinMatches, m_maxPlateSolveMatches);
        m_plateSolveMatchRadius = qBound(m_minPlateSolveMatchRadius, m_plateSolveMatchRadius, m_maxPlateSolveMatchRadius);
        m_plateSolveFinalMatchRadius = qBound(m_minPlateSolveMatchRadius, m_plateSolveFinalMatchRadius, m_maxPlateSolveMatchRadius);
        m_plateSolveAzElSearchRadius = qBound(m_minPlateSolveAzElSearchRadius, m_plateSolveAzElSearchRadius, m_maxPlateSolveAzElSearchRadius);
        m_plateSolveFovTolerance = qBound(m_minPlateSolveFovTolerance, m_plateSolveFovTolerance, m_maxPlateSolveFovTolerance);
        m_plateSolveApplyMode = static_cast<PlateSolveApplyMode>(qBound(0, static_cast<int>(m_plateSolveApplyMode), 3));
        m_plateSolveStartMode = static_cast<PlateSolveStartMode>(qBound(0, static_cast<int>(m_plateSolveStartMode), 6));
        m_plateSolveLabelMode = static_cast<PlateSolveLabelMode>(qBound(0, static_cast<int>(m_plateSolveLabelMode), 3));
        m_plateSolveCatalogSource = static_cast<PlateSolveCatalogSource>(qBound(0, static_cast<int>(m_plateSolveCatalogSource), 3));
        m_lensCenterOffsetX = qBound(m_minLensCenterOffset, m_lensCenterOffsetX, m_maxLensCenterOffset);
        m_lensCenterOffsetY = qBound(m_minLensCenterOffset, m_lensCenterOffsetY, m_maxLensCenterOffset);
        m_lensDistortionK1 = qBound(m_minLensDistortionK1, m_lensDistortionK1, m_maxLensDistortionK1);
        if (!m_plateSolveDateTime.isValid()) {
            m_plateSolveDateTime = m_plateSolveDateTimeUtc ? QDateTime::currentDateTimeUtc() : QDateTime::currentDateTime();
        }
        m_motionHistory = qBound(m_minMotionHistory, m_motionHistory, m_maxMotionHistory);
        m_motionVarThreshold = qBound(m_minMotionVarThreshold, m_motionVarThreshold, m_maxMotionVarThreshold);
        m_motionLearningRate = qBound(m_minLearningRate, m_motionLearningRate, m_maxLearningRate);
        m_motionConfirmFrames = qBound(m_minMotionConfirmFrames, m_motionConfirmFrames, m_maxMotionConfirmFrames);
        const QList<double> validDownscales{1.0, 0.5, 0.25};
        if (!validDownscales.contains(m_motionDownscale)) {
            m_motionDownscale = 1.0;
        }
        m_motionOpenSize = qBound(m_minMorphologyKernel, m_motionOpenSize, m_maxMorphologyKernel);
        m_motionCloseSize = qBound(m_minMorphologyKernel, m_motionCloseSize, m_maxMorphologyKernel);
        m_motionPersistenceFrames = qBound(m_minNonNegative, m_motionPersistenceFrames, m_maxShortHistoryFrames);
        m_minContourArea = qBound(m_minContourAreaBound, m_minContourArea, m_maxContourAreaBound);
        d.readBool(234, &m_recordRawFits, false);
        d.readBool(235, &m_recordCalibratedMedia, true);
        d.readBool(236, &m_recordPostProcessedMedia, false);
        d.readBool(258, &m_recordFilteredMedia, false);
        d.readBool(240, &m_keogramEnabled, false);
        d.readString(241, &m_keogramFileName, "keogram.png");
        qint32 keogramDirection = static_cast<qint32>(KeogramVertical);
        qint32 keogramDayMode = static_cast<qint32>(KeogramMidnightToMidnight);
        d.readS32(242, &keogramDirection, static_cast<qint32>(KeogramVertical));
        d.readS32(243, &keogramDayMode, static_cast<qint32>(KeogramMidnightToMidnight));
        m_keogramDirection = static_cast<KeogramDirection>(keogramDirection);
        m_keogramDayMode = static_cast<KeogramDayMode>(keogramDayMode);
        d.readS32(244, &m_keogramSamplePeriodMinutes, 5);
        d.readBool(245, &m_keogramShowPreview, false);
        d.readBool(246, &m_youtubeStreamEnabled, false);
        d.readString(247, &m_youtubeStreamUrl, "rtmps://a.rtmps.youtube.com/live2");
        d.readString(248, &m_youtubeStreamKey, "");
        d.readBool(249, &m_youtubeStreamPostProcessed, true);
        d.readS32(250, &m_youtubeStreamBitrateKbps, 6800);
        d.readS32(251, &m_youtubeStreamFps, 25);
        d.readS32(252, &m_youtubeStreamWidth, 0);
        d.readS32(253, &m_youtubeStreamHeight, 0);
        qint32 videoCodec = static_cast<qint32>(VideoCodecH264);
        d.readS32(254, &videoCodec, static_cast<qint32>(VideoCodecH264));
        m_videoCodec = static_cast<VideoCodec>(videoCodec);
        d.readS32(255, &m_videoPlaybackAudioOffsetMs, 0);
        d.readS32(256, &m_videoRecordBitrateKbps, 0);
        d.readDouble(207, &m_postProcessWhiteBalanceHighlightProtection, 0.0);
        m_postProcessWhiteBalanceHighlightProtection = qBound(m_minNormalized, m_postProcessWhiteBalanceHighlightProtection, m_maxNormalized);
        d.readBool(211, &m_postProcessUseCuda, false);
        d.readBool(134, &m_overlaySpectrum, false);
        d.readString(135, &m_spectrumDevice, "");
        d.readS32(136, &m_spectrumOffsetX, 0);
        d.readS32(137, &m_spectrumOffsetY, 0);
        m_spectrumOffsetX = qBound(m_minSignedUiPixelOffset, m_spectrumOffsetX, m_maxSignedUiPixelOffset);
        m_spectrumOffsetY = qBound(m_minSignedUiPixelOffset, m_spectrumOffsetY, m_maxSignedUiPixelOffset);
        d.readDouble(138, &m_spectrumScale, 1.0);
        m_spectrumScale = qBound(m_minSpectrumScale, m_spectrumScale, m_maxSpectrumScale);
        d.readString(139, &m_dateTimeFormat, "yyyy-MM-dd hh:mm:ss");
        d.readS32(140, &m_dateTimePosX, 4);
        d.readS32(141, &m_dateTimePosY, 0);
        m_dateTimePosX = qBound(m_minUiPixelOffset, m_dateTimePosX, m_maxUiPixelOffset);
        m_dateTimePosY = qBound(m_minUiPixelOffset, m_dateTimePosY, m_maxUiPixelOffset);
        d.readBool(149, &m_equatorialGrid, false);
        uint32_t equatorialGridColorRgba = QColor(80, 170, 255).rgba();
        d.readU32(150, &equatorialGridColorRgba, QColor(80, 170, 255).rgba());
        m_equatorialGridColor = QColor::fromRgba(equatorialGridColorRgba);
        d.readBool(151, &m_altAzGrid, false);
        uint32_t altAzGridColorRgba = QColor(255, 170, 80).rgba();
        d.readU32(152, &altAzGridColorRgba, QColor(255, 170, 80).rgba());
        m_altAzGridColor = QColor::fromRgba(altAzGridColorRgba);
        d.readBool(153, &m_constellation, false);
        uint32_t constellationColorRgba = QColor(255, 255, 120).rgba();
        d.readU32(154, &constellationColorRgba, QColor(255, 255, 120).rgba());
        m_constellationColor = QColor::fromRgba(constellationColorRgba);
        d.readS32(155, reinterpret_cast<qint32*>(&m_constellationOverlay), static_cast<qint32>(ConstellationOverlayUrsaMajor));
        d.readBool(156, &m_trackObjects, false);
        d.readBool(238, &m_trackObjectTrails, false);
        d.readBool(239, &m_trackObjectHeatMap, false);
        d.readBool(275, &m_trackObjectRange, false);
        d.readDouble(157, &m_trackObjectMinElevation, 0.0);
        m_trackObjectMinElevation = qBound(m_minNormalized, m_trackObjectMinElevation, static_cast<double>(m_maxElevation));
        uint32_t trackObjectColorRgba = QColor(80, 255, 80).rgba();
        d.readU32(158, &trackObjectColorRgba, QColor(80, 255, 80).rgba());
        m_trackObjectColor = QColor::fromRgba(trackObjectColorRgba);
        d.readDouble(159, &m_trackObjectFontScale, 9.0);
        m_trackObjectFontScale = qBound(m_minOverlayFontScale, m_trackObjectFontScale, m_maxOverlayFontScale);
        d.readString(160, &m_gridLabelFontFamily, "");
        d.readDouble(161, &m_gridLabelFontScale, 9.0);
        m_gridLabelFontScale = qBound(m_minOverlayFontScale, m_gridLabelFontScale, m_maxOverlayFontScale);
        d.readBool(142, &m_overlayText, false);
        d.readString(143, &m_overlayTextString, DEFAULT_OVERLAY_TEXT_STRING);
        uint32_t overlayTextColorRgba = QColor(Qt::white).rgba();
        d.readU32(144, &overlayTextColorRgba, QColor(Qt::white).rgba());
        m_overlayTextColor = QColor::fromRgba(overlayTextColorRgba);
        d.readString(145, &m_overlayTextFontFamily, "");
        d.readDouble(146, &m_overlayTextFontScale, 12.0);
        m_overlayTextFontScale = qBound(m_minOverlayFontScale, m_overlayTextFontScale, m_maxOverlayFontScale);
        d.readS32(147, &m_overlayTextPosX, 4);
        d.readS32(148, &m_overlayTextPosY, 0);
        m_overlayTextPosX = qBound(m_minUiPixelOffset, m_overlayTextPosX, m_maxUiPixelOffset);
        m_overlayTextPosY = qBound(m_minUiPixelOffset, m_overlayTextPosY, m_maxUiPixelOffset);
        d.readBool(162, &m_yoloEnabled, false);
        d.readString(163, &m_yoloModelPath, "");
        d.readString(164, &m_yoloLabelsPath, "");
        d.readDouble(165, &m_yoloConfThreshold, 0.5);
        d.readDouble(166, &m_yoloNmsThreshold, 0.45);
        m_yoloConfThreshold = qBound(m_minNormalized, m_yoloConfThreshold, m_maxNormalized);
        m_yoloNmsThreshold = qBound(m_minNormalized, m_yoloNmsThreshold, m_maxNormalized);
        uint32_t yoloBoxColorRgba = QColor(Qt::green).rgba();
        d.readU32(167, &yoloBoxColorRgba, QColor(Qt::green).rgba());
        m_yoloBoxColor = QColor::fromRgba(yoloBoxColorRgba);
        d.readS32(169, (qint32 *) &m_yoloDnnTarget, (qint32) CPU);

        d.readBool(171, &m_audioMute, true);
        d.readS32(170, &m_audioPreviewVolume, 100);
        m_audioPreviewVolume = qBound(0, m_audioPreviewVolume, 100);
        d.readString(172, &m_audioDeviceName, "");
        d.readS32(173, &m_whiteBalanceMode, 0);
        m_whiteBalanceMode = std::max(m_minNonNegative, m_whiteBalanceMode);
        d.readDouble(174, &m_exposureCompensation, 0.0);
        m_exposureCompensation = qBound(m_minExposureCompensation, m_exposureCompensation, m_maxExposureCompensation);
        d.readS32(175, &m_focusMode, 0);
        m_focusMode = std::max(m_minNonNegative, m_focusMode);
        d.readDouble(176, &m_focusDistance, 1.0);
        m_focusDistance = qBound(m_minNormalized, m_focusDistance, m_maxNormalized);
        d.readDouble(177, &m_zoomFactor, 1.0);
        m_zoomFactor = std::max(m_minZoomFactor, m_zoomFactor);

        d.readString(193, &m_videoFileCameraPath, "");
        QString imageFileCameraPathsJson;
        d.readString(233, &imageFileCameraPathsJson, "");
        m_imageFileCameraPaths = deserializeStringList(imageFileCameraPathsJson);
        d.readBool(19, &m_videoLoop, false);
        d.readDouble(20, &m_videoPlaybackRate, 1.0);
        m_videoPlaybackRate = qBound(m_minVideoPlaybackRate, m_videoPlaybackRate, m_maxVideoPlaybackRate);

        d.readBool(194, &m_useReverseAPI);
        d.readString(195, &m_reverseAPIAddress);
        d.readU32(196, &utmp, 0);

        if ((utmp > 1023) && (utmp < 65535)) {
            m_reverseAPIPort = utmp;
        } else {
            m_reverseAPIPort = 8888;
        }
        d.readU32(197, &utmp, 0);
        m_reverseAPIFeatureSetIndex = utmp > 99 ? 99 : utmp;
        d.readU32(198, &utmp, 0);
        m_reverseAPIFeatureIndex = utmp > 99 ? 99 : utmp;
        d.readS32(199, &m_asiCoolerOn, -1);
        d.readS32(200, &m_asiTargetTemp, std::numeric_limits<int>::min());
        d.readS32(201, &m_asiUsbBandwidth, -1);
        d.readS32(202, &m_asiHighSpeedMode, -1);
        d.readS32(203, (qint32 *) &m_asiColorImageType, (qint32) AsiColorImageTypeRgb24);
        d.readBool(206, &m_asiAutoExposureGain, false);
        d.readBool(219, &m_yoloTileLargeImages, true);
        d.readS32(220, &m_yoloTileOverlapPercent, 20);
        m_yoloTileOverlapPercent = qBound(0, m_yoloTileOverlapPercent, 90);
        QString yoloIgnoredClassNamesJson;
        d.readString(257, &yoloIgnoredClassNamesJson, "");
        m_yoloIgnoredClassNames = deserializeStringList(yoloIgnoredClassNamesJson);
        QString streamUrlHistoryJson;
        d.readString(259, &streamUrlHistoryJson, "");
        m_streamUrlHistory = deserializeStringList(streamUrlHistoryJson);
        d.readString(260, &m_streamUrl, "");
        d.readDouble(262, &m_streamBufferingSeconds, 1.0);
        d.readBool(263, &m_showStarDetectionBoxes, true);
        d.readBool(264, &m_plateSolveLabelHideSyntheticNames, false);
        d.readBool(265, &m_scaleEnabled, false);
        d.readS32(266, &m_scaleWidth, 0);
        d.readS32(267, &m_scaleHeight, 0);
        d.readBool(268, &m_scaleKeepAspectRatio, true);
        d.readString(269, &m_trackObjectFontFamily, "");
        d.readString(270, &m_directionSensor, "");
        d.readFloat(271, &m_azimuthOffset, 0.0f);
        d.readFloat(272, &m_elevationOffset, 0.0f);
        d.readFloat(273, &m_rollOffset, 0.0f);
        qint32 scaleJustification = static_cast<qint32>(ScaleJustifyCenter);
        d.readS32(274, &scaleJustification, scaleJustification);
        m_scaleJustification = static_cast<ScaleJustification>(qBound(
            static_cast<qint32>(ScaleJustifyCenter),
            scaleJustification,
            static_cast<qint32>(ScaleJustifyBottom)));
        if (isStreamCamera() && m_streamUrl.isEmpty()) {
            m_streamUrl = m_videoFileCameraPath;
        }
        m_yoloDnnTarget = qBound(CPU, m_yoloDnnTarget, TensorRT_FP16);
        d.readS32(221, reinterpret_cast<qint32*>(&m_stackDisplayMode), static_cast<qint32>(StackDisplayStacked));
        d.readS32(222, &m_stackDisplayFrameIndex, 0);
        d.readBool(223, &m_stackRejectBadFrames, false);
        d.readBool(224, &m_autoExposureGainEnabled, false);
        d.readS32(225, reinterpret_cast<qint32*>(&m_autoExposureGainMode), static_cast<qint32>(AutoExposureGainExposureFirst));
        d.readDouble(226, &m_autoExposureTargetPercentile, 95.0);
        d.readDouble(227, &m_autoExposureTargetBrightness, 70.0);
        d.readDouble(228, &m_autoExposureMaxChangePercent, 25.0);
        d.readDouble(229, &m_autoExposureMinMs, m_minExposureTimeMs);
        d.readDouble(230, &m_autoExposureMaxMs, 60000.0);
        d.readS32(231, &m_autoExposureMinGain, 0);
        d.readS32(232, &m_autoExposureMaxGain, 100);
        d.readS32(237, &m_starCatalogDiskCacheSizeGb, 32);
        m_starCatalogDiskCacheSizeGb = qBound(
            m_minStarCatalogDiskCacheSizeGb,
            m_starCatalogDiskCacheSizeGb,
            m_maxStarCatalogDiskCacheSizeGb);
        m_asiCoolerOn = qBound(m_minAsiControl, m_asiCoolerOn, m_maxAsiControl);
        m_asiUsbBandwidth = std::max(m_minAsiControl, m_asiUsbBandwidth);
        m_asiHighSpeedMode = qBound(m_minAsiControl, m_asiHighSpeedMode, m_maxAsiControl);
        m_asiColorImageType = qBound(AsiColorImageTypeRgb24, m_asiColorImageType, AsiColorImageTypeRaw8);
        m_autoExposureGainMode = qBound(AutoExposureGainExposureFirst, m_autoExposureGainMode, AutoExposureGainGainOnly);
        m_autoExposureTargetPercentile = qBound(50.0, m_autoExposureTargetPercentile, 99.9);
        m_autoExposureTargetBrightness = qBound(1.0, m_autoExposureTargetBrightness, 99.0);
        m_autoExposureMaxChangePercent = qBound(1.0, m_autoExposureMaxChangePercent, 100.0);
        m_autoExposureMinMs = std::max(m_minExposureTimeMs, m_autoExposureMinMs);
        m_autoExposureMaxMs = std::max(m_autoExposureMinMs, m_autoExposureMaxMs);
        m_autoExposureMinGain = std::max(0, m_autoExposureMinGain);
        m_autoExposureMaxGain = std::max(m_autoExposureMinGain, m_autoExposureMaxGain);
        m_stackFrameCount = qBound(m_minStackFrameCount, m_stackFrameCount, m_maxStackFrameCount);
        m_stackMethod = qBound(StackMethodAverage, m_stackMethod, StackMethodHDR);
        m_stackHdrAlgorithm = qBound(StackHdrAlgorithmDebevec, m_stackHdrAlgorithm, StackHdrAlgorithmMertens);
        m_stackDisplayMode = qBound(StackDisplayStacked, m_stackDisplayMode, StackDisplayHistoryTiles);
        m_stackDisplayFrameIndex = qBound(0, m_stackDisplayFrameIndex, m_maxStackFrameCount - 1);
        m_stackHdrExposureCount = qBound(m_minHdrExposureCount, m_stackHdrExposureCount, m_maxHdrExposureCount);
        for (double& exposureTimeMs : m_stackHdrExposureTimesMs) {
            exposureTimeMs = std::max(m_minExposureTimeMs, exposureTimeMs);
        }
        m_stackAlignmentMethod = qBound(StackAlignmentNone, m_stackAlignmentMethod, StackAlignmentStarCentroidMatching);
        m_scaleWidth = qBound(0, m_scaleWidth, 65535);
        m_scaleHeight = qBound(0, m_scaleHeight, 65535);
        m_keogramDirection = static_cast<KeogramDirection>(qBound(0, static_cast<int>(m_keogramDirection), 1));
        m_keogramDayMode = static_cast<KeogramDayMode>(qBound(0, static_cast<int>(m_keogramDayMode), 1));
        m_keogramSamplePeriodMinutes = qBound(1, m_keogramSamplePeriodMinutes, 1440);
        m_youtubeStreamBitrateKbps = qBound(100, m_youtubeStreamBitrateKbps, 240000);
        m_youtubeStreamFps = qBound(1, m_youtubeStreamFps, 120);
        m_youtubeStreamWidth = qBound(0, m_youtubeStreamWidth, 16384);
        m_youtubeStreamHeight = qBound(0, m_youtubeStreamHeight, 16384);
        m_videoCodec = static_cast<VideoCodec>(qBound(0, static_cast<int>(m_videoCodec), 1));
        m_videoRecordBitrateKbps = qBound(0, m_videoRecordBitrateKbps, 240000);
        m_videoPlaybackAudioOffsetMs = qBound(
            m_minVideoPlaybackAudioOffsetMs,
            m_videoPlaybackAudioOffsetMs,
            m_maxVideoPlaybackAudioOffsetMs);
        m_streamBufferingSeconds = qBound(
            m_minStreamBufferingSeconds,
            m_streamBufferingSeconds,
            m_maxStreamBufferingSeconds);
        m_latitude = qBound(m_minLatitude, m_latitude, m_maxLatitude);
        m_longitude = qBound(m_minLongitude, m_longitude, m_maxLongitude);
        m_altitude = qBound(m_minAltitude, m_altitude, m_maxAltitude);
        m_azimuth = normalizePositiveDegrees(m_azimuth);
        m_elevation = qBound(m_minElevation, m_elevation, m_maxElevation);
        m_roll = normalizeSignedDegrees(m_roll);
        m_azimuthOffset = normalizeSignedDegrees(m_azimuthOffset);
        m_elevationOffset = qBound(-180.0f, m_elevationOffset, 180.0f);
        m_rollOffset = normalizeSignedDegrees(m_rollOffset);
        if (m_fovMode == FovModeSensorFocalLength) {
            m_fov = calculateLongEdgeFovDegrees(m_fovSensorWidthMm, m_fovSensorHeightMm, m_fovFocalLengthMm);
        }
        m_fov = qBound(m_minFov, m_fov, m_maxFov);
        m_lensProjection = (LensProjection) qBound((int) LensProjectionRectilinear, (int) m_lensProjection, (int) LensProjectionEquisolid);

        return true;
    }

    resetToDefaults();
    return false;
}

QByteArray CameraSettings::serializeMotionExclusionRects(const QList<QRect>& rects) const
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);
    stream << rects;
    return data;
}

void CameraSettings::deserializeMotionExclusionRects(const QByteArray& data, QList<QRect>& rects)
{
    if (data.isEmpty()) {
        rects.clear();
        return;
    }

    QDataStream stream(data);
    stream >> rects;
}

void CameraSettings::applySettings(const QStringList& settingsKeys, const CameraSettings& settings)
{
    if (settingsKeys.contains("title")) {
        m_title = settings.m_title;
    }
    if (settingsKeys.contains("rgbColor")) {
        m_rgbColor = settings.m_rgbColor;
    }
    if (settingsKeys.contains("cameraProtocol")) {
        m_cameraProtocol = settings.m_cameraProtocol;
    }
    if (settingsKeys.contains("cameraId")) {
        m_cameraId = settings.m_cameraId;
    }
    if (settingsKeys.contains("cameraDescription")) {
        m_cameraDescription = settings.m_cameraDescription;
    }
    if (settingsKeys.contains("resolutionWidth")) {
        m_resolutionWidth = std::max(m_minResolution, settings.m_resolutionWidth);
    }
    if (settingsKeys.contains("resolutionHeight")) {
        m_resolutionHeight = std::max(m_minResolution, settings.m_resolutionHeight);
    }
    if (settingsKeys.contains("framesPerSecond")) {
        m_framesPerSecond = std::max(m_minFramesPerSecond, settings.m_framesPerSecond);
    }
    if (settingsKeys.contains("captureMode")) {
        m_captureMode = qBound(CaptureModeFrameRate, settings.m_captureMode, CaptureModeInterval);
    }
    if (settingsKeys.contains("captureInterval")) {
        m_captureInterval = std::max(m_minCaptureInterval, settings.m_captureInterval);
    }
    if (settingsKeys.contains("captureIntervalUnits")) {
        m_captureIntervalUnits = qBound(CaptureIntervalSeconds, settings.m_captureIntervalUnits, CaptureIntervalMinutes);
    }
    if (settingsKeys.contains("exposureTimeMs")) {
        m_exposureTimeMs = std::max(m_minExposureTimeMs, settings.m_exposureTimeMs);
    }
    if (settingsKeys.contains("isoSensitivity")) {
        m_isoSensitivity = std::max(m_minIsoSensitivity, settings.m_isoSensitivity);
    }
    if (settingsKeys.contains("alpacaDiscoveryEnabled")) {
        m_alpacaDiscoveryEnabled = settings.m_alpacaDiscoveryEnabled;
    }
    if (settingsKeys.contains("alpacaApiLogEnabled")) {
        m_alpacaApiLogEnabled = settings.m_alpacaApiLogEnabled;
    }
    if (settingsKeys.contains("alpacaHost")) {
        m_alpacaHost = settings.m_alpacaHost;
    }
    if (settingsKeys.contains("alpacaPort")) {
        m_alpacaPort = settings.m_alpacaPort;
    }
    if (settingsKeys.contains("alpacaFocuserEnabled")) {
        m_alpacaFocuserEnabled = settings.m_alpacaFocuserEnabled;
    }
    if (settingsKeys.contains("alpacaFocuserHost")) {
        m_alpacaFocuserHost = settings.m_alpacaFocuserHost;
    }
    if (settingsKeys.contains("alpacaFocuserPort")) {
        m_alpacaFocuserPort = settings.m_alpacaFocuserPort;
    }
    if (settingsKeys.contains("alpacaFocuserDeviceNumber")) {
        m_alpacaFocuserDeviceNumber = std::max(m_minNonNegative, settings.m_alpacaFocuserDeviceNumber);
    }
    if (settingsKeys.contains("alpacaFocusPosition")) {
        m_alpacaFocusPosition = std::max(m_minNonNegative, settings.m_alpacaFocusPosition);
    }
    if (settingsKeys.contains("alpacaFocusStepSize")) {
        m_alpacaFocusStepSize = std::max(m_minPositive, settings.m_alpacaFocusStepSize);
    }
    if (settingsKeys.contains("alpacaFilterWheelEnabled")) {
        m_alpacaFilterWheelEnabled = settings.m_alpacaFilterWheelEnabled;
    }
    if (settingsKeys.contains("alpacaFilterWheelHost")) {
        m_alpacaFilterWheelHost = settings.m_alpacaFilterWheelHost;
    }
    if (settingsKeys.contains("alpacaFilterWheelPort")) {
        m_alpacaFilterWheelPort = settings.m_alpacaFilterWheelPort;
    }
    if (settingsKeys.contains("alpacaFilterWheelDeviceNumber")) {
        m_alpacaFilterWheelDeviceNumber = std::max(m_minNonNegative, settings.m_alpacaFilterWheelDeviceNumber);
    }
    if (settingsKeys.contains("alpacaFilterWheelPosition")) {
        m_alpacaFilterWheelPosition = std::max(m_minNonNegative, settings.m_alpacaFilterWheelPosition);
    }
    if (settingsKeys.contains("cameraBinX")) {
        m_cameraBinX = std::max(m_minPositive, settings.m_cameraBinX);
    }
    if (settingsKeys.contains("cameraBinY")) {
        m_cameraBinY = std::max(m_minPositive, settings.m_cameraBinY);
    }
    if (settingsKeys.contains("cameraNumX")) {
        m_cameraNumX = std::max(m_minNonNegative, settings.m_cameraNumX);
    }
    if (settingsKeys.contains("cameraNumY")) {
        m_cameraNumY = std::max(m_minNonNegative, settings.m_cameraNumY);
    }
    if (settingsKeys.contains("cameraStartX")) {
        m_cameraStartX = std::max(m_minNonNegative, settings.m_cameraStartX);
    }
    if (settingsKeys.contains("cameraStartY")) {
        m_cameraStartY = std::max(m_minNonNegative, settings.m_cameraStartY);
    }
    if (settingsKeys.contains("cameraGain")) {
        m_cameraGain = settings.m_cameraGain;
    }
    if (settingsKeys.contains("cameraOffset")) {
        m_cameraOffset = settings.m_cameraOffset;
    }
    if (settingsKeys.contains("cameraReadoutMode")) {
        m_cameraReadoutMode = std::max(m_minNonNegative, settings.m_cameraReadoutMode);
    }
    if (settingsKeys.contains("asiCoolerOn")) {
        m_asiCoolerOn = qBound(m_minAsiControl, settings.m_asiCoolerOn, m_maxAsiControl);
    }
    if (settingsKeys.contains("asiTargetTemp")) {
        m_asiTargetTemp = settings.m_asiTargetTemp;
    }
    if (settingsKeys.contains("asiUsbBandwidth")) {
        m_asiUsbBandwidth = std::max(m_minAsiControl, settings.m_asiUsbBandwidth);
    }
    if (settingsKeys.contains("asiHighSpeedMode")) {
        m_asiHighSpeedMode = qBound(m_minAsiControl, settings.m_asiHighSpeedMode, m_maxAsiControl);
    }
    if (settingsKeys.contains("asiAutoExposureGain")) {
        m_asiAutoExposureGain = settings.m_asiAutoExposureGain;
    }
    if (settingsKeys.contains("autoExposureGainEnabled")) {
        m_autoExposureGainEnabled = settings.m_autoExposureGainEnabled;
    }
    if (settingsKeys.contains("autoExposureGainMode")) {
        m_autoExposureGainMode = qBound(AutoExposureGainExposureFirst, settings.m_autoExposureGainMode, AutoExposureGainGainOnly);
    }
    if (settingsKeys.contains("autoExposureTargetPercentile")) {
        m_autoExposureTargetPercentile = qBound(50.0, settings.m_autoExposureTargetPercentile, 99.9);
    }
    if (settingsKeys.contains("autoExposureTargetBrightness")) {
        m_autoExposureTargetBrightness = qBound(1.0, settings.m_autoExposureTargetBrightness, 99.0);
    }
    if (settingsKeys.contains("autoExposureMaxChangePercent")) {
        m_autoExposureMaxChangePercent = qBound(1.0, settings.m_autoExposureMaxChangePercent, 100.0);
    }
    if (settingsKeys.contains("autoExposureMinMs")) {
        m_autoExposureMinMs = std::max(m_minExposureTimeMs, settings.m_autoExposureMinMs);
        m_autoExposureMaxMs = std::max(m_autoExposureMinMs, m_autoExposureMaxMs);
    }
    if (settingsKeys.contains("autoExposureMaxMs")) {
        m_autoExposureMaxMs = std::max(m_autoExposureMinMs, settings.m_autoExposureMaxMs);
    }
    if (settingsKeys.contains("autoExposureMinGain")) {
        m_autoExposureMinGain = std::max(0, settings.m_autoExposureMinGain);
        m_autoExposureMaxGain = std::max(m_autoExposureMinGain, m_autoExposureMaxGain);
    }
    if (settingsKeys.contains("autoExposureMaxGain")) {
        m_autoExposureMaxGain = std::max(m_autoExposureMinGain, settings.m_autoExposureMaxGain);
    }
    if (settingsKeys.contains("asiColorImageType")) {
        m_asiColorImageType = qBound(AsiColorImageTypeRgb24, settings.m_asiColorImageType, AsiColorImageTypeRaw8);
    }
    if (settingsKeys.contains("saveImage")) {
        m_saveImage = settings.m_saveImage;
    }
    if (settingsKeys.contains("imageFileName")) {
        m_imageFileName = settings.m_imageFileName;
    }
    if (settingsKeys.contains("saveVideo")) {
        m_saveVideo = settings.m_saveVideo;
    }
    if (settingsKeys.contains("videoFileName")) {
        m_videoFileName = settings.m_videoFileName;
    }
    if (settingsKeys.contains("videoFileCameraPath")) {
        m_videoFileCameraPath = settings.m_videoFileCameraPath;
    }
    if (settingsKeys.contains("streamUrl")) {
        m_streamUrl = settings.m_streamUrl;
    }
    if (settingsKeys.contains("imageFileCameraPaths")) {
        m_imageFileCameraPaths = settings.m_imageFileCameraPaths;
    }
    if (settingsKeys.contains("streamUrlHistory")) {
        m_streamUrlHistory = settings.m_streamUrlHistory;
    }
    if (settingsKeys.contains("streamBufferingSeconds")) {
        m_streamBufferingSeconds = qBound(
            m_minStreamBufferingSeconds,
            settings.m_streamBufferingSeconds,
            m_maxStreamBufferingSeconds);
    }
    if (settingsKeys.contains("videoLoop")) {
        m_videoLoop = settings.m_videoLoop;
    }
    if (settingsKeys.contains("videoPlaybackRate")) {
        m_videoPlaybackRate = qBound(m_minVideoPlaybackRate, settings.m_videoPlaybackRate, m_maxVideoPlaybackRate);
    }
    if (settingsKeys.contains("videoPlaybackAudioOffsetMs")) {
        m_videoPlaybackAudioOffsetMs = qBound(
            m_minVideoPlaybackAudioOffsetMs,
            settings.m_videoPlaybackAudioOffsetMs,
            m_maxVideoPlaybackAudioOffsetMs);
    }
    if (settingsKeys.contains("videoHwAcceleration")) {
        m_videoHwAcceleration = settings.m_videoHwAcceleration;
    }
    if (settingsKeys.contains("videoCodec")) {
        m_videoCodec = static_cast<VideoCodec>(qBound(0, static_cast<int>(settings.m_videoCodec), 1));
    }
    if (settingsKeys.contains("videoRecordBitrateKbps")) {
        m_videoRecordBitrateKbps = qBound(0, settings.m_videoRecordBitrateKbps, 240000);
    }
    if (settingsKeys.contains("videoPreRecordBufferSeconds")) {
        m_videoPreRecordBufferSeconds = qBound(0, settings.m_videoPreRecordBufferSeconds, 60);
    }
    if (settingsKeys.contains("imageRecordLimit")) {
        m_imageRecordLimit = std::max(m_minNonNegative, settings.m_imageRecordLimit);
    }
    if (settingsKeys.contains("videoRecordLimitSeconds")) {
        m_videoRecordLimitSeconds = std::max(m_minNonNegative, settings.m_videoRecordLimitSeconds);
    }
    if (settingsKeys.contains("keogramEnabled")) {
        m_keogramEnabled = settings.m_keogramEnabled;
    }
    if (settingsKeys.contains("keogramFileName")) {
        m_keogramFileName = settings.m_keogramFileName;
    }
    if (settingsKeys.contains("keogramDirection")) {
        m_keogramDirection = static_cast<KeogramDirection>(qBound(0, static_cast<int>(settings.m_keogramDirection), 1));
    }
    if (settingsKeys.contains("keogramDayMode")) {
        m_keogramDayMode = static_cast<KeogramDayMode>(qBound(0, static_cast<int>(settings.m_keogramDayMode), 1));
    }
    if (settingsKeys.contains("keogramSamplePeriodMinutes")) {
        m_keogramSamplePeriodMinutes = qBound(1, settings.m_keogramSamplePeriodMinutes, 1440);
    }
    if (settingsKeys.contains("keogramShowPreview")) {
        m_keogramShowPreview = settings.m_keogramShowPreview;
    }
    if (settingsKeys.contains("youtubeStreamEnabled")) {
        m_youtubeStreamEnabled = settings.m_youtubeStreamEnabled;
    }
    if (settingsKeys.contains("youtubeStreamUrl")) {
        m_youtubeStreamUrl = settings.m_youtubeStreamUrl;
    }
    if (settingsKeys.contains("youtubeStreamKey")) {
        m_youtubeStreamKey = settings.m_youtubeStreamKey;
    }
    if (settingsKeys.contains("youtubeStreamPostProcessed")) {
        m_youtubeStreamPostProcessed = settings.m_youtubeStreamPostProcessed;
    }
    if (settingsKeys.contains("youtubeStreamBitrateKbps")) {
        m_youtubeStreamBitrateKbps = qBound(100, settings.m_youtubeStreamBitrateKbps, 240000);
    }
    if (settingsKeys.contains("youtubeStreamFps")) {
        m_youtubeStreamFps = qBound(1, settings.m_youtubeStreamFps, 120);
    }
    if (settingsKeys.contains("youtubeStreamWidth")) {
        m_youtubeStreamWidth = qBound(0, settings.m_youtubeStreamWidth, 16384);
    }
    if (settingsKeys.contains("youtubeStreamHeight")) {
        m_youtubeStreamHeight = qBound(0, settings.m_youtubeStreamHeight, 16384);
    }
    if (settingsKeys.contains("stackEnabled")) {
        m_stackEnabled = settings.m_stackEnabled;
    }
    if (settingsKeys.contains("stackFrameCount")) {
        m_stackFrameCount = qBound(m_minStackFrameCount, settings.m_stackFrameCount, m_maxStackFrameCount);
    }
    if (settingsKeys.contains("stackMethod")) {
        m_stackMethod = qBound(StackMethodAverage, settings.m_stackMethod, StackMethodHDR);
    }
    if (settingsKeys.contains("stackHdrAlgorithm")) {
        m_stackHdrAlgorithm = qBound(StackHdrAlgorithmDebevec, settings.m_stackHdrAlgorithm, StackHdrAlgorithmMertens);
    }
    if (settingsKeys.contains("stackHdrExposureCount")) {
        m_stackHdrExposureCount = qBound(m_minHdrExposureCount, settings.m_stackHdrExposureCount, m_maxHdrExposureCount);
    }
    if (settingsKeys.contains("stackHdrExposure1Ms")) {
        m_stackHdrExposureTimesMs[0] = std::max(m_minExposureTimeMs, settings.m_stackHdrExposureTimesMs[0]);
    }
    if (settingsKeys.contains("stackHdrExposure2Ms")) {
        m_stackHdrExposureTimesMs[1] = std::max(m_minExposureTimeMs, settings.m_stackHdrExposureTimesMs[1]);
    }
    if (settingsKeys.contains("stackHdrExposure3Ms")) {
        m_stackHdrExposureTimesMs[2] = std::max(m_minExposureTimeMs, settings.m_stackHdrExposureTimesMs[2]);
    }
    if (settingsKeys.contains("stackHdrExposure4Ms")) {
        m_stackHdrExposureTimesMs[3] = std::max(m_minExposureTimeMs, settings.m_stackHdrExposureTimesMs[3]);
    }
    if (settingsKeys.contains("stackAlignmentMethod")) {
        m_stackAlignmentMethod = qBound(StackAlignmentNone, settings.m_stackAlignmentMethod, StackAlignmentStarCentroidMatching);
    }
    if (settingsKeys.contains("stackDisplayMode")) {
        m_stackDisplayMode = qBound(StackDisplayStacked, settings.m_stackDisplayMode, StackDisplayHistoryTiles);
    }
    if (settingsKeys.contains("stackDisplayFrameIndex")) {
        m_stackDisplayFrameIndex = qBound(0, settings.m_stackDisplayFrameIndex, m_maxStackFrameCount - 1);
    }
    if (settingsKeys.contains("stackRejectBadFrames")) {
        m_stackRejectBadFrames = settings.m_stackRejectBadFrames;
    }
    if (settingsKeys.contains("scaleEnabled")) {
        m_scaleEnabled = settings.m_scaleEnabled;
    }
    if (settingsKeys.contains("scaleWidth")) {
        m_scaleWidth = qBound(0, settings.m_scaleWidth, 65535);
    }
    if (settingsKeys.contains("scaleHeight")) {
        m_scaleHeight = qBound(0, settings.m_scaleHeight, 65535);
    }
    if (settingsKeys.contains("scaleKeepAspectRatio")) {
        m_scaleKeepAspectRatio = settings.m_scaleKeepAspectRatio;
    }
    if (settingsKeys.contains("scaleJustification")) {
        m_scaleJustification = static_cast<ScaleJustification>(qBound(
            static_cast<int>(ScaleJustifyCenter),
            static_cast<int>(settings.m_scaleJustification),
            static_cast<int>(ScaleJustifyBottom)));
    }
    if (settingsKeys.contains("stackDarkFileName")) {
        m_stackDarkFileName = settings.m_stackDarkFileName;
    }
    if (settingsKeys.contains("stackFlatFileName")) {
        m_stackFlatFileName = settings.m_stackFlatFileName;
    }
    if (settingsKeys.contains("stackBiasFileName")) {
        m_stackBiasFileName = settings.m_stackBiasFileName;
    }
    if (settingsKeys.contains("latitude")) {
        m_latitude = qBound(m_minLatitude, settings.m_latitude, m_maxLatitude);
    }
    if (settingsKeys.contains("longitude")) {
        m_longitude = qBound(m_minLongitude, settings.m_longitude, m_maxLongitude);
    }
    if (settingsKeys.contains("altitude")) {
        m_altitude = qBound(m_minAltitude, settings.m_altitude, m_maxAltitude);
    }
    if (settingsKeys.contains("positionSync")) {
        m_positionSync = settings.m_positionSync;
    }
    if (settingsKeys.contains("owmAPIKey")) {
        m_owmAPIKey = settings.m_owmAPIKey;
    }
    if (settingsKeys.contains("azimuth")) {
        m_azimuth = normalizePositiveDegrees(settings.m_azimuth);
    }
    if (settingsKeys.contains("elevation")) {
        m_elevation = qBound(m_minElevation, settings.m_elevation, m_maxElevation);
    }
    if (settingsKeys.contains("roll")) {
        m_roll = normalizeSignedDegrees(settings.m_roll);
    }
    if (settingsKeys.contains("rotator")) {
        m_rotator = settings.m_rotator;
    }
    if (settingsKeys.contains("directionSensor")) {
        m_directionSensor = settings.m_directionSensor;
    }
    if (settingsKeys.contains("azimuthOffset")) {
        m_azimuthOffset = normalizeSignedDegrees(settings.m_azimuthOffset);
    }
    if (settingsKeys.contains("elevationOffset")) {
        m_elevationOffset = qBound(-180.0f, settings.m_elevationOffset, 180.0f);
    }
    if (settingsKeys.contains("rollOffset")) {
        m_rollOffset = normalizeSignedDegrees(settings.m_rollOffset);
    }
    if (settingsKeys.contains("fov")) {
        m_fov = qBound(m_minFov, settings.m_fov, m_maxFov);
    }
    if (settingsKeys.contains("fovMode")) {
        m_fovMode = static_cast<FovMode>(qBound(
            static_cast<qint32>(FovModeDirect),
            static_cast<qint32>(settings.m_fovMode),
            static_cast<qint32>(FovModeSensorFocalLength)));
    }
    if (settingsKeys.contains("fovSensorWidthMm")) {
        m_fovSensorWidthMm = std::max(0.001, settings.m_fovSensorWidthMm);
    }
    if (settingsKeys.contains("fovSensorHeightMm")) {
        m_fovSensorHeightMm = std::max(0.001, settings.m_fovSensorHeightMm);
    }
    if (settingsKeys.contains("fovFocalLengthMm")) {
        m_fovFocalLengthMm = std::max(0.001, settings.m_fovFocalLengthMm);
    }
    if (settingsKeys.contains("lensProjection")) {
        m_lensProjection = (LensProjection) qBound((int) LensProjectionRectilinear, (int) settings.m_lensProjection, (int) LensProjectionEquisolid);
    }
    if (settingsKeys.contains("lensCenterOffsetX")) {
        m_lensCenterOffsetX = qBound(m_minLensCenterOffset, settings.m_lensCenterOffsetX, m_maxLensCenterOffset);
    }
    if (settingsKeys.contains("lensCenterOffsetY")) {
        m_lensCenterOffsetY = qBound(m_minLensCenterOffset, settings.m_lensCenterOffsetY, m_maxLensCenterOffset);
    }
    if (settingsKeys.contains("lensDistortionK1")) {
        m_lensDistortionK1 = qBound(m_minLensDistortionK1, settings.m_lensDistortionK1, m_maxLensDistortionK1);
    }
    if (settingsKeys.contains("workspaceIndex")) {
        m_workspaceIndex = settings.m_workspaceIndex;
    }
    if (settingsKeys.contains("brightness")) {
        m_brightness = qBound(m_minBrightness, settings.m_brightness, m_maxBrightness);
    }
    if (settingsKeys.contains("postProcessWhiteBalanceMode")) {
        m_postProcessWhiteBalanceMode = qBound(m_minNonNegative, settings.m_postProcessWhiteBalanceMode, 2);
    }
    if (settingsKeys.contains("postProcessWhiteBalanceRedGain")) {
        m_postProcessWhiteBalanceRedGain = qBound(m_minWhiteBalanceGain, settings.m_postProcessWhiteBalanceRedGain, m_maxWhiteBalanceGain);
    }
    if (settingsKeys.contains("postProcessWhiteBalanceGreenGain")) {
        m_postProcessWhiteBalanceGreenGain = qBound(m_minWhiteBalanceGain, settings.m_postProcessWhiteBalanceGreenGain, m_maxWhiteBalanceGain);
    }
    if (settingsKeys.contains("postProcessWhiteBalanceBlueGain")) {
        m_postProcessWhiteBalanceBlueGain = qBound(m_minWhiteBalanceGain, settings.m_postProcessWhiteBalanceBlueGain, m_maxWhiteBalanceGain);
    }
    if (settingsKeys.contains("postProcessWhiteBalanceHighlightProtection")) {
        m_postProcessWhiteBalanceHighlightProtection = qBound(m_minNormalized, settings.m_postProcessWhiteBalanceHighlightProtection, m_maxNormalized);
    }
    if (settingsKeys.contains("postProcessUseCuda")) {
        m_postProcessUseCuda = settings.m_postProcessUseCuda;
    }
    if (settingsKeys.contains("postProcessUnwarp")) {
        m_postProcessUnwarp = settings.m_postProcessUnwarp;
    }
    if (settingsKeys.contains("histogramStretch")) {
        m_histogramStretch = qBound(HistogramStretchOff, settings.m_histogramStretch, HistogramStretchCLAHE);
    }
    if (settingsKeys.contains("histogramStretchBlackPoint")) {
        m_histogramStretchBlackPoint = qBound(m_minNormalized, settings.m_histogramStretchBlackPoint, m_maxNormalized);
    }
    if (settingsKeys.contains("histogramStretchWhitePoint")) {
        m_histogramStretchWhitePoint = qBound(m_histogramStretchBlackPoint + m_minHistogramWhitePointGap, settings.m_histogramStretchWhitePoint, m_maxNormalized);
    }
    if (settingsKeys.contains("histogramStretchGamma")) {
        m_histogramStretchGamma = qBound(m_minHistogramStrength, settings.m_histogramStretchGamma, m_maxWhiteBalanceGain);
    }
    if (settingsKeys.contains("histogramStretchAsinhStrength")) {
        m_histogramStretchAsinhStrength = qBound(m_minHistogramStrength, settings.m_histogramStretchAsinhStrength, m_maxHistogramStrength);
    }
    if (settingsKeys.contains("histogramStretchLogStrength")) {
        m_histogramStretchLogStrength = qBound(m_minHistogramStrength, settings.m_histogramStretchLogStrength, m_maxHistogramStrength);
    }
    if (settingsKeys.contains("histogramVisible")) {
        m_histogramVisible = settings.m_histogramVisible;
    }
    if (settingsKeys.contains("postProcessGreyscale")) {
        m_postProcessGreyscale = settings.m_postProcessGreyscale;
    }
    if (settingsKeys.contains("saturation")) {
        m_saturation = qBound(m_minFilterAmount, settings.m_saturation, m_maxFilterAmount);
    }
    if (settingsKeys.contains("gamma")) {
        m_gamma = qBound(m_minHistogramStrength, settings.m_gamma, m_maxFilterAmount);
    }
    if (settingsKeys.contains("gaussianBlur")) {
        m_gaussianBlur = qBound(m_minBlurRadius, settings.m_gaussianBlur, m_maxBlurRadius);
    }
    if (settingsKeys.contains("medianBlur")) {
        m_medianBlur = qBound(m_minBlurRadius, settings.m_medianBlur, m_maxBlurRadius);
    }
    if (settingsKeys.contains("sharpen")) {
        m_sharpen = qBound(m_minFilterAmount, settings.m_sharpen, m_maxFilterAmount);
    }
    if (settingsKeys.contains("edgeDisplayMode")) {
        m_edgeDisplayMode = static_cast<EdgeDisplayMode>(qBound(
            static_cast<qint32>(EdgeDisplayOverlay),
            static_cast<qint32>(settings.m_edgeDisplayMode),
            static_cast<qint32>(EdgeDisplayEdgesOnly)));
    }
    if (settingsKeys.contains("sobelEdge")) {
        m_sobelEdge = qBound(m_minFilterAmount, settings.m_sobelEdge, m_maxFilterAmount);
    }
    if (settingsKeys.contains("cannyEdge")) {
        m_cannyEdge = qBound(m_minFilterAmount, settings.m_cannyEdge, m_maxFilterAmount);
    }
    if (settingsKeys.contains("flipX")) {
        m_flipX = settings.m_flipX;
    }
    if (settingsKeys.contains("flipY")) {
        m_flipY = settings.m_flipY;
    }
    if (settingsKeys.contains("imageRotation")) {
        m_imageRotation = normalizeImageRotation(settings.m_imageRotation);
    }
    if (settingsKeys.contains("contrast")) {
        m_contrast = qBound(m_minWhiteBalanceGain, settings.m_contrast, m_maxFilterAmount);
    }
    if (settingsKeys.contains("invertColors")) {
        m_invertColors = settings.m_invertColors;
    }
    if (settingsKeys.contains("overlayDateTime")) {
        m_overlayDateTime = settings.m_overlayDateTime;
    }
    if (settingsKeys.contains("dateTimeColor")) {
        m_dateTimeColor = settings.m_dateTimeColor;
    }
    if (settingsKeys.contains("diffMask")) {
        m_diffMask = settings.m_diffMask;
    }
    if (settingsKeys.contains("diffThreshold")) {
        m_diffThreshold = qBound(m_minThreshold8Bit, settings.m_diffThreshold, m_maxThreshold8Bit);
    }
    if (settingsKeys.contains("diffMaskOpenSize")) {
        m_diffMaskOpenSize = qBound(m_minMorphologyKernel, settings.m_diffMaskOpenSize, m_maxMorphologyKernel);
    }
    if (settingsKeys.contains("dilationSize")) {
        m_dilationSize = qBound(m_minMorphologyKernel, settings.m_dilationSize, m_maxMorphologyKernel);
    }
    if (settingsKeys.contains("diffMaskHistoryFrames")) {
        m_diffMaskHistoryFrames = qBound(m_minShortHistoryFrames, settings.m_diffMaskHistoryFrames, m_maxShortHistoryFrames);
    }
    if (settingsKeys.contains("diffMaskCloseSize")) {
        m_diffMaskCloseSize = qBound(m_minMorphologyKernel, settings.m_diffMaskCloseSize, m_maxMorphologyKernel);
    }
    if (settingsKeys.contains("overlayFontFamily")) {
        m_overlayFontFamily = settings.m_overlayFontFamily;
    }
    if (settingsKeys.contains("overlayFontScale")) {
        m_overlayFontScale = qBound(m_minOverlayFontScale, settings.m_overlayFontScale, m_maxOverlayFontScale);
    }
    if (settingsKeys.contains("detectionRoiX")) {
        m_detectionRoiX = qBound(m_minUiPixelOffset, settings.m_detectionRoiX, m_maxUiPixelOffset);
    }
    if (settingsKeys.contains("detectionRoiY")) {
        m_detectionRoiY = qBound(m_minUiPixelOffset, settings.m_detectionRoiY, m_maxUiPixelOffset);
    }
    if (settingsKeys.contains("detectionRoiWidth")) {
        m_detectionRoiWidth = qBound(m_minUiPixelOffset, settings.m_detectionRoiWidth, m_maxUiPixelOffset);
    }
    if (settingsKeys.contains("detectionRoiHeight")) {
        m_detectionRoiHeight = qBound(m_minUiPixelOffset, settings.m_detectionRoiHeight, m_maxUiPixelOffset);
    }
    if (settingsKeys.contains("showDetectionRoi")) {
        m_showDetectionRoi = settings.m_showDetectionRoi;
    }
    if (settingsKeys.contains("showStarDetectionBoxes")) {
        m_showStarDetectionBoxes = settings.m_showStarDetectionBoxes;
    }
    if (settingsKeys.contains("motionDetect")) {
        m_motionDetect = settings.m_motionDetect;
    }
    if (settingsKeys.contains("motionBackgroundSubtractor")) {
        m_motionBackgroundSubtractor = static_cast<MotionBackgroundSubtractor>(qBound(
            static_cast<qint32>(MotionBackgroundSubtractorMOG2),
            static_cast<qint32>(settings.m_motionBackgroundSubtractor),
            static_cast<qint32>(MotionBackgroundSubtractorKNN)));
    }
    if (settingsKeys.contains("motionMaskView")) {
        m_motionMaskView = static_cast<MotionMaskView>(qBound(
            static_cast<qint32>(MotionMaskViewOff),
            static_cast<qint32>(settings.m_motionMaskView),
            static_cast<qint32>(MotionMaskViewFinal)));
    }
    if (settingsKeys.contains("motionHistory")) {
        m_motionHistory = qBound(m_minMotionHistory, settings.m_motionHistory, m_maxMotionHistory);
    }
    if (settingsKeys.contains("motionVarThreshold")) {
        m_motionVarThreshold = qBound(m_minMotionVarThreshold, settings.m_motionVarThreshold, m_maxMotionVarThreshold);
    }
    if (settingsKeys.contains("motionLearningRate")) {
        m_motionLearningRate = qBound(m_minLearningRate, settings.m_motionLearningRate, m_maxLearningRate);
    }
    if (settingsKeys.contains("motionConfirmFrames")) {
        m_motionConfirmFrames = qBound(m_minMotionConfirmFrames, settings.m_motionConfirmFrames, m_maxMotionConfirmFrames);
    }
    if (settingsKeys.contains("motionDownscale")) {
        const QList<double> validDownscales{1.0, 0.5, 0.25};
        m_motionDownscale = validDownscales.contains(settings.m_motionDownscale) ? settings.m_motionDownscale : 1.0;
    }
    if (settingsKeys.contains("motionDetectShadows")) {
        m_motionDetectShadows = settings.m_motionDetectShadows;
    }
    if (settingsKeys.contains("motionOpenSize")) {
        m_motionOpenSize = qBound(m_minMorphologyKernel, settings.m_motionOpenSize, m_maxMorphologyKernel);
    }
    if (settingsKeys.contains("motionCloseSize")) {
        m_motionCloseSize = qBound(m_minMorphologyKernel, settings.m_motionCloseSize, m_maxMorphologyKernel);
    }
    if (settingsKeys.contains("motionPersistenceFrames")) {
        m_motionPersistenceFrames = qBound(m_minNonNegative, settings.m_motionPersistenceFrames, m_maxShortHistoryFrames);
    }
    if (settingsKeys.contains("motionBoxColor")) {
        m_motionBoxColor = settings.m_motionBoxColor;
    }
    if (settingsKeys.contains("minContourArea")) {
        m_minContourArea = qBound(m_minContourAreaBound, settings.m_minContourArea, m_maxContourAreaBound);
    }
    if (settingsKeys.contains("motionExclusionRects")) {
        m_motionExclusionRects = settings.m_motionExclusionRects;
    }
    if (settingsKeys.contains("starDetect")) {
        m_starDetect = settings.m_starDetect;
    }
    if (settingsKeys.contains("starThreshold")) {
        m_starThreshold = qBound(m_minThreshold8Bit, settings.m_starThreshold, m_maxThreshold8Bit);
    }
    if (settingsKeys.contains("starBackgroundBlur")) {
        m_starBackgroundBlur = qBound(m_minStarBackgroundBlur, settings.m_starBackgroundBlur, m_maxStarBackgroundBlur);
    }
    if (settingsKeys.contains("starMinArea")) {
        m_starMinArea = qBound(m_minContourAreaBound, settings.m_starMinArea, m_maxContourAreaBound);
    }
    if (settingsKeys.contains("starMaxArea")) {
        m_starMaxArea = qBound(m_starMinArea, settings.m_starMaxArea, m_maxContourAreaBound);
    }
    if (settingsKeys.contains("starMaxAspectRatio")) {
        m_starMaxAspectRatio = qBound(m_minStarAspectRatio, settings.m_starMaxAspectRatio, m_maxStarAspectRatio);
    }
    if (settingsKeys.contains("starDebugView")) {
        m_starDebugView = static_cast<StarDebugView>(qBound(
            static_cast<qint32>(StarDebugViewOff),
            static_cast<qint32>(settings.m_starDebugView),
            static_cast<qint32>(StarDebugViewFinal)));
    }
    if (settingsKeys.contains("starColor")) {
        m_starColor = settings.m_starColor;
    }
    if (settingsKeys.contains("plateSolve")) {
        m_plateSolve = settings.m_plateSolve;
    }
    if (settingsKeys.contains("plateSolveMaxMagnitude")) {
        m_plateSolveMaxMagnitude = settings.m_plateSolveMaxMagnitude;
    }
    if (settingsKeys.contains("plateSolveMinMatches")) {
        m_plateSolveMinMatches = settings.m_plateSolveMinMatches;
    }
    if (settingsKeys.contains("plateSolveMatchRadius")) {
        m_plateSolveMatchRadius = settings.m_plateSolveMatchRadius;
    }
    if (settingsKeys.contains("plateSolveFinalMatchRadius")) {
        m_plateSolveFinalMatchRadius = settings.m_plateSolveFinalMatchRadius;
    }
    if (settingsKeys.contains("plateSolveSearchRadius")) {
        m_plateSolveAzElSearchRadius = settings.m_plateSolveAzElSearchRadius;
    }
    if (settingsKeys.contains("plateSolveFovTolerance")) {
        m_plateSolveFovTolerance = settings.m_plateSolveFovTolerance;
    }
    if (settingsKeys.contains("plateSolveStartMode")) {
        m_plateSolveStartMode = settings.m_plateSolveStartMode;
    }
    if (settingsKeys.contains("plateSolveLabelMode")) {
        m_plateSolveLabelMode = settings.m_plateSolveLabelMode;
    }
    if (settingsKeys.contains("plateSolveLabelHideSyntheticNames")) {
        m_plateSolveLabelHideSyntheticNames = settings.m_plateSolveLabelHideSyntheticNames;
    }
    if (settingsKeys.contains("plateSolveUseCaptureDateTime")) {
        m_plateSolveUseCaptureDateTime = settings.m_plateSolveUseCaptureDateTime;
    }
    if (settingsKeys.contains("plateSolveDateTime")) {
        m_plateSolveDateTime = settings.m_plateSolveDateTime;
    }
    if (settingsKeys.contains("plateSolveDateTimeUtc")) {
        m_plateSolveDateTimeUtc = settings.m_plateSolveDateTimeUtc;
    }
    if (settingsKeys.contains("plateSolveUseDownloadedCatalog")) {
        m_plateSolveUseDownloadedCatalog = settings.m_plateSolveUseDownloadedCatalog;
    }
    if (settingsKeys.contains("plateSolveCatalogSource")) {
        m_plateSolveCatalogSource = settings.m_plateSolveCatalogSource;
    }
    if (settingsKeys.contains("plateSolveApplyMode")) {
        m_plateSolveApplyMode = settings.m_plateSolveApplyMode;
    }
    if (settingsKeys.contains("starCatalogDiskCacheSizeGb")) {
        m_starCatalogDiskCacheSizeGb = qBound(
            m_minStarCatalogDiskCacheSizeGb,
            settings.m_starCatalogDiskCacheSizeGb,
            m_maxStarCatalogDiskCacheSizeGb);
    }
    if (settingsKeys.contains("recordRawFits")) {
        m_recordRawFits = settings.m_recordRawFits;
    }
    if (settingsKeys.contains("recordCalibratedMedia")) {
        m_recordCalibratedMedia = settings.m_recordCalibratedMedia;
    }
    if (settingsKeys.contains("recordFilteredMedia")) {
        m_recordFilteredMedia = settings.m_recordFilteredMedia;
    }
    if (settingsKeys.contains("recordPostProcessedMedia")) {
        m_recordPostProcessedMedia = settings.m_recordPostProcessedMedia;
    }
    if (settingsKeys.contains("overlaySpectrum")) {
        m_overlaySpectrum = settings.m_overlaySpectrum;
    }
    if (settingsKeys.contains("spectrumDevice")) {
        m_spectrumDevice = settings.m_spectrumDevice;
    }
    if (settingsKeys.contains("spectrumOffsetX")) {
        m_spectrumOffsetX = qBound(m_minSignedUiPixelOffset, settings.m_spectrumOffsetX, m_maxSignedUiPixelOffset);
    }
    if (settingsKeys.contains("spectrumOffsetY")) {
        m_spectrumOffsetY = qBound(m_minSignedUiPixelOffset, settings.m_spectrumOffsetY, m_maxSignedUiPixelOffset);
    }
    if (settingsKeys.contains("spectrumScale")) {
        m_spectrumScale = qBound(m_minSpectrumScale, settings.m_spectrumScale, m_maxSpectrumScale);
    }
    if (settingsKeys.contains("dateTimeFormat")) {
        m_dateTimeFormat = settings.m_dateTimeFormat;
    }
    if (settingsKeys.contains("dateTimePosX")) {
        m_dateTimePosX = qBound(m_minUiPixelOffset, settings.m_dateTimePosX, m_maxUiPixelOffset);
    }
    if (settingsKeys.contains("dateTimePosY")) {
        m_dateTimePosY = qBound(m_minUiPixelOffset, settings.m_dateTimePosY, m_maxUiPixelOffset);
    }
    if (settingsKeys.contains("equatorialGrid")) {
        m_equatorialGrid = settings.m_equatorialGrid;
    }
    if (settingsKeys.contains("equatorialGridColor")) {
        m_equatorialGridColor = settings.m_equatorialGridColor;
    }
    if (settingsKeys.contains("altAzGrid")) {
        m_altAzGrid = settings.m_altAzGrid;
    }
    if (settingsKeys.contains("altAzGridColor")) {
        m_altAzGridColor = settings.m_altAzGridColor;
    }
    if (settingsKeys.contains("constellation")) {
        m_constellation = settings.m_constellation;
    }
    if (settingsKeys.contains("constellationColor")) {
        m_constellationColor = settings.m_constellationColor;
    }
    if (settingsKeys.contains("constellationOverlay")) {
        m_constellationOverlay = settings.m_constellationOverlay;
    }
    if (settingsKeys.contains("trackObjects")) {
        m_trackObjects = settings.m_trackObjects;
    }
    if (settingsKeys.contains("trackObjectTrails")) {
        m_trackObjectTrails = settings.m_trackObjectTrails;
    }
    if (settingsKeys.contains("trackObjectHeatMap")) {
        m_trackObjectHeatMap = settings.m_trackObjectHeatMap;
    }
    if (settingsKeys.contains("trackObjectRange")) {
        m_trackObjectRange = settings.m_trackObjectRange;
    }
    if (settingsKeys.contains("trackObjectMinElevation")) {
        m_trackObjectMinElevation = qBound(m_minNormalized, settings.m_trackObjectMinElevation, static_cast<double>(m_maxElevation));
    }
    if (settingsKeys.contains("trackObjectColor")) {
        m_trackObjectColor = settings.m_trackObjectColor;
    }
    if (settingsKeys.contains("trackObjectFontFamily")) {
        m_trackObjectFontFamily = settings.m_trackObjectFontFamily;
    }
    if (settingsKeys.contains("trackObjectFontScale")) {
        m_trackObjectFontScale = qBound(m_minOverlayFontScale, settings.m_trackObjectFontScale, m_maxOverlayFontScale);
    }
    if (settingsKeys.contains("gridLabelFontFamily")) {
        m_gridLabelFontFamily = settings.m_gridLabelFontFamily;
    }
    if (settingsKeys.contains("gridLabelFontScale")) {
        m_gridLabelFontScale = qBound(m_minOverlayFontScale, settings.m_gridLabelFontScale, m_maxOverlayFontScale);
    }
    if (settingsKeys.contains("overlayText")) {
        m_overlayText = settings.m_overlayText;
    }
    if (settingsKeys.contains("overlayTextString")) {
        m_overlayTextString = settings.m_overlayTextString;
    }
    if (settingsKeys.contains("overlayTextColor")) {
        m_overlayTextColor = settings.m_overlayTextColor;
    }
    if (settingsKeys.contains("overlayTextFontFamily")) {
        m_overlayTextFontFamily = settings.m_overlayTextFontFamily;
    }
    if (settingsKeys.contains("overlayTextFontScale")) {
        m_overlayTextFontScale = qBound(m_minOverlayFontScale, settings.m_overlayTextFontScale, m_maxOverlayFontScale);
    }
    if (settingsKeys.contains("overlayTextPosX")) {
        m_overlayTextPosX = qBound(m_minUiPixelOffset, settings.m_overlayTextPosX, m_maxUiPixelOffset);
    }
    if (settingsKeys.contains("overlayTextPosY")) {
        m_overlayTextPosY = qBound(m_minUiPixelOffset, settings.m_overlayTextPosY, m_maxUiPixelOffset);
    }
    if (settingsKeys.contains("yoloEnabled")) {
        m_yoloEnabled = settings.m_yoloEnabled;
    }
    if (settingsKeys.contains("yoloModelPath")) {
        m_yoloModelPath = settings.m_yoloModelPath;
    }
    if (settingsKeys.contains("yoloLabelsPath")) {
        m_yoloLabelsPath = settings.m_yoloLabelsPath;
    }
    if (settingsKeys.contains("yoloConfThreshold")) {
        m_yoloConfThreshold = qBound(m_minNormalized, settings.m_yoloConfThreshold, m_maxNormalized);
    }
    if (settingsKeys.contains("yoloNmsThreshold")) {
        m_yoloNmsThreshold = qBound(m_minNormalized, settings.m_yoloNmsThreshold, m_maxNormalized);
    }
    if (settingsKeys.contains("yoloBoxColor")) {
        m_yoloBoxColor = settings.m_yoloBoxColor;
    }
    if (settingsKeys.contains("yoloTileLargeImages")) {
        m_yoloTileLargeImages = settings.m_yoloTileLargeImages;
    }
    if (settingsKeys.contains("yoloTileOverlapPercent")) {
        m_yoloTileOverlapPercent = qBound(0, settings.m_yoloTileOverlapPercent, 90);
    }
    if (settingsKeys.contains("yoloIgnoredClassNames")) {
        m_yoloIgnoredClassNames = settings.m_yoloIgnoredClassNames;
    }
    if (settingsKeys.contains("yoloDnnTarget")) {
        m_yoloDnnTarget = qBound(CPU, settings.m_yoloDnnTarget, TensorRT_FP16);
    }
    if (settingsKeys.contains("audioMute")) {
        m_audioMute = settings.m_audioMute;
    }
    if (settingsKeys.contains("audioPreviewVolume")) {
        m_audioPreviewVolume = qBound(0, settings.m_audioPreviewVolume, 100);
    }
    if (settingsKeys.contains("audioDeviceName")) {
        m_audioDeviceName = settings.m_audioDeviceName;
    }
    if (settingsKeys.contains("whiteBalanceMode")) {
        m_whiteBalanceMode = std::max(m_minNonNegative, settings.m_whiteBalanceMode);
    }
    if (settingsKeys.contains("exposureCompensation")) {
        m_exposureCompensation = qBound(m_minExposureCompensation, settings.m_exposureCompensation, m_maxExposureCompensation);
    }
    if (settingsKeys.contains("focusMode")) {
        m_focusMode = std::max(m_minNonNegative, settings.m_focusMode);
    }
    if (settingsKeys.contains("focusDistance")) {
        m_focusDistance = qBound(m_minNormalized, settings.m_focusDistance, m_maxNormalized);
    }
    if (settingsKeys.contains("zoomFactor")) {
        m_zoomFactor = std::max(m_minZoomFactor, settings.m_zoomFactor);
    }
}

QString CameraSettings::getDebugString(const QStringList& settingsKeys, bool force) const
{
    std::ostringstream ostr;

    if (settingsKeys.contains("cameraProtocol") || force) {
        ostr << " m_cameraProtocol: " << m_cameraProtocol.toStdString();
    }
    if (settingsKeys.contains("cameraId") || force) {
        ostr << " m_cameraId: " << m_cameraId.toStdString();
    }
    if (settingsKeys.contains("cameraDescription") || force) {
        ostr << " m_cameraDescription: " << m_cameraDescription.toStdString();
    }
    if (settingsKeys.contains("resolutionWidth") || force) {
        ostr << " m_resolutionWidth: " << m_resolutionWidth;
    }
    if (settingsKeys.contains("resolutionHeight") || force) {
        ostr << " m_resolutionHeight: " << m_resolutionHeight;
    }
    if (settingsKeys.contains("framesPerSecond") || force) {
        ostr << " m_framesPerSecond: " << m_framesPerSecond;
    }
    if (settingsKeys.contains("captureMode") || force) {
        ostr << " m_captureMode: " << m_captureMode;
    }
    if (settingsKeys.contains("captureInterval") || force) {
        ostr << " m_captureInterval: " << m_captureInterval;
    }
    if (settingsKeys.contains("captureIntervalUnits") || force) {
        ostr << " m_captureIntervalUnits: " << m_captureIntervalUnits;
    }
    if (settingsKeys.contains("exposureTimeMs") || force) {
        ostr << " m_exposureTimeMs: " << m_exposureTimeMs;
    }
    if (settingsKeys.contains("isoSensitivity") || force) {
        ostr << " m_isoSensitivity: " << m_isoSensitivity;
    }
    if (settingsKeys.contains("alpacaDiscoveryEnabled") || force) {
        ostr << " m_alpacaDiscoveryEnabled: " << m_alpacaDiscoveryEnabled;
    }
    if (settingsKeys.contains("alpacaApiLogEnabled") || force) {
        ostr << " m_alpacaApiLogEnabled: " << m_alpacaApiLogEnabled;
    }
    if (settingsKeys.contains("alpacaHost") || force) {
        ostr << " m_alpacaHost: " << m_alpacaHost.toStdString();
    }
    if (settingsKeys.contains("alpacaPort") || force) {
        ostr << " m_alpacaPort: " << m_alpacaPort;
    }
    if (settingsKeys.contains("alpacaFocuserEnabled") || force) {
        ostr << " m_alpacaFocuserEnabled: " << m_alpacaFocuserEnabled;
    }
    if (settingsKeys.contains("alpacaFocuserHost") || force) {
        ostr << " m_alpacaFocuserHost: " << m_alpacaFocuserHost.toStdString();
    }
    if (settingsKeys.contains("alpacaFocuserPort") || force) {
        ostr << " m_alpacaFocuserPort: " << m_alpacaFocuserPort;
    }
    if (settingsKeys.contains("alpacaFocuserDeviceNumber") || force) {
        ostr << " m_alpacaFocuserDeviceNumber: " << m_alpacaFocuserDeviceNumber;
    }
    if (settingsKeys.contains("alpacaFocusPosition") || force) {
        ostr << " m_alpacaFocusPosition: " << m_alpacaFocusPosition;
    }
    if (settingsKeys.contains("alpacaFocusStepSize") || force) {
        ostr << " m_alpacaFocusStepSize: " << m_alpacaFocusStepSize;
    }
    if (settingsKeys.contains("alpacaFilterWheelEnabled") || force) {
        ostr << " m_alpacaFilterWheelEnabled: " << m_alpacaFilterWheelEnabled;
    }
    if (settingsKeys.contains("alpacaFilterWheelHost") || force) {
        ostr << " m_alpacaFilterWheelHost: " << m_alpacaFilterWheelHost.toStdString();
    }
    if (settingsKeys.contains("alpacaFilterWheelPort") || force) {
        ostr << " m_alpacaFilterWheelPort: " << m_alpacaFilterWheelPort;
    }
    if (settingsKeys.contains("alpacaFilterWheelDeviceNumber") || force) {
        ostr << " m_alpacaFilterWheelDeviceNumber: " << m_alpacaFilterWheelDeviceNumber;
    }
    if (settingsKeys.contains("alpacaFilterWheelPosition") || force) {
        ostr << " m_alpacaFilterWheelPosition: " << m_alpacaFilterWheelPosition;
    }
    if (settingsKeys.contains("cameraBinX") || force) {
        ostr << " m_cameraBinX: " << m_cameraBinX;
    }
    if (settingsKeys.contains("cameraBinY") || force) {
        ostr << " m_cameraBinY: " << m_cameraBinY;
    }
    if (settingsKeys.contains("cameraNumX") || force) {
        ostr << " m_cameraNumX: " << m_cameraNumX;
    }
    if (settingsKeys.contains("cameraNumY") || force) {
        ostr << " m_cameraNumY: " << m_cameraNumY;
    }
    if (settingsKeys.contains("cameraStartX") || force) {
        ostr << " m_cameraStartX: " << m_cameraStartX;
    }
    if (settingsKeys.contains("cameraStartY") || force) {
        ostr << " m_cameraStartY: " << m_cameraStartY;
    }
    if (settingsKeys.contains("cameraGain") || force) {
        ostr << " m_cameraGain: " << m_cameraGain;
    }
    if (settingsKeys.contains("cameraOffset") || force) {
        ostr << " m_cameraOffset: " << m_cameraOffset;
    }
    if (settingsKeys.contains("cameraReadoutMode") || force) {
        ostr << " m_cameraReadoutMode: " << m_cameraReadoutMode;
    }
    if (settingsKeys.contains("asiCoolerOn") || force) {
        ostr << " m_asiCoolerOn: " << m_asiCoolerOn;
    }
    if (settingsKeys.contains("asiTargetTemp") || force) {
        ostr << " m_asiTargetTemp: " << m_asiTargetTemp;
    }
    if (settingsKeys.contains("asiUsbBandwidth") || force) {
        ostr << " m_asiUsbBandwidth: " << m_asiUsbBandwidth;
    }
    if (settingsKeys.contains("asiHighSpeedMode") || force) {
        ostr << " m_asiHighSpeedMode: " << m_asiHighSpeedMode;
    }
    if (settingsKeys.contains("asiAutoExposureGain") || force) {
        ostr << " m_asiAutoExposureGain: " << m_asiAutoExposureGain;
    }
    if (settingsKeys.contains("autoExposureGainEnabled") || force) {
        ostr << " m_autoExposureGainEnabled: " << m_autoExposureGainEnabled;
    }
    if (settingsKeys.contains("autoExposureGainMode") || force) {
        ostr << " m_autoExposureGainMode: " << m_autoExposureGainMode;
    }
    if (settingsKeys.contains("autoExposureTargetPercentile") || force) {
        ostr << " m_autoExposureTargetPercentile: " << m_autoExposureTargetPercentile;
    }
    if (settingsKeys.contains("autoExposureTargetBrightness") || force) {
        ostr << " m_autoExposureTargetBrightness: " << m_autoExposureTargetBrightness;
    }
    if (settingsKeys.contains("autoExposureMaxChangePercent") || force) {
        ostr << " m_autoExposureMaxChangePercent: " << m_autoExposureMaxChangePercent;
    }
    if (settingsKeys.contains("autoExposureMinMs") || force) {
        ostr << " m_autoExposureMinMs: " << m_autoExposureMinMs;
    }
    if (settingsKeys.contains("autoExposureMaxMs") || force) {
        ostr << " m_autoExposureMaxMs: " << m_autoExposureMaxMs;
    }
    if (settingsKeys.contains("autoExposureMinGain") || force) {
        ostr << " m_autoExposureMinGain: " << m_autoExposureMinGain;
    }
    if (settingsKeys.contains("autoExposureMaxGain") || force) {
        ostr << " m_autoExposureMaxGain: " << m_autoExposureMaxGain;
    }
    if (settingsKeys.contains("asiColorImageType") || force) {
        ostr << " m_asiColorImageType: " << m_asiColorImageType;
    }
    if (settingsKeys.contains("saveImage") || force) {
        ostr << " m_saveImage: " << m_saveImage;
    }
    if (settingsKeys.contains("imageFileName") || force) {
        ostr << " m_imageFileName: " << m_imageFileName.toStdString();
    }
    if (settingsKeys.contains("saveVideo") || force) {
        ostr << " m_saveVideo: " << m_saveVideo;
    }
    if (settingsKeys.contains("videoFileName") || force) {
        ostr << " m_videoFileName: " << m_videoFileName.toStdString();
    }
    if (settingsKeys.contains("videoFileCameraPath") || force) {
        ostr << " m_videoFileCameraPath: " << m_videoFileCameraPath.toStdString();
    }
    if (settingsKeys.contains("streamUrl") || force) {
        ostr << " m_streamUrl: " << m_streamUrl.toStdString();
    }
    if (settingsKeys.contains("imageFileCameraPaths") || force) {
        ostr << " m_imageFileCameraPaths: " << m_imageFileCameraPaths.join('|').toStdString();
    }
    if (settingsKeys.contains("streamUrlHistory") || force) {
        ostr << " m_streamUrlHistory: " << m_streamUrlHistory.join('|').toStdString();
    }
    if (settingsKeys.contains("streamBufferingSeconds") || force) {
        ostr << " m_streamBufferingSeconds: " << m_streamBufferingSeconds;
    }
    if (settingsKeys.contains("videoLoop") || force) {
        ostr << " m_videoLoop: " << m_videoLoop;
    }
    if (settingsKeys.contains("videoPlaybackRate") || force) {
        ostr << " m_videoPlaybackRate: " << m_videoPlaybackRate;
    }
    if (settingsKeys.contains("videoPlaybackAudioOffsetMs") || force) {
        ostr << " m_videoPlaybackAudioOffsetMs: " << m_videoPlaybackAudioOffsetMs;
    }
    if (settingsKeys.contains("videoHwAcceleration") || force) {
        ostr << " m_videoHwAcceleration: " << m_videoHwAcceleration;
    }
    if (settingsKeys.contains("videoCodec") || force) {
        ostr << " m_videoCodec: " << static_cast<int>(m_videoCodec);
    }
    if (settingsKeys.contains("videoRecordBitrateKbps") || force) {
        ostr << " m_videoRecordBitrateKbps: " << m_videoRecordBitrateKbps;
    }
    if (settingsKeys.contains("videoPreRecordBufferSeconds") || force) {
        ostr << " m_videoPreRecordBufferSeconds: " << m_videoPreRecordBufferSeconds;
    }
    if (settingsKeys.contains("imageRecordLimit") || force) {
        ostr << " m_imageRecordLimit: " << m_imageRecordLimit;
    }
    if (settingsKeys.contains("videoRecordLimitSeconds") || force) {
        ostr << " m_videoRecordLimitSeconds: " << m_videoRecordLimitSeconds;
    }
    if (settingsKeys.contains("keogramEnabled") || force) {
        ostr << " m_keogramEnabled: " << m_keogramEnabled;
    }
    if (settingsKeys.contains("keogramFileName") || force) {
        ostr << " m_keogramFileName: " << m_keogramFileName.toStdString();
    }
    if (settingsKeys.contains("keogramDirection") || force) {
        ostr << " m_keogramDirection: " << static_cast<int>(m_keogramDirection);
    }
    if (settingsKeys.contains("keogramDayMode") || force) {
        ostr << " m_keogramDayMode: " << static_cast<int>(m_keogramDayMode);
    }
    if (settingsKeys.contains("keogramSamplePeriodMinutes") || force) {
        ostr << " m_keogramSamplePeriodMinutes: " << m_keogramSamplePeriodMinutes;
    }
    if (settingsKeys.contains("keogramShowPreview") || force) {
        ostr << " m_keogramShowPreview: " << m_keogramShowPreview;
    }
    if (settingsKeys.contains("youtubeStreamEnabled") || force) {
        ostr << " m_youtubeStreamEnabled: " << m_youtubeStreamEnabled;
    }
    if (settingsKeys.contains("youtubeStreamUrl") || force) {
        ostr << " m_youtubeStreamUrl: " << m_youtubeStreamUrl.toStdString();
    }
    if (settingsKeys.contains("youtubeStreamKey") || force) {
        ostr << " m_youtubeStreamKeySet: " << !m_youtubeStreamKey.isEmpty();
    }
    if (settingsKeys.contains("youtubeStreamPostProcessed") || force) {
        ostr << " m_youtubeStreamPostProcessed: " << m_youtubeStreamPostProcessed;
    }
    if (settingsKeys.contains("youtubeStreamBitrateKbps") || force) {
        ostr << " m_youtubeStreamBitrateKbps: " << m_youtubeStreamBitrateKbps;
    }
    if (settingsKeys.contains("youtubeStreamFps") || force) {
        ostr << " m_youtubeStreamFps: " << m_youtubeStreamFps;
    }
    if (settingsKeys.contains("youtubeStreamWidth") || force) {
        ostr << " m_youtubeStreamWidth: " << m_youtubeStreamWidth;
    }
    if (settingsKeys.contains("youtubeStreamHeight") || force) {
        ostr << " m_youtubeStreamHeight: " << m_youtubeStreamHeight;
    }
    if (settingsKeys.contains("stackEnabled") || force) {
        ostr << " m_stackEnabled: " << m_stackEnabled;
    }
    if (settingsKeys.contains("stackFrameCount") || force) {
        ostr << " m_stackFrameCount: " << m_stackFrameCount;
    }
    if (settingsKeys.contains("stackMethod") || force) {
        ostr << " m_stackMethod: " << m_stackMethod;
    }
    if (settingsKeys.contains("stackHdrAlgorithm") || force) {
        ostr << " m_stackHdrAlgorithm: " << m_stackHdrAlgorithm;
    }
    if (settingsKeys.contains("stackHdrExposureCount") || force) {
        ostr << " m_stackHdrExposureCount: " << m_stackHdrExposureCount;
    }
    if (settingsKeys.contains("stackHdrExposure1Ms") || force) {
        ostr << " m_stackHdrExposure1Ms: " << m_stackHdrExposureTimesMs[0];
    }
    if (settingsKeys.contains("stackHdrExposure2Ms") || force) {
        ostr << " m_stackHdrExposure2Ms: " << m_stackHdrExposureTimesMs[1];
    }
    if (settingsKeys.contains("stackHdrExposure3Ms") || force) {
        ostr << " m_stackHdrExposure3Ms: " << m_stackHdrExposureTimesMs[2];
    }
    if (settingsKeys.contains("stackHdrExposure4Ms") || force) {
        ostr << " m_stackHdrExposure4Ms: " << m_stackHdrExposureTimesMs[3];
    }
    if (settingsKeys.contains("stackAlignmentMethod") || force) {
        ostr << " m_stackAlignmentMethod: " << m_stackAlignmentMethod;
    }
    if (settingsKeys.contains("stackDisplayMode") || force) {
        ostr << " m_stackDisplayMode: " << m_stackDisplayMode;
    }
    if (settingsKeys.contains("stackDisplayFrameIndex") || force) {
        ostr << " m_stackDisplayFrameIndex: " << m_stackDisplayFrameIndex;
    }
    if (settingsKeys.contains("stackRejectBadFrames") || force) {
        ostr << " m_stackRejectBadFrames: " << m_stackRejectBadFrames;
    }
    if (settingsKeys.contains("scaleEnabled") || force) {
        ostr << " m_scaleEnabled: " << m_scaleEnabled;
    }
    if (settingsKeys.contains("scaleWidth") || force) {
        ostr << " m_scaleWidth: " << m_scaleWidth;
    }
    if (settingsKeys.contains("scaleHeight") || force) {
        ostr << " m_scaleHeight: " << m_scaleHeight;
    }
    if (settingsKeys.contains("scaleKeepAspectRatio") || force) {
        ostr << " m_scaleKeepAspectRatio: " << m_scaleKeepAspectRatio;
    }
    if (settingsKeys.contains("scaleJustification") || force) {
        ostr << " m_scaleJustification: " << static_cast<int>(m_scaleJustification);
    }
    if (settingsKeys.contains("stackDarkFileName") || force) {
        ostr << " m_stackDarkFileName: " << m_stackDarkFileName.toStdString();
    }
    if (settingsKeys.contains("stackFlatFileName") || force) {
        ostr << " m_stackFlatFileName: " << m_stackFlatFileName.toStdString();
    }
    if (settingsKeys.contains("stackBiasFileName") || force) {
        ostr << " m_stackBiasFileName: " << m_stackBiasFileName.toStdString();
    }
    if (settingsKeys.contains("latitude") || force) {
        ostr << " m_latitude: " << m_latitude;
    }
    if (settingsKeys.contains("longitude") || force) {
        ostr << " m_longitude: " << m_longitude;
    }
    if (settingsKeys.contains("altitude") || force) {
        ostr << " m_altitude: " << m_altitude;
    }
    if (settingsKeys.contains("positionSync") || force) {
        ostr << " m_positionSync: " << m_positionSync;
    }
    if (settingsKeys.contains("owmAPIKey") || force) {
        ostr << " m_owmAPIKey: " << m_owmAPIKey.toStdString();
    }
    if (settingsKeys.contains("azimuth") || force) {
        ostr << " m_azimuth: " << m_azimuth;
    }
    if (settingsKeys.contains("elevation") || force) {
        ostr << " m_elevation: " << m_elevation;
    }
    if (settingsKeys.contains("roll") || force) {
        ostr << " m_roll: " << m_roll;
    }
    if (settingsKeys.contains("rotator") || force) {
        ostr << " m_rotator: " << m_rotator.toStdString();
    }
    if (settingsKeys.contains("directionSensor") || force) {
        ostr << " m_directionSensor: " << m_directionSensor.toStdString();
    }
    if (settingsKeys.contains("azimuthOffset") || force) {
        ostr << " m_azimuthOffset: " << m_azimuthOffset;
    }
    if (settingsKeys.contains("elevationOffset") || force) {
        ostr << " m_elevationOffset: " << m_elevationOffset;
    }
    if (settingsKeys.contains("rollOffset") || force) {
        ostr << " m_rollOffset: " << m_rollOffset;
    }
    if (settingsKeys.contains("fov") || force) {
        ostr << " m_fov: " << m_fov;
    }
    if (settingsKeys.contains("fovMode") || force) {
        ostr << " m_fovMode: " << m_fovMode;
    }
    if (settingsKeys.contains("fovSensorWidthMm") || force) {
        ostr << " m_fovSensorWidthMm: " << m_fovSensorWidthMm;
    }
    if (settingsKeys.contains("fovSensorHeightMm") || force) {
        ostr << " m_fovSensorHeightMm: " << m_fovSensorHeightMm;
    }
    if (settingsKeys.contains("fovFocalLengthMm") || force) {
        ostr << " m_fovFocalLengthMm: " << m_fovFocalLengthMm;
    }
    if (settingsKeys.contains("lensProjection") || force) {
        ostr << " m_lensProjection: " << m_lensProjection;
    }
    if (settingsKeys.contains("lensCenterOffsetX") || force) {
        ostr << " m_lensCenterOffsetX: " << m_lensCenterOffsetX;
    }
    if (settingsKeys.contains("lensCenterOffsetY") || force) {
        ostr << " m_lensCenterOffsetY: " << m_lensCenterOffsetY;
    }
    if (settingsKeys.contains("lensDistortionK1") || force) {
        ostr << " m_lensDistortionK1: " << m_lensDistortionK1;
    }
    if (settingsKeys.contains("brightness") || force) {
        ostr << " m_brightness: " << m_brightness;
    }
    if (settingsKeys.contains("postProcessWhiteBalanceMode") || force) {
        ostr << " m_postProcessWhiteBalanceMode: " << m_postProcessWhiteBalanceMode;
    }
    if (settingsKeys.contains("postProcessWhiteBalanceRedGain") || force) {
        ostr << " m_postProcessWhiteBalanceRedGain: " << m_postProcessWhiteBalanceRedGain;
    }
    if (settingsKeys.contains("postProcessWhiteBalanceGreenGain") || force) {
        ostr << " m_postProcessWhiteBalanceGreenGain: " << m_postProcessWhiteBalanceGreenGain;
    }
    if (settingsKeys.contains("postProcessWhiteBalanceBlueGain") || force) {
        ostr << " m_postProcessWhiteBalanceBlueGain: " << m_postProcessWhiteBalanceBlueGain;
    }
    if (settingsKeys.contains("postProcessWhiteBalanceHighlightProtection") || force) {
        ostr << " m_postProcessWhiteBalanceHighlightProtection: " << m_postProcessWhiteBalanceHighlightProtection;
    }
    if (settingsKeys.contains("postProcessUseCuda") || force) {
        ostr << " m_postProcessUseCuda: " << m_postProcessUseCuda;
    }
    if (settingsKeys.contains("postProcessUnwarp") || force) {
        ostr << " m_postProcessUnwarp: " << m_postProcessUnwarp;
    }
    if (settingsKeys.contains("histogramStretch") || force) {
        ostr << " m_histogramStretch: " << m_histogramStretch;
    }
    if (settingsKeys.contains("histogramStretchBlackPoint") || force) {
        ostr << " m_histogramStretchBlackPoint: " << m_histogramStretchBlackPoint;
    }
    if (settingsKeys.contains("histogramStretchWhitePoint") || force) {
        ostr << " m_histogramStretchWhitePoint: " << m_histogramStretchWhitePoint;
    }
    if (settingsKeys.contains("histogramStretchGamma") || force) {
        ostr << " m_histogramStretchGamma: " << m_histogramStretchGamma;
    }
    if (settingsKeys.contains("histogramStretchAsinhStrength") || force) {
        ostr << " m_histogramStretchAsinhStrength: " << m_histogramStretchAsinhStrength;
    }
    if (settingsKeys.contains("histogramStretchLogStrength") || force) {
        ostr << " m_histogramStretchLogStrength: " << m_histogramStretchLogStrength;
    }
    if (settingsKeys.contains("histogramVisible") || force) {
        ostr << " m_histogramVisible: " << m_histogramVisible;
    }
    if (settingsKeys.contains("postProcessGreyscale") || force) {
        ostr << " m_postProcessGreyscale: " << m_postProcessGreyscale;
    }
    if (settingsKeys.contains("saturation") || force) {
        ostr << " m_saturation: " << m_saturation;
    }
    if (settingsKeys.contains("gamma") || force) {
        ostr << " m_gamma: " << m_gamma;
    }
    if (settingsKeys.contains("gaussianBlur") || force) {
        ostr << " m_gaussianBlur: " << m_gaussianBlur;
    }
    if (settingsKeys.contains("medianBlur") || force) {
        ostr << " m_medianBlur: " << m_medianBlur;
    }
    if (settingsKeys.contains("sharpen") || force) {
        ostr << " m_sharpen: " << m_sharpen;
    }
    if (settingsKeys.contains("edgeDisplayMode") || force) {
        ostr << " m_edgeDisplayMode: " << m_edgeDisplayMode;
    }
    if (settingsKeys.contains("sobelEdge") || force) {
        ostr << " m_sobelEdge: " << m_sobelEdge;
    }
    if (settingsKeys.contains("cannyEdge") || force) {
        ostr << " m_cannyEdge: " << m_cannyEdge;
    }
    if (settingsKeys.contains("flipX") || force) {
        ostr << " m_flipX: " << m_flipX;
    }
    if (settingsKeys.contains("flipY") || force) {
        ostr << " m_flipY: " << m_flipY;
    }
    if (settingsKeys.contains("imageRotation") || force) {
        ostr << " m_imageRotation: " << m_imageRotation;
    }
    if (settingsKeys.contains("contrast") || force) {
        ostr << " m_contrast: " << m_contrast;
    }
    if (settingsKeys.contains("invertColors") || force) {
        ostr << " m_invertColors: " << m_invertColors;
    }
    if (settingsKeys.contains("overlayDateTime") || force) {
        ostr << " m_overlayDateTime: " << m_overlayDateTime;
    }
    if (settingsKeys.contains("dateTimeColor") || force) {
        ostr << " m_dateTimeColor: " << m_dateTimeColor.name().toStdString();
    }
    if (settingsKeys.contains("diffMask") || force) {
        ostr << " m_diffMask: " << m_diffMask;
    }
    if (settingsKeys.contains("diffThreshold") || force) {
        ostr << " m_diffThreshold: " << m_diffThreshold;
    }
    if (settingsKeys.contains("diffMaskOpenSize") || force) {
        ostr << " m_diffMaskOpenSize: " << m_diffMaskOpenSize;
    }
    if (settingsKeys.contains("dilationSize") || force) {
        ostr << " m_dilationSize: " << m_dilationSize;
    }
    if (settingsKeys.contains("diffMaskHistoryFrames") || force) {
        ostr << " m_diffMaskHistoryFrames: " << m_diffMaskHistoryFrames;
    }
    if (settingsKeys.contains("diffMaskCloseSize") || force) {
        ostr << " m_diffMaskCloseSize: " << m_diffMaskCloseSize;
    }
    if (settingsKeys.contains("overlayFontFamily") || force) {
        ostr << " m_overlayFontFamily: " << m_overlayFontFamily.toStdString();
    }
    if (settingsKeys.contains("overlayFontScale") || force) {
        ostr << " m_overlayFontScale: " << m_overlayFontScale;
    }
    if (settingsKeys.contains("detectionRoiX") || force) {
        ostr << " m_detectionRoiX: " << m_detectionRoiX;
    }
    if (settingsKeys.contains("detectionRoiY") || force) {
        ostr << " m_detectionRoiY: " << m_detectionRoiY;
    }
    if (settingsKeys.contains("detectionRoiWidth") || force) {
        ostr << " m_detectionRoiWidth: " << m_detectionRoiWidth;
    }
    if (settingsKeys.contains("detectionRoiHeight") || force) {
        ostr << " m_detectionRoiHeight: " << m_detectionRoiHeight;
    }
    if (settingsKeys.contains("showDetectionRoi") || force) {
        ostr << " m_showDetectionRoi: " << m_showDetectionRoi;
    }
    if (settingsKeys.contains("showStarDetectionBoxes") || force) {
        ostr << " m_showStarDetectionBoxes: " << m_showStarDetectionBoxes;
    }
    if (settingsKeys.contains("motionDetect") || force) {
        ostr << " m_motionDetect: " << m_motionDetect;
    }
    if (settingsKeys.contains("motionBackgroundSubtractor") || force) {
        ostr << " m_motionBackgroundSubtractor: " << m_motionBackgroundSubtractor;
    }
    if (settingsKeys.contains("motionMaskView") || force) {
        ostr << " m_motionMaskView: " << m_motionMaskView;
    }
    if (settingsKeys.contains("motionHistory") || force) {
        ostr << " m_motionHistory: " << m_motionHistory;
    }
    if (settingsKeys.contains("motionVarThreshold") || force) {
        ostr << " m_motionVarThreshold: " << m_motionVarThreshold;
    }
    if (settingsKeys.contains("motionLearningRate") || force) {
        ostr << " m_motionLearningRate: " << m_motionLearningRate;
    }
    if (settingsKeys.contains("motionConfirmFrames") || force) {
        ostr << " m_motionConfirmFrames: " << m_motionConfirmFrames;
    }
    if (settingsKeys.contains("motionDownscale") || force) {
        ostr << " m_motionDownscale: " << m_motionDownscale;
    }
    if (settingsKeys.contains("motionDetectShadows") || force) {
        ostr << " m_motionDetectShadows: " << m_motionDetectShadows;
    }
    if (settingsKeys.contains("motionOpenSize") || force) {
        ostr << " m_motionOpenSize: " << m_motionOpenSize;
    }
    if (settingsKeys.contains("motionCloseSize") || force) {
        ostr << " m_motionCloseSize: " << m_motionCloseSize;
    }
    if (settingsKeys.contains("motionPersistenceFrames") || force) {
        ostr << " m_motionPersistenceFrames: " << m_motionPersistenceFrames;
    }
    if (settingsKeys.contains("minContourArea") || force) {
        ostr << " m_minContourArea: " << m_minContourArea;
    }
    if (settingsKeys.contains("motionExclusionRects") || force) {
        ostr << " m_motionExclusionRects: " << m_motionExclusionRects.size();
    }
    if (settingsKeys.contains("starDetect") || force) {
        ostr << " m_starDetect: " << m_starDetect;
    }
    if (settingsKeys.contains("starThreshold") || force) {
        ostr << " m_starThreshold: " << m_starThreshold;
    }
    if (settingsKeys.contains("starBackgroundBlur") || force) {
        ostr << " m_starBackgroundBlur: " << m_starBackgroundBlur;
    }
    if (settingsKeys.contains("starMinArea") || force) {
        ostr << " m_starMinArea: " << m_starMinArea;
    }
    if (settingsKeys.contains("starMaxArea") || force) {
        ostr << " m_starMaxArea: " << m_starMaxArea;
    }
    if (settingsKeys.contains("starMaxAspectRatio") || force) {
        ostr << " m_starMaxAspectRatio: " << m_starMaxAspectRatio;
    }
    if (settingsKeys.contains("starDebugView") || force) {
        ostr << " m_starDebugView: " << m_starDebugView;
    }
    if (settingsKeys.contains("starColor") || force) {
        ostr << " m_starColor: " << m_starColor.name().toStdString();
    }
    if (settingsKeys.contains("plateSolve") || force) {
        ostr << " m_plateSolve: " << m_plateSolve;
    }
    if (settingsKeys.contains("plateSolveMaxMagnitude") || force) {
        ostr << " m_plateSolveMaxMagnitude: " << m_plateSolveMaxMagnitude;
    }
    if (settingsKeys.contains("plateSolveMinMatches") || force) {
        ostr << " m_plateSolveMinMatches: " << m_plateSolveMinMatches;
    }
    if (settingsKeys.contains("plateSolveMatchRadius") || force) {
        ostr << " m_plateSolveMatchRadius: " << m_plateSolveMatchRadius;
    }
    if (settingsKeys.contains("plateSolveFinalMatchRadius") || force) {
        ostr << " m_plateSolveFinalMatchRadius: " << m_plateSolveFinalMatchRadius;
    }
    if (settingsKeys.contains("plateSolveSearchRadius") || force) {
        ostr << " m_plateSolveAzElSearchRadius: " << m_plateSolveAzElSearchRadius;
    }
    if (settingsKeys.contains("plateSolveFovTolerance") || force) {
        ostr << " m_plateSolveFovTolerance: " << m_plateSolveFovTolerance;
    }
    if (settingsKeys.contains("plateSolveStartMode") || force) {
        ostr << " m_plateSolveStartMode: " << static_cast<int>(m_plateSolveStartMode);
    }
    if (settingsKeys.contains("plateSolveUseCaptureDateTime") || force) {
        ostr << " m_plateSolveUseCaptureDateTime: " << m_plateSolveUseCaptureDateTime;
    }
    if (settingsKeys.contains("plateSolveDateTime") || force) {
        ostr << " m_plateSolveDateTime: " << m_plateSolveDateTime.toString(Qt::ISODateWithMs).toStdString();
    }
    if (settingsKeys.contains("plateSolveDateTimeUtc") || force) {
        ostr << " m_plateSolveDateTimeUtc: " << m_plateSolveDateTimeUtc;
    }
    if (settingsKeys.contains("plateSolveUseDownloadedCatalog") || force) {
        ostr << " m_plateSolveUseDownloadedCatalog: " << m_plateSolveUseDownloadedCatalog;
    }
    if (settingsKeys.contains("plateSolveCatalogSource") || force) {
        ostr << " m_plateSolveCatalogSource: " << static_cast<int>(m_plateSolveCatalogSource);
    }
    if (settingsKeys.contains("plateSolveLabelMode") || force) {
        ostr << " m_plateSolveLabelMode: " << static_cast<int>(m_plateSolveLabelMode);
    }
    if (settingsKeys.contains("plateSolveLabelHideSyntheticNames") || force) {
        ostr << " m_plateSolveLabelHideSyntheticNames: " << m_plateSolveLabelHideSyntheticNames;
    }
    if (settingsKeys.contains("plateSolveApplyMode") || force) {
        ostr << " m_plateSolveApplyMode: " << static_cast<int>(m_plateSolveApplyMode);
    }
    if (settingsKeys.contains("starCatalogDiskCacheSizeGb") || force) {
        ostr << " m_starCatalogDiskCacheSizeGb: " << m_starCatalogDiskCacheSizeGb;
    }
    if (settingsKeys.contains("recordRawFits") || force) {
        ostr << " m_recordRawFits: " << m_recordRawFits;
    }
    if (settingsKeys.contains("recordCalibratedMedia") || force) {
        ostr << " m_recordCalibratedMedia: " << m_recordCalibratedMedia;
    }
    if (settingsKeys.contains("recordFilteredMedia") || force) {
        ostr << " m_recordFilteredMedia: " << m_recordFilteredMedia;
    }
    if (settingsKeys.contains("recordPostProcessedMedia") || force) {
        ostr << " m_recordPostProcessedMedia: " << m_recordPostProcessedMedia;
    }
    if (settingsKeys.contains("overlaySpectrum") || force) {
        ostr << " m_overlaySpectrum: " << m_overlaySpectrum;
    }
    if (settingsKeys.contains("spectrumDevice") || force) {
        ostr << " m_spectrumDevice: " << m_spectrumDevice.toStdString();
    }
    if (settingsKeys.contains("spectrumOffsetX") || force) {
        ostr << " m_spectrumOffsetX: " << m_spectrumOffsetX;
    }
    if (settingsKeys.contains("spectrumOffsetY") || force) {
        ostr << " m_spectrumOffsetY: " << m_spectrumOffsetY;
    }
    if (settingsKeys.contains("spectrumScale") || force) {
        ostr << " m_spectrumScale: " << m_spectrumScale;
    }
    if (settingsKeys.contains("dateTimeFormat") || force) {
        ostr << " m_dateTimeFormat: " << m_dateTimeFormat.toStdString();
    }
    if (settingsKeys.contains("dateTimePosX") || force) {
        ostr << " m_dateTimePosX: " << m_dateTimePosX;
    }
    if (settingsKeys.contains("dateTimePosY") || force) {
        ostr << " m_dateTimePosY: " << m_dateTimePosY;
    }
    if (settingsKeys.contains("equatorialGrid") || force) {
        ostr << " m_equatorialGrid: " << m_equatorialGrid;
    }
    if (settingsKeys.contains("equatorialGridColor") || force) {
        ostr << " m_equatorialGridColor: " << m_equatorialGridColor.name().toStdString();
    }
    if (settingsKeys.contains("altAzGrid") || force) {
        ostr << " m_altAzGrid: " << m_altAzGrid;
    }
    if (settingsKeys.contains("altAzGridColor") || force) {
        ostr << " m_altAzGridColor: " << m_altAzGridColor.name().toStdString();
    }
    if (settingsKeys.contains("constellation") || force) {
        ostr << " m_constellation: " << m_constellation;
    }
    if (settingsKeys.contains("constellationColor") || force) {
        ostr << " m_constellationColor: " << m_constellationColor.name().toStdString();
    }
    if (settingsKeys.contains("constellationOverlay") || force) {
        ostr << " m_constellationOverlay: " << static_cast<int>(m_constellationOverlay);
    }
    if (settingsKeys.contains("trackObjects") || force) {
        ostr << " m_trackObjects: " << m_trackObjects;
    }
    if (settingsKeys.contains("trackObjectTrails") || force) {
        ostr << " m_trackObjectTrails: " << m_trackObjectTrails;
    }
    if (settingsKeys.contains("trackObjectHeatMap") || force) {
        ostr << " m_trackObjectHeatMap: " << m_trackObjectHeatMap;
    }
    if (settingsKeys.contains("trackObjectRange") || force) {
        ostr << " m_trackObjectRange: " << m_trackObjectRange;
    }
    if (settingsKeys.contains("trackObjectMinElevation") || force) {
        ostr << " m_trackObjectMinElevation: " << m_trackObjectMinElevation;
    }
    if (settingsKeys.contains("trackObjectColor") || force) {
        ostr << " m_trackObjectColor: " << m_trackObjectColor.name().toStdString();
    }
    if (settingsKeys.contains("trackObjectFontFamily") || force) {
        ostr << " m_trackObjectFontFamily: " << m_trackObjectFontFamily.toStdString();
    }
    if (settingsKeys.contains("trackObjectFontScale") || force) {
        ostr << " m_trackObjectFontScale: " << m_trackObjectFontScale;
    }
    if (settingsKeys.contains("gridLabelFontFamily") || force) {
        ostr << " m_gridLabelFontFamily: " << m_gridLabelFontFamily.toStdString();
    }
    if (settingsKeys.contains("gridLabelFontScale") || force) {
        ostr << " m_gridLabelFontScale: " << m_gridLabelFontScale;
    }
    if (settingsKeys.contains("overlayText") || force) {
        ostr << " m_overlayText: " << m_overlayText;
    }
    if (settingsKeys.contains("overlayTextString") || force) {
        ostr << " m_overlayTextString: " << m_overlayTextString.toStdString();
    }
    if (settingsKeys.contains("overlayTextColor") || force) {
        ostr << " m_overlayTextColor: " << m_overlayTextColor.name().toStdString();
    }
    if (settingsKeys.contains("overlayTextFontFamily") || force) {
        ostr << " m_overlayTextFontFamily: " << m_overlayTextFontFamily.toStdString();
    }
    if (settingsKeys.contains("overlayTextFontScale") || force) {
        ostr << " m_overlayTextFontScale: " << m_overlayTextFontScale;
    }
    if (settingsKeys.contains("overlayTextPosX") || force) {
        ostr << " m_overlayTextPosX: " << m_overlayTextPosX;
    }
    if (settingsKeys.contains("overlayTextPosY") || force) {
        ostr << " m_overlayTextPosY: " << m_overlayTextPosY;
    }
    if (settingsKeys.contains("yoloEnabled") || force) {
        ostr << " m_yoloEnabled: " << m_yoloEnabled;
    }
    if (settingsKeys.contains("yoloModelPath") || force) {
        ostr << " m_yoloModelPath: " << m_yoloModelPath.toStdString();
    }
    if (settingsKeys.contains("yoloLabelsPath") || force) {
        ostr << " m_yoloLabelsPath: " << m_yoloLabelsPath.toStdString();
    }
    if (settingsKeys.contains("yoloConfThreshold") || force) {
        ostr << " m_yoloConfThreshold: " << m_yoloConfThreshold;
    }
    if (settingsKeys.contains("yoloNmsThreshold") || force) {
        ostr << " m_yoloNmsThreshold: " << m_yoloNmsThreshold;
    }
    if (settingsKeys.contains("yoloTileLargeImages") || force) {
        ostr << " m_yoloTileLargeImages: " << m_yoloTileLargeImages;
    }
    if (settingsKeys.contains("yoloTileOverlapPercent") || force) {
        ostr << " m_yoloTileOverlapPercent: " << m_yoloTileOverlapPercent;
    }
    if (settingsKeys.contains("yoloIgnoredClassNames") || force) {
        ostr << " m_yoloIgnoredClassNames: [ " << m_yoloIgnoredClassNames.join(", ").toStdString() << " ]";
    }
    if (settingsKeys.contains("yoloDnnTarget") || force) {
        ostr << " m_yoloDnnTarget: " << m_yoloDnnTarget;
    }
    if (settingsKeys.contains("audioMute") || force) {
        ostr << " m_audioMute: " << m_audioMute;
    }
    if (settingsKeys.contains("audioPreviewVolume") || force) {
        ostr << " m_audioPreviewVolume: " << m_audioPreviewVolume;
    }
    if (settingsKeys.contains("audioDeviceName") || force) {
        ostr << " m_audioDeviceName: " << m_audioDeviceName.toStdString();
    }
    if (settingsKeys.contains("whiteBalanceMode") || force) {
        ostr << " m_whiteBalanceMode: " << m_whiteBalanceMode;
    }
    if (settingsKeys.contains("exposureCompensation") || force) {
        ostr << " m_exposureCompensation: " << m_exposureCompensation;
    }
    if (settingsKeys.contains("focusMode") || force) {
        ostr << " m_focusMode: " << m_focusMode;
    }
    if (settingsKeys.contains("focusDistance") || force) {
        ostr << " m_focusDistance: " << m_focusDistance;
    }
    if (settingsKeys.contains("zoomFactor") || force) {
        ostr << " m_zoomFactor: " << m_zoomFactor;
    }

    return QString(ostr.str().c_str());
}

bool CameraSettings::isAlpacaCamera() const
{
    return m_cameraProtocol == CameraProtocol::alpaca();
}

bool CameraSettings::isAsiCamera() const
{
    return m_cameraProtocol == CameraProtocol::asi();
}

bool CameraSettings::isQtCamera() const
{
    return m_cameraProtocol == CameraProtocol::qt();
}

bool CameraSettings::isFileCamera() const
{
    return isVideoFileCamera() || isImageFileSequenceCamera() || isStreamCamera();
}

bool CameraSettings::isVideoFileCamera() const
{
    return m_cameraProtocol == CameraProtocol::video();
}

bool CameraSettings::isImageFileSequenceCamera() const
{
    return m_cameraProtocol == CameraProtocol::images();
}

bool CameraSettings::isStreamCamera() const
{
    return m_cameraProtocol == CameraProtocol::stream();
}

bool CameraSettings::isFfmpegMediaSource() const
{
    return isVideoFileCamera() || isStreamCamera();
}

bool CameraSettings::hasFileCameraSource() const
{
    return (isFfmpegMediaSource() && !ffmpegMediaSourcePath().isEmpty())
        || (isImageFileSequenceCamera() && !m_imageFileCameraPaths.isEmpty());
}

QString CameraSettings::ffmpegMediaSourcePath() const
{
    return isStreamCamera() ? m_streamUrl : m_videoFileCameraPath;
}

int CameraSettings::cameraIdInt() const
{
    if (isAlpacaCamera() || isAsiCamera())
    {
        bool ok;
        int id = m_cameraId.toInt(&ok);
        if (ok) {
            return id;
        }
    }
    return -1;
}

QString CameraSettings::cameraIdString() const
{
    return m_cameraId;
}

QString CameraSettings::cameraDescription() const
{
    return m_cameraDescription;
}

QString CameraSettings::cameraDisplayName() const
{
    if (isImageFileSequenceCamera())
    {
        if (m_imageFileCameraPaths.isEmpty()) {
            return CameraProtocol::playbackDisplayText(CameraProtocol::images(), QString());
        }
        const QString description = QStringLiteral("%1 image%2")
            .arg(m_imageFileCameraPaths.size())
            .arg(m_imageFileCameraPaths.size() == 1 ? QString() : QStringLiteral("s"));
        return CameraProtocol::playbackDisplayText(CameraProtocol::images(), description);
    }

    if (isFfmpegMediaSource()) {
        return CameraProtocol::playbackDisplayText(m_cameraProtocol, m_cameraDescription);
    }

    if (!m_cameraProtocol.isEmpty() && !m_cameraDescription.isEmpty()) {
        return QString("%1:%2").arg(m_cameraProtocol, m_cameraDescription);
    }

    if (!m_cameraDescription.isEmpty()) {
        return m_cameraDescription;
    }

    if (!m_cameraProtocol.isEmpty()) {
        return m_cameraProtocol;
    }

    return m_cameraId;
}

bool CameraSettings::isIntervalCaptureMode() const
{
    return m_captureMode == CaptureModeInterval;
}

double CameraSettings::getCaptureIntervalSeconds() const
{
    const double interval = std::max(m_minCaptureInterval, m_captureInterval);
    return m_captureIntervalUnits == CaptureIntervalMinutes ? interval * 60.0 : interval;
}

int CameraSettings::getCaptureIntervalMs() const
{
    return std::max(100, static_cast<int>(std::lround(getCaptureIntervalSeconds() * 1000.0)));
}

double CameraSettings::getCaptureFrameRate() const
{
    if (isImageFileSequenceCamera()) {
        return qBound(m_minVideoPlaybackRate, m_videoPlaybackRate, m_maxVideoPlaybackRate);
    }

    if (!isIntervalCaptureMode()) {
        return std::max(m_minFramesPerSecond, m_framesPerSecond);
    }

    return std::max(m_minExposureTimeMs, 1.0 / getCaptureIntervalSeconds());
}

// Map URL to filename where it will be downloaded
QString CameraSettings::urlToFilename(const QString &url, const QString& destSubDir)
{
    if (url.startsWith("http://") || url.startsWith("https://"))
    {
        QString dirPath = HttpDownloadManager::downloadDir() + "/" + destSubDir;

        return dirPath + "/" + QUrl(url).fileName();
    }
    else
    {
        return url;
    }
}
