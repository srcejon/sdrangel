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
#include <sstream>

#include "util/simpleserializer.h"
#include "util/httpdownloadmanager.h"
#include "settings/serializable.h"
#include "maincore.h"
#include "camerasettings.h"

#define DEFAULT_OVERLAY_TEXT_STRING "<img src=\":/sdrangel_icon.png\"><h1 style=\"color:blue\">SDRangel</h1>\n<p>\nText overlay "

QDataStream& operator<<(QDataStream& out, const CameraSettings::ObjectDeviceSettings* settings)
{
    out << settings->m_deviceSetIndex;
    out << settings->m_presetGroup;
    out << settings->m_presetFrequency;
    out << settings->m_presetDescription;
    out << settings->m_startOnDetect;
    out << settings->m_stopOnDisappear;
    out << settings->m_startStopFileSink;
    out << settings->m_detectCommand;
    out << settings->m_disappearCommand;
    out << settings->m_recordVideo;
    out << settings->m_detectSpeech;
    out << settings->m_disappearSpeech;
    out << settings->m_saveCurrentImage;
    return out;
}

QDataStream& operator>>(QDataStream& in, CameraSettings::ObjectDeviceSettings*& settings)
{
    settings = new CameraSettings::ObjectDeviceSettings();
    in >> settings->m_deviceSetIndex;
    in >> settings->m_presetGroup;
    in >> settings->m_presetFrequency;
    in >> settings->m_presetDescription;
    in >> settings->m_startOnDetect;
    in >> settings->m_stopOnDisappear;
    in >> settings->m_startStopFileSink;
    in >> settings->m_detectCommand;
    in >> settings->m_disappearCommand;
    if (!in.atEnd()) {
        in >> settings->m_recordVideo;
    }
    if (!in.atEnd()) {
        in >> settings->m_detectSpeech;
    }
    if (!in.atEnd()) {
        in >> settings->m_disappearSpeech;
    }
    if (!in.atEnd()) {
        in >> settings->m_saveCurrentImage;
    }
    return in;
}

QDataStream& operator<<(QDataStream& out, const QList<CameraSettings::ObjectDeviceSettings *> *list)
{
    out << *list;
    return out;
}

QDataStream& operator>>(QDataStream& in, QList<CameraSettings::ObjectDeviceSettings *>*& list)
{
    list = new QList<CameraSettings::ObjectDeviceSettings *>();
    in >> *list;
    return in;
}

CameraSettings::ObjectDeviceSettings::ObjectDeviceSettings() :
    m_deviceSetIndex(0),
    m_presetFrequency(0),
    m_startOnDetect(true),
    m_stopOnDisappear(true),
    m_startStopFileSink(false),
    m_saveCurrentImage(false),
    m_recordVideo(false)
{
    m_detectCommand.clear();
    m_disappearCommand.clear();
    m_detectSpeech.clear();
    m_disappearSpeech.clear();
}

void CameraSettings::ObjectDeviceSettings::getDebugString(std::ostringstream& ostr) const
{
    ostr << "{"
         << " deviceSetIndex: " << m_deviceSetIndex
         << " presetGroup: " << m_presetGroup.toStdString()
         << " presetFrequency: " << m_presetFrequency
         << " presetDescription: " << m_presetDescription.toStdString()
         << " startOnDetect: " << m_startOnDetect
         << " stopOnDisappear: " << m_stopOnDisappear
         << " startStopFileSink: " << m_startStopFileSink
         << " saveCurrentImage: " << m_saveCurrentImage
         << " recordVideo: " << m_recordVideo
         << " detectCommand: " << m_detectCommand.toStdString()
         << " disappearCommand: " << m_disappearCommand.toStdString()
         << " detectSpeech: " << m_detectSpeech.toStdString()
         << " disappearSpeech: " << m_disappearSpeech.toStdString()
         << " }";
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
    m_alpacaDiscoveryEnabled = false;
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
    m_asiColorImageType = AsiColorImageTypeRgb24;
    m_saveImage = false;
    m_imageFileName = "camera.jpg";
    m_saveVideo = false;
    m_videoFileCameraPath.clear();
    m_videoFileName = "camera.mp4";
    m_videoLoop = false;
    m_videoPlaybackRate = 1.0;
    m_videoHwAcceleration = true;
    m_stackEnabled = false;
    m_stackFrameCount = 4;
    m_stackMethod = StackMethodAverage;
    m_stackAlignmentMethod = StackAlignmentNone;
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
    m_fov = 60.0f;
    m_lensProjection = LensProjectionRectilinear;
    m_scheduleEnabled = false;
    m_scheduleStartTime = QStringLiteral("20:00:00");
    m_scheduleEndTime = QStringLiteral("06:00:00");
    m_scheduleWeekdays = 0x7f;
    m_workspaceIndex = 0;
    m_geometryBytes.clear();
    m_postProcessWhiteBalanceMode = 0;
    m_postProcessWhiteBalanceRedGain = 1.0;
    m_postProcessWhiteBalanceGreenGain = 1.0;
    m_postProcessWhiteBalanceBlueGain = 1.0;
    m_postProcessUnwarp = false;
    m_histogramStretch = HistogramStretchOff;
    m_histogramStretchBlackPoint = 0.0;
    m_histogramStretchWhitePoint = 1.0;
    m_histogramStretchGamma = 1.0;
    m_histogramStretchAsinhStrength = 10.0;
    m_histogramStretchLogStrength = 10.0;
    m_postProcessGreyscale = false;
    m_saturation = 1.0;
    m_gamma = 1.0;
    m_gaussianBlur = 0;
    m_medianBlur = 0;
    m_sharpen = 0.0;
    m_edgeDisplayMode = EdgeDisplayOverlay;
    m_sobelEdge = 0.0;
    m_cannyEdge = 0.0;
    m_lineEnhancement = 0.0;
    m_ridgeDetection = 0.0;
    m_ridgeDetectionKernelSize = 3;
    m_ridgeDetectionScale = 1.0;
    m_ridgeDetectionDelta = 0.0;
    m_flipX = false;
    m_flipY = false;
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
    m_trackObjectMinElevation = 0.0;
    m_trackObjectColor = QColor(80, 255, 80);
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
    m_streakDetect = false;
    m_streakThreshold = 24;
    m_streakMinLength = 80;
    m_streakHoughThreshold = 30;
    m_streakMaxGap = 12.0;
    m_streakPersistenceFrames = 1;
    m_streakDownscale = 0.5;
    m_streakDebugView = StreakDebugViewOff;
    m_streakOverlayStyle = StreakOverlayStyleLines;
    m_streakLineEnhancementPlacement = StreakLineEnhancementOff;
    m_streakColor = QColor(255, 255, 80);
    m_starDetect = false;
    m_starThreshold = 24;
    m_starBackgroundBlur = 12;
    m_starMinArea = 1;
    m_starMaxArea = 36;
    m_starMaxAspectRatio = 2.5;
    m_starDebugView = StarDebugViewOff;
    m_starColor = QColor(120, 255, 255);
    m_plateSolve = false;
    m_plateSolveMaxMagnitude = 3.5;
    m_plateSolveMinMatches = 4;
    m_plateSolveMatchRadius = 24.0;
    m_plateSolveSearchRadius = 12.0;
    m_plateSolveUseCurrentDateTime = true;
    m_plateSolveDateTime = QDateTime::currentDateTime();
    m_plateSolveUseDownloadedCatalog = false;
    m_recordMode = SavedMediaRaw;
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
    m_yoloDisappearDebounce = 0.0;
    m_yoloDnnTarget = CPU;
    m_audioMute = true;
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
    s.writeBool(188, m_videoLoop);
    s.writeDouble(189, m_videoPlaybackRate);
    s.writeBool(130, m_stackEnabled);
    s.writeS32(131, m_stackFrameCount);
    s.writeS32(132, m_stackMethod);
    s.writeS32(133, m_stackAlignmentMethod);
    s.writeString(134, m_stackDarkFileName);
    s.writeString(135, m_stackFlatFileName);
    s.writeString(136, m_stackBiasFileName);
    s.writeFloat(137, m_latitude);
    s.writeFloat(138, m_longitude);
    s.writeFloat(139, m_altitude);
    s.writeBool(140, m_positionSync);
    s.writeString(164, m_owmAPIKey);
    s.writeFloat(141, m_azimuth);
    s.writeFloat(142, m_elevation);
    s.writeFloat(150, m_roll);
    s.writeString(143, m_rotator);
    s.writeFloat(144, m_fov);
    s.writeS32(149, m_lensProjection);
    s.writeBool(151, m_scheduleEnabled);
    s.writeString(152, m_scheduleStartTime);
    s.writeString(153, m_scheduleEndTime);
    s.writeS32(154, m_scheduleWeekdays);

    if (m_rollupState) {
        s.writeBlob(19, m_rollupState->serialize());
    }

    s.writeS32(20, m_workspaceIndex);
    s.writeBlob(21, m_geometryBytes);
    s.writeS32(22, m_cameraBinX);
    s.writeS32(23, m_cameraBinY);
    s.writeS32(24, m_cameraGain);
    s.writeS32(25, m_cameraReadoutMode);
    s.writeS32(26, m_cameraOffset);
    s.writeS32(27, m_cameraNumX);
    s.writeS32(28, m_cameraNumY);
    s.writeS32(29, m_cameraStartX);
    s.writeS32(30, m_cameraStartY);
    s.writeS32(31, m_postProcessWhiteBalanceMode);
    s.writeDouble(32, m_postProcessWhiteBalanceRedGain);
    s.writeDouble(33, m_postProcessWhiteBalanceGreenGain);
    s.writeDouble(34, m_postProcessWhiteBalanceBlueGain);
    s.writeBool(157, m_postProcessUnwarp);
    s.writeS32(158, static_cast<qint32>(m_histogramStretch));
    s.writeDouble(159, m_histogramStretchBlackPoint);
    s.writeDouble(160, m_histogramStretchWhitePoint);
    s.writeDouble(161, m_histogramStretchGamma);
    s.writeDouble(162, m_histogramStretchAsinhStrength);
    s.writeDouble(163, m_histogramStretchLogStrength);
    s.writeBool(127, m_postProcessGreyscale);
    s.writeDouble(35, m_saturation);
    s.writeDouble(36, m_gamma);
    s.writeS32(37, m_gaussianBlur);
    s.writeS32(38, m_medianBlur);
    s.writeDouble(39, m_sharpen);
    s.writeS32(191, static_cast<qint32>(m_edgeDisplayMode));
    s.writeDouble(40, m_sobelEdge);
    s.writeDouble(190, m_cannyEdge);
    s.writeDouble(192, m_lineEnhancement);
    s.writeDouble(193, m_ridgeDetection);
    s.writeS32(195, m_ridgeDetectionKernelSize);
    s.writeDouble(196, m_ridgeDetectionScale);
    s.writeDouble(197, m_ridgeDetectionDelta);
    s.writeBool(41, m_flipX);
    s.writeBool(42, m_flipY);
    s.writeDouble(43, m_brightness);
    s.writeDouble(44, m_contrast);
    s.writeBool(45, m_invertColors);
    s.writeBool(46, m_overlayDateTime);
    s.writeU32(47, m_dateTimeColor.rgba());
    s.writeBool(48, m_diffMask);
    s.writeS32(49, m_diffThreshold);
    s.writeS32(50, m_dilationSize);
    s.writeS32(51, m_diffMaskHistoryFrames);
    s.writeS32(52, m_diffMaskCloseSize);
    s.writeString(53, m_overlayFontFamily);
    s.writeDouble(54, m_overlayFontScale);
    s.writeS32(55, m_detectionRoiX);
    s.writeS32(56, m_detectionRoiY);
    s.writeS32(57, m_detectionRoiWidth);
    s.writeS32(58, m_detectionRoiHeight);
    s.writeBool(201, m_showDetectionRoi);
    s.writeBool(59, m_motionDetect);
    s.writeS32(175, static_cast<qint32>(m_motionBackgroundSubtractor));
    s.writeS32(176, static_cast<qint32>(m_motionMaskView));
    s.writeS32(60, m_motionHistory);
    s.writeDouble(61, m_motionVarThreshold);
    s.writeBool(62, m_motionDetectShadows);
    s.writeS32(63, m_motionOpenSize);
    s.writeS32(64, m_motionCloseSize);
    s.writeS32(65, m_motionPersistenceFrames);
    s.writeU32(66, m_motionBoxColor.rgba());
    s.writeS32(67, m_minContourArea);
    s.writeDouble(171, m_motionLearningRate);
    s.writeS32(172, m_motionConfirmFrames);
    s.writeDouble(173, m_motionDownscale);
    s.writeBlob(174, serializeMotionExclusionRects(m_motionExclusionRects));
    s.writeBool(186, m_showMotionExclusionRects);
    s.writeBool(177, m_streakDetect);
    s.writeS32(178, m_streakThreshold);
    s.writeS32(179, m_streakMinLength);
    s.writeS32(180, m_streakHoughThreshold);
    s.writeDouble(181, m_streakMaxGap);
    s.writeS32(182, m_streakPersistenceFrames);
    s.writeDouble(183, m_streakDownscale);
    s.writeU32(184, m_streakColor.rgba());
    s.writeS32(185, static_cast<qint32>(m_streakDebugView));
    s.writeS32(187, static_cast<qint32>(m_streakOverlayStyle));
    s.writeS32(194, static_cast<qint32>(m_streakLineEnhancementPlacement));
    s.writeBool(202, m_starDetect);
    s.writeS32(203, m_starThreshold);
    s.writeS32(204, m_starBackgroundBlur);
    s.writeS32(205, m_starMinArea);
    s.writeS32(206, m_starMaxArea);
    s.writeDouble(207, m_starMaxAspectRatio);
    s.writeS32(208, static_cast<qint32>(m_starDebugView));
    s.writeU32(209, m_starColor.rgba());
    s.writeBool(210, m_plateSolve);
    s.writeDouble(211, m_plateSolveMaxMagnitude);
    s.writeS32(212, m_plateSolveMinMatches);
    s.writeDouble(213, m_plateSolveMatchRadius);
    s.writeDouble(214, m_plateSolveSearchRadius);
    s.writeBool(215, m_plateSolveUseDownloadedCatalog);
    s.writeBool(216, m_plateSolveUseCurrentDateTime);
    s.writeS64(217, m_plateSolveDateTime.isValid() ? m_plateSolveDateTime.toMSecsSinceEpoch() : 0);
    s.writeBool(68, m_recordMode != SavedMediaRaw);
    s.writeBool(69, m_overlaySpectrum);
    s.writeString(70, m_spectrumDevice);
    s.writeS32(71, m_spectrumOffsetX);
    s.writeS32(72, m_spectrumOffsetY);
    s.writeDouble(73, m_spectrumScale);
    s.writeString(74, m_dateTimeFormat);
    s.writeS32(75, m_dateTimePosX);
    s.writeS32(76, m_dateTimePosY);
    s.writeBool(77, m_overlayText);
    s.writeString(78, m_overlayTextString);
    s.writeU32(79, m_overlayTextColor.rgba());
    s.writeString(80, m_overlayTextFontFamily);
    s.writeDouble(81, m_overlayTextFontScale);
    s.writeS32(82, m_overlayTextPosX);
    s.writeS32(83, m_overlayTextPosY);
    s.writeBool(145, m_equatorialGrid);
    s.writeU32(146, m_equatorialGridColor.rgba());
    s.writeBool(147, m_altAzGrid);
    s.writeU32(148, m_altAzGridColor.rgba());
    s.writeBool(198, m_constellation);
    s.writeU32(199, m_constellationColor.rgba());
    s.writeS32(200, static_cast<qint32>(m_constellationOverlay));
    s.writeBool(165, m_trackObjects);
    s.writeDouble(166, m_trackObjectMinElevation);
    s.writeU32(169, m_trackObjectColor.rgba());
    s.writeDouble(170, m_trackObjectFontScale);
    s.writeString(155, m_gridLabelFontFamily);
    s.writeDouble(156, m_gridLabelFontScale);
    s.writeBool(84, m_yoloEnabled);
    s.writeString(85, m_yoloModelPath);
    s.writeString(86, m_yoloLabelsPath);
    s.writeDouble(87, m_yoloConfThreshold);
    s.writeDouble(88, m_yoloNmsThreshold);
    s.writeU32(89, m_yoloBoxColor.rgba());
    s.writeDouble(90, m_yoloDisappearDebounce);
    s.writeS32(91, m_yoloDnnTarget);
    s.writeBlob(92, serializeObjectDeviceSettings(m_objectDeviceSettings));
    s.writeBool(93, m_audioMute);
    s.writeString(94, m_audioDeviceName);
    s.writeS32(95, m_whiteBalanceMode);
    s.writeDouble(96, m_exposureCompensation);
    s.writeS32(97, m_focusMode);
    s.writeDouble(98, m_focusDistance);
    s.writeDouble(99, m_zoomFactor);
    s.writeDouble(100, m_captureInterval);
    s.writeS32(101, m_captureIntervalUnits);
    s.writeBool(102, m_alpacaDiscoveryEnabled);
    s.writeBool(103, m_alpacaApiLogEnabled);
    s.writeBool(104, m_alpacaFocuserEnabled);
    s.writeString(105, m_alpacaFocuserHost);
    s.writeU32(106, m_alpacaFocuserPort);
    s.writeS32(107, m_alpacaFocuserDeviceNumber);
    s.writeS32(108, m_alpacaFocusPosition);
    s.writeS32(109, m_alpacaFocusStepSize);
    s.writeBool(110, m_alpacaFilterWheelEnabled);
    s.writeString(111, m_alpacaFilterWheelHost);
    s.writeU32(112, m_alpacaFilterWheelPort);
    s.writeS32(113, m_alpacaFilterWheelDeviceNumber);
    s.writeS32(114, m_alpacaFilterWheelPosition);
    s.writeString(115, m_videoFileCameraPath);

    s.writeBool(116, m_useReverseAPI);
    s.writeString(117, m_reverseAPIAddress);
    s.writeU32(118, m_reverseAPIPort);
    s.writeU32(119, m_reverseAPIFeatureSetIndex);
    s.writeU32(120, m_reverseAPIFeatureIndex);
    s.writeS32(121, m_asiCoolerOn);
    s.writeS32(122, m_asiTargetTemp);
    s.writeS32(123, m_asiUsbBandwidth);
    s.writeS32(124, m_asiHighSpeedMode);
    s.writeS32(125, m_asiColorImageType);
    s.writeS32(126, m_diffMaskOpenSize);
    s.writeS32(128, m_recordMode);
    s.writeBool(168, m_asiAutoExposureGain);

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
        d.readDouble(100, &m_captureInterval, 1.0);
        d.readS32(101, (qint32 *) &m_captureIntervalUnits, (qint32) CaptureIntervalSeconds);
        d.readBool(102, &m_alpacaDiscoveryEnabled, false);
        d.readBool(103, &m_alpacaApiLogEnabled, false);
        d.readBool(104, &m_alpacaFocuserEnabled, false);
        d.readString(105, &m_alpacaFocuserHost, "127.0.0.1");
        d.readU32(106, &utmp, 11111);
        m_alpacaFocuserPort = (utmp <= 65535) ? static_cast<uint16_t>(utmp) : 11111;
        d.readS32(107, &m_alpacaFocuserDeviceNumber, 0);
        d.readS32(108, &m_alpacaFocusPosition, 0);
        d.readS32(109, &m_alpacaFocusStepSize, 100);
        d.readBool(110, &m_alpacaFilterWheelEnabled, false);
        d.readString(111, &m_alpacaFilterWheelHost, "127.0.0.1");
        d.readU32(112, &utmp, 11111);
        m_alpacaFilterWheelPort = (utmp <= 65535) ? static_cast<uint16_t>(utmp) : 11111;
        d.readS32(113, &m_alpacaFilterWheelDeviceNumber, 0);
        d.readS32(114, &m_alpacaFilterWheelPosition, 0);
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
        d.readBool(130, &m_stackEnabled, false);
        d.readS32(131, &m_stackFrameCount, 4);
        d.readS32(132, (qint32 *) &m_stackMethod, (qint32) StackMethodAverage);
        d.readS32(133, (qint32 *) &m_stackAlignmentMethod, (qint32) StackAlignmentNone);
        d.readString(134, &m_stackDarkFileName, "");
        d.readString(135, &m_stackFlatFileName, "");
        d.readString(136, &m_stackBiasFileName, "");
        d.readFloat(137, &m_latitude, MainCore::instance()->getSettings().getLatitude());
        d.readFloat(138, &m_longitude, MainCore::instance()->getSettings().getLongitude());
        d.readFloat(139, &m_altitude, MainCore::instance()->getSettings().getAltitude());
        d.readBool(140, &m_positionSync, false);
        d.readString(164, &m_owmAPIKey, "");
        d.readFloat(141, &m_azimuth, 0.0f);
        d.readFloat(142, &m_elevation, 0.0f);
        d.readFloat(150, &m_roll, 0.0f);
        d.readString(143, &m_rotator, "");
        d.readFloat(144, &m_fov, 60.0f);
        d.readS32(149, (int *) &m_lensProjection, LensProjectionRectilinear);
        d.readBool(151, &m_scheduleEnabled, false);
        d.readString(152, &m_scheduleStartTime, "20:00:00");
        d.readString(153, &m_scheduleEndTime, "06:00:00");
        d.readS32(154, &m_scheduleWeekdays, 0x7f);

        if (m_rollupState)
        {
            d.readBlob(19, &bytetmp);
            m_rollupState->deserialize(bytetmp);
        }

        d.readS32(20, &m_workspaceIndex, 0);
        d.readBlob(21, &m_geometryBytes);
        d.readS32(22, &m_cameraBinX, 1);
        d.readS32(23, &m_cameraBinY, 1);
        d.readS32(24, &m_cameraGain, 100);
        d.readS32(25, &m_cameraReadoutMode, 0);
        d.readS32(26, &m_cameraOffset, 1);
        d.readS32(27, &m_cameraNumX, 0);
        d.readS32(28, &m_cameraNumY, 0);
        d.readS32(29, &m_cameraStartX, 0);
        d.readS32(30, &m_cameraStartY, 0);
        m_cameraBinX = std::max(m_minPositive, m_cameraBinX);
        m_cameraBinY = std::max(m_minPositive, m_cameraBinY);
        m_cameraNumX = std::max(m_minNonNegative, m_cameraNumX);
        m_cameraNumY = std::max(m_minNonNegative, m_cameraNumY);
        m_cameraStartX = std::max(m_minNonNegative, m_cameraStartX);
        m_cameraStartY = std::max(m_minNonNegative, m_cameraStartY);
        m_cameraReadoutMode = std::max(m_minNonNegative, m_cameraReadoutMode);
        d.readS32(31, &m_postProcessWhiteBalanceMode, 0);
        d.readDouble(32, &m_postProcessWhiteBalanceRedGain, 1.0);
        d.readDouble(33, &m_postProcessWhiteBalanceGreenGain, 1.0);
        d.readDouble(34, &m_postProcessWhiteBalanceBlueGain, 1.0);
        d.readBool(157, &m_postProcessUnwarp, false);
        d.readS32(158, reinterpret_cast<qint32*>(&m_histogramStretch), static_cast<qint32>(HistogramStretchOff));
        d.readDouble(159, &m_histogramStretchBlackPoint, 0.0);
        d.readDouble(160, &m_histogramStretchWhitePoint, 1.0);
        d.readDouble(161, &m_histogramStretchGamma, 1.0);
        d.readDouble(162, &m_histogramStretchAsinhStrength, 10.0);
        d.readDouble(163, &m_histogramStretchLogStrength, 10.0);
        d.readBool(127, &m_postProcessGreyscale, false);
        d.readDouble(35, &m_saturation, 1.0);
        d.readDouble(36, &m_gamma, 1.0);
        d.readS32(37, &m_gaussianBlur, 0);
        d.readS32(38, &m_medianBlur, 0);
        d.readDouble(39, &m_sharpen, 0.0);
        qint32 edgeDisplayMode = static_cast<qint32>(EdgeDisplayOverlay);
        d.readS32(191, &edgeDisplayMode, static_cast<qint32>(EdgeDisplayOverlay));
        m_edgeDisplayMode = static_cast<EdgeDisplayMode>(qBound(
            static_cast<qint32>(EdgeDisplayOverlay),
            edgeDisplayMode,
            static_cast<qint32>(EdgeDisplayEdgesOnly)));
        d.readDouble(40, &m_sobelEdge, 0.0);
        d.readDouble(190, &m_cannyEdge, 0.0);
        d.readDouble(192, &m_lineEnhancement, 0.0);
        d.readDouble(193, &m_ridgeDetection, 0.0);
        d.readS32(195, &m_ridgeDetectionKernelSize, 3);
        d.readDouble(196, &m_ridgeDetectionScale, 1.0);
        d.readDouble(197, &m_ridgeDetectionDelta, 0.0);
        d.readBool(41, &m_flipX, false);
        d.readBool(42, &m_flipY, false);
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
        m_lineEnhancement = qBound(m_minFilterAmount, m_lineEnhancement, m_maxFilterAmount);
        m_ridgeDetection = qBound(m_minFilterAmount, m_ridgeDetection, m_maxFilterAmount);
        m_ridgeDetectionKernelSize = (m_ridgeDetectionKernelSize <= 1) ? 1 :
            (m_ridgeDetectionKernelSize <= 3) ? 3 :
            (m_ridgeDetectionKernelSize <= 5) ? 5 : 7;
        m_ridgeDetectionScale = qBound(m_minRidgeScale, m_ridgeDetectionScale, m_maxRidgeScale);
        m_ridgeDetectionDelta = qBound(m_minRidgeDelta, m_ridgeDetectionDelta, m_maxRidgeDelta);

        d.readDouble(43, &m_brightness, 0.0);
        d.readDouble(44, &m_contrast, 1.0);
        d.readBool(45, &m_invertColors, false);
        d.readBool(46, &m_overlayDateTime, false);
        uint32_t colorRgba = QColor(Qt::white).rgba();
        d.readU32(47, &colorRgba, QColor(Qt::white).rgba());
        m_dateTimeColor = QColor::fromRgba(colorRgba);
        d.readBool(48, &m_diffMask, false);
        d.readS32(49, &m_diffThreshold, 30);
        d.readS32(50, &m_dilationSize, 3);
        d.readS32(51, &m_diffMaskHistoryFrames, 1);
        d.readS32(52, &m_diffMaskCloseSize, 0);
        d.readS32(126, &m_diffMaskOpenSize, 0);
        m_brightness = qBound(m_minBrightness, m_brightness, m_maxBrightness);
        m_contrast = qBound(m_minWhiteBalanceGain, m_contrast, m_maxFilterAmount);
        m_diffThreshold = qBound(m_minThreshold8Bit, m_diffThreshold, m_maxThreshold8Bit);
        m_diffMaskOpenSize = qBound(m_minMorphologyKernel, m_diffMaskOpenSize, m_maxMorphologyKernel);
        m_dilationSize = qBound(m_minMorphologyKernel, m_dilationSize, m_maxMorphologyKernel);
        m_diffMaskHistoryFrames = qBound(m_minShortHistoryFrames, m_diffMaskHistoryFrames, m_maxShortHistoryFrames);
        m_diffMaskCloseSize = qBound(m_minMorphologyKernel, m_diffMaskCloseSize, m_maxMorphologyKernel);

        d.readString(53, &m_overlayFontFamily, "");
        d.readDouble(54, &m_overlayFontScale, 12.0);
        d.readS32(55, &m_detectionRoiX, 0);
        d.readS32(56, &m_detectionRoiY, 0);
        d.readS32(57, &m_detectionRoiWidth, 0);
        d.readS32(58, &m_detectionRoiHeight, 0);
        d.readBool(201, &m_showDetectionRoi, true);
        d.readBool(59, &m_motionDetect, false);
        qint32 motionBackgroundSubtractor = static_cast<qint32>(MotionBackgroundSubtractorMOG2);
        d.readS32(175, &motionBackgroundSubtractor, static_cast<qint32>(MotionBackgroundSubtractorMOG2));
        m_motionBackgroundSubtractor = static_cast<MotionBackgroundSubtractor>(qBound(
            static_cast<qint32>(MotionBackgroundSubtractorMOG2),
            motionBackgroundSubtractor,
            static_cast<qint32>(MotionBackgroundSubtractorKNN)));
        qint32 motionMaskView = static_cast<qint32>(MotionMaskViewOff);
        d.readS32(176, &motionMaskView, static_cast<qint32>(MotionMaskViewOff));
        m_motionMaskView = static_cast<MotionMaskView>(qBound(
            static_cast<qint32>(MotionMaskViewOff),
            motionMaskView,
            static_cast<qint32>(MotionMaskViewFinal)));
        d.readS32(60, &m_motionHistory, 500);
        d.readDouble(61, &m_motionVarThreshold, 16.0);
        d.readDouble(171, &m_motionLearningRate, -1.0);
        d.readS32(172, &m_motionConfirmFrames, 1);
        d.readDouble(173, &m_motionDownscale, 1.0);
        d.readBool(62, &m_motionDetectShadows, true);
        d.readS32(63, &m_motionOpenSize, 0);
        d.readS32(64, &m_motionCloseSize, 0);
        d.readS32(65, &m_motionPersistenceFrames, 0);
        uint32_t motionBoxColorRgba = QColor(Qt::red).rgba();
        d.readU32(66, &motionBoxColorRgba, QColor(Qt::red).rgba());
        m_motionBoxColor = QColor::fromRgba(motionBoxColorRgba);
        d.readS32(67, &m_minContourArea, 100);
        d.readBlob(174, &bytetmp);
        deserializeMotionExclusionRects(bytetmp, m_motionExclusionRects);
        d.readBool(186, &m_showMotionExclusionRects, true);
        d.readBool(177, &m_streakDetect, false);
        d.readS32(178, &m_streakThreshold, 24);
        d.readS32(179, &m_streakMinLength, 80);
        d.readS32(180, &m_streakHoughThreshold, 30);
        d.readDouble(181, &m_streakMaxGap, 12.0);
        d.readS32(182, &m_streakPersistenceFrames, 1);
        d.readDouble(183, &m_streakDownscale, 0.5);
        uint32_t streakColorRgba = QColor(255, 255, 80).rgba();
        d.readU32(184, &streakColorRgba, QColor(255, 255, 80).rgba());
        m_streakColor = QColor::fromRgba(streakColorRgba);
        qint32 streakDebugView = static_cast<qint32>(StreakDebugViewOff);
        d.readS32(185, &streakDebugView, static_cast<qint32>(StreakDebugViewOff));
        m_streakDebugView = static_cast<StreakDebugView>(qBound(
            static_cast<qint32>(StreakDebugViewOff),
            streakDebugView,
            static_cast<qint32>(StreakDebugViewFinal)));
        qint32 streakOverlayStyle = static_cast<qint32>(StreakOverlayStyleLines);
        d.readS32(187, &streakOverlayStyle, static_cast<qint32>(StreakOverlayStyleLines));
        m_streakOverlayStyle = static_cast<StreakOverlayStyle>(qBound(
            static_cast<qint32>(StreakOverlayStyleLines),
            streakOverlayStyle,
            static_cast<qint32>(StreakOverlayStyleBoundingBoxes)));
        qint32 streakLineEnhancementPlacement = static_cast<qint32>(StreakLineEnhancementOff);
        d.readS32(194, &streakLineEnhancementPlacement, static_cast<qint32>(StreakLineEnhancementOff));
        m_streakLineEnhancementPlacement = static_cast<StreakLineEnhancementPlacement>(qBound(
            static_cast<qint32>(StreakLineEnhancementOff),
            streakLineEnhancementPlacement,
            static_cast<qint32>(StreakLineEnhancementAfterBackground)));
        d.readBool(202, &m_starDetect, false);
        d.readS32(203, &m_starThreshold, 24);
        d.readS32(204, &m_starBackgroundBlur, 12);
        d.readS32(205, &m_starMinArea, 1);
        d.readS32(206, &m_starMaxArea, 36);
        d.readDouble(207, &m_starMaxAspectRatio, 2.5);
        qint32 starDebugView = static_cast<qint32>(StarDebugViewOff);
        d.readS32(208, &starDebugView, static_cast<qint32>(StarDebugViewOff));
        m_starDebugView = static_cast<StarDebugView>(qBound(
            static_cast<qint32>(StarDebugViewOff),
            starDebugView,
            static_cast<qint32>(StarDebugViewFinal)));
        uint32_t starColorRgba = QColor(120, 255, 255).rgba();
        d.readU32(209, &starColorRgba, QColor(120, 255, 255).rgba());
        m_starColor = QColor::fromRgba(starColorRgba);
        d.readBool(210, &m_plateSolve, false);
        d.readDouble(211, &m_plateSolveMaxMagnitude, 3.5);
        d.readS32(212, &m_plateSolveMinMatches, 4);
        d.readDouble(213, &m_plateSolveMatchRadius, 24.0);
        d.readDouble(214, &m_plateSolveSearchRadius, 12.0);
        d.readBool(215, &m_plateSolveUseDownloadedCatalog, false);
        d.readBool(216, &m_plateSolveUseCurrentDateTime, true);
        qint64 plateSolveDateTimeMs = QDateTime::currentDateTime().toMSecsSinceEpoch();
        d.readS64(217, &plateSolveDateTimeMs, plateSolveDateTimeMs);
        m_plateSolveDateTime = QDateTime::fromMSecsSinceEpoch(std::max(plateSolveDateTimeMs, m_minPlateSolveDateTimeMs));
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
        m_plateSolveSearchRadius = qBound(m_minPlateSolveSearchRadius, m_plateSolveSearchRadius, m_maxPlateSolveSearchRadius);
        if (!m_plateSolveDateTime.isValid()) {
            m_plateSolveDateTime = QDateTime::currentDateTime();
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
        bool legacyVideoPostProcess = false;
        d.readBool(68, &legacyVideoPostProcess, false);
        qint32 videoPostProcessMode = legacyVideoPostProcess ? static_cast<qint32>(SavedMediaProcessed)
                                                             : static_cast<qint32>(SavedMediaRaw);
        d.readS32(128, &videoPostProcessMode, videoPostProcessMode);
        m_recordMode = qBound(SavedMediaRaw, static_cast<SavedMediaMode>(videoPostProcessMode), SavedMediaBoth);
        d.readBool(69, &m_overlaySpectrum, false);
        d.readString(70, &m_spectrumDevice, "");
        d.readS32(71, &m_spectrumOffsetX, 0);
        d.readS32(72, &m_spectrumOffsetY, 0);
        m_spectrumOffsetX = qBound(m_minSignedUiPixelOffset, m_spectrumOffsetX, m_maxSignedUiPixelOffset);
        m_spectrumOffsetY = qBound(m_minSignedUiPixelOffset, m_spectrumOffsetY, m_maxSignedUiPixelOffset);
        d.readDouble(73, &m_spectrumScale, 1.0);
        m_spectrumScale = qBound(m_minSpectrumScale, m_spectrumScale, m_maxSpectrumScale);
        d.readString(74, &m_dateTimeFormat, "yyyy-MM-dd hh:mm:ss");
        d.readS32(75, &m_dateTimePosX, 4);
        d.readS32(76, &m_dateTimePosY, 0);
        m_dateTimePosX = qBound(m_minUiPixelOffset, m_dateTimePosX, m_maxUiPixelOffset);
        m_dateTimePosY = qBound(m_minUiPixelOffset, m_dateTimePosY, m_maxUiPixelOffset);
        d.readBool(145, &m_equatorialGrid, false);
        uint32_t equatorialGridColorRgba = QColor(80, 170, 255).rgba();
        d.readU32(146, &equatorialGridColorRgba, QColor(80, 170, 255).rgba());
        m_equatorialGridColor = QColor::fromRgba(equatorialGridColorRgba);
        d.readBool(147, &m_altAzGrid, false);
        uint32_t altAzGridColorRgba = QColor(255, 170, 80).rgba();
        d.readU32(148, &altAzGridColorRgba, QColor(255, 170, 80).rgba());
        m_altAzGridColor = QColor::fromRgba(altAzGridColorRgba);
        d.readBool(198, &m_constellation, false);
        uint32_t constellationColorRgba = QColor(255, 255, 120).rgba();
        d.readU32(199, &constellationColorRgba, QColor(255, 255, 120).rgba());
        m_constellationColor = QColor::fromRgba(constellationColorRgba);
        d.readS32(200, reinterpret_cast<qint32*>(&m_constellationOverlay), static_cast<qint32>(ConstellationOverlayUrsaMajor));
        d.readBool(165, &m_trackObjects, false);
        d.readDouble(166, &m_trackObjectMinElevation, 0.0);
        m_trackObjectMinElevation = qBound(m_minNormalized, m_trackObjectMinElevation, static_cast<double>(m_maxElevation));
        uint32_t trackObjectColorRgba = QColor(80, 255, 80).rgba();
        d.readU32(169, &trackObjectColorRgba, QColor(80, 255, 80).rgba());
        m_trackObjectColor = QColor::fromRgba(trackObjectColorRgba);
        d.readDouble(170, &m_trackObjectFontScale, 9.0);
        m_trackObjectFontScale = qBound(m_minOverlayFontScale, m_trackObjectFontScale, m_maxOverlayFontScale);
        d.readString(155, &m_gridLabelFontFamily, "");
        d.readDouble(156, &m_gridLabelFontScale, 9.0);
        m_gridLabelFontScale = qBound(m_minOverlayFontScale, m_gridLabelFontScale, m_maxOverlayFontScale);
        d.readBool(77, &m_overlayText, false);
        d.readString(78, &m_overlayTextString, DEFAULT_OVERLAY_TEXT_STRING);
        uint32_t overlayTextColorRgba = QColor(Qt::white).rgba();
        d.readU32(79, &overlayTextColorRgba, QColor(Qt::white).rgba());
        m_overlayTextColor = QColor::fromRgba(overlayTextColorRgba);
        d.readString(80, &m_overlayTextFontFamily, "");
        d.readDouble(81, &m_overlayTextFontScale, 12.0);
        m_overlayTextFontScale = qBound(m_minOverlayFontScale, m_overlayTextFontScale, m_maxOverlayFontScale);
        d.readS32(82, &m_overlayTextPosX, 4);
        d.readS32(83, &m_overlayTextPosY, 0);
        m_overlayTextPosX = qBound(m_minUiPixelOffset, m_overlayTextPosX, m_maxUiPixelOffset);
        m_overlayTextPosY = qBound(m_minUiPixelOffset, m_overlayTextPosY, m_maxUiPixelOffset);
        d.readBool(84, &m_yoloEnabled, false);
        d.readString(85, &m_yoloModelPath, "");
        d.readString(86, &m_yoloLabelsPath, "");
        d.readDouble(87, &m_yoloConfThreshold, 0.5);
        d.readDouble(88, &m_yoloNmsThreshold, 0.45);
        m_yoloConfThreshold = qBound(m_minNormalized, m_yoloConfThreshold, m_maxNormalized);
        m_yoloNmsThreshold = qBound(m_minNormalized, m_yoloNmsThreshold, m_maxNormalized);
        uint32_t yoloBoxColorRgba = QColor(Qt::green).rgba();
        d.readU32(89, &yoloBoxColorRgba, QColor(Qt::green).rgba());
        m_yoloBoxColor = QColor::fromRgba(yoloBoxColorRgba);
        d.readDouble(90, &m_yoloDisappearDebounce, 0.0);
        m_yoloDisappearDebounce = qBound(m_minYoloDisappearDebounce, m_yoloDisappearDebounce, m_maxYoloDisappearDebounce);
        d.readS32(91, (qint32 *) &m_yoloDnnTarget, (qint32) CPU);
        d.readBlob(92, &bytetmp);
        deserializeObjectDeviceSettings(bytetmp, m_objectDeviceSettings);

        d.readBool(93, &m_audioMute, true);
        d.readString(94, &m_audioDeviceName, "");
        d.readS32(95, &m_whiteBalanceMode, 0);
        m_whiteBalanceMode = std::max(m_minNonNegative, m_whiteBalanceMode);
        d.readDouble(96, &m_exposureCompensation, 0.0);
        m_exposureCompensation = qBound(m_minExposureCompensation, m_exposureCompensation, m_maxExposureCompensation);
        d.readS32(97, &m_focusMode, 0);
        m_focusMode = std::max(m_minNonNegative, m_focusMode);
        d.readDouble(98, &m_focusDistance, 1.0);
        m_focusDistance = qBound(m_minNormalized, m_focusDistance, m_maxNormalized);
        d.readDouble(99, &m_zoomFactor, 1.0);
        m_zoomFactor = std::max(m_minZoomFactor, m_zoomFactor);

        d.readString(115, &m_videoFileCameraPath, "");
        d.readBool(188, &m_videoLoop, false);
        d.readDouble(189, &m_videoPlaybackRate, 1.0);
        m_videoPlaybackRate = qBound(m_minVideoPlaybackRate, m_videoPlaybackRate, m_maxVideoPlaybackRate);

        d.readBool(116, &m_useReverseAPI);
        d.readString(117, &m_reverseAPIAddress);
        d.readU32(118, &utmp, 0);

        if ((utmp > 1023) && (utmp < 65535)) {
            m_reverseAPIPort = utmp;
        } else {
            m_reverseAPIPort = 8888;
        }
        d.readU32(119, &utmp, 0);
        m_reverseAPIFeatureSetIndex = utmp > 99 ? 99 : utmp;
        d.readU32(120, &utmp, 0);
        m_reverseAPIFeatureIndex = utmp > 99 ? 99 : utmp;
        d.readS32(121, &m_asiCoolerOn, -1);
        d.readS32(122, &m_asiTargetTemp, std::numeric_limits<int>::min());
        d.readS32(123, &m_asiUsbBandwidth, -1);
        d.readS32(124, &m_asiHighSpeedMode, -1);
        d.readS32(125, (qint32 *) &m_asiColorImageType, (qint32) AsiColorImageTypeRgb24);
        d.readBool(168, &m_asiAutoExposureGain, false);
        m_asiCoolerOn = qBound(m_minAsiControl, m_asiCoolerOn, m_maxAsiControl);
        m_asiUsbBandwidth = std::max(m_minAsiControl, m_asiUsbBandwidth);
        m_asiHighSpeedMode = qBound(m_minAsiControl, m_asiHighSpeedMode, m_maxAsiControl);
        m_asiColorImageType = qBound(AsiColorImageTypeRgb24, m_asiColorImageType, AsiColorImageTypeRaw16);
        m_stackFrameCount = qBound(m_minStackFrameCount, m_stackFrameCount, m_maxStackFrameCount);
        m_stackMethod = qBound(StackMethodAverage, m_stackMethod, StackMethodSigmaClippedAverage);
        m_stackAlignmentMethod = qBound(StackAlignmentNone, m_stackAlignmentMethod, StackAlignmentStarCentroidMatching);
        m_latitude = qBound(m_minLatitude, m_latitude, m_maxLatitude);
        m_longitude = qBound(m_minLongitude, m_longitude, m_maxLongitude);
        m_altitude = qBound(m_minAltitude, m_altitude, m_maxAltitude);
        m_azimuth = std::fmod(m_azimuth, m_fullRotationDegrees);
        if (m_azimuth < 0.0f) {
            m_azimuth += m_fullRotationDegrees;
        }
        m_elevation = qBound(m_minElevation, m_elevation, m_maxElevation);
        m_roll = std::fmod(m_roll, m_fullRotationDegrees);
        if (m_roll < 0.0f) {
            m_roll += m_fullRotationDegrees;
        }
        m_fov = qBound(m_minFov, m_fov, m_maxFov);
        m_lensProjection = (LensProjection) qBound((int) LensProjectionRectilinear, (int) m_lensProjection, (int) LensProjectionEquisolid);

        return true;
    }

    resetToDefaults();
    return false;
}

QByteArray CameraSettings::serializeObjectDeviceSettings(QHash<QString, QList<ObjectDeviceSettings *> *> objectDeviceSettings) const
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);
    stream << objectDeviceSettings;
    return data;
}

void CameraSettings::deserializeObjectDeviceSettings(const QByteArray& data, QHash<QString, QList<ObjectDeviceSettings *> *>& objectDeviceSettings)
{
    if (data.isEmpty()) {
        objectDeviceSettings.clear();
        return;
    }

    QDataStream stream(data);
    stream >> objectDeviceSettings;
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
    if (settingsKeys.contains("cameraBinX") || settingsKeys.contains("alpacaBinX")) {
        m_cameraBinX = std::max(m_minPositive, settings.m_cameraBinX);
    }
    if (settingsKeys.contains("cameraBinY") || settingsKeys.contains("alpacaBinY")) {
        m_cameraBinY = std::max(m_minPositive, settings.m_cameraBinY);
    }
    if (settingsKeys.contains("cameraNumX") || settingsKeys.contains("alpacaNumX")) {
        m_cameraNumX = std::max(m_minNonNegative, settings.m_cameraNumX);
    }
    if (settingsKeys.contains("cameraNumY") || settingsKeys.contains("alpacaNumY")) {
        m_cameraNumY = std::max(m_minNonNegative, settings.m_cameraNumY);
    }
    if (settingsKeys.contains("cameraStartX") || settingsKeys.contains("alpacaStartX")) {
        m_cameraStartX = std::max(m_minNonNegative, settings.m_cameraStartX);
    }
    if (settingsKeys.contains("cameraStartY") || settingsKeys.contains("alpacaStartY")) {
        m_cameraStartY = std::max(m_minNonNegative, settings.m_cameraStartY);
    }
    if (settingsKeys.contains("cameraGain") || settingsKeys.contains("alpacaGain")) {
        m_cameraGain = settings.m_cameraGain;
    }
    if (settingsKeys.contains("cameraOffset") || settingsKeys.contains("alpacaOffset")) {
        m_cameraOffset = settings.m_cameraOffset;
    }
    if (settingsKeys.contains("cameraReadoutMode") || settingsKeys.contains("alpacaReadoutMode")) {
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
    if (settingsKeys.contains("asiColorImageType")) {
        m_asiColorImageType = qBound(AsiColorImageTypeRgb24, settings.m_asiColorImageType, AsiColorImageTypeRaw16);
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
    if (settingsKeys.contains("videoLoop")) {
        m_videoLoop = settings.m_videoLoop;
    }
    if (settingsKeys.contains("videoPlaybackRate")) {
        m_videoPlaybackRate = qBound(m_minVideoPlaybackRate, settings.m_videoPlaybackRate, m_maxVideoPlaybackRate);
    }
    if (settingsKeys.contains("videoHwAcceleration")) {
        m_videoHwAcceleration = settings.m_videoHwAcceleration;
    }
    if (settingsKeys.contains("stackEnabled")) {
        m_stackEnabled = settings.m_stackEnabled;
    }
    if (settingsKeys.contains("stackFrameCount")) {
        m_stackFrameCount = qBound(m_minStackFrameCount, settings.m_stackFrameCount, m_maxStackFrameCount);
    }
    if (settingsKeys.contains("stackMethod")) {
        m_stackMethod = qBound(StackMethodAverage, settings.m_stackMethod, StackMethodSigmaClippedAverage);
    }
    if (settingsKeys.contains("stackAlignmentMethod")) {
        m_stackAlignmentMethod = qBound(StackAlignmentNone, settings.m_stackAlignmentMethod, StackAlignmentStarCentroidMatching);
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
        m_azimuth = std::fmod(settings.m_azimuth, m_fullRotationDegrees);
        if (m_azimuth < 0.0f) {
            m_azimuth += m_fullRotationDegrees;
        }
    }
    if (settingsKeys.contains("elevation")) {
        m_elevation = qBound(m_minElevation, settings.m_elevation, m_maxElevation);
    }
    if (settingsKeys.contains("roll")) {
        m_roll = std::fmod(settings.m_roll, m_fullRotationDegrees);
        if (m_roll < 0.0f) {
            m_roll += m_fullRotationDegrees;
        }
    }
    if (settingsKeys.contains("rotator")) {
        m_rotator = settings.m_rotator;
    }
    if (settingsKeys.contains("fov")) {
        m_fov = qBound(m_minFov, settings.m_fov, m_maxFov);
    }
    if (settingsKeys.contains("lensProjection")) {
        m_lensProjection = (LensProjection) qBound((int) LensProjectionRectilinear, (int) settings.m_lensProjection, (int) LensProjectionEquisolid);
    }
    if (settingsKeys.contains("scheduleEnabled")) {
        m_scheduleEnabled = settings.m_scheduleEnabled;
    }
    if (settingsKeys.contains("scheduleStartTime")) {
        m_scheduleStartTime = settings.m_scheduleStartTime;
    }
    if (settingsKeys.contains("scheduleEndTime")) {
        m_scheduleEndTime = settings.m_scheduleEndTime;
    }
    if (settingsKeys.contains("scheduleWeekdays")) {
        m_scheduleWeekdays = settings.m_scheduleWeekdays & 0x7f;
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
    if (settingsKeys.contains("lineEnhancement")) {
        m_lineEnhancement = qBound(m_minFilterAmount, settings.m_lineEnhancement, m_maxFilterAmount);
    }
    if (settingsKeys.contains("ridgeDetection")) {
        m_ridgeDetection = qBound(m_minFilterAmount, settings.m_ridgeDetection, m_maxFilterAmount);
    }
    if (settingsKeys.contains("ridgeDetectionKernelSize")) {
        m_ridgeDetectionKernelSize = (settings.m_ridgeDetectionKernelSize <= 1) ? 1 :
            (settings.m_ridgeDetectionKernelSize <= 3) ? 3 :
            (settings.m_ridgeDetectionKernelSize <= 5) ? 5 : 7;
    }
    if (settingsKeys.contains("ridgeDetectionScale")) {
        m_ridgeDetectionScale = qBound(m_minRidgeScale, settings.m_ridgeDetectionScale, m_maxRidgeScale);
    }
    if (settingsKeys.contains("ridgeDetectionDelta")) {
        m_ridgeDetectionDelta = qBound(m_minRidgeDelta, settings.m_ridgeDetectionDelta, m_maxRidgeDelta);
    }
    if (settingsKeys.contains("flipX")) {
        m_flipX = settings.m_flipX;
    }
    if (settingsKeys.contains("flipY")) {
        m_flipY = settings.m_flipY;
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
    if (settingsKeys.contains("showMotionExclusionRects")) {
        m_showMotionExclusionRects = settings.m_showMotionExclusionRects;
    }
    if (settingsKeys.contains("streakDetect")) {
        m_streakDetect = settings.m_streakDetect;
    }
    if (settingsKeys.contains("streakThreshold")) {
        m_streakThreshold = qBound(m_minThreshold8Bit, settings.m_streakThreshold, m_maxThreshold8Bit);
    }
    if (settingsKeys.contains("streakMinLength")) {
        m_streakMinLength = qBound(m_minStreakLength, settings.m_streakMinLength, m_maxStreakLength);
    }
    if (settingsKeys.contains("streakHoughThreshold")) {
        m_streakHoughThreshold = qBound(m_minStreakHoughThreshold, settings.m_streakHoughThreshold, m_maxStreakHoughThreshold);
    }
    if (settingsKeys.contains("streakMaxGap")) {
        m_streakMaxGap = qBound(m_minStreakMaxGap, settings.m_streakMaxGap, m_maxStreakMaxGap);
    }
    if (settingsKeys.contains("streakPersistenceFrames")) {
        m_streakPersistenceFrames = qBound(m_minNonNegative, settings.m_streakPersistenceFrames, m_maxShortHistoryFrames);
    }
    if (settingsKeys.contains("streakDownscale")) {
        const QList<double> validDownscales{1.0, 0.5, 0.25};
        m_streakDownscale = validDownscales.contains(settings.m_streakDownscale) ? settings.m_streakDownscale : 1.0;
    }
    if (settingsKeys.contains("streakDebugView")) {
        m_streakDebugView = static_cast<StreakDebugView>(qBound(
            static_cast<qint32>(StreakDebugViewOff),
            static_cast<qint32>(settings.m_streakDebugView),
            static_cast<qint32>(StreakDebugViewFinal)));
    }
    if (settingsKeys.contains("streakOverlayStyle")) {
        m_streakOverlayStyle = static_cast<StreakOverlayStyle>(qBound(
            static_cast<qint32>(StreakOverlayStyleLines),
            static_cast<qint32>(settings.m_streakOverlayStyle),
            static_cast<qint32>(StreakOverlayStyleBoundingBoxes)));
    }
    if (settingsKeys.contains("streakLineEnhancementPlacement")) {
        m_streakLineEnhancementPlacement = static_cast<StreakLineEnhancementPlacement>(qBound(
            static_cast<qint32>(StreakLineEnhancementOff),
            static_cast<qint32>(settings.m_streakLineEnhancementPlacement),
            static_cast<qint32>(StreakLineEnhancementAfterBackground)));
    }
    if (settingsKeys.contains("streakColor")) {
        m_streakColor = settings.m_streakColor;
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
    if (settingsKeys.contains("plateSolveSearchRadius")) {
        m_plateSolveSearchRadius = settings.m_plateSolveSearchRadius;
    }
    if (settingsKeys.contains("plateSolveUseCurrentDateTime")) {
        m_plateSolveUseCurrentDateTime = settings.m_plateSolveUseCurrentDateTime;
    }
    if (settingsKeys.contains("plateSolveDateTime")) {
        m_plateSolveDateTime = settings.m_plateSolveDateTime;
    }
    if (settingsKeys.contains("plateSolveUseDownloadedCatalog")) {
        m_plateSolveUseDownloadedCatalog = settings.m_plateSolveUseDownloadedCatalog;
    }
    if (settingsKeys.contains("videoPostProcess")) {
        m_recordMode = qBound(SavedMediaRaw, settings.m_recordMode, SavedMediaBoth);
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
    if (settingsKeys.contains("trackObjectMinElevation")) {
        m_trackObjectMinElevation = qBound(m_minNormalized, settings.m_trackObjectMinElevation, static_cast<double>(m_maxElevation));
    }
    if (settingsKeys.contains("trackObjectColor")) {
        m_trackObjectColor = settings.m_trackObjectColor;
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
    if (settingsKeys.contains("yoloDisappearDebounce")) {
        m_yoloDisappearDebounce = qBound(m_minYoloDisappearDebounce, settings.m_yoloDisappearDebounce, m_maxYoloDisappearDebounce);
    }
    if (settingsKeys.contains("yoloDnnTarget")) {
        m_yoloDnnTarget = settings.m_yoloDnnTarget;
    }
    if (settingsKeys.contains("objectDeviceSettings")) {
        m_objectDeviceSettings = settings.m_objectDeviceSettings;
    }
    if (settingsKeys.contains("audioMute")) {
        m_audioMute = settings.m_audioMute;
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
    if (settingsKeys.contains("cameraBinX") || settingsKeys.contains("alpacaBinX") || force) {
        ostr << " m_cameraBinX: " << m_cameraBinX;
    }
    if (settingsKeys.contains("cameraBinY") || settingsKeys.contains("alpacaBinY") || force) {
        ostr << " m_cameraBinY: " << m_cameraBinY;
    }
    if (settingsKeys.contains("cameraNumX") || settingsKeys.contains("alpacaNumX") || force) {
        ostr << " m_cameraNumX: " << m_cameraNumX;
    }
    if (settingsKeys.contains("cameraNumY") || settingsKeys.contains("alpacaNumY") || force) {
        ostr << " m_cameraNumY: " << m_cameraNumY;
    }
    if (settingsKeys.contains("cameraStartX") || settingsKeys.contains("alpacaStartX") || force) {
        ostr << " m_cameraStartX: " << m_cameraStartX;
    }
    if (settingsKeys.contains("cameraStartY") || settingsKeys.contains("alpacaStartY") || force) {
        ostr << " m_cameraStartY: " << m_cameraStartY;
    }
    if (settingsKeys.contains("cameraGain") || settingsKeys.contains("alpacaGain") || force) {
        ostr << " m_cameraGain: " << m_cameraGain;
    }
    if (settingsKeys.contains("cameraOffset") || settingsKeys.contains("alpacaOffset") || force) {
        ostr << " m_cameraOffset: " << m_cameraOffset;
    }
    if (settingsKeys.contains("cameraReadoutMode") || settingsKeys.contains("alpacaReadoutMode") || force) {
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
    if (settingsKeys.contains("videoLoop") || force) {
        ostr << " m_videoLoop: " << m_videoLoop;
    }
    if (settingsKeys.contains("videoPlaybackRate") || force) {
        ostr << " m_videoPlaybackRate: " << m_videoPlaybackRate;
    }
    if (settingsKeys.contains("videoHwAcceleration") || force) {
        ostr << " m_videoHwAcceleration: " << m_videoHwAcceleration;
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
    if (settingsKeys.contains("stackAlignmentMethod") || force) {
        ostr << " m_stackAlignmentMethod: " << m_stackAlignmentMethod;
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
    if (settingsKeys.contains("fov") || force) {
        ostr << " m_fov: " << m_fov;
    }
    if (settingsKeys.contains("lensProjection") || force) {
        ostr << " m_lensProjection: " << m_lensProjection;
    }
    if (settingsKeys.contains("scheduleEnabled") || force) {
        ostr << " m_scheduleEnabled: " << m_scheduleEnabled;
    }
    if (settingsKeys.contains("scheduleStartTime") || force) {
        ostr << " m_scheduleStartTime: " << m_scheduleStartTime.toStdString();
    }
    if (settingsKeys.contains("scheduleEndTime") || force) {
        ostr << " m_scheduleEndTime: " << m_scheduleEndTime.toStdString();
    }
    if (settingsKeys.contains("scheduleWeekdays") || force) {
        ostr << " m_scheduleWeekdays: " << m_scheduleWeekdays;
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
    if (settingsKeys.contains("lineEnhancement") || force) {
        ostr << " m_lineEnhancement: " << m_lineEnhancement;
    }
    if (settingsKeys.contains("ridgeDetection") || force) {
        ostr << " m_ridgeDetection: " << m_ridgeDetection;
    }
    if (settingsKeys.contains("ridgeDetectionKernelSize") || force) {
        ostr << " m_ridgeDetectionKernelSize: " << m_ridgeDetectionKernelSize;
    }
    if (settingsKeys.contains("ridgeDetectionScale") || force) {
        ostr << " m_ridgeDetectionScale: " << m_ridgeDetectionScale;
    }
    if (settingsKeys.contains("ridgeDetectionDelta") || force) {
        ostr << " m_ridgeDetectionDelta: " << m_ridgeDetectionDelta;
    }
    if (settingsKeys.contains("flipX") || force) {
        ostr << " m_flipX: " << m_flipX;
    }
    if (settingsKeys.contains("flipY") || force) {
        ostr << " m_flipY: " << m_flipY;
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
    if (settingsKeys.contains("showMotionExclusionRects") || force) {
        ostr << " m_showMotionExclusionRects: " << m_showMotionExclusionRects;
    }
    if (settingsKeys.contains("streakDetect") || force) {
        ostr << " m_streakDetect: " << m_streakDetect;
    }
    if (settingsKeys.contains("streakThreshold") || force) {
        ostr << " m_streakThreshold: " << m_streakThreshold;
    }
    if (settingsKeys.contains("streakMinLength") || force) {
        ostr << " m_streakMinLength: " << m_streakMinLength;
    }
    if (settingsKeys.contains("streakHoughThreshold") || force) {
        ostr << " m_streakHoughThreshold: " << m_streakHoughThreshold;
    }
    if (settingsKeys.contains("streakMaxGap") || force) {
        ostr << " m_streakMaxGap: " << m_streakMaxGap;
    }
    if (settingsKeys.contains("streakPersistenceFrames") || force) {
        ostr << " m_streakPersistenceFrames: " << m_streakPersistenceFrames;
    }
    if (settingsKeys.contains("streakDownscale") || force) {
        ostr << " m_streakDownscale: " << m_streakDownscale;
    }
    if (settingsKeys.contains("streakDebugView") || force) {
        ostr << " m_streakDebugView: " << m_streakDebugView;
    }
    if (settingsKeys.contains("streakOverlayStyle") || force) {
        ostr << " m_streakOverlayStyle: " << m_streakOverlayStyle;
    }
    if (settingsKeys.contains("streakLineEnhancementPlacement") || force) {
        ostr << " m_streakLineEnhancementPlacement: " << m_streakLineEnhancementPlacement;
    }
    if (settingsKeys.contains("streakColor") || force) {
        ostr << " m_streakColor: " << m_streakColor.name().toStdString();
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
    if (settingsKeys.contains("plateSolveSearchRadius") || force) {
        ostr << " m_plateSolveSearchRadius: " << m_plateSolveSearchRadius;
    }
    if (settingsKeys.contains("plateSolveUseCurrentDateTime") || force) {
        ostr << " m_plateSolveUseCurrentDateTime: " << m_plateSolveUseCurrentDateTime;
    }
    if (settingsKeys.contains("plateSolveDateTime") || force) {
        ostr << " m_plateSolveDateTime: " << m_plateSolveDateTime.toString(Qt::ISODateWithMs).toStdString();
    }
    if (settingsKeys.contains("plateSolveUseDownloadedCatalog") || force) {
        ostr << " m_plateSolveUseDownloadedCatalog: " << m_plateSolveUseDownloadedCatalog;
    }
    if (settingsKeys.contains("videoPostProcess") || force) {
        ostr << " m_videoPostProcess: " << m_recordMode;
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
    if (settingsKeys.contains("trackObjectMinElevation") || force) {
        ostr << " m_trackObjectMinElevation: " << m_trackObjectMinElevation;
    }
    if (settingsKeys.contains("trackObjectColor") || force) {
        ostr << " m_trackObjectColor: " << m_trackObjectColor.name().toStdString();
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
    if (settingsKeys.contains("yoloDisappearDebounce") || force) {
        ostr << " m_yoloDisappearDebounce: " << m_yoloDisappearDebounce;
    }
    if (settingsKeys.contains("yoloDnnTarget") || force) {
        ostr << " m_yoloDnnTarget: " << m_yoloDnnTarget;
    }
    if (settingsKeys.contains("objectDeviceSettings") || force)
    {
        ostr << " m_objectDeviceSettings: [";
        QHash<QString, QList<ObjectDeviceSettings *> *>::const_iterator it;
        for (it = m_objectDeviceSettings.cbegin(); it != m_objectDeviceSettings.cend(); ++it)
        {
            ostr << " class=" << it.key().toStdString() << " settings=";
            if (it.value())
            {
                ostr << "[";
                for (const auto *devSettings : *it.value()) {
                    if (devSettings) {
                        devSettings->getDebugString(ostr);
                    }
                }
                ostr << "]";
            }
            else
            {
                ostr << "null";
            }
        }
        ostr << " ]";
    }
    if (settingsKeys.contains("audioMute") || force) {
        ostr << " m_audioMute: " << m_audioMute;
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
    return m_cameraProtocol == "alpaca";
}

bool CameraSettings::isAsiCamera() const
{
    return m_cameraProtocol == "asi";
}

bool CameraSettings::isQtCamera() const
{
    return m_cameraProtocol == "qt";
}

bool CameraSettings::isFileCamera() const
{
    return m_cameraProtocol == "file";
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
    if (isFileCamera()) {
        return m_cameraDescription.isEmpty() ? QStringLiteral("file:") : QStringLiteral("file:%1").arg(m_cameraDescription);
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
