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
    m_asiColorImageType = AsiColorImageTypeRgb24;
    m_saveImage = false;
    m_imageFileName = "camera.jpg";
    m_saveVideo = false;
    m_videoFileCameraPath.clear();
    m_videoFileName = "camera.mp4";
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
    m_azimuth = 0.0f;
    m_elevation = 0.0f;
    m_roll = 0.0f;
    m_rotator.clear();
    m_fov = 60.0f;
    m_lensProjection = LensProjectionRectilinear;
    m_workspaceIndex = 0;
    m_geometryBytes.clear();
    m_postProcessWhiteBalanceMode = 0;
    m_postProcessWhiteBalanceRedGain = 1.0;
    m_postProcessWhiteBalanceGreenGain = 1.0;
    m_postProcessWhiteBalanceBlueGain = 1.0;
    m_postProcessGreyscale = false;
    m_saturation = 1.0;
    m_gamma = 1.0;
    m_gaussianBlur = 0;
    m_medianBlur = 0;
    m_sharpen = 0.0;
    m_sobelEdge = 0.0;
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
    m_motionDetect = false;
    m_motionHistory = 500;
    m_motionVarThreshold = 16.0;
    m_motionDetectShadows = true;
    m_motionOpenSize = 0;
    m_motionCloseSize = 0;
    m_motionPersistenceFrames = 0;
    m_motionBoxColor = Qt::red;
    m_minContourArea = 100;
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
    s.writeFloat(141, m_azimuth);
    s.writeFloat(142, m_elevation);
    s.writeFloat(150, m_roll);
    s.writeString(143, m_rotator);
    s.writeFloat(144, m_fov);
    s.writeS32(149, m_lensProjection);

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
    s.writeBool(127, m_postProcessGreyscale);
    s.writeDouble(35, m_saturation);
    s.writeDouble(36, m_gamma);
    s.writeS32(37, m_gaussianBlur);
    s.writeS32(38, m_medianBlur);
    s.writeDouble(39, m_sharpen);
    s.writeDouble(40, m_sobelEdge);
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
    s.writeBool(59, m_motionDetect);
    s.writeS32(60, m_motionHistory);
    s.writeDouble(61, m_motionVarThreshold);
    s.writeBool(62, m_motionDetectShadows);
    s.writeS32(63, m_motionOpenSize);
    s.writeS32(64, m_motionCloseSize);
    s.writeS32(65, m_motionPersistenceFrames);
    s.writeU32(66, m_motionBoxColor.rgba());
    s.writeS32(67, m_minContourArea);
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
        m_resolutionWidth = std::max(16, m_resolutionWidth);
        m_resolutionHeight = std::max(16, m_resolutionHeight);
        m_framesPerSecond = std::max(1, m_framesPerSecond);
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
        m_alpacaFocuserDeviceNumber = std::max(0, m_alpacaFocuserDeviceNumber);
        m_alpacaFocusPosition = std::max(0, m_alpacaFocusPosition);
        m_alpacaFocusStepSize = std::max(1, m_alpacaFocusStepSize);
        m_alpacaFilterWheelDeviceNumber = std::max(0, m_alpacaFilterWheelDeviceNumber);
        m_alpacaFilterWheelPosition = std::max(0, m_alpacaFilterWheelPosition);
        m_captureMode = qBound(CaptureModeFrameRate, m_captureMode, CaptureModeInterval);
        m_captureInterval = std::max(0.1, m_captureInterval);
        m_captureIntervalUnits = qBound(CaptureIntervalSeconds, m_captureIntervalUnits, CaptureIntervalMinutes);
        m_exposureTimeMs = std::max(0.001, m_exposureTimeMs);
        m_isoSensitivity = std::max(-1, m_isoSensitivity);
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
        d.readFloat(141, &m_azimuth, 0.0f);
        d.readFloat(142, &m_elevation, 0.0f);
        d.readFloat(150, &m_roll, 0.0f);
        d.readString(143, &m_rotator, "");
        d.readFloat(144, &m_fov, 60.0f);
        d.readS32(149, (int *) &m_lensProjection, LensProjectionRectilinear);

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
        m_cameraBinX = std::max(1, m_cameraBinX);
        m_cameraBinY = std::max(1, m_cameraBinY);
        m_cameraNumX = std::max(0, m_cameraNumX);
        m_cameraNumY = std::max(0, m_cameraNumY);
        m_cameraStartX = std::max(0, m_cameraStartX);
        m_cameraStartY = std::max(0, m_cameraStartY);
        m_cameraReadoutMode = std::max(0, m_cameraReadoutMode);
        d.readS32(31, &m_postProcessWhiteBalanceMode, 0);
        d.readDouble(32, &m_postProcessWhiteBalanceRedGain, 1.0);
        d.readDouble(33, &m_postProcessWhiteBalanceGreenGain, 1.0);
        d.readDouble(34, &m_postProcessWhiteBalanceBlueGain, 1.0);
        d.readBool(127, &m_postProcessGreyscale, false);
        d.readDouble(35, &m_saturation, 1.0);
        d.readDouble(36, &m_gamma, 1.0);
        d.readS32(37, &m_gaussianBlur, 0);
        d.readS32(38, &m_medianBlur, 0);
        d.readDouble(39, &m_sharpen, 0.0);
        d.readDouble(40, &m_sobelEdge, 0.0);
        d.readBool(41, &m_flipX, false);
        d.readBool(42, &m_flipY, false);
        m_postProcessWhiteBalanceMode = qBound(0, m_postProcessWhiteBalanceMode, 2);
        m_postProcessWhiteBalanceRedGain = qBound(0.1, m_postProcessWhiteBalanceRedGain, 8.0);
        m_postProcessWhiteBalanceGreenGain = qBound(0.1, m_postProcessWhiteBalanceGreenGain, 8.0);
        m_postProcessWhiteBalanceBlueGain = qBound(0.1, m_postProcessWhiteBalanceBlueGain, 8.0);
        m_saturation = qBound(0.0, m_saturation, 3.0);
        m_gamma = qBound(0.1, m_gamma, 3.0);
        m_gaussianBlur = qBound(0, m_gaussianBlur, 15);
        m_medianBlur = qBound(0, m_medianBlur, 15);
        m_sharpen = qBound(0.0, m_sharpen, 3.0);
        m_sobelEdge = qBound(0.0, m_sobelEdge, 3.0);

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
        m_brightness = qBound(-100.0, m_brightness, 100.0);
        m_contrast = qBound(0.1, m_contrast, 3.0);
        m_diffThreshold = qBound(0, m_diffThreshold, 255);
        m_diffMaskOpenSize = qBound(0, m_diffMaskOpenSize, 20);
        m_dilationSize = qBound(0, m_dilationSize, 20);
        m_diffMaskHistoryFrames = qBound(1, m_diffMaskHistoryFrames, 120);
        m_diffMaskCloseSize = qBound(0, m_diffMaskCloseSize, 20);

        d.readString(53, &m_overlayFontFamily, "");
        d.readDouble(54, &m_overlayFontScale, 12.0);
        d.readS32(55, &m_detectionRoiX, 0);
        d.readS32(56, &m_detectionRoiY, 0);
        d.readS32(57, &m_detectionRoiWidth, 0);
        d.readS32(58, &m_detectionRoiHeight, 0);
        d.readBool(59, &m_motionDetect, false);
        d.readS32(60, &m_motionHistory, 500);
        d.readDouble(61, &m_motionVarThreshold, 16.0);
        d.readBool(62, &m_motionDetectShadows, true);
        d.readS32(63, &m_motionOpenSize, 0);
        d.readS32(64, &m_motionCloseSize, 0);
        d.readS32(65, &m_motionPersistenceFrames, 0);
        uint32_t motionBoxColorRgba = QColor(Qt::red).rgba();
        d.readU32(66, &motionBoxColorRgba, QColor(Qt::red).rgba());
        m_motionBoxColor = QColor::fromRgba(motionBoxColorRgba);
        d.readS32(67, &m_minContourArea, 100);
        m_overlayFontScale = qBound(4.0, m_overlayFontScale, 144.0);
        m_detectionRoiX = qBound(0, m_detectionRoiX, 4096);
        m_detectionRoiY = qBound(0, m_detectionRoiY, 4096);
        m_detectionRoiWidth = qBound(0, m_detectionRoiWidth, 4096);
        m_detectionRoiHeight = qBound(0, m_detectionRoiHeight, 4096);
        m_motionHistory = qBound(1, m_motionHistory, 5000);
        m_motionVarThreshold = qBound(1.0, m_motionVarThreshold, 200.0);
        m_motionOpenSize = qBound(0, m_motionOpenSize, 20);
        m_motionCloseSize = qBound(0, m_motionCloseSize, 20);
        m_motionPersistenceFrames = qBound(0, m_motionPersistenceFrames, 120);
        m_minContourArea = qBound(0, m_minContourArea, 10000);
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
        m_spectrumOffsetX = qBound(-4096, m_spectrumOffsetX, 4096);
        m_spectrumOffsetY = qBound(-4096, m_spectrumOffsetY, 4096);
        d.readDouble(73, &m_spectrumScale, 1.0);
        m_spectrumScale = qBound(0.1, m_spectrumScale, 4.0);
        d.readString(74, &m_dateTimeFormat, "yyyy-MM-dd hh:mm:ss");
        d.readS32(75, &m_dateTimePosX, 4);
        d.readS32(76, &m_dateTimePosY, 0);
        m_dateTimePosX = qBound(0, m_dateTimePosX, 4096);
        m_dateTimePosY = qBound(0, m_dateTimePosY, 4096);
        d.readBool(145, &m_equatorialGrid, false);
        uint32_t equatorialGridColorRgba = QColor(80, 170, 255).rgba();
        d.readU32(146, &equatorialGridColorRgba, QColor(80, 170, 255).rgba());
        m_equatorialGridColor = QColor::fromRgba(equatorialGridColorRgba);
        d.readBool(147, &m_altAzGrid, false);
        uint32_t altAzGridColorRgba = QColor(255, 170, 80).rgba();
        d.readU32(148, &altAzGridColorRgba, QColor(255, 170, 80).rgba());
        m_altAzGridColor = QColor::fromRgba(altAzGridColorRgba);
        d.readBool(77, &m_overlayText, false);
        d.readString(78, &m_overlayTextString, DEFAULT_OVERLAY_TEXT_STRING);
        uint32_t overlayTextColorRgba = QColor(Qt::white).rgba();
        d.readU32(79, &overlayTextColorRgba, QColor(Qt::white).rgba());
        m_overlayTextColor = QColor::fromRgba(overlayTextColorRgba);
        d.readString(80, &m_overlayTextFontFamily, "");
        d.readDouble(81, &m_overlayTextFontScale, 12.0);
        m_overlayTextFontScale = qBound(4.0, m_overlayTextFontScale, 144.0);
        d.readS32(82, &m_overlayTextPosX, 4);
        d.readS32(83, &m_overlayTextPosY, 0);
        m_overlayTextPosX = qBound(0, m_overlayTextPosX, 4096);
        m_overlayTextPosY = qBound(0, m_overlayTextPosY, 4096);
        d.readBool(84, &m_yoloEnabled, false);
        d.readString(85, &m_yoloModelPath, "");
        d.readString(86, &m_yoloLabelsPath, "");
        d.readDouble(87, &m_yoloConfThreshold, 0.5);
        d.readDouble(88, &m_yoloNmsThreshold, 0.45);
        m_yoloConfThreshold = qBound(0.0, m_yoloConfThreshold, 1.0);
        m_yoloNmsThreshold = qBound(0.0, m_yoloNmsThreshold, 1.0);
        uint32_t yoloBoxColorRgba = QColor(Qt::green).rgba();
        d.readU32(89, &yoloBoxColorRgba, QColor(Qt::green).rgba());
        m_yoloBoxColor = QColor::fromRgba(yoloBoxColorRgba);
        d.readDouble(90, &m_yoloDisappearDebounce, 0.0);
        m_yoloDisappearDebounce = qBound(0.0, m_yoloDisappearDebounce, 60.0);
        d.readS32(91, (qint32 *) &m_yoloDnnTarget, (qint32) CPU);
        d.readBlob(92, &bytetmp);
        deserializeObjectDeviceSettings(bytetmp, m_objectDeviceSettings);

        d.readBool(93, &m_audioMute, true);
        d.readString(94, &m_audioDeviceName, "");
        d.readS32(95, &m_whiteBalanceMode, 0);
        m_whiteBalanceMode = std::max(0, m_whiteBalanceMode);
        d.readDouble(96, &m_exposureCompensation, 0.0);
        m_exposureCompensation = qBound(-2.0, m_exposureCompensation, 2.0);
        d.readS32(97, &m_focusMode, 0);
        m_focusMode = std::max(0, m_focusMode);
        d.readDouble(98, &m_focusDistance, 1.0);
        m_focusDistance = qBound(0.0, m_focusDistance, 1.0);
        d.readDouble(99, &m_zoomFactor, 1.0);
        m_zoomFactor = std::max(1.0, m_zoomFactor);

        d.readString(115, &m_videoFileCameraPath, "");

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
        m_asiCoolerOn = qBound(-1, m_asiCoolerOn, 1);
        m_asiUsbBandwidth = std::max(-1, m_asiUsbBandwidth);
        m_asiHighSpeedMode = qBound(-1, m_asiHighSpeedMode, 1);
        m_asiColorImageType = qBound(AsiColorImageTypeRgb24, m_asiColorImageType, AsiColorImageTypeRaw16);
        m_stackFrameCount = qBound(1, m_stackFrameCount, 256);
        m_stackMethod = qBound(StackMethodAverage, m_stackMethod, StackMethodSigmaClippedAverage);
        m_stackAlignmentMethod = qBound(StackAlignmentNone, m_stackAlignmentMethod, StackAlignmentStarCentroidMatching);
        m_latitude = qBound(-90.0f, m_latitude, 90.0f);
        m_longitude = qBound(-180.0f, m_longitude, 180.0f);
        m_altitude = qBound(-1000.0f, m_altitude, 100000.0f);
        m_azimuth = std::fmod(m_azimuth, 360.0f);
        if (m_azimuth < 0.0f) {
            m_azimuth += 360.0f;
        }
        m_elevation = qBound(-90.0f, m_elevation, 90.0f);
        m_roll = std::fmod(m_roll, 360.0f);
        if (m_roll < 0.0f) {
            m_roll += 360.0f;
        }
        m_fov = qBound(0.01f, m_fov, 360.0f);
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
        m_resolutionWidth = std::max(16, settings.m_resolutionWidth);
    }
    if (settingsKeys.contains("resolutionHeight")) {
        m_resolutionHeight = std::max(16, settings.m_resolutionHeight);
    }
    if (settingsKeys.contains("framesPerSecond")) {
        m_framesPerSecond = std::max(1, settings.m_framesPerSecond);
    }
    if (settingsKeys.contains("captureMode")) {
        m_captureMode = qBound(CaptureModeFrameRate, settings.m_captureMode, CaptureModeInterval);
    }
    if (settingsKeys.contains("captureInterval")) {
        m_captureInterval = std::max(0.1, settings.m_captureInterval);
    }
    if (settingsKeys.contains("captureIntervalUnits")) {
        m_captureIntervalUnits = qBound(CaptureIntervalSeconds, settings.m_captureIntervalUnits, CaptureIntervalMinutes);
    }
    if (settingsKeys.contains("exposureTimeMs")) {
        m_exposureTimeMs = std::max(0.001, settings.m_exposureTimeMs);
    }
    if (settingsKeys.contains("isoSensitivity")) {
        m_isoSensitivity = std::max(-1, settings.m_isoSensitivity);
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
        m_alpacaFocuserDeviceNumber = std::max(0, settings.m_alpacaFocuserDeviceNumber);
    }
    if (settingsKeys.contains("alpacaFocusPosition")) {
        m_alpacaFocusPosition = std::max(0, settings.m_alpacaFocusPosition);
    }
    if (settingsKeys.contains("alpacaFocusStepSize")) {
        m_alpacaFocusStepSize = std::max(1, settings.m_alpacaFocusStepSize);
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
        m_alpacaFilterWheelDeviceNumber = std::max(0, settings.m_alpacaFilterWheelDeviceNumber);
    }
    if (settingsKeys.contains("alpacaFilterWheelPosition")) {
        m_alpacaFilterWheelPosition = std::max(0, settings.m_alpacaFilterWheelPosition);
    }
    if (settingsKeys.contains("cameraBinX") || settingsKeys.contains("alpacaBinX")) {
        m_cameraBinX = std::max(1, settings.m_cameraBinX);
    }
    if (settingsKeys.contains("cameraBinY") || settingsKeys.contains("alpacaBinY")) {
        m_cameraBinY = std::max(1, settings.m_cameraBinY);
    }
    if (settingsKeys.contains("cameraNumX") || settingsKeys.contains("alpacaNumX")) {
        m_cameraNumX = std::max(0, settings.m_cameraNumX);
    }
    if (settingsKeys.contains("cameraNumY") || settingsKeys.contains("alpacaNumY")) {
        m_cameraNumY = std::max(0, settings.m_cameraNumY);
    }
    if (settingsKeys.contains("cameraStartX") || settingsKeys.contains("alpacaStartX")) {
        m_cameraStartX = std::max(0, settings.m_cameraStartX);
    }
    if (settingsKeys.contains("cameraStartY") || settingsKeys.contains("alpacaStartY")) {
        m_cameraStartY = std::max(0, settings.m_cameraStartY);
    }
    if (settingsKeys.contains("cameraGain") || settingsKeys.contains("alpacaGain")) {
        m_cameraGain = settings.m_cameraGain;
    }
    if (settingsKeys.contains("cameraOffset") || settingsKeys.contains("alpacaOffset")) {
        m_cameraOffset = settings.m_cameraOffset;
    }
    if (settingsKeys.contains("cameraReadoutMode") || settingsKeys.contains("alpacaReadoutMode")) {
        m_cameraReadoutMode = std::max(0, settings.m_cameraReadoutMode);
    }
    if (settingsKeys.contains("asiCoolerOn")) {
        m_asiCoolerOn = qBound(-1, settings.m_asiCoolerOn, 1);
    }
    if (settingsKeys.contains("asiTargetTemp")) {
        m_asiTargetTemp = settings.m_asiTargetTemp;
    }
    if (settingsKeys.contains("asiUsbBandwidth")) {
        m_asiUsbBandwidth = std::max(-1, settings.m_asiUsbBandwidth);
    }
    if (settingsKeys.contains("asiHighSpeedMode")) {
        m_asiHighSpeedMode = qBound(-1, settings.m_asiHighSpeedMode, 1);
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
    if (settingsKeys.contains("videoHwAcceleration")) {
        m_videoHwAcceleration = settings.m_videoHwAcceleration;
    }
    if (settingsKeys.contains("stackEnabled")) {
        m_stackEnabled = settings.m_stackEnabled;
    }
    if (settingsKeys.contains("stackFrameCount")) {
        m_stackFrameCount = qBound(1, settings.m_stackFrameCount, 256);
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
        m_latitude = qBound(-90.0f, settings.m_latitude, 90.0f);
    }
    if (settingsKeys.contains("longitude")) {
        m_longitude = qBound(-180.0f, settings.m_longitude, 180.0f);
    }
    if (settingsKeys.contains("altitude")) {
        m_altitude = qBound(-1000.0f, settings.m_altitude, 100000.0f);
    }
    if (settingsKeys.contains("positionSync")) {
        m_positionSync = settings.m_positionSync;
    }
    if (settingsKeys.contains("azimuth")) {
        m_azimuth = std::fmod(settings.m_azimuth, 360.0f);
        if (m_azimuth < 0.0f) {
            m_azimuth += 360.0f;
        }
    }
    if (settingsKeys.contains("elevation")) {
        m_elevation = qBound(-90.0f, settings.m_elevation, 90.0f);
    }
    if (settingsKeys.contains("roll")) {
        m_roll = std::fmod(settings.m_roll, 360.0f);
        if (m_roll < 0.0f) {
            m_roll += 360.0f;
        }
    }
    if (settingsKeys.contains("rotator")) {
        m_rotator = settings.m_rotator;
    }
    if (settingsKeys.contains("fov")) {
        m_fov = qBound(0.01f, settings.m_fov, 360.0f);
    }
    if (settingsKeys.contains("lensProjection")) {
        m_lensProjection = (LensProjection) qBound((int) LensProjectionRectilinear, (int) settings.m_lensProjection, (int) LensProjectionEquisolid);
    }
    if (settingsKeys.contains("workspaceIndex")) {
        m_workspaceIndex = settings.m_workspaceIndex;
    }
    if (settingsKeys.contains("brightness")) {
        m_brightness = qBound(-100.0, settings.m_brightness, 100.0);
    }
    if (settingsKeys.contains("postProcessWhiteBalanceMode")) {
        m_postProcessWhiteBalanceMode = qBound(0, settings.m_postProcessWhiteBalanceMode, 2);
    }
    if (settingsKeys.contains("postProcessWhiteBalanceRedGain")) {
        m_postProcessWhiteBalanceRedGain = qBound(0.1, settings.m_postProcessWhiteBalanceRedGain, 8.0);
    }
    if (settingsKeys.contains("postProcessWhiteBalanceGreenGain")) {
        m_postProcessWhiteBalanceGreenGain = qBound(0.1, settings.m_postProcessWhiteBalanceGreenGain, 8.0);
    }
    if (settingsKeys.contains("postProcessWhiteBalanceBlueGain")) {
        m_postProcessWhiteBalanceBlueGain = qBound(0.1, settings.m_postProcessWhiteBalanceBlueGain, 8.0);
    }
    if (settingsKeys.contains("postProcessGreyscale")) {
        m_postProcessGreyscale = settings.m_postProcessGreyscale;
    }
    if (settingsKeys.contains("saturation")) {
        m_saturation = qBound(0.0, settings.m_saturation, 3.0);
    }
    if (settingsKeys.contains("gamma")) {
        m_gamma = qBound(0.1, settings.m_gamma, 3.0);
    }
    if (settingsKeys.contains("gaussianBlur")) {
        m_gaussianBlur = qBound(0, settings.m_gaussianBlur, 15);
    }
    if (settingsKeys.contains("medianBlur")) {
        m_medianBlur = qBound(0, settings.m_medianBlur, 15);
    }
    if (settingsKeys.contains("sharpen")) {
        m_sharpen = qBound(0.0, settings.m_sharpen, 3.0);
    }
    if (settingsKeys.contains("sobelEdge")) {
        m_sobelEdge = qBound(0.0, settings.m_sobelEdge, 3.0);
    }
    if (settingsKeys.contains("flipX")) {
        m_flipX = settings.m_flipX;
    }
    if (settingsKeys.contains("flipY")) {
        m_flipY = settings.m_flipY;
    }
    if (settingsKeys.contains("contrast")) {
        m_contrast = qBound(0.1, settings.m_contrast, 3.0);
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
        m_diffThreshold = qBound(0, settings.m_diffThreshold, 255);
    }
    if (settingsKeys.contains("diffMaskOpenSize")) {
        m_diffMaskOpenSize = qBound(0, settings.m_diffMaskOpenSize, 20);
    }
    if (settingsKeys.contains("dilationSize")) {
        m_dilationSize = qBound(0, settings.m_dilationSize, 20);
    }
    if (settingsKeys.contains("diffMaskHistoryFrames")) {
        m_diffMaskHistoryFrames = qBound(1, settings.m_diffMaskHistoryFrames, 120);
    }
    if (settingsKeys.contains("diffMaskCloseSize")) {
        m_diffMaskCloseSize = qBound(0, settings.m_diffMaskCloseSize, 20);
    }
    if (settingsKeys.contains("overlayFontFamily")) {
        m_overlayFontFamily = settings.m_overlayFontFamily;
    }
    if (settingsKeys.contains("overlayFontScale")) {
        m_overlayFontScale = qBound(4.0, settings.m_overlayFontScale, 144.0);
    }
    if (settingsKeys.contains("detectionRoiX")) {
        m_detectionRoiX = qBound(0, settings.m_detectionRoiX, 4096);
    }
    if (settingsKeys.contains("detectionRoiY")) {
        m_detectionRoiY = qBound(0, settings.m_detectionRoiY, 4096);
    }
    if (settingsKeys.contains("detectionRoiWidth")) {
        m_detectionRoiWidth = qBound(0, settings.m_detectionRoiWidth, 4096);
    }
    if (settingsKeys.contains("detectionRoiHeight")) {
        m_detectionRoiHeight = qBound(0, settings.m_detectionRoiHeight, 4096);
    }
    if (settingsKeys.contains("motionDetect")) {
        m_motionDetect = settings.m_motionDetect;
    }
    if (settingsKeys.contains("motionHistory")) {
        m_motionHistory = qBound(1, settings.m_motionHistory, 5000);
    }
    if (settingsKeys.contains("motionVarThreshold")) {
        m_motionVarThreshold = qBound(1.0, settings.m_motionVarThreshold, 200.0);
    }
    if (settingsKeys.contains("motionDetectShadows")) {
        m_motionDetectShadows = settings.m_motionDetectShadows;
    }
    if (settingsKeys.contains("motionOpenSize")) {
        m_motionOpenSize = qBound(0, settings.m_motionOpenSize, 20);
    }
    if (settingsKeys.contains("motionCloseSize")) {
        m_motionCloseSize = qBound(0, settings.m_motionCloseSize, 20);
    }
    if (settingsKeys.contains("motionPersistenceFrames")) {
        m_motionPersistenceFrames = qBound(0, settings.m_motionPersistenceFrames, 120);
    }
    if (settingsKeys.contains("motionBoxColor")) {
        m_motionBoxColor = settings.m_motionBoxColor;
    }
    if (settingsKeys.contains("minContourArea")) {
        m_minContourArea = qBound(0, settings.m_minContourArea, 10000);
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
        m_spectrumOffsetX = qBound(-4096, settings.m_spectrumOffsetX, 4096);
    }
    if (settingsKeys.contains("spectrumOffsetY")) {
        m_spectrumOffsetY = qBound(-4096, settings.m_spectrumOffsetY, 4096);
    }
    if (settingsKeys.contains("spectrumScale")) {
        m_spectrumScale = qBound(0.1, settings.m_spectrumScale, 4.0);
    }
    if (settingsKeys.contains("dateTimeFormat")) {
        m_dateTimeFormat = settings.m_dateTimeFormat;
    }
    if (settingsKeys.contains("dateTimePosX")) {
        m_dateTimePosX = qBound(0, settings.m_dateTimePosX, 4096);
    }
    if (settingsKeys.contains("dateTimePosY")) {
        m_dateTimePosY = qBound(0, settings.m_dateTimePosY, 4096);
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
        m_overlayTextFontScale = qBound(4.0, settings.m_overlayTextFontScale, 144.0);
    }
    if (settingsKeys.contains("overlayTextPosX")) {
        m_overlayTextPosX = qBound(0, settings.m_overlayTextPosX, 4096);
    }
    if (settingsKeys.contains("overlayTextPosY")) {
        m_overlayTextPosY = qBound(0, settings.m_overlayTextPosY, 4096);
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
        m_yoloConfThreshold = qBound(0.0, settings.m_yoloConfThreshold, 1.0);
    }
    if (settingsKeys.contains("yoloNmsThreshold")) {
        m_yoloNmsThreshold = qBound(0.0, settings.m_yoloNmsThreshold, 1.0);
    }
    if (settingsKeys.contains("yoloBoxColor")) {
        m_yoloBoxColor = settings.m_yoloBoxColor;
    }
    if (settingsKeys.contains("yoloDisappearDebounce")) {
        m_yoloDisappearDebounce = qBound(0.0, settings.m_yoloDisappearDebounce, 60.0);
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
        m_whiteBalanceMode = std::max(0, settings.m_whiteBalanceMode);
    }
    if (settingsKeys.contains("exposureCompensation")) {
        m_exposureCompensation = qBound(-2.0, settings.m_exposureCompensation, 2.0);
    }
    if (settingsKeys.contains("focusMode")) {
        m_focusMode = std::max(0, settings.m_focusMode);
    }
    if (settingsKeys.contains("focusDistance")) {
        m_focusDistance = qBound(0.0, settings.m_focusDistance, 1.0);
    }
    if (settingsKeys.contains("zoomFactor")) {
        m_zoomFactor = std::max(1.0, settings.m_zoomFactor);
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
    if (settingsKeys.contains("sobelEdge") || force) {
        ostr << " m_sobelEdge: " << m_sobelEdge;
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
    if (settingsKeys.contains("motionDetect") || force) {
        ostr << " m_motionDetect: " << m_motionDetect;
    }
    if (settingsKeys.contains("motionHistory") || force) {
        ostr << " m_motionHistory: " << m_motionHistory;
    }
    if (settingsKeys.contains("motionVarThreshold") || force) {
        ostr << " m_motionVarThreshold: " << m_motionVarThreshold;
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
    const double interval = std::max(0.1, m_captureInterval);
    return m_captureIntervalUnits == CaptureIntervalMinutes ? interval * 60.0 : interval;
}

int CameraSettings::getCaptureIntervalMs() const
{
    return std::max(100, static_cast<int>(std::lround(getCaptureIntervalSeconds() * 1000.0)));
}

double CameraSettings::getCaptureFrameRate() const
{
    if (!isIntervalCaptureMode()) {
        return std::max(1, m_framesPerSecond);
    }

    return std::max(0.001, 1.0 / getCaptureIntervalSeconds());
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
