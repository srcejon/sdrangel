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
#include <QColor>
#include <QDataStream>
#include <QIODevice>
#include <sstream>

#include "util/simpleserializer.h"
#include "util/httpdownloadmanager.h"
#include "settings/serializable.h"
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
    m_cameraId.clear();
    m_resolutionWidth = 1280;
    m_resolutionHeight = 720;
    m_framesPerSecond = 10;
    m_captureMode = CaptureModeFrameRate;
    m_captureInterval = 1.0;
    m_captureIntervalUnits = CaptureIntervalSeconds;
    m_exposureTimeMs = 50;
    m_isoSensitivity = -1; // -1 is auto
    m_alpacaHost = "127.0.0.1";
    m_alpacaPort = 11111;
    m_alpacaBinX = 1;
    m_alpacaBinY = 1;
    m_alpacaGain = -1;
    m_alpacaOffset = -1;
    m_alpacaReadoutMode = 0;
    m_saveImage = false;
    m_imageFileName = "camera.jpg";
    m_saveVideo = false;
    m_videoFileName = "camera.mp4";
    m_workspaceIndex = 0;
    m_geometryBytes.clear();
    m_postProcessWhiteBalanceMode = 0;
    m_postProcessWhiteBalanceRedGain = 1.0;
    m_postProcessWhiteBalanceGreenGain = 1.0;
    m_postProcessWhiteBalanceBlueGain = 1.0;
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
    m_overlayText = false;
    m_overlayTextString = QStringLiteral(DEFAULT_OVERLAY_TEXT_STRING);
    m_overlayTextColor = Qt::white;
    m_overlayTextFontFamily.clear();
    m_overlayTextFontScale = 12.0;
    m_overlayTextPosX = 4;
    m_overlayTextPosY = 0;
    m_diffMask = false;
    m_dilationSize = 3;
    m_overlayFontFamily.clear();
    m_overlayFontScale = 12.0;
    m_motionDetect = false;
    m_motionBoxColor = Qt::red;
    m_minContourArea = 100;
    m_videoPostProcess = false;
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
    m_audioMute = true;
    m_audioDeviceName.clear();
    m_whiteBalanceMode = 0;
    m_exposureCompensation = 0.0;
    m_focusMode = 0;
    m_focusDistance = 1.0;
    m_zoomFactor = 1.0;
}

QByteArray CameraSettings::serialize() const
{
    SimpleSerializer s(2);

    s.writeString(1, m_title);
    s.writeU32(2, m_rgbColor);
    s.writeString(4, m_cameraId);
    s.writeS32(5, m_resolutionWidth);
    s.writeS32(6, m_resolutionHeight);
    s.writeS32(7, m_framesPerSecond);
    s.writeS32(12, m_captureMode);
    s.writeDouble(8, m_exposureTimeMs);
    s.writeS32(9, m_isoSensitivity);
    s.writeString(10, m_alpacaHost);
    s.writeU32(11, m_alpacaPort);
    s.writeBool(13, m_saveImage);
    s.writeString(14, m_imageFileName);
    s.writeBool(15, m_saveVideo);
    s.writeString(16, m_videoFileName);

    if (m_rollupState) {
        s.writeBlob(18, m_rollupState->serialize());
    }

    s.writeS32(19, m_workspaceIndex);
    s.writeBlob(20, m_geometryBytes);
    s.writeS32(21, m_alpacaBinX);
    s.writeS32(22, m_alpacaBinY);
    s.writeS32(23, m_alpacaGain);
    s.writeS32(24, m_alpacaReadoutMode);
    s.writeS32(25, m_alpacaOffset);
    s.writeS32(69, m_postProcessWhiteBalanceMode);
    s.writeDouble(70, m_postProcessWhiteBalanceRedGain);
    s.writeDouble(71, m_postProcessWhiteBalanceGreenGain);
    s.writeDouble(72, m_postProcessWhiteBalanceBlueGain);
    s.writeDouble(76, m_saturation);
    s.writeDouble(73, m_gamma);
    s.writeS32(77, m_gaussianBlur);
    s.writeS32(78, m_medianBlur);
    s.writeDouble(79, m_sharpen);
    s.writeDouble(80, m_sobelEdge);
    s.writeBool(74, m_flipX);
    s.writeBool(75, m_flipY);
    s.writeDouble(26, m_brightness);
    s.writeDouble(27, m_contrast);
    s.writeBool(28, m_invertColors);
    s.writeBool(29, m_overlayDateTime);
    s.writeU32(30, m_dateTimeColor.rgba());
    s.writeBool(31, m_diffMask);
    s.writeS32(32, m_dilationSize);
    s.writeString(33, m_overlayFontFamily);
    s.writeDouble(34, m_overlayFontScale);
    s.writeBool(35, m_motionDetect);
    s.writeU32(36, m_motionBoxColor.rgba());
    s.writeS32(37, m_minContourArea);
    s.writeBool(38, m_videoPostProcess);
    s.writeBool(39, m_overlaySpectrum);
    s.writeString(40, m_spectrumDevice);
    s.writeS32(41, m_spectrumOffsetX);
    s.writeS32(42, m_spectrumOffsetY);
    s.writeDouble(43, m_spectrumScale);
    s.writeString(44, m_dateTimeFormat);
    s.writeS32(45, m_dateTimePosX);
    s.writeS32(46, m_dateTimePosY);
    s.writeBool(62, m_overlayText);
    s.writeString(63, m_overlayTextString);
    s.writeU32(64, m_overlayTextColor.rgba());
    s.writeString(65, m_overlayTextFontFamily);
    s.writeDouble(66, m_overlayTextFontScale);
    s.writeS32(67, m_overlayTextPosX);
    s.writeS32(68, m_overlayTextPosY);
    s.writeBool(47, m_yoloEnabled);
    s.writeString(48, m_yoloModelPath);
    s.writeString(49, m_yoloLabelsPath);
    s.writeDouble(50, m_yoloConfThreshold);
    s.writeDouble(51, m_yoloNmsThreshold);
    s.writeU32(52, m_yoloBoxColor.rgba());
    s.writeDouble(60, m_yoloDisappearDebounce);
    s.writeBlob(61, serializeObjectDeviceSettings(m_objectDeviceSettings));
    s.writeBool(53, m_audioMute);
    s.writeString(54, m_audioDeviceName);
    s.writeS32(55, m_whiteBalanceMode);
    s.writeDouble(56, m_exposureCompensation);
    s.writeS32(57, m_focusMode);
    s.writeDouble(58, m_focusDistance);
    s.writeDouble(59, m_zoomFactor);
    s.writeDouble(81, m_captureInterval);
    s.writeS32(82, m_captureIntervalUnits);

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
        int32_t itmp;
        uint32_t utmp;
        QByteArray bytetmp;

        d.readString(1, &m_title, "Camera");
        d.readU32(2, &m_rgbColor, QColor(64, 128, 255).rgb());
        d.readS32(3, &itmp, 0);
        d.readString(4, &m_cameraId, "");
        d.readS32(5, &m_resolutionWidth, 1280);
        d.readS32(6, &m_resolutionHeight, 720);
        d.readS32(7, &m_framesPerSecond, 10);
        d.readS32(12, (qint32 *) &m_captureMode, (qint32) CaptureModeFrameRate);
        int exposureTimeMs = 50;
        d.readS32(8, &exposureTimeMs, 50);
        m_exposureTimeMs = exposureTimeMs;
        d.readS32(9, &m_isoSensitivity, -1);
        m_resolutionWidth = std::max(16, m_resolutionWidth);
        m_resolutionHeight = std::max(16, m_resolutionHeight);
        m_framesPerSecond = std::max(1, m_framesPerSecond);
        d.readDouble(81, &m_captureInterval, 1.0);
        d.readS32(82, (qint32 *) &m_captureIntervalUnits, (qint32) CaptureIntervalSeconds);
        m_captureMode = qBound(CaptureModeFrameRate, m_captureMode, CaptureModeInterval);
        m_captureInterval = std::max(0.1, m_captureInterval);
        m_captureIntervalUnits = qBound(CaptureIntervalSeconds, m_captureIntervalUnits, CaptureIntervalMinutes);
        m_exposureTimeMs = std::max(0.001, m_exposureTimeMs);
        m_isoSensitivity = std::max(-1, m_isoSensitivity);
        d.readString(10, &m_alpacaHost, "127.0.0.1");
        d.readU32(11, &utmp, 11111);
        m_alpacaPort = (utmp <= 65535) ? static_cast<uint16_t>(utmp) : 11111;
        d.readBool(13, &m_saveImage, false);
        d.readString(14, &m_imageFileName, "camera.jpg");
        d.readBool(15, &m_saveVideo, false);
        d.readString(16, &m_videoFileName, "camera.mp4");

        if (m_rollupState)
        {
            d.readBlob(18, &bytetmp);
            m_rollupState->deserialize(bytetmp);
        }

        d.readS32(19, &m_workspaceIndex, 0);
        d.readBlob(20, &m_geometryBytes);
        d.readS32(21, &m_alpacaBinX, 1);
        d.readS32(22, &m_alpacaBinY, 1);
        d.readS32(23, &m_alpacaGain, -1);
        d.readS32(24, &m_alpacaReadoutMode, 0);
        d.readS32(25, &m_alpacaOffset, -1);
        m_alpacaBinX = std::max(1, m_alpacaBinX);
        m_alpacaBinY = std::max(1, m_alpacaBinY);
        m_alpacaReadoutMode = std::max(0, m_alpacaReadoutMode);
        d.readS32(69, &m_postProcessWhiteBalanceMode, 0);
        d.readDouble(70, &m_postProcessWhiteBalanceRedGain, 1.0);
        d.readDouble(71, &m_postProcessWhiteBalanceGreenGain, 1.0);
        d.readDouble(72, &m_postProcessWhiteBalanceBlueGain, 1.0);
        d.readDouble(76, &m_saturation, 1.0);
        d.readDouble(73, &m_gamma, 1.0);
        d.readS32(77, &m_gaussianBlur, 0);
        d.readS32(78, &m_medianBlur, 0);
        d.readDouble(79, &m_sharpen, 0.0);
        d.readDouble(80, &m_sobelEdge, 0.0);
        d.readBool(74, &m_flipX, false);
        d.readBool(75, &m_flipY, false);
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

        d.readDouble(26, &m_brightness, 0.0);
        d.readDouble(27, &m_contrast, 1.0);
        d.readBool(28, &m_invertColors, false);
        d.readBool(29, &m_overlayDateTime, false);
        uint32_t colorRgba = QColor(Qt::white).rgba();
        d.readU32(30, &colorRgba, QColor(Qt::white).rgba());
        m_dateTimeColor = QColor::fromRgba(colorRgba);
        d.readBool(31, &m_diffMask, false);
        d.readS32(32, &m_dilationSize, 3);
        m_brightness = qBound(-100.0, m_brightness, 100.0);
        m_contrast = qBound(0.1, m_contrast, 3.0);
        m_dilationSize = qBound(0, m_dilationSize, 20);

        d.readString(33, &m_overlayFontFamily, "");
        d.readDouble(34, &m_overlayFontScale, 12.0);
        d.readBool(35, &m_motionDetect, false);
        uint32_t motionBoxColorRgba = QColor(Qt::red).rgba();
        d.readU32(36, &motionBoxColorRgba, QColor(Qt::red).rgba());
        m_motionBoxColor = QColor::fromRgba(motionBoxColorRgba);
        d.readS32(37, &m_minContourArea, 100);
        m_overlayFontScale = qBound(4.0, m_overlayFontScale, 144.0);
        m_minContourArea = qBound(0, m_minContourArea, 10000);
        d.readBool(38, &m_videoPostProcess, false);
        d.readBool(39, &m_overlaySpectrum, false);
        d.readString(40, &m_spectrumDevice, "");
        d.readS32(41, &m_spectrumOffsetX, 0);
        d.readS32(42, &m_spectrumOffsetY, 0);
        m_spectrumOffsetX = qBound(-4096, m_spectrumOffsetX, 4096);
        m_spectrumOffsetY = qBound(-4096, m_spectrumOffsetY, 4096);
        d.readDouble(43, &m_spectrumScale, 1.0);
        m_spectrumScale = qBound(0.1, m_spectrumScale, 4.0);
        d.readString(44, &m_dateTimeFormat, "yyyy-MM-dd hh:mm:ss");
        d.readS32(45, &m_dateTimePosX, 4);
        d.readS32(46, &m_dateTimePosY, 0);
        m_dateTimePosX = qBound(0, m_dateTimePosX, 4096);
        m_dateTimePosY = qBound(0, m_dateTimePosY, 4096);
        d.readBool(62, &m_overlayText, false);
        d.readString(63, &m_overlayTextString, DEFAULT_OVERLAY_TEXT_STRING);
        uint32_t overlayTextColorRgba = QColor(Qt::white).rgba();
        d.readU32(64, &overlayTextColorRgba, QColor(Qt::white).rgba());
        m_overlayTextColor = QColor::fromRgba(overlayTextColorRgba);
        d.readString(65, &m_overlayTextFontFamily, "");
        d.readDouble(66, &m_overlayTextFontScale, 12.0);
        m_overlayTextFontScale = qBound(4.0, m_overlayTextFontScale, 144.0);
        d.readS32(67, &m_overlayTextPosX, 4);
        d.readS32(68, &m_overlayTextPosY, 0);
        m_overlayTextPosX = qBound(0, m_overlayTextPosX, 4096);
        m_overlayTextPosY = qBound(0, m_overlayTextPosY, 4096);
        d.readBool(47, &m_yoloEnabled, false);
        d.readString(48, &m_yoloModelPath, "");
        d.readString(49, &m_yoloLabelsPath, "");
        d.readDouble(50, &m_yoloConfThreshold, 0.5);
        d.readDouble(51, &m_yoloNmsThreshold, 0.45);
        m_yoloConfThreshold = qBound(0.0, m_yoloConfThreshold, 1.0);
        m_yoloNmsThreshold = qBound(0.0, m_yoloNmsThreshold, 1.0);
        uint32_t yoloBoxColorRgba = QColor(Qt::green).rgba();
        d.readU32(52, &yoloBoxColorRgba, QColor(Qt::green).rgba());
        m_yoloBoxColor = QColor::fromRgba(yoloBoxColorRgba);
        d.readDouble(60, &m_yoloDisappearDebounce, 0.0);
        m_yoloDisappearDebounce = qBound(0.0, m_yoloDisappearDebounce, 60.0);
        d.readBlob(61, &bytetmp);
        deserializeObjectDeviceSettings(bytetmp, m_objectDeviceSettings);

        d.readBool(53, &m_audioMute, true);
        d.readString(54, &m_audioDeviceName, "");

        d.readS32(55, &m_whiteBalanceMode, 0);
        m_whiteBalanceMode = std::max(0, m_whiteBalanceMode);
        d.readDouble(56, &m_exposureCompensation, 0.0);
        m_exposureCompensation = qBound(-2.0, m_exposureCompensation, 2.0);
        d.readS32(57, &m_focusMode, 0);
        m_focusMode = std::max(0, m_focusMode);
        d.readDouble(58, &m_focusDistance, 1.0);
        m_focusDistance = qBound(0.0, m_focusDistance, 1.0);
        d.readDouble(59, &m_zoomFactor, 1.0);
        m_zoomFactor = std::max(1.0, m_zoomFactor);

        return true;
    }
    else if (d.getVersion() == 2)
    {
        int32_t itmp;
        uint32_t utmp;
        QByteArray bytetmp;

        d.readString(1, &m_title, "Camera");
        d.readU32(2, &m_rgbColor, QColor(64, 128, 255).rgb());
        d.readS32(3, &itmp, 0);
        d.readString(4, &m_cameraId, "");
        d.readS32(5, &m_resolutionWidth, 1280);
        d.readS32(6, &m_resolutionHeight, 720);
        d.readS32(7, &m_framesPerSecond, 10);
        d.readS32(12, (qint32 *) &m_captureMode, (qint32) CaptureModeFrameRate);
        d.readDouble(8, &m_exposureTimeMs, 50.0);
        d.readS32(9, &m_isoSensitivity, -1);
        m_resolutionWidth = std::max(16, m_resolutionWidth);
        m_resolutionHeight = std::max(16, m_resolutionHeight);
        m_framesPerSecond = std::max(1, m_framesPerSecond);
        d.readDouble(81, &m_captureInterval, 1.0);
        d.readS32(82, (qint32 *) &m_captureIntervalUnits, (qint32) CaptureIntervalSeconds);
        m_captureMode = qBound(CaptureModeFrameRate, m_captureMode, CaptureModeInterval);
        m_captureInterval = std::max(0.1, m_captureInterval);
        m_captureIntervalUnits = qBound(CaptureIntervalSeconds, m_captureIntervalUnits, CaptureIntervalMinutes);
        m_exposureTimeMs = std::max(0.001, m_exposureTimeMs);
        m_isoSensitivity = std::max(-1, m_isoSensitivity);
        d.readString(10, &m_alpacaHost, "127.0.0.1");
        d.readU32(11, &utmp, 11111);
        m_alpacaPort = (utmp <= 65535) ? static_cast<uint16_t>(utmp) : 11111;
        d.readBool(13, &m_saveImage, false);
        d.readString(14, &m_imageFileName, "camera.jpg");
        d.readBool(15, &m_saveVideo, false);
        d.readString(16, &m_videoFileName, "camera.mp4");

        if (m_rollupState)
        {
            d.readBlob(18, &bytetmp);
            m_rollupState->deserialize(bytetmp);
        }

        d.readS32(19, &m_workspaceIndex, 0);
        d.readBlob(20, &m_geometryBytes);
        d.readS32(21, &m_alpacaBinX, 1);
        d.readS32(22, &m_alpacaBinY, 1);
        d.readS32(23, &m_alpacaGain, -1);
        d.readS32(24, &m_alpacaReadoutMode, 0);
        d.readS32(25, &m_alpacaOffset, -1);
        m_alpacaBinX = std::max(1, m_alpacaBinX);
        m_alpacaBinY = std::max(1, m_alpacaBinY);
        m_alpacaReadoutMode = std::max(0, m_alpacaReadoutMode);
        d.readS32(69, &m_postProcessWhiteBalanceMode, 0);
        d.readDouble(70, &m_postProcessWhiteBalanceRedGain, 1.0);
        d.readDouble(71, &m_postProcessWhiteBalanceGreenGain, 1.0);
        d.readDouble(72, &m_postProcessWhiteBalanceBlueGain, 1.0);
        d.readDouble(76, &m_saturation, 1.0);
        d.readDouble(73, &m_gamma, 1.0);
        d.readS32(77, &m_gaussianBlur, 0);
        d.readS32(78, &m_medianBlur, 0);
        d.readDouble(79, &m_sharpen, 0.0);
        d.readDouble(80, &m_sobelEdge, 0.0);
        d.readBool(74, &m_flipX, false);
        d.readBool(75, &m_flipY, false);
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

        d.readDouble(26, &m_brightness, 0.0);
        d.readDouble(27, &m_contrast, 1.0);
        d.readBool(28, &m_invertColors, false);
        d.readBool(29, &m_overlayDateTime, false);
        uint32_t colorRgba = QColor(Qt::white).rgba();
        d.readU32(30, &colorRgba, QColor(Qt::white).rgba());
        m_dateTimeColor = QColor::fromRgba(colorRgba);
        d.readBool(31, &m_diffMask, false);
        d.readS32(32, &m_dilationSize, 3);
        m_brightness = qBound(-100.0, m_brightness, 100.0);
        m_contrast = qBound(0.1, m_contrast, 3.0);
        m_dilationSize = qBound(0, m_dilationSize, 20);

        d.readString(33, &m_overlayFontFamily, "");
        d.readDouble(34, &m_overlayFontScale, 12.0);
        d.readBool(35, &m_motionDetect, false);
        uint32_t motionBoxColorRgba = QColor(Qt::red).rgba();
        d.readU32(36, &motionBoxColorRgba, QColor(Qt::red).rgba());
        m_motionBoxColor = QColor::fromRgba(motionBoxColorRgba);
        d.readS32(37, &m_minContourArea, 100);
        m_overlayFontScale = qBound(4.0, m_overlayFontScale, 144.0);
        m_minContourArea = qBound(0, m_minContourArea, 10000);
        d.readBool(38, &m_videoPostProcess, false);
        d.readBool(39, &m_overlaySpectrum, false);
        d.readString(40, &m_spectrumDevice, "");
        d.readS32(41, &m_spectrumOffsetX, 0);
        d.readS32(42, &m_spectrumOffsetY, 0);
        m_spectrumOffsetX = qBound(-4096, m_spectrumOffsetX, 4096);
        m_spectrumOffsetY = qBound(-4096, m_spectrumOffsetY, 4096);
        d.readDouble(43, &m_spectrumScale, 1.0);
        m_spectrumScale = qBound(0.1, m_spectrumScale, 4.0);
        d.readString(44, &m_dateTimeFormat, "yyyy-MM-dd hh:mm:ss");
        d.readS32(45, &m_dateTimePosX, 4);
        d.readS32(46, &m_dateTimePosY, 0);
        m_dateTimePosX = qBound(0, m_dateTimePosX, 4096);
        m_dateTimePosY = qBound(0, m_dateTimePosY, 4096);
        d.readBool(62, &m_overlayText, false);
        d.readString(63, &m_overlayTextString, DEFAULT_OVERLAY_TEXT_STRING);
        uint32_t overlayTextColorRgba = QColor(Qt::white).rgba();
        d.readU32(64, &overlayTextColorRgba, QColor(Qt::white).rgba());
        m_overlayTextColor = QColor::fromRgba(overlayTextColorRgba);
        d.readString(65, &m_overlayTextFontFamily, "");
        d.readDouble(66, &m_overlayTextFontScale, 12.0);
        m_overlayTextFontScale = qBound(4.0, m_overlayTextFontScale, 144.0);
        d.readS32(67, &m_overlayTextPosX, 4);
        d.readS32(68, &m_overlayTextPosY, 0);
        m_overlayTextPosX = qBound(0, m_overlayTextPosX, 4096);
        m_overlayTextPosY = qBound(0, m_overlayTextPosY, 4096);
        d.readBool(47, &m_yoloEnabled, false);
        d.readString(48, &m_yoloModelPath, "");
        d.readString(49, &m_yoloLabelsPath, "");
        d.readDouble(50, &m_yoloConfThreshold, 0.5);
        d.readDouble(51, &m_yoloNmsThreshold, 0.45);
        m_yoloConfThreshold = qBound(0.0, m_yoloConfThreshold, 1.0);
        m_yoloNmsThreshold = qBound(0.0, m_yoloNmsThreshold, 1.0);
        uint32_t yoloBoxColorRgba = QColor(Qt::green).rgba();
        d.readU32(52, &yoloBoxColorRgba, QColor(Qt::green).rgba());
        m_yoloBoxColor = QColor::fromRgba(yoloBoxColorRgba);
        d.readDouble(60, &m_yoloDisappearDebounce, 0.0);
        m_yoloDisappearDebounce = qBound(0.0, m_yoloDisappearDebounce, 60.0);
        d.readBlob(61, &bytetmp);
        deserializeObjectDeviceSettings(bytetmp, m_objectDeviceSettings);

        d.readBool(53, &m_audioMute, true);
        d.readString(54, &m_audioDeviceName, "");
        d.readS32(55, &m_whiteBalanceMode, 0);
        m_whiteBalanceMode = std::max(0, m_whiteBalanceMode);
        d.readDouble(56, &m_exposureCompensation, 0.0);
        m_exposureCompensation = qBound(-2.0, m_exposureCompensation, 2.0);
        d.readS32(57, &m_focusMode, 0);
        m_focusMode = std::max(0, m_focusMode);
        d.readDouble(58, &m_focusDistance, 1.0);
        m_focusDistance = qBound(0.0, m_focusDistance, 1.0);
        d.readDouble(59, &m_zoomFactor, 1.0);
        m_zoomFactor = std::max(1.0, m_zoomFactor);

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
    if (settingsKeys.contains("cameraId")) {
        m_cameraId = settings.m_cameraId;
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
    if (settingsKeys.contains("alpacaHost")) {
        m_alpacaHost = settings.m_alpacaHost;
    }
    if (settingsKeys.contains("alpacaPort")) {
        m_alpacaPort = settings.m_alpacaPort;
    }
    if (settingsKeys.contains("alpacaBinX")) {
        m_alpacaBinX = std::max(1, settings.m_alpacaBinX);
    }
    if (settingsKeys.contains("alpacaBinY")) {
        m_alpacaBinY = std::max(1, settings.m_alpacaBinY);
    }
    if (settingsKeys.contains("alpacaGain")) {
        m_alpacaGain = settings.m_alpacaGain;
    }
    if (settingsKeys.contains("alpacaOffset")) {
        m_alpacaOffset = settings.m_alpacaOffset;
    }
    if (settingsKeys.contains("alpacaReadoutMode")) {
        m_alpacaReadoutMode = std::max(0, settings.m_alpacaReadoutMode);
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
    if (settingsKeys.contains("dilationSize")) {
        m_dilationSize = qBound(0, settings.m_dilationSize, 20);
    }
    if (settingsKeys.contains("overlayFontFamily")) {
        m_overlayFontFamily = settings.m_overlayFontFamily;
    }
    if (settingsKeys.contains("overlayFontScale")) {
        m_overlayFontScale = qBound(4.0, settings.m_overlayFontScale, 144.0);
    }
    if (settingsKeys.contains("motionDetect")) {
        m_motionDetect = settings.m_motionDetect;
    }
    if (settingsKeys.contains("motionBoxColor")) {
        m_motionBoxColor = settings.m_motionBoxColor;
    }
    if (settingsKeys.contains("minContourArea")) {
        m_minContourArea = qBound(0, settings.m_minContourArea, 10000);
    }
    if (settingsKeys.contains("videoPostProcess")) {
        m_videoPostProcess = settings.m_videoPostProcess;
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

    if (settingsKeys.contains("cameraId") || force) {
        ostr << " m_cameraId: " << m_cameraId.toStdString();
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
    if (settingsKeys.contains("alpacaHost") || force) {
        ostr << " m_alpacaHost: " << m_alpacaHost.toStdString();
    }
    if (settingsKeys.contains("alpacaPort") || force) {
        ostr << " m_alpacaPort: " << m_alpacaPort;
    }
    if (settingsKeys.contains("alpacaBinX") || force) {
        ostr << " m_alpacaBinX: " << m_alpacaBinX;
    }
    if (settingsKeys.contains("alpacaBinY") || force) {
        ostr << " m_alpacaBinY: " << m_alpacaBinY;
    }
    if (settingsKeys.contains("alpacaGain") || force) {
        ostr << " m_alpacaGain: " << m_alpacaGain;
    }
    if (settingsKeys.contains("alpacaOffset") || force) {
        ostr << " m_alpacaOffset: " << m_alpacaOffset;
    }
    if (settingsKeys.contains("alpacaReadoutMode") || force) {
        ostr << " m_alpacaReadoutMode: " << m_alpacaReadoutMode;
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
    if (settingsKeys.contains("dilationSize") || force) {
        ostr << " m_dilationSize: " << m_dilationSize;
    }
    if (settingsKeys.contains("overlayFontFamily") || force) {
        ostr << " m_overlayFontFamily: " << m_overlayFontFamily.toStdString();
    }
    if (settingsKeys.contains("overlayFontScale") || force) {
        ostr << " m_overlayFontScale: " << m_overlayFontScale;
    }
    if (settingsKeys.contains("motionDetect") || force) {
        ostr << " m_motionDetect: " << m_motionDetect;
    }
    if (settingsKeys.contains("minContourArea") || force) {
        ostr << " m_minContourArea: " << m_minContourArea;
    }
    if (settingsKeys.contains("videoPostProcess") || force) {
        ostr << " m_videoPostProcess: " << m_videoPostProcess;
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
    return m_cameraId.startsWith("alpaca:");
}

bool CameraSettings::isQtCamera() const
{
    return m_cameraId.startsWith("qt:");
}

int CameraSettings::cameraIdInt() const
{
    if (isAlpacaCamera())
    {
        QString idStr = m_cameraId.split(':')[1];
        bool ok;
        int id = idStr.toInt(&ok);
        if (ok) {
            return id;
        }
    }
    return -1;
}

QString CameraSettings::cameraIdString() const
{
    if (isQtCamera()) {
        return m_cameraId.split(':')[1];
    }
    return "";
}

QString CameraSettings::cameraDescription() const
{
    if (isAlpacaCamera() || isQtCamera()) {
        return m_cameraId.split(':')[2];
    }
    return "";
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
