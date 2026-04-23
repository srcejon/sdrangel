///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
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
#include <QColor>
#include <sstream>

#include "util/simpleserializer.h"
#include "settings/serializable.h"
#include "camerasettings.h"

CameraSettings::CameraSettings() :
    m_rollupState(nullptr)
{
    resetToDefaults();
}

void CameraSettings::resetToDefaults()
{
    m_title = "Camera";
    m_rgbColor = QColor(64, 128, 255).rgb();
    m_cameraAPI = CameraAPIAlpaca;
    m_cameraId.clear();
    m_resolutionWidth = 1280;
    m_resolutionHeight = 720;
    m_framesPerSecond = 10;
    m_exposureTimeMs = 50;
    m_isoSensitivity = 400;
    m_alpacaHost = "127.0.0.1";
    m_alpacaPort = 11111;
    m_alpacaCameraId = 0;
    m_alpacaBinX = 1;
    m_alpacaBinY = 1;
    m_alpacaGain = -1;
    m_alpacaOffset = -1;
    m_alpacaReadoutMode = 0;
    m_saveImage = false;
    m_imageFileName = "camera.jpg";
    m_saveVideo = false;
    m_videoFileName = "camera.mp4";
    m_captureActive = false;
    m_workspaceIndex = 0;
    m_geometryBytes.clear();
}

QByteArray CameraSettings::serialize() const
{
    SimpleSerializer s(1);

    s.writeString(1, m_title);
    s.writeU32(2, m_rgbColor);
    s.writeS32(3, static_cast<int>(m_cameraAPI));
    s.writeString(4, m_cameraId);
    s.writeS32(5, m_resolutionWidth);
    s.writeS32(6, m_resolutionHeight);
    s.writeS32(7, m_framesPerSecond);
    s.writeS32(8, m_exposureTimeMs);
    s.writeS32(9, m_isoSensitivity);
    s.writeString(10, m_alpacaHost);
    s.writeU32(11, m_alpacaPort);
    s.writeS32(12, m_alpacaCameraId);
    s.writeBool(13, m_saveImage);
    s.writeString(14, m_imageFileName);
    s.writeBool(15, m_saveVideo);
    s.writeString(16, m_videoFileName);
    s.writeBool(17, m_captureActive);

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
        m_cameraAPI = static_cast<CameraAPI>(itmp);
        d.readString(4, &m_cameraId, "");
        d.readS32(5, &m_resolutionWidth, 1280);
        d.readS32(6, &m_resolutionHeight, 720);
        d.readS32(7, &m_framesPerSecond, 10);
        d.readS32(8, &m_exposureTimeMs, 50);
        d.readS32(9, &m_isoSensitivity, 400);
        m_resolutionWidth = std::max(16, m_resolutionWidth);
        m_resolutionHeight = std::max(16, m_resolutionHeight);
        m_framesPerSecond = std::max(1, m_framesPerSecond);
        m_exposureTimeMs = std::max(1, m_exposureTimeMs);
        m_isoSensitivity = std::max(1, m_isoSensitivity);
        d.readString(10, &m_alpacaHost, "127.0.0.1");
        d.readU32(11, &utmp, 11111);
        m_alpacaPort = (utmp <= 65535) ? static_cast<uint16_t>(utmp) : 11111;
        d.readS32(12, &m_alpacaCameraId, 0);
        d.readBool(13, &m_saveImage, false);
        d.readString(14, &m_imageFileName, "camera.jpg");
        d.readBool(15, &m_saveVideo, false);
        d.readString(16, &m_videoFileName, "camera.mp4");
        d.readBool(17, &m_captureActive, false);

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

        return true;
    }

    resetToDefaults();
    return false;
}

void CameraSettings::applySettings(const QStringList& settingsKeys, const CameraSettings& settings)
{
    if (settingsKeys.contains("title")) {
        m_title = settings.m_title;
    }
    if (settingsKeys.contains("rgbColor")) {
        m_rgbColor = settings.m_rgbColor;
    }
    if (settingsKeys.contains("cameraAPI")) {
        m_cameraAPI = settings.m_cameraAPI;
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
    if (settingsKeys.contains("exposureTimeMs")) {
        m_exposureTimeMs = std::max(1, settings.m_exposureTimeMs);
    }
    if (settingsKeys.contains("isoSensitivity")) {
        m_isoSensitivity = std::max(1, settings.m_isoSensitivity);
    }
    if (settingsKeys.contains("alpacaHost")) {
        m_alpacaHost = settings.m_alpacaHost;
    }
    if (settingsKeys.contains("alpacaPort")) {
        m_alpacaPort = settings.m_alpacaPort;
    }
    if (settingsKeys.contains("alpacaCameraId")) {
        m_alpacaCameraId = settings.m_alpacaCameraId;
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
    if (settingsKeys.contains("captureActive")) {
        m_captureActive = settings.m_captureActive;
    }
    if (settingsKeys.contains("workspaceIndex")) {
        m_workspaceIndex = settings.m_workspaceIndex;
    }
}

QString CameraSettings::getDebugString(const QStringList& settingsKeys, bool force) const
{
    std::ostringstream ostr;

    if (settingsKeys.contains("cameraAPI") || force) {
        ostr << " m_cameraAPI: " << static_cast<int>(m_cameraAPI);
    }
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
    if (settingsKeys.contains("alpacaCameraId") || force) {
        ostr << " m_alpacaCameraId: " << m_alpacaCameraId;
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
    if (settingsKeys.contains("captureActive") || force) {
        ostr << " m_captureActive: " << m_captureActive;
    }

    return QString(ostr.str().c_str());
}
