///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
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
#include <sstream>

#include <QColor>

#include "settings/serializable.h"
#include "util/simpleserializer.h"
#include "meteorsettings.h"

MeteorSettings::MeteorSettings() :
    m_channelMarker(nullptr),
    m_spectrumGUI(nullptr),
    m_headSpectrumGUI(nullptr),
    m_rollupState(nullptr)
{
    resetToDefaults();
}

void MeteorSettings::resetToDefaults()
{
    m_inputFrequencyOffset = 0;
    m_frequencyMode = Offset;
    m_frequency = 0;
    m_channelSampleRate = m_defaultSampleRate;
    m_powerLPFCutoff = 50.0f;
    m_detectionThresholdDB = 6.0f;
    m_minDurationMS = 5;
    m_maxDurationMS = 20000;
    m_maxFrequencyDrift = 50.0f;
    m_detectionsTableColumnHidden = 0;
    m_detectionBoxPaddingPixels = 4;
    m_detectionLabelMode = DetectionLabelNone;
    m_transmitterLatitude = 0.0;
    m_transmitterLongitude = 0.0;
    m_antennaAzimuth = 0.0f;
    m_antennaElevation = 0.0f;
    m_antennaBeamwidth = 0.0f;
    m_rotator.clear();
    m_rgbColor = QColor(255, 170, 0).rgb();
    m_title = "Meteor";
    m_streamIndex = 0;
    m_workspaceIndex = 0;
    m_hidden = false;
}

QByteArray MeteorSettings::serialize() const
{
    SimpleSerializer s(1);

    s.writeS32(1, m_inputFrequencyOffset);
    s.writeS32(2, (int) m_frequencyMode);
    s.writeS64(3, m_frequency);
    s.writeS32(4, m_channelSampleRate);
    s.writeFloat(5, m_powerLPFCutoff);
    s.writeFloat(6, m_detectionThresholdDB);
    s.writeS32(7, m_minDurationMS);
    s.writeS32(8, m_maxDurationMS);
    s.writeFloat(9, m_maxFrequencyDrift);
    s.writeU32(10, m_detectionsTableColumnHidden);
    s.writeS32(11, m_detectionBoxPaddingPixels);
    s.writeS32(12, (int) m_detectionLabelMode);
    s.writeDouble(13, m_transmitterLatitude);
    s.writeDouble(14, m_transmitterLongitude);
    s.writeFloat(15, m_antennaAzimuth);
    s.writeFloat(16, m_antennaElevation);
    s.writeFloat(17, m_antennaBeamwidth);
    s.writeString(18, m_rotator);

    s.writeU32(21, m_rgbColor);
    s.writeString(22, m_title);

    if (m_channelMarker) {
        s.writeBlob(23, m_channelMarker->serialize());
    }

    s.writeS32(24, m_streamIndex);

    if (m_spectrumGUI) {
        s.writeBlob(25, m_spectrumGUI->serialize());
    }

    if (m_headSpectrumGUI) {
        s.writeBlob(26, m_headSpectrumGUI->serialize());
    }

    if (m_rollupState) {
        s.writeBlob(30, m_rollupState->serialize());
    }

    s.writeS32(32, m_workspaceIndex);
    s.writeBlob(33, m_geometryBytes);
    s.writeBool(34, m_hidden);

    return s.final();
}

bool MeteorSettings::deserialize(const QByteArray& data)
{
    SimpleDeserializer d(data);

    if (!d.isValid())
    {
        resetToDefaults();
        return false;
    }

    if (d.getVersion() == 1)
    {
        QByteArray bytetmp;

        d.readS32(1, &m_inputFrequencyOffset, 0);
        d.readS32(2, (int *) &m_frequencyMode, (int) Offset);
        d.readS64(3, &m_frequency, 0);
        d.readS32(4, &m_channelSampleRate, m_defaultSampleRate);
        m_channelSampleRate = validatedSampleRate(m_channelSampleRate);
        d.readFloat(5, &m_powerLPFCutoff, 50.0f);
        d.readFloat(6, &m_detectionThresholdDB, 6.0f);
        d.readS32(7, &m_minDurationMS, 5);
        d.readS32(8, &m_maxDurationMS, 20000);
        d.readFloat(9, &m_maxFrequencyDrift, 50.0f);
        d.readU32(10, &m_detectionsTableColumnHidden, 0);
        d.readS32(11, &m_detectionBoxPaddingPixels, 4);
        d.readS32(12, (int *) &m_detectionLabelMode, (int) DetectionLabelNone);
        m_detectionLabelMode = (DetectionLabelMode) std::clamp(
            (int) m_detectionLabelMode,
            (int) DetectionLabelNone,
            (int) DetectionLabelRight);
        d.readDouble(13, &m_transmitterLatitude, 0.0);
        d.readDouble(14, &m_transmitterLongitude, 0.0);
        d.readFloat(15, &m_antennaAzimuth, 0.0f);
        d.readFloat(16, &m_antennaElevation, 0.0f);
        d.readFloat(17, &m_antennaBeamwidth, 0.0f);
        d.readString(18, &m_rotator, "");
        m_transmitterLatitude = std::isfinite(m_transmitterLatitude)
            ? std::clamp(m_transmitterLatitude, -90.0, 90.0) : 0.0;
        m_transmitterLongitude = std::isfinite(m_transmitterLongitude)
            ? std::clamp(m_transmitterLongitude, -180.0, 180.0) : 0.0;
        m_antennaAzimuth = std::isfinite(m_antennaAzimuth)
            ? std::clamp(m_antennaAzimuth, 0.0f, 360.0f) : 0.0f;
        m_antennaElevation = std::isfinite(m_antennaElevation)
            ? std::clamp(m_antennaElevation, -90.0f, 90.0f) : 0.0f;
        m_antennaBeamwidth = std::isfinite(m_antennaBeamwidth)
            ? std::clamp(m_antennaBeamwidth, 0.0f, 360.0f) : 0.0f;

        d.readU32(21, &m_rgbColor, QColor(255, 170, 0).rgb());
        d.readString(22, &m_title, "Meteor");

        if (m_channelMarker)
        {
            d.readBlob(23, &bytetmp);
            m_channelMarker->deserialize(bytetmp);
        }

        d.readS32(24, &m_streamIndex, 0);

        if (m_spectrumGUI)
        {
            d.readBlob(25, &bytetmp);
            m_spectrumGUI->deserialize(bytetmp);
        }

        if (m_headSpectrumGUI && d.readBlob(26, &bytetmp)) {
            m_headSpectrumGUI->deserialize(bytetmp);
        }

        if (m_rollupState)
        {
            d.readBlob(30, &bytetmp);
            m_rollupState->deserialize(bytetmp);
        }

        d.readS32(32, &m_workspaceIndex, 0);
        d.readBlob(33, &m_geometryBytes);
        d.readBool(34, &m_hidden, false);

        return true;
    }
    else
    {
        resetToDefaults();
        return false;
    }
}

void MeteorSettings::applySettings(const QStringList& settingsKeys, const MeteorSettings& settings)
{
    if (settingsKeys.contains("inputFrequencyOffset")) {
        m_inputFrequencyOffset = settings.m_inputFrequencyOffset;
    }
    if (settingsKeys.contains("frequencyMode")) {
        m_frequencyMode = settings.m_frequencyMode;
    }
    if (settingsKeys.contains("frequency")) {
        m_frequency = settings.m_frequency;
    }
    if (settingsKeys.contains("channelSampleRate")) {
        m_channelSampleRate = validatedSampleRate(settings.m_channelSampleRate);
    }
    if (settingsKeys.contains("powerLPFCutoff")) {
        m_powerLPFCutoff = settings.m_powerLPFCutoff;
    }
    if (settingsKeys.contains("detectionThresholdDB")) {
        m_detectionThresholdDB = settings.m_detectionThresholdDB;
    }
    if (settingsKeys.contains("minDurationMS")) {
        m_minDurationMS = settings.m_minDurationMS;
    }
    if (settingsKeys.contains("maxDurationMS")) {
        m_maxDurationMS = settings.m_maxDurationMS;
    }
    if (settingsKeys.contains("maxFrequencyDrift")) {
        m_maxFrequencyDrift = settings.m_maxFrequencyDrift;
    }
    if (settingsKeys.contains("detectionsTableColumnHidden")) {
        m_detectionsTableColumnHidden = settings.m_detectionsTableColumnHidden;
    }
    if (settingsKeys.contains("detectionBoxPaddingPixels")) {
        m_detectionBoxPaddingPixels = settings.m_detectionBoxPaddingPixels;
    }
    if (settingsKeys.contains("detectionLabelMode")) {
        m_detectionLabelMode = settings.m_detectionLabelMode;
    }
    if (settingsKeys.contains("transmitterLatitude")) {
        m_transmitterLatitude = settings.m_transmitterLatitude;
    }
    if (settingsKeys.contains("transmitterLongitude")) {
        m_transmitterLongitude = settings.m_transmitterLongitude;
    }
    if (settingsKeys.contains("antennaAzimuth")) {
        m_antennaAzimuth = settings.m_antennaAzimuth;
    }
    if (settingsKeys.contains("antennaElevation")) {
        m_antennaElevation = settings.m_antennaElevation;
    }
    if (settingsKeys.contains("antennaBeamwidth")) {
        m_antennaBeamwidth = settings.m_antennaBeamwidth;
    }
    if (settingsKeys.contains("rotator")) {
        m_rotator = settings.m_rotator;
    }
    if (settingsKeys.contains("rgbColor")) {
        m_rgbColor = settings.m_rgbColor;
    }
    if (settingsKeys.contains("title")) {
        m_title = settings.m_title;
    }
    if (settingsKeys.contains("streamIndex")) {
        m_streamIndex = settings.m_streamIndex;
    }
    if (settingsKeys.contains("workspaceIndex")) {
        m_workspaceIndex = settings.m_workspaceIndex;
    }
    if (settingsKeys.contains("hidden")) {
        m_hidden = settings.m_hidden;
    }
}

QString MeteorSettings::getDebugString(const QStringList& settingsKeys, bool force) const
{
    std::ostringstream ostr;

    if (settingsKeys.contains("inputFrequencyOffset") || force) {
        ostr << " m_inputFrequencyOffset: " << m_inputFrequencyOffset;
    }
    if (settingsKeys.contains("frequencyMode") || force) {
        ostr << " m_frequencyMode: " << (int) m_frequencyMode;
    }
    if (settingsKeys.contains("frequency") || force) {
        ostr << " m_frequency: " << m_frequency;
    }
    if (settingsKeys.contains("channelSampleRate") || force) {
        ostr << " m_channelSampleRate: " << m_channelSampleRate;
    }
    if (settingsKeys.contains("powerLPFCutoff") || force) {
        ostr << " m_powerLPFCutoff: " << m_powerLPFCutoff;
    }
    if (settingsKeys.contains("detectionThresholdDB") || force) {
        ostr << " m_detectionThresholdDB: " << m_detectionThresholdDB;
    }
    if (settingsKeys.contains("minDurationMS") || force) {
        ostr << " m_minDurationMS: " << m_minDurationMS;
    }
    if (settingsKeys.contains("maxDurationMS") || force) {
        ostr << " m_maxDurationMS: " << m_maxDurationMS;
    }
    if (settingsKeys.contains("maxFrequencyDrift") || force) {
        ostr << " m_maxFrequencyDrift: " << m_maxFrequencyDrift;
    }
    if (settingsKeys.contains("detectionsTableColumnHidden") || force) {
        ostr << " m_detectionsTableColumnHidden: " << m_detectionsTableColumnHidden;
    }
    if (settingsKeys.contains("detectionBoxPaddingPixels") || force) {
        ostr << " m_detectionBoxPaddingPixels: " << m_detectionBoxPaddingPixels;
    }
    if (settingsKeys.contains("detectionLabelMode") || force) {
        ostr << " m_detectionLabelMode: " << (int) m_detectionLabelMode;
    }
    if (settingsKeys.contains("transmitterLatitude") || force) {
        ostr << " m_transmitterLatitude: " << m_transmitterLatitude;
    }
    if (settingsKeys.contains("transmitterLongitude") || force) {
        ostr << " m_transmitterLongitude: " << m_transmitterLongitude;
    }
    if (settingsKeys.contains("antennaAzimuth") || force) {
        ostr << " m_antennaAzimuth: " << m_antennaAzimuth;
    }
    if (settingsKeys.contains("antennaElevation") || force) {
        ostr << " m_antennaElevation: " << m_antennaElevation;
    }
    if (settingsKeys.contains("antennaBeamwidth") || force) {
        ostr << " m_antennaBeamwidth: " << m_antennaBeamwidth;
    }
    if (settingsKeys.contains("rotator") || force) {
        ostr << " m_rotator: " << m_rotator.toStdString();
    }
    if (settingsKeys.contains("streamIndex") || force) {
        ostr << " m_streamIndex: " << m_streamIndex;
    }

    return QString(ostr.str().c_str());
}
