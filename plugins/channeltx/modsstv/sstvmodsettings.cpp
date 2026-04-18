///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by Copilot / Claude Sonnet                                          //
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

#include <sstream>
#include <QColor>
#include <QDebug>

#include "util/simpleserializer.h"
#include "settings/serializable.h"
#include "sstvmodsettings.h"

SSTVModSettings::SSTVModSettings() :
    m_channelMarker(nullptr),
    m_rollupState(nullptr),
    m_spectrumGUI(nullptr),
    m_scopeGUI(nullptr)
{
    resetToDefaults();
}

void SSTVModSettings::resetToDefaults()
{
    m_inputFrequencyOffset = 0;
    m_rfBandwidth = 16000.0f;
    m_fmDeviation = 5000.0f;
    m_modulation = ModulationFM;
    m_imagePath = QString();
    m_repeat = false;
    m_rgbColor = QColor(255, 0, 0).rgb();
    m_title = "SSTV Modulator";
    m_streamIndex = 0;
    m_useReverseAPI = false;
    m_reverseAPIAddress = "127.0.0.1";
    m_reverseAPIPort = 8888;
    m_reverseAPIDeviceIndex = 0;
    m_reverseAPIChannelIndex = 0;
    m_workspaceIndex = 0;
    m_hidden = false;
}

QByteArray SSTVModSettings::serialize() const
{
    SimpleSerializer s(1);

    s.writeS32(1, m_inputFrequencyOffset);
    s.writeFloat(2, m_rfBandwidth);
    s.writeFloat(3, m_fmDeviation);
    s.writeS32(4, (int) m_modulation);
    s.writeString(5, m_imagePath);
    s.writeBool(21, m_repeat);
    s.writeU32(6, m_rgbColor);
    s.writeString(7, m_title);
    s.writeS32(8, m_streamIndex);
    s.writeBool(9, m_useReverseAPI);
    s.writeString(10, m_reverseAPIAddress);
    s.writeU32(11, m_reverseAPIPort);
    s.writeU32(12, m_reverseAPIDeviceIndex);
    s.writeU32(13, m_reverseAPIChannelIndex);
    s.writeS32(14, m_workspaceIndex);
    s.writeBlob(15, m_geometryBytes);
    s.writeBool(16, m_hidden);

    if (m_channelMarker) {
        s.writeBlob(17, m_channelMarker->serialize());
    }
    if (m_rollupState) {
        s.writeBlob(18, m_rollupState->serialize());
    }
    if (m_spectrumGUI) {
        s.writeBlob(19, m_spectrumGUI->serialize());
    }
    if (m_scopeGUI) {
        s.writeBlob(20, m_scopeGUI->serialize());
    }

    return s.final();
}

bool SSTVModSettings::deserialize(const QByteArray& data)
{
    SimpleDeserializer d(data);

    if (!d.isValid()) {
        resetToDefaults();
        return false;
    }

    if (d.getVersion() == 1)
    {
        QByteArray bytetmp;
        uint32_t utmp;
        int tmp;

        d.readS32(1, &tmp, 0);
        m_inputFrequencyOffset = tmp;
        d.readFloat(2, &m_rfBandwidth, 16000.0f);
        d.readFloat(3, &m_fmDeviation, 5000.0f);
        d.readS32(4, &tmp, 0);
        m_modulation = (Modulation) tmp;
        d.readString(5, &m_imagePath, QString());
        d.readBool(21, &m_repeat, false);
        d.readU32(6, &m_rgbColor, QColor(255, 0, 0).rgb());
        d.readString(7, &m_title, "SSTV Modulator");
        d.readS32(8, &m_streamIndex, 0);
        d.readBool(9, &m_useReverseAPI, false);
        d.readString(10, &m_reverseAPIAddress, "127.0.0.1");
        d.readU32(11, &utmp, 8888);
        m_reverseAPIPort = utmp & 0xFFFF;
        d.readU32(12, &utmp, 0);
        m_reverseAPIDeviceIndex = utmp & 0xFFFF;
        d.readU32(13, &utmp, 0);
        m_reverseAPIChannelIndex = utmp & 0xFFFF;
        d.readS32(14, &m_workspaceIndex, 0);
        d.readBlob(15, &m_geometryBytes);
        d.readBool(16, &m_hidden, false);

        if (m_channelMarker)
        {
            d.readBlob(17, &bytetmp);
            m_channelMarker->deserialize(bytetmp);
        }
        if (m_rollupState)
        {
            d.readBlob(18, &bytetmp);
            m_rollupState->deserialize(bytetmp);
        }
        if (m_spectrumGUI)
        {
            d.readBlob(19, &bytetmp);
            m_spectrumGUI->deserialize(bytetmp);
        }
        if (m_scopeGUI)
        {
            d.readBlob(20, &bytetmp);
            m_scopeGUI->deserialize(bytetmp);
        }

        return true;
    }
    else
    {
        resetToDefaults();
        return false;
    }
}

void SSTVModSettings::applySettings(const QStringList& settingsKeys, const SSTVModSettings& settings)
{
    if (settingsKeys.contains("inputFrequencyOffset")) {
        m_inputFrequencyOffset = settings.m_inputFrequencyOffset;
    }
    if (settingsKeys.contains("rfBandwidth")) {
        m_rfBandwidth = settings.m_rfBandwidth;
    }
    if (settingsKeys.contains("fmDeviation")) {
        m_fmDeviation = settings.m_fmDeviation;
    }
    if (settingsKeys.contains("modulation")) {
        m_modulation = settings.m_modulation;
    }
    if (settingsKeys.contains("imagePath")) {
        m_imagePath = settings.m_imagePath;
    }
    if (settingsKeys.contains("repeat")) {
        m_repeat = settings.m_repeat;
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
    if (settingsKeys.contains("useReverseAPI")) {
        m_useReverseAPI = settings.m_useReverseAPI;
    }
    if (settingsKeys.contains("reverseAPIAddress")) {
        m_reverseAPIAddress = settings.m_reverseAPIAddress;
    }
    if (settingsKeys.contains("reverseAPIPort")) {
        m_reverseAPIPort = settings.m_reverseAPIPort;
    }
    if (settingsKeys.contains("reverseAPIDeviceIndex")) {
        m_reverseAPIDeviceIndex = settings.m_reverseAPIDeviceIndex;
    }
    if (settingsKeys.contains("reverseAPIChannelIndex")) {
        m_reverseAPIChannelIndex = settings.m_reverseAPIChannelIndex;
    }
    if (settingsKeys.contains("workspaceIndex")) {
        m_workspaceIndex = settings.m_workspaceIndex;
    }
    if (settingsKeys.contains("geometryBytes")) {
        m_geometryBytes = settings.m_geometryBytes;
    }
    if (settingsKeys.contains("hidden")) {
        m_hidden = settings.m_hidden;
    }
}

QString SSTVModSettings::getDebugString(const QStringList& settingsKeys, bool force) const
{
    std::ostringstream ostr;

    if (settingsKeys.contains("inputFrequencyOffset") || force) {
        ostr << " m_inputFrequencyOffset: " << m_inputFrequencyOffset;
    }
    if (settingsKeys.contains("rfBandwidth") || force) {
        ostr << " m_rfBandwidth: " << m_rfBandwidth;
    }
    if (settingsKeys.contains("fmDeviation") || force) {
        ostr << " m_fmDeviation: " << m_fmDeviation;
    }
    if (settingsKeys.contains("modulation") || force) {
        ostr << " m_modulation: " << m_modulation;
    }
    if (settingsKeys.contains("imagePath") || force) {
        ostr << " m_imagePath: " << m_imagePath.toStdString();
    }
    if (settingsKeys.contains("repeat") || force) {
        ostr << " m_repeat: " << m_repeat;
    }
    if (settingsKeys.contains("title") || force) {
        ostr << " m_title: " << m_title.toStdString();
    }

    return QString(ostr.str().c_str());
}
