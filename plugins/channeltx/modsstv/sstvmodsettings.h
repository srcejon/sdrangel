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

#ifndef PLUGINS_CHANNELTX_MODSSTV_SSTVMODSETTINGS_H_
#define PLUGINS_CHANNELTX_MODSSTV_SSTVMODSETTINGS_H_

#include <QByteArray>
#include <QString>

class Serializable;

struct SSTVModSettings
{
    /** RF modulation type applied to the SSTV audio sub-carrier. */
    typedef enum
    {
        ModulationFM,   //!< Frequency modulation (VHF/UHF)
        ModulationUSB,  //!< Upper sideband SSB (HF)
        ModulationLSB   //!< Lower sideband SSB (HF)
    } Modulation;

    qint64    m_inputFrequencyOffset;   //!< Channel frequency offset from device centre (Hz)
    float     m_rfBandwidth;            //!< RF bandwidth (Hz)
    float     m_fmDeviation;            //!< FM peak deviation (Hz) – only used when m_modulation == ModulationFM
    Modulation m_modulation;            //!< RF modulation type
    QString   m_imagePath;              //!< Path to the image file to transmit (PNG or JPEG)

    quint32   m_rgbColor;
    QString   m_title;
    int       m_streamIndex;
    bool      m_useReverseAPI;
    QString   m_reverseAPIAddress;
    uint16_t  m_reverseAPIPort;
    uint16_t  m_reverseAPIDeviceIndex;
    uint16_t  m_reverseAPIChannelIndex;
    int       m_workspaceIndex;
    QByteArray m_geometryBytes;
    bool      m_hidden;

    Serializable *m_channelMarker;
    Serializable *m_rollupState;
    Serializable *m_spectrumGUI;
    Serializable *m_scopeGUI;

    SSTVModSettings();
    void resetToDefaults();
    void setChannelMarker(Serializable *channelMarker) { m_channelMarker = channelMarker; }
    void setRollupState(Serializable *rollupState) { m_rollupState = rollupState; }
    void setSpectrumGUI(Serializable *spectrumGUI) { m_spectrumGUI = spectrumGUI; }
    void setScopeGUI(Serializable *scopeGUI) { m_scopeGUI = scopeGUI; }
    QByteArray serialize() const;
    bool deserialize(const QByteArray& data);
    void applySettings(const QStringList& settingsKeys, const SSTVModSettings& settings);
    QString getDebugString(const QStringList& settingsKeys, bool force = false) const;
};

#endif // PLUGINS_CHANNELTX_MODSSTV_SSTVMODSETTINGS_H_
