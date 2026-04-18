///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012 maintech GmbH, Otto-Hahn-Str. 15, 97204 Hoechberg, Germany //
// written by Christian Daniel                                                   //
// Copyright (C) 2015-2019, 2021-2022 Edouard Griffiths, F4EXB <f4exb06@gmail.com> //
// Copyright (C) 2021-2026 Jon Beniston, M7RCE <jon@beniston.com>                //
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

#ifndef INCLUDE_SSTVDEMODSETTINGS_H
#define INCLUDE_SSTVDEMODSETTINGS_H

#include <QByteArray>
#include <QString>

class Serializable;

struct SSTVDemodSettings
{
    /** Demodulation type */
    enum Modulation {
        ModulationFM  = 0,  //!< FM (wideband) demodulation
        ModulationUSB = 1,  //!< Upper sideband (USB) SSB demodulation
        ModulationLSB = 2   //!< Lower sideband (LSB) SSB demodulation
    };

    /** SSTV PD image mode selection. */
    enum class PDMode {
        PD50  = 0,
        PD90  = 1,
        PD120 = 2,
        PD160 = 3,
        PD180 = 4,
        PD240 = 5,
        PD290 = 6
    };

    /** Mode-specific timing and dimension parameters for a PD mode. */
    struct PDModeParams {
        int     width;          //!< Image width in pixels
        int     height;         //!< Image height in pixels (must be even; height/2 = number of line pairs)
        int     linePairs;      //!< Number of scan-line pairs (= height / 2)
        float   pixelTimeMs;    //!< Duration of one pixel scan in milliseconds
        uint8_t visCode;        //!< 7-bit VIS identification code for this mode
    };

    /** Return the PDModeParams for the given \p mode. */
    static PDModeParams getPDModeParams(PDMode mode);

    qint32 m_inputFrequencyOffset;  //!< Frequency offset from device centre (Hz)
    float m_rfBandwidth;            //!< RF pre-filter bandwidth (Hz)
    float m_fmDeviation;            //!< FM deviation used for tone scaling (Hz)
    Modulation m_modulation;        //!< Demodulation type (FM / USB / LSB)
    PDMode m_pdMode;                //!< SSTV PD image mode
    bool m_decodeEnabled;           //!< Enable SSTV image decoding
    bool m_autoSave;                //!< Automatically save received images
    QString m_autoSavePath;         //!< Directory to auto-save images

    quint32 m_rgbColor;
    QString m_title;
    Serializable *m_channelMarker;
    int m_streamIndex;
    bool m_useReverseAPI;
    QString m_reverseAPIAddress;
    uint16_t m_reverseAPIPort;
    uint16_t m_reverseAPIDeviceIndex;
    uint16_t m_reverseAPIChannelIndex;
    Serializable *m_scopeGUI;
    Serializable *m_spectrumGUI;
    Serializable *m_rollupState;
    int m_workspaceIndex;
    QByteArray m_geometryBytes;
    bool m_hidden;

    static const int m_scopeStreams = 5; //!< Number of scope streams: fmDemod, freq, isSyncTone, pllLocked, state

    SSTVDemodSettings();
    void resetToDefaults();
    void setChannelMarker(Serializable *channelMarker) { m_channelMarker = channelMarker; }
    void setRollupState(Serializable *rollupState) { m_rollupState = rollupState; }
    void setScopeGUI(Serializable *scopeGUI) { m_scopeGUI = scopeGUI; }
    void setSpectrumGUI(Serializable *spectrumGUI) { m_spectrumGUI = spectrumGUI; }
    QByteArray serialize() const;
    bool deserialize(const QByteArray& data);
    void applySettings(const QStringList& settingsKeys, const SSTVDemodSettings& settings);
    QString getDebugString(const QStringList& settingsKeys, bool force = false) const;
};

#endif // INCLUDE_SSTVDEMODSETTINGS_H
