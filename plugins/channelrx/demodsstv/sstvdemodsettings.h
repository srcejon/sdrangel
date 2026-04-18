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

    /** SSTV mode family — determines the scan-line structure. */
    enum class SSTVModeFamily {
        PD,       //!< PD family: YCbCr, line-pair based, 4 sections per pair
        Robot36,  //!< Robot 36: YCbCr, individual lines, Y + separator + chroma
        Scottie,  //!< Scottie family: RGB, individual lines, G + porch + B + sync + porch + R
        Martin    //!< Martin family: RGB, individual lines, G + B + R (no separator)
    };

    /** SSTV image mode selection. */
    enum class SSTVMode {
        PD50      = 0,
        PD90      = 1,
        PD120     = 2,
        PD160     = 3,
        PD180     = 4,
        PD240     = 5,
        PD290     = 6,
        Robot36   = 7,
        ScottieS1 = 8,
        ScottieS2 = 9,
        ScottieDX = 10,
        MartinM1  = 11,
        MartinM2  = 12
    };

    /** Mode-specific timing and dimension parameters. */
    struct SSTVModeParams {
        SSTVModeFamily family;         //!< Mode family (determines state-machine structure)
        int     width;                 //!< Image width in pixels
        int     height;                //!< Image height in pixels
        int     linesTotal;            //!< PD: line pairs (height/2); others: individual scan lines
        float   syncMs;                //!< Sync pulse duration (ms)
        float   porchMs;               //!< Porch duration after sync (ms)
        float   separatorMs;           //!< Inter-section gap (ms): Robot36 Y↔chroma sep; Scottie G↔B porch; 0 for Martin/PD
        int     chromaWidth;           //!< Chroma pixels per line for Robot36 (width/2 = 160); 0 for others
        float   chromaPixelTimeMs;     //!< Robot36 chroma pixel duration (ms); 0 for others
        float   pixelTimeMs;           //!< Main pixel channel duration per pixel (ms)
        uint8_t visCode;               //!< 7-bit VIS identification code
    };

    /** Return the SSTVModeParams for the given \p mode. */
    static SSTVModeParams getModeParams(SSTVMode mode);

    qint32 m_inputFrequencyOffset;  //!< Frequency offset from device centre (Hz)
    float m_rfBandwidth;            //!< RF pre-filter bandwidth (Hz)
    float m_fmDeviation;            //!< FM deviation used for tone scaling (Hz)
    Modulation m_modulation;        //!< Demodulation type (FM / USB / LSB)
    SSTVMode m_sstvMode;            //!< SSTV image mode
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
