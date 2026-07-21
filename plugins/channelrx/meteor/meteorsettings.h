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

#ifndef INCLUDE_METEORSETTINGS_H
#define INCLUDE_METEORSETTINGS_H

#include <array>

#include <QByteArray>
#include <QString>

class Serializable;

struct MeteorSettings
{
    inline static constexpr std::array<int, 4> m_supportedSampleRates = {100, 300, 1000, 3000};
    inline static constexpr int m_defaultSampleRate = 1000;

    static constexpr bool isSupportedSampleRate(int sampleRate)
    {
        for (int supportedSampleRate : m_supportedSampleRates)
        {
            if (sampleRate == supportedSampleRate) {
                return true;
            }
        }

        return false;
    }

    static constexpr int validatedSampleRate(int sampleRate)
    {
        return isSupportedSampleRate(sampleRate) ? sampleRate : m_defaultSampleRate;
    }

    enum FrequencyMode {
        Offset,
        Absolute
    } m_frequencyMode;

    qint32 m_inputFrequencyOffset;
    qint64 m_frequency;
    int m_channelSampleRate;
    float m_powerLPFCutoff;
    float m_detectionThresholdDB;
    int m_minDurationMS;
    int m_maxDurationMS;
    float m_maxFrequencyDrift;
    quint32 m_detectionsTableColumnHidden;
    int m_detectionBoxPaddingPixels;
    enum DetectionLabelMode {
        DetectionLabelNone,
        DetectionLabelTop,
        DetectionLabelRight
    } m_detectionLabelMode;
    double m_transmitterLatitude;
    double m_transmitterLongitude;
    float m_transmitterAzimuth;
    float m_transmitterElevation;
    float m_transmitterBeamwidth;
    float m_antennaAzimuth;
    float m_antennaElevation;
    float m_antennaBeamwidth;
    QString m_rotator; //!< "<featureSetIndex>:<featureIndex>" of the GS232Controller to follow

    quint32 m_rgbColor;
    QString m_title;
    Serializable *m_channelMarker;
    Serializable *m_spectrumGUI;
    Serializable *m_headSpectrumGUI;
    Serializable *m_rollupState;
    int m_streamIndex;
    int m_workspaceIndex;
    QByteArray m_geometryBytes;
    bool m_hidden;

    MeteorSettings();
    void resetToDefaults();
    void setChannelMarker(Serializable *channelMarker) { m_channelMarker = channelMarker; }
    void setSpectrumGUI(Serializable *spectrumGUI) { m_spectrumGUI = spectrumGUI; }
    void setHeadSpectrumGUI(Serializable *spectrumGUI) { m_headSpectrumGUI = spectrumGUI; }
    void setRollupState(Serializable *rollupState) { m_rollupState = rollupState; }
    QByteArray serialize() const;
    bool deserialize(const QByteArray& data);
    void applySettings(const QStringList& settingsKeys, const MeteorSettings& settings);
    QString getDebugString(const QStringList& settingsKeys, bool force=false) const;
};

#endif // INCLUDE_METEORSETTINGS_H
