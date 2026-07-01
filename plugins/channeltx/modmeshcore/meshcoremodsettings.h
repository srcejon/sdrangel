///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012 maintech GmbH, Otto-Hahn-Str. 15, 97204 Hoechberg, Germany //
// written by Christian Daniel                                                   //
// Copyright (C) 2015-2020, 2022 Edouard Griffiths, F4EXB <f4exb06@gmail.com>    //
// Copyright (C) 2021 Jon Beniston, M7RCE <jon@beniston.com>                     //
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

#ifndef PLUGINS_CHANNELTX_MODMESHCORE_MESHCOREMODSETTINGS_H_
#define PLUGINS_CHANNELTX_MODMESHCORE_MESHCOREMODSETTINGS_H_

#include <QByteArray>
#include <QString>

#include <stdint.h>

class Serializable;

struct MeshcoreModSettings
{
    enum CodingScheme
    {
        CodingLoRa,  //!< Standard LoRa
    };

    enum MessageType
    {
        MessageText,        //!< Plain text payload (raw, no MeshCore framing)
        MessageAdvert,      //!< Self-ADVERT — broadcasts our identity pubkey + name + (opt) location
        MessageTxtMsg,      //!< MeshCore TXT_MSG (encrypted to a known contact via ECDH)
        MessageGrpTxt,      //!< MeshCore GRP_TXT (encrypted with a group channel PSK)
        MessageAnonReq,     //!< MeshCore ANON_REQ (encrypted, sender pubkey embedded)
        MessageAck          //!< MeshCore ACK (plaintext)
    };

    int m_inputFrequencyOffset;
    int m_bandwidthIndex;
    int m_spreadFactor;
    int m_deBits;                  //!< Low data rate optimize (DE) bits
    unsigned int m_preambleChirps; //!< Number of preamble chirps
    int m_quietMillis;             //!< Number of milliseconds to pause between transmissions
    int m_nbParityBits;            //!< Hamming parity bits (LoRa)
    static const bool m_hasCRC;    //!< Payload has CRC (LoRa)
    static const bool m_hasHeader; //!< Header present before actual payload (LoRa)
    unsigned char m_syncWord;
    bool m_channelMute;
    static const CodingScheme m_codingScheme;
    MessageType m_messageType;     //!< user-selectable: Text/Advert/TxtMsg/GrpTxt/AnonReq/Ack
    QString m_textMessage;
    QByteArray m_bytesMessage;
    int m_messageRepeat;
    bool m_udpEnabled;
    QString m_udpAddress;
    uint16_t m_udpPort;
    bool m_invertRamps;            //!< Invert chirp ramps vs standard LoRa (up/down/up is standard)
    QString m_meshcoreRegionCode;  //!< Meshcore region code (e.g. "US", "EU_868")
    QString m_meshcorePresetName;  //!< MeshCore regional preset name (e.g. "EU_NARROW")
    int m_meshcoreChannelIndex;    //!< Meshcore channel index (0-based)

    // MeshCore app-layer / identity fields. All optional; resetToDefaults() seats
    // sensible values matching EU_868 + Public channel + auto-generated identity.
    QString m_meshIdentityPath;       //!< override identity file path (empty = AppDataLocation default)
    QString m_meshNodeName;           //!< ADVERT name field; defaults to "SDRangel-<short-pubkey>"
    bool m_meshAdvertLocationEnabled; //!< include lat/lon in ADVERT (default false)
    double m_meshAdvertLat;           //!< ADVERT location latitude (degrees)
    double m_meshAdvertLon;           //!< ADVERT location longitude (degrees)
    int m_meshAdvertIntervalSec;      //!< auto-ADVERT interval seconds (0 = manual only)
    QString m_meshDestPubKeyHex;      //!< destination Ed25519 pubkey (hex64) for TxtMsg/AnonReq/Ack
    QString m_meshGroupChannelName;   //!< channel name for GRP_TXT (default "public")
    QString m_meshGroupChannelPskHex; //!< channel PSK (hex16/32); empty = use built-in public PSK
    QString m_meshAckMsgHashHex;      //!< 4-byte msg hash (hex8) for ACK frames

    uint32_t m_rgbColor;
    QString m_title;
    int m_streamIndex;
    bool m_useReverseAPI;
    QString m_reverseAPIAddress;
    uint16_t m_reverseAPIPort;
    uint16_t m_reverseAPIDeviceIndex;
    uint16_t m_reverseAPIChannelIndex;
    int m_workspaceIndex;
    QByteArray m_geometryBytes;
    bool m_hidden;

    Serializable *m_channelMarker;
    Serializable *m_rollupState;

    static const int bandwidths[];
    static const int nbBandwidths;
    static const int oversampling;

    MeshcoreModSettings();
    void resetToDefaults();
    unsigned int getNbSFDFourths() const; //!< Get the number of SFD period fourths (depends on coding scheme)
    bool hasSyncWord() const;             //!< Only LoRa has a syncword (for the moment)
    void setChannelMarker(Serializable *channelMarker) { m_channelMarker = channelMarker; }
    void setRollupState(Serializable *rollupState) { m_rollupState = rollupState; }
    QByteArray serialize() const;
    bool deserialize(const QByteArray& data);
    void applySettings(const QStringList& settingsKeys, const MeshcoreModSettings& settings);
    QString getDebugString(const QStringList& settingsKeys, bool force=false) const;
};



#endif /* PLUGINS_CHANNELTX_MODMESHCORE_MESHCOREMODSETTINGS_H_ */
