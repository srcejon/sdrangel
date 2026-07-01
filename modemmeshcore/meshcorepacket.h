///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Tom Hensel <code@jitter.eu>                                //
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

#ifndef MODEMMESHCORE_MESHCOREPACKET_H_
#define MODEMMESHCORE_MESHCOREPACKET_H_

#include <QByteArray>
#include <QProcessEnvironment>
#include <QString>
#include <QtGlobal>
#include <QVector>

#include <stdint.h>

#include "export.h"

namespace modemmeshcore
{

// MeshCore default radio params (EU primary): 869.618 MHz / 62.5 kHz / SF 8 / CR 8
constexpr int kDefaultBandwidthHz = 62500;
constexpr int kDefaultSpreadFactor = 8;
constexpr int kDefaultParityBits = 4;            // 4 -> CR 4/8
constexpr uint8_t kDefaultSyncWord = 0x12;       // MeshCore wire sync (verify against firmware)
constexpr int kDefaultPreambleChirps = 16;       // MeshCore minimum: 16 for SF>8, 32 for SF<9
constexpr qint64 kDefaultCenterFrequencyHz = 869618000LL;

// ---- Crypto sizes (MeshCore wire spec) ----
constexpr int kPubKeySize = 32;        // Ed25519 / X25519 public key
constexpr int kPrvKeySize = 64;        // expanded Ed25519 private key
constexpr int kSignatureSize = 64;     // Ed25519 signature
constexpr int kCipherKeySize = 16;     // AES-128
constexpr int kCipherBlockSize = 16;
constexpr int kCipherMacSize = 2;      // HMAC-SHA256 truncated to 2 bytes

// ---- Header byte (first OTA byte) layout ----
// bits [1:0] route_type (4 values)
// bits [5:2] payload_type (12 named + reserved)
// bits [7:6] version (currently 0)
constexpr uint8_t kRouteMask = 0x03;
constexpr int     kPayloadTypeShift = 2;
constexpr uint8_t kPayloadTypeMask = 0x0F;
constexpr int     kVersionShift = 6;
constexpr uint8_t kVersionMask = 0x03;

enum RouteType : uint8_t
{
    RouteTFlood = 0x00,
    RouteFlood = 0x01,
    RouteDirect = 0x02,
    RouteTDirect = 0x03,
};

enum PayloadType : uint8_t
{
    PayloadReq = 0x00,
    PayloadResp = 0x01,
    PayloadTxt = 0x02,
    PayloadAck = 0x03,
    PayloadAdvert = 0x04,
    PayloadGrpTxt = 0x05,
    PayloadGrpData = 0x06,
    PayloadAnonReq = 0x07,
    PayloadPath = 0x08,
    PayloadTrace = 0x09,
    PayloadMulti = 0x0A,
    PayloadCtrl = 0x0B,
    PayloadRawCustom = 0x0F,
};

// ---- Path packing ----
// path_len byte:
//   bits [7:6] hash size selector: 0=>1B, 1=>2B, 2=>3B, 3=>reserved
//   bits [5:0] hash count (0..63)
// total path bytes on wire = hash_count * hash_size
constexpr uint8_t kPathModeMask  = 0xC0;
constexpr int     kPathModeShift = 6;
constexpr uint8_t kPathCountMask = 0x3F;

struct PathLen
{
    int hashCount;     // number of hop hashes (0..63)
    int hashSize;      // bytes per hash (1, 2, or 3)
    int totalBytes;    // hashCount * hashSize
};

MODEMMESHCORE_API PathLen decodePathLen(uint8_t raw);
MODEMMESHCORE_API uint8_t encodePathLen(int hashCount, int hashSize);

// ---- ADVERT app-data flags ----
enum AdvertFlags : uint8_t
{
    AdvertNodeChat = 0x01,
    AdvertNodeRepeater = 0x02,
    AdvertNodeRoom = 0x03,
    AdvertNodeSensor = 0x04,
    AdvertHasLocation = 0x10,
    AdvertHasFeature1 = 0x20,
    AdvertHasFeature2 = 0x40,
    AdvertHasName = 0x80,
};

struct MODEMMESHCORE_API DecodeResult
{
    struct Field
    {
        QString path;
        QString value;
    };

    bool isFrame = false;
    bool dataDecoded = false;     // any meaningful payload-level decode produced
    bool decrypted = false;
    uint8_t routeType = 0;
    uint8_t payloadType = 0;
    uint8_t payloadVer = 0;
    QString keyLabel;
    QString summary;
    QVector<Field> fields;
};

struct MODEMMESHCORE_API TxRadioSettings
{
    bool hasCommand = false;
    bool hasLoRaParams = false;
    int bandwidthHz = kDefaultBandwidthHz;
    int spreadFactor = kDefaultSpreadFactor;
    int parityBits = kDefaultParityBits;
    int deBits = 0;
    uint8_t syncWord = kDefaultSyncWord;
    int preambleChirps = kDefaultPreambleChirps;

    bool hasCenterFrequency = false;
    qint64 centerFrequencyHz = kDefaultCenterFrequencyHz;

    QString summary;
};

class MODEMMESHCORE_API Packet
{
public:
    static bool isCommand(const QString& text);

    static bool buildFrameFromCommand(
        const QString& command,
        QByteArray& frame,
        QString& summary,
        QString& error
    );

    static bool decodeFrame(
        const QByteArray& frame,
        DecodeResult& result
    );

    static bool decodeFrame(
        const QByteArray& frame,
        DecodeResult& result,
        const QString& keySpecList
    );

    static bool validateKeySpecList(
        const QString& keySpecList,
        QString& error,
        int* keyCount = nullptr
    );

    static QString defaultKeysFromEnv();

    static bool deriveTxRadioSettings(
        const QString& command,
        TxRadioSettings& settings,
        QString& error
    );
};

} // namespace modemmeshcore

#endif // MODEMMESHCORE_MESHCOREPACKET_H_
