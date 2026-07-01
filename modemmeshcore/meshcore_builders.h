///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Tom Hensel <code@jitter.eu>                                //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// MeshCore wire-packet builders. Each function returns a complete OTA packet    //
// ready to be handed to the LoRa PHY encoder.                                   //
//                                                                               //
// Wire envelope (build_wire_packet in gr4-lora/meshcore_tx.py):                 //
//   [header(1)] [opt transport(4)] [path_len(1)] [path(N)] [payload(M)]         //
///////////////////////////////////////////////////////////////////////////////////

#ifndef MODEMMESHCORE_MESHCORE_BUILDERS_H_
#define MODEMMESHCORE_MESHCORE_BUILDERS_H_

#include <QByteArray>
#include <QString>

#include <optional>
#include <utility>

#include "export.h"

namespace modemmeshcore
{
namespace builders
{

// ---- header byte --------------------------------------------------------------

// Build the MeshCore header byte: version[7:6] | payload_type[5:2] | route[1:0]
MODEMMESHCORE_API uint8_t makeHeader(uint8_t routeType, uint8_t payloadType, uint8_t version = 0);

// ---- wire envelope ------------------------------------------------------------

struct WireOptions
{
    QByteArray path;                                          // 0..63*hashSize bytes
    int pathHashSize = 1;                                     // 1, 2, or 3
    std::optional<std::pair<uint16_t, uint16_t>> transport;   // 4-byte transport block
};

// Compose the full OTA packet from header byte + per-payload-type body.
MODEMMESHCORE_API QByteArray buildWirePacket(
    uint8_t header,
    const QByteArray& payload,
    const WireOptions& opts = WireOptions{});

// ---- ADVERT -------------------------------------------------------------------

struct AdvertOptions
{
    uint8_t nodeType = 0x01;                  // AdvertNodeChat
    QString name;                             // empty -> no name flag
    std::optional<std::pair<double, double>> latLon;  // (lat, lon) in degrees
    uint32_t timestamp = 0;                   // 0 -> caller must inject; 0 stays as-is
    uint8_t routeType = 0x01;                 // RouteFlood (default per MeshCore convention)
    std::optional<std::pair<uint16_t, uint16_t>> transport;
};

// Build a signed ADVERT wire packet from a 32-byte Ed25519 seed.
//
// Body layout: [pubkey(32)][timestamp(4 LE)][signature(64)][app_data]
//   app_data = [flags(1)][opt lat(4 LE)+lon(4 LE)][opt name(UTF-8)]
//   signature is over: pubkey || timestamp || app_data
//
// Returns the OTA-ready packet, empty on error.
MODEMMESHCORE_API QByteArray buildAdvert(
    const QByteArray& seed,
    const QByteArray& pubKey,
    const AdvertOptions& opts);

// ---- TXT_MSG (encrypted to a known contact via ECDH) --------------------------

enum TxtType : uint8_t
{
    TxtTypePlain  = 0x00,
    TxtTypeCli    = 0x01,
    TxtTypeSigned = 0x02,
};

struct TxtMsgOptions
{
    uint8_t txtType = TxtTypePlain;     // upper 6 bits of txt_type_attempt byte
    uint8_t attempt = 0;                // lower 2 bits (0..3)
    uint32_t timestamp = 0;
    uint8_t routeType = 0x02;           // RouteDirect default for known contacts
    std::optional<std::pair<uint16_t, uint16_t>> transport;
};

// Encrypted text message to a known contact identified by destPub32.
//
// Body: [dest_hash(1)] [src_hash(1)] [MAC(2)] [AES-128-ECB ciphertext]
//   plaintext: [ts(4 LE)] [txt_type<<2 | attempt(1)] [text bytes]
//
// dest_hash = destPub32[0:1], src_hash = derivePubKey(seed)[0:1].
MODEMMESHCORE_API QByteArray buildTxtMsg(
    const QByteArray& seed,
    const QByteArray& destPub32,
    const QByteArray& text,
    const TxtMsgOptions& opts);

// ---- ANON_REQ (encrypted with sender pubkey in cleartext) ---------------------

struct AnonReqOptions
{
    uint32_t timestamp = 0;
    uint8_t routeType = 0x02;           // RouteDirect default
    std::optional<std::pair<uint16_t, uint16_t>> transport;
};

// Body: [dest_hash(1)] [sender_pub(32)] [MAC(2)] [AES-128-ECB ciphertext]
//   plaintext: [ts(4 LE)] [data]
MODEMMESHCORE_API QByteArray buildAnonReq(
    const QByteArray& seed,
    const QByteArray& destPub32,
    const QByteArray& data,
    const AnonReqOptions& opts);

// ---- GRP_TXT (group channel pre-shared key) -----------------------------------

// A MeshCore group channel: 16- or 32-byte PSK identified by a 1-byte hash.
struct MODEMMESHCORE_API GroupChannel
{
    QString name;        // human label
    QByteArray pskRaw;   // 16 or 32 raw bytes
    QByteArray secret;   // pskRaw zero-padded to 32 bytes (HMAC key)
    QByteArray hash;     // SHA-256(pskRaw)[0:1]

    static GroupChannel fromPsk(const QString& name, const QByteArray& pskRaw);
    bool isValid() const { return !pskRaw.isEmpty() && hash.size() == 1; }
};

struct GrpTxtOptions
{
    uint8_t txtType = TxtTypePlain;
    uint8_t attempt = 0;
    uint32_t timestamp = 0;
    uint8_t routeType = 0x01;           // RouteFlood default for group channels
    std::optional<std::pair<uint16_t, uint16_t>> transport;
};

// Body: [channel_hash(1)] [MAC(2)] [AES-128-ECB ciphertext]
//   plaintext: [ts(4 LE)] [txt_type<<2 | attempt(1)] [text bytes]
//
// MeshCore firmware convention: text is formatted as "SenderName: message".
MODEMMESHCORE_API QByteArray buildGrpTxt(
    const GroupChannel& channel,
    const QByteArray& text,
    const GrpTxtOptions& opts);

// ---- ACK (control plaintext, no encryption) -----------------------------------

struct AckOptions
{
    uint8_t routeType = 0x02;           // RouteDirect default
    std::optional<std::pair<uint16_t, uint16_t>> transport;
};

// Body: [dest_hash(1)] [msg_hash(4)]
MODEMMESHCORE_API QByteArray buildAck(
    const QByteArray& destPub32,
    const QByteArray& msgHash4,
    const AckOptions& opts);

// ---- Public-channel helper (fixed PSK convention) -----------------------------

// MeshCore firmware fixes the "public" channel PSK at this 16-byte value and
// always seats it at channel index 0 in firmware/companion-app contact lists.
MODEMMESHCORE_API QByteArray publicChannelPsk();

} // namespace builders
} // namespace modemmeshcore

#endif // MODEMMESHCORE_MESHCORE_BUILDERS_H_
