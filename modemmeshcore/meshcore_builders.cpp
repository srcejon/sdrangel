///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Tom Hensel <code@jitter.eu>                                //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// MeshCore wire-packet builder implementations.                                 //
///////////////////////////////////////////////////////////////////////////////////

#include "meshcore_builders.h"

#include "meshcorepacket.h"
#include "meshcore_crypto.h"

#include <QCryptographicHash>
#include <QtEndian>

#include <cmath>
#include <cstring>

namespace modemmeshcore
{
namespace builders
{

namespace
{

// little-endian uint32 -> 4 bytes
QByteArray u32LE(uint32_t v)
{
    QByteArray out(4, '\0');
    qToLittleEndian(v, reinterpret_cast<uchar*>(out.data()));
    return out;
}

QByteArray i32LE(int32_t v)
{
    QByteArray out(4, '\0');
    qToLittleEndian(v, reinterpret_cast<uchar*>(out.data()));
    return out;
}

QByteArray u16LE(uint16_t v)
{
    QByteArray out(2, '\0');
    qToLittleEndian(v, reinterpret_cast<uchar*>(out.data()));
    return out;
}

} // namespace

uint8_t makeHeader(uint8_t routeType, uint8_t payloadType, uint8_t version)
{
    return static_cast<uint8_t>(
        ((version & kVersionMask) << kVersionShift)
      | ((payloadType & kPayloadTypeMask) << kPayloadTypeShift)
      |  (routeType & kRouteMask));
}

QByteArray buildWirePacket(uint8_t header, const QByteArray& payload, const WireOptions& opts)
{
    if (opts.pathHashSize < 1 || opts.pathHashSize > 3) {
        return QByteArray();
    }
    if (opts.path.size() % opts.pathHashSize != 0) {
        return QByteArray(); // path length must be a multiple of hash size
    }

    const int hashCount = opts.path.size() / opts.pathHashSize;
    if (hashCount > 63) {
        return QByteArray(); // path_count field is 6 bits
    }

    QByteArray out;
    out.reserve(1 + (opts.transport ? 4 : 0) + 1 + opts.path.size() + payload.size());

    // [0] header
    out.append(static_cast<char>(header));

    // [1..4] optional transport_codes (LE u16, u16)
    if (opts.transport) {
        out.append(u16LE(opts.transport->first));
        out.append(u16LE(opts.transport->second));
    }

    // [N] path_len byte (encodes hash size + count)
    out.append(static_cast<char>(encodePathLen(hashCount, opts.pathHashSize)));

    // [N+1 .. N+P] path
    out.append(opts.path);

    // [N+1+P ..] payload
    out.append(payload);

    return out;
}

QByteArray buildAdvert(const QByteArray& seed, const QByteArray& pubKey, const AdvertOptions& opts)
{
    if (seed.size() != kPubKeySize || pubKey.size() != kPubKeySize) {
        return QByteArray();
    }

    // ---- compute flags ----
    uint8_t flags = static_cast<uint8_t>(opts.nodeType & 0x0F);
    if (opts.latLon) {
        flags |= AdvertHasLocation;
    }
    if (!opts.name.isEmpty()) {
        flags |= AdvertHasName;
    }

    // ---- assemble app_data ----
    QByteArray appData;
    appData.reserve(1 + (opts.latLon ? 8 : 0) + opts.name.toUtf8().size());
    appData.append(static_cast<char>(flags));
    if (opts.latLon) {
        const int32_t latI = static_cast<int32_t>(std::lround(opts.latLon->first  * 1e6));
        const int32_t lonI = static_cast<int32_t>(std::lround(opts.latLon->second * 1e6));
        appData.append(i32LE(latI));
        appData.append(i32LE(lonI));
    }
    if (!opts.name.isEmpty()) {
        appData.append(opts.name.toUtf8());
    }

    // ---- sign over (pubkey || ts || app_data) ----
    const QByteArray tsBytes = u32LE(opts.timestamp);
    QByteArray message;
    message.reserve(kPubKeySize + 4 + appData.size());
    message.append(pubKey);
    message.append(tsBytes);
    message.append(appData);

    const QByteArray signature = detail::signEd25519(seed, message);
    if (signature.size() != kSignatureSize) {
        return QByteArray();
    }

    // ---- assemble ADVERT body: [pubkey(32)][ts(4)][signature(64)][app_data] ----
    QByteArray body;
    body.reserve(kPubKeySize + 4 + kSignatureSize + appData.size());
    body.append(pubKey);
    body.append(tsBytes);
    body.append(signature);
    body.append(appData);

    // ---- wrap in wire envelope ----
    WireOptions wireOpts;
    wireOpts.transport = opts.transport;
    return buildWirePacket(
        makeHeader(opts.routeType, PayloadAdvert),
        body,
        wireOpts);
}

// ---- TXT_MSG ------------------------------------------------------------------

QByteArray buildTxtMsg(const QByteArray& seed,
                       const QByteArray& destPub32,
                       const QByteArray& text,
                       const TxtMsgOptions& opts)
{
    if (seed.size() != kPubKeySize || destPub32.size() != kPubKeySize) {
        return QByteArray();
    }

    const QByteArray secret = detail::sharedSecret(seed, destPub32);
    if (secret.size() != kPubKeySize) {
        return QByteArray();
    }

    // plaintext: [ts(4 LE)] [txt_type<<2 | attempt(1)] [text]
    QByteArray plaintext;
    plaintext.reserve(4 + 1 + text.size());
    plaintext.append(u32LE(opts.timestamp));
    plaintext.append(static_cast<char>(((opts.txtType & 0x3F) << 2) | (opts.attempt & 0x03)));
    plaintext.append(text);

    const QByteArray encrypted = detail::encryptThenMac(secret, plaintext);
    if (encrypted.isEmpty()) {
        return QByteArray();
    }

    const QByteArray myPub = detail::derivePubKey(seed);
    if (myPub.size() != kPubKeySize) {
        return QByteArray();
    }

    // body: [dest_hash(1)] [src_hash(1)] [MAC(2) || ciphertext]
    QByteArray body;
    body.reserve(1 + 1 + encrypted.size());
    body.append(destPub32.left(1));
    body.append(myPub.left(1));
    body.append(encrypted);

    WireOptions wireOpts;
    wireOpts.transport = opts.transport;
    return buildWirePacket(makeHeader(opts.routeType, PayloadTxt), body, wireOpts);
}

// ---- ANON_REQ -----------------------------------------------------------------

QByteArray buildAnonReq(const QByteArray& seed,
                        const QByteArray& destPub32,
                        const QByteArray& data,
                        const AnonReqOptions& opts)
{
    if (seed.size() != kPubKeySize || destPub32.size() != kPubKeySize) {
        return QByteArray();
    }

    const QByteArray secret = detail::sharedSecret(seed, destPub32);
    if (secret.size() != kPubKeySize) {
        return QByteArray();
    }

    // plaintext: [ts(4)] [data]
    QByteArray plaintext;
    plaintext.reserve(4 + data.size());
    plaintext.append(u32LE(opts.timestamp));
    plaintext.append(data);

    const QByteArray encrypted = detail::encryptThenMac(secret, plaintext);
    if (encrypted.isEmpty()) {
        return QByteArray();
    }

    const QByteArray myPub = detail::derivePubKey(seed);
    if (myPub.size() != kPubKeySize) {
        return QByteArray();
    }

    // body: [dest_hash(1)] [sender_pub(32)] [MAC(2) || ciphertext]
    QByteArray body;
    body.reserve(1 + kPubKeySize + encrypted.size());
    body.append(destPub32.left(1));
    body.append(myPub);
    body.append(encrypted);

    WireOptions wireOpts;
    wireOpts.transport = opts.transport;
    return buildWirePacket(makeHeader(opts.routeType, PayloadAnonReq), body, wireOpts);
}

// ---- GroupChannel + GRP_TXT ---------------------------------------------------

GroupChannel GroupChannel::fromPsk(const QString& name, const QByteArray& pskRaw)
{
    GroupChannel ch;
    if (pskRaw.size() != 16 && pskRaw.size() != 32) {
        return ch; // invalid -> isValid() == false
    }
    ch.name = name;
    ch.pskRaw = pskRaw;

    // Zero-pad PSK to 32 bytes (used as HMAC key).
    ch.secret = pskRaw;
    if (ch.secret.size() < kPubKeySize) {
        ch.secret.append(kPubKeySize - ch.secret.size(), '\0');
    }

    // Channel hash = SHA-256(pskRaw)[0:1]
    ch.hash = QCryptographicHash::hash(pskRaw, QCryptographicHash::Sha256).left(1);
    return ch;
}

QByteArray buildGrpTxt(const GroupChannel& channel,
                       const QByteArray& text,
                       const GrpTxtOptions& opts)
{
    if (!channel.isValid()) {
        return QByteArray();
    }

    QByteArray plaintext;
    plaintext.reserve(4 + 1 + text.size());
    plaintext.append(u32LE(opts.timestamp));
    plaintext.append(static_cast<char>(((opts.txtType & 0x3F) << 2) | (opts.attempt & 0x03)));
    plaintext.append(text);

    const QByteArray encrypted = detail::encryptThenMac(channel.secret, plaintext);
    if (encrypted.isEmpty()) {
        return QByteArray();
    }

    // body: [channel_hash(1)] [MAC(2) || ciphertext]
    QByteArray body;
    body.reserve(1 + encrypted.size());
    body.append(channel.hash);
    body.append(encrypted);

    WireOptions wireOpts;
    wireOpts.transport = opts.transport;
    return buildWirePacket(makeHeader(opts.routeType, PayloadGrpTxt), body, wireOpts);
}

// ---- ACK ----------------------------------------------------------------------

QByteArray buildAck(const QByteArray& destPub32,
                    const QByteArray& msgHash4,
                    const AckOptions& opts)
{
    if (destPub32.size() != kPubKeySize || msgHash4.size() != 4) {
        return QByteArray();
    }

    QByteArray body;
    body.reserve(1 + 4);
    body.append(destPub32.left(1));
    body.append(msgHash4);

    WireOptions wireOpts;
    wireOpts.transport = opts.transport;
    return buildWirePacket(makeHeader(opts.routeType, PayloadAck), body, wireOpts);
}

// ---- Public channel PSK -------------------------------------------------------

QByteArray publicChannelPsk()
{
    static const QByteArray kPsk = QByteArray::fromHex("8b3387e9c5cdea6ac9e5edbaa115cd72");
    return kPsk;
}

} // namespace builders
} // namespace modemmeshcore
