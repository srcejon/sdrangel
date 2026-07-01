///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Tom Hensel <code@jitter.eu>                                //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// MeshCore wire-packet decoder implementation.                                  //
///////////////////////////////////////////////////////////////////////////////////

#include "meshcore_decoder.h"

#include "meshcore_builders.h"
#include "meshcore_crypto.h"

#include <QStringList>
#include <QtEndian>

namespace modemmeshcore
{
namespace decoder
{

namespace
{

void addField(DecodeResult& out, const QString& path, const QString& value)
{
    DecodeResult::Field f;
    f.path = path;
    f.value = value;
    out.fields.append(f);
}

void addField(DecodeResult& out, const QString& path, int v)
{
    addField(out, path, QString::number(v));
}

QString hexOf(const QByteArray& bytes) { return QString::fromLatin1(bytes.toHex()); }

uint32_t readU32LE(const QByteArray& wire, int off)
{
    if (off + 4 > wire.size()) {
        return 0;
    }
    return qFromLittleEndian<uint32_t>(reinterpret_cast<const uchar*>(wire.constData() + off));
}

int32_t readI32LE(const QByteArray& wire, int off)
{
    if (off + 4 > wire.size()) {
        return 0;
    }
    return qFromLittleEndian<int32_t>(reinterpret_cast<const uchar*>(wire.constData() + off));
}

// txt_type/attempt is upper 6 bits / lower 2 bits of one byte; same layout
// for TXT_MSG and GRP_TXT.
QString txtTypeName(uint8_t txtType)
{
    switch (txtType) {
        case 0x00: return QStringLiteral("plain");
        case 0x01: return QStringLiteral("cli");
        case 0x02: return QStringLiteral("signed");
        default:   return QString::number(txtType);
    }
}

// Common decrypt + extract for TXT_MSG / GRP_TXT plaintext layout.
//   ts(4 LE) | txt_type<<2|attempt(1) | text || pad
bool extractTxtPlaintext(const QByteArray& plaintext,
                         uint32_t& timestamp, uint8_t& txtType,
                         uint8_t& attempt, QString& text)
{
    if (plaintext.size() < 5) {
        return false;
    }
    timestamp = qFromLittleEndian<uint32_t>(reinterpret_cast<const uchar*>(plaintext.constData()));
    const uint8_t flags = static_cast<uint8_t>(plaintext[4]);
    txtType = static_cast<uint8_t>(flags >> 2);
    attempt = static_cast<uint8_t>(flags & 0x03);

    QByteArray body = plaintext.mid(5);
    // Strip trailing zero padding from the encryption block alignment.
    while (!body.isEmpty() && body.endsWith('\0')) {
        body.chop(1);
    }
    text = QString::fromUtf8(body);
    return true;
}

} // namespace

// ---- Header parser ------------------------------------------------------------

bool parseHeader(const QByteArray& wire, ParsedHeader& hdr)
{
    if (wire.size() < 2) {
        return false;
    }
    const uint8_t b = static_cast<uint8_t>(wire[0]);
    hdr.routeType   = b & kRouteMask;
    hdr.payloadType = (b >> kPayloadTypeShift) & kPayloadTypeMask;
    hdr.version     = (b >> kVersionShift) & kVersionMask;
    hdr.hasTransport = (hdr.routeType == RouteTFlood) || (hdr.routeType == RouteTDirect);

    int off = 1;
    if (hdr.hasTransport) {
        if (off + 4 > wire.size()) {
            return false;
        }
        off += 4;
    }
    if (off >= wire.size()) {
        return false;
    }
    hdr.pathLenByte = static_cast<uint8_t>(wire[off]);
    const PathLen p = decodePathLen(static_cast<uint8_t>(hdr.pathLenByte));
    off += 1 + p.totalBytes;
    if (off > wire.size()) {
        return false;
    }
    hdr.payloadOffset = off;
    return true;
}

// ---- Key spec list ------------------------------------------------------------

bool parseKeySpecList(const QString& spec, QVector<KeySpec>& outKeys,
                      QString& error, bool validateOnly)
{
    outKeys.clear();
    error.clear();

    const QStringList tokens = spec.split(';', Qt::SkipEmptyParts);
    for (const QString& raw : tokens)
    {
        const QString tok = raw.trimmed();
        if (tok.isEmpty()) {
            continue;
        }
        const int eq = tok.indexOf('=');
        if (eq <= 0) {
            error = QString("malformed key entry: '%1' (expected key=value)").arg(tok);
            return false;
        }
        const QString lhs = tok.left(eq).trimmed();
        const QString rhs = tok.mid(eq + 1).trimmed();

        const int colon = lhs.indexOf(':');
        QString prefix = (colon < 0) ? lhs.toLower() : lhs.left(colon).trimmed().toLower();
        QString name   = (colon < 0) ? QString()      : lhs.mid(colon + 1).trimmed();

        KeySpec ks;
        ks.name = name;

        // 'channel:public' is a special-case alias for the public PSK.
        if (prefix == "channel" && rhs.compare("public", Qt::CaseInsensitive) == 0) {
            ks.kind = KeySpec::Channel;
            if (!validateOnly) {
                ks.data = builders::publicChannelPsk();
                ks.name = name.isEmpty() ? QStringLiteral("public") : name;
            }
            outKeys.append(ks);
            continue;
        }

        QByteArray bytes = QByteArray::fromHex(rhs.toLatin1());
        if (bytes.isEmpty()) {
            error = QString("invalid hex value for '%1'").arg(lhs);
            return false;
        }

        if (prefix == "identity") {
            if (bytes.size() != kPubKeySize) {
                error = QString("identity must be %1 hex bytes, got %2")
                            .arg(kPubKeySize).arg(bytes.size());
                return false;
            }
            ks.kind = KeySpec::Identity;
        } else if (prefix == "contact") {
            if (bytes.size() != kPubKeySize) {
                error = QString("contact:%1 must be %2 hex bytes, got %3")
                            .arg(name).arg(kPubKeySize).arg(bytes.size());
                return false;
            }
            if (name.isEmpty()) {
                error = "contact:<name> requires a name";
                return false;
            }
            ks.kind = KeySpec::Contact;
        } else if (prefix == "channel") {
            if (bytes.size() != 16 && bytes.size() != 32) {
                error = QString("channel:%1 must be 16 or 32 hex bytes, got %2")
                            .arg(name).arg(bytes.size());
                return false;
            }
            if (name.isEmpty()) {
                error = "channel:<name> requires a name";
                return false;
            }
            ks.kind = KeySpec::Channel;
        } else {
            error = QString("unknown key prefix: '%1' (expected identity|contact|channel)").arg(prefix);
            return false;
        }

        if (!validateOnly) {
            ks.data = bytes;
        }
        outKeys.append(ks);
    }
    return true;
}

// ---- ADVERT decoder -----------------------------------------------------------

bool decodeAdvert(const QByteArray& wire, const ParsedHeader& hdr, DecodeResult& out)
{
    if (hdr.payloadType != PayloadAdvert) {
        return false;
    }

    const int off = hdr.payloadOffset;
    const int needBase = kPubKeySize + 4 + kSignatureSize + 1; // pub + ts + sig + flags
    if (off + needBase > wire.size()) {
        return false;
    }

    const QByteArray pubKey = wire.mid(off, kPubKeySize);
    const uint32_t   ts     = readU32LE(wire, off + kPubKeySize);
    const QByteArray sig    = wire.mid(off + kPubKeySize + 4, kSignatureSize);

    const int appOff = off + kPubKeySize + 4 + kSignatureSize;
    const uint8_t flags = static_cast<uint8_t>(wire[appOff]);
    QByteArray appData = wire.mid(appOff);

    addField(out, "advert.pubkey", hexOf(pubKey));
    addField(out, "advert.timestamp", QString::number(ts));
    addField(out, "advert.flags", QString("0x%1").arg(flags, 2, 16, QChar('0')));

    int nameOff = appOff + 1;
    if (flags & AdvertHasLocation) {
        if (nameOff + 8 <= wire.size()) {
            const int32_t latI = readI32LE(wire, nameOff);
            const int32_t lonI = readI32LE(wire, nameOff + 4);
            addField(out, "advert.lat", QString::number(latI / 1e6, 'f', 6));
            addField(out, "advert.lon", QString::number(lonI / 1e6, 'f', 6));
        }
        nameOff += 8;
    }
    if (flags & AdvertHasFeature1) { nameOff += 2; }
    if (flags & AdvertHasFeature2) { nameOff += 2; }
    if ((flags & AdvertHasName) && nameOff < wire.size()) {
        QByteArray rawName = wire.mid(nameOff, 32);
        while (!rawName.isEmpty() && rawName.endsWith('\0')) {
            rawName.chop(1);
        }
        addField(out, "advert.name", QString::fromUtf8(rawName));
    }

    // Verify Ed25519 signature: sign target = pubkey || ts || appData
    QByteArray msg;
    msg.reserve(kPubKeySize + 4 + appData.size());
    msg.append(pubKey);
    msg.append(wire.mid(off + kPubKeySize, 4));
    msg.append(appData);
    const bool sigOk = detail::verifyEd25519(sig, pubKey, msg);
    addField(out, "advert.signature_valid", sigOk ? QStringLiteral("true") : QStringLiteral("false"));

    out.dataDecoded = true;
    out.summary = QString("MESHCORE RX|advert pub=%1 ts=%2%3")
                      .arg(hexOf(pubKey).left(16))
                      .arg(ts)
                      .arg(sigOk ? QString() : QStringLiteral(" SIG_INVALID"));
    return true;
}

// ---- ACK decoder --------------------------------------------------------------

bool decodeAck(const QByteArray& wire, const ParsedHeader& hdr, DecodeResult& out)
{
    if (hdr.payloadType != PayloadAck) {
        return false;
    }
    const int off = hdr.payloadOffset;
    if (off + 1 + 4 > wire.size()) {
        return false;
    }
    const QByteArray destHash = wire.mid(off, 1);
    const QByteArray msgHash = wire.mid(off + 1, 4);
    addField(out, "ack.dest_hash", hexOf(destHash));
    addField(out, "ack.msg_hash", hexOf(msgHash));
    out.dataDecoded = true;
    out.summary = QString("MESHCORE RX|ack dest=%1 hash=%2")
                      .arg(hexOf(destHash))
                      .arg(hexOf(msgHash));
    return true;
}

// ---- TXT_MSG decoder ----------------------------------------------------------

bool decodeTxtMsg(const QByteArray& wire, const ParsedHeader& hdr,
                  const QVector<KeySpec>& keys, DecodeResult& out)
{
    if (hdr.payloadType != PayloadTxt) {
        return false;
    }

    const int off = hdr.payloadOffset;
    if (off + 2 + kCipherMacSize + kCipherBlockSize > wire.size()) {
        return false;
    }

    const QByteArray destHash = wire.mid(off, 1);
    const QByteArray srcHash = wire.mid(off + 1, 1);
    const QByteArray macThenCt = wire.mid(off + 2);

    addField(out, "txt.dest_hash", hexOf(destHash));
    addField(out, "txt.src_hash", hexOf(srcHash));

    out.dataDecoded = true;
    out.summary = QString("MESHCORE RX|txt_msg dest=%1 src=%2")
                      .arg(hexOf(destHash))
                      .arg(hexOf(srcHash));

    // Find our identity (if any) — required for ECDH trial-decryption.
    QByteArray ourSeed;
    for (const KeySpec& k : keys) {
        if (k.kind == KeySpec::Identity) { ourSeed = k.data; break; }
    }
    if (ourSeed.isEmpty()) {
        return true; // Without identity we can only show the envelope.
    }

    // Trial-decrypt against every known contact pubkey.
    for (const KeySpec& k : keys) {
        if (k.kind != KeySpec::Contact) {
            continue;
        }
        const QByteArray secret = detail::sharedSecret(ourSeed, k.data);
        if (secret.size() != kPubKeySize) {
            continue;
        }
        const QByteArray pt = detail::macThenDecrypt(secret, macThenCt);
        if (pt.isEmpty()) {
            continue;
        }
        uint32_t ts; uint8_t txtType, attempt; QString text;
        if (!extractTxtPlaintext(pt, ts, txtType, attempt, text)) {
            continue;
        }
        addField(out, "txt.timestamp", QString::number(ts));
        addField(out, "txt.txt_type", txtTypeName(txtType));
        addField(out, "txt.attempt", attempt);
        addField(out, "txt.text", text);
        addField(out, "txt.sender_pubkey", hexOf(k.data));
        addField(out, "txt.sender_name", k.name);
        out.decrypted = true;
        out.keyLabel = QString("contact:%1").arg(k.name);
        out.summary = QString("MESHCORE RX|txt_msg from=%1 ts=%2 text=\"%3\"")
                          .arg(k.name).arg(ts).arg(text);
        return true;
    }

    return true; // envelope decoded, decryption failed (no matching contact)
}

// ---- GRP_TXT decoder ----------------------------------------------------------

bool decodeGrpTxt(const QByteArray& wire, const ParsedHeader& hdr,
                  const QVector<KeySpec>& keys, DecodeResult& out)
{
    if (hdr.payloadType != PayloadGrpTxt) {
        return false;
    }

    const int off = hdr.payloadOffset;
    if (off + 1 + kCipherMacSize + kCipherBlockSize > wire.size()) {
        return false;
    }

    const QByteArray channelHash = wire.mid(off, 1);
    const QByteArray macThenCt = wire.mid(off + 1);

    addField(out, "grp.channel_hash", hexOf(channelHash));
    out.dataDecoded = true;
    out.summary = QString("MESHCORE RX|grp_txt hash=%1").arg(hexOf(channelHash));

    // Trial-decrypt against every channel PSK.
    for (const KeySpec& k : keys) {
        if (k.kind != KeySpec::Channel) {
            continue;
        }
        const builders::GroupChannel gc = builders::GroupChannel::fromPsk(k.name, k.data);
        if (!gc.isValid() || gc.hash != channelHash) {
            continue;
        }
        const QByteArray pt = detail::macThenDecrypt(gc.secret, macThenCt);
        if (pt.isEmpty()) {
            continue;
        }
        uint32_t ts; uint8_t txtType, attempt; QString text;
        if (!extractTxtPlaintext(pt, ts, txtType, attempt, text)) {
            continue;
        }
        addField(out, "grp.channel", k.name);
        addField(out, "grp.timestamp", QString::number(ts));
        addField(out, "grp.txt_type", txtTypeName(txtType));
        addField(out, "grp.attempt", attempt);
        addField(out, "grp.text", text);
        out.decrypted = true;
        out.keyLabel = QString("channel:%1").arg(k.name);
        out.summary = QString("MESHCORE RX|grp_txt channel=%1 ts=%2 text=\"%3\"")
                          .arg(k.name).arg(ts).arg(text);
        return true;
    }

    return true;
}

// ---- ANON_REQ decoder ---------------------------------------------------------

bool decodeAnonReq(const QByteArray& wire, const ParsedHeader& hdr,
                   const QVector<KeySpec>& keys, DecodeResult& out)
{
    if (hdr.payloadType != PayloadAnonReq) {
        return false;
    }
    const int off = hdr.payloadOffset;
    if (off + 1 + kPubKeySize + kCipherMacSize + kCipherBlockSize > wire.size()) {
        return false;
    }

    const QByteArray destHash = wire.mid(off, 1);
    const QByteArray senderPub = wire.mid(off + 1, kPubKeySize);
    const QByteArray macThenCt = wire.mid(off + 1 + kPubKeySize);

    addField(out, "anon.dest_hash", hexOf(destHash));
    addField(out, "anon.sender_pubkey", hexOf(senderPub));

    out.dataDecoded = true;
    out.summary = QString("MESHCORE RX|anon_req dest=%1 sender=%2")
                      .arg(hexOf(destHash))
                      .arg(hexOf(senderPub).left(16));

    // Need our identity to derive the ECDH secret.
    QByteArray ourSeed;
    for (const KeySpec& k : keys) {
        if (k.kind == KeySpec::Identity) { ourSeed = k.data; break; }
    }
    if (ourSeed.isEmpty()) {
        return true;
    }

    const QByteArray secret = detail::sharedSecret(ourSeed, senderPub);
    if (secret.size() != kPubKeySize) {
        return true;
    }
    const QByteArray pt = detail::macThenDecrypt(secret, macThenCt);
    if (pt.isEmpty()) {
        return true;
    }
    if (pt.size() < 4) {
        return true;
    }

    const uint32_t ts = qFromLittleEndian<uint32_t>(reinterpret_cast<const uchar*>(pt.constData()));
    QByteArray data = pt.mid(4);
    while (!data.isEmpty() && data.endsWith('\0')) {
        data.chop(1);
    }

    addField(out, "anon.timestamp", QString::number(ts));
    addField(out, "anon.data_hex", hexOf(data));
    // Show as text if it parses as UTF-8 cleanly
    addField(out, "anon.text", QString::fromUtf8(data));
    out.decrypted = true;
    out.summary = QString("MESHCORE RX|anon_req from=%1 ts=%2 len=%3")
                      .arg(hexOf(senderPub).left(16))
                      .arg(ts)
                      .arg(data.size());
    return true;
}

} // namespace decoder
} // namespace modemmeshcore
