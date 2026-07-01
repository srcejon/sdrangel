///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Tom Hensel <code@jitter.eu>                                //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// MeshCore wire-packet decoder. Inverse of meshcore_builders.                   //
///////////////////////////////////////////////////////////////////////////////////

#ifndef MODEMMESHCORE_MESHCORE_DECODER_H_
#define MODEMMESHCORE_MESHCORE_DECODER_H_

#include <QByteArray>
#include <QString>
#include <QVector>

#include "export.h"
#include "meshcorepacket.h"

namespace modemmeshcore
{
namespace decoder
{

// Parsed envelope header (fields populated from the first 1+optional-4+1+N
// bytes on the wire).
struct ParsedHeader
{
    uint8_t routeType = 0;
    uint8_t payloadType = 0;
    uint8_t version = 0;
    bool hasTransport = false;
    int pathLenByte = 0;          // raw path_len byte
    int payloadOffset = 0;        // byte offset into the wire packet at which
                                  // the payload-type-specific body begins
};

// Returns true on success, false if the wire packet is too short or
// malformed. Populates hdr.
MODEMMESHCORE_API bool parseHeader(const QByteArray& wire, ParsedHeader& hdr);

// Key spec list — the keys an operator has loaded for the RX side. Used by
// Packet::decodeFrame to trial-decrypt incoming encrypted frames.
//
// String format (case-insensitive keys, semicolon-separated entries):
//   identity=<hex64>                   our 32-byte Ed25519 seed
//   contact:<name>=<hex64>             a known peer's 32-byte Ed25519 pubkey
//   channel:<name>=<hex16|hex32|name=public>   a group channel PSK
//
// Example:
//   identity=01..20; contact:alice=ab..cd; channel:public=8b3387e9...cd72
struct KeySpec
{
    enum Kind { Identity, Contact, Channel };
    Kind kind = Identity;
    QString name;        // identifier (empty for Identity, non-empty for Contact/Channel)
    QByteArray data;     // raw bytes
};

// Parse the key spec list. Returns true on success. Sets error on failure.
// If validateOnly, don't store data into outKeys.
MODEMMESHCORE_API bool parseKeySpecList(
    const QString& spec,
    QVector<KeySpec>& outKeys,
    QString& error,
    bool validateOnly = false);

// ---- per-payload decoders -----------------------------------------------------

// ADVERT (no key needed). Populates fields:
//   advert.pubkey  (hex)
//   advert.timestamp
//   advert.flags
//   advert.name (if HasName flag set)
//   advert.lat / advert.lon (if HasLocation flag set)
//   advert.signature_valid (Ed25519 verify)
MODEMMESHCORE_API bool decodeAdvert(const QByteArray& wire, const ParsedHeader& hdr, DecodeResult& out);

// ACK (no key needed). Populates:
//   ack.dest_hash (hex 1 byte)
//   ack.msg_hash  (hex 4 bytes)
MODEMMESHCORE_API bool decodeAck(const QByteArray& wire, const ParsedHeader& hdr, DecodeResult& out);

// TXT_MSG. Trial-decrypts against (identity, contact pubkey) pairs.
//   On success: txt.text, txt.sender_pubkey (hex), txt.timestamp, txt.txt_type
//   Always populates: txt.dest_hash, txt.src_hash
MODEMMESHCORE_API bool decodeTxtMsg(const QByteArray& wire,
                                    const ParsedHeader& hdr,
                                    const QVector<KeySpec>& keys,
                                    DecodeResult& out);

// GRP_TXT. Trial-decrypts against channel PSKs.
//   On success: grp.text, grp.channel, grp.timestamp, grp.txt_type
//   Always: grp.channel_hash
MODEMMESHCORE_API bool decodeGrpTxt(const QByteArray& wire,
                                    const ParsedHeader& hdr,
                                    const QVector<KeySpec>& keys,
                                    DecodeResult& out);

// ANON_REQ. Decrypts using identity + sender pubkey embedded in the packet.
//   On success: anon.text/data, anon.sender_pubkey, anon.timestamp
//   Always: anon.dest_hash
MODEMMESHCORE_API bool decodeAnonReq(const QByteArray& wire,
                                     const ParsedHeader& hdr,
                                     const QVector<KeySpec>& keys,
                                     DecodeResult& out);

} // namespace decoder
} // namespace modemmeshcore

#endif // MODEMMESHCORE_MESHCORE_DECODER_H_
