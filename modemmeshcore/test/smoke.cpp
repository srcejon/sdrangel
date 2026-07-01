///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Tom Hensel <code@jitter.eu>                                //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// modemmeshcore smoke test. Exercises the crypto + ADVERT builder round-trip.   //
// No QtTest infra yet — this is a standalone executable that aborts on first    //
// inconsistency. Run from build dir: ./modemmeshcore/modemmeshcore_smoke        //
///////////////////////////////////////////////////////////////////////////////////

#include "meshcorepacket.h"
#include "meshcore_crypto.h"
#include "meshcore_builders.h"
#include "meshcore_command.h"
#include "meshcore_decoder.h"

#include <QByteArray>
#include <QMap>

#include <cstdio>
#include <cstdlib>

namespace
{

#define REQUIRE(cond, msg) do { \
        if (!(cond)) { \
            std::fprintf(stderr, "FAIL: %s (line %d): %s\n", msg, __LINE__, #cond); \
            std::abort(); \
        } \
        std::printf("  ok: %s\n", msg); \
    } while (0)

QByteArray fromHex(const char* hex) { return QByteArray::fromHex(QByteArray(hex)); }

void testCryptoRoundtrip()
{
    using namespace modemmeshcore;

    // Deterministic 32-byte seed (test vector).
    const QByteArray seed = fromHex("0102030405060708090a0b0c0d0e0f10111213141516171819"
                                    "1a1b1c1d1e1f20");
    REQUIRE(seed.size() == kPubKeySize, "seed is 32 bytes");

    const QByteArray pub32 = detail::derivePubKey(seed);
    REQUIRE(pub32.size() == kPubKeySize, "pub32 derived from seed is 32 bytes");

    // expandedKey(seed) is MeshCore's 64-byte form (clamp(SHA512(seed)[0..32]) || ..).
    const QByteArray expanded = detail::expandedKey(seed);
    REQUIRE(expanded.size() == kPrvKeySize, "expandedKey is 64 bytes");

    // ECDH to self: computing shared secret with our own pub key should succeed
    // and produce 32 bytes. (Round-trip validation: same secret on both sides.)
    const QByteArray sharedSelf = detail::sharedSecret(seed, pub32);
    REQUIRE(sharedSelf.size() == kPubKeySize, "ECDH self-shared secret is 32B");

    // Encrypt-then-MAC round-trip with the self-shared secret.
    const QByteArray plaintext = QByteArray("Hello MeshCore!");
    const QByteArray encrypted = detail::encryptThenMac(sharedSelf, plaintext);
    REQUIRE(!encrypted.isEmpty(), "encryptThenMac produced output");
    REQUIRE(encrypted.size() >= kCipherMacSize + kCipherBlockSize,
            "encryptThenMac output >= 18 bytes");

    const QByteArray decrypted = detail::macThenDecrypt(sharedSelf, encrypted);
    REQUIRE(!decrypted.isEmpty(), "macThenDecrypt validated MAC");
    // decrypted is zero-padded; the prefix must equal the plaintext
    REQUIRE(decrypted.startsWith(plaintext), "decrypted starts with plaintext");

    // MAC tamper: flip one byte in the ciphertext, decrypt should fail
    QByteArray tampered = encrypted;
    tampered[5] = static_cast<char>(static_cast<uint8_t>(tampered[5]) ^ 0x01);
    const QByteArray badDecrypt = detail::macThenDecrypt(sharedSelf, tampered);
    REQUIRE(badDecrypt.isEmpty(), "tampered MAC rejected");

    // Ed25519 sign / verify
    const QByteArray msg = QByteArray("MeshCore signing test message");
    const QByteArray sig = detail::signEd25519(seed, msg);
    REQUIRE(sig.size() == kSignatureSize, "signature is 64 bytes");
    REQUIRE(detail::verifyEd25519(sig, pub32, msg), "signature verifies under pub32");

    // Tampered message must fail verification
    QByteArray badMsg = msg;
    badMsg[0] = static_cast<char>(static_cast<uint8_t>(badMsg[0]) ^ 0x01);
    REQUIRE(!detail::verifyEd25519(sig, pub32, badMsg), "tampered msg fails verify");
}

void testAdvertBuilder()
{
    using namespace modemmeshcore;

    const QByteArray seed = fromHex("0102030405060708090a0b0c0d0e0f10111213141516171819"
                                    "1a1b1c1d1e1f20");
    const QByteArray pub32 = detail::derivePubKey(seed);

    builders::AdvertOptions opts;
    opts.nodeType = AdvertNodeChat;
    opts.name = QStringLiteral("SDRangelTest");
    opts.timestamp = 1700000000U;
    opts.routeType = RouteFlood;

    const QByteArray packet = builders::buildAdvert(seed, pub32, opts);
    REQUIRE(!packet.isEmpty(), "ADVERT packet built");

    // Minimum size: header(1) + path_len(1) + body(pub32+ts4+sig64+flags1+name)
    const int expectedMin = 1 + 1 + kPubKeySize + 4 + kSignatureSize + 1
                            + opts.name.toUtf8().size();
    REQUIRE(packet.size() == expectedMin, "ADVERT exact size matches spec");

    // Header byte: route=Flood(1), payload=Advert(4), version=0
    const uint8_t header = static_cast<uint8_t>(packet[0]);
    REQUIRE((header & kRouteMask) == RouteFlood, "header route = Flood");
    REQUIRE(((header >> kPayloadTypeShift) & kPayloadTypeMask) == PayloadAdvert,
            "header payload = Advert");

    // path_len byte: empty path -> 0
    REQUIRE(static_cast<uint8_t>(packet[1]) == 0, "path_len = 0 (no path)");

    // pubkey at offset 2 must equal pub32
    REQUIRE(packet.mid(2, kPubKeySize) == pub32, "pubkey at offset 2 matches");
}

void testPathLenEncoding()
{
    using namespace modemmeshcore;

    // 1-byte hash mode, 0 hashes -> 0x00
    REQUIRE(encodePathLen(0, 1) == 0x00, "encode(0,1) == 0x00");
    // 1-byte hash mode, 5 hashes -> 0x05
    REQUIRE(encodePathLen(5, 1) == 0x05, "encode(5,1) == 0x05");
    // 2-byte hash mode, 3 hashes -> 0x40 | 3 = 0x43
    REQUIRE(encodePathLen(3, 2) == 0x43, "encode(3,2) == 0x43");
    // 3-byte hash mode, 1 hash  -> 0x80 | 1 = 0x81
    REQUIRE(encodePathLen(1, 3) == 0x81, "encode(1,3) == 0x81");

    // Round-trip
    const PathLen p1 = decodePathLen(0x05);
    REQUIRE(p1.hashCount == 5 && p1.hashSize == 1 && p1.totalBytes == 5,
            "decode(0x05) = {5,1,5}");
    const PathLen p2 = decodePathLen(0x43);
    REQUIRE(p2.hashCount == 3 && p2.hashSize == 2 && p2.totalBytes == 6,
            "decode(0x43) = {3,2,6}");
    const PathLen p3 = decodePathLen(0x81);
    REQUIRE(p3.hashCount == 1 && p3.hashSize == 3 && p3.totalBytes == 3,
            "decode(0x81) = {1,3,3}");
}

} // namespace

void testEncryptedBuilders()
{
    using namespace modemmeshcore;
    namespace b = modemmeshcore::builders;

    const QByteArray seedA = fromHex("0102030405060708090a0b0c0d0e0f10111213141516171819"
                                     "1a1b1c1d1e1f20");
    const QByteArray seedB = fromHex("a1a2a3a4a5a6a7a8a9aaabacadaeafb0"
                                     "b1b2b3b4b5b6b7b8b9babbbcbdbebfc0");
    const QByteArray pubA = detail::derivePubKey(seedA);
    const QByteArray pubB = detail::derivePubKey(seedB);
    REQUIRE(pubA.size() == kPubKeySize, "pubA derived");
    REQUIRE(pubB.size() == kPubKeySize, "pubB derived");

    // ECDH symmetry: shared(A->B) == shared(B->A)
    const QByteArray sharedAB = detail::sharedSecret(seedA, pubB);
    const QByteArray sharedBA = detail::sharedSecret(seedB, pubA);
    REQUIRE(!sharedAB.isEmpty() && sharedAB == sharedBA, "ECDH symmetric");

    // ---- TXT_MSG: A -> B ----
    {
        b::TxtMsgOptions opts;
        opts.timestamp = 1700000001U;
        opts.routeType = RouteDirect;
        const QByteArray text = QByteArray("Hello from A");
        const QByteArray pkt = b::buildTxtMsg(seedA, pubB, text, opts);
        REQUIRE(!pkt.isEmpty(), "TXT_MSG built");

        // Header byte: route=Direct, payload=Txt
        REQUIRE((static_cast<uint8_t>(pkt[0]) & kRouteMask) == RouteDirect,
                "TXT_MSG route=Direct");
        REQUIRE(((static_cast<uint8_t>(pkt[0]) >> kPayloadTypeShift) & kPayloadTypeMask)
                    == PayloadTxt, "TXT_MSG payload=Txt");
        REQUIRE(static_cast<uint8_t>(pkt[1]) == 0, "TXT_MSG path_len=0");

        // dest_hash and src_hash at offsets 2 and 3
        REQUIRE(static_cast<uint8_t>(pkt[2]) == static_cast<uint8_t>(pubB[0]),
                "TXT_MSG dest_hash = pubB[0]");
        REQUIRE(static_cast<uint8_t>(pkt[3]) == static_cast<uint8_t>(pubA[0]),
                "TXT_MSG src_hash  = pubA[0]");

        // RX-side: B decrypts using shared secret.
        const QByteArray macThenCt = pkt.mid(4);
        const QByteArray plaintext = detail::macThenDecrypt(sharedBA, macThenCt);
        REQUIRE(!plaintext.isEmpty(), "TXT_MSG decrypts under B's secret");
        // plaintext layout: ts(4) || flags(1) || text || pad
        REQUIRE(plaintext.size() >= 5 + text.size(), "TXT_MSG plaintext length OK");
        REQUIRE(plaintext.mid(5, text.size()) == text,
                "TXT_MSG plaintext text matches");
    }

    // ---- ANON_REQ: A -> B ----
    {
        b::AnonReqOptions opts;
        opts.timestamp = 1700000002U;
        const QByteArray data = QByteArray("anon-req payload");
        const QByteArray pkt = b::buildAnonReq(seedA, pubB, data, opts);
        REQUIRE(!pkt.isEmpty(), "ANON_REQ built");

        REQUIRE(((static_cast<uint8_t>(pkt[0]) >> kPayloadTypeShift) & kPayloadTypeMask)
                    == PayloadAnonReq, "ANON_REQ payload=AnonReq");

        // dest_hash at 2, sender_pub at 3..3+32
        REQUIRE(static_cast<uint8_t>(pkt[2]) == static_cast<uint8_t>(pubB[0]),
                "ANON_REQ dest_hash = pubB[0]");
        REQUIRE(pkt.mid(3, kPubKeySize) == pubA, "ANON_REQ sender_pub at offset 3");

        // RX-side decrypt
        const QByteArray macThenCt = pkt.mid(3 + kPubKeySize);
        const QByteArray plaintext = detail::macThenDecrypt(sharedBA, macThenCt);
        REQUIRE(!plaintext.isEmpty(), "ANON_REQ decrypts");
        REQUIRE(plaintext.mid(4, data.size()) == data, "ANON_REQ data matches");
    }

    // ---- GRP_TXT: public channel ----
    {
        const QByteArray psk = b::publicChannelPsk();
        REQUIRE(psk.size() == 16, "public PSK is 16 bytes");
        const b::GroupChannel pub = b::GroupChannel::fromPsk("public", psk);
        REQUIRE(pub.isValid(), "public channel valid");
        REQUIRE(pub.hash.size() == 1, "channel hash is 1 byte");
        REQUIRE(pub.secret.size() == kPubKeySize, "channel secret zero-padded to 32");

        b::GrpTxtOptions opts;
        opts.timestamp = 1700000003U;
        const QByteArray text = QByteArray("Test: public message");
        const QByteArray pkt = b::buildGrpTxt(pub, text, opts);
        REQUIRE(!pkt.isEmpty(), "GRP_TXT built");

        REQUIRE(((static_cast<uint8_t>(pkt[0]) >> kPayloadTypeShift) & kPayloadTypeMask)
                    == PayloadGrpTxt, "GRP_TXT payload=GrpTxt");
        REQUIRE(static_cast<uint8_t>(pkt[2]) == static_cast<uint8_t>(pub.hash[0]),
                "GRP_TXT channel_hash matches");

        // Decrypt with channel secret
        const QByteArray macThenCt = pkt.mid(3);
        const QByteArray plaintext = detail::macThenDecrypt(pub.secret, macThenCt);
        REQUIRE(!plaintext.isEmpty(), "GRP_TXT decrypts under channel secret");
        REQUIRE(plaintext.mid(5, text.size()) == text, "GRP_TXT text matches");
    }

    // ---- ACK ----
    {
        const QByteArray msgHash = fromHex("deadbeef");
        b::AckOptions opts;
        const QByteArray pkt = b::buildAck(pubA, msgHash, opts);
        REQUIRE(!pkt.isEmpty(), "ACK built");
        REQUIRE(((static_cast<uint8_t>(pkt[0]) >> kPayloadTypeShift) & kPayloadTypeMask)
                    == PayloadAck, "ACK payload=Ack");
        REQUIRE(pkt.size() == 1 + 1 + 1 + 4, "ACK size = header + path_len + dest + msg_hash");
        REQUIRE(static_cast<uint8_t>(pkt[2]) == static_cast<uint8_t>(pubA[0]),
                "ACK dest_hash = pubA[0]");
        REQUIRE(pkt.mid(3, 4) == msgHash, "ACK msg_hash matches");
    }
}

void testCommandParser()
{
    using namespace modemmeshcore;

    // ---- isCommand ----
    REQUIRE(Packet::isCommand("MESHCORE: type=advert"), "isCommand uppercase prefix");
    REQUIRE(Packet::isCommand("meshcore:type=ack"), "isCommand lowercase no space");
    REQUIRE(!Packet::isCommand("Hello world"), "isCommand rejects plain text");

    // ---- tokenize ----
    {
        QMap<QString, QString> kv;
        QString err;
        REQUIRE(command::tokenize("a=1; b=hello world; c=0xff", kv, err),
                "tokenize ok");
        REQUIRE(kv.value("a") == "1", "tokenize a=1");
        REQUIRE(kv.value("b") == "hello world", "tokenize whitespace value");
        REQUIRE(kv.value("c") == "0xff", "tokenize hex value");
        REQUIRE(kv.size() == 3, "tokenize 3 entries");
    }
    {
        QMap<QString, QString> kv;
        QString err;
        REQUIRE(!command::tokenize("a=1; bad-token", kv, err),
                "tokenize rejects malformed");
        REQUIRE(!err.isEmpty(), "tokenize sets error");
    }

    // ---- ADVERT via command ----
    const QByteArray seedHex = "0102030405060708090a0b0c0d0e0f10"
                               "111213141516171819""1a1b1c1d1e1f20";
    {
        QString cmd = QStringLiteral("MESHCORE: type=advert; seed=") +
                      QString::fromLatin1(seedHex) +
                      QStringLiteral("; name=NodeA; ts=1700000100");
        QByteArray frame;
        QString summary, err;
        REQUIRE(Packet::buildFrameFromCommand(cmd, frame, summary, err),
                "advert command builds");
        REQUIRE(!frame.isEmpty(), "advert frame non-empty");
        // Header: route=Flood, payload=Advert
        REQUIRE((static_cast<uint8_t>(frame[0]) & kRouteMask) == RouteFlood,
                "advert command default route=Flood");
        REQUIRE(((static_cast<uint8_t>(frame[0]) >> kPayloadTypeShift) & kPayloadTypeMask)
                    == PayloadAdvert,
                "advert command payload=Advert");
        REQUIRE(summary.contains("advert"), "advert summary mentions type");
    }

    // ---- TXT_MSG via command ----
    {
        // Generate a destination identity
        const QByteArray destSeed = QByteArray::fromHex(
            "a1a2a3a4a5a6a7a8a9aaabacadaeafb0"
            "b1b2b3b4b5b6b7b8b9babbbcbdbebfc0");
        const QByteArray destPub = detail::derivePubKey(destSeed);

        QString cmd = QStringLiteral("MESHCORE: type=txt_msg; seed=") +
                      QString::fromLatin1(seedHex) +
                      QStringLiteral("; dest=") +
                      QString::fromLatin1(destPub.toHex()) +
                      QStringLiteral("; text=Hello via parser; ts=1700000200");
        QByteArray frame;
        QString summary, err;
        REQUIRE(Packet::buildFrameFromCommand(cmd, frame, summary, err),
                "txt_msg command builds");
        REQUIRE(((static_cast<uint8_t>(frame[0]) >> kPayloadTypeShift) & kPayloadTypeMask)
                    == PayloadTxt, "txt_msg payload=Txt");
    }

    // ---- GRP_TXT via channel=public ----
    {
        QString cmd = QStringLiteral("MESHCORE: type=grp_txt; channel=public; "
                                     "text=Hello group; ts=1700000300");
        QByteArray frame;
        QString summary, err;
        REQUIRE(Packet::buildFrameFromCommand(cmd, frame, summary, err),
                "grp_txt public builds");
        REQUIRE(summary.contains("grp_txt"), "grp_txt summary");
    }

    // ---- ACK ----
    {
        const QByteArray destSeed = QByteArray::fromHex(
            "a1a2a3a4a5a6a7a8a9aaabacadaeafb0"
            "b1b2b3b4b5b6b7b8b9babbbcbdbebfc0");
        const QByteArray destPub = detail::derivePubKey(destSeed);
        QString cmd = QStringLiteral("MESHCORE: type=ack; dest=") +
                      QString::fromLatin1(destPub.toHex()) +
                      QStringLiteral("; msg_hash=deadbeef");
        QByteArray frame;
        QString summary, err;
        REQUIRE(Packet::buildFrameFromCommand(cmd, frame, summary, err),
                "ack command builds");
        REQUIRE(frame.size() == 1 + 1 + 1 + 4, "ack exact size");
    }

    // ---- error cases ----
    {
        QByteArray frame;
        QString summary, err;
        REQUIRE(!Packet::buildFrameFromCommand("MESHCORE: name=foo", frame, summary, err),
                "missing type rejected");
        REQUIRE(err.contains("type"), "error mentions type");
    }
    {
        QByteArray frame;
        QString summary, err;
        REQUIRE(!Packet::buildFrameFromCommand("MESHCORE: type=advert; seed=00",
                                               frame, summary, err),
                "short seed rejected");
        REQUIRE(err.contains("seed"), "error mentions seed");
    }

    // ---- deriveTxRadioSettings ----
    {
        TxRadioSettings settings;
        QString err;
        REQUIRE(Packet::deriveTxRadioSettings(
                    "MESHCORE: type=advert; sf=8; bw=62500; cr=8; sync=0x12; freq=869.618M",
                    settings, err),
                "deriveTxRadioSettings parses");
        REQUIRE(settings.spreadFactor == 8, "sf=8 applied");
        REQUIRE(settings.bandwidthHz == 62500, "bw=62500 applied");
        REQUIRE(settings.parityBits == 4, "cr=8 -> parity=4");
        REQUIRE(settings.syncWord == 0x12, "sync=0x12 applied");
        REQUIRE(settings.centerFrequencyHz == 869618000, "freq=869.618M applied");
        REQUIRE(settings.hasCommand, "hasCommand set");
        REQUIRE(settings.hasLoRaParams, "hasLoRaParams set");
        REQUIRE(settings.hasCenterFrequency, "hasCenterFrequency set");
    }
    {
        TxRadioSettings settings;
        QString err;
        REQUIRE(Packet::deriveTxRadioSettings("Hello world", settings, err),
                "non-command -> defaults OK");
        REQUIRE(settings.bandwidthHz == kDefaultBandwidthHz, "default BW");
        REQUIRE(settings.spreadFactor == kDefaultSpreadFactor, "default SF");
        REQUIRE(settings.parityBits == kDefaultParityBits, "default parity");
    }
}

QString findField(const modemmeshcore::DecodeResult& r, const QString& path)
{
    for (const auto& f : r.fields) {
        if (f.path == path) return f.value;
    }
    return QString();
}

void testRoundtripDecode()
{
    using namespace modemmeshcore;
    namespace b = modemmeshcore::builders;

    const QByteArray seedA = fromHex("0102030405060708090a0b0c0d0e0f10"
                                     "111213141516171819""1a1b1c1d1e1f20");
    const QByteArray seedB = fromHex("a1a2a3a4a5a6a7a8a9aaabacadaeafb0"
                                     "b1b2b3b4b5b6b7b8b9babbbcbdbebfc0");
    const QByteArray pubA = detail::derivePubKey(seedA);
    const QByteArray pubB = detail::derivePubKey(seedB);

    // ---- ADVERT round-trip ----
    {
        b::AdvertOptions o;
        o.timestamp = 1700001000U;
        o.name = "NodeA";
        o.routeType = RouteFlood;
        const QByteArray pkt = b::buildAdvert(seedA, pubA, o);
        REQUIRE(!pkt.isEmpty(), "advert built");

        DecodeResult r;
        REQUIRE(Packet::decodeFrame(pkt, r), "advert decoded");
        REQUIRE(r.isFrame, "advert isFrame");
        REQUIRE(r.payloadType == PayloadAdvert, "advert payloadType=Advert");
        REQUIRE(r.dataDecoded, "advert dataDecoded");
        REQUIRE(findField(r, "advert.pubkey") == QString::fromLatin1(pubA.toHex()),
                "advert.pubkey roundtrip");
        REQUIRE(findField(r, "advert.timestamp") == "1700001000",
                "advert.timestamp roundtrip");
        REQUIRE(findField(r, "advert.name") == "NodeA", "advert.name roundtrip");
        REQUIRE(findField(r, "advert.signature_valid") == "true",
                "advert signature verifies");
    }

    // ---- TXT_MSG round-trip A -> B (B's identity + A as contact) ----
    {
        b::TxtMsgOptions o;
        o.timestamp = 1700002000U;
        o.routeType = RouteDirect;
        const QByteArray text = QByteArray("Hello B from A");
        const QByteArray pkt = b::buildTxtMsg(seedA, pubB, text, o);
        REQUIRE(!pkt.isEmpty(), "txt_msg built A->B");

        const QString keys = QString("identity=") + QString::fromLatin1(seedB.toHex())
                           + "; contact:alice=" + QString::fromLatin1(pubA.toHex());
        DecodeResult r;
        REQUIRE(Packet::decodeFrame(pkt, r, keys), "txt_msg decoded");
        REQUIRE(r.payloadType == PayloadTxt, "txt_msg payloadType=Txt");
        REQUIRE(r.decrypted, "txt_msg decrypted");
        REQUIRE(findField(r, "txt.text") == text, "txt.text roundtrip");
        REQUIRE(findField(r, "txt.sender_name") == "alice", "txt.sender_name");
        REQUIRE(r.keyLabel == "contact:alice", "txt keyLabel");
    }

    // Wrong-key path: B has a key list with no matching contact
    {
        const QByteArray text = QByteArray("Hello");
        b::TxtMsgOptions o;
        o.timestamp = 1700002001U;
        const QByteArray pkt = b::buildTxtMsg(seedA, pubB, text, o);

        const QString keys = QString("identity=") + QString::fromLatin1(seedB.toHex());
        DecodeResult r;
        REQUIRE(Packet::decodeFrame(pkt, r, keys), "txt_msg envelope decoded");
        REQUIRE(!r.decrypted, "txt_msg not decrypted (no matching contact)");
    }

    // ---- GRP_TXT round-trip ----
    {
        const QByteArray psk = b::publicChannelPsk();
        const b::GroupChannel pub = b::GroupChannel::fromPsk("public", psk);
        b::GrpTxtOptions o;
        o.timestamp = 1700003000U;
        const QByteArray text = QByteArray("Public group test");
        const QByteArray pkt = b::buildGrpTxt(pub, text, o);
        REQUIRE(!pkt.isEmpty(), "grp_txt built");

        const QString keys = "channel:public=public";
        DecodeResult r;
        REQUIRE(Packet::decodeFrame(pkt, r, keys), "grp_txt decoded");
        REQUIRE(r.payloadType == PayloadGrpTxt, "grp_txt payloadType=GrpTxt");
        REQUIRE(r.decrypted, "grp_txt decrypted");
        REQUIRE(findField(r, "grp.text") == text, "grp.text roundtrip");
        REQUIRE(findField(r, "grp.channel") == "public", "grp.channel matches");
    }

    // ---- ANON_REQ round-trip ----
    {
        const QByteArray data = QByteArray("anon-payload");
        b::AnonReqOptions o;
        o.timestamp = 1700004000U;
        const QByteArray pkt = b::buildAnonReq(seedA, pubB, data, o);
        const QString keys = QString("identity=") + QString::fromLatin1(seedB.toHex());
        DecodeResult r;
        REQUIRE(Packet::decodeFrame(pkt, r, keys), "anon_req decoded");
        REQUIRE(r.payloadType == PayloadAnonReq, "anon_req payloadType=AnonReq");
        REQUIRE(r.decrypted, "anon_req decrypted");
        REQUIRE(findField(r, "anon.text") == QString::fromUtf8(data), "anon.text roundtrip");
    }

    // ---- ACK round-trip ----
    {
        const QByteArray msgHash = fromHex("cafebabe");
        b::AckOptions o;
        const QByteArray pkt = b::buildAck(pubA, msgHash, o);
        DecodeResult r;
        REQUIRE(Packet::decodeFrame(pkt, r), "ack decoded");
        REQUIRE(r.payloadType == PayloadAck, "ack payloadType=Ack");
        REQUIRE(findField(r, "ack.msg_hash") == "cafebabe", "ack.msg_hash roundtrip");
    }

    // ---- validateKeySpecList ----
    {
        QString err; int n = -1;
        const QString good = QString("identity=") + QString::fromLatin1(seedA.toHex())
                           + "; channel:public=public";
        REQUIRE(Packet::validateKeySpecList(good, err, &n), "valid key list");
        REQUIRE(n == 2, "valid key list count = 2");

        REQUIRE(!Packet::validateKeySpecList("identity=ff", err, &n),
                "short identity rejected");
        REQUIRE(!Packet::validateKeySpecList("", err, &n), "empty rejected");
    }
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    std::printf("modemmeshcore smoke test\n");
    std::printf("[1] testPathLenEncoding\n");
    testPathLenEncoding();
    std::printf("[2] testCryptoRoundtrip\n");
    testCryptoRoundtrip();
    std::printf("[3] testAdvertBuilder\n");
    testAdvertBuilder();
    std::printf("[4] testEncryptedBuilders\n");
    testEncryptedBuilders();
    std::printf("[5] testCommandParser\n");
    testCommandParser();
    std::printf("[6] testRoundtripDecode\n");
    testRoundtripDecode();
    std::printf("\nALL OK\n");
    return 0;
}
