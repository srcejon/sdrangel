///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Tom Hensel <code@jitter.eu>                                //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// MESHCORE: command parser implementation.                                      //
///////////////////////////////////////////////////////////////////////////////////

#include "meshcore_command.h"

#include "meshcore_builders.h"
#include "meshcore_crypto.h"

#include <QDateTime>
#include <QStringList>

namespace modemmeshcore
{
namespace command
{

namespace
{

QString lower(const QString& s) { return s.toLower(); }

// Parse a hex string into bytes; empty on parse error.
QByteArray hex(const QString& v)
{
    QByteArray b = QByteArray::fromHex(v.toLatin1());
    return b;
}

// Resolve a key (case-insensitive). Empty if missing.
QString get(const QMap<QString, QString>& kv, const char* key)
{
    auto it = kv.find(QString::fromLatin1(key));
    return (it != kv.end()) ? it.value() : QString();
}

bool toUInt(const QString& s, uint32_t& out)
{
    bool ok = false;
    if (s.startsWith("0x", Qt::CaseInsensitive)) {
        out = s.mid(2).toUInt(&ok, 16);
    } else {
        out = s.toUInt(&ok, 10);
    }
    return ok;
}

bool toRouteType(const QString& s, uint8_t& out)
{
    const QString l = lower(s);
    if (l == "t_flood")  { out = RouteTFlood;  return true; }
    if (l == "flood")    { out = RouteFlood;   return true; }
    if (l == "direct")   { out = RouteDirect;  return true; }
    if (l == "t_direct") { out = RouteTDirect; return true; }
    return false;
}

uint32_t resolveTimestamp(const QMap<QString, QString>& kv)
{
    const QString ts = get(kv, "ts");
    if (ts.isEmpty()) {
        return static_cast<uint32_t>(QDateTime::currentSecsSinceEpoch());
    }
    uint32_t v = 0;
    return toUInt(ts, v) ? v : 0;
}

} // namespace

bool tokenize(const QString& body, QMap<QString, QString>& kv, QString& error)
{
    kv.clear();
    const QStringList tokens = body.split(';', Qt::SkipEmptyParts);
    for (const QString& raw : tokens)
    {
        const QString tok = raw.trimmed();
        if (tok.isEmpty()) {
            continue;
        }
        const int eq = tok.indexOf('=');
        if (eq <= 0) {
            error = QString("malformed token (expected key=value): '%1'").arg(tok);
            return false;
        }
        const QString k = tok.left(eq).trimmed().toLower();
        const QString v = tok.mid(eq + 1).trimmed();
        if (k.isEmpty()) {
            error = QString("empty key in token: '%1'").arg(tok);
            return false;
        }
        kv.insert(k, v);
    }
    return true;
}

namespace
{

// MeshCore regional preset table.
//
// Entries below are the well-known regional defaults the MeshCore
// community uses.  They map directly to the firmware CLI's
// `set radio freq_MHz,bw_kHz,sf,cr` form (cr is encoded as 5..8 for
// 4/5..4/8 — `cr` here is therefore parityBits + 4).  Default profile
// is `EU_NARROW` (MeshCore EU/UK Narrow / Recommended channel).
struct MeshcorePresetEntry
{
    const char* key;             // canonical preset key (uppercase, snake_case)
    const char* displayName;     // human-readable label (for logs/summary)
    qint64 freqHz;
    int bandwidthHz;
    int spreadFactor;
    int parityBits;              // CR = parityBits + 4
};

constexpr MeshcorePresetEntry kMeshcorePresets[] = {
    {"EU_NARROW",          "EU/UK Narrow (Recommended)",   869618000,  62500, 8, 4},
    {"EU_LONG_RANGE",      "EU/UK Long Range",             869525000, 250000, 11, 1},
    {"EU_MEDIUM_RANGE",    "EU/UK Medium Range",           869525000, 250000, 10, 1},
    {"AU",                 "Australia",                    915800000, 250000, 10, 1},
    {"AU_VICTORIA",        "Australia: Victoria",          916575000,  62500, 7, 4},
    {"CZ_NARROW",          "Czech Republic Narrow",        869525000,  62500, 7, 1},
    {"EU_433_LONG_RANGE",  "EU 433 MHz Long Range",        433650000, 250000, 11, 1},
    {"NZ",                 "New Zealand",                  917375000, 250000, 11, 1},
    {"NZ_NARROW",          "New Zealand Narrow",           917375000,  62500, 7, 1},
    {"PT_433",             "Portugal 433",                 433375000,  62500, 9, 2},
    {"PT_868",             "Portugal 868",                 869618000,  62500, 7, 2},
    {"CH",                 "Switzerland",                  869618000,  62500, 8, 4},
    {"USA",                "USA / Canada",                 910525000,  62500, 7, 1},
    {"VN",                 "Vietnam",                      920250000, 250000, 11, 1},
};

bool applyMeshcorePreset(const QString& preset, TxRadioSettings& settings, QString& error)
{
    const QByteArray key = preset.toUpper().toLatin1();
    if (key == "USER") {
        return true;        // USER preset = no overrides; caller's individual keys win
    }
    for (const auto& entry : kMeshcorePresets) {
        if (key == entry.key) {
            settings.centerFrequencyHz = entry.freqHz;
            settings.hasCenterFrequency = true;
            settings.bandwidthHz = entry.bandwidthHz;
            settings.spreadFactor = entry.spreadFactor;
            settings.parityBits = entry.parityBits;
            settings.hasLoRaParams = true;
            settings.summary = QString("MeshCore preset %1").arg(entry.displayName);
            return true;
        }
    }
    error = QString("invalid MeshCore preset: '%1'").arg(preset);
    return false;
}

} // namespace

bool applyRadioParams(const QMap<QString, QString>& kv, TxRadioSettings& settings, QString& error)
{
    bool changed = false;

    // Apply MeshCore regional preset first (if present), then let individual
    // sf/bw/cr/freq/sync/preamble keys override specific fields.
    const QString presetS = get(kv, "preset");
    if (!presetS.isEmpty()) {
        if (!applyMeshcorePreset(presetS, settings, error)) {
            return false;
        }
        // SF-dependent preamble preamble: SF < 9 → 32, SF > 8 → 16.
        // Matches upstream meshcore-py commit 2026-06.
        if (settings.spreadFactor < 9) { settings.preambleChirps = 32; }
        else { settings.preambleChirps = 16; }
        changed = true;
    }

    const QString sfS = get(kv, "sf");
    if (!sfS.isEmpty()) {
        uint32_t sf = 0;
        if (!toUInt(sfS, sf) || sf < 7 || sf > 12) {
            error = QString("invalid sf: '%1' (expected 7..12)").arg(sfS);
            return false;
        }
        settings.spreadFactor = static_cast<int>(sf);
        settings.hasLoRaParams = true;
        changed = true;
    }

    const QString bwS = get(kv, "bw");
    if (!bwS.isEmpty()) {
        uint32_t bw = 0;
        if (!toUInt(bwS, bw) || bw == 0) {
            error = QString("invalid bw: '%1' (expected Hz)").arg(bwS);
            return false;
        }
        settings.bandwidthHz = static_cast<int>(bw);
        settings.hasLoRaParams = true;
        changed = true;
    }

    const QString crS = get(kv, "cr");
    if (!crS.isEmpty()) {
        uint32_t cr = 0;
        if (!toUInt(crS, cr) || cr < 5 || cr > 8) {
            error = QString("invalid cr: '%1' (expected 5..8 for 4/5..4/8)").arg(crS);
            return false;
        }
        settings.parityBits = static_cast<int>(cr) - 4; // 5 -> 1, 8 -> 4
        settings.hasLoRaParams = true;
        changed = true;
    }

    const QString syncS = get(kv, "sync");
    if (!syncS.isEmpty()) {
        uint32_t sync = 0;
        if (!toUInt(syncS, sync) || sync > 0xFF) {
            error = QString("invalid sync: '%1'").arg(syncS);
            return false;
        }
        settings.syncWord = static_cast<uint8_t>(sync);
        settings.hasLoRaParams = true;
        changed = true;
    }

    const QString freqS = get(kv, "freq");
    if (!freqS.isEmpty()) {
        // Accept "869618000" (Hz) or "869.618M" / "869.618MHz"
        QString s = freqS;
        s.replace("Hz", "", Qt::CaseInsensitive);
        const bool isMhz = s.contains('M', Qt::CaseInsensitive);
        s.replace("M", "", Qt::CaseInsensitive);
        bool ok = false;
        const double v = s.toDouble(&ok);
        if (!ok || v <= 0) {
            error = QString("invalid freq: '%1'").arg(freqS);
            return false;
        }
        settings.centerFrequencyHz = static_cast<qint64>(isMhz ? v * 1.0e6 : v);
        settings.hasCenterFrequency = true;
        changed = true;
    }

    const QString prS = get(kv, "preamble");
    if (!prS.isEmpty()) {
        uint32_t pr = 0;
        if (!toUInt(prS, pr) || pr == 0 || pr > 1024) {
            error = QString("invalid preamble: '%1'").arg(prS);
            return false;
        }
        settings.preambleChirps = static_cast<int>(pr);
        settings.hasLoRaParams = true;
        changed = true;
    }

    if (changed) {
        settings.hasCommand = true;
    }
    return true;
}

namespace
{

bool buildAdvertFromKv(const QMap<QString, QString>& kv,
                       QByteArray& frame, QString& summary, QString& error)
{
    const QByteArray seed = hex(get(kv, "seed"));
    if (seed.size() != kPubKeySize) {
        error = "advert: seed (hex 32 bytes) is required";
        return false;
    }
    const QByteArray pub = detail::derivePubKey(seed);
    if (pub.size() != kPubKeySize) {
        error = "advert: failed to derive public key";
        return false;
    }

    builders::AdvertOptions opts;
    opts.timestamp = resolveTimestamp(kv);
    opts.name = get(kv, "name");

    const QString latS = get(kv, "lat");
    const QString lonS = get(kv, "lon");
    if (!latS.isEmpty() && !lonS.isEmpty()) {
        bool okLat = false, okLon = false;
        const double lat = latS.toDouble(&okLat);
        const double lon = lonS.toDouble(&okLon);
        if (!okLat || !okLon) {
            error = "advert: invalid lat/lon";
            return false;
        }
        opts.latLon = std::make_pair(lat, lon);
    }

    const QString routeS = get(kv, "route");
    if (!routeS.isEmpty()) {
        if (!toRouteType(routeS, opts.routeType)) {
            error = QString("advert: invalid route '%1'").arg(routeS);
            return false;
        }
    }

    frame = builders::buildAdvert(seed, pub, opts);
    if (frame.isEmpty()) {
        error = "advert: build failed";
        return false;
    }
    summary = QString("MESHCORE TX|advert pub=%1 ts=%2 name=\"%3\"")
                  .arg(QString::fromLatin1(pub.toHex().left(16)))
                  .arg(opts.timestamp)
                  .arg(opts.name);
    return true;
}

bool buildTxtMsgFromKv(const QMap<QString, QString>& kv,
                       QByteArray& frame, QString& summary, QString& error)
{
    const QByteArray seed = hex(get(kv, "seed"));
    const QByteArray dest = hex(get(kv, "dest"));
    const QString text = get(kv, "text");
    if (seed.size() != kPubKeySize) {
        error = "txt_msg: seed (hex 32 bytes) required";
        return false;
    }
    if (dest.size() != kPubKeySize) {
        error = "txt_msg: dest (hex 32 bytes) required";
        return false;
    }
    if (text.isEmpty()) {
        error = "txt_msg: text required";
        return false;
    }

    builders::TxtMsgOptions opts;
    opts.timestamp = resolveTimestamp(kv);

    uint32_t txtType = 0, attempt = 0;
    const QString tt = get(kv, "txt_type");
    if (!tt.isEmpty() && (!toUInt(tt, txtType) || txtType > 0x3F)) {
        error = "txt_msg: invalid txt_type";
        return false;
    }
    const QString at = get(kv, "attempt");
    if (!at.isEmpty() && (!toUInt(at, attempt) || attempt > 3)) {
        error = "txt_msg: invalid attempt (0..3)";
        return false;
    }
    opts.txtType = static_cast<uint8_t>(txtType);
    opts.attempt = static_cast<uint8_t>(attempt);

    const QString routeS = get(kv, "route");
    if (!routeS.isEmpty() && !toRouteType(routeS, opts.routeType)) {
        error = QString("txt_msg: invalid route '%1'").arg(routeS);
        return false;
    }

    frame = builders::buildTxtMsg(seed, dest, text.toUtf8(), opts);
    if (frame.isEmpty()) {
        error = "txt_msg: build failed";
        return false;
    }
    summary = QString("MESHCORE TX|txt_msg dest=%1 ts=%2 text=\"%3\"")
                  .arg(QString::fromLatin1(dest.toHex().left(16)))
                  .arg(opts.timestamp)
                  .arg(text);
    return true;
}

bool buildAnonReqFromKv(const QMap<QString, QString>& kv,
                        QByteArray& frame, QString& summary, QString& error)
{
    const QByteArray seed = hex(get(kv, "seed"));
    const QByteArray dest = hex(get(kv, "dest"));
    if (seed.size() != kPubKeySize) {
        error = "anon_req: seed required";
        return false;
    }
    if (dest.size() != kPubKeySize) {
        error = "anon_req: dest required";
        return false;
    }

    QByteArray data;
    const QString dataHex = get(kv, "data");
    const QString dataTxt = get(kv, "text");
    if (!dataHex.isEmpty()) {
        data = hex(dataHex);
        if (data.isEmpty()) {
            error = "anon_req: invalid data hex";
            return false;
        }
    } else if (!dataTxt.isEmpty()) {
        data = dataTxt.toUtf8();
    } else {
        error = "anon_req: data or text required";
        return false;
    }

    builders::AnonReqOptions opts;
    opts.timestamp = resolveTimestamp(kv);
    const QString routeS = get(kv, "route");
    if (!routeS.isEmpty() && !toRouteType(routeS, opts.routeType)) {
        error = QString("anon_req: invalid route '%1'").arg(routeS);
        return false;
    }

    frame = builders::buildAnonReq(seed, dest, data, opts);
    if (frame.isEmpty()) {
        error = "anon_req: build failed";
        return false;
    }
    summary = QString("MESHCORE TX|anon_req dest=%1 ts=%2 len=%3")
                  .arg(QString::fromLatin1(dest.toHex().left(16)))
                  .arg(opts.timestamp)
                  .arg(data.size());
    return true;
}

bool buildGrpTxtFromKv(const QMap<QString, QString>& kv,
                       QByteArray& frame, QString& summary, QString& error)
{
    const QString text = get(kv, "text");
    if (text.isEmpty()) {
        error = "grp_txt: text required";
        return false;
    }

    QByteArray psk;
    QString chanName = get(kv, "channel");
    const QString pskS = get(kv, "channel_psk");
    if (!pskS.isEmpty()) {
        psk = hex(pskS);
        if (psk.size() != 16 && psk.size() != 32) {
            error = "grp_txt: channel_psk must be 16 or 32 hex bytes";
            return false;
        }
        if (chanName.isEmpty()) {
            chanName = "psk";
        }
    } else if (chanName.compare("public", Qt::CaseInsensitive) == 0) {
        psk = builders::publicChannelPsk();
    } else {
        error = "grp_txt: channel_psk or channel=public required";
        return false;
    }

    const builders::GroupChannel ch = builders::GroupChannel::fromPsk(chanName, psk);
    if (!ch.isValid()) {
        error = "grp_txt: invalid channel";
        return false;
    }

    builders::GrpTxtOptions opts;
    opts.timestamp = resolveTimestamp(kv);

    uint32_t txtType = 0, attempt = 0;
    const QString tt = get(kv, "txt_type");
    if (!tt.isEmpty() && (!toUInt(tt, txtType) || txtType > 0x3F)) {
        error = "grp_txt: invalid txt_type";
        return false;
    }
    const QString at = get(kv, "attempt");
    if (!at.isEmpty() && (!toUInt(at, attempt) || attempt > 3)) {
        error = "grp_txt: invalid attempt";
        return false;
    }
    opts.txtType = static_cast<uint8_t>(txtType);
    opts.attempt = static_cast<uint8_t>(attempt);

    const QString routeS = get(kv, "route");
    if (!routeS.isEmpty() && !toRouteType(routeS, opts.routeType)) {
        error = QString("grp_txt: invalid route '%1'").arg(routeS);
        return false;
    }

    frame = builders::buildGrpTxt(ch, text.toUtf8(), opts);
    if (frame.isEmpty()) {
        error = "grp_txt: build failed";
        return false;
    }
    summary = QString("MESHCORE TX|grp_txt channel=\"%1\" hash=%2 ts=%3 text=\"%4\"")
                  .arg(chanName)
                  .arg(QString::fromLatin1(ch.hash.toHex()))
                  .arg(opts.timestamp)
                  .arg(text);
    return true;
}

bool buildAckFromKv(const QMap<QString, QString>& kv,
                    QByteArray& frame, QString& summary, QString& error)
{
    const QByteArray dest = hex(get(kv, "dest"));
    const QByteArray msgHash = hex(get(kv, "msg_hash"));
    if (dest.size() != kPubKeySize) {
        error = "ack: dest required";
        return false;
    }
    if (msgHash.size() != 4) {
        error = "ack: msg_hash must be 4 hex bytes";
        return false;
    }

    builders::AckOptions opts;
    const QString routeS = get(kv, "route");
    if (!routeS.isEmpty() && !toRouteType(routeS, opts.routeType)) {
        error = QString("ack: invalid route '%1'").arg(routeS);
        return false;
    }

    frame = builders::buildAck(dest, msgHash, opts);
    if (frame.isEmpty()) {
        error = "ack: build failed";
        return false;
    }
    summary = QString("MESHCORE TX|ack dest=%1 hash=%2")
                  .arg(QString::fromLatin1(dest.toHex().left(16)))
                  .arg(QString::fromLatin1(msgHash.toHex()));
    return true;
}

} // namespace

bool buildFrameByType(const QString& typeName,
                      const QMap<QString, QString>& kv,
                      QByteArray& frame,
                      QString& summary,
                      QString& error)
{
    const QString t = typeName.toLower();
    if (t == "advert")   { return buildAdvertFromKv(kv, frame, summary, error); }
    if (t == "txt_msg")  { return buildTxtMsgFromKv(kv, frame, summary, error); }
    if (t == "anon_req") { return buildAnonReqFromKv(kv, frame, summary, error); }
    if (t == "grp_txt")  { return buildGrpTxtFromKv(kv, frame, summary, error); }
    if (t == "ack")      { return buildAckFromKv(kv, frame, summary, error); }
    error = QString("unknown type: '%1' (expected advert|txt_msg|anon_req|grp_txt|ack)")
                .arg(typeName);
    return false;
}

} // namespace command
} // namespace modemmeshcore
