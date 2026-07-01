///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Tom Hensel <code@jitter.eu>                                //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// MeshCore packet entry point. Wires the public Packet API to the internal      //
// command parser + builders + (eventually) decoder.                             //
///////////////////////////////////////////////////////////////////////////////////

#include "meshcorepacket.h"
#include "meshcore_command.h"
#include "meshcore_decoder.h"

#include <QDebug>
#include <QProcessEnvironment>

namespace modemmeshcore
{

namespace
{

constexpr const char* kCommandPrefix = "MESHCORE:";

// Strip the leading "MESHCORE:" prefix and return the remaining body.
QString stripPrefix(const QString& cmd)
{
    QString t = cmd.trimmed();
    if (t.startsWith(QString::fromLatin1(kCommandPrefix), Qt::CaseInsensitive)) {
        return t.mid(static_cast<int>(strlen(kCommandPrefix))).trimmed();
    }
    return QString();
}

} // namespace

QString Packet::defaultKeysFromEnv()
{
    const QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    const QString keys = env.value(QStringLiteral("SDRANGEL_MESHCORE_KEYS")).trimmed();
    if (!keys.isEmpty()) {
        qInfo("modemmeshcore::Packet::defaultKeysFromEnv: using SDRANGEL_MESHCORE_KEYS");
        return keys;
    }
    return {};
}

bool Packet::isCommand(const QString& text)
{
    return text.trimmed().startsWith(QString::fromLatin1(kCommandPrefix), Qt::CaseInsensitive);
}

bool Packet::buildFrameFromCommand(const QString& command, QByteArray& frame, QString& summary, QString& error)
{
    const QString body = stripPrefix(command);
    if (body.isEmpty()) {
        error = "missing MESHCORE: prefix or empty body";
        return false;
    }

    QMap<QString, QString> kv;
    if (!command::tokenize(body, kv, error)) {
        return false;
    }

    auto typeIt = kv.find(QStringLiteral("type"));
    if (typeIt == kv.end()) {
        error = "missing 'type=' (expected advert|txt_msg|anon_req|grp_txt|ack)";
        return false;
    }

    return command::buildFrameByType(typeIt.value(), kv, frame, summary, error);
}

bool Packet::decodeFrame(const QByteArray& frame, DecodeResult& result)
{
    return decodeFrame(frame, result, QString());
}

bool Packet::decodeFrame(const QByteArray& frame, DecodeResult& result, const QString& keySpecList)
{
    result = DecodeResult();

    decoder::ParsedHeader hdr;
    if (!decoder::parseHeader(frame, hdr)) {
        return false;
    }
    result.isFrame = true;
    result.routeType = hdr.routeType;
    result.payloadType = hdr.payloadType;
    result.payloadVer = hdr.version;

    QVector<decoder::KeySpec> keys;
    if (!keySpecList.isEmpty()) {
        QString tmpErr;
        if (!decoder::parseKeySpecList(keySpecList, keys, tmpErr, /*validateOnly=*/false)) {
            // Bad key list — keep going on the envelope-only path.
            qWarning() << "modemmeshcore::Packet::decodeFrame: invalid key list:" << tmpErr;
            keys.clear();
        }
    }

    switch (hdr.payloadType)
    {
    case PayloadAdvert:  return decoder::decodeAdvert(frame, hdr, result);
    case PayloadAck:     return decoder::decodeAck(frame, hdr, result);
    case PayloadTxt:     return decoder::decodeTxtMsg(frame, hdr, keys, result);
    case PayloadGrpTxt:  return decoder::decodeGrpTxt(frame, hdr, keys, result);
    case PayloadAnonReq: return decoder::decodeAnonReq(frame, hdr, keys, result);
    default:
        // Envelope decoded; payload-type-specific decoder not implemented yet.
        return true;
    }
}

bool Packet::validateKeySpecList(const QString& keySpecList, QString& error, int* keyCount)
{
    QVector<decoder::KeySpec> keys;
    if (!decoder::parseKeySpecList(keySpecList, keys, error, /*validateOnly=*/true)) {
        if (keyCount) {
            *keyCount = 0;
        }
        return false;
    }
    if (keyCount) {
        *keyCount = keys.size();
    }
    if (keys.isEmpty()) {
        error = "no keys found";
        return false;
    }
    error.clear();
    return true;
}

bool Packet::deriveTxRadioSettings(const QString& command, TxRadioSettings& settings, QString& error)
{
    settings = TxRadioSettings();           // MeshCore EU defaults
    settings.summary = QStringLiteral("MeshCore EU defaults");

    if (!isCommand(command)) {
        return true;                        // not a command -> defaults stand
    }

    const QString body = stripPrefix(command);
    QMap<QString, QString> kv;
    QString tokErr;
    if (!command::tokenize(body, kv, tokErr)) {
        error = tokErr;
        return false;
    }

    if (!command::applyRadioParams(kv, settings, error)) {
        return false;
    }
    if (settings.hasCommand) {
        settings.summary = QStringLiteral("MeshCore radio override applied");
    }
    return true;
}

} // namespace modemmeshcore
