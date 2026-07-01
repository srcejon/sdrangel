///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Tom Hensel <code@jitter.eu>                                //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// MESHCORE: command-line parser. Translates a single line like                  //
//                                                                               //
//   MESHCORE: type=advert; name=SDRangelTest; lat=53.55; lon=9.99               //
//   MESHCORE: type=txt_msg; dest=<hex64>; text=Hello                            //
//   MESHCORE: type=grp_txt; channel=public; text=Hello group                    //
//   MESHCORE: type=ack; dest=<hex64>; msg_hash=<hex8>                           //
//                                                                               //
// into a wire packet (via builders::*) and / or a TxRadioSettings override.     //
//                                                                               //
// Tokens are split on ';'.  Each token is a key=value pair (key compared        //
// case-insensitively).  Whitespace around tokens and around '=' is trimmed.     //
///////////////////////////////////////////////////////////////////////////////////

#ifndef MODEMMESHCORE_MESHCORE_COMMAND_H_
#define MODEMMESHCORE_MESHCORE_COMMAND_H_

#include <QByteArray>
#include <QMap>
#include <QString>

#include "export.h"
#include "meshcorepacket.h"

namespace modemmeshcore
{
namespace command
{

// Tokenize a "key1=val1; key2=val2; ..." string into a case-insensitive map.
// Returns true on parse OK, sets error on malformed input.
MODEMMESHCORE_API bool tokenize(
    const QString& body,
    QMap<QString, QString>& kv,
    QString& error);

// Apply common radio-param keys (sf, bw, cr, sync, route, freq, preamble) to
// the given TxRadioSettings.  Unknown keys are ignored (handled by caller).
// Returns true on success, false on invalid value (sets error).
MODEMMESHCORE_API bool applyRadioParams(
    const QMap<QString, QString>& kv,
    TxRadioSettings& settings,
    QString& error);

// Build a wire packet from a parsed token map.  The "type" key must already
// have been validated by the caller.  Sets summary on success.
//
// Required tokens by type:
//   advert   : seed (hex64).  Optional: name, lat, lon, ts, route.
//   txt_msg  : seed, dest (hex64), text.  Optional: txt_type, attempt, ts, route.
//   anon_req : seed, dest (hex64), data (hex or text:<utf8>).  Optional: ts, route.
//   grp_txt  : channel_psk (hex16/32) OR channel=public, text.  Optional ts, route.
//   ack      : dest (hex64), msg_hash (hex8).  Optional route.
//
// Returns true and fills frame on success, false sets error.
MODEMMESHCORE_API bool buildFrameByType(
    const QString& typeName,
    const QMap<QString, QString>& kv,
    QByteArray& frame,
    QString& summary,
    QString& error);

} // namespace command
} // namespace modemmeshcore

#endif // MODEMMESHCORE_MESHCORE_COMMAND_H_
