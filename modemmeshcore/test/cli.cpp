///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Tom Hensel <code@jitter.eu>                                //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// modmeshcore_cli — print the wire packet for a given MESHCORE: command line.   //
//                                                                               //
// Usage:                                                                        //
//   modmeshcore_cli "MESHCORE: type=advert; seed=<hex64>; name=Test; ts=42"     //
//                                                                               //
// Prints the OTA-ready packet as hex on stdout, plus a one-line summary on      //
// stderr. Used for golden-vector parity tests against the python                //
// gr4-lora/scripts/src/lora/tools/meshcore_tx.py implementation.                //
///////////////////////////////////////////////////////////////////////////////////

#include "meshcorepacket.h"

#include <QByteArray>
#include <QString>

#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::fprintf(stderr,
            "usage: %s \"MESHCORE: type=...; ...\"\n"
            "examples:\n"
            "  %s \"MESHCORE: type=advert; seed=0102030405060708090a0b0c0d0e0f10"
            "111213141516171819""1a1b1c1d1e1f20; name=Test; ts=1700000000\"\n"
            "  %s \"MESHCORE: type=grp_txt; channel=public; text=Hello group; "
            "ts=1700000000\"\n",
            argv[0], argv[0], argv[0]);
        return 2;
    }

    const QString command = QString::fromUtf8(argv[1]);

    if (!modemmeshcore::Packet::isCommand(command)) {
        std::fprintf(stderr, "error: argument is not a MESHCORE: command\n");
        return 2;
    }

    QByteArray frame;
    QString summary, error;
    if (!modemmeshcore::Packet::buildFrameFromCommand(command, frame, summary, error)) {
        std::fprintf(stderr, "error: %s\n", error.toUtf8().constData());
        return 1;
    }

    std::fprintf(stderr, "%s\n", summary.toUtf8().constData());
    std::fprintf(stderr, "size: %d bytes\n", frame.size());
    std::fwrite(frame.toHex().constData(), 1, frame.toHex().size(), stdout);
    std::fputc('\n', stdout);
    return 0;
}
