///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Tom Hensel <code@jitter.eu>                                //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// MeshCore on-host identity store. Persistent 32-byte Ed25519 seed +            //
// derived 32-byte public key, loaded from / saved to a 64-byte file at          //
// AppDataLocation/meshcore/identity.bin.                                        //
//                                                                               //
// Shared between modmeshcore (TX seed source for builders::buildAdvert /        //
// buildTxtMsg / ...) and demodmeshcore (default identity= entry in the          //
// KeysDialog so trial-decrypt against our prv key works without manual setup).  //
///////////////////////////////////////////////////////////////////////////////////

#ifndef MODEMMESHCORE_MESHCORE_IDENTITY_H_
#define MODEMMESHCORE_MESHCORE_IDENTITY_H_

#include <QByteArray>
#include <QString>

#include "export.h"

namespace modemmeshcore
{
namespace identity
{

struct MODEMMESHCORE_API Identity
{
    QByteArray seed;   // 32 bytes (Ed25519 seed; treated as private)
    QByteArray pub;    // 32 bytes (derived from seed)

    bool isValid() const
    {
        return seed.size() == 32 && pub.size() == 32;
    }

    // Helpers — convenience for callers that want hex strings.
    QString seedHex() const { return QString::fromLatin1(seed.toHex()); }
    QString pubHex() const  { return QString::fromLatin1(pub.toHex()); }

    // First 8 hex chars of the public key — used for the default node name
    // suffix ("SDRangel-73ccef30") and any short-display label.
    QString pubShort() const
    {
        return pub.size() == 32 ? QString::fromLatin1(pub.left(4).toHex()) : QString();
    }
};

// Default on-disk path: <AppDataLocation>/meshcore/identity.bin
//   macOS: ~/Library/Application Support/SDRangel/meshcore/identity.bin
//   Linux: ~/.local/share/SDRangel/meshcore/identity.bin
//   Windows: %APPDATA%/SDRangel/meshcore/identity.bin
MODEMMESHCORE_API QString defaultIdentityPath();

// Load existing identity. Returns an invalid Identity (isValid() == false)
// if the file is missing, wrong size (must be 64), or fails the
// seed → pub re-derivation check.
MODEMMESHCORE_API Identity loadIdentity(const QString& path);

// Save identity to disk. Creates parent directories as needed. Sets file
// mode 0600 on POSIX (best-effort on Windows). Returns false on I/O error
// or invalid input.
MODEMMESHCORE_API bool saveIdentity(const Identity& id, const QString& path);

// Generate a new random Identity. Uses QRandomGenerator::system() (cryptographic
// quality on platforms that support it). Always returns a valid Identity.
MODEMMESHCORE_API Identity generateIdentity();

// Load if file exists and is valid; otherwise generate + save + return.
// Common entry point for both RX and TX.
MODEMMESHCORE_API Identity loadOrCreateIdentity(const QString& path);

// Format a default node name from a public key: "SDRangel-<8hex>".
MODEMMESHCORE_API QString defaultNodeNameFor(const Identity& id);

} // namespace identity
} // namespace modemmeshcore

#endif // MODEMMESHCORE_MESHCORE_IDENTITY_H_
