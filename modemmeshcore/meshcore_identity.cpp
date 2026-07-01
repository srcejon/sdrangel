///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Tom Hensel <code@jitter.eu>                                //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#include "meshcore_identity.h"

#include <QByteArray>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QRandomGenerator>
#include <QSaveFile>
#include <QStandardPaths>
#include <QString>
#include <QtGlobal>

#include "meshcore_crypto.h"

#ifndef Q_OS_WIN
#include <sys/stat.h>
#endif

namespace modemmeshcore
{
namespace identity
{

namespace
{

constexpr int kSeedSize = 32;
constexpr int kPubSize = 32;
constexpr int kFileSize = kSeedSize + kPubSize;

bool ensureParentDir(const QString& path)
{
    QFileInfo fi(path);
    QDir parent = fi.dir();
    if (parent.exists()) return true;
    return parent.mkpath(QStringLiteral("."));
}

void chmodOwnerOnly(const QString& path)
{
#ifndef Q_OS_WIN
    ::chmod(QFile::encodeName(path).constData(), S_IRUSR | S_IWUSR);
#else
    Q_UNUSED(path);
#endif
}

} // anonymous

QString defaultIdentityPath()
{
    QString base = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    if (base.isEmpty()) {
        base = QDir::homePath();
    }
    return QDir::cleanPath(base + QStringLiteral("/meshcore/identity.bin"));
}

Identity loadIdentity(const QString& path)
{
    Identity id;
    QFile f(path);
    if (!f.exists()) return id;
    if (!f.open(QIODevice::ReadOnly)) return id;
    const QByteArray blob = f.readAll();
    f.close();
    if (blob.size() != kFileSize) return id;

    QByteArray seed = blob.left(kSeedSize);
    QByteArray pub  = blob.mid(kSeedSize, kPubSize);

    // Validate seed→pub re-derivation matches stored pub.
    QByteArray derived = detail::derivePubKey(seed);
    if (derived.size() != kPubSize || derived != pub) {
        return id;
    }

    id.seed = seed;
    id.pub = pub;
    return id;
}

bool saveIdentity(const Identity& id, const QString& path)
{
    if (!id.isValid()) return false;
    if (!ensureParentDir(path)) return false;

    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    QByteArray blob;
    blob.reserve(kFileSize);
    blob.append(id.seed);
    blob.append(id.pub);
    if (f.write(blob) != kFileSize) {
        f.cancelWriting();
        return false;
    }
    if (!f.commit()) return false;

    chmodOwnerOnly(path);
    return true;
}

Identity generateIdentity()
{
    QByteArray seed(kSeedSize, Qt::Uninitialized);
    QRandomGenerator* rng = QRandomGenerator::system();
    // Fill 32 bytes from the system CSPRNG via 8 quint32 draws.
    quint32* p32 = reinterpret_cast<quint32*>(seed.data());
    for (int i = 0; i < kSeedSize / 4; ++i) {
        p32[i] = rng->generate();
    }
    Identity id;
    id.seed = seed;
    id.pub = detail::derivePubKey(seed);
    return id;
}

Identity loadOrCreateIdentity(const QString& path)
{
    Identity id = loadIdentity(path);
    if (id.isValid()) return id;
    id = generateIdentity();
    saveIdentity(id, path);
    return id;
}

QString defaultNodeNameFor(const Identity& id)
{
    if (!id.isValid()) return QStringLiteral("SDRangel");
    return QStringLiteral("SDRangel-") + id.pubShort();
}

} // namespace identity
} // namespace modemmeshcore
