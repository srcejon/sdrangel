///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                  //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#include "cameraplatesolverinternal.h"

// Siril SPCC/Gaia network catalog path + disk-cache helpers (WS5).

QString CameraPlateSolver::SolverContext::sirilAstroCatalogPath()
{
    return QDir(downloadedCatalogDir()).filePath(QString::fromUtf8(kSirilAstroFileName));
}

QString CameraPlateSolver::SolverContext::sirilAstroCompressedCatalogPath()
{
    return QDir(downloadedCatalogDir()).filePath(QString::fromUtf8(kSirilAstroCompressedFileName));
}

QString CameraPlateSolver::SolverContext::sirilSpccChunkUrl(int chunkIndex, int sourceIndex)
{
    const QString fileName = QString::fromUtf8(kSirilSpccFileNamePattern).arg(chunkIndex);
    if (sourceIndex == 1)
    {
        return QStringLiteral("%1/%2?download=1").arg(
            QString::fromUtf8(kSirilSpccZenodoBaseUrl),
            fileName);
    }

    return QStringLiteral("%1/%2").arg(
        QString::fromUtf8(kSirilSpccBaseUrl),
        fileName);
}

QString CameraPlateSolver::SolverContext::sirilSpccSourceName(int sourceIndex)
{
    return sourceIndex == 1 ? QStringLiteral("Zenodo") : QStringLiteral("Hugging Face");
}

QString CameraPlateSolver::SolverContext::sirilRangeCacheKey(int chunkIndex, qint64 firstByte, qint64 lastByte)
{
    return QStringLiteral("%1:%2:%3").arg(chunkIndex).arg(firstByte).arg(lastByte);
}

QString CameraPlateSolver::SolverContext::sirilCacheRootDir()
{
    return QDir(downloadedCatalogDir()).filePath(QString::fromUtf8(kSirilCacheDir));
}

QString CameraPlateSolver::SolverContext::sirilIndexDiskCachePath(int chunkIndex)
{
    return QDir(QDir(sirilCacheRootDir()).filePath(QStringLiteral("indexes")))
        .filePath(QStringLiteral("chunk-%1.idx").arg(chunkIndex));
}

QString CameraPlateSolver::SolverContext::sirilRangeDiskCachePath(int chunkIndex, qint64 firstByte, qint64 lastByte)
{
    return QDir(QDir(sirilCacheRootDir()).filePath(QStringLiteral("ranges")))
        .filePath(QStringLiteral("chunk-%1-%2-%3.bin").arg(chunkIndex).arg(firstByte).arg(lastByte));
}

QString CameraPlateSolver::SolverContext::sirilRegionCacheRootDir()
{
    return QDir(downloadedCatalogDir()).filePath(QString::fromUtf8(kSirilRegionCacheDir));
}

QString CameraPlateSolver::SolverContext::sirilAstroRegionCacheRootDir()
{
    return QDir(downloadedCatalogDir()).filePath(QString::fromUtf8(kSirilAstroRegionCacheDir));
}

QString CameraPlateSolver::SolverContext::sirilRegionCacheKey(double centerRaDegrees, double centerDecDegrees, double queryRadiusDegrees, double maxMagnitude)
{
    return QStringLiteral("ra%1_dec%2_r%3_m%4.bin")
        .arg(qRound64(normalizeDegrees(centerRaDegrees) * 1000.0))
        .arg(qRound64(centerDecDegrees * 1000.0))
        .arg(qRound64(queryRadiusDegrees * 1000.0))
        .arg(qRound64(maxMagnitude * 100.0));
}

QString CameraPlateSolver::SolverContext::sirilRegionDiskCachePath(double centerRaDegrees, double centerDecDegrees, double queryRadiusDegrees, double maxMagnitude)
{
    return QDir(sirilRegionCacheRootDir()).filePath(
        sirilRegionCacheKey(centerRaDegrees, centerDecDegrees, queryRadiusDegrees, maxMagnitude));
}

QString CameraPlateSolver::SolverContext::sirilAstroRegionDiskCachePath(double centerRaDegrees, double centerDecDegrees, double queryRadiusDegrees, double maxMagnitude)
{
    return QDir(sirilAstroRegionCacheRootDir()).filePath(
        sirilRegionCacheKey(centerRaDegrees, centerDecDegrees, queryRadiusDegrees, maxMagnitude));
}

QVector<CameraPlateSolver::SolverContext::CatalogStar> CameraPlateSolver::SolverContext::readSirilRegionDiskCacheFile(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        return {};
    }

    const QByteArray data = file.readAll();
    const char *p = data.constData();
    const char *end = p + data.size();

    // Validate header
    if (data.size() < 9
        || p[0] != kBinCacheMagic[0]
        || p[1] != kBinCacheMagic[1]
        || p[2] != kBinCacheMagic[2]
        || p[3] != kBinCacheMagic[3]
        || static_cast<quint8>(p[4]) != kBinCacheVersion)
    {
        return {};
    }
    p += 5;

    quint32 starCount = 0;
    std::memcpy(&starCount, p, 4);
    starCount = qFromLittleEndian(starCount);
    p += 4;

    if (starCount == 0) {
        return {};
    }

    QVector<CatalogStar> stars;
    stars.reserve(static_cast<int>(std::min<quint32>(starCount, 8000000u)));

    for (quint32 i = 0; i < starCount; ++i)
    {
        // Need at least ra(8) + dec(8) + mag(4) + nameLen(1) + spectralLen(1) = 22 bytes
        if ((end - p) < 22) {
            return {};
        }

        double ra = 0.0;
        double dec = 0.0;
        float mag = 0.0f;
        std::memcpy(&ra, p, 8); p += 8;
        std::memcpy(&dec, p, 8); p += 8;
        std::memcpy(&mag, p, 4); p += 4;
        ra  = qFromLittleEndian(ra);
        dec = qFromLittleEndian(dec);
        mag = qFromLittleEndian(mag);

        const quint8 nameLen = static_cast<quint8>(*p++);
        if ((end - p) < nameLen + 1) {
            return {};
        }
        const QString name = QString::fromUtf8(p, nameLen);
        p += nameLen;

        const quint8 spectralLen = static_cast<quint8>(*p++);
        if ((end - p) < spectralLen) {
            return {};
        }
        const QString spectralType = spectralLen > 0 ? QString::fromUtf8(p, spectralLen) : QString();
        p += spectralLen;

        stars.append({name, ra, dec, static_cast<double>(mag), spectralType});
    }

    return stars;
}

qint64 CameraPlateSolver::SolverContext::sirilRegionDiskCacheMaxBytes(int maxSizeGb)
{
    if (maxSizeGb <= 0) {
        return 0;
    }

    return static_cast<qint64>(maxSizeGb) * 1024LL * 1024LL * 1024LL;
}

void CameraPlateSolver::SolverContext::enforceSirilRegionDiskCacheLimit(const QString& directoryPath, int maxSizeGb)
{
    const qint64 maxBytes = sirilRegionDiskCacheMaxBytes(maxSizeGb);
    if (maxBytes <= 0) {   // 0 = unlimited
        return;
    }
    QDir dir(directoryPath);
    if (!dir.exists()) {
        return;
    }
    // Oldest first (QDir::Time is newest-first; Reversed flips it).
    const QFileInfoList files = dir.entryInfoList(
        QStringList() << QStringLiteral("*.bin"),
        QDir::Files | QDir::NoSymLinks,
        QDir::Time | QDir::Reversed);
    qint64 total = 0;
    for (const QFileInfo& info : files) {
        total += info.size();
    }
    if (total <= maxBytes) {
        return;
    }
    // Evict to a low-water mark (95%) so we don't re-scan/evict on every subsequent write.
    const qint64 lowWater = maxBytes - (maxBytes / 20);
    for (const QFileInfo& info : files) {
        if (total <= lowWater) {
            break;
        }
        const qint64 size = info.size();
        if (QFile::remove(info.absoluteFilePath())) {
            total -= size;
        }
    }
}

void CameraPlateSolver::SolverContext::writeSirilRegionDiskCacheFile(const QString& path, const QVector<CatalogStar>& stars, int maxSizeGb)
{
    if (stars.isEmpty()) {
        return;
    }

    QDir dir(QFileInfo(path).absolutePath());
    if (!dir.exists() && !dir.mkpath(QStringLiteral("."))) {
        return;
    }

    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return;
    }

    // Binary format — see readSirilRegionDiskCacheFile for layout.
    // Pre-size by ~22 bytes/star (minimum record) + header; actual size varies by
    // name/spectral length but is always smaller than the equivalent TSV.
    QByteArray buf;
    buf.reserve(static_cast<qsizetype>(stars.size()) * 24 + 9);

    // Header
    buf.append(kBinCacheMagic, 4);
    buf.append(static_cast<char>(kBinCacheVersion));
    const quint32 starCountLE = qToLittleEndian(static_cast<quint32>(stars.size()));
    buf.append(reinterpret_cast<const char*>(&starCountLE), 4);

    for (const CatalogStar& star : stars)
    {
        const double raLE  = qToLittleEndian(star.rightAscensionDegrees);
        const double decLE = qToLittleEndian(star.declinationDegrees);
        const float  magLE = qToLittleEndian(static_cast<float>(star.magnitude));
        buf.append(reinterpret_cast<const char*>(&raLE),  8);
        buf.append(reinterpret_cast<const char*>(&decLE), 8);
        buf.append(reinterpret_cast<const char*>(&magLE), 4);

        const QByteArray nameUtf8     = star.name.toUtf8();
        const QByteArray spectralUtf8 = star.spectralType.toUtf8();
        const quint8 nameLen     = static_cast<quint8>(std::min<int>(nameUtf8.size(),     255));
        const quint8 spectralLen = static_cast<quint8>(std::min<int>(spectralUtf8.size(), 255));
        buf.append(static_cast<char>(nameLen));
        buf.append(nameUtf8.constData(), nameLen);
        buf.append(static_cast<char>(spectralLen));
        buf.append(spectralUtf8.constData(), spectralLen);
    }

    file.write(buf);
    if (!file.commit()) {
        return;
    }
    enforceSirilRegionDiskCacheLimit(QFileInfo(path).absolutePath(), maxSizeGb);
}

QByteArray CameraPlateSolver::SolverContext::readSirilDiskCacheFile(const QString& path, qint64 expectedSize)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        return {};
    }
    if ((expectedSize >= 0) && (file.size() != expectedSize)) {
        return {};
    }
    return file.readAll();
}

void CameraPlateSolver::SolverContext::writeSirilDiskCacheFile(const QString& path, const QByteArray& data)
{
    if (data.isEmpty()) {
        return;
    }

    QDir dir(QFileInfo(path).absolutePath());
    if (!dir.exists() && !dir.mkpath(QStringLiteral("."))) {
        return;
    }

    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        return;
    }
    if (file.write(data) != data.size()) {
        file.cancelWriting();
        return;
    }
    file.commit();
}

QSet<quint32> CameraPlateSolver::SolverContext::sampleSirilHealpixPixels(double centerRaDegrees, double centerDecDegrees, double radiusDegrees)
{
    QSet<quint32> pixels;
    const double boundedRadius = std::max(0.1, radiusDegrees);
    const double stepDegrees = std::clamp(boundedRadius / 35.0, 0.03, 0.20);
    const double minDec = std::max(-90.0, centerDecDegrees - boundedRadius);
    const double maxDec = std::min(90.0, centerDecDegrees + boundedRadius);

    for (double dec = minDec; dec <= maxDec; dec += stepDegrees)
    {
        const double cosDec = std::max(0.05, std::cos(degToRad(dec)));
        const double raHalfWidth = std::min(180.0, boundedRadius / cosDec);
        const double raStep = std::max(stepDegrees, stepDegrees / cosDec);
        for (double raOffset = -raHalfWidth; raOffset <= raHalfWidth; raOffset += raStep)
        {
            const double ra = normalizeDegrees(centerRaDegrees + raOffset);
            if (angularSeparationDegrees(centerRaDegrees, centerDecDegrees, ra, dec) <= (boundedRadius + stepDegrees)) {
                pixels.insert(healpixNestedPixel(ra, dec));
            }
        }
    }

    pixels.insert(healpixNestedPixel(centerRaDegrees, centerDecDegrees));
    pixels.insert(healpixNestedPixel(centerRaDegrees - boundedRadius, centerDecDegrees));
    pixels.insert(healpixNestedPixel(centerRaDegrees + boundedRadius, centerDecDegrees));
    pixels.insert(healpixNestedPixel(centerRaDegrees, minDec));
    pixels.insert(healpixNestedPixel(centerRaDegrees, maxDec));
    return pixels;
}

QByteArray CameraPlateSolver::SolverContext::readSirilAstroIndex(QFile& file)
{
    if (!file.seek(kSirilHeaderSize)) {
        return {};
    }

    const QByteArray indexBytes = file.read(kSirilAstroIndexSize);
    if (indexBytes.size() != kSirilAstroIndexSize)
    {
        qWarning() << "CameraPlateSolver: Siril Gaia astrometric index read failed"
                   << "path" << file.fileName()
                   << "expected" << kSirilAstroIndexSize
                   << "got" << indexBytes.size();
        return {};
    }

    return indexBytes;
}

bool CameraPlateSolver::SolverContext::sirilAstroCellRecordRange(const QByteArray& indexBytes, quint32 pixel, qint64& firstRecord, qint64& recordCount)
{
    if ((pixel >= static_cast<quint32>(kSirilAstroPixels)) || (indexBytes.size() != kSirilAstroIndexSize)) {
        return false;
    }

    const char *cellEndBytes = indexBytes.constData() + static_cast<qint64>(pixel) * sizeof(quint32);
    const quint32 cellStart = (pixel > 0)
        ? qFromLittleEndian<quint32>(reinterpret_cast<const uchar *>(cellEndBytes - sizeof(quint32)))
        : 0;
    const quint32 cellEnd = qFromLittleEndian<quint32>(reinterpret_cast<const uchar *>(cellEndBytes));
    const quint32 catalogRecordCount = qFromLittleEndian<quint32>(
        reinterpret_cast<const uchar *>(indexBytes.constData() + kSirilAstroIndexSize - sizeof(quint32)));
    if ((cellEnd < cellStart) || (cellEnd > catalogRecordCount))
    {
        qWarning() << "CameraPlateSolver: Siril Gaia astrometric invalid cell index"
                   << "pixel" << pixel
                   << "cellStart" << cellStart
                   << "cellEnd" << cellEnd
                   << "catalogRecords" << catalogRecordCount;
        return false;
    }

    firstRecord = cellStart;
    recordCount = static_cast<qint64>(cellEnd) - static_cast<qint64>(cellStart);
    if (recordCount > kSirilMaxRecordsPerCell)
    {
        qWarning() << "CameraPlateSolver: Siril Gaia astrometric cell has too many records"
                   << "pixel" << pixel
                   << "records" << recordCount
                   << "max" << kSirilMaxRecordsPerCell;
        return false;
    }

    return true;
}

