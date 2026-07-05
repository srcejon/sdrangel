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

// Catalog I/O, projection, visibility, signatures/seeds, matching - the remaining solver core (WS5).

qint64 CameraPlateSolver::SolverContext::boundedMultiply(qint64 lhs, qint64 rhs)
{
    if ((lhs <= 0) || (rhs <= 0)) {
        return 0;
    }

    const qint64 maxValue = std::numeric_limits<qint64>::max();
    if (lhs > (maxValue / rhs)) {
        return maxValue;
    }

    return lhs * rhs;
}

qint64 CameraPlateSolver::SolverContext::estimateRefinementWorkUnits(int candidateCount, int visibleStarCount, int detectionCount)
{
    const qint64 perCandidateWork = boundedMultiply(
        std::max<qint64>(1, visibleStarCount),
        std::max<qint64>(1, detectionCount));
    return boundedMultiply(std::max<qint64>(0, candidateCount), perCandidateWork);
}

int CameraPlateSolver::SolverContext::refinementWorkerThreadCount(int candidateCount, qint64 estimatedWorkUnits)
{
    static constexpr int kMinParallelRefinementCandidates = 128;
    static constexpr int kRefinementCandidatesPerWorker = 32;
    static constexpr qint64 kMinParallelRefinementWorkUnits = 300LL * 1000LL * 1000LL;
    static constexpr qint64 kRefinementWorkUnitsPerWorker = 250LL * 1000LL * 1000LL;

    if (candidateCount < 2) {
        return 1;
    }

    const int idealThreadCount = QThread::idealThreadCount();
    if (idealThreadCount <= 2) {
        return 1;
    }

    const bool enoughCandidateCount = candidateCount >= kMinParallelRefinementCandidates;
    const bool enoughEstimatedWork = estimatedWorkUnits >= kMinParallelRefinementWorkUnits;
    if (!enoughCandidateCount && !enoughEstimatedWork) {
        return 1;
    }

    const int countLimitedThreadCount = candidateCount / kRefinementCandidatesPerWorker;
    const int workLimitedThreadCount = static_cast<int>(
        (estimatedWorkUnits + kRefinementWorkUnitsPerWorker - 1) / kRefinementWorkUnitsPerWorker);
    const int requestedThreadCount = std::max(countLimitedThreadCount, workLimitedThreadCount);
    if (requestedThreadCount < 2) {
        return 1;
    }

    return std::max(1, std::min({requestedThreadCount, candidateCount, idealThreadCount - 2}));
}

int CameraPlateSolver::SolverContext::visibleCatalogWorkerThreadCount(int catalogStarCount)
{
    static constexpr int kMinParallelVisibleCatalogStars = 120000;
    static constexpr int kVisibleCatalogStarsPerWorker = 80000;

    if (catalogStarCount < kMinParallelVisibleCatalogStars) {
        return 1;
    }

    const int idealThreadCount = QThread::idealThreadCount();
    if (idealThreadCount <= 2) {
        return 1;
    }

    const int requestedThreadCount = (catalogStarCount + kVisibleCatalogStarsPerWorker - 1) / kVisibleCatalogStarsPerWorker;
    return std::max(1, std::min(requestedThreadCount, idealThreadCount - 2));
}

double CameraPlateSolver::SolverContext::degToRad(double value)
{
    return value * kPi / 180.0;
}

double CameraPlateSolver::SolverContext::normalizeDegrees(double value)
{
    value = std::fmod(value, 360.0);
    if (value < 0.0) {
        value += 360.0;
    }
    return value;
}

CameraPlateSolver::SolverContext::SkyVector CameraPlateSolver::SolverContext::vectorFromAltAz(double azimuthDegrees, double elevationDegrees)
{
    const double azimuth = degToRad(azimuthDegrees);
    const double elevation = degToRad(elevationDegrees);
    const double cosElevation = std::cos(elevation);

    return {
        cosElevation * std::sin(azimuth),
        cosElevation * std::cos(azimuth),
        std::sin(elevation)
    };
}

double CameraPlateSolver::SolverContext::dot(const SkyVector& lhs, const SkyVector& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

CameraPlateSolver::SolverContext::SkyVector CameraPlateSolver::SolverContext::cross(const SkyVector& lhs, const SkyVector& rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x
    };
}

double CameraPlateSolver::SolverContext::length(const SkyVector& vector)
{
    return std::sqrt(dot(vector, vector));
}

CameraPlateSolver::SolverContext::SkyVector CameraPlateSolver::SolverContext::normalize(const SkyVector& vector)
{
    const double vectorLength = length(vector);
    if (vectorLength <= 0.0) {
        return {0.0, 0.0, 0.0};
    }

    return {
        vector.x / vectorLength,
        vector.y / vectorLength,
        vector.z / vectorLength
    };
}

CameraPlateSolver::SolverContext::SkyVector CameraPlateSolver::SolverContext::rotateAroundAxis(const SkyVector& vector, const SkyVector& axis, double angleRadians)
{
    const double cosAngle = std::cos(angleRadians);
    const double sinAngle = std::sin(angleRadians);
    const SkyVector axisCrossVector = cross(axis, vector);
    const double axisDotVector = dot(axis, vector);

    return {
        vector.x * cosAngle + axisCrossVector.x * sinAngle + axis.x * axisDotVector * (1.0 - cosAngle),
        vector.y * cosAngle + axisCrossVector.y * sinAngle + axis.y * axisDotVector * (1.0 - cosAngle),
        vector.z * cosAngle + axisCrossVector.z * sinAngle + axis.z * axisDotVector * (1.0 - cosAngle)
    };
}

double CameraPlateSolver::SolverContext::parseRightAscensionDegrees(const QString& value)
{
    const QStringList fields = value.split(' ', Qt::SkipEmptyParts);
    if (fields.size() != 3) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    bool okHours = false;
    bool okMinutes = false;
    bool okSeconds = false;
    const double hours = fields[0].toDouble(&okHours);
    const double minutes = fields[1].toDouble(&okMinutes);
    const double seconds = fields[2].toDouble(&okSeconds);
    if (!okHours || !okMinutes || !okSeconds) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return 15.0 * (hours + minutes / 60.0 + seconds / 3600.0);
}

double CameraPlateSolver::SolverContext::parseDeclinationDegrees(const QString& value)
{
    const QStringList fields = value.split(' ', Qt::SkipEmptyParts);
    if (fields.size() != 3) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    bool okDegrees = false;
    bool okMinutes = false;
    bool okSeconds = false;
    const double degreesWithSign = fields[0].toDouble(&okDegrees);
    const double minutes = fields[1].toDouble(&okMinutes);
    const double seconds = fields[2].toDouble(&okSeconds);
    if (!okDegrees || !okMinutes || !okSeconds) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Take the sign from the string, not the parsed double: a "-00 mm ss" degrees field parses to
    // -0.0, and -0.0 < 0.0 is false, so deriving the sign from the double drops the minus for
    // declinations in (-1, 0) deg (e.g. Mintaka, -00 17 56).
    const bool negative = fields[0].trimmed().startsWith(QLatin1Char('-'));
    const double sign = negative ? -1.0 : 1.0;
    const double absoluteDegrees = std::fabs(degreesWithSign);
    return sign * (absoluteDegrees + minutes / 60.0 + seconds / 3600.0);
}

QString CameraPlateSolver::SolverContext::stripQuotedField(const QString& value)
{
    QString stripped = value.trimmed();
    if (stripped.startsWith('"') && stripped.endsWith('"') && (stripped.size() >= 2)) {
        stripped = stripped.mid(1, stripped.size() - 2);
    }
    return stripped.replace(QStringLiteral("\"\""), QStringLiteral("\""));
}

QString CameraPlateSolver::SolverContext::downloadedCatalogDir()
{
    // Track 0a: point every catalog + Siril cache path at a versioned snapshot dir instead of the
    // per-user AppData location. Combined with SDRANGEL_CAMERA_PLATE_SOLVER_OFFLINE this makes a
    // solve fully reproducible from a frozen cache, so the harness is a hermetic gate. The override
    // is the equivalent of the AppData ".../camera" dir (it holds siril-spcc-cache/, the base
    // catalog files, etc.).
    const QByteArray overrideDir = qgetenv("SDRANGEL_CAMERA_PLATE_SOLVER_CACHE_DIR");
    if (!overrideDir.isEmpty()) {
        return QString::fromLocal8Bit(overrideDir);
    }
    const QString baseDir = QStandardPaths::standardLocations(QStandardPaths::AppDataLocation).value(0);
    return QDir(baseDir).filePath(QString::fromUtf8(kDownloadedCatalogDir));
}

QString CameraPlateSolver::SolverContext::downloadedCatalogReducedPath()
{
    return QDir(downloadedCatalogDir()).filePath(QString::fromUtf8(kDownloadedCatalogReducedFile));
}

QByteArray CameraPlateSolver::SolverContext::gunzipData(const QByteArray& compressedData, QString* errorMessage)
{
    if (compressedData.isEmpty()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Downloaded catalog archive is empty.");
        }
        return QByteArray();
    }

    z_stream stream;
    std::memset(&stream, 0, sizeof(stream));
    stream.next_in = reinterpret_cast<Bytef *>(const_cast<char *>(compressedData.constData()));
    stream.avail_in = static_cast<uInt>(compressedData.size());

    const int windowBits = 16 + MAX_WBITS;
    if (inflateInit2(&stream, windowBits) != Z_OK)
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to initialize gzip decompressor.");
        }
        return QByteArray();
    }

    QByteArray uncompressedData;
    char buffer[32768];
    int inflateStatus = Z_OK;
    do
    {
        stream.next_out = reinterpret_cast<Bytef *>(buffer);
        stream.avail_out = sizeof(buffer);
        inflateStatus = inflate(&stream, Z_NO_FLUSH);
        if ((inflateStatus != Z_OK) && (inflateStatus != Z_STREAM_END))
        {
            inflateEnd(&stream);
            if (errorMessage) {
                *errorMessage = QStringLiteral("Failed to decompress gzip catalog archive.");
            }
            return QByteArray();
        }
        uncompressedData.append(buffer, sizeof(buffer) - static_cast<int>(stream.avail_out));
    } while (inflateStatus != Z_STREAM_END);

    inflateEnd(&stream);
    return uncompressedData;
}

QVector<CameraPlateSolver::SolverContext::CatalogStar> CameraPlateSolver::SolverContext::parseBundledCatalog(const QString& text)
{
    QVector<CatalogStar> stars;
    const QStringList lines = text.split('\n');
    for (const QString& rawLine : lines)
    {
        const QString line = rawLine.trimmed();
        if (line.isEmpty() || line.startsWith('#') || line.startsWith(QStringLiteral("name|"))) {
            continue;
        }

        const QStringList fields = line.split('|');
        if ((fields.size() != 4) && (fields.size() != 5)) {
            continue;
        }

        bool okMagnitude = false;
        const double rightAscensionDegrees = parseRightAscensionDegrees(fields[1].trimmed());
        const double declinationDegrees = parseDeclinationDegrees(fields[2].trimmed());
        const double magnitude = fields[3].trimmed().toDouble(&okMagnitude);
        if (!std::isfinite(rightAscensionDegrees) || !std::isfinite(declinationDegrees) || !okMagnitude) {
            continue;
        }

        stars.append({
            fields[0].trimmed(),
            rightAscensionDegrees,
            declinationDegrees,
            magnitude,
            (fields.size() >= 5) ? fields[4].trimmed() : QString()
        });
    }

    return stars;
}

QVector<CameraPlateSolver::SolverContext::CatalogStar> CameraPlateSolver::SolverContext::parseDownloadedHygCatalog(const QString& text)
{
    QVector<CatalogStar> stars;
    const QStringList lines = text.split('\n');
    if (lines.isEmpty()) {
        return stars;
    }

    const QStringList headers = lines.front().trimmed().split(',');
    QHash<QString, int> indices;
    for (int i = 0; i < headers.size(); ++i) {
        indices.insert(stripQuotedField(headers[i]), i);
    }

    const int properIndex = indices.value(QStringLiteral("proper"), -1);
    const int bfIndex = indices.value(QStringLiteral("bf"), -1);
    const int raIndex = indices.value(QStringLiteral("ra"), -1);
    const int decIndex = indices.value(QStringLiteral("dec"), -1);
    const int magIndex = indices.value(QStringLiteral("mag"), -1);
    const int hipIndex = indices.value(QStringLiteral("hip"), -1);
    const int hdIndex = indices.value(QStringLiteral("hd"), -1);
    const int hrIndex = indices.value(QStringLiteral("hr"), -1);
    const int spectIndex = indices.value(QStringLiteral("spect"), -1);
    const int bayerIndex = indices.value(QStringLiteral("bayer"), -1);
    const int flamIndex = indices.value(QStringLiteral("flam"), -1);
    const int conIndex = indices.value(QStringLiteral("con"), -1);

    if ((raIndex < 0) || (decIndex < 0) || (magIndex < 0)) {
        return stars;
    }

    stars.reserve(lines.size());
    for (int lineIndex = 1; lineIndex < lines.size(); ++lineIndex)
    {
        const QString line = lines[lineIndex].trimmed();
        if (line.isEmpty()) {
            continue;
        }

        const QStringList fields = line.split(',');
        if (fields.size() <= std::max({raIndex, decIndex, magIndex})) {
            continue;
        }

        bool okRa = false;
        bool okDec = false;
        bool okMag = false;
        const double rightAscensionHours = stripQuotedField(fields[raIndex]).toDouble(&okRa);
        const double declinationDegrees = stripQuotedField(fields[decIndex]).toDouble(&okDec);
        const double magnitude = stripQuotedField(fields[magIndex]).toDouble(&okMag);
        if (!okRa || !okDec || !okMag) {
            continue;
        }

        QString name;
        if ((properIndex >= 0) && (properIndex < fields.size())) {
            name = stripQuotedField(fields[properIndex]);
        }
        if (name.isEmpty() && (bfIndex >= 0) && (bfIndex < fields.size())) {
            name = stripQuotedField(fields[bfIndex]);
        }
        if (name.isEmpty())
        {
            const QString bayer = ((bayerIndex >= 0) && (bayerIndex < fields.size())) ? stripQuotedField(fields[bayerIndex]) : QString();
            const QString flam = ((flamIndex >= 0) && (flamIndex < fields.size())) ? stripQuotedField(fields[flamIndex]) : QString();
            const QString constellation = ((conIndex >= 0) && (conIndex < fields.size())) ? stripQuotedField(fields[conIndex]) : QString();
            if (!bayer.isEmpty() && !constellation.isEmpty()) {
                name = QStringLiteral("%1 %2").arg(bayer, constellation);
            } else if (!flam.isEmpty() && !constellation.isEmpty()) {
                name = QStringLiteral("%1 %2").arg(flam, constellation);
            }
        }
        if (name.isEmpty() && (hipIndex >= 0) && (hipIndex < fields.size()))
        {
            const QString hip = stripQuotedField(fields[hipIndex]);
            if (!hip.isEmpty()) {
                name = QStringLiteral("HIP %1").arg(hip);
            }
        }
        if (name.isEmpty() && (hrIndex >= 0) && (hrIndex < fields.size()))
        {
            const QString hr = stripQuotedField(fields[hrIndex]);
            if (!hr.isEmpty()) {
                name = QStringLiteral("HR %1").arg(hr);
            }
        }
        if (name.isEmpty() && (hdIndex >= 0) && (hdIndex < fields.size()))
        {
            const QString hd = stripQuotedField(fields[hdIndex]);
            if (!hd.isEmpty()) {
                name = QStringLiteral("HD %1").arg(hd);
            }
        }
        if (name.isEmpty()) {
            continue;
        }

        const QString spectralType = ((spectIndex >= 0) && (spectIndex < fields.size()))
            ? stripQuotedField(fields[spectIndex]) : QString();

        stars.append({name, rightAscensionHours * 15.0, declinationDegrees, magnitude, spectralType});
    }

    return stars;
}

QString CameraPlateSolver::SolverContext::formatRightAscensionHours(double rightAscensionDegrees)
{
    const double totalHours = normalizeDegrees(rightAscensionDegrees) / 15.0;
    int hours = static_cast<int>(std::floor(totalHours));
    const double totalMinutes = (totalHours - hours) * 60.0;
    int minutes = static_cast<int>(std::floor(totalMinutes));
    double seconds = (totalMinutes - minutes) * 60.0;
    // Carry if seconds would round up to 60 at the formatted precision (5 dp), so the field is
    // never the invalid "60.00000".
    if (seconds >= 60.0 - 0.5e-5)
    {
        seconds = 0.0;
        if (++minutes >= 60)
        {
            minutes = 0;
            if (++hours >= 24) {
                hours = 0;
            }
        }
    }
    return QStringLiteral("%1 %2 %3")
        .arg(hours, 2, 10, QLatin1Char('0'))
        .arg(minutes, 2, 10, QLatin1Char('0'))
        .arg(seconds, 0, 'f', 5);
}

QString CameraPlateSolver::SolverContext::formatDeclinationDegrees(double declinationDegrees)
{
    const QChar sign = (declinationDegrees < 0.0) ? QLatin1Char('-') : QLatin1Char('+');
    const double absoluteDegrees = std::fabs(declinationDegrees);
    int degrees = static_cast<int>(std::floor(absoluteDegrees));
    const double totalMinutes = (absoluteDegrees - degrees) * 60.0;
    int minutes = static_cast<int>(std::floor(totalMinutes));
    double seconds = (totalMinutes - minutes) * 60.0;
    // Carry if seconds would round up to 60 at the formatted precision (4 dp).
    if (seconds >= 60.0 - 0.5e-4)
    {
        seconds = 0.0;
        if (++minutes >= 60)
        {
            minutes = 0;
            ++degrees;
        }
    }
    return QStringLiteral("%1%2 %3 %4")
        .arg(sign)
        .arg(degrees, 2, 10, QLatin1Char('0'))
        .arg(minutes, 2, 10, QLatin1Char('0'))
        .arg(seconds, 0, 'f', 4);
}

bool CameraPlateSolver::SolverContext::writeReducedCatalog(const QVector<CatalogStar>& stars, const QString& path, QString* errorMessage)
{
    // QSaveFile (write-to-temp + atomic rename) so concurrent solver processes can't read a
    // half-written reduced catalog or corrupt each other when both regenerate it at once;
    // concurrent writers resolve to last-writer-wins with each version complete.
    QSaveFile outputFile(path);
    if (!outputFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to write reduced HYG catalog: %1").arg(path);
        }
        return false;
    }

    QByteArray data;
    data.append("name|ra_hms|dec_dms|magnitude|spect\n");
    for (const CatalogStar& star : stars)
    {
        data.append(star.name.toUtf8());
        data.append('|');
        data.append(formatRightAscensionHours(star.rightAscensionDegrees).toUtf8());
        data.append('|');
        data.append(formatDeclinationDegrees(star.declinationDegrees).toUtf8());
        data.append('|');
        data.append(QByteArray::number(star.magnitude, 'f', 2));
        data.append('|');
        data.append(star.spectralType.toUtf8());
        data.append('\n');
    }

    if (outputFile.write(data) != data.size())
    {
        outputFile.cancelWriting();
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to fully write reduced HYG catalog: %1").arg(path);
        }
        return false;
    }

    if (!outputFile.commit())
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to commit reduced HYG catalog: %1").arg(path);
        }
        return false;
    }
    return true;
}

QVector<CameraPlateSolver::SolverContext::CatalogStar> CameraPlateSolver::SolverContext::loadCatalogFromTextFile(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return {};
    }

    const QString text = QString::fromUtf8(file.readAll());
    if (path.endsWith(QStringLiteral(".txt"))) {
        return parseBundledCatalog(text);
    }
    return parseDownloadedHygCatalog(text);
}

QString CameraPlateSolver::SolverContext::currentCatalogPath(const CameraSettings& settings)
{
    (void) settings;
    if (QFileInfo::exists(downloadedCatalogReducedPath())) {
        return downloadedCatalogReducedPath();
    }
    return QString::fromUtf8(kBundledCatalogPath);
}

QString CameraPlateSolver::SolverContext::currentCatalogSource(const CameraSettings& settings)
{
    (void) settings;
    return QFileInfo::exists(downloadedCatalogReducedPath())
        ? QStringLiteral("HYG")
        : QStringLiteral("Bundled");
}

double CameraPlateSolver::SolverContext::firstPassPlateSolveMaxMagnitude(const CameraSettings& settings)
{
    if (plateSolveStartUsesDirection(settings)
        && (isNarrowField(settings))
        && (settings.m_plateSolveMaxMagnitude > kNarrowGuidedBrightCatalogMaxMagnitude))
    {
        return kNarrowGuidedBrightCatalogMaxMagnitude;
    }

    if (isWidePlateSolveContext(settings)
        && (settings.m_plateSolveMaxMagnitude > kWideFovBrightFirstPassMaxMagnitude))
    {
        return kWideFovBrightFirstPassMaxMagnitude;
    }

    return settings.m_plateSolveMaxMagnitude;
}

double CameraPlateSolver::SolverContext::narrowGuidedFullSearchMaxMagnitude(const CameraSettings& settings)
{
    if (isNarrowGuidedDirectionSolve(settings))
    {
        // Mag 18 is a good Gaia depth for narrow guided solving: enough stars
        // for galaxy fields without letting dense faint stars dominate matches.
        return std::min(static_cast<double>(settings.m_plateSolveMaxMagnitude), 18.0);
    }

    return settings.m_plateSolveMaxMagnitude;
}

bool CameraPlateSolver::SolverContext::isLowMagnitudeNarrowGuidedSolve(const CameraSettings& settings)
{
    return plateSolveStartUsesDirection(settings)
        && (isNarrowField(settings))
        && (settings.m_plateSolveMaxMagnitude <= 17.0);
}

double CameraPlateSolver::SolverContext::fovLongEdgeDiagonalDegrees(const QSize& imageSize, double fovDegrees)
{
    const int longEdge = std::max(imageSize.width(), imageSize.height());
    const int shortEdge = std::min(imageSize.width(), imageSize.height());
    if ((longEdge <= 0) || (shortEdge <= 0)) {
        return fovDegrees;
    }

    const double shortToLong = static_cast<double>(shortEdge) / static_cast<double>(longEdge);
    return fovDegrees * std::sqrt(1.0 + shortToLong * shortToLong);
}

double CameraPlateSolver::SolverContext::halfHorizontalFovFromLongEdgeFov(CameraSettings::LensProjection lensProjection, const QSize& imageSize, double fovDegrees)
{
    const double halfLongEdgeFov = degToRad(fovDegrees) * 0.5;
    if ((imageSize.width() <= 0) || (imageSize.height() <= 0) || (imageSize.width() >= imageSize.height())) {
        return halfLongEdgeFov;
    }

    const double aspect = static_cast<double>(imageSize.height()) / static_cast<double>(imageSize.width());
    switch (lensProjection)
    {
    case CameraSettings::LensProjectionEquidistant:
        return halfLongEdgeFov / aspect;
    case CameraSettings::LensProjectionEquisolid:
        return 2.0 * std::asin(std::clamp(std::sin(halfLongEdgeFov * 0.5) / aspect, -1.0, 1.0));
    case CameraSettings::LensProjectionRectilinear:
    default:
        return std::atan(std::tan(halfLongEdgeFov) / aspect);
    }
}

quint32 CameraPlateSolver::SolverContext::interleaveHealpixBits(int x, int y)
{
    quint32 pixel = 0;
    for (int bit = 0; bit < kSirilHealpixLevel; ++bit)
    {
        pixel |= ((static_cast<quint32>(x) >> bit) & 1u) << (2 * bit);
        pixel |= ((static_cast<quint32>(y) >> bit) & 1u) << (2 * bit + 1);
    }
    return pixel;
}

quint32 CameraPlateSolver::SolverContext::healpixNestedPixel(double rightAscensionDegrees, double declinationDegrees)
{
    const double z = std::sin(degToRad(declinationDegrees));
    const double za = std::fabs(z);
    double tt = normalizeDegrees(rightAscensionDegrees) / 90.0;
    if (tt >= 4.0) {
        tt -= 4.0;
    }

    int face = 0;
    int ix = 0;
    int iy = 0;
    if (za <= (2.0 / 3.0))
    {
        const int jp = static_cast<int>(std::floor(kSirilNside * (0.5 + tt - z * 0.75)));
        const int jm = static_cast<int>(std::floor(kSirilNside * (0.5 + tt + z * 0.75)));
        const int ifp = jp / kSirilNside;
        const int ifm = jm / kSirilNside;
        face = (ifp == ifm) ? (ifp | 4) : ((ifp < ifm) ? ifp : ifm + 8);
        ix = jm & (kSirilNside - 1);
        iy = kSirilNside - (jp & (kSirilNside - 1)) - 1;
    }
    else
    {
        int ntt = static_cast<int>(std::floor(tt));
        if (ntt >= 4) {
            ntt = 3;
        }
        const double tp = tt - ntt;
        const double tmp = kSirilNside * std::sqrt(3.0 * (1.0 - za));
        const int jp = std::min(kSirilNside - 1, static_cast<int>(std::floor(tp * tmp)));
        const int jm = std::min(kSirilNside - 1, static_cast<int>(std::floor((1.0 - tp) * tmp)));
        if (z >= 0.0)
        {
            face = ntt;
            ix = kSirilNside - jm - 1;
            iy = kSirilNside - jp - 1;
        }
        else
        {
            face = ntt + 8;
            ix = jp;
            iy = jm;
        }
    }

    return static_cast<quint32>(face * kSirilNside * kSirilNside) + interleaveHealpixBits(ix, iy);
}

double CameraPlateSolver::SolverContext::angularSeparationDegrees(double raDegreesA, double decDegreesA, double raDegreesB, double decDegreesB)
{
    const double raA = degToRad(raDegreesA);
    const double decA = degToRad(decDegreesA);
    const double raB = degToRad(raDegreesB);
    const double decB = degToRad(decDegreesB);
    const double cosSeparation = std::sin(decA) * std::sin(decB)
        + std::cos(decA) * std::cos(decB) * std::cos(raA - raB);
    return std::acos(std::clamp(cosSeparation, -1.0, 1.0)) * 180.0 / kPi;
}

bool CameraPlateSolver::SolverContext::plateSolveStartUsesFov(const CameraSettings& settings)
{
    return settings.m_plateSolveStartMode != CameraSettings::PlateSolveStartBlind;
}

bool CameraPlateSolver::SolverContext::plateSolveStartUsesCurrentSettingsOnly(const CameraSettings& settings)
{
    return settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartCurrentSettingsOnly;
}

bool CameraPlateSolver::SolverContext::usesSeedProjectedBrightGate(const CameraSettings& settings)
{
    return plateSolveStartUsesRoll(settings) || plateSolveStartUsesCurrentSettingsOnly(settings);
}

bool CameraPlateSolver::SolverContext::plateSolveStartUsesElevation(const CameraSettings& settings)
{
    switch (settings.m_plateSolveStartMode)
    {
    case CameraSettings::PlateSolveStartFovElevation:
    case CameraSettings::PlateSolveStartFovAzEl:
    case CameraSettings::PlateSolveStartFovAzElRoll:
    case CameraSettings::PlateSolveStartFovAzElRollLens:
    case CameraSettings::PlateSolveStartCurrentSettingsOnly:
        return true;
    default:
        return false;
    }
}

bool CameraPlateSolver::SolverContext::plateSolveStartUsesDirection(const CameraSettings& settings)
{
    switch (settings.m_plateSolveStartMode)
    {
    case CameraSettings::PlateSolveStartFovAzEl:
    case CameraSettings::PlateSolveStartFovAzElRoll:
    case CameraSettings::PlateSolveStartFovAzElRollLens:
    case CameraSettings::PlateSolveStartCurrentSettingsOnly:
        return true;
    default:
        return false;
    }
}

bool CameraPlateSolver::SolverContext::plateSolveStartUsesRoll(const CameraSettings& settings)
{
    switch (settings.m_plateSolveStartMode)
    {
    case CameraSettings::PlateSolveStartFovAzElRoll:
    case CameraSettings::PlateSolveStartFovAzElRollLens:
    case CameraSettings::PlateSolveStartCurrentSettingsOnly:
        return true;
    default:
        return false;
    }
}

bool CameraPlateSolver::SolverContext::plateSolveStartUsesLens(const CameraSettings& settings)
{
    switch (settings.m_plateSolveStartMode)
    {
    case CameraSettings::PlateSolveStartFovAzElRollLens:
    case CameraSettings::PlateSolveStartCurrentSettingsOnly:
        return true;
    default:
        return false;
    }
}

bool CameraPlateSolver::SolverContext::isWidePlateSolveContext(const CameraSettings& settings)
{
    return (settings.m_lensProjection != CameraSettings::LensProjectionRectilinear)
        && ((settings.m_fov >= kWideFovMagnitudePreferenceThresholdDegrees)
            || (settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartBlind));
}

bool CameraPlateSolver::SolverContext::isNarrowField(const CameraSettings& settings)
{
    return settings.m_fov <= kNarrowFieldMaxFovDegrees;
}

bool CameraPlateSolver::SolverContext::isNarrowGuidedDirectionSolve(const CameraSettings& settings)
{
    return plateSolveStartUsesDirection(settings) && isNarrowField(settings);
}

bool CameraPlateSolver::SolverContext::seedFovCompatibleWithStartFov(const CameraSettings& settings, double seedFovDegrees)
{
    if (!plateSolveStartUsesFov(settings)) {
        return true;
    }

    const double referenceFov = std::clamp(
        static_cast<double>(settings.m_fov),
        static_cast<double>(CameraSettings::m_minFov),
        static_cast<double>(CameraSettings::m_maxFov));
    const bool wideFisheye = isWidePlateSolveContext(settings);
    const double toleranceDegrees = wideFisheye
        ? std::max(30.0, referenceFov * 0.35)
        : std::max(0.20, referenceFov * 0.35);

    return std::fabs(seedFovDegrees - referenceFov) <= toleranceDegrees;
}

bool CameraPlateSolver::SolverContext::canCalibrateLens(const CameraSettings& settings)
{
    return (settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartFovAzElRollLens)
        || isWidePlateSolveContext(settings);
}

bool CameraPlateSolver::SolverContext::canCalibratePrincipalPoint(const CameraSettings& settings)
{
    return (settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartFovAzElRollLens)
        || isWidePlateSolveContext(settings);
}

QVector<CameraPlateSolver::SolverContext::CatalogStar> CameraPlateSolver::SolverContext::brightStarCatalog(const CameraSettings& settings)
{
    static QMutex s_catalogMutex;
    static QString s_loadedPath;
    static QDateTime s_loadedModified;
    static QVector<CatalogStar> s_catalog;

    const QString path = currentCatalogPath(settings);
    const bool isResource = path.startsWith(QLatin1String(":/"));
    const QDateTime modified = isResource ? QDateTime() : QFileInfo(path).lastModified();

    QMutexLocker locker(&s_catalogMutex);
    if ((path != s_loadedPath) || (!isResource && (modified != s_loadedModified)) || s_catalog.isEmpty())
    {
        s_catalog = loadCatalogFromTextFile(path);
        s_loadedPath = path;
        s_loadedModified = modified;
    }

    return s_catalog;
}

QVector<CameraPlateSolver::SolverContext::CatalogStar> CameraPlateSolver::SolverContext::bundledAliasCatalog()
{
    static QMutex s_catalogMutex;
    static QVector<CatalogStar> s_catalog;

    QMutexLocker locker(&s_catalogMutex);
    if (s_catalog.isEmpty())
    {
        if (QFileInfo::exists(downloadedCatalogReducedPath())) {
            s_catalog = loadCatalogFromTextFile(downloadedCatalogReducedPath());
        }
        if (s_catalog.isEmpty()) {
            s_catalog = loadCatalogFromTextFile(QString::fromUtf8(kBundledCatalogPath));
        }
    }
    return s_catalog;
}

QVector<int> CameraPlateSolver::SolverContext::aliasCatalogDeclinationSortedIndices(const QVector<CatalogStar>& aliasStars)
{
    static QMutex s_aliasIndexMutex;
    static const QVector<CatalogStar>* s_aliasStars = nullptr;
    static const CatalogStar* s_aliasStarsData = nullptr;
    static int s_aliasStarsSize = 0;
    static QVector<int> s_sortedIndices;

    QMutexLocker locker(&s_aliasIndexMutex);
    if ((s_aliasStars != &aliasStars)
        || (s_aliasStarsData != aliasStars.constData())
        || (s_aliasStarsSize != aliasStars.size())
        || s_sortedIndices.isEmpty())
    {
        s_aliasStars = &aliasStars;
        s_aliasStarsData = aliasStars.constData();
        s_aliasStarsSize = aliasStars.size();
        s_sortedIndices.resize(aliasStars.size());
        for (int i = 0; i < aliasStars.size(); ++i) {
            s_sortedIndices[i] = i;
        }
        std::sort(s_sortedIndices.begin(), s_sortedIndices.end(), [&aliasStars](int lhs, int rhs) {
            return aliasStars[lhs].declinationDegrees < aliasStars[rhs].declinationDegrees;
        });
    }

    return s_sortedIndices;
}

QString CameraPlateSolver::SolverContext::resolveNamedAliasForCatalogStar(const CatalogStar& star, const QVector<CatalogStar>& aliasStars, const QVector<int>& sortedAliasIndices, double* outScore)
{
    if (outScore) {
        *outScore = std::numeric_limits<double>::infinity();
    }

    if (aliasStars.isEmpty() || sortedAliasIndices.isEmpty()) {
        return QString();
    }

    const double maxSeparationDegrees = kSirilAliasMaxSeparationArcSec / 3600.0;
    const double minDeclination = star.declinationDegrees - maxSeparationDegrees;
    const double maxDeclination = star.declinationDegrees + maxSeparationDegrees;
    const auto first = std::lower_bound(
        sortedAliasIndices.cbegin(),
        sortedAliasIndices.cend(),
        minDeclination,
        [&aliasStars](int catalogIndex, double declination) {
            return aliasStars[catalogIndex].declinationDegrees < declination;
        });

    QString bestName;
    double bestScore = std::numeric_limits<double>::infinity();

    for (auto it = first; it != sortedAliasIndices.cend(); ++it)
    {
        const CatalogStar& candidate = aliasStars[*it];
        if (candidate.declinationDegrees > maxDeclination) {
            break;
        }

        if (candidate.name.isEmpty()) {
            continue;
        }

        const double magnitudeDifference = std::fabs(candidate.magnitude - star.magnitude);
        if (magnitudeDifference > kSirilAliasMaxMagnitudeDifference) {
            continue;
        }

        const double separationArcSec = angularSeparationDegrees(
            star.rightAscensionDegrees,
            star.declinationDegrees,
            candidate.rightAscensionDegrees,
            candidate.declinationDegrees) * 3600.0;
        if (separationArcSec > kSirilAliasMaxSeparationArcSec) {
            continue;
        }

        const double score = separationArcSec + magnitudeDifference * 4.0;
        if (score < bestScore)
        {
            const QString candidateName = candidate.name.trimmed();
            if (candidateName.isEmpty()) {
                continue;
            }
            bestScore = score;
            bestName = candidateName;
        }
    }

    if (outScore && !bestName.isEmpty()) {
        *outScore = bestScore;
    }

    return bestName;
}

void CameraPlateSolver::SolverContext::applyNamedAliasesToCatalog(const CameraSettings& settings, QVector<CatalogStar>& stars)
{
    Q_UNUSED(settings)

    if (stars.isEmpty()) {
        return;
    }

    const QVector<CatalogStar> aliasStars = bundledAliasCatalog();
    const QVector<int> sortedAliasIndices = aliasCatalogDeclinationSortedIndices(aliasStars);

    // Resolve alias names as a one-to-one assignment: each named alias (e.g. "HIP 37078")
    // must end up on at most one catalog star -- the single closest/best-scoring match.
    //
    // Resolving independently per catalog star (the previous approach: "for every Gaia star,
    // find its best alias and stamp the name on") is a many-to-one match. With a 30 arcsec /
    // 2.5 magnitude tolerance, several distinct nearby Gaia DR3 sources can each independently
    // consider the same bright named star their best (or only) alias candidate, so the same
    // name (e.g. "HIP 37078") ends up stamped onto multiple catalog entries scattered around
    // the true star's position. That confuses anchor-based searches, which key off named
    // stars and implicitly assume a name identifies a single, unique catalog entry -- the
    // search can latch onto a spurious duplicate (wrong position, "right" name) instead of
    // the genuine star.
    //
    // To keep the assignment unique, gather every candidate (catalogIndex, alias, score) and
    // keep only the best-scoring catalog star for each alias name.
    QHash<QString, int> bestIndexForAlias;
    QHash<QString, double> bestScoreForAlias;

    for (int i = 0; i < stars.size(); ++i)
    {
        double score = std::numeric_limits<double>::infinity();
        const QString alias = resolveNamedAliasForCatalogStar(stars[i], aliasStars, sortedAliasIndices, &score);
        if (alias.isEmpty()) {
            continue;
        }

        const auto existing = bestScoreForAlias.constFind(alias);
        if ((existing == bestScoreForAlias.constEnd()) || (score < existing.value()))
        {
            bestIndexForAlias.insert(alias, i);
            bestScoreForAlias.insert(alias, score);
        }
    }

    int aliasCount = 0;
    for (auto it = bestIndexForAlias.constBegin(); it != bestIndexForAlias.constEnd(); ++it)
    {
        stars[it.value()].name = it.key();
        ++aliasCount;
    }

    if (aliasCount > 0) {
        qDebug() << "CameraPlateSolver: applied bundled catalog aliases" << aliasCount;
    }
}

double CameraPlateSolver::SolverContext::catalogAngularSeparationDegrees(const CatalogStar& lhs, const CatalogStar& rhs)
{
    const double lhsRa = degToRad(lhs.rightAscensionDegrees);
    const double lhsDec = degToRad(lhs.declinationDegrees);
    const double rhsRa = degToRad(rhs.rightAscensionDegrees);
    const double rhsDec = degToRad(rhs.declinationDegrees);
    const double sinLhsDec = std::sin(lhsDec);
    const double cosLhsDec = std::cos(lhsDec);
    const double sinRhsDec = std::sin(rhsDec);
    const double cosRhsDec = std::cos(rhsDec);
    const double cosDeltaRa = std::cos(lhsRa - rhsRa);
    const double dotProduct = std::clamp(
        sinLhsDec * sinRhsDec + cosLhsDec * cosRhsDec * cosDeltaRa,
        -1.0,
        1.0);
    return std::acos(dotProduct) * 180.0 / kPi;
}

void CameraPlateSolver::SolverContext::mergeBundledBrightStarsIntoCatalog(const CameraSettings& settings, QVector<CatalogStar>& catalogStars, double maxMagnitude, double centerRaDegrees, double centerDecDegrees, double queryRadiusDegrees)
{
    const QVector<CatalogStar> brightStars = brightStarCatalog(settings);
    const bool narrowDirectionSolve = plateSolveStartUsesDirection(settings)
        && (isNarrowField(settings));
    const double mergeMaxMagnitude = std::min(
        maxMagnitude,
        narrowDirectionSolve ? 10.5 : 7.0);
    const double duplicateRadiusDegrees = 60.0 / 3600.0;
    const bool filterToQueryRadius = std::isfinite(centerRaDegrees)
        && std::isfinite(centerDecDegrees)
        && std::isfinite(queryRadiusDegrees)
        && (queryRadiusDegrees > 0.0);
    int addedCount = 0;
    int updatedCount = 0;
    const int initialCatalogSize = catalogStars.size();
    QVector<int> catalogIndicesByDeclination;
    catalogIndicesByDeclination.resize(initialCatalogSize);
    for (int i = 0; i < initialCatalogSize; ++i) {
        catalogIndicesByDeclination[i] = i;
    }
    std::sort(catalogIndicesByDeclination.begin(), catalogIndicesByDeclination.end(), [&catalogStars](int lhs, int rhs) {
        return catalogStars[lhs].declinationDegrees < catalogStars[rhs].declinationDegrees;
    });
    QVector<int> appendedCatalogIndices;

    for (const CatalogStar& brightStar : brightStars)
    {
        if (brightStar.magnitude > mergeMaxMagnitude) {
            continue;
        }
        if (filterToQueryRadius
            && (angularSeparationDegrees(
                    centerRaDegrees,
                    centerDecDegrees,
                    brightStar.rightAscensionDegrees,
                    brightStar.declinationDegrees) > (queryRadiusDegrees + duplicateRadiusDegrees)))
        {
            continue;
        }

        int duplicateIndex = -1;
        const double minDeclination = brightStar.declinationDegrees - duplicateRadiusDegrees;
        const double maxDeclination = brightStar.declinationDegrees + duplicateRadiusDegrees;
        const auto first = std::lower_bound(
            catalogIndicesByDeclination.cbegin(),
            catalogIndicesByDeclination.cend(),
            minDeclination,
            [&catalogStars](int catalogIndex, double declination) {
                return catalogStars[catalogIndex].declinationDegrees < declination;
            });
        for (auto it = first; it != catalogIndicesByDeclination.cend(); ++it)
        {
            const int catalogIndex = *it;
            if (catalogStars[catalogIndex].declinationDegrees > maxDeclination) {
                break;
            }
            if (catalogAngularSeparationDegrees(brightStar, catalogStars[catalogIndex]) <= duplicateRadiusDegrees)
            {
                if ((duplicateIndex < 0) || (catalogIndex < duplicateIndex)) {
                    duplicateIndex = catalogIndex;
                }
            }
        }
        if (duplicateIndex < 0)
        {
            for (int catalogIndex : appendedCatalogIndices)
            {
                if (catalogAngularSeparationDegrees(brightStar, catalogStars[catalogIndex]) <= duplicateRadiusDegrees)
                {
                    duplicateIndex = catalogIndex;
                    break;
                }
            }
        }

        if (duplicateIndex >= 0)
        {
            CatalogStar& existing = catalogStars[duplicateIndex];
            const bool existingHasGenericName = existing.name.isEmpty() || isGenericGaiaCatalogName(existing.name);
            if (existingHasGenericName && !brightStar.name.isEmpty())
            {
                existing.name = brightStar.name;
                existing.spectralType = brightStar.spectralType;
                ++updatedCount;
            }
            if (brightStar.magnitude < existing.magnitude)
            {
                existing.name = brightStar.name;
                existing.magnitude = brightStar.magnitude;
                existing.spectralType = brightStar.spectralType;
                ++updatedCount;
            }
        }
        else
        {
            appendedCatalogIndices.append(catalogStars.size());
            catalogStars.append(brightStar);
            ++addedCount;
        }
    }

    if ((addedCount > 0) || (updatedCount > 0))
    {
        qDebug() << "CameraPlateSolver: merged bundled bright stars into catalog"
                 << "added" << addedCount
                 << "updated" << updatedCount;
    }
}

QString CameraPlateSolver::SolverContext::matchSummary(const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<Match>& matches)
{
    QStringList parts;
    parts.reserve(matches.size());
    for (const Match& match : matches)
    {
        if ((match.detectionIndex < 0)
            || (match.detectionIndex >= starDetections.size())
            || (match.catalogIndex < 0)
            || (match.catalogIndex >= catalogContext.catalogStars.size()))
        {
            continue;
        }

        const CameraPipelineStarDetection& detection = starDetections[match.detectionIndex];
        const CatalogStar& star = catalogContext.catalogStars[match.catalogIndex];
        parts.append(QStringLiteral("#%1 %2 d=%3 px mag=%4 snr=%5 fwhm=%6")
            .arg(match.detectionIndex)
            .arg(catalogDisplayName(star))
            .arg(match.distancePixels, 0, 'f', 2)
            .arg(star.magnitude, 0, 'f', 2)
            .arg(detection.m_snr, 0, 'f', 1)
            .arg(detection.m_fwhm, 0, 'f', 2));
    }
    return parts.join(QStringLiteral("; "));
}

void CameraPlateSolver::SolverContext::repairFinalMatchCollisions(const QVector<CameraPipelineStarDetection>& starDetections, const QVector<ProjectedCatalogStar>& projectedStars, double matchRadius, QVector<Match>& matches)
{
    constexpr double kMinSnrForRepair = 50.0;
    // Strict condition: unmatched detection is CLOSER than current match, same-or-brighter.
    constexpr double kSnrRatioStrict = 1.0;
    // Relaxed condition: unmatched detection is within 1.5x current-match distance and 2x brighter.
    // Handles cases where a bright star loses to a dimmer one due to BRE scoring, even when
    // the bright star is slightly farther from the projected catalog position.
    constexpr double kDistRatioRelaxed = 1.5;
    constexpr double kSnrRatioRelaxed = 2.0;
    constexpr double kMinAbsoluteSnrRelaxed = 150.0;
    const double matchRadiusSq = matchRadius * matchRadius;

    QSet<int> matchedDetections;
    QHash<int, int> catalogToMatchIdx;
    matchedDetections.reserve(matches.size());
    catalogToMatchIdx.reserve(matches.size());
    for (int i = 0; i < matches.size(); ++i) {
        matchedDetections.insert(matches[i].detectionIndex);
        catalogToMatchIdx.insert(matches[i].catalogIndex, i);
    }

    struct SwapCandidate {
        // Strict candidates sort before relaxed; within each tier, lower distanceRatio wins.
        bool relaxed;
        double distanceRatio;   // newDist / currentDist
        int detectionIndex;
        int catalogIndex;
        double newDistance;
    };
    QVector<SwapCandidate> candidates;

    for (int detIdx = 0; detIdx < starDetections.size(); ++detIdx) {
        if (matchedDetections.contains(detIdx)) continue;
        const CameraPipelineStarDetection& det = starDetections[detIdx];
        if (det.m_snr < kMinSnrForRepair || det.m_hotPixelSuspect) continue;

        double nearestDist = std::numeric_limits<double>::infinity();
        int nearestCatalogIdx = -1;
        for (const ProjectedCatalogStar& proj : projectedStars) {
            const double dx = det.m_center.x() - proj.point.x();
            const double dy = det.m_center.y() - proj.point.y();
            const double distSq = dx*dx + dy*dy;
            if (distSq > matchRadiusSq) continue;
            const double dist = std::sqrt(distSq);
            if (dist < nearestDist) { nearestDist = dist; nearestCatalogIdx = proj.catalogIndex; }
        }
        if (nearestCatalogIdx < 0) continue;

        const auto it = catalogToMatchIdx.constFind(nearestCatalogIdx);
        if (it == catalogToMatchIdx.constEnd()) continue;

        const Match& current = matches[it.value()];
        const double distRatio = nearestDist / current.distancePixels;
        const double snrRatio = det.m_snr / starDetections[current.detectionIndex].m_snr;

        if (distRatio < 1.0 && snrRatio >= kSnrRatioStrict) {
            candidates.append({false, distRatio, detIdx, nearestCatalogIdx, nearestDist});
        } else if (distRatio <= kDistRatioRelaxed
                   && snrRatio >= kSnrRatioRelaxed
                   && det.m_snr >= kMinAbsoluteSnrRelaxed) {
            candidates.append({true, distRatio, detIdx, nearestCatalogIdx, nearestDist});
        }
    }

    if (candidates.isEmpty()) return;

    std::sort(candidates.begin(), candidates.end(),
              [](const SwapCandidate& a, const SwapCandidate& b) {
                  if (a.relaxed != b.relaxed) return !a.relaxed;  // strict first
                  return a.distanceRatio < b.distanceRatio;
              });

    QSet<int> usedDetections;
    QSet<int> usedCatalogs;
    for (const SwapCandidate& cand : candidates) {
        if (usedDetections.contains(cand.detectionIndex)) continue;
        // Guard the catalog star too: without this, two candidates targeting the same catalog
        // star both apply, and the second overwrites the first's match -- unseating the first's
        // detection, which stays flagged used but ends up matched to nothing. Candidates are
        // sorted best-first (strict before relaxed, then ascending distance ratio), so the first
        // claim on a catalog star is the one to keep.
        if (usedCatalogs.contains(cand.catalogIndex)) continue;
        const int matchIdx = catalogToMatchIdx.value(cand.catalogIndex, -1);
        if (matchIdx < 0) continue;
        matches[matchIdx].detectionIndex = cand.detectionIndex;
        matches[matchIdx].distancePixels = cand.newDistance;
        usedDetections.insert(cand.detectionIndex);
        usedCatalogs.insert(cand.catalogIndex);
    }
}

void CameraPlateSolver::SolverContext::debugDumpUnmatchedDetections(const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<ProjectedCatalogStar>& projectedStars, const QVector<Match>& finalMatches, double finalMatchRadius)
{
    if (qgetenv("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_UNMATCHED").isEmpty()) {
        return;
    }

    QSet<int> matchedDetections;
    QSet<int> matchedCatalogIndices;
    for (const Match& match : finalMatches)
    {
        matchedDetections.insert(match.detectionIndex);
        matchedCatalogIndices.insert(match.catalogIndex);
    }

    constexpr double kMinSnrForDump = 50.0;
    for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
    {
        if (matchedDetections.contains(detectionIndex)) {
            continue;
        }
        const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
        if (detection.m_snr < kMinSnrForDump) {
            continue;
        }

        double nearestDistance = std::numeric_limits<double>::infinity();
        int nearestCatalogIndex = -1;
        QPointF nearestPoint;
        double nearestMagnitude = 0.0;
        for (const ProjectedCatalogStar& projected : projectedStars)
        {
            const double dx = detection.m_center.x() - projected.point.x();
            const double dy = detection.m_center.y() - projected.point.y();
            const double distance = std::hypot(dx, dy);
            if (distance < nearestDistance)
            {
                nearestDistance = distance;
                nearestCatalogIndex = projected.catalogIndex;
                nearestPoint = projected.point;
                nearestMagnitude = projected.magnitude;
            }
        }

        if (nearestCatalogIndex < 0)
        {
            qDebug() << "CameraPlateSolver[unmatched-detection]"
                     << "detection" << detectionIndex
                     << "x" << detection.m_center.x()
                     << "y" << detection.m_center.y()
                     << "snr" << detection.m_snr
                     << "nearestCatalog" << "<none, projectedStars empty>";
            continue;
        }

        int claimedByDetection = -1;
        if (matchedCatalogIndices.contains(nearestCatalogIndex))
        {
            for (const Match& match : finalMatches)
            {
                if (match.catalogIndex == nearestCatalogIndex)
                {
                    claimedByDetection = match.detectionIndex;
                    break;
                }
            }
        }

        qDebug() << "CameraPlateSolver[unmatched-detection]"
                 << "detection" << detectionIndex
                 << "x" << detection.m_center.x()
                 << "y" << detection.m_center.y()
                 << "snr" << detection.m_snr
                 << "nearestCatalog" << catalogDisplayName(catalogContext.catalogStars[nearestCatalogIndex])
                 << "catalogIndex" << nearestCatalogIndex
                 << "mag" << nearestMagnitude
                 << "projX" << nearestPoint.x()
                 << "projY" << nearestPoint.y()
                 << "distance" << nearestDistance
                 << "withinFinalMatchRadius" << (nearestDistance <= finalMatchRadius)
                 << "claimedByDetection" << claimedByDetection;
    }
}

bool CameraPlateSolver::SolverContext::debugCatalogStarMatches(const PlateSolveCatalogContext& catalogContext, int catalogIndex)
{
    const QByteArray debugStar = qgetenv("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_STAR");
    if (debugStar.trimmed().isEmpty()
        || (catalogIndex < 0)
        || (catalogIndex >= catalogContext.catalogStars.size()))
    {
        return false;
    }

    return catalogDisplayName(catalogContext.catalogStars[catalogIndex]).contains(
        QString::fromUtf8(debugStar).trimmed(),
        Qt::CaseInsensitive);
}

bool CameraPlateSolver::SolverContext::debugTriangleContainsCatalogStar(const PlateSolveCatalogContext& catalogContext, const std::array<int, 3>& catalogIndices)
{
    for (int catalogIndex : catalogIndices)
    {
        if (debugCatalogStarMatches(catalogContext, catalogIndex)) {
            return true;
        }
    }
    return false;
}

QString CameraPlateSolver::SolverContext::debugTriangleAnchorSummary(const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const std::array<int, 3>& detectionIndices, const std::array<int, 3>& catalogIndices)
{
    QStringList parts;
    for (int i = 0; i < 3; ++i)
    {
        const int detectionIndex = detectionIndices[i];
        const int catalogIndex = catalogIndices[i];
        if ((detectionIndex < 0)
            || (detectionIndex >= starDetections.size())
            || (catalogIndex < 0)
            || (catalogIndex >= catalogContext.catalogStars.size()))
        {
            continue;
        }

        const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
        parts.append(QStringLiteral("#%1(%2,%3)->%4")
            .arg(detectionIndex)
            .arg(detection.m_center.x(), 0, 'f', 1)
            .arg(detection.m_center.y(), 0, 'f', 1)
            .arg(catalogDisplayName(catalogContext.catalogStars[catalogIndex])));
    }
    return parts.join(QStringLiteral("; "));
}

CameraPlateSolver::SolverContext::SkyProjector CameraPlateSolver::SolverContext::createProjector(const CameraSettings& settings, const QSize& size, double azimuthDegrees, double elevationDegrees, double rollDegrees, double fovDegrees, double centerOffsetXPixels, double centerOffsetYPixels, double distortionK1)
{
    SkyProjector projector;
    projector.width = size.width();
    projector.height = size.height();
    projector.lensProjection = settings.m_lensProjection;

    if ((projector.width <= 0) || (projector.height <= 0) || (fovDegrees <= 0.0)) {
        return projector;
    }

    const double azimuthRadians = degToRad(azimuthDegrees);
    projector.center = normalize(vectorFromAltAz(azimuthDegrees, elevationDegrees));
    projector.right = normalize({std::cos(azimuthRadians), -std::sin(azimuthRadians), 0.0});
    projector.up = normalize(cross(projector.right, projector.center));
    if ((length(projector.right) <= 0.0) || (length(projector.up) <= 0.0)) {
        return projector;
    }

    const double rollRadians = degToRad(rollDegrees);
    if (std::fabs(rollRadians) > 1e-9)
    {
        projector.right = normalize(rotateAroundAxis(projector.right, projector.center, rollRadians));
        projector.up = normalize(rotateAroundAxis(projector.up, projector.center, rollRadians));
    }

    const double halfHorizontalFov = halfHorizontalFovFromLongEdgeFov(
        settings.m_lensProjection,
        size,
        fovDegrees);
    if ((halfHorizontalFov <= 0.0) || (halfHorizontalFov >= (kPi * 0.5))) {
        return projector;
    }

    projector.halfHorizontalFov = halfHorizontalFov;
    const double aspect = static_cast<double>(projector.height) / static_cast<double>(projector.width);
    projector.horizontalScale = 1.0;
    projector.verticalScale = aspect;
    projector.principalPointX = static_cast<double>(projector.width) * 0.5 + centerOffsetXPixels;
    projector.principalPointY = static_cast<double>(projector.height) * 0.5 + centerOffsetYPixels;
    projector.distortionK1 = distortionK1;
    projector.valid = projector.verticalScale > 0.0;
    return projector;
}

bool CameraPlateSolver::SolverContext::projectVector(const SkyProjector& projector, const SkyVector& vector, QPointF& point)
{
    if (!projector.valid) {
        return false;
    }

    const double depth = dot(vector, projector.center);
    if (depth <= 0.0) {
        return false;
    }

    const double planeX = dot(vector, projector.right);
    const double planeY = dot(vector, projector.up);
    if (!std::isfinite(planeX) || !std::isfinite(planeY)) {
        return false;
    }

    const double theta = std::acos(std::clamp(depth, -1.0, 1.0));
    const double phi = std::atan2(planeY, planeX);
    const double projectionRadius = [&]() -> double
    {
        switch (projector.lensProjection)
        {
        case CameraSettings::LensProjectionEquidistant:
            return theta / projector.halfHorizontalFov;
        case CameraSettings::LensProjectionEquisolid:
            return std::sin(theta * 0.5) / std::sin(projector.halfHorizontalFov * 0.5);
        case CameraSettings::LensProjectionRectilinear:
        default:
            return std::tan(theta) / std::tan(projector.halfHorizontalFov);
        }
    }();

    double projectedX = std::cos(phi) * projectionRadius;
    double projectedY = std::sin(phi) * projectionRadius;
    if (std::fabs(projector.distortionK1) > 1e-9)
    {
        const double radiusSquared = projectedX * projectedX + projectedY * projectedY;
        const double distortionScale = 1.0
            + projector.distortionK1 * radiusSquared;
        // A non-positive scale means the distortion model has folded this direction
        // back past the lens singularity — no valid image point exists.
        if (distortionScale <= 0.0) return false;
        projectedX *= distortionScale;
        projectedY *= distortionScale;
    }

    const double normalizedX = projectedX / projector.horizontalScale;
    const double normalizedY = projectedY / projector.verticalScale;
    if (!std::isfinite(normalizedX) || !std::isfinite(normalizedY)) {
        return false;
    }

    point.setX(projector.principalPointX + normalizedX * 0.5 * static_cast<double>(projector.width));
    point.setY(projector.principalPointY - normalizedY * 0.5 * static_cast<double>(projector.height));
    return true;
}

bool CameraPlateSolver::SolverContext::projectAltAz(const SkyProjector& projector, double azimuthDegrees, double elevationDegrees, QPointF& point)
{
    return projectVector(projector, vectorFromAltAz(azimuthDegrees, elevationDegrees), point);
}

bool CameraPlateSolver::SolverContext::unprojectPixelToVector(const SkyProjector& projector, const QPointF& point, SkyVector& vector)
{
    if (!projector.valid || (projector.width <= 0) || (projector.height <= 0)) {
        return false;
    }

    double projectedX = ((point.x() - projector.principalPointX) / (0.5 * static_cast<double>(projector.width)))
        * projector.horizontalScale;
    double projectedY = (-(point.y() - projector.principalPointY) / (0.5 * static_cast<double>(projector.height)))
        * projector.verticalScale;
    if (!std::isfinite(projectedX) || !std::isfinite(projectedY)) {
        return false;
    }

    if (std::fabs(projector.distortionK1) > 1e-9)
    {
        double undistortedX = projectedX;
        double undistortedY = projectedY;
        bool undistortOk = true;
        for (int iteration = 0; iteration < 8; ++iteration)
        {
            const double radiusSquared = undistortedX * undistortedX + undistortedY * undistortedY;
            const double distortionScale = 1.0
                + projector.distortionK1 * radiusSquared;
            // Non-positive scale means the pixel lies in the folded region of the
            // distortion model — it cannot be mapped back to a sky direction.
            if (distortionScale <= 0.0) { undistortOk = false; break; }
            undistortedX = projectedX / distortionScale;
            undistortedY = projectedY / distortionScale;
        }
        if (!undistortOk) return false;
        projectedX = undistortedX;
        projectedY = undistortedY;
    }

    const double projectionRadius = std::hypot(projectedX, projectedY);
    double theta = 0.0;
    switch (projector.lensProjection)
    {
    case CameraSettings::LensProjectionEquidistant:
        theta = projectionRadius * projector.halfHorizontalFov;
        break;
    case CameraSettings::LensProjectionEquisolid:
        theta = 2.0 * std::asin(std::clamp(
            projectionRadius * std::sin(projector.halfHorizontalFov * 0.5),
            -1.0,
            1.0));
        break;
    case CameraSettings::LensProjectionRectilinear:
    default:
        theta = std::atan(projectionRadius * std::tan(projector.halfHorizontalFov));
        break;
    }
    if (!std::isfinite(theta)) {
        return false;
    }

    const double phi = std::atan2(projectedY, projectedX);
    const double sinTheta = std::sin(theta);
    const double cosTheta = std::cos(theta);
    vector = normalize({
        projector.center.x * cosTheta + (projector.right.x * std::cos(phi) + projector.up.x * std::sin(phi)) * sinTheta,
        projector.center.y * cosTheta + (projector.right.y * std::cos(phi) + projector.up.y * std::sin(phi)) * sinTheta,
        projector.center.z * cosTheta + (projector.right.z * std::cos(phi) + projector.up.z * std::sin(phi)) * sinTheta
    });
    return length(vector) > 0.0;
}

bool CameraPlateSolver::SolverContext::rotateBasisToAlignVector(const SkyProjector& projector, const SkyVector& fromVector, const SkyVector& toVector, SkyVector& alignedCenter, SkyVector& alignedRight, SkyVector& alignedUp)
{
    const SkyVector from = normalize(fromVector);
    const SkyVector to = normalize(toVector);
    if ((length(from) <= 0.0) || (length(to) <= 0.0)) {
        return false;
    }

    const double alignment = std::clamp(dot(from, to), -1.0, 1.0);
    SkyVector rotationAxis = cross(from, to);
    double axisLength = length(rotationAxis);
    if (axisLength <= 1e-9)
    {
        if (alignment > 0.0)
        {
            alignedCenter = projector.center;
            alignedRight = projector.right;
            alignedUp = projector.up;
            return true;
        }

        rotationAxis = cross(from, projector.right);
        axisLength = length(rotationAxis);
        if (axisLength <= 1e-9) {
            rotationAxis = cross(from, projector.up);
            axisLength = length(rotationAxis);
        }
        if (axisLength <= 1e-9) {
            return false;
        }
    }

    rotationAxis = normalize(rotationAxis);
    const double angle = std::acos(alignment);
    alignedCenter = normalize(rotateAroundAxis(projector.center, rotationAxis, angle));
    alignedRight = normalize(rotateAroundAxis(projector.right, rotationAxis, angle));
    alignedUp = normalize(rotateAroundAxis(projector.up, rotationAxis, angle));
    return (length(alignedCenter) > 0.0) && (length(alignedRight) > 0.0) && (length(alignedUp) > 0.0);
}

bool CameraPlateSolver::SolverContext::projectorPoseFromBasis(const SkyVector& center, const SkyVector& right, double& azimuthDegrees, double& elevationDegrees, double& rollDegrees)
{
    const SkyVector normalizedCenter = normalize(center);
    if (length(normalizedCenter) <= 0.0) {
        return false;
    }

    azimuthDegrees = normalizeDegrees(std::atan2(normalizedCenter.x, normalizedCenter.y) * 180.0 / kPi);
    elevationDegrees = std::asin(std::clamp(normalizedCenter.z, -1.0, 1.0)) * 180.0 / kPi;

    const double azimuthRadians = degToRad(azimuthDegrees);
    const SkyVector baseRight = normalize({std::cos(azimuthRadians), -std::sin(azimuthRadians), 0.0});
    SkyVector projectedRight = {
        right.x - normalizedCenter.x * dot(right, normalizedCenter),
        right.y - normalizedCenter.y * dot(right, normalizedCenter),
        right.z - normalizedCenter.z * dot(right, normalizedCenter)
    };
    projectedRight = normalize(projectedRight);
    if ((length(baseRight) <= 0.0) || (length(projectedRight) <= 0.0)) {
        return false;
    }

    const SkyVector rightCross = cross(baseRight, projectedRight);
    rollDegrees = std::atan2(
        dot(rightCross, normalizedCenter),
        std::clamp(dot(baseRight, projectedRight), -1.0, 1.0)) * 180.0 / kPi;
    return std::isfinite(azimuthDegrees) && std::isfinite(elevationDegrees) && std::isfinite(rollDegrees);
}

CameraPlateSolver::SolverContext::SkyVector CameraPlateSolver::SolverContext::subtractScaled(const SkyVector& lhs, const SkyVector& rhs, double scale)
{
    return {
        lhs.x - rhs.x * scale,
        lhs.y - rhs.y * scale,
        lhs.z - rhs.z * scale
    };
}

CameraPlateSolver::SolverContext::SkyVector CameraPlateSolver::SolverContext::combineBasis(const SkyVector& xAxis, const SkyVector& yAxis, const SkyVector& zAxis, double x, double y, double z)
{
    return {
        xAxis.x * x + yAxis.x * y + zAxis.x * z,
        xAxis.y * x + yAxis.y * y + zAxis.y * z,
        xAxis.z * x + yAxis.z * y + zAxis.z * z
    };
}

bool CameraPlateSolver::SolverContext::mappedBasisVector(const SkyVector& sourceVector, const SkyVector& sourceXAxis, const SkyVector& sourceYAxis, const SkyVector& sourceZAxis, const SkyVector& targetXAxis, const SkyVector& targetYAxis, const SkyVector& targetZAxis, SkyVector& mappedVector)
{
    const double x = dot(sourceVector, sourceXAxis);
    const double y = dot(sourceVector, sourceYAxis);
    const double z = dot(sourceVector, sourceZAxis);
    mappedVector = normalize(combineBasis(targetXAxis, targetYAxis, targetZAxis, x, y, z));
    return length(mappedVector) > 0.0;
}

bool CameraPlateSolver::SolverContext::poseFromTwoVectorPairs(const SkyProjector& baseProjector, const SkyVector& sourceA, const SkyVector& sourceB, const SkyVector& targetA, const SkyVector& targetB, double& azimuthDegrees, double& elevationDegrees, double& rollDegrees)
{
    const SkyVector sourceXAxis = normalize(sourceA);
    const SkyVector targetXAxis = normalize(targetA);
    // The second basis axis is the component of B perpendicular to A. When A and B are
    // (near-)parallel this residual collapses toward zero, and normalize() would then amplify
    // floating-point noise into a finite but garbage unit axis that slips past the length() <= 0
    // checks below and produces a wildly wrong pose. Reject when either residual is too small to
    // define a stable axis. For unit inputs the residual length equals sin(angle between A and B),
    // so 1e-6 (~0.00006 deg) is far below any realistic detection/catalog pair separation and only
    // rejects genuinely degenerate inputs.
    const SkyVector sourceYResidual = subtractScaled(sourceB, sourceXAxis, dot(sourceB, sourceXAxis));
    const SkyVector targetYResidual = subtractScaled(targetB, targetXAxis, dot(targetB, targetXAxis));
    constexpr double kMinPerpendicularResidualLength = 1e-6;
    if ((length(sourceXAxis) <= 0.0)
        || (length(targetXAxis) <= 0.0)
        || (length(sourceYResidual) < kMinPerpendicularResidualLength)
        || (length(targetYResidual) < kMinPerpendicularResidualLength))
    {
        return false;
    }
    const SkyVector sourceYAxis = normalize(sourceYResidual);
    const SkyVector sourceZAxis = normalize(cross(sourceXAxis, sourceYAxis));
    const SkyVector targetYAxis = normalize(targetYResidual);
    const SkyVector targetZAxis = normalize(cross(targetXAxis, targetYAxis));
    if ((length(sourceZAxis) <= 0.0)
        || (length(targetZAxis) <= 0.0))
    {
        return false;
    }

    SkyVector mappedCenter;
    SkyVector mappedRight;
    if (!mappedBasisVector(
            baseProjector.center,
            sourceXAxis,
            sourceYAxis,
            sourceZAxis,
            targetXAxis,
            targetYAxis,
            targetZAxis,
            mappedCenter)
        || !mappedBasisVector(
            baseProjector.right,
            sourceXAxis,
            sourceYAxis,
            sourceZAxis,
            targetXAxis,
            targetYAxis,
            targetZAxis,
            mappedRight))
    {
        return false;
    }

    return projectorPoseFromBasis(mappedCenter, mappedRight, azimuthDegrees, elevationDegrees, rollDegrees);
}

CameraPlateSolver::SolverContext::SkyVector CameraPlateSolver::SolverContext::rotateVectorByMatrix(const std::array<std::array<double, 3>, 3>& rotation, const SkyVector& vector)
{
    return {
        rotation[0][0] * vector.x + rotation[0][1] * vector.y + rotation[0][2] * vector.z,
        rotation[1][0] * vector.x + rotation[1][1] * vector.y + rotation[1][2] * vector.z,
        rotation[2][0] * vector.x + rotation[2][1] * vector.y + rotation[2][2] * vector.z
    };
}

std::array<std::array<double, 3>, 3> CameraPlateSolver::SolverContext::transposeRotationMatrix(const std::array<std::array<double, 3>, 3>& rotation)
{
    return {{
        {{rotation[0][0], rotation[1][0], rotation[2][0]}},
        {{rotation[0][1], rotation[1][1], rotation[2][1]}},
        {{rotation[0][2], rotation[1][2], rotation[2][2]}}
    }};
}

bool CameraPlateSolver::SolverContext::vectorPairRotationError(const std::array<std::array<double, 3>, 3>& rotation, const std::array<SkyVector, 3>& sourceVectors, const std::array<SkyVector, 3>& targetVectors, double& rmsAngularErrorDegrees, double& maxAngularErrorDegrees)
{
    double squaredAngularError = 0.0;
    maxAngularErrorDegrees = 0.0;
    for (int index = 0; index < 3; ++index)
    {
        const SkyVector rotatedSource = normalize(rotateVectorByMatrix(rotation, sourceVectors[index]));
        if (length(rotatedSource) <= 0.0) {
            return false;
        }
        const double angularErrorDegrees = std::acos(std::clamp(
            dot(rotatedSource, targetVectors[index]),
            -1.0,
            1.0)) * 180.0 / kPi;
        squaredAngularError += angularErrorDegrees * angularErrorDegrees;
        maxAngularErrorDegrees = std::max(maxAngularErrorDegrees, angularErrorDegrees);
    }
    rmsAngularErrorDegrees = std::sqrt(squaredAngularError / 3.0);
    return std::isfinite(rmsAngularErrorDegrees) && std::isfinite(maxAngularErrorDegrees);
}

bool CameraPlateSolver::SolverContext::normalizeQuaternion(std::array<double, 4>& quaternion)
{
    double norm = 0.0;
    for (double value : quaternion) {
        norm += value * value;
    }
    norm = std::sqrt(norm);
    if (norm <= 0.0) {
        return false;
    }

    for (double& value : quaternion) {
        value /= norm;
    }
    if (quaternion[0] < 0.0) {
        for (double& value : quaternion) {
            value = -value;
        }
    }
    return true;
}

std::array<double, 4> CameraPlateSolver::SolverContext::largestSymmetric4Eigenvector(const std::array<std::array<double, 4>, 4>& matrix)
{
    double shift = 1.0;
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col) {
            shift += std::fabs(matrix[row][col]);
        }
    }

    std::array<double, 4> vector {{1.0, 0.2, -0.3, 0.4}};
    normalizeQuaternion(vector);
    for (int iteration = 0; iteration < 48; ++iteration)
    {
        std::array<double, 4> next {{0.0, 0.0, 0.0, 0.0}};
        for (int row = 0; row < 4; ++row)
        {
            next[row] = shift * vector[row];
            for (int col = 0; col < 4; ++col) {
                next[row] += matrix[row][col] * vector[col];
            }
        }
        if (!normalizeQuaternion(next)) {
            break;
        }
        vector = next;
    }

    return vector;
}

std::array<std::array<double, 3>, 3> CameraPlateSolver::SolverContext::rotationMatrixFromQuaternion(const std::array<double, 4>& quaternion)
{
    const double w = quaternion[0];
    const double x = quaternion[1];
    const double y = quaternion[2];
    const double z = quaternion[3];

    return {{
        {{
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y)
        }},
        {{
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x)
        }},
        {{
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y)
        }}
    }};
}

bool CameraPlateSolver::SolverContext::wahbaRotationFromVectorPairs(const std::array<SkyVector, 3>& sourceVectors, const std::array<SkyVector, 3>& targetVectors, bool transposeCorrelation, std::array<std::array<double, 3>, 3>& rotation, double& rmsAngularErrorDegrees, double& maxAngularErrorDegrees)
{
    std::array<std::array<double, 3>, 3> correlation {{
        {{0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}}
    }};

    for (int index = 0; index < 3; ++index)
    {
        const std::array<double, 3> source {{sourceVectors[index].x, sourceVectors[index].y, sourceVectors[index].z}};
        const std::array<double, 3> target {{targetVectors[index].x, targetVectors[index].y, targetVectors[index].z}};
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                correlation[row][col] += transposeCorrelation
                    ? source[row] * target[col]
                    : target[row] * source[col];
            }
        }
    }

    const double sigma = correlation[0][0] + correlation[1][1] + correlation[2][2];
    const std::array<double, 3> zVector {{
        correlation[1][2] - correlation[2][1],
        correlation[2][0] - correlation[0][2],
        correlation[0][1] - correlation[1][0]
    }};
    const std::array<std::array<double, 3>, 3> symmetric {{
        {{
            2.0 * correlation[0][0] - sigma,
            correlation[0][1] + correlation[1][0],
            correlation[0][2] + correlation[2][0]
        }},
        {{
            correlation[1][0] + correlation[0][1],
            2.0 * correlation[1][1] - sigma,
            correlation[1][2] + correlation[2][1]
        }},
        {{
            correlation[2][0] + correlation[0][2],
            correlation[2][1] + correlation[1][2],
            2.0 * correlation[2][2] - sigma
        }}
    }};

    const std::array<std::array<double, 4>, 4> davenport {{
        {{sigma, zVector[0], zVector[1], zVector[2]}},
        {{zVector[0], symmetric[0][0], symmetric[0][1], symmetric[0][2]}},
        {{zVector[1], symmetric[1][0], symmetric[1][1], symmetric[1][2]}},
        {{zVector[2], symmetric[2][0], symmetric[2][1], symmetric[2][2]}}
    }};

    std::array<double, 4> quaternion = largestSymmetric4Eigenvector(davenport);
    if (!normalizeQuaternion(quaternion)) {
        return false;
    }

    std::array<std::array<double, 3>, 3> rotationCandidate = rotationMatrixFromQuaternion(quaternion);
    double rmsAngularError = std::numeric_limits<double>::infinity();
    double maxAngularError = std::numeric_limits<double>::infinity();
    bool haveRotation = vectorPairRotationError(
        rotationCandidate,
        sourceVectors,
        targetVectors,
        rmsAngularError,
        maxAngularError);

    const std::array<std::array<double, 3>, 3> transposedRotationCandidate =
        transposeRotationMatrix(rotationCandidate);
    double transposedRmsAngularError = std::numeric_limits<double>::infinity();
    double transposedMaxAngularError = std::numeric_limits<double>::infinity();
    const bool haveTransposedRotation = vectorPairRotationError(
        transposedRotationCandidate,
        sourceVectors,
        targetVectors,
        transposedRmsAngularError,
        transposedMaxAngularError);

    if (haveTransposedRotation
        && (!haveRotation || (transposedRmsAngularError < rmsAngularError)))
    {
        rotationCandidate = transposedRotationCandidate;
        rmsAngularError = transposedRmsAngularError;
        maxAngularError = transposedMaxAngularError;
        haveRotation = true;
    }

    if (!haveRotation) {
        return false;
    }

    rotation = rotationCandidate;
    rmsAngularErrorDegrees = rmsAngularError;
    maxAngularErrorDegrees = maxAngularError;
    return true;
}

bool CameraPlateSolver::SolverContext::poseFromThreeVectorPairs(const SkyProjector& baseProjector, const std::array<SkyVector, 3>& sourceVectors, const std::array<SkyVector, 3>& targetVectors, double& azimuthDegrees, double& elevationDegrees, double& rollDegrees, double* rmsAngularErrorDegrees, double* maxAngularErrorDegrees)
{
    std::array<std::array<double, 3>, 3> bestRotation;
    double bestRmsAngularErrorDegrees = std::numeric_limits<double>::infinity();
    double bestMaxAngularErrorDegrees = std::numeric_limits<double>::infinity();
    bool haveRotation = false;

    for (bool transposeCorrelation : {false, true})
    {
        std::array<std::array<double, 3>, 3> rotation;
        double rmsAngularErrorDegreesCandidate = std::numeric_limits<double>::infinity();
        double maxAngularErrorDegreesCandidate = std::numeric_limits<double>::infinity();
        if (!wahbaRotationFromVectorPairs(
                sourceVectors,
                targetVectors,
                transposeCorrelation,
                rotation,
                rmsAngularErrorDegreesCandidate,
                maxAngularErrorDegreesCandidate))
        {
            continue;
        }

        if (rmsAngularErrorDegreesCandidate < bestRmsAngularErrorDegrees)
        {
            bestRotation = rotation;
            bestRmsAngularErrorDegrees = rmsAngularErrorDegreesCandidate;
            bestMaxAngularErrorDegrees = maxAngularErrorDegreesCandidate;
            haveRotation = true;
        }
    }

    if (!haveRotation) {
        return false;
    }

    const SkyVector mappedCenter = normalize(rotateVectorByMatrix(bestRotation, baseProjector.center));
    const SkyVector mappedRight = normalize(rotateVectorByMatrix(bestRotation, baseProjector.right));
    if ((length(mappedCenter) <= 0.0)
        || (length(mappedRight) <= 0.0)
        || !projectorPoseFromBasis(mappedCenter, mappedRight, azimuthDegrees, elevationDegrees, rollDegrees))
    {
        return false;
    }

    if (rmsAngularErrorDegrees) {
        *rmsAngularErrorDegrees = bestRmsAngularErrorDegrees;
    }
    if (maxAngularErrorDegrees) {
        *maxAngularErrorDegrees = bestMaxAngularErrorDegrees;
    }
    return true;
}

bool CameraPlateSolver::SolverContext::vectorPairRotationErrorN(const std::array<std::array<double, 3>, 3>& rotation, const QVector<SkyVector>& sourceVectors, const QVector<SkyVector>& targetVectors, double& rmsAngularErrorDegrees, double& maxAngularErrorDegrees)
{
    const int count = sourceVectors.size();
    if ((count < 1) || (targetVectors.size() != count)) {
        return false;
    }

    double squaredAngularError = 0.0;
    maxAngularErrorDegrees = 0.0;
    for (int index = 0; index < count; ++index)
    {
        const SkyVector rotatedSource = normalize(rotateVectorByMatrix(rotation, sourceVectors[index]));
        if (length(rotatedSource) <= 0.0) {
            return false;
        }
        const double angularErrorDegrees = std::acos(std::clamp(
            dot(rotatedSource, targetVectors[index]),
            -1.0,
            1.0)) * 180.0 / kPi;
        squaredAngularError += angularErrorDegrees * angularErrorDegrees;
        maxAngularErrorDegrees = std::max(maxAngularErrorDegrees, angularErrorDegrees);
    }
    rmsAngularErrorDegrees = std::sqrt(squaredAngularError / count);
    return std::isfinite(rmsAngularErrorDegrees) && std::isfinite(maxAngularErrorDegrees);
}

bool CameraPlateSolver::SolverContext::wahbaRotationFromVectorPairsN(const QVector<SkyVector>& sourceVectors, const QVector<SkyVector>& targetVectors, bool transposeCorrelation, std::array<std::array<double, 3>, 3>& rotation, double& rmsAngularErrorDegrees, double& maxAngularErrorDegrees)
{
    const int count = sourceVectors.size();
    if ((count < 2) || (targetVectors.size() != count)) {
        return false;
    }

    std::array<std::array<double, 3>, 3> correlation {{
        {{0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}}
    }};

    for (int index = 0; index < count; ++index)
    {
        const std::array<double, 3> source {{sourceVectors[index].x, sourceVectors[index].y, sourceVectors[index].z}};
        const std::array<double, 3> target {{targetVectors[index].x, targetVectors[index].y, targetVectors[index].z}};
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                correlation[row][col] += transposeCorrelation
                    ? source[row] * target[col]
                    : target[row] * source[col];
            }
        }
    }

    const double sigma = correlation[0][0] + correlation[1][1] + correlation[2][2];
    const std::array<double, 3> zVector {{
        correlation[1][2] - correlation[2][1],
        correlation[2][0] - correlation[0][2],
        correlation[0][1] - correlation[1][0]
    }};
    const std::array<std::array<double, 3>, 3> symmetric {{
        {{
            2.0 * correlation[0][0] - sigma,
            correlation[0][1] + correlation[1][0],
            correlation[0][2] + correlation[2][0]
        }},
        {{
            correlation[1][0] + correlation[0][1],
            2.0 * correlation[1][1] - sigma,
            correlation[1][2] + correlation[2][1]
        }},
        {{
            correlation[2][0] + correlation[0][2],
            correlation[2][1] + correlation[1][2],
            2.0 * correlation[2][2] - sigma
        }}
    }};

    const std::array<std::array<double, 4>, 4> davenport {{
        {{sigma, zVector[0], zVector[1], zVector[2]}},
        {{zVector[0], symmetric[0][0], symmetric[0][1], symmetric[0][2]}},
        {{zVector[1], symmetric[1][0], symmetric[1][1], symmetric[1][2]}},
        {{zVector[2], symmetric[2][0], symmetric[2][1], symmetric[2][2]}}
    }};

    std::array<double, 4> quaternion = largestSymmetric4Eigenvector(davenport);
    if (!normalizeQuaternion(quaternion)) {
        return false;
    }

    std::array<std::array<double, 3>, 3> rotationCandidate = rotationMatrixFromQuaternion(quaternion);
    double rmsAngularError = std::numeric_limits<double>::infinity();
    double maxAngularError = std::numeric_limits<double>::infinity();
    bool haveRotation = vectorPairRotationErrorN(
        rotationCandidate,
        sourceVectors,
        targetVectors,
        rmsAngularError,
        maxAngularError);

    const std::array<std::array<double, 3>, 3> transposedRotationCandidate =
        transposeRotationMatrix(rotationCandidate);
    double transposedRmsAngularError = std::numeric_limits<double>::infinity();
    double transposedMaxAngularError = std::numeric_limits<double>::infinity();
    const bool haveTransposedRotation = vectorPairRotationErrorN(
        transposedRotationCandidate,
        sourceVectors,
        targetVectors,
        transposedRmsAngularError,
        transposedMaxAngularError);

    if (haveTransposedRotation
        && (!haveRotation || (transposedRmsAngularError < rmsAngularError)))
    {
        rotationCandidate = transposedRotationCandidate;
        rmsAngularError = transposedRmsAngularError;
        maxAngularError = transposedMaxAngularError;
        haveRotation = true;
    }

    if (!haveRotation) {
        return false;
    }

    rotation = rotationCandidate;
    rmsAngularErrorDegrees = rmsAngularError;
    maxAngularErrorDegrees = maxAngularError;
    return true;
}

bool CameraPlateSolver::SolverContext::poseFromVectorPairsN(const SkyProjector& baseProjector, const QVector<SkyVector>& sourceVectors, const QVector<SkyVector>& targetVectors, double& azimuthDegrees, double& elevationDegrees, double& rollDegrees, double* rmsAngularErrorDegrees, double* maxAngularErrorDegrees)
{
    std::array<std::array<double, 3>, 3> bestRotation;
    double bestRmsAngularErrorDegrees = std::numeric_limits<double>::infinity();
    double bestMaxAngularErrorDegrees = std::numeric_limits<double>::infinity();
    bool haveRotation = false;

    for (bool transposeCorrelation : {false, true})
    {
        std::array<std::array<double, 3>, 3> rotation;
        double rmsAngularErrorDegreesCandidate = std::numeric_limits<double>::infinity();
        double maxAngularErrorDegreesCandidate = std::numeric_limits<double>::infinity();
        if (!wahbaRotationFromVectorPairsN(
                sourceVectors,
                targetVectors,
                transposeCorrelation,
                rotation,
                rmsAngularErrorDegreesCandidate,
                maxAngularErrorDegreesCandidate))
        {
            continue;
        }

        if (rmsAngularErrorDegreesCandidate < bestRmsAngularErrorDegrees)
        {
            bestRotation = rotation;
            bestRmsAngularErrorDegrees = rmsAngularErrorDegreesCandidate;
            bestMaxAngularErrorDegrees = maxAngularErrorDegreesCandidate;
            haveRotation = true;
        }
    }

    if (!haveRotation) {
        return false;
    }

    const SkyVector mappedCenter = normalize(rotateVectorByMatrix(bestRotation, baseProjector.center));
    const SkyVector mappedRight = normalize(rotateVectorByMatrix(bestRotation, baseProjector.right));
    if ((length(mappedCenter) <= 0.0)
        || (length(mappedRight) <= 0.0)
        || !projectorPoseFromBasis(mappedCenter, mappedRight, azimuthDegrees, elevationDegrees, rollDegrees))
    {
        return false;
    }

    if (rmsAngularErrorDegrees) {
        *rmsAngularErrorDegrees = bestRmsAngularErrorDegrees;
    }
    if (maxAngularErrorDegrees) {
        *maxAngularErrorDegrees = bestMaxAngularErrorDegrees;
    }
    return true;
}

CameraPlateSolver::SolverContext::QuadVectorCode CameraPlateSolver::SolverContext::buildVectorQuadCode(const std::array<SkyVector, 4>& vectors)
{
    QuadVectorCode code;

    // Step 1: most-separated pair (A, B) = the pair with the smallest dot product
    // (largest angular separation) among the 6 pairs.
    int indexA = 0;
    int indexB = 1;
    double minDot = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            const double dotProduct = dot(vectors[i], vectors[j]);
            if (dotProduct < minDot) {
                minDot = dotProduct;
                indexA = i;
                indexB = j;
            }
        }
    }

    std::array<int, 2> remaining;
    int remainingCount = 0;
    for (int i = 0; i < 4; i++) {
        if ((i != indexA) && (i != indexB)) {
            remaining[remainingCount++] = i;
        }
    }
    const int indexC = remaining[0];
    const int indexD = remaining[1];

    // Step 2: gnomonic-project all 4 unit vectors onto the tangent plane at their centroid.
    SkyVector centroid = normalize({
        vectors[0].x + vectors[1].x + vectors[2].x + vectors[3].x,
        vectors[0].y + vectors[1].y + vectors[2].y + vectors[3].y,
        vectors[0].z + vectors[1].z + vectors[2].z + vectors[3].z
    });
    if (length(centroid) <= 0.0) {
        return code;
    }

    // Orthonormal basis (axisU, axisV) spanning the tangent plane at the centroid. Any valid
    // basis works: rotating all 4 input vectors by a proper rotation rotates the centroid and
    // this basis the same way, so the resulting code is unchanged.
    const SkyVector referenceAxis = (std::abs(centroid.z) < 0.9)
        ? SkyVector{0.0, 0.0, 1.0}
        : SkyVector{1.0, 0.0, 0.0};
    SkyVector axisU = cross(referenceAxis, centroid);
    if (length(axisU) <= 0.0) {
        return code;
    }
    axisU = normalize(axisU);
    const SkyVector axisV = normalize(cross(centroid, axisU));

    std::array<QPointF, 4> planePoints;
    for (int i = 0; i < 4; i++) {
        const double centerDot = dot(vectors[i], centroid);
        if (centerDot <= 1e-6) {
            // Vector is on or behind the tangent plane; code undefined for this quad.
            return code;
        }
        const SkyVector scaled = {
            vectors[i].x / centerDot,
            vectors[i].y / centerDot,
            vectors[i].z / centerDot
        };
        const SkyVector offset = {
            scaled.x - centroid.x,
            scaled.y - centroid.y,
            scaled.z - centroid.z
        };
        planePoints[i] = QPointF(dot(offset, axisU), dot(offset, axisV));
    }

    // Step 3: affine-map A -> (0, 0), B -> (1, 1) using orthogonal equal-length basis vectors
    // e_x' = (d + perp(d)) / 2, e_y' = (d - perp(d)) / 2, where d = B - A and
    // perp(d) = (-d.y, d.x). These sum to d (so B maps to (1,1)) and are orthogonal with
    // length |d| / sqrt(2).
    const QPointF pointA = planePoints[indexA];
    const QPointF pointB = planePoints[indexB];
    const QPointF d = pointB - pointA;
    const double dLengthSquared = d.x() * d.x() + d.y() * d.y();
    if (dLengthSquared <= 0.0) {
        return code;
    }

    const QPointF perpD(-d.y(), d.x());
    const QPointF basisX((d.x() + perpD.x()) / 2.0, (d.y() + perpD.y()) / 2.0);
    const QPointF basisY((d.x() - perpD.x()) / 2.0, (d.y() - perpD.y()) / 2.0);

    auto codeCoordinates = [&](const QPointF& point) -> QPointF {
        const QPointF relative = point - pointA;
        const double u = (relative.x() * basisX.x() + relative.y() * basisX.y()) * 2.0 / dLengthSquared;
        const double v = (relative.x() * basisY.x() + relative.y() * basisY.y()) * 2.0 / dLengthSquared;
        return QPointF(u, v);
    };

    QPointF codeC = codeCoordinates(planePoints[indexC]);
    QPointF codeD = codeCoordinates(planePoints[indexD]);

    // Step 4: canonicalize. First, swap C/D so xC <= xD.
    int orderA = indexA;
    int orderB = indexB;
    int orderC = indexC;
    int orderD = indexD;
    if (codeC.x() > codeD.x()) {
        std::swap(codeC, codeD);
        std::swap(orderC, orderD);
    }

    // Then, swap A/B (equivalent to mapping (x, y) -> (1 - x, 1 - y)) so xC + xD <= 1. This
    // flip reverses the x-ordering of C and D, so re-swap them afterwards.
    bool abSwapped = false;
    if ((codeC.x() + codeD.x()) > 1.0) {
        codeC = QPointF(1.0 - codeC.x(), 1.0 - codeC.y());
        codeD = QPointF(1.0 - codeD.x(), 1.0 - codeD.y());
        std::swap(codeC, codeD);
        std::swap(orderC, orderD);
        std::swap(orderA, orderB);
        abSwapped = true;
    }

    code.valid = true;
    code.xC = codeC.x();
    code.yC = codeC.y();
    code.xD = codeD.x();
    code.yD = codeD.y();
    code.order = {orderA, orderB, orderC, orderD};
    code.abSwapped = abSwapped;

    // Step 5: AB great-circle angle, for optional scale-aware (mode 1) matching.
    const double abDot = std::clamp(dot(vectors[indexA], vectors[indexB]), -1.0, 1.0);
    code.angleABRadians = std::acos(abDot);

    return code;
}

bool CameraPlateSolver::SolverContext::anchorAlignedPoseFromPixel(const CameraSettings& settings, const QSize& imageSize, const QPointF& anchorPoint, const SkyVector& anchorVector, double baseAzimuthDegrees, double baseElevationDegrees, double baseRollDegrees, double fovDegrees, double centerOffsetXPixels, double centerOffsetYPixels, double distortionK1, double& alignedAzimuthDegrees, double& alignedElevationDegrees, double& alignedRollDegrees)
{
    const SkyProjector baseProjector = createProjector(
        settings,
        imageSize,
        baseAzimuthDegrees,
        baseElevationDegrees,
        baseRollDegrees,
        fovDegrees,
        centerOffsetXPixels,
        centerOffsetYPixels,
        distortionK1);
    if (!baseProjector.valid) {
        return false;
    }

    SkyVector pixelVector;
    if (!unprojectPixelToVector(baseProjector, anchorPoint, pixelVector)) {
        return false;
    }

    SkyVector alignedCenter;
    SkyVector alignedRight;
    SkyVector alignedUp;
    if (!rotateBasisToAlignVector(baseProjector, pixelVector, anchorVector, alignedCenter, alignedRight, alignedUp)) {
        return false;
    }

    Q_UNUSED(alignedUp)
    return projectorPoseFromBasis(alignedCenter, alignedRight, alignedAzimuthDegrees, alignedElevationDegrees, alignedRollDegrees);
}

bool CameraPlateSolver::SolverContext::estimateScreenSimilarity(const std::array<QPointF, 3>& sourcePoints, const std::array<QPointF, 3>& targetPoints, double& scale, double& rotationDegrees, double& rmsErrorPixels)
{
    QPointF sourceCentroid(0.0, 0.0);
    QPointF targetCentroid(0.0, 0.0);
    for (int i = 0; i < 3; ++i)
    {
        sourceCentroid += sourcePoints[i];
        targetCentroid += targetPoints[i];
    }
    sourceCentroid /= 3.0;
    targetCentroid /= 3.0;

    double denominator = 0.0;
    double a = 0.0;
    double b = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        const double sx = sourcePoints[i].x() - sourceCentroid.x();
        const double sy = sourcePoints[i].y() - sourceCentroid.y();
        const double tx = targetPoints[i].x() - targetCentroid.x();
        const double ty = targetPoints[i].y() - targetCentroid.y();
        denominator += sx * sx + sy * sy;
        a += sx * tx + sy * ty;
        b += sx * ty - sy * tx;
    }
    if (denominator <= 1e-6) {
        return false;
    }

    a /= denominator;
    b /= denominator;
    scale = std::hypot(a, b);
    if (!std::isfinite(scale) || (scale <= 0.0)) {
        return false;
    }
    rotationDegrees = std::atan2(b, a) * 180.0 / kPi;

    double squaredError = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        const double sx = sourcePoints[i].x() - sourceCentroid.x();
        const double sy = sourcePoints[i].y() - sourceCentroid.y();
        const QPointF projected(
            targetCentroid.x() + a * sx - b * sy,
            targetCentroid.y() + b * sx + a * sy);
        const double error = std::hypot(
            projected.x() - targetPoints[i].x(),
            projected.y() - targetPoints[i].y());
        squaredError += error * error;
    }
    rmsErrorPixels = std::sqrt(squaredError / 3.0);
    return std::isfinite(rotationDegrees) && std::isfinite(rmsErrorPixels);
}

QVector<CameraPlateSolver::SolverContext::GuidedTrianglePoseSeed> CameraPlateSolver::SolverContext::guidedTriangleSimilarityPoseSeeds(const CameraSettings& settings, const QSize& imageSize, const std::array<QPointF, 3>& detectionPoints, const std::array<VisibleCatalogStar, 3>& triangleStars, const std::array<int, 3>& permutation, double baseAzimuthDegrees, double baseElevationDegrees, double seedFovDegrees, double centerOffsetXPixels, double centerOffsetYPixels, double distortionK1)
{
    QVector<GuidedTrianglePoseSeed> seeds;
    const SkyProjector projector = createProjector(
        settings,
        imageSize,
        baseAzimuthDegrees,
        baseElevationDegrees,
        0.0,
        seedFovDegrees,
        centerOffsetXPixels,
        centerOffsetYPixels,
        distortionK1);
    if (!projector.valid) {
        return seeds;
    }

    std::array<QPointF, 3> projectedPoints;
    for (int i = 0; i < 3; ++i)
    {
        if (!projectVector(projector, triangleStars[permutation[i]].vector, projectedPoints[i])) {
            return seeds;
        }
    }

    double scale = 1.0;
    double rotationDegrees = 0.0;
    double similarityRmsPixels = std::numeric_limits<double>::infinity();
    if (!estimateScreenSimilarity(projectedPoints, detectionPoints, scale, rotationDegrees, similarityRmsPixels)
        || (similarityRmsPixels > std::max(18.0, static_cast<double>(settings.m_plateSolveMatchRadius) * 0.75)))
    {
        return seeds;
    }

    QVector<double> fovCandidates;
    const double scaledFov = std::clamp(
        seedFovDegrees / std::max(0.25, std::min(4.0, scale)),
        static_cast<double>(CameraSettings::m_minFov),
        static_cast<double>(CameraSettings::m_maxFov));
    fovCandidates.append(seedFovDegrees);
    if (std::fabs(scaledFov - seedFovDegrees) > std::max(0.01, seedFovDegrees * 0.015)) {
        fovCandidates.append(scaledFov);
    }

    QVector<double> rollCandidates;
    const auto appendRollCandidate = [&rollCandidates](double rollDegrees) {
        for (double existingRoll : rollCandidates)
        {
            const double delta = std::fabs(normalizeDegrees(rollDegrees - existingRoll + 180.0) - 180.0);
            if (delta < 1.0) {
                return;
            }
        }
        rollCandidates.append(rollDegrees);
    };
    appendRollCandidate(rotationDegrees);
    appendRollCandidate(-rotationDegrees);
    const QLineF detectionBase(detectionPoints[0], detectionPoints[1]);
    const QLineF projectedBase(projectedPoints[0], projectedPoints[1]);
    appendRollCandidate(projectedBase.angleTo(detectionBase));

    int brightestAnchor = 0;
    for (int i = 1; i < 3; ++i)
    {
        if (triangleStars[permutation[i]].magnitude < triangleStars[permutation[brightestAnchor]].magnitude) {
            brightestAnchor = i;
        }
    }
    std::array<int, 2> anchorIndexes {{brightestAnchor, 0}};
    if (anchorIndexes[1] == anchorIndexes[0]) {
        anchorIndexes[1] = 1;
    }

    for (double fovDegrees : fovCandidates)
    {
        if (!seedFovCompatibleWithStartFov(settings, fovDegrees)) {
            continue;
        }
        for (double rollDegrees : rollCandidates)
        {
            for (int anchorIndex : anchorIndexes)
            {
                double alignedAzimuth = baseAzimuthDegrees;
                double alignedElevation = baseElevationDegrees;
                double alignedRoll = rollDegrees;
                if (!anchorAlignedPoseFromPixel(
                        settings,
                        imageSize,
                        detectionPoints[anchorIndex],
                        triangleStars[permutation[anchorIndex]].vector,
                        baseAzimuthDegrees,
                        baseElevationDegrees,
                        rollDegrees,
                        fovDegrees,
                        centerOffsetXPixels,
                        centerOffsetYPixels,
                        distortionK1,
                        alignedAzimuth,
                        alignedElevation,
                        alignedRoll))
                {
                    continue;
                }

                GuidedTrianglePoseSeed seed;
                seed.azimuthDegrees = alignedAzimuth;
                seed.elevationDegrees = alignedElevation;
                seed.rollDegrees = alignedRoll;
                seed.fovDegrees = fovDegrees;
                seeds.append(seed);
                if (seeds.size() >= 8) {
                    return seeds;
                }
            }
        }
    }

    return seeds;
}

QPointF CameraPlateSolver::SolverContext::projectorPrincipalPoint(const SkyProjector& projector)
{
    return QPointF(projector.principalPointX, projector.principalPointY);
}

double CameraPlateSolver::SolverContext::pointDistancePixels(const QPointF& lhs, const QPointF& rhs)
{
    return std::hypot(lhs.x() - rhs.x(), lhs.y() - rhs.y());
}

double CameraPlateSolver::SolverContext::detectionBrightnessMetric(const CameraPipelineStarDetection& detection)
{
    const double peak = std::isfinite(static_cast<double>(detection.m_peakValue))
        ? std::max(0.0, static_cast<double>(detection.m_peakValue))
        : 0.0;
    const double flux = std::isfinite(static_cast<double>(detection.m_flux))
        ? std::max(0.0, static_cast<double>(detection.m_flux))
        : 0.0;
    const double quality = std::isfinite(static_cast<double>(detection.m_qualityScore))
        ? std::max(0.0, static_cast<double>(detection.m_qualityScore))
        : 0.0;
    const double brightness = (flux > 0.0) ? std::sqrt(flux) * std::sqrt(std::max(1.0, peak)) : peak;
    double metric = brightness * (1.0 + 0.05 * std::log1p(quality));
    if (detection.m_hotPixelSuspect) {
        metric *= 0.10;
    }
    return metric;
}

bool CameraPlateSolver::SolverContext::triangleBrightnessOrderCompatible(const QVector<CameraPipelineStarDetection>& starDetections, const std::array<int, 3>& detectionIndices, const std::array<VisibleCatalogStar, 3>& triangleStars, const std::array<int, 3>& permutation)
{
    std::array<double, 3> detectionBrightness {{0.0, 0.0, 0.0}};
    std::array<double, 3> catalogMagnitude {{0.0, 0.0, 0.0}};
    for (int i = 0; i < 3; ++i)
    {
        const int detectionIndex = detectionIndices[i];
        if ((detectionIndex < 0) || (detectionIndex >= starDetections.size())) {
            return false;
        }
        const int catalogIndex = permutation[i];
        if ((catalogIndex < 0) || (catalogIndex >= 3)) {
            return false;
        }

        detectionBrightness[i] = detectionBrightnessMetric(starDetections[detectionIndex]);
        catalogMagnitude[i] = triangleStars[catalogIndex].magnitude;
        if (!std::isfinite(detectionBrightness[i]) || !std::isfinite(catalogMagnitude[i])) {
            return true;
        }
    }

    constexpr double kDetectionContrast = 2.0;
    constexpr double kCatalogMagnitudeContrast = 1.0;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = i + 1; j < 3; ++j)
        {
            const double brightnessRatio =
                (detectionBrightness[i] + 1.0) / std::max(1.0, detectionBrightness[j] + 1.0);
            const double magnitudeDelta = catalogMagnitude[i] - catalogMagnitude[j];

            if ((brightnessRatio >= kDetectionContrast)
                && (magnitudeDelta >= kCatalogMagnitudeContrast))
            {
                return false;
            }
            if ((brightnessRatio <= (1.0 / kDetectionContrast))
                && (magnitudeDelta <= -kCatalogMagnitudeContrast))
            {
                return false;
            }
        }
    }

    return true;
}

double CameraPlateSolver::SolverContext::triangleBrightnessMagnitudeError(const QVector<CameraPipelineStarDetection>& starDetections, const std::array<int, 3>& detectionIndices, const std::array<VisibleCatalogStar, 3>& triangleStars, const std::array<int, 3>& permutation)
{
    std::array<double, 3> detectionBrightness {{0.0, 0.0, 0.0}};
    std::array<double, 3> catalogMagnitude {{0.0, 0.0, 0.0}};
    for (int i = 0; i < 3; ++i)
    {
        const int detectionIndex = detectionIndices[i];
        const int catalogIndex = permutation[i];
        if ((detectionIndex < 0)
            || (detectionIndex >= starDetections.size())
            || (catalogIndex < 0)
            || (catalogIndex >= 3))
        {
            return std::numeric_limits<double>::infinity();
        }

        detectionBrightness[i] = detectionBrightnessMetric(starDetections[detectionIndex]);
        catalogMagnitude[i] = triangleStars[catalogIndex].magnitude;
        if (!std::isfinite(detectionBrightness[i]) || !std::isfinite(catalogMagnitude[i])) {
            return 0.0;
        }
    }

    double error = 0.0;
    double weightSum = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = i + 1; j < 3; ++j)
        {
            const double observedLogRatio = std::log(
                (detectionBrightness[i] + 1.0) / std::max(1.0, detectionBrightness[j] + 1.0));
            const double expectedLogRatio = -0.9210340371976183
                * std::clamp(catalogMagnitude[i] - catalogMagnitude[j], -8.0, 8.0);
            const double contrastWeight = std::clamp(std::fabs(expectedLogRatio) / 1.2, 0.25, 1.0);
            const double normalizedError = std::fabs(observedLogRatio - expectedLogRatio)
                / (1.0 + 0.30 * std::fabs(expectedLogRatio));
            error += contrastWeight * normalizedError;
            weightSum += contrastWeight;
        }
    }

    return weightSum > 0.0 ? error / weightSum : 0.0;
}

bool CameraPlateSolver::SolverContext::isDetectionUsableForBrightPrior(const CameraPipelineStarDetection& detection)
{
    if (detection.m_hotPixelSuspect) {
        return false;
    }
    if (detection.m_aspectRatio > 2.5f) {
        return false;
    }
    if ((detection.m_fwhm > 0.0f) && (detection.m_fwhm < 1.0f)) {
        return false;
    }
    return true;
}

bool CameraPlateSolver::SolverContext::isImplausiblyCompactBrightCatalogDetection(const CameraPipelineStarDetection& detection, double catalogMagnitude, bool narrowGuidedSolve)
{
    if (!std::isfinite(catalogMagnitude) || (catalogMagnitude > 9.5)) {
        return false;
    }

    const double fwhm = std::isfinite(static_cast<double>(detection.m_fwhm))
        ? static_cast<double>(detection.m_fwhm)
        : 0.0;
    const double snr = std::isfinite(static_cast<double>(detection.m_snr))
        ? static_cast<double>(detection.m_snr)
        : 0.0;
    if (fwhm <= 0.0) {
        return false;
    }

    if (((catalogMagnitude <= 8.5) || narrowGuidedSolve)
        && (fwhm < 4.5)
        && (snr < 220.0))
    {
        return true;
    }

    return (fwhm < (detection.m_saturated ? 3.2 : 3.0))
        && (snr < 170.0);
}

bool CameraPlateSolver::SolverContext::isNamedSparseGuidedCatalogStar(const CatalogStar& star)
{
    const QString displayName = catalogDisplayName(star);
    return displayName.startsWith(QStringLiteral("HIP "), Qt::CaseInsensitive)
        || displayName.startsWith(QStringLiteral("HR "), Qt::CaseInsensitive)
        || displayName.startsWith(QStringLiteral("HD "), Qt::CaseInsensitive);
}

bool CameraPlateSolver::SolverContext::isStrongSparseGuidedDetection(const CameraPipelineStarDetection& detection)
{
    if (detection.m_hotPixelSuspect) {
        return false;
    }

    const double fwhm = std::isfinite(static_cast<double>(detection.m_fwhm))
        ? static_cast<double>(detection.m_fwhm)
        : 0.0;
    const double snr = std::isfinite(static_cast<double>(detection.m_snr))
        ? static_cast<double>(detection.m_snr)
        : 0.0;
    const double shapeScore =
        std::max(0.0, static_cast<double>(detection.m_roundness))
        * std::max(0.0, static_cast<double>(detection.m_fillRatio))
        / std::max(1.0, static_cast<double>(detection.m_aspectRatio));
    return (detection.m_saturated && (fwhm >= 3.0))
        || ((fwhm >= 4.0) && (snr >= 35.0) && (shapeScore >= 0.10))
        || ((fwhm >= 3.5) && (snr >= 80.0) && (detection.m_aspectRatio <= 2.5f));
}

double CameraPlateSolver::SolverContext::detectionReliabilityMetric(const CameraPipelineStarDetection& detection)
{
    const double quality = std::isfinite(static_cast<double>(detection.m_qualityScore))
        ? std::max(0.0, static_cast<double>(detection.m_qualityScore))
        : 0.0;
    const double snr = std::isfinite(static_cast<double>(detection.m_snr))
        ? std::max(0.0, static_cast<double>(detection.m_snr))
        : 0.0;
    const double centroidUncertainty = std::isfinite(static_cast<double>(detection.m_centroidUncertainty))
        ? std::max(0.05, static_cast<double>(detection.m_centroidUncertainty))
        : 8.0;
    const double fwhm = std::isfinite(static_cast<double>(detection.m_fwhm))
        ? std::max(0.0, static_cast<double>(detection.m_fwhm))
        : 0.0;
    double reliability = std::log1p(quality) * (1.0 + std::log1p(snr)) / centroidUncertainty;
    reliability *= std::max(0.25, static_cast<double>(detection.m_roundness));
    reliability *= std::max(0.25, static_cast<double>(detection.m_fillRatio));
    reliability /= std::max(1.0, static_cast<double>(detection.m_aspectRatio));
    if (detection.m_saturated) {
        reliability *= 0.80;
    }
    if (detection.m_hotPixelSuspect) {
        reliability *= 0.20;
    }
    if ((fwhm > 0.0) && (fwhm < 0.75)) {
        reliability *= 0.35;
    }
    return reliability;
}

double CameraPlateSolver::SolverContext::faintCatalogAssignmentPenalty(double magnitude)
{
    if (!std::isfinite(magnitude)) {
        return 0.0;
    }

    // Gaia/SPCC solves can now include very faint stars. Keep them available,
    // but prefer brighter catalog stars when distance and brightness rank are
    // otherwise comparable, reducing accidental dense-field assignments.
    return std::max(0.0, magnitude - 11.0) * 0.08;
}

double CameraPlateSolver::SolverContext::brightDetectionFaintCatalogAssignmentPenalty(double detectionBrightnessRank, double catalogMagnitude, bool narrowGuidedSolve)
{
    if (!narrowGuidedSolve || !std::isfinite(catalogMagnitude)) {
        return 0.0;
    }

    // In a narrow field, the solver can otherwise build a dense but wrong solution by
    // letting faint Gaia stars consume the brightest detections. Keep faint stars usable
    // for the tail of the field, but make them increasingly implausible anchors for the
    // brightest detections.
    const double brightDetectionWeight = std::pow(
        std::clamp(1.0 - detectionBrightnessRank, 0.0, 1.0),
        2.0);
    if (brightDetectionWeight <= 0.0) {
        return 0.0;
    }

    const double faintExcess = std::max(0.0, catalogMagnitude - 11.0);
    return brightDetectionWeight * faintExcess * faintExcess * 0.12;
}

double CameraPlateSolver::SolverContext::catalogMagnitudeSupportWeight(double magnitude)
{
    if (!std::isfinite(magnitude)) {
        return 1.0;
    }

    // This is deliberately much gentler than true flux scaling, but it still
    // spans the whole practical catalog range.  Each magnitude step changes the
    // evidence by about 22%, so mag 0 remains stronger than mag 5/10/15 without
    // letting one bright star drown out a coherent fainter field.
    const double boundedMagnitude = std::clamp(magnitude, -2.0, 20.0);
    return std::pow(1.22, 10.0 - boundedMagnitude);
}

double CameraPlateSolver::SolverContext::matchDistanceSupportWeight(double distancePixels, double matchRadiusPixels)
{
    if (!std::isfinite(distancePixels)) {
        return 0.0;
    }

    const double safeRadius = std::max(1.0, matchRadiusPixels);
    const double normalizedDistance = std::clamp(distancePixels / safeRadius, 0.0, 2.0);
    return 1.0 / (1.0 + normalizedDistance * normalizedDistance);
}

double CameraPlateSolver::SolverContext::narrowGuidedMagnitudePriorityScore(const FinalMatchPassEvaluation& evaluation)
{
    return evaluation.magnitudeWeightedSupport
        + 2.0 * evaluation.priorityMagnitudeWeightedSupport
        + 1.5 * evaluation.matchedProjectedMagnitudeSupport
        + 2.5 * evaluation.matchedSeedProjectedMagnitudeSupport
        + 0.35 * evaluation.matchedSeedRadialMagnitudeSupport;
}

double CameraPlateSolver::SolverContext::prioritySeedRadialAffinity(const FinalMatchPassEvaluation& evaluation, double matchRadiusPixels)
{
    if ((evaluation.prioritySeedRadialChecks < 2)
        || !std::isfinite(evaluation.prioritySeedRadialErrorPixels))
    {
        return 1.0;
    }

    const double safeRadius = std::max(1.0, matchRadiusPixels);
    const double normalizedError = evaluation.prioritySeedRadialErrorPixels / std::max(48.0, safeRadius * 3.0);
    return 1.0 / (1.0 + 2.5 * normalizedError * normalizedError);
}

double CameraPlateSolver::SolverContext::prioritySeedProjectedAffinity(const FinalMatchPassEvaluation& evaluation, double matchRadiusPixels)
{
    if ((evaluation.prioritySeedProjectedChecks < 1)
        || !std::isfinite(evaluation.prioritySeedProjectedErrorPixels))
    {
        return 1.0;
    }

    const double safeRadius = std::max(1.0, matchRadiusPixels);
    const double normalizedError = evaluation.prioritySeedProjectedErrorPixels / std::max(48.0, safeRadius * 2.0);
    return 1.0 / (1.0 + 1.5 * normalizedError * normalizedError);
}

double CameraPlateSolver::SolverContext::seedRadialMagnitudeCoverageAffinity(const FinalMatchPassEvaluation& evaluation)
{
    if (evaluation.seedRadialMagnitudeSupport <= 0.0) {
        return 1.0;
    }

    const double coverage = std::clamp(evaluation.seedRadialMagnitudeMatchFraction, 0.0, 1.0);
    return 0.05 + 0.95 * coverage * coverage;
}

double CameraPlateSolver::SolverContext::seedProjectedMagnitudeCoverageAffinity(const FinalMatchPassEvaluation& evaluation)
{
    if (evaluation.seedProjectedMagnitudeSupport <= 0.0) {
        return 1.0;
    }

    const double coverage = std::clamp(evaluation.seedProjectedMagnitudeMatchFraction, 0.0, 1.0);
    return 0.08 + 0.92 * coverage * coverage;
}

bool CameraPlateSolver::SolverContext::hasPoorNoRollSeedRadialSupport(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation, bool useSeedProjectedBrightGate)
{
    if (useSeedProjectedBrightGate
        || (!isNarrowField(settings))
        || (evaluation.seedRadialMagnitudeSupport < 40.0)
        || (evaluation.prioritySeedRadialChecks < 8)
        || !std::isfinite(evaluation.prioritySeedRadialErrorPixels))
    {
        return false;
    }

    const double radialErrorThreshold = std::max(
        96.0,
        static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 4.0);
    return (evaluation.seedRadialMagnitudeMatchFraction < 0.08)
        && (evaluation.prioritySeedRadialErrorPixels > radialErrorThreshold);
}

bool CameraPlateSolver::SolverContext::hasComparableSeedRadialChecks(const FinalMatchPassEvaluation& lhs, const FinalMatchPassEvaluation& rhs)
{
    return (lhs.prioritySeedRadialChecks >= 4)
        && (rhs.prioritySeedRadialChecks >= 4)
        && std::isfinite(lhs.prioritySeedRadialErrorPixels)
        && std::isfinite(rhs.prioritySeedRadialErrorPixels);
}

bool CameraPlateSolver::SolverContext::shouldPreferSeedRadialConsistency(const FinalMatchPassEvaluation& candidate, const FinalMatchPassEvaluation& best, int matchTolerance)
{
    if (!hasComparableSeedRadialChecks(candidate, best)) {
        return false;
    }

    const int matchDelta = static_cast<int>(candidate.finalMatches.size())
        - static_cast<int>(best.finalMatches.size());
    if (std::abs(matchDelta) > matchTolerance) {
        return false;
    }

    if ((candidate.seedRadialMagnitudeSupport > 0.0)
        && (best.seedRadialMagnitudeSupport > 0.0))
    {
        const double candidateCoverage = std::clamp(candidate.seedRadialMagnitudeMatchFraction, 0.0, 1.0);
        const double bestCoverage = std::clamp(best.seedRadialMagnitudeMatchFraction, 0.0, 1.0);
        if ((candidateCoverage - bestCoverage) >= 0.18) {
            return true;
        }

        const double supportDelta =
            candidate.matchedSeedRadialMagnitudeSupport - best.matchedSeedRadialMagnitudeSupport;
        const double supportScale = std::max(
            candidate.matchedSeedRadialMagnitudeSupport,
            best.matchedSeedRadialMagnitudeSupport);
        if (supportDelta >= std::max(4.0, 0.20 * supportScale)) {
            return true;
        }
    }

    const double candidateError = candidate.prioritySeedRadialErrorPixels;
    const double bestError = best.prioritySeedRadialErrorPixels;
    const double meaningfulDelta = std::max(32.0, 0.25 * std::max(candidateError, bestError));
    return (bestError - candidateError) >= meaningfulDelta;
}

double CameraPlateSolver::SolverContext::narrowGuidedAnchorShapeScore(const CameraPipelineStarDetection& detection)
{
    if (detection.m_hotPixelSuspect) {
        return -100.0;
    }

    double score = 0.0;
    const double fwhm = std::isfinite(static_cast<double>(detection.m_fwhm))
        ? std::max(0.0, static_cast<double>(detection.m_fwhm))
        : 0.0;

    if (detection.m_saturated) {
        score += 35.0;
    }
    if (fwhm >= 3.0) {
        score += std::min(45.0, (fwhm - 3.0) * 10.0);
    } else if ((fwhm > 0.0) && (fwhm < 2.5)) {
        score -= (2.5 - fwhm) * 12.0;
    }

    if (detection.m_aspectRatio > 1.8f) {
        score -= std::min(30.0, (static_cast<double>(detection.m_aspectRatio) - 1.8) * 20.0);
    }
    score += std::min(8.0, std::max(0.0, static_cast<double>(detection.m_roundness) - 0.7) * 20.0);
    return score;
}

double CameraPlateSolver::SolverContext::brightGuidedDetectionPriorityScore(const CameraPipelineStarDetection& detection, double brightness, double reliability, double shapeScore)
{
    const double fwhm = std::isfinite(static_cast<double>(detection.m_fwhm))
        ? std::max(0.0, static_cast<double>(detection.m_fwhm))
        : 0.0;
    const double flux = std::isfinite(static_cast<double>(detection.m_flux))
        ? std::max(0.0, static_cast<double>(detection.m_flux))
        : 0.0;
    const double broadSaturatedBonus = detection.m_saturated
        ? std::min(55.0, std::max(0.0, fwhm - 2.5) * 7.5 + std::log1p(flux) * 2.0)
        : 0.0;

    return std::log1p(std::max(0.0, brightness)) * 9.0
        + std::log1p(std::max(0.0, reliability)) * 6.0
        + shapeScore * 1.8
        + broadSaturatedBonus;
}

void CameraPlateSolver::SolverContext::buildProjectedCatalogInto(const PlateSolveCatalogContext& catalogContext, const SkyProjector& projector, double searchMarginPixels, const QVector<int>* allowedCatalogIndices, QVector<ProjectedCatalogStar>& projectedStars)
{
    projectedStars.clear();
    const QRectF expandedBounds(
        -searchMarginPixels,
        -searchMarginPixels,
        projector.width + 2.0 * searchMarginPixels,
        projector.height + 2.0 * searchMarginPixels);

    const auto appendProjectedStar = [&](const VisibleCatalogStar& visibleStar)
    {
        QPointF point;
        if (!projectVector(projector, visibleStar.vector, point)) {
            return;
        }
        if (!expandedBounds.contains(point)) {
            return;
        }

        projectedStars.append({visibleStar.catalogIndex, point, visibleStar.magnitude});
    };

    // Angular cone cull. Only catalog stars whose direction lies within the projector's field
    // cone can land inside the (expanded) image bounds, so for a narrow field a cheap dot-product
    // test against the bore-sight rejects the vast majority of candidate stars before the much
    // more expensive projectVector(). Only enabled for genuinely narrow fields; for wide/fisheye
    // fields the cone spans most of the sky and would reject nothing, so the dot product is skipped.
    //
    // The cone half-angle must cover the angle from the bore-sight (projector.center, which maps to
    // the principal point) to the farthest in-bounds pixel. A lens centre offset shifts the
    // principal point away from the image centre (by up to +/-2048 px), so the maximum normalized
    // image radius is derived from the principal point itself (which already includes the offset)
    // plus the search margin, NOT from a centred half-diagonal. This keeps the cone a strict
    // superset of anything projectVector()+bounds accepts even with a large centre offset. The 2x
    // factor plus 1 deg of slack leaves headroom for the small-angle approximation; for a centred
    // principal point this only widens the cone, so the culled set is identical to the unculled scan.
    const double halfWidthPixels = 0.5 * static_cast<double>(projector.width);
    const double maxRadiusXPixels = std::max(projector.principalPointX, projector.width - projector.principalPointX) + searchMarginPixels;
    const double maxRadiusYPixels = std::max(projector.principalPointY, projector.height - projector.principalPointY) + searchMarginPixels;
    const double maxNormalizedRadius = (halfWidthPixels > 0.0)
        ? std::sqrt(maxRadiusXPixels * maxRadiusXPixels + maxRadiusYPixels * maxRadiusYPixels) / halfWidthPixels
        : 0.0;
    const double halfDiagonalFovRadians = projector.halfHorizontalFov * maxNormalizedRadius;
    const double coneHalfAngleRadians = halfDiagonalFovRadians * 2.0 + degToRad(1.0);
    // projectVector() applies radial distortion (scale = 1 + k1*r^2 + k2*r^4) AFTER projection.
    // With barrel distortion (scale < 1) a star at a larger undistorted angle is pulled
    // radially inward and can still land in bounds, so an undistorted-angle cone would wrongly
    // reject it. The solver sweeps negative k1 (distortion sweep + LM refinement), so the cone is
    // only a guaranteed superset of the in-bounds set when k1 >= 0 (positive/pincushion
    // distortion pushes stars outward, shrinking the in-bounds angular region).
    // Disable the cull otherwise.
    const bool useConeCull = (coneHalfAngleRadians < degToRad(15.0))
        && (projector.distortionK1 >= 0.0);
    const double coneCosThreshold = useConeCull ? std::cos(coneHalfAngleRadians) : -2.0;
    const auto withinFieldCone = [&](const VisibleCatalogStar& visibleStar) {
        return !useConeCull || (dot(projector.center, visibleStar.vector) >= coneCosThreshold);
    };

    if (allowedCatalogIndices)
    {
        projectedStars.reserve(allowedCatalogIndices->size());
        for (int catalogIndex : *allowedCatalogIndices)
        {
            const auto it = catalogContext.visibleStarIndexByCatalogIndex.constFind(catalogIndex);
            if (it != catalogContext.visibleStarIndexByCatalogIndex.cend())
            {
                const VisibleCatalogStar& visibleStar = catalogContext.visibleStars[*it];
                if (withinFieldCone(visibleStar)) {
                    appendProjectedStar(visibleStar);
                }
            }
        }
    }
    else
    {
        projectedStars.reserve(catalogContext.visibleStars.size());
        for (const VisibleCatalogStar& visibleStar : catalogContext.visibleStars)
        {
            if (withinFieldCone(visibleStar)) {
                appendProjectedStar(visibleStar);
            }
        }
    }
}

QVector<CameraPlateSolver::SolverContext::ProjectedCatalogStar> CameraPlateSolver::SolverContext::buildProjectedCatalog(const PlateSolveCatalogContext& catalogContext, const SkyProjector& projector, double searchMarginPixels, const QVector<int>* allowedCatalogIndices)
{
    QVector<ProjectedCatalogStar> projectedStars;
    buildProjectedCatalogInto(catalogContext, projector, searchMarginPixels, allowedCatalogIndices, projectedStars);
    return projectedStars;
}

CameraPlateSolver::SolverContext::RaDecToAzAltParams CameraPlateSolver::SolverContext::buildRaDecToAzAltParams(double latitude, double longitude, const QDateTime& captureDateTimeUtc)
{
    RaDecToAzAltParams p;
    const double lat_rad = degToRad(latitude);
    p.sinLat = std::sin(lat_rad);
    p.cosLat = std::cos(lat_rad);
    p.lst_deg = Astronomy::localSiderealTime(captureDateTimeUtc, longitude);
    p.jd = Astronomy::julianDate(captureDateTimeUtc);
    p.jd_from = Astronomy::jd_j2000();

    // Precession rotation matrix — same formula as Astronomy::precess(),
    // precomputed once for (J2000 → current epoch) so each star only needs
    // a 3x3 matrix multiply instead of rebuilding the matrix from scratch.
    const double days_per_century = 36524.219878;
    const double t0 = (p.jd_from - Astronomy::jd_b1950()) / days_per_century;
    const double t  = (p.jd       - p.jd_from)            / days_per_century;
    p.rot[0][0] = 1.0 - ((29696.0 + 26.0*t0)*t*t - 13.0*t*t*t)*1e-8;
    p.rot[1][0] = ((2234941.0 + 1355.0*t0)*t - 676.0*t*t + 221.0*t*t*t)*1e-8;
    p.rot[2][0] = ((971690.0  -  414.0*t0)*t + 207.0*t*t +  96.0*t*t*t)*1e-8;
    p.rot[0][1] = -p.rot[1][0];
    p.rot[1][1] = 1.0 - ((24975.0 + 30.0*t0)*t*t - 15.0*t*t*t)*1e-8;
    p.rot[2][1] = -((10858.0 + 2.0*t0)*t*t)*1e-8;
    p.rot[0][2] = -p.rot[2][0];
    p.rot[1][2] = p.rot[2][1];
    p.rot[2][2] = 1.0 - ((4721.0 - 4.0*t0)*t*t)*1e-8;
    return p;
}

bool CameraPlateSolver::SolverContext::raDecToAzAltFast(const RaDecToAzAltParams& p, double rightAscensionDegrees, double declinationDegrees, double& az, double& alt)
{
    // Apply precession (J2000 → current epoch) via precomputed rotation matrix.
    // Convert RA/Dec to unit vector, rotate, convert back.
    const double ra_rad  = degToRad(rightAscensionDegrees);
    const double dec_rad0 = degToRad(declinationDegrees);
    const double cosDec = std::cos(dec_rad0);
    double x = cosDec * std::cos(ra_rad);
    double y = cosDec * std::sin(ra_rad);
    double z = std::sin(dec_rad0);
    const double xp = p.rot[0][0]*x + p.rot[0][1]*y + p.rot[0][2]*z;
    const double yp = p.rot[1][0]*x + p.rot[1][1]*y + p.rot[1][2]*z;
    const double zp = p.rot[2][0]*x + p.rot[2][1]*y + p.rot[2][2]*z;

    // Recover precessed RA/Dec from rotated vector.
    const double dec_rad = std::asin(std::clamp(zp, -1.0, 1.0));
    const double ra_prec_rad = std::atan2(yp, xp);
    const double ra_prec_deg = (ra_prec_rad < 0.0 ? ra_prec_rad + 2.0 * kPi : ra_prec_rad) * (180.0 / kPi);

    // Hour angle.
    const double ha_deg = std::fmod(p.lst_deg - ra_prec_deg, 360.0);
    const double ha_rad = degToRad(ha_deg);

    // Altitude and azimuth.
    const double sinDec = std::sin(dec_rad);
    const double cosDec2 = std::cos(dec_rad);
    const double cosHa  = std::cos(ha_rad);
    const double alt_rad = std::asin(std::clamp(sinDec*p.sinLat + cosDec2*p.cosLat*cosHa, -1.0, 1.0));
    alt = alt_rad * (180.0 / kPi);

    if (!std::isfinite(alt) || (alt < kVisibleAltitudeFloor)) {
        return false;
    }

    const double cosAlt = std::cos(alt_rad);
    if (std::fabs(cosAlt) < 1e-12) {
        az = 0.0;
        return true;
    }
    const double cosA = std::clamp((sinDec - std::sin(alt_rad)*p.sinLat) / (cosAlt*p.cosLat), -1.0, 1.0);
    const double a = std::acos(cosA) * (180.0 / kPi);
    az = (std::sin(ha_rad) < 0.0) ? a : 360.0 - a;
    return std::isfinite(az);
}

QVector<CameraPlateSolver::SolverContext::VisibleCatalogStar> CameraPlateSolver::SolverContext::buildVisibleCatalog(const CameraSettings& settings, const QVector<CatalogStar>& catalogStars, const QDateTime& captureDateTimeUtc, double maxMagnitude)
{
    // Precompute all time/location-dependent constants once for all stars.
    const RaDecToAzAltParams azAltParams = buildRaDecToAzAltParams(
        settings.m_latitude, settings.m_longitude, captureDateTimeUtc);

    const auto buildVisibleCatalogRange = [&](int firstStarIndex, int lastStarIndex) {
        QVector<VisibleCatalogStar> visibleStars;
        visibleStars.reserve(std::max(0, lastStarIndex - firstStarIndex));

        for (int i = firstStarIndex; i < lastStarIndex; ++i)
        {
            const CatalogStar& star = catalogStars[i];
            if (star.magnitude > maxMagnitude) {
                continue;
            }

            double az = 0.0;
            double alt = 0.0;
            if (!raDecToAzAltFast(azAltParams, star.rightAscensionDegrees, star.declinationDegrees, az, alt)) {
                continue;
            }

            visibleStars.append({
                i,
                az,
                alt,
                star.magnitude,
                normalize(vectorFromAltAz(az, alt))
            });
        }

        return visibleStars;
    };

    const int workerThreadCount = visibleCatalogWorkerThreadCount(catalogStars.size());
    QVector<VisibleCatalogStar> visibleStars;
    if (workerThreadCount <= 1)
    {
        visibleStars = buildVisibleCatalogRange(0, catalogStars.size());
    }
    else
    {
        QVector<QVector<VisibleCatalogStar>> chunkVisibleStars(workerThreadCount);
        QThreadPool workerPool;
        workerPool.setMaxThreadCount(workerThreadCount);
        for (int workerIndex = 0; workerIndex < workerThreadCount; ++workerIndex)
        {
            const int firstStarIndex = (catalogStars.size() * workerIndex) / workerThreadCount;
            const int lastStarIndex = (catalogStars.size() * (workerIndex + 1)) / workerThreadCount;
            workerPool.start(QRunnable::create([&, workerIndex, firstStarIndex, lastStarIndex]() {
                chunkVisibleStars[workerIndex] = buildVisibleCatalogRange(firstStarIndex, lastStarIndex);
            }));
        }
        workerPool.waitForDone();

        int visibleStarCount = 0;
        for (const QVector<VisibleCatalogStar>& chunk : chunkVisibleStars) {
            visibleStarCount += chunk.size();
        }
        visibleStars.reserve(visibleStarCount);
        for (QVector<VisibleCatalogStar>& chunk : chunkVisibleStars) {
            visibleStars += chunk;
        }
    }

    std::sort(visibleStars.begin(), visibleStars.end(), [](const VisibleCatalogStar& lhs, const VisibleCatalogStar& rhs) {
        return lhs.magnitude < rhs.magnitude;
    });

    return visibleStars;
}

void CameraPlateSolver::SolverContext::buildVisibleStarIndex(const QVector<VisibleCatalogStar>& visibleStars, QHash<int, int>& visibleStarIndexByCatalogIndex)
{
    visibleStarIndexByCatalogIndex.clear();
    visibleStarIndexByCatalogIndex.reserve(visibleStars.size());
    for (int i = 0; i < visibleStars.size(); ++i) {
        visibleStarIndexByCatalogIndex.insert(visibleStars[i].catalogIndex, i);
    }
}

QMutex& CameraPlateSolver::SolverContext::visibleCatalogCacheMutex()
{
    static QMutex s_cacheMutex;
    return s_cacheMutex;
}

QHash<QString, CameraPlateSolver::SolverContext::VisibleCatalogCacheEntry>& CameraPlateSolver::SolverContext::visibleCatalogCache()
{
    static QHash<QString, VisibleCatalogCacheEntry> s_cache;
    return s_cache;
}

QStringList& CameraPlateSolver::SolverContext::visibleCatalogCacheOrder()
{
    static QStringList s_cacheOrder;
    return s_cacheOrder;
}

QString CameraPlateSolver::SolverContext::visibleCatalogFingerprint(const QVector<CatalogStar>& catalogStars)
{
    if (catalogStars.isEmpty()) {
        return QStringLiteral("0");
    }

    const int middleIndex = catalogStars.size() / 2;
    const int lastIndex = catalogStars.size() - 1;
    const int sampleIndexes[] = {0, middleIndex, lastIndex};
    QStringList fields;
    fields.reserve(1 + 3 * 3);
    fields.append(QString::number(catalogStars.size()));
    for (int index : sampleIndexes)
    {
        const CatalogStar& star = catalogStars[index];
        fields.append(QString::number(qRound64(star.rightAscensionDegrees * 1000000.0)));
        fields.append(QString::number(qRound64(star.declinationDegrees * 1000000.0)));
        fields.append(QString::number(qRound64(star.magnitude * 1000.0)));
    }
    return fields.join(QLatin1Char(':'));
}

double CameraPlateSolver::SolverContext::visibleCatalogCacheMagnitudeLimit(const QVector<CatalogStar>& catalogStars, double maxMagnitude)
{
    double loadedMaxMagnitude = -std::numeric_limits<double>::infinity();
    for (const CatalogStar& star : catalogStars)
    {
        if (std::isfinite(star.magnitude)) {
            loadedMaxMagnitude = std::max(loadedMaxMagnitude, star.magnitude);
        }
    }

    return std::isfinite(loadedMaxMagnitude)
        ? std::min(maxMagnitude, loadedMaxMagnitude)
        : maxMagnitude;
}

QString CameraPlateSolver::SolverContext::visibleCatalogCacheKey(const CameraSettings& settings, const QVector<CatalogStar>& catalogStars, const QDateTime& captureDateTimeUtc, double maxMagnitude)
{
    const QString path = currentCatalogPath(settings);
    const bool isResource = path.startsWith(QLatin1String(":/"));
    const qint64 modifiedSecs = isResource ? 0 : QFileInfo(path).lastModified().toSecsSinceEpoch();
    const qint64 timeBucket = captureDateTimeUtc.toSecsSinceEpoch() / 30;
    const double effectiveMaxMagnitude = visibleCatalogCacheMagnitudeLimit(catalogStars, maxMagnitude);

    return QStringLiteral("%1|%2|%3|%4|%5|%6|%7|%8")
        .arg(path)
        .arg(modifiedSecs)
        .arg(visibleCatalogFingerprint(catalogStars))
        .arg(qRound64(settings.m_latitude * 100000.0))
        .arg(qRound64(settings.m_longitude * 100000.0))
        .arg(timeBucket)
        .arg(qRound64(effectiveMaxMagnitude * 100.0))
        .arg(static_cast<int>(settings.m_plateSolveCatalogSource));
}

bool CameraPlateSolver::SolverContext::loadVisibleCatalogCache(const CameraSettings& settings, const QVector<CatalogStar>& catalogStars, const QDateTime& captureDateTimeUtc, double maxMagnitude, QVector<VisibleCatalogStar>& visibleStars, QHash<int, int>& visibleStarIndexByCatalogIndex)
{
    const QString key = visibleCatalogCacheKey(settings, catalogStars, captureDateTimeUtc, maxMagnitude);
    QMutexLocker locker(&visibleCatalogCacheMutex());
    const QHash<QString, VisibleCatalogCacheEntry>& cache = visibleCatalogCache();
    const auto it = cache.constFind(key);
    if (it == cache.cend()) {
        return false;
    }

    visibleStars = it->visibleStars;
    visibleStarIndexByCatalogIndex = it->visibleStarIndexByCatalogIndex;
    return true;
}

void CameraPlateSolver::SolverContext::storeVisibleCatalogCache(const CameraSettings& settings, const QVector<CatalogStar>& catalogStars, const QDateTime& captureDateTimeUtc, double maxMagnitude, const QVector<VisibleCatalogStar>& visibleStars, const QHash<int, int>& visibleStarIndexByCatalogIndex)
{
    static constexpr int kMaxVisibleCatalogCacheEntries = 8;

    const QString key = visibleCatalogCacheKey(settings, catalogStars, captureDateTimeUtc, maxMagnitude);
    QMutexLocker locker(&visibleCatalogCacheMutex());
    QHash<QString, VisibleCatalogCacheEntry>& cache = visibleCatalogCache();
    QStringList& cacheOrder = visibleCatalogCacheOrder();
    if (!cache.contains(key))
    {
        cacheOrder.append(key);
        while (cacheOrder.size() > kMaxVisibleCatalogCacheEntries) {
            cache.remove(cacheOrder.takeFirst());
        }
    }
    cache.insert(key, {visibleStars, visibleStarIndexByCatalogIndex});
}

// File-local: the P-A seed cache's catalog-generation stamp. Monotonic within a process, not
// Date/Random-based (deterministic/resume-safe). Only populateVisibleCatalogContext bumps it.
static quint64 nextVisibleStarsGeneration()
{
    static std::atomic<quint64> counter{0};
    return counter.fetch_add(1, std::memory_order_relaxed) + 1;
}

void CameraPlateSolver::SolverContext::populateVisibleCatalogContext(PlateSolveCatalogContext& context, const CameraSettings& settings, const QDateTime& captureDateTimeUtc, double maxMagnitude, bool allowCache)
{
    // Every assignment to context.visibleStars happens in this function, so stamping a fresh
    // generation on each populate is a sound cache-invalidation signal for the P-A seed-prior cache.
    context.visibleStarsGeneration = nextVisibleStarsGeneration();
    if (allowCache
        && loadVisibleCatalogCache(
            settings,
            context.catalogStars,
            captureDateTimeUtc,
            maxMagnitude,
            context.visibleStars,
            context.visibleStarIndexByCatalogIndex))
    {
        return;
    }

    context.visibleStars = buildVisibleCatalog(settings, context.catalogStars, captureDateTimeUtc, maxMagnitude);
    buildVisibleStarIndex(context.visibleStars, context.visibleStarIndexByCatalogIndex);
    if (allowCache) {
        storeVisibleCatalogCache(
            settings,
            context.catalogStars,
            captureDateTimeUtc,
            maxMagnitude,
            context.visibleStars,
            context.visibleStarIndexByCatalogIndex);
    }
}

QVector<CameraPlateSolver::SolverContext::VisibleCatalogStar> CameraPlateSolver::SolverContext::selectLocalVisibleStars(const QVector<VisibleCatalogStar>& visibleStars, double centerAzimuthDegrees, double centerElevationDegrees, double radiusDegrees, int maxStars)
{
    QVector<VisibleCatalogStar> localStars;
    if ((maxStars <= 0) || visibleStars.isEmpty()) {
        return localStars;
    }

    const SkyVector center = normalize(vectorFromAltAz(centerAzimuthDegrees, centerElevationDegrees));
    const double radiusRadians = std::max(0.0, radiusDegrees) * kPi / 180.0;
    const double minDot = std::cos(radiusRadians);
    localStars.reserve(std::min(maxStars, static_cast<int>(visibleStars.size())));
    for (const VisibleCatalogStar& star : visibleStars)
    {
        if (dot(center, star.vector) >= minDot) {
            localStars.append(star);
        }
    }

    std::sort(localStars.begin(), localStars.end(), [](const VisibleCatalogStar& lhs, const VisibleCatalogStar& rhs) {
        return lhs.magnitude < rhs.magnitude;
    });
    if (localStars.size() > maxStars) {
        localStars.resize(maxStars);
    }
    return localStars;
}

QVector<CameraPlateSolver::SolverContext::VisibleCatalogStar> CameraPlateSolver::SolverContext::selectVisibleStarsNearElevation(const QVector<VisibleCatalogStar>& visibleStars, double centerElevationDegrees, double radiusDegrees, int maxStars)
{
    QVector<VisibleCatalogStar> localStars;
    if ((maxStars <= 0) || visibleStars.isEmpty()) {
        return localStars;
    }

    const double radius = std::max(0.0, radiusDegrees);
    localStars.reserve(std::min(maxStars, static_cast<int>(visibleStars.size())));
    for (const VisibleCatalogStar& star : visibleStars)
    {
        if (std::fabs(star.elevationDegrees - centerElevationDegrees) <= radius) {
            localStars.append(star);
        }
    }

    std::sort(localStars.begin(), localStars.end(), [](const VisibleCatalogStar& lhs, const VisibleCatalogStar& rhs) {
        return lhs.magnitude < rhs.magnitude;
    });
    if (localStars.size() > maxStars) {
        localStars.resize(maxStars);
    }
    return localStars;
}

CameraPlateSolver::SolverContext::TriangleSignature CameraPlateSolver::SolverContext::buildTriangleSignature(const std::array<QPointF, 3>& points)
{
    struct EdgeInfo {
        double length = 0.0;
        int a = -1;
        int b = -1;
    };

    std::array<EdgeInfo, 3> edges {{
        {QLineF(points[0], points[1]).length(), 0, 1},
        {QLineF(points[0], points[2]).length(), 0, 2},
        {QLineF(points[1], points[2]).length(), 1, 2}
    }};
    std::sort(edges.begin(), edges.end(), [](const EdgeInfo& lhs, const EdgeInfo& rhs) {
        return lhs.length < rhs.length;
    });

    TriangleSignature signature;
    if (edges[2].length <= 1e-6) {
        return signature;
    }

    signature.ratioShortToLong = edges[0].length / edges[2].length;
    signature.ratioMidToLong = edges[1].length / edges[2].length;
    signature.longestDistance = edges[2].length;

    const QPointF base = points[edges[2].a];
    const QPointF tip = points[edges[2].b];
    const int thirdIndex = 3 - edges[2].a - edges[2].b;
    const QPointF third = points[thirdIndex];
    signature.orientation = (tip.x() - base.x()) * (third.y() - base.y())
        - (tip.y() - base.y()) * (third.x() - base.x());
    signature.areaScore = std::clamp(
        std::fabs(signature.orientation) / std::max(1.0, edges[2].length * edges[2].length),
        0.0,
        1.0);

    return signature;
}

std::array<QPointF, 4> CameraPlateSolver::SolverContext::orderQuadPoints(const std::array<QPointF, 4>& points)
{
    QPointF centroid;
    for (const QPointF& point : points) {
        centroid += point;
    }
    centroid /= 4.0;

    std::array<int, 4> order{{0, 1, 2, 3}};
    std::sort(order.begin(), order.end(), [&points, &centroid](int lhs, int rhs) {
        return std::atan2(points[lhs].y() - centroid.y(), points[lhs].x() - centroid.x())
            < std::atan2(points[rhs].y() - centroid.y(), points[rhs].x() - centroid.x());
    });

    std::array<QPointF, 4> orderedPoints;
    for (int i = 0; i < 4; ++i) {
        orderedPoints[i] = points[order[i]];
    }
    return orderedPoints;
}

CameraPlateSolver::SolverContext::QuadSignature CameraPlateSolver::SolverContext::buildQuadSignature(const std::array<QPointF, 4>& unorderedPoints)
{
    QuadSignature signature;
    const std::array<QPointF, 4> points = orderQuadPoints(unorderedPoints);

    std::array<double, 6> distances{{
        QLineF(points[0], points[1]).length(),
        QLineF(points[0], points[2]).length(),
        QLineF(points[0], points[3]).length(),
        QLineF(points[1], points[2]).length(),
        QLineF(points[1], points[3]).length(),
        QLineF(points[2], points[3]).length()
    }};
    std::sort(distances.begin(), distances.end());
    if (distances[5] <= 1e-6) {
        return signature;
    }

    signature.longestDistance = distances[5];
    for (int i = 0; i < 5; ++i) {
        signature.edgeRatios[i] = distances[i] / distances[5];
    }

    double twiceArea = 0.0;
    for (int i = 0; i < 4; ++i) {
        const QPointF& a = points[i];
        const QPointF& b = points[(i + 1) % 4];
        twiceArea += (a.x() * b.y()) - (a.y() * b.x());
    }
    signature.orientation = twiceArea;
    signature.areaScore = std::clamp(
        std::fabs(twiceArea) / std::max(1.0, distances[5] * distances[5]),
        0.0,
        1.0);
    return signature;
}

bool CameraPlateSolver::SolverContext::isDistinctiveTriangleSignature(const TriangleSignature& signature)
{
    if (signature.longestDistance <= 0.0) {
        return false;
    }
    if (signature.areaScore < 0.025) {
        return false;
    }
    if (signature.ratioShortToLong < 0.10) {
        return false;
    }
    return std::fabs(signature.ratioMidToLong - signature.ratioShortToLong) > 0.015;
}

bool CameraPlateSolver::SolverContext::isDistinctiveQuadSignature(const QuadSignature& signature)
{
    if (signature.longestDistance <= 0.0) {
        return false;
    }
    if (signature.areaScore < 0.025) {
        return false;
    }
    return std::fabs(signature.edgeRatios[1] - signature.edgeRatios[0]) > 0.010;
}

double CameraPlateSolver::SolverContext::catalogSignatureBrightnessScore(const QVector<VisibleCatalogStar>& visibleStars, const int *indices, int count)
{
    double score = 0.0;
    for (int i = 0; i < count; ++i)
    {
        const int starIndex = indices[i];
        if ((starIndex < 0) || (starIndex >= visibleStars.size())) {
            continue;
        }
        const double magnitude = visibleStars[starIndex].magnitude;
        score += std::isfinite(magnitude)
            ? 1.0 / (1.0 + std::exp((magnitude - 8.0) * 0.45))
            : 0.0;
    }
    return score / std::max(1, count);
}

qint64 CameraPlateSolver::SolverContext::signatureBucketKey(int firstBin, int secondBin)
{
    return (static_cast<qint64>(firstBin) << 32)
        ^ static_cast<quint32>(secondBin);
}

int CameraPlateSolver::SolverContext::signatureRatioBin(double value, double binWidth)
{
    return static_cast<int>(std::floor(value / std::max(1e-6, binWidth)));
}

QHash<qint64, QVector<int>> CameraPlateSolver::SolverContext::buildTriangleSignatureBuckets(const QVector<TriangleSignature>& signatures, double binWidth)
{
    QHash<qint64, QVector<int>> buckets;
    for (int i = 0; i < signatures.size(); ++i)
    {
        const TriangleSignature& signature = signatures.at(i);
        const int firstBin = signatureRatioBin(signature.ratioShortToLong, binWidth);
        const int secondBin = signatureRatioBin(signature.ratioMidToLong, binWidth);
        buckets[signatureBucketKey(firstBin, secondBin)].append(i);
    }
    return buckets;
}

QHash<qint64, QVector<int>> CameraPlateSolver::SolverContext::buildQuadSignatureBuckets(const QVector<QuadSignature>& signatures, double binWidth)
{
    QHash<qint64, QVector<int>> buckets;
    for (int i = 0; i < signatures.size(); ++i)
    {
        const QuadSignature& signature = signatures.at(i);
        const int firstBin = signatureRatioBin(signature.edgeRatios[0], binWidth);
        const int secondBin = signatureRatioBin(signature.edgeRatios[1], binWidth);
        buckets[signatureBucketKey(firstBin, secondBin)].append(i);
    }
    return buckets;
}

double CameraPlateSolver::SolverContext::medianCandidateDistancePixels(const QVector<Match>& matches)
{
    if (matches.isEmpty()) {
        return 0.0;
    }

    QVector<double> distances;
    distances.reserve(matches.size());
    for (const Match& match : matches) {
        distances.append(match.distancePixels);
    }

    std::sort(distances.begin(), distances.end());
    const int middle = distances.size() / 2;
    if ((distances.size() % 2) == 0) {
        return 0.5 * (distances[middle - 1] + distances[middle]);
    }
    return distances[middle];
}

double CameraPlateSolver::SolverContext::seedWrappedAngularDistanceDegrees(double lhs, double rhs)
{
    double delta = std::fabs(lhs - rhs);
    while (delta > 360.0) {
        delta -= 360.0;
    }
    return delta > 180.0 ? 360.0 - delta : delta;
}

double CameraPlateSolver::SolverContext::seedCandidateConsensusScore(const Evaluation& candidate)
{
    if (!candidate.valid) {
        return -std::numeric_limits<double>::infinity();
    }

    const double rmsPenalty = std::isfinite(candidate.rmsErrorPixels)
        ? candidate.rmsErrorPixels * 0.10
        : 1000.0;
    const double magnitudePenalty = std::isfinite(candidate.meanCatalogMagnitude)
        ? std::max(0.0, candidate.meanCatalogMagnitude - 8.0) * 0.25
        : 0.0;
    const double anchorBonus =
        (candidate.anchored ? 2.0 : 0.0)
        + (candidate.guidedTriangle ? 2.0 : 0.0)
        + (candidate.sparseGuidedPair ? 1.0 : 0.0);
    return candidate.matchCount * 4.0
        + candidate.matches.size() * 0.5
        + anchorBonus
        - rmsPenalty
        - magnitudePenalty;
}

qint64 CameraPlateSolver::SolverContext::catalogTriangleKey(int a, int b, int c)
{
    std::array<int, 3> sorted {{a, b, c}};
    std::sort(sorted.begin(), sorted.end());
    return (static_cast<qint64>(sorted[0]) << 42)
        | (static_cast<qint64>(sorted[1]) << 21)
        | static_cast<qint64>(sorted[2]);
}

CameraPlateSolver::SolverContext::SkyProjector CameraPlateSolver::SolverContext::buildDetectionRayVectors(const CameraSettings& settings, const QSize& imageSize, double referenceFovDegrees, double centerOffsetXPixels, double centerOffsetYPixels, double distortionK1, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, QVector<SkyVector>& detectionRayVectors, QVector<quint8>& detectionRayVectorValid)
{
    const SkyProjector referenceProjector = createProjector(
        settings,
        imageSize,
        0.0,
        90.0,
        0.0,
        referenceFovDegrees,
        centerOffsetXPixels,
        centerOffsetYPixels,
        distortionK1);

    // QVector::assign() is Qt6-only; fill(value, size) resizes+fills on both Qt5 and Qt6.
    detectionRayVectors.fill(SkyVector{0.0, 0.0, 0.0}, detectionIndices.size());
    detectionRayVectorValid.fill(0, detectionIndices.size());
    if (!referenceProjector.valid) {
        return referenceProjector;
    }

    for (int i = 0; i < detectionIndices.size(); ++i)
    {
        const int detectionIndex = detectionIndices[i];
        if ((detectionIndex >= 0)
            && (detectionIndex < starDetections.size())
            && unprojectPixelToVector(referenceProjector, starDetections[detectionIndex].m_center, detectionRayVectors[i]))
        {
            detectionRayVectorValid[i] = 1;
        }
    }
    return referenceProjector;
}

CameraPlateSolver::SolverContext::CatalogQuadCodeIndex CameraPlateSolver::SolverContext::buildCatalogQuadCodeIndex(const QVector<VisibleCatalogStar>& visibleStars, bool includeScaleDimension, int brightPoolLimit)
{
    CatalogQuadCodeIndex index;
    index.dimensions = includeScaleDimension ? 5 : 4;
    if (visibleStars.size() < 4) {
        return index;
    }

    // Bright-pool mode: build an unstructured C(k,4) index over the brightest
    // brightPoolLimit catalog stars (visibleStars is magnitude-sorted, brightest
    // first), mirroring the proven legacy buildCatalogQuadSignatures support.
    // This aligns the catalog quad support with the detection side (which forms
    // quads from the brightest ~16 detections): the anchor+6-nearest-neighbour
    // structure below draws neighbours from ALL visible stars, so a bright
    // anchor's nearest catalog neighbours are faint nearby stars that differ
    // from its nearest *detected* neighbours - giving 0 code matches on wide
    // fisheye mode-1 images (e.g. stars-wide-2.jpg). A shared C(k,4)-over-bright
    // pool guarantees any 4 detected bright stars have their quad in the index.
    if (brightPoolLimit > 0) {
        const int poolSize = std::min<int>(brightPoolLimit, visibleStars.size());
        for (int a = 0; a < poolSize; ++a) {
            for (int b = a + 1; b < poolSize; ++b) {
                for (int c = b + 1; c < poolSize; ++c) {
                    for (int d = c + 1; d < poolSize; ++d) {
                        const std::array<int, 4> starIndices = {{a, b, c, d}};
                        const std::array<SkyVector, 4> vectors = {{
                            visibleStars[a].vector,
                            visibleStars[b].vector,
                            visibleStars[c].vector,
                            visibleStars[d].vector
                        }};
                        const QuadVectorCode quadCode = buildVectorQuadCode(vectors);
                        if (!quadCode.valid) {
                            continue;
                        }
                        CatalogQuadCodeEntry entry;
                        entry.code[0] = quadCode.xC;
                        entry.code[1] = quadCode.yC;
                        entry.code[2] = quadCode.xD;
                        entry.code[3] = quadCode.yD;
                        entry.code[4] = quadCode.angleABRadians;
                        for (int k = 0; k < 4; ++k) {
                            entry.visibleStarIndices[k] = starIndices[quadCode.order[k]];
                        }
                        index.addEntry(entry);
                    }
                }
            }
        }
        return index;
    }

    constexpr double cellSizeDegrees = 10.0;
    constexpr int maxStarsPerCell = 6;
    constexpr int neighborCount = 6;
    constexpr int maxQuadsPerAnchor = 8;
    constexpr int maxTotalQuads = 50000;
    constexpr double minNeighborSeparationDegrees = 2.0;
    constexpr double maxNeighborSeparationDegrees = 70.0;

    QHash<quint64, QVector<int>> cellStars; // az/el cell key -> visibleStars indices
    for (int i = 0; i < visibleStars.size(); ++i)
    {
        const qint64 azCell = static_cast<qint64>(std::floor(visibleStars[i].azimuthDegrees / cellSizeDegrees));
        const qint64 elCell = static_cast<qint64>(std::floor((visibleStars[i].elevationDegrees + 90.0) / cellSizeDegrees));
        const quint64 key = (static_cast<quint64>(azCell + 64) << 16) | static_cast<quint64>(elCell + 64);
        cellStars[key].append(i);
    }

    QVector<int> anchorIndices;
    for (auto it = cellStars.begin(); it != cellStars.end(); ++it)
    {
        QVector<int>& starsInCell = it.value();
        std::sort(starsInCell.begin(), starsInCell.end(), [&](int lhs, int rhs) {
            return visibleStars[lhs].magnitude < visibleStars[rhs].magnitude;
        });
        const int keepCount = std::min<int>(maxStarsPerCell, starsInCell.size());
        for (int j = 0; j < keepCount; ++j) {
            anchorIndices.append(starsInCell[j]);
        }
    }

    const double minNeighborDot = std::cos(degToRad(maxNeighborSeparationDegrees));
    const double maxNeighborDot = std::cos(degToRad(minNeighborSeparationDegrees));

    for (int anchor : anchorIndices)
    {
        if (index.entries.size() >= maxTotalQuads) {
            break;
        }

        QVector<std::pair<double, int>> neighborDistances;
        neighborDistances.reserve(visibleStars.size());
        for (int j = 0; j < visibleStars.size(); ++j)
        {
            if (j == anchor) {
                continue;
            }
            const double angleDot = std::clamp(dot(visibleStars[anchor].vector, visibleStars[j].vector), -1.0, 1.0);
            // Restrict neighbours to the design's scale band: too-close stars don't add
            // distinguishing geometry, too-far stars rarely co-occur within one detection FoV.
            if ((angleDot < minNeighborDot) || (angleDot > maxNeighborDot)) {
                continue;
            }
            neighborDistances.append({std::acos(angleDot), j});
        }

        const int kCount = std::min<int>(neighborCount, static_cast<int>(neighborDistances.size()));
        if (kCount < 3) {
            continue;
        }
        std::partial_sort(neighborDistances.begin(), neighborDistances.begin() + kCount, neighborDistances.end(),
            [](const std::pair<double, int>& lhs, const std::pair<double, int>& rhs) {
                return lhs.first < rhs.first;
            });

        int quadsForAnchor = 0;
        for (int a = 0; (a < kCount) && (quadsForAnchor < maxQuadsPerAnchor); ++a)
        {
            for (int b = a + 1; (b < kCount) && (quadsForAnchor < maxQuadsPerAnchor); ++b)
            {
                for (int c = b + 1; (c < kCount) && (quadsForAnchor < maxQuadsPerAnchor); ++c)
                {
                    const std::array<int, 4> starIndices = {{
                        anchor,
                        neighborDistances[a].second,
                        neighborDistances[b].second,
                        neighborDistances[c].second
                    }};
                    const std::array<SkyVector, 4> vectors = {{
                        visibleStars[starIndices[0]].vector,
                        visibleStars[starIndices[1]].vector,
                        visibleStars[starIndices[2]].vector,
                        visibleStars[starIndices[3]].vector
                    }};
                    const QuadVectorCode quadCode = buildVectorQuadCode(vectors);
                    if (!quadCode.valid) {
                        continue;
                    }

                    CatalogQuadCodeEntry entry;
                    entry.code[0] = quadCode.xC;
                    entry.code[1] = quadCode.yC;
                    entry.code[2] = quadCode.xD;
                    entry.code[3] = quadCode.yD;
                    entry.code[4] = quadCode.angleABRadians;
                    for (int k = 0; k < 4; ++k) {
                        entry.visibleStarIndices[k] = starIndices[quadCode.order[k]];
                    }
                    index.addEntry(entry);
                    ++quadsForAnchor;
                    if (index.entries.size() >= maxTotalQuads) {
                        break;
                    }
                }
                if (index.entries.size() >= maxTotalQuads) {
                    break;
                }
            }
            if (index.entries.size() >= maxTotalQuads) {
                break;
            }
        }
    }

    return index;
}

quint64 CameraPlateSolver::SolverContext::spatialCellKey(int x, int y)
{
    return (static_cast<quint64>(static_cast<quint32>(x)) << 32)
        | static_cast<quint32>(y);
}

double CameraPlateSolver::SolverContext::angularDistanceDegrees(double lhs, double rhs)
{
    double delta = std::fabs(lhs - rhs);
    while (delta > 360.0) {
        delta -= 360.0;
    }
    if (delta > 180.0) {
        delta = 360.0 - delta;
    }
    return delta;
}

double CameraPlateSolver::SolverContext::meanCatalogMagnitudeForMatches(const QVector<CatalogStar>& catalogStars, const QVector<Match>& matches)
{
    double sumMagnitude = 0.0;
    int count = 0;

    for (const Match& match : matches)
    {
        if ((match.catalogIndex < 0) || (match.catalogIndex >= catalogStars.size())) {
            continue;
        }

        const double magnitude = catalogStars[match.catalogIndex].magnitude;
        if (!std::isfinite(magnitude)) {
            continue;
        }

        sumMagnitude += magnitude;
        ++count;
    }

    return count > 0 ? sumMagnitude / count : std::numeric_limits<double>::infinity();
}

void CameraPlateSolver::SolverContext::appendSupplementalMatches(const QVector<CameraPipelineStarDetection>& starDetections, const QVector<ProjectedCatalogStar>& projectedStars, double matchRadiusPixels, const QVector<int>* supplementalDetectionIndices, bool narrowGuidedSolve, bool allowTightArtifactCoincidence, QVector<Match>& matches)
{
    QVector<bool> detectionMatched(starDetections.size(), false);
    QHash<int, bool> catalogMatched;
    catalogMatched.reserve(matches.size());
    for (const Match& match : matches)
    {
        // Match::detectionIndex defaults to -1; guard the QVector write (the catalog side is a
        // QHash, so a stray key is harmless). Mirrors the guard in appendWideBrightSupplementalMatches.
        if ((match.detectionIndex >= 0) && (match.detectionIndex < detectionMatched.size())) {
            detectionMatched[match.detectionIndex] = true;
        }
        catalogMatched.insert(match.catalogIndex, true);
    }

    // Bound supplemental matches to a quality envelope derived from the already-accepted
    // inlier matches. Without this gate, every unmatched detection within the loose radius
    // would be promoted to a match, inflating m_matchedStars and corrupting the reported
    // RMS with what the bipartite assignment had already rejected.
    const double medianAcceptedDistance = medianCandidateDistancePixels(matches);
    const double supplementalRadiusCap = (medianAcceptedDistance > 0.0)
        ? std::min(matchRadiusPixels, std::max(2.0, medianAcceptedDistance * 1.5))
        : matchRadiusPixels;
    const double maxDistanceSquared = supplementalRadiusCap * supplementalRadiusCap;

    // Pre-sort projected stars into a spatial grid to avoid the O(D x P) loop that the
    // original implementation used; for D=32 detections and P=500 catalog stars this cut
    // ~16k coordinate compares per call.
    const double cellSize = std::max(1.0, supplementalRadiusCap);
    QHash<quint64, QVector<int>> projectedGrid;
    projectedGrid.reserve(projectedStars.size());
    for (int projectedIndex = 0; projectedIndex < projectedStars.size(); ++projectedIndex)
    {
        const QPointF& point = projectedStars[projectedIndex].point;
        const int cellX = static_cast<int>(std::floor(point.x() / cellSize));
        const int cellY = static_cast<int>(std::floor(point.y() / cellSize));
        projectedGrid[spatialCellKey(cellX, cellY)].append(projectedIndex);
    }

    // A hot-pixel-suspect detection is normally excluded from supplemental matching (its
    // compact/low-fill signature is shared with hot pixels and cosmic-ray spikes). But in a
    // WIDE FISHEYE real stars are heavily undersampled (~1 px) and routinely get flagged
    // suspect, and a TIGHT positional coincidence with a projected catalogue star at the solved
    // pose is strong evidence the detection is that real star: an artifact is fixed to the
    // sensor and would not land on a moving sky position (the single-frame equivalent of the
    // star tracking across frames). So for wide fisheye only, admit a hot-pixel-suspect
    // detection -- and waive the compact-bright guard for it -- when it falls within this tight
    // radius (well inside the loose supplemental cap). Non-suspect detections and narrow fields
    // keep the original behaviour, so genuine artifacts within the loose radius stay rejected.
    // The exception is deliberately narrow: a tight radius, and only for a genuinely BRIGHT
    // detection coinciding with a BRIGHT catalogue star. A real bright star mis-flagged as a hot
    // pixel (Dziban: snr~134, mag 4.57) fits this; faint compact artifacts -- the dangerous,
    // common case in dense/synthetic frames -- do not, so they stay excluded.
    const double tightCoincidenceRadius = std::min(supplementalRadiusCap, std::max(2.0, matchRadiusPixels * 0.10));
    const double tightCoincidenceRadiusSquared = tightCoincidenceRadius * tightCoincidenceRadius;
    // Lowered from 100 (tuned for Dziban, snr~134) to 50 so genuine naked-eye stars (mag ~5-6,
    // snr ~50-70 when heavily undersampled in a wide field, e.g. eps2 Lyr snr~67) recover their
    // label. The tight coincidence with a bright (mag <= 6.5) catalogue star is the real
    // discriminator; this is label-only/post-acceptance so it cannot change a solve verdict.
    constexpr double kSuspectCoincidenceMinSnr = 50.0;
    constexpr double kSuspectCoincidenceMaxCatalogMag = 6.5;

    const auto appendSupplementalForDetection = [&](int detectionIndex)
    {
        if ((detectionIndex < 0)
            || (detectionIndex >= starDetections.size())
            || detectionMatched[detectionIndex])
        {
            return;
        }
        const bool detectionSuspect = starDetections[detectionIndex].m_hotPixelSuspect;
        // Hot-pixel-suspect detections are admitted only via the wide-fisheye tight-coincidence
        // exception, and only when the detection itself is bright; otherwise keep the original
        // outright exclusion.
        if (detectionSuspect
            && (!allowTightArtifactCoincidence
                || (static_cast<double>(starDetections[detectionIndex].m_snr) < kSuspectCoincidenceMinSnr)))
        {
            return;
        }

        const QPointF detectionPoint = starDetections[detectionIndex].m_center;
        const int cellX = static_cast<int>(std::floor(detectionPoint.x() / cellSize));
        const int cellY = static_cast<int>(std::floor(detectionPoint.y() / cellSize));
        double bestDistanceSquared = std::numeric_limits<double>::max();
        double bestScore = std::numeric_limits<double>::max();
        int bestCatalogIndex = -1;

        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                const auto it = projectedGrid.constFind(spatialCellKey(cellX + dx, cellY + dy));
                if (it == projectedGrid.cend()) {
                    continue;
                }
                for (int projectedIndex : it.value())
                {
                    const ProjectedCatalogStar& projected = projectedStars[projectedIndex];
                    if (catalogMatched.contains(projected.catalogIndex)) {
                        continue;
                    }
                    const double dxp = detectionPoint.x() - projected.point.x();
                    const double dyp = detectionPoint.y() - projected.point.y();
                    const double distanceSquared = dxp * dxp + dyp * dyp;
                    if (distanceSquared > maxDistanceSquared) {
                        continue;
                    }
                    // A hot-pixel-suspect detection (wide fisheye) is admitted only on a tight
                    // coincidence; the compact-bright guard is waived for exactly that case.
                    const bool tightSuspectCoincidence = detectionSuspect
                        && allowTightArtifactCoincidence
                        && (distanceSquared <= tightCoincidenceRadiusSquared)
                        && (projected.magnitude <= kSuspectCoincidenceMaxCatalogMag);
                    if (detectionSuspect && !tightSuspectCoincidence) {
                        continue;
                    }
                    const bool compactReject = isImplausiblyCompactBrightCatalogDetection(
                        starDetections[detectionIndex],
                        projected.magnitude,
                        narrowGuidedSolve);
                    if (compactReject && !tightSuspectCoincidence) {
                        continue;
                    }
                    const double distancePixels = std::sqrt(distanceSquared);
                    const double score = distancePixels
                        + supplementalRadiusCap * faintCatalogAssignmentPenalty(projected.magnitude)
                        - std::min(supplementalRadiusCap * 0.15,
                            std::log1p(detectionReliabilityMetric(starDetections[detectionIndex])));
                    if (score < bestScore)
                    {
                        bestScore = score;
                        bestDistanceSquared = distanceSquared;
                        bestCatalogIndex = projected.catalogIndex;
                    }
                }
            }
        }

        if (bestCatalogIndex >= 0)
        {
            const double distancePixels = std::sqrt(bestDistanceSquared);
            matches.append({detectionIndex, bestCatalogIndex, distancePixels});
            detectionMatched[detectionIndex] = true;
            catalogMatched.insert(bestCatalogIndex, true);
            if (kLogPlateSolveCandidates) {
                qDebug() << "CameraPlateSolver: supplemental final match"
                         << "detection=" << detectionIndex
                         << "catalog=" << bestCatalogIndex
                         << "distance=" << distancePixels;
            }
        }
    };

    if (supplementalDetectionIndices)
    {
        for (int detectionIndex : *supplementalDetectionIndices)
        {
            if ((detectionIndex >= 0) && (detectionIndex < starDetections.size())) {
                appendSupplementalForDetection(detectionIndex);
            }
        }
    }
    else
    {
        for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex) {
            appendSupplementalForDetection(detectionIndex);
        }
    }
}

void CameraPlateSolver::SolverContext::appendWideBrightSupplementalMatches(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<ProjectedCatalogStar>& projectedStars, const QSize& imageSize, double matchRadiusPixels, QVector<Match>& matches)
{
    if (!isWidePlateSolveContext(settings)
        || matches.isEmpty())
    {
        return;
    }

    const double brightRadius = std::min(
        matchRadiusPixels + 6.0,
        std::max(matchRadiusPixels, matchRadiusPixels * 1.25));
    if (brightRadius <= matchRadiusPixels) {
        return;
    }

    QVector<bool> detectionMatched(starDetections.size(), false);
    QHash<int, bool> catalogMatched;
    catalogMatched.reserve(matches.size());
    for (const Match& match : matches)
    {
        if ((match.detectionIndex >= 0) && (match.detectionIndex < detectionMatched.size())) {
            detectionMatched[match.detectionIndex] = true;
        }
        catalogMatched.insert(match.catalogIndex, true);
    }

    struct BrightCandidate
    {
        int detectionIndex = -1;
        int catalogIndex = -1;
        double distancePixels = 0.0;
        double magnitude = 0.0;
    };
    QVector<BrightCandidate> candidates;
    const double brightRadiusSquared = brightRadius * brightRadius;
    const QRectF imageBounds(
        -brightRadius,
        -brightRadius,
        imageSize.width() + 2.0 * brightRadius,
        imageSize.height() + 2.0 * brightRadius);

    for (const ProjectedCatalogStar& projectedStar : projectedStars)
    {
        if ((projectedStar.magnitude > 2.5)
            || catalogMatched.contains(projectedStar.catalogIndex)
            || !imageBounds.contains(projectedStar.point))
        {
            continue;
        }

        int nearestDetectionIndex = -1;
        double nearestDistanceSquared = brightRadiusSquared;
        for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
        {
            if (detectionMatched[detectionIndex]
                || !isDetectionUsableForBrightPrior(starDetections[detectionIndex]))
            {
                continue;
            }
            if (isImplausiblyCompactBrightCatalogDetection(starDetections[detectionIndex], projectedStar.magnitude)) {
                continue;
            }

            const QPointF delta = starDetections[detectionIndex].m_center - projectedStar.point;
            const double distanceSquared = delta.x() * delta.x() + delta.y() * delta.y();
            if (distanceSquared < nearestDistanceSquared)
            {
                nearestDistanceSquared = distanceSquared;
                nearestDetectionIndex = detectionIndex;
            }
        }

        if (nearestDetectionIndex >= 0)
        {
            candidates.append({
                nearestDetectionIndex,
                projectedStar.catalogIndex,
                std::sqrt(nearestDistanceSquared),
                projectedStar.magnitude
            });
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const BrightCandidate& lhs, const BrightCandidate& rhs) {
        if (!qFuzzyCompare(lhs.magnitude + 1.0, rhs.magnitude + 1.0)) {
            return lhs.magnitude < rhs.magnitude;
        }
        return lhs.distancePixels < rhs.distancePixels;
    });

    int appended = 0;
    for (const BrightCandidate& candidate : candidates)
    {
        if (appended >= 4) {
            break;
        }
        if (detectionMatched[candidate.detectionIndex]
            || catalogMatched.contains(candidate.catalogIndex))
        {
            continue;
        }

        detectionMatched[candidate.detectionIndex] = true;
        catalogMatched.insert(candidate.catalogIndex, true);
        matches.append({candidate.detectionIndex, candidate.catalogIndex, candidate.distancePixels});
        ++appended;
    }
}

bool CameraPlateSolver::SolverContext::hasDenseFinalEvidenceOverridingSeedRadial(const CameraSettings& settings, const FinalMatchPassEvaluation& finalPass)
{
    return !usesSeedProjectedBrightGate(settings)
        && (finalPass.finalMatches.size() >= static_cast<qsizetype>(std::max(settings.m_plateSolveMinMatches + 50, 80)))
        && (finalPass.matchedBrightDetections >= std::min(12, std::max(4, finalPass.brightDetections / 2)))
        && (finalPass.matchedBrightProjectedStars >= std::min(8, std::max(3, finalPass.brightProjectedStars / 2)))
        && (finalPass.matchedProjectedMagnitudeSupport >= 60.0)
        && (finalPass.brightDetectionMagnitudeError <= 1.30)
        && (!std::isfinite(finalPass.brightnessRankError) || (finalPass.brightnessRankError <= 0.30))
        && (finalPass.rmsErrorPixels <= std::min(
                static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.60,
                16.0));
}

bool CameraPlateSolver::SolverContext::isBetterByGeometricTieBreak(const Evaluation& candidate, const Evaluation& best)
{
    if (candidate.matchCount != best.matchCount) {
        return candidate.matchCount > best.matchCount;
    }
    if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
        return candidate.rmsErrorPixels < best.rmsErrorPixels;
    }
    const double candidateCalibrationMagnitude = calibrationMagnitude(candidate);
    const double bestCalibrationMagnitude = calibrationMagnitude(best);
    if (!qFuzzyCompare(candidateCalibrationMagnitude + 1.0, bestCalibrationMagnitude + 1.0)) {
        return candidateCalibrationMagnitude < bestCalibrationMagnitude;
    }

    return candidate.fovDegrees == best.fovDegrees
        ? candidate.rollDegrees < best.rollDegrees
        : candidate.fovDegrees < best.fovDegrees;
}

bool CameraPlateSolver::SolverContext::isBetterEvaluation(const Evaluation& candidate, const Evaluation& best)
{
    if (!candidate.valid) {
        return false;
    }
    if (!best.valid) {
        return true;
    }
    return isBetterByGeometricTieBreak(candidate, best);
}

bool CameraPlateSolver::SolverContext::sameEvaluationBasin(const Evaluation& lhs, const Evaluation& rhs)
{
    if (lhs.anchored != rhs.anchored) {
        return false;
    }
    if (lhs.anchored
        && ((lhs.anchorDetectionIndex != rhs.anchorDetectionIndex)
            || (lhs.anchorCatalogIndex != rhs.anchorCatalogIndex)
            || (lhs.secondaryAnchorDetectionIndex != rhs.secondaryAnchorDetectionIndex)
            || (lhs.secondaryAnchorCatalogIndex != rhs.secondaryAnchorCatalogIndex)
            || (lhs.guidedTriangle != rhs.guidedTriangle)
            || (lhs.tertiaryAnchorDetectionIndex != rhs.tertiaryAnchorDetectionIndex)
            || (lhs.tertiaryAnchorCatalogIndex != rhs.tertiaryAnchorCatalogIndex)))
    {
        return false;
    }

    return angularDistanceDegrees(lhs.azimuthDegrees, rhs.azimuthDegrees) <= 20.0
        && std::fabs(lhs.elevationDegrees - rhs.elevationDegrees) <= 10.0
        && angularDistanceDegrees(lhs.rollDegrees, rhs.rollDegrees) <= 20.0
        && std::fabs(lhs.fovDegrees - rhs.fovDegrees) <= 10.0;
}

bool CameraPlateSolver::SolverContext::sameEvaluationIdentity(const Evaluation& lhs, const Evaluation& rhs)
{
    return lhs.valid == rhs.valid
        && lhs.anchored == rhs.anchored
        && lhs.sparseGuidedPair == rhs.sparseGuidedPair
        && lhs.guidedTriangle == rhs.guidedTriangle
        && lhs.anchorDetectionIndex == rhs.anchorDetectionIndex
        && lhs.anchorCatalogIndex == rhs.anchorCatalogIndex
        && lhs.secondaryAnchorDetectionIndex == rhs.secondaryAnchorDetectionIndex
        && lhs.secondaryAnchorCatalogIndex == rhs.secondaryAnchorCatalogIndex
        && lhs.tertiaryAnchorDetectionIndex == rhs.tertiaryAnchorDetectionIndex
        && lhs.tertiaryAnchorCatalogIndex == rhs.tertiaryAnchorCatalogIndex
        && lhs.matchCount == rhs.matchCount
        && qFuzzyCompare(lhs.azimuthDegrees + 1.0, rhs.azimuthDegrees + 1.0)
        && qFuzzyCompare(lhs.elevationDegrees + 1.0, rhs.elevationDegrees + 1.0)
        && qFuzzyCompare(lhs.rollDegrees + 1.0, rhs.rollDegrees + 1.0)
        && qFuzzyCompare(lhs.fovDegrees + 1.0, rhs.fovDegrees + 1.0)
        && qFuzzyCompare(lhs.rmsErrorPixels + 1.0, rhs.rmsErrorPixels + 1.0)
        && qFuzzyCompare(lhs.centerOffsetXPixels + 1.0, rhs.centerOffsetXPixels + 1.0)
        && qFuzzyCompare(lhs.centerOffsetYPixels + 1.0, rhs.centerOffsetYPixels + 1.0)
        && qFuzzyCompare(lhs.distortionK1 + 1.0, rhs.distortionK1 + 1.0);
}

double CameraPlateSolver::SolverContext::plateSolveLmParameterValue(const PlateSolveLmPose& pose, PlateSolveLmParameter parameter)
{
    switch (parameter)
    {
    case PlateSolveLmAzimuth:
        return pose.azimuthDegrees;
    case PlateSolveLmElevation:
        return pose.elevationDegrees;
    case PlateSolveLmRoll:
        return pose.rollDegrees;
    case PlateSolveLmFov:
        return pose.fovDegrees;
    case PlateSolveLmCenterX:
        return pose.centerOffsetXPixels;
    case PlateSolveLmCenterY:
        return pose.centerOffsetYPixels;
    case PlateSolveLmDistortionK1:
        return pose.distortionK1;
    default:
        return 0.0;
    }
}

double CameraPlateSolver::SolverContext::plateSolveLmFiniteDifferenceStep(const PlateSolveLmPose& pose, PlateSolveLmParameter parameter)
{
    switch (parameter)
    {
    case PlateSolveLmAzimuth:
    case PlateSolveLmElevation:
    case PlateSolveLmRoll:
        return std::max(1e-4, std::min(0.02, pose.fovDegrees * 0.001));
    case PlateSolveLmFov:
        return std::max(1e-4, pose.fovDegrees * 1e-4);
    case PlateSolveLmCenterX:
    case PlateSolveLmCenterY:
        return 0.25;
    case PlateSolveLmDistortionK1:
        return 1e-4;
    default:
        return 1e-4;
    }
}

double CameraPlateSolver::SolverContext::plateSolveLmMaximumStep(const QSize& imageSize, const PlateSolveLmPose& pose, PlateSolveLmParameter parameter)
{
    switch (parameter)
    {
    case PlateSolveLmAzimuth:
    case PlateSolveLmElevation:
        return std::max(0.05, std::min(5.0, pose.fovDegrees * 0.75));
    case PlateSolveLmRoll:
        return 30.0;
    case PlateSolveLmFov:
        return std::max(0.02, pose.fovDegrees * 0.25);
    case PlateSolveLmCenterX:
        return std::max(1.0, static_cast<double>(imageSize.width()) * 0.05);
    case PlateSolveLmCenterY:
        return std::max(1.0, static_cast<double>(imageSize.height()) * 0.05);
    case PlateSolveLmDistortionK1:
        return 0.10;
    default:
        return 1.0;
    }
}

QVector<int> CameraPlateSolver::SolverContext::detectionIndicesForMatches(const QVector<Match>& matches)
{
    QVector<int> detectionIndices;
    QSet<int> seen;
    detectionIndices.reserve(matches.size());
    for (const Match& match : matches)
    {
        if (seen.contains(match.detectionIndex)) {
            continue;
        }
        seen.insert(match.detectionIndex);
        detectionIndices.append(match.detectionIndex);
    }
    return detectionIndices;
}

void CameraPlateSolver::SolverContext::clearSolvedStars(QVector<CameraPipelineStarDetection>& starDetections)
{
    for (CameraPipelineStarDetection& detection : starDetections)
    {
        detection.m_label.clear();
        detection.m_projectedCenter = QPointF();
        detection.m_matchDistancePixels = 0.0f;
        detection.m_catalogMagnitude = 0.0f;
        detection.m_catalogRightAscensionDegrees = std::numeric_limits<double>::quiet_NaN();
        detection.m_catalogDeclinationDegrees = std::numeric_limits<double>::quiet_NaN();
        detection.m_catalogSpectralType.clear();
        detection.m_solved = false;
    }
}

