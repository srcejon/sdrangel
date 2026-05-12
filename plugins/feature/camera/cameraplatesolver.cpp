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

#include "cameraplatesolver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMutex>
#include <QMutexLocker>
#include <QPointF>
#include <QRectF>
#include <QStandardPaths>
#include <QString>
#include <QStringList>
#include <QVector>

#include <zlib.h>

#include "util/astronomy.h"

namespace {

struct CatalogStar
{
    QString name;
    double rightAscensionDegrees;
    double declinationDegrees;
    double magnitude;
};

struct SkyVector
{
    double x;
    double y;
    double z;
};

struct ProjectedCatalogStar
{
    int catalogIndex = -1;
    QPointF point;
};

struct CandidatePair
{
    int detectionIndex = -1;
    int catalogIndex = -1;
    double distancePixels = 0.0;
};

struct Match
{
    int detectionIndex = -1;
    int catalogIndex = -1;
    double distancePixels = 0.0;
};

struct Evaluation
{
    bool valid = false;
    int matchCount = 0;
    double rmsErrorPixels = std::numeric_limits<double>::infinity();
    double azimuthDegrees = 0.0;
    double elevationDegrees = 0.0;
    double rollDegrees = 0.0;
    double fovDegrees = 0.0;
    QVector<Match> matches;
};

struct SkyProjector
{
    bool valid = false;
    CameraSettings::LensProjection lensProjection = CameraSettings::LensProjectionRectilinear;
    SkyVector center;
    SkyVector right;
    SkyVector up;
    double halfHorizontalFov = 0.0;
    double horizontalScale = 1.0;
    double verticalScale = 1.0;
    int width = 0;
    int height = 0;
};

constexpr double kPi = 3.14159265358979323846;
constexpr double kVisibleAltitudeFloor = -5.0;
constexpr int kMaxDetectionsForSolve = 24;
const char* const kBundledCatalogPath = ":/camera/brightstarcatalog.txt";
const char* const kDownloadedCatalogDir = "camera";
const char* const kDownloadedCatalogArchiveFile = "hyg_v42.csv.gz";
const char* const kDownloadedCatalogCsvFile = "hyg_v42.csv";

double degToRad(double value)
{
    return value * kPi / 180.0;
}

double normalizeDegrees(double value)
{
    value = std::fmod(value, 360.0);
    if (value < 0.0) {
        value += 360.0;
    }
    return value;
}

SkyVector vectorFromAltAz(double azimuthDegrees, double elevationDegrees)
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

double dot(const SkyVector& lhs, const SkyVector& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

SkyVector cross(const SkyVector& lhs, const SkyVector& rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x
    };
}

double length(const SkyVector& vector)
{
    return std::sqrt(dot(vector, vector));
}

SkyVector normalize(const SkyVector& vector)
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

SkyVector rotateAroundAxis(const SkyVector& vector, const SkyVector& axis, double angleRadians)
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

double parseRightAscensionDegrees(const QString& value)
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

double parseDeclinationDegrees(const QString& value)
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

    const double sign = (degreesWithSign < 0.0) ? -1.0 : 1.0;
    const double absoluteDegrees = std::fabs(degreesWithSign);
    return sign * (absoluteDegrees + minutes / 60.0 + seconds / 3600.0);
}

QString stripQuotedField(const QString& value)
{
    QString stripped = value.trimmed();
    if (stripped.startsWith('"') && stripped.endsWith('"') && (stripped.size() >= 2)) {
        stripped = stripped.mid(1, stripped.size() - 2);
    }
    return stripped.replace(QStringLiteral("\"\""), QStringLiteral("\""));
}

QString downloadedCatalogDir()
{
    const QString baseDir = QStandardPaths::standardLocations(QStandardPaths::AppDataLocation).value(0);
    return QDir(baseDir).filePath(QString::fromUtf8(kDownloadedCatalogDir));
}

QByteArray gunzipData(const QByteArray& compressedData, QString* errorMessage)
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

QVector<CatalogStar> parseBundledCatalog(const QString& text)
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
        if (fields.size() != 4) {
            continue;
        }

        bool okMagnitude = false;
        const double rightAscensionDegrees = parseRightAscensionDegrees(fields[1].trimmed());
        const double declinationDegrees = parseDeclinationDegrees(fields[2].trimmed());
        const double magnitude = fields[3].trimmed().toDouble(&okMagnitude);
        if (!std::isfinite(rightAscensionDegrees) || !std::isfinite(declinationDegrees) || !okMagnitude) {
            continue;
        }

        stars.append({fields[0].trimmed(), rightAscensionDegrees, declinationDegrees, magnitude});
    }

    return stars;
}

QVector<CatalogStar> parseDownloadedHygCatalog(const QString& text)
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

        stars.append({name, rightAscensionHours * 15.0, declinationDegrees, magnitude});
    }

    return stars;
}

QVector<CatalogStar> loadCatalogFromTextFile(const QString& path)
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

QString currentCatalogPath(const CameraSettings& settings)
{
    if (settings.m_plateSolveUseDownloadedCatalog && QFileInfo::exists(CameraPlateSolver::downloadedCatalogCsvPath())) {
        return CameraPlateSolver::downloadedCatalogCsvPath();
    }
    return QString::fromUtf8(kBundledCatalogPath);
}

const QVector<CatalogStar>& brightStarCatalog(const CameraSettings& settings)
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

SkyProjector createProjector(const CameraSettings& settings,
                             const QSize& size,
                             double azimuthDegrees,
                             double elevationDegrees,
                             double rollDegrees,
                             double fovDegrees)
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

    const double halfHorizontalFov = degToRad(fovDegrees) * 0.5;
    if ((halfHorizontalFov <= 0.0) || (halfHorizontalFov >= (kPi * 0.5))) {
        return projector;
    }

    projector.halfHorizontalFov = halfHorizontalFov;
    const double aspect = static_cast<double>(projector.height) / static_cast<double>(projector.width);
    projector.horizontalScale = 1.0;
    projector.verticalScale = aspect;
    projector.valid = projector.verticalScale > 0.0;
    return projector;
}

bool projectAltAz(const SkyProjector& projector, double azimuthDegrees, double elevationDegrees, QPointF& point)
{
    if (!projector.valid) {
        return false;
    }

    const SkyVector vector = vectorFromAltAz(azimuthDegrees, elevationDegrees);
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

    const double normalizedX = std::cos(phi) * projectionRadius / projector.horizontalScale;
    const double normalizedY = std::sin(phi) * projectionRadius / projector.verticalScale;
    if (!std::isfinite(normalizedX) || !std::isfinite(normalizedY)) {
        return false;
    }

    point.setX((normalizedX + 1.0) * 0.5 * static_cast<double>(projector.width));
    point.setY((1.0 - normalizedY) * 0.5 * static_cast<double>(projector.height));
    return true;
}

QVector<int> selectDetectionIndicesForSolve(const QVector<CameraPipelineStarDetection>& starDetections)
{
    QVector<int> indices;
    indices.reserve(starDetections.size());
    for (int i = 0; i < starDetections.size(); ++i) {
        indices.append(i);
    }

    std::sort(indices.begin(), indices.end(), [&starDetections](int lhs, int rhs) {
        if (starDetections[lhs].m_peakValue != starDetections[rhs].m_peakValue) {
            return starDetections[lhs].m_peakValue > starDetections[rhs].m_peakValue;
        }

        return starDetections[lhs].m_radius > starDetections[rhs].m_radius;
    });

    if (indices.size() > kMaxDetectionsForSolve) {
        indices.resize(kMaxDetectionsForSolve);
    }

    return indices;
}

QVector<ProjectedCatalogStar> buildProjectedCatalog(const CameraSettings& settings,
                                                    const SkyProjector& projector,
                                                    const QDateTime& captureDateTimeUtc,
                                                    double maxMagnitude,
                                                    double searchMarginPixels)
{
    QVector<ProjectedCatalogStar> projectedStars;
    const QVector<CatalogStar>& catalogStars = brightStarCatalog(settings);

    for (int i = 0; i < catalogStars.size(); ++i)
    {
        const CatalogStar& star = catalogStars[i];
        if (star.magnitude > maxMagnitude) {
            continue;
        }

        const RADec raDec{star.rightAscensionDegrees / 15.0, star.declinationDegrees};
        const AzAlt azAlt = Astronomy::raDecToAzAlt(
            raDec,
            settings.m_latitude,
            settings.m_longitude,
            captureDateTimeUtc,
            true);

        if (!std::isfinite(azAlt.az) || !std::isfinite(azAlt.alt) || (azAlt.alt < kVisibleAltitudeFloor)) {
            continue;
        }

        QPointF point;
        if (!projectAltAz(projector, azAlt.az, azAlt.alt, point)) {
            continue;
        }

        const QRectF expandedBounds(
            -searchMarginPixels,
            -searchMarginPixels,
            projector.width + 2.0 * searchMarginPixels,
            projector.height + 2.0 * searchMarginPixels);
        if (!expandedBounds.contains(point)) {
            continue;
        }

        projectedStars.append({i, point});
    }

    return projectedStars;
}

QVector<Match> buildMatches(const CameraSettings& settings,
                            const QVector<CameraPipelineStarDetection>& starDetections,
                            const QVector<int>& detectionIndices,
                            const QVector<ProjectedCatalogStar>& projectedStars,
                            double matchRadiusPixels)
{
    QVector<CandidatePair> candidatePairs;
    const QVector<CatalogStar>& catalogStars = brightStarCatalog(settings);
    const double maxDistanceSquared = matchRadiusPixels * matchRadiusPixels;

    for (int detectionIndex : detectionIndices)
    {
        const QPointF detectionPoint = starDetections[detectionIndex].m_center;
        for (const ProjectedCatalogStar& projected : projectedStars)
        {
            const double dx = detectionPoint.x() - projected.point.x();
            const double dy = detectionPoint.y() - projected.point.y();
            const double distanceSquared = dx * dx + dy * dy;
            if (distanceSquared > maxDistanceSquared) {
                continue;
            }

            candidatePairs.append({
                detectionIndex,
                projected.catalogIndex,
                std::sqrt(distanceSquared)
            });
        }
    }

    std::sort(candidatePairs.begin(), candidatePairs.end(), [&catalogStars](const CandidatePair& lhs, const CandidatePair& rhs) {
        if (lhs.distancePixels != rhs.distancePixels) {
            return lhs.distancePixels < rhs.distancePixels;
        }

        return catalogStars[lhs.catalogIndex].magnitude < catalogStars[rhs.catalogIndex].magnitude;
    });

    QVector<Match> matches;
    QVector<bool> detectionMatched(starDetections.size(), false);
    QVector<bool> catalogMatched(catalogStars.size(), false);

    for (const CandidatePair& pair : candidatePairs)
    {
        if (detectionMatched[pair.detectionIndex] || catalogMatched[pair.catalogIndex]) {
            continue;
        }

        detectionMatched[pair.detectionIndex] = true;
        catalogMatched[pair.catalogIndex] = true;
        matches.append({pair.detectionIndex, pair.catalogIndex, pair.distancePixels});
    }

    return matches;
}

Evaluation evaluatePose(const CameraSettings& settings,
                        const QSize& imageSize,
                        const QDateTime& captureDateTimeUtc,
                        const QVector<CameraPipelineStarDetection>& starDetections,
                        const QVector<int>& detectionIndices,
                        double azimuthDegrees,
                        double elevationDegrees,
                        double rollDegrees,
                        double fovDegrees)
{
    Evaluation evaluation;
    evaluation.azimuthDegrees = normalizeDegrees(azimuthDegrees);
    evaluation.elevationDegrees = elevationDegrees;
    evaluation.rollDegrees = rollDegrees;
    evaluation.fovDegrees = fovDegrees;

    const SkyProjector projector = createProjector(
        settings,
        imageSize,
        evaluation.azimuthDegrees,
        evaluation.elevationDegrees,
        evaluation.rollDegrees,
        evaluation.fovDegrees);
    if (!projector.valid) {
        return evaluation;
    }

    const QVector<ProjectedCatalogStar> projectedStars = buildProjectedCatalog(
        settings,
        projector,
        captureDateTimeUtc,
        settings.m_plateSolveMaxMagnitude,
        settings.m_plateSolveMatchRadius);
    if (projectedStars.isEmpty()) {
        return evaluation;
    }

    evaluation.matches = buildMatches(
        settings,
        starDetections,
        detectionIndices,
        projectedStars,
        settings.m_plateSolveMatchRadius);
    evaluation.matchCount = evaluation.matches.size();
    if (evaluation.matchCount <= 0) {
        return evaluation;
    }

    double sumSquaredError = 0.0;
    for (const Match& match : evaluation.matches) {
        sumSquaredError += match.distancePixels * match.distancePixels;
    }
    evaluation.rmsErrorPixels = std::sqrt(sumSquaredError / evaluation.matchCount);
    evaluation.valid = true;
    return evaluation;
}

bool isBetterEvaluation(const Evaluation& candidate, const Evaluation& best)
{
    if (!candidate.valid) {
        return false;
    }
    if (!best.valid) {
        return true;
    }
    if (candidate.matchCount != best.matchCount) {
        return candidate.matchCount > best.matchCount;
    }
    if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
        return candidate.rmsErrorPixels < best.rmsErrorPixels;
    }

    return candidate.fovDegrees == best.fovDegrees
        ? candidate.rollDegrees < best.rollDegrees
        : candidate.fovDegrees < best.fovDegrees;
}

Evaluation searchBestPose(const CameraSettings& settings,
                          const QSize& imageSize,
                          const QDateTime& captureDateTimeUtc,
                          const QVector<CameraPipelineStarDetection>& starDetections,
                          const QVector<int>& detectionIndices)
{
    Evaluation best;

    const double coarseSearchRadius = std::max(0.0, settings.m_plateSolveSearchRadius);
    const double coarseRollRadius = std::max(4.0, std::min(20.0, static_cast<double>(settings.m_fov) * 0.20));
    const double coarseFovRadius = std::max(2.0, std::min(12.0, static_cast<double>(settings.m_fov) * 0.10));

    const std::array<double, 5> coarseOffsets = {{-1.0, -0.5, 0.0, 0.5, 1.0}};
    const std::array<double, 3> coarseFovOffsets = {{-1.0, 0.0, 1.0}};

    for (double azFactor : coarseOffsets)
    {
        for (double elFactor : coarseOffsets)
        {
            for (double rollFactor : coarseOffsets)
            {
                for (double fovFactor : coarseFovOffsets)
                {
                    const Evaluation candidate = evaluatePose(
                        settings,
                        imageSize,
                        captureDateTimeUtc,
                        starDetections,
                        detectionIndices,
                        settings.m_azimuth + azFactor * coarseSearchRadius,
                        settings.m_elevation + elFactor * coarseSearchRadius,
                        settings.m_roll + rollFactor * coarseRollRadius,
                        std::max(static_cast<double>(CameraSettings::m_minFov),
                                 static_cast<double>(settings.m_fov) + fovFactor * coarseFovRadius));
                    if (isBetterEvaluation(candidate, best)) {
                        best = candidate;
                    }
                }
            }
        }
    }

    if (!best.valid) {
        const std::array<double, 13> wideRollOffsets = {{-180.0, -150.0, -120.0, -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0}};
        for (double azimuthDegrees = 0.0; azimuthDegrees < 360.0; azimuthDegrees += 30.0)
        {
            for (double elevationDegrees = -60.0; elevationDegrees <= 75.0; elevationDegrees += 15.0)
            {
                for (double rollDegrees : wideRollOffsets)
                {
                    const Evaluation candidate = evaluatePose(
                        settings,
                        imageSize,
                        captureDateTimeUtc,
                        starDetections,
                        detectionIndices,
                        azimuthDegrees,
                        elevationDegrees,
                        rollDegrees,
                        static_cast<double>(settings.m_fov));
                    if (isBetterEvaluation(candidate, best)) {
                        best = candidate;
                    }
                }
            }
        }
    }

    if (!best.valid) {
        return best;
    }

    double azCenter = best.azimuthDegrees;
    double elCenter = best.elevationDegrees;
    double rollCenter = best.rollDegrees;
    double fovCenter = best.fovDegrees;
    double azStep = std::max(0.5, coarseSearchRadius * 0.25);
    double rollStep = std::max(1.0, coarseRollRadius * 0.25);
    double fovStep = std::max(0.5, coarseFovRadius * 0.25);

    for (int iteration = 0; iteration < 2; ++iteration)
    {
        for (double azOffset : coarseFovOffsets)
        {
            for (double elOffset : coarseFovOffsets)
            {
                for (double rollOffset : coarseFovOffsets)
                {
                    for (double fovOffset : coarseFovOffsets)
                    {
                        const Evaluation candidate = evaluatePose(
                            settings,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            detectionIndices,
                            azCenter + azOffset * azStep,
                            elCenter + elOffset * azStep,
                            rollCenter + rollOffset * rollStep,
                            std::max(static_cast<double>(CameraSettings::m_minFov), fovCenter + fovOffset * fovStep));
                        if (isBetterEvaluation(candidate, best)) {
                            best = candidate;
                        }
                    }
                }
            }
        }

        azCenter = best.azimuthDegrees;
        elCenter = best.elevationDegrees;
        rollCenter = best.rollDegrees;
        fovCenter = best.fovDegrees;
        azStep *= 0.5;
        rollStep *= 0.5;
        fovStep *= 0.5;
    }

    const QVector<int> allDetectionIndices = [&starDetections]() {
        QVector<int> indices;
        indices.reserve(starDetections.size());
        for (int i = 0; i < starDetections.size(); ++i) {
            indices.append(i);
        }
        return indices;
    }();

    azCenter = best.azimuthDegrees;
    elCenter = best.elevationDegrees;
    rollCenter = best.rollDegrees;
    fovCenter = best.fovDegrees;
    azStep = std::max(0.1, azStep);
    rollStep = std::max(0.25, rollStep);
    fovStep = std::max(0.1, fovStep);

    for (int iteration = 0; iteration < 4; ++iteration)
    {
        bool improved = false;
        const std::array<double, 3> refineOffsets = {{-1.0, 0.0, 1.0}};
        for (double azOffset : refineOffsets)
        {
            for (double elOffset : refineOffsets)
            {
                for (double rollOffset : refineOffsets)
                {
                    for (double fovOffset : refineOffsets)
                    {
                        const Evaluation candidate = evaluatePose(
                            settings,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            allDetectionIndices,
                            azCenter + azOffset * azStep,
                            elCenter + elOffset * azStep,
                            rollCenter + rollOffset * rollStep,
                            std::max(static_cast<double>(CameraSettings::m_minFov), fovCenter + fovOffset * fovStep));
                        if (isBetterEvaluation(candidate, best)) {
                            best = candidate;
                            improved = true;
                        }
                    }
                }
            }
        }

        azCenter = best.azimuthDegrees;
        elCenter = best.elevationDegrees;
        rollCenter = best.rollDegrees;
        fovCenter = best.fovDegrees;
        if (!improved) {
            azStep *= 0.5;
            rollStep *= 0.5;
            fovStep *= 0.5;
        }
    }

    return best;
}

void clearSolvedStars(QVector<CameraPipelineStarDetection>& starDetections)
{
    for (CameraPipelineStarDetection& detection : starDetections)
    {
        detection.m_label.clear();
        detection.m_matchDistancePixels = 0.0f;
        detection.m_catalogMagnitude = 0.0f;
        detection.m_solved = false;
    }
}

} // namespace

QString CameraPlateSolver::downloadedCatalogArchivePath()
{
    return QDir(downloadedCatalogDir()).filePath(QString::fromUtf8(kDownloadedCatalogArchiveFile));
}

QString CameraPlateSolver::downloadedCatalogCsvPath()
{
    return QDir(downloadedCatalogDir()).filePath(QString::fromUtf8(kDownloadedCatalogCsvFile));
}

bool CameraPlateSolver::importDownloadedCatalogArchive(const QString& archivePath, QString* errorMessage)
{
    QFile inputFile(archivePath);
    if (!inputFile.open(QIODevice::ReadOnly))
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to open downloaded catalog archive: %1").arg(archivePath);
        }
        return false;
    }

    const QByteArray compressedData = inputFile.readAll();
    const QByteArray uncompressedData = gunzipData(compressedData, errorMessage);
    if (uncompressedData.isEmpty()) {
        return false;
    }

    const QString outputDirPath = downloadedCatalogDir();
    QDir outputDir;
    if (!outputDir.mkpath(outputDirPath))
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to create plate-solver catalog directory: %1").arg(outputDirPath);
        }
        return false;
    }

    QFile outputFile(downloadedCatalogCsvPath());
    if (!outputFile.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to write imported HYG catalog: %1").arg(downloadedCatalogCsvPath());
        }
        return false;
    }

    if (outputFile.write(uncompressedData) != uncompressedData.size())
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to fully write imported HYG catalog: %1").arg(downloadedCatalogCsvPath());
        }
        return false;
    }

    return true;
}

CameraPlateSolveResult CameraPlateSolver::solve(const CameraSettings& settings,
                                                const QSize& imageSize,
                                                const QDateTime& captureDateTime,
                                                QVector<CameraPipelineStarDetection>& starDetections)
{
    CameraPlateSolveResult result;
    clearSolvedStars(starDetections);

    if (!settings.m_plateSolve || (starDetections.size() < settings.m_plateSolveMinMatches)) {
        return result;
    }

    const QVector<int> detectionIndices = selectDetectionIndicesForSolve(starDetections);
    if (detectionIndices.size() < settings.m_plateSolveMinMatches) {
        return result;
    }

    const QDateTime captureDateTimeUtc = (captureDateTime.isValid() ? captureDateTime : QDateTime::currentDateTime()).toUTC();
    const Evaluation best = searchBestPose(settings, imageSize, captureDateTimeUtc, starDetections, detectionIndices);
    if (!best.valid || (best.matchCount < settings.m_plateSolveMinMatches)) {
        return result;
    }

    const SkyProjector finalProjector = createProjector(
        settings,
        imageSize,
        best.azimuthDegrees,
        best.elevationDegrees,
        best.rollDegrees,
        best.fovDegrees);
    if (!finalProjector.valid) {
        return result;
    }

    const QVector<ProjectedCatalogStar> projectedStars = buildProjectedCatalog(
        settings,
        finalProjector,
        captureDateTimeUtc,
        settings.m_plateSolveMaxMagnitude,
        settings.m_plateSolveMatchRadius);
    const QVector<int> allDetectionIndices = [&starDetections]() {
        QVector<int> indices;
        indices.reserve(starDetections.size());
        for (int i = 0; i < starDetections.size(); ++i) {
            indices.append(i);
        }
        return indices;
    }();
    const QVector<Match> allMatches = buildMatches(
        settings,
        starDetections,
        allDetectionIndices,
        projectedStars,
        settings.m_plateSolveMatchRadius);

    if (allMatches.size() < settings.m_plateSolveMinMatches) {
        return result;
    }

    double sumSquaredError = 0.0;
    for (const Match& match : allMatches)
    {
        CameraPipelineStarDetection& detection = starDetections[match.detectionIndex];
        const CatalogStar& catalogStar = brightStarCatalog(settings)[match.catalogIndex];
        detection.m_label = catalogStar.name;
        detection.m_matchDistancePixels = static_cast<float>(match.distancePixels);
        detection.m_catalogMagnitude = static_cast<float>(catalogStar.magnitude);
        detection.m_solved = true;
        sumSquaredError += match.distancePixels * match.distancePixels;
    }

    result.m_solved = true;
    result.m_matchedStars = allMatches.size();
    result.m_rmsErrorPixels = std::sqrt(sumSquaredError / allMatches.size());
    result.m_azimuthDegrees = best.azimuthDegrees;
    result.m_elevationDegrees = best.elevationDegrees;
    result.m_rollDegrees = best.rollDegrees;
    result.m_fovDegrees = best.fovDegrees;
    return result;
}
