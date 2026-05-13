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
#include <QDebug>
#include <QFile>
#include <QFileInfo>
#include <QHash>
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
#include "util/profiler.h"

namespace {

struct CatalogStar
{
    QString name;
    double rightAscensionDegrees;
    double declinationDegrees;
    double magnitude;
    QString spectralType;
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
    double magnitude = 0.0;
};

struct VisibleCatalogStar
{
    int catalogIndex = -1;
    double azimuthDegrees = 0.0;
    double elevationDegrees = 0.0;
    double magnitude = 0.0;
    SkyVector vector;
};

struct CandidatePair
{
    int detectionIndex = -1;
    int catalogIndex = -1;
    double distancePixels = 0.0;
    int geometricSupport = 0;
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
    double centerOffsetXPixels = 0.0;
    double centerOffsetYPixels = 0.0;
    double distortionK1 = 0.0;
    QVector<Match> matches;
};

struct TriangleSignature
{
    double ratioShortToLong = 0.0;
    double ratioMidToLong = 0.0;
    double orientation = 0.0;
    double longestDistance = 0.0;
    std::array<int, 3> indices{{-1, -1, -1}};
};

struct QuadSignature
{
    std::array<double, 5> edgeRatios{{0.0, 0.0, 0.0, 0.0, 0.0}};
    double orientation = 0.0;
    double longestDistance = 0.0;
    std::array<int, 4> indices{{-1, -1, -1, -1}};
};

constexpr std::array<std::array<int, 3>, 6> kTrianglePermutations {{
    {{0, 1, 2}},
    {{0, 2, 1}},
    {{1, 0, 2}},
    {{1, 2, 0}},
    {{2, 0, 1}},
    {{2, 1, 0}}
}};

constexpr std::array<std::array<int, 4>, 8> kQuadPermutations {{
    {{0, 1, 2, 3}},
    {{1, 2, 3, 0}},
    {{2, 3, 0, 1}},
    {{3, 0, 1, 2}},
    {{0, 3, 2, 1}},
    {{3, 2, 1, 0}},
    {{2, 1, 0, 3}},
    {{1, 0, 3, 2}}
}};

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
    double principalPointX = 0.0;
    double principalPointY = 0.0;
    double distortionK1 = 0.0;
    int width = 0;
    int height = 0;
};

constexpr double kPi = 3.14159265358979323846;
constexpr double kVisibleAltitudeFloor = -5.0;
constexpr int kMaxDetectionsForSolve = 32;
constexpr double kBlindSeedRatioTolerance = 0.035;
constexpr double kBlindSeedMaxRmsPixels = 18.0;
constexpr double kBlindSeedMaxMedianPixels = 14.0;
constexpr double kUnnamedCatalogMagnitudeLimit = 4.5;
const char* const kBundledCatalogPath = ":/camera/brightstarcatalog.txt";
const char* const kDownloadedCatalogDir = "camera";
const char* const kDownloadedCatalogArchiveFile = "hyg_v42.csv.gz";
const char* const kDownloadedCatalogCsvFile = "hyg_v42.csv";
const char* const kDownloadedCatalogReducedFile = "hyg_v42_reduced.txt";

Evaluation evaluatePose(const CameraSettings& settings,
                        const QSize& imageSize,
                        const QDateTime& captureDateTimeUtc,
                        const QVector<CameraPipelineStarDetection>& starDetections,
                        const QVector<int>& detectionIndices,
                        double azimuthDegrees,
                        double elevationDegrees,
                        double rollDegrees,
                        double fovDegrees,
                        const QVector<int>* allowedCatalogIndices = nullptr,
                        double centerOffsetXPixels = 0.0,
                        double centerOffsetYPixels = 0.0,
                        double distortionK1 = 0.0);

Evaluation refinePoseFromMatches(const CameraSettings& settings,
                                 const QSize& imageSize,
                                 const QDateTime& captureDateTimeUtc,
                                 const QVector<CameraPipelineStarDetection>& starDetections,
                                 const Evaluation& initialEvaluation);

QVector<Match> rejectOutlierMatches(const QVector<Match>& matches,
                                    int minMatches,
                                    double matchRadiusPixels,
                                    int* outlierCount);

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

QString downloadedCatalogReducedPath()
{
    return QDir(downloadedCatalogDir()).filePath(QString::fromUtf8(kDownloadedCatalogReducedFile));
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

bool isGenericCatalogName(const QString& name)
{
    return name.startsWith(QStringLiteral("HIP "))
        || name.startsWith(QStringLiteral("HR "))
        || name.startsWith(QStringLiteral("HD "));
}

QVector<CatalogStar> filterCatalogStars(const QVector<CatalogStar>& stars)
{
    QVector<CatalogStar> filtered;
    filtered.reserve(stars.size());
    QHash<QString, int> bestByName;

    for (const CatalogStar& star : stars)
    {
        if (!std::isfinite(star.rightAscensionDegrees)
            || !std::isfinite(star.declinationDegrees)
            || !std::isfinite(star.magnitude)
            || star.name.trimmed().isEmpty()
            || (star.magnitude > CameraSettings::m_maxPlateSolveMagnitude))
        {
            continue;
        }

        if (isGenericCatalogName(star.name) && (star.magnitude > kUnnamedCatalogMagnitudeLimit)) {
            continue;
        }

        const QString normalizedName = star.name.trimmed().toUpper();
        const auto existingIt = bestByName.constFind(normalizedName);
        if (existingIt != bestByName.constEnd())
        {
            CatalogStar& existing = filtered[*existingIt];
            if (star.magnitude < existing.magnitude) {
                existing = star;
            }
            continue;
        }

        bestByName.insert(normalizedName, filtered.size());
        filtered.append(star);
    }

    std::sort(filtered.begin(), filtered.end(), [](const CatalogStar& lhs, const CatalogStar& rhs) {
        if (!qFuzzyCompare(lhs.magnitude + 1.0, rhs.magnitude + 1.0)) {
            return lhs.magnitude < rhs.magnitude;
        }
        return lhs.name < rhs.name;
    });

    return filtered;
}

QString formatRightAscensionHours(double rightAscensionDegrees)
{
    const double totalHours = normalizeDegrees(rightAscensionDegrees) / 15.0;
    const int hours = static_cast<int>(std::floor(totalHours));
    const double totalMinutes = (totalHours - hours) * 60.0;
    const int minutes = static_cast<int>(std::floor(totalMinutes));
    const double seconds = (totalMinutes - minutes) * 60.0;
    return QStringLiteral("%1 %2 %3")
        .arg(hours, 2, 10, QLatin1Char('0'))
        .arg(minutes, 2, 10, QLatin1Char('0'))
        .arg(seconds, 0, 'f', 5);
}

QString formatDeclinationDegrees(double declinationDegrees)
{
    const QChar sign = (declinationDegrees < 0.0) ? QLatin1Char('-') : QLatin1Char('+');
    const double absoluteDegrees = std::fabs(declinationDegrees);
    const int degrees = static_cast<int>(std::floor(absoluteDegrees));
    const double totalMinutes = (absoluteDegrees - degrees) * 60.0;
    const int minutes = static_cast<int>(std::floor(totalMinutes));
    const double seconds = (totalMinutes - minutes) * 60.0;
    return QStringLiteral("%1%2 %3 %4")
        .arg(sign)
        .arg(degrees, 2, 10, QLatin1Char('0'))
        .arg(minutes, 2, 10, QLatin1Char('0'))
        .arg(seconds, 0, 'f', 4);
}

bool writeReducedCatalog(const QVector<CatalogStar>& stars, const QString& path, QString* errorMessage)
{
    QFile outputFile(path);
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
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to fully write reduced HYG catalog: %1").arg(path);
        }
        return false;
    }

    return true;
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
    if (settings.m_plateSolveUseDownloadedCatalog && QFileInfo::exists(downloadedCatalogReducedPath())) {
        return downloadedCatalogReducedPath();
    }
    return QString::fromUtf8(kBundledCatalogPath);
}

QString currentCatalogSource(const CameraSettings& settings)
{
    return (settings.m_plateSolveUseDownloadedCatalog && QFileInfo::exists(downloadedCatalogReducedPath()))
        ? QStringLiteral("HYG")
        : QStringLiteral("Bundled");
}

bool plateSolveStartUsesFov(const CameraSettings& settings)
{
    return settings.m_plateSolveStartMode != CameraSettings::PlateSolveStartBlind;
}

bool plateSolveStartUsesElevation(const CameraSettings& settings)
{
    return settings.m_plateSolveStartMode >= CameraSettings::PlateSolveStartFovElevation;
}

bool plateSolveStartUsesDirection(const CameraSettings& settings)
{
    return settings.m_plateSolveStartMode >= CameraSettings::PlateSolveStartFovAzElRoll;
}

bool plateSolveStartUsesLens(const CameraSettings& settings)
{
    return settings.m_plateSolveStartMode >= CameraSettings::PlateSolveStartFovAzElRollLens;
}

bool canCalibrateLens(const CameraSettings& settings)
{
    return plateSolveStartUsesLens(settings);
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
                             double fovDegrees,
                             double centerOffsetXPixels = 0.0,
                             double centerOffsetYPixels = 0.0,
                             double distortionK1 = 0.0)
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
    projector.principalPointX = static_cast<double>(projector.width) * 0.5 + centerOffsetXPixels;
    projector.principalPointY = static_cast<double>(projector.height) * 0.5 + centerOffsetYPixels;
    projector.distortionK1 = distortionK1;
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

    double projectedX = std::cos(phi) * projectionRadius;
    double projectedY = std::sin(phi) * projectionRadius;
    if (std::fabs(projector.distortionK1) > 1e-9)
    {
        const double radiusSquared = projectedX * projectedX + projectedY * projectedY;
        const double distortionScale = std::max(0.1, 1.0 + projector.distortionK1 * radiusSquared);
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

QVector<int> selectDetectionIndicesForSolve(const QVector<CameraPipelineStarDetection>& starDetections,
                                             const QSize& imageSize)
{
    QVector<int> indices;
    indices.reserve(starDetections.size());
    for (int i = 0; i < starDetections.size(); ++i) {
        indices.append(i);
    }

    std::sort(indices.begin(), indices.end(), [&starDetections](int lhs, int rhs) {
        if (!qFuzzyCompare(starDetections[lhs].m_qualityScore + 1.0f, starDetections[rhs].m_qualityScore + 1.0f)) {
            return starDetections[lhs].m_qualityScore > starDetections[rhs].m_qualityScore;
        }
        if (starDetections[lhs].m_peakValue != starDetections[rhs].m_peakValue) {
            return starDetections[lhs].m_peakValue > starDetections[rhs].m_peakValue;
        }
        if (starDetections[lhs].m_saturated != starDetections[rhs].m_saturated) {
            return !starDetections[lhs].m_saturated;
        }
        return starDetections[lhs].m_roundness > starDetections[rhs].m_roundness;
    });

    if (indices.size() > kMaxDetectionsForSolve) {
        indices.resize(kMaxDetectionsForSolve);
    }

    // Prefer stars spread across the image: reject candidates too close to already-selected ones.
    // This improves the geometric diversity of triangle/quad patterns used for blind matching.
    const double minSpreadPixels = std::min(imageSize.width(), imageSize.height()) / (kMaxDetectionsForSolve * 0.75);
    const double minSpreadSquared = minSpreadPixels * minSpreadPixels;
    QVector<int> spread;
    spread.reserve(indices.size());
    for (int candidate : indices) {
        const QPointF& candidatePos = starDetections[candidate].m_center;
        bool tooClose = false;
        for (int selected : spread) {
            const double dx = candidatePos.x() - starDetections[selected].m_center.x();
            const double dy = candidatePos.y() - starDetections[selected].m_center.y();
            if ((dx * dx + dy * dy) < minSpreadSquared) {
                tooClose = true;
                break;
            }
        }
        if (!tooClose) {
            spread.append(candidate);
        }
    }
    if (spread.size() >= 4) {
        indices = spread;
    }

    return indices;
}

QVector<ProjectedCatalogStar> buildProjectedCatalog(const CameraSettings& settings,
                                                    const SkyProjector& projector,
                                                    const QDateTime& captureDateTimeUtc,
                                                    double maxMagnitude,
                                                    double searchMarginPixels,
                                                    const QVector<int>* allowedCatalogIndices = nullptr)
{
    QVector<ProjectedCatalogStar> projectedStars;
    const QVector<CatalogStar>& catalogStars = brightStarCatalog(settings);

    const auto appendProjectedStar = [&](int i)
    {
        const CatalogStar& star = catalogStars[i];
        if (star.magnitude > maxMagnitude) {
            return;
        }

        const RADec raDec{star.rightAscensionDegrees / 15.0, star.declinationDegrees};
        const AzAlt azAlt = Astronomy::raDecToAzAlt(
            raDec,
            settings.m_latitude,
            settings.m_longitude,
            captureDateTimeUtc,
            true);

        if (!std::isfinite(azAlt.az) || !std::isfinite(azAlt.alt) || (azAlt.alt < kVisibleAltitudeFloor)) {
            return;
        }

        QPointF point;
        if (!projectAltAz(projector, azAlt.az, azAlt.alt, point)) {
            return;
        }

        const QRectF expandedBounds(
            -searchMarginPixels,
            -searchMarginPixels,
            projector.width + 2.0 * searchMarginPixels,
            projector.height + 2.0 * searchMarginPixels);
        if (!expandedBounds.contains(point)) {
            return;
        }

        projectedStars.append({i, point, star.magnitude});
    };

    if (allowedCatalogIndices)
    {
        projectedStars.reserve(allowedCatalogIndices->size());
        for (int catalogIndex : *allowedCatalogIndices)
        {
            if ((catalogIndex >= 0) && (catalogIndex < catalogStars.size())) {
                appendProjectedStar(catalogIndex);
            }
        }
    }
    else
    {
        for (int i = 0; i < catalogStars.size(); ++i) {
            appendProjectedStar(i);
        }
    }

    return projectedStars;
}

QVector<VisibleCatalogStar> buildVisibleCatalog(const CameraSettings& settings,
                                                const QDateTime& captureDateTimeUtc,
                                                double maxMagnitude)
{
    QVector<VisibleCatalogStar> visibleStars;
    const QVector<CatalogStar>& catalogStars = brightStarCatalog(settings);
    visibleStars.reserve(catalogStars.size());

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

        visibleStars.append({
            i,
            azAlt.az,
            azAlt.alt,
            star.magnitude,
            normalize(vectorFromAltAz(azAlt.az, azAlt.alt))
        });
    }

    std::sort(visibleStars.begin(), visibleStars.end(), [](const VisibleCatalogStar& lhs, const VisibleCatalogStar& rhs) {
        return lhs.magnitude < rhs.magnitude;
    });

    return visibleStars;
}

TriangleSignature buildTriangleSignature(const std::array<QPointF, 3>& points)
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

    return signature;
}

std::array<QPointF, 4> orderQuadPoints(const std::array<QPointF, 4>& points)
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

QuadSignature buildQuadSignature(const std::array<QPointF, 4>& unorderedPoints)
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
    return signature;
}

double medianCandidateDistancePixels(const QVector<Match>& matches)
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

bool isStrongBlindSeedEvaluation(const CameraSettings& settings,
                                 const QVector<int>& detectionIndices,
                                 const Evaluation& candidate)
{
    if (!candidate.valid) {
        return false;
    }

    const int minBlindSeedMatches = std::max(
        settings.m_plateSolveMinMatches + 1,
        std::min(6, static_cast<int>(detectionIndices.size())));
    if (candidate.matchCount < minBlindSeedMatches) {
        return false;
    }

    const double medianError = medianCandidateDistancePixels(candidate.matches);
    const double maxRmsError = std::min(settings.m_plateSolveMatchRadius * 0.60, kBlindSeedMaxRmsPixels);
    const double maxMedianError = std::min(settings.m_plateSolveMatchRadius * 0.50, kBlindSeedMaxMedianPixels);
    return (candidate.rmsErrorPixels <= maxRmsError) && (medianError <= maxMedianError);
}

Evaluation verifyBlindSeedCandidate(const CameraSettings& settings,
                                    const QSize& imageSize,
                                    const QDateTime& captureDateTimeUtc,
                                    const QVector<CameraPipelineStarDetection>& starDetections,
                                    const QVector<int>& detectionIndices,
                                    const Evaluation& candidate)
{
    if (!isStrongBlindSeedEvaluation(settings, detectionIndices, candidate)) {
        return Evaluation{};
    }

    int outlierCount = 0;
    const QVector<Match> inlierMatches = rejectOutlierMatches(
        candidate.matches,
        std::max(settings.m_plateSolveMinMatches, 4),
        std::min(settings.m_plateSolveMatchRadius, 12.0),
        &outlierCount);

    const int minConsensusMatches = std::max(settings.m_plateSolveMinMatches + 1, std::min(6, static_cast<int>(detectionIndices.size())));
    if (inlierMatches.size() < minConsensusMatches) {
        return Evaluation{};
    }

    Evaluation consensusCandidate = candidate;
    consensusCandidate.matches = inlierMatches;
    consensusCandidate.matchCount = inlierMatches.size();
    double sumSquaredError = 0.0;
    for (const Match& match : inlierMatches) {
        sumSquaredError += match.distancePixels * match.distancePixels;
    }
    consensusCandidate.rmsErrorPixels = std::sqrt(sumSquaredError / consensusCandidate.matchCount);
    consensusCandidate.valid = true;

    Evaluation refinedCandidate = refinePoseFromMatches(
        settings,
        imageSize,
        captureDateTimeUtc,
        starDetections,
        consensusCandidate);

    if (!isStrongBlindSeedEvaluation(settings, detectionIndices, refinedCandidate)) {
        return Evaluation{};
    }

    return refinedCandidate;
}

QVector<TriangleSignature> buildDetectionTriangleSignatures(const QVector<CameraPipelineStarDetection>& starDetections,
                                                            const QVector<int>& detectionIndices)
{
    QVector<TriangleSignature> signatures;
    const int maxDetections = std::min<int>(10, static_cast<int>(detectionIndices.size()));
    for (int i = 0; i < maxDetections; ++i)
    {
        for (int j = i + 1; j < maxDetections; ++j)
        {
            for (int k = j + 1; k < maxDetections; ++k)
            {
                const std::array<int, 3> indices {{
                    detectionIndices[i],
                    detectionIndices[j],
                    detectionIndices[k]
                }};
                const std::array<QPointF, 3> points {{
                    starDetections[indices[0]].m_center,
                    starDetections[indices[1]].m_center,
                    starDetections[indices[2]].m_center
                }};
                TriangleSignature signature = buildTriangleSignature(points);
                if (signature.longestDistance < 20.0) {
                    continue;
                }
                signature.indices = indices;
                signatures.append(signature);
            }
        }
    }
    return signatures;
}

QVector<TriangleSignature> buildCatalogTriangleSignatures(const CameraSettings& settings,
                                                          const QVector<VisibleCatalogStar>& visibleStars)
{
    QVector<TriangleSignature> signatures;
    const int maxCatalogStars = std::min<int>(32, static_cast<int>(visibleStars.size()));
    for (int i = 0; i < maxCatalogStars; ++i)
    {
        for (int j = i + 1; j < maxCatalogStars; ++j)
        {
            for (int k = j + 1; k < maxCatalogStars; ++k)
            {
                const SkyVector center = normalize({
                    visibleStars[i].vector.x + visibleStars[j].vector.x + visibleStars[k].vector.x,
                    visibleStars[i].vector.y + visibleStars[j].vector.y + visibleStars[k].vector.y,
                    visibleStars[i].vector.z + visibleStars[j].vector.z + visibleStars[k].vector.z
                });
                if (length(center) <= 0.0) {
                    continue;
                }

                const double centerAzimuth = normalizeDegrees(std::atan2(center.x, center.y) * 180.0 / kPi);
                const double centerElevation = std::asin(std::clamp(center.z, -1.0, 1.0)) * 180.0 / kPi;
                const SkyProjector localProjector = createProjector(
                    settings,
                    QSize(1000, 1000),
                    centerAzimuth,
                    centerElevation,
                    0.0,
                    std::max(20.0, static_cast<double>(settings.m_fov)));
                if (!localProjector.valid) {
                    continue;
                }

                std::array<QPointF, 3> points;
                bool allProjected = true;
                const std::array<int, 3> starIndices {{i, j, k}};
                for (int pointIndex = 0; pointIndex < 3; ++pointIndex)
                {
                    if (!projectAltAz(localProjector,
                                      visibleStars[starIndices[pointIndex]].azimuthDegrees,
                                      visibleStars[starIndices[pointIndex]].elevationDegrees,
                                      points[pointIndex]))
                    {
                        allProjected = false;
                        break;
                    }
                }
                if (!allProjected) {
                    continue;
                }

                TriangleSignature signature = buildTriangleSignature(points);
                if (signature.longestDistance < 10.0) {
                    continue;
                }
                signature.indices = {{i, j, k}};
                signatures.append(signature);
            }
        }
    }
    return signatures;
}

QVector<QuadSignature> buildDetectionQuadSignatures(const QVector<CameraPipelineStarDetection>& starDetections,
                                                    const QVector<int>& detectionIndices)
{
    QVector<QuadSignature> signatures;
    const int maxDetections = std::min<int>(8, static_cast<int>(detectionIndices.size()));
    for (int i = 0; i < maxDetections; ++i)
    {
        for (int j = i + 1; j < maxDetections; ++j)
        {
            for (int k = j + 1; k < maxDetections; ++k)
            {
                for (int l = k + 1; l < maxDetections; ++l)
                {
                    const std::array<int, 4> indices {{
                        detectionIndices[i],
                        detectionIndices[j],
                        detectionIndices[k],
                        detectionIndices[l]
                    }};
                    const std::array<QPointF, 4> points {{
                        starDetections[indices[0]].m_center,
                        starDetections[indices[1]].m_center,
                        starDetections[indices[2]].m_center,
                        starDetections[indices[3]].m_center
                    }};
                    QuadSignature signature = buildQuadSignature(points);
                    if (signature.longestDistance < 30.0) {
                        continue;
                    }
                    signature.indices = indices;
                    signatures.append(signature);
                }
            }
        }
    }
    return signatures;
}

QVector<QuadSignature> buildCatalogQuadSignatures(const CameraSettings& settings,
                                                  const QVector<VisibleCatalogStar>& visibleStars)
{
    QVector<QuadSignature> signatures;
    const int maxCatalogStars = std::min<int>(24, static_cast<int>(visibleStars.size()));
    for (int i = 0; i < maxCatalogStars; ++i)
    {
        for (int j = i + 1; j < maxCatalogStars; ++j)
        {
            for (int k = j + 1; k < maxCatalogStars; ++k)
            {
                for (int l = k + 1; l < maxCatalogStars; ++l)
                {
                    const SkyVector center = normalize({
                        visibleStars[i].vector.x + visibleStars[j].vector.x + visibleStars[k].vector.x + visibleStars[l].vector.x,
                        visibleStars[i].vector.y + visibleStars[j].vector.y + visibleStars[k].vector.y + visibleStars[l].vector.y,
                        visibleStars[i].vector.z + visibleStars[j].vector.z + visibleStars[k].vector.z + visibleStars[l].vector.z
                    });
                    if (length(center) <= 0.0) {
                        continue;
                    }

                    const double centerAzimuth = normalizeDegrees(std::atan2(center.x, center.y) * 180.0 / kPi);
                    const double centerElevation = std::asin(std::clamp(center.z, -1.0, 1.0)) * 180.0 / kPi;
                    const SkyProjector localProjector = createProjector(
                        settings,
                        QSize(1000, 1000),
                        centerAzimuth,
                        centerElevation,
                        0.0,
                        std::max(20.0, static_cast<double>(settings.m_fov)));
                    if (!localProjector.valid) {
                        continue;
                    }

                    std::array<QPointF, 4> points;
                    bool allProjected = true;
                    const std::array<int, 4> starIndices {{i, j, k, l}};
                    for (int pointIndex = 0; pointIndex < 4; ++pointIndex)
                    {
                        if (!projectAltAz(localProjector,
                                          visibleStars[starIndices[pointIndex]].azimuthDegrees,
                                          visibleStars[starIndices[pointIndex]].elevationDegrees,
                                          points[pointIndex]))
                        {
                            allProjected = false;
                            break;
                        }
                    }
                    if (!allProjected) {
                        continue;
                    }

                    QuadSignature signature = buildQuadSignature(points);
                    if (signature.longestDistance < 15.0) {
                        continue;
                    }
                    signature.indices = {{i, j, k, l}};
                    signatures.append(signature);
                }
            }
        }
    }
    return signatures;
}

QVector<Evaluation> buildBlindTriangleSeeds(const CameraSettings& settings,
                                            const QSize& imageSize,
                                            const QDateTime& captureDateTimeUtc,
                                            const QVector<CameraPipelineStarDetection>& starDetections,
                                            const QVector<int>& detectionIndices,
                                            const QVector<VisibleCatalogStar>& visibleStars)
{
    QVector<Evaluation> seeds;
    if (visibleStars.size() < settings.m_plateSolveMinMatches) {
        return seeds;
    }

    const QVector<TriangleSignature> detectionTriangles = buildDetectionTriangleSignatures(starDetections, detectionIndices);
    const QVector<TriangleSignature> catalogTriangles = buildCatalogTriangleSignatures(settings, visibleStars);
    if (detectionTriangles.isEmpty() || catalogTriangles.isEmpty()) {
        return seeds;
    }

    const int minBlindSeedMatches = std::max(
        settings.m_plateSolveMinMatches + 1,
        std::min(6, static_cast<int>(detectionIndices.size())));

    struct TriedDirection { double azimuthDegrees; double elevationDegrees; };
    QVector<TriedDirection> triedDirections;
    bool earlyExit = false;

    for (const TriangleSignature& detectionTriangle : detectionTriangles)
    {
        if (earlyExit) break;
        for (const TriangleSignature& catalogTriangle : catalogTriangles)
        {
            if (earlyExit) break;
            if (std::fabs(detectionTriangle.ratioShortToLong - catalogTriangle.ratioShortToLong) > kBlindSeedRatioTolerance
                || std::fabs(detectionTriangle.ratioMidToLong - catalogTriangle.ratioMidToLong) > kBlindSeedRatioTolerance)
            {
                continue;
            }

            if ((detectionTriangle.orientation * catalogTriangle.orientation) < 0.0) {
                continue;
            }

            const VisibleCatalogStar& a = visibleStars[catalogTriangle.indices[0]];
            const VisibleCatalogStar& b = visibleStars[catalogTriangle.indices[1]];
            const VisibleCatalogStar& c = visibleStars[catalogTriangle.indices[2]];
            const SkyVector center = normalize({
                a.vector.x + b.vector.x + c.vector.x,
                a.vector.y + b.vector.y + c.vector.y,
                a.vector.z + b.vector.z + c.vector.z
            });
            if (length(center) <= 0.0) {
                continue;
            }

            const double seedAzimuth = normalizeDegrees(std::atan2(center.x, center.y) * 180.0 / kPi);
            const double seedElevation = std::asin(std::clamp(center.z, -1.0, 1.0)) * 180.0 / kPi;

            // Skip sky directions already tried by a previous triangle match.
            bool alreadyTried = false;
            for (const TriedDirection& tried : triedDirections) {
                if (std::fabs(seedAzimuth - tried.azimuthDegrees) < 3.0
                    && std::fabs(seedElevation - tried.elevationDegrees) < 3.0)
                {
                    alreadyTried = true;
                    break;
                }
            }
            if (alreadyTried) continue;
            triedDirections.append({seedAzimuth, seedElevation});

            // Use the longest pairwise angular distance so the scale estimate matches
            // detectionTriangle.longestDistance regardless of which edge is longest.
            const double abAngle = std::acos(std::clamp(dot(a.vector, b.vector), -1.0, 1.0)) * 180.0 / kPi;
            const double acAngle = std::acos(std::clamp(dot(a.vector, c.vector), -1.0, 1.0)) * 180.0 / kPi;
            const double bcAngle = std::acos(std::clamp(dot(b.vector, c.vector), -1.0, 1.0)) * 180.0 / kPi;
            const double catalogAngularDistance = std::max({abAngle, acAngle, bcAngle});
            if (catalogAngularDistance <= 0.01) {
                continue;
            }

            const double baseSeedFov = std::clamp(
                catalogAngularDistance * static_cast<double>(imageSize.width()) / std::max(1.0, detectionTriangle.longestDistance),
                static_cast<double>(CameraSettings::m_minFov),
                static_cast<double>(CameraSettings::m_maxFov));

            const std::array<QPointF, 3> detectionPoints {{
                starDetections[detectionTriangle.indices[0]].m_center,
                starDetections[detectionTriangle.indices[1]].m_center,
                starDetections[detectionTriangle.indices[2]].m_center
            }};
            const std::array<VisibleCatalogStar, 3> triangleStars {{a, b, c}};

            QVector<int> allowedCatalogIndices {
                a.catalogIndex,
                b.catalogIndex,
                c.catalogIndex
            };

            // For very wide fields, allow a broader but still bounded FoV sweep around the seed estimate.
            for (double fovScale : {0.85, 0.93, 1.0, 1.07, 1.15})
            {
                if (earlyExit) break;
                const double seedFov = std::clamp(
                    baseSeedFov * fovScale,
                    static_cast<double>(CameraSettings::m_minFov),
                    180.0);
                const SkyProjector rollProjector = createProjector(settings, imageSize, seedAzimuth, seedElevation, 0.0, seedFov);
                if (!rollProjector.valid) {
                    continue;
                }

                std::array<QPointF, 3> projectedPoints;
                bool projected = true;
                for (int idx = 0; idx < 3; ++idx)
                {
                    if (!projectAltAz(rollProjector, triangleStars[idx].azimuthDegrees, triangleStars[idx].elevationDegrees, projectedPoints[idx]))
                    {
                        projected = false;
                        break;
                    }
                }
                if (!projected) {
                    continue;
                }

                for (const std::array<int, 3>& permutation : kTrianglePermutations)
                {
                    if (earlyExit) break;
                    const QLineF detectionBase(detectionPoints[0], detectionPoints[1]);
                    const QLineF projectedBase(projectedPoints[permutation[0]], projectedPoints[permutation[1]]);
                    double baseRoll = projectedBase.angleTo(detectionBase);
                    if (!std::isfinite(baseRoll)) {
                        baseRoll = 0.0;
                    }

                    // Sweep small roll perturbations to tolerate centroiding noise on the reference edge.
                    for (double rollDelta : {-10.0, -5.0, 0.0, 5.0, 10.0})
                    {
                        if (earlyExit) break;
                        const double seedRoll = baseRoll + rollDelta;

                        const Evaluation candidate = evaluatePose(
                            settings,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            detectionIndices,
                            seedAzimuth,
                            seedElevation,
                            seedRoll,
                            seedFov,
                            &allowedCatalogIndices,
                            settings.m_lensCenterOffsetX,
                            settings.m_lensCenterOffsetY,
                            settings.m_lensDistortionK1);
                        const Evaluation verifiedCandidate = verifyBlindSeedCandidate(
                            settings,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            detectionIndices,
                            candidate);
                        if (verifiedCandidate.valid) {
                            seeds.append(verifiedCandidate);
                            if (verifiedCandidate.matchCount >= minBlindSeedMatches + 3
                                && verifiedCandidate.rmsErrorPixels < kBlindSeedMaxRmsPixels * 0.5)
                            {
                                earlyExit = true;
                            }
                        }
                    }
                }
            }
        }
    }

    std::sort(seeds.begin(), seeds.end(), [](const Evaluation& lhs, const Evaluation& rhs) {
        if (lhs.matchCount != rhs.matchCount) {
            return lhs.matchCount > rhs.matchCount;
        }
        return lhs.rmsErrorPixels < rhs.rmsErrorPixels;
    });
    if (seeds.size() > 16) {
        seeds.resize(16);
    }

    return seeds;
}

QVector<Evaluation> buildBlindQuadSeeds(const CameraSettings& settings,
                                        const QSize& imageSize,
                                        const QDateTime& captureDateTimeUtc,
                                        const QVector<CameraPipelineStarDetection>& starDetections,
                                        const QVector<int>& detectionIndices,
                                        const QVector<VisibleCatalogStar>& visibleStars)
{
    QVector<Evaluation> seeds;
    if (visibleStars.size() < settings.m_plateSolveMinMatches) {
        return seeds;
    }

    const QVector<QuadSignature> detectionQuads = buildDetectionQuadSignatures(starDetections, detectionIndices);
    const QVector<QuadSignature> catalogQuads = buildCatalogQuadSignatures(settings, visibleStars);
    if (detectionQuads.isEmpty() || catalogQuads.isEmpty()) {
        return seeds;
    }

    const int minBlindSeedMatches = std::max(
        settings.m_plateSolveMinMatches + 1,
        std::min(6, static_cast<int>(detectionIndices.size())));

    struct TriedDirection { double azimuthDegrees; double elevationDegrees; };
    QVector<TriedDirection> triedDirections;
    bool earlyExit = false;

    for (const QuadSignature& detectionQuad : detectionQuads)
    {
        if (earlyExit) break;
        for (const QuadSignature& catalogQuad : catalogQuads)
        {
            if (earlyExit) break;
            bool ratiosMatch = true;
            for (int idx = 0; idx < 5; ++idx)
            {
                if (std::fabs(detectionQuad.edgeRatios[idx] - catalogQuad.edgeRatios[idx]) > 0.03)
                {
                    ratiosMatch = false;
                    break;
                }
            }
            if (!ratiosMatch) {
                continue;
            }

            if ((detectionQuad.orientation * catalogQuad.orientation) < 0.0) {
                continue;
            }

            const VisibleCatalogStar& a = visibleStars[catalogQuad.indices[0]];
            const VisibleCatalogStar& b = visibleStars[catalogQuad.indices[1]];
            const VisibleCatalogStar& c = visibleStars[catalogQuad.indices[2]];
            const VisibleCatalogStar& d = visibleStars[catalogQuad.indices[3]];
            const SkyVector center = normalize({
                a.vector.x + b.vector.x + c.vector.x + d.vector.x,
                a.vector.y + b.vector.y + c.vector.y + d.vector.y,
                a.vector.z + b.vector.z + c.vector.z + d.vector.z
            });
            if (length(center) <= 0.0) {
                continue;
            }

            const double seedAzimuth = normalizeDegrees(std::atan2(center.x, center.y) * 180.0 / kPi);
            const double seedElevation = std::asin(std::clamp(center.z, -1.0, 1.0)) * 180.0 / kPi;

            // Skip sky directions already tried by a previous quad match.
            bool alreadyTried = false;
            for (const TriedDirection& tried : triedDirections) {
                if (std::fabs(seedAzimuth - tried.azimuthDegrees) < 3.0
                    && std::fabs(seedElevation - tried.elevationDegrees) < 3.0)
                {
                    alreadyTried = true;
                    break;
                }
            }
            if (alreadyTried) continue;
            triedDirections.append({seedAzimuth, seedElevation});

            std::array<double, 6> angularDistances{{
                std::acos(std::clamp(dot(a.vector, b.vector), -1.0, 1.0)) * 180.0 / kPi,
                std::acos(std::clamp(dot(a.vector, c.vector), -1.0, 1.0)) * 180.0 / kPi,
                std::acos(std::clamp(dot(a.vector, d.vector), -1.0, 1.0)) * 180.0 / kPi,
                std::acos(std::clamp(dot(b.vector, c.vector), -1.0, 1.0)) * 180.0 / kPi,
                std::acos(std::clamp(dot(b.vector, d.vector), -1.0, 1.0)) * 180.0 / kPi,
                std::acos(std::clamp(dot(c.vector, d.vector), -1.0, 1.0)) * 180.0 / kPi
            }};
            const double maxAngularDistance = *std::max_element(angularDistances.begin(), angularDistances.end());
            if (maxAngularDistance <= 0.01) {
                continue;
            }

            const double baseSeedFov = std::clamp(
                maxAngularDistance * static_cast<double>(imageSize.width()) / std::max(1.0, detectionQuad.longestDistance),
                static_cast<double>(CameraSettings::m_minFov),
                static_cast<double>(CameraSettings::m_maxFov));

            const std::array<QPointF, 4> detectionPoints {{
                starDetections[detectionQuad.indices[0]].m_center,
                starDetections[detectionQuad.indices[1]].m_center,
                starDetections[detectionQuad.indices[2]].m_center,
                starDetections[detectionQuad.indices[3]].m_center
            }};
            const std::array<VisibleCatalogStar, 4> quadStars {{a, b, c, d}};
            QVector<int> allowedCatalogIndices {
                a.catalogIndex,
                b.catalogIndex,
                c.catalogIndex,
                d.catalogIndex
            };

            for (double fovScale : {0.85, 0.95, 1.0, 1.10, 1.20})
            {
                if (earlyExit) break;
                const double seedFov = std::clamp(
                    baseSeedFov * fovScale,
                    static_cast<double>(CameraSettings::m_minFov),
                    180.0);
                const SkyProjector rollProjector = createProjector(settings, imageSize, seedAzimuth, seedElevation, 0.0, seedFov);
                if (!rollProjector.valid) {
                    continue;
                }

                std::array<QPointF, 4> projectedPoints;
                bool projected = true;
                for (int idx = 0; idx < 4; ++idx)
                {
                    if (!projectAltAz(rollProjector, quadStars[idx].azimuthDegrees, quadStars[idx].elevationDegrees, projectedPoints[idx]))
                    {
                        projected = false;
                        break;
                    }
                }
                if (!projected) {
                    continue;
                }

                for (const std::array<int, 4>& permutation : kQuadPermutations)
                {
                    if (earlyExit) break;
                    const QLineF detectionBase(detectionPoints[0], detectionPoints[1]);
                    const QLineF projectedBase(projectedPoints[permutation[0]], projectedPoints[permutation[1]]);
                    double baseRoll = projectedBase.angleTo(detectionBase);
                    if (!std::isfinite(baseRoll)) {
                        baseRoll = 0.0;
                    }

                    // Sweep small roll perturbations to tolerate centroiding noise on the reference edge.
                    for (double rollDelta : {-10.0, -5.0, 0.0, 5.0, 10.0})
                    {
                        if (earlyExit) break;
                        const double seedRoll = baseRoll + rollDelta;

                        const Evaluation candidate = evaluatePose(
                            settings,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            detectionIndices,
                            seedAzimuth,
                            seedElevation,
                            seedRoll,
                            seedFov,
                            &allowedCatalogIndices,
                            settings.m_lensCenterOffsetX,
                            settings.m_lensCenterOffsetY,
                            settings.m_lensDistortionK1);
                        const Evaluation verifiedCandidate = verifyBlindSeedCandidate(
                            settings,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            detectionIndices,
                            candidate);
                        if (verifiedCandidate.valid) {
                            seeds.append(verifiedCandidate);
                            if (verifiedCandidate.matchCount >= minBlindSeedMatches + 3
                                && verifiedCandidate.rmsErrorPixels < kBlindSeedMaxRmsPixels * 0.5)
                            {
                                earlyExit = true;
                            }
                        }
                    }
                }
            }
        }
    }

    std::sort(seeds.begin(), seeds.end(), [](const Evaluation& lhs, const Evaluation& rhs) {
        if (lhs.matchCount != rhs.matchCount) {
            return lhs.matchCount > rhs.matchCount;
        }
        return lhs.rmsErrorPixels < rhs.rmsErrorPixels;
    });
    if (seeds.size() > 12) {
        seeds.resize(12);
    }

    return seeds;
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
    QHash<int, QPointF> projectedPointByCatalogIndex;
    projectedPointByCatalogIndex.reserve(projectedStars.size());
    for (const ProjectedCatalogStar& projected : projectedStars) {
        projectedPointByCatalogIndex.insert(projected.catalogIndex, projected.point);
    }

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
                std::sqrt(distanceSquared),
                0
            });
        }
    }

    // Limit to the closest few candidates per detection to keep geometric support computation O(1) per detection.
    constexpr int kMaxCandidatesPerDetection = 4;
    std::sort(candidatePairs.begin(), candidatePairs.end(), [](const CandidatePair& lhs, const CandidatePair& rhs) {
        if (lhs.detectionIndex != rhs.detectionIndex) {
            return lhs.detectionIndex < rhs.detectionIndex;
        }
        return lhs.distancePixels < rhs.distancePixels;
    });
    {
        QVector<CandidatePair> cappedPairs;
        cappedPairs.reserve(candidatePairs.size());
        int lastDetectionIndex = -1;
        int countForDetection = 0;
        for (const CandidatePair& pair : candidatePairs) {
            if (pair.detectionIndex != lastDetectionIndex) {
                lastDetectionIndex = pair.detectionIndex;
                countForDetection = 0;
            }
            if (countForDetection < kMaxCandidatesPerDetection) {
                cappedPairs.append(pair);
                ++countForDetection;
            }
        }
        candidatePairs = std::move(cappedPairs);
    }

    for (int i = 0; i < candidatePairs.size(); ++i)
    {
        for (int j = i + 1; j < candidatePairs.size(); ++j)
        {
            const CandidatePair& lhs = candidatePairs[i];
            const CandidatePair& rhs = candidatePairs[j];
            if ((lhs.detectionIndex == rhs.detectionIndex) || (lhs.catalogIndex == rhs.catalogIndex)) {
                continue;
            }

            const QPointF detectionDelta = starDetections[lhs.detectionIndex].m_center - starDetections[rhs.detectionIndex].m_center;
            const double detectionDistance = std::hypot(detectionDelta.x(), detectionDelta.y());
            const QPointF lhsCatalogPoint = projectedPointByCatalogIndex.value(lhs.catalogIndex);
            const QPointF rhsCatalogPoint = projectedPointByCatalogIndex.value(rhs.catalogIndex);
            const QPointF catalogDelta = lhsCatalogPoint - rhsCatalogPoint;
            const double catalogDistance = std::hypot(catalogDelta.x(), catalogDelta.y());
            const double tolerance = std::max(2.0, 0.15 * std::max(detectionDistance, catalogDistance));
            if (std::fabs(detectionDistance - catalogDistance) <= tolerance) {
                ++candidatePairs[i].geometricSupport;
                ++candidatePairs[j].geometricSupport;
            }
        }
    }

    std::sort(candidatePairs.begin(), candidatePairs.end(), [&catalogStars, &starDetections](const CandidatePair& lhs, const CandidatePair& rhs) {
        if (lhs.geometricSupport != rhs.geometricSupport) {
            return lhs.geometricSupport > rhs.geometricSupport;
        }
        if (lhs.distancePixels != rhs.distancePixels) {
            return lhs.distancePixels < rhs.distancePixels;
        }
        if (!qFuzzyCompare(starDetections[lhs.detectionIndex].m_qualityScore + 1.0f, starDetections[rhs.detectionIndex].m_qualityScore + 1.0f)) {
            return starDetections[lhs.detectionIndex].m_qualityScore > starDetections[rhs.detectionIndex].m_qualityScore;
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

void appendSupplementalMatches(const QVector<CameraPipelineStarDetection>& starDetections,
                               const QVector<ProjectedCatalogStar>& projectedStars,
                               double matchRadiusPixels,
                               QVector<Match>& matches)
{
    QVector<bool> detectionMatched(starDetections.size(), false);
    QHash<int, bool> catalogMatched;
    catalogMatched.reserve(matches.size());
    for (const Match& match : matches)
    {
        detectionMatched[match.detectionIndex] = true;
        catalogMatched.insert(match.catalogIndex, true);
    }

    const double maxDistanceSquared = matchRadiusPixels * matchRadiusPixels;
    for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
    {
        if (detectionMatched[detectionIndex]) {
            continue;
        }

        const QPointF detectionPoint = starDetections[detectionIndex].m_center;
        double bestDistanceSquared = std::numeric_limits<double>::max();
        int bestCatalogIndex = -1;

        for (const ProjectedCatalogStar& projected : projectedStars)
        {
            if (catalogMatched.contains(projected.catalogIndex)) {
                continue;
            }

            const double dx = detectionPoint.x() - projected.point.x();
            const double dy = detectionPoint.y() - projected.point.y();
            const double distanceSquared = dx * dx + dy * dy;
            if (distanceSquared > maxDistanceSquared) {
                continue;
            }
            if (distanceSquared < bestDistanceSquared)
            {
                bestDistanceSquared = distanceSquared;
                bestCatalogIndex = projected.catalogIndex;
            }
        }

        if (bestCatalogIndex >= 0)
        {
            const double distancePixels = std::sqrt(bestDistanceSquared);
            matches.append({detectionIndex, bestCatalogIndex, distancePixels});
            detectionMatched[detectionIndex] = true;
            catalogMatched.insert(bestCatalogIndex, true);
            qDebug() << "CameraPlateSolver: supplemental final match"
                     << "detection=" << detectionIndex
                     << "catalog=" << bestCatalogIndex
                     << "distance=" << distancePixels;
        }
    }
}

void logUnmatchedDetections(const CameraSettings& settings,
                            const QVector<CameraPipelineStarDetection>& starDetections,
                            const QVector<ProjectedCatalogStar>& projectedStars,
                            const QVector<Match>& matches,
                            double matchRadiusPixels)
{
    QVector<bool> detectionMatched(starDetections.size(), false);
    QHash<int, bool> catalogMatched;
    catalogMatched.reserve(matches.size());
    for (const Match& match : matches)
    {
        detectionMatched[match.detectionIndex] = true;
        catalogMatched.insert(match.catalogIndex, true);
    }

    const QVector<CatalogStar>& catalogStars = brightStarCatalog(settings);
    for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
    {
        if (detectionMatched[detectionIndex]) {
            continue;
        }

        const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
        double nearestAnyDistance = std::numeric_limits<double>::max();
        int nearestAnyCatalogIndex = -1;
        bool nearestAnyCatalogMatched = false;
        double nearestUnmatchedDistance = std::numeric_limits<double>::max();
        int nearestUnmatchedCatalogIndex = -1;

        for (const ProjectedCatalogStar& projected : projectedStars)
        {
            const double dx = detection.m_center.x() - projected.point.x();
            const double dy = detection.m_center.y() - projected.point.y();
            const double distancePixels = std::hypot(dx, dy);
            if (distancePixels < nearestAnyDistance)
            {
                nearestAnyDistance = distancePixels;
                nearestAnyCatalogIndex = projected.catalogIndex;
                nearestAnyCatalogMatched = catalogMatched.contains(projected.catalogIndex);
            }
            if (!catalogMatched.contains(projected.catalogIndex) && (distancePixels < nearestUnmatchedDistance))
            {
                nearestUnmatchedDistance = distancePixels;
                nearestUnmatchedCatalogIndex = projected.catalogIndex;
            }
        }

        QString reason = QStringLiteral("no nearby catalog candidate");
        if (nearestAnyCatalogIndex >= 0)
        {
            if (nearestAnyDistance <= matchRadiusPixels) {
                reason = nearestAnyCatalogMatched
                    ? QStringLiteral("closest catalog star already matched")
                    : QStringLiteral("candidate within radius was not selected");
            } else if (nearestAnyDistance <= (matchRadiusPixels * 1.5)) {
                reason = QStringLiteral("just outside match radius");
            }
        }

        const QString nearestAnyName =
            ((nearestAnyCatalogIndex >= 0) && (nearestAnyCatalogIndex < catalogStars.size()))
                ? catalogStars[nearestAnyCatalogIndex].name
                : QString();
        const QString nearestUnmatchedName =
            ((nearestUnmatchedCatalogIndex >= 0) && (nearestUnmatchedCatalogIndex < catalogStars.size()))
                ? catalogStars[nearestUnmatchedCatalogIndex].name
                : QString();

        qDebug().noquote()
            << "CameraPlateSolver: unmatched detection"
            << "index=" << detectionIndex
            << "center=" << detection.m_center
            << "quality=" << detection.m_qualityScore
            << "peak=" << detection.m_peakValue
            << "nearestAnyDistance=" << (std::isfinite(nearestAnyDistance) ? nearestAnyDistance : -1.0)
            << "nearestAnyCatalog=" << nearestAnyCatalogIndex
            << "nearestAnyName=" << nearestAnyName
            << "nearestAnyMatched=" << nearestAnyCatalogMatched
            << "nearestUnmatchedDistance=" << (std::isfinite(nearestUnmatchedDistance) ? nearestUnmatchedDistance : -1.0)
            << "nearestUnmatchedCatalog=" << nearestUnmatchedCatalogIndex
            << "nearestUnmatchedName=" << nearestUnmatchedName
            << "reason=" << reason;
    }
}

Evaluation evaluatePose(const CameraSettings& settings,
                        const QSize& imageSize,
                        const QDateTime& captureDateTimeUtc,
                        const QVector<CameraPipelineStarDetection>& starDetections,
                        const QVector<int>& detectionIndices,
                        double azimuthDegrees,
                        double elevationDegrees,
                        double rollDegrees,
                        double fovDegrees,
                        const QVector<int>* allowedCatalogIndices,
                        double centerOffsetXPixels,
                        double centerOffsetYPixels,
                        double distortionK1)
{
    Evaluation evaluation;
    evaluation.azimuthDegrees = normalizeDegrees(azimuthDegrees);
    evaluation.elevationDegrees = elevationDegrees;
    evaluation.rollDegrees = rollDegrees;
    evaluation.fovDegrees = fovDegrees;
    evaluation.centerOffsetXPixels = centerOffsetXPixels;
    evaluation.centerOffsetYPixels = centerOffsetYPixels;
    evaluation.distortionK1 = distortionK1;

    const SkyProjector projector = createProjector(
        settings,
        imageSize,
        evaluation.azimuthDegrees,
        evaluation.elevationDegrees,
        evaluation.rollDegrees,
        evaluation.fovDegrees,
        evaluation.centerOffsetXPixels,
        evaluation.centerOffsetYPixels,
        evaluation.distortionK1);
    if (!projector.valid) {
        return evaluation;
    }

    const QVector<ProjectedCatalogStar> projectedStars = buildProjectedCatalog(
        settings,
        projector,
        captureDateTimeUtc,
        settings.m_plateSolveMaxMagnitude,
        settings.m_plateSolveMatchRadius,
        allowedCatalogIndices);
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

double medianDistancePixels(const QVector<Match>& matches)
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

QVector<Match> rejectOutlierMatches(const QVector<Match>& matches,
                                    int minMatches,
                                    double matchRadiusPixels,
                                    int* outlierCount = nullptr)
{
    if (outlierCount) {
        *outlierCount = 0;
    }

    if (matches.size() <= minMatches) {
        return matches;
    }

    double sumSquaredError = 0.0;
    for (const Match& match : matches) {
        sumSquaredError += match.distancePixels * match.distancePixels;
    }

    const double rmsError = std::sqrt(sumSquaredError / matches.size());
    const double medianError = medianDistancePixels(matches);
    const double threshold = std::min(
        matchRadiusPixels,
        std::max(2.0, std::max(medianError * 2.5, rmsError * 1.75)));

    QVector<Match> inliers;
    inliers.reserve(matches.size());
    for (const Match& match : matches) {
        if (match.distancePixels <= threshold) {
            inliers.append(match);
        }
    }

    if (inliers.size() < minMatches) {
        return matches;
    }

    if (outlierCount) {
        *outlierCount = matches.size() - inliers.size();
    }
    return inliers;
}

bool isAcceptableBlindSolve(const CameraSettings& settings,
                            const QVector<CameraPipelineStarDetection>& starDetections,
                            const QVector<Match>& matches,
                            double rmsErrorPixels,
                            double maxErrorPixels)
{
    const int minAcceptedMatches = std::max(settings.m_plateSolveMinMatches + 2,
        std::min(10, std::max(6, static_cast<int>(std::ceil(static_cast<double>(starDetections.size()) * 0.20)))));
    if (matches.size() < minAcceptedMatches) {
        return false;
    }

    const double medianError = medianDistancePixels(matches);
    const double maxRmsError = std::min(settings.m_plateSolveMatchRadius * 0.70, 20.0);
    const double maxMedianError = std::min(settings.m_plateSolveMatchRadius * 0.55, 15.0);
    const double maxWorstError = std::min(settings.m_plateSolveMatchRadius * 1.10, 45.0);

    return (rmsErrorPixels <= maxRmsError)
        && (medianError <= maxMedianError)
        && (maxErrorPixels <= maxWorstError);
}

bool isStrongGuidedSolve(const CameraSettings& settings,
                         int minMatchCount,
                         const Evaluation& evaluation)
{
    if (!evaluation.valid) {
        return false;
    }

    const int minAcceptedMatches = std::max(minMatchCount + 2, 6);
    const double maxRmsError = std::min(settings.m_plateSolveMatchRadius * 0.45, 12.0);
    return (evaluation.matchCount >= minAcceptedMatches) && (evaluation.rmsErrorPixels <= maxRmsError);
}

bool isAcceptableDirectionSeedSolve(const CameraSettings& settings,
                                    int minMatchCount,
                                    const Evaluation& evaluation)
{
    if (!evaluation.valid) {
        return false;
    }

    const int minAcceptedMatches = std::max(minMatchCount, 4);
    const double maxRmsError = std::min(settings.m_plateSolveMatchRadius * 0.75, 20.0);
    return (evaluation.matchCount >= minAcceptedMatches) && (evaluation.rmsErrorPixels <= maxRmsError);
}

bool isAcceptableElevationSeedEvaluation(const CameraSettings& settings,
                                         int minMatchCount,
                                         const Evaluation& evaluation)
{
    if (!evaluation.valid) {
        return false;
    }

    const int minAcceptedMatches = std::max(minMatchCount + 1, 5);
    const double maxRmsError = std::min(settings.m_plateSolveMatchRadius * 0.85, 22.0);
    return (evaluation.matchCount >= minAcceptedMatches) && (evaluation.rmsErrorPixels <= maxRmsError);
}

bool isAcceptableElevationSeedSolve(const CameraSettings& settings,
                                    const QVector<CameraPipelineStarDetection>& starDetections,
                                    const QVector<Match>& matches,
                                    double rmsErrorPixels,
                                    double maxErrorPixels)
{
    const int minAcceptedMatches = std::max(settings.m_plateSolveMinMatches + 1,
        std::min(8, std::max(5, static_cast<int>(std::ceil(static_cast<double>(starDetections.size()) * 0.15)))));
    if (matches.size() < minAcceptedMatches) {
        return false;
    }

    const double medianError = medianDistancePixels(matches);
    const double maxRmsError = std::min(settings.m_plateSolveMatchRadius * 0.85, 24.0);
    const double maxMedianError = std::min(settings.m_plateSolveMatchRadius * 0.70, 18.0);
    const double maxWorstError = std::min(settings.m_plateSolveMatchRadius * 1.20, 50.0);

    return (rmsErrorPixels <= maxRmsError)
        && (medianError <= maxMedianError)
        && (maxErrorPixels <= maxWorstError);
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
    const double candidateCalibrationMagnitude = std::fabs(candidate.centerOffsetXPixels)
        + std::fabs(candidate.centerOffsetYPixels)
        + 100.0 * std::fabs(candidate.distortionK1);
    const double bestCalibrationMagnitude = std::fabs(best.centerOffsetXPixels)
        + std::fabs(best.centerOffsetYPixels)
        + 100.0 * std::fabs(best.distortionK1);
    if (!qFuzzyCompare(candidateCalibrationMagnitude + 1.0, bestCalibrationMagnitude + 1.0)) {
        return candidateCalibrationMagnitude < bestCalibrationMagnitude;
    }

    return candidate.fovDegrees == best.fovDegrees
        ? candidate.rollDegrees < best.rollDegrees
        : candidate.fovDegrees < best.fovDegrees;
}

void logPlateSolveEvaluation(const char *stage,
                             const Evaluation& evaluation,
                             bool isNewBest = false)
{
    if (!evaluation.valid) {
        return;
    }

    qDebug().noquote().nospace()
        << "CameraPlateSolver[" << stage << "] "
        << (isNewBest ? "best " : "candidate ")
        << "Az=" << evaluation.azimuthDegrees
        << " El=" << evaluation.elevationDegrees
        << " Roll=" << evaluation.rollDegrees
        << " FoV=" << evaluation.fovDegrees
        << " matches=" << evaluation.matchCount
        << " RMS=" << evaluation.rmsErrorPixels
        << " Cx=" << evaluation.centerOffsetXPixels
        << " Cy=" << evaluation.centerOffsetYPixels
        << " K1=" << evaluation.distortionK1;
}

Evaluation searchBestPose(const CameraSettings& settings,
                          const QSize& imageSize,
                          const QDateTime& captureDateTimeUtc,
                          const QVector<CameraPipelineStarDetection>& starDetections,
                          const QVector<int>& detectionIndices)
{
    Evaluation best;
    const int minMatchCount = std::max(1, settings.m_plateSolveMinMatches);
    const bool useStartFov = plateSolveStartUsesFov(settings);
    const bool useStartElevation = plateSolveStartUsesElevation(settings);
    const bool useStartDirection = plateSolveStartUsesDirection(settings);
    const bool useElevationSeedOnly = useStartElevation && !useStartDirection;
    const bool useStartLens = plateSolveStartUsesLens(settings);
    const double fixedCenterOffsetX = useStartLens ? settings.m_lensCenterOffsetX : 0.0;
    const double fixedCenterOffsetY = useStartLens ? settings.m_lensCenterOffsetY : 0.0;
    const double fixedDistortionK1 = useStartLens ? settings.m_lensDistortionK1 : 0.0;

    const double coarseSearchRadius = std::max(0.0, settings.m_plateSolveSearchRadius);
    const double coarseRollRadius = std::max(4.0, std::min(20.0, static_cast<double>(settings.m_fov) * 0.20));
    const double coarseFovRadius = std::max(2.0, std::min(12.0, static_cast<double>(settings.m_fov) * 0.10));

    const double minAzimuthDegrees = 0.0;
    const double maxAzimuthDegrees = 360.0;
    const double azimuthStepDegrees = 5.0;
    const double minElevationDegrees = 0.0;
    const double maxElevationDegrees = 90.0;
    const double elevationStepDegrees = 15.0;

    const std::array<double, 5> coarseOffsets = {{-1.0, -0.5, 0.0, 0.5, 1.0}};
    const std::array<double, 3> coarseFovOffsets = {{-1.0, 0.0, 1.0}};
    const std::array<double, 5> coarseOffsetsOrdered = {{0.0, -0.5, 0.5, -1.0, 1.0}};
    const std::array<double, 3> coarseFovOffsetsOrdered = {{0.0, -1.0, 1.0}};
    const std::array<double, 13> wideRollOffsets = {{-180.0, -150.0, -120.0, -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0}};
    const std::array<double, 13> wideRollOffsetsOrdered = {{0.0, -30.0, 30.0, -60.0, 60.0, -90.0, 90.0, -120.0, 120.0, -150.0, 150.0, -180.0, 180.0}};

    auto evaluateSeed = [&](const char *stage,
                            double azimuthDegrees,
                            double elevationDegrees,
                            double rollDegrees,
                            double fovDegrees) {
        const Evaluation candidate = evaluatePose(
            settings,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            azimuthDegrees,
            elevationDegrees,
            rollDegrees,
            fovDegrees,
            nullptr,
            fixedCenterOffsetX,
            fixedCenterOffsetY,
            fixedDistortionK1);
        logPlateSolveEvaluation(stage, candidate);
        if (isBetterEvaluation(candidate, best)) {
            best = candidate;
            logPlateSolveEvaluation(stage, best, true);
        }
    };

    auto hasGoodGuidedSeed = [&]() {
        if (useStartDirection) {
            return isAcceptableDirectionSeedSolve(settings, minMatchCount, best);
        }
        if (useElevationSeedOnly) {
            return isAcceptableElevationSeedEvaluation(settings, minMatchCount, best);
        }
        if (useStartFov) {
            return isStrongGuidedSolve(settings, minMatchCount, best);
        }
        return false;
    };

    if (useStartDirection)
    {
        bool guidedSatisfied = false;
        for (double fovFactor : coarseFovOffsetsOrdered)
        {
            if (guidedSatisfied) break;
            for (double elFactor : coarseOffsetsOrdered)
            {
                if (guidedSatisfied) break;
                for (double azFactor : coarseOffsetsOrdered)
                {
                    if (guidedSatisfied) break;
                    for (double rollFactor : coarseOffsetsOrdered)
                    {
                        evaluateSeed(
                            "guided-direction",
                            settings.m_azimuth + azFactor * coarseSearchRadius,
                            settings.m_elevation + elFactor * coarseSearchRadius,
                            settings.m_roll + rollFactor * coarseRollRadius,
                            std::max(static_cast<double>(CameraSettings::m_minFov),
                                     static_cast<double>(settings.m_fov) + fovFactor * coarseFovRadius));
                        if (hasGoodGuidedSeed()) {
                            guidedSatisfied = true;
                            break;
                        }
                    }
                }
            }
        }
    }
    else if (useStartElevation)
    {
        const std::array<double, 5> elevationSeedFovScales = {{0.70, 0.85, 1.00, 1.15, 1.30}};
        for (double fovScale : elevationSeedFovScales)
        {
            for (double elFactor : coarseOffsetsOrdered)
            {
                for (double azimuthDegrees = minAzimuthDegrees; azimuthDegrees < maxAzimuthDegrees; azimuthDegrees += azimuthStepDegrees)
                {
                    for (double rollDegrees : wideRollOffsetsOrdered)
                    {
                        evaluateSeed(
                            "guided-elevation",
                            azimuthDegrees,
                            settings.m_elevation + elFactor * coarseSearchRadius,
                            rollDegrees,
                            std::clamp(static_cast<double>(settings.m_fov) * fovScale,
                                static_cast<double>(CameraSettings::m_minFov),
                                static_cast<double>(CameraSettings::m_maxFov)));
                    }
                }
            }
        }

        if (best.valid && (best.matchCount >= std::max(2, minMatchCount - 1)))
        {
            const std::array<double, 5> azimuthOffsets = {{-2.0, -1.0, 0.0, 1.0, 2.0}};
            const std::array<double, 3> elevationOffsets = {{-1.0, 0.0, 1.0}};
            const std::array<double, 5> rollOffsets = {{-2.0, -1.0, 0.0, 1.0, 2.0}};
            const std::array<double, 3> refineFovScales = {{0.92, 1.00, 1.08}};
            const double azimuthRefineStep = 5.0;
            const double elevationRefineStep = std::max(1.0, coarseSearchRadius * 0.25);
            const double rollRefineStep = 5.0;

            for (double azimuthOffset : azimuthOffsets)
            {
                for (double elevationOffset : elevationOffsets)
                {
                    for (double rollOffset : rollOffsets)
                    {
                        for (double fovScale : refineFovScales)
                        {
                            evaluateSeed(
                                "guided-elevation-refine",
                                best.azimuthDegrees + azimuthOffset * azimuthRefineStep,
                                best.elevationDegrees + elevationOffset * elevationRefineStep,
                                best.rollDegrees + rollOffset * rollRefineStep,
                                std::clamp(best.fovDegrees * fovScale,
                                    static_cast<double>(CameraSettings::m_minFov),
                                    static_cast<double>(CameraSettings::m_maxFov)));
                        }
                    }
                }
            }
        }
    }
    else if (useStartFov)
    {
        for (double fovFactor : coarseFovOffsetsOrdered)
        {
            for (double elevationDegrees = minAzimuthDegrees; elevationDegrees <= maxAzimuthDegrees; elevationDegrees += elevationStepDegrees)
            {
                for (double azimuthDegrees = minAzimuthDegrees; azimuthDegrees < maxAzimuthDegrees; azimuthDegrees += azimuthStepDegrees)
                {
                    for (double rollDegrees : wideRollOffsetsOrdered)
                    {
                        evaluateSeed(
                            "guided-fov",
                            azimuthDegrees,
                            elevationDegrees,
                            rollDegrees,
                            std::max(static_cast<double>(CameraSettings::m_minFov),
                                     static_cast<double>(settings.m_fov) + fovFactor * coarseFovRadius));
                    }
                }
            }
        }

    }

    const bool needBlindSearch = !useStartFov
        || (useStartDirection
            ? !isAcceptableDirectionSeedSolve(settings, minMatchCount, best)
            : useElevationSeedOnly
                ? !isAcceptableElevationSeedEvaluation(settings, minMatchCount, best)
            : !isStrongGuidedSolve(settings, minMatchCount, best));

    if (needBlindSearch)
    {
        const QVector<VisibleCatalogStar> visibleStars = buildVisibleCatalog(settings, captureDateTimeUtc, settings.m_plateSolveMaxMagnitude);

        const QVector<Evaluation> blindTriangleSeeds = buildBlindTriangleSeeds(
            settings,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            visibleStars);
        for (const Evaluation& seed : blindTriangleSeeds)
        {
            logPlateSolveEvaluation("blind-triangle-seed", seed);
            if (isBetterEvaluation(seed, best)) {
                best = seed;
                logPlateSolveEvaluation("blind-triangle-seed", best, true);
            }
        }

        const QVector<Evaluation> blindQuadSeeds = buildBlindQuadSeeds(
            settings,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            visibleStars);
        for (const Evaluation& seed : blindQuadSeeds)
        {
            logPlateSolveEvaluation("blind-quad-seed", seed);
            if (isBetterEvaluation(seed, best)) {
                best = seed;
                logPlateSolveEvaluation("blind-quad-seed", best, true);
            }
        }
    }

    if ((!best.valid || (best.matchCount < minMatchCount))
        && (!useStartDirection || !best.valid))
    {
        const std::array<double, 3> wideFovScales = {{0.70, 1.00, 1.30}};
        const std::array<double, 8> wideBlindFovs = {{15.0, 25.0, 40.0, 60.0, 90.0, 130.0, 160.0, 180.0}};
        for (double azimuthDegrees = minAzimuthDegrees; azimuthDegrees < maxAzimuthDegrees; azimuthDegrees += azimuthStepDegrees)
        {
            for (double elevationDegrees = minElevationDegrees; elevationDegrees <= maxElevationDegrees; elevationDegrees += elevationStepDegrees)
            {
                for (double rollDegrees : wideRollOffsets)
                {
                    if (useStartFov)
                    {
                        for (double fovScale : wideFovScales)
                        {
                            evaluateSeed(
                                "wide-fallback-fov",
                                azimuthDegrees,
                                elevationDegrees,
                                rollDegrees,
                                std::clamp(static_cast<double>(settings.m_fov) * fovScale,
                                    static_cast<double>(CameraSettings::m_minFov),
                                    static_cast<double>(CameraSettings::m_maxFov)));
                        }
                    }
                    else
                    {
                        for (double fovDegrees : wideBlindFovs)
                        {
                            evaluateSeed(
                                "wide-fallback-blind",
                                azimuthDegrees,
                                elevationDegrees,
                                rollDegrees,
                                std::clamp(fovDegrees,
                                    static_cast<double>(CameraSettings::m_minFov),
                                    180.0));
                        }
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
    double elStep = azStep;
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
                            elCenter + elOffset * elStep,
                            rollCenter + rollOffset * rollStep,
                            std::max(static_cast<double>(CameraSettings::m_minFov), fovCenter + fovOffset * fovStep),
                            nullptr,
                            fixedCenterOffsetX,
                            fixedCenterOffsetY,
                            fixedDistortionK1);
                        logPlateSolveEvaluation("coarse-refine", candidate);
                        if (isBetterEvaluation(candidate, best)) {
                            best = candidate;
                            logPlateSolveEvaluation("coarse-refine", best, true);
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
        elStep *= 0.5;
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
    elStep = std::max(0.1, elStep);
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
                            elCenter + elOffset * elStep,
                            rollCenter + rollOffset * rollStep,
                            std::max(static_cast<double>(CameraSettings::m_minFov), fovCenter + fovOffset * fovStep),
                            nullptr,
                            fixedCenterOffsetX,
                            fixedCenterOffsetY,
                            fixedDistortionK1);
                        logPlateSolveEvaluation("full-refine", candidate);
                        if (isBetterEvaluation(candidate, best)) {
                            best = candidate;
                            logPlateSolveEvaluation("full-refine", best, true);
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
            elStep *= 0.5;
            rollStep *= 0.5;
            fovStep *= 0.5;
        }
    }

    return best;
}

Evaluation refinePoseFromMatches(const CameraSettings& settings,
                                 const QSize& imageSize,
                                 const QDateTime& captureDateTimeUtc,
                                 const QVector<CameraPipelineStarDetection>& starDetections,
                                 const Evaluation& initialEvaluation)
{
    if (!initialEvaluation.valid || initialEvaluation.matches.isEmpty()) {
        return initialEvaluation;
    }
    const bool calibrateLens = canCalibrateLens(settings);

    int initialOutlierCount = 0;
    const QVector<Match> inlierMatches = rejectOutlierMatches(
        initialEvaluation.matches,
        settings.m_plateSolveMinMatches,
        settings.m_plateSolveMatchRadius,
        &initialOutlierCount);

    QVector<int> detectionIndices;
    QVector<int> catalogIndices;
    detectionIndices.reserve(inlierMatches.size());
    catalogIndices.reserve(inlierMatches.size());
    for (const Match& match : inlierMatches)
    {
        detectionIndices.append(match.detectionIndex);
        catalogIndices.append(match.catalogIndex);
    }

    Evaluation best = evaluatePose(
        settings,
        imageSize,
        captureDateTimeUtc,
        starDetections,
        detectionIndices,
        initialEvaluation.azimuthDegrees,
        initialEvaluation.elevationDegrees,
        initialEvaluation.rollDegrees,
        initialEvaluation.fovDegrees,
        &catalogIndices,
        calibrateLens ? initialEvaluation.centerOffsetXPixels : settings.m_lensCenterOffsetX,
        calibrateLens ? initialEvaluation.centerOffsetYPixels : settings.m_lensCenterOffsetY,
        calibrateLens ? initialEvaluation.distortionK1 : settings.m_lensDistortionK1);
    if (!best.valid) {
        best = initialEvaluation;
    }

    double azCenter = best.azimuthDegrees;
    double elCenter = best.elevationDegrees;
    double rollCenter = best.rollDegrees;
    double fovCenter = best.fovDegrees;
    double centerOffsetXCenter = best.centerOffsetXPixels;
    double centerOffsetYCenter = best.centerOffsetYPixels;
    double distortionCenter = best.distortionK1;
    double azStep = std::max(0.05, settings.m_plateSolveSearchRadius * 0.05);
    double rollStep = std::max(0.10, std::max(1.0, static_cast<double>(settings.m_fov) * 0.02));
    double fovStep = std::max(0.05, std::max(0.5, static_cast<double>(settings.m_fov) * 0.01));
    double centerOffsetXStep = std::max(1.0, static_cast<double>(imageSize.width()) * 0.01);
    double centerOffsetYStep = std::max(1.0, static_cast<double>(imageSize.height()) * 0.01);
    double distortionStep = 0.05;
    const std::array<double, 3> offsets = {{-1.0, 0.0, 1.0}};

    for (int iteration = 0; iteration < 5; ++iteration)
    {
        bool improved = false;
        for (double azOffset : offsets)
        {
            for (double elOffset : offsets)
            {
                for (double rollOffset : offsets)
                {
                    for (double fovOffset : offsets)
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
                            std::max(static_cast<double>(CameraSettings::m_minFov), fovCenter + fovOffset * fovStep),
                            &catalogIndices,
                            centerOffsetXCenter,
                            centerOffsetYCenter,
                            distortionCenter);
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

    if (calibrateLens)
    {
        azCenter = best.azimuthDegrees;
        elCenter = best.elevationDegrees;
        rollCenter = best.rollDegrees;
        fovCenter = best.fovDegrees;
        centerOffsetXCenter = best.centerOffsetXPixels;
        centerOffsetYCenter = best.centerOffsetYPixels;
        distortionCenter = best.distortionK1;

        for (int iteration = 0; iteration < 4; ++iteration)
        {
            bool improved = false;
            for (double centerOffsetXOffset : offsets)
            {
                for (double centerOffsetYOffset : offsets)
                {
                    for (double distortionOffset : offsets)
                    {
                        const Evaluation candidate = evaluatePose(
                            settings,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            detectionIndices,
                            azCenter,
                            elCenter,
                            rollCenter,
                            fovCenter,
                            &catalogIndices,
                            centerOffsetXCenter + centerOffsetXOffset * centerOffsetXStep,
                            centerOffsetYCenter + centerOffsetYOffset * centerOffsetYStep,
                            std::clamp(distortionCenter + distortionOffset * distortionStep, -0.75, 0.75));
                        if (isBetterEvaluation(candidate, best)) {
                            best = candidate;
                            improved = true;
                        }
                    }
                }
            }

            centerOffsetXCenter = best.centerOffsetXPixels;
            centerOffsetYCenter = best.centerOffsetYPixels;
            distortionCenter = best.distortionK1;
            if (!improved) {
                centerOffsetXStep *= 0.5;
                centerOffsetYStep *= 0.5;
                distortionStep *= 0.5;
            }
        }
    }
    else
    {
        best.centerOffsetXPixels = settings.m_lensCenterOffsetX;
        best.centerOffsetYPixels = settings.m_lensCenterOffsetY;
        best.distortionK1 = settings.m_lensDistortionK1;
    }

    azCenter = best.azimuthDegrees;
    elCenter = best.elevationDegrees;
    rollCenter = best.rollDegrees;
    fovCenter = best.fovDegrees;
    centerOffsetXCenter = best.centerOffsetXPixels;
    centerOffsetYCenter = best.centerOffsetYPixels;
    distortionCenter = best.distortionK1;

    for (int iteration = 0; iteration < 2; ++iteration)
    {
        bool improved = false;
        for (double azOffset : offsets)
        {
            for (double elOffset : offsets)
            {
                for (double rollOffset : offsets)
                {
                    for (double fovOffset : offsets)
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
                            std::max(static_cast<double>(CameraSettings::m_minFov), fovCenter + fovOffset * fovStep),
                            &catalogIndices,
                            centerOffsetXCenter,
                            centerOffsetYCenter,
                            distortionCenter);
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
        detection.m_projectedCenter = QPointF();
        detection.m_matchDistancePixels = 0.0f;
        detection.m_catalogMagnitude = 0.0f;
        detection.m_catalogSpectralType.clear();
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

    const QVector<CatalogStar> reducedCatalog = filterCatalogStars(parseDownloadedHygCatalog(QString::fromUtf8(uncompressedData)));
    if (reducedCatalog.isEmpty())
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Imported HYG catalog did not yield any usable plate-solver stars.");
        }
        return false;
    }

    if (!writeReducedCatalog(reducedCatalog, downloadedCatalogReducedPath(), errorMessage)) {
        return false;
    }

    return true;
}

CameraPlateSolveResult CameraPlateSolver::solve(const CameraSettings& settings,
                                                const QSize& imageSize,
                                                const QDateTime& captureDateTime,
                                                QVector<CameraPipelineStarDetection>& starDetections)
{
    PROFILER_START();

    CameraPlateSolveResult result;
    clearSolvedStars(starDetections);
    result.m_catalogSource = currentCatalogSource(settings);
    result.m_catalogStarsLoaded = brightStarCatalog(settings).size();
    result.m_detectedStarsConsidered = starDetections.size();
    const bool useStartElevation = plateSolveStartUsesElevation(settings);
    const bool useStartDirection = plateSolveStartUsesDirection(settings);
    const bool useElevationSeedOnly = useStartElevation && !useStartDirection;

    if ((starDetections.size() < settings.m_plateSolveMinMatches)) {
        return result;
    }

    const QVector<int> detectionIndices = selectDetectionIndicesForSolve(starDetections, imageSize);
    if (detectionIndices.size() < settings.m_plateSolveMinMatches) {
        return result;
    }

    const QDateTime solveDateTime = settings.m_plateSolveUseCurrentDateTime
        ? QDateTime::currentDateTime()
        : (settings.m_plateSolveDateTime.isValid() ? settings.m_plateSolveDateTime : captureDateTime);
    const QDateTime captureDateTimeUtc = (solveDateTime.isValid() ? solveDateTime : QDateTime::currentDateTime()).toUTC();
    Evaluation best = searchBestPose(settings, imageSize, captureDateTimeUtc, starDetections, detectionIndices);
    if (!best.valid || (best.matchCount < settings.m_plateSolveMinMatches)) {
        return result;
    }
    best = refinePoseFromMatches(settings, imageSize, captureDateTimeUtc, starDetections, best);
    logPlateSolveEvaluation("refine-from-matches", best, true);
    if (!best.valid || (best.matchCount < settings.m_plateSolveMinMatches)) {
        qDebug() << "CameraPlateSolver: refinePoseFromMatches failed to keep a valid solution";
        return result;
    }

    const SkyProjector finalProjector = createProjector(
        settings,
        imageSize,
        best.azimuthDegrees,
        best.elevationDegrees,
        best.rollDegrees,
        best.fovDegrees,
        best.centerOffsetXPixels,
        best.centerOffsetYPixels,
        best.distortionK1);
    if (!finalProjector.valid) {
        return result;
    }

    const QVector<ProjectedCatalogStar> projectedStars = buildProjectedCatalog(
        settings,
        finalProjector,
        captureDateTimeUtc,
        settings.m_plateSolveMaxMagnitude,
        settings.m_plateSolveMatchRadius);
    result.m_catalogCandidateStars = projectedStars.size();
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

    int outlierCount = 0;
    QVector<Match> finalMatches = rejectOutlierMatches(
        allMatches,
        settings.m_plateSolveMinMatches,
        settings.m_plateSolveMatchRadius,
        &outlierCount);
    result.m_outlierStars = outlierCount;

    appendSupplementalMatches(
        starDetections,
        projectedStars,
        settings.m_plateSolveMatchRadius,
        finalMatches);

    if (finalMatches.size() < settings.m_plateSolveMinMatches) {
        PROFILER_STOP(__FUNCTION__ ": insufficient matches");
        return result;
    }

    double sumSquaredError = 0.0;
    double maxError = 0.0;
    QHash<int, QPointF> projectedPointsByCatalogIndex;
    for (const ProjectedCatalogStar& projectedStar : projectedStars) {
        projectedPointsByCatalogIndex.insert(projectedStar.catalogIndex, projectedStar.point);
    }
    for (const Match& match : finalMatches)
    {
        CameraPipelineStarDetection& detection = starDetections[match.detectionIndex];
        const CatalogStar& catalogStar = brightStarCatalog(settings)[match.catalogIndex];
        detection.m_label = catalogStar.name;
        detection.m_projectedCenter = projectedPointsByCatalogIndex.value(match.catalogIndex);
        detection.m_matchDistancePixels = static_cast<float>(match.distancePixels);
        detection.m_catalogMagnitude = static_cast<float>(catalogStar.magnitude);
        detection.m_catalogSpectralType = catalogStar.spectralType;
        detection.m_solved = true;
        sumSquaredError += match.distancePixels * match.distancePixels;
        maxError = std::max(maxError, match.distancePixels);
    }

    result.m_solved = true;
    result.m_matchedStars = finalMatches.size();
    result.m_rmsErrorPixels = std::sqrt(sumSquaredError / finalMatches.size());
    result.m_maxErrorPixels = maxError;
    if (useElevationSeedOnly
        && !isAcceptableElevationSeedSolve(settings, starDetections, finalMatches, result.m_rmsErrorPixels, result.m_maxErrorPixels))
    {
        qDebug() << "CameraPlateSolver: rejecting elevation-seeded solution"
                 << "matches=" << finalMatches.size()
                 << "rms=" << result.m_rmsErrorPixels
                 << "max=" << result.m_maxErrorPixels;
        clearSolvedStars(starDetections);
        PROFILER_STOP(__FUNCTION__ ": unacceptable elevation-seeded solve");
        return CameraPlateSolveResult();
    }

    if (!useStartElevation
        && !useStartDirection
        && !isAcceptableBlindSolve(settings, starDetections, finalMatches, result.m_rmsErrorPixels, result.m_maxErrorPixels))
    {
        qDebug() << "CameraPlateSolver: rejecting blind solution"
                 << "matches=" << finalMatches.size()
                 << "rms=" << result.m_rmsErrorPixels
                 << "max=" << result.m_maxErrorPixels;
        clearSolvedStars(starDetections);
        PROFILER_STOP(__FUNCTION__ ": unacceptable blind solve");
        return CameraPlateSolveResult();
    }
    result.m_azimuthDegrees = best.azimuthDegrees;
    result.m_elevationDegrees = best.elevationDegrees;
    result.m_rollDegrees = best.rollDegrees;
    result.m_fovDegrees = best.fovDegrees;
    result.m_centerOffsetXPixels = best.centerOffsetXPixels;
    result.m_centerOffsetYPixels = best.centerOffsetYPixels;
    result.m_distortionK1 = best.distortionK1;

    logUnmatchedDetections(
        settings,
        starDetections,
        projectedStars,
        finalMatches,
        settings.m_plateSolveMatchRadius);

    PROFILER_STOP(__FUNCTION__);

    return result;
}
