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

struct PlateSolveCatalogContext
{
    const QVector<CatalogStar>* catalogStars = nullptr;
    QVector<VisibleCatalogStar> visibleStars;
    QHash<int, int> visibleStarIndexByCatalogIndex;
};

struct CandidatePair
{
    int detectionIndex = -1;
    int catalogIndex = -1;
    int projectedIndex = -1;
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

struct FinalMatchPassEvaluation
{
    bool projectorValid = false;
    Evaluation pose;
    QVector<ProjectedCatalogStar> projectedStars;
    QVector<Match> finalMatches;
    int rawMatchCount = 0;
    int outlierCount = 0;
    double rmsErrorPixels = std::numeric_limits<double>::infinity();
    double medianErrorPixels = std::numeric_limits<double>::infinity();
    double maxErrorPixels = std::numeric_limits<double>::infinity();
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
constexpr bool kLogPlateSolveCandidates = false;
constexpr bool kLogWeakModeCandidatePools = true;
constexpr bool kLogWeakModeTailRejects = false;

// Normalisation radius (pixels) used by the weak-mode scoring comparator and coarse
// candidate-pool admission. Set once at the start of CameraPlateSolver::solve() so weak
// FoV/Blind searches compare basins against the loose acquisition geometry rather than the
// tighter final acceptance radius.
thread_local double g_weakModeNormalizationPixels = 24.0;
const char* const kBundledCatalogPath = ":/camera/brightstarcatalog.txt";
const char* const kDownloadedCatalogDir = "camera";
const char* const kDownloadedCatalogArchiveFile = "hyg_v42.csv.gz";
const char* const kDownloadedCatalogCsvFile = "hyg_v42.csv";
const char* const kDownloadedCatalogReducedFile = "hyg_v42_reduced.txt";

Evaluation evaluatePose(const CameraSettings& settings,
                        const PlateSolveCatalogContext& catalogContext,
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
                        double distortionK1 = 0.0,
                        double matchRadiusOverride = -1.0);

Evaluation refinePoseFromMatches(const CameraSettings& settings,
                                 const PlateSolveCatalogContext& catalogContext,
                                 const QSize& imageSize,
                                 const QDateTime& captureDateTimeUtc,
                                 const QVector<CameraPipelineStarDetection>& starDetections,
                                 const Evaluation& initialEvaluation);

QVector<Match> rejectOutlierMatches(const QVector<Match>& matches,
                                    int minMatches,
                                    double matchRadiusPixels,
                                    int* outlierCount);

bool isBetterWeakModeEvaluation(const Evaluation& candidate, const Evaluation& best);
bool isBetterWeakModeRefinedEvaluation(const Evaluation& candidate, const Evaluation& best);
bool isBetterEvaluationForMode(const Evaluation& candidate,
                               const Evaluation& best,
                               bool useWeakModeScoring);
double weakModeEvaluationScore(const Evaluation& evaluation,
                               double normalizationRadius = g_weakModeNormalizationPixels);
bool sameEvaluationIdentity(const Evaluation& lhs, const Evaluation& rhs);
void logWeakModePoolDecision(const char *stage,
                             const char *decision,
                             const Evaluation& candidate,
                             double poolQualityRadius,
                             const Evaluation *other = nullptr);
void logWeakModeCandidatePool(const char *stage, const QVector<Evaluation>& candidates);

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

bool plateSolveStartUsesCurrentSettingsOnly(const CameraSettings& settings)
{
    return settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartCurrentSettingsOnly;
}

bool plateSolveStartUsesElevation(const CameraSettings& settings)
{
    switch (settings.m_plateSolveStartMode)
    {
    case CameraSettings::PlateSolveStartFovElevation:
    case CameraSettings::PlateSolveStartFovAzElRoll:
    case CameraSettings::PlateSolveStartFovAzElRollLens:
    case CameraSettings::PlateSolveStartCurrentSettingsOnly:
        return true;
    default:
        return false;
    }
}

bool plateSolveStartUsesDirection(const CameraSettings& settings)
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

bool plateSolveStartUsesLens(const CameraSettings& settings)
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

bool canCalibrateLens(const CameraSettings& settings)
{
    return settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartFovAzElRollLens;
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

bool projectVector(const SkyProjector& projector, const SkyVector& vector, QPointF& point)
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

bool projectAltAz(const SkyProjector& projector, double azimuthDegrees, double elevationDegrees, QPointF& point)
{
    return projectVector(projector, vectorFromAltAz(azimuthDegrees, elevationDegrees), point);
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

QVector<ProjectedCatalogStar> buildProjectedCatalog(const PlateSolveCatalogContext& catalogContext,
                                                    const SkyProjector& projector,
                                                    double searchMarginPixels,
                                                    const QVector<int>* allowedCatalogIndices = nullptr)
{
    QVector<ProjectedCatalogStar> projectedStars;
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

    if (allowedCatalogIndices)
    {
        projectedStars.reserve(allowedCatalogIndices->size());
        for (int catalogIndex : *allowedCatalogIndices)
        {
            const auto it = catalogContext.visibleStarIndexByCatalogIndex.constFind(catalogIndex);
            if (it != catalogContext.visibleStarIndexByCatalogIndex.cend()) {
                appendProjectedStar(catalogContext.visibleStars[*it]);
            }
        }
    }
    else
    {
        projectedStars.reserve(catalogContext.visibleStars.size());
        for (const VisibleCatalogStar& visibleStar : catalogContext.visibleStars) {
            appendProjectedStar(visibleStar);
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

PlateSolveCatalogContext buildPlateSolveCatalogContext(const CameraSettings& settings,
                                                       const QDateTime& captureDateTimeUtc,
                                                       double maxMagnitude)
{
    PlateSolveCatalogContext context;
    context.catalogStars = &brightStarCatalog(settings);
    context.visibleStars = buildVisibleCatalog(settings, captureDateTimeUtc, maxMagnitude);
    context.visibleStarIndexByCatalogIndex.reserve(context.visibleStars.size());
    for (int i = 0; i < context.visibleStars.size(); ++i) {
        context.visibleStarIndexByCatalogIndex.insert(context.visibleStars[i].catalogIndex, i);
    }
    return context;
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
    // Score blind seeds against the *final* (tighter) acceptance radius — the acquisition
    // radius is intentionally generous so that buildMatches finds candidates, but a strong
    // seed must already be well-localised relative to where the final solver will accept.
    const double seedRadius = std::min(
        static_cast<double>(settings.m_plateSolveMatchRadius),
        static_cast<double>(settings.m_plateSolveFinalMatchRadius));
    const double maxRmsError = std::min(seedRadius * 0.60, kBlindSeedMaxRmsPixels);
    const double maxMedianError = std::min(seedRadius * 0.50, kBlindSeedMaxMedianPixels);
    return (candidate.rmsErrorPixels <= maxRmsError) && (medianError <= maxMedianError);
}

Evaluation verifyBlindSeedCandidate(const CameraSettings& settings,
                                    const PlateSolveCatalogContext& catalogContext,
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
        catalogContext,
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
                const SkyVector& va = visibleStars[i].vector;
                const SkyVector& vb = visibleStars[j].vector;
                const SkyVector& vc = visibleStars[k].vector;
                const SkyVector center = normalize({va.x + vb.x + vc.x,
                                                    va.y + vb.y + vc.y,
                                                    va.z + vb.z + vc.z});
                if (length(center) <= 0.0) {
                    continue;
                }

                // Use the synthetic projector for both ratios *and* orientation so the sign
                // convention matches the detection-side 2D cross product. Ratios computed in
                // pure angular space would be projection-independent, but then the orientation
                // sign would have to match a y-flip convention that is fragile to changes in
                // the SkyProjector implementation. Both ratios and orientation come from the
                // same projected points here so the signs are guaranteed to match the detection
                // triangles built from real image coordinates.
                const double centerAzimuth = normalizeDegrees(std::atan2(center.x, center.y) * 180.0 / kPi);
                const double centerElevation = std::asin(std::clamp(center.z, -1.0, 1.0)) * 180.0 / kPi;
                // Pick a synthetic FoV broad enough to encompass the actual triangle's angular
                // span. Previously this used `max(20, settings.m_fov)`, which under-projected
                // wide-field triangles (forcing them through edge distortion) and biased the
                // ratios.  Compute the maximum pairwise angle so we can scale appropriately.
                const double maxPairAngleRad = std::max({
                    std::acos(std::clamp(dot(va, vb), -1.0, 1.0)),
                    std::acos(std::clamp(dot(va, vc), -1.0, 1.0)),
                    std::acos(std::clamp(dot(vb, vc), -1.0, 1.0))
                });
                const double maxPairAngleDeg = maxPairAngleRad * 180.0 / kPi;
                const double syntheticFov = std::clamp(
                    std::max(maxPairAngleDeg * 2.5, static_cast<double>(settings.m_fov)),
                    20.0,
                    160.0);
                const SkyProjector localProjector = createProjector(
                    settings,
                    QSize(1000, 1000),
                    centerAzimuth,
                    centerElevation,
                    0.0,
                    syntheticFov);
                if (!localProjector.valid) {
                    continue;
                }

                std::array<QPointF, 3> points;
                bool allProjected = true;
                const std::array<int, 3> starIndices {{i, j, k}};
                for (int pointIndex = 0; pointIndex < 3; ++pointIndex)
                {
                    if (!projectVector(localProjector,
                                       visibleStars[starIndices[pointIndex]].vector,
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
                    // Scale the synthetic projector's FoV to the actual angular span of the
                    // quad so wide-field constellations don't get pushed through projection
                    // distortion at the synthetic edges. See the matching comment in
                    // buildCatalogTriangleSignatures for rationale.
                    const SkyVector& va = visibleStars[i].vector;
                    const SkyVector& vb = visibleStars[j].vector;
                    const SkyVector& vc = visibleStars[k].vector;
                    const SkyVector& vd = visibleStars[l].vector;
                    const double maxPairAngleRad = std::max({
                        std::acos(std::clamp(dot(va, vb), -1.0, 1.0)),
                        std::acos(std::clamp(dot(va, vc), -1.0, 1.0)),
                        std::acos(std::clamp(dot(va, vd), -1.0, 1.0)),
                        std::acos(std::clamp(dot(vb, vc), -1.0, 1.0)),
                        std::acos(std::clamp(dot(vb, vd), -1.0, 1.0)),
                        std::acos(std::clamp(dot(vc, vd), -1.0, 1.0))
                    });
                    const double maxPairAngleDeg = maxPairAngleRad * 180.0 / kPi;
                    const double syntheticFov = std::clamp(
                        std::max(maxPairAngleDeg * 2.5, static_cast<double>(settings.m_fov)),
                        20.0,
                        160.0);
                    const SkyProjector localProjector = createProjector(
                        settings,
                        QSize(1000, 1000),
                        centerAzimuth,
                        centerElevation,
                        0.0,
                        syntheticFov);
                    if (!localProjector.valid) {
                        continue;
                    }

                    std::array<QPointF, 4> points;
                    bool allProjected = true;
                    const std::array<int, 4> starIndices {{i, j, k, l}};
                    for (int pointIndex = 0; pointIndex < 4; ++pointIndex)
                    {
                        if (!projectVector(localProjector,
                                           visibleStars[starIndices[pointIndex]].vector,
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
                                            const PlateSolveCatalogContext& catalogContext,
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
    const bool useStartLens = plateSolveStartUsesLens(settings);
    const double fixedCenterOffsetX = useStartLens ? settings.m_lensCenterOffsetX : 0.0;
    const double fixedCenterOffsetY = useStartLens ? settings.m_lensCenterOffsetY : 0.0;
    const double fixedDistortionK1 = useStartLens ? settings.m_lensDistortionK1 : 0.0;

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

            // Skip sky directions already tried by a previous triangle match. The dedup
            // radius scales with the seed FoV: at 90° FoV the original 3° tolerance is fine,
            // but at 15-25° wide-field blind FoVs a 3° basin can swallow the correct
            // direction after a near-miss. Use 5% of seed FoV with a 0.5°-5° clamp.
            const double dedupRadiusDegrees = std::clamp(baseSeedFov * 0.05, 0.5, 5.0);
            bool alreadyTried = false;
            for (const TriedDirection& tried : triedDirections) {
                if (std::fabs(seedAzimuth - tried.azimuthDegrees) < dedupRadiusDegrees
                    && std::fabs(seedElevation - tried.elevationDegrees) < dedupRadiusDegrees)
                {
                    alreadyTried = true;
                    break;
                }
            }
            if (alreadyTried) continue;
            triedDirections.append({seedAzimuth, seedElevation});

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

            // Sweep FoV around the seed estimate. The base estimate is derived from a
            // rectilinear scale model (image_width / longest_pixel_distance), which biases
            // the result low for fisheye lenses; widen the sweep accordingly. Also clamp to
            // CameraSettings::m_maxFov for consistency with the base-clamp above — the
            // earlier hard-coded 180.0 ceiling was inconsistent with the rest of the file.
            const bool isFisheyeLens = (settings.m_lensProjection != CameraSettings::LensProjectionRectilinear);
            const std::array<double, 5> rectilinearFovScales = {{0.85, 0.93, 1.0, 1.07, 1.15}};
            const std::array<double, 5> fisheyeFovScales = {{0.60, 0.80, 1.0, 1.25, 1.60}};
            const auto& fovScales = isFisheyeLens ? fisheyeFovScales : rectilinearFovScales;
            for (double fovScale : fovScales)
            {
                if (earlyExit) break;
                const double seedFov = std::clamp(
                    baseSeedFov * fovScale,
                    static_cast<double>(CameraSettings::m_minFov),
                    static_cast<double>(CameraSettings::m_maxFov));
                const SkyProjector rollProjector = createProjector(
                    settings,
                    imageSize,
                    seedAzimuth,
                    seedElevation,
                    0.0,
                    seedFov,
                    fixedCenterOffsetX,
                    fixedCenterOffsetY,
                    fixedDistortionK1);
                if (!rollProjector.valid) {
                    continue;
                }

                std::array<QPointF, 3> projectedPoints;
                bool projected = true;
                for (int idx = 0; idx < 3; ++idx)
                {
                    if (!projectVector(rollProjector, triangleStars[idx].vector, projectedPoints[idx]))
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
                            catalogContext,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            detectionIndices,
                            seedAzimuth,
                            seedElevation,
                            seedRoll,
                            seedFov,
                            &allowedCatalogIndices,
                            fixedCenterOffsetX,
                            fixedCenterOffsetY,
                            fixedDistortionK1);
                        const Evaluation verifiedCandidate = verifyBlindSeedCandidate(
                            settings,
                            catalogContext,
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
        return isBetterWeakModeEvaluation(lhs, rhs);
    });
    if (seeds.size() > 16) {
        seeds.resize(16);
    }

    return seeds;
}

QVector<Evaluation> buildBlindQuadSeeds(const CameraSettings& settings,
                                        const PlateSolveCatalogContext& catalogContext,
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
    const bool useStartLens = plateSolveStartUsesLens(settings);
    const double fixedCenterOffsetX = useStartLens ? settings.m_lensCenterOffsetX : 0.0;
    const double fixedCenterOffsetY = useStartLens ? settings.m_lensCenterOffsetY : 0.0;
    const double fixedDistortionK1 = useStartLens ? settings.m_lensDistortionK1 : 0.0;

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

            // FoV-scaled dedup so wide-field seeds don't swallow nearby distinct directions.
            const double dedupRadiusDegrees = std::clamp(baseSeedFov * 0.05, 0.5, 5.0);
            bool alreadyTried = false;
            for (const TriedDirection& tried : triedDirections) {
                if (std::fabs(seedAzimuth - tried.azimuthDegrees) < dedupRadiusDegrees
                    && std::fabs(seedElevation - tried.elevationDegrees) < dedupRadiusDegrees)
                {
                    alreadyTried = true;
                    break;
                }
            }
            if (alreadyTried) continue;
            triedDirections.append({seedAzimuth, seedElevation});

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

            // See buildBlindTriangleSeeds for the rationale on the broader fisheye sweep.
            const bool isFisheyeLensQ = (settings.m_lensProjection != CameraSettings::LensProjectionRectilinear);
            const std::array<double, 5> rectilinearQuadFovScales = {{0.85, 0.95, 1.0, 1.10, 1.20}};
            const std::array<double, 5> fisheyeQuadFovScales = {{0.60, 0.80, 1.0, 1.25, 1.60}};
            const auto& quadFovScales = isFisheyeLensQ ? fisheyeQuadFovScales : rectilinearQuadFovScales;
            for (double fovScale : quadFovScales)
            {
                if (earlyExit) break;
                const double seedFov = std::clamp(
                    baseSeedFov * fovScale,
                    static_cast<double>(CameraSettings::m_minFov),
                    static_cast<double>(CameraSettings::m_maxFov));
                const SkyProjector rollProjector = createProjector(
                    settings,
                    imageSize,
                    seedAzimuth,
                    seedElevation,
                    0.0,
                    seedFov,
                    fixedCenterOffsetX,
                    fixedCenterOffsetY,
                    fixedDistortionK1);
                if (!rollProjector.valid) {
                    continue;
                }

                std::array<QPointF, 4> projectedPoints;
                bool projected = true;
                for (int idx = 0; idx < 4; ++idx)
                {
                    if (!projectVector(rollProjector, quadStars[idx].vector, projectedPoints[idx]))
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
                            catalogContext,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            detectionIndices,
                            seedAzimuth,
                            seedElevation,
                            seedRoll,
                            seedFov,
                            &allowedCatalogIndices,
                            fixedCenterOffsetX,
                            fixedCenterOffsetY,
                            fixedDistortionK1);
                        const Evaluation verifiedCandidate = verifyBlindSeedCandidate(
                            settings,
                            catalogContext,
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
        return isBetterWeakModeEvaluation(lhs, rhs);
    });
    if (seeds.size() > 12) {
        seeds.resize(12);
    }

    return seeds;
}

quint64 spatialCellKey(int x, int y)
{
    return (static_cast<quint64>(static_cast<quint32>(x)) << 32)
        | static_cast<quint32>(y);
}

QVector<Match> buildMatches(const PlateSolveCatalogContext& catalogContext,
                            const QVector<CameraPipelineStarDetection>& starDetections,
                            const QVector<int>& detectionIndices,
                            const QVector<ProjectedCatalogStar>& projectedStars,
                            double matchRadiusPixels)
{
    QVector<CandidatePair> candidatePairs;
    const QVector<CatalogStar>& catalogStars = *catalogContext.catalogStars;
    const double maxDistanceSquared = matchRadiusPixels * matchRadiusPixels;
    const double cellSize = std::max(1.0, matchRadiusPixels);
    QHash<quint64, QVector<int>> projectedStarGrid;
    projectedStarGrid.reserve(projectedStars.size());
    for (int projectedIndex = 0; projectedIndex < projectedStars.size(); ++projectedIndex)
    {
        const QPointF& point = projectedStars[projectedIndex].point;
        const int cellX = static_cast<int>(std::floor(point.x() / cellSize));
        const int cellY = static_cast<int>(std::floor(point.y() / cellSize));
        projectedStarGrid[spatialCellKey(cellX, cellY)].append(projectedIndex);
    }

    for (int detectionIndex : detectionIndices)
    {
        const QPointF detectionPoint = starDetections[detectionIndex].m_center;
        const int cellX = static_cast<int>(std::floor(detectionPoint.x() / cellSize));
        const int cellY = static_cast<int>(std::floor(detectionPoint.y() / cellSize));
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                const auto it = projectedStarGrid.constFind(spatialCellKey(cellX + dx, cellY + dy));
                if (it == projectedStarGrid.cend()) {
                    continue;
                }

                for (int projectedIndex : it.value())
                {
                    const ProjectedCatalogStar& projected = projectedStars[projectedIndex];
                    const double deltaX = detectionPoint.x() - projected.point.x();
                    const double deltaY = detectionPoint.y() - projected.point.y();
                    const double distanceSquared = deltaX * deltaX + deltaY * deltaY;
                    if (distanceSquared > maxDistanceSquared) {
                        continue;
                    }

                    candidatePairs.append({
                        detectionIndex,
                        projected.catalogIndex,
                        projectedIndex,
                        std::sqrt(distanceSquared),
                        0
                    });
                }
            }
        }
    }

    // Allow a wider candidate set for large acquisition radii so wide-field weak modes
    // do not discard the true match before geometric support is computed.
    const int maxCandidatesPerDetection = std::clamp(
        static_cast<int>(std::ceil(matchRadiusPixels / 15.0)),
        4,
        10);
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
            if (countForDetection < maxCandidatesPerDetection) {
                cappedPairs.append(pair);
                ++countForDetection;
            }
        }
        candidatePairs = std::move(cappedPairs);
    }

    // Geometric-support tally. For each candidate (detection, catalog) pair we count how
    // many *other* candidate pairs agree on the inter-star distance — i.e. the detection-
    // space and catalog-space distances match within tolerance. A naive O(N^2) sweep can
    // be ~65k comparisons for N=256, and this runs inside every evaluatePose call, so we
    // also cap support accumulation per pair to avoid quadratic blow-up on dense fields.
    constexpr int kGeometricSupportCap = 8;
    for (int i = 0; i < candidatePairs.size(); ++i)
    {
        if (candidatePairs[i].geometricSupport >= kGeometricSupportCap) {
            continue;
        }
        const CandidatePair& lhs = candidatePairs[i];
        const QPointF lhsDetection = starDetections[lhs.detectionIndex].m_center;
        const QPointF& lhsCatalogPoint = projectedStars[lhs.projectedIndex].point;
        for (int j = i + 1; j < candidatePairs.size(); ++j)
        {
            CandidatePair& rhsRef = candidatePairs[j];
            if (rhsRef.geometricSupport >= kGeometricSupportCap) {
                continue;
            }
            const CandidatePair& rhs = rhsRef;
            if ((lhs.detectionIndex == rhs.detectionIndex) || (lhs.catalogIndex == rhs.catalogIndex)) {
                continue;
            }

            const double detDx = lhsDetection.x() - starDetections[rhs.detectionIndex].m_center.x();
            const double detDy = lhsDetection.y() - starDetections[rhs.detectionIndex].m_center.y();
            const double detectionDistance = std::hypot(detDx, detDy);
            const QPointF& rhsCatalogPoint = projectedStars[rhs.projectedIndex].point;
            const double catDx = lhsCatalogPoint.x() - rhsCatalogPoint.x();
            const double catDy = lhsCatalogPoint.y() - rhsCatalogPoint.y();
            const double catalogDistance = std::hypot(catDx, catDy);
            const double tolerance = std::max(2.0, 0.15 * std::max(detectionDistance, catalogDistance));
            if (std::fabs(detectionDistance - catalogDistance) <= tolerance) {
                ++candidatePairs[i].geometricSupport;
                ++rhsRef.geometricSupport;
                if (candidatePairs[i].geometricSupport >= kGeometricSupportCap) {
                    break;
                }
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

    for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
    {
        if (detectionMatched[detectionIndex]) {
            continue;
        }

        const QPointF detectionPoint = starDetections[detectionIndex].m_center;
        const int cellX = static_cast<int>(std::floor(detectionPoint.x() / cellSize));
        const int cellY = static_cast<int>(std::floor(detectionPoint.y() / cellSize));
        double bestDistanceSquared = std::numeric_limits<double>::max();
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
                    if (distanceSquared < bestDistanceSquared)
                    {
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
    }
}

void logUnmatchedDetections(const CameraSettings& settings,
                            const QVector<CameraPipelineStarDetection>& starDetections,
                            const QVector<ProjectedCatalogStar>& projectedStars,
                            const QVector<Match>& matches,
                            double matchRadiusPixels)
{
    if (!kLogPlateSolveCandidates) {
        return;
    }
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
                        const PlateSolveCatalogContext& catalogContext,
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
                        double distortionK1,
                        double matchRadiusOverride)
{
    Evaluation evaluation;
    evaluation.azimuthDegrees = normalizeDegrees(azimuthDegrees);
    evaluation.elevationDegrees = elevationDegrees;
    evaluation.rollDegrees = rollDegrees;
    evaluation.fovDegrees = fovDegrees;
    evaluation.centerOffsetXPixels = centerOffsetXPixels;
    evaluation.centerOffsetYPixels = centerOffsetYPixels;
    evaluation.distortionK1 = distortionK1;

    // Default to the acquisition radius; callers (e.g. refinePoseFromMatches' final pass)
    // can pass the tighter final-match radius so the split-radii intent ("search wide,
    // accept narrow") actually influences scoring during refinement.
    const double matchRadiusPixels = (matchRadiusOverride > 0.0)
        ? matchRadiusOverride
        : settings.m_plateSolveMatchRadius;

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
        catalogContext,
        projector,
        matchRadiusPixels,
        allowedCatalogIndices);
    if (projectedStars.isEmpty()) {
        return evaluation;
    }

    evaluation.matches = buildMatches(
        catalogContext,
        starDetections,
        detectionIndices,
        projectedStars,
        matchRadiusPixels);
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
    const double maxRmsError = std::min(settings.m_plateSolveFinalMatchRadius * 0.70, 20.0);
    const double maxMedianError = std::min(settings.m_plateSolveFinalMatchRadius * 0.55, 15.0);
    const double maxWorstError = std::min(settings.m_plateSolveFinalMatchRadius * 1.10, 45.0);

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
    const double maxRmsError = std::min(settings.m_plateSolveFinalMatchRadius * 0.45, 12.0);
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
    const double maxRmsError = std::min(settings.m_plateSolveFinalMatchRadius * 0.75, 20.0);
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
    const double maxRmsError = std::min(settings.m_plateSolveFinalMatchRadius * 0.85, 22.0);
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
    const double maxRmsError = std::min(settings.m_plateSolveFinalMatchRadius * 0.85, 24.0);
    const double maxMedianError = std::min(settings.m_plateSolveFinalMatchRadius * 0.70, 18.0);
    const double maxWorstError = std::min(settings.m_plateSolveFinalMatchRadius * 1.20, 50.0);

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

double weakModeEvaluationScore(const Evaluation& evaluation,
                               double normalizationRadius)
{
    if (!evaluation.valid) {
        return -std::numeric_limits<double>::infinity();
    }

    // Non-linear penalty proportional to the acceptance radius: a small RMS leaves the score
    // essentially equal to matchCount, but as RMS approaches the radius the per-match value
    // collapses to ~0.5 so a single false coincidence cannot outweigh a tighter cluster.
    const double safeRadius = std::max(1.0, normalizationRadius);
    const double normalizedRms = evaluation.rmsErrorPixels / safeRadius;
    const double clampedRms = std::min(1.0, std::max(0.0, normalizedRms));
    const double perMatchQuality = 1.0 - 0.5 * clampedRms * clampedRms;
    return static_cast<double>(evaluation.matchCount) * perMatchQuality;
}

bool isBetterWeakModeEvaluation(const Evaluation& candidate, const Evaluation& best)
{
    if (!candidate.valid) {
        return false;
    }
    if (!best.valid) {
        return true;
    }

    const double candidateScore = weakModeEvaluationScore(candidate);
    const double bestScore = weakModeEvaluationScore(best);
    if (std::fabs(candidateScore - bestScore) > 1e-6) {
        return candidateScore > bestScore;
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

bool isBetterWeakModeRefinedEvaluation(const Evaluation& candidate, const Evaluation& best)
{
    if (!candidate.valid) {
        return false;
    }
    if (!best.valid) {
        return true;
    }

    const int matchDelta = candidate.matchCount - best.matchCount;
    if (std::abs(matchDelta) <= 1)
    {
        const double candidateMedian = medianDistancePixels(candidate.matches);
        const double bestMedian = medianDistancePixels(best.matches);
        if (!qFuzzyCompare(candidateMedian + 1.0, bestMedian + 1.0)) {
            return candidateMedian < bestMedian;
        }
        if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
            return candidate.rmsErrorPixels < best.rmsErrorPixels;
        }
    }

    return isBetterWeakModeEvaluation(candidate, best);
}

FinalMatchPassEvaluation evaluateFinalMatchPass(const CameraSettings& settings,
                                                const PlateSolveCatalogContext& catalogContext,
                                                const QSize& imageSize,
                                                const QVector<CameraPipelineStarDetection>& starDetections,
                                                const QVector<int>& allDetectionIndices,
                                                const Evaluation& candidate,
                                                double finalMatchRadius)
{
    FinalMatchPassEvaluation finalPass;
    finalPass.pose = candidate;

    const SkyProjector projector = createProjector(
        settings,
        imageSize,
        candidate.azimuthDegrees,
        candidate.elevationDegrees,
        candidate.rollDegrees,
        candidate.fovDegrees,
        candidate.centerOffsetXPixels,
        candidate.centerOffsetYPixels,
        candidate.distortionK1);
    if (!projector.valid) {
        return finalPass;
    }

    finalPass.projectorValid = true;
    finalPass.projectedStars = buildProjectedCatalog(
        catalogContext,
        projector,
        finalMatchRadius);

    const QVector<Match> allMatches = buildMatches(
        catalogContext,
        starDetections,
        allDetectionIndices,
        finalPass.projectedStars,
        finalMatchRadius);
    finalPass.rawMatchCount = allMatches.size();

    finalPass.finalMatches = rejectOutlierMatches(
        allMatches,
        settings.m_plateSolveMinMatches,
        finalMatchRadius,
        &finalPass.outlierCount);

    appendSupplementalMatches(
        starDetections,
        finalPass.projectedStars,
        finalMatchRadius,
        finalPass.finalMatches);

    if (!finalPass.finalMatches.isEmpty())
    {
        double sumSquaredError = 0.0;
        double maxError = 0.0;
        for (const Match& match : finalPass.finalMatches)
        {
            sumSquaredError += match.distancePixels * match.distancePixels;
            maxError = std::max(maxError, match.distancePixels);
        }
        finalPass.rmsErrorPixels = std::sqrt(sumSquaredError / finalPass.finalMatches.size());
        finalPass.medianErrorPixels = medianDistancePixels(finalPass.finalMatches);
        finalPass.maxErrorPixels = maxError;
    }

    return finalPass;
}

void logFinalMatchPassEvaluation(const char *stage,
                                 const FinalMatchPassEvaluation& evaluation,
                                 bool best = false)
{
    if (!evaluation.projectorValid) {
        return;
    }

    qDebug().noquote().nospace()
        << "CameraPlateSolver[" << stage << "] "
        << (best ? "best " : "candidate ")
        << "Az=" << evaluation.pose.azimuthDegrees
        << " El=" << evaluation.pose.elevationDegrees
        << " Roll=" << evaluation.pose.rollDegrees
        << " FoV=" << evaluation.pose.fovDegrees
        << " finalMatches=" << evaluation.finalMatches.size()
        << " rawMatches=" << evaluation.rawMatchCount
        << " RMS=" << evaluation.rmsErrorPixels
        << " Median=" << evaluation.medianErrorPixels
        << " Max=" << evaluation.maxErrorPixels
        << " projectedStars=" << evaluation.projectedStars.size()
        << " K1=" << evaluation.pose.distortionK1;
}

bool isBetterFinalPassEvaluation(const Evaluation& candidate,
                                 const Evaluation& best,
                                 int retainedMatchThreshold)
{
    if (!candidate.valid) {
        return false;
    }
    if (!best.valid) {
        return true;
    }

    const bool candidateMeetsThreshold = candidate.matchCount >= retainedMatchThreshold;
    const bool bestMeetsThreshold = best.matchCount >= retainedMatchThreshold;
    if (candidateMeetsThreshold != bestMeetsThreshold) {
        return candidateMeetsThreshold;
    }

    if (candidateMeetsThreshold)
    {
        const double candidateMedian = medianDistancePixels(candidate.matches);
        const double bestMedian = medianDistancePixels(best.matches);
        if (!qFuzzyCompare(candidateMedian + 1.0, bestMedian + 1.0)) {
            return candidateMedian < bestMedian;
        }
        if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
            return candidate.rmsErrorPixels < best.rmsErrorPixels;
        }
        if (candidate.matchCount != best.matchCount) {
            return candidate.matchCount > best.matchCount;
        }
        return isBetterEvaluation(candidate, best);
    }

    if (candidate.matchCount != best.matchCount) {
        return candidate.matchCount > best.matchCount;
    }
    if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
        return candidate.rmsErrorPixels < best.rmsErrorPixels;
    }

    return isBetterEvaluation(candidate, best);
}

int minimumRetainedMatchesForFinalPass(const Evaluation& reference,
                                       int minMatchCount)
{
    if (!reference.valid) {
        return std::max(3, minMatchCount);
    }

    // Keep the tightening pass honest: it should preserve most of the coarse/refined
    // correspondences, not collapse to a tiny high-confidence subset that hijacks the solve.
    const int relativeFloor = static_cast<int>(std::ceil(static_cast<double>(reference.matchCount) * 0.70));
    return std::min(reference.matchCount, std::max(minMatchCount, std::max(3, relativeFloor)));
}

bool isBetterWeakModeFinalMatchPass(const CameraSettings& settings,
                                    const QVector<CameraPipelineStarDetection>& starDetections,
                                    bool blindMode,
                                    const FinalMatchPassEvaluation& candidate,
                                    const FinalMatchPassEvaluation& best)
{
    if (!candidate.projectorValid) {
        return false;
    }
    if (!best.projectorValid) {
        return true;
    }

    const bool candidateMeetsMinMatches = candidate.finalMatches.size() >= settings.m_plateSolveMinMatches;
    const bool bestMeetsMinMatches = best.finalMatches.size() >= settings.m_plateSolveMinMatches;
    const bool candidateBlindAccepted = blindMode
        && candidateMeetsMinMatches
        && isAcceptableBlindSolve(settings, starDetections, candidate.finalMatches, candidate.rmsErrorPixels, candidate.maxErrorPixels);
    const bool bestBlindAccepted = blindMode
        && bestMeetsMinMatches
        && isAcceptableBlindSolve(settings, starDetections, best.finalMatches, best.rmsErrorPixels, best.maxErrorPixels);

    if (blindMode && (candidateBlindAccepted != bestBlindAccepted)) {
        return candidateBlindAccepted;
    }
    if (candidateMeetsMinMatches != bestMeetsMinMatches) {
        return candidateMeetsMinMatches;
    }

    const int finalMatchDelta = static_cast<int>(candidate.finalMatches.size()) - static_cast<int>(best.finalMatches.size());
    if (std::abs(finalMatchDelta) <= 1)
    {
        if (!qFuzzyCompare(candidate.medianErrorPixels + 1.0, best.medianErrorPixels + 1.0)) {
            return candidate.medianErrorPixels < best.medianErrorPixels;
        }
        if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
            return candidate.rmsErrorPixels < best.rmsErrorPixels;
        }
    }

    if (finalMatchDelta != 0) {
        return finalMatchDelta > 0;
    }
    if (candidate.rawMatchCount != best.rawMatchCount) {
        return candidate.rawMatchCount > best.rawMatchCount;
    }

    return isBetterWeakModeRefinedEvaluation(candidate.pose, best.pose);
}

bool isBetterEvaluationForMode(const Evaluation& candidate,
                               const Evaluation& best,
                               bool useWeakModeScoring)
{
    return useWeakModeScoring
        ? isBetterWeakModeEvaluation(candidate, best)
        : isBetterEvaluation(candidate, best);
}

double angularDistanceDegrees(double lhs, double rhs)
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

bool sameEvaluationBasin(const Evaluation& lhs, const Evaluation& rhs)
{
    return angularDistanceDegrees(lhs.azimuthDegrees, rhs.azimuthDegrees) <= 20.0
        && std::fabs(lhs.elevationDegrees - rhs.elevationDegrees) <= 10.0
        && angularDistanceDegrees(lhs.rollDegrees, rhs.rollDegrees) <= 20.0
        && std::fabs(lhs.fovDegrees - rhs.fovDegrees) <= 10.0;
}

void insertDistinctEvaluationCandidate(QVector<Evaluation>& candidates,
                                       const Evaluation& candidate,
                                       int maxCandidates,
                                       bool useWeakModeScoring = false,
                                       const char *stage = nullptr,
                                       int interestingMatchCount = 0,
                                       int minPoolMatchCount = 3)
{
    if (!candidate.valid) {
        return;
    }

    // Quality floor: don't admit obvious noise into the multi-hypothesis pool. A candidate
    // must have at least 3 matches *and* an RMS well inside the acquisition radius. This is
    // intentionally based on the coarse acquisition geometry, not the much tighter final
    // acceptance radius, so rough-but-promising weak-mode basins survive long enough to be
    // rescored and refined.
    const int effectiveMinPoolMatchCount = std::max(3, minPoolMatchCount);
    if (candidate.matchCount < effectiveMinPoolMatchCount) {
        if (useWeakModeScoring && stage && (candidate.matchCount >= interestingMatchCount)) {
            logWeakModePoolDecision(stage, "reject-too-few-matches", candidate, 0.0);
        }
        return;
    }
    const double poolQualityRadius = std::max(2.0, g_weakModeNormalizationPixels * 0.95);
    if (candidate.rmsErrorPixels > poolQualityRadius) {
        if (useWeakModeScoring && stage && (candidate.matchCount >= interestingMatchCount)) {
            logWeakModePoolDecision(stage, "reject-rms-floor", candidate, poolQualityRadius);
        }
        return;
    }

    for (Evaluation& existing : candidates)
    {
        if (sameEvaluationBasin(candidate, existing))
        {
            if (isBetterEvaluationForMode(candidate, existing, useWeakModeScoring)) {
                if (useWeakModeScoring && stage && (candidate.matchCount >= interestingMatchCount)) {
                    logWeakModePoolDecision(stage, "replace-same-basin", candidate, poolQualityRadius, &existing);
                }
                existing = candidate;
            } else if (useWeakModeScoring && stage && (candidate.matchCount >= interestingMatchCount)) {
                logWeakModePoolDecision(stage, "keep-existing-basin", candidate, poolQualityRadius, &existing);
            }
            return;
        }
    }

    // Once the weak-mode pool is full, avoid appending/sorting candidates that cannot
    // possibly survive. This cuts a lot of blind/FoV churn without changing the winner
    // selection logic for genuinely competitive basins.
    if (useWeakModeScoring && (candidates.size() >= maxCandidates) && !candidates.isEmpty())
    {
        const Evaluation& weakestCandidate = candidates.constLast();
        if (!isBetterEvaluationForMode(candidate, weakestCandidate, useWeakModeScoring))
        {
            const bool nearTailChallenge = (candidate.matchCount >= weakestCandidate.matchCount)
                || (candidate.rmsErrorPixels <= weakestCandidate.rmsErrorPixels);
            if (stage && nearTailChallenge && (candidate.matchCount >= interestingMatchCount)) {
                logWeakModePoolDecision(stage, "reject-below-tail", candidate, poolQualityRadius, &weakestCandidate);
            }
            return;
        }
    }

    candidates.append(candidate);
    std::sort(candidates.begin(), candidates.end(), [useWeakModeScoring](const Evaluation& lhs, const Evaluation& rhs) {
        return isBetterEvaluationForMode(lhs, rhs, useWeakModeScoring);
    });
    bool candidateKept = true;
    if (candidates.size() > maxCandidates) {
        candidates.resize(maxCandidates);
        candidateKept = std::any_of(candidates.cbegin(), candidates.cend(), [&candidate](const Evaluation& existing) {
            return sameEvaluationIdentity(existing, candidate);
        });
    }
    if (useWeakModeScoring && stage && (candidate.matchCount >= interestingMatchCount)) {
        if (candidateKept) {
            logWeakModePoolDecision(stage, "add-distinct-basin", candidate, poolQualityRadius);
        } else {
            logWeakModePoolDecision(stage, "drop-pool-full", candidate, poolQualityRadius);
        }
    }
}

Evaluation rescoreWeakModeCandidateWithDistortionSweep(const CameraSettings& settings,
                                                       const PlateSolveCatalogContext& catalogContext,
                                                       const QSize& imageSize,
                                                       const QDateTime& captureDateTimeUtc,
                                                       const QVector<CameraPipelineStarDetection>& starDetections,
                                                       const QVector<int>& detectionIndices,
                                                       const Evaluation& candidate)
{
    if (!candidate.valid) {
        return candidate;
    }

    // Preserve the user's calibrated lens centre when we can't calibrate the lens during
    // this solve; previously this function unconditionally zeroed the centre offsets,
    // throwing away a manual calibration and biasing the rescore against it. Likewise the
    // distortion sweep only makes sense when we're allowed to calibrate the lens — if the
    // user disabled calibration, a single re-evaluation with their settings is enough.
    const bool calibrate = canCalibrateLens(settings);
    const double baseCenterOffsetX = calibrate ? candidate.centerOffsetXPixels : settings.m_lensCenterOffsetX;
    const double baseCenterOffsetY = calibrate ? candidate.centerOffsetYPixels : settings.m_lensCenterOffsetY;
    const double baseDistortionK1 = calibrate ? candidate.distortionK1 : settings.m_lensDistortionK1;

    Evaluation best = candidate;
    const std::array<double, 4> distortionSweep = {{-0.05, -0.025, 0.0, 0.025}};
    for (double distortionDelta : distortionSweep)
    {
        const double distortionK1 = calibrate ? distortionDelta : baseDistortionK1;
        const Evaluation rescored = evaluatePose(
            settings,
            catalogContext,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            candidate.azimuthDegrees,
            candidate.elevationDegrees,
            candidate.rollDegrees,
            candidate.fovDegrees,
            nullptr,
            baseCenterOffsetX,
            baseCenterOffsetY,
            distortionK1);
        if (isBetterWeakModeEvaluation(rescored, best)) {
            best = rescored;
        }
        if (!calibrate) {
            // No need to sweep distortion if we can't calibrate; the loop body is identical.
            break;
        }
    }

    return best;
}

void logPlateSolveEvaluation(const char *stage,
                             const Evaluation& evaluation,
                             bool isNewBest = false,
                             bool forceLog = false)
{
    if (!evaluation.valid || (!isNewBest && !kLogPlateSolveCandidates && !forceLog)) {
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

bool sameEvaluationIdentity(const Evaluation& lhs, const Evaluation& rhs)
{
    return lhs.valid == rhs.valid
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

void logWeakModePoolDecision(const char *stage,
                             const char *decision,
                             const Evaluation& candidate,
                             double poolQualityRadius,
                             const Evaluation *other)
{
    if (!kLogWeakModeCandidatePools || !candidate.valid) {
        return;
    }
    if (!kLogWeakModeTailRejects && (qstrcmp(decision, "reject-below-tail") == 0)) {
        return;
    }

    qDebug().noquote().nospace()
        << "CameraPlateSolver[" << stage << "] "
        << decision
        << " score=" << weakModeEvaluationScore(candidate)
        << " matches=" << candidate.matchCount
        << " RMS=" << candidate.rmsErrorPixels
        << " Median=" << medianDistancePixels(candidate.matches)
        << " poolRmsCap=" << poolQualityRadius
        << " Az=" << candidate.azimuthDegrees
        << " El=" << candidate.elevationDegrees
        << " Roll=" << candidate.rollDegrees
        << " FoV=" << candidate.fovDegrees
        << " K1=" << candidate.distortionK1;

    if (other && other->valid) {
        qDebug().noquote().nospace()
            << "CameraPlateSolver[" << stage << "] "
            << "compared-to"
            << " score=" << weakModeEvaluationScore(*other)
            << " matches=" << other->matchCount
            << " RMS=" << other->rmsErrorPixels
            << " Median=" << medianDistancePixels(other->matches)
            << " Az=" << other->azimuthDegrees
            << " El=" << other->elevationDegrees
            << " Roll=" << other->rollDegrees
            << " FoV=" << other->fovDegrees
            << " K1=" << other->distortionK1;
    }
}

void logWeakModeCandidatePool(const char *stage, const QVector<Evaluation>& candidates)
{
    if (!kLogWeakModeCandidatePools) {
        return;
    }

    qDebug().noquote().nospace()
        << "CameraPlateSolver[" << stage << "] pool-size=" << candidates.size();
    for (int i = 0; i < candidates.size(); ++i)
    {
        const Evaluation& candidate = candidates.at(i);
        qDebug().noquote().nospace()
            << "CameraPlateSolver[" << stage << "] #"<< i
            << " score=" << weakModeEvaluationScore(candidate)
            << " matches=" << candidate.matchCount
            << " RMS=" << candidate.rmsErrorPixels
            << " Median=" << medianDistancePixels(candidate.matches)
            << " Az=" << candidate.azimuthDegrees
            << " El=" << candidate.elevationDegrees
            << " Roll=" << candidate.rollDegrees
            << " FoV=" << candidate.fovDegrees
            << " Cx=" << candidate.centerOffsetXPixels
            << " Cy=" << candidate.centerOffsetYPixels
            << " K1=" << candidate.distortionK1;
    }
}

Evaluation searchBestPose(const CameraSettings& settings,
                          const PlateSolveCatalogContext& catalogContext,
                          const QSize& imageSize,
                          const QDateTime& captureDateTimeUtc,
                          const QVector<CameraPipelineStarDetection>& starDetections,
                          const QVector<int>& detectionIndices,
                          QVector<Evaluation>* candidatePool = nullptr)
{
    Evaluation best;
    constexpr int kMaxMultiHypothesisCandidates = 10;
    const int minMatchCount = std::max(1, settings.m_plateSolveMinMatches);
    const bool useStartFov = plateSolveStartUsesFov(settings);
    const bool useStartElevation = plateSolveStartUsesElevation(settings);
    const bool useStartDirection = plateSolveStartUsesDirection(settings);
    const bool useElevationSeedOnly = useStartElevation && !useStartDirection;
    const bool useStartLens = plateSolveStartUsesLens(settings);
    const bool useWeakModeScoring = !useStartDirection && !useElevationSeedOnly;
    const bool keepMultipleCandidates = candidatePool && !useStartDirection;
    const int interestingWeakModeMatchCount = std::max(3, minMatchCount - 1);
    const int weakModeCandidatePoolMinMatches = std::max(3, minMatchCount - 2);
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
            catalogContext,
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
        if (keepMultipleCandidates) {
            insertDistinctEvaluationCandidate(
                *candidatePool,
                candidate,
                kMaxMultiHypothesisCandidates,
                useWeakModeScoring,
                stage,
                interestingWeakModeMatchCount,
                weakModeCandidatePoolMinMatches);
        }
        if (isBetterEvaluationForMode(candidate, best, useWeakModeScoring)) {
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
        const std::array<double, 5> elevationSeedFovScales = {{1.00, 0.85, 1.15, 0.70, 1.30}};
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
            for (double elevationDegrees = minElevationDegrees; elevationDegrees <= maxElevationDegrees; elevationDegrees += elevationStepDegrees)
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
        const QVector<VisibleCatalogStar>& visibleStars = catalogContext.visibleStars;

        const QVector<Evaluation> blindTriangleSeeds = buildBlindTriangleSeeds(
            settings,
            catalogContext,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            visibleStars);
        for (const Evaluation& seed : blindTriangleSeeds)
        {
            logPlateSolveEvaluation("blind-triangle-seed", seed);
            if (candidatePool) {
                insertDistinctEvaluationCandidate(
                    *candidatePool,
                    seed,
                    kMaxMultiHypothesisCandidates,
                    useWeakModeScoring,
                    "blind-triangle-seed",
                    interestingWeakModeMatchCount,
                    weakModeCandidatePoolMinMatches);
            }
            if (isBetterEvaluationForMode(seed, best, useWeakModeScoring)) {
                best = seed;
                logPlateSolveEvaluation("blind-triangle-seed", best, true);
            }
        }

        const QVector<Evaluation> blindQuadSeeds = buildBlindQuadSeeds(
            settings,
            catalogContext,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            visibleStars);
        for (const Evaluation& seed : blindQuadSeeds)
        {
            logPlateSolveEvaluation("blind-quad-seed", seed);
            if (candidatePool) {
                insertDistinctEvaluationCandidate(
                    *candidatePool,
                    seed,
                    kMaxMultiHypothesisCandidates,
                    useWeakModeScoring,
                    "blind-quad-seed",
                    interestingWeakModeMatchCount,
                    weakModeCandidatePoolMinMatches);
            }
            if (isBetterEvaluationForMode(seed, best, useWeakModeScoring)) {
                best = seed;
                logPlateSolveEvaluation("blind-quad-seed", best, true);
            }
        }
    }

    // Short-circuit the exhaustive wide-fallback grid when the blind seeds have already
    // landed in a reasonable basin. The grid runs ~52k evaluatePose calls (72 az × 7 el ×
    // 13 roll × 8 fov), so any half-decent prior result is worth keeping. Acceptance bar:
    // at least half the minimum required matches *and* an RMS that fits inside the
    // acquisition radius — the subsequent refinement loops will tighten this further.
    const double wideFallbackRmsCap = std::max(
        static_cast<double>(settings.m_plateSolveMatchRadius) * 0.9,
        2.0);
    const bool blindSeedAlreadyAcceptable = best.valid
        && (best.matchCount >= std::max(2, minMatchCount / 2))
        && (best.rmsErrorPixels <= wideFallbackRmsCap);

    if ((!best.valid || (best.matchCount < minMatchCount))
        && (!useStartDirection || !best.valid)
        && !blindSeedAlreadyAcceptable)
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
                            catalogContext,
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
                        if (keepMultipleCandidates) {
                            insertDistinctEvaluationCandidate(
                                *candidatePool,
                            candidate,
                            kMaxMultiHypothesisCandidates,
                            useWeakModeScoring,
                            "coarse-refine",
                            interestingWeakModeMatchCount,
                            weakModeCandidatePoolMinMatches);
                        }
                        if (isBetterEvaluationForMode(candidate, best, useWeakModeScoring)) {
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
                            catalogContext,
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
                        if (keepMultipleCandidates) {
                            insertDistinctEvaluationCandidate(
                                *candidatePool,
                            candidate,
                            kMaxMultiHypothesisCandidates,
                            useWeakModeScoring,
                            "full-refine",
                            interestingWeakModeMatchCount,
                            weakModeCandidatePoolMinMatches);
                        }
                        if (isBetterEvaluationForMode(candidate, best, useWeakModeScoring)) {
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
                                 const PlateSolveCatalogContext& catalogContext,
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
        catalogContext,
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
    double elStep = azStep;
    double rollStep = std::max(0.10, std::max(1.0, static_cast<double>(settings.m_fov) * 0.02));
    double fovStep = std::max(0.05, std::max(0.5, static_cast<double>(settings.m_fov) * 0.01));
    double centerOffsetXStep = std::max(1.0, static_cast<double>(imageSize.width()) * 0.01);
    double centerOffsetYStep = std::max(1.0, static_cast<double>(imageSize.height()) * 0.01);
    double distortionStep = 0.05;
    const std::array<double, 3> offsets = {{-1.0, 0.0, 1.0}};

    for (int iteration = 0; iteration < 5; ++iteration)
    {
        bool improvedAz = false;
        bool improvedEl = false;
        bool improvedRoll = false;
        bool improvedFov = false;
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
                            catalogContext,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            detectionIndices,
                            azCenter + azOffset * azStep,
                            elCenter + elOffset * elStep,
                            rollCenter + rollOffset * rollStep,
                            std::max(static_cast<double>(CameraSettings::m_minFov), fovCenter + fovOffset * fovStep),
                            &catalogIndices,
                            centerOffsetXCenter,
                            centerOffsetYCenter,
                            distortionCenter);
                        if (isBetterEvaluation(candidate, best)) {
                            best = candidate;
                            if (azOffset != 0.0) improvedAz = true;
                            if (elOffset != 0.0) improvedEl = true;
                            if (rollOffset != 0.0) improvedRoll = true;
                            if (fovOffset != 0.0) improvedFov = true;
                        }
                    }
                }
            }
        }

        azCenter = best.azimuthDegrees;
        elCenter = best.elevationDegrees;
        rollCenter = best.rollDegrees;
        fovCenter = best.fovDegrees;
        // Per-axis shrinking: only shrink the axes whose best offset was 0 this iteration.
        // This avoids prematurely shrinking an axis that just hasn't been visited yet because
        // another axis improved first.
        if (!improvedAz)   azStep   *= 0.5;
        if (!improvedEl)   elStep   *= 0.5;
        if (!improvedRoll) rollStep *= 0.5;
        if (!improvedFov)  fovStep  *= 0.5;
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
                            catalogContext,
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

    // Tighten in stages. Jumping directly from the loose acquisition radius to the tight final
    // radius can leave wide-field weak solves stuck with a coarse-but-high-match-count pose that
    // never wins the final comparison, even when the correct basin is nearby.
    const double finalPassRadius = std::min(
        static_cast<double>(settings.m_plateSolveMatchRadius),
        std::max(1.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius)));
    const double intermediatePassRadius = std::min(
        static_cast<double>(settings.m_plateSolveMatchRadius),
        std::max(finalPassRadius + 10.0, finalPassRadius * 2.0));
    QVector<double> tighteningPassRadii;
    if (intermediatePassRadius > (finalPassRadius + 1e-6)) {
        tighteningPassRadii.append(intermediatePassRadius);
    }
    tighteningPassRadii.append(finalPassRadius);

    for (double tighteningRadius : tighteningPassRadii)
    {
        const Evaluation preTighteningBest = best;
        const int retainedMatchThreshold = minimumRetainedMatchesForFinalPass(
            preTighteningBest,
            settings.m_plateSolveMinMatches);
        Evaluation tighteningBest = evaluatePose(
            settings,
            catalogContext,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            azCenter,
            elCenter,
            rollCenter,
            fovCenter,
            &catalogIndices,
            centerOffsetXCenter,
            centerOffsetYCenter,
            distortionCenter,
            tighteningRadius);

        for (int iteration = 0; iteration < 2; ++iteration)
        {
            bool improvedAz = false;
            bool improvedEl = false;
            bool improvedRoll = false;
            bool improvedFov = false;
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
                                catalogContext,
                                imageSize,
                                captureDateTimeUtc,
                                starDetections,
                                detectionIndices,
                                azCenter + azOffset * azStep,
                                elCenter + elOffset * elStep,
                                rollCenter + rollOffset * rollStep,
                                std::max(static_cast<double>(CameraSettings::m_minFov), fovCenter + fovOffset * fovStep),
                                &catalogIndices,
                                centerOffsetXCenter,
                                centerOffsetYCenter,
                                distortionCenter,
                                tighteningRadius);
                            if (isBetterFinalPassEvaluation(candidate, tighteningBest, retainedMatchThreshold)) {
                                tighteningBest = candidate;
                                if (azOffset != 0.0) improvedAz = true;
                                if (elOffset != 0.0) improvedEl = true;
                                if (rollOffset != 0.0) improvedRoll = true;
                                if (fovOffset != 0.0) improvedFov = true;
                            }
                        }
                    }
                }
            }

            if (tighteningBest.valid) {
                azCenter = tighteningBest.azimuthDegrees;
                elCenter = tighteningBest.elevationDegrees;
                rollCenter = tighteningBest.rollDegrees;
                fovCenter = tighteningBest.fovDegrees;
            }
            if (!improvedAz)   azStep   *= 0.5;
            if (!improvedEl)   elStep   *= 0.5;
            if (!improvedRoll) rollStep *= 0.5;
            if (!improvedFov)  fovStep  *= 0.5;
        }

        if (tighteningBest.valid && (tighteningBest.matchCount >= retainedMatchThreshold)) {
            best = tighteningBest;
            azCenter = best.azimuthDegrees;
            elCenter = best.elevationDegrees;
            rollCenter = best.rollDegrees;
            fovCenter = best.fovDegrees;
            centerOffsetXCenter = best.centerOffsetXPixels;
            centerOffsetYCenter = best.centerOffsetYPixels;
            distortionCenter = best.distortionK1;
        } else {
            best = preTighteningBest;
            azCenter = best.azimuthDegrees;
            elCenter = best.elevationDegrees;
            rollCenter = best.rollDegrees;
            fovCenter = best.fovDegrees;
            centerOffsetXCenter = best.centerOffsetXPixels;
            centerOffsetYCenter = best.centerOffsetYPixels;
            distortionCenter = best.distortionK1;
            break;
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
    const bool useCurrentSettingsOnly = plateSolveStartUsesCurrentSettingsOnly(settings);
    const bool useStartFov = plateSolveStartUsesFov(settings);
    const bool useStartElevation = plateSolveStartUsesElevation(settings);
    const bool useStartDirection = plateSolveStartUsesDirection(settings);
    const bool useElevationSeedOnly = useStartElevation && !useStartDirection;
    const double finalMatchRadius = settings.m_plateSolveFinalMatchRadius;
    const bool useMultiHypothesisRefine = !useCurrentSettingsOnly && !useStartDirection;
    const bool useWeakModeScoring = !useStartDirection && !useElevationSeedOnly;

    // Configure weak-mode scoring normalisation for this solve. Weak FoV/Blind searches need
    // to rank and preserve coarse basins using the loose acquisition geometry; the tighter
    // final-match radius is reserved for the late refinement/acceptance stages.
    g_weakModeNormalizationPixels = std::max(1.0, static_cast<double>(settings.m_plateSolveMatchRadius));

    if (starDetections.isEmpty()) {
        return result;
    }

    const QDateTime solveDateTime = settings.m_plateSolveUseCurrentDateTime
        ? QDateTime::currentDateTime()
        : (settings.m_plateSolveDateTime.isValid() ? settings.m_plateSolveDateTime : captureDateTime);
    const QDateTime captureDateTimeUtc = (solveDateTime.isValid() ? solveDateTime : QDateTime::currentDateTime()).toUTC();
    const PlateSolveCatalogContext catalogContext = buildPlateSolveCatalogContext(
        settings,
        captureDateTimeUtc,
        settings.m_plateSolveMaxMagnitude);

    const QVector<int> allDetectionIndices = [&starDetections]() {
        QVector<int> indices;
        indices.reserve(starDetections.size());
        for (int i = 0; i < starDetections.size(); ++i) {
            indices.append(i);
        }
        return indices;
    }();

    if (useCurrentSettingsOnly)
    {
        const SkyProjector currentSettingsProjector = createProjector(
            settings,
            imageSize,
            settings.m_azimuth,
            settings.m_elevation,
            settings.m_roll,
            settings.m_fov,
            settings.m_lensCenterOffsetX,
            settings.m_lensCenterOffsetY,
            settings.m_lensDistortionK1);
        if (!currentSettingsProjector.valid) {
            PROFILER_STOP(__FUNCTION__ ": invalid current-settings projector");
            return result;
        }

        const QVector<ProjectedCatalogStar> projectedStars = buildProjectedCatalog(
            catalogContext,
            currentSettingsProjector,
            finalMatchRadius);
        result.m_catalogCandidateStars = projectedStars.size();

        const QVector<Match> allMatches = buildMatches(
            catalogContext,
            starDetections,
            allDetectionIndices,
            projectedStars,
            finalMatchRadius);

        int outlierCount = 0;
        QVector<Match> finalMatches = rejectOutlierMatches(
            allMatches,
            settings.m_plateSolveMinMatches,
            finalMatchRadius,
            &outlierCount);
        result.m_outlierStars = outlierCount;

        appendSupplementalMatches(
            starDetections,
            projectedStars,
            finalMatchRadius,
            finalMatches);

        if (finalMatches.isEmpty()) {
            PROFILER_STOP(__FUNCTION__ ": no current-settings matches");
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

        result.m_matchedStars = finalMatches.size();
        result.m_rmsErrorPixels = std::sqrt(sumSquaredError / finalMatches.size());
        result.m_maxErrorPixels = maxError;

        // Apply the same quality acceptance applied to every other code path. Without this
        // gate, the current-settings path would mark m_solved=true on three random near-
        // coincidences within the (loose) acquisition radius.
        const double currentSettingsMaxRms = std::min(finalMatchRadius * 0.85, 24.0);
        const double currentSettingsMaxMedian = std::min(finalMatchRadius * 0.70, 18.0);
        const double currentSettingsMedian = medianDistancePixels(finalMatches);
        result.m_solved = (finalMatches.size() >= settings.m_plateSolveMinMatches)
            && (result.m_rmsErrorPixels <= currentSettingsMaxRms)
            && (currentSettingsMedian <= currentSettingsMaxMedian);
        if (!result.m_solved) {
            clearSolvedStars(starDetections);
        }
        result.m_azimuthDegrees = settings.m_azimuth;
        result.m_elevationDegrees = settings.m_elevation;
        result.m_rollDegrees = settings.m_roll;
        result.m_fovDegrees = settings.m_fov;
        result.m_centerOffsetXPixels = settings.m_lensCenterOffsetX;
        result.m_centerOffsetYPixels = settings.m_lensCenterOffsetY;
        result.m_distortionK1 = settings.m_lensDistortionK1;

        logUnmatchedDetections(
            settings,
            starDetections,
            projectedStars,
            finalMatches,
            finalMatchRadius);

        PROFILER_STOP(__FUNCTION__ ": current-settings-only");
        return result;
    }

    if ((starDetections.size() < settings.m_plateSolveMinMatches)) {
        return result;
    }

    const QVector<int> detectionIndices = selectDetectionIndicesForSolve(starDetections, imageSize);
    if (detectionIndices.size() < settings.m_plateSolveMinMatches) {
        return result;
    }

    QVector<Evaluation> coarseCandidates;
    FinalMatchPassEvaluation selectedFinalPass;
    Evaluation best = searchBestPose(
        settings,
        catalogContext,
        imageSize,
        captureDateTimeUtc,
        starDetections,
        detectionIndices,
        useMultiHypothesisRefine ? &coarseCandidates : nullptr);
    if (!best.valid || (best.matchCount < settings.m_plateSolveMinMatches)) {
        return result;
    }
    if (useMultiHypothesisRefine) {
        const int weakModeCandidatePoolMinMatches = std::max(3, settings.m_plateSolveMinMatches - 2);
        const int weakModeRefineMinMatches = std::max(3, settings.m_plateSolveMinMatches - 1);
        insertDistinctEvaluationCandidate(
            coarseCandidates,
            best,
            10,
            useWeakModeScoring,
            "coarse-candidate-pool",
            std::max(3, settings.m_plateSolveMinMatches - 1),
            weakModeCandidatePoolMinMatches);
        logWeakModeCandidatePool("coarse-candidate-pool", coarseCandidates);
        QVector<Evaluation> rescoredCandidates;
        rescoredCandidates.reserve(coarseCandidates.size());
        for (const Evaluation& candidate : coarseCandidates)
        {
            const Evaluation rescoredCandidate = rescoreWeakModeCandidateWithDistortionSweep(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                candidate);
            logPlateSolveEvaluation("rescore-distortion-sweep", rescoredCandidate, false, true);
            insertDistinctEvaluationCandidate(
                rescoredCandidates,
                rescoredCandidate,
                10,
                useWeakModeScoring,
                "rescored-candidate-pool",
                std::max(3, settings.m_plateSolveMinMatches - 1),
                weakModeCandidatePoolMinMatches);
        }
        logWeakModeCandidatePool("rescored-candidate-pool", rescoredCandidates);

        Evaluation refinedBest;
        FinalMatchPassEvaluation refinedBestFinalPass;
        for (const Evaluation& candidate : rescoredCandidates)
        {
            if (!candidate.valid || (candidate.matchCount < weakModeRefineMinMatches)) {
                continue;
            }

            Evaluation refinedCandidate = refinePoseFromMatches(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                candidate);
            logPlateSolveEvaluation("refine-from-matches-multi", refinedCandidate, false, true);
            if (!refinedCandidate.valid || (refinedCandidate.matchCount < weakModeRefineMinMatches)) {
                continue;
            }

            const FinalMatchPassEvaluation finalPassEvaluation = evaluateFinalMatchPass(
                settings,
                catalogContext,
                imageSize,
                starDetections,
                allDetectionIndices,
                refinedCandidate,
                finalMatchRadius);
            logFinalMatchPassEvaluation("final-match-pass-multi", finalPassEvaluation);

            if (isBetterWeakModeFinalMatchPass(
                    settings,
                    starDetections,
                    !useStartFov,
                    finalPassEvaluation,
                    refinedBestFinalPass))
            {
                refinedBest = refinedCandidate;
                refinedBestFinalPass = finalPassEvaluation;
                logPlateSolveEvaluation("refine-from-matches-multi", refinedBest, true);
                logFinalMatchPassEvaluation("final-match-pass-multi", refinedBestFinalPass, true);
            }
        }

        if (refinedBest.valid) {
            best = refinedBest;
            selectedFinalPass = refinedBestFinalPass;
        } else {
            best = refinePoseFromMatches(settings, catalogContext, imageSize, captureDateTimeUtc, starDetections, best);
            logPlateSolveEvaluation("refine-from-matches", best, true);
        }
    } else {
        best = refinePoseFromMatches(settings, catalogContext, imageSize, captureDateTimeUtc, starDetections, best);
        logPlateSolveEvaluation("refine-from-matches", best, true);
    }
    if (!best.valid || (!selectedFinalPass.projectorValid && (best.matchCount < settings.m_plateSolveMinMatches))) {
        qDebug().noquote().nospace()
            << "CameraPlateSolver: refine stage rejected candidate"
            << " valid=" << best.valid
            << " matches=" << best.matchCount
            << " minMatches=" << settings.m_plateSolveMinMatches
            << " RMS=" << best.rmsErrorPixels
            << " Az=" << best.azimuthDegrees
            << " El=" << best.elevationDegrees
            << " Roll=" << best.rollDegrees
            << " FoV=" << best.fovDegrees
            << " K1=" << best.distortionK1;
        return result;
    }

    if (!selectedFinalPass.projectorValid) {
        selectedFinalPass = evaluateFinalMatchPass(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            allDetectionIndices,
            best,
            finalMatchRadius);
    }
    if (!selectedFinalPass.projectorValid) {
        return result;
    }

    const QVector<ProjectedCatalogStar>& projectedStars = selectedFinalPass.projectedStars;
    const QVector<Match>& finalMatches = selectedFinalPass.finalMatches;
    result.m_catalogCandidateStars = projectedStars.size();
    result.m_outlierStars = selectedFinalPass.outlierCount;

    if (finalMatches.size() < settings.m_plateSolveMinMatches) {
        qDebug().noquote().nospace()
            << "CameraPlateSolver: final match pass rejected candidate"
            << " finalMatches=" << finalMatches.size()
            << " minMatches=" << settings.m_plateSolveMinMatches
            << " rawMatches=" << selectedFinalPass.rawMatchCount
            << " outliers=" << selectedFinalPass.outlierCount
            << " finalRadius=" << finalMatchRadius
            << " projectedStars=" << projectedStars.size()
            << " Az=" << selectedFinalPass.pose.azimuthDegrees
            << " El=" << selectedFinalPass.pose.elevationDegrees
            << " Roll=" << selectedFinalPass.pose.rollDegrees
            << " FoV=" << selectedFinalPass.pose.fovDegrees
            << " K1=" << selectedFinalPass.pose.distortionK1;
        PROFILER_STOP(__FUNCTION__ ": insufficient matches");
        return result;
    }

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
    }

    result.m_solved = true;
    result.m_matchedStars = finalMatches.size();
    result.m_rmsErrorPixels = selectedFinalPass.rmsErrorPixels;
    result.m_maxErrorPixels = selectedFinalPass.maxErrorPixels;
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

    if (!useStartFov
        && !useStartElevation
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
    result.m_azimuthDegrees = selectedFinalPass.pose.azimuthDegrees;
    result.m_elevationDegrees = selectedFinalPass.pose.elevationDegrees;
    result.m_rollDegrees = selectedFinalPass.pose.rollDegrees;
    result.m_fovDegrees = selectedFinalPass.pose.fovDegrees;
    result.m_centerOffsetXPixels = selectedFinalPass.pose.centerOffsetXPixels;
    result.m_centerOffsetYPixels = selectedFinalPass.pose.centerOffsetYPixels;
    result.m_distortionK1 = selectedFinalPass.pose.distortionK1;

    logUnmatchedDetections(
        settings,
        starDetections,
        projectedStars,
        finalMatches,
        finalMatchRadius);

    PROFILER_STOP(__FUNCTION__);

    return result;
}
