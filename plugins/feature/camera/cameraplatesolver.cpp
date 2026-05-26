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
#include <functional>
#include <limits>

#include <QDir>
#include <QDebug>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QMutex>
#include <QMutexLocker>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPointF>
#include <QRectF>
#include <QSaveFile>
#include <QSet>
#include <QStandardPaths>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QUrl>
#include <QVector>
#include <QtEndian>

#include <zlib.h>

#include "util/astronomy.h"
#include "util/profiler.h"

class CameraPlateSolver::SolverContext
{
public:

// owner may be nullptr when SolverContext is used for static utility functions
// (e.g. downloadedCatalogArchivePath) that don't need network access.
explicit SolverContext(CameraPlateSolver *owner = nullptr) :
    m_owner(owner),
    m_networkManager(owner ? owner->m_networkManager : nullptr)
{
}

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

// Cache entry for the blind-grid roll-sweep optimisation.
// Stores projected pixel offsets from the principal point at roll=0.
// Each of the 13 roll steps is then obtained by a cheap 2D rotation of these
// offsets — no acos/atan2/projection-formula calls are needed per roll.
// This is valid for all projection types and any distortionK1 because radial
// distortion is rotation-invariant: rolling the camera rotates phi but not
// theta, leaving r² (and therefore the distortion scale factor) unchanged.
struct BlindGridCachedStar {
    int   catalogIndex = -1;
    float dxRef = 0.0f;    // pixel_x − principalPointX at roll=0
    float dyRef = 0.0f;    // principalPointY − pixel_y at roll=0 (up-positive)
    float magnitude = 0.0f;
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
    QVector<CatalogStar> catalogStars;
    QString catalogSource;
    QVector<VisibleCatalogStar> visibleStars;
    QHash<int, int> visibleStarIndexByCatalogIndex;
};

struct CandidatePair
{
    int detectionIndex = -1;
    int catalogIndex = -1;
    int projectedIndex = -1;
    double distancePixels = 0.0;
    double brightnessRankError = 0.0;
    int geometricSupport = 0;
};

struct Match
{
    int detectionIndex = -1;
    int catalogIndex = -1;
    double distancePixels = 0.0;
};

struct GuidedAnchorPair
{
    int detectionIndex = -1;
    int catalogIndex = -1;
    double radialErrorPixels = 0.0;
    double initialDistancePixels = 0.0;
    double estimatedRollDegrees = 0.0;
    double magnitude = 0.0;
    double detectionReliability = 0.0;
    double detectionBrightnessRank = 1.0;
};

struct Evaluation
{
    bool valid = false;
    bool anchored = false;
    int anchorDetectionIndex = -1;
    int anchorCatalogIndex = -1;
    int matchCount = 0;
    double rmsErrorPixels = std::numeric_limits<double>::infinity();
    double azimuthDegrees = 0.0;
    double elevationDegrees = 0.0;
    double rollDegrees = 0.0;
    double fovDegrees = 0.0;
    double centerOffsetXPixels = 0.0;
    double centerOffsetYPixels = 0.0;
    double distortionK1 = 0.0;
    double brightnessRankError = std::numeric_limits<double>::infinity();
    double meanCatalogMagnitude = std::numeric_limits<double>::infinity();
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
    int brightProjectedStars = 0;
    int matchedBrightProjectedStars = 0;
    double brightProjectedMatchFraction = 1.0;
    int brightDetections = 0;
    int matchedBrightDetections = 0;
    double brightDetectionMatchFraction = 1.0;
    double brightDetectionMagnitudeError = 0.0;
    double rmsErrorPixels = std::numeric_limits<double>::infinity();
    double medianErrorPixels = std::numeric_limits<double>::infinity();
    double maxErrorPixels = std::numeric_limits<double>::infinity();
    double brightnessRankError = std::numeric_limits<double>::infinity();
    double meanCatalogMagnitude = std::numeric_limits<double>::infinity();
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

static constexpr std::array<std::array<int, 3>, 6> kTrianglePermutations {{
    {{0, 1, 2}},
    {{0, 2, 1}},
    {{1, 0, 2}},
    {{1, 2, 0}},
    {{2, 0, 1}},
    {{2, 1, 0}}
}};

static constexpr std::array<std::array<int, 4>, 8> kQuadPermutations {{
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

static constexpr double kPi = 3.14159265358979323846;
static constexpr double kVisibleAltitudeFloor = -5.0;
static constexpr int kMaxDetectionsForSolve = 64;
static constexpr double kBlindSeedRatioTolerance = 0.035;
static constexpr double kBlindSeedMaxRmsPixels = 18.0;
static constexpr double kBlindSeedMaxMedianPixels = 14.0;
static constexpr bool kLogPlateSolveCandidates = false;
static constexpr bool kLogWeakModeCandidatePools = false;
static constexpr bool kLogWeakModeTailRejects = false;

// Normalisation radius (pixels) used by the weak-mode scoring comparator and coarse
// candidate-pool admission. This is per-solve state, so it lives on the solver context
// instance rather than in thread-local globals.
double m_weakModeNormalizationPixels = 24.0;
bool m_useElevationSeedPreference = false;
double m_elevationSeedReferenceDegrees = 0.0;
double m_elevationSeedReferenceFovDegrees = 0.0;
double m_elevationSeedScaleDegrees = 1.0;
double m_elevationSeedFovScaleDegrees = 1.0;
bool m_useAllSkyZenithPreference = false;
double m_allSkyZenithReferenceElevationDegrees = 90.0;
double m_allSkyZenithScaleDegrees = 20.0;
bool m_useDirectionSeedPreference = false;
bool m_useWideCatalogMagnitudePreference = false;
bool m_directionSeedHasRollPreference = false;
double m_directionSeedReferenceAzimuthDegrees = 0.0;
double m_directionSeedReferenceElevationDegrees = 0.0;
double m_directionSeedReferenceRollDegrees = 0.0;
double m_directionSeedReferenceFovDegrees = 0.0;
double m_directionSeedAzElScaleDegrees = 1.0;
double m_directionSeedRollScaleDegrees = 1.0;
double m_directionSeedFovScaleDegrees = 1.0;
int m_directionSeedMinMatchCount = 4;
bool m_useFovSeedPreference = false;
double m_fovSeedReferenceDegrees = 0.0;
double m_fovSeedScaleDegrees = 1.0;
CameraPlateSolver *m_owner = nullptr;
QNetworkAccessManager *m_networkManager = nullptr;
QHash<QString, QByteArray> m_sirilRangeCache;
QHash<int, QByteArray> m_sirilIndexCache;
QVector<double> m_detectionBrightnessMetricCache;
QVector<double> m_detectionReliabilityMetricCache;
QVector<ProjectedCatalogStar> m_projectedCatalogScratch;
QVector<CandidatePair> m_candidatePairScratch;
QVector<BlindGridCachedStar> m_blindGridCache;
QVector<ProjectedCatalogStar> m_blindGridProjectedScratch;
QHash<quint64, QVector<int>> m_projectedStarGridScratch;
struct ProfileTiming
{
    qint64 totalMs = 0;
    qint64 maxMs = 0;
    int count = 0;
};
QStringList m_profileTimingOrder;
QHash<QString, ProfileTiming> m_profileTimingStats;
QStringList m_profileMetricOrder;
QHash<QString, qint64> m_profileMetrics;
// Maximum total size of m_sirilRangeCache before it is cleared after a solve.
// m_sirilIndexCache is bounded naturally (≤ 48 chunks × 64 KB = 3 MB) and never evicted.
static constexpr qint64 kSirilMaxRangeCacheBytes = 32LL * 1024 * 1024;
static constexpr const char* kSirilCacheDir = "siril-spcc-cache/v1";
static constexpr const char* kSirilRegionCacheDir = "siril-spcc-region-cache/v1";
static constexpr const char* kBundledCatalogPath = ":/camera/brightstarcatalog.txt";
static constexpr const char* kDownloadedCatalogDir = "camera";
static constexpr const char* kDownloadedCatalogArchiveFile = "hyg_v42.csv.gz";
static constexpr const char* kDownloadedCatalogCsvFile = "hyg_v42.csv";
static constexpr const char* kDownloadedCatalogReducedFile = "hyg_v42_reduced.txt";
static constexpr const char* kSirilSpccBaseUrl = "https://huggingface.co/datasets/siril-spcc/gaia/resolve/main";
static constexpr const char* kSirilSpccZenodoBaseUrl = "https://zenodo.org/records/17988559/files";
static constexpr const char* kSirilSpccFileNamePattern = "siril_cat1_healpix8_xpsamp_%1.dat";
static constexpr int kSirilHealpixLevel = 8;
static constexpr int kSirilNside = 1 << kSirilHealpixLevel;
static constexpr int kSirilChunkLevel = 1;
static constexpr int kSirilPixelsPerChunk = 1 << (2 * (kSirilHealpixLevel - kSirilChunkLevel));
static constexpr int kSirilHeaderSize = 128;
static constexpr int kSirilIndexSize = kSirilPixelsPerChunk * static_cast<int>(sizeof(quint32));
static constexpr int kSirilRecordSize = 701;
static constexpr qint64 kSirilMaxRecordsPerCell = 250000;
static constexpr qint64 kSirilMinRangeRequestSize = 64 * 1024;
static constexpr qint64 kSirilMaxMergedRangeRequestSize = 1024 * 1024;
static constexpr qint64 kSirilMaxMergedRangeGapBytes = 64 * 1024;
static constexpr double kSirilAngleScale = 360.0 / 2147483647.0;

void clearProfileTimings()
{
    m_profileTimingOrder.clear();
    m_profileTimingStats.clear();
    m_profileMetricOrder.clear();
    m_profileMetrics.clear();
}

void recordProfileTiming(const QString& stage, qint64 elapsedMs)
{
    if (!m_profileTimingStats.contains(stage)) {
        m_profileTimingOrder.append(stage);
    }

    ProfileTiming& timing = m_profileTimingStats[stage];
    timing.totalMs += elapsedMs;
    timing.maxMs = std::max(timing.maxMs, elapsedMs);
    ++timing.count;
}

void recordProfileMetric(const QString& name, qint64 value)
{
    if (!m_profileMetrics.contains(name)) {
        m_profileMetricOrder.append(name);
    }
    m_profileMetrics[name] += value;
}

QString profileSummary() const
{
    QStringList summary;
    summary.reserve(m_profileTimingOrder.size() + m_profileMetricOrder.size());
    for (const QString& stage : m_profileTimingOrder)
    {
        const ProfileTiming timing = m_profileTimingStats.value(stage);
        if (timing.count <= 1)
        {
            summary.append(QStringLiteral("%1=%2").arg(stage).arg(timing.totalMs));
        }
        else
        {
            summary.append(QStringLiteral("%1=%2(count=%3,avg=%4,max=%5)")
                .arg(stage)
                .arg(timing.totalMs)
                .arg(timing.count)
                .arg(timing.totalMs / timing.count)
                .arg(timing.maxMs));
        }
    }
    for (const QString& metric : m_profileMetricOrder) {
        summary.append(QStringLiteral("%1=%2").arg(metric).arg(m_profileMetrics.value(metric)));
    }
    return summary.join(QStringLiteral(";"));
}
static constexpr double kSirilAutoMaxFovDegrees = 15.0;
static constexpr double kSirilMaxQueryRadiusDegrees = 20.0;
static constexpr double kSirilAliasMaxSeparationArcSec = 30.0;
static constexpr double kSirilAliasMaxMagnitudeDifference = 2.5;
static constexpr double kWideFovMagnitudePreferenceThresholdDegrees = 30.0;
static constexpr double kWideFovBrightFirstPassMaxMagnitude = 5.0;
static constexpr double kWideFovMagnitudePreferenceMinDelta = 1.0;
static constexpr double kWideFovMagnitudePreferenceMaxRmsPixels = 24.0;

struct SirilCellRange
{
    int chunkIndex = 0;
    qint64 firstRecord = 0;
    qint64 recordCount = 0;
    qint64 firstByte = 0;
    qint64 lastByte = 0;
};

struct SirilMergedRange
{
    int chunkIndex = 0;
    qint64 firstByte = 0;
    qint64 lastByte = 0;
    QVector<int> cellIndexes;
};

CameraPlateSolveResult solve(const CameraSettings& settings,
                             const QSize& imageSize,
                             const QDateTime& captureDateTime,
                             QVector<CameraPipelineStarDetection>& starDetections);

static double degToRad(double value)
{
    return value * kPi / 180.0;
}

static double normalizeDegrees(double value)
{
    value = std::fmod(value, 360.0);
    if (value < 0.0) {
        value += 360.0;
    }
    return value;
}

static SkyVector vectorFromAltAz(double azimuthDegrees, double elevationDegrees)
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

static double dot(const SkyVector& lhs, const SkyVector& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

static SkyVector cross(const SkyVector& lhs, const SkyVector& rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x
    };
}

static double length(const SkyVector& vector)
{
    return std::sqrt(dot(vector, vector));
}

static SkyVector normalize(const SkyVector& vector)
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

static SkyVector rotateAroundAxis(const SkyVector& vector, const SkyVector& axis, double angleRadians)
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

static double parseRightAscensionDegrees(const QString& value)
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

static double parseDeclinationDegrees(const QString& value)
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

static QString stripQuotedField(const QString& value)
{
    QString stripped = value.trimmed();
    if (stripped.startsWith('"') && stripped.endsWith('"') && (stripped.size() >= 2)) {
        stripped = stripped.mid(1, stripped.size() - 2);
    }
    return stripped.replace(QStringLiteral("\"\""), QStringLiteral("\""));
}

static QString downloadedCatalogDir()
{
    const QString baseDir = QStandardPaths::standardLocations(QStandardPaths::AppDataLocation).value(0);
    return QDir(baseDir).filePath(QString::fromUtf8(kDownloadedCatalogDir));
}

static QString downloadedCatalogReducedPath()
{
    return QDir(downloadedCatalogDir()).filePath(QString::fromUtf8(kDownloadedCatalogReducedFile));
}

static QByteArray gunzipData(const QByteArray& compressedData, QString* errorMessage)
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

static QVector<CatalogStar> parseBundledCatalog(const QString& text)
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

static QVector<CatalogStar> parseDownloadedHygCatalog(const QString& text)
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

static QString formatRightAscensionHours(double rightAscensionDegrees)
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

static QString formatDeclinationDegrees(double declinationDegrees)
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

static bool writeReducedCatalog(const QVector<CatalogStar>& stars, const QString& path, QString* errorMessage)
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

static QVector<CatalogStar> loadCatalogFromTextFile(const QString& path)
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

static QString currentCatalogPath(const CameraSettings& settings)
{
    if (settings.m_plateSolveUseDownloadedCatalog && QFileInfo::exists(downloadedCatalogReducedPath())) {
        return downloadedCatalogReducedPath();
    }
    return QString::fromUtf8(kBundledCatalogPath);
}

static QString currentCatalogSource(const CameraSettings& settings)
{
    return (settings.m_plateSolveUseDownloadedCatalog && QFileInfo::exists(downloadedCatalogReducedPath()))
        ? QStringLiteral("HYG")
        : QStringLiteral("Bundled");
}

static double firstPassPlateSolveMaxMagnitude(const CameraSettings& settings)
{
    if (plateSolveStartUsesDirection(settings)
        && (settings.m_fov <= 5.0)
        && (settings.m_plateSolveMaxMagnitude > 9.0))
    {
        return 9.0;
    }

    if (isWidePlateSolveContext(settings)
        && (settings.m_plateSolveMaxMagnitude > kWideFovBrightFirstPassMaxMagnitude))
    {
        return kWideFovBrightFirstPassMaxMagnitude;
    }

    return settings.m_plateSolveMaxMagnitude;
}

static QString sirilSpccChunkUrl(int chunkIndex, int sourceIndex)
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

static QString sirilSpccSourceName(int sourceIndex)
{
    return sourceIndex == 1 ? QStringLiteral("Zenodo") : QStringLiteral("Hugging Face");
}

static QString sirilRangeCacheKey(int chunkIndex, qint64 firstByte, qint64 lastByte)
{
    return QStringLiteral("%1:%2:%3").arg(chunkIndex).arg(firstByte).arg(lastByte);
}

static QString sirilCacheRootDir()
{
    return QDir(downloadedCatalogDir()).filePath(QString::fromUtf8(kSirilCacheDir));
}

static QString sirilIndexDiskCachePath(int chunkIndex)
{
    return QDir(QDir(sirilCacheRootDir()).filePath(QStringLiteral("indexes")))
        .filePath(QStringLiteral("chunk-%1.idx").arg(chunkIndex));
}

static QString sirilRangeDiskCachePath(int chunkIndex, qint64 firstByte, qint64 lastByte)
{
    return QDir(QDir(sirilCacheRootDir()).filePath(QStringLiteral("ranges")))
        .filePath(QStringLiteral("chunk-%1-%2-%3.bin").arg(chunkIndex).arg(firstByte).arg(lastByte));
}

static QString sirilRegionCacheRootDir()
{
    return QDir(downloadedCatalogDir()).filePath(QString::fromUtf8(kSirilRegionCacheDir));
}

static QString sirilRegionCacheKey(double centerRaDegrees,
                                   double centerDecDegrees,
                                   double queryRadiusDegrees,
                                   double maxMagnitude)
{
    return QStringLiteral("ra%1_dec%2_r%3_m%4.tsv")
        .arg(qRound64(normalizeDegrees(centerRaDegrees) * 1000.0))
        .arg(qRound64(centerDecDegrees * 1000.0))
        .arg(qRound64(queryRadiusDegrees * 1000.0))
        .arg(qRound64(maxMagnitude * 100.0));
}

static QString sirilRegionDiskCachePath(double centerRaDegrees,
                                        double centerDecDegrees,
                                        double queryRadiusDegrees,
                                        double maxMagnitude)
{
    return QDir(sirilRegionCacheRootDir()).filePath(
        sirilRegionCacheKey(centerRaDegrees, centerDecDegrees, queryRadiusDegrees, maxMagnitude));
}

static QVector<CatalogStar> readSirilRegionDiskCacheFile(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        return {};
    }

    QVector<CatalogStar> stars;
    while (!file.atEnd())
    {
        QByteArray line = file.readLine();
        while (line.endsWith('\n') || line.endsWith('\r')) {
            line.chop(1);
        }
        if (line.isEmpty() || line.startsWith('#')) {
            continue;
        }
        const QList<QByteArray> fields = line.split('\t');
        if (fields.size() < 4) {
            continue;
        }
        bool raOk = false;
        bool decOk = false;
        bool magOk = false;
        const double ra = fields[1].toDouble(&raOk);
        const double dec = fields[2].toDouble(&decOk);
        const double mag = fields[3].toDouble(&magOk);
        if (!raOk || !decOk || !magOk) {
            continue;
        }
        stars.append({
            QString::fromUtf8(fields[0]),
            ra,
            dec,
            mag,
            fields.size() >= 5 ? QString::fromUtf8(fields[4]) : QString()
        });
    }
    return stars;
}

static void writeSirilRegionDiskCacheFile(const QString& path, const QVector<CatalogStar>& stars)
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
    file.write("#name\tra\tdec\tmag\tspectral\n");
    for (const CatalogStar& star : stars)
    {
        QByteArray line;
        line += star.name.toUtf8();
        line += '\t';
        line += QByteArray::number(star.rightAscensionDegrees, 'g', 16);
        line += '\t';
        line += QByteArray::number(star.declinationDegrees, 'g', 16);
        line += '\t';
        line += QByteArray::number(star.magnitude, 'g', 8);
        line += '\t';
        line += star.spectralType.toUtf8();
        line += '\n';
        file.write(line);
    }
    file.commit();
}

static QByteArray readSirilDiskCacheFile(const QString& path, qint64 expectedSize)
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

static void writeSirilDiskCacheFile(const QString& path, const QByteArray& data)
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

static double fovLongEdgeDiagonalDegrees(const QSize& imageSize, double fovDegrees)
{
    const int longEdge = std::max(imageSize.width(), imageSize.height());
    const int shortEdge = std::min(imageSize.width(), imageSize.height());
    if ((longEdge <= 0) || (shortEdge <= 0)) {
        return fovDegrees;
    }

    const double shortToLong = static_cast<double>(shortEdge) / static_cast<double>(longEdge);
    return fovDegrees * std::sqrt(1.0 + shortToLong * shortToLong);
}

static double halfHorizontalFovFromLongEdgeFov(CameraSettings::LensProjection lensProjection,
                                               const QSize& imageSize,
                                               double fovDegrees)
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

QByteArray fetchSirilRangeFromSource(int chunkIndex, qint64 firstByte, qint64 lastByte, int sourceIndex)
{
    if (!m_networkManager)
    {
        qWarning() << "CameraPlateSolver: Siril SPCC range request has no active network manager"
                   << "source" << sirilSpccSourceName(sourceIndex)
                   << "chunk" << chunkIndex << "bytes" << firstByte << lastByte;
        return {};
    }

    // Fast-exit if cancellation was already requested before we even start.
    if (m_owner && m_owner->m_cancelNetworkRequests) {
        return {};
    }

    QNetworkRequest request(QUrl(sirilSpccChunkUrl(chunkIndex, sourceIndex)));
    request.setRawHeader("Range", QByteArray("bytes=%1-%2").replace("%1", QByteArray::number(firstByte)).replace("%2", QByteArray::number(lastByte)));
    request.setRawHeader("Accept-Encoding", "identity");
    request.setRawHeader("User-Agent", "SDRangel CameraPlateSolver/1.0");
    request.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::NoLessSafeRedirectPolicy);
    request.setAttribute(QNetworkRequest::Http2AllowedAttribute, false);

    QNetworkReply *reply = m_networkManager->get(request);

    // Register the active reply so requestNetworkCancellation() can abort it while
    // loop.exec() is running.  Both this function and captureActiveChanged() run on
    // the star-detector thread, so no mutex is needed.
    if (m_owner) {
        m_owner->m_activeNetworkReply = reply;
    }

    QEventLoop loop;
    QTimer timeoutTimer;
    timeoutTimer.setSingleShot(true);
    QObject::connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
    QObject::connect(&timeoutTimer, &QTimer::timeout, &loop, &QEventLoop::quit);
    timeoutTimer.start(30000);
    loop.exec();
    timeoutTimer.stop();  // stop the timer if the reply finished before it fired

    if (m_owner) {
        m_owner->m_activeNetworkReply = nullptr;
    }

    QByteArray data;
    const bool timedOut = !timeoutTimer.isActive() && !reply->isFinished();
    if (timedOut)
    {
        reply->abort();
        qWarning() << "CameraPlateSolver: Siril SPCC range request timed out"
                   << "source" << sirilSpccSourceName(sourceIndex)
                   << "chunk" << chunkIndex << "bytes" << firstByte << lastByte;
    }
    else if (reply->error() != QNetworkReply::NoError)
    {
        // Includes the OperationCanceledError case from requestNetworkCancellation().
        if (m_owner && m_owner->m_cancelNetworkRequests) {
            qDebug() << "CameraPlateSolver: Siril SPCC range request cancelled";
        } else {
            qWarning() << "CameraPlateSolver: Siril SPCC range request failed"
                       << "source" << sirilSpccSourceName(sourceIndex)
                       << "chunk" << chunkIndex << "bytes" << firstByte << lastByte
                       << reply->errorString();
        }
    }
    else
    {
        const int httpStatus = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
        data = reply->readAll();
        // Require HTTP 206 Partial Content. A 200 OK full-file response cannot be
        // safely accepted here because fetchSirilRange() slices the returned bytes
        // using offset = firstByte - requestFirstByte (relative to the start of the
        // requested range), which is only valid when data[0] == byte requestFirstByte
        // of the file.  A 200 response starts at byte 0, so the slice would cut the
        // wrong bytes and cache corrupted catalog data.
        if ((httpStatus != 206) || data.isEmpty())
        {
            qWarning() << "CameraPlateSolver: Siril SPCC range request returned unexpected response"
                       << "source" << sirilSpccSourceName(sourceIndex)
                       << "chunk" << chunkIndex << "status" << httpStatus
                       << "bytes" << firstByte << lastByte << "size" << data.size()
                       << "contentRange" << reply->rawHeader("Content-Range");
            data.clear();
        }
    }
    reply->deleteLater();

    return data;
}

QByteArray fetchSirilRange(int chunkIndex, qint64 firstByte, qint64 lastByte)
{
    if ((chunkIndex < 0) || (firstByte < 0) || (lastByte < firstByte)) {
        return {};
    }

    const QString cacheKey = sirilRangeCacheKey(chunkIndex, firstByte, lastByte);
    const auto cachedRange = m_sirilRangeCache.constFind(cacheKey);
    if (cachedRange != m_sirilRangeCache.constEnd()) {
        return cachedRange.value();
    }
    const qint64 requestedSize = lastByte - firstByte + 1;
    const QByteArray cachedDiskRange = readSirilDiskCacheFile(
        sirilRangeDiskCachePath(chunkIndex, firstByte, lastByte),
        requestedSize);
    if (!cachedDiskRange.isEmpty())
    {
        m_sirilRangeCache.insert(cacheKey, cachedDiskRange);
        return cachedDiskRange;
    }

    const bool smallRange = requestedSize < kSirilMinRangeRequestSize;
    const qint64 requestFirstByte = smallRange
        ? (firstByte / kSirilMinRangeRequestSize) * kSirilMinRangeRequestSize
        : firstByte;
    const qint64 requestLastByte = smallRange
        ? std::max(lastByte, requestFirstByte + kSirilMinRangeRequestSize - 1)
        : lastByte;
    const QString requestCacheKey = sirilRangeCacheKey(chunkIndex, requestFirstByte, requestLastByte);
    if (requestCacheKey != cacheKey)
    {
        const auto it = m_sirilRangeCache.constFind(requestCacheKey);
        if (it != m_sirilRangeCache.constEnd())
        {
            const qint64 offset = firstByte - requestFirstByte;
            if (it.value().size() >= (offset + requestedSize)) {
                return it.value().mid(static_cast<int>(offset), static_cast<int>(requestedSize));
            }
        }
        const qint64 requestSize = requestLastByte - requestFirstByte + 1;
        const QByteArray cachedDiskRequest = readSirilDiskCacheFile(
            sirilRangeDiskCachePath(chunkIndex, requestFirstByte, requestLastByte),
            requestSize);
        if (!cachedDiskRequest.isEmpty())
        {
            const qint64 offset = firstByte - requestFirstByte;
            if (cachedDiskRequest.size() >= (offset + requestedSize))
            {
                const QByteArray requestedData = cachedDiskRequest.mid(static_cast<int>(offset), static_cast<int>(requestedSize));
                m_sirilRangeCache.insert(requestCacheKey, cachedDiskRequest);
                m_sirilRangeCache.insert(cacheKey, requestedData);
                writeSirilDiskCacheFile(sirilRangeDiskCachePath(chunkIndex, firstByte, lastByte), requestedData);
                return requestedData;
            }
        }
    }

    QByteArray data;
    for (int sourceIndex = 0; sourceIndex < 2 && data.isEmpty(); ++sourceIndex) {
        data = fetchSirilRangeFromSource(chunkIndex, requestFirstByte, requestLastByte, sourceIndex);
    }

    if (!data.isEmpty())
    {
        const qint64 offset = firstByte - requestFirstByte;
        if (data.size() < (offset + requestedSize))
        {
            qWarning() << "CameraPlateSolver: Siril SPCC expanded range request was too short"
                       << "chunk" << chunkIndex
                       << "requestedBytes" << firstByte << lastByte
                       << "fetchedBytes" << requestFirstByte << requestLastByte
                       << "size" << data.size();
            return {};
        }

        const QByteArray requestedData = data.mid(static_cast<int>(offset), static_cast<int>(requestedSize));
        m_sirilRangeCache.insert(requestCacheKey, data);
        m_sirilRangeCache.insert(cacheKey, requestedData);
        writeSirilDiskCacheFile(sirilRangeDiskCachePath(chunkIndex, requestFirstByte, requestLastByte), data);
        if (requestCacheKey != cacheKey) {
            writeSirilDiskCacheFile(sirilRangeDiskCachePath(chunkIndex, firstByte, lastByte), requestedData);
        }
        return requestedData;
    }

    return {};
}

QByteArray fetchSirilChunkIndex(int chunkIndex)
{
    const auto cachedIndex = m_sirilIndexCache.constFind(chunkIndex);
    if (cachedIndex != m_sirilIndexCache.constEnd()) {
        return cachedIndex.value();
    }

    const QByteArray cachedDiskIndex = readSirilDiskCacheFile(sirilIndexDiskCachePath(chunkIndex), kSirilIndexSize);
    if (!cachedDiskIndex.isEmpty())
    {
        m_sirilIndexCache.insert(chunkIndex, cachedDiskIndex);
        return cachedDiskIndex;
    }

    const qint64 firstByte = kSirilHeaderSize;
    const qint64 lastByte = kSirilHeaderSize + kSirilIndexSize - 1;
    const QByteArray indexBytes = fetchSirilRange(chunkIndex, firstByte, lastByte);
    if (indexBytes.size() != kSirilIndexSize)
    {
        qWarning() << "CameraPlateSolver: Siril SPCC chunk index request failed"
                   << "chunk" << chunkIndex
                   << "expected" << kSirilIndexSize
                   << "got" << indexBytes.size();
        m_sirilIndexCache.insert(chunkIndex, QByteArray());
        return {};
    }

    m_sirilIndexCache.insert(chunkIndex, indexBytes);
    writeSirilDiskCacheFile(sirilIndexDiskCachePath(chunkIndex), indexBytes);
    return indexBytes;
}

void prefetchSirilMergedRanges(const QVector<SirilMergedRange>& mergedRanges)
{
    if (!m_networkManager || mergedRanges.isEmpty()) {
        return;
    }

    struct PendingRange
    {
        SirilMergedRange range;
        QNetworkReply *reply = nullptr;
        QTimer *timer = nullptr;
    };

    QVector<SirilMergedRange> missingRanges;
    missingRanges.reserve(mergedRanges.size());
    for (const SirilMergedRange& range : mergedRanges)
    {
        const qint64 expectedByteCount = range.lastByte - range.firstByte + 1;
        const QString cacheKey = sirilRangeCacheKey(range.chunkIndex, range.firstByte, range.lastByte);
        if (m_sirilRangeCache.contains(cacheKey)) {
            continue;
        }

        const QByteArray cached = readSirilDiskCacheFile(
            sirilRangeDiskCachePath(range.chunkIndex, range.firstByte, range.lastByte),
            expectedByteCount);
        if (!cached.isEmpty())
        {
            m_sirilRangeCache.insert(cacheKey, cached);
            continue;
        }

        missingRanges.append(range);
    }

    if (missingRanges.isEmpty()) {
        return;
    }

    constexpr int kMaxConcurrentSirilRequests = 6;
    int nextRange = 0;
    int activeCount = 0;
    QEventLoop loop;
    QVector<PendingRange *> pending;

    const auto finishPending = [&](PendingRange *item) {
        if (!item || !item->reply) {
            return;
        }

        const SirilMergedRange range = item->range;
        const qint64 expectedByteCount = range.lastByte - range.firstByte + 1;
        QByteArray data;
        if ((item->reply->error() == QNetworkReply::NoError)
            && (item->reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt() == 206))
        {
            data = item->reply->readAll();
        }

        if (data.size() == expectedByteCount)
        {
            const QString cacheKey = sirilRangeCacheKey(range.chunkIndex, range.firstByte, range.lastByte);
            m_sirilRangeCache.insert(cacheKey, data);
            writeSirilDiskCacheFile(sirilRangeDiskCachePath(range.chunkIndex, range.firstByte, range.lastByte), data);
        }

        item->reply->deleteLater();
        item->reply = nullptr;
        if (item->timer)
        {
            item->timer->deleteLater();
            item->timer = nullptr;
        }
        --activeCount;
        if ((nextRange >= missingRanges.size()) && (activeCount <= 0)) {
            loop.quit();
        }
    };

    std::function<void()> startMore = [&]() {
        while ((activeCount < kMaxConcurrentSirilRequests)
            && (nextRange < missingRanges.size())
            && !(m_owner && m_owner->m_cancelNetworkRequests))
        {
            const SirilMergedRange range = missingRanges[nextRange++];
            QNetworkRequest request(QUrl(sirilSpccChunkUrl(range.chunkIndex, 0)));
            request.setRawHeader("Range", QByteArray("bytes=%1-%2")
                .replace("%1", QByteArray::number(range.firstByte))
                .replace("%2", QByteArray::number(range.lastByte)));
            request.setRawHeader("Accept-Encoding", "identity");
            request.setRawHeader("User-Agent", "SDRangel CameraPlateSolver/1.0");
            request.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::NoLessSafeRedirectPolicy);
            request.setAttribute(QNetworkRequest::Http2AllowedAttribute, false);

            PendingRange *item = new PendingRange{range, m_networkManager->get(request), new QTimer(&loop)};
            item->timer->setSingleShot(true);
            pending.append(item);
            ++activeCount;
            QObject::connect(item->reply, &QNetworkReply::finished, &loop, [&, item]() {
                finishPending(item);
                startMore();
            });
            QObject::connect(item->timer, &QTimer::timeout, &loop, [item]() {
                if (item->reply) {
                    item->reply->abort();
                }
            });
            item->timer->start(30000);
        }

        if ((nextRange >= missingRanges.size()) && (activeCount <= 0)) {
            loop.quit();
        }
    };

    startMore();
    if (activeCount > 0) {
        loop.exec();
    }

    for (PendingRange *item : pending)
    {
        if (item && item->reply)
        {
            item->reply->abort();
            item->reply->deleteLater();
        }
        delete item;
    }
}

static quint32 interleaveHealpixBits(int x, int y)
{
    quint32 pixel = 0;
    for (int bit = 0; bit < kSirilHealpixLevel; ++bit)
    {
        pixel |= ((static_cast<quint32>(x) >> bit) & 1u) << (2 * bit);
        pixel |= ((static_cast<quint32>(y) >> bit) & 1u) << (2 * bit + 1);
    }
    return pixel;
}

static quint32 healpixNestedPixel(double rightAscensionDegrees, double declinationDegrees)
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

static double angularSeparationDegrees(double raDegreesA, double decDegreesA, double raDegreesB, double decDegreesB)
{
    const double raA = degToRad(raDegreesA);
    const double decA = degToRad(decDegreesA);
    const double raB = degToRad(raDegreesB);
    const double decB = degToRad(decDegreesB);
    const double cosSeparation = std::sin(decA) * std::sin(decB)
        + std::cos(decA) * std::cos(decB) * std::cos(raA - raB);
    return std::acos(std::clamp(cosSeparation, -1.0, 1.0)) * 180.0 / kPi;
}

struct SirilQueryGeometry
{
    bool valid = false;
    bool tooWide = false;
    double centerRaDegrees = 0.0;
    double centerDecDegrees = 0.0;
    double queryRadiusDegrees = 0.0;
    QString failureReason;
};

SirilQueryGeometry sirilQueryGeometry(const CameraSettings& settings,
                                      const QSize& imageSize,
                                      const QDateTime& captureDateTimeUtc)
{
    SirilQueryGeometry geometry;
    if (!plateSolveStartUsesDirection(settings))
    {
        geometry.failureReason = QStringLiteral("Siril SPCC Gaia DR3 unavailable for start mode");
        return geometry;
    }

    const RADec centerRaDec = Astronomy::azAltToRaDec(
        AzAlt{settings.m_azimuth, settings.m_elevation},
        settings.m_latitude,
        settings.m_longitude,
        captureDateTimeUtc);
    geometry.centerRaDegrees = normalizeDegrees(centerRaDec.ra * 15.0);
    geometry.centerDecDegrees = centerRaDec.dec;
    if (!std::isfinite(geometry.centerRaDegrees) || !std::isfinite(geometry.centerDecDegrees))
    {
        geometry.failureReason = QStringLiteral("Siril SPCC Gaia DR3 unavailable for invalid pointing");
        return geometry;
    }

    const double diagonalFov = fovLongEdgeDiagonalDegrees(imageSize, settings.m_fov);
    geometry.queryRadiusDegrees = std::max(0.5, diagonalFov * 0.5 + settings.m_plateSolveSearchRadius + 1.0);
    if (geometry.queryRadiusDegrees > kSirilMaxQueryRadiusDegrees)
    {
        geometry.tooWide = true;
        geometry.failureReason = QStringLiteral("Siril SPCC Gaia DR3 unavailable for wide query");
        return geometry;
    }

    geometry.valid = true;
    return geometry;
}

static QSet<quint32> sampleSirilHealpixPixels(double centerRaDegrees,
                                               double centerDecDegrees,
                                               double radiusDegrees)
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

bool sirilCellRecordRange(quint32 pixel, int& chunkIndex, qint64& firstRecord, qint64& recordCount)
{
    chunkIndex = static_cast<int>(pixel / kSirilPixelsPerChunk);
    const int localPixel = static_cast<int>(pixel % kSirilPixelsPerChunk);
    const QByteArray indexBytes = fetchSirilChunkIndex(chunkIndex);
    if (indexBytes.size() != kSirilIndexSize) {
        return false;
    }

    const char *cellEndBytes = indexBytes.constData() + static_cast<qint64>(localPixel) * sizeof(quint32);
    const quint32 cellStart = (localPixel > 0)
        ? qFromLittleEndian<quint32>(reinterpret_cast<const uchar *>(cellEndBytes - sizeof(quint32)))
        : 0;
    const quint32 cellEnd = qFromLittleEndian<quint32>(reinterpret_cast<const uchar *>(cellEndBytes));
    const quint32 chunkRecordCount = qFromLittleEndian<quint32>(
        reinterpret_cast<const uchar *>(indexBytes.constData() + kSirilIndexSize - sizeof(quint32)));
    if ((cellEnd < cellStart) || (cellEnd > chunkRecordCount))
    {
        qWarning() << "CameraPlateSolver: Siril SPCC invalid cell index"
                   << "chunk" << chunkIndex
                   << "localPixel" << localPixel
                   << "cellStart" << cellStart
                   << "cellEnd" << cellEnd
                   << "chunkRecords" << chunkRecordCount;
        return false;
    }

    firstRecord = cellStart;
    recordCount = static_cast<qint64>(cellEnd) - static_cast<qint64>(cellStart);
    if (recordCount > kSirilMaxRecordsPerCell)
    {
        qWarning() << "CameraPlateSolver: Siril SPCC cell has too many records"
                   << "chunk" << chunkIndex
                   << "localPixel" << localPixel
                   << "records" << recordCount
                   << "max" << kSirilMaxRecordsPerCell;
        return false;
    }

    return true;
}

QVector<CatalogStar> loadSirilSpccCatalog(const CameraSettings& settings,
                                          const QSize& imageSize,
                                          const QDateTime& captureDateTimeUtc,
                                          double maxMagnitude,
                                          QString* catalogSource)
{
    QVector<CatalogStar> stars;
    const SirilQueryGeometry geometry = sirilQueryGeometry(settings, imageSize, captureDateTimeUtc);
    if (!geometry.valid)
    {
        if (catalogSource) {
            *catalogSource = geometry.failureReason;
        }
        if (geometry.tooWide)
        {
            qWarning() << "CameraPlateSolver: Siril SPCC Gaia query is too wide, falling back to HYG/bundled"
                       << "radius" << geometry.queryRadiusDegrees
                       << "max" << kSirilMaxQueryRadiusDegrees;
        }
        return stars;
    }
    const double centerRaDegrees = geometry.centerRaDegrees;
    const double centerDecDegrees = geometry.centerDecDegrees;
    const double queryRadius = geometry.queryRadiusDegrees;
    const QString regionCachePath = sirilRegionDiskCachePath(
        centerRaDegrees,
        centerDecDegrees,
        queryRadius,
        maxMagnitude);
    stars = readSirilRegionDiskCacheFile(regionCachePath);
    if (!stars.isEmpty())
    {
        if (catalogSource) {
            *catalogSource = QStringLiteral("Siril SPCC Gaia DR3");
        }
        qDebug() << "CameraPlateSolver: loaded cached Siril SPCC Gaia stars"
                 << stars.size()
                 << "center RA" << centerRaDegrees
                 << "Dec" << centerDecDegrees
                 << "radius" << queryRadius
                 << "maxMag" << maxMagnitude;
        return stars;
    }

    const QSet<quint32> pixels = sampleSirilHealpixPixels(centerRaDegrees, centerDecDegrees, queryRadius);
    QVector<SirilCellRange> cellRanges;
    cellRanges.reserve(pixels.size());
    for (quint32 pixel : pixels)
    {
        int chunkIndex = 0;
        qint64 firstRecord = 0;
        qint64 recordCount = 0;
        if (!sirilCellRecordRange(pixel, chunkIndex, firstRecord, recordCount) || (recordCount <= 0)) {
            continue;
        }

        const qint64 dataStart = kSirilHeaderSize
            + static_cast<qint64>(kSirilPixelsPerChunk) * sizeof(quint32)
            + firstRecord * kSirilRecordSize;
        const qint64 recordByteCount = recordCount * kSirilRecordSize;
        if (recordByteCount > std::numeric_limits<int>::max())
        {
            qWarning() << "CameraPlateSolver: Siril SPCC record range is too large"
                       << "chunk" << chunkIndex
                       << "firstRecord" << firstRecord
                       << "records" << recordCount
                       << "bytes" << recordByteCount;
            continue;
        }

        cellRanges.append({
            chunkIndex,
            firstRecord,
            recordCount,
            dataStart,
            dataStart + recordByteCount - 1
        });
    }

    std::sort(cellRanges.begin(), cellRanges.end(), [](const SirilCellRange& lhs, const SirilCellRange& rhs) {
        if (lhs.chunkIndex != rhs.chunkIndex) {
            return lhs.chunkIndex < rhs.chunkIndex;
        }
        return lhs.firstByte < rhs.firstByte;
    });

    QVector<SirilMergedRange> mergedRanges;
    for (int cellIndex = 0; cellIndex < cellRanges.size(); ++cellIndex)
    {
        const SirilCellRange& cell = cellRanges[cellIndex];
        if (!mergedRanges.isEmpty())
        {
            SirilMergedRange& range = mergedRanges.last();
            const qint64 mergedLastByte = std::max(range.lastByte, cell.lastByte);
            const bool canMerge = (range.chunkIndex == cell.chunkIndex)
                && (cell.firstByte <= (range.lastByte + kSirilMaxMergedRangeGapBytes + 1))
                && ((mergedLastByte - range.firstByte + 1) <= kSirilMaxMergedRangeRequestSize);
            if (canMerge)
            {
                range.lastByte = mergedLastByte;
                range.cellIndexes.append(cellIndex);
                continue;
            }
        }

        mergedRanges.append({
            cell.chunkIndex,
            cell.firstByte,
            cell.lastByte,
            QVector<int>{cellIndex}
        });
    }

    QSet<quint64> seenStars;
    stars.reserve(cellRanges.size() * 8);

    qDebug() << "CameraPlateSolver: Siril SPCC Gaia request"
             << "pixels" << pixels.size()
             << "cells" << cellRanges.size()
             << "ranges" << mergedRanges.size()
             << "center RA" << centerRaDegrees
             << "Dec" << centerDecDegrees
             << "radius" << queryRadius
             << "maxMag" << maxMagnitude;

    prefetchSirilMergedRanges(mergedRanges);

    for (const SirilMergedRange& range : mergedRanges)
    {
        if (m_owner && m_owner->m_cancelNetworkRequests) {
            break;
        }
        const qint64 expectedByteCount = range.lastByte - range.firstByte + 1;
        if (expectedByteCount > std::numeric_limits<int>::max())
        {
            qWarning() << "CameraPlateSolver: Siril SPCC merged record range is too large"
                       << "chunk" << range.chunkIndex
                       << "bytes" << expectedByteCount;
            continue;
        }

        const QByteArray recordBytes = fetchSirilRange(range.chunkIndex, range.firstByte, range.lastByte);
        if (recordBytes.size() != expectedByteCount)
        {
            qWarning() << "CameraPlateSolver: Siril SPCC merged record range request failed"
                       << "chunk" << range.chunkIndex
                       << "cells" << range.cellIndexes.size()
                       << "expectedBytes" << expectedByteCount
                       << "got" << recordBytes.size();
            continue;
        }

        for (int cellIndex : range.cellIndexes)
        {
            const SirilCellRange& cell = cellRanges[cellIndex];
            const qint64 cellOffset = cell.firstByte - range.firstByte;
            for (qint64 recordIndex = 0; recordIndex < cell.recordCount; ++recordIndex)
            {
                const qint64 recordOffset = cellOffset + recordIndex * kSirilRecordSize;
                if ((recordOffset < 0) || ((recordOffset + kSirilRecordSize) > recordBytes.size()))
                {
                    qWarning() << "CameraPlateSolver: Siril SPCC record offset outside merged range"
                               << "chunk" << range.chunkIndex
                               << "firstRecord" << cell.firstRecord
                               << "records" << cell.recordCount
                               << "offset" << recordOffset
                               << "size" << recordBytes.size();
                    break;
                }

                const char *record = recordBytes.constData() + recordOffset;
                const qint32 rawRa = qFromLittleEndian<qint32>(reinterpret_cast<const uchar *>(record));
                const qint32 rawDec = qFromLittleEndian<qint32>(reinterpret_cast<const uchar *>(record + 4));
                const qint16 rawMag = qFromLittleEndian<qint16>(reinterpret_cast<const uchar *>(record + 12));
                const double starRaDegrees = normalizeDegrees(static_cast<double>(rawRa) * kSirilAngleScale);
                const double starDecDegrees = static_cast<double>(rawDec) * kSirilAngleScale;
                const double magnitude = static_cast<double>(rawMag) / 1000.0;
                if (!std::isfinite(starRaDegrees)
                    || !std::isfinite(starDecDegrees)
                    || !std::isfinite(magnitude)
                    || (magnitude > maxMagnitude)
                    || (angularSeparationDegrees(centerRaDegrees, centerDecDegrees, starRaDegrees, starDecDegrees) > queryRadius))
                {
                    continue;
                }

                const quint64 starKey = (static_cast<quint64>(static_cast<quint32>(rawRa)) << 32)
                    | static_cast<quint32>(rawDec);
                if (seenStars.contains(starKey)) {
                    continue;
                }
                seenStars.insert(starKey);
                stars.append({
                    QStringLiteral("Gaia SPCC %1:%2").arg(cell.chunkIndex).arg(cell.firstRecord + recordIndex),
                    starRaDegrees,
                    starDecDegrees,
                    magnitude,
                    QString()
                });
            }
        }
    }

    std::sort(stars.begin(), stars.end(), [](const CatalogStar& lhs, const CatalogStar& rhs) {
        return lhs.magnitude < rhs.magnitude;
    });

    if (catalogSource) {
        *catalogSource = QStringLiteral("Siril SPCC Gaia DR3");
    }
    writeSirilRegionDiskCacheFile(regionCachePath, stars);
    qDebug() << "CameraPlateSolver: loaded Siril SPCC Gaia stars"
             << stars.size()
             << "pixels" << pixels.size()
             << "center RA" << centerRaDegrees
             << "Dec" << centerDecDegrees
             << "radius" << queryRadius
             << "maxMag" << maxMagnitude;
    return stars;
}

static bool plateSolveStartUsesFov(const CameraSettings& settings)
{
    return settings.m_plateSolveStartMode != CameraSettings::PlateSolveStartBlind;
}

static bool plateSolveStartUsesCurrentSettingsOnly(const CameraSettings& settings)
{
    return settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartCurrentSettingsOnly;
}

static bool plateSolveStartUsesElevation(const CameraSettings& settings)
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

static bool plateSolveStartUsesDirection(const CameraSettings& settings)
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

static bool plateSolveStartUsesRoll(const CameraSettings& settings)
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

static bool plateSolveStartUsesLens(const CameraSettings& settings)
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

static bool isWidePlateSolveContext(const CameraSettings& settings)
{
    return (settings.m_lensProjection != CameraSettings::LensProjectionRectilinear)
        && ((settings.m_fov >= kWideFovMagnitudePreferenceThresholdDegrees)
            || (settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartBlind));
}

static bool seedFovCompatibleWithStartFov(const CameraSettings& settings, double seedFovDegrees)
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

static bool canCalibrateLens(const CameraSettings& settings)
{
    return (settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartFovAzElRollLens)
        || isWidePlateSolveContext(settings);
}

static const QVector<CatalogStar>& brightStarCatalog(const CameraSettings& settings)
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

static const QVector<CatalogStar>& bundledAliasCatalog()
{
    static QMutex s_catalogMutex;
    static QVector<CatalogStar> s_catalog;

    QMutexLocker locker(&s_catalogMutex);
    if (s_catalog.isEmpty()) {
        s_catalog = loadCatalogFromTextFile(QString::fromUtf8(kBundledCatalogPath));
    }
    return s_catalog;
}

static QVector<int> aliasCatalogDeclinationSortedIndices(const QVector<CatalogStar>& aliasStars)
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

static QString resolveNamedAliasForCatalogStar(const CatalogStar& star,
                                               const QVector<CatalogStar>& aliasStars,
                                               const QVector<int>& sortedAliasIndices)
{
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

        if (candidate.name.trimmed().isEmpty()) {
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
            bestScore = score;
            bestName = candidate.name.trimmed();
        }
    }

    return bestName;
}

static void applyNamedAliasesToCatalog(const CameraSettings& settings, QVector<CatalogStar>& stars)
{
    Q_UNUSED(settings)

    if (stars.isEmpty()) {
        return;
    }

    const QVector<CatalogStar>& aliasStars = bundledAliasCatalog();
    const QVector<int> sortedAliasIndices = aliasCatalogDeclinationSortedIndices(aliasStars);
    int aliasCount = 0;

    for (CatalogStar& star : stars)
    {
        const QString alias = resolveNamedAliasForCatalogStar(star, aliasStars, sortedAliasIndices);
        if (!alias.isEmpty())
        {
            star.name = alias;
            ++aliasCount;
        }
    }

    if (aliasCount > 0) {
        qDebug() << "CameraPlateSolver: applied bundled catalog aliases" << aliasCount;
    }
}

static QString catalogDisplayName(const CatalogStar& star)
{
    const QVector<CatalogStar>& aliasStars = bundledAliasCatalog();
    const QVector<int> sortedAliasIndices = aliasCatalogDeclinationSortedIndices(aliasStars);
    const QString alias = resolveNamedAliasForCatalogStar(star, aliasStars, sortedAliasIndices);
    return alias.isEmpty() ? star.name : alias;
}

static double catalogAngularSeparationDegrees(const CatalogStar& lhs, const CatalogStar& rhs)
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

static void mergeBundledBrightStarsIntoCatalog(const CameraSettings& settings,
                                               QVector<CatalogStar>& catalogStars,
                                               double maxMagnitude,
                                               double centerRaDegrees,
                                               double centerDecDegrees,
                                               double queryRadiusDegrees)
{
    const QVector<CatalogStar>& brightStars = brightStarCatalog(settings);
    const double mergeMaxMagnitude = std::min(maxMagnitude, 7.0);
    const double duplicateRadiusDegrees = 60.0 / 3600.0;
    const bool filterToQueryRadius = std::isfinite(centerRaDegrees)
        && std::isfinite(centerDecDegrees)
        && std::isfinite(queryRadiusDegrees)
        && (queryRadiusDegrees > 0.0);
    int addedCount = 0;
    int updatedCount = 0;

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
        for (int i = 0; i < catalogStars.size(); ++i)
        {
            if (catalogAngularSeparationDegrees(brightStar, catalogStars[i]) <= duplicateRadiusDegrees)
            {
                duplicateIndex = i;
                break;
            }
        }

        if (duplicateIndex >= 0)
        {
            CatalogStar& existing = catalogStars[duplicateIndex];
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

static QString matchSummary(const PlateSolveCatalogContext& catalogContext,
                            const QVector<CameraPipelineStarDetection>& starDetections,
                            const QVector<Match>& matches)
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

static SkyProjector createProjector(const CameraSettings& settings,
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

static bool projectVector(const SkyProjector& projector, const SkyVector& vector, QPointF& point)
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
        const double distortionScale = 1.0 + projector.distortionK1 * radiusSquared;
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

static bool projectAltAz(const SkyProjector& projector, double azimuthDegrees, double elevationDegrees, QPointF& point)
{
    return projectVector(projector, vectorFromAltAz(azimuthDegrees, elevationDegrees), point);
}

static bool unprojectPixelToVector(const SkyProjector& projector, const QPointF& point, SkyVector& vector)
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
            const double distortionScale = 1.0 + projector.distortionK1 * radiusSquared;
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

static bool rotateBasisToAlignVector(const SkyProjector& projector,
                                     const SkyVector& fromVector,
                                     const SkyVector& toVector,
                                     SkyVector& alignedCenter,
                                     SkyVector& alignedRight,
                                     SkyVector& alignedUp)
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

static bool projectorPoseFromBasis(const SkyVector& center,
                                   const SkyVector& right,
                                   double& azimuthDegrees,
                                   double& elevationDegrees,
                                   double& rollDegrees)
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

static SkyVector subtractScaled(const SkyVector& lhs, const SkyVector& rhs, double scale)
{
    return {
        lhs.x - rhs.x * scale,
        lhs.y - rhs.y * scale,
        lhs.z - rhs.z * scale
    };
}

static SkyVector combineBasis(const SkyVector& xAxis,
                              const SkyVector& yAxis,
                              const SkyVector& zAxis,
                              double x,
                              double y,
                              double z)
{
    return {
        xAxis.x * x + yAxis.x * y + zAxis.x * z,
        xAxis.y * x + yAxis.y * y + zAxis.y * z,
        xAxis.z * x + yAxis.z * y + zAxis.z * z
    };
}

static bool mappedBasisVector(const SkyVector& sourceVector,
                              const SkyVector& sourceXAxis,
                              const SkyVector& sourceYAxis,
                              const SkyVector& sourceZAxis,
                              const SkyVector& targetXAxis,
                              const SkyVector& targetYAxis,
                              const SkyVector& targetZAxis,
                              SkyVector& mappedVector)
{
    const double x = dot(sourceVector, sourceXAxis);
    const double y = dot(sourceVector, sourceYAxis);
    const double z = dot(sourceVector, sourceZAxis);
    mappedVector = normalize(combineBasis(targetXAxis, targetYAxis, targetZAxis, x, y, z));
    return length(mappedVector) > 0.0;
}

static bool poseFromTwoVectorPairs(const SkyProjector& baseProjector,
                                   const SkyVector& sourceA,
                                   const SkyVector& sourceB,
                                   const SkyVector& targetA,
                                   const SkyVector& targetB,
                                   double& azimuthDegrees,
                                   double& elevationDegrees,
                                   double& rollDegrees)
{
    const SkyVector sourceXAxis = normalize(sourceA);
    const SkyVector sourceYAxis = normalize(subtractScaled(sourceB, sourceXAxis, dot(sourceB, sourceXAxis)));
    const SkyVector sourceZAxis = normalize(cross(sourceXAxis, sourceYAxis));
    const SkyVector targetXAxis = normalize(targetA);
    const SkyVector targetYAxis = normalize(subtractScaled(targetB, targetXAxis, dot(targetB, targetXAxis)));
    const SkyVector targetZAxis = normalize(cross(targetXAxis, targetYAxis));
    if ((length(sourceXAxis) <= 0.0)
        || (length(sourceYAxis) <= 0.0)
        || (length(sourceZAxis) <= 0.0)
        || (length(targetXAxis) <= 0.0)
        || (length(targetYAxis) <= 0.0)
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

static bool anchorAlignedPoseFromPixel(const CameraSettings& settings,
                                       const QSize& imageSize,
                                       const QPointF& anchorPoint,
                                       const SkyVector& anchorVector,
                                       double baseAzimuthDegrees,
                                       double baseElevationDegrees,
                                       double baseRollDegrees,
                                       double fovDegrees,
                                       double centerOffsetXPixels,
                                       double centerOffsetYPixels,
                                       double distortionK1,
                                       double& alignedAzimuthDegrees,
                                       double& alignedElevationDegrees,
                                       double& alignedRollDegrees)
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

static QPointF projectorPrincipalPoint(const SkyProjector& projector)
{
    return QPointF(projector.principalPointX, projector.principalPointY);
}

static double pointDistancePixels(const QPointF& lhs, const QPointF& rhs)
{
    return std::hypot(lhs.x() - rhs.x(), lhs.y() - rhs.y());
}

static double detectionBrightnessMetric(const CameraPipelineStarDetection& detection)
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

static bool isDetectionUsableForBrightPrior(const CameraPipelineStarDetection& detection)
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

static double detectionReliabilityMetric(const CameraPipelineStarDetection& detection)
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

void prepareDetectionMetricCache(const QVector<CameraPipelineStarDetection>& starDetections)
{
    m_detectionBrightnessMetricCache.resize(starDetections.size());
    m_detectionReliabilityMetricCache.resize(starDetections.size());
    for (int i = 0; i < starDetections.size(); ++i)
    {
        m_detectionBrightnessMetricCache[i] = detectionBrightnessMetric(starDetections[i]);
        m_detectionReliabilityMetricCache[i] = detectionReliabilityMetric(starDetections[i]);
    }
}

double cachedDetectionBrightnessMetric(const QVector<CameraPipelineStarDetection>& starDetections,
                                       int detectionIndex) const
{
    if ((detectionIndex >= 0)
        && (detectionIndex < m_detectionBrightnessMetricCache.size())
        && std::isfinite(m_detectionBrightnessMetricCache[detectionIndex]))
    {
        return m_detectionBrightnessMetricCache[detectionIndex];
    }
    return ((detectionIndex >= 0) && (detectionIndex < starDetections.size()))
        ? detectionBrightnessMetric(starDetections[detectionIndex])
        : 0.0;
}

double cachedDetectionReliabilityMetric(const QVector<CameraPipelineStarDetection>& starDetections,
                                        int detectionIndex) const
{
    if ((detectionIndex >= 0)
        && (detectionIndex < m_detectionReliabilityMetricCache.size())
        && std::isfinite(m_detectionReliabilityMetricCache[detectionIndex]))
    {
        return m_detectionReliabilityMetricCache[detectionIndex];
    }
    return ((detectionIndex >= 0) && (detectionIndex < starDetections.size()))
        ? detectionReliabilityMetric(starDetections[detectionIndex])
        : 0.0;
}

QVector<int> selectDetectionIndicesForSolve(const QVector<CameraPipelineStarDetection>& starDetections,
                                            const QSize& imageSize)
{
    const int detectionCount = static_cast<int>(starDetections.size());
    QVector<int> byReliability;
    byReliability.reserve(detectionCount);
    for (int i = 0; i < detectionCount; ++i) {
        byReliability.append(i);
    }

    std::sort(byReliability.begin(), byReliability.end(), [this, &starDetections](int lhs, int rhs) {
        if (starDetections[lhs].m_hotPixelSuspect != starDetections[rhs].m_hotPixelSuspect) {
            return !starDetections[lhs].m_hotPixelSuspect;
        }
        const double lhsReliability = cachedDetectionReliabilityMetric(starDetections, lhs);
        const double rhsReliability = cachedDetectionReliabilityMetric(starDetections, rhs);
        if (!qFuzzyCompare(lhsReliability + 1.0, rhsReliability + 1.0)) {
            return lhsReliability > rhsReliability;
        }
        const double lhsBrightness = cachedDetectionBrightnessMetric(starDetections, lhs);
        const double rhsBrightness = cachedDetectionBrightnessMetric(starDetections, rhs);
        if (!qFuzzyCompare(lhsBrightness + 1.0, rhsBrightness + 1.0)) {
            return lhsBrightness > rhsBrightness;
        }
        if (starDetections[lhs].m_peakValue != starDetections[rhs].m_peakValue) {
            return starDetections[lhs].m_peakValue > starDetections[rhs].m_peakValue;
        }
        if (starDetections[lhs].m_saturated != starDetections[rhs].m_saturated) {
            return !starDetections[lhs].m_saturated;
        }
        return starDetections[lhs].m_roundness > starDetections[rhs].m_roundness;
    });

    QVector<int> byBrightness = byReliability;
    std::sort(byBrightness.begin(), byBrightness.end(), [this, &starDetections](int lhs, int rhs) {
        if (starDetections[lhs].m_hotPixelSuspect != starDetections[rhs].m_hotPixelSuspect) {
            return !starDetections[lhs].m_hotPixelSuspect;
        }
        const double lhsBrightness = cachedDetectionBrightnessMetric(starDetections, lhs);
        const double rhsBrightness = cachedDetectionBrightnessMetric(starDetections, rhs);
        if (!qFuzzyCompare(lhsBrightness + 1.0, rhsBrightness + 1.0)) {
            return lhsBrightness > rhsBrightness;
        }
        return cachedDetectionReliabilityMetric(starDetections, lhs)
            > cachedDetectionReliabilityMetric(starDetections, rhs);
    });

    QVector<int> ranked;
    ranked.reserve(std::min(detectionCount, kMaxDetectionsForSolve * 3));
    QSet<int> seen;
    seen.reserve(std::min(detectionCount, kMaxDetectionsForSolve * 3));
    const auto appendUnique = [&ranked, &seen](int index) {
        if (seen.contains(index)) {
            return;
        }
        seen.insert(index);
        ranked.append(index);
    };
    for (int i = 0; (i < detectionCount) && (ranked.size() < kMaxDetectionsForSolve * 3); ++i)
    {
        appendUnique(byReliability[i]);
        appendUnique(byBrightness[i]);
    }
    if (ranked.isEmpty()) {
        return ranked;
    }

    // Prefer stars spread across the image: reject candidates too close to already-selected ones.
    // This improves the geometric diversity of triangle/quad patterns used for blind matching.
    const double minSpreadPixels = std::min(imageSize.width(), imageSize.height()) / (kMaxDetectionsForSolve * 0.75);
    const double minSpreadSquared = minSpreadPixels * minSpreadPixels;
    QVector<int> spread;
    spread.reserve(std::min(static_cast<int>(ranked.size()), kMaxDetectionsForSolve));
    for (int candidate : ranked) {
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
        if (spread.size() >= kMaxDetectionsForSolve) {
            break;
        }
    }
    if (spread.size() >= 4) {
        return spread;
    }

    if (ranked.size() > kMaxDetectionsForSolve) {
        ranked.resize(kMaxDetectionsForSolve);
    }
    return ranked;
}

QVector<int> selectDetectionIndicesForBlindSignatures(const QVector<CameraPipelineStarDetection>& starDetections,
                                                      const QVector<int>& detectionIndices,
                                                      int spreadLimit,
                                                      int brightLimit,
                                                      int totalLimit) const
{
    QVector<int> signatureIndices;
    signatureIndices.reserve(std::min(totalLimit, static_cast<int>(detectionIndices.size())));
    QSet<int> seen;
    seen.reserve(std::min(totalLimit, static_cast<int>(detectionIndices.size())));

    const auto appendUnique = [&signatureIndices, &seen, totalLimit](int index) {
        if ((signatureIndices.size() >= totalLimit) || seen.contains(index)) {
            return;
        }
        seen.insert(index);
        signatureIndices.append(index);
    };

    for (int i = 0; (i < detectionIndices.size()) && (i < spreadLimit); ++i) {
        appendUnique(detectionIndices[i]);
    }

    QVector<int> brightIndices;
    brightIndices.reserve(detectionIndices.size());
    for (int index : detectionIndices)
    {
        if ((index >= 0)
            && (index < starDetections.size())
            && isDetectionUsableForBrightPrior(starDetections[index]))
        {
            brightIndices.append(index);
        }
    }
    std::sort(brightIndices.begin(), brightIndices.end(), [this, &starDetections](int lhs, int rhs) {
        const double lhsBrightness = cachedDetectionBrightnessMetric(starDetections, lhs);
        const double rhsBrightness = cachedDetectionBrightnessMetric(starDetections, rhs);
        if (!qFuzzyCompare(lhsBrightness + 1.0, rhsBrightness + 1.0)) {
            return lhsBrightness > rhsBrightness;
        }
        return cachedDetectionReliabilityMetric(starDetections, lhs)
            > cachedDetectionReliabilityMetric(starDetections, rhs);
    });

    for (int i = 0; (i < brightIndices.size()) && (i < brightLimit); ++i) {
        appendUnique(brightIndices[i]);
    }

    for (int index : detectionIndices) {
        appendUnique(index);
    }

    return signatureIndices;
}

static void buildProjectedCatalogInto(const PlateSolveCatalogContext& catalogContext,
                                      const SkyProjector& projector,
                                      double searchMarginPixels,
                                      const QVector<int>* allowedCatalogIndices,
                                      QVector<ProjectedCatalogStar>& projectedStars)
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
}

static QVector<ProjectedCatalogStar> buildProjectedCatalog(const PlateSolveCatalogContext& catalogContext,
                                                    const SkyProjector& projector,
                                                    double searchMarginPixels,
                                                    const QVector<int>* allowedCatalogIndices = nullptr)
{
    QVector<ProjectedCatalogStar> projectedStars;
    buildProjectedCatalogInto(catalogContext, projector, searchMarginPixels, allowedCatalogIndices, projectedStars);
    return projectedStars;
}

static QVector<VisibleCatalogStar> buildVisibleCatalog(const CameraSettings& settings,
                                                const QVector<CatalogStar>& catalogStars,
                                                const QDateTime& captureDateTimeUtc,
                                                double maxMagnitude)
{
    QVector<VisibleCatalogStar> visibleStars;
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

static void buildVisibleStarIndex(const QVector<VisibleCatalogStar>& visibleStars,
                                  QHash<int, int>& visibleStarIndexByCatalogIndex)
{
    visibleStarIndexByCatalogIndex.clear();
    visibleStarIndexByCatalogIndex.reserve(visibleStars.size());
    for (int i = 0; i < visibleStars.size(); ++i) {
        visibleStarIndexByCatalogIndex.insert(visibleStars[i].catalogIndex, i);
    }
}

struct VisibleCatalogCacheEntry
{
    QVector<VisibleCatalogStar> visibleStars;
    QHash<int, int> visibleStarIndexByCatalogIndex;
};

static QMutex& visibleCatalogCacheMutex()
{
    static QMutex s_cacheMutex;
    return s_cacheMutex;
}

static QHash<QString, VisibleCatalogCacheEntry>& visibleCatalogCache()
{
    static QHash<QString, VisibleCatalogCacheEntry> s_cache;
    return s_cache;
}

static QStringList& visibleCatalogCacheOrder()
{
    static QStringList s_cacheOrder;
    return s_cacheOrder;
}

static QString visibleCatalogCacheKey(const CameraSettings& settings,
                                      const QVector<CatalogStar>& catalogStars,
                                      const QDateTime& captureDateTimeUtc,
                                      double maxMagnitude)
{
    const QString path = currentCatalogPath(settings);
    const bool isResource = path.startsWith(QLatin1String(":/"));
    const qint64 modifiedSecs = isResource ? 0 : QFileInfo(path).lastModified().toSecsSinceEpoch();
    const qint64 timeBucket = captureDateTimeUtc.toSecsSinceEpoch() / 30;

    return QStringLiteral("%1|%2|%3|%4|%5|%6|%7|%8")
        .arg(path)
        .arg(modifiedSecs)
        .arg(catalogStars.size())
        .arg(qRound64(settings.m_latitude * 100000.0))
        .arg(qRound64(settings.m_longitude * 100000.0))
        .arg(timeBucket)
        .arg(qRound64(maxMagnitude * 100.0))
        .arg(static_cast<int>(settings.m_plateSolveCatalogSource));
}

static bool loadVisibleCatalogCache(const CameraSettings& settings,
                                    const QVector<CatalogStar>& catalogStars,
                                    const QDateTime& captureDateTimeUtc,
                                    double maxMagnitude,
                                    QVector<VisibleCatalogStar>& visibleStars,
                                    QHash<int, int>& visibleStarIndexByCatalogIndex)
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

static void storeVisibleCatalogCache(const CameraSettings& settings,
                                     const QVector<CatalogStar>& catalogStars,
                                     const QDateTime& captureDateTimeUtc,
                                     double maxMagnitude,
                                     const QVector<VisibleCatalogStar>& visibleStars,
                                     const QHash<int, int>& visibleStarIndexByCatalogIndex)
{
    static constexpr int kMaxVisibleCatalogCacheEntries = 32;

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

static void populateVisibleCatalogContext(PlateSolveCatalogContext& context,
                                          const CameraSettings& settings,
                                          const QDateTime& captureDateTimeUtc,
                                          double maxMagnitude,
                                          bool allowCache)
{
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

PlateSolveCatalogContext buildPlateSolveCatalogContext(const CameraSettings& settings,
                                                       const QSize& imageSize,
                                                       const QDateTime& captureDateTimeUtc,
                                                       double maxMagnitude,
                                                       double catalogLoadMaxMagnitude = -1.0)
{
    PlateSolveCatalogContext context;
    const double effectiveCatalogLoadMaxMagnitude = (catalogLoadMaxMagnitude > 0.0)
        ? std::max(maxMagnitude, catalogLoadMaxMagnitude)
        : maxMagnitude;
    const bool requestSiril = (settings.m_plateSolveCatalogSource == CameraSettings::PlateSolveCatalogSirilSpccGaia)
        || ((settings.m_plateSolveCatalogSource == CameraSettings::PlateSolveCatalogAuto)
            && plateSolveStartUsesDirection(settings)
            && (settings.m_fov <= kSirilAutoMaxFovDegrees));
    if (requestSiril)
    {
        context.catalogStars = loadSirilSpccCatalog(
            settings,
            imageSize,
            captureDateTimeUtc,
            effectiveCatalogLoadMaxMagnitude,
            &context.catalogSource);
        if (!context.catalogStars.isEmpty())
        {
            const SirilQueryGeometry geometry = sirilQueryGeometry(settings, imageSize, captureDateTimeUtc);
            applyNamedAliasesToCatalog(settings, context.catalogStars);
            mergeBundledBrightStarsIntoCatalog(
                settings,
                context.catalogStars,
                effectiveCatalogLoadMaxMagnitude,
                geometry.valid ? geometry.centerRaDegrees : std::numeric_limits<double>::quiet_NaN(),
                geometry.valid ? geometry.centerDecDegrees : std::numeric_limits<double>::quiet_NaN(),
                geometry.valid ? geometry.queryRadiusDegrees : -1.0);
        }
        if (context.catalogStars.size() < settings.m_plateSolveMinMatches)
        {
            qWarning() << "CameraPlateSolver: Siril SPCC Gaia catalog did not provide enough stars, falling back to HYG/bundled"
                       << "stars" << context.catalogStars.size()
                       << "minMatches" << settings.m_plateSolveMinMatches;
            context.catalogStars.clear();
            context.catalogSource.clear();
        }
    }

    if (context.catalogStars.isEmpty())
    {
        context.catalogStars = brightStarCatalog(settings);
        context.catalogSource = currentCatalogSource(settings);
    }

    const bool allowVisibleCache = !context.catalogSource.contains(QStringLiteral("Siril SPCC Gaia"));
    populateVisibleCatalogContext(context, settings, captureDateTimeUtc, maxMagnitude, allowVisibleCache);
    return context;
}

void rebuildVisibleCatalogContext(PlateSolveCatalogContext& context,
                                  const CameraSettings& settings,
                                  const QDateTime& captureDateTimeUtc,
                                  double maxMagnitude)
{
    const bool allowVisibleCache = !context.catalogSource.contains(QStringLiteral("Siril SPCC Gaia"));
    populateVisibleCatalogContext(context, settings, captureDateTimeUtc, maxMagnitude, allowVisibleCache);
}

static QVector<VisibleCatalogStar> selectLocalVisibleStars(const QVector<VisibleCatalogStar>& visibleStars,
                                                           double centerAzimuthDegrees,
                                                           double centerElevationDegrees,
                                                           double radiusDegrees,
                                                           int maxStars)
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

static QVector<VisibleCatalogStar> selectVisibleStarsNearElevation(const QVector<VisibleCatalogStar>& visibleStars,
                                                                   double centerElevationDegrees,
                                                                   double radiusDegrees,
                                                                   int maxStars)
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

QVector<GuidedAnchorPair> findGuidedAnchorPairs(const CameraSettings& settings,
                                                const PlateSolveCatalogContext& catalogContext,
                                                const QSize& imageSize,
                                                const QVector<CameraPipelineStarDetection>& starDetections,
                                                const QVector<VisibleCatalogStar>& localVisibleStars)
{
    QVector<GuidedAnchorPair> anchors;
    const bool useWideWeakAnchorSearch = !plateSolveStartUsesDirection(settings)
        && isWidePlateSolveContext(settings);
    if ((!plateSolveStartUsesDirection(settings) && !useWideWeakAnchorSearch)
        || starDetections.isEmpty()
        || localVisibleStars.isEmpty())
    {
        return anchors;
    }

    const bool useStartLens = plateSolveStartUsesLens(settings);
    const SkyProjector currentProjector = createProjector(
        settings,
        imageSize,
        settings.m_azimuth,
        settings.m_elevation,
        settings.m_roll,
        settings.m_fov,
        useStartLens ? settings.m_lensCenterOffsetX : 0.0,
        useStartLens ? settings.m_lensCenterOffsetY : 0.0,
        useStartLens ? settings.m_lensDistortionK1 : 0.0);
    if (!currentProjector.valid) {
        return anchors;
    }

    const QPointF center = projectorPrincipalPoint(currentProjector);
    const double maxImageDimension = std::max(imageSize.width(), imageSize.height());
    const double radialTolerancePixels = useWideWeakAnchorSearch
        ? maxImageDimension
        : std::max(
            static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 6.0,
            maxImageDimension * 0.08);
    const double directDistanceLimitPixels = useWideWeakAnchorSearch
        ? maxImageDimension * 1.5
        : std::max(
            radialTolerancePixels * 3.0,
            maxImageDimension * 0.35);
    const double anchorMaxMagnitude = std::min(settings.m_plateSolveMaxMagnitude, 7.0);
    QVector<int> allDetectionIndices;
    allDetectionIndices.reserve(starDetections.size());
    for (int i = 0; i < starDetections.size(); ++i) {
        allDetectionIndices.append(i);
    }
    const QHash<int, double> detectionRanks = detectionBrightnessRanks(starDetections, allDetectionIndices);

    for (const VisibleCatalogStar& visibleStar : localVisibleStars)
    {
        if ((visibleStar.catalogIndex < 0)
            || (visibleStar.catalogIndex >= catalogContext.catalogStars.size())
            || (visibleStar.magnitude > anchorMaxMagnitude))
        {
            continue;
        }

        QPointF projectedPoint;
        if (!projectVector(currentProjector, visibleStar.vector, projectedPoint)) {
            continue;
        }

        const double projectedRadius = pointDistancePixels(projectedPoint, center);
        QVector<GuidedAnchorPair> starAnchors;
        starAnchors.reserve(3);
        for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
        {
            const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
            if (detection.m_hotPixelSuspect) {
                continue;
            }

            const double reliability = cachedDetectionReliabilityMetric(starDetections, detectionIndex);
            if (reliability <= 0.0) {
                continue;
            }

            const double detectionBrightnessRank = detectionRanks.value(detectionIndex, 1.0);
            const bool brightCatalogAnchor = visibleStar.magnitude <= 5.0;
            const bool brightDetectionAnchor = brightCatalogAnchor
                && (detectionBrightnessRank <= 0.25)
                && !detection.m_hotPixelSuspect;
            const double detectionRadius = pointDistancePixels(detection.m_center, center);
            const double radialError = std::fabs(detectionRadius - projectedRadius);
            const double directDistance = pointDistancePixels(detection.m_center, projectedPoint);
            if (!brightDetectionAnchor
                && (radialError > radialTolerancePixels)
                && (directDistance > radialTolerancePixels))
            {
                continue;
            }
            if (directDistance > directDistanceLimitPixels) {
                continue;
            }

            const QLineF projectedLine(center, projectedPoint);
            const QLineF detectionLine(center, detection.m_center);
            double estimatedRoll = settings.m_roll + projectedLine.angleTo(detectionLine);
            if (!std::isfinite(estimatedRoll)) {
                estimatedRoll = settings.m_roll;
            }

            starAnchors.append({
                detectionIndex,
                visibleStar.catalogIndex,
                radialError,
                directDistance,
                normalizeDegrees(estimatedRoll),
                visibleStar.magnitude,
                reliability,
                detectionBrightnessRank
            });
        }

        std::sort(starAnchors.begin(), starAnchors.end(), [](const GuidedAnchorPair& lhs, const GuidedAnchorPair& rhs) {
            const bool lhsBright = lhs.magnitude <= 5.0;
            const bool rhsBright = rhs.magnitude <= 5.0;
            const double lhsScore = lhsBright
                ? lhs.initialDistancePixels * 0.15
                    + lhs.radialErrorPixels * 0.05
                    + lhs.detectionBrightnessRank * 500.0
                    - std::min(80.0, std::log1p(lhs.detectionReliability) * 8.0)
                : lhs.initialDistancePixels
                    + lhs.radialErrorPixels * 0.25
                    - std::min(80.0, std::log1p(lhs.detectionReliability) * 8.0);
            const double rhsScore = rhsBright
                ? rhs.initialDistancePixels * 0.15
                    + rhs.radialErrorPixels * 0.05
                    + rhs.detectionBrightnessRank * 500.0
                    - std::min(80.0, std::log1p(rhs.detectionReliability) * 8.0)
                : rhs.initialDistancePixels
                    + rhs.radialErrorPixels * 0.25
                    - std::min(80.0, std::log1p(rhs.detectionReliability) * 8.0);
            if (!qFuzzyCompare(lhsScore + 1.0, rhsScore + 1.0)) {
                return lhsScore < rhsScore;
            }
            return lhs.magnitude < rhs.magnitude;
        });
        const int maxAnchorsForStar = visibleStar.magnitude <= 5.0 ? 6 : 3;
        while (starAnchors.size() > maxAnchorsForStar) {
            starAnchors.removeLast();
        }
        anchors += starAnchors;
    }

    std::sort(anchors.begin(), anchors.end(), [](const GuidedAnchorPair& lhs, const GuidedAnchorPair& rhs) {
        const bool lhsBright = lhs.magnitude <= 5.0;
        const bool rhsBright = rhs.magnitude <= 5.0;
        const double lhsScore = lhs.magnitude * 45.0
            + (lhsBright ? lhs.initialDistancePixels * 0.15 : lhs.initialDistancePixels)
            + (lhsBright ? lhs.radialErrorPixels * 0.05 : lhs.radialErrorPixels * 0.25)
            + (lhsBright ? lhs.detectionBrightnessRank * 500.0 : 0.0)
            - std::min(100.0, std::log1p(lhs.detectionReliability) * 10.0);
        const double rhsScore = rhs.magnitude * 45.0
            + (rhsBright ? rhs.initialDistancePixels * 0.15 : rhs.initialDistancePixels)
            + (rhsBright ? rhs.radialErrorPixels * 0.05 : rhs.radialErrorPixels * 0.25)
            + (rhsBright ? rhs.detectionBrightnessRank * 500.0 : 0.0)
            - std::min(100.0, std::log1p(rhs.detectionReliability) * 10.0);
        if (!qFuzzyCompare(lhsScore + 1.0, rhsScore + 1.0)) {
            return lhsScore < rhsScore;
        }
        if (lhs.catalogIndex != rhs.catalogIndex) {
            return lhs.catalogIndex < rhs.catalogIndex;
        }
        return lhs.detectionIndex < rhs.detectionIndex;
    });
    const int anchorPoolLimit = useWideWeakAnchorSearch ? 96 : 24;
    while (anchors.size() > anchorPoolLimit) {
        anchors.removeLast();
    }

    return anchors;
}

static TriangleSignature buildTriangleSignature(const std::array<QPointF, 3>& points)
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

static std::array<QPointF, 4> orderQuadPoints(const std::array<QPointF, 4>& points)
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

static QuadSignature buildQuadSignature(const std::array<QPointF, 4>& unorderedPoints)
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

static qint64 signatureBucketKey(int firstBin, int secondBin)
{
    return (static_cast<qint64>(firstBin) << 32)
        ^ static_cast<quint32>(secondBin);
}

static int signatureRatioBin(double value, double binWidth)
{
    return static_cast<int>(std::floor(value / std::max(1e-6, binWidth)));
}

static QHash<qint64, QVector<int>> buildTriangleSignatureBuckets(const QVector<TriangleSignature>& signatures,
                                                                 double binWidth)
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

static QHash<qint64, QVector<int>> buildQuadSignatureBuckets(const QVector<QuadSignature>& signatures,
                                                             double binWidth)
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

static double medianCandidateDistancePixels(const QVector<Match>& matches)
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

    const int minBlindSeedMatches = (plateSolveStartUsesDirection(settings) && (settings.m_fov <= 5.0))
        ? std::max(settings.m_plateSolveMinMatches, 4)
        : std::max(settings.m_plateSolveMinMatches + 1,
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

template<size_t N>
bool hasSeedAnchorSupport(const Evaluation& candidate,
                          const std::array<int, N>& detectionIndices,
                          const std::array<int, N>& catalogIndices,
                          int requiredMatches)
{
    if (!candidate.valid || (requiredMatches <= 0)) {
        return false;
    }

    int matchedAnchors = 0;
    for (size_t anchorIndex = 0; anchorIndex < N; ++anchorIndex)
    {
        for (const Match& match : candidate.matches)
        {
            if ((match.detectionIndex == detectionIndices[anchorIndex])
                && (match.catalogIndex == catalogIndices[anchorIndex]))
            {
                ++matchedAnchors;
                break;
            }
        }
    }

    return matchedAnchors >= std::min<int>(requiredMatches, static_cast<int>(N));
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

    const int minConsensusMatches = (plateSolveStartUsesDirection(settings) && (settings.m_fov <= 5.0))
        ? std::max(settings.m_plateSolveMinMatches, 4)
        : std::max(settings.m_plateSolveMinMatches + 1, std::min(6, static_cast<int>(detectionIndices.size())));
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
                                                            const QVector<int>& detectionIndices,
                                                            int maxDetectionCount = 16)
{
    QVector<TriangleSignature> signatures;
    const int maxDetections = std::min<int>(maxDetectionCount, static_cast<int>(detectionIndices.size()));
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
                                                    const QVector<int>& detectionIndices,
                                                    int maxDetectionCount = 14)
{
    QVector<QuadSignature> signatures;
    const int maxDetections = std::min<int>(maxDetectionCount, static_cast<int>(detectionIndices.size()));
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

    const bool isWideFisheyeLens = isWidePlateSolveContext(settings);
    const QVector<int> signatureDetectionIndices = isWideFisheyeLens
        ? selectDetectionIndicesForBlindSignatures(starDetections, detectionIndices, 12, 12, 20)
        : detectionIndices;
    const QVector<TriangleSignature> detectionTriangles = buildDetectionTriangleSignatures(
        starDetections,
        signatureDetectionIndices,
        isWideFisheyeLens ? 20 : 16);
    const QVector<TriangleSignature> catalogTriangles = buildCatalogTriangleSignatures(settings, visibleStars);
    if (detectionTriangles.isEmpty() || catalogTriangles.isEmpty()) {
        return seeds;
    }
    const double ratioTolerance = (plateSolveStartUsesDirection(settings) && (settings.m_fov <= 5.0))
        ? 0.08
        : isWideFisheyeLens ? 0.08
        : kBlindSeedRatioTolerance;
    const bool ignoreOrientationHandedness = isWideFisheyeLens
        || (plateSolveStartUsesDirection(settings) && (settings.m_fov <= 5.0));
    const QHash<qint64, QVector<int>> catalogTriangleBuckets =
        buildTriangleSignatureBuckets(catalogTriangles, ratioTolerance);
    const int bucketRadius = 1;
    recordProfileMetric(QStringLiteral("search.triangleDetectionSignatures"), detectionTriangles.size());
    recordProfileMetric(QStringLiteral("search.triangleCatalogSignatures"), catalogTriangles.size());

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
    qint64 ratioCandidates = 0;
    qint64 ratioMatches = 0;
    qint64 seedEvaluations = 0;
    qint64 verifiedSeeds = 0;

    for (const TriangleSignature& detectionTriangle : detectionTriangles)
    {
        if (earlyExit) break;
        const int detectionShortBin = signatureRatioBin(detectionTriangle.ratioShortToLong, ratioTolerance);
        const int detectionMidBin = signatureRatioBin(detectionTriangle.ratioMidToLong, ratioTolerance);
        for (int shortBinOffset = -bucketRadius; shortBinOffset <= bucketRadius; ++shortBinOffset)
        {
            if (earlyExit) break;
            for (int midBinOffset = -bucketRadius; midBinOffset <= bucketRadius; ++midBinOffset)
            {
                if (earlyExit) break;
                const auto bucketIt = catalogTriangleBuckets.constFind(
                    signatureBucketKey(detectionShortBin + shortBinOffset, detectionMidBin + midBinOffset));
                if (bucketIt == catalogTriangleBuckets.constEnd()) {
                    continue;
                }
                for (int catalogTriangleIndex : *bucketIt)
                {
                    if (earlyExit) break;
                    const TriangleSignature& catalogTriangle = catalogTriangles.at(catalogTriangleIndex);
                    ++ratioCandidates;
                    if (std::fabs(detectionTriangle.ratioShortToLong - catalogTriangle.ratioShortToLong) > ratioTolerance
                        || std::fabs(detectionTriangle.ratioMidToLong - catalogTriangle.ratioMidToLong) > ratioTolerance)
                    {
                        continue;
                    }

                    ++ratioMatches;
                    if (!ignoreOrientationHandedness && ((detectionTriangle.orientation * catalogTriangle.orientation) < 0.0)) {
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
                catalogAngularDistance * static_cast<double>(std::max(imageSize.width(), imageSize.height())) / std::max(1.0, detectionTriangle.longestDistance),
                static_cast<double>(CameraSettings::m_minFov),
                static_cast<double>(CameraSettings::m_maxFov));
            if (!seedFovCompatibleWithStartFov(settings, baseSeedFov)) {
                continue;
            }

            // Skip sky directions already tried by a previous triangle match. The dedup
            // radius scales with the seed FoV: at 90° FoV the original 3° tolerance is fine,
            // but at 15-25° wide-field blind FoVs a 3° basin can swallow the correct
            // direction after a near-miss. Use 5% of seed FoV with a 0.5°-5° clamp.
            const double dedupRadiusDegrees = std::clamp(baseSeedFov * 0.05, 0.5, 5.0);
            bool alreadyTried = false;
            for (const TriedDirection& tried : triedDirections) {
                // Use wrap-aware azimuth distance so seeds near 0°/360° are correctly deduped.
                const double azDiff = std::fabs(seedAzimuth - tried.azimuthDegrees);
                const double azDist = (azDiff <= 180.0) ? azDiff : 360.0 - azDiff;
                if (azDist < dedupRadiusDegrees
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
            // rectilinear scale model (long_edge_pixels / longest_pixel_distance), which biases
            // the result low for fisheye lenses; widen the sweep accordingly. Also clamp to
            // CameraSettings::m_maxFov for consistency with the base-clamp above — the
            // earlier hard-coded 180.0 ceiling was inconsistent with the rest of the file.
            const bool isFisheyeLens = (settings.m_lensProjection != CameraSettings::LensProjectionRectilinear);
            const std::array<double, 5> fovScales = plateSolveStartUsesFov(settings)
                ? std::array<double, 5>{{0.96, 1.0, 1.04, 1.0, 1.0}}
                : isFisheyeLens
                    ? std::array<double, 5>{{0.60, 0.80, 1.0, 1.25, 1.60}}
                    : std::array<double, 5>{{0.85, 0.93, 1.0, 1.07, 1.15}};
            const int fovScaleCount = plateSolveStartUsesFov(settings) ? 3 : 5;
            const double seedBaseFov = plateSolveStartUsesFov(settings)
                ? static_cast<double>(settings.m_fov)
                : baseSeedFov;
            for (int fovScaleIndex = 0; fovScaleIndex < fovScaleCount; ++fovScaleIndex)
            {
                if (earlyExit) break;
                const double fovScale = fovScales[fovScaleIndex];
                const double seedFov = std::clamp(
                    seedBaseFov * fovScale,
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

                        const Evaluation seededCandidate = evaluatePose(
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
                        ++seedEvaluations;
                        if (!seededCandidate.valid) {
                            continue;
                        }
                        const std::array<int, 3> anchorCatalogIndices {{
                            triangleStars[permutation[0]].catalogIndex,
                            triangleStars[permutation[1]].catalogIndex,
                            triangleStars[permutation[2]].catalogIndex
                        }};
                        if (!hasSeedAnchorSupport(seededCandidate, detectionTriangle.indices, anchorCatalogIndices, 2)) {
                            continue;
                        }

                        const Evaluation candidate = evaluatePose(
                            settings,
                            catalogContext,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            detectionIndices,
                            seededCandidate.azimuthDegrees,
                            seededCandidate.elevationDegrees,
                            seededCandidate.rollDegrees,
                            seededCandidate.fovDegrees,
                            nullptr,
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
                            ++verifiedSeeds;
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

        }
    }

    recordProfileMetric(QStringLiteral("search.triangleRatioCandidates"), ratioCandidates);
    recordProfileMetric(QStringLiteral("search.triangleRatioMatches"), ratioMatches);
    recordProfileMetric(QStringLiteral("search.triangleSeedEvaluations"), seedEvaluations);
    recordProfileMetric(QStringLiteral("search.triangleVerifiedSeeds"), verifiedSeeds);

    std::sort(seeds.begin(), seeds.end(), [this](const Evaluation& lhs, const Evaluation& rhs) {
        return isBetterWeakModeEvaluation(lhs, rhs);
    });
    const int seedLimit = isWideFisheyeLens ? 64 : 16;
    if (seeds.size() > seedLimit) {
        seeds.resize(seedLimit);
    }

    return seeds;
}

QVector<Evaluation> buildBrightPairSeeds(const CameraSettings& settings,
                                         const PlateSolveCatalogContext& catalogContext,
                                         const QSize& imageSize,
                                         const QDateTime& captureDateTimeUtc,
                                         const QVector<CameraPipelineStarDetection>& starDetections,
                                         const QVector<int>& detectionIndices,
                                         const QVector<VisibleCatalogStar>& visibleStars)
{
    QVector<Evaluation> seeds;
    if (!isWidePlateSolveContext(settings)
        || (starDetections.size() < settings.m_plateSolveMinMatches)
        || (visibleStars.size() < settings.m_plateSolveMinMatches))
    {
        return seeds;
    }

    QVector<int> brightDetectionIndices = selectDetectionIndicesForBlindSignatures(
        starDetections,
        detectionIndices,
        10,
        10,
        16);
    if (brightDetectionIndices.size() < 2) {
        return seeds;
    }
    if (brightDetectionIndices.size() > 10) {
        brightDetectionIndices.resize(10);
    }

    QVector<VisibleCatalogStar> brightCatalogStars;
    brightCatalogStars.reserve(20);
    for (const VisibleCatalogStar& visibleStar : visibleStars)
    {
        if (visibleStar.magnitude > kWideFovBrightFirstPassMaxMagnitude) {
            break;
        }
        brightCatalogStars.append(visibleStar);
        if (brightCatalogStars.size() >= 20) {
            break;
        }
    }
    if (brightCatalogStars.size() < 2) {
        return seeds;
    }

    QVector<double> seedFovs;
    if (plateSolveStartUsesFov(settings))
    {
        for (double scale : {0.96, 1.0, 1.04})
        {
            seedFovs.append(std::clamp(
                static_cast<double>(settings.m_fov) * scale,
                static_cast<double>(CameraSettings::m_minFov),
                static_cast<double>(CameraSettings::m_maxFov)));
        }
    }
    else
    {
        for (double fovDegrees : {130.0, 160.0, 180.0}) {
            seedFovs.append(fovDegrees);
        }
    }

    const bool useStartLens = plateSolveStartUsesLens(settings);
    const double fixedCenterOffsetX = useStartLens ? settings.m_lensCenterOffsetX : 0.0;
    const double fixedCenterOffsetY = useStartLens ? settings.m_lensCenterOffsetY : 0.0;
    const double fixedDistortionK1 = useStartLens ? settings.m_lensDistortionK1 : 0.0;
    qint64 seedEvaluations = 0;
    qint64 verifiedSeeds = 0;

    for (double seedFov : seedFovs)
    {
        const SkyProjector baseProjector = createProjector(
            settings,
            imageSize,
            0.0,
            90.0,
            0.0,
            seedFov,
            fixedCenterOffsetX,
            fixedCenterOffsetY,
            fixedDistortionK1);
        if (!baseProjector.valid) {
            continue;
        }

        for (int firstDetection = 0; firstDetection < brightDetectionIndices.size(); ++firstDetection)
        {
            for (int secondDetection = firstDetection + 1; secondDetection < brightDetectionIndices.size(); ++secondDetection)
            {
                const int detectionIndexA = brightDetectionIndices[firstDetection];
                const int detectionIndexB = brightDetectionIndices[secondDetection];
                SkyVector sourceA;
                SkyVector sourceB;
                if (!unprojectPixelToVector(baseProjector, starDetections[detectionIndexA].m_center, sourceA)
                    || !unprojectPixelToVector(baseProjector, starDetections[detectionIndexB].m_center, sourceB))
                {
                    continue;
                }
                const double sourceSeparationRadians = std::acos(std::clamp(dot(sourceA, sourceB), -1.0, 1.0));
                if (sourceSeparationRadians < degToRad(1.0)) {
                    continue;
                }

                for (int firstCatalog = 0; firstCatalog < brightCatalogStars.size(); ++firstCatalog)
                {
                    for (int secondCatalog = 0; secondCatalog < brightCatalogStars.size(); ++secondCatalog)
                    {
                        if (firstCatalog == secondCatalog) {
                            continue;
                        }
                        const double catalogSeparationRadians = std::acos(std::clamp(
                            dot(brightCatalogStars[firstCatalog].vector, brightCatalogStars[secondCatalog].vector),
                            -1.0,
                            1.0));
                        const double separationToleranceRadians = plateSolveStartUsesFov(settings)
                            ? std::max(degToRad(2.0), sourceSeparationRadians * 0.18)
                            : std::max(degToRad(5.0), sourceSeparationRadians * 0.30);
                        if (std::fabs(sourceSeparationRadians - catalogSeparationRadians) > separationToleranceRadians) {
                            continue;
                        }

                        double seedAzimuth = 0.0;
                        double seedElevation = 0.0;
                        double seedRoll = 0.0;
                        if (!poseFromTwoVectorPairs(
                                baseProjector,
                                sourceA,
                                sourceB,
                                brightCatalogStars[firstCatalog].vector,
                                brightCatalogStars[secondCatalog].vector,
                                seedAzimuth,
                                seedElevation,
                                seedRoll))
                        {
                            continue;
                        }

                        QVector<int> allowedCatalogIndices {
                            brightCatalogStars[firstCatalog].catalogIndex,
                            brightCatalogStars[secondCatalog].catalogIndex
                        };
                        const Evaluation seededCandidate = evaluatePose(
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
                        ++seedEvaluations;
                        if (!seededCandidate.valid) {
                            continue;
                        }
                        const std::array<int, 2> anchorDetectionIndices {{
                            detectionIndexA,
                            detectionIndexB
                        }};
                        const std::array<int, 2> anchorCatalogIndices {{
                            brightCatalogStars[firstCatalog].catalogIndex,
                            brightCatalogStars[secondCatalog].catalogIndex
                        }};
                        if (!hasSeedAnchorSupport(seededCandidate, anchorDetectionIndices, anchorCatalogIndices, 2)) {
                            continue;
                        }

                        const Evaluation candidate = evaluatePose(
                            settings,
                            catalogContext,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            detectionIndices,
                            seededCandidate.azimuthDegrees,
                            seededCandidate.elevationDegrees,
                            seededCandidate.rollDegrees,
                            seededCandidate.fovDegrees,
                            nullptr,
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
                            ++verifiedSeeds;
                        }
                    }
                }
            }
        }
    }
    recordProfileMetric(QStringLiteral("search.brightPairSeedEvaluations"), seedEvaluations);
    recordProfileMetric(QStringLiteral("search.brightPairVerifiedSeeds"), verifiedSeeds);

    std::sort(seeds.begin(), seeds.end(), [this](const Evaluation& lhs, const Evaluation& rhs) {
        return isBetterWeakModeEvaluation(lhs, rhs);
    });
    if (seeds.size() > 64) {
        seeds.resize(64);
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

    const bool isWideFisheyeLens = isWidePlateSolveContext(settings);
    const QVector<int> signatureDetectionIndices = isWideFisheyeLens
        ? selectDetectionIndicesForBlindSignatures(starDetections, detectionIndices, 10, 10, 16)
        : detectionIndices;
    const QVector<QuadSignature> detectionQuads = buildDetectionQuadSignatures(
        starDetections,
        signatureDetectionIndices,
        isWideFisheyeLens ? 16 : 14);
    const QVector<QuadSignature> catalogQuads = buildCatalogQuadSignatures(settings, visibleStars);
    if (detectionQuads.isEmpty() || catalogQuads.isEmpty()) {
        return seeds;
    }
    const double ratioTolerance = (plateSolveStartUsesDirection(settings) && (settings.m_fov <= 5.0))
        ? 0.06
        : isWideFisheyeLens ? 0.06
        : 0.03;
    const bool ignoreOrientationHandedness = isWideFisheyeLens
        || (plateSolveStartUsesDirection(settings) && (settings.m_fov <= 5.0));
    const QHash<qint64, QVector<int>> catalogQuadBuckets =
        buildQuadSignatureBuckets(catalogQuads, ratioTolerance);
    const int bucketRadius = 1;
    recordProfileMetric(QStringLiteral("search.quadDetectionSignatures"), detectionQuads.size());
    recordProfileMetric(QStringLiteral("search.quadCatalogSignatures"), catalogQuads.size());

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
    qint64 ratioCandidates = 0;
    qint64 ratioMatches = 0;
    qint64 seedEvaluations = 0;
    qint64 verifiedSeeds = 0;

    for (const QuadSignature& detectionQuad : detectionQuads)
    {
        if (earlyExit) break;
        const int detectionFirstBin = signatureRatioBin(detectionQuad.edgeRatios[0], ratioTolerance);
        const int detectionSecondBin = signatureRatioBin(detectionQuad.edgeRatios[1], ratioTolerance);
        for (int firstBinOffset = -bucketRadius; firstBinOffset <= bucketRadius; ++firstBinOffset)
        {
            if (earlyExit) break;
            for (int secondBinOffset = -bucketRadius; secondBinOffset <= bucketRadius; ++secondBinOffset)
            {
                if (earlyExit) break;
                const auto bucketIt = catalogQuadBuckets.constFind(
                    signatureBucketKey(detectionFirstBin + firstBinOffset, detectionSecondBin + secondBinOffset));
                if (bucketIt == catalogQuadBuckets.constEnd()) {
                    continue;
                }
                for (int catalogQuadIndex : *bucketIt)
                {
                    if (earlyExit) break;
                    const QuadSignature& catalogQuad = catalogQuads.at(catalogQuadIndex);
                    ++ratioCandidates;
                    bool ratiosMatch = true;
                    for (int idx = 0; idx < 5; ++idx)
                    {
                        if (std::fabs(detectionQuad.edgeRatios[idx] - catalogQuad.edgeRatios[idx]) > ratioTolerance)
                        {
                            ratiosMatch = false;
                            break;
                        }
                    }
                    if (!ratiosMatch) {
                        continue;
                    }

                    ++ratioMatches;
                    if (!ignoreOrientationHandedness && ((detectionQuad.orientation * catalogQuad.orientation) < 0.0)) {
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
                maxAngularDistance * static_cast<double>(std::max(imageSize.width(), imageSize.height())) / std::max(1.0, detectionQuad.longestDistance),
                static_cast<double>(CameraSettings::m_minFov),
                static_cast<double>(CameraSettings::m_maxFov));
            if (!seedFovCompatibleWithStartFov(settings, baseSeedFov)) {
                continue;
            }

            // FoV-scaled dedup so wide-field seeds don't swallow nearby distinct directions.
            const double dedupRadiusDegrees = std::clamp(baseSeedFov * 0.05, 0.5, 5.0);
            bool alreadyTried = false;
            for (const TriedDirection& tried : triedDirections) {
                // Use wrap-aware azimuth distance so seeds near 0°/360° are correctly deduped.
                const double azDiff = std::fabs(seedAzimuth - tried.azimuthDegrees);
                const double azDist = (azDiff <= 180.0) ? azDiff : 360.0 - azDiff;
                if (azDist < dedupRadiusDegrees
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
            const std::array<double, 5> quadFovScales = plateSolveStartUsesFov(settings)
                ? std::array<double, 5>{{0.96, 1.0, 1.04, 1.0, 1.0}}
                : isFisheyeLensQ
                    ? std::array<double, 5>{{0.60, 0.80, 1.0, 1.25, 1.60}}
                    : std::array<double, 5>{{0.85, 0.95, 1.0, 1.10, 1.20}};
            const int quadFovScaleCount = plateSolveStartUsesFov(settings) ? 3 : 5;
            const double quadSeedBaseFov = plateSolveStartUsesFov(settings)
                ? static_cast<double>(settings.m_fov)
                : baseSeedFov;
            for (int fovScaleIndex = 0; fovScaleIndex < quadFovScaleCount; ++fovScaleIndex)
            {
                if (earlyExit) break;
                const double fovScale = quadFovScales[fovScaleIndex];
                const double seedFov = std::clamp(
                    quadSeedBaseFov * fovScale,
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

                        const Evaluation seededCandidate = evaluatePose(
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
                        ++seedEvaluations;
                        if (!seededCandidate.valid) {
                            continue;
                        }
                        const std::array<int, 4> anchorCatalogIndices {{
                            quadStars[permutation[0]].catalogIndex,
                            quadStars[permutation[1]].catalogIndex,
                            quadStars[permutation[2]].catalogIndex,
                            quadStars[permutation[3]].catalogIndex
                        }};
                        if (!hasSeedAnchorSupport(seededCandidate, detectionQuad.indices, anchorCatalogIndices, 3)) {
                            continue;
                        }

                        const Evaluation candidate = evaluatePose(
                            settings,
                            catalogContext,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            detectionIndices,
                            seededCandidate.azimuthDegrees,
                            seededCandidate.elevationDegrees,
                            seededCandidate.rollDegrees,
                            seededCandidate.fovDegrees,
                            nullptr,
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
                            ++verifiedSeeds;
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

        }
    }

    recordProfileMetric(QStringLiteral("search.quadRatioCandidates"), ratioCandidates);
    recordProfileMetric(QStringLiteral("search.quadRatioMatches"), ratioMatches);
    recordProfileMetric(QStringLiteral("search.quadSeedEvaluations"), seedEvaluations);
    recordProfileMetric(QStringLiteral("search.quadVerifiedSeeds"), verifiedSeeds);

    std::sort(seeds.begin(), seeds.end(), [this](const Evaluation& lhs, const Evaluation& rhs) {
        return isBetterWeakModeEvaluation(lhs, rhs);
    });
    const int seedLimit = isWideFisheyeLens ? 64 : 12;
    if (seeds.size() > seedLimit) {
        seeds.resize(seedLimit);
    }

    return seeds;
}

static quint64 spatialCellKey(int x, int y)
{
    return (static_cast<quint64>(static_cast<quint32>(x)) << 32)
        | static_cast<quint32>(y);
}

static double angularDistanceDegrees(double lhs, double rhs)
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

QHash<int, double> detectionBrightnessRanks(const QVector<CameraPipelineStarDetection>& starDetections,
                                            const QVector<int>& detectionIndices) const
{
    QVector<int> sorted = detectionIndices;
    std::sort(sorted.begin(), sorted.end(), [this, &starDetections](int lhs, int rhs) {
        const double lhsBrightness = cachedDetectionBrightnessMetric(starDetections, lhs);
        const double rhsBrightness = cachedDetectionBrightnessMetric(starDetections, rhs);
        if (!qFuzzyCompare(lhsBrightness + 1.0, rhsBrightness + 1.0)) {
            return lhsBrightness > rhsBrightness;
        }
        return starDetections[lhs].m_qualityScore > starDetections[rhs].m_qualityScore;
    });

    QHash<int, double> ranks;
    ranks.reserve(sorted.size());
    const double divisor = std::max(1, static_cast<int>(sorted.size()) - 1);
    for (int i = 0; i < sorted.size(); ++i) {
        ranks.insert(sorted[i], static_cast<double>(i) / divisor);
    }
    return ranks;
}

static QVector<double> projectedBrightnessRanks(const QVector<ProjectedCatalogStar>& projectedStars)
{
    bool alreadyMagnitudeSorted = true;
    for (int i = 1; i < projectedStars.size(); ++i)
    {
        if (projectedStars[i - 1].magnitude > projectedStars[i].magnitude)
        {
            alreadyMagnitudeSorted = false;
            break;
        }
    }
    if (alreadyMagnitudeSorted)
    {
        QVector<double> ranks(projectedStars.size(), 0.0);
        const double divisor = std::max(1, static_cast<int>(projectedStars.size()) - 1);
        for (int i = 0; i < projectedStars.size(); ++i) {
            ranks[i] = static_cast<double>(i) / divisor;
        }
        return ranks;
    }

    QVector<int> sorted;
    sorted.reserve(projectedStars.size());
    for (int i = 0; i < projectedStars.size(); ++i) {
        sorted.append(i);
    }
    std::sort(sorted.begin(), sorted.end(), [&projectedStars](int lhs, int rhs) {
        return projectedStars[lhs].magnitude < projectedStars[rhs].magnitude;
    });

    QVector<double> ranks(projectedStars.size(), 0.0);
    const double divisor = std::max(1, static_cast<int>(sorted.size()) - 1);
    for (int i = 0; i < sorted.size(); ++i) {
        ranks[sorted[i]] = static_cast<double>(i) / divisor;
    }
    return ranks;
}

double matchBrightnessRankError(const QVector<CameraPipelineStarDetection>& starDetections,
                                const QVector<int>& detectionIndices,
                                const QVector<ProjectedCatalogStar>& projectedStars,
                                const QVector<Match>& matches) const
{
    if (matches.size() < 2) {
        return 0.0;
    }

    const QHash<int, double> detectionRanks = detectionBrightnessRanks(starDetections, detectionIndices);
    const QVector<double> projectedRanks = projectedBrightnessRanks(projectedStars);
    QHash<int, double> catalogRanks;
    catalogRanks.reserve(projectedStars.size());
    for (int i = 0; i < projectedStars.size(); ++i) {
        catalogRanks.insert(projectedStars[i].catalogIndex, projectedRanks[i]);
    }

    double sumError = 0.0;
    int count = 0;
    QSet<int> matchedDetections;
    matchedDetections.reserve(matches.size());
    for (const Match& match : matches)
    {
        matchedDetections.insert(match.detectionIndex);
        const auto detectionIt = detectionRanks.constFind(match.detectionIndex);
        const auto catalogIt = catalogRanks.constFind(match.catalogIndex);
        if ((detectionIt == detectionRanks.cend()) || (catalogIt == catalogRanks.cend())) {
            continue;
        }
        sumError += std::fabs(detectionIt.value() - catalogIt.value());
        ++count;
    }

    const int brightDetectionCount = std::min(
        static_cast<int>(detectionIndices.size()),
        std::max(3, std::min(8, static_cast<int>(matches.size()) + 1)));
    const double brightRankThreshold = (detectionIndices.size() > 1)
        ? static_cast<double>(brightDetectionCount - 1) / static_cast<double>(detectionIndices.size() - 1)
        : 0.0;
    for (int detectionIndex : detectionIndices)
    {
        const auto detectionIt = detectionRanks.constFind(detectionIndex);
        if ((detectionIt == detectionRanks.cend())
            || (detectionIt.value() > brightRankThreshold)
            || matchedDetections.contains(detectionIndex))
        {
            continue;
        }

        sumError += 0.75 + 0.25 * (1.0 - detectionIt.value());
        ++count;
    }

    return count > 0 ? sumError / count : 0.0;
}

static double meanCatalogMagnitudeForMatches(const QVector<CatalogStar>& catalogStars,
                                             const QVector<Match>& matches)
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

QVector<Match> buildMatches(const PlateSolveCatalogContext& catalogContext,
                            const QVector<CameraPipelineStarDetection>& starDetections,
                            const QVector<int>& detectionIndices,
                            const QVector<ProjectedCatalogStar>& projectedStars,
                            double matchRadiusPixels)
{
    QVector<CandidatePair>& candidatePairs = m_candidatePairScratch;
    candidatePairs.clear();
    const QVector<CatalogStar>& catalogStars = catalogContext.catalogStars;
    const QHash<int, double> detectionRanks = detectionBrightnessRanks(starDetections, detectionIndices);
    const QVector<double> projectedRanks = projectedBrightnessRanks(projectedStars);
    const double maxDistanceSquared = matchRadiusPixels * matchRadiusPixels;
    const double cellSize = std::max(1.0, matchRadiusPixels);
    QHash<quint64, QVector<int>>& projectedStarGrid = m_projectedStarGridScratch;
    projectedStarGrid.clear();
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

                    const double detectionRank = detectionRanks.value(detectionIndex, 0.5);
                    const double catalogRank = (projectedIndex >= 0) && (projectedIndex < projectedRanks.size())
                        ? projectedRanks[projectedIndex]
                        : 0.5;
                    candidatePairs.append({
                        detectionIndex,
                        projected.catalogIndex,
                        projectedIndex,
                        std::sqrt(distanceSquared),
                        std::fabs(detectionRank - catalogRank),
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
    std::sort(candidatePairs.begin(), candidatePairs.end(), [this, &starDetections, matchRadiusPixels](const CandidatePair& lhs, const CandidatePair& rhs) {
        if (lhs.detectionIndex != rhs.detectionIndex) {
            return lhs.detectionIndex < rhs.detectionIndex;
        }
        const double lhsReliability = cachedDetectionReliabilityMetric(starDetections, lhs.detectionIndex);
        const double rhsReliability = cachedDetectionReliabilityMetric(starDetections, rhs.detectionIndex);
        const double lhsCost = lhs.distancePixels + matchRadiusPixels * 0.75 * lhs.brightnessRankError
            - std::min(matchRadiusPixels * 0.25, std::log1p(lhsReliability));
        const double rhsCost = rhs.distancePixels + matchRadiusPixels * 0.75 * rhs.brightnessRankError
            - std::min(matchRadiusPixels * 0.25, std::log1p(rhsReliability));
        return lhsCost < rhsCost;
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

    std::sort(candidatePairs.begin(), candidatePairs.end(), [this, &catalogStars, &starDetections, matchRadiusPixels](const CandidatePair& lhs, const CandidatePair& rhs) {
        const double lhsReliability = cachedDetectionReliabilityMetric(starDetections, lhs.detectionIndex);
        const double rhsReliability = cachedDetectionReliabilityMetric(starDetections, rhs.detectionIndex);
        const double lhsSupportScore = static_cast<double>(lhs.geometricSupport)
            - 1.25 * lhs.brightnessRankError
            + 0.20 * std::log1p(lhsReliability);
        const double rhsSupportScore = static_cast<double>(rhs.geometricSupport)
            - 1.25 * rhs.brightnessRankError
            + 0.20 * std::log1p(rhsReliability);
        if (std::fabs(lhsSupportScore - rhsSupportScore) > 0.20) {
            return lhsSupportScore > rhsSupportScore;
        }
        const double lhsCost = lhs.distancePixels + matchRadiusPixels * 0.75 * lhs.brightnessRankError
            - std::min(matchRadiusPixels * 0.25, std::log1p(lhsReliability));
        const double rhsCost = rhs.distancePixels + matchRadiusPixels * 0.75 * rhs.brightnessRankError
            - std::min(matchRadiusPixels * 0.25, std::log1p(rhsReliability));
        if (!qFuzzyCompare(lhsCost + 1.0, rhsCost + 1.0)) {
            return lhsCost < rhsCost;
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

static void appendSupplementalMatches(const QVector<CameraPipelineStarDetection>& starDetections,
                                      const QVector<ProjectedCatalogStar>& projectedStars,
                                      double matchRadiusPixels,
                                      const QVector<int>* supplementalDetectionIndices,
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

    const auto appendSupplementalForDetection = [&](int detectionIndex)
    {
        if (detectionMatched[detectionIndex]) {
            return;
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

static void appendWideBrightSupplementalMatches(const CameraSettings& settings,
                                                const QVector<CameraPipelineStarDetection>& starDetections,
                                                const QVector<ProjectedCatalogStar>& projectedStars,
                                                const QSize& imageSize,
                                                double matchRadiusPixels,
                                                QVector<Match>& matches)
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

void logUnmatchedDetections(const PlateSolveCatalogContext& catalogContext,
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

    const QVector<CatalogStar>& catalogStars = catalogContext.catalogStars;
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
                        const QVector<int>* allowedCatalogIndices = nullptr,
                        double centerOffsetXPixels = 0.0,
                        double centerOffsetYPixels = 0.0,
                        double distortionK1 = 0.0,
                        double matchRadiusOverride = -1.0)
{
    Q_UNUSED(captureDateTimeUtc)

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

    buildProjectedCatalogInto(
        catalogContext,
        projector,
        matchRadiusPixels,
        allowedCatalogIndices,
        m_projectedCatalogScratch);
    const QVector<ProjectedCatalogStar>& projectedStars = m_projectedCatalogScratch;
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
    evaluation.brightnessRankError = matchBrightnessRankError(
        starDetections,
        detectionIndices,
        projectedStars,
        evaluation.matches);
    evaluation.meanCatalogMagnitude = meanCatalogMagnitudeForMatches(
        catalogContext.catalogStars,
        evaluation.matches);

    double sumSquaredError = 0.0;
    for (const Match& match : evaluation.matches) {
        sumSquaredError += match.distancePixels * match.distancePixels;
    }
    evaluation.rmsErrorPixels = std::sqrt(sumSquaredError / evaluation.matchCount);
    evaluation.valid = true;
    return evaluation;
}

// -------------------------------------------------------------------------
// Blind-grid roll-sweep optimisation helpers
//
// For the exhaustive blind grid (Az × El × FOV × Roll ≈ 52 k evaluations),
// rolling the camera is a pure 2D rotation in pixel space around the
// principal point. We therefore project the catalog just once per (Az,El,FOV)
// at roll=0, cache the pixel offsets in m_blindGridCache, and then for each
// of the 13 roll values compute the rolled positions by 2D rotation — saving
// ~13× the catalog-projection cost in the inner loop.
// -------------------------------------------------------------------------

// Project every visible catalog star at roll=0 and store pixel offsets
// relative to the principal point in m_blindGridCache.  No image-bounds
// check is performed here; populateBlindGridProjectedCatalog applies the
// bounds filter after rotating to a specific roll angle.
void buildBlindGridCache(const PlateSolveCatalogContext& catalogContext,
                         const SkyProjector& refProjector,
                         const QVector<int>* allowedCatalogIndices = nullptr)
{
    m_blindGridCache.clear();
    if (!refProjector.valid)
        return;

    const auto appendCachedStar = [&](const VisibleCatalogStar& vs)
    {
        QPointF pt;
        if (!projectVector(refProjector, vs.vector, pt))
            return;
        BlindGridCachedStar cs;
        cs.catalogIndex = vs.catalogIndex;
        cs.dxRef        = static_cast<float>(pt.x() - refProjector.principalPointX);
        cs.dyRef        = static_cast<float>(refProjector.principalPointY - pt.y());
        cs.magnitude    = static_cast<float>(vs.magnitude);
        m_blindGridCache.push_back(cs);
    };

    if (allowedCatalogIndices)
    {
        m_blindGridCache.reserve(allowedCatalogIndices->size());
        for (int catalogIndex : *allowedCatalogIndices)
        {
            const auto it = catalogContext.visibleStarIndexByCatalogIndex.constFind(catalogIndex);
            if (it != catalogContext.visibleStarIndexByCatalogIndex.cend()) {
                appendCachedStar(catalogContext.visibleStars[*it]);
            }
        }
    }
    else
    {
        m_blindGridCache.reserve(catalogContext.visibleStars.size());
        for (const VisibleCatalogStar& visibleStar : catalogContext.visibleStars) {
            appendCachedStar(visibleStar);
        }
    }
}

// Rotate the roll=0 offsets by rollDegrees and populate
// m_blindGridProjectedScratch with only the stars that fall within the
// expanded image bounds (image rect + matchRadiusPixels margin).
void populateBlindGridProjectedCatalog(double rollDegrees,
                                       double matchRadiusPixels,
                                       const SkyProjector& refProjector)
{
    m_blindGridProjectedScratch.clear();
    if (m_blindGridCache.isEmpty() || !refProjector.valid)
        return;
    const double rollRad = degToRad(rollDegrees);
    const double cosR    = std::cos(rollRad);
    const double sinR    = std::sin(rollRad);
    const double cx      = refProjector.principalPointX;
    const double cy      = refProjector.principalPointY;
    const QRectF expandedBounds(-matchRadiusPixels,
                                -matchRadiusPixels,
                                refProjector.width  + 2.0 * matchRadiusPixels,
                                refProjector.height + 2.0 * matchRadiusPixels);
    m_blindGridProjectedScratch.reserve(m_blindGridCache.size());
    for (const BlindGridCachedStar& cs : m_blindGridCache)
    {
        const double dxRef = static_cast<double>(cs.dxRef);
        const double dyRef = static_cast<double>(cs.dyRef);
        // 2-D rotation of the (right, up) offset vector. This matches
        // createProjector's camera-basis roll: right' = right*cos - up*sin,
        // up' = right*sin + up*cos.
        const double dx = dxRef * cosR - dyRef * sinR;
        const double dy = dxRef * sinR + dyRef * cosR;
        const QPointF pt(cx + dx, cy - dy);
        if (!expandedBounds.contains(pt))
            continue;
        m_blindGridProjectedScratch.push_back({cs.catalogIndex, pt, static_cast<double>(cs.magnitude)});
    }
}

// Evaluate a pose using the pre-rotated catalog already stored in
// m_blindGridProjectedScratch.  Mirrors evaluatePose but skips the
// createProjector + buildProjectedCatalogInto steps.
Evaluation evaluatePoseFromPrecomputedCatalog(
    const CameraSettings& settings,
    const PlateSolveCatalogContext& catalogContext,
    const QVector<CameraPipelineStarDetection>& starDetections,
    const QVector<int>& detectionIndices,
    double azimuthDegrees, double elevationDegrees,
    double rollDegrees, double fovDegrees,
    double centerOffsetXPixels, double centerOffsetYPixels,
    double distortionK1,
    double matchRadiusPixels)
{
    Evaluation evaluation;
    evaluation.azimuthDegrees      = normalizeDegrees(azimuthDegrees);
    evaluation.elevationDegrees    = elevationDegrees;
    evaluation.rollDegrees         = rollDegrees;
    evaluation.fovDegrees          = fovDegrees;
    evaluation.centerOffsetXPixels = centerOffsetXPixels;
    evaluation.centerOffsetYPixels = centerOffsetYPixels;
    evaluation.distortionK1        = distortionK1;

    const QVector<ProjectedCatalogStar>& projectedStars = m_blindGridProjectedScratch;
    if (projectedStars.isEmpty())
        return evaluation;

    evaluation.matches = buildMatches(
        catalogContext, starDetections, detectionIndices, projectedStars, matchRadiusPixels);
    evaluation.matchCount = evaluation.matches.size();
    if (evaluation.matchCount <= 0)
        return evaluation;

    evaluation.brightnessRankError = matchBrightnessRankError(
        starDetections, detectionIndices, projectedStars, evaluation.matches);
    evaluation.meanCatalogMagnitude = meanCatalogMagnitudeForMatches(
        catalogContext.catalogStars, evaluation.matches);
    double sumSq = 0.0;
    for (const Match& m : evaluation.matches)
        sumSq += m.distancePixels * m.distancePixels;
    evaluation.rmsErrorPixels = std::sqrt(sumSq / evaluation.matchCount);
    evaluation.valid = true;
    return evaluation;
}

Evaluation evaluateAnchoredPose(const CameraSettings& settings,
                                const PlateSolveCatalogContext& catalogContext,
                                const QSize& imageSize,
                                const QDateTime& captureDateTimeUtc,
                                const QVector<CameraPipelineStarDetection>& starDetections,
                                const QVector<int>& detectionIndices,
                                const QVector<int>& allowedCatalogIndices,
                                const GuidedAnchorPair& anchor,
                                double azimuthDegrees,
                                double elevationDegrees,
                                double rollDegrees,
                                double fovDegrees,
                                double centerOffsetXPixels,
                                double centerOffsetYPixels,
                                double distortionK1,
                                double matchRadiusPixels)
{
    Q_UNUSED(captureDateTimeUtc)

    Evaluation evaluation;
    evaluation.anchored = true;
    evaluation.anchorDetectionIndex = anchor.detectionIndex;
    evaluation.anchorCatalogIndex = anchor.catalogIndex;
    evaluation.azimuthDegrees = normalizeDegrees(azimuthDegrees);
    evaluation.elevationDegrees = elevationDegrees;
    evaluation.rollDegrees = rollDegrees;
    evaluation.fovDegrees = fovDegrees;
    evaluation.centerOffsetXPixels = centerOffsetXPixels;
    evaluation.centerOffsetYPixels = centerOffsetYPixels;
    evaluation.distortionK1 = distortionK1;

    if ((anchor.detectionIndex < 0)
        || (anchor.detectionIndex >= starDetections.size())
        || (anchor.catalogIndex < 0)
        || (anchor.catalogIndex >= catalogContext.catalogStars.size()))
    {
        return evaluation;
    }

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

    const auto anchorVisibleIt = catalogContext.visibleStarIndexByCatalogIndex.constFind(anchor.catalogIndex);
    if (anchorVisibleIt == catalogContext.visibleStarIndexByCatalogIndex.cend()) {
        return evaluation;
    }

    QPointF anchorProjectedPoint;
    if (!projectVector(projector, catalogContext.visibleStars[*anchorVisibleIt].vector, anchorProjectedPoint)) {
        return evaluation;
    }

    const double anchorDistance = pointDistancePixels(
        starDetections[anchor.detectionIndex].m_center,
        anchorProjectedPoint);
    if (anchorDistance > matchRadiusPixels) {
        return evaluation;
    }

    buildProjectedCatalogInto(
        catalogContext,
        projector,
        matchRadiusPixels,
        allowedCatalogIndices.isEmpty() ? nullptr : &allowedCatalogIndices,
        m_projectedCatalogScratch);
    const QVector<ProjectedCatalogStar>& projectedStars = m_projectedCatalogScratch;
    if (projectedStars.isEmpty()) {
        return evaluation;
    }

    const QVector<Match> automaticMatches = buildMatches(
        catalogContext,
        starDetections,
        detectionIndices,
        projectedStars,
        matchRadiusPixels);

    evaluation.matches.reserve(automaticMatches.size() + 1);
    evaluation.matches.append({anchor.detectionIndex, anchor.catalogIndex, anchorDistance});
    QSet<int> matchedDetections;
    QSet<int> matchedCatalogStars;
    matchedDetections.insert(anchor.detectionIndex);
    matchedCatalogStars.insert(anchor.catalogIndex);
    for (const Match& match : automaticMatches)
    {
        if (matchedDetections.contains(match.detectionIndex)
            || matchedCatalogStars.contains(match.catalogIndex))
        {
            continue;
        }
        matchedDetections.insert(match.detectionIndex);
        matchedCatalogStars.insert(match.catalogIndex);
        evaluation.matches.append(match);
    }

    evaluation.matchCount = evaluation.matches.size();
    if (evaluation.matchCount <= 0) {
        return evaluation;
    }

    evaluation.brightnessRankError = matchBrightnessRankError(
        starDetections,
        detectionIndices,
        projectedStars,
        evaluation.matches);
    evaluation.meanCatalogMagnitude = meanCatalogMagnitudeForMatches(
        catalogContext.catalogStars,
        evaluation.matches);

    double sumSquaredError = 0.0;
    for (const Match& match : evaluation.matches) {
        sumSquaredError += match.distancePixels * match.distancePixels;
    }
    evaluation.rmsErrorPixels = std::sqrt(sumSquaredError / evaluation.matchCount);
    evaluation.valid = true;
    return evaluation;
}

static double medianDistancePixels(const QVector<Match>& matches)
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

static double maxDistancePixels(const QVector<Match>& matches)
{
    double maxDistance = 0.0;
    for (const Match& match : matches) {
        maxDistance = std::max(maxDistance, match.distancePixels);
    }
    return maxDistance;
}

static double detectionMatchWeight(const CameraPipelineStarDetection& detection)
{
    const double centroidUncertainty = std::isfinite(static_cast<double>(detection.m_centroidUncertainty))
        ? std::max(0.05, static_cast<double>(detection.m_centroidUncertainty))
        : 4.0;
    const double snr = std::isfinite(static_cast<double>(detection.m_snr))
        ? std::max(0.0, static_cast<double>(detection.m_snr))
        : 0.0;
    double weight = 1.0 / (centroidUncertainty * centroidUncertainty);
    weight *= std::max(0.25, static_cast<double>(detection.m_roundness));
    weight *= std::max(0.25, static_cast<double>(detection.m_fillRatio));
    weight /= std::max(1.0, static_cast<double>(detection.m_aspectRatio));
    weight *= 1.0 + std::min(4.0, std::log1p(snr));
    if (detection.m_saturated) {
        weight *= 0.65;
    }
    if (detection.m_hotPixelSuspect) {
        weight *= 0.10;
    }
    return std::clamp(weight, 0.01, 100.0);
}

static double weightedRmsDistancePixels(const QVector<CameraPipelineStarDetection>& starDetections,
                                        const QVector<Match>& matches)
{
    double weightedSumSquaredError = 0.0;
    double weightSum = 0.0;
    for (const Match& match : matches)
    {
        if ((match.detectionIndex < 0) || (match.detectionIndex >= starDetections.size())) {
            continue;
        }
        const double weight = detectionMatchWeight(starDetections[match.detectionIndex]);
        weightedSumSquaredError += weight * match.distancePixels * match.distancePixels;
        weightSum += weight;
    }
    if (weightSum <= 0.0) {
        double sumSquaredError = 0.0;
        for (const Match& match : matches) {
            sumSquaredError += match.distancePixels * match.distancePixels;
        }
        return matches.isEmpty() ? std::numeric_limits<double>::infinity()
            : std::sqrt(sumSquaredError / matches.size());
    }
    return std::sqrt(weightedSumSquaredError / weightSum);
}

static bool hasGeometricallyConsistentMatches(const QVector<CameraPipelineStarDetection>& starDetections,
                                              const QVector<ProjectedCatalogStar>& projectedStars,
                                              const QVector<Match>& matches,
                                              double matchRadiusPixels)
{
    if (matches.size() < 4) {
        return true;
    }

    QHash<int, QPointF> projectedPointByCatalogIndex;
    projectedPointByCatalogIndex.reserve(projectedStars.size());
    for (const ProjectedCatalogStar& projectedStar : projectedStars) {
        projectedPointByCatalogIndex.insert(projectedStar.catalogIndex, projectedStar.point);
    }

    int checkedPairs = 0;
    int consistentPairs = 0;
    for (int i = 0; i < matches.size(); ++i)
    {
        const Match& lhs = matches[i];
        if ((lhs.detectionIndex < 0) || (lhs.detectionIndex >= starDetections.size())) {
            continue;
        }
        const auto lhsProjectedIt = projectedPointByCatalogIndex.constFind(lhs.catalogIndex);
        if (lhsProjectedIt == projectedPointByCatalogIndex.cend()) {
            continue;
        }
        for (int j = i + 1; j < matches.size(); ++j)
        {
            const Match& rhs = matches[j];
            if ((rhs.detectionIndex < 0) || (rhs.detectionIndex >= starDetections.size())) {
                continue;
            }
            const auto rhsProjectedIt = projectedPointByCatalogIndex.constFind(rhs.catalogIndex);
            if (rhsProjectedIt == projectedPointByCatalogIndex.cend()) {
                continue;
            }

            const double detectionDistance = pointDistancePixels(
                starDetections[lhs.detectionIndex].m_center,
                starDetections[rhs.detectionIndex].m_center);
            const double projectedDistance = pointDistancePixels(lhsProjectedIt.value(), rhsProjectedIt.value());
            const double tolerance = std::max(
                std::max(3.0, matchRadiusPixels * 0.35),
                0.08 * std::max(detectionDistance, projectedDistance));
            ++checkedPairs;
            if (std::fabs(detectionDistance - projectedDistance) <= tolerance) {
                ++consistentPairs;
            }
        }
    }

    if (checkedPairs < 6) {
        return true;
    }
    const double fraction = static_cast<double>(consistentPairs) / static_cast<double>(checkedPairs);
    return fraction >= 0.55;
}

static QVector<Match> rejectOutlierMatches(const QVector<Match>& matches,
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

static bool isAcceptableBlindSolve(const CameraSettings& settings,
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

static bool isAcceptableSparseWideBlindSolve(const CameraSettings& settings,
                                             const QVector<CameraPipelineStarDetection>& starDetections,
                                             const FinalMatchPassEvaluation& evaluation)
{
    if (!isWidePlateSolveContext(settings)
        || (settings.m_plateSolveStartMode != CameraSettings::PlateSolveStartBlind)
        || (starDetections.size() > 12)
        || (evaluation.finalMatches.size() < 3))
    {
        return false;
    }

    return (evaluation.rmsErrorPixels <= std::min(settings.m_plateSolveFinalMatchRadius * 0.70, 18.0))
        && (evaluation.medianErrorPixels <= std::min(settings.m_plateSolveFinalMatchRadius * 0.60, 16.0))
        && (evaluation.maxErrorPixels <= std::min(settings.m_plateSolveFinalMatchRadius * 1.25, 32.0))
        && (evaluation.matchedBrightDetections >= 2)
        && (evaluation.brightDetectionMagnitudeError <= 1.50);
}

static bool isStrongGuidedSolve(const CameraSettings& settings,
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

static int minimumDirectionSeedAcceptedMatches(const CameraSettings& settings,
                                               const QVector<CameraPipelineStarDetection>& starDetections)
{
    const int configuredMinimum = std::max(settings.m_plateSolveMinMatches, 4);

    // In a guided narrow-field solve the user is explicitly providing FoV + direction,
    // so requiring extra matches just because the detector found many faint blobs rejects
    // valid 1-degree telescope images. Keep the match count at the configured floor and
    // let RMS/median/max-distance gates carry the false-positive protection.
    if (settings.m_fov <= 5.0) {
        return configuredMinimum;
    }
    if (settings.m_fov <= 15.0)
    {
        return std::max(configuredMinimum,
            std::min(6, std::max(4, static_cast<int>(std::ceil(static_cast<double>(starDetections.size()) * 0.10)))));
    }

    return std::max(settings.m_plateSolveMinMatches + 1,
        std::min(8, std::max(5, static_cast<int>(std::ceil(static_cast<double>(starDetections.size()) * 0.15)))));
}

static QString directionSeedRejectionReason(const CameraSettings& settings,
                                            const QVector<CameraPipelineStarDetection>& starDetections,
                                            const QVector<Match>& matches,
                                            double rmsErrorPixels,
                                            double maxErrorPixels)
{
    // For narrow-field (telescope) solves the FOV is pinned, so residuals reflect true
    // centroid accuracy rather than FOV-absorbed error.  Use slightly looser thresholds.
    const bool narrowField = settings.m_fov <= 5.0;
    const int requiredMatches = minimumDirectionSeedAcceptedMatches(settings, starDetections);
    const double medianError = medianDistancePixels(matches);
    const double maxRmsError = std::min(settings.m_plateSolveFinalMatchRadius * (narrowField ? 0.75 : 0.70), 18.0);
    // Narrow-field: residuals reflect real centroid noise with pinned FOV — use 0.75×.
    // Wide-field fisheye: lens distortion at large angles raises residuals even for correct
    // solves; 0.65x (= 15.6 px at 24 px radius) is needed to accept these valid solutions
    // while still rejecting clearly wrong ones.
    const double maxMedianError = narrowField
        ? std::min(settings.m_plateSolveFinalMatchRadius * 0.75, 18.0)
        : std::min(settings.m_plateSolveFinalMatchRadius * 0.65, 18.0);
    const double maxWorstError = std::min(
        settings.m_plateSolveFinalMatchRadius * (isWidePlateSolveContext(settings) ? 1.15 : 1.05),
        36.0);

    QStringList reasons;
    if (matches.size() < requiredMatches) {
        reasons.append(QStringLiteral("matches %1 < required %2").arg(matches.size()).arg(requiredMatches));
    }
    if (rmsErrorPixels > maxRmsError) {
        reasons.append(QStringLiteral("RMS %1 > %2").arg(rmsErrorPixels, 0, 'f', 2).arg(maxRmsError, 0, 'f', 2));
    }
    if (medianError > maxMedianError) {
        reasons.append(QStringLiteral("median %1 > %2").arg(medianError, 0, 'f', 2).arg(maxMedianError, 0, 'f', 2));
    }
    if (maxErrorPixels > maxWorstError) {
        reasons.append(QStringLiteral("max %1 > %2").arg(maxErrorPixels, 0, 'f', 2).arg(maxWorstError, 0, 'f', 2));
    }
    return reasons.isEmpty() ? QStringLiteral("accepted") : reasons.join(QStringLiteral(", "));
}

static bool isAcceptableDirectionSeedSolve(const CameraSettings& settings,
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

static bool isAcceptableDirectionSeedSolve(const CameraSettings& settings,
                                    const QVector<CameraPipelineStarDetection>& starDetections,
                                    const QVector<Match>& matches,
                                    double rmsErrorPixels,
                                    double maxErrorPixels)
{
    const int minAcceptedMatches = minimumDirectionSeedAcceptedMatches(settings, starDetections);
    if (matches.size() < minAcceptedMatches) {
        return false;
    }

    // For narrow-field (telescope) solves the FOV is pinned, so residuals reflect true
    // centroid accuracy rather than FOV-absorbed error.  Use slightly looser thresholds.
    // For wide-field fisheye, lens distortion at large angles raises residuals; 0.65x
    // (= 15.6 px at 24 px radius) is needed to accept valid wide-angle solutions.
    const bool narrowField = settings.m_fov <= 5.0;
    const double medianError = medianDistancePixels(matches);
    const double maxRmsError = std::min(settings.m_plateSolveFinalMatchRadius * (narrowField ? 0.75 : 0.70), 18.0);
    const double maxMedianError = narrowField
        ? std::min(settings.m_plateSolveFinalMatchRadius * 0.75, 18.0)
        : std::min(settings.m_plateSolveFinalMatchRadius * 0.65, 18.0);
    const double maxWorstError = std::min(
        settings.m_plateSolveFinalMatchRadius * (isWidePlateSolveContext(settings) ? 1.15 : 1.05),
        36.0);

    return (rmsErrorPixels <= maxRmsError)
        && (medianError <= maxMedianError)
        && (maxErrorPixels <= maxWorstError);
}

static bool isAcceptableElevationSeedEvaluation(const CameraSettings& settings,
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

static bool isAcceptableElevationSeedSolve(const CameraSettings& settings,
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

double evaluationRmsQuality(const Evaluation& evaluation,
                            double normalizationRadius)
{
    const double safeRadius = std::max(1.0, normalizationRadius);
    const double normalizedRms = evaluation.rmsErrorPixels / safeRadius;
    const double clampedRms = std::min(1.0, std::max(0.0, normalizedRms));
    return 1.0 - 0.5 * clampedRms * clampedRms;
}

double brightnessAffinity(const Evaluation& evaluation)
{
    if (!std::isfinite(evaluation.brightnessRankError) || (evaluation.matches.size() < 3)) {
        return 1.0;
    }

    const double clampedError = std::min(1.0, std::max(0.0, evaluation.brightnessRankError));
    return 1.0 / (1.0 + 3.0 * clampedError * clampedError);
}

double catalogMagnitudeAffinity(const Evaluation& evaluation)
{
    if (!m_useWideCatalogMagnitudePreference
        || !std::isfinite(evaluation.meanCatalogMagnitude)
        || (evaluation.matches.size() < 3))
    {
        return 1.0;
    }

    const double faintExcess = std::max(0.0, evaluation.meanCatalogMagnitude - 2.0);
    return 1.0 / (1.0 + 0.65 * faintExcess * faintExcess);
}

double wideEvaluationMatchWeight(const Evaluation& evaluation)
{
    if (!m_useWideCatalogMagnitudePreference) {
        return static_cast<double>(evaluation.matchCount);
    }

    const int usefulMatchCap = std::max(m_directionSeedMinMatchCount + 4, 8);
    const int cappedMatches = std::min(evaluation.matchCount, usefulMatchCap);
    const int extraMatches = std::max(0, evaluation.matchCount - usefulMatchCap);
    return static_cast<double>(cappedMatches) + 0.15 * std::log1p(static_cast<double>(extraMatches));
}

bool shouldPreferBrighterCatalogMean(double candidateMeanMagnitude,
                                     double bestMeanMagnitude,
                                     double candidateRmsPixels,
                                     double bestRmsPixels,
                                     double comparableRmsPixels) const
{
    if (!m_useWideCatalogMagnitudePreference
        || !std::isfinite(candidateMeanMagnitude)
        || !std::isfinite(bestMeanMagnitude)
        || !std::isfinite(candidateRmsPixels)
        || !std::isfinite(bestRmsPixels))
    {
        return false;
    }

    if ((candidateRmsPixels > comparableRmsPixels) || (bestRmsPixels > comparableRmsPixels)) {
        return false;
    }

    return (bestMeanMagnitude - candidateMeanMagnitude) >= kWideFovMagnitudePreferenceMinDelta;
}

bool hasAcceptableBrightnessConsistency(const Evaluation& evaluation)
{
    return !std::isfinite(evaluation.brightnessRankError)
        || (evaluation.matches.size() < 4)
        || (evaluation.brightnessRankError <= 0.45);
}

bool hasAcceptableGuidedFinalBrightnessConsistency(const CameraSettings& settings,
                                                   const FinalMatchPassEvaluation& evaluation)
{
    if (!std::isfinite(evaluation.brightnessRankError) || (evaluation.finalMatches.size() < 4)) {
        return true;
    }

    if (settings.m_fov <= 5.0) {
        // For narrow-field (telescope) solves the FOV is pinned to the user's value,
        // which strongly constrains the geometry.  Brightness rank ordering is also less
        // reliable when the matched set spans mag 2-13 (saturated bright star + very faint
        // stars).  Use the same relaxed threshold regardless of whether the pose is anchored.
        return evaluation.brightnessRankError <= 0.45;
    }

    const double threshold = (settings.m_fov <= 30.0) ? 0.50
        : 0.60;
    if (evaluation.brightnessRankError > threshold) {
        return false;
    }

    if (isWidePlateSolveContext(settings)
        && (evaluation.brightDetections >= 5)
        && (evaluation.brightDetectionMagnitudeError > 0.85))
    {
        return false;
    }

    return true;
}

bool needsWideBrightAnchorSupport(const CameraSettings& settings,
                                  const QVector<CameraPipelineStarDetection>& starDetections)
{
    return isWidePlateSolveContext(settings)
        && (starDetections.size() >= 32);
}

bool hasAcceptableWideBrightAnchorSupport(const CameraSettings& settings,
                                          const QVector<CameraPipelineStarDetection>& starDetections,
                                          const FinalMatchPassEvaluation& evaluation)
{
    if (!needsWideBrightAnchorSupport(settings, starDetections)) {
        return true;
    }
    if (evaluation.brightDetections < 5) {
        return false;
    }

    const bool brightDetectionsAgree =
        (evaluation.matchedBrightDetections >= 4)
        && (evaluation.brightDetectionMagnitudeError <= 1.50);
    const bool brightDetectionsStronglyAgree =
        (evaluation.finalMatches.size() >= std::max(settings.m_plateSolveMinMatches + 4, 8))
        && (evaluation.brightDetections >= 6)
        && (evaluation.matchedBrightDetections >= 6)
        && (evaluation.brightDetectionMatchFraction >= 0.75)
        && (evaluation.brightDetectionMagnitudeError <= 0.25);
    const bool brightProjectedStarsAgree =
        (evaluation.brightProjectedStars < 5)
        || (evaluation.matchedBrightProjectedStars >= 2)
        || (evaluation.brightProjectedMatchFraction >= 0.25);

    if (settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartBlind) {
        return brightDetectionsStronglyAgree
            || (brightDetectionsAgree && brightProjectedStarsAgree);
    }

    return brightDetectionsAgree
        && (brightProjectedStarsAgree || (evaluation.matchedBrightDetections >= 4));
}

double directionSeedAffinity(const Evaluation& evaluation)
{
    if (!m_useDirectionSeedPreference) {
        return 1.0;
    }

    const double safeAzElScale = std::max(1.0, m_directionSeedAzElScaleDegrees);
    const double safeRollScale = std::max(1.0, m_directionSeedRollScaleDegrees);
    const double safeFovScale = std::max(1.0, m_directionSeedFovScaleDegrees);
    const double normalizedAzimuthDelta = angularDistanceDegrees(
        evaluation.azimuthDegrees,
        m_directionSeedReferenceAzimuthDegrees) / safeAzElScale;
    const double normalizedElevationDelta = std::fabs(
        evaluation.elevationDegrees - m_directionSeedReferenceElevationDegrees) / safeAzElScale;
    const double normalizedRollDelta = m_directionSeedHasRollPreference
        ? angularDistanceDegrees(evaluation.rollDegrees, m_directionSeedReferenceRollDegrees) / safeRollScale
        : 0.0;
    const double normalizedFovDelta = std::fabs(
        evaluation.fovDegrees - m_directionSeedReferenceFovDegrees) / safeFovScale;

    return 1.0 / (1.0
        + 0.35 * normalizedAzimuthDelta * normalizedAzimuthDelta
        + 0.35 * normalizedElevationDelta * normalizedElevationDelta
        + 0.18 * normalizedRollDelta * normalizedRollDelta
        + 0.75 * normalizedFovDelta * normalizedFovDelta);
}

double fovSeedAffinity(const Evaluation& evaluation)
{
    if (!m_useFovSeedPreference) {
        return 1.0;
    }

    const double safeScale = std::max(1.0, m_fovSeedScaleDegrees);
    const double normalizedFovDelta = std::fabs(
        evaluation.fovDegrees - m_fovSeedReferenceDegrees) / safeScale;
    return 1.0 / (1.0 + 1.25 * normalizedFovDelta * normalizedFovDelta);
}

double allSkyZenithAffinity(const Evaluation& evaluation)
{
    if (!m_useAllSkyZenithPreference) {
        return 1.0;
    }

    const double safeScale = std::max(5.0, m_allSkyZenithScaleDegrees);
    const double normalizedElevationDelta =
        std::fabs(evaluation.elevationDegrees - m_allSkyZenithReferenceElevationDegrees) / safeScale;
    return 1.0 / (1.0 + 1.5 * normalizedElevationDelta * normalizedElevationDelta);
}

double guidedDirectionEvaluationScore(const Evaluation& evaluation,
                                      double normalizationRadius = -1.0)
{
    if (normalizationRadius < 0.0) {
        normalizationRadius = m_weakModeNormalizationPixels;
    }
    if (!evaluation.valid) {
        return -std::numeric_limits<double>::infinity();
    }

    return wideEvaluationMatchWeight(evaluation)
        * evaluationRmsQuality(evaluation, normalizationRadius)
        * brightnessAffinity(evaluation)
        * catalogMagnitudeAffinity(evaluation)
        * directionSeedAffinity(evaluation)
        * fovSeedAffinity(evaluation)
        * allSkyZenithAffinity(evaluation);
}

static bool isBetterEvaluation(const Evaluation& candidate, const Evaluation& best)
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
                               double normalizationRadius = -1.0)
{
    if (normalizationRadius < 0.0) {
        normalizationRadius = m_weakModeNormalizationPixels;
    }
    if (!evaluation.valid) {
        return -std::numeric_limits<double>::infinity();
    }

    // Non-linear penalty proportional to the acceptance radius: a small RMS leaves the score
    // essentially equal to matchCount, but as RMS approaches the radius the per-match value
    // collapses to ~0.5 so a single false coincidence cannot outweigh a tighter cluster.
    double score = wideEvaluationMatchWeight(evaluation)
        * evaluationRmsQuality(evaluation, normalizationRadius)
        * brightnessAffinity(evaluation)
        * catalogMagnitudeAffinity(evaluation)
        * allSkyZenithAffinity(evaluation);

    if (m_useElevationSeedPreference)
    {
        const double safeElevationScale = std::max(1.0, m_elevationSeedScaleDegrees);
        const double safeFovScale = std::max(1.0, m_elevationSeedFovScaleDegrees);
        const double normalizedElevationDelta = std::fabs(
            evaluation.elevationDegrees - m_elevationSeedReferenceDegrees) / safeElevationScale;
        const double normalizedFovDelta = std::fabs(
            evaluation.fovDegrees - m_elevationSeedReferenceFovDegrees) / safeFovScale;
        const double seedAffinity = 1.0 / (1.0
            + 0.5 * normalizedElevationDelta * normalizedElevationDelta
            + 1.5 * normalizedFovDelta * normalizedFovDelta);
        score *= seedAffinity;
    }

    return score;
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

bool isBetterGuidedDirectionEvaluation(const Evaluation& candidate, const Evaluation& best)
{
    if (!candidate.valid) {
        return false;
    }
    if (!best.valid) {
        return true;
    }

    const bool candidateMeetsMinMatches = candidate.matchCount >= m_directionSeedMinMatchCount;
    const bool bestMeetsMinMatches = best.matchCount >= m_directionSeedMinMatchCount;
    if (candidateMeetsMinMatches != bestMeetsMinMatches) {
        return candidateMeetsMinMatches;
    }

    const double candidateScore = guidedDirectionEvaluationScore(candidate);
    const double bestScore = guidedDirectionEvaluationScore(best);
    if (std::fabs(candidateScore - bestScore) > 0.05) {
        return candidateScore > bestScore;
    }

    if (candidate.matchCount != best.matchCount) {
        return candidate.matchCount > best.matchCount;
    }
    if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
        return candidate.rmsErrorPixels < best.rmsErrorPixels;
    }
    if (!qFuzzyCompare(candidate.brightnessRankError + 1.0, best.brightnessRankError + 1.0)) {
        return candidate.brightnessRankError < best.brightnessRankError;
    }

    return isBetterEvaluation(candidate, best);
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
                                                const QVector<int>& detectionIndices,
                                                const Evaluation& candidate,
                                                double finalMatchRadius,
                                                bool restrictSupplementalMatchesToDetectionIndices = false)
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

    QVector<Match> allMatches;
    if (candidate.anchored)
    {
        int anchorProjectedIndex = -1;
        for (int i = 0; i < finalPass.projectedStars.size(); ++i)
        {
            if (finalPass.projectedStars[i].catalogIndex == candidate.anchorCatalogIndex)
            {
                anchorProjectedIndex = i;
                break;
            }
        }

        if ((anchorProjectedIndex >= 0)
            && (candidate.anchorDetectionIndex >= 0)
            && (candidate.anchorDetectionIndex < starDetections.size()))
        {
            const double anchorDistance = pointDistancePixels(
                starDetections[candidate.anchorDetectionIndex].m_center,
                finalPass.projectedStars[anchorProjectedIndex].point);
            if (anchorDistance <= finalMatchRadius)
            {
                const QVector<Match> automaticMatches = buildMatches(
                    catalogContext,
                    starDetections,
                    detectionIndices,
                    finalPass.projectedStars,
                    finalMatchRadius);
                allMatches.reserve(automaticMatches.size() + 1);
                allMatches.append({candidate.anchorDetectionIndex, candidate.anchorCatalogIndex, anchorDistance});
                QSet<int> matchedDetections;
                QSet<int> matchedCatalogStars;
                matchedDetections.insert(candidate.anchorDetectionIndex);
                matchedCatalogStars.insert(candidate.anchorCatalogIndex);
                for (const Match& match : automaticMatches)
                {
                    if (matchedDetections.contains(match.detectionIndex)
                        || matchedCatalogStars.contains(match.catalogIndex))
                    {
                        continue;
                    }
                    matchedDetections.insert(match.detectionIndex);
                    matchedCatalogStars.insert(match.catalogIndex);
                    allMatches.append(match);
                }
            }
        }
    }

    if (allMatches.isEmpty())
    {
        allMatches = buildMatches(
            catalogContext,
            starDetections,
            detectionIndices,
            finalPass.projectedStars,
            finalMatchRadius);
    }
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
        restrictSupplementalMatchesToDetectionIndices ? &detectionIndices : nullptr,
        finalPass.finalMatches);
    appendWideBrightSupplementalMatches(
        settings,
        starDetections,
        finalPass.projectedStars,
        imageSize,
        finalMatchRadius,
        finalPass.finalMatches);

    if (!finalPass.finalMatches.isEmpty())
    {
        finalPass.brightnessRankError = matchBrightnessRankError(
            starDetections,
            detectionIndices,
            finalPass.projectedStars,
            finalPass.finalMatches);
        finalPass.meanCatalogMagnitude = meanCatalogMagnitudeForMatches(
            catalogContext.catalogStars,
            finalPass.finalMatches);
        QSet<int> matchedCatalogIndices;
        matchedCatalogIndices.reserve(finalPass.finalMatches.size());
        QSet<int> matchedDetectionIndices;
        matchedDetectionIndices.reserve(finalPass.finalMatches.size());
        for (const Match& match : finalPass.finalMatches)
        {
            matchedCatalogIndices.insert(match.catalogIndex);
            matchedDetectionIndices.insert(match.detectionIndex);
        }
        QVector<int> brightDetectionIndices = detectionIndices;
        brightDetectionIndices.erase(
            std::remove_if(
                brightDetectionIndices.begin(),
                brightDetectionIndices.end(),
                [&starDetections](int detectionIndex) {
                    return (detectionIndex < 0)
                        || (detectionIndex >= starDetections.size())
                        || !isDetectionUsableForBrightPrior(starDetections[detectionIndex]);
                }),
            brightDetectionIndices.end());
        std::sort(brightDetectionIndices.begin(), brightDetectionIndices.end(), [this, &starDetections](int lhs, int rhs) {
            const double lhsBrightness = cachedDetectionBrightnessMetric(starDetections, lhs);
            const double rhsBrightness = cachedDetectionBrightnessMetric(starDetections, rhs);
            const double lhsReliability = cachedDetectionReliabilityMetric(starDetections, lhs);
            const double rhsReliability = cachedDetectionReliabilityMetric(starDetections, rhs);
            const double lhsImportance = lhsBrightness * std::sqrt(std::max(0.0, lhsReliability));
            const double rhsImportance = rhsBrightness * std::sqrt(std::max(0.0, rhsReliability));
            if (!qFuzzyCompare(lhsImportance + 1.0, rhsImportance + 1.0)) {
                return lhsImportance > rhsImportance;
            }
            return lhsBrightness > rhsBrightness;
        });
        const int brightDetectionLimit = std::min(8, static_cast<int>(brightDetectionIndices.size()));
        QHash<int, int> matchedCatalogByDetection;
        matchedCatalogByDetection.reserve(finalPass.finalMatches.size());
        for (const Match& match : finalPass.finalMatches) {
            matchedCatalogByDetection.insert(match.detectionIndex, match.catalogIndex);
        }
        double brightMagnitudePenalty = 0.0;
        double brightMagnitudeWeight = 0.0;
        for (int i = 0; i < brightDetectionLimit; ++i)
        {
            const int detectionIndex = brightDetectionIndices[i];
            ++finalPass.brightDetections;
            if (matchedDetectionIndices.contains(detectionIndex)) {
                ++finalPass.matchedBrightDetections;
            }
            const double rankWeight = (i == 0) ? 4.0 : (i < 3) ? 2.0 : 1.0;
            const auto catalogIt = matchedCatalogByDetection.constFind(detectionIndex);
            if (catalogIt == matchedCatalogByDetection.cend())
            {
                brightMagnitudePenalty += rankWeight * ((i == 0) ? 3.0 : (i < 3) ? 2.0 : 1.0);
                brightMagnitudeWeight += rankWeight;
                continue;
            }
            const int catalogIndex = catalogIt.value();
            const double catalogMagnitude = ((catalogIndex >= 0) && (catalogIndex < catalogContext.catalogStars.size()))
                ? catalogContext.catalogStars[catalogIndex].magnitude
                : std::numeric_limits<double>::infinity();
            const double expectedMaxMagnitude = (i == 0) ? 1.0 : (i < 3) ? 3.0 : 4.5;
            brightMagnitudePenalty += rankWeight * std::max(0.0, catalogMagnitude - expectedMaxMagnitude);
            brightMagnitudeWeight += rankWeight;
        }
        finalPass.brightDetectionMatchFraction = (finalPass.brightDetections > 0)
            ? static_cast<double>(finalPass.matchedBrightDetections) / static_cast<double>(finalPass.brightDetections)
            : 1.0;
        finalPass.brightDetectionMagnitudeError = (brightMagnitudeWeight > 0.0)
            ? brightMagnitudePenalty / brightMagnitudeWeight
            : 0.0;
        const QRectF brightProjectedBounds(
            -finalMatchRadius,
            -finalMatchRadius,
            imageSize.width() + 2.0 * finalMatchRadius,
            imageSize.height() + 2.0 * finalMatchRadius);
        QVector<ProjectedCatalogStar> brightProjectedStars;
        brightProjectedStars.reserve(16);
        for (const ProjectedCatalogStar& projectedStar : finalPass.projectedStars)
        {
            if ((projectedStar.magnitude > 5.0)
                || !brightProjectedBounds.contains(projectedStar.point))
            {
                continue;
            }
            brightProjectedStars.append(projectedStar);
        }
        std::sort(brightProjectedStars.begin(), brightProjectedStars.end(), [](const ProjectedCatalogStar& lhs, const ProjectedCatalogStar& rhs) {
            return lhs.magnitude < rhs.magnitude;
        });
        if (brightProjectedStars.size() > 8) {
            brightProjectedStars.resize(8);
        }
        for (const ProjectedCatalogStar& projectedStar : brightProjectedStars)
        {
            ++finalPass.brightProjectedStars;
            if (matchedCatalogIndices.contains(projectedStar.catalogIndex)) {
                ++finalPass.matchedBrightProjectedStars;
            }
        }
        finalPass.brightProjectedMatchFraction = (finalPass.brightProjectedStars > 0)
            ? static_cast<double>(finalPass.matchedBrightProjectedStars) / static_cast<double>(finalPass.brightProjectedStars)
            : 1.0;
        double maxError = 0.0;
        for (const Match& match : finalPass.finalMatches)
        {
            maxError = std::max(maxError, match.distancePixels);
        }
        finalPass.rmsErrorPixels = weightedRmsDistancePixels(starDetections, finalPass.finalMatches);
        finalPass.medianErrorPixels = medianDistancePixels(finalPass.finalMatches);
        finalPass.maxErrorPixels = maxError;
        finalPass.pose.meanCatalogMagnitude = finalPass.meanCatalogMagnitude;
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
        << " BrightnessErr=" << evaluation.brightnessRankError
        << " MeanMag=" << evaluation.meanCatalogMagnitude
        << " BrightDetections=" << evaluation.matchedBrightDetections << "/" << evaluation.brightDetections
        << " BrightMagErr=" << evaluation.brightDetectionMagnitudeError
        << " BrightProjected=" << evaluation.matchedBrightProjectedStars << "/" << evaluation.brightProjectedStars
        << " projectedStars=" << evaluation.projectedStars.size()
        << " K1=" << evaluation.pose.distortionK1;
}

bool isBetterFinalPassEvaluation(const Evaluation& candidate,
                                 const Evaluation& best,
                                 int retainedMatchThreshold,
                                 bool useGuidedDirectionScoring)
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
        const double comparableRms = std::min(
            m_weakModeNormalizationPixels,
            kWideFovMagnitudePreferenceMaxRmsPixels);
        if (shouldPreferBrighterCatalogMean(
                candidate.meanCatalogMagnitude,
                best.meanCatalogMagnitude,
                candidate.rmsErrorPixels,
                best.rmsErrorPixels,
                comparableRms))
        {
            return true;
        }
        if (shouldPreferBrighterCatalogMean(
                best.meanCatalogMagnitude,
                candidate.meanCatalogMagnitude,
                best.rmsErrorPixels,
                candidate.rmsErrorPixels,
                comparableRms))
        {
            return false;
        }

        const double candidateMedian = medianDistancePixels(candidate.matches);
        const double bestMedian = medianDistancePixels(best.matches);
        if (!qFuzzyCompare(candidateMedian + 1.0, bestMedian + 1.0)) {
            return candidateMedian < bestMedian;
        }
        if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
            return candidate.rmsErrorPixels < best.rmsErrorPixels;
        }
        if (!qFuzzyCompare(candidate.brightnessRankError + 1.0, best.brightnessRankError + 1.0)) {
            return candidate.brightnessRankError < best.brightnessRankError;
        }
        if (candidate.matchCount != best.matchCount) {
            return candidate.matchCount > best.matchCount;
        }
        return useGuidedDirectionScoring
            ? isBetterGuidedDirectionEvaluation(candidate, best)
            : isBetterEvaluation(candidate, best);
    }

    if (candidate.matchCount != best.matchCount) {
        return candidate.matchCount > best.matchCount;
    }
    if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
        return candidate.rmsErrorPixels < best.rmsErrorPixels;
    }

    return useGuidedDirectionScoring
        ? isBetterGuidedDirectionEvaluation(candidate, best)
        : isBetterEvaluation(candidate, best);
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

double wideFinalPassMatchWeight(const CameraSettings& settings,
                                int matchCount)
{
    if (!m_useWideCatalogMagnitudePreference) {
        return static_cast<double>(matchCount);
    }

    const int usefulMatchCap = std::max(settings.m_plateSolveMinMatches + 4, 8);
    const int cappedMatches = std::min(matchCount, usefulMatchCap);
    const int extraMatches = std::max(0, matchCount - usefulMatchCap);
    return static_cast<double>(cappedMatches) + 0.15 * std::log1p(static_cast<double>(extraMatches));
}

double wideFinalPassMagnitudeAffinity(const FinalMatchPassEvaluation& evaluation)
{
    if (!m_useWideCatalogMagnitudePreference
        || !std::isfinite(evaluation.meanCatalogMagnitude)
        || (evaluation.finalMatches.size() < 3))
    {
        return 1.0;
    }

    const double faintExcess = std::max(0.0, evaluation.meanCatalogMagnitude - 2.0);
    return 1.0 / (1.0 + 0.65 * faintExcess * faintExcess);
}

double brightDetectionCoverageAffinity(const CameraSettings& settings,
                                       const FinalMatchPassEvaluation& evaluation)
{
    if (!isWidePlateSolveContext(settings)
        || (evaluation.brightDetections < 3))
    {
        return 1.0;
    }

    const double coverage = std::clamp(evaluation.brightDetectionMatchFraction, 0.0, 1.0);
    return 0.20 + 0.80 * coverage * coverage;
}

double brightProjectedCoverageAffinity(const CameraSettings& settings,
                                       const FinalMatchPassEvaluation& evaluation)
{
    if (!isWidePlateSolveContext(settings)
        || (evaluation.brightProjectedStars < 3))
    {
        return 1.0;
    }

    const double coverage = std::clamp(evaluation.brightProjectedMatchFraction, 0.0, 1.0);
    return 0.10 + 0.90 * coverage * coverage;
}

double brightDetectionMagnitudeAffinity(const CameraSettings& settings,
                                        const FinalMatchPassEvaluation& evaluation)
{
    if (!isWidePlateSolveContext(settings)
        || (evaluation.brightDetections < 3))
    {
        return 1.0;
    }

    const double error = std::max(0.0, evaluation.brightDetectionMagnitudeError);
    return 1.0 / (1.0 + 4.0 * error * error);
}

double finalMatchPassScore(const CameraSettings& settings,
                           const FinalMatchPassEvaluation& evaluation)
{
    if (!evaluation.projectorValid) {
        return -std::numeric_limits<double>::infinity();
    }

    Evaluation pose = evaluation.pose;
    pose.matchCount = evaluation.finalMatches.size();
    pose.matches = evaluation.finalMatches;
    pose.rmsErrorPixels = evaluation.rmsErrorPixels;
    pose.brightnessRankError = evaluation.brightnessRankError;
    pose.meanCatalogMagnitude = evaluation.meanCatalogMagnitude;

    const double normalizationRadius = std::min(
        std::max(1.0, m_weakModeNormalizationPixels),
        std::max(1.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius)));
    return wideFinalPassMatchWeight(settings, evaluation.finalMatches.size())
        * evaluationRmsQuality(pose, normalizationRadius)
        * brightnessAffinity(pose)
        * wideFinalPassMagnitudeAffinity(evaluation)
        * brightDetectionCoverageAffinity(settings, evaluation)
        * brightProjectedCoverageAffinity(settings, evaluation)
        * brightDetectionMagnitudeAffinity(settings, evaluation)
        * directionSeedAffinity(pose)
        * fovSeedAffinity(pose)
        * allSkyZenithAffinity(pose);
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

    if (candidateMeetsMinMatches != bestMeetsMinMatches) {
        return candidateMeetsMinMatches;
    }

    if (isWidePlateSolveContext(settings))
    {
        const bool candidateBrightAnchorAccepted =
            hasAcceptableWideBrightAnchorSupport(settings, starDetections, candidate);
        const bool bestBrightAnchorAccepted =
            hasAcceptableWideBrightAnchorSupport(settings, starDetections, best);
        if (candidateBrightAnchorAccepted != bestBrightAnchorAccepted) {
            return candidateBrightAnchorAccepted;
        }
    }

    if (blindMode && (candidateBlindAccepted != bestBlindAccepted)) {
        return candidateBlindAccepted;
    }

    if (m_useDirectionSeedPreference)
    {
        const bool candidateDirectionAccepted = candidateMeetsMinMatches
            && isAcceptableDirectionSeedSolve(
                settings,
                starDetections,
                candidate.finalMatches,
                candidate.rmsErrorPixels,
                candidate.maxErrorPixels)
            && hasAcceptableGuidedFinalBrightnessConsistency(settings, candidate);
        const bool bestDirectionAccepted = bestMeetsMinMatches
            && isAcceptableDirectionSeedSolve(
                settings,
                starDetections,
                best.finalMatches,
                best.rmsErrorPixels,
                best.maxErrorPixels)
            && hasAcceptableGuidedFinalBrightnessConsistency(settings, best);
        if (candidateDirectionAccepted != bestDirectionAccepted) {
            return candidateDirectionAccepted;
        }

        const bool candidateBrightnessAccepted = hasAcceptableGuidedFinalBrightnessConsistency(settings, candidate);
        const bool bestBrightnessAccepted = hasAcceptableGuidedFinalBrightnessConsistency(settings, best);
        if (candidateBrightnessAccepted != bestBrightnessAccepted) {
            return candidateBrightnessAccepted;
        }
    }

    if (isWidePlateSolveContext(settings))
    {
        const double projectedCoverageDelta = candidate.brightProjectedMatchFraction - best.brightProjectedMatchFraction;
        if ((candidate.brightProjectedStars >= 3)
            && (best.brightProjectedStars >= 3)
            && (std::fabs(projectedCoverageDelta) >= 0.25))
        {
            return projectedCoverageDelta > 0.0;
        }

        const double magnitudeErrorDelta = candidate.brightDetectionMagnitudeError - best.brightDetectionMagnitudeError;
        if ((candidate.brightDetections >= 3)
            && (best.brightDetections >= 3)
            && (std::fabs(magnitudeErrorDelta) >= 0.15))
        {
            return magnitudeErrorDelta < 0.0;
        }

        const double coverageDelta = candidate.brightDetectionMatchFraction - best.brightDetectionMatchFraction;
        if ((candidate.brightDetections >= 3)
            && (best.brightDetections >= 3)
            && (std::fabs(coverageDelta) >= 0.25))
        {
            return coverageDelta > 0.0;
        }
    }

    if (m_useWideCatalogMagnitudePreference || m_useDirectionSeedPreference)
    {
        const double candidateScore = finalMatchPassScore(settings, candidate);
        const double bestScore = finalMatchPassScore(settings, best);
        if (std::fabs(candidateScore - bestScore) > 0.05) {
            return candidateScore > bestScore;
        }
    }

    const int finalMatchDelta = static_cast<int>(candidate.finalMatches.size()) - static_cast<int>(best.finalMatches.size());
    if (std::abs(finalMatchDelta) <= 1)
    {
        const double comparableRms = std::min(
            static_cast<double>(settings.m_plateSolveFinalMatchRadius),
            kWideFovMagnitudePreferenceMaxRmsPixels);
        if (shouldPreferBrighterCatalogMean(
                candidate.meanCatalogMagnitude,
                best.meanCatalogMagnitude,
                candidate.rmsErrorPixels,
                best.rmsErrorPixels,
                comparableRms))
        {
            return true;
        }
        if (shouldPreferBrighterCatalogMean(
                best.meanCatalogMagnitude,
                candidate.meanCatalogMagnitude,
                best.rmsErrorPixels,
                candidate.rmsErrorPixels,
                comparableRms))
        {
            return false;
        }

        if (!qFuzzyCompare(candidate.medianErrorPixels + 1.0, best.medianErrorPixels + 1.0)) {
            return candidate.medianErrorPixels < best.medianErrorPixels;
        }
        if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
            return candidate.rmsErrorPixels < best.rmsErrorPixels;
        }
        if (!qFuzzyCompare(candidate.brightnessRankError + 1.0, best.brightnessRankError + 1.0)) {
            return candidate.brightnessRankError < best.brightnessRankError;
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
                               bool useWeakModeScoring,
                               bool useGuidedDirectionScoring = false)
{
    if (useGuidedDirectionScoring) {
        return isBetterGuidedDirectionEvaluation(candidate, best);
    }
    if (useWeakModeScoring) {
        return isBetterWeakModeEvaluation(candidate, best);
    }
    return isBetterEvaluation(candidate, best);
}

double guidedAnchorEvaluationScore(const Evaluation& evaluation,
                                   double matchRadiusPixels)
{
    if (!evaluation.valid || evaluation.matches.isEmpty()) {
        return -std::numeric_limits<double>::infinity();
    }

    const double anchorDistance = evaluation.matches.first().distancePixels;
    const double safeRadius = std::max(1.0, matchRadiusPixels);
    const double anchorRadius = std::max(4.0, std::min(24.0, safeRadius));
    const double anchorAffinity = 1.0 / (1.0 + std::pow(anchorDistance / anchorRadius, 2.0));
    const double rmsAffinity = evaluationRmsQuality(evaluation, safeRadius);

    return (wideEvaluationMatchWeight(evaluation) * 0.6 + 6.0 * anchorAffinity)
        * rmsAffinity
        * brightnessAffinity(evaluation)
        * catalogMagnitudeAffinity(evaluation)
        * directionSeedAffinity(evaluation)
        * fovSeedAffinity(evaluation);
}

double guidedAnchorSearchScore(const Evaluation& evaluation,
                               const GuidedAnchorPair& anchor,
                               double matchRadiusPixels)
{
    if (!evaluation.valid) {
        return -std::numeric_limits<double>::infinity();
    }

    const double directPenalty = std::pow(anchor.initialDistancePixels / 48.0, 2.0);
    const double radialPenalty = std::pow(anchor.radialErrorPixels / 96.0, 2.0);
    const double reliabilityBonus = std::min(1.5, std::log1p(anchor.detectionReliability) * 0.15);
    return guidedAnchorEvaluationScore(evaluation, matchRadiusPixels)
        - directPenalty
        - radialPenalty
        + reliabilityBonus;
}

bool isBetterGuidedAnchorEvaluation(const Evaluation& candidate,
                                    const Evaluation& best,
                                    double matchRadiusPixels)
{
    if (!candidate.valid) {
        return false;
    }
    if (!best.valid) {
        return true;
    }

    const double candidateScore = guidedAnchorEvaluationScore(candidate, matchRadiusPixels);
    const double bestScore = guidedAnchorEvaluationScore(best, matchRadiusPixels);
    if (std::fabs(candidateScore - bestScore) > 0.05) {
        return candidateScore > bestScore;
    }

    const double candidateAnchorDistance = candidate.matches.isEmpty()
        ? std::numeric_limits<double>::infinity()
        : candidate.matches.first().distancePixels;
    const double bestAnchorDistance = best.matches.isEmpty()
        ? std::numeric_limits<double>::infinity()
        : best.matches.first().distancePixels;
    const double anchorRadius = std::max(4.0, std::min(24.0, matchRadiusPixels));
    const bool candidateAnchorTight = candidateAnchorDistance <= anchorRadius;
    const bool bestAnchorTight = bestAnchorDistance <= anchorRadius;
    if (candidateAnchorTight != bestAnchorTight) {
        return candidateAnchorTight;
    }
    if (!qFuzzyCompare(candidateAnchorDistance + 1.0, bestAnchorDistance + 1.0)) {
        return candidateAnchorDistance < bestAnchorDistance;
    }
    if (candidate.matchCount != best.matchCount) {
        return candidate.matchCount > best.matchCount;
    }
    if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
        return candidate.rmsErrorPixels < best.rmsErrorPixels;
    }
    return candidate.meanCatalogMagnitude < best.meanCatalogMagnitude;
}

Evaluation searchGuidedAnchorPose(const CameraSettings& settings,
                                  const PlateSolveCatalogContext& catalogContext,
                                  const QSize& imageSize,
                                  const QDateTime& captureDateTimeUtc,
                                  const QVector<CameraPipelineStarDetection>& starDetections,
                                  const QVector<int>& detectionIndices,
                                  QVector<Evaluation> *candidatePool = nullptr)
{
    Evaluation best;
    const bool useStartDirection = plateSolveStartUsesDirection(settings);
    const bool useStartRoll = plateSolveStartUsesRoll(settings);
    const bool useStartLens = plateSolveStartUsesLens(settings);
    const bool useWidePlateSolve = isWidePlateSolveContext(settings);
    const bool useDenseWideGuidedDirection = useStartDirection
        && useWidePlateSolve
        && (starDetections.size() > 32);
    const bool useWideWeakAnchorSearch = !plateSolveStartUsesDirection(settings)
        && useWidePlateSolve;
    if ((!useStartDirection && !useWideWeakAnchorSearch)
        || starDetections.isEmpty()
        || catalogContext.visibleStars.isEmpty())
    {
        return best;
    }

    const double localRadiusDegrees = std::max(
        static_cast<double>(settings.m_plateSolveSearchRadius) + static_cast<double>(settings.m_fov) * 2.0,
        static_cast<double>(settings.m_fov) * 4.0);
    const QVector<VisibleCatalogStar> localVisibleStars = selectLocalVisibleStars(
        catalogContext.visibleStars,
        settings.m_azimuth,
        settings.m_elevation,
        localRadiusDegrees,
        2048);
    if (localVisibleStars.isEmpty()) {
        return best;
    }

    const QVector<GuidedAnchorPair> anchors = findGuidedAnchorPairs(
        settings,
        catalogContext,
        imageSize,
        starDetections,
        localVisibleStars);
    if (anchors.isEmpty()) {
        return best;
    }

    QVector<int> allowedCatalogIndices;
    allowedCatalogIndices.reserve(localVisibleStars.size());
    for (const VisibleCatalogStar& star : localVisibleStars) {
        allowedCatalogIndices.append(star.catalogIndex);
    }

    const double fixedCenterOffsetX = useStartLens ? settings.m_lensCenterOffsetX : 0.0;
    const double fixedCenterOffsetY = useStartLens ? settings.m_lensCenterOffsetY : 0.0;
    const double fixedDistortionK1 = useStartLens ? settings.m_lensDistortionK1 : 0.0;
    const bool calibrateLens = canCalibrateLens(settings);
    QVector<QPointF> centerOffsetSeeds;
    centerOffsetSeeds.append(QPointF(fixedCenterOffsetX, fixedCenterOffsetY));
    QVector<double> distortionSeeds;
    distortionSeeds.append(fixedDistortionK1);
    if (calibrateLens && useWidePlateSolve)
    {
        const double xStep = imageSize.width() * 0.08;
        const double yStep = imageSize.height() * 0.08;
        const std::array<QPointF, 4> coarseOffsets = {{
            QPointF(-xStep, 0.0),
            QPointF(xStep, 0.0),
            QPointF(0.0, -yStep),
            QPointF(0.0, yStep)
        }};
        for (const QPointF& offset : coarseOffsets) {
            centerOffsetSeeds.append(centerOffsetSeeds.first() + offset);
        }
    }
    const double finalMatchRadius = std::max(1.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius));
    const double anchorMatchRadiusMultiplier = (useStartDirection && useWidePlateSolve)
        ? 3.0
        : 4.0;
    const double anchorMatchRadius = std::max(finalMatchRadius,
        std::min(128.0, std::max(finalMatchRadius, static_cast<double>(settings.m_plateSolveMatchRadius)) * anchorMatchRadiusMultiplier));
    const double minimumFovStep = std::max(0.02, std::min(0.5, static_cast<double>(settings.m_fov) * 0.02));
    const std::array<double, 3> offsets = {{-1.0, 0.0, 1.0}};
    const QVector<double> primaryRollSeedOffsets = useStartRoll
        ? QVector<double>{0.0, -5.0, 5.0, -10.0, 10.0}
        : QVector<double>{0.0, -5.0, 5.0, -10.0, 10.0, -20.0, 20.0, -35.0, 35.0};
    const QVector<double> expandedRollSeedOffsets = {
        -30.0, 30.0, -45.0, 45.0, -60.0, 60.0, -90.0, 90.0,
        -120.0, 120.0, -150.0, 150.0, -180.0, 180.0
    };
    // Direction-seeded anchor refinement already sweeps FoV locally from the 1.0x seed,
    // so extra scale seeds only multiply the expensive anchor search.
    const QVector<double> fovSeedScales = useStartDirection
        ? QVector<double>{1.0}
        : QVector<double>{0.88, 0.96, 1.0, 1.04, 1.12};
    const int anchorLimit = std::min(
        useWideWeakAnchorSearch ? 32
            : useDenseWideGuidedDirection ? 8
            : (useStartDirection && useWidePlateSolve && (starDetections.size() <= 16)) ? 4
            : 12,
        static_cast<int>(anchors.size()));
    const int expandedRollMinMatches = std::max(3, settings.m_plateSolveMinMatches - 1);
    const int refinementIterations = useDenseWideGuidedDirection ? 4 : 5;
    double bestSearchScore = -std::numeric_limits<double>::infinity();

    if (kLogPlateSolveCandidates)
    {
        qDebug() << "CameraPlateSolver: guided anchor search"
                 << "anchors" << anchors.size()
                 << "using" << anchorLimit
                 << "localStars" << localVisibleStars.size()
                 << "radius" << localRadiusDegrees;
        for (int i = 0; i < anchorLimit; ++i)
        {
            const GuidedAnchorPair& anchor = anchors[i];
            const QString anchorName = ((anchor.catalogIndex >= 0) && (anchor.catalogIndex < catalogContext.catalogStars.size()))
                ? catalogDisplayName(catalogContext.catalogStars[anchor.catalogIndex])
                : QString();
            qDebug() << "CameraPlateSolver: guided anchor candidate"
                     << i
                     << anchorName
                     << "catalog" << anchor.catalogIndex
                     << "detection" << anchor.detectionIndex
                     << "mag" << anchor.magnitude
                     << "radial" << anchor.radialErrorPixels
                     << "direct" << anchor.initialDistancePixels
                     << "roll" << anchor.estimatedRollDegrees;
        }
    }

    auto evaluateRollSeedOffsets = [&](const QVector<double>& rollSeedOffsets)
    {
        for (int anchorIndex = 0; anchorIndex < anchorLimit; ++anchorIndex)
        {
            const GuidedAnchorPair& anchor = anchors[anchorIndex];
            const auto anchorVisibleIt = catalogContext.visibleStarIndexByCatalogIndex.constFind(anchor.catalogIndex);
            if (anchorVisibleIt == catalogContext.visibleStarIndexByCatalogIndex.cend()) {
                continue;
            }
            const SkyVector anchorVector = catalogContext.visibleStars[*anchorVisibleIt].vector;
            QVector<int> anchoredDetectionIndices = detectionIndices;
            if (!anchoredDetectionIndices.contains(anchor.detectionIndex)) {
                anchoredDetectionIndices.append(anchor.detectionIndex);
            }

            for (double fovScale : fovSeedScales)
            {
                for (double rollSeedOffset : rollSeedOffsets)
                {
                    const double seedFov = std::clamp(static_cast<double>(settings.m_fov) * fovScale,
                        static_cast<double>(CameraSettings::m_minFov),
                        static_cast<double>(CameraSettings::m_maxFov));
                    const double baseSeedRoll = anchor.estimatedRollDegrees + rollSeedOffset;

                    auto evaluateAnchorSeed = [&](double seedAzimuth,
                                                  double seedElevation,
                                                  double seedRoll,
                                                  double seedCenterOffsetX,
                                                  double seedCenterOffsetY,
                                                  double seedDistortionK1)
                    {
                        Evaluation localBest = evaluateAnchoredPose(
                            settings,
                            catalogContext,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            anchoredDetectionIndices,
                            allowedCatalogIndices,
                            anchor,
                            seedAzimuth,
                            seedElevation,
                            seedRoll,
                            seedFov,
                            seedCenterOffsetX,
                            seedCenterOffsetY,
                            seedDistortionK1,
                            anchorMatchRadius);
                        if (!localBest.valid) {
                            return;
                        }

                        double azCenter = localBest.azimuthDegrees;
                        double elCenter = localBest.elevationDegrees;
                        double rollCenter = localBest.rollDegrees;
                        double fovCenter = localBest.fovDegrees;
                        double azStep = std::max(0.04, static_cast<double>(settings.m_plateSolveSearchRadius) * 0.20);
                        double elStep = azStep;
                        double rollStep = 6.0;
                        // For narrow-field direction-seeded solves, hold FOV fixed at the seed value
                        // (which is exactly settings.m_fov for the single 1.0× seed).
                        double fovStep = (settings.m_fov <= 5.0)
                            ? 0.0
                            : std::max(minimumFovStep, static_cast<double>(settings.m_fov) * 0.05);

                        for (int iteration = 0; iteration < refinementIterations; ++iteration)
                        {
                            bool improved = false;
                            for (double azOffset : offsets)
                            {
                                for (double elOffset : offsets)
                                {
                                    for (double rollOffset : offsets)
                                    {
                                        const int fovOffsetCount = fovStep > 0.0 ? static_cast<int>(offsets.size()) : 1;
                                        for (int fovOffsetIndex = 0; fovOffsetIndex < fovOffsetCount; ++fovOffsetIndex)
                                        {
                                            const double fovOffset = fovStep > 0.0 ? offsets[fovOffsetIndex] : 0.0;
                                            const Evaluation candidate = evaluateAnchoredPose(
                                                settings,
                                                catalogContext,
                                                imageSize,
                                                captureDateTimeUtc,
                                                starDetections,
                                                anchoredDetectionIndices,
                                                allowedCatalogIndices,
                                                anchor,
                                                azCenter + azOffset * azStep,
                                                elCenter + elOffset * elStep,
                                                rollCenter + rollOffset * rollStep,
                                                std::clamp(fovCenter + fovOffset * fovStep,
                                                    static_cast<double>(CameraSettings::m_minFov),
                                                    static_cast<double>(CameraSettings::m_maxFov)),
                                                seedCenterOffsetX,
                                                seedCenterOffsetY,
                                                seedDistortionK1,
                                                anchorMatchRadius);
                                            if (isBetterGuidedAnchorEvaluation(candidate, localBest, anchorMatchRadius))
                                            {
                                                localBest = candidate;
                                                improved = true;
                                            }
                                        }
                                    }
                                }
                            }

                            azCenter = localBest.azimuthDegrees;
                            elCenter = localBest.elevationDegrees;
                            rollCenter = localBest.rollDegrees;
                            fovCenter = localBest.fovDegrees;
                            if (!improved)
                            {
                                azStep *= 0.5;
                                elStep *= 0.5;
                                rollStep *= 0.5;
                                fovStep *= 0.5;
                            }
                        }

                        logPlateSolveEvaluation("guided-anchor", localBest);
                        if (candidatePool && localBest.valid && (localBest.matchCount >= 2)) {
                            candidatePool->append(localBest);
                        }
                        const double localSearchScore = guidedAnchorSearchScore(localBest, anchor, anchorMatchRadius);
                        if ((localSearchScore > bestSearchScore)
                            || (std::fabs(localSearchScore - bestSearchScore) <= 0.05
                                && isBetterGuidedAnchorEvaluation(localBest, best, anchorMatchRadius)))
                        {
                            bestSearchScore = localSearchScore;
                            best = localBest;
                            logPlateSolveEvaluation("guided-anchor", best, true);
                        }
                    };

                    const bool tryCoarseLensSeeds = (anchorIndex < 6)
                        && (anchor.magnitude <= 3.0)
                        && calibrateLens
                        && useWidePlateSolve
                        && (!useStartDirection || useStartLens);
                    const int centerSeedCount = tryCoarseLensSeeds ? centerOffsetSeeds.size() : 1;
                    for (int centerSeedIndex = 0; centerSeedIndex < centerSeedCount; ++centerSeedIndex)
                    {
                        const QPointF centerOffsetSeed = centerOffsetSeeds[centerSeedIndex];
                        for (double distortionSeed : distortionSeeds)
                        {
                            if (centerSeedIndex == 0)
                            {
                                evaluateAnchorSeed(
                                    settings.m_azimuth,
                                    settings.m_elevation,
                                    baseSeedRoll,
                                    centerOffsetSeed.x(),
                                    centerOffsetSeed.y(),
                                    distortionSeed);
                            }

                            double alignedAzimuth = settings.m_azimuth;
                            double alignedElevation = settings.m_elevation;
                            double alignedRoll = baseSeedRoll;
                            if (anchorAlignedPoseFromPixel(
                                    settings,
                                    imageSize,
                                    starDetections[anchor.detectionIndex].m_center,
                                    anchorVector,
                                    settings.m_azimuth,
                                    settings.m_elevation,
                                    baseSeedRoll,
                                    seedFov,
                                    centerOffsetSeed.x(),
                                    centerOffsetSeed.y(),
                                    distortionSeed,
                                    alignedAzimuth,
                                    alignedElevation,
                                    alignedRoll))
                            {
                                evaluateAnchorSeed(
                                    alignedAzimuth,
                                    alignedElevation,
                                    alignedRoll,
                                    centerOffsetSeed.x(),
                                    centerOffsetSeed.y(),
                                    distortionSeed);
                            }
                        }
                    }
                }
            }
        }
    };

    evaluateRollSeedOffsets(primaryRollSeedOffsets);
    if (!best.valid || (best.matchCount < expandedRollMinMatches)) {
        evaluateRollSeedOffsets(expandedRollSeedOffsets);
    }

    return best;
}

static bool sameEvaluationBasin(const Evaluation& lhs, const Evaluation& rhs)
{
    if (lhs.anchored != rhs.anchored) {
        return false;
    }
    if (lhs.anchored
        && ((lhs.anchorDetectionIndex != rhs.anchorDetectionIndex)
            || (lhs.anchorCatalogIndex != rhs.anchorCatalogIndex)))
    {
        return false;
    }

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
                                       int minPoolMatchCount = 3,
                                       bool useGuidedDirectionScoring = false)
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
    const double poolQualityRadius = (candidate.anchored && useGuidedDirectionScoring)
        ? std::max(2.0, m_weakModeNormalizationPixels * 4.0)
        : std::max(2.0, m_weakModeNormalizationPixels * 0.95);
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
            if (isBetterEvaluationForMode(candidate, existing, useWeakModeScoring, useGuidedDirectionScoring)) {
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
        if (!isBetterEvaluationForMode(candidate, weakestCandidate, useWeakModeScoring, useGuidedDirectionScoring))
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
    std::sort(candidates.begin(), candidates.end(), [this, useWeakModeScoring, useGuidedDirectionScoring](const Evaluation& lhs, const Evaluation& rhs) {
        return isBetterEvaluationForMode(lhs, rhs, useWeakModeScoring, useGuidedDirectionScoring);
    });
    bool candidateKept = true;
    if (candidates.size() > maxCandidates) {
        candidates.resize(maxCandidates);
        candidateKept = std::any_of(candidates.cbegin(), candidates.cend(), [this, &candidate](const Evaluation& existing) {
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
        << " BrightnessErr=" << evaluation.brightnessRankError
        << " MeanMag=" << evaluation.meanCatalogMagnitude
        << " GuidedScore=" << guidedDirectionEvaluationScore(evaluation)
        << " Cx=" << evaluation.centerOffsetXPixels
        << " Cy=" << evaluation.centerOffsetYPixels
        << " K1=" << evaluation.distortionK1;
}

static bool sameEvaluationIdentity(const Evaluation& lhs, const Evaluation& rhs)
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
                             const Evaluation *other = nullptr)
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
        << " BrightnessErr=" << candidate.brightnessRankError
        << " MeanMag=" << candidate.meanCatalogMagnitude
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
            << " MeanMag=" << other->meanCatalogMagnitude
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
            << " BrightnessErr=" << candidate.brightnessRankError
            << " MeanMag=" << candidate.meanCatalogMagnitude
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
    const bool profilePlateSolve = qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_PROFILE");
    QElapsedTimer searchProfileTimer;
    searchProfileTimer.start();
    auto logSearchProfile = [&](const char *stage, qint64 startedMs) {
        const qint64 elapsedMs = searchProfileTimer.elapsed() - startedMs;
        recordProfileTiming(QStringLiteral("search.%1").arg(QString::fromUtf8(stage)), elapsedMs);
        if (!profilePlateSolve) {
            return;
        }
        qDebug().noquote().nospace()
            << "CameraPlateSolverProfile search." << stage
            << " elapsedMs=" << elapsedMs
            << " totalMs=" << searchProfileTimer.elapsed()
            << " bestValid=" << best.valid
            << " bestMatches=" << best.matchCount
            << " bestRms=" << best.rmsErrorPixels
            << " pool=" << (candidatePool ? candidatePool->size() : 0);
    };
    const int minMatchCount = std::max(1, settings.m_plateSolveMinMatches);
    const bool useStartFov = plateSolveStartUsesFov(settings);
    const bool useStartElevation = plateSolveStartUsesElevation(settings);
    const bool useStartDirection = plateSolveStartUsesDirection(settings);
    const bool useStartRoll = plateSolveStartUsesRoll(settings);
    const bool useElevationSeedOnly = useStartElevation && !useStartDirection;
    const bool useStartLens = plateSolveStartUsesLens(settings);
    const bool useWeakModeScoring = !useStartDirection;
    const bool useGuidedDirectionScoring = useStartDirection;
    const bool keepMultipleCandidates = candidatePool != nullptr;
    const bool wideWeakMode = useWeakModeScoring && isWidePlateSolveContext(settings);
    const bool useWideFovBlindSeedFirst = wideWeakMode
        && useStartFov
        && !useStartElevation
        && !useStartDirection;
    const int maxMultiHypothesisCandidates = (useStartDirection && (settings.m_fov <= 5.0)) ? 24
        : wideWeakMode ? 64
        : 10;
    const int interestingWeakModeMatchCount = std::max(3, minMatchCount - 1);
    const int weakModeCandidatePoolMinMatches = std::max(3, minMatchCount - 2);
    const double fixedCenterOffsetX = useStartLens ? settings.m_lensCenterOffsetX : 0.0;
    const double fixedCenterOffsetY = useStartLens ? settings.m_lensCenterOffsetY : 0.0;
    const double fixedDistortionK1 = useStartLens ? settings.m_lensDistortionK1 : 0.0;

    const double finalMatchRadius = std::max(1.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius));
    const double maxImageDimension = std::max(imageSize.width(), imageSize.height());
    const bool useWideFovSeedRadius = useStartFov
        && !useStartElevation
        && !useStartDirection
        && (settings.m_fov >= 120.0)
        && (settings.m_lensProjection != CameraSettings::LensProjectionRectilinear);
    const bool useWideBlindSeedRadius = !useStartFov
        && !useStartElevation
        && !useStartDirection
        && isWidePlateSolveContext(settings);
    const double wideFovSeedMatchRadius = useWideFovSeedRadius
        ? std::max(finalMatchRadius, std::min(240.0, std::max(120.0, maxImageDimension * 0.065)))
        : finalMatchRadius;
    const double wideBlindSeedMatchRadius = useWideBlindSeedRadius
        ? std::max(finalMatchRadius, std::min(240.0, std::max(120.0, maxImageDimension * 0.065)))
        : finalMatchRadius;
    const double guidedSeedMatchRadius = (useStartDirection && (settings.m_fov <= 5.0))
        ? std::max(finalMatchRadius,
            std::min(96.0, std::max(finalMatchRadius, static_cast<double>(settings.m_plateSolveMatchRadius)) * 4.0))
        : finalMatchRadius;
    const double searchMatchRadiusOverride = useStartDirection
        ? guidedSeedMatchRadius
        : useWideFovSeedRadius ? wideFovSeedMatchRadius
        : -1.0;
    if (useStartDirection && (settings.m_fov <= 5.0)) {
        m_weakModeNormalizationPixels = std::max(m_weakModeNormalizationPixels, guidedSeedMatchRadius);
    }
    if (useWideFovSeedRadius) {
        m_weakModeNormalizationPixels = std::max(m_weakModeNormalizationPixels, wideFovSeedMatchRadius);
    }
    if (useWideBlindSeedRadius) {
        m_weakModeNormalizationPixels = std::max(m_weakModeNormalizationPixels, wideBlindSeedMatchRadius);
    }
    const double coarseSearchRadius = std::max(0.0, settings.m_plateSolveSearchRadius);
    const double coarseRollRadius = std::max(4.0, std::min(20.0, static_cast<double>(settings.m_fov) * 0.20));
    const double coarseFovRadius = std::max(0.05, std::min(12.0, static_cast<double>(settings.m_fov) * 0.10));
    const double minimumFovRefineStep = std::max(0.02, std::min(0.5, static_cast<double>(settings.m_fov) * 0.02));

    const double minAzimuthDegrees = 0.0;
    const double maxAzimuthDegrees = 360.0;
    const double azimuthStepDegrees = 5.0;
    const double minElevationDegrees = 0.0;
    const double maxElevationDegrees = 90.0;
    const double elevationStepDegrees = 15.0;
    const double fovGridStepDegrees = std::max(0.25, std::min(5.0, static_cast<double>(settings.m_fov) * 0.5));
    const double fovGridAzimuthStepDegrees = fovGridStepDegrees;
    const double fovGridElevationStepDegrees = std::max(0.25, std::min(15.0, static_cast<double>(settings.m_fov) * 0.5));

    const std::array<double, 3> coarseFovOffsets = {{-1.0, 0.0, 1.0}};
    const std::array<double, 5> coarseOffsetsOrdered = {{0.0, -0.5, 0.5, -1.0, 1.0}};
    const std::array<double, 3> coarseFovOffsetsOrdered = {{0.0, -1.0, 1.0}};
    // -180° and +180° are numerically identical rotations; keep only -180° to avoid
    // duplicating ~7.7% of blind-grid evaluations.
    const std::array<double, 12> wideRollOffsets = {{-180.0, -150.0, -120.0, -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0, 120.0, 150.0}};
    const std::array<double, 12> wideRollOffsetsOrdered = {{0.0, -30.0, 30.0, -60.0, 60.0, -90.0, 90.0, -120.0, 120.0, -150.0, 150.0, -180.0}};
    const std::array<double, 24> wideRollOffsetsFineOrdered = {{
        0.0, -15.0, 15.0, -30.0, 30.0, -45.0, 45.0, -60.0, 60.0,
        -75.0, 75.0, -90.0, 90.0, -105.0, 105.0, -120.0, 120.0,
        -135.0, 135.0, -150.0, 150.0, -165.0, 165.0, -180.0
    }};
    QVector<double> fovSearchRollOffsets;
    if (wideWeakMode && useStartFov && !useStartElevation && !useStartDirection) {
        fovSearchRollOffsets.reserve(static_cast<int>(wideRollOffsetsFineOrdered.size()));
        for (double rollOffset : wideRollOffsetsFineOrdered) {
            fovSearchRollOffsets.append(rollOffset);
        }
    } else {
        fovSearchRollOffsets.reserve(static_cast<int>(wideRollOffsetsOrdered.size()));
        for (double rollOffset : wideRollOffsetsOrdered) {
            fovSearchRollOffsets.append(rollOffset);
        }
    }
    QVector<int> wideFirstPassCatalogIndices;
    if (wideWeakMode && useStartFov)
    {
        wideFirstPassCatalogIndices.reserve(std::min(192, static_cast<int>(catalogContext.visibleStars.size())));
        for (const VisibleCatalogStar& star : catalogContext.visibleStars)
        {
            if ((star.magnitude <= kWideFovBrightFirstPassMaxMagnitude)
                || (wideFirstPassCatalogIndices.size() < 96))
            {
                wideFirstPassCatalogIndices.append(star.catalogIndex);
            }
            if (wideFirstPassCatalogIndices.size() >= 192) {
                break;
            }
        }
    }

    auto evaluateSeed = [&](const char *stage,
                            double azimuthDegrees,
                            double elevationDegrees,
                            double rollDegrees,
                            double fovDegrees,
                            double matchRadiusOverride = -1.0,
                            const QVector<int>* allowedCatalogIndices = nullptr) {
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
            allowedCatalogIndices,
            fixedCenterOffsetX,
            fixedCenterOffsetY,
            fixedDistortionK1,
            matchRadiusOverride);
        logPlateSolveEvaluation(stage, candidate);
        if (keepMultipleCandidates) {
            insertDistinctEvaluationCandidate(
                *candidatePool,
                candidate,
                maxMultiHypothesisCandidates,
                useWeakModeScoring,
                stage,
                interestingWeakModeMatchCount,
                weakModeCandidatePoolMinMatches,
                useGuidedDirectionScoring);
        }
        if (isBetterEvaluationForMode(candidate, best, useWeakModeScoring, useGuidedDirectionScoring)) {
            best = candidate;
            logPlateSolveEvaluation(stage, best, true);
        }
    };

    // Lightweight variant used by the blind-grid roll sweep.  The projected
    // catalog is already stored in m_blindGridProjectedScratch — no
    // createProjector or buildProjectedCatalogInto needed.
    auto evaluateSeedFromCache = [&](const char *stage,
                                     double azimuthDegrees,
                                     double elevationDegrees,
                                     double rollDegrees,
                                     double fovDegrees,
                                     double matchRadiusPixels) {
        const Evaluation candidate = evaluatePoseFromPrecomputedCatalog(
            settings, catalogContext, starDetections, detectionIndices,
            azimuthDegrees, elevationDegrees, rollDegrees, fovDegrees,
            fixedCenterOffsetX, fixedCenterOffsetY, fixedDistortionK1,
            matchRadiusPixels);
        logPlateSolveEvaluation(stage, candidate);
        if (keepMultipleCandidates) {
            insertDistinctEvaluationCandidate(
                *candidatePool, candidate, maxMultiHypothesisCandidates,
                useWeakModeScoring, stage, interestingWeakModeMatchCount,
                weakModeCandidatePoolMinMatches, useGuidedDirectionScoring);
        }
        if (isBetterEvaluationForMode(candidate, best, useWeakModeScoring, useGuidedDirectionScoring)) {
            best = candidate;
            logPlateSolveEvaluation(stage, best, true);
        }
    };

    auto hasGoodGuidedSeed = [&]() {
        if (useStartDirection) {
            if (!isAcceptableDirectionSeedSolve(settings, minMatchCount, best)) {
                return false;
            }
            if (isWidePlateSolveContext(settings)) {
                return true;
            }
            return hasAcceptableBrightnessConsistency(best);
        }
        if (useElevationSeedOnly) {
            return isAcceptableElevationSeedEvaluation(settings, minMatchCount, best);
        }
        if (useStartFov) {
            return isStrongGuidedSolve(settings, minMatchCount, best);
        }
        return false;
    };

    auto appendUniqueOffset = [](QVector<double>& offsets, double value) {
        for (double existing : offsets)
        {
            if (std::fabs(existing - value) < 1e-6) {
                return;
            }
        }
        offsets.append(value);
    };

    auto buildGuidedDirectionOffsets = [&](double radius) {
        QVector<double> offsets;
        appendUniqueOffset(offsets, 0.0);
        if (radius <= 0.0) {
            return offsets;
        }

        const double fineStep = std::min(radius, std::max(0.02, static_cast<double>(settings.m_fov) * 0.04));
        const double fovStep = std::min(radius, std::max(0.10, static_cast<double>(settings.m_fov) * 0.45));
        for (double value : {fineStep, fineStep * 2.0, fineStep * 4.0, fovStep, fovStep * 2.0, radius * 0.5, radius})
        {
            const double clamped = std::min(radius, value);
            appendUniqueOffset(offsets, -clamped);
            appendUniqueOffset(offsets, clamped);
        }
        return offsets;
    };

    auto buildGuidedRollOffsets = [&]() {
        QVector<double> offsets;
        if (useStartRoll)
        {
            for (double factor : {0.0, -0.5, 0.5, -1.0, 1.0}) {
                appendUniqueOffset(offsets, factor * coarseRollRadius);
            }
            return offsets;
        }

        const double step = (settings.m_fov <= 5.0) ? 10.0
            : (settings.m_fov <= 15.0) ? 15.0
            : 30.0;
        appendUniqueOffset(offsets, 0.0);
        for (double value = step; value <= 180.0 + 1e-6; value += step)
        {
            appendUniqueOffset(offsets, -std::min(180.0, value));
            appendUniqueOffset(offsets, std::min(180.0, value));
        }
        return offsets;
    };

    if (useStartDirection)
    {
        const qint64 stageStartMs = searchProfileTimer.elapsed();
        bool guidedSatisfied = false;
        const bool allowGuidedEarlyStop = settings.m_fov < kWideFovMagnitudePreferenceThresholdDegrees;
        const QVector<double> directionOffsets = buildGuidedDirectionOffsets(coarseSearchRadius);
        const QVector<double> rollOffsets = buildGuidedRollOffsets();
        for (double fovFactor : coarseFovOffsetsOrdered)
        {
            if (guidedSatisfied) break;
            for (double elOffset : directionOffsets)
            {
                if (guidedSatisfied) break;
                for (double azOffset : directionOffsets)
                {
                    if (guidedSatisfied) break;
                    for (double rollOffset : rollOffsets)
                    {
                        evaluateSeed(
                            "guided-direction",
                            settings.m_azimuth + azOffset,
                            settings.m_elevation + elOffset,
                            settings.m_roll + rollOffset,
                            std::max(static_cast<double>(CameraSettings::m_minFov),
                                     static_cast<double>(settings.m_fov) + fovFactor * coarseFovRadius),
                            guidedSeedMatchRadius);
                        if (allowGuidedEarlyStop && hasGoodGuidedSeed()) {
                            guidedSatisfied = true;
                            break;
                        }
                    }
                }
            }
        }
        logSearchProfile("guided-direction", stageStartMs);
    }
    else if (useStartElevation)
    {
        const qint64 stageStartMs = searchProfileTimer.elapsed();
        const std::array<double, 5> elevationSeedFovScales = {{1.00, 0.85, 1.15, 0.70, 1.30}};
        for (double fovScale : elevationSeedFovScales)
        {
            for (double elFactor : coarseOffsetsOrdered)
            {
                for (double azimuthDegrees = minAzimuthDegrees; azimuthDegrees < maxAzimuthDegrees; azimuthDegrees += fovGridAzimuthStepDegrees)
                {
                    for (double rollDegrees : fovSearchRollOffsets)
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
            const double azimuthRefineStep = fovGridAzimuthStepDegrees;
            const double elevationRefineStep = std::max(0.25, std::min(fovGridElevationStepDegrees, coarseSearchRadius * 0.25));
            const double rollRefineStep = std::max(0.25, std::min(5.0, static_cast<double>(settings.m_fov) * 0.5));

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
        logSearchProfile("guided-elevation", stageStartMs);
    }
    else if (useStartFov)
    {
        const qint64 stageStartMs = searchProfileTimer.elapsed();
        const double guidedFovMatchRadius = useWideFovSeedRadius
            ? wideFovSeedMatchRadius
            : static_cast<double>(settings.m_plateSolveMatchRadius);
        const bool runGuidedFovGrid = !useWideFovBlindSeedFirst;
        if (runGuidedFovGrid)
        {
            for (double fovFactor : coarseFovOffsetsOrdered)
            {
                for (double elevationDegrees = minElevationDegrees; elevationDegrees <= maxElevationDegrees; elevationDegrees += fovGridElevationStepDegrees)
                {
                    for (double azimuthDegrees = minAzimuthDegrees; azimuthDegrees < maxAzimuthDegrees; azimuthDegrees += fovGridAzimuthStepDegrees)
                    {
                        if (wideWeakMode)
                        {
                            const double fovDegrees = std::max(
                                static_cast<double>(CameraSettings::m_minFov),
                                static_cast<double>(settings.m_fov) + fovFactor * coarseFovRadius);
                            const SkyProjector refProjector = createProjector(
                                settings,
                                imageSize,
                                azimuthDegrees,
                                elevationDegrees,
                                0.0,
                                fovDegrees,
                                fixedCenterOffsetX,
                                fixedCenterOffsetY,
                                fixedDistortionK1);
                            buildBlindGridCache(
                                catalogContext,
                                refProjector,
                                wideFirstPassCatalogIndices.isEmpty() ? nullptr : &wideFirstPassCatalogIndices);
                            for (double rollDegrees : wideRollOffsetsOrdered)
                            {
                                populateBlindGridProjectedCatalog(rollDegrees, guidedFovMatchRadius, refProjector);
                                evaluateSeedFromCache(
                                    "guided-fov",
                                    azimuthDegrees,
                                    elevationDegrees,
                                    rollDegrees,
                                    fovDegrees,
                                    guidedFovMatchRadius);
                            }
                        }
                        else
                        {
                            for (double rollDegrees : wideRollOffsetsOrdered)
                            {
                                evaluateSeed(
                                    "guided-fov",
                                    azimuthDegrees,
                                    elevationDegrees,
                                    rollDegrees,
                                    std::max(static_cast<double>(CameraSettings::m_minFov),
                                             static_cast<double>(settings.m_fov) + fovFactor * coarseFovRadius),
                                    useWideFovSeedRadius ? wideFovSeedMatchRadius : -1.0,
                                    wideFirstPassCatalogIndices.isEmpty() ? nullptr : &wideFirstPassCatalogIndices);
                            }
                        }
                    }
                }
            }
        }

        logSearchProfile("guided-fov", stageStartMs);
    }

    const bool needBlindSearch = !useStartFov
        || (useStartDirection
            ? !hasGoodGuidedSeed()
            : useElevationSeedOnly
                ? !isAcceptableElevationSeedEvaluation(settings, minMatchCount, best)
            : !isStrongGuidedSolve(settings, minMatchCount, best));

    if (needBlindSearch)
    {
        const qint64 stageStartMs = searchProfileTimer.elapsed();
        QVector<VisibleCatalogStar> localVisibleStars;
        const QVector<VisibleCatalogStar>* blindVisibleStars = &catalogContext.visibleStars;
        if (useStartDirection && (settings.m_fov <= 5.0))
        {
            const double localRadiusDegrees = std::max(
                static_cast<double>(settings.m_plateSolveSearchRadius) + static_cast<double>(settings.m_fov) * 2.0,
                static_cast<double>(settings.m_fov) * 4.0);
            localVisibleStars = selectLocalVisibleStars(
                catalogContext.visibleStars,
                settings.m_azimuth,
                settings.m_elevation,
                localRadiusDegrees,
                64);
            if (!localVisibleStars.isEmpty()) {
                blindVisibleStars = &localVisibleStars;
            }
            qDebug() << "CameraPlateSolver: guided narrow blind seed catalog"
                     << "stars" << blindVisibleStars->size()
                     << "radius" << localRadiusDegrees;
        }
        else if (useElevationSeedOnly)
        {
            const double localElevationRadiusDegrees = std::max(
                static_cast<double>(settings.m_plateSolveSearchRadius) + static_cast<double>(settings.m_fov) * 0.5,
                static_cast<double>(settings.m_fov));
            localVisibleStars = selectVisibleStarsNearElevation(
                catalogContext.visibleStars,
                settings.m_elevation,
                localElevationRadiusDegrees,
                128);
            if (!localVisibleStars.isEmpty()) {
                blindVisibleStars = &localVisibleStars;
            }
        }

        auto consumeBlindSeeds = [&](const QVector<Evaluation>& seeds, const char *stage) {
            for (const Evaluation& seed : seeds)
            {
                logPlateSolveEvaluation(stage, seed);
                if (candidatePool) {
                    insertDistinctEvaluationCandidate(
                        *candidatePool,
                        seed,
                        maxMultiHypothesisCandidates,
                        useWeakModeScoring,
                        stage,
                        interestingWeakModeMatchCount,
                        weakModeCandidatePoolMinMatches,
                        useGuidedDirectionScoring);
                }
                if (isBetterEvaluationForMode(seed, best, useWeakModeScoring, useGuidedDirectionScoring)) {
                    best = seed;
                    logPlateSolveEvaluation(stage, best, true);
                }
            }
        };
        auto hasGoodWideBlindSeed = [&]() {
            return wideWeakMode
                && isStrongBlindSeedEvaluation(settings, detectionIndices, best)
                && hasAcceptableBrightnessConsistency(best);
        };

        qint64 seedStageStartMs = searchProfileTimer.elapsed();
        const QVector<Evaluation> brightPairSeeds = buildBrightPairSeeds(
            settings,
            catalogContext,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            *blindVisibleStars);
        logSearchProfile("bright-pair-seeds", seedStageStartMs);
        if (wideWeakMode) {
            consumeBlindSeeds(brightPairSeeds, "bright-pair-seed");
        }

        seedStageStartMs = searchProfileTimer.elapsed();
        const QVector<Evaluation> blindTriangleSeeds = buildBlindTriangleSeeds(
            settings,
            catalogContext,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            *blindVisibleStars);
        logSearchProfile("blind-triangle-seeds", seedStageStartMs);
        consumeBlindSeeds(blindTriangleSeeds, "blind-triangle-seed");

        if (!wideWeakMode) {
            consumeBlindSeeds(brightPairSeeds, "bright-pair-seed");
        }

        if (hasGoodWideBlindSeed())
        {
            recordProfileMetric(QStringLiteral("search.blindQuadSkipped"), 1);
            logSearchProfile("blind-quad-seeds", searchProfileTimer.elapsed());
        }
        else
        {
            seedStageStartMs = searchProfileTimer.elapsed();
            const QVector<Evaluation> blindQuadSeeds = buildBlindQuadSeeds(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                *blindVisibleStars);
            logSearchProfile("blind-quad-seeds", seedStageStartMs);
            consumeBlindSeeds(blindQuadSeeds, "blind-quad-seed");
        }
        logSearchProfile("blind-seeds", stageStartMs);
    }

    // Short-circuit the exhaustive wide-fallback grid when the blind seeds have already
    // landed in a reasonable basin. The grid runs ~52k match evaluations (72 az × 7 el ×
    // 13 roll × 8 fov — but only ~4 k full catalog projections thanks to the roll-sweep
    // cache), so any half-decent prior result is worth keeping. Acceptance bar:
    // at least half the minimum required matches *and* an RMS that fits inside the
    // acquisition radius — the subsequent refinement loops will tighten this further.
    const double wideFallbackRmsCap = std::max(
        static_cast<double>(settings.m_plateSolveMatchRadius) * 0.9,
        2.0);
    const bool blindSeedAlreadyAcceptable = best.valid
        && (best.matchCount >= std::max(2, minMatchCount / 2))
        && (best.rmsErrorPixels <= wideFallbackRmsCap);
    const bool blindSeedVeryStrong = best.valid
        && (best.matchCount >= std::max(10, minMatchCount + 6))
        && (best.rmsErrorPixels <= std::min(wideFallbackRmsCap * 0.50, 8.0));
    const bool wideWeakBestLooksLikeFalseBrightMatch = wideWeakMode
        && !blindSeedVeryStrong
        && best.valid
        && ((!hasAcceptableBrightnessConsistency(best))
            || (std::isfinite(best.meanCatalogMagnitude) && (best.meanCatalogMagnitude > 3.0)));

    if (((!best.valid || (best.matchCount < minMatchCount)) || wideWeakBestLooksLikeFalseBrightMatch)
        && (!useStartDirection || !best.valid || wideWeakBestLooksLikeFalseBrightMatch)
        && (!blindSeedAlreadyAcceptable || wideWeakBestLooksLikeFalseBrightMatch))
    {
        const qint64 stageStartMs = searchProfileTimer.elapsed();
        const std::array<double, 3> wideFovScales = {{0.70, 1.00, 1.30}};
        const std::array<double, 8> wideBlindFovs = {{15.0, 25.0, 40.0, 60.0, 90.0, 130.0, 160.0, 180.0}};
        const std::array<double, 3> wideAllSkyBlindFovs = {{130.0, 160.0, 180.0}};
        QVector<double> blindFallbackFovs;
        blindFallbackFovs.reserve(static_cast<int>(wideBlindFovs.size()) + 3);
        if (settings.m_fov < 15.0)
        {
            const auto& seedFovs = useWideBlindSeedRadius ? wideAllSkyBlindFovs : wideFovScales;
            for (double fovScale : seedFovs)
            {
                const double fovDegrees = useWideBlindSeedRadius
                    ? fovScale
                    : static_cast<double>(settings.m_fov) * fovScale;
                blindFallbackFovs.append(std::clamp(fovDegrees,
                    static_cast<double>(CameraSettings::m_minFov),
                    static_cast<double>(CameraSettings::m_maxFov)));
            }
        }
        if (!useWideBlindSeedRadius)
        {
            for (double fovDegrees : wideBlindFovs) {
                blindFallbackFovs.append(fovDegrees);
            }
        }
        const bool useNarrowBlindFallbackGrid = !useStartFov && (settings.m_fov < 15.0);
        const double fallbackAzimuthStepDegrees = useWideBlindSeedRadius ? 15.0
            : (useStartFov || useNarrowBlindFallbackGrid)
            ? fovGridAzimuthStepDegrees
            : azimuthStepDegrees;
        const double fallbackElevationStepDegrees = (useStartFov || useNarrowBlindFallbackGrid)
            ? fovGridElevationStepDegrees
            : elevationStepDegrees;
        // The effective match radius is constant across all grid points; compute
        // it once so populateBlindGridProjectedCatalog can apply the same bounds.
        const double blindMatchRadius = useStartFov
            ? (useWideFovSeedRadius  ? wideFovSeedMatchRadius
                                     : static_cast<double>(settings.m_plateSolveMatchRadius))
            : (useWideBlindSeedRadius ? wideBlindSeedMatchRadius
                                      : static_cast<double>(settings.m_plateSolveMatchRadius));

        // Roll-sweep optimisation: project the catalog ONCE per (Az, El, FOV)
        // at roll=0, cache the pixel offsets, then rotate by 2-D matrix for
        // each of the 13 roll values — ~13× fewer acos/atan2 calls per
        // Az×El×FOV cell.  Loop order is now FOV-outer, Roll-inner.
        for (double azimuthDegrees = minAzimuthDegrees; azimuthDegrees < maxAzimuthDegrees; azimuthDegrees += fallbackAzimuthStepDegrees)
        {
            for (double elevationDegrees = minElevationDegrees; elevationDegrees <= maxElevationDegrees; elevationDegrees += fallbackElevationStepDegrees)
            {
                if (useStartFov)
                {
                    for (double fovScale : wideFovScales)
                    {
                        const double fovDegrees = std::clamp(
                            static_cast<double>(settings.m_fov) * fovScale,
                            static_cast<double>(CameraSettings::m_minFov),
                            static_cast<double>(CameraSettings::m_maxFov));
                        const SkyProjector refProjector = createProjector(
                            settings, imageSize,
                            azimuthDegrees, elevationDegrees, 0.0, fovDegrees,
                            fixedCenterOffsetX, fixedCenterOffsetY, fixedDistortionK1);
                        buildBlindGridCache(catalogContext, refProjector);
                        for (double rollDegrees : wideRollOffsets)
                        {
                            populateBlindGridProjectedCatalog(rollDegrees, blindMatchRadius, refProjector);
                            evaluateSeedFromCache(
                                "wide-fallback-fov",
                                azimuthDegrees, elevationDegrees, rollDegrees,
                                fovDegrees, blindMatchRadius);
                        }
                    }
                }
                else
                {
                    for (double fovDegrees : blindFallbackFovs)
                    {
                        const double clampedFov = std::clamp(
                            fovDegrees,
                            static_cast<double>(CameraSettings::m_minFov),
                            180.0);
                        const SkyProjector refProjector = createProjector(
                            settings, imageSize,
                            azimuthDegrees, elevationDegrees, 0.0, clampedFov,
                            fixedCenterOffsetX, fixedCenterOffsetY, fixedDistortionK1);
                        buildBlindGridCache(catalogContext, refProjector);
                        for (double rollDegrees : wideRollOffsets)
                        {
                            populateBlindGridProjectedCatalog(rollDegrees, blindMatchRadius, refProjector);
                            evaluateSeedFromCache(
                                "wide-fallback-blind",
                                azimuthDegrees, elevationDegrees, rollDegrees,
                                clampedFov, blindMatchRadius);
                        }
                    }
                }
            }
        }
        logSearchProfile("wide-fallback-grid", stageStartMs);
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
    double fovStep = std::max(minimumFovRefineStep, coarseFovRadius * 0.25);

    for (int iteration = 0; iteration < 2; ++iteration)
    {
        const qint64 stageStartMs = searchProfileTimer.elapsed();
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
                            fixedDistortionK1,
                            searchMatchRadiusOverride);
                        logPlateSolveEvaluation("coarse-refine", candidate);
                        if (keepMultipleCandidates) {
                            insertDistinctEvaluationCandidate(
                                *candidatePool,
                            candidate,
                            maxMultiHypothesisCandidates,
                            useWeakModeScoring,
                            "coarse-refine",
                            interestingWeakModeMatchCount,
                            weakModeCandidatePoolMinMatches,
                            useGuidedDirectionScoring);
                        }
                        if (isBetterEvaluationForMode(candidate, best, useWeakModeScoring, useGuidedDirectionScoring)) {
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
        logSearchProfile("coarse-refine", stageStartMs);
    }

    const QVector<int> allDetectionIndices = [&starDetections]() {
        QVector<int> indices;
        indices.reserve(starDetections.size());
        for (int i = 0; i < starDetections.size(); ++i) {
            indices.append(i);
        }
        return indices;
    }();
    const QVector<int>& fullRefineDetectionIndices =
        wideWeakMode ? detectionIndices : allDetectionIndices;

    azCenter = best.azimuthDegrees;
    elCenter = best.elevationDegrees;
    rollCenter = best.rollDegrees;
    fovCenter = best.fovDegrees;
    azStep = std::max(0.1, azStep);
    elStep = std::max(0.1, elStep);
    rollStep = std::max(0.25, rollStep);
    fovStep = std::max(minimumFovRefineStep, fovStep);

    for (int iteration = 0; iteration < 4; ++iteration)
    {
        const qint64 stageStartMs = searchProfileTimer.elapsed();
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
                            fullRefineDetectionIndices,
                            azCenter + azOffset * azStep,
                            elCenter + elOffset * elStep,
                            rollCenter + rollOffset * rollStep,
                            std::max(static_cast<double>(CameraSettings::m_minFov), fovCenter + fovOffset * fovStep),
                            nullptr,
                            fixedCenterOffsetX,
                            fixedCenterOffsetY,
                            fixedDistortionK1,
                            searchMatchRadiusOverride);
                        logPlateSolveEvaluation("full-refine", candidate);
                        if (keepMultipleCandidates) {
                            insertDistinctEvaluationCandidate(
                                *candidatePool,
                            candidate,
                            maxMultiHypothesisCandidates,
                            useWeakModeScoring,
                            "full-refine",
                            interestingWeakModeMatchCount,
                            weakModeCandidatePoolMinMatches,
                            useGuidedDirectionScoring);
                        }
                        if (isBetterEvaluationForMode(candidate, best, useWeakModeScoring, useGuidedDirectionScoring)) {
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
        logSearchProfile("full-refine", stageStartMs);
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
    const bool useGuidedDirectionScoring = plateSolveStartUsesDirection(settings);

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

    const bool forceAnchor = initialEvaluation.anchored
        && (initialEvaluation.anchorDetectionIndex >= 0)
        && (initialEvaluation.anchorCatalogIndex >= 0);
    GuidedAnchorPair forcedAnchor;
    if (forceAnchor)
    {
        forcedAnchor.detectionIndex = initialEvaluation.anchorDetectionIndex;
        forcedAnchor.catalogIndex = initialEvaluation.anchorCatalogIndex;
        if (!detectionIndices.contains(forcedAnchor.detectionIndex)) {
            detectionIndices.append(forcedAnchor.detectionIndex);
        }
        if (!catalogIndices.contains(forcedAnchor.catalogIndex)) {
            catalogIndices.append(forcedAnchor.catalogIndex);
        }
    }

    auto evaluateRefinementPose = [&](const QVector<int>& activeDetectionIndices,
                                      const QVector<int>& activeCatalogIndices,
                                      double azimuthDegrees,
                                      double elevationDegrees,
                                      double rollDegrees,
                                      double fovDegrees,
                                      double centerOffsetXPixels,
                                      double centerOffsetYPixels,
                                      double distortionK1,
                                      double matchRadiusOverride = -1.0) {
        if (forceAnchor)
        {
            return evaluateAnchoredPose(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                activeDetectionIndices,
                activeCatalogIndices,
                forcedAnchor,
                azimuthDegrees,
                elevationDegrees,
                rollDegrees,
                fovDegrees,
                centerOffsetXPixels,
                centerOffsetYPixels,
                distortionK1,
                matchRadiusOverride > 0.0
                    ? matchRadiusOverride
                    : static_cast<double>(settings.m_plateSolveMatchRadius));
        }

        return evaluatePose(
            settings,
            catalogContext,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            activeDetectionIndices,
            azimuthDegrees,
            elevationDegrees,
            rollDegrees,
            fovDegrees,
            &activeCatalogIndices,
            centerOffsetXPixels,
            centerOffsetYPixels,
            distortionK1,
            matchRadiusOverride);
    };

    Evaluation best = evaluateRefinementPose(
        detectionIndices,
        catalogIndices,
        initialEvaluation.azimuthDegrees,
        initialEvaluation.elevationDegrees,
        initialEvaluation.rollDegrees,
        initialEvaluation.fovDegrees,
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
    const double minimumFovStep = std::max(0.02, std::min(0.5, static_cast<double>(settings.m_fov) * 0.02));
    // For narrow-field direction-seeded solves, hold FOV fixed at settings.m_fov throughout
    // refinement.  The anchor search already found (Az, El, Roll) at the correct FOV.
    double fovStep = (useGuidedDirectionScoring && settings.m_fov <= 5.0)
        ? 0.0
        : std::max(minimumFovStep, static_cast<double>(settings.m_fov) * 0.01);
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
                        const Evaluation candidate = evaluateRefinementPose(
                            detectionIndices,
                            catalogIndices,
                            azCenter + azOffset * azStep,
                            elCenter + elOffset * elStep,
                            rollCenter + rollOffset * rollStep,
                            std::max(static_cast<double>(CameraSettings::m_minFov), fovCenter + fovOffset * fovStep),
                            centerOffsetXCenter,
                            centerOffsetYCenter,
                            distortionCenter);
                        if (isBetterEvaluationForMode(candidate, best, false, useGuidedDirectionScoring)) {
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
                        const Evaluation candidate = evaluateRefinementPose(
                            detectionIndices,
                            catalogIndices,
                            azCenter,
                            elCenter,
                            rollCenter,
                            fovCenter,
                            centerOffsetXCenter + centerOffsetXOffset * centerOffsetXStep,
                            centerOffsetYCenter + centerOffsetYOffset * centerOffsetYStep,
                            std::clamp(distortionCenter + distortionOffset * distortionStep, -0.75, 0.75));
                        if (isBetterEvaluationForMode(candidate, best, false, useGuidedDirectionScoring)) {
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
        const int retainedMatchThreshold = forceAnchor
            ? std::max(2, std::min(settings.m_plateSolveMinMatches, preTighteningBest.matchCount))
            : minimumRetainedMatchesForFinalPass(
                preTighteningBest,
                settings.m_plateSolveMinMatches);
        Evaluation tighteningBest = evaluateRefinementPose(
            detectionIndices,
            catalogIndices,
            azCenter,
            elCenter,
            rollCenter,
            fovCenter,
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
                            const Evaluation candidate = evaluateRefinementPose(
                                detectionIndices,
                                catalogIndices,
                                azCenter + azOffset * azStep,
                                elCenter + elOffset * elStep,
                                rollCenter + rollOffset * rollStep,
                                std::max(static_cast<double>(CameraSettings::m_minFov), fovCenter + fovOffset * fovStep),
                                centerOffsetXCenter,
                                centerOffsetYCenter,
                                distortionCenter,
                                tighteningRadius);
                            if (isBetterFinalPassEvaluation(candidate, tighteningBest, retainedMatchThreshold, useGuidedDirectionScoring)) {
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

static void clearSolvedStars(QVector<CameraPipelineStarDetection>& starDetections)
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

};

CameraPlateSolver::CameraPlateSolver(QObject *parent) :
    QObject(parent),
    m_networkManager(new QNetworkAccessManager(this))
{
}

CameraPlateSolver::~CameraPlateSolver() = default;

QString CameraPlateSolver::downloadedCatalogArchivePath()
{
    SolverContext context;
    return QDir(context.downloadedCatalogDir()).filePath(QString::fromUtf8(SolverContext::kDownloadedCatalogArchiveFile));
}

QString CameraPlateSolver::downloadedCatalogCsvPath()
{
    SolverContext context;
    return QDir(context.downloadedCatalogDir()).filePath(QString::fromUtf8(SolverContext::kDownloadedCatalogCsvFile));
}

bool CameraPlateSolver::importDownloadedCatalogArchive(const QString& archivePath, QString* errorMessage)
{
    SolverContext context;
    QFile inputFile(archivePath);
    if (!inputFile.open(QIODevice::ReadOnly))
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to open downloaded catalog archive: %1").arg(archivePath);
        }
        return false;
    }

    const QByteArray compressedData = inputFile.readAll();
    const QByteArray uncompressedData = context.gunzipData(compressedData, errorMessage);
    if (uncompressedData.isEmpty()) {
        return false;
    }

    const QString outputDirPath = context.downloadedCatalogDir();
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

    const QVector<SolverContext::CatalogStar> reducedCatalog =
        context.filterCatalogStars(context.parseDownloadedHygCatalog(QString::fromUtf8(uncompressedData)));
    if (reducedCatalog.isEmpty())
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Imported HYG catalog did not yield any usable plate-solver stars.");
        }
        return false;
    }

    if (!context.writeReducedCatalog(reducedCatalog, context.downloadedCatalogReducedPath(), errorMessage)) {
        return false;
    }

    return true;
}

CameraPlateSolveResult CameraPlateSolver::SolverContext::solve(const CameraSettings& settings,
                                                               const QSize& imageSize,
                                                               const QDateTime& captureDateTime,
                                                               QVector<CameraPipelineStarDetection>& starDetections)
{
    PROFILER_START();

    CameraPlateSolveResult result;
    const bool profilePlateSolve = qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_PROFILE");
    QElapsedTimer solveProfileTimer;
    solveProfileTimer.start();
    clearProfileTimings();
    auto logSolveProfile = [&](const char *stage, qint64 startedMs) {
        const qint64 elapsedMs = solveProfileTimer.elapsed() - startedMs;
        recordProfileTiming(QStringLiteral("solve.%1").arg(QString::fromUtf8(stage)), elapsedMs);
        if (!profilePlateSolve) {
            return;
        }
        qDebug().noquote().nospace()
            << "CameraPlateSolverProfile solve." << stage
            << " elapsedMs=" << elapsedMs
            << " totalMs=" << solveProfileTimer.elapsed();
    };
    clearSolvedStars(starDetections);
    result.m_detectedStarsConsidered = starDetections.size();
    const bool useCurrentSettingsOnly = plateSolveStartUsesCurrentSettingsOnly(settings);
    const bool useStartFov = plateSolveStartUsesFov(settings);
    const bool useStartElevation = plateSolveStartUsesElevation(settings);
    const bool useStartDirection = plateSolveStartUsesDirection(settings);
    const bool useStartRoll = plateSolveStartUsesRoll(settings);
    const bool useElevationSeedOnly = useStartElevation && !useStartDirection;
    const double finalMatchRadius = settings.m_plateSolveFinalMatchRadius;
    const bool useMultiHypothesisRefine = !useCurrentSettingsOnly;
    const bool useWeakModeScoring = !useStartDirection;

    // Configure weak-mode scoring normalisation for this solve. Weak FoV/Blind searches need
    // to rank and preserve coarse basins using the loose acquisition geometry; the tighter
    // final-match radius is reserved for the late refinement/acceptance stages.
    m_weakModeNormalizationPixels = std::max(1.0, static_cast<double>(settings.m_plateSolveMatchRadius));
    m_useElevationSeedPreference = useElevationSeedOnly;
    m_elevationSeedReferenceDegrees = settings.m_elevation;
    m_elevationSeedReferenceFovDegrees = settings.m_fov;
    m_elevationSeedScaleDegrees = std::max(2.0, static_cast<double>(settings.m_plateSolveSearchRadius) * 0.35);
    m_elevationSeedFovScaleDegrees = std::max(4.0, static_cast<double>(settings.m_fov) * 0.05);
    m_useDirectionSeedPreference = useStartDirection;
    m_directionSeedHasRollPreference = useStartRoll;
    m_directionSeedReferenceAzimuthDegrees = settings.m_azimuth;
    m_directionSeedReferenceElevationDegrees = settings.m_elevation;
    m_directionSeedReferenceRollDegrees = settings.m_roll;
    m_directionSeedReferenceFovDegrees = settings.m_fov;
    m_directionSeedAzElScaleDegrees = (settings.m_fov <= 5.0)
        ? std::max(0.5, static_cast<double>(settings.m_fov))
        : std::max(2.0, static_cast<double>(settings.m_plateSolveSearchRadius) * 0.35);
    m_directionSeedRollScaleDegrees = std::max(8.0, std::min(45.0, static_cast<double>(settings.m_fov) * 0.15));
    m_directionSeedFovScaleDegrees = (settings.m_fov <= 5.0)
        ? std::max(0.3, static_cast<double>(settings.m_fov) * 0.25)
        : std::max(2.0, static_cast<double>(settings.m_fov) * 0.08);
    m_directionSeedMinMatchCount = useStartDirection
        ? minimumDirectionSeedAcceptedMatches(settings, starDetections)
        : std::max(1, settings.m_plateSolveMinMatches);
    m_useFovSeedPreference = useStartFov;
    m_fovSeedReferenceDegrees = settings.m_fov;
    m_fovSeedScaleDegrees = (settings.m_fov <= 5.0)
        ? std::max(0.3, static_cast<double>(settings.m_fov) * 0.25)
        : std::max(2.0, static_cast<double>(settings.m_fov) * 0.06);
    m_useWideCatalogMagnitudePreference = isWidePlateSolveContext(settings);
    m_useAllSkyZenithPreference = useStartFov
        && useStartElevation
        && !useStartDirection
        && (settings.m_lensProjection != CameraSettings::LensProjectionRectilinear)
        && (settings.m_fov >= 120.0)
        && std::isfinite(static_cast<double>(settings.m_elevation))
        && (settings.m_elevation >= 45.0);
    m_allSkyZenithReferenceElevationDegrees = settings.m_elevation;
    m_allSkyZenithScaleDegrees = std::max(12.0, std::min(25.0, static_cast<double>(settings.m_fov) * 0.10));

    if (starDetections.isEmpty())
    {
        m_detectionBrightnessMetricCache.clear();
        m_detectionReliabilityMetricCache.clear();
        result.m_failureReason = QStringLiteral("no star detections");
        return result;
    }
    prepareDetectionMetricCache(starDetections);
    result.m_requiredMatches = useStartDirection
        ? minimumDirectionSeedAcceptedMatches(settings, starDetections)
        : std::max(settings.m_plateSolveMinMatches, 4);

    QDateTime configuredSolveDateTime = settings.m_plateSolveDateTime.isValid()
        ? settings.m_plateSolveDateTime
        : captureDateTime;
    if (configuredSolveDateTime.isValid()) {
        configuredSolveDateTime.setTimeSpec(settings.m_plateSolveDateTimeUtc ? Qt::UTC : Qt::LocalTime);
    }
    const QDateTime solveDateTime = settings.m_plateSolveUseCurrentDateTime
        ? QDateTime::currentDateTime()
        : configuredSolveDateTime;
    const QDateTime captureDateTimeUtc = (solveDateTime.isValid() ? solveDateTime : QDateTime::currentDateTime()).toUTC();
    const double solveMaxMagnitude = firstPassPlateSolveMaxMagnitude(settings);
    const bool useBrightFirstPassCatalog = (solveMaxMagnitude < settings.m_plateSolveMaxMagnitude)
        && useStartDirection
        && (settings.m_fov <= 5.0);
    if (useBrightFirstPassCatalog)
    {
        qDebug() << "CameraPlateSolver: guided narrow-field solve using bright first-pass catalog"
                 << "requestedMaxMag" << settings.m_plateSolveMaxMagnitude
                 << "solveMaxMag" << solveMaxMagnitude
                 << "fov" << settings.m_fov;
    }
    qint64 stageStartMs = solveProfileTimer.elapsed();
    PlateSolveCatalogContext catalogContext = buildPlateSolveCatalogContext(
        settings,
        imageSize,
        captureDateTimeUtc,
        solveMaxMagnitude,
        useBrightFirstPassCatalog ? settings.m_plateSolveMaxMagnitude : solveMaxMagnitude);
    logSolveProfile("catalog", stageStartMs);
    result.m_catalogSource = catalogContext.catalogSource;
    result.m_catalogStarsLoaded = catalogContext.catalogStars.size();

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
            result.m_failureReason = QStringLiteral("invalid current-settings projector");
            PROFILER_STOP(QString("%1: invalid current-settings projector").arg(__FUNCTION__));
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
            nullptr,
            finalMatches);

        if (finalMatches.isEmpty()) {
            result.m_failureReason = QStringLiteral("no current-settings matches");
            PROFILER_STOP(QString("%1: no current-settings matches").arg(__FUNCTION__));
            return result;
        }

        double maxError = 0.0;
        QHash<int, QPointF> projectedPointsByCatalogIndex;
        for (const ProjectedCatalogStar& projectedStar : projectedStars) {
            projectedPointsByCatalogIndex.insert(projectedStar.catalogIndex, projectedStar.point);
        }
        for (const Match& match : finalMatches)
        {
            CameraPipelineStarDetection& detection = starDetections[match.detectionIndex];
            const CatalogStar& catalogStar = catalogContext.catalogStars[match.catalogIndex];
            detection.m_label = catalogDisplayName(catalogStar);
            detection.m_projectedCenter = projectedPointsByCatalogIndex.value(match.catalogIndex);
            detection.m_matchDistancePixels = static_cast<float>(match.distancePixels);
            detection.m_catalogMagnitude = static_cast<float>(catalogStar.magnitude);
            detection.m_catalogSpectralType = catalogStar.spectralType;
            detection.m_solved = true;
            maxError = std::max(maxError, match.distancePixels);
        }

        result.m_matchedStars = finalMatches.size();
        result.m_rmsErrorPixels = weightedRmsDistancePixels(starDetections, finalMatches);
        result.m_maxErrorPixels = maxError;
        result.m_matchSummary = matchSummary(catalogContext, starDetections, finalMatches);

        // Apply the same quality acceptance applied to every other code path. Without this
        // gate, the current-settings path would mark m_solved=true on three random near-
        // coincidences within the (loose) acquisition radius.
        const double currentSettingsMaxRms = std::min(finalMatchRadius * 0.85, 24.0);
        const double currentSettingsMaxMedian = std::min(finalMatchRadius * 0.70, 18.0);
        const double currentSettingsMedian = medianDistancePixels(finalMatches);
        result.m_solved = (finalMatches.size() >= settings.m_plateSolveMinMatches)
            && (result.m_rmsErrorPixels <= currentSettingsMaxRms)
            && (currentSettingsMedian <= currentSettingsMaxMedian);
        if (!result.m_solved)
        {
            result.m_failureReason = QStringLiteral("current settings rejected: matches=%1 required=%2 RMS=%3 median=%4")
                .arg(finalMatches.size())
                .arg(settings.m_plateSolveMinMatches)
                .arg(result.m_rmsErrorPixels, 0, 'f', 2)
                .arg(currentSettingsMedian, 0, 'f', 2);
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
            catalogContext,
            starDetections,
            projectedStars,
            finalMatches,
            finalMatchRadius);

        PROFILER_STOP(QString("%1: current-settings-only").arg(__FUNCTION__));
        return result;
    }

    if ((starDetections.size() < settings.m_plateSolveMinMatches))
    {
        result.m_failureReason = QStringLiteral("too few star detections: %1 < %2")
            .arg(starDetections.size())
            .arg(settings.m_plateSolveMinMatches);
        return result;
    }

    const QVector<int> detectionIndices = selectDetectionIndicesForSolve(starDetections, imageSize);
    if (detectionIndices.size() < settings.m_plateSolveMinMatches)
    {
        result.m_failureReason = QStringLiteral("too few selected detections: %1 < %2")
            .arg(detectionIndices.size())
            .arg(settings.m_plateSolveMinMatches);
        return result;
    }

    QVector<Evaluation> coarseCandidates;
    FinalMatchPassEvaluation selectedFinalPass;
    stageStartMs = solveProfileTimer.elapsed();
    Evaluation best = searchBestPose(
        settings,
        catalogContext,
        imageSize,
        captureDateTimeUtc,
        starDetections,
        detectionIndices,
        useMultiHypothesisRefine ? &coarseCandidates : nullptr);
    logSolveProfile("searchBestPose", stageStartMs);
    bool usingFullCatalogForGuidedAnchor = false;
    const bool useWideWeakAnchorSearch = !useStartDirection
        && isWidePlateSolveContext(settings);
    const int guidedAnchorExtraMatches = useStartDirection
        ? (isWidePlateSolveContext(settings)
            ? (starDetections.size() <= 16 ? 0 : 1)
            : 2)
        : 0;
    const int guidedAnchorSkipRequiredMatches = result.m_requiredMatches + guidedAnchorExtraMatches;
    const bool guidedAnchorSkipBrightnessAccepted =
        (useStartDirection && isWidePlateSolveContext(settings))
        || hasAcceptableBrightnessConsistency(best);
    const double guidedAnchorSkipRmsCap = (useStartDirection && isWidePlateSolveContext(settings))
        ? std::min(finalMatchRadius * 0.90, 24.0)
        : std::min(finalMatchRadius * 0.60, 16.0);
    bool bestStrongEnoughToSkipGuidedAnchor = best.valid
        && (best.matchCount >= guidedAnchorSkipRequiredMatches)
        && (best.rmsErrorPixels <= guidedAnchorSkipRmsCap)
        && guidedAnchorSkipBrightnessAccepted;
    if (bestStrongEnoughToSkipGuidedAnchor)
    {
        stageStartMs = solveProfileTimer.elapsed();
        FinalMatchPassEvaluation skipFinalPass = evaluateFinalMatchPass(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            isWidePlateSolveContext(settings) ? detectionIndices : allDetectionIndices,
            best,
            finalMatchRadius,
            isWidePlateSolveContext(settings));
        const bool skipFinalPassMatchesDirection = !useStartDirection
            || (isAcceptableDirectionSeedSolve(
                    settings,
                    starDetections,
                    skipFinalPass.finalMatches,
                    skipFinalPass.rmsErrorPixels,
                    skipFinalPass.maxErrorPixels)
                && hasAcceptableGuidedFinalBrightnessConsistency(settings, skipFinalPass));
        const bool skipFinalPassAccepted = skipFinalPass.projectorValid
            && (skipFinalPass.finalMatches.size() >= result.m_requiredMatches)
            && skipFinalPassMatchesDirection
            && hasAcceptableWideBrightAnchorSupport(settings, starDetections, skipFinalPass);
        if (skipFinalPassAccepted)
        {
            selectedFinalPass = skipFinalPass;
            logFinalMatchPassEvaluation("final-match-pass-guided-skip", selectedFinalPass, true);
        }
        else
        {
            bestStrongEnoughToSkipGuidedAnchor = false;
            if (profilePlateSolve && (useStartDirection || useWideWeakAnchorSearch))
            {
                qDebug().noquote().nospace()
                    << "CameraPlateSolverProfile solve.guidedAnchor skip rejected"
                    << " bestMatches=" << best.matchCount
                    << " finalMatches=" << skipFinalPass.finalMatches.size()
                    << " required=" << result.m_requiredMatches
                    << " finalRms=" << skipFinalPass.rmsErrorPixels;
            }
        }
        logSolveProfile("guidedSkipFinalPass", stageStartMs);
    }
    if (profilePlateSolve && (useStartDirection || useWideWeakAnchorSearch) && bestStrongEnoughToSkipGuidedAnchor)
    {
        qDebug().noquote().nospace()
            << "CameraPlateSolverProfile solve.guidedAnchor skipped"
            << " bestMatches=" << best.matchCount
            << " required=" << result.m_requiredMatches
            << " bestRms=" << best.rmsErrorPixels;
    }
    if ((useStartDirection || useWideWeakAnchorSearch) && !bestStrongEnoughToSkipGuidedAnchor)
    {
        stageStartMs = solveProfileTimer.elapsed();
        PlateSolveCatalogContext guidedAnchorCatalogContext;
        const PlateSolveCatalogContext *guidedAnchorCatalogContextPtr = &catalogContext;
        if (useBrightFirstPassCatalog)
        {
            guidedAnchorCatalogContext = catalogContext;
            rebuildVisibleCatalogContext(
                guidedAnchorCatalogContext,
                settings,
                captureDateTimeUtc,
                settings.m_plateSolveMaxMagnitude);
            if (!guidedAnchorCatalogContext.catalogStars.isEmpty()) {
                guidedAnchorCatalogContextPtr = &guidedAnchorCatalogContext;
            }
        }

        QVector<Evaluation> guidedAnchorCandidates;
        const Evaluation guidedAnchorBest = searchGuidedAnchorPose(
            settings,
            *guidedAnchorCatalogContextPtr,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            &guidedAnchorCandidates);
        if (guidedAnchorBest.valid)
        {
            bool guidedAnchorMatchesActiveCatalog = guidedAnchorCatalogContextPtr == &catalogContext;
            if (guidedAnchorCatalogContextPtr != &catalogContext)
            {
                catalogContext = std::move(guidedAnchorCatalogContext);
                result.m_catalogSource = catalogContext.catalogSource;
                result.m_catalogStarsLoaded = catalogContext.catalogStars.size();
                coarseCandidates.clear();
                usingFullCatalogForGuidedAnchor = true;
                guidedAnchorMatchesActiveCatalog = true;
            }
            if (guidedAnchorMatchesActiveCatalog)
            {
                const int guidedAnchorPoolLimit = useWideWeakAnchorSearch ? 64 : 24;
                coarseCandidates.prepend(guidedAnchorBest);
                for (const Evaluation& guidedAnchorCandidate : guidedAnchorCandidates)
                {
                    insertDistinctEvaluationCandidate(
                        coarseCandidates,
                        guidedAnchorCandidate,
                        guidedAnchorPoolLimit,
                        useWeakModeScoring,
                        "guided-anchor-candidate-pool",
                        std::max(3, settings.m_plateSolveMinMatches - 1),
                        2,
                        useStartDirection);
                }
                insertDistinctEvaluationCandidate(
                    coarseCandidates,
                    guidedAnchorBest,
                    guidedAnchorPoolLimit,
                    useWeakModeScoring,
                    "guided-anchor-candidate",
                    std::max(3, settings.m_plateSolveMinMatches - 1),
                    std::max(3, settings.m_plateSolveMinMatches - 2),
                    useStartDirection);
                if ((guidedAnchorBest.matchCount >= std::max(3, settings.m_plateSolveMinMatches - 1))
                    || isBetterEvaluationForMode(guidedAnchorBest, best, useWeakModeScoring, useStartDirection))
                {
                    best = guidedAnchorBest;
                    logPlateSolveEvaluation("guided-anchor", best, true);
                }
            }
        }
        logSolveProfile("guidedAnchor", stageStartMs);
    }
    if (!best.valid)
    {
        result.m_failureReason = QStringLiteral("no valid pose candidate");
        return result;
    }
    const int weakModeCandidatePoolMinMatches = std::max(3, settings.m_plateSolveMinMatches - 2);
    const int weakModeRefineMinMatches = std::max(3, settings.m_plateSolveMinMatches - 1);
    const bool rankWideFinalPassWithSelectedDetections = isWidePlateSolveContext(settings);
    const int multiHypothesisCandidateLimit = (useStartDirection && (settings.m_fov <= 5.0)) ? 24
        : rankWideFinalPassWithSelectedDetections ? 64
        : 10;
    if ((best.matchCount < settings.m_plateSolveMinMatches)
        && (!useMultiHypothesisRefine || (best.matchCount < weakModeRefineMinMatches)))
    {
        result.m_matchedStars = best.matchCount;
        result.m_rmsErrorPixels = std::isfinite(best.rmsErrorPixels) ? best.rmsErrorPixels : 0.0;
        result.m_maxErrorPixels = maxDistancePixels(best.matches);
        result.m_matchSummary = matchSummary(catalogContext, starDetections, best.matches);
        result.m_failureReason = QStringLiteral("best pose had too few matches: %1 < %2")
            .arg(best.matchCount)
            .arg(settings.m_plateSolveMinMatches);
        return result;
    }
    if (useMultiHypothesisRefine) {
        stageStartMs = solveProfileTimer.elapsed();
        insertDistinctEvaluationCandidate(
            coarseCandidates,
            best,
            multiHypothesisCandidateLimit,
            useWeakModeScoring,
            "coarse-candidate-pool",
            std::max(3, settings.m_plateSolveMinMatches - 1),
            weakModeCandidatePoolMinMatches,
            useStartDirection);
        logWeakModeCandidatePool("coarse-candidate-pool", coarseCandidates);
        QVector<Evaluation> rescoredCandidates;
        rescoredCandidates.reserve(coarseCandidates.size());
        for (const Evaluation& candidate : coarseCandidates)
        {
            const Evaluation rescoredCandidate = candidate.anchored
                ? candidate
                : rescoreWeakModeCandidateWithDistortionSweep(
                    settings,
                    catalogContext,
                    imageSize,
                    captureDateTimeUtc,
                    starDetections,
                    detectionIndices,
                    candidate);
            logPlateSolveEvaluation("rescore-distortion-sweep", rescoredCandidate, false, true);
            if (rescoredCandidate.anchored) {
                rescoredCandidates.prepend(rescoredCandidate);
            } else {
                insertDistinctEvaluationCandidate(
                    rescoredCandidates,
                    rescoredCandidate,
                    multiHypothesisCandidateLimit,
                    useWeakModeScoring,
                    "rescored-candidate-pool",
                    std::max(3, settings.m_plateSolveMinMatches - 1),
                    weakModeCandidatePoolMinMatches,
                    useStartDirection);
            }
        }
        logWeakModeCandidatePool("rescored-candidate-pool", rescoredCandidates);
        logSolveProfile("rescoreCandidates", stageStartMs);

        stageStartMs = solveProfileTimer.elapsed();
        Evaluation refinedBest = selectedFinalPass.projectorValid ? selectedFinalPass.pose : Evaluation();
        FinalMatchPassEvaluation refinedBestFinalPass = selectedFinalPass;
        for (const Evaluation& candidate : rescoredCandidates)
        {
            const int candidateRefineMinMatches = candidate.anchored ? 2 : weakModeRefineMinMatches;
            if (!candidate.valid || (candidate.matchCount < candidateRefineMinMatches)) {
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
            if (!refinedCandidate.valid || (refinedCandidate.matchCount < candidateRefineMinMatches)) {
                continue;
            }

            const FinalMatchPassEvaluation finalPassEvaluation = evaluateFinalMatchPass(
                settings,
                catalogContext,
                imageSize,
                starDetections,
                rankWideFinalPassWithSelectedDetections ? detectionIndices : allDetectionIndices,
                refinedCandidate,
                finalMatchRadius,
                rankWideFinalPassWithSelectedDetections);
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
        logSolveProfile("refineCandidates", stageStartMs);

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
        result.m_failureReason = QStringLiteral("refine stage rejected candidate: valid=%1 matches=%2 required=%3 RMS=%4")
            .arg(best.valid)
            .arg(best.matchCount)
            .arg(settings.m_plateSolveMinMatches)
            .arg(best.rmsErrorPixels, 0, 'f', 2);
        result.m_matchSummary = matchSummary(catalogContext, starDetections, best.matches);
        return result;
    }

    if (useBrightFirstPassCatalog && !usingFullCatalogForGuidedAnchor)
    {
        PlateSolveCatalogContext fullCatalogContext = catalogContext;
        rebuildVisibleCatalogContext(
            fullCatalogContext,
            settings,
            captureDateTimeUtc,
            settings.m_plateSolveMaxMagnitude);
        if (!fullCatalogContext.catalogStars.isEmpty())
        {
            catalogContext = std::move(fullCatalogContext);
            result.m_catalogSource = catalogContext.catalogSource;
            result.m_catalogStarsLoaded = catalogContext.catalogStars.size();
            selectedFinalPass = FinalMatchPassEvaluation();
        }
    }

    if (selectedFinalPass.projectorValid
        && rankWideFinalPassWithSelectedDetections
        && useStartFov
        && (std::fabs(best.fovDegrees - settings.m_fov) >= 2.0))
    {
        stageStartMs = solveProfileTimer.elapsed();
        Evaluation bestFovPinnedEvaluation;
        FinalMatchPassEvaluation bestFovPinnedFinalPass;
        const std::array<double, 5> distortionSeeds = {{
            best.distortionK1,
            0.0,
            0.05,
            -0.05,
            0.10
        }};
        for (double distortionSeed : distortionSeeds)
        {
            for (double azimuthOffset : {-2.0, 0.0, 2.0})
            {
                for (double elevationOffset : {-2.0, 0.0, 2.0})
                {
                    for (double rollOffset : {-2.0, 0.0, 2.0})
                    {
                        Evaluation fovPinnedBest = best;
                        fovPinnedBest.azimuthDegrees += azimuthOffset;
                        fovPinnedBest.elevationDegrees += elevationOffset;
                        fovPinnedBest.rollDegrees += rollOffset;
                        fovPinnedBest.fovDegrees = settings.m_fov;
                        fovPinnedBest.distortionK1 = distortionSeed;
                        const FinalMatchPassEvaluation fovPinnedFinalPass = evaluateFinalMatchPass(
                            settings,
                            catalogContext,
                            imageSize,
                            starDetections,
                            detectionIndices,
                            fovPinnedBest,
                            finalMatchRadius,
                            true);
                        logFinalMatchPassEvaluation("final-match-pass-fov-pinned", fovPinnedFinalPass);
                        if (isBetterWeakModeFinalMatchPass(
                                settings,
                                starDetections,
                                !useStartFov,
                                fovPinnedFinalPass,
                                bestFovPinnedFinalPass))
                        {
                            bestFovPinnedEvaluation = fovPinnedBest;
                            bestFovPinnedFinalPass = fovPinnedFinalPass;
                        }
                    }
                }
            }
        }
        if (bestFovPinnedFinalPass.projectorValid
            && hasAcceptableWideBrightAnchorSupport(settings, starDetections, bestFovPinnedFinalPass))
        {
            best = bestFovPinnedEvaluation;
            selectedFinalPass = bestFovPinnedFinalPass;
            logFinalMatchPassEvaluation("final-match-pass-fov-pinned", selectedFinalPass, true);
        }
        logSolveProfile("fovPinnedFinalPass", stageStartMs);
    }

    stageStartMs = solveProfileTimer.elapsed();
    if (selectedFinalPass.projectorValid && rankWideFinalPassWithSelectedDetections)
    {
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
        selectedFinalPass = evaluateFinalMatchPass(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            allDetectionIndices,
            best,
            finalMatchRadius);
    }
    if (!selectedFinalPass.projectorValid)
    {
        result.m_failureReason = QStringLiteral("final match projector invalid");
        return result;
    }
    logSolveProfile("finalMatchPass", stageStartMs);

    const QVector<ProjectedCatalogStar>& projectedStars = selectedFinalPass.projectedStars;
    const QVector<Match>& finalMatches = selectedFinalPass.finalMatches;
    result.m_catalogCandidateStars = projectedStars.size();
    result.m_outlierStars = selectedFinalPass.outlierCount;

    const bool selectedWideBrightAnchorAccepted =
        hasAcceptableWideBrightAnchorSupport(settings, starDetections, selectedFinalPass);
    if (!selectedWideBrightAnchorAccepted)
    {
        qDebug() << "CameraPlateSolver: final match pass rejected candidate due to weak wide-field bright-star support"
                 << "finalMatches" << finalMatches.size()
                 << "brightDetections" << selectedFinalPass.matchedBrightDetections
                 << "/" << selectedFinalPass.brightDetections
                 << "brightMagErr" << selectedFinalPass.brightDetectionMagnitudeError
                 << "brightProjected" << selectedFinalPass.matchedBrightProjectedStars
                 << "/" << selectedFinalPass.brightProjectedStars
                 << "Az" << selectedFinalPass.pose.azimuthDegrees
                 << "El" << selectedFinalPass.pose.elevationDegrees
                 << "Roll" << selectedFinalPass.pose.rollDegrees
                 << "FoV" << selectedFinalPass.pose.fovDegrees;
        result.m_matchedStars = finalMatches.size();
        result.m_rmsErrorPixels = selectedFinalPass.rmsErrorPixels;
        result.m_maxErrorPixels = selectedFinalPass.maxErrorPixels;
        result.m_matchSummary = matchSummary(catalogContext, starDetections, finalMatches);
        result.m_failureReason = QStringLiteral("wide-field candidate lacked bright-star support: brightDetections=%1/%2 brightProjected=%3/%4 brightMagErr=%5")
            .arg(selectedFinalPass.matchedBrightDetections)
            .arg(selectedFinalPass.brightDetections)
            .arg(selectedFinalPass.matchedBrightProjectedStars)
            .arg(selectedFinalPass.brightProjectedStars)
            .arg(selectedFinalPass.brightDetectionMagnitudeError, 0, 'f', 2);
        PROFILER_STOP(QString("%1: weak wide bright-star support").arg(__FUNCTION__));
        return result;
    }

    const bool sparseWideBlindAccepted = isAcceptableSparseWideBlindSolve(
        settings,
        starDetections,
        selectedFinalPass);
    const bool denseWideBlindAccepted = !useStartFov
        && !useStartElevation
        && !useStartDirection
        && isWidePlateSolveContext(settings)
        && (finalMatches.size() >= settings.m_plateSolveMinMatches)
        && selectedWideBrightAnchorAccepted
        && (selectedFinalPass.rmsErrorPixels <= std::min(settings.m_plateSolveFinalMatchRadius * 0.85, 24.0))
        && (selectedFinalPass.maxErrorPixels <= std::min(settings.m_plateSolveFinalMatchRadius * 1.25, 36.0));

    if ((finalMatches.size() < settings.m_plateSolveMinMatches) && !sparseWideBlindAccepted) {
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
        result.m_failureReason = QStringLiteral("final match pass rejected candidate: matches=%1 required=%2 raw=%3 outliers=%4 RMS=%5")
            .arg(finalMatches.size())
            .arg(settings.m_plateSolveMinMatches)
            .arg(selectedFinalPass.rawMatchCount)
            .arg(selectedFinalPass.outlierCount)
            .arg(selectedFinalPass.rmsErrorPixels, 0, 'f', 2);
        result.m_matchSummary = matchSummary(catalogContext, starDetections, finalMatches);
        PROFILER_STOP(QString("%1: insufficient matches").arg(__FUNCTION__));
        return result;
    }

    if (!hasGeometricallyConsistentMatches(starDetections, projectedStars, finalMatches, finalMatchRadius))
    {
        result.m_matchedStars = finalMatches.size();
        result.m_rmsErrorPixels = selectedFinalPass.rmsErrorPixels;
        result.m_maxErrorPixels = selectedFinalPass.maxErrorPixels;
        result.m_matchSummary = matchSummary(catalogContext, starDetections, finalMatches);
        result.m_failureReason = QStringLiteral("final match pass rejected candidate: inconsistent inter-star geometry");
        PROFILER_STOP(QString("%1: inconsistent match geometry").arg(__FUNCTION__));
        return result;
    }

    QHash<int, QPointF> projectedPointsByCatalogIndex;
    for (const ProjectedCatalogStar& projectedStar : projectedStars) {
        projectedPointsByCatalogIndex.insert(projectedStar.catalogIndex, projectedStar.point);
    }
    for (const Match& match : finalMatches)
    {
        CameraPipelineStarDetection& detection = starDetections[match.detectionIndex];
        const CatalogStar& catalogStar = catalogContext.catalogStars[match.catalogIndex];
        detection.m_label = catalogDisplayName(catalogStar);
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
    result.m_matchSummary = matchSummary(catalogContext, starDetections, finalMatches);
    const bool directionSeedSolveAcceptable = !useStartDirection
        || (isAcceptableDirectionSeedSolve(settings, starDetections, finalMatches, result.m_rmsErrorPixels, result.m_maxErrorPixels)
            && hasAcceptableGuidedFinalBrightnessConsistency(settings, selectedFinalPass));
    if (!directionSeedSolveAcceptable)
    {
        const QString rejectionReason = directionSeedRejectionReason(
            settings,
            starDetections,
            finalMatches,
            result.m_rmsErrorPixels,
            result.m_maxErrorPixels);
        qDebug() << "CameraPlateSolver: rejecting direction-seeded solution"
                 << "matches=" << finalMatches.size()
                 << "required=" << minimumDirectionSeedAcceptedMatches(settings, starDetections)
                 << "rms=" << result.m_rmsErrorPixels
                 << "max=" << result.m_maxErrorPixels
                 << "brightnessErr=" << selectedFinalPass.brightnessRankError
                 << "reason=" << rejectionReason
                 << "matchSummary=" << result.m_matchSummary;
        result.m_failureReason = QStringLiteral("direction-seeded solution rejected: %1 brightnessErr=%2")
            .arg(rejectionReason)
            .arg(selectedFinalPass.brightnessRankError, 0, 'f', 3);
        clearSolvedStars(starDetections);
        result.m_solved = false;
        PROFILER_STOP(QString("%1: unacceptable direction-seeded solve").arg(__FUNCTION__));
        return result;
    }

    if (useElevationSeedOnly
        && !isAcceptableElevationSeedSolve(settings, starDetections, finalMatches, result.m_rmsErrorPixels, result.m_maxErrorPixels))
    {
        qDebug() << "CameraPlateSolver: rejecting elevation-seeded solution"
                 << "matches=" << finalMatches.size()
                 << "rms=" << result.m_rmsErrorPixels
                 << "max=" << result.m_maxErrorPixels
                 << "matchSummary=" << result.m_matchSummary;
        result.m_failureReason = QStringLiteral("elevation-seeded solution rejected: matches=%1 RMS=%2 max=%3")
            .arg(finalMatches.size())
            .arg(result.m_rmsErrorPixels, 0, 'f', 2)
            .arg(result.m_maxErrorPixels, 0, 'f', 2);
        clearSolvedStars(starDetections);
        result.m_solved = false;
        PROFILER_STOP(QString("%1: unacceptable elevation-seeded solve").arg(__FUNCTION__));
        return result;
    }

    if (!useStartFov
        && !useStartElevation
        && !useStartDirection
        && !sparseWideBlindAccepted
        && !denseWideBlindAccepted
        && !isAcceptableBlindSolve(settings, starDetections, finalMatches, result.m_rmsErrorPixels, result.m_maxErrorPixels))
    {
        qDebug() << "CameraPlateSolver: rejecting blind solution"
                 << "matches=" << finalMatches.size()
                 << "rms=" << result.m_rmsErrorPixels
                 << "max=" << result.m_maxErrorPixels
                 << "matchSummary=" << result.m_matchSummary;
        result.m_failureReason = QStringLiteral("blind solution rejected: matches=%1 RMS=%2 max=%3")
            .arg(finalMatches.size())
            .arg(result.m_rmsErrorPixels, 0, 'f', 2)
            .arg(result.m_maxErrorPixels, 0, 'f', 2);
        clearSolvedStars(starDetections);
        result.m_solved = false;
        PROFILER_STOP(QString("%1: unacceptable blind solve").arg(__FUNCTION__));
        return result;
    }
    result.m_azimuthDegrees = selectedFinalPass.pose.azimuthDegrees;
    result.m_elevationDegrees = selectedFinalPass.pose.elevationDegrees;
    result.m_rollDegrees = selectedFinalPass.pose.rollDegrees;
    result.m_fovDegrees = selectedFinalPass.pose.fovDegrees;
    result.m_centerOffsetXPixels = selectedFinalPass.pose.centerOffsetXPixels;
    result.m_centerOffsetYPixels = selectedFinalPass.pose.centerOffsetYPixels;
    result.m_distortionK1 = selectedFinalPass.pose.distortionK1;

    logUnmatchedDetections(
        catalogContext,
        starDetections,
        projectedStars,
        finalMatches,
        finalMatchRadius);

    PROFILER_STOP(__FUNCTION__);

    return result;
}

void CameraPlateSolver::requestNetworkCancellation()
{
    m_cancelNetworkRequests = true;
    if (m_activeNetworkReply) {
        m_activeNetworkReply->abort();
    }
}

CameraPlateSolveResult CameraPlateSolver::solve(const CameraSettings& settings,
                                                const QSize& imageSize,
                                                const QDateTime& captureDateTime,
                                                QVector<CameraPipelineStarDetection>& starDetections)
{
    // Reset cancellation flag at the start of each solve so that a cancellation
    // from a previous solve doesn't block subsequent ones.
    m_cancelNetworkRequests = false;

    SolverContext context(this);

    // Swap the persistent caches into the SolverContext so that Siril SPCC data
    // fetched in this solve is reused in future solves rather than discarded.
    std::swap(context.m_sirilRangeCache, m_sirilRangeCache);
    std::swap(context.m_sirilIndexCache, m_sirilIndexCache);

    CameraPlateSolveResult result = context.solve(settings, imageSize, captureDateTime, starDetections);
    result.m_profileSummary = context.profileSummary();

    std::swap(context.m_sirilRangeCache, m_sirilRangeCache);
    std::swap(context.m_sirilIndexCache, m_sirilIndexCache);

    // Evict the range cache if it has grown beyond the size limit.  The index cache
    // is bounded naturally (≤ 48 chunks × 64 KB ≈ 3 MB) and is always kept.
    qint64 rangeCacheBytes = 0;
    for (const QByteArray& v : m_sirilRangeCache) {
        rangeCacheBytes += v.size();
    }
    if (rangeCacheBytes > SolverContext::kSirilMaxRangeCacheBytes) {
        qDebug() << "CameraPlateSolver: Siril range cache exceeded" << SolverContext::kSirilMaxRangeCacheBytes
                 << "bytes (" << rangeCacheBytes << "), clearing";
        m_sirilRangeCache.clear();
    }

    return result;
}
