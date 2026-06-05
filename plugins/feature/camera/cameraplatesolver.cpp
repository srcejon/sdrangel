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
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <utility>

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
#include <QRunnable>
#include <QSaveFile>
#include <QSet>
#include <QStandardPaths>
#include <QString>
#include <QStringList>
#include <QThread>
#include <QThreadPool>
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
    double catalogMagnitude = 0.0;
    int geometricSupport = 0;
    double detectionReliability = 0.0;
    double catalogAssignmentPenalty = 0.0;
    double detectionReliabilityLog = 0.0;
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
    double detectionShapeScore = 0.0;
};

struct Evaluation
{
    bool valid = false;
    bool anchored = false;
    bool sparseGuidedPair = false;
    bool guidedTriangle = false;
    int anchorDetectionIndex = -1;
    int anchorCatalogIndex = -1;
    int secondaryAnchorDetectionIndex = -1;
    int secondaryAnchorCatalogIndex = -1;
    int tertiaryAnchorDetectionIndex = -1;
    int tertiaryAnchorCatalogIndex = -1;
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
    int seedProjectedBrightStars = 0;
    int matchedSeedProjectedBrightStars = 0;
    double seedProjectedBrightMatchFraction = 1.0;
    double seedProjectedMagnitudeSupport = 0.0;
    double matchedSeedProjectedMagnitudeSupport = 0.0;
    double seedProjectedMagnitudeMatchFraction = 1.0;
    int seedRadialCatalogStars = 0;
    int matchedSeedRadialCatalogStars = 0;
    double seedRadialMagnitudeSupport = 0.0;
    double matchedSeedRadialMagnitudeSupport = 0.0;
    double seedRadialMagnitudeMatchFraction = 1.0;
    int brightCatalogShapeChecks = 0;
    int brightCatalogShapeMismatches = 0;
    int brightDetections = 0;
    int matchedBrightDetections = 0;
    double brightDetectionMatchFraction = 1.0;
    double brightDetectionMagnitudeError = 0.0;
    double rmsErrorPixels = std::numeric_limits<double>::infinity();
    double medianErrorPixels = std::numeric_limits<double>::infinity();
    double maxErrorPixels = std::numeric_limits<double>::infinity();
    double brightnessRankError = std::numeric_limits<double>::infinity();
    double meanCatalogMagnitude = std::numeric_limits<double>::infinity();
    double magnitudeWeightedSupport = 0.0;
    double priorityMagnitudeWeightedSupport = 0.0;
    double projectedMagnitudeSupport = 0.0;
    double matchedProjectedMagnitudeSupport = 0.0;
    double projectedMagnitudeMatchFraction = 1.0;
    int prioritySeedRadialChecks = 0;
    double prioritySeedRadialErrorPixels = std::numeric_limits<double>::infinity();
    int prioritySeedProjectedChecks = 0;
    double prioritySeedProjectedErrorPixels = std::numeric_limits<double>::infinity();
    int namedBrightAnchorMatches = 0;
    double namedBrightAnchorRmsErrorPixels = std::numeric_limits<double>::infinity();
    double namedBrightAnchorMeanMagnitude = std::numeric_limits<double>::infinity();
    int sparseGuidedNamedAnchorMatches = 0;
    double sparseGuidedAnchorRmsErrorPixels = std::numeric_limits<double>::infinity();
    double sparseGuidedAnchorBrightnessRankError = std::numeric_limits<double>::infinity();
    double sparseGuidedAnchorMeanMagnitude = std::numeric_limits<double>::infinity();
};

struct CandidateRefinementResult
{
    bool seedFinalPassEvaluated = false;
    bool refinedCandidateEvaluated = false;
    bool finalPassEvaluated = false;
    FinalMatchPassEvaluation seedFinalPassEvaluation;
    Evaluation refinedCandidate;
    FinalMatchPassEvaluation finalPassEvaluation;
};

struct TriangleSignature
{
    double ratioShortToLong = 0.0;
    double ratioMidToLong = 0.0;
    double orientation = 0.0;
    double longestDistance = 0.0;
    double areaScore = 0.0;
    double reliabilityScore = 0.0;
    double brightnessScore = 0.0;
    std::array<int, 3> indices{{-1, -1, -1}};
};

struct QuadSignature
{
    std::array<double, 5> edgeRatios{{0.0, 0.0, 0.0, 0.0, 0.0}};
    double orientation = 0.0;
    double longestDistance = 0.0;
    double areaScore = 0.0;
    double reliabilityScore = 0.0;
    double brightnessScore = 0.0;
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
// FoV at/below which the field is treated as a narrow (telescope) view rather than a
// wide-angle/all-sky view. This boundary selects narrow- vs wide-field behaviour
// throughout the solver (search strategy, residual gates, bright-anchor support).
static constexpr double kNarrowFieldMaxFovDegrees = 5.0;
static constexpr int kMaxDetectionsForSolve = 96;
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
bool m_disableRollRecovery = false;
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
QVector<double> m_detectionBrightnessRankCache;
QVector<int> m_detectionBrightnessRankIndices;
QVector<ProjectedCatalogStar> m_projectedCatalogScratch;
QVector<CandidatePair> m_candidatePairScratch;
QVector<CandidatePair> m_cappedCandidatePairScratch;
QVector<BlindGridCachedStar> m_blindGridCache;
QVector<ProjectedCatalogStar> m_blindGridProjectedScratch;
QVector<int> m_projectedStarGridHeadsScratch;
QVector<int> m_projectedStarGridNextScratch;
QVector<int> m_projectedStarGridCellXScratch;
QVector<int> m_projectedStarGridCellYScratch;
QVector<double> m_projectedBrightnessRankScratch;
QVector<int> m_projectedBrightnessRankSortScratch;
QVector<double> m_catalogBrightnessRankScratch;
QVector<int> m_catalogBrightnessRankGeneration;
QVector<int> m_brightnessMatchedDetectionGeneration;
int m_brightnessRankGeneration = 0;
QVector<int> m_detectionMatchGeneration;
QVector<int> m_catalogMatchGeneration;
int m_matchGeneration = 0;
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
static constexpr const char* kSirilRegionCacheDir = "siril-spcc-region-cache/v3";
static constexpr const char* kSirilAstroRegionCacheDir = "siril-astro-region-cache/v3";
static constexpr const char* kBundledCatalogPath = ":/camera/brightstarcatalog.txt";
static constexpr const char* kDownloadedCatalogDir = "camera";
static constexpr const char* kDownloadedCatalogArchiveFile = "hyg_v42.csv.gz";
static constexpr const char* kDownloadedCatalogCsvFile = "hyg_v42.csv";
static constexpr const char* kDownloadedCatalogReducedFile = "hyg_v42_reduced.txt";
static constexpr const char* kSirilSpccBaseUrl = "https://huggingface.co/datasets/siril-spcc/gaia/resolve/main";
static constexpr const char* kSirilSpccZenodoBaseUrl = "https://zenodo.org/records/17988559/files";
static constexpr const char* kSirilSpccFileNamePattern = "siril_cat1_healpix8_xpsamp_%1.dat";
static constexpr const char* kSirilAstroFileName = "siril_cat_healpix8_astro.dat";
static constexpr const char* kSirilAstroCompressedFileName = "siril_cat_healpix8_astro.dat.bz2";
static constexpr int kSirilHealpixLevel = 8;
static constexpr int kSirilNside = 1 << kSirilHealpixLevel;
static constexpr int kSirilChunkLevel = 1;
static constexpr int kSirilPixelsPerChunk = 1 << (2 * (kSirilHealpixLevel - kSirilChunkLevel));
static constexpr int kSirilAstroPixels = 12 * kSirilNside * kSirilNside;
static constexpr int kSirilHeaderSize = 128;
static constexpr int kSirilIndexSize = kSirilPixelsPerChunk * static_cast<int>(sizeof(quint32));
static constexpr int kSirilAstroIndexSize = kSirilAstroPixels * static_cast<int>(sizeof(quint32));
static constexpr int kSirilRecordSize = 701;
static constexpr int kSirilAstroRecordSize = 16;
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

bool isCancellationRequested() const
{
    return m_owner && m_owner->isCancellationRequested();
}

void copySearchStateFrom(const SolverContext& other)
{
    m_weakModeNormalizationPixels = other.m_weakModeNormalizationPixels;
    m_useElevationSeedPreference = other.m_useElevationSeedPreference;
    m_elevationSeedReferenceDegrees = other.m_elevationSeedReferenceDegrees;
    m_elevationSeedReferenceFovDegrees = other.m_elevationSeedReferenceFovDegrees;
    m_elevationSeedScaleDegrees = other.m_elevationSeedScaleDegrees;
    m_elevationSeedFovScaleDegrees = other.m_elevationSeedFovScaleDegrees;
    m_useAllSkyZenithPreference = other.m_useAllSkyZenithPreference;
    m_allSkyZenithReferenceElevationDegrees = other.m_allSkyZenithReferenceElevationDegrees;
    m_allSkyZenithScaleDegrees = other.m_allSkyZenithScaleDegrees;
    m_useDirectionSeedPreference = other.m_useDirectionSeedPreference;
    m_useWideCatalogMagnitudePreference = other.m_useWideCatalogMagnitudePreference;
    m_directionSeedHasRollPreference = other.m_directionSeedHasRollPreference;
    m_directionSeedReferenceAzimuthDegrees = other.m_directionSeedReferenceAzimuthDegrees;
    m_directionSeedReferenceElevationDegrees = other.m_directionSeedReferenceElevationDegrees;
    m_directionSeedReferenceRollDegrees = other.m_directionSeedReferenceRollDegrees;
    m_directionSeedReferenceFovDegrees = other.m_directionSeedReferenceFovDegrees;
    m_directionSeedAzElScaleDegrees = other.m_directionSeedAzElScaleDegrees;
    m_directionSeedRollScaleDegrees = other.m_directionSeedRollScaleDegrees;
    m_directionSeedFovScaleDegrees = other.m_directionSeedFovScaleDegrees;
    m_directionSeedMinMatchCount = other.m_directionSeedMinMatchCount;
    m_useFovSeedPreference = other.m_useFovSeedPreference;
    m_fovSeedReferenceDegrees = other.m_fovSeedReferenceDegrees;
    m_fovSeedScaleDegrees = other.m_fovSeedScaleDegrees;
    m_detectionBrightnessMetricCache = other.m_detectionBrightnessMetricCache;
    m_detectionReliabilityMetricCache = other.m_detectionReliabilityMetricCache;
    m_detectionBrightnessRankCache = other.m_detectionBrightnessRankCache;
    m_detectionBrightnessRankIndices.clear();
}

static qint64 boundedMultiply(qint64 lhs, qint64 rhs)
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

static qint64 estimateRefinementWorkUnits(int candidateCount, int visibleStarCount, int detectionCount)
{
    const qint64 perCandidateWork = boundedMultiply(
        std::max<qint64>(1, visibleStarCount),
        std::max<qint64>(1, detectionCount));
    return boundedMultiply(std::max<qint64>(0, candidateCount), perCandidateWork);
}

static int refinementWorkerThreadCount(int candidateCount, qint64 estimatedWorkUnits)
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

static int visibleCatalogWorkerThreadCount(int catalogStarCount)
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

static constexpr double kSirilAutoMaxFovDegrees = 15.0;
static constexpr double kSirilMaxQueryRadiusDegrees = 20.0;
static constexpr double kSirilAliasMaxSeparationArcSec = 30.0;
static constexpr double kSirilAliasMaxMagnitudeDifference = 2.5;
static constexpr double kWideFovMagnitudePreferenceThresholdDegrees = 30.0;
static constexpr double kWideFovBrightFirstPassMaxMagnitude = 5.0;
static constexpr double kNarrowGuidedBrightCatalogMaxMagnitude = 12.0;
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

static QString sirilAstroCatalogPath()
{
    return QDir(downloadedCatalogDir()).filePath(QString::fromUtf8(kSirilAstroFileName));
}

static QString sirilAstroCompressedCatalogPath()
{
    return QDir(downloadedCatalogDir()).filePath(QString::fromUtf8(kSirilAstroCompressedFileName));
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

static QString formatGaiaCoordinateLabel(double rightAscensionDegrees,
                                         double declinationDegrees,
                                         double magnitude)
{
    constexpr int tenthsPerHour = 60 * 60 * 10;
    constexpr int tenthsPerDay = 24 * tenthsPerHour;
    int totalTenths = static_cast<int>(std::round(normalizeDegrees(rightAscensionDegrees) / 15.0 * tenthsPerHour));
    if (totalTenths >= tenthsPerDay) {
        totalTenths -= tenthsPerDay;
    }
    const int hours = totalTenths / tenthsPerHour;
    totalTenths %= tenthsPerHour;
    const int minutes = totalTenths / (60 * 10);
    const double seconds = (totalTenths % (60 * 10)) / 10.0;

    const double absoluteDeclinationDegrees = std::fabs(declinationDegrees);
    const int degrees = static_cast<int>(std::floor(absoluteDeclinationDegrees));
    const double totalArcMinutes = (absoluteDeclinationDegrees - degrees) * 60.0;
    const int arcMinutes = static_cast<int>(std::floor(totalArcMinutes));
    int arcSeconds = static_cast<int>(std::round((totalArcMinutes - arcMinutes) * 60.0));
    int normalizedArcMinutes = arcMinutes;
    int normalizedDegrees = degrees;
    if (arcSeconds >= 60)
    {
        arcSeconds = 0;
        ++normalizedArcMinutes;
        if (normalizedArcMinutes >= 60)
        {
            normalizedArcMinutes = 0;
            ++normalizedDegrees;
        }
    }

    const QChar declinationSign = declinationDegrees < 0.0 ? QLatin1Char('-') : QLatin1Char('+');
    return QStringLiteral("Gaia J%1%2%3%4%5%6%7 G%8")
        .arg(hours, 2, 10, QLatin1Char('0'))
        .arg(minutes, 2, 10, QLatin1Char('0'))
        .arg(seconds, 4, 'f', 1, QLatin1Char('0'))
        .arg(declinationSign)
        .arg(normalizedDegrees, 2, 10, QLatin1Char('0'))
        .arg(normalizedArcMinutes, 2, 10, QLatin1Char('0'))
        .arg(arcSeconds, 2, 10, QLatin1Char('0'))
        .arg(magnitude, 0, 'f', 1);
}

static bool isGenericGaiaCatalogName(const QString& name)
{
    const QString trimmed = name.trimmed();
    return trimmed.startsWith(QStringLiteral("Gaia Astro "), Qt::CaseInsensitive)
        || trimmed.startsWith(QStringLiteral("Gaia SPCC "), Qt::CaseInsensitive)
        || trimmed.startsWith(QStringLiteral("Gaia J"), Qt::CaseInsensitive);
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
    (void) settings;
    if (QFileInfo::exists(downloadedCatalogReducedPath())) {
        return downloadedCatalogReducedPath();
    }
    return QString::fromUtf8(kBundledCatalogPath);
}

static QString currentCatalogSource(const CameraSettings& settings)
{
    (void) settings;
    return QFileInfo::exists(downloadedCatalogReducedPath())
        ? QStringLiteral("HYG")
        : QStringLiteral("Bundled");
}

static double firstPassPlateSolveMaxMagnitude(const CameraSettings& settings)
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

static double narrowGuidedFullSearchMaxMagnitude(const CameraSettings& settings)
{
    if (isNarrowGuidedDirectionSolve(settings))
    {
        // Mag 18 is a good Gaia depth for narrow guided solving: enough stars
        // for galaxy fields without letting dense faint stars dominate matches.
        return std::min(static_cast<double>(settings.m_plateSolveMaxMagnitude), 18.0);
    }

    return settings.m_plateSolveMaxMagnitude;
}

static bool isLowMagnitudeNarrowGuidedSolve(const CameraSettings& settings)
{
    return plateSolveStartUsesDirection(settings)
        && (isNarrowField(settings))
        && (settings.m_plateSolveMaxMagnitude <= 17.0);
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

static QString sirilAstroRegionCacheRootDir()
{
    return QDir(downloadedCatalogDir()).filePath(QString::fromUtf8(kSirilAstroRegionCacheDir));
}

static QString sirilRegionCacheKey(double centerRaDegrees,
                                   double centerDecDegrees,
                                   double queryRadiusDegrees,
                                   double maxMagnitude)
{
    return QStringLiteral("ra%1_dec%2_r%3_m%4.bin")
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

static QString sirilAstroRegionDiskCachePath(double centerRaDegrees,
                                             double centerDecDegrees,
                                             double queryRadiusDegrees,
                                             double maxMagnitude)
{
    return QDir(sirilAstroRegionCacheRootDir()).filePath(
        sirilRegionCacheKey(centerRaDegrees, centerDecDegrees, queryRadiusDegrees, maxMagnitude));
}

// Binary region cache format (v3, little-endian):
//
//   Header (9 bytes):
//     char[4]   magic      "SRCB"
//     uint8_t   version    1
//     uint32_t  starCount
//
//   Per star (variable length):
//     double    ra         right ascension in degrees
//     double    dec        declination in degrees
//     float     mag        magnitude  (float gives ~7 sig. digits, plenty for mag)
//     uint8_t   nameLen    length of name in bytes  (0–255)
//     char[]    name       UTF-8, nameLen bytes, no null terminator
//     uint8_t   spectralLen
//     char[]    spectralType UTF-8, spectralLen bytes
//
// This format is ~2–4× smaller than the equivalent TSV (no text representation of
// doubles, no line terminators, shorter per-record overhead) and reads with a single
// memcpy-style scan rather than strtod + string parsing.

static constexpr char kBinCacheMagic[4] = {'S', 'R', 'C', 'B'};
static constexpr quint8 kBinCacheVersion = 1;

static QVector<CatalogStar> readSirilRegionDiskCacheFile(const QString& path)
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
    if (isCancellationRequested()) {
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
        if (isCancellationRequested()) {
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
            && !isCancellationRequested())
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

static QByteArray readSirilAstroIndex(QFile& file)
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

static bool sirilAstroCellRecordRange(const QByteArray& indexBytes,
                                      quint32 pixel,
                                      qint64& firstRecord,
                                      qint64& recordCount)
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

QVector<CatalogStar> loadSirilAstroCatalog(const CameraSettings& settings,
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
            qWarning() << "CameraPlateSolver: Siril Gaia astrometric query is too wide, falling back to HYG/bundled"
                       << "radius" << geometry.queryRadiusDegrees
                       << "max" << kSirilMaxQueryRadiusDegrees;
        }
        return stars;
    }

    const QString catalogPath = sirilAstroCatalogPath();
    if (!QFileInfo::exists(catalogPath))
    {
        if (QFileInfo::exists(sirilAstroCompressedCatalogPath()))
        {
            qWarning() << "CameraPlateSolver: Siril Gaia astrometric catalog is compressed; decompress before use"
                       << "compressed" << sirilAstroCompressedCatalogPath()
                       << "expected" << catalogPath;
        }
        else
        {
            qWarning() << "CameraPlateSolver: Siril Gaia astrometric catalog is not installed"
                       << "expected" << catalogPath;
        }
        if (catalogSource) {
            *catalogSource = QStringLiteral("Siril Gaia DR3 Astrometric unavailable");
        }
        return stars;
    }

    const double centerRaDegrees = geometry.centerRaDegrees;
    const double centerDecDegrees = geometry.centerDecDegrees;
    const double queryRadius = geometry.queryRadiusDegrees;
    const QString regionCachePath = sirilAstroRegionDiskCachePath(
        centerRaDegrees,
        centerDecDegrees,
        queryRadius,
        maxMagnitude);
    stars = readSirilRegionDiskCacheFile(regionCachePath);
    if (!stars.isEmpty())
    {
        if (catalogSource) {
            *catalogSource = QStringLiteral("Siril Gaia DR3 Astrometric");
        }
        qDebug() << "CameraPlateSolver: loaded cached Siril Gaia astrometric stars"
                 << stars.size()
                 << "center RA" << centerRaDegrees
                 << "Dec" << centerDecDegrees
                 << "radius" << queryRadius
                 << "maxMag" << maxMagnitude;
        return stars;
    }

    QFile file(catalogPath);
    if (!file.open(QIODevice::ReadOnly))
    {
        qWarning() << "CameraPlateSolver: Siril Gaia astrometric catalog open failed"
                   << "path" << catalogPath
                   << "error" << file.errorString();
        if (catalogSource) {
            *catalogSource = QStringLiteral("Siril Gaia DR3 Astrometric unavailable");
        }
        return stars;
    }

    const QByteArray indexBytes = readSirilAstroIndex(file);
    if (indexBytes.isEmpty())
    {
        if (catalogSource) {
            *catalogSource = QStringLiteral("Siril Gaia DR3 Astrometric unavailable");
        }
        return stars;
    }

    const QSet<quint32> pixels = sampleSirilHealpixPixels(centerRaDegrees, centerDecDegrees, queryRadius);
    QVector<SirilCellRange> cellRanges;
    cellRanges.reserve(pixels.size());
    for (quint32 pixel : pixels)
    {
        qint64 firstRecord = 0;
        qint64 recordCount = 0;
        if (!sirilAstroCellRecordRange(indexBytes, pixel, firstRecord, recordCount) || (recordCount <= 0)) {
            continue;
        }

        const qint64 dataStart = kSirilHeaderSize
            + static_cast<qint64>(kSirilAstroIndexSize)
            + firstRecord * kSirilAstroRecordSize;
        const qint64 recordByteCount = recordCount * kSirilAstroRecordSize;
        cellRanges.append({
            0,
            firstRecord,
            recordCount,
            dataStart,
            dataStart + recordByteCount - 1
        });
    }

    std::sort(cellRanges.begin(), cellRanges.end(), [](const SirilCellRange& lhs, const SirilCellRange& rhs) {
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
            const bool canMerge = (cell.firstByte <= (range.lastByte + kSirilMaxMergedRangeGapBytes + 1))
                && ((mergedLastByte - range.firstByte + 1) <= kSirilMaxMergedRangeRequestSize);
            if (canMerge)
            {
                range.lastByte = mergedLastByte;
                range.cellIndexes.append(cellIndex);
                continue;
            }
        }

        mergedRanges.append({
            0,
            cell.firstByte,
            cell.lastByte,
            QVector<int>{cellIndex}
        });
    }

    QSet<quint64> seenStars;
    stars.reserve(cellRanges.size() * 16);
    qDebug() << "CameraPlateSolver: Siril Gaia astrometric request"
             << "pixels" << pixels.size()
             << "cells" << cellRanges.size()
             << "ranges" << mergedRanges.size()
             << "center RA" << centerRaDegrees
             << "Dec" << centerDecDegrees
             << "radius" << queryRadius
             << "maxMag" << maxMagnitude;

    for (const SirilMergedRange& range : mergedRanges)
    {
        if (isCancellationRequested()) {
            break;
        }
        const qint64 expectedByteCount = range.lastByte - range.firstByte + 1;
        if ((expectedByteCount <= 0) || (expectedByteCount > std::numeric_limits<int>::max())) {
            continue;
        }
        if (!file.seek(range.firstByte))
        {
            qWarning() << "CameraPlateSolver: Siril Gaia astrometric seek failed"
                       << "offset" << range.firstByte
                       << "error" << file.errorString();
            continue;
        }

        const QByteArray recordBytes = file.read(expectedByteCount);
        if (recordBytes.size() != expectedByteCount)
        {
            qWarning() << "CameraPlateSolver: Siril Gaia astrometric record read failed"
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
                const qint64 recordOffset = cellOffset + recordIndex * kSirilAstroRecordSize;
                if ((recordOffset < 0) || ((recordOffset + kSirilAstroRecordSize) > recordBytes.size())) {
                    break;
                }

                const char *record = recordBytes.constData() + recordOffset;
                const qint32 rawRa = qFromLittleEndian<qint32>(reinterpret_cast<const uchar *>(record));
                const qint32 rawDec = qFromLittleEndian<qint32>(reinterpret_cast<const uchar *>(record + 4));
                const qint16 rawMag = qFromLittleEndian<qint16>(reinterpret_cast<const uchar *>(record + 14));
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
                    formatGaiaCoordinateLabel(starRaDegrees, starDecDegrees, magnitude),
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
        *catalogSource = QStringLiteral("Siril Gaia DR3 Astrometric");
    }
    writeSirilRegionDiskCacheFile(regionCachePath, stars);
    qDebug() << "CameraPlateSolver: loaded Siril Gaia astrometric stars"
             << stars.size()
             << "pixels" << pixels.size()
             << "center RA" << centerRaDegrees
             << "Dec" << centerDecDegrees
             << "radius" << queryRadius
             << "maxMag" << maxMagnitude;
    return stars;
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
        if (isCancellationRequested()) {
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
                    formatGaiaCoordinateLabel(starRaDegrees, starDecDegrees, magnitude),
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

// The seed-projected bright-star gate only applies when the user pinned roll (or is
// re-solving the current settings), so the seed orientation -- and therefore the
// projected positions of the bright catalog stars -- can be trusted.
static bool usesSeedProjectedBrightGate(const CameraSettings& settings)
{
    return plateSolveStartUsesRoll(settings) || plateSolveStartUsesCurrentSettingsOnly(settings);
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

// True for narrow (telescope) fields. See kNarrowFieldMaxFovDegrees.
static bool isNarrowField(const CameraSettings& settings)
{
    return settings.m_fov <= kNarrowFieldMaxFovDegrees;
}

// The common "user gave us a direction and it's a narrow telescope field" case that
// many search and acceptance paths special-case.
static bool isNarrowGuidedDirectionSolve(const CameraSettings& settings)
{
    return plateSolveStartUsesDirection(settings) && isNarrowField(settings);
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

static bool canCalibratePrincipalPoint(const CameraSettings& settings)
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
    if (!alias.isEmpty()) {
        return alias;
    }
    if (isGenericGaiaCatalogName(star.name)) {
        return formatGaiaCoordinateLabel(star.rightAscensionDegrees, star.declinationDegrees, star.magnitude);
    }
    return star.name;
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

static bool debugCatalogStarMatches(const PlateSolveCatalogContext& catalogContext,
                                    int catalogIndex)
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

static bool debugTriangleContainsCatalogStar(const PlateSolveCatalogContext& catalogContext,
                                             const std::array<int, 3>& catalogIndices)
{
    for (int catalogIndex : catalogIndices)
    {
        if (debugCatalogStarMatches(catalogContext, catalogIndex)) {
            return true;
        }
    }
    return false;
}

static QString debugTriangleAnchorSummary(const PlateSolveCatalogContext& catalogContext,
                                          const QVector<CameraPipelineStarDetection>& starDetections,
                                          const std::array<int, 3>& detectionIndices,
                                          const std::array<int, 3>& catalogIndices)
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

static SkyVector rotateVectorByMatrix(const std::array<std::array<double, 3>, 3>& rotation,
                                      const SkyVector& vector)
{
    return {
        rotation[0][0] * vector.x + rotation[0][1] * vector.y + rotation[0][2] * vector.z,
        rotation[1][0] * vector.x + rotation[1][1] * vector.y + rotation[1][2] * vector.z,
        rotation[2][0] * vector.x + rotation[2][1] * vector.y + rotation[2][2] * vector.z
    };
}

static std::array<std::array<double, 3>, 3> transposeRotationMatrix(
    const std::array<std::array<double, 3>, 3>& rotation)
{
    return {{
        {{rotation[0][0], rotation[1][0], rotation[2][0]}},
        {{rotation[0][1], rotation[1][1], rotation[2][1]}},
        {{rotation[0][2], rotation[1][2], rotation[2][2]}}
    }};
}

static bool vectorPairRotationError(const std::array<std::array<double, 3>, 3>& rotation,
                                    const std::array<SkyVector, 3>& sourceVectors,
                                    const std::array<SkyVector, 3>& targetVectors,
                                    double& rmsAngularErrorDegrees,
                                    double& maxAngularErrorDegrees)
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

static bool normalizeQuaternion(std::array<double, 4>& quaternion)
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

static std::array<double, 4> largestSymmetric4Eigenvector(const std::array<std::array<double, 4>, 4>& matrix)
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

static std::array<std::array<double, 3>, 3> rotationMatrixFromQuaternion(const std::array<double, 4>& quaternion)
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

static bool wahbaRotationFromVectorPairs(const std::array<SkyVector, 3>& sourceVectors,
                                         const std::array<SkyVector, 3>& targetVectors,
                                         bool transposeCorrelation,
                                         std::array<std::array<double, 3>, 3>& rotation,
                                         double& rmsAngularErrorDegrees,
                                         double& maxAngularErrorDegrees)
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

static bool poseFromThreeVectorPairs(const SkyProjector& baseProjector,
                                     const std::array<SkyVector, 3>& sourceVectors,
                                     const std::array<SkyVector, 3>& targetVectors,
                                     double& azimuthDegrees,
                                     double& elevationDegrees,
                                     double& rollDegrees,
                                     double* rmsAngularErrorDegrees = nullptr,
                                     double* maxAngularErrorDegrees = nullptr)
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

struct GuidedTrianglePoseSeed
{
    double azimuthDegrees = 0.0;
    double elevationDegrees = 0.0;
    double rollDegrees = 0.0;
    double fovDegrees = 0.0;
};

static bool estimateScreenSimilarity(const std::array<QPointF, 3>& sourcePoints,
                                     const std::array<QPointF, 3>& targetPoints,
                                     double& scale,
                                     double& rotationDegrees,
                                     double& rmsErrorPixels)
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

static QVector<GuidedTrianglePoseSeed> guidedTriangleSimilarityPoseSeeds(
    const CameraSettings& settings,
    const QSize& imageSize,
    const std::array<QPointF, 3>& detectionPoints,
    const std::array<VisibleCatalogStar, 3>& triangleStars,
    const std::array<int, 3>& permutation,
    double baseAzimuthDegrees,
    double baseElevationDegrees,
    double seedFovDegrees,
    double centerOffsetXPixels,
    double centerOffsetYPixels,
    double distortionK1)
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

static bool triangleBrightnessOrderCompatible(const QVector<CameraPipelineStarDetection>& starDetections,
                                              const std::array<int, 3>& detectionIndices,
                                              const std::array<VisibleCatalogStar, 3>& triangleStars,
                                              const std::array<int, 3>& permutation)
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

static double triangleBrightnessMagnitudeError(const QVector<CameraPipelineStarDetection>& starDetections,
                                               const std::array<int, 3>& detectionIndices,
                                               const std::array<VisibleCatalogStar, 3>& triangleStars,
                                               const std::array<int, 3>& permutation)
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

static bool isImplausiblyCompactBrightCatalogDetection(const CameraPipelineStarDetection& detection,
                                                       double catalogMagnitude,
                                                       bool narrowGuidedSolve = false)
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

static bool isNamedSparseGuidedCatalogStar(const CatalogStar& star)
{
    const QString displayName = catalogDisplayName(star);
    return displayName.startsWith(QStringLiteral("HIP "), Qt::CaseInsensitive)
        || displayName.startsWith(QStringLiteral("HR "), Qt::CaseInsensitive)
        || displayName.startsWith(QStringLiteral("HD "), Qt::CaseInsensitive);
}

static bool isStrongSparseGuidedDetection(const CameraPipelineStarDetection& detection)
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

static double faintCatalogAssignmentPenalty(double magnitude)
{
    if (!std::isfinite(magnitude)) {
        return 0.0;
    }

    // Gaia/SPCC solves can now include very faint stars. Keep them available,
    // but prefer brighter catalog stars when distance and brightness rank are
    // otherwise comparable, reducing accidental dense-field assignments.
    return std::max(0.0, magnitude - 11.0) * 0.08;
}

static double brightDetectionFaintCatalogAssignmentPenalty(double detectionBrightnessRank,
                                                           double catalogMagnitude,
                                                           bool narrowGuidedSolve)
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

static double catalogMagnitudeSupportWeight(double magnitude)
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

static double matchDistanceSupportWeight(double distancePixels, double matchRadiusPixels)
{
    if (!std::isfinite(distancePixels)) {
        return 0.0;
    }

    const double safeRadius = std::max(1.0, matchRadiusPixels);
    const double normalizedDistance = std::clamp(distancePixels / safeRadius, 0.0, 2.0);
    return 1.0 / (1.0 + normalizedDistance * normalizedDistance);
}

static double narrowGuidedMagnitudePriorityScore(const FinalMatchPassEvaluation& evaluation)
{
    return evaluation.magnitudeWeightedSupport
        + 2.0 * evaluation.priorityMagnitudeWeightedSupport
        + 1.5 * evaluation.matchedProjectedMagnitudeSupport
        + 2.5 * evaluation.matchedSeedProjectedMagnitudeSupport
        + 0.35 * evaluation.matchedSeedRadialMagnitudeSupport;
}

static double prioritySeedRadialAffinity(const FinalMatchPassEvaluation& evaluation,
                                         double matchRadiusPixels)
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

static double prioritySeedProjectedAffinity(const FinalMatchPassEvaluation& evaluation,
                                            double matchRadiusPixels)
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

static double seedRadialMagnitudeCoverageAffinity(const FinalMatchPassEvaluation& evaluation)
{
    if (evaluation.seedRadialMagnitudeSupport <= 0.0) {
        return 1.0;
    }

    const double coverage = std::clamp(evaluation.seedRadialMagnitudeMatchFraction, 0.0, 1.0);
    return 0.05 + 0.95 * coverage * coverage;
}

static double seedProjectedMagnitudeCoverageAffinity(const FinalMatchPassEvaluation& evaluation)
{
    if (evaluation.seedProjectedMagnitudeSupport <= 0.0) {
        return 1.0;
    }

    const double coverage = std::clamp(evaluation.seedProjectedMagnitudeMatchFraction, 0.0, 1.0);
    return 0.08 + 0.92 * coverage * coverage;
}

static bool hasPoorNoRollSeedRadialSupport(const CameraSettings& settings,
                                           const FinalMatchPassEvaluation& evaluation,
                                           bool useSeedProjectedBrightGate)
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

static bool hasComparableSeedRadialChecks(const FinalMatchPassEvaluation& lhs,
                                          const FinalMatchPassEvaluation& rhs)
{
    return (lhs.prioritySeedRadialChecks >= 4)
        && (rhs.prioritySeedRadialChecks >= 4)
        && std::isfinite(lhs.prioritySeedRadialErrorPixels)
        && std::isfinite(rhs.prioritySeedRadialErrorPixels);
}

static bool shouldPreferSeedRadialConsistency(const FinalMatchPassEvaluation& candidate,
                                              const FinalMatchPassEvaluation& best,
                                              int matchTolerance)
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

static double narrowGuidedAnchorShapeScore(const CameraPipelineStarDetection& detection)
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

static double brightGuidedDetectionPriorityScore(const CameraPipelineStarDetection& detection,
                                                 double brightness,
                                                 double reliability,
                                                 double shapeScore)
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

void prepareDetectionMetricCache(const QVector<CameraPipelineStarDetection>& starDetections)
{
    m_detectionBrightnessMetricCache.resize(starDetections.size());
    m_detectionReliabilityMetricCache.resize(starDetections.size());
    m_detectionBrightnessRankCache.resize(starDetections.size());
    std::fill(m_detectionBrightnessRankCache.begin(), m_detectionBrightnessRankCache.end(), 0.5);
    m_detectionBrightnessRankIndices.clear();
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
    const auto appendSpreadCandidate = [&spread, &starDetections, minSpreadSquared](int candidate) {
        const QPointF& candidatePos = starDetections[candidate].m_center;
        for (int selected : spread)
        {
            const double dx = candidatePos.x() - starDetections[selected].m_center.x();
            const double dy = candidatePos.y() - starDetections[selected].m_center.y();
            if ((dx * dx + dy * dy) < minSpreadSquared) {
                return false;
            }
        }
        spread.append(candidate);
        return true;
    };

    QVector<int> broadSaturated;
    broadSaturated.reserve(detectionCount);
    for (int i = 0; i < detectionCount; ++i)
    {
        const CameraPipelineStarDetection& detection = starDetections[i];
        if (!detection.m_hotPixelSuspect
            && detection.m_saturated
            && (detection.m_fwhm >= 3.0f))
        {
            broadSaturated.append(i);
        }
    }
    std::sort(broadSaturated.begin(), broadSaturated.end(), [this, &starDetections](int lhs, int rhs) {
        const double lhsScore = cachedDetectionBrightnessMetric(starDetections, lhs)
            * std::sqrt(std::max(1.0f, starDetections[lhs].m_fwhm));
        const double rhsScore = cachedDetectionBrightnessMetric(starDetections, rhs)
            * std::sqrt(std::max(1.0f, starDetections[rhs].m_fwhm));
        if (!qFuzzyCompare(lhsScore + 1.0, rhsScore + 1.0)) {
            return lhsScore > rhsScore;
        }
        return cachedDetectionReliabilityMetric(starDetections, lhs)
            > cachedDetectionReliabilityMetric(starDetections, rhs);
    });
    const int broadSaturatedLimit = std::min(64, static_cast<int>(broadSaturated.size()));
    for (int i = 0; (i < broadSaturatedLimit) && (spread.size() < kMaxDetectionsForSolve); ++i) {
        appendSpreadCandidate(broadSaturated[i]);
    }

    for (int candidate : ranked) {
        if (spread.contains(candidate)) {
            continue;
        }
        appendSpreadCandidate(candidate);
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

QVector<int> selectDetectionIndicesForBrightGuidedTriangles(const QVector<CameraPipelineStarDetection>& starDetections,
                                                            const QVector<int>& detectionIndices) const
{
    QVector<int> triangleDetections = selectDetectionIndicesForBlindSignatures(
        starDetections,
        detectionIndices,
        16,
        24,
        36);
    QSet<int> seen;
    seen.reserve(triangleDetections.size() + 64);
    for (int detectionIndex : triangleDetections) {
        seen.insert(detectionIndex);
    }

    QVector<int> broadStarLikeDetections;
    broadStarLikeDetections.reserve(starDetections.size());
    for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
    {
        const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
        if (detection.m_hotPixelSuspect) {
            continue;
        }
        if ((detection.m_saturated && (detection.m_fwhm >= 3.0f))
            || (narrowGuidedAnchorShapeScore(detection) >= 35.0))
        {
            broadStarLikeDetections.append(detectionIndex);
        }
    }
    std::sort(broadStarLikeDetections.begin(), broadStarLikeDetections.end(), [this, &starDetections](int lhs, int rhs) {
        const double lhsScore = narrowGuidedAnchorShapeScore(starDetections[lhs])
            + std::min(30.0, std::log1p(cachedDetectionBrightnessMetric(starDetections, lhs)) * 3.0)
            + std::min(20.0, std::log1p(cachedDetectionReliabilityMetric(starDetections, lhs)) * 5.0);
        const double rhsScore = narrowGuidedAnchorShapeScore(starDetections[rhs])
            + std::min(30.0, std::log1p(cachedDetectionBrightnessMetric(starDetections, rhs)) * 3.0)
            + std::min(20.0, std::log1p(cachedDetectionReliabilityMetric(starDetections, rhs)) * 5.0);
        if (!qFuzzyCompare(lhsScore + 1.0, rhsScore + 1.0)) {
            return lhsScore > rhsScore;
        }
        return cachedDetectionBrightnessMetric(starDetections, lhs)
            > cachedDetectionBrightnessMetric(starDetections, rhs);
    });

    for (int detectionIndex : broadStarLikeDetections)
    {
        if (triangleDetections.size() >= 48) {
            break;
        }
        if (seen.contains(detectionIndex)) {
            continue;
        }
        seen.insert(detectionIndex);
        triangleDetections.append(detectionIndex);
    }

    if (triangleDetections.size() > 48) {
        triangleDetections.resize(48);
    }
    return triangleDetections;
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
    // projectVector() applies radial distortion (scale = 1 + k1*r^2) AFTER projection. With
    // barrel distortion (k1 < 0, scale < 1) a star at a larger undistorted angle is pulled
    // radially inward and can still land in bounds, so an undistorted-angle cone would wrongly
    // reject it. The solver sweeps negative k1 (distortion sweep + LM refinement), so the cone is
    // only a guaranteed superset of the in-bounds set when k1 >= 0 (positive/pincushion distortion
    // pushes stars outward, shrinking the in-bounds angular region). Disable the cull otherwise.
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

static QVector<ProjectedCatalogStar> buildProjectedCatalog(const PlateSolveCatalogContext& catalogContext,
                                                    const SkyProjector& projector,
                                                    double searchMarginPixels,
                                                    const QVector<int>* allowedCatalogIndices = nullptr)
{
    QVector<ProjectedCatalogStar> projectedStars;
    buildProjectedCatalogInto(catalogContext, projector, searchMarginPixels, allowedCatalogIndices, projectedStars);
    return projectedStars;
}

// Precomputed per-solve constants for the raDecToAzAlt inner loop.
// All fields depend only on (latitude, longitude, datetime) which are constant
// for every star in a catalog build — so we compute them once and reuse.
struct RaDecToAzAltParams
{
    double sinLat;      // sin(latitude_rad)
    double cosLat;      // cos(latitude_rad)
    double lst_deg;     // local sidereal time in degrees
    double jd;          // Julian date (for precession)
    double jd_from;     // J2000 Julian date
    // Precession rotation matrix row/col [row][col]
    double rot[3][3];
};

static RaDecToAzAltParams buildRaDecToAzAltParams(double latitude,
                                                   double longitude,
                                                   const QDateTime& captureDateTimeUtc)
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

// Fast per-star RaDecToAzAlt using precomputed constants.
// Returns false if the star is below the horizon (or non-finite).
static bool raDecToAzAltFast(const RaDecToAzAltParams& p,
                              double rightAscensionDegrees,
                              double declinationDegrees,
                              double& az,
                              double& alt)
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

static QVector<VisibleCatalogStar> buildVisibleCatalog(const CameraSettings& settings,
                                                const QVector<CatalogStar>& catalogStars,
                                                const QDateTime& captureDateTimeUtc,
                                                double maxMagnitude)
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

static QString visibleCatalogFingerprint(const QVector<CatalogStar>& catalogStars)
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

static double visibleCatalogCacheMagnitudeLimit(const QVector<CatalogStar>& catalogStars,
                                                double maxMagnitude)
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

static QString visibleCatalogCacheKey(const CameraSettings& settings,
                                      const QVector<CatalogStar>& catalogStars,
                                      const QDateTime& captureDateTimeUtc,
                                      double maxMagnitude)
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
    const bool autoSiril = (settings.m_plateSolveCatalogSource == CameraSettings::PlateSolveCatalogAuto)
        && plateSolveStartUsesDirection(settings)
        && (settings.m_fov <= kSirilAutoMaxFovDegrees);
    const bool requestSirilAstro = (settings.m_plateSolveCatalogSource == CameraSettings::PlateSolveCatalogSirilAstroGaia)
        || ((settings.m_plateSolveCatalogSource == CameraSettings::PlateSolveCatalogAuto)
            && autoSiril
            && QFileInfo::exists(sirilAstroCatalogPath()));
    const bool requestSirilSpcc = (settings.m_plateSolveCatalogSource == CameraSettings::PlateSolveCatalogSirilSpccGaia)
        || (autoSiril && !requestSirilAstro);
    if (requestSirilAstro)
    {
        context.catalogStars = loadSirilAstroCatalog(
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
            qWarning() << "CameraPlateSolver: Siril Gaia astrometric catalog did not provide enough stars, falling back to HYG/bundled"
                       << "stars" << context.catalogStars.size()
                       << "minMatches" << settings.m_plateSolveMinMatches;
            context.catalogStars.clear();
            context.catalogSource.clear();
        }
    }

    if (context.catalogStars.isEmpty() && requestSirilSpcc)
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

    populateVisibleCatalogContext(context, settings, captureDateTimeUtc, maxMagnitude, true);
    return context;
}

void rebuildVisibleCatalogContext(PlateSolveCatalogContext& context,
                                  const CameraSettings& settings,
                                  const QDateTime& captureDateTimeUtc,
                                  double maxMagnitude)
{
    populateVisibleCatalogContext(context, settings, captureDateTimeUtc, maxMagnitude, true);
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
    const bool useStartDirection = plateSolveStartUsesDirection(settings);
    const bool useStartRoll = plateSolveStartUsesRoll(settings);
    const bool useWidePlateSolve = isWidePlateSolveContext(settings);
    const bool useFaintNarrowAnchors = useStartDirection
        && !useWidePlateSolve
        && (isNarrowField(settings));
    const bool useWideWeakAnchorSearch = !useStartDirection
        && useWidePlateSolve;
    if ((!useStartDirection && !useWideWeakAnchorSearch)
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
        : (useFaintNarrowAnchors && !useStartRoll)
            ? maxImageDimension * 1.5
        : std::max(
            radialTolerancePixels * 3.0,
            maxImageDimension * 0.35);
    const double anchorMaxMagnitude = std::min(
        settings.m_plateSolveMaxMagnitude,
        useFaintNarrowAnchors ? 10.0 : 7.0);
    QVector<int> allDetectionIndices;
    allDetectionIndices.reserve(starDetections.size());
    for (int i = 0; i < starDetections.size(); ++i) {
        allDetectionIndices.append(i);
    }
    const QVector<double>& detectionRanks = detectionBrightnessRanks(starDetections, allDetectionIndices);

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

            const double detectionBrightnessRank = ((detectionIndex >= 0) && (detectionIndex < detectionRanks.size()))
                ? detectionRanks[detectionIndex]
                : 1.0;
            const bool brightCatalogAnchor = useFaintNarrowAnchors
                ? (visibleStar.magnitude <= 10.0)
                : (visibleStar.magnitude <= 5.0);
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
                detectionBrightnessRank,
                useFaintNarrowAnchors ? narrowGuidedAnchorShapeScore(detection) : 0.0
            });
        }

        std::sort(starAnchors.begin(), starAnchors.end(), [useFaintNarrowAnchors, useStartRoll](const GuidedAnchorPair& lhs, const GuidedAnchorPair& rhs) {
            const bool lhsBright = useFaintNarrowAnchors ? (lhs.magnitude <= 10.0) : (lhs.magnitude <= 5.0);
            const bool rhsBright = useFaintNarrowAnchors ? (rhs.magnitude <= 10.0) : (rhs.magnitude <= 5.0);
            const auto scoreAnchor = [useFaintNarrowAnchors, useStartRoll](const GuidedAnchorPair& anchor, bool bright) {
                if (useFaintNarrowAnchors && !useStartRoll)
                {
                    return anchor.initialDistancePixels * (bright ? 0.05 : 0.18)
                        + anchor.radialErrorPixels * (bright ? 0.03 : 0.10)
                        + (bright ? anchor.detectionBrightnessRank * 60.0 : anchor.detectionBrightnessRank * 120.0)
                        - std::min(120.0, std::log1p(anchor.detectionReliability) * 10.0)
                        - anchor.detectionShapeScore * (bright ? 2.0 : 1.0);
                }

                return bright
                    ? anchor.initialDistancePixels * (useFaintNarrowAnchors ? 0.35 : 0.15)
                        + anchor.radialErrorPixels * (useFaintNarrowAnchors ? 0.15 : 0.05)
                        + anchor.detectionBrightnessRank * (useFaintNarrowAnchors ? 80.0 : 500.0)
                        - std::min(80.0, std::log1p(anchor.detectionReliability) * 8.0)
                        - (useFaintNarrowAnchors ? anchor.detectionShapeScore : 0.0)
                    : anchor.initialDistancePixels
                        + anchor.radialErrorPixels * 0.25
                        - std::min(80.0, std::log1p(anchor.detectionReliability) * 8.0);
            };
            const double lhsScore = scoreAnchor(lhs, lhsBright);
            const double rhsScore = scoreAnchor(rhs, rhsBright);
            if (!qFuzzyCompare(lhsScore + 1.0, rhsScore + 1.0)) {
                return lhsScore < rhsScore;
            }
            return lhs.magnitude < rhs.magnitude;
        });
        const int maxAnchorsForStar = (useFaintNarrowAnchors && !useStartRoll)
            ? (visibleStar.magnitude <= 10.0 ? 16 : 6)
            : (visibleStar.magnitude <= (useFaintNarrowAnchors ? 10.0 : 5.0) ? 6 : 3);
        while (starAnchors.size() > maxAnchorsForStar) {
            starAnchors.removeLast();
        }
        anchors += starAnchors;
    }

    std::sort(anchors.begin(), anchors.end(), [useFaintNarrowAnchors, useStartRoll](const GuidedAnchorPair& lhs, const GuidedAnchorPair& rhs) {
        const bool lhsBright = useFaintNarrowAnchors ? (lhs.magnitude <= 10.0) : (lhs.magnitude <= 5.0);
        const bool rhsBright = useFaintNarrowAnchors ? (rhs.magnitude <= 10.0) : (rhs.magnitude <= 5.0);
        const auto scoreAnchor = [useFaintNarrowAnchors, useStartRoll](const GuidedAnchorPair& anchor, bool bright) {
            if (useFaintNarrowAnchors && !useStartRoll)
            {
                return anchor.magnitude * 45.0
                    + anchor.initialDistancePixels * (bright ? 0.05 : 0.18)
                    + anchor.radialErrorPixels * (bright ? 0.03 : 0.10)
                    + (bright ? anchor.detectionBrightnessRank * 60.0 : anchor.detectionBrightnessRank * 120.0)
                    - std::min(120.0, std::log1p(anchor.detectionReliability) * 10.0)
                    - anchor.detectionShapeScore * (bright ? 2.0 : 1.0);
            }

            return anchor.magnitude * 45.0
                + (bright ? anchor.initialDistancePixels * (useFaintNarrowAnchors ? 0.35 : 0.15) : anchor.initialDistancePixels)
                + (bright ? anchor.radialErrorPixels * (useFaintNarrowAnchors ? 0.15 : 0.05) : anchor.radialErrorPixels * 0.25)
                + (bright ? anchor.detectionBrightnessRank * (useFaintNarrowAnchors ? 80.0 : 500.0) : 0.0)
                - std::min(100.0, std::log1p(anchor.detectionReliability) * 10.0)
                - (useFaintNarrowAnchors ? anchor.detectionShapeScore : 0.0);
        };
        const double lhsScore = scoreAnchor(lhs, lhsBright);
        const double rhsScore = scoreAnchor(rhs, rhsBright);
        if (!qFuzzyCompare(lhsScore + 1.0, rhsScore + 1.0)) {
            return lhsScore < rhsScore;
        }
        if (lhs.catalogIndex != rhs.catalogIndex) {
            return lhs.catalogIndex < rhs.catalogIndex;
        }
        return lhs.detectionIndex < rhs.detectionIndex;
    });
    const int anchorPoolLimit = useWideWeakAnchorSearch ? 96
        : (useFaintNarrowAnchors && !useStartRoll) ? 192
        : useFaintNarrowAnchors ? 96
        : 24;
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
    signature.areaScore = std::clamp(
        std::fabs(signature.orientation) / std::max(1.0, edges[2].length * edges[2].length),
        0.0,
        1.0);

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
    signature.areaScore = std::clamp(
        std::fabs(twiceArea) / std::max(1.0, distances[5] * distances[5]),
        0.0,
        1.0);
    return signature;
}

static bool isDistinctiveTriangleSignature(const TriangleSignature& signature)
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

static bool isDistinctiveQuadSignature(const QuadSignature& signature)
{
    if (signature.longestDistance <= 0.0) {
        return false;
    }
    if (signature.areaScore < 0.025) {
        return false;
    }
    return std::fabs(signature.edgeRatios[1] - signature.edgeRatios[0]) > 0.010;
}

double signatureReliabilityScore(const QVector<CameraPipelineStarDetection>& starDetections,
                                        const int *indices,
                                        int count)
{
    double score = 0.0;
    for (int i = 0; i < count; ++i)
    {
        const int detectionIndex = indices[i];
        if ((detectionIndex < 0) || (detectionIndex >= starDetections.size())) {
            continue;
        }
        score += std::log1p(cachedDetectionReliabilityMetric(starDetections, detectionIndex));
    }
    return score / std::max(1, count);
}

double signatureBrightnessScore(const QVector<CameraPipelineStarDetection>& starDetections,
                                       const int *indices,
                                       int count)
{
    double score = 0.0;
    for (int i = 0; i < count; ++i)
    {
        const int detectionIndex = indices[i];
        if ((detectionIndex < 0) || (detectionIndex >= starDetections.size())) {
            continue;
        }
        score += std::log1p(cachedDetectionBrightnessMetric(starDetections, detectionIndex));
    }
    return score / std::max(1, count);
}

static double catalogSignatureBrightnessScore(const QVector<VisibleCatalogStar>& visibleStars,
                                              const int *indices,
                                              int count)
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

    const int minBlindSeedMatches = (isNarrowGuidedDirectionSolve(settings))
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
int countSeedAnchorSupport(const Evaluation& candidate,
                           const std::array<int, N>& detectionIndices,
                           const std::array<int, N>& catalogIndices)
{
    if (!candidate.valid) {
        return 0;
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

    return matchedAnchors;
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

    const int matchedAnchors = countSeedAnchorSupport(candidate, detectionIndices, catalogIndices);
    return matchedAnchors >= std::min<int>(requiredMatches, static_cast<int>(N));
}

template<size_t N>
int countProjectedAnchorSupport(const CameraSettings& settings,
                                const PlateSolveCatalogContext& catalogContext,
                                const QSize& imageSize,
                                const QVector<CameraPipelineStarDetection>& starDetections,
                                const Evaluation& candidate,
                                const std::array<int, N>& detectionIndices,
                                const std::array<int, N>& catalogIndices,
                                double maxDistancePixels)
{
    if (!candidate.valid) {
        return 0;
    }

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
        return 0;
    }

    const double maxDistanceSquared = maxDistancePixels * maxDistancePixels;
    int matchedAnchors = 0;
    for (size_t anchorIndex = 0; anchorIndex < N; ++anchorIndex)
    {
        const int detectionIndex = detectionIndices[anchorIndex];
        const int catalogIndex = catalogIndices[anchorIndex];
        if ((detectionIndex < 0)
            || (detectionIndex >= starDetections.size())
            || (catalogIndex < 0))
        {
            continue;
        }

        const auto visibleIt = catalogContext.visibleStarIndexByCatalogIndex.constFind(catalogIndex);
        if (visibleIt == catalogContext.visibleStarIndexByCatalogIndex.cend()) {
            continue;
        }

        QPointF projectedPoint;
        if (!projectVector(projector, catalogContext.visibleStars[*visibleIt].vector, projectedPoint)) {
            continue;
        }

        const QPointF delta = projectedPoint - starDetections[detectionIndex].m_center;
        const double distanceSquared = delta.x() * delta.x() + delta.y() * delta.y();
        if (distanceSquared <= maxDistanceSquared) {
            ++matchedAnchors;
        }
    }

    return matchedAnchors;
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

    const int minConsensusMatches = (isNarrowGuidedDirectionSolve(settings))
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

struct SeedConsensusBasin
{
    double azimuthDegrees = 0.0;
    double elevationDegrees = 0.0;
    double rollDegrees = 0.0;
    double fovDegrees = 0.0;
    QVector<Evaluation> candidates;
    QSet<int> detectionSupport;
    QSet<int> catalogSupport;
    double score = 0.0;
};

static double seedWrappedAngularDistanceDegrees(double lhs, double rhs)
{
    double delta = std::fabs(lhs - rhs);
    while (delta > 360.0) {
        delta -= 360.0;
    }
    return delta > 180.0 ? 360.0 - delta : delta;
}

static double seedCandidateConsensusScore(const Evaluation& candidate)
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

QVector<Evaluation> selectConsensusSeedRepresentatives(const CameraSettings& settings,
                                                       const QVector<Evaluation>& seeds,
                                                       int seedLimit,
                                                       const char *profilePrefix)
{
    if (seeds.size() <= std::max(1, seedLimit)) {
        return seeds;
    }

    const double azElThreshold = std::clamp(
        static_cast<double>(settings.m_fov) * 0.12,
        0.25,
        plateSolveStartUsesDirection(settings) ? 2.0 : 6.0);
    const double rollThreshold = plateSolveStartUsesRoll(settings)
        ? std::max(4.0, std::min(12.0, static_cast<double>(settings.m_fov) * 0.20))
        : 15.0;
    const double fovThreshold = std::clamp(
        static_cast<double>(settings.m_fov) * 0.10,
        0.10,
        8.0);

    QVector<SeedConsensusBasin> basins;
    basins.reserve(std::min<int>(static_cast<int>(seeds.size()), 64));
    for (const Evaluation& seed : seeds)
    {
        if (!seed.valid) {
            continue;
        }

        int basinIndex = -1;
        for (int i = 0; i < basins.size(); ++i)
        {
            const SeedConsensusBasin& basin = basins[i];
            if ((seedWrappedAngularDistanceDegrees(seed.azimuthDegrees, basin.azimuthDegrees) <= azElThreshold)
                && (std::fabs(seed.elevationDegrees - basin.elevationDegrees) <= azElThreshold)
                && (seedWrappedAngularDistanceDegrees(seed.rollDegrees, basin.rollDegrees) <= rollThreshold)
                && (std::fabs(seed.fovDegrees - basin.fovDegrees) <= fovThreshold))
            {
                basinIndex = i;
                break;
            }
        }

        if (basinIndex < 0)
        {
            SeedConsensusBasin basin;
            basin.azimuthDegrees = seed.azimuthDegrees;
            basin.elevationDegrees = seed.elevationDegrees;
            basin.rollDegrees = seed.rollDegrees;
            basin.fovDegrees = seed.fovDegrees;
            basins.append(basin);
            basinIndex = basins.size() - 1;
        }

        SeedConsensusBasin& basin = basins[basinIndex];
        basin.candidates.append(seed);
        const double count = static_cast<double>(basin.candidates.size());
        basin.azimuthDegrees += (seed.azimuthDegrees - basin.azimuthDegrees) / count;
        basin.elevationDegrees += (seed.elevationDegrees - basin.elevationDegrees) / count;
        basin.rollDegrees += (seed.rollDegrees - basin.rollDegrees) / count;
        basin.fovDegrees += (seed.fovDegrees - basin.fovDegrees) / count;
        for (const Match& match : seed.matches)
        {
            basin.detectionSupport.insert(match.detectionIndex);
            basin.catalogSupport.insert(match.catalogIndex);
        }
    }

    for (SeedConsensusBasin& basin : basins)
    {
        std::sort(basin.candidates.begin(), basin.candidates.end(), [](const Evaluation& lhs, const Evaluation& rhs) {
            return seedCandidateConsensusScore(lhs) > seedCandidateConsensusScore(rhs);
        });
        const double independentSupport = static_cast<double>(
            std::min<int>(
                static_cast<int>(basin.detectionSupport.size()),
                static_cast<int>(basin.catalogSupport.size())));
        basin.score = basin.candidates.size() * 2.0
            + independentSupport * 3.0
            + (basin.candidates.isEmpty() ? 0.0 : seedCandidateConsensusScore(basin.candidates.first()));
    }

    std::sort(basins.begin(), basins.end(), [](const SeedConsensusBasin& lhs, const SeedConsensusBasin& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score > rhs.score;
        }
        return lhs.candidates.size() > rhs.candidates.size();
    });

    QVector<Evaluation> selected;
    selected.reserve(seedLimit);
    for (const SeedConsensusBasin& basin : basins)
    {
        const int takeCount = std::min<int>(
            static_cast<int>(basin.candidates.size()),
            std::max(1, seedLimit / std::max(1, std::min(4, static_cast<int>(basins.size())))));
        for (int i = 0; (i < takeCount) && (selected.size() < seedLimit); ++i) {
            selected.append(basin.candidates[i]);
        }
        if (selected.size() >= seedLimit) {
            break;
        }
    }

    recordProfileMetric(QStringLiteral("search.%1ConsensusBasins").arg(QString::fromUtf8(profilePrefix)), basins.size());
    recordProfileMetric(QStringLiteral("search.%1ConsensusSelected").arg(QString::fromUtf8(profilePrefix)), selected.size());
    return selected;
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
                if ((signature.longestDistance < 20.0) || !isDistinctiveTriangleSignature(signature)) {
                    continue;
                }
                signature.indices = indices;
                signature.reliabilityScore = signatureReliabilityScore(starDetections, signature.indices.data(), 3);
                signature.brightnessScore = signatureBrightnessScore(starDetections, signature.indices.data(), 3);
                signatures.append(signature);
            }
        }
    }
    std::sort(signatures.begin(), signatures.end(), [](const TriangleSignature& lhs, const TriangleSignature& rhs) {
        const double lhsScore = lhs.areaScore * 2.0 + lhs.reliabilityScore + lhs.brightnessScore * 0.25;
        const double rhsScore = rhs.areaScore * 2.0 + rhs.reliabilityScore + rhs.brightnessScore * 0.25;
        return lhsScore > rhsScore;
    });
    return signatures;
}

bool appendCatalogTriangleSignature(const CameraSettings& settings,
                                    const QVector<VisibleCatalogStar>& visibleStars,
                                    int i,
                                    int j,
                                    int k,
                                    QVector<TriangleSignature>& signatures)
{
    const SkyVector& va = visibleStars[i].vector;
    const SkyVector& vb = visibleStars[j].vector;
    const SkyVector& vc = visibleStars[k].vector;
    const SkyVector center = normalize({va.x + vb.x + vc.x,
                                        va.y + vb.y + vc.y,
                                        va.z + vb.z + vc.z});
    if (length(center) <= 0.0) {
        return false;
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
    // ratios. Compute the maximum pairwise angle so we can scale appropriately.
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
        return false;
    }

    std::array<QPointF, 3> points;
    const std::array<int, 3> starIndices {{i, j, k}};
    for (int pointIndex = 0; pointIndex < 3; ++pointIndex)
    {
        if (!projectVector(localProjector,
                           visibleStars[starIndices[pointIndex]].vector,
                           points[pointIndex]))
        {
            return false;
        }
    }

    TriangleSignature signature = buildTriangleSignature(points);
    if ((signature.longestDistance < 10.0) || !isDistinctiveTriangleSignature(signature)) {
        return false;
    }
    signature.indices = {{i, j, k}};
    signature.reliabilityScore = signature.areaScore;
    signature.brightnessScore = catalogSignatureBrightnessScore(visibleStars, signature.indices.data(), 3);
    signatures.append(signature);
    return true;
}

static qint64 catalogTriangleKey(int a, int b, int c)
{
    std::array<int, 3> sorted {{a, b, c}};
    std::sort(sorted.begin(), sorted.end());
    return (static_cast<qint64>(sorted[0]) << 42)
        | (static_cast<qint64>(sorted[1]) << 21)
        | static_cast<qint64>(sorted[2]);
}

QVector<TriangleSignature> buildLocalCatalogTriangleSignatures(const CameraSettings& settings,
                                                               const QVector<VisibleCatalogStar>& visibleStars)
{
    QVector<TriangleSignature> signatures;
    if (visibleStars.size() < 3) {
        return signatures;
    }

    const double binSizeDegrees = std::clamp(static_cast<double>(settings.m_fov) * 2.5, 2.0, 6.0);
    const int azimuthBinCount = std::max(1, static_cast<int>(std::ceil(360.0 / binSizeDegrees)));
    const int elevationBinCount = std::max(1, static_cast<int>(std::ceil(90.0 / binSizeDegrees)));
    QVector<QVector<int>> bins(azimuthBinCount * elevationBinCount);
    const auto binIndex = [azimuthBinCount, elevationBinCount, binSizeDegrees](double azimuthDegrees, double elevationDegrees) {
        const int azimuthBin = std::clamp(
            static_cast<int>(std::floor(normalizeDegrees(azimuthDegrees) / binSizeDegrees)),
            0,
            azimuthBinCount - 1);
        const int elevationBin = std::clamp(
            static_cast<int>(std::floor(std::clamp(elevationDegrees, 0.0, 89.999999) / binSizeDegrees)),
            0,
            elevationBinCount - 1);
        return elevationBin * azimuthBinCount + azimuthBin;
    };

    for (int i = 0; i < visibleStars.size(); ++i) {
        bins[binIndex(visibleStars[i].azimuthDegrees, visibleStars[i].elevationDegrees)].append(i);
    }

    QSet<qint64> seen;
    const int maxLocalStars = 10;
    for (int elevationBin = 0; elevationBin < elevationBinCount; ++elevationBin)
    {
        for (int azimuthBin = 0; azimuthBin < azimuthBinCount; ++azimuthBin)
        {
            QVector<int> localStars;
            localStars.reserve(maxLocalStars * 4);
            for (int elevationOffset = -1; elevationOffset <= 1; ++elevationOffset)
            {
                const int neighborElevationBin = elevationBin + elevationOffset;
                if ((neighborElevationBin < 0) || (neighborElevationBin >= elevationBinCount)) {
                    continue;
                }
                for (int azimuthOffset = -1; azimuthOffset <= 1; ++azimuthOffset)
                {
                    const int neighborAzimuthBin = (azimuthBin + azimuthOffset + azimuthBinCount) % azimuthBinCount;
                    const QVector<int>& cellStars = bins[neighborElevationBin * azimuthBinCount + neighborAzimuthBin];
                    const int takeCount = std::min(maxLocalStars, static_cast<int>(cellStars.size()));
                    for (int i = 0; i < takeCount; ++i) {
                        localStars.append(cellStars[i]);
                    }
                }
            }

            std::sort(localStars.begin(), localStars.end(), [&visibleStars](int lhs, int rhs) {
                return visibleStars[lhs].magnitude < visibleStars[rhs].magnitude;
            });
            QSet<int> localSeen;
            QVector<int> uniqueLocalStars;
            uniqueLocalStars.reserve(localStars.size());
            for (int starIndex : localStars)
            {
                if (localSeen.contains(starIndex)) {
                    continue;
                }
                localSeen.insert(starIndex);
                uniqueLocalStars.append(starIndex);
            }
            localStars = uniqueLocalStars;
            if (localStars.size() > maxLocalStars) {
                localStars.resize(maxLocalStars);
            }

            for (int i = 0; i < localStars.size(); ++i)
            {
                for (int j = i + 1; j < localStars.size(); ++j)
                {
                    for (int k = j + 1; k < localStars.size(); ++k)
                    {
                        const int starI = localStars[i];
                        const int starJ = localStars[j];
                        const int starK = localStars[k];
                        const qint64 key = catalogTriangleKey(starI, starJ, starK);
                        if (seen.contains(key)) {
                            continue;
                        }
                        seen.insert(key);
                        appendCatalogTriangleSignature(settings, visibleStars, starI, starJ, starK, signatures);
                    }
                }
            }
        }
    }

    std::sort(signatures.begin(), signatures.end(), [](const TriangleSignature& lhs, const TriangleSignature& rhs) {
        const double lhsScore = lhs.areaScore * 2.0 + lhs.brightnessScore;
        const double rhsScore = rhs.areaScore * 2.0 + rhs.brightnessScore;
        return lhsScore > rhsScore;
    });
    return signatures;
}

QVector<TriangleSignature> buildCatalogTriangleSignatures(const CameraSettings& settings,
                                                          const QVector<VisibleCatalogStar>& visibleStars,
                                                          int maxCatalogStarCount = -1)
{
    if (plateSolveStartUsesFov(settings)
        && !plateSolveStartUsesElevation(settings)
        && !plateSolveStartUsesDirection(settings)
        && !isWidePlateSolveContext(settings)
        && (settings.m_fov < 15.0))
    {
        return buildLocalCatalogTriangleSignatures(settings, visibleStars);
    }

    QVector<TriangleSignature> signatures;
    const int requestedMaxCatalogStars = (maxCatalogStarCount > 0) ? maxCatalogStarCount : 32;
    const int maxCatalogStars = std::min<int>(requestedMaxCatalogStars, static_cast<int>(visibleStars.size()));
    for (int i = 0; i < maxCatalogStars; ++i)
    {
        for (int j = i + 1; j < maxCatalogStars; ++j)
        {
            for (int k = j + 1; k < maxCatalogStars; ++k)
            {
                appendCatalogTriangleSignature(settings, visibleStars, i, j, k, signatures);
            }
        }
    }
    std::sort(signatures.begin(), signatures.end(), [](const TriangleSignature& lhs, const TriangleSignature& rhs) {
        const double lhsScore = lhs.areaScore * 2.0 + lhs.brightnessScore;
        const double rhsScore = rhs.areaScore * 2.0 + rhs.brightnessScore;
        return lhsScore > rhsScore;
    });
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
                    if ((signature.longestDistance < 30.0) || !isDistinctiveQuadSignature(signature)) {
                        continue;
                    }
                    signature.indices = indices;
                    signature.reliabilityScore = signatureReliabilityScore(starDetections, signature.indices.data(), 4);
                    signature.brightnessScore = signatureBrightnessScore(starDetections, signature.indices.data(), 4);
                    signatures.append(signature);
                }
            }
        }
    }
    std::sort(signatures.begin(), signatures.end(), [](const QuadSignature& lhs, const QuadSignature& rhs) {
        const double lhsScore = lhs.areaScore * 2.0 + lhs.reliabilityScore + lhs.brightnessScore * 0.25;
        const double rhsScore = rhs.areaScore * 2.0 + rhs.reliabilityScore + rhs.brightnessScore * 0.25;
        return lhsScore > rhsScore;
    });
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
                    if ((signature.longestDistance < 15.0) || !isDistinctiveQuadSignature(signature)) {
                        continue;
                    }
                    signature.indices = {{i, j, k, l}};
                    signature.reliabilityScore = signature.areaScore;
                    signature.brightnessScore = catalogSignatureBrightnessScore(visibleStars, signature.indices.data(), 4);
                    signatures.append(signature);
                }
            }
        }
    }
    std::sort(signatures.begin(), signatures.end(), [](const QuadSignature& lhs, const QuadSignature& rhs) {
        const double lhsScore = lhs.areaScore * 2.0 + lhs.brightnessScore;
        const double rhsScore = rhs.areaScore * 2.0 + rhs.brightnessScore;
        return lhsScore > rhsScore;
    });
    return signatures;
}

QVector<Evaluation> buildBlindTriangleSeeds(const CameraSettings& settings,
                                            const PlateSolveCatalogContext& catalogContext,
                                            const QSize& imageSize,
                                            const QDateTime& captureDateTimeUtc,
                                            const QVector<CameraPipelineStarDetection>& starDetections,
                                            const QVector<int>& detectionIndices,
                                            const QVector<VisibleCatalogStar>& visibleStars,
                                            const QVector<int>* signatureDetectionIndicesOverride = nullptr,
                                            int maxDetectionSignatureCount = -1,
                                            int maxCatalogSignatureStars = -1,
                                            int requiredAnchorMatches = 2,
                                            int seedLimitOverride = -1)
{
    QVector<Evaluation> seeds;
    if (isCancellationRequested() || (visibleStars.size() < settings.m_plateSolveMinMatches)) {
        return seeds;
    }

    const bool isWideFisheyeLens = isWidePlateSolveContext(settings);
    const QVector<int> defaultSignatureDetectionIndices = isWideFisheyeLens
        ? selectDetectionIndicesForBlindSignatures(starDetections, detectionIndices, 12, 12, 20)
        : detectionIndices;
    const QVector<int>& signatureDetectionIndices = signatureDetectionIndicesOverride
        ? *signatureDetectionIndicesOverride
        : defaultSignatureDetectionIndices;
    const int signatureDetectionCount = (maxDetectionSignatureCount > 0)
        ? maxDetectionSignatureCount
        : isWideFisheyeLens ? 20 : 16;
    const QVector<TriangleSignature> detectionTriangles = buildDetectionTriangleSignatures(
        starDetections,
        signatureDetectionIndices,
        signatureDetectionCount);
    if (isCancellationRequested()) {
        return seeds;
    }
    const QVector<TriangleSignature> catalogTriangles = buildCatalogTriangleSignatures(
        settings,
        visibleStars,
        maxCatalogSignatureStars);
    if (isCancellationRequested() || detectionTriangles.isEmpty() || catalogTriangles.isEmpty()) {
        return seeds;
    }
    const bool brightGuidedTriangleMode = signatureDetectionIndicesOverride
        && plateSolveStartUsesDirection(settings)
        && (isNarrowField(settings))
        && !isWideFisheyeLens;
    const double ratioTolerance = brightGuidedTriangleMode
        ? 0.055
        : (isNarrowGuidedDirectionSolve(settings))
        ? 0.08
        : isWideFisheyeLens ? 0.08
        : kBlindSeedRatioTolerance;
    const bool ignoreOrientationHandedness = isWideFisheyeLens
        || (isNarrowGuidedDirectionSolve(settings));
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
    qint64 seedValidEvaluations = 0;
    qint64 seedAnchorSupported = 0;
    int maxSeedAnchorMatches = 0;
    qint64 verifiedSeeds = 0;
    QVector<quint8> baseDetectionSelected(starDetections.size(), 0);
    for (int detectionIndex : detectionIndices)
    {
        if ((detectionIndex >= 0) && (detectionIndex < baseDetectionSelected.size())) {
            baseDetectionSelected[detectionIndex] = 1;
        }
    }

    for (const TriangleSignature& detectionTriangle : detectionTriangles)
    {
        if (earlyExit || isCancellationRequested()) break;
        QVector<int> seedDetectionIndices;
        const QVector<int>* seedDetectionIndicesPtr = &detectionIndices;
        for (int detectionIndex : detectionTriangle.indices)
        {
            if ((detectionIndex < 0)
                || (detectionIndex >= baseDetectionSelected.size())
                || baseDetectionSelected[detectionIndex])
            {
                continue;
            }
            if (seedDetectionIndices.isEmpty()) {
                seedDetectionIndices = detectionIndices;
            }
            if (!seedDetectionIndices.contains(detectionIndex)) {
                seedDetectionIndices.append(detectionIndex);
            }
        }
        if (!seedDetectionIndices.isEmpty()) {
            seedDetectionIndicesPtr = &seedDetectionIndices;
        }

        const int detectionShortBin = signatureRatioBin(detectionTriangle.ratioShortToLong, ratioTolerance);
        const int detectionMidBin = signatureRatioBin(detectionTriangle.ratioMidToLong, ratioTolerance);
        for (int shortBinOffset = -bucketRadius; shortBinOffset <= bucketRadius; ++shortBinOffset)
        {
            if (earlyExit || isCancellationRequested()) break;
            for (int midBinOffset = -bucketRadius; midBinOffset <= bucketRadius; ++midBinOffset)
            {
                if (earlyExit || isCancellationRequested()) break;
                const auto bucketIt = catalogTriangleBuckets.constFind(
                    signatureBucketKey(detectionShortBin + shortBinOffset, detectionMidBin + midBinOffset));
                if (bucketIt == catalogTriangleBuckets.constEnd()) {
                    continue;
                }
                for (int catalogTriangleIndex : *bucketIt)
                {
                    if (earlyExit || isCancellationRequested()) break;
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

            const double seedAzimuth = brightGuidedTriangleMode
                ? static_cast<double>(settings.m_azimuth)
                : normalizeDegrees(std::atan2(center.x, center.y) * 180.0 / kPi);
            const double seedElevation = brightGuidedTriangleMode
                ? static_cast<double>(settings.m_elevation)
                : std::asin(std::clamp(center.z, -1.0, 1.0)) * 180.0 / kPi;

            // Use the longest pairwise angular distance so the scale estimate matches
            // detectionTriangle.longestDistance regardless of which edge is longest.
            const double abAngle = std::acos(std::clamp(dot(a.vector, b.vector), -1.0, 1.0)) * 180.0 / kPi;
            const double acAngle = std::acos(std::clamp(dot(a.vector, c.vector), -1.0, 1.0)) * 180.0 / kPi;
            const double bcAngle = std::acos(std::clamp(dot(b.vector, c.vector), -1.0, 1.0)) * 180.0 / kPi;
            const double catalogAngularDistance = std::max({abAngle, acAngle, bcAngle});
            if (catalogAngularDistance <= 0.01) {
                continue;
            }

            const double triangleSeedFov = std::clamp(
                catalogAngularDistance * static_cast<double>(std::max(imageSize.width(), imageSize.height())) / std::max(1.0, detectionTriangle.longestDistance),
                static_cast<double>(CameraSettings::m_minFov),
                static_cast<double>(CameraSettings::m_maxFov));
            const double baseSeedFov = brightGuidedTriangleMode
                ? static_cast<double>(settings.m_fov)
                : triangleSeedFov;
            if (!seedFovCompatibleWithStartFov(settings, baseSeedFov)) {
                continue;
            }

            // Skip sky directions already tried by a previous triangle match. The dedup
            // radius scales with the seed FoV: at 90° FoV the original 3° tolerance is fine,
            // but at 15-25° wide-field blind FoVs a 3° basin can swallow the correct
            // direction after a near-miss. Use 5% of seed FoV with a 0.5°-5° clamp.
            if (!brightGuidedTriangleMode)
            {
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
            }

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
                if (earlyExit || isCancellationRequested()) break;
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

                const SkyProjector sourceProjector = createProjector(
                    settings,
                    imageSize,
                    0.0,
                    90.0,
                    0.0,
                    seedFov,
                    fixedCenterOffsetX,
                    fixedCenterOffsetY,
                    fixedDistortionK1);
                SkyVector sourceA;
                SkyVector sourceB;
                std::array<SkyVector, 3> sourceTriangleVectors;
                bool haveSourceTriangle = sourceProjector.valid;
                if (haveSourceTriangle)
                {
                    for (int idx = 0; idx < 3; ++idx)
                    {
                        if (!unprojectPixelToVector(sourceProjector, detectionPoints[idx], sourceTriangleVectors[idx]))
                        {
                            haveSourceTriangle = false;
                            break;
                        }
                    }
                }
                const bool haveSourcePair = sourceProjector.valid
                    && unprojectPixelToVector(sourceProjector, detectionPoints[0], sourceA)
                    && unprojectPixelToVector(sourceProjector, detectionPoints[1], sourceB);

                for (const std::array<int, 3>& permutation : kTrianglePermutations)
                {
                    if (earlyExit || isCancellationRequested()) break;
                    const bool brightnessOrderCompatible = triangleBrightnessOrderCompatible(
                            starDetections,
                            detectionTriangle.indices,
                            triangleStars,
                            permutation);
                    if (brightGuidedTriangleMode && !brightnessOrderCompatible)
                    {
                        continue;
                    }
                    if (!brightGuidedTriangleMode)
                    {
                        const double brightnessMagnitudeError = triangleBrightnessMagnitudeError(
                            starDetections,
                            detectionTriangle.indices,
                            triangleStars,
                            permutation);
                        if (!brightnessOrderCompatible && (brightnessMagnitudeError > 2.75)) {
                            continue;
                        }
                    }

                    QVector<GuidedTrianglePoseSeed> poseSeeds;
                    if (brightGuidedTriangleMode)
                    {
                        poseSeeds = guidedTriangleSimilarityPoseSeeds(
                            settings,
                            imageSize,
                            detectionPoints,
                            triangleStars,
                            permutation,
                            seedAzimuth,
                            seedElevation,
                            seedFov,
                            fixedCenterOffsetX,
                            fixedCenterOffsetY,
                            fixedDistortionK1);
                        if (!haveSourceTriangle) {
                            if (poseSeeds.isEmpty()) {
                                continue;
                            }
                        }
                        else
                        {
                            GuidedTrianglePoseSeed wahbaSeed;
                            wahbaSeed.azimuthDegrees = seedAzimuth;
                            wahbaSeed.elevationDegrees = seedElevation;
                            wahbaSeed.rollDegrees = 0.0;
                            wahbaSeed.fovDegrees = seedFov;
                            const std::array<SkyVector, 3> targetTriangleVectors {{
                                triangleStars[permutation[0]].vector,
                                triangleStars[permutation[1]].vector,
                                triangleStars[permutation[2]].vector
                            }};
                            double triangleRmsAngularError = std::numeric_limits<double>::infinity();
                            double triangleMaxAngularError = std::numeric_limits<double>::infinity();
                            if (poseFromThreeVectorPairs(
                                    sourceProjector,
                                    sourceTriangleVectors,
                                    targetTriangleVectors,
                                    wahbaSeed.azimuthDegrees,
                                    wahbaSeed.elevationDegrees,
                                    wahbaSeed.rollDegrees,
                                    &triangleRmsAngularError,
                                    &triangleMaxAngularError))
                            {
                                Q_UNUSED(triangleRmsAngularError)
                                Q_UNUSED(triangleMaxAngularError)
                                poseSeeds.append(wahbaSeed);
                            }
                        }
                        if (poseSeeds.isEmpty()) {
                            continue;
                        }
                    }
                    else
                    {
                        GuidedTrianglePoseSeed poseSeed;
                        poseSeed.azimuthDegrees = seedAzimuth;
                        poseSeed.elevationDegrees = seedElevation;
                        poseSeed.rollDegrees = 0.0;
                        poseSeed.fovDegrees = seedFov;
                        if (haveSourcePair)
                        {
                            if (!poseFromTwoVectorPairs(
                                    sourceProjector,
                                    sourceA,
                                    sourceB,
                                    triangleStars[permutation[0]].vector,
                                    triangleStars[permutation[1]].vector,
                                    poseSeed.azimuthDegrees,
                                    poseSeed.elevationDegrees,
                                    poseSeed.rollDegrees))
                            {
                                continue;
                            }
                        }
                        else
                        {
                            const QLineF detectionBase(detectionPoints[0], detectionPoints[1]);
                            const QLineF projectedBase(projectedPoints[permutation[0]], projectedPoints[permutation[1]]);
                            poseSeed.rollDegrees = projectedBase.angleTo(detectionBase);
                            if (!std::isfinite(poseSeed.rollDegrees)) {
                                poseSeed.rollDegrees = 0.0;
                            }
                        }
                        poseSeeds.append(poseSeed);
                    }

                    for (const GuidedTrianglePoseSeed& poseSeed : poseSeeds)
                    {
                        if (earlyExit || isCancellationRequested()) break;
                        // Sweep small roll perturbations to tolerate centroiding noise on the reference edge.
                        for (double rollDelta : {-10.0, -5.0, 0.0, 5.0, 10.0})
                        {
                            if (earlyExit || isCancellationRequested()) break;
                            const double seedRoll = poseSeed.rollDegrees + rollDelta;

                            const Evaluation seededCandidate = evaluatePose(
                                settings,
                                catalogContext,
                                imageSize,
                                captureDateTimeUtc,
                                starDetections,
                                *seedDetectionIndicesPtr,
                                poseSeed.azimuthDegrees,
                                poseSeed.elevationDegrees,
                                seedRoll,
                                poseSeed.fovDegrees,
                                &allowedCatalogIndices,
                                fixedCenterOffsetX,
                                fixedCenterOffsetY,
                                fixedDistortionK1);
                            ++seedEvaluations;
                            if (brightGuidedTriangleMode && (seedEvaluations >= 4000)) {
                                earlyExit = true;
                            }
                            if (!seededCandidate.valid) {
                                continue;
                            }
                            ++seedValidEvaluations;
                            const std::array<int, 3> anchorCatalogIndices {{
                                triangleStars[permutation[0]].catalogIndex,
                                triangleStars[permutation[1]].catalogIndex,
                                triangleStars[permutation[2]].catalogIndex
                            }};
                            const double anchorDistancePixels = brightGuidedTriangleMode
                                ? std::min(
                                    128.0,
                                    std::max(
                                        static_cast<double>(settings.m_plateSolveMatchRadius),
                                        static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 4.0))
                                : std::min(
                                    static_cast<double>(settings.m_plateSolveMatchRadius),
                                    std::max(6.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius)));
                            const int seedAnchorMatches = countProjectedAnchorSupport(
                                settings,
                                catalogContext,
                                imageSize,
                                starDetections,
                                seededCandidate,
                                detectionTriangle.indices,
                                anchorCatalogIndices,
                                anchorDistancePixels);
                            maxSeedAnchorMatches = std::max(maxSeedAnchorMatches, seedAnchorMatches);
                            if (seedAnchorMatches < std::min<int>(requiredAnchorMatches, 3)) {
                                continue;
                            }
                            ++seedAnchorSupported;

                            Evaluation candidate = evaluatePose(
                                settings,
                                catalogContext,
                                imageSize,
                                captureDateTimeUtc,
                                starDetections,
                                *seedDetectionIndicesPtr,
                                seededCandidate.azimuthDegrees,
                                seededCandidate.elevationDegrees,
                                seededCandidate.rollDegrees,
                                seededCandidate.fovDegrees,
                                nullptr,
                                fixedCenterOffsetX,
                                fixedCenterOffsetY,
                                fixedDistortionK1);
                            if (brightGuidedTriangleMode && candidate.valid)
                            {
                                candidate.anchored = true;
                                candidate.guidedTriangle = true;
                                candidate.anchorDetectionIndex = detectionTriangle.indices[0];
                                candidate.anchorCatalogIndex = anchorCatalogIndices[0];
                                candidate.secondaryAnchorDetectionIndex = detectionTriangle.indices[1];
                                candidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
                                candidate.tertiaryAnchorDetectionIndex = detectionTriangle.indices[2];
                                candidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];
                                seeds.append(candidate);
                            }
                            const Evaluation verifiedCandidate = verifyBlindSeedCandidate(
                                settings,
                                catalogContext,
                                imageSize,
                                captureDateTimeUtc,
                                starDetections,
                                *seedDetectionIndicesPtr,
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
                        if (brightGuidedTriangleMode && (seedEvaluations >= 4000))
                        {
                            earlyExit = true;
                            break;
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
    recordProfileMetric(QStringLiteral("search.triangleSeedValid"), seedValidEvaluations);
    recordProfileMetric(QStringLiteral("search.triangleSeedAnchorSupported"), seedAnchorSupported);
    recordProfileMetric(QStringLiteral("search.triangleSeedMaxAnchors"), maxSeedAnchorMatches);
    recordProfileMetric(QStringLiteral("search.triangleVerifiedSeeds"), verifiedSeeds);

    std::sort(seeds.begin(), seeds.end(), [this](const Evaluation& lhs, const Evaluation& rhs) {
        return isBetterWeakModeEvaluation(lhs, rhs);
    });
    const int seedLimit = (seedLimitOverride > 0)
        ? seedLimitOverride
        : isWideFisheyeLens ? 64 : 16;
    return selectConsensusSeedRepresentatives(settings, seeds, seedLimit, "triangle");
}

QVector<Evaluation> buildBrightGuidedTriangleSeeds(const CameraSettings& settings,
                                                   const PlateSolveCatalogContext& catalogContext,
                                                   const QSize& imageSize,
                                                   const QDateTime& captureDateTimeUtc,
                                                   const QVector<CameraPipelineStarDetection>& starDetections,
                                                   const QVector<int>& detectionIndices,
                                                   const QVector<VisibleCatalogStar>& visibleStars)
{
    if (isCancellationRequested()
        || !plateSolveStartUsesDirection(settings)
        || isWidePlateSolveContext(settings)
        || (!isNarrowField(settings))
        || (starDetections.size() < 3)
        || (visibleStars.size() < 3))
    {
        return {};
    }

    const QVector<int> triangleDetectionIndices = selectDetectionIndicesForBrightGuidedTriangles(
        starDetections,
        detectionIndices);
    if (triangleDetectionIndices.size() < 3) {
        return {};
    }

    QVector<VisibleCatalogStar> triangleCatalogStars;
    triangleCatalogStars.reserve(std::min(64, static_cast<int>(visibleStars.size())));
    const double maxMagnitude = static_cast<double>(settings.m_plateSolveMaxMagnitude);
    for (const VisibleCatalogStar& visibleStar : visibleStars)
    {
        if (visibleStar.magnitude > maxMagnitude) {
            break;
        }
        triangleCatalogStars.append(visibleStar);
        if (triangleCatalogStars.size() >= 64) {
            break;
        }
    }
    if (triangleCatalogStars.size() < 3) {
        return {};
    }

    if (qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE"))
    {
        qDebug() << "CameraPlateSolver: bright-triangle pools"
                 << "detections" << triangleDetectionIndices.size()
                 << "catalog" << triangleCatalogStars.size();
    }

    return buildBlindTriangleSeeds(
        settings,
        catalogContext,
        imageSize,
        captureDateTimeUtc,
        starDetections,
        detectionIndices,
        triangleCatalogStars,
        &triangleDetectionIndices,
        32,
        48,
        3,
        48);
}

QVector<Evaluation> buildBrightGuidedAnchorTriangleSeeds(const CameraSettings& settings,
                                                         const PlateSolveCatalogContext& catalogContext,
                                                         const QSize& imageSize,
                                                         const QDateTime& captureDateTimeUtc,
                                                         const QVector<CameraPipelineStarDetection>& starDetections,
                                                         const QVector<int>& detectionIndices,
                                                         const QVector<VisibleCatalogStar>& visibleStars)
{
    QVector<Evaluation> seeds;
    if (isCancellationRequested()
        || !plateSolveStartUsesDirection(settings)
        || isWidePlateSolveContext(settings)
        || (!isNarrowField(settings))
        || (starDetections.size() < 3)
        || (visibleStars.size() < 3))
    {
        return seeds;
    }

    const bool useStartLens = plateSolveStartUsesLens(settings);
    const double fixedCenterOffsetX = useStartLens ? settings.m_lensCenterOffsetX : 0.0;
    const double fixedCenterOffsetY = useStartLens ? settings.m_lensCenterOffsetY : 0.0;
    const double fixedDistortionK1 = useStartLens ? settings.m_lensDistortionK1 : 0.0;
    const SkyProjector radialProjector = createProjector(
        settings,
        imageSize,
        settings.m_azimuth,
        settings.m_elevation,
        0.0,
        settings.m_fov,
        fixedCenterOffsetX,
        fixedCenterOffsetY,
        fixedDistortionK1);
    if (!radialProjector.valid) {
        return seeds;
    }

    QVector<int> allDetectionIndices;
    allDetectionIndices.reserve(starDetections.size());
    for (int i = 0; i < starDetections.size(); ++i) {
        allDetectionIndices.append(i);
    }
    const QVector<double>& detectionRanks = detectionBrightnessRanks(starDetections, allDetectionIndices);

    struct AnchorCandidate
    {
        int detectionIndex = -1;
        int catalogIndex = -1;
        VisibleCatalogStar catalogStar;
        double score = 0.0;
        double radialErrorPixels = 0.0;
        double magnitude = 0.0;
    };

    const QPointF principalPoint = projectorPrincipalPoint(radialProjector);
    const double maxImageDimension = std::max(imageSize.width(), imageSize.height());
    const double radialTolerancePixels = std::max(
        static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 12.0,
        maxImageDimension * 0.24);
    const double maxCatalogMagnitude = static_cast<double>(settings.m_plateSolveMaxMagnitude);
    const int maxCatalogStars = 120;
    const double maxFrameRadius = std::max(
        std::max(pointDistancePixels(QPointF(0.0, 0.0), principalPoint),
                 pointDistancePixels(QPointF(imageSize.width(), 0.0), principalPoint)),
        std::max(pointDistancePixels(QPointF(0.0, imageSize.height()), principalPoint),
                 pointDistancePixels(QPointF(imageSize.width(), imageSize.height()), principalPoint)));

    struct ProjectedAnchorCatalogStar
    {
        VisibleCatalogStar visibleStar;
        QPointF projectedPoint;
        double projectedRadius = 0.0;
    };
    QVector<ProjectedAnchorCatalogStar> anchorCatalogStars;
    anchorCatalogStars.reserve(maxCatalogStars * 2);
    int anchorCatalogScanned = 0;
    int anchorCatalogProjected = 0;
    for (const VisibleCatalogStar& visibleStar : visibleStars)
    {
        if (isCancellationRequested()) {
            return seeds;
        }
        if (visibleStar.magnitude > maxCatalogMagnitude) {
            break;
        }
        ++anchorCatalogScanned;

        QPointF projectedPoint;
        if (!projectVector(radialProjector, visibleStar.vector, projectedPoint)) {
            continue;
        }
        ++anchorCatalogProjected;

        const double projectedRadius = pointDistancePixels(projectedPoint, principalPoint);
        if (projectedRadius > (maxFrameRadius + radialTolerancePixels)) {
            continue;
        }

        anchorCatalogStars.append({visibleStar, projectedPoint, projectedRadius});
    }
    std::sort(anchorCatalogStars.begin(), anchorCatalogStars.end(), [](const ProjectedAnchorCatalogStar& lhs, const ProjectedAnchorCatalogStar& rhs) {
        if (!qFuzzyCompare(lhs.visibleStar.magnitude + 1.0, rhs.visibleStar.magnitude + 1.0)) {
            return lhs.visibleStar.magnitude < rhs.visibleStar.magnitude;
        }
        if (!qFuzzyCompare(lhs.projectedRadius + 1.0, rhs.projectedRadius + 1.0)) {
            return lhs.projectedRadius < rhs.projectedRadius;
        }
        return lhs.visibleStar.catalogIndex < rhs.visibleStar.catalogIndex;
    });
    if (anchorCatalogStars.size() > maxCatalogStars) {
        anchorCatalogStars.resize(maxCatalogStars);
    }
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleCatalogStars"), anchorCatalogStars.size());
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleCatalogScanned"), anchorCatalogScanned);
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleCatalogProjected"), anchorCatalogProjected);

    QVector<AnchorCandidate> anchors;
    anchors.reserve(maxCatalogStars * 6);
    for (const ProjectedAnchorCatalogStar& projectedCatalogStar : anchorCatalogStars)
    {
        if (isCancellationRequested()) {
            return seeds;
        }
        const VisibleCatalogStar& visibleStar = projectedCatalogStar.visibleStar;

        const bool namedCatalogStar = (visibleStar.catalogIndex >= 0)
            && (visibleStar.catalogIndex < catalogContext.catalogStars.size())
            && isNamedSparseGuidedCatalogStar(catalogContext.catalogStars[visibleStar.catalogIndex]);
        const double projectedRadius = projectedCatalogStar.projectedRadius;
        QVector<AnchorCandidate> perCatalogAnchors;
        perCatalogAnchors.reserve(12);
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

            const double shapeScore = narrowGuidedAnchorShapeScore(detection);
            if (!detection.m_saturated
                && (shapeScore < 8.0)
                && (cachedDetectionBrightnessMetric(starDetections, detectionIndex) < 25.0)
                && (reliability < 25.0))
            {
                continue;
            }

            const double radialError = std::fabs(
                pointDistancePixels(detection.m_center, principalPoint) - projectedRadius);
            if (radialError > radialTolerancePixels) {
                continue;
            }

            const double detectionRank = ((detectionIndex >= 0) && (detectionIndex < detectionRanks.size()))
                ? detectionRanks[detectionIndex]
                : 0.5;
            const double score = radialError * 0.20
                + visibleStar.magnitude * 8.0
                + detectionRank * 70.0
                - std::min(90.0, std::log1p(reliability) * 8.0)
                - std::min(60.0, shapeScore * 0.8)
                - (namedCatalogStar ? 18.0 : 0.0);
            perCatalogAnchors.append({
                detectionIndex,
                visibleStar.catalogIndex,
                visibleStar,
                score,
                radialError,
                visibleStar.magnitude
            });
        }

        std::sort(perCatalogAnchors.begin(), perCatalogAnchors.end(), [](const AnchorCandidate& lhs, const AnchorCandidate& rhs) {
            if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
                return lhs.score < rhs.score;
            }
            return lhs.radialErrorPixels < rhs.radialErrorPixels;
        });
        const int perCatalogLimit = std::min(
            static_cast<int>(perCatalogAnchors.size()),
            visibleStar.magnitude <= 10.0 ? 12 : 8);
        for (int i = 0; i < perCatalogLimit; ++i) {
            anchors.append(perCatalogAnchors[i]);
        }
    }

    QSet<int> uniqueAnchorCatalogs;
    QSet<int> uniqueAnchorDetections;
    for (const AnchorCandidate& anchor : anchors)
    {
        uniqueAnchorCatalogs.insert(anchor.catalogIndex);
        uniqueAnchorDetections.insert(anchor.detectionIndex);
    }
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleUniqueCatalogs"), uniqueAnchorCatalogs.size());
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleUniqueDetections"), uniqueAnchorDetections.size());

    if ((anchors.size() < 3) || (uniqueAnchorCatalogs.size() < 3) || (uniqueAnchorDetections.size() < 3)) {
        recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleAnchors"), anchors.size());
        return seeds;
    }

    std::sort(anchors.begin(), anchors.end(), [](const AnchorCandidate& lhs, const AnchorCandidate& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score < rhs.score;
        }
        return lhs.magnitude < rhs.magnitude;
    });
    if (anchors.size() > 192) {
        anchors.resize(192);
    }

    const auto sortedRatios = [](std::array<double, 3> distances) {
        std::sort(distances.begin(), distances.end());
        if (distances[2] <= 1e-6) {
            return std::array<double, 3>{{0.0, 0.0, 0.0}};
        }
        return std::array<double, 3>{{
            distances[0] / distances[2],
            distances[1] / distances[2],
            distances[2]
        }};
    };

    struct TriangleCandidate
    {
        std::array<int, 3> anchorIndices{{-1, -1, -1}};
        double score = 0.0;
        double baseFov = 0.0;
    };
    QVector<TriangleCandidate> triangleCandidates;
    triangleCandidates.reserve(1024);
    qint64 ratioCandidates = 0;
    qint64 ratioMatches = 0;
    const double ratioTolerance = 0.085;
    const double startDirectionMaxDelta = std::max(
        static_cast<double>(settings.m_plateSolveSearchRadius) + static_cast<double>(settings.m_fov) * 2.0,
        static_cast<double>(settings.m_fov) * 4.0);

#if 1
    struct RollConsensusDetection
    {
        int detectionIndex = -1;
        double radiusPixels = 0.0;
        double angleDegrees = 0.0;
        double score = 0.0;
        double brightness = 0.0;
        double reliability = 0.0;
        double shapeScore = 0.0;
    };
    QVector<RollConsensusDetection> rollConsensusDetections;
    rollConsensusDetections.reserve(starDetections.size());
    for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
    {
        const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
        if (!isDetectionUsableForBrightPrior(detection)) {
            continue;
        }

        const double dx = detection.m_center.x() - principalPoint.x();
        const double dy = principalPoint.y() - detection.m_center.y();
        const double radiusPixels = std::hypot(dx, dy);
        if (radiusPixels < 18.0) {
            continue;
        }

        const double brightness = cachedDetectionBrightnessMetric(starDetections, detectionIndex);
        const double reliability = cachedDetectionReliabilityMetric(starDetections, detectionIndex);
        const double shapeScore = narrowGuidedAnchorShapeScore(detection);
        RollConsensusDetection rollDetection;
        rollDetection.detectionIndex = detectionIndex;
        rollDetection.radiusPixels = radiusPixels;
        rollDetection.angleDegrees = std::atan2(dy, dx) * 180.0 / kPi;
        rollDetection.brightness = brightness;
        rollDetection.reliability = reliability;
        rollDetection.shapeScore = shapeScore;
        rollDetection.score = brightGuidedDetectionPriorityScore(
            detection,
            brightness,
            reliability,
            shapeScore);
        rollConsensusDetections.append(rollDetection);
    }
    std::sort(rollConsensusDetections.begin(), rollConsensusDetections.end(), [](const RollConsensusDetection& lhs, const RollConsensusDetection& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score > rhs.score;
        }
        return lhs.brightness > rhs.brightness;
    });
    const int rollConsensusDetectionLimit = std::min(
        static_cast<int>(rollConsensusDetections.size()),
        96);
    if (rollConsensusDetections.size() > rollConsensusDetectionLimit) {
        rollConsensusDetections.resize(rollConsensusDetectionLimit);
    }

    struct RollConsensusPair
    {
        int catalogAnchorIndex = -1;
        int detectionIndex = -1;
        double rollDegrees = 0.0;
        double radialErrorPixels = 0.0;
        double score = 0.0;
        double magnitude = 0.0;
    };
    QVector<RollConsensusPair> rollConsensusPairs;
    rollConsensusPairs.reserve(1024);
    qint64 rollConsensusPairCandidates = 0;
    const int rollConsensusCatalogLimit = std::min(
        static_cast<int>(anchorCatalogStars.size()),
        72);
    const double rollConsensusRadialTolerancePixels = std::min(
        maxImageDimension * 0.20,
        std::max(
            static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 5.0,
            std::max(38.0, maxImageDimension * 0.030)));
    for (int catalogAnchorIndex = 0; catalogAnchorIndex < rollConsensusCatalogLimit; ++catalogAnchorIndex)
    {
        const ProjectedAnchorCatalogStar& catalogAnchor = anchorCatalogStars[catalogAnchorIndex];
        const double catalogDx = catalogAnchor.projectedPoint.x() - principalPoint.x();
        const double catalogDy = principalPoint.y() - catalogAnchor.projectedPoint.y();
        const double catalogRadiusPixels = std::hypot(catalogDx, catalogDy);
        if (catalogRadiusPixels < 18.0) {
            continue;
        }

        const double catalogAngleDegrees = std::atan2(catalogDy, catalogDx) * 180.0 / kPi;
        const bool namedCatalogStar = (catalogAnchor.visibleStar.catalogIndex >= 0)
            && (catalogAnchor.visibleStar.catalogIndex < catalogContext.catalogStars.size())
            && isNamedSparseGuidedCatalogStar(catalogContext.catalogStars[catalogAnchor.visibleStar.catalogIndex]);
        QVector<RollConsensusPair> perCatalogPairs;
        perCatalogPairs.reserve(12);
        for (int detectionRank = 0; detectionRank < rollConsensusDetections.size(); ++detectionRank)
        {
            const RollConsensusDetection& detection = rollConsensusDetections[detectionRank];
            const CameraPipelineStarDetection& starDetection = starDetections[detection.detectionIndex];
            if (isImplausiblyCompactBrightCatalogDetection(
                    starDetection,
                    catalogAnchor.visibleStar.magnitude,
                    true))
            {
                continue;
            }

            const double radialErrorPixels = std::fabs(detection.radiusPixels - catalogRadiusPixels);
            if (radialErrorPixels > rollConsensusRadialTolerancePixels) {
                continue;
            }
            ++rollConsensusPairCandidates;

            const double detectionRankFraction = rollConsensusDetections.size() > 1
                ? static_cast<double>(detectionRank) / static_cast<double>(rollConsensusDetections.size() - 1)
                : 0.0;
            const double catalogRankFraction = rollConsensusCatalogLimit > 1
                ? static_cast<double>(catalogAnchorIndex) / static_cast<double>(rollConsensusCatalogLimit - 1)
                : 0.0;
            const double rankError = std::fabs(detectionRankFraction - catalogRankFraction);
            const double reliabilityBonus = std::min(
                65.0,
                std::log1p(std::max(0.0, detection.reliability)) * 7.0);
            const double shapeBonus = std::min(
                50.0,
                std::max(0.0, detection.shapeScore) * 0.65);
            const double brightnessBonus = std::min(
                30.0,
                std::log1p(std::max(0.0, detection.brightness)) * 2.2);
            RollConsensusPair pair;
            pair.catalogAnchorIndex = catalogAnchorIndex;
            pair.detectionIndex = detection.detectionIndex;
            pair.rollDegrees = normalizeDegrees(detection.angleDegrees - catalogAngleDegrees + 180.0) - 180.0;
            pair.radialErrorPixels = radialErrorPixels;
            pair.magnitude = catalogAnchor.visibleStar.magnitude;
            pair.score = radialErrorPixels * 0.45
                + catalogAnchor.visibleStar.magnitude * 8.0
                + rankError * 60.0
                - reliabilityBonus
                - shapeBonus
                - brightnessBonus
                - (namedCatalogStar ? 14.0 : 0.0);
            perCatalogPairs.append(pair);
        }

        std::sort(perCatalogPairs.begin(), perCatalogPairs.end(), [](const RollConsensusPair& lhs, const RollConsensusPair& rhs) {
            if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
                return lhs.score < rhs.score;
            }
            return lhs.radialErrorPixels < rhs.radialErrorPixels;
        });
        const int perCatalogPairLimit = catalogAnchor.visibleStar.magnitude <= 12.0 ? 8 : 5;
        if (perCatalogPairs.size() > perCatalogPairLimit) {
            perCatalogPairs.resize(perCatalogPairLimit);
        }
        rollConsensusPairs += perCatalogPairs;
    }
    std::sort(rollConsensusPairs.begin(), rollConsensusPairs.end(), [](const RollConsensusPair& lhs, const RollConsensusPair& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score < rhs.score;
        }
        return lhs.magnitude < rhs.magnitude;
    });
    if (rollConsensusPairs.size() > 640) {
        rollConsensusPairs.resize(640);
    }

    qint64 rollConsensusDirectEvaluations = 0;
    qint64 rollConsensusDirectSeeds = 0;
    {
        QSet<int> triedRollBins;
        const int directRollEvaluationLimit = 160;
        const int directRollSeedLimit = 48;
        for (const RollConsensusPair& pair : rollConsensusPairs)
        {
            if (isCancellationRequested()
                || (rollConsensusDirectEvaluations >= directRollEvaluationLimit)
                || (rollConsensusDirectSeeds >= directRollSeedLimit))
            {
                break;
            }

            const int rollBin = static_cast<int>(std::floor((pair.rollDegrees + 180.0) * 2.0));
            if (triedRollBins.contains(rollBin)) {
                continue;
            }
            triedRollBins.insert(rollBin);

            Evaluation candidate = evaluatePose(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                settings.m_azimuth,
                settings.m_elevation,
                pair.rollDegrees,
                settings.m_fov,
                nullptr,
                fixedCenterOffsetX,
                fixedCenterOffsetY,
                fixedDistortionK1);
            ++rollConsensusDirectEvaluations;
            candidate.anchored = true;
            candidate.anchorDetectionIndex = pair.detectionIndex;
            candidate.anchorCatalogIndex = anchorCatalogStars[pair.catalogAnchorIndex].visibleStar.catalogIndex;
            const bool strongDirectAnchorPair =
                (pair.magnitude <= 12.0)
                && (pair.radialErrorPixels <= std::max(
                    static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 2.5,
                    rollConsensusRadialTolerancePixels * 0.50));
            const double directRmsCap = strongDirectAnchorPair
                ? std::min(
                    static_cast<double>(settings.m_plateSolveFinalMatchRadius),
                    24.0)
                : std::min(
                    static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.80,
                    20.0);
            if (!candidate.valid
                || (candidate.matchCount < std::max(4, settings.m_plateSolveMinMatches))
                || (candidate.rmsErrorPixels > directRmsCap)
                || (!strongDirectAnchorPair && !hasAcceptableBrightnessConsistency(candidate)))
            {
                continue;
            }

            seeds.append(candidate);
            ++rollConsensusDirectSeeds;
        }

        if (!plateSolveStartUsesRoll(settings) && isNarrowGuidedDirectionSolve(settings))
        {
            static const std::array<double, 17> localRollOffsets {{
                -20.0, -15.0, -12.5, -10.0, -7.5, -5.0, -2.5, 0.0, 2.5,
                5.0, 7.5, 10.0, 12.5, 15.0, 20.0, -30.0, 30.0
            }};
            QVector<int> localRollDetectionIndices;
            localRollDetectionIndices.reserve(starDetections.size());
            for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex) {
                localRollDetectionIndices.append(detectionIndex);
            }
            QSet<int> triedLocalRollBins;
            for (double rollOffset : localRollOffsets)
            {
                if (isCancellationRequested()
                    || (rollConsensusDirectEvaluations >= directRollEvaluationLimit)
                    || (rollConsensusDirectSeeds >= directRollSeedLimit))
                {
                    break;
                }

                const double rollDegrees = normalizeDegrees(settings.m_roll + rollOffset + 180.0) - 180.0;
                const int rollBin = static_cast<int>(std::floor((rollDegrees + 180.0) * 2.0));
                if (triedLocalRollBins.contains(rollBin)) {
                    continue;
                }
                triedLocalRollBins.insert(rollBin);

                Evaluation candidate = evaluatePose(
                    settings,
                    catalogContext,
                    imageSize,
                    captureDateTimeUtc,
                    starDetections,
                    localRollDetectionIndices,
                    settings.m_azimuth,
                    settings.m_elevation,
                    rollDegrees,
                    settings.m_fov,
                    nullptr,
                    fixedCenterOffsetX,
                    fixedCenterOffsetY,
                    fixedDistortionK1);
                ++rollConsensusDirectEvaluations;
                if (!candidate.valid
                    || (candidate.matchCount < std::max(4, settings.m_plateSolveMinMatches))
                    || (candidate.rmsErrorPixels > std::min(
                        static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.90,
                        22.0)))
                {
                    continue;
                }

                seeds.append(candidate);
                ++rollConsensusDirectSeeds;
            }
        }
    }

    struct RollConsensusTriangleCandidate
    {
        std::array<int, 3> pairIndices{{-1, -1, -1}};
        double score = 0.0;
    };
    QVector<RollConsensusTriangleCandidate> rollConsensusTriangleCandidates;
    rollConsensusTriangleCandidates.reserve(768);
    qint64 rollConsensusClusters = 0;
    qint64 rollConsensusSupportPairs = 0;
    qint64 rollConsensusTriangleCandidateCount = 0;
    const double rollConsensusToleranceDegrees = std::max(4.0, std::min(12.0, settings.m_fov * 5.0));
    const int rollConsensusClusterLimit = 96;
    for (int seedPairIndex = 0;
         (seedPairIndex < rollConsensusPairs.size()) && (rollConsensusClusters < rollConsensusClusterLimit);
         ++seedPairIndex)
    {
        if (isCancellationRequested()) {
            return seeds;
        }

        QVector<int> supportPairIndices;
        supportPairIndices.reserve(16);
        QSet<int> usedCatalogAnchors;
        QSet<int> usedDetections;
        double weightedSin = 0.0;
        double weightedCos = 0.0;
        for (int pairIndex = seedPairIndex; pairIndex < rollConsensusPairs.size(); ++pairIndex)
        {
            const RollConsensusPair& pair = rollConsensusPairs[pairIndex];
            const double rollDelta = std::fabs(normalizeDegrees(pair.rollDegrees - rollConsensusPairs[seedPairIndex].rollDegrees + 180.0) - 180.0);
            if (rollDelta > rollConsensusToleranceDegrees) {
                continue;
            }
            if (usedCatalogAnchors.contains(pair.catalogAnchorIndex)
                || usedDetections.contains(pair.detectionIndex))
            {
                continue;
            }

            supportPairIndices.append(pairIndex);
            usedCatalogAnchors.insert(pair.catalogAnchorIndex);
            usedDetections.insert(pair.detectionIndex);
            const double weight = 1.0 / (1.0 + std::max(0.0, pair.score));
            weightedSin += std::sin(degToRad(pair.rollDegrees)) * weight;
            weightedCos += std::cos(degToRad(pair.rollDegrees)) * weight;
            if (supportPairIndices.size() >= 10) {
                break;
            }
        }

        if (supportPairIndices.size() < 3) {
            continue;
        }
        ++rollConsensusClusters;
        rollConsensusSupportPairs += supportPairIndices.size();

        const double clusterRollDegrees = std::atan2(weightedSin, weightedCos) * 180.0 / kPi;
        for (int a = 0; a < supportPairIndices.size(); ++a)
        {
            for (int b = a + 1; b < supportPairIndices.size(); ++b)
            {
                for (int c = b + 1; c < supportPairIndices.size(); ++c)
                {
                    const RollConsensusPair& pairA = rollConsensusPairs[supportPairIndices[a]];
                    const RollConsensusPair& pairB = rollConsensusPairs[supportPairIndices[b]];
                    const RollConsensusPair& pairC = rollConsensusPairs[supportPairIndices[c]];
                    const double rollSpread =
                        std::fabs(normalizeDegrees(pairA.rollDegrees - clusterRollDegrees + 180.0) - 180.0)
                        + std::fabs(normalizeDegrees(pairB.rollDegrees - clusterRollDegrees + 180.0) - 180.0)
                        + std::fabs(normalizeDegrees(pairC.rollDegrees - clusterRollDegrees + 180.0) - 180.0);
                    ++rollConsensusTriangleCandidateCount;
                    RollConsensusTriangleCandidate candidate;
                    candidate.pairIndices = {{supportPairIndices[a], supportPairIndices[b], supportPairIndices[c]}};
                    candidate.score = pairA.score + pairB.score + pairC.score + rollSpread * 10.0;
                    rollConsensusTriangleCandidates.append(candidate);
                }
            }
        }
    }

    std::sort(rollConsensusTriangleCandidates.begin(), rollConsensusTriangleCandidates.end(), [](const RollConsensusTriangleCandidate& lhs, const RollConsensusTriangleCandidate& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score < rhs.score;
        }
        return lhs.pairIndices[0] < rhs.pairIndices[0];
    });
    if (rollConsensusTriangleCandidates.size() > 768) {
        rollConsensusTriangleCandidates.resize(768);
    }

    qint64 rollConsensusSeedEvaluations = 0;
    qint64 rollConsensusSeedValid = 0;
    qint64 rollConsensusAnchorSupported = 0;
    qint64 rollConsensusSeeds = 0;
    qint64 rollConsensusVerifiedSeeds = 0;
    int rollConsensusMaxAnchors = 0;
    const int rollConsensusEvaluationLimit = 1200;
    const int rollConsensusSeedLimit = 96;
    const std::array<double, 3> rollConsensusFovScales = plateSolveStartUsesFov(settings)
        ? std::array<double, 3>{{1.0, 1.0, 1.0}}
        : std::array<double, 3>{{0.98, 1.0, 1.02}};
    for (const RollConsensusTriangleCandidate& rollTriangleCandidate : rollConsensusTriangleCandidates)
    {
        if (isCancellationRequested()
            || (rollConsensusSeedEvaluations >= rollConsensusEvaluationLimit)
            || (seeds.size() >= rollConsensusSeedLimit))
        {
            break;
        }

        const RollConsensusPair& pairA = rollConsensusPairs[rollTriangleCandidate.pairIndices[0]];
        const RollConsensusPair& pairB = rollConsensusPairs[rollTriangleCandidate.pairIndices[1]];
        const RollConsensusPair& pairC = rollConsensusPairs[rollTriangleCandidate.pairIndices[2]];
        const std::array<int, 3> anchorDetectionIndices {{
            pairA.detectionIndex,
            pairB.detectionIndex,
            pairC.detectionIndex
        }};
        const std::array<int, 3> anchorCatalogIndices {{
            anchorCatalogStars[pairA.catalogAnchorIndex].visibleStar.catalogIndex,
            anchorCatalogStars[pairB.catalogAnchorIndex].visibleStar.catalogIndex,
            anchorCatalogStars[pairC.catalogAnchorIndex].visibleStar.catalogIndex
        }};
        QVector<int> allowedCatalogIndices {
            anchorCatalogIndices[0],
            anchorCatalogIndices[1],
            anchorCatalogIndices[2]
        };

        double previousSeedFov = -1.0;
        for (double fovScale : rollConsensusFovScales)
        {
            if (isCancellationRequested()
                || (rollConsensusSeedEvaluations >= rollConsensusEvaluationLimit)
                || (seeds.size() >= rollConsensusSeedLimit))
            {
                break;
            }

            const double seedFov = std::clamp(
                static_cast<double>(settings.m_fov) * fovScale,
                static_cast<double>(CameraSettings::m_minFov),
                static_cast<double>(CameraSettings::m_maxFov));
            if (std::fabs(seedFov - previousSeedFov) < 1e-9) {
                continue;
            }
            previousSeedFov = seedFov;
            if (!seedFovCompatibleWithStartFov(settings, seedFov)) {
                continue;
            }

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

            std::array<SkyVector, 3> sourceVectors;
            bool haveSourceVectors = true;
            for (int idx = 0; idx < 3; ++idx)
            {
                if (!unprojectPixelToVector(
                        baseProjector,
                        starDetections[anchorDetectionIndices[idx]].m_center,
                        sourceVectors[idx]))
                {
                    haveSourceVectors = false;
                    break;
                }
            }
            if (!haveSourceVectors) {
                continue;
            }

            const std::array<SkyVector, 3> targetVectors {{
                anchorCatalogStars[pairA.catalogAnchorIndex].visibleStar.vector,
                anchorCatalogStars[pairB.catalogAnchorIndex].visibleStar.vector,
                anchorCatalogStars[pairC.catalogAnchorIndex].visibleStar.vector
            }};
            double seedAzimuth = 0.0;
            double seedElevation = 0.0;
            double seedRoll = 0.0;
            double triangleRmsAngularError = std::numeric_limits<double>::infinity();
            double triangleMaxAngularError = std::numeric_limits<double>::infinity();
            if (!poseFromThreeVectorPairs(
                    baseProjector,
                    sourceVectors,
                    targetVectors,
                    seedAzimuth,
                    seedElevation,
                    seedRoll,
                    &triangleRmsAngularError,
                    &triangleMaxAngularError))
            {
                continue;
            }
            Q_UNUSED(triangleRmsAngularError)
            Q_UNUSED(triangleMaxAngularError)

            const double directionDelta = std::acos(std::clamp(
                dot(vectorFromAltAz(seedAzimuth, seedElevation), vectorFromAltAz(settings.m_azimuth, settings.m_elevation)),
                -1.0,
                1.0)) * 180.0 / kPi;
            if (directionDelta > startDirectionMaxDelta) {
                continue;
            }

            Evaluation seededCandidate = evaluatePose(
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
            ++rollConsensusSeedEvaluations;
            if (!seededCandidate.valid) {
                continue;
            }
            ++rollConsensusSeedValid;

            const double anchorDistancePixels = std::min(
                96.0,
                std::max(
                    static_cast<double>(settings.m_plateSolveMatchRadius),
                    static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 3.0));
            const int anchorSupport = countProjectedAnchorSupport(
                settings,
                catalogContext,
                imageSize,
                starDetections,
                seededCandidate,
                anchorDetectionIndices,
                anchorCatalogIndices,
                anchorDistancePixels);
            rollConsensusMaxAnchors = std::max(rollConsensusMaxAnchors, anchorSupport);
            if (anchorSupport < 3) {
                continue;
            }
            ++rollConsensusAnchorSupported;
            seededCandidate.anchored = true;
            seededCandidate.guidedTriangle = true;
            seededCandidate.anchorDetectionIndex = anchorDetectionIndices[0];
            seededCandidate.anchorCatalogIndex = anchorCatalogIndices[0];
            seededCandidate.secondaryAnchorDetectionIndex = anchorDetectionIndices[1];
            seededCandidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
            seededCandidate.tertiaryAnchorDetectionIndex = anchorDetectionIndices[2];
            seededCandidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];
            seededCandidate.matches = {
                { anchorDetectionIndices[0], anchorCatalogIndices[0], 0.0 },
                { anchorDetectionIndices[1], anchorCatalogIndices[1], 0.0 },
                { anchorDetectionIndices[2], anchorCatalogIndices[2], 0.0 }
            };
            seededCandidate.matchCount = seededCandidate.matches.size();
            seededCandidate.rmsErrorPixels = std::min(
                seededCandidate.rmsErrorPixels,
                static_cast<double>(settings.m_plateSolveFinalMatchRadius));
            seeds.append(seededCandidate);
            ++rollConsensusSeeds;

            Evaluation candidate = evaluatePose(
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
            candidate.anchored = true;
            candidate.guidedTriangle = true;
            candidate.anchorDetectionIndex = anchorDetectionIndices[0];
            candidate.anchorCatalogIndex = anchorCatalogIndices[0];
            candidate.secondaryAnchorDetectionIndex = anchorDetectionIndices[1];
            candidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
            candidate.tertiaryAnchorDetectionIndex = anchorDetectionIndices[2];
            candidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];

            const double guidedTriangleSeedRmsCap = std::min(
                std::max(18.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.75),
                static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.90);
            if (candidate.valid
                && (candidate.matchCount >= std::max(4, settings.m_plateSolveMinMatches))
                && (candidate.rmsErrorPixels <= guidedTriangleSeedRmsCap)
                && hasAcceptableBrightnessConsistency(candidate))
            {
                seeds.append(candidate);
                ++rollConsensusSeeds;
            }

            Evaluation verifiedCandidate = verifyBlindSeedCandidate(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                candidate);
            if (verifiedCandidate.valid)
            {
                verifiedCandidate.anchored = true;
                verifiedCandidate.guidedTriangle = true;
                verifiedCandidate.anchorDetectionIndex = anchorDetectionIndices[0];
                verifiedCandidate.anchorCatalogIndex = anchorCatalogIndices[0];
                verifiedCandidate.secondaryAnchorDetectionIndex = anchorDetectionIndices[1];
                verifiedCandidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
                verifiedCandidate.tertiaryAnchorDetectionIndex = anchorDetectionIndices[2];
                verifiedCandidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];
                seeds.append(verifiedCandidate);
                ++rollConsensusSeeds;
                ++rollConsensusVerifiedSeeds;
            }
        }
    }
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusDetections"), rollConsensusDetections.size());
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusPairs"), rollConsensusPairs.size());
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusPairCandidates"), rollConsensusPairCandidates);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusDirectEvaluations"), rollConsensusDirectEvaluations);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusDirectSeeds"), rollConsensusDirectSeeds);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusClusters"), rollConsensusClusters);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusSupportPairs"), rollConsensusSupportPairs);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusTriangleCandidates"), rollConsensusTriangleCandidateCount);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusKept"), rollConsensusTriangleCandidates.size());
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusEvaluations"), rollConsensusSeedEvaluations);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusSeedValid"), rollConsensusSeedValid);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusAnchorSupported"), rollConsensusAnchorSupported);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusMaxAnchors"), rollConsensusMaxAnchors);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusSeeds"), rollConsensusSeeds);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusVerifiedSeeds"), rollConsensusVerifiedSeeds);
#else
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusDetections"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusPairs"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusPairCandidates"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusDirectEvaluations"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusDirectSeeds"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusClusters"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusSupportPairs"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusTriangleCandidates"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusKept"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusEvaluations"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusSeedValid"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusAnchorSupported"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusMaxAnchors"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusSeeds"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusVerifiedSeeds"), 0);
#endif

    struct ProjectedAnchorCandidate
    {
        int catalogAnchorIndex = -1;
        int detectionIndex = -1;
        double score = 0.0;
        double distancePixels = 0.0;
    };
    const bool expandedProjectedAnchorSearch = settings.m_plateSolveMaxMagnitude <= 14.0;
    const int projectedCatalogLimit = std::min(
        expandedProjectedAnchorSearch ? 48 : 32,
        static_cast<int>(anchorCatalogStars.size()));
    const double projectedAnchorTolerancePixels = std::min(
        maxImageDimension * 0.32,
        std::max(
            static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 10.0,
            std::max(180.0, maxImageDimension * 0.16)));
    QVector<QVector<ProjectedAnchorCandidate>> projectedAnchorCandidates(projectedCatalogLimit);
    qint64 projectedAnchorPairs = 0;
    for (int catalogAnchorIndex = 0; catalogAnchorIndex < projectedCatalogLimit; ++catalogAnchorIndex)
    {
        const ProjectedAnchorCatalogStar& catalogAnchor = anchorCatalogStars[catalogAnchorIndex];
        QVector<ProjectedAnchorCandidate>& perCatalogAnchors = projectedAnchorCandidates[catalogAnchorIndex];
        perCatalogAnchors.reserve(12);
        const bool namedCatalogStar = (catalogAnchor.visibleStar.catalogIndex >= 0)
            && (catalogAnchor.visibleStar.catalogIndex < catalogContext.catalogStars.size())
            && isNamedSparseGuidedCatalogStar(catalogContext.catalogStars[catalogAnchor.visibleStar.catalogIndex]);
        for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
        {
            const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
            if (!isDetectionUsableForBrightPrior(detection)) {
                continue;
            }
            if (isImplausiblyCompactBrightCatalogDetection(
                    detection,
                    catalogAnchor.visibleStar.magnitude,
                    true))
            {
                continue;
            }

            const double distancePixels = pointDistancePixels(detection.m_center, catalogAnchor.projectedPoint);
            if (distancePixels > projectedAnchorTolerancePixels) {
                continue;
            }

            const double detectionRank = ((detectionIndex >= 0) && (detectionIndex < detectionRanks.size()))
                ? detectionRanks[detectionIndex]
                : 0.5;
            const double reliability = cachedDetectionReliabilityMetric(starDetections, detectionIndex);
            const double shapeScore = narrowGuidedAnchorShapeScore(detection);
            const double brightness = cachedDetectionBrightnessMetric(starDetections, detectionIndex);
            const double score = distancePixels * (expandedProjectedAnchorSearch ? 0.40 : 0.45)
                + catalogAnchor.visibleStar.magnitude * (expandedProjectedAnchorSearch ? 6.0 : 9.0)
                + detectionRank * 55.0
                - std::min(70.0, std::log1p(std::max(0.0, reliability)) * 8.0)
                - std::min(55.0, shapeScore * 0.9)
                - std::min(30.0, std::log1p(std::max(0.0, brightness)) * 2.0)
                - (namedCatalogStar ? (expandedProjectedAnchorSearch ? 30.0 : 12.0) : 0.0);
            perCatalogAnchors.append({
                catalogAnchorIndex,
                detectionIndex,
                score,
                distancePixels
            });
        }

        std::sort(perCatalogAnchors.begin(), perCatalogAnchors.end(), [](const ProjectedAnchorCandidate& lhs, const ProjectedAnchorCandidate& rhs) {
            if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
                return lhs.score < rhs.score;
            }
            return lhs.distancePixels < rhs.distancePixels;
        });
        const int perCatalogLimit = catalogAnchor.visibleStar.magnitude <= 12.0
            ? (expandedProjectedAnchorSearch ? 12 : 8)
            : (expandedProjectedAnchorSearch ? 6 : 5);
        if (perCatalogAnchors.size() > perCatalogLimit) {
            perCatalogAnchors.resize(perCatalogLimit);
        }
        projectedAnchorPairs += perCatalogAnchors.size();
    }

    struct ProjectedTriangleCandidate
    {
        std::array<int, 3> catalogAnchorIndices{{-1, -1, -1}};
        std::array<int, 3> detectionIndices{{-1, -1, -1}};
        double score = 0.0;
        double seedFov = 0.0;
        double similarityRmsPixels = 0.0;
        double brightnessMagnitudeError = 0.0;
    };
    QVector<ProjectedTriangleCandidate> projectedTriangleCandidates;
    const int projectedTriangleKeepLimit = expandedProjectedAnchorSearch ? 1024 : 768;
    projectedTriangleCandidates.reserve(projectedTriangleKeepLimit);
    qint64 projectedTriangleCandidateCount = 0;
    qint64 projectedTriangleRatioMatches = 0;
    const double projectedTriangleRatioTolerance = 0.10;
    const double projectedTriangleSimilarityTolerancePixels = std::max(
        static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 4.0,
        std::min(160.0, projectedAnchorTolerancePixels * 0.55));

    for (int i = 0; i < projectedCatalogLimit; ++i)
    {
        if (projectedAnchorCandidates[i].isEmpty()) {
            continue;
        }
        for (int j = i + 1; j < projectedCatalogLimit; ++j)
        {
            if (projectedAnchorCandidates[j].isEmpty()) {
                continue;
            }
            for (int k = j + 1; k < projectedCatalogLimit; ++k)
            {
                if (isCancellationRequested()) {
                    return seeds;
                }
                if (projectedAnchorCandidates[k].isEmpty()) {
                    continue;
                }

                const std::array<QPointF, 3> catalogProjectedPoints {{
                    anchorCatalogStars[i].projectedPoint,
                    anchorCatalogStars[j].projectedPoint,
                    anchorCatalogStars[k].projectedPoint
                }};
                const TriangleSignature catalogSignature = buildTriangleSignature(catalogProjectedPoints);
                if (catalogSignature.longestDistance < 20.0) {
                    continue;
                }
                const std::array<VisibleCatalogStar, 3> projectedStars {{
                    anchorCatalogStars[i].visibleStar,
                    anchorCatalogStars[j].visibleStar,
                    anchorCatalogStars[k].visibleStar
                }};

                for (const ProjectedAnchorCandidate& anchorA : projectedAnchorCandidates[i])
                {
                    for (const ProjectedAnchorCandidate& anchorB : projectedAnchorCandidates[j])
                    {
                        if (anchorA.detectionIndex == anchorB.detectionIndex) {
                            continue;
                        }
                        for (const ProjectedAnchorCandidate& anchorC : projectedAnchorCandidates[k])
                        {
                            if ((anchorA.detectionIndex == anchorC.detectionIndex)
                                || (anchorB.detectionIndex == anchorC.detectionIndex))
                            {
                                continue;
                            }
                            ++projectedTriangleCandidateCount;

                            const std::array<int, 3> anchorDetectionIndices {{
                                anchorA.detectionIndex,
                                anchorB.detectionIndex,
                                anchorC.detectionIndex
                            }};
                            if (!triangleBrightnessOrderCompatible(
                                    starDetections,
                                    anchorDetectionIndices,
                                    projectedStars,
                                    std::array<int, 3>{{0, 1, 2}}))
                            {
                                continue;
                            }
                            const double brightnessMagnitudeError = triangleBrightnessMagnitudeError(
                                starDetections,
                                anchorDetectionIndices,
                                projectedStars,
                                std::array<int, 3>{{0, 1, 2}});
                            if (!std::isfinite(brightnessMagnitudeError)
                                || (brightnessMagnitudeError > 4.0))
                            {
                                continue;
                            }

                            const std::array<QPointF, 3> detectionPoints {{
                                starDetections[anchorA.detectionIndex].m_center,
                                starDetections[anchorB.detectionIndex].m_center,
                                starDetections[anchorC.detectionIndex].m_center
                            }};
                            const TriangleSignature detectionSignature = buildTriangleSignature(detectionPoints);
                            if (detectionSignature.longestDistance < 20.0) {
                                continue;
                            }

                            const double ratioError =
                                std::fabs(detectionSignature.ratioShortToLong - catalogSignature.ratioShortToLong)
                                + std::fabs(detectionSignature.ratioMidToLong - catalogSignature.ratioMidToLong);
                            if (ratioError > projectedTriangleRatioTolerance) {
                                continue;
                            }
                            ++projectedTriangleRatioMatches;

                            double similarityScale = 1.0;
                            double similarityRotationDegrees = 0.0;
                            double similarityRmsPixels = std::numeric_limits<double>::infinity();
                            if (!estimateScreenSimilarity(
                                    catalogProjectedPoints,
                                    detectionPoints,
                                    similarityScale,
                                    similarityRotationDegrees,
                                    similarityRmsPixels)
                                || (similarityRmsPixels > projectedTriangleSimilarityTolerancePixels))
                            {
                                continue;
                            }
                            Q_UNUSED(similarityRotationDegrees)

                            const double seedFov = plateSolveStartUsesFov(settings)
                                ? std::clamp(
                                    static_cast<double>(settings.m_fov),
                                    static_cast<double>(CameraSettings::m_minFov),
                                    static_cast<double>(CameraSettings::m_maxFov))
                                : std::clamp(
                                    static_cast<double>(settings.m_fov) / std::max(0.50, std::min(2.0, similarityScale)),
                                    static_cast<double>(CameraSettings::m_minFov),
                                    static_cast<double>(CameraSettings::m_maxFov));
                            if (!seedFovCompatibleWithStartFov(settings, seedFov)) {
                                continue;
                            }

                            const double distanceScore =
                                anchorA.distancePixels + anchorB.distancePixels + anchorC.distancePixels;
                            const double anchorScore = anchorA.score + anchorB.score + anchorC.score;
                            const double magnitudeSum =
                                projectedStars[0].magnitude + projectedStars[1].magnitude + projectedStars[2].magnitude;
                            ProjectedTriangleCandidate candidate;
                            candidate.catalogAnchorIndices = {{i, j, k}};
                            candidate.detectionIndices = anchorDetectionIndices;
                            candidate.seedFov = seedFov;
                            candidate.similarityRmsPixels = similarityRmsPixels;
                            candidate.brightnessMagnitudeError = brightnessMagnitudeError;
                            candidate.score = anchorScore
                                + similarityRmsPixels * 4.0
                                + ratioError * 180.0
                                + distanceScore * 0.10
                                + magnitudeSum * 2.5
                                + brightnessMagnitudeError * 45.0;
                            projectedTriangleCandidates.append(candidate);
                        }
                    }
                }
            }
        }
    }

    std::sort(projectedTriangleCandidates.begin(), projectedTriangleCandidates.end(), [](const ProjectedTriangleCandidate& lhs, const ProjectedTriangleCandidate& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score < rhs.score;
        }
        if (!qFuzzyCompare(lhs.brightnessMagnitudeError + 1.0, rhs.brightnessMagnitudeError + 1.0)) {
            return lhs.brightnessMagnitudeError < rhs.brightnessMagnitudeError;
        }
        return lhs.similarityRmsPixels < rhs.similarityRmsPixels;
    });
    if (projectedTriangleCandidates.size() > projectedTriangleKeepLimit) {
        projectedTriangleCandidates.resize(projectedTriangleKeepLimit);
    }

    qint64 projectedTriangleEvaluations = 0;
    qint64 projectedTriangleSeedValid = 0;
    qint64 projectedTriangleAnchorSupported = 0;
    qint64 projectedTriangleSeeds = 0;
    qint64 projectedTriangleVerifiedSeeds = 0;
    int projectedTriangleMaxAnchors = 0;
    const int projectedTriangleEvaluationLimit = expandedProjectedAnchorSearch ? 1600 : 1200;
    for (const ProjectedTriangleCandidate& projectedTriangleCandidate : projectedTriangleCandidates)
    {
        if (isCancellationRequested()
            || (projectedTriangleEvaluations >= projectedTriangleEvaluationLimit)
            || (seeds.size() >= 96))
        {
            break;
        }

        const std::array<VisibleCatalogStar, 3> projectedStars {{
            anchorCatalogStars[projectedTriangleCandidate.catalogAnchorIndices[0]].visibleStar,
            anchorCatalogStars[projectedTriangleCandidate.catalogAnchorIndices[1]].visibleStar,
            anchorCatalogStars[projectedTriangleCandidate.catalogAnchorIndices[2]].visibleStar
        }};
        const std::array<QPointF, 3> detectionPoints {{
            starDetections[projectedTriangleCandidate.detectionIndices[0]].m_center,
            starDetections[projectedTriangleCandidate.detectionIndices[1]].m_center,
            starDetections[projectedTriangleCandidate.detectionIndices[2]].m_center
        }};
        QVector<GuidedTrianglePoseSeed> poseSeeds = guidedTriangleSimilarityPoseSeeds(
            settings,
            imageSize,
            detectionPoints,
            projectedStars,
            std::array<int, 3>{{0, 1, 2}},
            settings.m_azimuth,
            settings.m_elevation,
            projectedTriangleCandidate.seedFov,
            fixedCenterOffsetX,
            fixedCenterOffsetY,
            fixedDistortionK1);

        const SkyProjector sourceProjector = createProjector(
            settings,
            imageSize,
            0.0,
            90.0,
            0.0,
            projectedTriangleCandidate.seedFov,
            fixedCenterOffsetX,
            fixedCenterOffsetY,
            fixedDistortionK1);
        if (sourceProjector.valid)
        {
            std::array<SkyVector, 3> sourceVectors;
            bool haveSourceVectors = true;
            for (int idx = 0; idx < 3; ++idx)
            {
                if (!unprojectPixelToVector(
                        sourceProjector,
                        detectionPoints[idx],
                        sourceVectors[idx]))
                {
                    haveSourceVectors = false;
                    break;
                }
            }
            if (haveSourceVectors)
            {
                const std::array<SkyVector, 3> targetVectors {{
                    projectedStars[0].vector,
                    projectedStars[1].vector,
                    projectedStars[2].vector
                }};
                GuidedTrianglePoseSeed wahbaSeed;
                wahbaSeed.fovDegrees = projectedTriangleCandidate.seedFov;
                double triangleRmsAngularError = std::numeric_limits<double>::infinity();
                double triangleMaxAngularError = std::numeric_limits<double>::infinity();
                if (poseFromThreeVectorPairs(
                        sourceProjector,
                        sourceVectors,
                        targetVectors,
                        wahbaSeed.azimuthDegrees,
                        wahbaSeed.elevationDegrees,
                        wahbaSeed.rollDegrees,
                        &triangleRmsAngularError,
                        &triangleMaxAngularError))
                {
                    Q_UNUSED(triangleRmsAngularError)
                    Q_UNUSED(triangleMaxAngularError)
                    poseSeeds.append(wahbaSeed);
                }
            }
        }

        const std::array<int, 3> anchorCatalogIndices {{
            projectedStars[0].catalogIndex,
            projectedStars[1].catalogIndex,
            projectedStars[2].catalogIndex
        }};
        const bool debugTargetTriangle =
            debugTriangleContainsCatalogStar(catalogContext, anchorCatalogIndices);
        QVector<int> allowedCatalogIndices {
            anchorCatalogIndices[0],
            anchorCatalogIndices[1],
            anchorCatalogIndices[2]
        };
        double previousAzimuth = std::numeric_limits<double>::quiet_NaN();
        double previousElevation = std::numeric_limits<double>::quiet_NaN();
        double previousRoll = std::numeric_limits<double>::quiet_NaN();
        for (const GuidedTrianglePoseSeed& poseSeed : poseSeeds)
        {
            if (isCancellationRequested()
                || (projectedTriangleEvaluations >= projectedTriangleEvaluationLimit)
                || (seeds.size() >= 96))
            {
                break;
            }
            if (std::isfinite(previousAzimuth)
                && (angularDistanceDegrees(previousAzimuth, poseSeed.azimuthDegrees) < 0.02)
                && (std::fabs(previousElevation - poseSeed.elevationDegrees) < 0.02)
                && (angularDistanceDegrees(previousRoll, poseSeed.rollDegrees) < 0.5))
            {
                continue;
            }
            previousAzimuth = poseSeed.azimuthDegrees;
            previousElevation = poseSeed.elevationDegrees;
            previousRoll = poseSeed.rollDegrees;

            const double directionDelta = std::acos(std::clamp(
                dot(vectorFromAltAz(poseSeed.azimuthDegrees, poseSeed.elevationDegrees), vectorFromAltAz(settings.m_azimuth, settings.m_elevation)),
                -1.0,
                1.0)) * 180.0 / kPi;
            if (directionDelta > startDirectionMaxDelta) {
                continue;
            }

            Evaluation seededCandidate = evaluatePose(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                poseSeed.azimuthDegrees,
                poseSeed.elevationDegrees,
                poseSeed.rollDegrees,
                poseSeed.fovDegrees,
                &allowedCatalogIndices,
                fixedCenterOffsetX,
                fixedCenterOffsetY,
                fixedDistortionK1);
            ++projectedTriangleEvaluations;
            if (!seededCandidate.valid) {
                if (debugTargetTriangle)
                {
                    qDebug().noquote() << "CameraPlateSolver: projected triangle target seed invalid"
                        << debugTriangleAnchorSummary(
                            catalogContext,
                            starDetections,
                            projectedTriangleCandidate.detectionIndices,
                            anchorCatalogIndices);
                }
                continue;
            }
            ++projectedTriangleSeedValid;

            const double anchorDistancePixels = std::min(
                static_cast<double>(settings.m_plateSolveMatchRadius),
                std::max(6.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius)));
            const int anchorSupport = countProjectedAnchorSupport(
                settings,
                catalogContext,
                imageSize,
                starDetections,
                seededCandidate,
                projectedTriangleCandidate.detectionIndices,
                anchorCatalogIndices,
                anchorDistancePixels);
            projectedTriangleMaxAnchors = std::max(projectedTriangleMaxAnchors, anchorSupport);
            if (debugTargetTriangle)
            {
                qDebug().noquote() << "CameraPlateSolver: projected triangle target seed"
                    << "anchorSupport" << anchorSupport
                    << "matches" << seededCandidate.matchCount
                    << "rms" << seededCandidate.rmsErrorPixels
                    << "Az" << seededCandidate.azimuthDegrees
                    << "El" << seededCandidate.elevationDegrees
                    << "Roll" << seededCandidate.rollDegrees
                    << "FoV" << seededCandidate.fovDegrees
                    << debugTriangleAnchorSummary(
                        catalogContext,
                        starDetections,
                        projectedTriangleCandidate.detectionIndices,
                        anchorCatalogIndices);
            }
            if (anchorSupport < 3) {
                continue;
            }
            ++projectedTriangleAnchorSupported;

            Evaluation candidate = evaluatePose(
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
            candidate.anchored = true;
            candidate.guidedTriangle = true;
            candidate.anchorDetectionIndex = projectedTriangleCandidate.detectionIndices[0];
            candidate.anchorCatalogIndex = anchorCatalogIndices[0];
            candidate.secondaryAnchorDetectionIndex = projectedTriangleCandidate.detectionIndices[1];
            candidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
            candidate.tertiaryAnchorDetectionIndex = projectedTriangleCandidate.detectionIndices[2];
            candidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];

            const double guidedTriangleSeedRmsCap = std::min(
                std::max(18.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.75),
                static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.90);
            if (candidate.valid
                && (candidate.matchCount >= std::max(4, settings.m_plateSolveMinMatches))
                && (candidate.rmsErrorPixels <= guidedTriangleSeedRmsCap)
                && hasAcceptableBrightnessConsistency(candidate))
            {
                seeds.append(candidate);
                ++projectedTriangleSeeds;
                if (debugTargetTriangle) {
                    qDebug() << "CameraPlateSolver: projected triangle target appended";
                }
            }

            Evaluation verifiedCandidate = verifyBlindSeedCandidate(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                candidate);
            if (verifiedCandidate.valid)
            {
                verifiedCandidate.anchored = true;
                verifiedCandidate.guidedTriangle = true;
                verifiedCandidate.anchorDetectionIndex = projectedTriangleCandidate.detectionIndices[0];
                verifiedCandidate.anchorCatalogIndex = anchorCatalogIndices[0];
                verifiedCandidate.secondaryAnchorDetectionIndex = projectedTriangleCandidate.detectionIndices[1];
                verifiedCandidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
                verifiedCandidate.tertiaryAnchorDetectionIndex = projectedTriangleCandidate.detectionIndices[2];
                verifiedCandidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];
                seeds.append(verifiedCandidate);
                ++projectedTriangleSeeds;
                ++projectedTriangleVerifiedSeeds;
                if (debugTargetTriangle) {
                    qDebug() << "CameraPlateSolver: projected triangle target verified appended";
                }
            }
        }
    }
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleAnchorPairs"), projectedAnchorPairs);
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleCandidates"), projectedTriangleCandidateCount);
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleRatioMatches"), projectedTriangleRatioMatches);
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleKept"), projectedTriangleCandidates.size());
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleEvaluations"), projectedTriangleEvaluations);
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleSeedValid"), projectedTriangleSeedValid);
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleAnchorSupported"), projectedTriangleAnchorSupported);
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleMaxAnchors"), projectedTriangleMaxAnchors);
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleSeeds"), projectedTriangleSeeds);
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleVerifiedSeeds"), projectedTriangleVerifiedSeeds);

    struct OrderedDetection
    {
        int detectionIndex = -1;
        double brightness = 0.0;
        double reliability = 0.0;
        double shapeScore = 0.0;
        double score = 0.0;
    };
    QVector<OrderedDetection> orderedDetections;
    orderedDetections.reserve(starDetections.size());
    for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
    {
        const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
        if (!isDetectionUsableForBrightPrior(detection)) {
            continue;
        }

        const double brightness = cachedDetectionBrightnessMetric(starDetections, detectionIndex);
        const double reliability = cachedDetectionReliabilityMetric(starDetections, detectionIndex);
        const double shapeScore = narrowGuidedAnchorShapeScore(detection);
        if (!detection.m_saturated
            && (brightness < 12.0)
            && (reliability < 10.0)
            && (shapeScore < 0.0))
        {
            continue;
        }

        OrderedDetection orderedDetection;
        orderedDetection.detectionIndex = detectionIndex;
        orderedDetection.brightness = brightness;
        orderedDetection.reliability = reliability;
        orderedDetection.shapeScore = shapeScore;
        orderedDetection.score = brightGuidedDetectionPriorityScore(
            detection,
            brightness,
            reliability,
            shapeScore);
        orderedDetections.append(orderedDetection);
    }

    std::sort(orderedDetections.begin(), orderedDetections.end(), [](const OrderedDetection& lhs, const OrderedDetection& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score > rhs.score;
        }
        return lhs.brightness > rhs.brightness;
    });
    const int orderedDetectionLimit = std::min(22, static_cast<int>(orderedDetections.size()));
    const int orderedCatalogLimit = std::min(24, static_cast<int>(anchorCatalogStars.size()));

    struct OrderedTriangleCandidate
    {
        std::array<int, 3> detectionIndices{{-1, -1, -1}};
        std::array<int, 3> catalogAnchorIndices{{-1, -1, -1}};
        double score = 0.0;
        double seedFov = 0.0;
        double similarityRmsPixels = 0.0;
        double brightnessMagnitudeError = 0.0;
    };
    QVector<OrderedTriangleCandidate> orderedTriangleCandidates;
    orderedTriangleCandidates.reserve(768);
    qint64 orderedTriangleCandidateCount = 0;
    qint64 orderedTriangleRatioMatches = 0;
    const double orderedRatioTolerance = 0.075;
    const double orderedSimilarityTolerancePixels = std::max(
        static_cast<double>(settings.m_plateSolveMatchRadius) * 1.25,
        28.0);

    for (int a = 0; a < orderedDetectionLimit; ++a)
    {
        if (isCancellationRequested()) {
            return seeds;
        }
        for (int b = a + 1; b < orderedDetectionLimit; ++b)
        {
            for (int c = b + 1; c < orderedDetectionLimit; ++c)
            {
                std::array<int, 3> detectionOrder {{
                    orderedDetections[a].detectionIndex,
                    orderedDetections[b].detectionIndex,
                    orderedDetections[c].detectionIndex
                }};
                std::sort(detectionOrder.begin(), detectionOrder.end(), [this, &starDetections](int lhs, int rhs) {
                    const double lhsBrightness = cachedDetectionBrightnessMetric(starDetections, lhs);
                    const double rhsBrightness = cachedDetectionBrightnessMetric(starDetections, rhs);
                    if (!qFuzzyCompare(lhsBrightness + 1.0, rhsBrightness + 1.0)) {
                        return lhsBrightness > rhsBrightness;
                    }
                    return cachedDetectionReliabilityMetric(starDetections, lhs)
                        > cachedDetectionReliabilityMetric(starDetections, rhs);
                });
                const std::array<QPointF, 3> detectionPoints {{
                    starDetections[detectionOrder[0]].m_center,
                    starDetections[detectionOrder[1]].m_center,
                    starDetections[detectionOrder[2]].m_center
                }};
                const TriangleSignature detectionSignature = buildTriangleSignature(detectionPoints);
                if (detectionSignature.longestDistance < 20.0) {
                    continue;
                }

                for (int i = 0; i < orderedCatalogLimit; ++i)
                {
                    for (int j = i + 1; j < orderedCatalogLimit; ++j)
                    {
                        for (int k = j + 1; k < orderedCatalogLimit; ++k)
                        {
                            ++orderedTriangleCandidateCount;
                            std::array<int, 3> catalogOrder {{i, j, k}};
                            std::sort(catalogOrder.begin(), catalogOrder.end(), [&anchorCatalogStars](int lhs, int rhs) {
                                const double lhsMagnitude = anchorCatalogStars[lhs].visibleStar.magnitude;
                                const double rhsMagnitude = anchorCatalogStars[rhs].visibleStar.magnitude;
                                if (!qFuzzyCompare(lhsMagnitude + 1.0, rhsMagnitude + 1.0)) {
                                    return lhsMagnitude < rhsMagnitude;
                                }
                                return anchorCatalogStars[lhs].visibleStar.catalogIndex
                                    < anchorCatalogStars[rhs].visibleStar.catalogIndex;
                            });

                            const std::array<QPointF, 3> orderedCatalogProjectedPoints {{
                                anchorCatalogStars[catalogOrder[0]].projectedPoint,
                                anchorCatalogStars[catalogOrder[1]].projectedPoint,
                                anchorCatalogStars[catalogOrder[2]].projectedPoint
                            }};
                            const TriangleSignature catalogSignature = buildTriangleSignature(orderedCatalogProjectedPoints);
                            if (catalogSignature.longestDistance < 10.0) {
                                continue;
                            }

                            const double ratioError = std::fabs(detectionSignature.ratioShortToLong - catalogSignature.ratioShortToLong)
                                + std::fabs(detectionSignature.ratioMidToLong - catalogSignature.ratioMidToLong);
                            if (ratioError > orderedRatioTolerance) {
                                continue;
                            }
                            ++orderedTriangleRatioMatches;

                            const std::array<VisibleCatalogStar, 3> orderedStars {{
                                anchorCatalogStars[catalogOrder[0]].visibleStar,
                                anchorCatalogStars[catalogOrder[1]].visibleStar,
                                anchorCatalogStars[catalogOrder[2]].visibleStar
                            }};
                            const double catalogMagnitudeSum =
                                orderedStars[0].magnitude + orderedStars[1].magnitude + orderedStars[2].magnitude;
                            const double shapePenalty = std::max(0.0, 20.0 - narrowGuidedAnchorShapeScore(starDetections[detectionOrder[0]]))
                                + std::max(0.0, 20.0 - narrowGuidedAnchorShapeScore(starDetections[detectionOrder[1]]))
                                + std::max(0.0, 20.0 - narrowGuidedAnchorShapeScore(starDetections[detectionOrder[2]]));

                            for (const std::array<int, 3>& permutation : kTrianglePermutations)
                            {
                                if (!triangleBrightnessOrderCompatible(
                                        starDetections,
                                        detectionOrder,
                                        orderedStars,
                                        permutation))
                                {
                                    continue;
                                }
                                const double brightnessMagnitudeError = triangleBrightnessMagnitudeError(
                                    starDetections,
                                    detectionOrder,
                                    orderedStars,
                                    permutation);
                                if (!std::isfinite(brightnessMagnitudeError)
                                    || (brightnessMagnitudeError > 4.5))
                                {
                                    continue;
                                }

                                const std::array<int, 3> mappedCatalogOrder {{
                                    catalogOrder[permutation[0]],
                                    catalogOrder[permutation[1]],
                                    catalogOrder[permutation[2]]
                                }};
                                const std::array<QPointF, 3> catalogProjectedPoints {{
                                    anchorCatalogStars[mappedCatalogOrder[0]].projectedPoint,
                                    anchorCatalogStars[mappedCatalogOrder[1]].projectedPoint,
                                    anchorCatalogStars[mappedCatalogOrder[2]].projectedPoint
                                }};
                                double similarityScale = 1.0;
                                double similarityRotationDegrees = 0.0;
                                double similarityRmsPixels = std::numeric_limits<double>::infinity();
                                if (!estimateScreenSimilarity(
                                        catalogProjectedPoints,
                                        detectionPoints,
                                        similarityScale,
                                        similarityRotationDegrees,
                                        similarityRmsPixels)
                                    || (similarityRmsPixels > orderedSimilarityTolerancePixels))
                                {
                                    continue;
                                }
                                Q_UNUSED(similarityRotationDegrees)

                                const double seedFov = plateSolveStartUsesFov(settings)
                                    ? std::clamp(
                                        static_cast<double>(settings.m_fov),
                                        static_cast<double>(CameraSettings::m_minFov),
                                        static_cast<double>(CameraSettings::m_maxFov))
                                    : std::clamp(
                                        static_cast<double>(settings.m_fov) / std::max(0.50, std::min(2.0, similarityScale)),
                                        static_cast<double>(CameraSettings::m_minFov),
                                        static_cast<double>(CameraSettings::m_maxFov));
                                if (!seedFovCompatibleWithStartFov(settings, seedFov)) {
                                    continue;
                                }

                                const double fovPenalty = plateSolveStartUsesFov(settings)
                                    ? std::fabs(seedFov - static_cast<double>(settings.m_fov)) * 20.0
                                    : 0.0;
                                OrderedTriangleCandidate candidate;
                                candidate.detectionIndices = detectionOrder;
                                candidate.catalogAnchorIndices = mappedCatalogOrder;
                                candidate.seedFov = seedFov;
                                candidate.similarityRmsPixels = similarityRmsPixels;
                                candidate.brightnessMagnitudeError = brightnessMagnitudeError;
                                candidate.score = similarityRmsPixels * 4.0
                                    + ratioError * 160.0
                                    + catalogMagnitudeSum * 9.0
                                    + shapePenalty
                                    + fovPenalty
                                    + brightnessMagnitudeError * 55.0;
                                orderedTriangleCandidates.append(candidate);
                            }
                        }
                    }
                }
            }
        }
    }

    std::sort(orderedTriangleCandidates.begin(), orderedTriangleCandidates.end(), [](const OrderedTriangleCandidate& lhs, const OrderedTriangleCandidate& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score < rhs.score;
        }
        if (!qFuzzyCompare(lhs.brightnessMagnitudeError + 1.0, rhs.brightnessMagnitudeError + 1.0)) {
            return lhs.brightnessMagnitudeError < rhs.brightnessMagnitudeError;
        }
        return lhs.similarityRmsPixels < rhs.similarityRmsPixels;
    });
    if (orderedTriangleCandidates.size() > 768) {
        orderedTriangleCandidates.resize(768);
    }

    qint64 orderedTriangleEvaluations = 0;
    qint64 orderedTriangleSeedValid = 0;
    qint64 orderedTriangleAnchorSupported = 0;
    qint64 orderedTriangleSeeds = 0;
    qint64 orderedTriangleVerifiedSeeds = 0;
    int orderedTriangleMaxAnchors = 0;
    const int orderedTriangleEvaluationLimit = 1500;
    const std::array<double, 3> orderedFovScales = plateSolveStartUsesFov(settings)
        ? std::array<double, 3>{{1.0, 1.0, 1.0}}
        : std::array<double, 3>{{0.98, 1.0, 1.02}};
    for (const OrderedTriangleCandidate& orderedTriangleCandidate : orderedTriangleCandidates)
    {
        if (isCancellationRequested()
            || (orderedTriangleEvaluations >= orderedTriangleEvaluationLimit)
            || (seeds.size() >= 96))
        {
            break;
        }

        const std::array<VisibleCatalogStar, 3> orderedStars {{
            anchorCatalogStars[orderedTriangleCandidate.catalogAnchorIndices[0]].visibleStar,
            anchorCatalogStars[orderedTriangleCandidate.catalogAnchorIndices[1]].visibleStar,
            anchorCatalogStars[orderedTriangleCandidate.catalogAnchorIndices[2]].visibleStar
        }};
        const std::array<int, 3> anchorDetectionIndices = orderedTriangleCandidate.detectionIndices;
        const std::array<int, 3> anchorCatalogIndices {{
            orderedStars[0].catalogIndex,
            orderedStars[1].catalogIndex,
            orderedStars[2].catalogIndex
        }};
        const bool debugTargetTriangle =
            debugTriangleContainsCatalogStar(catalogContext, anchorCatalogIndices);
        QVector<int> allowedCatalogIndices {
            anchorCatalogIndices[0],
            anchorCatalogIndices[1],
            anchorCatalogIndices[2]
        };
        double previousSeedFov = -1.0;
        for (double fovScale : orderedFovScales)
        {
            if (isCancellationRequested()
                || (orderedTriangleEvaluations >= orderedTriangleEvaluationLimit)
                || (seeds.size() >= 96))
            {
                break;
            }

            const double seedFov = std::clamp(
                orderedTriangleCandidate.seedFov * fovScale,
                static_cast<double>(CameraSettings::m_minFov),
                static_cast<double>(CameraSettings::m_maxFov));
            if (std::fabs(seedFov - previousSeedFov) < 1e-9) {
                continue;
            }
            previousSeedFov = seedFov;
            if (!seedFovCompatibleWithStartFov(settings, seedFov)) {
                continue;
            }

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

            std::array<SkyVector, 3> sourceVectors;
            bool haveSourceVectors = true;
            for (int idx = 0; idx < 3; ++idx)
            {
                if (!unprojectPixelToVector(
                        baseProjector,
                        starDetections[anchorDetectionIndices[idx]].m_center,
                        sourceVectors[idx]))
                {
                    haveSourceVectors = false;
                    break;
                }
            }
            if (!haveSourceVectors) {
                continue;
            }

            const std::array<SkyVector, 3> targetVectors {{
                orderedStars[0].vector,
                orderedStars[1].vector,
                orderedStars[2].vector
            }};
            double seedAzimuth = 0.0;
            double seedElevation = 0.0;
            double seedRoll = 0.0;
            double triangleRmsAngularError = std::numeric_limits<double>::infinity();
            double triangleMaxAngularError = std::numeric_limits<double>::infinity();
            if (!poseFromThreeVectorPairs(
                    baseProjector,
                    sourceVectors,
                    targetVectors,
                    seedAzimuth,
                    seedElevation,
                    seedRoll,
                    &triangleRmsAngularError,
                    &triangleMaxAngularError))
            {
                continue;
            }
            Q_UNUSED(triangleRmsAngularError)
            Q_UNUSED(triangleMaxAngularError)

            const double directionDelta = std::acos(std::clamp(
                dot(vectorFromAltAz(seedAzimuth, seedElevation), vectorFromAltAz(settings.m_azimuth, settings.m_elevation)),
                -1.0,
                1.0)) * 180.0 / kPi;
            if (directionDelta > startDirectionMaxDelta) {
                continue;
            }

            Evaluation seededCandidate = evaluatePose(
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
            ++orderedTriangleEvaluations;
            if (!seededCandidate.valid) {
                if (debugTargetTriangle)
                {
                    qDebug().noquote() << "CameraPlateSolver: ordered triangle target seed invalid"
                        << debugTriangleAnchorSummary(
                            catalogContext,
                            starDetections,
                            anchorDetectionIndices,
                            anchorCatalogIndices);
                }
                continue;
            }
            ++orderedTriangleSeedValid;

            const double anchorDistancePixels = std::min(
                128.0,
                std::max(
                    static_cast<double>(settings.m_plateSolveMatchRadius),
                    static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 4.0));
            const int orderedAnchorSupport = countProjectedAnchorSupport(
                    settings,
                    catalogContext,
                    imageSize,
                    starDetections,
                    seededCandidate,
                    anchorDetectionIndices,
                    anchorCatalogIndices,
                    anchorDistancePixels);
                orderedTriangleMaxAnchors = std::max(orderedTriangleMaxAnchors, orderedAnchorSupport);
                if (debugTargetTriangle)
                {
                    qDebug().noquote() << "CameraPlateSolver: ordered triangle target seed"
                        << "anchorSupport" << orderedAnchorSupport
                        << "matches" << seededCandidate.matchCount
                        << "rms" << seededCandidate.rmsErrorPixels
                        << "Az" << seededCandidate.azimuthDegrees
                        << "El" << seededCandidate.elevationDegrees
                        << "Roll" << seededCandidate.rollDegrees
                        << "FoV" << seededCandidate.fovDegrees
                        << debugTriangleAnchorSummary(
                            catalogContext,
                            starDetections,
                            anchorDetectionIndices,
                            anchorCatalogIndices);
                }
                if (orderedAnchorSupport < 3)
                {
                    continue;
                }
                ++orderedTriangleAnchorSupported;

                Evaluation candidate = evaluatePose(
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
                candidate.anchored = true;
                candidate.guidedTriangle = true;
                candidate.anchorDetectionIndex = anchorDetectionIndices[0];
                candidate.anchorCatalogIndex = anchorCatalogIndices[0];
                candidate.secondaryAnchorDetectionIndex = anchorDetectionIndices[1];
                candidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
                candidate.tertiaryAnchorDetectionIndex = anchorDetectionIndices[2];
                candidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];

                const double guidedTriangleSeedRmsCap = std::min(
                    std::max(18.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.75),
                    static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.90);
                if (candidate.valid
                    && (candidate.matchCount >= std::max(4, settings.m_plateSolveMinMatches))
                    && (candidate.rmsErrorPixels <= guidedTriangleSeedRmsCap)
                    && hasAcceptableBrightnessConsistency(candidate))
                {
                    seeds.append(candidate);
                    ++orderedTriangleSeeds;
                    if (debugTargetTriangle) {
                        qDebug() << "CameraPlateSolver: ordered triangle target appended";
                    }
                }

                Evaluation verifiedCandidate = verifyBlindSeedCandidate(
                    settings,
                    catalogContext,
                    imageSize,
                    captureDateTimeUtc,
                    starDetections,
                    detectionIndices,
                    candidate);
                if (verifiedCandidate.valid)
                {
                    verifiedCandidate.anchored = true;
                    verifiedCandidate.guidedTriangle = true;
                    verifiedCandidate.anchorDetectionIndex = anchorDetectionIndices[0];
                    verifiedCandidate.anchorCatalogIndex = anchorCatalogIndices[0];
                    verifiedCandidate.secondaryAnchorDetectionIndex = anchorDetectionIndices[1];
                    verifiedCandidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
                    verifiedCandidate.tertiaryAnchorDetectionIndex = anchorDetectionIndices[2];
                    verifiedCandidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];
                    seeds.append(verifiedCandidate);
                    ++orderedTriangleSeeds;
                    ++orderedTriangleVerifiedSeeds;
                    if (debugTargetTriangle) {
                        qDebug() << "CameraPlateSolver: ordered triangle target verified appended";
                    }
                }
        }
    }
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleCandidates"), orderedTriangleCandidateCount);
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleRatioMatches"), orderedTriangleRatioMatches);
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleKept"), orderedTriangleCandidates.size());
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleEvaluations"), orderedTriangleEvaluations);
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleSeedValid"), orderedTriangleSeedValid);
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleAnchorSupported"), orderedTriangleAnchorSupported);
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleMaxAnchors"), orderedTriangleMaxAnchors);
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleSeeds"), orderedTriangleSeeds);
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleVerifiedSeeds"), orderedTriangleVerifiedSeeds);

    for (int a = 0; a < anchors.size(); ++a)
    {
        if (isCancellationRequested()) {
            return seeds;
        }
        for (int b = a + 1; b < anchors.size(); ++b)
        {
            if ((anchors[a].detectionIndex == anchors[b].detectionIndex)
                || (anchors[a].catalogIndex == anchors[b].catalogIndex))
            {
                continue;
            }
            for (int c = b + 1; c < anchors.size(); ++c)
            {
                if ((anchors[a].detectionIndex == anchors[c].detectionIndex)
                    || (anchors[b].detectionIndex == anchors[c].detectionIndex)
                    || (anchors[a].catalogIndex == anchors[c].catalogIndex)
                    || (anchors[b].catalogIndex == anchors[c].catalogIndex))
                {
                    continue;
                }
                ++ratioCandidates;

                const QPointF& da = starDetections[anchors[a].detectionIndex].m_center;
                const QPointF& db = starDetections[anchors[b].detectionIndex].m_center;
                const QPointF& dc = starDetections[anchors[c].detectionIndex].m_center;
                const std::array<double, 3> detectionRatios = sortedRatios({{
                    QLineF(da, db).length(),
                    QLineF(da, dc).length(),
                    QLineF(db, dc).length()
                }});
                if (detectionRatios[2] < 20.0) {
                    continue;
                }

                const SkyVector& va = anchors[a].catalogStar.vector;
                const SkyVector& vb = anchors[b].catalogStar.vector;
                const SkyVector& vc = anchors[c].catalogStar.vector;
                const std::array<double, 3> catalogAngularDistances {{
                    std::acos(std::clamp(dot(va, vb), -1.0, 1.0)) * 180.0 / kPi,
                    std::acos(std::clamp(dot(va, vc), -1.0, 1.0)) * 180.0 / kPi,
                    std::acos(std::clamp(dot(vb, vc), -1.0, 1.0)) * 180.0 / kPi
                }};
                const std::array<double, 3> catalogRatios = sortedRatios(catalogAngularDistances);
                if (catalogRatios[2] <= 0.01) {
                    continue;
                }

                const double ratioError = std::fabs(detectionRatios[0] - catalogRatios[0])
                    + std::fabs(detectionRatios[1] - catalogRatios[1]);
                if (ratioError > ratioTolerance) {
                    continue;
                }

                ++ratioMatches;
                const double baseFov = std::clamp(
                    catalogRatios[2] * static_cast<double>(std::max(imageSize.width(), imageSize.height())) / std::max(1.0, detectionRatios[2]),
                    static_cast<double>(CameraSettings::m_minFov),
                    static_cast<double>(CameraSettings::m_maxFov));
                if (!seedFovCompatibleWithStartFov(settings, baseFov)) {
                    continue;
                }

                const SkyVector triangleCenter = normalize({
                    va.x + vb.x + vc.x,
                    va.y + vb.y + vc.y,
                    va.z + vb.z + vc.z
                });
                if (length(triangleCenter) <= 0.0) {
                    continue;
                }
                const double centerDelta = std::acos(std::clamp(
                    dot(triangleCenter, vectorFromAltAz(settings.m_azimuth, settings.m_elevation)), -1.0, 1.0)) * 180.0 / kPi;
                if (centerDelta > startDirectionMaxDelta) {
                    continue;
                }

                const double fovPenalty = plateSolveStartUsesFov(settings)
                    ? std::fabs(baseFov - static_cast<double>(settings.m_fov)) * 20.0
                    : 0.0;
                TriangleCandidate triangleCandidate;
                triangleCandidate.anchorIndices = {{a, b, c}};
                triangleCandidate.score = anchors[a].score + anchors[b].score + anchors[c].score + ratioError * 140.0 + fovPenalty;
                triangleCandidate.baseFov = baseFov;
                triangleCandidates.append(triangleCandidate);
            }
        }
    }

    if (triangleCandidates.isEmpty()) {
        recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleAnchors"), anchors.size());
        recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleRatioCandidates"), ratioCandidates);
        recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleRatioMatches"), ratioMatches);
        return seeds;
    }

    std::sort(triangleCandidates.begin(), triangleCandidates.end(), [](const TriangleCandidate& lhs, const TriangleCandidate& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score < rhs.score;
        }
        return lhs.baseFov < rhs.baseFov;
    });
    if (triangleCandidates.size() > 2048) {
        triangleCandidates.resize(2048);
    }

    const std::array<double, 3> seedFovScales = {{0.96, 1.0, 1.04}};
    qint64 seedEvaluations = 0;
    qint64 verifiedSeeds = 0;
    const int seedEvaluationLimit = 4000;
    const int seedLimit = 64;

    for (const TriangleCandidate& triangleCandidate : triangleCandidates)
    {
        if (isCancellationRequested() || (seedEvaluations >= seedEvaluationLimit) || (seeds.size() >= seedLimit)) {
            break;
        }

        std::array<AnchorCandidate, 3> triangleAnchors {{
            anchors[triangleCandidate.anchorIndices[0]],
            anchors[triangleCandidate.anchorIndices[1]],
            anchors[triangleCandidate.anchorIndices[2]]
        }};
        std::array<int, 3> anchorDetectionIndices {{
            triangleAnchors[0].detectionIndex,
            triangleAnchors[1].detectionIndex,
            triangleAnchors[2].detectionIndex
        }};
        std::array<int, 3> anchorCatalogIndices {{
            triangleAnchors[0].catalogIndex,
            triangleAnchors[1].catalogIndex,
            triangleAnchors[2].catalogIndex
        }};
        const bool debugTargetTriangle =
            debugTriangleContainsCatalogStar(catalogContext, anchorCatalogIndices);
        QVector<int> allowedCatalogIndices {
            anchorCatalogIndices[0],
            anchorCatalogIndices[1],
            anchorCatalogIndices[2]
        };

        for (double fovScale : seedFovScales)
        {
            if (isCancellationRequested() || (seedEvaluations >= seedEvaluationLimit) || (seeds.size() >= seedLimit)) {
                break;
            }
            const double seedFov = std::clamp(
                (plateSolveStartUsesFov(settings) ? static_cast<double>(settings.m_fov) : triangleCandidate.baseFov) * fovScale,
                static_cast<double>(CameraSettings::m_minFov),
                static_cast<double>(CameraSettings::m_maxFov));
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

            std::array<SkyVector, 3> sourceVectors;
            bool haveSourceVectors = true;
            for (int idx = 0; idx < 3; ++idx)
            {
                if (!unprojectPixelToVector(
                        baseProjector,
                        starDetections[anchorDetectionIndices[idx]].m_center,
                        sourceVectors[idx]))
                {
                    haveSourceVectors = false;
                    break;
                }
            }
            if (!haveSourceVectors) {
                continue;
            }

            std::array<SkyVector, 3> targetVectors {{
                triangleAnchors[0].catalogStar.vector,
                triangleAnchors[1].catalogStar.vector,
                triangleAnchors[2].catalogStar.vector
            }};
            double seedAzimuth = 0.0;
            double seedElevation = 0.0;
            double seedRoll = 0.0;
            double triangleRmsAngularError = std::numeric_limits<double>::infinity();
            double triangleMaxAngularError = std::numeric_limits<double>::infinity();
            if (!poseFromThreeVectorPairs(
                    baseProjector,
                    sourceVectors,
                    targetVectors,
                    seedAzimuth,
                    seedElevation,
                    seedRoll,
                    &triangleRmsAngularError,
                    &triangleMaxAngularError))
            {
                continue;
            }
            Q_UNUSED(triangleRmsAngularError)
            Q_UNUSED(triangleMaxAngularError)

            const double directionDelta = std::acos(std::clamp(
                dot(vectorFromAltAz(seedAzimuth, seedElevation), vectorFromAltAz(settings.m_azimuth, settings.m_elevation)),
                -1.0,
                1.0)) * 180.0 / kPi;
            if (directionDelta > startDirectionMaxDelta) {
                continue;
            }

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
                if (debugTargetTriangle)
                {
                    qDebug().noquote() << "CameraPlateSolver: anchor triangle target seed invalid"
                        << debugTriangleAnchorSummary(
                            catalogContext,
                            starDetections,
                            anchorDetectionIndices,
                            anchorCatalogIndices);
                }
                continue;
            }

            const double anchorDistancePixels = std::min(
                static_cast<double>(settings.m_plateSolveMatchRadius),
                std::max(6.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius)));
            const int anchorSupport = countProjectedAnchorSupport(
                    settings,
                    catalogContext,
                    imageSize,
                    starDetections,
                    seededCandidate,
                    anchorDetectionIndices,
                    anchorCatalogIndices,
                    anchorDistancePixels);
            if (debugTargetTriangle)
            {
                qDebug().noquote() << "CameraPlateSolver: anchor triangle target seed"
                    << "anchorSupport" << anchorSupport
                    << "matches" << seededCandidate.matchCount
                    << "rms" << seededCandidate.rmsErrorPixels
                    << "Az" << seededCandidate.azimuthDegrees
                    << "El" << seededCandidate.elevationDegrees
                    << "Roll" << seededCandidate.rollDegrees
                    << "FoV" << seededCandidate.fovDegrees
                    << debugTriangleAnchorSummary(
                        catalogContext,
                        starDetections,
                        anchorDetectionIndices,
                        anchorCatalogIndices);
            }
            if (anchorSupport < 3)
            {
                continue;
            }

            Evaluation candidate = evaluatePose(
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
            candidate.anchored = true;
            candidate.guidedTriangle = true;
            candidate.anchorDetectionIndex = anchorDetectionIndices[0];
            candidate.anchorCatalogIndex = anchorCatalogIndices[0];
            candidate.secondaryAnchorDetectionIndex = anchorDetectionIndices[1];
            candidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
            candidate.tertiaryAnchorDetectionIndex = anchorDetectionIndices[2];
            candidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];

            if (candidate.valid
                && (candidate.matchCount >= std::max(3, settings.m_plateSolveMinMatches)))
            {
                seeds.append(candidate);
                if (debugTargetTriangle) {
                    qDebug() << "CameraPlateSolver: anchor triangle target appended";
                }
            }

            Evaluation verifiedCandidate = verifyBlindSeedCandidate(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                candidate);
            if (verifiedCandidate.valid)
            {
                verifiedCandidate.anchored = true;
                verifiedCandidate.guidedTriangle = true;
                verifiedCandidate.anchorDetectionIndex = anchorDetectionIndices[0];
                verifiedCandidate.anchorCatalogIndex = anchorCatalogIndices[0];
                verifiedCandidate.secondaryAnchorDetectionIndex = anchorDetectionIndices[1];
                verifiedCandidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
                verifiedCandidate.tertiaryAnchorDetectionIndex = anchorDetectionIndices[2];
                verifiedCandidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];
                seeds.append(verifiedCandidate);
                ++verifiedSeeds;
                if (debugTargetTriangle) {
                    qDebug() << "CameraPlateSolver: anchor triangle target verified appended";
                }
            }
        }
    }

    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleAnchors"), anchors.size());
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleRatioCandidates"), ratioCandidates);
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleRatioMatches"), ratioMatches);
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleCandidates"), triangleCandidates.size());
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleSeedEvaluations"), seedEvaluations);
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleVerifiedSeeds"), verifiedSeeds);

    std::sort(seeds.begin(), seeds.end(), [this](const Evaluation& lhs, const Evaluation& rhs) {
        return isBetterWeakModeEvaluation(lhs, rhs);
    });
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
    const bool useNarrowGuidedPairSeeds = plateSolveStartUsesDirection(settings)
        && !isWidePlateSolveContext(settings)
        && (isNarrowField(settings));
    if (isCancellationRequested()
        || (!isWidePlateSolveContext(settings) && !useNarrowGuidedPairSeeds)
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
    if (useNarrowGuidedPairSeeds)
    {
        QSet<int> brightDetectionSeen;
        brightDetectionSeen.reserve(brightDetectionIndices.size() + 64);
        for (int detectionIndex : brightDetectionIndices) {
            brightDetectionSeen.insert(detectionIndex);
        }

        QVector<int> broadStarLikeDetections;
        broadStarLikeDetections.reserve(starDetections.size());
        for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
        {
            const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
            if (detection.m_hotPixelSuspect) {
                continue;
            }
            if ((detection.m_saturated && (detection.m_fwhm >= 3.0f))
                || (narrowGuidedAnchorShapeScore(detection) >= 35.0))
            {
                broadStarLikeDetections.append(detectionIndex);
            }
        }
        std::sort(broadStarLikeDetections.begin(), broadStarLikeDetections.end(), [this, &starDetections](int lhs, int rhs) {
            const double lhsScore = narrowGuidedAnchorShapeScore(starDetections[lhs])
                + std::min(30.0, std::log1p(cachedDetectionBrightnessMetric(starDetections, lhs)) * 3.0)
                + std::min(20.0, std::log1p(cachedDetectionReliabilityMetric(starDetections, lhs)) * 5.0);
            const double rhsScore = narrowGuidedAnchorShapeScore(starDetections[rhs])
                + std::min(30.0, std::log1p(cachedDetectionBrightnessMetric(starDetections, rhs)) * 3.0)
                + std::min(20.0, std::log1p(cachedDetectionReliabilityMetric(starDetections, rhs)) * 5.0);
            if (!qFuzzyCompare(lhsScore + 1.0, rhsScore + 1.0)) {
                return lhsScore > rhsScore;
            }
            return cachedDetectionBrightnessMetric(starDetections, lhs)
                > cachedDetectionBrightnessMetric(starDetections, rhs);
        });
        const int broadDetectionLimit = std::min(64, static_cast<int>(broadStarLikeDetections.size()));
        for (int i = 0; i < broadDetectionLimit; ++i)
        {
            const int detectionIndex = broadStarLikeDetections[i];
            if (brightDetectionSeen.contains(detectionIndex)) {
                continue;
            }
            brightDetectionSeen.insert(detectionIndex);
            brightDetectionIndices.append(detectionIndex);
        }
    }
    if (brightDetectionIndices.size() < 2) {
        return seeds;
    }

    QVector<VisibleCatalogStar> brightCatalogStars;
    const double brightCatalogMaxMagnitude = useNarrowGuidedPairSeeds
        ? std::min(settings.m_plateSolveMaxMagnitude, kNarrowGuidedBrightCatalogMaxMagnitude)
        : kWideFovBrightFirstPassMaxMagnitude;
    const int brightCatalogLimit = useNarrowGuidedPairSeeds ? 96 : 20;
    brightCatalogStars.reserve(brightCatalogLimit);
    QSet<int> brightCatalogSeen;
    brightCatalogSeen.reserve(brightCatalogLimit + 32);
    for (const VisibleCatalogStar& visibleStar : visibleStars)
    {
        if (visibleStar.magnitude > brightCatalogMaxMagnitude) {
            break;
        }
        brightCatalogStars.append(visibleStar);
        brightCatalogSeen.insert(visibleStar.catalogIndex);
        if (brightCatalogStars.size() >= brightCatalogLimit) {
            break;
        }
    }
    if (useNarrowGuidedPairSeeds)
    {
        const int namedCatalogExtraLimit = 256;
        int namedCatalogExtraCount = 0;
        const auto appendNamedCatalogStar = [&](const VisibleCatalogStar& visibleStar) {
            if (visibleStar.magnitude > brightCatalogMaxMagnitude
                || brightCatalogSeen.contains(visibleStar.catalogIndex)
                || (visibleStar.catalogIndex < 0)
                || (visibleStar.catalogIndex >= catalogContext.catalogStars.size()))
            {
                return false;
            }

            const QString name = catalogDisplayName(catalogContext.catalogStars[visibleStar.catalogIndex]);
            if (!name.startsWith(QStringLiteral("HIP "), Qt::CaseInsensitive)
                && !name.startsWith(QStringLiteral("HR "), Qt::CaseInsensitive)
                && !name.startsWith(QStringLiteral("HD "), Qt::CaseInsensitive))
            {
                return false;
            }

            brightCatalogStars.append(visibleStar);
            brightCatalogSeen.insert(visibleStar.catalogIndex);
            return true;
        };
        const SkyVector center = normalize(vectorFromAltAz(settings.m_azimuth, settings.m_elevation));
        const double coreRadiusDegrees = std::max(1.5, static_cast<double>(settings.m_fov) * 1.75);
        const double coreMinDot = std::cos(coreRadiusDegrees * kPi / 180.0);
        for (const VisibleCatalogStar& visibleStar : catalogContext.visibleStars)
        {
            if (visibleStar.magnitude > brightCatalogMaxMagnitude) {
                break;
            }
            if (dot(center, visibleStar.vector) < coreMinDot) {
                continue;
            }
            if (appendNamedCatalogStar(visibleStar) && (++namedCatalogExtraCount >= namedCatalogExtraLimit)) {
                break;
            }
        }
        for (const VisibleCatalogStar& visibleStar : visibleStars)
        {
            if (visibleStar.magnitude > brightCatalogMaxMagnitude) {
                break;
            }
            if (appendNamedCatalogStar(visibleStar) && (++namedCatalogExtraCount >= namedCatalogExtraLimit)) {
                break;
            }
        }
        if (namedCatalogExtraCount < namedCatalogExtraLimit)
        {
            const double localRadiusDegrees = std::max(
                static_cast<double>(settings.m_plateSolveSearchRadius) + static_cast<double>(settings.m_fov) * 2.0,
                static_cast<double>(settings.m_fov) * 4.0);
            const double minDot = std::cos(localRadiusDegrees * kPi / 180.0);
            for (const VisibleCatalogStar& visibleStar : catalogContext.visibleStars)
            {
                if (visibleStar.magnitude > brightCatalogMaxMagnitude) {
                    continue;
                }
                if (dot(center, visibleStar.vector) < minDot) {
                    continue;
                }
                if (appendNamedCatalogStar(visibleStar) && (++namedCatalogExtraCount >= namedCatalogExtraLimit)) {
                    break;
                }
            }
        }
    }
    if (brightCatalogStars.size() < 2) {
        return seeds;
    }

    // Precompute the angular separation of every bright catalog-star pair once. The inner
    // seed loop below compares each detection pair's separation against every catalog pair's
    // separation; that catalog separation depends only on the catalog pair, not the detection
    // pair, yet the original code recomputed acos(dot(...)) for all ~N^2 catalog pairs on every
    // one of the ~thousands of detection pairs (hundreds of millions of acos calls on dense
    // narrow-field frames). Caching it turns the hot inner test into a single array lookup while
    // leaving the iteration order — and therefore the set/order of seeds produced — unchanged.
    const int brightCatalogCount = brightCatalogStars.size();
    QVector<double> catalogPairSeparationRadians(
        static_cast<qsizetype>(brightCatalogCount) * static_cast<qsizetype>(brightCatalogCount), 0.0);
    for (int i = 0; i < brightCatalogCount; ++i)
    {
        const SkyVector& iVector = brightCatalogStars[i].vector;
        for (int j = i + 1; j < brightCatalogCount; ++j)
        {
            const double separation = std::acos(std::clamp(
                dot(iVector, brightCatalogStars[j].vector), -1.0, 1.0));
            catalogPairSeparationRadians[static_cast<qsizetype>(i) * brightCatalogCount + j] = separation;
            catalogPairSeparationRadians[static_cast<qsizetype>(j) * brightCatalogCount + i] = separation;
        }
    }

    // Per-star neighbour index: for each catalog star, the other stars ordered by angular
    // separation, with the matching separations stored in a parallel array. The seed loop
    // accepts a catalog partner only when its separation lies within a tolerance window of the
    // current detection pair's separation; a binary search over this sorted index finds that
    // window directly instead of scanning all ~N partners per detection pair. Narrow guided
    // solves then rank the candidate catalog pairs so the limited seed budget is spent on
    // the brightest and most separation-consistent possibilities first, instead of on
    // arbitrary catalog-index order aliases.
    QVector<QVector<double>> catalogNeighborSortedSeparation(brightCatalogCount);
    QVector<QVector<int>> catalogNeighborSortedIndex(brightCatalogCount);
    {
        QVector<int> neighborOrder;
        for (int i = 0; i < brightCatalogCount; ++i)
        {
            const qsizetype rowBase = static_cast<qsizetype>(i) * brightCatalogCount;
            neighborOrder.clear();
            neighborOrder.reserve(brightCatalogCount - 1);
            for (int j = 0; j < brightCatalogCount; ++j)
            {
                if (j != i) {
                    neighborOrder.append(j);
                }
            }
            std::sort(neighborOrder.begin(), neighborOrder.end(), [&](int a, int b) {
                return catalogPairSeparationRadians[rowBase + a] < catalogPairSeparationRadians[rowBase + b];
            });
            QVector<double>& rowSeparations = catalogNeighborSortedSeparation[i];
            QVector<int>& rowPartners = catalogNeighborSortedIndex[i];
            rowSeparations.reserve(neighborOrder.size());
            rowPartners.reserve(neighborOrder.size());
            for (int j : neighborOrder)
            {
                rowSeparations.append(catalogPairSeparationRadians[rowBase + j]);
                rowPartners.append(j);
            }
        }
    }
    const bool useNarrowGuidedNoRoll = useNarrowGuidedPairSeeds && !plateSolveStartUsesRoll(settings);
    const bool useNarrowGuidedRadialPrior = useNarrowGuidedNoRoll
        && (static_cast<double>(settings.m_plateSolveSearchRadius)
            <= std::max(1.0, static_cast<double>(settings.m_fov) * 1.25));
    SkyProjector radialProjector;
    QPointF radialCenter;
    double radialTolerancePixels = 0.0;
    QVector<double> radialDetectionRadii;
    QVector<double> brightCatalogRadialRadii;
    if (useNarrowGuidedRadialPrior)
    {
        const bool useStartLens = plateSolveStartUsesLens(settings);
        radialProjector = createProjector(
            settings,
            imageSize,
            settings.m_azimuth,
            settings.m_elevation,
            0.0,
            settings.m_fov,
            useStartLens ? settings.m_lensCenterOffsetX : 0.0,
            useStartLens ? settings.m_lensCenterOffsetY : 0.0,
            useStartLens ? settings.m_lensDistortionK1 : 0.0);
        radialCenter = projectorPrincipalPoint(radialProjector);
        radialTolerancePixels = std::max(
            static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 6.0,
            std::max(imageSize.width(), imageSize.height()) * 0.12);
        if (radialProjector.valid)
        {
            radialDetectionRadii.resize(starDetections.size());
            for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex) {
                radialDetectionRadii[detectionIndex] = pointDistancePixels(starDetections[detectionIndex].m_center, radialCenter);
            }

            brightCatalogRadialRadii.resize(brightCatalogStars.size());
            for (int catalogIndex = 0; catalogIndex < brightCatalogStars.size(); ++catalogIndex)
            {
                QPointF projectedPoint;
                brightCatalogRadialRadii[catalogIndex] = projectVector(radialProjector, brightCatalogStars[catalogIndex].vector, projectedPoint)
                    ? pointDistancePixels(projectedPoint, radialCenter)
                    : std::numeric_limits<double>::quiet_NaN();
            }
        }
    }

    const auto isRadiallyCompatibleWithGuidedStart = [&](int detectionIndex, int brightCatalogIndex) {
        if (!useNarrowGuidedRadialPrior || !radialProjector.valid) {
            return true;
        }
        if ((detectionIndex < 0)
            || (detectionIndex >= radialDetectionRadii.size())
            || (brightCatalogIndex < 0)
            || (brightCatalogIndex >= brightCatalogRadialRadii.size())
            || !std::isfinite(brightCatalogRadialRadii[brightCatalogIndex]))
        {
            return false;
        }
        return std::fabs(radialDetectionRadii[detectionIndex] - brightCatalogRadialRadii[brightCatalogIndex]) <= radialTolerancePixels;
    };

    if (useNarrowGuidedRadialPrior)
    {
        if (radialProjector.valid)
        {
            QSet<int> radialSeen;
            radialSeen.reserve(brightDetectionIndices.size() + brightCatalogStars.size() * 4);
            for (int detectionIndex : brightDetectionIndices) {
                radialSeen.insert(detectionIndex);
            }
            QHash<int, double> namedRadialAnchorScores;
            namedRadialAnchorScores.reserve(brightDetectionIndices.size() + 64);
            for (int catalogIndex = 0; catalogIndex < brightCatalogStars.size(); ++catalogIndex)
            {
                if ((catalogIndex >= brightCatalogRadialRadii.size())
                    || !std::isfinite(brightCatalogRadialRadii[catalogIndex]))
                {
                    continue;
                }
                const double projectedRadius = brightCatalogRadialRadii[catalogIndex];
                const bool namedCatalogStar = (brightCatalogStars[catalogIndex].catalogIndex >= 0)
                    && (brightCatalogStars[catalogIndex].catalogIndex < catalogContext.catalogStars.size())
                    && isNamedSparseGuidedCatalogStar(catalogContext.catalogStars[brightCatalogStars[catalogIndex].catalogIndex]);
                QVector<QPair<double, int>> radialCandidates;
                radialCandidates.reserve(starDetections.size());
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
                    const double detectionRadius = radialDetectionRadii[detectionIndex];
                    const double radialError = std::fabs(detectionRadius - projectedRadius);
                    if (radialError > radialTolerancePixels) {
                        continue;
                    }
                    const double score = radialError
                        + detection.m_aspectRatio * 2.0
                        - std::min(12.0, std::log1p(reliability) * 1.2)
                        - std::min(30.0, narrowGuidedAnchorShapeScore(detection) * 0.35);
                    radialCandidates.append(qMakePair(score, detectionIndex));
                }
                std::sort(radialCandidates.begin(), radialCandidates.end(), [](const QPair<double, int>& lhs, const QPair<double, int>& rhs) {
                    if (!qFuzzyCompare(lhs.first + 1.0, rhs.first + 1.0)) {
                        return lhs.first < rhs.first;
                    }
                    return lhs.second < rhs.second;
                });
                const int perCatalogLimit = std::min(
                    namedCatalogStar ? 24 : 8,
                    static_cast<int>(radialCandidates.size()));
                for (int i = 0; i < perCatalogLimit; ++i)
                {
                    const int detectionIndex = radialCandidates[i].second;
                    if (namedCatalogStar)
                    {
                        const double namedScore = radialCandidates[i].first
                            + brightCatalogStars[catalogIndex].magnitude * 6.0;
                        const auto existingScoreIt = namedRadialAnchorScores.constFind(detectionIndex);
                        if ((existingScoreIt == namedRadialAnchorScores.cend())
                            || (namedScore < existingScoreIt.value()))
                        {
                            namedRadialAnchorScores.insert(detectionIndex, namedScore);
                        }
                    }
                    if (radialSeen.contains(detectionIndex)) {
                        continue;
                    }
                    radialSeen.insert(detectionIndex);
                    brightDetectionIndices.append(detectionIndex);
                }
            }
            if (!namedRadialAnchorScores.isEmpty())
            {
                QVector<QPair<double, int>> namedRadialAnchors;
                namedRadialAnchors.reserve(namedRadialAnchorScores.size());
                for (auto it = namedRadialAnchorScores.cbegin(); it != namedRadialAnchorScores.cend(); ++it) {
                    namedRadialAnchors.append(qMakePair(it.value(), it.key()));
                }
                std::sort(namedRadialAnchors.begin(), namedRadialAnchors.end(), [](const QPair<double, int>& lhs, const QPair<double, int>& rhs) {
                    if (!qFuzzyCompare(lhs.first + 1.0, rhs.first + 1.0)) {
                        return lhs.first < rhs.first;
                    }
                    return lhs.second < rhs.second;
                });

                QVector<int> reorderedBrightDetections;
                reorderedBrightDetections.reserve(brightDetectionIndices.size());
                QSet<int> reorderedSeen;
                reorderedSeen.reserve(brightDetectionIndices.size());
                for (const QPair<double, int>& namedAnchor : namedRadialAnchors)
                {
                    if (reorderedSeen.contains(namedAnchor.second)) {
                        continue;
                    }
                    reorderedSeen.insert(namedAnchor.second);
                    reorderedBrightDetections.append(namedAnchor.second);
                }
                for (int detectionIndex : brightDetectionIndices)
                {
                    if (reorderedSeen.contains(detectionIndex)) {
                        continue;
                    }
                    reorderedSeen.insert(detectionIndex);
                    reorderedBrightDetections.append(detectionIndex);
                }
                brightDetectionIndices = std::move(reorderedBrightDetections);
            }
        }

    }

    const int brightDetectionLimit = useNarrowGuidedPairSeeds ? 128 : 10;
    if (brightDetectionIndices.size() > brightDetectionLimit) {
        brightDetectionIndices.resize(brightDetectionLimit);
    }
    if (useNarrowGuidedPairSeeds
        && qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE"))
    {
        const int debugDetectionCount = std::min(48, static_cast<int>(brightDetectionIndices.size()));
        qDebug() << "CameraPlateSolver: bright-pair detection pool"
                 << brightDetectionIndices.size()
                 << "showing" << debugDetectionCount;
        for (int i = 0; i < debugDetectionCount; ++i)
        {
            const int detectionIndex = brightDetectionIndices[i];
            if ((detectionIndex < 0) || (detectionIndex >= starDetections.size())) {
                continue;
            }
            const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
            qDebug().noquote().nospace()
                << "CameraPlateSolver: bright detection #" << i
                << " idx=" << detectionIndex
                << " x=" << detection.m_center.x()
                << " y=" << detection.m_center.y()
                << " snr=" << detection.m_snr
                << " fwhm=" << detection.m_fwhm
                << " flux=" << detection.m_flux
                << " peak=" << detection.m_peakValue
                << " round=" << detection.m_roundness
                << " aspect=" << detection.m_aspectRatio
                << " sat=" << detection.m_saturated
                << " shape=" << narrowGuidedAnchorShapeScore(detection)
                << " bright=" << cachedDetectionBrightnessMetric(starDetections, detectionIndex)
                << " reliab=" << cachedDetectionReliabilityMetric(starDetections, detectionIndex);
        }
    }

    QVector<double> seedFovs;
    if (plateSolveStartUsesFov(settings))
    {
        for (double scale : {1.0, 0.96, 1.04})
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
    QVector<int> narrowPairSeedVerificationDetectionIndices;
    const QVector<int>* pairSeedVerificationDetectionIndices = &detectionIndices;
    if (useNarrowGuidedPairSeeds)
    {
        QSet<int> verificationSeen;
        verificationSeen.reserve(std::min(static_cast<int>(starDetections.size()), 320));
        const auto appendVerificationDetection = [&](int detectionIndex) {
            if ((detectionIndex < 0)
                || (detectionIndex >= starDetections.size())
                || verificationSeen.contains(detectionIndex)
                || starDetections[detectionIndex].m_hotPixelSuspect)
            {
                return;
            }
            verificationSeen.insert(detectionIndex);
            narrowPairSeedVerificationDetectionIndices.append(detectionIndex);
        };
        for (int detectionIndex : detectionIndices) {
            appendVerificationDetection(detectionIndex);
        }
        for (int detectionIndex : brightDetectionIndices) {
            appendVerificationDetection(detectionIndex);
        }

        QVector<int> rankedVerificationDetections;
        rankedVerificationDetections.reserve(starDetections.size());
        for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
        {
            if (!starDetections[detectionIndex].m_hotPixelSuspect) {
                rankedVerificationDetections.append(detectionIndex);
            }
        }
        std::sort(rankedVerificationDetections.begin(), rankedVerificationDetections.end(), [this, &starDetections](int lhs, int rhs) {
            const double lhsScore = cachedDetectionReliabilityMetric(starDetections, lhs)
                + std::min(80.0, cachedDetectionBrightnessMetric(starDetections, lhs) * 0.04)
                + narrowGuidedAnchorShapeScore(starDetections[lhs]) * 0.35;
            const double rhsScore = cachedDetectionReliabilityMetric(starDetections, rhs)
                + std::min(80.0, cachedDetectionBrightnessMetric(starDetections, rhs) * 0.04)
                + narrowGuidedAnchorShapeScore(starDetections[rhs]) * 0.35;
            if (!qFuzzyCompare(lhsScore + 1.0, rhsScore + 1.0)) {
                return lhsScore > rhsScore;
            }
            return cachedDetectionBrightnessMetric(starDetections, lhs)
                > cachedDetectionBrightnessMetric(starDetections, rhs);
        });
        const int verificationDetectionLimit = std::min(256, static_cast<int>(starDetections.size()));
        for (int detectionIndex : rankedVerificationDetections)
        {
            if (narrowPairSeedVerificationDetectionIndices.size() >= verificationDetectionLimit) {
                break;
            }
            appendVerificationDetection(detectionIndex);
        }
        pairSeedVerificationDetectionIndices = &narrowPairSeedVerificationDetectionIndices;
    }
    QVector<quint8> pairSeedBaseDetectionSelected(starDetections.size(), 0);
    for (int detectionIndex : detectionIndices)
    {
        if ((detectionIndex >= 0) && (detectionIndex < pairSeedBaseDetectionSelected.size())) {
            pairSeedBaseDetectionSelected[detectionIndex] = 1;
        }
    }
    QVector<double> brightDetectionBrightness;
    brightDetectionBrightness.reserve(brightDetectionIndices.size());
    for (int detectionIndex : brightDetectionIndices) {
        brightDetectionBrightness.append(cachedDetectionBrightnessMetric(starDetections, detectionIndex));
    }
    qint64 seedEvaluations = 0;
    qint64 verifiedSeeds = 0;
    const qint64 verifiedSeedLimit = useNarrowGuidedPairSeeds ? 192 : 12;
    bool reachedSeedLimit = false;
    bool cancelledSeedSearch = false;
    const auto shouldStopBrightPairSeedSearch = [&]() {
        return isCancellationRequested() || cancelledSeedSearch || reachedSeedLimit;
    };
    const auto appendVerifiedSeed = [&](const Evaluation& seed) {
        seeds.append(seed);
        ++verifiedSeeds;
        if (verifiedSeeds >= verifiedSeedLimit) {
            reachedSeedLimit = true;
        }
    };

    struct BrightPairSeedWorkResult
    {
        QVector<Evaluation> seeds;
        qint64 seedEvaluations = 0;
        bool cancelled = false;
    };

    struct BrightCatalogPairCandidate
    {
        int firstCatalog = -1;
        int secondCatalog = -1;
        double separationErrorRadians = 0.0;
        double score = 0.0;
    };

    const auto processBrightPairFirstDetection = [&](SolverContext& context,
                                                     const SkyProjector& baseProjector,
                                                     const QVector<SkyVector>& brightDetectionVectors,
                                                     const QVector<quint8>& brightDetectionVectorValid,
                                                     double seedFov,
                                                     int firstDetection) {
        BrightPairSeedWorkResult result;
        if ((firstDetection < 0) || (firstDetection >= brightDetectionIndices.size() - 1)) {
            return result;
        }
        if ((firstDetection >= brightDetectionVectorValid.size()) || !brightDetectionVectorValid[firstDetection]) {
            return result;
        }

        QVector<BrightCatalogPairCandidate> catalogPairCandidates;
        catalogPairCandidates.reserve(brightCatalogCount);
        QVector<int> windowSecondCatalogs;
        windowSecondCatalogs.reserve(brightCatalogCount);
        QVector<int> seedDetectionIndices;
        seedDetectionIndices.reserve(detectionIndices.size() + 2);
        QVector<int> allowedCatalogIndices(2);
        const qsizetype localSeedLimit = useNarrowGuidedPairSeeds ? 8 : verifiedSeedLimit;
        const qsizetype localPrioritySeedLimit = useNarrowGuidedPairSeeds ? 16 : 0;
        const qsizetype localHardSeedLimit = localSeedLimit + localPrioritySeedLimit;
        const qsizetype localSeedLimitPerDetectionPair = useNarrowGuidedPairSeeds ? 8 : verifiedSeedLimit;
        const qsizetype localPrioritySeedLimitPerDetectionPair = useNarrowGuidedPairSeeds ? 16 : 0;
        result.seeds.reserve(static_cast<int>(std::min<qsizetype>(localHardSeedLimit, 16)));
        const auto localStop = [&]() {
            return context.isCancellationRequested()
                || (result.seeds.size() >= localHardSeedLimit);
        };
        qsizetype localPrioritySeeds = 0;

        const int detectionIndexA = brightDetectionIndices[firstDetection];
        const double detectionBrightnessA = brightDetectionBrightness[firstDetection];
        const SkyVector& sourceA = brightDetectionVectors[firstDetection];
        const bool detectionASelected = (detectionIndexA >= 0)
            && (detectionIndexA < pairSeedBaseDetectionSelected.size())
            && pairSeedBaseDetectionSelected[detectionIndexA];
        const auto isPriorityCatalogPairCandidate = [&](int firstCatalog, int secondCatalog) {
            if (!useNarrowGuidedPairSeeds) {
                return false;
            }

            const auto isPriorityCatalogStar = [&](int brightCatalogIndex) {
                if ((brightCatalogIndex < 0) || (brightCatalogIndex >= brightCatalogStars.size())) {
                    return false;
                }
                const int sourceCatalogIndex = brightCatalogStars[brightCatalogIndex].catalogIndex;
                if ((sourceCatalogIndex < 0) || (sourceCatalogIndex >= catalogContext.catalogStars.size())) {
                    return false;
                }
                const CatalogStar& catalogStar = catalogContext.catalogStars[sourceCatalogIndex];
                return (catalogStar.magnitude <= std::min(
                            static_cast<double>(settings.m_plateSolveMaxMagnitude),
                            10.5))
                    && isNamedSparseGuidedCatalogStar(catalogStar);
            };

            return isPriorityCatalogStar(firstCatalog)
                && isPriorityCatalogStar(secondCatalog);
        };
        for (int secondDetection = firstDetection + 1; secondDetection < brightDetectionIndices.size(); ++secondDetection)
        {
            if (localStop()) {
                break;
            }
            if ((secondDetection >= brightDetectionVectorValid.size()) || !brightDetectionVectorValid[secondDetection]) {
                continue;
            }

            const int detectionIndexB = brightDetectionIndices[secondDetection];
            const double detectionBrightnessB = brightDetectionBrightness[secondDetection];
            const double detectionLogBrightnessRatio = std::fabs(std::log(
                (detectionBrightnessA + 1.0) / (detectionBrightnessB + 1.0)));
            const QVector<int>* seedDetectionIndicesPtr = &detectionIndices;
            const bool detectionBSelected = (detectionIndexB >= 0)
                && (detectionIndexB < pairSeedBaseDetectionSelected.size())
                && pairSeedBaseDetectionSelected[detectionIndexB];
            if (!detectionASelected || !detectionBSelected)
            {
                seedDetectionIndices = detectionIndices;
                if (!detectionASelected) {
                    seedDetectionIndices.append(detectionIndexA);
                }
                if (!detectionBSelected && (detectionIndexB != detectionIndexA)) {
                    seedDetectionIndices.append(detectionIndexB);
                }
                seedDetectionIndicesPtr = &seedDetectionIndices;
            }
            const QVector<int>& seedMatchDetectionIndices = *seedDetectionIndicesPtr;
            const SkyVector& sourceB = brightDetectionVectors[secondDetection];
            const double sourceSeparationRadians = std::acos(std::clamp(dot(sourceA, sourceB), -1.0, 1.0));
            const double minSourceSeparationDegrees = useNarrowGuidedPairSeeds ? 0.05 : 1.0;
            if (sourceSeparationRadians < degToRad(minSourceSeparationDegrees)) {
                continue;
            }
            qsizetype localDetectionPairSeeds = 0;
            qsizetype localDetectionPairPrioritySeeds = 0;
            const auto detectionPairHardStop = [&]() {
                return localStop()
                    || ((localDetectionPairSeeds >= localSeedLimitPerDetectionPair)
                        && (localDetectionPairPrioritySeeds >= localPrioritySeedLimitPerDetectionPair));
            };
            const auto appendDetectionPairSeed = [&](const Evaluation& seed, bool prioritySeed) {
                if (detectionPairHardStop()) {
                    return;
                }
                if (prioritySeed)
                {
                    if ((localPrioritySeeds >= localPrioritySeedLimit)
                        || (localDetectionPairPrioritySeeds >= localPrioritySeedLimitPerDetectionPair))
                    {
                        return;
                    }
                    result.seeds.append(seed);
                    ++localPrioritySeeds;
                    ++localDetectionPairPrioritySeeds;
                    return;
                }
                if ((localDetectionPairSeeds >= localSeedLimitPerDetectionPair)
                    || (result.seeds.size() >= localSeedLimit))
                {
                    return;
                }
                result.seeds.append(seed);
                ++localDetectionPairSeeds;
            };
            const double separationToleranceRadians = useNarrowGuidedPairSeeds
                ? std::max(degToRad(0.04), sourceSeparationRadians * 0.18)
                : plateSolveStartUsesFov(settings)
                    ? std::max(degToRad(2.0), sourceSeparationRadians * 0.18)
                    : std::max(degToRad(5.0), sourceSeparationRadians * 0.30);
            const double windowLowSeparation = sourceSeparationRadians - separationToleranceRadians - 1e-9;
            const double windowHighSeparation = sourceSeparationRadians + separationToleranceRadians + 1e-9;

            catalogPairCandidates.clear();
            for (int firstCatalog = 0; firstCatalog < brightCatalogStars.size(); ++firstCatalog)
            {
                if (localStop()) {
                    break;
                }

                const QVector<double>& rowSeparations = catalogNeighborSortedSeparation[firstCatalog];
                const QVector<int>& rowPartners = catalogNeighborSortedIndex[firstCatalog];
                const auto windowBegin = std::lower_bound(
                    rowSeparations.cbegin(), rowSeparations.cend(), windowLowSeparation);
                const auto windowEnd = std::upper_bound(
                    rowSeparations.cbegin(), rowSeparations.cend(), windowHighSeparation);
                windowSecondCatalogs.clear();
                for (auto it = windowBegin; it != windowEnd; ++it) {
                    windowSecondCatalogs.append(rowPartners[it - rowSeparations.cbegin()]);
                }

                for (int secondCatalog : windowSecondCatalogs)
                {
                    if (localStop()) {
                        break;
                    }
                    if (firstCatalog == secondCatalog) {
                        continue;
                    }
                    const double catalogSeparationRadians = catalogPairSeparationRadians[
                        static_cast<qsizetype>(firstCatalog) * brightCatalogCount + secondCatalog];
                    const double separationErrorRadians = std::fabs(sourceSeparationRadians - catalogSeparationRadians);
                    if (separationErrorRadians > separationToleranceRadians) {
                        continue;
                    }
                    if (useNarrowGuidedPairSeeds)
                    {
                        const double catalogMagnitudeDelta = std::fabs(
                            brightCatalogStars[firstCatalog].magnitude
                            - brightCatalogStars[secondCatalog].magnitude);
                        if (((detectionLogBrightnessRatio < 0.05) && (catalogMagnitudeDelta > 6.5))
                            || ((detectionLogBrightnessRatio > 3.0) && (catalogMagnitudeDelta < 0.05)))
                        {
                            continue;
                        }
                    }
                    if (!isRadiallyCompatibleWithGuidedStart(detectionIndexA, firstCatalog)
                        || !isRadiallyCompatibleWithGuidedStart(detectionIndexB, secondCatalog))
                    {
                        continue;
                    }

                    const double firstMagnitude = brightCatalogStars[firstCatalog].magnitude;
                    const double secondMagnitude = brightCatalogStars[secondCatalog].magnitude;
                    double score = static_cast<double>(firstCatalog) * 1e-6
                        + static_cast<double>(secondCatalog) * 1e-9;
                    if (useNarrowGuidedPairSeeds)
                    {
                        const double signedDetectionLogBrightnessRatio = std::log(
                            (detectionBrightnessA + 1.0) / (detectionBrightnessB + 1.0));
                        const double signedCatalogLogBrightnessRatio = -0.9210340371976183
                            * (firstMagnitude - secondMagnitude);
                        const double catalogMagnitudeDelta = std::fabs(firstMagnitude - secondMagnitude);
                        const bool firstNamed = (brightCatalogStars[firstCatalog].catalogIndex >= 0)
                            && (brightCatalogStars[firstCatalog].catalogIndex < catalogContext.catalogStars.size())
                            && isNamedSparseGuidedCatalogStar(catalogContext.catalogStars[brightCatalogStars[firstCatalog].catalogIndex]);
                        const bool secondNamed = (brightCatalogStars[secondCatalog].catalogIndex >= 0)
                            && (brightCatalogStars[secondCatalog].catalogIndex < catalogContext.catalogStars.size())
                            && isNamedSparseGuidedCatalogStar(catalogContext.catalogStars[brightCatalogStars[secondCatalog].catalogIndex]);
                        score = (separationErrorRadians / std::max(degToRad(0.01), separationToleranceRadians)) * 80.0
                            + (firstMagnitude + secondMagnitude) * 1.6
                            + catalogMagnitudeDelta * 0.12
                            + std::fabs(signedDetectionLogBrightnessRatio - signedCatalogLogBrightnessRatio) * 2.0
                            - (firstNamed ? 8.0 : 0.0)
                            - (secondNamed ? 8.0 : 0.0);
                    }
                    catalogPairCandidates.append({firstCatalog, secondCatalog, separationErrorRadians, score});
                }
            }

            std::sort(catalogPairCandidates.begin(), catalogPairCandidates.end(),
                [](const BrightCatalogPairCandidate& lhs, const BrightCatalogPairCandidate& rhs) {
                    if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
                        return lhs.score < rhs.score;
                    }
                    if (!qFuzzyCompare(lhs.separationErrorRadians + 1.0, rhs.separationErrorRadians + 1.0)) {
                        return lhs.separationErrorRadians < rhs.separationErrorRadians;
                    }
                    if (lhs.firstCatalog != rhs.firstCatalog) {
                        return lhs.firstCatalog < rhs.firstCatalog;
                    }
                    return lhs.secondCatalog < rhs.secondCatalog;
                });

            const int maxCatalogPairCandidates = useNarrowGuidedPairSeeds
                ? std::min(static_cast<int>(catalogPairCandidates.size()), 192)
                : static_cast<int>(catalogPairCandidates.size());
            for (int catalogPairIndex = 0; catalogPairIndex < maxCatalogPairCandidates; ++catalogPairIndex)
            {
                if (detectionPairHardStop()) {
                    break;
                }
                const BrightCatalogPairCandidate& catalogPairCandidate = catalogPairCandidates[catalogPairIndex];
                const int firstCatalog = catalogPairCandidate.firstCatalog;
                const int secondCatalog = catalogPairCandidate.secondCatalog;
                const bool priorityCatalogPair = isPriorityCatalogPairCandidate(firstCatalog, secondCatalog);
                if ((localDetectionPairSeeds >= localSeedLimitPerDetectionPair)
                    && !priorityCatalogPair)
                {
                    continue;
                }
                if ((result.seeds.size() >= localSeedLimit)
                    && !priorityCatalogPair)
                {
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

                    allowedCatalogIndices[0] = brightCatalogStars[firstCatalog].catalogIndex;
                    allowedCatalogIndices[1] = brightCatalogStars[secondCatalog].catalogIndex;
                    const Evaluation seededCandidate = context.evaluatePose(
                        settings,
                        catalogContext,
                        imageSize,
                        captureDateTimeUtc,
                        starDetections,
                        seedMatchDetectionIndices,
                        seedAzimuth,
                        seedElevation,
                        seedRoll,
                        seedFov,
                        &allowedCatalogIndices,
                        fixedCenterOffsetX,
                        fixedCenterOffsetY,
                        fixedDistortionK1);
                    ++result.seedEvaluations;
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
                    const bool hasAnchorSupport = context.hasSeedAnchorSupport(seededCandidate, anchorDetectionIndices, anchorCatalogIndices, 2);
                    if (!hasAnchorSupport) {
                        continue;
                    }

                    Evaluation sparsePairCandidate = seededCandidate;
                    sparsePairCandidate.sparseGuidedPair = useNarrowGuidedPairSeeds;
                    sparsePairCandidate.anchored = useNarrowGuidedPairSeeds;
                    sparsePairCandidate.anchorDetectionIndex = detectionIndexA;
                    sparsePairCandidate.anchorCatalogIndex = brightCatalogStars[firstCatalog].catalogIndex;
                    sparsePairCandidate.secondaryAnchorDetectionIndex = detectionIndexB;
                    sparsePairCandidate.secondaryAnchorCatalogIndex = brightCatalogStars[secondCatalog].catalogIndex;
                    const auto applySparsePairAnchors = [&](Evaluation& candidate) {
                        candidate.sparseGuidedPair = useNarrowGuidedPairSeeds;
                        candidate.anchored = useNarrowGuidedPairSeeds;
                        candidate.anchorDetectionIndex = detectionIndexA;
                        candidate.anchorCatalogIndex = brightCatalogStars[firstCatalog].catalogIndex;
                        candidate.secondaryAnchorDetectionIndex = detectionIndexB;
                        candidate.secondaryAnchorCatalogIndex = brightCatalogStars[secondCatalog].catalogIndex;
                    };
                    if (context.isAcceptableSparseGuidedPairEvaluation(
                            settings,
                            catalogContext,
                            starDetections,
                            sparsePairCandidate))
                    {
                        appendDetectionPairSeed(sparsePairCandidate, priorityCatalogPair);
                        if (detectionPairHardStop()) {
                            break;
                        }
                    }

                    const QVector<int>& verificationDetectionIndices = useNarrowGuidedPairSeeds
                        ? *pairSeedVerificationDetectionIndices
                        : seedMatchDetectionIndices;
                    const Evaluation candidate = context.evaluatePose(
                        settings,
                        catalogContext,
                        imageSize,
                        captureDateTimeUtc,
                        starDetections,
                        verificationDetectionIndices,
                        seededCandidate.azimuthDegrees,
                        seededCandidate.elevationDegrees,
                        seededCandidate.rollDegrees,
                        seededCandidate.fovDegrees,
                        nullptr,
                        fixedCenterOffsetX,
                        fixedCenterOffsetY,
                        fixedDistortionK1);
                    const Evaluation verifiedCandidate = context.verifyBlindSeedCandidate(
                        settings,
                        catalogContext,
                        imageSize,
                        captureDateTimeUtc,
                        starDetections,
                        verificationDetectionIndices,
                        candidate);
                    if (verifiedCandidate.valid)
                    {
                        Evaluation verifiedSparsePairCandidate = verifiedCandidate;
                        applySparsePairAnchors(verifiedSparsePairCandidate);
                        appendDetectionPairSeed(verifiedSparsePairCandidate, priorityCatalogPair);
                    }
                }
            }

        result.cancelled = context.isCancellationRequested();
        return result;
    };

    const int brightPairSeedWorkerThreadCount = [&]() {
        if (!useNarrowGuidedPairSeeds || (brightDetectionIndices.size() < 64)) {
            return 1;
        }
        const int idealThreadCount = QThread::idealThreadCount();
        if (idealThreadCount <= 2) {
            return 1;
        }
        const int workLimitedThreadCount = brightDetectionIndices.size() / 12;
        return std::max(1, std::min(workLimitedThreadCount, idealThreadCount - 2));
    }();
    recordProfileMetric(QStringLiteral("search.brightPairSeedThreads"), brightPairSeedWorkerThreadCount);

    const auto mergeBrightPairSeedWorkResult = [&](const BrightPairSeedWorkResult& result) {
        seedEvaluations += result.seedEvaluations;
        if (result.cancelled) {
            cancelledSeedSearch = true;
            return;
        }
        for (const Evaluation& seed : result.seeds)
        {
            if (shouldStopBrightPairSeedSearch()) {
                break;
            }
            appendVerifiedSeed(seed);
        }
    };
    for (double seedFov : seedFovs)
    {
        if (shouldStopBrightPairSeedSearch()) break;
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
        QVector<SkyVector> brightDetectionVectors(brightDetectionIndices.size());
        QVector<quint8> brightDetectionVectorValid(brightDetectionIndices.size(), 0);
        for (int brightDetection = 0; brightDetection < brightDetectionIndices.size(); ++brightDetection)
        {
            const int detectionIndex = brightDetectionIndices[brightDetection];
            if ((detectionIndex >= 0)
                && (detectionIndex < starDetections.size())
                && unprojectPixelToVector(baseProjector, starDetections[detectionIndex].m_center, brightDetectionVectors[brightDetection]))
            {
                brightDetectionVectorValid[brightDetection] = 1;
            }
        }
        if (brightPairSeedWorkerThreadCount <= 1)
        {
            for (int firstDetection = 0; firstDetection < brightDetectionIndices.size(); ++firstDetection)
            {
                if (shouldStopBrightPairSeedSearch()) {
                    break;
                }
                const BrightPairSeedWorkResult workResult = processBrightPairFirstDetection(
                    *this,
                    baseProjector,
                    brightDetectionVectors,
                    brightDetectionVectorValid,
                    seedFov,
                    firstDetection);
                mergeBrightPairSeedWorkResult(workResult);
            }
        }
        else
        {
            QThreadPool brightPairSeedPool;
            brightPairSeedPool.setMaxThreadCount(brightPairSeedWorkerThreadCount);
            for (int batchStart = 0; batchStart < brightDetectionIndices.size(); batchStart += brightPairSeedWorkerThreadCount)
            {
                if (shouldStopBrightPairSeedSearch()) {
                    break;
                }
                const int remainingDetectionCount = static_cast<int>(brightDetectionIndices.size()) - batchStart;
                const int batchCount = std::min(brightPairSeedWorkerThreadCount, remainingDetectionCount);
                QVector<BrightPairSeedWorkResult> batchResults(batchCount);
                for (int batchIndex = 0; batchIndex < batchCount; ++batchIndex)
                {
                    const int firstDetection = batchStart + batchIndex;
                    brightPairSeedPool.start(QRunnable::create([&, batchIndex, firstDetection, seedFov, baseProjector]() {
                        SolverContext workerContext(m_owner);
                        workerContext.copySearchStateFrom(*this);
                        batchResults[batchIndex] = processBrightPairFirstDetection(
                            workerContext,
                            baseProjector,
                            brightDetectionVectors,
                            brightDetectionVectorValid,
                            seedFov,
                            firstDetection);
                    }));
                }
                brightPairSeedPool.waitForDone();
                for (const BrightPairSeedWorkResult& workResult : batchResults)
                {
                    mergeBrightPairSeedWorkResult(workResult);
                    if (shouldStopBrightPairSeedSearch()) {
                        break;
                    }
                }
            }
        }
    }
    recordProfileMetric(QStringLiteral("search.brightPairSeedEvaluations"), seedEvaluations);
    recordProfileMetric(QStringLiteral("search.brightPairVerifiedSeeds"), verifiedSeeds);

    QVector<Evaluation> sparseGuidedPairSeeds;
    if (useNarrowGuidedPairSeeds)
    {
        for (const Evaluation& seed : seeds)
        {
            if (seed.sparseGuidedPair) {
                sparseGuidedPairSeeds.append(seed);
            }
        }
        std::sort(sparseGuidedPairSeeds.begin(), sparseGuidedPairSeeds.end(), [this](const Evaluation& lhs, const Evaluation& rhs) {
            return isBetterGuidedDirectionEvaluation(lhs, rhs);
        });
        if (qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE"))
        {
            const int debugSeedCount = std::min(64, static_cast<int>(sparseGuidedPairSeeds.size()));
            qDebug() << "CameraPlateSolver: sparse guided pair seeds"
                     << sparseGuidedPairSeeds.size()
                     << "showing" << debugSeedCount;
            for (int i = 0; i < debugSeedCount; ++i)
            {
                const Evaluation& seed = sparseGuidedPairSeeds[i];
                const QString primaryName = ((seed.anchorCatalogIndex >= 0) && (seed.anchorCatalogIndex < catalogContext.catalogStars.size()))
                    ? catalogDisplayName(catalogContext.catalogStars[seed.anchorCatalogIndex])
                    : QString();
                const QString secondaryName = ((seed.secondaryAnchorCatalogIndex >= 0) && (seed.secondaryAnchorCatalogIndex < catalogContext.catalogStars.size()))
                    ? catalogDisplayName(catalogContext.catalogStars[seed.secondaryAnchorCatalogIndex])
                    : QString();
                qDebug().noquote().nospace()
                    << "CameraPlateSolver: sparse seed #" << i
                    << " Az=" << seed.azimuthDegrees
                    << " El=" << seed.elevationDegrees
                    << " Roll=" << seed.rollDegrees
                    << " FoV=" << seed.fovDegrees
                    << " matches=" << seed.matchCount
                    << " RMS=" << seed.rmsErrorPixels
                    << " primary=" << primaryName << "/" << seed.anchorDetectionIndex
                    << " secondary=" << secondaryName << "/" << seed.secondaryAnchorDetectionIndex;
            }
        }
        if (sparseGuidedPairSeeds.size() > 256)
        {
            QVector<Evaluation> retainedSparseSeeds;
            retainedSparseSeeds.reserve(512);
            QSet<qint64> retainedPoseCells;
            const double cellSizeDegrees = std::max(0.4, std::min(1.5, static_cast<double>(settings.m_fov)));
            const double rollCellSizeDegrees = std::max(10.0, std::min(45.0, static_cast<double>(settings.m_fov) * 12.0));
            const auto poseCellKey = [cellSizeDegrees, rollCellSizeDegrees](const Evaluation& seed) {
                const qint64 azCell = static_cast<qint64>(std::floor(normalizeDegrees(seed.azimuthDegrees) / cellSizeDegrees));
                const qint64 elCell = static_cast<qint64>(std::floor((std::clamp(seed.elevationDegrees, -90.0, 90.0) + 90.0) / cellSizeDegrees));
                const qint64 rollCell = static_cast<qint64>(std::floor(normalizeDegrees(seed.rollDegrees) / rollCellSizeDegrees));
                return ((azCell & 0xfffffLL) << 40)
                    ^ ((elCell & 0xfffffLL) << 12)
                    ^ (rollCell & 0xfffLL);
            };
            const auto appendSparseSeed = [&](const Evaluation& seed) {
                const bool alreadyKept = std::any_of(retainedSparseSeeds.cbegin(), retainedSparseSeeds.cend(), [&seed](const Evaluation& retainedSeed) {
                    return sameEvaluationIdentity(retainedSeed, seed);
                });
                if (alreadyKept) {
                    return false;
                }
                retainedSparseSeeds.append(seed);
                retainedPoseCells.insert(poseCellKey(seed));
                return true;
            };
            for (int i = 0; (i < sparseGuidedPairSeeds.size()) && (retainedSparseSeeds.size() < 256); ++i) {
                appendSparseSeed(sparseGuidedPairSeeds[i]);
            }

            QVector<Evaluation> diverseSparseSeeds = sparseGuidedPairSeeds;
            std::sort(diverseSparseSeeds.begin(), diverseSparseSeeds.end(), [this](const Evaluation& lhs, const Evaluation& rhs) {
                return isBetterWeakModeEvaluation(lhs, rhs);
            });
            for (const Evaluation& seed : diverseSparseSeeds)
            {
                if (retainedSparseSeeds.size() >= 512) {
                    break;
                }
                const qint64 cellKey = poseCellKey(seed);
                if (retainedPoseCells.contains(cellKey)) {
                    continue;
                }
                appendSparseSeed(seed);
            }
            sparseGuidedPairSeeds = std::move(retainedSparseSeeds);
        }
    }

    std::sort(seeds.begin(), seeds.end(), [this](const Evaluation& lhs, const Evaluation& rhs) {
        return isBetterWeakModeEvaluation(lhs, rhs);
    });
    if (seeds.size() > 64) {
        seeds.resize(64);
    }
    for (const Evaluation& sparseSeed : sparseGuidedPairSeeds)
    {
        const bool alreadyKept = std::any_of(seeds.cbegin(), seeds.cend(), [&sparseSeed](const Evaluation& seed) {
            return sameEvaluationIdentity(seed, sparseSeed);
        });
        if (!alreadyKept) {
            seeds.append(sparseSeed);
        }
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
    if (isCancellationRequested() || (visibleStars.size() < settings.m_plateSolveMinMatches)) {
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
    if (isCancellationRequested()) {
        return seeds;
    }
    const QVector<QuadSignature> catalogQuads = buildCatalogQuadSignatures(settings, visibleStars);
    if (isCancellationRequested() || detectionQuads.isEmpty() || catalogQuads.isEmpty()) {
        return seeds;
    }
    const double ratioTolerance = (isNarrowGuidedDirectionSolve(settings))
        ? 0.06
        : isWideFisheyeLens ? 0.06
        : 0.03;
    const bool ignoreOrientationHandedness = isWideFisheyeLens
        || (isNarrowGuidedDirectionSolve(settings));
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
        if (earlyExit || isCancellationRequested()) break;
        const int detectionFirstBin = signatureRatioBin(detectionQuad.edgeRatios[0], ratioTolerance);
        const int detectionSecondBin = signatureRatioBin(detectionQuad.edgeRatios[1], ratioTolerance);
        for (int firstBinOffset = -bucketRadius; firstBinOffset <= bucketRadius; ++firstBinOffset)
        {
            if (earlyExit || isCancellationRequested()) break;
            for (int secondBinOffset = -bucketRadius; secondBinOffset <= bucketRadius; ++secondBinOffset)
            {
                if (earlyExit || isCancellationRequested()) break;
                const auto bucketIt = catalogQuadBuckets.constFind(
                    signatureBucketKey(detectionFirstBin + firstBinOffset, detectionSecondBin + secondBinOffset));
                if (bucketIt == catalogQuadBuckets.constEnd()) {
                    continue;
                }
                for (int catalogQuadIndex : *bucketIt)
                {
                    if (earlyExit || isCancellationRequested()) break;
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
                if (earlyExit || isCancellationRequested()) break;
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
                    if (earlyExit || isCancellationRequested()) break;
                    const QLineF detectionBase(detectionPoints[0], detectionPoints[1]);
                    const QLineF projectedBase(projectedPoints[permutation[0]], projectedPoints[permutation[1]]);
                    double baseRoll = projectedBase.angleTo(detectionBase);
                    if (!std::isfinite(baseRoll)) {
                        baseRoll = 0.0;
                    }

                    // Sweep small roll perturbations to tolerate centroiding noise on the reference edge.
                    for (double rollDelta : {-10.0, -5.0, 0.0, 5.0, 10.0})
                    {
                        if (earlyExit || isCancellationRequested()) break;
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
                        const double anchorDistancePixels = std::min(
                            static_cast<double>(settings.m_plateSolveMatchRadius),
                            std::max(6.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius)));
                        const int seedAnchorMatches = countProjectedAnchorSupport(
                            settings,
                            catalogContext,
                            imageSize,
                            starDetections,
                            seededCandidate,
                            detectionQuad.indices,
                            anchorCatalogIndices,
                            anchorDistancePixels);
                        if (seedAnchorMatches < 3) {
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
    return selectConsensusSeedRepresentatives(settings, seeds, seedLimit, "quad");
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

const QVector<double>& detectionBrightnessRanks(const QVector<CameraPipelineStarDetection>& starDetections,
                                                const QVector<int>& detectionIndices)
{
    if (m_detectionBrightnessRankCache.size() != starDetections.size())
    {
        m_detectionBrightnessRankCache.resize(starDetections.size());
        std::fill(m_detectionBrightnessRankCache.begin(), m_detectionBrightnessRankCache.end(), 0.5);
        m_detectionBrightnessRankIndices.clear();
    }
    if (m_detectionBrightnessRankIndices == detectionIndices) {
        return m_detectionBrightnessRankCache;
    }

    for (int detectionIndex : m_detectionBrightnessRankIndices)
    {
        if ((detectionIndex >= 0) && (detectionIndex < m_detectionBrightnessRankCache.size())) {
            m_detectionBrightnessRankCache[detectionIndex] = 0.5;
        }
    }

    QVector<int> sorted = detectionIndices;
    std::sort(sorted.begin(), sorted.end(), [this, &starDetections](int lhs, int rhs) {
        const double lhsBrightness = cachedDetectionBrightnessMetric(starDetections, lhs);
        const double rhsBrightness = cachedDetectionBrightnessMetric(starDetections, rhs);
        if (!qFuzzyCompare(lhsBrightness + 1.0, rhsBrightness + 1.0)) {
            return lhsBrightness > rhsBrightness;
        }
        return starDetections[lhs].m_qualityScore > starDetections[rhs].m_qualityScore;
    });

    const double divisor = std::max(1, static_cast<int>(sorted.size()) - 1);
    for (int i = 0; i < sorted.size(); ++i)
    {
        const int detectionIndex = sorted[i];
        if ((detectionIndex >= 0) && (detectionIndex < m_detectionBrightnessRankCache.size())) {
            m_detectionBrightnessRankCache[detectionIndex] = static_cast<double>(i) / divisor;
        }
    }
    m_detectionBrightnessRankIndices = detectionIndices;
    return m_detectionBrightnessRankCache;
}

const QVector<double>& projectedBrightnessRanks(const QVector<ProjectedCatalogStar>& projectedStars)
{
    m_projectedBrightnessRankScratch.resize(projectedStars.size());
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
        const double divisor = std::max(1, static_cast<int>(projectedStars.size()) - 1);
        for (int i = 0; i < projectedStars.size(); ++i) {
            m_projectedBrightnessRankScratch[i] = static_cast<double>(i) / divisor;
        }
        return m_projectedBrightnessRankScratch;
    }

    QVector<int>& sorted = m_projectedBrightnessRankSortScratch;
    sorted.clear();
    sorted.reserve(projectedStars.size());
    for (int i = 0; i < projectedStars.size(); ++i) {
        sorted.append(i);
    }
    std::sort(sorted.begin(), sorted.end(), [&projectedStars](int lhs, int rhs) {
        return projectedStars[lhs].magnitude < projectedStars[rhs].magnitude;
    });

    const double divisor = std::max(1, static_cast<int>(sorted.size()) - 1);
    for (int i = 0; i < sorted.size(); ++i) {
        m_projectedBrightnessRankScratch[sorted[i]] = static_cast<double>(i) / divisor;
    }
    return m_projectedBrightnessRankScratch;
}

double matchBrightnessRankError(const QVector<CameraPipelineStarDetection>& starDetections,
                                const QVector<int>& detectionIndices,
                                const QVector<ProjectedCatalogStar>& projectedStars,
                                const QVector<Match>& matches)
{
    if (matches.size() < 2) {
        return 0.0;
    }

    const QVector<double>& detectionRanks = detectionBrightnessRanks(starDetections, detectionIndices);
    const QVector<double>& projectedRanks = projectedBrightnessRanks(projectedStars);
    int maxCatalogIndex = -1;
    for (const ProjectedCatalogStar& projectedStar : projectedStars) {
        maxCatalogIndex = std::max(maxCatalogIndex, projectedStar.catalogIndex);
    }
    if (maxCatalogIndex >= 0)
    {
        const int oldSize = m_catalogBrightnessRankGeneration.size();
        if (oldSize <= maxCatalogIndex)
        {
            m_catalogBrightnessRankGeneration.resize(maxCatalogIndex + 1);
            std::fill(
                m_catalogBrightnessRankGeneration.begin() + oldSize,
                m_catalogBrightnessRankGeneration.end(),
                0);
            m_catalogBrightnessRankScratch.resize(maxCatalogIndex + 1);
        }
    }
    {
        const int oldSize = m_brightnessMatchedDetectionGeneration.size();
        if (oldSize < starDetections.size())
        {
            m_brightnessMatchedDetectionGeneration.resize(starDetections.size());
            std::fill(
                m_brightnessMatchedDetectionGeneration.begin() + oldSize,
                m_brightnessMatchedDetectionGeneration.end(),
                0);
        }
    }
    ++m_brightnessRankGeneration;
    if (m_brightnessRankGeneration == std::numeric_limits<int>::max())
    {
        std::fill(m_catalogBrightnessRankGeneration.begin(), m_catalogBrightnessRankGeneration.end(), 0);
        std::fill(m_brightnessMatchedDetectionGeneration.begin(), m_brightnessMatchedDetectionGeneration.end(), 0);
        m_brightnessRankGeneration = 1;
    }
    const int rankGeneration = m_brightnessRankGeneration;
    for (int i = 0; i < projectedStars.size(); ++i)
    {
        const int catalogIndex = projectedStars[i].catalogIndex;
        if ((catalogIndex >= 0) && (catalogIndex < m_catalogBrightnessRankGeneration.size()))
        {
            m_catalogBrightnessRankGeneration[catalogIndex] = rankGeneration;
            m_catalogBrightnessRankScratch[catalogIndex] = (i < projectedRanks.size()) ? projectedRanks[i] : 0.5;
        }
    }

    double sumError = 0.0;
    int count = 0;
    for (const Match& match : matches)
    {
        if ((match.detectionIndex >= 0) && (match.detectionIndex < m_brightnessMatchedDetectionGeneration.size())) {
            m_brightnessMatchedDetectionGeneration[match.detectionIndex] = rankGeneration;
        }
        const double detectionRank = ((match.detectionIndex >= 0) && (match.detectionIndex < detectionRanks.size()))
            ? detectionRanks[match.detectionIndex]
            : 0.5;
        if ((match.catalogIndex < 0)
            || (match.catalogIndex >= m_catalogBrightnessRankGeneration.size())
            || (m_catalogBrightnessRankGeneration[match.catalogIndex] != rankGeneration))
        {
            continue;
        }
        sumError += std::fabs(detectionRank - m_catalogBrightnessRankScratch[match.catalogIndex]);
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
        const double detectionRank = ((detectionIndex >= 0) && (detectionIndex < detectionRanks.size()))
            ? detectionRanks[detectionIndex]
            : 0.5;
        if ((detectionRank > brightRankThreshold)
            || ((detectionIndex >= 0)
                && (detectionIndex < m_brightnessMatchedDetectionGeneration.size())
                && (m_brightnessMatchedDetectionGeneration[detectionIndex] == rankGeneration)))
        {
            continue;
        }

        sumError += 0.75 + 0.25 * (1.0 - detectionRank);
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

bool shouldPopulatePoseScoringMetrics(const CameraSettings& settings,
                                      const Evaluation& evaluation,
                                      bool forceSparseSeedMetrics = false) const
{
    if (!evaluation.valid || (evaluation.matchCount <= 0)) {
        return false;
    }
    if (forceSparseSeedMetrics) {
        return evaluation.matchCount >= 2;
    }

    // The brightness/magnitude scoring path builds rank maps and is paid many
    // thousands of times during seed sweeps.  It only matters once a pose has
    // enough matches to be a plausible winner.
    const int metricMatchThreshold = 2;
    return evaluation.matchCount >= metricMatchThreshold;
}

void populatePoseScoringMetrics(const CameraSettings& settings,
                                const QVector<CameraPipelineStarDetection>& starDetections,
                                const QVector<int>& detectionIndices,
                                const QVector<ProjectedCatalogStar>& projectedStars,
                                const QVector<CatalogStar>& catalogStars,
                                Evaluation& evaluation,
                                bool forceSparseSeedMetrics = false)
{
    if (!shouldPopulatePoseScoringMetrics(settings, evaluation, forceSparseSeedMetrics)) {
        return;
    }

    evaluation.brightnessRankError = matchBrightnessRankError(
        starDetections,
        detectionIndices,
        projectedStars,
        evaluation.matches);
    evaluation.meanCatalogMagnitude = meanCatalogMagnitudeForMatches(
        catalogStars,
        evaluation.matches);
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
    const QVector<double>& detectionRanks = detectionBrightnessRanks(starDetections, detectionIndices);
    const QVector<double>& detectionReliability = m_detectionReliabilityMetricCache;
    const QVector<double>& projectedRanks = projectedBrightnessRanks(projectedStars);
    const bool useNarrowGuidedBrightShapePrior =
        m_useDirectionSeedPreference && (m_directionSeedReferenceFovDegrees <= 5.0);
    const double maxDistanceSquared = matchRadiusPixels * matchRadiusPixels;
    const double cellSize = std::max(1.0, matchRadiusPixels);
    QVector<int>& projectedGridHeads = m_projectedStarGridHeadsScratch;
    QVector<int>& projectedGridNext = m_projectedStarGridNextScratch;
    QVector<int>& projectedCellX = m_projectedStarGridCellXScratch;
    QVector<int>& projectedCellY = m_projectedStarGridCellYScratch;
    projectedGridHeads.clear();
    projectedGridNext.resize(projectedStars.size());
    projectedCellX.resize(projectedStars.size());
    projectedCellY.resize(projectedStars.size());
    int minCellX = std::numeric_limits<int>::max();
    int maxCellX = std::numeric_limits<int>::min();
    int minCellY = std::numeric_limits<int>::max();
    int maxCellY = std::numeric_limits<int>::min();
    for (int projectedIndex = 0; projectedIndex < projectedStars.size(); ++projectedIndex)
    {
        const QPointF& point = projectedStars[projectedIndex].point;
        const int cellX = static_cast<int>(std::floor(point.x() / cellSize));
        const int cellY = static_cast<int>(std::floor(point.y() / cellSize));
        projectedCellX[projectedIndex] = cellX;
        projectedCellY[projectedIndex] = cellY;
        minCellX = std::min(minCellX, cellX);
        maxCellX = std::max(maxCellX, cellX);
        minCellY = std::min(minCellY, cellY);
        maxCellY = std::max(maxCellY, cellY);
    }
    if (!projectedStars.isEmpty())
    {
        const int gridWidth = maxCellX - minCellX + 1;
        const int gridHeight = maxCellY - minCellY + 1;
        projectedGridHeads.resize(gridWidth * gridHeight);
        std::fill(projectedGridHeads.begin(), projectedGridHeads.end(), -1);
        for (int projectedIndex = 0; projectedIndex < projectedStars.size(); ++projectedIndex)
        {
            const int gridIndex = (projectedCellY[projectedIndex] - minCellY) * gridWidth
                + (projectedCellX[projectedIndex] - minCellX);
            projectedGridNext[projectedIndex] = projectedGridHeads[gridIndex];
            projectedGridHeads[gridIndex] = projectedIndex;
        }
    }

    for (int detectionIndex : detectionIndices)
    {
        if ((detectionIndex < 0)
            || (detectionIndex >= starDetections.size())
            || starDetections[detectionIndex].m_hotPixelSuspect)
        {
            continue;
        }
        const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
        const QPointF detectionPoint = detection.m_center;
        const double detectionRank = (detectionIndex < detectionRanks.size())
            ? detectionRanks[detectionIndex]
            : 0.5;
        const double detectionReliabilityValue = (detectionIndex < detectionReliability.size())
            ? detectionReliability[detectionIndex]
            : cachedDetectionReliabilityMetric(starDetections, detectionIndex);
        const int cellX = static_cast<int>(std::floor(detectionPoint.x() / cellSize));
        const int cellY = static_cast<int>(std::floor(detectionPoint.y() / cellSize));
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                const int queryCellX = cellX + dx;
                const int queryCellY = cellY + dy;
                if (queryCellX < minCellX || queryCellX > maxCellX
                    || queryCellY < minCellY || queryCellY > maxCellY)
                {
                    continue;
                }

                const int gridIndex = (queryCellY - minCellY) * (maxCellX - minCellX + 1)
                    + (queryCellX - minCellX);
                for (int projectedIndex = projectedGridHeads[gridIndex]; projectedIndex >= 0; projectedIndex = projectedGridNext[projectedIndex])
                {
                    const ProjectedCatalogStar& projected = projectedStars[projectedIndex];
                    if (useNarrowGuidedBrightShapePrior
                        && isImplausiblyCompactBrightCatalogDetection(
                            detection,
                            projected.magnitude,
                            true))
                    {
                        continue;
                    }
                    const double deltaX = detectionPoint.x() - projected.point.x();
                    const double deltaY = detectionPoint.y() - projected.point.y();
                    const double distanceSquared = deltaX * deltaX + deltaY * deltaY;
                    if (distanceSquared > maxDistanceSquared) {
                        continue;
                    }

                    const double catalogRank = (projectedIndex < projectedRanks.size())
                        ? projectedRanks[projectedIndex]
                        : 0.5;
                    const double catalogAssignmentPenalty = faintCatalogAssignmentPenalty(projected.magnitude)
                        + brightDetectionFaintCatalogAssignmentPenalty(
                            detectionRank,
                            projected.magnitude,
                            useNarrowGuidedBrightShapePrior);
                    const double detectionReliabilityLog = std::log1p(detectionReliabilityValue);
                    candidatePairs.append({
                        detectionIndex,
                        projected.catalogIndex,
                        projectedIndex,
                        std::sqrt(distanceSquared),
                        std::fabs(detectionRank - catalogRank),
                        projected.magnitude,
                        0,
                        detectionReliabilityValue,
                        catalogAssignmentPenalty,
                        detectionReliabilityLog
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
    std::sort(candidatePairs.begin(), candidatePairs.end(), [matchRadiusPixels](const CandidatePair& lhs, const CandidatePair& rhs) {
        if (lhs.detectionIndex != rhs.detectionIndex) {
            return lhs.detectionIndex < rhs.detectionIndex;
        }
        const double lhsCost = lhs.distancePixels
            + matchRadiusPixels * (0.75 * lhs.brightnessRankError + lhs.catalogAssignmentPenalty)
            - std::min(matchRadiusPixels * 0.25, lhs.detectionReliabilityLog);
        const double rhsCost = rhs.distancePixels
            + matchRadiusPixels * (0.75 * rhs.brightnessRankError + rhs.catalogAssignmentPenalty)
            - std::min(matchRadiusPixels * 0.25, rhs.detectionReliabilityLog);
        return lhsCost < rhsCost;
    });
    {
        QVector<CandidatePair>& cappedPairs = m_cappedCandidatePairScratch;
        cappedPairs.clear();
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
        candidatePairs.swap(cappedPairs);
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

    std::sort(candidatePairs.begin(), candidatePairs.end(), [&catalogStars, &starDetections, matchRadiusPixels](const CandidatePair& lhs, const CandidatePair& rhs) {
        const double lhsSupportScore = static_cast<double>(lhs.geometricSupport)
            - 1.25 * lhs.brightnessRankError
            - lhs.catalogAssignmentPenalty
            + 0.20 * lhs.detectionReliabilityLog;
        const double rhsSupportScore = static_cast<double>(rhs.geometricSupport)
            - 1.25 * rhs.brightnessRankError
            - rhs.catalogAssignmentPenalty
            + 0.20 * rhs.detectionReliabilityLog;
        if (std::fabs(lhsSupportScore - rhsSupportScore) > 0.20) {
            return lhsSupportScore > rhsSupportScore;
        }
        const double lhsCost = lhs.distancePixels
            + matchRadiusPixels * (0.75 * lhs.brightnessRankError + lhs.catalogAssignmentPenalty)
            - std::min(matchRadiusPixels * 0.25, lhs.detectionReliabilityLog);
        const double rhsCost = rhs.distancePixels
            + matchRadiusPixels * (0.75 * rhs.brightnessRankError + rhs.catalogAssignmentPenalty)
            - std::min(matchRadiusPixels * 0.25, rhs.detectionReliabilityLog);
        if (!qFuzzyCompare(lhsCost + 1.0, rhsCost + 1.0)) {
            return lhsCost < rhsCost;
        }
        if (!qFuzzyCompare(starDetections[lhs.detectionIndex].m_qualityScore + 1.0f, starDetections[rhs.detectionIndex].m_qualityScore + 1.0f)) {
            return starDetections[lhs.detectionIndex].m_qualityScore > starDetections[rhs.detectionIndex].m_qualityScore;
        }

        return catalogStars[lhs.catalogIndex].magnitude < catalogStars[rhs.catalogIndex].magnitude;
    });

    QVector<Match> matches;
    if (m_detectionMatchGeneration.size() < starDetections.size()) {
        m_detectionMatchGeneration.resize(starDetections.size());
    }
    if (m_catalogMatchGeneration.size() < catalogStars.size()) {
        m_catalogMatchGeneration.resize(catalogStars.size());
    }
    ++m_matchGeneration;
    if (m_matchGeneration == std::numeric_limits<int>::max())
    {
        std::fill(m_detectionMatchGeneration.begin(), m_detectionMatchGeneration.end(), 0);
        std::fill(m_catalogMatchGeneration.begin(), m_catalogMatchGeneration.end(), 0);
        m_matchGeneration = 1;
    }

    for (const CandidatePair& pair : candidatePairs)
    {
        if ((pair.detectionIndex < 0)
            || (pair.detectionIndex >= m_detectionMatchGeneration.size())
            || (pair.catalogIndex < 0)
            || (pair.catalogIndex >= m_catalogMatchGeneration.size())
            || (m_detectionMatchGeneration[pair.detectionIndex] == m_matchGeneration)
            || (m_catalogMatchGeneration[pair.catalogIndex] == m_matchGeneration))
        {
            continue;
        }

        m_detectionMatchGeneration[pair.detectionIndex] = m_matchGeneration;
        m_catalogMatchGeneration[pair.catalogIndex] = m_matchGeneration;
        matches.append({pair.detectionIndex, pair.catalogIndex, pair.distancePixels});
    }

    return matches;
}

static void appendSupplementalMatches(const QVector<CameraPipelineStarDetection>& starDetections,
                                      const QVector<ProjectedCatalogStar>& projectedStars,
                                      double matchRadiusPixels,
                                      const QVector<int>* supplementalDetectionIndices,
                                      bool narrowGuidedSolve,
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
        if ((detectionIndex < 0)
            || (detectionIndex >= starDetections.size())
            || detectionMatched[detectionIndex]
            || starDetections[detectionIndex].m_hotPixelSuspect)
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
                    if (isImplausiblyCompactBrightCatalogDetection(
                            starDetections[detectionIndex],
                            projected.magnitude,
                            narrowGuidedSolve))
                    {
                        continue;
                    }
                    const double dxp = detectionPoint.x() - projected.point.x();
                    const double dyp = detectionPoint.y() - projected.point.y();
                    const double distanceSquared = dxp * dxp + dyp * dyp;
                    if (distanceSquared > maxDistanceSquared) {
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

    double sumSquaredError = 0.0;
    for (const Match& match : evaluation.matches) {
        sumSquaredError += match.distancePixels * match.distancePixels;
    }
    evaluation.rmsErrorPixels = std::sqrt(sumSquaredError / evaluation.matchCount);
    evaluation.valid = true;
    populatePoseScoringMetrics(
        settings,
        starDetections,
        detectionIndices,
        projectedStars,
        catalogContext.catalogStars,
        evaluation,
        allowedCatalogIndices && (allowedCatalogIndices->size() <= 2));
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

    double sumSq = 0.0;
    for (const Match& m : evaluation.matches)
        sumSq += m.distancePixels * m.distancePixels;
    evaluation.rmsErrorPixels = std::sqrt(sumSq / evaluation.matchCount);
    evaluation.valid = true;
    populatePoseScoringMetrics(
        settings,
        starDetections,
        detectionIndices,
        projectedStars,
        catalogContext.catalogStars,
        evaluation);
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

    double sumSquaredError = 0.0;
    for (const Match& match : evaluation.matches) {
        sumSquaredError += match.distancePixels * match.distancePixels;
    }
    evaluation.rmsErrorPixels = std::sqrt(sumSquaredError / evaluation.matchCount);
    evaluation.valid = true;
    populatePoseScoringMetrics(
        settings,
        starDetections,
        detectionIndices,
        projectedStars,
        catalogContext.catalogStars,
        evaluation);
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

// RMS ceiling for a sparse guided-anchor solve, relaxed as more matches accumulate
// (more matches => a high RMS is less likely to be a coincidence). Shared by the
// pre-final evaluation gate and the final-pass ranking gate so the ladder stays in
// one place.
static double sparseGuidedMaxRms(const CameraSettings& settings, int matchCount)
{
    const double radius = static_cast<double>(settings.m_plateSolveFinalMatchRadius);
    if (matchCount >= std::max(settings.m_plateSolveMinMatches + 8, 12)) {
        return std::min(radius * 0.75, 18.0);
    }
    if (matchCount >= std::max(settings.m_plateSolveMinMatches + 4, 8)) {
        return std::min(radius * 0.68, 16.0);
    }
    return std::min(radius * 0.62, 14.5);
}

bool isAcceptableSparseGuidedPairEvaluation(const CameraSettings& settings,
                                            const PlateSolveCatalogContext& catalogContext,
                                            const QVector<CameraPipelineStarDetection>& starDetections,
                                            const Evaluation& evaluation)
{
    const bool isGuidedAnchorPose = evaluation.sparseGuidedPair || evaluation.guidedTriangle;
    if (!evaluation.valid
        || !isGuidedAnchorPose
        || !plateSolveStartUsesDirection(settings)
        || isWidePlateSolveContext(settings)
        || (!isNarrowField(settings))
        || (evaluation.matches.size() < (evaluation.guidedTriangle ? 3 : 2)))
    {
        return false;
    }

    const double maxDirectionDelta = std::max(
        1.0,
        std::min(
            static_cast<double>(settings.m_plateSolveSearchRadius) + static_cast<double>(settings.m_fov) * 0.5,
            static_cast<double>(settings.m_fov) * 2.5));
    if (angularDistanceDegrees(evaluation.azimuthDegrees, settings.m_azimuth) > maxDirectionDelta) {
        return false;
    }
    if (std::fabs(evaluation.elevationDegrees - settings.m_elevation) > maxDirectionDelta) {
        return false;
    }

    const double maxFovDelta = std::max(0.08, static_cast<double>(settings.m_fov) * 0.12);
    if (std::fabs(evaluation.fovDegrees - settings.m_fov) > maxFovDelta) {
        return false;
    }
    if (plateSolveStartUsesRoll(settings)
        && (angularDistanceDegrees(evaluation.rollDegrees, settings.m_roll)
            > std::max(5.0, static_cast<double>(settings.m_fov) * 4.0)))
    {
        return false;
    }

    const double maxPairRms = sparseGuidedMaxRms(settings, evaluation.matchCount);
    if (!std::isfinite(evaluation.rmsErrorPixels) || (evaluation.rmsErrorPixels > maxPairRms)) {
        return false;
    }

    const double maxAnchorDistance = std::min(
        static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.70,
        14.0);
    int strongNamedMatches = 0;
    QSet<int> matchedDetections;
    QSet<int> matchedCatalogStars;
    for (const Match& match : evaluation.matches)
    {
        if ((match.detectionIndex < 0)
            || (match.detectionIndex >= starDetections.size())
            || (match.catalogIndex < 0)
            || (match.catalogIndex >= catalogContext.catalogStars.size())
            || matchedDetections.contains(match.detectionIndex)
            || matchedCatalogStars.contains(match.catalogIndex))
        {
            continue;
        }

        const CatalogStar& catalogStar = catalogContext.catalogStars[match.catalogIndex];
        if ((catalogStar.magnitude > std::min(static_cast<double>(settings.m_plateSolveMaxMagnitude), kNarrowGuidedBrightCatalogMaxMagnitude))
            || !isNamedSparseGuidedCatalogStar(catalogStar)
            || !isStrongSparseGuidedDetection(starDetections[match.detectionIndex])
            || (match.distancePixels > maxAnchorDistance))
        {
            continue;
        }

        matchedDetections.insert(match.detectionIndex);
        matchedCatalogStars.insert(match.catalogIndex);
        ++strongNamedMatches;
    }

    return strongNamedMatches >= (evaluation.guidedTriangle ? 3 : 2);
}

bool isAcceptableSparseGuidedPairFinalPass(const CameraSettings& settings,
                                           const PlateSolveCatalogContext& catalogContext,
                                           const QVector<CameraPipelineStarDetection>& starDetections,
                                           const FinalMatchPassEvaluation& finalPass)
{
    const bool debugSparse = qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE");
    const bool isGuidedAnchorPose = finalPass.pose.sparseGuidedPair || finalPass.pose.guidedTriangle;
    if (!finalPass.projectorValid
        || !isGuidedAnchorPose
        || (finalPass.finalMatches.size() < std::max(4, settings.m_plateSolveMinMatches)))
    {
        if (debugSparse && finalPass.projectorValid && isGuidedAnchorPose)
        {
            qDebug() << "CameraPlateSolver: sparse final rejected before evaluation"
                     << "matches" << finalPass.finalMatches.size()
                     << "min" << std::max(4, settings.m_plateSolveMinMatches)
                     << "Az" << finalPass.pose.azimuthDegrees
                     << "El" << finalPass.pose.elevationDegrees
                     << "Roll" << finalPass.pose.rollDegrees;
        }
        return false;
    }

    const bool highConfidenceSparseAnchors =
        hasHighConfidenceSparseGuidedAnchors(settings, finalPass)
        || hasHighConfidenceGuidedTriangleSupport(settings, finalPass);
    if (hasWeakNarrowGuidedBrightSupport(settings, finalPass)
        && !highConfidenceSparseAnchors)
    {
        if (debugSparse)
        {
            qDebug() << "CameraPlateSolver: sparse final rejected by weak bright support"
                     << "matches" << finalPass.finalMatches.size()
                     << "brightDetections" << finalPass.matchedBrightDetections << "/" << finalPass.brightDetections
                     << "brightProjected" << finalPass.matchedBrightProjectedStars << "/" << finalPass.brightProjectedStars
                     << "seedBright" << finalPass.matchedSeedProjectedBrightStars << "/" << finalPass.seedProjectedBrightStars
                     << "sparseAnchors" << finalPass.sparseGuidedNamedAnchorMatches
                     << "anchorRms" << finalPass.sparseGuidedAnchorRmsErrorPixels
                     << "anchorBrightErr" << finalPass.sparseGuidedAnchorBrightnessRankError
                     << "Az" << finalPass.pose.azimuthDegrees
                     << "El" << finalPass.pose.elevationDegrees
                     << "Roll" << finalPass.pose.rollDegrees;
        }
        return false;
    }

    Evaluation evaluation = finalPass.pose;
    evaluation.matches = finalPass.finalMatches;
    evaluation.matchCount = finalPass.finalMatches.size();
    evaluation.rmsErrorPixels = finalPass.rmsErrorPixels;
    evaluation.brightnessRankError = finalPass.brightnessRankError;
    evaluation.meanCatalogMagnitude = finalPass.meanCatalogMagnitude;
    evaluation.valid = true;
    const bool accepted = isAcceptableSparseGuidedPairEvaluation(
        settings,
        catalogContext,
        starDetections,
        evaluation);
    if (debugSparse && !accepted)
    {
        qDebug() << "CameraPlateSolver: sparse final rejected by evaluation"
                 << "matches" << finalPass.finalMatches.size()
                 << "rms" << finalPass.rmsErrorPixels
                 << "Az" << finalPass.pose.azimuthDegrees
                 << "El" << finalPass.pose.elevationDegrees
                 << "Roll" << finalPass.pose.rollDegrees
                 << "primary" << finalPass.pose.anchorCatalogIndex << finalPass.pose.anchorDetectionIndex
                 << "secondary" << finalPass.pose.secondaryAnchorCatalogIndex << finalPass.pose.secondaryAnchorDetectionIndex;
    }
    return accepted;
}

Evaluation promoteSparseGuidedPairFromMatches(const CameraSettings& settings,
                                              const PlateSolveCatalogContext& catalogContext,
                                              const QVector<CameraPipelineStarDetection>& starDetections,
                                              const Evaluation& candidate)
{
    Evaluation promoted;
    if (!candidate.valid
        || !plateSolveStartUsesDirection(settings)
        || plateSolveStartUsesRoll(settings)
        || isWidePlateSolveContext(settings)
        || (!isNarrowField(settings)))
    {
        return promoted;
    }

    QVector<Match> strongMatches;
    strongMatches.reserve(candidate.matches.size());
    const double maxAnchorDistance = std::min(
        static_cast<double>(settings.m_plateSolveFinalMatchRadius),
        24.0);
    for (const Match& match : candidate.matches)
    {
        if ((match.detectionIndex < 0)
            || (match.detectionIndex >= starDetections.size())
            || (match.catalogIndex < 0)
            || (match.catalogIndex >= catalogContext.catalogStars.size())
            || (match.distancePixels > maxAnchorDistance))
        {
            continue;
        }

        if (!isNamedSparseGuidedCatalogStar(catalogContext.catalogStars[match.catalogIndex])
            || !isStrongSparseGuidedDetection(starDetections[match.detectionIndex]))
        {
            continue;
        }

        strongMatches.append(match);
    }
    if (strongMatches.size() < 2) {
        return promoted;
    }

    std::sort(strongMatches.begin(), strongMatches.end(), [&catalogContext, &starDetections, this](const Match& lhs, const Match& rhs) {
        const auto scoreMatch = [&catalogContext, &starDetections, this](const Match& match) {
            const CatalogStar& catalogStar = catalogContext.catalogStars[match.catalogIndex];
            const CameraPipelineStarDetection& detection = starDetections[match.detectionIndex];
            return match.distancePixels
                + catalogStar.magnitude * 0.75
                - std::min(12.0, narrowGuidedAnchorShapeScore(detection) * 0.15)
                - std::min(8.0, std::log1p(cachedDetectionBrightnessMetric(starDetections, match.detectionIndex)) * 0.7);
        };
        const double lhsScore = scoreMatch(lhs);
        const double rhsScore = scoreMatch(rhs);
        if (!qFuzzyCompare(lhsScore + 1.0, rhsScore + 1.0)) {
            return lhsScore < rhsScore;
        }
        return lhs.distancePixels < rhs.distancePixels;
    });

    QVector<Match> anchors;
    anchors.reserve(2);
    QSet<int> usedDetections;
    QSet<int> usedCatalogStars;
    for (const Match& match : strongMatches)
    {
        if (usedDetections.contains(match.detectionIndex)
            || usedCatalogStars.contains(match.catalogIndex))
        {
            continue;
        }
        anchors.append(match);
        usedDetections.insert(match.detectionIndex);
        usedCatalogStars.insert(match.catalogIndex);
        if (anchors.size() >= 2) {
            break;
        }
    }
    if (anchors.size() < 2) {
        return promoted;
    }

    double sumSquaredError = 0.0;
    for (const Match& match : anchors) {
        sumSquaredError += match.distancePixels * match.distancePixels;
    }

    promoted = candidate;
    promoted.matches = anchors;
    promoted.matchCount = anchors.size();
    promoted.rmsErrorPixels = std::sqrt(sumSquaredError / static_cast<double>(anchors.size()));
    promoted.sparseGuidedPair = true;
    promoted.anchored = true;
    promoted.anchorDetectionIndex = anchors[0].detectionIndex;
    promoted.anchorCatalogIndex = anchors[0].catalogIndex;
    promoted.secondaryAnchorDetectionIndex = anchors[1].detectionIndex;
    promoted.secondaryAnchorCatalogIndex = anchors[1].catalogIndex;
    if (!isAcceptableSparseGuidedPairEvaluation(
            settings,
            catalogContext,
            starDetections,
            promoted))
    {
        promoted = Evaluation();
    }

    return promoted;
}

// Strong, largely roll-independent final-pass evidence that justifies ignoring a weak
// seed-radial signal: many final matches, solid matched bright-detection and
// bright-projected support, good magnitude support, low brightness-rank error and a
// tight RMS. This stops the no-roll seed-radial gate from vetoing a pose that is
// clearly correct on its own merits -- e.g. a free-roll narrow solve whose roll is
// far from the (default-0) seed roll, which inflates the seed-projected radial error
// even though the solve is right (galaxy-m101 @maxMag16: 300 matches, 11/12 bright
// projected, both named anchors within 9 px). The bright-magnitude-error ceiling is
// 1.30 rather than 1.0 because narrow-field frames routinely include a saturated
// bright star whose measured magnitude is unreliable; the strong match-count and
// bright-support requirements still exclude the genuinely weak poses (e.g.
// stars-narrow-1/3, which match only ~3 bright detections).
static bool hasDenseFinalEvidenceOverridingSeedRadial(const CameraSettings& settings,
                                                      const FinalMatchPassEvaluation& finalPass)
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

// ---------------------------------------------------------------------------------
// Experimental robust verifier (SHADOW MODE -- computed and logged for corpus
// comparison, not yet wired into the accept/reject decision).
//
// Replaces the hand-tuned bright-support heuristics with a single statistical test:
// the log-odds that the matched configuration is a true alignment (H1) rather than a
// chance coincidence (H0), summed over matches as a foreground/background likelihood
// ratio (cf. astrometry.net's verification step; Sutherland & Saunders 1992):
//
//   log LR_i = -d_i^2 / (2 sigma^2)        // H1: real star, Gaussian centroid scatter
//              - log(2 pi sigma^2)         // H1 normalisation (per px^2)
//              - log( rho(<= m_i) )        // H0: uniform catalog background (per px^2)
//
// rho(<= m) = (# catalog stars projected into the field at least as bright as m) /
// image area. A match to a *rare* (bright) star therefore contributes far more
// evidence than a match to a common faint star -- the bright-star weighting that the
// heuristics hand-code falls out of the density automatically, with no per-channel
// rules. Loose matches (d >> sigma) contribute little or negative evidence, so a
// wrong pose riding on many loose faint coincidences scores low.
//
// sigma is tied to the geometric match radius (r/4) rather than a tuned setting: a
// real match is expected well inside the match disk. The only free knob this exposes
// to an eventual accept decision is a single log-odds threshold.
static double poseFalseAlarmLogOdds(const PlateSolveCatalogContext& catalogContext,
                                    const FinalMatchPassEvaluation& finalPass,
                                    const QSize& imageSize,
                                    double matchRadiusPixels,
                                    int detectionCount)
{
    if (!finalPass.projectorValid
        || finalPass.finalMatches.isEmpty()
        || (imageSize.width() <= 0)
        || (imageSize.height() <= 0)
        || (matchRadiusPixels <= 0.0)
        || finalPass.projectedStars.isEmpty())
    {
        return 0.0;
    }

    const double area = static_cast<double>(imageSize.width())
        * static_cast<double>(imageSize.height());
    const double matchDiskArea = kPi * matchRadiusPixels * matchRadiusPixels;
    const int projectedCount = static_cast<int>(finalPass.projectedStars.size());
    // Tightness scale for the bright bonus. Real matches here sit well inside the match
    // radius but not sub-pixel (pose-model error), so r/2 gives a gentle discount to
    // loose matches rather than killing them.
    const double sigma = std::max(1.0, matchRadiusPixels * 0.5);
    const double inverseTwoSigmaSquared = 1.0 / (2.0 * sigma * sigma);

    // Sorted projected-catalog magnitudes for a fast rho(<= m) lookup.
    QVector<double> projectedMagnitudes;
    projectedMagnitudes.reserve(finalPass.projectedStars.size());
    for (const ProjectedCatalogStar& star : finalPass.projectedStars) {
        projectedMagnitudes.append(star.magnitude);
    }
    std::sort(projectedMagnitudes.begin(), projectedMagnitudes.end());

    // Magnitude-limited count-surprise (Poisson log-likelihood ratio). Under H0 (chance)
    // the number of *bright* detection<->catalog coincidences within the match radius is
    // ~Poisson(lambda_bright). Observing K_bright >> lambda_bright is strong evidence of
    // a real alignment. Restricting the count to bright stars is deliberate: in a deep
    // field (mag 15-19) a *wrong* roll still matches hundreds of faint stars, so an
    // all-magnitude count-surprise rewards wrong rolls too; limiting it to the bright
    // tier means only a roll that lands the genuinely-bright stars earns count-surprise,
    // which is what separates the correct roll from a faint-coincidence one. A correct
    // dense solve still wins comfortably because it matches far more bright stars than a
    // wrong roll (so m51-2 stays safe). Fields with no bright tier fall back to the
    // rarity bonus below.
    constexpr double kCountSurpriseBrightLimit = 13.0;
    int brightMatches = 0;
    for (const Match& match : finalPass.finalMatches) {
        if ((match.catalogIndex >= 0)
            && (match.catalogIndex < catalogContext.catalogStars.size())
            && (catalogContext.catalogStars[match.catalogIndex].magnitude <= kCountSurpriseBrightLimit)) {
            ++brightMatches;
        }
    }
    const int brightProjectedCount = static_cast<int>(
        std::upper_bound(projectedMagnitudes.cbegin(), projectedMagnitudes.cend(), kCountSurpriseBrightLimit)
        - projectedMagnitudes.cbegin());
    const double safeDetections = static_cast<double>(std::max(detectionCount, 1));
    const double brightDensity = static_cast<double>(brightProjectedCount) / area;
    const double lambda = std::min(
        std::min(safeDetections, static_cast<double>(std::max(1, brightProjectedCount))),
        safeDetections * brightDensity * matchDiskArea);
    double logOdds = 0.0;
    if ((static_cast<double>(brightMatches) > lambda) && (lambda > 0.0))
    {
        logOdds += static_cast<double>(brightMatches)
                * std::log(static_cast<double>(brightMatches) / lambda)
            - (static_cast<double>(brightMatches) - lambda);
    }

    // Bright-rarity bonus: a tight match to a star rarer than the field average (a
    // bright star) is far more surprising than one to a common faint star. The rarity
    // is log(projectedCount / #stars-at-least-this-bright); it is weighted by tightness
    // so loose coincidences add little. This is what separates the correct roll (which
    // lands the rare bright stars on detections) from a faint-coincidence roll.
    for (const Match& match : finalPass.finalMatches)
    {
        if ((match.catalogIndex < 0) || (match.catalogIndex >= catalogContext.catalogStars.size())) {
            continue;
        }
        const double magnitude = catalogContext.catalogStars[match.catalogIndex].magnitude;
        const int brighterOrEqual = static_cast<int>(
            std::upper_bound(projectedMagnitudes.cbegin(), projectedMagnitudes.cend(), magnitude)
            - projectedMagnitudes.cbegin());
        const double rarity = std::log(
            static_cast<double>(projectedCount) / static_cast<double>(std::max(1, brighterOrEqual)));
        if (rarity <= 0.0) {
            continue;
        }
        const double d = match.distancePixels;
        const double tightness = std::exp(-d * d * inverseTwoSigmaSquared);
        logOdds += rarity * tightness;
    }
    return logOdds;
}

// A narrow guided-direction final pass with many matches at sub-pixel RMS is a true
// alignment by overwhelming statistical evidence (cf. poseFalseAlarmLogOdds): scores of
// catalog stars cannot land sub-pixel on detections by chance. Such a pose is genuinely
// solved even when the field carries NO bright/saturated anchors — faint Milky-Way or
// high-galactic-latitude fields legitimately lack them, and the bright-anchor support
// gates then wrongly reject an otherwise perfect solve (surfaced by the synthetic random
// corpus: 9/38 zero-jitter failures were correct fits at rms 0.15-0.30 with 55-89 matches).
// Used to bypass the *brightness* acceptance gates only; the FoV and direction-seed
// quality gates still apply.
bool hasOverwhelmingFaintGuidedSupport(const CameraSettings& settings,
                                       const FinalMatchPassEvaluation& finalPass)
{
    if (!finalPass.projectorValid || (!isNarrowField(settings))) {
        return false;
    }
    const int matchCount = finalPass.finalMatches.size();
    const double rms = finalPass.rmsErrorPixels;
    const double maxErr = finalPass.maxErrorPixels;
    const int strongFloor = std::max(settings.m_plateSolveMinMatches + 20, 30);
    const double tightRms = std::min(1.5, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.12);
    const double maxErrCap = std::max(6.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.5);
    return (matchCount >= strongFloor)
        && std::isfinite(rms) && (rms <= tightRms)
        && std::isfinite(maxErr) && (maxErr <= maxErrCap);
}

bool hasWeakNarrowGuidedBrightSupport(const CameraSettings& settings,
                                      const FinalMatchPassEvaluation& finalPass)
{
    if (!m_useDirectionSeedPreference
        || (!isNarrowField(settings))
        || !finalPass.projectorValid)
    {
        return false;
    }

    const bool useSeedProjectedBrightGate = usesSeedProjectedBrightGate(settings);
    const bool debugSparse = qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE");

    if (hasHighConfidenceGuidedTriangleSupport(settings, finalPass)
        || hasStrongDenseNarrowGuidedFinalPass(settings, finalPass))
    {
        return false;
    }

    const bool denseFinalEvidenceOverridesSeedRadial =
        hasDenseFinalEvidenceOverridingSeedRadial(settings, finalPass);

    // Computed once and shared by both the low-magnitude branch below and the
    // general bright-support checks afterwards (previously duplicated verbatim in
    // both scopes with identical inputs).
    const bool poorNoRollSeedRadialSupport =
        hasPoorNoRollSeedRadialSupport(settings, finalPass, useSeedProjectedBrightGate);
    const bool projectedBrightSupportCanOverrideDetectedBright =
        !useSeedProjectedBrightGate
        && !poorNoRollSeedRadialSupport
        && (finalPass.matchedBrightProjectedStars >= 5)
        && (finalPass.projectedMagnitudeMatchFraction >= 0.30)
        && (finalPass.seedProjectedMagnitudeSupport < 80.0);

    if (isLowMagnitudeNarrowGuidedSolve(settings))
    {
        const bool projectedMagnitudeSupportGood =
            (finalPass.matchedProjectedMagnitudeSupport >= 12.0)
            && (finalPass.projectedMagnitudeMatchFraction >= 0.35);
        if (debugSparse)
        {
            qDebug() << "CameraPlateSolver: narrow guided bright support"
                     << finalPassBrightDiagnosticSummary(finalPass)
                     << "Az" << finalPass.pose.azimuthDegrees
                     << "El" << finalPass.pose.elevationDegrees
                     << "Roll" << finalPass.pose.rollDegrees
                     << "FoV" << finalPass.pose.fovDegrees;
        }
        if ((finalPass.brightCatalogShapeChecks >= 1)
            && (finalPass.brightCatalogShapeMismatches > 0))
        {
            return true;
        }
        if (poorNoRollSeedRadialSupport && !denseFinalEvidenceOverridesSeedRadial) {
            return true;
        }
        if (useSeedProjectedBrightGate
            && (finalPass.seedProjectedBrightStars >= 2)
            && (finalPass.matchedSeedProjectedBrightStars < std::min(2, finalPass.seedProjectedBrightStars)))
        {
            return true;
        }
        if (!useSeedProjectedBrightGate
            && (finalPass.seedProjectedBrightStars >= 4)
            && (finalPass.matchedSeedProjectedBrightStars == 0)
            && (finalPass.seedProjectedMagnitudeSupport >= 80.0)
            && (finalPass.seedProjectedMagnitudeMatchFraction < 0.08))
        {
            return true;
        }
        if ((finalPass.brightProjectedStars >= 4)
            && (finalPass.matchedBrightProjectedStars < 2))
        {
            return true;
        }
        if ((finalPass.brightProjectedStars >= 10)
            && (finalPass.matchedBrightProjectedStars < 5)
            && !projectedMagnitudeSupportGood)
        {
            return true;
        }
        if ((finalPass.brightProjectedStars >= 6)
            && (finalPass.matchedBrightProjectedStars < 3))
        {
            return true;
        }
        if ((finalPass.brightDetections >= 12)
            && (finalPass.matchedBrightDetections < 3)
            && !projectedBrightSupportCanOverrideDetectedBright)
        {
            return true;
        }
        if (useSeedProjectedBrightGate
            && (finalPass.seedProjectedBrightStars >= 1)
            && (finalPass.matchedSeedProjectedBrightStars == 0))
        {
            return true;
        }
        if ((finalPass.brightDetections >= 6)
            && (finalPass.brightDetectionMagnitudeError > 2.25))
        {
            return true;
        }
        if (!useSeedProjectedBrightGate
            && (finalPass.seedRadialMagnitudeSupport >= 60.0)
            && (finalPass.seedRadialMagnitudeMatchFraction < 0.04)
            && (finalPass.prioritySeedRadialChecks >= 12)
            && !denseFinalEvidenceOverridesSeedRadial
            && std::isfinite(finalPass.prioritySeedRadialErrorPixels)
            && (finalPass.prioritySeedRadialErrorPixels > std::max(
                    300.0,
                    static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 8.0)))
        {
            return true;
        }
    }

    if (useSeedProjectedBrightGate
        && (finalPass.seedProjectedBrightStars >= 6)
        && (finalPass.matchedSeedProjectedBrightStars == 0))
    {
        const int minimumBrightDetectionMatches = std::min(
            8,
            std::max(3, finalPass.brightDetections / 3));
        if (finalPass.matchedBrightDetections < minimumBrightDetectionMatches) {
            return true;
        }
    }
    if (useSeedProjectedBrightGate
        && (finalPass.seedProjectedBrightStars >= 10)
        && (finalPass.matchedSeedProjectedBrightStars <= 1))
    {
        const int minimumBrightDetectionMatches = std::min(
            8,
            std::max(4, finalPass.brightDetections / 3));
        if (finalPass.matchedBrightDetections < minimumBrightDetectionMatches) {
            return true;
        }
    }

    return ((finalPass.brightDetections >= 6)
            && (finalPass.matchedBrightDetections < 2)
            && !projectedBrightSupportCanOverrideDetectedBright)
        || (poorNoRollSeedRadialSupport && !denseFinalEvidenceOverridesSeedRadial)
        || ((finalPass.brightProjectedStars >= 5) && (finalPass.matchedBrightProjectedStars < 2))
        || ((finalPass.brightDetections >= 6) && (finalPass.brightDetectionMagnitudeError > 2.35));
}

bool isAcceptableSparseGuidedRankingFinalPass(const CameraSettings& settings,
                                              const FinalMatchPassEvaluation& finalPass)
{
    if (!finalPass.projectorValid
        || !finalPass.pose.sparseGuidedPair
        || !plateSolveStartUsesDirection(settings)
        || isWidePlateSolveContext(settings)
        || (!isNarrowField(settings))
        || (finalPass.finalMatches.size() < std::max(4, settings.m_plateSolveMinMatches))
        || (finalPass.matchedBrightProjectedStars < 2))
    {
        return false;
    }

    if (hasWeakNarrowGuidedBrightSupport(settings, finalPass)) {
        return false;
    }

    const double maxRmsError = sparseGuidedMaxRms(settings, static_cast<int>(finalPass.finalMatches.size()));

    return std::isfinite(finalPass.rmsErrorPixels)
        && (finalPass.rmsErrorPixels <= maxRmsError);
}

bool hasStrongDenseNarrowGuidedFinalPass(const CameraSettings& settings,
                                         const FinalMatchPassEvaluation& finalPass)
{
    if (!m_useDirectionSeedPreference
        || (!isNarrowField(settings))
        || !finalPass.projectorValid
        || (finalPass.finalMatches.size() < static_cast<qsizetype>(std::max(settings.m_plateSolveMinMatches + 18, 24)))
        || !std::isfinite(finalPass.rmsErrorPixels)
        || (finalPass.rmsErrorPixels > maxDirectionSeedRmsError(settings, finalPass.finalMatches.size())))
    {
        return false;
    }

    const bool veryDenseMatchSet =
        finalPass.finalMatches.size() >= static_cast<qsizetype>(std::max(settings.m_plateSolveMinMatches + 76, 96));
    const int projectedBrightThreshold = std::min(
        finalPass.brightProjectedStars,
        std::max(veryDenseMatchSet ? 3 : 5,
            static_cast<int>(std::ceil(finalPass.brightProjectedStars * (veryDenseMatchSet ? 0.45 : 0.70)))));
    const int detectedBrightThreshold = std::min(
        finalPass.brightDetections,
        std::max(veryDenseMatchSet ? 3 : 6,
            static_cast<int>(std::ceil(finalPass.brightDetections * (veryDenseMatchSet ? 0.20 : 0.33)))));
    const double projectedMagnitudeThreshold = veryDenseMatchSet ? 0.18 : 0.70;
    const double seedProjectedMagnitudeThreshold = veryDenseMatchSet ? 0.18 : 0.60;
    return (finalPass.brightProjectedStars >= 6)
        && (finalPass.matchedBrightProjectedStars >= projectedBrightThreshold)
        && (finalPass.matchedBrightDetections >= detectedBrightThreshold)
        && (finalPass.projectedMagnitudeMatchFraction >= projectedMagnitudeThreshold)
        && ((finalPass.seedProjectedMagnitudeSupport <= 0.0)
            || (finalPass.seedProjectedMagnitudeMatchFraction >= seedProjectedMagnitudeThreshold)
            || veryDenseMatchSet)
        && ((finalPass.brightDetectionMagnitudeError <= 1.60)
            || (finalPass.matchedBrightDetections >= std::min(finalPass.brightDetections, 12)));
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

// Residual acceptance thresholds shared by the seed-mode "is this solve good
// enough" checks. Every mode applies the same shape -- a minimum match count, then
// an RMS / median / worst-case pixel-error ceiling -- and differs only in the
// threshold values, so the shape lives in passesResidualGates() and each mode just
// fills in its own numbers.
struct ResidualGates
{
    int minMatches = 0;
    double maxRms = std::numeric_limits<double>::infinity();
    double maxMedian = std::numeric_limits<double>::infinity();
    double maxWorst = std::numeric_limits<double>::infinity();
};

static bool passesResidualGates(const QVector<Match>& matches,
                                double rmsErrorPixels,
                                double maxErrorPixels,
                                const ResidualGates& gates)
{
    if (matches.size() < gates.minMatches) {
        return false;
    }
    return (rmsErrorPixels <= gates.maxRms)
        && (medianDistancePixels(matches) <= gates.maxMedian)
        && (maxErrorPixels <= gates.maxWorst);
}

static bool isAcceptableBlindSolve(const CameraSettings& settings,
                            const QVector<CameraPipelineStarDetection>& starDetections,
                            const QVector<Match>& matches,
                            double rmsErrorPixels,
                            double maxErrorPixels)
{
    ResidualGates gates;
    gates.minMatches = std::max(settings.m_plateSolveMinMatches + 2,
        std::min(10, std::max(6, static_cast<int>(std::ceil(static_cast<double>(starDetections.size()) * 0.20)))));
    gates.maxRms = std::min(settings.m_plateSolveFinalMatchRadius * 0.70, 20.0);
    gates.maxMedian = std::min(settings.m_plateSolveFinalMatchRadius * 0.55, 15.0);
    gates.maxWorst = std::min(settings.m_plateSolveFinalMatchRadius * 1.10, 45.0);
    return passesResidualGates(matches, rmsErrorPixels, maxErrorPixels, gates);
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
    if (isNarrowField(settings)) {
        return configuredMinimum;
    }
    if (settings.m_fov <= 15.0)
    {
        return std::max(configuredMinimum,
            std::min(6, std::max(4, static_cast<int>(std::ceil(static_cast<double>(starDetections.size()) * 0.10)))));
    }
    if (starDetections.size() <= 12) {
        return configuredMinimum;
    }

    return std::max(settings.m_plateSolveMinMatches + 1,
        std::min(8, std::max(5, static_cast<int>(std::ceil(static_cast<double>(starDetections.size()) * 0.15)))));
}

static double maxDirectionSeedRmsError(const CameraSettings& settings, int matchCount)
{
    const bool narrowField = isNarrowField(settings);
    const bool denseNarrowFieldSolve = narrowField
        && (matchCount >= (isLowMagnitudeNarrowGuidedSolve(settings) ? 16 : 20));
    if (narrowField)
    {
        return denseNarrowFieldSolve
            ? std::min(settings.m_plateSolveFinalMatchRadius * 0.70, 17.0)
            : std::min(settings.m_plateSolveFinalMatchRadius * 0.62, 14.5);
    }

    return std::min(settings.m_plateSolveFinalMatchRadius * 0.70, 18.0);
}

static double maxNarrowGuidedFovDeltaDegrees(const CameraSettings& settings)
{
    return std::max(0.08, static_cast<double>(settings.m_fov) * 0.08);
}

static bool isAcceptableNarrowGuidedFov(const CameraSettings& settings, double fovDegrees)
{
    if (!plateSolveStartUsesDirection(settings)
        || !plateSolveStartUsesFov(settings)
        || (!isNarrowField(settings)))
    {
        return true;
    }

    return std::isfinite(fovDegrees)
        && (std::fabs(fovDegrees - settings.m_fov) <= maxNarrowGuidedFovDeltaDegrees(settings));
}

static QString narrowGuidedFovRejectionReason(const CameraSettings& settings, double fovDegrees)
{
    return QStringLiteral("FoV %1 outside start FoV %2 +/- %3")
        .arg(fovDegrees, 0, 'f', 3)
        .arg(static_cast<double>(settings.m_fov), 0, 'f', 3)
        .arg(maxNarrowGuidedFovDeltaDegrees(settings), 0, 'f', 3);
}

// Residual gates for a direction-seeded solve. Shared by isAcceptableDirectionSeedSolve()
// and directionSeedRejectionReason() so the accept decision and the human-readable
// reason can never drift apart.
//
// Narrow-field (telescope) solves pin the FoV, so residuals reflect real centroid noise
// -> looser 0.75x median gate. Wide-field fisheye distortion at large angles raises
// residuals even for correct solves, so the median gate is 0.65x (= 15.6 px at a 24 px
// radius) to accept those while still rejecting clearly wrong ones.
static ResidualGates directionSeedResidualGates(const CameraSettings& settings,
                                                const QVector<CameraPipelineStarDetection>& starDetections,
                                                int matchCount)
{
    const bool narrowField = isNarrowField(settings);
    ResidualGates gates;
    gates.minMatches = minimumDirectionSeedAcceptedMatches(settings, starDetections);
    gates.maxRms = maxDirectionSeedRmsError(settings, matchCount);
    gates.maxMedian = narrowField
        ? std::min(settings.m_plateSolveFinalMatchRadius * 0.75, 18.0)
        : std::min(settings.m_plateSolveFinalMatchRadius * 0.65, 18.0);
    gates.maxWorst = std::min(
        settings.m_plateSolveFinalMatchRadius * (isWidePlateSolveContext(settings) ? 1.15 : 1.05),
        36.0);
    return gates;
}

static QString directionSeedRejectionReason(const CameraSettings& settings,
                                            const QVector<CameraPipelineStarDetection>& starDetections,
                                            const QVector<Match>& matches,
                                            double rmsErrorPixels,
                                            double maxErrorPixels)
{
    const ResidualGates gates = directionSeedResidualGates(settings, starDetections, matches.size());
    const double medianError = medianDistancePixels(matches);

    QStringList reasons;
    if (matches.size() < gates.minMatches) {
        reasons.append(QStringLiteral("matches %1 < required %2").arg(matches.size()).arg(gates.minMatches));
    }
    if (rmsErrorPixels > gates.maxRms) {
        reasons.append(QStringLiteral("RMS %1 > %2").arg(rmsErrorPixels, 0, 'f', 2).arg(gates.maxRms, 0, 'f', 2));
    }
    if (medianError > gates.maxMedian) {
        reasons.append(QStringLiteral("median %1 > %2").arg(medianError, 0, 'f', 2).arg(gates.maxMedian, 0, 'f', 2));
    }
    if (maxErrorPixels > gates.maxWorst) {
        reasons.append(QStringLiteral("max %1 > %2").arg(maxErrorPixels, 0, 'f', 2).arg(gates.maxWorst, 0, 'f', 2));
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
    if (!isAcceptableNarrowGuidedFov(settings, evaluation.fovDegrees)) {
        return false;
    }

    const int minAcceptedMatches = std::max(minMatchCount, 4);
    const double maxRmsError = maxDirectionSeedRmsError(settings, evaluation.matchCount);
    return (evaluation.matchCount >= minAcceptedMatches) && (evaluation.rmsErrorPixels <= maxRmsError);
}

static bool isAcceptableDirectionSeedSolve(const CameraSettings& settings,
                                    const QVector<CameraPipelineStarDetection>& starDetections,
                                    const QVector<Match>& matches,
                                    double rmsErrorPixels,
                                    double maxErrorPixels)
{
    const ResidualGates gates = directionSeedResidualGates(settings, starDetections, matches.size());
    return passesResidualGates(matches, rmsErrorPixels, maxErrorPixels, gates);
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
    ResidualGates gates;
    gates.minMatches = std::max(settings.m_plateSolveMinMatches + 1,
        std::min(8, std::max(5, static_cast<int>(std::ceil(static_cast<double>(starDetections.size()) * 0.15)))));
    gates.maxRms = std::min(settings.m_plateSolveFinalMatchRadius * 0.85, 24.0);
    gates.maxMedian = std::min(settings.m_plateSolveFinalMatchRadius * 0.70, 18.0);
    gates.maxWorst = std::min(settings.m_plateSolveFinalMatchRadius * 1.20, 50.0);
    return passesResidualGates(matches, rmsErrorPixels, maxErrorPixels, gates);
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

    if (isNarrowField(settings)) {
        if ((evaluation.brightCatalogShapeChecks >= 2)
            && (evaluation.brightCatalogShapeMismatches > 0))
        {
            return false;
        }

        const bool lowMagnitudeNarrowGuided = isLowMagnitudeNarrowGuidedSolve(settings);
        const bool useSeedProjectedBrightGate = usesSeedProjectedBrightGate(settings);
        const bool poorNoRollSeedRadialSupport =
            hasPoorNoRollSeedRadialSupport(settings, evaluation, useSeedProjectedBrightGate);
        const bool projectedBrightSupportCanOverrideDetectedBright =
            !useSeedProjectedBrightGate
            && !poorNoRollSeedRadialSupport
            && (evaluation.matchedBrightProjectedStars >= 5)
            && (evaluation.projectedMagnitudeMatchFraction >= 0.30)
            && (evaluation.seedProjectedMagnitudeSupport < 80.0);
        if (lowMagnitudeNarrowGuided
            && poorNoRollSeedRadialSupport
            && !hasDenseFinalEvidenceOverridingSeedRadial(settings, evaluation)) {
            return false;
        }
        if (useSeedProjectedBrightGate
            && lowMagnitudeNarrowGuided
            && (evaluation.seedProjectedBrightStars > 0)
            && (evaluation.matchedSeedProjectedBrightStars == 0))
        {
            const bool strongProjectedBrightSupport =
                (evaluation.brightProjectedStars >= 4)
                && (evaluation.brightProjectedMatchFraction >= 0.85)
                && ((evaluation.brightDetections < 6)
                    || (evaluation.brightDetectionMatchFraction >= 0.65))
                && ((evaluation.brightDetectionMagnitudeError <= 1.50)
                    || (evaluation.matchedBrightProjectedStars >= 6));
            const bool strongDetectedBrightSupport =
                (evaluation.brightDetections >= 8)
                && (evaluation.brightDetectionMatchFraction >= 0.65)
                && (evaluation.brightDetectionMagnitudeError <= 0.45);
            if (!strongProjectedBrightSupport && !strongDetectedBrightSupport) {
                return false;
            }
        }

        if ((evaluation.finalMatches.size() >= 20)
            && (evaluation.brightnessRankError <= 0.35))
        {
            return true;
        }

        if (lowMagnitudeNarrowGuided
            && (evaluation.finalMatches.size() >= 16)
            && (evaluation.rmsErrorPixels > std::min(settings.m_plateSolveFinalMatchRadius * 0.62, 14.5))
            && (evaluation.brightProjectedStars >= 3)
            && (evaluation.matchedBrightProjectedStars >= 2)
            && (evaluation.brightDetectionMagnitudeError <= 1.45)
            && (evaluation.brightCatalogShapeMismatches == 0)
            && (!std::isfinite(evaluation.brightnessRankError) || (evaluation.brightnessRankError <= 0.70)))
        {
            return true;
        }
        if ((evaluation.brightProjectedStars >= 2)
            && (evaluation.matchedBrightProjectedStars == 0))
        {
            return false;
        }
        if ((evaluation.brightProjectedStars >= 5)
            && (evaluation.matchedBrightProjectedStars < 2))
        {
            return false;
        }
        if (lowMagnitudeNarrowGuided
            && (evaluation.brightDetections >= 12)
            && (evaluation.matchedBrightDetections < 3)
            && !projectedBrightSupportCanOverrideDetectedBright)
        {
            return false;
        }

        // For narrow-field (telescope) solves the FOV is pinned to the user's value,
        // which strongly constrains the geometry.  Brightness rank ordering is also less
        // reliable when the matched set spans mag 2-13 (saturated bright star + very faint
        // stars).  When at least two bright projected stars are retained, give that
        // geometric bright-star support a little more weight than the global rank order.
        const double threshold =
            ((evaluation.matchedBrightProjectedStars >= 2)
                && (evaluation.brightDetectionMagnitudeError <= 1.60))
            ? 0.70
            : 0.65;
        return evaluation.brightnessRankError <= threshold;
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

bool isStrongEarlyGuidedFinalPass(const CameraSettings& settings,
                                  const QVector<CameraPipelineStarDetection>& starDetections,
                                  const FinalMatchPassEvaluation& evaluation,
                                  int requiredMatches,
                                  double finalMatchRadius)
{
    if (!m_useDirectionSeedPreference || !evaluation.projectorValid) {
        return false;
    }

    // A roll prior can make an early final pass look convincing while still
    // sitting in the wrong local basin. Let narrow roll-constrained solves run
    // the multi-hypothesis comparison before accepting a solution.
    if ((isNarrowField(settings)) && plateSolveStartUsesRoll(settings)) {
        return false;
    }
    if (!isAcceptableNarrowGuidedFov(settings, evaluation.pose.fovDegrees)) {
        return false;
    }

    const int minimumEarlyMatches = (isNarrowField(settings))
        ? std::max(requiredMatches + 12, 16)
        : std::max(requiredMatches + 6, 10);
    if (evaluation.finalMatches.size() < minimumEarlyMatches) {
        return false;
    }

    const bool denseNarrowHighSupport =
        (isNarrowField(settings))
        && (evaluation.finalMatches.size() >= std::max(requiredMatches + 48, 64))
        && (evaluation.rmsErrorPixels <= std::min(maxDirectionSeedRmsError(settings, evaluation.finalMatches.size()) * 0.95, finalMatchRadius * 0.70))
        && (evaluation.maxErrorPixels <= std::min(finalMatchRadius * 1.05, 30.0));
    if (denseNarrowHighSupport
        && isAcceptableDirectionSeedSolve(
            settings,
            starDetections,
            evaluation.finalMatches,
            evaluation.rmsErrorPixels,
            evaluation.maxErrorPixels)
        && hasGeometricallyConsistentMatches(
            starDetections,
            evaluation.projectedStars,
            evaluation.finalMatches,
            finalMatchRadius))
    {
        return true;
    }

    if (!hasAcceptableWideBrightAnchorSupport(settings, starDetections, evaluation)
        || hasWeakNarrowGuidedBrightSupport(settings, evaluation)
        || !hasAcceptableGuidedFinalBrightnessConsistency(settings, evaluation))
    {
        return false;
    }

    if (!isAcceptableDirectionSeedSolve(
            settings,
            starDetections,
            evaluation.finalMatches,
            evaluation.rmsErrorPixels,
            evaluation.maxErrorPixels))
    {
        return false;
    }

    return hasGeometricallyConsistentMatches(
        starDetections,
        evaluation.projectedStars,
        evaluation.finalMatches,
        finalMatchRadius);
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

double directionSeedAngularDistanceDegrees(const Evaluation& evaluation) const
{
    if (!m_useDirectionSeedPreference) {
        return 0.0;
    }

    const double azimuthDelta = angularDistanceDegrees(
        evaluation.azimuthDegrees,
        m_directionSeedReferenceAzimuthDegrees);
    const double elevationDelta = std::fabs(
        evaluation.elevationDegrees - m_directionSeedReferenceElevationDegrees);
    return std::sqrt(azimuthDelta * azimuthDelta + elevationDelta * elevationDelta);
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

// Aggregate "how much non-zero lens calibration this pose carries" — smaller is
// preferred as a tie-break so the solver does not invent center-offset/distortion
// it does not need. The 100x weight puts distortion k1 (a small dimensionless
// value) on a comparable footing with the pixel offsets.
static double calibrationMagnitude(const Evaluation& evaluation)
{
    return std::fabs(evaluation.centerOffsetXPixels)
        + std::fabs(evaluation.centerOffsetYPixels)
        + 100.0 * std::fabs(evaluation.distortionK1);
}

// Deterministic geometric tie-break shared by the evaluation comparators: prefer
// more matches, then lower RMS, then less lens calibration, then the narrower FoV
// (or smaller roll at equal FoV). Callers handle validity and any higher-priority
// scoring before delegating here. Assumes both evaluations are valid.
static bool isBetterByGeometricTieBreak(const Evaluation& candidate, const Evaluation& best)
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

static bool isBetterEvaluation(const Evaluation& candidate, const Evaluation& best)
{
    if (!candidate.valid) {
        return false;
    }
    if (!best.valid) {
        return true;
    }
    return isBetterByGeometricTieBreak(candidate, best);
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

    return isBetterByGeometricTieBreak(candidate, best);
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
    QVector<Match> forcedAnchorMatches;
    bool rejectAnchoredCandidate = false;
    if (candidate.anchored)
    {
        QSet<int> matchedDetections;
        QSet<int> matchedCatalogStars;
        const auto appendForcedAnchor = [&](int detectionIndex, int catalogIndex) {
            if ((detectionIndex < 0)
                || (detectionIndex >= starDetections.size())
                || (catalogIndex < 0)
                || (catalogIndex >= catalogContext.catalogStars.size())
                || matchedDetections.contains(detectionIndex)
                || matchedCatalogStars.contains(catalogIndex))
            {
                return false;
            }

            int projectedIndex = -1;
            for (int i = 0; i < finalPass.projectedStars.size(); ++i)
            {
                if (finalPass.projectedStars[i].catalogIndex == catalogIndex)
                {
                    projectedIndex = i;
                    break;
                }
            }
            if (projectedIndex < 0) {
                return false;
            }

            const double anchorDistance = pointDistancePixels(
                starDetections[detectionIndex].m_center,
                finalPass.projectedStars[projectedIndex].point);
            const bool anchorHasImplausibleBrightShape =
                isImplausiblyCompactBrightCatalogDetection(
                    starDetections[detectionIndex],
                    catalogContext.catalogStars[catalogIndex].magnitude,
                    m_useDirectionSeedPreference && (isNarrowField(settings)));
            if ((anchorDistance > finalMatchRadius) || anchorHasImplausibleBrightShape) {
                return false;
            }

            allMatches.append({detectionIndex, catalogIndex, anchorDistance});
            forcedAnchorMatches.append({detectionIndex, catalogIndex, anchorDistance});
            matchedDetections.insert(detectionIndex);
            matchedCatalogStars.insert(catalogIndex);
            return true;
        };

        bool forcedAnchorsValid = appendForcedAnchor(candidate.anchorDetectionIndex, candidate.anchorCatalogIndex);
        if (candidate.sparseGuidedPair || candidate.guidedTriangle) {
            forcedAnchorsValid = forcedAnchorsValid
                && appendForcedAnchor(candidate.secondaryAnchorDetectionIndex, candidate.secondaryAnchorCatalogIndex);
        }
        if (candidate.guidedTriangle) {
            forcedAnchorsValid = forcedAnchorsValid
                && appendForcedAnchor(candidate.tertiaryAnchorDetectionIndex, candidate.tertiaryAnchorCatalogIndex);
        }

        if (forcedAnchorsValid)
        {
            const QVector<Match> automaticMatches = buildMatches(
                catalogContext,
                starDetections,
                detectionIndices,
                finalPass.projectedStars,
                finalMatchRadius);
            allMatches.reserve(automaticMatches.size() + allMatches.size());
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
        else
        {
            allMatches.clear();
            rejectAnchoredCandidate = true;
        }
    }

    if (rejectAnchoredCandidate) {
        return finalPass;
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
    if ((candidate.sparseGuidedPair || candidate.guidedTriangle)
        && (forcedAnchorMatches.size() >= (candidate.guidedTriangle ? 3 : 2)))
    {
        double anchorSquaredError = 0.0;
        int namedAnchorMatches = 0;
        QVector<Match> namedAnchorMatchesForBrightness;
        namedAnchorMatchesForBrightness.reserve(forcedAnchorMatches.size());
        for (const Match& match : forcedAnchorMatches)
        {
            if ((match.catalogIndex < 0)
                || (match.catalogIndex >= catalogContext.catalogStars.size())
                || (match.detectionIndex < 0)
                || (match.detectionIndex >= starDetections.size())
                || !isNamedSparseGuidedCatalogStar(catalogContext.catalogStars[match.catalogIndex])
                || !isStrongSparseGuidedDetection(starDetections[match.detectionIndex]))
            {
                continue;
            }

            anchorSquaredError += match.distancePixels * match.distancePixels;
            ++namedAnchorMatches;
            namedAnchorMatchesForBrightness.append(match);
        }

        finalPass.sparseGuidedNamedAnchorMatches = namedAnchorMatches;
        if (namedAnchorMatches > 0)
        {
            finalPass.sparseGuidedAnchorRmsErrorPixels =
                std::sqrt(anchorSquaredError / static_cast<double>(namedAnchorMatches));
            finalPass.sparseGuidedAnchorBrightnessRankError = matchBrightnessRankError(
                starDetections,
                detectionIndices,
                finalPass.projectedStars,
                namedAnchorMatchesForBrightness);
            finalPass.sparseGuidedAnchorMeanMagnitude = meanCatalogMagnitudeForMatches(
                catalogContext.catalogStars,
                namedAnchorMatchesForBrightness);
        }
    }

    finalPass.finalMatches = rejectOutlierMatches(
        allMatches,
        settings.m_plateSolveMinMatches,
        finalMatchRadius,
        &finalPass.outlierCount);
    if (candidate.anchored)
    {
        auto preserveForcedAnchorMatch = [&](int detectionIndex, int catalogIndex) {
            if ((detectionIndex < 0) || (catalogIndex < 0)) {
                return;
            }
            for (const Match& match : finalPass.finalMatches)
            {
                if ((match.detectionIndex == detectionIndex) || (match.catalogIndex == catalogIndex)) {
                    return;
                }
            }
            for (const Match& match : allMatches)
            {
                if ((match.detectionIndex == detectionIndex) && (match.catalogIndex == catalogIndex))
                {
                    finalPass.finalMatches.append(match);
                    return;
                }
            }
        };
        preserveForcedAnchorMatch(candidate.anchorDetectionIndex, candidate.anchorCatalogIndex);
        if (candidate.sparseGuidedPair || candidate.guidedTriangle) {
            preserveForcedAnchorMatch(candidate.secondaryAnchorDetectionIndex, candidate.secondaryAnchorCatalogIndex);
        }
        if (candidate.guidedTriangle) {
            preserveForcedAnchorMatch(candidate.tertiaryAnchorDetectionIndex, candidate.tertiaryAnchorCatalogIndex);
        }
    }

    const bool useNarrowGuidedBrightPrior = m_useDirectionSeedPreference && (isNarrowField(settings));
    appendSupplementalMatches(
        starDetections,
        finalPass.projectedStars,
        finalMatchRadius,
        restrictSupplementalMatchesToDetectionIndices ? &detectionIndices : nullptr,
        useNarrowGuidedBrightPrior,
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
        double namedBrightAnchorSquaredError = 0.0;
        double namedBrightAnchorMagnitudeSum = 0.0;
        QSet<int> namedBrightAnchorCatalogs;
        QSet<int> namedBrightAnchorDetections;
        namedBrightAnchorCatalogs.reserve(finalPass.finalMatches.size());
        namedBrightAnchorDetections.reserve(finalPass.finalMatches.size());
        for (const Match& match : finalPass.finalMatches)
        {
            if ((match.catalogIndex < 0)
                || (match.catalogIndex >= catalogContext.catalogStars.size())
                || (match.detectionIndex < 0)
                || (match.detectionIndex >= starDetections.size())
                || namedBrightAnchorCatalogs.contains(match.catalogIndex)
                || namedBrightAnchorDetections.contains(match.detectionIndex))
            {
                continue;
            }

            const CatalogStar& catalogStar = catalogContext.catalogStars[match.catalogIndex];
            if (!isNamedSparseGuidedCatalogStar(catalogStar)
                || !isStrongSparseGuidedDetection(starDetections[match.detectionIndex])
                || (match.distancePixels > finalMatchRadius))
            {
                continue;
            }

            namedBrightAnchorCatalogs.insert(match.catalogIndex);
            namedBrightAnchorDetections.insert(match.detectionIndex);
            namedBrightAnchorSquaredError += match.distancePixels * match.distancePixels;
            namedBrightAnchorMagnitudeSum += catalogStar.magnitude;
        }
        finalPass.namedBrightAnchorMatches = namedBrightAnchorCatalogs.size();
        if (finalPass.namedBrightAnchorMatches > 0)
        {
            finalPass.namedBrightAnchorRmsErrorPixels =
                std::sqrt(namedBrightAnchorSquaredError / static_cast<double>(finalPass.namedBrightAnchorMatches));
            finalPass.namedBrightAnchorMeanMagnitude =
                namedBrightAnchorMagnitudeSum / static_cast<double>(finalPass.namedBrightAnchorMatches);
        }

        QSet<int> matchedCatalogIndices;
        matchedCatalogIndices.reserve(finalPass.finalMatches.size());
        QSet<int> matchedDetectionIndices;
        matchedDetectionIndices.reserve(finalPass.finalMatches.size());
        QHash<int, double> matchedDistanceByCatalog;
        matchedDistanceByCatalog.reserve(finalPass.finalMatches.size());
        QHash<int, int> matchedDetectionByCatalog;
        matchedDetectionByCatalog.reserve(finalPass.finalMatches.size());
        const double priorityMagnitudeLimit = useNarrowGuidedBrightPrior
            ? settings.m_plateSolveMaxMagnitude
            : 5.0;
        const bool useSeedRadialPrior = useNarrowGuidedBrightPrior && !m_directionSeedHasRollPreference;
        const SkyProjector seedRadialProjector = useSeedRadialPrior
            ? createProjector(
                settings,
                imageSize,
                m_directionSeedReferenceAzimuthDegrees,
                m_directionSeedReferenceElevationDegrees,
                m_directionSeedReferenceRollDegrees,
                m_directionSeedReferenceFovDegrees,
                settings.m_lensCenterOffsetX,
                settings.m_lensCenterOffsetY,
                settings.m_lensDistortionK1)
            : SkyProjector();
        const QPointF seedRadialCenter = projectorPrincipalPoint(seedRadialProjector);
        double prioritySeedRadialWeightedError = 0.0;
        double prioritySeedRadialWeight = 0.0;
        for (const Match& match : finalPass.finalMatches)
        {
            matchedCatalogIndices.insert(match.catalogIndex);
            matchedDetectionIndices.insert(match.detectionIndex);
            if ((match.catalogIndex < 0)
                || (match.catalogIndex >= catalogContext.catalogStars.size())
                || (match.detectionIndex < 0)
                || (match.detectionIndex >= starDetections.size()))
            {
                continue;
            }

            const CameraPipelineStarDetection& detection = starDetections[match.detectionIndex];
            const double magnitude = catalogContext.catalogStars[match.catalogIndex].magnitude;
            const auto existingDistanceIt = matchedDistanceByCatalog.constFind(match.catalogIndex);
            if ((existingDistanceIt == matchedDistanceByCatalog.cend())
                || (match.distancePixels < existingDistanceIt.value()))
            {
                matchedDistanceByCatalog.insert(match.catalogIndex, match.distancePixels);
                matchedDetectionByCatalog.insert(match.catalogIndex, match.detectionIndex);
            }
            const double magnitudeSupport =
                catalogMagnitudeSupportWeight(magnitude)
                * matchDistanceSupportWeight(match.distancePixels, finalMatchRadius);
            finalPass.magnitudeWeightedSupport += magnitudeSupport;
            if (magnitude <= priorityMagnitudeLimit) {
                finalPass.priorityMagnitudeWeightedSupport += magnitudeSupport;
                if (seedRadialProjector.valid)
                {
                    const auto visibleIt = catalogContext.visibleStarIndexByCatalogIndex.constFind(match.catalogIndex);
                    if (visibleIt != catalogContext.visibleStarIndexByCatalogIndex.cend())
                    {
                        const int visibleIndex = visibleIt.value();
                        if ((visibleIndex >= 0) && (visibleIndex < catalogContext.visibleStars.size()))
                        {
                            QPointF seedPoint;
                            if (projectVector(seedRadialProjector, catalogContext.visibleStars[visibleIndex].vector, seedPoint))
                            {
                                const double seedRadius = pointDistancePixels(seedPoint, seedRadialCenter);
                                const double detectionRadius = pointDistancePixels(detection.m_center, seedRadialCenter);
                                const double radialError = std::fabs(detectionRadius - seedRadius);
                                const double radialWeight = catalogMagnitudeSupportWeight(magnitude);
                                prioritySeedRadialWeightedError += radialWeight * radialError;
                                prioritySeedRadialWeight += radialWeight;
                                ++finalPass.prioritySeedRadialChecks;
                            }
                        }
                    }
                }
            }

            if (magnitude <= 9.5)
            {
                ++finalPass.brightCatalogShapeChecks;
                if (isImplausiblyCompactBrightCatalogDetection(detection, magnitude, useNarrowGuidedBrightPrior))
                {
                    ++finalPass.brightCatalogShapeMismatches;
                }
            }
        }
        if (prioritySeedRadialWeight > 0.0) {
            finalPass.prioritySeedRadialErrorPixels = prioritySeedRadialWeightedError / prioritySeedRadialWeight;
        }
        if (seedRadialProjector.valid)
        {
            const double seedMatchRadius = std::max(
                static_cast<double>(finalMatchRadius) * 2.0,
                48.0);
            const double maxImageRadius = std::hypot(
                std::max(seedRadialCenter.x(), static_cast<double>(imageSize.width()) - seedRadialCenter.x()),
                std::max(seedRadialCenter.y(), static_cast<double>(imageSize.height()) - seedRadialCenter.y()))
                + seedMatchRadius;
            QVector<double> detectionRadii;
            detectionRadii.reserve(starDetections.size());
            for (const CameraPipelineStarDetection& detection : starDetections) {
                detectionRadii.append(pointDistancePixels(detection.m_center, seedRadialCenter));
            }
            std::sort(detectionRadii.begin(), detectionRadii.end());
            const auto hasDetectionAtSeedRadius = [&detectionRadii, seedMatchRadius](double seedRadius) {
                const auto lower = std::lower_bound(
                    detectionRadii.cbegin(),
                    detectionRadii.cend(),
                    seedRadius - seedMatchRadius);
                return (lower != detectionRadii.cend()) && (*lower <= (seedRadius + seedMatchRadius));
            };

            for (const VisibleCatalogStar& visibleStar : catalogContext.visibleStars)
            {
                if ((visibleStar.catalogIndex < 0)
                    || (visibleStar.catalogIndex >= catalogContext.catalogStars.size()))
                {
                    continue;
                }

                const CatalogStar& catalogStar = catalogContext.catalogStars[visibleStar.catalogIndex];
                if (catalogStar.magnitude > priorityMagnitudeLimit) {
                    continue;
                }

                QPointF seedPoint;
                if (!projectVector(seedRadialProjector, visibleStar.vector, seedPoint)) {
                    continue;
                }

                const double seedRadius = pointDistancePixels(seedPoint, seedRadialCenter);
                if ((seedRadius > maxImageRadius) || !hasDetectionAtSeedRadius(seedRadius)) {
                    continue;
                }

                const double seedRadialWeight = catalogMagnitudeSupportWeight(catalogStar.magnitude);
                finalPass.seedRadialMagnitudeSupport += seedRadialWeight;
                ++finalPass.seedRadialCatalogStars;

                const auto matchedDetectionIt = matchedDetectionByCatalog.constFind(visibleStar.catalogIndex);
                if (matchedDetectionIt == matchedDetectionByCatalog.cend()) {
                    continue;
                }

                const int matchedDetectionIndex = matchedDetectionIt.value();
                if ((matchedDetectionIndex < 0) || (matchedDetectionIndex >= starDetections.size())) {
                    continue;
                }

                const double detectionRadius = pointDistancePixels(
                    starDetections[matchedDetectionIndex].m_center,
                    seedRadialCenter);
                const double radialDistanceWeight = matchDistanceSupportWeight(
                    std::fabs(detectionRadius - seedRadius),
                    seedMatchRadius);
                const auto matchedDistanceIt = matchedDistanceByCatalog.constFind(visibleStar.catalogIndex);
                const double finalDistanceWeight = (matchedDistanceIt == matchedDistanceByCatalog.cend())
                    ? 1.0
                    : matchDistanceSupportWeight(matchedDistanceIt.value(), finalMatchRadius);
                finalPass.matchedSeedRadialMagnitudeSupport +=
                    seedRadialWeight * radialDistanceWeight * finalDistanceWeight;
                ++finalPass.matchedSeedRadialCatalogStars;
            }

            finalPass.seedRadialMagnitudeMatchFraction = (finalPass.seedRadialMagnitudeSupport > 0.0)
                ? finalPass.matchedSeedRadialMagnitudeSupport / finalPass.seedRadialMagnitudeSupport
                : 1.0;
        }
        QVector<int> brightDetectionIndices;
        if (useNarrowGuidedBrightPrior)
        {
            brightDetectionIndices.reserve(starDetections.size());
            for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex) {
                brightDetectionIndices.append(detectionIndex);
            }
        }
        else
        {
            brightDetectionIndices = detectionIndices;
        }
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
        const int brightDetectionLimit = std::min(
            useNarrowGuidedBrightPrior ? 24 : 8,
            static_cast<int>(brightDetectionIndices.size()));
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
            const double expectedMaxMagnitude = m_useDirectionSeedPreference && (isNarrowField(settings))
                ? ((i == 0) ? 8.5 : (i < 3) ? 10.0 : 11.5)
                : ((i == 0) ? 1.0 : (i < 3) ? 3.0 : 4.5);
            brightMagnitudePenalty += rankWeight * std::max(0.0, catalogMagnitude - expectedMaxMagnitude);
            brightMagnitudeWeight += rankWeight;
        }
        finalPass.brightDetectionMatchFraction = (finalPass.brightDetections > 0)
            ? static_cast<double>(finalPass.matchedBrightDetections) / static_cast<double>(finalPass.brightDetections)
            : 1.0;
        finalPass.brightDetectionMagnitudeError = (brightMagnitudeWeight > 0.0)
            ? brightMagnitudePenalty / brightMagnitudeWeight
            : 0.0;

        if (useNarrowGuidedBrightPrior)
        {
            const SkyProjector seedProjector = createProjector(
                settings,
                imageSize,
                m_directionSeedReferenceAzimuthDegrees,
                m_directionSeedReferenceElevationDegrees,
                m_directionSeedReferenceRollDegrees,
                m_directionSeedReferenceFovDegrees,
                settings.m_lensCenterOffsetX,
                settings.m_lensCenterOffsetY,
                settings.m_lensDistortionK1);
            const double seedMatchRadius = std::max(
                static_cast<double>(finalMatchRadius) * 2.0,
                48.0);
            const QRectF seedBrightBounds(
                -seedMatchRadius,
                -seedMatchRadius,
                imageSize.width() + 2.0 * seedMatchRadius,
                imageSize.height() + 2.0 * seedMatchRadius);
            if (seedProjector.valid)
            {
                double prioritySeedProjectedWeightedError = 0.0;
                double prioritySeedProjectedWeight = 0.0;
                for (const VisibleCatalogStar& visibleStar : catalogContext.visibleStars)
                {
                    if ((visibleStar.catalogIndex < 0)
                        || (visibleStar.catalogIndex >= catalogContext.catalogStars.size()))
                    {
                        continue;
                    }

                    const CatalogStar& catalogStar = catalogContext.catalogStars[visibleStar.catalogIndex];
                    if (catalogStar.magnitude > priorityMagnitudeLimit) {
                        continue;
                    }

                    QPointF seedPoint;
                    if (!projectVector(seedProjector, visibleStar.vector, seedPoint)
                        || !seedBrightBounds.contains(seedPoint))
                    {
                        continue;
                    }

                    bool hasNearbySeedDetection = false;
                    for (const CameraPipelineStarDetection& detection : starDetections)
                    {
                        if (pointDistancePixels(detection.m_center, seedPoint) <= seedMatchRadius)
                        {
                            hasNearbySeedDetection = true;
                            break;
                        }
                    }
                    if (!hasNearbySeedDetection) {
                        continue;
                    }

                    const bool isSeedBrightStar = catalogStar.magnitude <= 10.5;
                    const double seedProjectedWeight = catalogMagnitudeSupportWeight(catalogStar.magnitude);
                    finalPass.seedProjectedMagnitudeSupport += seedProjectedWeight;
                    if (isSeedBrightStar) {
                        ++finalPass.seedProjectedBrightStars;
                    }
                    for (const Match& match : finalPass.finalMatches)
                    {
                        if ((match.catalogIndex != visibleStar.catalogIndex)
                            || (match.detectionIndex < 0)
                            || (match.detectionIndex >= starDetections.size()))
                        {
                            continue;
                        }

                        if (pointDistancePixels(starDetections[match.detectionIndex].m_center, seedPoint) <= seedMatchRadius)
                        {
                            const double seedProjectedError = pointDistancePixels(
                                starDetections[match.detectionIndex].m_center,
                                seedPoint);
                            const double matchedDistanceWeight =
                                matchDistanceSupportWeight(match.distancePixels, finalMatchRadius);
                            finalPass.matchedSeedProjectedMagnitudeSupport +=
                                seedProjectedWeight * matchedDistanceWeight;
                            prioritySeedProjectedWeightedError += seedProjectedWeight * seedProjectedError;
                            prioritySeedProjectedWeight += seedProjectedWeight;
                            ++finalPass.prioritySeedProjectedChecks;
                            if (isSeedBrightStar) {
                                ++finalPass.matchedSeedProjectedBrightStars;
                            }
                            break;
                        }
                    }
                }
                if (prioritySeedProjectedWeight > 0.0) {
                    finalPass.prioritySeedProjectedErrorPixels =
                        prioritySeedProjectedWeightedError / prioritySeedProjectedWeight;
                }
            }
            finalPass.seedProjectedBrightMatchFraction = (finalPass.seedProjectedBrightStars > 0)
                ? static_cast<double>(finalPass.matchedSeedProjectedBrightStars) / static_cast<double>(finalPass.seedProjectedBrightStars)
                : 1.0;
            finalPass.seedProjectedMagnitudeMatchFraction = (finalPass.seedProjectedMagnitudeSupport > 0.0)
                ? finalPass.matchedSeedProjectedMagnitudeSupport / finalPass.seedProjectedMagnitudeSupport
                : 1.0;
        }

        const QRectF projectedMagnitudeBounds(
            -finalMatchRadius,
            -finalMatchRadius,
            imageSize.width() + 2.0 * finalMatchRadius,
            imageSize.height() + 2.0 * finalMatchRadius);
        if (useNarrowGuidedBrightPrior)
        {
            for (const ProjectedCatalogStar& projectedStar : finalPass.projectedStars)
            {
                if (!projectedMagnitudeBounds.contains(projectedStar.point)
                    || !std::isfinite(projectedStar.magnitude))
                {
                    continue;
                }

                const double projectedWeight = catalogMagnitudeSupportWeight(projectedStar.magnitude);
                finalPass.projectedMagnitudeSupport += projectedWeight;
                const auto matchedDistanceIt = matchedDistanceByCatalog.constFind(projectedStar.catalogIndex);
                if (matchedDistanceIt != matchedDistanceByCatalog.cend())
                {
                    finalPass.matchedProjectedMagnitudeSupport +=
                        projectedWeight * matchDistanceSupportWeight(matchedDistanceIt.value(), finalMatchRadius);
                }
            }
            finalPass.projectedMagnitudeMatchFraction = (finalPass.projectedMagnitudeSupport > 0.0)
                ? finalPass.matchedProjectedMagnitudeSupport / finalPass.projectedMagnitudeSupport
                : 1.0;
        }

        const QRectF brightProjectedBounds(
            -finalMatchRadius,
            -finalMatchRadius,
            imageSize.width() + 2.0 * finalMatchRadius,
            imageSize.height() + 2.0 * finalMatchRadius);
        const bool useNarrowGuidedBrightProjectedPrior = m_useDirectionSeedPreference && (isNarrowField(settings));
        const double brightProjectedMagnitudeLimit = useNarrowGuidedBrightProjectedPrior
            ? std::min(settings.m_plateSolveMaxMagnitude, 13.0)
            : 5.0;
        QVector<ProjectedCatalogStar> brightProjectedStars;
        brightProjectedStars.reserve(16);
        for (const ProjectedCatalogStar& projectedStar : finalPass.projectedStars)
        {
            if ((projectedStar.magnitude > brightProjectedMagnitudeLimit)
                || !brightProjectedBounds.contains(projectedStar.point))
            {
                continue;
            }
            brightProjectedStars.append(projectedStar);
        }
        std::sort(brightProjectedStars.begin(), brightProjectedStars.end(), [](const ProjectedCatalogStar& lhs, const ProjectedCatalogStar& rhs) {
            return lhs.magnitude < rhs.magnitude;
        });
        const int brightProjectedLimit = useNarrowGuidedBrightProjectedPrior ? 12 : 8;
        if (brightProjectedStars.size() > brightProjectedLimit) {
            brightProjectedStars.resize(brightProjectedLimit);
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
        << " MagSupport=" << evaluation.magnitudeWeightedSupport
        << " PriorityMagSupport=" << evaluation.priorityMagnitudeWeightedSupport
        << " ProjectedMagSupport=" << evaluation.matchedProjectedMagnitudeSupport << "/" << evaluation.projectedMagnitudeSupport
        << " ProjectedMagFraction=" << evaluation.projectedMagnitudeMatchFraction
        << " SeedMagSupport=" << evaluation.matchedSeedProjectedMagnitudeSupport << "/" << evaluation.seedProjectedMagnitudeSupport
        << " SeedMagFraction=" << evaluation.seedProjectedMagnitudeMatchFraction
        << " SeedRadialMagSupport=" << evaluation.matchedSeedRadialMagnitudeSupport << "/" << evaluation.seedRadialMagnitudeSupport
        << " SeedRadialMagFraction=" << evaluation.seedRadialMagnitudeMatchFraction
        << " SeedRadialStars=" << evaluation.matchedSeedRadialCatalogStars << "/" << evaluation.seedRadialCatalogStars
        << " SeedRadial=" << evaluation.prioritySeedRadialErrorPixels << "/" << evaluation.prioritySeedRadialChecks
        << " SeedProjected=" << evaluation.prioritySeedProjectedErrorPixels << "/" << evaluation.prioritySeedProjectedChecks
        << " SeedBright=" << evaluation.matchedSeedProjectedBrightStars << "/" << evaluation.seedProjectedBrightStars
        << " BrightShape=" << evaluation.brightCatalogShapeMismatches << "/" << evaluation.brightCatalogShapeChecks
        << " NamedAnchors=" << evaluation.namedBrightAnchorMatches
        << " NamedAnchorRMS=" << evaluation.namedBrightAnchorRmsErrorPixels
        << " SparseAnchors=" << evaluation.sparseGuidedNamedAnchorMatches
        << " AnchorRMS=" << evaluation.sparseGuidedAnchorRmsErrorPixels
        << " AnchorBrightErr=" << evaluation.sparseGuidedAnchorBrightnessRankError
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
    const bool useNarrowGuidedMatchCap = m_useDirectionSeedPreference
        && (isNarrowField(settings));
    if (!m_useWideCatalogMagnitudePreference && !useNarrowGuidedMatchCap) {
        return static_cast<double>(matchCount);
    }

    const int usefulMatchCap = useNarrowGuidedMatchCap
        ? std::max(settings.m_plateSolveMinMatches + 14, 18)
        : std::max(settings.m_plateSolveMinMatches + 4, 8);
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
    const bool useBrightDetectionPreference = isWidePlateSolveContext(settings)
        || (m_useDirectionSeedPreference && (isNarrowField(settings)));
    if (!useBrightDetectionPreference
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
    const bool useNarrowGuidedProjectedPreference = m_useDirectionSeedPreference
        && (isNarrowField(settings));
    const int minimumProjectedStars = useNarrowGuidedProjectedPreference ? 2 : 3;
    if ((!isWidePlateSolveContext(settings) && !useNarrowGuidedProjectedPreference)
        || (evaluation.brightProjectedStars < minimumProjectedStars))
    {
        return 1.0;
    }

    const double coverage = std::clamp(evaluation.brightProjectedMatchFraction, 0.0, 1.0);
    return (useNarrowGuidedProjectedPreference ? 0.05 : 0.10)
        + (useNarrowGuidedProjectedPreference ? 0.95 : 0.90) * coverage * coverage;
}

double brightDetectionMagnitudeAffinity(const CameraSettings& settings,
                                        const FinalMatchPassEvaluation& evaluation)
{
    const bool useNarrowGuidedMagnitudePrior = m_useDirectionSeedPreference
        && (isNarrowField(settings));
    if ((!isWidePlateSolveContext(settings) && !useNarrowGuidedMagnitudePrior)
        || (evaluation.brightDetections < 3))
    {
        return 1.0;
    }

    const double error = std::max(0.0, evaluation.brightDetectionMagnitudeError);
    const double strength = useNarrowGuidedMagnitudePrior ? 2.5 : 4.0;
    return 1.0 / (1.0 + strength * error * error);
}

double projectedMagnitudeCoverageAffinity(const CameraSettings& settings,
                                          const FinalMatchPassEvaluation& evaluation)
{
    if (!m_useDirectionSeedPreference
        || (!isNarrowField(settings))
        || (evaluation.projectedMagnitudeSupport <= 0.0))
    {
        return 1.0;
    }

    const double coverage = std::clamp(evaluation.projectedMagnitudeMatchFraction, 0.0, 1.0);
    return 0.10 + 0.90 * coverage * coverage;
}

double narrowGuidedBrightConsistencyScore(const CameraSettings& settings,
                                          const FinalMatchPassEvaluation& evaluation)
{
    if (!isLowMagnitudeNarrowGuidedSolve(settings) || !evaluation.projectorValid) {
        return 0.0;
    }

    const double detectionCoverage = (evaluation.brightDetections > 0)
        ? std::clamp(evaluation.brightDetectionMatchFraction, 0.0, 1.0)
        : 1.0;
    const double projectedCoverage = (evaluation.brightProjectedStars > 0)
        ? std::clamp(evaluation.brightProjectedMatchFraction, 0.0, 1.0)
        : 1.0;
    const double magnitudeAffinity = 1.0 / (1.0
        + 2.0 * std::max(0.0, evaluation.brightDetectionMagnitudeError)
            * std::max(0.0, evaluation.brightDetectionMagnitudeError));
    const double projectedMagnitudeCoverage = (evaluation.projectedMagnitudeSupport > 0.0)
        ? std::clamp(evaluation.projectedMagnitudeMatchFraction, 0.0, 1.0)
        : 1.0;
    const double shapeAffinity = (evaluation.brightCatalogShapeChecks > 0)
        ? 1.0 - std::clamp(
            static_cast<double>(evaluation.brightCatalogShapeMismatches)
                / static_cast<double>(evaluation.brightCatalogShapeChecks),
            0.0,
            1.0)
        : 1.0;

    return 4.0 * detectionCoverage
        + 5.0 * projectedCoverage
        + 5.0 * projectedMagnitudeCoverage
        + 3.0 * magnitudeAffinity
        + 2.0 * shapeAffinity
        + 0.20 * static_cast<double>(evaluation.matchedBrightDetections)
        + 0.35 * static_cast<double>(evaluation.matchedBrightProjectedStars);
}

double narrowGuidedSeedConsistencyScore(const CameraSettings& settings,
                                        const FinalMatchPassEvaluation& evaluation)
{
    if (!m_useDirectionSeedPreference
        || (!isNarrowField(settings))
        || !evaluation.projectorValid)
    {
        return 0.0;
    }

    double score = 0.0;
    const bool useSeedProjectedBrightGate = usesSeedProjectedBrightGate(settings);
    if (useSeedProjectedBrightGate && (evaluation.seedProjectedMagnitudeSupport > 0.0))
    {
        const double weightedCoverage = std::clamp(
            evaluation.seedProjectedMagnitudeMatchFraction,
            0.0,
            1.0);
        score += 12.0 * weightedCoverage;
        score += 0.8 * std::min(6.0, evaluation.matchedSeedProjectedMagnitudeSupport);
        score -= 4.0 * (1.0 - weightedCoverage);
    }
    else if (useSeedProjectedBrightGate && (evaluation.seedProjectedBrightStars > 0))
    {
        score += 6.0 * std::clamp(evaluation.seedProjectedBrightMatchFraction, 0.0, 1.0);
    }
    if (useSeedProjectedBrightGate) {
        score += 0.35 * static_cast<double>(evaluation.matchedSeedProjectedBrightStars);
    }

    if ((evaluation.prioritySeedRadialChecks >= 4)
        && std::isfinite(evaluation.prioritySeedRadialErrorPixels))
    {
        const double normalizedError = evaluation.prioritySeedRadialErrorPixels
            / std::max(48.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 2.0);
        score += 8.0 / (1.0 + normalizedError * normalizedError);
    }
    if ((evaluation.prioritySeedProjectedChecks >= 1)
        && std::isfinite(evaluation.prioritySeedProjectedErrorPixels))
    {
        const double normalizedError = evaluation.prioritySeedProjectedErrorPixels
            / std::max(48.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 2.0);
        score += 6.0 / (1.0 + 1.5 * normalizedError * normalizedError);
    }
    if (!useSeedProjectedBrightGate && (evaluation.seedRadialMagnitudeSupport > 0.0))
    {
        const double weightedCoverage = std::clamp(
            evaluation.seedRadialMagnitudeMatchFraction,
            0.0,
            1.0);
        score += 16.0 * weightedCoverage;
        score += 0.10 * std::min(80.0, evaluation.matchedSeedRadialMagnitudeSupport);
        score -= 6.0 * (1.0 - weightedCoverage);
    }
    if (!useSeedProjectedBrightGate && (evaluation.seedProjectedMagnitudeSupport > 0.0))
    {
        const double weightedCoverage = std::clamp(
            evaluation.seedProjectedMagnitudeMatchFraction,
            0.0,
            1.0);
        score += 8.0 * weightedCoverage;
        score += 0.12 * std::min(40.0, evaluation.matchedSeedProjectedMagnitudeSupport);
    }

    if ((evaluation.sparseGuidedNamedAnchorMatches >= 2)
        && std::isfinite(evaluation.sparseGuidedAnchorRmsErrorPixels))
    {
        score += 2.0 / (1.0 + std::max(0.0, evaluation.sparseGuidedAnchorRmsErrorPixels) / 8.0);
    }

    return score;
}

QString finalPassBrightDiagnosticSummary(const FinalMatchPassEvaluation& evaluation)
{
    return QStringLiteral("brightDetections=%1/%2 brightProjected=%3/%4 seedBright=%5/%6 seedMag=%7/%8(%9) seedRadialMag=%10/%11(%12) brightMagErr=%13 brightShape=%14/%15 brightnessErr=%16 magSupport=%17/%18 projectedMag=%19/%20(%21) seedRadial=%22/%23 seedProjected=%24/%25 namedAnchors=%26 namedRms=%27 sparseAnchors=%28 rms=%29 anchorBrightErr=%30")
        .arg(evaluation.matchedBrightDetections)
        .arg(evaluation.brightDetections)
        .arg(evaluation.matchedBrightProjectedStars)
        .arg(evaluation.brightProjectedStars)
        .arg(evaluation.matchedSeedProjectedBrightStars)
        .arg(evaluation.seedProjectedBrightStars)
        .arg(evaluation.matchedSeedProjectedMagnitudeSupport, 0, 'f', 1)
        .arg(evaluation.seedProjectedMagnitudeSupport, 0, 'f', 1)
        .arg(evaluation.seedProjectedMagnitudeMatchFraction, 0, 'f', 3)
        .arg(evaluation.matchedSeedRadialMagnitudeSupport, 0, 'f', 1)
        .arg(evaluation.seedRadialMagnitudeSupport, 0, 'f', 1)
        .arg(evaluation.seedRadialMagnitudeMatchFraction, 0, 'f', 3)
        .arg(evaluation.brightDetectionMagnitudeError, 0, 'f', 2)
        .arg(evaluation.brightCatalogShapeMismatches)
        .arg(evaluation.brightCatalogShapeChecks)
        .arg(evaluation.brightnessRankError, 0, 'f', 3)
        .arg(evaluation.magnitudeWeightedSupport, 0, 'f', 1)
        .arg(evaluation.priorityMagnitudeWeightedSupport, 0, 'f', 1)
        .arg(evaluation.matchedProjectedMagnitudeSupport, 0, 'f', 1)
        .arg(evaluation.projectedMagnitudeSupport, 0, 'f', 1)
        .arg(evaluation.projectedMagnitudeMatchFraction, 0, 'f', 3)
        .arg(evaluation.prioritySeedRadialErrorPixels, 0, 'f', 1)
        .arg(evaluation.prioritySeedRadialChecks)
        .arg(evaluation.prioritySeedProjectedErrorPixels, 0, 'f', 1)
        .arg(evaluation.prioritySeedProjectedChecks)
        .arg(evaluation.namedBrightAnchorMatches)
        .arg(evaluation.namedBrightAnchorRmsErrorPixels, 0, 'f', 2)
        .arg(evaluation.sparseGuidedNamedAnchorMatches)
        .arg(evaluation.sparseGuidedAnchorRmsErrorPixels, 0, 'f', 2)
        .arg(evaluation.sparseGuidedAnchorBrightnessRankError, 0, 'f', 3);
}

bool hasHighConfidenceSparseGuidedAnchors(const CameraSettings& settings,
                                          const FinalMatchPassEvaluation& evaluation)
{
    if (!evaluation.projectorValid
        || !evaluation.pose.sparseGuidedPair
        || !m_useDirectionSeedPreference
        || (!isNarrowField(settings))
        || (evaluation.sparseGuidedNamedAnchorMatches < 2)
        || !std::isfinite(evaluation.sparseGuidedAnchorRmsErrorPixels)
        || !std::isfinite(evaluation.sparseGuidedAnchorBrightnessRankError))
    {
        return false;
    }

    const double maxAnchorRms = std::min(
        static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.45,
        10.0);
    const double directionDistance = directionSeedAngularDistanceDegrees(evaluation.pose);
    const double maxAnchorDirectionDistance = std::max(1.6, settings.m_fov * 1.25);
    const bool enoughBrightDetectionsToCrossCheck =
        isLowMagnitudeNarrowGuidedSolve(settings)
        && (evaluation.brightDetections >= 12);
    if (enoughBrightDetectionsToCrossCheck
        && (evaluation.matchedBrightDetections < 3))
    {
        return false;
    }
    return (evaluation.sparseGuidedAnchorRmsErrorPixels <= maxAnchorRms)
        && (evaluation.sparseGuidedAnchorBrightnessRankError <= 0.86)
        && (directionDistance <= maxAnchorDirectionDistance);
}

bool hasHighConfidenceGuidedTriangleSupport(const CameraSettings& settings,
                                            const FinalMatchPassEvaluation& evaluation)
{
    if (!evaluation.projectorValid
        || !evaluation.pose.guidedTriangle
        || !m_useDirectionSeedPreference
        || (!isNarrowField(settings))
        || (evaluation.finalMatches.size() < std::max(settings.m_plateSolveMinMatches + 12, 16))
        || !std::isfinite(evaluation.rmsErrorPixels)
        || (evaluation.rmsErrorPixels > maxDirectionSeedRmsError(settings, evaluation.finalMatches.size()))
        || ((evaluation.brightCatalogShapeChecks >= 1) && (evaluation.brightCatalogShapeMismatches > 0)))
    {
        return false;
    }

    const double directionDistance = directionSeedAngularDistanceDegrees(evaluation.pose);
    const double maxDirectionDistance = std::max(1.6, settings.m_fov * 1.25);
    const double fovDelta = std::fabs(evaluation.pose.fovDegrees - settings.m_fov);
    const double maxFovDelta = std::max(0.08, settings.m_fov * 0.08);
    if ((directionDistance > maxDirectionDistance) || (fovDelta > maxFovDelta)) {
        return false;
    }

    const bool projectedBrightSupport =
        (evaluation.matchedBrightProjectedStars >= 3)
        && ((evaluation.brightProjectedStars < 10)
            || (evaluation.matchedBrightProjectedStars >= 4))
        && (evaluation.brightProjectedMatchFraction >= 0.30);
    const bool detectedBrightSupport =
        (evaluation.brightDetections < 12)
        || ((evaluation.matchedBrightDetections >= 3)
            && (evaluation.brightDetectionMagnitudeError <= 1.60));
    const bool projectedMagnitudeSupport =
        (evaluation.projectedMagnitudeSupport <= 0.0)
        || ((evaluation.matchedProjectedMagnitudeSupport >= 16.0)
            && (evaluation.projectedMagnitudeMatchFraction >= 0.55));
    const bool seedMagnitudeSupport =
        (evaluation.seedProjectedMagnitudeSupport <= 0.0)
        || ((evaluation.matchedSeedProjectedMagnitudeSupport >= 10.0)
            && (evaluation.seedProjectedMagnitudeMatchFraction >= 0.35));
    const bool brightnessOrderReasonable =
        !std::isfinite(evaluation.brightnessRankError)
        || (evaluation.brightnessRankError <= 0.70);

    return projectedBrightSupport
        && detectedBrightSupport
        && projectedMagnitudeSupport
        && seedMagnitudeSupport
        && brightnessOrderReasonable;
}

double sparseGuidedAnchorRankingScore(const FinalMatchPassEvaluation& evaluation)
{
    if (!std::isfinite(evaluation.sparseGuidedAnchorRmsErrorPixels)
        || !std::isfinite(evaluation.sparseGuidedAnchorBrightnessRankError))
    {
        return std::numeric_limits<double>::infinity();
    }

    const double meanMagnitude = std::isfinite(evaluation.sparseGuidedAnchorMeanMagnitude)
        ? evaluation.sparseGuidedAnchorMeanMagnitude
        : 12.0;
    const double directionDistance = directionSeedAngularDistanceDegrees(evaluation.pose);
    return evaluation.sparseGuidedAnchorRmsErrorPixels
        + 8.0 * evaluation.sparseGuidedAnchorBrightnessRankError
        + 0.12 * meanMagnitude
        + 3.5 * directionDistance;
}

double namedBrightAnchorEvidenceScore(const FinalMatchPassEvaluation& evaluation)
{
    if ((evaluation.namedBrightAnchorMatches <= 0)
        || !std::isfinite(evaluation.namedBrightAnchorRmsErrorPixels))
    {
        return 0.0;
    }

    const double meanMagnitude = std::isfinite(evaluation.namedBrightAnchorMeanMagnitude)
        ? evaluation.namedBrightAnchorMeanMagnitude
        : 12.0;
    const double rmsQuality = 1.0 / (1.0 + std::max(0.0, evaluation.namedBrightAnchorRmsErrorPixels) / 8.0);
    const double magnitudeQuality = 1.0 / (1.0 + std::max(0.0, meanMagnitude - 7.0) / 5.0);
    return static_cast<double>(evaluation.namedBrightAnchorMatches) * (1.0 + rmsQuality + 0.6 * magnitudeQuality);
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
    double matchSupport = wideFinalPassMatchWeight(settings, evaluation.finalMatches.size());
    if (m_useDirectionSeedPreference
        && (isNarrowField(settings))
        && (evaluation.magnitudeWeightedSupport > 0.0))
    {
        const double prioritySupport = narrowGuidedMagnitudePriorityScore(evaluation);
        const double usefulSupportCap = static_cast<double>(
            std::max(settings.m_plateSolveMinMatches + 14, 18));
        matchSupport = std::min(prioritySupport, usefulSupportCap)
            + 0.15 * std::log1p(std::max(0.0, prioritySupport - usefulSupportCap));
    }

    return matchSupport
        * evaluationRmsQuality(pose, normalizationRadius)
        * brightnessAffinity(pose)
        * wideFinalPassMagnitudeAffinity(evaluation)
        * brightDetectionCoverageAffinity(settings, evaluation)
        * brightProjectedCoverageAffinity(settings, evaluation)
        * brightDetectionMagnitudeAffinity(settings, evaluation)
        * projectedMagnitudeCoverageAffinity(settings, evaluation)
        * prioritySeedProjectedAffinity(evaluation, settings.m_plateSolveFinalMatchRadius)
        * prioritySeedRadialAffinity(evaluation, settings.m_plateSolveFinalMatchRadius)
        * seedProjectedMagnitudeCoverageAffinity(evaluation)
        * seedRadialMagnitudeCoverageAffinity(evaluation)
        * (m_useDirectionSeedPreference && (isNarrowField(settings))
            ? (1.0 + 0.18 * namedBrightAnchorEvidenceScore(evaluation))
            : 1.0)
        * directionSeedAffinity(pose)
        * fovSeedAffinity(pose)
        * allSkyZenithAffinity(pose);
}

double narrowGuidedEvidenceScore(const CameraSettings& settings,
                                 const FinalMatchPassEvaluation& evaluation)
{
    if (!m_useDirectionSeedPreference
        || (!isNarrowField(settings))
        || !evaluation.projectorValid
        || !std::isfinite(evaluation.rmsErrorPixels))
    {
        return -std::numeric_limits<double>::infinity();
    }

    const int usefulMatchCap = std::max(settings.m_plateSolveMinMatches + 14, 18);
    const int matchCount = static_cast<int>(evaluation.finalMatches.size());
    const double matchSupport = std::min(matchCount, usefulMatchCap)
        + 0.05 * std::log1p(static_cast<double>(std::max(0, matchCount - usefulMatchCap)));
    const double rmsPenalty = 6.0 * evaluation.rmsErrorPixels
        / std::max(1.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius));
    const double directionPenalty = 1.8 * directionSeedAngularDistanceDegrees(evaluation.pose)
        / std::max(0.25, static_cast<double>(settings.m_fov));
    const double fovPenalty = 2.0
        * std::fabs(evaluation.pose.fovDegrees - m_directionSeedReferenceFovDegrees)
        / std::max(0.03, static_cast<double>(settings.m_fov) * 0.03);

    return 1.20 * narrowGuidedMagnitudePriorityScore(evaluation)
        + 2.75 * narrowGuidedSeedConsistencyScore(settings, evaluation)
        + 2.25 * narrowGuidedBrightConsistencyScore(settings, evaluation)
        + 0.55 * matchSupport
        - rmsPenalty
        - directionPenalty
        - fovPenalty;
}

double finalMatchPassEvidenceScore(const CameraSettings& settings,
                                   const FinalMatchPassEvaluation& evaluation)
{
    if (!evaluation.projectorValid
        || !std::isfinite(evaluation.rmsErrorPixels))
    {
        return -std::numeric_limits<double>::infinity();
    }

    const bool narrowGuided =
        m_useDirectionSeedPreference && (isNarrowField(settings));
    const int usefulMatchCap = narrowGuided
        ? std::max(settings.m_plateSolveMinMatches + 14, 18)
        : std::max(settings.m_plateSolveMinMatches + 8, 12);
    const int matchCount = static_cast<int>(evaluation.finalMatches.size());
    double matchEvidence = std::min(matchCount, usefulMatchCap)
        + 0.20 * std::log1p(static_cast<double>(std::max(0, matchCount - usefulMatchCap)));
    if (narrowGuided && (evaluation.magnitudeWeightedSupport > 0.0))
    {
        const double prioritySupport = narrowGuidedMagnitudePriorityScore(evaluation);
        matchEvidence = std::max(
            matchEvidence,
            std::min(prioritySupport, static_cast<double>(usefulMatchCap))
                + 0.10 * std::log1p(std::max(0.0, prioritySupport - static_cast<double>(usefulMatchCap))));
    }

    const double radius = std::max(
        1.0,
        static_cast<double>(settings.m_plateSolveFinalMatchRadius));
    const double rmsPenalty = 5.5 * std::min(4.0, evaluation.rmsErrorPixels / radius);
    const double maxPenalty = std::isfinite(evaluation.maxErrorPixels)
        ? 0.8 * std::min(4.0, evaluation.maxErrorPixels / std::max(radius, 1.0))
        : 0.0;
    const double brightnessPenalty = std::isfinite(evaluation.brightnessRankError)
        ? 5.0 * std::clamp(evaluation.brightnessRankError, 0.0, 1.5)
        : 0.0;
    const double outlierPenalty = 0.55 * static_cast<double>(evaluation.outlierCount);

    double score = 1.25 * matchEvidence
        - rmsPenalty
        - maxPenalty
        - brightnessPenalty
        - outlierPenalty;

    if (std::isfinite(evaluation.meanCatalogMagnitude))
    {
        const double referenceMagnitude = narrowGuided ? 12.0 : 5.0;
        score -= 0.18 * std::max(0.0, evaluation.meanCatalogMagnitude - referenceMagnitude);
    }

    if (evaluation.brightDetections >= 3)
    {
        const double coverage = std::clamp(evaluation.brightDetectionMatchFraction, 0.0, 1.0);
        score += (narrowGuided ? 7.0 : 5.0) * coverage;
        score -= (narrowGuided ? 4.0 : 2.5) * (1.0 - coverage);
    }
    if (evaluation.brightProjectedStars >= (narrowGuided ? 2 : 3))
    {
        const double coverage = std::clamp(evaluation.brightProjectedMatchFraction, 0.0, 1.0);
        score += (narrowGuided ? 8.0 : 5.5) * coverage;
        score -= (narrowGuided ? 5.0 : 3.0) * (1.0 - coverage);
    }
    if (evaluation.projectedMagnitudeSupport > 0.0)
    {
        const double coverage = std::clamp(evaluation.projectedMagnitudeMatchFraction, 0.0, 1.0);
        score += (narrowGuided ? 8.0 : 4.0) * coverage;
        score += 0.08 * std::min(80.0, evaluation.matchedProjectedMagnitudeSupport);
    }
    if (evaluation.seedProjectedMagnitudeSupport > 0.0)
    {
        const double coverage = std::clamp(evaluation.seedProjectedMagnitudeMatchFraction, 0.0, 1.0);
        score += (narrowGuided ? 6.0 : 2.0) * coverage;
        score += 0.05 * std::min(80.0, evaluation.matchedSeedProjectedMagnitudeSupport);
    }
    if ((evaluation.prioritySeedProjectedChecks >= 1)
        && std::isfinite(evaluation.prioritySeedProjectedErrorPixels))
    {
        const double seedProjectedAffinity = prioritySeedProjectedAffinity(
            evaluation,
            static_cast<double>(settings.m_plateSolveFinalMatchRadius));
        score += (narrowGuided ? 5.0 : 1.5) * seedProjectedAffinity;
        score -= (narrowGuided ? 3.0 : 1.0) * (1.0 - seedProjectedAffinity);
    }
    if (evaluation.seedRadialMagnitudeSupport > 0.0)
    {
        const double coverage = std::clamp(evaluation.seedRadialMagnitudeMatchFraction, 0.0, 1.0);
        score += (narrowGuided ? 7.0 : 2.0) * coverage;
        score += 0.05 * std::min(80.0, evaluation.matchedSeedRadialMagnitudeSupport);
    }
    if (evaluation.brightCatalogShapeChecks > 0)
    {
        const double mismatchFraction = std::clamp(
            static_cast<double>(evaluation.brightCatalogShapeMismatches)
                / static_cast<double>(evaluation.brightCatalogShapeChecks),
            0.0,
            1.0);
        score -= 4.0 * mismatchFraction;
    }
    if (evaluation.brightDetections >= 3)
    {
        const double magnitudeError = std::max(0.0, evaluation.brightDetectionMagnitudeError);
        score -= (narrowGuided ? 4.0 : 2.5) * std::min(2.0, magnitudeError * magnitudeError);
    }

    if (m_useDirectionSeedPreference)
    {
        const double directionDistance = directionSeedAngularDistanceDegrees(evaluation.pose);
        const double directionScale = narrowGuided
            ? std::max(0.25, static_cast<double>(settings.m_fov))
            : std::max(5.0, static_cast<double>(settings.m_plateSolveSearchRadius));
        score -= 2.0 * std::min(9.0, std::pow(directionDistance / directionScale, 2.0));

        const double fovDelta = std::fabs(evaluation.pose.fovDegrees - m_directionSeedReferenceFovDegrees);
        const double fovScale = narrowGuided
            ? std::max(0.03, static_cast<double>(settings.m_fov) * 0.03)
            : std::max(1.0, static_cast<double>(settings.m_fov) * 0.08);
        score -= 1.5 * std::min(9.0, std::pow(fovDelta / fovScale, 2.0));

        if (m_directionSeedHasRollPreference)
        {
            const double rollDelta = angularDistanceDegrees(
                evaluation.pose.rollDegrees,
                m_directionSeedReferenceRollDegrees);
            score -= 1.0 * std::min(9.0, std::pow(
                rollDelta / std::max(1.0, m_directionSeedRollScaleDegrees),
                2.0));
        }
    }

    if (narrowGuided) {
        score += 0.70 * narrowGuidedSeedConsistencyScore(settings, evaluation);
        score += 0.45 * narrowGuidedBrightConsistencyScore(settings, evaluation);
        score += 2.25 * namedBrightAnchorEvidenceScore(evaluation);
    }

    return score;
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

    const bool useNarrowGuidedMatchCap = m_useDirectionSeedPreference
        && (isNarrowField(settings));
    if (useNarrowGuidedMatchCap)
    {
        const int candidateMatches = static_cast<int>(candidate.finalMatches.size());
        const int bestMatches = static_cast<int>(best.finalMatches.size());
        const double namedAnchorRmsCap = std::min(
            maxDirectionSeedRmsError(settings, std::max(candidateMatches, bestMatches)) * 1.10,
            static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.75);
        const bool candidateNamedAnchorAccepted =
            (candidate.namedBrightAnchorMatches >= 2)
            && std::isfinite(candidate.namedBrightAnchorRmsErrorPixels)
            && (candidate.namedBrightAnchorRmsErrorPixels <= namedAnchorRmsCap)
            && (candidate.rmsErrorPixels <= std::min(
                maxDirectionSeedRmsError(settings, candidateMatches) * 1.15,
                static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.78));
        const bool bestNamedAnchorAccepted =
            (best.namedBrightAnchorMatches >= 2)
            && std::isfinite(best.namedBrightAnchorRmsErrorPixels)
            && (best.namedBrightAnchorRmsErrorPixels <= namedAnchorRmsCap)
            && (best.rmsErrorPixels <= std::min(
                maxDirectionSeedRmsError(settings, bestMatches) * 1.15,
                static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.78));
        const double strongNamedAnchorRmsCap = std::max(
            namedAnchorRmsCap,
            static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.65);
        const bool candidateHasStrongNamedAnchorSet =
            (candidate.namedBrightAnchorMatches >= 3)
            && std::isfinite(candidate.namedBrightAnchorRmsErrorPixels)
            && (candidate.namedBrightAnchorRmsErrorPixels <= strongNamedAnchorRmsCap)
            && (candidateMatches >= std::max(settings.m_plateSolveMinMatches + 6, 10));
        const bool bestHasStrongNamedAnchorSet =
            (best.namedBrightAnchorMatches >= 3)
            && std::isfinite(best.namedBrightAnchorRmsErrorPixels)
            && (best.namedBrightAnchorRmsErrorPixels <= strongNamedAnchorRmsCap)
            && (bestMatches >= std::max(settings.m_plateSolveMinMatches + 6, 10));
        if (candidateHasStrongNamedAnchorSet != bestHasStrongNamedAnchorSet) {
            const bool candidateHasStrongDenseGuidedSupport =
                hasStrongDenseNarrowGuidedFinalPass(settings, candidate);
            const bool bestHasStrongDenseGuidedSupport =
                hasStrongDenseNarrowGuidedFinalPass(settings, best);
            if (candidateHasStrongDenseGuidedSupport != bestHasStrongDenseGuidedSupport) {
                return candidateHasStrongDenseGuidedSupport;
            }
            return candidateHasStrongNamedAnchorSet;
        }
        const bool candidateHasStrongDenseGuidedSupport =
            hasStrongDenseNarrowGuidedFinalPass(settings, candidate);
        const bool bestHasStrongDenseGuidedSupport =
            hasStrongDenseNarrowGuidedFinalPass(settings, best);
        if (candidateHasStrongDenseGuidedSupport != bestHasStrongDenseGuidedSupport) {
            return candidateHasStrongDenseGuidedSupport;
        }
        if (candidateNamedAnchorAccepted != bestNamedAnchorAccepted)
        {
            const bool candidateHasEnoughSupport =
                candidateMatches >= std::max(settings.m_plateSolveMinMatches + 10, 16);
            const bool bestHasEnoughSupport =
                bestMatches >= std::max(settings.m_plateSolveMinMatches + 10, 16);
            if (candidateHasEnoughSupport && bestHasEnoughSupport) {
                return candidateNamedAnchorAccepted;
            }
        }
        if (candidateNamedAnchorAccepted && bestNamedAnchorAccepted)
        {
            const double namedAnchorDelta =
                namedBrightAnchorEvidenceScore(candidate) - namedBrightAnchorEvidenceScore(best);
            if (std::fabs(namedAnchorDelta) >= 1.25) {
                return namedAnchorDelta > 0.0;
            }
        }

        const int seedConsistencyMatchTolerance = std::max(
            20,
            static_cast<int>(std::ceil(static_cast<double>(std::max(candidateMatches, bestMatches)) * 0.25)));
        const bool seedConsistencyComparableSupport =
            (candidateMatches >= std::max(settings.m_plateSolveMinMatches + 4, bestMatches - seedConsistencyMatchTolerance))
            && (bestMatches >= std::max(settings.m_plateSolveMinMatches + 4, candidateMatches - seedConsistencyMatchTolerance));
        const double seedConsistencyRmsTolerance = std::max(
            3.0,
            static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.20);
        const bool seedConsistencyComparableRms =
            (candidate.rmsErrorPixels <= (best.rmsErrorPixels + seedConsistencyRmsTolerance))
            && (best.rmsErrorPixels <= (candidate.rmsErrorPixels + seedConsistencyRmsTolerance));
        if (!plateSolveStartUsesRoll(settings)
            && seedConsistencyComparableSupport
            && seedConsistencyComparableRms)
        {
            const double candidateSeedConsistency = narrowGuidedSeedConsistencyScore(settings, candidate);
            const double bestSeedConsistency = narrowGuidedSeedConsistencyScore(settings, best);
            const double seedConsistencyDelta = candidateSeedConsistency - bestSeedConsistency;
            if (std::fabs(seedConsistencyDelta) >= 1.5) {
                return seedConsistencyDelta > 0.0;
            }
        }

        if (seedConsistencyComparableSupport
            && seedConsistencyComparableRms
            && ((candidate.seedProjectedMagnitudeSupport > 0.0)
                || (best.seedProjectedMagnitudeSupport > 0.0)))
        {
            const double candidateSeedProjectedSupport =
                candidate.matchedSeedProjectedMagnitudeSupport;
            const double bestSeedProjectedSupport =
                best.matchedSeedProjectedMagnitudeSupport;
            const double seedProjectedSupportDelta =
                candidateSeedProjectedSupport - bestSeedProjectedSupport;
            const double meaningfulSeedProjectedSupportDelta = std::max(
                4.0,
                0.10 * std::max(candidateSeedProjectedSupport, bestSeedProjectedSupport));
            if (std::fabs(seedProjectedSupportDelta) >= meaningfulSeedProjectedSupportDelta) {
                return seedProjectedSupportDelta > 0.0;
            }

            const double seedProjectedFractionDelta =
                candidate.seedProjectedMagnitudeMatchFraction - best.seedProjectedMagnitudeMatchFraction;
            if (std::fabs(seedProjectedFractionDelta) >= 0.04) {
                return seedProjectedFractionDelta > 0.0;
            }
        }
        if (seedConsistencyComparableSupport
            && seedConsistencyComparableRms
            && (candidate.prioritySeedProjectedChecks >= 1)
            && (best.prioritySeedProjectedChecks >= 1)
            && std::isfinite(candidate.prioritySeedProjectedErrorPixels)
            && std::isfinite(best.prioritySeedProjectedErrorPixels))
        {
            const double projectedErrorDelta =
                best.prioritySeedProjectedErrorPixels - candidate.prioritySeedProjectedErrorPixels;
            const double meaningfulProjectedErrorDelta = std::max(
                10.0,
                static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.75);
            if (std::fabs(projectedErrorDelta) >= meaningfulProjectedErrorDelta) {
                return projectedErrorDelta > 0.0;
            }
        }

        const bool candidateHasStrongSparseAnchors =
            hasHighConfidenceSparseGuidedAnchors(settings, candidate);
        const bool bestHasStrongSparseAnchors =
            hasHighConfidenceSparseGuidedAnchors(settings, best);
        const double candidateAnchorMagnitudeSupport = narrowGuidedMagnitudePriorityScore(candidate);
        const double bestAnchorMagnitudeSupport = narrowGuidedMagnitudePriorityScore(best);
        const double anchorMagnitudeSupportTolerance = std::max(
            4.0,
            0.08 * std::max(candidateAnchorMagnitudeSupport, bestAnchorMagnitudeSupport));
        const double anchorRmsTolerance = std::max(2.5, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.15);
        const bool anchorCandidatesHaveComparableRms =
            (candidate.rmsErrorPixels <= (best.rmsErrorPixels + anchorRmsTolerance))
            && (best.rmsErrorPixels <= (candidate.rmsErrorPixels + anchorRmsTolerance));
        if (candidateHasStrongSparseAnchors != bestHasStrongSparseAnchors)
        {
            if (anchorCandidatesHaveComparableRms
                && (std::fabs(candidateAnchorMagnitudeSupport - bestAnchorMagnitudeSupport) >= anchorMagnitudeSupportTolerance))
            {
                return candidateAnchorMagnitudeSupport > bestAnchorMagnitudeSupport;
            }
            return candidateHasStrongSparseAnchors;
        }
        if (candidateHasStrongSparseAnchors && bestHasStrongSparseAnchors)
        {
            const double candidateAnchorScore = sparseGuidedAnchorRankingScore(candidate);
            const double bestAnchorScore = sparseGuidedAnchorRankingScore(best);
            const double anchorScoreDelta = candidateAnchorScore - bestAnchorScore;
            const int candidateMatches = static_cast<int>(candidate.finalMatches.size());
            const int bestMatches = static_cast<int>(best.finalMatches.size());
            const int matchDelta = candidateMatches - bestMatches;
            const int meaningfulMatchDelta = std::max(
                10,
                static_cast<int>(std::ceil(static_cast<double>(std::max(candidateMatches, bestMatches)) * 0.10)));
            const double rmsTolerance = std::max(2.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.10);
            if ((std::abs(matchDelta) >= meaningfulMatchDelta)
                && (candidate.rmsErrorPixels <= (best.rmsErrorPixels + rmsTolerance))
                && (best.rmsErrorPixels <= (candidate.rmsErrorPixels + rmsTolerance)))
            {
                return matchDelta > 0;
            }

            const int seedRadialMatchTolerance = std::max(
                meaningfulMatchDelta,
                static_cast<int>(std::ceil(static_cast<double>(std::max(candidateMatches, bestMatches)) * 0.25)));
            if (anchorCandidatesHaveComparableRms
                && shouldPreferSeedRadialConsistency(candidate, best, seedRadialMatchTolerance))
            {
                return true;
            }
            if (anchorCandidatesHaveComparableRms
                && shouldPreferSeedRadialConsistency(best, candidate, seedRadialMatchTolerance))
            {
                return false;
            }

            const bool candidateHasComparableSupport =
                candidateMatches >= std::max(settings.m_plateSolveMinMatches + 4, static_cast<int>(std::floor(bestMatches * 0.90)));
            const bool bestHasComparableSupport =
                bestMatches >= std::max(settings.m_plateSolveMinMatches + 4, static_cast<int>(std::floor(candidateMatches * 0.90)));
            if ((std::fabs(anchorScoreDelta) >= 1.0)
                && candidateHasComparableSupport
                && bestHasComparableSupport)
            {
                return anchorScoreDelta < 0.0;
            }

            if (anchorCandidatesHaveComparableRms
                && (std::fabs(candidateAnchorMagnitudeSupport - bestAnchorMagnitudeSupport) >= anchorMagnitudeSupportTolerance))
            {
                return candidateAnchorMagnitudeSupport > bestAnchorMagnitudeSupport;
            }

            const int actualAnchorMatchDelta = candidateMatches - bestMatches;
            if ((std::abs(actualAnchorMatchDelta) >= 6)
                && (std::fabs(anchorScoreDelta) <= 2.0)
                && (candidate.rmsErrorPixels <= (best.rmsErrorPixels + rmsTolerance))
                && (best.rmsErrorPixels <= (candidate.rmsErrorPixels + rmsTolerance)))
            {
                return actualAnchorMatchDelta > 0;
            }
        }
    }

    if (m_useDirectionSeedPreference)
    {
        const bool candidateDirectionAccepted = candidateMeetsMinMatches
            && (isAcceptableDirectionSeedSolve(
                    settings,
                    starDetections,
                    candidate.finalMatches,
                    candidate.rmsErrorPixels,
                    candidate.maxErrorPixels)
                || isAcceptableSparseGuidedRankingFinalPass(settings, candidate))
            && hasAcceptableGuidedFinalBrightnessConsistency(settings, candidate);
        const bool bestDirectionAccepted = bestMeetsMinMatches
            && (isAcceptableDirectionSeedSolve(
                    settings,
                    starDetections,
                    best.finalMatches,
                    best.rmsErrorPixels,
                    best.maxErrorPixels)
                || isAcceptableSparseGuidedRankingFinalPass(settings, best))
            && hasAcceptableGuidedFinalBrightnessConsistency(settings, best);
        if (candidateDirectionAccepted != bestDirectionAccepted) {
            return candidateDirectionAccepted;
        }

        const bool candidateBrightnessAccepted = hasAcceptableGuidedFinalBrightnessConsistency(settings, candidate);
        const bool bestBrightnessAccepted = hasAcceptableGuidedFinalBrightnessConsistency(settings, best);
        if (candidateBrightnessAccepted != bestBrightnessAccepted) {
            return candidateBrightnessAccepted;
        }

        if (useNarrowGuidedMatchCap
            && candidateDirectionAccepted
            && bestDirectionAccepted
            && candidateBrightnessAccepted
            && bestBrightnessAccepted)
        {
            const double candidateEvidenceScore = std::max(
                narrowGuidedEvidenceScore(settings, candidate),
                finalMatchPassEvidenceScore(settings, candidate));
            const double bestEvidenceScore = std::max(
                narrowGuidedEvidenceScore(settings, best),
                finalMatchPassEvidenceScore(settings, best));
            const double evidenceScale = std::max({
                1.0,
                std::fabs(candidateEvidenceScore),
                std::fabs(bestEvidenceScore)
            });
            if (std::fabs(candidateEvidenceScore - bestEvidenceScore)
                >= std::max(2.0, evidenceScale * 0.08))
            {
                return candidateEvidenceScore > bestEvidenceScore;
            }
        }

        if (isLowMagnitudeNarrowGuidedSolve(settings)
            && candidateDirectionAccepted
            && bestDirectionAccepted
            && candidateBrightnessAccepted
            && bestBrightnessAccepted)
        {
            const int actualMatchDelta =
                static_cast<int>(candidate.finalMatches.size()) - static_cast<int>(best.finalMatches.size());
            const double rmsTolerance = std::max(2.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.10);
            if ((std::abs(actualMatchDelta) >= 16)
                && (candidate.rmsErrorPixels <= (best.rmsErrorPixels + rmsTolerance))
                && (best.rmsErrorPixels <= (candidate.rmsErrorPixels + rmsTolerance)))
            {
                const double candidateBrightScore = narrowGuidedBrightConsistencyScore(settings, candidate);
                const double bestBrightScore = narrowGuidedBrightConsistencyScore(settings, best);
                if (std::fabs(candidateBrightScore - bestBrightScore) <= 1.0) {
                    return actualMatchDelta > 0;
                }
            }
        }
    }

    const int narrowGuidedMatchCap = std::max(settings.m_plateSolveMinMatches + 14, 18);
    const int candidateComparableMatchCount = useNarrowGuidedMatchCap
        ? std::min(static_cast<int>(candidate.finalMatches.size()), narrowGuidedMatchCap)
        : static_cast<int>(candidate.finalMatches.size());
    const int bestComparableMatchCount = useNarrowGuidedMatchCap
        ? std::min(static_cast<int>(best.finalMatches.size()), narrowGuidedMatchCap)
        : static_cast<int>(best.finalMatches.size());
    const int finalMatchDelta = candidateComparableMatchCount - bestComparableMatchCount;
    const int actualFinalMatchDelta = static_cast<int>(candidate.finalMatches.size()) - static_cast<int>(best.finalMatches.size());
    const int narrowGuidedTieBreakMatchTolerance = std::max(3, settings.m_plateSolveMinMatches + 1);
    const bool narrowGuidedActualMatchCountsAreClose =
        std::abs(actualFinalMatchDelta) <= narrowGuidedTieBreakMatchTolerance;

    if (useNarrowGuidedMatchCap && (std::abs(finalMatchDelta) <= 1))
    {
        const bool useSeedProjectedBrightTieBreak = usesSeedProjectedBrightGate(settings);
        const int seedBrightDelta =
            candidate.matchedSeedProjectedBrightStars - best.matchedSeedProjectedBrightStars;
        if (useSeedProjectedBrightTieBreak
            && ((candidate.seedProjectedBrightStars > 0) || (best.seedProjectedBrightStars > 0))
            && (seedBrightDelta != 0))
        {
            return seedBrightDelta > 0;
        }

        const double seedBrightFractionDelta =
            candidate.seedProjectedBrightMatchFraction - best.seedProjectedBrightMatchFraction;
        if (useSeedProjectedBrightTieBreak
            && ((candidate.seedProjectedBrightStars >= 2) || (best.seedProjectedBrightStars >= 2))
            && (std::fabs(seedBrightFractionDelta) >= 0.25))
        {
            return seedBrightFractionDelta > 0.0;
        }
    }

    if (m_useDirectionSeedPreference && (isNarrowField(settings)))
    {
        const double candidateFovDelta = std::fabs(candidate.pose.fovDegrees - m_directionSeedReferenceFovDegrees);
        const double bestFovDelta = std::fabs(best.pose.fovDegrees - m_directionSeedReferenceFovDegrees);
        const double fovDelta = candidateFovDelta - bestFovDelta;
        const double meaningfulFovDelta = std::max(0.02, static_cast<double>(settings.m_fov) * 0.02);
        const double rmsTolerance = std::max(3.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.20);
        if (narrowGuidedActualMatchCountsAreClose
            && (std::fabs(fovDelta) >= meaningfulFovDelta)
            && (candidate.rmsErrorPixels <= (best.rmsErrorPixels + rmsTolerance))
            && (best.rmsErrorPixels <= (candidate.rmsErrorPixels + rmsTolerance)))
        {
            return fovDelta < 0.0;
        }

        const double candidateDirectionDelta = directionSeedAngularDistanceDegrees(candidate.pose);
        const double bestDirectionDelta = directionSeedAngularDistanceDegrees(best.pose);
        const double directionDelta = candidateDirectionDelta - bestDirectionDelta;
        const double meaningfulDirectionDelta = std::max(0.18, static_cast<double>(settings.m_fov) * 0.15);
        if (narrowGuidedActualMatchCountsAreClose
            && (std::fabs(directionDelta) >= meaningfulDirectionDelta)
            && (candidate.rmsErrorPixels <= (best.rmsErrorPixels + rmsTolerance))
            && (best.rmsErrorPixels <= (candidate.rmsErrorPixels + rmsTolerance)))
        {
            return directionDelta < 0.0;
        }
    }

    if (useNarrowGuidedMatchCap)
    {
        const double candidateMagnitudeSupport = narrowGuidedMagnitudePriorityScore(candidate);
        const double bestMagnitudeSupport = narrowGuidedMagnitudePriorityScore(best);
        const double magnitudeSupportDelta = candidateMagnitudeSupport - bestMagnitudeSupport;
        const double meaningfulMagnitudeSupportDelta = std::max(
            2.5,
            0.08 * std::max(candidateMagnitudeSupport, bestMagnitudeSupport));
        const double rmsTolerance = std::max(2.5, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.12);
        const bool candidateBrightnessAccepted = hasAcceptableGuidedFinalBrightnessConsistency(settings, candidate);
        const bool bestBrightnessAccepted = hasAcceptableGuidedFinalBrightnessConsistency(settings, best);
        if ((std::fabs(magnitudeSupportDelta) >= meaningfulMagnitudeSupportDelta)
            && (candidate.rmsErrorPixels <= (best.rmsErrorPixels + rmsTolerance))
            && (best.rmsErrorPixels <= (candidate.rmsErrorPixels + rmsTolerance))
            && candidateBrightnessAccepted
            && bestBrightnessAccepted)
        {
            return magnitudeSupportDelta > 0.0;
        }

        const double projectedCoverageDelta = candidate.brightProjectedMatchFraction - best.brightProjectedMatchFraction;
        if ((candidate.brightProjectedStars >= 2)
            && (best.brightProjectedStars >= 2)
            && narrowGuidedActualMatchCountsAreClose
            && (std::fabs(projectedCoverageDelta) >= 0.20))
        {
            return projectedCoverageDelta > 0.0;
        }

        const double magnitudeErrorDelta = candidate.brightDetectionMagnitudeError - best.brightDetectionMagnitudeError;
        if ((candidate.brightDetections >= 3)
            && (best.brightDetections >= 3)
            && narrowGuidedActualMatchCountsAreClose
            && (std::fabs(magnitudeErrorDelta) >= 0.25))
        {
            return magnitudeErrorDelta < 0.0;
        }

        const double coverageDelta = candidate.brightDetectionMatchFraction - best.brightDetectionMatchFraction;
        if ((candidate.brightDetections >= 3)
            && (best.brightDetections >= 3)
            && narrowGuidedActualMatchCountsAreClose
            && (std::fabs(coverageDelta) >= 0.15))
        {
            return coverageDelta > 0.0;
        }
    }

    if (isWidePlateSolveContext(settings) || (m_useDirectionSeedPreference && (isNarrowField(settings))))
    {
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
        }
        else if (m_useDirectionSeedPreference && (isNarrowField(settings)))
        {
            const double projectedCoverageDelta = candidate.brightProjectedMatchFraction - best.brightProjectedMatchFraction;
            if ((candidate.brightProjectedStars >= 2)
                && (best.brightProjectedStars >= 2)
                && narrowGuidedActualMatchCountsAreClose
                && (std::fabs(projectedCoverageDelta) >= 0.20))
            {
                return projectedCoverageDelta > 0.0;
            }

            const double magnitudeErrorDelta = candidate.brightDetectionMagnitudeError - best.brightDetectionMagnitudeError;
            if ((candidate.brightDetections >= 3)
                && (best.brightDetections >= 3)
                && narrowGuidedActualMatchCountsAreClose
                && (std::fabs(magnitudeErrorDelta) >= 0.25))
            {
                return magnitudeErrorDelta < 0.0;
            }
        }

        const double coverageDelta = candidate.brightDetectionMatchFraction - best.brightDetectionMatchFraction;
        if ((candidate.brightDetections >= 3)
            && (best.brightDetections >= 3)
            && (!(m_useDirectionSeedPreference && (isNarrowField(settings)))
                || narrowGuidedActualMatchCountsAreClose)
            && (std::fabs(coverageDelta) >= 0.15))
        {
            return coverageDelta > 0.0;
        }
    }

    if (m_useWideCatalogMagnitudePreference || m_useDirectionSeedPreference)
    {
        const double candidateScore = finalMatchPassScore(settings, candidate);
        const double bestScore = finalMatchPassScore(settings, best);
        const double scoreScale = std::max({
            1e-9,
            std::fabs(candidateScore),
            std::fabs(bestScore)
        });
        if (std::fabs(candidateScore - bestScore) > std::max(1e-8, scoreScale * 0.08)) {
            return candidateScore > bestScore;
        }
    }

    if (isLowMagnitudeNarrowGuidedSolve(settings) && narrowGuidedActualMatchCountsAreClose)
    {
        const double candidateBrightScore = narrowGuidedBrightConsistencyScore(settings, candidate);
        const double bestBrightScore = narrowGuidedBrightConsistencyScore(settings, best);
        if (std::fabs(candidateBrightScore - bestBrightScore) > 0.35) {
            return candidateBrightScore > bestBrightScore;
        }
    }

    if (useNarrowGuidedMatchCap && (finalMatchDelta != 0)) {
        return finalMatchDelta > 0;
    }

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

bool hasCompetitiveRollAlias(const CameraSettings& settings,
                             const PlateSolveCatalogContext& catalogContext,
                             const QSize& imageSize,
                             const QVector<CameraPipelineStarDetection>& starDetections,
                             const QVector<int>& detectionIndices,
                             const FinalMatchPassEvaluation& winner,
                             double finalMatchRadius,
                             QString *reason,
                             FinalMatchPassEvaluation *betterAlias = nullptr)
{
    if (!m_useDirectionSeedPreference
        || m_directionSeedHasRollPreference
        || !winner.projectorValid
        || (!isNarrowField(settings))
        || (winner.finalMatches.size() < settings.m_plateSolveMinMatches)
        || !std::isfinite(winner.rmsErrorPixels))
    {
        return false;
    }

    const int competitiveMatchCount = std::max(
        settings.m_plateSolveMinMatches,
        static_cast<int>(std::ceil(static_cast<double>(winner.finalMatches.size()) * 0.85)));
    const double maxCompetitiveRms = std::max(
        winner.rmsErrorPixels + 2.0,
        winner.rmsErrorPixels * 1.15);
    const double maxCompetitiveWorstError = std::max(
        winner.maxErrorPixels + 4.0,
        winner.maxErrorPixels * 1.15);
    const std::array<double, 12> rollOffsets = {{
        -30.0, 30.0,
        -60.0, 60.0,
        -90.0, 90.0,
        -120.0, 120.0,
        -150.0, 150.0,
        180.0, -180.0
    }};
    const bool winnerHasStrongSparseAnchors =
        hasHighConfidenceSparseGuidedAnchors(settings, winner);
    const double winnerSeedConsistency =
        narrowGuidedSeedConsistencyScore(settings, winner);
    const double winnerBrightConsistency =
        narrowGuidedBrightConsistencyScore(settings, winner);
    const double winnerFinalScore = std::max(
        finalMatchPassScore(settings, winner),
        finalMatchPassEvidenceScore(settings, winner));
    const double winnerLogOdds = poseFalseAlarmLogOdds(
        catalogContext, winner, imageSize, finalMatchRadius, static_cast<int>(starDetections.size()));

    // Adoption: if a roll alias matches the rare *bright* stars far better than the
    // winner (much higher bright-weighted log-odds), it is almost certainly the true
    // pose and the winner is a faint-coincidence look-alike. Rather than reject as
    // ambiguous, hand that alias back so the caller adopts it. Tracked across all roll
    // offsets so the best such alias wins, and preferred over an ambiguity rejection.
    constexpr double kRollAdoptLogOddsMargin = 15.0;
    FinalMatchPassEvaluation adoptableAlias;
    double adoptableAliasLogOdds = -std::numeric_limits<double>::infinity();
    bool haveAdoptableAlias = false;
    bool ambiguousAliasFound = false;

    for (double rollOffset : rollOffsets)
    {
        Evaluation competitorPose = winner.pose;
        competitorPose.anchored = false;
        competitorPose.sparseGuidedPair = false;
        competitorPose.guidedTriangle = false;
        competitorPose.anchorDetectionIndex = -1;
        competitorPose.anchorCatalogIndex = -1;
        competitorPose.secondaryAnchorDetectionIndex = -1;
        competitorPose.secondaryAnchorCatalogIndex = -1;
        competitorPose.tertiaryAnchorDetectionIndex = -1;
        competitorPose.tertiaryAnchorCatalogIndex = -1;
        competitorPose.rollDegrees = normalizeDegrees(winner.pose.rollDegrees + rollOffset);
        competitorPose.matches.clear();
        competitorPose.matchCount = 0;
        competitorPose.rmsErrorPixels = std::numeric_limits<double>::infinity();

        if (angularDistanceDegrees(competitorPose.rollDegrees, winner.pose.rollDegrees) < 1.0) {
            continue;
        }

        const FinalMatchPassEvaluation competitor = evaluateFinalMatchPass(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            detectionIndices,
            competitorPose,
            finalMatchRadius);
        if (!competitor.projectorValid
            || (competitor.finalMatches.size() < competitiveMatchCount)
            || !std::isfinite(competitor.rmsErrorPixels)
            || (competitor.rmsErrorPixels > maxCompetitiveRms)
            || (competitor.maxErrorPixels > maxCompetitiveWorstError)
            || !hasGeometricallyConsistentMatches(
                starDetections,
                competitor.projectedStars,
                competitor.finalMatches,
                finalMatchRadius)
            || !isAcceptableDirectionSeedSolve(
                settings,
                starDetections,
                competitor.finalMatches,
                competitor.rmsErrorPixels,
                competitor.maxErrorPixels)
            || !hasAcceptableGuidedFinalBrightnessConsistency(settings, competitor))
        {
            continue;
        }

        if (winnerHasStrongSparseAnchors
            && !hasHighConfidenceSparseGuidedAnchors(settings, competitor))
        {
            continue;
        }

        // Bright-weighted log-odds tie-break. Roll ambiguity in deep fields is a
        // false-alarm problem: many rolls match hundreds of *faint* stars at similar
        // count/RMS, so those statistics cannot tell the correct roll apart. The
        // false-alarm log-odds weights each match by the rarity of its catalog star
        // (-log density), so the rare *bright* stars dominate -- whichever roll lands
        // them on real detections wins decisively. If the winner's log-odds clearly
        // beats this alias, the alias is a faint-coincidence look-alike, not a genuine
        // competitor, so dismiss it. This overrides the count/RMS-based
        // aliasIsStrictlyBetterFit below, which is precisely what gets fooled in deep
        // fields (e.g. galaxy-m101 @maxMag15). It cannot create a false positive: a
        // wrong winner matching only faint coincidences has *lower* log-odds than a
        // correct bright-matching alias, so it never wins this comparison.
        const double competitorLogOdds = poseFalseAlarmLogOdds(
            catalogContext, competitor, imageSize, finalMatchRadius, static_cast<int>(starDetections.size()));
        constexpr double kRollLogOddsMargin = 5.0;
        if (winnerLogOdds >= (competitorLogOdds + kRollLogOddsMargin))
        {
            qDebug() << "CameraPlateSolver: ignoring roll alias with weaker bright-weighted log-odds"
                     << "winnerRoll" << winner.pose.rollDegrees << "winnerLogOdds" << winnerLogOdds
                     << "aliasRoll" << competitor.pose.rollDegrees << "aliasLogOdds" << competitorLogOdds;
            continue;
        }
        // Adopt direction: this alias matches the rare bright stars far better than the
        // winner -> it is the true pose. Record the best such alias (do not reject).
        if (competitorLogOdds >= (winnerLogOdds + kRollAdoptLogOddsMargin))
        {
            if (!haveAdoptableAlias || (competitorLogOdds > adoptableAliasLogOdds))
            {
                adoptableAlias = competitor;
                adoptableAliasLogOdds = competitorLogOdds;
                haveAdoptableAlias = true;
            }
            continue;
        }

        const double competitorSeedConsistency =
            narrowGuidedSeedConsistencyScore(settings, competitor);
        const double competitorBrightConsistency =
            narrowGuidedBrightConsistencyScore(settings, competitor);
        const double competitorFinalScore = std::max(
            finalMatchPassScore(settings, competitor),
            finalMatchPassEvidenceScore(settings, competitor));
        const double seedConsistencyDelta =
            winnerSeedConsistency - competitorSeedConsistency;
        const double brightConsistencyDelta =
            winnerBrightConsistency - competitorBrightConsistency;
        const auto winnerScoreAtLeastFraction = [&](double fraction) {
            if ((std::fabs(winnerFinalScore) < 1e-9)
                && (std::fabs(competitorFinalScore) < 1e-9))
            {
                return true;
            }
            return winnerFinalScore >= (competitorFinalScore * fraction);
        };
        const auto winnerScoreStrictlyBetter = [&](double factor) {
            if ((std::fabs(winnerFinalScore) < 1e-9)
                && (std::fabs(competitorFinalScore) < 1e-9))
            {
                return false;
            }
            return winnerFinalScore >= (competitorFinalScore * factor);
        };

        // When the roll alias is a strictly better geometric fit than the winner --
        // a comparable number of final matches but a meaningfully lower RMS -- the
        // winner is almost certainly the wrong pose, and the soft seed-consistency /
        // brightness / final-score heuristics below must not be allowed to dismiss the
        // alias. Doing so returns a confident wrong-roll pose: e.g. galaxy-m101 at
        // maxMag 16 returns roll ~-2.6 deg (123 matches, RMS 16.23) when the true roll
        // is ~+87 deg (120 matches, RMS 15.80) -- the alias matches nearly as many
        // stars at a clearly lower RMS, yet scores higher on the seed-consistency
        // heuristic, so the dismissal accepts the wrong pose.
        //
        // The "strictly lower RMS" requirement is what keeps this from over-rejecting
        // correct solves in ultra-dense fields. There, coincidental roll aliases can
        // tie the winner on match count, but the correct winner still has the lowest
        // RMS (e.g. galaxy-m31: winner RMS 15.87 vs nearest alias 15.95), so the guard
        // does not fire and the existing seed-consistency dismissals still apply.
        const double kAliasRmsImprovementMargin = 0.25;
        const bool aliasIsStrictlyBetterFit =
            std::isfinite(competitor.rmsErrorPixels)
            && (competitor.finalMatches.size()
                >= static_cast<qsizetype>(std::floor(static_cast<double>(winner.finalMatches.size()) * 0.95)))
            && (competitor.rmsErrorPixels <= (winner.rmsErrorPixels - kAliasRmsImprovementMargin));

        if (!plateSolveStartUsesRoll(settings) && !aliasIsStrictlyBetterFit)
        {
            const double seedConsistencyMargin =
                winner.pose.sparseGuidedPair ? 0.75 : 1.5;
            if ((seedConsistencyDelta >= seedConsistencyMargin)
                && winnerScoreAtLeastFraction(0.85))
            {
                qDebug() << "CameraPlateSolver: ignoring roll alias with weaker seed consistency"
                         << "winnerRoll" << winner.pose.rollDegrees
                         << "winnerMatches" << winner.finalMatches.size()
                         << "winnerRms" << winner.rmsErrorPixels
                         << "winnerSeed" << winnerSeedConsistency
                         << "winnerScore" << winnerFinalScore
                         << "aliasRoll" << competitor.pose.rollDegrees
                         << "aliasMatches" << competitor.finalMatches.size()
                         << "aliasRms" << competitor.rmsErrorPixels
                         << "aliasSeed" << competitorSeedConsistency
                         << "aliasScore" << competitorFinalScore;
                continue;
            }

            const double seedProjectedSupportDelta =
                winner.matchedSeedProjectedMagnitudeSupport - competitor.matchedSeedProjectedMagnitudeSupport;
            const double seedProjectedFractionDelta =
                winner.seedProjectedMagnitudeMatchFraction - competitor.seedProjectedMagnitudeMatchFraction;
            if (((seedProjectedSupportDelta >= std::max(1.5, 0.12 * std::max(
                        winner.matchedSeedProjectedMagnitudeSupport,
                        competitor.matchedSeedProjectedMagnitudeSupport)))
                    || (seedProjectedFractionDelta >= 0.035))
                && winnerScoreAtLeastFraction(0.88))
            {
                qDebug() << "CameraPlateSolver: ignoring roll alias with weaker seed-projected consistency"
                         << "winnerRoll" << winner.pose.rollDegrees
                         << "winnerMatches" << winner.finalMatches.size()
                         << "winnerRms" << winner.rmsErrorPixels
                         << "winnerSeedProjected" << winner.matchedSeedProjectedMagnitudeSupport
                         << "winnerSeedProjectedFraction" << winner.seedProjectedMagnitudeMatchFraction
                         << "winnerScore" << winnerFinalScore
                         << "aliasRoll" << competitor.pose.rollDegrees
                         << "aliasMatches" << competitor.finalMatches.size()
                         << "aliasRms" << competitor.rmsErrorPixels
                         << "aliasSeedProjected" << competitor.matchedSeedProjectedMagnitudeSupport
                         << "aliasSeedProjectedFraction" << competitor.seedProjectedMagnitudeMatchFraction
                         << "aliasScore" << competitorFinalScore;
                continue;
            }

            if ((brightConsistencyDelta >= 1.0)
                && winnerScoreAtLeastFraction(0.90))
            {
                qDebug() << "CameraPlateSolver: ignoring roll alias with weaker brightness consistency"
                         << "winnerRoll" << winner.pose.rollDegrees
                         << "winnerMatches" << winner.finalMatches.size()
                         << "winnerRms" << winner.rmsErrorPixels
                         << "winnerBright" << winnerBrightConsistency
                         << "winnerScore" << winnerFinalScore
                         << "aliasRoll" << competitor.pose.rollDegrees
                         << "aliasMatches" << competitor.finalMatches.size()
                         << "aliasRms" << competitor.rmsErrorPixels
                         << "aliasBright" << competitorBrightConsistency
                         << "aliasScore" << competitorFinalScore;
                continue;
            }

            if (winnerScoreStrictlyBetter(1.12))
            {
                qDebug() << "CameraPlateSolver: ignoring roll alias with weaker final score"
                         << "winnerRoll" << winner.pose.rollDegrees
                         << "winnerMatches" << winner.finalMatches.size()
                         << "winnerRms" << winner.rmsErrorPixels
                         << "winnerScore" << winnerFinalScore
                         << "aliasRoll" << competitor.pose.rollDegrees
                         << "aliasMatches" << competitor.finalMatches.size()
                         << "aliasRms" << competitor.rmsErrorPixels
                         << "aliasScore" << competitorFinalScore;
                continue;
            }
        }

        if (reason)
        {
            *reason = QStringLiteral("roll alias at %1 deg: matches=%2/%3 RMS=%4/%5 seed=%6/%7 score=%8/%9")
                .arg(competitor.pose.rollDegrees, 0, 'f', 2)
                .arg(competitor.finalMatches.size())
                .arg(winner.finalMatches.size())
                .arg(competitor.rmsErrorPixels, 0, 'f', 2)
                .arg(winner.rmsErrorPixels, 0, 'f', 2)
                .arg(competitorSeedConsistency, 0, 'f', 2)
                .arg(winnerSeedConsistency, 0, 'f', 2)
                .arg(competitorFinalScore, 0, 'f', 2)
                .arg(winnerFinalScore, 0, 'f', 2);
        }
        qDebug() << "CameraPlateSolver: rejecting ambiguous direction-seeded solution"
                 << "winnerRoll" << winner.pose.rollDegrees
                 << "winnerMatches" << winner.finalMatches.size()
                 << "winnerRms" << winner.rmsErrorPixels
                 << "winnerSeed" << winnerSeedConsistency
                 << "winnerBright" << winnerBrightConsistency
                 << "winnerScore" << winnerFinalScore
                 << "aliasRoll" << competitor.pose.rollDegrees
                 << "aliasMatches" << competitor.finalMatches.size()
                 << "aliasRms" << competitor.rmsErrorPixels
                 << "aliasSeed" << competitorSeedConsistency
                 << "aliasBright" << competitorBrightConsistency
                 << "aliasScore" << competitorFinalScore;
        // Genuinely ambiguous on count/RMS. Don't reject yet -- keep scanning, because
        // a later roll offset may be an adoptable bright-better alias, which is the
        // correct resolution and takes precedence over an ambiguity rejection.
        ambiguousAliasFound = true;
    }

    // A bright-better alias decisively wins (matches the rare bright stars) -> adopt it
    // rather than rejecting. Takes precedence over a count/RMS ambiguity.
    if (haveAdoptableAlias)
    {
        if (betterAlias) {
            *betterAlias = adoptableAlias;
        }
        qDebug() << "CameraPlateSolver: adopting bright-better roll alias"
                 << "winnerRoll" << winner.pose.rollDegrees << "winnerLogOdds" << winnerLogOdds
                 << "aliasRoll" << adoptableAlias.pose.rollDegrees
                 << "aliasLogOdds" << adoptableAliasLogOdds
                 << "aliasMatches" << adoptableAlias.finalMatches.size();
        return false;
    }

    return ambiguousAliasFound;
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
    const bool useStartFov = plateSolveStartUsesFov(settings);
    const bool useStartDirection = plateSolveStartUsesDirection(settings);
    const bool useStartRoll = plateSolveStartUsesRoll(settings);
    const bool useStartLens = plateSolveStartUsesLens(settings);
    const bool useWidePlateSolve = isWidePlateSolveContext(settings);
    const bool useFaintNarrowAnchors = useStartDirection
        && !useWidePlateSolve
        && (isNarrowField(settings));
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
    const bool calibratePrincipalPoint = canCalibratePrincipalPoint(settings);
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
        : (useStartFov && useWideWeakAnchorSearch)
            ? QVector<double>{0.96, 1.0, 1.04}
            : QVector<double>{0.88, 0.96, 1.0, 1.04, 1.12};
    const int anchorLimit = std::min(
        useWideWeakAnchorSearch ? 24
            : useFaintNarrowAnchors ? 48
            : useDenseWideGuidedDirection ? 8
            : (useStartDirection && useWidePlateSolve && (starDetections.size() <= 16)) ? 4
            : 12,
        static_cast<int>(anchors.size()));
    const int expandedRollMinMatches = std::max(3, settings.m_plateSolveMinMatches - 1);
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

                        if (!useWideWeakAnchorSearch || (localBest.matchCount >= 2))
                        {
                            localBest = refineGuidedAnchorSeedWithLm(
                                settings,
                                catalogContext,
                                imageSize,
                                captureDateTimeUtc,
                                starDetections,
                                anchoredDetectionIndices,
                                allowedCatalogIndices,
                                anchor,
                                localBest,
                                seedCenterOffsetX,
                                seedCenterOffsetY,
                                seedDistortionK1,
                                anchorMatchRadius,
                                !isNarrowField(settings));
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
                        && useStartLens;
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
    const bool denseNarrowDirectionSolve = useStartDirection
        && !useStartRoll
        && !useWidePlateSolve
        && (isNarrowField(settings))
        && (starDetections.size() > kMaxDetectionsForSolve * 2);
    if (!best.valid || (best.matchCount < expandedRollMinMatches) || denseNarrowDirectionSolve) {
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
    const int effectiveMinPoolMatchCount = candidate.sparseGuidedPair
        ? std::max(2, minPoolMatchCount)
        : std::max(3, minPoolMatchCount);
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
    if (candidates.size() > maxCandidates)
    {
        QVector<Evaluation> trimmedCandidates;
        trimmedCandidates.reserve(candidates.size());
        for (const Evaluation& existing : candidates)
        {
            if (existing.sparseGuidedPair || existing.guidedTriangle) {
                trimmedCandidates.append(existing);
            }
        }
        for (const Evaluation& existing : candidates)
        {
            if (existing.sparseGuidedPair || existing.guidedTriangle) {
                continue;
            }
            if (trimmedCandidates.size() >= maxCandidates) {
                break;
            }
            trimmedCandidates.append(existing);
        }
        if (trimmedCandidates.isEmpty()) {
            candidates.resize(maxCandidates);
        } else {
            candidates = std::move(trimmedCandidates);
        }
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
        Evaluation rescored = evaluatePose(
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
        rescored.anchored = candidate.anchored;
        rescored.sparseGuidedPair = candidate.sparseGuidedPair;
        rescored.guidedTriangle = candidate.guidedTriangle;
        rescored.anchorDetectionIndex = candidate.anchorDetectionIndex;
        rescored.anchorCatalogIndex = candidate.anchorCatalogIndex;
        rescored.secondaryAnchorDetectionIndex = candidate.secondaryAnchorDetectionIndex;
        rescored.secondaryAnchorCatalogIndex = candidate.secondaryAnchorCatalogIndex;
        rescored.tertiaryAnchorDetectionIndex = candidate.tertiaryAnchorDetectionIndex;
        rescored.tertiaryAnchorCatalogIndex = candidate.tertiaryAnchorCatalogIndex;
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
    if (isCancellationRequested()) {
        return best;
    }
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
    const bool skipNarrowFovOnlyFullSkyGrids = useStartFov
        && !useStartElevation
        && !useStartDirection
        && !isWidePlateSolveContext(settings)
        && (settings.m_fov < 15.0);
    const int maxMultiHypothesisCandidates = (useStartDirection && (isNarrowField(settings)))
        ? (useStartRoll ? 24 : 256)
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
    const double guidedSeedMatchRadius = (useStartDirection && (isNarrowField(settings)))
        ? std::max(finalMatchRadius,
            std::min(96.0, std::max(finalMatchRadius, static_cast<double>(settings.m_plateSolveMatchRadius)) * 4.0))
        : finalMatchRadius;
    const double searchMatchRadiusOverride = useStartDirection
        ? guidedSeedMatchRadius
        : useWideFovSeedRadius ? wideFovSeedMatchRadius
        : -1.0;
    if (useStartDirection && (isNarrowField(settings))) {
        m_weakModeNormalizationPixels = std::max(m_weakModeNormalizationPixels, guidedSeedMatchRadius);
    }
    if (useWideFovSeedRadius) {
        m_weakModeNormalizationPixels = std::max(m_weakModeNormalizationPixels, wideFovSeedMatchRadius);
    }
    if (useWideBlindSeedRadius) {
        m_weakModeNormalizationPixels = std::max(m_weakModeNormalizationPixels, wideBlindSeedMatchRadius);
    }
    const double coarseSearchRadius = std::max(0.0, settings.m_plateSolveSearchRadius);
    const double coarseRollRadius = useStartRoll
        ? std::max(15.0, std::min(45.0, static_cast<double>(settings.m_fov) * 0.20))
        : std::max(4.0, std::min(20.0, static_cast<double>(settings.m_fov) * 0.20));
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
    QVector<int> narrowGuidedFirstPassCatalogIndices;
    const QVector<int>* guidedFirstPassCatalogIndices = nullptr;
    if (useStartDirection && (isNarrowField(settings)))
    {
        const double localRadiusDegrees = std::max(
            static_cast<double>(settings.m_plateSolveSearchRadius) + static_cast<double>(settings.m_fov) * 2.0,
            static_cast<double>(settings.m_fov) * 4.0);
        const QVector<VisibleCatalogStar> localBrightStars = selectLocalVisibleStars(
            catalogContext.visibleStars,
            settings.m_azimuth,
            settings.m_elevation,
            localRadiusDegrees,
            256);
        const double firstPassMaxMagnitude = std::min(
            static_cast<double>(settings.m_plateSolveMaxMagnitude),
            kNarrowGuidedBrightCatalogMaxMagnitude);
        narrowGuidedFirstPassCatalogIndices.reserve(localBrightStars.size());
        for (const VisibleCatalogStar& star : localBrightStars)
        {
            if (star.magnitude > firstPassMaxMagnitude) {
                break;
            }
            narrowGuidedFirstPassCatalogIndices.append(star.catalogIndex);
        }
        if (narrowGuidedFirstPassCatalogIndices.size() >= std::max(2, minMatchCount - 1)) {
            guidedFirstPassCatalogIndices = &narrowGuidedFirstPassCatalogIndices;
        }
    }

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
        if (isCancellationRequested()) {
            return;
        }
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
                                     double matchRadiusPixels,
                                     bool promoteSparseGuidedPair = true) {
        if (isCancellationRequested()) {
            return;
        }
        const Evaluation candidate = evaluatePoseFromPrecomputedCatalog(
            settings, catalogContext, starDetections, detectionIndices,
            azimuthDegrees, elevationDegrees, rollDegrees, fovDegrees,
            fixedCenterOffsetX, fixedCenterOffsetY, fixedDistortionK1,
            matchRadiusPixels);
        logPlateSolveEvaluation(stage, candidate);
        if (keepMultipleCandidates) {
            if (promoteSparseGuidedPair)
            {
                const Evaluation promotedSparseGuidedPair = promoteSparseGuidedPairFromMatches(
                    settings,
                    catalogContext,
                    starDetections,
                    candidate);
                if (promotedSparseGuidedPair.valid)
                {
                    const bool alreadyQueued = std::any_of(candidatePool->cbegin(), candidatePool->cend(), [&promotedSparseGuidedPair](const Evaluation& existing) {
                        return sameEvaluationIdentity(existing, promotedSparseGuidedPair);
                    });
                    if (!alreadyQueued) {
                        candidatePool->append(promotedSparseGuidedPair);
                    }
                }
            }
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
        const double fovDegrees = std::max(0.01, static_cast<double>(settings.m_fov));
        for (double value : {
            fineStep,
            fineStep * 2.0,
            fineStep * 4.0,
            fovStep,
            fovStep * 2.0,
            fovDegrees * 0.5,
            fovDegrees * 0.75,
            fovDegrees,
            fovDegrees * 1.25,
            fovDegrees * 1.5,
            fovDegrees * 2.0,
            radius * 0.5,
            radius})
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

        const double step = (isNarrowField(settings)) ? 10.0
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
        const bool allowGuidedEarlyStop = (settings.m_fov < kWideFovMagnitudePreferenceThresholdDegrees)
            && useStartRoll;
        const QVector<double> directionOffsets = buildGuidedDirectionOffsets(coarseSearchRadius);
        const QVector<double> rollOffsets = buildGuidedRollOffsets();
        for (double fovFactor : coarseFovOffsetsOrdered)
        {
            if (isCancellationRequested()) break;
            if (guidedSatisfied) break;
            for (double elOffset : directionOffsets)
            {
                if (isCancellationRequested()) break;
                if (guidedSatisfied) break;
                for (double azOffset : directionOffsets)
                {
                    if (isCancellationRequested()) break;
                    if (guidedSatisfied) break;
                    const double azimuthDegrees = settings.m_azimuth + azOffset;
                    const double elevationDegrees = settings.m_elevation + elOffset;
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
                        guidedFirstPassCatalogIndices);
                    for (double rollOffset : rollOffsets)
                    {
                        if (isCancellationRequested()) break;
                        const double rollDegrees = settings.m_roll + rollOffset;
                        populateBlindGridProjectedCatalog(rollDegrees, guidedSeedMatchRadius, refProjector);
                        evaluateSeedFromCache(
                            "guided-direction",
                            azimuthDegrees,
                            elevationDegrees,
                            rollDegrees,
                            fovDegrees,
                            guidedSeedMatchRadius,
                            false);
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
            if (isCancellationRequested()) break;
            for (double elFactor : coarseOffsetsOrdered)
            {
                if (isCancellationRequested()) break;
                for (double azimuthDegrees = minAzimuthDegrees; azimuthDegrees < maxAzimuthDegrees; azimuthDegrees += fovGridAzimuthStepDegrees)
                {
                    if (isCancellationRequested()) break;
                    for (double rollDegrees : fovSearchRollOffsets)
                    {
                        if (isCancellationRequested()) break;
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
                if (isCancellationRequested()) break;
                for (double elevationOffset : elevationOffsets)
                {
                    if (isCancellationRequested()) break;
                    for (double rollOffset : rollOffsets)
                    {
                        if (isCancellationRequested()) break;
                        for (double fovScale : refineFovScales)
                        {
                            if (isCancellationRequested()) break;
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
        const bool runGuidedFovGrid = !useWideFovBlindSeedFirst
            && !skipNarrowFovOnlyFullSkyGrids;
        if (runGuidedFovGrid)
        {
            for (double fovFactor : coarseFovOffsetsOrdered)
            {
                if (isCancellationRequested()) break;
                for (double elevationDegrees = minElevationDegrees; elevationDegrees <= maxElevationDegrees; elevationDegrees += fovGridElevationStepDegrees)
                {
                    if (isCancellationRequested()) break;
                    for (double azimuthDegrees = minAzimuthDegrees; azimuthDegrees < maxAzimuthDegrees; azimuthDegrees += fovGridAzimuthStepDegrees)
                    {
                        if (isCancellationRequested()) break;
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
                                if (isCancellationRequested()) break;
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
                                if (isCancellationRequested()) break;
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
        if (useStartDirection && (isNarrowField(settings)))
        {
            const double localRadiusDegrees = std::max(
                static_cast<double>(settings.m_plateSolveSearchRadius) + static_cast<double>(settings.m_fov) * 2.0,
                static_cast<double>(settings.m_fov) * 4.0);
            localVisibleStars = selectLocalVisibleStars(
                catalogContext.visibleStars,
                settings.m_azimuth,
                settings.m_elevation,
                localRadiusDegrees,
                2048);
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
                    if (seed.sparseGuidedPair)
                    {
                        const bool alreadyQueued = std::any_of(candidatePool->cbegin(), candidatePool->cend(), [&seed](const Evaluation& existing) {
                            return sameEvaluationIdentity(existing, seed);
                        });
                        if (!alreadyQueued) {
                            candidatePool->append(seed);
                        }
                    }
                    else
                    {
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
                }
                if (isBetterEvaluationForMode(seed, best, useWeakModeScoring, useGuidedDirectionScoring)) {
                    best = seed;
                    logPlateSolveEvaluation(stage, best, true);
                }
            }
        };
        auto isStrongWideWeakBlindSeed = [&](const Evaluation& candidate) {
            const double strongWideSeedRmsCap = std::min(
                std::max(static_cast<double>(settings.m_plateSolveMatchRadius) * 0.9, 2.0) * 0.60,
                14.0);
            return wideWeakMode
                && candidate.valid
                && (candidate.matchCount >= std::max(10, minMatchCount + 6))
                && (candidate.rmsErrorPixels <= strongWideSeedRmsCap);
        };
        auto hasGoodWideBlindSeed = [&]() {
            return wideWeakMode
                && ((isStrongBlindSeedEvaluation(settings, detectionIndices, best)
                        && hasAcceptableBrightnessConsistency(best))
                    || isStrongWideWeakBlindSeed(best));
        };

        const bool useBrightGuidedTriangles = useStartDirection
            && !wideWeakMode
            && (isNarrowField(settings));

        qint64 seedStageStartMs = searchProfileTimer.elapsed();
        if (isCancellationRequested()) {
            return best;
        }
        QVector<Evaluation> brightTriangleSeeds;
        if (useBrightGuidedTriangles)
        {
            brightTriangleSeeds = buildBrightGuidedAnchorTriangleSeeds(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                *blindVisibleStars);
            const bool haveAcceptableAnchorTriangle = std::any_of(
                brightTriangleSeeds.cbegin(),
                brightTriangleSeeds.cend(),
                [&](const Evaluation& seed) {
                    return isAcceptableDirectionSeedSolve(settings, minMatchCount, seed)
                        && hasAcceptableBrightnessConsistency(seed);
                });
            if (!haveAcceptableAnchorTriangle)
            {
                const QVector<Evaluation> ratioTriangleSeeds = buildBrightGuidedTriangleSeeds(
                    settings,
                    catalogContext,
                    imageSize,
                    captureDateTimeUtc,
                    starDetections,
                    detectionIndices,
                    *blindVisibleStars);
                brightTriangleSeeds += ratioTriangleSeeds;
            }
        }
        logSearchProfile("bright-triangle-seeds", seedStageStartMs);
        consumeBlindSeeds(brightTriangleSeeds, "bright-triangle-seed");

        const bool brightTriangleSeedAlreadyAcceptable = useBrightGuidedTriangles
            && isAcceptableDirectionSeedSolve(settings, minMatchCount, best)
            && hasAcceptableBrightnessConsistency(best);

        seedStageStartMs = searchProfileTimer.elapsed();
        if (isCancellationRequested()) {
            return best;
        }
        const QVector<Evaluation> brightPairSeeds = wideWeakMode
            ? buildBrightPairSeeds(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                *blindVisibleStars)
            : QVector<Evaluation>();
        logSearchProfile("bright-pair-seeds", seedStageStartMs);
        const bool consumeBrightPairsBeforeTriangle = wideWeakMode;
        if (consumeBrightPairsBeforeTriangle) {
            consumeBlindSeeds(brightPairSeeds, "bright-pair-seed");
        }

        const bool brightPairSeedAlreadyAcceptable = useStartDirection
            && !wideWeakMode
            && isAcceptableDirectionSeedSolve(settings, minMatchCount, best)
            && hasAcceptableBrightnessConsistency(best);
        const bool wideBrightPairSeedAlreadyAcceptable = wideWeakMode && hasGoodWideBlindSeed();
        if (brightTriangleSeedAlreadyAcceptable || brightPairSeedAlreadyAcceptable || wideBrightPairSeedAlreadyAcceptable)
        {
            recordProfileMetric(QStringLiteral("search.blindTriangleSkipped"), 1);
            recordProfileMetric(QStringLiteral("search.blindQuadSkipped"), 1);
            logSearchProfile("blind-triangle-seeds", searchProfileTimer.elapsed());
            logSearchProfile("blind-quad-seeds", searchProfileTimer.elapsed());
        }
        else
        {
            seedStageStartMs = searchProfileTimer.elapsed();
            if (isCancellationRequested()) {
                return best;
            }
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

            if (!wideWeakMode && !consumeBrightPairsBeforeTriangle) {
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
                if (isCancellationRequested()) {
                    return best;
                }
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
    const bool blindSeedVeryStrong = wideWeakMode
        && best.valid
        && (best.matchCount >= std::max(10, minMatchCount + 6))
        && (best.rmsErrorPixels <= std::min(wideFallbackRmsCap * 0.60, 14.0));
    const bool wideWeakBestLooksLikeFalseBrightMatch = wideWeakMode
        && !blindSeedVeryStrong
        && best.valid
        && ((!hasAcceptableBrightnessConsistency(best))
            || (std::isfinite(best.meanCatalogMagnitude) && (best.meanCatalogMagnitude > 3.0)));

    if (!skipNarrowFovOnlyFullSkyGrids
        && (((!best.valid || (best.matchCount < minMatchCount)) || wideWeakBestLooksLikeFalseBrightMatch)
        && (!useStartDirection || !best.valid || wideWeakBestLooksLikeFalseBrightMatch)
        && (!blindSeedAlreadyAcceptable || wideWeakBestLooksLikeFalseBrightMatch)))
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
            if (isCancellationRequested()) break;
            for (double elevationDegrees = minElevationDegrees; elevationDegrees <= maxElevationDegrees; elevationDegrees += fallbackElevationStepDegrees)
            {
                if (isCancellationRequested()) break;
                if (useStartFov)
                {
                    for (double fovScale : wideFovScales)
                    {
                        if (isCancellationRequested()) break;
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
                            if (isCancellationRequested()) break;
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
                        if (isCancellationRequested()) break;
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
                            if (isCancellationRequested()) break;
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
    if (isCancellationRequested()) {
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
        if (isCancellationRequested()) break;
        const qint64 stageStartMs = searchProfileTimer.elapsed();
        for (double azOffset : coarseFovOffsets)
        {
            if (isCancellationRequested()) break;
            for (double elOffset : coarseFovOffsets)
            {
                if (isCancellationRequested()) break;
                for (double rollOffset : coarseFovOffsets)
                {
                    if (isCancellationRequested()) break;
                    for (double fovOffset : coarseFovOffsets)
                    {
                        if (isCancellationRequested()) break;
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
        if (isCancellationRequested()) break;
        const qint64 stageStartMs = searchProfileTimer.elapsed();
        bool improved = false;
        const std::array<double, 3> refineOffsets = {{-1.0, 0.0, 1.0}};
        for (double azOffset : refineOffsets)
        {
            if (isCancellationRequested()) break;
            for (double elOffset : refineOffsets)
            {
                if (isCancellationRequested()) break;
                for (double rollOffset : refineOffsets)
                {
                    if (isCancellationRequested()) break;
                    for (double fovOffset : refineOffsets)
                    {
                        if (isCancellationRequested()) break;
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

enum PlateSolveLmParameter
{
    PlateSolveLmAzimuth = 0,
    PlateSolveLmElevation,
    PlateSolveLmRoll,
    PlateSolveLmFov,
    PlateSolveLmCenterX,
    PlateSolveLmCenterY,
    PlateSolveLmDistortionK1,
    PlateSolveLmParameterCount
};

struct PlateSolveLmPose
{
    double azimuthDegrees = 0.0;
    double elevationDegrees = 0.0;
    double rollDegrees = 0.0;
    double fovDegrees = 0.0;
    double centerOffsetXPixels = 0.0;
    double centerOffsetYPixels = 0.0;
    double distortionK1 = 0.0;
};

struct PlateSolveLmEvaluation
{
    bool valid = false;
    PlateSolveLmPose pose;
    Evaluation evaluation;
    QVector<double> residuals;
    QVector<double> pairResidualNorms;
    double robustCost = std::numeric_limits<double>::infinity();
};

static double normalizeSignedDegrees(double value)
{
    value = normalizeDegrees(value);
    if (value > 180.0) {
        value -= 360.0;
    }
    return value;
}

static PlateSolveLmPose poseFromEvaluation(const Evaluation& evaluation)
{
    PlateSolveLmPose pose;
    pose.azimuthDegrees = evaluation.azimuthDegrees;
    pose.elevationDegrees = evaluation.elevationDegrees;
    pose.rollDegrees = evaluation.rollDegrees;
    pose.fovDegrees = evaluation.fovDegrees;
    pose.centerOffsetXPixels = evaluation.centerOffsetXPixels;
    pose.centerOffsetYPixels = evaluation.centerOffsetYPixels;
    pose.distortionK1 = evaluation.distortionK1;
    return pose;
}

static void clampPlateSolveLmPose(const QSize& imageSize, PlateSolveLmPose& pose)
{
    pose.azimuthDegrees = normalizeDegrees(pose.azimuthDegrees);
    pose.elevationDegrees = std::clamp(pose.elevationDegrees, kVisibleAltitudeFloor, 90.0);
    pose.rollDegrees = normalizeSignedDegrees(pose.rollDegrees);
    pose.fovDegrees = std::clamp(
        pose.fovDegrees,
        static_cast<double>(CameraSettings::m_minFov),
        static_cast<double>(CameraSettings::m_maxFov));
    const double maxCenterOffset = std::max(1.0, static_cast<double>(std::max(imageSize.width(), imageSize.height())));
    pose.centerOffsetXPixels = std::clamp(pose.centerOffsetXPixels, -maxCenterOffset, maxCenterOffset);
    pose.centerOffsetYPixels = std::clamp(pose.centerOffsetYPixels, -maxCenterOffset, maxCenterOffset);
    pose.distortionK1 = std::clamp(pose.distortionK1, -0.75, 0.75);
}

static double plateSolveLmParameterValue(const PlateSolveLmPose& pose, PlateSolveLmParameter parameter)
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

static void addPlateSolveLmParameterDelta(const QSize& imageSize,
                                          PlateSolveLmPose& pose,
                                          PlateSolveLmParameter parameter,
                                          double delta)
{
    switch (parameter)
    {
    case PlateSolveLmAzimuth:
        pose.azimuthDegrees += delta;
        break;
    case PlateSolveLmElevation:
        pose.elevationDegrees += delta;
        break;
    case PlateSolveLmRoll:
        pose.rollDegrees += delta;
        break;
    case PlateSolveLmFov:
        pose.fovDegrees += delta;
        break;
    case PlateSolveLmCenterX:
        pose.centerOffsetXPixels += delta;
        break;
    case PlateSolveLmCenterY:
        pose.centerOffsetYPixels += delta;
        break;
    case PlateSolveLmDistortionK1:
        pose.distortionK1 += delta;
        break;
    default:
        break;
    }
    clampPlateSolveLmPose(imageSize, pose);
}

static double plateSolveLmFiniteDifferenceStep(const PlateSolveLmPose& pose, PlateSolveLmParameter parameter)
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

static double plateSolveLmMaximumStep(const QSize& imageSize,
                                      const PlateSolveLmPose& pose,
                                      PlateSolveLmParameter parameter)
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

static double robustPlateSolveLmWeight(double residualNormPixels, double thresholdPixels)
{
    if ((residualNormPixels <= thresholdPixels) || (residualNormPixels <= 1e-9)) {
        return 1.0;
    }
    return thresholdPixels / residualNormPixels;
}

static bool solveSmallLinearSystem(double matrix[PlateSolveLmParameterCount][PlateSolveLmParameterCount],
                                   double rhs[PlateSolveLmParameterCount],
                                   double solution[PlateSolveLmParameterCount],
                                   int size)
{
    for (int i = 0; i < size; ++i) {
        solution[i] = 0.0;
    }
    for (int column = 0; column < size; ++column)
    {
        int pivotRow = column;
        double pivotAbs = std::fabs(matrix[column][column]);
        for (int row = column + 1; row < size; ++row)
        {
            const double candidateAbs = std::fabs(matrix[row][column]);
            if (candidateAbs > pivotAbs)
            {
                pivotAbs = candidateAbs;
                pivotRow = row;
            }
        }
        if (pivotAbs < 1e-12) {
            return false;
        }
        if (pivotRow != column)
        {
            for (int col = column; col < size; ++col) {
                std::swap(matrix[column][col], matrix[pivotRow][col]);
            }
            std::swap(rhs[column], rhs[pivotRow]);
        }

        const double pivot = matrix[column][column];
        for (int col = column; col < size; ++col) {
            matrix[column][col] /= pivot;
        }
        rhs[column] /= pivot;

        for (int row = 0; row < size; ++row)
        {
            if (row == column) {
                continue;
            }
            const double factor = matrix[row][column];
            if (std::fabs(factor) <= 0.0) {
                continue;
            }
            for (int col = column; col < size; ++col) {
                matrix[row][col] -= factor * matrix[column][col];
            }
            rhs[row] -= factor * rhs[column];
        }
    }
    for (int i = 0; i < size; ++i) {
        solution[i] = rhs[i];
    }
    return true;
}

QVector<Match> uniqueValidMatchesForRefinement(const PlateSolveCatalogContext& catalogContext,
                                               const QVector<CameraPipelineStarDetection>& starDetections,
                                               const QVector<Match>& matches,
                                               const GuidedAnchorPair *forcedAnchor = nullptr) const
{
    QVector<Match> uniqueMatches;
    uniqueMatches.reserve(matches.size() + (forcedAnchor ? 1 : 0));
    QSet<int> matchedDetections;
    QSet<int> matchedCatalogStars;

    const auto appendIfValid = [&](int detectionIndex, int catalogIndex, double distancePixels)
    {
        if ((detectionIndex < 0)
            || (detectionIndex >= starDetections.size())
            || (catalogIndex < 0)
            || (catalogIndex >= catalogContext.catalogStars.size())
            || matchedDetections.contains(detectionIndex)
            || matchedCatalogStars.contains(catalogIndex)
            || !catalogContext.visibleStarIndexByCatalogIndex.contains(catalogIndex))
        {
            return;
        }

        matchedDetections.insert(detectionIndex);
        matchedCatalogStars.insert(catalogIndex);
        uniqueMatches.append({detectionIndex, catalogIndex, distancePixels});
    };

    if (forcedAnchor) {
        appendIfValid(forcedAnchor->detectionIndex, forcedAnchor->catalogIndex, forcedAnchor->initialDistancePixels);
    }
    for (const Match& match : matches) {
        appendIfValid(match.detectionIndex, match.catalogIndex, match.distancePixels);
    }
    return uniqueMatches;
}

static QVector<int> detectionIndicesForMatches(const QVector<Match>& matches)
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

PlateSolveLmEvaluation evaluateFixedPlateSolveLmPose(const CameraSettings& settings,
                                                     const PlateSolveCatalogContext& catalogContext,
                                                     const QSize& imageSize,
                                                     const QVector<CameraPipelineStarDetection>& starDetections,
                                                     const QVector<Match>& fixedMatches,
                                                     const QVector<int>& rankDetectionIndices,
                                                     const PlateSolveLmPose& inputPose,
                                                     double robustThresholdPixels,
                                                     const Evaluation& seedEvaluation)
{
    PlateSolveLmEvaluation lmEvaluation;
    lmEvaluation.pose = inputPose;
    clampPlateSolveLmPose(imageSize, lmEvaluation.pose);

    Evaluation& evaluation = lmEvaluation.evaluation;
    evaluation.azimuthDegrees = lmEvaluation.pose.azimuthDegrees;
    evaluation.elevationDegrees = lmEvaluation.pose.elevationDegrees;
    evaluation.rollDegrees = lmEvaluation.pose.rollDegrees;
    evaluation.fovDegrees = lmEvaluation.pose.fovDegrees;
    evaluation.centerOffsetXPixels = lmEvaluation.pose.centerOffsetXPixels;
    evaluation.centerOffsetYPixels = lmEvaluation.pose.centerOffsetYPixels;
    evaluation.distortionK1 = lmEvaluation.pose.distortionK1;
    evaluation.anchored = seedEvaluation.anchored;
    evaluation.sparseGuidedPair = seedEvaluation.sparseGuidedPair;
    evaluation.guidedTriangle = seedEvaluation.guidedTriangle;
    evaluation.anchorDetectionIndex = seedEvaluation.anchorDetectionIndex;
    evaluation.anchorCatalogIndex = seedEvaluation.anchorCatalogIndex;
    evaluation.secondaryAnchorDetectionIndex = seedEvaluation.secondaryAnchorDetectionIndex;
    evaluation.secondaryAnchorCatalogIndex = seedEvaluation.secondaryAnchorCatalogIndex;
    evaluation.tertiaryAnchorDetectionIndex = seedEvaluation.tertiaryAnchorDetectionIndex;
    evaluation.tertiaryAnchorCatalogIndex = seedEvaluation.tertiaryAnchorCatalogIndex;

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
        return lmEvaluation;
    }

    QVector<ProjectedCatalogStar> projectedStars;
    projectedStars.reserve(fixedMatches.size());
    evaluation.matches.reserve(fixedMatches.size());
    lmEvaluation.residuals.reserve(fixedMatches.size() * 2);
    lmEvaluation.pairResidualNorms.reserve(fixedMatches.size());

    double sumSquaredError = 0.0;
    double robustCost = 0.0;
    const double thresholdPixels = std::max(1.0, robustThresholdPixels);
    for (const Match& fixedMatch : fixedMatches)
    {
        if ((fixedMatch.detectionIndex < 0)
            || (fixedMatch.detectionIndex >= starDetections.size())
            || (fixedMatch.catalogIndex < 0)
            || (fixedMatch.catalogIndex >= catalogContext.catalogStars.size()))
        {
            return lmEvaluation;
        }

        const auto visibleIt = catalogContext.visibleStarIndexByCatalogIndex.constFind(fixedMatch.catalogIndex);
        if (visibleIt == catalogContext.visibleStarIndexByCatalogIndex.cend()) {
            return lmEvaluation;
        }

        const VisibleCatalogStar& visibleStar = catalogContext.visibleStars[visibleIt.value()];
        QPointF projectedPoint;
        if (!projectVector(projector, visibleStar.vector, projectedPoint)) {
            return lmEvaluation;
        }

        const QPointF delta = projectedPoint - starDetections[fixedMatch.detectionIndex].m_center;
        const double distancePixels = std::hypot(delta.x(), delta.y());
        evaluation.matches.append({fixedMatch.detectionIndex, fixedMatch.catalogIndex, distancePixels});
        projectedStars.append({fixedMatch.catalogIndex, projectedPoint, visibleStar.magnitude});
        lmEvaluation.residuals.append(delta.x());
        lmEvaluation.residuals.append(delta.y());
        lmEvaluation.pairResidualNorms.append(distancePixels);
        sumSquaredError += distancePixels * distancePixels;
        robustCost += robustPlateSolveLmWeight(distancePixels, thresholdPixels) * distancePixels * distancePixels;
    }

    evaluation.matchCount = evaluation.matches.size();
    if (evaluation.matchCount <= 0) {
        return lmEvaluation;
    }

    evaluation.rmsErrorPixels = std::sqrt(sumSquaredError / evaluation.matchCount);
    evaluation.valid = true;
    populatePoseScoringMetrics(
        settings,
        starDetections,
        rankDetectionIndices,
        projectedStars,
        catalogContext.catalogStars,
        evaluation);
    lmEvaluation.valid = true;
    lmEvaluation.robustCost = robustCost;
    return lmEvaluation;
}

Evaluation runPlateSolveLmRefinement(const CameraSettings& settings,
                                     const PlateSolveCatalogContext& catalogContext,
                                     const QSize& imageSize,
                                     const QVector<CameraPipelineStarDetection>& starDetections,
                                     const QVector<Match>& fixedMatches,
                                     const QVector<int>& rankDetectionIndices,
                                     const Evaluation& seedEvaluation,
                                     std::array<bool, PlateSolveLmParameterCount> activeParameters,
                                     double matchRadiusPixels)
{
    if (fixedMatches.isEmpty()) {
        return seedEvaluation;
    }

    const int pairCount = fixedMatches.size();
    if (pairCount < 6)
    {
        activeParameters[PlateSolveLmCenterX] = false;
        activeParameters[PlateSolveLmCenterY] = false;
        activeParameters[PlateSolveLmDistortionK1] = false;
    }
    if (pairCount < 4) {
        activeParameters[PlateSolveLmFov] = false;
    }

    auto activeParameterList = [&activeParameters]() {
        QVector<int> active;
        active.reserve(PlateSolveLmParameterCount);
        for (int parameter = 0; parameter < PlateSolveLmParameterCount; ++parameter)
        {
            if (activeParameters[parameter]) {
                active.append(parameter);
            }
        }
        return active;
    };
    QVector<int> activeParametersList = activeParameterList();
    while ((activeParametersList.size() > pairCount * 2) && !activeParametersList.isEmpty())
    {
        const int parameter = activeParametersList.takeLast();
        activeParameters[parameter] = false;
    }
    if (activeParametersList.isEmpty()) {
        return evaluateFixedPlateSolveLmPose(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            fixedMatches,
            rankDetectionIndices,
            poseFromEvaluation(seedEvaluation),
            matchRadiusPixels,
            seedEvaluation).evaluation;
    }

    PlateSolveLmPose pose = poseFromEvaluation(seedEvaluation);
    clampPlateSolveLmPose(imageSize, pose);
    PlateSolveLmEvaluation best = evaluateFixedPlateSolveLmPose(
        settings,
        catalogContext,
        imageSize,
        starDetections,
        fixedMatches,
        rankDetectionIndices,
        pose,
        matchRadiusPixels,
        seedEvaluation);
    if (!best.valid) {
        return seedEvaluation;
    }

    double lambda = 1e-3;
    for (int iteration = 0; iteration < 15; ++iteration)
    {
        const double robustThresholdPixels = std::max(
            2.0,
            std::min(matchRadiusPixels, std::max(2.0, best.evaluation.rmsErrorPixels * 2.0)));
        best = evaluateFixedPlateSolveLmPose(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            fixedMatches,
            rankDetectionIndices,
            pose,
            robustThresholdPixels,
            seedEvaluation);
        if (!best.valid) {
            return seedEvaluation;
        }

        const int residualCount = best.residuals.size();
        QVector<QVector<double>> jacobianColumns;
        jacobianColumns.reserve(activeParametersList.size());
        bool jacobianValid = true;
        for (int parameterIndex : activeParametersList)
        {
            const PlateSolveLmParameter parameter = static_cast<PlateSolveLmParameter>(parameterIndex);
            double step = plateSolveLmFiniteDifferenceStep(pose, parameter);
            PlateSolveLmPose steppedPose = pose;
            addPlateSolveLmParameterDelta(imageSize, steppedPose, parameter, step);
            auto parameterDelta = [&pose, parameter](const PlateSolveLmPose& candidatePose) {
                const double rawDelta = plateSolveLmParameterValue(candidatePose, parameter)
                    - plateSolveLmParameterValue(pose, parameter);
                return ((parameter == PlateSolveLmAzimuth) || (parameter == PlateSolveLmRoll))
                    ? normalizeSignedDegrees(rawDelta)
                    : rawDelta;
            };
            double appliedStep = parameterDelta(steppedPose);
            if (std::fabs(appliedStep) < 1e-12)
            {
                steppedPose = pose;
                addPlateSolveLmParameterDelta(imageSize, steppedPose, parameter, -step);
                appliedStep = parameterDelta(steppedPose);
            }
            if (std::fabs(appliedStep) < 1e-12)
            {
                jacobianValid = false;
                break;
            }

            const PlateSolveLmEvaluation stepped = evaluateFixedPlateSolveLmPose(
                settings,
                catalogContext,
                imageSize,
                starDetections,
                fixedMatches,
                rankDetectionIndices,
                steppedPose,
                robustThresholdPixels,
                seedEvaluation);
            if (!stepped.valid || (stepped.residuals.size() != residualCount))
            {
                jacobianValid = false;
                break;
            }

            QVector<double> column;
            column.reserve(residualCount);
            for (int residualIndex = 0; residualIndex < residualCount; ++residualIndex) {
                column.append((stepped.residuals[residualIndex] - best.residuals[residualIndex]) / appliedStep);
            }
            jacobianColumns.append(column);
        }
        if (!jacobianValid) {
            break;
        }

        double normalMatrix[PlateSolveLmParameterCount][PlateSolveLmParameterCount] = {};
        double gradient[PlateSolveLmParameterCount] = {};
        const int activeCount = activeParametersList.size();
        for (int pairIndex = 0; pairIndex < pairCount; ++pairIndex)
        {
            const double weight = robustPlateSolveLmWeight(best.pairResidualNorms[pairIndex], robustThresholdPixels);
            for (int component = 0; component < 2; ++component)
            {
                const int residualIndex = pairIndex * 2 + component;
                for (int lhs = 0; lhs < activeCount; ++lhs)
                {
                    const double weightedJacobian = weight * jacobianColumns[lhs][residualIndex];
                    gradient[lhs] += weightedJacobian * best.residuals[residualIndex];
                    for (int rhs = lhs; rhs < activeCount; ++rhs) {
                        normalMatrix[lhs][rhs] += weightedJacobian * jacobianColumns[rhs][residualIndex];
                    }
                }
            }
        }
        for (int lhs = 0; lhs < activeCount; ++lhs)
        {
            for (int rhs = 0; rhs < lhs; ++rhs) {
                normalMatrix[lhs][rhs] = normalMatrix[rhs][lhs];
            }
        }

        double dampedMatrix[PlateSolveLmParameterCount][PlateSolveLmParameterCount] = {};
        double rhs[PlateSolveLmParameterCount] = {};
        for (int row = 0; row < activeCount; ++row)
        {
            rhs[row] = -gradient[row];
            for (int col = 0; col < activeCount; ++col) {
                dampedMatrix[row][col] = normalMatrix[row][col];
            }
            dampedMatrix[row][row] += lambda * std::max(std::fabs(normalMatrix[row][row]), 1e-9);
        }

        double delta[PlateSolveLmParameterCount] = {};
        if (!solveSmallLinearSystem(dampedMatrix, rhs, delta, activeCount))
        {
            lambda = std::min(lambda * 10.0, 1e12);
            continue;
        }

        PlateSolveLmPose proposedPose = pose;
        double maxNormalizedDelta = 0.0;
        for (int i = 0; i < activeCount; ++i)
        {
            const PlateSolveLmParameter parameter = static_cast<PlateSolveLmParameter>(activeParametersList[i]);
            const double maxStep = plateSolveLmMaximumStep(imageSize, pose, parameter);
            const double clampedDelta = std::clamp(delta[i], -maxStep, maxStep);
            addPlateSolveLmParameterDelta(imageSize, proposedPose, parameter, clampedDelta);
            maxNormalizedDelta = std::max(maxNormalizedDelta, std::fabs(clampedDelta) / std::max(maxStep, 1e-9));
        }

        const PlateSolveLmEvaluation proposed = evaluateFixedPlateSolveLmPose(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            fixedMatches,
            rankDetectionIndices,
            proposedPose,
            robustThresholdPixels,
            seedEvaluation);
        if (proposed.valid && (proposed.robustCost < best.robustCost))
        {
            const double previousCost = best.robustCost;
            pose = proposed.pose;
            best = proposed;
            lambda = std::max(lambda * 0.3, 1e-9);
            if ((maxNormalizedDelta < 1e-4)
                || ((best.robustCost > 0.0)
                    && ((previousCost - best.robustCost) / std::max(previousCost, 1.0) < 1e-8)))
            {
                break;
            }
        }
        else
        {
            lambda = std::min(lambda * 10.0, 1e12);
        }
    }

    return best.evaluation;
}

Evaluation refineGuidedAnchorSeedWithLm(const CameraSettings& settings,
                                        const PlateSolveCatalogContext& catalogContext,
                                        const QSize& imageSize,
                                        const QDateTime& captureDateTimeUtc,
                                        const QVector<CameraPipelineStarDetection>& starDetections,
                                        const QVector<int>& detectionIndices,
                                        const QVector<int>& allowedCatalogIndices,
                                        const GuidedAnchorPair& anchor,
                                        const Evaluation& seedEvaluation,
                                        double centerOffsetXPixels,
                                        double centerOffsetYPixels,
                                        double distortionK1,
                                        double matchRadiusPixels,
                                        bool refineFov)
{
    QVector<Match> fixedMatches = uniqueValidMatchesForRefinement(
        catalogContext,
        starDetections,
        seedEvaluation.matches,
        &anchor);
    if (fixedMatches.isEmpty()) {
        return seedEvaluation;
    }

    std::array<bool, PlateSolveLmParameterCount> activeParameters = {{
        true,
        true,
        true,
        refineFov,
        false,
        false,
        false
    }};

    Evaluation best = seedEvaluation;
    for (int pass = 0; pass < 2; ++pass)
    {
        Evaluation lmEvaluation = runPlateSolveLmRefinement(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            fixedMatches,
            detectionIndices,
            best,
            activeParameters,
            matchRadiusPixels);
        if (isBetterGuidedAnchorEvaluation(lmEvaluation, best, matchRadiusPixels)) {
            best = lmEvaluation;
        }

        const Evaluation rematched = evaluateAnchoredPose(
            settings,
            catalogContext,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            allowedCatalogIndices,
            anchor,
            best.azimuthDegrees,
            best.elevationDegrees,
            best.rollDegrees,
            best.fovDegrees,
            centerOffsetXPixels,
            centerOffsetYPixels,
            distortionK1,
            matchRadiusPixels);
        if (isBetterGuidedAnchorEvaluation(rematched, best, matchRadiusPixels)) {
            best = rematched;
        }

        QVector<Match> rematchedFixedMatches = uniqueValidMatchesForRefinement(
            catalogContext,
            starDetections,
            best.matches,
            &anchor);
        if (rematchedFixedMatches.size() <= fixedMatches.size()) {
            break;
        }
        fixedMatches = rematchedFixedMatches;
    }

    return best;
}

QVector<Match> rebuildRefinementMatchesAtPose(const CameraSettings& settings,
                                              const PlateSolveCatalogContext& catalogContext,
                                              const QSize& imageSize,
                                              const QVector<CameraPipelineStarDetection>& starDetections,
                                              const QVector<int>& detectionIndices,
                                              const Evaluation& pose,
                                              double matchRadiusPixels,
                                              const GuidedAnchorPair *forcedAnchor = nullptr)
{
    const SkyProjector projector = createProjector(
        settings,
        imageSize,
        pose.azimuthDegrees,
        pose.elevationDegrees,
        pose.rollDegrees,
        pose.fovDegrees,
        pose.centerOffsetXPixels,
        pose.centerOffsetYPixels,
        pose.distortionK1);
    if (!projector.valid) {
        return QVector<Match>();
    }

    QVector<ProjectedCatalogStar> projectedStars = buildProjectedCatalog(
        catalogContext,
        projector,
        matchRadiusPixels);
    if (projectedStars.isEmpty()) {
        return QVector<Match>();
    }

    QVector<Match> matches = buildMatches(
        catalogContext,
        starDetections,
        detectionIndices,
        projectedStars,
        matchRadiusPixels);

    if (forcedAnchor
        && (forcedAnchor->detectionIndex >= 0)
        && (forcedAnchor->detectionIndex < starDetections.size()))
    {
        int anchorProjectedIndex = -1;
        for (int i = 0; i < projectedStars.size(); ++i)
        {
            if (projectedStars[i].catalogIndex == forcedAnchor->catalogIndex)
            {
                anchorProjectedIndex = i;
                break;
            }
        }
        if (anchorProjectedIndex >= 0)
        {
            const double anchorDistance = pointDistancePixels(
                starDetections[forcedAnchor->detectionIndex].m_center,
                projectedStars[anchorProjectedIndex].point);
            if (anchorDistance <= matchRadiusPixels)
            {
                QVector<Match> anchoredMatches;
                anchoredMatches.reserve(matches.size() + 1);
                anchoredMatches.append({
                    forcedAnchor->detectionIndex,
                    forcedAnchor->catalogIndex,
                    anchorDistance
                });
                QSet<int> matchedDetections;
                QSet<int> matchedCatalogStars;
                matchedDetections.insert(forcedAnchor->detectionIndex);
                matchedCatalogStars.insert(forcedAnchor->catalogIndex);
                for (const Match& match : matches)
                {
                    if (matchedDetections.contains(match.detectionIndex)
                        || matchedCatalogStars.contains(match.catalogIndex))
                    {
                        continue;
                    }
                    matchedDetections.insert(match.detectionIndex);
                    matchedCatalogStars.insert(match.catalogIndex);
                    anchoredMatches.append(match);
                }
                matches = anchoredMatches;
            }
        }
    }

    int outlierCount = 0;
    matches = rejectOutlierMatches(
        matches,
        settings.m_plateSolveMinMatches,
        matchRadiusPixels,
        &outlierCount);
    appendSupplementalMatches(
        starDetections,
        projectedStars,
        matchRadiusPixels,
        &detectionIndices,
        m_useDirectionSeedPreference && (isNarrowField(settings)),
        matches);
    appendWideBrightSupplementalMatches(
        settings,
        starDetections,
        projectedStars,
        imageSize,
        matchRadiusPixels,
        matches);
    return matches;
}

Evaluation refinePoseFromMatches(const CameraSettings& settings,
                                 const PlateSolveCatalogContext& catalogContext,
                                 const QSize& imageSize,
                                 const QDateTime& captureDateTimeUtc,
                                 const QVector<CameraPipelineStarDetection>& starDetections,
                                 const Evaluation& initialEvaluation)
{
    Q_UNUSED(captureDateTimeUtc)

    if (!initialEvaluation.valid || initialEvaluation.matches.isEmpty()) {
        return initialEvaluation;
    }
    const bool calibrateLens = canCalibrateLens(settings);
    const bool calibratePrincipalPoint = canCalibratePrincipalPoint(settings);
    const bool useGuidedDirectionScoring = plateSolveStartUsesDirection(settings);

    const QVector<Match> inlierMatches = rejectOutlierMatches(
        initialEvaluation.matches,
        settings.m_plateSolveMinMatches,
        settings.m_plateSolveMatchRadius,
        nullptr);

    const bool forceAnchor = initialEvaluation.anchored
        && (initialEvaluation.anchorDetectionIndex >= 0)
        && (initialEvaluation.anchorCatalogIndex >= 0);
    GuidedAnchorPair forcedAnchor;
    if (forceAnchor)
    {
        forcedAnchor.detectionIndex = initialEvaluation.anchorDetectionIndex;
        forcedAnchor.catalogIndex = initialEvaluation.anchorCatalogIndex;
        for (const Match& match : initialEvaluation.matches)
        {
            if ((match.detectionIndex == forcedAnchor.detectionIndex)
                && (match.catalogIndex == forcedAnchor.catalogIndex))
            {
                forcedAnchor.initialDistancePixels = match.distancePixels;
                break;
            }
        }
    }

    Evaluation seed = initialEvaluation;
    seed.centerOffsetXPixels = calibratePrincipalPoint ? initialEvaluation.centerOffsetXPixels : settings.m_lensCenterOffsetX;
    seed.centerOffsetYPixels = calibratePrincipalPoint ? initialEvaluation.centerOffsetYPixels : settings.m_lensCenterOffsetY;
    seed.distortionK1 = calibrateLens ? initialEvaluation.distortionK1 : settings.m_lensDistortionK1;

    QVector<Match> fixedMatches = uniqueValidMatchesForRefinement(
        catalogContext,
        starDetections,
        inlierMatches,
        forceAnchor ? &forcedAnchor : nullptr);
    if (fixedMatches.isEmpty()) {
        return seed;
    }

    QVector<int> rankDetectionIndices = detectionIndicesForMatches(fixedMatches);
    std::array<bool, PlateSolveLmParameterCount> activeParameters = {{
        true,
        true,
        true,
        !(useGuidedDirectionScoring && isNarrowField(settings)),
        calibratePrincipalPoint,
        calibratePrincipalPoint,
        calibrateLens
    }};

    Evaluation best = runPlateSolveLmRefinement(
        settings,
        catalogContext,
        imageSize,
        starDetections,
        fixedMatches,
        rankDetectionIndices,
        seed,
        activeParameters,
        static_cast<double>(settings.m_plateSolveMatchRadius));
    if (!best.valid) {
        best = seed;
    }

    QVector<int> expandedDetectionIndices = selectDetectionIndicesForSolve(starDetections, imageSize);
    QSet<int> expandedDetectionSet;
    expandedDetectionSet.reserve(expandedDetectionIndices.size() + rankDetectionIndices.size());
    for (int detectionIndex : expandedDetectionIndices) {
        expandedDetectionSet.insert(detectionIndex);
    }
    for (int detectionIndex : rankDetectionIndices)
    {
        if (!expandedDetectionSet.contains(detectionIndex))
        {
            expandedDetectionSet.insert(detectionIndex);
            expandedDetectionIndices.append(detectionIndex);
        }
    }

    const double finalPassRadius = std::min(
        static_cast<double>(settings.m_plateSolveMatchRadius),
        std::max(1.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius)));
    const double intermediatePassRadius = std::min(
        static_cast<double>(settings.m_plateSolveMatchRadius),
        std::max(finalPassRadius + 10.0, finalPassRadius * 2.0));

    const QVector<Match> rebuiltMatches = rebuildRefinementMatchesAtPose(
        settings,
        catalogContext,
        imageSize,
        starDetections,
        expandedDetectionIndices,
        best,
        intermediatePassRadius,
        forceAnchor ? &forcedAnchor : nullptr);
    if (!rebuiltMatches.isEmpty())
    {
        QVector<Match> rebuiltFixedMatches = uniqueValidMatchesForRefinement(
            catalogContext,
            starDetections,
            rebuiltMatches,
            forceAnchor ? &forcedAnchor : nullptr);
        if (rebuiltFixedMatches.size() >= fixedMatches.size())
        {
            Evaluation expandedSeed = best;
            expandedSeed.matches = rebuiltFixedMatches;
            expandedSeed.matchCount = rebuiltFixedMatches.size();
            const Evaluation expandedBest = runPlateSolveLmRefinement(
                settings,
                catalogContext,
                imageSize,
                starDetections,
                rebuiltFixedMatches,
                expandedDetectionIndices,
                expandedSeed,
                activeParameters,
                intermediatePassRadius);
            if (isBetterEvaluationForMode(expandedBest, best, false, useGuidedDirectionScoring)) {
                best = expandedBest;
            }
        }
    }

    const int retainedMatchThreshold = forceAnchor
        ? std::max(2, std::min(settings.m_plateSolveMinMatches, best.matchCount))
        : minimumRetainedMatchesForFinalPass(best, settings.m_plateSolveMinMatches);
    const FinalMatchPassEvaluation finalPass = evaluateFinalMatchPass(
        settings,
        catalogContext,
        imageSize,
        starDetections,
        expandedDetectionIndices,
        best,
        finalPassRadius);
    if (finalPass.projectorValid && (finalPass.finalMatches.size() >= retainedMatchThreshold))
    {
        Evaluation finalEvaluation = best;
        finalEvaluation.matches = finalPass.finalMatches;
        finalEvaluation.matchCount = finalPass.finalMatches.size();
        finalEvaluation.rmsErrorPixels = finalPass.rmsErrorPixels;
        finalEvaluation.brightnessRankError = finalPass.brightnessRankError;
        finalEvaluation.meanCatalogMagnitude = finalPass.meanCatalogMagnitude;
        finalEvaluation.valid = true;
        if (isBetterFinalPassEvaluation(finalEvaluation, best, retainedMatchThreshold, useGuidedDirectionScoring)
            || (finalEvaluation.matchCount >= best.matchCount))
        {
            best = finalEvaluation;
        }
    }

    if (!calibratePrincipalPoint)
    {
        best.centerOffsetXPixels = settings.m_lensCenterOffsetX;
        best.centerOffsetYPixels = settings.m_lensCenterOffsetY;
    }
    if (!calibrateLens)
    {
        best.distortionK1 = settings.m_lensDistortionK1;
    }

    return best;
}

// Sharpen a narrow-field final pass by progressively shrinking the match radius and
// re-fitting on the tight inlier core. The full match radius (e.g. 24 px) admits many
// coincidental associations in dense catalogs, which bias the least-squares pose and
// inflate residuals to a uniform ~r/2 floor that has nothing to do with astrometric
// accuracy (it is matching contamination, not distortion). Re-matching at a tighter
// radius and re-fitting drives the pose from the astrometric core, which both
// stabilises edge-star associations and collapses real-match residuals toward the
// true precision -- the latter being what lets a statistical verifier separate real
// alignments from chance.
//
// Safe by construction: the tightened pose is adopted only if, re-evaluated at the
// full radius, it keeps a comparable match count and a strictly-not-worse RMS. Any
// drift to a worse local optimum is discarded and the original pose is returned.
FinalMatchPassEvaluation tightenNarrowFinalPass(const CameraSettings& settings,
                                                const PlateSolveCatalogContext& catalogContext,
                                                const QSize& imageSize,
                                                const QVector<CameraPipelineStarDetection>& starDetections,
                                                const QVector<int>& detectionIndices,
                                                const FinalMatchPassEvaluation& original,
                                                double finalMatchRadius)
{
    const int minMatches = std::max(4, settings.m_plateSolveMinMatches);
    if (!original.projectorValid
        || !isNarrowField(settings)
        || (original.finalMatches.size() < minMatches))
    {
        return original;
    }

    Evaluation pose = original.pose;
    for (const double fraction : {0.5, 0.33})
    {
        const double tightRadius = std::max(4.0, finalMatchRadius * fraction);
        const FinalMatchPassEvaluation tightPass = evaluateFinalMatchPass(
            settings, catalogContext, imageSize, starDetections, detectionIndices, pose, tightRadius);
        if (!tightPass.projectorValid || (tightPass.finalMatches.size() < minMatches)) {
            break;
        }
        Evaluation tightSeed = pose;
        tightSeed.valid = true;
        tightSeed.matches = tightPass.finalMatches;
        tightSeed.matchCount = static_cast<int>(tightPass.finalMatches.size());
        const Evaluation refined = refinePoseFromMatches(
            settings, catalogContext, imageSize, QDateTime(), starDetections, tightSeed);
        if (refined.valid) {
            pose = refined;
        }
    }

    const FinalMatchPassEvaluation tightened = evaluateFinalMatchPass(
        settings, catalogContext, imageSize, starDetections, detectionIndices, pose, finalMatchRadius);
    if (tightened.projectorValid
        && std::isfinite(tightened.rmsErrorPixels)
        && (tightened.finalMatches.size()
            >= static_cast<qsizetype>(std::floor(static_cast<double>(original.finalMatches.size()) * 0.9)))
        && (tightened.rmsErrorPixels <= original.rmsErrorPixels))
    {
        return tightened;
    }
    return original;
}

CandidateRefinementResult refineMultiHypothesisCandidate(const CameraSettings& settings,
                                                         const PlateSolveCatalogContext& catalogContext,
                                                         const QSize& imageSize,
                                                         const QDateTime& captureDateTimeUtc,
                                                         const QVector<CameraPipelineStarDetection>& starDetections,
                                                         const QVector<int>& detectionIndices,
                                                         const QVector<int>& allDetectionIndices,
                                                         const Evaluation& candidate,
                                                         int weakModeRefineMinMatches,
                                                         double finalMatchRadius,
                                                         bool rankFinalPassWithSelectedDetections,
                                                         bool useStartDirection)
{
    CandidateRefinementResult result;
    const int candidateRefineMinMatches = (candidate.anchored || candidate.sparseGuidedPair)
        ? 2
        : weakModeRefineMinMatches;
    if (!candidate.valid || (candidate.matchCount < candidateRefineMinMatches)) {
        return result;
    }

    if (candidate.sparseGuidedPair || candidate.guidedTriangle)
    {
        result.seedFinalPassEvaluation = evaluateFinalMatchPass(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            allDetectionIndices,
            candidate,
            finalMatchRadius);
        result.seedFinalPassEvaluated = true;
    }

    result.refinedCandidate = refinePoseFromMatches(
        settings,
        catalogContext,
        imageSize,
        captureDateTimeUtc,
        starDetections,
        candidate);
    result.refinedCandidateEvaluated = true;
    if (!result.refinedCandidate.valid
        || (result.refinedCandidate.matchCount < candidateRefineMinMatches))
    {
        return result;
    }

    // For the narrow dense-field case the full-detection pass overrides the
    // selected-detection pass whenever it is valid (the common case), so compute the
    // full pass first and only fall back to the selected-detection pass when needed.
    if (rankFinalPassWithSelectedDetections
        && useStartDirection
        && (isNarrowField(settings)))
    {
        result.finalPassEvaluation = evaluateFinalMatchPass(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            allDetectionIndices,
            result.refinedCandidate,
            finalMatchRadius);
        if (!result.finalPassEvaluation.projectorValid)
        {
            result.finalPassEvaluation = evaluateFinalMatchPass(
                settings,
                catalogContext,
                imageSize,
                starDetections,
                detectionIndices,
                result.refinedCandidate,
                finalMatchRadius,
                rankFinalPassWithSelectedDetections);
        }
    }
    else
    {
        result.finalPassEvaluation = evaluateFinalMatchPass(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            rankFinalPassWithSelectedDetections ? detectionIndices : allDetectionIndices,
            result.refinedCandidate,
            finalMatchRadius,
            rankFinalPassWithSelectedDetections);
    }
    result.finalPassEvaluated = true;
    return result;
}

static void clearSolvedStars(QVector<CameraPipelineStarDetection>& starDetections)
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

QString CameraPlateSolver::sirilAstroCatalogPath()
{
    SolverContext context;
    return QDir(context.downloadedCatalogDir()).filePath(QString::fromUtf8(SolverContext::kSirilAstroFileName));
}

QString CameraPlateSolver::sirilAstroCompressedCatalogPath()
{
    SolverContext context;
    return QDir(context.downloadedCatalogDir()).filePath(QString::fromUtf8(SolverContext::kSirilAstroCompressedFileName));
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
    auto finishCancelled = [&]() {
        clearSolvedStars(starDetections);
        result.m_solved = false;
        result.m_failureReason = QStringLiteral("plate solving cancelled");
        result.m_profileSummary = profileSummary();
        PROFILER_STOP(QString("%1: cancelled").arg(__FUNCTION__));
        return result;
    };
    if (isCancellationRequested()) {
        return finishCancelled();
    }
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
    m_directionSeedAzElScaleDegrees = (isNarrowField(settings))
        ? std::max(
            2.0,
            std::min(
                static_cast<double>(settings.m_plateSolveSearchRadius) * 0.35,
                static_cast<double>(settings.m_fov) * 2.0))
        : std::max(2.0, static_cast<double>(settings.m_plateSolveSearchRadius) * 0.35);
    m_directionSeedRollScaleDegrees = std::max(15.0, std::min(45.0, static_cast<double>(settings.m_fov) * 0.15));
    m_directionSeedFovScaleDegrees = (isNarrowField(settings))
        ? std::max(0.3, static_cast<double>(settings.m_fov) * 0.25)
        : std::max(2.0, static_cast<double>(settings.m_fov) * 0.08);
    m_directionSeedMinMatchCount = useStartDirection
        ? minimumDirectionSeedAcceptedMatches(settings, starDetections)
        : std::max(1, settings.m_plateSolveMinMatches);
    m_useFovSeedPreference = useStartFov;
    m_fovSeedReferenceDegrees = settings.m_fov;
    m_fovSeedScaleDegrees = (isNarrowField(settings))
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
    const QDateTime solveDateTime = settings.m_plateSolveUseCaptureDateTime
        ? captureDateTime
        : configuredSolveDateTime;
    const QDateTime captureDateTimeUtc = (solveDateTime.isValid() ? solveDateTime : QDateTime::currentDateTime()).toUTC();
    const double solveMaxMagnitude = useCurrentSettingsOnly
        ? static_cast<double>(settings.m_plateSolveMaxMagnitude)
        : firstPassPlateSolveMaxMagnitude(settings);
    const double fullSearchMaxMagnitude = narrowGuidedFullSearchMaxMagnitude(settings);
    const bool useBrightFirstPassCatalog = (solveMaxMagnitude < settings.m_plateSolveMaxMagnitude)
        && useStartDirection
        && (isNarrowField(settings));
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
        useBrightFirstPassCatalog ? fullSearchMaxMagnitude : solveMaxMagnitude);
    logSolveProfile("catalog", stageStartMs);
    if (isCancellationRequested()) {
        return finishCancelled();
    }
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
            isNarrowGuidedDirectionSolve(settings),
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
            detection.m_catalogRightAscensionDegrees = catalogStar.rightAscensionDegrees;
            detection.m_catalogDeclinationDegrees = catalogStar.declinationDegrees;
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
    if (isCancellationRequested()) {
        return finishCancelled();
    }
    bool usingFullCatalogForGuidedAnchor = false;
    bool usingRequestedMaxMagnitudeCatalog = false;
    const bool useWideWeakAnchorSearch = !useStartDirection
        && isWidePlateSolveContext(settings);
    const int guidedAnchorExtraMatches = useStartDirection
        ? (isWidePlateSolveContext(settings)
            ? (starDetections.size() <= 16 ? 0 : 1)
            : 2)
        : 0;
    const int guidedAnchorSkipRequiredMatches = result.m_requiredMatches + guidedAnchorExtraMatches;
    const bool wideWeakStrongSkipCandidate = useWideWeakAnchorSearch
        && best.valid
        && (best.matchCount >= std::max(10, result.m_requiredMatches + 6))
        && (best.rmsErrorPixels <= std::min(finalMatchRadius * 0.60, 16.0));
    const bool guidedAnchorSkipBrightnessAccepted =
        (useStartDirection && isWidePlateSolveContext(settings))
        || hasAcceptableBrightnessConsistency(best)
        || wideWeakStrongSkipCandidate;
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
            const qint64 rebuildStartMs = solveProfileTimer.elapsed();
            rebuildVisibleCatalogContext(
                guidedAnchorCatalogContext,
                settings,
                captureDateTimeUtc,
                fullSearchMaxMagnitude);
            logSolveProfile("catalog.rebuild.guidedAnchor", rebuildStartMs);
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
        if (isCancellationRequested()) {
            return finishCancelled();
        }
        if (guidedAnchorBest.valid)
        {
            bool guidedAnchorMatchesActiveCatalog = guidedAnchorCatalogContextPtr == &catalogContext;
            if (guidedAnchorCatalogContextPtr != &catalogContext)
            {
                catalogContext = std::move(guidedAnchorCatalogContext);
                result.m_catalogSource = catalogContext.catalogSource;
                result.m_catalogStarsLoaded = catalogContext.catalogStars.size();
                usingFullCatalogForGuidedAnchor = true;
                usingRequestedMaxMagnitudeCatalog = fullSearchMaxMagnitude >= settings.m_plateSolveMaxMagnitude;
                guidedAnchorMatchesActiveCatalog = true;
            }
            if (guidedAnchorMatchesActiveCatalog)
            {
                const int guidedAnchorPoolLimit = useWideWeakAnchorSearch ? 64
                    : (useStartDirection && (isNarrowField(settings)) && !useStartRoll) ? 96
                    : 24;
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
    const bool rankFinalPassWithSelectedDetections = isWidePlateSolveContext(settings)
        || (useStartDirection
            && (isNarrowField(settings))
            && (starDetections.size() > kMaxDetectionsForSolve * 2));
    const int multiHypothesisCandidateLimit = (useStartDirection && (isNarrowField(settings)))
        ? (useStartRoll ? 24 : 72)
        : rankFinalPassWithSelectedDetections ? 64
        : 10;
    if (useBrightFirstPassCatalog
        && !usingFullCatalogForGuidedAnchor
        && (best.matchCount < settings.m_plateSolveMinMatches))
    {
        stageStartMs = solveProfileTimer.elapsed();
        PlateSolveCatalogContext fullCatalogContext = buildPlateSolveCatalogContext(
            settings,
            imageSize,
            captureDateTimeUtc,
            fullSearchMaxMagnitude,
            settings.m_plateSolveMaxMagnitude);
        logSolveProfile("catalog.fullCatalogRetry", stageStartMs);
        if (!fullCatalogContext.catalogStars.isEmpty()
            && ((fullCatalogContext.catalogStars.size() > catalogContext.catalogStars.size())
                || (fullCatalogContext.visibleStars.size() > catalogContext.visibleStars.size())))
        {
            QVector<Evaluation> fullCatalogCoarseCandidates;
            stageStartMs = solveProfileTimer.elapsed();
            Evaluation fullCatalogBest = searchBestPose(
                settings,
                fullCatalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                &fullCatalogCoarseCandidates);
            logSolveProfile("searchBestPose.fullCatalogRetry", stageStartMs);
            if (isCancellationRequested()) {
                return finishCancelled();
            }
            if (fullCatalogBest.valid
                && ((fullCatalogBest.matchCount > best.matchCount)
                    || !best.valid
                    || isBetterEvaluationForMode(
                        fullCatalogBest,
                        best,
                        useWeakModeScoring,
                        useStartDirection)))
            {
                catalogContext = std::move(fullCatalogContext);
                result.m_catalogSource = catalogContext.catalogSource;
                result.m_catalogStarsLoaded = catalogContext.catalogStars.size();
                coarseCandidates = std::move(fullCatalogCoarseCandidates);
                best = fullCatalogBest;
                usingFullCatalogForGuidedAnchor = true;
                usingRequestedMaxMagnitudeCatalog = fullSearchMaxMagnitude >= settings.m_plateSolveMaxMagnitude;
                selectedFinalPass = FinalMatchPassEvaluation();
                logPlateSolveEvaluation("full-catalog-retry", best, true);
            }
        }
    }
    int bestAvailableCandidateMatches = best.matchCount;
    bool hasRefinableSparseGuidedPairCandidate = best.valid
        && best.sparseGuidedPair
        && (best.matchCount >= 2);
    if (useMultiHypothesisRefine)
    {
        for (const Evaluation& candidate : coarseCandidates)
        {
            if (!candidate.valid) {
                continue;
            }
            bestAvailableCandidateMatches = std::max(bestAvailableCandidateMatches, candidate.matchCount);
            if (candidate.sparseGuidedPair && (candidate.matchCount >= 2)) {
                hasRefinableSparseGuidedPairCandidate = true;
            }
        }
    }
    if ((best.matchCount < settings.m_plateSolveMinMatches)
        && (!useMultiHypothesisRefine
            || ((bestAvailableCandidateMatches < weakModeRefineMinMatches)
                && !hasRefinableSparseGuidedPairCandidate)))
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
    bool skipMultiHypothesisRefine = false;
    if (useMultiHypothesisRefine
        && useStartDirection
        && (isNarrowField(settings))
        && best.valid)
    {
        stageStartMs = solveProfileTimer.elapsed();
        const FinalMatchPassEvaluation earlyFinalPass = evaluateFinalMatchPass(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            allDetectionIndices,
            best,
            finalMatchRadius);
        if (isStrongEarlyGuidedFinalPass(
                settings,
                starDetections,
                earlyFinalPass,
                result.m_requiredMatches,
                finalMatchRadius))
        {
            best = earlyFinalPass.pose;
            selectedFinalPass = earlyFinalPass;
            skipMultiHypothesisRefine = true;
            recordProfileMetric(QStringLiteral("solve.refineCandidatesSkipped"), 1);
            logFinalMatchPassEvaluation("final-match-pass-early-guided", selectedFinalPass, true);
        }
        logSolveProfile("earlyFinalPass", stageStartMs);
    }

    if (useMultiHypothesisRefine && !skipMultiHypothesisRefine) {
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
            if (isCancellationRequested()) {
                return finishCancelled();
            }
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
        FinalMatchPassEvaluation bestSparseGuidedPairFinalPass;
        QVector<int> refinableCandidateIndices;
        refinableCandidateIndices.reserve(rescoredCandidates.size());
        for (int candidateIndex = 0; candidateIndex < rescoredCandidates.size(); ++candidateIndex)
        {
            const Evaluation& candidate = rescoredCandidates[candidateIndex];
            const int candidateRefineMinMatches = (candidate.anchored || candidate.sparseGuidedPair)
                ? 2
                : weakModeRefineMinMatches;
            if (candidate.valid && (candidate.matchCount >= candidateRefineMinMatches)) {
                refinableCandidateIndices.append(candidateIndex);
            }
        }

        const qint64 estimatedRefinementWorkUnits = estimateRefinementWorkUnits(
            refinableCandidateIndices.size(),
            catalogContext.visibleStars.size(),
            allDetectionIndices.size());
        recordProfileMetric(
            QStringLiteral("solve.refineCandidateWorkM"),
            estimatedRefinementWorkUnits / (1000LL * 1000LL));
        recordProfileMetric(
            QStringLiteral("solve.refineCandidatesQueued"),
            refinableCandidateIndices.size());
        const int workerThreadCount = refinementWorkerThreadCount(
            refinableCandidateIndices.size(),
            estimatedRefinementWorkUnits);
        recordProfileMetric(QStringLiteral("solve.refineCandidateThreads"), workerThreadCount);
        const auto applyRefinementResult = [&](const CandidateRefinementResult& refinementResult) -> bool
        {
            if (isCancellationRequested()) {
                return false;
            }
            if (refinementResult.seedFinalPassEvaluated)
            {
                logFinalMatchPassEvaluation("final-match-pass-sparse-guided-seed", refinementResult.seedFinalPassEvaluation);
                if (isAcceptableSparseGuidedPairFinalPass(
                        settings,
                        catalogContext,
                        starDetections,
                        refinementResult.seedFinalPassEvaluation)
                    && (!bestSparseGuidedPairFinalPass.projectorValid
                        || isBetterWeakModeFinalMatchPass(
                            settings,
                            starDetections,
                            false,
                            refinementResult.seedFinalPassEvaluation,
                            bestSparseGuidedPairFinalPass)))
                {
                    bestSparseGuidedPairFinalPass = refinementResult.seedFinalPassEvaluation;
                    logFinalMatchPassEvaluation("final-match-pass-sparse-guided-pair", bestSparseGuidedPairFinalPass, true);
                }
                if (isAcceptableSparseGuidedPairFinalPass(
                        settings,
                        catalogContext,
                        starDetections,
                        refinementResult.seedFinalPassEvaluation)
                    && isBetterWeakModeFinalMatchPass(
                        settings,
                        starDetections,
                        false,
                        refinementResult.seedFinalPassEvaluation,
                        refinedBestFinalPass))
                {
                    refinedBest = refinementResult.seedFinalPassEvaluation.pose;
                    refinedBestFinalPass = refinementResult.seedFinalPassEvaluation;
                    logPlateSolveEvaluation("refine-from-matches-multi", refinedBest, true);
                    logFinalMatchPassEvaluation("final-match-pass-sparse-guided-seed", refinedBestFinalPass, true);
                }
            }
            if (refinementResult.refinedCandidateEvaluated) {
                logPlateSolveEvaluation("refine-from-matches-multi", refinementResult.refinedCandidate, false, true);
            }
            if (!refinementResult.finalPassEvaluated) {
                return true;
            }

            const FinalMatchPassEvaluation& finalPassEvaluation = refinementResult.finalPassEvaluation;
            logFinalMatchPassEvaluation("final-match-pass-multi", finalPassEvaluation);

            if (isAcceptableSparseGuidedPairFinalPass(
                    settings,
                    catalogContext,
                    starDetections,
                    finalPassEvaluation)
                && (!bestSparseGuidedPairFinalPass.projectorValid
                    || isBetterWeakModeFinalMatchPass(
                        settings,
                        starDetections,
                        false,
                        finalPassEvaluation,
                        bestSparseGuidedPairFinalPass)))
            {
                bestSparseGuidedPairFinalPass = finalPassEvaluation;
                logFinalMatchPassEvaluation("final-match-pass-sparse-guided-pair", bestSparseGuidedPairFinalPass, true);
            }

            if (isBetterWeakModeFinalMatchPass(
                    settings,
                    starDetections,
                    !useStartFov,
                    finalPassEvaluation,
                    refinedBestFinalPass))
            {
                refinedBest = refinementResult.refinedCandidate;
                refinedBestFinalPass = finalPassEvaluation;
                logPlateSolveEvaluation("refine-from-matches-multi", refinedBest, true);
                logFinalMatchPassEvaluation("final-match-pass-multi", refinedBestFinalPass, true);
            }
            return true;
        };
        if (workerThreadCount <= 1)
        {
            for (int candidateIndex : refinableCandidateIndices)
            {
                if (isCancellationRequested()) {
                    return finishCancelled();
                }
                const CandidateRefinementResult refinementResult = refineMultiHypothesisCandidate(
                    settings,
                    catalogContext,
                    imageSize,
                    captureDateTimeUtc,
                    starDetections,
                    detectionIndices,
                    allDetectionIndices,
                    rescoredCandidates[candidateIndex],
                    weakModeRefineMinMatches,
                    finalMatchRadius,
                    rankFinalPassWithSelectedDetections,
                    useStartDirection);
                if (!applyRefinementResult(refinementResult)) {
                    return finishCancelled();
                }
            }
        }
        else
        {
            // Refine every candidate with a fixed set of strided workers: worker w handles
            // candidates w, w+T, w+2T, ... Each worker builds its SolverContext once (instead of
            // once per candidate) and runs its whole stride to completion with a single
            // waitForDone, so threads stay busy with no per-batch barrier and the per-candidate
            // state-copy overhead is removed. Each candidate result is written to its own slot and
            // merged in candidate order afterwards, so the outcome is identical to the serial loop.
            const int refinableCount = static_cast<int>(refinableCandidateIndices.size());
            QVector<CandidateRefinementResult> refinementResults(refinableCount);
            QThreadPool refinementPool;
            refinementPool.setMaxThreadCount(workerThreadCount);
            for (int workerIndex = 0; workerIndex < workerThreadCount; ++workerIndex)
            {
                refinementPool.start(QRunnable::create([&, workerIndex]() {
                    SolverContext workerContext(m_owner);
                    workerContext.copySearchStateFrom(*this);
                    for (int i = workerIndex; i < refinableCount; i += workerThreadCount)
                    {
                        if (workerContext.isCancellationRequested()) {
                            break;
                        }
                        refinementResults[i] = workerContext.refineMultiHypothesisCandidate(
                            settings,
                            catalogContext,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            detectionIndices,
                            allDetectionIndices,
                            rescoredCandidates[refinableCandidateIndices[i]],
                            weakModeRefineMinMatches,
                            finalMatchRadius,
                            rankFinalPassWithSelectedDetections,
                            useStartDirection);
                    }
                }));
            }
            refinementPool.waitForDone();
            for (int i = 0; i < refinableCount; ++i)
            {
                if (!applyRefinementResult(refinementResults[i])) {
                    return finishCancelled();
                }
            }
        }

        logSolveProfile("refineCandidates", stageStartMs);

        if (refinedBest.valid) {
            const bool narrowGuidedResidualWeak =
                m_useDirectionSeedPreference
                && (isNarrowField(settings))
                && refinedBestFinalPass.projectorValid
                && (refinedBestFinalPass.rmsErrorPixels > maxDirectionSeedRmsError(settings, refinedBestFinalPass.finalMatches.size()));
            if (bestSparseGuidedPairFinalPass.projectorValid
                && (hasWeakNarrowGuidedBrightSupport(settings, refinedBestFinalPass)
                    || narrowGuidedResidualWeak)
                && isBetterWeakModeFinalMatchPass(
                    settings,
                    starDetections,
                    false,
                    bestSparseGuidedPairFinalPass,
                    refinedBestFinalPass))
            {
                refinedBest = bestSparseGuidedPairFinalPass.pose;
                refinedBestFinalPass = bestSparseGuidedPairFinalPass;
            }
            best = refinedBest;
            selectedFinalPass = refinedBestFinalPass;
        } else {
            best = refinePoseFromMatches(settings, catalogContext, imageSize, captureDateTimeUtc, starDetections, best);
            logPlateSolveEvaluation("refine-from-matches", best, true);
        }
    } else if (!skipMultiHypothesisRefine) {
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

    if (useBrightFirstPassCatalog && !usingRequestedMaxMagnitudeCatalog)
    {
        const qint64 rebuildStartMs = solveProfileTimer.elapsed();
        PlateSolveCatalogContext fullCatalogContext = catalogContext;
        rebuildVisibleCatalogContext(
            fullCatalogContext,
            settings,
            captureDateTimeUtc,
            settings.m_plateSolveMaxMagnitude);
        logSolveProfile("catalog.rebuild.finalPass", rebuildStartMs);
        if (!fullCatalogContext.catalogStars.isEmpty())
        {
            catalogContext = std::move(fullCatalogContext);
            result.m_catalogSource = catalogContext.catalogSource;
            result.m_catalogStarsLoaded = catalogContext.catalogStars.size();
            selectedFinalPass = FinalMatchPassEvaluation();
        }
    }

    // Shared "selection already strong" predicate (named-anchor / dense-guided / large
    // accepted support). Used to skip the expensive fov-pinned and roll-recovery
    // recovery grids when the current selection is already a confident solve — those
    // grids exist to rescue weak/ambiguous solves, so a strong selection cannot benefit.
    const auto selectionHasStrongSupport = [&](const FinalMatchPassEvaluation& sel) -> bool {
        if (!sel.projectorValid) {
            return false;
        }
        const double namedAnchorRmsCap = std::max(
            std::min(
                maxDirectionSeedRmsError(settings, std::max(
                    settings.m_plateSolveMinMatches,
                    static_cast<int>(sel.finalMatches.size()))) * 1.10,
                static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.75),
            static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.65);
        const bool strongNamedAnchors =
            (sel.namedBrightAnchorMatches >= 3)
            && std::isfinite(sel.namedBrightAnchorRmsErrorPixels)
            && (sel.namedBrightAnchorRmsErrorPixels <= namedAnchorRmsCap)
            && (sel.finalMatches.size() >= static_cast<qsizetype>(std::max(settings.m_plateSolveMinMatches + 6, 10)));
        const bool strongDenseGuided = hasStrongDenseNarrowGuidedFinalPass(settings, sel);
        const bool largeAcceptedDenseGuided =
            (isNarrowField(settings))
            && (sel.finalMatches.size() >= static_cast<qsizetype>(std::max(settings.m_plateSolveMinMatches + 156, 160)))
            && std::isfinite(sel.rmsErrorPixels)
            && (sel.rmsErrorPixels <= std::min(
                static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.75,
                18.0));
        return strongNamedAnchors || strongDenseGuided || largeAcceptedDenseGuided;
    };

    // Evaluate `count` independent recovery-grid poses across worker threads, each with
    // its own SolverContext (mirrors the candidate-refinement parallelism). evalFn must
    // depend only on the index and read-only shared state — never on the mutable accept
    // bookkeeping — and the caller merges results in index order, so the outcome is
    // bit-identical to the serial loop. Falls back to serial for small grids.
    const auto evaluateRecoveryPosesParallel = [&](int count, const auto& evalFn) {
        using ResultType = std::decay_t<decltype(evalFn(*this, 0))>;
        QVector<ResultType> results(std::max(0, count));
        if (count <= 0) {
            return results;
        }
        const qint64 estimatedWork = estimateRefinementWorkUnits(
            count, catalogContext.visibleStars.size(), allDetectionIndices.size());
        const int threadCount = refinementWorkerThreadCount(count, estimatedWork);
        if (threadCount <= 1)
        {
            for (int i = 0; i < count; ++i)
            {
                if (isCancellationRequested()) {
                    break;
                }
                results[i] = evalFn(*this, i);
            }
            return results;
        }
        QThreadPool recoveryPool;
        recoveryPool.setMaxThreadCount(threadCount);
        for (int workerIndex = 0; workerIndex < threadCount; ++workerIndex)
        {
            recoveryPool.start(QRunnable::create([&, workerIndex]() {
                SolverContext workerContext(m_owner);
                workerContext.copySearchStateFrom(*this);
                for (int i = workerIndex; i < count; i += threadCount)
                {
                    if (workerContext.isCancellationRequested()) {
                        break;
                    }
                    results[i] = evalFn(workerContext, i);
                }
            }));
        }
        recoveryPool.waitForDone();
        return results;
    };

    const bool useNarrowKnownFovRecovery =
        m_useDirectionSeedPreference
        && useStartFov
        && (isNarrowField(settings))
        && selectedFinalPass.projectorValid
        && (std::fabs(best.fovDegrees - settings.m_fov)
            >= std::max(0.01, static_cast<double>(settings.m_fov) * 0.008));
    if (selectedFinalPass.projectorValid
        && rankFinalPassWithSelectedDetections
        && useStartFov
        && ((std::fabs(best.fovDegrees - settings.m_fov) >= 2.0)
            || useNarrowKnownFovRecovery))
    {
        stageStartMs = solveProfileTimer.elapsed();
        Evaluation bestFovPinnedEvaluation;
        FinalMatchPassEvaluation bestFovPinnedFinalPass;
        QVector<double> distortionSeeds = useNarrowKnownFovRecovery
            ? QVector<double>{best.distortionK1, 0.0}
            : QVector<double>{best.distortionK1, 0.0, 0.05, -0.05, 0.10};
        // best.distortionK1 is frequently 0.0 (narrow fields rarely seed lens
        // distortion), which collides with the literal 0.0 seed and would evaluate
        // the entire az/el/roll grid a second time for identical results. Drop the
        // duplicate seed(s) — the outcome is bit-identical, the cost is halved.
        for (int i = distortionSeeds.size() - 1; i >= 1; --i) {
            for (int j = 0; j < i; ++j) {
                if (std::fabs(distortionSeeds[i] - distortionSeeds[j]) < 1e-9) {
                    distortionSeeds.removeAt(i);
                    break;
                }
            }
        }
        const QVector<double> azimuthOffsets = useNarrowKnownFovRecovery
            ? QVector<double>{-0.6, -0.3, 0.0, 0.3, 0.6}
            : QVector<double>{-2.0, 0.0, 2.0};
        const QVector<double> elevationOffsets = useNarrowKnownFovRecovery
            ? QVector<double>{-0.6, -0.3, 0.0, 0.3, 0.6}
            : QVector<double>{-2.0, 0.0, 2.0};
        QVector<double> rollOffsets = useNarrowKnownFovRecovery
            ? QVector<double>{-18.0, -14.0, -10.0, -6.0, -3.0, 0.0, 3.0, 6.0, 10.0, 14.0, 18.0}
            : QVector<double>{-2.0, 0.0, 2.0};
        if (useNarrowKnownFovRecovery)
        {
            const double startRollOffset = normalizeDegrees(
                static_cast<double>(settings.m_roll) - best.rollDegrees + 180.0) - 180.0;
            for (double delta : {startRollOffset - 12.0, startRollOffset - 6.0, startRollOffset, startRollOffset + 6.0, startRollOffset + 12.0})
            {
                bool alreadyPresent = false;
                for (double existing : rollOffsets)
                {
                    if (std::fabs(existing - delta) < 0.25)
                    {
                        alreadyPresent = true;
                        break;
                    }
                }
                if (!alreadyPresent) {
                    rollOffsets.append(delta);
                }
            }
        }
        QVector<Evaluation> fovPinnedPoses;
        fovPinnedPoses.reserve(static_cast<qsizetype>(distortionSeeds.size())
            * azimuthOffsets.size() * elevationOffsets.size() * rollOffsets.size());
        for (double distortionSeed : distortionSeeds)
        {
            for (double azimuthOffset : azimuthOffsets)
            {
                for (double elevationOffset : elevationOffsets)
                {
                    for (double rollOffset : rollOffsets)
                    {
                        Evaluation fovPinnedBest = best;
                        fovPinnedBest.azimuthDegrees += azimuthOffset;
                        fovPinnedBest.elevationDegrees += elevationOffset;
                        fovPinnedBest.rollDegrees += rollOffset;
                        fovPinnedBest.fovDegrees = settings.m_fov;
                        fovPinnedBest.distortionK1 = distortionSeed;
                        fovPinnedPoses.append(fovPinnedBest);
                    }
                }
            }
        }
        // The grid points are independent full final-match-pass evaluations — evaluate
        // them across worker threads, then merge in index order (identical to serial).
        const QVector<FinalMatchPassEvaluation> fovPinnedResults = evaluateRecoveryPosesParallel(
            static_cast<int>(fovPinnedPoses.size()),
            [&](SolverContext& workerContext, int i) {
                return workerContext.evaluateFinalMatchPass(
                    settings,
                    catalogContext,
                    imageSize,
                    starDetections,
                    detectionIndices,
                    fovPinnedPoses[i],
                    finalMatchRadius,
                    true);
            });
        if (isCancellationRequested()) {
            return finishCancelled();
        }
        for (int i = 0; i < fovPinnedPoses.size(); ++i)
        {
            const FinalMatchPassEvaluation& fovPinnedFinalPass = fovPinnedResults[i];
            logFinalMatchPassEvaluation("final-match-pass-fov-pinned", fovPinnedFinalPass);
            if (isBetterWeakModeFinalMatchPass(
                    settings,
                    starDetections,
                    !useStartFov,
                    fovPinnedFinalPass,
                    bestFovPinnedFinalPass))
            {
                bestFovPinnedEvaluation = fovPinnedPoses[i];
                bestFovPinnedFinalPass = fovPinnedFinalPass;
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

    if (!m_disableRollRecovery
        && useStartDirection
        && useStartFov
        && !useStartRoll
        && (isNarrowField(settings))
        && !selectionHasStrongSupport(selectedFinalPass))
    {
        stageStartMs = solveProfileTimer.elapsed();
        static const std::array<double, 21> rollRecoveryOffsets {{
            -30.0, -25.0, -20.0, -15.0, -12.5, -10.0, -7.5, -5.0, -2.5, 0.0,
            2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 20.0, 25.0, 30.0, -45.0, 45.0
        }};
        static const std::array<double, 3> directionRecoveryAzimuthOffsets {{-0.15, 0.0, 0.15}};
        static const std::array<double, 3> directionRecoveryElevationOffsets {{-0.08, 0.0, 0.08}};
        QSet<quint64> evaluatedDirectionRollBins;
        const auto rollRecoveryAccepted = [&](const FinalMatchPassEvaluation& finalPass) {
            return finalPass.projectorValid
                && (finalPass.finalMatches.size() >= result.m_requiredMatches)
                && isAcceptableDirectionSeedSolve(
                    settings,
                    starDetections,
                    finalPass.finalMatches,
                    finalPass.rmsErrorPixels,
                    finalPass.maxErrorPixels)
                && hasAcceptableGuidedFinalBrightnessConsistency(settings, finalPass)
                && hasAcceptableWideBrightAnchorSupport(settings, starDetections, finalPass);
        };
        // Build the deduped recovery-pose grid sequentially (preserving order so the
        // selection below is identical to the serial version), then evaluate each pose
        // (final-match-pass + optional refine + re-evaluate) across worker threads. The
        // per-pose work is pure; all acceptance/selection stays in the serial merge.
        const bool useRecoveryLens = plateSolveStartUsesLens(settings);
        QVector<Evaluation> rollRecoveryPoses;
        rollRecoveryPoses.reserve(static_cast<qsizetype>(directionRecoveryAzimuthOffsets.size())
            * directionRecoveryElevationOffsets.size() * rollRecoveryOffsets.size());
        for (double azimuthOffset : directionRecoveryAzimuthOffsets)
        {
            for (double elevationOffset : directionRecoveryElevationOffsets)
            {
                for (double rollOffset : rollRecoveryOffsets)
                {
                    const double azimuthDegrees = normalizeDegrees(
                        static_cast<double>(settings.m_azimuth) + azimuthOffset);
                    const double elevationDegrees = std::clamp(
                        static_cast<double>(settings.m_elevation) + elevationOffset,
                        -90.0,
                        90.0);
                    const double rollDegrees = normalizeDegrees(
                        static_cast<double>(settings.m_roll) + rollOffset + 180.0) - 180.0;
                    const int azimuthBin = static_cast<int>(std::floor(azimuthDegrees * 20.0));
                    const int elevationBin = static_cast<int>(std::floor((elevationDegrees + 90.0) * 20.0));
                    const int rollBin = static_cast<int>(std::floor((rollDegrees + 180.0) * 4.0));
                    const quint64 directionRollBin =
                        (static_cast<quint64>(static_cast<quint32>(azimuthBin)) << 32)
                        | (static_cast<quint64>(static_cast<quint16>(elevationBin)) << 16)
                        | static_cast<quint16>(rollBin);
                    if (evaluatedDirectionRollBins.contains(directionRollBin)) {
                        continue;
                    }
                    evaluatedDirectionRollBins.insert(directionRollBin);

                    Evaluation rollRecoveryPose;
                    rollRecoveryPose.valid = true;
                    rollRecoveryPose.azimuthDegrees = azimuthDegrees;
                    rollRecoveryPose.elevationDegrees = elevationDegrees;
                    rollRecoveryPose.rollDegrees = rollDegrees;
                    rollRecoveryPose.fovDegrees = settings.m_fov;
                    rollRecoveryPose.centerOffsetXPixels = useRecoveryLens ? settings.m_lensCenterOffsetX : 0.0;
                    rollRecoveryPose.centerOffsetYPixels = useRecoveryLens ? settings.m_lensCenterOffsetY : 0.0;
                    rollRecoveryPose.distortionK1 = useRecoveryLens ? settings.m_lensDistortionK1 : 0.0;
                    rollRecoveryPoses.append(rollRecoveryPose);
                }
            }
        }

        struct RollRecoveryWorkResult {
            FinalMatchPassEvaluation raw;
            FinalMatchPassEvaluation refined;
            bool refinedComputed = false;
        };
        const QVector<RollRecoveryWorkResult> rollRecoveryResults = evaluateRecoveryPosesParallel(
            static_cast<int>(rollRecoveryPoses.size()),
            [&](SolverContext& workerContext, int i) {
                RollRecoveryWorkResult out;
                out.raw = workerContext.evaluateFinalMatchPass(
                    settings,
                    catalogContext,
                    imageSize,
                    starDetections,
                    allDetectionIndices,
                    rollRecoveryPoses[i],
                    finalMatchRadius,
                    false);
                if (out.raw.projectorValid
                    && (out.raw.finalMatches.size() >= settings.m_plateSolveMinMatches)
                    && (out.raw.rmsErrorPixels <= std::min(
                        static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 1.25,
                        32.0)))
                {
                    Evaluation refinementSeed = out.raw.pose;
                    refinementSeed.matches = out.raw.finalMatches;
                    refinementSeed.matchCount = out.raw.finalMatches.size();
                    refinementSeed.rmsErrorPixels = out.raw.rmsErrorPixels;
                    refinementSeed.brightnessRankError = out.raw.brightnessRankError;
                    refinementSeed.meanCatalogMagnitude = out.raw.meanCatalogMagnitude;
                    const Evaluation refinedRollRecoveryPose = workerContext.refinePoseFromMatches(
                        settings,
                        catalogContext,
                        imageSize,
                        captureDateTimeUtc,
                        starDetections,
                        refinementSeed);
                    if (refinedRollRecoveryPose.valid)
                    {
                        out.refined = workerContext.evaluateFinalMatchPass(
                            settings,
                            catalogContext,
                            imageSize,
                            starDetections,
                            allDetectionIndices,
                            refinedRollRecoveryPose,
                            finalMatchRadius,
                            false);
                        out.refinedComputed = true;
                    }
                }
                return out;
            });
        if (isCancellationRequested()) {
            return finishCancelled();
        }

        for (int i = 0; i < rollRecoveryPoses.size(); ++i)
        {
            FinalMatchPassEvaluation rollRecoveryFinalPass = rollRecoveryResults[i].raw;
            if (rollRecoveryResults[i].refinedComputed)
            {
                const FinalMatchPassEvaluation& refinedRollRecoveryFinalPass = rollRecoveryResults[i].refined;
                const bool refinedLosesNamedAnchorSupport =
                    (rollRecoveryFinalPass.namedBrightAnchorMatches >= 2)
                    && (refinedRollRecoveryFinalPass.namedBrightAnchorMatches
                        < rollRecoveryFinalPass.namedBrightAnchorMatches);
                const bool refinedIsAccepted = rollRecoveryAccepted(refinedRollRecoveryFinalPass);
                const bool rawIsAccepted = rollRecoveryAccepted(rollRecoveryFinalPass);
                if (!refinedLosesNamedAnchorSupport
                    && ((refinedIsAccepted && !rawIsAccepted)
                        || ((refinedIsAccepted == rawIsAccepted)
                            && isBetterWeakModeFinalMatchPass(
                            settings,
                            starDetections,
                            false,
                            refinedRollRecoveryFinalPass,
                            rollRecoveryFinalPass))))
                {
                    rollRecoveryFinalPass = refinedRollRecoveryFinalPass;
                }
            }
            logFinalMatchPassEvaluation("final-match-pass-roll-recovery", rollRecoveryFinalPass);
            const bool rollRecoveryIsAccepted = rollRecoveryAccepted(rollRecoveryFinalPass);
            const bool selectedIsAccepted = rollRecoveryAccepted(selectedFinalPass);
            if ((rollRecoveryIsAccepted && !selectedIsAccepted)
                || (rollRecoveryIsAccepted
                    && selectedIsAccepted
                    && (rollRecoveryFinalPass.namedBrightAnchorMatches >= 2)
                    && (selectedFinalPass.namedBrightAnchorMatches < 2))
                || ((rollRecoveryIsAccepted == selectedIsAccepted)
                    && isBetterWeakModeFinalMatchPass(
                    settings,
                    starDetections,
                    false,
                    rollRecoveryFinalPass,
                    selectedFinalPass)))
            {
                best = rollRecoveryFinalPass.pose;
                selectedFinalPass = rollRecoveryFinalPass;
                logFinalMatchPassEvaluation("final-match-pass-roll-recovery", selectedFinalPass, true);
            }
        }
        recordProfileMetric(QStringLiteral("solve.rollRecoveryEvaluations"), evaluatedDirectionRollBins.size());
        logSolveProfile("rollRecoveryFinalPass", stageStartMs);
    }

    stageStartMs = solveProfileTimer.elapsed();
    if (isCancellationRequested()) {
        return finishCancelled();
    }
    FinalMatchPassEvaluation selectedFinalPassForAcceptance = selectedFinalPass;
    if (selectedFinalPass.projectorValid && rankFinalPassWithSelectedDetections)
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
    if (!selectedFinalPassForAcceptance.projectorValid
        || !rankFinalPassWithSelectedDetections
        || (useStartDirection && (isNarrowField(settings))))
    {
        selectedFinalPassForAcceptance = selectedFinalPass;
    }
    if (!selectedFinalPass.projectorValid)
    {
        result.m_failureReason = QStringLiteral("final match projector invalid");
        return result;
    }
    logSolveProfile("finalMatchPass", stageStartMs);

    // Tighten the pose on its inlier core (see tightenNarrowFinalPass). Adopted only
    // if it is a strict improvement, so it can sharpen edge-star associations and
    // residuals without destabilising solves that are already good.
    {
        const bool acceptanceTracksSelected =
            (selectedFinalPassForAcceptance.finalMatches.size() == selectedFinalPass.finalMatches.size())
            && (selectedFinalPassForAcceptance.rmsErrorPixels == selectedFinalPass.rmsErrorPixels)
            && (selectedFinalPassForAcceptance.pose.rollDegrees == selectedFinalPass.pose.rollDegrees);
        selectedFinalPass = tightenNarrowFinalPass(
            settings, catalogContext, imageSize, starDetections, allDetectionIndices,
            selectedFinalPass, finalMatchRadius);
        if (acceptanceTracksSelected) {
            selectedFinalPassForAcceptance = selectedFinalPass;
        }
    }

    const QVector<ProjectedCatalogStar>& projectedStars = selectedFinalPass.projectedStars;
    const QVector<Match>& finalMatches = selectedFinalPass.finalMatches;
    result.m_catalogCandidateStars = projectedStars.size();
    result.m_outlierStars = selectedFinalPass.outlierCount;
    result.m_solverQualityScore = std::max(
        finalMatchPassScore(settings, selectedFinalPass),
        finalMatchPassEvidenceScore(settings, selectedFinalPass));
    result.m_seedConsistencyScore = narrowGuidedSeedConsistencyScore(settings, selectedFinalPass);
    result.m_namedBrightAnchorMatches = selectedFinalPass.namedBrightAnchorMatches;
    result.m_namedBrightAnchorRmsErrorPixels = selectedFinalPass.namedBrightAnchorRmsErrorPixels;
    result.m_seedRadialMagnitudeMatchFraction = selectedFinalPass.seedRadialMagnitudeMatchFraction;
    result.m_prioritySeedProjectedChecks = selectedFinalPass.prioritySeedProjectedChecks;
    result.m_prioritySeedProjectedErrorPixels = selectedFinalPass.prioritySeedProjectedErrorPixels;

    // SHADOW MODE: record the robust false-alarm log-odds for the selected pose so it
    // can be compared against the heuristic accept/reject decision across the corpus.
    // Does not affect the decision yet (see poseFalseAlarmLogOdds).
    recordProfileMetric(QStringLiteral("verify.faLogOddsMilli"),
        static_cast<qint64>(std::llround(
            poseFalseAlarmLogOdds(catalogContext, selectedFinalPass, imageSize, finalMatchRadius,
                static_cast<int>(starDetections.size()))
            * 1000.0)));

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
    const bool sparseGuidedPairAccepted = isAcceptableSparseGuidedPairFinalPass(
        settings,
        catalogContext,
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

    if ((finalMatches.size() < settings.m_plateSolveMinMatches)
        && !sparseWideBlindAccepted
        && !sparseGuidedPairAccepted)
    {
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

    QString rollAmbiguityReason;
    FinalMatchPassEvaluation rollAdoptedAlias;
    if (hasCompetitiveRollAlias(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            allDetectionIndices,
            selectedFinalPass,
            finalMatchRadius,
            &rollAmbiguityReason,
            &rollAdoptedAlias))
    {
        result.m_matchedStars = finalMatches.size();
        result.m_rmsErrorPixels = selectedFinalPass.rmsErrorPixels;
        result.m_maxErrorPixels = selectedFinalPass.maxErrorPixels;
        result.m_matchSummary = matchSummary(catalogContext, starDetections, finalMatches);
        result.m_failureReason = QStringLiteral("direction-seeded solution rejected: ambiguous roll (%1)")
            .arg(rollAmbiguityReason);
        PROFILER_STOP(QString("%1: ambiguous roll").arg(__FUNCTION__));
        return result;
    }
    // A roll alias matched the rare bright stars decisively better -> adopt it as the
    // solution. The projectedStars/finalMatches const-refs above alias selectedFinalPass
    // members, so reassigning it makes them reflect the adopted pose for the labelling
    // and result population below; the remaining acceptance gates re-run on it too.
    if (rollAdoptedAlias.projectorValid)
    {
        selectedFinalPass = rollAdoptedAlias;
        selectedFinalPassForAcceptance = rollAdoptedAlias;
        result.m_catalogCandidateStars = selectedFinalPass.projectedStars.size();
        result.m_outlierStars = selectedFinalPass.outlierCount;
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
        detection.m_catalogRightAscensionDegrees = catalogStar.rightAscensionDegrees;
        detection.m_catalogDeclinationDegrees = catalogStar.declinationDegrees;
        detection.m_catalogSpectralType = catalogStar.spectralType;
        detection.m_solved = true;
    }

    result.m_solved = true;
    result.m_matchedStars = finalMatches.size();
    result.m_rmsErrorPixels = selectedFinalPass.rmsErrorPixels;
    result.m_maxErrorPixels = selectedFinalPass.maxErrorPixels;
    result.m_matchSummary = matchSummary(catalogContext, starDetections, finalMatches);
    result.m_azimuthDegrees = selectedFinalPass.pose.azimuthDegrees;
    result.m_elevationDegrees = selectedFinalPass.pose.elevationDegrees;
    result.m_rollDegrees = selectedFinalPass.pose.rollDegrees;
    result.m_fovDegrees = selectedFinalPass.pose.fovDegrees;
    result.m_centerOffsetXPixels = selectedFinalPass.pose.centerOffsetXPixels;
    result.m_centerOffsetYPixels = selectedFinalPass.pose.centerOffsetYPixels;
    result.m_distortionK1 = selectedFinalPass.pose.distortionK1;
    if (qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE")
        && useStartDirection
        && (isNarrowField(settings)))
    {
        qDebug() << "CameraPlateSolver: final direction acceptance"
                 << "sparsePairAccepted" << sparseGuidedPairAccepted
                 << "highConfidenceSparseAnchors" << hasHighConfidenceSparseGuidedAnchors(settings, selectedFinalPassForAcceptance)
                 << "highConfidenceTriangle" << hasHighConfidenceGuidedTriangleSupport(settings, selectedFinalPassForAcceptance)
                 << "FoVAccepted" << isAcceptableNarrowGuidedFov(settings, selectedFinalPassForAcceptance.pose.fovDegrees)
                 << finalPassBrightDiagnosticSummary(selectedFinalPassForAcceptance)
                 << "Az" << selectedFinalPassForAcceptance.pose.azimuthDegrees
                 << "El" << selectedFinalPassForAcceptance.pose.elevationDegrees
                 << "Roll" << selectedFinalPassForAcceptance.pose.rollDegrees
                 << "FoV" << selectedFinalPassForAcceptance.pose.fovDegrees;
    }
    // A tight, richly-matched fit is a true alignment even without bright anchors; let it
    // bypass the bright-support gates (but not the FoV / direction-seed quality gates).
    const bool overwhelmingFaintSupport =
        useStartDirection
        && hasOverwhelmingFaintGuidedSupport(settings, selectedFinalPassForAcceptance);
    const bool weakNarrowGuidedBrightSupport =
        useStartDirection
        && !sparseGuidedPairAccepted
        && !overwhelmingFaintSupport
        && !hasHighConfidenceSparseGuidedAnchors(settings, selectedFinalPassForAcceptance)
        && hasWeakNarrowGuidedBrightSupport(settings, selectedFinalPassForAcceptance);
    const bool narrowGuidedFovAccepted =
        isAcceptableNarrowGuidedFov(settings, selectedFinalPassForAcceptance.pose.fovDegrees);
    const bool directionSeedSolveAcceptable = !useStartDirection
        || sparseGuidedPairAccepted
        || (!weakNarrowGuidedBrightSupport
            && narrowGuidedFovAccepted
            && isAcceptableDirectionSeedSolve(
                settings,
                starDetections,
                selectedFinalPassForAcceptance.finalMatches,
                selectedFinalPassForAcceptance.rmsErrorPixels,
                selectedFinalPassForAcceptance.maxErrorPixels)
            && (overwhelmingFaintSupport
                || hasAcceptableGuidedFinalBrightnessConsistency(settings, selectedFinalPassForAcceptance)));
    if (!directionSeedSolveAcceptable)
    {
        const QString rejectionReason = !narrowGuidedFovAccepted
            ? narrowGuidedFovRejectionReason(settings, selectedFinalPassForAcceptance.pose.fovDegrees)
            : (weakNarrowGuidedBrightSupport
                ? QStringLiteral("weak bright-anchor support")
                : directionSeedRejectionReason(
                settings,
                starDetections,
                selectedFinalPassForAcceptance.finalMatches,
                selectedFinalPassForAcceptance.rmsErrorPixels,
                selectedFinalPassForAcceptance.maxErrorPixels));
        qDebug() << "CameraPlateSolver: rejecting direction-seeded solution"
                 << "matches=" << selectedFinalPassForAcceptance.finalMatches.size()
                 << "required=" << minimumDirectionSeedAcceptedMatches(settings, starDetections)
                 << "rms=" << selectedFinalPassForAcceptance.rmsErrorPixels
                 << "max=" << selectedFinalPassForAcceptance.maxErrorPixels
                 << "brightnessErr=" << selectedFinalPassForAcceptance.brightnessRankError
                 << "bright=" << finalPassBrightDiagnosticSummary(selectedFinalPassForAcceptance)
                 << "reason=" << rejectionReason
                 << "matchSummary=" << matchSummary(catalogContext, starDetections, selectedFinalPassForAcceptance.finalMatches);
        result.m_failureReason = QStringLiteral("direction-seeded solution rejected: %1 brightnessErr=%2 %3")
            .arg(rejectionReason)
            .arg(selectedFinalPassForAcceptance.brightnessRankError, 0, 'f', 3)
            .arg(finalPassBrightDiagnosticSummary(selectedFinalPassForAcceptance));
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
    requestCancellation();
}

void CameraPlateSolver::requestCancellation()
{
    m_cancelRequested.store(true);
    m_cancelNetworkRequests.store(true);
    if (m_activeNetworkReply) {
        QMetaObject::invokeMethod(m_activeNetworkReply, &QNetworkReply::abort, Qt::QueuedConnection);
    }
}

bool CameraPlateSolver::isCancellationRequested() const
{
    return m_cancelRequested.load();
}

CameraPlateSolveResult CameraPlateSolver::solve(const CameraSettings& settings,
                                                const QSize& imageSize,
                                                const QDateTime& captureDateTime,
                                                QVector<CameraPipelineStarDetection>& starDetections)
{
    // Reset cancellation flag at the start of each solve so that a cancellation
    // from a previous solve doesn't block subsequent ones.
    m_cancelRequested.store(false);
    m_cancelNetworkRequests.store(false);

    auto evictSirilRangeCacheIfNeeded = [&]() {
        qint64 rangeCacheBytes = 0;
        for (const QByteArray& v : m_sirilRangeCache) {
            rangeCacheBytes += v.size();
        }
        if (rangeCacheBytes > SolverContext::kSirilMaxRangeCacheBytes) {
            qDebug() << "CameraPlateSolver: Siril range cache exceeded" << SolverContext::kSirilMaxRangeCacheBytes
                     << "bytes (" << rangeCacheBytes << "), clearing";
            m_sirilRangeCache.clear();
        }
    };

    struct SolveRunProfile
    {
        QString reason;
        qint64 elapsedMs = 0;
        bool solved = false;
        int matchedStars = 0;
        QString summary;
    };
    QVector<SolveRunProfile> solveRunProfiles;
    QElapsedTimer totalSolveTimer;
    totalSolveTimer.start();

    auto runSolve = [&](const CameraSettings& runSettings, const QString& reason, bool disableRollRecovery = false) {
        QElapsedTimer runTimer;
        runTimer.start();
        SolverContext context(this);
        context.m_disableRollRecovery = disableRollRecovery;

        // Swap the persistent caches into the SolverContext so that Siril SPCC data
        // fetched in this solve is reused in future solves rather than discarded.
        std::swap(context.m_sirilRangeCache, m_sirilRangeCache);
        std::swap(context.m_sirilIndexCache, m_sirilIndexCache);

        CameraPlateSolveResult runResult = context.solve(runSettings, imageSize, captureDateTime, starDetections);
        runResult.m_profileSummary = context.profileSummary();
        solveRunProfiles.append({
            disableRollRecovery ? QStringLiteral("%1-no-roll-recovery").arg(reason) : reason,
            runTimer.elapsed(),
            runResult.m_solved,
            runResult.m_matchedStars,
            runResult.m_profileSummary
        });

        std::swap(context.m_sirilRangeCache, m_sirilRangeCache);
        std::swap(context.m_sirilIndexCache, m_sirilIndexCache);
        evictSirilRangeCacheIfNeeded();

        return runResult;
    };

    CameraPlateSolveResult result;
    auto appendOuterProfile = [&]() {
        if (solveRunProfiles.size() <= 1)
        {
            if (!result.m_profileSummary.contains(QStringLiteral("solve.totalWallMs"))) {
                if (!result.m_profileSummary.isEmpty()) {
                    result.m_profileSummary.append(QLatin1Char(';'));
                }
                result.m_profileSummary.append(QStringLiteral("solve.totalWallMs=%1").arg(totalSolveTimer.elapsed()));
            }
            return;
        }

        QStringList outerProfile;
        outerProfile.reserve(2 + solveRunProfiles.size() * 4);
        outerProfile.append(QStringLiteral("solve.runs=%1").arg(solveRunProfiles.size()));
        outerProfile.append(QStringLiteral("solve.totalWallMs=%1").arg(totalSolveTimer.elapsed()));
        for (int i = 0; i < solveRunProfiles.size(); ++i)
        {
            const SolveRunProfile& runProfile = solveRunProfiles.at(i);
            const QString prefix = QStringLiteral("solve.run%1.").arg(i + 1);
            outerProfile.append(prefix + QStringLiteral("reason=%1").arg(runProfile.reason));
            outerProfile.append(prefix + QStringLiteral("ms=%1").arg(runProfile.elapsedMs));
            outerProfile.append(prefix + QStringLiteral("solved=%1").arg(runProfile.solved ? 1 : 0));
            outerProfile.append(prefix + QStringLiteral("matches=%1").arg(runProfile.matchedStars));
        }
        if (!result.m_profileSummary.isEmpty()) {
            result.m_profileSummary.append(QLatin1Char(';'));
        }
        result.m_profileSummary.append(outerProfile.join(QLatin1Char(';')));
    };

    const bool solveUsesDirection =
        (settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartFovAzEl)
        || (settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartFovAzElRoll)
        || (settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartFovAzElRollLens)
        || (settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartCurrentSettingsOnly);
    const bool solveUsesRoll =
        (settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartFovAzElRoll)
        || (settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartFovAzElRollLens)
        || (settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartCurrentSettingsOnly);
    const bool rollPriorNarrowDirectionSolve =
        ((settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartFovAzElRoll)
            || (settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartFovAzElRollLens))
        && (SolverContext::isNarrowField(settings));
    const bool denseNarrowDirectionSolve = solveUsesDirection
        && !solveUsesRoll
        && (SolverContext::isNarrowField(settings))
        && (starDetections.size() > 128);
    const bool tryWithoutRollBeforeRollPrior =
        rollPriorNarrowDirectionSolve
        && (settings.m_plateSolveMaxMagnitude <= 13.0f)
        && (starDetections.size() > 128);
    static constexpr double kRetrySearchRadiusDegrees = 12.0;
    QVector<double> attemptedDenseNarrowBrightCatalogMagnitudes;
    const auto hasAttemptedDenseNarrowBrightCatalogMagnitude = [&attemptedDenseNarrowBrightCatalogMagnitudes](double magnitude) {
        for (double attemptedMagnitude : attemptedDenseNarrowBrightCatalogMagnitudes)
        {
            if (std::fabs(attemptedMagnitude - magnitude) < 1e-6) {
                return true;
            }
        }
        return false;
    };
    const auto markAttemptedDenseNarrowBrightCatalogMagnitude = [&attemptedDenseNarrowBrightCatalogMagnitudes,
                                                                &hasAttemptedDenseNarrowBrightCatalogMagnitude](double magnitude) {
        if (!hasAttemptedDenseNarrowBrightCatalogMagnitude(magnitude)) {
            attemptedDenseNarrowBrightCatalogMagnitudes.append(magnitude);
        }
    };
    if (denseNarrowDirectionSolve
        && (settings.m_plateSolveMaxMagnitude >= 18.0f)
        && (starDetections.size() > 128))
    {
        CameraSettings retrySettings(settings);
        retrySettings.m_plateSolveMaxMagnitude = 15.0f;
        retrySettings.m_plateSolveSearchRadius = static_cast<float>(std::max(
            static_cast<double>(retrySettings.m_plateSolveSearchRadius),
            kRetrySearchRadiusDegrees));
        qDebug() << "CameraPlateSolver: trying dense narrow bright catalog before full catalog"
                 << "maxMagnitude" << retrySettings.m_plateSolveMaxMagnitude
                 << "searchRadius" << retrySettings.m_plateSolveSearchRadius;
        markAttemptedDenseNarrowBrightCatalogMagnitude(retrySettings.m_plateSolveMaxMagnitude);
        result = runSolve(retrySettings, QStringLiteral("bright-catalog"));
    }
    if (tryWithoutRollBeforeRollPrior)
    {
        CameraSettings retrySettings(settings);
        retrySettings.m_plateSolveStartMode = CameraSettings::PlateSolveStartFovAzEl;
        qDebug() << "CameraPlateSolver: trying dense narrow roll-prior solve without roll constraint first"
                 << "maxMagnitude" << settings.m_plateSolveMaxMagnitude;
        result = runSolve(retrySettings, QStringLiteral("without-roll"));
    }
    const bool modestEarlyBrightCatalogSolve =
        denseNarrowDirectionSolve
        && result.m_solved
        && (settings.m_plateSolveMaxMagnitude >= 18.0f)
        && (result.m_matchedStars < std::max(settings.m_plateSolveMinMatches + 120, 160));
    if (!result.m_solved || modestEarlyBrightCatalogSolve)
    {
        CameraPlateSolveResult initialResult = runSolve(settings, QStringLiteral("initial"));
        if (!result.m_solved
            || (initialResult.m_solved
                && (initialResult.m_matchedStars > result.m_matchedStars + 20)))
        {
            result = initialResult;
        }
    }

    const auto denseNarrowResultRetryScore = [](const CameraPlateSolveResult& solveResult) {
        if (solveResult.m_matchedStars <= 0) {
            return -std::numeric_limits<double>::infinity();
        }

        return solveResult.m_solverQualityScore
            + 0.75 * solveResult.m_seedConsistencyScore
            + 0.05 * static_cast<double>(std::min(solveResult.m_matchedStars, 160))
            + (solveResult.m_solved ? 1.5 : 0.0);
    };
    bool denseNarrowBrightCatalogRetried = false;
    auto retryDenseNarrowDirectionWithBrightCatalog = [&]() {
        if (!denseNarrowDirectionSolve
            || isCancellationRequested()
            || (settings.m_plateSolveMaxMagnitude <= 13.0f)
            || denseNarrowBrightCatalogRetried)
        {
            return;
        }
        denseNarrowBrightCatalogRetried = true;

        QVector<double> brightRetryMagnitudes;
        const auto appendBrightRetryMagnitude = [&](double magnitude, bool allowCurrentMagnitude = false) {
            if (((magnitude >= static_cast<double>(settings.m_plateSolveMaxMagnitude))
                    && !(allowCurrentMagnitude
                        && (std::fabs(magnitude - static_cast<double>(settings.m_plateSolveMaxMagnitude)) < 1e-6)))
                || (magnitude < 13.0))
            {
                return;
            }
            for (double existing : brightRetryMagnitudes)
            {
                if (std::fabs(existing - magnitude) < 1e-6) {
                    return;
                }
            }
            brightRetryMagnitudes.append(magnitude);
        };
        if (settings.m_plateSolveMaxMagnitude <= 15.0f) {
            appendBrightRetryMagnitude(settings.m_plateSolveMaxMagnitude, true);
        }
        appendBrightRetryMagnitude(15.0);
        appendBrightRetryMagnitude(13.0);
        for (double retryMagnitude : brightRetryMagnitudes)
        {
            if (hasAttemptedDenseNarrowBrightCatalogMagnitude(retryMagnitude)) {
                continue;
            }
            markAttemptedDenseNarrowBrightCatalogMagnitude(retryMagnitude);
            CameraSettings retrySettings(settings);
            retrySettings.m_plateSolveMaxMagnitude = static_cast<float>(retryMagnitude);
            retrySettings.m_plateSolveSearchRadius = static_cast<float>(std::max(
                static_cast<double>(retrySettings.m_plateSolveSearchRadius),
                kRetrySearchRadiusDegrees));
            qDebug() << "CameraPlateSolver: retrying dense narrow direction solve with bright catalog"
                     << "maxMagnitude" << retrySettings.m_plateSolveMaxMagnitude
                     << "searchRadius" << retrySettings.m_plateSolveSearchRadius;
            QVector<CameraPipelineStarDetection> detectionsBeforeRetry = starDetections;
            CameraPlateSolveResult retryResult = runSolve(retrySettings, QStringLiteral("bright-catalog"));
            const double currentScore = denseNarrowResultRetryScore(result);
            const double retryScore = denseNarrowResultRetryScore(retryResult);
            const bool retryScoreBetter =
                retryResult.m_solved
                && (retryScore > (currentScore + 0.50));
            const bool retrySeedMuchBetter =
                retryResult.m_solved
                && (retryResult.m_seedConsistencyScore > (result.m_seedConsistencyScore + 2.0));
            const double competitiveSupportFraction =
                !result.m_solved || retryScoreBetter || retrySeedMuchBetter
                    ? 0.45
                    : 0.90;
            const int competitiveSolvedMatchFloor = (result.m_matchedStars > 0)
                ? std::max(
                    settings.m_plateSolveMinMatches + 8,
                    static_cast<int>(std::floor(static_cast<double>(result.m_matchedStars) * competitiveSupportFraction)))
                : settings.m_plateSolveMinMatches;
            const bool solvedRetryKeepsSupport =
                retryResult.m_solved
                && (retryResult.m_matchedStars >= competitiveSolvedMatchFloor);
            if (retryResult.m_solved && !solvedRetryKeepsSupport && !retryScoreBetter)
            {
                qDebug() << "CameraPlateSolver: ignoring weak bright-catalog solved retry"
                         << "matches" << retryResult.m_matchedStars
                         << "required" << competitiveSolvedMatchFloor
                         << "currentMatches" << result.m_matchedStars
                         << "score" << retryScore
                         << "currentScore" << currentScore
                         << "seed" << retryResult.m_seedConsistencyScore
                         << "currentSeed" << result.m_seedConsistencyScore
                         << "maxMagnitude" << retrySettings.m_plateSolveMaxMagnitude;
            }
            const bool solvedRetryBeatsUnsolved =
                retryResult.m_solved
                && !result.m_solved
                && (retryResult.m_matchedStars >= std::max(settings.m_plateSolveMinMatches + 8, 18));
            const bool solvedBrightRetryHasUsefulAnchorSupport =
                retryResult.m_solved
                && (retryMagnitude <= 15.0)
                && (retryResult.m_matchedStars >= std::max(settings.m_plateSolveMinMatches + 24, 40))
                && (!result.m_solved
                    || (result.m_matchedStars >= 120)
                    || (retryResult.m_seedConsistencyScore > (result.m_seedConsistencyScore + 0.75)));
            if (solvedRetryBeatsUnsolved
                || solvedBrightRetryHasUsefulAnchorSupport
                || solvedRetryKeepsSupport
                || retryScoreBetter
                || (retrySeedMuchBetter && (retryResult.m_matchedStars >= competitiveSolvedMatchFloor))
                || (retryResult.m_matchedStars > result.m_matchedStars))
            {
                result = retryResult;
            } else {
                starDetections = detectionsBeforeRetry;
            }
            if (result.m_solved || isCancellationRequested()) {
                break;
            }
        }
    };

    if (!result.m_solved && (result.m_matchedStars >= 100)) {
        retryDenseNarrowDirectionWithBrightCatalog();
    }

    // NB: a "strong rejected anchor" guard used to suppress the recenter retry here, but
    // it mis-fired on wrong-roll aliases that coincidentally match a few named bright
    // anchors (e.g. m51 @16 from a ~1° offset seed locks onto a roll-58° alias with 3
    // named anchors at low RMS) and so blocked the az/el recenter that actually finds the
    // true pose. The recenter loop below keeps the current result as its floor and only
    // adopts a clearly-better *solved* pose, so always attempting it is safe; the seed
    // offset, not catalog depth, is what these cases need to overcome (see
    // doc/camera/plate-solver-notes.md "REFRAMING").
    // The recenter retry (az/el seed-jitter) is eligible for any narrow direction-seeded
    // solve, including sparse fields: a ~0.3-1° pointing error throws the seed out of the
    // solve basin regardless of star count (e.g. stars-narrow-1 has only 29 detections but
    // solves cleanly at 25 matches / rms 0.3 once the ~0.31° elevation seed error is
    // corrected). `denseNarrowDirectionSolve` (its >128-detection bright-catalog retry
    // sibling) is too strict here.
    const bool narrowDirectionRecenterEligible =
        solveUsesDirection && !solveUsesRoll && SolverContext::isNarrowField(settings);
    if (!result.m_solved
        && narrowDirectionRecenterEligible
        && !isCancellationRequested())
    {
        const double fovDegrees = std::max(0.1, static_cast<double>(settings.m_fov));
        QVector<std::pair<double, double>> recenterOffsets;
        recenterOffsets.reserve(16);
        const auto appendRecenteringOffset = [&](double azimuthOffset, double elevationOffset) {
            for (const auto& existingOffset : recenterOffsets)
            {
                if ((std::fabs(existingOffset.first - azimuthOffset) < 0.05)
                    && (std::fabs(existingOffset.second - elevationOffset) < 0.05))
                {
                    return;
                }
            }
            recenterOffsets.append({azimuthOffset, elevationOffset});
        };
        // Coarse az offsets first (a ~1° / 0.75-fov pointing error is the common case and
        // these are kept in their original order so they still resolve as before), then a
        // finer az tier: a sub-fov seed error (e.g. m101 @15 is only ~0.42° = 0.33-fov off)
        // is *overshot* by the ±0.75/±1.0-fov steps, so a near-true seed is never sampled
        // and the solve locks onto a wrong roll. Then the same coverage in *elevation*
        // (stars-narrow-1's seed is ~0.31° = 0.24-fov low in elevation) — previously the
        // elevation offsets sat past the attempt budget and never ran, so the recenter was
        // azimuth-only in practice. Cases that resolve on an az offset early-stop (strong
        // solved candidate) before reaching the elevation tier, so ordering is preserved.
        const std::array<std::pair<double, double>, 16> defaultRecenterOffsets = {{
            { fovDegrees * 0.75, 0.0 },
            { -fovDegrees * 0.75, 0.0 },
            { fovDegrees, 0.0 },
            { -fovDegrees, 0.0 },
            { fovDegrees * 0.33, 0.0 },
            { -fovDegrees * 0.33, 0.0 },
            { fovDegrees * 0.5, 0.0 },
            { -fovDegrees * 0.5, 0.0 },
            { 0.0, fovDegrees * 0.33 },
            { 0.0, -fovDegrees * 0.33 },
            { 0.0, fovDegrees * 0.5 },
            { 0.0, -fovDegrees * 0.5 },
            { 0.0, fovDegrees * 0.75 },
            { 0.0, -fovDegrees * 0.75 },
            { fovDegrees * 0.75, fovDegrees * 0.75 },
            { -fovDegrees * 0.75, -fovDegrees * 0.75 }
        }};
        for (const auto& offset : defaultRecenterOffsets) {
            appendRecenteringOffset(offset.first, offset.second);
        }
        CameraPlateSolveResult bestRecenterResult = result;
        QVector<CameraPipelineStarDetection> bestRecenterDetections = starDetections;
        const QVector<CameraPipelineStarDetection> detectionsBeforeRecenters = starDetections;
        const SolverContext::SkyVector originalSeedVector = SolverContext::normalize(
            SolverContext::vectorFromAltAz(settings.m_azimuth, settings.m_elevation));
        const auto originalSeedAngularDistance = [&originalSeedVector](const CameraPlateSolveResult& solveResult) {
            if (!solveResult.m_solved) {
                return std::numeric_limits<double>::infinity();
            }

            const SolverContext::SkyVector solvedVector = SolverContext::normalize(
                SolverContext::vectorFromAltAz(
                    solveResult.m_azimuthDegrees,
                    solveResult.m_elevationDegrees));
            const double cosDistance = std::clamp(
                SolverContext::dot(originalSeedVector, solvedVector),
                -1.0,
                1.0);
            return std::acos(cosDistance) * 180.0 / 3.14159265358979323846;
        };
        const auto denseNarrowRecenterScore = [&originalSeedAngularDistance, &settings](const CameraPlateSolveResult& solveResult) {
            if (solveResult.m_matchedStars <= 0) {
                return -std::numeric_limits<double>::infinity();
            }

            const double directionDistance = originalSeedAngularDistance(solveResult);
            const double directionScale = std::max(1.0, static_cast<double>(settings.m_fov));
            const double directionPenalty = std::isfinite(directionDistance)
                ? 1.5 * std::pow(directionDistance / directionScale, 2.0)
                : 0.0;

            return solveResult.m_solverQualityScore
                + 0.75 * solveResult.m_seedConsistencyScore
                + 0.05 * static_cast<double>(std::min(solveResult.m_matchedStars, 160))
                + (solveResult.m_solved ? 1.5 : 0.0)
                - directionPenalty;
        };
        const int strongRecenterMatchCount = std::max(settings.m_plateSolveMinMatches + 48, 80);
        // Budget enough attempts to reach the finer az tier (indices 4..7) and the
        // elevation tier (indices 8..13) so a sub-fov seed error in either axis is
        // recovered; the coarse offsets that already resolve a case early-stop well before
        // this, so the extra budget only costs time on otherwise-failing solves.
        const int maxRecenterAttempts = 14;
        int recenterAttempts = 0;
        for (const auto& offset : recenterOffsets)
        {
            if (isCancellationRequested()) {
                break;
            }
            if (recenterAttempts >= maxRecenterAttempts)
            {
                qDebug() << "CameraPlateSolver: stopping dense narrow recenter retries after attempt budget"
                         << "attempts" << recenterAttempts
                         << "bestSolved" << bestRecenterResult.m_solved
                         << "bestMatches" << bestRecenterResult.m_matchedStars
                         << "bestScore" << denseNarrowRecenterScore(bestRecenterResult);
                break;
            }
            ++recenterAttempts;
            starDetections = detectionsBeforeRecenters;
            CameraSettings retrySettings(settings);
            retrySettings.m_azimuth = static_cast<float>(SolverContext::normalizeDegrees(
                static_cast<double>(settings.m_azimuth) + offset.first));
            retrySettings.m_elevation = static_cast<float>(std::clamp(
                static_cast<double>(settings.m_elevation) + offset.second,
                -90.0,
                90.0));
            retrySettings.m_plateSolveSearchRadius = static_cast<float>(std::max(
                static_cast<double>(retrySettings.m_plateSolveSearchRadius),
                kRetrySearchRadiusDegrees));
            qDebug() << "CameraPlateSolver: retrying dense narrow direction solve with recentered seed"
                     << "azimuth" << retrySettings.m_azimuth
                     << "elevation" << retrySettings.m_elevation
                     << "searchRadius" << retrySettings.m_plateSolveSearchRadius;
            CameraPlateSolveResult retryResult = runSolve(retrySettings, QStringLiteral("recenter"), true);
            const double retryScore = denseNarrowRecenterScore(retryResult);
            const double bestScore = denseNarrowRecenterScore(bestRecenterResult);
            const double retryOriginalDistance = originalSeedAngularDistance(retryResult);
            const double bestOriginalDistance = originalSeedAngularDistance(bestRecenterResult);
            const bool retryHasMeaningfullyLessSupport =
                retryResult.m_solved
                && bestRecenterResult.m_solved
                && (retryResult.m_matchedStars + 6 < bestRecenterResult.m_matchedStars);
            const bool retryMovesFartherFromOriginal =
                std::isfinite(retryOriginalDistance)
                && std::isfinite(bestOriginalDistance)
                && (retryOriginalDistance > (bestOriginalDistance + std::max(0.25, static_cast<double>(settings.m_fov) * 0.25)));
            const bool retryLosesSupportAndMovesFarther =
                retryHasMeaningfullyLessSupport && retryMovesFartherFromOriginal;
            const bool solvedRetryBeatsUnsolved =
                retryResult.m_solved
                && !bestRecenterResult.m_solved
                && (retryResult.m_matchedStars >= std::max(settings.m_plateSolveMinMatches + 8, 18));
            const bool retryBetter =
                !retryLosesSupportAndMovesFarther
                && (solvedRetryBeatsUnsolved
                    || (retryScore > (bestScore + 0.25))
                    || ((std::fabs(retryScore - bestScore) <= 0.25)
                        && ((retryResult.m_solved && !bestRecenterResult.m_solved)
                            || (retryResult.m_solved == bestRecenterResult.m_solved
                                && (retryResult.m_matchedStars > bestRecenterResult.m_matchedStars)))));
            if (retryBetter)
            {
                bestRecenterResult = retryResult;
                bestRecenterDetections = starDetections;
                if (bestRecenterResult.m_solved
                    && (bestRecenterResult.m_matchedStars >= strongRecenterMatchCount))
                {
                    qDebug() << "CameraPlateSolver: stopping dense narrow recenter retries after strong solved candidate"
                             << "matches" << bestRecenterResult.m_matchedStars
                             << "score" << retryScore
                             << "azimuth" << bestRecenterResult.m_azimuthDegrees
                             << "elevation" << bestRecenterResult.m_elevationDegrees
                             << "roll" << bestRecenterResult.m_rollDegrees;
                    break;
                }
            }
        }
        result = bestRecenterResult;
        starDetections = bestRecenterDetections;
    }

    retryDenseNarrowDirectionWithBrightCatalog();

    const auto wrappedAngularDistanceDegrees = [](double lhs, double rhs) {
        double delta = std::fabs(lhs - rhs);
        while (delta > 360.0) {
            delta -= 360.0;
        }
        if (delta > 180.0) {
            delta = 360.0 - delta;
        }
        return delta;
    };
    const auto directionDeltaDegrees = [&settings, &wrappedAngularDistanceDegrees](const CameraPlateSolveResult& solveResult) {
        const double azimuthDelta = wrappedAngularDistanceDegrees(
            solveResult.m_azimuthDegrees,
            settings.m_azimuth);
        const double elevationDelta = std::fabs(
            solveResult.m_elevationDegrees - static_cast<double>(settings.m_elevation));
        return std::sqrt(azimuthDelta * azimuthDelta + elevationDelta * elevationDelta);
    };
    const double rollPriorDirectionRetryThreshold = std::max(0.5, static_cast<double>(settings.m_fov) * 0.5);
    if (rollPriorNarrowDirectionSolve
        && !isCancellationRequested()
        && (!result.m_solved || (directionDeltaDegrees(result) > rollPriorDirectionRetryThreshold)))
    {
        QVector<CameraPipelineStarDetection> rollPriorDetections = starDetections;
        CameraSettings retrySettings(settings);
        retrySettings.m_plateSolveStartMode = CameraSettings::PlateSolveStartFovAzEl;
        qDebug() << "CameraPlateSolver: retrying narrow roll-prior solve without roll constraint"
                 << "directionDelta" << (result.m_solved ? directionDeltaDegrees(result) : -1.0)
                 << "threshold" << rollPriorDirectionRetryThreshold;
        CameraPlateSolveResult retryResult = runSolve(retrySettings, QStringLiteral("without-roll"));
        const double retryDirectionDelta = retryResult.m_solved
            ? directionDeltaDegrees(retryResult)
            : std::numeric_limits<double>::infinity();
        const double resultDirectionDelta = result.m_solved
            ? directionDeltaDegrees(result)
            : std::numeric_limits<double>::infinity();
        const bool retryKeepsMatchSupport = retryResult.m_matchedStars >= std::max(
            settings.m_plateSolveMinMatches,
            result.m_solved ? (result.m_matchedStars - 2) : settings.m_plateSolveMinMatches);
        const bool retryKeepsResidualQuality = !result.m_solved
            || (retryResult.m_rmsErrorPixels <= (result.m_rmsErrorPixels
                + std::max(3.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.15)));
        if (retryResult.m_solved
            && retryKeepsMatchSupport
            && retryKeepsResidualQuality
            && (!result.m_solved || ((retryDirectionDelta + 0.1) < resultDirectionDelta)))
        {
            result = retryResult;
        }
        else if (result.m_solved)
        {
            starDetections = rollPriorDetections;
        }
    }

    appendOuterProfile();
    return result;
}
