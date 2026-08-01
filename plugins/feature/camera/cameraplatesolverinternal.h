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

#ifndef INCLUDE_FEATURE_CAMERAPLATESOLVERINTERNAL_H_
#define INCLUDE_FEATURE_CAMERAPLATESOLVERINTERNAL_H_

// Private implementation header for CameraPlateSolver: the SolverContext class declaration
// (nested types + member functions). Included by cameraplatesolver.cpp (orchestrator) and the
// per-theme cameraplatesolver*.cpp translation units that define its members out-of-line (WS5).

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
#include <QLoggingCategory>
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

// Per-solve tracing (pose candidates, seed retries, accept/reject reasons). Defined in
// cameraplatesolver.cpp; defaults to QtWarningMsg, so a solve emits nothing at debug level
// unless it is asked for. A solve prints ~100 lines, which otherwise buries the per-frame
// lines that matter for operation (CameraPointingError / CameraAutoguide, both qInfo).
// Turn the trace back on with the standard Qt filter:
//     QT_LOGGING_RULES="camera.platesolver.debug=true"
// (harness recipes using QT_LOGGING_RULES="*.debug=true" keep working unchanged - an
// explicit rule overrides the category's default level).
Q_DECLARE_LOGGING_CATEGORY(cameraPlateSolverLog)

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
    // Monotonic id stamped whenever visibleStars is (re)populated (populateVisibleCatalogContext).
    // Used by the evaluateFinalMatchPass seed-prior cache (P-A) to detect a catalog rebuild: two
    // distinct-content contexts never share a generation because that is the single assignment site.
    quint64 visibleStarsGeneration = 0;
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

    // P-C: derived gate/score values the ranking comparator would otherwise recompute for both
    // sides on every pairwise comparison (the incumbent's are re-derived once per candidate). These
    // are pure functions of (settings, [starDetections,] this evaluation), so evaluateFinalMatchPass
    // computes them once when the pass is fully populated and sets cachedGatesValid; the cached*
    // accessors read them, falling back to direct computation when the flag is unset (e.g. an
    // early-return pass). Bit-identical to recomputing.
    bool cachedGatesValid = false;
    bool cachedStrongDenseNarrowGuided = false;
    double cachedFinalMatchPassScore = 0.0;
    double cachedNarrowGuidedBrightConsistencyScore = 0.0;
    double cachedNarrowGuidedSeedConsistencyScore = 0.0;
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
    // Handedness. When true the projector maps to/from a horizontally-MIRRORED image: an
    // up-looking all-sky camera images the sky reflected, so a pose recovered on the mirrored
    // detection set (CameraPlateSolveResult::m_mirrored) must reflect pixel x about the image
    // centre to overlay onto the ORIGINAL image. Default false keeps the projector orientation-
    // preserving and every existing solve/overlay path byte-identical; only projectors built for
    // OUTPUT/overlay of a mirrored solve set it. See projectVector/unprojectPixelToVector.
    bool mirrorX = false;
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

// P-A: per-(catalog,seed,detections) cache of the candidate-INDEPENDENT seed-prior work that
// evaluateFinalMatchPass would otherwise recompute on every call (it runs hundreds of times per
// solve). Covers the seed-radial projector + the O(V) seed projection of every visible star, the
// sorted detection radii, and the sorted bright-detection list -- all functions of settings, the
// direction-seed reference, the detections and the visible catalog only. Rebuilt only when the
// guard changes; the stored values are bit-identical to the former inline computation.
struct FinalPassSeedCache
{
    bool valid = false;
    quint64 catalogGeneration = 0;
    int detectionCount = -1;
    bool useNarrowGuidedBrightPrior = false;
    bool useSeedRadialPrior = false;
    double refAz = 0.0, refEl = 0.0, refRoll = 0.0, refFov = 0.0;
    double lensCx = 0.0, lensCy = 0.0, lensK1 = 0.0;
    SkyProjector seedRadialProjector;
    QPointF seedRadialCenter;
    QVector<QPointF> seedPoints;      // parallel to catalogContext.visibleStars (only when useSeedRadialPrior)
    QVector<quint8> seedPointValid;   // parallel to catalogContext.visibleStars
    QVector<double> sortedDetectionRadii;
    QVector<int> brightDetectionIndices;  // all-detections filtered+sorted (only when useNarrowGuidedBrightPrior)
};
FinalPassSeedCache m_finalPassSeedCache;

// Build-or-reuse m_finalPassSeedCache for the given inputs and return it. Bit-identical to the
// former inline computation in evaluateFinalMatchPass.
const FinalPassSeedCache& finalPassSeedCache(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QVector<CameraPipelineStarDetection>& starDetections);

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
// Maximum total size of an on-disk per-region star cache directory (siril-*-region-cache).
// Each region file is ~12-68 MB (one per unique ra/dec/radius/mag) and they accumulated
// unbounded — the astro-region cache had grown to ~63 GB and filled the disk. After writing a
// region file the oldest files in its directory are evicted (FIFO by mtime) until back under
// the configured cap. A 0 GB cap disables pruning.
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

void clearProfileTimings();

void recordProfileTiming(const QString& stage, qint64 elapsedMs);

void recordProfileMetric(const QString& name, qint64 value);

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

void copySearchStateFrom(const SolverContext& other);

static qint64 boundedMultiply(qint64 lhs, qint64 rhs);

static qint64 estimateRefinementWorkUnits(int candidateCount, int visibleStarCount, int detectionCount);

static int refinementWorkerThreadCount(int candidateCount, qint64 estimatedWorkUnits);

static int visibleCatalogWorkerThreadCount(int catalogStarCount);

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

static double degToRad(double value);

static double normalizeDegrees(double value);

static SkyVector vectorFromAltAz(double azimuthDegrees, double elevationDegrees);

static double dot(const SkyVector& lhs, const SkyVector& rhs);

static SkyVector cross(const SkyVector& lhs, const SkyVector& rhs);

static double length(const SkyVector& vector);

static SkyVector normalize(const SkyVector& vector);

static SkyVector rotateAroundAxis(const SkyVector& vector, const SkyVector& axis, double angleRadians);

static double parseRightAscensionDegrees(const QString& value);

static double parseDeclinationDegrees(const QString& value);

static QString stripQuotedField(const QString& value);

static QString downloadedCatalogDir();

static QString downloadedCatalogReducedPath();

static QString sirilAstroCatalogPath();

static QString sirilAstroCompressedCatalogPath();

static QByteArray gunzipData(const QByteArray& compressedData, QString* errorMessage);

static QVector<CatalogStar> parseBundledCatalog(const QString& text);

static QVector<CatalogStar> parseDownloadedHygCatalog(const QString& text);

QVector<CatalogStar> filterCatalogStars(const QVector<CatalogStar>& stars);

static QString formatRightAscensionHours(double rightAscensionDegrees);

static QString formatDeclinationDegrees(double declinationDegrees);

static QString formatGaiaCoordinateLabel(double rightAscensionDegrees, double declinationDegrees);

static bool isGenericGaiaCatalogName(const QString& name);

static bool writeReducedCatalog(const QVector<CatalogStar>& stars, const QString& path, QString* errorMessage);

static QVector<CatalogStar> loadCatalogFromTextFile(const QString& path);

static QString currentCatalogPath(const CameraSettings& settings);

static QString currentCatalogSource(const CameraSettings& settings);

static double firstPassPlateSolveMaxMagnitude(const CameraSettings& settings);

static double narrowGuidedFullSearchMaxMagnitude(const CameraSettings& settings);

static bool isLowMagnitudeNarrowGuidedSolve(const CameraSettings& settings);

static QString sirilSpccChunkUrl(int chunkIndex, int sourceIndex);

static QString sirilSpccSourceName(int sourceIndex);

static QString sirilRangeCacheKey(int chunkIndex, qint64 firstByte, qint64 lastByte);

static QString sirilCacheRootDir();

static QString sirilIndexDiskCachePath(int chunkIndex);

static QString sirilRangeDiskCachePath(int chunkIndex, qint64 firstByte, qint64 lastByte);

static QString sirilRegionCacheRootDir();

static QString sirilAstroRegionCacheRootDir();

static QString sirilRegionCacheKey(double centerRaDegrees, double centerDecDegrees, double queryRadiusDegrees, double maxMagnitude);

static QString sirilRegionDiskCachePath(double centerRaDegrees, double centerDecDegrees, double queryRadiusDegrees, double maxMagnitude);

static QString sirilAstroRegionDiskCachePath(double centerRaDegrees, double centerDecDegrees, double queryRadiusDegrees, double maxMagnitude);

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

static QVector<CatalogStar> readSirilRegionDiskCacheFile(const QString& path);

static qint64 sirilRegionDiskCacheMaxBytes(int maxSizeGb);

// Cap a per-region disk-cache directory at sirilRegionDiskCacheMaxBytes(), evicting the
// oldest files (FIFO by modification time) until back under the cap. Best-effort and
// concurrency-safe across solver processes: a delete that loses a race (file already gone,
// or held open by another process mid-read) is simply skipped — a missing region file is a
// cache miss that re-fetches, never a corruption.
static void enforceSirilRegionDiskCacheLimit(const QString& directoryPath, int maxSizeGb);

static void writeSirilRegionDiskCacheFile(const QString& path, const QVector<CatalogStar>& stars, int maxSizeGb);

static QByteArray readSirilDiskCacheFile(const QString& path, qint64 expectedSize);

static void writeSirilDiskCacheFile(const QString& path, const QByteArray& data);

static double fovLongEdgeDiagonalDegrees(const QSize& imageSize, double fovDegrees);

static double halfHorizontalFovFromLongEdgeFov(CameraSettings::LensProjection lensProjection, const QSize& imageSize, double fovDegrees);

QByteArray fetchSirilRangeFromSource(int chunkIndex, qint64 firstByte, qint64 lastByte, int sourceIndex);

QByteArray fetchSirilRange(int chunkIndex, qint64 firstByte, qint64 lastByte);

QByteArray fetchSirilChunkIndex(int chunkIndex);

void prefetchSirilMergedRanges(const QVector<SirilMergedRange>& mergedRanges);

static quint32 interleaveHealpixBits(int x, int y);

static quint32 healpixNestedPixel(double rightAscensionDegrees, double declinationDegrees);

static double angularSeparationDegrees(double raDegreesA, double decDegreesA, double raDegreesB, double decDegreesB);

struct SirilQueryGeometry
{
    bool valid = false;
    bool tooWide = false;
    double centerRaDegrees = 0.0;
    double centerDecDegrees = 0.0;
    double queryRadiusDegrees = 0.0;
    QString failureReason;
};

SirilQueryGeometry sirilQueryGeometry(const CameraSettings& settings, const QSize& imageSize, const QDateTime& captureDateTimeUtc);

static QSet<quint32> sampleSirilHealpixPixels(double centerRaDegrees, double centerDecDegrees, double radiusDegrees);

bool sirilCellRecordRange(quint32 pixel, int& chunkIndex, qint64& firstRecord, qint64& recordCount);

static QByteArray readSirilAstroIndex(QFile& file);

static bool sirilAstroCellRecordRange(const QByteArray& indexBytes, quint32 pixel, qint64& firstRecord, qint64& recordCount);

QVector<CatalogStar> loadSirilAstroCatalog(const CameraSettings& settings, const QSize& imageSize, const QDateTime& captureDateTimeUtc, double maxMagnitude, QString* catalogSource);

QVector<CatalogStar> loadSirilSpccCatalog(const CameraSettings& settings, const QSize& imageSize, const QDateTime& captureDateTimeUtc, double maxMagnitude, QString* catalogSource);

static bool plateSolveStartUsesFov(const CameraSettings& settings);

static bool plateSolveStartUsesCurrentSettingsOnly(const CameraSettings& settings);

// The seed-projected bright-star gate only applies when the user pinned roll (or is
// re-solving the current settings), so the seed orientation -- and therefore the
// projected positions of the bright catalog stars -- can be trusted.
static bool usesSeedProjectedBrightGate(const CameraSettings& settings);

static bool plateSolveStartUsesElevation(const CameraSettings& settings);

static bool plateSolveStartUsesDirection(const CameraSettings& settings);

static bool plateSolveStartUsesRoll(const CameraSettings& settings);

static bool plateSolveStartUsesLens(const CameraSettings& settings);

static bool isWidePlateSolveContext(const CameraSettings& settings);

// True for narrow (telescope) fields. See kNarrowFieldMaxFovDegrees.
static bool isNarrowField(const CameraSettings& settings);

// The common "user gave us a direction and it's a narrow telescope field" case that
// many search and acceptance paths special-case.
static bool isNarrowGuidedDirectionSolve(const CameraSettings& settings);

static bool seedFovCompatibleWithStartFov(const CameraSettings& settings, double seedFovDegrees);

static bool canCalibrateLens(const CameraSettings& settings);

static bool canCalibratePrincipalPoint(const CameraSettings& settings);

// Returns by value (QVector is copy-on-write): the static cache is reassignable by another
// concurrent solver under the lock, so a const& would dangle once the lock is released.
static QVector<CatalogStar> brightStarCatalog(const CameraSettings& settings);

static QVector<CatalogStar> bundledAliasCatalog();

static QVector<int> aliasCatalogDeclinationSortedIndices(const QVector<CatalogStar>& aliasStars);

static QString resolveNamedAliasForCatalogStar(const CatalogStar& star, const QVector<CatalogStar>& aliasStars, const QVector<int>& sortedAliasIndices, double* outScore = nullptr);

static void applyNamedAliasesToCatalog(const CameraSettings& settings, QVector<CatalogStar>& stars);

static QString catalogDisplayName(const CatalogStar& star);

static double catalogAngularSeparationDegrees(const CatalogStar& lhs, const CatalogStar& rhs);

static void mergeBundledBrightStarsIntoCatalog(const CameraSettings& settings, QVector<CatalogStar>& catalogStars, double maxMagnitude, double centerRaDegrees, double centerDecDegrees, double queryRadiusDegrees);

static QString matchSummary(const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<Match>& matches);

// Explain a low-match failure that is really a catalogue-depth failure. Returns a note when
// the magnitude limit leaves too few catalogue stars in the frame for the required match
// count to be reachable at all, and an empty string when the catalogue is well populated (so
// a genuine pose failure keeps its concise reason). See the call site in solve().
static QString catalogDepthDiagnostic(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const Evaluation& best, bool useStartDirection);

// Post-match collision repair: for each unmatched detection that is (a) closer to a
// matched catalog star than the current match and (b) substantially brighter (SNR ratio),
// replace the matched detection with the closer brighter one.  This corrects "brightness-
// rank collisions" where the sort formula assigns a dimmer detection to a catalog star
// because it happens to have a better brightness-rank match, while the true bright-star
// detection ends up unmatched despite being geometrically closer.
//
// Safety properties:
//   - Condition (a) alone is safe when the correct detection is already matched (passing
//     cases): in those cases the correct detection IS the closest match, so no unmatched
//     detection satisfies condition (a) for its catalog star.
//   - Condition (b) guards against swapping in accidental close artifacts (hot pixels,
//     noise spikes) that are dimmer than the legitimate matched detection.
static void repairFinalMatchCollisions( const QVector<CameraPipelineStarDetection>& starDetections, const QVector<ProjectedCatalogStar>& projectedStars, double matchRadius, QVector<Match>& matches);

// Diagnostic-only (SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_UNMATCHED=1): for each strong
// detection that the final match pass left unmatched, report the nearest projected
// catalog star under the accepted pose -- whether it falls within finalMatchRadius,
// and whether it was instead claimed by a different detection. This distinguishes
// "no catalog star nearby" (candidate-pool/coverage gap) from "the right catalog
// star is there but lost the assignment to another detection" (match collision).
static void debugDumpUnmatchedDetections(const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<ProjectedCatalogStar>& projectedStars, const QVector<Match>& finalMatches, double finalMatchRadius);

static bool debugCatalogStarMatches(const PlateSolveCatalogContext& catalogContext, int catalogIndex);

static bool debugTriangleContainsCatalogStar(const PlateSolveCatalogContext& catalogContext, const std::array<int, 3>& catalogIndices);

static QString debugTriangleAnchorSummary(const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const std::array<int, 3>& detectionIndices, const std::array<int, 3>& catalogIndices);

static SkyProjector createProjector(const CameraSettings& settings, const QSize& size, double azimuthDegrees, double elevationDegrees, double rollDegrees, double fovDegrees, double centerOffsetXPixels = 0.0, double centerOffsetYPixels = 0.0, double distortionK1 = 0.0);

static bool projectVector(const SkyProjector& projector, const SkyVector& vector, QPointF& point);

static bool projectAltAz(const SkyProjector& projector, double azimuthDegrees, double elevationDegrees, QPointF& point);

static bool unprojectPixelToVector(const SkyProjector& projector, const QPointF& point, SkyVector& vector);

static bool rotateBasisToAlignVector(const SkyProjector& projector, const SkyVector& fromVector, const SkyVector& toVector, SkyVector& alignedCenter, SkyVector& alignedRight, SkyVector& alignedUp);

static bool projectorPoseFromBasis(const SkyVector& center, const SkyVector& right, double& azimuthDegrees, double& elevationDegrees, double& rollDegrees);

static SkyVector subtractScaled(const SkyVector& lhs, const SkyVector& rhs, double scale);

static SkyVector combineBasis(const SkyVector& xAxis, const SkyVector& yAxis, const SkyVector& zAxis, double x, double y, double z);

static bool mappedBasisVector(const SkyVector& sourceVector, const SkyVector& sourceXAxis, const SkyVector& sourceYAxis, const SkyVector& sourceZAxis, const SkyVector& targetXAxis, const SkyVector& targetYAxis, const SkyVector& targetZAxis, SkyVector& mappedVector);

static bool poseFromTwoVectorPairs(const SkyProjector& baseProjector, const SkyVector& sourceA, const SkyVector& sourceB, const SkyVector& targetA, const SkyVector& targetB, double& azimuthDegrees, double& elevationDegrees, double& rollDegrees);

static SkyVector rotateVectorByMatrix(const std::array<std::array<double, 3>, 3>& rotation, const SkyVector& vector);

static std::array<std::array<double, 3>, 3> transposeRotationMatrix( const std::array<std::array<double, 3>, 3>& rotation);

static bool vectorPairRotationError(const std::array<std::array<double, 3>, 3>& rotation, const std::array<SkyVector, 3>& sourceVectors, const std::array<SkyVector, 3>& targetVectors, double& rmsAngularErrorDegrees, double& maxAngularErrorDegrees);

static bool normalizeQuaternion(std::array<double, 4>& quaternion);

static std::array<double, 4> largestSymmetric4Eigenvector(const std::array<std::array<double, 4>, 4>& matrix);

static std::array<std::array<double, 3>, 3> rotationMatrixFromQuaternion(const std::array<double, 4>& quaternion);

static bool wahbaRotationFromVectorPairs(const std::array<SkyVector, 3>& sourceVectors, const std::array<SkyVector, 3>& targetVectors, bool transposeCorrelation, std::array<std::array<double, 3>, 3>& rotation, double& rmsAngularErrorDegrees, double& maxAngularErrorDegrees);

static bool poseFromThreeVectorPairs(const SkyProjector& baseProjector, const std::array<SkyVector, 3>& sourceVectors, const std::array<SkyVector, 3>& targetVectors, double& azimuthDegrees, double& elevationDegrees, double& rollDegrees, double* rmsAngularErrorDegrees = nullptr, double* maxAngularErrorDegrees = nullptr);

// N-vector (N>=2) generalizations of vectorPairRotationError / wahbaRotationFromVectorPairs /
// poseFromThreeVectorPairs, used by the quad-hash blind seed engine where each hypothesis
// supplies 4 ray<->sky correspondences. Kept as separate overloads (not refactored in terms
// of the 3-vector versions) to avoid any risk to the existing triangle-seed callers.
static bool vectorPairRotationErrorN(const std::array<std::array<double, 3>, 3>& rotation, const QVector<SkyVector>& sourceVectors, const QVector<SkyVector>& targetVectors, double& rmsAngularErrorDegrees, double& maxAngularErrorDegrees);

static bool wahbaRotationFromVectorPairsN(const QVector<SkyVector>& sourceVectors, const QVector<SkyVector>& targetVectors, bool transposeCorrelation, std::array<std::array<double, 3>, 3>& rotation, double& rmsAngularErrorDegrees, double& maxAngularErrorDegrees);

// Recovers az/el/roll of `baseProjector` rotated so that each sourceVectors[i] (camera-frame
// rays, e.g. from unprojectPixelToVector) maps onto targetVectors[i] (sky vectors, e.g. from
// the visible catalog context). Requires N>=2 correspondences; N>=3 recommended for a
// well-conditioned rotation (a quad gives N=4).
static bool poseFromVectorPairsN(const SkyProjector& baseProjector, const QVector<SkyVector>& sourceVectors, const QVector<SkyVector>& targetVectors, double& azimuthDegrees, double& elevationDegrees, double& rollDegrees, double* rmsAngularErrorDegrees = nullptr, double* maxAngularErrorDegrees = nullptr);

// Continuous vector-space "quad code" (astrometry.net-style), used by the blind quad-hash
// engine for both detection rays (camera frame, post-unprojection) and catalog sky vectors.
// Operates purely on 4 unit vectors so the same code function applies to either frame; the
// resulting (xC, yC, xD, yD) code is invariant under any proper rotation of the 4 vectors,
// which is what lets a detection-ray code be looked up directly against a catalog code index.
struct QuadVectorCode
{
    bool valid = false;
    double xC = 0.0;
    double yC = 0.0;
    double xD = 0.0;
    double yD = 0.0;
    // Great-circle angle between the most-separated pair (A, B), in radians. Only meaningful
    // for scale-aware (mode 1, fov-known) matching; ignored for mode 0.
    double angleABRadians = 0.0;
    // Indices into the input array corresponding to A, B, C, D after canonicalization.
    std::array<int, 4> order = {0, 1, 2, 3};
    // True if the canonical A/B swap (1-x, 1-y flip) was applied.
    bool abSwapped = false;
};

static QuadVectorCode buildVectorQuadCode(const std::array<SkyVector, 4>& vectors);

static bool anchorAlignedPoseFromPixel(const CameraSettings& settings, const QSize& imageSize, const QPointF& anchorPoint, const SkyVector& anchorVector, double baseAzimuthDegrees, double baseElevationDegrees, double baseRollDegrees, double fovDegrees, double centerOffsetXPixels, double centerOffsetYPixels, double distortionK1, double& alignedAzimuthDegrees, double& alignedElevationDegrees, double& alignedRollDegrees);

struct GuidedTrianglePoseSeed
{
    double azimuthDegrees = 0.0;
    double elevationDegrees = 0.0;
    double rollDegrees = 0.0;
    double fovDegrees = 0.0;
};

static bool estimateScreenSimilarity(const std::array<QPointF, 3>& sourcePoints, const std::array<QPointF, 3>& targetPoints, double& scale, double& rotationDegrees, double& rmsErrorPixels);

static QVector<GuidedTrianglePoseSeed> guidedTriangleSimilarityPoseSeeds( const CameraSettings& settings, const QSize& imageSize, const std::array<QPointF, 3>& detectionPoints, const std::array<VisibleCatalogStar, 3>& triangleStars, const std::array<int, 3>& permutation, double baseAzimuthDegrees, double baseElevationDegrees, double seedFovDegrees, double centerOffsetXPixels, double centerOffsetYPixels, double distortionK1);

static QPointF projectorPrincipalPoint(const SkyProjector& projector);

static double pointDistancePixels(const QPointF& lhs, const QPointF& rhs);

static double detectionBrightnessMetric(const CameraPipelineStarDetection& detection);

static bool triangleBrightnessOrderCompatible(const QVector<CameraPipelineStarDetection>& starDetections, const std::array<int, 3>& detectionIndices, const std::array<VisibleCatalogStar, 3>& triangleStars, const std::array<int, 3>& permutation);

static double triangleBrightnessMagnitudeError(const QVector<CameraPipelineStarDetection>& starDetections, const std::array<int, 3>& detectionIndices, const std::array<VisibleCatalogStar, 3>& triangleStars, const std::array<int, 3>& permutation);

static bool isDetectionUsableForBrightPrior(const CameraPipelineStarDetection& detection);

static bool isImplausiblyCompactBrightCatalogDetection(const CameraPipelineStarDetection& detection, double catalogMagnitude, bool narrowGuidedSolve = false);

static bool isNamedSparseGuidedCatalogStar(const CatalogStar& star);

static bool isStrongSparseGuidedDetection(const CameraPipelineStarDetection& detection);

static double detectionReliabilityMetric(const CameraPipelineStarDetection& detection);

static double faintCatalogAssignmentPenalty(double magnitude);

static double brightDetectionFaintCatalogAssignmentPenalty(double detectionBrightnessRank, double catalogMagnitude, bool narrowGuidedSolve);

static double catalogMagnitudeSupportWeight(double magnitude);

static double matchDistanceSupportWeight(double distancePixels, double matchRadiusPixels);

static double narrowGuidedMagnitudePriorityScore(const FinalMatchPassEvaluation& evaluation);

static double prioritySeedRadialAffinity(const FinalMatchPassEvaluation& evaluation, double matchRadiusPixels);

static double prioritySeedProjectedAffinity(const FinalMatchPassEvaluation& evaluation, double matchRadiusPixels);

static double seedRadialMagnitudeCoverageAffinity(const FinalMatchPassEvaluation& evaluation);

static double seedProjectedMagnitudeCoverageAffinity(const FinalMatchPassEvaluation& evaluation);

static bool hasPoorNoRollSeedRadialSupport(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation, bool useSeedProjectedBrightGate);

static bool hasComparableSeedRadialChecks(const FinalMatchPassEvaluation& lhs, const FinalMatchPassEvaluation& rhs);

static bool shouldPreferSeedRadialConsistency(const FinalMatchPassEvaluation& candidate, const FinalMatchPassEvaluation& best, int matchTolerance);

static double narrowGuidedAnchorShapeScore(const CameraPipelineStarDetection& detection);

static double brightGuidedDetectionPriorityScore(const CameraPipelineStarDetection& detection, double brightness, double reliability, double shapeScore);

void prepareDetectionMetricCache(const QVector<CameraPipelineStarDetection>& starDetections);

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

QVector<int> selectDetectionIndicesForSolve(const QVector<CameraPipelineStarDetection>& starDetections, const QSize& imageSize);

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

static void buildProjectedCatalogInto(const PlateSolveCatalogContext& catalogContext, const SkyProjector& projector, double searchMarginPixels, const QVector<int>* allowedCatalogIndices, QVector<ProjectedCatalogStar>& projectedStars);

static QVector<ProjectedCatalogStar> buildProjectedCatalog(const PlateSolveCatalogContext& catalogContext, const SkyProjector& projector, double searchMarginPixels, const QVector<int>* allowedCatalogIndices = nullptr);

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

static RaDecToAzAltParams buildRaDecToAzAltParams(double latitude, double longitude, const QDateTime& captureDateTimeUtc);

// Fast per-star RaDecToAzAlt using precomputed constants.
// Returns false if the star is below the horizon (or non-finite).
static bool raDecToAzAltFast(const RaDecToAzAltParams& p, double rightAscensionDegrees, double declinationDegrees, double& az, double& alt);

static QVector<VisibleCatalogStar> buildVisibleCatalog(const CameraSettings& settings, const QVector<CatalogStar>& catalogStars, const QDateTime& captureDateTimeUtc, double maxMagnitude);

static void buildVisibleStarIndex(const QVector<VisibleCatalogStar>& visibleStars, QHash<int, int>& visibleStarIndexByCatalogIndex);

struct VisibleCatalogCacheEntry
{
    QVector<VisibleCatalogStar> visibleStars;
    QHash<int, int> visibleStarIndexByCatalogIndex;
};

static QMutex& visibleCatalogCacheMutex();

static QHash<QString, VisibleCatalogCacheEntry>& visibleCatalogCache();

static QStringList& visibleCatalogCacheOrder();

static QString visibleCatalogFingerprint(const QVector<CatalogStar>& catalogStars);

static double visibleCatalogCacheMagnitudeLimit(const QVector<CatalogStar>& catalogStars, double maxMagnitude);

static QString visibleCatalogCacheKey(const CameraSettings& settings, const QVector<CatalogStar>& catalogStars, const QDateTime& captureDateTimeUtc, double maxMagnitude);

static bool loadVisibleCatalogCache(const CameraSettings& settings, const QVector<CatalogStar>& catalogStars, const QDateTime& captureDateTimeUtc, double maxMagnitude, QVector<VisibleCatalogStar>& visibleStars, QHash<int, int>& visibleStarIndexByCatalogIndex);

static void storeVisibleCatalogCache(const CameraSettings& settings, const QVector<CatalogStar>& catalogStars, const QDateTime& captureDateTimeUtc, double maxMagnitude, const QVector<VisibleCatalogStar>& visibleStars, const QHash<int, int>& visibleStarIndexByCatalogIndex);

static void populateVisibleCatalogContext(PlateSolveCatalogContext& context, const CameraSettings& settings, const QDateTime& captureDateTimeUtc, double maxMagnitude, bool allowCache);

PlateSolveCatalogContext buildPlateSolveCatalogContext(const CameraSettings& settings, const QSize& imageSize, const QDateTime& captureDateTimeUtc, double maxMagnitude, double catalogLoadMaxMagnitude = -1.0);

void rebuildVisibleCatalogContext(PlateSolveCatalogContext& context, const CameraSettings& settings, const QDateTime& captureDateTimeUtc, double maxMagnitude);

static QVector<VisibleCatalogStar> selectLocalVisibleStars(const QVector<VisibleCatalogStar>& visibleStars, double centerAzimuthDegrees, double centerElevationDegrees, double radiusDegrees, int maxStars);

static QVector<VisibleCatalogStar> selectVisibleStarsNearElevation(const QVector<VisibleCatalogStar>& visibleStars, double centerElevationDegrees, double radiusDegrees, int maxStars);

QVector<GuidedAnchorPair> findGuidedAnchorPairs(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<VisibleCatalogStar>& localVisibleStars);

static TriangleSignature buildTriangleSignature(const std::array<QPointF, 3>& points);

static std::array<QPointF, 4> orderQuadPoints(const std::array<QPointF, 4>& points);

static QuadSignature buildQuadSignature(const std::array<QPointF, 4>& unorderedPoints);

static bool isDistinctiveTriangleSignature(const TriangleSignature& signature);

static bool isDistinctiveQuadSignature(const QuadSignature& signature);

double signatureReliabilityScore(const QVector<CameraPipelineStarDetection>& starDetections, const int *indices, int count);

double signatureBrightnessScore(const QVector<CameraPipelineStarDetection>& starDetections, const int *indices, int count);

static double catalogSignatureBrightnessScore(const QVector<VisibleCatalogStar>& visibleStars, const int *indices, int count);

static qint64 signatureBucketKey(int firstBin, int secondBin);

static int signatureRatioBin(double value, double binWidth);

static QHash<qint64, QVector<int>> buildTriangleSignatureBuckets(const QVector<TriangleSignature>& signatures, double binWidth);

static QHash<qint64, QVector<int>> buildQuadSignatureBuckets(const QVector<QuadSignature>& signatures, double binWidth);

static double medianCandidateDistancePixels(const QVector<Match>& matches);

bool isStrongBlindSeedEvaluation(const CameraSettings& settings, const QVector<int>& detectionIndices, const Evaluation& candidate);

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

Evaluation verifyBlindSeedCandidate(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const Evaluation& candidate);

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

static double seedWrappedAngularDistanceDegrees(double lhs, double rhs);

static double seedCandidateConsensusScore(const Evaluation& candidate);

QVector<Evaluation> selectConsensusSeedRepresentatives(const CameraSettings& settings, const QVector<Evaluation>& seeds, int seedLimit, const char *profilePrefix);

QVector<TriangleSignature> buildDetectionTriangleSignatures(const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, int maxDetectionCount = 16);

bool appendCatalogTriangleSignature(const CameraSettings& settings, const QVector<VisibleCatalogStar>& visibleStars, int i, int j, int k, QVector<TriangleSignature>& signatures);

static qint64 catalogTriangleKey(int a, int b, int c);

QVector<TriangleSignature> buildLocalCatalogTriangleSignatures(const CameraSettings& settings, const QVector<VisibleCatalogStar>& visibleStars);

QVector<TriangleSignature> buildCatalogTriangleSignatures(const CameraSettings& settings, const QVector<VisibleCatalogStar>& visibleStars, int maxCatalogStarCount = -1);

QVector<QuadSignature> buildDetectionQuadSignatures(const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, int maxDetectionCount = 14);

// Unprojects detection pixel centers into unit vectors in a fixed reference camera frame
// (az=0/el=90/roll=0), for use by the vector-space quad-hash blind seed engine. This mirrors
// the brightDetectionVectors pattern used by the bright-pair seeder: the reference frame is
// arbitrary but fixed, so poseFromVectorPairsN can later recover the true az/el/roll by
// rotating this baseProjector's center/right vectors into the catalog (sky) frame.
// detectionRayVectors and detectionRayVectorValid are parallel to detectionIndices.
static SkyProjector buildDetectionRayVectors(const CameraSettings& settings, const QSize& imageSize, double referenceFovDegrees, double centerOffsetXPixels, double centerOffsetYPixels, double distortionK1, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, QVector<SkyVector>& detectionRayVectors, QVector<quint8>& detectionRayVectorValid);

// A single quad in the catalog quad-code index: its vector code (buildVectorQuadCode output,
// optionally extended with the AB great-circle angle for scale-aware mode-1 matching) plus
// the visibleStars indices of its four corner stars in canonical (A,B,C,D) order.
struct CatalogQuadCodeEntry
{
    double code[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<int, 4> visibleStarIndices = {{-1, -1, -1, -1}};
};

// Uniform-grid spatial index over CatalogQuadCodeEntry::code, supporting epsilon-ball range
// queries. This plays the role of the "small in-house 4D/5D k-d tree" from the design notes:
// a fixed-size grid hash gives the same epsilon-ball lookup with much less code than a
// balanced tree, and follows the same QHash bucketing pattern already used by
// buildQuadSignatureBuckets for the legacy 2D quad matcher.
struct CatalogQuadCodeIndex
{
    QVector<CatalogQuadCodeEntry> entries;
    QHash<quint64, QVector<int>> buckets; // grid cell key -> indices into entries
    double cellSize = 0.02;
    int dimensions = 4; // 4 (mode 0, fov unknown) or 5 (mode 1, includes AB angle)

    quint64 cellKey(const std::array<qint64, 5>& cell) const
    {
        quint64 key = 0;
        for (int d = 0; d < dimensions; ++d) {
            // Pack each dimension into 12 bits (range +/-2048 cells); with cellSize=0.02
            // that covers code values out to +/-40, well beyond any valid quad code.
            key = (key << 12) | (static_cast<quint64>(cell[d] + 2048) & 0xFFFu);
        }
        return key;
    }

    std::array<qint64, 5> cellOf(const double* code) const
    {
        std::array<qint64, 5> cell{};
        for (int d = 0; d < dimensions; ++d) {
            cell[d] = static_cast<qint64>(std::floor(code[d] / cellSize));
        }
        return cell;
    }

    void addEntry(const CatalogQuadCodeEntry& entry)
    {
        const int entryIndex = entries.size();
        entries.append(entry);
        buckets[cellKey(cellOf(entry.code))].append(entryIndex);
    }

    // Calls visitor(entryIndex) for every entry whose code lies within epsilon (L-infinity)
    // of target across all dimensions.
    template <typename Visitor>
    void queryEpsilonBall(const double* target, double epsilon, Visitor&& visitor) const
    {
        const qint64 cellSpan = std::max<qint64>(1, static_cast<qint64>(std::ceil(epsilon / cellSize)));
        const qint64 span = 2 * cellSpan + 1;
        std::array<qint64, 5> baseCell{};
        for (int d = 0; d < dimensions; ++d) {
            baseCell[d] = static_cast<qint64>(std::floor(target[d] / cellSize)) - cellSpan;
        }
        qint64 totalCells = 1;
        for (int d = 0; d < dimensions; ++d) {
            totalCells *= span;
        }

        for (qint64 linear = 0; linear < totalCells; ++linear)
        {
            qint64 remainder = linear;
            std::array<qint64, 5> cell{};
            for (int d = 0; d < dimensions; ++d) {
                cell[d] = baseCell[d] + (remainder % span);
                remainder /= span;
            }
            const auto it = buckets.find(cellKey(cell));
            if (it == buckets.end()) {
                continue;
            }
            for (int entryIndex : it.value())
            {
                bool withinEpsilon = true;
                for (int d = 0; d < dimensions; ++d) {
                    if (std::fabs(entries[entryIndex].code[d] - target[d]) > epsilon) {
                        withinEpsilon = false;
                        break;
                    }
                }
                if (withinEpsilon) {
                    visitor(entryIndex);
                }
            }
        }
    }
};

// Builds the vector-space quad-code index over the visible catalog for the blind quad-hash
// engine (design notes item 3). Stars are bucketed into ~10deg az/el cells and the brightest
// few per cell are kept as quad anchors; each anchor is combined with 3-of-k of its
// angularly-nearest neighbours (restricted to the ~2-70deg scale band) to form quads, capped
// per-anchor and overall so the index stays bounded (~50k quads) for large catalogs.
static CatalogQuadCodeIndex buildCatalogQuadCodeIndex(const QVector<VisibleCatalogStar>& visibleStars, bool includeScaleDimension, int brightPoolLimit = 0);

QVector<QuadSignature> buildCatalogQuadSignatures(const CameraSettings& settings, const QVector<VisibleCatalogStar>& visibleStars);

QVector<Evaluation> buildBlindTriangleSeeds(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<VisibleCatalogStar>& visibleStars, const QVector<int>* signatureDetectionIndicesOverride = nullptr, int maxDetectionSignatureCount = -1, int maxCatalogSignatureStars = -1, int requiredAnchorMatches = 2, int seedLimitOverride = -1);

QVector<Evaluation> buildBrightGuidedTriangleSeeds(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<VisibleCatalogStar>& visibleStars);

QVector<Evaluation> buildBrightGuidedAnchorTriangleSeeds(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<VisibleCatalogStar>& visibleStars);

QVector<Evaluation> buildBrightPairSeeds(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<VisibleCatalogStar>& visibleStars);

// Vector-space quad-hash blind seed engine (design notes "Blind quad-hash geometric
// indexing" plan, items 1-6). Mode 1 only (FoV known/trusted): unprojects detection pixels
// into camera-frame rays at the known FoV, builds vector quad codes for groups of 4
// detections, looks them up in a catalog quad-code index built from the visible sky, and
// recovers az/el/roll for each code match via the N-vector Wahba solver. For mode 0 (unknown
// FoV) the caller sweeps a set of candidate FoVs via seedFovOverride; the wide-fallback grid
// below is the final blind-search backstop.
QVector<Evaluation> buildVectorQuadBlindSeeds(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<VisibleCatalogStar>& visibleStars, double seedFovOverride = -1.0);

static quint64 spatialCellKey(int x, int y);

static double angularDistanceDegrees(double lhs, double rhs);

const QVector<double>& detectionBrightnessRanks(const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices);

const QVector<double>& projectedBrightnessRanks(const QVector<ProjectedCatalogStar>& projectedStars);

double matchBrightnessRankError(const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<ProjectedCatalogStar>& projectedStars, const QVector<Match>& matches);

static double meanCatalogMagnitudeForMatches(const QVector<CatalogStar>& catalogStars, const QVector<Match>& matches);

bool shouldPopulatePoseScoringMetrics(const CameraSettings& /*settings*/,
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

void populatePoseScoringMetrics(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<ProjectedCatalogStar>& projectedStars, const QVector<CatalogStar>& catalogStars, Evaluation& evaluation, bool forceSparseSeedMetrics = false);

QVector<Match> buildMatches(const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<ProjectedCatalogStar>& projectedStars, double matchRadiusPixels);

static void appendSupplementalMatches(const QVector<CameraPipelineStarDetection>& starDetections, const QVector<ProjectedCatalogStar>& projectedStars, double matchRadiusPixels, const QVector<int>* supplementalDetectionIndices, bool narrowGuidedSolve, bool allowTightArtifactCoincidence, QVector<Match>& matches);

static void appendWideBrightSupplementalMatches(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<ProjectedCatalogStar>& projectedStars, const QSize& imageSize, double matchRadiusPixels, QVector<Match>& matches);

void logUnmatchedDetections(const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<ProjectedCatalogStar>& projectedStars, const QVector<Match>& matches, double matchRadiusPixels);

Evaluation evaluatePose(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, double azimuthDegrees, double elevationDegrees, double rollDegrees, double fovDegrees, const QVector<int>* allowedCatalogIndices = nullptr, double centerOffsetXPixels = 0.0, double centerOffsetYPixels = 0.0, double distortionK1 = 0.0, double matchRadiusOverride = -1.0);

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
void buildBlindGridCache(const PlateSolveCatalogContext& catalogContext, const SkyProjector& refProjector, const QVector<int>* allowedCatalogIndices = nullptr);

// Rotate the roll=0 offsets by rollDegrees and populate
// m_blindGridProjectedScratch with only the stars that fall within the
// expanded image bounds (image rect + matchRadiusPixels margin).
void populateBlindGridProjectedCatalog(double rollDegrees, double matchRadiusPixels, const SkyProjector& refProjector);

// Evaluate a pose using the pre-rotated catalog already stored in
// m_blindGridProjectedScratch.  Mirrors evaluatePose but skips the
// createProjector + buildProjectedCatalogInto steps.
Evaluation evaluatePoseFromPrecomputedCatalog( const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, double azimuthDegrees, double elevationDegrees, double rollDegrees, double fovDegrees, double centerOffsetXPixels, double centerOffsetYPixels, double distortionK1, double matchRadiusPixels);

Evaluation evaluateAnchoredPose(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<int>& allowedCatalogIndices, const GuidedAnchorPair& anchor, double azimuthDegrees, double elevationDegrees, double rollDegrees, double fovDegrees, double centerOffsetXPixels, double centerOffsetYPixels, double distortionK1, double matchRadiusPixels);

static double medianDistancePixels(const QVector<Match>& matches);

static double maxDistancePixels(const QVector<Match>& matches);

// RMS ceiling for a sparse guided-anchor solve, relaxed as more matches accumulate
// (more matches => a high RMS is less likely to be a coincidence). Shared by the
// pre-final evaluation gate and the final-pass ranking gate so the ladder stays in
// one place.
static double sparseGuidedMaxRms(const CameraSettings& settings, int matchCount);

bool isAcceptableSparseGuidedPairEvaluation(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const Evaluation& evaluation);

bool isAcceptableSparseGuidedPairFinalPass(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const FinalMatchPassEvaluation& finalPass);

Evaluation promoteSparseGuidedPairFromMatches(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const Evaluation& candidate);

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
static bool hasDenseFinalEvidenceOverridingSeedRadial(const CameraSettings& settings, const FinalMatchPassEvaluation& finalPass);

// Named-bright-anchor certificate: a pose that fits >= 2 named bright anchors at
// <= 3 px rms, fits a dense match set at <= 3 px rms, and stays within the
// pointing-error budget of the seed direction is geometrically verified far beyond what
// a chance alignment can produce (contaminated candidates fit the same anchors at
// >= 9 px and sit > 4 degrees from the seed). The per-branch bright-support heuristics
// can falsely veto such a pose: the seed-reference-projector checks (seed-radial /
// seed-projected) assume the recorded pointing is accurate to a few hundredths of a
// degree with near-exact roll, when it is only trusted to ~1 degree (and roll is an
// unused placeholder for start modes without it); and matchedBrightProjectedStars
// undercounts in dense catalogs because a bright star's detection can be *assigned* to
// a nearby fainter catalog star even when the bright star projects sub-2px onto it.
bool hasNamedBrightAnchorCertifiedPose(const CameraSettings& settings,
                                       const FinalMatchPassEvaluation& finalPass) const
{
    constexpr double kMaxSeedDirectionOffsetDegrees = 1.5;
    constexpr double kMaxNamedAnchorRmsPixels = 3.0;
    constexpr double kMaxOverallRmsPixels = 3.0;
    return isNarrowField(settings)
        && finalPass.projectorValid
        && (finalPass.namedBrightAnchorMatches >= 2)
        && std::isfinite(finalPass.namedBrightAnchorRmsErrorPixels)
        && (finalPass.namedBrightAnchorRmsErrorPixels <= kMaxNamedAnchorRmsPixels)
        && std::isfinite(finalPass.rmsErrorPixels)
        && (finalPass.rmsErrorPixels <= kMaxOverallRmsPixels)
        && (finalPass.finalMatches.size()
            >= static_cast<qsizetype>(std::max(settings.m_plateSolveMinMatches + 50, 80)))
        && (directionSeedAngularDistanceDegrees(finalPass.pose) <= kMaxSeedDirectionOffsetDegrees);
}

// Sparse-tight certificate: the shallow-catalog analogue of the named-bright-anchor
// certificate above, for fields whose detection set is too sparse to ever reach the
// dense certificate's >= 80 matches (e.g. a short-exposure frame with ~40 detections).
// Against a bright-only catalog a correct pose matches a *sparse* set - but every
// match is tight and the set includes a named bright anchor pinning the field. A
// wrong-roll alias can keep the anchor itself tight (any rotation about the anchor
// does) yet cannot place the remaining bright stars: measured on sadr.jpg @mag10 the
// true pose fits 4/4 at rms 1.44 / max 1.73 px while the best alias manages rms 4.77 /
// max 8.67. The whole-set max bound is the load-bearing part: a pose that also picks
// up coincidental matches anywhere inside the (much larger) final match radius fails
// it, so this cannot certify a pose riding on loose faint coincidences.
bool hasSparseTightBrightCertifiedPose(const CameraSettings& settings,
                                       const FinalMatchPassEvaluation& finalPass) const
{
    constexpr double kMaxSeedDirectionOffsetDegrees = 1.5;
    constexpr double kMaxNamedAnchorRmsPixels = 3.0;
    constexpr double kMaxOverallRmsPixels = 2.0;
    constexpr double kMaxWorstMatchPixels = 3.0;
    return isNarrowField(settings)
        && isLowMagnitudeNarrowGuidedSolve(settings)
        && finalPass.projectorValid
        && (finalPass.namedBrightAnchorMatches >= 1)
        && std::isfinite(finalPass.namedBrightAnchorRmsErrorPixels)
        && (finalPass.namedBrightAnchorRmsErrorPixels <= kMaxNamedAnchorRmsPixels)
        && (finalPass.finalMatches.size()
            >= static_cast<qsizetype>(std::max(settings.m_plateSolveMinMatches, 4)))
        && std::isfinite(finalPass.rmsErrorPixels)
        && (finalPass.rmsErrorPixels <= kMaxOverallRmsPixels)
        && std::isfinite(finalPass.maxErrorPixels)
        && (finalPass.maxErrorPixels <= kMaxWorstMatchPixels)
        && (directionSeedAngularDistanceDegrees(finalPass.pose) <= kMaxSeedDirectionOffsetDegrees);
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
// Ablation harness hook (WS3 acceptance-layer consolidation): SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_GATE
// is a comma-separated list of acceptance-gate tokens to neutralise (force each to its permissive /
// non-rejecting value), so the marginal contribution of every accept/reject gate to the corpus +
// negative suites can be measured one at a time, exactly like the seed-engine ablation. Inert unless
// the env var is set (read once, cached). See doc/camera/plate-solver-notes.md "WS3".
static bool gateAblationDisabled(const char* token);

static double poseFalseAlarmLogOdds(const PlateSolveCatalogContext& catalogContext, const FinalMatchPassEvaluation& finalPass, const QSize& imageSize, double matchRadiusPixels, int detectionCount);

// Track 1a (SHADOW, not yet gating): astrometry.net-style verification log-odds. Unlike
// poseFalseAlarmLogOdds (a per-match SUM that grows with match count, so dense and sparse solves
// aren't comparable) this is a per-DETECTION foreground/background mixture with a distractor
// fraction (Sutherland-Saunders): every matched detection contributes a bounded +ve foreground vs
// background log-ratio, and every UNMATCHED detection contributes log(distractorFraction) < 0, so a
// dense wrong solve that leaves most detections unmatched is penalised. Recorded to the profile
// (verify.mixtureLogOddsMilli) for corpus comparison against the heuristic accept/reject bands.
static double poseVerificationLogOdds(const PlateSolveCatalogContext& catalogContext, const FinalMatchPassEvaluation& finalPass, const QSize& imageSize, double matchRadiusPixels, int detectionCount);

// (WS3 2026-06-19) hasOverwhelmingFaintGuidedSupport was removed: it bypassed the brightness
// acceptance gates for faint anchor-less fields, but ablation showed it casts no deciding vote on
// REAL + RAND2 (the faint synthetic regime) + FISHEYE + negatives, even jointly with the other two
// accept-bypasses. If a real faint/high-galactic-latitude field is later wrongly rejected for weak
// bright support, this is the bypass to reinstate (git history / doc/camera/plate-solver-notes.md).

bool hasWeakNarrowGuidedBrightSupport(const CameraSettings& settings, const FinalMatchPassEvaluation& finalPass);

bool isAcceptableSparseGuidedRankingFinalPass(const CameraSettings& settings, const FinalMatchPassEvaluation& finalPass);

bool hasStrongDenseNarrowGuidedFinalPass(const CameraSettings& settings, const FinalMatchPassEvaluation& finalPass);

static double detectionMatchWeight(const CameraPipelineStarDetection& detection);

static double weightedRmsDistancePixels(const QVector<CameraPipelineStarDetection>& starDetections, const QVector<Match>& matches);

static bool hasGeometricallyConsistentMatches(const QVector<CameraPipelineStarDetection>& starDetections, const QVector<ProjectedCatalogStar>& projectedStars, const QVector<Match>& matches, double matchRadiusPixels);

static QVector<Match> rejectOutlierMatches(const QVector<Match>& matches, int minMatches, double matchRadiusPixels, int* outlierCount = nullptr);

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
    // Number of largest residuals to ignore when applying the maxWorst gate. A wide
    // fisheye projects its rim stars at very high field angles, so even a correct solve
    // can have a couple of extreme-rim residuals; gating on the literal single worst
    // star then vetoes an otherwise-excellent dense solve. When > 0 the worst gate is
    // applied to the worst residual *after* dropping this many of the largest, so a lone
    // rim outlier cannot sink the solve while RMS/median still carry false-positive
    // protection. 0 keeps the strict absolute-maximum behaviour.
    int worstTrimCount = 0;
};

// Worst residual after discarding the `dropCount` largest matches. Falls back to the
// absolute maximum when there are too few matches to spare any (so sparse solves stay
// strict).
static double trimmedWorstDistancePixels(const QVector<Match>& matches, int dropCount);

static bool passesResidualGates(const QVector<Match>& matches, double rmsErrorPixels, double maxErrorPixels, const ResidualGates& gates);

static bool isAcceptableBlindSolve(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<Match>& matches, double rmsErrorPixels, double maxErrorPixels);

static bool isAcceptableSparseWideBlindSolve(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const FinalMatchPassEvaluation& evaluation);

static bool isStrongGuidedSolve(const CameraSettings& settings, int minMatchCount, const Evaluation& evaluation);

static int minimumDirectionSeedAcceptedMatches(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections);

static double maxDirectionSeedRmsError(const CameraSettings& settings, int matchCount);

static double maxNarrowGuidedFovDeltaDegrees(const CameraSettings& settings);

static bool isAcceptableNarrowGuidedFov(const CameraSettings& settings, double fovDegrees);

static QString narrowGuidedFovRejectionReason(const CameraSettings& settings, double fovDegrees);

// Seed-distance plausibility gate for narrow direction-seeded solves: the accepted centre
// must lie within seed-error range of the run's seed direction. Legitimate pointing errors
// are sub-FoV and the recenter ladder itself only roams +/- 1 FoV per hop, while
// rotation-about-anchor aliases can land many FoV away (a false mode-4 accept sat 7.3 deg =
// 5.7 FoV from its seed) - such a pose is on a different star field and the seed's whole
// purpose is gone.
static double maxDirectionSeedDistanceDegrees(const CameraSettings& settings);

static double directionSeedSeparationDegrees(const CameraSettings& settings, double azimuthDegrees, double elevationDegrees);

static bool isAcceptableDirectionSeedDistance(const CameraSettings& settings, double azimuthDegrees, double elevationDegrees);

static QString directionSeedDistanceRejectionReason(const CameraSettings& settings, double azimuthDegrees, double elevationDegrees);

// Residual gates for a direction-seeded solve. Shared by isAcceptableDirectionSeedSolve()
// and directionSeedRejectionReason() so the accept decision and the human-readable
// reason can never drift apart.
//
// Narrow-field (telescope) solves pin the FoV, so residuals reflect real centroid noise
// -> looser 0.75x median gate. Wide-field fisheye distortion at large angles raises
// residuals even for correct solves, so the median gate is 0.65x (= 15.6 px at a 24 px
// radius) to accept those while still rejecting clearly wrong ones.
static ResidualGates directionSeedResidualGates(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, int matchCount);

static QString directionSeedRejectionReason(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<Match>& matches, double rmsErrorPixels, double maxErrorPixels);

static bool isAcceptableDirectionSeedSolve(const CameraSettings& settings, int minMatchCount, const Evaluation& evaluation);

static bool isAcceptableDirectionSeedSolve(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<Match>& matches, double rmsErrorPixels, double maxErrorPixels);

static bool isAcceptableElevationSeedEvaluation(const CameraSettings& settings, int minMatchCount, const Evaluation& evaluation);

static bool isAcceptableElevationSeedSolve(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<Match>& matches, double rmsErrorPixels, double maxErrorPixels);

double evaluationRmsQuality(const Evaluation& evaluation, double normalizationRadius);

double brightnessAffinity(const Evaluation& evaluation);

double catalogMagnitudeAffinity(const Evaluation& evaluation);

double wideEvaluationMatchWeight(const Evaluation& evaluation);

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

bool hasAcceptableBrightnessConsistency(const Evaluation& evaluation);

bool hasAcceptableGuidedFinalBrightnessConsistency(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation);

bool needsWideBrightAnchorSupport(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections);

bool hasAcceptableWideBrightAnchorSupport(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const FinalMatchPassEvaluation& evaluation);

bool isStrongEarlyGuidedFinalPass(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const FinalMatchPassEvaluation& evaluation, int requiredMatches, double finalMatchRadius);

double directionSeedAffinity(const Evaluation& evaluation);

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

double fovSeedAffinity(const Evaluation& evaluation);

double allSkyZenithAffinity(const Evaluation& evaluation);

double guidedDirectionEvaluationScore(const Evaluation& evaluation, double normalizationRadius = -1.0);

// Aggregate "how much non-zero lens calibration this pose carries" — smaller is
// preferred as a tie-break so the solver does not invent center-offset/distortion
// it does not need. The 100x weight puts distortion k1 (a small dimensionless
// value) on a comparable footing with the pixel offsets.
static double calibrationMagnitude(const Evaluation& evaluation);

// Deterministic geometric tie-break shared by the evaluation comparators: prefer
// more matches, then lower RMS, then less lens calibration, then the narrower FoV
// (or smaller roll at equal FoV). Callers handle validity and any higher-priority
// scoring before delegating here. Assumes both evaluations are valid.
static bool isBetterByGeometricTieBreak(const Evaluation& candidate, const Evaluation& best);

static bool isBetterEvaluation(const Evaluation& candidate, const Evaluation& best);

double weakModeEvaluationScore(const Evaluation& evaluation, double normalizationRadius = -1.0);

bool isBetterWeakModeEvaluation(const Evaluation& candidate, const Evaluation& best);

bool isBetterGuidedDirectionEvaluation(const Evaluation& candidate, const Evaluation& best);

bool isBetterWeakModeRefinedEvaluation(const Evaluation& candidate, const Evaluation& best);

FinalMatchPassEvaluation evaluateFinalMatchPass(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const Evaluation& candidate, double finalMatchRadius, bool restrictSupplementalMatchesToDetectionIndices = false);

void logFinalMatchPassEvaluation(const char *stage, const FinalMatchPassEvaluation& evaluation, bool best = false);

bool isBetterFinalPassEvaluation(const Evaluation& candidate, const Evaluation& best, int retainedMatchThreshold, bool useGuidedDirectionScoring);

int minimumRetainedMatchesForFinalPass(const Evaluation& reference, int minMatchCount);

double wideFinalPassMatchWeight(const CameraSettings& settings, int matchCount);

double wideFinalPassMagnitudeAffinity(const FinalMatchPassEvaluation& evaluation);

double brightDetectionCoverageAffinity(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation);

double brightProjectedCoverageAffinity(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation);

double brightDetectionMagnitudeAffinity(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation);

double projectedMagnitudeCoverageAffinity(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation);

double narrowGuidedBrightConsistencyScore(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation);

double narrowGuidedSeedConsistencyScore(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation);

QString finalPassBrightDiagnosticSummary(const FinalMatchPassEvaluation& evaluation);

bool hasHighConfidenceSparseGuidedAnchors(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation);

bool hasHighConfidenceGuidedTriangleSupport(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation);

double sparseGuidedAnchorRankingScore(const FinalMatchPassEvaluation& evaluation);

double namedBrightAnchorEvidenceScore(const FinalMatchPassEvaluation& evaluation);

double finalMatchPassScore(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation);

// P-C accessors: return the cached derived value when evaluateFinalMatchPass populated it,
// otherwise compute it directly. Bit-identical; only avoids the per-comparison recomputation.
bool cachedHasStrongDenseNarrowGuidedFinalPass(const CameraSettings& settings, const FinalMatchPassEvaluation& e)
{
    return e.cachedGatesValid ? e.cachedStrongDenseNarrowGuided : hasStrongDenseNarrowGuidedFinalPass(settings, e);
}
double cachedFinalMatchPassScore(const CameraSettings& settings, const FinalMatchPassEvaluation& e)
{
    return e.cachedGatesValid ? e.cachedFinalMatchPassScore : finalMatchPassScore(settings, e);
}
double cachedNarrowGuidedBrightConsistencyScore(const CameraSettings& settings, const FinalMatchPassEvaluation& e)
{
    return e.cachedGatesValid ? e.cachedNarrowGuidedBrightConsistencyScore : narrowGuidedBrightConsistencyScore(settings, e);
}
double cachedNarrowGuidedSeedConsistencyScore(const CameraSettings& settings, const FinalMatchPassEvaluation& e)
{
    return e.cachedGatesValid ? e.cachedNarrowGuidedSeedConsistencyScore : narrowGuidedSeedConsistencyScore(settings, e);
}

double narrowGuidedEvidenceScore(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation);

double finalMatchPassEvidenceScore(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation);

bool isBetterWeakModeFinalMatchPass(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, bool blindMode, const FinalMatchPassEvaluation& candidate, const FinalMatchPassEvaluation& best);

bool hasCompetitiveRollAlias(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const FinalMatchPassEvaluation& winner, double finalMatchRadius, QString *reason, FinalMatchPassEvaluation *betterAlias = nullptr);

bool isBetterEvaluationForMode(const Evaluation& candidate, const Evaluation& best, bool useWeakModeScoring, bool useGuidedDirectionScoring = false);

double guidedAnchorEvaluationScore(const Evaluation& evaluation, double matchRadiusPixels);

double guidedAnchorSearchScore(const Evaluation& evaluation, const GuidedAnchorPair& anchor, double matchRadiusPixels);

bool isBetterGuidedAnchorEvaluation(const Evaluation& candidate, const Evaluation& best, double matchRadiusPixels);

Evaluation searchGuidedAnchorPose(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, QVector<Evaluation> *candidatePool = nullptr);

// Bright-detection-anchored verifier rescue: a failure-path-only fallback for narrow
// guided-direction solves about to be rejected. The normal anchor/triangle/quad seed
// generators all require >=3 mutually-consistent matches to "verify" a seed before it is
// refined - in a sparse or contaminated field the true bright-aligned pose can fail that
// gate at every roll, so it is generated but never survives to the acceptance check
// (narrow-7: triangleVerifiedSeeds=0, quadVerifiedSeeds=0, guidedAnchorTriangleVerifiedSeeds=0
// even though the named anchor IS detected at its catalog position). This routine instead
// pairs each plausible bright detection with each plausible bright catalog star directly -
// bypassing seed verification entirely - sweeps roll, refines with the same LM step the
// guided-anchor search uses, and ranks the results with poseFalseAlarmLogOdds (the
// Poisson count-surprise + bright-rarity statistic already proven to separate true
// alignments from coincidental ones, see poseFalseAlarmLogOdds). It returns only the
// single best-ranked candidate; the caller re-validates it through the unchanged
// acceptance gate, so this can only ever surface a pose that the existing, already-tuned
// gate independently agrees is acceptable.
FinalMatchPassEvaluation searchBrightAnchorVerifierRescue(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, double finalMatchRadius);

static bool sameEvaluationBasin(const Evaluation& lhs, const Evaluation& rhs);

void insertDistinctEvaluationCandidate(QVector<Evaluation>& candidates, const Evaluation& candidate, int maxCandidates, bool useWeakModeScoring = false, const char *stage = nullptr, int interestingMatchCount = 0, int minPoolMatchCount = 3, bool useGuidedDirectionScoring = false);

Evaluation rescoreWeakModeCandidateWithDistortionSweep(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const Evaluation& candidate);

void logPlateSolveEvaluation(const char *stage, const Evaluation& evaluation, bool isNewBest = false, bool forceLog = false);

static bool sameEvaluationIdentity(const Evaluation& lhs, const Evaluation& rhs);

void logWeakModePoolDecision(const char *stage, const char *decision, const Evaluation& candidate, double poolQualityRadius, const Evaluation *other = nullptr);

void logWeakModeCandidatePool(const char *stage, const QVector<Evaluation>& candidates);

Evaluation searchBestPose(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, QVector<Evaluation>* candidatePool = nullptr);

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

static double normalizeSignedDegrees(double value);

static PlateSolveLmPose poseFromEvaluation(const Evaluation& evaluation);

static void clampPlateSolveLmPose(const QSize& imageSize, PlateSolveLmPose& pose);

static double plateSolveLmParameterValue(const PlateSolveLmPose& pose, PlateSolveLmParameter parameter);

// WS2: rotation-vector LM orientation parameterization. Enabled by
// SDRANGEL_CAMERA_PLATE_SOLVER_ROTVEC_LM (default OFF, so the legacy az/el/roll coordinate path is
// byte-for-byte unchanged when the env var is unset). When on, the LM's orientation deltas are
// applied as small rotations about the *camera-frame* axes (pitch/yaw/roll) instead of additions to
// the az/el/roll coordinates. The three camera axes are always orthonormal, so the orientation
// Jacobian stays well-conditioned even near zenith where az and roll rotate about the same (vertical)
// axis and the coordinate parameterization is degenerate.
static bool rotVecLmEnabled();

// Rot-vec is applied only to DIRECTION-SEEDED (guided) solves. Blind / fov-only solves keep the
// legacy LM: a global rot-vec default regressed the blind-fisheye corpus (FISHEYE-full 56->51),
// whereas guided-only keeps the near-zenith payoff (wide-7/8/9 pin retired) and is neutral on
// REAL/RAND2/WIDE/FISHEYE-full (only the approved guided FISH4 -2 remains).
static bool rotVecLmActive(const CameraSettings& settings);

// Camera basis from az/el/roll, matching createProjector's exact convention.
static void lmBasisFromAzElRoll(double azimuthDegrees, double elevationDegrees, double rollDegrees, SkyVector& center, SkyVector& right, SkyVector& up);

// Exact inverse of lmBasisFromAzElRoll: recover az/el/roll from a camera basis.
static void lmAzElRollFromBasis(const SkyVector& center, const SkyVector& right, double& azimuthDegrees, double& elevationDegrees, double& rollDegrees);

// Rotate the pose orientation by deltaDegrees about a camera-frame axis: 0=pitch(right),
// 1=yaw(up), 2=roll(boresight/center). Singularity-free regardless of elevation.
static void lmRotateOrientationCameraFrame(double& azimuthDegrees, double& elevationDegrees, double& rollDegrees, int cameraAxis, double deltaDegrees);

static void addPlateSolveLmParameterDelta(const QSize& imageSize, PlateSolveLmPose& pose, PlateSolveLmParameter parameter, double delta, bool useRotVec);

static double plateSolveLmFiniteDifferenceStep(const PlateSolveLmPose& pose, PlateSolveLmParameter parameter);

static double plateSolveLmMaximumStep(const QSize& imageSize, const PlateSolveLmPose& pose, PlateSolveLmParameter parameter);

static double robustPlateSolveLmWeight(double residualNormPixels, double thresholdPixels);

static bool solveSmallLinearSystem(double matrix[PlateSolveLmParameterCount][PlateSolveLmParameterCount], double rhs[PlateSolveLmParameterCount], double solution[PlateSolveLmParameterCount], int size);

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

static QVector<int> detectionIndicesForMatches(const QVector<Match>& matches);

PlateSolveLmEvaluation evaluateFixedPlateSolveLmPose(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<Match>& fixedMatches, const QVector<int>& rankDetectionIndices, const PlateSolveLmPose& inputPose, double robustThresholdPixels, const Evaluation& seedEvaluation, bool populateScoringMetrics = true);

Evaluation runPlateSolveLmRefinement(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<Match>& fixedMatches, const QVector<int>& rankDetectionIndices, const Evaluation& seedEvaluation, std::array<bool, PlateSolveLmParameterCount> activeParameters, double matchRadiusPixels);

Evaluation refineGuidedAnchorSeedWithLm(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<int>& allowedCatalogIndices, const GuidedAnchorPair& anchor, const Evaluation& seedEvaluation, double centerOffsetXPixels, double centerOffsetYPixels, double distortionK1, double matchRadiusPixels, bool refineFov);

QVector<Match> rebuildRefinementMatchesAtPose(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const Evaluation& pose, double matchRadiusPixels, const GuidedAnchorPair *forcedAnchor = nullptr);

Evaluation refinePoseFromMatches(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const Evaluation& initialEvaluation, bool forceFovRefine = false);

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
FinalMatchPassEvaluation tightenNarrowFinalPass(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const FinalMatchPassEvaluation& original, double finalMatchRadius);

CandidateRefinementResult refineMultiHypothesisCandidate(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<int>& allDetectionIndices, const Evaluation& candidate, int weakModeRefineMinMatches, double finalMatchRadius, bool rankFinalPassWithSelectedDetections, bool useStartDirection);

static void clearSolvedStars(QVector<CameraPipelineStarDetection>& starDetections);

};

#endif // INCLUDE_FEATURE_CAMERAPLATESOLVERINTERNAL_H_
