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

#ifndef INCLUDE_FEATURE_CAMERAPLATESOLVER_H_
#define INCLUDE_FEATURE_CAMERAPLATESOLVER_H_

#include <QByteArray>
#include <QDateTime>
#include <QElapsedTimer>
#include <QHash>
#include <QMutex>
#include <QObject>
#include <QSize>
#include <QString>
#include <QVector>

#include <atomic>
#include <limits>

#include "camerapipelineframe.h"
#include "camerasettings.h"

/**
 * \brief Outcome of a plate-solve attempt: the recovered pointing solution plus match diagnostics.
 *
 * Holds whether a solution was found and, if so, the telescope pointing it implies — azimuth,
 * elevation, roll and field of view, together with lens centre offset and distortion (k1). Also
 * carries rich diagnostics: how many detected/catalog stars were considered and matched, RMS/max
 * residual errors, the catalog source used, a human-readable failure reason and match/profile
 * summaries, and several internal quality/consistency scores used to grade and accept a solve.
 */
struct CameraPlateSolveResult
{
    bool m_solved = false;
    // True when this solve was obtained on the horizontally-mirrored (handedness-flipped)
    // detection set — i.e. the image is a mirror of the modelled sky (up-looking all-sky
    // fisheye cameras image the sky reflected, which the orientation-preserving projector
    // cannot otherwise fit). The recovered az/el/roll pose is expressed in the mirrored
    // frame; any projector built to overlay this pose back onto the ORIGINAL image must
    // reflect pixel x about the image centre (SkyProjector::mirrorX). Detection positions
    // returned to the caller are already restored to original-image coordinates.
    bool m_mirrored = false;
    int m_matchedStars = 0;
    int m_detectedStarsConsidered = 0;
    int m_catalogStarsLoaded = 0;
    int m_catalogCandidateStars = 0;
    int m_outlierStars = 0;
    double m_rmsErrorPixels = 0.0;
    double m_maxErrorPixels = 0.0;
    double m_azimuthDegrees = 0.0;
    double m_elevationDegrees = 0.0;
    double m_rollDegrees = 0.0;
    double m_fovDegrees = 0.0;
    double m_centerOffsetXPixels = 0.0;
    double m_centerOffsetYPixels = 0.0;
    double m_distortionK1 = 0.0;
    QString m_catalogSource;
    QString m_failureReason;
    QString m_matchSummary;
    QString m_profileSummary;
    int m_requiredMatches = 0;
    double m_solverQualityScore = 0.0;
    double m_seedConsistencyScore = 0.0;
    int m_namedBrightAnchorMatches = 0;
    double m_namedBrightAnchorRmsErrorPixels = std::numeric_limits<double>::infinity();
    double m_seedRadialMagnitudeMatchFraction = 1.0;
    int m_prioritySeedProjectedChecks = 0;
    double m_prioritySeedProjectedErrorPixels = std::numeric_limits<double>::infinity();
    // Bright-star agreement for the selected pose. These are the measurements the accept
    // decision turns on, and they separate a true pose from a wrong-roll alias far better
    // than the match count does (measured on a guiding session: bright projected 5-9 of 12
    // for correct poses against 0-2 of 12 for wrong ones). Carried on the result so an
    // ACCEPTED pose can be judged from an operational log, not just a rejected one - the
    // rejection path already reports them inside its failure reason.
    int m_brightDetections = 0;
    int m_matchedBrightDetections = 0;
    int m_brightProjectedStars = 0;
    int m_matchedBrightProjectedStars = 0;
    double m_brightDetectionMagnitudeError = std::numeric_limits<double>::infinity();
    double m_brightnessRankError = std::numeric_limits<double>::infinity();
};

class QNetworkAccessManager;
class QNetworkReply;

/**
 * \brief Solves telescope pointing (az/el/roll/FOV) from detected stars against a star catalog.
 *
 * Given a set of detected stars, the image size, a capture timestamp and the current CameraSettings
 * (observer location, lens model, search hints, magnitude limits, etc.), solve() matches the stars
 * to a star catalog and recovers the camera's pointing and field of view, returning a
 * CameraPlateSolveResult. Catalogs include locally imported data and the Siril SPCC service; Siril
 * range/index downloads are cached across solve() calls so repeated solves on the same sky area do
 * not re-download.
 *
 * \note A QObject so it can own a QNetworkAccessManager and run nested event loops for Siril
 *       requests, but solve() is CPU-bound and is normally invoked on the star-detector thread.
 * \note Cancellation is cooperative and thread-safe via atomics: requestCancellation() can be set
 *       from the feature thread while a solve runs, and requestNetworkCancellation() aborts a Siril
 *       HTTP request blocking inside a nested loop.exec() (safe to call during event processing).
 */
class CameraPlateSolver : public QObject
{
public:
    explicit CameraPlateSolver(QObject *parent = nullptr);
    ~CameraPlateSolver() override;

    static QString downloadedCatalogArchivePath();
    static QString downloadedCatalogCsvPath();
    static QString sirilAstroCatalogPath();
    static QString sirilAstroCompressedCatalogPath();
    static bool importDownloadedCatalogArchive(const QString& archivePath, QString* errorMessage = nullptr);

    // Bright-star catalog positions (user-configured file, downloaded catalog, or the
    // bundled fallback resource), for consumers outside the solver such as the cloud
    // detector's star-visibility sensing. Thread-safe; the underlying catalog is cached.
    struct BrightStar
    {
        double rightAscensionDegrees;
        double declinationDegrees;
        double magnitude;
    };
    static QVector<BrightStar> brightStarCatalog(const CameraSettings& settings);

    CameraPlateSolveResult solve(const CameraSettings& settings,
                                 const QSize& imageSize,
                                 const QDateTime& captureDateTime,
                                 QVector<CameraPipelineStarDetection>& starDetections);
    void requestCancellation();
    bool isCancellationRequested() const;
    // Cancel any Siril SPCC network request currently blocking in loop.exec() inside
    // fetchSirilRangeFromSource.  Safe to call from the star-detector thread during
    // event processing (e.g. from CameraStarDetector::captureActiveChanged(false)).
    void requestNetworkCancellation();

private:
    class SolverContext;
    friend class SolverContext;
    QNetworkAccessManager *m_networkManager = nullptr;
    // Siril SPCC catalog caches — persisted across solve() calls so repeated plate
    // solves on the same sky area don't re-download the same data each frame.
    QHash<QString, QByteArray> m_sirilRangeCache;
    QHash<int, QByteArray> m_sirilIndexCache;
    // The general cancellation flag can be set from the feature thread while the
    // solver is CPU-bound in the star-detector thread. Network cancellation is
    // also checked by the nested Siril request event loops.
    std::atomic_bool m_cancelRequested {false};
    std::atomic_bool m_cancelNetworkRequests {false};
    // R1 defensive bound: a hard wall-clock budget for one top-level solve(). The retry
    // ladders and the inner solve are individually bounded, but each spawns a full inner
    // solve, so a pathological input (or a disabled acceptance gate — WS3 pass-2 found the
    // rollAlias reject is one ablation away from a multi-minute solve) can chain enough of
    // them to run for minutes. isCancellationRequested() also reports true once this budget
    // is exceeded, so every existing cancellation checkpoint (inner solve, worker contexts,
    // outer ladders) doubles as a deadline check and the solve returns best-so-far. The
    // clock is started once per solve() on the solver thread before any workers spawn;
    // QElapsedTimer::elapsed() is a mutation-free monotonic read so concurrent worker reads
    // are safe. Budget 0 disables the bound. Overridable via
    // SDRANGEL_CAMERA_PLATE_SOLVER_BUDGET_MS (0 = off); default kDefaultSolveBudgetMs.
    QElapsedTimer m_solveWallClock;
    std::atomic<qint64> m_solveBudgetMs {0};
    // The active Siril network reply, set/cleared on the star-detector thread (where the solver
    // runs) but read by requestCancellation() which can be invoked synchronously from the feature
    // thread (Camera::applySettings -> CameraStarDetector::requestPlateSolveCancellation). Guard
    // every access with this mutex so the raw pointer is never read while being written/destroyed.
    QMutex m_activeNetworkReplyMutex;
    QNetworkReply *m_activeNetworkReply = nullptr;
};

#endif // INCLUDE_FEATURE_CAMERAPLATESOLVER_H_
