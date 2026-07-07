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

#include <QLoggingCategory>
#include <QTimeZone>

// Per-solve diagnostic dump (settings / observer location / catalog path). These are intentional
// GUI-vs-harness mismatch diagnostics (e.g. they pinned the m51-2 UTC bug), but for continuous
// solving they are noisy, so they are gated behind this category (off by default at QtWarningMsg).
// Enable with QT_LOGGING_RULES="camera.platesolver.info=true". qWarning/qCritical stay unconditional.
Q_LOGGING_CATEGORY(cameraPlateSolverLog, "camera.platesolver", QtWarningMsg)


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

    // Atomic write (write-temp + rename) so a concurrent solver process can't observe a
    // partially-written catalog or race another import of the same file.
    QSaveFile outputFile(downloadedCatalogCsvPath());
    if (!outputFile.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to write imported HYG catalog: %1").arg(downloadedCatalogCsvPath());
        }
        return false;
    }

    if (outputFile.write(uncompressedData) != uncompressedData.size())
    {
        outputFile.cancelWriting();
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to fully write imported HYG catalog: %1").arg(downloadedCatalogCsvPath());
        }
        return false;
    }

    if (!outputFile.commit())
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to commit imported HYG catalog: %1").arg(downloadedCatalogCsvPath());
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
    // Which catalog file this solve actually loads. downloadedCatalogDir() resolves via
    // QStandardPaths AppDataLocation, which differs by application name -- so the GUI
    // (sdrangel) and the test harness can pick different files (downloaded HYG vs bundled
    // resource), giving different star positions from otherwise-identical inputs.
    qCInfo(cameraPlateSolverLog).nospace() << "CameraPlateSolver: solve catalogPath=" << currentCatalogPath(settings);
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
    m_elevationSeedScaleDegrees = std::max(2.0, static_cast<double>(settings.m_plateSolveAzElSearchRadius) * 0.35);
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
                static_cast<double>(settings.m_plateSolveAzElSearchRadius) * 0.35,
                static_cast<double>(settings.m_fov) * 2.0))
        : std::max(2.0, static_cast<double>(settings.m_plateSolveAzElSearchRadius) * 0.35);
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
        // Reinterpret the entered wall-clock in the chosen zone (keep hh:mm:ss, change the spec).
#if QT_VERSION >= QT_VERSION_CHECK(6, 5, 0)
        configuredSolveDateTime.setTimeZone(QTimeZone(settings.m_plateSolveDateTimeUtc ? QTimeZone::UTC : QTimeZone::LocalTime));
#else
        configuredSolveDateTime.setTimeSpec(settings.m_plateSolveDateTimeUtc ? Qt::UTC : Qt::LocalTime);  // deprecated in Qt 6.9
#endif
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
    // For FoV-seeded wide-field solves the bright-first-pass catalog (mag 5) is too sparse:
    // the true pose may match fewer bright stars than a false-positive candidate, so the
    // full-magnitude catalog is preloaded (but kept invisible until the guided-anchor rebuild)
    // to avoid re-reading the catalog file from disk later.
    const bool useWideFovFullCatalogLoad = !useCurrentSettingsOnly
        && !useStartDirection
        && isWidePlateSolveContext(settings)
        && (solveMaxMagnitude < static_cast<double>(settings.m_plateSolveMaxMagnitude));
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
        useBrightFirstPassCatalog ? fullSearchMaxMagnitude
        : useWideFovFullCatalogLoad ? static_cast<double>(settings.m_plateSolveMaxMagnitude)
        : solveMaxMagnitude);
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
            false,   // tight-artifact coincidence is applied label-only post-acceptance, not during scoring
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
        else if (useWideWeakAnchorSearch
                 && (solveMaxMagnitude < static_cast<double>(settings.m_plateSolveMaxMagnitude)))
        {
            // Wide FoV blind search used a bright (mag 5) catalog; for the guided-anchor
            // pass rebuild to the full magnitude so the true pose is not outscored by
            // coincidental bright-star matches at a false-positive location.
            guidedAnchorCatalogContext = catalogContext;
            const qint64 rebuildStartMs = solveProfileTimer.elapsed();
            rebuildVisibleCatalogContext(
                guidedAnchorCatalogContext,
                settings,
                captureDateTimeUtc,
                settings.m_plateSolveMaxMagnitude);
            logSolveProfile("catalog.rebuild.wideFovGuidedAnchor", rebuildStartMs);
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

        // Seed-anchored direct refine. With a trusted pointing (a direction start mode) the
        // true pose lies near the entered Az/El, so descend to it deterministically instead of
        // relying on the chaotic far-ranging search: evaluate the seed pose, then iterate
        // (LM refine -> re-match) to convergence and add the result to the candidate pool. A
        // direct descent from a good seed is insensitive to the ~1e-4 px cross-build numerical
        // noise that otherwise perturbs the broad search and flips marginal images
        // (stars-wide-7 GUI-vs-harness divergence). Only reached when the main search best was
        // not already early-accepted -- i.e. exactly the cases that need it -- and it only ever
        // ADDS a candidate, so the final acceptance still picks the best-scoring solution.
        // Also runs for an elevation-only start (mode 2): there the entered azimuth is used as the
        // lens-recovery calibration pose, but azimuth then refines freely (see lockSeedAz below),
        // so the off-centre lens is recovered and the best azimuth can win on quality instead of
        // the search settling on a degenerate near-zenith azimuth/roll alias with no lens.
        if (useStartDirection || plateSolveStartUsesElevation(settings))
        {
            const bool seedUsesLens = plateSolveStartUsesLens(settings);
            Evaluation seedAnchored = evaluatePose(
                settings, catalogContext, imageSize, captureDateTimeUtc, starDetections,
                detectionIndices,
                settings.m_azimuth, settings.m_elevation, settings.m_roll, settings.m_fov,
                nullptr,
                seedUsesLens ? settings.m_lensCenterOffsetX : 0.0,
                seedUsesLens ? settings.m_lensCenterOffsetY : 0.0,
                seedUsesLens ? settings.m_lensDistortionK1 : 0.0);
            if (seedAnchored.valid && (seedAnchored.matchCount >= settings.m_plateSolveMinMatches))
            {
                // Distortion + principal-point co-refinement. A wide fisheye solved in a direction
                // mode (no entered lens calibration) must recover both a large radial K1 and a
                // possibly off-centre principal point from scratch. A free-parameter LM from the
                // distortion-free seed mislocates them (mis-projected rim -> wrong matches), and a
                // single coarse (Cx,Cy,K1) grid leaves the candidate too loose, so the downstream
                // free-principal-point refine then diverges build-to-build (it reached cx=-41 in one
                // build, cx=+91 in another). Instead recover the lens by MATCH COUNT (LM-free, hence
                // ULP-robust and identical across builds): a coarse grid to find the basin, then
                // alternate a pose LM (principal point LOCKED) with a shrinking-step (Cx,Cy,K1) grid
                // around the current best. This converges to a tight solution deterministically, so
                // the downstream refine starts already-converged and cannot diverge.
                const auto isBetterSeedAnchored = [](const Evaluation& a, const Evaluation& b) {
                    if (!b.valid) { return a.valid; }
                    if (!a.valid) { return false; }
                    if (a.matchCount != b.matchCount) { return a.matchCount > b.matchCount; }
                    return a.rmsErrorPixels < b.rmsErrorPixels;
                };
                // For WIDE FISHEYE, pin azimuth+elevation to the seed during this refine (refine
                // ROLL/FoV/K1 + the gridded principal point only); narrow fields keep Az/El free.
                // This builds a strong hypothesis AT THE ENTERED DIRECTION with the off-centre lens
                // recovered. Near zenith Az and Roll are degenerate, so a free Az LM slides along
                // that valley to a near-tied alternate basin (same match count + RMS, wrong pose),
                // and which basin it lands in flips with ~1e-13 trig ULP between the independently-
                // compiled GUI DLL and test EXE (mode 3 wide-9: one build kept Az~51 -> 174 matches,
                // the other slid to Az~43 -> refine ran away). Pinning Az removes that choice; the
                // downstream refine-from-matches does the final sub-degree polish. For an elevation-
                // only start (mode 2) this candidate is ADDITIVE -- the broad azimuth search still
                // runs and keeps azimuth free; this just adds a lens-recovered hypothesis at the
                // entered azimuth so, when that azimuth is roughly right, the good rms~1 solution
                // beats the lens-less degenerate alias the broad search would otherwise settle on.
                // Narrow guided solves have no such degeneracy and DO need precise Az/El here
                // (locking regressed galaxy-m51-2), so the lock is wide-only.
                // WS2 payoff: the wide-fisheye Az/El pin is a band-aid for the near-zenith Az<->Roll
                // ULP basin flip. With the rotation-vector LM the orientation is well-conditioned at
                // zenith, so the pin is no longer needed there -- let the LM refine Az/El freely. The
                // pin stays in force on the legacy (rot-vec off) path.
                const bool lockSeedDirection = isWidePlateSolveContext(settings) && !rotVecLmActive(settings);
                const std::array<bool, PlateSolveLmParameterCount> seedActiveParameters = {{
                    !lockSeedDirection,          // PlateSolveLmAzimuth (wide: pinned to entered direction)
                    !lockSeedDirection,          // PlateSolveLmElevation (wide: pinned to entered direction)
                    true,                        // PlateSolveLmRoll
                    true,                        // PlateSolveLmFov
                    false,                       // PlateSolveLmCenterX (recovered by the grid)
                    false,                       // PlateSolveLmCenterY (recovered by the grid)
                    canCalibrateLens(settings)   // PlateSolveLmDistortionK1
                }};
                const auto evalLens = [&](const Evaluation& pose, double cx, double cy, double k1) {
                    return evaluatePose(
                        settings, catalogContext, imageSize, captureDateTimeUtc, starDetections,
                        detectionIndices,
                        pose.azimuthDegrees, pose.elevationDegrees, pose.rollDegrees, pose.fovDegrees,
                        nullptr, cx, cy, k1);
                };
                const auto refinePoseStep = [&](const Evaluation& start) {
                    const QVector<Match> fixedMatches = uniqueValidMatchesForRefinement(
                        catalogContext, starDetections, start.matches, nullptr);
                    if (fixedMatches.size() < settings.m_plateSolveMinMatches) {
                        return start;
                    }
                    const QVector<int> rankDetectionIndices = detectionIndicesForMatches(fixedMatches);
                    const Evaluation lm = runPlateSolveLmRefinement(
                        settings, catalogContext, imageSize, starDetections, fixedMatches,
                        rankDetectionIndices, start, seedActiveParameters, finalMatchRadius);
                    const Evaluation rematched = evalLens(
                        lm, lm.centerOffsetXPixels, lm.centerOffsetYPixels, lm.distortionK1);
                    const Evaluation iterate = isBetterSeedAnchored(rematched, lm) ? rematched : lm;
                    return isBetterSeedAnchored(iterate, start) ? iterate : start;
                };

                if (canCalibrateLens(settings))
                {
                    // Coarse pass: locate the principal-point / distortion basin at the seed pose.
                    // NB: load-bearing even under rot-vec/WS1a -- it finds the off-centre fisheye
                    // principal-point basin the free-pp LM cannot reach from zero (gating it off the
                    // rot-vec path regressed synth-fisheye-031). Kept; only the downstream pp clamp
                    // (a pure cross-build band-aid) was retired.
                    Evaluation bestSeed = seedAnchored;
                    static const double kCenterGrid[] = { -60.0, -30.0, 0.0, 30.0, 60.0 };
                    static const double kK1Grid[] = { -0.15, -0.10, -0.05, 0.0, 0.05 };
                    for (const double cx : kCenterGrid) {
                        // 125 full-catalog evaluations; check for cancellation each cx row so a
                        // cancel request during a slow wide-fisheye solve is honoured promptly.
                        if (isCancellationRequested()) { return finishCancelled(); }
                        for (const double cy : kCenterGrid) {
                            for (const double k1 : kK1Grid) {
                                const Evaluation swept = evalLens(seedAnchored, cx, cy, k1);
                                if (isBetterSeedAnchored(swept, bestSeed)) {
                                    bestSeed = swept;
                                }
                            }
                        }
                    }
                    seedAnchored = bestSeed;
                }

                // Refine the pose (az/el/roll/fov + K1) from the grid-selected (or seed) start
                // with the principal point LOCKED at the grid value; keep the best iterate. From
                // this good basin the LM converges without overfitting, and the downstream refine
                // polishes the principal point to its precise value.
                for (int pass = 0; pass < 5; ++pass)
                {
                    if (isCancellationRequested()) { return finishCancelled(); }
                    const int prevMatches = seedAnchored.matchCount;
                    const Evaluation refined = refinePoseStep(seedAnchored);
                    if (!isBetterSeedAnchored(refined, seedAnchored)) {
                        break;
                    }
                    seedAnchored = refined;
                    if (seedAnchored.matchCount <= prevMatches) {
                        break;
                    }
                }

                // Clamped free-principal-point polish. The match-count grid above finds the basin
                // robustly but only a COARSE principal point (no match-count gradient near the
                // optimum), which caps the match count; reaching the precise off-centre principal
                // point needs an RMS-minimising LM, which is the build-dependent step (it converged
                // to cx=-41 in one build but overfit to cx=+91 in another, leaving the GUI matching
                // far fewer stars). Free the principal point here but CLAMP it to a window around
                // the robust grid value after each step: the true principal point lies within the
                // window (it is at most a grid step from the chosen cell) so convergence is
                // unaffected, while a runaway overfit is clamped to a worse pose and rejected by the
                // keep-best rule -- making the precise refine converge identically across builds.
                if (canCalibratePrincipalPoint(settings))
                {
                    const double anchorCx = seedAnchored.centerOffsetXPixels;
                    const double anchorCy = seedAnchored.centerOffsetYPixels;
                    constexpr double kCenterWindowPixels = 30.0;
                    // Az/El follow the same wide-only lock as the pose LM above (lockSeedDirection);
                    // Roll/FoV/principal-point/K1 are freed, with the principal point clamped to the
                    // grid window below.
                    const std::array<bool, PlateSolveLmParameterCount> freeParameters = {{
                        !lockSeedDirection, !lockSeedDirection, true, true, true, true, canCalibrateLens(settings)
                    }};
                    for (int pass = 0; pass < 5; ++pass)
                    {
                        if (isCancellationRequested()) { return finishCancelled(); }
                        const int prevMatches = seedAnchored.matchCount;
                        const QVector<Match> fixedMatches = uniqueValidMatchesForRefinement(
                            catalogContext, starDetections, seedAnchored.matches, nullptr);
                        if (fixedMatches.size() < settings.m_plateSolveMinMatches) {
                            break;
                        }
                        const QVector<int> rankDetectionIndices = detectionIndicesForMatches(fixedMatches);
                        Evaluation lm = runPlateSolveLmRefinement(
                            settings, catalogContext, imageSize, starDetections, fixedMatches,
                            rankDetectionIndices, seedAnchored, freeParameters, finalMatchRadius);
                        // WS2 follow-on: the ±window principal-point clamp existed to make the
                        // RMS-minimising LM converge identically between the independently-compiled
                        // GUI DLL and test EXE (cx=-41 vs cx=+91 across builds). WS1a makes both link
                        // the same solver object, so there is no longer a cross-build choice to pin,
                        // and the keep-best rule below already rejects a genuine overfit (fewer stars).
                        // Drop the clamp on the modern (rot-vec) path; the legacy path keeps it.
                        if (!rotVecLmActive(settings))
                        {
                            lm.centerOffsetXPixels = std::clamp(
                                lm.centerOffsetXPixels, anchorCx - kCenterWindowPixels, anchorCx + kCenterWindowPixels);
                            lm.centerOffsetYPixels = std::clamp(
                                lm.centerOffsetYPixels, anchorCy - kCenterWindowPixels, anchorCy + kCenterWindowPixels);
                        }
                        const Evaluation rematched = evalLens(
                            lm, lm.centerOffsetXPixels, lm.centerOffsetYPixels, lm.distortionK1);
                        if (!isBetterSeedAnchored(rematched, seedAnchored)) {
                            break;
                        }
                        seedAnchored = rematched;
                        if (seedAnchored.matchCount <= prevMatches) {
                            break;
                        }
                    }
                }

                logPlateSolveEvaluation("seed-anchored-refine", seedAnchored, false, true);
                insertDistinctEvaluationCandidate(
                    coarseCandidates,
                    seedAnchored,
                    multiHypothesisCandidateLimit,
                    useWeakModeScoring,
                    "seed-anchored-refine",
                    std::max(3, settings.m_plateSolveMinMatches - 1),
                    weakModeCandidatePoolMinMatches,
                    useStartDirection);
            }
        }
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
    // FoV-pinned re-check trigger: fire only when the refined FoV has drifted beyond what
    // the user's FoV confidence allows. m_plateSolveFovTolerance is a percentage of the FoV
    // (0 = exact, so any drift re-checks/pins to the entered FoV; larger lets the refined
    // FoV stand within that band). This is the user-facing "how well do you know your FoV"
    // knob: a calibrated lens (telescope or characterised fisheye) sets it low and the
    // entered FoV is honoured; a first, un-calibrated wide/fisheye image sets it high and
    // the solver is free to refine the FoV (e.g. stars-wide-5 refines 163.5->159.8 rather
    // than being force-pinned to a wrong seed). Narrow fields keep the legacy 2 deg trigger
    // and rely on useNarrowKnownFovRecovery for their precise FoV recovery.
    // The tolerance applies to GUIDED wide solves (mode 3+: the user has deliberately set
    // up the camera, so the entered FoV is a calibrated-but-possibly-approximate value).
    // Narrow fields and non-direction modes (blind / FoV-only) keep the tight legacy 2 deg
    // pinned re-check, which robustly rescues a wrong FoV refinement when the entered FoV is
    // exact (as the synthetic corpora's seeds are).
    const double fovPinTriggerDegrees = (isNarrowField(settings) || !useStartDirection)
        ? 2.0
        : std::max(0.0, static_cast<double>(settings.m_fov)
            * static_cast<double>(settings.m_plateSolveFovTolerance) / 100.0);
    if (selectedFinalPass.projectorValid
        && rankFinalPassWithSelectedDetections
        && useStartFov
        && ((std::fabs(best.fovDegrees - settings.m_fov) >= fovPinTriggerDegrees)
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
        // For FoV-seeded wide-field solves the detectionIndices subset was chosen for the
        // blind-search 'best' direction, so isBetterWeakModeFinalMatchPass is biased:
        // brightProjectedMatchFraction is low for true-pose candidates in other sky regions
        // because their bright catalog stars are not in the 96-star subset. Also the
        // fisheye k1 is unknown (useStartLens=false), making brightProjectedMatchFraction
        // unreliable regardless.
        //
        // Strategy:
        //   1. Re-evaluate the current regular best with allDetectionIndices to get an
        //      unbiased bright-anchor result.
        //   2. Evaluate every viable coarseCandidate with allDetectionIndices.
        //   3. Among accepted wide-FoV candidates, pick by magnitudeWeightedSupport.
        //   4. Override the regular best only when it fails the acceptance gate (re-evaluated
        //      with allDetections) and a wide-FoV candidate passes — so passing cases are
        //      not disturbed.
        // The rescue block is relevant whenever the guided-anchor coarseCandidates pool was
        // built from the full-magnitude catalog — either via a rebuild (usingFullCatalogForGuidedAnchor)
        // or because the original catalog was already at full magnitude (solveMaxMagnitude already
        // equals the requested maxMagnitude, so no rebuild was triggered but the pool is full).
        const bool guidedAnchorUsedFullMagnitudeCatalog =
            usingFullCatalogForGuidedAnchor
            || (solveMaxMagnitude >= static_cast<double>(settings.m_plateSolveMaxMagnitude));
        if (useWideWeakAnchorSearch && guidedAnchorUsedFullMagnitudeCatalog) {
            // Compute whether the fovPinnedFinalPass result already passes the acceptance gate.
            // Used to log the rescue decision; the rescue still runs regardless to find the
            // best wide-FoV candidate, but replacement only happens when it strictly beats
            // bestFovPinnedFinalPass by magnitudeWeightedSupport.
            // Compute BrightMagErr for the fovPinnedFinalPass best (used to detect
            // false-positive fovPinnedFinalPass results: genuine accepted poses from the
            // fovPinnedFinalPass 3×3×3 grid have BrightMagErr well below the 1.50 gate
            // because the refined grid finds the true pose; false positives that coincidentally
            // pass the gate tend to have BrightMagErr > 0.70).
            const double fovPinnedBrightMagErr =
                bestFovPinnedFinalPass.projectorValid
                ? bestFovPinnedFinalPass.brightDetectionMagnitudeError
                : std::numeric_limits<double>::max();
            // The regular fovPinnedFinalPass result is considered "strongly accepted" when it
            // also has a low BrightMagErr — this tighter gate prevents false positives that
            // happen to pass hasAcceptableWideBrightAnchorSupport from blocking the rescue.
            // 0.80 sits in the gap between genuine accepted poses (case 044: BrightMagErr=0.718)
            // and confirmed false positives (case 034: BrightMagErr=0.94).
            static constexpr double kRegularBestStrongAcceptanceMagErrThreshold = 0.80;
            const bool regularBestAccepted =
                bestFovPinnedFinalPass.projectorValid
                && (fovPinnedBrightMagErr <= kRegularBestStrongAcceptanceMagErrThreshold)
                && hasAcceptableWideBrightAnchorSupport(
                    settings, starDetections, bestFovPinnedFinalPass);

            const int wideCandMinMatches = std::max(3, settings.m_plateSolveMinMatches - 1);
            QVector<Evaluation> wideFovCandidatePoses;
            for (const Evaluation& cand : coarseCandidates) {
                if (!cand.valid || (cand.matchCount < wideCandMinMatches)) {
                    continue;
                }
                for (double kSeed : distortionSeeds) {
                    Evaluation pinned = cand;
                    pinned.fovDegrees = settings.m_fov;
                    pinned.distortionK1 = kSeed;
                    // Clear the anchor: guided-anchor candidates store anchorCatalogIndex
                    // relative to guidedAnchorCatalogContext; evaluating with catalogContext
                    // fails the forced-anchor lookup and returns an empty result.
                    pinned.anchored = false;
                    wideFovCandidatePoses.append(pinned);
                }
            }
            const QVector<FinalMatchPassEvaluation> wideFovCandidateResults =
                evaluateRecoveryPosesParallel(
                    static_cast<int>(wideFovCandidatePoses.size()),
                    [&](SolverContext& workerContext, int i) {
                        return workerContext.evaluateFinalMatchPass(
                            settings,
                            catalogContext,
                            imageSize,
                            starDetections,
                            allDetectionIndices,
                            wideFovCandidatePoses[i],
                            finalMatchRadius,
                            true);
                    });

            // When the fovPinnedFinalPass best fails strong acceptance, pick the coarse
            // candidate with the highest magnitudeWeightedSupport that passes the acceptance
            // gate. Near-true-direction and correct-pose candidates consistently score higher
            // on this metric than false positives in unrelated sky regions.
            Evaluation bestWideCandEval;
            FinalMatchPassEvaluation bestWideCandPass;
            for (int i = 0; i < wideFovCandidatePoses.size(); ++i) {
                const FinalMatchPassEvaluation& fp = wideFovCandidateResults[i];
                logFinalMatchPassEvaluation("final-match-pass-fov-pinned-wide", fp);
                if (!fp.projectorValid
                    || !hasAcceptableWideBrightAnchorSupport(settings, starDetections, fp)) {
                    continue;
                }
                if (!regularBestAccepted) {
                    if (!bestWideCandPass.projectorValid
                        || (fp.magnitudeWeightedSupport > bestWideCandPass.magnitudeWeightedSupport)) {
                        bestWideCandEval = wideFovCandidatePoses[i];
                        bestWideCandPass = fp;
                    }
                }
            }
            if (bestWideCandPass.projectorValid
                && (!bestFovPinnedFinalPass.projectorValid
                    || bestWideCandPass.magnitudeWeightedSupport
                       > bestFovPinnedFinalPass.magnitudeWeightedSupport)) {
                bestFovPinnedEvaluation = bestWideCandEval;
                bestFovPinnedFinalPass = bestWideCandPass;
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
    stageStartMs = solveProfileTimer.elapsed();

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
    logSolveProfile("tightenFinalPass", stageStartMs);
    stageStartMs = solveProfileTimer.elapsed();

    const QVector<ProjectedCatalogStar>& projectedStars = selectedFinalPass.projectedStars;
    repairFinalMatchCollisions(starDetections, projectedStars, finalMatchRadius, selectedFinalPass.finalMatches);
    const QVector<Match>& finalMatches = selectedFinalPass.finalMatches;
    debugDumpUnmatchedDetections(catalogContext, starDetections, projectedStars, finalMatches, finalMatchRadius);
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
    // SHADOW MODE (Track 1a): the astrometry.net per-detection mixture verifier, recorded alongside
    // faLogOdds so the two can be compared against the heuristic accept/reject bands across the
    // corpus. Also not gating -- the success criterion is cleaner accept/reject separation than the
    // per-match sum before it is wired into any decision.
    recordProfileMetric(QStringLiteral("verify.mixtureLogOddsMilli"),
        static_cast<qint64>(std::llround(
            poseVerificationLogOdds(catalogContext, selectedFinalPass, imageSize, finalMatchRadius,
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
    const qint64 rollAliasStartMs = solveProfileTimer.elapsed();
    const bool rollAliasDetected = hasCompetitiveRollAlias(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            allDetectionIndices,
            selectedFinalPass,
            finalMatchRadius,
            &rollAmbiguityReason,
            &rollAdoptedAlias);
    // WS3 ablation: gateAblationDisabled("rollAlias") suppresses only the ambiguous-roll REJECT.
    // The function is still called, so the adopt-better-alias path (rollAdoptedAlias, below) is
    // unaffected by the toggle.
    const bool competitiveRollAlias = rollAliasDetected && !gateAblationDisabled("rollAlias");
    logSolveProfile("rollAliasCheck", rollAliasStartMs);
    if (competitiveRollAlias)
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

    // Label-only recovery (wide fisheye): admit hot-pixel-suspect detections that sit in tight
    // coincidence with a bright catalogue star at the ACCEPTED pose -- a real bright star the
    // detector mis-flagged as a hot pixel in this heavily-undersampled field (e.g. Dziban in
    // stars-wide-9). This runs AFTER acceptance and only on the winning pose, so it can add
    // labels but cannot change the solve verdict, candidate selection, or acceptance (the scoring
    // passes all use allowTightArtifactCoincidence=false).
    appendSupplementalMatches(
        starDetections,
        projectedStars,
        finalMatchRadius,
        nullptr,
        isNarrowGuidedDirectionSolve(settings),
        isWidePlateSolveContext(settings),
        selectedFinalPass.finalMatches);

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
    // Factored into a lambda so the bright-anchor verifier rescue below can re-run the
    // exact same, unchanged acceptance decision against a rescued candidate pose.
    struct DirectionSeedAcceptance
    {
        bool acceptable = false;
        bool weakBrightSupport = false;
        bool fovAccepted = false;
    };
    auto directionSeedAcceptanceFor = [&](const FinalMatchPassEvaluation& pass) -> DirectionSeedAcceptance
    {
        DirectionSeedAcceptance acceptance;
        // WS3 (2026-06-19): the overwhelmingFaint / sparsePair / highConfSparseAnchors accept-bypasses
        // were removed from this decision after ablation proved them jointly inert across REAL + RAND2
        // + FISHEYE-mode4 + near-boundary/garbage negatives (zero pass-drops, zero false positives;
        // see doc/camera/plate-solver-notes.md "WS3"). The remaining gates keep their ablation hooks
        // (SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_GATE) for future measurement; all are inert when unset.
        acceptance.weakBrightSupport = useStartDirection
            && !gateAblationDisabled("weakBrightSupport")
            && hasWeakNarrowGuidedBrightSupport(settings, pass);
        acceptance.fovAccepted = gateAblationDisabled("fov")
            || isAcceptableNarrowGuidedFov(settings, pass.pose.fovDegrees);
        acceptance.acceptable = !useStartDirection
            || (!acceptance.weakBrightSupport
                && acceptance.fovAccepted
                && (gateAblationDisabled("residual")
                    || isAcceptableDirectionSeedSolve(
                        settings,
                        starDetections,
                        pass.finalMatches,
                        pass.rmsErrorPixels,
                        pass.maxErrorPixels))
                && (gateAblationDisabled("brightnessConsistency")
                    || hasAcceptableGuidedFinalBrightnessConsistency(settings, pass)));
        return acceptance;
    };

    DirectionSeedAcceptance directionSeedAcceptance = directionSeedAcceptanceFor(selectedFinalPassForAcceptance);
    logSolveProfile("acceptance", stageStartMs);
    stageStartMs = solveProfileTimer.elapsed();

    if (qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE")
        && useStartDirection
        && isNarrowField(settings))
    {
        qDebug() << "CameraPlateSolver: pre-polish acceptance"
                 << "acceptable=" << directionSeedAcceptance.acceptable
                 << "weakBrightSupport=" << directionSeedAcceptance.weakBrightSupport
                 << "fovAccepted=" << directionSeedAcceptance.fovAccepted
                 << "matches=" << selectedFinalPassForAcceptance.finalMatches.size()
                 << "projectorValid=" << selectedFinalPassForAcceptance.projectorValid
                 << "Az=" << selectedFinalPassForAcceptance.pose.azimuthDegrees
                 << "El=" << selectedFinalPassForAcceptance.pose.elevationDegrees
                 << "Roll=" << selectedFinalPassForAcceptance.pose.rollDegrees;
    }

    // Dense-match FoV/pose polish (failure path only): a richly-matched (>= minMatches+50)
    // narrow direction-seeded candidate can converge to a pose whose RMS is dominated by a
    // small, roughly-uniform FoV mismatch (FoV is deliberately left fixed for these solves -
    // see refinePoseFromMatches's activeParameters), leaving many genuinely-aligned bright
    // catalog stars a few px outside finalMatchRadius of their detections and so unmatched
    // (hasWeakNarrowGuidedBrightSupport's matchedBrightProjectedStars/brightProjectedStars
    // stays low even though the basin is correct). Re-running LM with FoV free, seeded from
    // the candidate's own large match set (well-constrained, unlike the bright-anchor
    // rescue's 1-2 anchor matches), can correct this. Adopted only if it independently
    // clears the *same*, unchanged acceptance gate and retains >=90% of the original
    // matches (mirroring tightenNarrowFinalPass's safety margin) - so this can only ever
    // surface a pose the existing gate agrees is acceptable.
    if (!directionSeedAcceptance.acceptable
        && directionSeedAcceptance.weakBrightSupport
        && useStartDirection
        && isNarrowField(settings)
        && selectedFinalPassForAcceptance.projectorValid
        && (selectedFinalPassForAcceptance.finalMatches.size() >= settings.m_plateSolveMinMatches + 50))
    {
        Evaluation polishSeed = selectedFinalPassForAcceptance.pose;
        polishSeed.valid = true;
        polishSeed.matches = selectedFinalPassForAcceptance.finalMatches;
        polishSeed.matchCount = static_cast<int>(selectedFinalPassForAcceptance.finalMatches.size());
        const Evaluation polished = refinePoseFromMatches(
            settings, catalogContext, imageSize, captureDateTimeUtc, starDetections, polishSeed,
            /*forceFovRefine=*/true);
        const bool debugSparse = qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE");
        if (polished.valid)
        {
            const FinalMatchPassEvaluation polishedPass = evaluateFinalMatchPass(
                settings, catalogContext, imageSize, starDetections, allDetectionIndices, polished, finalMatchRadius);
            const qsizetype minRetainedMatches = static_cast<qsizetype>(
                std::floor(static_cast<double>(selectedFinalPassForAcceptance.finalMatches.size()) * 0.9));
            if (debugSparse)
            {
                qDebug() << "CameraPlateSolver: dense-match FoV/pose polish candidate"
                         << "matches=" << polishedPass.finalMatches.size()
                         << "minRetained=" << minRetainedMatches
                         << "rms=" << polishedPass.rmsErrorPixels
                         << "Az=" << polishedPass.pose.azimuthDegrees
                         << "El=" << polishedPass.pose.elevationDegrees
                         << "Roll=" << polishedPass.pose.rollDegrees
                         << "FoV=" << polishedPass.pose.fovDegrees
                         << "projectorValid=" << polishedPass.projectorValid
                         << "geomConsistent=" << hasGeometricallyConsistentMatches(starDetections, polishedPass.projectedStars, polishedPass.finalMatches, finalMatchRadius);
            }
            if (polishedPass.projectorValid
                && (polishedPass.finalMatches.size() >= minRetainedMatches)
                && hasGeometricallyConsistentMatches(starDetections, polishedPass.projectedStars, polishedPass.finalMatches, finalMatchRadius))
            {
                const DirectionSeedAcceptance polishedAcceptance = directionSeedAcceptanceFor(polishedPass);
                if (debugSparse)
                {
                    qDebug() << "CameraPlateSolver: dense-match FoV/pose polish acceptance"
                             << "acceptable=" << polishedAcceptance.acceptable
                             << "weakBrightSupport=" << polishedAcceptance.weakBrightSupport
                             << "fovAccepted=" << polishedAcceptance.fovAccepted;
                }
                if (polishedAcceptance.acceptable)
                {
                    qDebug() << "CameraPlateSolver: dense-match FoV/pose polish adopted pose"
                             << "matches=" << polishedPass.finalMatches.size()
                             << "rms=" << polishedPass.rmsErrorPixels
                             << "Az=" << polishedPass.pose.azimuthDegrees
                             << "El=" << polishedPass.pose.elevationDegrees
                             << "Roll=" << polishedPass.pose.rollDegrees
                             << "FoV=" << polishedPass.pose.fovDegrees;
                    selectedFinalPass = polishedPass;
                    selectedFinalPassForAcceptance = polishedPass;
                    directionSeedAcceptance = polishedAcceptance;
                    result.m_catalogCandidateStars = selectedFinalPass.projectedStars.size();
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
                    result.m_azimuthDegrees = selectedFinalPass.pose.azimuthDegrees;
                    result.m_elevationDegrees = selectedFinalPass.pose.elevationDegrees;
                    result.m_rollDegrees = selectedFinalPass.pose.rollDegrees;
                    result.m_fovDegrees = selectedFinalPass.pose.fovDegrees;
                    result.m_centerOffsetXPixels = selectedFinalPass.pose.centerOffsetXPixels;
                    result.m_centerOffsetYPixels = selectedFinalPass.pose.centerOffsetYPixels;
                    result.m_distortionK1 = selectedFinalPass.pose.distortionK1;

                    // finalMatches/projectedStars alias selectedFinalPass's members (now
                    // polishedPass), but the labelling loop and result match counters above
                    // (lines ~21184-21202) already ran against the pre-polish pass - redo
                    // them here so the per-detection labels and summary reflect the
                    // adopted polished pose.
                    clearSolvedStars(starDetections);
                    QHash<int, QPointF> polishedProjectedPointsByCatalogIndex;
                    for (const ProjectedCatalogStar& projectedStar : projectedStars) {
                        polishedProjectedPointsByCatalogIndex.insert(projectedStar.catalogIndex, projectedStar.point);
                    }
                    for (const Match& match : finalMatches)
                    {
                        CameraPipelineStarDetection& detection = starDetections[match.detectionIndex];
                        const CatalogStar& catalogStar = catalogContext.catalogStars[match.catalogIndex];
                        detection.m_label = catalogDisplayName(catalogStar);
                        detection.m_projectedCenter = polishedProjectedPointsByCatalogIndex.value(match.catalogIndex);
                        detection.m_matchDistancePixels = static_cast<float>(match.distancePixels);
                        detection.m_catalogMagnitude = static_cast<float>(catalogStar.magnitude);
                        detection.m_catalogRightAscensionDegrees = catalogStar.rightAscensionDegrees;
                        detection.m_catalogDeclinationDegrees = catalogStar.declinationDegrees;
                        detection.m_catalogSpectralType = catalogStar.spectralType;
                        detection.m_solved = true;
                    }
                    result.m_matchedStars = finalMatches.size();
                    result.m_rmsErrorPixels = selectedFinalPass.rmsErrorPixels;
                    result.m_maxErrorPixels = selectedFinalPass.maxErrorPixels;
                    result.m_matchSummary = matchSummary(catalogContext, starDetections, finalMatches);
                }
            }
        }
        else if (debugSparse)
        {
            qDebug() << "CameraPlateSolver: dense-match FoV/pose polish refine invalid";
        }
    }
    logSolveProfile("densePolish", stageStartMs);
    stageStartMs = solveProfileTimer.elapsed();

    // Bright-detection-anchored verifier rescue (failure path only): the seed-verification
    // gate (>= 3 mutually-consistent matches) can reject every geometric seed in a sparse
    // or contaminated narrow field even when the true bright-aligned pose is reachable, so
    // it is generated but never survives to here (e.g. narrow-7's named anchor IS detected
    // at its catalog position, yet triangleVerifiedSeeds=quadVerifiedSeeds=
    // guidedAnchorTriangleVerifiedSeeds=0). searchBrightAnchorVerifierRescue builds
    // candidates directly from (bright detection, bright catalog star, roll) triples - which
    // bypasses that gate - and ranks them by poseFalseAlarmLogOdds. The result is adopted
    // only if it independently clears this *same*, unchanged acceptance gate, so this can
    // only ever surface a pose the existing, already-tuned gate agrees is acceptable - it
    // can rescue a missed true alignment, but can never relax what counts as one.
    if (!directionSeedAcceptance.acceptable
        && useStartDirection
        && isNarrowField(settings)
        && !isCancellationRequested())
    {
        const FinalMatchPassEvaluation rescuePass = searchBrightAnchorVerifierRescue(
            settings,
            catalogContext,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            allDetectionIndices,
            finalMatchRadius);
        if (rescuePass.projectorValid)
        {
            const DirectionSeedAcceptance rescueAcceptance = directionSeedAcceptanceFor(rescuePass);
            if (rescueAcceptance.acceptable
                && hasGeometricallyConsistentMatches(starDetections, rescuePass.projectedStars, rescuePass.finalMatches, finalMatchRadius))
            {
                qDebug() << "CameraPlateSolver: bright-anchor verifier rescue adopted pose"
                         << "matches=" << rescuePass.finalMatches.size()
                         << "rms=" << rescuePass.rmsErrorPixels
                         << "Az=" << rescuePass.pose.azimuthDegrees
                         << "El=" << rescuePass.pose.elevationDegrees
                         << "Roll=" << rescuePass.pose.rollDegrees
                         << "FoV=" << rescuePass.pose.fovDegrees
                         << "faLogOdds=" << poseFalseAlarmLogOdds(catalogContext, rescuePass, imageSize, finalMatchRadius, static_cast<int>(starDetections.size()));
                selectedFinalPass = rescuePass;
                selectedFinalPassForAcceptance = rescuePass;
                directionSeedAcceptance = rescueAcceptance;
                result.m_catalogCandidateStars = selectedFinalPass.projectedStars.size();
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
                result.m_azimuthDegrees = selectedFinalPass.pose.azimuthDegrees;
                result.m_elevationDegrees = selectedFinalPass.pose.elevationDegrees;
                result.m_rollDegrees = selectedFinalPass.pose.rollDegrees;
                result.m_fovDegrees = selectedFinalPass.pose.fovDegrees;
                result.m_centerOffsetXPixels = selectedFinalPass.pose.centerOffsetXPixels;
                result.m_centerOffsetYPixels = selectedFinalPass.pose.centerOffsetYPixels;
                result.m_distortionK1 = selectedFinalPass.pose.distortionK1;

                // finalMatches/projectedStars alias selectedFinalPass's members (now
                // rescuePass), but the per-detection labelling and the match counters
                // already ran against the pre-rescue pass — redo them so the result
                // payload reflects the adopted pose (without this, the run reports
                // solved=true with the *previous* selected pose's fields).
                clearSolvedStars(starDetections);
                QHash<int, QPointF> rescueProjectedPointsByCatalogIndex;
                for (const ProjectedCatalogStar& projectedStar : projectedStars) {
                    rescueProjectedPointsByCatalogIndex.insert(projectedStar.catalogIndex, projectedStar.point);
                }
                for (const Match& match : finalMatches)
                {
                    CameraPipelineStarDetection& detection = starDetections[match.detectionIndex];
                    const CatalogStar& catalogStar = catalogContext.catalogStars[match.catalogIndex];
                    detection.m_label = catalogDisplayName(catalogStar);
                    detection.m_projectedCenter = rescueProjectedPointsByCatalogIndex.value(match.catalogIndex);
                    detection.m_matchDistancePixels = static_cast<float>(match.distancePixels);
                    detection.m_catalogMagnitude = static_cast<float>(catalogStar.magnitude);
                    detection.m_catalogRightAscensionDegrees = catalogStar.rightAscensionDegrees;
                    detection.m_catalogDeclinationDegrees = catalogStar.declinationDegrees;
                    detection.m_catalogSpectralType = catalogStar.spectralType;
                    detection.m_solved = true;
                }
                result.m_matchedStars = finalMatches.size();
                result.m_rmsErrorPixels = selectedFinalPass.rmsErrorPixels;
                result.m_maxErrorPixels = selectedFinalPass.maxErrorPixels;
                result.m_matchSummary = matchSummary(catalogContext, starDetections, finalMatches);
            }
        }
    }
    logSolveProfile("brightAnchorRescue", stageStartMs);

    const bool weakNarrowGuidedBrightSupport = directionSeedAcceptance.weakBrightSupport;
    const bool narrowGuidedFovAccepted = directionSeedAcceptance.fovAccepted;
    const bool directionSeedSolveAcceptable = directionSeedAcceptance.acceptable;
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
        && !gateAblationDisabled("elevationSeed")
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
    // Abort is posted (QueuedConnection) so it runs on the reply's owning thread; the mutex
    // makes reading the pointer safe against the star-detector thread setting/clearing it.
    QMutexLocker activeReplyLocker(&m_activeNetworkReplyMutex);
    if (m_activeNetworkReply) {
        QMetaObject::invokeMethod(m_activeNetworkReply, &QNetworkReply::abort, Qt::QueuedConnection);
    }
}

// R1: default hard wall-clock budget for one top-level solve() (ms). The slowest legitimate
// solve in the corpus is the dense narrow-7 field at ~30s; this 120s ceiling is >4x that, so
// it never trips a real solve but caps a pathological or gate-ablated runaway (WS3 pass-2 found
// the rollAlias reject is one ablation away from a multi-minute solve). 0 disables the bound.
static constexpr qint64 kDefaultSolveBudgetMs = 120000;

static qint64 resolveSolveBudgetMs()
{
    const QByteArray budgetOverride = qgetenv("SDRANGEL_CAMERA_PLATE_SOLVER_BUDGET_MS");
    if (!budgetOverride.isEmpty()) {
        bool ok = false;
        const qint64 value = budgetOverride.toLongLong(&ok);
        if (ok && (value >= 0)) {
            return value;
        }
    }
    return kDefaultSolveBudgetMs;
}

bool CameraPlateSolver::isCancellationRequested() const
{
    if (m_cancelRequested.load()) {
        return true;
    }
    // R1: a top-level solve that blows its wall-clock budget is treated as cancelled, so every
    // existing checkpoint (inner solve, worker contexts, outer retry ladders) unwinds and the
    // solve returns best-so-far instead of running for minutes. The budget is far above the
    // slowest legitimate solve, so this never affects a real result.
    const qint64 budgetMs = m_solveBudgetMs.load();
    return (budgetMs > 0) && m_solveWallClock.isValid() && (m_solveWallClock.elapsed() > budgetMs);
}

// The configured Az/El search radius is an upper bound on how far the true pointing might
// be from the seed. For a narrow (telescope) FoV that bound is physically much smaller --
// you cannot be 12 deg off with a tracking mount -- and a too-wide search invites spurious
// matches (a flat 12 deg radius was breaking narrow galaxy solves). Cap the configured
// radius by a FoV-relative value so narrow fields auto-tighten while wide/fisheye fields
// keep the full configured radius: min(setting, max(3.5, fov*5)). This reproduces the
// per-FoV tuning the test harness used to apply locally, but now in the solver so the GUI
// and the harness behave identically from the single configured setting.
static double effectiveAzElSearchRadiusDegrees(const CameraSettings& settings)
{
    const double configured = std::max(0.0, static_cast<double>(settings.m_plateSolveAzElSearchRadius));
    const double fovCap = std::max(3.5, static_cast<double>(settings.m_fov) * 5.0);
    return std::min(configured, fovCap);
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
    // R1: arm the per-solve wall-clock deadline. Started here on the solver thread before any
    // worker threads spawn; isCancellationRequested() reports true once it is exceeded so the
    // solve is provably time-bounded regardless of input or acceptance-gate configuration.
    m_solveBudgetMs.store(resolveSolveBudgetMs());
    m_solveWallClock.start();

    // Dump every setting that drives the solve so a GUI run and a test-harness run
    // can be diffed line-for-line when they disagree on the same image. Split across
    // several statements: the MSVC/Qt toolchain silently truncates a single qInfo
    // with >= ~10 streamed/arg fields, so keep each line short.
    qCInfo(cameraPlateSolverLog).nospace() << "CameraPlateSolver: solve image=" << imageSize.width() << "x" << imageSize.height()
                      << " detections=" << starDetections.size()
                      << " captureDateTime=" << captureDateTime.toString(Qt::ISODate);
    qCInfo(cameraPlateSolverLog).nospace() << "CameraPlateSolver: solve seed Az=" << settings.m_azimuth
                      << " El=" << settings.m_elevation
                      << " Roll=" << settings.m_roll
                      << " FoV=" << settings.m_fov
                      << " startMode=" << static_cast<int>(settings.m_plateSolveStartMode);
    qCInfo(cameraPlateSolverLog).nospace() << "CameraPlateSolver: solve lens projection=" << static_cast<int>(settings.m_lensProjection)
                      << " Cx=" << settings.m_lensCenterOffsetX
                      << " Cy=" << settings.m_lensCenterOffsetY
                      << " K1=" << settings.m_lensDistortionK1;
    qCInfo(cameraPlateSolverLog).nospace() << "CameraPlateSolver: solve detect=" << settings.m_starDetect
                      << " threshold=" << settings.m_starThreshold
                      << " backgroundBlur=" << settings.m_starBackgroundBlur
                      << " minArea=" << settings.m_starMinArea
                      << " maxArea=" << settings.m_starMaxArea
                      << " maxAspectRatio=" << settings.m_starMaxAspectRatio;
    qCInfo(cameraPlateSolverLog).nospace() << "CameraPlateSolver: solve minMatches=" << settings.m_plateSolveMinMatches
                      << " matchRadius=" << settings.m_plateSolveMatchRadius
                      << " finalMatchRadius=" << settings.m_plateSolveFinalMatchRadius
                      << " maxMagnitude=" << settings.m_plateSolveMaxMagnitude
                      << " azElSearchRadius=" << settings.m_plateSolveAzElSearchRadius
                      << " fovTolerance=" << settings.m_plateSolveFovTolerance;
    qCInfo(cameraPlateSolverLog).nospace() << "CameraPlateSolver: solve applyMode=" << static_cast<int>(settings.m_plateSolveApplyMode)
                      << " catalogSource=" << static_cast<int>(settings.m_plateSolveCatalogSource)
                      << " useDownloadedCatalog=" << settings.m_plateSolveUseDownloadedCatalog
                      << " useCaptureDateTime=" << settings.m_plateSolveUseCaptureDateTime;
    // Observer location and the configured solve time. For an Az/El-seeded solve these
    // map the pointing to RA/Dec, so a location or time mismatch between GUI and harness
    // shifts where every catalog star projects -- the prime suspect when detections and
    // pose seed match but the fit is systematically off.
    qCInfo(cameraPlateSolverLog).nospace() << "CameraPlateSolver: solve latitude=" << settings.m_latitude
                      << " longitude=" << settings.m_longitude
                      << " plateSolveDateTime=" << settings.m_plateSolveDateTime.toString(Qt::ISODate)
                      << " dateTimeUtc=" << settings.m_plateSolveDateTimeUtc;
    // FoV-aware cap of the Az/El search radius, applied once to the base settings so every
    // run -- including retries that escalate from this base -- uses the tightened radius
    // for narrow fields. See effectiveAzElSearchRadiusDegrees().
    CameraSettings cappedSettings(settings);
    cappedSettings.m_plateSolveAzElSearchRadius = effectiveAzElSearchRadiusDegrees(settings);
    if (std::fabs(cappedSettings.m_plateSolveAzElSearchRadius
                  - static_cast<double>(settings.m_plateSolveAzElSearchRadius)) > 1e-3) {
        qCInfo(cameraPlateSolverLog).nospace() << "CameraPlateSolver: solve azElSearchRadius capped "
                          << settings.m_plateSolveAzElSearchRadius << " -> "
                          << cappedSettings.m_plateSolveAzElSearchRadius
                          << " (FoV " << settings.m_fov << ")";
    }

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
        double azimuthDegrees = 0.0;
        double elevationDegrees = 0.0;
        double rollDegrees = 0.0;
        double rmsErrorPixels = 0.0;
        double fovDegrees = 0.0;
        double centerOffsetXPixels = 0.0;
        double centerOffsetYPixels = 0.0;
        double distortionK1 = 0.0;
        QString summary;
    };
    QVector<SolveRunProfile> solveRunProfiles;
    QElapsedTimer totalSolveTimer;
    totalSolveTimer.start();

    // Handedness (mirror) adoption test for all-sky fisheye. An up-looking all-sky camera
    // images the sky REFLECTED, so the orientation-preserving projector cannot fit any pose and
    // the solver stalls on the least-bad wrong-roll pose. Solving the horizontally-mirrored
    // detection set can recover the true pose. This comparator decides when to keep the mirror.
    //
    // The available signal is match count + global rms (the named bright-anchor field is 0 on
    // real all-sky corpora, whose catalogue stars are not HIP/HR/HD-named). That signal does NOT
    // reliably separate a true mirror from a coincidental wrong-roll pose: on a dense all-sky
    // field a wrong pose can match as many faint stars at a similar (or lower) rms than the
    // correct pose. So this test is deliberately conservative and the whole retry is opt-in
    // (mirrorHandednessEnabled, default OFF) — see the block below. Thresholds are env-overridable
    // (SDRANGEL_CAMERA_PLATE_SOLVER_MIRROR_*) for calibration; the baked defaults are conservative.
    struct MirrorAdoptThresholds {
        int minMatches;         // sanity floor on the mirror's matched stars
        double maxRmsPixels;    // sanity cap on the mirror's global rms
        double rmsAbsMargin;    // mirror rms must be <= normal rms - this (vs a solved normal)
        double rmsRatio;        // and <= normal rms * this
        double minMatchRatio;   // and mirror matches >= normal matches * this (not a much sparser fit)
        double soloMaxRmsPixels;// when normal did NOT solve, adopt if mirror rms <= this
    };
    const auto readMirrorThreshold = [](const char* name, double fallback) -> double {
        const QByteArray value = qgetenv(name);
        if (!value.isEmpty()) {
            bool ok = false;
            const double parsed = value.toDouble(&ok);
            if (ok) { return parsed; }
        }
        return fallback;
    };
    // All-sky handedness (mirror) auto-detect. DEFAULT OFF: measurement showed that adopting a
    // mirror on match-statistics alone regresses correct NON-mirrored solves (synthetic fisheye
    // mode1 -2, mode4 -1) because a wrong mirror pose on a dense field can score a lower rms than
    // the correct normal pose — the same all-sky "verifier wall" that blocks wrong-pose rejection
    // elsewhere in this solver. There is no match-statistics signal that separates a true mirror
    // from a coincidental one, so enabling adoption by default would change synthetic verdicts
    // unexplained. The mechanism is kept behind an opt-in for a KNOWN-mirrored all-sky camera
    // (where every frame benefits and the false-adopt risk is absent). See the plan doc's
    // "AUTO-DETECT" section for the full evidence and the image-level-flip follow-on.
    const bool mirrorHandednessEnabled =
        qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_MIRROR");
    const MirrorAdoptThresholds mirrorThresholds {
        static_cast<int>(readMirrorThreshold("SDRANGEL_CAMERA_PLATE_SOLVER_MIRROR_MIN_MATCHES", 12)),
        readMirrorThreshold("SDRANGEL_CAMERA_PLATE_SOLVER_MIRROR_MAX_RMS", 12.0),
        readMirrorThreshold("SDRANGEL_CAMERA_PLATE_SOLVER_MIRROR_RMS_ABS_MARGIN", 2.5),
        readMirrorThreshold("SDRANGEL_CAMERA_PLATE_SOLVER_MIRROR_RMS_RATIO", 0.80),
        readMirrorThreshold("SDRANGEL_CAMERA_PLATE_SOLVER_MIRROR_MIN_MATCH_RATIO", 0.80),
        readMirrorThreshold("SDRANGEL_CAMERA_PLATE_SOLVER_MIRROR_SOLO_MAX_RMS", 9.0)
    };
    const auto mirrorResultClearlyBetter = [&mirrorThresholds](const CameraPlateSolveResult& mirror,
                                              const CameraPlateSolveResult& normal) -> bool {
        // The named bright-anchor discriminator is unavailable on real all-sky corpora (their
        // catalogue stars are not HIP/HR/HD-named, so namedBrightAnchorMatches is 0), so this
        // works from total matched count and global rms. That signal CANNOT reliably tell a true
        // mirror from a coincidental one (the documented all-sky verifier wall): a wrong pose can
        // match many faint stars at a low rms. Consequently adoption is NOT always safe — a wrong
        // mirror with a low rms can even displace a CORRECT normal solve whose rms is higher
        // (measured on a near-zenith truth-seeded TREx frame). That is why the whole retry is
        // opt-in (default OFF): with it off, every corpus is byte-identical. When on, the rule is
        // kept deliberately conservative so it fires only on a solid, clearly-better mirror.
        if (!mirror.m_solved
            || (mirror.m_matchedStars < mirrorThresholds.minMatches)
            || !(mirror.m_rmsErrorPixels > 0.0)
            || (mirror.m_rmsErrorPixels > mirrorThresholds.maxRmsPixels))
        {
            return false;
        }
        if (normal.m_solved)
        {
            const bool lowerRms =
                (mirror.m_rmsErrorPixels <= (normal.m_rmsErrorPixels - mirrorThresholds.rmsAbsMargin))
                && (mirror.m_rmsErrorPixels <= (mirrorThresholds.rmsRatio * normal.m_rmsErrorPixels));
            const bool notMuchSparser =
                mirror.m_matchedStars
                    >= static_cast<int>(std::floor(mirrorThresholds.minMatchRatio * normal.m_matchedStars));
            return lowerRms && notMuchSparser;
        }
        // Normal did not solve: adopt a solid, tight mirror lock.
        return mirror.m_rmsErrorPixels <= mirrorThresholds.soloMaxRmsPixels;
    };
    // Clear a detection's per-solve match annotations (keeping its intrinsic geometry/quality)
    // so a re-solve on the same vector starts from a clean slate. Used before the mirror solve
    // so the adopted-mirror output carries only the mirror pose's matches.
    const auto clearDetectionSolveState = [](CameraPipelineStarDetection& detection) {
        detection.m_projectedCenter = QPointF();
        detection.m_label.clear();
        detection.m_matchDistancePixels = 0.0f;
        detection.m_catalogMagnitude = 0.0f;
        detection.m_catalogRightAscensionDegrees = std::numeric_limits<double>::quiet_NaN();
        detection.m_catalogDeclinationDegrees = std::numeric_limits<double>::quiet_NaN();
        detection.m_catalogSpectralType.clear();
        detection.m_solved = false;
    };

    auto runSolve = [&](const CameraSettings& runSettings, const QString& reason, bool disableRollRecovery = false) {
        QElapsedTimer runTimer;
        runTimer.start();

        // One solve on the current detection vector, swapping the persistent Siril caches in
        // and out so SPCC data fetched here is reused by later solves rather than discarded.
        const auto solveOnce = [&](QVector<CameraPipelineStarDetection>& detections) {
            SolverContext context(this);
            context.m_disableRollRecovery = disableRollRecovery;
            std::swap(context.m_sirilRangeCache, m_sirilRangeCache);
            std::swap(context.m_sirilIndexCache, m_sirilIndexCache);
            CameraPlateSolveResult once = context.solve(runSettings, imageSize, captureDateTime, detections);
            once.m_profileSummary = context.profileSummary();
            std::swap(context.m_sirilRangeCache, m_sirilRangeCache);
            std::swap(context.m_sirilIndexCache, m_sirilIndexCache);
            evictSirilRangeCacheIfNeeded();
            return once;
        };

        CameraPlateSolveResult runResult = solveOnce(starDetections);

        // All-sky handedness: for a fisheye/wide (non-rectilinear) context, ALWAYS also solve
        // the mirrored detection set and adopt it when decisively better (see
        // mirrorResultClearlyBetter). Gated to non-rectilinear so REAL/narrow stays byte-
        // identical and pays no 2x cost. The retry must run regardless of runResult.m_solved:
        // the all-sky failures return m_solved=true on a wrong-roll pose, so a failure-gated
        // retry would miss exactly the cases that need it.
        const bool fisheyeContext =
            (runSettings.m_lensProjection != CameraSettings::LensProjectionRectilinear);
        if (fisheyeContext && mirrorHandednessEnabled && !isCancellationRequested())
        {
            const double maxX = static_cast<double>(imageSize.width() - 1);
            // Solve the horizontally-mirrored detection set on a COPY, so the original
            // detections' centroids are never disturbed. Each mirrored detection keeps the same
            // index as its original, so an adopted mirror's match annotations can be transferred
            // back by index while the original centroid (which the caller/harness reads for
            // validation) stays put. This ties label -> original-position by index, so an
            // adopted mirror shows its matches at the true image positions with no coordinate
            // round-trip on the shared vector.
            QVector<CameraPipelineStarDetection> mirroredDetections = starDetections;
            for (CameraPipelineStarDetection& detection : mirroredDetections)
            {
                detection.m_center.setX(maxX - detection.m_center.x());
                clearDetectionSolveState(detection);
            }
            const CameraPlateSolveResult mirrorResult = solveOnce(mirroredDetections);
            const bool adoptMirror = mirrorResultClearlyBetter(mirrorResult, runResult);
            // Split across statements: MSVC/Qt qCInfo silently truncates a single line past
            // ~10 streamed fields, so keep each short.
            qCInfo(cameraPlateSolverLog).nospace()
                << "CameraPlateSolver: handedness " << reason << " adopt=" << adoptMirror
                << " normal solved=" << runResult.m_solved
                << " m=" << runResult.m_matchedStars
                << " rms=" << runResult.m_rmsErrorPixels
                << " az=" << runResult.m_azimuthDegrees
                << " roll=" << runResult.m_rollDegrees;
            qCInfo(cameraPlateSolverLog).nospace()
                << "CameraPlateSolver: handedness " << reason << " adopt=" << adoptMirror
                << " mirror solved=" << mirrorResult.m_solved
                << " m=" << mirrorResult.m_matchedStars
                << " rms=" << mirrorResult.m_rmsErrorPixels
                << " az=" << mirrorResult.m_azimuthDegrees
                << " roll=" << mirrorResult.m_rollDegrees;
            if (adoptMirror)
            {
                runResult = mirrorResult;
                runResult.m_mirrored = true;
                // Transfer the mirror pose's match annotations onto the original detections,
                // keeping each detection's ORIGINAL centroid; reflect the projected-centre x
                // back into the original image frame. The solve does not resize/reorder the
                // detection vector (the normal path relies on the same in-place annotation), so
                // index i is the same physical detection in both.
                const int transferCount = std::min(starDetections.size(), mirroredDetections.size());
                for (int i = 0; i < transferCount; ++i)
                {
                    CameraPipelineStarDetection& dst = starDetections[i];
                    const CameraPipelineStarDetection& src = mirroredDetections.at(i);
                    dst.m_solved = src.m_solved;
                    dst.m_label = src.m_label;
                    dst.m_matchDistancePixels = src.m_matchDistancePixels;
                    dst.m_catalogMagnitude = src.m_catalogMagnitude;
                    dst.m_catalogRightAscensionDegrees = src.m_catalogRightAscensionDegrees;
                    dst.m_catalogDeclinationDegrees = src.m_catalogDeclinationDegrees;
                    dst.m_catalogSpectralType = src.m_catalogSpectralType;
                    dst.m_projectedCenter = src.m_solved
                        ? QPointF(maxX - src.m_projectedCenter.x(), src.m_projectedCenter.y())
                        : QPointF();
                    // dst.m_center intentionally left at the original detection centroid.
                }
            }
            // If not adopted, starDetections keeps the normal solve's annotations (untouched).
        }

        solveRunProfiles.append({
            disableRollRecovery ? QStringLiteral("%1-no-roll-recovery").arg(reason) : reason,
            runTimer.elapsed(),
            runResult.m_solved,
            runResult.m_matchedStars,
            runResult.m_azimuthDegrees,
            runResult.m_elevationDegrees,
            runResult.m_rollDegrees,
            runResult.m_rmsErrorPixels,
            runResult.m_fovDegrees,
            runResult.m_centerOffsetXPixels,
            runResult.m_centerOffsetYPixels,
            runResult.m_distortionK1,
            runResult.m_profileSummary
        });

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
            outerProfile.append(prefix + QStringLiteral("az=%1").arg(runProfile.azimuthDegrees, 0, 'f', 4));
            outerProfile.append(prefix + QStringLiteral("el=%1").arg(runProfile.elevationDegrees, 0, 'f', 4));
            outerProfile.append(prefix + QStringLiteral("roll=%1").arg(runProfile.rollDegrees, 0, 'f', 4));
            outerProfile.append(prefix + QStringLiteral("rms=%1").arg(runProfile.rmsErrorPixels, 0, 'f', 4));
            outerProfile.append(prefix + QStringLiteral("fov=%1").arg(runProfile.fovDegrees, 0, 'f', 4));
            outerProfile.append(prefix + QStringLiteral("cx=%1").arg(runProfile.centerOffsetXPixels, 0, 'f', 3));
            outerProfile.append(prefix + QStringLiteral("cy=%1").arg(runProfile.centerOffsetYPixels, 0, 'f', 3));
            outerProfile.append(prefix + QStringLiteral("k1=%1").arg(runProfile.distortionK1, 0, 'f', 6));
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
        CameraSettings retrySettings(cappedSettings);
        retrySettings.m_plateSolveMaxMagnitude = 15.0f;
        retrySettings.m_plateSolveAzElSearchRadius = static_cast<float>(std::max(
            static_cast<double>(retrySettings.m_plateSolveAzElSearchRadius),
            kRetrySearchRadiusDegrees));
        qDebug() << "CameraPlateSolver: trying dense narrow bright catalog before full catalog"
                 << "maxMagnitude" << retrySettings.m_plateSolveMaxMagnitude
                 << "searchRadius" << retrySettings.m_plateSolveAzElSearchRadius;
        markAttemptedDenseNarrowBrightCatalogMagnitude(retrySettings.m_plateSolveMaxMagnitude);
        result = runSolve(retrySettings, QStringLiteral("bright-catalog"));
    }
    if (tryWithoutRollBeforeRollPrior)
    {
        CameraSettings retrySettings(cappedSettings);
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
        CameraPlateSolveResult initialResult = runSolve(cappedSettings, QStringLiteral("initial"));
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
            CameraSettings retrySettings(cappedSettings);
            retrySettings.m_plateSolveMaxMagnitude = static_cast<float>(retryMagnitude);
            retrySettings.m_plateSolveAzElSearchRadius = static_cast<float>(std::max(
                static_cast<double>(retrySettings.m_plateSolveAzElSearchRadius),
                kRetrySearchRadiusDegrees));
            qDebug() << "CameraPlateSolver: retrying dense narrow direction solve with bright catalog"
                     << "maxMagnitude" << retrySettings.m_plateSolveMaxMagnitude
                     << "searchRadius" << retrySettings.m_plateSolveAzElSearchRadius;
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

    // Deepen-escape retry: the mirror image of depth-escape. A detection-rich narrow
    // direction-seeded image can fail purely because the *requested* catalog depth is too
    // shallow for the field — only a handful of in-FoV catalog candidates face hundreds of
    // detections, too few for an acceptable dense fit (m51@14: 509 detections vs 103
    // candidates fails, while the identical image/seed solves at mag 15 with 149/165
    // matched). Retry one then two magnitudes deeper and adopt any *solved* result: unlike
    // depth-escape (which escapes INTO the lenient sparse-catalog acceptance regime and so
    // needs its own tight-fit gate), deepening moves into the denser, stricter acceptance
    // regime — an adopted result is exactly as trustworthy as the same solve requested at
    // that depth directly. No rms gate here: correct galaxy-field solves carry rms 12-13px
    // from fuzzy detections (m51@15: rms 13.17), which a depth-escape-style <=3px bar
    // would wrongly reject. The catalog-starvation trigger (candidates <= detections/3,
    // detections >= 128) keeps this off every other current failure path.
    // Each deepen depth also sweeps a few coarse azimuth recenter offsets: the recenter
    // ladder only runs at the *requested* depth, so a catalog-starved field whose seed is
    // also ~1 degree off (m51@14: az seed 92.0 vs true 93.0) needs depth and recentering
    // together — at the raw seed, mag 15 reaches only 46 matches while the identical
    // image solves with 149 from the true centre. Budgeted to keep the failure path
    // bounded; any solved result early-stops.
    const auto attemptDeepenEscape = [&]() {
        const bool catalogStarvedNarrowDirectionSolve =
            solveUsesDirection && !solveUsesRoll && SolverContext::isNarrowField(settings)
            && (starDetections.size() >= 128)
            && (result.m_catalogCandidateStars > 0)
            && (result.m_catalogCandidateStars <= (starDetections.size() / 3));
        if (result.m_solved
            || !catalogStarvedNarrowDirectionSolve
            || isCancellationRequested())
        {
            return;
        }
        const QVector<CameraPipelineStarDetection> detectionsBeforeDeepenEscape = starDetections;
        const double requestedMag = static_cast<double>(settings.m_plateSolveMaxMagnitude);
        const double deepenFovDegrees = std::max(0.1, static_cast<double>(settings.m_fov));
        const std::array<double, 5> deepenAzimuthOffsets = {{
            0.0, deepenFovDegrees * 0.75, -deepenFovDegrees * 0.75, deepenFovDegrees, -deepenFovDegrees
        }};
        constexpr int kMaxDeepenEscapeRuns = 6;
        int deepenEscapeRuns = 0;
        bool deepenAliasContaminated = false;
        for (double escapeMagnitude : {requestedMag + 1.0, requestedMag + 2.0})
        {
            if (result.m_solved
                || deepenAliasContaminated
                || (escapeMagnitude > 16.5)
                || hasAttemptedDenseNarrowBrightCatalogMagnitude(escapeMagnitude))
            {
                continue;
            }
            markAttemptedDenseNarrowBrightCatalogMagnitude(escapeMagnitude);
            for (double azimuthOffset : deepenAzimuthOffsets)
            {
                if (isCancellationRequested() || (deepenEscapeRuns >= kMaxDeepenEscapeRuns)) {
                    break;
                }
                ++deepenEscapeRuns;
                CameraSettings retrySettings(cappedSettings);
                retrySettings.m_plateSolveMaxMagnitude = static_cast<float>(escapeMagnitude);
                retrySettings.m_azimuth = static_cast<float>(SolverContext::normalizeDegrees(
                    static_cast<double>(settings.m_azimuth) + azimuthOffset));
                qDebug() << "CameraPlateSolver: deepen-escape retry at deeper catalog"
                         << "maxMagnitude" << retrySettings.m_plateSolveMaxMagnitude
                         << "azimuthOffset" << azimuthOffset
                         << "detections" << starDetections.size()
                         << "candidates" << result.m_catalogCandidateStars;
                CameraPlateSolveResult escapeResult = runSolve(retrySettings, QStringLiteral("deepen-escape"));
                // Adopt only a solved result that matches most of the deeper catalog's
                // in-FoV candidates: a correct catalog-starved solve is near-complete
                // (m51@14 deepen: 150/165 = 0.91) while a wrong-roll alias that clears
                // the gate from an offset seed is not (m101@15 deepen: 134/328 = 0.41,
                // roll -25 vs true 87 — observed when this adoption was unconditional).
                const bool escapeDominatesCandidates =
                    escapeResult.m_solved
                    && (escapeResult.m_matchedStars
                        >= static_cast<int>(std::ceil(0.6 * std::max(1, escapeResult.m_catalogCandidateStars))));
                if (escapeDominatesCandidates)
                {
                    result = escapeResult;
                    break;
                }
                if (escapeResult.m_solved)
                {
                    // A solved-but-low-coverage result means the deeper catalog is
                    // feeding acceptable-looking aliases for this field — further
                    // offsets/depths can only find more of the same, so stop the whole
                    // escape rather than spend (and risk) the remaining budget.
                    qDebug() << "CameraPlateSolver: deepen-escape solved but matches too few of the deeper candidates, rejecting"
                             << "matches" << escapeResult.m_matchedStars
                             << "candidates" << escapeResult.m_catalogCandidateStars;
                    deepenAliasContaminated = true;
                    starDetections = detectionsBeforeDeepenEscape;
                    break;
                }
                starDetections = detectionsBeforeDeepenEscape;
            }
            if (result.m_solved) {
                break;
            }
        }
    };
    // A catalog-starved failure is recognisable from the very first attempt, and the
    // recenter ladder below cannot help it (it re-tries the same starved depth up to 16
    // more times — ~30s on m51@14 before the deepen that actually solves it). Try the
    // deepen escape first; the ladder and the late fallback below only run if it fails.
    attemptDeepenEscape();

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
        recenterOffsets.reserve(18);
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
        // El now matches az coverage: ±0.33/0.5/0.75/1.0·fov (sweep data showed el at 54%
        // vs az at 78%, partly due to the missing el ±1.0·fov tier).
        const std::array<std::pair<double, double>, 18> defaultRecenterOffsets = {{
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
            { 0.0, fovDegrees },
            { 0.0, -fovDegrees },
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
        // Budget enough attempts to reach the finer az tier (indices 4..7), the
        // elevation tier (indices 8..13), and now also the el ±1.0·fov tier (indices
        // 14..15); the coarse offsets that already resolve a case early-stop well before
        // this, so the extra budget only costs time on otherwise-failing solves.
        const int maxRecenterAttempts = 16;
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
            CameraSettings retrySettings(cappedSettings);
            retrySettings.m_azimuth = static_cast<float>(SolverContext::normalizeDegrees(
                static_cast<double>(settings.m_azimuth) + offset.first));
            retrySettings.m_elevation = static_cast<float>(std::clamp(
                static_cast<double>(settings.m_elevation) + offset.second,
                -90.0,
                90.0));
            retrySettings.m_plateSolveAzElSearchRadius = static_cast<float>(std::max(
                static_cast<double>(retrySettings.m_plateSolveAzElSearchRadius),
                kRetrySearchRadiusDegrees));
            qDebug() << "CameraPlateSolver: retrying dense narrow direction solve with recentered seed"
                     << "azimuth" << retrySettings.m_azimuth
                     << "elevation" << retrySettings.m_elevation
                     << "searchRadius" << retrySettings.m_plateSolveAzElSearchRadius;
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
                // The absolute strong-match floor (>= 80) is unreachable on sparse fields
                // whose whole in-FoV catalog is a few dozen stars, so a solved sparse
                // result used to keep burning the remaining recenter budget for nothing
                // (narrow-4: solved with 42/50 candidates at attempt 4, then ran 13 more
                // attempts that never beat it). A solved result matching most of the
                // available candidates is equally conclusive evidence.
                const bool dominantSparseCoverage =
                    bestRecenterResult.m_matchedStars >= std::max(
                        settings.m_plateSolveMinMatches + 8,
                        static_cast<int>(std::ceil(
                            0.6 * std::max(1, bestRecenterResult.m_catalogCandidateStars))));
                if (bestRecenterResult.m_solved
                    && ((bestRecenterResult.m_matchedStars >= strongRecenterMatchCount)
                        || dominantSparseCoverage))
                {
                    qDebug() << "CameraPlateSolver: stopping dense narrow recenter retries after strong solved candidate"
                             << "matches" << bestRecenterResult.m_matchedStars
                             << "candidates" << bestRecenterResult.m_catalogCandidateStars
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

    // For an already-solved result with substantial match support, the bright-catalog
    // re-run can only re-derive the same pose from a shallower catalog (cluster-m7: a
    // 9.4s mag-13 re-run of a 1447-match solve adopted the identical pose with half the
    // matches; nebula-c11: a 4.3s re-run was discarded) — its useful work is rescuing
    // unsolved or weakly-supported results.
    if (!result.m_solved || (result.m_matchedStars < 120)) {
        retryDenseNarrowDirectionWithBrightCatalog();
    }

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
        CameraSettings retrySettings(cappedSettings);
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

    // Depth-escape retry. A dense narrow direction-seeded solve can fail purely because
    // the requested catalog is deep enough for faint stars to feed a coincidental (often
    // 180°) roll-alias that out-counts the true roll — the *same* field/seed solves at a
    // shallower catalog (verified on the synthetic random corpus: rand-007/027/008 fail at
    // mag 13 but solve at mag 11/12, zero seed offset). When still unsolved, retry one then
    // two magnitudes shallower and adopt only a *solved* result. Acceptance still requires a
    // tight, well-supported fit, so a wrong pose at shallower depth is rejected — this
    // cannot introduce a false positive, and (gated to already-failed dense solves) cannot
    // regress a passing case.
    // Density floor lower than denseNarrowDirectionSolve's (>128): the depth-induced
    // roll-alias also strikes moderate fields (e.g. synth-rand-015/028/032 at 87-117
    // detections solve at mag 11/12 but not 13), which the >128 gate wrongly excluded.
    const bool moderateNarrowDirectionSolve =
        solveUsesDirection && !solveUsesRoll && SolverContext::isNarrowField(settings)
        && (starDetections.size() > 64);
    if (!result.m_solved
        && moderateNarrowDirectionSolve
        && !isCancellationRequested()
        && (settings.m_plateSolveMaxMagnitude >= 12.0f))
    {
        // The motivating synthetic wins (rand-007/008/027/045) all solve sub-pixel with
        // ~100+ matches at the shallower catalog. A solved-but-loose result (e.g. m51@14:
        // 10 matches, rms 14.26px) is a wrong-pose false positive, not a rescued true
        // solve, so require a tight residual fit before adopting.
        constexpr double kDepthEscapeMaxAcceptableRmsPixels = 3.0;
        const QVector<CameraPipelineStarDetection> detectionsBeforeDepthEscape = starDetections;
        const double requestedMag = static_cast<double>(settings.m_plateSolveMaxMagnitude);
        for (double escapeMagnitude : {requestedMag - 1.0, requestedMag - 2.0})
        {
            if ((escapeMagnitude < 10.0) || hasAttemptedDenseNarrowBrightCatalogMagnitude(escapeMagnitude)) {
                continue;
            }
            markAttemptedDenseNarrowBrightCatalogMagnitude(escapeMagnitude);
            CameraSettings retrySettings(cappedSettings);
            retrySettings.m_plateSolveMaxMagnitude = static_cast<float>(escapeMagnitude);
            qDebug() << "CameraPlateSolver: depth-escape retry at shallower catalog"
                     << "maxMagnitude" << retrySettings.m_plateSolveMaxMagnitude;
            CameraPlateSolveResult escapeResult = runSolve(retrySettings, QStringLiteral("depth-escape"));
            if (escapeResult.m_solved && (escapeResult.m_rmsErrorPixels <= kDepthEscapeMaxAcceptableRmsPixels))
            {
                result = escapeResult;
                break;
            }
            if (escapeResult.m_solved)
            {
                qDebug() << "CameraPlateSolver: depth-escape retry solved but fit too loose, rejecting"
                         << "maxMagnitude" << retrySettings.m_plateSolveMaxMagnitude
                         << "matches" << escapeResult.m_matchedStars
                         << "rms" << escapeResult.m_rmsErrorPixels;
            }
            starDetections = detectionsBeforeDepthEscape;
        }
    }

    // Late deepen-escape fallback: the early attempt before the recenter ladder covers
    // the starvation signature visible after the initial run; this one catches it when
    // the signature only emerges from the ladder/bright-catalog results (attempted
    // magnitudes are deduped, so nothing reruns).
    attemptDeepenEscape();

    appendOuterProfile();
    return result;
}
