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

// Solve accept/reject gates + false-alarm scoring + residual gating (WS5).

double CameraPlateSolver::SolverContext::medianDistancePixels(const QVector<Match>& matches)
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

double CameraPlateSolver::SolverContext::maxDistancePixels(const QVector<Match>& matches)
{
    double maxDistance = 0.0;
    for (const Match& match : matches) {
        maxDistance = std::max(maxDistance, match.distancePixels);
    }
    return maxDistance;
}

double CameraPlateSolver::SolverContext::sparseGuidedMaxRms(const CameraSettings& settings, int matchCount)
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

bool CameraPlateSolver::SolverContext::gateAblationDisabled(const char* token)
{
    static const QString disabled = qEnvironmentVariable("SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_GATE");
    if (disabled.isEmpty()) {
        return false;
    }
    const QStringList tokens = disabled.split(QLatin1Char(','), Qt::SkipEmptyParts);
    for (const QString& t : tokens) {
        if (t.trimmed().compare(QLatin1String(token), Qt::CaseInsensitive) == 0) {
            return true;
        }
    }
    return false;
}

double CameraPlateSolver::SolverContext::poseFalseAlarmLogOdds(const PlateSolveCatalogContext& catalogContext, const FinalMatchPassEvaluation& finalPass, const QSize& imageSize, double matchRadiusPixels, int detectionCount)
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

double CameraPlateSolver::SolverContext::poseVerificationLogOdds(const PlateSolveCatalogContext& catalogContext, const FinalMatchPassEvaluation& finalPass, const QSize& imageSize, double matchRadiusPixels, int detectionCount)
{
    Q_UNUSED(catalogContext);
    if (!finalPass.projectorValid
        || finalPass.finalMatches.isEmpty()
        || (imageSize.width() <= 0)
        || (imageSize.height() <= 0)
        || (matchRadiusPixels <= 0.0)
        || (detectionCount <= 0))
    {
        return 0.0;
    }

    // Astrometry.net-style Bayesian verification (Lang et al. 2010, after Sutherland & Saunders
    // 1992). Per FIELD detection, compare:
    //   H1 (this WCS is correct): background density rho_bg + a foreground Gaussian bump wherever a
    //       reference star projects;
    //   H0 (random WCS): background density rho_bg everywhere.
    // The per-detection likelihood ratio is 1 + foreground/rho_bg for a matched detection and
    //   exactly 1 (log 0) for an unmatched one -- an unmatched detection is background under BOTH
    // hypotheses, so it carries no evidence. (An earlier revision wrongly charged log(f) per
    // unmatched detection; the shadow log showed that over-penalised correct deep/nebular fields
    // whose detector returns hundreds of sub-catalogue-depth detections -- e.g. stars-narrow-3,
    // ngc-2403 scored below the pollux false solve. Dropping that term is the correct model.)
    // rho_bg is the empirical detection density, so a dense field automatically discounts each
    // match: a chance coincidence sits near background (ratio ~1, log ~0) while a genuine tight
    // match rises far above it, which is what separates a correct solve from a dense wrong one.
    constexpr double kForegroundDetectability = 0.75; // (1-f): prob. a projected star is a real, detectable source
    const double area = static_cast<double>(imageSize.width()) * static_cast<double>(imageSize.height());
    // Positional-uncertainty scale: real matches sit inside the radius but carry pose-model error,
    // so r/2 (as in poseFalseAlarmLogOdds) is the effective 1-sigma.
    const double sigma = std::max(1.0, matchRadiusPixels * 0.5);
    const double twoSigmaSquared = 2.0 * sigma * sigma;
    const double backgroundDensity = std::max(1.0, static_cast<double>(detectionCount)) / area;

    double logOdds = 0.0;
    for (const Match& match : finalPass.finalMatches)
    {
        const double d = match.distancePixels;
        if (d > matchRadiusPixels) {
            continue; // shouldn't happen for an accepted match; skip defensively
        }
        // Foreground positional density of one reference star: (1/(2*pi*sigma^2)) exp(-d^2/2sigma^2).
        const double foregroundDensity = std::exp(-(d * d) / twoSigmaSquared) / (kPi * twoSigmaSquared);
        logOdds += std::log(1.0 + kForegroundDetectability * foregroundDensity / backgroundDensity);
    }
    return logOdds;
}

double CameraPlateSolver::SolverContext::detectionMatchWeight(const CameraPipelineStarDetection& detection)
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

double CameraPlateSolver::SolverContext::weightedRmsDistancePixels(const QVector<CameraPipelineStarDetection>& starDetections, const QVector<Match>& matches)
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

bool CameraPlateSolver::SolverContext::hasGeometricallyConsistentMatches(const QVector<CameraPipelineStarDetection>& starDetections, const QVector<ProjectedCatalogStar>& projectedStars, const QVector<Match>& matches, double matchRadiusPixels)
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

QVector<CameraPlateSolver::SolverContext::Match> CameraPlateSolver::SolverContext::rejectOutlierMatches(const QVector<Match>& matches, int minMatches, double matchRadiusPixels, int* outlierCount)
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

double CameraPlateSolver::SolverContext::trimmedWorstDistancePixels(const QVector<Match>& matches, int dropCount)
{
    if (matches.isEmpty()) {
        return 0.0;
    }
    if ((dropCount <= 0) || (matches.size() <= dropCount + 1)) {
        return maxDistancePixels(matches);
    }
    QVector<double> distances;
    distances.reserve(matches.size());
    for (const Match& match : matches) {
        distances.append(match.distancePixels);
    }
    std::sort(distances.begin(), distances.end());
    return distances[distances.size() - 1 - dropCount];
}

bool CameraPlateSolver::SolverContext::passesResidualGates(const QVector<Match>& matches, double rmsErrorPixels, double maxErrorPixels, const ResidualGates& gates)
{
    if (matches.size() < gates.minMatches) {
        return false;
    }
    const double worstForGate = (gates.worstTrimCount > 0)
        ? trimmedWorstDistancePixels(matches, gates.worstTrimCount)
        : maxErrorPixels;
    return (rmsErrorPixels <= gates.maxRms)
        && (medianDistancePixels(matches) <= gates.maxMedian)
        && (worstForGate <= gates.maxWorst);
}

bool CameraPlateSolver::SolverContext::isAcceptableBlindSolve(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<Match>& matches, double rmsErrorPixels, double maxErrorPixels)
{
    ResidualGates gates;
    gates.minMatches = std::max(settings.m_plateSolveMinMatches + 2,
        std::min(10, std::max(6, static_cast<int>(std::ceil(static_cast<double>(starDetections.size()) * 0.20)))));
    gates.maxRms = std::min(settings.m_plateSolveFinalMatchRadius * 0.70, 20.0);
    gates.maxMedian = std::min(settings.m_plateSolveFinalMatchRadius * 0.55, 15.0);
    gates.maxWorst = std::min(settings.m_plateSolveFinalMatchRadius * 1.10, 45.0);
    return passesResidualGates(matches, rmsErrorPixels, maxErrorPixels, gates);
}

bool CameraPlateSolver::SolverContext::isAcceptableSparseWideBlindSolve(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const FinalMatchPassEvaluation& evaluation)
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

bool CameraPlateSolver::SolverContext::isStrongGuidedSolve(const CameraSettings& settings, int minMatchCount, const Evaluation& evaluation)
{
    if (!evaluation.valid) {
        return false;
    }

    const int minAcceptedMatches = std::max(minMatchCount + 2, 6);
    const double maxRmsError = std::min(settings.m_plateSolveFinalMatchRadius * 0.45, 12.0);
    return (evaluation.matchCount >= minAcceptedMatches) && (evaluation.rmsErrorPixels <= maxRmsError);
}

int CameraPlateSolver::SolverContext::minimumDirectionSeedAcceptedMatches(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections)
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

double CameraPlateSolver::SolverContext::maxDirectionSeedRmsError(const CameraSettings& settings, int matchCount)
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

double CameraPlateSolver::SolverContext::maxNarrowGuidedFovDeltaDegrees(const CameraSettings& settings)
{
    return std::max(0.08, static_cast<double>(settings.m_fov) * 0.08);
}

bool CameraPlateSolver::SolverContext::isAcceptableNarrowGuidedFov(const CameraSettings& settings, double fovDegrees)
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

QString CameraPlateSolver::SolverContext::narrowGuidedFovRejectionReason(const CameraSettings& settings, double fovDegrees)
{
    return QStringLiteral("FoV %1 outside start FoV %2 +/- %3")
        .arg(fovDegrees, 0, 'f', 3)
        .arg(static_cast<double>(settings.m_fov), 0, 'f', 3)
        .arg(maxNarrowGuidedFovDeltaDegrees(settings), 0, 'f', 3);
}

bool CameraPlateSolver::SolverContext::hasImplausibleBrightDisagreement(const CameraSettings& settings, const FinalMatchPassEvaluation& finalPass)
{
    if (!plateSolveStartUsesDirection(settings)
        || !isNarrowField(settings)
        || !finalPass.projectorValid)
    {
        return false;
    }

    // "Almost none of them matched" only means something when the pose predicts a decent
    // number of bright stars in the first place.
    if (finalPass.brightProjectedStars < 6) {
        return false;
    }

    // Sparse solves only. Bright-star agreement is decisive when the bright stars are
    // essentially all the evidence there is (the wrong-roll alias this exists to stop was
    // accepted on 15 matches), but it is the LEAST reliable evidence in a dense nebulous
    // field, where nebulosity and saturated cores corrupt bright detections and their
    // magnitudes while a thousand faint stars pin the pose perfectly well. Left unguarded
    // this vetoed nebula-c11's correct pose - 1285 matches, roll -4.9 against a declared 0,
    // all three named stars projected - in favour of a denser wrong-roll alias at -19.8.
    // The threshold is the same "dense evidence" bar hasDenseFinalEvidenceOverridingSeedRadial
    // uses, so the two agree on what counts as a densely-supported pose.
    if (finalPass.finalMatches.size()
        >= static_cast<qsizetype>(std::max(settings.m_plateSolveMinMatches + 50, 80)))
    {
        return false;
    }

    // All three have to be bad at once. The conjunction is deliberate - a correct pose in a
    // thin, cloud-hit frame can look bad on any one of them, but not on all three.
    //
    // Thresholds are the union of two measured guiding sessions, taking the worst case on
    // each side so both are covered (correct | wrong):
    //   bright projected matched   5-12 of 12  |  0-3 of 12
    //   bright magnitude error     0.03-1.38   |  0.97-2.92
    //   rms                        0.85-13.83  |  12.35-15.53 px
    // The first session's wrong poses sat at <= 2 matched with magnitude error > 1.2, and
    // thresholds set there missed the second session's, which reached 3 matched at 0.97 -
    // each of its three wrong poses escaped on a different one of the two. Note the wide
    // overlap on rms alone (a correct pose reached 13.83), and that bright-projected
    // agreement is the one measurement that separates cleanly in both sessions: no correct
    // pose ever fell below 5 of 12, no wrong pose ever reached 4.
    return (finalPass.matchedBrightProjectedStars <= 3)
        && (finalPass.brightDetectionMagnitudeError > 0.90)
        && (finalPass.rmsErrorPixels > (0.35 * static_cast<double>(settings.m_plateSolveFinalMatchRadius)));
}

QString CameraPlateSolver::SolverContext::brightDisagreementRejectionReason(const FinalMatchPassEvaluation& finalPass)
{
    return QStringLiteral("bright stars disagree: %1/%2 bright projected matched, magnitude error %3, rms %4")
        .arg(finalPass.matchedBrightProjectedStars)
        .arg(finalPass.brightProjectedStars)
        .arg(finalPass.brightDetectionMagnitudeError, 0, 'f', 2)
        .arg(finalPass.rmsErrorPixels, 0, 'f', 2);
}

double CameraPlateSolver::SolverContext::maxDirectionSeedDistanceDegrees(const CameraSettings& settings)
{
    // 3 FoV (2 deg floor for tiny fields) is far beyond any legitimate seed error - the
    // recenter ladder's own scoring already treats poses much nearer than this as
    // essentially disqualified (quadratic distance penalty) - yet well inside where
    // observed false accepts land.
    return std::max(3.0 * static_cast<double>(settings.m_fov), 2.0);
}

double CameraPlateSolver::SolverContext::directionSeedSeparationDegrees(const CameraSettings& settings, double azimuthDegrees, double elevationDegrees)
{
    const SkyVector seedVector = normalize(vectorFromAltAz(settings.m_azimuth, settings.m_elevation));
    const SkyVector solvedVector = normalize(vectorFromAltAz(azimuthDegrees, elevationDegrees));
    const double cosSeparation = std::clamp(dot(seedVector, solvedVector), -1.0, 1.0);
    return std::acos(cosSeparation) * (180.0 / 3.14159265358979323846);
}

bool CameraPlateSolver::SolverContext::isAcceptableDirectionSeedDistance(const CameraSettings& settings, double azimuthDegrees, double elevationDegrees)
{
    if (!plateSolveStartUsesDirection(settings) || !isNarrowField(settings)) {
        return true;
    }
    if (!std::isfinite(azimuthDegrees) || !std::isfinite(elevationDegrees)) {
        return false;
    }

    return directionSeedSeparationDegrees(settings, azimuthDegrees, elevationDegrees)
        <= maxDirectionSeedDistanceDegrees(settings);
}

QString CameraPlateSolver::SolverContext::directionSeedDistanceRejectionReason(const CameraSettings& settings, double azimuthDegrees, double elevationDegrees)
{
    return QStringLiteral("solution %1 deg from seed direction (limit %2 deg)")
        .arg(directionSeedSeparationDegrees(settings, azimuthDegrees, elevationDegrees), 0, 'f', 2)
        .arg(maxDirectionSeedDistanceDegrees(settings), 0, 'f', 2);
}

CameraPlateSolver::SolverContext::ResidualGates CameraPlateSolver::SolverContext::directionSeedResidualGates(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, int matchCount)
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
    // For a dense wide-fisheye solve, ignore the few most extreme rim residuals when
    // applying the worst gate -- rim distortion at ~80 deg field angle inflates the
    // single worst star even on a correct pose, and one such star should not veto a
    // 70-star, median-4 px solution. Capped at 3 and only above 20 matches so sparse
    // solves keep the strict absolute-max gate.
    if (!narrowField && isWidePlateSolveContext(settings) && (matchCount >= 20)) {
        gates.worstTrimCount = std::min(3,
            std::max(1, static_cast<int>(std::ceil(matchCount * 0.03))));
    }
    return gates;
}

QString CameraPlateSolver::SolverContext::directionSeedRejectionReason(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<Match>& matches, double rmsErrorPixels, double maxErrorPixels)
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
    const double worstForGate = (gates.worstTrimCount > 0)
        ? trimmedWorstDistancePixels(matches, gates.worstTrimCount)
        : maxErrorPixels;
    if (worstForGate > gates.maxWorst) {
        reasons.append(QStringLiteral("max %1 > %2").arg(worstForGate, 0, 'f', 2).arg(gates.maxWorst, 0, 'f', 2));
    }
    return reasons.isEmpty() ? QStringLiteral("accepted") : reasons.join(QStringLiteral(", "));
}

bool CameraPlateSolver::SolverContext::isAcceptableDirectionSeedSolve(const CameraSettings& settings, int minMatchCount, const Evaluation& evaluation)
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

bool CameraPlateSolver::SolverContext::isAcceptableDirectionSeedSolve(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<Match>& matches, double rmsErrorPixels, double maxErrorPixels)
{
    const ResidualGates gates = directionSeedResidualGates(settings, starDetections, matches.size());
    return passesResidualGates(matches, rmsErrorPixels, maxErrorPixels, gates);
}

bool CameraPlateSolver::SolverContext::isAcceptableElevationSeedEvaluation(const CameraSettings& settings, int minMatchCount, const Evaluation& evaluation)
{
    if (!evaluation.valid) {
        return false;
    }

    const int minAcceptedMatches = std::max(minMatchCount + 1, 5);
    const double maxRmsError = std::min(settings.m_plateSolveFinalMatchRadius * 0.85, 22.0);
    return (evaluation.matchCount >= minAcceptedMatches) && (evaluation.rmsErrorPixels <= maxRmsError);
}

bool CameraPlateSolver::SolverContext::isAcceptableElevationSeedSolve(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<Match>& matches, double rmsErrorPixels, double maxErrorPixels)
{
    ResidualGates gates;
    gates.minMatches = std::max(settings.m_plateSolveMinMatches + 1,
        std::min(8, std::max(5, static_cast<int>(std::ceil(static_cast<double>(starDetections.size()) * 0.15)))));
    gates.maxRms = std::min(settings.m_plateSolveFinalMatchRadius * 0.85, 24.0);
    gates.maxMedian = std::min(settings.m_plateSolveFinalMatchRadius * 0.70, 18.0);
    gates.maxWorst = std::min(settings.m_plateSolveFinalMatchRadius * 1.20, 50.0);
    return passesResidualGates(matches, rmsErrorPixels, maxErrorPixels, gates);
}

double CameraPlateSolver::SolverContext::calibrationMagnitude(const Evaluation& evaluation)
{
    return std::fabs(evaluation.centerOffsetXPixels)
        + std::fabs(evaluation.centerOffsetYPixels)
        + 100.0 * std::fabs(evaluation.distortionK1);
}

