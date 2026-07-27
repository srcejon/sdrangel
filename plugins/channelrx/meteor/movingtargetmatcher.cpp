///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#include "movingtargetmatcher.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
    constexpr double Pi = 3.14159265358979323846;
    constexpr double SpeedOfLightMPS = 299792458.0;
    constexpr double WGS84SemiMajorAxisM = 6378137.0;
    constexpr double WGS84EccentricitySquared = 6.69437999014e-3;
    constexpr int MaximumAlternativeCount = 3;

    struct Vector3
    {
        double m_x = 0.0;
        double m_y = 0.0;
        double m_z = 0.0;

        Vector3 operator+(const Vector3& other) const
        {
            return {m_x + other.m_x, m_y + other.m_y, m_z + other.m_z};
        }

        Vector3 operator-(const Vector3& other) const
        {
            return {m_x - other.m_x, m_y - other.m_y, m_z - other.m_z};
        }

        Vector3 operator*(double scalar) const
        {
            return {m_x * scalar, m_y * scalar, m_z * scalar};
        }
    };

    double dot(const Vector3& left, const Vector3& right)
    {
        return left.m_x * right.m_x + left.m_y * right.m_y + left.m_z * right.m_z;
    }

    double length(const Vector3& vector)
    {
        return std::sqrt(dot(vector, vector));
    }

    Vector3 geodeticToECEF(const MovingTargetMatcher::Site& site)
    {
        const double latitude = site.m_latitudeDegrees * Pi / 180.0;
        const double longitude = site.m_longitudeDegrees * Pi / 180.0;
        const double sinLatitude = std::sin(latitude);
        const double cosLatitude = std::cos(latitude);
        const double sinLongitude = std::sin(longitude);
        const double cosLongitude = std::cos(longitude);
        const double primeVerticalRadius = WGS84SemiMajorAxisM
            / std::sqrt(1.0 - WGS84EccentricitySquared * sinLatitude * sinLatitude);

        return {
            (primeVerticalRadius + site.m_altitudeM) * cosLatitude * cosLongitude,
            (primeVerticalRadius + site.m_altitudeM) * cosLatitude * sinLongitude,
            (primeVerticalRadius * (1.0 - WGS84EccentricitySquared) + site.m_altitudeM) * sinLatitude
        };
    }

    Vector3 enuVelocityToECEF(const MovingTargetMatcher::TargetState& target)
    {
        const double latitude = target.m_position.m_latitudeDegrees * Pi / 180.0;
        const double longitude = target.m_position.m_longitudeDegrees * Pi / 180.0;
        const double sinLatitude = std::sin(latitude);
        const double cosLatitude = std::cos(latitude);
        const double sinLongitude = std::sin(longitude);
        const double cosLongitude = std::cos(longitude);
        const Vector3 east {-sinLongitude, cosLongitude, 0.0};
        const Vector3 north {
            -sinLatitude * cosLongitude,
            -sinLatitude * sinLongitude,
            cosLatitude
        };
        const Vector3 up {
            cosLatitude * cosLongitude,
            cosLatitude * sinLongitude,
            sinLatitude
        };

        return east * target.m_eastVelocityMPS
            + north * target.m_northVelocityMPS
            + up * target.m_upVelocityMPS;
    }

    bool finiteSite(const MovingTargetMatcher::Site& site)
    {
        return std::isfinite(site.m_latitudeDegrees)
            && std::isfinite(site.m_longitudeDegrees)
            && std::isfinite(site.m_altitudeM)
            && (site.m_latitudeDegrees >= -90.0)
            && (site.m_latitudeDegrees <= 90.0)
            && (site.m_longitudeDegrees >= -180.0)
            && (site.m_longitudeDegrees <= 180.0);
    }

    bool finiteTarget(const MovingTargetMatcher::TargetState& target)
    {
        return target.m_dateTimeUtc.isValid()
            && finiteSite(target.m_position)
            && std::isfinite(target.m_eastVelocityMPS)
            && std::isfinite(target.m_northVelocityMPS)
            && std::isfinite(target.m_upVelocityMPS);
    }

    double bistaticDopplerOffset(
        const Vector3& targetPosition,
        const Vector3& targetVelocity,
        const Vector3& transmitterPosition,
        const Vector3& receiverPosition,
        double referenceFrequencyHz)
    {
        const Vector3 transmitterToTarget = targetPosition - transmitterPosition;
        const Vector3 receiverToTarget = targetPosition - receiverPosition;
        const double transmitterRange = length(transmitterToTarget);
        const double receiverRange = length(receiverToTarget);

        if ((transmitterRange < 1.0) || (receiverRange < 1.0)) {
            return std::numeric_limits<double>::quiet_NaN();
        }

        const double pathRateMPS = dot(targetVelocity, transmitterToTarget) / transmitterRange
            + dot(targetVelocity, receiverToTarget) / receiverRange;

        // A shortening TX-target-RX path has a negative path rate and a positive
        // received-frequency offset, matching Astronomy::dopplerToVelocity().
        return -referenceFrequencyHz * pathRateMPS / SpeedOfLightMPS;
    }

    double secondsBetween(const QDateTime& from, const QDateTime& to)
    {
        return (double) from.msecsTo(to) / 1000.0;
    }

    bool sameCandidate(
        const MovingTargetMatcher::Candidate& first,
        const MovingTargetMatcher::Candidate& second)
    {
        return (first.m_source == second.m_source)
            && (first.m_id == second.m_id)
            && (first.m_label == second.m_label);
    }

    void insertRankedCandidate(
        QVector<MovingTargetMatcher::Candidate>& candidates,
        const MovingTargetMatcher::Candidate& candidate)
    {
        if (!std::isfinite(candidate.m_scorePercent)) {
            return;
        }

        for (int index = 0; index < candidates.size(); ++index)
        {
            if (!sameCandidate(candidates[index], candidate)) {
                continue;
            }

            if (candidates[index].m_scorePercent >= candidate.m_scorePercent) {
                return;
            }

            candidates.removeAt(index);
            break;
        }

        int insertionIndex = 0;
        while ((insertionIndex < candidates.size())
            && (candidates[insertionIndex].m_scorePercent >= candidate.m_scorePercent))
        {
            ++insertionIndex;
        }
        candidates.insert(insertionIndex, candidate);

        const int maximumCandidateCount = MaximumAlternativeCount + 1;
        if (candidates.size() > maximumCandidateCount) {
            candidates.resize(maximumCandidateCount);
        }
    }

    MovingTargetMatcher::Candidate candidateFromMatch(
        const MovingTargetMatcher::Match& match)
    {
        MovingTargetMatcher::Candidate candidate;
        candidate.m_source = match.m_source;
        candidate.m_id = match.m_id;
        candidate.m_label = match.m_label;
        candidate.m_scorePercent = match.m_scorePercent;
        candidate.m_endpointResidualRMSHz = match.m_endpointResidualRMSHz;
        candidate.m_centerResidualHz = match.m_centerResidualHz;
        candidate.m_driftResidualHz = match.m_driftResidualHz;
        candidate.m_stateAgeS = match.m_stateAgeS;
        candidate.m_prediction = match.m_prediction;
        return candidate;
    }

    void setBestCandidate(
        MovingTargetMatcher::Match& match,
        const MovingTargetMatcher::Candidate& candidate)
    {
        match.m_hasCandidate = true;
        match.m_source = candidate.m_source;
        match.m_id = candidate.m_id;
        match.m_label = candidate.m_label;
        match.m_scorePercent = candidate.m_scorePercent;
        match.m_endpointResidualRMSHz = candidate.m_endpointResidualRMSHz;
        match.m_centerResidualHz = candidate.m_centerResidualHz;
        match.m_driftResidualHz = candidate.m_driftResidualHz;
        match.m_stateAgeS = candidate.m_stateAgeS;
        match.m_prediction = candidate.m_prediction;
    }

    void finalizeMatch(
        MovingTargetMatcher::Match& match,
        const QVector<MovingTargetMatcher::Candidate>& rankedCandidates,
        const MovingTargetMatcher::Tunables& tunables)
    {
        if (rankedCandidates.isEmpty()) {
            return;
        }

        setBestCandidate(match, rankedCandidates.first());
        match.m_secondBestScorePercent = rankedCandidates.size() > 1
            ? rankedCandidates[1].m_scorePercent
            : 0.0;
        match.m_alternatives.clear();

        for (int index = 1;
            (index < rankedCandidates.size())
                && (match.m_alternatives.size() < MaximumAlternativeCount);
            ++index)
        {
            if ((match.m_scorePercent - rankedCandidates[index].m_scorePercent)
                >= tunables.m_minimumScoreMarginPercent)
            {
                break;
            }
            match.m_alternatives.append(rankedCandidates[index]);
        }

        const double scoreMarginPercent = match.m_scorePercent
            - match.m_secondBestScorePercent;
        match.m_ambiguous =
            (match.m_scorePercent >= tunables.m_minimumMatchScorePercent)
            && (match.m_secondBestScorePercent > 0.0)
            && (scoreMarginPercent < tunables.m_minimumScoreMarginPercent);
        match.m_matched = (match.m_scorePercent >= tunables.m_minimumMatchScorePercent)
            && !match.m_ambiguous;
    }
}

double MovingTargetMatcher::elevationDegrees(
    const Site& observer,
    const TargetState& target,
    const QDateTime& dateTimeUtc)
{
    if (!dateTimeUtc.isValid() || !finiteSite(observer) || !finiteTarget(target)) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const Vector3 observerPosition = geodeticToECEF(observer);
    const Vector3 targetPosition = geodeticToECEF(target.m_position)
        + enuVelocityToECEF(target) * secondsBetween(target.m_dateTimeUtc, dateTimeUtc);
    const Vector3 lineOfSight = targetPosition - observerPosition;
    const double range = length(lineOfSight);

    if (!(range > 1.0)) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const double latitude = observer.m_latitudeDegrees * Pi / 180.0;
    const double longitude = observer.m_longitudeDegrees * Pi / 180.0;
    const Vector3 up {
        std::cos(latitude) * std::cos(longitude),
        std::cos(latitude) * std::sin(longitude),
        std::sin(latitude)
    };
    const double normalizedUp = std::clamp(dot(lineOfSight, up) / range, -1.0, 1.0);
    return std::asin(normalizedUp) * 180.0 / Pi;
}

MovingTargetMatcher::Prediction MovingTargetMatcher::predict(
    const Observation& observation,
    const TargetState& target)
{
    Prediction prediction;

    if (!observation.m_startDateTimeUtc.isValid()
        || !(observation.m_durationS > 0.0)
        || !(observation.m_referenceFrequencyHz > 0.0)
        || !finiteSite(observation.m_transmitter)
        || !finiteSite(observation.m_receiver)
        || !finiteTarget(target))
    {
        return prediction;
    }

    const Vector3 transmitterPosition = geodeticToECEF(observation.m_transmitter);
    const Vector3 receiverPosition = geodeticToECEF(observation.m_receiver);
    const Vector3 targetEpochPosition = geodeticToECEF(target.m_position);
    const Vector3 targetVelocity = enuVelocityToECEF(target);
    const double startOffsetS = secondsBetween(target.m_dateTimeUtc, observation.m_startDateTimeUtc);
    const double endOffsetS = startOffsetS + observation.m_durationS;
    const Vector3 targetStartPosition = targetEpochPosition + targetVelocity * startOffsetS;
    const Vector3 targetEndPosition = targetEpochPosition + targetVelocity * endOffsetS;

    prediction.m_startFrequencyOffsetHz = bistaticDopplerOffset(
        targetStartPosition,
        targetVelocity,
        transmitterPosition,
        receiverPosition,
        observation.m_referenceFrequencyHz);
    prediction.m_endFrequencyOffsetHz = bistaticDopplerOffset(
        targetEndPosition,
        targetVelocity,
        transmitterPosition,
        receiverPosition,
        observation.m_referenceFrequencyHz);
    prediction.m_valid = std::isfinite(prediction.m_startFrequencyOffsetHz)
        && std::isfinite(prediction.m_endFrequencyOffsetHz);

    if (prediction.m_valid)
    {
        prediction.m_centerFrequencyOffsetHz = 0.5
            * (prediction.m_startFrequencyOffsetHz + prediction.m_endFrequencyOffsetHz);
        prediction.m_frequencyDriftHz = prediction.m_endFrequencyOffsetHz
            - prediction.m_startFrequencyOffsetHz;
    }

    return prediction;
}

namespace {
    struct ObservationScales
    {
        double m_centerScaleHz = 1.0;
        double m_driftScaleHz = 1.0;
        double m_observedStartFrequencyHz = 0.0;
        double m_observedEndFrequencyHz = 0.0;
    };

    ObservationScales makeObservationScales(
        const MovingTargetMatcher::Observation& observation,
        const MovingTargetMatcher::Tunables& tunables)
    {
        ObservationScales scales;
        scales.m_observedStartFrequencyHz = observation.m_centerFrequencyOffsetHz
            - 0.5 * observation.m_frequencyDriftHz;
        scales.m_observedEndFrequencyHz = observation.m_centerFrequencyOffsetHz
            + 0.5 * observation.m_frequencyDriftHz;
        scales.m_centerScaleHz = std::max(
            tunables.m_minimumCenterScaleHz,
            std::fabs(observation.m_frequencySpanHz) * tunables.m_centerSpanScale);
        scales.m_driftScaleHz = std::max(
            tunables.m_minimumDriftScaleHz,
            std::fabs(observation.m_frequencyDriftHz) * tunables.m_driftScale);
        return scales;
    }

    constexpr double ApproximateDirectionScoreFactor = 0.85;

    MovingTargetMatcher::Candidate scoreCandidate(
        const MovingTargetMatcher::Observation& observation,
        const ObservationScales& scales,
        const QString& source,
        const QString& id,
        const QString& label,
        double stateAgeS,
        const MovingTargetMatcher::Prediction& prediction,
        double scoreFactor)
    {
        const double centerResidualHz = observation.m_centerFrequencyOffsetHz
            - prediction.m_centerFrequencyOffsetHz;
        const double driftResidualHz = observation.m_frequencyDriftHz
            - prediction.m_frequencyDriftHz;
        const double normalizedSquared =
            std::pow(centerResidualHz / scales.m_centerScaleHz, 2.0)
            + std::pow(driftResidualHz / scales.m_driftScaleHz, 2.0);
        const double scorePercent = 100.0 * scoreFactor * std::exp(-0.5 * normalizedSquared);
        const double startResidualHz = scales.m_observedStartFrequencyHz
            - prediction.m_startFrequencyOffsetHz;
        const double endResidualHz = scales.m_observedEndFrequencyHz
            - prediction.m_endFrequencyOffsetHz;
        const double endpointResidualRMSHz = std::sqrt(
            0.5 * (startResidualHz * startResidualHz + endResidualHz * endResidualHz));

        MovingTargetMatcher::Candidate candidate;
        candidate.m_source = source;
        candidate.m_id = id;
        candidate.m_label = label;
        candidate.m_scorePercent = scorePercent;
        candidate.m_endpointResidualRMSHz = endpointResidualRMSHz;
        candidate.m_centerResidualHz = centerResidualHz;
        candidate.m_driftResidualHz = driftResidualHz;
        candidate.m_stateAgeS = stateAgeS;
        candidate.m_prediction = prediction;
        return candidate;
    }
}

MovingTargetMatcher::Match MovingTargetMatcher::match(
    const Observation& observation,
    const QVector<TargetState>& targets,
    const Tunables& tunables)
{
    Match bestMatch;
    QVector<Candidate> rankedCandidates;
    rankedCandidates.reserve(MaximumAlternativeCount + 1);
    const QDateTime centerDateTimeUtc = observation.m_startDateTimeUtc.addMSecs(
        (qint64) std::llround(observation.m_durationS * 500.0));
    const ObservationScales scales = makeObservationScales(observation, tunables);

    for (const TargetState& target : targets)
    {
        const double stateAgeS = std::fabs(secondsBetween(target.m_dateTimeUtc, centerDateTimeUtc));

        if (!std::isfinite(stateAgeS) || (stateAgeS > tunables.m_maximumStateAgeS)) {
            continue;
        }

        const Prediction prediction = predict(observation, target);

        if (!prediction.m_valid) {
            continue;
        }

        insertRankedCandidate(rankedCandidates, scoreCandidate(
            observation,
            scales,
            target.m_source,
            target.m_id,
            target.m_label,
            stateAgeS,
            prediction,
            target.m_approximateDirection ? ApproximateDirectionScoreFactor : 1.0));
    }

    finalizeMatch(bestMatch, rankedCandidates, tunables);
    return bestMatch;
}

MovingTargetMatcher::Match MovingTargetMatcher::matchPredictions(
    const Observation& observation,
    const QVector<PredictedCandidate>& candidates,
    const Tunables& tunables)
{
    Match bestMatch;
    QVector<Candidate> rankedCandidates;
    rankedCandidates.reserve(MaximumAlternativeCount + 1);
    const ObservationScales scales = makeObservationScales(observation, tunables);

    // No state-age gate here: each prediction was evaluated at the observation's own
    // epochs, so there is no extrapolation age to bound.
    for (const PredictedCandidate& candidate : candidates)
    {
        if (!candidate.m_prediction.m_valid) {
            continue;
        }

        insertRankedCandidate(rankedCandidates, scoreCandidate(
            observation,
            scales,
            candidate.m_source,
            candidate.m_id,
            candidate.m_label,
            candidate.m_stateAgeS,
            candidate.m_prediction,
            1.0));
    }

    finalizeMatch(bestMatch, rankedCandidates, tunables);
    return bestMatch;
}

MovingTargetMatcher::Match MovingTargetMatcher::combine(
    const Match& first,
    const Match& second,
    const Tunables& tunables)
{
    if (!first.m_hasCandidate) {
        return second;
    }

    if (!second.m_hasCandidate) {
        return first;
    }

    QVector<Candidate> rankedCandidates;
    rankedCandidates.reserve(2 * (MaximumAlternativeCount + 1));
    insertRankedCandidate(rankedCandidates, candidateFromMatch(first));
    for (const Candidate& candidate : first.m_alternatives) {
        insertRankedCandidate(rankedCandidates, candidate);
    }
    insertRankedCandidate(rankedCandidates, candidateFromMatch(second));
    for (const Candidate& candidate : second.m_alternatives) {
        insertRankedCandidate(rankedCandidates, candidate);
    }

    Match combined;
    finalizeMatch(combined, rankedCandidates, tunables);
    return combined;
}
