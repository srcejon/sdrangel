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

MovingTargetMatcher::Match MovingTargetMatcher::match(
    const Observation& observation,
    const QVector<TargetState>& targets,
    const Tunables& tunables)
{
    Match bestMatch;
    const QDateTime centerDateTimeUtc = observation.m_startDateTimeUtc.addMSecs(
        (qint64) std::llround(observation.m_durationS * 500.0));
    const double observedStartFrequencyHz = observation.m_centerFrequencyOffsetHz
        - 0.5 * observation.m_frequencyDriftHz;
    const double observedEndFrequencyHz = observation.m_centerFrequencyOffsetHz
        + 0.5 * observation.m_frequencyDriftHz;
    const double centerScaleHz = std::max(
        tunables.m_minimumCenterScaleHz,
        std::fabs(observation.m_frequencySpanHz) * tunables.m_centerSpanScale);
    const double driftScaleHz = std::max(
        tunables.m_minimumDriftScaleHz,
        std::fabs(observation.m_frequencyDriftHz) * tunables.m_driftScale);

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

        const double centerResidualHz = observation.m_centerFrequencyOffsetHz
            - prediction.m_centerFrequencyOffsetHz;
        const double driftResidualHz = observation.m_frequencyDriftHz
            - prediction.m_frequencyDriftHz;
        const double normalizedSquared = std::pow(centerResidualHz / centerScaleHz, 2.0)
            + std::pow(driftResidualHz / driftScaleHz, 2.0);
        const double scorePercent = 100.0 * std::exp(-0.5 * normalizedSquared);
        const double startResidualHz = observedStartFrequencyHz
            - prediction.m_startFrequencyOffsetHz;
        const double endResidualHz = observedEndFrequencyHz
            - prediction.m_endFrequencyOffsetHz;
        const double endpointResidualRMSHz = std::sqrt(
            0.5 * (startResidualHz * startResidualHz + endResidualHz * endResidualHz));

        if (!bestMatch.m_hasCandidate || (scorePercent > bestMatch.m_scorePercent))
        {
            bestMatch.m_secondBestScorePercent = bestMatch.m_hasCandidate
                ? bestMatch.m_scorePercent
                : 0.0;
            bestMatch.m_hasCandidate = true;
            bestMatch.m_source = target.m_source;
            bestMatch.m_id = target.m_id;
            bestMatch.m_label = target.m_label;
            bestMatch.m_scorePercent = scorePercent;
            bestMatch.m_endpointResidualRMSHz = endpointResidualRMSHz;
            bestMatch.m_centerResidualHz = centerResidualHz;
            bestMatch.m_driftResidualHz = driftResidualHz;
            bestMatch.m_stateAgeS = stateAgeS;
            bestMatch.m_prediction = prediction;
        }
        else if (scorePercent > bestMatch.m_secondBestScorePercent) {
            bestMatch.m_secondBestScorePercent = scorePercent;
        }
    }

    if (bestMatch.m_hasCandidate)
    {
        const double scoreMarginPercent = bestMatch.m_scorePercent
            - bestMatch.m_secondBestScorePercent;
        bestMatch.m_ambiguous = (bestMatch.m_secondBestScorePercent > 0.0)
            && (scoreMarginPercent < tunables.m_minimumScoreMarginPercent);
        bestMatch.m_matched = (bestMatch.m_scorePercent >= tunables.m_minimumMatchScorePercent)
            && !bestMatch.m_ambiguous;
    }

    return bestMatch;
}
