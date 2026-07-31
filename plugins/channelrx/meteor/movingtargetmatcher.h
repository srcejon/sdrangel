///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_MOVINGTARGETMATCHER_H
#define INCLUDE_MOVINGTARGETMATCHER_H

#include <QDateTime>
#include <QString>
#include <QVector>

class MovingTargetMatcher
{
public:
    struct Site
    {
        double m_latitudeDegrees = 0.0;
        double m_longitudeDegrees = 0.0;
        double m_altitudeM = 0.0;
    };

    // A geodetic position and local ENU velocity at one UTC epoch. This form is
    // deliberately source-neutral so ADS-B and future TLE adapters can share it.
    struct TargetState
    {
        QString m_source;
        QString m_id;
        QString m_label;
        QDateTime m_dateTimeUtc;
        Site m_position;
        double m_eastVelocityMPS = 0.0;
        double m_northVelocityMPS = 0.0;
        double m_upVelocityMPS = 0.0;
        // The velocity direction is inferred rather than measured over ground (ADS-B
        // heading standing in for track): the score is down-weighted so such a state
        // can still match but loses ties against measured-direction candidates.
        bool m_approximateDirection = false;
    };

    struct Observation
    {
        struct FrequencySample
        {
            double m_timeOffsetS = 0.0;
            double m_frequencyOffsetHz = 0.0;
            double m_uncertaintyHz = 0.0;
        };

        QDateTime m_startDateTimeUtc;
        double m_durationS = 0.0;
        double m_centerFrequencyOffsetHz = 0.0;
        double m_frequencyDriftHz = 0.0;
        double m_frequencySpanHz = 0.0;
        double m_referenceFrequencyHz = 0.0;
        double m_frequencyUncertaintyHz = 25.0;
        double m_timingUncertaintyS = 0.05;
        bool m_applySystemClockCorrection = true;
        QVector<FrequencySample> m_frequencySamples;
        Site m_transmitter;
        Site m_receiver;
    };

    struct Prediction
    {
        bool m_valid = false;
        double m_startFrequencyOffsetHz = 0.0;
        double m_endFrequencyOffsetHz = 0.0;
        double m_centerFrequencyOffsetHz = 0.0;
        double m_frequencyDriftHz = 0.0;
        QVector<double> m_frequencySamplesHz;
    };

    struct Candidate
    {
        QString m_source;
        QString m_id;
        QString m_label;
        double m_scorePercent = 0.0;
        double m_endpointResidualRMSHz = 0.0;
        double m_centerResidualHz = 0.0;
        double m_driftResidualHz = 0.0;
        double m_trajectoryResidualRMSHz = 0.0;
        double m_fittedFrequencyBiasHz = 0.0;
        double m_stateAgeS = 0.0;
        bool m_softGeometry = false;
        Prediction m_prediction;
    };

    struct Match
    {
        bool m_hasCandidate = false;
        bool m_matched = false;
        bool m_ambiguous = false;
        QString m_source;
        QString m_id;
        QString m_label;
        double m_scorePercent = 0.0;
        double m_secondBestScorePercent = 0.0;
        double m_endpointResidualRMSHz = 0.0;
        double m_centerResidualHz = 0.0;
        double m_driftResidualHz = 0.0;
        double m_trajectoryResidualRMSHz = 0.0;
        double m_fittedFrequencyBiasHz = 0.0;
        double m_stateAgeS = 0.0;
        int m_candidateCount = 0;
        int m_closeCandidateCount = 0;
        int m_passFragmentCount = 1;
        double m_appliedClockCorrectionS = 0.0;
        bool m_softGeometry = false;
        QString m_diagnostic;
        Prediction m_prediction;
        QVector<Candidate> m_alternatives;
    };

    struct Tunables
    {
        double m_maximumStateAgeS = 45.0;
        double m_minimumCenterScaleHz = 25.0;
        double m_centerSpanScale = 0.20;
        double m_minimumDriftScaleHz = 12.0;
        double m_driftScale = 0.25;
        double m_defaultFrequencyUncertaintyHz = 25.0;
        // Detector trajectories are reconstructed from correlated fragment centres and
        // slopes rather than independent ridge samples. This floor represents that model
        // discrepancy and prevents FFT-bin precision from making the match overconfident.
        double m_trajectoryModelUncertaintyFloorHz = 25.0;
        double m_reconstructedTrajectoryWeight = 0.20;
        double m_maximumTrajectoryPenalty = 1.0;
        double m_maximumFittedFrequencyBiasHz = 20.0;
        double m_huberTransitionSigma = 1.5;
        double m_minimumMatchScorePercent = 60.0;
        double m_minimumScoreMarginPercent = 8.0;

        // User-declared, defaulted out of line: lets GCC/Clang defer the NSDMI
        // exception-spec evaluation past the `= Tunables()` default argument below
        // (MSVC accepts the in-class form, GCC/Clang reject it).
        Tunables();
    };

    // A candidate whose Doppler prediction was computed externally at the exact
    // observation epochs (e.g. per-endpoint SGP4 propagation), bypassing the
    // constant-velocity extrapolation in predict().
    struct PredictedCandidate
    {
        QString m_source;
        QString m_id;
        QString m_label;
        double m_stateAgeS = 0.0;
        double m_scoreFactor = 1.0;
        bool m_softGeometry = false;
        Prediction m_prediction;
    };

    static double elevationDegrees(
        const Site& observer,
        const TargetState& target,
        const QDateTime& dateTimeUtc);
    static Prediction predict(const Observation& observation, const TargetState& target);
    static Match match(
        const Observation& observation,
        const QVector<TargetState>& targets,
        const Tunables& tunables = Tunables());
    static Match matchPredictions(
        const Observation& observation,
        const QVector<PredictedCandidate>& candidates,
        const Tunables& tunables = Tunables());
    static Match matchPredictionGroups(
        const QVector<Observation>& observations,
        const QVector<QVector<PredictedCandidate>>& candidateGroups,
        const Tunables& tunables = Tunables());
    static Match combine(
        const Match& first,
        const Match& second,
        const Tunables& tunables = Tunables());
};

inline MovingTargetMatcher::Tunables::Tunables() = default;

#endif // INCLUDE_MOVINGTARGETMATCHER_H
