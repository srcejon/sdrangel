///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License.                     //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_METEORSATELLITEMATCHER_H
#define INCLUDE_METEORSATELLITEMATCHER_H

#include <QDateTime>
#include <QObject>
#include <QStringList>
#include <QVector>

#include "movingtargetmatcher.h"

class QThread;
class MeteorSatelliteMatcherWorker;

class MeteorSatelliteMatcher : public QObject
{
    Q_OBJECT

public:
    struct Beam
    {
        double m_azimuthDegrees = 0.0;
        double m_elevationDegrees = 0.0;
        double m_horizontalBeamwidthDegrees = 0.0;
        double m_verticalBeamwidthDegrees = 0.0;
    };

    struct Geometry
    {
        Beam m_transmitterBeam;
        Beam m_receiverBeam;
        double m_maximumAltitudeM = 0.0;
    };

    struct CatalogStatistics
    {
        QString m_status = QStringLiteral("Orbital catalog is loading");
        QDateTime m_loadedDateTimeUtc;
        int m_catalogEntries = 0;
        int m_activeCatalogEntries = 0;
        int m_supplementalCatalogEntries = 0;
        int m_spaceTrackCatalogEntries = 0;
        int m_payloadEntries = 0;
        int m_rocketBodyEntries = 0;
        int m_debrisEntries = 0;
        int m_otherEntries = 0;
        int m_satcatOnOrbitEntries = 0;
        bool m_spaceTrackConfigured = false;
        bool m_spaceTrackCacheAvailable = false;
        QDateTime m_spaceTrackCacheDateTimeUtc;
        QStringList m_sourceWarnings;

        bool m_clockCheckPending = false;
        bool m_clockCheckAvailable = false;
        QDateTime m_clockCheckDateTimeUtc;
        QString m_clockTimeSource;
        QString m_clockCheckError;
        double m_localClockErrorMS = 0.0;  // positive when the local clock is fast
        double m_clockUncertaintyMS = 0.0;
        double m_clockRoundTripMS = 0.0;
        double m_acceptableClockErrorMS = 1000.0;

        bool m_snapshotValid = false;
        QDateTime m_snapshotDateTimeUtc;
        double m_maximumAltitudeKM = 0.0;
        int m_staleElementEntries = 0;
        int m_propagationFailureEntries = 0;
        int m_aboveMaximumAltitudeEntries = 0;
        int m_belowTransmitterHorizonEntries = 0;
        int m_belowReceiverHorizonEntries = 0;
        int m_outsideTransmitterBeamEntries = 0;
        int m_outsideReceiverBeamEntries = 0;
        int m_candidateEntries = 0;
    };

    struct MoonPrediction
    {
        bool m_possible = false;
        double m_transmitterAzimuthDegrees = 0.0;
        double m_transmitterElevationDegrees = 0.0;
        double m_receiverAzimuthDegrees = 0.0;
        double m_receiverElevationDegrees = 0.0;
        MovingTargetMatcher::Match m_match;
    };

    struct TrackPoint
    {
        QDateTime m_dateTimeUtc;
        double m_latitudeDegrees = 0.0;
        double m_longitudeDegrees = 0.0;
        double m_altitudeM = 0.0;
    };

    struct Track
    {
        QString m_source;
        QString m_id;
        QString m_label;
        QString m_objectType;
        MovingTargetMatcher::Prediction m_prediction;
        double m_matchScorePercent = 0.0;
        double m_endpointResidualRMSHz = 0.0;
        QVector<TrackPoint> m_points;

        bool isValid() const
        {
            return (m_source == QStringLiteral("TLE"))
                && !m_id.isEmpty()
                && (m_points.size() >= 2);
        }
    };

    struct CalibrationResult
    {
        struct ObservationComparison
        {
            QDateTime m_dateTimeUtc;
            MovingTargetMatcher::Match m_before;
            MovingTargetMatcher::Match m_after;
        };

        bool m_success = false;
        QString m_error;
        int m_observationCount = 0;
        int m_modelledObservationCount = 0;
        int m_matchedBefore = 0;
        int m_matchedAfter = 0;
        int m_ambiguousBefore = 0;
        int m_ambiguousAfter = 0;
        double m_meanScoreBefore = 0.0;
        double m_meanScoreAfter = 0.0;
        double m_meanResidualBeforeHz = 0.0;
        double m_meanResidualAfterHz = 0.0;
        // Add this correction to the observation timestamps used for matching.
        double m_timeOffsetS = 0.0;
        // Observed frequency minus predicted frequency; subtract this from observations.
        double m_frequencyBiasHz = 0.0;
        double m_timeUncertaintyS = 0.0;
        double m_frequencyUncertaintyHz = 0.0;
        bool m_timeAtSearchLimit = false;
        bool m_frequencyAtSearchLimit = false;
        bool m_usedExtendedSearch = false;
        bool m_extendedSearchRejected = false;
        double m_extendedTimeOffsetS = 0.0;
        double m_extendedFrequencyBiasHz = 0.0;
        bool m_recommendationReliable = false;
        QVector<ObservationComparison> m_observationComparisons;
    };

    explicit MeteorSatelliteMatcher(
        const QString& spaceTrackUsername = QString(),
        const QString& spaceTrackPassword = QString(),
        QObject *parent = nullptr);
    ~MeteorSatelliteMatcher() override;

    static bool beamContainsLookDirection(
        double azimuthDegrees,
        double elevationDegrees,
        const Beam& beam,
        double marginDegrees = 0.0);
    static MoonPrediction predictMoon(
        const MovingTargetMatcher::Observation& observation,
        const Geometry& geometry);
    void requestMatch(
        quint64 requestId,
        const MovingTargetMatcher::Observation& observation,
        const Geometry& geometry);
    void requestCalibration(
        const QVector<MovingTargetMatcher::Observation>& observations,
        const Geometry& geometry);
    void requestStatistics();
    void refreshCatalog();
    void setSpaceTrackCredentials(
        const QString& username,
        const QString& password);

signals:
    void matchReady(
        quint64 requestId,
        const MovingTargetMatcher::Match& match,
        const MoonPrediction& moonPrediction,
        const Track& track);
    void calibrationReady(const CalibrationResult& result);
    void statisticsReady(const CatalogStatistics& statistics);

private:
    friend class MeteorSatelliteMatcherWorker;

    void deliverMatch(
        quint64 requestId,
        const MovingTargetMatcher::Match& match,
        const MoonPrediction& moonPrediction,
        const Track& track);
    void deliverCalibration(const CalibrationResult& result);
    void deliverStatistics(const CatalogStatistics& statistics);

    QThread *m_thread;
    MeteorSatelliteMatcherWorker *m_worker;
};

#endif // INCLUDE_METEORSATELLITEMATCHER_H
