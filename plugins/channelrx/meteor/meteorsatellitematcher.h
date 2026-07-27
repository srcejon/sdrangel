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
        int m_payloadEntries = 0;
        int m_rocketBodyEntries = 0;
        int m_debrisEntries = 0;
        int m_otherEntries = 0;
        int m_satcatOnOrbitEntries = 0;
        QStringList m_sourceWarnings;

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

    explicit MeteorSatelliteMatcher(QObject *parent = nullptr);
    ~MeteorSatelliteMatcher() override;

    static MoonPrediction predictMoon(
        const MovingTargetMatcher::Observation& observation,
        const Geometry& geometry);
    void requestMatch(
        quint64 requestId,
        const MovingTargetMatcher::Observation& observation,
        const Geometry& geometry);
    void requestStatistics();
    void refreshCatalog();

signals:
    void matchReady(
        quint64 requestId,
        const MovingTargetMatcher::Match& match,
        const MoonPrediction& moonPrediction);
    void statisticsReady(const CatalogStatistics& statistics);

private:
    friend class MeteorSatelliteMatcherWorker;

    void deliverMatch(
        quint64 requestId,
        const MovingTargetMatcher::Match& match,
        const MoonPrediction& moonPrediction);
    void deliverStatistics(const CatalogStatistics& statistics);

    QThread *m_thread;
    MeteorSatelliteMatcherWorker *m_worker;
};

#endif // INCLUDE_METEORSATELLITEMATCHER_H
