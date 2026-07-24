///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License.                     //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_METEORSATELLITEMATCHER_H
#define INCLUDE_METEORSATELLITEMATCHER_H

#include <QObject>

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
    };

    explicit MeteorSatelliteMatcher(QObject *parent = nullptr);
    ~MeteorSatelliteMatcher() override;

    void requestMatch(
        quint64 requestId,
        const MovingTargetMatcher::Observation& observation,
        const Geometry& geometry);
    void refreshCatalog();

signals:
    void matchReady(
        quint64 requestId,
        const MovingTargetMatcher::Match& match,
        int catalogSize,
        const QString& status);
    void catalogStatusChanged(int catalogSize, const QString& status);

private:
    friend class MeteorSatelliteMatcherWorker;

    void deliverMatch(
        quint64 requestId,
        const MovingTargetMatcher::Match& match,
        int catalogSize,
        const QString& status);
    void deliverCatalogStatus(int catalogSize, const QString& status);

    QThread *m_thread;
    MeteorSatelliteMatcherWorker *m_worker;
};

#endif // INCLUDE_METEORSATELLITEMATCHER_H
