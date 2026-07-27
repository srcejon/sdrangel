///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License.                     //
///////////////////////////////////////////////////////////////////////////////////

#include "meteorsatellitematcher.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <vector>

#include <QBuffer>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QMetaObject>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPointer>
#include <QSaveFile>
#include <QStandardPaths>
#include <QTextStream>
#include <QThread>
#include <QTimer>
#include <QUrl>

#include "util/csv.h"
#include "util/astronomy.h"

#ifdef METEOR_HAS_SGP4
#include <CoordGeodetic.h>
#include <DateTime.h>
#include <DecayedException.h>
#include <Eci.h>
#include <SatelliteException.h>
#include <SGP4.h>
#include <Tle.h>
#include <TleException.h>
#include <Util.h>
#endif

namespace {
    constexpr double Pi = 3.14159265358979323846;
    constexpr double WGS84SemiMajorAxisM = 6378137.0;
    constexpr double WGS84EccentricitySquared = 6.69437999014e-3;
    constexpr qint64 CatalogMaximumAgeS = 24 * 60 * 60;
    constexpr qint64 TLEMaximumAgeS = 14 * 24 * 60 * 60;
    constexpr qint64 SnapshotIntervalMS = 5000;
    constexpr int MaximumSnapshotCount = 8;
    constexpr double MinimumElevationDegrees = -1.0;

    enum class CatalogFileKind
    {
        Satcat,
        ActiveGP,
        SupplementalGP
    };

    struct CatalogDownloadSource
    {
        const char *m_url;
        const char *m_cacheName;
        CatalogFileKind m_kind;
        int m_minimumEntries;
    };

    const CatalogDownloadSource CatalogDownloadSources[] {
        {
            "https://celestrak.org/pub/satcat.csv",
            "celestrak-satcat.csv",
            CatalogFileKind::Satcat,
            10000
        },
        {
            "https://celestrak.org/NORAD/elements/gp.php?GROUP=active&FORMAT=CSV",
            "celestrak-active.csv",
            CatalogFileKind::ActiveGP,
            1000
        },
        {
            "https://celestrak.org/NORAD/elements/gp.php?GROUP=visual&FORMAT=CSV",
            "celestrak-visual.csv",
            CatalogFileKind::SupplementalGP,
            50
        },
        {
            "https://celestrak.org/NORAD/elements/gp.php?GROUP=analyst&FORMAT=CSV",
            "celestrak-analyst.csv",
            CatalogFileKind::SupplementalGP,
            100
        },
        {
            "https://celestrak.org/NORAD/elements/gp.php?GROUP=last-30-days&FORMAT=CSV",
            "celestrak-last-30-days.csv",
            CatalogFileKind::SupplementalGP,
            10
        },
        {
            "https://celestrak.org/NORAD/elements/gp.php?SPECIAL=GPZ-PLUS&FORMAT=CSV",
            "celestrak-gpz-plus.csv",
            CatalogFileKind::SupplementalGP,
            100
        },
        {
            "https://celestrak.org/NORAD/elements/gp.php?SPECIAL=DECAYING&FORMAT=CSV",
            "celestrak-decaying.csv",
            CatalogFileKind::SupplementalGP,
            1
        },
        {
            "https://celestrak.org/NORAD/elements/gp.php?GROUP=fengyun-1c-debris&FORMAT=CSV",
            "celestrak-fengyun-1c-debris.csv",
            CatalogFileKind::SupplementalGP,
            100
        },
        {
            "https://celestrak.org/NORAD/elements/gp.php?GROUP=iridium-33-debris&FORMAT=CSV",
            "celestrak-iridium-33-debris.csv",
            CatalogFileKind::SupplementalGP,
            50
        },
        {
            "https://celestrak.org/NORAD/elements/gp.php?GROUP=cosmos-2251-debris&FORMAT=CSV",
            "celestrak-cosmos-2251-debris.csv",
            CatalogFileKind::SupplementalGP,
            100
        }
    };
    constexpr int CatalogDownloadSourceCount =
        sizeof(CatalogDownloadSources) / sizeof(CatalogDownloadSources[0]);

    struct Vector3
    {
        double m_x = 0.0;
        double m_y = 0.0;
        double m_z = 0.0;

        Vector3 operator-(const Vector3& other) const
        {
            return {m_x - other.m_x, m_y - other.m_y, m_z - other.m_z};
        }

        Vector3 operator*(double scalar) const
        {
            return {m_x * scalar, m_y * scalar, m_z * scalar};
        }
    };

    void ecefToENU(
        const Vector3& vector,
        const MovingTargetMatcher::Site& origin,
        double& east,
        double& north,
        double& up);

    bool ecefToGeodetic(const Vector3& position, MovingTargetMatcher::Site& site)
    {
        const double longitude = std::atan2(position.m_y, position.m_x);
        const double horizontalRadius = std::hypot(position.m_x, position.m_y);

        if (!(horizontalRadius > 1.0) || !std::isfinite(position.m_z)) {
            return false;
        }

        double latitude = std::atan2(
            position.m_z,
            horizontalRadius * (1.0 - WGS84EccentricitySquared));
        double altitudeM = 0.0;

        for (int iteration = 0; iteration < 6; ++iteration)
        {
            const double sinLatitude = std::sin(latitude);
            const double primeVerticalRadius = WGS84SemiMajorAxisM
                / std::sqrt(1.0 - WGS84EccentricitySquared
                    * sinLatitude * sinLatitude);
            altitudeM = horizontalRadius / std::cos(latitude)
                - primeVerticalRadius;
            latitude = std::atan2(
                position.m_z,
                horizontalRadius * (1.0
                    - WGS84EccentricitySquared * primeVerticalRadius
                        / (primeVerticalRadius + altitudeM)));
        }

        site = {
            latitude * 180.0 / Pi,
            longitude * 180.0 / Pi,
            altitudeM
        };
        return std::isfinite(site.m_latitudeDegrees)
            && std::isfinite(site.m_longitudeDegrees)
            && std::isfinite(site.m_altitudeM);
    }

    bool moonECEFAtTime(const QDateTime& dateTimeUtc, Vector3& position)
    {
        if (!dateTimeUtc.isValid()) {
            return false;
        }

        AzAlt topocentric;
        RADec topocentricRD;
        RADec geocentricRD;
        double distanceM = 0.0;
        Astronomy::moonPosition(
            topocentric,
            topocentricRD,
            0.0,
            0.0,
            dateTimeUtc,
            geocentricRD,
            distanceM);

        if (!(distanceM > WGS84SemiMajorAxisM)
            || !std::isfinite(geocentricRD.ra)
            || !std::isfinite(geocentricRD.dec))
        {
            return false;
        }

        const double latitude = geocentricRD.dec * Pi / 180.0;
        const double earthFixedLongitude = std::remainder(
            geocentricRD.ra * 15.0
                - Astronomy::localSiderealTime(dateTimeUtc, 0.0),
            360.0) * Pi / 180.0;
        const double cosLatitude = std::cos(latitude);
        position = {
            distanceM * cosLatitude * std::cos(earthFixedLongitude),
            distanceM * cosLatitude * std::sin(earthFixedLongitude),
            distanceM * std::sin(latitude)
        };
        return true;
    }

    bool moonTargetState(
        const QDateTime& dateTimeUtc,
        MovingTargetMatcher::TargetState& state,
        Vector3& targetPosition)
    {
        constexpr double VelocityDeltaS = 5.0;
        Vector3 beforePosition;
        Vector3 afterPosition;

        if (!moonECEFAtTime(dateTimeUtc, targetPosition)
            || !moonECEFAtTime(
                dateTimeUtc.addMSecs((qint64) std::llround(-VelocityDeltaS * 1000.0)),
                beforePosition)
            || !moonECEFAtTime(
                dateTimeUtc.addMSecs((qint64) std::llround(VelocityDeltaS * 1000.0)),
                afterPosition)
            || !ecefToGeodetic(targetPosition, state.m_position))
        {
            return false;
        }

        state.m_source = QStringLiteral("Moon");
        state.m_id = QStringLiteral("Moon");
        state.m_label = QStringLiteral("Moon");
        state.m_dateTimeUtc = dateTimeUtc;
        const Vector3 velocityECEF = (afterPosition - beforePosition)
            * (1.0 / (2.0 * VelocityDeltaS));
        ecefToENU(
            velocityECEF,
            state.m_position,
            state.m_eastVelocityMPS,
            state.m_northVelocityMPS,
            state.m_upVelocityMPS);
        return true;
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
            (primeVerticalRadius * (1.0 - WGS84EccentricitySquared) + site.m_altitudeM)
                * sinLatitude
        };
    }

    void ecefToENU(
        const Vector3& vector,
        const MovingTargetMatcher::Site& origin,
        double& east,
        double& north,
        double& up)
    {
        const double latitude = origin.m_latitudeDegrees * Pi / 180.0;
        const double longitude = origin.m_longitudeDegrees * Pi / 180.0;
        const double sinLatitude = std::sin(latitude);
        const double cosLatitude = std::cos(latitude);
        const double sinLongitude = std::sin(longitude);
        const double cosLongitude = std::cos(longitude);

        east = -sinLongitude * vector.m_x + cosLongitude * vector.m_y;
        north = -sinLatitude * cosLongitude * vector.m_x
            - sinLatitude * sinLongitude * vector.m_y
            + cosLatitude * vector.m_z;
        up = cosLatitude * cosLongitude * vector.m_x
            + cosLatitude * sinLongitude * vector.m_y
            + sinLatitude * vector.m_z;
    }

    double wrappedAngleDifference(double leftDegrees, double rightDegrees)
    {
        return std::fabs(std::remainder(leftDegrees - rightDegrees, 360.0));
    }

    bool siteLookAngles(
        const MovingTargetMatcher::Site& site,
        const Vector3& targetPosition,
        double& azimuthDegrees,
        double& elevationDegrees)
    {
        const Vector3 relative = targetPosition - geodeticToECEF(site);
        double east;
        double north;
        double up;
        ecefToENU(relative, site, east, north, up);
        const double range = std::sqrt(east * east + north * north + up * up);

        if (!(range > 1.0)) {
            return false;
        }

        azimuthDegrees = std::atan2(east, north) * 180.0 / Pi;
        if (azimuthDegrees < 0.0) {
            azimuthDegrees += 360.0;
        }
        elevationDegrees = std::asin(std::clamp(up / range, -1.0, 1.0)) * 180.0 / Pi;
        return true;
    }

    bool insideBeam(
        double azimuthDegrees,
        double elevationDegrees,
        const MeteorSatelliteMatcher::Beam& beam)
    {
        const bool horizontalOK = !(beam.m_horizontalBeamwidthDegrees > 0.0)
            || (wrappedAngleDifference(azimuthDegrees, beam.m_azimuthDegrees)
                <= beam.m_horizontalBeamwidthDegrees * 0.5);
        const bool verticalOK = !(beam.m_verticalBeamwidthDegrees > 0.0)
            || (std::fabs(elevationDegrees - beam.m_elevationDegrees)
                <= beam.m_verticalBeamwidthDegrees * 0.5);
        return horizontalOK && verticalOK;
    }

    QString geometryKey(
        const MovingTargetMatcher::Observation& observation,
        const MeteorSatelliteMatcher::Geometry& geometry)
    {
        const MeteorSatelliteMatcher::Beam& tx = geometry.m_transmitterBeam;
        const MeteorSatelliteMatcher::Beam& rx = geometry.m_receiverBeam;
        return QStringLiteral(
            "%1,%2,%3;%4,%5,%6;%7,%8,%9,%10;%11,%12,%13,%14;%15")
            .arg(observation.m_transmitter.m_latitudeDegrees, 0, 'f', 6)
            .arg(observation.m_transmitter.m_longitudeDegrees, 0, 'f', 6)
            .arg(observation.m_transmitter.m_altitudeM, 0, 'f', 1)
            .arg(observation.m_receiver.m_latitudeDegrees, 0, 'f', 6)
            .arg(observation.m_receiver.m_longitudeDegrees, 0, 'f', 6)
            .arg(observation.m_receiver.m_altitudeM, 0, 'f', 1)
            .arg(tx.m_azimuthDegrees, 0, 'f', 2)
            .arg(tx.m_elevationDegrees, 0, 'f', 2)
            .arg(tx.m_horizontalBeamwidthDegrees, 0, 'f', 2)
            .arg(tx.m_verticalBeamwidthDegrees, 0, 'f', 2)
            .arg(rx.m_azimuthDegrees, 0, 'f', 2)
            .arg(rx.m_elevationDegrees, 0, 'f', 2)
            .arg(rx.m_horizontalBeamwidthDegrees, 0, 'f', 2)
            .arg(rx.m_verticalBeamwidthDegrees, 0, 'f', 2)
            .arg(geometry.m_maximumAltitudeM, 0, 'f', 1);
    }
}

MeteorSatelliteMatcher::MoonPrediction MeteorSatelliteMatcher::predictMoon(
    const MovingTargetMatcher::Observation& observation,
    const Geometry& geometry)
{
    MoonPrediction result;

    if (!observation.m_startDateTimeUtc.isValid()
        || !(observation.m_durationS > 0.0)
        || !(observation.m_referenceFrequencyHz > 0.0))
    {
        return result;
    }

    const QDateTime centerTimeUtc = observation.m_startDateTimeUtc.addMSecs(
        (qint64) std::llround(observation.m_durationS * 500.0));
    MovingTargetMatcher::TargetState moonState;
    Vector3 moonPosition;

    if (!moonTargetState(centerTimeUtc, moonState, moonPosition)
        || !siteLookAngles(
            observation.m_transmitter,
            moonPosition,
            result.m_transmitterAzimuthDegrees,
            result.m_transmitterElevationDegrees)
        || !siteLookAngles(
            observation.m_receiver,
            moonPosition,
            result.m_receiverAzimuthDegrees,
            result.m_receiverElevationDegrees))
    {
        return result;
    }

    result.m_possible =
        (result.m_transmitterElevationDegrees >= MinimumElevationDegrees)
        && (result.m_receiverElevationDegrees >= MinimumElevationDegrees)
        && insideBeam(
            result.m_transmitterAzimuthDegrees,
            result.m_transmitterElevationDegrees,
            geometry.m_transmitterBeam)
        && insideBeam(
            result.m_receiverAzimuthDegrees,
            result.m_receiverElevationDegrees,
            geometry.m_receiverBeam);

    if (result.m_possible) {
        result.m_match = MovingTargetMatcher::match(observation, {moonState});
    }

    return result;
}

class MeteorSatelliteMatcherWorker : public QObject
{
public:
    explicit MeteorSatelliteMatcherWorker(MeteorSatelliteMatcher *owner) :
        m_owner(owner)
    {}

    void start()
    {
#ifdef METEOR_HAS_SGP4
        m_networkManager = new QNetworkAccessManager(this);
        m_refreshTimer = new QTimer(this);
        m_refreshTimer->setInterval(6 * 60 * 60 * 1000);
        connect(m_refreshTimer, &QTimer::timeout, this, [this]() {
            refreshCatalogIfNeeded();
        });
        m_refreshTimer->start();

        const QString directoryPath = QStandardPaths::writableLocation(
            QStandardPaths::AppDataLocation) + QStringLiteral("/meteor");
        QDir directory;
        directory.mkpath(directoryPath);
        m_catalogDirectory = directoryPath;
        loadCatalogFiles();
        refreshCatalogIfNeeded();
#else
        m_status = QStringLiteral("Satellite matching unavailable: SGP4 was not found");
        deliverStatistics();
#endif
    }

    void refreshCatalog()
    {
#ifdef METEOR_HAS_SGP4
        if (m_downloadInProgress || !m_networkManager) {
            return;
        }

        m_downloadInProgress = true;
        m_downloadSourceIndex = 0;
        m_refreshWarnings.clear();
        downloadNextCatalogSource();
#endif
    }

    void requestMatch(
        quint64 requestId,
        const MovingTargetMatcher::Observation& observation,
        const MeteorSatelliteMatcher::Geometry& geometry)
    {
        MovingTargetMatcher::Match match;
        const MeteorSatelliteMatcher::MoonPrediction moonPrediction =
            MeteorSatelliteMatcher::predictMoon(observation, geometry);
        match = moonPrediction.m_match;

#ifdef METEOR_HAS_SGP4
        if (m_catalog.empty())
        {
            if (m_downloadInProgress)
            {
                m_pendingRequests.push_back({requestId, observation, geometry});
                return;
            }
        }

        if (!m_catalog.empty() && observation.m_startDateTimeUtc.isValid())
        {
            const QDateTime centerTimeUtc = observation.m_startDateTimeUtc.addMSecs(
                (qint64) std::llround(observation.m_durationS * 500.0));
            const QVector<MovingTargetMatcher::TargetState>& states =
                statesForTime(centerTimeUtc, observation, geometry);
            match = MovingTargetMatcher::combine(
                match,
                MovingTargetMatcher::match(observation, states));
        }
#endif

        QPointer<MeteorSatelliteMatcher> owner(m_owner);
        QMetaObject::invokeMethod(
            m_owner,
            [owner, requestId, match, moonPrediction]() {
                if (owner) {
                    owner->deliverMatch(
                        requestId,
                        match,
                        moonPrediction);
                }
            },
            Qt::QueuedConnection);
    }

    void requestStatistics()
    {
        deliverStatistics();
    }

private:
#ifdef METEOR_HAS_SGP4
    struct CatalogMetadata
    {
        QString m_name;
        QString m_objectType;
        bool m_onOrbit = true;
    };

    struct CatalogEntry
    {
        QString m_name;
        QString m_noradId;
        QString m_objectType;
        QDateTime m_epochUtc;
        bool m_fromActiveCatalog = false;
        std::unique_ptr<libsgp4::SGP4> m_propagator;
    };

    struct PendingRequest
    {
        quint64 m_requestId;
        MovingTargetMatcher::Observation m_observation;
        MeteorSatelliteMatcher::Geometry m_geometry;
    };

    struct Snapshot
    {
        QVector<MovingTargetMatcher::TargetState> m_states;
        MeteorSatelliteMatcher::CatalogStatistics m_statistics;
    };

    QString catalogFileName(const CatalogDownloadSource& source) const
    {
        return m_catalogDirectory + QChar('/') + QString::fromLatin1(source.m_cacheName);
    }

    void downloadNextCatalogSource()
    {
        if (m_downloadSourceIndex >= CatalogDownloadSourceCount)
        {
            m_downloadInProgress = false;

            if (!loadCatalogFiles())
            {
                m_status = m_catalog.empty()
                    ? QStringLiteral("No usable satellite orbital elements are available")
                    : QStringLiteral("Satellite catalog refresh failed; using cached elements");
                if (!m_refreshWarnings.isEmpty()) {
                    m_status += QStringLiteral(" (%1)").arg(m_refreshWarnings.join(QStringLiteral("; ")));
                }
                deliverStatistics();
                if (m_catalog.empty()) {
                    completePendingRequestsWithoutMatch();
                }
            }
            return;
        }

        const CatalogDownloadSource source =
            CatalogDownloadSources[m_downloadSourceIndex];
        QNetworkRequest request(QUrl(QString::fromLatin1(source.m_url)));
        request.setHeader(
            QNetworkRequest::UserAgentHeader,
            QStringLiteral("SDRangel Meteor satellite matcher"));
        request.setAttribute(
            QNetworkRequest::RedirectPolicyAttribute,
            QNetworkRequest::NoLessSafeRedirectPolicy);
        // A stalled connection must not wedge the refresh until restart.
        request.setTransferTimeout(2 * 60 * 1000);
        QNetworkReply *reply = m_networkManager->get(request);
        connect(reply, &QNetworkReply::finished, this, [this, reply, source]() {
            const QByteArray data = reply->readAll();
            QString error = reply->error() == QNetworkReply::NoError
                ? QString()
                : reply->errorString();
            reply->deleteLater();

            if (error.isEmpty())
            {
                if (source.m_kind == CatalogFileKind::Satcat)
                {
                    QHash<QString, CatalogMetadata> metadata;
                    int onOrbitCount = 0;
                    if (!parseSatcat(data, metadata, onOrbitCount, error)) {
                        error.prepend(QStringLiteral("invalid SATCAT: "));
                    }
                }
                else
                {
                    std::vector<CatalogEntry> catalog;
                    const QHash<QString, CatalogMetadata> metadata;
                    if (!parseCatalog(
                            data,
                            catalog,
                            metadata,
                            source.m_kind == CatalogFileKind::ActiveGP,
                            source.m_minimumEntries,
                            error))
                    {
                        error.prepend(QStringLiteral("invalid GP data: "));
                    }
                }
            }

            if (error.isEmpty())
            {
                QSaveFile file(catalogFileName(source));
                if (!file.open(QIODevice::WriteOnly)
                    || (file.write(data) != data.size())
                    || !file.commit())
                {
                    error = QStringLiteral("could not cache %1")
                        .arg(QString::fromLatin1(source.m_cacheName));
                }
            }

            if (!error.isEmpty())
            {
                m_refreshWarnings.append(
                    QStringLiteral("%1: %2")
                        .arg(QString::fromLatin1(source.m_cacheName), error));
            }

            ++m_downloadSourceIndex;
            downloadNextCatalogSource();
        });
    }

    static libsgp4::DateTime toSGP4DateTime(const QDateTime& dateTime)
    {
        const QDateTime utc = dateTime.toUTC();
        return libsgp4::DateTime(
            utc.date().year(),
            utc.date().month(),
            utc.date().day(),
            utc.time().hour(),
            utc.time().minute(),
            utc.time().second(),
            utc.time().msec() * 1000);
    }

    static QString tleExponential(double value)
    {
        if (!std::isfinite(value) || (value == 0.0)) {
            return QStringLiteral(" 00000-0");
        }

        const QChar sign = value < 0.0 ? QChar('-') : QChar(' ');
        double magnitude = std::fabs(value);
        int exponent = (int) std::floor(std::log10(magnitude)) + 1;
        int mantissa = (int) std::llround(magnitude * std::pow(10.0, 5 - exponent));

        if (mantissa >= 100000)
        {
            mantissa /= 10;
            ++exponent;
        }

        exponent = std::clamp(exponent, -9, 9);
        return QStringLiteral("%1%2%3%4")
            .arg(sign)
            .arg(mantissa, 5, 10, QChar('0'))
            .arg(exponent < 0 ? QChar('-') : QChar('+'))
            .arg(std::abs(exponent));
    }

    static bool makeCompatibilityTLE(
        int syntheticId,
        const QDateTime& epochUtc,
        double meanMotion,
        double eccentricity,
        double inclination,
        double ascendingNode,
        double argumentPericenter,
        double meanAnomaly,
        double bstar,
        int revolution,
        QString& line1,
        QString& line2)
    {
        // libsgp4 accepts fixed-width TLE input only. Use a synthetic five-digit
        // parser ID while retaining the full OMM NORAD ID in CatalogEntry.
        if (!epochUtc.isValid()
            || !(meanMotion > 0.0)
            || !std::isfinite(meanMotion)
            || !std::isfinite(eccentricity)
            || (eccentricity < 0.0)
            || (eccentricity >= 1.0)
            || !std::isfinite(inclination)
            || (inclination < 0.0)
            || (inclination > 180.0)
            || !std::isfinite(ascendingNode)
            || !std::isfinite(argumentPericenter)
            || !std::isfinite(meanAnomaly)
            || !std::isfinite(bstar))
        {
            return false;
        }

        const QDateTime utc = epochUtc.toUTC();
        const double epochDay = utc.date().dayOfYear()
            + (double) QTime(0, 0).msecsTo(utc.time()) / 86400000.0;
        const QString satelliteNumber = QStringLiteral("%1")
            .arg((syntheticId % 99999) + 1, 5, 10, QChar('0'));
        line1 = QString(69, QChar(' '));
        line1.replace(0, 1, QStringLiteral("1"));
        line1.replace(2, 5, satelliteNumber);
        line1.replace(7, 1, QStringLiteral("U"));
        line1.replace(
            18,
            2,
            QStringLiteral("%1").arg(utc.date().year() % 100, 2, 10, QChar('0')));
        line1.replace(
            20,
            12,
            QStringLiteral("%1").arg(epochDay, 12, 'f', 8, QChar('0')));
        line1.replace(33, 10, QStringLiteral(" .00000000"));
        line1.replace(44, 8, QStringLiteral(" 00000-0"));
        line1.replace(53, 8, tleExponential(bstar));
        line1.replace(62, 1, QStringLiteral("0"));
        line1.replace(64, 4, QStringLiteral("   0"));
        line1.replace(68, 1, QStringLiteral("0"));

        const int eccentricityDigits = std::clamp(
            (int) std::llround(eccentricity * 1.0e7),
            0,
            9999999);
        line2 = QString(69, QChar(' '));
        line2.replace(0, 1, QStringLiteral("2"));
        line2.replace(2, 5, satelliteNumber);
        line2.replace(
            8,
            8,
            QStringLiteral("%1").arg(inclination, 8, 'f', 4, QChar(' ')));
        line2.replace(
            17,
            8,
            QStringLiteral("%1").arg(ascendingNode, 8, 'f', 4, QChar(' ')));
        line2.replace(
            26,
            7,
            QStringLiteral("%1").arg(eccentricityDigits, 7, 10, QChar('0')));
        line2.replace(
            34,
            8,
            QStringLiteral("%1").arg(argumentPericenter, 8, 'f', 4, QChar(' ')));
        line2.replace(
            43,
            8,
            QStringLiteral("%1").arg(meanAnomaly, 8, 'f', 4, QChar(' ')));
        line2.replace(
            52,
            11,
            QStringLiteral("%1").arg(meanMotion, 11, 'f', 8, QChar(' ')));
        line2.replace(
            63,
            5,
            QStringLiteral("%1").arg(std::abs(revolution) % 100000, 5, 10, QChar('0')));
        line2.replace(68, 1, QStringLiteral("0"));
        return true;
    }

    static bool parseSatcat(
        const QByteArray& data,
        QHash<QString, CatalogMetadata>& metadata,
        int& onOrbitCount,
        QString& error)
    {
        QBuffer buffer;
        buffer.setData(data);
        if (!buffer.open(QIODevice::ReadOnly))
        {
            error = QStringLiteral("catalog buffer could not be opened");
            return false;
        }

        QTextStream stream(&buffer);
        const QStringList requiredColumns {
            QStringLiteral("OBJECT_NAME"),
            QStringLiteral("NORAD_CAT_ID"),
            QStringLiteral("OBJECT_TYPE"),
            QStringLiteral("DECAY_DATE")
        };
        const QHash<QString, int> columns = CSV::readHeader(
            stream,
            requiredColumns,
            error);
        if (!error.isEmpty()) {
            return false;
        }

        metadata.clear();
        metadata.reserve(70000);
        onOrbitCount = 0;
        QStringList row;

        while (CSV::readRow(stream, &row))
        {
            auto field = [&columns, &row](const QString& name) {
                const int index = columns.value(name, -1);
                return (index >= 0) && (index < row.size())
                    ? row[index].trimmed()
                    : QString();
            };
            const QString noradId = field(QStringLiteral("NORAD_CAT_ID"));
            if (noradId.isEmpty()) {
                continue;
            }

            CatalogMetadata entry;
            entry.m_name = field(QStringLiteral("OBJECT_NAME"));
            entry.m_objectType = field(QStringLiteral("OBJECT_TYPE"));
            entry.m_onOrbit = field(QStringLiteral("DECAY_DATE")).isEmpty();
            if (entry.m_onOrbit) {
                ++onOrbitCount;
            }
            metadata.insert(noradId, entry);
        }

        if (metadata.size() < 10000)
        {
            error = QStringLiteral("only %1 SATCAT entries were found").arg(metadata.size());
            return false;
        }

        return true;
    }

    static bool parseCatalog(
        const QByteArray& data,
        std::vector<CatalogEntry>& catalog,
        const QHash<QString, CatalogMetadata>& metadata,
        bool fromActiveCatalog,
        int minimumEntries,
        QString& error)
    {
        QBuffer buffer;
        buffer.setData(data);
        if (!buffer.open(QIODevice::ReadOnly))
        {
            error = QStringLiteral("catalog buffer could not be opened");
            return false;
        }

        QTextStream stream(&buffer);
        const QStringList requiredColumns {
            QStringLiteral("OBJECT_NAME"),
            QStringLiteral("EPOCH"),
            QStringLiteral("MEAN_MOTION"),
            QStringLiteral("ECCENTRICITY"),
            QStringLiteral("INCLINATION"),
            QStringLiteral("RA_OF_ASC_NODE"),
            QStringLiteral("ARG_OF_PERICENTER"),
            QStringLiteral("MEAN_ANOMALY"),
            QStringLiteral("NORAD_CAT_ID"),
            QStringLiteral("REV_AT_EPOCH"),
            QStringLiteral("BSTAR")
        };
        const QHash<QString, int> columns = CSV::readHeader(
            stream,
            requiredColumns,
            error);
        if (!error.isEmpty()) {
            return false;
        }

        catalog.reserve(12000);
        QStringList row;
        int syntheticId = 0;

        while (CSV::readRow(stream, &row))
        {
            auto field = [&columns, &row](const QString& name) {
                const int index = columns.value(name, -1);
                return (index >= 0) && (index < row.size())
                    ? row[index].trimmed()
                    : QString();
            };
            bool meanMotionOK;
            bool eccentricityOK;
            bool inclinationOK;
            bool ascendingNodeOK;
            bool argumentPericenterOK;
            bool meanAnomalyOK;
            bool bstarOK;
            bool revolutionOK;
            const double meanMotion = field(QStringLiteral("MEAN_MOTION")).toDouble(&meanMotionOK);
            const double eccentricity = field(QStringLiteral("ECCENTRICITY")).toDouble(&eccentricityOK);
            const double inclination = field(QStringLiteral("INCLINATION")).toDouble(&inclinationOK);
            const double ascendingNode = field(QStringLiteral("RA_OF_ASC_NODE")).toDouble(&ascendingNodeOK);
            const double argumentPericenter = field(QStringLiteral("ARG_OF_PERICENTER")).toDouble(&argumentPericenterOK);
            const double meanAnomaly = field(QStringLiteral("MEAN_ANOMALY")).toDouble(&meanAnomalyOK);
            const double bstar = field(QStringLiteral("BSTAR")).toDouble(&bstarOK);
            const int revolution = field(QStringLiteral("REV_AT_EPOCH")).toInt(&revolutionOK);
            QDateTime epochUtc = QDateTime::fromString(
                field(QStringLiteral("EPOCH")),
                Qt::ISODateWithMs);
            if (!epochUtc.isValid()) {
                epochUtc = QDateTime::fromString(
                    field(QStringLiteral("EPOCH")),
                    Qt::ISODate);
            }
            if (epochUtc.timeSpec() == Qt::LocalTime) {
                epochUtc.setTimeSpec(Qt::UTC);
            } else {
                epochUtc = epochUtc.toUTC();
            }

            const QString noradId = field(QStringLiteral("NORAD_CAT_ID"));
            const auto metadataEntry = metadata.constFind(noradId);
            if (noradId.isEmpty()
                || ((metadataEntry != metadata.cend()) && !metadataEntry->m_onOrbit))
            {
                continue;
            }

            QString name = field(QStringLiteral("OBJECT_NAME"));
            QString objectType;
            if (metadataEntry != metadata.cend())
            {
                if (!metadataEntry->m_name.isEmpty()) {
                    name = metadataEntry->m_name;
                }
                objectType = metadataEntry->m_objectType;
            }
            if (!objectType.isEmpty()
                && (objectType != QStringLiteral("PAY")))
            {
                name += QStringLiteral(" [%1]").arg(objectType);
            }

            QString line1;
            QString line2;
            if (!meanMotionOK
                || !eccentricityOK
                || !inclinationOK
                || !ascendingNodeOK
                || !argumentPericenterOK
                || !meanAnomalyOK
                || !bstarOK
                || !revolutionOK
                || !makeCompatibilityTLE(
                    syntheticId++,
                    epochUtc,
                    meanMotion,
                    eccentricity,
                    inclination,
                    ascendingNode,
                    argumentPericenter,
                    meanAnomaly,
                    bstar,
                    revolution,
                    line1,
                    line2))
            {
                continue;
            }

            try
            {
                std::unique_ptr<libsgp4::Tle> tle(new libsgp4::Tle(
                    name.toStdString(),
                    line1.toStdString(),
                    line2.toStdString()));
                CatalogEntry entry;
                entry.m_noradId = noradId;
                entry.m_name = name.isEmpty()
                    ? QStringLiteral("NORAD %1").arg(noradId)
                    : name;
                entry.m_objectType = objectType;
                entry.m_epochUtc = epochUtc;
                entry.m_fromActiveCatalog = fromActiveCatalog;
                entry.m_propagator.reset(new libsgp4::SGP4(*tle));
                catalog.push_back(std::move(entry));
            }
            catch (const std::exception&) {
                // A bad element must not prevent the rest of the public catalog loading.
            }
        }

        if ((int) catalog.size() < minimumEntries)
        {
            error = QStringLiteral("only %1 valid elements were found").arg(catalog.size());
            return false;
        }

        return true;
    }

    static void mergeCatalog(
        std::vector<CatalogEntry>& catalog,
        std::vector<CatalogEntry>&& additions,
        QHash<QString, int>& indices)
    {
        for (CatalogEntry& addition : additions)
        {
            const auto existingIndex = indices.constFind(addition.m_noradId);
            if (existingIndex == indices.cend())
            {
                indices.insert(addition.m_noradId, (int) catalog.size());
                catalog.push_back(std::move(addition));
                continue;
            }

            CatalogEntry& existing = catalog[*existingIndex];
            const bool fromActiveCatalog =
                existing.m_fromActiveCatalog || addition.m_fromActiveCatalog;
            if (addition.m_epochUtc > existing.m_epochUtc)
            {
                if (addition.m_objectType.isEmpty()) {
                    addition.m_objectType = existing.m_objectType;
                }
                addition.m_fromActiveCatalog = fromActiveCatalog;
                existing = std::move(addition);
            }
            else
            {
                existing.m_fromActiveCatalog = fromActiveCatalog;
                if (existing.m_objectType.isEmpty()) {
                    existing.m_objectType = addition.m_objectType;
                }
            }
        }
    }

    bool loadCatalogFiles()
    {
        QHash<QString, CatalogMetadata> metadata;
        int onOrbitCount = 0;
        QStringList warnings;
        const CatalogDownloadSource& satcatSource = CatalogDownloadSources[0];
        QFile satcatFile(catalogFileName(satcatSource));

        if (satcatFile.open(QIODevice::ReadOnly))
        {
            QString error;
            if (!parseSatcat(
                    satcatFile.readAll(),
                    metadata,
                    onOrbitCount,
                    error))
            {
                warnings.append(QStringLiteral("SATCAT: %1").arg(error));
                metadata.clear();
                onOrbitCount = 0;
            }
        }
        else {
            warnings.append(QStringLiteral("SATCAT cache is unavailable"));
        }

        std::vector<CatalogEntry> catalog;
        catalog.reserve(22000);
        QHash<QString, int> indices;
        indices.reserve(22000);

        for (int sourceIndex = 1;
            sourceIndex < CatalogDownloadSourceCount;
            ++sourceIndex)
        {
            const CatalogDownloadSource& source =
                CatalogDownloadSources[sourceIndex];
            QFile file(catalogFileName(source));
            if (!file.open(QIODevice::ReadOnly))
            {
                warnings.append(QStringLiteral("%1 is unavailable")
                    .arg(QString::fromLatin1(source.m_cacheName)));
                continue;
            }

            std::vector<CatalogEntry> additions;
            QString error;
            if (!parseCatalog(
                    file.readAll(),
                    additions,
                    metadata,
                    source.m_kind == CatalogFileKind::ActiveGP,
                    source.m_minimumEntries,
                    error))
            {
                warnings.append(QStringLiteral("%1: %2")
                    .arg(QString::fromLatin1(source.m_cacheName), error));
                continue;
            }
            mergeCatalog(catalog, std::move(additions), indices);
        }

        if (catalog.empty()) {
            return false;
        }

        int activeCount = 0;
        for (const CatalogEntry& entry : catalog) {
            if (entry.m_fromActiveCatalog) {
                ++activeCount;
            }
        }
        warnings.append(m_refreshWarnings);
        installCatalog(
            std::move(catalog),
            activeCount,
            onOrbitCount,
            warnings);
        return true;
    }

    void installCatalog(
        std::vector<CatalogEntry>&& catalog,
        int activeCount,
        int onOrbitCount,
        const QStringList& warnings)
    {
        m_catalog = std::move(catalog);
        m_snapshots.clear();
        m_snapshotOrder.clear();
        const int supplementalCount = (int) m_catalog.size() - activeCount;
        m_status = QStringLiteral(
            "%1 orbital elements loaded (%2 active, %3 supplemental)")
                .arg(m_catalog.size())
                .arg(activeCount)
                .arg(supplementalCount);
        if (onOrbitCount > 0) {
            m_status += QStringLiteral("; SATCAT has %1 on-orbit objects")
                .arg(onOrbitCount);
        }
        if (!warnings.isEmpty()) {
            m_status += QStringLiteral("; %1 source warning(s)").arg(warnings.size());
        }

        MeteorSatelliteMatcher::CatalogStatistics statistics;
        statistics.m_status = m_status;
        statistics.m_loadedDateTimeUtc = QDateTime::currentDateTimeUtc();
        statistics.m_catalogEntries = (int) m_catalog.size();
        statistics.m_activeCatalogEntries = activeCount;
        statistics.m_supplementalCatalogEntries = supplementalCount;
        statistics.m_satcatOnOrbitEntries = onOrbitCount;
        statistics.m_sourceWarnings = warnings;

        for (const CatalogEntry& entry : m_catalog)
        {
            if (entry.m_objectType == QStringLiteral("PAY")) {
                ++statistics.m_payloadEntries;
            } else if (entry.m_objectType == QStringLiteral("R/B")) {
                ++statistics.m_rocketBodyEntries;
            } else if (entry.m_objectType == QStringLiteral("DEB")) {
                ++statistics.m_debrisEntries;
            } else {
                ++statistics.m_otherEntries;
            }
        }

        m_statistics = statistics;
        deliverStatistics();
        const std::vector<PendingRequest> pendingRequests =
            std::move(m_pendingRequests);
        m_pendingRequests.clear();
        for (const PendingRequest& request : pendingRequests) {
            requestMatch(request.m_requestId, request.m_observation, request.m_geometry);
        }
    }

    void refreshCatalogIfNeeded()
    {
        bool stale = m_catalog.empty();
        const QDateTime nowUtc = QDateTime::currentDateTimeUtc();

        for (int sourceIndex = 0;
            !stale && (sourceIndex < CatalogDownloadSourceCount);
            ++sourceIndex)
        {
            const QFileInfo fileInfo(catalogFileName(
                CatalogDownloadSources[sourceIndex]));
            stale = !fileInfo.exists()
                || (fileInfo.lastModified().toUTC().secsTo(nowUtc)
                    >= CatalogMaximumAgeS);
        }

        if (stale) {
            refreshCatalog();
        }
    }

    void completePendingRequestsWithoutMatch()
    {
        const std::vector<PendingRequest> pendingRequests = std::move(m_pendingRequests);
        m_pendingRequests.clear();

        for (const PendingRequest& request : pendingRequests)
        {
            const MeteorSatelliteMatcher::MoonPrediction moonPrediction =
                MeteorSatelliteMatcher::predictMoon(
                    request.m_observation,
                    request.m_geometry);
            QPointer<MeteorSatelliteMatcher> owner(m_owner);
            QMetaObject::invokeMethod(
                m_owner,
                [owner,
                    requestId = request.m_requestId,
                    moonPrediction]() {
                    if (owner) {
                        owner->deliverMatch(
                            requestId,
                            moonPrediction.m_match,
                            moonPrediction);
                    }
                },
                Qt::QueuedConnection);
        }
    }

    const QVector<MovingTargetMatcher::TargetState>& statesForTime(
        const QDateTime& centerTimeUtc,
        const MovingTargetMatcher::Observation& observation,
        const MeteorSatelliteMatcher::Geometry& geometry)
    {
        const QString requestedGeometryKey = geometryKey(observation, geometry);

        if (requestedGeometryKey != m_geometryKey)
        {
            m_geometryKey = requestedGeometryKey;
            m_snapshots.clear();
            m_snapshotOrder.clear();
        }

        const qint64 centerMSecs = centerTimeUtc.toMSecsSinceEpoch();
        const qint64 bucketMSecs = (centerMSecs / SnapshotIntervalMS) * SnapshotIntervalMS;
        auto existing = m_snapshots.find(bucketMSecs);

        if (existing != m_snapshots.end())
        {
            m_statistics = existing->m_statistics;
            return existing->m_states;
        }

        const QDateTime snapshotTimeUtc = QDateTime::fromMSecsSinceEpoch(
            bucketMSecs,
            Qt::UTC);
        Snapshot snapshot;
        QVector<MovingTargetMatcher::TargetState>& states = snapshot.m_states;
        states.reserve((int) std::min<size_t>(m_catalog.size(), 1024));
        snapshot.m_statistics = m_statistics;
        snapshot.m_statistics.m_snapshotValid = true;
        snapshot.m_statistics.m_snapshotDateTimeUtc = snapshotTimeUtc;
        snapshot.m_statistics.m_maximumAltitudeKM =
            geometry.m_maximumAltitudeM / 1000.0;
        snapshot.m_statistics.m_staleElementEntries = 0;
        snapshot.m_statistics.m_propagationFailureEntries = 0;
        snapshot.m_statistics.m_aboveMaximumAltitudeEntries = 0;
        snapshot.m_statistics.m_belowTransmitterHorizonEntries = 0;
        snapshot.m_statistics.m_belowReceiverHorizonEntries = 0;
        snapshot.m_statistics.m_outsideTransmitterBeamEntries = 0;
        snapshot.m_statistics.m_outsideReceiverBeamEntries = 0;
        snapshot.m_statistics.m_candidateEntries = 0;
        const libsgp4::DateTime sgp4Time = toSGP4DateTime(snapshotTimeUtc);

        for (CatalogEntry& entry : m_catalog)
        {
            if (std::llabs(entry.m_epochUtc.secsTo(snapshotTimeUtc)) > TLEMaximumAgeS)
            {
                ++snapshot.m_statistics.m_staleElementEntries;
                continue;
            }

            try
            {
                const libsgp4::Eci eci = entry.m_propagator->FindPosition(sgp4Time);
                const libsgp4::CoordGeodetic geo = eci.ToGeodetic();
                MovingTargetMatcher::TargetState state;
                state.m_source = QStringLiteral("TLE");
                state.m_id = entry.m_noradId;
                state.m_label = entry.m_name;
                state.m_dateTimeUtc = snapshotTimeUtc;
                state.m_position = {
                    libsgp4::Util::RadiansToDegrees(geo.latitude),
                    libsgp4::Util::RadiansToDegrees(geo.longitude),
                    geo.altitude * 1000.0
                };

                if ((geometry.m_maximumAltitudeM > 0.0)
                    && (state.m_position.m_altitudeM > geometry.m_maximumAltitudeM))
                {
                    ++snapshot.m_statistics.m_aboveMaximumAltitudeEntries;
                    continue;
                }

                const Vector3 targetPosition = geodeticToECEF(state.m_position);
                double txAzimuth;
                double txElevation;
                double rxAzimuth;
                double rxElevation;

                if (!siteLookAngles(
                        observation.m_transmitter,
                        targetPosition,
                        txAzimuth,
                        txElevation))
                {
                    ++snapshot.m_statistics.m_propagationFailureEntries;
                    continue;
                }
                if (!siteLookAngles(
                        observation.m_receiver,
                        targetPosition,
                        rxAzimuth,
                        rxElevation))
                {
                    ++snapshot.m_statistics.m_propagationFailureEntries;
                    continue;
                }
                if (txElevation < MinimumElevationDegrees)
                {
                    ++snapshot.m_statistics.m_belowTransmitterHorizonEntries;
                    continue;
                }
                if (rxElevation < MinimumElevationDegrees)
                {
                    ++snapshot.m_statistics.m_belowReceiverHorizonEntries;
                    continue;
                }
                if (!insideBeam(
                        txAzimuth,
                        txElevation,
                        geometry.m_transmitterBeam))
                {
                    ++snapshot.m_statistics.m_outsideTransmitterBeamEntries;
                    continue;
                }
                if (!insideBeam(
                        rxAzimuth,
                        rxElevation,
                        geometry.m_receiverBeam))
                {
                    ++snapshot.m_statistics.m_outsideReceiverBeamEntries;
                    continue;
                }

                const double velocityDeltaS = 0.5;
                const libsgp4::CoordGeodetic beforeGeo = entry.m_propagator->FindPosition(
                    sgp4Time.AddSeconds(-velocityDeltaS)).ToGeodetic();
                const libsgp4::CoordGeodetic afterGeo = entry.m_propagator->FindPosition(
                    sgp4Time.AddSeconds(velocityDeltaS)).ToGeodetic();
                const MovingTargetMatcher::Site beforeSite {
                    libsgp4::Util::RadiansToDegrees(beforeGeo.latitude),
                    libsgp4::Util::RadiansToDegrees(beforeGeo.longitude),
                    beforeGeo.altitude * 1000.0
                };
                const MovingTargetMatcher::Site afterSite {
                    libsgp4::Util::RadiansToDegrees(afterGeo.latitude),
                    libsgp4::Util::RadiansToDegrees(afterGeo.longitude),
                    afterGeo.altitude * 1000.0
                };
                const Vector3 velocityECEF =
                    (geodeticToECEF(afterSite) - geodeticToECEF(beforeSite))
                    * (1.0 / (2.0 * velocityDeltaS));
                ecefToENU(
                    velocityECEF,
                    state.m_position,
                    state.m_eastVelocityMPS,
                    state.m_northVelocityMPS,
                    state.m_upVelocityMPS);
                states.append(state);
                ++snapshot.m_statistics.m_candidateEntries;
            }
            catch (const std::exception&) {
                // Decayed and malformed objects are expected in large public catalogs.
                ++snapshot.m_statistics.m_propagationFailureEntries;
            }
        }

        m_statistics = snapshot.m_statistics;
        if (m_snapshotOrder.size() >= MaximumSnapshotCount)
        {
            m_snapshots.remove(m_snapshotOrder.front());
            m_snapshotOrder.pop_front();
        }
        m_snapshotOrder.append(bucketMSecs);
        return m_snapshots.insert(bucketMSecs, snapshot).value().m_states;
    }
#endif

    void deliverStatistics()
    {
        QPointer<MeteorSatelliteMatcher> owner(m_owner);
        MeteorSatelliteMatcher::CatalogStatistics statistics = m_statistics;
        statistics.m_status = m_status;
        for (const QString& warning : m_refreshWarnings)
        {
            if (!statistics.m_sourceWarnings.contains(warning)) {
                statistics.m_sourceWarnings.append(warning);
            }
        }
        QMetaObject::invokeMethod(
            m_owner,
            [owner, statistics]() {
                if (owner) {
                    owner->deliverStatistics(statistics);
                }
            },
            Qt::QueuedConnection);
    }

    MeteorSatelliteMatcher *m_owner;
    QNetworkAccessManager *m_networkManager = nullptr;
    QTimer *m_refreshTimer = nullptr;
    QString m_catalogDirectory;
    QString m_status = QStringLiteral("Orbital catalog is loading");
    MeteorSatelliteMatcher::CatalogStatistics m_statistics;
    QStringList m_refreshWarnings;
    bool m_downloadInProgress = false;
    int m_downloadSourceIndex = 0;
#ifdef METEOR_HAS_SGP4
    std::vector<CatalogEntry> m_catalog;
    std::vector<PendingRequest> m_pendingRequests;
    QHash<qint64, Snapshot> m_snapshots;
    QList<qint64> m_snapshotOrder;
    QString m_geometryKey;
#else
    std::vector<int> m_catalog;
#endif
};

MeteorSatelliteMatcher::MeteorSatelliteMatcher(QObject *parent) :
    QObject(parent),
    m_thread(new QThread(this)),
    m_worker(new MeteorSatelliteMatcherWorker(this))
{
    m_thread->setObjectName(QStringLiteral("Meteor satellite matcher"));
    m_worker->moveToThread(m_thread);
    connect(m_thread, &QThread::started, m_worker, [this]() {
        m_worker->start();
    });
    m_thread->start();
}

MeteorSatelliteMatcher::~MeteorSatelliteMatcher()
{
    if (m_worker && m_thread->isRunning())
    {
        MeteorSatelliteMatcherWorker *worker = m_worker;
        QMetaObject::invokeMethod(
            worker,
            [worker]() {
                delete worker;
            },
            Qt::BlockingQueuedConnection);
        m_worker = nullptr;
        m_thread->quit();
        m_thread->wait();
    }
}

void MeteorSatelliteMatcher::requestMatch(
    quint64 requestId,
    const MovingTargetMatcher::Observation& observation,
    const Geometry& geometry)
{
    if (!m_worker || !m_thread->isRunning()) {
        return;
    }

    MeteorSatelliteMatcherWorker *worker = m_worker;
    QMetaObject::invokeMethod(
        worker,
        [worker, requestId, observation, geometry]() {
            worker->requestMatch(requestId, observation, geometry);
        },
        Qt::QueuedConnection);
}

void MeteorSatelliteMatcher::refreshCatalog()
{
    if (!m_worker || !m_thread->isRunning()) {
        return;
    }

    MeteorSatelliteMatcherWorker *worker = m_worker;
    QMetaObject::invokeMethod(
        worker,
        [worker]() {
            worker->refreshCatalog();
        },
        Qt::QueuedConnection);
}

void MeteorSatelliteMatcher::requestStatistics()
{
    if (!m_worker || !m_thread->isRunning()) {
        return;
    }

    MeteorSatelliteMatcherWorker *worker = m_worker;
    QMetaObject::invokeMethod(
        worker,
        [worker]() {
            worker->requestStatistics();
        },
        Qt::QueuedConnection);
}

void MeteorSatelliteMatcher::deliverMatch(
    quint64 requestId,
    const MovingTargetMatcher::Match& match,
    const MoonPrediction& moonPrediction)
{
    emit matchReady(requestId, match, moonPrediction);
}

void MeteorSatelliteMatcher::deliverStatistics(
    const CatalogStatistics& statistics)
{
    emit statisticsReady(statistics);
}
