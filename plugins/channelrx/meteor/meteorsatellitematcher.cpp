///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License.                     //
///////////////////////////////////////////////////////////////////////////////////

#include "meteorsatellitematcher.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <vector>

#include <QBuffer>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QLocale>
#include <QMetaObject>
#include <QNetworkAccessManager>
#include <QNetworkCookie>
#include <QNetworkCookieJar>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPointer>
#include <QSaveFile>
#include <QStandardPaths>
#include <QStringList>
#include <QTextStream>
#include <QThread>
#include <QTimer>
#include <QUrl>
#include <QUrlQuery>

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
    constexpr double SpeedOfLightMPS = 299792458.0;
    constexpr qint64 CelesTrakMinimumDownloadIntervalS = 2 * 60 * 60;
    constexpr qint64 CatalogMaximumAgeS = 24 * 60 * 60;
    constexpr qint64 TLEMaximumAgeS = 14 * 24 * 60 * 60;
    constexpr qint64 SnapshotIntervalMS = 5000;
    constexpr int MaximumSnapshotCount = 8;
    constexpr double MinimumElevationDegrees = -1.0;
    // Snapshot culling is evaluated at a bucketed instant with zero-margin beam edges,
    // but a LEO target moves up to ~1 deg/s: pad the horizon/beam tests so the object
    // that produced the sweep is not culled while actually inside the beam mid-sweep.
    // The exact per-endpoint scoring afterwards decides for real.
    constexpr double SnapshotCullMarginDegrees = 10.0;
    // The maximum-altitude setting doubles as the map projection altitude, where values
    // like 100 km (the meteor ablation region) are natural; below any real satellite
    // orbit it cannot be a meaningful matcher cull, so treat it as display-only rather
    // than silently discarding the whole catalog.
    constexpr double MinimumAltitudeCullM = 300000.0;
    constexpr int SpaceTrackMinimumEntries = 10000;
    constexpr qint64 ClockCheckIntervalMS = 30 * 60 * 1000;
    constexpr qint64 ClockCheckTimeoutMS = 10 * 1000;
    constexpr qint64 ClockDateHeaderHalfResolutionMS = 500;
    constexpr double AcceptableClockErrorMS = 1000.0;
    constexpr char ClockTimeSourceUrl[] =
        "https://www.google.com/generate_204";
    constexpr char SpaceTrackLoginUrl[] =
        "https://www.space-track.org/ajaxauth/login";
    constexpr char SpaceTrackCatalogUrl[] =
        "https://www.space-track.org/basicspacedata/query/class/gp/"
        "EPOCH/%3Enow-30/orderby/NORAD_CAT_ID,EPOCH/format/3le";
    constexpr char SpaceTrackCacheName[] = "space-track-gp.3le";

    QDateTime parseHttpDate(const QByteArray& dateHeader)
    {
        const QString dateText = QString::fromLatin1(dateHeader).trimmed();
        QDateTime dateTime = QDateTime::fromString(dateText, Qt::RFC2822Date);

        if (dateTime.isValid()) {
            return dateTime.toUTC();
        }

        // HTTP-date uses IMF-fixdate, but recipients should also accept the two
        // obsolete forms. Parse them explicitly because Qt's RFC 2822 parser
        // does not consistently accept the named GMT zone.
        static const QStringList formats {
            QStringLiteral("ddd, dd MMM yyyy HH:mm:ss 'GMT'"),
            QStringLiteral("dddd, dd-MMM-yy HH:mm:ss 'GMT'"),
            QStringLiteral("ddd MMM d HH:mm:ss yyyy")
        };

        for (const QString& format : formats)
        {
            dateTime = QLocale::c().toDateTime(dateText, format);

            if (dateTime.isValid()) {
                return QDateTime(dateTime.date(), dateTime.time(), Qt::UTC);
            }
        }

        return {};
    }

    class MeteorNetworkCookieJar : public QNetworkCookieJar
    {
    public:
        explicit MeteorNetworkCookieJar(QObject *parent = nullptr) :
            QNetworkCookieJar(parent)
        {}

        void clear()
        {
            setAllCookies({});
        }
    };

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

    double dot(const Vector3& left, const Vector3& right)
    {
        return left.m_x * right.m_x + left.m_y * right.m_y + left.m_z * right.m_z;
    }

    double length(const Vector3& vector)
    {
        return std::sqrt(dot(vector, vector));
    }

    // Same convention as MovingTargetMatcher: shortening TX-target-RX path -> positive
    // received-frequency offset.
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
        return -referenceFrequencyHz * pathRateMPS / SpeedOfLightMPS;
    }

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
        const MeteorSatelliteMatcher::Beam& beam,
        double marginDegrees = 0.0)
    {
        const double azimuth = azimuthDegrees * Pi / 180.0;
        const double elevation = elevationDegrees * Pi / 180.0;
        const double beamAzimuth = beam.m_azimuthDegrees * Pi / 180.0;
        const double beamElevation = beam.m_elevationDegrees * Pi / 180.0;
        const Vector3 direction {
            std::cos(elevation) * std::sin(azimuth),
            std::cos(elevation) * std::cos(azimuth),
            std::sin(elevation)
        };
        const Vector3 boresight {
            std::cos(beamElevation) * std::sin(beamAzimuth),
            std::cos(beamElevation) * std::cos(beamAzimuth),
            std::sin(beamElevation)
        };
        const Vector3 horizontalAxis {
            std::cos(beamAzimuth),
            -std::sin(beamAzimuth),
            0.0
        };
        const Vector3 verticalAxis {
            -std::sin(beamElevation) * std::sin(beamAzimuth),
            -std::sin(beamElevation) * std::cos(beamAzimuth),
            std::cos(beamElevation)
        };
        const double forward = dot(direction, boresight);
        const double horizontalOffset = std::fabs(
            std::atan2(dot(direction, horizontalAxis), forward) * 180.0 / Pi);
        const double verticalOffset = std::fabs(
            std::atan2(dot(direction, verticalAxis), forward) * 180.0 / Pi);
        const bool horizontalOK = !(beam.m_horizontalBeamwidthDegrees > 0.0)
            || (beam.m_horizontalBeamwidthDegrees >= 360.0)
            || (horizontalOffset <= beam.m_horizontalBeamwidthDegrees * 0.5 + marginDegrees);
        const bool verticalOK = !(beam.m_verticalBeamwidthDegrees > 0.0)
            || (beam.m_verticalBeamwidthDegrees >= 180.0)
            || (verticalOffset <= beam.m_verticalBeamwidthDegrees * 0.5 + marginDegrees);
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

bool MeteorSatelliteMatcher::beamContainsLookDirection(
    double azimuthDegrees,
    double elevationDegrees,
    const Beam& beam,
    double marginDegrees)
{
    return insideBeam(azimuthDegrees, elevationDegrees, beam, marginDegrees);
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
    explicit MeteorSatelliteMatcherWorker(
        MeteorSatelliteMatcher *owner,
        const QString& spaceTrackUsername,
        const QString& spaceTrackPassword) :
        m_owner(owner),
        m_spaceTrackUsername(spaceTrackUsername.trimmed()),
        m_spaceTrackPassword(spaceTrackPassword)
    {}

    void start()
    {
#ifdef METEOR_HAS_SGP4
        m_networkManager = new QNetworkAccessManager(this);
        m_cookieJar = new MeteorNetworkCookieJar(m_networkManager);
        m_networkManager->setCookieJar(m_cookieJar);
        m_refreshTimer = new QTimer(this);
        m_refreshTimer->setInterval(6 * 60 * 60 * 1000);
        connect(m_refreshTimer, &QTimer::timeout, this, [this]() {
            refreshCatalogIfNeeded();
        });
        m_refreshTimer->start();
        m_clockCheckTimer = new QTimer(this);
        m_clockCheckTimer->setInterval(ClockCheckIntervalMS);
        connect(m_clockCheckTimer, &QTimer::timeout, this, [this]() {
            startClockCheck();
        });
        m_clockCheckTimer->start();
        startClockCheck();

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

    void setSpaceTrackCredentials(
        const QString& username,
        const QString& password)
    {
#ifdef METEOR_HAS_SGP4
        m_spaceTrackUsername = username.trimmed();
        m_spaceTrackPassword = password;
        m_statistics.m_spaceTrackConfigured =
            !m_spaceTrackUsername.isEmpty() && !m_spaceTrackPassword.isEmpty();

        if (m_cookieJar) {
            m_cookieJar->clear();
        }

        if (m_downloadInProgress)
        {
            m_spaceTrackRefreshRequested = true;
            deliverStatistics();
            return;
        }

        refreshCatalog(false, true);
#else
        Q_UNUSED(username)
        Q_UNUSED(password)
#endif
    }

    void refreshCatalog(
        bool forceAll = true,
        bool forceSpaceTrack = false)
    {
#ifdef METEOR_HAS_SGP4
        if (m_downloadInProgress || !m_networkManager || m_stopRequested.load()) {
            return;
        }

        // Download only the sources whose cache is actually stale: one permanently
        // failing source must not force a full ten-source re-download of everything
        // (CelesTrak rate-limits abusive clients) at every start and refresh tick.
        const QDateTime nowUtc = QDateTime::currentDateTimeUtc();
        m_downloadQueue.clear();
        const QFileInfo spaceTrackCache(spaceTrackCatalogFileName());
        const bool spaceTrackStale = !spaceTrackCache.exists()
            || (spaceTrackCache.lastModified().toUTC().secsTo(nowUtc)
                >= CatalogMaximumAgeS);
        m_spaceTrackDownloadPending =
            !m_spaceTrackUsername.isEmpty()
            && !m_spaceTrackPassword.isEmpty()
            && (forceAll || forceSpaceTrack || spaceTrackStale);

        for (int sourceIndex = 0; sourceIndex < CatalogDownloadSourceCount; ++sourceIndex)
        {
            const QFileInfo fileInfo(catalogFileName(
                CatalogDownloadSources[sourceIndex]));
            const qint64 cacheAgeS = fileInfo.exists()
                ? fileInfo.lastModified().toUTC().secsTo(nowUtc)
                : -1;
            // CelesTrak rejects another request for the same file within two
            // hours of a successful download. This server-side limit also
            // applies to explicit refreshes, so a force request may bypass the
            // normal one-day freshness policy but never this cooldown.
            const bool downloadCooldownActive =
                fileInfo.exists()
                && (cacheAgeS < CelesTrakMinimumDownloadIntervalS);
            const bool stale = !fileInfo.exists()
                || (cacheAgeS >= CatalogMaximumAgeS);

            if (!downloadCooldownActive && (forceAll || stale)) {
                m_downloadQueue.append(sourceIndex);
            }
        }

        if (!m_spaceTrackDownloadPending && m_downloadQueue.isEmpty()) {
            return;
        }

        m_downloadInProgress = true;
        m_downloadQueuePos = 0;
        m_refreshWarnings.clear();
        if (m_spaceTrackDownloadPending) {
            authenticateSpaceTrack();
        } else {
            downloadNextCatalogSource();
        }
#endif
    }

    // Called directly from the GUI thread during teardown: the worker checks this in
    // its long loops so channel close is not blocked behind a catalog parse or a full
    // snapshot propagation pass.
    void requestStop()
    {
        m_stopRequested.store(true);
    }

    void requestMatch(
        quint64 requestId,
        const MovingTargetMatcher::Observation& observation,
        const MeteorSatelliteMatcher::Geometry& geometry)
    {
        if (m_stopRequested.load()) {
            return;
        }

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
            const Snapshot& snapshot = candidatesForTime(centerTimeUtc, observation, geometry);
            QVector<MovingTargetMatcher::PredictedCandidate> candidates;
            candidates.reserve(snapshot.m_candidateIndices.size());

            for (int catalogIndex : snapshot.m_candidateIndices)
            {
                MovingTargetMatcher::PredictedCandidate candidate;

                if (predictCatalogEntry(m_catalog[catalogIndex], observation, candidate)) {
                    candidates.append(candidate);
                }
            }

            match = MovingTargetMatcher::combine(
                match,
                MovingTargetMatcher::matchPredictions(observation, candidates));
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
        const QDateTime nowUtc = QDateTime::currentDateTimeUtc();
        if (!m_clockCheckInProgress
            && (!m_clockCheckDateTimeUtc.isValid()
                || (m_clockCheckDateTimeUtc.msecsTo(nowUtc) >= ClockCheckIntervalMS))) {
            startClockCheck();
        }
        deliverStatistics();
    }

private:
    void startClockCheck()
    {
        if (!m_networkManager || m_clockCheckInProgress || m_stopRequested.load()) {
            return;
        }

        m_clockCheckInProgress = true;
        m_clockCheckError.clear();
        const QDateTime localStartUtc = QDateTime::currentDateTimeUtc();
        const auto elapsed = std::make_shared<QElapsedTimer>();
        elapsed->start();

        QUrl clockUrl(QString::fromLatin1(ClockTimeSourceUrl));
        QUrlQuery clockQuery;
        clockQuery.addQueryItem(
            QStringLiteral("sdrangel-clock-check"),
            QString::number(localStartUtc.toMSecsSinceEpoch()));
        clockUrl.setQuery(clockQuery);
        QNetworkRequest request(clockUrl);
        request.setHeader(
            QNetworkRequest::UserAgentHeader,
            QStringLiteral("SDRangel Meteor clock check"));
        request.setRawHeader("Cache-Control", "no-cache");
        request.setAttribute(
            QNetworkRequest::RedirectPolicyAttribute,
            QNetworkRequest::NoLessSafeRedirectPolicy);
        request.setAttribute(
            QNetworkRequest::CacheLoadControlAttribute,
            QNetworkRequest::AlwaysNetwork);
        request.setTransferTimeout(ClockCheckTimeoutMS);

        QNetworkReply *reply = m_networkManager->head(request);
        connect(reply, &QNetworkReply::finished, this, [this, reply, localStartUtc, elapsed]() {
            const qint64 roundTripMS = elapsed->elapsed();
            const int httpStatus = reply->attribute(
                QNetworkRequest::HttpStatusCodeAttribute).toInt();
            const QNetworkReply::NetworkError networkError = reply->error();
            const QString networkErrorText = reply->errorString();
            const QByteArray dateHeader = reply->rawHeader("Date");
            reply->deleteLater();

            m_clockCheckInProgress = false;
            m_clockCheckDateTimeUtc = QDateTime::currentDateTimeUtc();
            m_clockRoundTripMS = (double) roundTripMS;
            m_clockUncertaintyMS =
                0.5 * (double) roundTripMS + ClockDateHeaderHalfResolutionMS;
            m_clockCheckAvailable = false;

            if (m_stopRequested.load()) {
                return;
            }

            if ((networkError != QNetworkReply::NoError)
                || (httpStatus < 200)
                || (httpStatus >= 400))
            {
                m_clockCheckError = networkError != QNetworkReply::NoError
                    ? networkErrorText
                    : QStringLiteral("time request returned HTTP %1").arg(httpStatus);
            }
            else
            {
                const QDateTime internetTimeUtc = parseHttpDate(dateHeader);
                if (!internetTimeUtc.isValid())
                {
                    m_clockCheckError = dateHeader.isEmpty()
                        ? QStringLiteral("time response did not contain a Date header")
                        : QStringLiteral("time response contained an invalid Date header");
                }
                else
                {
                    const QDateTime localMidpointUtc = localStartUtc.addMSecs(
                        roundTripMS / 2);
                    // HTTP Date has whole-second precision and is normally truncated.
                    // Compare against the centre of that represented one-second interval.
                    m_localClockErrorMS = (double)
                        internetTimeUtc.addMSecs(ClockDateHeaderHalfResolutionMS)
                            .msecsTo(localMidpointUtc);
                    m_clockCheckAvailable = true;
                    m_clockCheckError.clear();
                }
            }

            deliverStatistics();
        });
    }

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
        bool m_fromSpaceTrack = false;
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
        // Indices into m_catalog of the entries passing the (margin-padded) geometric
        // culling at the snapshot instant; the per-request exact endpoint propagation
        // works from these. Valid until the next installCatalog (which clears snapshots).
        QVector<int> m_candidateIndices;
        MeteorSatelliteMatcher::CatalogStatistics m_statistics;
    };

    QString catalogFileName(const CatalogDownloadSource& source) const
    {
        return m_catalogDirectory + QChar('/') + QString::fromLatin1(source.m_cacheName);
    }

    QString spaceTrackCatalogFileName() const
    {
        return m_catalogDirectory + QChar('/')
            + QString::fromLatin1(SpaceTrackCacheName);
    }

    void finishCatalogDownloads()
    {
        m_downloadInProgress = false;

        if (!loadCatalogFiles())
        {
            m_status = m_catalog.empty()
                ? QStringLiteral("No usable satellite orbital elements are available")
                : QStringLiteral("Satellite catalog refresh failed; using cached elements");
            if (!m_refreshWarnings.isEmpty()) {
                m_status += QStringLiteral(" (%1)")
                    .arg(m_refreshWarnings.join(QStringLiteral("; ")));
            }
            deliverStatistics();
            if (m_catalog.empty()) {
                completePendingRequestsWithoutMatch();
            }
        }

        if (m_spaceTrackRefreshRequested && !m_stopRequested.load())
        {
            m_spaceTrackRefreshRequested = false;
            QMetaObject::invokeMethod(
                this,
                [this]() {
                    refreshCatalog(false, true);
                },
                Qt::QueuedConnection);
        }
    }

    void appendSpaceTrackWarning(const QString& error)
    {
        m_refreshWarnings.append(
            QStringLiteral("Space-Track: %1").arg(error));
    }

    void authenticateSpaceTrack()
    {
        if (m_stopRequested.load())
        {
            m_downloadInProgress = false;
            return;
        }

        QNetworkRequest request(QUrl(QString::fromLatin1(SpaceTrackLoginUrl)));
        request.setHeader(
            QNetworkRequest::UserAgentHeader,
            QStringLiteral("SDRangel Meteor satellite matcher"));
        request.setHeader(
            QNetworkRequest::ContentTypeHeader,
            QStringLiteral("application/x-www-form-urlencoded"));
        request.setAttribute(
            QNetworkRequest::RedirectPolicyAttribute,
            QNetworkRequest::NoLessSafeRedirectPolicy);
        request.setTransferTimeout(2 * 60 * 1000);

        QUrlQuery form;
        form.addQueryItem(QStringLiteral("identity"), m_spaceTrackUsername);
        form.addQueryItem(QStringLiteral("password"), m_spaceTrackPassword);
        QNetworkReply *reply = m_networkManager->post(
            request,
            form.query(QUrl::FullyEncoded).toUtf8());
        connect(reply, &QNetworkReply::finished, this, [this, reply]() {
            const int httpStatus = reply->attribute(
                QNetworkRequest::HttpStatusCodeAttribute).toInt();
            const QNetworkReply::NetworkError networkError = reply->error();
            const QString networkErrorText = reply->errorString();
            reply->readAll();
            reply->deleteLater();
            m_spaceTrackDownloadPending = false;

            if (m_stopRequested.load())
            {
                m_downloadInProgress = false;
                return;
            }

            if ((networkError != QNetworkReply::NoError)
                || (httpStatus < 200)
                || (httpStatus >= 300))
            {
                appendSpaceTrackWarning(
                    networkError != QNetworkReply::NoError
                        ? networkErrorText
                        : QStringLiteral("login returned HTTP %1").arg(httpStatus));
                downloadNextCatalogSource();
                return;
            }

            downloadSpaceTrackCatalog();
        });
    }

    void downloadSpaceTrackCatalog()
    {
        QNetworkRequest request(
            QUrl::fromEncoded(QByteArray(SpaceTrackCatalogUrl)));
        request.setHeader(
            QNetworkRequest::UserAgentHeader,
            QStringLiteral("SDRangel Meteor satellite matcher"));
        request.setAttribute(
            QNetworkRequest::RedirectPolicyAttribute,
            QNetworkRequest::NoLessSafeRedirectPolicy);
        request.setTransferTimeout(5 * 60 * 1000);
        QNetworkReply *reply = m_networkManager->get(request);
        connect(reply, &QNetworkReply::finished, this, [this, reply]() {
            const QByteArray data = reply->readAll();
            const int httpStatus = reply->attribute(
                QNetworkRequest::HttpStatusCodeAttribute).toInt();
            QString error = reply->error() == QNetworkReply::NoError
                ? QString()
                : reply->errorString();
            reply->deleteLater();

            if (m_stopRequested.load())
            {
                m_downloadInProgress = false;
                return;
            }

            if (error.isEmpty()
                && ((httpStatus < 200) || (httpStatus >= 300)))
            {
                error = QStringLiteral("catalog request returned HTTP %1")
                    .arg(httpStatus);
            }

            if (error.isEmpty()
                && !validateSpaceTrack3LE(
                    data,
                    SpaceTrackMinimumEntries,
                    &m_stopRequested,
                    error))
            {
                error.prepend(QStringLiteral("invalid catalog data: "));
            }

            if (error.isEmpty())
            {
                QSaveFile file(spaceTrackCatalogFileName());
                if (!file.open(QIODevice::WriteOnly)
                    || (file.write(data) != data.size())
                    || !file.commit())
                {
                    error = QStringLiteral("could not cache %1")
                        .arg(QString::fromLatin1(SpaceTrackCacheName));
                }
            }

            if (!error.isEmpty()) {
                appendSpaceTrackWarning(error);
            }

            downloadNextCatalogSource();
        });
    }

    void downloadNextCatalogSource()
    {
        if (m_stopRequested.load())
        {
            m_downloadInProgress = false;
            return;
        }

        if (m_downloadQueuePos >= m_downloadQueue.size())
        {
            finishCatalogDownloads();
            return;
        }

        const CatalogDownloadSource source =
            CatalogDownloadSources[m_downloadQueue[m_downloadQueuePos]];
        QNetworkRequest request(QUrl(QString::fromLatin1(source.m_url)));
        request.setHeader(
            QNetworkRequest::UserAgentHeader,
            QStringLiteral("SDRangel Meteor satellite matcher"));
        const QFileInfo cachedInfo(catalogFileName(source));

        // CelesTrak asks clients to cache: let the server answer 304 when unchanged.
        if (cachedInfo.exists()) {
            request.setHeader(
                QNetworkRequest::IfModifiedSinceHeader,
                cachedInfo.lastModified());
        }

        request.setAttribute(
            QNetworkRequest::RedirectPolicyAttribute,
            QNetworkRequest::NoLessSafeRedirectPolicy);
        // A stalled connection must not wedge the refresh until restart.
        request.setTransferTimeout(2 * 60 * 1000);
        QNetworkReply *reply = m_networkManager->get(request);
        connect(reply, &QNetworkReply::finished, this, [this, reply, source]() {
            const QByteArray data = reply->readAll();
            const int httpStatus = reply->attribute(
                QNetworkRequest::HttpStatusCodeAttribute).toInt();
            QString error = reply->error() == QNetworkReply::NoError
                ? QString()
                : reply->errorString();
            reply->deleteLater();

            if (m_stopRequested.load())
            {
                m_downloadInProgress = false;
                return;
            }

            if (error.isEmpty() && (httpStatus == 304))
            {
                // Not modified: keep the cache and refresh its timestamp so the
                // staleness check does not re-request it before the next full period.
                QFile cachedFile(catalogFileName(source));
                if (cachedFile.open(QIODevice::ReadWrite))
                {
                    cachedFile.setFileTime(
                        QDateTime::currentDateTimeUtc(),
                        QFileDevice::FileModificationTime);
                }
            }
            else
            {
                if (error.isEmpty()
                    && !validateCatalogData(
                        data,
                        source.m_kind,
                        source.m_minimumEntries,
                        &m_stopRequested,
                        error))
                {
                    error.prepend(QStringLiteral("invalid catalog data: "));
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
            }

            if (!error.isEmpty())
            {
                m_refreshWarnings.append(
                    QStringLiteral("%1: %2")
                        .arg(QString::fromLatin1(source.m_cacheName), error));
            }

            ++m_downloadQueuePos;
            downloadNextCatalogSource();
        });
    }

    static bool validateSpaceTrack3LE(
        const QByteArray& data,
        int minimumEntries,
        const std::atomic<bool> *cancel,
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
        int entryCount = 0;
        int lineNumber = 0;
        QString nameLine;
        QString line1;
        QString line2;

        while (!stream.atEnd())
        {
            const QString line = stream.readLine();
            ++lineNumber;
            if (line.trimmed().isEmpty()) {
                continue;
            }

            const int position = nameLine.isEmpty()
                ? 0
                : (line1.isEmpty() ? 1 : 2);
            if (position == 0)
            {
                if (!line.startsWith(QStringLiteral("0 ")))
                {
                    error = QStringLiteral("line %1 is not a 3LE name line")
                        .arg(lineNumber);
                    return false;
                }
                nameLine = line;
            }
            else if (position == 1)
            {
                if (!line.startsWith(QStringLiteral("1 ")))
                {
                    error = QStringLiteral("line %1 is not TLE line 1")
                        .arg(lineNumber);
                    return false;
                }
                line1 = line;
            }
            else
            {
                if (!line.startsWith(QStringLiteral("2 ")))
                {
                    error = QStringLiteral("line %1 is not TLE line 2")
                        .arg(lineNumber);
                    return false;
                }
                line2 = line;
                ++entryCount;
                nameLine.clear();
                line1.clear();
                line2.clear();

                if (cancel && ((entryCount & 1023) == 0) && cancel->load())
                {
                    error = QStringLiteral("cancelled");
                    return false;
                }
            }
        }

        if (!nameLine.isEmpty() || !line1.isEmpty())
        {
            error = QStringLiteral("catalog ends with an incomplete 3LE record");
            return false;
        }
        if (entryCount < minimumEntries)
        {
            error = QStringLiteral("only %1 3LE entries were found")
                .arg(entryCount);
            return false;
        }
        return true;
    }

    // Structural validation only (header + row count): the full parse - including
    // ~15k SGP4 propagator constructions - happens exactly once, in loadCatalogFiles.
    static bool validateCatalogData(
        const QByteArray& data,
        CatalogFileKind kind,
        int minimumEntries,
        const std::atomic<bool> *cancel,
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
        CSV::readHeader(
            stream,
            kind == CatalogFileKind::Satcat
                ? satcatRequiredColumns()
                : catalogRequiredColumns(),
            error);

        if (!error.isEmpty()) {
            return false;
        }

        int rowCount = 0;
        QStringList row;

        while (CSV::readRow(stream, &row))
        {
            ++rowCount;

            if (cancel && ((rowCount & 1023) == 0) && cancel->load())
            {
                error = QStringLiteral("cancelled");
                return false;
            }
        }

        if (rowCount < minimumEntries)
        {
            error = QStringLiteral("only %1 rows were found").arg(rowCount);
            return false;
        }

        return true;
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

    static const QStringList& satcatRequiredColumns()
    {
        static const QStringList columns {
            QStringLiteral("OBJECT_NAME"),
            QStringLiteral("NORAD_CAT_ID"),
            QStringLiteral("OBJECT_TYPE"),
            QStringLiteral("DECAY_DATE")
        };
        return columns;
    }

    static const QStringList& catalogRequiredColumns()
    {
        static const QStringList columns {
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
        return columns;
    }

    static QString decodeTLECatalogId(const QString& field)
    {
        const QString value = field.trimmed().toUpper();
        bool numericOK = false;
        const int numericId = value.toInt(&numericOK);
        if (numericOK) {
            return QString::number(numericId);
        }

        if (value.size() != 5) {
            return QString();
        }

        // Space-Track Alpha-5 IDs use A-H, J-N and P-Z for the ten-thousands
        // prefix; I and O are deliberately omitted.
        static const QString prefixes =
            QStringLiteral("ABCDEFGHJKLMNPQRSTUVWXYZ");
        const int prefixIndex = prefixes.indexOf(value.front());
        bool suffixOK = false;
        const int suffix = value.mid(1).toInt(&suffixOK);
        if ((prefixIndex < 0) || !suffixOK) {
            return QString();
        }
        return QString::number((prefixIndex + 10) * 10000 + suffix);
    }

    static bool parseTLEEpoch(
        const QString& line1,
        QDateTime& epochUtc)
    {
        if (line1.size() < 32) {
            return false;
        }

        bool yearOK = false;
        bool dayOK = false;
        const int shortYear = line1.mid(18, 2).toInt(&yearOK);
        const double epochDay = line1.mid(20, 12).toDouble(&dayOK);
        const int year = shortYear >= 57 ? 1900 + shortYear : 2000 + shortYear;
        const int wholeDay = (int) std::floor(epochDay);
        const QDate firstDay(year, 1, 1);
        if (!yearOK
            || !dayOK
            || !firstDay.isValid()
            || (wholeDay < 1)
            || (wholeDay > firstDay.daysInYear()))
        {
            return false;
        }

        const qint64 milliseconds = std::llround(
            (epochDay - wholeDay) * 86400000.0);
        epochUtc = QDateTime(
            firstDay.addDays(wholeDay - 1),
            QTime(0, 0),
            Qt::UTC).addMSecs(milliseconds);
        return epochUtc.isValid();
    }

    static bool parseTLEExponent(
        const QString& field,
        double& value)
    {
        const QString fixed = field.rightJustified(8, QChar(' '));
        if (fixed.size() != 8) {
            return false;
        }

        const QChar mantissaSign = fixed[0];
        const QChar exponentSign = fixed[6];
        bool mantissaOK = false;
        bool exponentOK = false;
        const int mantissa = fixed.mid(1, 5).toInt(&mantissaOK);
        const int exponent = fixed.mid(7, 1).toInt(&exponentOK);
        if (!mantissaOK || !exponentOK) {
            return false;
        }

        value = mantissa * 1.0e-5
            * std::pow(10.0, exponentSign == QChar('-') ? -exponent : exponent);
        if (mantissaSign == QChar('-')) {
            value = -value;
        }
        return std::isfinite(value);
    }

    static bool parseSpaceTrack3LE(
        const QByteArray& data,
        std::vector<CatalogEntry>& catalog,
        const QHash<QString, CatalogMetadata>& metadata,
        int minimumEntries,
        const std::atomic<bool> *cancel,
        QString& error)
    {
        if (!validateSpaceTrack3LE(
                data,
                minimumEntries,
                cancel,
                error))
        {
            return false;
        }

        QBuffer buffer;
        buffer.setData(data);
        if (!buffer.open(QIODevice::ReadOnly))
        {
            error = QStringLiteral("catalog buffer could not be opened");
            return false;
        }

        QTextStream stream(&buffer);
        catalog.reserve(32000);
        int syntheticId = 0;
        int identifiedRows = 0;
        while (!stream.atEnd())
        {
            QString nameLine = stream.readLine();
            if (nameLine.trimmed().isEmpty()) {
                continue;
            }
            if (stream.atEnd()) {
                break;
            }
            const QString sourceLine1 = stream.readLine();
            if (stream.atEnd()) {
                break;
            }
            const QString sourceLine2 = stream.readLine();
            ++identifiedRows;

            if (cancel
                && ((identifiedRows & 1023) == 0)
                && cancel->load())
            {
                error = QStringLiteral("cancelled");
                return false;
            }

            const QString noradId = decodeTLECatalogId(
                sourceLine1.mid(2, 5));
            if (noradId.isEmpty()
                || (noradId != decodeTLECatalogId(sourceLine2.mid(2, 5))))
            {
                continue;
            }

            const auto metadataEntry = metadata.constFind(noradId);
            if ((metadataEntry != metadata.cend()) && !metadataEntry->m_onOrbit) {
                continue;
            }

            bool meanMotionOK = false;
            bool inclinationOK = false;
            bool ascendingNodeOK = false;
            bool eccentricityOK = false;
            bool argumentPericenterOK = false;
            bool meanAnomalyOK = false;
            bool revolutionOK = false;
            const double meanMotion =
                sourceLine2.mid(52, 11).toDouble(&meanMotionOK);
            const double inclination =
                sourceLine2.mid(8, 8).toDouble(&inclinationOK);
            const double ascendingNode =
                sourceLine2.mid(17, 8).toDouble(&ascendingNodeOK);
            const double eccentricity =
                QStringLiteral("0.%1").arg(sourceLine2.mid(26, 7))
                    .toDouble(&eccentricityOK);
            const double argumentPericenter =
                sourceLine2.mid(34, 8).toDouble(&argumentPericenterOK);
            const double meanAnomaly =
                sourceLine2.mid(43, 8).toDouble(&meanAnomalyOK);
            const int revolution =
                sourceLine2.mid(63, 5).toInt(&revolutionOK);
            double bstar = 0.0;
            QDateTime epochUtc;
            if (!meanMotionOK
                || !inclinationOK
                || !ascendingNodeOK
                || !eccentricityOK
                || !argumentPericenterOK
                || !meanAnomalyOK
                || !revolutionOK
                || !parseTLEExponent(sourceLine1.mid(53, 8), bstar)
                || !parseTLEEpoch(sourceLine1, epochUtc))
            {
                continue;
            }

            QString name = nameLine.mid(2).trimmed();
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
            if (!makeCompatibilityTLE(
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
                entry.m_fromSpaceTrack = true;
                entry.m_propagator.reset(new libsgp4::SGP4(*tle));
                catalog.push_back(std::move(entry));
            }
            catch (const std::exception&) {
                // Continue past individual malformed orbital elements.
            }
        }

        if (identifiedRows < minimumEntries)
        {
            error = QStringLiteral("only %1 identified 3LE entries were found")
                .arg(identifiedRows);
            return false;
        }
        return true;
    }

    static bool parseSatcat(
        const QByteArray& data,
        QHash<QString, CatalogMetadata>& metadata,
        int& onOrbitCount,
        int minimumEntries,
        const std::atomic<bool> *cancel,
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
        const QHash<QString, int> columns = CSV::readHeader(
            stream,
            satcatRequiredColumns(),
            error);
        if (!error.isEmpty()) {
            return false;
        }

        metadata.clear();
        metadata.reserve(70000);
        onOrbitCount = 0;
        QStringList row;
        int rowCount = 0;

        while (CSV::readRow(stream, &row))
        {
            if (cancel && ((++rowCount & 1023) == 0) && cancel->load())
            {
                error = QStringLiteral("cancelled");
                return false;
            }

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

        if (metadata.size() < minimumEntries)
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
        const std::atomic<bool> *cancel,
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
        const QHash<QString, int> columns = CSV::readHeader(
            stream,
            catalogRequiredColumns(),
            error);
        if (!error.isEmpty()) {
            return false;
        }

        catalog.reserve(12000);
        QStringList row;
        int syntheticId = 0;
        int identifiedRows = 0;
        int rowCount = 0;

        while (CSV::readRow(stream, &row))
        {
            if (cancel && ((++rowCount & 1023) == 0) && cancel->load())
            {
                error = QStringLiteral("cancelled");
                return false;
            }

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

            if (noradId.isEmpty()) {
                continue;
            }

            // The minimum-entry sanity gate counts identified rows BEFORE the decayed
            // filter, so a small group whose members have mostly decayed still counts
            // as a well-formed file (matching what download validation accepted).
            ++identifiedRows;
            const auto metadataEntry = metadata.constFind(noradId);

            if ((metadataEntry != metadata.cend()) && !metadataEntry->m_onOrbit) {
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

        if (identifiedRows < minimumEntries)
        {
            error = QStringLiteral("only %1 identified rows were found").arg(identifiedRows);
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
            const bool fromSpaceTrack =
                existing.m_fromSpaceTrack || addition.m_fromSpaceTrack;
            if (addition.m_epochUtc > existing.m_epochUtc)
            {
                if (addition.m_objectType.isEmpty()) {
                    addition.m_objectType = existing.m_objectType;
                }
                addition.m_fromActiveCatalog = fromActiveCatalog;
                addition.m_fromSpaceTrack = fromSpaceTrack;
                existing = std::move(addition);
            }
            else
            {
                existing.m_fromActiveCatalog = fromActiveCatalog;
                existing.m_fromSpaceTrack = fromSpaceTrack;
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
                    satcatSource.m_minimumEntries,
                    &m_stopRequested,
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
        catalog.reserve(34000);
        QHash<QString, int> indices;
        indices.reserve(34000);

        const QFileInfo spaceTrackInfo(spaceTrackCatalogFileName());
        if (spaceTrackInfo.exists())
        {
            QFile file(spaceTrackInfo.filePath());
            if (!file.open(QIODevice::ReadOnly))
            {
                warnings.append(QStringLiteral("Space-Track cache could not be opened"));
            }
            else
            {
                std::vector<CatalogEntry> additions;
                QString error;
                if (!parseSpaceTrack3LE(
                        file.readAll(),
                        additions,
                        metadata,
                        SpaceTrackMinimumEntries,
                        &m_stopRequested,
                        error))
                {
                    warnings.append(QStringLiteral("Space-Track cache: %1").arg(error));
                }
                else {
                    mergeCatalog(catalog, std::move(additions), indices);
                }
            }
        }

        for (int sourceIndex = 1;
            sourceIndex < CatalogDownloadSourceCount;
            ++sourceIndex)
        {
            if (m_stopRequested.load()) {
                return false;
            }

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
                    &m_stopRequested,
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
        int spaceTrackCount = 0;
        for (const CatalogEntry& entry : catalog) {
            if (entry.m_fromActiveCatalog) {
                ++activeCount;
            }
            if (entry.m_fromSpaceTrack) {
                ++spaceTrackCount;
            }
        }
        warnings.append(m_refreshWarnings);
        installCatalog(
            std::move(catalog),
            activeCount,
            spaceTrackCount,
            onOrbitCount,
            warnings);
        return true;
    }

    void installCatalog(
        std::vector<CatalogEntry>&& catalog,
        int activeCount,
        int spaceTrackCount,
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
        statistics.m_spaceTrackCatalogEntries = spaceTrackCount;
        statistics.m_satcatOnOrbitEntries = onOrbitCount;
        statistics.m_spaceTrackConfigured =
            !m_spaceTrackUsername.isEmpty() && !m_spaceTrackPassword.isEmpty();
        const QFileInfo spaceTrackInfo(spaceTrackCatalogFileName());
        statistics.m_spaceTrackCacheAvailable = spaceTrackInfo.exists();
        if (spaceTrackInfo.exists()) {
            statistics.m_spaceTrackCacheDateTimeUtc =
                spaceTrackInfo.lastModified().toUTC();
        }
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
        // With no usable catalog every source is worth (re)fetching; otherwise only
        // the stale ones are queued (no-op when everything is fresh).
        refreshCatalog(m_catalog.empty());
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

    const Snapshot& candidatesForTime(
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
            return existing.value();
        }

        const QDateTime snapshotTimeUtc = QDateTime::fromMSecsSinceEpoch(
            bucketMSecs,
            Qt::UTC);
        Snapshot snapshot;
        QVector<int>& candidateIndices = snapshot.m_candidateIndices;
        candidateIndices.reserve((int) std::min<size_t>(m_catalog.size(), 1024));
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

        for (int catalogIndex = 0; catalogIndex < (int) m_catalog.size(); ++catalogIndex)
        {
            if (m_stopRequested.load(std::memory_order_relaxed)) {
                break;
            }

            CatalogEntry& entry = m_catalog[catalogIndex];

            if (std::llabs(entry.m_epochUtc.secsTo(snapshotTimeUtc)) > TLEMaximumAgeS)
            {
                ++snapshot.m_statistics.m_staleElementEntries;
                continue;
            }

            try
            {
                const libsgp4::Eci eci = entry.m_propagator->FindPosition(sgp4Time);
                const libsgp4::CoordGeodetic geo = eci.ToGeodetic();
                const MovingTargetMatcher::Site position {
                    libsgp4::Util::RadiansToDegrees(geo.latitude),
                    libsgp4::Util::RadiansToDegrees(geo.longitude),
                    geo.altitude * 1000.0
                };

                if ((geometry.m_maximumAltitudeM >= MinimumAltitudeCullM)
                    && (position.m_altitudeM > geometry.m_maximumAltitudeM))
                {
                    ++snapshot.m_statistics.m_aboveMaximumAltitudeEntries;
                    continue;
                }

                const Vector3 targetPosition = geodeticToECEF(position);
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
                if (txElevation < MinimumElevationDegrees - SnapshotCullMarginDegrees)
                {
                    ++snapshot.m_statistics.m_belowTransmitterHorizonEntries;
                    continue;
                }
                if (rxElevation < MinimumElevationDegrees - SnapshotCullMarginDegrees)
                {
                    ++snapshot.m_statistics.m_belowReceiverHorizonEntries;
                    continue;
                }
                if (!insideBeam(
                        txAzimuth,
                        txElevation,
                        geometry.m_transmitterBeam,
                        SnapshotCullMarginDegrees))
                {
                    ++snapshot.m_statistics.m_outsideTransmitterBeamEntries;
                    continue;
                }
                if (!insideBeam(
                        rxAzimuth,
                        rxElevation,
                        geometry.m_receiverBeam,
                        SnapshotCullMarginDegrees))
                {
                    ++snapshot.m_statistics.m_outsideReceiverBeamEntries;
                    continue;
                }

                candidateIndices.append(catalogIndex);
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
        return m_snapshots.insert(bucketMSecs, snapshot).value();
    }

    // Exact per-endpoint evaluation: SGP4 position AND velocity at the sweep's own
    // start/end epochs. A single constant-velocity extrapolation misses the orbital
    // acceleration along the lines of sight (up to ~8 Hz/s of drift at 143 MHz for
    // LEO), which is comparable to the matcher's drift tolerance floor.
    bool predictCatalogEntry(
        CatalogEntry& entry,
        const MovingTargetMatcher::Observation& observation,
        MovingTargetMatcher::PredictedCandidate& candidate)
    {
        constexpr double velocityDeltaS = 0.5;
        const Vector3 transmitterPosition = geodeticToECEF(observation.m_transmitter);
        const Vector3 receiverPosition = geodeticToECEF(observation.m_receiver);
        const QDateTime endpointTimesUtc[2] = {
            observation.m_startDateTimeUtc,
            observation.m_startDateTimeUtc.addMSecs(
                (qint64) std::llround(observation.m_durationS * 1000.0))
        };
        double offsetsHz[2];

        try
        {
            for (int endpoint = 0; endpoint < 2; ++endpoint)
            {
                const libsgp4::DateTime sgp4Time = toSGP4DateTime(endpointTimesUtc[endpoint]);
                auto ecefAt = [&entry](const libsgp4::DateTime& when) {
                    const libsgp4::CoordGeodetic geo =
                        entry.m_propagator->FindPosition(when).ToGeodetic();
                    const MovingTargetMatcher::Site site {
                        libsgp4::Util::RadiansToDegrees(geo.latitude),
                        libsgp4::Util::RadiansToDegrees(geo.longitude),
                        geo.altitude * 1000.0
                    };
                    return geodeticToECEF(site);
                };
                const Vector3 position = ecefAt(sgp4Time);
                const Vector3 velocity =
                    (ecefAt(sgp4Time.AddSeconds(velocityDeltaS))
                        - ecefAt(sgp4Time.AddSeconds(-velocityDeltaS)))
                    * (1.0 / (2.0 * velocityDeltaS));
                offsetsHz[endpoint] = bistaticDopplerOffset(
                    position,
                    velocity,
                    transmitterPosition,
                    receiverPosition,
                    observation.m_referenceFrequencyHz);

                if (!std::isfinite(offsetsHz[endpoint])) {
                    return false;
                }
            }
        }
        catch (const std::exception&) {
            return false;
        }

        candidate.m_source = QStringLiteral("TLE");
        candidate.m_id = entry.m_noradId;
        candidate.m_label = entry.m_name;
        candidate.m_stateAgeS = 0.0;
        candidate.m_prediction.m_startFrequencyOffsetHz = offsetsHz[0];
        candidate.m_prediction.m_endFrequencyOffsetHz = offsetsHz[1];
        candidate.m_prediction.m_centerFrequencyOffsetHz =
            0.5 * (offsetsHz[0] + offsetsHz[1]);
        candidate.m_prediction.m_frequencyDriftHz = offsetsHz[1] - offsetsHz[0];
        candidate.m_prediction.m_valid = true;
        return true;
    }
#endif

    void deliverStatistics()
    {
        QPointer<MeteorSatelliteMatcher> owner(m_owner);
        MeteorSatelliteMatcher::CatalogStatistics statistics = m_statistics;
        statistics.m_status = m_status;
        statistics.m_clockCheckPending = m_clockCheckInProgress;
        statistics.m_clockCheckAvailable = m_clockCheckAvailable;
        statistics.m_clockCheckDateTimeUtc = m_clockCheckDateTimeUtc;
        statistics.m_clockTimeSource =
            QString::fromLatin1(ClockTimeSourceUrl);
        statistics.m_clockCheckError = m_clockCheckError;
        statistics.m_localClockErrorMS = m_localClockErrorMS;
        statistics.m_clockUncertaintyMS = m_clockUncertaintyMS;
        statistics.m_clockRoundTripMS = m_clockRoundTripMS;
        statistics.m_acceptableClockErrorMS = AcceptableClockErrorMS;
#ifdef METEOR_HAS_SGP4
        statistics.m_spaceTrackConfigured =
            !m_spaceTrackUsername.isEmpty() && !m_spaceTrackPassword.isEmpty();
        const QFileInfo spaceTrackInfo(spaceTrackCatalogFileName());
        statistics.m_spaceTrackCacheAvailable = spaceTrackInfo.exists();
        if (spaceTrackInfo.exists()) {
            statistics.m_spaceTrackCacheDateTimeUtc =
                spaceTrackInfo.lastModified().toUTC();
        }
#endif
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
    MeteorNetworkCookieJar *m_cookieJar = nullptr;
    QTimer *m_refreshTimer = nullptr;
    QTimer *m_clockCheckTimer = nullptr;
    QString m_catalogDirectory;
    QString m_status = QStringLiteral("Orbital catalog is loading");
    MeteorSatelliteMatcher::CatalogStatistics m_statistics;
    QStringList m_refreshWarnings;
    QString m_spaceTrackUsername;
    QString m_spaceTrackPassword;
    bool m_downloadInProgress = false;
    bool m_spaceTrackDownloadPending = false;
    bool m_spaceTrackRefreshRequested = false;
    bool m_clockCheckInProgress = false;
    bool m_clockCheckAvailable = false;
    QDateTime m_clockCheckDateTimeUtc;
    QString m_clockCheckError;
    double m_localClockErrorMS = 0.0;
    double m_clockUncertaintyMS = 0.0;
    double m_clockRoundTripMS = 0.0;
    QVector<int> m_downloadQueue;
    int m_downloadQueuePos = 0;
    std::atomic<bool> m_stopRequested {false};
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

MeteorSatelliteMatcher::MeteorSatelliteMatcher(
    const QString& spaceTrackUsername,
    const QString& spaceTrackPassword,
    QObject *parent) :
    QObject(parent),
    m_thread(new QThread(this)),
    m_worker(new MeteorSatelliteMatcherWorker(
        this,
        spaceTrackUsername,
        spaceTrackPassword))
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
    // Stop flag first so a long parse or snapshot pass bails out promptly, then stop
    // the event loop; remaining queued work is discarded with the worker. Deleting the
    // worker (and its network manager/timer children) from this thread is safe once
    // its thread has finished.
    if (m_worker) {
        m_worker->requestStop();
    }

    m_thread->quit();
    m_thread->wait();
    delete m_worker;
    m_worker = nullptr;
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
            worker->refreshCatalog(true);
        },
        Qt::QueuedConnection);
}

void MeteorSatelliteMatcher::setSpaceTrackCredentials(
    const QString& username,
    const QString& password)
{
    if (!m_worker || !m_thread->isRunning()) {
        return;
    }

    MeteorSatelliteMatcherWorker *worker = m_worker;
    QMetaObject::invokeMethod(
        worker,
        [worker, username, password]() {
            worker->setSpaceTrackCredentials(username, password);
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
