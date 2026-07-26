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
    const char * const ActiveCatalogUrl =
        "https://celestrak.org/NORAD/elements/gp.php?GROUP=active&FORMAT=CSV";

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
            "%1,%2,%3;%4,%5,%6;%7,%8,%9,%10;%11,%12,%13,%14")
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
            .arg(rx.m_verticalBeamwidthDegrees, 0, 'f', 2);
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
        m_catalogFileName = directoryPath + QStringLiteral("/celestrak-active.csv");
        loadCatalogFile();
        refreshCatalogIfNeeded();
#else
        deliverCatalogStatus(QStringLiteral("Satellite matching unavailable: SGP4 was not found"));
#endif
    }

    void refreshCatalog()
    {
#ifdef METEOR_HAS_SGP4
        if (m_downloadInProgress || !m_networkManager) {
            return;
        }

        m_downloadInProgress = true;
        QNetworkRequest request(QUrl(QString::fromLatin1(ActiveCatalogUrl)));
        request.setHeader(
            QNetworkRequest::UserAgentHeader,
            QStringLiteral("SDRangel Meteor satellite matcher"));
        request.setAttribute(
            QNetworkRequest::RedirectPolicyAttribute,
            QNetworkRequest::NoLessSafeRedirectPolicy);
        // A stalled connection must not wedge m_downloadInProgress until restart: the
        // finished signal always fires once the transfer timeout elapses.
        request.setTransferTimeout(2 * 60 * 1000);
        QNetworkReply *reply = m_networkManager->get(request);
        connect(reply, &QNetworkReply::finished, this, [this, reply]() {
            m_downloadInProgress = false;
            const QByteArray data = reply->readAll();
            const QString error = reply->error() == QNetworkReply::NoError
                ? QString()
                : reply->errorString();
            reply->deleteLater();

            if (!error.isEmpty())
            {
                m_status = QStringLiteral("TLE update failed: %1").arg(error);
                deliverCatalogStatus(m_status);
                completePendingRequestsWithoutMatch();
                return;
            }

            std::vector<CatalogEntry> catalog;
            QString parseError;

            if (!parseCatalog(data, catalog, parseError))
            {
                m_status = QStringLiteral("TLE update rejected: %1").arg(parseError);
                deliverCatalogStatus(m_status);
                completePendingRequestsWithoutMatch();
                return;
            }

            QSaveFile file(m_catalogFileName);
            if (file.open(QIODevice::WriteOnly)
                && (file.write(data) == data.size())
                && file.commit())
            {
                installCatalog(std::move(catalog));
            }
            else
            {
                m_status = QStringLiteral("TLE catalog could not be cached");
                installCatalog(std::move(catalog));
            }
        });
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
        const int catalogSize = (int) m_catalog.size();
        const QString status = m_status;
        QMetaObject::invokeMethod(
            m_owner,
            [owner, requestId, match, moonPrediction, catalogSize, status]() {
                if (owner) {
                    owner->deliverMatch(
                        requestId,
                        match,
                        moonPrediction,
                        catalogSize,
                        status);
                }
            },
            Qt::QueuedConnection);
    }

private:
#ifdef METEOR_HAS_SGP4
    struct CatalogEntry
    {
        QString m_name;
        QString m_noradId;
        QDateTime m_epochUtc;
        std::unique_ptr<libsgp4::SGP4> m_propagator;
    };

    struct PendingRequest
    {
        quint64 m_requestId;
        MovingTargetMatcher::Observation m_observation;
        MeteorSatelliteMatcher::Geometry m_geometry;
    };

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

    static bool parseCatalog(
        const QByteArray& data,
        std::vector<CatalogEntry>& catalog,
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
                const QString noradId = field(QStringLiteral("NORAD_CAT_ID"));
                const QString name = field(QStringLiteral("OBJECT_NAME"));
                std::unique_ptr<libsgp4::Tle> tle(new libsgp4::Tle(
                    name.toStdString(),
                    line1.toStdString(),
                    line2.toStdString()));
                CatalogEntry entry;
                entry.m_noradId = noradId;
                entry.m_name = name.isEmpty()
                    ? QStringLiteral("NORAD %1").arg(noradId)
                    : name;
                entry.m_epochUtc = epochUtc;
                entry.m_propagator.reset(new libsgp4::SGP4(*tle));
                catalog.push_back(std::move(entry));
            }
            catch (const std::exception&) {
                // A bad element must not prevent the rest of the active catalog loading.
            }
        }

        if (catalog.size() < 100)
        {
            error = QStringLiteral("only %1 valid elements were found").arg(catalog.size());
            return false;
        }

        return true;
    }

    void loadCatalogFile()
    {
        QFile file(m_catalogFileName);

        if (!file.open(QIODevice::ReadOnly)) {
            return;
        }

        std::vector<CatalogEntry> catalog;
        QString error;

        if (parseCatalog(file.readAll(), catalog, error)) {
            installCatalog(std::move(catalog));
        } else {
            m_status = QStringLiteral("Cached TLE catalog rejected: %1").arg(error);
        }
    }

    void installCatalog(std::vector<CatalogEntry>&& catalog)
    {
        m_catalog = std::move(catalog);
        m_snapshots.clear();
        m_snapshotOrder.clear();
        m_status = QStringLiteral("%1 active TLEs loaded").arg(m_catalog.size());
        deliverCatalogStatus(m_status);
        const std::vector<PendingRequest> pendingRequests = std::move(m_pendingRequests);
        m_pendingRequests.clear();
        for (const PendingRequest& request : pendingRequests) {
            requestMatch(request.m_requestId, request.m_observation, request.m_geometry);
        }
    }

    void refreshCatalogIfNeeded()
    {
        const QFileInfo fileInfo(m_catalogFileName);
        const bool stale = m_catalog.empty()
            || !fileInfo.exists()
            || (fileInfo.lastModified().toUTC().secsTo(QDateTime::currentDateTimeUtc())
                >= CatalogMaximumAgeS);

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
            const QString status = m_status;
            QMetaObject::invokeMethod(
                m_owner,
                [owner,
                    requestId = request.m_requestId,
                    moonPrediction,
                    status]() {
                    if (owner) {
                        owner->deliverMatch(
                            requestId,
                            moonPrediction.m_match,
                            moonPrediction,
                            0,
                            status);
                    }
                },
                Qt::QueuedConnection);
        }
    }

    QVector<MovingTargetMatcher::TargetState>& statesForTime(
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

        if (existing != m_snapshots.end()) {
            return existing.value();
        }

        const QDateTime snapshotTimeUtc = QDateTime::fromMSecsSinceEpoch(
            bucketMSecs,
            Qt::UTC);
        QVector<MovingTargetMatcher::TargetState> states;
        states.reserve((int) std::min<size_t>(m_catalog.size(), 1024));
        const libsgp4::DateTime sgp4Time = toSGP4DateTime(snapshotTimeUtc);

        for (CatalogEntry& entry : m_catalog)
        {
            if (std::llabs(entry.m_epochUtc.secsTo(snapshotTimeUtc)) > TLEMaximumAgeS) {
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
                const Vector3 targetPosition = geodeticToECEF(state.m_position);
                double txAzimuth;
                double txElevation;
                double rxAzimuth;
                double rxElevation;

                if (!siteLookAngles(
                        observation.m_transmitter,
                        targetPosition,
                        txAzimuth,
                        txElevation)
                    || !siteLookAngles(
                        observation.m_receiver,
                        targetPosition,
                        rxAzimuth,
                        rxElevation)
                    || (txElevation < MinimumElevationDegrees)
                    || (rxElevation < MinimumElevationDegrees)
                    || !insideBeam(txAzimuth, txElevation, geometry.m_transmitterBeam)
                    || !insideBeam(rxAzimuth, rxElevation, geometry.m_receiverBeam))
                {
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
            }
            catch (const std::exception&) {
                // Decayed and malformed objects are expected in large public catalogs.
            }
        }

        if (m_snapshotOrder.size() >= MaximumSnapshotCount)
        {
            m_snapshots.remove(m_snapshotOrder.front());
            m_snapshotOrder.pop_front();
        }
        m_snapshotOrder.append(bucketMSecs);
        return m_snapshots.insert(bucketMSecs, states).value();
    }
#endif

    void deliverCatalogStatus(const QString& status)
    {
        QPointer<MeteorSatelliteMatcher> owner(m_owner);
        const int catalogSize = (int) m_catalog.size();
        QMetaObject::invokeMethod(
            m_owner,
            [owner, catalogSize, status]() {
                if (owner) {
                    owner->deliverCatalogStatus(catalogSize, status);
                }
            },
            Qt::QueuedConnection);
    }

    MeteorSatelliteMatcher *m_owner;
    QNetworkAccessManager *m_networkManager = nullptr;
    QTimer *m_refreshTimer = nullptr;
    QString m_catalogFileName;
    QString m_status = QStringLiteral("TLE catalog is loading");
    bool m_downloadInProgress = false;
#ifdef METEOR_HAS_SGP4
    std::vector<CatalogEntry> m_catalog;
    std::vector<PendingRequest> m_pendingRequests;
    QHash<qint64, QVector<MovingTargetMatcher::TargetState>> m_snapshots;
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

void MeteorSatelliteMatcher::deliverMatch(
    quint64 requestId,
    const MovingTargetMatcher::Match& match,
    const MoonPrediction& moonPrediction,
    int catalogSize,
    const QString& status)
{
    emit matchReady(requestId, match, moonPrediction, catalogSize, status);
}

void MeteorSatelliteMatcher::deliverCatalogStatus(int catalogSize, const QString& status)
{
    emit catalogStatusChanged(catalogSize, status);
}
