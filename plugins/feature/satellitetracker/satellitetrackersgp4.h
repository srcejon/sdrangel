///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2021-2022 Jon Beniston, M7RCE <jon@beniston.com>                //
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

#ifndef INCLUDE_FEATURE_SATELLITETRACKERSGP4_H_
#define INCLUDE_FEATURE_SATELLITETRACKERSGP4_H_

#include <memory>

#include <QList>
#include <QDateTime>
#include <QtCharts/QLineSeries>

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
using namespace QtCharts;
#endif

struct SatelliteState;

class GroundTrackDetails {
    QDateTime m_quantizedDateTime;
    QString m_tle0;
    QString m_tle1;
    QString m_tle2;
    int m_groundTrackSteps = 0;
public:
    bool match(const QDateTime& quantizedDateTime, const QString& tle0, const QString& tle1, const QString& tle2, int groundTrackSteps) const;
    void update(const QDateTime& quantizedDateTime, const QString& tle0, const QString& tle1, const QString& tle2, int groundTrackSteps);
    void invalidate();
};

class SatelliteStateContext {
public:
    SatelliteStateContext();
    ~SatelliteStateContext();
    SatelliteStateContext(const SatelliteStateContext&) = delete;
    SatelliteStateContext& operator=(const SatelliteStateContext&) = delete;

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;

    friend void getSatelliteState(QDateTime dateTime,
                            const QString& tle0, const QString& tle1, const QString& tle2,
                            double latitude, double longitude, double altitude,
                            int predictionPeriod, int minAOSElevationDeg, int minPassElevationDeg,
                            QTime passStartTime, QTime passFinishTime, bool utc,
                            int noOfPasses,
                            bool calcGroundTrack, int groundTrackSteps, SatelliteState *satState,
                            SatelliteStateContext *context);
};

struct SatellitePass {
    QDateTime m_aos;
    QDateTime m_los;
    double m_maxElevation;              // Degrees
    double m_aosAzimuth;                // Degrees
    double m_losAzimuth;                // Degrees
    bool m_northToSouth;
};

struct SatelliteTrack {
    QList<double> m_latitudes;
    QList<double> m_longitudes;
    QList<double> m_altitudes;
    QList<qint64> m_dateTimeMsecs;
    quint64 m_revision = 0;

    void clear()
    {
        m_latitudes.clear();
        m_longitudes.clear();
        m_altitudes.clear();
        m_dateTimeMsecs.clear();
    }

    void reserve(int size)
    {
        m_latitudes.reserve(size);
        m_longitudes.reserve(size);
        m_altitudes.reserve(size);
        m_dateTimeMsecs.reserve(size);
    }

    bool isValid() const
    {
        return (m_latitudes.size() == m_longitudes.size())
            && (m_latitudes.size() == m_altitudes.size())
            && (m_latitudes.size() == m_dateTimeMsecs.size());
    }
};

struct SatelliteState {
    QString m_name;
    double m_latitude = 0.0;            // Degrees
    double m_longitude = 0.0;           // Degrees
    double m_altitude = 0.0;            // km
    double m_azimuth = 0.0;             // Degrees
    double m_elevation = 0.0;           // Degrees
    double m_range = 0.0;               // km
    double m_rangeRate = 0.0;           // km/s
    double m_speed = 0.0;
    double m_period = 0.0;
    QString m_error;
    QList<SatellitePass> m_passes;              // Used in worker and GUI threads
    SatelliteTrack m_groundTrack;               // Used only in worker thread, to send to Map
    SatelliteTrack m_predictedGroundTrack;
    GroundTrackDetails m_groundTrackDetails;
};

void getSatelliteState(QDateTime dateTime,
                        const QString& tle0, const QString& tle1, const QString& tle2,
                        double latitude, double longitude, double altitude,
                        int predictionPeriod, int minAOSElevationDeg, int minPassElevationDeg,
                        QTime passStartTime, QTime passFinishTime, bool utc,
                        int noOfPasses, 
                        bool calcGroundTrack, int groundTrackSteps, SatelliteState *satState,
                        SatelliteStateContext *context = nullptr);

void getPassAzEl(QLineSeries *azimuth, QLineSeries *elevation, QLineSeries *polar,
                        const QString& tle0, const QString& tle1, const QString& tle2,
                        double latitude, double longitude, double altitude,
                        const QDateTime& aos, const QDateTime& los);

bool getPassesThrough0Deg(const QString& tle0, const QString& tle1, const QString& tle2,
                          double latitude, double longitude, double altitude,
                          QDateTime& aos, QDateTime& los);

#endif // INCLUDE_FEATURE_SATELLITETRACKERSGP4_H_
