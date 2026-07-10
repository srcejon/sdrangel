///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_RMOBREPORT_H
#define INCLUDE_RMOBREPORT_H

#include <QDate>
#include <QMap>
#include <QString>
#include <QVector>

class RMOBReport
{
public:
    using HourlyCounts = QMap<QDate, QVector<int>>;
    using HourlyData = QMap<QDate, QVector<bool>>;

    struct Data
    {
        HourlyCounts m_hourlyCounts;
        HourlyData m_hourlyData;
        int m_totalCount = 0;
    };

    struct Metadata
    {
        QString m_observer;
        double m_latitude = 0.0;
        double m_longitude = 0.0;
        qint64 m_frequency = 0;
        QString m_receiver;
        QString m_observingMethod = QStringLiteral("SDRangel");
    };

    static bool load(
        const QString& fileName,
        const QDate& reportMonthDate,
        Data& data,
        QString *error = nullptr);
    static bool save(
        const QString& fileName,
        const QDate& reportMonthDate,
        const Data& data,
        const Metadata& metadata,
        QString *error = nullptr);

    static QString formatFrequency(qint64 frequency);
    static QString formatCoordinate(double coordinate, bool latitude);

private:
    static bool hasHourData(const Data& data, const QDate& date, int hour);
};

#endif // INCLUDE_RMOBREPORT_H
