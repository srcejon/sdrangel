///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <cmath>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QLocale>
#include <QSaveFile>
#include <QTextStream>

#include "rmobreport.h"

bool RMOBReport::load(
    const QString& fileName,
    const QDate& reportMonthDate,
    Data& data,
    QString *error)
{
    QFile file(fileName);

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (error) {
            *error = QString("Failed to open file %1: %2").arg(fileName, file.errorString());
        }

        return false;
    }

    const QDate monthDate = reportMonthDate.isValid() ? reportMonthDate : QDate::currentDate();
    const int year = monthDate.year();
    const int month = monthDate.month();
    const int daysInMonth = monthDate.daysInMonth();
    Data loadedData;
    QTextStream in(&file);

    while (!in.atEnd())
    {
        const QString line = in.readLine();

        if (line.startsWith('[')) {
            break;
        }

        const QStringList fields = line.split('|');

        if (fields.size() < 25) {
            continue;
        }

        bool dayOK = false;
        const int day = fields[0].trimmed().toInt(&dayOK);

        if (!dayOK || (day < 1) || (day > daysInMonth)) {
            continue;
        }

        const QDate date(year, month, day);
        QVector<int> counts(24, 0);
        QVector<bool> present(24, false);
        bool hasData = false;

        for (int hour = 0; hour < 24; hour++)
        {
            const QString value = fields.value(hour + 1).trimmed();

            if (value.isEmpty() || (value == "???")) {
                continue;
            }

            bool countOK = false;
            const int count = value.toInt(&countOK);

            if (countOK)
            {
                const int nonNegativeCount = std::max(0, count);
                hasData = true;
                present[hour] = true;
                counts[hour] = nonNegativeCount;
                loadedData.m_totalCount += nonNegativeCount;
            }
        }

        if (hasData)
        {
            loadedData.m_hourlyCounts[date] = counts;
            loadedData.m_hourlyData[date] = present;
        }
    }

    data = loadedData;
    return true;
}

bool RMOBReport::save(
    const QString& fileName,
    const QDate& reportMonthDate,
    const Data& data,
    const Metadata& metadata,
    QString *error)
{
    QDir dir = QFileInfo(fileName).absoluteDir();

    if (!dir.exists() && !dir.mkpath("."))
    {
        if (error) {
            *error = QString("Failed to create directory %1").arg(dir.absolutePath());
        }

        return false;
    }

    QSaveFile file(fileName);

    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        if (error) {
            *error = QString("Failed to open file %1").arg(fileName);
        }

        return false;
    }

    const QDate monthDate = reportMonthDate.isValid() ? reportMonthDate : QDate::currentDate();
    const int year = monthDate.year();
    const int month = monthDate.month();
    const int daysInMonth = monthDate.daysInMonth();
    const QString monthName = QLocale::c().standaloneMonthName(month, QLocale::ShortFormat).toLower();
    QTextStream out(&file);

    out << monthName << "|";

    for (int hour = 0; hour < 24; hour++) {
        out << QString("%1h|").arg(hour, 2, 10, QLatin1Char('0'));
    }

    out << "\n";

    for (int day = 1; day <= daysInMonth; day++)
    {
        const QDate date(year, month, day);
        out << QString("%1|").arg(day, 2, 10, QLatin1Char('0'));

        for (int hour = 0; hour < 24; hour++)
        {
            const QString value = hasHourData(data, date, hour)
                ? QString::number(data.m_hourlyCounts.value(date).value(hour, 0))
                : QStringLiteral("???");
            out << QString("%1|").arg(value, -4, QLatin1Char(' '));
        }

        out << "\n";
    }

    out << "[Observer]" << metadata.m_observer << "\n";
    out << "[Country]\n";
    out << "[City]\n";
    out << "[Longitude]" << formatCoordinate(metadata.m_longitude, false) << "\n";
    out << "[Latitude ]" << formatCoordinate(metadata.m_latitude, true) << "\n";
    out << "[Longitude GMAP]" << QString::number(metadata.m_longitude, 'f', 4) << "\n";
    out << "[Latitude GMAP]" << QString("%1%2")
        .arg(metadata.m_latitude >= 0.0 ? "+" : "")
        .arg(metadata.m_latitude, 0, 'f', 4) << "\n";
    out << "[Frequencies]" << formatFrequency(metadata.m_frequency) << "\n";
    out << "[Antenna]\n";
    out << "[Azimut Antenna]\n";
    out << "[Elevation Antenna]\n";
    out << "[Pre-Amplifier]\n";
    out << "[Receiver]" << metadata.m_receiver << "\n";
    out << "[Observing Method]" << metadata.m_observingMethod << "\n";
    out.flush();

    if ((out.status() != QTextStream::Ok) || !file.commit())
    {
        if (error) {
            *error = QString("Failed to save file %1: %2").arg(fileName, file.errorString());
        }

        return false;
    }

    return true;
}

QString RMOBReport::formatFrequency(qint64 frequency)
{
    if (frequency <= 0) {
        return QString();
    }

    const qint64 frequencyHz = std::llabs(frequency);
    const qint64 mhz = frequencyHz / 1000000;
    const qint64 khz = (frequencyHz / 1000) % 1000;
    const qint64 hz = frequencyHz % 1000;

    return QString("%1.%2.%3")
        .arg(mhz, 4, 10, QLatin1Char('0'))
        .arg(khz, 3, 10, QLatin1Char('0'))
        .arg(hz, 3, 10, QLatin1Char('0'));
}

QString RMOBReport::formatCoordinate(double coordinate, bool latitude)
{
    const double absCoordinate = std::fabs(coordinate);
    int degrees = (int) std::floor(absCoordinate);
    double minutesDecimal = (absCoordinate - (double) degrees) * 60.0;
    int minutes = (int) std::floor(minutesDecimal);
    int seconds = (int) std::round((minutesDecimal - (double) minutes) * 60.0);

    if (seconds >= 60)
    {
        seconds -= 60;
        minutes++;
    }

    if (minutes >= 60)
    {
        minutes -= 60;
        degrees++;
    }

    const QChar hemisphere = latitude
        ? (coordinate >= 0.0 ? QChar('N') : QChar('S'))
        : (coordinate >= 0.0 ? QChar('E') : QChar('W'));

    return QString("%1%2%3%4 %5")
        .arg(degrees, 3, 10, QLatin1Char('0'))
        .arg(QChar(0x00b0))
        .arg(minutes, 2, 10, QLatin1Char('0'))
        .arg(seconds, 2, 10, QLatin1Char('0'))
        .arg(hemisphere);
}

bool RMOBReport::hasHourData(const Data& data, const QDate& date, int hour)
{
    return (hour >= 0)
        && (hour < 24)
        && data.m_hourlyData.contains(date)
        && data.m_hourlyData.value(date).value(hour);
}
