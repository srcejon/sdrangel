///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License.                     //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_SYSTEMCLOCKOFFSET_H
#define INCLUDE_SYSTEMCLOCKOFFSET_H

#include <QDateTime>
#include <QList>
#include <QObject>
#include <QString>

#include "export.h"

class QElapsedTimer;
class QNetworkAccessManager;
class QNetworkReply;
class QTimer;
class QUdpSocket;

class SDRBASE_API SystemClockOffset : public QObject
{
    Q_OBJECT

public:
    static constexpr qint64 DefaultCheckIntervalMS = 30 * 60 * 1000;
    static constexpr qint64 DefaultMaximumAgeMS = 60 * 60 * 1000;
    static constexpr double DefaultAcceptableClockErrorMS = 1000.0;

    enum class Assessment
    {
        Unavailable,
        Acceptable,
        Inconclusive,
        Unacceptable
    };

    struct SDRBASE_API Measurement
    {
        bool m_pending = false;
        bool m_available = false;
        QDateTime m_dateTimeUtc;
        QString m_timeSource;
        QString m_error;
        double m_localClockErrorMS = 0.0; // Positive when the local clock is fast.
        double m_uncertaintyMS = 0.0;
        double m_roundTripMS = 0.0;

        bool isFresh(
            qint64 maximumAgeMS = DefaultMaximumAgeMS,
            const QDateTime& nowUtc = QDateTime::currentDateTimeUtc()) const;
        Assessment assess(
            double acceptableClockErrorMS =
                DefaultAcceptableClockErrorMS) const;
        bool correctedEpochUsecs(
            qint64 localEpochUsecs,
            qint64& correctedEpochUsecs,
            qint64 maximumAgeMS = DefaultMaximumAgeMS) const;
        QDateTime correctedDateTimeUtc(
            const QDateTime& localDateTimeUtc,
            qint64 maximumAgeMS = DefaultMaximumAgeMS) const;
    };

    explicit SystemClockOffset(QObject *parent = nullptr);
    ~SystemClockOffset() override;

    const Measurement& measurement() const { return m_measurement; }
    void start(qint64 intervalMS = DefaultCheckIntervalMS);
    void stop();
    void checkIfStale(qint64 maximumAgeMS = DefaultCheckIntervalMS);

public slots:
    void checkNow();

signals:
    void measurementUpdated(const SystemClockOffset::Measurement& measurement);

private:
    struct NtpSample
    {
        double m_localClockErrorMS = 0.0;
        double m_networkDelayMS = 0.0;
        double m_roundTripMS = 0.0;
        double m_rootDispersionMS = 0.0;
    };

    void startNtpCheck();
    void cleanupNtpCheck();
    void sendNtpProbe();
    void scheduleNextNtpProbe();
    void processNtpReplies();
    void handleNtpProbeTimeout();
    void finishNtpCheck(const QString& error);
    void startHttpCheck(const QString& ntpError);
    void publishMeasurement();

    static QDateTime parseHttpDate(const QByteArray& dateHeader);
    static QByteArray ntpTimestamp(double unixTimeMS);
    static double ntpTimestampToUnixMS(
        const char *data,
        double referenceUnixTimeMS);
    static bool parseNtpReply(
        const QByteArray& reply,
        const QByteArray& expectedOriginateTimestamp,
        double localSendTimeMS,
        double localReceiveTimeMS,
        NtpSample& sample,
        QString& error);

    QNetworkAccessManager *m_networkManager;
    QTimer *m_checkTimer;
    QTimer *m_ntpTimeoutTimer;
    QUdpSocket *m_ntpSocket = nullptr;
    QNetworkReply *m_httpReply = nullptr;
    QElapsedTimer *m_ntpElapsed;
    QByteArray m_ntpTransmitTimestamp;
    double m_ntpLocalSendTimeMS = 0.0;
    int m_ntpAttempts = 0;
    int m_ntpConsecutiveFailures = 0;
    bool m_ntpConnecting = false;
    bool m_ntpProbePending = false;
    QString m_ntpLastError;
    QList<NtpSample> m_ntpSamples;
    Measurement m_measurement;
};

Q_DECLARE_METATYPE(SystemClockOffset::Measurement)

#endif // INCLUDE_SYSTEMCLOCKOFFSET_H
