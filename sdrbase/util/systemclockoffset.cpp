///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
// Some code by AI                                                               //
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

#include "systemclockoffset.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include <QElapsedTimer>
#include <QLocale>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QTimer>
#include <QUdpSocket>
#include <QUrl>
#include <QUrlQuery>

namespace {
    constexpr qint64 CheckTimeoutMS = 10 * 1000;
    constexpr qint64 DateHeaderHalfResolutionMS = 500;
    constexpr int NtpPort = 123;
    constexpr int NtpProbeCount = 5;
    constexpr int NtpProbeIntervalMS = 100;
    constexpr int NtpProbeTimeoutMS = 1000;
    constexpr int NtpConnectTimeoutMS = 2000;
    constexpr int NtpInitialFailureLimit = 2;
    constexpr qint64 NtpUnixEpochOffsetS = 2208988800LL;
    constexpr char NtpHost[] = "time.cloudflare.com";
    constexpr char HttpTimeSourceUrl[] =
        "https://www.google.com/generate_204";

    quint32 readUnsignedBigEndian32(const char *data)
    {
        const auto bytes = reinterpret_cast<const unsigned char *>(data);
        return (quint32(bytes[0]) << 24)
            | (quint32(bytes[1]) << 16)
            | (quint32(bytes[2]) << 8)
            | quint32(bytes[3]);
    }

    void writeUnsignedBigEndian32(char *data, quint32 value)
    {
        auto bytes = reinterpret_cast<unsigned char *>(data);
        bytes[0] = (unsigned char) (value >> 24);
        bytes[1] = (unsigned char) (value >> 16);
        bytes[2] = (unsigned char) (value >> 8);
        bytes[3] = (unsigned char) value;
    }
}

bool SystemClockOffset::Measurement::isFresh(
    qint64 maximumAgeMS,
    const QDateTime& nowUtc) const
{
    return m_available
        && m_dateTimeUtc.isValid()
        && nowUtc.isValid()
        && (m_dateTimeUtc.msecsTo(nowUtc) >= 0)
        && (m_dateTimeUtc.msecsTo(nowUtc) <= maximumAgeMS);
}

SystemClockOffset::Assessment SystemClockOffset::Measurement::assess(
    double acceptableClockErrorMS) const
{
    if (!m_available) {
        return Assessment::Unavailable;
    }

    const double absoluteErrorMS = std::abs(m_localClockErrorMS);
    if (absoluteErrorMS + m_uncertaintyMS <= acceptableClockErrorMS) {
        return Assessment::Acceptable;
    }
    if (absoluteErrorMS - m_uncertaintyMS > acceptableClockErrorMS) {
        return Assessment::Unacceptable;
    }
    return Assessment::Inconclusive;
}

bool SystemClockOffset::Measurement::correctedEpochUsecs(
    qint64 localEpochUsecs,
    qint64& correctedEpochUsecs,
    qint64 maximumAgeMS) const
{
    if (!isFresh(maximumAgeMS)) {
        return false;
    }

    correctedEpochUsecs = localEpochUsecs
        - (qint64) std::llround(m_localClockErrorMS * 1000.0);
    return true;
}

QDateTime SystemClockOffset::Measurement::correctedDateTimeUtc(
    const QDateTime& localDateTimeUtc,
    qint64 maximumAgeMS) const
{
    if (!localDateTimeUtc.isValid() || !isFresh(maximumAgeMS)) {
        return {};
    }

    return localDateTimeUtc.toUTC().addMSecs(
        -(qint64) std::llround(m_localClockErrorMS));
}

SystemClockOffset::SystemClockOffset(QObject *parent) :
    QObject(parent),
    m_networkManager(new QNetworkAccessManager(this)),
    m_checkTimer(new QTimer(this)),
    m_ntpTimeoutTimer(new QTimer(this)),
    m_ntpElapsed(new QElapsedTimer())
{
    qRegisterMetaType<SystemClockOffset::Measurement>();
    m_checkTimer->setInterval(DefaultCheckIntervalMS);
    connect(m_checkTimer, &QTimer::timeout, this, &SystemClockOffset::checkNow);
    m_ntpTimeoutTimer->setSingleShot(true);
    connect(
        m_ntpTimeoutTimer,
        &QTimer::timeout,
        this,
        &SystemClockOffset::handleNtpProbeTimeout);
}

SystemClockOffset::~SystemClockOffset()
{
    stop();
    delete m_ntpElapsed;
}

void SystemClockOffset::start(qint64 intervalMS)
{
    m_checkTimer->setInterval(
        (int) std::min<qint64>(
            std::numeric_limits<int>::max(),
            std::max<qint64>(1000, intervalMS)));
    m_checkTimer->start();
    checkIfStale(0);
}

void SystemClockOffset::stop()
{
    m_checkTimer->stop();
    m_ntpTimeoutTimer->stop();
    cleanupNtpCheck();

    if (m_httpReply)
    {
        m_httpReply->disconnect(this);
        m_httpReply->abort();
        m_httpReply->deleteLater();
        m_httpReply = nullptr;
    }

    m_measurement.m_pending = false;
}

void SystemClockOffset::checkIfStale(qint64 maximumAgeMS)
{
    if (!m_measurement.m_pending
        && (!m_measurement.m_dateTimeUtc.isValid()
            || !m_measurement.isFresh(maximumAgeMS)))
    {
        checkNow();
    }
}

void SystemClockOffset::checkNow()
{
    if (m_measurement.m_pending) {
        return;
    }

    m_measurement.m_pending = true;
    m_measurement.m_error.clear();
    m_measurement.m_timeSource = QStringLiteral("SNTP: %1").arg(
        QString::fromLatin1(NtpHost));
    publishMeasurement();
    startNtpCheck();
}

void SystemClockOffset::startNtpCheck()
{
    cleanupNtpCheck();
    m_ntpAttempts = 0;
    m_ntpConsecutiveFailures = 0;
    m_ntpConnecting = true;
    m_ntpLastError.clear();
    m_ntpSamples.clear();
    m_ntpSocket = new QUdpSocket(this);

    connect(m_ntpSocket, &QUdpSocket::connected, this, [this]() {
        if (!m_measurement.m_pending || !m_ntpSocket) {
            return;
        }

        m_ntpTimeoutTimer->stop();
        m_ntpConnecting = false;
        sendNtpProbe();
    });
    connect(
        m_ntpSocket,
        &QUdpSocket::readyRead,
        this,
        &SystemClockOffset::processNtpReplies);
    connect(
        m_ntpSocket,
        &QUdpSocket::errorOccurred,
        this,
        [this](QAbstractSocket::SocketError) {
            if (m_measurement.m_pending && m_ntpSocket) {
                finishNtpCheck(m_ntpSocket->errorString());
            }
        });

    m_ntpSocket->connectToHost(QString::fromLatin1(NtpHost), NtpPort);
    m_ntpTimeoutTimer->start(NtpConnectTimeoutMS);
}

void SystemClockOffset::cleanupNtpCheck()
{
    m_ntpTimeoutTimer->stop();

    if (m_ntpSocket)
    {
        m_ntpSocket->disconnect(this);
        m_ntpSocket->abort();
        m_ntpSocket->deleteLater();
        m_ntpSocket = nullptr;
    }

    m_ntpConnecting = false;
    m_ntpProbePending = false;
}

void SystemClockOffset::sendNtpProbe()
{
    if (!m_measurement.m_pending || !m_ntpSocket) {
        return;
    }

    ++m_ntpAttempts;
    QByteArray packet(48, '\0');
    packet[0] = 0x23; // LI = 0, version = 4, client mode.
    m_ntpLocalSendTimeMS = (double) QDateTime::currentMSecsSinceEpoch();
    m_ntpTransmitTimestamp = ntpTimestamp(m_ntpLocalSendTimeMS);
    std::copy(
        m_ntpTransmitTimestamp.cbegin(),
        m_ntpTransmitTimestamp.cend(),
        packet.begin() + 40);
    m_ntpElapsed->start();

    if (m_ntpSocket->write(packet) != packet.size())
    {
        m_ntpProbePending = false;
        m_ntpLastError = m_ntpSocket->errorString();
        ++m_ntpConsecutiveFailures;
        scheduleNextNtpProbe();
        return;
    }

    m_ntpProbePending = true;
    m_ntpTimeoutTimer->start(NtpProbeTimeoutMS);
}

void SystemClockOffset::scheduleNextNtpProbe()
{
    if (!m_measurement.m_pending || !m_ntpSocket) {
        return;
    }

    if (m_ntpAttempts >= NtpProbeCount)
    {
        finishNtpCheck(m_ntpLastError);
        return;
    }

    QUdpSocket *socket = m_ntpSocket;
    QTimer::singleShot(NtpProbeIntervalMS, this, [this, socket]() {
        if (m_ntpSocket == socket) {
            sendNtpProbe();
        }
    });
}

void SystemClockOffset::processNtpReplies()
{
    if (!m_measurement.m_pending || !m_ntpSocket) {
        return;
    }

    while (m_ntpSocket->hasPendingDatagrams())
    {
        QByteArray reply;
        reply.resize((int) m_ntpSocket->pendingDatagramSize());
        if (m_ntpSocket->readDatagram(reply.data(), reply.size()) < 0) {
            continue;
        }
        if (!m_ntpProbePending) {
            continue;
        }

        NtpSample sample;
        QString error;
        const double localReceiveTimeMS =
            (double) QDateTime::currentMSecsSinceEpoch();
        if (!parseNtpReply(
                reply,
                m_ntpTransmitTimestamp,
                m_ntpLocalSendTimeMS,
                localReceiveTimeMS,
                sample,
                error))
        {
            m_ntpLastError = error;
            continue;
        }

        m_ntpTimeoutTimer->stop();
        m_ntpProbePending = false;
        sample.m_roundTripMS =
            (double) m_ntpElapsed->nsecsElapsed() / 1000000.0;
        m_ntpSamples.append(sample);
        m_ntpConsecutiveFailures = 0;
        m_ntpLastError.clear();
        scheduleNextNtpProbe();
        return;
    }
}

void SystemClockOffset::handleNtpProbeTimeout()
{
    if (!m_measurement.m_pending || !m_ntpSocket) {
        return;
    }

    if (m_ntpConnecting)
    {
        finishNtpCheck(QStringLiteral("SNTP connection timed out"));
        return;
    }

    m_ntpProbePending = false;
    ++m_ntpConsecutiveFailures;
    m_ntpLastError = QStringLiteral("SNTP request timed out");
    if (m_ntpSamples.isEmpty()
        && (m_ntpConsecutiveFailures >= NtpInitialFailureLimit))
    {
        finishNtpCheck(m_ntpLastError);
        return;
    }

    scheduleNextNtpProbe();
}

void SystemClockOffset::finishNtpCheck(const QString& error)
{
    cleanupNtpCheck();

    if (m_ntpSamples.isEmpty())
    {
        startHttpCheck(
            error.isEmpty()
                ? QStringLiteral("SNTP did not return a usable response")
                : error);
        return;
    }

    std::sort(
        m_ntpSamples.begin(),
        m_ntpSamples.end(),
        [](const NtpSample& left, const NtpSample& right) {
            return left.m_networkDelayMS < right.m_networkDelayMS;
        });
    const int selectedCount = std::min<int>(3, (int) m_ntpSamples.size());
    QList<double> selectedErrors;
    selectedErrors.reserve(selectedCount);
    double rootDispersionMS = 0.0;

    for (int i = 0; i < selectedCount; ++i)
    {
        selectedErrors.append(m_ntpSamples[i].m_localClockErrorMS);
        rootDispersionMS = std::max(
            rootDispersionMS,
            m_ntpSamples[i].m_rootDispersionMS);
    }

    std::sort(selectedErrors.begin(), selectedErrors.end());
    const int middle = selectedCount / 2;
    m_measurement.m_localClockErrorMS = (selectedCount % 2)
        ? selectedErrors[middle]
        : 0.5 * (selectedErrors[middle - 1] + selectedErrors[middle]);
    double sampleSpreadMS = 0.0;
    for (double sampleErrorMS : selectedErrors) {
        sampleSpreadMS = std::max(
            sampleSpreadMS,
            std::abs(
                sampleErrorMS - m_measurement.m_localClockErrorMS));
    }

    const NtpSample& bestSample = m_ntpSamples.front();
    m_measurement.m_roundTripMS = bestSample.m_roundTripMS;
    m_measurement.m_uncertaintyMS = std::max(
        1.0,
        0.5 * bestSample.m_networkDelayMS
            + sampleSpreadMS
            + rootDispersionMS);
    m_measurement.m_dateTimeUtc = QDateTime::currentDateTimeUtc();
    m_measurement.m_available = true;
    m_measurement.m_pending = false;
    m_measurement.m_error.clear();
    publishMeasurement();
}

void SystemClockOffset::startHttpCheck(const QString& ntpError)
{
    const QDateTime localStartUtc = QDateTime::currentDateTimeUtc();
    const auto elapsed = std::make_shared<QElapsedTimer>();
    elapsed->start();
    m_measurement.m_timeSource = QStringLiteral("HTTPS fallback: %1").arg(
        QString::fromLatin1(HttpTimeSourceUrl));

    QUrl clockUrl(QString::fromLatin1(HttpTimeSourceUrl));
    QUrlQuery clockQuery;
    clockQuery.addQueryItem(
        QStringLiteral("sdrangel-clock-check"),
        QString::number(localStartUtc.toMSecsSinceEpoch()));
    clockUrl.setQuery(clockQuery);
    QNetworkRequest request(clockUrl);
    request.setHeader(
        QNetworkRequest::UserAgentHeader,
        QStringLiteral("SDRangel clock check"));
    request.setRawHeader("Cache-Control", "no-cache");
    request.setAttribute(
        QNetworkRequest::RedirectPolicyAttribute,
        QNetworkRequest::NoLessSafeRedirectPolicy);
    request.setAttribute(
        QNetworkRequest::CacheLoadControlAttribute,
        QNetworkRequest::AlwaysNetwork);
    request.setTransferTimeout(CheckTimeoutMS);

    m_httpReply = m_networkManager->head(request);
    connect(
        m_httpReply,
        &QNetworkReply::finished,
        this,
        [this, localStartUtc, elapsed, ntpError]() {
            QNetworkReply *reply = m_httpReply;
            m_httpReply = nullptr;
            if (!reply) {
                return;
            }

            const qint64 roundTripMS = elapsed->elapsed();
            const int httpStatus = reply->attribute(
                QNetworkRequest::HttpStatusCodeAttribute).toInt();
            const QNetworkReply::NetworkError networkError = reply->error();
            const QString networkErrorText = reply->errorString();
            const QByteArray dateHeader = reply->rawHeader("Date");
            reply->deleteLater();

            m_measurement.m_pending = false;
            QString failure;

            if ((networkError != QNetworkReply::NoError)
                || (httpStatus < 200)
                || (httpStatus >= 400))
            {
                const QString httpError = networkError
                    != QNetworkReply::NoError
                        ? networkErrorText
                        : QStringLiteral(
                            "time request returned HTTP %1").arg(httpStatus);
                failure = QStringLiteral(
                    "SNTP failed (%1); HTTPS fallback failed (%2)")
                        .arg(ntpError, httpError);
            }
            else
            {
                const QDateTime internetTimeUtc = parseHttpDate(dateHeader);
                if (!internetTimeUtc.isValid())
                {
                    const QString httpError = dateHeader.isEmpty()
                        ? QStringLiteral(
                            "time response did not contain a Date header")
                        : QStringLiteral(
                            "time response contained an invalid Date header");
                    failure = QStringLiteral(
                        "SNTP failed (%1); HTTPS fallback failed (%2)")
                            .arg(ntpError, httpError);
                }
                else
                {
                    const QDateTime localMidpointUtc =
                        localStartUtc.addMSecs(roundTripMS / 2);
                    m_measurement.m_localClockErrorMS = (double)
                        internetTimeUtc.addMSecs(DateHeaderHalfResolutionMS)
                            .msecsTo(localMidpointUtc);
                    m_measurement.m_dateTimeUtc =
                        QDateTime::currentDateTimeUtc();
                    m_measurement.m_roundTripMS = (double) roundTripMS;
                    m_measurement.m_uncertaintyMS =
                        0.5 * (double) roundTripMS
                            + DateHeaderHalfResolutionMS;
                    m_measurement.m_available = true;
                    m_measurement.m_error.clear();
                }
            }

            if (!failure.isEmpty())
            {
                if (m_lastSuccessfulMeasurement.m_available)
                {
                    m_measurement = m_lastSuccessfulMeasurement;
                    m_measurement.m_pending = false;
                    m_measurement.m_error = failure;
                }
                else
                {
                    m_measurement.m_dateTimeUtc =
                        QDateTime::currentDateTimeUtc();
                    m_measurement.m_roundTripMS = (double) roundTripMS;
                    m_measurement.m_uncertaintyMS =
                        0.5 * (double) roundTripMS
                            + DateHeaderHalfResolutionMS;
                    m_measurement.m_available = false;
                    m_measurement.m_error = failure;
                }
            }

            publishMeasurement();
        });
}

void SystemClockOffset::publishMeasurement()
{
    if (m_measurement.m_available
        && !m_measurement.m_pending
        && m_measurement.m_error.isEmpty())
    {
        m_lastSuccessfulMeasurement = m_measurement;
    }

    emit measurementUpdated(m_measurement);
}

QDateTime SystemClockOffset::parseHttpDate(const QByteArray& dateHeader)
{
    const QString dateText = QString::fromLatin1(dateHeader).trimmed();
    QDateTime dateTime = QDateTime::fromString(dateText, Qt::RFC2822Date);

    if (dateTime.isValid()) {
        return dateTime.toUTC();
    }

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

QByteArray SystemClockOffset::ntpTimestamp(double unixTimeMS)
{
    QByteArray timestamp(8, '\0');
    const double unixTimeS = unixTimeMS / 1000.0;
    const qint64 wholeUnixSeconds = (qint64) std::floor(unixTimeS);
    const double fractionalSecond = unixTimeS - wholeUnixSeconds;
    const quint64 ntpSeconds =
        quint64(wholeUnixSeconds + NtpUnixEpochOffsetS);
    const quint32 ntpFraction = (quint32) std::llround(
        fractionalSecond * 4294967296.0);
    writeUnsignedBigEndian32(timestamp.data(), (quint32) ntpSeconds);
    writeUnsignedBigEndian32(timestamp.data() + 4, ntpFraction);
    return timestamp;
}

double SystemClockOffset::ntpTimestampToUnixMS(
    const char *data,
    double referenceUnixTimeMS)
{
    const quint32 rawSeconds = readUnsignedBigEndian32(data);
    const quint32 fraction = readUnsignedBigEndian32(data + 4);
    const qint64 referenceNtpSeconds =
        (qint64) std::floor(referenceUnixTimeMS / 1000.0)
        + NtpUnixEpochOffsetS;
    qint64 seconds = (referenceNtpSeconds & ~0xffffffffLL) | rawSeconds;

    if (seconds - referenceNtpSeconds > 0x80000000LL) {
        seconds -= 0x100000000LL;
    } else if (referenceNtpSeconds - seconds > 0x80000000LL) {
        seconds += 0x100000000LL;
    }

    return (double) (seconds - NtpUnixEpochOffsetS) * 1000.0
        + (double) fraction * (1000.0 / 4294967296.0);
}

bool SystemClockOffset::parseNtpReply(
    const QByteArray& reply,
    const QByteArray& expectedOriginateTimestamp,
    double localSendTimeMS,
    double localReceiveTimeMS,
    NtpSample& sample,
    QString& error)
{
    if (reply.size() < 48)
    {
        error = QStringLiteral("short SNTP response");
        return false;
    }

    const unsigned char flags = (unsigned char) reply[0];
    const int leapIndicator = flags >> 6;
    const int version = (flags >> 3) & 0x7;
    const int mode = flags & 0x7;
    const int stratum = (unsigned char) reply[1];

    if ((leapIndicator == 3) || (version < 3) || (mode != 4)
        || (stratum < 1) || (stratum > 15))
    {
        error = QStringLiteral("invalid SNTP server response");
        return false;
    }

    if (reply.mid(24, 8) != expectedOriginateTimestamp)
    {
        error = QStringLiteral("SNTP response did not match the request");
        return false;
    }

    const QByteArray receiveTimestamp = reply.mid(32, 8);
    const QByteArray transmitTimestamp = reply.mid(40, 8);
    if ((receiveTimestamp == QByteArray(8, '\0'))
        || (transmitTimestamp == QByteArray(8, '\0')))
    {
        error = QStringLiteral("SNTP response omitted server timestamps");
        return false;
    }

    const double serverReceiveTimeMS = ntpTimestampToUnixMS(
        reply.constData() + 32,
        localReceiveTimeMS);
    const double serverTransmitTimeMS = ntpTimestampToUnixMS(
        reply.constData() + 40,
        localReceiveTimeMS);
    const double serverProcessingMS =
        serverTransmitTimeMS - serverReceiveTimeMS;
    const double localRoundTripMS = localReceiveTimeMS - localSendTimeMS;

    if (!std::isfinite(serverReceiveTimeMS)
        || !std::isfinite(serverTransmitTimeMS)
        || (serverProcessingMS < -1.0)
        || (localRoundTripMS < 0.0))
    {
        error = QStringLiteral("SNTP response timestamps were inconsistent");
        return false;
    }

    const double serverMinusLocalMS =
        ((serverReceiveTimeMS - localSendTimeMS)
            + (serverTransmitTimeMS - localReceiveTimeMS))
        * 0.5;
    sample.m_localClockErrorMS = -serverMinusLocalMS;
    sample.m_networkDelayMS = std::max(
        0.0,
        localRoundTripMS - serverProcessingMS);
    sample.m_rootDispersionMS =
        (double) readUnsignedBigEndian32(reply.constData() + 8)
        * (1000.0 / 65536.0);
    return true;
}
