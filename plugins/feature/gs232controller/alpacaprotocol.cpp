///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
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

#include <limits>

#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QRandomGenerator>
#include <QSharedPointer>
#include <QUrl>
#include <QUrlQuery>
#include <QDateTime>

#include "util/astronomy.h"

#include "alpacaprotocol.h"

MESSAGE_CLASS_DEFINITION(AlpacaProtocol::MsgReportParkState, Message)

namespace
{
    constexpr int kTelescopeDeviceNumber = 0;
    constexpr int kAlpacaCoordinatePrecision = 8;
    constexpr int kSlewDebounceMs = 250;
    constexpr int kSlewRetryMs = 1000;
    constexpr int kAlpacaErrorInvalidOperation = 1279;
}

AlpacaProtocol::AlpacaProtocol() :
    m_networkManager(new QNetworkAccessManager(this)),
    m_clientId(QRandomGenerator::global()->bounded(quint32(1), quint32(std::numeric_limits<quint32>::max()))),
    m_clientTransactionId(1),
    m_connected(false),
    m_connectionPending(false),
    m_capabilitiesReady(false),
    m_capabilitiesPending(false),
    m_canSlewAltAzAsync(false),
    m_canSlewAltAz(false),
    m_canSlewAsync(false),
    m_canSlew(false),
    m_canPark(false),
    m_atPark(false),
    m_atParkValid(false),
    m_atParkQueryPending(false),
    m_slewPending(false),
    m_slewingQueryPending(false),
    m_queuedSlew(false),
    m_queuedAzimuth(0.0f),
    m_queuedElevation(0.0f),
    m_slewRetryTimer(this)
{
    m_slewRetryTimer.setSingleShot(true);
    connect(&m_slewRetryTimer, &QTimer::timeout, this, &AlpacaProtocol::attemptQueuedSlew);
}

AlpacaProtocol::~AlpacaProtocol()
{
}

void AlpacaProtocol::setAzimuthElevation(float azimuth, float elevation)
{
    runWhenConnected([this, azimuth, elevation]() {
        queueSlew(azimuth, elevation);
    });

    ControllerProtocol::setAzimuthElevation(azimuth, elevation);
}

void AlpacaProtocol::readData()
{
}

void AlpacaProtocol::update()
{
    runWhenConnected([this]() {
        pollAzimuthAltitude();
        queryAtPark();
    });
}

void AlpacaProtocol::park()
{
    runWhenConnected([this]() {
        if (!m_canPark)
        {
            reportError("Alpaca telescope driver does not support parking");
            return;
        }

        m_queuedSlew = false;
        m_slewRetryTimer.stop();
        sendSimplePutCommand("park", "Telescope park", [this](bool success) {
            if (success) {
                queryAtPark();
            }
        });
    });
}

void AlpacaProtocol::unpark()
{
    runWhenConnected([this]() {
        sendSimplePutCommand("unpark", "Telescope unpark", [this](bool success) {
            if (success) {
                queryAtPark();
            }
        });
    });
}

void AlpacaProtocol::applySettings(const GS232ControllerSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    const bool endpointChanged = force
        || settingsKeys.contains("host")
        || settingsKeys.contains("port")
        || settingsKeys.contains("protocol");

    if (endpointChanged)
    {
        m_connected = false;
        m_connectionPending = false;
        m_capabilitiesReady = false;
        m_capabilitiesPending = false;
        m_canSlewAltAzAsync = false;
        m_canSlewAltAz = false;
        m_canSlewAsync = false;
        m_canSlew = false;
        m_canPark = false;
        m_atPark = false;
        m_atParkValid = false;
        m_atParkQueryPending = false;
        m_slewPending = false;
        m_slewingQueryPending = false;
        m_queuedSlew = false;
        m_slewRetryTimer.stop();
        m_pendingConnectedContinuations.clear();
    }

    ControllerProtocol::applySettings(settings, settingsKeys, force);
}

QString AlpacaProtocol::baseUrl() const
{
    return QString("http://%1:%2/api/v1/telescope/%3")
        .arg(m_settings.m_host)
        .arg(m_settings.m_port)
        .arg(kTelescopeDeviceNumber);
}

QUrl AlpacaProtocol::deviceUrl(const QString& property) const
{
    return QUrl(baseUrl() + "/" + property);
}

QUrlQuery AlpacaProtocol::transactionQuery()
{
    QUrlQuery query;
    query.addQueryItem("ClientID", QString::number(m_clientId));
    query.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));
    return query;
}

bool AlpacaProtocol::parseAlpacaResponse(QNetworkReply *reply, const QByteArray& payload, QJsonObject& object, const QString& context, bool reportErrors, int *errorNumber)
{
    if (errorNumber) {
        *errorNumber = 0;
    }

    if (reply->error() != QNetworkReply::NoError)
    {
        const QString message = QString("%1 transport error: %2").arg(context).arg(reply->errorString());
        qWarning() << "AlpacaProtocol::parseAlpacaResponse -" << message;
        if (reportErrors) {
            reportError(message);
        }
        return false;
    }

    const QJsonDocument doc = QJsonDocument::fromJson(payload);

    if (!doc.isObject())
    {
        const QString message = QString("%1 returned invalid JSON").arg(context);
        qWarning() << "AlpacaProtocol::parseAlpacaResponse -" << message << payload;
        if (reportErrors) {
            reportError(message);
        }
        return false;
    }

    object = doc.object();
    const int alpacaErrorNumber = object.value("ErrorNumber").toInt(0);

    if (errorNumber) {
        *errorNumber = alpacaErrorNumber;
    }

    if (alpacaErrorNumber != 0)
    {
        const QString errorMessage = object.value("ErrorMessage").toString();
        const QString message = QString("%1 Alpaca error %2: %3").arg(context).arg(alpacaErrorNumber).arg(errorMessage);
        qWarning() << "AlpacaProtocol::parseAlpacaResponse -" << message;
        if (reportErrors) {
            reportError(message);
        }
        return false;
    }

    return true;
}

void AlpacaProtocol::runWhenConnected(const std::function<void()>& continuation)
{
    if (m_connected)
    {
        if (m_capabilitiesReady)
        {
            if (continuation) {
                continuation();
            }
            return;
        }

        if (continuation) {
            m_pendingConnectedContinuations.append(continuation);
        }

        if (!m_capabilitiesPending)
        {
            queryCapabilities([this](bool capabilitiesOk) {
                const auto continuations = m_pendingConnectedContinuations;
                m_pendingConnectedContinuations.clear();

                if (!capabilitiesOk) {
                    return;
                }

                for (const auto& continuation : continuations)
                {
                    if (continuation) {
                        continuation();
                    }
                }
            });
        }
        return;
    }

    if (continuation) {
        m_pendingConnectedContinuations.append(continuation);
    }

    if (m_connectionPending) {
        return;
    }

    m_connectionPending = true;

    setConnected(true, [this](bool success) {
        m_connectionPending = false;

        if (!success)
        {
            m_pendingConnectedContinuations.clear();
            return;
        }

        queryCapabilities([this](bool capabilitiesOk) {
            const auto continuations = m_pendingConnectedContinuations;
            m_pendingConnectedContinuations.clear();

            if (!capabilitiesOk) {
                return;
            }

            for (const auto& continuation : continuations)
            {
                if (continuation) {
                    continuation();
                }
            }
        });
    });
}

void AlpacaProtocol::setConnected(bool connected, const std::function<void(bool)>& continuation)
{
    QUrl url = deviceUrl("connected");
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body = transactionQuery();
    body.addQueryItem("Connected", connected ? "true" : "false");

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    qDebug() << "AlpacaProtocol::setConnected" << url.toString() << payload;

    QNetworkReply *reply = m_networkManager->put(request, payload);

    connect(reply, &QNetworkReply::finished, this, [this, reply, connected, continuation]() {
        const QByteArray payload = reply->readAll();
        QJsonObject response;
        const bool success = parseAlpacaResponse(reply, payload, response, "Telescope connected");

        if (success) {
            m_connected = connected;
        } else if (connected) {
            m_connected = false;
        }

        if (continuation) {
            continuation(success);
        }

        reply->deleteLater();
    });
}

void AlpacaProtocol::queryCapabilities(const std::function<void(bool)>& continuation)
{
    if (m_capabilitiesReady)
    {
        if (continuation) {
            continuation(true);
        }
        return;
    }

    if (m_capabilitiesPending) {
        return;
    }

    struct CapabilityInfo {
        bool canSlewAltAzAsync = false;
        bool canSlewAltAz = false;
        bool canSlewAsync = false;
        bool canSlew = false;
        bool canPark = false;
        bool canSlewAltAzAsyncValid = false;
        bool canSlewAltAzValid = false;
        bool canSlewAsyncValid = false;
        bool canSlewValid = false;
        bool canParkValid = false;
        int pending = 5;
    };

    m_capabilitiesPending = true;

    auto info = QSharedPointer<CapabilityInfo>::create();
    auto checkDone = [this, info, continuation]() {
        info->pending--;

        if (info->pending != 0) {
            return;
        }

        m_capabilitiesPending = false;

        if (!info->canSlewAltAzAsyncValid
            || !info->canSlewAltAzValid
            || !info->canSlewAsyncValid
            || !info->canSlewValid
            || !info->canParkValid)
        {
            reportError("Failed to query Alpaca telescope capabilities");
            if (continuation) {
                continuation(false);
            }
            return;
        }

        m_canSlewAltAzAsync = info->canSlewAltAzAsync;
        m_canSlewAltAz = info->canSlewAltAz;
        m_canSlewAsync = info->canSlewAsync;
        m_canSlew = info->canSlew;
        m_canPark = info->canPark;
        m_capabilitiesReady = true;

        qDebug() << "AlpacaProtocol::queryCapabilities"
                 << "CanSlewAltAzAsync" << m_canSlewAltAzAsync
                 << "CanSlewAltAz" << m_canSlewAltAz
                 << "CanSlewAsync" << m_canSlewAsync
                 << "CanSlew" << m_canSlew
                 << "CanPark" << m_canPark;

        queryAtPark();
        reportParkState();

        if (continuation) {
            continuation(true);
        }
    };

    getBoolProperty("canslewaltazasync", &info->canSlewAltAzAsync, &info->canSlewAltAzAsyncValid, checkDone);
    getBoolProperty("canslewaltaz", &info->canSlewAltAz, &info->canSlewAltAzValid, checkDone);
    getBoolProperty("canslewasync", &info->canSlewAsync, &info->canSlewAsyncValid, checkDone);
    getBoolProperty("canslew", &info->canSlew, &info->canSlewValid, checkDone);
    getBoolProperty("canpark", &info->canPark, &info->canParkValid, checkDone);
}

void AlpacaProtocol::getBoolProperty(const QString& property, bool *value, bool *valid, const std::function<void()>& checkDone)
{
    QUrl url = deviceUrl(property);
    url.setQuery(transactionQuery());

    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));

    connect(reply, &QNetworkReply::finished, this, [this, reply, property, value, valid, checkDone]() {
        const QByteArray payload = reply->readAll();
        QJsonObject response;

        if (parseAlpacaResponse(reply, payload, response, QString("Telescope %1").arg(property), false))
        {
            *value = response.value("Value").toBool();
            *valid = true;
        }

        if (checkDone) {
            checkDone();
        }

        reply->deleteLater();
    });
}

void AlpacaProtocol::slewToAltAz(float azimuth, float elevation)
{
    queueSlew(azimuth, elevation);
}

void AlpacaProtocol::queueSlew(float azimuth, float elevation)
{
    m_queuedAzimuth = azimuth;
    m_queuedElevation = elevation;
    m_queuedSlew = true;
    scheduleSlewAttempt(kSlewDebounceMs);
}

void AlpacaProtocol::scheduleSlewAttempt(int delayMs)
{
    m_slewRetryTimer.start(delayMs);
}

void AlpacaProtocol::attemptQueuedSlew()
{
    if (!m_queuedSlew || m_slewPending || m_slewingQueryPending) {
        return;
    }

    querySlewing([this](bool success, bool slewing) {
        if (!m_queuedSlew) {
            return;
        }

        if (success && slewing)
        {
            scheduleSlewAttempt(kSlewRetryMs);
            return;
        }

        const float azimuth = m_queuedAzimuth;
        const float elevation = m_queuedElevation;
        m_queuedSlew = false;
        dispatchSlew(azimuth, elevation);
    });
}

void AlpacaProtocol::querySlewing(const std::function<void(bool, bool)>& continuation)
{
    QUrl url = deviceUrl("slewing");
    url.setQuery(transactionQuery());

    m_slewingQueryPending = true;
    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));

    connect(reply, &QNetworkReply::finished, this, [this, reply, continuation]() {
        const QByteArray payload = reply->readAll();
        QJsonObject response;
        const bool success = parseAlpacaResponse(reply, payload, response, "Telescope slewing", false);
        const bool slewing = success && response.value("Value").toBool();
        m_slewingQueryPending = false;

        if (continuation) {
            continuation(success, slewing);
        }

        reply->deleteLater();
    });
}

void AlpacaProtocol::dispatchSlew(float azimuth, float elevation)
{
    if (m_slewPending) {
        return;
    }

    if (m_atParkValid && m_atPark)
    {
        m_queuedSlew = false;
        reportError("Alpaca telescope is parked. Unpark before slewing.");
        return;
    }

    if (m_canSlewAltAzAsync || m_canSlewAltAz)
    {
        QUrlQuery body;
        body.addQueryItem("Azimuth", QString::number(azimuth, 'f', kAlpacaCoordinatePrecision));
        body.addQueryItem("Altitude", QString::number(elevation, 'f', kAlpacaCoordinatePrecision));

        if (m_canSlewAltAzAsync) {
            sendSlewCommand("slewtoaltazasync", body, "Telescope slewtoaltazasync");
        } else {
            sendSlewCommand("slewtoaltaz", body, "Telescope slewtoaltaz");
        }
    }
    else if (m_canSlewAsync || m_canSlew)
    {
        slewToRaDec(azimuth, elevation, m_canSlewAsync);
    }
    else
    {
        reportError("Alpaca telescope driver does not support Alt/Az or RA/Dec slewing");
    }
}

void AlpacaProtocol::sendSlewCommand(const QString& method, const QUrlQuery& commandBody, const QString& context)
{
    QUrl url = deviceUrl(method);
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body = transactionQuery();
    const auto queryItems = commandBody.queryItems();

    for (const auto& queryItem : queryItems) {
        body.addQueryItem(queryItem.first, queryItem.second);
    }

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    qDebug() << "AlpacaProtocol::sendSlewCommand" << url.toString() << payload;

    m_slewPending = true;
    QNetworkReply *reply = m_networkManager->put(request, payload);

    connect(reply, &QNetworkReply::finished, this, [this, reply, context]() {
        const QByteArray payload = reply->readAll();
        QJsonObject response;
        int errorNumber = 0;
        const bool success = parseAlpacaResponse(reply, payload, response, context, false, &errorNumber);
        m_slewPending = false;

        if (!success && (errorNumber == kAlpacaErrorInvalidOperation))
        {
            qDebug() << "AlpacaProtocol::sendSlewCommand - telescope is moving, will retry queued target";
            m_queuedSlew = true;
            scheduleSlewAttempt(kSlewRetryMs);
        }
        else if (!success)
        {
            const QString errorMessage = response.value("ErrorMessage").toString();
            reportError(errorMessage.isEmpty()
                ? QString("%1 failed").arg(context)
                : QString("%1 Alpaca error %2: %3").arg(context).arg(errorNumber).arg(errorMessage));
        }
        else if (m_queuedSlew)
        {
            scheduleSlewAttempt(kSlewDebounceMs);
        }

        reply->deleteLater();
    });
}

void AlpacaProtocol::sendSimplePutCommand(const QString& method, const QString& context, const std::function<void(bool)>& continuation)
{
    QUrl url = deviceUrl(method);
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    const QByteArray payload = transactionQuery().toString(QUrl::FullyEncoded).toUtf8();
    qDebug() << "AlpacaProtocol::sendSimplePutCommand" << url.toString() << payload;

    QNetworkReply *reply = m_networkManager->put(request, payload);

    connect(reply, &QNetworkReply::finished, this, [this, reply, context, continuation]() {
        const QByteArray payload = reply->readAll();
        QJsonObject response;
        const bool success = parseAlpacaResponse(reply, payload, response, context);

        if (continuation) {
            continuation(success);
        }

        reply->deleteLater();
    });
}

void AlpacaProtocol::slewToRaDec(float azimuth, float elevation, bool asynchronous)
{
    float latitude, longitude;
    getPosition(latitude, longitude);

    AzAlt aa;
    aa.az = azimuth;
    aa.alt = elevation;

    RADec rd = Astronomy::azAltToRaDec(aa, latitude, longitude, QDateTime::currentDateTime());

    QUrlQuery body;
    body.addQueryItem("RightAscension", QString::number(rd.ra, 'f', kAlpacaCoordinatePrecision));
    body.addQueryItem("Declination", QString::number(rd.dec, 'f', kAlpacaCoordinatePrecision));

    if (asynchronous) {
        sendSlewCommand("slewtocoordinatesasync", body, "Telescope slewtocoordinatesasync");
    } else {
        sendSlewCommand("slewtocoordinates", body, "Telescope slewtocoordinates");
    }
}

void AlpacaProtocol::pollAzimuthAltitude()
{
    struct PositionInfo {
        double azimuth = 0.0;
        double altitude = 0.0;
        bool azimuthValid = false;
        bool altitudeValid = false;
        int pending = 2;
    };

    auto info = QSharedPointer<PositionInfo>::create();
    auto checkDone = [this, info]() {
        info->pending--;

        if (info->pending == 0 && info->azimuthValid && info->altitudeValid) {
            reportAzEl((float) info->azimuth, (float) info->altitude);
        }
    };

    QUrl azUrl = deviceUrl("azimuth");
    azUrl.setQuery(transactionQuery());
    QNetworkReply *azReply = m_networkManager->get(QNetworkRequest(azUrl));
    connect(azReply, &QNetworkReply::finished, this, [this, azReply, info, checkDone]() {
        handlePositionReply("azimuth", azReply, info->azimuth, info->azimuthValid, checkDone);
    });

    QUrl altUrl = deviceUrl("altitude");
    altUrl.setQuery(transactionQuery());
    QNetworkReply *altReply = m_networkManager->get(QNetworkRequest(altUrl));
    connect(altReply, &QNetworkReply::finished, this, [this, altReply, info, checkDone]() {
        handlePositionReply("altitude", altReply, info->altitude, info->altitudeValid, checkDone);
    });
}

void AlpacaProtocol::queryAtPark(const std::function<void(bool, bool)>& continuation)
{
    if (m_atParkQueryPending) {
        return;
    }

    QUrl url = deviceUrl("atpark");
    url.setQuery(transactionQuery());

    m_atParkQueryPending = true;
    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));

    connect(reply, &QNetworkReply::finished, this, [this, reply, continuation]() {
        const QByteArray payload = reply->readAll();
        QJsonObject response;
        const bool success = parseAlpacaResponse(reply, payload, response, "Telescope atpark", false);

        if (success)
        {
            m_atPark = response.value("Value").toBool();
            m_atParkValid = true;
            reportParkState();
        }
        else
        {
            m_atParkValid = false;
            reportParkState(false);
        }

        m_atParkQueryPending = false;

        if (continuation) {
            continuation(success, m_atPark);
        }

        reply->deleteLater();
    });
}

void AlpacaProtocol::reportParkState(bool valid)
{
    sendMessage(MsgReportParkState::create(m_canPark, m_atPark, valid && m_atParkValid));
}

void AlpacaProtocol::handlePositionReply(const QString& property, QNetworkReply *reply, double& value, bool& valid, const std::function<void()>& checkDone)
{
    const QByteArray payload = reply->readAll();
    QJsonObject response;

    if (parseAlpacaResponse(reply, payload, response, QString("Telescope %1").arg(property)))
    {
        value = response.value("Value").toDouble();
        valid = true;
    }

    if (checkDone) {
        checkDone();
    }

    reply->deleteLater();
}
