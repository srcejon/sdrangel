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

#include "alpacaprotocol.h"

namespace
{
    constexpr int kTelescopeDeviceNumber = 0;
}

AlpacaProtocol::AlpacaProtocol() :
    m_networkManager(new QNetworkAccessManager(this)),
    m_clientId(QRandomGenerator::global()->bounded(quint32(1), quint32(std::numeric_limits<quint32>::max()))),
    m_clientTransactionId(1),
    m_connected(false),
    m_connectionPending(false),
    m_slewPending(false)
{
}

AlpacaProtocol::~AlpacaProtocol()
{
}

void AlpacaProtocol::setAzimuthElevation(float azimuth, float elevation)
{
    runWhenConnected([this, azimuth, elevation]() {
        slewToAltAz(azimuth, elevation);
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
        m_slewPending = false;
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

bool AlpacaProtocol::parseAlpacaResponse(QNetworkReply *reply, const QByteArray& payload, QJsonObject& object, const QString& context)
{
    if (reply->error() != QNetworkReply::NoError)
    {
        const QString message = QString("%1 transport error: %2").arg(context).arg(reply->errorString());
        qWarning() << "AlpacaProtocol::parseAlpacaResponse -" << message;
        reportError(message);
        return false;
    }

    const QJsonDocument doc = QJsonDocument::fromJson(payload);

    if (!doc.isObject())
    {
        const QString message = QString("%1 returned invalid JSON").arg(context);
        qWarning() << "AlpacaProtocol::parseAlpacaResponse -" << message << payload;
        reportError(message);
        return false;
    }

    object = doc.object();
    const int errorNumber = object.value("ErrorNumber").toInt(0);

    if (errorNumber != 0)
    {
        const QString errorMessage = object.value("ErrorMessage").toString();
        const QString message = QString("%1 Alpaca error %2: %3").arg(context).arg(errorNumber).arg(errorMessage);
        qWarning() << "AlpacaProtocol::parseAlpacaResponse -" << message;
        reportError(message);
        return false;
    }

    return true;
}

void AlpacaProtocol::runWhenConnected(const std::function<void()>& continuation)
{
    if (m_connected)
    {
        if (continuation) {
            continuation();
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

        const auto continuations = m_pendingConnectedContinuations;
        m_pendingConnectedContinuations.clear();

        for (const auto& continuation : continuations)
        {
            if (continuation) {
                continuation();
            }
        }
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

void AlpacaProtocol::slewToAltAz(float azimuth, float elevation)
{
    if (m_slewPending) {
        return;
    }

    QUrl url = deviceUrl("slewtoaltazasync");
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body = transactionQuery();
    body.addQueryItem("Azimuth", QString::number(azimuth, 'f', m_settings.m_precision));
    body.addQueryItem("Altitude", QString::number(elevation, 'f', m_settings.m_precision));

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    qDebug() << "AlpacaProtocol::slewToAltAz" << url.toString() << payload;

    m_slewPending = true;
    QNetworkReply *reply = m_networkManager->put(request, payload);

    connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        const QByteArray payload = reply->readAll();
        QJsonObject response;
        parseAlpacaResponse(reply, payload, response, "Telescope slewtoaltazasync");
        m_slewPending = false;
        reply->deleteLater();
    });
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
