///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
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

#include "cameraalpacacontroller.h"

#include <algorithm>
#include <cstring>
#include <limits>

#include <QDebug>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaEnum>
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QRandomGenerator>
#include <QSharedPointer>
#include <QTimer>
#include <QUrlQuery>
#include <QtEndian>

namespace {

constexpr int alpacaDefaultNetworkTimeoutMs = 15000;
constexpr int alpacaImageArrayNetworkTimeoutMs = 60000;

void installAlpacaReplyTimeout(QNetworkReply *reply, int timeoutMs, const QString& operation)
{
    if (!reply || (timeoutMs <= 0)) {
        return;
    }

    QTimer *timer = new QTimer(reply);
    timer->setSingleShot(true);
    QObject::connect(timer, &QTimer::timeout, reply, [reply, timeoutMs, operation]() {
        if (reply->isRunning())
        {
            qWarning() << "CameraAlpacaController:" << operation
                       << "timed out after" << timeoutMs << "ms"
                       << reply->request().url().toString();
            reply->abort();
        }
    });
    QObject::connect(reply, &QNetworkReply::finished, timer, &QObject::deleteLater);
    timer->start(timeoutMs);
}

}

CameraAlpacaController::CameraAlpacaController() :
    m_frameRequestPending(false),
    m_clientId(QRandomGenerator::global()->bounded(quint32(1), quint32(std::numeric_limits<quint32>::max()))),
    m_clientTransactionId(1),
    m_sensorType(0),
    m_cameraSizeX(0),
    m_cameraSizeY(0),
    m_bayerOffsetX(0),
    m_bayerOffsetY(0),
    m_imageBytesSupported(true),
    m_lastErrorNumber(0),
    m_lastErrorMessage(),
    m_lastReceiveImageFormat(),
    m_connected(false),
    m_connectionPending(false),
    m_focuserConnected(false),
    m_focuserConnectionPending(false),
    m_filterWheelConnected(false),
    m_filterWheelConnectionPending(false),
    m_bootstrapPending(false),
    m_paramsInitialized(false),
    m_exposureSeenActive(false),
    m_lastBinX(0),
    m_lastBinY(0),
    m_lastNumX(0),
    m_lastNumY(0),
    m_lastEffectiveNumX(-1),
    m_lastEffectiveNumY(-1),
    m_lastStartX(0),
    m_lastStartY(0),
    m_lastGain(-1),
    m_lastOffset(-1),
    m_lastReadoutMode(0),
    m_exposureMinMs(0.001),
    m_exposureMaxMs(60000.0),
    m_lastCaptureTimeMs(-1)
{
}

void CameraAlpacaController::resetConnectionState()
{
    m_connected = false;
    m_connectionPending = false;
    m_pendingConnectedContinuations.clear();
    m_pendingConnectedFailureContinuations.clear();
    m_bootstrapPending = false;
    m_pendingBootstrapContinuations.clear();
    setLastError(0, QString());
}

void CameraAlpacaController::resetFocuserConnectionState()
{
    m_focuserConnected = false;
    m_focuserConnectionPending = false;
    m_pendingFocuserConnectedContinuations.clear();
}

void CameraAlpacaController::resetFilterWheelConnectionState()
{
    m_filterWheelConnected = false;
    m_filterWheelConnectionPending = false;
    m_pendingFilterWheelConnectedContinuations.clear();
}

void CameraAlpacaController::setLastError(int errorNumber, const QString& errorMessage)
{
    m_lastErrorNumber = errorNumber;
    m_lastErrorMessage = errorMessage;
}

void CameraAlpacaController::resetCaptureState()
{
    m_lastCaptureTimeMs = -1;
    m_captureTimer.invalidate();
    m_paramsInitialized = false;
}

void CameraAlpacaController::runPendingContinuations(QVector<std::function<void()>>& continuations)
{
    const auto pendingContinuations = std::move(continuations);
    continuations.clear();

    for (const auto& continuation : pendingContinuations)
    {
        if (continuation) {
            continuation();
        }
    }
}

void CameraAlpacaController::logRequest(const CameraSettings& settings, const QString& method, const QUrl& url, const QByteArray& payload) const
{
    if (!settings.m_alpacaApiLogEnabled) {
        return;
    }

    if (payload.isEmpty()) {
        qDebug() << "CameraAlpacaController::AlpacaAPI request" << method << url.toString();
    } else {
        qDebug() << "CameraAlpacaController::AlpacaAPI request" << method << url.toString() << payload;
    }
}

void CameraAlpacaController::logResponse(const CameraSettings& settings, const QString& method, const QUrl& url,
    QNetworkReply *reply, const QByteArray& payload, bool updateLastError)
{
    const QString path = url.path();

    int alpacaErrorNumber = 0;
    QString alpacaErrorMessage;
    const bool alpacaPayloadParsed = parseErrorPayload(payload, alpacaErrorNumber, alpacaErrorMessage);
    const bool optionalCapabilityUnavailable = alpacaPayloadParsed
        && (alpacaErrorNumber != 0)
        && isOptionalCapabilityPath(path);

    if (updateLastError)
    {
        if (reply->error() != QNetworkReply::NoError)
        {
            m_lastErrorNumber = static_cast<int>(reply->error());
            m_lastErrorMessage = reply->errorString();
        }
        else if (alpacaPayloadParsed && !optionalCapabilityUnavailable)
        {
            m_lastErrorNumber = alpacaErrorNumber;
            m_lastErrorMessage = alpacaErrorMessage;
        }
    }

    if (!settings.m_alpacaApiLogEnabled) {
        return;
    }

    if (path.endsWith(QStringLiteral("/imagearray"), Qt::CaseInsensitive))
    {
        qDebug() << "CameraAlpacaController::AlpacaAPI response" << method << url.toString()
                 << transportError(reply)
                 << "bytes" << payload.size()
                 << "contentType" << reply->header(QNetworkRequest::ContentTypeHeader).toString();
        return;
    }

    if (alpacaPayloadParsed)
    {
        if (optionalCapabilityUnavailable)
        {
            qDebug() << "CameraAlpacaController::AlpacaAPI optional capability unavailable" << method << url.toString()
                     << transportError(reply)
                     << "alpacaError" << alpacaErrorNumber << alpacaErrorMessage;
        }
        else
        {
            qDebug() << "CameraAlpacaController::AlpacaAPI response" << method << url.toString()
                     << transportError(reply)
                     << "alpacaError" << alpacaErrorNumber << alpacaErrorMessage
                     << payload;
        }
    }
    else
    {
        qDebug() << "CameraAlpacaController::AlpacaAPI response" << method << url.toString()
                 << transportError(reply)
                 << payload;
    }
}

void CameraAlpacaController::disconnectCamera(QNetworkAccessManager *networkManager, const CameraSettings& settings)
{
    if (!networkManager) {
        return;
    }

    const QString baseUrlString = baseUrl(settings);
    const int cameraId = settings.cameraIdInt();
    QUrl url(baseUrlString + QString("/api/v1/camera/%1/connected").arg(cameraId));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem("Connected", QStringLiteral("false"));
    body.addQueryItem("ClientID", QString::number(m_clientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    logRequest(settings, "PUT", url, payload);

    QNetworkReply *reply = networkManager->put(request, payload);
    installAlpacaReplyTimeout(reply, alpacaDefaultNetworkTimeoutMs, QStringLiteral("PUT disconnect camera"));
    QObject::connect(reply, &QNetworkReply::finished, reply, [this, settings, reply]() {
        const QByteArray responseBody = reply->readAll();
        logResponse(settings, "PUT", reply->request().url(), reply, responseBody, false);
        reply->deleteLater();
    });
}

void CameraAlpacaController::setConnected(QNetworkAccessManager *networkManager, const CameraSettings& settings, bool connected,
    std::function<void()> reportStatus, std::function<void()> continuation, std::function<void()> onFailure)
{
    if (!networkManager)
    {
        if (onFailure) {
            onFailure();
        }
        return;
    }

    const QString baseUrlString = baseUrl(settings);
    const int cameraId = settings.cameraIdInt();
    QUrl url(baseUrlString + QString("/api/v1/camera/%1/connected").arg(cameraId));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem("Connected", connected ? QStringLiteral("true") : QStringLiteral("false"));
    body.addQueryItem("ClientID", QString::number(m_clientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    logRequest(settings, "PUT", url, payload);

    QNetworkReply *reply = networkManager->put(request, payload);
    installAlpacaReplyTimeout(reply, alpacaDefaultNetworkTimeoutMs, QStringLiteral("PUT connected"));

    QObject::connect(reply, &QNetworkReply::finished, reply, [this, settings, reply, connected, reportStatus, continuation, onFailure]() {
        const QByteArray responseBody = reply->readAll();
        logResponse(settings, "PUT", reply->request().url(), reply, responseBody);

        bool success = false;

        if (reply->error() == QNetworkReply::NoError)
        {
            const QJsonDocument doc = QJsonDocument::fromJson(responseBody);

            if (doc.isObject()) {
                success = (doc.object().value("ErrorNumber").toInt(-1) == 0);
            }
        }

        if (success)
        {
            m_connected = connected;
        }
        else if (connected)
        {
            if (reply->error() != QNetworkReply::NoError) {
                setLastError(static_cast<int>(reply->error()), reply->errorString());
            }
            m_connected = false;
            m_frameRequestPending = false;
            if (reportStatus) {
                reportStatus();
            }
        }
        else
        {
            if (reply->error() != QNetworkReply::NoError) {
                setLastError(static_cast<int>(reply->error()), reply->errorString());
            }
            if (reportStatus) {
                reportStatus();
            }
        }

        if (!connected) {
            m_connectionPending = false;
            m_connected = false;
            m_pendingConnectedContinuations.clear();
        }

        if (success)
        {
            if (continuation) {
                continuation();
            }
        }
        else if (connected)
        {
            m_connectionPending = false;
            m_pendingConnectedContinuations.clear();
            if (onFailure) {
                onFailure();
            }
        }

        reply->deleteLater();
    });
}

void CameraAlpacaController::runWhenConnected(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    std::function<void()> reportStatus, std::function<void()> continuation, std::function<void()> onFailure)
{
    if (!networkManager)
    {
        if (onFailure) {
            onFailure();
        }
        return;
    }

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
    if (onFailure) {
        m_pendingConnectedFailureContinuations.append(onFailure);
    }

    if (m_connectionPending) {
        return;
    }

    m_connectionPending = true;
    setConnected(networkManager, settings, true, reportStatus, [this]() {
        m_connectionPending = false;
        runPendingContinuations(m_pendingConnectedContinuations);
        m_pendingConnectedFailureContinuations.clear();
    }, [this]() {
        m_connectionPending = false;
        m_pendingConnectedContinuations.clear();
        runPendingContinuations(m_pendingConnectedFailureContinuations);
    });
}

void CameraAlpacaController::bootstrap(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    std::function<void()> reportStatus,
    std::function<void()> onConnected,
    std::function<void(const CapabilitiesReport&)> onCapabilities,
    std::function<void(const StatusReport&)> onStatus,
    std::function<void()> onComplete,
    std::function<void()> continuation)
{
    if (!networkManager)
    {
        setLastError(-1, QStringLiteral("No network manager"));
        if (reportStatus) {
            reportStatus();
        }
        if (continuation) {
            continuation();
        }
        return;
    }

    if (continuation) {
        m_pendingBootstrapContinuations.append(continuation);
    }

    if (m_bootstrapPending) {
        return;
    }

    m_bootstrapPending = true;

    runWhenConnected(networkManager, settings, reportStatus, [this, networkManager, settings, onConnected, onCapabilities, onStatus, onComplete]() {
        if (onConnected) {
            onConnected();
        }

        queryCameraCapabilities(networkManager, settings, [this, networkManager, settings, onCapabilities, onStatus, onComplete](const CapabilitiesReport& report) {
            if (onCapabilities) {
                onCapabilities(report);
            }

            pollStatus(networkManager, settings, [this, onStatus, onComplete](const StatusReport& status) {
                if (onStatus) {
                    onStatus(status);
                }

                m_bootstrapPending = false;
                runPendingContinuations(m_pendingBootstrapContinuations);

                if (onComplete) {
                    onComplete();
                }
            });
        });
    }, [this, reportStatus]() {
        m_bootstrapPending = false;
        m_pendingBootstrapContinuations.clear();
        if (reportStatus) {
            reportStatus();
        }
    });
}

void CameraAlpacaController::setFocuserConnected(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    bool connected, std::function<void()> reportStatus, std::function<void()> continuation)
{
    if (!networkManager || !settings.isAlpacaCamera() || !settings.m_alpacaFocuserEnabled) {
        return;
    }

    const QString baseUrlString = focuserBaseUrl(settings);
    const int deviceNumber = std::max(0, settings.m_alpacaFocuserDeviceNumber);
    QUrl url(baseUrlString + QString("/api/v1/focuser/%1/connected").arg(deviceNumber));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem("Connected", connected ? "true" : "false");
    body.addQueryItem("ClientID", QString::number(m_clientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    logRequest(settings, "PUT", url, payload);

    QNetworkReply *reply = networkManager->put(request, payload);
    installAlpacaReplyTimeout(reply, alpacaDefaultNetworkTimeoutMs, QStringLiteral("PUT focuser connected"));

    QObject::connect(reply, &QNetworkReply::finished, reply, [this, settings, reply, connected, reportStatus, continuation]() {
        const QByteArray responseBody = reply->readAll();
        logResponse(settings, "PUT", reply->request().url(), reply, responseBody);

        bool success = false;

        if (reply->error() == QNetworkReply::NoError)
        {
            const QJsonDocument doc = QJsonDocument::fromJson(responseBody);

            if (doc.isObject()) {
                success = (doc.object().value("ErrorNumber").toInt(-1) == 0);
            }
        }

        if (success) {
            m_focuserConnected = connected;
        } else if (connected) {
            if (reply->error() != QNetworkReply::NoError) {
                setLastError(static_cast<int>(reply->error()), reply->errorString());
            }
            m_focuserConnected = false;
            if (reportStatus) {
                reportStatus();
            }
        } else {
            if (reply->error() != QNetworkReply::NoError) {
                setLastError(static_cast<int>(reply->error()), reply->errorString());
            }
            if (reportStatus) {
                reportStatus();
            }
        }

        if (!connected) {
            m_focuserConnectionPending = false;
            m_focuserConnected = false;
            m_pendingFocuserConnectedContinuations.clear();
        }

        if (success)
        {
            if (continuation) {
                continuation();
            }
        }
        else if (connected)
        {
            m_focuserConnectionPending = false;
            m_pendingFocuserConnectedContinuations.clear();
        }

        reply->deleteLater();
    });
}

void CameraAlpacaController::runFocuserWhenConnected(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    std::function<void()> reportStatus, std::function<void()> continuation)
{
    if (m_focuserConnected)
    {
        if (continuation) {
            continuation();
        }
        return;
    }

    if (continuation) {
        m_pendingFocuserConnectedContinuations.append(continuation);
    }

    if (m_focuserConnectionPending) {
        return;
    }

    m_focuserConnectionPending = true;
    setFocuserConnected(networkManager, settings, true, reportStatus, [this]() {
        m_focuserConnectionPending = false;
        runPendingContinuations(m_pendingFocuserConnectedContinuations);
    });
}

void CameraAlpacaController::moveFocuser(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    std::function<void()> reportStatus)
{
    moveFocuserToPosition(networkManager, settings, settings.m_alpacaFocusPosition, reportStatus);
}

void CameraAlpacaController::moveFocuserToPosition(QNetworkAccessManager *networkManager, const CameraSettings& settings, int position,
    std::function<void()> reportStatus, std::function<void()> onSuccess, std::function<void()> onFailure)
{
    if (!networkManager || !settings.isAlpacaCamera() || !settings.m_alpacaFocuserEnabled) {
        return;
    }

    runFocuserWhenConnected(networkManager, settings, reportStatus, [this, networkManager, settings, position, reportStatus, onSuccess, onFailure]() {
        const QString baseUrlString = focuserBaseUrl(settings);
        const int deviceNumber = std::max(0, settings.m_alpacaFocuserDeviceNumber);
        QUrl url(baseUrlString + QString("/api/v1/focuser/%1/move").arg(deviceNumber));
        QNetworkRequest request(url);
        request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

        QUrlQuery body;
        body.addQueryItem("Position", QString::number(std::max(0, position)));
        body.addQueryItem("ClientID", QString::number(m_clientId));
        body.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));

        const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
        logRequest(settings, "PUT", url, payload);

        QNetworkReply *reply = networkManager->put(request, payload);
        installAlpacaReplyTimeout(reply, alpacaDefaultNetworkTimeoutMs, QStringLiteral("PUT focuser move"));
        QObject::connect(reply, &QNetworkReply::finished, reply, [this, settings, reply, reportStatus, onSuccess, onFailure]() {
            const QByteArray responseBody = reply->readAll();
            logResponse(settings, "PUT", reply->request().url(), reply, responseBody);
            int alpacaErrorNumber = 0;
            QString alpacaErrorMessage;
            const bool alpacaPayloadParsed = parseErrorPayload(responseBody, alpacaErrorNumber, alpacaErrorMessage);
            const bool success = (reply->error() == QNetworkReply::NoError)
                && (!alpacaPayloadParsed || (alpacaErrorNumber == 0));

            if (!success)
            {
                if (reply->error() != QNetworkReply::NoError) {
                    setLastError(static_cast<int>(reply->error()), reply->errorString());
                } else {
                    setLastError(alpacaErrorNumber, alpacaErrorMessage);
                }

                if (reportStatus) {
                    reportStatus();
                }

                if (onFailure) {
                    onFailure();
                }
            }
            else if (onSuccess)
            {
                onSuccess();
            }
            reply->deleteLater();
        });
    });
}

void CameraAlpacaController::setFilterWheelConnected(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    bool connected, std::function<void()> reportStatus, std::function<void()> continuation)
{
    if (!networkManager)
    {
        return;
    }

    const QString baseUrlString = filterWheelBaseUrl(settings);
    const int deviceNumber = std::max(0, settings.m_alpacaFilterWheelDeviceNumber);
    QUrl url(baseUrlString + QString("/api/v1/filterwheel/%1/connected").arg(deviceNumber));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem("Connected", connected ? QStringLiteral("true") : QStringLiteral("false"));
    body.addQueryItem("ClientID", QString::number(m_clientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    logRequest(settings, "PUT", url, payload);

    QNetworkReply *reply = networkManager->put(request, payload);
    installAlpacaReplyTimeout(reply, alpacaDefaultNetworkTimeoutMs, QStringLiteral("PUT filter wheel connected"));

    QObject::connect(reply, &QNetworkReply::finished, reply, [this, settings, reply, connected, reportStatus, continuation]() {
        const QByteArray responseBody = reply->readAll();
        logResponse(settings, "PUT", reply->request().url(), reply, responseBody);

        bool success = false;

        if (reply->error() == QNetworkReply::NoError)
        {
            const QJsonDocument doc = QJsonDocument::fromJson(responseBody);

            if (doc.isObject()) {
                success = (doc.object().value("ErrorNumber").toInt(-1) == 0);
            }
        }

        if (success) {
            m_filterWheelConnected = connected;
        } else if (connected) {
            if (reply->error() != QNetworkReply::NoError) {
                setLastError(static_cast<int>(reply->error()), reply->errorString());
            }
            m_filterWheelConnected = false;
            if (reportStatus) {
                reportStatus();
            }
        } else {
            if (reply->error() != QNetworkReply::NoError) {
                setLastError(static_cast<int>(reply->error()), reply->errorString());
            }
            if (reportStatus) {
                reportStatus();
            }
        }

        if (!connected) {
            m_filterWheelConnectionPending = false;
            m_filterWheelConnected = false;
            m_pendingFilterWheelConnectedContinuations.clear();
        }

        if (success)
        {
            if (continuation) {
                continuation();
            }
        }
        else if (connected)
        {
            m_filterWheelConnectionPending = false;
            m_pendingFilterWheelConnectedContinuations.clear();
        }

        reply->deleteLater();
    });
}

void CameraAlpacaController::runFilterWheelWhenConnected(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    std::function<void()> reportStatus, std::function<void()> continuation)
{
    if (m_filterWheelConnected)
    {
        if (continuation) {
            continuation();
        }
        return;
    }

    if (continuation) {
        m_pendingFilterWheelConnectedContinuations.append(continuation);
    }

    if (m_filterWheelConnectionPending) {
        return;
    }

    m_filterWheelConnectionPending = true;
    setFilterWheelConnected(networkManager, settings, true, reportStatus, [this]() {
        m_filterWheelConnectionPending = false;
        runPendingContinuations(m_pendingFilterWheelConnectedContinuations);
    });
}

void CameraAlpacaController::queryFilterWheelInfo(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    std::function<void(const FilterWheelInfo&)> continuation)
{
    if (!networkManager || !settings.isAlpacaCamera() || !settings.m_alpacaFilterWheelEnabled) {
        return;
    }

    runFilterWheelWhenConnected(networkManager, settings, nullptr, [this, networkManager, settings, continuation]() {
        const QString baseUrlString = filterWheelBaseUrl(settings);
        const int deviceNumber = std::max(0, settings.m_alpacaFilterWheelDeviceNumber);
        auto info = QSharedPointer<FilterWheelInfo>::create();
        auto pending = QSharedPointer<int>::create(2);

        auto checkDone = [info, pending, continuation]() {
            (*pending)--;
            if (*pending > 0) {
                return;
            }

            if (continuation) {
                continuation(*info);
            }
        };

        auto makeGet = [this, networkManager, settings, baseUrlString, deviceNumber](const QString& prop) {
            QUrl url(baseUrlString + QString("/api/v1/filterwheel/%1/%2").arg(deviceNumber).arg(prop));
            QUrlQuery q;
            q.addQueryItem("ClientID", QString::number(m_clientId));
            q.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));
            url.setQuery(q);
            logRequest(settings, "GET", url);
            QNetworkReply *reply = networkManager->get(QNetworkRequest(url));
            installAlpacaReplyTimeout(reply, alpacaDefaultNetworkTimeoutMs, QStringLiteral("GET filter wheel info"));
            return reply;
        };

        QNetworkReply *namesReply = makeGet("names");
        QObject::connect(namesReply, &QNetworkReply::finished, namesReply, [this, settings, namesReply, info, checkDone]() {
            const QByteArray responseBody = namesReply->readAll();
            logResponse(settings, "GET", namesReply->request().url(), namesReply, responseBody);
            if (namesReply->error() == QNetworkReply::NoError) {
                const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
                if (doc.isObject()) {
                    const QJsonObject root = doc.object();
                    if (root.value("ErrorNumber").toInt(0) == 0) {
                        const QJsonValue namesValue = root.contains(QStringLiteral("Value"))
                            ? root.value(QStringLiteral("Value"))
                            : root.value(QStringLiteral("value"));

                        if (namesValue.isArray())
                        {
                            const QJsonArray values = namesValue.toArray();

                            for (const QJsonValue& value : values)
                            {
                                if (value.isString()) {
                                    info->m_names.append(value.toString());
                                } else if (value.isObject()) {
                                    const QJsonObject obj = value.toObject();
                                    info->m_names.append(obj.value(QStringLiteral("Name")).toString(
                                        obj.value(QStringLiteral("name")).toString()));
                                }
                            }
                        }
                        else if (namesValue.isString())
                        {
                            info->m_names.append(namesValue.toString());
                        }
                    }
                }
            }
            namesReply->deleteLater();
            checkDone();
        });

        QNetworkReply *positionReply = makeGet("position");
        QObject::connect(positionReply, &QNetworkReply::finished, positionReply, [this, settings, positionReply, info, checkDone]() {
            const QByteArray responseBody = positionReply->readAll();
            logResponse(settings, "GET", positionReply->request().url(), positionReply, responseBody);
            if (positionReply->error() == QNetworkReply::NoError) {
                const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
                if (doc.isObject()) {
                    const QJsonObject root = doc.object();
                    if (root.value("ErrorNumber").toInt(0) == 0) {
                        const QJsonValue positionValue = root.contains(QStringLiteral("Value"))
                            ? root.value(QStringLiteral("Value"))
                            : root.value(QStringLiteral("value"));
                        info->m_position = std::max(0, positionValue.toInt(0));
                    }
                }
            }
            positionReply->deleteLater();
            checkDone();
        });
    });
}

void CameraAlpacaController::queryFilterWheelPosition(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    std::function<void(int)> continuation)
{
    if (!networkManager || !settings.isAlpacaCamera() || !settings.m_alpacaFilterWheelEnabled) {
        if (continuation) {
            continuation(-1);
        }
        return;
    }

    runFilterWheelWhenConnected(networkManager, settings, nullptr, [this, networkManager, settings, continuation]() {
        const QString baseUrlString = filterWheelBaseUrl(settings);
        const int deviceNumber = std::max(0, settings.m_alpacaFilterWheelDeviceNumber);
        QUrl url(baseUrlString + QString("/api/v1/filterwheel/%1/position").arg(deviceNumber));
        QUrlQuery q;
        q.addQueryItem("ClientID", QString::number(m_clientId));
        q.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));
        url.setQuery(q);
        logRequest(settings, "GET", url);

        QNetworkReply *reply = networkManager->get(QNetworkRequest(url));
        installAlpacaReplyTimeout(reply, alpacaDefaultNetworkTimeoutMs, QStringLiteral("GET filter wheel position"));
        QObject::connect(reply, &QNetworkReply::finished, reply, [this, settings, reply, continuation]() {
            int position = -1;
            const QByteArray responseBody = reply->readAll();
            logResponse(settings, "GET", reply->request().url(), reply, responseBody);
            if (reply->error() == QNetworkReply::NoError) {
                const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
                if (doc.isObject()) {
                    const QJsonObject root = doc.object();
                    if (root.value("ErrorNumber").toInt(0) == 0) {
                        const QJsonValue positionValue = root.contains(QStringLiteral("Value"))
                            ? root.value(QStringLiteral("Value"))
                            : root.value(QStringLiteral("value"));
                        position = positionValue.toInt(-1);
                    }
                }
            }
            reply->deleteLater();
            if (continuation) {
                continuation(position);
            }
        });
    });
}

void CameraAlpacaController::setFilterWheelPosition(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    std::function<void()> onSuccess, std::function<void()> reportStatus)
{
    if (!networkManager || !settings.isAlpacaCamera() || !settings.m_alpacaFilterWheelEnabled) {
        return;
    }

    runFilterWheelWhenConnected(networkManager, settings, reportStatus, [this, networkManager, settings, onSuccess, reportStatus]() {
        const QString baseUrlString = filterWheelBaseUrl(settings);
        const int deviceNumber = std::max(0, settings.m_alpacaFilterWheelDeviceNumber);
        QUrl url(baseUrlString + QString("/api/v1/filterwheel/%1/position").arg(deviceNumber));
        QNetworkRequest request(url);
        request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

        QUrlQuery body;
        body.addQueryItem("Position", QString::number(std::max(0, settings.m_alpacaFilterWheelPosition)));
        body.addQueryItem("ClientID", QString::number(m_clientId));
        body.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));

        const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
        logRequest(settings, "PUT", url, payload);

        QNetworkReply *reply = networkManager->put(request, payload);
        installAlpacaReplyTimeout(reply, alpacaDefaultNetworkTimeoutMs, QStringLiteral("PUT filter wheel position"));
        QObject::connect(reply, &QNetworkReply::finished, reply, [this, settings, reply, onSuccess, reportStatus]() {
            const QByteArray responseBody = reply->readAll();
            logResponse(settings, "PUT", reply->request().url(), reply, responseBody);
            int alpacaErrorNumber = 0;
            QString alpacaErrorMessage;
            const bool alpacaPayloadParsed = parseErrorPayload(responseBody, alpacaErrorNumber, alpacaErrorMessage);
            const bool success = (reply->error() == QNetworkReply::NoError)
                && (!alpacaPayloadParsed || (alpacaErrorNumber == 0));

            if (success)
            {
                if (onSuccess) {
                    onSuccess();
                }
            }
            else
            {
                if (reply->error() != QNetworkReply::NoError) {
                    setLastError(static_cast<int>(reply->error()), reply->errorString());
                } else {
                    setLastError(alpacaErrorNumber, alpacaErrorMessage);
                }

                if (reportStatus) {
                    reportStatus();
                }
            }
            reply->deleteLater();
        });
    });
}

void CameraAlpacaController::setCameraParams(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    std::function<bool()> isCapturing, std::function<void()> onStartExposure, std::function<void()> onFailure)
{
    const int camId = settings.cameraIdInt();
    const bool forceAllParams = !m_paramsInitialized;
    const int maxSubframeX = std::max(1, m_cameraSizeX / std::max(1, settings.m_cameraBinX));
    const int maxSubframeY = std::max(1, m_cameraSizeY / std::max(1, settings.m_cameraBinY));
    const bool fullFrameNumXRequested = (settings.m_cameraNumX == 0);
    const bool fullFrameNumYRequested = (settings.m_cameraNumY == 0);
    const bool canResolveNumX = !fullFrameNumXRequested || (m_cameraSizeX > 0);
    const bool canResolveNumY = !fullFrameNumYRequested || (m_cameraSizeY > 0);
    const int effectiveNumX = fullFrameNumXRequested
        ? std::max(1, maxSubframeX - std::max(0, settings.m_cameraStartX))
        : settings.m_cameraNumX;
    const int effectiveNumY = fullFrameNumYRequested
        ? std::max(1, maxSubframeY - std::max(0, settings.m_cameraStartY))
        : settings.m_cameraNumY;
    const bool setBinX = forceAllParams || (m_lastBinX != settings.m_cameraBinX);
    const bool setBinY = forceAllParams || (m_lastBinY != settings.m_cameraBinY);
    const bool setNumX = canResolveNumX && (forceAllParams || (m_lastEffectiveNumX != effectiveNumX));
    const bool setNumY = canResolveNumY && (forceAllParams || (m_lastEffectiveNumY != effectiveNumY));
    const bool setStartX = forceAllParams || (m_lastStartX != settings.m_cameraStartX);
    const bool setStartY = forceAllParams || (m_lastStartY != settings.m_cameraStartY);
    const bool setGain = (settings.m_cameraGain >= 0) && (forceAllParams || (m_lastGain != settings.m_cameraGain));
    const bool setOffset = (settings.m_cameraOffset >= 0) && (forceAllParams || (m_lastOffset != settings.m_cameraOffset));
    const bool setReadoutMode = forceAllParams || (m_lastReadoutMode != settings.m_cameraReadoutMode);

    auto stillCapturing = [isCapturing]() {
        return !isCapturing || isCapturing();
    };

    auto doStartExposure = [this, stillCapturing, onStartExposure]() {
        if (stillCapturing()) {
            m_paramsInitialized = true;
            if (onStartExposure) {
                onStartExposure();
            }
        } else {
            m_frameRequestPending = false;
        }
    };

    auto handleParamFailure = [this, onFailure](int errorNumber, const QString& errorMessage) {
        m_lastErrorNumber = errorNumber;
        m_lastErrorMessage = errorMessage;
        m_frameRequestPending = false;
        if (onFailure) {
            onFailure();
        }
    };

    auto handleOptionalParamFailure = [this, handleParamFailure](const QString& property, int errorNumber, const QString& errorMessage,
            const std::function<void()>& markAttempted, const std::function<void()>& continuation) {
        if (isDriverError(errorNumber))
        {
            qDebug() << "CameraWorker:" << property << "Alpaca error is non-fatal for exposure:"
                     << errorNumber << errorMessage;
            m_lastErrorNumber = errorNumber;
            m_lastErrorMessage = errorMessage;
            if (markAttempted) {
                markAttempted();
            }
            if (continuation) {
                continuation();
            }
            return;
        }

        handleParamFailure(errorNumber, errorMessage);
    };

    auto doReadoutMode = [this, networkManager, settings, camId, doStartExposure, setReadoutMode, handleOptionalParamFailure, stillCapturing]() {
        if (!stillCapturing()) { m_frameRequestPending = false; return; }
        if (setReadoutMode) {
            putIntProperty(networkManager, settings, camId, "readoutmode", "ReadoutMode",
                settings.m_cameraReadoutMode, doStartExposure,
                [this, settings]() { m_lastReadoutMode = settings.m_cameraReadoutMode; },
                [this, settings, handleOptionalParamFailure, doStartExposure](int errorNumber, const QString& errorMessage) {
                    handleOptionalParamFailure(
                        QStringLiteral("readoutmode"),
                        errorNumber,
                        errorMessage,
                        [this, settings]() { m_lastReadoutMode = settings.m_cameraReadoutMode; },
                        doStartExposure);
                });
        } else {
            doStartExposure();
        }
    };

    auto doOffset = [this, networkManager, settings, camId, doReadoutMode, setOffset, handleOptionalParamFailure, stillCapturing]() {
        if (!stillCapturing()) { m_frameRequestPending = false; return; }
        if (setOffset) {
            putIntProperty(networkManager, settings, camId, "offset", "Offset",
                settings.m_cameraOffset, doReadoutMode,
                [this, settings]() { m_lastOffset = settings.m_cameraOffset; },
                [this, settings, handleOptionalParamFailure, doReadoutMode](int errorNumber, const QString& errorMessage) {
                    handleOptionalParamFailure(
                        QStringLiteral("offset"),
                        errorNumber,
                        errorMessage,
                        [this, settings]() { m_lastOffset = settings.m_cameraOffset; },
                        doReadoutMode);
                });
        } else {
            doReadoutMode();
        }
    };

    auto doGain = [this, networkManager, settings, camId, doOffset, setGain, handleOptionalParamFailure, stillCapturing]() {
        if (!stillCapturing()) { m_frameRequestPending = false; return; }
        if (setGain) {
            putIntProperty(networkManager, settings, camId, "gain", "Gain",
                settings.m_cameraGain, doOffset,
                [this, settings]() { m_lastGain = settings.m_cameraGain; },
                [this, settings, handleOptionalParamFailure, doOffset](int errorNumber, const QString& errorMessage) {
                    handleOptionalParamFailure(
                        QStringLiteral("gain"),
                        errorNumber,
                        errorMessage,
                        [this, settings]() { m_lastGain = settings.m_cameraGain; },
                        doOffset);
                });
        } else {
            doOffset();
        }
    };

    auto doAxisY = [this, networkManager, settings, camId, doGain, setNumY, setStartY, effectiveNumY, handleParamFailure, stillCapturing]() {
        if (!stillCapturing()) { m_frameRequestPending = false; return; }

        std::function<void()> maybeSetStartYAfterNum = [this, networkManager, settings, camId, doGain, setStartY, handleParamFailure, stillCapturing]() {
            if (!stillCapturing()) { m_frameRequestPending = false; return; }
            if (setStartY) {
                putIntProperty(networkManager, settings, camId, "starty", "StartY",
                    settings.m_cameraStartY, doGain,
                    [this, settings]() { m_lastStartY = settings.m_cameraStartY; },
                    handleParamFailure);
            } else {
                doGain();
            }
        };

        std::function<void()> maybeSetNumYAfterStart = [this, networkManager, settings, camId, doGain, setNumY, effectiveNumY, handleParamFailure, stillCapturing]() {
            if (!stillCapturing()) { m_frameRequestPending = false; return; }
            if (setNumY) {
                putIntProperty(networkManager, settings, camId, "numy", "NumY",
                    effectiveNumY, doGain,
                    [this, settings, effectiveNumY]() {
                        m_lastNumY = settings.m_cameraNumY;
                        m_lastEffectiveNumY = effectiveNumY;
                    },
                    handleParamFailure);
            } else {
                doGain();
            }
        };

        if (setStartY && (settings.m_cameraStartY < m_lastStartY))
        {
            putIntProperty(networkManager, settings, camId, "starty", "StartY",
                settings.m_cameraStartY, maybeSetNumYAfterStart,
                [this, settings]() { m_lastStartY = settings.m_cameraStartY; },
                handleParamFailure);
        }
        else if (setNumY)
        {
            putIntProperty(networkManager, settings, camId, "numy", "NumY",
                effectiveNumY, maybeSetStartYAfterNum,
                [this, settings, effectiveNumY]() {
                    m_lastNumY = settings.m_cameraNumY;
                    m_lastEffectiveNumY = effectiveNumY;
                },
                handleParamFailure);
        }
        else
        {
            maybeSetStartYAfterNum();
        }
    };

    auto doAxisX = [this, networkManager, settings, camId, doAxisY, setNumX, setStartX, effectiveNumX, handleParamFailure, stillCapturing]() {
        if (!stillCapturing()) { m_frameRequestPending = false; return; }

        std::function<void()> maybeSetStartXAfterNum = [this, networkManager, settings, camId, doAxisY, setStartX, handleParamFailure, stillCapturing]() {
            if (!stillCapturing()) { m_frameRequestPending = false; return; }
            if (setStartX) {
                putIntProperty(networkManager, settings, camId, "startx", "StartX",
                    settings.m_cameraStartX, doAxisY,
                    [this, settings]() { m_lastStartX = settings.m_cameraStartX; },
                    handleParamFailure);
            } else {
                doAxisY();
            }
        };

        std::function<void()> maybeSetNumXAfterStart = [this, networkManager, settings, camId, doAxisY, setNumX, effectiveNumX, handleParamFailure, stillCapturing]() {
            if (!stillCapturing()) { m_frameRequestPending = false; return; }
            if (setNumX) {
                putIntProperty(networkManager, settings, camId, "numx", "NumX",
                    effectiveNumX, doAxisY,
                    [this, settings, effectiveNumX]() {
                        m_lastNumX = settings.m_cameraNumX;
                        m_lastEffectiveNumX = effectiveNumX;
                    },
                    handleParamFailure);
            } else {
                doAxisY();
            }
        };

        if (setStartX && (settings.m_cameraStartX < m_lastStartX))
        {
            putIntProperty(networkManager, settings, camId, "startx", "StartX",
                settings.m_cameraStartX, maybeSetNumXAfterStart,
                [this, settings]() { m_lastStartX = settings.m_cameraStartX; },
                handleParamFailure);
        }
        else if (setNumX)
        {
            putIntProperty(networkManager, settings, camId, "numx", "NumX",
                effectiveNumX, maybeSetStartXAfterNum,
                [this, settings, effectiveNumX]() {
                    m_lastNumX = settings.m_cameraNumX;
                    m_lastEffectiveNumX = effectiveNumX;
                },
                handleParamFailure);
        }
        else
        {
            maybeSetStartXAfterNum();
        }
    };

    auto doBinY = [this, networkManager, settings, camId, doAxisX, setBinY, handleParamFailure, stillCapturing]() {
        if (!stillCapturing()) { m_frameRequestPending = false; return; }
        if (setBinY) {
            putIntProperty(networkManager, settings, camId, "biny", "BinY",
                settings.m_cameraBinY, doAxisX,
                [this, settings]() { m_lastBinY = settings.m_cameraBinY; },
                handleParamFailure);
        } else {
            doAxisX();
        }
    };

    if (setBinX) {
        putIntProperty(networkManager, settings, camId, "binx", "BinX",
            settings.m_cameraBinX, doBinY,
            [this, settings]() { m_lastBinX = settings.m_cameraBinX; },
            handleParamFailure);
    } else {
        doBinY();
    }
}

void CameraAlpacaController::putIntProperty(
    QNetworkAccessManager *networkManager,
    const CameraSettings& settings,
    int cameraId,
    const QString& property,
    const QString& bodyKey,
    int value,
    std::function<void()> continuation,
    std::function<void()> onSuccess,
    std::function<void(int, const QString&)> onFailure)
{
    if (!networkManager)
    {
        if (onFailure) {
            onFailure(-1, QStringLiteral("No network manager"));
        }
        return;
    }

    QUrl url(baseUrl(settings) + QString("/api/v1/camera/%1/%2").arg(cameraId).arg(property));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem(bodyKey, QString::number(value));
    body.addQueryItem("ClientID", QString::number(m_clientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    logRequest(settings, "PUT", url, payload);
    QNetworkReply *reply = networkManager->put(request, payload);
    installAlpacaReplyTimeout(reply, alpacaDefaultNetworkTimeoutMs, QStringLiteral("PUT camera property"));

    QObject::connect(reply, &QNetworkReply::finished, reply, [this, settings, reply, property, continuation, onSuccess, onFailure]() {
        const QByteArray responseBody = reply->readAll();
        logResponse(settings, "PUT", reply->request().url(), reply, responseBody);

        int alpacaErrorNumber = 0;
        QString alpacaErrorMessage;
        const bool alpacaPayloadParsed = parseErrorPayload(responseBody, alpacaErrorNumber, alpacaErrorMessage);
        bool success = (reply->error() == QNetworkReply::NoError);

        if (success && alpacaPayloadParsed) {
            success = (alpacaErrorNumber == 0);
        }

        if (success)
        {
            if (onSuccess) {
                onSuccess();
            }
            reply->deleteLater();
            if (continuation) {
                continuation();
            }
            return;
        }

        if (reply->error() != QNetworkReply::NoError)
        {
            qDebug() << "CameraWorker: PUT" << property << "error:" << reply->errorString();
            if (onFailure) {
                onFailure(static_cast<int>(reply->error()), reply->errorString());
            }
        }
        else
        {
            qDebug() << "CameraWorker: PUT" << property << "Alpaca error:"
                     << alpacaErrorNumber << alpacaErrorMessage;
            if (onFailure) {
                onFailure(alpacaErrorNumber, alpacaErrorMessage);
            }
        }
        reply->deleteLater();
    });
}

void CameraAlpacaController::startExposure(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    double exposureTimeMs, std::function<void()> onSuccess, std::function<void()> onFailure)
{
    if (!networkManager)
    {
        if (onFailure) {
            onFailure();
        }
        return;
    }

    QUrl url(baseUrl(settings) + QString("/api/v1/camera/%1/startexposure").arg(settings.cameraIdInt()));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem("Duration", QString::number(exposureTimeMs / 1000.0, 'f', 6));
    body.addQueryItem("Light", "True");
    body.addQueryItem("ClientID", QString::number(m_clientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    logRequest(settings, "PUT", url, payload);
    QNetworkReply *reply = networkManager->put(request, payload);
    installAlpacaReplyTimeout(reply, alpacaDefaultNetworkTimeoutMs, QStringLiteral("PUT start exposure"));

    QObject::connect(reply, &QNetworkReply::finished, reply, [this, settings, reply, onSuccess, onFailure]() {
        const QByteArray responseBody = reply->readAll();
        logResponse(settings, "PUT", reply->request().url(), reply, responseBody);

        auto fail = [reply, onFailure]() {
            if (onFailure) {
                onFailure();
            }
            reply->deleteLater();
        };

        if (reply->error() != QNetworkReply::NoError)
        {
            qDebug() << "CameraAlpacaController::startExposure: error:" << reply->errorString();
            setLastError(static_cast<int>(reply->error()), reply->errorString());
            fail();
            return;
        }

        int alpacaErrorNumber = 0;
        QString alpacaErrorMessage;
        if (parseErrorPayload(responseBody, alpacaErrorNumber, alpacaErrorMessage)
            && (alpacaErrorNumber != 0))
        {
            setLastError(alpacaErrorNumber, alpacaErrorMessage);
            qDebug() << "CameraAlpacaController::startExposure: Alpaca error"
                     << m_lastErrorNumber << m_lastErrorMessage;
            fail();
            return;
        }

        m_exposureSeenActive = false;

        if (onSuccess) {
            onSuccess();
        }
        reply->deleteLater();
    });
}

void CameraAlpacaController::abortExposure(QNetworkAccessManager *networkManager, const CameraSettings& settings)
{
    if (!networkManager) {
        return;
    }

    QUrl url(baseUrl(settings) + QString("/api/v1/camera/%1/abortexposure").arg(settings.cameraIdInt()));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem("ClientID", QString::number(m_clientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    logRequest(settings, "PUT", url, payload);
    QNetworkReply *reply = networkManager->put(request, payload);
    installAlpacaReplyTimeout(reply, alpacaDefaultNetworkTimeoutMs, QStringLiteral("PUT abort exposure"));

    QObject::connect(reply, &QNetworkReply::finished, reply, [this, settings, reply]() {
        const QByteArray responseBody = reply->readAll();
        logResponse(settings, "PUT", reply->request().url(), reply, responseBody);

        if (reply->error() != QNetworkReply::NoError)
        {
            qDebug() << "CameraAlpacaController::abortExposure: error:" << reply->errorString();
            reply->deleteLater();
            return;
        }

        int alpacaErrorNumber = 0;
        QString alpacaErrorMessage;
        if (parseErrorPayload(responseBody, alpacaErrorNumber, alpacaErrorMessage)
            && (alpacaErrorNumber != 0))
        {
            qDebug() << "CameraAlpacaController::abortExposure: Alpaca error"
                     << alpacaErrorNumber << alpacaErrorMessage;
        }

        reply->deleteLater();
    });
}

void CameraAlpacaController::checkImageReady(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    std::function<void(bool)> onSuccess, std::function<void()> onFailure)
{
    if (!networkManager)
    {
        if (onFailure) {
            onFailure();
        }
        return;
    }

    QUrl url(baseUrl(settings) + QString("/api/v1/camera/%1/imageready").arg(settings.cameraIdInt()));
    QUrlQuery query;
    query.addQueryItem("ClientID", QString::number(m_clientId));
    query.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));
    url.setQuery(query);

    logRequest(settings, "GET", url);
    QNetworkReply *reply = networkManager->get(QNetworkRequest(url));
    installAlpacaReplyTimeout(reply, alpacaDefaultNetworkTimeoutMs, QStringLiteral("GET imageready"));

    QObject::connect(reply, &QNetworkReply::finished, reply, [this, settings, reply, onSuccess, onFailure]() {
        const QByteArray responseBody = reply->readAll();
        logResponse(settings, "GET", reply->request().url(), reply, responseBody);

        auto fail = [reply, onFailure]() {
            if (onFailure) {
                onFailure();
            }
            reply->deleteLater();
        };

        if (reply->error() != QNetworkReply::NoError)
        {
            setLastError(static_cast<int>(reply->error()), reply->errorString());
            fail();
            return;
        }

        int alpacaErrorNumber = 0;
        QString alpacaErrorMessage;
        if (parseErrorPayload(responseBody, alpacaErrorNumber, alpacaErrorMessage)
            && (alpacaErrorNumber != 0))
        {
            setLastError(alpacaErrorNumber, alpacaErrorMessage);
            fail();
            return;
        }

        bool ready = false;
        const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
        if (doc.isObject()) {
            ready = doc.object().value("Value").toBool(false);
        }

        if (onSuccess) {
            onSuccess(ready);
        }
        reply->deleteLater();
    });
}

void CameraAlpacaController::checkCameraStateForImageReady(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    std::function<void(int)> onSuccess, std::function<void()> onFailure)
{
    if (!networkManager)
    {
        if (onFailure) {
            onFailure();
        }
        return;
    }

    QUrl url(baseUrl(settings) + QString("/api/v1/camera/%1/camerastate").arg(settings.cameraIdInt()));
    QUrlQuery query;
    query.addQueryItem("ClientID", QString::number(m_clientId));
    query.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));
    url.setQuery(query);

    logRequest(settings, "GET", url);
    QNetworkReply *reply = networkManager->get(QNetworkRequest(url));
    installAlpacaReplyTimeout(reply, alpacaDefaultNetworkTimeoutMs, QStringLiteral("GET camerastate"));

    QObject::connect(reply, &QNetworkReply::finished, reply, [this, settings, reply, onSuccess, onFailure]() {
        const QByteArray responseBody = reply->readAll();
        logResponse(settings, "GET", reply->request().url(), reply, responseBody);

        auto fail = [reply, onFailure]() {
            if (onFailure) {
                onFailure();
            }
            reply->deleteLater();
        };

        if (reply->error() != QNetworkReply::NoError)
        {
            setLastError(static_cast<int>(reply->error()), reply->errorString());
            fail();
            return;
        }

        int alpacaErrorNumber = 0;
        QString alpacaErrorMessage;
        if (parseErrorPayload(responseBody, alpacaErrorNumber, alpacaErrorMessage)
            && (alpacaErrorNumber != 0))
        {
            setLastError(alpacaErrorNumber, alpacaErrorMessage);
            fail();
            return;
        }

        int cameraState = -1;
        const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
        if (doc.isObject()) {
            cameraState = doc.object().value("Value").toInt(-1);
        }

        if (cameraState > 0) {
            m_exposureSeenActive = true;
        }

        if (onSuccess) {
            onSuccess(cameraState);
        }
        reply->deleteLater();
    });
}

void CameraAlpacaController::fetchImageArray(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    const QImage& fallbackImage, std::function<void(const ImageFetchResult&)> onSuccess, std::function<void()> onFailure)
{
    if (!networkManager)
    {
        m_frameRequestPending = false;
        if (onFailure) {
            onFailure();
        }
        return;
    }

    QUrl url(baseUrl(settings) + QString("/api/v1/camera/%1/imagearray").arg(settings.cameraIdInt()));
    QUrlQuery query;
    query.addQueryItem("ClientID", QString::number(m_clientId));
    query.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));
    url.setQuery(query);

    QNetworkRequest request(url);
    if (m_imageBytesSupported) {
        request.setRawHeader("Accept", "application/imagebytes");
    }

    logRequest(settings, "GET", url);
    QNetworkReply *reply = networkManager->get(request);
    installAlpacaReplyTimeout(reply, alpacaImageArrayNetworkTimeoutMs, QStringLiteral("GET imagearray"));

    QObject::connect(reply, &QNetworkReply::finished, reply, [this, settings, fallbackImage, reply, onSuccess, onFailure]() {
        m_frameRequestPending = false;
        const QByteArray data = reply->readAll();
        logResponse(settings, "GET", reply->request().url(), reply, data);

        auto fail = [reply, onFailure]() {
            if (onFailure) {
                onFailure();
            }
            reply->deleteLater();
        };

        if (reply->error() != QNetworkReply::NoError)
        {
            setLastError(static_cast<int>(reply->error()), reply->errorString());
            fail();
            return;
        }

        ImageFetchResult result;
        result.m_image = fallbackImage;
        result.m_bayerPattern = CameraPipelineFrame::BayerNone;

        const QString contentType = reply->header(QNetworkRequest::ContentTypeHeader).toString();
        if (contentType.contains(QLatin1String("application/imagebytes"), Qt::CaseInsensitive))
        {
            int imageBytesErrorNumber = 0;
            if (parseImageBytesError(data, imageBytesErrorNumber) && (imageBytesErrorNumber != 0))
            {
                setLastError(imageBytesErrorNumber, QStringLiteral("ImageBytes Alpaca error %1").arg(imageBytesErrorNumber));
                fail();
                return;
            }

            m_imageBytesSupported = true;
            result.m_image = parseImageBytes(data, fallbackImage, &result.m_receiveImageFormat, &result.m_bayerPattern);
        }
        else
        {
            int alpacaErrorNumber = 0;
            QString alpacaErrorMessage;
            if (parseErrorPayload(data, alpacaErrorNumber, alpacaErrorMessage) && (alpacaErrorNumber != 0))
            {
                setLastError(alpacaErrorNumber, alpacaErrorMessage);
                fail();
                return;
            }

            if (m_imageBytesSupported) {
                qDebug() << "CameraAlpacaController::fetchImageArray: server returned JSON; disabling ImageBytes for this camera";
                m_imageBytesSupported = false;
            }
            result.m_image = parseImageArray(data, fallbackImage, &result.m_receiveImageFormat, &result.m_bayerPattern);
        }

        m_lastReceiveImageFormat = result.m_receiveImageFormat;

        if (m_captureTimer.isValid())
        {
            m_lastCaptureTimeMs = m_captureTimer.elapsed();
            m_captureTimer.invalidate();
        }

        if (onSuccess) {
            onSuccess(result);
        }
        reply->deleteLater();
    });
}

void CameraAlpacaController::queryCameraCapabilities(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    std::function<void(const CapabilitiesReport&)> continuation)
{
    if (!networkManager) {
        return;
    }

    m_imageBytesSupported = true;

    const QString baseUrlString = baseUrl(settings);
    const int camId = settings.cameraIdInt();
    auto report = QSharedPointer<CapabilitiesReport>::create();

    static const QStringList properties = {
        "name", "description",
        "maxbinx", "maxbiny", "gains", "gainmin", "gainmax",
        "offsets", "offsetmin", "offsetmax",
        "readoutmodes", "sensorname", "sensortype", "bayeroffsetx", "bayeroffsety",
        "pixelsizex", "pixelsizey", "cameraxsize", "cameraysize",
        "ccdtemperature", "exposuremin", "exposuremax", "exposureresolution"
    };

    auto pending = QSharedPointer<int>::create(properties.size());

    auto checkDone = [this, report, pending, continuation]() {
        (*pending)--;
        if (*pending > 0) {
            return;
        }

        m_sensorType = report->m_sensorType;
        m_cameraSizeX = std::max(0, report->m_cameraSizeX);
        m_cameraSizeY = std::max(0, report->m_cameraSizeY);
        m_bayerOffsetX = report->m_bayerOffsetX;
        m_bayerOffsetY = report->m_bayerOffsetY;
        report->m_exposureMinMs = std::max(0.001, report->m_exposureMinMs);
        report->m_exposureResolutionMs = std::max(0.001, report->m_exposureResolutionMs);
        report->m_exposureMaxMs = std::max(report->m_exposureMinMs, report->m_exposureMaxMs);
        m_exposureMinMs = report->m_exposureMinMs;
        m_exposureMaxMs = report->m_exposureMaxMs;

        if (continuation) {
            continuation(*report);
        }
    };

    auto query = [this, networkManager, settings, baseUrlString, camId, report, checkDone](const QString& prop) {
        QUrl url(baseUrlString + QString("/api/v1/camera/%1/%2").arg(camId).arg(prop));
        QUrlQuery q;
        q.addQueryItem("ClientID", QString::number(m_clientId));
        q.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));
        url.setQuery(q);
        logRequest(settings, "GET", url);

        QNetworkReply *reply = networkManager->get(QNetworkRequest(url));
        installAlpacaReplyTimeout(reply, alpacaDefaultNetworkTimeoutMs, QStringLiteral("GET camera capability"));

        QObject::connect(reply, &QNetworkReply::finished, reply, [this, settings, reply, prop, report, checkDone]() {
            const QByteArray responseBody = reply->readAll();
            logResponse(settings, "GET", reply->request().url(), reply, responseBody);
            if (reply->error() == QNetworkReply::NoError) {
                const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
                if (doc.isObject()) {
                    const QJsonObject root = doc.object();
                    const int errNum = root.value("ErrorNumber").toInt(0);
                    if (errNum == 0) {
                        const QJsonValue val = root.value("Value");
                        if (prop == "name") {
                            report->m_name = val.toString();
                        } else if (prop == "description") {
                            report->m_description = val.toString();
                        } else if (prop == "maxbinx") {
                            report->m_maxBinX = std::max(1, val.toInt(1));
                        } else if (prop == "maxbiny") {
                            report->m_maxBinY = std::max(1, val.toInt(1));
                        } else if (prop == "gains") {
                            if (val.isArray()) {
                                for (const QJsonValue& g : val.toArray()) {
                                    report->m_gains.append(g.toString());
                                }
                            }
                        } else if (prop == "gainmin") {
                            report->m_gainMin = val.toInt(0);
                        } else if (prop == "gainmax") {
                            report->m_gainMax = val.toInt(0);
                        } else if (prop == "offsets") {
                            if (val.isArray()) {
                                for (const QJsonValue& o : val.toArray()) {
                                    report->m_offsets.append(o.toString());
                                }
                            }
                        } else if (prop == "offsetmin") {
                            report->m_offsetMin = val.toInt(0);
                        } else if (prop == "offsetmax") {
                            report->m_offsetMax = val.toInt(0);
                        } else if (prop == "readoutmodes") {
                            if (val.isArray()) {
                                for (const QJsonValue& m : val.toArray()) {
                                    report->m_readoutModes.append(m.toString());
                                }
                            }
                        } else if (prop == "sensorname") {
                            report->m_sensorName = val.toString();
                        } else if (prop == "sensortype") {
                            report->m_sensorType = val.toInt(0);
                        } else if (prop == "bayeroffsetx") {
                            report->m_bayerOffsetX = val.toInt(0);
                        } else if (prop == "bayeroffsety") {
                            report->m_bayerOffsetY = val.toInt(0);
                        } else if (prop == "pixelsizex") {
                            report->m_pixelSizeX = val.toDouble(0.0);
                        } else if (prop == "pixelsizey") {
                            report->m_pixelSizeY = val.toDouble(0.0);
                        } else if (prop == "cameraxsize") {
                            report->m_cameraSizeX = val.toInt(0);
                        } else if (prop == "cameraysize") {
                            report->m_cameraSizeY = val.toInt(0);
                        } else if (prop == "ccdtemperature") {
                            report->m_ccdTemperature = val.toDouble(0.0);
                            report->m_ccdTemperatureValid = true;
                        } else if (prop == "exposuremin") {
                            report->m_exposureMinMs = std::max(0.001, val.toDouble(0.0) * 1000.0);
                        } else if (prop == "exposuremax") {
                            report->m_exposureMaxMs = std::max(report->m_exposureMinMs, val.toDouble(0.0) * 1000.0);
                        } else if (prop == "exposureresolution") {
                            report->m_exposureResolutionMs = std::max(0.001, val.toDouble(0.0) * 1000.0);
                        }
                    }
                }
            }
            reply->deleteLater();
            checkDone();
        });
    };

    for (const QString& prop : properties) {
        query(prop);
    }
}

void CameraAlpacaController::pollStatus(QNetworkAccessManager *networkManager, const CameraSettings& settings,
    std::function<void(const StatusReport&)> continuation)
{
    if (!networkManager) {
        return;
    }

    const QString baseUrlString = baseUrl(settings);
    const int camId = settings.cameraIdInt();
    auto status = QSharedPointer<StatusReport>::create();
    auto pending = QSharedPointer<int>::create(2);

    auto checkDone = [status, pending, continuation]() {
        (*pending)--;
        if (*pending > 0) {
            return;
        }
        if (continuation) {
            continuation(*status);
        }
    };

    auto makeGet = [this, networkManager, settings, baseUrlString, camId](const QString& prop) {
        QUrl url(baseUrlString + QString("/api/v1/camera/%1/%2").arg(camId).arg(prop));
        QUrlQuery q;
        q.addQueryItem("ClientID", QString::number(m_clientId));
        q.addQueryItem("ClientTransactionID", QString::number(m_clientTransactionId++));
        url.setQuery(q);
        logRequest(settings, "GET", url);
        QNetworkReply *reply = networkManager->get(QNetworkRequest(url));
        installAlpacaReplyTimeout(reply, alpacaDefaultNetworkTimeoutMs, QStringLiteral("GET camera status"));
        return reply;
    };

    QNetworkReply *stateReply = makeGet("camerastate");
    QObject::connect(stateReply, &QNetworkReply::finished, stateReply, [this, settings, stateReply, status, checkDone]() {
        const QByteArray responseBody = stateReply->readAll();
        logResponse(settings, "GET", stateReply->request().url(), stateReply, responseBody);
        if (stateReply->error() == QNetworkReply::NoError) {
            const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
            if (doc.isObject()) {
                const QJsonObject root = doc.object();
                if (root.value("ErrorNumber").toInt(0) == 0) {
                    status->m_cameraState = root.value("Value").toInt(-1);
                }
            }
        }
        stateReply->deleteLater();
        checkDone();
    });

    QNetworkReply *tempReply = makeGet("ccdtemperature");
    QObject::connect(tempReply, &QNetworkReply::finished, tempReply, [this, settings, tempReply, status, checkDone]() {
        const QByteArray responseBody = tempReply->readAll();
        logResponse(settings, "GET", tempReply->request().url(), tempReply, responseBody);
        if (tempReply->error() == QNetworkReply::NoError) {
            const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
            if (doc.isObject()) {
                const QJsonObject root = doc.object();
                if (root.value("ErrorNumber").toInt(0) == 0) {
                    status->m_ccdTemperature = root.value("Value").toDouble(0.0);
                    status->m_ccdTemperatureValid = true;
                }
            }
        }
        tempReply->deleteLater();
        checkDone();
    });
}

bool CameraAlpacaController::parseErrorPayload(const QByteArray& payload, int& errorNumber, QString& errorMessage)
{
    errorNumber = 0;
    errorMessage.clear();

    const QJsonDocument doc = QJsonDocument::fromJson(payload);

    if (!doc.isObject()) {
        return false;
    }

    const QJsonObject root = doc.object();

    if (!root.contains(QStringLiteral("ErrorNumber")) && !root.contains(QStringLiteral("ErrorMessage"))) {
        return false;
    }

    errorNumber = root.value(QStringLiteral("ErrorNumber")).toInt(0);
    errorMessage = root.value(QStringLiteral("ErrorMessage")).toString();
    return true;
}

bool CameraAlpacaController::parseImageBytesError(const QByteArray& payload, int& errorNumber)
{
    errorNumber = 0;

    if (payload.size() < 8) {
        return false;
    }

    const char *data = payload.constData();
    const qint32 metadataVersion = qFromLittleEndian<qint32>(data);

    if (metadataVersion != 1) {
        return false;
    }

    errorNumber = qFromLittleEndian<qint32>(data + 4);
    return true;
}

bool CameraAlpacaController::isDriverError(int errorNumber)
{
    return errorNumber >= 1024;
}

bool CameraAlpacaController::isOptionalCapabilityPath(const QString& path)
{
    const QString prop = path.section('/', -1).toLower();
    return (prop == QStringLiteral("gains"))
        || (prop == QStringLiteral("gainmin"))
        || (prop == QStringLiteral("gainmax"))
        || (prop == QStringLiteral("offsets"))
        || (prop == QStringLiteral("offsetmin"))
        || (prop == QStringLiteral("offsetmax"))
        || (prop == QStringLiteral("readoutmodes"))
        || (prop == QStringLiteral("ccdtemperature"));
}

QString CameraAlpacaController::baseUrl(const CameraSettings& settings)
{
    return QString("http://%1:%2")
        .arg(settings.m_alpacaHost)
        .arg(settings.m_alpacaPort);
}

QString CameraAlpacaController::focuserBaseUrl(const CameraSettings& settings)
{
    return QString("http://%1:%2")
        .arg(settings.m_alpacaFocuserHost)
        .arg(settings.m_alpacaFocuserPort);
}

QString CameraAlpacaController::filterWheelBaseUrl(const CameraSettings& settings)
{
    return QString("http://%1:%2")
        .arg(settings.m_alpacaFilterWheelHost)
        .arg(settings.m_alpacaFilterWheelPort);
}

QString CameraAlpacaController::transportError(QNetworkReply *reply)
{
    if (reply->error() == QNetworkReply::NoError) {
        return "";
    } else {
        return QString("%1 %2").arg(QMetaEnum::fromType<QNetworkReply::NetworkError>().valueToKey(reply->error())).arg(reply->errorString());
    }
}

QImage CameraAlpacaController::renderGrayscaleRaw(const QVector<QVector<int>>& raw, int width, int height, bool use16Bit)
{
    int minValue = std::numeric_limits<int>::max();
    int maxValue = std::numeric_limits<int>::min();

    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            minValue = std::min(minValue, raw[x][y]);
            maxValue = std::max(maxValue, raw[x][y]);
        }
    }

    const int range = maxValue - minValue;
    if (use16Bit)
    {
        const int uniformGray = (range == 0) ? (minValue > 0 ? 32768 : 0) : 0;
        QImage image(width, height, QImage::Format_Grayscale16);
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                const int value = (range > 0)
                    ? qBound(0, static_cast<int>(((raw[x][y] - minValue) * 65535.0) / range), 65535)
                    : uniformGray;
                reinterpret_cast<quint16*>(image.scanLine(y))[x] = static_cast<quint16>(value);
            }
        }
        return image;
    }

    const int uniformGray = (range == 0) ? (minValue > 0 ? 128 : 0) : 0;
    QImage image(width, height, QImage::Format_Grayscale8);
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            const int value = (range > 0)
                ? qBound(0, static_cast<int>(((raw[x][y] - minValue) * 255.0) / range), 255)
                : uniformGray;
            image.scanLine(y)[x] = static_cast<uchar>(value);
        }
    }

    return image;
}

QImage CameraAlpacaController::parseImageArray(const QByteArray& payload, const QImage& fallbackImage, QString *receiveImageFormat,
    CameraPipelineFrame::BayerPattern *bayerPattern) const
{
    if (bayerPattern) {
        *bayerPattern = CameraPipelineFrame::BayerNone;
    }

    const QJsonDocument doc = QJsonDocument::fromJson(payload);
    if (!doc.isObject()) {
        return fallbackImage;
    }

    const QJsonObject root = doc.object();

    const int errorNumber = root.value("ErrorNumber").toInt(0);
    if (errorNumber != 0) {
        qDebug() << "CameraAlpacaController::parseImageArray: Alpaca error" << errorNumber
                 << root.value("ErrorMessage").toString();
        return fallbackImage;
    }

    const int rank = root.value("Rank").toInt(0);
    const QJsonArray value = root.value("Value").toArray();

    if (value.isEmpty()) {
        return fallbackImage;
    }

    // Alpaca imagearray layout:
    //   Rank 2 (monochrome or Bayer): Value[column][row]
    //   Rank 3 (colour):              Value[plane][column][row], plane 0=R, 1=G, 2=B
    if (rank == 2)
    {
        const int width = value.size();
        if (width == 0) {
            return fallbackImage;
        }
        const QJsonArray firstCol = value[0].toArray();
        const int height = firstCol.size();
        if (height == 0) {
            return fallbackImage;
        }

        // First pass: find minimum and maximum pixel values for black-level correction and scaling.
        int minVal = std::numeric_limits<int>::max();
        int maxVal = std::numeric_limits<int>::min();
        for (const QJsonValue& col : value) {
            for (const QJsonValue& pix : col.toArray()) {
                const int v = pix.toInt(0);
                if (v < minVal) { minVal = v; }
                if (v > maxVal) { maxVal = v; }
            }
        }
        const bool use16Bit = (minVal < 0) || (maxVal > 255);
        if (receiveImageFormat) {
            *receiveImageFormat = QStringLiteral("ImageArray rank2 %1")
                .arg(use16Bit ? QStringLiteral("16-bit") : QStringLiteral("8-bit"));
        }

        QVector<QVector<int>> raw(width, QVector<int>(height, 0));
        for (int x = 0; x < width; ++x) {
            const QJsonArray col = value[x].toArray();
            for (int y = 0; y < height; ++y) {
                raw[x][y] = col[y].toInt(0);
            }
        }

        // Bayer demosaicing for sensorType 2 (RGGB), 3 (CMYG), 4 (CMYG2), 5 (LRGB)
        // sensorType 0 = Monochrome, 1 = Colour (handled by rank 3 normally)
        return renderRawPixelArray(raw, width, height, use16Bit, bayerPattern);
    }
    else if (rank == 3)
    {
        if (value.size() < 3) {
            return fallbackImage;
        }
        const QJsonArray planeR = value[0].toArray();
        const QJsonArray planeG = value[1].toArray();
        const QJsonArray planeB = value[2].toArray();

        const int width = planeR.size();
        if (width == 0) {
            return fallbackImage;
        }
        const int height = planeR[0].toArray().size();
        if (height == 0) {
            return fallbackImage;
        }

        // First pass: find minimum and maximum pixel values for black-level correction and scaling.
        int minVal = std::numeric_limits<int>::max();
        int maxVal = std::numeric_limits<int>::min();
        for (const QJsonArray* plane : {&planeR, &planeG, &planeB}) {
            for (const QJsonValue& col : *plane) {
                for (const QJsonValue& pix : col.toArray()) {
                    const int v = pix.toInt(0);
                    if (v < minVal) { minVal = v; }
                    if (v > maxVal) { maxVal = v; }
                }
            }
        }
        const int range3 = maxVal - minVal;
        const bool use16Bit = (minVal < 0) || (maxVal > 255);
        if (receiveImageFormat) {
            *receiveImageFormat = QStringLiteral("ImageArray rank3 %1")
                .arg(use16Bit ? QStringLiteral("16-bit") : QStringLiteral("8-bit"));
        }
        const double scale = (range3 > 0) ? ((use16Bit ? 65535.0 : 255.0) / range3) : 0.0;
        const int uniformGray3 = (range3 == 0) ? (minVal > 0 ? (use16Bit ? 32768 : 128) : 0) : 0;

        QImage image(width, height, use16Bit ? QImage::Format_RGBA64 : QImage::Format_RGB32);
        for (int x = 0; x < width; ++x) {
            const QJsonArray colR = planeR[x].toArray();
            const QJsonArray colG = planeG[x].toArray();
            const QJsonArray colB = planeB[x].toArray();
            for (int y = 0; y < height; ++y) {
                const int maxComponent = use16Bit ? 65535 : 255;
                const int r = (range3 > 0) ? qBound(0, static_cast<int>((colR[y].toInt(0) - minVal) * scale), maxComponent) : uniformGray3;
                const int g = (range3 > 0) ? qBound(0, static_cast<int>((colG[y].toInt(0) - minVal) * scale), maxComponent) : uniformGray3;
                const int b = (range3 > 0) ? qBound(0, static_cast<int>((colB[y].toInt(0) - minVal) * scale), maxComponent) : uniformGray3;

                if (use16Bit) {
                    reinterpret_cast<QRgba64*>(image.scanLine(y))[x] = qRgba64(r, g, b, 65535);
                } else {
                    reinterpret_cast<QRgb*>(image.scanLine(y))[x] = qRgb(r, g, b);
                }
            }
        }
        return image;
    }

    return fallbackImage;
}
QImage CameraAlpacaController::renderRawPixelArray(const QVector<QVector<int>>& raw, int width, int height, bool use16Bit,
    CameraPipelineFrame::BayerPattern *bayerPattern) const
{
    int minValue = std::numeric_limits<int>::max();
    int maxValue = std::numeric_limits<int>::min();

    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            minValue = std::min(minValue, raw[x][y]);
            maxValue = std::max(maxValue, raw[x][y]);
        }
    }

    if (m_sensorType == 2)
    {
        const int phaseX = ((m_bayerOffsetX % 2) + 2) % 2;
        const int phaseY = ((m_bayerOffsetY % 2) + 2) % 2;

        if (bayerPattern)
        {
            if ((phaseX == 0) && (phaseY == 0)) {
                *bayerPattern = CameraPipelineFrame::BayerRGGB;
            } else if ((phaseX == 1) && (phaseY == 0)) {
                *bayerPattern = CameraPipelineFrame::BayerGRBG;
            } else if ((phaseX == 0) && (phaseY == 1)) {
                *bayerPattern = CameraPipelineFrame::BayerGBRG;
            } else {
                *bayerPattern = CameraPipelineFrame::BayerBGGR;
            }
        }

        return renderGrayscaleRaw(raw, width, height, use16Bit);
    }

    // Monochrome (sensorType 0 or 1 returning rank 2, or unsupported Bayer types 3-5)
    return renderGrayscaleRaw(raw, width, height, use16Bit);
}

// Alpaca ImageBytes binary format (ASCOM Alpaca spec):
// https://ascom-standards.org/api/?urls.primaryName=ASCOM%20Alpaca%20Device%20API#/Camera/get__device_type___device_number__imagearray
//
//   Byte  0- 3: MetaDataVersion  (int32 LE) â€” must be 1
//   Byte  4- 7: ErrorNumber      (int32 LE) â€” 0 = success
//   Byte  8-11: ClientTransactionID (int32 LE)
//   Byte 12-15: ServerTransactionID (int32 LE)
//   Byte 16-19: DataStart        (int32 LE) â€” byte offset to pixel data, typically 44
//   Byte 20-23: ImageElementType (int32 LE) â€” original ADU element type (ASCOM ImageArrayElementTypes enum)
//   Byte 24-27: TransmissionElementType (int32 LE) â€” wire type (same enum)
//   Byte 28-31: Rank             (int32 LE) â€” 2 or 3
//   Byte 32-35: Dimension1       (int32 LE) â€” rank2: width; rank3: number of planes
//   Byte 36-39: Dimension2       (int32 LE) â€” rank2: height; rank3: width
//   Byte 40-43: Dimension3       (int32 LE) â€” rank2: unused (0); rank3: height
//
// ASCOM ImageArrayElementTypes enum:
//   1=Int16, 2=Int32, 3=Double, 4=Single, 5=UInt64, 6=Byte, 7=Int64, 8=UInt16, 9=UInt32
//
// Pixel data is column-major within each rank-2 plane: pixel[x][y] at index (x*height + y)
// For rank 3: pixel[plane][x][y] at index (plane*width*height + x*height + y)
QImage CameraAlpacaController::parseImageBytes(const QByteArray& payload, const QImage& fallbackImage, QString *receiveImageFormat,
    CameraPipelineFrame::BayerPattern *bayerPattern) const
{
    if (bayerPattern) {
        *bayerPattern = CameraPipelineFrame::BayerNone;
    }

    static constexpr int    kHeaderSize         = 44;
    // ASCOM ImageArrayElementTypes enum values
    static constexpr qint32 kElementTypeInt16   = 1;
    static constexpr qint32 kElementTypeInt32   = 2;
    static constexpr qint32 kElementTypeDouble  = 3;
    static constexpr qint32 kElementTypeSingle  = 4;
    static constexpr qint32 kElementTypeByte    = 6;
    static constexpr qint32 kElementTypeUInt16  = 8;

    if (payload.size() < kHeaderSize) {
        qDebug() << "CameraAlpacaController::parseImageBytes: payload too small" << payload.size();
        return fallbackImage;
    }

    const char* hdr = payload.constData();

    const qint32 metadataVersion  = qFromLittleEndian<qint32>(hdr + 0);
    const qint32 errorNumber      = qFromLittleEndian<qint32>(hdr + 4);
    // clientTransactionID        = qFromLittleEndian<qint32>(hdr + 8);  // informational
    // serverTransactionID        = qFromLittleEndian<qint32>(hdr + 12); // informational
    const qint32 dataStart        = qFromLittleEndian<qint32>(hdr + 16);
    const qint32 imageElementType = qFromLittleEndian<qint32>(hdr + 20);
    const qint32 transmissionType = qFromLittleEndian<qint32>(hdr + 24);
    const qint32 rank             = qFromLittleEndian<qint32>(hdr + 28);
    const qint32 dim1             = qFromLittleEndian<qint32>(hdr + 32);
    const qint32 dim2             = qFromLittleEndian<qint32>(hdr + 36);
    const qint32 dim3             = qFromLittleEndian<qint32>(hdr + 40);

    if (metadataVersion != 1) {
        qDebug() << "CameraAlpacaController::parseImageBytes: unsupported MetaDataVersion" << metadataVersion;
        return fallbackImage;
    }

    if (errorNumber != 0) {
        qDebug() << "CameraAlpacaController::parseImageBytes: Alpaca error" << errorNumber;
        return fallbackImage;
    }

    if (dataStart < kHeaderSize || dataStart > payload.size()) {
        qDebug() << "CameraAlpacaController::parseImageBytes: invalid DataStart" << dataStart;
        return fallbackImage;
    }

    int elementSize = 0;
    switch (transmissionType) {
        case kElementTypeInt16:  elementSize = 2; break;
        case kElementTypeInt32:  elementSize = 4; break;
        case kElementTypeDouble: elementSize = 8; break;
        case kElementTypeSingle: elementSize = 4; break;
        case kElementTypeByte:   elementSize = 1; break;
        case kElementTypeUInt16: elementSize = 2; break;
      default:
            qDebug() << "CameraAlpacaController::parseImageBytes: unknown TransmissionElementType" << transmissionType;
            return fallbackImage;
    }

    auto elementTypeName = [](qint32 elementType) -> QString {
        switch (elementType) {
        case kElementTypeInt16: return QStringLiteral("Int16");
        case kElementTypeInt32: return QStringLiteral("Int32");
        case kElementTypeDouble: return QStringLiteral("Double");
        case kElementTypeSingle: return QStringLiteral("Single");
        case kElementTypeByte: return QStringLiteral("Byte");
        case kElementTypeUInt16: return QStringLiteral("UInt16");
        default: return QStringLiteral("Unknown");
        }
    };

    const char*   pixels         = payload.constData() + dataStart;
    const qsizetype pixelDataLen = payload.size() - dataStart;

    // Read one pixel value as double (any supported element type) at a given byte offset.
    // Using double avoids overflow during the min/max scan and subsequent scaling.
    auto readPixelAsDouble = [&](qsizetype byteOffset) -> double {
        if (byteOffset + elementSize > pixelDataLen) {
            return 0.0;
        }
        const char* p = pixels + byteOffset;
        switch (transmissionType) {
            case kElementTypeInt16:
                return static_cast<double>(qFromLittleEndian<qint16>(p));
            case kElementTypeInt32:
                return static_cast<double>(qFromLittleEndian<qint32>(p));
            case kElementTypeDouble: {
                // Use memcpy + assume little-endian host (x86/x64/ARM â€” all Qt-supported platforms).
                // Qt does not provide qFromLittleEndian<double> for all versions.
                double v;
                std::memcpy(&v, p, sizeof(v));
                return v;
            }
            case kElementTypeSingle: {
                float v;
                std::memcpy(&v, p, sizeof(v));
                return static_cast<double>(v);
            }
            case kElementTypeByte:
                return static_cast<double>(static_cast<quint8>(*p));
            case kElementTypeUInt16:
                return static_cast<double>(qFromLittleEndian<quint16>(p));
            default:
                return 0.0;
        }
    };

    if (rank == 2)
    {
        const int width  = static_cast<int>(dim1);
        const int height = static_cast<int>(dim2);
        if (width <= 0 || height <= 0) {
            return fallbackImage;
        }
        const qsizetype required = static_cast<qsizetype>(width) * height * elementSize;
        if (pixelDataLen < required) {
            qDebug() << "CameraAlpacaController::parseImageBytes: insufficient pixel data for rank 2:"
                     << pixelDataLen << "<" << required;
            return fallbackImage;
        }

        // First pass: min/max for black-level correction and linear scaling to 8-bit
        double minVal = std::numeric_limits<double>::max();
        double maxVal = std::numeric_limits<double>::lowest();
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                const double v = readPixelAsDouble(static_cast<qsizetype>(x * height + y) * elementSize);
                if (v < minVal) { minVal = v; }
                if (v > maxVal) { maxVal = v; }
            }
        }
        const bool use16Bit = (imageElementType == kElementTypeInt16)
                           || (imageElementType == kElementTypeUInt16)
                           || (transmissionType == kElementTypeInt16)
                           || (transmissionType == kElementTypeUInt16)
                           || (minVal < 0.0)
                           || (maxVal > 255.0);
        if (receiveImageFormat) {
            *receiveImageFormat = QStringLiteral("ImageBytes rank2 %1/%2%3")
                .arg(elementTypeName(imageElementType),
                     elementTypeName(transmissionType),
                     use16Bit ? QStringLiteral(" 16-bit") : QString());
        }

        QVector<QVector<int>> raw(width, QVector<int>(height, 0));
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                const double v = readPixelAsDouble(static_cast<qsizetype>(x * height + y) * elementSize);
                raw[x][y] = qRound(v);
            }
        }

        return renderRawPixelArray(raw, width, height, use16Bit, bayerPattern);
    }
    else if (rank == 3)
    {
        const int planes = static_cast<int>(dim1);
        const int width  = static_cast<int>(dim2);
        const int height = static_cast<int>(dim3);
        if (planes < 3 || width <= 0 || height <= 0) {
            return fallbackImage;
        }
        const qsizetype required = static_cast<qsizetype>(planes) * width * height * elementSize;
        if (pixelDataLen < required) {
            qDebug() << "CameraAlpacaController::parseImageBytes: insufficient pixel data for rank 3:"
                     << pixelDataLen << "<" << required;
            return fallbackImage;
        }

        auto pixelAt = [&](int plane, int x, int y) -> double {
            return readPixelAsDouble(static_cast<qsizetype>(plane * width * height + x * height + y) * elementSize);
        };

        double minVal = std::numeric_limits<double>::max();
        double maxVal = std::numeric_limits<double>::lowest();
        for (int p = 0; p < 3; ++p) {
            for (int x = 0; x < width; ++x) {
                for (int y = 0; y < height; ++y) {
                    const double v = pixelAt(p, x, y);
                    if (v < minVal) { minVal = v; }
                    if (v > maxVal) { maxVal = v; }
                }
            }
        }
        const double range3 = maxVal - minVal;
        const bool use16Bit = (imageElementType == kElementTypeInt16)
                           || (imageElementType == kElementTypeUInt16)
                           || (transmissionType == kElementTypeInt16)
                           || (transmissionType == kElementTypeUInt16)
                           || (minVal < 0.0)
                           || (maxVal > 255.0);
        if (receiveImageFormat) {
            *receiveImageFormat = QStringLiteral("ImageBytes rank3 %1/%2%3")
                .arg(elementTypeName(imageElementType),
                     elementTypeName(transmissionType),
                     use16Bit ? QStringLiteral(" 16-bit") : QString());
        }
        const double scale = (range3 > 0.0) ? ((use16Bit ? 65535.0 : 255.0) / range3) : 0.0;
        const int uniformGray3 = (range3 == 0.0) ? (minVal > 0.0 ? (use16Bit ? 32768 : 128) : 0) : 0;

        QImage image(width, height, use16Bit ? QImage::Format_RGBA64 : QImage::Format_RGB32);
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                const int maxComponent = use16Bit ? 65535 : 255;
                const int r = (range3 > 0.0)
                    ? qBound(0, static_cast<int>((pixelAt(0, x, y) - minVal) * scale), maxComponent)
                    : uniformGray3;
                const int g = (range3 > 0.0)
                    ? qBound(0, static_cast<int>((pixelAt(1, x, y) - minVal) * scale), maxComponent)
                    : uniformGray3;
                const int b = (range3 > 0.0)
                    ? qBound(0, static_cast<int>((pixelAt(2, x, y) - minVal) * scale), maxComponent)
                    : uniformGray3;

                if (use16Bit) {
                    reinterpret_cast<QRgba64*>(image.scanLine(y))[x] = qRgba64(r, g, b, 65535);
                } else {
                    reinterpret_cast<QRgb*>(image.scanLine(y))[x] = qRgb(r, g, b);
                }
            }
        }
        return image;
    }

    return fallbackImage;
}
