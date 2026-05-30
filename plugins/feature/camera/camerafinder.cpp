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

#include <QDebug>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkAccessManager>
#include <QHostAddress>
#include <QFileInfo>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QTimer>
#include <QUdpSocket>
#include <QUrl>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QCameraDevice>
#include <QMediaDevices>
#else
#include <QCameraInfo>
#endif

#ifdef ASICAMERA_FOUND
#include <ASICamera2.h>
#endif

#include "util/messagequeue.h"
#ifdef ASICAMERA_FOUND
#include "cameraasisdk.h"
#endif
#include "camerafinder.h"
#include "cameraworker.h"

const QByteArray CameraFinder::m_alpacaDiscoveryMessage("alpacadiscovery1");

CameraFinder::CameraFinder(QObject* parent) :
    QObject(parent),
    m_msgQueueToGUI(nullptr),
    m_networkManager(nullptr),
    m_discoverySocket(nullptr),
    m_discoveryTimer(nullptr),
    m_requestId(0),
    m_pendingConfiguredDeviceReplies(0)
{
}

CameraFinder::~CameraFinder()
{
    delete m_discoveryTimer;
    delete m_discoverySocket;
    delete m_networkManager;
}

void CameraFinder::reportCameraList(const CameraSettings& settings)
{
    m_settings = settings;
    m_currentCameras = listQtCameras();
#ifdef ASICAMERA_FOUND
    m_currentCameras.append(listAsiCameras());
#endif
    m_currentFocusers.clear();
    m_currentFilterWheels.clear();
    m_discoveredEndpointKeys.clear();
    m_pendingConfiguredDeviceReplies = 0;
    ++m_requestId;

    if (!m_msgQueueToGUI) {
        return;
    }

    if (!m_networkManager) {
        m_networkManager = new QNetworkAccessManager(this);
    }

    if (settings.m_alpacaDiscoveryEnabled) {
        startAlpacaDiscovery();
    } else {
        queryConfiguredDevices({{settings.m_alpacaHost, settings.m_alpacaPort}}, m_requestId);
    }
}

QList<CameraInfo> CameraFinder::listQtCameras()
{
    QList<CameraInfo> qtCameras;

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();

    for (const QCameraDevice& camera : cameras)
    {
        qtCameras.append({
            QStringLiteral("qt"),
            QString::fromUtf8(camera.id()),
            camera.description(),
            {},
            0
        });
    }
#else
    const QList<QCameraInfo> cameras = QCameraInfo::availableCameras();

    for (const QCameraInfo& camera : cameras)
    {
        qtCameras.append({
            QStringLiteral("qt"),
            camera.deviceName(),
            camera.description(),
            {},
            0
        });
    }
#endif

    return qtCameras;
}

#ifdef ASICAMERA_FOUND
QList<CameraInfo> CameraFinder::listAsiCameras()
{
    CameraAsiSdkLocker asiSdkLocker;
    QList<CameraInfo> asiCameras;
    const int cameraCount = ASIGetNumOfConnectedCameras();
    const int boundedCameraCount = qBound(0, cameraCount, 64);

    if (cameraCount != boundedCameraCount) {
        qWarning() << "CameraFinder::listAsiCameras: unexpected ASI camera count" << cameraCount;
    }

    for (int index = 0; index < boundedCameraCount; ++index)
    {
        ASI_CAMERA_INFO camera {};

        if (ASIGetCameraProperty(&camera, index) != ASI_SUCCESS) {
            continue;
        }

        asiCameras.append({
            QStringLiteral("asi"),
            QString::number(camera.CameraID),
            QString::fromUtf8(camera.Name),
            {},
            0
        });
    }

    return asiCameras;
}
#endif

QList<AlpacaDeviceInfo> CameraFinder::parseAlpacaDeviceList(const QByteArray& payload, const QString& deviceType, const QString& host, quint16 port)
{
    QList<AlpacaDeviceInfo> result;
    const QJsonDocument doc = QJsonDocument::fromJson(payload);

    if (!doc.isObject()) {
        return result;
    }

    QJsonArray devices;
    const QJsonObject root = doc.object();

    if (root.contains("Value") && root.value("Value").isArray()) {
        devices = root.value("Value").toArray();
    } else if (root.contains("value") && root.value("value").isArray()) {
        devices = root.value("value").toArray();
    }

    for (const QJsonValue& value : devices)
    {
        if (!value.isObject()) {
            continue;
        }

        const QJsonObject obj = value.toObject();
        const QString type = obj.value("DeviceType").toString().toLower();

        if (type != deviceType) {
            continue;
        }

        const int number = obj.value("DeviceNumber").toInt(-1);
        const QString name = obj.value("DeviceName").toString();

        if (number >= 0) {
            result.append({
                deviceType,
                QString::number(number),
                name,
                host,
                port
            });
        }
    }

    return result;
}

QString CameraFinder::buildAlpacaBaseUrl(const CameraSettings& settings)
{
    return buildAlpacaBaseUrl(settings.m_alpacaHost, settings.m_alpacaPort);
}

QString CameraFinder::buildAlpacaBaseUrl(const QString& host, quint16 port)
{
    return QString("http://%1:%2").arg(host).arg(port);
}

QString CameraFinder::endpointKey(const QString& host, quint16 port)
{
    return QStringLiteral("%1:%2").arg(host).arg(port);
}

void CameraFinder::finalizeCameraList(int requestId)
{
    if (requestId != m_requestId || !m_msgQueueToGUI) {
        return;
    }

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    // Have file sources as the last entries
    m_currentCameras.append({
        QStringLiteral("file"),
        m_settings.m_videoFileCameraPath,
        m_settings.m_videoFileCameraPath.isEmpty() ? QString() : QFileInfo(m_settings.m_videoFileCameraPath).fileName(),
        {},
        0
        });
    m_currentCameras.append({
        QStringLiteral("files"),
        QStringLiteral("files"),
        m_settings.m_imageFileCameraPaths.isEmpty()
            ? QString()
            : QStringLiteral("%1 image%2")
                .arg(m_settings.m_imageFileCameraPaths.size())
                .arg(m_settings.m_imageFileCameraPaths.size() == 1 ? QString() : QStringLiteral("s")),
        {},
        0
        });
#endif

    m_msgQueueToGUI->push(CameraWorker::MsgReportCameraList::create(m_currentCameras));
    m_msgQueueToGUI->push(CameraWorker::MsgReportAlpacaDeviceList::create(m_currentFocusers, m_currentFilterWheels));
}

void CameraFinder::startAlpacaDiscovery()
{
    if (!m_discoverySocket)
    {
        m_discoverySocket = new QUdpSocket(this);
        connect(m_discoverySocket, &QUdpSocket::readyRead, this, &CameraFinder::readAlpacaDiscoveryResponses);
    }

    if (!m_discoveryTimer)
    {
        m_discoveryTimer = new QTimer(this);
        m_discoveryTimer->setSingleShot(true);
        connect(m_discoveryTimer, &QTimer::timeout, this, [this]() { finishAlpacaDiscovery(m_requestId); });
    }

    m_discoverySocket->close();
    m_discoveredEndpointKeys.clear();

    if (!m_discoverySocket->bind(QHostAddress::AnyIPv4, 0, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint))
    {
        qWarning() << "CameraFinder: failed to bind Alpaca discovery socket:" << m_discoverySocket->errorString();
    }
    else
    {
        m_discoverySocket->writeDatagram(m_alpacaDiscoveryMessage, QHostAddress::Broadcast, m_alpacaDiscoveryPort);
    }

    // Always start the timeout so discovery completes (reporting whatever was found, or
    // an empty list on bind failure) rather than leaving the caller waiting.
    m_discoveryTimer->start(m_alpacaDiscoveryTimeoutMs);
}

void CameraFinder::finishAlpacaDiscovery(int requestId)
{
    if (requestId != m_requestId) {
        return;
    }

    QList<AlpacaEndpoint> endpoints;

    for (auto it = m_discoveredEndpointKeys.cbegin(); it != m_discoveredEndpointKeys.cend(); ++it)
    {
        const QString& key = *it;
        const int separator = key.lastIndexOf(':');

        if (separator <= 0) {
            continue;
        }

        bool ok = false;
        const quint16 port = key.mid(separator + 1).toUShort(&ok);

        if (!ok) {
            continue;
        }

        endpoints.append({key.left(separator), port});
    }

    if (endpoints.isEmpty()) {
        endpoints.append({m_settings.m_alpacaHost, m_settings.m_alpacaPort});
    }

    queryConfiguredDevices(endpoints, requestId);
}

void CameraFinder::queryConfiguredDevices(const QList<AlpacaEndpoint>& endpoints, int requestId)
{
    if (requestId != m_requestId) {
        return;
    }

    QList<AlpacaEndpoint> uniqueEndpoints;
    QSet<QString> seenEndpoints;

    for (const AlpacaEndpoint& endpoint : endpoints)
    {
        if (endpoint.m_host.isEmpty() || endpoint.m_port == 0) {
            continue;
        }

        const QString key = endpointKey(endpoint.m_host, endpoint.m_port);

        if (!seenEndpoints.contains(key))
        {
            seenEndpoints.insert(key);
            uniqueEndpoints.append(endpoint);
        }
    }

    m_pendingConfiguredDeviceReplies = uniqueEndpoints.size();

    if (m_pendingConfiguredDeviceReplies == 0)
    {
        finalizeCameraList(requestId);
        return;
    }

    for (const AlpacaEndpoint& endpoint : uniqueEndpoints)
    {
        QNetworkRequest request(QUrl(buildAlpacaBaseUrl(endpoint.m_host, endpoint.m_port) + "/management/v1/configureddevices"));
        QNetworkReply* reply = m_networkManager->get(request);

        connect(reply, &QNetworkReply::finished, this, [this, reply, endpoint, requestId]() {
            if (requestId == m_requestId && reply->error() == QNetworkReply::NoError)
            {
                const QByteArray payload = reply->readAll();
                const QList<AlpacaDeviceInfo> cameras = parseAlpacaDeviceList(payload, QStringLiteral("camera"), endpoint.m_host, endpoint.m_port);
                for (const AlpacaDeviceInfo& camera : cameras)
                {
                    m_currentCameras.append({
                        QStringLiteral("alpaca"),
                        camera.m_id,
                        camera.m_description,
                        camera.m_host,
                        camera.m_port
                    });
                }
                m_currentFocusers.append(parseAlpacaDeviceList(payload, QStringLiteral("focuser"), endpoint.m_host, endpoint.m_port));
                m_currentFilterWheels.append(parseAlpacaDeviceList(payload, QStringLiteral("filterwheel"), endpoint.m_host, endpoint.m_port));
            }

            reply->deleteLater();

            if (requestId != m_requestId) {
                return;
            }

            --m_pendingConfiguredDeviceReplies;

            if (m_pendingConfiguredDeviceReplies <= 0) {
                finalizeCameraList(requestId);
            }
        });
    }
}

void CameraFinder::readAlpacaDiscoveryResponses()
{
    if (!m_discoverySocket) {
        return;
    }

    while (m_discoverySocket->hasPendingDatagrams())
    {
        QHostAddress sender;
        quint16 senderPort = 0;
        QByteArray payload;
        payload.resize(static_cast<int>(m_discoverySocket->pendingDatagramSize()));
        m_discoverySocket->readDatagram(payload.data(), payload.size(), &sender, &senderPort);

        Q_UNUSED(senderPort)

        const QJsonDocument doc = QJsonDocument::fromJson(payload);

        if (!doc.isObject()) {
            continue;
        }

        const QJsonObject obj = doc.object();
        const QJsonValue portValue = obj.value(QStringLiteral("AlpacaPort"));

        if (!portValue.isDouble()) {
            continue;
        }

        const int alpacaPort = portValue.toInt(0);

        if (alpacaPort <= 0 || alpacaPort > 65535) {
            continue;
        }

        m_discoveredEndpointKeys.insert(endpointKey(sender.toString(), static_cast<quint16>(alpacaPort)));
    }
}
