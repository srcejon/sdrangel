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

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QUrl>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QCameraDevice>
#include <QMediaDevices>
#else
#include <QCameraInfo>
#endif

#include "util/messagequeue.h"
#include "camerafinder.h"
#include "cameraworker.h"

CameraFinder::CameraFinder(QObject* parent) :
    QObject(parent),
    m_msgQueueToGUI(nullptr),
    m_networkManager(nullptr)
{
}

CameraFinder::~CameraFinder()
{
    delete m_networkManager;
}

void CameraFinder::reportCameraList(const CameraSettings& settings)
{
    const QStringList qtCameraIds = listQtCameraIds();

    if (!m_msgQueueToGUI) {
        return;
    }

    if (!m_networkManager) {
        m_networkManager = new QNetworkAccessManager(this);
    }

    QNetworkRequest request(QUrl(buildAlpacaBaseUrl(settings) + "/management/v1/configureddevices"));
    QNetworkReply* reply = m_networkManager->get(request);

    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply, qtCameraIds]() {
        QStringList cameraIds = qtCameraIds;

        if (reply->error() == QNetworkReply::NoError) {
            cameraIds.append(parseAlpacaCameraList(reply->readAll()));
        }

        if (m_msgQueueToGUI) {
            m_msgQueueToGUI->push(CameraWorker::MsgReportCameraList::create(cameraIds));
        }

        reply->deleteLater();
    });
}

QStringList CameraFinder::listQtCameraIds()
{
    QStringList qtCameraIds;

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();

    for (const QCameraDevice& camera : cameras)
    {
        const QString id = QString("qt:%1:%2").arg(QString::fromUtf8(camera.id())).arg(camera.description());
        qtCameraIds.append(id);
    }
#else
    const QList<QCameraInfo> cameras = QCameraInfo::availableCameras();

    for (const QCameraInfo& info : cameras)
    {
        const QString id = QString("qt:%1:%2").arg(info.deviceName()).arg(info.description());
        qtCameraIds.append(id);
    }
#endif

    return qtCameraIds;
}

QStringList CameraFinder::parseAlpacaCameraList(const QByteArray& payload)
{
    QStringList result;
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

        if (type != "camera") {
            continue;
        }

        const int number = obj.value("DeviceNumber").toInt(-1);
        const QString name = obj.value("DeviceName").toString();

        if (number >= 0) {
            result.append(QString("alpaca:%1:%2").arg(number).arg(name));
        }
    }

    return result;
}

QString CameraFinder::buildAlpacaBaseUrl(const CameraSettings& settings)
{
    return QString("http://%1:%2")
        .arg(settings.m_alpacaHost)
        .arg(settings.m_alpacaPort);
}
