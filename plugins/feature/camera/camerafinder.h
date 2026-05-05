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

#ifndef INCLUDE_FEATURE_CAMERAFINDER_H_
#define INCLUDE_FEATURE_CAMERAFINDER_H_

#include <QObject>
#include <QSet>

#include "camerainfo.h"
#include "camerasettings.h"

class MessageQueue;
class QNetworkAccessManager;
class QTimer;
class QUdpSocket;

class CameraFinder : public QObject
{
    Q_OBJECT
public:
    explicit CameraFinder(QObject* parent = nullptr);
    ~CameraFinder() override;

    void setMessageQueueToGUI(MessageQueue* messageQueue) { m_msgQueueToGUI = messageQueue; }
    void reportCameraList(const CameraSettings& settings);

private:
    struct AlpacaEndpoint
    {
        QString m_host;
        quint16 m_port;
    };

    MessageQueue* m_msgQueueToGUI;
    QNetworkAccessManager* m_networkManager;
    QUdpSocket* m_discoverySocket;
    QTimer* m_discoveryTimer;
    int m_requestId;
    int m_pendingConfiguredDeviceReplies;
    CameraSettings m_settings;
    QList<CameraInfo> m_currentCameras;
    QList<AlpacaDeviceInfo> m_currentFocusers;
    QList<AlpacaDeviceInfo> m_currentFilterWheels;
    QSet<QString> m_discoveredEndpointKeys;

    static constexpr quint16 m_alpacaDiscoveryPort = 32227;
    static constexpr int m_alpacaDiscoveryTimeoutMs = 1000;
    static const QByteArray m_alpacaDiscoveryMessage;

    static QList<CameraInfo> listQtCameras();
#ifdef ASICAMERA_FOUND
    static QList<CameraInfo> listAsiCameras();
#endif
    static QList<AlpacaDeviceInfo> parseAlpacaDeviceList(const QByteArray& payload, const QString& deviceType, const QString& host, quint16 port);
    static QString buildAlpacaBaseUrl(const CameraSettings& settings);
    static QString buildAlpacaBaseUrl(const QString& host, quint16 port);
    static QString endpointKey(const QString& host, quint16 port);

    void finalizeCameraList(int requestId);
    void startAlpacaDiscovery();
    void finishAlpacaDiscovery(int requestId);
    void queryConfiguredDevices(const QList<AlpacaEndpoint>& endpoints, int requestId);

private slots:
    void readAlpacaDiscoveryResponses();
};

#endif // INCLUDE_FEATURE_CAMERAFINDER_H_
