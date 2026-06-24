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
#include <QElapsedTimer>
#include <QMutex>

#include "camerainfo.h"
#include "camerasettings.h"

class MessageQueue;
class QNetworkAccessManager;
class QTimer;
class QUdpSocket;

/**
 * \brief Discovers connected cameras (and Alpaca accessories) and reports them to the GUI.
 *
 * Enumerates available cameras from multiple backends — Qt Multimedia cameras, ZWO ASI cameras
 * (when the SDK is present) and ASCOM Alpaca devices — and reports the combined list, along with
 * discovered Alpaca focusers and filter wheels, back to the GUI message queue. Alpaca devices are
 * located both via configured host/port and via UDP broadcast discovery on the Alpaca discovery
 * port, then queried over HTTP for their configured device lists.
 *
 * \note A QObject that uses asynchronous UDP discovery and HTTP queries; reportCameraList() kicks
 *       off discovery and the result is delivered later through the GUI MessageQueue, not returned
 *       synchronously. A request id guards against overlapping/stale discovery rounds.
 */
class CameraFinder : public QObject
{
    Q_OBJECT
public:
    explicit CameraFinder(QObject* parent = nullptr);
    ~CameraFinder() override;

    void setMessageQueueToGUI(MessageQueue* messageQueue) { m_msgQueueToGUI = messageQueue; }
    void reportCameraList(const CameraSettings& settings, bool forceLocalRefresh = false);

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
    static constexpr qint64 m_localCameraCacheMaxAgeMs = 15000;
    static const QByteArray m_alpacaDiscoveryMessage;
    static QMutex m_localCameraCacheMutex;
    static QElapsedTimer m_localCameraCacheAge;
    static QList<CameraInfo> m_cachedLocalCameras;

    static QList<CameraInfo> listLocalCamerasCached(bool forceRefresh);
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
