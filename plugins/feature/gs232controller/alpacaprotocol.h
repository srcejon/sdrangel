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

#ifndef INCLUDE_FEATURE_ALPACAPROTOCOL_H_
#define INCLUDE_FEATURE_ALPACAPROTOCOL_H_

#include <functional>

#include <QObject>
#include <QUrl>
#include <QUrlQuery>

#include "controllerprotocol.h"

class QJsonObject;
class QNetworkAccessManager;
class QNetworkReply;

class AlpacaProtocol : public QObject, public ControllerProtocol
{
    Q_OBJECT
public:
    AlpacaProtocol();
    ~AlpacaProtocol();

    void setAzimuthElevation(float azimuth, float elevation) override;
    void readData() override;
    void update() override;
    bool usesIODevice() const override { return false; }
    void applySettings(const GS232ControllerSettings& settings, const QList<QString>& settingsKeys, bool force) override;

private:
    QString baseUrl() const;
    QUrl deviceUrl(const QString& property) const;
    QUrlQuery transactionQuery();
    bool parseAlpacaResponse(QNetworkReply *reply, const QByteArray& payload, QJsonObject& object, const QString& context);
    void runWhenConnected(const std::function<void()>& continuation);
    void setConnected(bool connected, const std::function<void(bool)>& continuation = {});
    void slewToAltAz(float azimuth, float elevation);
    void pollAzimuthAltitude();
    void handlePositionReply(const QString& property, QNetworkReply *reply, double& value, bool& valid, const std::function<void()>& checkDone);

    QNetworkAccessManager *m_networkManager;
    quint32 m_clientId;
    quint32 m_clientTransactionId;
    bool m_connected;
    bool m_connectionPending;
    bool m_slewPending;
    QList<std::function<void()>> m_pendingConnectedContinuations;
};

#endif // INCLUDE_FEATURE_ALPACAPROTOCOL_H_
