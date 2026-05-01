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
#include <QTimer>
#include <QUrl>
#include <QUrlQuery>

#include "util/message.h"
#include "controllerprotocol.h"

class QJsonObject;
class QNetworkAccessManager;
class QNetworkReply;

class AlpacaProtocol : public QObject, public ControllerProtocol
{
    Q_OBJECT
public:
    class MsgReportParkState : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        bool canPark() const { return m_canPark; }
        bool atPark() const { return m_atPark; }
        bool valid() const { return m_valid; }

        static MsgReportParkState* create(bool canPark, bool atPark, bool valid)
        {
            return new MsgReportParkState(canPark, atPark, valid);
        }

    private:
        bool m_canPark;
        bool m_atPark;
        bool m_valid;

        MsgReportParkState(bool canPark, bool atPark, bool valid) :
            Message(),
            m_canPark(canPark),
            m_atPark(atPark),
            m_valid(valid)
        {
        }
    };

    AlpacaProtocol();
    ~AlpacaProtocol();

    void setAzimuthElevation(float azimuth, float elevation) override;
    void readData() override;
    void update() override;
    bool usesIODevice() const override { return false; }
    void park() override;
    void unpark() override;
    void applySettings(const GS232ControllerSettings& settings, const QList<QString>& settingsKeys, bool force) override;

private:
    QString baseUrl() const;
    QUrl deviceUrl(const QString& property) const;
    QUrlQuery transactionQuery();
    bool parseAlpacaResponse(QNetworkReply *reply, const QByteArray& payload, QJsonObject& object, const QString& context, bool reportErrors = true, int *errorNumber = nullptr);
    void runWhenConnected(const std::function<void()>& continuation);
    void setConnected(bool connected, const std::function<void(bool)>& continuation = {});
    void queryCapabilities(const std::function<void(bool)>& continuation);
    void getBoolProperty(const QString& property, bool *value, bool *valid, const std::function<void()>& checkDone);
    void slewToAltAz(float azimuth, float elevation);
    void queueSlew(float azimuth, float elevation);
    void scheduleSlewAttempt(int delayMs);
    void attemptQueuedSlew();
    void querySlewing(const std::function<void(bool, bool)>& continuation);
    void dispatchSlew(float azimuth, float elevation);
    void sendSlewCommand(const QString& method, const QUrlQuery& commandBody, const QString& context);
    void sendSimplePutCommand(const QString& method, const QString& context, const std::function<void(bool)>& continuation = {});
    void slewToRaDec(float azimuth, float elevation, bool asynchronous);
    void pollAzimuthAltitude();
    void queryAtPark(const std::function<void(bool, bool)>& continuation = {});
    void reportParkState(bool valid = true);
    void handlePositionReply(const QString& property, QNetworkReply *reply, double& value, bool& valid, const std::function<void()>& checkDone);

    QNetworkAccessManager *m_networkManager;
    quint32 m_clientId;
    quint32 m_clientTransactionId;
    bool m_connected;
    bool m_connectionPending;
    bool m_capabilitiesReady;
    bool m_capabilitiesPending;
    bool m_canSlewAltAzAsync;
    bool m_canSlewAltAz;
    bool m_canSlewAsync;
    bool m_canSlew;
    bool m_canPark;
    bool m_atPark;
    bool m_atParkValid;
    bool m_atParkQueryPending;
    bool m_slewPending;
    bool m_slewingQueryPending;
    bool m_queuedSlew;
    float m_queuedAzimuth;
    float m_queuedElevation;
    QTimer m_slewRetryTimer;
    QList<std::function<void()>> m_pendingConnectedContinuations;
};

#endif // INCLUDE_FEATURE_ALPACAPROTOCOL_H_
