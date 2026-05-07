///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019-2020 Edouard Griffiths, F4EXB <f4exb06@gmail.com>          //
// Copyright (C) 2020, 2023 Jon Beniston, M7RCE <jon@beniston.com>               //
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

#ifndef INCLUDE_FEATURE_CONTROLLERPROTOCOL_H_
#define INCLUDE_FEATURE_CONTROLLERPROTOCOL_H_

#include <QIODevice>
#include <QDateTime>

#include "util/messagequeue.h"
#include "util/message.h"

#include "gs232controllersettings.h"
#include "gs232controller.h"

class ControllerProtocol
{
public:

    class MsgReportParkState : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        bool canPark() const { return m_canPark; }
        bool atPark() const { return m_atPark; }
        bool parkValid() const { return m_parkValid; }
        bool canFindHome() const { return m_canFindHome; }
        bool atHome() const { return m_atHome; }
        bool homeValid() const { return m_homeValid; }
        bool valid() const { return m_parkValid; }

        static MsgReportParkState* create(bool canPark, bool atPark, bool parkValid, bool canFindHome, bool atHome, bool homeValid)
        {
            return new MsgReportParkState(canPark, atPark, parkValid, canFindHome, atHome, homeValid);
        }

    private:
        bool m_canPark;
        bool m_atPark;
        bool m_parkValid;
        bool m_canFindHome;
        bool m_atHome;
        bool m_homeValid;

        MsgReportParkState(bool canPark, bool atPark, bool parkValid, bool canFindHome, bool atHome, bool homeValid) :
            Message(),
            m_canPark(canPark),
            m_atPark(atPark),
            m_parkValid(parkValid),
            m_canFindHome(canFindHome),
            m_atHome(atHome),
            m_homeValid(homeValid)
        {
        }
    };

    class MsgReportPositionMismatch : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        double rotatorLatitude() const { return m_rotatorLatitude; }
        double rotatorLongitude() const { return m_rotatorLongitude; }
        double rotatorElevation() const { return m_rotatorElevation; }
        bool rotatorAltitudeValue() const { return m_rotatorAltitudeValid; }
        double localLatitude() const { return m_localLatitude; }
        double localLongitude() const { return m_localLongitude; }
        double localElevation() const { return m_localElevation; }

        static MsgReportPositionMismatch* create(
            double rotatorLatitude,
            double rotatorLongitude,
            double rotatorElevation,
            bool rotatorAltitudeValid,
            double localLatitude,
            double localLongitude,
            double localElevation)
        {
            return new MsgReportPositionMismatch(
                rotatorLatitude,
                rotatorLongitude,
                rotatorElevation,
                rotatorAltitudeValid,
                localLatitude,
                localLongitude,
                localElevation);
        }

    private:
        double m_rotatorLatitude;
        double m_rotatorLongitude;
        double m_rotatorElevation;
        bool m_rotatorAltitudeValid;
        double m_localLatitude;
        double m_localLongitude;
        double m_localElevation;

        MsgReportPositionMismatch(
            double rotatorLatitude,
            double rotatorLongitude,
            double rotatorElevation,
            bool rotatorAltitudeValid,
            double localLatitude,
            double localLongitude,
            double localElevation) :
            Message(),
            m_rotatorLatitude(rotatorLatitude),
            m_rotatorLongitude(rotatorLongitude),
            m_rotatorElevation(rotatorElevation),
            m_rotatorAltitudeValid(rotatorAltitudeValid),
            m_localLatitude(localLatitude),
            m_localLongitude(localLongitude),
            m_localElevation(localElevation)
        {
        }
    };

    class MsgReportDateTimeMismatch : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        QDateTime rotatorUtcDate() const { return m_rotatorUtcDate; }
        QDateTime localUtcDate() const { return m_localUtcDate; }

        static MsgReportDateTimeMismatch* create(
            const QDateTime& rotatorUtcDate,
            const QDateTime& localUtcDate)
        {
            return new MsgReportDateTimeMismatch(
                rotatorUtcDate,
                localUtcDate);
        }

    private:
        QDateTime m_rotatorUtcDate;
        QDateTime m_localUtcDate;

        MsgReportDateTimeMismatch(
            const QDateTime& rotatorUtcDate,
            const QDateTime& localUtcDate) :
            Message(),
            m_rotatorUtcDate(rotatorUtcDate),
            m_localUtcDate(localUtcDate)
        {
        }
    };

    ControllerProtocol();
    virtual ~ControllerProtocol();
    virtual void setAzimuth(float azimuth);
    virtual void setAzimuthElevation(float azimuth, float elevation) = 0;
    virtual void readData() = 0;
    virtual void update() = 0;
    virtual bool usesIODevice() const { return true; }
    virtual void park() {}
    virtual void unpark() {}
    virtual void home() {}
    virtual void setPosition(double latitude, double longitude, double elevation) { (void) latitude; (void) longitude; (void) elevation; }
    virtual void setDateTime(const QDateTime& utcDate) { (void) utcDate; }
    void setDevice(QIODevice *device) { m_device = device; }
    virtual void applySettings(const GS232ControllerSettings& settings, const QList<QString>& settingsKeys, bool force);
    void setMessageQueue(MessageQueue *messageQueue) { m_msgQueueToFeature = messageQueue; }
    void sendMessage(Message *message);
    void reportAzEl(float az, float el);
    void reportError(const QString &message);
    void getPosition(float& latitude, float& longitude);

    static ControllerProtocol *create(GS232ControllerSettings::Protocol protocol);

protected:
    QIODevice *m_device;
    GS232ControllerSettings m_settings;
    float m_lastAzimuth;
    float m_lastElevation;

private:
    MessageQueue *m_msgQueueToFeature;
};

#endif // INCLUDE_FEATURE_CONTROLLERPROTOCOL_H_
