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

#ifndef INCLUDE_FEATURE_AIRCRAFT_H_
#define INCLUDE_FEATURE_AIRCRAFT_H_

#include "feature/feature.h"
#include "util/message.h"
#include "availablechannelorfeaturehandler.h"

#include "aircraftsettings.h"
#include "aircrafttracker.h"

class WebAPIAdapterInterface;
class QThread;

// Collates aircraft data from the ADS-B and ACARS (VHF/VDL-2/HFDL/Aero) demodulators
// into a combined aircraft/flight model, and is the source of aircraft items for
// the Map feature. Demodulators send MainCore::MsgAircraftReport over the
// "aircraftreport" message pipe; the model itself lives in the GUI (like the AIS
// feature, decode-side data currently originates in the demodulator GUIs).
class Aircraft : public Feature
{
	Q_OBJECT
public:
    class MsgConfigureAircraft : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const AircraftSettings& getSettings() const { return m_settings; }
        const QList<QString>& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureAircraft* create(const AircraftSettings& settings, const QList<QString>& settingsKeys, bool force) {
            return new MsgConfigureAircraft(settings, settingsKeys, force);
        }

    private:
        AircraftSettings m_settings;
        QList<QString> m_settingsKeys;
        bool m_force;

        MsgConfigureAircraft(const AircraftSettings& settings, const QList<QString>& settingsKeys, bool force) :
            Message(),
            m_settings(settings),
            m_settingsKeys(settingsKeys),
            m_force(force)
        { }
    };

    Aircraft(WebAPIAdapterInterface *webAPIAdapterInterface);
    virtual ~Aircraft();
    virtual void destroy() { delete this; }
    virtual bool handleMessage(const Message& cmd);

    // The tracker runs on its own thread for the feature's lifetime; the GUI
    // connects to its signals and sends it messages
    AircraftTracker *getTracker() { return m_tracker; }

    virtual void getIdentifier(QString& id) const { id = objectName(); }
    virtual QString getIdentifier() const { return objectName(); }
    virtual void getTitle(QString& title) const { title = m_settings.m_title; }

    virtual QByteArray serialize() const;
    virtual bool deserialize(const QByteArray& data);

    static const char* const m_featureIdURI;
    static const char* const m_featureId;

private:
    AircraftSettings m_settings;
    AvailableChannelOrFeatureHandler m_availableChannelHandler;
    QThread *m_thread;
    AircraftTracker *m_tracker;

    void start();
    void stop();
    void applySettings(const AircraftSettings& settings, const QList<QString>& settingsKeys, bool force = false);

private slots:
    void handleChannelMessageQueue(MessageQueue* messageQueue);
};

#endif // INCLUDE_FEATURE_AIRCRAFT_H_
