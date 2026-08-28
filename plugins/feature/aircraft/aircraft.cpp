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
#include <QThread>

#include "SWGMapItem.h"

#include "maincore.h"

#include "channel/channelapi.h"

#include "aircraft.h"

MESSAGE_CLASS_DEFINITION(Aircraft::MsgConfigureAircraft, Message)

const char* const Aircraft::m_featureIdURI = "sdrangel.feature.aircraft";
const char* const Aircraft::m_featureId = "Aircraft";

Aircraft::Aircraft(WebAPIAdapterInterface *webAPIAdapterInterface) :
    Feature(m_featureIdURI, webAPIAdapterInterface),
    m_availableChannelHandler({"sdrangel.channel.adsbdemod", "sdrangel.channel.acarsdemod"}, QStringList{"aircraftreport"})
{
    qDebug("Aircraft::Aircraft: webAPIAdapterInterface: %p", webAPIAdapterInterface);
    setObjectName(m_featureId);
    m_state = StIdle;
    m_errorMessage = "Aircraft error";
    QObject::connect(
        &m_availableChannelHandler,
        &AvailableChannelOrFeatureHandler::messageEnqueued,
        this,
        &Aircraft::handleChannelMessageQueue);

    // All processing runs on the tracker's thread, for the feature's lifetime,
    // so it works the same with or without a GUI
    m_thread = new QThread();
    m_tracker = new AircraftTracker(this);
    m_tracker->moveToThread(m_thread);
    QObject::connect(m_thread, &QThread::started, m_tracker, &AircraftTracker::startWork);
    QObject::connect(m_thread, &QThread::finished, m_tracker, &QObject::deleteLater);
    QObject::connect(m_thread, &QThread::finished, m_thread, &QThread::deleteLater);
    m_thread->start();
}

Aircraft::~Aircraft()
{
    QObject::disconnect(
        &m_availableChannelHandler,
        &AvailableChannelOrFeatureHandler::messageEnqueued,
        this,
        &Aircraft::handleChannelMessageQueue);
    m_thread->quit();
    m_thread->wait();
    m_tracker = nullptr;
    m_thread = nullptr;
}

void Aircraft::start()
{
    qDebug("Aircraft::start");
    m_state = StRunning;
}

void Aircraft::stop()
{
    qDebug("Aircraft::stop");
    m_state = StIdle;
}

bool Aircraft::handleMessage(const Message& cmd)
{
    if (MsgConfigureAircraft::match(cmd))
    {
        MsgConfigureAircraft& cfg = (MsgConfigureAircraft&) cmd;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (MainCore::MsgAircraftReport::match(cmd))
    {
        // All processing happens on the tracker's thread
        MainCore::MsgAircraftReport& report = (MainCore::MsgAircraftReport&) cmd;

        // The user can turn individual demodulators off
        bool wanted = true;
        if (!m_settings.m_disabledSources.isEmpty())
        {
            const ChannelAPI *channel = qobject_cast<const ChannelAPI *>(report.getPipeSource());
            if (channel && m_settings.m_disabledSources.contains(channel->getURI())) {
                wanted = false;
            }
        }

        if (m_tracker && wanted)
        {
            m_tracker->getInputMessageQueue()->push(AircraftTracker::MsgReport::create(report.getReport()));
            report.getReport().m_mapItem = nullptr;     // Ownership moved with the copy
        }
        else if (report.getReport().m_mapItem)
        {
            delete report.getReport().m_mapItem;
            report.getReport().m_mapItem = nullptr;
        }
        return true;
    }
    else
    {
        return false;
    }
}

QByteArray Aircraft::serialize() const
{
    return m_settings.serialize();
}

bool Aircraft::deserialize(const QByteArray& data)
{
    bool ok = m_settings.deserialize(data);
    if (!ok) {
        m_settings.resetToDefaults();
    }
    MsgConfigureAircraft *msg = MsgConfigureAircraft::create(m_settings, QList<QString>(), true);
    m_inputMessageQueue.push(msg);
    return ok;
}

void Aircraft::applySettings(const AircraftSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "Aircraft::applySettings:" << settings.getDebugString(settingsKeys, force) << force;

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (m_tracker) {
        m_tracker->getInputMessageQueue()->push(AircraftTracker::MsgConfigureTracker::create(settings, settingsKeys, force));
    }
}

void Aircraft::handleChannelMessageQueue(MessageQueue* messageQueue)
{
    Message* message;

    while ((message = messageQueue->pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}
