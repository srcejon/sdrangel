///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
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

#include "SWGWorkspaceInfo.h"

#include "device/deviceapi.h"
#include "dsp/dspcommands.h"

#include "meteor.h"

MESSAGE_CLASS_DEFINITION(Meteor::MsgConfigureMeteor, Message)

const char * const Meteor::m_channelIdURI = "sdrangel.channel.meteor";
const char * const Meteor::m_channelId = "Meteor";

Meteor::Meteor(DeviceAPI *deviceAPI) :
    ChannelAPI(m_channelIdURI, ChannelAPI::StreamSingleSink),
    m_deviceAPI(deviceAPI),
    m_spectrumVis(SDR_RX_SCALEF),
    m_basebandSampleRate(0),
    m_centerFrequency(0)
{
    setObjectName(m_channelId);

    m_basebandSink = new MeteorBaseband();
    m_basebandSink->setScopeSink(&m_scopeVis);
    m_basebandSink->setSpectrumSink(&m_spectrumVis);
    m_basebandSink->setChannel(this);
    m_basebandSink->moveToThread(&m_thread);

    applySettings(m_settings, QStringList(), true);

    m_deviceAPI->addChannelSink(this);
    m_deviceAPI->addChannelSinkAPI(this);

    QObject::connect(
        this,
        &ChannelAPI::indexInDeviceSetChanged,
        this,
        &Meteor::handleIndexInDeviceSetChanged
    );
}

Meteor::~Meteor()
{
    qDebug("Meteor::~Meteor");

    m_deviceAPI->removeChannelSinkAPI(this);
    m_deviceAPI->removeChannelSink(this, true);

    if (m_basebandSink->isRunning()) {
        stop();
    }

    delete m_basebandSink;
}

void Meteor::setDeviceAPI(DeviceAPI *deviceAPI)
{
    if (deviceAPI != m_deviceAPI)
    {
        m_deviceAPI->removeChannelSinkAPI(this);
        m_deviceAPI->removeChannelSink(this, false);
        m_deviceAPI = deviceAPI;
        m_deviceAPI->addChannelSink(this);
        m_deviceAPI->addChannelSinkAPI(this);
    }
}

void Meteor::setMessageQueueToGUI(MessageQueue *queue)
{
    ChannelAPI::setMessageQueueToGUI(queue);
    m_basebandSink->setMessageQueueToGUI(queue);
}

uint32_t Meteor::getNumberOfDeviceStreams() const
{
    return m_deviceAPI->getNbSourceStreams();
}

void Meteor::feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end, bool firstOfBurst)
{
    (void) firstOfBurst;
    m_basebandSink->feed(begin, end);
}

void Meteor::start()
{
    qDebug("Meteor::start");

    m_basebandSink->reset();
    m_basebandSink->startWork();
    m_thread.start();

    DSPSignalNotification *dspMsg = new DSPSignalNotification(m_basebandSampleRate, m_centerFrequency);
    m_basebandSink->getInputMessageQueue()->push(dspMsg);

    MeteorBaseband::MsgConfigureMeteorBaseband *msg =
        MeteorBaseband::MsgConfigureMeteorBaseband::create(m_settings, QStringList(), true);
    m_basebandSink->getInputMessageQueue()->push(msg);

    if (getMessageQueueToGUI()) {
        getMessageQueueToGUI()->push(new DSPSignalNotification(m_basebandSampleRate, m_centerFrequency));
    }
}

void Meteor::stop()
{
    qDebug("Meteor::stop");
    m_basebandSink->stopWork();
    m_thread.quit();
    m_thread.wait();
}

bool Meteor::handleMessage(const Message& cmd)
{
    if (MsgConfigureMeteor::match(cmd))
    {
        MsgConfigureMeteor& cfg = (MsgConfigureMeteor&) cmd;
        qDebug() << "Meteor::handleMessage: MsgConfigureMeteor";
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());

        return true;
    }
    else if (DSPSignalNotification::match(cmd))
    {
        DSPSignalNotification& notif = (DSPSignalNotification&) cmd;
        m_basebandSampleRate = notif.getSampleRate();
        m_centerFrequency = notif.getCenterFrequency();
        qDebug() << "Meteor::handleMessage: DSPSignalNotification";

        m_basebandSink->getInputMessageQueue()->push(new DSPSignalNotification(notif));

        if (getMessageQueueToGUI()) {
            getMessageQueueToGUI()->push(new DSPSignalNotification(notif));
        }

        return true;
    }
    else
    {
        return false;
    }
}

void Meteor::setCenterFrequency(qint64 frequency)
{
    MeteorSettings settings = m_settings;
    settings.m_inputFrequencyOffset = frequency;
    settings.m_frequency = m_centerFrequency + settings.m_inputFrequencyOffset;
    applySettings(settings, {"inputFrequencyOffset", "frequency"}, false);

    if (m_guiMessageQueue)
    {
        MsgConfigureMeteor *msgToGUI = MsgConfigureMeteor::create(settings, {"inputFrequencyOffset", "frequency"}, false);
        m_guiMessageQueue->push(msgToGUI);
    }
}

void Meteor::applySettings(const MeteorSettings& settings, const QStringList& settingsKeys, bool force)
{
    qDebug() << "Meteor::applySettings:" << settings.getDebugString(settingsKeys, force)
             << " force:" << force;

    if (settingsKeys.contains("streamIndex"))
    {
        if (m_deviceAPI->getSampleMIMO())
        {
            m_deviceAPI->removeChannelSinkAPI(this);
            m_deviceAPI->removeChannelSink(this, m_settings.m_streamIndex);
            m_deviceAPI->addChannelSink(this, settings.m_streamIndex);
            m_deviceAPI->addChannelSinkAPI(this);
            m_settings.m_streamIndex = settings.m_streamIndex;
            emit streamIndexChanged(settings.m_streamIndex);
        }
    }

    MeteorBaseband::MsgConfigureMeteorBaseband *msg =
        MeteorBaseband::MsgConfigureMeteorBaseband::create(settings, settingsKeys, force);
    m_basebandSink->getInputMessageQueue()->push(msg);

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }
}

QByteArray Meteor::serialize() const
{
    return m_settings.serialize();
}

bool Meteor::deserialize(const QByteArray& data)
{
    if (m_settings.deserialize(data))
    {
        MsgConfigureMeteor *msg = MsgConfigureMeteor::create(m_settings, QStringList(), true);
        m_inputMessageQueue.push(msg);
        return true;
    }
    else
    {
        m_settings.resetToDefaults();
        MsgConfigureMeteor *msg = MsgConfigureMeteor::create(m_settings, QStringList(), true);
        m_inputMessageQueue.push(msg);
        return false;
    }
}

int Meteor::webapiWorkspaceGet(
        SWGSDRangel::SWGWorkspaceInfo& response,
        QString& errorMessage)
{
    (void) errorMessage;
    response.setIndex(m_settings.m_workspaceIndex);
    return 200;
}

void Meteor::handleIndexInDeviceSetChanged(int index)
{
    if (index < 0) {
        return;
    }

    QString fifoLabel = QString("%1 [%2:%3]")
        .arg(m_channelId)
        .arg(m_deviceAPI->getDeviceSetIndex())
        .arg(index);
    m_basebandSink->setFifoLabel(fifoLabel);
}
