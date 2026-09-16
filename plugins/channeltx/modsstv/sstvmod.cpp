///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by Copilot / Claude Sonnet                                          //
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
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QBuffer>
#include <QThread>

#include "SWGChannelSettings.h"
#include "SWGWorkspaceInfo.h"

#include "dsp/dspcommands.h"
#include "dsp/scopevis.h"
#include "device/deviceapi.h"
#include "maincore.h"

#include "sstvmodbaseband.h"
#include "sstvmod.h"

MESSAGE_CLASS_DEFINITION(SSTVMod::MsgConfigureSSTVMod, Message)
MESSAGE_CLASS_DEFINITION(SSTVMod::MsgStartStop, Message)
MESSAGE_CLASS_DEFINITION(SSTVMod::MsgReportTransmitComplete, Message)

const char* const SSTVMod::m_channelIdURI = "sdrangel.channeltx.modsstv";
const char* const SSTVMod::m_channelId    = "SSTVMod";

SSTVMod::SSTVMod(DeviceAPI *deviceAPI) :
    ChannelAPI(m_channelIdURI, ChannelAPI::StreamSingleSource),
    m_deviceAPI(deviceAPI),
    m_spectrumVis(SDR_TX_SCALEF)
{
    setObjectName(m_channelId);
    applySettings(QStringList(), m_settings, true);

    m_basebandSource = new SSTVModBaseband();
    m_basebandSource->setChannel(this);
    m_basebandSource->setSpectrumSampleSink(&m_spectrumVis);

    m_deviceAPI->addChannelSource(this);
    m_deviceAPI->addChannelSourceAPI(this);

    m_networkManager = new QNetworkAccessManager();
    QObject::connect(
        m_networkManager,
        &QNetworkAccessManager::finished,
        this,
        &SSTVMod::networkManagerFinished
    );
}

SSTVMod::~SSTVMod()
{
    QObject::disconnect(
        m_networkManager,
        &QNetworkAccessManager::finished,
        this,
        &SSTVMod::networkManagerFinished
    );
    delete m_networkManager;
    m_deviceAPI->removeChannelSourceAPI(this);
    m_deviceAPI->removeChannelSource(this, true);
    stop();

    delete m_basebandSource;
}

void SSTVMod::setDeviceAPI(DeviceAPI *deviceAPI)
{
    if (deviceAPI != m_deviceAPI)
    {
        m_deviceAPI->removeChannelSourceAPI(this);
        m_deviceAPI->removeChannelSource(this, false);
        m_deviceAPI = deviceAPI;
        m_deviceAPI->addChannelSource(this);
        m_deviceAPI->addChannelSinkAPI(this);
    }
}

void SSTVMod::start()
{
    if (m_running) {
        return;
    }
    qDebug("SSTVMod::start");
    m_thread = new QThread(this);
    m_basebandSource->reset();
    m_basebandSource->moveToThread(m_thread);

    QObject::connect(m_thread, &QThread::finished, m_thread, &QThread::deleteLater);
    QObject::connect(m_basebandSource, &SSTVModBaseband::transmitComplete, this, &SSTVMod::onTransmitComplete);

    m_thread->start();

    DSPSignalNotification *dspMsg = new DSPSignalNotification(m_basebandSampleRate, m_centerFrequency);
    m_basebandSource->getInputMessageQueue()->push(dspMsg);

    SSTVModBaseband::MsgConfigureSSTVModBaseband *msg =
        SSTVModBaseband::MsgConfigureSSTVModBaseband::create(QStringList(), m_settings, true);
    m_basebandSource->getInputMessageQueue()->push(msg);

    m_running = true;
}

void SSTVMod::stop()
{
    if (!m_running) {
        return;
    }
    qDebug("SSTVMod::stop");
    m_running = false;
    m_thread->quit();
    m_thread->wait();
}

void SSTVMod::pull(SampleVector::iterator& begin, unsigned int nbSamples)
{
    if (m_running) {
        m_basebandSource->pull(begin, nbSamples);
    }
}

void SSTVMod::setCenterFrequency(qint64 frequency)
{
    SSTVModSettings settings = m_settings;
    settings.m_inputFrequencyOffset = frequency;
    applySettings(QStringList("inputFrequencyOffset"), settings, false);

    if (m_guiMessageQueue)
    {
        auto *msgToGUI = MsgConfigureSSTVMod::create(QStringList("inputFrequencyOffset"), settings, false);
        m_guiMessageQueue->push(msgToGUI);
    }
}

bool SSTVMod::handleMessage(const Message& cmd)
{
    if (MsgConfigureSSTVMod::match(cmd))
    {
        const auto& cfg = static_cast<const MsgConfigureSSTVMod&>(cmd);
        qDebug() << "SSTVMod::handleMessage: MsgConfigureSSTVMod";
        applySettings(cfg.getSettingsKeys(), cfg.getSettings(), cfg.getForce());
        return true;
    }
    else if (MsgStartStop::match(cmd))
    {
        const auto& msg = static_cast<const MsgStartStop&>(cmd);
        if (m_running)
        {
            auto *bbMsg = SSTVModBaseband::MsgStartStop::create(msg.getStart());
            m_basebandSource->getInputMessageQueue()->push(bbMsg);
        }
        return true;
    }
    else if (DSPSignalNotification::match(cmd))
    {
        const auto& notif = static_cast<const DSPSignalNotification&>(cmd);
        m_basebandSampleRate = notif.getSampleRate();
        m_centerFrequency    = notif.getCenterFrequency();
        if (m_running)
        {
            auto *dspMsg = new DSPSignalNotification(notif);
            m_basebandSource->getInputMessageQueue()->push(dspMsg);
        }
        return true;
    }
    else
    {
        return false;
    }
}

void SSTVMod::applySettings(const QStringList& settingsKeys, const SSTVModSettings& settings, bool force)
{
    qDebug() << "SSTVMod::applySettings:" << settings.getDebugString(settingsKeys, force) << " force:" << force;
    QMutexLocker mutexLocker(&m_settingsMutex);

    if (m_running)
    {
        auto *msg = SSTVModBaseband::MsgConfigureSSTVModBaseband::create(settingsKeys, settings, force);
        m_basebandSource->getInputMessageQueue()->push(msg);
    }

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }
}

QByteArray SSTVMod::serialize() const
{
    return m_settings.serialize();
}

bool SSTVMod::deserialize(const QByteArray& data)
{
    if (m_settings.deserialize(data))
    {
        MsgConfigureSSTVMod *msg = MsgConfigureSSTVMod::create(QStringList(), m_settings, true);
        m_inputMessageQueue.push(msg);
        return true;
    }
    else
    {
        m_settings.resetToDefaults();
        MsgConfigureSSTVMod *msg = MsgConfigureSSTVMod::create(QStringList(), m_settings, true);
        m_inputMessageQueue.push(msg);
        return false;
    }
}

int SSTVMod::webapiSettingsGet(SWGSDRangel::SWGChannelSettings& response, QString& errorMessage)
{
    (void) errorMessage;
    response.setChannelType(new QString(m_channelId));
    return 200;
}

int SSTVMod::webapiWorkspaceGet(SWGSDRangel::SWGWorkspaceInfo& response, QString& errorMessage)
{
    (void) errorMessage;
    response.setIndex(m_settings.m_workspaceIndex);
    return 200;
}

int SSTVMod::webapiSettingsPutPatch(bool force, const QStringList& channelSettingsKeys,
    SWGSDRangel::SWGChannelSettings& response, QString& errorMessage)
{
    (void) force;
    (void) channelSettingsKeys;
    (void) response;
    errorMessage = "SSTV modulator settings via WebAPI not yet fully implemented";
    return 501;
}

void SSTVMod::webapiFormatChannelSettings(SWGSDRangel::SWGChannelSettings& response, const SSTVModSettings& settings)
{
    (void) response;
    (void) settings;
}

void SSTVMod::webapiUpdateChannelSettings(SSTVModSettings& settings, const QStringList& channelSettingsKeys,
    SWGSDRangel::SWGChannelSettings& response)
{
    (void) settings;
    (void) channelSettingsKeys;
    (void) response;
}

double SSTVMod::getMagSq() const
{
    if (m_running) {
        return m_basebandSource->getMagSq();
    }
    return 0.0;
}

ScopeVis *SSTVMod::getScopeSink()
{
    return m_basebandSource->getScopeSink();
}

uint32_t SSTVMod::getNumberOfDeviceStreams() const
{
    return m_deviceAPI->getNbSinkStreams();
}

void SSTVMod::onTransmitComplete()
{
    qDebug("SSTVMod::onTransmitComplete");
    if (m_guiMessageQueue) {
        m_guiMessageQueue->push(MsgReportTransmitComplete::create());
    }
}

void SSTVMod::networkManagerFinished(QNetworkReply *reply) const
{
    QNetworkReply::NetworkError replyError = reply->error();
    if (replyError)
    {
        qWarning() << "SSTVMod::networkManagerFinished: error(" << (int) replyError
                   << "): " << replyError << ": " << reply->errorString();
    }
    else
    {
        QString answer = reply->readAll();
        answer.chop(1);
        qDebug("SSTVMod::networkManagerFinished: reply:\n%s", answer.toStdString().c_str());
    }
    reply->deleteLater();
}

void SSTVMod::webapiReverseSendSettings(const QList<QString>& /*channelSettingsKeys*/,
    const SSTVModSettings& /*settings*/, bool /*force*/)
{
    // Reverse API not fully implemented for SSTVMod
}

void SSTVMod::sendChannelSettings(const QList<ObjectPipe*>& /*pipes*/,
    const QList<QString>& /*channelSettingsKeys*/,
    const SSTVModSettings& /*settings*/, bool /*force*/)
{
    // Channel settings pipe broadcasting not fully implemented for SSTVMod
}
