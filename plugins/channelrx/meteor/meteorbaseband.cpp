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

#include <algorithm>

#include <QDebug>
#include <QThread>

#include "dsp/downchannelizer.h"
#include "dsp/dspcommands.h"
#include "dsp/scopevis.h"
#include "dsp/spectrumvis.h"

#include "meteorbaseband.h"

MESSAGE_CLASS_DEFINITION(MeteorBaseband::MsgConfigureMeteorBaseband, Message)

MeteorBaseband::MeteorBaseband() :
    m_spectrumVis(nullptr),
    m_running(false),
    m_inactivityTimer(new QTimer(this))
{
    qDebug("MeteorBaseband::MeteorBaseband");

    m_sampleFifo.setSize(SampleSinkFifo::getSizePolicy(192000));
    m_channelizer = new DownChannelizer(&m_sink);
    m_inactivityTimer->setInterval(250);
    m_inactivityTimer->setSingleShot(false);
    connect(m_inactivityTimer, &QTimer::timeout, this, &MeteorBaseband::handleInactivity);
}

MeteorBaseband::~MeteorBaseband()
{
    m_inputMessageQueue.clear();
    delete m_channelizer;
}

void MeteorBaseband::reset()
{
    QMutexLocker mutexLocker(&m_mutex);
    m_inputMessageQueue.clear();
    m_sampleFifo.reset();
}

void MeteorBaseband::startWork()
{
    QMutexLocker mutexLocker(&m_mutex);
    QObject::connect(
        &m_sampleFifo,
        &SampleSinkFifo::dataReady,
        this,
        &MeteorBaseband::handleData,
        Qt::QueuedConnection
    );
    connect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));
    m_running = true;
    m_lastDataTimer.start();

    if (QThread::currentThread() == thread()) {
        startInactivityTimer();
    } else {
        QMetaObject::invokeMethod(this, "startInactivityTimer", Qt::QueuedConnection);
    }
}

void MeteorBaseband::stopWork()
{
    if (QThread::currentThread() == thread()) {
        stopInactivityTimer();
    } else if (thread()->isRunning()) {
        QMetaObject::invokeMethod(this, "stopInactivityTimer", Qt::BlockingQueuedConnection);
    }

    QMutexLocker mutexLocker(&m_mutex);
    disconnect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));
    QObject::disconnect(
        &m_sampleFifo,
        &SampleSinkFifo::dataReady,
        this,
        &MeteorBaseband::handleData
    );
    m_running = false;
}

void MeteorBaseband::setChannel(ChannelAPI *channel)
{
    m_sink.setChannel(channel);
}

void MeteorBaseband::setScopeSink(ScopeVis* scopeSink)
{
    if (scopeSink) {
        scopeSink->setNbStreams(MeteorDemodSink::m_scopeStreams);
    }

    m_sink.setScopeSink(scopeSink);
}

void MeteorBaseband::feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end)
{
    m_sampleFifo.write(begin, end);
}

void MeteorBaseband::handleData()
{
    QMutexLocker mutexLocker(&m_mutex);

    while ((m_sampleFifo.fill() > 0) && (m_inputMessageQueue.size() == 0))
    {
        SampleVector::iterator part1begin;
        SampleVector::iterator part1end;
        SampleVector::iterator part2begin;
        SampleVector::iterator part2end;

        std::size_t count = m_sampleFifo.readBegin(m_sampleFifo.fill(), &part1begin, &part1end, &part2begin, &part2end);

        if (part1begin != part1end) {
            m_channelizer->feed(part1begin, part1end);
        }

        if (part2begin != part2end) {
            m_channelizer->feed(part2begin, part2end);
        }

        m_sampleFifo.readCommit((unsigned int) count);

        if (count > 0) {
            m_lastDataTimer.restart();
        }
    }
}

void MeteorBaseband::startInactivityTimer()
{
    m_inactivityTimer->start();
}

void MeteorBaseband::stopInactivityTimer()
{
    m_inactivityTimer->stop();
}

void MeteorBaseband::handleInactivity()
{
    QMutexLocker mutexLocker(&m_mutex);

    if (!m_running || !m_lastDataTimer.isValid() || (m_lastDataTimer.elapsed() < 250)) {
        return;
    }

    if (m_sink.flushPendingPulse()) {
        m_lastDataTimer.restart();
    }
}

void MeteorBaseband::handleInputMessages()
{
    Message* message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

bool MeteorBaseband::handleMessage(const Message& cmd)
{
    if (MsgConfigureMeteorBaseband::match(cmd))
    {
        QMutexLocker mutexLocker(&m_mutex);
        MsgConfigureMeteorBaseband& cfg = (MsgConfigureMeteorBaseband&) cmd;
        qDebug() << "MeteorBaseband::handleMessage: MsgConfigureMeteorBaseband";

        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());

        return true;
    }
    else if (DSPSignalNotification::match(cmd))
    {
        QMutexLocker mutexLocker(&m_mutex);
        DSPSignalNotification& notif = (DSPSignalNotification&) cmd;
        qDebug() << "MeteorBaseband::handleMessage: DSPSignalNotification: basebandSampleRate:" << notif.getSampleRate();

        setBasebandSampleRate(notif.getSampleRate());
        m_sampleFifo.setSize(SampleSinkFifo::getSizePolicy(std::max(notif.getSampleRate(), 192000)));

        return true;
    }
    else
    {
        return false;
    }
}

void MeteorBaseband::applySettings(const MeteorSettings& settings, const QStringList& settingsKeys, bool force)
{
    const bool channelizationChange =
        (settingsKeys.contains("inputFrequencyOffset") && (settings.m_inputFrequencyOffset != m_settings.m_inputFrequencyOffset))
        || (settingsKeys.contains("channelSampleRate") && (settings.m_channelSampleRate != m_settings.m_channelSampleRate))
        || force;

    m_sink.applySettings(settings, settingsKeys, force);

    if (channelizationChange)
    {
        m_channelizer->setChannelization(settings.m_channelSampleRate, settings.m_inputFrequencyOffset);
        m_sink.applyChannelSettings(m_channelizer->getChannelSampleRate(), m_channelizer->getChannelFrequencyOffset(), true);
        notifyVisualSampleRate();
    }

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }
}

void MeteorBaseband::setBasebandSampleRate(int sampleRate)
{
    m_channelizer->setBasebandSampleRate(sampleRate);
    m_channelizer->setChannelization(m_settings.m_channelSampleRate, m_settings.m_inputFrequencyOffset);
    m_sink.applyChannelSettings(m_channelizer->getChannelSampleRate(), m_channelizer->getChannelFrequencyOffset(), true);
    notifyVisualSampleRate();
}

int MeteorBaseband::getChannelSampleRate() const
{
    return m_sink.getOutputSampleRate();
}

void MeteorBaseband::notifyVisualSampleRate()
{
    if (m_spectrumVis)
    {
        DSPSignalNotification *msg = new DSPSignalNotification(m_settings.m_channelSampleRate, 0);
        m_spectrumVis->getInputMessageQueue()->push(msg);
    }
}
