///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019-2021 Edouard Griffiths, F4EXB <f4exb06@gmail.com>          //
// Copyright (C) 2020-2023 Jon Beniston, M7RCE <jon@beniston.com>                //
// Copyright (C) 2022 Jiří Pinkava <jiri.pinkava@rossum.ai>                      //
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

#include <cmath>

#include <QDateTime>
#include <QDebug>

#include "dsp/downchannelizer.h"
#include "dsp/dspcommands.h"
#include "maincore.h"

#include "remotetcpsinkbaseband.h"
#include "remotetcpsink.h"

RemoteTCPSinkBaseband::RemoteTCPSinkBaseband() :
    m_running(false),
    m_basebandSampleRate(48000),
    m_nextSampleTimeUsecs(0),
    m_haveSampleTime(false)
{
    qDebug("RemoteTCPSinkBaseband::RemoteTCPSinkBaseband");
    m_sampleFifo.setSize(SampleSinkFifo::getSizePolicy(48000));
    m_channelizer = new DownChannelizer(&m_sink);
    m_sink.setParent(this); // Set parent, so sink is moved to same thread as this baseband object (without this, networking in sink will not work properly!)
    connect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));
}

RemoteTCPSinkBaseband::~RemoteTCPSinkBaseband()
{
    qDebug("RemoteTCPSinkBaseband::~RemoteTCPSinkBaseband");
    delete m_channelizer;
}

void RemoteTCPSinkBaseband::reset()
{
    QMutexLocker mutexLocker(&m_mutex);
    m_inputMessageQueue.clear();
    m_sampleFifo.reset();
    m_nextSampleTimeUsecs = 0;
    m_haveSampleTime = false;
    m_sink.setNextSampleTime(0);
    m_sink.init();
}

void RemoteTCPSinkBaseband::startWork()
{
    QMutexLocker mutexLocker(&m_mutex);
    QObject::connect(
        &m_sampleFifo,
        &SampleSinkFifo::dataReady,
        this,
        &RemoteTCPSinkBaseband::handleData,
        Qt::QueuedConnection
    );
    QObject::connect(
        &m_sampleFifo,
        &SampleSinkFifo::written,
        this,
        &RemoteTCPSinkBaseband::handleSamplesWritten,
        Qt::QueuedConnection
    );
    connect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));
    m_sink.start();
    m_running = true;
}

void RemoteTCPSinkBaseband::stopWork()
{
    QMutexLocker mutexLocker(&m_mutex);
    m_sink.stop();
    disconnect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));
    QObject::disconnect(
        &m_sampleFifo,
        &SampleSinkFifo::dataReady,
        this,
        &RemoteTCPSinkBaseband::handleData
    );
    QObject::disconnect(
        &m_sampleFifo,
        &SampleSinkFifo::written,
        this,
        &RemoteTCPSinkBaseband::handleSamplesWritten
    );
    m_running = false;
}

void RemoteTCPSinkBaseband::feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end)
{
    m_sampleFifo.write(begin, end);
}

void RemoteTCPSinkBaseband::handleData()
{
    QMutexLocker mutexLocker(&m_mutex);

    while ((m_sampleFifo.fill() > 0) && (m_inputMessageQueue.size() == 0))
    {
        SampleVector::iterator part1begin;
        SampleVector::iterator part1end;
        SampleVector::iterator part2begin;
        SampleVector::iterator part2end;

        std::size_t count = m_sampleFifo.readBegin(m_sampleFifo.fill(), &part1begin, &part1end, &part2begin, &part2end);

        // first part of FIFO data
        if (part1begin != part1end) {
            m_channelizer->feed(part1begin, part1end);
        }

        // second part of FIFO data (used when block wraps around)
        if(part2begin != part2end) {
            m_channelizer->feed(part2begin, part2end);
        }

        m_sampleFifo.readCommit((unsigned int) count);
    }
}

void RemoteTCPSinkBaseband::handleSamplesWritten(
    int samples,
    qint64 elapsedNsecs)
{
    if ((samples <= 0) || (m_basebandSampleRate <= 0)) {
        return;
    }

    const qint64 sampleDurationUsecs = (qint64) std::llround(
        (double) samples * 1000000.0
        / (double) m_basebandSampleRate);

    if (!m_haveSampleTime)
    {
        const qint64 nowElapsedNsecs =
            MainCore::instance()->getElapsedNsecs();
        const qint64 nowEpochUsecs =
            QDateTime::currentMSecsSinceEpoch() * 1000;
        m_nextSampleTimeUsecs = nowEpochUsecs
            - ((nowElapsedNsecs - elapsedNsecs) / 1000)
            - sampleDurationUsecs;
        m_haveSampleTime = true;
    }

    m_nextSampleTimeUsecs += sampleDurationUsecs;
    m_sink.setNextSampleTime(m_nextSampleTimeUsecs);
}

void RemoteTCPSinkBaseband::handleInputMessages()
{
    Message* message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

bool RemoteTCPSinkBaseband::handleMessage(const Message& cmd)
{
    if (RemoteTCPSink::MsgConfigureRemoteTCPSink::match(cmd))
    {
        QMutexLocker mutexLocker(&m_mutex);
        const RemoteTCPSink::MsgConfigureRemoteTCPSink& cfg = (const RemoteTCPSink::MsgConfigureRemoteTCPSink&) cmd;
        qDebug() << "RemoteTCPSinkBaseband::handleMessage: MsgConfigureRemoteTCPSink";

        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce(), cfg.getRestartRequired());

        return true;
    }
    else if (DSPSignalNotification::match(cmd))
    {
        const DSPSignalNotification& notif = (const DSPSignalNotification&) cmd;
        qDebug() << "RemoteTCPSinkBaseband::handleMessage: DSPSignalNotification: basebandSampleRate:" << notif.getSampleRate();
        setBasebandSampleRate(notif.getSampleRate());
        m_sampleFifo.setSize(SampleSinkFifo::getSizePolicy(notif.getSampleRate()));

        return true;
    }
    else if (RemoteTCPSink::MsgSendMessage::match(cmd))
    {
        const RemoteTCPSink::MsgSendMessage& msg = (const RemoteTCPSink::MsgSendMessage&) cmd;

        m_sink.sendMessage(msg.getAddress(), msg.getPort(), msg.getCallsign(), msg.getText(), msg.getBroadcast());

        return true;
    }
    else
    {
        return false;
    }
}

void RemoteTCPSinkBaseband::applySettings(const RemoteTCPSinkSettings& settings, const QStringList& settingsKeys, bool force, bool restartRequired)
{
    qDebug() << "RemoteTCPSinkBaseband::applySettings:"
        << "m_channelSampleRate:" << settings.m_channelSampleRate
        << "m_inputFrequencyOffset:" << settings.m_inputFrequencyOffset
        << " force: " << force;

    if (settingsKeys.contains("channelSampleRate") || settingsKeys.contains("inputFrequencyOffset") || force)
    {
        m_channelizer->setChannelization(settings.m_channelSampleRate, settings.m_inputFrequencyOffset);
        m_sink.applyChannelSettings(m_channelizer->getChannelSampleRate(), m_channelizer->getChannelFrequencyOffset());
    }

    m_sink.applySettings(settings, settingsKeys, force, restartRequired);
    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }
}

int RemoteTCPSinkBaseband::getChannelSampleRate() const
{
    return m_channelizer->getChannelSampleRate();
}

void RemoteTCPSinkBaseband::setBasebandSampleRate(int sampleRate)
{
    if (sampleRate != m_basebandSampleRate)
    {
        m_basebandSampleRate = sampleRate;
        m_nextSampleTimeUsecs = 0;
        m_haveSampleTime = false;
        m_sink.setNextSampleTime(0);
    }

    m_channelizer->setBasebandSampleRate(sampleRate);
    m_sink.applyChannelSettings(m_channelizer->getChannelSampleRate(), m_channelizer->getChannelFrequencyOffset());
}
