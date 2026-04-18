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

#include "dsp/upchannelizer.h"
#include "dsp/dspengine.h"
#include "dsp/dspcommands.h"

#include "sstvmodbaseband.h"

MESSAGE_CLASS_DEFINITION(SSTVModBaseband::MsgConfigureSSTVModBaseband, Message)
MESSAGE_CLASS_DEFINITION(SSTVModBaseband::MsgStartStop, Message)

SSTVModBaseband::SSTVModBaseband()
{
    m_sampleFifo.resize(SampleSourceFifo::getSizePolicy(48000));
    m_channelizer = new UpChannelizer(&m_source);
    m_source.setScopeSink(&m_scopeSink);

    QObject::connect(
        &m_sampleFifo,
        &SampleSourceFifo::dataRead,
        this,
        &SSTVModBaseband::handleData,
        Qt::QueuedConnection
    );
    QObject::connect(
        &m_source,
        &SSTVModSource::transmitComplete,
        this,
        &SSTVModBaseband::onTransmitComplete,
        Qt::QueuedConnection
    );

    connect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));
}

SSTVModBaseband::~SSTVModBaseband()
{
    delete m_channelizer;
}

void SSTVModBaseband::reset()
{
    QMutexLocker mutexLocker(&m_mutex);
    m_sampleFifo.reset();
}

void SSTVModBaseband::setChannel(ChannelAPI *channel)
{
    m_source.setChannel(channel);
}

void SSTVModBaseband::pull(const SampleVector::iterator& begin, unsigned int nbSamples)
{
    unsigned int part1Begin, part1End, part2Begin, part2End;
    m_sampleFifo.read(nbSamples, part1Begin, part1End, part2Begin, part2End);
    SampleVector& data = m_sampleFifo.getData();

    if (part1Begin != part1End) {
        std::copy(data.begin() + part1Begin, data.begin() + part1End, begin);
    }
    const unsigned int shift = part1End - part1Begin;
    if (part2Begin != part2End) {
        std::copy(data.begin() + part2Begin, data.begin() + part2End, begin + shift);
    }
}

void SSTVModBaseband::handleData()
{
    QMutexLocker mutexLocker(&m_mutex);
    SampleVector& data = m_sampleFifo.getData();
    unsigned int ipart1begin, ipart1end, ipart2begin, ipart2end;
    unsigned int remainder = m_sampleFifo.remainder();

    while ((remainder > 0) && (m_inputMessageQueue.size() == 0))
    {
        m_sampleFifo.write(remainder, ipart1begin, ipart1end, ipart2begin, ipart2end);
        if (ipart1begin != ipart1end) {
            processFifo(data, ipart1begin, ipart1end);
        }
        if (ipart2begin != ipart2end) {
            processFifo(data, ipart2begin, ipart2end);
        }
        remainder = m_sampleFifo.remainder();
    }
}

void SSTVModBaseband::processFifo(SampleVector& data, unsigned int iBegin, unsigned int iEnd)
{
    m_channelizer->prefetch(iEnd - iBegin);
    m_channelizer->pull(data.begin() + iBegin, iEnd - iBegin);
}

void SSTVModBaseband::handleInputMessages()
{
    Message* message;
    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

bool SSTVModBaseband::handleMessage(const Message& cmd)
{
    if (MsgConfigureSSTVModBaseband::match(cmd))
    {
        QMutexLocker mutexLocker(&m_mutex);
        const auto& cfg = static_cast<const MsgConfigureSSTVModBaseband&>(cmd);
        qDebug() << "SSTVModBaseband::handleMessage: MsgConfigureSSTVModBaseband";
        applySettings(cfg.getSettingsKeys(), cfg.getSettings(), cfg.getForce());
        return true;
    }
    else if (MsgStartStop::match(cmd))
    {
        QMutexLocker mutexLocker(&m_mutex);
        const auto& msg = static_cast<const MsgStartStop&>(cmd);
        if (msg.getStart()) {
            m_source.startTransmit();
        } else {
            m_source.stopTransmit();
        }
        return true;
    }
    else if (DSPSignalNotification::match(cmd))
    {
        QMutexLocker mutexLocker(&m_mutex);
        const auto& notif = static_cast<const DSPSignalNotification&>(cmd);
        qDebug() << "SSTVModBaseband::handleMessage: DSPSignalNotification: basebandSampleRate:" << notif.getSampleRate();
        m_sampleFifo.resize(SampleSourceFifo::getSizePolicy(notif.getSampleRate()));
        m_channelizer->setBasebandSampleRate(notif.getSampleRate());
        m_source.applyChannelSettings(m_channelizer->getChannelSampleRate(), m_channelizer->getChannelFrequencyOffset());
        return true;
    }
    else
    {
        return false;
    }
}

void SSTVModBaseband::applySettings(const QStringList& settingsKeys, const SSTVModSettings& settings, bool force)
{
    if ((settingsKeys.contains("inputFrequencyOffset") || force) &&
        settings.m_inputFrequencyOffset != m_settings.m_inputFrequencyOffset)
    {
        m_channelizer->setChannelization(48000, settings.m_inputFrequencyOffset);
        m_source.applyChannelSettings(m_channelizer->getChannelSampleRate(), m_channelizer->getChannelFrequencyOffset());
    }

    m_source.applySettings(settingsKeys, settings, force);

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }
}

int SSTVModBaseband::getChannelSampleRate() const
{
    return m_channelizer->getChannelSampleRate();
}

void SSTVModBaseband::onTransmitComplete()
{
    emit transmitComplete();
}
