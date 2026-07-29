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
#include <cmath>

#include <QDebug>
#include <QThread>

#include "dsp/downchannelizer.h"
#include "dsp/dspcommands.h"
#include "dsp/scopevis.h"
#include "dsp/spectrumvis.h"
#include "maincore.h"

#include "meteorbaseband.h"

MESSAGE_CLASS_DEFINITION(MeteorBaseband::MsgConfigureMeteorBaseband, Message)

namespace {

constexpr int InactivityCheckIntervalMS = 250;
constexpr int InactivityFlushDelayMS = 2000;

}

MeteorBaseband::MeteorBaseband() :
    m_spectrumVis(nullptr),
    m_secondarySpectrumVis(nullptr),
    m_running(false),
    m_inactivityFlushEnabled(true),
    m_inactivityTimer(new QTimer(this))
{
    qDebug("MeteorBaseband::MeteorBaseband");

    m_sampleFifo.setSize(SampleSinkFifo::getSizePolicy(192000));
    m_channelizer = new DownChannelizer(&m_sink);
    m_inactivityTimer->setInterval(InactivityCheckIntervalMS);
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
    resetLocalChannelTiming();
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
    QObject::connect(
        &m_sampleFifo,
        &SampleSinkFifo::written,
        this,
        &MeteorBaseband::handleSamplesWritten,
        Qt::DirectConnection
    );
    connect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));
    m_running = true;
    m_lastDataTimer.start();

    if (m_inactivityFlushEnabled)
    {
        if (QThread::currentThread() == thread()) {
            startInactivityTimer();
        } else {
            QMetaObject::invokeMethod(this, "startInactivityTimer", Qt::QueuedConnection);
        }
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
    QObject::disconnect(
        &m_sampleFifo,
        &SampleSinkFifo::written,
        this,
        &MeteorBaseband::handleSamplesWritten
    );
    // Deliberate stop = end of stream: emit whatever is still buffered, including
    // retiring active sweeps (a meteor that ended just before stop is otherwise lost).
    m_sink.flushPendingPulse(true);
    m_running = false;
}

void MeteorBaseband::setChannel(ChannelAPI *channel)
{
    m_sink.setChannel(channel);
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
        const qint64 inputEndElapsedNsecs = consumeInputTiming((int) count);

        if (part1begin != part1end) {
            m_channelizer->feed(part1begin, part1end);
        }

        if (part2begin != part2end) {
            m_channelizer->feed(part2begin, part2end);
        }

        m_sampleFifo.readCommit((unsigned int) count);
        updateLocalChannelTiming(inputEndElapsedNsecs);

        if (count > 0) {
            m_lastDataTimer.restart();
        }
    }
}

void MeteorBaseband::handleSamplesWritten(int samples, qint64 elapsedNsecs)
{
    const int sampleRate =
        m_timingBasebandSampleRate.load(std::memory_order_relaxed);

    if ((samples <= 0) || (sampleRate <= 0)) {
        return;
    }

    QMutexLocker timingLocker(&m_timingMutex);
    m_inputTimingBlocks.push_back({
        samples,
        elapsedNsecs,
        sampleRate
    });
}

qint64 MeteorBaseband::consumeInputTiming(int samples)
{
    if (samples <= 0) {
        return 0;
    }

    QMutexLocker timingLocker(&m_timingMutex);
    int samplesRemaining = samples;
    qint64 consumedEndElapsedNsecs = 0;

    while ((samplesRemaining > 0) && !m_inputTimingBlocks.empty())
    {
        InputTimingBlock& block = m_inputTimingBlocks.front();
        const int consumed = std::min(samplesRemaining, block.m_samplesRemaining);
        const int samplesAfter = block.m_samplesRemaining - consumed;

        if (block.m_sampleRate > 0)
        {
            consumedEndElapsedNsecs = block.m_endElapsedNsecs
                - (qint64) std::llround(
                    (double) samplesAfter * 1000000000.0
                    / (double) block.m_sampleRate);
        }

        block.m_samplesRemaining = samplesAfter;
        samplesRemaining -= consumed;

        if (block.m_samplesRemaining == 0) {
            m_inputTimingBlocks.pop_front();
        }
    }

    if (samplesRemaining > 0)
    {
        // A reset, rate change, or overflow can break the correspondence. Drop
        // this measurement rather than applying a timestamp to the wrong block.
        m_inputTimingBlocks.clear();
        return 0;
    }

    return consumedEndElapsedNsecs;
}

void MeteorBaseband::updateLocalChannelTiming(qint64 inputEndElapsedNsecs)
{
    if (inputEndElapsedNsecs <= 0) {
        return;
    }

    const qint64 processingEndElapsedNsecs =
        MainCore::instance()->getElapsedNsecs();
    const double latencyUsecs = std::max(
        0.0,
        (double) (processingEndElapsedNsecs - inputEndElapsedNsecs) / 1000.0);

    // Smooth scheduler jitter while retaining enough responsiveness to expose
    // a growing channel backlog. Twice the mean absolute deviation is reported
    // as the local measurement uncertainty.
    constexpr double alpha = 0.1;

    if (!m_haveLocalLatency)
    {
        m_localLatencyMeanUsecs = latencyUsecs;
        m_localLatencyDeviationUsecs = 0.0;
        m_haveLocalLatency = true;
    }
    else
    {
        const double delta = latencyUsecs - m_localLatencyMeanUsecs;
        m_localLatencyMeanUsecs += alpha * delta;
        m_localLatencyDeviationUsecs += alpha
            * (std::abs(delta) - m_localLatencyDeviationUsecs);
    }

    const int basebandRate =
        m_timingBasebandSampleRate.load(std::memory_order_relaxed);
    const double inputSampleUsecs = basebandRate > 0
        ? 1000000.0 / (double) basebandRate
        : 0.0;
    const qint64 uncertaintyUsecs = (qint64) std::llround(
        std::max(inputSampleUsecs, 2.0 * m_localLatencyDeviationUsecs));
    const qint64 filterDelayUsecs = (qint64) std::llround(
        localFilterDelaySeconds() * 1000000.0);

    m_localProcessingLatencyUsecs.store(
        (qint64) std::llround(m_localLatencyMeanUsecs),
        std::memory_order_relaxed);
    m_localTimingUncertaintyUsecs.store(
        std::max<qint64>(0, uncertaintyUsecs),
        std::memory_order_relaxed);
    m_localFilterDelayUsecs.store(
        std::max<qint64>(0, filterDelayUsecs),
        std::memory_order_relaxed);
}

double MeteorBaseband::localFilterDelaySeconds() const
{
    const int basebandRate =
        m_timingBasebandSampleRate.load(std::memory_order_relaxed);
    const int channelizerRate = m_channelizer
        ? m_channelizer->getChannelSampleRate()
        : 0;

    if ((basebandRate <= 0) || (channelizerRate <= 0)) {
        return 0.0;
    }

    double delaySeconds = 0.0;
    int stageRate = basebandRate;

    while (stageRate > channelizerRate)
    {
        delaySeconds +=
            ((double) DOWNCHANNELIZER_HB_FILTER_ORDER / 2.0)
            / (double) stageRate;
        stageRate /= 2;

        if (stageRate <= 0) {
            break;
        }
    }

    // MeteorDemodSink's 16-phase, two-taps-per-phase resampler has 32 taps
    // per phase and therefore a 15.5-input-sample linear-phase delay.
    delaySeconds += 15.5 / (double) channelizerRate;
    return delaySeconds;
}

void MeteorBaseband::resetLocalChannelTiming()
{
    {
        QMutexLocker timingLocker(&m_timingMutex);
        m_inputTimingBlocks.clear();
    }

    m_localLatencyMeanUsecs = 0.0;
    m_localLatencyDeviationUsecs = 0.0;
    m_haveLocalLatency = false;
    m_localProcessingLatencyUsecs.store(-1, std::memory_order_relaxed);
    m_localTimingUncertaintyUsecs.store(0, std::memory_order_relaxed);
    m_localFilterDelayUsecs.store(0, std::memory_order_relaxed);
}

bool MeteorBaseband::getLocalChannelTiming(
    double& latencySeconds,
    double& uncertaintySeconds,
    double& filterDelaySeconds) const
{
    const qint64 processingLatencyUsecs =
        m_localProcessingLatencyUsecs.load(std::memory_order_relaxed);
    const qint64 uncertaintyUsecs =
        m_localTimingUncertaintyUsecs.load(std::memory_order_relaxed);
    const qint64 filterDelayUsecs =
        m_localFilterDelayUsecs.load(std::memory_order_relaxed);

    if ((processingLatencyUsecs < 0)
        || (processingLatencyUsecs > 60LL * 1000000LL)
        || (uncertaintyUsecs < 0)
        || (uncertaintyUsecs > 60LL * 1000000LL)
        || (filterDelayUsecs < 0)
        || (filterDelayUsecs > 60LL * 1000000LL))
    {
        latencySeconds = 0.0;
        uncertaintySeconds = 0.0;
        filterDelaySeconds = 0.0;
        return false;
    }

    latencySeconds =
        (double) (processingLatencyUsecs + filterDelayUsecs) / 1000000.0;
    uncertaintySeconds = (double) uncertaintyUsecs / 1000000.0;
    filterDelaySeconds = (double) filterDelayUsecs / 1000000.0;
    return true;
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

    if (!m_inactivityFlushEnabled
        || !m_running
        || !m_lastDataTimer.isValid()
        || (m_lastDataTimer.elapsed() < InactivityFlushDelayMS))
    {
        return;
    }

    if (m_sink.flushPendingPulse(false))
    {
        qDebug() << "MeteorBaseband::handleInactivity: flushed pending detector state after"
                 << m_lastDataTimer.elapsed() << "ms without samples";
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
        resetLocalChannelTiming();
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
    resetLocalChannelTiming();
    m_timingBasebandSampleRate.store(sampleRate, std::memory_order_relaxed);
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

    if (m_secondarySpectrumVis)
    {
        DSPSignalNotification *msg = new DSPSignalNotification(m_settings.m_channelSampleRate, 0);
        m_secondarySpectrumVis->getInputMessageQueue()->push(msg);
    }
}
