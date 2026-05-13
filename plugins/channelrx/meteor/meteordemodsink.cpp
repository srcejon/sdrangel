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

#include "dsp/scopevis.h"
#include "dsp/spectrumvis.h"
#include "util/messagequeue.h"

#include "meteordemodsink.h"

MESSAGE_CLASS_DEFINITION(MeteorDemodSink::MsgMeteorDetected, Message)

MeteorDemodSink::MeteorDemodSink() :
    m_scopeSink(nullptr),
    m_spectrumSink(nullptr),
    m_messageQueueToGUI(nullptr),
    m_channelSampleRate(48000),
    m_channelFrequencyOffset(0),
    m_interpolatorDistance(48.0f),
    m_interpolatorDistanceRemain(0.0f),
    m_scopeSampleBufferSize(0),
    m_scopeSampleBufferIndex(0)
{
    resizeScopeBuffers();
    resetDetector();
    applySettings(m_settings, QStringList(), true);
    applyChannelSettings(m_channelSampleRate, m_channelFrequencyOffset, true);
}

MeteorDemodSink::~MeteorDemodSink()
{
}

void MeteorDemodSink::feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end)
{
    if ((m_channelSampleRate <= 0) || (m_settings.m_channelSampleRate <= 0)) {
        return;
    }

    Complex ci;

    for (SampleVector::const_iterator it = begin; it != end; ++it)
    {
        Complex c(it->real(), it->imag());
        c *= m_nco.nextIQ();

        if (m_interpolatorDistance < 1.0f)
        {
            while (!m_interpolator.interpolate(&m_interpolatorDistanceRemain, c, &ci))
            {
                processOneSample(ci);
                m_interpolatorDistanceRemain += m_interpolatorDistance;
            }
        }
        else
        {
            if (m_interpolator.decimate(&m_interpolatorDistanceRemain, c, &ci))
            {
                processOneSample(ci);
                m_interpolatorDistanceRemain += m_interpolatorDistance;
            }
        }
    }
}

void MeteorDemodSink::processOneSample(Complex& ci)
{
    const Real re = ci.real() / SDR_RX_SCALEF;
    const Real im = ci.imag() / SDR_RX_SCALEF;
    const Complex normalized(re, im);
    const double power = std::max<double>(re*re + im*im, 1e-20);
    const double filteredPower = std::max<double>(m_powerLowpass.filter((Real) power), 0.0);

    bool frequencyValid = false;
    double frequency = 0.0;

    if (m_havePrevSample)
    {
        const Complex delta = normalized * std::conj(m_prevSample);
        const double phase = std::atan2(delta.imag(), delta.real());
        frequency = phase * (double) m_settings.m_channelSampleRate / (2.0 * std::acos(-1.0));
        frequencyValid = std::isfinite(frequency);
    }

    m_prevSample = normalized;
    m_havePrevSample = true;

    const bool detected = processDetectorSample(power, filteredPower, frequency, frequencyValid);
    sampleToScope(normalized, power, filteredPower, detected);
    feedSpectrum(normalized);
    m_sampleCounter++;
}

bool MeteorDemodSink::processDetectorSample(double power, double filteredPower, double frequency, bool frequencyValid)
{
    const double threshold = getDetectionThresholdPower();
    const double releaseThreshold = threshold * 0.7;

    if (m_rearmNeeded)
    {
        if (filteredPower < releaseThreshold) {
            m_rearmNeeded = false;
        }

        updateNoiseFloor(filteredPower);
        return false;
    }

    if (!m_pulseActive)
    {
        updateNoiseFloor(filteredPower);

        if (m_noiseFloorInitialized && (filteredPower > threshold))
        {
            startPulse(power, frequency, frequencyValid);
            return true;
        }

        return false;
    }

    updatePulse(power, frequency, frequencyValid);

    if (filteredPower > releaseThreshold) {
        m_pulseLastAboveSample = m_sampleCounter;
    }

    if ((m_settings.m_maxFrequencyDrift > 0.0f) && (m_pulseFreqCount > 1))
    {
        const double frequencySpan = m_pulseFreqMax - m_pulseFreqMin;
        const double frequencyDrift = m_pulseFreqLast - m_pulseFreqFirst;

        if ((std::fabs(frequencySpan) > m_settings.m_maxFrequencyDrift)
            || (std::fabs(frequencyDrift) > m_settings.m_maxFrequencyDrift))
        {
            finishPulse(true);
            m_rearmNeeded = true;
            return false;
        }
    }

    const int holdSamples = std::max(2, m_settings.m_channelSampleRate / 20);
    const int maxSamples = std::max(1, (m_settings.m_maxDurationMS * m_settings.m_channelSampleRate) / 1000);

    if ((int) (m_sampleCounter - m_pulseStartSample + 1) > maxSamples)
    {
        finishPulse(true);
        m_rearmNeeded = true;
        return false;
    }

    if ((int) (m_sampleCounter - m_pulseLastAboveSample) > holdSamples)
    {
        finishPulse(false);
        return false;
    }

    return true;
}

void MeteorDemodSink::startPulse(double power, double frequency, bool frequencyValid)
{
    m_pulseActive = true;
    m_pulseStartSample = m_sampleCounter;
    m_pulseLastAboveSample = m_sampleCounter;
    m_pulseStartDateTimeUtc = QDateTime::currentDateTimeUtc();
    m_pulsePeakPower = power;
    m_pulseFreqMin = frequency;
    m_pulseFreqMax = frequency;
    m_pulseFreqFirst = frequency;
    m_pulseFreqLast = frequency;
    m_pulseFreqCount = frequencyValid ? 1 : 0;
}

void MeteorDemodSink::updatePulse(double power, double frequency, bool frequencyValid)
{
    if (power > m_pulsePeakPower) {
        m_pulsePeakPower = power;
    }

    if (frequencyValid)
    {
        if (m_pulseFreqCount == 0)
        {
            m_pulseFreqMin = frequency;
            m_pulseFreqMax = frequency;
            m_pulseFreqFirst = frequency;
        }

        m_pulseFreqMin = std::min(m_pulseFreqMin, frequency);
        m_pulseFreqMax = std::max(m_pulseFreqMax, frequency);
        m_pulseFreqLast = frequency;
        m_pulseFreqCount++;
    }
}

void MeteorDemodSink::finishPulse(bool forceRejected)
{
    const quint64 endSample = std::max(m_pulseLastAboveSample, m_pulseStartSample);
    const double durationS = (double) (endSample - m_pulseStartSample + 1) / (double) m_settings.m_channelSampleRate;
    const double durationMS = 1000.0 * durationS;
    const double frequencySpan = m_pulseFreqCount > 1 ? m_pulseFreqMax - m_pulseFreqMin : 0.0;
    const double frequencyDrift = m_pulseFreqCount > 1 ? m_pulseFreqLast - m_pulseFreqFirst : 0.0;
    const bool durationOK = (durationMS >= m_settings.m_minDurationMS) && (durationMS <= m_settings.m_maxDurationMS);
    const bool driftOK = (m_settings.m_maxFrequencyDrift <= 0.0f)
        || ((std::fabs(frequencySpan) <= m_settings.m_maxFrequencyDrift) && (std::fabs(frequencyDrift) <= m_settings.m_maxFrequencyDrift));

    if (!forceRejected && durationOK && driftOK && m_messageQueueToGUI)
    {
        const double peakAmplitude = std::sqrt(std::max(m_pulsePeakPower, 0.0));
        const double peakPowerDB = 10.0 * std::log10(std::max(m_pulsePeakPower, 1e-20));

        m_messageQueueToGUI->push(MsgMeteorDetected::create(
            m_pulseStartDateTimeUtc,
            peakAmplitude,
            peakPowerDB,
            durationS,
            frequencySpan,
            frequencyDrift,
            m_settings.m_channelSampleRate
        ));
    }

    m_pulseActive = false;
}

void MeteorDemodSink::updateNoiseFloor(double filteredPower)
{
    const double power = std::max(filteredPower, 1e-20);

    if (!m_noiseFloorInitialized)
    {
        m_noiseFloor = power;
        m_noiseFloorInitialized = true;
        return;
    }

    const double timeConstantS = 10.0;
    const double alpha = std::min(1.0, 1.0 / ((double) m_settings.m_channelSampleRate * timeConstantS));
    m_noiseFloor = (1.0 - alpha) * m_noiseFloor + alpha * power;
}

double MeteorDemodSink::getDetectionThresholdPower() const
{
    const double thresholdLinear = std::pow(10.0, m_settings.m_detectionThresholdDB / 10.0);
    return std::max(m_noiseFloor, 1e-20) * thresholdLinear;
}

void MeteorDemodSink::configureInterpolator()
{
    if ((m_channelSampleRate <= 0) || (m_settings.m_channelSampleRate <= 0)) {
        return;
    }

    const Real cutoff = std::min<Real>(
        (Real) m_settings.m_channelSampleRate * 0.45f,
        (Real) m_channelSampleRate * 0.45f
    );
    m_interpolator.create(16, m_channelSampleRate, cutoff, 2.0f);
    m_interpolatorDistanceRemain = 0.0f;
    m_interpolatorDistance = (Real) m_channelSampleRate / (Real) m_settings.m_channelSampleRate;
}

void MeteorDemodSink::configurePowerLowpass()
{
    const double cutoff = std::min<double>(
        std::max<double>(m_settings.m_powerLPFCutoff, 0.1),
        std::max<double>(0.1, m_settings.m_channelSampleRate * 0.45)
    );

    m_powerLowpass.create(51, m_settings.m_channelSampleRate, cutoff);
}

void MeteorDemodSink::resizeScopeBuffers()
{
    m_scopeSampleBufferSize = std::max(16, m_settings.m_channelSampleRate / 10);

    for (auto& buffer : m_scopeSampleBuffer) {
        buffer.resize(m_scopeSampleBufferSize);
    }

    m_scopeSampleBufferIndex = 0;
    m_spectrumBuffer.clear();
    m_spectrumBuffer.reserve(m_scopeSampleBufferSize);
}

void MeteorDemodSink::resetDetector()
{
    m_sampleCounter = 0;
    m_havePrevSample = false;
    m_noiseFloorInitialized = false;
    m_noiseFloor = 1e-12;
    m_pulseActive = false;
    m_rearmNeeded = false;
    m_pulseStartSample = 0;
    m_pulseLastAboveSample = 0;
    m_pulsePeakPower = 0.0;
    m_pulseFreqMin = 0.0;
    m_pulseFreqMax = 0.0;
    m_pulseFreqFirst = 0.0;
    m_pulseFreqLast = 0.0;
    m_pulseFreqCount = 0;
}

void MeteorDemodSink::sampleToScope(const Complex& sample, double power, double filteredPower, bool detected)
{
    if (!m_scopeSink) {
        return;
    }

    m_scopeSampleBuffer[0][m_scopeSampleBufferIndex] = sample;
    m_scopeSampleBuffer[1][m_scopeSampleBufferIndex] = Complex((Real) power, 0.0f);
    m_scopeSampleBuffer[2][m_scopeSampleBufferIndex] = Complex((Real) filteredPower, 0.0f);
    m_scopeSampleBuffer[3][m_scopeSampleBufferIndex] = Complex(detected ? 1.0f : 0.0f, 0.0f);
    m_scopeSampleBufferIndex++;

    if (m_scopeSampleBufferIndex == m_scopeSampleBufferSize)
    {
        std::vector<ComplexVector::const_iterator> vbegin;

        for (const auto& buffer : m_scopeSampleBuffer) {
            vbegin.push_back(buffer.begin());
        }

        m_scopeSink->feed(vbegin, m_scopeSampleBufferSize);
        m_scopeSampleBufferIndex = 0;
    }
}

void MeteorDemodSink::feedSpectrum(const Complex& sample)
{
    if (!m_spectrumSink) {
        return;
    }

    m_spectrumBuffer.push_back(sample);

    if ((int) m_spectrumBuffer.size() >= m_scopeSampleBufferSize)
    {
        m_spectrumSink->feed(m_spectrumBuffer.begin(), m_spectrumBuffer.end(), false);
        m_spectrumBuffer.clear();
    }
}

void MeteorDemodSink::applyChannelSettings(int channelSampleRate, int channelFrequencyOffset, bool force)
{
    qDebug() << "MeteorDemodSink::applyChannelSettings:"
            << " channelSampleRate:" << channelSampleRate
            << " channelFrequencyOffset:" << channelFrequencyOffset
            << " force:" << force;

    if ((m_channelFrequencyOffset != channelFrequencyOffset) ||
        (m_channelSampleRate != channelSampleRate) || force)
    {
        m_nco.setFreq(-channelFrequencyOffset, channelSampleRate);
        m_channelSampleRate = channelSampleRate;
        m_channelFrequencyOffset = channelFrequencyOffset;
        configureInterpolator();
        m_havePrevSample = false;
    }
}

void MeteorDemodSink::applySettings(const MeteorSettings& settings, const QStringList& settingsKeys, bool force)
{
    qDebug() << "MeteorDemodSink::applySettings:" << settings.getDebugString(settingsKeys, force)
             << " force:" << force;

    const bool sampleRateChanged = (settingsKeys.contains("channelSampleRate")
        && (settings.m_channelSampleRate != m_settings.m_channelSampleRate)) || force;
    const bool lpfChanged = (settingsKeys.contains("powerLPFCutoff")
        && (settings.m_powerLPFCutoff != m_settings.m_powerLPFCutoff)) || force;

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (sampleRateChanged)
    {
        configureInterpolator();
        resizeScopeBuffers();
        resetDetector();
    }

    if (sampleRateChanged || lpfChanged) {
        configurePowerLowpass();
    }
}
