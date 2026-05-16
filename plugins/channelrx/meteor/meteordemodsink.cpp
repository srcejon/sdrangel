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

namespace {
    Real powerToDB(double power)
    {
        return (Real) (10.0 * std::log10(std::max(power, 1e-20)));
    }
}

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

bool MeteorDemodSink::flushPendingPulse()
{
    if (!m_pulseActive) {
        return false;
    }

    Complex zero(0.0f, 0.0f);
    const int tailSamples = std::max(m_settings.m_channelSampleRate / 2, m_settings.m_channelSampleRate / 20 + 4);

    for (int i = 0; i < tailSamples && m_pulseActive; i++) {
        processOneSample(zero);
    }

    return true;
}

void MeteorDemodSink::processOneSample(Complex& ci)
{
    const Real re = ci.real() / SDR_RX_SCALEF;
    const Real im = ci.imag() / SDR_RX_SCALEF;
    const Complex normalized(re, im);
    const double power = std::max<double>(re*re + im*im, 1e-20);
    const double filteredPower = std::max<double>(m_powerLowpass.filter((Real) power), 0.0);

    const bool detected = processDetectorSample(normalized, power, filteredPower);
    sampleToScope(normalized, power, filteredPower, detected);
    feedSpectrum(normalized);
    m_sampleCounter++;
}

bool MeteorDemodSink::processDetectorSample(const Complex& sample, double power, double filteredPower)
{
    if (m_rearmNeeded)
    {
        updateNoiseFloor(filteredPower);
        const double releaseThreshold = getDetectionThresholdPower() * 0.7;

        if (filteredPower < releaseThreshold) {
            m_rearmNeeded = false;
        }

        return false;
    }

    if (!m_pulseActive)
    {
        updateNoiseFloor(filteredPower);
        const double threshold = getDetectionThresholdPower();

        if (m_noiseFloorInitialized && (filteredPower > threshold))
        {
            startPulse(sample, power);
            return true;
        }

        return false;
    }

    updatePulse(sample, power);

    const double releaseThreshold = getDetectionThresholdPower() * 0.7;

    if (filteredPower > releaseThreshold) {
        m_pulseLastAboveSample = m_sampleCounter;
    }

    const int holdSamples = std::max(2, m_settings.m_channelSampleRate / 10);
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

void MeteorDemodSink::startPulse(const Complex& sample, double power)
{
    m_pulseActive = true;
    m_pulseStartSample = m_sampleCounter;
    m_pulseLastAboveSample = m_sampleCounter;
    m_pulseStartDateTimeUtc = sampleCounterToDateTimeUtc(m_pulseStartSample);
    m_pulsePeakPower = power;
    m_pulseSamples.clear();
    m_pulseSamples.push_back(sample);
}

void MeteorDemodSink::updatePulse(const Complex& sample, double power)
{
    if (power > m_pulsePeakPower) {
        m_pulsePeakPower = power;
    }

    const int maxSamples = std::max(1, (m_settings.m_maxDurationMS * m_settings.m_channelSampleRate) / 1000);

    if ((int) m_pulseSamples.size() < maxSamples + std::max(2, m_settings.m_channelSampleRate / 20)) {
        m_pulseSamples.push_back(sample);
    }
}

double MeteorDemodSink::averageFrequency(const std::vector<double>& frequencies, int begin, int end)
{
    begin = std::max(0, begin);
    end = std::min((int) frequencies.size(), end);

    if (begin >= end) {
        return 0.0;
    }

    double sum = 0.0;

    for (int i = begin; i < end; i++) {
        sum += frequencies[i];
    }

    return sum / (double) (end - begin);
}

bool MeteorDemodSink::estimateWindowPeakFrequency(int startIndex, int windowSize, double& frequency, double& strength, double& prominence) const
{
    if ((startIndex < 0) || (windowSize < 8) || ((startIndex + windowSize) > (int) m_pulseSamples.size())) {
        return false;
    }

    const double twoPi = 2.0 * std::acos(-1.0);
    double windowedEnergy = 0.0;
    int bestBin = 0;
    double bestMagnitudeSq = 0.0;
    double magnitudeSqSum = 0.0;
    int binCount = 0;

    for (int bin = -windowSize / 2; bin < windowSize / 2; bin++)
    {
        double realSum = 0.0;
        double imagSum = 0.0;

        for (int i = 0; i < windowSize; i++)
        {
            const double window = 0.5 - 0.5 * std::cos(twoPi * (double) i / (double) (windowSize - 1));
            const Complex sample = m_pulseSamples[startIndex + i] * (Real) window;
            const double angle = -twoPi * (double) bin * (double) i / (double) windowSize;
            const double c = std::cos(angle);
            const double s = std::sin(angle);

            realSum += sample.real() * c - sample.imag() * s;
            imagSum += sample.real() * s + sample.imag() * c;

            if (bin == -windowSize / 2) {
                windowedEnergy += std::norm(sample);
            }
        }

        const double magnitudeSq = realSum*realSum + imagSum*imagSum;
        magnitudeSqSum += magnitudeSq;
        binCount++;

        if (magnitudeSq > bestMagnitudeSq)
        {
            bestMagnitudeSq = magnitudeSq;
            bestBin = bin;
        }
    }

    if ((windowedEnergy <= 1e-20) || !std::isfinite(bestMagnitudeSq)) {
        return false;
    }

    frequency = (double) bestBin * (double) m_settings.m_channelSampleRate / (double) windowSize;
    strength = bestMagnitudeSq;
    prominence = binCount > 0 ? bestMagnitudeSq / std::max(magnitudeSqSum / (double) binCount, 1e-30) : 0.0;
    return std::isfinite(frequency) && std::isfinite(strength) && std::isfinite(prominence);
}

bool MeteorDemodSink::estimatePulseSpectralBand(int windowSize, int hopSize, double& centerFrequency, double& bandwidth, double& contrastDB) const
{
    centerFrequency = 0.0;
    bandwidth = 0.0;
    contrastDB = 0.0;

    if ((windowSize < 8) || ((int) m_pulseSamples.size() < windowSize)) {
        return false;
    }

    const double twoPi = 2.0 * std::acos(-1.0);
    std::vector<double> binPower(windowSize, 0.0);
    int windowCount = 0;

    for (int start = 0; (start + windowSize) <= (int) m_pulseSamples.size(); start += hopSize)
    {
        for (int bin = -windowSize / 2; bin < windowSize / 2; bin++)
        {
            double realSum = 0.0;
            double imagSum = 0.0;

            for (int i = 0; i < windowSize; i++)
            {
                const double window = 0.5 - 0.5 * std::cos(twoPi * (double) i / (double) (windowSize - 1));
                const Complex sample = m_pulseSamples[start + i] * (Real) window;
                const double angle = -twoPi * (double) bin * (double) i / (double) windowSize;
                const double c = std::cos(angle);
                const double s = std::sin(angle);

                realSum += sample.real() * c - sample.imag() * s;
                imagSum += sample.real() * s + sample.imag() * c;
            }

            binPower[bin + windowSize / 2] += realSum*realSum + imagSum*imagSum;
        }

        windowCount++;
    }

    if (windowCount == 0) {
        return false;
    }

    for (double& power : binPower) {
        power /= (double) windowCount;
    }

    const auto peakIt = std::max_element(binPower.begin(), binPower.end());

    if ((peakIt == binPower.end()) || (*peakIt <= 1e-30)) {
        return false;
    }

    std::vector<double> sortedPower = binPower;
    std::sort(sortedPower.begin(), sortedPower.end());
    const double floorPower = std::max(sortedPower[sortedPower.size() / 2], 1e-30);
    const int peakIndex = (int) std::distance(binPower.begin(), peakIt);
    const double peakPower = *peakIt;
    const double threshold = floorPower + 0.25 * (peakPower - floorPower);
    int lowIndex = peakIndex;
    int highIndex = peakIndex;

    while ((lowIndex > 0) && (binPower[lowIndex - 1] >= threshold)) {
        lowIndex--;
    }

    while ((highIndex + 1 < (int) binPower.size()) && (binPower[highIndex + 1] >= threshold)) {
        highIndex++;
    }

    double weightedFrequencySum = 0.0;
    double weightSum = 0.0;

    for (int i = lowIndex; i <= highIndex; i++)
    {
        const double excessPower = std::max(0.0, binPower[i] - floorPower);
        const double frequency = (double) (i - windowSize / 2) * (double) m_settings.m_channelSampleRate / (double) windowSize;
        weightedFrequencySum += frequency * excessPower;
        weightSum += excessPower;
    }

    if (weightSum > 0.0) {
        centerFrequency = weightedFrequencySum / weightSum;
    } else {
        centerFrequency = (double) (peakIndex - windowSize / 2) * (double) m_settings.m_channelSampleRate / (double) windowSize;
    }

    bandwidth = (double) (highIndex - lowIndex + 1) * (double) m_settings.m_channelSampleRate / (double) windowSize;
    contrastDB = 10.0 * std::log10(peakPower / floorPower);

    return std::isfinite(centerFrequency) && std::isfinite(bandwidth) && std::isfinite(contrastDB);
}

bool MeteorDemodSink::estimatePulseFrequency(
    double& centerFrequency,
    double& frequencySpan,
    double& frequencyDrift,
    double& sweepScore,
    double& spectralProminence,
    double& frequencyConcentration,
    double& spectralBandwidth,
    double& spectralBandContrastDB) const
{
    centerFrequency = 0.0;
    frequencySpan = 0.0;
    frequencyDrift = 0.0;
    sweepScore = 0.0;
    spectralProminence = 0.0;
    frequencyConcentration = 0.0;
    spectralBandwidth = 0.0;
    spectralBandContrastDB = 0.0;

    if (m_pulseSamples.size() < 8) {
        return false;
    }

    int windowSize = std::clamp(m_settings.m_channelSampleRate / 4, 32, 512);
    windowSize = std::min(windowSize, (int) m_pulseSamples.size());

    if (windowSize < 8) {
        return false;
    }

    const int hopSize = std::max(1, windowSize / 4);

    estimatePulseSpectralBand(windowSize, hopSize, centerFrequency, spectralBandwidth, spectralBandContrastDB);

    std::vector<double> frequencies;
    std::vector<double> strengths;
    std::vector<double> prominences;

    for (int start = 0; (start + windowSize) <= (int) m_pulseSamples.size(); start += hopSize)
    {
        double frequency = 0.0;
        double strength = 0.0;
        double prominence = 0.0;

        if (estimateWindowPeakFrequency(start, windowSize, frequency, strength, prominence))
        {
            frequencies.push_back(frequency);
            strengths.push_back(strength);
            prominences.push_back(prominence);
        }
    }

    if (frequencies.empty()) {
        return false;
    }

    const double maxStrength = *std::max_element(strengths.begin(), strengths.end());
    const double minSelectedStrength = maxStrength * 0.25;
    std::vector<double> selectedFrequencies;

    for (int i = 0; i < (int) frequencies.size(); i++)
    {
        if (strengths[i] >= minSelectedStrength)
        {
            selectedFrequencies.push_back(frequencies[i]);
            spectralProminence = std::max(spectralProminence, prominences[i]);
        }
    }

    if (!selectedFrequencies.empty()) {
        frequencies = selectedFrequencies;
    } else if (!prominences.empty()) {
        spectralProminence = *std::max_element(prominences.begin(), prominences.end());
    }

    const double trackCenterFrequency = averageFrequency(frequencies, 0, (int) frequencies.size());

    if (spectralBandwidth <= 0.0) {
        centerFrequency = trackCenterFrequency;
    }

    const double concentrationWindowHz = std::max(20.0, (double) m_settings.m_channelSampleRate * 0.03);
    int bestConcentrationCount = 0;

    for (double frequency : frequencies)
    {
        int concentrationCount = 0;

        for (double otherFrequency : frequencies)
        {
            if (std::fabs(otherFrequency - frequency) <= concentrationWindowHz) {
                concentrationCount++;
            }
        }

        bestConcentrationCount = std::max(bestConcentrationCount, concentrationCount);
    }

    frequencyConcentration = frequencies.empty() ? 0.0 : (double) bestConcentrationCount / (double) frequencies.size();

    if (frequencies.size() == 1)
    {
        return std::isfinite(centerFrequency);
    }

    std::vector<double> sortedFrequencies = frequencies;
    std::sort(sortedFrequencies.begin(), sortedFrequencies.end());

    const int count = (int) sortedFrequencies.size();
    const int lowIndex = count >= 5 ? (count - 1) / 10 : 0;
    const int highIndex = count >= 5 ? (9 * (count - 1)) / 10 : count - 1;
    const int edgeCount = std::clamp(count / 4, 1, 4);

    frequencySpan = sortedFrequencies[highIndex] - sortedFrequencies[lowIndex];
    frequencyDrift = averageFrequency(frequencies, count - edgeCount, count) - averageFrequency(frequencies, 0, edgeCount);

    if (count >= 4)
    {
        const double meanX = 0.5 * (double) (count - 1);
        const double meanY = averageFrequency(frequencies, 0, count);
        double ssXX = 0.0;
        double ssXY = 0.0;
        double ssYY = 0.0;

        for (int i = 0; i < count; i++)
        {
            const double dx = (double) i - meanX;
            const double dy = frequencies[i] - meanY;
            ssXX += dx*dx;
            ssXY += dx*dy;
            ssYY += dy*dy;
        }

        if ((ssXX > 0.0) && (ssYY > 0.0))
        {
            const double r = ssXY / std::sqrt(ssXX * ssYY);
            sweepScore = r * r;
        }
    }

    return std::isfinite(centerFrequency) && std::isfinite(frequencySpan) && std::isfinite(frequencyDrift) && std::isfinite(sweepScore) && std::isfinite(spectralProminence) && std::isfinite(frequencyConcentration) && std::isfinite(spectralBandwidth) && std::isfinite(spectralBandContrastDB);
}

void MeteorDemodSink::finishPulse(bool forceRejected)
{
    const quint64 endSample = std::max(m_pulseLastAboveSample, m_pulseStartSample);
    const double durationS = (double) (endSample - m_pulseStartSample + 1) / (double) m_settings.m_channelSampleRate;
    const double durationMS = 1000.0 * durationS;
    double centerFrequency = 0.0;
    double frequencySpan = 0.0;
    double frequencyDrift = 0.0;
    double sweepScore = 0.0;
    double spectralProminence = 0.0;
    double frequencyConcentration = 0.0;
    double spectralBandwidth = 0.0;
    double spectralBandContrastDB = 0.0;

    estimatePulseFrequency(centerFrequency, frequencySpan, frequencyDrift, sweepScore, spectralProminence, frequencyConcentration, spectralBandwidth, spectralBandContrastDB);

    const bool durationOK = (durationMS >= m_settings.m_minDurationMS) && (durationMS <= m_settings.m_maxDurationMS);
    const bool sweepRejected = (m_settings.m_maxFrequencyDrift > 0.0f)
        && (sweepScore >= 0.75)
        && ((std::fabs(frequencySpan) > m_settings.m_maxFrequencyDrift)
            || (std::fabs(frequencyDrift) > m_settings.m_maxFrequencyDrift));
    const bool driftOK = !sweepRejected;
    const double sampleDurationMS = 1000.0 / (double) std::max(1, m_settings.m_channelSampleRate);
    const double compactFrequencyLimit = std::max(10.0, std::min(50.0, (double) m_settings.m_channelSampleRate * 0.05));
    const bool clippedDurationOK = forceRejected
        && (durationMS >= m_settings.m_minDurationMS)
        && (durationMS <= (double) m_settings.m_maxDurationMS + 2.0 * sampleDurationMS);
    const bool compactClippedMeteor = clippedDurationOK
        && (std::fabs(frequencySpan) <= compactFrequencyLimit)
        && (std::fabs(frequencyDrift) <= compactFrequencyLimit)
        && (sweepScore < 0.75);
    const double stableFrequencyLimit = std::max(compactFrequencyLimit, std::min(180.0, (double) m_settings.m_channelSampleRate * 0.18));
    const bool stableLineOK = (spectralProminence >= 7.5)
        && (frequencyConcentration >= 0.60)
        && (std::fabs(frequencySpan) <= stableFrequencyLimit)
        && (std::fabs(frequencyDrift) <= stableFrequencyLimit);
    const bool boundedBandOK = (spectralProminence >= 9.0)
        && (spectralBandContrastDB >= 3.0)
        && (spectralBandwidth >= compactFrequencyLimit)
        && (spectralBandwidth <= stableFrequencyLimit)
        && (std::fabs(frequencyDrift) <= m_settings.m_maxFrequencyDrift);
    const bool spectralEvidenceOK = stableLineOK || boundedBandOK;
    const bool shortStandaloneLine = stableLineOK && !compactClippedMeteor && (durationS < 0.5);
    const bool directAccepted = ((!forceRejected && durationOK) || compactClippedMeteor) && driftOK && spectralEvidenceOK && !shortStandaloneLine && m_messageQueueToGUI;
    const int broadValidationGapSamples = std::max(2, m_settings.m_channelSampleRate / 10);

    if (m_pendingBroadPulse.m_valid
        && (m_pulseStartSample > m_pendingBroadPulse.m_endSample + (quint64) broadValidationGapSamples))
    {
        m_pendingBroadPulse = PulseReport();
    }

    const bool broadPulseCandidate = forceRejected
        && durationOK
        && driftOK
        && !spectralEvidenceOK
        && (durationS >= 0.5)
        && (spectralProminence >= 9.0)
        && (frequencyConcentration <= 0.35)
        && (sweepScore < 0.25)
        && (std::fabs(frequencyDrift) <= stableFrequencyLimit);
    const bool validatesPendingBroadPulse = m_messageQueueToGUI
        && stableLineOK
        && (durationS <= 1.0)
        && m_pendingBroadPulse.m_valid
        && (m_pulseStartSample >= m_pendingBroadPulse.m_endSample)
        && ((m_pulseStartSample - m_pendingBroadPulse.m_endSample) <= (quint64) broadValidationGapSamples);
    const bool accepted = directAccepted || validatesPendingBroadPulse;

    qDebug() << "MeteorDemodSink::finishPulse:"
             << " accepted:" << accepted
             << " directAccepted:" << directAccepted
             << " broadPulseCandidate:" << broadPulseCandidate
             << " validatesPendingBroadPulse:" << validatesPendingBroadPulse
             << " forceRejected:" << forceRejected
             << " durationOK:" << durationOK
             << " compactClippedMeteor:" << compactClippedMeteor
             << " driftOK:" << driftOK
             << " sweepRejected:" << sweepRejected
             << " spectralEvidenceOK:" << spectralEvidenceOK
             << " stableLineOK:" << stableLineOK
             << " boundedBandOK:" << boundedBandOK
             << " shortStandaloneLine:" << shortStandaloneLine
             << " durationS:" << durationS
             << " peakPowerDB:" << 10.0 * std::log10(std::max(m_pulsePeakPower, 1e-20))
             << " backgroundPowerDB:" << 10.0 * std::log10(std::max(m_noiseFloor, 1e-20))
             << " centerFrequency:" << centerFrequency
             << " frequencySpan:" << frequencySpan
             << " frequencyDrift:" << frequencyDrift
             << " sweepScore:" << sweepScore
             << " spectralProminence:" << spectralProminence
             << " frequencyConcentration:" << frequencyConcentration
             << " spectralBandwidth:" << spectralBandwidth
             << " spectralBandContrastDB:" << spectralBandContrastDB
             << " startSample:" << m_pulseStartSample
             << " endSample:" << endSample;

    if (accepted)
    {
        PulseReport report;

        if (validatesPendingBroadPulse)
        {
            report = m_pendingBroadPulse;
            report.m_centerFrequency = centerFrequency;
            report.m_frequencySpan = std::min(stableFrequencyLimit, std::max(std::fabs(report.m_frequencySpan), compactFrequencyLimit * 3.0));
        }
        else
        {
            report.m_valid = true;
            report.m_dateTimeUtc = m_pulseStartDateTimeUtc;
            report.m_startSample = m_pulseStartSample;
            report.m_endSample = endSample;
            report.m_peakPower = m_pulsePeakPower;
            report.m_backgroundPower = m_noiseFloor;
            report.m_durationS = durationS;
            report.m_centerFrequency = centerFrequency;
            report.m_frequencySpan = frequencySpan > 0.0 ? frequencySpan : spectralBandwidth;
            report.m_frequencyDrift = frequencyDrift;
        }

        const double peakAmplitude = std::sqrt(std::max(report.m_peakPower, 0.0));
        const double peakPowerDB = 10.0 * std::log10(std::max(report.m_peakPower, 1e-20));
        const double backgroundPowerDB = 10.0 * std::log10(std::max(report.m_backgroundPower, 1e-20));

        m_messageQueueToGUI->push(MsgMeteorDetected::create(
            report.m_dateTimeUtc,
            peakAmplitude,
            peakPowerDB,
            backgroundPowerDB,
            report.m_durationS,
            report.m_centerFrequency,
            report.m_frequencySpan,
            report.m_frequencyDrift,
            m_settings.m_channelSampleRate
        ));

        m_pendingBroadPulse = PulseReport();
    }
    else if (broadPulseCandidate)
    {
        m_pendingBroadPulse.m_valid = true;
        m_pendingBroadPulse.m_dateTimeUtc = m_pulseStartDateTimeUtc;
        m_pendingBroadPulse.m_startSample = m_pulseStartSample;
        m_pendingBroadPulse.m_endSample = endSample;
        m_pendingBroadPulse.m_peakPower = m_pulsePeakPower;
        m_pendingBroadPulse.m_backgroundPower = m_noiseFloor;
        m_pendingBroadPulse.m_durationS = durationS;
        m_pendingBroadPulse.m_centerFrequency = centerFrequency;
        m_pendingBroadPulse.m_frequencySpan = frequencySpan;
        m_pendingBroadPulse.m_frequencyDrift = frequencyDrift;
    }

    m_pulseActive = false;
    m_pulseSamples.clear();
}

QDateTime MeteorDemodSink::sampleCounterToDateTimeUtc(quint64 sampleCounter) const
{
    if (!m_streamStartDateTimeUtc.isValid() || (m_settings.m_channelSampleRate <= 0)) {
        return QDateTime::currentDateTimeUtc();
    }

    const qint64 offsetMSecs = (qint64) std::llround(1000.0 * (double) sampleCounter / (double) m_settings.m_channelSampleRate);
    return m_streamStartDateTimeUtc.addMSecs(offsetMSecs);
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
    m_noiseFloorInitialized = false;
    m_noiseFloor = 1e-12;
    m_pulseActive = false;
    m_rearmNeeded = false;
    m_streamStartDateTimeUtc = QDateTime::currentDateTimeUtc();
    m_pulseStartSample = 0;
    m_pulseLastAboveSample = 0;
    m_pulsePeakPower = 0.0;
    m_pulseSamples.clear();
    m_pendingBroadPulse = PulseReport();
}

void MeteorDemodSink::sampleToScope(const Complex& sample, double power, double filteredPower, bool detected)
{
    if (!m_scopeSink) {
        return;
    }

    m_scopeSampleBuffer[0][m_scopeSampleBufferIndex] = sample;
    m_scopeSampleBuffer[1][m_scopeSampleBufferIndex] = Complex(powerToDB(power), 0.0f);
    m_scopeSampleBuffer[2][m_scopeSampleBufferIndex] = Complex(powerToDB(filteredPower), 0.0f);
    m_scopeSampleBuffer[3][m_scopeSampleBufferIndex] = Complex(detected ? 1.0f : 0.0f, 0.0f);
    m_scopeSampleBuffer[4][m_scopeSampleBufferIndex] = Complex(powerToDB(m_noiseFloor), 0.0f);
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
