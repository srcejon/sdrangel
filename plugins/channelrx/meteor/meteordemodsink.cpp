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
#include <limits>

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
    m_scopeSampleBufferIndex(0),
    m_spectralFrameSize(0),
    m_spectralHopSize(0),
    m_spectralNoiseFloorInitialized(false),
    m_spectralEventActiveForScope(false)
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
    if (!m_pulseActive && m_spectralEvents.empty()) {
        return false;
    }

    Complex zero(0.0f, 0.0f);
    const int tailSamples = std::max(
        m_settings.m_channelSampleRate / 2,
        m_spectralFrameSize + m_spectralHopSize * 8 + m_settings.m_channelSampleRate / 20 + 4
    );

    for (int i = 0; i < tailSamples && (m_pulseActive || !m_spectralEvents.empty()); i++) {
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

    const bool powerDetected = processDetectorSample(normalized, power, filteredPower);
    const bool spectralDetected = processSpectralSample(normalized, power);
    const bool detected = powerDetected || spectralDetected;
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
        const bool detectorWarm = m_noiseFloorInitialized
            && (m_sampleCounter >= (quint64) std::max(1, m_settings.m_channelSampleRate / 2))
            && (m_noiseFloor > 1e-16);

        if (detectorWarm && (filteredPower > threshold))
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

bool MeteorDemodSink::processSpectralSample(const Complex& sample, double power)
{
    (void) power;

    if ((m_spectralFrameSize < 8) || (m_spectralHopSize < 1)) {
        return false;
    }

    m_spectralFrameBuffer.push_back(sample);

    if ((int) m_spectralFrameBuffer.size() >= m_spectralFrameSize)
    {
        const quint64 frameStartSample = m_sampleCounter + 1 - (quint64) m_spectralFrameBuffer.size();
        processSpectralFrame(frameStartSample);

        const int removeCount = std::min(m_spectralHopSize, (int) m_spectralFrameBuffer.size());
        m_spectralFrameBuffer.erase(m_spectralFrameBuffer.begin(), m_spectralFrameBuffer.begin() + removeCount);
    }

    return m_spectralEventActiveForScope;
}

void MeteorDemodSink::processSpectralFrame(quint64 frameStartSample)
{
    if ((int) m_spectralFrameBuffer.size() < m_spectralFrameSize) {
        return;
    }

    const double twoPi = 2.0 * std::acos(-1.0);
    std::vector<double> binPower(m_spectralFrameSize, 0.0);
    double framePeakPower = 1e-20;

    for (int i = 0; i < m_spectralFrameSize; i++) {
        framePeakPower = std::max(framePeakPower, (double) std::norm(m_spectralFrameBuffer[i]));
    }

    for (int bin = -m_spectralFrameSize / 2; bin < m_spectralFrameSize / 2; bin++)
    {
        double realSum = 0.0;
        double imagSum = 0.0;

        for (int i = 0; i < m_spectralFrameSize; i++)
        {
            const double window = 0.5 - 0.5 * std::cos(twoPi * (double) i / (double) (m_spectralFrameSize - 1));
            const Complex windowedSample = m_spectralFrameBuffer[i] * (Real) window;
            const double angle = -twoPi * (double) bin * (double) i / (double) m_spectralFrameSize;
            const double c = std::cos(angle);
            const double s = std::sin(angle);

            realSum += windowedSample.real() * c - windowedSample.imag() * s;
            imagSum += windowedSample.real() * s + windowedSample.imag() * c;
        }

        binPower[bin + m_spectralFrameSize / 2] = std::max(realSum*realSum + imagSum*imagSum, 1e-30);
    }

    if (!m_spectralNoiseFloorInitialized || (m_spectralNoiseFloor.size() != binPower.size()))
    {
        m_spectralNoiseFloor = binPower;
        m_spectralNoiseFloorInitialized = true;
        return;
    }

    const bool noiseFloorWarm = m_noiseFloorInitialized
        && (m_sampleCounter >= (quint64) std::max(1, m_settings.m_channelSampleRate / 2))
        && (m_noiseFloor > 1e-16);

    if (!noiseFloorWarm)
    {
        for (int i = 0; i < (int) binPower.size(); i++) {
            m_spectralNoiseFloor[i] = 0.75 * std::max(m_spectralNoiseFloor[i], 1e-30) + 0.25 * binPower[i];
        }

        return;
    }

    std::vector<SpectralBand> bands = detectSpectralBands(binPower, framePeakPower);
    std::vector<bool> activeBins(binPower.size(), false);

    for (const SpectralBand& band : bands)
    {
        for (int i = std::max(0, band.m_lowIndex); i <= std::min((int) activeBins.size() - 1, band.m_highIndex); i++) {
            activeBins[i] = true;
        }
    }

    for (int i = 0; i < (int) binPower.size(); i++)
    {
        const double alpha = m_spectralNoiseFloor[i] < (binPower[i] * 0.01) ? 0.25 : (activeBins[i] ? 0.001 : 0.02);
        m_spectralNoiseFloor[i] = (1.0 - alpha) * std::max(m_spectralNoiseFloor[i], 1e-30) + alpha * binPower[i];
    }

    updateSpectralEvents(bands, frameStartSample + (quint64) (m_spectralFrameSize / 2));
    m_spectralEventActiveForScope = !bands.empty() || !m_spectralEvents.empty();
}

std::vector<MeteorDemodSink::SpectralBand> MeteorDemodSink::detectSpectralBands(const std::vector<double>& binPower, double framePeakPower)
{
    std::vector<SpectralBand> bands;

    if (binPower.empty() || (m_spectralNoiseFloor.size() != binPower.size())) {
        return bands;
    }

    std::vector<double> sortedPower = binPower;
    std::sort(sortedPower.begin(), sortedPower.end());

    const double medianFramePower = std::max(sortedPower[sortedPower.size() / 2], 1e-30);
    const double minimumBinNoise = medianFramePower * 0.25;
    const double thresholdRatio = std::max(10.0, std::pow(10.0, m_settings.m_detectionThresholdDB / 10.0));
    const double spanRatio = std::max(1.6, thresholdRatio * 0.30);
    const double binWidth = (double) m_settings.m_channelSampleRate / (double) m_spectralFrameSize;
    const double maxBandWidth = std::min(
        (double) m_settings.m_channelSampleRate * 0.45,
        std::max(220.0, (double) m_settings.m_channelSampleRate * 0.25)
    );

    for (int i = 0; i < (int) binPower.size(); i++)
    {
        const double ratio = binPower[i] / std::max({m_spectralNoiseFloor[i], minimumBinNoise, 1e-30});

        if (ratio < thresholdRatio) {
            continue;
        }

        int lowIndex = i;
        int highIndex = i;

        while ((lowIndex > 0)
            && ((binPower[lowIndex - 1] / std::max({m_spectralNoiseFloor[lowIndex - 1], minimumBinNoise, 1e-30})) >= spanRatio))
        {
            lowIndex--;
        }

        while ((highIndex + 1 < (int) binPower.size())
            && ((binPower[highIndex + 1] / std::max({m_spectralNoiseFloor[highIndex + 1], minimumBinNoise, 1e-30})) >= spanRatio))
        {
            highIndex++;
        }

        SpectralBand band;
        double weightedFrequencySum = 0.0;
        double weightSum = 0.0;
        double peakRatio = 0.0;
        double peakPower = 0.0;
        double peakNoise = 1e-30;

        for (int j = lowIndex; j <= highIndex; j++)
        {
            const double noise = std::max({m_spectralNoiseFloor[j], minimumBinNoise, 1e-30});
            const double excessPower = std::max(0.0, binPower[j] - noise);
            const double frequency = ((double) j - (double) m_spectralFrameSize / 2.0) * binWidth;
            const double binRatio = binPower[j] / noise;

            weightedFrequencySum += frequency * excessPower;
            weightSum += excessPower;

            if (binPower[j] > peakPower)
            {
                peakPower = binPower[j];
                peakNoise = noise;
            }

            peakRatio = std::max(peakRatio, binRatio);
        }

        band.m_valid = true;
        band.m_lowIndex = lowIndex;
        band.m_highIndex = highIndex;
        band.m_lowFrequency = ((double) lowIndex - (double) m_spectralFrameSize / 2.0) * binWidth - 0.5 * binWidth;
        band.m_highFrequency = ((double) highIndex - (double) m_spectralFrameSize / 2.0) * binWidth + 0.5 * binWidth;
        band.m_bandwidth = std::max(binWidth, band.m_highFrequency - band.m_lowFrequency);
        band.m_centerFrequency = weightSum > 0.0 ? weightedFrequencySum / weightSum : 0.5 * (band.m_lowFrequency + band.m_highFrequency);
        band.m_peakBinPower = peakPower;
        band.m_totalExcessPower = weightSum;
        band.m_contrastDB = 10.0 * std::log10(std::max(peakPower, 1e-30) / std::max(peakNoise, 1e-30));
        band.m_peakRatio = peakRatio;
        band.m_framePeakPower = framePeakPower;

        if ((band.m_bandwidth <= maxBandWidth) && (band.m_totalExcessPower > 0.0)) {
            bands.push_back(band);
        }

        i = highIndex;
    }

    return bands;
}

void MeteorDemodSink::updateSpectralEvents(const std::vector<SpectralBand>& bands, quint64 frameCenterSample)
{
    std::vector<bool> bandMatched(bands.size(), false);
    const int maxMissingFrames = std::max(2, (int) std::ceil(0.15 * (double) m_settings.m_channelSampleRate / (double) std::max(1, m_spectralHopSize)));

    for (SpectralEvent& event : m_spectralEvents)
    {
        if (!event.m_valid) {
            continue;
        }

        int bestBandIndex = -1;
        double bestDistance = std::numeric_limits<double>::max();
        const double lastFrequency = event.m_trackFrequencies.empty() ? 0.5 * (event.m_minFrequency + event.m_maxFrequency) : event.m_trackFrequencies.back();

        for (int i = 0; i < (int) bands.size(); i++)
        {
            if (bandMatched[i]) {
                continue;
            }

            const SpectralBand& band = bands[i];
            const double padding = std::max(12.0, (double) m_settings.m_channelSampleRate * 0.012);
            const double maxTrackingJump = std::max(45.0, (double) m_settings.m_channelSampleRate * 0.08);
            const double eventHalfWidth = std::min(
                maxTrackingJump,
                std::max(event.m_maxBandwidth, band.m_bandwidth) * 0.5 + padding
            );
            const bool overlaps = (band.m_highFrequency >= lastFrequency - eventHalfWidth)
                && (band.m_lowFrequency <= lastFrequency + eventHalfWidth);
            const double distance = std::fabs(band.m_centerFrequency - lastFrequency);

            if (overlaps && (distance < bestDistance))
            {
                bestDistance = distance;
                bestBandIndex = i;
            }
        }

        if (bestBandIndex >= 0)
        {
            updateSpectralEvent(event, bands[bestBandIndex], frameCenterSample);
            bandMatched[bestBandIndex] = true;
        }
        else
        {
            event.m_missingFrames++;

            if (event.m_missingFrames > maxMissingFrames)
            {
                finishSpectralEvent(event);
                event.m_valid = false;
            }
        }

        const double durationMS = 1000.0 * (double) (event.m_lastCenterSample - event.m_startCenterSample + m_spectralHopSize)
            / (double) std::max(1, m_settings.m_channelSampleRate);

        if (event.m_valid && (durationMS > (double) m_settings.m_maxDurationMS))
        {
            finishSpectralEvent(event);
            event.m_valid = false;
        }
    }

    m_spectralEvents.erase(
        std::remove_if(
            m_spectralEvents.begin(),
            m_spectralEvents.end(),
            [](const SpectralEvent& event) { return !event.m_valid; }),
        m_spectralEvents.end()
    );

    for (int i = 0; i < (int) bands.size(); i++)
    {
        if (bandMatched[i]) {
            continue;
        }

        SpectralEvent event;
        event.m_valid = true;
        event.m_startCenterSample = frameCenterSample;
        event.m_lastCenterSample = frameCenterSample;
        event.m_backgroundPower = std::max(m_noiseFloor, 1e-20);
        updateSpectralEvent(event, bands[i], frameCenterSample);
        m_spectralEvents.push_back(event);
    }
}

void MeteorDemodSink::updateSpectralEvent(SpectralEvent& event, const SpectralBand& band, quint64 frameCenterSample)
{
    if (!event.m_valid) {
        return;
    }

    if (event.m_trackFrequencies.empty())
    {
        event.m_minFrequency = band.m_lowFrequency;
        event.m_maxFrequency = band.m_highFrequency;
    }
    else
    {
        event.m_minFrequency = std::min(event.m_minFrequency, band.m_lowFrequency);
        event.m_maxFrequency = std::max(event.m_maxFrequency, band.m_highFrequency);
    }

    event.m_lastCenterSample = frameCenterSample;
    event.m_missingFrames = 0;
    event.m_peakPower = std::max(event.m_peakPower, band.m_framePeakPower);
    event.m_maxBandwidth = std::max(event.m_maxBandwidth, band.m_bandwidth);
    event.m_weightedFrequencySum += band.m_centerFrequency * std::max(band.m_totalExcessPower, 1e-30);
    event.m_weightSum += std::max(band.m_totalExcessPower, 1e-30);
    event.m_maxContrastDB = std::max(event.m_maxContrastDB, band.m_contrastDB);
    event.m_maxPeakRatio = std::max(event.m_maxPeakRatio, band.m_peakRatio);
    event.m_trackFrequencies.push_back(band.m_centerFrequency);
    event.m_trackSamples.push_back(frameCenterSample);
    event.m_trackStrengths.push_back(std::max(band.m_totalExcessPower, 1e-30));
}

void MeteorDemodSink::finishSpectralEvent(const SpectralEvent& event)
{
    if (!event.m_valid || !m_messageQueueToGUI || event.m_trackFrequencies.empty()) {
        return;
    }

    double startSampleEstimate = 0.0;
    const double durationSamples = estimateSpectralEventDurationSamples(event, startSampleEstimate);
    const quint64 startSample = startSampleEstimate > 0.0 ? (quint64) std::llround(startSampleEstimate) : 0;
    const quint64 durationSampleCount = (quint64) std::max(1.0, (double) std::llround(durationSamples));
    const quint64 endSample = startSample + durationSampleCount - 1;
    const double durationS = durationSamples / (double) std::max(1, m_settings.m_channelSampleRate);
    const double durationMS = 1000.0 * durationS;
    const bool durationOK = (durationMS >= m_settings.m_minDurationMS) && (durationMS <= m_settings.m_maxDurationMS);
    const double centerFrequency = event.m_weightSum > 0.0
        ? event.m_weightedFrequencySum / event.m_weightSum
        : averageFrequency(event.m_trackFrequencies, 0, (int) event.m_trackFrequencies.size());
    const double frequencySpan = std::max(std::fabs(event.m_maxFrequency - event.m_minFrequency), event.m_maxBandwidth);
    const int count = (int) event.m_trackFrequencies.size();
    const int edgeCount = std::clamp(count / 4, 1, 4);
    const double frequencyDrift = count > 1
        ? averageFrequency(event.m_trackFrequencies, count - edgeCount, count) - averageFrequency(event.m_trackFrequencies, 0, edgeCount)
        : 0.0;
    double sweepScore = 0.0;

    if (count >= 4)
    {
        const double meanX = 0.5 * (double) (count - 1);
        const double meanY = averageFrequency(event.m_trackFrequencies, 0, count);
        double ssXX = 0.0;
        double ssXY = 0.0;
        double ssYY = 0.0;

        for (int i = 0; i < count; i++)
        {
            const double dx = (double) i - meanX;
            const double dy = event.m_trackFrequencies[i] - meanY;
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

    const double compactFrequencyLimit = std::max(10.0, std::min(50.0, (double) m_settings.m_channelSampleRate * 0.05));
    const double stableFrequencyLimit = std::max(compactFrequencyLimit, std::min(220.0, (double) m_settings.m_channelSampleRate * 0.22));
    const double sweepFrequencyLimit = std::max(
        80.0,
        m_settings.m_maxFrequencyDrift > 0.0f
            ? std::min((double) m_settings.m_maxFrequencyDrift, (double) m_settings.m_channelSampleRate * 0.12)
            : (double) m_settings.m_channelSampleRate * 0.12
    );
    const bool smoothSweepRejected = (sweepScore >= 0.65)
        && ((std::fabs(frequencyDrift) > sweepFrequencyLimit) || (frequencySpan > sweepFrequencyLimit));
    const bool longDriftRejected = (durationS >= 1.0)
        && (std::fabs(frequencyDrift) > sweepFrequencyLimit)
        && (frequencySpan > sweepFrequencyLimit);
    const bool sweepRejected = smoothSweepRejected || longDriftRejected;
    const double peakAboveBackgroundDB = 10.0 * std::log10(std::max(event.m_peakPower, 1e-20) / std::max(event.m_backgroundPower, 1e-20));
    const bool enoughFrames = (count >= 3)
        || ((count >= 2) && (event.m_maxContrastDB >= 14.0) && (peakAboveBackgroundDB >= 10.0));
    const bool strongLineOK = (event.m_maxContrastDB >= 10.0)
        && (peakAboveBackgroundDB >= std::max(8.0, (double) m_settings.m_detectionThresholdDB));
    const bool boundedBandOK = (event.m_maxContrastDB >= 8.0)
        && (peakAboveBackgroundDB >= 8.0)
        && (event.m_maxBandwidth <= stableFrequencyLimit);
    const bool spectralEvidenceOK = strongLineOK || boundedBandOK;
    const bool insideUsableBandwidth = std::fabs(centerFrequency) <= (double) m_settings.m_channelSampleRate * 0.30;
    const bool duplicate = isDuplicateDetection(startSample, endSample, centerFrequency, frequencySpan);
    const bool accepted = enoughFrames && durationOK && !sweepRejected && spectralEvidenceOK && insideUsableBandwidth && !duplicate;

    if (accepted || (spectralEvidenceOK && (durationS >= 1.0)))
    {
        qDebug() << "MeteorDemodSink::finishSpectralEvent:"
                 << " accepted:" << accepted
                 << " duplicate:" << duplicate
                 << " durationOK:" << durationOK
                 << " enoughFrames:" << enoughFrames
                 << " sweepRejected:" << sweepRejected
                 << " smoothSweepRejected:" << smoothSweepRejected
                 << " longDriftRejected:" << longDriftRejected
                 << " spectralEvidenceOK:" << spectralEvidenceOK
                 << " insideUsableBandwidth:" << insideUsableBandwidth
                 << " strongLineOK:" << strongLineOK
                 << " boundedBandOK:" << boundedBandOK
             << " durationS:" << durationS
             << " peakPowerDB:" << 10.0 * std::log10(std::max(event.m_peakPower, 1e-20))
                 << " backgroundPowerDB:" << 10.0 * std::log10(std::max(event.m_backgroundPower, 1e-20))
                 << " peakAboveBackgroundDB:" << peakAboveBackgroundDB
                 << " centerFrequency:" << centerFrequency
                 << " frequencySpan:" << frequencySpan
                 << " frequencyDrift:" << frequencyDrift
                 << " sweepScore:" << sweepScore
                 << " maxBandwidth:" << event.m_maxBandwidth
                 << " maxContrastDB:" << event.m_maxContrastDB
                 << " maxPeakRatio:" << event.m_maxPeakRatio
                 << " frames:" << event.m_trackFrequencies.size()
                 << " startSample:" << startSample
                 << " endSample:" << endSample;
    }

    if (!accepted) {
        return;
    }

    PulseReport report;
    report.m_valid = true;
    report.m_dateTimeUtc = sampleCounterToDateTimeUtc(startSample);
    report.m_startSample = startSample;
    report.m_endSample = endSample;
    report.m_peakPower = event.m_peakPower;
    report.m_backgroundPower = event.m_backgroundPower;
    report.m_durationS = durationS;
    report.m_centerFrequency = centerFrequency;
    report.m_frequencySpan = frequencySpan;
    report.m_frequencyDrift = frequencyDrift;

    emitOrDeferSpectralReport(report);
}

double MeteorDemodSink::estimateSpectralEventDurationSamples(const SpectralEvent& event, double& startSample) const
{
    const int count = (int) event.m_trackSamples.size();

    if ((count <= 0) || (m_spectralHopSize <= 0))
    {
        startSample = (double) event.m_startCenterSample;
        return 1.0;
    }

    double maxStrength = 0.0;

    for (double strength : event.m_trackStrengths) {
        maxStrength = std::max(maxStrength, strength);
    }

    if (maxStrength <= 0.0)
    {
        startSample = (double) event.m_startCenterSample - 0.5 * (double) m_spectralHopSize;
        return std::max(1.0, (double) (event.m_lastCenterSample - event.m_startCenterSample + m_spectralHopSize));
    }

    const double edgeThreshold = 0.25 * maxStrength;
    const double firstStrength = event.m_trackStrengths.empty() ? maxStrength : event.m_trackStrengths.front();
    const double lastStrength = event.m_trackStrengths.empty() ? maxStrength : event.m_trackStrengths.back();
    const double leadingSamples = (double) m_spectralHopSize * std::clamp(1.0 - edgeThreshold / std::max(firstStrength, edgeThreshold), 0.0, 1.0);
    const double trailingSamples = (double) m_spectralHopSize * std::clamp(1.0 - edgeThreshold / std::max(lastStrength, edgeThreshold), 0.0, 1.0);
    const double centerSpan = count > 1
        ? (double) (event.m_trackSamples.back() - event.m_trackSamples.front())
        : 0.0;

    startSample = (double) event.m_trackSamples.front() - leadingSamples;
    return std::max(1.0, centerSpan + leadingSamples + trailingSamples);
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
    const double threshold = floorPower + 0.10 * (peakPower - floorPower);
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
    const double compactFrequencyLimit = std::max(10.0, std::min(50.0, (double) m_settings.m_channelSampleRate * 0.05));
    // Max-duration scalar power pulses are the failure mode the spectral tracker avoids.
    const bool compactClippedMeteor = false;
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
    const double peakAboveBackgroundDB = 10.0 * std::log10(std::max(m_pulsePeakPower, 1e-20) / std::max(m_noiseFloor, 1e-20));
    const bool veryShortLineOK = (durationS <= 0.25)
        && (peakAboveBackgroundDB >= 6.0)
        && (spectralProminence >= 4.0)
        && (frequencyConcentration >= 0.60)
        && (spectralBandContrastDB >= 8.0)
        && (spectralBandwidth <= stableFrequencyLimit)
        && (std::fabs(frequencyDrift) <= stableFrequencyLimit);
    const bool spectralEvidenceOK = stableLineOK || boundedBandOK || veryShortLineOK;
    const bool strongShortLine = stableLineOK && (peakAboveBackgroundDB >= 12.0);
    const bool shortStandaloneLine = stableLineOK && !strongShortLine && !veryShortLineOK && !compactClippedMeteor && (durationS < 0.5);
    const bool directAccepted = false;
    const int broadValidationGapSamples = std::max(2, m_settings.m_channelSampleRate / 10);
    const double reportedFrequencySpan = std::max(std::fabs(frequencySpan), spectralBandwidth);

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
    const bool broadValidationLineOK = (spectralProminence >= 6.0)
        && (frequencyConcentration >= 0.60)
        && (spectralBandContrastDB >= 6.0)
        && (spectralBandwidth <= stableFrequencyLimit)
        && (std::fabs(frequencyDrift) <= stableFrequencyLimit);
    const bool validatesPendingBroadPulse = m_messageQueueToGUI
        && broadValidationLineOK
        && (durationS <= 1.0)
        && m_pendingBroadPulse.m_valid
        && (m_pulseStartSample >= m_pendingBroadPulse.m_endSample)
        && ((m_pulseStartSample - m_pendingBroadPulse.m_endSample) <= (quint64) broadValidationGapSamples);
    const bool accepted = directAccepted || validatesPendingBroadPulse;

    qDebug() << "MeteorDemodSink::finishPulse:"
             << " accepted:" << accepted
             << " directAccepted:" << directAccepted
             << " broadPulseCandidate:" << broadPulseCandidate
             << " broadValidationLineOK:" << broadValidationLineOK
             << " validatesPendingBroadPulse:" << validatesPendingBroadPulse
             << " forceRejected:" << forceRejected
             << " durationOK:" << durationOK
             << " compactClippedMeteor:" << compactClippedMeteor
             << " driftOK:" << driftOK
             << " sweepRejected:" << sweepRejected
             << " spectralEvidenceOK:" << spectralEvidenceOK
             << " stableLineOK:" << stableLineOK
             << " boundedBandOK:" << boundedBandOK
             << " veryShortLineOK:" << veryShortLineOK
             << " shortStandaloneLine:" << shortStandaloneLine
             << " strongShortLine:" << strongShortLine
             << " durationS:" << durationS
             << " peakPowerDB:" << 10.0 * std::log10(std::max(m_pulsePeakPower, 1e-20))
             << " backgroundPowerDB:" << 10.0 * std::log10(std::max(m_noiseFloor, 1e-20))
             << " peakAboveBackgroundDB:" << peakAboveBackgroundDB
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

    const bool usePulseEnvelopeForSpectralReports = !forceRejected
        && durationOK
        && driftOK
        && (durationS >= 0.5);

    finishPendingSpectralReportsForPulse(endSample, usePulseEnvelopeForSpectralReports);

    if (accepted)
    {
        PulseReport report;

        if (validatesPendingBroadPulse)
        {
            report = m_pendingBroadPulse;
            report.m_centerFrequency = centerFrequency;
            report.m_frequencySpan = std::min(stableFrequencyLimit, std::max({std::fabs(report.m_frequencySpan), reportedFrequencySpan, compactFrequencyLimit * 3.0}));
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
            report.m_frequencySpan = reportedFrequencySpan;
            report.m_frequencyDrift = frequencyDrift;
        }

        if (!isDuplicateDetection(report.m_startSample, report.m_endSample, report.m_centerFrequency, report.m_frequencySpan)) {
            emitDetectionReport(report, "power");
        } else {
            qDebug() << "MeteorDemodSink::finishPulse: duplicate power-gate detection suppressed";
        }

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
        m_pendingBroadPulse.m_frequencySpan = reportedFrequencySpan;
        m_pendingBroadPulse.m_frequencyDrift = frequencyDrift;
    }

    m_pulseActive = false;
    m_pulseSamples.clear();
}

bool MeteorDemodSink::isDuplicateDetection(quint64 startSample, quint64 endSample) const
{
    return isDuplicateDetection(startSample, endSample, 0.0, 0.0);
}

bool MeteorDemodSink::isDuplicateDetection(quint64 startSample, quint64 endSample, double centerFrequency, double frequencySpan) const
{
    if (endSample < startSample) {
        return false;
    }

    const quint64 detectionLength = endSample - startSample + 1;
    const bool hasFrequencyRange = frequencySpan > 0.0;
    const double halfSpan = 0.5 * std::max(0.0, frequencySpan);
    const double lowFrequency = centerFrequency - halfSpan;
    const double highFrequency = centerFrequency + halfSpan;
    const double binWidth = m_spectralFrameSize > 0
        ? (double) m_settings.m_channelSampleRate / (double) m_spectralFrameSize
        : 0.0;
    const double frequencyPadding = std::max(4.0, binWidth);
    const quint64 sameEventGapSamples = (quint64) std::max(1, m_settings.m_channelSampleRate) * 5;
    const quint64 shortFragmentSamples = (quint64) std::max(1, m_settings.m_channelSampleRate) / 5;

    for (const DetectionRange& range : m_recentDetectionRanges)
    {
        const quint64 rangeLength = range.m_endSample - range.m_startSample + 1;
        const bool overlapsInTime = (endSample >= range.m_startSample) && (startSample <= range.m_endSample);

        if (overlapsInTime)
        {
            const quint64 overlapStart = std::max(startSample, range.m_startSample);
            const quint64 overlapEnd = std::min(endSample, range.m_endSample);
            const quint64 overlapLength = overlapEnd - overlapStart + 1;
            const quint64 shorterLength = std::max<quint64>(1, std::min(detectionLength, rangeLength));

            if ((double) overlapLength >= 0.50 * (double) shorterLength) {
                return true;
            }
        }

        if (!hasFrequencyRange || (range.m_highFrequency <= range.m_lowFrequency)) {
            continue;
        }

        const quint64 gapSamples = endSample < range.m_startSample
            ? range.m_startSample - endSample
            : (startSample > range.m_endSample ? startSample - range.m_endSample : 0);
        const bool shortFragmentPair = (detectionLength <= shortFragmentSamples) && (rangeLength <= shortFragmentSamples);
        const bool closeInTime = shortFragmentPair && (gapSamples <= sameEventGapSamples);
        const bool overlapsInFrequency = (highFrequency + frequencyPadding >= range.m_lowFrequency)
            && (lowFrequency - frequencyPadding <= range.m_highFrequency);

        if (closeInTime && overlapsInFrequency)
        {
            qDebug() << "MeteorDemodSink::isDuplicateDetection: close spectral duplicate suppressed"
                     << " gapSamples:" << gapSamples
                     << " startSample:" << startSample
                     << " endSample:" << endSample
                     << " frequencyLow:" << lowFrequency
                     << " frequencyHigh:" << highFrequency
                     << " previousStart:" << range.m_startSample
                     << " previousEnd:" << range.m_endSample
                     << " previousFrequencyLow:" << range.m_lowFrequency
                     << " previousFrequencyHigh:" << range.m_highFrequency;
            return true;
        }
    }

    return false;
}

void MeteorDemodSink::rememberDetection(quint64 startSample, quint64 endSample)
{
    rememberDetection(startSample, endSample, 0.0, 0.0);
}

void MeteorDemodSink::rememberDetection(quint64 startSample, quint64 endSample, double centerFrequency, double frequencySpan)
{
    if (endSample < startSample) {
        return;
    }

    const double halfSpan = 0.5 * std::max(0.0, frequencySpan);
    m_recentDetectionRanges.push_back({startSample, endSample, centerFrequency - halfSpan, centerFrequency + halfSpan});
    pruneRecentDetections();
}

void MeteorDemodSink::pruneRecentDetections()
{
    if (m_recentDetectionRanges.empty()) {
        return;
    }

    const quint64 keepSamples = (quint64) std::max(1, m_settings.m_channelSampleRate) * 180;
    const quint64 oldestSample = m_sampleCounter > keepSamples ? m_sampleCounter - keepSamples : 0;

    m_recentDetectionRanges.erase(
        std::remove_if(
            m_recentDetectionRanges.begin(),
            m_recentDetectionRanges.end(),
            [oldestSample](const DetectionRange& range) { return range.m_endSample < oldestSample; }),
        m_recentDetectionRanges.end()
    );
}

void MeteorDemodSink::emitOrDeferSpectralReport(const PulseReport& report)
{
    if (!report.m_valid) {
        return;
    }

    if (m_pulseActive)
    {
        const quint64 pulseEndSample = std::max(m_pulseLastAboveSample, m_pulseStartSample);
        const int holdSamples = std::max(2, m_settings.m_channelSampleRate / 10);
        const quint64 guardedPulseEndSample = pulseEndSample + (quint64) holdSamples;

        if (reportsOverlap(report.m_startSample, report.m_endSample, m_pulseStartSample, guardedPulseEndSample))
        {
            m_pendingSpectralReports.push_back(report);
            return;
        }
    }

    emitDetectionReport(report, "spectral");
}

void MeteorDemodSink::finishPendingSpectralReportsForPulse(quint64 pulseEndSample, bool usePulseEnvelope)
{
    if (m_pendingSpectralReports.empty()) {
        return;
    }

    std::vector<PulseReport> remainingReports;

    for (PulseReport report : m_pendingSpectralReports)
    {
        if (!reportsOverlap(report.m_startSample, report.m_endSample, m_pulseStartSample, pulseEndSample))
        {
            remainingReports.push_back(report);
            continue;
        }

        if (usePulseEnvelope)
        {
            PulseReport extendedReport = report;

            if (estimatePulseBandEnvelope(extendedReport)) {
                report = extendedReport;
            }
        }

        if (!isDuplicateDetection(report.m_startSample, report.m_endSample, report.m_centerFrequency, report.m_frequencySpan)) {
            emitDetectionReport(report, "spectral");
        } else {
            qDebug() << "MeteorDemodSink::finishPendingSpectralReportsForPulse: duplicate pending spectral detection suppressed";
        }
    }

    m_pendingSpectralReports = remainingReports;
}

bool MeteorDemodSink::estimatePulseBandEnvelope(PulseReport& report) const
{
    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);
    int frameSize = std::clamp(sampleRate / 4, 64, 256);

    if ((frameSize % 2) != 0) {
        frameSize++;
    }

    if ((int) m_pulseSamples.size() < frameSize) {
        return false;
    }

    const int hopSize = std::max(1, frameSize / 8);
    const double binWidth = (double) sampleRate / (double) frameSize;
    const double halfBandwidth = std::max({0.5 * std::fabs(report.m_frequencySpan), 20.0, 2.0 * binWidth});
    const double lowFrequency = report.m_centerFrequency - halfBandwidth;
    const double highFrequency = report.m_centerFrequency + halfBandwidth;
    const double twoPi = 2.0 * std::acos(-1.0);
    std::vector<double> strengths;
    std::vector<quint64> centerSamples;

    for (int start = 0; (start + frameSize) <= (int) m_pulseSamples.size(); start += hopSize)
    {
        double bandPower = 0.0;

        for (int bin = -frameSize / 2; bin < frameSize / 2; bin++)
        {
            const double frequency = (double) bin * binWidth;

            if ((frequency < lowFrequency) || (frequency > highFrequency)) {
                continue;
            }

            double realSum = 0.0;
            double imagSum = 0.0;

            for (int i = 0; i < frameSize; i++)
            {
                const double window = 0.5 - 0.5 * std::cos(twoPi * (double) i / (double) (frameSize - 1));
                const Complex windowedSample = m_pulseSamples[start + i] * (Real) window;
                const double angle = -twoPi * (double) bin * (double) i / (double) frameSize;
                const double c = std::cos(angle);
                const double s = std::sin(angle);

                realSum += windowedSample.real() * c - windowedSample.imag() * s;
                imagSum += windowedSample.real() * s + windowedSample.imag() * c;
            }

            bandPower += realSum*realSum + imagSum*imagSum;
        }

        strengths.push_back(std::max(bandPower, 1e-30));
        centerSamples.push_back(m_pulseStartSample + (quint64) start + (quint64) (frameSize / 2));
    }

    if (strengths.empty()) {
        return false;
    }

    std::vector<double> sortedStrengths = strengths;
    std::sort(sortedStrengths.begin(), sortedStrengths.end());
    const double floorStrength = std::max(sortedStrengths[sortedStrengths.size() / 5], 1e-30);
    const quint64 reportCenterSample = report.m_startSample + (report.m_endSample - report.m_startSample) / 2;
    const quint64 searchRadiusSamples = (quint64) sampleRate * 2;
    int peakIndex = -1;
    double peakStrength = 0.0;

    for (int i = 0; i < (int) strengths.size(); i++)
    {
        const quint64 centerSample = centerSamples[i];
        const quint64 distance = centerSample > reportCenterSample
            ? centerSample - reportCenterSample
            : reportCenterSample - centerSample;

        if ((distance <= searchRadiusSamples) && (strengths[i] > peakStrength))
        {
            peakStrength = strengths[i];
            peakIndex = i;
        }
    }

    if ((peakIndex < 0) || (peakStrength <= floorStrength)) {
        return false;
    }

    const double threshold = floorStrength + 0.03 * (peakStrength - floorStrength);
    const int maxGapFrames = std::max(2, sampleRate / (hopSize * 10));
    int firstIndex = peakIndex;
    int lastIndex = peakIndex;
    int gapFrames = 0;

    for (int i = peakIndex - 1; i >= 0; i--)
    {
        if (strengths[i] >= threshold)
        {
            firstIndex = i;
            gapFrames = 0;
        }
        else if (++gapFrames > maxGapFrames)
        {
            break;
        }
    }

    gapFrames = 0;

    for (int i = peakIndex + 1; i < (int) strengths.size(); i++)
    {
        if (strengths[i] >= threshold)
        {
            lastIndex = i;
            gapFrames = 0;
        }
        else if (++gapFrames > maxGapFrames)
        {
            break;
        }
    }

    const quint64 startSample = centerSamples[firstIndex] > (quint64) (frameSize / 2)
        ? centerSamples[firstIndex] - (quint64) (frameSize / 2)
        : 0;
    const quint64 endSample = centerSamples[lastIndex] + (quint64) (frameSize / 2);
    const double durationS = (double) (endSample - startSample + 1) / (double) sampleRate;

    if ((endSample <= startSample)
        || (durationS <= report.m_durationS * 1.5)
        || (durationS > (double) m_settings.m_maxDurationMS / 1000.0))
    {
        return false;
    }

    report.m_dateTimeUtc = sampleCounterToDateTimeUtc(startSample);
    report.m_startSample = startSample;
    report.m_endSample = endSample;
    report.m_peakPower = std::max(report.m_peakPower, m_pulsePeakPower);
    report.m_backgroundPower = m_noiseFloor;
    report.m_durationS = durationS;
    return true;
}

bool MeteorDemodSink::reportsOverlap(
    quint64 firstStartSample,
    quint64 firstEndSample,
    quint64 secondStartSample,
    quint64 secondEndSample) const
{
    return (firstEndSample >= secondStartSample) && (firstStartSample <= secondEndSample);
}

void MeteorDemodSink::emitDetectionReport(const PulseReport& report, const char *source)
{
    if (!report.m_valid || !m_messageQueueToGUI) {
        return;
    }

    const double peakAmplitude = std::sqrt(std::max(report.m_peakPower, 0.0));
    const double peakPowerDB = 10.0 * std::log10(std::max(report.m_peakPower, 1e-20));
    const double backgroundPowerDB = 10.0 * std::log10(std::max(report.m_backgroundPower, 1e-20));

    qDebug() << "MeteorDemodSink::emitDetectionReport:"
             << " source:" << source
             << " dateTimeUtc:" << report.m_dateTimeUtc
             << " durationS:" << report.m_durationS
             << " peakPowerDB:" << peakPowerDB
             << " backgroundPowerDB:" << backgroundPowerDB
             << " centerFrequency:" << report.m_centerFrequency
             << " frequencySpan:" << report.m_frequencySpan
             << " frequencyDrift:" << report.m_frequencyDrift
             << " startSample:" << report.m_startSample
             << " endSample:" << report.m_endSample;

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

    rememberDetection(report.m_startSample, report.m_endSample, report.m_centerFrequency, report.m_frequencySpan);
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
    const double slowAlpha = std::min(1.0, 1.0 / ((double) m_settings.m_channelSampleRate * timeConstantS));
    const double alpha = m_noiseFloor < (power * 0.01) ? 0.05 : slowAlpha;
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

void MeteorDemodSink::configureSpectralDetector()
{
    m_spectralFrameSize = std::clamp(m_settings.m_channelSampleRate / 8, 32, 256);
    if ((m_spectralFrameSize % 2) != 0) {
        m_spectralFrameSize++;
    }
    m_spectralHopSize = std::max(1, m_spectralFrameSize / 4);
    m_spectralFrameBuffer.clear();
    m_spectralNoiseFloor.clear();
    m_spectralEvents.clear();
    m_pendingSpectralReports.clear();
    m_spectralNoiseFloorInitialized = false;
    m_spectralEventActiveForScope = false;
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
    m_spectralFrameBuffer.clear();
    m_spectralNoiseFloor.clear();
    m_spectralEvents.clear();
    m_recentDetectionRanges.clear();
    m_pendingSpectralReports.clear();
    m_spectralNoiseFloorInitialized = false;
    m_spectralEventActiveForScope = false;
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
        configureSpectralDetector();
        resizeScopeBuffers();
        resetDetector();
    }

    if (sampleRateChanged || lpfChanged) {
        configurePowerLowpass();
    }
}
