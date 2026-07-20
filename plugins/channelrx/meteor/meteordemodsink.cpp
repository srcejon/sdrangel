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
#include <utility>

#include <QDebug>

#include "dsp/fftengine.h"
#include "dsp/spectrumvis.h"
#include "util/messagequeue.h"
#include "maincore.h"

#include "meteordemodsink.h"

MESSAGE_CLASS_DEFINITION(MeteorDemodSink::MsgMeteorDetected, Message)
MESSAGE_CLASS_DEFINITION(MeteorDemodSink::MsgMeteorDataCollected, Message)

namespace {
    FFTEngine *createMeteorFFT()
    {
        FFTEngine *fft = FFTEngine::create(QString(), QStringLiteral("Kiss"));

        if (!fft) {
            qFatal("MeteorDemodSink: no FFT engine is available");
        }

        return fft;
    }

    void makeHannWindow(std::vector<Real>& window, int size)
    {
        window.resize(size);

        if (size <= 1)
        {
            if (size == 1) {
                window[0] = 1.0f;
            }

            return;
        }

        const double twoPi = 2.0 * std::acos(-1.0);

        for (int i = 0; i < size; i++) {
            window[i] = (Real) (0.5 - 0.5 * std::cos(twoPi * (double) i / (double) (size - 1)));
        }
    }

    Real powerToDB(double power)
    {
        return (Real) (10.0 * std::log10(std::max(power, 1e-20)));
    }

    double medianPower(const std::vector<double>& power)
    {
        if (power.empty()) {
            return 1e-30;
        }

        std::vector<double> values = power;
        const auto middle = values.begin() + values.size() / 2;
        std::nth_element(values.begin(), middle, values.end());
        return std::max(*middle, 1e-30);
    }

    bool solve3x3(double matrix[3][4], double solution[3])
    {
        for (int column = 0; column < 3; column++)
        {
            int pivot = column;

            for (int row = column + 1; row < 3; row++)
            {
                if (std::fabs(matrix[row][column]) > std::fabs(matrix[pivot][column])) {
                    pivot = row;
                }
            }

            if (std::fabs(matrix[pivot][column]) < 1e-12) {
                return false;
            }

            if (pivot != column)
            {
                for (int j = column; j < 4; j++) {
                    std::swap(matrix[column][j], matrix[pivot][j]);
                }
            }

            const double divisor = matrix[column][column];

            for (int j = column; j < 4; j++) {
                matrix[column][j] /= divisor;
            }

            for (int row = 0; row < 3; row++)
            {
                if (row == column) {
                    continue;
                }

                const double factor = matrix[row][column];

                for (int j = column; j < 4; j++) {
                    matrix[row][j] -= factor * matrix[column][j];
                }
            }
        }

        for (int i = 0; i < 3; i++) {
            solution[i] = matrix[i][3];
        }

        return true;
    }
}

MeteorDemodSink::MeteorDemodSink() :
    m_spectrumSink(nullptr),
    m_secondarySpectrumSink(nullptr),
    m_messageQueueToGUI(nullptr),
    m_channelSampleRate(48000),
    m_channelFrequencyOffset(0),
    m_interpolatorDistance(48.0f),
    m_interpolatorDistanceRemain(0.0f),
    m_spectrumBufferSize(0),
    m_nextMeteorEventId(1),
    m_nextComponentFlushSample(std::numeric_limits<quint64>::max()),
    m_spectralFFT(nullptr),
    m_pulseFFT(nullptr),
    m_spectralFrameSize(0),
    m_spectralHopSize(0),
    m_spectralFFTSize(0),
    m_pulseFFTSize(0),
    m_spectralEnergyScale(1.0),
    m_detectionSampleRingStart(0),
    m_detectionSampleRingCount(0),
    m_detectionSampleRingStartSample(0),
    m_spectralNoiseFloorInitialized(false),
    m_nextDisplayTimeAnchorSample(0),
    m_nextDataMarkerSample(0)
{
    resizeSpectrumBuffer();
    resetDetector();
    applySettings(m_settings, QStringList(), true);
    applyChannelSettings(m_channelSampleRate, m_channelFrequencyOffset, true);
}

MeteorDemodSink::~MeteorDemodSink()
{
    delete m_spectralFFT;
    delete m_pulseFFT;
}

void MeteorDemodSink::setDetectorTunables(const DetectorTunables& tunables)
{
    m_detectorTunables = tunables;
    resolveDetectorTunables();
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
    if (!m_pulseActive && m_spectralEvents.empty() && m_activeMeteorEvents.empty()) {
        return false;
    }

    Complex zero(0.0f, 0.0f);
    const int tailSamples = std::max(
        m_settings.m_channelSampleRate / 2,
        m_spectralFrameSize + m_spectralHopSize * 8 + m_settings.m_channelSampleRate / 20 + 4
    );

    for (int i = 0; i < tailSamples && (m_pulseActive || !m_spectralEvents.empty()); i++) {
        processOneSample(zero, false);
    }

    flushPendingComponentReports(true);
    return true;
}

void MeteorDemodSink::processOneSample(Complex& ci, bool realSample)
{
    recordDisplayTimeAnchor();

    if (realSample) {
        emitDataCollectionMarker();
    }

    const Real re = ci.real() / SDR_RX_SCALEF;
    const Real im = ci.imag() / SDR_RX_SCALEF;
    const Complex normalized(re, im);
    const double power = std::max<double>(re*re + im*im, 1e-20);
    const double filteredPower = std::max<double>(m_powerLowpass.filter((Real) power), 0.0);

    appendDetectionSample(normalized);
    processDetectorSample(normalized, power, filteredPower);
    processSpectralSample(normalized);
    flushPendingComponentReports(false);
    feedSpectrum(normalized);
    m_sampleCounter++;
}

void MeteorDemodSink::processDetectorSample(const Complex& sample, double power, double filteredPower)
{
    if (m_rearmNeeded)
    {
        updateNoiseFloor(filteredPower);
        const double releaseThreshold = getDetectionThresholdPower() * 0.7;

        if (filteredPower < releaseThreshold) {
            m_rearmNeeded = false;
        }

        return;
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
            return;
        }

        return;
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
        return;
    }

    if ((int) (m_sampleCounter - m_pulseLastAboveSample) > holdSamples)
    {
        finishPulse(false);
        return;
    }
}

void MeteorDemodSink::startPulse(const Complex& sample, double power)
{
    m_pulseActive = true;
    m_pulseStartSample = m_sampleCounter;
    m_pulseLastAboveSample = m_sampleCounter;
    m_pulsePeakSample = m_sampleCounter;
    m_pulseStartDateTimeUtc = sampleCounterToDateTimeUtc(m_pulseStartSample);
    m_pulsePeakPower = power;
    m_pulseSamples.clear();
    m_pulseSamples.push_back(sample);
}

void MeteorDemodSink::updatePulse(const Complex& sample, double power)
{
    if (power > m_pulsePeakPower) {
        m_pulsePeakPower = power;
        m_pulsePeakSample = m_sampleCounter;
    }

    const int maxSamples = std::max(1, (m_settings.m_maxDurationMS * m_settings.m_channelSampleRate) / 1000);

    if ((int) m_pulseSamples.size() < maxSamples + std::max(2, m_settings.m_channelSampleRate / 20)) {
        m_pulseSamples.push_back(sample);
    }
}

void MeteorDemodSink::processSpectralSample(const Complex& sample)
{
    if ((m_spectralFrameSize < 8) || (m_spectralHopSize < 1)) {
        return;
    }

    m_spectralFrameBuffer.push_back(sample);

    if ((int) m_spectralFrameBuffer.size() >= m_spectralFrameSize)
    {
        const quint64 frameStartSample = m_sampleCounter + 1 - (quint64) m_spectralFrameBuffer.size();
        processSpectralFrame(frameStartSample);

        const int removeCount = std::min(m_spectralHopSize, (int) m_spectralFrameBuffer.size());
        m_spectralFrameBuffer.erase(m_spectralFrameBuffer.begin(), m_spectralFrameBuffer.begin() + removeCount);
    }

}

void MeteorDemodSink::processSpectralFrame(quint64 frameStartSample)
{
    if ((int) m_spectralFrameBuffer.size() < m_spectralFrameSize) {
        return;
    }

    if ((int) m_spectralBinPower.size() != m_spectralFrameSize) {
        m_spectralBinPower.assign(m_spectralFrameSize, 0.0);
    } else {
        std::fill(m_spectralBinPower.begin(), m_spectralBinPower.end(), 0.0);
    }

    Complex *fftIn = m_spectralFFT->in();

    for (int i = 0; i < m_spectralFrameSize; i++) {
        fftIn[i] = m_spectralFrameBuffer[i] * m_spectralWindow[i];
    }

    m_spectralFFT->transform();
    const Complex *fftOut = m_spectralFFT->out();
    const int halfSize = m_spectralFrameSize / 2;

    for (int bin = -halfSize; bin < halfSize; bin++)
    {
        const int fftIndex = bin < 0 ? bin + m_spectralFrameSize : bin;
        m_spectralBinPower[bin + halfSize] = std::max((double) std::norm(fftOut[fftIndex]), 1e-30);
    }

    SpectralFrameSnapshot frame;
    frame.m_startSample = frameStartSample;
    frame.m_binPower = m_spectralBinPower;

    if (!m_spectralNoiseFloorInitialized)
    {
        m_spectralCalibrationFrames.push_back(std::move(frame));
        const quint64 frameCenterSample = frameStartSample + (quint64) (m_spectralFrameSize / 2);
        const quint64 calibrationSamples = (quint64) std::max(
            m_spectralFrameSize + 4 * m_spectralHopSize,
            2 * m_settings.m_channelSampleRate);

        if ((frameCenterSample < calibrationSamples) || (m_spectralCalibrationFrames.size() < 5)) {
            return;
        }

        initializeSpectralNoiseFloor();
        std::vector<SpectralFrameSnapshot> calibrationFrames;
        calibrationFrames.swap(m_spectralCalibrationFrames);

        for (const SpectralFrameSnapshot& calibrationFrame : calibrationFrames) {
            processCalibratedSpectralFrame(calibrationFrame);
        }

        return;
    }

    processCalibratedSpectralFrame(frame);
}

void MeteorDemodSink::initializeSpectralNoiseFloor()
{
    if (m_spectralCalibrationFrames.empty()) {
        return;
    }

    const int binCount = (int) m_spectralCalibrationFrames.front().m_binPower.size();
    m_spectralNoiseFloor.assign(binCount, 1e-30);
    std::vector<double> values;
    values.reserve(m_spectralCalibrationFrames.size());

    for (int bin = 0; bin < binCount; bin++)
    {
        values.clear();

        for (const SpectralFrameSnapshot& frame : m_spectralCalibrationFrames)
        {
            if (bin < (int) frame.m_binPower.size()) {
                values.push_back(frame.m_binPower[bin]);
            }
        }

        if (values.empty()) {
            continue;
        }

        const int quantileIndex = std::clamp((int) (0.50 * (double) (values.size() - 1)), 0, (int) values.size() - 1);
        std::nth_element(values.begin(), values.begin() + quantileIndex, values.end());
        m_spectralNoiseFloor[bin] = std::max(values[quantileIndex], 1e-30);
    }

    m_spectralMinimumNoiseFloor = m_spectralNoiseFloor;
    m_spectralNoiseFloorInitialized = true;
}

void MeteorDemodSink::updateMinimumStatisticsNoiseFloor(const SpectralFrameSnapshot& frame)
{
    if (frame.m_binPower.empty()) {
        return;
    }

    m_minimumNoiseCurrentBlock.push_back(frame);

    if ((int) m_minimumNoiseCurrentBlock.size()
        < m_resolvedDetectorTunables.m_minimumNoiseFramesPerBlock)
    {
        return;
    }

    const int binCount = (int) frame.m_binPower.size();
    std::vector<double> blockFloor(binCount, 1e-30);
    std::vector<double> values;
    values.reserve(m_minimumNoiseCurrentBlock.size());

    for (int bin = 0; bin < binCount; bin++)
    {
        values.clear();

        for (const SpectralFrameSnapshot& blockFrame : m_minimumNoiseCurrentBlock)
        {
            if (bin < (int) blockFrame.m_binPower.size()) {
                values.push_back(blockFrame.m_binPower[bin]);
            }
        }

        if (values.empty()) {
            continue;
        }

        const int quantileIndex = std::clamp(
            (int) std::llround(
                std::clamp(m_detectorTunables.m_minimumNoiseBlockQuantile, 0.0, 1.0)
                    * (double) (values.size() - 1)),
            0,
            (int) values.size() - 1);
        std::nth_element(values.begin(), values.begin() + quantileIndex, values.end());
        blockFloor[bin] = std::max(values[quantileIndex], 1e-30);
    }

    m_minimumNoiseCurrentBlock.clear();
    m_minimumNoiseBlocks.push_back(std::move(blockFloor));

    while ((int) m_minimumNoiseBlocks.size() > std::max(1, m_detectorTunables.m_minimumNoiseBlockCount)) {
        m_minimumNoiseBlocks.erase(m_minimumNoiseBlocks.begin());
    }

    m_spectralMinimumNoiseFloor.assign(binCount, 1e-30);

    for (int bin = 0; bin < binCount; bin++)
    {
        double minimum = std::numeric_limits<double>::max();

        for (const std::vector<double>& block : m_minimumNoiseBlocks)
        {
            if (bin < (int) block.size()) {
                minimum = std::min(minimum, block[bin]);
            }
        }

        if (minimum < std::numeric_limits<double>::max()) {
            m_spectralMinimumNoiseFloor[bin] = std::max(minimum, 1e-30);
        }
    }
}

void MeteorDemodSink::processCalibratedSpectralFrame(const SpectralFrameSnapshot& frame)
{
    if (!m_spectralNoiseFloorInitialized || (m_spectralNoiseFloor.size() != frame.m_binPower.size())) {
        return;
    }

    std::vector<SpectralBand> bands = detectSpectralBands(frame.m_binPower);
    updateMinimumStatisticsNoiseFloor(frame);

    if (m_spectralActiveBins.size() != m_spectralBinPower.size()) {
        m_spectralActiveBins.assign(m_spectralBinPower.size(), 0);
    } else {
        std::fill(m_spectralActiveBins.begin(), m_spectralActiveBins.end(), 0);
    }

    for (const SpectralBand& band : bands)
    {
        for (int i = std::max(0, band.m_lowIndex); i <= std::min((int) m_spectralActiveBins.size() - 1, band.m_highIndex); i++) {
            m_spectralActiveBins[i] = 1;
        }
    }

    protectActiveMeteorNoiseBins();

    for (int i = 0; i < (int) frame.m_binPower.size(); i++)
    {
        const double alpha = m_spectralActiveBins[i]
            ? m_detectorTunables.m_spectralActiveNoiseAlpha
            : (m_spectralNoiseFloor[i] < (frame.m_binPower[i] * 0.01)
                ? m_detectorTunables.m_spectralRisingNoiseAlpha
                : m_detectorTunables.m_spectralStableNoiseAlpha);
        m_spectralNoiseFloor[i] = (1.0 - alpha) * std::max(m_spectralNoiseFloor[i], 1e-30) + alpha * frame.m_binPower[i];
    }

    const quint64 frameCenterSample = frame.m_startSample + (quint64) (m_spectralFrameSize / 2);
    updateSpectralEvents(bands, frameCenterSample);
    updateActiveMeteorEvents(bands, frameCenterSample);
}

std::vector<MeteorDemodSink::SpectralBand> MeteorDemodSink::detectSpectralBands(const std::vector<double>& binPower)
{
    std::vector<SpectralBand> bands;

    if (binPower.empty() || (m_spectralNoiseFloor.size() != binPower.size())) {
        return bands;
    }

    const double medianFramePower = medianPower(binPower);
    const double minimumBinNoise = medianFramePower * 0.25;
    const double seedThresholdDB = std::max(10.0, (double) m_settings.m_detectionThresholdDB);
    const double growThresholdDB = std::max(3.0, seedThresholdDB - 5.0);
    const double thresholdRatio = std::pow(10.0, seedThresholdDB / 10.0);
    const double spanRatio = std::pow(10.0, growThresholdDB / 10.0);
    const double continuationThresholdRatio = std::pow(
        10.0,
        std::max(3.0, seedThresholdDB - m_detectorTunables.m_continuationThresholdReductionDB) / 10.0);
    const double continuationSpanRatio = std::pow(
        10.0,
        std::max(2.0, growThresholdDB - m_detectorTunables.m_continuationThresholdReductionDB) / 10.0);
    const double binWidth = (double) m_settings.m_channelSampleRate / (double) m_spectralFrameSize;
    const double maxBandWidth = m_resolvedDetectorTunables.m_maxSegmentedBandWidthHz;
    const int edgeBins = std::max(
        0,
        (int) std::ceil(m_detectorTunables.m_edgeExclusionFraction * (double) binPower.size()));
    const int usableLowIndex = std::min(edgeBins, (int) binPower.size() - 1);
    const int usableHighIndex = std::max(usableLowIndex, (int) binPower.size() - edgeBins - 1);
    int occupiedBins = 0;

    for (int i = usableLowIndex; i <= usableHighIndex; i++)
    {
        const double noise = std::max({m_spectralNoiseFloor[i], minimumBinNoise, 1e-30});

        if ((binPower[i] / noise) >= spanRatio) {
            occupiedBins++;
        }
    }

    const double frameOccupiedFraction = (double) occupiedBins
        / (double) std::max(1, usableHighIndex - usableLowIndex + 1);

    int minAllowedIndex = usableLowIndex;

    for (int i = usableLowIndex; i <= usableHighIndex; i++)
    {
        const double ratio = binPower[i] / std::max({m_spectralNoiseFloor[i], minimumBinNoise, 1e-30});
        const double frequency = ((double) i - (double) m_spectralFrameSize / 2.0) * binWidth;
        const bool continuationBin = frequencyProtectedByActiveMeteor(frequency);
        const double localThresholdRatio = continuationBin ? continuationThresholdRatio : thresholdRatio;

        if (ratio < localThresholdRatio) {
            continue;
        }

        int lowIndex = i;
        int highIndex = i;

        while (lowIndex > minAllowedIndex)
        {
            const double lowFrequency = ((double) (lowIndex - 1) - (double) m_spectralFrameSize / 2.0) * binWidth;
            const double lowSpanRatio = frequencyProtectedByActiveMeteor(lowFrequency)
                ? continuationSpanRatio
                : spanRatio;

            if ((binPower[lowIndex - 1]
                / std::max({m_spectralNoiseFloor[lowIndex - 1], minimumBinNoise, 1e-30})) < lowSpanRatio)
            {
                break;
            }

            lowIndex--;
        }

        while (highIndex < usableHighIndex)
        {
            const double highFrequency = ((double) (highIndex + 1) - (double) m_spectralFrameSize / 2.0) * binWidth;
            const double highSpanRatio = frequencyProtectedByActiveMeteor(highFrequency)
                ? continuationSpanRatio
                : spanRatio;

            if ((binPower[highIndex + 1]
                / std::max({m_spectralNoiseFloor[highIndex + 1], minimumBinNoise, 1e-30})) < highSpanRatio)
            {
                break;
            }

            highIndex++;
        }

        SpectralBand band;
        double weightedFrequencySum = 0.0;
        double weightSum = 0.0;
        double peakRatio = 0.0;
        double peakPower = 0.0;
        double peakNoise = 1e-30;
        double peakFrequency = 0.0;
        double backgroundBinPower = 0.0;
        double minimumBackgroundBinPower = 0.0;
        double minimumPeakNoise = 1e-30;

        for (int j = lowIndex; j <= highIndex; j++)
        {
            const double noise = std::max({m_spectralNoiseFloor[j], minimumBinNoise, 1e-30});
            const double excessPower = std::max(0.0, binPower[j] - noise);
            const double frequency = ((double) j - (double) m_spectralFrameSize / 2.0) * binWidth;
            const double binRatio = binPower[j] / noise;

            backgroundBinPower += noise;
            const double minimumNoise = (j < (int) m_spectralMinimumNoiseFloor.size())
                ? std::max(m_spectralMinimumNoiseFloor[j], 1e-30)
                : noise;
            minimumBackgroundBinPower += minimumNoise;

            weightedFrequencySum += frequency * excessPower;
            weightSum += excessPower;

            if (binPower[j] > peakPower)
            {
                peakPower = binPower[j];
                peakNoise = noise;
                minimumPeakNoise = minimumNoise;
                peakFrequency = frequency;
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
        band.m_peakFrequency = peakFrequency;
        band.m_peakBinPower = peakPower;
        band.m_totalExcessPower = weightSum;
        band.m_contrastDB = 10.0 * std::log10(std::max(peakPower, 1e-30) / std::max(peakNoise, 1e-30));
        band.m_peakRatio = peakRatio;
        band.m_framePeakPower = (weightSum + backgroundBinPower) * m_spectralEnergyScale
            / (double) std::max(1, m_spectralHopSize);
        band.m_frameOccupiedFraction = frameOccupiedFraction;
        band.m_backgroundPower = backgroundBinPower * m_spectralEnergyScale
            / (double) std::max(1, m_spectralHopSize);
        band.m_minimumNoiseContrastDB = 10.0 * std::log10(
            std::max(peakPower, 1e-30) / std::max(minimumPeakNoise, 1e-30));
        band.m_noiseFloorDeltaDB = 10.0 * std::log10(
            std::max(backgroundBinPower, 1e-30) / std::max(minimumBackgroundBinPower, 1e-30));

        if ((band.m_bandwidth <= maxBandWidth) && (band.m_totalExcessPower > 0.0)) {
            bands.push_back(band);
            minAllowedIndex = highIndex + 1;
        }

        i = highIndex;
    }

    return bands;
}

bool MeteorDemodSink::frequencyProtectedByActiveMeteor(double frequency) const
{
    const double binWidth = (double) std::max(1, m_settings.m_channelSampleRate)
        / (double) std::max(1, m_spectralFrameSize);
    const double padding = m_resolvedDetectorTunables.m_continuationFrequencyPaddingHz;

    for (const ActiveMeteorEvent& event : m_activeMeteorEvents)
    {
        const PulseReport& report = event.m_report;

        if (!report.m_spectralParentEligible) {
            continue;
        }

        if ((event.m_spectralComponentCount < 2) && (report.m_durationS < 1.0)) {
            continue;
        }

        const double center = report.m_hasRobustFrequency
            ? report.m_robustCenterFrequency
            : report.m_centerFrequency;
        const double reportSpan = std::max({
            std::fabs(report.m_robustFrequencySpan),
            std::fabs(report.m_duplicateFrequencySpan),
            std::fabs(report.m_frequencySpan),
            binWidth});
        const double span = std::min(
            reportSpan,
            std::max(24.0, m_resolvedDetectorTunables.m_compactBandwidthHz));

        if ((frequency >= center - 0.5 * span - padding)
            && (frequency <= center + 0.5 * span + padding))
        {
            return true;
        }
    }

    return false;
}

void MeteorDemodSink::protectActiveMeteorNoiseBins()
{
    if (m_spectralActiveBins.empty()) {
        return;
    }

    const double binWidth = (double) std::max(1, m_settings.m_channelSampleRate)
        / (double) std::max(1, m_spectralFrameSize);

    for (int i = 0; i < (int) m_spectralActiveBins.size(); i++)
    {
        const double frequency = ((double) i - (double) m_spectralFrameSize / 2.0) * binWidth;

        if (frequencyProtectedByActiveMeteor(frequency)) {
            m_spectralActiveBins[i] = 1;
        }
    }
}

bool MeteorDemodSink::bandCompatibleWithActiveMeteor(
    const ActiveMeteorEvent& event,
    const SpectralBand& band) const
{
    const PulseReport& report = event.m_report;
    const double binWidth = (double) std::max(1, m_settings.m_channelSampleRate)
        / (double) std::max(1, m_spectralFrameSize);
    const double padding = m_resolvedDetectorTunables.m_continuationFrequencyPaddingHz;
    const double center = report.m_hasRobustFrequency
        ? report.m_robustCenterFrequency
        : report.m_centerFrequency;
    const double reportSpan = std::max({
        std::fabs(report.m_robustFrequencySpan),
        std::fabs(report.m_duplicateFrequencySpan),
        std::fabs(report.m_frequencySpan),
        binWidth});
    const double span = std::min(
        reportSpan,
        std::max(24.0, m_resolvedDetectorTunables.m_compactBandwidthHz));
    const double eventLow = center - 0.5 * span;
    const double eventHigh = center + 0.5 * span;
    const double overlap = std::max(
        0.0,
        std::min(eventHigh + padding, band.m_highFrequency)
            - std::max(eventLow - padding, band.m_lowFrequency));
    const double centerDistance = std::fabs(band.m_centerFrequency - center);

    return (overlap > 0.0)
        && (centerDistance <= std::max(padding, 0.35 * span));
}

void MeteorDemodSink::addMeteorEventObservation(
    ActiveMeteorEvent& event,
    const MeteorEventObservation& observation)
{
    if ((int) event.m_observations.size() >= m_detectorTunables.m_maxParentObservations)
    {
        std::vector<MeteorEventObservation> compacted;
        compacted.reserve(event.m_observations.size() / 2 + 1);

        for (int i = 0; i < (int) event.m_observations.size(); i += 2) {
            compacted.push_back(event.m_observations[i]);
        }

        event.m_observations.swap(compacted);
    }

    event.m_observations.push_back(observation);
}

void MeteorDemodSink::updateActiveMeteorEvents(
    const std::vector<SpectralBand>& bands,
    quint64 frameCenterSample)
{
    if (m_activeMeteorEvents.empty() || bands.empty()) {
        return;
    }

    const quint64 maxEvidenceGapSamples = (quint64) std::max(
        1.0,
        m_detectorTunables.m_continuationMaxEvidenceGapS
            * (double) std::max(1, m_settings.m_channelSampleRate));
    std::vector<bool> bandMatched(bands.size(), false);

    for (ActiveMeteorEvent& event : m_activeMeteorEvents)
    {
        if (!event.m_report.m_spectralParentEligible) {
            continue;
        }

        if ((event.m_spectralComponentCount < 2) && (event.m_report.m_durationS < 1.0)) {
            continue;
        }

        if ((frameCenterSample > event.m_lastEvidenceSample)
            && ((frameCenterSample - event.m_lastEvidenceSample) > maxEvidenceGapSamples))
        {
            continue;
        }

        int bestBand = -1;
        double bestScore = std::numeric_limits<double>::max();
        const double eventCenter = event.m_report.m_hasRobustFrequency
            ? event.m_report.m_robustCenterFrequency
            : event.m_report.m_centerFrequency;

        for (int i = 0; i < (int) bands.size(); i++)
        {
            const SpectralBand& band = bands[i];
            const double maxContinuationBandwidth = std::max(
                24.0,
                m_resolvedDetectorTunables.m_compactBandwidthHz);
            const bool broadband = (band.m_frameOccupiedFraction
                    >= m_detectorTunables.m_broadbandImpulseMinOccupiedFraction)
                || (band.m_bandwidth > maxContinuationBandwidth);

            if (bandMatched[i] || broadband || !bandCompatibleWithActiveMeteor(event, band)) {
                continue;
            }

            bool supportedByTrack = false;

            for (const SpectralEvent& spectralEvent : m_spectralEvents)
            {
                if (!spectralEvent.m_valid
                    || (spectralEvent.m_lastCenterSample != frameCenterSample)
                    || (spectralEvent.m_trackFrequencies.size() < 2))
                {
                    continue;
                }

                const double trackCenter = spectralEvent.m_trackFrequencies.back();
                const double trackTolerance = std::max(
                    12.0,
                    0.5 * std::max(band.m_bandwidth, spectralEvent.m_maxBandwidth));

                if (std::fabs(trackCenter - band.m_centerFrequency) <= trackTolerance)
                {
                    supportedByTrack = true;
                    break;
                }
            }

            if (!supportedByTrack) {
                continue;
            }

            const double score = std::fabs(band.m_centerFrequency - eventCenter)
                - 0.1 * std::min(30.0, band.m_contrastDB);

            if (score < bestScore)
            {
                bestScore = score;
                bestBand = i;
            }
        }

        if (bestBand < 0) {
            continue;
        }

        const SpectralBand& band = bands[bestBand];
        bandMatched[bestBand] = true;
        const quint64 oldEndSample = event.m_report.m_endSample;
        const quint64 evidenceEndSample = frameCenterSample
            + (quint64) std::max(1, m_spectralHopSize / 2);

        event.m_lastEvidenceSample = std::max(event.m_lastEvidenceSample, frameCenterSample);
        event.m_report.m_endSample = std::max(event.m_report.m_endSample, evidenceEndSample);
        event.m_report.m_displayEndSample = event.m_report.m_endSample;
        event.m_report.m_peakPower = std::max(event.m_report.m_peakPower, band.m_framePeakPower);
        event.m_report.m_totalPower += std::max(band.m_totalExcessPower, 0.0) * m_spectralEnergyScale;
        event.m_extendedByContinuation = event.m_extendedByContinuation
            || (event.m_report.m_endSample > oldEndSample);

        MeteorEventObservation observation;
        observation.m_sample = frameCenterSample;
        observation.m_centerFrequency = band.m_centerFrequency;
        observation.m_lowFrequency = band.m_lowFrequency;
        observation.m_highFrequency = band.m_highFrequency;
        observation.m_weight = std::max(band.m_totalExcessPower, 1e-30);
        observation.m_peakPower = band.m_framePeakPower;
        observation.m_backgroundPower = band.m_backgroundPower;
        observation.m_broadband = false;
        addMeteorEventObservation(event, observation);
    }

    scheduleNextComponentFlush();
}

void MeteorDemodSink::updateSpectralEvents(const std::vector<SpectralBand>& bands, quint64 frameCenterSample)
{
    struct MatchCandidate {
        int m_eventIndex;
        int m_bandIndex;
        double m_score;
    };

    std::vector<bool> bandMatched(bands.size(), false);
    std::vector<bool> eventMatched(m_spectralEvents.size(), false);
    std::vector<MatchCandidate> matches;
    const int maxMissingFrames = std::max(2, (int) std::ceil(0.15 * (double) m_settings.m_channelSampleRate / (double) std::max(1, m_spectralHopSize)));
    const double padding = m_resolvedDetectorTunables.m_trackingFrequencyPaddingHz;
    const double maxTrackingJump = m_resolvedDetectorTunables.m_maxTrackingJumpHz;

    for (int eventIndex = 0; eventIndex < (int) m_spectralEvents.size(); eventIndex++)
    {
        const SpectralEvent& event = m_spectralEvents[eventIndex];

        if (!event.m_valid) {
            continue;
        }

        double predictedFrequency = event.m_trackFrequencies.empty()
            ? 0.5 * (event.m_minFrequency + event.m_maxFrequency)
            : event.m_trackFrequencies.back();

        if (event.m_trackFrequencies.size() >= 2)
        {
            const int last = (int) event.m_trackFrequencies.size() - 1;
            const double velocity = std::clamp(
                event.m_trackFrequencies[last] - event.m_trackFrequencies[last - 1],
                -maxTrackingJump,
                maxTrackingJump);
            predictedFrequency += velocity;
        }

        const double eventLow = event.m_trackLowFrequencies.empty()
            ? predictedFrequency - 0.5 * event.m_maxBandwidth
            : event.m_trackLowFrequencies.back();
        const double eventHigh = event.m_trackHighFrequencies.empty()
            ? predictedFrequency + 0.5 * event.m_maxBandwidth
            : event.m_trackHighFrequencies.back();

        for (int i = 0; i < (int) bands.size(); i++)
        {
            const SpectralBand& band = bands[i];
            const double overlap = std::max(
                0.0,
                std::min(eventHigh, band.m_highFrequency) - std::max(eventLow, band.m_lowFrequency));
            const double frequencyGap = overlap > 0.0
                ? 0.0
                : std::max(eventLow - band.m_highFrequency, band.m_lowFrequency - eventHigh);
            const double centerDistance = std::fabs(band.m_centerFrequency - predictedFrequency);
            const double ridgeDistance = std::fabs(band.m_peakFrequency - predictedFrequency);
            const bool compatible = (frequencyGap <= padding)
                || (centerDistance <= maxTrackingJump);

            if (!compatible) {
                continue;
            }

            const double scale = std::max({padding, event.m_maxBandwidth, band.m_bandwidth, 1.0});
            const double bandwidthChange = std::fabs(band.m_bandwidth - event.m_maxBandwidth) / scale;
            const double score = centerDistance / scale
                + 0.20 * ridgeDistance / scale
                + frequencyGap / padding
                + 0.15 * bandwidthChange
                - 0.02 * std::min(30.0, band.m_contrastDB);
            matches.push_back({eventIndex, i, score});
        }
    }

    std::sort(
        matches.begin(),
        matches.end(),
        [](const MatchCandidate& left, const MatchCandidate& right) { return left.m_score < right.m_score; });

    for (const MatchCandidate& match : matches)
    {
        if (eventMatched[match.m_eventIndex] || bandMatched[match.m_bandIndex]) {
            continue;
        }

        updateSpectralEvent(m_spectralEvents[match.m_eventIndex], bands[match.m_bandIndex], frameCenterSample);
        eventMatched[match.m_eventIndex] = true;
        bandMatched[match.m_bandIndex] = true;
    }

    for (int eventIndex = 0; eventIndex < (int) m_spectralEvents.size(); eventIndex++)
    {
        SpectralEvent& event = m_spectralEvents[eventIndex];

        if (!event.m_valid) {
            continue;
        }

        if (!eventMatched[eventIndex]) {
            event.m_missingFrames++;
        }

        const double durationMS = 1000.0 * (double) (event.m_lastCenterSample - event.m_startCenterSample + m_spectralHopSize)
            / (double) std::max(1, m_settings.m_channelSampleRate);

        if ((event.m_missingFrames > maxMissingFrames)
            || (durationMS > (double) m_settings.m_maxDurationMS))
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
    event.m_backgroundPowerSum += std::max(band.m_backgroundPower, 1e-20);
    event.m_backgroundFrameCount++;
    event.m_backgroundPower = event.m_backgroundPowerSum / (double) event.m_backgroundFrameCount;
    event.m_totalPower += std::max(band.m_totalExcessPower, 0.0) * m_spectralEnergyScale;
    event.m_maxBandwidth = std::max(event.m_maxBandwidth, band.m_bandwidth);
    event.m_weightedFrequencySum += band.m_centerFrequency * std::max(band.m_totalExcessPower, 1e-30);
    event.m_weightSum += std::max(band.m_totalExcessPower, 1e-30);
    event.m_maxContrastDB = std::max(event.m_maxContrastDB, band.m_contrastDB);
    event.m_maxPeakRatio = std::max(event.m_maxPeakRatio, band.m_peakRatio);
    event.m_maxMinimumNoiseContrastDB = std::max(
        event.m_maxMinimumNoiseContrastDB,
        band.m_minimumNoiseContrastDB);
    event.m_weightedNoiseFloorDeltaDBSum += band.m_noiseFloorDeltaDB
        * std::max(band.m_totalExcessPower, 1e-30);
    event.m_trackFrequencies.push_back(band.m_centerFrequency);
    event.m_trackPeakFrequencies.push_back(band.m_peakFrequency);
    event.m_trackLowFrequencies.push_back(band.m_lowFrequency);
    event.m_trackHighFrequencies.push_back(band.m_highFrequency);
    event.m_trackBandwidths.push_back(band.m_bandwidth);
    event.m_trackSamples.push_back(frameCenterSample);
    event.m_trackStrengths.push_back(std::max(band.m_totalExcessPower, 1e-30));
    event.m_trackFrameOccupiedFractions.push_back(band.m_frameOccupiedFraction);
}

void MeteorDemodSink::finishSpectralEvent(const SpectralEvent& event)
{
    SpectralCandidate candidate = buildSpectralCandidate(event);

    if (candidate.m_valid && candidate.m_accepted && isSweepContinuation(candidate))
    {
        candidate.m_sweepContinuationRejected = true;
        candidate.m_accepted = false;
        candidate.m_classification = "sweep";
        candidate.m_rejectionReason = "sweep-continuation";
    }

    if (candidate.m_broadbandImpulse || candidate.m_sweepRejected || candidate.m_sweepContinuationRejected) {
        rememberSpectralInterference(candidate);
    }

    if (!candidate.m_valid)
    {
        auditSpectralCandidate(candidate);
        return;
    }

    const bool nearScoreMiss = !candidate.m_accepted
        && candidate.m_valid
        && !candidate.m_duplicate
        && candidate.m_durationOK
        && candidate.m_enoughFrames
        && (candidate.m_scoreMargin >= -1.0);

    if (candidate.m_accepted || nearScoreMiss || (candidate.m_spectralEvidenceOK && (candidate.m_durationS >= 1.0)))
    {
        qDebug() << "MeteorDemodSink::finishSpectralEvent:"
                 << " accepted:" << candidate.m_accepted
                 << " duplicate:" << candidate.m_duplicate
                 << " durationOK:" << candidate.m_durationOK
                 << " enoughFrames:" << candidate.m_enoughFrames
                 << " sweepRejected:" << candidate.m_sweepRejected
                 << " smoothSweepRejected:" << candidate.m_smoothSweepRejected
                 << " longDriftRejected:" << candidate.m_longDriftRejected
                 << " spectralEvidenceOK:" << candidate.m_spectralEvidenceOK
                 << " insideUsableBandwidth:" << candidate.m_insideUsableBandwidth
                 << " strongLineOK:" << candidate.m_strongLineOK
                 << " boundedBandOK:" << candidate.m_boundedBandOK
                 << " acceptanceScore:" << candidate.m_acceptanceScore
                 << " acceptanceThreshold:" << candidate.m_acceptanceThreshold
                 << " scoreMargin:" << candidate.m_scoreMargin
                 << " signalScore:" << candidate.m_signalScore
                 << " supportScore:" << candidate.m_supportScore
                 << " shapeScore:" << candidate.m_shapeScore
                 << " rejectionPenalty:" << candidate.m_rejectionPenalty
                 << " scoreOK:" << candidate.m_scoreOK
                 << " classification:" << candidate.m_classification
                 << " rejectionReason:" << candidate.m_rejectionReason
                 << " durationS:" << candidate.m_durationS
                 << " peakPowerDB:" << 10.0 * std::log10(std::max(event.m_peakPower, 1e-20))
                 << " backgroundPowerDB:" << 10.0 * std::log10(std::max(event.m_backgroundPower, 1e-20))
                 << " peakAboveBackgroundDB:" << candidate.m_peakAboveBackgroundDB
                 << " integratedSupportDB:" << candidate.m_integratedSupportDB
                 << " centerFrequency:" << candidate.m_centerFrequency
                 << " frequencySpan:" << candidate.m_frequencySpan
                 << " frequencyDrift:" << candidate.m_frequencyDrift
                 << " robustCenterFrequency:" << candidate.m_robustCenterFrequency
                 << " robustFrequencySpan:" << candidate.m_robustFrequencySpan
                 << " robustFrequencyDrift:" << candidate.m_robustFrequencyDrift
                 << " reportFrequencySpan:" << candidate.m_reportFrequencySpan
                 << " sweepScore:" << candidate.m_sweepScore
                 << " maxBandwidth:" << event.m_maxBandwidth
                 << " maxContrastDB:" << event.m_maxContrastDB
                 << " maxPeakRatio:" << event.m_maxPeakRatio
                 << " frames:" << candidate.m_frameCount
                 << " startSample:" << candidate.m_startSample
                 << " endSample:" << candidate.m_endSample
                 << " peakSample:" << candidate.m_peakSample
                 << " displayStartSample:" << candidate.m_displayStartSample
                 << " displayEndSample:" << candidate.m_displayEndSample;
    }

    if (!candidate.m_accepted)
    {
        captureCandidateDiagnostic(candidate);
        auditSpectralCandidate(candidate);
        return;
    }

    PulseReport report;
    report.m_valid = true;
    report.m_dateTimeUtc = sampleCounterToDateTimeUtc(candidate.m_startSample);
    report.m_startSample = candidate.m_startSample;
    report.m_endSample = candidate.m_endSample;
    report.m_hasDisplaySamples = true;
    report.m_displayStartSample = candidate.m_displayStartSample;
    report.m_displayEndSample = candidate.m_displayEndSample;
    report.m_peakPower = event.m_peakPower;
    report.m_backgroundPower = event.m_backgroundPower;
    report.m_totalPower = event.m_totalPower;
    report.m_durationS = candidate.m_durationS;
    report.m_centerFrequency = candidate.m_centerFrequency;
    report.m_frequencySpan = candidate.m_frequencySpan;
    report.m_frequencyDrift = candidate.m_frequencyDrift;
    report.m_reportFrequencySpan = candidate.m_reportFrequencySpan;
    report.m_duplicateFrequencySpan = candidate.m_frequencySpan;
    report.m_hasRobustFrequency = true;
    report.m_robustCenterFrequency = candidate.m_robustCenterFrequency;
    report.m_robustFrequencySpan = candidate.m_robustFrequencySpan;
    report.m_robustFrequencyDrift = candidate.m_robustFrequencyDrift;
    report.m_confidence = candidate.m_scoreMargin;
    report.m_componentSupportDB = candidate.m_integratedSupportDB;
    report.m_truncated = candidate.m_truncated;
    report.m_spectralParentEligible = true;

    const AssociationResult association = emitOrDeferSpectralReport(report);
    candidate.m_parentEventId = association.m_parentEventId;
    candidate.m_associationDecision = association.m_decision;
    auditSpectralCandidate(candidate);
}

bool MeteorDemodSink::isSweepContinuation(const SpectralCandidate& candidate) const
{
    if (!candidate.m_valid || (candidate.m_durationS > 0.5)) {
        return false;
    }

    const quint64 paddingSamples = (quint64) std::max(1, m_settings.m_channelSampleRate / 3);

    for (const SpectralInterferenceRange& range : m_recentSpectralInterference)
    {
        if ((candidate.m_startSample > range.m_endSample + paddingSamples)
            || (candidate.m_endSample + paddingSamples < range.m_startSample))
        {
            continue;
        }

        if (!range.m_broadband
            && (candidate.m_startSample <= range.m_endSample)
            && (candidate.m_endSample >= range.m_startSample))
        {
            return true;
        }

        const double centerDistance = std::fabs(candidate.m_robustCenterFrequency - range.m_centerFrequency);
        const double compatibleDistance = std::max(
            0.25 * (double) m_settings.m_channelSampleRate,
            0.5 * (candidate.m_robustFrequencySpan + range.m_frequencySpan) + 80.0);

        if (centerDistance <= compatibleDistance) {
            return true;
        }
    }

    return false;
}

bool MeteorDemodSink::overlapsBroadbandInterference(quint64 startSample, quint64 endSample) const
{
    const quint64 paddingSamples = (quint64) std::max(1, m_settings.m_channelSampleRate);
    const quint64 paddedStartSample = startSample > paddingSamples ? startSample - paddingSamples : 0;
    const quint64 paddedEndSample = endSample + paddingSamples;

    for (const SpectralInterferenceRange& range : m_recentSpectralInterference)
    {
        if (range.m_broadband
            && (paddedEndSample >= range.m_startSample)
            && (paddedStartSample <= range.m_endSample))
        {
            return true;
        }
    }

    return false;
}

void MeteorDemodSink::rememberSpectralInterference(const SpectralCandidate& candidate)
{
    if (!candidate.m_valid) {
        return;
    }

    const quint64 paddingSamples = (quint64) std::max(1, m_settings.m_channelSampleRate / 3);
    const double candidateLow = candidate.m_robustCenterFrequency - 0.5 * candidate.m_robustFrequencySpan;
    const double candidateHigh = candidate.m_robustCenterFrequency + 0.5 * candidate.m_robustFrequencySpan;
    bool extended = false;

    for (SpectralInterferenceRange& range : m_recentSpectralInterference)
    {
        if ((candidate.m_startSample > range.m_endSample + paddingSamples)
            || (candidate.m_endSample + paddingSamples < range.m_startSample))
        {
            continue;
        }

        const double rangeLow = range.m_centerFrequency - 0.5 * range.m_frequencySpan;
        const double rangeHigh = range.m_centerFrequency + 0.5 * range.m_frequencySpan;
        const double frequencyGap = std::max(rangeLow - candidateHigh, candidateLow - rangeHigh);
        const double frequencyPadding = std::max(80.0, 0.25 * (double) m_settings.m_channelSampleRate);

        if (!range.m_broadband && !candidate.m_broadbandImpulse && (frequencyGap > frequencyPadding)) {
            continue;
        }

        const double mergedLow = std::min(rangeLow, candidateLow);
        const double mergedHigh = std::max(rangeHigh, candidateHigh);
        range.m_startSample = std::min(range.m_startSample, candidate.m_startSample);
        range.m_endSample = std::max(range.m_endSample, candidate.m_endSample);
        range.m_centerFrequency = 0.5 * (mergedLow + mergedHigh);
        range.m_frequencySpan = mergedHigh - mergedLow;
        range.m_broadband = range.m_broadband || candidate.m_broadbandImpulse;
        extended = true;
        break;
    }

    if (!extended)
    {
        m_recentSpectralInterference.push_back({
            candidate.m_startSample,
            candidate.m_endSample,
            candidate.m_robustCenterFrequency,
            candidate.m_robustFrequencySpan,
            candidate.m_broadbandImpulse
        });
    }

    const quint64 keepSamples = (quint64) std::max(1, m_settings.m_channelSampleRate)
        * (quint64) std::max(3, m_settings.m_maxDurationMS / 1000 + 2);
    const quint64 oldestSample = candidate.m_endSample > keepSamples ? candidate.m_endSample - keepSamples : 0;
    m_recentSpectralInterference.erase(
        std::remove_if(
            m_recentSpectralInterference.begin(),
            m_recentSpectralInterference.end(),
            [oldestSample](const SpectralInterferenceRange& range) { return range.m_endSample < oldestSample; }),
        m_recentSpectralInterference.end());
}

void MeteorDemodSink::auditSpectralCandidate(const SpectralCandidate& candidate) const
{
    if (m_candidateAuditCallback) {
        m_candidateAuditCallback(candidate);
    }
}

void MeteorDemodSink::captureCandidateDiagnostic(const SpectralCandidate& candidate) const
{
    if (!m_candidateDiagnosticCaptureCallback
        || !candidate.m_valid
        || candidate.m_accepted
        || candidate.m_duplicate
        || !candidate.m_durationOK
        || !candidate.m_insideUsableBandwidth
        || candidate.m_sweepRejected
        || candidate.m_sweepContinuationRejected
        || candidate.m_broadbandImpulse
        || (candidate.m_frameCount < m_detectorTunables.m_candidateDiagnosticMinimumFrames)
        || (candidate.m_scoreMargin < m_detectorTunables.m_candidateDiagnosticMinimumMargin)
        || (candidate.m_scoreMargin > m_detectorTunables.m_candidateDiagnosticMaximumMargin))
    {
        return;
    }

    ComplexVector samples;

    if (copyDetectionSamples(candidate.m_startSample, candidate.m_endSample, samples)) {
        m_candidateDiagnosticCaptureCallback(candidate, samples);
    }
}

MeteorDemodSink::SpectralCandidate MeteorDemodSink::buildSpectralCandidate(const SpectralEvent& event) const
{
    SpectralCandidate candidate;

    if (!event.m_valid || event.m_trackFrequencies.empty()) {
        return candidate;
    }

    double startSampleEstimate = 0.0;
    const double estimatedDurationSamples = estimateSpectralEventDurationSamples(event, startSampleEstimate);
    const quint64 startSample = startSampleEstimate > 0.0 ? (quint64) std::llround(startSampleEstimate) : 0;
    const quint64 maximumDurationSamples = (quint64) std::max(
        1,
        m_settings.m_maxDurationMS * std::max(1, m_settings.m_channelSampleRate) / 1000);
    const quint64 estimatedDurationSampleCount = (quint64) std::max(1.0, (double) std::llround(estimatedDurationSamples));
    const quint64 durationSampleCount = std::min(estimatedDurationSampleCount, maximumDurationSamples);
    const quint64 endSample = startSample + durationSampleCount - 1;
    const int count = (int) event.m_trackFrequencies.size();
    const int edgeCount = std::clamp(count / 4, 1, 4);
    const std::vector<double>& driftTrack = event.m_trackPeakFrequencies.size() == event.m_trackFrequencies.size()
        ? event.m_trackPeakFrequencies
        : event.m_trackFrequencies;
    int peakFrameIndex = 0;
    double sweepScore = 0.0;

    if (event.m_trackStrengths.size() == event.m_trackFrequencies.size())
    {
        for (int i = 1; i < count; i++)
        {
            if (event.m_trackStrengths[i] > event.m_trackStrengths[peakFrameIndex]) {
                peakFrameIndex = i;
            }
        }
    }

    candidate.m_valid = true;
    candidate.m_sampleRate = m_settings.m_channelSampleRate;
    candidate.m_truncated = estimatedDurationSampleCount > maximumDurationSamples;
    candidate.m_startSample = startSample;
    candidate.m_endSample = endSample;
    candidate.m_peakSample = event.m_trackSamples.size() == event.m_trackFrequencies.size()
        ? event.m_trackSamples[peakFrameIndex]
        : startSample + durationSampleCount / 2;
    candidate.m_displayStartSample = candidate.m_peakSample > (durationSampleCount / 2)
        ? candidate.m_peakSample - (durationSampleCount / 2)
        : 0;
    candidate.m_displayEndSample = candidate.m_displayStartSample + durationSampleCount - 1;
    candidate.m_durationS = (double) durationSampleCount / (double) std::max(1, m_settings.m_channelSampleRate);
    candidate.m_centerFrequency = event.m_weightSum > 0.0
        ? event.m_weightedFrequencySum / event.m_weightSum
        : averageFrequency(event.m_trackFrequencies, 0, count);
    candidate.m_frequencySpan = std::max(std::fabs(event.m_maxFrequency - event.m_minFrequency), event.m_maxBandwidth);
    candidate.m_frequencyDrift = count > 1
        ? averageFrequency(driftTrack, count - edgeCount, count) - averageFrequency(driftTrack, 0, edgeCount)
        : 0.0;
    candidate.m_robustCenterFrequency = weightedQuantile(
        event.m_trackFrequencies,
        event.m_trackStrengths,
        0.5,
        candidate.m_centerFrequency);
    candidate.m_robustFrequencySpan = candidate.m_frequencySpan;
    candidate.m_robustFrequencyDrift = candidate.m_frequencyDrift;
    candidate.m_reportFrequencySpan = candidate.m_frequencySpan;

    if ((count >= 5)
        && (event.m_trackLowFrequencies.size() == event.m_trackFrequencies.size())
        && (event.m_trackHighFrequencies.size() == event.m_trackFrequencies.size()))
    {
        const double lowFrequency = weightedQuantile(
            event.m_trackLowFrequencies,
            event.m_trackStrengths,
            0.10,
            event.m_minFrequency);
        const double highFrequency = weightedQuantile(
            event.m_trackHighFrequencies,
            event.m_trackStrengths,
            0.90,
            event.m_maxFrequency);
        candidate.m_robustFrequencySpan = std::max(event.m_maxBandwidth, highFrequency - lowFrequency);

        const int robustEdgeCount = std::clamp(count / 3, 2, 6);
        candidate.m_robustFrequencyDrift = weightedQuantile(
            driftTrack,
            event.m_trackStrengths,
            count - robustEdgeCount,
            count,
            0.5,
            averageFrequency(driftTrack, count - robustEdgeCount, count))
            - weightedQuantile(
                driftTrack,
                event.m_trackStrengths,
                0,
                robustEdgeCount,
                0.5,
                averageFrequency(driftTrack, 0, robustEdgeCount));
    }

    if ((event.m_trackBandwidths.size() == event.m_trackFrequencies.size())
        && (event.m_trackStrengths.size() == event.m_trackFrequencies.size()))
    {
        const double bandwidth90 = weightedQuantile(
            event.m_trackBandwidths,
            event.m_trackStrengths,
            0.90,
            event.m_maxBandwidth);
        const double binWidth = m_spectralFrameSize > 0
            ? (double) std::max(1, m_settings.m_channelSampleRate) / (double) m_spectralFrameSize
            : 0.0;
        const double driftCoveredSpan = bandwidth90 + std::fabs(candidate.m_robustFrequencyDrift);
        const double paddedSpan = std::max({
            candidate.m_frequencySpan,
            candidate.m_robustFrequencySpan,
            driftCoveredSpan,
            bandwidth90
        }) + (2.0 * binWidth);
        candidate.m_reportFrequencySpan = std::min(
            std::max(0.0, paddedSpan),
            0.90 * (double) std::max(1, m_settings.m_channelSampleRate));
    }

    candidate.m_peakAboveBackgroundDB = 10.0 * std::log10(std::max(event.m_peakPower, 1e-20) / std::max(event.m_backgroundPower, 1e-20));
    const double averageExcessPower = event.m_totalPower / (double) std::max<quint64>(1, durationSampleCount);
    candidate.m_integratedSupportDB = 10.0 * std::log10(
        std::max(averageExcessPower, 1e-20) / std::max(event.m_backgroundPower, 1e-20));
    candidate.m_maxBandwidth = event.m_maxBandwidth;
    candidate.m_maxContrastDB = event.m_maxContrastDB;
    candidate.m_maxPeakRatio = event.m_maxPeakRatio;
    candidate.m_minimumNoiseContrastDB = event.m_maxMinimumNoiseContrastDB;
    candidate.m_noiseFloorDeltaDB = event.m_weightSum > 0.0
        ? event.m_weightedNoiseFloorDeltaDBSum / event.m_weightSum
        : 0.0;
    candidate.m_frameCount = count;
    const int expectedFrameCount = event.m_trackSamples.size() == event.m_trackFrequencies.size()
        ? 1 + (int) ((event.m_trackSamples.back() - event.m_trackSamples.front())
            / (quint64) std::max(1, m_spectralHopSize))
        : count;
    candidate.m_trackOccupancy = std::clamp(
        (double) count / (double) std::max(1, expectedFrameCount),
        0.0,
        1.0);

    if ((event.m_trackFrameOccupiedFractions.size() == event.m_trackFrequencies.size())
        && (event.m_trackStrengths.size() == event.m_trackFrequencies.size()))
    {
        candidate.m_frameOccupiedFraction = weightedQuantile(
            event.m_trackFrameOccupiedFractions,
            event.m_trackStrengths,
            0.5,
            0.0);
    }

    if (event.m_trackStrengths.size() == event.m_trackFrequencies.size())
    {
        std::vector<double> frequencyDeviations(count);

        for (int i = 0; i < count; i++) {
            frequencyDeviations[i] = std::fabs(event.m_trackFrequencies[i] - candidate.m_robustCenterFrequency);
        }

        const double medianDeviation = weightedQuantile(
            frequencyDeviations,
            event.m_trackStrengths,
            0.5,
            0.5 * event.m_maxBandwidth);
        const double coherenceScale = std::max(
            (double) std::max(1, m_settings.m_channelSampleRate) / (double) std::max(1, m_spectralFrameSize),
            0.5 * event.m_maxBandwidth);
        candidate.m_frequencyCoherence = std::clamp(1.0 - medianDeviation / coherenceScale, 0.0, 1.0);
    }

    if (count >= 4)
    {
        const double meanX = 0.5 * (double) (count - 1);
        const double meanY = averageFrequency(driftTrack, 0, count);
        double ssXX = 0.0;
        double ssXY = 0.0;
        double ssYY = 0.0;

        for (int i = 0; i < count; i++)
        {
            const double dx = (double) i - meanX;
            const double dy = driftTrack[i] - meanY;
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

    candidate.m_sweepScore = sweepScore;

    const double durationMS = 1000.0 * candidate.m_durationS;
    const double compactFrequencyLimit = m_resolvedDetectorTunables.m_compactBandwidthHz;
    const double stableFrequencyLimit = m_resolvedDetectorTunables.m_stableBandwidthHz;
    const bool driftLimitEnabled = m_settings.m_maxFrequencyDrift > 0.0f;
    const double sweepFrequencyLimit = driftLimitEnabled
        ? (double) m_settings.m_maxFrequencyDrift
        : std::numeric_limits<double>::max();

    candidate.m_durationOK = (durationMS >= m_settings.m_minDurationMS) && (durationMS <= m_settings.m_maxDurationMS);
    bool recentCompatibleFragment = false;

    if (count == 2)
    {
        const quint64 fragmentGapSamples = (quint64) std::max(1, m_settings.m_channelSampleRate) * 5;
        const double candidateLow = candidate.m_robustCenterFrequency - 0.5 * candidate.m_robustFrequencySpan;
        const double candidateHigh = candidate.m_robustCenterFrequency + 0.5 * candidate.m_robustFrequencySpan;
        const double frequencyPadding = std::max(12.0, 0.012 * (double) m_settings.m_channelSampleRate);

        for (const DetectionRange& range : m_recentDetectionRanges)
        {
            if ((candidate.m_startSample < range.m_endSample)
                || (candidate.m_startSample - range.m_endSample > fragmentGapSamples))
            {
                continue;
            }

            const double frequencyGap = std::max(
                range.m_lowFrequency - candidateHigh,
                candidateLow - range.m_highFrequency);

            if (frequencyGap <= frequencyPadding)
            {
                recentCompatibleFragment = true;
                break;
            }
        }
    }

    const bool compactTwoFrameEcho = (count == 2)
        && !recentCompatibleFragment
        && (candidate.m_maxBandwidth <= m_resolvedDetectorTunables.m_twoFrameMaxBandwidthHz)
        && (candidate.m_maxContrastDB >= m_detectorTunables.m_morphologyRescueContrastDB)
        && (candidate.m_integratedSupportDB >= m_detectorTunables.m_twoFrameMinIntegratedSupportDB)
        && (candidate.m_frequencyCoherence >= m_detectorTunables.m_twoFrameMinFrequencyCoherence);
    const bool localizedWideTwoFrameEcho = (count == 2)
        && (candidate.m_frameOccupiedFraction < m_detectorTunables.m_localizedOccupiedMaxFraction)
        && (candidate.m_peakAboveBackgroundDB >= m_detectorTunables.m_localizedTwoFrameMinPeakDB)
        && (candidate.m_maxContrastDB >= m_detectorTunables.m_localizedTwoFrameMinContrastDB)
        && (candidate.m_integratedSupportDB >= 0.0)
        && (candidate.m_frequencyCoherence >= m_detectorTunables.m_twoFrameMinFrequencyCoherence);
    const bool coherentWideTwoFrameEcho = (count == 2)
        && !recentCompatibleFragment
        && (candidate.m_maxBandwidth
            >= m_resolvedDetectorTunables.m_coherentWideTwoFrameMinBandwidthHz)
        && (candidate.m_maxBandwidth
            <= m_resolvedDetectorTunables.m_coherentWideTwoFrameMaxBandwidthHz)
        && (candidate.m_frameOccupiedFraction
            < m_detectorTunables.m_coherentWideTwoFrameMaxOccupiedFraction)
        && (candidate.m_peakAboveBackgroundDB
            >= m_detectorTunables.m_coherentWideTwoFrameMinPeakDB)
        && (candidate.m_maxContrastDB
            >= m_detectorTunables.m_coherentWideTwoFrameMinContrastDB)
        && (candidate.m_integratedSupportDB
            >= m_detectorTunables.m_coherentWideTwoFrameMinIntegratedSupportDB)
        && (candidate.m_frequencyCoherence
            >= m_detectorTunables.m_coherentWideTwoFrameMinFrequencyCoherence)
        && (candidate.m_sweepScore < m_detectorTunables.m_driftSweepMinR2);
    candidate.m_enoughFrames = (count >= 3)
        || compactTwoFrameEcho
        || localizedWideTwoFrameEcho
        || coherentWideTwoFrameEcho;
    const bool exceptionalShortEcho = (candidate.m_peakAboveBackgroundDB >= 25.0)
        && (candidate.m_integratedSupportDB >= 15.0);
    const bool sustainedSmoothSweep = (candidate.m_durationS >= m_detectorTunables.m_sustainedSweepMinDurationS)
        && (candidate.m_frameCount >= m_detectorTunables.m_sustainedSweepMinFrames)
        && (candidate.m_sweepScore >= m_detectorTunables.m_sustainedSweepMinR2)
        && (std::fabs(candidate.m_robustFrequencyDrift)
            >= m_resolvedDetectorTunables.m_sustainedSweepMinDriftHz);
    const bool compactSmoothSweep = (candidate.m_durationS >= m_detectorTunables.m_compactSweepMinDurationS)
        && (candidate.m_frameCount >= m_detectorTunables.m_compactSweepMinFrames)
        && (candidate.m_trackOccupancy < m_detectorTunables.m_compactSweepMaxTrackOccupancy)
        && (candidate.m_maxContrastDB < m_detectorTunables.m_compactSweepMaxContrastDB)
        && (candidate.m_sweepScore >= m_detectorTunables.m_compactSweepMinR2)
        && (std::fabs(candidate.m_robustFrequencyDrift)
            >= m_resolvedDetectorTunables.m_compactSweepMinDriftHz);
    candidate.m_smoothSweepRejected = sustainedSmoothSweep || compactSmoothSweep
        || (driftLimitEnabled
            && (candidate.m_sweepScore >= m_detectorTunables.m_driftSweepMinR2)
            && (std::fabs(candidate.m_robustFrequencyDrift) > sweepFrequencyLimit)
            && ((candidate.m_durationS >= 0.5) || !exceptionalShortEcho));
    candidate.m_longDriftRejected = driftLimitEnabled
        && (candidate.m_durationS >= 1.0)
        && (std::fabs(candidate.m_robustFrequencyDrift) > sweepFrequencyLimit)
        && (candidate.m_robustFrequencySpan > sweepFrequencyLimit);
    candidate.m_sweepRejected = candidate.m_smoothSweepRejected || candidate.m_longDriftRejected;
    const double morphologyContrastDB = std::max(13.0, (double) m_settings.m_detectionThresholdDB + 3.0);
    const bool morphologyRescueOK = (event.m_maxContrastDB >= m_detectorTunables.m_morphologyRescueContrastDB)
        || (candidate.m_peakAboveBackgroundDB >= m_detectorTunables.m_morphologyRescueContrastDB)
        || (candidate.m_integratedSupportDB >= -2.0);
    candidate.m_strongLineOK = morphologyRescueOK
        && (event.m_maxContrastDB >= morphologyContrastDB)
        && (candidate.m_peakAboveBackgroundDB >= std::max(8.0, (double) m_settings.m_detectionThresholdDB));
    candidate.m_boundedBandOK = morphologyRescueOK
        && (event.m_maxContrastDB >= morphologyContrastDB)
        && (candidate.m_peakAboveBackgroundDB >= 8.0)
        && (event.m_maxBandwidth <= stableFrequencyLimit);
    candidate.m_spectralEvidenceOK = candidate.m_strongLineOK || candidate.m_boundedBandOK;
    const bool marginalThreeFrameNoise = (candidate.m_frameCount == 3)
        && (candidate.m_maxContrastDB < m_detectorTunables.m_morphologyRescueContrastDB)
        && ((candidate.m_frequencyCoherence < 0.90)
            || ((candidate.m_integratedSupportDB < 7.0) && (candidate.m_peakAboveBackgroundDB < 12.0)));

    if (marginalThreeFrameNoise) {
        candidate.m_spectralEvidenceOK = false;
    }
    candidate.m_broadbandImpulse = (candidate.m_durationS <= m_detectorTunables.m_broadbandImpulseMaxDurationS)
        && (candidate.m_frameCount <= m_detectorTunables.m_broadbandImpulseMaxFrames)
        && (candidate.m_peakAboveBackgroundDB >= m_detectorTunables.m_broadbandImpulseMinPeakDB)
        && (candidate.m_robustFrequencySpan
            >= m_resolvedDetectorTunables.m_broadbandImpulseMinSpanHz)
        && (candidate.m_maxBandwidth
            >= m_resolvedDetectorTunables.m_broadbandImpulseMinBandwidthHz)
        && (candidate.m_frameOccupiedFraction >= m_detectorTunables.m_broadbandImpulseMinOccupiedFraction);
    candidate.m_insideUsableBandwidth = std::fabs(candidate.m_robustCenterFrequency)
        <= m_resolvedDetectorTunables.m_usableBandwidthHz;
    candidate.m_duplicate = isDuplicateDetection(
        candidate.m_startSample,
        candidate.m_endSample,
        candidate.m_robustCenterFrequency,
        candidate.m_robustFrequencySpan);
    candidate.m_acceptanceThreshold = candidate.m_frameCount <= 2
        ? m_detectorTunables.m_shortCandidateAcceptanceScore
        : m_detectorTunables.m_candidateAcceptanceScore;

    if (candidate.m_integratedSupportDB < m_detectorTunables.m_weakSupportDB) {
        candidate.m_acceptanceThreshold += m_detectorTunables.m_weakSupportScorePenalty;
    }

    calculateShadowCandidateFeatures(candidate, event);
    classifySpectralCandidate(candidate);

    candidate.m_curvedSweepRejected = m_detectorTunables.m_enableCurvatureSweepRejection
        && (candidate.m_quadraticSweepR2 >= m_detectorTunables.m_curvatureMinimumR2)
        && (candidate.m_quadraticSweepImprovement
            >= m_detectorTunables.m_curvatureMinimumR2Improvement)
        && (std::fabs(candidate.m_quadraticCurvatureHzPerS2)
            >= m_detectorTunables.m_curvatureMinimumHzPerS2)
        && (candidate.m_matchedEnvelopeScore < m_detectorTunables.m_rescueMinimumMatchedEnvelopeScore);

    if (candidate.m_curvedSweepRejected)
    {
        candidate.m_sweepRejected = true;
        classifySpectralCandidate(candidate);
    }

    const bool learnedRescue = m_detectorTunables.m_learnedModelEnabled
        && (candidate.m_learnedScore >= m_detectorTunables.m_learnedRescueProbability);
    const bool twoFrameRescue = (candidate.m_frameCount == 2)
        && (candidate.m_maxContrastDB >= m_detectorTunables.m_rescueTwoFrameMinimumContrastDB)
        && (candidate.m_peakAboveBackgroundDB >= m_detectorTunables.m_rescueTwoFrameMinimumPeakDB)
        && (candidate.m_integratedSupportDB
            >= m_detectorTunables.m_rescueTwoFrameMinimumIntegratedSupportDB)
        && (candidate.m_frequencyCoherence
            >= m_detectorTunables.m_rescueTwoFrameMinimumFrequencyCoherence);
    const bool threeFrameRescue = (candidate.m_frameCount == 3)
        && (candidate.m_maxContrastDB >= m_detectorTunables.m_rescueMinimumContrastDB)
        && (candidate.m_peakAboveBackgroundDB >= m_detectorTunables.m_rescueMinimumPeakDB)
        && (candidate.m_frequencyCoherence
            >= m_detectorTunables.m_rescueMinimumFrequencyCoherence)
        && ((candidate.m_matchedEnvelopeScore
                >= m_detectorTunables.m_rescueMinimumMatchedEnvelopeScore)
            || (candidate.m_integratedSupportDB
                >= m_detectorTunables.m_rescueThreeFrameMinimumIntegratedSupportDB));
    const bool rescueSafetyOK = candidate.m_valid
        && !candidate.m_accepted
        && !candidate.m_duplicate
        && candidate.m_durationOK
        && candidate.m_insideUsableBandwidth
        && !candidate.m_sweepRejected
        && !candidate.m_sweepContinuationRejected
        && !candidate.m_broadbandImpulse
        && (candidate.m_frameCount >= 2)
        && (candidate.m_frameOccupiedFraction < m_detectorTunables.m_rescueMaximumOccupiedFraction)
        && (candidate.m_maxBandwidth <= m_resolvedDetectorTunables.m_stableBandwidthHz)
        && ((candidate.m_frameCount < 4)
            || (candidate.m_sweepScore < m_detectorTunables.m_driftSweepMinR2));

    if (m_detectorTunables.m_enableCalibratedRescue
        && rescueSafetyOK
        && (candidate.m_scoreMargin >= m_detectorTunables.m_rescueMinimumScoreMargin)
        && (learnedRescue || twoFrameRescue || threeFrameRescue))
    {
        candidate.m_calibratedRescue = true;
        candidate.m_rescuedFramesGate = !candidate.m_enoughFrames;
        candidate.m_rescuedSpectralEvidenceGate = !candidate.m_spectralEvidenceOK;
        candidate.m_enoughFrames = true;
        candidate.m_spectralEvidenceOK = true;
        classifySpectralCandidate(candidate);
    }

    return candidate;
}

void MeteorDemodSink::calculateShadowCandidateFeatures(
    SpectralCandidate& candidate,
    const SpectralEvent& event) const
{
    const int count = (int) event.m_trackStrengths.size();
    const double sampleRate = (double) std::max(1, m_settings.m_channelSampleRate);
    candidate.m_centerFrequencyRateFraction = candidate.m_robustCenterFrequency / sampleRate;
    candidate.m_frequencySpanRateFraction = candidate.m_robustFrequencySpan / sampleRate;
    candidate.m_frequencyDriftRateFraction = candidate.m_robustFrequencyDrift / sampleRate;
    candidate.m_maxBandwidthRateFraction = candidate.m_maxBandwidth / sampleRate;

    if ((count >= m_detectorTunables.m_matchedEnvelopeMinimumFrames)
        && (event.m_trackSamples.size() == event.m_trackStrengths.size()))
    {
        const auto peak = std::max_element(event.m_trackStrengths.begin(), event.m_trackStrengths.end());
        const int peakIndex = (int) std::distance(event.m_trackStrengths.begin(), peak);
        const double peakStrength = std::max(*peak, 1e-30);
        for (double timeConstantS : m_detectorTunables.m_matchedEnvelopeTimeConstantsS)
        {
            double dot = 0.0;
            double signalEnergy = 0.0;
            double templateEnergy = 0.0;

            for (int i = peakIndex; i < count; i++)
            {
                const double elapsedS = (double) (event.m_trackSamples[i] - event.m_trackSamples[peakIndex])
                    / (double) std::max(1, m_settings.m_channelSampleRate);
                const double signal = std::clamp(event.m_trackStrengths[i] / peakStrength, 0.0, 1.0);
                const double model = std::exp(-elapsedS / timeConstantS);
                dot += signal * model;
                signalEnergy += signal * signal;
                templateEnergy += model * model;
            }

            const double score = dot / std::sqrt(std::max(1e-30, signalEnergy * templateEnergy));

            if (score > candidate.m_matchedEnvelopeScore)
            {
                candidate.m_matchedEnvelopeScore = score;
                candidate.m_decayTimeConstantS = timeConstantS;
            }
        }
    }

    if ((count < m_detectorTunables.m_curvatureMinimumFrames)
        || (candidate.m_durationS < m_detectorTunables.m_curvatureMinimumDurationS)
        || (event.m_trackSamples.size() != event.m_trackStrengths.size())
        || (event.m_trackPeakFrequencies.size() != event.m_trackStrengths.size()))
    {
        candidate.m_learnedScore = evaluateLearnedCandidate(candidate);
        return;
    }

    const quint64 middleSample = event.m_trackSamples[count / 2];
    double sums[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double weightedMean = 0.0;
    double weightSum = 0.0;

    for (int i = 0; i < count; i++)
    {
        const double x = ((double) event.m_trackSamples[i] - (double) middleSample) / sampleRate;
        const double y = event.m_trackPeakFrequencies[i];
        const double weight = std::sqrt(std::max(event.m_trackStrengths[i], 1e-30));
        const double x2 = x * x;
        sums[0] += weight;
        sums[1] += weight * x;
        sums[2] += weight * x2;
        sums[3] += weight * x2 * x;
        sums[4] += weight * x2 * x2;
        sums[5] += weight * y;
        sums[6] += weight * x * y;
        weightedMean += weight * y;
        weightSum += weight;
    }

    double quadraticY = 0.0;

    for (int i = 0; i < count; i++)
    {
        const double x = ((double) event.m_trackSamples[i] - (double) middleSample) / sampleRate;
        const double y = event.m_trackPeakFrequencies[i];
        const double weight = std::sqrt(std::max(event.m_trackStrengths[i], 1e-30));
        quadraticY += weight * x * x * y;
    }

    double matrix[3][4] = {
        {sums[0], sums[1], sums[2], sums[5]},
        {sums[1], sums[2], sums[3], sums[6]},
        {sums[2], sums[3], sums[4], quadraticY}
    };
    double coefficients[3];

    if (!solve3x3(matrix, coefficients) || (weightSum <= 0.0))
    {
        candidate.m_learnedScore = evaluateLearnedCandidate(candidate);
        return;
    }

    weightedMean /= weightSum;
    double totalError = 0.0;
    double quadraticError = 0.0;
    double linearError = 0.0;
    const double linearSlope = std::fabs(sums[0] * sums[2] - sums[1] * sums[1]) > 1e-12
        ? (sums[0] * sums[6] - sums[1] * sums[5])
            / (sums[0] * sums[2] - sums[1] * sums[1])
        : 0.0;
    const double linearIntercept = (sums[5] - linearSlope * sums[1]) / sums[0];

    for (int i = 0; i < count; i++)
    {
        const double x = ((double) event.m_trackSamples[i] - (double) middleSample) / sampleRate;
        const double y = event.m_trackPeakFrequencies[i];
        const double weight = std::sqrt(std::max(event.m_trackStrengths[i], 1e-30));
        const double totalResidual = y - weightedMean;
        const double quadraticResidual = y - (coefficients[0] + coefficients[1] * x + coefficients[2] * x * x);
        const double linearResidual = y - (linearIntercept + linearSlope * x);
        totalError += weight * totalResidual * totalResidual;
        quadraticError += weight * quadraticResidual * quadraticResidual;
        linearError += weight * linearResidual * linearResidual;
    }

    candidate.m_quadraticSweepR2 = totalError > 1e-20
        ? std::clamp(1.0 - quadraticError / totalError, 0.0, 1.0)
        : 0.0;
    candidate.m_quadraticSweepImprovement = linearError > 1e-20
        ? std::clamp((linearError - quadraticError) / linearError, 0.0, 1.0)
        : 0.0;
    candidate.m_quadraticCurvatureHzPerS2 = 2.0 * coefficients[2];
    candidate.m_learnedScore = evaluateLearnedCandidate(candidate);
}

MeteorDemodSink::LearnedFeatureVector MeteorDemodSink::candidateLearnedFeatures(
    const SpectralCandidate& candidate) const
{
    return {{
        candidate.m_peakAboveBackgroundDB,
        candidate.m_maxContrastDB,
        candidate.m_integratedSupportDB,
        std::log10(std::max(candidate.m_maxPeakRatio, 1.0)),
        candidate.m_durationS,
        (double) candidate.m_frameCount,
        candidate.m_trackOccupancy,
        candidate.m_frequencyCoherence,
        candidate.m_frameOccupiedFraction,
        candidate.m_centerFrequencyRateFraction,
        candidate.m_frequencySpanRateFraction,
        candidate.m_frequencyDriftRateFraction,
        candidate.m_maxBandwidthRateFraction,
        candidate.m_matchedEnvelopeScore,
        candidate.m_quadraticSweepImprovement,
        candidate.m_noiseFloorDeltaDB
    }};
}

double MeteorDemodSink::evaluateLearnedCandidate(const SpectralCandidate& candidate) const
{
    if (!m_detectorTunables.m_learnedModelEnabled) {
        return 0.0;
    }

    const LearnedFeatureVector features = candidateLearnedFeatures(candidate);
    double logit = m_detectorTunables.m_learnedModelIntercept;

    for (int i = 0; i < m_learnedFeatureCount; i++)
    {
        const double scale = std::fabs(m_detectorTunables.m_learnedModelScales[i]) > 1e-12
            ? m_detectorTunables.m_learnedModelScales[i]
            : 1.0;
        logit += m_detectorTunables.m_learnedModelWeights[i]
            * ((features[i] - m_detectorTunables.m_learnedModelMeans[i]) / scale);
    }

    if (logit >= 0.0) {
        return 1.0 / (1.0 + std::exp(-logit));
    }

    const double exponential = std::exp(logit);
    return exponential / (1.0 + exponential);
}

void MeteorDemodSink::classifySpectralCandidate(SpectralCandidate& candidate) const
{
    candidate.m_acceptanceScore = scoreSpectralCandidate(candidate);
    candidate.m_scoreMargin = candidate.m_acceptanceScore - candidate.m_acceptanceThreshold;
    candidate.m_scoreOK = candidate.m_scoreMargin >= 0.0;
    candidate.m_classification = "rejected";
    candidate.m_rejectionReason = "none";
    candidate.m_accepted = false;

    if (!candidate.m_valid)
    {
        candidate.m_classification = "invalid";
        candidate.m_rejectionReason = "invalid";
        return;
    }

    if (candidate.m_duplicate)
    {
        candidate.m_classification = "duplicate";
        candidate.m_rejectionReason = "duplicate";
        return;
    }

    if (!candidate.m_durationOK)
    {
        candidate.m_rejectionReason = "duration";
        return;
    }

    if (!candidate.m_enoughFrames)
    {
        candidate.m_rejectionReason = "frames";
        return;
    }

    if (candidate.m_sweepRejected)
    {
        candidate.m_classification = "sweep";
        candidate.m_rejectionReason = candidate.m_curvedSweepRejected
            ? "curved-sweep"
            : (candidate.m_longDriftRejected ? "long-drift" : "smooth-sweep");
        return;
    }

    if (candidate.m_broadbandImpulse)
    {
        candidate.m_classification = "interference";
        candidate.m_rejectionReason = "broadband-impulse";
        return;
    }

    if (!candidate.m_spectralEvidenceOK)
    {
        candidate.m_rejectionReason = "spectral-evidence";
        return;
    }

    if (!candidate.m_insideUsableBandwidth)
    {
        candidate.m_classification = "out-of-band";
        candidate.m_rejectionReason = "bandwidth";
        return;
    }

    if (!candidate.m_scoreOK)
    {
        candidate.m_rejectionReason = "score";
        return;
    }

    candidate.m_classification = "meteor";
    candidate.m_rejectionReason = "accepted";
    candidate.m_accepted = true;
}

double MeteorDemodSink::scoreSpectralCandidate(SpectralCandidate& candidate) const
{
    if (!candidate.m_valid) {
        return 0.0;
    }

    const double thresholdDB = std::max(6.0, (double) m_settings.m_detectionThresholdDB);
    const double usableBandwidth = m_resolvedDetectorTunables.m_usableBandwidthHz;
    const double stableBandwidth = m_resolvedDetectorTunables.m_stableBandwidthHz;
    const bool driftLimitEnabled = m_settings.m_maxFrequencyDrift > 0.0f;
    const double sweepFrequencyLimit = driftLimitEnabled ? (double) m_settings.m_maxFrequencyDrift : 1.0;
    const double driftRatio = driftLimitEnabled
        ? std::fabs(candidate.m_robustFrequencyDrift) / std::max(1.0, sweepFrequencyLimit)
        : 0.0;
    const double bandwidthRatio = candidate.m_maxBandwidth / std::max(1.0, stableBandwidth);
    const double usableDistanceRatio = std::fabs(candidate.m_robustCenterFrequency) / std::max(1.0, usableBandwidth);

    candidate.m_signalScore =
        std::clamp((candidate.m_peakAboveBackgroundDB - thresholdDB) / 5.0, -2.0, 3.0)
        + std::clamp((candidate.m_maxContrastDB - 8.0) / 5.0, -1.5, 2.5)
        + std::clamp(std::log10(std::max(candidate.m_maxPeakRatio, 1.0)) / 2.0, 0.0, 1.5)
        + std::clamp((candidate.m_integratedSupportDB - 1.0) / 3.0, -2.0, 2.0);

    candidate.m_supportScore =
        std::clamp(((double) candidate.m_frameCount - 2.0) / 4.0, -1.0, 2.0)
        + std::clamp(
            (candidate.m_durationS - m_detectorTunables.m_scoreDurationFloorS)
                / std::max(m_detectorTunables.m_scoreDurationRangeS, 1e-6),
            -0.5,
            1.0)
        + std::clamp(candidate.m_trackOccupancy, 0.0, 1.0)
        + std::clamp(candidate.m_frequencyCoherence, 0.0, 1.0);

    candidate.m_shapeScore =
        (candidate.m_durationOK ? 1.0 : -3.0)
        + (candidate.m_insideUsableBandwidth ? 1.0 : -3.0)
        + (candidate.m_strongLineOK ? 1.0 : 0.0)
        + (candidate.m_boundedBandOK ? 1.0 : 0.0)
        + (candidate.m_spectralEvidenceOK ? 0.5 : -1.0)
        + std::clamp(1.0 - usableDistanceRatio, -1.0, 1.0)
        + std::clamp(1.0 - bandwidthRatio, -1.0, 1.0);

    candidate.m_rejectionPenalty = 0.0;

    if (candidate.m_duplicate) {
        candidate.m_rejectionPenalty += 5.0;
    }

    if (!candidate.m_enoughFrames) {
        candidate.m_rejectionPenalty += 1.5;
    }

    if (candidate.m_integratedSupportDB < -7.0) {
        candidate.m_rejectionPenalty += std::clamp((-7.0 - candidate.m_integratedSupportDB) / 2.0, 0.0, 3.0);
    }

    if (candidate.m_broadbandImpulse) {
        candidate.m_rejectionPenalty += 4.0;
    }

    if (candidate.m_sweepRejected) {
        candidate.m_rejectionPenalty += 2.0 + std::clamp(driftRatio - 1.0, 0.0, 3.0);
    } else {
        candidate.m_shapeScore += std::clamp(1.0 - driftRatio, 0.0, 1.0);
    }

    return candidate.m_signalScore + candidate.m_supportScore + candidate.m_shapeScore - candidate.m_rejectionPenalty;
}

double MeteorDemodSink::weightedQuantile(
    const std::vector<double>& values,
    const std::vector<double>& weights,
    double quantile,
    double fallback)
{
    return weightedQuantile(values, weights, 0, (int) values.size(), quantile, fallback);
}

double MeteorDemodSink::weightedQuantile(
    const std::vector<double>& values,
    const std::vector<double>& weights,
    int begin,
    int end,
    double quantile,
    double fallback)
{
    if (values.empty() || (values.size() != weights.size())) {
        return fallback;
    }

    begin = std::max(0, begin);
    end = std::min((int) values.size(), end);

    if (begin >= end) {
        return fallback;
    }

    std::vector<std::pair<double, double>> samples;
    samples.reserve(end - begin);
    double totalWeight = 0.0;

    for (int i = begin; i < end; i++)
    {
        if (!std::isfinite(values[i]) || !std::isfinite(weights[i]) || (weights[i] <= 0.0)) {
            continue;
        }

        samples.push_back({values[i], weights[i]});
        totalWeight += weights[i];
    }

    if (samples.empty() || (totalWeight <= 0.0)) {
        return fallback;
    }

    std::sort(
        samples.begin(),
        samples.end(),
        [](const std::pair<double, double>& left, const std::pair<double, double>& right)
        {
            return left.first < right.first;
        });

    const double targetWeight = std::clamp(quantile, 0.0, 1.0) * totalWeight;
    double cumulativeWeight = 0.0;

    for (const std::pair<double, double>& sample : samples)
    {
        cumulativeWeight += sample.second;

        if (cumulativeWeight >= targetWeight) {
            return sample.first;
        }
    }

    return samples.back().first;
}

double MeteorDemodSink::estimateSpectralEventDurationSamples(const SpectralEvent& event, double& startSample) const
{
    const int count = (int) event.m_trackSamples.size();

    if ((count <= 0) || (m_spectralHopSize <= 0))
    {
        startSample = (double) event.m_startCenterSample;
        return 1.0;
    }

    const double halfFrameSamples = 0.5 * (double) std::max(m_spectralFrameSize, m_spectralHopSize);
    const double centerSpan = count > 1
        ? (double) (event.m_trackSamples.back() - event.m_trackSamples.front())
        : 0.0;

    startSample = std::max(0.0, (double) event.m_trackSamples.front() - halfFrameSamples);
    return std::max(1.0, centerSpan + 2.0 * halfFrameSamples);
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

bool MeteorDemodSink::ensurePulseFFT(int windowSize)
{
    if (windowSize < 8) {
        return false;
    }

    if (!m_pulseFFT) {
        m_pulseFFT = createMeteorFFT();
    }

    if (m_pulseFFTSize != windowSize)
    {
        m_pulseFFT->configure(windowSize, false);
        m_pulseFFTSize = windowSize;
        makeHannWindow(m_pulseFFTWindow, windowSize);
    }

    return true;
}

bool MeteorDemodSink::computePulseWindowSpectrum(int startIndex, int windowSize, std::vector<double>& binPower, double *windowedEnergy)
{
    if ((startIndex < 0) || (windowSize < 8) || ((startIndex + windowSize) > (int) m_pulseSamples.size())) {
        return false;
    }

    binPower.assign(windowSize, 0.0);

    if (windowedEnergy) {
        *windowedEnergy = 0.0;
    }

    if (!ensurePulseFFT(windowSize) || ((int) m_pulseFFTWindow.size() != windowSize)) {
        return false;
    }

    Complex *fftIn = m_pulseFFT->in();

    for (int i = 0; i < windowSize; i++)
    {
        const Complex sample = m_pulseSamples[startIndex + i] * m_pulseFFTWindow[i];
        fftIn[i] = sample;

        if (windowedEnergy) {
            *windowedEnergy += std::norm(sample);
        }
    }

    m_pulseFFT->transform();
    const Complex *fftOut = m_pulseFFT->out();
    const int halfSize = windowSize / 2;

    for (int bin = -halfSize; bin < halfSize; bin++)
    {
        const int fftIndex = bin < 0 ? bin + windowSize : bin;
        binPower[bin + halfSize] = std::max((double) std::norm(fftOut[fftIndex]), 1e-30);
    }

    return true;
}

bool MeteorDemodSink::estimateWindowPeakFrequency(int startIndex, int windowSize, double& frequency, double& strength, double& prominence)
{
    double windowedEnergy = 0.0;
    std::vector<double> binPower;

    if (!computePulseWindowSpectrum(startIndex, windowSize, binPower, &windowedEnergy)) {
        return false;
    }

    int bestBin = 0;
    double bestMagnitudeSq = 0.0;
    double magnitudeSqSum = 0.0;
    int binCount = 0;

    for (int bin = -windowSize / 2; bin < windowSize / 2; bin++)
    {
        const double magnitudeSq = binPower[bin + windowSize / 2];
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

bool MeteorDemodSink::estimatePulseSpectralBand(int windowSize, int hopSize, double& centerFrequency, double& bandwidth, double& contrastDB)
{
    centerFrequency = 0.0;
    bandwidth = 0.0;
    contrastDB = 0.0;

    if ((windowSize < 8) || ((int) m_pulseSamples.size() < windowSize)) {
        return false;
    }

    std::vector<double> binPower(windowSize, 0.0);
    std::vector<double> windowPower;
    int windowCount = 0;

    for (int start = 0; (start + windowSize) <= (int) m_pulseSamples.size(); start += hopSize)
    {
        if (!computePulseWindowSpectrum(start, windowSize, windowPower, nullptr)) {
            continue;
        }

        for (int i = 0; i < windowSize; i++) {
            binPower[i] += windowPower[i];
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
    double& spectralBandContrastDB)
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

    if (isDuplicateDetection(m_pulseStartSample, endSample))
    {
        m_pulseActive = false;
        m_pulseSamples.clear();
        return;
    }

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
        && (sweepScore >= m_detectorTunables.m_powerSweepMinR2)
        && ((std::fabs(frequencySpan) > m_settings.m_maxFrequencyDrift)
            || (std::fabs(frequencyDrift) > m_settings.m_maxFrequencyDrift));
    const bool driftOK = !sweepRejected;
    const double compactFrequencyLimit = std::max(10.0, std::min(50.0, (double) m_settings.m_channelSampleRate * 0.05));
    const double stableFrequencyLimit = std::max(
        compactFrequencyLimit,
        m_resolvedDetectorTunables.m_powerStableBandwidthHz);
    const bool stableLineOK = (spectralProminence >= 7.5)
        && (frequencyConcentration >= 0.60)
        && (std::fabs(frequencySpan) <= stableFrequencyLimit)
        && (std::fabs(frequencyDrift) <= stableFrequencyLimit);
    const bool boundedBandOK = (spectralProminence >= m_detectorTunables.m_powerBoundedMinProminenceDB)
        && (spectralBandContrastDB >= 3.0)
        && (spectralBandwidth >= compactFrequencyLimit)
        && (spectralBandwidth <= stableFrequencyLimit)
        && (std::fabs(frequencyDrift) <= m_settings.m_maxFrequencyDrift);
    const double peakAboveBackgroundDB = 10.0 * std::log10(std::max(m_pulsePeakPower, 1e-20) / std::max(m_noiseFloor, 1e-20));
    const bool compactClippedMeteor = forceRejected
        && (durationMS >= (double) m_settings.m_maxDurationMS)
        && driftOK
        && boundedBandOK
        && stableLineOK
        && (peakAboveBackgroundDB >= 12.0);
    const bool veryShortLineOK = (durationS <= 0.25)
        && (peakAboveBackgroundDB >= 6.0)
        && (spectralProminence >= 4.0)
        && (frequencyConcentration >= 0.60)
        && (spectralBandContrastDB >= 8.0)
        && (spectralBandwidth <= stableFrequencyLimit)
        && (std::fabs(frequencyDrift) <= stableFrequencyLimit);
    const bool spectralEvidenceOK = stableLineOK || boundedBandOK || veryShortLineOK;
    const bool strongShortLine = stableLineOK && (peakAboveBackgroundDB >= 12.0);
    const bool strongCoherentLine = stableLineOK
        && (peakAboveBackgroundDB >= m_detectorTunables.m_powerStrongCoherentMinPeakDB)
        && (spectralProminence >= m_detectorTunables.m_powerStrongCoherentMinProminenceDB)
        && (frequencyConcentration >= 0.90)
        && (spectralBandContrastDB >= 3.5);
    const bool shortStandaloneLine = stableLineOK && !strongShortLine && !veryShortLineOK && !compactClippedMeteor && (durationS < 0.5);
    const bool directAccepted = compactClippedMeteor;
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
    const bool validatesPendingBroadPulse = broadValidationLineOK
        && (durationS <= 1.0)
        && m_pendingBroadPulse.m_valid
        && (m_pulseStartSample >= m_pendingBroadPulse.m_endSample)
        && ((m_pulseStartSample - m_pendingBroadPulse.m_endSample) <= (quint64) broadValidationGapSamples);
    const bool broadbandContaminatedPulse = overlapsBroadbandInterference(m_pulseStartSample, endSample);
    const bool accepted = (directAccepted || validatesPendingBroadPulse) && !broadbandContaminatedPulse;

    if (accepted
        || broadPulseCandidate
        || (durationOK && driftOK && (strongShortLine || strongCoherentLine)))
    {
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
                 << " strongCoherentLine:" << strongCoherentLine
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
    }

    const bool powerLineFallback = !accepted
        && (strongShortLine || strongCoherentLine)
        && driftOK
        && (peakAboveBackgroundDB >= 12.0)
        && (frequencyConcentration >= 0.50)
        && ((spectralBandContrastDB >= 6.0) || strongCoherentLine)
        && !broadbandContaminatedPulse
        && (std::fabs(centerFrequency) <= m_resolvedDetectorTunables.m_usableBandwidthHz);

    auto emitPowerLineFallback = [this, centerFrequency, reportedFrequencySpan, frequencyDrift]() -> void
    {
        PulseReport report;
        report.m_valid = true;
        report.m_startSample = m_pulsePeakSample;
        report.m_endSample = m_pulsePeakSample;
        report.m_dateTimeUtc = sampleCounterToDateTimeUtc(report.m_startSample);
        report.m_peakPower = m_pulsePeakPower;
        report.m_backgroundPower = m_noiseFloor;
        report.m_totalPower = estimatePulseTotalPower(report.m_startSample, report.m_endSample, report.m_backgroundPower);
        report.m_durationS = 1.0 / (double) std::max(1, m_settings.m_channelSampleRate);
        report.m_centerFrequency = centerFrequency;
        report.m_frequencySpan = reportedFrequencySpan;
        report.m_frequencyDrift = frequencyDrift;
        report.m_allowComponentMerge = false;

        if (estimatePulseBandEnvelope(report)
            && (1000.0 * report.m_durationS >= (double) m_settings.m_minDurationMS)
            && (1000.0 * report.m_durationS <= (double) m_settings.m_maxDurationMS))
        {
            if (!isDuplicateDetection(report.m_startSample, report.m_endSample, report.m_centerFrequency, report.m_frequencySpan)) {
                queueSpectralComponentReport(report);
            } else {
                qDebug() << "MeteorDemodSink::finishPulse: duplicate power-line detection suppressed";
            }
        }
    };

    if (powerLineFallback) {
        emitPowerLineFallback();
    }

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
            const quint64 maximumDurationSamples = (quint64) std::max(
                1,
                m_settings.m_maxDurationMS * std::max(1, m_settings.m_channelSampleRate) / 1000);
            report.m_valid = true;
            report.m_dateTimeUtc = m_pulseStartDateTimeUtc;
            report.m_startSample = m_pulseStartSample;
            report.m_endSample = compactClippedMeteor
                ? m_pulseStartSample + maximumDurationSamples - 1
                : endSample;
            report.m_peakPower = m_pulsePeakPower;
            report.m_backgroundPower = m_noiseFloor;
            report.m_totalPower = estimatePulseTotalPower(report.m_startSample, report.m_endSample, report.m_backgroundPower);
            report.m_durationS = (double) (report.m_endSample - report.m_startSample + 1)
                / (double) std::max(1, m_settings.m_channelSampleRate);
            report.m_centerFrequency = centerFrequency;
            report.m_frequencySpan = reportedFrequencySpan;
            report.m_frequencyDrift = frequencyDrift;
            report.m_truncated = compactClippedMeteor;
        }

        report.m_allowComponentMerge = false;

        if (!isDuplicateDetection(report.m_startSample, report.m_endSample, report.m_centerFrequency, report.m_frequencySpan)) {
            queueSpectralComponentReport(report);
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
        m_pendingBroadPulse.m_totalPower = estimatePulseTotalPower(m_pendingBroadPulse.m_startSample, m_pendingBroadPulse.m_endSample, m_pendingBroadPulse.m_backgroundPower);
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
    const quint64 sameEventGapSamples = (quint64) std::max(
        m_spectralFrameSize * 4,
        3 * std::max(1, m_settings.m_channelSampleRate) / 4);
    const quint64 similarEventGapSamples = (quint64) std::max(1, m_settings.m_channelSampleRate) * 5;
    const quint64 compactEventSamples = (quint64) std::max(1, m_settings.m_channelSampleRate) * 2;

    for (const DetectionRange& range : m_recentDetectionRanges)
    {
        const quint64 rangeLength = range.m_endSample - range.m_startSample + 1;
        const bool overlapsInTime = (endSample >= range.m_startSample) && (startSample <= range.m_endSample);
        const bool rangeHasFrequencyRange = hasFrequencyRange && (range.m_highFrequency > range.m_lowFrequency);
        double frequencyOverlapRatio = 0.0;
        double centerDistance = std::numeric_limits<double>::max();
        double frequencyGap = std::numeric_limits<double>::max();
        double width = std::max(1.0, highFrequency - lowFrequency);
        double rangeWidth = std::max(1.0, range.m_highFrequency - range.m_lowFrequency);

        if (rangeHasFrequencyRange)
        {
            const double overlapLow = std::max(lowFrequency, range.m_lowFrequency);
            const double overlapHigh = std::min(highFrequency, range.m_highFrequency);
            const double overlapWidth = std::max(0.0, overlapHigh - overlapLow);
            frequencyOverlapRatio = overlapWidth / std::min(width, rangeWidth);
            centerDistance = std::fabs(centerFrequency - 0.5 * (range.m_lowFrequency + range.m_highFrequency));
            frequencyGap = overlapWidth > 0.0
                ? 0.0
                : std::max(range.m_lowFrequency - highFrequency, lowFrequency - range.m_highFrequency);
        }

        if (overlapsInTime)
        {
            const quint64 overlapStart = std::max(startSample, range.m_startSample);
            const quint64 overlapEnd = std::min(endSample, range.m_endSample);
            const quint64 overlapLength = overlapEnd - overlapStart + 1;
            const quint64 shorterLength = std::max<quint64>(1, std::min(detectionLength, rangeLength));

            if ((double) overlapLength >= 0.50 * (double) shorterLength)
            {
                if (!rangeHasFrequencyRange
                    || (frequencyOverlapRatio >= m_detectorTunables.m_duplicateFrequencyOverlapFraction))
                {
                    return true;
                }
            }
        }

        if (!rangeHasFrequencyRange) {
            continue;
        }

        const quint64 gapSamples = endSample < range.m_startSample
            ? range.m_startSample - endSample
            : (startSample > range.m_endSample ? startSample - range.m_endSample : 0);
        const bool compactEventPair = (detectionLength <= compactEventSamples) && (rangeLength <= compactEventSamples);
        const bool verySimilarFrequency = (frequencyOverlapRatio >= m_detectorTunables.m_duplicateStrongFrequencyOverlapFraction)
            && (centerDistance <= std::max(frequencyPadding, 0.20 * std::min(width, rangeWidth)));
        const bool closeInTime = compactEventPair
            && ((gapSamples <= sameEventGapSamples)
                || (verySimilarFrequency && (gapSamples <= similarEventGapSamples)));
        const bool adjacentFrequency = (frequencyGap <= frequencyPadding)
            && (centerDistance <= std::max(25.0, 0.5 * std::max(width, rangeWidth)));
        const bool overlapsInFrequency = (frequencyOverlapRatio >= m_detectorTunables.m_duplicateFrequencyOverlapFraction)
            || (centerDistance <= frequencyPadding)
            || adjacentFrequency;

        if (closeInTime && overlapsInFrequency)
        {
            qDebug() << "MeteorDemodSink::isDuplicateDetection: close spectral duplicate suppressed"
                     << " gapSamples:" << gapSamples
                     << " startSample:" << startSample
                     << " endSample:" << endSample
                     << " frequencyLow:" << lowFrequency
                     << " frequencyHigh:" << highFrequency
                     << " frequencyOverlapRatio:" << frequencyOverlapRatio
                     << " centerDistance:" << centerDistance
                     << " frequencyGap:" << frequencyGap
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

MeteorDemodSink::AssociationResult MeteorDemodSink::emitOrDeferSpectralReport(const PulseReport& report)
{
    if (!report.m_valid) {
        return {};
    }

    PulseReport outputReport = reportWithRobustFrequency(report);
    refineSpectralComponentEnvelope(outputReport);
    const double durationMS = 1000.0 * outputReport.m_durationS;

    if ((durationMS < (double) m_settings.m_minDurationMS)
        || (durationMS > (double) m_settings.m_maxDurationMS))
    {
        return {0, QStringLiteral("duration-rejected")};
    }

    const double duplicateFrequencySpan = outputReport.m_duplicateFrequencySpan > 0.0
        ? outputReport.m_duplicateFrequencySpan
        : outputReport.m_frequencySpan;

    if (!isDuplicateDetection(
        outputReport.m_startSample,
        outputReport.m_endSample,
        outputReport.m_centerFrequency,
        duplicateFrequencySpan))
    {
        return queueSpectralComponentReport(outputReport);
    }

    return {0, QStringLiteral("duplicate-rejected")};
}

MeteorDemodSink::AssociationResult MeteorDemodSink::queueSpectralComponentReport(const PulseReport& report)
{
    const quint64 mergeGapSamples = (quint64) std::max(1, m_settings.m_channelSampleRate / 4);
    const quint64 maxEvidenceGapSamples = (quint64) std::max(
        1.0,
        m_detectorTunables.m_continuationMaxEvidenceGapS
            * (double) std::max(1, m_settings.m_channelSampleRate));

    for (ActiveMeteorEvent& event : m_activeMeteorEvents)
    {
        PulseReport& pending = event.m_report;
        const quint64 gapSamples = report.m_endSample < pending.m_startSample
            ? pending.m_startSample - report.m_endSample
            : (report.m_startSample > pending.m_endSample ? report.m_startSample - pending.m_endSample : 0);
        const double reportSpan = std::max(std::fabs(report.m_duplicateFrequencySpan), std::fabs(report.m_frequencySpan));
        const double pendingSpan = std::max(std::fabs(pending.m_duplicateFrequencySpan), std::fabs(pending.m_frequencySpan));
        const bool broadTemporalDuplicate = (gapSamples == 0)
            && (std::max(reportSpan, pendingSpan) >= 0.20 * (double) std::max(1, m_settings.m_channelSampleRate));
        const bool evidenceBridge = (gapSamples <= mergeGapSamples)
            || ((report.m_startSample <= event.m_lastEvidenceSample + maxEvidenceGapSamples)
                && (event.m_lastEvidenceSample >= event.m_lastComponentSample));

        const bool hasSpectralPrimary = report.m_spectralParentEligible
            || pending.m_spectralParentEligible;

        if ((!report.m_allowComponentMerge || !pending.m_allowComponentMerge)
            && !broadTemporalDuplicate
            && !hasSpectralPrimary)
        {
            continue;
        }

        if (!evidenceBridge
            || (!reportsFrequencyCompatible(report, pending) && !broadTemporalDuplicate))
        {
            continue;
        }

        const bool reportHasClearlyBroaderMorphology = (reportSpan > 1.75 * std::max(1.0, pendingSpan))
            && (report.m_confidence >= 0.0);
        const bool pendingHasClearlyBroaderMorphology = (pendingSpan > 1.75 * std::max(1.0, reportSpan))
            && (pending.m_confidence >= 0.0);
        const bool reportIsPrimary = (report.m_spectralParentEligible && !pending.m_spectralParentEligible)
            || ((report.m_spectralParentEligible == pending.m_spectralParentEligible)
                && (reportHasClearlyBroaderMorphology
            || (!pendingHasClearlyBroaderMorphology
                && ((report.m_componentSupportDB > pending.m_componentSupportDB + 0.25)
                    || ((std::fabs(report.m_componentSupportDB - pending.m_componentSupportDB) <= 0.25)
                        && ((report.m_confidence > pending.m_confidence + 0.5)
                            || ((std::fabs(report.m_confidence - pending.m_confidence) <= 0.5)
                                && (report.m_durationS > pending.m_durationS))))))));
        const quint64 mergedStartSample = std::min(report.m_startSample, pending.m_startSample);
        const quint64 mergedEndSample = std::max(report.m_endSample, pending.m_endSample);

        if (reportIsPrimary)
        {
            pending.m_centerFrequency = report.m_centerFrequency;
            pending.m_frequencySpan = report.m_frequencySpan;
            pending.m_frequencyDrift = report.m_frequencyDrift;
            pending.m_reportFrequencySpan = report.m_reportFrequencySpan;
            pending.m_duplicateFrequencySpan = report.m_duplicateFrequencySpan;
            pending.m_hasRobustFrequency = report.m_hasRobustFrequency;
            pending.m_robustCenterFrequency = report.m_robustCenterFrequency;
            pending.m_robustFrequencySpan = report.m_robustFrequencySpan;
            pending.m_robustFrequencyDrift = report.m_robustFrequencyDrift;
            pending.m_backgroundPower = report.m_backgroundPower;
        }

        pending.m_startSample = mergedStartSample;
        pending.m_endSample = mergedEndSample;
        pending.m_dateTimeUtc = sampleCounterToDateTimeUtc(mergedStartSample);
        pending.m_durationS = (double) (mergedEndSample - mergedStartSample + 1)
            / (double) std::max(1, m_settings.m_channelSampleRate);
        pending.m_peakPower = std::max(pending.m_peakPower, report.m_peakPower);
        pending.m_totalPower += report.m_totalPower;
        pending.m_confidence = std::max(pending.m_confidence, report.m_confidence);
        pending.m_componentSupportDB = std::max(pending.m_componentSupportDB, report.m_componentSupportDB);
        pending.m_truncated = pending.m_truncated || report.m_truncated;
        pending.m_spectralParentEligible = pending.m_spectralParentEligible
            || report.m_spectralParentEligible;
        event.m_lastEvidenceSample = std::max(event.m_lastEvidenceSample, report.m_endSample);
        event.m_lastComponentSample = std::max(event.m_lastComponentSample, report.m_endSample);
        event.m_componentCount++;
        event.m_spectralComponentCount += report.m_spectralParentEligible ? 1 : 0;
        event.m_strong = event.m_strong
            || (10.0 * std::log10(std::max(pending.m_peakPower, 1e-20)
                / std::max(pending.m_backgroundPower, 1e-20))
                >= m_detectorTunables.m_continuationStrongPeakDB);

        MeteorEventObservation observation;
        observation.m_sample = report.m_startSample + (report.m_endSample - report.m_startSample) / 2;
        observation.m_centerFrequency = report.m_centerFrequency;
        observation.m_lowFrequency = report.m_centerFrequency - 0.5 * report.m_frequencySpan;
        observation.m_highFrequency = report.m_centerFrequency + 0.5 * report.m_frequencySpan;
        observation.m_weight = std::max(report.m_peakPower - report.m_backgroundPower, 1e-20);
        observation.m_peakPower = report.m_peakPower;
        observation.m_backgroundPower = report.m_backgroundPower;
        addMeteorEventObservation(event, observation);

        if (pending.m_hasDisplaySamples || report.m_hasDisplaySamples)
        {
            pending.m_hasDisplaySamples = true;
            pending.m_displayStartSample = mergedStartSample;
            pending.m_displayEndSample = mergedEndSample;
        }

        scheduleNextComponentFlush();

        qDebug() << "MeteorDemodSink::queueSpectralComponentReport: attached component"
                 << " parentEventId:" << event.m_id
                 << " componentCount:" << event.m_componentCount
                 << " gapSamples:" << gapSamples
                 << " startSample:" << report.m_startSample
                 << " endSample:" << report.m_endSample;
        return {event.m_id, QStringLiteral("attached")};
    }

    if ((int) m_activeMeteorEvents.size() >= m_detectorTunables.m_maxActiveMeteorEvents)
    {
        auto oldest = std::min_element(
            m_activeMeteorEvents.begin(),
            m_activeMeteorEvents.end(),
            [](const ActiveMeteorEvent& left, const ActiveMeteorEvent& right) {
                return left.m_lastEvidenceSample < right.m_lastEvidenceSample;
            });

        if (oldest != m_activeMeteorEvents.end())
        {
            finalizeActiveMeteorEvent(*oldest);
            m_activeMeteorEvents.erase(oldest);
        }
    }

    ActiveMeteorEvent event;
    event.m_id = m_nextMeteorEventId++;
    event.m_report = report;
    event.m_lastEvidenceSample = report.m_endSample;
    event.m_lastComponentSample = report.m_endSample;
    event.m_componentCount = 1;
    event.m_spectralComponentCount = report.m_spectralParentEligible ? 1 : 0;
    event.m_strong = 10.0 * std::log10(std::max(report.m_peakPower, 1e-20)
            / std::max(report.m_backgroundPower, 1e-20))
        >= m_detectorTunables.m_continuationStrongPeakDB;

    MeteorEventObservation observation;
    observation.m_sample = report.m_startSample + (report.m_endSample - report.m_startSample) / 2;
    observation.m_centerFrequency = report.m_centerFrequency;
    observation.m_lowFrequency = report.m_centerFrequency - 0.5 * report.m_frequencySpan;
    observation.m_highFrequency = report.m_centerFrequency + 0.5 * report.m_frequencySpan;
    observation.m_weight = std::max(report.m_peakPower - report.m_backgroundPower, 1e-20);
    observation.m_peakPower = report.m_peakPower;
    observation.m_backgroundPower = report.m_backgroundPower;
    addMeteorEventObservation(event, observation);
    const quint64 parentEventId = event.m_id;
    m_activeMeteorEvents.push_back(std::move(event));
    scheduleNextComponentFlush();

    qDebug() << "MeteorDemodSink::queueSpectralComponentReport: created parent"
             << " parentEventId:" << parentEventId
             << " startSample:" << report.m_startSample
             << " endSample:" << report.m_endSample;
    return {parentEventId, QStringLiteral("created")};
}

void MeteorDemodSink::scheduleNextComponentFlush()
{
    m_nextComponentFlushSample = std::numeric_limits<quint64>::max();

    const quint64 maxDurationSamples = (quint64) std::max(
        1,
        m_settings.m_maxDurationMS * std::max(1, m_settings.m_channelSampleRate) / 1000);

    for (const ActiveMeteorEvent& event : m_activeMeteorEvents)
    {
        const bool continuationEligible = event.m_report.m_spectralParentEligible
            && ((event.m_spectralComponentCount >= 2) || (event.m_report.m_durationS >= 1.0));
        const double holdS = !continuationEligible
            ? 0.5
            : (event.m_strong
                ? m_detectorTunables.m_continuationStrongHoldS
                : m_detectorTunables.m_continuationOrdinaryHoldS);
        const quint64 holdSamples = (quint64) std::max(
            1.0,
            holdS * (double) std::max(1, m_settings.m_channelSampleRate));
        m_nextComponentFlushSample = std::min(
            m_nextComponentFlushSample,
            std::min(
                event.m_lastEvidenceSample + holdSamples,
                event.m_report.m_startSample + maxDurationSamples));
    }
}

void MeteorDemodSink::flushPendingComponentReports(bool force)
{
    if (m_activeMeteorEvents.empty())
    {
        m_nextComponentFlushSample = std::numeric_limits<quint64>::max();
        return;
    }

    if (!force && (m_sampleCounter <= m_nextComponentFlushSample)) {
        return;
    }

    std::vector<ActiveMeteorEvent> remainingEvents;

    m_nextComponentFlushSample = std::numeric_limits<quint64>::max();

    const quint64 maxDurationSamples = (quint64) std::max(
        1,
        m_settings.m_maxDurationMS * std::max(1, m_settings.m_channelSampleRate) / 1000);

    for (ActiveMeteorEvent& event : m_activeMeteorEvents)
    {
        const bool continuationEligible = event.m_report.m_spectralParentEligible
            && ((event.m_spectralComponentCount >= 2) || (event.m_report.m_durationS >= 1.0));
        const double holdS = !continuationEligible
            ? 0.5
            : (event.m_strong
                ? m_detectorTunables.m_continuationStrongHoldS
                : m_detectorTunables.m_continuationOrdinaryHoldS);
        const quint64 holdSamples = (quint64) std::max(
            1.0,
            holdS * (double) std::max(1, m_settings.m_channelSampleRate));
        const bool durationLimitReached = event.m_lastEvidenceSample
            >= event.m_report.m_startSample + maxDurationSamples;

        if (!force
            && !durationLimitReached
            && (m_sampleCounter <= event.m_lastEvidenceSample + holdSamples))
        {
            remainingEvents.push_back(std::move(event));
            continue;
        }

        if (durationLimitReached)
        {
            event.m_report.m_endSample = clipDetectionEndSample(
                event.m_report.m_startSample,
                event.m_report.m_endSample,
                maxDurationSamples,
                event.m_report.m_truncated);
            event.m_report.m_displayEndSample = event.m_report.m_endSample;
        }

        finalizeActiveMeteorEvent(event);
    }

    m_activeMeteorEvents.swap(remainingEvents);
    scheduleNextComponentFlush();
}

quint64 MeteorDemodSink::clipDetectionEndSample(
    quint64 startSample,
    quint64 endSample,
    quint64 maximumDurationSamples,
    bool& truncated)
{
    maximumDurationSamples = std::max<quint64>(1, maximumDurationSamples);
    const quint64 maximumEndSample = startSample + maximumDurationSamples - 1;
    truncated = endSample > maximumEndSample;
    return truncated ? maximumEndSample : std::max(startSample, endSample);
}

void MeteorDemodSink::updateReportFromActiveMeteorEvent(ActiveMeteorEvent& event) const
{
    PulseReport& report = event.m_report;
    report.m_endSample = std::max(report.m_startSample, report.m_endSample);
    report.m_dateTimeUtc = sampleCounterToDateTimeUtc(report.m_startSample);
    report.m_durationS = (double) (report.m_endSample - report.m_startSample + 1)
        / (double) std::max(1, m_settings.m_channelSampleRate);
    report.m_hasDisplaySamples = true;
    report.m_displayStartSample = report.m_startSample;
    report.m_displayEndSample = report.m_endSample;

    if (report.m_spectralParentEligible && (event.m_spectralComponentCount > 1))
    {
        const double acceptedCoreSpan = std::max({
            std::fabs(report.m_frequencySpan),
            std::fabs(report.m_reportFrequencySpan),
            std::fabs(report.m_robustFrequencySpan)});
        std::vector<double> centers;
        std::vector<double> lows;
        std::vector<double> highs;
        std::vector<double> weights;
        const double maxPersistentBandwidth = m_resolvedDetectorTunables.m_stableBandwidthHz;

        for (const MeteorEventObservation& observation : event.m_observations)
        {
            if (observation.m_broadband
                || ((observation.m_highFrequency - observation.m_lowFrequency) > maxPersistentBandwidth))
            {
                continue;
            }

            centers.push_back(observation.m_centerFrequency);
            lows.push_back(observation.m_lowFrequency);
            highs.push_back(observation.m_highFrequency);
            weights.push_back(std::max(observation.m_weight, 1e-30));
        }

        if (!centers.empty())
        {
            const double center = weightedQuantile(
                centers,
                weights,
                0.5,
                report.m_centerFrequency);
            const double low = weightedQuantile(
                lows,
                weights,
                m_detectorTunables.m_parentFrequencyLowQuantile,
                center - 0.5 * report.m_frequencySpan);
            const double high = weightedQuantile(
                highs,
                weights,
                m_detectorTunables.m_parentFrequencyHighQuantile,
                center + 0.5 * report.m_frequencySpan);
            const double binWidth = (double) std::max(1, m_settings.m_channelSampleRate)
                / (double) std::max(1, m_spectralFrameSize);
            const double span = std::max({binWidth, high - low, acceptedCoreSpan});

            double drift = report.m_frequencyDrift;

            if (centers.size() >= 4)
            {
                const int edgeCount = std::max(1, (int) centers.size() / 4);
                const double firstFrequency = weightedQuantile(
                    centers,
                    weights,
                    0,
                    edgeCount,
                    0.5,
                    centers.front());
                const double lastFrequency = weightedQuantile(
                    centers,
                    weights,
                    (int) centers.size() - edgeCount,
                    (int) centers.size(),
                    0.5,
                    centers.back());
                drift = std::clamp(lastFrequency - firstFrequency, -span, span);
            }

            report.m_centerFrequency = center;
            report.m_frequencySpan = span;
            report.m_frequencyDrift = drift;
            report.m_reportFrequencySpan = span;
            report.m_duplicateFrequencySpan = span;
            report.m_hasRobustFrequency = true;
            report.m_robustCenterFrequency = center;
            report.m_robustFrequencySpan = span;
            report.m_robustFrequencyDrift = drift;
        }
    }

}

void MeteorDemodSink::finalizeActiveMeteorEvent(ActiveMeteorEvent& event)
{
    updateReportFromActiveMeteorEvent(event);
    PulseReport& report = event.m_report;

    if (m_detectorTunables.m_enableSettledParentReanalysis)
    {
        const quint64 originalStartSample = report.m_startSample;
        const quint64 originalEndSample = report.m_endSample;

        if (estimatePulseBandEnvelope(report))
        {
            report.m_totalPower = estimatePulseTotalPower(
                report.m_startSample,
                report.m_endSample,
                report.m_backgroundPower);
            qDebug() << "MeteorDemodSink::finalizeActiveMeteorEvent: settled envelope expanded"
                     << " parentEventId:" << event.m_id
                     << " fromStartSample:" << originalStartSample
                     << " fromEndSample:" << originalEndSample
                     << " toStartSample:" << report.m_startSample
                     << " toEndSample:" << report.m_endSample;
        }
    }

    const double durationMS = 1000.0 * report.m_durationS;

    qDebug() << "MeteorDemodSink::finalizeActiveMeteorEvent:"
             << " parentEventId:" << event.m_id
             << " components:" << event.m_componentCount
             << " spectralComponents:" << event.m_spectralComponentCount
             << " observations:" << event.m_observations.size()
             << " extendedByContinuation:" << event.m_extendedByContinuation
             << " durationS:" << report.m_durationS
             << " centerFrequency:" << report.m_centerFrequency
             << " frequencySpan:" << report.m_frequencySpan
             << " frequencyDrift:" << report.m_frequencyDrift
             << " truncated:" << report.m_truncated;

    if ((durationMS < (double) m_settings.m_minDurationMS)
        || (durationMS > (double) m_settings.m_maxDurationMS + 1.0))
    {
        return;
    }

    const double duplicateFrequencySpan = report.m_duplicateFrequencySpan > 0.0
        ? report.m_duplicateFrequencySpan
        : report.m_frequencySpan;

    if (!isDuplicateDetection(
        report.m_startSample,
        report.m_endSample,
        report.m_centerFrequency,
        duplicateFrequencySpan))
    {
        if (m_diagnosticCaptureCallback)
        {
            ComplexVector samples;

            if (copyDetectionSamples(report.m_startSample, report.m_endSample, samples)) {
                m_diagnosticCaptureCallback(event.m_id, report, samples);
            }
        }

        emitDetectionReport(
            report,
            report.m_spectralParentEligible ? "spectral-parent" : "power-parent");
    }
}

MeteorDemodSink::PulseReport MeteorDemodSink::reportWithRobustFrequency(const PulseReport& report) const
{
    PulseReport output = report;

    if (output.m_duplicateFrequencySpan <= 0.0) {
        output.m_duplicateFrequencySpan = output.m_frequencySpan;
    }

    if (output.m_hasRobustFrequency)
    {
        output.m_centerFrequency = output.m_robustCenterFrequency;
        output.m_frequencyDrift = output.m_robustFrequencyDrift;
    }

    if (output.m_reportFrequencySpan > 0.0) {
        output.m_frequencySpan = std::max(output.m_frequencySpan, output.m_reportFrequencySpan);
    }

    return output;
}

bool MeteorDemodSink::estimatePulseBandEnvelope(PulseReport& report) const
{
    return refineBandEnvelope(report, false);
}

bool MeteorDemodSink::refineSpectralComponentEnvelope(PulseReport& report) const
{
    return refineBandEnvelope(report, true);
}

bool MeteorDemodSink::refineBandEnvelope(PulseReport& report, bool allowShrink) const
{
    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);
    int frameSize = std::clamp(sampleRate / 4, 64, 256);

    if ((frameSize % 2) != 0) {
        frameSize++;
    }

    if ((m_detectionSampleRingCount < frameSize) || (report.m_endSample < report.m_startSample)) {
        return false;
    }

    const int hopSize = std::max(1, frameSize / 8);
    const double binWidth = (double) sampleRate / (double) frameSize;
    const double halfBandwidth = std::max({0.5 * std::fabs(report.m_frequencySpan), 20.0, 2.0 * binWidth});
    const double twoPi = 2.0 * std::acos(-1.0);
    std::vector<Real> window;
    std::vector<double> strengths;
    std::vector<quint64> centerSamples;
    std::vector<double> probeFrequencies;
    std::vector<std::vector<Complex>> probeOscillators;
    const quint64 reportCenterSample = report.m_startSample + (report.m_endSample - report.m_startSample) / 2;
    const quint64 historyStartSample = m_detectionSampleRingStartSample;
    const quint64 historyEndSample = historyStartSample + (quint64) m_detectionSampleRingCount - 1;
    const quint64 localRadiusSamples = (quint64) sampleRate * 4;
    const quint64 requestedStartSample = report.m_startSample > localRadiusSamples + (quint64) frameSize
        ? report.m_startSample - localRadiusSamples - (quint64) frameSize
        : 0;
    const quint64 scanStartSample = std::max(historyStartSample, requestedStartSample);
    const quint64 scanEndSample = std::min(
        historyEndSample,
        report.m_endSample + localRadiusSamples + (quint64) frameSize);
    ComplexVector detectionSamples;

    if (!copyDetectionSamples(scanStartSample, scanEndSample, detectionSamples)
        || ((int) detectionSamples.size() < frameSize))
    {
        return false;
    }

    const int scanStart = 0;
    const int scanEnd = (int) detectionSamples.size();
    const double maxUsableFrequency = m_resolvedDetectorTunables.m_usableBandwidthHz;

    auto addProbeFrequency = [&probeFrequencies, maxUsableFrequency](double frequency)
    {
        if (std::fabs(frequency) > maxUsableFrequency) {
            return;
        }

        for (double existingFrequency : probeFrequencies)
        {
            if (std::fabs(existingFrequency - frequency) < 1.0) {
                return;
            }
        }

        probeFrequencies.push_back(frequency);
    };

    addProbeFrequency(report.m_centerFrequency);
    addProbeFrequency(report.m_centerFrequency - 0.5 * halfBandwidth);
    addProbeFrequency(report.m_centerFrequency + 0.5 * halfBandwidth);

    if (halfBandwidth > 60.0)
    {
        addProbeFrequency(report.m_centerFrequency
            - m_detectorTunables.m_frequencyRefinementOuterProbeFraction * halfBandwidth);
        addProbeFrequency(report.m_centerFrequency
            + m_detectorTunables.m_frequencyRefinementOuterProbeFraction * halfBandwidth);
    }

    if (probeFrequencies.empty()) {
        return false;
    }

    makeHannWindow(window, frameSize);

    for (double frequency : probeFrequencies)
    {
        std::vector<Complex> oscillator(frameSize);
        const double phaseStep = -twoPi * frequency / (double) sampleRate;
        const Complex rotation((Real) std::cos(phaseStep), (Real) std::sin(phaseStep));
        Complex phase(1.0f, 0.0f);

        for (int i = 0; i < frameSize; i++)
        {
            oscillator[i] = phase;
            phase *= rotation;
        }

        probeOscillators.push_back(std::move(oscillator));
    }

    for (int start = scanStart; (start + frameSize) <= scanEnd; start += hopSize)
    {
        double bandPower = 0.0;

        for (const std::vector<Complex>& oscillator : probeOscillators)
        {
            Complex sum(0.0f, 0.0f);

            for (int i = 0; i < frameSize; i++)
            {
                const Complex windowedSample = detectionSamples[start + i] * window[i];
                sum += windowedSample * oscillator[i];
            }

            bandPower += std::norm(sum);
        }

        strengths.push_back(std::max(bandPower, 1e-30));
        centerSamples.push_back(scanStartSample + (quint64) start + (quint64) (frameSize / 2));
    }

    if (strengths.empty()) {
        return false;
    }

    std::vector<double> sortedStrengths = strengths;
    std::sort(sortedStrengths.begin(), sortedStrengths.end());
    const double floorStrength = std::max(sortedStrengths[sortedStrengths.size() / 5], 1e-30);
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

    const double threshold = allowShrink
        ? std::max(1.6 * floorStrength, floorStrength + 0.003 * (peakStrength - floorStrength))
        : floorStrength + 0.03 * (peakStrength - floorStrength);
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

    const double minDurationS = (double) m_settings.m_minDurationMS / 1000.0;
    const double maxDurationS = (double) m_settings.m_maxDurationMS / 1000.0;
    const bool usefulRefinement = allowShrink || (durationS > report.m_durationS * 1.5);

    if ((endSample <= startSample)
        || !usefulRefinement
        || (durationS < minDurationS)
        || (durationS > maxDurationS))
    {
        return false;
    }

    report.m_dateTimeUtc = sampleCounterToDateTimeUtc(startSample);
    report.m_startSample = startSample;
    report.m_endSample = endSample;
    if (report.m_hasDisplaySamples)
    {
        const quint64 durationSamples = endSample - startSample + 1;
        const quint64 displayCenterSample = report.m_displayStartSample + (report.m_displayEndSample - report.m_displayStartSample) / 2;
        report.m_displayStartSample = displayCenterSample > (durationSamples / 2)
            ? displayCenterSample - (durationSamples / 2)
            : 0;
        report.m_displayEndSample = report.m_displayStartSample + durationSamples - 1;
    }
    double envelopePeakPower = report.m_peakPower;
    const int firstSampleIndex = std::max(0, (int) (startSample - scanStartSample));
    const int lastSampleIndex = std::min((int) detectionSamples.size() - 1, (int) (endSample - scanStartSample));

    for (int i = firstSampleIndex; i <= lastSampleIndex; i++) {
        envelopePeakPower = std::max(envelopePeakPower, (double) std::norm(detectionSamples[i]));
    }

    report.m_peakPower = envelopePeakPower;

    if (report.m_totalPower <= 0.0) {
        report.m_totalPower = estimatePulseTotalPower(report.m_startSample, report.m_endSample, report.m_backgroundPower);
    }

    report.m_durationS = durationS;
    return true;
}

void MeteorDemodSink::appendDetectionSample(const Complex& sample)
{
    const int capacity = (int) m_detectionSampleRing.size();

    if (capacity <= 0) {
        return;
    }

    if (m_detectionSampleRingCount == 0)
    {
        m_detectionSampleRingStart = 0;
        m_detectionSampleRingStartSample = m_sampleCounter;
    }

    if (m_detectionSampleRingCount < capacity)
    {
        const int index = (m_detectionSampleRingStart + m_detectionSampleRingCount) % capacity;
        m_detectionSampleRing[index] = sample;
        m_detectionSampleRingCount++;
    }
    else
    {
        m_detectionSampleRing[m_detectionSampleRingStart] = sample;
        m_detectionSampleRingStart = (m_detectionSampleRingStart + 1) % capacity;
        m_detectionSampleRingStartSample++;
    }
}

bool MeteorDemodSink::copyDetectionSamples(quint64 startSample, quint64 endSample, ComplexVector& samples) const
{
    samples.clear();

    if ((endSample < startSample) || (m_detectionSampleRingCount <= 0) || m_detectionSampleRing.empty()) {
        return false;
    }

    const quint64 historyEndSample = m_detectionSampleRingStartSample + (quint64) m_detectionSampleRingCount - 1;

    if ((startSample < m_detectionSampleRingStartSample) || (endSample > historyEndSample)) {
        return false;
    }

    const int capacity = (int) m_detectionSampleRing.size();
    const int count = (int) (endSample - startSample + 1);
    const int firstOffset = (int) (startSample - m_detectionSampleRingStartSample);
    samples.resize(count);

    for (int i = 0; i < count; i++) {
        samples[i] = m_detectionSampleRing[(m_detectionSampleRingStart + firstOffset + i) % capacity];
    }

    return true;
}

double MeteorDemodSink::estimatePulseTotalPower(quint64 startSample, quint64 endSample, double backgroundPower) const
{
    ComplexVector samples;

    if (!copyDetectionSamples(startSample, endSample, samples)) {
        return 0.0;
    }

    double totalPower = 0.0;

    for (const Complex& sample : samples) {
        totalPower += std::max(0.0, (double) std::norm(sample) - backgroundPower);
    }

    return totalPower;
}

bool MeteorDemodSink::reportsFrequencyCompatible(const PulseReport& first, const PulseReport& second) const
{
    const double firstSpan = std::max(
        std::fabs(first.m_duplicateFrequencySpan),
        std::fabs(first.m_frequencySpan));
    const double secondSpan = std::max(
        std::fabs(second.m_duplicateFrequencySpan),
        std::fabs(second.m_frequencySpan));
    const double binWidth = m_spectralFrameSize > 0
        ? (double) std::max(1, m_settings.m_channelSampleRate) / (double) m_spectralFrameSize
        : 1.0;
    const double padding = std::max(20.0, 2.5 * binWidth);
    const double firstLow = first.m_centerFrequency - 0.5 * firstSpan - padding;
    const double firstHigh = first.m_centerFrequency + 0.5 * firstSpan + padding;
    const double secondLow = second.m_centerFrequency - 0.5 * secondSpan - padding;
    const double secondHigh = second.m_centerFrequency + 0.5 * secondSpan + padding;
    const double overlap = std::max(0.0, std::min(firstHigh, secondHigh) - std::max(firstLow, secondLow));
    const double narrowerWidth = std::max(1.0, std::min(firstHigh - firstLow, secondHigh - secondLow));

    return (overlap >= 0.35 * narrowerWidth)
        || ((overlap > 0.0) && (std::fabs(first.m_centerFrequency - second.m_centerFrequency) <= std::max(firstSpan, secondSpan)))
        || (std::fabs(first.m_centerFrequency - second.m_centerFrequency) <= padding);
}

void MeteorDemodSink::emitDetectionReport(const PulseReport& report, const char *source)
{
    if (!report.m_valid) {
        return;
    }

    if ((QString::fromLatin1(source) != QStringLiteral("spectral-parent"))
        && overlapsBroadbandInterference(report.m_startSample, report.m_endSample))
    {
        qDebug() << "MeteorDemodSink::emitDetectionReport: broadband-contaminated report suppressed"
                 << " source:" << source
                 << " startSample:" << report.m_startSample
                 << " endSample:" << report.m_endSample;
        return;
    }

    const double peakPowerDB = 10.0 * std::log10(std::max(report.m_peakPower, 1e-20));
    const double backgroundPowerDB = 10.0 * std::log10(std::max(report.m_backgroundPower, 1e-20));
    double totalPower = report.m_totalPower;
    const quint64 sampleDuration = report.m_endSample >= report.m_startSample
        ? report.m_endSample - report.m_startSample + 1
        : 1;

    if (totalPower <= 0.0) {
        totalPower = estimatePulseTotalPower(report.m_startSample, report.m_endSample, report.m_backgroundPower);
    }

    if (totalPower <= 0.0) {
        totalPower = std::max(report.m_peakPower - report.m_backgroundPower, report.m_peakPower) * (double) std::max<quint64>(1, sampleDuration);
    }

    const double totalPowerDB = 10.0 * std::log10(std::max(totalPower, 1e-20));
    const quint64 displayStartSample = report.m_hasDisplaySamples ? report.m_displayStartSample : report.m_startSample;
    const quint64 displayEndSample = report.m_hasDisplaySamples ? report.m_displayEndSample : report.m_endSample;
    const QDateTime displayDateTimeUtc = sampleCounterToDisplayDateTimeUtc(displayStartSample);
    const QDateTime displayEndDateTimeUtc = sampleCounterToDisplayDateTimeUtc(displayEndSample + 1);
    double displayDurationS = report.m_durationS;

    if (displayDateTimeUtc.isValid() && displayEndDateTimeUtc.isValid())
    {
        const qint64 displayDurationMS = displayDateTimeUtc.msecsTo(displayEndDateTimeUtc);

        if (displayDurationMS >= 0) {
            displayDurationS = std::max(0.001, (double) displayDurationMS / 1000.0);
        }
    }

    qDebug() << "MeteorDemodSink::emitDetectionReport:"
             << " source:" << source
             << " dateTimeUtc:" << report.m_dateTimeUtc
             << " displayDateTimeUtc:" << displayDateTimeUtc
             << " durationS:" << report.m_durationS
             << " displayDurationS:" << displayDurationS
             << " peakPowerDB:" << peakPowerDB
             << " backgroundPowerDB:" << backgroundPowerDB
             << " totalPowerDB:" << totalPowerDB
             << " centerFrequency:" << report.m_centerFrequency
             << " frequencySpan:" << report.m_frequencySpan
             << " frequencyDrift:" << report.m_frequencyDrift
             << " startSample:" << report.m_startSample
             << " endSample:" << report.m_endSample
             << " displayStartSample:" << displayStartSample
             << " displayEndSample:" << displayEndSample;

    if (m_messageQueueToGUI)
    {
        DetectionRecord detection = report;
        detection.m_displayDateTimeUtc = displayDateTimeUtc;
        detection.m_displayDurationS = displayDurationS;
        detection.m_displayStartSample = displayStartSample;
        detection.m_displayEndSample = displayEndSample;
        detection.m_totalPower = totalPower;
        detection.m_sampleRate = m_settings.m_channelSampleRate;
        m_messageQueueToGUI->push(MsgMeteorDetected::create(detection));
    }

    rememberDetection(
        report.m_startSample,
        report.m_endSample,
        report.m_centerFrequency,
        report.m_duplicateFrequencySpan > 0.0 ? report.m_duplicateFrequencySpan : report.m_frequencySpan);

    // Send to event pipes
    if (!m_channel) {
        return;
    }

    QList<ObjectPipe*> eventPipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_channel, "event", eventPipes);

    QString eventData = QString("peakPowerDB=%1,totalPowerDB=%2,duration=%3,truncated=%4")
        .arg(peakPowerDB)
        .arg(totalPowerDB)
        .arg(displayDurationS)
        .arg(report.m_truncated ? 1 : 0);

    for (const auto& pipe : eventPipes)
    {
        MessageQueue *messageQueue = qobject_cast<MessageQueue*>(pipe->m_element);

        if (messageQueue) {
            messageQueue->push(MainCore::MsgEvent::create(m_channel, displayDateTimeUtc, MainCore::MsgEvent::EventType::MeteorScatterEvent, eventData));
        }
    }
}

QDateTime MeteorDemodSink::sampleCounterToDateTimeUtc(quint64 sampleCounter) const
{
    if (!m_streamStartDateTimeUtc.isValid() || (m_settings.m_channelSampleRate <= 0)) {
        return QDateTime::currentDateTimeUtc();
    }

    const qint64 offsetMSecs = (qint64) std::llround(1000.0 * (double) sampleCounter / (double) m_settings.m_channelSampleRate);
    return m_streamStartDateTimeUtc.addMSecs(offsetMSecs);
}

QDateTime MeteorDemodSink::sampleCounterToDisplayDateTimeUtc(quint64 sampleCounter) const
{
    if (m_displayTimeAnchors.empty() || (m_settings.m_channelSampleRate <= 0)) {
        return sampleCounterToDateTimeUtc(sampleCounter);
    }

    auto anchorIt = std::lower_bound(
        m_displayTimeAnchors.begin(),
        m_displayTimeAnchors.end(),
        sampleCounter,
        [](const DisplayTimeAnchor& anchor, quint64 sample)
        {
            return anchor.m_sampleCounter < sample;
        });

    auto interpolateFromAnchor = [this](const DisplayTimeAnchor& anchor, quint64 sample) -> QDateTime
    {
        const double sampleDelta = (double) sample - (double) anchor.m_sampleCounter;
        const qint64 offsetMSecs = (qint64) std::llround(1000.0 * sampleDelta / (double) m_settings.m_channelSampleRate);
        return anchor.m_dateTimeUtc.addMSecs(offsetMSecs);
    };

    if (anchorIt == m_displayTimeAnchors.begin()) {
        return interpolateFromAnchor(*anchorIt, sampleCounter);
    }

    if (anchorIt == m_displayTimeAnchors.end()) {
        return interpolateFromAnchor(m_displayTimeAnchors.back(), sampleCounter);
    }

    if (anchorIt->m_sampleCounter == sampleCounter) {
        return anchorIt->m_dateTimeUtc;
    }

    const DisplayTimeAnchor& right = *anchorIt;
    const DisplayTimeAnchor& left = *(anchorIt - 1);
    const quint64 sampleSpan = right.m_sampleCounter - left.m_sampleCounter;

    if (sampleSpan == 0) {
        return left.m_dateTimeUtc;
    }

    const qint64 timeSpanMSecs = left.m_dateTimeUtc.msecsTo(right.m_dateTimeUtc);
    const double fraction = (double) (sampleCounter - left.m_sampleCounter) / (double) sampleSpan;
    return left.m_dateTimeUtc.addMSecs((qint64) std::llround((double) timeSpanMSecs * fraction));
}

void MeteorDemodSink::recordDisplayTimeAnchor()
{
    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);
    const quint64 anchorInterval = (quint64) std::max(1, sampleRate / 20);

    if (!m_displayTimeAnchors.empty() && (m_sampleCounter < m_nextDisplayTimeAnchorSample)) {
        return;
    }

    m_displayTimeAnchors.push_back({
        m_sampleCounter,
        QDateTime::currentDateTimeUtc()
    });
    m_nextDisplayTimeAnchorSample = m_sampleCounter + anchorInterval;

    const int maxDurationSamples = std::max(1, (m_settings.m_maxDurationMS * sampleRate) / 1000);
    const quint64 keepSamples = (quint64) std::max(sampleRate * 120, maxDurationSamples + sampleRate * 20);

    if (m_sampleCounter <= keepSamples) {
        return;
    }

    const quint64 oldestSample = m_sampleCounter - keepSamples;
    auto firstKeep = std::lower_bound(
        m_displayTimeAnchors.begin(),
        m_displayTimeAnchors.end(),
        oldestSample,
        [](const DisplayTimeAnchor& anchor, quint64 sample)
        {
            return anchor.m_sampleCounter < sample;
        });

    if ((firstKeep != m_displayTimeAnchors.begin()) && (firstKeep != m_displayTimeAnchors.end())) {
        --firstKeep;
    }

    if (firstKeep != m_displayTimeAnchors.begin()) {
        m_displayTimeAnchors.erase(m_displayTimeAnchors.begin(), firstKeep);
    }
}

void MeteorDemodSink::emitDataCollectionMarker()
{
    if (!m_messageQueueToGUI) {
        return;
    }

    if (m_sampleCounter < m_nextDataMarkerSample) {
        return;
    }

    const QDateTime displayDateTimeUtc = sampleCounterToDisplayDateTimeUtc(m_sampleCounter);

    if (displayDateTimeUtc.isValid()) {
        m_messageQueueToGUI->push(MsgMeteorDataCollected::create(displayDateTimeUtc));
    }

    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);
    m_nextDataMarkerSample = m_sampleCounter + (quint64) sampleRate;
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

    const double slowAlpha = std::min(
        1.0,
        1.0 / ((double) m_settings.m_channelSampleRate * m_detectorTunables.m_scalarNoiseTimeConstantS));
    const double alpha = m_noiseFloor < (power * 0.01)
        ? m_detectorTunables.m_scalarRisingNoiseAlpha
        : slowAlpha;
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

void MeteorDemodSink::configureDetectionHistory()
{
    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);
    const int maxDurationSamples = std::max(1, m_settings.m_maxDurationMS * sampleRate / 1000);
    const int historyCapacity = maxDurationSamples + 5 * sampleRate;
    m_detectionSampleRing.assign(historyCapacity, Complex(0.0f, 0.0f));
    m_detectionSampleRingStart = 0;
    m_detectionSampleRingCount = 0;
    m_detectionSampleRingStartSample = m_sampleCounter;
}

void MeteorDemodSink::resolveDetectorTunables()
{
    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);
    const int frameSize = std::max(1, m_spectralFrameSize);
    const double halfUsableBandwidth = std::max(
        1.0,
        std::min(0.49, m_detectorTunables.m_usableBandwidthRateFraction) * (double) sampleRate);
    const double fullUsableBandwidth = 2.0 * halfUsableBandwidth;
    auto resolveBandwidth = [fullUsableBandwidth](double value) {
        return std::clamp(value, 1.0, fullUsableBandwidth);
    };

    m_resolvedDetectorTunables.m_binWidthHz = (double) sampleRate / (double) frameSize;
    m_resolvedDetectorTunables.m_usableBandwidthHz = halfUsableBandwidth;
    m_resolvedDetectorTunables.m_maxSegmentedBandWidthHz = resolveBandwidth(
        m_detectorTunables.m_maxSegmentedBandWidthHz);
    m_resolvedDetectorTunables.m_compactBandwidthHz = resolveBandwidth(m_detectorTunables.m_compactBandwidthHz);
    m_resolvedDetectorTunables.m_stableBandwidthHz = std::max(
        m_resolvedDetectorTunables.m_compactBandwidthHz,
        resolveBandwidth(m_detectorTunables.m_stableBandwidthHz));
    m_resolvedDetectorTunables.m_twoFrameMaxBandwidthHz = resolveBandwidth(m_detectorTunables.m_twoFrameMaxBandwidthHz);
    m_resolvedDetectorTunables.m_coherentWideTwoFrameMinBandwidthHz = resolveBandwidth(
        m_detectorTunables.m_coherentWideTwoFrameMinBandwidthHz);
    m_resolvedDetectorTunables.m_coherentWideTwoFrameMaxBandwidthHz = std::max(
        m_resolvedDetectorTunables.m_coherentWideTwoFrameMinBandwidthHz,
        resolveBandwidth(m_detectorTunables.m_coherentWideTwoFrameMaxBandwidthHz));
    m_resolvedDetectorTunables.m_broadbandImpulseMinSpanHz = resolveBandwidth(
        m_detectorTunables.m_broadbandImpulseMinSpanHz);
    m_resolvedDetectorTunables.m_broadbandImpulseMinBandwidthHz = resolveBandwidth(
        m_detectorTunables.m_broadbandImpulseMinBandwidthHz);
    m_resolvedDetectorTunables.m_powerStableBandwidthHz = resolveBandwidth(
        m_detectorTunables.m_powerStableBandwidthHz);
    m_resolvedDetectorTunables.m_trackingFrequencyPaddingHz = std::max(
        m_resolvedDetectorTunables.m_binWidthHz,
        m_detectorTunables.m_trackingFrequencyPaddingHz);
    m_resolvedDetectorTunables.m_maxTrackingJumpHz = std::max(
        m_resolvedDetectorTunables.m_trackingFrequencyPaddingHz,
        m_detectorTunables.m_maxTrackingJumpHz);
    m_resolvedDetectorTunables.m_sustainedSweepMinDriftHz = resolveBandwidth(
        m_detectorTunables.m_sustainedSweepMinDriftHz);
    m_resolvedDetectorTunables.m_compactSweepMinDriftHz = resolveBandwidth(
        m_detectorTunables.m_compactSweepMinDriftHz);
    m_resolvedDetectorTunables.m_continuationFrequencyPaddingHz = resolveBandwidth(
        m_detectorTunables.m_continuationFrequencyPaddingHz);
    const double hopDurationS = (double) std::max(1, m_spectralHopSize) / (double) sampleRate;
    m_resolvedDetectorTunables.m_minimumNoiseFramesPerBlock = std::max(
        3,
        (int) std::llround(m_detectorTunables.m_minimumNoiseBlockDurationS / std::max(1e-6, hopDurationS)));
}

void MeteorDemodSink::configureSpectralDetector()
{
    m_spectralFrameSize = std::clamp(
        (int) std::llround(
            m_detectorTunables.m_spectralFrameDurationS
                * (double) std::max(1, m_settings.m_channelSampleRate)),
        32,
        256);
    if ((m_spectralFrameSize % 2) != 0) {
        m_spectralFrameSize++;
    }
    m_spectralHopSize = std::max(
        1,
        (int) std::floor(
            m_detectorTunables.m_spectralHopFraction * (double) m_spectralFrameSize));
    resolveDetectorTunables();

    if (!m_spectralFFT) {
        m_spectralFFT = createMeteorFFT();
    }

    if (m_spectralFFTSize != m_spectralFrameSize)
    {
        m_spectralFFT->configure(m_spectralFrameSize, false);
        m_spectralFFTSize = m_spectralFrameSize;
    }

    makeHannWindow(m_spectralWindow, m_spectralFrameSize);
    double windowEnergy = 0.0;

    for (Real coefficient : m_spectralWindow) {
        windowEnergy += (double) coefficient * (double) coefficient;
    }

    m_spectralEnergyScale = (double) m_spectralHopSize
        / std::max(1e-30, (double) m_spectralFrameSize * windowEnergy);
    m_spectralBinPower.assign(m_spectralFrameSize, 0.0);
    m_spectralActiveBins.assign(m_spectralFrameSize, 0);
    m_spectralFrameBuffer.clear();
    m_spectralNoiseFloor.clear();
    m_spectralMinimumNoiseFloor.clear();
    m_minimumNoiseCurrentBlock.clear();
    m_minimumNoiseBlocks.clear();
    m_spectralCalibrationFrames.clear();
    m_spectralEvents.clear();
    m_activeMeteorEvents.clear();
    m_nextMeteorEventId = 1;
    m_nextComponentFlushSample = std::numeric_limits<quint64>::max();
    m_spectralNoiseFloorInitialized = false;
    configureDetectionHistory();
}

void MeteorDemodSink::resizeSpectrumBuffer()
{
    m_spectrumBufferSize = std::max(16, m_settings.m_channelSampleRate / 10);
    m_spectrumBuffer.clear();
    m_spectrumBuffer.reserve(m_spectrumBufferSize);
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
    m_pulsePeakSample = 0;
    m_pulsePeakPower = 0.0;
    m_displayTimeAnchors.clear();
    m_nextDisplayTimeAnchorSample = 0;
    m_nextDataMarkerSample = 0;
    m_pulseSamples.clear();
    m_detectionSampleRingStart = 0;
    m_detectionSampleRingCount = 0;
    m_detectionSampleRingStartSample = 0;
    m_pendingBroadPulse = PulseReport();
    m_spectralFrameBuffer.clear();
    m_spectralNoiseFloor.clear();
    m_spectralMinimumNoiseFloor.clear();
    m_minimumNoiseCurrentBlock.clear();
    m_minimumNoiseBlocks.clear();
    m_spectralCalibrationFrames.clear();
    m_spectralEvents.clear();
    m_recentDetectionRanges.clear();
    m_recentSpectralInterference.clear();
    m_activeMeteorEvents.clear();
    m_nextMeteorEventId = 1;
    m_spectralNoiseFloorInitialized = false;
}

void MeteorDemodSink::feedSpectrum(const Complex& sample)
{
    if (!m_spectrumSink && !m_secondarySpectrumSink) {
        return;
    }

    m_spectrumBuffer.push_back(sample);

    if ((int) m_spectrumBuffer.size() >= m_spectrumBufferSize)
    {
        if (m_spectrumSink) {
            m_spectrumSink->feed(m_spectrumBuffer.begin(), m_spectrumBuffer.end(), false);
        }

        if (m_secondarySpectrumSink) {
            m_secondarySpectrumSink->feed(m_spectrumBuffer.begin(), m_spectrumBuffer.end(), false);
        }

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
    const bool historyChanged = (settingsKeys.contains("maxDurationMS")
        && (settings.m_maxDurationMS != m_settings.m_maxDurationMS)) || force;

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (sampleRateChanged)
    {
        configureInterpolator();
        configureSpectralDetector();
        resizeSpectrumBuffer();
        resetDetector();
    }

    if (sampleRateChanged || lpfChanged) {
        configurePowerLowpass();
    }

    if (!sampleRateChanged && historyChanged) {
        configureDetectionHistory();
    }
}
