///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012 maintech GmbH, Otto-Hahn-Str. 15, 97204 Hoechberg, Germany //
// written by Christian Daniel                                                   //
// Copyright (C) 2014 John Greb <karikoa@One.greyskull>                          //
// Copyright (C) 2015-2023 Edouard Griffiths, F4EXB <f4exb06@gmail.com>          //
// Copyright (C) 2022 Jiří Pinkava <jiri.pinkava@rossum.ai>                      //
// Copyright (C) 2023 Arne Jünemann <das-iro@das-iro.de>                         //
// Copyright (C) 2023 Vladimir Pleskonjic                                        //
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
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

#include "SWGGLSpectrum.h"
#include "SWGSpectrumServer.h"
#include "SWGSuccessResponse.h"

#include "glspectruminterface.h"
#include "dspcommands.h"
#include "dspengine.h"
#include "fftfactory.h"
#include "util/messagequeue.h"
#include "util/profiler.h"

#include "spectrumvis.h"

namespace {
// Morlet wavelet central angular frequency.
// omega0 = 6 is the standard choice that gives a good trade-off between time and
// frequency resolution while keeping the wavelet admissibility condition satisfied.
static constexpr float kMorletCentralFrequency = 6.0f;
// Limit summation to bins within 4 sigma of the wavelet centre (in relative units).
// |j/k - 1| <= kSigmaCutoff; bins beyond this threshold contribute < 0.01% of total weight.
static constexpr float kSigmaCutoff = 4.0f / kMorletCentralFrequency; // ~0.667
static constexpr float kHalfOmega0Sq = kMorletCentralFrequency * kMorletCentralFrequency / 2.0f; // 18.0
} // namespace

MESSAGE_CLASS_DEFINITION(SpectrumVis::MsgConfigureSpectrumVis, Message)
MESSAGE_CLASS_DEFINITION(SpectrumVis::MsgConfigureScalingFactor, Message)
MESSAGE_CLASS_DEFINITION(SpectrumVis::MsgConfigureWSpectrumOpenClose, Message)
MESSAGE_CLASS_DEFINITION(SpectrumVis::MsgConfigureWSpectrum, Message)
MESSAGE_CLASS_DEFINITION(SpectrumVis::MsgStartStop, Message)

const Real SpectrumVis::m_mult = (10.0f / log2(10.0f));

SpectrumVis::SpectrumVis(Real scalef) :
	BasebandSampleSink(),
    m_running(true),
	m_fft(nullptr),
    m_fftEngineSequence(0),
	m_fftBuffer(4096),
	m_powerSpectrum(4096),
    m_mathMemory(4096),
	m_fftBufferFill(0),
    m_cwtFft(nullptr),
    m_cwtFftEngineSequence(0),
    m_cwtPowFFTMul(1.0f),
    m_cwtBufferFill(0),
	m_needMoreSamples(false),
	m_scalef(scalef),
	m_glSpectrum(nullptr),
    m_specMax(0.0f),
    m_centerFrequency(0),
    m_sampleRate(48000),
    m_powFFTMul(1.0f),
    m_guiMessageQueue(nullptr)
{
	setObjectName("SpectrumVis");
    connect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));
    applySettings(m_settings, true);
}

SpectrumVis::~SpectrumVis()
{
    FFTFactory *fftFactory = DSPEngine::instance()->getFFTFactory();
    fftFactory->releaseEngine(m_settings.m_fftSize, false, m_fftEngineSequence);
    if (m_cwtFft) {
        fftFactory->releaseEngine(
            static_cast<unsigned int>(m_settings.m_fftSize) * m_settings.m_cwtTimeSteps,
            false, m_cwtFftEngineSequence);
    }
}

void SpectrumVis::setScalef(Real scalef)
{
    MsgConfigureScalingFactor* cmd = new MsgConfigureScalingFactor(scalef);
    m_inputMessageQueue.push(cmd);
}

void SpectrumVis::configureWSSpectrum(const QString& address, uint16_t port)
{
    MsgConfigureWSpectrum* cmd = new MsgConfigureWSpectrum(address, port);
    m_inputMessageQueue.push(cmd);
}

void SpectrumVis::feedTriggered(const SampleVector::const_iterator& triggerPoint, const SampleVector::const_iterator& end, bool positiveOnly)
{
	feed(triggerPoint, end, positiveOnly); // normal feed from trigger point
	/*
	if (triggerPoint == end)
	{
		// the following piece of code allows to terminate the FFT that ends past the end of scope captured data
		// that is the spectrum will include the captured data
		// just do nothing if you want the spectrum to be included inside the scope captured data
		// that is to drop the FFT that dangles past the end of captured data
		if (m_needMoreSamples) {
			feed(begin, end, positiveOnly);
			m_needMoreSamples = false;      // force finish
		}
	}
	else
	{
		feed(triggerPoint, end, positiveOnly); // normal feed from trigger point
	}*/
}

void SpectrumVis::feed(const Complex *begin, unsigned int length)
{
    if (!m_glSpectrum && !m_wsSpectrum.socketOpened()) {
        return;
    }

    if (!m_mutex.tryLock(0)) { // prevent conflicts with configuration process
        return;
    }

    processFFT(begin, false, false, length);

    m_mutex.unlock();
}

void SpectrumVis::feed(const ComplexVector::const_iterator& cbegin, const ComplexVector::const_iterator& end, bool positiveOnly)
{
    if (!m_running) {
        return;
    }

	// if no visualisation is set, send the samples to /dev/null
	if (!m_glSpectrum && !m_wsSpectrum.socketOpened()) {
		return;
	}

    if (!m_mutex.tryLock(0)) { // prevent conflicts with configuration process
        return;
    }

	ComplexVector::const_iterator begin(cbegin);

	while (begin < end)
	{
		std::size_t todo = end - begin;

        if (m_settings.m_transformType == SpectrumSettings::CWT)
        {
            // Sliding window: collect fftSize new samples per step.
            // New samples fill the tail of m_cwtBuffer; on completion the buffer is slid left
            // by fftSize so the oldest fftSize samples are discarded and the history is kept.
            // This gives one output row per fftSize samples (same rate as no-overlap FFT).
            const std::size_t fftStep = static_cast<std::size_t>(m_settings.m_fftSize);
            const std::size_t cwtTotal = fftStep * m_settings.m_cwtTimeSteps;
            const std::size_t insertBase = cwtTotal - fftStep;  // start of the new-samples region
            std::size_t samplesNeeded = fftStep - m_cwtBufferFill;

            if (todo >= samplesNeeded)
            {
                std::copy(begin, begin + samplesNeeded, m_cwtBuffer.begin() + insertBase + m_cwtBufferFill);
                begin += samplesNeeded;
                performCWT(positiveOnly);
                // Slide history: discard oldest fftSize samples so the tail is ready for the next step.
                std::copy(m_cwtBuffer.begin() + fftStep, m_cwtBuffer.end(), m_cwtBuffer.begin());
                m_cwtBufferFill = 0;
                m_needMoreSamples = false;
            }
            else
            {
                std::copy(begin, end, m_cwtBuffer.begin() + insertBase + m_cwtBufferFill);
                begin = end;
                m_cwtBufferFill += todo;
                m_needMoreSamples = true;
            }
        }
        else
        {
            std::size_t samplesNeeded = m_settings.m_fftSize - m_fftBufferFill;

            if (todo >= samplesNeeded)
            {
                // fill up the buffer
                std::copy(begin, begin + samplesNeeded, m_fftBuffer.begin() + m_fftBufferFill);
                begin += samplesNeeded;
                performFFT(positiveOnly);

                // advance buffer respecting the fft overlap factor
                // undefined behavior if the memory regions overlap, valid code for 50% overlap
                std::copy(m_fftBuffer.begin() + m_refillSize, m_fftBuffer.end(), m_fftBuffer.begin());

                // start over
                m_fftBufferFill = m_overlapSize;
                m_needMoreSamples = false;
            }
            else
            {
                // not enough samples for FFT - just fill in new data and return
                std::copy(begin, end, m_fftBuffer.begin() + m_fftBufferFill);
                begin = end;
                m_fftBufferFill += todo;
                m_needMoreSamples = true;
            }
        }
	}

	m_mutex.unlock();
}

void SpectrumVis::feed(const SampleVector::const_iterator& cbegin, const SampleVector::const_iterator& end, bool positiveOnly)
{
    if (!m_running) {
        return;
    }

	// if no visualisation is set, send the samples to /dev/null
	if (!m_glSpectrum && !m_wsSpectrum.socketOpened()) {
		return;
	}

    if (!m_mutex.tryLock(0)) { // prevent conflicts with configuration process
        return;
    }

	SampleVector::const_iterator begin(cbegin);

	while (begin < end)
	{
		std::size_t todo = end - begin;

        if (m_settings.m_transformType == SpectrumSettings::CWT)
        {
            // Sliding window: collect fftSize new samples per step.
            // New samples fill the tail of m_cwtBuffer; on completion the buffer is slid left
            // by fftSize so the oldest fftSize samples are discarded and the history is kept.
            // This gives one output row per fftSize samples (same rate as no-overlap FFT).
            const std::size_t fftStep = static_cast<std::size_t>(m_settings.m_fftSize);
            const std::size_t cwtTotal = fftStep * m_settings.m_cwtTimeSteps;
            const std::size_t insertBase = cwtTotal - fftStep;
            std::size_t samplesNeeded = fftStep - m_cwtBufferFill;

            if (todo >= samplesNeeded)
            {
                auto it = m_cwtBuffer.begin() + insertBase + m_cwtBufferFill;
                for (std::size_t i = 0; i < samplesNeeded; ++i, ++begin) {
                    *it++ = Complex(begin->real() / m_scalef, begin->imag() / m_scalef);
                }
                performCWT(positiveOnly);
                std::copy(m_cwtBuffer.begin() + fftStep, m_cwtBuffer.end(), m_cwtBuffer.begin());
                m_cwtBufferFill = 0;
                m_needMoreSamples = false;
            }
            else
            {
                auto it = m_cwtBuffer.begin() + insertBase + m_cwtBufferFill;
                for (; begin < end; ++begin) {
                    *it++ = Complex(begin->real() / m_scalef, begin->imag() / m_scalef);
                }
                m_cwtBufferFill += todo;
                m_needMoreSamples = true;
            }
        }
        else
        {
            std::size_t samplesNeeded = m_settings.m_fftSize - m_fftBufferFill;

            if (todo >= samplesNeeded)
            {
                // fill up the buffer
                std::vector<Complex>::iterator it = m_fftBuffer.begin() + m_fftBufferFill;

                for (std::size_t i = 0; i < samplesNeeded; ++i, ++begin) {
                    *it++ = Complex(begin->real() / m_scalef, begin->imag() / m_scalef);
                }

                performFFT(positiveOnly);

                // advance buffer respecting the fft overlap factor
                // undefined behavior if the memory regions overlap, valid code for 50% overlap
                std::copy(m_fftBuffer.begin() + m_refillSize, m_fftBuffer.end(), m_fftBuffer.begin());

                // start over
                m_fftBufferFill = m_overlapSize;
                m_needMoreSamples = false;
            }
            else
            {
                // not enough samples for FFT - just fill in new data and return
                for (std::vector<Complex>::iterator it = m_fftBuffer.begin() + m_fftBufferFill; begin < end; ++begin) {
                    *it++ = Complex(begin->real() / m_scalef, begin->imag() / m_scalef);
                }

                m_fftBufferFill += todo;
                m_needMoreSamples = true;
            }
        }
	}

	m_mutex.unlock();
}

// Compute math operation on linear spectrum values
void SpectrumVis::mathLinear(std::vector<Real> &spectrum)
{
    if (m_settings.m_mathMode == SpectrumSettings::MathModeXMinusAvg)
    {
        for (std::size_t i = 0; i < spectrum.size(); i++) {
            spectrum[i] = std::max(spectrum[i] - (Real) m_mathMovingAverage.storeAndGetAvg(spectrum[i], i), 0.0f);
        }
    }
    else if ((m_settings.m_mathMode == SpectrumSettings::MathModeXMinusM1) || (m_settings.m_mathMode == SpectrumSettings::MathModeXMinusM2))
    {
        for (std::size_t i = 0; i < spectrum.size(); i++) {
            spectrum[i] = std::max(spectrum[i] - m_mathMemory[i], 0.0f);
        }
    }
}

// Compute math operation on dB spectrum values
void SpectrumVis::mathDB(std::vector<Real> &spectrum)
{
    if (m_settings.m_mathMode == SpectrumSettings::MathModeXMinusAvgDB)
    {
        for (std::size_t i = 0; i < spectrum.size(); i++) {
            spectrum[i] -= m_mathMovingAverage.storeAndGetAvg(spectrum[i], i);
        }
    }
    else if (m_settings.m_mathMode == SpectrumSettings::MathModeXMinusAvgPlusMinAvgDB)
    {
        Real minAvg = m_mathMovingAverage.getMin();
        for (std::size_t i = 0; i < spectrum.size(); i++) {
            spectrum[i] = spectrum[i] - m_mathMovingAverage.storeAndGetAvg(spectrum[i], i) + minAvg;
        }
    }
    else if (m_settings.m_mathMode == SpectrumSettings::MathModeAbsXMinusAvgDB)
    {
        for (std::size_t i = 0; i < spectrum.size(); i++) {
            spectrum[i] = abs(spectrum[i] - m_mathMovingAverage.storeAndGetAvg(spectrum[i], i));
        }
    }
    else if ((m_settings.m_mathMode == SpectrumSettings::MathModeXMinusM1DB) || (m_settings.m_mathMode == SpectrumSettings::MathModeXMinusM2DB))
    {
        for (std::size_t i = 0; i < spectrum.size(); i++) {
            spectrum[i] -= m_mathMemory[i];
        }
    }
    else if ((m_settings.m_mathMode == SpectrumSettings::MathModeAbsXMinusM1DB) || (m_settings.m_mathMode == SpectrumSettings::MathModeAbsXMinusM2DB))
    {
        for (std::size_t i = 0; i < spectrum.size(); i++) {
            spectrum[i] = abs(spectrum[i] - m_mathMemory[i]);
        }
    }
}

void SpectrumVis::performFFT(bool positiveOnly)
{
    // apply fft window (and copy from m_fftBuffer to m_fftIn)
    m_window.apply(&m_fftBuffer[0], m_fft->in());

    // calculate FFT
    m_fft->transform();

    // extract power spectrum and reorder buckets
    processFFT(m_fft->out(), true, positiveOnly, m_settings.m_fftSize);
}

void SpectrumVis::performCWT(bool positiveOnly)
{
    const int fftSize    = m_settings.m_fftSize;
    const int cwtFftSize = fftSize * m_settings.m_cwtTimeSteps;  // Large N_total-point FFT size
    const int halfSize   = fftSize / 2;

    // Apply the large analysis window and compute one N_total-point FFT over the full
    // sliding history buffer.  Using a larger FFT than the display size means that
    // low-frequency output bins are resolved from proportionally more samples
    // (constant-Q / scale-dependent resolution) — the defining CWT property.
    m_cwtWindow.apply(m_cwtBuffer.data(), m_cwtFft->in());
    m_cwtFft->transform();

    const Complex* fftOut = m_cwtFft->out();

    // Compute linear power spectrum from the large FFT (natural order: 0..N_total-1).
    for (int i = 0; i < cwtFftSize; i++) {
        const Complex& c = fftOut[i];
        m_cwtFftPower[i] = (c.real() * c.real() + c.imag() * c.imag()) * m_cwtPowFFTMul;
    }

    // Accumulate CWT power using the precomputed, pre-normalised log-spaced weight tables.
    // Tables are indexed into the large FFT, giving finer absolute frequency resolution at
    // low output bins (narrow Gaussian in a high-resolution spectrum).
    if (positiveOnly)
    {
        for (int i = 0; i < halfSize; i++)
        {
            float power = 0.0f;
            for (const auto& entry : m_cwtWeightsPos[i]) {
                power += m_cwtFftPower[entry.first] * entry.second;
            }
            m_powerSpectrum[i * 2]     = power;
            m_powerSpectrum[i * 2 + 1] = power;
        }
    }
    else
    {
        for (int kOut = 0; kOut < fftSize; kOut++)
        {
            float power = 0.0f;
            for (const auto& entry : m_cwtWeightsFull[kOut]) {
                power += m_cwtFftPower[entry.first] * entry.second;
            }
            m_powerSpectrum[kOut] = power;
        }
    }

    // Track spectrum maximum (linear power, before dB conversion).
    m_specMax = *std::max_element(&m_powerSpectrum[0], &m_powerSpectrum[fftSize]);

    // Perform math operation on linear value.
    if (m_settings.m_mathMode != SpectrumSettings::MathModeNone) {
        mathLinear(m_powerSpectrum);
    }

    // Convert to dB.
    if (!m_settings.m_linear) {
        for (int i = 0; i < fftSize; i++) {
            m_powerSpectrum[i] = m_mult * log2fapprox(m_powerSpectrum[i]);
        }
    }

    // Perform math operation on dB value.
    if (m_settings.m_mathMode != SpectrumSettings::MathModeNone) {
        mathDB(m_powerSpectrum);
    }

    if (m_settings.mathAverageUsed()) {
        m_mathMovingAverage.nextAverage();
    }

    // Send the single CWT spectrum row to the visualisation.
    if (m_glSpectrum) {
        m_glSpectrum->newSpectrum(&m_powerSpectrum[0], fftSize);
    }

    // Web socket spectrum connections.
    if (m_wsSpectrum.socketOpened())
    {
        m_wsSpectrum.newSpectrum(
            m_powerSpectrum,
            fftSize,
            m_centerFrequency,
            m_sampleRate,
            m_settings.m_linear,
            m_settings.m_ssb,
            m_settings.m_usb
        );
    }
}

void SpectrumVis::buildCWTWeightTables(int fftSize, int cwtTimeSteps)
{
    const int halfSize   = fftSize / 2;
    const int cwtFftSize = fftSize * cwtTimeSteps;    // Large N_total-point FFT size
    const int cwtHalf    = cwtFftSize / 2;             // = halfSize * cwtTimeSteps

    // Map output display-bin index i (0..n-1) to a log-spaced centre bin in the large FFT.
    // lo = cwtTimeSteps maps to the lowest positive frequency (1/fftSize) and
    // hi = cwtHalf maps to Nyquist.  The result is cwtTimeSteps times the centre bin
    // of the old fftSize-based tables; this larger absolute bin index means the Gaussian
    // spread also grows proportionally, pulling from more high-resolution FFT bins at every
    // scale — i.e. constant relative bandwidth (constant-Q).
    auto logCenterBin = [](int i, int n, int lo, int hi) -> int {
        if (n <= 1) return lo;
        const double t = static_cast<double>(i) / static_cast<double>(n - 1);
        const double val = static_cast<double>(lo)
            * std::pow(static_cast<double>(hi) / static_cast<double>(lo), t);
        return std::max(lo, std::min(hi, static_cast<int>(std::round(val))));
    };

    // --- Full (two-sided) mode ---
    // Output layout (centred): kOut=0 → most-negative, kOut=halfSize → DC (skipped),
    // kOut=fftSize-1 → most-positive.
    m_cwtWeightsFull.assign(fftSize, {});
    for (int kOut = 0; kOut < fftSize; kOut++)
    {
        if (kOut == halfSize) {
            continue; // DC bin: leave weight list empty so power stays 0
        }

        int k;
        if (kOut > halfSize)
        {
            // Positive side: halfSize-1 output bins, log-spaced k in [cwtTimeSteps, cwtHalf - cwtTimeSteps]
            const int iPosSlot = kOut - halfSize - 1;
            k = logCenterBin(iPosSlot, halfSize - 1, cwtTimeSteps, cwtHalf - cwtTimeSteps);
        }
        else
        {
            // Negative side: halfSize output bins, log-spaced |k| in [cwtTimeSteps, cwtHalf]
            const int iNegSlot = halfSize - 1 - kOut;
            k = -logCenterBin(iNegSlot, halfSize, cwtTimeSteps, cwtHalf);
        }

        const int kAbs     = std::abs(k);
        const int rangeBins = static_cast<int>(kSigmaCutoff * kAbs) + 2;

        int jCentMin, jCentMax;
        if (k > 0) {
            jCentMin = std::max(1, k - rangeBins);
            jCentMax = std::min(cwtHalf - 1, k + rangeBins);
        } else {
            jCentMin = std::max(-cwtHalf, k - rangeBins);
            jCentMax = std::min(-1, k + rangeBins);
        }

        // Single pass: accumulate raw Gaussian weights and their sum for normalisation.
        // Storing raw weights avoids recomputing exp() in the normalisation step.
        std::vector<float> rawWeights;
        rawWeights.reserve(static_cast<std::size_t>(jCentMax - jCentMin + 1));
        float norm = 0.0f;
        for (int jCent = jCentMin; jCent <= jCentMax; jCent++) {
            const float ratio = static_cast<float>(jCent) / static_cast<float>(k);
            const float diff  = ratio - 1.0f;
            const float w = std::exp(-kHalfOmega0Sq * diff * diff);
            rawWeights.push_back(w);
            norm += w;
        }

        auto& bin = m_cwtWeightsFull[kOut];
        bin.reserve(rawWeights.size());
        if (norm > 0.0f) {
            for (std::size_t idx = 0; idx < rawWeights.size(); ++idx) {
                const int jCent = jCentMin + static_cast<int>(idx);
                const int jNat  = (jCent + cwtFftSize) % cwtFftSize;
                bin.emplace_back(jNat, rawWeights[idx] / norm);
            }
        }
    }

    // --- Positive-only mode ---
    // halfSize output slots, each log-spaced to cover k = cwtTimeSteps..cwtHalf (including Nyquist).
    m_cwtWeightsPos.assign(halfSize, {});
    for (int i = 0; i < halfSize; i++)
    {
        const int k        = logCenterBin(i, halfSize, cwtTimeSteps, cwtHalf);
        const int rangeBins = static_cast<int>(kSigmaCutoff * k) + 2;
        const int jMin     = std::max(1, k - rangeBins);
        const int jMax     = std::min(cwtHalf, k + rangeBins);

        // Single pass: accumulate raw Gaussian weights and their sum for normalisation.
        std::vector<float> rawWeights;
        rawWeights.reserve(static_cast<std::size_t>(jMax - jMin + 1));
        float norm = 0.0f;
        for (int j = jMin; j <= jMax; j++) {
            const float ratio = static_cast<float>(j) / static_cast<float>(k);
            const float diff  = ratio - 1.0f;
            const float w = std::exp(-kHalfOmega0Sq * diff * diff);
            rawWeights.push_back(w);
            norm += w;
        }

        auto& bin = m_cwtWeightsPos[i];
        bin.reserve(rawWeights.size());
        if (norm > 0.0f) {
            for (std::size_t idx = 0; idx < rawWeights.size(); ++idx) {
                bin.emplace_back(jMin + static_cast<int>(idx), rawWeights[idx] / norm);
            }
        }
    }
}

void SpectrumVis::processFFT(const Complex* fftOut, bool reorder, bool positiveOnly, int fftSize)
{
    PROFILER_START();

    Complex c;
    Real v;
    int halfSize = fftSize / 2;
    bool ready = false;

    if (m_settings.m_averagingMode == SpectrumSettings::AvgModeNone)
    {
        if (positiveOnly)
        {
            for (int i = 0; i < halfSize; i++)
            {
                c = fftOut[i];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;
                m_powerSpectrum[i * 2] = v;
                m_powerSpectrum[i * 2 + 1] = v;
            }
        }
        else if (reorder)
        {
            for (int i = 0; i < halfSize; i++)
            {
                c = fftOut[i + halfSize];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;
                m_powerSpectrum[i] = v;

                c = fftOut[i];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;
                m_powerSpectrum[i + halfSize] = v;
            }
        }
        else
        {
            for (int i = 0; i < fftSize; i++)
            {
                c = fftOut[i];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;
                m_powerSpectrum[i] = v;
            }
        }

        ready = true;
    }
    else if (m_settings.m_averagingMode == SpectrumSettings::AvgModeMoving)
    {
        double avg;

        if (positiveOnly)
        {
            for (int i = 0; i < halfSize; i++)
            {
                c = fftOut[i];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;
                avg = m_movingAverage.storeAndGetAvg(v, i);
                m_powerSpectrum[i * 2] = (Real) avg;
                m_powerSpectrum[i * 2 + 1] = (Real) avg;
            }
        }
        else if (reorder)
        {
            for (int i = 0; i < halfSize; i++)
            {
                c = fftOut[i + halfSize];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;
                avg = m_movingAverage.storeAndGetAvg(v, i+halfSize);
                m_powerSpectrum[i] = (Real) avg;

                c = fftOut[i];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;
                avg = m_movingAverage.storeAndGetAvg(v, i);
                m_powerSpectrum[i + halfSize] = (Real) avg;
            }
        }
        else
        {
            for (int i = 0; i < fftSize; i++)
            {
                c = fftOut[i];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;
                avg = m_movingAverage.storeAndGetAvg(v, i);
                m_powerSpectrum[i] = (Real) avg;
            }
        }

        m_movingAverage.nextAverage();
        ready = true;
    }
    else if (m_settings.m_averagingMode == SpectrumSettings::AvgModeFixed)
    {
        double avg;

        if (positiveOnly)
        {
            for (int i = 0; i < halfSize; i++)
            {
                c = fftOut[i];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;

                // result available
                if (m_fixedAverage.storeAndGetAvg(avg, v, i))
                {
                    m_powerSpectrum[i * 2] = avg;
                    m_powerSpectrum[i * 2 + 1] = avg;
                }
            }
        }
        else if (reorder)
        {
            for (int i = 0; i < halfSize; i++)
            {
                c = fftOut[i + halfSize];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;

                // result available
                if (m_fixedAverage.storeAndGetAvg(avg, v, i+halfSize)) {
                    m_powerSpectrum[i] = avg;
                }

                c = fftOut[i];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;

                // result available
                if (m_fixedAverage.storeAndGetAvg(avg, v, i)) {
                    m_powerSpectrum[i + halfSize] = avg;
                }
            }
        }
        else
        {
            for (int i = 0; i < fftSize; i++)
            {
                c = fftOut[i];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;

                // result available
                if (m_fixedAverage.storeAndGetAvg(avg, v, i)) {
                    m_powerSpectrum[i] = avg;
                }
            }
        }

        // result available
        if (m_fixedAverage.nextAverage()) {
            ready = true;
        }
    }
    else if (m_settings.m_averagingMode == SpectrumSettings::AvgModeMax)
    {
        Real max;

        if (positiveOnly)
        {
            for (int i = 0; i < halfSize; i++)
            {
                c = fftOut[i];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;

                // result available
                if (m_max.storeAndGetMax(max, v, i))
                {
                    m_powerSpectrum[i * 2] = max;
                    m_powerSpectrum[i * 2 + 1] = max;
                }
            }
        }
        else if (reorder)
        {
            for (int i = 0; i < halfSize; i++)
            {
                c = fftOut[i + halfSize];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;

                // result available
                if (m_max.storeAndGetMax(max, v, i+halfSize)) {
                    m_powerSpectrum[i] = max;
                }

                c = fftOut[i];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;

                // result available
                if (m_max.storeAndGetMax(max, v, i)) {
                    m_powerSpectrum[i + halfSize] = max;
                }
            }
        }
        else
        {
            for (int i = 0; i < fftSize; i++)
            {
                c = fftOut[i];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;

                // result available
                if (m_max.storeAndGetMax(max, v, i)) {
                    m_powerSpectrum[i] = max;
                }
            }
        }

        // result available
        if (m_max.nextMax()) {
            ready = true;
        }
    }
    else if (m_settings.m_averagingMode == SpectrumSettings::AvgModeMin)
    {
        Real min;

        if (positiveOnly)
        {
            for (int i = 0; i < halfSize; i++)
            {
                c = fftOut[i];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;

                if (m_min.storeAndGetMin(min, v, i))
                {
                    m_powerSpectrum[i * 2] = min;
                    m_powerSpectrum[i * 2 + 1] = min;
                }
            }
        }
        else if (reorder)
        {
            for (int i = 0; i < halfSize; i++)
            {
                c = fftOut[i + halfSize];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;

                // result available
                if (m_min.storeAndGetMin(min, v, i+halfSize)) {
                    m_powerSpectrum[i] = min;
                }

                c = fftOut[i];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;

                // result available
                if (m_min.storeAndGetMin(min, v, i)) {
                    m_powerSpectrum[i + halfSize] = min;
                }
            }
        }
        else
        {
            for (int i = 0; i < fftSize; i++)
            {
                c = fftOut[i];
                v = c.real() * c.real() + c.imag() * c.imag();
                v *= m_powFFTMul;

                if (m_min.storeAndGetMin(min, v, i)) {
                    m_powerSpectrum[i] = min;
                }
            }
        }

        // result available
        if (m_min.nextMin()) {
            ready = true;
        }
    }

    if (ready)
    {
        for (int i = fftSize; i < m_settings.m_fftSize; i++) {
            m_powerSpectrum[i] = 0.0f;
        }

        // Calculate maximum value in spectrum
        m_specMax = *std::max_element(&m_powerSpectrum[0], &m_powerSpectrum[m_settings.m_fftSize]);

        // Perform math operation on linear value
        if (m_settings.m_mathMode != SpectrumSettings::MathModeNone) {
            mathLinear(m_powerSpectrum);
        }

        // Convert to dB
        if (!m_settings.m_linear)
        {
            for (int i = 0; i < m_settings.m_fftSize; i++) {
                m_powerSpectrum[i] = m_mult * log2fapprox(m_powerSpectrum[i]);
            }
        }

        // Perform math operation on dB value
        if (m_settings.m_mathMode != SpectrumSettings::MathModeNone) {
            mathDB(m_powerSpectrum);
        }

        if (m_settings.mathAverageUsed()) {
            m_mathMovingAverage.nextAverage();
        }

        // Stop profiling before newSpectrum, as we profile that separately
        PROFILER_STOP("processFFT");

        // send new data to visualisation
        if (m_glSpectrum)
        {
            m_glSpectrum->newSpectrum(
                &m_powerSpectrum.data()[0],
                m_settings.m_fftSize
            );
        }

        // web socket spectrum connections
        if (m_wsSpectrum.socketOpened())
        {
            m_wsSpectrum.newSpectrum(
                m_powerSpectrum,
                m_settings.m_fftSize,
                m_centerFrequency,
                m_sampleRate,
                m_settings.m_linear,
                m_settings.m_ssb,
                m_settings.m_usb
            );
        }
    }
    else
    {
        PROFILER_STOP("processFFT");
    }

}

void SpectrumVis::start()
{
    setRunning(true);

    if (getMessageQueueToGUI()) // propagate to GUI if any
    {
        MsgStartStop *msg = MsgStartStop::create(true);
        getMessageQueueToGUI()->push(msg);
    }
}

void SpectrumVis::stop()
{
    setRunning(false);

    if (getMessageQueueToGUI()) // propagate to GUI if any
    {
        MsgStartStop *msg = MsgStartStop::create(false);
        getMessageQueueToGUI()->push(msg);
    }
}

void SpectrumVis::pushMessage(Message *msg)
{
    m_inputMessageQueue.push(msg);
}

QString SpectrumVis::getSinkName()
{
    return objectName();
}

void SpectrumVis::handleInputMessages()
{
	Message* message;

	while ((message = m_inputMessageQueue.pop()) != 0)
	{
		if (handleMessage(*message)) {
			delete message;
		}
	}
}

bool SpectrumVis::handleMessage(const Message& message)
{
    if (DSPSignalNotification::match(message))
    {
        // This is coming from device engine and will apply to main spectrum
        DSPSignalNotification& notif = (DSPSignalNotification&) message;
        qDebug() << "SpectrumVis::handleMessage: DSPSignalNotification:"
            << " centerFrequency: " << notif.getCenterFrequency()
            << " sampleRate: " << notif.getSampleRate();
        handleConfigureDSP(notif.getCenterFrequency(), notif.getSampleRate());
        return true;
    }
	else if (MsgConfigureSpectrumVis::match(message))
	{
        MsgConfigureSpectrumVis& cfg = (MsgConfigureSpectrumVis&) message;
        qDebug() << "SpectrumVis::handleMessage: MsgConfigureSpectrumVis";
        applySettings(cfg.getSettings(), cfg.getForce());
		return true;
	}
    else if (MsgConfigureScalingFactor::match(message))
    {
        MsgConfigureScalingFactor& conf = (MsgConfigureScalingFactor&) message;
        handleScalef(conf.getScalef());
        return true;
    }
    else if (MsgConfigureWSpectrumOpenClose::match(message))
    {
        MsgConfigureWSpectrumOpenClose& conf = (MsgConfigureWSpectrumOpenClose&) message;
        handleWSOpenClose(conf.getOpenClose());
        return true;
    }
    else if (MsgConfigureWSpectrum::match(message)) {
        MsgConfigureWSpectrum& conf = (MsgConfigureWSpectrum&) message;
        handleConfigureWSSpectrum(conf.getAddress(), conf.getPort());
        return true;
    }
    else if (MsgStartStop::match(message))
    {
        MsgStartStop& cmd = (MsgStartStop&) message;
        setRunning(cmd.getStartStop());
        return true;
    }
	else
	{
		return false;
	}
}

void SpectrumVis::applySettings(const SpectrumSettings& settings, bool force)
{
    QMutexLocker mutexLocker(&m_mutex);

    int fftSize = settings.m_fftSize > (1<<SpectrumSettings::m_log2FFTSizeMax) ?
        (1<<SpectrumSettings::m_log2FFTSizeMax) :
        settings.m_fftSize < (1<<SpectrumSettings::m_log2FFTSizeMin) ?
            (1<<SpectrumSettings::m_log2FFTSizeMin) :
            settings.m_fftSize;

    qDebug() << "SpectrumVis::applySettings:"
        << " m_fftSize: " << fftSize
        << " m_fftWindow: " << settings.m_fftWindow
        << " m_fftOverlap: " << settings.m_fftOverlap
        << " m_averagingIndex: " << settings.m_averagingIndex
        << " m_averagingMode: " << settings.m_averagingMode
        << " m_refLevel: " << settings.m_refLevel
        << " m_powerRange: " << settings.m_powerRange
        << " m_fpsPeriodMs: " << settings.m_fpsPeriodMs
        << " m_linear: " << settings.m_linear
        << " m_ssb: " << settings.m_ssb
        << " m_usb: " << settings.m_usb
        << " m_wsSpectrumAddress: " << settings.m_wsSpectrumAddress
        << " m_wsSpectrumPort: " << settings.m_wsSpectrumPort
        << " force: " << force;

    if ((fftSize != m_settings.m_fftSize) || force)
    {
        FFTFactory *fftFactory = DSPEngine::instance()->getFFTFactory();

        // release previous engine allocation if any
        if (m_fft) {
            fftFactory->releaseEngine(m_settings.m_fftSize, false, m_fftEngineSequence);
        }

        m_fftEngineSequence = fftFactory->getEngine(fftSize, false, &m_fft);
        m_powFFTMul = 1.0f / (fftSize * fftSize);

        if (fftSize > m_settings.m_fftSize)
        {
            m_fftBuffer.resize(fftSize);
            m_powerSpectrum.resize(fftSize);
            m_mathMemory.resize(fftSize);
        }
    }

    if ((fftSize != m_settings.m_fftSize)
     || (settings.m_fftWindow != m_settings.m_fftWindow) || force)
    {
        m_window.create(settings.m_fftWindow, fftSize);
    }

    if ((fftSize != m_settings.m_fftSize)
     || (settings.m_fftOverlap != m_settings.m_fftOverlap) || force)
    {
		m_overlapSize = settings.m_fftOverlap < 0 ? 0 :
			settings.m_fftOverlap < fftSize ? settings.m_fftOverlap : (fftSize - 1);
        m_refillSize = fftSize - m_overlapSize;
        m_fftBufferFill = m_overlapSize;
    }

    if ((fftSize != m_settings.m_fftSize)
     || (settings.m_averagingIndex != m_settings.m_averagingIndex)
     || (settings.m_averagingMode != m_settings.m_averagingMode) || force)
    {
        unsigned int averagingValue = SpectrumSettings::getAveragingValue(settings.m_averagingIndex, settings.m_averagingMode);
        averagingValue = averagingValue > SpectrumSettings::getMaxAveragingValue(fftSize, settings.m_averagingMode) ?
            SpectrumSettings::getMaxAveragingValue(fftSize, settings.m_averagingMode) : averagingValue; // Capping to avoid out of memory condition
        m_movingAverage.resize(fftSize, averagingValue);
        m_fixedAverage.resize(fftSize, averagingValue);
        m_max.resize(fftSize, averagingValue);
        m_min.resize(fftSize, averagingValue);
    }

    if ((fftSize != m_settings.m_fftSize)
        || (settings.m_mathAvgCount != m_settings.m_mathAvgCount) || force)
    {
        m_mathMovingAverage.resize(fftSize, settings.m_mathAvgCount);
    }

    if (settings.m_mathMode != m_settings.m_mathMode) {
        m_mathMovingAverage.clear();
    }

    if ((settings.m_wsSpectrumAddress != m_settings.m_wsSpectrumAddress)
     || (settings.m_wsSpectrumPort != m_settings.m_wsSpectrumPort) || force) {
         handleConfigureWSSpectrum(settings.m_wsSpectrumAddress, settings.m_wsSpectrumPort);
    }

    if ((fftSize != m_settings.m_fftSize)
     || (settings.m_transformType != m_settings.m_transformType)
     || (settings.m_cwtTimeSteps != m_settings.m_cwtTimeSteps) || force)
    {
        const int cwtFftSize = fftSize * settings.m_cwtTimeSteps;  // Large N_total-point FFT size

        if (settings.m_transformType == SpectrumSettings::CWT)
        {
            // Release the previous CWT FFT engine when the size or depth changes while CWT is active.
            if (m_cwtFft) {
                fftFactory->releaseEngine(
                    static_cast<unsigned int>(m_settings.m_fftSize) * m_settings.m_cwtTimeSteps,
                    false, m_cwtFftEngineSequence);
            }
            m_cwtFftEngineSequence = fftFactory->getEngine(
                static_cast<unsigned int>(cwtFftSize), false, &m_cwtFft);
            m_cwtPowFFTMul = 1.0f / (static_cast<float>(cwtFftSize) * static_cast<float>(cwtFftSize));
            buildCWTWeightTables(fftSize, settings.m_cwtTimeSteps);
            m_cwtFftPower.resize(static_cast<std::size_t>(cwtFftSize));
            // Zero-initialise the history buffer so the first m_cwtTimeSteps-1 rows start cleanly.
            m_cwtBuffer.assign(static_cast<std::size_t>(cwtFftSize), Complex{0.0f, 0.0f});
            m_cwtBufferFill = 0;
        }
        else if (m_cwtFft)
        {
            // Leaving CWT mode: release engine and free all associated memory.
            fftFactory->releaseEngine(
                static_cast<unsigned int>(m_settings.m_fftSize) * m_settings.m_cwtTimeSteps,
                false, m_cwtFftEngineSequence);
            m_cwtFft = nullptr;
            m_cwtWeightsFull.clear();
            m_cwtWeightsFull.shrink_to_fit();
            m_cwtWeightsPos.clear();
            m_cwtWeightsPos.shrink_to_fit();
            m_cwtBuffer.clear();
            m_cwtBuffer.shrink_to_fit();
            m_cwtFftPower.clear();
            m_cwtFftPower.shrink_to_fit();
            m_cwtBufferFill = 0;
        }
    }

    // Recreate the CWT analysis window whenever its type, size or depth changes.
    // This is a fast operation so it is safe to run even if only the window type changed
    // without a size or mode change (the engine block above handles size/mode changes).
    if (settings.m_transformType == SpectrumSettings::CWT
     && ((fftSize != m_settings.m_fftSize)
      || (settings.m_fftWindow != m_settings.m_fftWindow)
      || (settings.m_transformType != m_settings.m_transformType)
      || (settings.m_cwtTimeSteps != m_settings.m_cwtTimeSteps)
      || force))
    {
        m_cwtWindow.create(settings.m_fftWindow, fftSize * settings.m_cwtTimeSteps);
    }

    m_settings = settings;
    m_settings.m_fftSize = fftSize;

    if (m_guiMessageQueue)
    {
        MsgConfigureSpectrumVis *msg = MsgConfigureSpectrumVis::create(m_settings, false);
        m_guiMessageQueue->push(msg);
    }
}

void SpectrumVis::handleConfigureDSP(uint64_t centerFrequency, int sampleRate)
{
    QMutexLocker mutexLocker(&m_mutex);
    m_centerFrequency = centerFrequency;
    m_sampleRate = sampleRate;
}

void SpectrumVis::handleScalef(Real scalef)
{
    QMutexLocker mutexLocker(&m_mutex);
    m_scalef = scalef;
}

void SpectrumVis::handleWSOpenClose(bool openClose)
{
    QMutexLocker mutexLocker(&m_mutex);

    if (openClose) {
        m_wsSpectrum.openSocket();
    } else {
        m_wsSpectrum.closeSocket();
    }
}

void SpectrumVis::handleConfigureWSSpectrum(const QString& address, uint16_t port)
{
    m_wsSpectrum.setListeningAddress(address);
    m_wsSpectrum.setPort(port);

    if (m_wsSpectrum.socketOpened())
    {
        m_wsSpectrum.closeSocket();
        m_wsSpectrum.openSocket();
    }
}

int SpectrumVis::webapiSpectrumSettingsGet(SWGSDRangel::SWGGLSpectrum& response, QString& errorMessage) const
{
    (void) errorMessage;
    response.init();
    webapiFormatSpectrumSettings(response, m_settings);
    return 200;
}

int SpectrumVis::webapiSpectrumSettingsPutPatch(
    bool force,
    const QStringList& spectrumSettingsKeys,
    SWGSDRangel::SWGGLSpectrum& response, // query + response
    QString& errorMessage)
{
    (void) errorMessage;
    SpectrumSettings settings = m_settings;
    webapiUpdateSpectrumSettings(settings, spectrumSettingsKeys, response);

    MsgConfigureSpectrumVis *msg = MsgConfigureSpectrumVis::create(settings, force);
    m_inputMessageQueue.push(msg);

    if (getMessageQueueToGUI()) // forward to GUI if any
    {
        MsgConfigureSpectrumVis *msgToGUI = MsgConfigureSpectrumVis::create(settings, force);
        getMessageQueueToGUI()->push(msgToGUI);
    }

    webapiFormatSpectrumSettings(response, settings);
    return 200;
}

int SpectrumVis::webapiSpectrumServerGet(SWGSDRangel::SWGSpectrumServer& response, QString& errorMessage) const
{
    (void) errorMessage;
    bool serverRunning = m_wsSpectrum.socketOpened();
    QList<QHostAddress> peerHosts;
    QList<quint16> peerPorts;
    m_wsSpectrum.getPeers(peerHosts, peerPorts);
    response.init();
    response.setRun(serverRunning ? 1 : 0);

    QHostAddress serverAddress = m_wsSpectrum.getListeningAddress();

    if (serverAddress != QHostAddress::Null) {
        response.setListeningAddress(new QString(serverAddress.toString()));
    }

    uint16_t serverPort = m_wsSpectrum.getListeningPort();

    if (serverPort != 0) {
        response.setListeningPort(serverPort);
    }

    if (peerHosts.size() > 0)
    {
        response.setClients(new QList<SWGSDRangel::SWGSpectrumServer_clients*>);

        for (int i = 0; i < peerHosts.size(); i++)
        {
            response.getClients()->push_back(new SWGSDRangel::SWGSpectrumServer_clients);
            response.getClients()->back()->setAddress(new QString(peerHosts.at(i).toString()));
            response.getClients()->back()->setPort(peerPorts.at(i));
        }
    }

    return 200;
}

int SpectrumVis::webapiSpectrumServerPost(SWGSDRangel::SWGSuccessResponse& response, QString& errorMessage)
{
    (void) errorMessage;
    MsgConfigureWSpectrumOpenClose *msg = MsgConfigureWSpectrumOpenClose::create(true);
    m_inputMessageQueue.push(msg);

    if (getMessageQueueToGUI()) // forward to GUI if any
    {
        MsgConfigureWSpectrumOpenClose *msgToGui = MsgConfigureWSpectrumOpenClose::create(true);
        getMessageQueueToGUI()->push(msgToGui);
    }

    response.setMessage(new QString("Websocket spectrum server started"));
    return 200;
}

int SpectrumVis::webapiSpectrumServerDelete(SWGSDRangel::SWGSuccessResponse& response, QString& errorMessage)
{
    (void) errorMessage;
    MsgConfigureWSpectrumOpenClose *msg = MsgConfigureWSpectrumOpenClose::create(false);
    m_inputMessageQueue.push(msg);

    if (getMessageQueueToGUI()) // forward to GUI if any
    {
        MsgConfigureWSpectrumOpenClose *msgToGui = MsgConfigureWSpectrumOpenClose::create(false);
        getMessageQueueToGUI()->push(msgToGui);
    }

    response.setMessage(new QString("Websocket spectrum server stopped"));
    return 200;
}

void SpectrumVis::webapiFormatSpectrumSettings(SWGSDRangel::SWGGLSpectrum& response, const SpectrumSettings& settings)
{
    settings.formatTo(&response);
}

void SpectrumVis::webapiUpdateSpectrumSettings(
    SpectrumSettings& settings,
    const QStringList& spectrumSettingsKeys,
    SWGSDRangel::SWGGLSpectrum& response)
{
    QStringList prefixedKeys;

    for (const auto &key : spectrumSettingsKeys) {
        prefixedKeys.append(tr("spectrumConfig.%1").arg(key));
    }

    settings.updateFrom(prefixedKeys, &response);
}

// To calculate power, the usual equation:
//    10*log10(v=V1/V2), where V2=fftSize^2
// is calculated using log2 instead, with:
//   mult = 10.0f / log2(10.0f)
//   dB = m_mult * log2f(v)
// However, while the gcc version of log2f is twice as fast as log10f,
// MSVC version is 6x slower.
// Also, we don't need full accuracy of log2f for calculating the power for the spectrum,
// so we can use the following approximation to get a good speed-up for both compilers:
// https://www.vplesko.com/posts/replacing_log2f.html
// https://www.vplesko.com/assets/replacing_log2f/main.c.txt
float SpectrumVis::log2fapprox(float x) const
{
    // IEEE 754 representation constants.
    const int32_t mantissaLen = 23;
    const int32_t mantissaMask = (1 << mantissaLen) - 1;
    const int32_t baseExponent = -127;

    // Reinterpret x as int in a standard compliant way.
    int32_t xi;
    memcpy(&xi, &x, sizeof(xi));

    // Calculate exponent of x.
    float e = (float)((xi >> mantissaLen) + baseExponent);

    // Calculate mantissa of x. It will be in range [1, 2).
    float m;
    int32_t mxi = (xi & mantissaMask) | ((-baseExponent) << mantissaLen);
    memcpy(&m, &mxi, sizeof(m));

    // Use Remez algorithm-generated approximation polynomial
    // for log2(a) where a is in range [1, 2].
    float l = 0.15824871f;
    l = l * m + -1.051875f;
    l = l * m + 3.0478842f;
    l = l * m + -2.1536207f;

    // Add exponent to the calculation.
    // Final log is log2(m*2^e)=log2(m)+e.
    l += e;

    return l;
}

void SpectrumVis::setMathMemory(const QList<Real> &values)
{
    QMutexLocker mutexLocker(&m_mutex);

    int s = std::min((int) values.size(), m_settings.m_fftSize);

    for (int i = 0; i < s; i++) {
        m_mathMemory[i] = values[i];
    }
}

void SpectrumVis::getMathMovingAverageCopy(QList<Real>& copy)
{
    QMutexLocker mutexLocker(&m_mutex);

    std::vector<Real> averages;
    m_mathMovingAverage.getAverages<Real>(averages);
    copy = QList<Real>(averages.begin(), averages.end());
}
