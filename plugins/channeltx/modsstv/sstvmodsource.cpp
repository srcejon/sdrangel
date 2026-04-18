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

#include <cmath>
#include <QDebug>

#include "dsp/dsptypes.h"
#include "dsp/basebandsamplesink.h"
#include "dsp/scopevis.h"
#include "sstvmodsource.h"

SSTVModSource::SSTVModSource(QObject *parent)
    : QObject(parent)
{
    m_scopeSampleBuffer.resize(m_scopeSampleBufferSize);
    m_specSampleBuffer.resize(m_specSampleBufferSize);
    applySettings(QStringList(), m_settings, true);
    applyChannelSettings(m_channelSampleRate, m_channelFrequencyOffset, true);
}

SSTVModSource::~SSTVModSource() = default;

// ---------------------------------------------------------------------------
// ChannelSampleSource interface
// ---------------------------------------------------------------------------

void SSTVModSource::pull(SampleVector::iterator begin, unsigned int nbSamples)
{
    std::for_each(
        begin,
        begin + nbSamples,
        [this](Sample& s) { pullOne(s); }
    );
}

void SSTVModSource::pullOne(Sample& sample)
{
    QMutexLocker lock(&m_mutex);

    Complex ci;

    if (m_interpolatorDistance > 1.0f) // decimate
    {
        modulateSample();

        while (!m_interpolator.decimate(&m_interpolatorDistanceRemain, m_modSample, &ci))
        {
            modulateSample();
        }
    }
    else // interpolate
    {
        if (m_interpolator.interpolate(&m_interpolatorDistanceRemain, m_modSample, &ci))
        {
            modulateSample();
        }
    }

    m_interpolatorDistanceRemain += m_interpolatorDistance;

    ci *= m_carrierNco.nextIQ();

    const double magsq = (ci.real() * ci.real() + ci.imag() * ci.imag()) / (SDR_TX_SCALED * SDR_TX_SCALED);
    m_movingAverage(magsq);
    m_magsq = m_movingAverage.asDouble();

    sample.m_real = (FixReal) ci.real();
    sample.m_imag = (FixReal) ci.imag();
}

void SSTVModSource::prefetch(unsigned int /*nbSamples*/)
{
    // Nothing to prefetch for a purely generated signal
}

/** Generate and store one baseband sample at SSTV_SAMPLE_RATE into m_modSample,
 *  then advance the encoder state machine by one SSTV_SAMPLE_RATE sample. */
void SSTVModSource::modulateSample()
{
    if (m_state == State::IDLE)
    {
        m_modSample = Complex(0.0f, 0.0f);
        return;
    }

    m_modSample = generateSample();

    // Advance state machine
    if (m_stateSamples > 0)
    {
        --m_stateSamples;
    }
    else
    {
        // Handle pixel sections differently (fractional samples per pixel)
        if (isPixelState(m_state))
        {
            m_pixelFrac += 1.0f;
            if (m_pixelFrac >= m_currentPixelSamples)
            {
                m_pixelFrac -= m_currentPixelSamples;
                ++m_pixelIndex;
                if (m_pixelIndex >= m_currentSectionWidth)
                {
                    // Section complete
                    advanceState();
                }
                else
                {
                    // Update tone for next pixel
                    m_currentFreq = getPixelFreq(m_linePair, m_state, m_pixelIndex);
                }
            }
            return;
        }
        advanceState();
    }
}

// ---------------------------------------------------------------------------
// Public control
// ---------------------------------------------------------------------------

void SSTVModSource::startTransmit()
{
    QMutexLocker lock(&m_mutex);

    QImage image(m_settings.m_imagePath);
    if (!image.isNull())
    {
        // Scale to active mode dimensions
        QImage scaled = image.scaled(m_modeWidth, m_modeHeight, Qt::IgnoreAspectRatio, Qt::SmoothTransformation)
            .convertToFormat(QImage::Format_RGB888);

        m_imageData.resize(m_modeWidth * m_modeHeight * 3);
        const uchar* src = scaled.constBits();
        memcpy(m_imageData.data(), src, m_imageData.size());
        m_imageValid = true;
        qDebug() << "SSTVModSource::loadImage: loaded" << m_modeWidth << "×" << m_modeHeight;

        const SSTVModSettings::SSTVModeParams modeParams = SSTVModSettings::getModeParams(m_settings.m_sstvMode);
        qDebug("SSTVModSource::startTransmit: starting SSTV transmission (mode %d)", (int) m_settings.m_sstvMode);
        m_linePair = 0;
        m_pixelIndex = 0;
        m_pixelFrac = 0.0f;
        m_fmPhasor = 0.0f;
        m_tonePhasor = 0.0f;
        // Build VIS byte: 7 data bits + even-parity bit
        uint8_t vis7 = modeParams.visCode & 0x7F;
        int ones = 0;
        for (int i = 0; i < 7; i++) {
            if (vis7 & (1 << i)) {
                ++ones;
            }
        }
        // Even parity: bit7 set if odd number of 1s in bits 0-6
        m_visData = vis7 | ((ones & 1) ? 0x80 : 0x00);
        m_visBit = 0;
        enterState(State::LEADER1, SSTV_LEADER_SAMPLES);
    }
    else
    {
        m_imageValid = false;
    }
}

void SSTVModSource::stopTransmit()
{
    QMutexLocker lock(&m_mutex);
    m_state = State::IDLE;
    m_stateSamples = 0;
    qDebug("SSTVModSource::stopTransmit");
}

// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------

void SSTVModSource::applySettings(const QStringList& settingsKeys, const SSTVModSettings& settings, bool force)
{
    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (force || settingsKeys.contains("sstvMode")) {
        applyMode();
    }
}

void SSTVModSource::applyMode()
{
    const SSTVModSettings::SSTVModeParams p = SSTVModSettings::getModeParams(m_settings.m_sstvMode);
    m_modeWidth             = p.width;
    m_modeHeight            = p.height;
    m_modeLinePairs         = p.linesTotal;
    m_modePixelSamples      = p.pixelTimeMs * float(SSTV_SAMPLE_RATE) / 1000.0f;
    m_modeSyncSamples       = (int)(p.syncMs  * float(SSTV_SAMPLE_RATE) / 1000.0f + 0.5f);
    m_modePorchSamples      = (int)(p.porchMs * float(SSTV_SAMPLE_RATE) / 1000.0f + 0.5f);
    m_modeSeparatorSamples  = (int)(p.separatorMs * float(SSTV_SAMPLE_RATE) / 1000.0f + 0.5f);
    m_modeChromaWidth       = p.chromaWidth;
    m_modeChromaPixelSamples = p.chromaPixelTimeMs * float(SSTV_SAMPLE_RATE) / 1000.0f;
}

void SSTVModSource::applyChannelSettings(int channelSampleRate, int channelFrequencyOffset, bool force)
{
    if ((channelFrequencyOffset != m_channelFrequencyOffset) || (channelSampleRate != m_channelSampleRate) || force) {
        m_carrierNco.setFreq(channelFrequencyOffset, channelSampleRate);
    }

    if ((channelSampleRate != m_channelSampleRate) || force)
    {
        m_interpolatorDistanceRemain = 0.0f;
        m_interpolatorDistance = static_cast<Real>(SSTV_SAMPLE_RATE) / static_cast<Real>(channelSampleRate);
        m_interpolator.create(48, SSTV_SAMPLE_RATE, m_settings.m_rfBandwidth / 2.2, 3.0);
    }

    m_channelSampleRate = channelSampleRate;
    m_channelFrequencyOffset = channelFrequencyOffset;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Spectrum / scope feed helpers
// ---------------------------------------------------------------------------

void SSTVModSource::sampleToSpectrum(Complex sample)
{
    if (m_spectrumSink)
    {
        Real r = std::real(sample) * SDR_TX_SCALEF;
        Real i = std::imag(sample) * SDR_TX_SCALEF;
        m_specSampleBuffer[m_specSampleBufferIndex++] = Sample(r, i);

        if (m_specSampleBufferIndex == m_specSampleBufferSize)
        {
            m_spectrumSink->feed(m_specSampleBuffer.begin(), m_specSampleBuffer.end(), false);
            m_specSampleBufferIndex = 0;
        }
    }
}

void SSTVModSource::sampleToScope(Complex sample)
{
    if (m_scopeSink)
    {
        Real r = std::real(sample) * SDR_RX_SCALEF;
        Real i = std::imag(sample) * SDR_RX_SCALEF;
        m_scopeSampleBuffer[m_scopeSampleBufferIndex++] = Sample(r, i);

        if (m_scopeSampleBufferIndex == m_scopeSampleBufferSize)
        {
            std::vector<SampleVector::const_iterator> vbegin;
            vbegin.push_back(m_scopeSampleBuffer.begin());
            m_scopeSink->feed(vbegin, m_scopeSampleBufferSize);
            m_scopeSampleBufferIndex = 0;
        }
    }
}

void SSTVModSource::finishTransmission()
{
    if (m_settings.m_repeat)
    {
        startTransmit();
    }
    else
    {
        m_state = State::DONE;
        m_stateSamples = 0;
        emit transmitComplete();
        m_state = State::IDLE;
    }
}

bool SSTVModSource::isPixelState(State s) const
{
    switch (s)
    {
        case State::LINE_Y_ODD:
        case State::LINE_CR:
        case State::LINE_CB:
        case State::LINE_Y_EVEN:
        case State::R36_Y:
        case State::R36_CHROMA:
        case State::SC_GREEN:
        case State::SC_BLUE:
        case State::SC_RED:
        case State::MT_GREEN:
        case State::MT_BLUE:
        case State::MT_RED:
            return true;
        default:
            return false;
    }
}

/** Map pixel luminance/chroma value [0..255] to SSTV tone frequency [Hz]. */
float SSTVModSource::pixelToFreq(int value) const
{
    return SSTV_BLACK_FREQ + (float(value) / 255.0f) * (SSTV_WHITE_FREQ - SSTV_BLACK_FREQ);
}

/** Return the tone frequency for pixel column `col` of the current section. */
float SSTVModSource::getPixelFreq(int line, State section, int col) const
{
    if (!m_imageValid || m_imageData.isEmpty()) {
        return SSTV_BLACK_FREQ;
    }

    const int lineOdd  = 2 * line;
    const int lineEven = 2 * line + 1;

    auto getPixel = [&](int l, int x) -> const uchar*
    {
        int lineClamp = qBound(0, l, m_modeHeight - 1);
        int xClamp    = qBound(0, x, m_modeWidth  - 1);
        return reinterpret_cast<const uchar*>(
            m_imageData.constData() + (lineClamp * m_modeWidth + xClamp) * 3
        );
    };

    const uchar* pOdd  = getPixel(lineOdd, col);
    const uchar* pEven = getPixel(lineEven, col);

    float rO = pOdd[0], gO = pOdd[1], bO = pOdd[2];
    float rE = pEven[0], gE = pEven[1], bE = pEven[2];

    int value = 0;
    switch (section)
    {
        case State::LINE_Y_ODD:
        {
            float y = 0.299f * rO + 0.587f * gO + 0.114f * bO;
            value = qBound(0, (int) y, 255);
            break;
        }
        case State::LINE_CR:
        {
            float rAvg = (rO + rE) * 0.5f;
            float gAvg = (gO + gE) * 0.5f;
            float bAvg = (bO + bE) * 0.5f;
            float cr = 128.0f + 0.5f * rAvg - 0.4187f * gAvg - 0.0813f * bAvg;
            value = qBound(0, (int) cr, 255);
            break;
        }
        case State::LINE_CB:
        {
            float rAvg = (rO + rE) * 0.5f;
            float gAvg = (gO + gE) * 0.5f;
            float bAvg = (bO + bE) * 0.5f;
            float cb = 128.0f - 0.1687f * rAvg - 0.3313f * gAvg + 0.5f * bAvg;
            value = qBound(0, (int) cb, 255);
            break;
        }
        case State::LINE_Y_EVEN:
        {
            float y = 0.299f * rE + 0.587f * gE + 0.114f * bE;
            value = qBound(0, (int) y, 255);
            break;
        }
        // Robot36: Y section — luminance of current scan line
        case State::R36_Y:
        {
            const uchar* p = getPixel(line, col);
            float y = 0.299f * p[0] + 0.587f * p[1] + 0.114f * p[2];
            value = qBound(0, (int) y, 255);
            break;
        }
        // Robot36: chroma section — Cr for even lines, Cb for odd lines
        // Each chroma pixel col covers image columns 2*col and 2*col+1
        case State::R36_CHROMA:
        {
            const uchar* p0 = getPixel(line, 2 * col);
            const uchar* p1 = getPixel(line, 2 * col + 1);
            float r = (p0[0] + p1[0]) * 0.5f;
            float g = (p0[1] + p1[1]) * 0.5f;
            float b = (p0[2] + p1[2]) * 0.5f;
            if (line % 2 == 0) {
                // Even line: Cr
                value = qBound(0, (int)(128.0f + 0.5f * r - 0.4187f * g - 0.0813f * b), 255);
            } else {
                // Odd line: Cb
                value = qBound(0, (int)(128.0f - 0.1687f * r - 0.3313f * g + 0.5f * b), 255);
            }
            break;
        }
        // Scottie and Martin: direct RGB channels
        case State::SC_GREEN:
        case State::MT_GREEN:
        {
            const uchar* p = getPixel(line, col);
            value = qBound(0, (int) p[1], 255);
            break;
        }
        case State::SC_BLUE:
        case State::MT_BLUE:
        {
            const uchar* p = getPixel(line, col);
            value = qBound(0, (int) p[2], 255);
            break;
        }
        case State::SC_RED:
        case State::MT_RED:
        {
            const uchar* p = getPixel(line, col);
            value = qBound(0, (int) p[0], 255);
            break;
        }
        default:
            return SSTV_BLACK_FREQ;
    }

    return pixelToFreq(value);
}

/** Generate one complex baseband sample at the current SSTV tone frequency.
 *
 *  Step 1: advance the audio-tone phasor at m_currentFreq to produce a real
 *          sinusoidal audio sample (the SSTV sub-carrier tone).
 *  Step 2: modulate that audio onto the baseband carrier:
 *    FM  – integrate audio sample into the FM carrier phase using fmDeviation.
 *          Δφ = 2π × fmDeviation[Hz] × audio / sampleRate
 *    USB – output the analytic signal of the tone: exp(+j·ωt)
 *    LSB – output the conjugate analytic signal:   exp(−j·ωt)
 *
 *  Both phasors are continuous across state transitions so there are no clicks. */
Complex SSTVModSource::generateSample()
{
    const float twoPi = 2.0f * static_cast<float>(M_PI);
    const float scale = 0.891235351562f * SDR_TX_SCALEF; // -1 dB

    // Advance SSTV audio-tone phasor (shared by all modulation modes)
    m_tonePhasor += twoPi * m_currentFreq / static_cast<float>(SSTV_SAMPLE_RATE);
    if (m_tonePhasor > static_cast<float>(M_PI)) {
        m_tonePhasor -= twoPi;
    }
    const float audio = std::cos(m_tonePhasor); // real SSTV audio tone [-1 .. +1]

    Complex ci;

    if (m_settings.m_modulation == SSTVModSettings::ModulationFM)
    {
        // FM: integrate audio amplitude into carrier phase.
        // At peak (audio = ±1) the instantaneous frequency deviates by ±fmDeviation Hz.
        m_fmPhasor += twoPi * m_settings.m_fmDeviation * audio / static_cast<float>(SSTV_SAMPLE_RATE);
        if (m_fmPhasor > static_cast<float>(M_PI)) {
            m_fmPhasor -= twoPi;
        }
        ci.real(std::cos(m_fmPhasor) * scale);
        ci.imag(std::sin(m_fmPhasor) * scale);
    }
    else if (m_settings.m_modulation == SSTVModSettings::ModulationUSB)
    {
        // USB SSB: analytic signal exp(+j·2π·f_tone·t)
        ci.real( std::cos(m_tonePhasor) * scale);
        ci.imag( std::sin(m_tonePhasor) * scale);
    }
    else // LSB
    {
        // LSB SSB: conjugate analytic signal exp(−j·2π·f_tone·t)
        ci.real( std::cos(m_tonePhasor) * scale);
        ci.imag(-std::sin(m_tonePhasor) * scale);
    }

    // Feed the audio tone (real SSTV sub-carrier) to spectrum and scope.
    // Represented as a complex with audio on the real axis, 0 on imaginary.
    sampleToSpectrum(Complex(audio, 0.0f));
    sampleToScope(Complex(audio, 0.0f));

    return ci;
}

/** Enter a new encoder state with `samples` remaining. */
void SSTVModSource::enterState(State s, int samples)
{
    m_state = s;
    m_stateSamples = samples - 1; // -1 because the current sample is also consumed

    switch (s)
    {
        case State::LEADER1:
        case State::LEADER2:
            m_currentFreq = SSTV_LEADER_FREQ;
            break;
        case State::BREAK:
        case State::VIS_START:
        case State::VIS_STOP:
        case State::LINE_SYNC:
            m_currentFreq = SSTV_SYNC_FREQ;
            break;
        case State::LINE_PORCH:
            m_currentFreq = SSTV_PORCH_FREQ;
            break;
        case State::VIS_BITS:
            // Frequency is set per-bit in advanceState
            break;
        case State::LINE_Y_ODD:
        case State::LINE_CR:
        case State::LINE_CB:
        case State::LINE_Y_EVEN:
            m_currentSectionWidth  = m_modeWidth;
            m_currentPixelSamples  = m_modePixelSamples;
            m_pixelIndex = 0;
            m_pixelFrac  = 0.0f;
            m_stateSamples = 0; // pixel sections are driven by m_pixelFrac, not m_stateSamples
            m_currentFreq = getPixelFreq(m_linePair, s, 0);
            break;
        // New non-pixel timed states
        case State::R36_SYNC:
        case State::SC_SYNC:
        case State::MT_SYNC:
            m_currentFreq = SSTV_SYNC_FREQ;
            break;
        case State::R36_PORCH:
        case State::SC_INIT_PORCH:
        case State::SC_PORCH:
        case State::SC_PORCH2:
        case State::MT_PORCH:
        case State::R36_SEP:
            m_currentFreq = SSTV_PORCH_FREQ;
            break;
        // New pixel states
        case State::R36_Y:
        case State::SC_GREEN:
        case State::SC_BLUE:
        case State::SC_RED:
        case State::MT_GREEN:
        case State::MT_BLUE:
        case State::MT_RED:
            m_currentSectionWidth  = m_modeWidth;
            m_currentPixelSamples  = m_modePixelSamples;
            m_pixelIndex = 0;
            m_pixelFrac  = 0.0f;
            m_stateSamples = 0;
            m_currentFreq = getPixelFreq(m_linePair, s, 0);
            break;
        case State::R36_CHROMA:
            m_currentSectionWidth  = m_modeChromaWidth;
            m_currentPixelSamples  = m_modeChromaPixelSamples;
            m_pixelIndex = 0;
            m_pixelFrac  = 0.0f;
            m_stateSamples = 0;
            m_currentFreq = getPixelFreq(m_linePair, s, 0);
            break;
        default:
            break;
    }
}

/** Called when a state's sample count expires; transitions to the next state. */
void SSTVModSource::advanceState()
{
    switch (m_state)
    {
        case State::LEADER1:
            enterState(State::BREAK, SSTV_BREAK_SAMPLES);
            break;
        case State::BREAK:
            enterState(State::LEADER2, SSTV_LEADER_SAMPLES);
            break;
        case State::LEADER2:
            enterState(State::VIS_START, SSTV_VIS_BIT_SAMPLES);
            break;
        case State::VIS_START:
            m_visBit = 0;
            enterState(State::VIS_BITS, SSTV_VIS_BIT_SAMPLES);
            // Set first bit frequency
            m_currentFreq = (m_visData & (1 << m_visBit)) ? SSTV_VIS_ONE_FREQ : SSTV_VIS_ZERO_FREQ;
            break;
        case State::VIS_BITS:
            ++m_visBit;
            if (m_visBit < 8)
            {
                enterState(State::VIS_BITS, SSTV_VIS_BIT_SAMPLES);
                m_currentFreq = (m_visData & (1 << m_visBit)) ? SSTV_VIS_ONE_FREQ : SSTV_VIS_ZERO_FREQ;
            }
            else
            {
                enterState(State::VIS_STOP, SSTV_VIS_BIT_SAMPLES);
            }
            break;
        case State::VIS_STOP:
        {
            m_linePair = 0;
            const SSTVModSettings::SSTVModeFamily family = SSTVModSettings::getModeParams(m_settings.m_sstvMode).family;
            if (family == SSTVModSettings::SSTVModeFamily::PD) {
                enterState(State::LINE_SYNC, m_modeSyncSamples);
            } else if (family == SSTVModSettings::SSTVModeFamily::Robot36) {
                enterState(State::R36_SYNC, m_modeSyncSamples);
            } else if (family == SSTVModSettings::SSTVModeFamily::Scottie) {
                enterState(State::SC_INIT_PORCH, m_modePorchSamples);
            } else {
                enterState(State::MT_SYNC, m_modeSyncSamples);
            }
            break;
        }
        case State::LINE_SYNC:
            enterState(State::LINE_PORCH, m_modePorchSamples);
            break;
        case State::LINE_PORCH:
            enterState(State::LINE_Y_ODD, 0);
            break;
        case State::LINE_Y_ODD:
            enterState(State::LINE_CR, 0);
            break;
        case State::LINE_CR:
            enterState(State::LINE_CB, 0);
            break;
        case State::LINE_CB:
            enterState(State::LINE_Y_EVEN, 0);
            break;
        case State::LINE_Y_EVEN:
            ++m_linePair;
            if (m_linePair < m_modeLinePairs)
            {
                enterState(State::LINE_SYNC, m_modeSyncSamples);
            }
            else
            {
                qDebug("SSTVModSource: PD transmission complete");
                finishTransmission();
            }
            break;

        // ---- Robot36 ----
        case State::R36_SYNC:
            enterState(State::R36_PORCH, m_modePorchSamples);
            break;
        case State::R36_PORCH:
            enterState(State::R36_Y, 0);
            break;
        case State::R36_Y:
            enterState(State::R36_SEP, m_modeSeparatorSamples);
            break;
        case State::R36_SEP:
            enterState(State::R36_CHROMA, 0);
            break;
        case State::R36_CHROMA:
            ++m_linePair;
            if (m_linePair < m_modeLinePairs)
            {
                enterState(State::R36_SYNC, m_modeSyncSamples);
            }
            else
            {
                qDebug("SSTVModSource: Robot36 transmission complete");
                finishTransmission();
            }
            break;

        // ---- Scottie ----
        case State::SC_INIT_PORCH:
            enterState(State::SC_GREEN, 0);
            break;
        case State::SC_GREEN:
            enterState(State::SC_PORCH, m_modePorchSamples);
            break;
        case State::SC_PORCH:
            enterState(State::SC_BLUE, 0);
            break;
        case State::SC_BLUE:
            enterState(State::SC_SYNC, m_modeSyncSamples);
            break;
        case State::SC_SYNC:
            enterState(State::SC_PORCH2, m_modePorchSamples);
            break;
        case State::SC_PORCH2:
            enterState(State::SC_RED, 0);
            break;
        case State::SC_RED:
            ++m_linePair;
            if (m_linePair < m_modeHeight)
            {
                enterState(State::SC_GREEN, 0);
            }
            else
            {
                qDebug("SSTVModSource: Scottie transmission complete");
                finishTransmission();
            }
            break;

        // ---- Martin ----
        case State::MT_SYNC:
            enterState(State::MT_PORCH, m_modePorchSamples);
            break;
        case State::MT_PORCH:
            enterState(State::MT_GREEN, 0);
            break;
        case State::MT_GREEN:
            enterState(State::MT_BLUE, 0);
            break;
        case State::MT_BLUE:
            enterState(State::MT_RED, 0);
            break;
        case State::MT_RED:
            ++m_linePair;
            if (m_linePair < m_modeHeight)
            {
                enterState(State::MT_SYNC, m_modeSyncSamples);
            }
            else
            {
                qDebug("SSTVModSource: Martin transmission complete");
                finishTransmission();
            }
            break;
        default:
            m_state = State::IDLE;
            break;
    }
}
