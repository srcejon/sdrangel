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
#include "sstvmodsource.h"

SSTVModSource::SSTVModSource(QObject *parent)
    : QObject(parent)
{
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

    if (m_state == State::IDLE)
    {
        sample.m_real = 0;
        sample.m_imag = 0;
        return;
    }

    Complex ci = generateSample();
    ci *= m_carrierNco.nextIQ();

    const double magsq = (ci.real() * ci.real() + ci.imag() * ci.imag()) / (SDR_TX_SCALED * SDR_TX_SCALED);
    m_movingAverage(magsq);
    m_magsq = m_movingAverage.asDouble();

    sample.m_real = (FixReal) ci.real();
    sample.m_imag = (FixReal) ci.imag();

    // Advance state machine
    if (m_stateSamples > 0)
    {
        --m_stateSamples;
    }
    else
    {
        // Handle pixel sections differently (fractional samples per pixel)
        if (m_state == State::LINE_Y_ODD  ||
            m_state == State::LINE_CR     ||
            m_state == State::LINE_CB     ||
            m_state == State::LINE_Y_EVEN)
        {
            m_pixelFrac += 1.0f;
            if (m_pixelFrac >= SSTV_PIXEL_SAMPLES)
            {
                m_pixelFrac -= SSTV_PIXEL_SAMPLES;
                ++m_pixelIndex;
                if (m_pixelIndex >= SSTV_IMAGE_WIDTH)
                {
                    // Section complete
                    advanceState();
                }
                else
                {
                    // Update tone for next pixel
                    m_currentFreq = getPixelFreqForColumn(m_linePair, m_state, m_pixelIndex);
                }
            }
            return;
        }
        advanceState();
    }
}

void SSTVModSource::prefetch(unsigned int /*nbSamples*/)
{
    // Nothing to prefetch for a purely generated signal
}

// ---------------------------------------------------------------------------
// Public control
// ---------------------------------------------------------------------------

void SSTVModSource::loadImage(const QImage& image)
{
    QMutexLocker lock(&m_mutex);
    // Scale to PD120 dimensions
    QImage scaled = image.scaled(SSTV_IMAGE_WIDTH, SSTV_IMAGE_HEIGHT, Qt::IgnoreAspectRatio, Qt::SmoothTransformation)
                        .convertToFormat(QImage::Format_RGB888);

    m_imageData.resize(SSTV_IMAGE_WIDTH * SSTV_IMAGE_HEIGHT * 3);
    const uchar* src = scaled.constBits();
    memcpy(m_imageData.data(), src, m_imageData.size());
    m_imageValid = true;
    qDebug() << "SSTVModSource::loadImage: loaded" << SSTV_IMAGE_WIDTH << "×" << SSTV_IMAGE_HEIGHT;
}

void SSTVModSource::startTransmit()
{
    QMutexLocker lock(&m_mutex);
    if (!m_imageValid) {
        qWarning("SSTVModSource::startTransmit: no image loaded");
        return;
    }
    qDebug("SSTVModSource::startTransmit: starting PD120 transmission");
    m_linePair = 0;
    m_pixelIndex = 0;
    m_pixelFrac = 0.0f;
    m_fmPhasor = 0.0f;
    m_tonePhasor = 0.0f;
    // Build VIS byte: 7 data bits + even-parity bit
    uint8_t vis7 = SSTV_VIS_CODE & 0x7F;
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
}

void SSTVModSource::applyChannelSettings(int channelSampleRate, int channelFrequencyOffset, bool force)
{
    if ((channelFrequencyOffset != m_channelFrequencyOffset) || force) {
        m_carrierNco.setFreq(channelFrequencyOffset, channelSampleRate);
    }
    m_channelSampleRate = channelSampleRate;
    m_channelFrequencyOffset = channelFrequencyOffset;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

/** Map pixel luminance/chroma value [0..255] to SSTV tone frequency [Hz]. */
float SSTVModSource::pixelToFreq(int value) const
{
    return SSTV_BLACK_FREQ + (float(value) / 255.0f) * (SSTV_WHITE_FREQ - SSTV_BLACK_FREQ);
}

/** Return the tone frequency for pixel column `col` of the current section.
 *
 *  The image is encoded as PD120 YCbCr:
 *   Y_odd  – luminance of line 2*linePair
 *   Cr     – red chroma for both lines (averaged)
 *   Cb     – blue chroma for both lines (averaged)
 *   Y_even – luminance of line 2*linePair+1
 *
 *  RGB → YCbCr conversion (JPEG / ITU-R BT.601 full-range):
 *   Y  = 0.299R + 0.587G + 0.114B
 *   Cb = 128 − 0.1687R − 0.3313G + 0.5B
 *   Cr = 128 + 0.5R   − 0.4187G − 0.0813B
 */
float SSTVModSource::getPixelFreqForColumn(int linePair, State section, int col) const
{
    if (!m_imageValid || m_imageData.isEmpty()) {
        return SSTV_BLACK_FREQ;
    }

    const int lineOdd  = 2 * linePair;
    const int lineEven = 2 * linePair + 1;

    auto getPixel = [&](int line, int x) -> const uchar*
    {
        int lineClamp = qBound(0, line, SSTV_IMAGE_HEIGHT - 1);
        int xClamp    = qBound(0, x,    SSTV_IMAGE_WIDTH  - 1);
        return reinterpret_cast<const uchar*>(
            m_imageData.constData() + (lineClamp * SSTV_IMAGE_WIDTH + xClamp) * 3
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
        default:
            return SSTV_BLACK_FREQ;
    }

    return pixelToFreq(value);
}

/** Generate one complex baseband sample at the current SSTV tone frequency.
 *  The phasor is continuous across state transitions so there are no clicks. */
Complex SSTVModSource::generateSample()
{
    Complex ci;

    if (m_settings.m_modulation == SSTVModSettings::ModulationFM)
    {
        // FM: the SSTV tone frequency drives the FM carrier directly.
        // The audio signal is a tone at m_currentFreq; its normalised amplitude
        // is m_currentFreq / fmDeviation (so that full deviation = 1).
        // Phase advance per sample = 2π × m_currentFreq / sampleRate
        // (This is equivalent to FM-modulating a cosine at the SSTV frequency
        //  because for a constant-frequency audio signal the instantaneous
        //  carrier frequency equals the audio tone frequency × deviation scale.)
        const float phaseInc = 2.0f * static_cast<float>(M_PI) * m_currentFreq / static_cast<float>(SSTV_SAMPLE_RATE);
        m_fmPhasor += phaseInc;
        if (m_fmPhasor > static_cast<float>(M_PI)) {
            m_fmPhasor -= 2.0f * static_cast<float>(M_PI);
        }
        const float scale = 0.891235351562f * SDR_TX_SCALEF; // -1 dB
        ci.real(std::cos(m_fmPhasor) * scale);
        ci.imag(std::sin(m_fmPhasor) * scale);
    }
    else
    {
        // SSB (USB or LSB): treat the SSTV signal as the analytic representation
        // of a single audio tone. For USB the sideband sits at carrier + f_tone;
        // for LSB at carrier - f_tone.
        const float scale = 0.891235351562f * SDR_TX_SCALEF;
        const float phaseInc = 2.0f * static_cast<float>(M_PI) * m_currentFreq / static_cast<float>(SSTV_SAMPLE_RATE);
        m_tonePhasor += phaseInc;
        if (m_tonePhasor > static_cast<float>(M_PI)) {
            m_tonePhasor -= 2.0f * static_cast<float>(M_PI);
        }
        if (m_settings.m_modulation == SSTVModSettings::ModulationUSB)
        {
            ci.real( std::cos(m_tonePhasor) * scale);
            ci.imag( std::sin(m_tonePhasor) * scale);
        }
        else // LSB
        {
            ci.real( std::cos(m_tonePhasor) * scale);
            ci.imag(-std::sin(m_tonePhasor) * scale);
        }
    }

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
            m_pixelIndex = 0;
            m_pixelFrac  = 0.0f;
            m_stateSamples = 0; // pixel sections are driven by m_pixelFrac, not m_stateSamples
            m_currentFreq = getPixelFreqForColumn(m_linePair, s, 0);
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
            // First scan-line pair
            m_linePair = 0;
            enterState(State::LINE_SYNC, SSTV_SYNC_SAMPLES);
            break;
        case State::LINE_SYNC:
            enterState(State::LINE_PORCH, SSTV_PORCH_SAMPLES);
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
            if (m_linePair < SSTV_LINE_PAIRS)
            {
                enterState(State::LINE_SYNC, SSTV_SYNC_SAMPLES);
            }
            else
            {
                qDebug("SSTVModSource: transmission complete");
                m_state = State::DONE;
                m_stateSamples = 0;
                emit transmitComplete();
                m_state = State::IDLE;
            }
            break;
        default:
            m_state = State::IDLE;
            break;
    }
}
