///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019-2021 Edouard Griffiths, F4EXB <f4exb06@gmail.com>          //
// Copyright (C) 2021 Jon Beniston, M7RCE <jon@beniston.com>                    //
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
#include <QImage>

#include "sstvdemod.h"
#include "sstvdemodsink.h"

SSTVDemodSink::SSTVDemodSink() :
    m_channelSampleRate(SSTVDEMOD_CHANNEL_SAMPLE_RATE),
    m_channelFrequencyOffset(0),
    m_magsqSum(0.0f),
    m_magsqPeak(0.0f),
    m_magsqCount(0),
    m_messageQueueToChannel(nullptr),
    m_state(WAITING_FOR_SYNC),
    m_stateSampleCount(0),
    m_pixelIndex(0),
    m_pixelAccum(0.0f),
    m_pixelSamplePos(0.0f),
    m_lineIndex(0)
{
    m_magsq = 0.0;

    // Stage-2: shift audio tone to near DC before discriminating
    m_audioNCO.setFreq(-SSTVDEMOD_AUDIO_CENTER_FREQ, SSTVDEMOD_CHANNEL_SAMPLE_RATE);
    m_audioPhaDiscri.setFMScaling(SSTVDEMOD_CHANNEL_SAMPLE_RATE / (2.0f * SSTVDEMOD_AUDIO_MAX_DEV));

    applySettings(QStringList(), m_settings, true);
    applyChannelSettings(m_channelSampleRate, m_channelFrequencyOffset, true);

    resetDecoder();
}

SSTVDemodSink::~SSTVDemodSink()
{}

void SSTVDemodSink::resetDecoder()
{
    m_state = WAITING_FOR_SYNC;
    m_stateSampleCount = 0;
    m_pixelIndex = 0;
    m_pixelAccum = 0.0f;
    m_pixelSamplePos = 0.0f;
    m_lineIndex = 0;

    // Reset audio tone discriminator state so stale samples don't corrupt new reception
    m_audioPhaDiscri.reset();
}

void SSTVDemodSink::feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end)
{
    Complex ci;

    for (SampleVector::const_iterator it = begin; it != end; ++it)
    {
        Complex c(it->real(), it->imag());
        c *= m_nco.nextIQ();

        if (m_interpolatorDistance < 1.0f) // interpolate
        {
            while (!m_interpolator.interpolate(&m_interpolatorDistanceRemain, c, &ci))
            {
                processOneSample(ci);
                m_interpolatorDistanceRemain += m_interpolatorDistance;
            }
        }
        else // decimate
        {
            if (m_interpolator.decimate(&m_interpolatorDistanceRemain, c, &ci))
            {
                processOneSample(ci);
                m_interpolatorDistanceRemain += m_interpolatorDistance;
            }
        }
    }
}

void SSTVDemodSink::processOneSample(Complex &ci)
{
    // -----------------------------------------------------------------------
    // Stage 1: FM discriminate the RF baseband IQ to recover the audio.
    // fmDemod is the modulating audio waveform — a sinusoid at the SSTV tone
    // frequency (1200–2300 Hz); it is NOT the tone frequency itself.
    // -----------------------------------------------------------------------
    double magsqRaw;
    Real deviation;
    Real fmDemod = m_phaseDiscri.phaseDiscriminatorDelta(ci, magsqRaw, deviation);

    // Update signal power levels
    Real magsq = magsqRaw / (SDR_RX_SCALED * SDR_RX_SCALED);
    m_movingAverage(magsq);
    m_magsq = m_movingAverage.asDouble();
    m_magsqSum += magsq;
    if (magsq > m_magsqPeak) {
        m_magsqPeak = magsq;
    }
    m_magsqCount++;

    if (!m_settings.m_decodeEnabled) {
        return;
    }

    // -----------------------------------------------------------------------
    // Stage 2: measure the instantaneous SSTV tone frequency.
    //
    // fmDemod is a real sinusoid at the SSTV tone frequency (1200–2300 Hz).
    // Multiplying by a complex NCO at -1750 Hz (the centre of that range)
    // produces a complex signal rotating at (tone_freq - 1750) Hz.  A second
    // PhaseDiscriminators instance measures that rotation rate; adding 1750 Hz
    // back gives the tone frequency.
    //
    // Note: because fmDemod is real, the mixing also produces an image at
    // -(tone_freq + 1750) Hz.  At pixel-accumulation timescales (~11 samples)
    // this high-frequency image contribution averages toward zero.
    // -----------------------------------------------------------------------
    Complex audioNco = m_audioNCO.nextIQ();
    Complex audioComplex(fmDemod * audioNco.real(), fmDemod * audioNco.imag());

    double audioMagsq;
    Real audioDev;
    Real audioDemod = m_audioPhaDiscri.phaseDiscriminatorDelta(audioComplex, audioMagsq, audioDev);

    // Scale from normalised discriminator output back to Hz
    float freq = audioDemod * SSTVDEMOD_AUDIO_MAX_DEV + SSTVDEMOD_AUDIO_CENTER_FREQ;

    // -----------------------------------------------------------------------
    // SSTV PD120 state machine; 'freq' is the reconstructed tone frequency (Hz)
    // -----------------------------------------------------------------------
    switch (m_state)
    {
    case WAITING_FOR_SYNC:
        // Look for frequency dropping to sync level
        if (freq < SSTVDEMOD_SYNC_THRESHOLD) {
            transitionTo(IN_SYNC);
        }
        break;

    case IN_SYNC:
        // Count samples while at sync level
        m_stateSampleCount++;
        if (freq >= SSTVDEMOD_SYNC_THRESHOLD)
        {
            // Sync pulse ended — check if it was long enough to be valid
            if (m_stateSampleCount >= SSTVDEMOD_SYNC_SAMPLES_MIN) {
                transitionTo(IN_PORCH);
            } else {
                // Too short — not a real sync pulse
                transitionTo(WAITING_FOR_SYNC);
            }
        }
        break;

    case IN_PORCH:
        // Wait for porch period to pass
        m_stateSampleCount++;
        if (m_stateSampleCount >= SSTVDEMOD_PORCH_SAMPLES) {
            transitionTo(DECODING_Y_ODD);
        }
        break;

    case DECODING_Y_ODD:
        decodePixelSample(freq, m_yOdd, SSTVDEMOD_IMAGE_WIDTH, DECODING_CR);
        break;

    case DECODING_CR:
        decodePixelSample(freq, m_cr, SSTVDEMOD_IMAGE_WIDTH / 2, DECODING_CB);
        break;

    case DECODING_CB:
        decodePixelSample(freq, m_cb, SSTVDEMOD_IMAGE_WIDTH / 2, DECODING_Y_EVEN);
        break;

    case DECODING_Y_EVEN:
        decodePixelSample(freq, m_yEven, SSTVDEMOD_IMAGE_WIDTH, WAITING_FOR_SYNC);
        break;
    }
}

void SSTVDemodSink::decodePixelSample(float freq, float *buf, int width, SSTVState nextState)
{
    // Accumulate frequency for the current pixel
    m_pixelAccum += freq;
    m_pixelSamplePos += 1.0f;

    // Check if we have accumulated enough samples for one pixel
    if (m_pixelSamplePos >= SSTVDEMOD_SAMPLES_PER_PIXEL)
    {
        // Compute average frequency for this pixel and store
        float avgFreq = m_pixelAccum / m_pixelSamplePos;
        buf[m_pixelIndex] = avgFreq;
        m_pixelIndex++;
        m_pixelAccum = 0.0f;
        // Keep fractional remainder to maintain timing accuracy
        m_pixelSamplePos -= SSTVDEMOD_SAMPLES_PER_PIXEL;

        // Check if we've decoded all pixels in this section
        if (m_pixelIndex >= width)
        {
            if (nextState == WAITING_FOR_SYNC) {
                // All four sections decoded — build and send the image block
                commitBlock();
            }
            transitionTo(nextState);
        }
    }
}

void SSTVDemodSink::commitBlock()
{
    if (m_lineIndex >= SSTVDEMOD_IMAGE_HEIGHT / 2) {
        m_lineIndex = 0;
    }

    // Build two RGB scan lines from Y_odd, Cr, Cb, Y_even (YCbCr 4:2:0 like PD120)
    // Each Cr/Cb value applies to the same horizontal position in both lines
    QImage lineImage(SSTVDEMOD_IMAGE_WIDTH, 2, QImage::Format_RGB32);

    for (int x = 0; x < SSTVDEMOD_IMAGE_WIDTH; x++)
    {
        // Cr and Cb are at half horizontal resolution
        int cx = x / 2;
        // Convert from frequency (Hz) to pixel value [0..255] then offset to [-128..127]
        float cr = static_cast<float>(freqToPixel(m_cr[cx])) - 128.0f;
        float cb = static_cast<float>(freqToPixel(m_cb[cx])) - 128.0f;

        // Decode odd scan line (top row of the block)
        {
            float y = static_cast<float>(freqToPixel(m_yOdd[x]));
            int r = static_cast<int>(y + 1.402f * cr);
            int g = static_cast<int>(y - 0.34414f * cb - 0.71414f * cr);
            int b = static_cast<int>(y + 1.772f * cb);
            r = qBound(0, r, 255);
            g = qBound(0, g, 255);
            b = qBound(0, b, 255);
            lineImage.setPixel(x, 0, qRgb(r, g, b));
        }

        // Decode even scan line (bottom row of the block)
        {
            float y = static_cast<float>(freqToPixel(m_yEven[x]));
            int r = static_cast<int>(y + 1.402f * cr);
            int g = static_cast<int>(y - 0.34414f * cb - 0.71414f * cr);
            int b = static_cast<int>(y + 1.772f * cb);
            r = qBound(0, r, 255);
            g = qBound(0, g, 255);
            b = qBound(0, b, 255);
            lineImage.setPixel(x, 1, qRgb(r, g, b));
        }
    }

    // Send the decoded pair of lines to the main demod via message queue
    if (m_messageQueueToChannel) {
        m_messageQueueToChannel->push(SSTVDemod::MsgImage::create(lineImage, m_lineIndex));
    }

    m_lineIndex++;
    if (m_lineIndex >= SSTVDEMOD_IMAGE_HEIGHT / 2) {
        m_lineIndex = 0;
    }
}

void SSTVDemodSink::transitionTo(SSTVState newState)
{
    m_state = newState;
    m_stateSampleCount = 0;

    // Reset pixel accumulator when starting a new decoding section
    if (newState == DECODING_Y_ODD || newState == DECODING_CR ||
        newState == DECODING_CB   || newState == DECODING_Y_EVEN)
    {
        m_pixelIndex = 0;
        m_pixelAccum = 0.0f;
        m_pixelSamplePos = 0.0f;
    }
}

void SSTVDemodSink::applyChannelSettings(int channelSampleRate, int channelFrequencyOffset, bool force)
{
    qDebug() << "SSTVDemodSink::applyChannelSettings:"
             << " channelSampleRate: " << channelSampleRate
             << " channelFrequencyOffset: " << channelFrequencyOffset;

    if ((m_channelFrequencyOffset != channelFrequencyOffset) ||
        (m_channelSampleRate != channelSampleRate) || force)
    {
        m_nco.setFreq(-channelFrequencyOffset, channelSampleRate);
    }

    if ((m_channelSampleRate != channelSampleRate) || force)
    {
        m_interpolator.create(16, channelSampleRate, m_settings.m_rfBandwidth, 2.2);
        m_interpolatorDistance = (Real) channelSampleRate / (Real) SSTVDEMOD_CHANNEL_SAMPLE_RATE;
        m_interpolatorDistanceRemain = m_interpolatorDistance;
    }

    m_channelSampleRate = channelSampleRate;
    m_channelFrequencyOffset = channelFrequencyOffset;
}

void SSTVDemodSink::applySettings(const QStringList& settingsKeys, const SSTVDemodSettings& settings, bool force)
{
    qDebug() << "SSTVDemodSink::applySettings:" << settings.getDebugString(settingsKeys, force);

    if ((settingsKeys.contains("rfBandwidth") && (settings.m_rfBandwidth != m_settings.m_rfBandwidth)) || force)
    {
        m_interpolator.create(16, m_channelSampleRate, settings.m_rfBandwidth, 2.2);
        m_interpolatorDistance = (Real) m_channelSampleRate / (Real) SSTVDEMOD_CHANNEL_SAMPLE_RATE;
        m_interpolatorDistanceRemain = m_interpolatorDistance;
    }

    if ((settingsKeys.contains("fmDeviation") && (settings.m_fmDeviation != m_settings.m_fmDeviation)) || force)
    {
        // setFMScaling maps: fmDemod = instantaneous_freq / fmDeviation
        // With scaling = sampleRate / (2 * fmDeviation):
        //   fmDemod = (2 * freq / sampleRate) * scaling = freq / fmDeviation
        m_phaseDiscri.setFMScaling(SSTVDEMOD_CHANNEL_SAMPLE_RATE / (2.0f * settings.m_fmDeviation));
    }

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }
}
