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
    m_lineIndex(0),
    m_hilbertIdx(0),
    m_freqLpFilter(SSTVDEMOD_CHANNEL_SAMPLE_RATE / (2.0f * float(M_PI) * SSTVDEMOD_LPF_CUTOFF_HZ))
{
    m_magsq = 0.0;

    // Pre-clear the Hilbert FIR delay line.
    memset(m_hilbertBuf, 0, sizeof(m_hilbertBuf));

    // Audio phase discriminator: output = instantaneous_freq_Hz directly.
    // phaseDiscriminatorDelta returns fmDev * fmScaling where
    // fmDev = Δphase/π = 2*f/Fs, so fmScaling = Fs/2 makes output = f in Hz.
    m_audioPhaDiscri.setFMScaling(SSTVDEMOD_CHANNEL_SAMPLE_RATE / 2.0f);

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

    // Reset the Stage-2 audio FM demodulator.
    m_audioPhaDiscri.reset();
    memset(m_hilbertBuf, 0, sizeof(m_hilbertBuf));
    m_hilbertIdx = 0;

    // Reset the low-pass filter on the frequency output.
    m_freqLpFilter.configure(SSTVDEMOD_CHANNEL_SAMPLE_RATE / (2.0f * float(M_PI) * SSTVDEMOD_LPF_CUTOFF_HZ));
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
    // Stage 1 – RF FM demodulation.
    // phaseDiscriminatorDelta with fmScaling = Fs/(2*fmDeviation) returns
    //   fmDemod = instantaneous_rf_freq_deviation / fmDeviation
    // For a signal FM-modulated with an audio tone A·cos(2π·f_tone·t):
    //   fmDemod ≈ A·cos(2π·f_tone·t)          (real audio waveform, ≤ 1.0)
    // The SSTV pixel data is encoded in f_tone (1200–2300 Hz), not in the
    // amplitude.  Stage 2 below extracts that frequency.
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
    // Stage 2 – Audio FM demodulation via Hilbert-transform analytic signal.
    //
    // fmDemod = A·cos(2π·f_tone·n/Fs) is a real narrowband signal whose
    // instantaneous frequency f_tone (1200–2300 Hz) carries the SSTV data.
    // To measure f_tone we form the complex analytic signal using a 63-tap
    // Hamming-windowed Type-III FIR Hilbert approximation (delay = 31 samples):
    //
    //   Q[n] = Σ_{k=1,3,…,31}  h_w[k] · (x[n−31+k] − x[n−31−k])
    //   h_w[k] = (2/(k·π)) · (0.54 + 0.46·cos(π·k/31))   (Hamming-windowed)
    //
    //   z[n] = Complex(x[n−31], −Q[n])  →  z ≈ A·e^(+j·2π·f_tone·(n−31)/Fs)
    //
    // Phase-discriminating z gives f_tone directly in Hz (fmScaling = Fs/2).
    // The Hamming window reduces Gibbs-phenomenon amplitude ripple to ±0.4%
    // across 1200–2300 Hz; without windowing the 63-tap truncated ideal FIR
    // has ~10% amplitude error at 1500 Hz (imaginary part 90% of real), causing
    // instantaneous frequency to oscillate ±150 Hz and cross the 1300 Hz sync
    // threshold — preventing the decoder from ever reaching IN_PORCH.
    // -----------------------------------------------------------------------

    // Write new fmDemod sample into the ring buffer.
    m_hilbertBuf[m_hilbertIdx] = fmDemod;

    // Helper: access x[n-k] from the ring buffer.
    const int idx = m_hilbertIdx;
    auto get = [&](int k) -> Real {
        return m_hilbertBuf[(idx - k + 63) % 63];
    };

    // 63-tap Hilbert FIR Q component (odd taps k=1,3,...,31 around centre 31).
    // Each term: k_hilbert[i] * (get(31-k) - get(31+k)), where k = 2*i+1.
    Real hilbert = k_hilbert[ 0] * (get(30) - get(32))
                 + k_hilbert[ 1] * (get(28) - get(34))
                 + k_hilbert[ 2] * (get(26) - get(36))
                 + k_hilbert[ 3] * (get(24) - get(38))
                 + k_hilbert[ 4] * (get(22) - get(40))
                 + k_hilbert[ 5] * (get(20) - get(42))
                 + k_hilbert[ 6] * (get(18) - get(44))
                 + k_hilbert[ 7] * (get(16) - get(46))
                 + k_hilbert[ 8] * (get(14) - get(48))
                 + k_hilbert[ 9] * (get(12) - get(50))
                 + k_hilbert[10] * (get(10) - get(52))
                 + k_hilbert[11] * (get( 8) - get(54))
                 + k_hilbert[12] * (get( 6) - get(56))
                 + k_hilbert[13] * (get( 4) - get(58))
                 + k_hilbert[14] * (get( 2) - get(60))
                 + k_hilbert[15] * (get( 0) - get(62));

    // Analytic signal: real part delayed by 31 samples, imaginary = Hilbert FIR.
    // The FIR computes Q = −sin(ω·(n−31)) so we negate it to form
    // z = cos(ω·(n−31)) + j·sin(ω·(n−31)) = e^{+jω·(n−31)},
    // giving positive phase rotation and hence positive frequency.
    Complex audioAnalytic(get(31), -hilbert);

    // Phase discriminate; m_audioPhaDiscri has fmScaling = Fs/2,
    // so fmDev·(Fs/2) = (2·f/Fs)·(Fs/2) = f → output is f_tone in Hz.
    //
    // Use phaseDiscriminator (conj(prev)·current, exact std::atan2) rather than
    // phaseDiscriminatorDelta (absolute atan2_approximation2), because the
    // approximation has a ~0.0083 rad step discontinuity at its branch boundary
    // |z| = 1.  That step fires four times per signal cycle and causes ±63 Hz
    // frequency spikes regardless of the Hilbert amplitude accuracy:
    //   spike = 0.0083/π × (Fs/2) = 0.0083/π × 24000 ≈ 63 Hz.
    // With phaseDiscriminator the angle of (conj(prev)·current) is always ≪ 1 rad
    // (≈ ω per sample), so the branch boundary is never reached.
    float freq = m_audioPhaDiscri.phaseDiscriminator(audioAnalytic);

    // Apply low-pass filter to suppress noise from the RF FM demodulator.
    // The filter passes DC and slow pixel transitions while attenuating
    // high-frequency noise: −10 dB at 10 kHz, −14 dB at Nyquist.
    m_freqLpFilter.process(freq, freq);

    // Advance ring-buffer write pointer.
    m_hilbertIdx = (m_hilbertIdx + 1) % 63;

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
