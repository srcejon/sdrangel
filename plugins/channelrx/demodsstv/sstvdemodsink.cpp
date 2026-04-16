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
    m_pixelSampleCount(0),
    m_lineIndex(0),
    m_sdftIdx(0),
    m_porchFreqAccum(0.0f),
    m_aboveThresholdCount(0)
{
    m_magsq = 0.0;

    // Precompute SDFT twiddle factors e^{+j·2π·k/N} for k = SDFT_K_STORE_MIN..SDFT_K_STORE_MAX.
    for (int i = 0; i < SDFT_NUM_BINS; i++) {
        // Compute in double to preserve M_PI accuracy before narrowing to float.
        const float angle = static_cast<float>(2.0 * M_PI * double(SDFT_K_STORE_MIN + i) / double(N_SDFT));
        m_sdftTwiddle[i] = Complex(std::cos(angle), std::sin(angle));
    }

    // Clear the SDFT circular buffer and running bin accumulators.
    memset(m_sdftBuf, 0, sizeof(m_sdftBuf));
    for (int i = 0; i < SDFT_NUM_BINS; i++) {
        m_sdftBins[i] = Complex(0.0f, 0.0f);
    }

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
    m_pixelSampleCount = 0;
    m_lineIndex = 0;
    m_porchFreqAccum = 0.0f;
    m_aboveThresholdCount = 0;

    // Reset the sliding-DFT spectral moment state.
    memset(m_sdftBuf, 0, sizeof(m_sdftBuf));
    m_sdftIdx = 0;
    for (int i = 0; i < SDFT_NUM_BINS; i++) {
        m_sdftBins[i] = Complex(0.0f, 0.0f);
    }
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
    // Stage 2 – Sliding-DFT spectral moment (MATLAB 'instfreq' tfmoment).
    //
    // The recurrence Z[k] ← twiddle[k]·(Z[k] + x_new − x_old) maintains
    // a phase-rotated DFT bin.  The power-weighted centroid over bins k=3..7
    // gives the instantaneous tone frequency used for both sync detection and
    // pixel decoding.  See SDFT_MEAS_WHITE_FREQ in the header for the
    // bias-correction rationale.
    // -----------------------------------------------------------------------

    // Update circular buffer and SDFT bins.
    const float xNew = fmDemod;
    const float xOld = m_sdftBuf[m_sdftIdx];
    m_sdftBuf[m_sdftIdx] = xNew;
    m_sdftIdx = (m_sdftIdx + 1) % N_SDFT;

    const float delta = xNew - xOld;
    for (int i = 0; i < SDFT_NUM_BINS; i++) {
        m_sdftBins[i] = m_sdftTwiddle[i] * (m_sdftBins[i] + delta);
    }

    // Compute power-weighted spectral centroid over bins k = SDFT_K_SUM_MIN..SDFT_K_SUM_MAX.
    float wMoment = 0.0f; // Σ k · |Z[k]|²
    float wPower  = 0.0f; // Σ     |Z[k]|²
    for (int k = SDFT_K_SUM_MIN; k <= SDFT_K_SUM_MAX; k++)
    {
        const int i = k - SDFT_K_STORE_MIN;
        const float p = std::norm(m_sdftBins[i]); // |Z[k]|²
        wMoment += float(k) * p;
        wPower  += p;
    }

    // Frequency estimate in Hz; fall back to sync frequency when no signal.
    const float freq = (wPower > 1.0e-10f)
        ? (wMoment / wPower) * (float(SSTVDEMOD_CHANNEL_SAMPLE_RATE) / float(N_SDFT))
        : SSTVDEMOD_SYNC_FREQ;

    // -----------------------------------------------------------------------
    // SSTV PD120 state machine; 'freq' is the reconstructed tone frequency (Hz)
    // -----------------------------------------------------------------------
    switch (m_state)
    {
    case WAITING_FOR_SYNC:
        // Look for frequency dropping to sync level.
        // While above the threshold, count consecutive samples to detect the
        // 1900 Hz leader tone that precedes each transmission's VIS code.
        if (freq < SSTVDEMOD_SYNC_THRESHOLD)
        {
            // If the signal has been above threshold long enough to be a leader
            // tone, this is the start of a new frame — reset the line counter.
            if (m_aboveThresholdCount >= SSTVDEMOD_LEADER_SAMPLES) {
                m_lineIndex = 0;
            }
            m_aboveThresholdCount = 0;
            transitionTo(IN_SYNC);
        }
        else
        {
            m_aboveThresholdCount++;
        }
        break;

    case IN_SYNC:
        // Count samples while at sync level.
        m_stateSampleCount++;

        if (freq >= SSTVDEMOD_SYNC_THRESHOLD)
        {
            // Sync pulse ended — accept only if within the valid duration window.
            // VIS start/stop bits and "0" data bits are 30 ms (1440 samples),
            // well above SYNC_SAMPLES_MAX (1200 samples = 25 ms), so they are
            // rejected here, preventing false image-decode triggering.
            if (m_stateSampleCount >= SSTVDEMOD_SYNC_SAMPLES_MIN &&
                m_stateSampleCount <= SSTVDEMOD_SYNC_SAMPLES_MAX)
            {
                transitionTo(IN_PORCH);
            }
            else
            {
                transitionTo(WAITING_FOR_SYNC);
            }
        }
        else if (m_stateSampleCount > SSTVDEMOD_SYNC_SAMPLES_MAX)
        {
            // Still at sync level but already too long (e.g. a VIS 0-bit that
            // never rises above the threshold within the valid window).
            transitionTo(WAITING_FOR_SYNC);
        }
        break;

    case IN_PORCH:
        // Wait for the porch period and validate its frequency.
        // A genuine scan-line porch is at the black level (~1500 Hz); the data
        // that follows a VIS code bit is at 1100–1300 Hz.  Discarding low-
        // frequency "porches" eliminates any false syncs that slipped past the
        // duration check above.
        m_stateSampleCount++;
        m_porchFreqAccum += freq;
        if (m_stateSampleCount >= SSTVDEMOD_PORCH_SAMPLES)
        {
            const float avgPorchFreq = m_porchFreqAccum / float(m_stateSampleCount);
            if (avgPorchFreq >= SSTVDEMOD_PORCH_FREQ_MIN) {
                transitionTo(DECODING_Y_ODD);
            } else {
                // Porch frequency too low — the preceding sync was not a real
                // scan-line sync (most likely a VIS code element).
                transitionTo(WAITING_FOR_SYNC);
            }
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
    m_pixelSampleCount++;
    m_pixelSamplePos += 1.0f;

    // Check if we have accumulated enough samples for one pixel
    if (m_pixelSamplePos >= SSTVDEMOD_SAMPLES_PER_PIXEL)
    {
        // Divide by the actual number of samples accumulated (not the fractional
        // position, which includes a carry-over from the previous pixel boundary
        // and would systematically underestimate the frequency by 4–8%).
        float avgFreq = m_pixelAccum / float(m_pixelSampleCount);
        buf[m_pixelIndex] = avgFreq;
        m_pixelIndex++;
        m_pixelAccum = 0.0f;
        m_pixelSampleCount = 0;
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

    // Reset the porch frequency accumulator on every state change so that each
    // new IN_PORCH entry starts with a clean average.
    m_porchFreqAccum = 0.0f;

    // When returning to WAITING_FOR_SYNC (rejected sync or end of frame) the
    // above-threshold counter must restart so that only a genuinely long
    // unbroken leader tone triggers a frame reset.
    if (newState == WAITING_FOR_SYNC) {
        m_aboveThresholdCount = 0;
    }

    // Reset pixel accumulator when starting a new decoding section.
    // Also reset the SDFT window so that the N=128-sample history from the
    // previous section does not contaminate the first ~11 pixels of the next
    // one (e.g. Y_odd frequencies bleeding into the Cr centroid estimate).
    if (newState == DECODING_Y_ODD || newState == DECODING_CR ||
        newState == DECODING_CB   || newState == DECODING_Y_EVEN)
    {
        m_pixelIndex = 0;
        m_pixelAccum = 0.0f;
        m_pixelSamplePos = 0.0f;
        m_pixelSampleCount = 0;

        memset(m_sdftBuf, 0, sizeof(m_sdftBuf));
        m_sdftIdx = 0;
        for (int i = 0; i < SDFT_NUM_BINS; i++) {
            m_sdftBins[i] = Complex(0.0f, 0.0f);
        }
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
