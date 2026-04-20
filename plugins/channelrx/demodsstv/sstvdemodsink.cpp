///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019-2021 Edouard Griffiths, F4EXB <f4exb06@gmail.com>          //
// Copyright (C) 2021-2026 Jon Beniston, M7RCE <jon@beniston.com>                //
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

#include <QDebug>
#include <QImage>

#include "dsp/scopevis.h"
#include "dsp/basebandsamplesink.h"

#include "sstvdemod.h"
#include "sstvdemodsink.h"

SSTVDemodSink::SSTVDemodSink() :
    m_channelSampleRate(SSTVDEMOD_CHANNEL_SAMPLE_RATE),
    m_channelFrequencyOffset(0),
    m_magsqSum(0.0f),
    m_magsqPeak(0.0f),
    m_magsqCount(0),
    m_messageQueueToChannel(nullptr),
    m_scopeSink(nullptr),
    m_spectrumSink(nullptr),
    m_sampleBufferIndex(0),
    m_specBufferIndex(0),
    m_state(WAITING_FOR_SYNC),
    m_stateSampleCount(0),
    m_pixelIndex(0),
    m_pixelAccum(0.0f),
    m_pixelSamplePos(0.0f),
    m_pixelSampleCount(0),
    m_modeWidth(SSTVDEMOD_IMAGE_WIDTH),
    m_modeHeight(SSTVDEMOD_IMAGE_HEIGHT),
    m_modeLinePairs(SSTVDEMOD_IMAGE_HEIGHT / 2),
    m_modeSamplesPerPixel(SSTVDEMOD_SAMPLES_PER_PIXEL),
    m_nominalLinePeriod(NOMINAL_LINE_PERIOD_SAMPLES),
    m_lineIndex(0),
    m_syncSamplesMin(SSTVDEMOD_SYNC_SAMPLES_MIN),
    m_syncSamplesMax(SSTVDEMOD_SYNC_SAMPLES_MAX),
    m_porchSamplesRemaining(SYNC_PORCH_SAMPLES),
    m_pixelSkipAtStart(0.0f),
    m_modeInterSectionSamples(0),
    m_modeChromaSamplesPerPixel(0.0f),
    m_modeChromaWidth(0),
    m_sdftIdx(0),
    m_hilbertIdx(0),
    m_hilbertDcXPrev(0.0f),
    m_hilbertDcY(0.0f),
    m_hilbertZ(0.0f, 0.0f),
    m_useHilbert(false),
    m_syncSdftIdx(0),
    m_pllLocked(false),
    m_pllPhase(0),
    m_pllPeriod(NOMINAL_LINE_PERIOD_SAMPLES),
    m_syncPulseSeen(false),
    m_visState(VIS_IDLE),
    m_visStateSamples(0),
    m_visBitCount(0),
    m_visByte(0),
    m_goertzel1100_s1(0.0f),
    m_goertzel1100_s2(0.0f),
    m_goertzel1300_s1(0.0f),
    m_goertzel1300_s2(0.0f),
    m_goertzelCount(0)
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

    // Precompute sync-energy SDFT twiddle factor e^{+j·2π·SYNC_K/N_SYNC}.
    {
        const float angle = static_cast<float>(2.0 * M_PI * double(SYNC_K) / double(N_SYNC));
        m_syncSdftTwiddle = Complex(std::cos(angle), std::sin(angle));
    }
    memset(m_syncSdftBuf, 0, sizeof(m_syncSdftBuf));
    m_syncSdftBin = Complex(0.0f, 0.0f);

    // Clear the Hilbert FIR ring buffer and DC-blocker state.
    memset(m_hilbertBuf, 0, sizeof(m_hilbertBuf));

    // Initialise scope and spectrum sample buffers.
    for (int i = 0; i < SSTVDemodSettings::m_scopeStreams; i++) {
        m_sampleBuffer[i].resize(m_sampleBufferSize);
    }
    m_specBuffer.resize(m_specBufferSize);

    applySettings(QStringList(), m_settings, true);
    applyChannelSettings(m_channelSampleRate, m_channelFrequencyOffset, true);

    resetDecoder();
}

SSTVDemodSink::~SSTVDemodSink()
{}

void SSTVDemodSink::sampleToScope(Real fmDemod, Real freq, Real isSyncTone, Real pllLocked, Real state)
{
    if (m_scopeSink)
    {
        m_sampleBuffer[0][m_sampleBufferIndex] = Complex(fmDemod, 0.0f);
        m_sampleBuffer[1][m_sampleBufferIndex] = Complex(freq, 0.0f);
        m_sampleBuffer[2][m_sampleBufferIndex] = Complex(isSyncTone, 0.0f);
        m_sampleBuffer[3][m_sampleBufferIndex] = Complex(pllLocked, 0.0f);
        m_sampleBuffer[4][m_sampleBufferIndex] = Complex(state, 0.0f);
        m_sampleBufferIndex++;

        if (m_sampleBufferIndex == m_sampleBufferSize)
        {
            std::vector<ComplexVector::const_iterator> vbegin;

            for (int i = 0; i < SSTVDemodSettings::m_scopeStreams; i++) {
                vbegin.push_back(m_sampleBuffer[i].begin());
            }

            m_scopeSink->feed(vbegin, m_sampleBufferSize);
            m_sampleBufferIndex = 0;
        }
    }
}

void SSTVDemodSink::sampleToSpectrum(Real fmDemod)
{
    if (m_spectrumSink)
    {
        m_specBuffer[m_specBufferIndex++] = Sample(fmDemod * SDR_RX_SCALEF, 0);

        if (m_specBufferIndex == m_specBufferSize)
        {
            m_spectrumSink->feed(m_specBuffer.begin(), m_specBuffer.end(), false);
            m_specBufferIndex = 0;
        }
    }
}

void SSTVDemodSink::resetDecoder()
{
    m_state = WAITING_FOR_SYNC;
    m_stateSampleCount = 0;
    m_pixelIndex = 0;
    m_pixelAccum = 0.0f;
    m_pixelSamplePos = 0.0f;
    m_pixelSampleCount = 0;
    m_lineIndex = 0;

    memset(m_scPendingGreen, 0, sizeof(m_scPendingGreen));
    memset(m_scPendingBlue,  0, sizeof(m_scPendingBlue));

    // Reset the sliding-DFT spectral moment state and post-SDFT moving average.
    memset(m_sdftBuf, 0, sizeof(m_sdftBuf));
    m_sdftIdx = 0;
    for (int i = 0; i < SDFT_NUM_BINS; i++) {
        m_sdftBins[i] = Complex(0.0f, 0.0f);
    }
    m_freqMovAvg.reset();
    m_syncTotalPowerMa.reset();

    // Reset the sync-energy sliding DFT.
    memset(m_syncSdftBuf, 0, sizeof(m_syncSdftBuf));
    m_syncSdftBin = Complex(0.0f, 0.0f);
    m_syncSdftIdx = 0;

    // Reset the sync timing PLL.
    m_pllLocked = false;
    m_pllPhase = 0;
    m_pllPeriod = m_nominalLinePeriod;
    m_syncPulseSeen = false;

    // Reset the Hilbert IF estimator state.
    memset(m_hilbertBuf, 0, sizeof(m_hilbertBuf));
    m_hilbertIdx = 0;
    m_hilbertDcXPrev = 0.0f;
    m_hilbertDcY = 0.0f;
    m_hilbertZ = Complex(0.0f, 0.0f);
    // m_freqMovAvg is reset above (shared with SDFT path).

    // Reset the VIS header detector.
    m_visState = VIS_IDLE;
    m_visStateSamples = 0;
    m_visBitCount = 0;
    m_visByte = 0;
    m_goertzel1100_s1 = 0.0f;
    m_goertzel1100_s2 = 0.0f;
    m_goertzel1300_s1 = 0.0f;
    m_goertzel1300_s2 = 0.0f;
    m_goertzelCount = 0;
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
    // Stage 1 – Demodulation.
    //
    // For FM: use the phase discriminator to extract the audio tone.
    //   fmDemod = instantaneous_rf_freq_deviation / fmDeviation
    // For USB/LSB (SSB): the complex baseband signal ci already encodes the
    //   audio tone as its instantaneous frequency.  The real part of ci gives
    //   the audio tone directly:
    //     USB: ci = A·exp(+j·2π·f_tone·t)  →  ci.real() = A·cos(2π·f_tone·t)
    //     LSB: ci = A·exp(−j·2π·f_tone·t)  →  ci.real() = A·cos(2π·f_tone·t)
    //   (the real parts are identical; the sideband selection affects only the
    //   imaginary sign, not the tone frequency content).
    //   The result is normalised by SDR_RX_SCALEF to bring it into the same
    //   ~[−1, +1] range as fmDemod, keeping Stage 2 thresholds valid.
    // -----------------------------------------------------------------------
    double magsqRaw;
    Real deviation;
    Real fmDemod = m_phaseDiscri.phaseDiscriminatorDelta(ci, magsqRaw, deviation);

    // audioSample is the Stage 1 output fed to Stage 2, sync detector and output
    const Real audioSample = (m_settings.m_modulation != SSTVDemodSettings::ModulationFM)
        ? (ci.real() / SDR_RX_SCALEF)
        : fmDemod;

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
        sampleToSpectrum(audioSample);
        return;
    }

    // -----------------------------------------------------------------------
    // Stage 2 – Instantaneous tone-frequency estimation.
    //
    // Two implementations are available; the active one is selected by m_useHilbert.
    // Both produce 'freq' in Hz, used by the state machine below.
    // -----------------------------------------------------------------------
    float freq;

    if (!m_useHilbert)
    {
        // ------------------------------------------------------------------
        // SDFT path: Sliding-DFT spectral moment (MATLAB 'instfreq' tfmoment).
        //
        // The recurrence Z[k] ← twiddle[k]·(Z[k] + x_new − x_old) maintains
        // a phase-rotated DFT bin.  The power-weighted centroid over bins k=1..2
        // gives the instantaneous tone frequency used for both sync detection and
        // pixel decoding.  freqToPixel() uses SDFT_CALIB_CENTROIDS to invert the
        // non-linear centroid-vs-frequency relationship; see the header for details.
        // ------------------------------------------------------------------

        // Update circular buffer and SDFT bins.
        const float xNew = audioSample;
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

        // Frequency estimate in Hz; fall back to black level when no signal so that
        // the first samples after a decoder reset produce black rather than an
        // out-of-range artefact (1200 Hz is below the black level and would cause
        // green pixels via the YCbCr conversion when Cr/Cb sections start up).
        const float rawFreq = (wPower > 1.0e-10f)
            ? (wMoment / wPower) * (float(SSTVDEMOD_CHANNEL_SAMPLE_RATE) / float(N_SDFT))
            : SSTVDEMOD_BLACK_FREQ;

        // Apply moving average to cancel the centroid oscillation that occurs at
        // 2×f_tone for a real-valued cosine input (beat between the positive and
        // aliased negative frequency components in the SDFT bins).
        // SDFT_FREQ_MA_LEN=40 spans exactly two 1200 Hz periods, completely
        // eliminating the sync-tone oscillation and leaving a stable ~1316 Hz
        // reading.  instantAverage() gives the correct mean during the initial
        // fill phase after reset (using only the samples seen so far).
        m_freqMovAvg(rawFreq);
        freq = m_freqMovAvg.instantAverage();
    }
    else
    {
        // ------------------------------------------------------------------
        // Hilbert path: analytic signal phase-difference IF estimator.
        //
        // 1. Store fmDemod in the Hilbert ring buffer.
        // 2. Compute the Hilbert transform output (imaginary part of the
        //    analytic signal) via the 63-tap FIR (non-zero at even indices
        //    k=0,2,...,62).
        // 3. The real part is the input delayed by HILBERT_M=31 samples.
        // 4. Instantaneous frequency = (Fs/2pi)*arg(conj(z_prev)*z_curr).
        // 5. Apply the 3000 Hz 1st-order IIR LPF to suppress wideband noise.
        // ------------------------------------------------------------------

        // Write new sample into the ring buffer.
        // Apply DC-blocking HPF first: y[n] = x[n] - x[n-1] + R*y[n-1].
        // This removes any carrier-frequency offset that would make the phasor z off-centre
        // and cause the phase-difference estimator to oscillate at f_tone Hz.
        const float dcBlocked = audioSample - m_hilbertDcXPrev + HILBERT_DC_R * m_hilbertDcY;
        m_hilbertDcXPrev = audioSample;
        m_hilbertDcY = dcBlocked;
        m_hilbertBuf[m_hilbertIdx] = dcBlocked;
        m_hilbertIdx = (m_hilbertIdx + 1) % N_HILBERT;

        // FIR Hilbert transform: convolve with HILBERT_COEFFS (even-index taps only).
        // k=0 maps to the newest sample, k=62 to the oldest in the ring buffer.
        float y_h = 0.0f;
        for (int k = 0; k < N_HILBERT; k += 2)
        {
            const int bufIdx = (m_hilbertIdx - 1 - k + N_HILBERT) % N_HILBERT;
            y_h += HILBERT_COEFFS[k] * m_hilbertBuf[bufIdx];
        }

        // Real part: input delayed by HILBERT_M samples (centre of FIR window).
        const float x_r = m_hilbertBuf[(m_hilbertIdx - 1 - HILBERT_M + N_HILBERT) % N_HILBERT];

        // Build complex analytic sample z_curr = x_r + j*y_h.
        const Complex z_curr(x_r, y_h);

        // Phase-difference instantaneous frequency (Fs/2pi)*arg(conj(z_prev)*z_curr).
        // Fall back to BLACK_FREQ when the signal power is negligible.
        float rawFreq;
        const float power = std::norm(z_curr);
        if (power > 1.0e-10f)
        {
            // conj(z_prev)*z_curr = (rp-j*ip)*(rc+j*ic)
            //   real part = rp*rc + ip*ic
            //   imag part = rp*ic - ip*rc
            const float rp = m_hilbertZ.real();
            const float ip = m_hilbertZ.imag();
            const float rc = z_curr.real();
            const float ic = z_curr.imag();
            const float phiDotReal = rp * rc + ip * ic;
            const float phiDotImag = rp * ic - ip * rc;
            rawFreq = (float(SSTVDEMOD_CHANNEL_SAMPLE_RATE) / (2.0f * float(M_PI)))
                      * std::atan2(phiDotImag, phiDotReal);
        }
        else
        {
            rawFreq = SSTVDEMOD_BLACK_FREQ;
        }

        m_hilbertZ = z_curr;

        // 40-sample moving average (same filter used by the SDFT path).
        // Spans exactly two periods of 1200 Hz (Fs/40 = 1200 Hz), giving exact nulls at
        // all multiples of 1200 Hz.  This completely cancels the two systematic artifacts
        // of the phase-difference estimator on a real cosine input:
        //   • 2×f_tone (2400 Hz) — from the slightly elliptical Hilbert phasor (|H|=0.985)
        //   • f_tone  (1200 Hz) — from any residual DC offset in the DC-blocked signal
        // See SDFT_FREQ_MA_LEN comment in the header for the rationale.
        m_freqMovAvg(rawFreq);
        freq = m_freqMovAvg.instantAverage();
    }

    // -----------------------------------------------------------------------
    // Sync-tone energy detector (runs every sample, state-independent).
    //
    // A separate N_SYNC=160 sliding-DFT bin at k=4 (1200 Hz) measures the
    // fraction of signal energy at the sync frequency.  N_SYNC=160 = Fs/300
    // places exact nulls at 1500, 1800, 2100, 2400 Hz so pixel-data tones
    // produce zero energy in this bin.
    //
    // syncRatio = |Z_sync|² / ((N_SYNC²/4) × MA(fmDemod²))
    //   ≈ 1.0 when a sustained 1200 Hz sync tone is present
    //   ≈ 0.0 for any pixel-data tone (1500–2300 Hz)
    //   ≈ 0.10 at the sync/pixel boundary (1420 Hz)
    // Threshold SYNC_DETECT_THRESHOLD = 0.4 sits comfortably above 0.10
    // and below 1.0, independent of signal amplitude.
    // -----------------------------------------------------------------------
    const float xNewSync = audioSample;
    const float xOldSync = m_syncSdftBuf[m_syncSdftIdx];
    m_syncSdftBuf[m_syncSdftIdx] = xNewSync;
    m_syncSdftIdx = (m_syncSdftIdx + 1) % N_SYNC;
    m_syncSdftBin = m_syncSdftTwiddle * (m_syncSdftBin + xNewSync - xOldSync);

    m_syncTotalPowerMa(audioSample * audioSample);
    const float syncTotalPower = m_syncTotalPowerMa.instantAverage();

    // isSyncTone: true when 1200 Hz sync tone is present, false for pixel data.
    const bool isSyncTone = (syncTotalPower > 1.0e-10f) &&
        (std::norm(m_syncSdftBin) >
            SYNC_DETECT_THRESHOLD * (float(N_SYNC * N_SYNC) / 4.0f) * syncTotalPower);

    // -----------------------------------------------------------------------
    // SSTV PD120 state machine.
    // 'freq' (from the pixel SDFT or Hilbert path) is used ONLY for pixel
    // decoding; sync detection uses isSyncTone exclusively.
    //
    // Sync timing PLL: when m_pllLocked is true, m_pllPhase increments every
    // sample.  The IN_SYNC → IN_PORCH transition is driven by the PLL phase
    // reaching m_pllPeriod (the smoothed line-period estimate) rather than by
    // the raw isSyncTone falling edge, eliminating per-line timing jitter.
    // -----------------------------------------------------------------------

    // Advance PLL phase accumulator in all states once locked.
    if (m_pllLocked) {
        m_pllPhase++;
    }

    switch (m_state)
    {
    case WAITING_FOR_SYNC:
        // Lose PLL lock if sync has been absent for more than 1.5 line periods
        // (signal lost or persistent noise).
        if (m_pllLocked && m_pllPhase > static_cast<int>(m_nominalLinePeriod * 1.5f)) {
            m_pllLocked = false;
        }
        // isSyncTone becomes true when the sync-bin energy exceeds the
        // threshold (~72 samples after the 1200 Hz tone starts, at threshold 0.4).
        if (isSyncTone) {
            transitionTo(IN_SYNC);
        }
        break;

    case IN_SYNC:
        // Count samples while the sync tone is present.
        m_stateSampleCount++;

        // On the falling edge of isSyncTone (first occurrence only per visit):
        // validate the sync-pulse duration and, when the PLL is locked, update
        // the period estimate.  The actual IN_PORCH transition is deferred to
        // the PLL-driven check below so that the transition time is controlled
        // by the smoothed estimate rather than the jittery raw detector.
        if (!isSyncTone && !m_syncPulseSeen)
        {
            // At SYNC_DETECT_THRESHOLD=0.4 the exit fires ≈ SYNC_PORCH_DELAY=88
            // samples into the porch (where energy has decayed to threshold).
            // m_stateSampleCount therefore includes: actual sync pulse
            // (≈960 samples) + 88 porch samples ≈ 1048 samples — still within
            // [SYNC_SAMPLES_MIN=720, SYNC_SAMPLES_MAX=1200].
            // VIS bits (30 ms = 1440 samples) produce a count ≈ 1528 > MAX → rejected.
            const bool validDuration = (m_stateSampleCount >= m_syncSamplesMin &&
                                        m_stateSampleCount <= m_syncSamplesMax);
            if (validDuration)
            {
                if (m_pllLocked)
                {
                    // Update the PLL: in the ideal case m_pllPhase == m_pllPeriod
                    // exactly when isSyncTone falls.  Any deviation is phase error.
                    const float error = float(m_pllPhase) - m_pllPeriod;
                    m_pllPeriod = qBound(
                        m_nominalLinePeriod * 0.9f,
                        m_pllPeriod + PLL_ALPHA * error,
                        m_nominalLinePeriod * 1.1f);
                    // Mark that a valid sync was seen; the PLL-driven transition
                    // below fires when m_pllPhase reaches m_pllPeriod.
                    m_syncPulseSeen = true;
                }
                else
                {
                    // First valid sync: lock the PLL and enter porch immediately
                    // using raw timing (no PLL history yet).
                    m_pllLocked = true;
                    m_pllPhase = 0;
                    m_pllPeriod = m_nominalLinePeriod;
                    m_syncPulseSeen = false;
                    transitionTo(IN_PORCH);
                }
            }
            else
            {
                // Invalid duration (e.g. noise glitch or VIS bit) — reject.
                m_pllLocked = false;
                transitionTo(WAITING_FOR_SYNC);
            }
        }
        else if (m_stateSampleCount > m_syncSamplesMax && !m_syncPulseSeen)
        {
            // Sync energy has been high for too long (e.g. a VIS 0-bit or
            // carrier hold that never transitions).  Abort.
            m_pllLocked = false;
            transitionTo(WAITING_FOR_SYNC);
        }

        // PLL-driven transition: once a valid sync pulse has been confirmed,
        // enter IN_PORCH when the smoothed period estimate is reached.  Any
        // overshoot is carried into the next period so small delays don't
        // accumulate.  If the raw detection was early (m_pllPhase < m_pllPeriod
        // when isSyncTone fell), we wait here in IN_SYNC — overlapping with the
        // actual porch — until the PLL fires.  This is safe because the pixel
        // SDFT only starts after IN_PORCH completes.
        if (m_pllLocked && m_syncPulseSeen && m_pllPhase >= static_cast<int>(m_pllPeriod))
        {
            m_pllPhase -= static_cast<int>(m_pllPeriod); // carry overshoot
            m_syncPulseSeen = false;
            transitionTo(IN_PORCH);
        }
        break;

    case IN_PORCH:
        m_stateSampleCount++;
        if (m_stateSampleCount >= m_porchSamplesRemaining)
        {
            SSTVState firstState = getFirstDecodeState();
            transitionTo(firstState);
            // Compensate for the SYNC_PORCH_DELAY samples consumed while waiting for
            // the sync energy to fall below threshold.  When m_pixelSkipAtStart is
            // less than one pixel period (Scottie / PD / Robot36) a plain pre-load of
            // m_pixelSamplePos is sufficient and produces no phantom pixels.
            //
            // When m_pixelSkipAtStart >= m_modeSamplesPerPixel (Martin M1/M2, and
            // Scottie S2), the direct pre-load fires phantom pixels — each from only
            // one real sample at the wrong image position (~3 columns into the image
            // placed at columns 0/1).  Instead, advance m_pixelIndex by the integer
            // count of full pixel-widths already elapsed and set m_pixelSamplePos to
            // the fractional intra-pixel remainder only, so every decoded pixel fires
            // from the correct number of samples at the right position.  The skipped
            // pixel slots are pre-filled from the current frequency estimate: the MA
            // output at this moment approximates the signal over the missed samples,
            // so white content stays near-white and black content stays near-black.
            const int skipPixels = int(m_pixelSkipAtStart / m_modeSamplesPerPixel);
            m_pixelSamplePos = m_pixelSkipAtStart - float(skipPixels) * m_modeSamplesPerPixel;
            m_pixelIndex = skipPixels;
            if (skipPixels > 0)
            {
                float *firstBuf = nullptr;
                switch (firstState)
                {
                    case DECODING_MT_GREEN: firstBuf = m_yOdd; break;
                    case DECODING_SC_RED:   firstBuf = m_cr;   break;
                    default:                                    break;
                }
                if (firstBuf)
                {
                    for (int i = 0; i < skipPixels; i++) {
                        firstBuf[i] = freq;
                    }
                }
            }
        }
        break;

    case DECODING_Y_ODD:
        decodePixelSample(freq, m_yOdd, m_modeWidth, DECODING_CR);
        break;

    case DECODING_CR:
        decodePixelSample(freq, m_cr, m_modeWidth, DECODING_CB);
        break;

    case DECODING_CB:
        decodePixelSample(freq, m_cb, m_modeWidth, DECODING_Y_EVEN);
        break;

    case DECODING_Y_EVEN:
        decodePixelSample(freq, m_yEven, m_modeWidth, WAITING_FOR_SYNC);
        break;

    // ---- Robot36 ----
    case DECODING_R36_Y:
        decodePixelSampleEx(freq, m_yOdd, m_modeWidth, m_modeSamplesPerPixel, DECODING_R36_SEP);
        break;

    case DECODING_R36_SEP:
        m_stateSampleCount++;
        if (m_stateSampleCount >= m_modeInterSectionSamples) {
            transitionTo(DECODING_R36_CHROMA);
        }
        break;

    case DECODING_R36_CHROMA:
    {
        float *chromaBuf = (m_lineIndex % 2 == 0) ? m_cr : m_cb;
        decodePixelSampleEx(freq, chromaBuf, m_modeChromaWidth, m_modeChromaSamplesPerPixel, WAITING_FOR_SYNC);
        if (m_state == WAITING_FOR_SYNC)
        {
            commitBlockRobot36();
        }
        break;
    }

    // ---- Scottie ----
    case DECODING_SC_RED:
        decodePixelSampleEx(freq, m_cr, m_modeWidth, m_modeSamplesPerPixel, DECODING_SC_GREEN);
        break;

    case DECODING_SC_GREEN:
        decodePixelSampleEx(freq, m_yOdd, m_modeWidth, m_modeSamplesPerPixel, DECODING_SC_PORCH);
        break;

    case DECODING_SC_PORCH:
        m_stateSampleCount++;
        if (m_stateSampleCount >= m_modeInterSectionSamples) {
            transitionTo(DECODING_SC_BLUE);
        }
        break;

    case DECODING_SC_BLUE:
        decodePixelSampleEx(freq, m_cb, m_modeWidth, m_modeSamplesPerPixel, WAITING_FOR_SYNC);
        if (m_state == WAITING_FOR_SYNC)
        {
            commitBlockScottie();
        }
        break;

    // ---- Martin ----
    case DECODING_MT_GREEN:
        decodePixelSampleEx(freq, m_yOdd, m_modeWidth, m_modeSamplesPerPixel, DECODING_MT_BLUE);
        break;

    case DECODING_MT_BLUE:
        decodePixelSampleEx(freq, m_cb, m_modeWidth, m_modeSamplesPerPixel, DECODING_MT_RED);
        break;

    case DECODING_MT_RED:
        decodePixelSampleEx(freq, m_cr, m_modeWidth, m_modeSamplesPerPixel, WAITING_FOR_SYNC);
        if (m_state == WAITING_FOR_SYNC)
        {
            commitBlockMartin();
        }
        break;
    }

    // -----------------------------------------------------------------------
    // VIS / VSS header detector (runs every sample, parallel to the main
    // decoder state machine).
    //
    // Detects the standard SSTV preamble: leader–break–leader–start–bits–stop.
    // When a complete valid VIS sequence is found the main decoder is reset
    // (so line acquisition restarts from line 0) and MsgVIS is sent to the GUI.
    // -----------------------------------------------------------------------
    {
        // Compute pixel value (calibrated for the active frequency estimator path)
        // used to detect the 1900 Hz leader tone (pixel ≈ 128, band ±32).
        const int visPixel = m_useHilbert ? freqToPixelDirect(freq) : freqToPixel(freq);
        const bool isLeaderTone = !isSyncTone
                                  && (visPixel >= VIS_LEADER_PIXEL_MIN)
                                  && (visPixel <= VIS_LEADER_PIXEL_MAX);

        switch (m_visState)
        {
        case VIS_IDLE:
            // Count consecutive samples at the 1900 Hz leader frequency.
            if (isLeaderTone) {
                m_visStateSamples++;
                if (m_visStateSamples >= VIS_LEADER_MIN_SAMPLES) {
                    m_visState = VIS_BREAK;
                    m_visStateSamples = 0;
                }
            } else {
                m_visStateSamples = 0;
            }
            break;

        case VIS_BREAK:
            // Wait for the 10 ms 1200 Hz break that follows the first leader.
            //
            // Two-phase detection (same rationale as VIS_START_BIT):
            //
            //   Phase 1 (m_visStateSamples == 0): we are still in the trailing
            //     samples of the first leader (1900 Hz, isSyncTone = false).
            //     Ignore all non-sync samples; only the first isSyncTone rising
            //     edge (the actual break) starts counting.
            //
            //   Phase 2 (m_visStateSamples > 0): sync is present and being
            //     counted.  When isSyncTone falls, validate the duration.
            //
            // Without phase 1 the code would see the leader tail (isSyncTone =
            // false, count = 0 < VIS_BREAK_MIN_SAMPLES) and immediately fall
            // back to VIS_IDLE, preventing VIS_LEADER2 from ever being reached.
            if (isSyncTone || m_visStateSamples > 0)
            {
                if (isSyncTone) {
                    m_visStateSamples++;
                } else {
                    // Break has ended — validate its duration.
                    if (m_visStateSamples >= VIS_BREAK_MIN_SAMPLES &&
                        m_visStateSamples <= VIS_BREAK_MAX_SAMPLES) {
                        m_visState = VIS_LEADER2;
                    } else {
                        m_visState = VIS_IDLE;
                    }
                    m_visStateSamples = 0;
                }
            }
            break;

        case VIS_LEADER2:
            // Count consecutive samples at the 1900 Hz leader frequency.
            //
            // Guard: only reset to VIS_IDLE for a non-leader non-sync sample
            // once counting has started (m_visStateSamples > 0).
            //
            // Rationale: when entering VIS_LEADER2 the frequency estimator
            // needs a brief warm-up period to stabilise from 1200 Hz (break)
            // to 1900 Hz (leader2).  During that warm-up isSyncTone is false
            // and visPixel is below VIS_LEADER_PIXEL_MIN (it is still near the
            // 1200 Hz sync frequency), so isLeaderTone is also false.  Without
            // the m_visStateSamples > 0 guard the "else if (!isSyncTone)" branch
            // would fire on the very first sample and immediately reset to
            // VIS_IDLE, preventing VIS_START_BIT from ever being reached.
            if (isLeaderTone) {
                m_visStateSamples++;
                if (m_visStateSamples >= VIS_LEADER_MIN_SAMPLES) {
                    m_visState = VIS_START_BIT;
                    m_visStateSamples = 0;
                }
            } else if (!isSyncTone && m_visStateSamples > 0) {
                // Lost the leader after counting had begun — restart.
                m_visState = VIS_IDLE;
                m_visStateSamples = 0;
            }
            break;

        case VIS_START_BIT:
            // Detect the 30 ms 1200 Hz VIS start bit, then count exactly one
            // VIS_BIT_SAMPLES window before entering VIS_BITS.
            //
            // IMPORTANT: the sync energy detector cannot reject 1100 Hz and
            // 1300 Hz VIS data bits — they are only 100 Hz from the 1200 Hz
            // sync bin and produce isSyncTone = true (leakage ≈ 83%).
            // The previous approach of waiting for isSyncTone to fall could
            // not exit this state. Instead, use isSyncTone rising as a
            // one-time timing reference, then count a fixed VIS_BIT_SAMPLES
            // duration unconditionally before transitioning to VIS_BITS.
            if (isSyncTone || m_visStateSamples > 0)
            {
                // Count from the first isSyncTone sample through one full
                // bit-period (1440 samples = 30 ms), regardless of isSyncTone state.
                if (++m_visStateSamples >= VIS_BIT_SAMPLES)
                {
                    m_visState = VIS_BITS;
                    m_visBitCount = 0;
                    m_visByte = 0;
                    m_goertzel1100_s1 = 0.0f;
                    m_goertzel1100_s2 = 0.0f;
                    m_goertzel1300_s1 = 0.0f;
                    m_goertzel1300_s2 = 0.0f;
                    m_goertzelCount = 0;
                    m_visStateSamples = 0;
                }
            }
            break;

        case VIS_BITS:
        {
            // Feed the current sample into the Goertzel filters for 1100 Hz and
            // 1300 Hz.  Both are exact DFT bins for N = VIS_BIT_SAMPLES = 1440.
            const float s0_1100 = audioSample
                                  + GOERTZEL_1100_COEFF * m_goertzel1100_s1
                                  - m_goertzel1100_s2;
            m_goertzel1100_s2 = m_goertzel1100_s1;
            m_goertzel1100_s1 = s0_1100;

            const float s0_1300 = audioSample
                                  + GOERTZEL_1300_COEFF * m_goertzel1300_s1
                                  - m_goertzel1300_s2;
            m_goertzel1300_s2 = m_goertzel1300_s1;
            m_goertzel1300_s1 = s0_1300;

            m_goertzelCount++;

            if (m_goertzelCount >= VIS_BIT_SAMPLES)
            {
                // End of bit window: compute terminal Goertzel power.
                const float p1100 = m_goertzel1100_s1 * m_goertzel1100_s1
                                    + m_goertzel1100_s2 * m_goertzel1100_s2
                                    - GOERTZEL_1100_COEFF * m_goertzel1100_s1 * m_goertzel1100_s2;
                const float p1300 = m_goertzel1300_s1 * m_goertzel1300_s1
                                    + m_goertzel1300_s2 * m_goertzel1300_s2
                                    - GOERTZEL_1300_COEFF * m_goertzel1300_s1 * m_goertzel1300_s2;

                // 1100 Hz = binary 1; 1300 Hz = binary 0.  VIS bits are LSB-first.
                const int bit = (p1100 > p1300) ? 1 : 0;
                m_visByte |= (bit << m_visBitCount);
                m_visBitCount++;

                // Reset Goertzel for the next bit.
                m_goertzel1100_s1 = 0.0f;
                m_goertzel1100_s2 = 0.0f;
                m_goertzel1300_s1 = 0.0f;
                m_goertzel1300_s2 = 0.0f;
                m_goertzelCount = 0;

                if (m_visBitCount >= 8)
                {
                    // All 8 bits received (bits 0–6 = VIS code, bit 7 = even parity).
                    const int visCode = m_visByte & 0x7F;
                    const int parityBit = (m_visByte >> 7) & 1;
                    int onesCount = 0;
                    for (int i = 0; i < 7; i++) {
                        onesCount += (visCode >> i) & 1;
                    }
                    const bool parityOK = ((onesCount + parityBit) % 2) == 0;

                    // Notify the GUI with the decoded VIS code.
                    if (m_messageQueueToChannel) {
                        m_messageQueueToChannel->push(
                            SSTVDemod::MsgVIS::create(visCode, parityOK));
                    }

                    // Reset the image decoder so acquisition starts from line 0.
                    resetDecoder();
                    // Override the VIS_IDLE set by resetDecoder() to absorb the
                    // stop bit before resuming normal sync hunting.
                    m_visState = VIS_STOP_BIT;
                    m_visStateSamples = 0;
                }
            }
            break;
        }

        case VIS_STOP_BIT:
            // Absorb the 30 ms 1200 Hz stop bit, then return to VIS_IDLE.
            m_visStateSamples++;
            if (m_visStateSamples > VIS_BIT_SAMPLES / 2 && !isSyncTone) {
                m_visState = VIS_IDLE;
                m_visStateSamples = 0;
            }
            break;
        }
    }

    // Feed scope and spectrum with demodulated signals.
    sampleToSpectrum(audioSample);
    sampleToScope(audioSample, freq, isSyncTone ? 1.0f : 0.0f, m_pllLocked ? 1.0f : 0.0f, float(m_state));
}

void SSTVDemodSink::decodePixelSample(float freq, float *buf, int width, SSTVState nextState)
{
    // Accumulate frequency for the current pixel
    m_pixelAccum += freq;
    m_pixelSampleCount++;
    m_pixelSamplePos += 1.0f;

    // Check if we have accumulated enough samples for one pixel
    if (m_pixelSamplePos >= m_modeSamplesPerPixel)
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
        m_pixelSamplePos -= m_modeSamplesPerPixel;

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
    if (m_lineIndex >= m_modeLinePairs) {
        m_lineIndex = 0;
    }

    // Build two RGB scan lines from Y_odd, Cr, Cb, Y_even
    // Each Cr/Cb value applies to the same horizontal position in both lines
    // Cr and Cb are at full horizontal resolution, but half vertical resolution
    QImage lineImage(m_modeWidth, 2, QImage::Format_RGB32);

    for (int x = 0; x < m_modeWidth; x++)
    {
        // Convert from frequency (Hz) to pixel value [0..255] then offset to [-128..127].
        // SDFT path uses the piecewise-calibrated map; Hilbert path uses the direct linear map.
        float cr, cb;
        if (m_useHilbert) {
            cr = static_cast<float>(freqToPixelDirect(m_cr[x])) - 128.0f;
            cb = static_cast<float>(freqToPixelDirect(m_cb[x])) - 128.0f;
        } else {
            cr = static_cast<float>(freqToPixel(m_cr[x])) - 128.0f;
            cb = static_cast<float>(freqToPixel(m_cb[x])) - 128.0f;
        }

        // Decode odd scan line (top row of the block)
        {
            float y = m_useHilbert ? static_cast<float>(freqToPixelDirect(m_yOdd[x]))
                                   : static_cast<float>(freqToPixel(m_yOdd[x]));
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
            float y = m_useHilbert ? static_cast<float>(freqToPixelDirect(m_yEven[x]))
                                   : static_cast<float>(freqToPixel(m_yEven[x]));
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
    if (m_lineIndex >= m_modeLinePairs) {
        m_lineIndex = 0;
    }
}

void SSTVDemodSink::transitionTo(SSTVState newState)
{
    m_state = newState;
    m_stateSampleCount = 0;

    // Clear the per-visit sync-seen flag whenever we (re-)enter IN_SYNC so
    // the PLL update and transition logic start fresh for each sync pulse.
    if (newState == IN_SYNC) {
        m_syncPulseSeen = false;
    }

    // Reset pixel accumulator when starting a new decoding section.
    // The SDFT history is intentionally NOT cleared here: adjacent sections
    // (Y_odd, Cr, Cb, Y_even) all operate in the same 1500–2300 Hz frequency
    // range, so the ~3-pixel bleed-in from the previous section is mild.
    // Clearing to zero would force those pixels to fall back to the zero-power
    // default (1200 Hz), producing green/teal artefacts in the Cr/Cb channels.
    if (newState == DECODING_Y_ODD    || newState == DECODING_CR   ||
        newState == DECODING_CB       || newState == DECODING_Y_EVEN ||
        newState == DECODING_R36_Y    || newState == DECODING_R36_CHROMA ||
        newState == DECODING_SC_RED   || newState == DECODING_SC_GREEN ||
        newState == DECODING_SC_BLUE  || newState == DECODING_MT_GREEN ||
        newState == DECODING_MT_BLUE  || newState == DECODING_MT_RED)
    {
        m_pixelIndex = 0;
        m_pixelAccum = 0.0f;
        m_pixelSamplePos = 0.0f;
        m_pixelSampleCount = 0;
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

    if (force || settingsKeys.contains("sstvMode")) {
        applyMode();
        resetDecoder();
    }
}

void SSTVDemodSink::applyMode()
{
    const SSTVDemodSettings::SSTVModeParams p = SSTVDemodSettings::getModeParams(m_settings.m_sstvMode);

    m_modeWidth           = p.width;
    m_modeHeight          = p.height;
    m_modeLinePairs       = p.linesTotal;
    m_modeSamplesPerPixel = p.pixelTimeMs * float(SSTVDEMOD_CHANNEL_SAMPLE_RATE) / 1000.0f;

    // Mode-specific sync acceptance window
    const int syncSamples = (int)(p.syncMs * float(SSTVDEMOD_CHANNEL_SAMPLE_RATE) / 1000.0f + 0.5f);
    m_syncSamplesMin = (int)(syncSamples * 0.75f);
    m_syncSamplesMax = (int)(syncSamples * 1.25f) + SYNC_PORCH_DELAY;

    // Mode-specific porch handling (SYNC_PORCH_DELAY = 88 samples)
    const int porchSamples = (int)(p.porchMs * float(SSTVDEMOD_CHANNEL_SAMPLE_RATE) / 1000.0f + 0.5f);
    m_porchSamplesRemaining = std::max(0, porchSamples - SYNC_PORCH_DELAY);
    m_pixelSkipAtStart      = float(std::max(0, SYNC_PORCH_DELAY - porchSamples));

    // Inter-section separator (Robot36) / inter-channel porch (Scottie)
    m_modeInterSectionSamples = (int)(p.separatorMs * float(SSTVDEMOD_CHANNEL_SAMPLE_RATE) / 1000.0f + 0.5f);

    // Robot36 chroma
    m_modeChromaWidth           = p.chromaWidth;
    m_modeChromaSamplesPerPixel = p.chromaPixelTimeMs * float(SSTVDEMOD_CHANNEL_SAMPLE_RATE) / 1000.0f;

    // PLL nominal line period (used as initial estimate)
    float lineSectionSamples;
    if (p.family == SSTVDemodSettings::SSTVModeFamily::PD) {
        lineSectionSamples = float(m_porchSamplesRemaining)
            + 4.0f * float(m_modeWidth) * m_modeSamplesPerPixel;
    } else if (p.family == SSTVDemodSettings::SSTVModeFamily::Robot36) {
        lineSectionSamples = float(m_porchSamplesRemaining)
            + float(m_modeWidth) * m_modeSamplesPerPixel
            + float(m_modeInterSectionSamples)
            + float(m_modeChromaWidth) * m_modeChromaSamplesPerPixel;
    } else {
        // Scottie / Martin: 3 colour channels + optional inter-section
        lineSectionSamples = -m_pixelSkipAtStart
            + 3.0f * float(m_modeWidth) * m_modeSamplesPerPixel
            + float(m_modeInterSectionSamples);
    }
    m_nominalLinePeriod = lineSectionSamples + float(syncSamples) + float(SYNC_PORCH_DELAY);
    m_pllPeriod = m_nominalLinePeriod;
}

void SSTVDemodSink::decodePixelSampleEx(float freq, float *buf, int width, float samplesPerPixel, SSTVState nextState)
{
    m_pixelAccum += freq;
    m_pixelSampleCount++;
    m_pixelSamplePos += 1.0f;

    if (m_pixelSamplePos >= samplesPerPixel)
    {
        float avgFreq = m_pixelAccum / float(m_pixelSampleCount);
        buf[m_pixelIndex] = avgFreq;
        m_pixelIndex++;
        m_pixelAccum = 0.0f;
        m_pixelSampleCount = 0;
        m_pixelSamplePos -= samplesPerPixel;

        if (m_pixelIndex >= width)
        {
            transitionTo(nextState);
        }
    }
}

SSTVDemodSink::SSTVState SSTVDemodSink::getFirstDecodeState() const
{
    const SSTVDemodSettings::SSTVModeParams p = SSTVDemodSettings::getModeParams(m_settings.m_sstvMode);
    switch (p.family)
    {
        case SSTVDemodSettings::SSTVModeFamily::PD:      return DECODING_Y_ODD;
        case SSTVDemodSettings::SSTVModeFamily::Robot36: return DECODING_R36_Y;
        case SSTVDemodSettings::SSTVModeFamily::Scottie: return DECODING_SC_RED;
        case SSTVDemodSettings::SSTVModeFamily::Martin:  return DECODING_MT_GREEN;
        default:                                          return DECODING_Y_ODD;
    }
}

void SSTVDemodSink::commitBlockRobot36()
{
    if (m_lineIndex % 2 == 0)
    {
        // Even line: save Y; Cr is already in m_cr. Wait for odd line.
        memcpy(m_yEven, m_yOdd, m_modeWidth * sizeof(float));
    }
    else
    {
        // Odd line: m_yEven=even-line Y, m_yOdd=odd-line Y, m_cr=Cr, m_cb=Cb
        QImage lineImage(m_modeWidth, 2, QImage::Format_RGB32);

        for (int x = 0; x < m_modeWidth; x++)
        {
            // Chroma index: 2 image pixels share one chroma pixel (half horizontal res)
            int cx = x / 2;
            float cr, cb;
            if (m_useHilbert) {
                cr = static_cast<float>(freqToPixelDirect(m_cr[cx])) - 128.0f;
                cb = static_cast<float>(freqToPixelDirect(m_cb[cx])) - 128.0f;
            } else {
                cr = static_cast<float>(freqToPixel(m_cr[cx])) - 128.0f;
                cb = static_cast<float>(freqToPixel(m_cb[cx])) - 128.0f;
            }

            // Even line (row 0 of block)
            {
                float y = m_useHilbert ? static_cast<float>(freqToPixelDirect(m_yEven[x]))
                                       : static_cast<float>(freqToPixel(m_yEven[x]));
                int r = qBound(0, (int)(y + 1.402f * cr),                     255);
                int g = qBound(0, (int)(y - 0.34414f * cb - 0.71414f * cr),   255);
                int b = qBound(0, (int)(y + 1.772f * cb),                     255);
                lineImage.setPixel(x, 0, qRgb(r, g, b));
            }
            // Odd line (row 1 of block)
            {
                float y = m_useHilbert ? static_cast<float>(freqToPixelDirect(m_yOdd[x]))
                                       : static_cast<float>(freqToPixel(m_yOdd[x]));
                int r = qBound(0, (int)(y + 1.402f * cr),                     255);
                int g = qBound(0, (int)(y - 0.34414f * cb - 0.71414f * cr),   255);
                int b = qBound(0, (int)(y + 1.772f * cb),                     255);
                lineImage.setPixel(x, 1, qRgb(r, g, b));
            }
        }

        const int pairIndex = m_lineIndex / 2;  // integer division; m_lineIndex is odd here
        if (m_messageQueueToChannel) {
            m_messageQueueToChannel->push(SSTVDemod::MsgImage::create(lineImage, pairIndex));
        }
    }

    m_lineIndex++;
    if (m_lineIndex >= m_modeHeight) {
        m_lineIndex = 0;
    }
}

void SSTVDemodSink::commitBlockScottie()
{
    // Build single-line image from: pending Green (prev cycle), pending Blue (prev cycle), Red (m_cr this cycle)
    QImage lineImage(m_modeWidth, 1, QImage::Format_RGB32);

    for (int x = 0; x < m_modeWidth; x++)
    {
        int r, g, b;
        if (m_useHilbert) {
            r = freqToPixelDirect(m_cr[x]);
            g = freqToPixelDirect(m_scPendingGreen[x]);
            b = freqToPixelDirect(m_scPendingBlue[x]);
        } else {
            r = freqToPixel(m_cr[x]);
            g = freqToPixel(m_scPendingGreen[x]);
            b = freqToPixel(m_scPendingBlue[x]);
        }
        lineImage.setPixel(x, 0, qRgb(r, g, b));
    }

    if (m_messageQueueToChannel) {
        m_messageQueueToChannel->push(SSTVDemod::MsgImage::create(lineImage, m_lineIndex));
    }

    // Save current Green (m_yOdd) and Blue (m_cb) as pending for the next line
    memcpy(m_scPendingGreen, m_yOdd, m_modeWidth * sizeof(float));
    memcpy(m_scPendingBlue,  m_cb,   m_modeWidth * sizeof(float));

    m_lineIndex++;
    if (m_lineIndex >= m_modeHeight) {
        m_lineIndex = 0;
    }
}

void SSTVDemodSink::commitBlockMartin()
{
    // Martin: Green=m_yOdd, Blue=m_cb, Red=m_cr
    QImage lineImage(m_modeWidth, 1, QImage::Format_RGB32);

    for (int x = 0; x < m_modeWidth; x++)
    {
        int r, g, b;
        if (m_useHilbert) {
            r = freqToPixelDirect(m_cr[x]);
            g = freqToPixelDirect(m_yOdd[x]);
            b = freqToPixelDirect(m_cb[x]);
        } else {
            r = freqToPixel(m_cr[x]);
            g = freqToPixel(m_yOdd[x]);
            b = freqToPixel(m_cb[x]);
        }
        lineImage.setPixel(x, 0, qRgb(r, g, b));
    }

    if (m_messageQueueToChannel) {
        m_messageQueueToChannel->push(SSTVDemod::MsgImage::create(lineImage, m_lineIndex));
    }

    m_lineIndex++;
    if (m_lineIndex >= m_modeHeight) {
        m_lineIndex = 0;
    }
}
