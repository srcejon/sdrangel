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

#ifndef INCLUDE_SSTVDEMODSINK_H
#define INCLUDE_SSTVDEMODSINK_H

#include <QImage>
#include <QVector>

#include <cmath>

#include "dsp/channelsamplesink.h"
#include "dsp/dsptypes.h"
#include "dsp/phasediscri.h"
#include "dsp/nco.h"
#include "dsp/interpolator.h"
#include "util/movingaverage.h"
#include "util/messagequeue.h"

#include "sstvdemodsettings.h"

// Internal audio sample rate for SSTV processing (Hz)
#define SSTVDEMOD_CHANNEL_SAMPLE_RATE  48000

// Maximum image width across all supported PD modes (PD-290 = 800 px)
#define SSTVDEMOD_MAX_IMAGE_WIDTH   800

// PD120 image dimensions (kept for reference; runtime dimensions come from settings)
#define SSTVDEMOD_IMAGE_WIDTH   640
#define SSTVDEMOD_IMAGE_HEIGHT  496   // 248 pairs of scan lines

// SSTV tone frequencies (Hz)
#define SSTVDEMOD_SYNC_FREQ       1200.0f   // Sync pulse frequency
#define SSTVDEMOD_BLACK_FREQ      1500.0f   // Black level / start tone
#define SSTVDEMOD_WHITE_FREQ      2300.0f   // White level
// Sync threshold (conceptual boundary, Hz).  Sync tone = 1200 Hz; pixel data starts at
// 1500 Hz (black level).  The sync detector does NOT compare a frequency estimate against
// this value; instead it uses a dedicated energy detector (see N_SYNC / SYNC_DETECT_THRESHOLD
// below) to avoid the instability of frequency estimation near the boundary.  This macro
// is retained as documentation of the frequency boundary and for potential future use.
#define SSTVDEMOD_SYNC_THRESHOLD  1420.0f   // Hz — conceptual boundary between sync and pixel data

// PD120 timing in milliseconds
#define SSTVDEMOD_SYNC_MS         20.0f     // Scan-line sync pulse duration
#define SSTVDEMOD_PORCH_MS        2.08f     // Porch duration after sync (at black level, 1500 Hz)
// Measured pixel clock: 190 µs/pixel (empirically confirmed; gives ~96 s/frame for PD120).
#define SSTVDEMOD_PIXEL_TIME_MS   0.190f

// Timing in samples at SSTVDEMOD_CHANNEL_SAMPLE_RATE
#define SSTVDEMOD_SYNC_SAMPLES      ((int)(SSTVDEMOD_SYNC_MS * SSTVDEMOD_CHANNEL_SAMPLE_RATE / 1000.0f))
#define SSTVDEMOD_PORCH_SAMPLES     ((int)(SSTVDEMOD_PORCH_MS * SSTVDEMOD_CHANNEL_SAMPLE_RATE / 1000.0f))
#define SSTVDEMOD_SAMPLES_PER_PIXEL (SSTVDEMOD_PIXEL_TIME_MS * SSTVDEMOD_CHANNEL_SAMPLE_RATE / 1000.0f)

// Scan-line sync pulse duration bounds (samples) for PD120 — used as default initialisers only.
// Mode-specific bounds are computed in applyMode() and stored in m_syncSamplesMin / m_syncSamplesMax.
#define SSTVDEMOD_SYNC_SAMPLES_MIN  ((int)(SSTVDEMOD_SYNC_SAMPLES * 0.75f))   // 720  – 15 ms lower bound
#define SSTVDEMOD_SYNC_SAMPLES_MAX  ((int)(SSTVDEMOD_SYNC_SAMPLES * 1.25f) + SYNC_PORCH_DELAY)  // 1288 – upper bound



class ScopeVis;
class BasebandSampleSink;
class SSTVDemod;

class SSTVDemodSink : public ChannelSampleSink {
public:
    /** SSTV PD120 decoder state machine states */
    enum SSTVState {
        WAITING_FOR_SYNC,  //!< Scanning for a sync pulse
        IN_SYNC,           //!< Currently inside a sync pulse, waiting for it to end
        IN_PORCH,          //!< In porch period after sync
        DECODING_Y_ODD,    //!< Decoding Y (luminance) for odd scan line (pixels 0..639)
        DECODING_CR,       //!< Decoding Cr (red-difference chroma) for both lines (pixels 0..319)
        DECODING_CB,       //!< Decoding Cb (blue-difference chroma) for both lines (pixels 0..319)
        DECODING_Y_EVEN,   //!< Decoding Y (luminance) for even scan line (pixels 0..639)
        // Robot36 states
        DECODING_R36_Y,      //!< Robot36: decoding Y (luminance) for current scan line
        DECODING_R36_SEP,    //!< Robot36: waiting through the 4.5 ms Y–chroma separator
        DECODING_R36_CHROMA, //!< Robot36: decoding Cr (even lines) or Cb (odd lines) chroma
        // Scottie states (after sync: porch + Red + Green + porch + Blue)
        DECODING_SC_RED,     //!< Scottie: decoding Red channel of current line
        DECODING_SC_GREEN,   //!< Scottie: decoding Green channel of current line
        DECODING_SC_PORCH,   //!< Scottie: waiting through 1.5 ms inter-channel porch (G → B)
        DECODING_SC_BLUE,    //!< Scottie: decoding Blue channel of current line
        // Martin states (after sync+porch: Green + Blue + Red, no separators)
        DECODING_MT_GREEN,   //!< Martin: decoding Green channel
        DECODING_MT_BLUE,    //!< Martin: decoding Blue channel
        DECODING_MT_RED,     //!< Martin: decoding Red channel
    };

    SSTVDemodSink();
    ~SSTVDemodSink();

    virtual void feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end);

    void setScopeSink(ScopeVis* scopeSink) { m_scopeSink = scopeSink; }
    void setSpectrumSink(BasebandSampleSink* spectrumSink) { m_spectrumSink = spectrumSink; }
    void applyChannelSettings(int channelSampleRate, int channelFrequencyOffset, bool force = false);
    void applySettings(const QStringList& settingsKeys, const SSTVDemodSettings& settings, bool force = false);
    void setMessageQueueToChannel(MessageQueue *messageQueue) { m_messageQueueToChannel = messageQueue; }

    double getMagSq() const { return m_magsq; }

    void getMagSqLevels(double& avg, double& peak, int& nbSamples)
    {
        if (m_magsqCount > 0)
        {
            m_magsq = m_magsqSum / m_magsqCount;
            m_magSqLevelStore.m_magsq = m_magsq;
            m_magSqLevelStore.m_magsqPeak = m_magsqPeak;
        }

        avg = m_magSqLevelStore.m_magsq;
        peak = m_magSqLevelStore.m_magsqPeak;
        nbSamples = m_magsqCount == 0 ? 1 : m_magsqCount;

        m_magsqSum = 0.0f;
        m_magsqPeak = 0.0f;
        m_magsqCount = 0;
    }

    /** Reset the SSTV decoder state and clear the image buffer */
    void resetDecoder();

private:
    struct MagSqLevelsStore
    {
        MagSqLevelsStore() :
            m_magsq(1e-12),
            m_magsqPeak(1e-12)
        {}
        double m_magsq;
        double m_magsqPeak;
    };

    SSTVDemodSettings m_settings;
    int m_channelSampleRate;
    int m_channelFrequencyOffset;

    ScopeVis *m_scopeSink;                //!< Scope GUI to display demod waveforms
    BasebandSampleSink *m_spectrumSink;   //!< Spectrum GUI to display fmDemod spectrum

    ComplexVector m_sampleBuffer[SSTVDemodSettings::m_scopeStreams];
    static const int m_sampleBufferSize = SSTVDEMOD_CHANNEL_SAMPLE_RATE / 20;
    int m_sampleBufferIndex;

    SampleVector m_specBuffer;
    static const int m_specBufferSize = SSTVDEMOD_CHANNEL_SAMPLE_RATE / 20;
    int m_specBufferIndex;

    NCO m_nco;
    Interpolator m_interpolator;
    Real m_interpolatorDistance;
    Real m_interpolatorDistanceRemain;

    double m_magsq;
    double m_magsqSum;
    double m_magsqPeak;
    int m_magsqCount;
    MagSqLevelsStore m_magSqLevelStore;

    MessageQueue *m_messageQueueToChannel;

    MovingAverageUtil<Real, double, 16> m_movingAverage;
    PhaseDiscriminators m_phaseDiscri;      //!< RF FM discriminator

    // -----------------------------------------------------------------------
    // Sliding-DFT spectral moment (MATLAB 'instfreq' tfmoment equivalent).
    //
    // The rectangular-window SDFT of the last N_SDFT samples is maintained
    // incrementally with one complex multiply per bin per input sample:
    //   Z[k] ← twiddle[k] · (Z[k] + x_new − x_old),   twiddle[k] = e^{+j·2π·k/N}
    //
    // Instantaneous frequency is the power-weighted spectral centroid:
    //   freq_raw = (Fs/N) · Σ_{k=K_MIN}^{K_MAX} k·|Z[k]|² / Σ |Z[k]|²
    //
    // Moving-average smoothing (SDFT_FREQ_MA_LEN samples):
    //   freq = MA(freq_raw)
    //
    // N_SDFT=32 reduces horizontal pixel blur to ~3.5 pixels (vs ~7 for N=64 or ~14
    // for N=128).  The 0.67 ms window fits comfortably within the 2.08 ms porch.
    //
    // Bins and frequency range (N=32, Fs=48000):
    //   Bin width = Fs/N = 1500 Hz.
    //   k=1: 1500 Hz (black level, pixel 0)
    //   k=2: 3000 Hz (above white level)
    //   Using k=1..2 covers the SSTV pixel range 1500–2300 Hz with a
    //   power-weighted centroid that varies from 1500 Hz (all-black, full
    //   energy in k=1) to ~2344 Hz (full-white, energy shared between k=1
    //   and k=2).  A piecewise calibration handles the inherent non-linearity.
    //
    // SDFT oscillation and the need for the MA:
    //   Because fmDemod = cos(2π·f_tone·t) is a real-valued signal, each SDFT
    //   bin receives contributions from both positive (k₀=f_tone·N/Fs) and
    //   negative (k₀_neg=N−k₀) frequency components.  The beat between these
    //   two components makes every bin magnitude oscillate at 2·f_tone, so the
    //   power-weighted centroid also oscillates at that rate.  For the 1200 Hz
    //   sync tone the period is Fs/(2·1200)=20 samples.  Applying a
    //   SDFT_FREQ_MA_LEN=40-sample MA (exactly two 1200 Hz periods) completely
    //   cancels the sync-tone oscillation, yielding a stable reading well below
    //   the pixel-level threshold.  For the Hilbert path, the same MA cancels
    //   the 1200 Hz and 2400 Hz Hilbert-FIR oscillations (see Hilbert section).
    //
    // Centroid linearisation (centroidToFreq):
    //   The centroid is only unbiased when summed over all N/2 bins.  Using
    //   k=1..2 (with N=32) introduces a systematic non-linear frequency shift
    //   that varies from 0 Hz at 1500 Hz to −210 Hz near 1900 Hz.
    //
    //   centroidToFreq() inverts this non-linearity using a 9-point lookup table
    //   (SDFT_CALIB_CENTROIDS) derived from hardware measurements: each entry is the
    //   observed steady-state MA centroid when a pure tone at the corresponding
    //   true frequency (1500–2300 Hz in 100 Hz steps) is applied.  Piecewise-linear
    //   interpolation between entries linearises the response across the full SSTV
    //   pixel range.  Tones below 1500 Hz are clamped to 1500 Hz.
    //
    //   This linearised value replaces the raw centroid as the output of the SDFT
    //   path, so both paths (SDFT and Hilbert) produce the same "true Hz" quantity.
    //   freqToPixelDirect() then maps both to pixels with a single linear formula.
    //
    // The SDFT history is NOT reset at section transitions (Y_odd→Cr→Cb→Y_even).
    // Adjacent sections share the same 1500–2300 Hz frequency range, so the
    // ~6-pixel bleed-in from the previous section is mild and self-correcting.
    // Resetting to zero would force those pixels to 1200 Hz (below black level),
    // producing green/teal artefacts in the Cr/Cb sections — worse than the
    // natural contamination.  The SDFT is only fully cleared in resetDecoder().
    // -----------------------------------------------------------------------
    static constexpr int N_SDFT             = 32;  //!< Sliding DFT window length (samples); bin width = Fs/N = 1500 Hz
    static constexpr int SDFT_FREQ_MA_LEN   = 40;  //!< Freq MA length = two periods of 1200 Hz sync tone; cancels centroid oscillation
    static constexpr int SDFT_K_STORE_MIN   = 1;   //!< Lowest stored bin  (k=1 → 1500 Hz, black level)
    static constexpr int SDFT_K_STORE_MAX   = 2;   //!< Highest stored bin (k=2 → 3000 Hz, above white level)
    static constexpr int SDFT_K_SUM_MIN     = 1;   //!< First bin in the moment sum
    static constexpr int SDFT_K_SUM_MAX     = 2;   //!< Last  bin in the moment sum
    static constexpr int SDFT_NUM_BINS      = SDFT_K_STORE_MAX - SDFT_K_STORE_MIN + 1; // 2

    // Centroid calibration lookup table: 9 entries at 100 Hz steps from 1500 Hz to 2300 Hz.
    // SDFT_CALIB_CENTROIDS[i] = measured steady-state MA centroid (Hz) for a true tone at
    // (SDFT_CALIB_TRUE_MIN + SDFT_CALIB_TRUE_STEP·i) Hz.
    // Derived from hardware measurements (not simulation): the raw SDFT centroid output
    // was observed at each 100 Hz step across the SSTV pixel range.
    static constexpr int   SDFT_CALIB_N         = 9;       //!< Number of centroid calibration entries
    static constexpr float SDFT_CALIB_TRUE_MIN  = 1500.0f; //!< True frequency (Hz) for first table entry
    static constexpr float SDFT_CALIB_TRUE_STEP = 100.0f;  //!< True-frequency step (Hz) between entries
    static constexpr float SDFT_CALIB_CENTROIDS[SDFT_CALIB_N] = {
        1511.0f, 1520.0f, 1550.0f, 1616.0f, 1712.0f, 1840.0f, 1980.0f, 2160.0f, 2320.0f
    };

    float   m_sdftBuf[N_SDFT];            //!< Circular ring buffer of fmDemod samples
    Complex m_sdftBins[SDFT_NUM_BINS];    //!< Running SDFT bins k = SDFT_K_STORE_MIN..SDFT_K_STORE_MAX
    Complex m_sdftTwiddle[SDFT_NUM_BINS]; //!< Twiddle factors e^{+j·2π·k/N} per bin
    int     m_sdftIdx;                    //!< Next write position in m_sdftBuf (0..N_SDFT−1)

    MovingAverageUtil<float, double, SDFT_FREQ_MA_LEN> m_freqMovAvg; //!< Post-SDFT/Hilbert freq MA; shared by both paths (pixel decode)

    // -----------------------------------------------------------------------
    // Dedicated sync-tone energy detector.
    //
    // The pixel SDFT (N=32, k=1..2) estimates tone frequency for pixel
    // decoding but is not used for sync detection.  Sync detection uses a
    // completely separate approach: measure the energy at the exact sync
    // frequency (1200 Hz) via a single-bin sliding DFT.
    //
    // Why N_SYNC = 160 and SYNC_K = 4:
    //   N_SYNC = Fs / (f_black − f_sync) = 48000 / (1500 − 1200) = 160
    //   SYNC_K = f_sync × N_SYNC / Fs   = 1200 × 160 / 48000    = 4
    //   This choice places exact DFT nulls at every multiple of Fs/N_SYNC = 300 Hz
    //   above the bin: 1500, 1800, 2100, 2400 Hz — the entire pixel-data
    //   frequency range.  The sync detector therefore produces zero output for
    //   any pure pixel-data tone regardless of its frequency.
    //
    // Energy normalisation:
    //   The raw bin energy |Z_sync[4]|² is divided by (N_SYNC²/4) × totalPower,
    //   where totalPower = MA(fmDemod²).  This produces a dimensionless ratio:
    //     syncRatio = |Z_sync|² / ((N_SYNC²/4) × totalPower)
    //   For a sustained pure 1200 Hz tone of any amplitude: syncRatio → 1.0
    //   For any pixel-data tone (1500+ Hz):                  syncRatio → 0.0
    //   For 1420 Hz (sync/pixel boundary):                   syncRatio ≈ 0.10
    //   The result is independent of signal amplitude.
    //
    // Timing compensation (SYNC_PORCH_SAMPLES):
    //   After the sync tone ends and the porch (1500 Hz) begins, the
    //   N_SYNC=160 sample window takes time to flush.  For SYNC_DETECT_THRESHOLD
    //   = 0.4, the exit is detected after ≈ SYNC_PORCH_DELAY = 88 porch samples.
    //   SYNC_PORCH_SAMPLES = PORCH_SAMPLES − SYNC_PORCH_DELAY (= 11 samples)
    //   ensures that by the time decoding starts the full PORCH_SAMPLES have
    //   elapsed, keeping pixel-start timing identical to the spec.
    //   At that point the pixel SDFT window (N=64) has had ≥ 99 porch samples —
    //   more than enough to flush all sync-frequency content before decoding.
    // -----------------------------------------------------------------------
    static constexpr int   N_SYNC                = 160;   //!< Sync SDFT window length: Fs/(f_black−f_sync) = 48000/300 = 160
    static constexpr int   SYNC_K                = 4;     //!< Sync SDFT bin: f_sync×N_SYNC/Fs = 1200×160/48000 = 4
    static constexpr float SYNC_DETECT_THRESHOLD = 0.4f;  //!< Normalised energy threshold: >0.4 → sync, ≤0.4 → pixel/porch
    static constexpr int   SYNC_PORCH_DELAY      = 88;    //!< Samples into porch before sync energy drops below threshold
    static constexpr int   SYNC_PORCH_SAMPLES    = SSTVDEMOD_PORCH_SAMPLES - SYNC_PORCH_DELAY; //!< = 11 samples

    float   m_syncSdftBuf[N_SYNC]; //!< Ring buffer for the sync-energy sliding DFT
    Complex m_syncSdftBin;         //!< Running SDFT bin at bin k=SYNC_K (1200 Hz)
    Complex m_syncSdftTwiddle;     //!< Twiddle factor e^{+j·2π·SYNC_K/N_SYNC}
    int     m_syncSdftIdx;         //!< Next write position in m_syncSdftBuf (0..N_SYNC−1)

    MovingAverageUtil<float, double, SDFT_FREQ_MA_LEN> m_syncTotalPowerMa; //!< MA of fmDemod² for sync-ratio normalisation

    // -----------------------------------------------------------------------
    // Sync timing PLL.
    //
    // Without the PLL every scan-line transition to IN_PORCH is triggered by
    // the raw isSyncTone falling edge, whose sample position has noise-driven
    // jitter of several samples (σ ≈ 10–30 samples at moderate SNR).  At
    // 9.12 samples/pixel this produces ±1–3 pixel misalignment per line,
    // making vertical edges in the image look jagged.
    //
    // After the first valid sync pulse the PLL tracks the line-block period
    // with a first-order IIR and uses the smoothed estimate to control the
    // IN_SYNC → IN_PORCH transition, reducing per-line jitter by a factor of
    // √(α/2) ≈ 0.22 for PLL_ALPHA=0.1 (σ_pll ≈ 0.22 × σ_raw ≈ sub-pixel).
    // Clock-frequency differences between transmitter and receiver (typically
    // < 100 ppm) cause m_pllPeriod to drift by < 2.5 samples/line; PLL_ALPHA
    // = 0.1 tracks that drift within ~10 lines.
    //
    // Phase reference:
    //   m_pllPhase resets to 0 (minus any overshoot carry-over) each time we
    //   enter IN_PORCH.  It increments by 1 every sample while m_pllLocked is
    //   true (i.e., in all states after the first valid sync).
    //
    // Expected phase trace (NOMINAL_LINE_PERIOD_SAMPLES = 24406.2):
    //   0       IN_PORCH entry
    //   11      DECODING_Y_ODD entry (after SYNC_PORCH_SAMPLES)
    //   23358   WAITING_FOR_SYNC entry (after 4 × 640 × 9.12 pixel samples)
    //   24318   isSyncTone rises (sync pulse starts, ~0 gap after Y_even)
    //   24406   isSyncTone falls (SYNC_PORCH_DELAY=88 samples into porch) ← m_pllPeriod
    //
    // Loop update (each line, when isSyncTone falls with a valid duration):
    //   error      = m_pllPhase − m_pllPeriod
    //   m_pllPeriod += PLL_ALPHA × error   (clamped to ±10 % of nominal)
    //
    // Transition rule:
    //   m_syncPulseSeen is set when isSyncTone falls with a valid duration.
    //   The IN_SYNC → IN_PORCH transition fires at m_pllPhase ≥ m_pllPeriod
    //   (not at the raw isSyncTone-fall sample), with overshoot carried into
    //   the next period via m_pllPhase -= (int)m_pllPeriod.
    // -----------------------------------------------------------------------
    static constexpr float PLL_ALPHA = 0.1f;   //!< Loop gain (time constant ≈ 10 scan-line blocks)
    // Nominal period (samples): SYNC_PORCH_SAMPLES remaining porch + 4 pixel sections
    //   + SYNC_SAMPLES of the next sync pulse + SYNC_PORCH_DELAY samples until isSyncTone falls.
    // Nominal period for PD-120 (samples): used only as an initialiser default.
    // The runtime period m_nominalLinePeriod is recomputed per mode in applyMode().
    static constexpr float NOMINAL_LINE_PERIOD_SAMPLES =
        float(SYNC_PORCH_SAMPLES)
        + 4.0f * float(SSTVDEMOD_IMAGE_WIDTH) * SSTVDEMOD_SAMPLES_PER_PIXEL
        + float(SSTVDEMOD_SYNC_SAMPLES)
        + float(SYNC_PORCH_DELAY);

    bool  m_pllLocked;      //!< True after the first valid sync pulse has been accepted
    int   m_pllPhase;       //!< Samples since last IN_PORCH entry (PLL phase accumulator)
    float m_pllPeriod;      //!< Estimated line-block period (samples); initialised to m_nominalLinePeriod
    bool  m_syncPulseSeen;  //!< True once isSyncTone has fallen with a valid duration in the current IN_SYNC visit

    // -----------------------------------------------------------------------
    // Hilbert instantaneous-frequency estimator (alternative to SDFT above).
    //
    // Selected by setting m_useHilbert = true.
    //
    // Step 1 — Analytic signal via 63-tap Hamming-windowed Hilbert FIR:
    //   For each real input sample x[n] (= fmDemod), the Hilbert transformer
    //   produces the imaginary part of the analytic signal:
    //     x̂[n] = Σ_{k=0}^{62} h[k] · x[n−k]
    //   Only the 32 even-indexed taps (k=0,2,...,62) are non-zero because this
    //   is a type-III FIR (antisymmetric, zero centre tap at k=31).
    //   The real part is the input delayed by (N−1)/2 = 31 samples:
    //     x_r[n] = x[n−31]
    //   Together: z[n] = x_r[n] + j·x̂[n]
    //
    //   Why 63 taps instead of the 31 used previously:
    //     The 31-tap filter had |H(1200 Hz)| ≈ 0.70, placing the sync tone
    //     deep in its transition band.  The resulting elliptical phasor caused
    //     the phase-difference estimator to oscillate at 2×1200 Hz with an
    //     amplitude of (1−0.70)/(1+0.70) × 2400 ≈ ±416 Hz, sweeping the sync
    //     reading from 784 to 1616 Hz.  The 3000 Hz post-LPF barely attenuated
    //     this (only −2.1 dB at 2400 Hz).
    //     With 63 taps, |H(1200 Hz)| = 0.985, reducing the oscillation to
    //     (1−0.985)/(1+0.985) × 2400 ≈ ±18 Hz — negligible for sync detection.
    //     The group delay increases from 15 to 31 samples (3.4 pixels), which
    //     is still far less than the SDFT path's ~11 pixels.
    //
    //   The quadrature-mixing + IIR BPF approach was also tried but suffers an
    //   identical image problem: mixing a real signal down to complex baseband
    //   produces both a signal component at (f−f_c) and an image at −(f+f_c),
    //   and a real-coefficient IIR cannot distinguish them.  The image-to-signal
    //   ratio with a 700 Hz IIR at the sync tone is ≈ 0.30, giving the same
    //   ±700 Hz oscillation range.  The Hilbert FIR properly forms the one-sided
    //   (analytic) signal, eliminating the image completely.
    //
    // Step 2 — Phase-difference instantaneous frequency:
    //   f_raw = (Fs/2π) · arg( conj(z[n−1]) · z[n] )
    //         = (Fs/2π) · atan2( Im(conj(z_prev)·z_curr),
    //                             Re(conj(z_prev)·z_curr) )
    //   When |z[n]| is near zero (no signal), f_raw falls back to BLACK_FREQ.
    //
    // Step 3 — 40-sample moving average (shared with SDFT path, m_freqMovAvg):
    //   The phase-difference estimator on a real (non-analytic) input produces
    //   two systematic oscillations in f_raw that must be suppressed:
    //
    //   (a) At 2×f_tone (2400 Hz for sync): the Hilbert FIR has |H(1200 Hz)| = 0.985 < 1,
    //       so the analytic phasor is slightly elliptical (semi-axes 1 and 0.985).
    //       The instantaneous-frequency of an ellipse oscillates at 2θ, giving
    //       ≈ ±18 Hz at 2400 Hz.
    //   (b) At f_tone (1200 Hz): any residual DC offset in the DC-blocked signal
    //       (e.g., during the DC blocker convergence transient) shifts the phasor
    //       off-centre, producing a component in atan2 that oscillates at f_tone.
    //
    //   The 3000 Hz IIR LPF previously used here attenuated these components by
    //   only −7 dB (1200 Hz) and −2 dB (2400 Hz) — clearly insufficient.
    //
    //   A 40-sample MA (SDFT_FREQ_MA_LEN = 40 = Fs/1200) has exact nulls at every
    //   multiple of Fs/40 = 1200 Hz, completely eliminating both (a) and (b):
    //     |H_MA(1200)| = |sin(π·1200·40/Fs)| / (40·|sin(π·1200/Fs)|) = |sin(π)|/… = 0
    //     |H_MA(2400)| = |sin(2π)| /… = 0
    //   Group delay = 19.5 samples ≈ 2.1 pixels; combined with the 31-sample Hilbert
    //   FIR delay the total latency is 50.5 samples ≈ 5.5 pixels — still less than
    //   the SDFT path (~11 pixels).
    //
    //   Since the SDFT and Hilbert paths are mutually exclusive (m_useHilbert flag),
    //   the shared m_freqMovAvg instance is used; it is reset in resetDecoder().
    //
    // Pixel mapping uses freqToPixelDirect(): a simple linear map
    //   [SSTVDEMOD_BLACK_FREQ, SSTVDEMOD_WHITE_FREQ] → [0, 255].
    //   Both paths produce a linearised true-Hz estimate so the same map applies.
    // -----------------------------------------------------------------------
    static constexpr int   N_HILBERT          = 63;          //!< Hilbert FIR length (samples); group delay = 31 samples = 3.4 pixels
    static constexpr int   HILBERT_M          = (N_HILBERT - 1) / 2; //!< Centre tap index = 31

    // 63-tap Hamming-windowed Hilbert FIR (type III, antisymmetric).
    // Non-zero only at even indices (odd offsets from centre index 31).
    // h[k] = (2/(pi*(k-31))) * (0.54 - 0.46*cos(2*pi*k/62)) for odd (k-31), else 0.
    // Antisymmetry: HILBERT_COEFFS[k] = -HILBERT_COEFFS[62-k].
    static constexpr float HILBERT_COEFFS[N_HILBERT] = {
        -0.00164289f, +0.00000000f, -0.00196290f, +0.00000000f,  // k=0..3
        -0.00276527f, +0.00000000f, -0.00413673f, +0.00000000f,  // k=4..7
        -0.00617453f, +0.00000000f, -0.00899382f, +0.00000000f,  // k=8..11
        -0.01274042f, +0.00000000f, -0.01761352f, +0.00000000f,  // k=12..15
        -0.02390714f, +0.00000000f, -0.03209054f, +0.00000000f,  // k=16..19
        -0.04297654f, +0.00000000f, -0.05811410f, +0.00000000f,  // k=20..23
        -0.08085332f, +0.00000000f, -0.11996456f, +0.00000000f,  // k=24..27
        -0.20772989f, +0.00000000f, -0.63511728f, +0.00000000f,  // k=28..31
        +0.63511728f, +0.00000000f, +0.20772989f, +0.00000000f,  // k=32..35
        +0.11996456f, +0.00000000f, +0.08085332f, +0.00000000f,  // k=36..39
        +0.05811410f, +0.00000000f, +0.04297654f, +0.00000000f,  // k=40..43
        +0.03209054f, +0.00000000f, +0.02390714f, +0.00000000f,  // k=44..47
        +0.01761352f, +0.00000000f, +0.01274042f, +0.00000000f,  // k=48..51
        +0.00899382f, +0.00000000f, +0.00617453f, +0.00000000f,  // k=52..55
        +0.00413673f, +0.00000000f, +0.00276527f, +0.00000000f,  // k=56..59
        +0.00196290f, +0.00000000f, +0.00164289f                  // k=60..62
    };

    // NOTE: the Hilbert path now shares m_freqMovAvg (see Step 3 comment above) and
    // no longer uses the HILBERT_LPF_ALPHA IIR filter.

    // DC-blocking high-pass filter applied to fmDemod before the Hilbert ring buffer.
    //
    // The FM discriminator output fmDemod = audio_tone / fmDeviation has a DC component
    // equal to f_carrier_offset / fmDeviation whenever the receiver is not tuned exactly
    // to the transmitter carrier.  Even a 2.5 kHz offset with fmDeviation = 5000 Hz gives
    // DC = 0.5, shifting the analytic phasor z off-centre.  An off-centre phasor produces
    // an instantaneous-frequency estimate that oscillates at f_tone Hz with amplitude
    // proportional to |DC/A|, degrading sync detection (0–4000 Hz observed with DC ≈ −0.9).
    //
    // The SDFT path is inherently DC-insensitive because it sums only bins k=1..3 (750–
    // 2250 Hz) and never includes the DC bin k=0.  The Hilbert path must block DC explicitly.
    //
    // First-order DC blocker (classic single-zero, single-pole HPF):
    //   y[n] = x[n] − x[n−1] + R·y[n−1]
    //   H(z) = (1 − z⁻¹) / (1 − R·z⁻¹)
    //   f_cutoff ≈ (1−R)·Fs/(2π) ≈ 200 Hz
    //   Gain at 1200 Hz = 0.9992 (essentially unity; negligible impact on frequency estimate)
    //   Convergence time constant ≈ 1/(1−R) ≈ 38 samples ≈ 0.8 ms << 20 ms sync period
    //
    // The DC block is applied before writing to m_hilbertBuf, so both the delayed real
    // part (x_r[n] = buf[n−31]) and the imaginary part (Hilbert FIR output) are computed
    // from the DC-free signal.  No phase mismatch is introduced between the two paths.
    static constexpr float HILBERT_DC_R = 0.97382f; //!< DC-blocker pole; R = 1 − 2π×200/48000 ≈ 0.9738

    float   m_hilbertBuf[N_HILBERT];   //!< Circular ring buffer of DC-blocked fmDemod samples
    int     m_hilbertIdx;              //!< Next write index in m_hilbertBuf (0..N_HILBERT-1)
    float   m_hilbertDcXPrev;          //!< Previous raw input x[n-1] for the DC-blocking HPF
    float   m_hilbertDcY;              //!< Previous HPF output y[n-1] for the DC-blocking HPF
    Complex m_hilbertZ;                //!< Previous analytic sample z[n-1] for phase-difference calculation
    // Post-smoothing uses m_freqMovAvg (shared with SDFT path; paths are mutually exclusive).

    bool    m_useHilbert;              //!< true = Hilbert IF path; false = SDFT spectral-centroid path (default)

    // SSTV decoder state
    SSTVState m_state;
    int m_stateSampleCount;  //!< Number of samples spent in current state

    // Pixel sampling state (sub-pixel accumulation)
    int m_pixelIndex;          //!< Index of current pixel being accumulated (within section)
    float m_pixelAccum;        //!< Accumulated frequency value for current pixel
    float m_pixelSamplePos;    //!< Fractional sample position within current pixel period
    int m_pixelSampleCount;    //!< Actual number of samples accumulated in current pixel (denominator for average)

    // PD mode line buffers — dimensioned to the maximum supported width (PD-290 = 800 px).
    // Only the first m_modeWidth elements are used at any given time.
    float m_yOdd[SSTVDEMOD_MAX_IMAGE_WIDTH];   //!< Y (luminance) values for odd line
    float m_cr[SSTVDEMOD_MAX_IMAGE_WIDTH];     //!< Cr (chroma-red) values shared by both lines
    float m_cb[SSTVDEMOD_MAX_IMAGE_WIDTH];     //!< Cb (chroma-blue) values shared by both lines
    float m_yEven[SSTVDEMOD_MAX_IMAGE_WIDTH];  //!< Y (luminance) values for even line

    // Pending Scottie green/blue from the previous decode cycle
    float m_scPendingGreen[SSTVDEMOD_MAX_IMAGE_WIDTH];
    float m_scPendingBlue[SSTVDEMOD_MAX_IMAGE_WIDTH];

    // -----------------------------------------------------------------------
    // Runtime mode parameters — refreshed by applyMode() whenever m_sstvMode changes.
    // -----------------------------------------------------------------------
    int   m_modeWidth;           //!< Active image width  (pixels)
    int   m_modeHeight;          //!< Active image height (pixels)
    int   m_modeLinePairs;       //!< Active number of scan-line pairs (= m_modeHeight / 2)
    float m_modeSamplesPerPixel; //!< Active samples per pixel (fractional)
    float m_nominalLinePeriod;   //!< Active nominal PLL line-period (samples)

    // Mode-specific timing (refreshed by applyMode())
    int   m_syncSamplesMin;         //!< Min acceptable sync+porch count (mode-dependent)
    int   m_syncSamplesMax;         //!< Max acceptable sync+porch count (mode-dependent)
    int   m_porchSamplesRemaining;  //!< IN_PORCH countdown after SYNC_PORCH_DELAY
    float m_pixelSkipAtStart;       //!< Samples into first channel already elapsed when IN_PORCH fires (= SYNC_PORCH_DELAY − porchSamples, clamped to 0); decomposed in IN_PORCH into integer skipPixels and fractional m_pixelSamplePos
    int   m_modeInterSectionSamples; //!< Samples for inter-section gap (Robot36 sep / Scottie G→B porch)
    float m_modeChromaSamplesPerPixel; //!< Robot36 chroma samples per pixel
    int   m_modeChromaWidth;        //!< Robot36 chroma pixels per line (160); 0 otherwise

    int m_lineIndex;    //!< Current block index (0..linePairs−1, each block = 2 scan lines)

    // -----------------------------------------------------------------------
    // VIS (Vertical Interval Signalling) header detector.
    //
    // Runs in parallel with the main decoder state machine, listening for the
    // standard SSTV preamble sequence that precedes every image transmission:
    //
    //   Leader 1  : 300 ms @ 1900 Hz (neutral grey, pixel ≈ 128)
    //   Break     : 10 ms  @ 1200 Hz (sync tone, isSyncTone = true)
    //   Leader 2  : 300 ms @ 1900 Hz
    //   VIS start : 30 ms  @ 1200 Hz (start bit, isSyncTone = true)
    //   VIS bits  : 8 × 30 ms — each either 1100 Hz (binary 1) or 1300 Hz (binary 0)
    //   VIS stop  : 30 ms  @ 1200 Hz
    //
    // When a valid VIS code is decoded the main decoder is reset (so image
    // acquisition starts from line 0) and a MsgVIS is sent to the GUI.
    //
    // VIS bit detection uses two Goertzel single-bin DFTs evaluated over each
    // 30 ms (VIS_BIT_SAMPLES = 1440) window:
    //   k_1100 = 1100 × 1440 / 48000 = 33  (exact integer bin)
    //   k_1300 = 1300 × 1440 / 48000 = 39  (exact integer bin)
    // The bit is 1 when the 1100 Hz Goertzel energy exceeds the 1300 Hz energy.
    // -----------------------------------------------------------------------
    enum VISState {
        VIS_IDLE,        //!< No header in progress
        VIS_BREAK,       //!< First leader complete; waiting for isSyncTone rising edge then counting the 1200 Hz break
        VIS_LEADER2,     //!< Break passed; collecting second 1900 Hz leader
        VIS_START_BIT,   //!< Second leader complete; waiting then counting one VIS_BIT_SAMPLES window for the start bit
        VIS_BITS,        //!< Decoding 8 VIS bits
        VIS_STOP_BIT     //!< All bits received; waiting for stop bit to pass
    };

    // VIS timing constants (samples at SSTVDEMOD_CHANNEL_SAMPLE_RATE = 48000 Hz)
    static constexpr int VIS_LEADER_MIN_SAMPLES = 9600;   //!< Min leader duration: 200 ms
    static constexpr int VIS_BREAK_MIN_SAMPLES  = 240;    //!< Min break duration:  5 ms
    static constexpr int VIS_BREAK_MAX_SAMPLES  = 960;    //!< Max break duration: 20 ms
    static constexpr int VIS_BIT_SAMPLES        = 1440;   //!< Samples per VIS bit: 30 ms
    static constexpr int VIS_LEADER_PIXEL_MIN   = 96;     //!< Leader tone pixel lower bound (128 − 32)
    static constexpr int VIS_LEADER_PIXEL_MAX   = 160;    //!< Leader tone pixel upper bound (128 + 32)

    // Goertzel coefficients: 2·cos(2π·k/N) with N = VIS_BIT_SAMPLES = 1440
    //   k_1100 = 1100 × 1440 / 48000 = 33   → 2·cos(2π·33/1440) ≈ 1.97926
    //   k_1300 = 1300 × 1440 / 48000 = 39   → 2·cos(2π·39/1440) ≈ 1.97121
    static constexpr float GOERTZEL_1100_COEFF = 1.97926f; //!< 2·cos(2π·33/1440) for 1100 Hz
    static constexpr float GOERTZEL_1300_COEFF = 1.97121f; //!< 2·cos(2π·39/1440) for 1300 Hz

    VISState m_visState;           //!< Current VIS header detector state
    int      m_visStateSamples;    //!< Samples accumulated in current VIS sub-state
    int      m_visBitCount;        //!< Number of VIS bits decoded so far (0–8)
    int      m_visByte;            //!< Accumulated VIS byte (bits shifted in LSB-first)
    float    m_goertzel1100_s1;    //!< Goertzel delay-1 register for 1100 Hz
    float    m_goertzel1100_s2;    //!< Goertzel delay-2 register for 1100 Hz
    float    m_goertzel1300_s1;    //!< Goertzel delay-1 register for 1300 Hz
    float    m_goertzel1300_s2;    //!< Goertzel delay-2 register for 1300 Hz
    int      m_goertzelCount;      //!< Samples fed to Goertzel for the current bit

    void processOneSample(Complex &ci);
    void decodePixelSample(float freq, float *buf, int width, SSTVState nextState);
    void decodePixelSampleEx(float freq, float *buf, int width, float samplesPerPixel, SSTVState nextState);
    void commitBlock();
    void commitBlockRobot36();
    void commitBlockScottie();
    void commitBlockMartin();
    void transitionTo(SSTVState newState);
    void sampleToScope(Real fmDemod, Real freq, Real isSyncTone, Real pllLocked, Real state);
    void sampleToSpectrum(Real fmDemod);
    void applyMode(); //!< Refresh runtime mode parameters from m_settings.m_sstvMode
    SSTVState getFirstDecodeState() const;

    /** Convert SDFT power-weighted centroid (Hz) to estimated true tone frequency (Hz).
     *  Inverts the non-linear centroid-vs-frequency response of the N=32, k=1..2 SDFT
     *  using SDFT_CALIB_CENTROIDS (9-point hardware-measured table, 100 Hz steps, 1500–2300 Hz).
     *  Tones below 1500 Hz are clamped to 1500 Hz (SSTV pixel range starts at 1500 Hz).
     *  Piecewise-linear interpolation between table entries.
     *  Applied by the SDFT path immediately after the moving-average filter so that
     *  the output 'freq' is a linearised frequency in Hz — the same quantity produced
     *  by the Hilbert path. */
    static float centroidToFreq(float centroid)
    {
        if (centroid <= SDFT_CALIB_CENTROIDS[0])
            return SDFT_CALIB_TRUE_MIN;
        if (centroid >= SDFT_CALIB_CENTROIDS[SDFT_CALIB_N - 1])
            return SDFT_CALIB_TRUE_MIN + float(SDFT_CALIB_N - 1) * SDFT_CALIB_TRUE_STEP;
        for (int i = 1; i < SDFT_CALIB_N; i++)
        {
            if (centroid <= SDFT_CALIB_CENTROIDS[i])
            {
                const float t = (centroid - SDFT_CALIB_CENTROIDS[i - 1]) /
                                (SDFT_CALIB_CENTROIDS[i] - SDFT_CALIB_CENTROIDS[i - 1]);
                return SDFT_CALIB_TRUE_MIN + (float(i - 1) + t) * SDFT_CALIB_TRUE_STEP;
            }
        }
        return SDFT_CALIB_TRUE_MIN + float(SDFT_CALIB_N - 1) * SDFT_CALIB_TRUE_STEP;
    }

    /** Convert true instantaneous frequency (Hz) to pixel luminance value [0..255].
     *  Uses a simple linear map [SSTVDEMOD_BLACK_FREQ, SSTVDEMOD_WHITE_FREQ] → [0, 255].
     *  Used by both paths: the Hilbert path (unbiased IF estimator) and the SDFT path
     *  (after centroidToFreq() has linearised the centroid). */
    static int freqToPixelDirect(float freq) {
        const float v = (freq - SSTVDEMOD_BLACK_FREQ) * 255.0f /
                        (SSTVDEMOD_WHITE_FREQ - SSTVDEMOD_BLACK_FREQ);
        return static_cast<int>(qBound(0.0f, v, 255.0f));
    }
};

#endif // INCLUDE_SSTVDEMODSINK_H
