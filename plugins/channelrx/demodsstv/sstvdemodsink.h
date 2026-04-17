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

#include <cmath>

#include "dsp/channelsamplesink.h"
#include "dsp/phasediscri.h"
#include "dsp/nco.h"
#include "dsp/interpolator.h"
#include "util/movingaverage.h"
#include "util/messagequeue.h"

#include "sstvdemodsettings.h"

// Internal audio sample rate for SSTV processing (Hz)
#define SSTVDEMOD_CHANNEL_SAMPLE_RATE  48000

// PD120 image dimensions
#define SSTVDEMOD_IMAGE_WIDTH   640
#define SSTVDEMOD_IMAGE_HEIGHT  496   // 248 pairs of scan lines

// SSTV tone frequencies (Hz)
#define SSTVDEMOD_SYNC_FREQ       1200.0f   // Sync pulse frequency
#define SSTVDEMOD_BLACK_FREQ      1500.0f   // Black level / start tone
#define SSTVDEMOD_WHITE_FREQ      2300.0f   // White level
// Sync threshold: with N_SDFT=64 and bins k=1..3 the SDFT centroid for a 1200 Hz sync
// tone has a steady-state mean of ~1316 Hz.  A 40-sample MA (SDFT_FREQ_MA_LEN) is
// applied to suppress the centroid oscillation that occurs at twice the input frequency,
// leaving a stable ~1316 Hz reading.  1420 Hz sits in the gap between that (~1316 Hz)
// and the 1500 Hz porch, giving ~84 Hz margin on each side.
// Note: N_SDFT=32 is not viable — the 1200 Hz sync tone maps to k₀=0.8 which is below
// k_min=1, so the sync centroid reads above the porch and the FSM cannot detect sync.
// The minimum viable window size is N_SDFT=42 (k₀_sync=1.05 > 1.0).
#define SSTVDEMOD_SYNC_THRESHOLD  1420.0f   // Below this = sync, above = pixel data

// PD120 timing in milliseconds
#define SSTVDEMOD_SYNC_MS         20.0f     // Scan-line sync pulse duration
#define SSTVDEMOD_PORCH_MS        2.08f     // Porch duration after sync (at black level, 1500 Hz)
// Measured pixel clock: 190 µs/pixel (empirically confirmed; gives ~96 s/frame for PD120).
#define SSTVDEMOD_PIXEL_TIME_MS   0.190f

// Timing in samples at SSTVDEMOD_CHANNEL_SAMPLE_RATE
#define SSTVDEMOD_SYNC_SAMPLES      ((int)(SSTVDEMOD_SYNC_MS * SSTVDEMOD_CHANNEL_SAMPLE_RATE / 1000.0f))
#define SSTVDEMOD_PORCH_SAMPLES     ((int)(SSTVDEMOD_PORCH_MS * SSTVDEMOD_CHANNEL_SAMPLE_RATE / 1000.0f))
#define SSTVDEMOD_SAMPLES_PER_PIXEL (SSTVDEMOD_PIXEL_TIME_MS * SSTVDEMOD_CHANNEL_SAMPLE_RATE / 1000.0f)

// Scan-line sync pulse duration bounds (samples).
// VIS start/stop bits and VIS "0" data bits are all 30 ms (1440 samples at 48 kHz),
// which is longer than a real 20 ms scan-line sync.  Rejecting pulses outside
// [MIN, MAX] prevents the VIS code from being decoded as image data.
#define SSTVDEMOD_SYNC_SAMPLES_MIN  ((int)(SSTVDEMOD_SYNC_SAMPLES * 0.75f))   // 720  – 15 ms lower bound
#define SSTVDEMOD_SYNC_SAMPLES_MAX  ((int)(SSTVDEMOD_SYNC_SAMPLES * 1.25f))   // 1200 – 25 ms upper bound (< 30 ms VIS bits)




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
        DECODING_Y_EVEN    //!< Decoding Y (luminance) for even scan line (pixels 0..639)
    };

    SSTVDemodSink();
    ~SSTVDemodSink();

    virtual void feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end);

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
    // N_SDFT=64 reduces horizontal pixel blur to ~11 pixels (vs ~14 for N=128).
    // The 1.33 ms window fits within the 2.08 ms porch period.
    //
    // SDFT oscillation and the need for the MA:
    //   Because fmDemod = cos(2π·f_tone·t) is a real-valued signal, each SDFT
    //   bin receives contributions from both positive (k₀=f_tone·N/Fs) and
    //   negative (k₀_neg=N−k₀) frequency components.  The beat between these
    //   two components makes every bin magnitude oscillate at 2·f_tone, so the
    //   power-weighted centroid also oscillates at that rate.  For the 1200 Hz
    //   sync tone the period is Fs/(2·1200)=20 samples.  Applying a
    //   SDFT_FREQ_MA_LEN=40-sample MA (exactly two 1200 Hz periods) completely
    //   cancels the sync-tone oscillation, yielding a stable ~1316 Hz reading
    //   that sits comfortably below the 1420 Hz threshold.
    //
    // Bins k=1–3 (750–2250 Hz) span the full SSTV tone range 1200–2300 Hz.
    // Using three bins instead of four reduces the neutral-chroma bias:
    //   k=1..4: 1900 Hz (neutral chroma) reads ≈1918 Hz → pixel 142 (bias +14)
    //           → white appears purple (R=B clipped, G reduced)
    //   k=1..3: 1900 Hz reads ≈1858 Hz; piecewise calibration maps this
    //           exactly to pixel 128, giving neutral grey at true neutral chroma.
    //
    // Truncated-range bias and piecewise calibration:
    //   The centroid is only unbiased when summed over all N/2 bins.  Using
    //   k=1..3 introduces a systematic shift in the SSTV pixel range.  Rather
    //   than a single linear scale from black (1500 Hz) to white (calibrated
    //   centroid), freqToPixel() uses two calibration points:
    //     • SDFT_MEAS_NEUTRAL_FREQ: measured centroid for a true 1900 Hz tone
    //       (= neutral chroma centre, should map to pixel 128).
    //     • SDFT_MEAS_WHITE_FREQ:   measured centroid for a true 2300 Hz tone
    //       (= full white, should map to pixel 255).
    //   A piecewise-linear map anchored at both points ensures neutral chroma
    //   decodes to exactly 128 and white to 255, eliminating the colour bias.
    //
    // The SDFT history is NOT reset at section transitions (Y_odd→Cr→Cb→Y_even).
    // Adjacent sections share the same 1500–2300 Hz frequency range, so the
    // ~11-pixel bleed-in from the previous section is mild and self-correcting.
    // Resetting to zero would force those pixels to 1200 Hz (below black level),
    // producing green/teal artefacts in the Cr/Cb sections — worse than the
    // natural contamination.  The SDFT is only fully cleared in resetDecoder().
    // -----------------------------------------------------------------------
    static constexpr int N_SDFT             = 64;  //!< Sliding DFT window length (samples); bin width = Fs/N = 750 Hz
    static constexpr int SDFT_FREQ_MA_LEN   = 40;  //!< Freq MA length = two periods of 1200 Hz sync tone; cancels centroid oscillation
    static constexpr int SDFT_K_STORE_MIN   = 1;   //!< Lowest stored bin  (k=1 → 750 Hz)
    static constexpr int SDFT_K_STORE_MAX   = 3;   //!< Highest stored bin (k=3 → 2250 Hz)
    static constexpr int SDFT_K_SUM_MIN     = 1;   //!< First bin in the moment sum (k=1 → 750 Hz, below sync 1200 Hz)
    static constexpr int SDFT_K_SUM_MAX     = 3;   //!< Last  bin in the moment sum (k=3 → 2250 Hz, above white 2300 Hz)
    static constexpr int SDFT_NUM_BINS      = SDFT_K_STORE_MAX - SDFT_K_STORE_MIN + 1; // 3

    // Piecewise calibration: two measured SDFT-centroid outputs used as anchor points.
    // Both values were obtained by simulation (N=64, k=1..3, MA=40, steady-state mean):
    //   • SDFT_MEAS_NEUTRAL_FREQ: centroid for a true 1900 Hz tone (neutral chroma = pixel 128)
    //   • SDFT_MEAS_WHITE_FREQ:   centroid for a true 2300 Hz tone (full white  = pixel 255)
    // freqToPixel() uses a two-segment linear map: [1500, NEUTRAL]→[0,128] and
    // [NEUTRAL, WHITE]→[128,255], ensuring both neutral chroma and white decode correctly.
    static constexpr float SDFT_MEAS_NEUTRAL_FREQ = 1858.0f; //!< Hz — SDFT centroid for true 1900 Hz neutral-chroma tone
    static constexpr float SDFT_MEAS_WHITE_FREQ   = 2245.0f; //!< Hz — SDFT centroid for true 2300 Hz white tone

    float   m_sdftBuf[N_SDFT];            //!< Circular ring buffer of fmDemod samples
    Complex m_sdftBins[SDFT_NUM_BINS];    //!< Running SDFT bins k = SDFT_K_STORE_MIN..SDFT_K_STORE_MAX
    Complex m_sdftTwiddle[SDFT_NUM_BINS]; //!< Twiddle factors e^{+j·2π·k/N} per bin
    int     m_sdftIdx;                    //!< Next write position in m_sdftBuf (0..N_SDFT−1)

    MovingAverageUtil<float, double, SDFT_FREQ_MA_LEN> m_freqMovAvg; //!< Post-SDFT freq MA to suppress sync-tone centroid oscillation

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
    // Step 3 — 1st-order IIR low-pass filter (cutoff 3000 Hz):
    //   Suppresses wideband FM-demodulator noise above the SSTV tone range.
    //   The residual ±18 Hz elliptical beat (at 2400 Hz) is further attenuated
    //   to < 14 Hz by this filter.  Group delay ≈ 2.1 samples ≈ 0.2 pixels.
    //     f[n] = α·f_raw + (1−α)·f[n−1],   α = HILBERT_LPF_ALPHA
    //
    // Pixel mapping uses freqToPixelDirect(): a simple linear map
    //   [SSTVDEMOD_BLACK_FREQ, SSTVDEMOD_WHITE_FREQ] → [0, 255]
    // with no piecewise calibration because the Hilbert estimator is unbiased.
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

    // 1st-order IIR LPF coefficient: alpha = 1 - exp(-2*pi*3000/48000).
    // y(n) = HILBERT_LPF_ALPHA*x(n) + (1-HILBERT_LPF_ALPHA)*y(n-1)
    // Time constant = 2.1 samples = 0.2 pixel blur at 190 us/pixel.
    static constexpr float HILBERT_LPF_ALPHA  = 0.32476809f; //!< IIR alpha for 3000 Hz LPF at 48 kHz

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
    float   m_hilbertFreqIIR;          //!< IIR LPF state for the Hilbert IF output

    bool    m_useHilbert;              //!< true = Hilbert IF path; false = SDFT spectral-centroid path (default)

    // SSTV decoder state
    SSTVState m_state;
    int m_stateSampleCount;   //!< Number of samples spent in current state

    // Pixel sampling state (sub-pixel accumulation)
    int m_pixelIndex;          //!< Index of current pixel being accumulated (within section)
    float m_pixelAccum;        //!< Accumulated frequency value for current pixel
    float m_pixelSamplePos;    //!< Fractional sample position within current pixel period
    int m_pixelSampleCount;    //!< Actual number of samples accumulated in current pixel (denominator for average)

    // PD120 line buffer: one block = odd + even scan lines
    float m_yOdd[SSTVDEMOD_IMAGE_WIDTH];        //!< Y (luminance) values for odd line
    float m_cr[SSTVDEMOD_IMAGE_WIDTH];          //!< Cr (chroma-red) values shared by both lines
    float m_cb[SSTVDEMOD_IMAGE_WIDTH];          //!< Cb (chroma-blue) values shared by both lines
    float m_yEven[SSTVDEMOD_IMAGE_WIDTH];       //!< Y (luminance) values for even line

    int m_lineIndex;    //!< Current block index (0..247, each block = 2 scan lines)

    void processOneSample(Complex &ci);
    void decodePixelSample(float freq, float *buf, int width, SSTVState nextState);
    void commitBlock();
    void transitionTo(SSTVState newState);

    /** Convert SDFT centroid frequency (Hz) to pixel luminance value [0..255].
     *  Uses a piecewise-linear map anchored at two measured SDFT calibration points:
     *    [SSTVDEMOD_BLACK_FREQ, SDFT_MEAS_NEUTRAL_FREQ] → [0, 128]  (lower segment)
     *    [SDFT_MEAS_NEUTRAL_FREQ, SDFT_MEAS_WHITE_FREQ] → [128, 255] (upper segment)
     *  This ensures neutral chroma (1900 Hz → centroid ≈1858 Hz) decodes to exactly
     *  pixel 128 (no colour bias) and white (2300 Hz → centroid ≈2245 Hz) to 255.
     *  Used by the SDFT path (m_useHilbert = false). */
    static int freqToPixel(float freq) {
        float v;
        if (freq <= SDFT_MEAS_NEUTRAL_FREQ) {
            v = (freq - SSTVDEMOD_BLACK_FREQ) * 128.0f / (SDFT_MEAS_NEUTRAL_FREQ - SSTVDEMOD_BLACK_FREQ);
        } else {
            v = 128.0f + (freq - SDFT_MEAS_NEUTRAL_FREQ) * 127.0f / (SDFT_MEAS_WHITE_FREQ - SDFT_MEAS_NEUTRAL_FREQ);
        }
        return static_cast<int>(qBound(0.0f, v, 255.0f));
    }

    /** Convert true instantaneous frequency (Hz) to pixel luminance value [0..255].
     *  Uses a simple linear map [SSTVDEMOD_BLACK_FREQ, SSTVDEMOD_WHITE_FREQ] → [0, 255]
     *  with no piecewise calibration, because the Hilbert IF estimator is unbiased.
     *  Used by the Hilbert path (m_useHilbert = true). */
    static int freqToPixelDirect(float freq) {
        const float v = (freq - SSTVDEMOD_BLACK_FREQ) * 255.0f /
                        (SSTVDEMOD_WHITE_FREQ - SSTVDEMOD_BLACK_FREQ);
        return static_cast<int>(qBound(0.0f, v, 255.0f));
    }
};

#endif // INCLUDE_SSTVDEMODSINK_H
