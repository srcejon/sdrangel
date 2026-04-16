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

#ifndef INCLUDE_SSTVDEMODSINK_H
#define INCLUDE_SSTVDEMODSINK_H

#include <QImage>

#include "dsp/channelsamplesink.h"
#include "dsp/filterrc.h"
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
#define SSTVDEMOD_SYNC_THRESHOLD  1300.0f   // Below this = sync, above = pixel data

// PD120 timing in milliseconds
#define SSTVDEMOD_SYNC_MS         20.0f     // Sync pulse duration
#define SSTVDEMOD_PORCH_MS        2.08f     // Porch duration after sync
// Pixel time = (120000ms - 248*(20+2.08)ms) / (248*1920 pixels) = 0.24052 ms/pixel
#define SSTVDEMOD_PIXEL_TIME_MS   0.24052f

// Timing in samples at SSTVDEMOD_CHANNEL_SAMPLE_RATE
#define SSTVDEMOD_SYNC_SAMPLES    ((int)(SSTVDEMOD_SYNC_MS * SSTVDEMOD_CHANNEL_SAMPLE_RATE / 1000.0f))
#define SSTVDEMOD_PORCH_SAMPLES   ((int)(SSTVDEMOD_PORCH_MS * SSTVDEMOD_CHANNEL_SAMPLE_RATE / 1000.0f))
#define SSTVDEMOD_SAMPLES_PER_PIXEL (SSTVDEMOD_PIXEL_TIME_MS * SSTVDEMOD_CHANNEL_SAMPLE_RATE / 1000.0f)

// Minimum sync pulse duration to be considered valid (75% of expected)
#define SSTVDEMOD_SYNC_SAMPLES_MIN  ((int)(SSTVDEMOD_SYNC_SAMPLES * 0.75f))

// Low-pass filter cut-off applied to the instantaneous-frequency output (Hz).
// The filter suppresses noise from the RF FM demodulator while still passing
// pixel-rate transitions.  At 3000 Hz the step response reaches 99 % of its
// final value within 12 samples (≈ 1 pixel period), so horizontal blurring is
// less than half a pixel.  Noise above 3 kHz is attenuated: −10 dB at 10 kHz,
// −14 dB at 24 kHz (Nyquist).
#define SSTVDEMOD_LPF_CUTOFF_HZ   3000.0f

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
    PhaseDiscriminators m_audioPhaDiscri;   //!< Audio FM discriminator (Stage 2)
    LowPassFilterRC m_freqLpFilter;         //!< Low-pass filter on instantaneous-frequency output

    // Hilbert FIR ring buffer for forming the analytic signal from fmDemod.
    // 63-tap Type-III FIR with Hamming window, delay = 31 samples.
    //
    // Ideal coefficients h[k] = 2/(k·π) for odd k = 1,3,...,31 are multiplied
    // by the Hamming window evaluated at tap offset k from centre:
    //   w(k) = 0.54 + 0.46·cos(π·k/31)
    //
    // The Hamming window eliminates Gibbs-phenomenon amplitude ripple that would
    // otherwise arise from truncating the infinite ideal Hilbert FIR.  Without
    // windowing the truncated 63-tap FIR has ~10% amplitude error at 1500 Hz
    // (user-verified: imaginary part was 90% of real), causing the instantaneous
    // frequency to oscillate 1358–1661 Hz for a constant 1500 Hz input.
    // With Hamming windowing the amplitude accuracy is ±0.4% across the whole
    // SSTV tone band (1200–2300 Hz), giving a per-sample frequency error of
    // only ±5 Hz — well below the 1300 Hz sync threshold.
    //
    // Sign convention: the code sums (get(31−k) − get(31+k)) for each odd k,
    // which equals −H{x}[n−31] ≈ sin(ω(n−31)).  Negating in the Complex
    // constructor gives z = e^{+jω(n−31)} (positive phase rotation).
    static constexpr float k_hilbert[16] = {
        0.63512f,  //!< 2/(1·π)  × w(1)
        0.20773f,  //!< 2/(3·π)  × w(3)
        0.11996f,  //!< 2/(5·π)  × w(5)
        0.08085f,  //!< 2/(7·π)  × w(7)
        0.05811f,  //!< 2/(9·π)  × w(9)
        0.04298f,  //!< 2/(11·π) × w(11)
        0.03209f,  //!< 2/(13·π) × w(13)
        0.02391f,  //!< 2/(15·π) × w(15)
        0.01761f,  //!< 2/(17·π) × w(17)
        0.01274f,  //!< 2/(19·π) × w(19)
        0.00899f,  //!< 2/(21·π) × w(21)
        0.00617f,  //!< 2/(23·π) × w(23)
        0.00414f,  //!< 2/(25·π) × w(25)
        0.00277f,  //!< 2/(27·π) × w(27)
        0.00196f,  //!< 2/(29·π) × w(29)
        0.00164f,  //!< 2/(31·π) × w(31)
    };

    Real m_hilbertBuf[63];  //!< Circular ring buffer holding 63 past fmDemod values
    int  m_hilbertIdx;      //!< Write index into m_hilbertBuf (0..62)

    // SSTV decoder state
    SSTVState m_state;
    int m_stateSampleCount;   //!< Number of samples spent in current state

    // Pixel sampling state (sub-pixel accumulation)
    int m_pixelIndex;          //!< Index of current pixel being accumulated (within section)
    float m_pixelAccum;        //!< Accumulated frequency value for current pixel
    float m_pixelSamplePos;    //!< Fractional sample position within current pixel period

    // PD120 line buffer: one block = odd + even scan lines
    float m_yOdd[SSTVDEMOD_IMAGE_WIDTH];        //!< Y (luminance) values for odd line
    float m_cr[SSTVDEMOD_IMAGE_WIDTH / 2];      //!< Cr (chroma-red) values shared by both lines
    float m_cb[SSTVDEMOD_IMAGE_WIDTH / 2];      //!< Cb (chroma-blue) values shared by both lines
    float m_yEven[SSTVDEMOD_IMAGE_WIDTH];       //!< Y (luminance) values for even line

    int m_lineIndex;    //!< Current block index (0..247, each block = 2 scan lines)

    void processOneSample(Complex &ci);
    void decodePixelSample(float freq, float *buf, int width, SSTVState nextState);
    void commitBlock();
    void transitionTo(SSTVState newState);

    /** Convert frequency (Hz) to pixel luminance value [0..255] */
    static int freqToPixel(float freq) {
        float v = (freq - SSTVDEMOD_BLACK_FREQ) * 255.0f / (SSTVDEMOD_WHITE_FREQ - SSTVDEMOD_BLACK_FREQ);
        if (v < 0.0f) { v = 0.0f; }
        if (v > 255.0f) { v = 255.0f; }
        return static_cast<int>(v);
    }
};

#endif // INCLUDE_SSTVDEMODSINK_H
