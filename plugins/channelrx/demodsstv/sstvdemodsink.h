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
#define SSTVDEMOD_SYNC_THRESHOLD  1300.0f   // Below this = sync, above = pixel data

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

    /** Convert frequency (Hz) to pixel value [0..255].
     *  Maps 1500 Hz (black) → 0 and 2300 Hz (white) → 255. */
    static int freqToPixel(float freq) {
        float v = (freq - SSTVDEMOD_BLACK_FREQ) * 255.0f / (SSTVDEMOD_WHITE_FREQ - SSTVDEMOD_BLACK_FREQ);
        if (v < 0.0f) { v = 0.0f; }
        if (v > 255.0f) { v = 255.0f; }
        return static_cast<int>(v);
    }
};

#endif // INCLUDE_SSTVDEMODSINK_H
