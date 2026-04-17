///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2024 Jon Beniston, M7RCE <jon@beniston.com>                     //
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

#ifndef PLUGINS_CHANNELTX_MODSSTV_SSTVMODSOURCE_H_
#define PLUGINS_CHANNELTX_MODSSTV_SSTVMODSOURCE_H_

#include <QObject>
#include <QImage>
#include <QRecursiveMutex>

#include "dsp/channelsamplesource.h"
#include "dsp/nco.h"
#include "util/movingaverage.h"

#include "sstvmodsettings.h"

class ChannelAPI;

// -----------------------------------------------------------------------
// PD120 timing constants (all at 48000 Hz sample rate)
// -----------------------------------------------------------------------
// PD120: 640×496 image (248 pairs of scan lines).  Each pair:
//   sync   20ms   1200 Hz
//   porch   2.08ms 1500 Hz
//   Y_odd  640 × 190 µs
//   Cr     640 × 190 µs
//   Cb     640 × 190 µs
//   Y_even 640 × 190 µs
//
// VIS preamble (before first line):
//   leader1  300ms 1900 Hz
//   break     10ms 1200 Hz
//   leader2  300ms 1900 Hz
//   VIS start 30ms 1200 Hz
//   7 data bits + 1 parity bit, each 30ms at 1100(=1) or 1300(=0) Hz
//   VIS stop  30ms 1200 Hz
//
// Pixel value → tone frequency: f = 1500 + (v/255) × 800  Hz
// -----------------------------------------------------------------------

static constexpr int    SSTV_SAMPLE_RATE    = 48000;
static constexpr int    SSTV_IMAGE_WIDTH    = 640;
static constexpr int    SSTV_IMAGE_HEIGHT   = 496;  // 248 pairs
static constexpr int    SSTV_LINE_PAIRS     = 248;

// Tone frequencies (Hz)
static constexpr float  SSTV_SYNC_FREQ     = 1200.0f;
static constexpr float  SSTV_PORCH_FREQ    = 1500.0f;  // black level
static constexpr float  SSTV_LEADER_FREQ   = 1900.0f;
static constexpr float  SSTV_VIS_ZERO_FREQ = 1300.0f;  // VIS bit 0
static constexpr float  SSTV_VIS_ONE_FREQ  = 1100.0f;  // VIS bit 1
static constexpr float  SSTV_BLACK_FREQ    = 1500.0f;
static constexpr float  SSTV_WHITE_FREQ    = 2300.0f;

// Durations in samples at 48000 Hz
static constexpr int    SSTV_LEADER_SAMPLES  = (int)(0.300f * SSTV_SAMPLE_RATE);  // 14400
static constexpr int    SSTV_BREAK_SAMPLES   = (int)(0.010f * SSTV_SAMPLE_RATE);  // 480
static constexpr int    SSTV_VIS_BIT_SAMPLES = (int)(0.030f * SSTV_SAMPLE_RATE);  // 1440
static constexpr int    SSTV_SYNC_SAMPLES    = (int)(0.020f * SSTV_SAMPLE_RATE);  // 960
static constexpr int    SSTV_PORCH_SAMPLES   = (int)(0.00208f * SSTV_SAMPLE_RATE); // 99
static constexpr float  SSTV_PIXEL_SAMPLES   = 0.190f * SSTV_SAMPLE_RATE / 1000.0f; // 9.12

// VIS code for PD120 = 95 = 0x5F
// 7 data bits (LSB first) + 1 even-parity bit
static constexpr uint8_t SSTV_VIS_CODE = 95;

/** PD120 SSTV modulator source.
 *
 * Generates complex baseband I/Q samples at 48000 Hz.  The PD120 signal
 * consists of a preamble (VIS header) followed by 248 scan-line pairs, each
 * containing a sync pulse, porch, Y_odd, Cr, Cb and Y_even pixel sections.
 *
 * Modulation can be FM, USB or LSB:
 *  - FM:  frequency of the complex carrier tracks the SSTV sub-carrier tone.
 *  - USB: the complex analytic representation of the SSTV tone is used directly.
 *  - LSB: the conjugate of the analytic SSTV tone is used.
 */
class SSTVModSource : public QObject, public ChannelSampleSource
{
    Q_OBJECT

public:
    explicit SSTVModSource(QObject *parent = nullptr);
    ~SSTVModSource() final;

    void pull(SampleVector::iterator begin, unsigned int nbSamples) final;
    void pullOne(Sample& sample) final;
    void prefetch(unsigned int nbSamples) final;

    /** Load an image (pre-scaled to 640×496 YCbCr internally). */
    void loadImage(const QImage& image);

    /** Start transmitting from the beginning of the image. */
    void startTransmit();

    /** Stop transmitting immediately (return to idle). */
    void stopTransmit();

    bool isTransmitting() const { return m_state != State::IDLE; }
    double getMagSq() const { return m_magsq; }

    void setChannel(ChannelAPI *channel) { m_channel = channel; }
    void applySettings(const QStringList& settingsKeys, const SSTVModSettings& settings, bool force = false);
    void applyChannelSettings(int channelSampleRate, int channelFrequencyOffset, bool force = false);

signals:
    /** Emitted when transmission completes (all lines sent). */
    void transmitComplete();

private:
    enum class State {
        IDLE,
        LEADER1,
        BREAK,
        LEADER2,
        VIS_START,
        VIS_BITS,
        VIS_STOP,
        LINE_SYNC,
        LINE_PORCH,
        LINE_Y_ODD,
        LINE_CR,
        LINE_CB,
        LINE_Y_EVEN,
        DONE
    };

    SSTVModSettings m_settings;
    int m_channelSampleRate = 48000;
    int m_channelFrequencyOffset = 0;
    ChannelAPI *m_channel = nullptr;

    NCO  m_carrierNco;         //!< Shifts signal to channel frequency offset

    // FM phasor (only used in FM mode)
    float m_fmPhasor = 0.0f;

    // SSB/USB/LSB phasor (used in USB/LSB modes) - tracks current tone
    float m_tonePhasor = 0.0f;

    // Current SSTV tone frequency (Hz) – drives both FM and SSB paths
    float m_currentFreq = SSTV_PORCH_FREQ;

    // Signal-level tracking for GUI meter
    double m_magsq = 0.0;
    MovingAverageUtil<double, double, 16> m_movingAverage;

    // -----------------------------------------------------------------------
    // Encoder state
    // -----------------------------------------------------------------------
    State m_state = State::IDLE;
    int   m_stateSamples = 0;     //!< Samples remaining in current state
    int   m_linePair = 0;         //!< Current line-pair index (0..247)
    int   m_pixelIndex = 0;       //!< Current pixel column (0..639)
    float m_pixelFrac = 0.0f;     //!< Fractional sample accumulator for sub-integer pixel width

    // VIS bit transmission
    int   m_visBit = 0;           //!< Which bit we're transmitting (0..7; 8 = stop)
    uint8_t m_visData = 0;        //!< VIS byte (with parity) to send

    // Image data stored as RGB888
    QByteArray m_imageData;   //!< width × height × 3 bytes (RGB)
    bool       m_imageValid = false;
    QRecursiveMutex m_mutex;

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------
    void advanceState();
    void enterState(State s, int samples);
    float pixelToFreq(int value) const;
    float getPixelFreqForColumn(int linePair, State section, int col) const;
    Complex generateSample();
};

#endif // PLUGINS_CHANNELTX_MODSSTV_SSTVMODSOURCE_H_
