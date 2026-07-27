///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
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

#ifndef INCLUDE_METEORDEMODSINK_H
#define INCLUDE_METEORDEMODSINK_H

#include <algorithm>
#include <cmath>
#include <vector>

#include <QDateTime>

#include "dsp/channelsamplesink.h"
#include "dsp/interpolator.h"
#include "dsp/nco.h"
#include "util/message.h"

#include "meteorsettings.h"
#include "meteorblobdetector.h"

class ChannelAPI;
class MessageQueue;
class SpectrumVis;
class FFTEngine;

class MeteorDemodSink : public ChannelSampleSink {
public:
    // Tunables for the 2D blob detector and the shared spectral front-end. The detection
    // algorithm itself lives in MeteorBlobDetector (an exact port of the offline reference
    // in test/detector2d/blobs.py); these knobs are harness-overridable and the user-facing
    // settings (sensitivity) mirror into them.
    struct DetectorTunables
    {
        struct SpectralTunables
        {
            double m_spectralFrameDurationS = 0.125;
            double m_spectralHopFraction = 0.25;
        } m_spectral;
        // Knobs for the 2D blob detector (mirror MeteorBlobDetector::Config); binHz/dt/fMin
        // are resolved from the channel at configure time, not stored here.
        struct BlobDetectorTunables
        {
            double m_seedDb = 11.0;
            double m_growDb = 6.5;
            double m_floorOffsetDb = 0.0;      // 0: median floor matches offline with no offset
            int    m_minPix = 3;
            int    m_closeFreqBins = 3;
            double m_linkGapSeconds = 0.12;    // max temporal gap when linking fragments
            double m_linkMaxDriftHzPerS = 300.0;
            double m_linkTolHz = 40.0;
            double m_trimKeepTime = 0.97;      // energy kept along time (loose: preserve tails)
            double m_trimKeepFreq = 0.90;      // energy kept along frequency (tight: hug the core)
            double m_occLimit = 0.5;
            double m_minPeakExcessDb = 16.0;       // "red blob" gate: reject faint spread noise
            double m_sweepMinLinR2 = 0.9;          // reject linear drifting tracks (satellite sweeps)
            double m_sweepMinAbsSlopeHzPerS = 20.0;
            double m_sweepMinDurationS = 0.4;
            // Weak-sweep EXTENSION: attach faint below-seed linear segments that continue an
            // already-detected sweep's drift line (lengthens real sweeps; cannot add rows). No
            // faint-only discovery (calibration: inseparable from noise).
            bool   m_weakSweepExtend = true;
            double m_faintSeedDb = 9.0;
            int    m_segMinCols = 4;
            int    m_segMinPix = 4;
            double m_segMinR2 = 0.8;
            double m_segSlopeMinHzPerS = 20.0;
            double m_segSlopeMaxHzPerS = 1000.0;
            double m_sweepLinkTolHz = 70.0;        // consolidation: freq tolerance on the drift line
            // Max time gap when stitching sweep fragments/segments along the drift line. Generous
            // because the line's frequency prediction is a strong discriminator — a collinear
            // segment seconds later still belongs to the same pass.
            double m_sweepLinkMaxGapS = 6.0;
            // Collinear-continuation merge: two consolidated sweeps that continue the same
            // pass (the fast, faint mid-pass of the Doppler S-curve can be invisible for
            // longer than the link gap) are merged instead of reported twice.
            double m_sweepMergeMaxGapS = 30.0;
            // Dash-walk box extension: the faint leading/trailing parts of a pass are dashed
            // fragments too short for shape gates; walk the sweep's fitted drift line outward
            // through chained on-line dashes (each step must find one within the step gap).
            double m_dashWalkStepGapS = 1.5;
            double m_dashWalkMaxS = 15.0;
            double m_scoreThreshold = 145.0;   // USER SENSITIVITY KNOB (integrated excess dB)
            // Stride scheduling (low latency): rolling window re-analysed every stride; a blob
            // is emitted ~finalMargin+stride (~1 s) after it ends. Window must stay much longer
            // than the longest meteor (median floor) — also bounds the max intact duration.
            double m_windowSeconds = 20.0;
            double m_minWindowSeconds = 10.0;
            double m_emitStrideS = 0.5;
            double m_finalMarginS = 0.5;
        } m_blob;
        struct DuplicateTunables
        {
            double m_duplicateFrequencyOverlapFraction = 0.65;
        } m_duplicate;
    };

    struct DetectionRecord
    {
        bool m_valid = false;
        QDateTime m_dateTimeUtc;
        QDateTime m_displayDateTimeUtc;
        quint64 m_startSample = 0;
        quint64 m_endSample = 0;
        bool m_hasDisplaySamples = false;
        quint64 m_displayStartSample = 0;
        quint64 m_displayEndSample = 0;
        double m_peakPower = 0.0;
        double m_backgroundPower = 1e-20;
        double m_totalPower = 0.0;
        double m_durationS = 0.0;
        double m_displayDurationS = 0.0;
        double m_centerFrequency = 0.0;
        double m_frequencySpan = 0.0;
        double m_frequencyDrift = 0.0;
        int m_sampleRate = 0;
        bool m_truncated = false;
    };

    class MsgMeteorDetected : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const DetectionRecord& getDetection() const { return m_detection; }
        const QDateTime& getDateTimeUtc() const { return m_detection.m_dateTimeUtc; }
        const QDateTime& getDisplayDateTimeUtc() const { return m_detection.m_displayDateTimeUtc; }
        double getPeakAmplitude() const { return std::sqrt(std::max(0.0, m_detection.m_peakPower)); }
        double getPeakPowerDB() const { return 10.0 * std::log10(std::max(1e-20, m_detection.m_peakPower)); }
        double getBackgroundPowerDB() const { return 10.0 * std::log10(std::max(1e-20, m_detection.m_backgroundPower)); }
        double getTotalPowerDB() const { return 10.0 * std::log10(std::max(1e-20, m_detection.m_totalPower)); }
        double getDurationS() const { return m_detection.m_durationS; }
        double getDisplayDurationS() const { return m_detection.m_displayDurationS; }
        double getCenterFrequency() const { return m_detection.m_centerFrequency; }
        double getFrequencySpan() const { return m_detection.m_frequencySpan; }
        double getFrequencyDrift() const { return m_detection.m_frequencyDrift; }
        int getSampleRate() const { return m_detection.m_sampleRate; }
        quint64 getStartSample() const { return m_detection.m_startSample; }
        quint64 getEndSample() const { return m_detection.m_endSample; }
        quint64 getDisplayStartSample() const { return m_detection.m_displayStartSample; }
        quint64 getDisplayEndSample() const { return m_detection.m_displayEndSample; }
        bool getTruncated() const { return m_detection.m_truncated; }

        static MsgMeteorDetected* create(const DetectionRecord& detection) {
            return new MsgMeteorDetected(detection);
        }

    private:
        DetectionRecord m_detection;

        explicit MsgMeteorDetected(const DetectionRecord& detection) :
            Message(),
            m_detection(detection)
        {}
    };

    class MsgSatelliteDetected : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const DetectionRecord& getDetection() const { return m_detection; }

        static MsgSatelliteDetected* create(const DetectionRecord& detection) {
            return new MsgSatelliteDetected(detection);
        }

    private:
        DetectionRecord m_detection;

        explicit MsgSatelliteDetected(const DetectionRecord& detection) :
            Message(),
            m_detection(detection)
        {}
    };

    class MsgMeteorDataCollected : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QDateTime& getDateTimeUtc() const { return m_dateTimeUtc; }

        static MsgMeteorDataCollected* create(const QDateTime& dateTimeUtc)
        {
            return new MsgMeteorDataCollected(dateTimeUtc);
        }

    private:
        QDateTime m_dateTimeUtc;

        MsgMeteorDataCollected(const QDateTime& dateTimeUtc) :
            Message(),
            m_dateTimeUtc(dateTimeUtc)
        {}
    };

    MeteorDemodSink();
    ~MeteorDemodSink();

    virtual void feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end);
    bool flushPendingPulse(bool finalizeSweeps);
    bool overlapsRecentDetection(
        quint64 startSample,
        quint64 endSample,
        double lowFrequency,
        double highFrequency) const;

    void setSpectrumSink(SpectrumVis* spectrumSink) { m_spectrumSink = spectrumSink; }
    void setSecondarySpectrumSink(SpectrumVis* spectrumSink) { m_secondarySpectrumSink = spectrumSink; }
    void setMessageQueueToGUI(MessageQueue *messageQueue) { m_messageQueueToGUI = messageQueue; }
    void setDetectorTunables(const DetectorTunables& tunables);
    const DetectorTunables& getDetectorTunables() const { return m_detectorTunables; }
    void setChannel(ChannelAPI *channel) { m_channel = channel; }
    void applyChannelSettings(int channelSampleRate, int channelFrequencyOffset, bool force = false);
    void applySettings(const MeteorSettings& settings, const QStringList& settingsKeys, bool force = false);
    int getOutputSampleRate() const { return m_settings.m_channelSampleRate; }

private:
    struct PulseReport : DetectionRecord {
        double m_reportFrequencySpan = 0.0;
        double m_duplicateFrequencySpan = 0.0;
    };

    struct DetectionRange {
        quint64 m_startSample;
        quint64 m_endSample;
        double m_lowFrequency;
        double m_highFrequency;
    };

    struct DisplayTimeAnchor {
        quint64 m_sampleCounter;
        QDateTime m_dateTimeUtc;
    };

    // Consolidates the detector's sweep fragments (which arrive across chunks) into one
    // Doppler-sweep detection by stitching collinear fragments along their drift line.
    struct ActiveSweep {
        double m_t0 = 0.0, m_t1 = 0.0;             // seconds (stream-relative)
        double m_f0 = 0.0, m_f1 = 0.0;             // Hz bounding box
        quint64 m_startSample = 0, m_endSample = 0;
        double m_peakPower = 0.0, m_totalPower = 0.0, m_backgroundSum = 0.0;
        int m_fragCount = 0;
        double m_sumT = 0.0, m_sumF = 0.0, m_sumTT = 0.0, m_sumTF = 0.0;  // running line fit
        int m_nPts = 0;
        double m_lastCentreT = 0.0, m_lastCentreF = 0.0, m_lastSlope = 0.0;
        quint64 m_lastSample = 0;
    };

    SpectrumVis* m_spectrumSink;
    SpectrumVis* m_secondarySpectrumSink;
    MessageQueue *m_messageQueueToGUI;
    DetectorTunables m_detectorTunables;
    MeteorSettings m_settings;
    ChannelAPI *m_channel;

    int m_channelSampleRate;
    int m_channelFrequencyOffset;
    NCO m_nco;
    Interpolator m_interpolator;
    Real m_interpolatorDistance;
    Real m_interpolatorDistanceRemain;

    int m_spectrumBufferSize;
    ComplexVector m_spectrumBuffer;
    ComplexVector m_detectionSampleRing;
    ComplexVector m_spectralFrameBuffer;
    std::vector<DetectionRange> m_recentDetectionRanges;
    int m_spectralFrameSize;
    int m_spectralHopSize;
    int m_detectionSampleRingStart;
    int m_detectionSampleRingCount;
    quint64 m_detectionSampleRingStartSample;
    quint64 m_sampleCounter;
    std::vector<DisplayTimeAnchor> m_displayTimeAnchors;
    quint64 m_nextDisplayTimeAnchorSample;
    quint64 m_nextDataMarkerSample;
    QDateTime m_streamStartDateTimeUtc;

    // 2D blob detector front-end: zero-padded FFT feeding MeteorBlobDetector, which owns
    // segmentation, the median noise floor and acceptance.
    MeteorBlobDetector m_blobDetector;
    FFTEngine *m_blobFFT;
    int m_blobFFTSize;                              // zero-padded FFT size (nfft)
    std::vector<Real> m_blobWindow;                // Hann over the frame (pre-pad)
    std::vector<double> m_blobBinPower;            // padded magnitude^2 per bin
    std::vector<ActiveSweep> m_activeSweeps;
    std::vector<MeteorBlobDetector::SweepSegment> m_faintDashes;   // relaxed segments for dash-walk

    void processOneSample(Complex& ci, bool realSample = true);
    void processSpectralSample(const Complex& sample);
    void configureBlobDetector();
    void processBlobDetectorFrame(quint64 frameStartSample);
    void emitBlobDetection(const MeteorBlobDetector::Blob& blob);
    void drainBlobDetections();
    void addSweepFragment(const MeteorBlobDetector::Blob& blob);
    void extendSweepWithSegment(const MeteorBlobDetector::SweepSegment& seg);
    void mergeSweepFragment(ActiveSweep& sweep, const MeteorBlobDetector::Blob& blob);
    bool absorbedByActiveSweep(const MeteorBlobDetector::Blob& blob);
    void finalizeStaleSweeps(quint64 currentSample, bool force);
    void retireSweepAt(size_t index);
    bool mergeIntoContinuationSweep(size_t sweepIndex);
    void walkSweepAlongDashes(ActiveSweep& sweep);
    void emitConsolidatedSweep(const ActiveSweep& sweep);
    void rememberDetection(quint64 startSample, quint64 endSample, double centerFrequency, double frequencySpan);
    void pruneRecentDetections();
    void appendDetectionSample(const Complex& sample);
    bool copyDetectionSamples(quint64 startSample, quint64 endSample, ComplexVector& samples) const;
    double estimatePulseTotalPower(quint64 startSample, quint64 endSample, double backgroundPower) const;
    void emitDetectionReport(const PulseReport& report, const char *source);
    void emitDetectionReport(
        const PulseReport& report,
        const char *source,
        quint64 duplicateStartSample,
        quint64 duplicateEndSample);
    QDateTime sampleCounterToDateTimeUtc(quint64 sampleCounter) const;
    QDateTime sampleCounterToDisplayDateTimeUtc(quint64 sampleCounter) const;
    void recordDisplayTimeAnchor();
    void emitDataCollectionMarker();
    void configureInterpolator();
    void configureDetectionHistory();
    void configureSpectralDetector();
    void resizeSpectrumBuffer();
    void resetDetector();
    void feedSpectrum(const Complex& sample);
};

#endif // INCLUDE_METEORDEMODSINK_H
