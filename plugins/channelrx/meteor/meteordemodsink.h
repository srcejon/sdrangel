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

#include <array>
#include <vector>

#include <QDateTime>

#include "dsp/channelsamplesink.h"
#include "dsp/firfilter.h"
#include "dsp/interpolator.h"
#include "dsp/nco.h"
#include "util/message.h"

#include "meteorsettings.h"

class MessageQueue;
class ScopeVis;
class SpectrumVis;
class FFTEngine;

class MeteorDemodSink : public ChannelSampleSink {
public:
    enum {
        m_scopeStreams = 5
    };

    class MsgMeteorDetected : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QDateTime& getDateTimeUtc() const { return m_dateTimeUtc; }
        const QDateTime& getDisplayDateTimeUtc() const { return m_displayDateTimeUtc; }
        double getPeakAmplitude() const { return m_peakAmplitude; }
        double getPeakPowerDB() const { return m_peakPowerDB; }
        double getBackgroundPowerDB() const { return m_backgroundPowerDB; }
        double getDurationS() const { return m_durationS; }
        double getDisplayDurationS() const { return m_displayDurationS; }
        double getCenterFrequency() const { return m_centerFrequency; }
        double getFrequencySpan() const { return m_frequencySpan; }
        double getFrequencyDrift() const { return m_frequencyDrift; }
        int getSampleRate() const { return m_sampleRate; }

        static MsgMeteorDetected* create(
            const QDateTime& dateTimeUtc,
            const QDateTime& displayDateTimeUtc,
            double peakAmplitude,
            double peakPowerDB,
            double backgroundPowerDB,
            double durationS,
            double displayDurationS,
            double centerFrequency,
            double frequencySpan,
            double frequencyDrift,
            int sampleRate)
        {
            return new MsgMeteorDetected(
                dateTimeUtc,
                displayDateTimeUtc,
                peakAmplitude,
                peakPowerDB,
                backgroundPowerDB,
                durationS,
                displayDurationS,
                centerFrequency,
                frequencySpan,
                frequencyDrift,
                sampleRate
            );
        }

    private:
        QDateTime m_dateTimeUtc;
        QDateTime m_displayDateTimeUtc;
        double m_peakAmplitude;
        double m_peakPowerDB;
        double m_backgroundPowerDB;
        double m_durationS;
        double m_displayDurationS;
        double m_centerFrequency;
        double m_frequencySpan;
        double m_frequencyDrift;
        int m_sampleRate;

        MsgMeteorDetected(
            const QDateTime& dateTimeUtc,
            const QDateTime& displayDateTimeUtc,
            double peakAmplitude,
            double peakPowerDB,
            double backgroundPowerDB,
            double durationS,
            double displayDurationS,
            double centerFrequency,
            double frequencySpan,
            double frequencyDrift,
            int sampleRate
        ) :
            Message(),
            m_dateTimeUtc(dateTimeUtc),
            m_displayDateTimeUtc(displayDateTimeUtc),
            m_peakAmplitude(peakAmplitude),
            m_peakPowerDB(peakPowerDB),
            m_backgroundPowerDB(backgroundPowerDB),
            m_durationS(durationS),
            m_displayDurationS(displayDurationS),
            m_centerFrequency(centerFrequency),
            m_frequencySpan(frequencySpan),
            m_frequencyDrift(frequencyDrift),
            m_sampleRate(sampleRate)
        {}
    };

    MeteorDemodSink();
    ~MeteorDemodSink();

    virtual void feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end);
    bool flushPendingPulse();

    void setScopeSink(ScopeVis* scopeSink) { m_scopeSink = scopeSink; }
    void setSpectrumSink(SpectrumVis* spectrumSink) { m_spectrumSink = spectrumSink; }
    void setMessageQueueToGUI(MessageQueue *messageQueue) { m_messageQueueToGUI = messageQueue; }
    void applyChannelSettings(int channelSampleRate, int channelFrequencyOffset, bool force = false);
    void applySettings(const MeteorSettings& settings, const QStringList& settingsKeys, bool force = false);
    int getOutputSampleRate() const { return m_settings.m_channelSampleRate; }

private:
    struct PulseReport {
        bool m_valid;
        QDateTime m_dateTimeUtc;
        quint64 m_startSample;
        quint64 m_endSample;
        double m_peakPower;
        double m_backgroundPower;
        double m_durationS;
        double m_centerFrequency;
        double m_frequencySpan;
        double m_frequencyDrift;

        PulseReport() :
            m_valid(false),
            m_startSample(0),
            m_endSample(0),
            m_peakPower(0.0),
            m_backgroundPower(1e-20),
            m_durationS(0.0),
            m_centerFrequency(0.0),
            m_frequencySpan(0.0),
            m_frequencyDrift(0.0)
        {}
    };

    struct SpectralBand {
        bool m_valid;
        double m_centerFrequency;
        double m_lowFrequency;
        double m_highFrequency;
        double m_bandwidth;
        double m_peakBinPower;
        double m_totalExcessPower;
        double m_contrastDB;
        double m_peakRatio;
        double m_framePeakPower;
        int m_lowIndex;
        int m_highIndex;

        SpectralBand() :
            m_valid(false),
            m_centerFrequency(0.0),
            m_lowFrequency(0.0),
            m_highFrequency(0.0),
            m_bandwidth(0.0),
            m_peakBinPower(0.0),
            m_totalExcessPower(0.0),
            m_contrastDB(0.0),
            m_peakRatio(0.0),
            m_framePeakPower(0.0),
            m_lowIndex(0),
            m_highIndex(0)
        {}
    };

    struct SpectralEvent {
        bool m_valid;
        quint64 m_startCenterSample;
        quint64 m_lastCenterSample;
        int m_missingFrames;
        double m_peakPower;
        double m_backgroundPower;
        double m_minFrequency;
        double m_maxFrequency;
        double m_maxBandwidth;
        double m_weightedFrequencySum;
        double m_weightSum;
        double m_maxContrastDB;
        double m_maxPeakRatio;
        std::vector<double> m_trackFrequencies;
        std::vector<quint64> m_trackSamples;
        std::vector<double> m_trackStrengths;

        SpectralEvent() :
            m_valid(false),
            m_startCenterSample(0),
            m_lastCenterSample(0),
            m_missingFrames(0),
            m_peakPower(0.0),
            m_backgroundPower(1e-20),
            m_minFrequency(0.0),
            m_maxFrequency(0.0),
            m_maxBandwidth(0.0),
            m_weightedFrequencySum(0.0),
            m_weightSum(0.0),
            m_maxContrastDB(0.0),
            m_maxPeakRatio(0.0)
        {}
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

    ScopeVis* m_scopeSink;
    SpectrumVis* m_spectrumSink;
    MessageQueue *m_messageQueueToGUI;
    MeteorSettings m_settings;

    int m_channelSampleRate;
    int m_channelFrequencyOffset;
    NCO m_nco;
    Interpolator m_interpolator;
    Real m_interpolatorDistance;
    Real m_interpolatorDistanceRemain;
    Lowpass<Real> m_powerLowpass;

    std::array<ComplexVector, m_scopeStreams> m_scopeSampleBuffer;
    int m_scopeSampleBufferSize;
    int m_scopeSampleBufferIndex;
    ComplexVector m_spectrumBuffer;
    ComplexVector m_pulseSamples;
    ComplexVector m_spectralFrameBuffer;
    std::vector<Real> m_spectralWindow;
    std::vector<double> m_spectralBinPower;
    std::vector<double> m_spectralNoiseFloor;
    std::vector<SpectralEvent> m_spectralEvents;
    std::vector<DetectionRange> m_recentDetectionRanges;
    std::vector<PulseReport> m_pendingSpectralReports;
    std::vector<char> m_spectralActiveBins;
    FFTEngine *m_spectralFFT;
    FFTEngine *m_pulseFFT;
    int m_spectralFrameSize;
    int m_spectralHopSize;
    int m_spectralFFTSize;
    int m_pulseFFTSize;
    std::vector<Real> m_pulseFFTWindow;
    bool m_spectralNoiseFloorInitialized;
    bool m_spectralEventActiveForScope;

    quint64 m_sampleCounter;
    std::vector<DisplayTimeAnchor> m_displayTimeAnchors;
    quint64 m_nextDisplayTimeAnchorSample;
    bool m_noiseFloorInitialized;
    double m_noiseFloor;
    bool m_pulseActive;
    bool m_rearmNeeded;
    QDateTime m_streamStartDateTimeUtc;
    quint64 m_pulseStartSample;
    quint64 m_pulseLastAboveSample;
    quint64 m_pulsePeakSample;
    QDateTime m_pulseStartDateTimeUtc;
    double m_pulsePeakPower;
    PulseReport m_pendingBroadPulse;
    void processOneSample(Complex& ci);
    bool processDetectorSample(const Complex& sample, double power, double filteredPower);
    void startPulse(const Complex& sample, double power);
    void updatePulse(const Complex& sample, double power);
    void finishPulse(bool forceRejected);
    bool processSpectralSample(const Complex& sample, double power);
    void processSpectralFrame(quint64 frameStartSample);
    std::vector<SpectralBand> detectSpectralBands(const std::vector<double>& binPower, double framePeakPower);
    void updateSpectralEvents(const std::vector<SpectralBand>& bands, quint64 frameCenterSample);
    void updateSpectralEvent(SpectralEvent& event, const SpectralBand& band, quint64 frameCenterSample);
    void finishSpectralEvent(const SpectralEvent& event);
    double estimateSpectralEventDurationSamples(const SpectralEvent& event, double& startSample) const;
    bool isDuplicateDetection(quint64 startSample, quint64 endSample) const;
    bool isDuplicateDetection(quint64 startSample, quint64 endSample, double centerFrequency, double frequencySpan) const;
    void rememberDetection(quint64 startSample, quint64 endSample);
    void rememberDetection(quint64 startSample, quint64 endSample, double centerFrequency, double frequencySpan);
    void pruneRecentDetections();
    void emitOrDeferSpectralReport(const PulseReport& report);
    void finishPendingSpectralReportsForPulse(
        quint64 pulseEndSample,
        bool usePulseEnvelope,
        bool usePulseFrequency,
        double pulseCenterFrequency,
        double pulseFrequencySpan,
        double pulseFrequencyDrift);
    bool estimatePulseBandEnvelope(PulseReport& report) const;
    bool reportsOverlap(quint64 firstStartSample, quint64 firstEndSample, quint64 secondStartSample, quint64 secondEndSample) const;
    void emitDetectionReport(const PulseReport& report, const char *source);
    QDateTime sampleCounterToDateTimeUtc(quint64 sampleCounter) const;
    QDateTime sampleCounterToDisplayDateTimeUtc(quint64 sampleCounter) const;
    void recordDisplayTimeAnchor();
    void updateNoiseFloor(double filteredPower);
    double getDetectionThresholdPower() const;
    bool estimatePulseFrequency(
        double& centerFrequency,
        double& frequencySpan,
        double& frequencyDrift,
        double& sweepScore,
        double& spectralProminence,
        double& frequencyConcentration,
        double& spectralBandwidth,
        double& spectralBandContrastDB);
    bool estimateWindowPeakFrequency(int startIndex, int windowSize, double& frequency, double& strength, double& prominence);
    bool estimatePulseSpectralBand(int windowSize, int hopSize, double& centerFrequency, double& bandwidth, double& contrastDB);
    bool computePulseWindowSpectrum(int startIndex, int windowSize, std::vector<double>& binPower, double *windowedEnergy);
    bool ensurePulseFFT(int windowSize);
    static double averageFrequency(const std::vector<double>& frequencies, int begin, int end);
    void configureInterpolator();
    void configurePowerLowpass();
    void configureSpectralDetector();
    void resizeScopeBuffers();
    void resetDetector();
    void sampleToScope(const Complex& sample, double power, double filteredPower, bool detected);
    void feedSpectrum(const Complex& sample);
};

#endif // INCLUDE_METEORDEMODSINK_H
