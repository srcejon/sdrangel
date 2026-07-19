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
#include <functional>
#include <vector>

#include <QDateTime>

#include "dsp/channelsamplesink.h"
#include "dsp/firfilter.h"
#include "dsp/interpolator.h"
#include "dsp/nco.h"
#include "util/message.h"

#include "meteorsettings.h"

class ChannelAPI;
class MessageQueue;
class SpectrumVis;
class FFTEngine;

class MeteorDemodSink : public ChannelSampleSink {
public:
    struct DetectorTunables
    {
        double m_spectralActiveNoiseAlpha = 0.0005;
        double m_spectralRisingNoiseAlpha = 0.05;
        double m_spectralStableNoiseAlpha = 0.02;
        double m_scalarNoiseTimeConstantS = 10.0;
        double m_scalarRisingNoiseAlpha = 0.05;
        double m_edgeExclusionFraction = 0.05;
        double m_usableBandwidthRateFraction = 0.45;
        double m_compactBandwidthRateFraction = 0.05;
        double m_stableBandwidthRateFraction = 0.22;
        double m_twoFrameMaxBandwidthRateFraction = 0.065;
        double m_twoFrameMinFrequencyCoherence = 0.75;
        double m_twoFrameMinIntegratedSupportDB = -7.0;
        double m_localizedTwoFrameMinPeakDB = 12.0;
        double m_localizedTwoFrameMinContrastDB = 18.0;
        double m_coherentWideTwoFrameMinBandwidthRateFraction = 0.09;
        double m_coherentWideTwoFrameMaxBandwidthRateFraction = 0.13;
        double m_coherentWideTwoFrameMaxOccupiedFraction = 0.25;
        double m_coherentWideTwoFrameMinPeakDB = 11.5;
        double m_coherentWideTwoFrameMinContrastDB = 14.0;
        double m_coherentWideTwoFrameMinIntegratedSupportDB = 5.0;
        double m_coherentWideTwoFrameMinFrequencyCoherence = 0.9;
        double m_morphologyRescueContrastDB = 14.0;
        double m_localizedOccupiedMaxFraction = 0.45;
        double m_sustainedSweepMinDurationS = 0.5;
        int m_sustainedSweepMinFrames = 8;
        double m_sustainedSweepMinR2 = 0.85;
        double m_sustainedSweepMinDriftBins = 2.0;
        double m_compactSweepMinDurationS = 0.2;
        int m_compactSweepMinFrames = 4;
        double m_compactSweepMaxTrackOccupancy = 0.80;
        double m_compactSweepMaxContrastDB = 18.0;
        double m_compactSweepMinR2 = 0.85;
        double m_compactSweepMinDriftBins = 4.0;
        double m_driftSweepMinR2 = 0.65;
        double m_broadbandImpulseMaxDurationS = 0.22;
        int m_broadbandImpulseMaxFrames = 3;
        double m_broadbandImpulseMinPeakDB = 12.0;
        double m_broadbandImpulseMinSpanRateFraction = 0.12;
        double m_broadbandImpulseMinBandwidthRateFraction = 0.10;
        double m_broadbandImpulseMinOccupiedFraction = 0.45;
        double m_shortCandidateAcceptanceScore = 9.5;
        double m_candidateAcceptanceScore = 9.0;
        double m_weakSupportDB = -8.0;
        double m_weakSupportScorePenalty = 1.0;
        double m_scoreDurationFloorS = 0.05;
        double m_scoreDurationRangeS = 0.45;
        double m_powerSweepMinR2 = 0.75;
        double m_powerStableBandwidthRateFraction = 0.18;
        double m_powerBoundedMinProminenceDB = 9.0;
        double m_powerStrongCoherentMinPeakDB = 18.0;
        double m_powerStrongCoherentMinProminenceDB = 18.0;
        double m_frequencyRefinementOuterProbeFraction = 0.85;
        double m_duplicateFrequencyOverlapFraction = 0.65;
        double m_duplicateStrongFrequencyOverlapFraction = 0.85;
        double m_continuationThresholdReductionDB = 3.0;
        double m_continuationOrdinaryHoldS = 1.0;
        double m_continuationStrongHoldS = 3.0;
        double m_continuationMaxEvidenceGapS = 0.75;
        double m_continuationStrongPeakDB = 12.0;
        double m_continuationFrequencyPaddingBins = 2.5;
        double m_parentFrequencyLowQuantile = 0.10;
        double m_parentFrequencyHighQuantile = 0.90;
        int m_maxActiveMeteorEvents = 16;
        int m_maxParentObservations = 512;
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

    struct SpectralCandidate : DetectionRecord
    {
        quint64 m_peakSample = 0;
        double m_robustCenterFrequency = 0.0;
        double m_robustFrequencySpan = 0.0;
        double m_robustFrequencyDrift = 0.0;
        double m_reportFrequencySpan = 0.0;
        double m_sweepScore = 0.0;
        double m_peakAboveBackgroundDB = 0.0;
        double m_integratedSupportDB = 0.0;
        double m_maxBandwidth = 0.0;
        double m_maxContrastDB = 0.0;
        double m_maxPeakRatio = 0.0;
        double m_acceptanceScore = 0.0;
        double m_acceptanceThreshold = 0.0;
        double m_scoreMargin = 0.0;
        double m_signalScore = 0.0;
        double m_supportScore = 0.0;
        double m_shapeScore = 0.0;
        double m_rejectionPenalty = 0.0;
        double m_trackOccupancy = 0.0;
        double m_frequencyCoherence = 0.0;
        double m_frameOccupiedFraction = 0.0;
        int m_frameCount = 0;
        bool m_durationOK = false;
        bool m_enoughFrames = false;
        bool m_smoothSweepRejected = false;
        bool m_longDriftRejected = false;
        bool m_sweepRejected = false;
        bool m_strongLineOK = false;
        bool m_boundedBandOK = false;
        bool m_spectralEvidenceOK = false;
        bool m_insideUsableBandwidth = false;
        bool m_duplicate = false;
        bool m_broadbandImpulse = false;
        bool m_sweepContinuationRejected = false;
        bool m_scoreOK = false;
        bool m_accepted = false;
        quint64 m_parentEventId = 0;
        QString m_associationDecision = QStringLiteral("none");
        QString m_classification = QStringLiteral("invalid");
        QString m_rejectionReason = QStringLiteral("invalid");
    };

    using CandidateAudit = SpectralCandidate;
    using CandidateAuditCallback = std::function<void(const CandidateAudit&)>;
    using DiagnosticCaptureCallback = std::function<void(
        quint64,
        const DetectionRecord&,
        const ComplexVector&)>;

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
    bool flushPendingPulse();

    void setSpectrumSink(SpectrumVis* spectrumSink) { m_spectrumSink = spectrumSink; }
    void setSecondarySpectrumSink(SpectrumVis* spectrumSink) { m_secondarySpectrumSink = spectrumSink; }
    void setMessageQueueToGUI(MessageQueue *messageQueue) { m_messageQueueToGUI = messageQueue; }
    void setCandidateAuditCallback(const CandidateAuditCallback& callback) { m_candidateAuditCallback = callback; }
    void setDiagnosticCaptureCallback(const DiagnosticCaptureCallback& callback) { m_diagnosticCaptureCallback = callback; }
    void setDetectorTunables(const DetectorTunables& tunables) { m_detectorTunables = tunables; }
    const DetectorTunables& getDetectorTunables() const { return m_detectorTunables; }
    void setChannel(ChannelAPI *channel) { m_channel = channel; }
    void applyChannelSettings(int channelSampleRate, int channelFrequencyOffset, bool force = false);
    void applySettings(const MeteorSettings& settings, const QStringList& settingsKeys, bool force = false);
    int getOutputSampleRate() const { return m_settings.m_channelSampleRate; }
    static quint64 clipDetectionEndSample(
        quint64 startSample,
        quint64 endSample,
        quint64 maximumDurationSamples,
        bool& truncated);

private:
    struct PulseReport : DetectionRecord {
        double m_reportFrequencySpan;
        double m_duplicateFrequencySpan;
        bool m_hasRobustFrequency;
        double m_robustCenterFrequency;
        double m_robustFrequencySpan;
        double m_robustFrequencyDrift;
        double m_confidence;
        double m_componentSupportDB;
        bool m_allowComponentMerge;
        bool m_spectralParentEligible;
        PulseReport() :
            m_reportFrequencySpan(0.0),
            m_duplicateFrequencySpan(0.0),
            m_hasRobustFrequency(false),
            m_robustCenterFrequency(0.0),
            m_robustFrequencySpan(0.0),
            m_robustFrequencyDrift(0.0),
            m_confidence(0.0),
            m_componentSupportDB(-200.0),
            m_allowComponentMerge(true),
            m_spectralParentEligible(false)
        {}
    };

    struct MeteorEventObservation {
        quint64 m_sample = 0;
        double m_centerFrequency = 0.0;
        double m_lowFrequency = 0.0;
        double m_highFrequency = 0.0;
        double m_weight = 1.0;
        double m_peakPower = 0.0;
        double m_backgroundPower = 1e-20;
        bool m_broadband = false;
    };

    struct ActiveMeteorEvent {
        quint64 m_id = 0;
        PulseReport m_report;
        quint64 m_lastEvidenceSample = 0;
        quint64 m_lastComponentSample = 0;
        int m_componentCount = 0;
        int m_spectralComponentCount = 0;
        bool m_strong = false;
        bool m_extendedByContinuation = false;
        std::vector<MeteorEventObservation> m_observations;
    };

    struct AssociationResult {
        quint64 m_parentEventId = 0;
        QString m_decision = QStringLiteral("rejected");
    };

    struct SpectralBand {
        bool m_valid;
        double m_centerFrequency;
        double m_peakFrequency;
        double m_lowFrequency;
        double m_highFrequency;
        double m_bandwidth;
        double m_peakBinPower;
        double m_totalExcessPower;
        double m_contrastDB;
        double m_peakRatio;
        double m_framePeakPower;
        double m_frameOccupiedFraction;
        double m_backgroundPower;
        int m_lowIndex;
        int m_highIndex;

        SpectralBand() :
            m_valid(false),
            m_centerFrequency(0.0),
            m_peakFrequency(0.0),
            m_lowFrequency(0.0),
            m_highFrequency(0.0),
            m_bandwidth(0.0),
            m_peakBinPower(0.0),
            m_totalExcessPower(0.0),
            m_contrastDB(0.0),
            m_peakRatio(0.0),
            m_framePeakPower(0.0),
            m_frameOccupiedFraction(0.0),
            m_backgroundPower(1e-20),
            m_lowIndex(0),
            m_highIndex(0)
        {}
    };

    struct SpectralFrameSnapshot {
        quint64 m_startSample;
        std::vector<double> m_binPower;

        SpectralFrameSnapshot() :
            m_startSample(0)
        {}
    };

    struct SpectralEvent {
        bool m_valid;
        quint64 m_startCenterSample;
        quint64 m_lastCenterSample;
        int m_missingFrames;
        double m_peakPower;
        double m_backgroundPower;
        double m_backgroundPowerSum;
        int m_backgroundFrameCount;
        double m_totalPower;
        double m_minFrequency;
        double m_maxFrequency;
        double m_maxBandwidth;
        double m_weightedFrequencySum;
        double m_weightSum;
        double m_maxContrastDB;
        double m_maxPeakRatio;
        std::vector<double> m_trackFrequencies;
        std::vector<double> m_trackPeakFrequencies;
        std::vector<double> m_trackLowFrequencies;
        std::vector<double> m_trackHighFrequencies;
        std::vector<double> m_trackBandwidths;
        std::vector<quint64> m_trackSamples;
        std::vector<double> m_trackStrengths;
        std::vector<double> m_trackFrameOccupiedFractions;

        SpectralEvent() :
            m_valid(false),
            m_startCenterSample(0),
            m_lastCenterSample(0),
            m_missingFrames(0),
            m_peakPower(0.0),
            m_backgroundPower(1e-20),
            m_backgroundPowerSum(0.0),
            m_backgroundFrameCount(0),
            m_totalPower(0.0),
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

    struct SpectralInterferenceRange {
        quint64 m_startSample;
        quint64 m_endSample;
        double m_centerFrequency;
        double m_frequencySpan;
        bool m_broadband;
    };

    struct DisplayTimeAnchor {
        quint64 m_sampleCounter;
        QDateTime m_dateTimeUtc;
    };

    SpectrumVis* m_spectrumSink;
    SpectrumVis* m_secondarySpectrumSink;
    MessageQueue *m_messageQueueToGUI;
    CandidateAuditCallback m_candidateAuditCallback;
    DiagnosticCaptureCallback m_diagnosticCaptureCallback;
    DetectorTunables m_detectorTunables;
    MeteorSettings m_settings;
    ChannelAPI *m_channel;

    int m_channelSampleRate;
    int m_channelFrequencyOffset;
    NCO m_nco;
    Interpolator m_interpolator;
    Real m_interpolatorDistance;
    Real m_interpolatorDistanceRemain;
    Lowpass<Real> m_powerLowpass;

    int m_spectrumBufferSize;
    ComplexVector m_spectrumBuffer;
    ComplexVector m_pulseSamples;
    ComplexVector m_detectionSampleRing;
    ComplexVector m_spectralFrameBuffer;
    std::vector<Real> m_spectralWindow;
    std::vector<double> m_spectralBinPower;
    std::vector<double> m_spectralNoiseFloor;
    std::vector<SpectralFrameSnapshot> m_spectralCalibrationFrames;
    std::vector<SpectralEvent> m_spectralEvents;
    std::vector<DetectionRange> m_recentDetectionRanges;
    std::vector<SpectralInterferenceRange> m_recentSpectralInterference;
    std::vector<ActiveMeteorEvent> m_activeMeteorEvents;
    quint64 m_nextMeteorEventId;
    quint64 m_nextComponentFlushSample;
    std::vector<char> m_spectralActiveBins;
    FFTEngine *m_spectralFFT;
    FFTEngine *m_pulseFFT;
    int m_spectralFrameSize;
    int m_spectralHopSize;
    int m_spectralFFTSize;
    int m_pulseFFTSize;
    double m_spectralEnergyScale;
    int m_detectionSampleRingStart;
    int m_detectionSampleRingCount;
    quint64 m_detectionSampleRingStartSample;
    std::vector<Real> m_pulseFFTWindow;
    bool m_spectralNoiseFloorInitialized;
    quint64 m_sampleCounter;
    std::vector<DisplayTimeAnchor> m_displayTimeAnchors;
    quint64 m_nextDisplayTimeAnchorSample;
    quint64 m_nextDataMarkerSample;
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
    void processOneSample(Complex& ci, bool realSample = true);
    void processDetectorSample(const Complex& sample, double power, double filteredPower);
    void startPulse(const Complex& sample, double power);
    void updatePulse(const Complex& sample, double power);
    void finishPulse(bool forceRejected);
    void processSpectralSample(const Complex& sample);
    void processSpectralFrame(quint64 frameStartSample);
    void initializeSpectralNoiseFloor();
    void processCalibratedSpectralFrame(const SpectralFrameSnapshot& frame);
    std::vector<SpectralBand> detectSpectralBands(const std::vector<double>& binPower);
    bool frequencyProtectedByActiveMeteor(double frequency) const;
    void protectActiveMeteorNoiseBins();
    void updateActiveMeteorEvents(const std::vector<SpectralBand>& bands, quint64 frameCenterSample);
    bool bandCompatibleWithActiveMeteor(const ActiveMeteorEvent& event, const SpectralBand& band) const;
    void addMeteorEventObservation(ActiveMeteorEvent& event, const MeteorEventObservation& observation);
    void updateSpectralEvents(const std::vector<SpectralBand>& bands, quint64 frameCenterSample);
    void updateSpectralEvent(SpectralEvent& event, const SpectralBand& band, quint64 frameCenterSample);
    void finishSpectralEvent(const SpectralEvent& event);
    void auditSpectralCandidate(const SpectralCandidate& candidate) const;
    bool isSweepContinuation(const SpectralCandidate& candidate) const;
    bool overlapsBroadbandInterference(quint64 startSample, quint64 endSample) const;
    void rememberSpectralInterference(const SpectralCandidate& candidate);
    SpectralCandidate buildSpectralCandidate(const SpectralEvent& event) const;
    void classifySpectralCandidate(SpectralCandidate& candidate) const;
    double scoreSpectralCandidate(SpectralCandidate& candidate) const;
    static double weightedQuantile(
        const std::vector<double>& values,
        const std::vector<double>& weights,
        double quantile,
        double fallback);
    static double weightedQuantile(
        const std::vector<double>& values,
        const std::vector<double>& weights,
        int begin,
        int end,
        double quantile,
        double fallback);
    double estimateSpectralEventDurationSamples(const SpectralEvent& event, double& startSample) const;
    bool isDuplicateDetection(quint64 startSample, quint64 endSample) const;
    bool isDuplicateDetection(quint64 startSample, quint64 endSample, double centerFrequency, double frequencySpan) const;
    void rememberDetection(quint64 startSample, quint64 endSample);
    void rememberDetection(quint64 startSample, quint64 endSample, double centerFrequency, double frequencySpan);
    void pruneRecentDetections();
    AssociationResult emitOrDeferSpectralReport(const PulseReport& report);
    AssociationResult queueSpectralComponentReport(const PulseReport& report);
    void scheduleNextComponentFlush();
    void flushPendingComponentReports(bool force);
    void finalizeActiveMeteorEvent(ActiveMeteorEvent& event);
    void updateReportFromActiveMeteorEvent(ActiveMeteorEvent& event) const;
    PulseReport reportWithRobustFrequency(const PulseReport& report) const;
    bool estimatePulseBandEnvelope(PulseReport& report) const;
    bool refineSpectralComponentEnvelope(PulseReport& report) const;
    bool refineBandEnvelope(PulseReport& report, bool allowShrink) const;
    void appendDetectionSample(const Complex& sample);
    bool copyDetectionSamples(quint64 startSample, quint64 endSample, ComplexVector& samples) const;
    double estimatePulseTotalPower(quint64 startSample, quint64 endSample, double backgroundPower) const;
    bool reportsFrequencyCompatible(const PulseReport& first, const PulseReport& second) const;
    void emitDetectionReport(const PulseReport& report, const char *source);
    QDateTime sampleCounterToDateTimeUtc(quint64 sampleCounter) const;
    QDateTime sampleCounterToDisplayDateTimeUtc(quint64 sampleCounter) const;
    void recordDisplayTimeAnchor();
    void emitDataCollectionMarker();
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
    void configureDetectionHistory();
    void configureSpectralDetector();
    void resizeSpectrumBuffer();
    void resetDetector();
    void feedSpectrum(const Complex& sample);
};

#endif // INCLUDE_METEORDEMODSINK_H
