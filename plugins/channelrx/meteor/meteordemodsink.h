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

class MeteorDemodSink : public ChannelSampleSink {
public:
    enum {
        m_scopeStreams = 5
    };

    class MsgMeteorDetected : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QDateTime& getDateTimeUtc() const { return m_dateTimeUtc; }
        double getPeakAmplitude() const { return m_peakAmplitude; }
        double getPeakPowerDB() const { return m_peakPowerDB; }
        double getDurationS() const { return m_durationS; }
        double getCenterFrequency() const { return m_centerFrequency; }
        double getFrequencySpan() const { return m_frequencySpan; }
        double getFrequencyDrift() const { return m_frequencyDrift; }
        int getSampleRate() const { return m_sampleRate; }

        static MsgMeteorDetected* create(
            const QDateTime& dateTimeUtc,
            double peakAmplitude,
            double peakPowerDB,
            double durationS,
            double centerFrequency,
            double frequencySpan,
            double frequencyDrift,
            int sampleRate)
        {
            return new MsgMeteorDetected(
                dateTimeUtc,
                peakAmplitude,
                peakPowerDB,
                durationS,
                centerFrequency,
                frequencySpan,
                frequencyDrift,
                sampleRate
            );
        }

    private:
        QDateTime m_dateTimeUtc;
        double m_peakAmplitude;
        double m_peakPowerDB;
        double m_durationS;
        double m_centerFrequency;
        double m_frequencySpan;
        double m_frequencyDrift;
        int m_sampleRate;

        MsgMeteorDetected(
            const QDateTime& dateTimeUtc,
            double peakAmplitude,
            double peakPowerDB,
            double durationS,
            double centerFrequency,
            double frequencySpan,
            double frequencyDrift,
            int sampleRate
        ) :
            Message(),
            m_dateTimeUtc(dateTimeUtc),
            m_peakAmplitude(peakAmplitude),
            m_peakPowerDB(peakPowerDB),
            m_durationS(durationS),
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

    quint64 m_sampleCounter;
    bool m_noiseFloorInitialized;
    double m_noiseFloor;
    bool m_pulseActive;
    bool m_rearmNeeded;
    quint64 m_pulseStartSample;
    quint64 m_pulseLastAboveSample;
    QDateTime m_pulseStartDateTimeUtc;
    double m_pulsePeakPower;
    void processOneSample(Complex& ci);
    bool processDetectorSample(const Complex& sample, double power, double filteredPower);
    void startPulse(const Complex& sample, double power);
    void updatePulse(const Complex& sample, double power);
    void finishPulse(bool forceRejected);
    void updateNoiseFloor(double filteredPower);
    double getDetectionThresholdPower() const;
    bool estimatePulseFrequency(double& centerFrequency, double& frequencySpan, double& frequencyDrift, double& sweepScore) const;
    bool estimateWindowPeakFrequency(int startIndex, int windowSize, double& frequency, double& strength) const;
    static double averageFrequency(const std::vector<double>& frequencies, int begin, int end);
    void configureInterpolator();
    void configurePowerLowpass();
    void resizeScopeBuffers();
    void resetDetector();
    void sampleToScope(const Complex& sample, double power, double filteredPower, bool detected);
    void feedSpectrum(const Complex& sample);
};

#endif // INCLUDE_METEORDEMODSINK_H
