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

#include <algorithm>
#include <cmath>
#include <limits>

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QList>
#include <QRegularExpression>
#include <QSet>
#include <QStringList>
#include <QTextStream>
#include <QTemporaryDir>
#include <QVector>
#include <QtEndian>
#include <QtGlobal>

#include "dsp/dspcommands.h"
#include "dsp/dsptypes.h"
#include "dsp/wavfilerecord.h"
#include "util/message.h"
#include "util/messagequeue.h"

#include "meteorbaseband.h"
#include "meteordemodsink.h"
#include "meteorsettings.h"
#include "rmobreport.h"

#ifndef METEOR_TEST_DATA_DIR
#define METEOR_TEST_DATA_DIR ""
#endif

namespace {
    bool runMeteorSettingsTests(QTextStream& errorStream)
    {
        MeteorSettings expected;
        expected.m_transmitterLatitude = 50.123456;
        expected.m_transmitterLongitude = -1.234567;
        expected.m_transmitterAzimuth = 135.5f;
        expected.m_transmitterElevation = 8.5f;
        expected.m_transmitterBeamwidth = 12.0f;
        expected.m_antennaAzimuth = 220.5f;
        expected.m_antennaElevation = 11.5f;
        expected.m_antennaBeamwidth = 30.0f;
        expected.m_rotator = QStringLiteral("2:3");

        MeteorSettings actual;

        if (!actual.deserialize(expected.serialize())
            || (actual.m_transmitterLatitude != expected.m_transmitterLatitude)
            || (actual.m_transmitterLongitude != expected.m_transmitterLongitude)
            || (actual.m_transmitterAzimuth != expected.m_transmitterAzimuth)
            || (actual.m_transmitterElevation != expected.m_transmitterElevation)
            || (actual.m_transmitterBeamwidth != expected.m_transmitterBeamwidth)
            || (actual.m_antennaAzimuth != expected.m_antennaAzimuth)
            || (actual.m_antennaElevation != expected.m_antennaElevation)
            || (actual.m_antennaBeamwidth != expected.m_antennaBeamwidth)
            || (actual.m_rotator != expected.m_rotator))
        {
            errorStream << "Meteor settings test: geometry settings did not round-trip\n";
            return false;
        }

        return true;
    }

    bool runRMOBReportTests(QTextStream& errorStream)
    {
        QTemporaryDir temporaryDir;

        if (!temporaryDir.isValid())
        {
            errorStream << "RMOB test: failed to create temporary directory\n";
            return false;
        }

        const QDate monthDate(2026, 5, 1);
        const QDate firstDay(2026, 5, 1);
        RMOBReport::Data expected;
        expected.m_hourlyCounts[firstDay] = QVector<int>(24, 0);
        expected.m_hourlyData[firstDay] = QVector<bool>(24, false);
        expected.m_hourlyCounts[firstDay][0] = 3;
        expected.m_hourlyData[firstDay][0] = true;
        expected.m_hourlyData[firstDay][1] = true;

        RMOBReport::Metadata metadata;
        metadata.m_observer = "TEST";
        metadata.m_latitude = 50.0535;
        metadata.m_longitude = 19.8235;
        metadata.m_frequency = 143050000;
        metadata.m_receiver = "Test receiver";
        metadata.m_antennaAzimuth = 220.0;
        metadata.m_antennaElevation = 11.0;
        metadata.m_antennaBeamwidth = 30.0;

        const QString fileName = temporaryDir.filePath("meteor_2026_05.rmob.txt");
        QString error;

        if (!RMOBReport::save(fileName, monthDate, expected, metadata, &error))
        {
            errorStream << "RMOB test: " << error << "\n";
            return false;
        }

        QFile reportFile(fileName);

        if (!reportFile.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            errorStream << "RMOB test: failed to reopen saved report\n";
            return false;
        }

        const QString reportText = QString::fromUtf8(reportFile.readAll());

        if (!reportText.startsWith("may|00h|01h|")
            || !reportText.contains("01|3   |0   |??? |")
            || !reportText.contains("[Observer]TEST\n")
            || !reportText.contains("[Frequencies]0143.050.000\n")
            || !reportText.contains("[Antenna]HPBW 30.0 deg\n")
            || !reportText.contains("[Azimut Antenna]220.0\n")
            || !reportText.contains("[Elevation Antenna]11.0\n")
            || !reportText.contains("[Receiver]Test receiver\n"))
        {
            errorStream << "RMOB test: saved report content is incorrect\n";
            return false;
        }

        RMOBReport::Data actual;

        if (!RMOBReport::load(fileName, monthDate, actual, &error))
        {
            errorStream << "RMOB test: " << error << "\n";
            return false;
        }

        const bool roundTripOK = (actual.m_totalCount == 3)
            && (actual.m_hourlyCounts.size() == 1)
            && (actual.m_hourlyData.size() == 1)
            && (actual.m_hourlyCounts.value(firstDay).value(0) == 3)
            && (actual.m_hourlyCounts.value(firstDay).value(1) == 0)
            && actual.m_hourlyData.value(firstDay).value(0)
            && actual.m_hourlyData.value(firstDay).value(1)
            && !actual.m_hourlyData.value(firstDay).value(2)
            && !actual.m_hourlyCounts.contains(QDate(2026, 5, 31))
            && !actual.m_hourlyData.contains(QDate(2026, 5, 31));

        if (!roundTripOK) {
            errorStream << "RMOB test: round-trip data differs\n";
        }

        return roundTripOK;
    }

    struct Detection
    {
        QDateTime dateTimeUtc;
        QDateTime displayDateTimeUtc;
        double peakAmplitude;
        double peakPowerDB;
        double backgroundPowerDB;
        double totalPowerDB;
        double durationS;
        double centerFrequency;
        double frequencySpan;
        double frequencyDrift;
        int sampleRate;
        quint64 startSample;
        quint64 endSample;
    };

    struct ExpectedDetection
    {
        int index;
        double timeOffsetS;
        double durationS;
        double centerFrequency;
        double frequencySpan;
        double frequencyDrift;
        double totalPowerDB;
        bool required = true;
    };

    struct CandidateLabel
    {
        quint64 startSample = 0;
        quint64 endSample = 0;
        double lowFrequency = 0.0;
        double highFrequency = 0.0;
        bool hasFrequencyRange = false;
        QString label;
        QString eventId;
    };

    struct CandidateLabelMatch
    {
        QString label;
        QString eventId;
    };

    struct Options
    {
        QString wavPath;
        QString testDir;
        QString candidateCsvPath;
        QString candidateLabelsPath;
        QString candidateCaptureDir;
        QVector<QPair<QString, QString>> tunableOverrides;
        MeteorSettings settings;
        int chunkSamples = 4096;
        int tailMS = 2000;
        int expectCount = -1;
        bool details = false;
        bool showHelp = false;
    };

    bool applyTunableOverrides(
        MeteorDemodSink::DetectorTunables& tunables,
        const QVector<QPair<QString, QString>>& overrides,
        QString& error)
    {
        using Tunables = MeteorDemodSink::DetectorTunables;
        static const QMap<QString, double Tunables::*> doubleMembers = {
            {QStringLiteral("spectralFrameDurationS"), &Tunables::m_spectralFrameDurationS},
            {QStringLiteral("spectralHopFraction"), &Tunables::m_spectralHopFraction},
            {QStringLiteral("spectralActiveNoiseAlpha"), &Tunables::m_spectralActiveNoiseAlpha},
            {QStringLiteral("spectralRisingNoiseAlpha"), &Tunables::m_spectralRisingNoiseAlpha},
            {QStringLiteral("spectralStableNoiseAlpha"), &Tunables::m_spectralStableNoiseAlpha},
            {QStringLiteral("minimumNoiseBlockDurationS"), &Tunables::m_minimumNoiseBlockDurationS},
            {QStringLiteral("minimumNoiseBlockQuantile"), &Tunables::m_minimumNoiseBlockQuantile},
            {QStringLiteral("scalarNoiseTimeConstantS"), &Tunables::m_scalarNoiseTimeConstantS},
            {QStringLiteral("scalarRisingNoiseAlpha"), &Tunables::m_scalarRisingNoiseAlpha},
            {QStringLiteral("edgeExclusionFraction"), &Tunables::m_edgeExclusionFraction},
            {QStringLiteral("usableBandwidthRateFraction"), &Tunables::m_usableBandwidthRateFraction},
            {QStringLiteral("maxSegmentedBandWidthHz"), &Tunables::m_maxSegmentedBandWidthHz},
            {QStringLiteral("compactBandwidthHz"), &Tunables::m_compactBandwidthHz},
            {QStringLiteral("stableBandwidthHz"), &Tunables::m_stableBandwidthHz},
            {QStringLiteral("twoFrameMaxBandwidthHz"), &Tunables::m_twoFrameMaxBandwidthHz},
            {QStringLiteral("twoFrameMinFrequencyCoherence"), &Tunables::m_twoFrameMinFrequencyCoherence},
            {QStringLiteral("twoFrameMinIntegratedSupportDB"), &Tunables::m_twoFrameMinIntegratedSupportDB},
            {QStringLiteral("localizedTwoFrameMinPeakDB"), &Tunables::m_localizedTwoFrameMinPeakDB},
            {QStringLiteral("localizedTwoFrameMinContrastDB"), &Tunables::m_localizedTwoFrameMinContrastDB},
            {QStringLiteral("coherentWideTwoFrameMinBandwidthHz"), &Tunables::m_coherentWideTwoFrameMinBandwidthHz},
            {QStringLiteral("coherentWideTwoFrameMaxBandwidthHz"), &Tunables::m_coherentWideTwoFrameMaxBandwidthHz},
            {QStringLiteral("coherentWideTwoFrameMaxOccupiedFraction"), &Tunables::m_coherentWideTwoFrameMaxOccupiedFraction},
            {QStringLiteral("coherentWideTwoFrameMinPeakDB"), &Tunables::m_coherentWideTwoFrameMinPeakDB},
            {QStringLiteral("coherentWideTwoFrameMinContrastDB"), &Tunables::m_coherentWideTwoFrameMinContrastDB},
            {QStringLiteral("coherentWideTwoFrameMinIntegratedSupportDB"), &Tunables::m_coherentWideTwoFrameMinIntegratedSupportDB},
            {QStringLiteral("coherentWideTwoFrameMinFrequencyCoherence"), &Tunables::m_coherentWideTwoFrameMinFrequencyCoherence},
            {QStringLiteral("morphologyRescueContrastDB"), &Tunables::m_morphologyRescueContrastDB},
            {QStringLiteral("localizedOccupiedMaxFraction"), &Tunables::m_localizedOccupiedMaxFraction},
            {QStringLiteral("sustainedSweepMinDurationS"), &Tunables::m_sustainedSweepMinDurationS},
            {QStringLiteral("sustainedSweepMinR2"), &Tunables::m_sustainedSweepMinR2},
            {QStringLiteral("sustainedSweepMinDriftHz"), &Tunables::m_sustainedSweepMinDriftHz},
            {QStringLiteral("compactSweepMinDurationS"), &Tunables::m_compactSweepMinDurationS},
            {QStringLiteral("compactSweepMaxTrackOccupancy"), &Tunables::m_compactSweepMaxTrackOccupancy},
            {QStringLiteral("compactSweepMaxContrastDB"), &Tunables::m_compactSweepMaxContrastDB},
            {QStringLiteral("compactSweepMinR2"), &Tunables::m_compactSweepMinR2},
            {QStringLiteral("compactSweepMinDriftHz"), &Tunables::m_compactSweepMinDriftHz},
            {QStringLiteral("localizedBurstMaxDurationS"), &Tunables::m_localizedBurstMaxDurationS},
            {QStringLiteral("localizedBurstMinTrackOccupancy"), &Tunables::m_localizedBurstMinTrackOccupancy},
            {QStringLiteral("localizedBurstMaxOccupiedFraction"), &Tunables::m_localizedBurstMaxOccupiedFraction},
            {QStringLiteral("localizedBurstMinPeakDB"), &Tunables::m_localizedBurstMinPeakDB},
            {QStringLiteral("localizedBurstMinContrastDB"), &Tunables::m_localizedBurstMinContrastDB},
            {QStringLiteral("localizedBurstMinIntegratedSupportDB"), &Tunables::m_localizedBurstMinIntegratedSupportDB},
            {QStringLiteral("localizedBurstMinFrequencyCoherence"), &Tunables::m_localizedBurstMinFrequencyCoherence},
            {QStringLiteral("localizedBurstMinMatchedEnvelopeScore"), &Tunables::m_localizedBurstMinMatchedEnvelopeScore},
            {QStringLiteral("driftSweepMinR2"), &Tunables::m_driftSweepMinR2},
            {QStringLiteral("broadbandImpulseMaxDurationS"), &Tunables::m_broadbandImpulseMaxDurationS},
            {QStringLiteral("broadbandImpulseMinPeakDB"), &Tunables::m_broadbandImpulseMinPeakDB},
            {QStringLiteral("broadbandImpulseMinSpanHz"), &Tunables::m_broadbandImpulseMinSpanHz},
            {QStringLiteral("broadbandImpulseMinBandwidthHz"), &Tunables::m_broadbandImpulseMinBandwidthHz},
            {QStringLiteral("broadbandImpulseMinOccupiedFraction"), &Tunables::m_broadbandImpulseMinOccupiedFraction},
            {QStringLiteral("shortCandidateAcceptanceScore"), &Tunables::m_shortCandidateAcceptanceScore},
            {QStringLiteral("candidateAcceptanceScore"), &Tunables::m_candidateAcceptanceScore},
            {QStringLiteral("weakSupportDB"), &Tunables::m_weakSupportDB},
            {QStringLiteral("weakSupportScorePenalty"), &Tunables::m_weakSupportScorePenalty},
            {QStringLiteral("scoreDurationFloorS"), &Tunables::m_scoreDurationFloorS},
            {QStringLiteral("scoreDurationRangeS"), &Tunables::m_scoreDurationRangeS},
            {QStringLiteral("powerSweepMinR2"), &Tunables::m_powerSweepMinR2},
            {QStringLiteral("powerStableBandwidthHz"), &Tunables::m_powerStableBandwidthHz},
            {QStringLiteral("powerBoundedMinProminenceDB"), &Tunables::m_powerBoundedMinProminenceDB},
            {QStringLiteral("powerStrongCoherentMinPeakDB"), &Tunables::m_powerStrongCoherentMinPeakDB},
            {QStringLiteral("powerStrongCoherentMinProminenceDB"), &Tunables::m_powerStrongCoherentMinProminenceDB},
            {QStringLiteral("frequencyRefinementOuterProbeFraction"), &Tunables::m_frequencyRefinementOuterProbeFraction},
            {QStringLiteral("duplicateFrequencyOverlapFraction"), &Tunables::m_duplicateFrequencyOverlapFraction},
            {QStringLiteral("duplicateStrongFrequencyOverlapFraction"), &Tunables::m_duplicateStrongFrequencyOverlapFraction},
            {QStringLiteral("detachedRepeatMinimumGapS"), &Tunables::m_detachedRepeatMinimumGapS},
            {QStringLiteral("detachedRepeatMinimumScoreMargin"), &Tunables::m_detachedRepeatMinimumScoreMargin},
            {QStringLiteral("detachedRepeatMinimumContrastDB"), &Tunables::m_detachedRepeatMinimumContrastDB},
            {QStringLiteral("detachedRepeatMinimumPeakDB"), &Tunables::m_detachedRepeatMinimumPeakDB},
            {QStringLiteral("detachedRepeatMinimumIntegratedSupportDB"), &Tunables::m_detachedRepeatMinimumIntegratedSupportDB},
            {QStringLiteral("detachedRepeatMinimumTrackOccupancy"), &Tunables::m_detachedRepeatMinimumTrackOccupancy},
            {QStringLiteral("detachedRepeatMinimumFrequencyCoherence"), &Tunables::m_detachedRepeatMinimumFrequencyCoherence},
            {QStringLiteral("detachedRepeatMaximumOccupiedFraction"), &Tunables::m_detachedRepeatMaximumOccupiedFraction},
            {QStringLiteral("detachedRepeatMinimumMatchedEnvelopeScore"), &Tunables::m_detachedRepeatMinimumMatchedEnvelopeScore},
            {QStringLiteral("continuationThresholdReductionDB"), &Tunables::m_continuationThresholdReductionDB},
            {QStringLiteral("initialComponentHoldS"), &Tunables::m_initialComponentHoldS},
            {QStringLiteral("continuationOrdinaryHoldS"), &Tunables::m_continuationOrdinaryHoldS},
            {QStringLiteral("continuationStrongHoldS"), &Tunables::m_continuationStrongHoldS},
            {QStringLiteral("continuationMaxEvidenceGapS"), &Tunables::m_continuationMaxEvidenceGapS},
            {QStringLiteral("continuationStrongPeakDB"), &Tunables::m_continuationStrongPeakDB},
            {QStringLiteral("singleComponentContinuationMinimumConfidence"), &Tunables::m_singleComponentContinuationMinimumConfidence},
            {QStringLiteral("singleComponentContinuationTailPaddingS"), &Tunables::m_singleComponentContinuationTailPaddingS},
            {QStringLiteral("continuationFrequencyPaddingHz"), &Tunables::m_continuationFrequencyPaddingHz},
            {QStringLiteral("trackingFrequencyPaddingHz"), &Tunables::m_trackingFrequencyPaddingHz},
            {QStringLiteral("maxTrackingJumpHz"), &Tunables::m_maxTrackingJumpHz},
            {QStringLiteral("candidateDiagnosticMinimumMargin"), &Tunables::m_candidateDiagnosticMinimumMargin},
            {QStringLiteral("candidateDiagnosticMaximumMargin"), &Tunables::m_candidateDiagnosticMaximumMargin},
            {QStringLiteral("matchedEnvelopeMaximumPeakPosition"), &Tunables::m_matchedEnvelopeMaximumPeakPosition},
            {QStringLiteral("matchedEnvelopeMinimumDecayDB"), &Tunables::m_matchedEnvelopeMinimumDecayDB},
            {QStringLiteral("matchedEnvelopeMaximumDecayDB"), &Tunables::m_matchedEnvelopeMaximumDecayDB},
            {QStringLiteral("matchedEnvelopeMinimumMonotonicFraction"), &Tunables::m_matchedEnvelopeMinimumMonotonicFraction},
            {QStringLiteral("curvatureMinimumDurationS"), &Tunables::m_curvatureMinimumDurationS},
            {QStringLiteral("curvatureMinimumR2"), &Tunables::m_curvatureMinimumR2},
            {QStringLiteral("curvatureMinimumR2Improvement"), &Tunables::m_curvatureMinimumR2Improvement},
            {QStringLiteral("curvatureMinimumHzPerS2"), &Tunables::m_curvatureMinimumHzPerS2},
            {QStringLiteral("rescueMinimumScoreMargin"), &Tunables::m_rescueMinimumScoreMargin},
            {QStringLiteral("rescueMinimumContrastDB"), &Tunables::m_rescueMinimumContrastDB},
            {QStringLiteral("rescueMinimumPeakDB"), &Tunables::m_rescueMinimumPeakDB},
            {QStringLiteral("rescueMinimumFrequencyCoherence"), &Tunables::m_rescueMinimumFrequencyCoherence},
            {QStringLiteral("rescueMinimumMatchedEnvelopeScore"), &Tunables::m_rescueMinimumMatchedEnvelopeScore},
            {QStringLiteral("rescueTwoFrameMinimumContrastDB"), &Tunables::m_rescueTwoFrameMinimumContrastDB},
            {QStringLiteral("rescueTwoFrameMinimumPeakDB"), &Tunables::m_rescueTwoFrameMinimumPeakDB},
            {QStringLiteral("rescueTwoFrameMinimumIntegratedSupportDB"), &Tunables::m_rescueTwoFrameMinimumIntegratedSupportDB},
            {QStringLiteral("rescueTwoFrameMinimumFrequencyCoherence"), &Tunables::m_rescueTwoFrameMinimumFrequencyCoherence},
            {QStringLiteral("rescueTwoFrameMinimumMatchedEnvelopeScore"), &Tunables::m_rescueTwoFrameMinimumMatchedEnvelopeScore},
            {QStringLiteral("rescueThreeFrameMinimumIntegratedSupportDB"), &Tunables::m_rescueThreeFrameMinimumIntegratedSupportDB},
            {QStringLiteral("rescueMaximumOccupiedFraction"), &Tunables::m_rescueMaximumOccupiedFraction},
            {QStringLiteral("compactThreeFrameEvidenceMaximumDurationS"), &Tunables::m_compactThreeFrameEvidenceMaximumDurationS},
            {QStringLiteral("compactThreeFrameEvidenceMinimumScoreMargin"), &Tunables::m_compactThreeFrameEvidenceMinimumScoreMargin},
            {QStringLiteral("compactThreeFrameEvidenceMinimumContrastDB"), &Tunables::m_compactThreeFrameEvidenceMinimumContrastDB},
            {QStringLiteral("compactThreeFrameEvidenceMinimumIntegratedSupportDB"), &Tunables::m_compactThreeFrameEvidenceMinimumIntegratedSupportDB},
            {QStringLiteral("compactThreeFrameEvidenceMinimumMatchedEnvelopeScore"), &Tunables::m_compactThreeFrameEvidenceMinimumMatchedEnvelopeScore},
            {QStringLiteral("compactThreeFrameEvidenceMaximumOccupiedFraction"), &Tunables::m_compactThreeFrameEvidenceMaximumOccupiedFraction},
            {QStringLiteral("compactThreeFrameEvidenceMaximumSweepScore"), &Tunables::m_compactThreeFrameEvidenceMaximumSweepScore},
            {QStringLiteral("compactThreeFrameEvidenceMaximumBandwidthHz"), &Tunables::m_compactThreeFrameEvidenceMaximumBandwidthHz},
            {QStringLiteral("narrowThreeFrameEvidenceMinimumScoreMargin"), &Tunables::m_narrowThreeFrameEvidenceMinimumScoreMargin},
            {QStringLiteral("narrowThreeFrameEvidenceMinimumContrastDB"), &Tunables::m_narrowThreeFrameEvidenceMinimumContrastDB},
            {QStringLiteral("narrowThreeFrameEvidenceMinimumIntegratedSupportDB"), &Tunables::m_narrowThreeFrameEvidenceMinimumIntegratedSupportDB},
            {QStringLiteral("narrowThreeFrameEvidenceMinimumMatchedEnvelopeScore"), &Tunables::m_narrowThreeFrameEvidenceMinimumMatchedEnvelopeScore},
            {QStringLiteral("narrowThreeFrameEvidenceMaximumOccupiedFraction"), &Tunables::m_narrowThreeFrameEvidenceMaximumOccupiedFraction},
            {QStringLiteral("narrowThreeFrameEvidenceMaximumBandwidthHz"), &Tunables::m_narrowThreeFrameEvidenceMaximumBandwidthHz},
            {QStringLiteral("sustainedEvidenceMaximumDurationS"), &Tunables::m_sustainedEvidenceMaximumDurationS},
            {QStringLiteral("sustainedEvidenceMinimumScoreMargin"), &Tunables::m_sustainedEvidenceMinimumScoreMargin},
            {QStringLiteral("sustainedEvidenceMinimumContrastDB"), &Tunables::m_sustainedEvidenceMinimumContrastDB},
            {QStringLiteral("sustainedEvidenceMinimumPeakDB"), &Tunables::m_sustainedEvidenceMinimumPeakDB},
            {QStringLiteral("sustainedEvidenceMinimumIntegratedSupportDB"), &Tunables::m_sustainedEvidenceMinimumIntegratedSupportDB},
            {QStringLiteral("sustainedEvidenceMinimumFrequencyCoherence"), &Tunables::m_sustainedEvidenceMinimumFrequencyCoherence},
            {QStringLiteral("sustainedEvidenceMaximumOccupiedFraction"), &Tunables::m_sustainedEvidenceMaximumOccupiedFraction},
            {QStringLiteral("sustainedEvidenceMaximumBandwidthHz"), &Tunables::m_sustainedEvidenceMaximumBandwidthHz},
            {QStringLiteral("sustainedEvidenceMinimumMatchedEnvelopeScore"), &Tunables::m_sustainedEvidenceMinimumMatchedEnvelopeScore},
            {QStringLiteral("sustainedEvidenceMaximumSweepScore"), &Tunables::m_sustainedEvidenceMaximumSweepScore},
            {QStringLiteral("sustainedEvidenceMaximumQuadraticSweepR2"), &Tunables::m_sustainedEvidenceMaximumQuadraticSweepR2},
            {QStringLiteral("learnedModelIntercept"), &Tunables::m_learnedModelIntercept},
            {QStringLiteral("learnedRescueProbability"), &Tunables::m_learnedRescueProbability},
            {QStringLiteral("rejectedCandidateReanalysisDelayS"), &Tunables::m_rejectedCandidateReanalysisDelayS},
            {QStringLiteral("rejectedCandidateMinimumScoreMargin"), &Tunables::m_rejectedCandidateMinimumScoreMargin},
            {QStringLiteral("rejectedCandidateMinimumFrequencyCoherence"), &Tunables::m_rejectedCandidateMinimumFrequencyCoherence},
            {QStringLiteral("rejectedCandidateMaximumOccupiedFraction"), &Tunables::m_rejectedCandidateMaximumOccupiedFraction},
            {QStringLiteral("rejectedTwoFrameMinimumBandwidthHz"), &Tunables::m_rejectedTwoFrameMinimumBandwidthHz},
            {QStringLiteral("rejectedTwoFrameStrongNarrowMinimumBandwidthHz"), &Tunables::m_rejectedTwoFrameStrongNarrowMinimumBandwidthHz},
            {QStringLiteral("rejectedTwoFrameStrongNarrowMinimumPeakDB"), &Tunables::m_rejectedTwoFrameStrongNarrowMinimumPeakDB},
            {QStringLiteral("rejectedTwoFrameStrongNarrowMinimumScoreMargin"), &Tunables::m_rejectedTwoFrameStrongNarrowMinimumScoreMargin},
            {QStringLiteral("rejectedStrongTwoFrameMaximumDurationS"), &Tunables::m_rejectedStrongTwoFrameMaximumDurationS},
            {QStringLiteral("rejectedStrongTwoFrameMinimumScoreMargin"), &Tunables::m_rejectedStrongTwoFrameMinimumScoreMargin},
            {QStringLiteral("rejectedStrongTwoFrameMinimumPeakDB"), &Tunables::m_rejectedStrongTwoFrameMinimumPeakDB},
            {QStringLiteral("rejectedStrongTwoFrameMinimumIntegratedSupportDB"), &Tunables::m_rejectedStrongTwoFrameMinimumIntegratedSupportDB},
            {QStringLiteral("rejectedStrongTwoFrameMinimumContrastDB"), &Tunables::m_rejectedStrongTwoFrameMinimumContrastDB},
            {QStringLiteral("rejectedStrongTwoFrameMinimumFrequencyCoherence"), &Tunables::m_rejectedStrongTwoFrameMinimumFrequencyCoherence},
            {QStringLiteral("rejectedStrongTwoFrameMinimumOccupiedFraction"), &Tunables::m_rejectedStrongTwoFrameMinimumOccupiedFraction},
            {QStringLiteral("rejectedStrongTwoFrameMaximumOccupiedFraction"), &Tunables::m_rejectedStrongTwoFrameMaximumOccupiedFraction},
            {QStringLiteral("rejectedStrongTwoFrameMaximumBandwidthHz"), &Tunables::m_rejectedStrongTwoFrameMaximumBandwidthHz},
            {QStringLiteral("rejectedStrongTwoFrameMaximumSweepScore"), &Tunables::m_rejectedStrongTwoFrameMaximumSweepScore},
            {QStringLiteral("rejectedStrongTwoFrameReanalysisDelayS"), &Tunables::m_rejectedStrongTwoFrameReanalysisDelayS},
            {QStringLiteral("rejectedCandidateMinimumExpansionS"), &Tunables::m_rejectedCandidateMinimumExpansionS},
            {QStringLiteral("rejectedCandidateMinimumDurationS"), &Tunables::m_rejectedCandidateMinimumDurationS},
            {QStringLiteral("rejectedCandidateMinimumPersistentLineS"), &Tunables::m_rejectedCandidateMinimumPersistentLineS},
            {QStringLiteral("rejectedCandidateMinimumPersistentLineProminenceDB"), &Tunables::m_rejectedCandidateMinimumPersistentLineProminenceDB},
            {QStringLiteral("rejectedCandidateMaximumPersistentLineJumpHz"), &Tunables::m_rejectedCandidateMaximumPersistentLineJumpHz},
            {QStringLiteral("rejectedCandidateSweepSearchHalfWidthHz"), &Tunables::m_rejectedCandidateSweepSearchHalfWidthHz},
            {QStringLiteral("rejectedCandidateSweepMinimumProminenceDB"), &Tunables::m_rejectedCandidateSweepMinimumProminenceDB},
            {QStringLiteral("rejectedCandidateSweepTemporalSupportFraction"), &Tunables::m_rejectedCandidateSweepTemporalSupportFraction},
            {QStringLiteral("rejectedCandidateSweepMinimumDriftHz"), &Tunables::m_rejectedCandidateSweepMinimumDriftHz},
            {QStringLiteral("rejectedCandidateSweepMinimumR2"), &Tunables::m_rejectedCandidateSweepMinimumR2},
            {QStringLiteral("rejectedCandidateActiveEventOverlapFraction"), &Tunables::m_rejectedCandidateActiveEventOverlapFraction},
            {QStringLiteral("compactMeteorSweepMaximumDurationS"), &Tunables::m_compactMeteorSweepMaximumDurationS},
            {QStringLiteral("compactMeteorSweepMinimumFrequencyCoherence"), &Tunables::m_compactMeteorSweepMinimumFrequencyCoherence},
            {QStringLiteral("compactMeteorSweepMinimumContrastDB"), &Tunables::m_compactMeteorSweepMinimumContrastDB},
            {QStringLiteral("compactMeteorSweepMaximumOccupiedFraction"), &Tunables::m_compactMeteorSweepMaximumOccupiedFraction},
            {QStringLiteral("parentReanalysisMinimumDurationS"), &Tunables::m_parentReanalysisMinimumDurationS},
            {QStringLiteral("twoComponentRangeMinimumExtensionBins"), &Tunables::m_twoComponentRangeMinimumExtensionBins},
            {QStringLiteral("parentFrequencyLowQuantile"), &Tunables::m_parentFrequencyLowQuantile},
            {QStringLiteral("parentFrequencyHighQuantile"), &Tunables::m_parentFrequencyHighQuantile},
            {QStringLiteral("componentEnvelopeSupportFraction"), &Tunables::m_componentEnvelopeSupportFraction},
            {QStringLiteral("componentEnvelopeCompactDurationS"), &Tunables::m_componentEnvelopeCompactDurationS},
            {QStringLiteral("componentEnvelopeMaximumCompactExpansionS"), &Tunables::m_componentEnvelopeMaximumCompactExpansionS},
            {QStringLiteral("componentEnvelopeSustainedDurationS"), &Tunables::m_componentEnvelopeSustainedDurationS},
            {QStringLiteral("componentEnvelopeSustainedExpansionFraction"), &Tunables::m_componentEnvelopeSustainedExpansionFraction},
            {QStringLiteral("componentEnvelopeMinimumSustainedExpansionS"), &Tunables::m_componentEnvelopeMinimumSustainedExpansionS},
            {QStringLiteral("parentEnvelopeSearchPaddingS"), &Tunables::m_parentEnvelopeSearchPaddingS},
            {QStringLiteral("parentEnvelopeNoiseContextS"), &Tunables::m_parentEnvelopeNoiseContextS},
            {QStringLiteral("parentEnvelopeMaximumLeadS"), &Tunables::m_parentEnvelopeMaximumLeadS},
            {QStringLiteral("parentEnvelopeMaximumTrailS"), &Tunables::m_parentEnvelopeMaximumTrailS},
            {QStringLiteral("parentEnvelopeEnterFraction"), &Tunables::m_parentEnvelopeEnterFraction},
            {QStringLiteral("parentEnvelopeExitFraction"), &Tunables::m_parentEnvelopeExitFraction},
            {QStringLiteral("parentEnvelopeMinimumFloorRatio"), &Tunables::m_parentEnvelopeMinimumFloorRatio},
            {QStringLiteral("parentEnvelopeMaximumGapS"), &Tunables::m_parentEnvelopeMaximumGapS},
            {QStringLiteral("parentEnvelopeMinimumExpansionS"), &Tunables::m_parentEnvelopeMinimumExpansionS},
        };
        static const QMap<QString, int Tunables::*> intMembers = {
            {QStringLiteral("minimumNoiseBlockCount"), &Tunables::m_minimumNoiseBlockCount},
            {QStringLiteral("sustainedSweepMinFrames"), &Tunables::m_sustainedSweepMinFrames},
            {QStringLiteral("compactSweepMinFrames"), &Tunables::m_compactSweepMinFrames},
            {QStringLiteral("localizedBurstMaxFrames"), &Tunables::m_localizedBurstMaxFrames},
            {QStringLiteral("localizedBurstMinEnvelopeTailFrames"), &Tunables::m_localizedBurstMinEnvelopeTailFrames},
            {QStringLiteral("broadbandImpulseMaxFrames"), &Tunables::m_broadbandImpulseMaxFrames},
            {QStringLiteral("detachedRepeatMinimumFrames"), &Tunables::m_detachedRepeatMinimumFrames},
            {QStringLiteral("candidateDiagnosticMinimumFrames"), &Tunables::m_candidateDiagnosticMinimumFrames},
            {QStringLiteral("matchedEnvelopeMinimumFrames"), &Tunables::m_matchedEnvelopeMinimumFrames},
            {QStringLiteral("matchedEnvelopeMinimumTailFrames"), &Tunables::m_matchedEnvelopeMinimumTailFrames},
            {QStringLiteral("curvatureMinimumFrames"), &Tunables::m_curvatureMinimumFrames},
            {QStringLiteral("compactThreeFrameEvidenceMinimumTailFrames"), &Tunables::m_compactThreeFrameEvidenceMinimumTailFrames},
            {QStringLiteral("sustainedEvidenceMinimumFrames"), &Tunables::m_sustainedEvidenceMinimumFrames},
            {QStringLiteral("sustainedEvidenceMaximumFrames"), &Tunables::m_sustainedEvidenceMaximumFrames},
            {QStringLiteral("rejectedCandidateMaximumFrames"), &Tunables::m_rejectedCandidateMaximumFrames},
            {QStringLiteral("compactMeteorSweepMaximumFrames"), &Tunables::m_compactMeteorSweepMaximumFrames},
            {QStringLiteral("maxPendingCandidateReanalyses"), &Tunables::m_maxPendingCandidateReanalyses},
            {QStringLiteral("maxActiveMeteorEvents"), &Tunables::m_maxActiveMeteorEvents},
            {QStringLiteral("maxParentObservations"), &Tunables::m_maxParentObservations},
        };
        static const QMap<QString, bool Tunables::*> boolMembers = {
            {QStringLiteral("enableCurvatureSweepRejection"), &Tunables::m_enableCurvatureSweepRejection},
            {QStringLiteral("enableCalibratedRescue"), &Tunables::m_enableCalibratedRescue},
            {QStringLiteral("learnedModelEnabled"), &Tunables::m_learnedModelEnabled},
            {QStringLiteral("enableSettledParentReanalysis"), &Tunables::m_enableSettledParentReanalysis},
            {QStringLiteral("enableRejectedCandidateReanalysis"), &Tunables::m_enableRejectedCandidateReanalysis},
        };

        for (const QPair<QString, QString>& override : overrides)
        {
            const QString& name = override.first;
            const QString& text = override.second;
            bool ok = false;

            if (doubleMembers.contains(name))
            {
                const double value = text.toDouble(&ok);

                if (!ok)
                {
                    error = QString("Invalid value for tunable %1: %2").arg(name, text);
                    return false;
                }

                tunables.*doubleMembers.value(name) = value;
            }
            else if (intMembers.contains(name))
            {
                const int value = text.toInt(&ok);

                if (!ok)
                {
                    error = QString("Invalid value for tunable %1: %2").arg(name, text);
                    return false;
                }

                tunables.*intMembers.value(name) = value;
            }
            else if (boolMembers.contains(name))
            {
                if ((text == "1") || (text.compare("true", Qt::CaseInsensitive) == 0)) {
                    tunables.*boolMembers.value(name) = true;
                } else if ((text == "0") || (text.compare("false", Qt::CaseInsensitive) == 0)) {
                    tunables.*boolMembers.value(name) = false;
                } else {
                    error = QString("Invalid boolean for tunable %1: %2").arg(name, text);
                    return false;
                }
            }
            else
            {
                error = QString("Unknown tunable: %1").arg(name);
                return false;
            }
        }

        return true;
    }

    bool parseIntValue(
        const QString& name,
        const QString& text,
        int& value,
        QString& error)
    {
        bool ok = false;
        const int parsed = text.toInt(&ok);

        if (!ok)
        {
            error = QString("Invalid integer for --%1: %2").arg(name, text);
            return false;
        }

        value = parsed;
        return true;
    }

    bool parseFloatValue(
        const QString& name,
        const QString& text,
        float& value,
        QString& error)
    {
        bool ok = false;
        const float parsed = text.toFloat(&ok);

        if (!ok)
        {
            error = QString("Invalid number for --%1: %2").arg(name, text);
            return false;
        }

        value = parsed;
        return true;
    }

    bool readOptionValue(const QStringList& args, int& index, const QString& name, QString& value, QString& error)
    {
        const QString arg = args[index];
        const QString longName = "--" + name;
        const QString valuePrefix = longName + "=";

        if (arg == longName)
        {
            if ((index + 1) >= args.size())
            {
                error = QString("Missing value for --%1").arg(name);
                return false;
            }

            value = args[++index];
            return true;
        }

        if (arg.startsWith(valuePrefix))
        {
            value = arg.mid(valuePrefix.size());
            return true;
        }

        return false;
    }

    bool validateOptions(Options& options, QString& error)
    {
        // Validate tunable overrides up front: runWavFile must not fail after
        // constructing the baseband (early destruction hangs process teardown).
        MeteorDemodSink::DetectorTunables tunablesProbe;

        if (!applyTunableOverrides(tunablesProbe, options.tunableOverrides, error)) {
            return false;
        }

        if (!MeteorSettings::isSupportedSampleRate(options.settings.m_channelSampleRate))
        {
            error = QString("Invalid --channel-sample-rate %1; expected 100, 300, 1000, or 3000")
                .arg(options.settings.m_channelSampleRate);
            return false;
        }

        if (options.settings.m_powerLPFCutoff <= 0.0f)
        {
            error = "--power-lpf-cutoff must be greater than zero";
            return false;
        }

        if (options.settings.m_detectionThresholdDB <= 0.0f)
        {
            error = "--threshold-db must be greater than zero";
            return false;
        }

        if (options.settings.m_minDurationMS < 1)
        {
            error = "--min-duration-ms must be at least 1";
            return false;
        }

        if (options.settings.m_maxDurationMS < options.settings.m_minDurationMS)
        {
            error = "--max-duration-ms must be greater than or equal to --min-duration-ms";
            return false;
        }

        if (options.settings.m_maxFrequencyDrift < 0.0f)
        {
            error = "--max-frequency-drift must be zero or greater";
            return false;
        }

        if (options.chunkSamples < 1)
        {
            error = "--chunk-samples must be at least 1";
            return false;
        }

        if (options.tailMS < 0)
        {
            error = "--tail-ms must be zero or greater";
            return false;
        }

        if (options.expectCount < -1)
        {
            error = "--expect-count must be zero or greater";
            return false;
        }

        if (options.wavPath.isEmpty() && options.testDir.isEmpty())
        {
            error = "Missing --wav <file.wav> or --test-dir <directory> option";
            return false;
        }

        return true;
    }

    bool parseOptions(const QStringList& args, Options& options, QString& error)
    {
        for (int i = 1; i < args.size(); i++)
        {
            const QString arg = args[i];
            QString value;

            if ((arg == "--help") || (arg == "-h"))
            {
                options.showHelp = true;
                return true;
            }
            else if (arg == "--details")
            {
                options.details = true;
            }
            else if (readOptionValue(args, i, "wav", value, error))
            {
                options.wavPath = value;
            }
            else if (readOptionValue(args, i, "test-dir", value, error))
            {
                options.testDir = value;
            }
            else if (readOptionValue(args, i, "candidate-csv", value, error))
            {
                options.candidateCsvPath = value;
            }
            else if (readOptionValue(args, i, "candidate-labels", value, error))
            {
                options.candidateLabelsPath = value;
            }
            else if (readOptionValue(args, i, "candidate-capture-dir", value, error))
            {
                options.candidateCaptureDir = value;
            }
            else if (readOptionValue(args, i, "tunable", value, error))
            {
                const int separator = value.indexOf('=');

                if (separator <= 0)
                {
                    error = QString("Expected --tunable name=value, got: %1").arg(value);
                    return false;
                }

                options.tunableOverrides.append({
                    value.left(separator).trimmed(),
                    value.mid(separator + 1).trimmed()});
            }
            else if (readOptionValue(args, i, "channel-sample-rate", value, error))
            {
                if (!parseIntValue("channel-sample-rate", value, options.settings.m_channelSampleRate, error)) {
                    return false;
                }
            }
            else if (readOptionValue(args, i, "input-frequency-offset", value, error))
            {
                if (!parseIntValue("input-frequency-offset", value, options.settings.m_inputFrequencyOffset, error)) {
                    return false;
                }
            }
            else if (readOptionValue(args, i, "power-lpf-cutoff", value, error))
            {
                if (!parseFloatValue("power-lpf-cutoff", value, options.settings.m_powerLPFCutoff, error)) {
                    return false;
                }
            }
            else if (readOptionValue(args, i, "threshold-db", value, error))
            {
                if (!parseFloatValue("threshold-db", value, options.settings.m_detectionThresholdDB, error)) {
                    return false;
                }
            }
            else if (readOptionValue(args, i, "min-duration-ms", value, error))
            {
                if (!parseIntValue("min-duration-ms", value, options.settings.m_minDurationMS, error)) {
                    return false;
                }
            }
            else if (readOptionValue(args, i, "max-duration-ms", value, error))
            {
                if (!parseIntValue("max-duration-ms", value, options.settings.m_maxDurationMS, error)) {
                    return false;
                }
            }
            else if (readOptionValue(args, i, "max-frequency-drift", value, error))
            {
                if (!parseFloatValue("max-frequency-drift", value, options.settings.m_maxFrequencyDrift, error)) {
                    return false;
                }
            }
            else if (readOptionValue(args, i, "chunk-samples", value, error))
            {
                if (!parseIntValue("chunk-samples", value, options.chunkSamples, error)) {
                    return false;
                }
            }
            else if (readOptionValue(args, i, "tail-ms", value, error))
            {
                if (!parseIntValue("tail-ms", value, options.tailMS, error)) {
                    return false;
                }
            }
            else if (readOptionValue(args, i, "expect-count", value, error))
            {
                if (!parseIntValue("expect-count", value, options.expectCount, error)) {
                    return false;
                }
            }
            else if (!error.isEmpty())
            {
                return false;
            }
            else
            {
                error = QString("Unknown option: %1").arg(arg);
                return false;
            }
        }

        if (options.wavPath.isEmpty() && options.testDir.isEmpty())
        {
            const QString defaultTestDir = QString::fromUtf8(METEOR_TEST_DATA_DIR);

            if (!defaultTestDir.isEmpty()) {
                options.testDir = defaultTestDir;
            }
        }

        return validateOptions(options, error);
    }

    void printHelp(QTextStream& out)
    {
        out << "Usage: meteor_demod_sink_test [--wav <file.wav> | --test-dir <directory>] [options]\n\n";
        out << "Play stereo 16-bit I/Q WAV files through MeteorBaseband and count or validate meteor detections.\n\n";
        out << "Options:\n";
        out << "  -h, --help                         Show this help text.\n";
        out << "      --wav <file.wav>               Input SDRangel stereo 16-bit I/Q WAV file.\n";
        out << "      --test-dir <directory>         Directory of paired .wav and .csv regression fixtures.\n";
        out << "      --candidate-csv <file.csv>     Write every spectral candidate and rejection decision.\n";
        out << "      --candidate-labels <file.csv>  Apply time-only or time/frequency labels to candidate rows.\n";
        out << "      --candidate-capture-dir <dir>  Save plausible rejected candidates as stereo signed 16-bit IQ.\n";
        out << "      --tunable <name=value>         Override a DetectorTunables member (repeatable). Names drop the\n";
        out << "                                     m_ prefix, e.g. --tunable rescueMinimumScoreMargin=1.5.\n";
        out << "      --channel-sample-rate <rate>   Detector sample rate: 100, 300, 1000, or 3000 Hz.\n";
        out << "      --input-frequency-offset <hz>  Channel input frequency offset in Hz.\n";
        out << "      --power-lpf-cutoff <hz>        Power low-pass filter cutoff in Hz.\n";
        out << "      --threshold-db <db>            Detection threshold above noise floor in dB.\n";
        out << "      --min-duration-ms <ms>         Minimum pulse duration in milliseconds.\n";
        out << "      --max-duration-ms <ms>         Maximum pulse duration in milliseconds.\n";
        out << "      --max-frequency-drift <hz>     Maximum accepted frequency span or drift in Hz.\n";
        out << "      --chunk-samples <samples>      Number of input samples to feed per chunk.\n";
        out << "      --tail-ms <ms>                 Trailing silence in milliseconds after EOF.\n";
        out << "      --expect-count <count>         Expected detection count; mismatch returns nonzero.\n";
        out << "      --details                      Print each MsgMeteorDetected message.\n";
    }

    FixReal scaleWavSample(qint16 value)
    {
        qint64 scaled = qRound64((double) value * (double) SDR_RX_SCALEF / 32768.0);
        scaled = std::clamp<qint64>(
            scaled,
            (qint64) std::numeric_limits<FixReal>::min(),
            (qint64) std::numeric_limits<FixReal>::max()
        );
        return (FixReal) scaled;
    }

    void processEvents()
    {
        QCoreApplication::processEvents(QEventLoop::AllEvents);
    }

    void drainDetections(MessageQueue& queue, QVector<Detection>& detections)
    {
        Message* message;

        while ((message = queue.pop()) != nullptr)
        {
            if (MeteorDemodSink::MsgMeteorDetected::match(*message))
            {
                const auto& detection = (const MeteorDemodSink::MsgMeteorDetected&) *message;
                detections.push_back({
                    detection.getDateTimeUtc(),
                    detection.getDisplayDateTimeUtc().isValid()
                        ? detection.getDisplayDateTimeUtc()
                        : detection.getDateTimeUtc(),
                    detection.getPeakAmplitude(),
                    detection.getPeakPowerDB(),
                    detection.getBackgroundPowerDB(),
                    detection.getTotalPowerDB(),
                    detection.getDurationS(),
                    detection.getCenterFrequency(),
                    detection.getFrequencySpan(),
                    detection.getFrequencyDrift(),
                    detection.getSampleRate(),
                    detection.getStartSample(),
                    detection.getEndSample()
                });
            }

            delete message;
        }
    }

    bool feedSamples(MeteorBaseband& baseband, const SampleVector& samples, MessageQueue& outputQueue, QVector<Detection>& detections)
    {
        if (samples.empty()) {
            return true;
        }

        baseband.feed(samples.begin(), samples.end());
        processEvents();
        drainDetections(outputQueue, detections);
        return true;
    }

    void appendSyntheticSignal(
        SampleVector& samples,
        int sampleRate,
        double durationS,
        double amplitude,
        double startFrequency,
        double endFrequency,
        quint32& noiseState)
    {
        constexpr double pi = 3.14159265358979323846;
        const int count = std::max(1, (int) std::llround(durationS * (double) sampleRate));
        double phase = 2.0 * pi * startFrequency
            * (double) samples.size() / (double) sampleRate;

        for (int i = 0; i < count; i++)
        {
            noiseState = 1664525U * noiseState + 1013904223U;
            const double noiseReal = ((double) ((noiseState >> 16) & 0xffU) / 127.5 - 1.0) * 2.0;
            noiseState = 1664525U * noiseState + 1013904223U;
            const double noiseImag = ((double) ((noiseState >> 16) & 0xffU) / 127.5 - 1.0) * 2.0;
            const double fraction = count > 1 ? (double) i / (double) (count - 1) : 0.0;
            const double frequency = startFrequency + fraction * (endFrequency - startFrequency);
            phase += 2.0 * pi * frequency / (double) sampleRate;
            samples.emplace_back(
                (FixReal) std::llround(noiseReal + amplitude * std::cos(phase)),
                (FixReal) std::llround(noiseImag + amplitude * std::sin(phase)));
        }
    }

    bool runSyntheticScenario(
        const QString& name,
        const SampleVector& samples,
        int expectedCount,
        double minimumFirstDurationS,
        QTextStream& errorStream)
    {
        constexpr int sampleRate = 1000;
        MessageQueue outputQueue;
        MeteorBaseband baseband;
        MeteorSettings settings;
        QVector<Detection> detections;
        QVector<MeteorDemodSink::CandidateAudit> candidateAudits;
        SampleVector chunk;
        int diagnosticCaptureCount = 0;

        settings.m_channelSampleRate = sampleRate;
        settings.m_minDurationMS = 20;
        settings.m_maxDurationMS = 20000;
        baseband.setInactivityFlushEnabled(false);
        baseband.setFifoLabel("meteor_synthetic_test");
        baseband.setMessageQueueToGUI(&outputQueue);
        baseband.setCandidateAuditCallback(
            [&candidateAudits](const MeteorDemodSink::CandidateAudit& audit) {
                candidateAudits.push_back(audit);
            });
        baseband.setDiagnosticCaptureCallback(
            [&diagnosticCaptureCount](
                quint64,
                const MeteorDemodSink::DetectionRecord&,
                const ComplexVector& capturedSamples)
            {
                if (!capturedSamples.empty()) {
                    diagnosticCaptureCount++;
                }
            });
        baseband.startWork();
        baseband.getInputMessageQueue()->push(new DSPSignalNotification(sampleRate, 143050000));
        baseband.getInputMessageQueue()->push(
            MeteorBaseband::MsgConfigureMeteorBaseband::create(settings, QStringList(), true));
        processEvents();

        for (int offset = 0; offset < (int) samples.size(); offset += 127)
        {
            const int count = std::min(127, (int) samples.size() - offset);
            chunk.assign(samples.begin() + offset, samples.begin() + offset + count);
            feedSamples(baseband, chunk, outputQueue, detections);
        }

        chunk.assign(3000, Sample(0, 0));
        feedSamples(baseband, chunk, outputQueue, detections);
        processEvents();
        drainDetections(outputQueue, detections);
        baseband.stopWork();

        bool ok = detections.size() == expectedCount;

        if (diagnosticCaptureCount != detections.size())
        {
            errorStream << QString("Synthetic %1: expected %2 diagnostic captures, got %3\n")
                .arg(name)
                .arg(detections.size())
                .arg(diagnosticCaptureCount);
            ok = false;
        }

        if (!ok)
        {
            errorStream << QString("Synthetic %1: expected %2 detections, got %3\n")
                .arg(name)
                .arg(expectedCount)
                .arg(detections.size());

            for (int i = 0; i < detections.size(); i++)
            {
                errorStream << QString("  actual %1: start=%2 duration=%3 center=%4 span=%5 drift=%6\n")
                    .arg(i + 1)
                    .arg(detections[i].startSample)
                    .arg(detections[i].durationS, 0, 'f', 3)
                    .arg(detections[i].centerFrequency, 0, 'f', 1)
                    .arg(detections[i].frequencySpan, 0, 'f', 1)
                    .arg(detections[i].frequencyDrift, 0, 'f', 1);
            }
        }

        if ((minimumFirstDurationS > 0.0)
            && (detections.isEmpty() || (detections.first().durationS < minimumFirstDurationS)))
        {
            errorStream << QString("Synthetic %1: first duration was shorter than %2 s\n")
                .arg(name)
                .arg(minimumFirstDurationS, 0, 'f', 2);
            if (!detections.isEmpty()) {
                errorStream << QString("  actual first: start=%1 duration=%2 center=%3 span=%4\n")
                    .arg(detections.first().startSample)
                    .arg(detections.first().durationS, 0, 'f', 3)
                    .arg(detections.first().centerFrequency, 0, 'f', 1)
                    .arg(detections.first().frequencySpan, 0, 'f', 1);
            }
            ok = false;
        }

        if (candidateAudits.isEmpty())
        {
            errorStream << QString("Synthetic %1: no spectral candidates were audited\n").arg(name);
            ok = false;
        }

        for (const MeteorDemodSink::CandidateAudit& audit : candidateAudits)
        {
            const bool finite = std::isfinite(audit.m_minimumNoiseContrastDB)
                && std::isfinite(audit.m_noiseFloorDeltaDB)
                && std::isfinite(audit.m_matchedEnvelopeScore)
                && std::isfinite(audit.m_decayTimeConstantS)
                && std::isfinite(audit.m_envelopePeakPosition)
                && std::isfinite(audit.m_envelopeDecayDB)
                && std::isfinite(audit.m_envelopeMonotonicFraction)
                && std::isfinite(audit.m_quadraticSweepR2)
                && std::isfinite(audit.m_quadraticSweepImprovement)
                && std::isfinite(audit.m_quadraticCurvatureHzPerS2)
                && std::isfinite(audit.m_centerFrequencyRateFraction)
                && std::isfinite(audit.m_frequencySpanRateFraction)
                && std::isfinite(audit.m_frequencyDriftRateFraction)
                && std::isfinite(audit.m_maxBandwidthRateFraction);
            const bool bounded = (audit.m_matchedEnvelopeScore >= 0.0)
                && (audit.m_matchedEnvelopeScore <= 1.0)
                && (audit.m_envelopePeakPosition >= 0.0)
                && (audit.m_envelopePeakPosition <= 1.0)
                && (audit.m_envelopeMonotonicFraction >= 0.0)
                && (audit.m_envelopeMonotonicFraction <= 1.0)
                && (audit.m_envelopeTailFrames >= 0)
                && (audit.m_quadraticSweepR2 >= 0.0)
                && (audit.m_quadraticSweepR2 <= 1.0)
                && (audit.m_quadraticSweepImprovement >= 0.0)
                && (audit.m_quadraticSweepImprovement <= 1.0);

            if (!finite || !bounded)
            {
                errorStream << QString("Synthetic %1: candidate shadow features are invalid\n").arg(name);
                ok = false;
                break;
            }
        }

        return ok;
    }

    bool runSyntheticDetectorTests(QTextStream& errorStream)
    {
        constexpr int sampleRate = 1000;
        bool ok = true;
        quint32 noiseState = 0x31415926U;
        SampleVector samples;

        appendSyntheticSignal(samples, sampleRate, 2.0, 0.0, 0.0, 0.0, noiseState);
        appendSyntheticSignal(samples, sampleRate, 1.5, 70.0, 55.0, 55.0, noiseState);
        appendSyntheticSignal(samples, sampleRate, 0.18, 12.0, 55.0, 55.0, noiseState);
        appendSyntheticSignal(samples, sampleRate, 1.7, 58.0, 55.0, 58.0, noiseState);
        appendSyntheticSignal(samples, sampleRate, 2.0, 0.0, 0.0, 0.0, noiseState);
        ok = runSyntheticScenario("faded-long-trail", samples, 1, 2.5, errorStream) && ok;

        samples.clear();
        appendSyntheticSignal(samples, sampleRate, 2.0, 0.0, 0.0, 0.0, noiseState);
        appendSyntheticSignal(samples, sampleRate, 0.6, 65.0, -150.0, -150.0, noiseState);
        appendSyntheticSignal(samples, sampleRate, 3.0, 0.0, 0.0, 0.0, noiseState);
        appendSyntheticSignal(samples, sampleRate, 0.6, 65.0, 150.0, 150.0, noiseState);
        appendSyntheticSignal(samples, sampleRate, 2.0, 0.0, 0.0, 0.0, noiseState);
        ok = runSyntheticScenario("separate-echoes", samples, 2, 0.0, errorStream) && ok;

        samples.clear();
        appendSyntheticSignal(samples, sampleRate, 2.0, 0.0, 0.0, 0.0, noiseState);
        appendSyntheticSignal(samples, sampleRate, 3.0, 60.0, -220.0, 220.0, noiseState);
        appendSyntheticSignal(samples, sampleRate, 2.0, 0.0, 0.0, 0.0, noiseState);
        ok = runSyntheticScenario("smooth-sweep", samples, 0, 0.0, errorStream) && ok;

        bool truncated = false;
        const quint64 clippedEnd = MeteorDemodSink::clipDetectionEndSample(2000, 23999, 20000, truncated);

        if (!truncated || (clippedEnd != 21999))
        {
            errorStream << "Synthetic max-duration: clipping boundary is incorrect\n";
            ok = false;
        }

        return ok;
    }

    bool readWavChunk(QFile& file, int maxSamples, SampleVector& samples, qint64& remainingBytes, QString& error)
    {
        const int bytesPerSample = 2 * (int) sizeof(qint16);
        const qint64 framesToRead = std::min<qint64>(maxSamples, remainingBytes / bytesPerSample);

        samples.clear();

        if (framesToRead <= 0) {
            return true;
        }

        const qint64 bytesToRead = framesToRead * bytesPerSample;
        const QByteArray bytes = file.read(bytesToRead);

        if (bytes.size() != bytesToRead)
        {
            error = QString("Short read from WAV data: expected %1 bytes, got %2").arg(bytesToRead).arg(bytes.size());
            return false;
        }

        samples.resize((int) framesToRead);
        const uchar *data = reinterpret_cast<const uchar*>(bytes.constData());

        for (int i = 0; i < (int) framesToRead; i++)
        {
            const qint16 real = qFromLittleEndian<qint16>(data + (i * bytesPerSample));
            const qint16 imag = qFromLittleEndian<qint16>(data + (i * bytesPerSample) + sizeof(qint16));
            samples[i].setReal(scaleWavSample(real));
            samples[i].setImag(scaleWavSample(imag));
        }

        remainingBytes -= bytesToRead;
        return true;
    }

    qint64 getCenterFrequency(const QString& wavPath, const WavFileRecord::Header& header)
    {
        if (header.m_auxiHeader.m_size > 0) {
            return header.m_auxi.m_centerFreq;
        }

        quint64 centerFrequency = 0;
        WavFileRecord::getCenterFrequency(wavPath, centerFrequency);
        return (qint64) centerFrequency;
    }

    void printDetails(QTextStream& out, const QVector<Detection>& detections)
    {
        for (int i = 0; i < detections.size(); i++)
        {
            const Detection& detection = detections[i];
            out << QString("Detection %1: timeUtc=%2 displayTimeUtc=%3 peakAmplitude=%4 peakPowerDB=%5 backgroundPowerDB=%6 totalPowerDB=%7 durationS=%8 centerFrequencyHz=%9 frequencySpanHz=%10 frequencyDriftHz=%11 sampleRate=%12 startSample=%13 endSample=%14\n")
                .arg(i + 1)
                .arg(detection.dateTimeUtc.toString(Qt::ISODateWithMs))
                .arg(detection.displayDateTimeUtc.toString(Qt::ISODateWithMs))
                .arg(detection.peakAmplitude, 0, 'f', 6)
                .arg(detection.peakPowerDB, 0, 'f', 2)
                .arg(detection.backgroundPowerDB, 0, 'f', 2)
                .arg(detection.totalPowerDB, 0, 'f', 2)
                .arg(detection.durationS, 0, 'f', 6)
                .arg(detection.centerFrequency, 0, 'f', 2)
                .arg(detection.frequencySpan, 0, 'f', 2)
                .arg(detection.frequencyDrift, 0, 'f', 2)
                .arg(detection.sampleRate)
                .arg(detection.startSample)
                .arg(detection.endSample);
        }
    }

    QString csvField(const QString& value)
    {
        QString escaped = value;
        escaped.replace('"', "\"\"");
        return '"' + escaped + '"';
    }

    bool loadCandidateLabels(const QString& path, QVector<CandidateLabel>& labels, QString& error)
    {
        labels.clear();

        if (path.isEmpty()) {
            return true;
        }

        QFile file(path);

        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            error = QString("Unable to read candidate labels CSV: %1").arg(path);
            return false;
        }

        QTextStream in(&file);
        bool firstLine = true;

        while (!in.atEnd())
        {
            const QString line = in.readLine().trimmed();

            if (line.isEmpty() || line.startsWith('#')) {
                continue;
            }

            const QStringList fields = line.split(',');

            bool firstValueNumeric = false;
            fields.value(0).trimmed().toULongLong(&firstValueNumeric);

            if (firstLine && !firstValueNumeric)
            {
                firstLine = false;
                continue;
            }

            firstLine = false;

            if (fields.size() < 3)
            {
                error = QString("Invalid candidate label row: %1").arg(line);
                return false;
            }

            bool startOK = false;
            bool endOK = false;
            CandidateLabel label;
            label.startSample = fields[0].trimmed().toULongLong(&startOK);
            label.endSample = fields[1].trimmed().toULongLong(&endOK);
            bool lowFrequencyOK = false;
            bool highFrequencyOK = false;

            if (fields.size() >= 5)
            {
                label.lowFrequency = fields[2].trimmed().toDouble(&lowFrequencyOK);
                label.highFrequency = fields[3].trimmed().toDouble(&highFrequencyOK);
            }

            if (lowFrequencyOK && highFrequencyOK)
            {
                label.hasFrequencyRange = true;
                label.label = fields[4].trimmed();
                label.eventId = fields.mid(5).join(',').trimmed();
            }
            else {
                label.label = fields.mid(2).join(',').trimmed();
            }

            if (!startOK || !endOK || (label.endSample < label.startSample) || label.label.isEmpty())
            {
                error = QString("Invalid candidate label row: %1").arg(line);
                return false;
            }

            if (label.hasFrequencyRange && (label.highFrequency < label.lowFrequency))
            {
                error = QString("Invalid candidate label frequency range: %1").arg(line);
                return false;
            }

            labels.push_back(label);
        }

        return true;
    }

    CandidateLabelMatch candidateLabelFor(
        const MeteorDemodSink::CandidateAudit& audit,
        const QVector<CandidateLabel>& labels)
    {
        CandidateLabelMatch bestMatch;
        quint64 bestOverlap = 0;
        const double candidateLow = audit.m_robustCenterFrequency - 0.5 * audit.m_robustFrequencySpan;
        const double candidateHigh = audit.m_robustCenterFrequency + 0.5 * audit.m_robustFrequencySpan;

        for (const CandidateLabel& label : labels)
        {
            const quint64 overlapStart = std::max(audit.m_startSample, label.startSample);
            const quint64 overlapEnd = std::min(audit.m_endSample, label.endSample);

            if (overlapEnd < overlapStart) {
                continue;
            }

            if (label.hasFrequencyRange
                && ((candidateHigh < label.lowFrequency) || (candidateLow > label.highFrequency)))
            {
                continue;
            }

            const quint64 overlap = overlapEnd - overlapStart + 1;

            if (overlap > bestOverlap)
            {
                bestOverlap = overlap;
                bestMatch.label = label.label;
                bestMatch.eventId = label.eventId;
            }
        }

        return bestMatch;
    }

    QString candidateCaptureFileName(const MeteorDemodSink::SpectralCandidate& candidate)
    {
        QString reason = candidate.m_rejectionReason;
        reason.replace(QRegularExpression("[^A-Za-z0-9_-]"), "_");
        return QString("candidate_%1_%2_%3.ci16")
            .arg(candidate.m_startSample)
            .arg(candidate.m_endSample)
            .arg(reason);
    }

    bool writeCandidateIQ(const QString& path, const ComplexVector& samples)
    {
        QFile file(path);

        if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
            return false;
        }

        QByteArray bytes;
        bytes.resize((int) samples.size() * 2 * (int) sizeof(qint16));
        qint16 *output = reinterpret_cast<qint16 *>(bytes.data());

        for (int i = 0; i < (int) samples.size(); i++)
        {
            const qint64 real = qRound64((double) samples[i].real() * 32768.0 / (double) SDR_RX_SCALEF);
            const qint64 imag = qRound64((double) samples[i].imag() * 32768.0 / (double) SDR_RX_SCALEF);
            output[2 * i] = qToLittleEndian((qint16) std::clamp<qint64>(real, -32768, 32767));
            output[2 * i + 1] = qToLittleEndian((qint16) std::clamp<qint64>(imag, -32768, 32767));
        }

        return file.write(bytes) == bytes.size();
    }

    bool writeCandidateAudits(
        const QString& path,
        const QVector<MeteorDemodSink::CandidateAudit>& audits,
        const QString& recordingId,
        const QVector<CandidateLabel>& labels,
        const QString& captureDir,
        const QSet<QString>& capturedFiles,
        QString& error)
    {
        QFile file(path);

        if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
        {
            error = QString("Unable to write candidate CSV: %1").arg(path);
            return false;
        }

        QTextStream out(&file);
        out << "recording,label,labelEventId,captureFile,index,sampleRate,startSample,endSample,peakSample,durationS,centerFrequencyHz,frequencySpanHz,frequencyDriftHz,"
               "peakAboveBackgroundDB,integratedSupportDB,maxBandwidthHz,maxContrastDB,logPeakRatio,sweepScore,acceptanceScore,"
               "acceptanceThreshold,scoreMargin,signalScore,supportScore,shapeScore,rejectionPenalty,trackOccupancy,"
               "minimumNoiseContrastDB,noiseFloorDeltaDB,matchedEnvelopeScore,decayTimeConstantS,"
               "envelopePeakPosition,envelopeDecayDB,envelopeMonotonicFraction,envelopeTailFrames,quadraticSweepR2,"
               "quadraticSweepImprovement,quadraticCurvatureHzPerS2,learnedScore,centerFrequencyRateFraction,"
               "frequencySpanRateFraction,frequencyDriftRateFraction,maxBandwidthRateFraction,calibratedRescue,"
               "rescuedFramesGate,rescuedSpectralEvidenceGate,curvedSweepRejected,"
               "frequencyCoherence,frameOccupiedFraction,frameCount,durationOK,enoughFrames,sweepRejected,spectralEvidenceOK,"
               "insideUsableBandwidth,duplicate,broadbandImpulse,sweepContinuationRejected,accepted,"
               "parentEventId,associationDecision,classification,rejectionReason\n";

        for (int i = 0; i < audits.size(); i++)
        {
            const MeteorDemodSink::CandidateAudit& audit = audits[i];
            const QString captureFile = candidateCaptureFileName(audit);
            const CandidateLabelMatch labelMatch = candidateLabelFor(audit, labels);
            out << csvField(recordingId) << ','
                << csvField(labelMatch.label) << ','
                << csvField(labelMatch.eventId) << ','
                << csvField(!captureDir.isEmpty() && capturedFiles.contains(captureFile) ? captureFile : QString()) << ','
                << i + 1 << ','
                << audit.m_sampleRate << ','
                << audit.m_startSample << ','
                << audit.m_endSample << ','
                << audit.m_peakSample << ','
                << QString::number(audit.m_durationS, 'f', 6) << ','
                << QString::number(audit.m_robustCenterFrequency, 'f', 3) << ','
                << QString::number(audit.m_robustFrequencySpan, 'f', 3) << ','
                << QString::number(audit.m_robustFrequencyDrift, 'f', 3) << ','
                << QString::number(audit.m_peakAboveBackgroundDB, 'f', 3) << ','
                << QString::number(audit.m_integratedSupportDB, 'f', 3) << ','
                << QString::number(audit.m_maxBandwidth, 'f', 3) << ','
                << QString::number(audit.m_maxContrastDB, 'f', 3) << ','
                << QString::number(std::log10(std::max(audit.m_maxPeakRatio, 1.0)), 'f', 6) << ','
                << QString::number(audit.m_sweepScore, 'f', 6) << ','
                << QString::number(audit.m_acceptanceScore, 'f', 3) << ','
                << QString::number(audit.m_acceptanceThreshold, 'f', 3) << ','
                << QString::number(audit.m_scoreMargin, 'f', 3) << ','
                << QString::number(audit.m_signalScore, 'f', 3) << ','
                << QString::number(audit.m_supportScore, 'f', 3) << ','
                << QString::number(audit.m_shapeScore, 'f', 3) << ','
                << QString::number(audit.m_rejectionPenalty, 'f', 3) << ','
                << QString::number(audit.m_trackOccupancy, 'f', 3) << ','
                << QString::number(audit.m_minimumNoiseContrastDB, 'f', 3) << ','
                << QString::number(audit.m_noiseFloorDeltaDB, 'f', 3) << ','
                << QString::number(audit.m_matchedEnvelopeScore, 'f', 6) << ','
                << QString::number(audit.m_decayTimeConstantS, 'f', 6) << ','
                << QString::number(audit.m_envelopePeakPosition, 'f', 6) << ','
                << QString::number(audit.m_envelopeDecayDB, 'f', 3) << ','
                << QString::number(audit.m_envelopeMonotonicFraction, 'f', 6) << ','
                << audit.m_envelopeTailFrames << ','
                << QString::number(audit.m_quadraticSweepR2, 'f', 6) << ','
                << QString::number(audit.m_quadraticSweepImprovement, 'f', 6) << ','
                << QString::number(audit.m_quadraticCurvatureHzPerS2, 'f', 3) << ','
                << QString::number(audit.m_learnedScore, 'f', 6) << ','
                << QString::number(audit.m_centerFrequencyRateFraction, 'f', 9) << ','
                << QString::number(audit.m_frequencySpanRateFraction, 'f', 9) << ','
                << QString::number(audit.m_frequencyDriftRateFraction, 'f', 9) << ','
                << QString::number(audit.m_maxBandwidthRateFraction, 'f', 9) << ','
                << (audit.m_calibratedRescue ? 1 : 0) << ','
                << (audit.m_rescuedFramesGate ? 1 : 0) << ','
                << (audit.m_rescuedSpectralEvidenceGate ? 1 : 0) << ','
                << (audit.m_curvedSweepRejected ? 1 : 0) << ','
                << QString::number(audit.m_frequencyCoherence, 'f', 3) << ','
                << QString::number(audit.m_frameOccupiedFraction, 'f', 3) << ','
                << audit.m_frameCount << ','
                << (audit.m_durationOK ? 1 : 0) << ','
                << (audit.m_enoughFrames ? 1 : 0) << ','
                << (audit.m_sweepRejected ? 1 : 0) << ','
                << (audit.m_spectralEvidenceOK ? 1 : 0) << ','
                << (audit.m_insideUsableBandwidth ? 1 : 0) << ','
                << (audit.m_duplicate ? 1 : 0) << ','
                << (audit.m_broadbandImpulse ? 1 : 0) << ','
                << (audit.m_sweepContinuationRejected ? 1 : 0) << ','
                << (audit.m_accepted ? 1 : 0) << ','
                << audit.m_parentEventId << ','
                << audit.m_associationDecision << ','
                << audit.m_classification << ','
                << audit.m_rejectionReason << '\n';
        }

        return true;
    }

    bool parseDoubleField(const QString& text, double& value)
    {
        bool ok = false;
        value = text.trimmed().toDouble(&ok);
        return ok && std::isfinite(value);
    }

    bool loadExpectedDetections(const QString& csvPath, QVector<ExpectedDetection>& detections, QString& error)
    {
        QFile file(csvPath);

        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            error = QString("Unable to open expectation CSV: %1").arg(csvPath);
            return false;
        }

        QTextStream in(&file);
        const QString header = in.readLine().trimmed();

        const QString legacyHeader =
            "index,timeOffsetS,durationS,centerFrequencyHz,frequencySpanHz,frequencyDriftHz,totalPowerDB";
        const QString optionalHeader = legacyHeader + ",required";
        const bool hasRequiredColumn = header == optionalHeader;

        if ((header != legacyHeader) && !hasRequiredColumn)
        {
            error = QString("Unexpected CSV header in %1").arg(csvPath);
            return false;
        }

        int row = 1;

        while (!in.atEnd())
        {
            row++;
            const QString line = in.readLine().trimmed();

            if (line.isEmpty()) {
                continue;
            }

            const QStringList fields = line.split(',');

            const int expectedFieldCount = hasRequiredColumn ? 8 : 7;

            if (fields.size() != expectedFieldCount)
            {
                error = QString("Expected %1 fields in %2 row %3")
                    .arg(expectedFieldCount)
                    .arg(csvPath)
                    .arg(row);
                return false;
            }

            bool indexOK = false;
            ExpectedDetection detection;
            detection.index = fields[0].trimmed().toInt(&indexOK);

            if (!indexOK
                || !parseDoubleField(fields[1], detection.timeOffsetS)
                || !parseDoubleField(fields[2], detection.durationS)
                || !parseDoubleField(fields[3], detection.centerFrequency)
                || !parseDoubleField(fields[4], detection.frequencySpan)
                || !parseDoubleField(fields[5], detection.frequencyDrift)
                || !parseDoubleField(fields[6], detection.totalPowerDB))
            {
                error = QString("Invalid numeric value in %1 row %2").arg(csvPath).arg(row);
                return false;
            }

            if (hasRequiredColumn)
            {
                const QString required = fields[7].trimmed().toLower();

                if ((required == "1") || (required == "true") || (required == "required")) {
                    detection.required = true;
                } else if ((required == "0") || (required == "false") || (required == "optional")) {
                    detection.required = false;
                } else {
                    error = QString("Invalid required value in %1 row %2").arg(csvPath).arg(row);
                    return false;
                }
            }

            detections.push_back(detection);
        }

        return true;
    }

    bool nearlyEqual(double actual, double expected, double tolerance)
    {
        return std::fabs(actual - expected) <= tolerance;
    }

    bool compareDetections(
        const QString& name,
        const QVector<Detection>& actual,
        const QVector<ExpectedDetection>& expected,
        QTextStream& err)
    {
        constexpr double timeToleranceS = 0.050;
        constexpr double durationToleranceS = 0.010;
        constexpr double frequencyToleranceHz = 1.0;
        constexpr double totalPowerToleranceDB = 0.5;
        bool ok = true;
        int requiredCount = 0;

        for (const ExpectedDetection& expectation : expected) {
            requiredCount += expectation.required ? 1 : 0;
        }

        if ((actual.size() < requiredCount) || (actual.size() > expected.size()))
        {
            err << QString("%1: expected %2 required and up to %3 optional detections, got %4\n")
                .arg(name)
                .arg(requiredCount)
                .arg(expected.size() - requiredCount)
                .arg(actual.size());
            ok = false;
        }

        if (expected.isEmpty()) {
            return ok;
        }

        auto compareDetection = [&](const Detection& detection, const ExpectedDetection& expectation)
        {
            const double timeOffsetS = (double) detection.startSample / (double) std::max(1, detection.sampleRate);

            if (!nearlyEqual(timeOffsetS, expectation.timeOffsetS, timeToleranceS))
            {
                err << QString("%1 detection %2: expected timeOffsetS %3, got %4\n")
                    .arg(name)
                    .arg(expectation.index)
                    .arg(expectation.timeOffsetS, 0, 'f', 3)
                    .arg(timeOffsetS, 0, 'f', 3);
                ok = false;
            }

            if (!nearlyEqual(detection.durationS, expectation.durationS, durationToleranceS))
            {
                err << QString("%1 detection %2: expected durationS %3, got %4\n")
                    .arg(name)
                    .arg(expectation.index)
                    .arg(expectation.durationS, 0, 'f', 6)
                    .arg(detection.durationS, 0, 'f', 6);
                ok = false;
            }

            if (!nearlyEqual(detection.centerFrequency, expectation.centerFrequency, frequencyToleranceHz))
            {
                err << QString("%1 detection %2: expected centerFrequencyHz %3, got %4\n")
                    .arg(name)
                    .arg(expectation.index)
                    .arg(expectation.centerFrequency, 0, 'f', 2)
                    .arg(detection.centerFrequency, 0, 'f', 2);
                ok = false;
            }

            if (!nearlyEqual(detection.frequencySpan, expectation.frequencySpan, frequencyToleranceHz))
            {
                err << QString("%1 detection %2: expected frequencySpanHz %3, got %4\n")
                    .arg(name)
                    .arg(expectation.index)
                    .arg(expectation.frequencySpan, 0, 'f', 2)
                    .arg(detection.frequencySpan, 0, 'f', 2);
                ok = false;
            }

            if (!nearlyEqual(detection.frequencyDrift, expectation.frequencyDrift, frequencyToleranceHz))
            {
                err << QString("%1 detection %2: expected frequencyDriftHz %3, got %4\n")
                    .arg(name)
                    .arg(expectation.index)
                    .arg(expectation.frequencyDrift, 0, 'f', 2)
                    .arg(detection.frequencyDrift, 0, 'f', 2);
                ok = false;
            }

            if (!nearlyEqual(detection.totalPowerDB, expectation.totalPowerDB, totalPowerToleranceDB))
            {
                err << QString("%1 detection %2: expected totalPowerDB %3, got %4\n")
                    .arg(name)
                    .arg(expectation.index)
                    .arg(expectation.totalPowerDB, 0, 'f', 2)
                    .arg(detection.totalPowerDB, 0, 'f', 2);
                ok = false;
            }
        };

        int actualIndex = 0;
        int expectedIndex = 0;

        while ((actualIndex < actual.size()) || (expectedIndex < expected.size()))
        {
            if (expectedIndex >= expected.size())
            {
                const Detection& detection = actual[actualIndex++];
                const double timeOffsetS = (double) detection.startSample
                    / (double) std::max(1, detection.sampleRate);
                err << QString("%1: unexpected detection at %2 s\n")
                    .arg(name)
                    .arg(timeOffsetS, 0, 'f', 3);
                ok = false;
                continue;
            }

            const ExpectedDetection& expectation = expected[expectedIndex];

            if (actualIndex >= actual.size())
            {
                if (expectation.required)
                {
                    err << QString("%1: missing required detection %2 at %3 s\n")
                        .arg(name)
                        .arg(expectation.index)
                        .arg(expectation.timeOffsetS, 0, 'f', 3);
                    ok = false;
                }

                expectedIndex++;
                continue;
            }

            const Detection& detection = actual[actualIndex];
            const double actualTimeOffsetS = (double) detection.startSample
                / (double) std::max(1, detection.sampleRate);

            if (nearlyEqual(actualTimeOffsetS, expectation.timeOffsetS, timeToleranceS))
            {
                compareDetection(detection, expectation);
                actualIndex++;
                expectedIndex++;
            }
            else if (!expectation.required && (actualTimeOffsetS > expectation.timeOffsetS))
            {
                expectedIndex++;
            }
            else if (actualTimeOffsetS < expectation.timeOffsetS)
            {
                err << QString("%1: unexpected detection at %2 s before expected detection %3\n")
                    .arg(name)
                    .arg(actualTimeOffsetS, 0, 'f', 3)
                    .arg(expectation.index);
                ok = false;
                actualIndex++;
            }
            else
            {
                err << QString("%1: missing required detection %2 at %3 s\n")
                    .arg(name)
                    .arg(expectation.index)
                    .arg(expectation.timeOffsetS, 0, 'f', 3);
                ok = false;
                expectedIndex++;
            }
        }

        return ok;
    }

    bool runWavFile(
        const Options& options,
        const QString& wavPath,
        QVector<Detection>& detections,
        QString& error,
        QVector<MeteorDemodSink::CandidateAudit> *candidateAudits = nullptr,
        QSet<QString> *capturedFiles = nullptr)
    {
        QFile wavFile(wavPath);

        if (!wavFile.open(QIODevice::ReadOnly))
        {
            error = QString("Unable to open WAV file: %1").arg(wavPath);
            return false;
        }

        WavFileRecord::Header header;

        if (!WavFileRecord::readHeader(wavFile, header))
        {
            error = "Unsupported WAV file. Expected stereo 16-bit PCM I/Q WAV data.";
            return false;
        }

        const qint64 dataStart = wavFile.pos();
        const qint64 availableDataBytes = std::max<qint64>(0, wavFile.size() - dataStart);
        qint64 remainingBytes = header.m_dataHeader.m_size > 0
            ? std::min<qint64>(header.m_dataHeader.m_size, availableDataBytes)
            : availableDataBytes;

        if ((header.m_sampleRate <= 0) || (remainingBytes < 0))
        {
            error = "Invalid WAV header.";
            return false;
        }

        if ((remainingBytes % (2 * (int) sizeof(qint16))) != 0) {
            remainingBytes -= remainingBytes % (2 * (int) sizeof(qint16));
        }

        const qint64 centerFrequency = getCenterFrequency(wavPath, header);
        MessageQueue outputQueue;
        MeteorBaseband baseband;
        SampleVector samples;
        bool candidateCaptureWriteFailed = false;

        baseband.setInactivityFlushEnabled(false);
        baseband.setFifoLabel("meteor_demod_sink_test");
        baseband.setMessageQueueToGUI(&outputQueue);

        if (!options.tunableOverrides.isEmpty())
        {
            MeteorDemodSink::DetectorTunables tunables = baseband.getDetectorTunables();

            if (!applyTunableOverrides(tunables, options.tunableOverrides, error)) {
                return false;
            }

            baseband.setDetectorTunables(tunables);
        }

        if (candidateAudits)
        {
            baseband.setCandidateAuditCallback(
                [candidateAudits](const MeteorDemodSink::CandidateAudit& audit) {
                    candidateAudits->push_back(audit);
                });
        }

        if (!options.candidateCaptureDir.isEmpty())
        {
            QDir captureDir;

            if (!captureDir.mkpath(options.candidateCaptureDir))
            {
                error = QString("Unable to create candidate capture directory: %1")
                    .arg(options.candidateCaptureDir);
                return false;
            }

            baseband.setCandidateDiagnosticCaptureCallback(
                [&options, &candidateCaptureWriteFailed, capturedFiles](
                    const MeteorDemodSink::SpectralCandidate& candidate,
                    const ComplexVector& candidateSamples) {
                    const QString fileName = candidateCaptureFileName(candidate);
                    const QString path = QDir(options.candidateCaptureDir).filePath(fileName);

                    if (writeCandidateIQ(path, candidateSamples)) {
                        if (capturedFiles) {
                            capturedFiles->insert(fileName);
                        }
                    } else {
                        candidateCaptureWriteFailed = true;
                    }
                });
        }

        baseband.startWork();
        baseband.getInputMessageQueue()->push(new DSPSignalNotification(header.m_sampleRate, centerFrequency));
        baseband.getInputMessageQueue()->push(MeteorBaseband::MsgConfigureMeteorBaseband::create(options.settings, QStringList(), true));
        processEvents();

        while (remainingBytes > 0)
        {
            if (!readWavChunk(wavFile, options.chunkSamples, samples, remainingBytes, error))
            {
                baseband.stopWork();
                return false;
            }

            feedSamples(baseband, samples, outputQueue, detections);
        }

        const qint64 tailSamples = ((qint64) header.m_sampleRate * options.tailMS) / 1000;
        qint64 tailSamplesRemaining = tailSamples;

        while (tailSamplesRemaining > 0)
        {
            const int count = (int) std::min<qint64>(options.chunkSamples, tailSamplesRemaining);
            samples.assign(count, Sample(0, 0));
            feedSamples(baseband, samples, outputQueue, detections);
            tailSamplesRemaining -= count;
        }

        processEvents();
        drainDetections(outputQueue, detections);
        baseband.stopWork();

        if (candidateCaptureWriteFailed)
        {
            error = QString("Unable to write one or more candidate IQ captures to: %1")
                .arg(options.candidateCaptureDir);
            return false;
        }

        return true;
    }

    bool runRegressionTests(const Options& options, QTextStream& out, QTextStream& err)
    {
        QDir testDir(options.testDir);

        if (!testDir.exists())
        {
            err << QString("Test directory does not exist: %1\n").arg(options.testDir);
            return false;
        }

        const QStringList csvFiles = testDir.entryList(QStringList() << "*.csv", QDir::Files, QDir::Name);

        if (csvFiles.isEmpty())
        {
            err << QString("No CSV regression fixtures found in %1\n").arg(options.testDir);
            return false;
        }

        bool allOK = true;

        for (const QString& csvFileName : csvFiles)
        {
            const QFileInfo csvInfo(testDir.filePath(csvFileName));
            const QString wavPath = testDir.filePath(csvInfo.completeBaseName() + ".wav");
            QVector<ExpectedDetection> expectedDetections;
            QVector<Detection> actualDetections;
            QString error;

            if (!QFileInfo::exists(wavPath))
            {
                err << QString("%1: missing WAV file %2\n").arg(csvFileName, wavPath);
                allOK = false;
                continue;
            }

            if (!loadExpectedDetections(csvInfo.filePath(), expectedDetections, error))
            {
                err << error << "\n";
                allOK = false;
                continue;
            }

            if (!runWavFile(options, wavPath, actualDetections, error))
            {
                err << QString("%1: %2\n").arg(csvFileName, error);
                allOK = false;
                continue;
            }

            const bool testOK = compareDetections(csvInfo.completeBaseName(), actualDetections, expectedDetections, err);
            out << QString("%1: %2 detections %3\n")
                .arg(csvInfo.completeBaseName())
                .arg(actualDetections.size())
                .arg(testOK ? "OK" : "FAILED");
            allOK = allOK && testOK;
        }

        return allOK;
    }
}

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName("meteor_demod_sink_test");
    QCoreApplication::setApplicationVersion("1.0");

    QTextStream out(stdout);
    QTextStream err(stderr);
    Options options;
    QString error;

    if (!parseOptions(app.arguments(), options, error))
    {
        err << error << "\n";
        err << "Use --help for usage.\n";
        return 1;
    }

    if (options.showHelp)
    {
        printHelp(out);
        return 0;
    }

    if (!runMeteorSettingsTests(err) || !runRMOBReportTests(err)) {
        return 2;
    }

    if (!runSyntheticDetectorTests(err)) {
        return 2;
    }

    if (!options.testDir.isEmpty() && options.wavPath.isEmpty())
    {
        return runRegressionTests(options, out, err) ? 0 : 2;
    }

    QVector<Detection> detections;
    QVector<MeteorDemodSink::CandidateAudit> candidateAudits;
    QVector<CandidateLabel> candidateLabels;
    QSet<QString> capturedFiles;

    if (!loadCandidateLabels(options.candidateLabelsPath, candidateLabels, error))
    {
        err << error << "\n";
        return 1;
    }

    if (!runWavFile(
        options,
        options.wavPath,
        detections,
        error,
        (options.candidateCsvPath.isEmpty() && options.candidateCaptureDir.isEmpty())
            ? nullptr
            : &candidateAudits,
        &capturedFiles))
    {
        err << error << "\n";
        return 1;
    }

    out << QString("Meteor detections: %1\n").arg(detections.size());

    if (options.details) {
        printDetails(out, detections);
    }

    if (!options.candidateCsvPath.isEmpty()
        && !writeCandidateAudits(
            options.candidateCsvPath,
            candidateAudits,
            QFileInfo(options.wavPath).fileName(),
            candidateLabels,
            options.candidateCaptureDir,
            capturedFiles,
            error))
    {
        err << error << "\n";
        return 1;
    }

    if ((options.expectCount >= 0) && (detections.size() != options.expectCount))
    {
        err << QString("Expected %1 detections, got %2\n").arg(options.expectCount).arg(detections.size());
        return 2;
    }

    return 0;
}
