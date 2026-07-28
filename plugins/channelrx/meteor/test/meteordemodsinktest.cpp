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

#include <functional>
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
#include "meteormapgeometry.h"
#include "meteorsatellitematcher.h"
#include "meteorsettings.h"
#include "movingtargetmatcher.h"
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
        expected.m_transmitterHPBW = 16.0f;
        expected.m_antennaAzimuth = 220.5f;
        expected.m_antennaElevation = 11.5f;
        expected.m_antennaBeamwidth = 30.0f;
        expected.m_rotator = QStringLiteral("2:3");
        expected.m_showAntennaPatterns = true;
        expected.m_mapMaxAltitudeKM = 750.0f;
        expected.m_detectionsTableColumnHidden = (1u << 2) | (1u << 9);

        MeteorSettings actual;

        if (!actual.deserialize(expected.serialize())
            || (actual.m_transmitterLatitude != expected.m_transmitterLatitude)
            || (actual.m_transmitterLongitude != expected.m_transmitterLongitude)
            || (actual.m_transmitterAzimuth != expected.m_transmitterAzimuth)
            || (actual.m_transmitterElevation != expected.m_transmitterElevation)
            || (actual.m_transmitterBeamwidth != expected.m_transmitterBeamwidth)
            || (actual.m_transmitterHPBW != expected.m_transmitterHPBW)
            || (actual.m_antennaAzimuth != expected.m_antennaAzimuth)
            || (actual.m_antennaElevation != expected.m_antennaElevation)
            || (actual.m_antennaBeamwidth != expected.m_antennaBeamwidth)
            || (actual.m_rotator != expected.m_rotator)
            || (actual.m_showAntennaPatterns != expected.m_showAntennaPatterns)
            || (actual.m_mapMaxAltitudeKM != expected.m_mapMaxAltitudeKM)
            || (actual.m_detectionsTableColumnHidden
                != expected.m_detectionsTableColumnHidden))
        {
            errorStream << "Meteor settings test: geometry settings did not round-trip\n";
            return false;
        }

        return true;
    }

    bool runMeteorMapGeometryTests(QTextStream& errorStream)
    {
        const double horizonDistance = MeteorMapGeometry::groundDistanceAtAltitude(0.0, 1000000.0);
        const double middleDistance = MeteorMapGeometry::groundDistanceAtAltitude(45.0, 1000000.0);
        const double zenithDistance = MeteorMapGeometry::groundDistanceAtAltitude(90.0, 1000000.0);

        if (!(horizonDistance > middleDistance)
            || !(middleDistance > zenithDistance)
            || (std::fabs(zenithDistance) > 0.001))
        {
            errorStream << "Meteor map geometry test: altitude intersection ranges are invalid\n";
            return false;
        }

        const std::vector<MeteorMapGeometry::Coordinate> footprint =
            MeteorMapGeometry::beamFootprint(47.348, 5.5151, 180.0, 27.5, 180.0, 25.0, 1000000.0);

        if (footprint.size() < 10)
        {
            errorStream << "Meteor map geometry test: GRAVES footprint is empty\n";
            return false;
        }

        for (const MeteorMapGeometry::Coordinate& coordinate : footprint)
        {
            if (!std::isfinite(coordinate.m_latitude)
                || !std::isfinite(coordinate.m_longitude)
                || (coordinate.m_latitude < -90.0)
                || (coordinate.m_latitude > 90.0)
                || (coordinate.m_longitude < -180.0)
                || (coordinate.m_longitude > 180.0)
                || (coordinate.m_altitudeM != 1000000.0))
            {
                errorStream << "Meteor map geometry test: footprint contains an invalid coordinate\n";
                return false;
            }
        }

        if (!MeteorMapGeometry::beamFootprint(
                47.348, 5.5151, 180.0, 27.5, 180.0, 0.0, 1000000.0).empty())
        {
            errorStream << "Meteor map geometry test: unspecified HPBW produced a footprint\n";
            return false;
        }

        const MeteorMapGeometry::BeamDefinition beam {
            47.348, 5.5151, 0.0, 180.0, 27.5, 60.0, 25.0, 1000000.0
        };
        const MeteorMapGeometry::Mesh volume = MeteorMapGeometry::beamVolume(beam);
        if (!volume.isValid() || (volume.m_footprint.size() < 10))
        {
            errorStream << "Meteor map geometry test: beam volume is invalid\n";
            return false;
        }

        const MeteorMapGeometry::Mesh identicalIntersection =
            MeteorMapGeometry::beamIntersection(beam, beam);
        if (!identicalIntersection.isValid() || identicalIntersection.m_footprint.empty())
        {
            errorStream << "Meteor map geometry test: identical beams have no volume intersection\n";
            return false;
        }

        MeteorMapGeometry::BeamDefinition oppositeBeam = beam;
        oppositeBeam.m_azimuthDegrees = 0.0;
        oppositeBeam.m_horizontalBeamwidthDegrees = 20.0;
        MeteorMapGeometry::BeamDefinition narrowBeam = beam;
        narrowBeam.m_horizontalBeamwidthDegrees = 20.0;
        if (MeteorMapGeometry::beamIntersection(narrowBeam, oppositeBeam).isValid())
        {
            errorStream << "Meteor map geometry test: opposing narrow beams unexpectedly intersect\n";
            return false;
        }

        return true;
    }

    bool runMovingTargetMatcherTests(QTextStream& errorStream)
    {
        const MeteorSatelliteMatcher::Beam zenithBeam {0.0, 90.0, 64.0, 64.0};
        if (!MeteorSatelliteMatcher::beamContainsLookDirection(0.0, 90.0, zenithBeam)
            || !MeteorSatelliteMatcher::beamContainsLookDirection(90.0, 70.0, zenithBeam)
            || !MeteorSatelliteMatcher::beamContainsLookDirection(270.0, 70.0, zenithBeam)
            || MeteorSatelliteMatcher::beamContainsLookDirection(90.0, 50.0, zenithBeam))
        {
            errorStream << "Moving-target matcher test: zenith beam geometry is incorrect\n";
            return false;
        }

        MovingTargetMatcher::Observation observation;
        observation.m_startDateTimeUtc = QDateTime(
            QDate(2026, 7, 22),
            QTime(12, 0),
            Qt::UTC);
        observation.m_durationS = 3.0;
        observation.m_frequencySpanHz = 120.0;
        observation.m_referenceFrequencyHz = 143050000.0;
        observation.m_transmitter = {47.3480, 5.5151, 0.0};
        observation.m_receiver = {50.0535, 19.8235, 0.0};

        MovingTargetMatcher::TargetState expectedTarget;
        expectedTarget.m_source = QStringLiteral("ADS-B");
        expectedTarget.m_id = QStringLiteral("ABC123");
        expectedTarget.m_label = QStringLiteral("TEST123");
        expectedTarget.m_dateTimeUtc = observation.m_startDateTimeUtc.addSecs(-5);
        expectedTarget.m_position = {49.0, 8.0, 10000.0};
        expectedTarget.m_eastVelocityMPS = 210.0;
        expectedTarget.m_northVelocityMPS = 45.0;
        expectedTarget.m_upVelocityMPS = 2.0;

        MovingTargetMatcher::TargetState horizonTarget;
        horizonTarget.m_dateTimeUtc = observation.m_startDateTimeUtc;
        horizonTarget.m_position = {0.0, 3.0, 11582.4};
        const MovingTargetMatcher::Site horizonObserver {0.0, 0.0, 0.0};
        const double aboveHorizonDegrees = MovingTargetMatcher::elevationDegrees(
            horizonObserver,
            horizonTarget,
            observation.m_startDateTimeUtc);
        horizonTarget.m_position.m_longitudeDegrees = 9.0;
        const double belowHorizonDegrees = MovingTargetMatcher::elevationDegrees(
            horizonObserver,
            horizonTarget,
            observation.m_startDateTimeUtc);

        if (!(aboveHorizonDegrees > 0.0) || !(belowHorizonDegrees < 0.0))
        {
            errorStream << "Moving-target matcher test: aircraft horizon geometry is incorrect\n";
            return false;
        }

        const MovingTargetMatcher::Prediction expectedPrediction =
            MovingTargetMatcher::predict(observation, expectedTarget);

        if (!expectedPrediction.m_valid)
        {
            errorStream << "Moving-target matcher test: valid state produced no prediction\n";
            return false;
        }

        observation.m_centerFrequencyOffsetHz = expectedPrediction.m_centerFrequencyOffsetHz;
        observation.m_frequencyDriftHz = expectedPrediction.m_frequencyDriftHz;
        MovingTargetMatcher::TargetState distractor = expectedTarget;
        distractor.m_id = QStringLiteral("DEF456");
        distractor.m_label = QStringLiteral("OTHER");
        distractor.m_eastVelocityMPS *= -1.0;
        distractor.m_northVelocityMPS *= -1.0;
        const MovingTargetMatcher::Match match = MovingTargetMatcher::match(
            observation,
            {distractor, expectedTarget});

        if (!match.m_matched
            || (match.m_id != expectedTarget.m_id)
            || (match.m_scorePercent < 99.9)
            || (match.m_endpointResidualRMSHz > 1e-6))
        {
            errorStream << "Moving-target matcher test: exact target was not selected\n";
            return false;
        }

        MovingTargetMatcher::TargetState duplicateTarget = expectedTarget;
        duplicateTarget.m_id = QStringLiteral("ABC124");
        MovingTargetMatcher::TargetState duplicateTarget2 = expectedTarget;
        duplicateTarget2.m_id = QStringLiteral("ABC125");
        MovingTargetMatcher::TargetState duplicateTarget3 = expectedTarget;
        duplicateTarget3.m_id = QStringLiteral("ABC126");
        MovingTargetMatcher::TargetState duplicateTarget4 = expectedTarget;
        duplicateTarget4.m_id = QStringLiteral("ABC127");
        const MovingTargetMatcher::Match ambiguousMatch = MovingTargetMatcher::match(
            observation,
            {
                expectedTarget,
                duplicateTarget,
                duplicateTarget2,
                duplicateTarget3,
                duplicateTarget4
            });

        if (!ambiguousMatch.m_ambiguous
            || ambiguousMatch.m_matched
            || (ambiguousMatch.m_alternatives.size() != 3)
            || (ambiguousMatch.m_alternatives[0].m_id != duplicateTarget.m_id)
            || (ambiguousMatch.m_alternatives[1].m_id != duplicateTarget2.m_id)
            || (ambiguousMatch.m_alternatives[2].m_id != duplicateTarget3.m_id))
        {
            errorStream << "Moving-target matcher test: ambiguous alternatives were not retained\n";
            return false;
        }

        MovingTargetMatcher::TargetState staleTarget = expectedTarget;
        staleTarget.m_dateTimeUtc = observation.m_startDateTimeUtc.addSecs(-120);
        const MovingTargetMatcher::Match staleMatch = MovingTargetMatcher::match(
            observation,
            {staleTarget});

        if (staleMatch.m_hasCandidate)
        {
            errorStream << "Moving-target matcher test: stale target was considered\n";
            return false;
        }

        MovingTargetMatcher::Match adsbMatch = match;
        adsbMatch.m_source = QStringLiteral("ADS-B");
        adsbMatch.m_scorePercent = 82.0;
        adsbMatch.m_secondBestScorePercent = 20.0;
        MovingTargetMatcher::Match tleMatch = match;
        tleMatch.m_source = QStringLiteral("TLE");
        tleMatch.m_scorePercent = 87.0;
        tleMatch.m_secondBestScorePercent = 30.0;
        const MovingTargetMatcher::Match combinedAmbiguous =
            MovingTargetMatcher::combine(adsbMatch, tleMatch);

        if (!combinedAmbiguous.m_ambiguous
            || combinedAmbiguous.m_matched
            || (combinedAmbiguous.m_source != QStringLiteral("TLE"))
            || (combinedAmbiguous.m_secondBestScorePercent != adsbMatch.m_scorePercent)
            || combinedAmbiguous.m_alternatives.isEmpty()
            || (combinedAmbiguous.m_alternatives.first().m_source
                != QStringLiteral("ADS-B")))
        {
            errorStream << "Moving-target matcher test: cross-source ambiguity was not retained\n";
            return false;
        }

        adsbMatch.m_scorePercent = 70.0;
        const MovingTargetMatcher::Match combinedMatch =
            MovingTargetMatcher::combine(adsbMatch, tleMatch);

        if (!combinedMatch.m_matched
            || combinedMatch.m_ambiguous
            || (combinedMatch.m_source != QStringLiteral("TLE"))
            || !combinedMatch.m_alternatives.isEmpty())
        {
            errorStream << "Moving-target matcher test: stronger TLE target was not selected\n";
            return false;
        }

        adsbMatch.m_scorePercent = 0.03;
        tleMatch.m_scorePercent = 0.04;
        const MovingTargetMatcher::Match poorMatch =
            MovingTargetMatcher::combine(adsbMatch, tleMatch);

        if (!poorMatch.m_hasCandidate
            || poorMatch.m_matched
            || poorMatch.m_ambiguous
            || (poorMatch.m_source != QStringLiteral("TLE")))
        {
            errorStream << "Moving-target matcher test: poor candidates were reported as ambiguous\n";
            return false;
        }

        return true;
    }

    bool runMoonTargetMatcherTests(QTextStream& errorStream)
    {
        MovingTargetMatcher::Observation observation;
        observation.m_durationS = 3.0;
        observation.m_frequencySpanHz = 100.0;
        observation.m_referenceFrequencyHz = 143050000.0;
        observation.m_transmitter = {50.0535, 19.8235, 0.0};
        observation.m_receiver = observation.m_transmitter;

        MeteorSatelliteMatcher::Geometry broadGeometry;
        broadGeometry.m_transmitterBeam = {0.0, 0.0, 360.0, 180.0};
        broadGeometry.m_receiverBeam = broadGeometry.m_transmitterBeam;
        MeteorSatelliteMatcher::MoonPrediction broadPrediction;
        const QDateTime searchStart(
            QDate(2026, 7, 22),
            QTime(0, 0),
            Qt::UTC);

        for (int hour = 0; hour < 48; ++hour)
        {
            observation.m_startDateTimeUtc = searchStart.addSecs(hour * 3600);
            broadPrediction = MeteorSatelliteMatcher::predictMoon(
                observation,
                broadGeometry);

            if (broadPrediction.m_possible) {
                break;
            }
        }

        if (!broadPrediction.m_possible
            || !broadPrediction.m_match.m_hasCandidate
            || !broadPrediction.m_match.m_prediction.m_valid)
        {
            errorStream << "Moon matcher test: visible Moon produced no Doppler prediction\n";
            return false;
        }

        observation.m_centerFrequencyOffsetHz =
            broadPrediction.m_match.m_prediction.m_centerFrequencyOffsetHz;
        observation.m_frequencyDriftHz =
            broadPrediction.m_match.m_prediction.m_frequencyDriftHz;
        broadPrediction = MeteorSatelliteMatcher::predictMoon(
            observation,
            broadGeometry);

        if (!broadPrediction.m_match.m_matched
            || (broadPrediction.m_match.m_source != QStringLiteral("Moon"))
            || (broadPrediction.m_match.m_endpointResidualRMSHz > 1e-6))
        {
            errorStream << "Moon matcher test: exact lunar Doppler was not matched\n";
            return false;
        }

        MeteorSatelliteMatcher::Geometry mispointedGeometry = broadGeometry;
        mispointedGeometry.m_receiverBeam = {
            std::remainder(
                broadPrediction.m_receiverAzimuthDegrees + 180.0,
                360.0),
            broadPrediction.m_receiverElevationDegrees,
            2.0,
            2.0
        };
        const MeteorSatelliteMatcher::MoonPrediction rejectedPrediction =
            MeteorSatelliteMatcher::predictMoon(
                observation,
                mispointedGeometry);

        if (rejectedPrediction.m_possible)
        {
            errorStream << "Moon matcher test: mispointed receiver beam accepted the Moon\n";
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

    struct Options
    {
        QString wavPath;
        QString testDir;
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
        static const QMap<QString, std::function<double&(Tunables&)>> doubleMembers = {
            {QStringLiteral("spectralFrameDurationS"), [](Tunables& t) -> auto& { return t.m_spectral.m_spectralFrameDurationS; }},
            {QStringLiteral("spectralHopFraction"), [](Tunables& t) -> auto& { return t.m_spectral.m_spectralHopFraction; }},
            {QStringLiteral("duplicateFrequencyOverlapFraction"), [](Tunables& t) -> auto& { return t.m_duplicate.m_duplicateFrequencyOverlapFraction; }},
            {QStringLiteral("blobSeedDb"), [](Tunables& t) -> auto& { return t.m_blob.m_seedDb; }},
            {QStringLiteral("blobGrowDb"), [](Tunables& t) -> auto& { return t.m_blob.m_growDb; }},
            {QStringLiteral("blobFloorOffsetDb"), [](Tunables& t) -> auto& { return t.m_blob.m_floorOffsetDb; }},
            {QStringLiteral("blobLinkGapSeconds"), [](Tunables& t) -> auto& { return t.m_blob.m_linkGapSeconds; }},
            {QStringLiteral("blobLinkMaxDriftHzPerS"), [](Tunables& t) -> auto& { return t.m_blob.m_linkMaxDriftHzPerS; }},
            {QStringLiteral("blobLinkTolHz"), [](Tunables& t) -> auto& { return t.m_blob.m_linkTolHz; }},
            {QStringLiteral("blobTrimKeepTime"), [](Tunables& t) -> auto& { return t.m_blob.m_trimKeepTime; }},
            {QStringLiteral("blobTrimKeepFreq"), [](Tunables& t) -> auto& { return t.m_blob.m_trimKeepFreq; }},
            {QStringLiteral("blobOccLimit"), [](Tunables& t) -> auto& { return t.m_blob.m_occLimit; }},
            {QStringLiteral("blobMinPeakExcessDb"), [](Tunables& t) -> auto& { return t.m_blob.m_minPeakExcessDb; }},
            {QStringLiteral("blobSweepMinLinR2"), [](Tunables& t) -> auto& { return t.m_blob.m_sweepMinLinR2; }},
            {QStringLiteral("blobSweepMinAbsSlopeHzPerS"), [](Tunables& t) -> auto& { return t.m_blob.m_sweepMinAbsSlopeHzPerS; }},
            {QStringLiteral("blobSweepMinDurationS"), [](Tunables& t) -> auto& { return t.m_blob.m_sweepMinDurationS; }},
            {QStringLiteral("blobFaintSeedDb"), [](Tunables& t) -> auto& { return t.m_blob.m_faintSeedDb; }},
            {QStringLiteral("blobSegMinR2"), [](Tunables& t) -> auto& { return t.m_blob.m_segMinR2; }},
            {QStringLiteral("blobSegSlopeMinHzPerS"), [](Tunables& t) -> auto& { return t.m_blob.m_segSlopeMinHzPerS; }},
            {QStringLiteral("blobSegSlopeMaxHzPerS"), [](Tunables& t) -> auto& { return t.m_blob.m_segSlopeMaxHzPerS; }},
            {QStringLiteral("blobSweepLinkTolHz"), [](Tunables& t) -> auto& { return t.m_blob.m_sweepLinkTolHz; }},
            {QStringLiteral("blobSweepLinkMaxGapS"), [](Tunables& t) -> auto& { return t.m_blob.m_sweepLinkMaxGapS; }},
            {QStringLiteral("blobSweepMergeMaxGapS"), [](Tunables& t) -> auto& { return t.m_blob.m_sweepMergeMaxGapS; }},
            {QStringLiteral("blobDashWalkStepGapS"), [](Tunables& t) -> auto& { return t.m_blob.m_dashWalkStepGapS; }},
            {QStringLiteral("blobDashWalkMaxS"), [](Tunables& t) -> auto& { return t.m_blob.m_dashWalkMaxS; }},
            {QStringLiteral("blobScoreThreshold"), [](Tunables& t) -> auto& { return t.m_blob.m_scoreThreshold; }},
            {QStringLiteral("blobWindowSeconds"), [](Tunables& t) -> auto& { return t.m_blob.m_windowSeconds; }},
            {QStringLiteral("blobMinWindowSeconds"), [](Tunables& t) -> auto& { return t.m_blob.m_minWindowSeconds; }},
            {QStringLiteral("blobEmitStrideS"), [](Tunables& t) -> auto& { return t.m_blob.m_emitStrideS; }},
            {QStringLiteral("blobFinalMarginS"), [](Tunables& t) -> auto& { return t.m_blob.m_finalMarginS; }},
        };
        static const QMap<QString, std::function<int&(Tunables&)>> intMembers = {
            {QStringLiteral("blobMinPix"), [](Tunables& t) -> auto& { return t.m_blob.m_minPix; }},
            {QStringLiteral("blobCloseFreqBins"), [](Tunables& t) -> auto& { return t.m_blob.m_closeFreqBins; }},
            {QStringLiteral("blobSegMinCols"), [](Tunables& t) -> auto& { return t.m_blob.m_segMinCols; }},
            {QStringLiteral("blobSegMinPix"), [](Tunables& t) -> auto& { return t.m_blob.m_segMinPix; }},
        };
        static const QMap<QString, std::function<bool&(Tunables&)>> boolMembers = {
            {QStringLiteral("blobWeakSweepExtend"), [](Tunables& t) -> auto& { return t.m_blob.m_weakSweepExtend; }},
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

                doubleMembers.value(name)(tunables) = value;
            }
            else if (intMembers.contains(name))
            {
                const int value = text.toInt(&ok);

                if (!ok)
                {
                    error = QString("Invalid value for tunable %1: %2").arg(name, text);
                    return false;
                }

                intMembers.value(name)(tunables) = value;
            }
            else if (boolMembers.contains(name))
            {
                if ((text == "1") || (text.compare("true", Qt::CaseInsensitive) == 0)) {
                    boolMembers.value(name)(tunables) = true;
                } else if ((text == "0") || (text.compare("false", Qt::CaseInsensitive) == 0)) {
                    boolMembers.value(name)(tunables) = false;
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

        if (options.settings.m_maxDurationMS < 1)
        {
            error = "--max-duration-ms must be at least 1";
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
            else if (readOptionValue(args, i, "max-duration-ms", value, error))
            {
                if (!parseIntValue("max-duration-ms", value, options.settings.m_maxDurationMS, error)) {
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
        out << "      --tunable <name=value>         Override a DetectorTunables member (repeatable). Names drop the\n";
        out << "                                     m_ prefix, e.g. --tunable blobScoreThreshold=100.\n";
        out << "      --channel-sample-rate <rate>   Detector sample rate: 100, 300, 1000, or 3000 Hz.\n";
        out << "      --input-frequency-offset <hz>  Channel input frequency offset in Hz.\n";
        out << "      --max-duration-ms <ms>         Maximum pulse duration in milliseconds.\n";
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

    void drainDetections(
        MessageQueue& queue,
        QVector<Detection>& detections,
        QVector<Detection> *satelliteDetections = nullptr)
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
            else if (satelliteDetections && MeteorDemodSink::MsgSatelliteDetected::match(*message))
            {
                const auto& messageDetection = (const MeteorDemodSink::MsgSatelliteDetected&) *message;
                const MeteorDemodSink::DetectionRecord& detection = messageDetection.getDetection();
                satelliteDetections->push_back({
                    detection.m_dateTimeUtc,
                    detection.m_displayDateTimeUtc.isValid()
                        ? detection.m_displayDateTimeUtc
                        : detection.m_dateTimeUtc,
                    std::sqrt(std::max(0.0, detection.m_peakPower)),
                    10.0 * std::log10(std::max(1e-20, detection.m_peakPower)),
                    10.0 * std::log10(std::max(1e-20, detection.m_backgroundPower)),
                    10.0 * std::log10(std::max(1e-20, detection.m_totalPower)),
                    detection.m_durationS,
                    detection.m_centerFrequency,
                    detection.m_frequencySpan,
                    detection.m_frequencyDrift,
                    detection.m_sampleRate,
                    detection.m_startSample,
                    detection.m_endSample
                });
            }

            delete message;
        }
    }

    bool feedSamples(
        MeteorBaseband& baseband,
        const SampleVector& samples,
        MessageQueue& outputQueue,
        QVector<Detection>& detections,
        QVector<Detection> *satelliteDetections = nullptr)
    {
        if (samples.empty()) {
            return true;
        }

        baseband.feed(samples.begin(), samples.end());
        processEvents();
        drainDetections(outputQueue, detections, satelliteDetections);
        return true;
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
        QVector<Detection> *satelliteDetections = nullptr)
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

        baseband.setInactivityFlushEnabled(false);
        baseband.setFifoLabel("meteor_demod_sink_test");
        baseband.setMessageQueueToGUI(&outputQueue);

        baseband.startWork();
        baseband.getInputMessageQueue()->push(new DSPSignalNotification(header.m_sampleRate, centerFrequency));
        baseband.getInputMessageQueue()->push(MeteorBaseband::MsgConfigureMeteorBaseband::create(options.settings, QStringList(), true));
        processEvents();

        // Tunable overrides go AFTER the settings configure: applying settings mirrors the
        // user-facing blob settings into the tunables, and test overrides must win over that.
        if (!options.tunableOverrides.isEmpty())
        {
            MeteorDemodSink::DetectorTunables tunables = baseband.getDetectorTunables();

            if (!applyTunableOverrides(tunables, options.tunableOverrides, error)) {
                return false;
            }

            baseband.setDetectorTunables(tunables);
            processEvents();
        }

        while (remainingBytes > 0)
        {
            if (!readWavChunk(wavFile, options.chunkSamples, samples, remainingBytes, error))
            {
                baseband.stopWork();
                return false;
            }

            feedSamples(baseband, samples, outputQueue, detections, satelliteDetections);
        }

        const qint64 tailSamples = ((qint64) header.m_sampleRate * options.tailMS) / 1000;
        qint64 tailSamplesRemaining = tailSamples;

        while (tailSamplesRemaining > 0)
        {
            const int count = (int) std::min<qint64>(options.chunkSamples, tailSamplesRemaining);
            samples.assign(count, Sample(0, 0));
            feedSamples(baseband, samples, outputQueue, detections, satelliteDetections);
            tailSamplesRemaining -= count;
        }

        processEvents();
        drainDetections(outputQueue, detections, satelliteDetections);
        baseband.stopWork();

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
            // "<base>@<rate>.csv" runs <base>.wav at the given channel sample rate, so both
            // calibrated rates can be covered by fixtures for the same recording.
            QString baseName = csvInfo.completeBaseName();
            Options fixtureOptions = options;
            const int atIndex = baseName.lastIndexOf(QChar('@'));

            if (atIndex > 0)
            {
                bool rateOk = false;
                const int rate = baseName.mid(atIndex + 1).toInt(&rateOk);

                if (rateOk && MeteorSettings::isSupportedSampleRate(rate))
                {
                    fixtureOptions.settings.m_channelSampleRate = rate;
                    baseName = baseName.left(atIndex);
                }
            }

            const QString wavPath = testDir.filePath(baseName + ".wav");
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

            if (!runWavFile(fixtureOptions, wavPath, actualDetections, error))
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
    // MainCore is reached indirectly (SampleSinkFifo write monitoring) and starts
    // continuous geolocation updates. On Windows an in-flight position request at
    // process exit deadlocks Windows.Devices.Geolocation's DLL detach and leaves
    // an unkillable process; the harness has no use for position data, so it opts
    // out via MainCore::initPosition's SDRANGEL_DISABLE_POSITION check.
    qputenv("SDRANGEL_DISABLE_POSITION", "1");

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

    if (!runMeteorSettingsTests(err)
        || !runMeteorMapGeometryTests(err)
        || !runMovingTargetMatcherTests(err)
        || !runMoonTargetMatcherTests(err)
        || !runRMOBReportTests(err))
    {
        return 2;
    }

    if (!options.testDir.isEmpty() && options.wavPath.isEmpty())
    {
        return runRegressionTests(options, out, err) ? 0 : 2;
    }

    QVector<Detection> detections;
    QVector<Detection> satelliteDetections;

    if (!runWavFile(
        options,
        options.wavPath,
        detections,
        error,
        &satelliteDetections))
    {
        err << error << "\n";
        return 1;
    }

    out << QString("Meteor detections: %1\n").arg(detections.size());
    out << QString("Satellite (sweep) detections: %1\n").arg(satelliteDetections.size());

    if (options.details) {
        printDetails(out, detections);
        if (!satelliteDetections.isEmpty()) {
            out << "Satellites:\n";
            printDetails(out, satelliteDetections);
        }
    }

    if ((options.expectCount >= 0) && (detections.size() != options.expectCount))
    {
        err << QString("Expected %1 detections, got %2\n").arg(options.expectCount).arg(detections.size());
        return 2;
    }

    return 0;
}
