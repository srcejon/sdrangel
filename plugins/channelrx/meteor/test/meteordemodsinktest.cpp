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
#include <QStringList>
#include <QTextStream>
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

#ifndef METEOR_TEST_DATA_DIR
#define METEOR_TEST_DATA_DIR ""
#endif

namespace {
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
    };

    struct Options
    {
        QString wavPath;
        QString testDir;
        MeteorSettings settings;
        int chunkSamples = 4096;
        int tailMS = 2000;
        int expectCount = -1;
        bool details = false;
        bool showHelp = false;
    };

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
        const QList<int> allowedSampleRates({100, 300, 1000, 3000});

        if (!allowedSampleRates.contains(options.settings.m_channelSampleRate))
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

        if (header != "index,timeOffsetS,durationS,centerFrequencyHz,frequencySpanHz,frequencyDriftHz,totalPowerDB")
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

            if (fields.size() != 7)
            {
                error = QString("Expected 7 fields in %1 row %2").arg(csvPath).arg(row);
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

        if (actual.size() != expected.size())
        {
            err << QString("%1: expected %2 detections, got %3\n")
                .arg(name)
                .arg(expected.size())
                .arg(actual.size());
            ok = false;
        }

        if (actual.isEmpty() || expected.isEmpty()) {
            return ok;
        }

        const int count = std::min(actual.size(), expected.size());
        for (int i = 0; i < count; i++)
        {
            const Detection& detection = actual[i];
            const ExpectedDetection& expectation = expected[i];
            const double timeOffsetS = (double) detection.startSample / (double) std::max(1, detection.sampleRate);

            if (!nearlyEqual(timeOffsetS, expectation.timeOffsetS, timeToleranceS))
            {
                err << QString("%1 detection %2: expected timeOffsetS %3, got %4\n")
                    .arg(name)
                    .arg(i + 1)
                    .arg(expectation.timeOffsetS, 0, 'f', 3)
                    .arg(timeOffsetS, 0, 'f', 3);
                ok = false;
            }

            if (!nearlyEqual(detection.durationS, expectation.durationS, durationToleranceS))
            {
                err << QString("%1 detection %2: expected durationS %3, got %4\n")
                    .arg(name)
                    .arg(i + 1)
                    .arg(expectation.durationS, 0, 'f', 6)
                    .arg(detection.durationS, 0, 'f', 6);
                ok = false;
            }

            if (!nearlyEqual(detection.centerFrequency, expectation.centerFrequency, frequencyToleranceHz))
            {
                err << QString("%1 detection %2: expected centerFrequencyHz %3, got %4\n")
                    .arg(name)
                    .arg(i + 1)
                    .arg(expectation.centerFrequency, 0, 'f', 2)
                    .arg(detection.centerFrequency, 0, 'f', 2);
                ok = false;
            }

            if (!nearlyEqual(detection.frequencySpan, expectation.frequencySpan, frequencyToleranceHz))
            {
                err << QString("%1 detection %2: expected frequencySpanHz %3, got %4\n")
                    .arg(name)
                    .arg(i + 1)
                    .arg(expectation.frequencySpan, 0, 'f', 2)
                    .arg(detection.frequencySpan, 0, 'f', 2);
                ok = false;
            }

            if (!nearlyEqual(detection.frequencyDrift, expectation.frequencyDrift, frequencyToleranceHz))
            {
                err << QString("%1 detection %2: expected frequencyDriftHz %3, got %4\n")
                    .arg(name)
                    .arg(i + 1)
                    .arg(expectation.frequencyDrift, 0, 'f', 2)
                    .arg(detection.frequencyDrift, 0, 'f', 2);
                ok = false;
            }

            if (!nearlyEqual(detection.totalPowerDB, expectation.totalPowerDB, totalPowerToleranceDB))
            {
                err << QString("%1 detection %2: expected totalPowerDB %3, got %4\n")
                    .arg(name)
                    .arg(i + 1)
                    .arg(expectation.totalPowerDB, 0, 'f', 2)
                    .arg(detection.totalPowerDB, 0, 'f', 2);
                ok = false;
            }
        }

        return ok;
    }

    bool runWavFile(const Options& options, const QString& wavPath, QVector<Detection>& detections, QString& error)
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
        qint64 remainingBytes = std::min<qint64>(header.m_dataHeader.m_size, availableDataBytes);

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

        baseband.setFifoLabel("meteor_demod_sink_test");
        baseband.setMessageQueueToGUI(&outputQueue);
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

    if (!options.testDir.isEmpty() && options.wavPath.isEmpty())
    {
        return runRegressionTests(options, out, err) ? 0 : 2;
    }

    QVector<Detection> detections;

    if (!runWavFile(options, options.wavPath, detections, error))
    {
        err << error << "\n";
        return 1;
    }

    out << QString("Meteor detections: %1\n").arg(detections.size());

    if (options.details) {
        printDetails(out, detections);
    }

    if ((options.expectCount >= 0) && (detections.size() != options.expectCount))
    {
        err << QString("Expected %1 detections, got %2\n").arg(options.expectCount).arg(detections.size());
        return 2;
    }

    return 0;
}
