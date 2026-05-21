///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3, or (at your option) later.         //
///////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <iostream>

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QLocale>
#include <QStringList>
#include <QTextStream>
#include <QTimer>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "camerastardetector.h"

#ifndef CAMERA_STAR_TEST_DATA_DIR
#define CAMERA_STAR_TEST_DATA_DIR "."
#endif

namespace
{
struct StarTestCase
{
    QString name;
    QString imagePath;
    QDateTime dateTime;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double azimuth = 0.0;
    double elevation = 0.0;
    double roll = 0.0;
    double fov = 0.0;
    CameraSettings::LensProjection projection = CameraSettings::LensProjectionRectilinear;
    double centerOffsetX = 0.0;
    double centerOffsetY = 0.0;
    double distortionK1 = 0.0;
    QStringList expectedStars;
};

struct DetectorRunResult
{
    bool completed = false;
    QString error;
    CameraPipelineFramePtr frame;
};

QStringList parseCsvLine(const QString& line, bool* ok)
{
    QStringList fields;
    QString current;
    bool inQuotes = false;

    for (int i = 0; i < line.size(); ++i)
    {
        const QChar ch = line.at(i);
        if (ch == QLatin1Char('"'))
        {
            if (inQuotes && (i + 1 < line.size()) && (line.at(i + 1) == QLatin1Char('"')))
            {
                current += QLatin1Char('"');
                ++i;
            }
            else
            {
                inQuotes = !inQuotes;
            }
        }
        else if ((ch == QLatin1Char(',')) && !inQuotes)
        {
            fields.append(current);
            current.clear();
        }
        else
        {
            current += ch;
        }
    }

    fields.append(current);
    if (ok) {
        *ok = !inQuotes;
    }
    return fields;
}

QString fieldValue(const QStringList& header, const QStringList& values, const QString& name, bool* ok)
{
    const int index = header.indexOf(name);
    if ((index < 0) || (index >= values.size()))
    {
        if (ok) {
            *ok = false;
        }
        return QString();
    }

    return values.at(index).trimmed();
}

double parseDouble(const QString& value, const QString& column, int lineNumber, bool* ok)
{
    bool localOk = false;
    const double result = QLocale::c().toDouble(value.trimmed(), &localOk);
    if (!localOk)
    {
        std::cerr << "Line " << lineNumber << ": invalid numeric value for " << column.toStdString()
                  << ": " << value.toStdString() << '\n';
        if (ok) {
            *ok = false;
        }
    }
    return result;
}

QDateTime parseDateTime(const QString& value, int lineNumber, bool* ok)
{
    QDateTime dateTime = QDateTime::fromString(value.trimmed(), QStringLiteral("yyyy-MM-dd HH:mm:ss"));
    if (!dateTime.isValid())
    {
        std::cerr << "Line " << lineNumber << ": invalid time: " << value.toStdString() << '\n';
        if (ok) {
            *ok = false;
        }
        return QDateTime();
    }

    return dateTime;
}

CameraSettings::LensProjection parseProjection(const QString& value)
{
    const QString normalized = value.trimmed().toCaseFolded();
    if (normalized.contains(QStringLiteral("equisolid"))) {
        return CameraSettings::LensProjectionEquisolid;
    }
    if (normalized.contains(QStringLiteral("equidistant")) || normalized.contains(QStringLiteral("fisheye"))) {
        return CameraSettings::LensProjectionEquidistant;
    }
    return CameraSettings::LensProjectionRectilinear;
}

QStringList parseExpectedStars(const QString& value)
{
    QStringList stars;
    const QStringList parts = value.split(QLatin1Char(','), Qt::SkipEmptyParts);
    for (const QString& part : parts)
    {
        const QString trimmed = part.trimmed();
        if (!trimmed.isEmpty()) {
            stars.append(trimmed);
        }
    }
    return stars;
}

bool readTestCases(const QString& csvPath, QVector<StarTestCase>& testCases)
{
    QFile file(csvPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        std::cerr << "Failed to open " << csvPath.toStdString() << '\n';
        return false;
    }

    QTextStream stream(&file);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    stream.setCodec("UTF-8");
#endif

    if (stream.atEnd())
    {
        std::cerr << "CSV file is empty: " << csvPath.toStdString() << '\n';
        return false;
    }

    bool csvOk = false;
    const QStringList header = parseCsvLine(stream.readLine(), &csvOk);
    if (!csvOk)
    {
        std::cerr << "CSV header has unterminated quotes\n";
        return false;
    }

    const QDir csvDir(QFileInfo(csvPath).absolutePath());
    int lineNumber = 1;
    while (!stream.atEnd())
    {
        const QString line = stream.readLine();
        ++lineNumber;
        if (line.trimmed().isEmpty()) {
            continue;
        }

        bool rowOk = false;
        const QStringList fields = parseCsvLine(line, &rowOk);
        if (!rowOk || (fields.size() != header.size()))
        {
            std::cerr << "Line " << lineNumber << ": expected " << header.size()
                      << " CSV columns, got " << fields.size() << '\n';
            return false;
        }

        bool ok = true;
        StarTestCase test;
        const QString image = fieldValue(header, fields, QStringLiteral("image"), &ok);
        test.imagePath = QFileInfo(image).isAbsolute() ? image : csvDir.filePath(image);
        test.name = QFileInfo(test.imagePath).fileName();
        test.dateTime = parseDateTime(fieldValue(header, fields, QStringLiteral("time"), &ok), lineNumber, &ok);
        test.latitude = parseDouble(fieldValue(header, fields, QStringLiteral("latitude"), &ok), QStringLiteral("latitude"), lineNumber, &ok);
        test.longitude = parseDouble(fieldValue(header, fields, QStringLiteral("longitude"), &ok), QStringLiteral("longitude"), lineNumber, &ok);
        test.altitude = parseDouble(fieldValue(header, fields, QStringLiteral("altitude"), &ok), QStringLiteral("altitude"), lineNumber, &ok);
        test.azimuth = parseDouble(fieldValue(header, fields, QStringLiteral("azimuth"), &ok), QStringLiteral("azimuth"), lineNumber, &ok);
        test.elevation = parseDouble(fieldValue(header, fields, QStringLiteral("elevation"), &ok), QStringLiteral("elevation"), lineNumber, &ok);
        test.roll = parseDouble(fieldValue(header, fields, QStringLiteral("roll"), &ok), QStringLiteral("roll"), lineNumber, &ok);
        test.fov = parseDouble(fieldValue(header, fields, QStringLiteral("fov"), &ok), QStringLiteral("fov"), lineNumber, &ok);
        test.projection = parseProjection(fieldValue(header, fields, QStringLiteral("projection"), &ok));
        test.centerOffsetX = parseDouble(fieldValue(header, fields, QStringLiteral("cx"), &ok), QStringLiteral("cx"), lineNumber, &ok);
        test.centerOffsetY = parseDouble(fieldValue(header, fields, QStringLiteral("cy"), &ok), QStringLiteral("cy"), lineNumber, &ok);
        test.distortionK1 = parseDouble(fieldValue(header, fields, QStringLiteral("k1"), &ok), QStringLiteral("k1"), lineNumber, &ok);
        test.expectedStars = parseExpectedStars(fieldValue(header, fields, QStringLiteral("stars"), &ok));

        if (!ok) {
            return false;
        }
        testCases.append(test);
    }

    return true;
}

CameraSettings makeSettings(const StarTestCase& test)
{
    CameraSettings settings;
    settings.m_starDetect = true;
    settings.m_plateSolve = true;
    settings.m_latitude = static_cast<float>(test.latitude);
    settings.m_longitude = static_cast<float>(test.longitude);
    settings.m_altitude = static_cast<float>(test.altitude);
    settings.m_azimuth = static_cast<float>(test.azimuth);
    settings.m_elevation = static_cast<float>(test.elevation);
    settings.m_roll = static_cast<float>(test.roll);
    settings.m_fov = static_cast<float>(test.fov);
    settings.m_lensProjection = test.projection;
    settings.m_lensCenterOffsetX = test.centerOffsetX;
    settings.m_lensCenterOffsetY = test.centerOffsetY;
    settings.m_lensDistortionK1 = test.distortionK1;
    settings.m_plateSolveUseCurrentDateTime = false;
    settings.m_plateSolveDateTime = test.dateTime;
    settings.m_plateSolveDateTimeUtc = false;
    settings.m_plateSolveUseDownloadedCatalog = true;
    settings.m_plateSolveCatalogSource = CameraSettings::PlateSolveCatalogSirilSpccGaia;
    settings.m_plateSolveStartMode = CameraSettings::PlateSolveStartFovAzElRollLens;
    settings.m_plateSolveLabelMode = CameraSettings::PlateSolveLabelName;
    settings.m_plateSolveMaxMagnitude = CameraSettings::m_maxPlateSolveMagnitude;
    settings.m_plateSolveMinMatches = CameraSettings::m_minPlateSolveMatches;
    settings.m_plateSolveMatchRadius = 24.0;
    settings.m_plateSolveFinalMatchRadius = 24.0;
    settings.m_plateSolveSearchRadius = 3.5;
    return settings;
}

QImage loadTestImage(const QString& imagePath, QString* error)
{
    const cv::Mat bgr = cv::imread(imagePath.toStdString(), cv::IMREAD_COLOR);
    if (!bgr.empty())
    {
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        return QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
    }

    QImage image(imagePath);
    if (!image.isNull()) {
        return image.convertToFormat(QImage::Format_RGB888);
    }

    if (error) {
        *error = QStringLiteral("Failed to load image: %1").arg(imagePath);
    }
    return QImage();
}

DetectorRunResult runDetector(const StarTestCase& test)
{
    DetectorRunResult result;
    QString imageError;
    QImage image = loadTestImage(test.imagePath, &imageError);
    if (image.isNull())
    {
        result.error = imageError;
        return result;
    }

    CameraStarDetector detector;
    MessageQueue outputQueue;
    QEventLoop loop;
    QTimer timeout;
    timeout.setSingleShot(true);

    QObject::connect(&outputQueue, &MessageQueue::messageEnqueued, &loop, [&]() {
        Message *message = nullptr;
        while ((message = outputQueue.pop()) != nullptr)
        {
            if (CameraDetectionStage::MsgProcessFrame::match(*message))
            {
                const CameraDetectionStage::MsgProcessFrame& frameMessage =
                    static_cast<const CameraDetectionStage::MsgProcessFrame&>(*message);
                result.frame = frameMessage.getFrame();
                result.completed = true;
                delete message;
                loop.quit();
                return;
            }

            delete message;
        }
    });
    QObject::connect(&timeout, &QTimer::timeout, &loop, [&]() {
        result.error = QStringLiteral("Timed out waiting for star detector output");
        loop.quit();
    });

    detector.setNextStageInputMessageQueue(&outputQueue);
    detector.startWork();
    detector.getInputMessageQueue()->push(CameraDetectionStage::MsgConfigureCameraDetectionStage::create(
        makeSettings(test),
        QList<QString>(),
        true));

    CameraPipelineFramePtr frame(new CameraPipelineFrame);
    frame->m_image = image;
    frame->m_unprocessedImage = image;
    frame->m_captureDateTime = test.dateTime;
    detector.getInputMessageQueue()->push(CameraDetectionStage::MsgProcessFrame::create(frame));

    timeout.start(120000);
    loop.exec();
    detector.stopWork();
    detector.getInputMessageQueue()->clear();
    outputQueue.clear();
    return result;
}

QString normalizedStarName(const QString& value)
{
    return value.trimmed().toCaseFolded();
}

bool labelMatchesExpectedStar(const QString& label, const QString& expected)
{
    const QString normalizedLabel = normalizedStarName(label);
    const QString normalizedExpected = normalizedStarName(expected);
    return (normalizedLabel == normalizedExpected)
        || normalizedLabel.contains(normalizedExpected)
        || normalizedExpected.contains(normalizedLabel);
}

QStringList solvedStarLabels(const CameraPipelineFramePtr& frame)
{
    QStringList labels;
    if (!frame) {
        return labels;
    }

    for (const CameraPipelineStarDetection& detection : frame->m_starDetections)
    {
        if (detection.m_solved && !detection.m_label.trimmed().isEmpty()) {
            labels.append(detection.m_label.trimmed());
        }
    }
    labels.removeDuplicates();
    std::sort(labels.begin(), labels.end(), [](const QString& a, const QString& b) {
        return a.localeAwareCompare(b) < 0;
    });
    return labels;
}

QStringList missingExpectedStars(const QStringList& detectedLabels, const QStringList& expectedStars)
{
    QStringList missing;
    for (const QString& expected : expectedStars)
    {
        const bool found = std::any_of(detectedLabels.cbegin(), detectedLabels.cend(), [&](const QString& label) {
            return labelMatchesExpectedStar(label, expected);
        });
        if (!found) {
            missing.append(expected);
        }
    }
    return missing;
}

int runTests(const QString& csvPath)
{
    QVector<StarTestCase> tests;
    if (!readTestCases(csvPath, tests)) {
        return 2;
    }

    if (tests.isEmpty())
    {
        std::cerr << "No test cases found in " << csvPath.toStdString() << '\n';
        return 2;
    }

    int failures = 0;
    for (const StarTestCase& test : tests)
    {
        const DetectorRunResult result = runDetector(test);
        if (!result.completed)
        {
            ++failures;
            std::cerr << "FAIL " << test.name.toStdString() << ": " << result.error.toStdString() << '\n';
            continue;
        }

        const QStringList labels = solvedStarLabels(result.frame);
        const QStringList missing = missingExpectedStars(labels, test.expectedStars);
        const bool pass = missing.isEmpty();
        if (!pass) {
            ++failures;
        }

        std::cout << (pass ? "PASS " : "FAIL ") << test.name.toStdString()
                  << ": detections=" << result.frame->m_starDetections.size()
                  << " matched=" << result.frame->m_plateSolvedMatches
                  << " solved=" << (result.frame->m_plateSolved ? "true" : "false")
                  << " catalog=" << result.frame->m_plateSolveCatalogSource.toStdString()
                  << " catalogStars=" << result.frame->m_plateSolveCatalogStarsLoaded
                  << " candidates=" << result.frame->m_plateSolveCatalogCandidateStars
                  << " outliers=" << result.frame->m_plateSolveOutlierStars
                  << " rms=" << result.frame->m_plateSolveRmsError
                  << '\n';
        std::cout << "  labels: " << labels.join(QStringLiteral(", ")).toStdString() << '\n';
        if (!missing.isEmpty()) {
            std::cout << "  missing: " << missing.join(QStringLiteral(", ")).toStdString() << '\n';
        }
    }

    if (failures > 0) {
        std::cerr << failures << " camera star test(s) failed\n";
        return 1;
    }

    std::cout << tests.size() << " camera star test(s) passed\n";
    return 0;
}
}

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setOrganizationName(QStringLiteral("f4exb"));
    QCoreApplication::setApplicationName(QStringLiteral("SDRangel"));

    const QStringList args = app.arguments();
    const QString csvPath = (args.size() > 1)
        ? args.at(1)
        : QDir(QString::fromUtf8(CAMERA_STAR_TEST_DATA_DIR)).filePath(QStringLiteral("star-tests.csv"));
    return runTests(QFileInfo(csvPath).absoluteFilePath());
}
