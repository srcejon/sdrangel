///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3, or (at your option) later.         //
///////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QImage>
#include <QLocale>
#include <QStringList>
#include <QTextStream>
#include <QTimer>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "camerastardetector.h"
#include "util/astronomy.h"

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

struct CatalogStar
{
    QString name;
    double rightAscensionDegrees = 0.0;
    double declinationDegrees = 0.0;
    double magnitude = 0.0;
};

struct SkyVector
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct TestProjector
{
    bool valid = false;
    CameraSettings::LensProjection lensProjection = CameraSettings::LensProjectionRectilinear;
    SkyVector center;
    SkyVector right;
    SkyVector up;
    double halfHorizontalFov = 0.0;
    double horizontalScale = 1.0;
    double verticalScale = 1.0;
    double principalPointX = 0.0;
    double principalPointY = 0.0;
    double distortionK1 = 0.0;
    int width = 0;
    int height = 0;
};

constexpr double kPi = 3.14159265358979323846;

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

QString normalizedStarName(const QString& value)
{
    return value.trimmed().toCaseFolded();
}

double degToRad(double value)
{
    return value * kPi / 180.0;
}

SkyVector normalize(const SkyVector& vector)
{
    const double length = std::sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
    if (length <= 0.0) {
        return {};
    }
    return {vector.x / length, vector.y / length, vector.z / length};
}

double dot(const SkyVector& lhs, const SkyVector& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

SkyVector cross(const SkyVector& lhs, const SkyVector& rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x
    };
}

SkyVector rotateAroundAxis(const SkyVector& vector, const SkyVector& axis, double angleRadians)
{
    const double cosAngle = std::cos(angleRadians);
    const double sinAngle = std::sin(angleRadians);
    const double axisDot = dot(axis, vector);
    return {
        vector.x * cosAngle + (axis.y * vector.z - axis.z * vector.y) * sinAngle + axis.x * axisDot * (1.0 - cosAngle),
        vector.y * cosAngle + (axis.z * vector.x - axis.x * vector.z) * sinAngle + axis.y * axisDot * (1.0 - cosAngle),
        vector.z * cosAngle + (axis.x * vector.y - axis.y * vector.x) * sinAngle + axis.z * axisDot * (1.0 - cosAngle)
    };
}

SkyVector vectorFromAltAz(double azimuthDegrees, double elevationDegrees)
{
    const double azimuth = degToRad(azimuthDegrees);
    const double elevation = degToRad(elevationDegrees);
    const double cosElevation = std::cos(elevation);
    return {
        cosElevation * std::sin(azimuth),
        cosElevation * std::cos(azimuth),
        std::sin(elevation)
    };
}

double parseHmsDegrees(const QString& value, bool* ok)
{
    const QStringList parts = value.split(QLatin1Char(' '), Qt::SkipEmptyParts);
    if (parts.size() != 3)
    {
        if (ok) {
            *ok = false;
        }
        return 0.0;
    }

    bool hOk = false;
    bool mOk = false;
    bool sOk = false;
    const double hours = QLocale::c().toDouble(parts.at(0), &hOk);
    const double minutes = QLocale::c().toDouble(parts.at(1), &mOk);
    const double seconds = QLocale::c().toDouble(parts.at(2), &sOk);
    if (ok) {
        *ok = hOk && mOk && sOk;
    }
    return (hours + minutes / 60.0 + seconds / 3600.0) * 15.0;
}

double parseDmsDegrees(const QString& value, bool* ok)
{
    const QStringList parts = value.split(QLatin1Char(' '), Qt::SkipEmptyParts);
    if (parts.size() != 3)
    {
        if (ok) {
            *ok = false;
        }
        return 0.0;
    }

    bool dOk = false;
    bool mOk = false;
    bool sOk = false;
    const double degrees = QLocale::c().toDouble(parts.at(0), &dOk);
    const double minutes = QLocale::c().toDouble(parts.at(1), &mOk);
    const double seconds = QLocale::c().toDouble(parts.at(2), &sOk);
    const double sign = degrees < 0.0 ? -1.0 : 1.0;
    if (ok) {
        *ok = dOk && mOk && sOk;
    }
    return degrees + sign * (minutes / 60.0 + seconds / 3600.0);
}

QVector<CatalogStar> loadDiagnosticCatalog()
{
    QVector<CatalogStar> stars;
    QFile file(QStringLiteral(":/camera/brightstarcatalog.txt"));
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return stars;
    }

    QTextStream stream(&file);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    stream.setCodec("UTF-8");
#endif
    if (!stream.atEnd()) {
        stream.readLine();
    }

    while (!stream.atEnd())
    {
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty()) {
            continue;
        }

        const QStringList fields = line.split(QLatin1Char('|'));
        if (fields.size() < 4) {
            continue;
        }

        bool raOk = false;
        bool decOk = false;
        bool magOk = false;
        CatalogStar star;
        star.name = fields.at(0).trimmed();
        star.rightAscensionDegrees = parseHmsDegrees(fields.at(1).trimmed(), &raOk);
        star.declinationDegrees = parseDmsDegrees(fields.at(2).trimmed(), &decOk);
        star.magnitude = QLocale::c().toDouble(fields.at(3).trimmed(), &magOk);
        if (raOk && decOk && magOk) {
            stars.append(star);
        }
    }

    return stars;
}

const CatalogStar* findDiagnosticStar(const QVector<CatalogStar>& catalog, const QString& expected)
{
    const QString normalizedExpected = normalizedStarName(expected);
    for (const CatalogStar& star : catalog)
    {
        const QString normalizedLabel = normalizedStarName(star.name);
        if ((normalizedLabel == normalizedExpected)
            || normalizedLabel.contains(normalizedExpected)
            || normalizedExpected.contains(normalizedLabel))
        {
            return &star;
        }
    }
    return nullptr;
}

double halfHorizontalFovFromLongEdgeFov(CameraSettings::LensProjection lensProjection,
                                       const QSize& imageSize,
                                       double fovDegrees)
{
    const double halfLongEdgeFov = degToRad(fovDegrees) * 0.5;
    if ((imageSize.width() <= 0) || (imageSize.height() <= 0) || (imageSize.width() >= imageSize.height())) {
        return halfLongEdgeFov;
    }

    const double aspect = static_cast<double>(imageSize.height()) / static_cast<double>(imageSize.width());
    switch (lensProjection)
    {
    case CameraSettings::LensProjectionEquidistant:
        return halfLongEdgeFov / aspect;
    case CameraSettings::LensProjectionEquisolid:
        return 2.0 * std::asin(std::clamp(std::sin(halfLongEdgeFov * 0.5) / aspect, -1.0, 1.0));
    case CameraSettings::LensProjectionRectilinear:
    default:
        return std::atan(std::tan(halfLongEdgeFov) / aspect);
    }
}

TestProjector createDiagnosticProjector(const StarTestCase& test,
                                        const CameraPipelineFramePtr& frame)
{
    TestProjector projector;
    if (!frame || frame->m_image.isNull()) {
        return projector;
    }

    const double azimuth = frame->m_plateSolved ? frame->m_plateSolveAzimuth : test.azimuth;
    const double elevation = frame->m_plateSolved ? frame->m_plateSolveElevation : test.elevation;
    const double roll = frame->m_plateSolved ? frame->m_plateSolveRoll : test.roll;
    const double fov = frame->m_plateSolved ? frame->m_plateSolveFov : test.fov;
    const double centerOffsetX = frame->m_plateSolved ? frame->m_plateSolveCenterOffsetX : test.centerOffsetX;
    const double centerOffsetY = frame->m_plateSolved ? frame->m_plateSolveCenterOffsetY : test.centerOffsetY;
    const double distortionK1 = frame->m_plateSolved ? frame->m_plateSolveDistortionK1 : test.distortionK1;

    projector.width = frame->m_image.width();
    projector.height = frame->m_image.height();
    projector.lensProjection = test.projection;
    if ((projector.width <= 0) || (projector.height <= 0) || (fov <= 0.0)) {
        return projector;
    }

    const double azimuthRadians = degToRad(azimuth);
    projector.center = normalize(vectorFromAltAz(azimuth, elevation));
    projector.right = normalize({std::cos(azimuthRadians), -std::sin(azimuthRadians), 0.0});
    projector.up = normalize(cross(projector.right, projector.center));
    if ((dot(projector.right, projector.right) <= 0.0) || (dot(projector.up, projector.up) <= 0.0)) {
        return projector;
    }

    const double rollRadians = degToRad(roll);
    if (std::fabs(rollRadians) > 1e-9)
    {
        projector.right = normalize(rotateAroundAxis(projector.right, projector.center, rollRadians));
        projector.up = normalize(rotateAroundAxis(projector.up, projector.center, rollRadians));
    }

    projector.halfHorizontalFov = halfHorizontalFovFromLongEdgeFov(test.projection, frame->m_image.size(), fov);
    if ((projector.halfHorizontalFov <= 0.0) || (projector.halfHorizontalFov >= (kPi * 0.5))) {
        return projector;
    }

    const double aspect = static_cast<double>(projector.height) / static_cast<double>(projector.width);
    projector.horizontalScale = 1.0;
    projector.verticalScale = aspect;
    projector.principalPointX = static_cast<double>(projector.width) * 0.5 + centerOffsetX;
    projector.principalPointY = static_cast<double>(projector.height) * 0.5 + centerOffsetY;
    projector.distortionK1 = distortionK1;
    projector.valid = projector.verticalScale > 0.0;
    return projector;
}

bool projectDiagnosticVector(const TestProjector& projector, const SkyVector& vector, QPointF& point)
{
    if (!projector.valid) {
        return false;
    }

    const double depth = dot(vector, projector.center);
    if (depth <= 0.0) {
        return false;
    }

    const double planeX = dot(vector, projector.right);
    const double planeY = dot(vector, projector.up);
    const double theta = std::acos(std::clamp(depth, -1.0, 1.0));
    const double phi = std::atan2(planeY, planeX);
    double projectionRadius = 0.0;
    switch (projector.lensProjection)
    {
    case CameraSettings::LensProjectionEquidistant:
        projectionRadius = theta / projector.halfHorizontalFov;
        break;
    case CameraSettings::LensProjectionEquisolid:
        projectionRadius = std::sin(theta * 0.5) / std::sin(projector.halfHorizontalFov * 0.5);
        break;
    case CameraSettings::LensProjectionRectilinear:
    default:
        projectionRadius = std::tan(theta) / std::tan(projector.halfHorizontalFov);
        break;
    }

    double projectedX = std::cos(phi) * projectionRadius;
    double projectedY = std::sin(phi) * projectionRadius;
    if (std::fabs(projector.distortionK1) > 1e-9)
    {
        const double radiusSquared = projectedX * projectedX + projectedY * projectedY;
        const double distortionScale = std::max(0.1, 1.0 + projector.distortionK1 * radiusSquared);
        projectedX *= distortionScale;
        projectedY *= distortionScale;
    }

    point.setX(projector.principalPointX + (projectedX / projector.horizontalScale) * 0.5 * static_cast<double>(projector.width));
    point.setY(projector.principalPointY - (projectedY / projector.verticalScale) * 0.5 * static_cast<double>(projector.height));
    return std::isfinite(point.x()) && std::isfinite(point.y());
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
    settings.m_plateSolveMaxMagnitude = (test.fov > 30.0)
        ? 5.0
        : CameraSettings::m_maxPlateSolveMagnitude;
    settings.m_plateSolveMinMatches = 4;
    settings.m_plateSolveMatchRadius = 24.0;
    settings.m_plateSolveFinalMatchRadius = 24.0;
    settings.m_plateSolveSearchRadius = (test.fov > 30.0) ? 12.0 : 3.5;
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

bool labelMatchesExpectedStar(const QString& label, const QString& expected)
{
    const QString normalizedLabel = normalizedStarName(label);
    const QString normalizedExpected = normalizedStarName(expected);
    return (normalizedLabel == normalizedExpected)
        || normalizedLabel.contains(normalizedExpected)
        || normalizedExpected.contains(normalizedLabel);
}

bool expectedStarHasNearbyDetection(const StarTestCase& test,
                                    const CameraPipelineFramePtr& frame,
                                    const QVector<CatalogStar>& catalog,
                                    const QString& expected)
{
    if (!frame) {
        return false;
    }

    const CatalogStar *star = findDiagnosticStar(catalog, expected);
    if (!star) {
        return false;
    }

    const TestProjector projector = createDiagnosticProjector(test, frame);
    const QDateTime solveDateTimeUtc = test.dateTime.toUTC();
    const AzAlt azAlt = Astronomy::raDecToAzAlt(
        RADec{star->rightAscensionDegrees / 15.0, star->declinationDegrees},
        test.latitude,
        test.longitude,
        solveDateTimeUtc,
        true);

    QPointF projected;
    if (!projectDiagnosticVector(projector, normalize(vectorFromAltAz(azAlt.az, azAlt.alt)), projected)) {
        return false;
    }

    double nearestDistance = std::numeric_limits<double>::infinity();
    for (const CameraPipelineStarDetection& detection : frame->m_starDetections)
    {
        const double dx = detection.m_center.x() - projected.x();
        const double dy = detection.m_center.y() - projected.y();
        nearestDistance = std::min(nearestDistance, std::hypot(dx, dy));
    }

    const QSize imageSize = frame->m_image.size();
    const double maxImageDimension = std::max(imageSize.width(), imageSize.height());
    const double tolerancePixels = std::max(72.0, std::min(128.0, maxImageDimension * 0.05));
    return nearestDistance <= tolerancePixels;
}

QStringList projectedExpectedStarDetections(const StarTestCase& test,
                                            const CameraPipelineFramePtr& frame,
                                            const QVector<CatalogStar>& catalog,
                                            const QStringList& detectedLabels)
{
    QStringList projectedDetections;
    for (const QString& expected : test.expectedStars)
    {
        const bool alreadyLabelled = std::any_of(detectedLabels.cbegin(), detectedLabels.cend(), [&](const QString& label) {
            return labelMatchesExpectedStar(label, expected);
        });
        if (!alreadyLabelled && expectedStarHasNearbyDetection(test, frame, catalog, expected)) {
            projectedDetections.append(expected);
        }
    }
    return projectedDetections;
}

QStringList missingExpectedStars(const QStringList& detectedLabels,
                                 const QStringList& projectedDetections,
                                 const QStringList& expectedStars)
{
    QStringList missing;
    for (const QString& expected : expectedStars)
    {
        const bool labelled = std::any_of(detectedLabels.cbegin(), detectedLabels.cend(), [&](const QString& label) {
            return labelMatchesExpectedStar(label, expected);
        });
        const bool projected = std::any_of(projectedDetections.cbegin(), projectedDetections.cend(), [&](const QString& label) {
            return labelMatchesExpectedStar(label, expected);
        });
        if (!labelled && !projected) {
            missing.append(expected);
        }
    }
    return missing;
}

void printDetectionDiagnostics(const CameraPipelineFramePtr& frame)
{
    if (!frame) {
        return;
    }

    std::cout << "  detections:\n";
    for (int i = 0; i < frame->m_starDetections.size(); ++i)
    {
        const CameraPipelineStarDetection& detection = frame->m_starDetections.at(i);
        std::cout << "    #" << i
                  << " x=" << detection.m_center.x()
                  << " y=" << detection.m_center.y()
                  << " peak=" << detection.m_peakValue
                  << " quality=" << detection.m_qualityScore
                  << " radius=" << detection.m_radius
                  << " round=" << detection.m_roundness
                  << " saturated=" << (detection.m_saturated ? "true" : "false")
                  << " solved=" << (detection.m_solved ? "true" : "false");
        if (detection.m_solved)
        {
            std::cout << " label=" << detection.m_label.toStdString()
                      << " mag=" << detection.m_catalogMagnitude
                      << " distance=" << detection.m_matchDistancePixels
                      << " projectedX=" << detection.m_projectedCenter.x()
                      << " projectedY=" << detection.m_projectedCenter.y();
        }
        std::cout << '\n';
    }
}

void printExpectedStarDiagnostics(const StarTestCase& test,
                                  const CameraPipelineFramePtr& frame)
{
    if (!frame || test.expectedStars.isEmpty()) {
        return;
    }

    const QVector<CatalogStar> catalog = loadDiagnosticCatalog();
    const TestProjector projector = createDiagnosticProjector(test, frame);
    const QDateTime solveDateTimeUtc = test.dateTime.toUTC();
    std::cout << "  expected-star projections"
              << (frame->m_plateSolved ? " (solved pose):\n" : " (input pose):\n");

    for (const QString& expected : test.expectedStars)
    {
        const CatalogStar *star = findDiagnosticStar(catalog, expected);
        if (!star)
        {
            std::cout << "    " << expected.toStdString() << ": not found in diagnostic catalog\n";
            continue;
        }

        const AzAlt azAlt = Astronomy::raDecToAzAlt(
            RADec{star->rightAscensionDegrees / 15.0, star->declinationDegrees},
            test.latitude,
            test.longitude,
            solveDateTimeUtc,
            true);
        QPointF projected;
        const bool projectedOk = projectDiagnosticVector(
            projector,
            normalize(vectorFromAltAz(azAlt.az, azAlt.alt)),
            projected);

        double nearestDistance = std::numeric_limits<double>::infinity();
        int nearestIndex = -1;
        for (int i = 0; projectedOk && (i < frame->m_starDetections.size()); ++i)
        {
            const CameraPipelineStarDetection& detection = frame->m_starDetections.at(i);
            const double dx = detection.m_center.x() - projected.x();
            const double dy = detection.m_center.y() - projected.y();
            const double distance = std::hypot(dx, dy);
            if (distance < nearestDistance)
            {
                nearestDistance = distance;
                nearestIndex = i;
            }
        }

        std::cout << "    " << star->name.toStdString()
                  << " mag=" << star->magnitude
                  << " az=" << azAlt.az
                  << " el=" << azAlt.alt;
        if (projectedOk)
        {
            std::cout << " x=" << projected.x()
                      << " y=" << projected.y()
                      << " nearest=#" << nearestIndex
                      << " nearestDistance=" << nearestDistance;
            if ((nearestIndex >= 0) && (nearestIndex < frame->m_starDetections.size()))
            {
                const CameraPipelineStarDetection& nearest = frame->m_starDetections.at(nearestIndex);
                if (!nearest.m_label.isEmpty()) {
                    std::cout << " nearestLabel=" << nearest.m_label.toStdString();
                }
            }
        }
        else
        {
            std::cout << " notProjected";
        }
        std::cout << '\n';
    }
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

    const QVector<CatalogStar> diagnosticCatalog = loadDiagnosticCatalog();

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
        const QStringList projectedDetections = projectedExpectedStarDetections(
            test,
            result.frame,
            diagnosticCatalog,
            labels);
        const QStringList missing = missingExpectedStars(labels, projectedDetections, test.expectedStars);
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
                  << " poseAz=" << result.frame->m_plateSolveAzimuth
                  << " poseEl=" << result.frame->m_plateSolveElevation
                  << " poseRoll=" << result.frame->m_plateSolveRoll
                  << " poseFov=" << result.frame->m_plateSolveFov
                  << '\n';
        std::cout << "  labels: " << labels.join(QStringLiteral(", ")).toStdString() << '\n';
        if (!projectedDetections.isEmpty()) {
            std::cout << "  projected detections: " << projectedDetections.join(QStringLiteral(", ")).toStdString() << '\n';
        }
        if (!missing.isEmpty()) {
            std::cout << "  missing: " << missing.join(QStringLiteral(", ")).toStdString() << '\n';
            printExpectedStarDiagnostics(test, result.frame);
            printDetectionDiagnostics(result.frame);
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
