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

#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QGuiApplication>
#include <QHash>
#include <QImage>
#include <QLocale>
#include <QPainter>
#include <QRegularExpression>
#include <QStringList>
#include <QTextStream>
#include <QTimer>
#include <QStandardPaths>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "camera.h"
#include "camerastardetector.h"
#include "util/astronomy.h"
#include "util/messagequeue.h"

MESSAGE_CLASS_DEFINITION(Camera::MsgConfigureCamera, Message)
MESSAGE_CLASS_DEFINITION(Camera::MsgProcessFrame, Message)
MESSAGE_CLASS_DEFINITION(Camera::MsgCaptureActive, Message)

// The test executable links cameradetector.cpp but not camera.cpp, so the
// pipeline-frame helpers used by the detector are duplicated here verbatim
// (same pattern as the message definitions above).
static int discardQueuedProcessFramesImpl(MessageQueue& queue, bool requireCaptureActive)
{
    QList<Message*> messages;
    Message *message = nullptr;
    bool hasCaptureActive = false;

    while ((message = queue.pop()) != nullptr)
    {
        hasCaptureActive = hasCaptureActive || Camera::MsgCaptureActive::match(*message);
        messages.append(message);
    }

    int dropped = 0;

    for (Message *queuedMessage : messages)
    {
        if ((!requireCaptureActive || hasCaptureActive) && Camera::MsgProcessFrame::match(*queuedMessage))
        {
            delete queuedMessage;
            ++dropped;
        }
        else
        {
            queue.push(queuedMessage, false);
        }
    }

    return dropped;
}

int Camera::discardQueuedProcessFrames(MessageQueue& queue)
{
    return discardQueuedProcessFramesImpl(queue, false);
}

int Camera::discardQueuedProcessFramesOnCaptureActive(MessageQueue& queue)
{
    return discardQueuedProcessFramesImpl(queue, true);
}

bool Camera::acceptsPipelineFrame(const CameraPipelineFramePtr& frame, bool captureActive, quint64 captureEpoch)
{
    return frame
        && (frame->m_manualPreviewFrame || (captureActive && (frame->m_captureEpoch == captureEpoch)));
}

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
    double maxMagnitude = std::numeric_limits<double>::quiet_NaN();
    double expectedRoll = std::numeric_limits<double>::quiet_NaN();
    double expectedRollTolerance = std::numeric_limits<double>::quiet_NaN();
    CameraSettings::PlateSolveStartMode plateSolveStartMode = CameraSettings::PlateSolveStartFovAzElRollLens;
    int roiX = 0;
    int roiY = 0;
    int roiWidth = 0;
    int roiHeight = 0;
    QStringList expectedStars;
    QHash<QString, QPointF> expectedStarPositions;
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

double parseOptionalDouble(const QStringList& header,
                           const QStringList& values,
                           const QString& name,
                           int lineNumber,
                           bool* ok)
{
    const int index = header.indexOf(name);
    if ((index < 0) || (index >= values.size())) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const QString value = values.at(index).trimmed();
    if (value.isEmpty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return parseDouble(value, name, lineNumber, ok);
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

    // CSV timestamps are UTC. Interpret the parsed wall-clock as UTC (not the test
    // machine's local time) so results are reproducible regardless of the machine
    // timezone -- QDateTime::fromString defaults to Qt::LocalTime, which previously
    // made the solve depend on where the test was run.
    dateTime.setTimeSpec(Qt::UTC);
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

QHash<QString, QPointF> parseExpectedStarPositions(const QString& value, int lineNumber, bool* ok)
{
    QHash<QString, QPointF> positions;
    const QString trimmedValue = value.trimmed();
    if (trimmedValue.isEmpty()) {
        return positions;
    }

    const QStringList parts = QString(trimmedValue).replace(QLatin1Char(';'), QLatin1Char(','))
        .split(QLatin1Char(','), Qt::SkipEmptyParts);
    for (const QString& part : parts)
    {
        const QString trimmed = part.trimmed();
        const int ySeparator = trimmed.lastIndexOf(QLatin1Char(':'));
        const int xSeparator = ySeparator > 0 ? trimmed.lastIndexOf(QLatin1Char(':'), ySeparator - 1) : -1;
        if ((xSeparator <= 0) || (ySeparator <= xSeparator))
        {
            std::cerr << "Line " << lineNumber << ": invalid starPositions entry: "
                      << trimmed.toStdString() << '\n';
            if (ok) {
                *ok = false;
            }
            continue;
        }

        const QString name = trimmed.left(xSeparator).trimmed();
        bool xOk = false;
        bool yOk = false;
        const double x = QLocale::c().toDouble(trimmed.mid(xSeparator + 1, ySeparator - xSeparator - 1).trimmed(), &xOk);
        const double y = QLocale::c().toDouble(trimmed.mid(ySeparator + 1).trimmed(), &yOk);
        if (name.isEmpty() || !xOk || !yOk)
        {
            std::cerr << "Line " << lineNumber << ": invalid starPositions entry: "
                      << trimmed.toStdString() << '\n';
            if (ok) {
                *ok = false;
            }
            continue;
        }

        positions.insert(name.trimmed().toCaseFolded(), QPointF(x, y));
    }

    return positions;
}

QString compactName(const QString& value)
{
    QString compact;
    compact.reserve(value.size());
    for (const QChar ch : value.trimmed().toCaseFolded())
    {
        if (ch.isLetterOrNumber()) {
            compact.append(ch);
        }
    }
    return compact;
}

CameraSettings::PlateSolveStartMode parsePlateSolveStartMode(const QString& value, int lineNumber, bool* ok)
{
    const QString trimmed = value.trimmed();
    if (trimmed.isEmpty()) {
        return CameraSettings::PlateSolveStartFovAzElRollLens;
    }

    bool intOk = false;
    const int modeIndex = QLocale::c().toInt(trimmed, &intOk);
    if (intOk)
    {
        if ((modeIndex >= static_cast<int>(CameraSettings::PlateSolveStartBlind))
            && (modeIndex <= static_cast<int>(CameraSettings::PlateSolveStartCurrentSettingsOnly)))
        {
            return static_cast<CameraSettings::PlateSolveStartMode>(modeIndex);
        }

        std::cerr << "Line " << lineNumber << ": invalid plateSolveStartMode index: "
                  << trimmed.toStdString() << '\n';
        if (ok) {
            *ok = false;
        }
        return CameraSettings::PlateSolveStartFovAzElRollLens;
    }

    const QString normalized = compactName(trimmed);
    if ((normalized == QStringLiteral("blind"))
        || (normalized == QStringLiteral("platesolvestartblind")))
    {
        return CameraSettings::PlateSolveStartBlind;
    }
    if ((normalized == QStringLiteral("fov"))
        || (normalized == QStringLiteral("platesolvestartfov")))
    {
        return CameraSettings::PlateSolveStartFov;
    }
    if ((normalized == QStringLiteral("fovelevation"))
        || (normalized == QStringLiteral("fovel"))
        || (normalized == QStringLiteral("platesolvestartfovelevation")))
    {
        return CameraSettings::PlateSolveStartFovElevation;
    }
    if ((normalized == QStringLiteral("fovazel"))
        || (normalized == QStringLiteral("fovazimuthelevation"))
        || (normalized == QStringLiteral("platesolvestartfovazel")))
    {
        return CameraSettings::PlateSolveStartFovAzEl;
    }
    if ((normalized == QStringLiteral("fovazelroll"))
        || (normalized == QStringLiteral("fovazimuthelevationroll"))
        || (normalized == QStringLiteral("platesolvestartfovazelroll")))
    {
        return CameraSettings::PlateSolveStartFovAzElRoll;
    }
    if ((normalized == QStringLiteral("fovazelrolllens"))
        || (normalized == QStringLiteral("fovazimuthelevationrolllens"))
        || (normalized == QStringLiteral("platesolvestartfovazelrolllens")))
    {
        return CameraSettings::PlateSolveStartFovAzElRollLens;
    }
    if ((normalized == QStringLiteral("currentsettings"))
        || (normalized == QStringLiteral("currentsettingsonly"))
        || (normalized == QStringLiteral("platesolvestartcurrentsettingsonly")))
    {
        return CameraSettings::PlateSolveStartCurrentSettingsOnly;
    }

    std::cerr << "Line " << lineNumber << ": invalid plateSolveStartMode: "
              << trimmed.toStdString() << '\n';
    if (ok) {
        *ok = false;
    }
    return CameraSettings::PlateSolveStartFovAzElRollLens;
}

double effectivePlateSolveMaxMagnitude(const StarTestCase& test)
{
    const double fallbackMaxMagnitude = (test.fov <= 5.0)
        ? 20.0
        : CameraSettings::m_maxPlateSolveMagnitude;
    return std::isfinite(test.maxMagnitude) ? test.maxMagnitude : fallbackMaxMagnitude;
}

double wrappedAngleDistanceDegrees(double a, double b)
{
    double difference = std::fmod(a - b + 180.0, 360.0);
    if (difference < 0.0) {
        difference += 360.0;
    }
    return std::abs(difference - 180.0);
}

QString normalizedStarName(const QString& value)
{
    // simplified() trims and collapses internal whitespace runs to a single space, so
    // catalog/CSV name spacing differences (e.g. "11    Sgr" vs "11 Sgr") still match.
    return value.simplified().toCaseFolded();
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

bool loadDiagnosticCatalogFile(const QString& path, QVector<CatalogStar>& stars)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return false;
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

    return true;
}

QStringList diagnosticCatalogPaths()
{
    QStringList paths;
    const QString appData = QString::fromLocal8Bit(qgetenv("APPDATA"));
    if (!appData.isEmpty()) {
        paths.append(QDir(appData).filePath(QStringLiteral("f4exb/SDRangel/camera/hyg_v42_reduced.txt")));
    }

    const QString genericData = QStandardPaths::writableLocation(QStandardPaths::GenericDataLocation);
    if (!genericData.isEmpty()) {
        paths.append(QDir(genericData).filePath(QStringLiteral("f4exb/SDRangel/camera/hyg_v42_reduced.txt")));
    }

    paths.append(QStringLiteral(":/camera/brightstarcatalog.txt"));
    paths.removeDuplicates();
    return paths;
}

QVector<CatalogStar> loadDiagnosticCatalog()
{
    QVector<CatalogStar> stars;
    for (const QString& path : diagnosticCatalogPaths())
    {
        if (loadDiagnosticCatalogFile(path, stars) && !stars.isEmpty()) {
            return stars;
        }
    }

    return stars;
}

const CatalogStar* findDiagnosticStar(const QVector<CatalogStar>& catalog, const QString& expected)
{
    const QString normalizedExpected = normalizedStarName(expected);
    const bool exactOnly = normalizedExpected.startsWith(QStringLiteral("hip"));
    for (const CatalogStar& star : catalog)
    {
        const QString normalizedLabel = normalizedStarName(star.name);
        if (normalizedLabel == normalizedExpected) {
            return &star;
        }
    }
    if (exactOnly) {
        return nullptr;
    }
    for (const CatalogStar& star : catalog)
    {
        const QString normalizedLabel = normalizedStarName(star.name);
        if (normalizedLabel.contains(normalizedExpected) || normalizedExpected.contains(normalizedLabel)) {
            return &star;
        }
    }
    return nullptr;
}

QVector<const CatalogStar*> findDiagnosticStarCandidates(const QVector<CatalogStar>& catalog, const QString& expected)
{
    QVector<const CatalogStar*> candidates;
    const QString normalizedExpected = normalizedStarName(expected);
    const bool exactOnly = normalizedExpected.startsWith(QStringLiteral("hip"));
    for (const CatalogStar& star : catalog)
    {
        const QString normalizedLabel = normalizedStarName(star.name);
        if (normalizedLabel == normalizedExpected) {
            candidates.append(&star);
        }
    }
    if (!candidates.isEmpty() || exactOnly) {
        return candidates;
    }

    for (const CatalogStar& star : catalog)
    {
        const QString normalizedLabel = normalizedStarName(star.name);
        if (normalizedLabel.contains(normalizedExpected) || normalizedExpected.contains(normalizedLabel)) {
            candidates.append(&star);
        }
    }
    return candidates;
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

    // Note: when the solve fails, this falls back to the recorded test.roll, but that value
    // is only meaningful ground truth when test.plateSolveStartMode includes roll
    // (FovAzElRoll/FovAzElRollLens). For lower start modes (e.g. FovAzEl=3, used by
    // narrow-7), test.roll is an unused placeholder and the resulting projector's
    // orientation should not be trusted for diagnostics.
    const double azimuth = frame->m_plateSolve.m_solved ? frame->m_plateSolve.m_azimuth : test.azimuth;
    const double elevation = frame->m_plateSolve.m_solved ? frame->m_plateSolve.m_elevation : test.elevation;
    const double roll = frame->m_plateSolve.m_solved ? frame->m_plateSolve.m_roll : test.roll;
    const double fov = frame->m_plateSolve.m_solved ? frame->m_plateSolve.m_fov : test.fov;
    const double centerOffsetX = frame->m_plateSolve.m_solved ? frame->m_plateSolve.m_centerOffsetX : test.centerOffsetX;
    const double centerOffsetY = frame->m_plateSolve.m_solved ? frame->m_plateSolve.m_centerOffsetY : test.centerOffsetY;
    const double distortionK1 = frame->m_plateSolve.m_solved ? frame->m_plateSolve.m_distortionK1 : test.distortionK1;

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
        int maxMagnitudeIndex = header.indexOf(QStringLiteral("plateSolveMaxMagnitude"));
        if (maxMagnitudeIndex < 0) {
            maxMagnitudeIndex = header.indexOf(QStringLiteral("maxMagnitude"));
        }
        if (maxMagnitudeIndex >= 0)
        {
            const QString maxMagnitude = fields.value(maxMagnitudeIndex).trimmed();
            if (!maxMagnitude.isEmpty())
            {
                test.maxMagnitude = parseDouble(maxMagnitude, QStringLiteral("plateSolveMaxMagnitude"), lineNumber, &ok);
                if ((test.maxMagnitude < CameraSettings::m_minPlateSolveMagnitude)
                    || (test.maxMagnitude > CameraSettings::m_maxPlateSolveMagnitude))
                {
                    std::cerr << "Line " << lineNumber << ": plateSolveMaxMagnitude out of range: "
                              << maxMagnitude.toStdString() << '\n';
                    ok = false;
                }
            }
        }
        const int plateSolveStartModeIndex = header.indexOf(QStringLiteral("plateSolveStartMode"));
        if (plateSolveStartModeIndex >= 0) {
            test.plateSolveStartMode = parsePlateSolveStartMode(fields.value(plateSolveStartModeIndex), lineNumber, &ok);
        }
        // Optional ROI columns — default to 0 (full frame) when absent
        auto parseOptionalInt = [&](const QString& colName) -> int {
            const int idx = header.indexOf(colName);
            if (idx < 0 || idx >= fields.size()) {
                return 0;
            }
            const QString val = fields.at(idx).trimmed();
            if (val.isEmpty()) {
                return 0;
            }
            bool intOk = false;
            const int result = QLocale::c().toInt(val, &intOk);
            if (!intOk) {
                std::cerr << "Line " << lineNumber << ": invalid integer for "
                          << colName.toStdString() << ": " << val.toStdString() << '\n';
                ok = false;
            }
            return result;
        };
        test.roiX      = parseOptionalInt(QStringLiteral("roiX"));
        test.roiY      = parseOptionalInt(QStringLiteral("roiY"));
        test.roiWidth  = parseOptionalInt(QStringLiteral("roiW"));
        test.roiHeight = parseOptionalInt(QStringLiteral("roiH"));
        test.expectedStars = parseExpectedStars(fieldValue(header, fields, QStringLiteral("stars"), &ok));
        const int starPositionsIndex = header.indexOf(QStringLiteral("starPositions"));
        if (starPositionsIndex >= 0) {
            test.expectedStarPositions = parseExpectedStarPositions(fields.value(starPositionsIndex), lineNumber, &ok);
        }
        test.expectedRoll = parseOptionalDouble(header, fields, QStringLiteral("expectedRoll"), lineNumber, &ok);
        test.expectedRollTolerance = parseOptionalDouble(header, fields, QStringLiteral("expectedRollTolerance"), lineNumber, &ok);
        if (std::isfinite(test.expectedRollTolerance) && (test.expectedRollTolerance < 0.0))
        {
            std::cerr << "Line " << lineNumber << ": expectedRollTolerance cannot be negative\n";
            ok = false;
        }

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
    settings.m_plateSolveUseCaptureDateTime = false;
    settings.m_plateSolveDateTime = test.dateTime;
    // CSV timestamps are UTC, so tell the solver to interpret m_plateSolveDateTime as
    // UTC rather than (test-machine) local time -- makes results machine-independent.
    settings.m_plateSolveDateTimeUtc = true;
    settings.m_plateSolveUseDownloadedCatalog = true;
    settings.m_plateSolveCatalogSource = CameraSettings::PlateSolveCatalogAuto;
    settings.m_plateSolveStartMode = test.plateSolveStartMode;
    settings.m_plateSolveLabelMode = CameraSettings::PlateSolveLabelName;
    settings.m_plateSolveMaxMagnitude = effectivePlateSolveMaxMagnitude(test);
    settings.m_plateSolveMinMatches = 4;
    settings.m_plateSolveMatchRadius = 24.0;
    settings.m_plateSolveFinalMatchRadius = 24.0;
    settings.m_plateSolveSearchRadius = (test.fov > 30.0)
        ? 24.0
        : static_cast<float>(std::max(3.5, test.fov * 5.0));
    settings.m_detectionRoiX      = test.roiX;
    settings.m_detectionRoiY      = test.roiY;
    settings.m_detectionRoiWidth  = test.roiWidth;
    settings.m_detectionRoiHeight = test.roiHeight;
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
            if (Camera::MsgProcessFrame::match(*message))
            {
                const Camera::MsgProcessFrame& frameMessage =
                    static_cast<const Camera::MsgProcessFrame&>(*message);
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
    detector.getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(
        makeSettings(test),
        QList<QString>(),
        true));
    // Pipeline stages reject frames unless a capture is active with a matching epoch
    // (Camera::acceptsPipelineFrame), so mark capture active before submitting the
    // test frame (which carries the default epoch of 0).
    detector.getInputMessageQueue()->push(Camera::MsgCaptureActive::create(true, 0));

    CameraPipelineFramePtr frame(new CameraPipelineFrame);
    frame->m_image = image;
    frame->m_unprocessedImage = image;
    frame->m_captureDateTime = test.dateTime;
    detector.getInputMessageQueue()->push(Camera::MsgProcessFrame::create(frame));

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

const CameraPipelineStarDetection* findSolvedDetectionForExpectedStar(const CameraPipelineFramePtr& frame,
                                                                      const QString& expected)
{
    if (!frame) {
        return nullptr;
    }

    for (const CameraPipelineStarDetection& detection : frame->m_starDetections)
    {
        if (detection.m_solved && labelMatchesExpectedStar(detection.m_label, expected)) {
            return &detection;
        }
    }

    return nullptr;
}

const CameraPipelineStarDetection* findSolvedDetectionForExpectedStarNearPosition(const CameraPipelineFramePtr& frame,
                                                                                  const QString& expected,
                                                                                  const QPointF& expectedPosition)
{
    if (!frame) {
        return nullptr;
    }

    const CameraPipelineStarDetection *nearestDetection = nullptr;
    double nearestDistance = std::numeric_limits<double>::infinity();
    for (const CameraPipelineStarDetection& detection : frame->m_starDetections)
    {
        if (!detection.m_solved || !labelMatchesExpectedStar(detection.m_label, expected)) {
            continue;
        }

        const double distance = std::hypot(
            detection.m_center.x() - expectedPosition.x(),
            detection.m_center.y() - expectedPosition.y());
        if (distance < nearestDistance)
        {
            nearestDistance = distance;
            nearestDetection = &detection;
        }
    }

    return nearestDetection;
}

const CameraPipelineStarDetection* findMatchSummaryDetectionForExpectedStarNearPosition(const CameraPipelineFramePtr& frame,
                                                                                        const QString& expected,
                                                                                        const QPointF& expectedPosition)
{
    if (!frame || frame->m_plateSolve.m_matchSummary.trimmed().isEmpty()) {
        return nullptr;
    }

    static const QRegularExpression matchPattern(QStringLiteral("^#(\\d+)\\s+(.+?)\\s+d="));
    const CameraPipelineStarDetection *nearestDetection = nullptr;
    double nearestDistance = std::numeric_limits<double>::infinity();
    const QStringList parts = frame->m_plateSolve.m_matchSummary.split(QLatin1Char(';'), Qt::SkipEmptyParts);
    for (const QString& part : parts)
    {
        const QRegularExpressionMatch match = matchPattern.match(part.trimmed());
        if (!match.hasMatch()) {
            continue;
        }

        bool indexOk = false;
        const int detectionIndex = match.captured(1).toInt(&indexOk);
        if (!indexOk || (detectionIndex < 0) || (detectionIndex >= frame->m_starDetections.size())) {
            continue;
        }

        if (!labelMatchesExpectedStar(match.captured(2), expected)) {
            continue;
        }

        const CameraPipelineStarDetection& detection = frame->m_starDetections.at(detectionIndex);
        const double distance = std::hypot(
            detection.m_center.x() - expectedPosition.x(),
            detection.m_center.y() - expectedPosition.y());
        if (distance < nearestDistance)
        {
            nearestDistance = distance;
            nearestDetection = &detection;
        }
    }

    return nearestDetection;
}

const CameraPipelineStarDetection* findProjectedDetectionForExpectedStar(const StarTestCase& test,
                                                                         const CameraPipelineFramePtr& frame,
                                                                         const QVector<CatalogStar>& catalog,
                                                                         const QString& expected)
{
    if (!frame) {
        return nullptr;
    }

    const CatalogStar *star = findDiagnosticStar(catalog, expected);
    if (!star) {
        return nullptr;
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
        return nullptr;
    }

    double nearestDistance = std::numeric_limits<double>::infinity();
    const CameraPipelineStarDetection *nearestDetection = nullptr;
    for (const CameraPipelineStarDetection& detection : frame->m_starDetections)
    {
        const double dx = detection.m_center.x() - projected.x();
        const double dy = detection.m_center.y() - projected.y();
        const double distance = std::hypot(dx, dy);
        if (distance < nearestDistance)
        {
            nearestDistance = distance;
            nearestDetection = &detection;
        }
    }

    const QSize imageSize = frame->m_image.size();
    const double maxImageDimension = std::max(imageSize.width(), imageSize.height());
    const double tolerancePixels = std::max(72.0, std::min(128.0, maxImageDimension * 0.05));
    return (nearestDistance <= tolerancePixels) ? nearestDetection : nullptr;
}

const CameraPipelineStarDetection* findProjectedDetectionForExpectedStarNearPosition(const StarTestCase& test,
                                                                                     const CameraPipelineFramePtr& frame,
                                                                                     const QVector<CatalogStar>& catalog,
                                                                                     const QString& expected,
                                                                                     const QPointF& expectedPosition)
{
    if (!frame) {
        return nullptr;
    }

    const QVector<const CatalogStar*> stars = findDiagnosticStarCandidates(catalog, expected);
    if (stars.isEmpty()) {
        return nullptr;
    }

    const TestProjector projector = createDiagnosticProjector(test, frame);
    const QDateTime solveDateTimeUtc = test.dateTime.toUTC();
    const CatalogStar *bestStar = nullptr;
    QPointF bestProjected;
    double bestProjectedDistance = std::numeric_limits<double>::infinity();
    for (const CatalogStar *star : stars)
    {
        const AzAlt azAlt = Astronomy::raDecToAzAlt(
            RADec{star->rightAscensionDegrees / 15.0, star->declinationDegrees},
            test.latitude,
            test.longitude,
            solveDateTimeUtc,
            true);

        QPointF projected;
        if (!projectDiagnosticVector(projector, normalize(vectorFromAltAz(azAlt.az, azAlt.alt)), projected)) {
            continue;
        }

        const double projectedDistance = std::hypot(
            projected.x() - expectedPosition.x(),
            projected.y() - expectedPosition.y());
        if (projectedDistance < bestProjectedDistance)
        {
            bestProjectedDistance = projectedDistance;
            bestProjected = projected;
            bestStar = star;
        }
    }
    if (!bestStar) {
        return nullptr;
    }

    double nearestDistance = std::numeric_limits<double>::infinity();
    const CameraPipelineStarDetection *nearestDetection = nullptr;
    for (const CameraPipelineStarDetection& detection : frame->m_starDetections)
    {
        const double distance = std::hypot(
            detection.m_center.x() - bestProjected.x(),
            detection.m_center.y() - bestProjected.y());
        if (distance < nearestDistance)
        {
            nearestDistance = distance;
            nearestDetection = &detection;
        }
    }

    const double tolerancePixels = std::max(24.0, std::min(72.0, std::max(48.0, bestProjectedDistance + 24.0)));
    return (nearestDistance <= tolerancePixels) ? nearestDetection : nullptr;
}

const CameraPipelineStarDetection* findSolvedDetectionNearPosition(const CameraPipelineFramePtr& frame,
                                                                   const QPointF& expectedPosition)
{
    if (!frame) {
        return nullptr;
    }

    static constexpr double kPositionTolerancePixels = 24.0;
    const CameraPipelineStarDetection *nearestDetection = nullptr;
    double nearestDistance = std::numeric_limits<double>::infinity();
    for (const CameraPipelineStarDetection& detection : frame->m_starDetections)
    {
        if (!detection.m_solved || detection.m_label.trimmed().isEmpty()) {
            continue;
        }

        const double distance = std::hypot(
            detection.m_center.x() - expectedPosition.x(),
            detection.m_center.y() - expectedPosition.y());
        if (distance < nearestDistance)
        {
            nearestDistance = distance;
            nearestDetection = &detection;
        }
    }

    return (nearestDistance <= kPositionTolerancePixels) ? nearestDetection : nullptr;
}

QStringList expectedStarPositionMismatches(const StarTestCase& test,
                                           const CameraPipelineFramePtr& frame,
                                           const QVector<CatalogStar>& catalog)
{
    QStringList mismatches;
    static constexpr double kPositionTolerancePixels = 24.0;

    for (auto it = test.expectedStarPositions.constBegin(); it != test.expectedStarPositions.constEnd(); ++it)
    {
        const QString expected = it.key();
        const QPointF& expectedPosition = it.value();

        const CameraPipelineStarDetection *detection =
            findSolvedDetectionNearPosition(frame, expectedPosition);
        if (!detection) {
            detection = findSolvedDetectionForExpectedStarNearPosition(frame, expected, expectedPosition);
        }
        if (!detection) {
            detection = findMatchSummaryDetectionForExpectedStarNearPosition(frame, expected, expectedPosition);
        }
        if (!detection) {
            detection = findProjectedDetectionForExpectedStarNearPosition(test, frame, catalog, expected, expectedPosition);
        }
        if (!detection)
        {
            mismatches.append(QStringLiteral("%1 missing labelled match near x=%2 y=%3")
                .arg(expected)
                .arg(expectedPosition.x(), 0, 'f', 1)
                .arg(expectedPosition.y(), 0, 'f', 1));
            continue;
        }

        const double distance = std::hypot(detection->m_center.x() - expectedPosition.x(), detection->m_center.y() - expectedPosition.y());
        if (distance > kPositionTolerancePixels)
        {
            mismatches.append(QStringLiteral("%1 expected x=%2 y=%3 matched x=%4 y=%5 distance=%6")
                .arg(expected)
                .arg(expectedPosition.x(), 0, 'f', 1)
                .arg(expectedPosition.y(), 0, 'f', 1)
                .arg(detection->m_center.x(), 0, 'f', 1)
                .arg(detection->m_center.y(), 0, 'f', 1)
                .arg(distance, 0, 'f', 1));
        }
    }

    return mismatches;
}

// An expected star carrying an explicit pixel position is "found" when a solved detection
// lands within the position tolerance of it — regardless of what name the solver attached.
// This reconciles synonym labelling: a bright star the solver labels by its proper name
// (e.g. "Pollux") still satisfies a CSV ground-truth that names it by catalogue id
// ("HIP 37826"), since the position is the authoritative ground truth and the HYG
// diagnostic catalogue cannot cross-reference HIP ids to proper names.
bool expectedStarPositionSatisfied(const StarTestCase& test,
                                   const CameraPipelineFramePtr& frame,
                                   const QVector<CatalogStar>& catalog,
                                   const QString& expected)
{
    const auto it = test.expectedStarPositions.constFind(expected);
    if (it == test.expectedStarPositions.constEnd()) {
        return false;
    }
    const QPointF& expectedPosition = it.value();

    const CameraPipelineStarDetection *detection =
        findSolvedDetectionNearPosition(frame, expectedPosition);
    if (!detection) {
        detection = findSolvedDetectionForExpectedStarNearPosition(frame, expected, expectedPosition);
    }
    if (!detection) {
        detection = findMatchSummaryDetectionForExpectedStarNearPosition(frame, expected, expectedPosition);
    }
    if (!detection) {
        detection = findProjectedDetectionForExpectedStarNearPosition(test, frame, catalog, expected, expectedPosition);
    }
    if (!detection) {
        return false;
    }

    const double distance = std::hypot(
        detection->m_center.x() - expectedPosition.x(),
        detection->m_center.y() - expectedPosition.y());
    return distance <= 24.0;
}

QStringList expectedPoseMismatches(const StarTestCase& test, const CameraPipelineFramePtr& frame)
{
    QStringList mismatches;
    if (!std::isfinite(test.expectedRoll)) {
        return mismatches;
    }

    const double tolerance = std::isfinite(test.expectedRollTolerance)
        ? test.expectedRollTolerance
        : 5.0;
    const double rollError = wrappedAngleDistanceDegrees(frame->m_plateSolve.m_roll, test.expectedRoll);
    if (rollError > tolerance)
    {
        mismatches.append(QStringLiteral("roll expected=%1 actual=%2 error=%3 tolerance=%4")
            .arg(test.expectedRoll, 0, 'f', 2)
            .arg(frame->m_plateSolve.m_roll, 0, 'f', 2)
            .arg(rollError, 0, 'f', 2)
            .arg(tolerance, 0, 'f', 2));
    }

    return mismatches;
}

QStringList requiredExpectedStars(const StarTestCase& test)
{
    if (test.expectedStarPositions.isEmpty()) {
        return test.expectedStars;
    }

    QStringList stars;
    stars.reserve(test.expectedStarPositions.size());
    for (auto it = test.expectedStarPositions.constBegin(); it != test.expectedStarPositions.constEnd(); ++it) {
        stars.append(it.key());
    }
    return stars;
}

bool expectedStarHasNearbyDetection(const StarTestCase& test,
                                    const CameraPipelineFramePtr& frame,
                                    const QVector<CatalogStar>& catalog,
                                    const QString& expected)
{
    return findProjectedDetectionForExpectedStar(test, frame, catalog, expected) != nullptr;
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
                  << " flux=" << detection.m_flux
                  << " snr=" << detection.m_snr
                  << " fwhm=" << detection.m_fwhm
                  << " centroidUncertainty=" << detection.m_centroidUncertainty
                  << " quality=" << detection.m_qualityScore
                  << " radius=" << detection.m_radius
                  << " round=" << detection.m_roundness
                  << " saturated=" << (detection.m_saturated ? "true" : "false")
                  << " hotPixel=" << (detection.m_hotPixelSuspect ? "true" : "false")
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
              << (frame->m_plateSolve.m_solved ? " (solved pose):\n" : " (input pose):\n");

    for (const QString& expected : test.expectedStars)
    {
        QStringList solvedMatches;
        for (int i = 0; i < frame->m_starDetections.size(); ++i)
        {
            const CameraPipelineStarDetection& detection = frame->m_starDetections.at(i);
            if (detection.m_solved && labelMatchesExpectedStar(detection.m_label, expected))
            {
                solvedMatches.append(QStringLiteral("#%1 x=%2 y=%3 label=%4 mag=%5 distance=%6")
                    .arg(i)
                    .arg(detection.m_center.x(), 0, 'f', 1)
                    .arg(detection.m_center.y(), 0, 'f', 1)
                    .arg(detection.m_label)
                    .arg(detection.m_catalogMagnitude, 0, 'f', 2)
                    .arg(detection.m_matchDistancePixels, 0, 'f', 2));
            }
        }
        if (!solvedMatches.isEmpty())
        {
            std::cout << "    " << expected.toStdString()
                      << " solved matches: "
                      << solvedMatches.join(QStringLiteral("; ")).toStdString()
                      << '\n';
        }
        else if (!frame->m_plateSolve.m_matchSummary.trimmed().isEmpty())
        {
            const QStringList parts = frame->m_plateSolve.m_matchSummary.split(QLatin1Char(';'), Qt::SkipEmptyParts);
            QStringList summaryMatches;
            static const QRegularExpression matchPattern(QStringLiteral("^#(\\d+)\\s+(.+?)\\s+d="));
            for (const QString& part : parts)
            {
                const QRegularExpressionMatch match = matchPattern.match(part.trimmed());
                if (match.hasMatch() && labelMatchesExpectedStar(match.captured(2), expected)) {
                    summaryMatches.append(part.trimmed());
                }
            }
            if (!summaryMatches.isEmpty())
            {
                std::cout << "    " << expected.toStdString()
                          << " match-summary matches: "
                          << summaryMatches.join(QStringLiteral("; ")).toStdString()
                          << '\n';
            }
        }

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

QString diagnosticImageFileName(const StarTestCase& test)
{
    QString name = QFileInfo(test.name).completeBaseName();
    if (name.isEmpty()) {
        name = test.name;
    }

    for (QChar& ch : name)
    {
        if (!ch.isLetterOrNumber() && (ch != QLatin1Char('-')) && (ch != QLatin1Char('_'))) {
            ch = QLatin1Char('_');
        }
    }

    return name + QStringLiteral("-matches.png");
}

void drawCrosshair(QPainter& painter, const QPointF& point, int radius)
{
    painter.drawLine(QPointF(point.x() - radius, point.y()), QPointF(point.x() + radius, point.y()));
    painter.drawLine(QPointF(point.x(), point.y() - radius), QPointF(point.x(), point.y() + radius));
}

bool writeMatchOverlayImage(const StarTestCase& test,
                            const CameraPipelineFramePtr& frame,
                            const QString& outputDirectory,
                            QString* outputPath)
{
    if (!frame) {
        return false;
    }

    QImage image = !frame->m_unprocessedImage.isNull() ? frame->m_unprocessedImage : frame->m_image;
    if (image.isNull()) {
        return false;
    }

    image = image.convertToFormat(QImage::Format_RGB32);
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing, true);

    QFont font = painter.font();
    font.setPointSize(std::max(9, image.width() / 260));
    font.setBold(true);
    painter.setFont(font);

    const int markerRadius = std::max(4, image.width() / 500);
    const int labelOffset = markerRadius + 4;

    if ((test.roiWidth > 0) && (test.roiHeight > 0))
    {
        painter.setPen(QPen(QColor(255, 180, 0), std::max(2, image.width() / 900)));
        painter.drawRect(QRect(test.roiX, test.roiY, test.roiWidth, test.roiHeight));
    }

    for (auto it = test.expectedStarPositions.constBegin(); it != test.expectedStarPositions.constEnd(); ++it)
    {
        painter.setPen(QPen(QColor(255, 80, 255), std::max(2, image.width() / 900)));
        drawCrosshair(painter, it.value(), markerRadius + 2);
        const QString label = QStringLiteral("expected %1").arg(it.key());
        const QRectF textRect(QPointF(it.value().x() + labelOffset, it.value().y() + labelOffset),
            QSizeF(image.width() * 0.35, font.pointSizeF() * 2.5));
        painter.setPen(QPen(Qt::black, std::max(3, image.width() / 650)));
        painter.drawText(textRect.translated(1.0, 1.0), Qt::AlignLeft | Qt::AlignTop, label);
        painter.setPen(QPen(QColor(255, 170, 255), 1));
        painter.drawText(textRect, Qt::AlignLeft | Qt::AlignTop, label);
    }

    painter.setPen(QPen(QColor(80, 170, 255, 180), std::max(1, image.width() / 1400)));
    for (const CameraPipelineStarDetection& detection : frame->m_starDetections) {
        drawCrosshair(painter, detection.m_center, std::max(2, markerRadius / 2));
    }

    for (const CameraPipelineStarDetection& detection : frame->m_starDetections)
    {
        if (!detection.m_solved || detection.m_label.trimmed().isEmpty()) {
            continue;
        }

        const QPointF center = detection.m_center;
        const QPointF projected = detection.m_projectedCenter;
        painter.setPen(QPen(QColor(120, 255, 120), std::max(2, image.width() / 900)));
        painter.drawEllipse(center, markerRadius, markerRadius);

        if (!projected.isNull())
        {
            painter.setPen(QPen(QColor(255, 220, 80), std::max(1, image.width() / 1400), Qt::DashLine));
            painter.drawLine(center, projected);
            drawCrosshair(painter, projected, std::max(2, markerRadius / 2));
        }

        const QString label = QStringLiteral("%1  d=%2px")
            .arg(detection.m_label.trimmed())
            .arg(static_cast<double>(detection.m_matchDistancePixels), 0, 'f', 1);
        const QPointF textPos(center.x() + labelOffset, center.y() - labelOffset);
        const QRectF textRect(textPos, QSizeF(image.width() * 0.35, font.pointSizeF() * 2.5));
        painter.setPen(QPen(Qt::black, std::max(3, image.width() / 650)));
        painter.drawText(textRect.translated(1.0, 1.0), Qt::AlignLeft | Qt::AlignTop, label);
        painter.setPen(QPen(QColor(180, 255, 180), 1));
        painter.drawText(textRect, Qt::AlignLeft | Qt::AlignTop, label);
    }

    painter.end();

    QDir outputDir(outputDirectory);
    if (!outputDir.exists() && !outputDir.mkpath(QStringLiteral("."))) {
        return false;
    }

    const QString path = outputDir.filePath(diagnosticImageFileName(test));
    if (!image.save(path)) {
        return false;
    }

    if (outputPath) {
        *outputPath = QFileInfo(path).absoluteFilePath();
    }
    return true;
}

int runTests(const QString& csvPath, const QString& outputDirectory)
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
    QElapsedTimer suiteTimer;
    suiteTimer.start();
    for (const StarTestCase& test : tests)
    {
        QElapsedTimer testTimer;
        testTimer.start();
        const DetectorRunResult result = runDetector(test);
        const qint64 elapsedMs = testTimer.elapsed();
        if (!result.completed)
        {
            ++failures;
            std::cerr << "FAIL " << test.name.toStdString() << ": " << result.error.toStdString()
                      << " timeMs=" << elapsedMs << '\n';
            continue;
        }

        QString overlayPath;
        // STAR_TESTS_NO_OVERLAY skips the per-case diagnostic overlay PNG. The overlays are
        // full-frame images written once per row; for bulk/parallel validation runs (e.g. the
        // seed-jitter sweep with hundreds of rows) they dominate disk writes and are the main
        // thing that fills the disk, so allow turning them off.
        const bool wroteOverlay = !qEnvironmentVariableIsSet("STAR_TESTS_NO_OVERLAY")
            && writeMatchOverlayImage(test, result.frame, outputDirectory, &overlayPath);

        const QStringList labels = solvedStarLabels(result.frame);
        const QStringList projectedDetections = projectedExpectedStarDetections(
            test,
            result.frame,
            diagnosticCatalog,
            labels);
        const QStringList requiredStars = requiredExpectedStars(test);
        QStringList missing = missingExpectedStars(labels, projectedDetections, requiredStars);
        // A required star whose explicit pixel position is satisfied by a solved detection
        // is found even if the solver labelled it with a synonym (proper name vs catalogue
        // id), so don't also report it as name-missing — the position check below is the
        // authoritative assertion for such stars.
        if (!missing.isEmpty() && !test.expectedStarPositions.isEmpty())
        {
            QStringList stillMissing;
            for (const QString& name : missing)
            {
                if (!expectedStarPositionSatisfied(test, result.frame, diagnosticCatalog, name)) {
                    stillMissing.append(name);
                }
            }
            missing = stillMissing;
        }
        const QStringList positionMismatches = expectedStarPositionMismatches(
            test,
            result.frame,
            diagnosticCatalog);
        const QStringList poseMismatches = expectedPoseMismatches(test, result.frame);
        const bool solved = result.frame->m_plateSolve.m_solved;
        const bool pass = solved && missing.isEmpty() && positionMismatches.isEmpty() && poseMismatches.isEmpty();
        if (!pass) {
            ++failures;
        }

        std::cout << (pass ? "PASS " : "FAIL ") << test.name.toStdString()
                  << ": detections=" << result.frame->m_starDetections.size()
                  << " matched=" << result.frame->m_plateSolve.m_matchedStars
                  << " solved=" << (result.frame->m_plateSolve.m_solved ? "true" : "false")
                  << " maxMag=" << effectivePlateSolveMaxMagnitude(test)
                  << " catalog=" << result.frame->m_plateSolve.m_catalogSource.toStdString()
                  << " catalogStars=" << result.frame->m_plateSolve.m_catalogStarsLoaded
                  << " candidates=" << result.frame->m_plateSolve.m_catalogCandidateStars
                  << " outliers=" << result.frame->m_plateSolve.m_outlierStars
                  << " required=" << result.frame->m_plateSolve.m_requiredMatches
                  << " rms=" << result.frame->m_plateSolve.m_rmsError
                  << " poseAz=" << result.frame->m_plateSolve.m_azimuth
                  << " poseEl=" << result.frame->m_plateSolve.m_elevation
                  << " poseRoll=" << result.frame->m_plateSolve.m_roll
                  << " poseFov=" << result.frame->m_plateSolve.m_fov
                  << " timeMs=" << elapsedMs
                  << " stages=" << result.frame->m_plateSolve.m_profileSummary.toStdString()
                  << '\n';
        std::cout << "  labels: " << labels.join(QStringLiteral(", ")).toStdString() << '\n';
        if (wroteOverlay) {
            std::cout << "  overlay: " << overlayPath.toStdString() << '\n';
        }
        if (!projectedDetections.isEmpty()) {
            std::cout << "  projected detections: " << projectedDetections.join(QStringLiteral(", ")).toStdString() << '\n';
        }
        if (!result.frame->m_plateSolve.m_failureReason.isEmpty()) {
            std::cout << "  failure: " << result.frame->m_plateSolve.m_failureReason.toStdString() << '\n';
        }
        if (!result.frame->m_plateSolve.m_matchSummary.isEmpty()) {
            std::cout << "  matches: " << result.frame->m_plateSolve.m_matchSummary.toStdString() << '\n';
        }
        if (!result.frame->m_plateSolve.m_profileSummary.isEmpty()) {
            std::cout << "  stages: " << result.frame->m_plateSolve.m_profileSummary.toStdString() << '\n';
        }
        if (!solved) {
            std::cout << "  plate solve did not complete\n";
        }
        if (result.frame->m_plateSolve.m_matchedStars <= 0) {
            std::cout << "  missing plate-solve matches\n";
        }
        if (!missing.isEmpty()) {
            std::cout << "  missing: " << missing.join(QStringLiteral(", ")).toStdString() << '\n';
            printExpectedStarDiagnostics(test, result.frame);
            printDetectionDiagnostics(result.frame);
        }
        if (!positionMismatches.isEmpty())
        {
            std::cout << "  position mismatches: " << positionMismatches.join(QStringLiteral("; ")).toStdString() << '\n';
            printExpectedStarDiagnostics(test, result.frame);
            printDetectionDiagnostics(result.frame);
        }
        if (!poseMismatches.isEmpty()) {
            std::cout << "  pose mismatches: " << poseMismatches.join(QStringLiteral("; ")).toStdString() << '\n';
        }
    }

    if (failures > 0) {
        std::cerr << failures << " camera star test(s) failed\n";
        return 1;
    }

    std::cout << tests.size() << " camera star test(s) passed"
              << " totalTimeMs=" << suiteTimer.elapsed() << '\n';
    return 0;
}
}

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    QGuiApplication::setOrganizationName(QStringLiteral("f4exb"));
    QGuiApplication::setApplicationName(QStringLiteral("SDRangel"));

    const QStringList args = app.arguments();
    const QString csvPath = (args.size() > 1)
        ? args.at(1)
        : QDir(QString::fromUtf8(CAMERA_STAR_TEST_DATA_DIR)).filePath(QStringLiteral("star-tests.csv"));
    const QString absoluteCsvPath = QFileInfo(csvPath).absoluteFilePath();
    const QString outputDirectory = (args.size() > 2)
        ? QFileInfo(args.at(2)).absoluteFilePath()
        : QFileInfo(absoluteCsvPath).absoluteDir().filePath(QStringLiteral("star-test-output"));
    return runTests(absoluteCsvPath, outputDirectory);
}
