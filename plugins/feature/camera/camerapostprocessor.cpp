///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
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
#include <array>
#include <cmath>

#include <QDebug>
#include <QFileInfo>
#include <QFont>
#include <QFontMetrics>
#include <QPainter>
#include <QTextDocument>
#include "SWGMapItem.h"
#include "util/azel.h"
#include "util/weather.h"
#include "util/profiler.h"
#include "maincore.h"
#include "camerapostprocessor.h"

MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgConfigureCameraPostProcessor, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgProcessFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgSpectrumFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgReportFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgReportSaveVideoState, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgReportSaveImageState, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgSetVideoRecordingEnabled, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgCaptureActive, Message)

namespace {

const QStringList kTrackedObjectPipeURIs = {
    QStringLiteral("sdrangel.channel.adsbdemod"),
    QStringLiteral("sdrangel.feature.satellitetracker"),
    QStringLiteral("sdrangel.feature.startracker")
};

struct EquatorialStar
{
    double rightAscensionDegrees;
    double declinationDegrees;
};

const std::array<EquatorialStar, 7> kUrsaMajorStars = {{
    {165.932083, 61.750833}, // Dubhe
    {165.460417, 56.382500}, // Merak
    {178.457500, 53.694722}, // Phecda
    {183.856667, 57.032500}, // Megrez
    {193.507083, 55.959722}, // Alioth
    {200.981250, 54.925278}, // Mizar
    {206.885000, 49.313333}  // Alkaid
}};

const std::array<EquatorialStar, 7> kOrionStars = {{
    {81.282917, 6.349722},   // Betelgeuse
    {78.634583, -8.201667},  // Rigel
    {88.792917, 7.406944},   // Bellatrix
    {86.939167, -9.669722},  // Saiph
    {84.053333, -1.201944},  // Alnitak
    {83.001667, -0.299167},  // Alnilam
    {81.572917, -2.397222}   // Mintaka
}};

const std::array<EquatorialStar, 4> kCruxStars = {{
    {186.649583, -63.099167}, // Acrux
    {191.930000, -59.688889}, // Mimosa
    {183.786250, -58.748889}, // Gacrux
    {187.791667, -57.113333}  // Delta Crucis
}};

struct SkyVector
{
    double x;
    double y;
    double z;
};

static double degToRad(double value)
{
    static constexpr double kPi = 3.14159265358979323846;
    return value * kPi / 180.0;
}

static double normalizeDegrees(double value)
{
    value = std::fmod(value, 360.0);
    if (value < 0.0) {
        value += 360.0;
    }
    return value;
}

static double julianDateUtc(const QDateTime& utcDateTime)
{
    const QDate date = utcDateTime.date();
    const QTime time = utcDateTime.time();
    int year = date.year();
    int month = date.month();
    if (month <= 2)
    {
        year -= 1;
        month += 12;
    }

    const int a = year / 100;
    const int b = 2 - a + (a / 4);
    const double fractionalDay = (static_cast<double>(time.hour())
        + (static_cast<double>(time.minute())
            + (static_cast<double>(time.second()) + static_cast<double>(time.msec()) / 1000.0) / 60.0) / 60.0) / 24.0;
    const double day = static_cast<double>(date.day()) + fractionalDay;

    return std::floor(365.25 * static_cast<double>(year + 4716))
        + std::floor(30.6001 * static_cast<double>(month + 1))
        + day + static_cast<double>(b) - 1524.5;
}

static double greenwichMeanSiderealDegrees(const QDateTime& utcDateTime)
{
    const double jd = julianDateUtc(utcDateTime);
    const double t = (jd - 2451545.0) / 36525.0;
    const double gmst = 280.46061837
        + 360.98564736629 * (jd - 2451545.0)
        + 0.000387933 * t * t
        - (t * t * t) / 38710000.0;
    return normalizeDegrees(gmst);
}

static QString formatSignedDegrees(double value)
{
    const int rounded = qRound(value);
    return QStringLiteral("%1%2°").arg(rounded >= 0 ? "+" : "").arg(rounded);
}

static QString formatAzimuthDegrees(double value)
{
    return QStringLiteral("%1°").arg(qRound(normalizeDegrees(value)));
}

static QString formatRightAscensionDegrees(double value)
{
    int hours = static_cast<int>(std::round(normalizeDegrees(value) / 15.0)) % 24;
    if (hours < 0) {
        hours += 24;
    }
    return QStringLiteral("%1h").arg(hours, 2, 10, QLatin1Char('0'));
}

static QString formatSolvedStarLabel(const CameraSettings& settings, const CameraPipelineStarDetection& detection)
{
    if (!detection.m_solved || detection.m_label.isEmpty()) {
        return QString();
    }

    QString label = detection.m_label;
    if (settings.m_plateSolveLabelMode >= CameraSettings::PlateSolveLabelNameMagnitude) {
        label += QStringLiteral("\nmag %1").arg(detection.m_catalogMagnitude, 0, 'f', 1);
    }
    if ((settings.m_plateSolveLabelMode >= CameraSettings::PlateSolveLabelNameMagnitudeSpectralType)
        && !detection.m_catalogSpectralType.trimmed().isEmpty())
    {
        label += QStringLiteral("\n%1").arg(detection.m_catalogSpectralType.trimmed());
    }

    return label;
}

static void drawOutlinedLabel(QPainter& painter,
                              const QRect& imageRect,
                              const QPointF& point,
                              const QString& text,
                              const QColor& color,
                              const QFontMetrics& fontMetrics)
{
    if (text.isEmpty()) {
        return;
    }

    const QStringList lines = text.split(QChar('\n'));
    int textWidth = 0;
    int lineCount = 0;
    for (const QString& line : lines)
    {
        textWidth = std::max(textWidth, fontMetrics.horizontalAdvance(line));
        lineCount++;
    }
    if (lineCount <= 0) {
        return;
    }

    QPointF labelPoint = point + QPointF(4.0, -4.0);
    QRect targetRect(
        qRound(labelPoint.x()),
        qRound(labelPoint.y()) - lineCount * fontMetrics.lineSpacing(),
        textWidth + 4,
        lineCount * fontMetrics.lineSpacing() + 2);

    if (!imageRect.adjusted(0, 0, -1, -1).intersects(targetRect)) {
        return;
    }

    painter.save();
    auto drawLines = [&](const QPoint& offset, const QColor& penColor)
    {
        painter.setPen(penColor);
        const int baseX = targetRect.left();
        int baselineY = targetRect.top() + fontMetrics.ascent();
        for (const QString& line : lines)
        {
            painter.drawText(baseX + offset.x(), baselineY + offset.y(), line);
            baselineY += fontMetrics.lineSpacing();
        }
    };

    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            if (dx == 0 && dy == 0) {
                continue;
            }
            drawLines(QPoint(dx, dy), Qt::black);
        }
    }
    drawLines(QPoint(0, 0), color);
    painter.restore();
}

static SkyVector vectorFromAltAz(double azimuthDegrees, double elevationDegrees)
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

static double dot(const SkyVector& lhs, const SkyVector& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

static SkyVector cross(const SkyVector& lhs, const SkyVector& rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x
    };
}

static double length(const SkyVector& vector)
{
    return std::sqrt(dot(vector, vector));
}

static SkyVector normalize(const SkyVector& vector)
{
    const double vectorLength = length(vector);
    if (vectorLength <= 0.0) {
        return {0.0, 0.0, 0.0};
    }

    return {
        vector.x / vectorLength,
        vector.y / vectorLength,
        vector.z / vectorLength
    };
}

static SkyVector rotateAroundAxis(const SkyVector& vector, const SkyVector& axis, double angleRadians)
{
    const double cosAngle = std::cos(angleRadians);
    const double sinAngle = std::sin(angleRadians);
    const SkyVector axisCrossVector = cross(axis, vector);
    const double axisDotVector = dot(axis, vector);

    return {
        vector.x * cosAngle + axisCrossVector.x * sinAngle + axis.x * axisDotVector * (1.0 - cosAngle),
        vector.y * cosAngle + axisCrossVector.y * sinAngle + axis.y * axisDotVector * (1.0 - cosAngle),
        vector.z * cosAngle + axisCrossVector.z * sinAngle + axis.z * axisDotVector * (1.0 - cosAngle)
    };
}

static bool equatorialToAltAz(double rightAscensionDegrees,
                              double declinationDegrees,
                              double latitudeDegrees,
                              double longitudeDegrees,
                              const QDateTime& utcDateTime,
                              double& azimuthDegrees,
                              double& elevationDegrees)
{
    const double lstDegrees = normalizeDegrees(greenwichMeanSiderealDegrees(utcDateTime) + longitudeDegrees);
    double hourAngleDegrees = normalizeDegrees(lstDegrees - rightAscensionDegrees);
    if (hourAngleDegrees > 180.0) {
        hourAngleDegrees -= 360.0;
    }

    const double hourAngle = degToRad(hourAngleDegrees);
    const double declination = degToRad(declinationDegrees);
    const double latitude = degToRad(latitudeDegrees);

    const double sinAltitude = std::sin(declination) * std::sin(latitude)
        + std::cos(declination) * std::cos(latitude) * std::cos(hourAngle);
    const double altitude = std::asin(std::clamp(sinAltitude, -1.0, 1.0));

    static constexpr double kPi = 3.14159265358979323846;
    const double azimuth = std::atan2(
        std::sin(hourAngle),
        std::cos(hourAngle) * std::sin(latitude) - std::tan(declination) * std::cos(latitude)) + kPi;

    azimuthDegrees = normalizeDegrees(azimuth * 180.0 / M_PI);
    elevationDegrees = altitude * 180.0 / M_PI;
    return std::isfinite(azimuthDegrees) && std::isfinite(elevationDegrees);
}

struct SkyProjector
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

    static SkyProjector create(const CameraSettings& settings, const QSize& size)
    {
        SkyProjector projector;
        projector.width = size.width();
        projector.height = size.height();
        projector.lensProjection = settings.m_lensProjection;

        if (projector.width <= 0 || projector.height <= 0 || settings.m_fov <= 0.0f) {
            return projector;
        }

        const double azimuth = degToRad(settings.m_azimuth);
        projector.center = normalize(vectorFromAltAz(settings.m_azimuth, settings.m_elevation));
        projector.right = normalize({std::cos(azimuth), -std::sin(azimuth), 0.0});
        projector.up = normalize(cross(projector.right, projector.center));
        if (length(projector.right) <= 0.0 || length(projector.up) <= 0.0) {
            return projector;
        }

        const double rollRadians = degToRad(settings.m_roll);
        if (std::fabs(rollRadians) > 1e-9)
        {
            projector.right = normalize(rotateAroundAxis(projector.right, projector.center, rollRadians));
            projector.up = normalize(rotateAroundAxis(projector.up, projector.center, rollRadians));
        }

        const double halfHorizontalFov = degToRad(settings.m_fov) * 0.5;
        static constexpr double kPi = 3.14159265358979323846;
        if (halfHorizontalFov <= 0.0 || halfHorizontalFov >= (kPi * 0.5)) {
            return projector;
        }

        projector.halfHorizontalFov = halfHorizontalFov;
        const double aspect = static_cast<double>(projector.height) / static_cast<double>(projector.width);
        projector.horizontalScale = 1.0;
        projector.verticalScale = aspect;
        projector.principalPointX = static_cast<double>(projector.width) * 0.5 + settings.m_lensCenterOffsetX;
        projector.principalPointY = static_cast<double>(projector.height) * 0.5 + settings.m_lensCenterOffsetY;
        projector.distortionK1 = settings.m_lensDistortionK1;
        projector.valid = projector.verticalScale > 0.0;
        return projector;
    }

    bool projectAltAz(double azimuthDegrees, double elevationDegrees, QPointF& point) const
    {
        if (!valid) {
            return false;
        }

        const SkyVector vector = vectorFromAltAz(azimuthDegrees, elevationDegrees);
        const double depth = dot(vector, center);
        if (depth <= 0.0) {
            return false;
        }

        const double planeX = dot(vector, right);
        const double planeY = dot(vector, up);
        if (!std::isfinite(planeX) || !std::isfinite(planeY)) {
            return false;
        }

        const double theta = std::acos(std::clamp(depth, -1.0, 1.0));
        const double phi = std::atan2(planeY, planeX);
        const double projectionRadius = [&]() -> double
        {
            switch (lensProjection)
            {
            case CameraSettings::LensProjectionEquidistant:
                return theta / halfHorizontalFov;
            case CameraSettings::LensProjectionEquisolid:
                return std::sin(theta * 0.5) / std::sin(halfHorizontalFov * 0.5);
            case CameraSettings::LensProjectionRectilinear:
            default:
                return std::tan(theta) / std::tan(halfHorizontalFov);
            }
        }();

        double projectedX = std::cos(phi) * projectionRadius;
        double projectedY = std::sin(phi) * projectionRadius;
        if (std::fabs(distortionK1) > 1e-9)
        {
            const double radiusSquared = projectedX * projectedX + projectedY * projectedY;
            const double distortionScale = std::max(0.1, 1.0 + distortionK1 * radiusSquared);
            projectedX *= distortionScale;
            projectedY *= distortionScale;
        }

        const double normalizedX = projectedX / horizontalScale;
        const double normalizedY = projectedY / verticalScale;
        if (!std::isfinite(normalizedX) || !std::isfinite(normalizedY)) {
            return false;
        }

        point.setX(principalPointX + normalizedX * 0.5 * static_cast<double>(width));
        point.setY(principalPointY - normalizedY * 0.5 * static_cast<double>(height));
        return true;
    }
};

template<typename StarArray>
void drawConstellationStars(QPainter& painter,
                            const QImage& image,
                            const SkyProjector& projector,
                            const QDateTime& utcDateTime,
                            const CameraSettings& settings,
                            const StarArray& stars)
{
    for (const EquatorialStar& star : stars)
    {
        double azimuth = 0.0;
        double elevation = 0.0;
        QPointF point;
        if (!equatorialToAltAz(
                star.rightAscensionDegrees,
                star.declinationDegrees,
                settings.m_latitude,
                settings.m_longitude,
                utcDateTime,
                azimuth,
                elevation)
            || !projector.projectAltAz(azimuth, elevation, point))
        {
            continue;
        }

        const QPoint centerPoint(static_cast<int>(std::lround(point.x())), static_cast<int>(std::lround(point.y())));
        if (!image.rect().adjusted(0, 0, -1, -1).contains(centerPoint)) {
            continue;
        }

        painter.drawRect(QRectF(point.x() - 3.0, point.y() - 3.0, 6.0, 6.0));
    }
}

QDateTime plateSolveOverlayDateTime(const CameraSettings& settings, const QDateTime& captureDateTime)
{
    if (settings.m_plateSolveUseCurrentDateTime) {
        return QDateTime::currentDateTime();
    }

    if (settings.m_plateSolveDateTime.isValid()) {
        return settings.m_plateSolveDateTime;
    }

    if (captureDateTime.isValid()) {
        return captureDateTime;
    }

    return QDateTime::currentDateTime();
}

} // namespace

CameraPostProcessor::CameraPostProcessor() :
    m_msgQueueToGUI(nullptr),
    m_availableChannelOrFeatureHandler(kTrackedObjectPipeURIs, QStringList{QStringLiteral("mapitems")}),
    m_captureActive(false),
    m_preRecordBufferFlushed(false),
    m_recordedImageFrames(0),
    m_processingFrame(false)
{}

CameraPostProcessor::~CameraPostProcessor()
{
    stopWork();
    m_inputMessageQueue.clear();
}

void CameraPostProcessor::startWork()
{
    QObject::connect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraPostProcessor::handleInputMessages);
    QObject::connect(&m_availableChannelOrFeatureHandler, &AvailableChannelOrFeatureHandler::messageEnqueued, this, &CameraPostProcessor::handlePipeMessageQueue);
    m_availableChannelOrFeatureHandler.scanAvailableChannelsAndFeatures();
    handleInputMessages();
}

void CameraPostProcessor::stopWork()
{
    QObject::disconnect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraPostProcessor::handleInputMessages);
    QObject::disconnect(&m_availableChannelOrFeatureHandler, &AvailableChannelOrFeatureHandler::messageEnqueued, this, &CameraPostProcessor::handlePipeMessageQueue);

    if (m_weather)
    {
        disconnect(m_weather, &Weather::weatherUpdated, this, &CameraPostProcessor::weatherUpdated);
        delete m_weather;
        m_weather = nullptr;
    }

    closeVideoWriters();
}

void CameraPostProcessor::handleInputMessages()
{
    Message* message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void CameraPostProcessor::handlePipeMessageQueue(MessageQueue* messageQueue)
{
    if (!messageQueue) {
        return;
    }

    Message* message = nullptr;
    while ((message = messageQueue->pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

bool CameraPostProcessor::handleMessage(const Message& cmd)
{
    if (MsgConfigureCameraPostProcessor::match(cmd))
    {
        MsgConfigureCameraPostProcessor& cfg = (MsgConfigureCameraPostProcessor&) cmd;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (MsgProcessFrame::match(cmd))
    {
        MsgProcessFrame& frameMsg = (MsgProcessFrame&) cmd;
        submitFrame(frameMsg.getFrame());
        return true;
    }
    else if (MsgSpectrumFrame::match(cmd))
    {
        MsgSpectrumFrame& frameMsg = (MsgSpectrumFrame&) cmd;
        m_spectrumViewImage = frameMsg.getImage();
        return true;
    }
    else if (MainCore::MsgMapItem::match(cmd))
    {
        const MainCore::MsgMapItem& msgMapItem = (const MainCore::MsgMapItem&) cmd;
        updateTrackedMapObject(msgMapItem.getPipeSource(), msgMapItem.getSWGMapItem());
        return true;
    }
    else if (MsgSetVideoRecordingEnabled::match(cmd))
    {
        MsgSetVideoRecordingEnabled& enabledMsg = (MsgSetVideoRecordingEnabled&) cmd;
        setVideoRecordingEnabled(enabledMsg.getEnabled());
        return true;
    }
    else if (MsgCaptureActive::match(cmd))
    {
        MsgCaptureActive& activeMsg = (MsgCaptureActive&) cmd;
        m_captureActive = activeMsg.isActive();

        if (m_captureActive)
        {
            m_lastFrame = CameraPipelineFrame();
        }
        else
        {
            closeVideoWriters();
            m_preRecordVideoFrames.clear();
            m_preRecordBufferFlushed = false;
            resetRecordingLimits();
        }

        QMutexLocker locker(&m_frameMutex);
        m_pendingFrame.reset();
        if (!m_captureActive) {
            m_processingFrame = false;
        }

        return true;
    }

    return false;
}

void CameraPostProcessor::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraPostProcessor::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    const bool wasSaveImage = m_settings.m_saveImage;
    const bool wasSaveVideo = m_settings.m_saveVideo;
    static const QStringList kPostProcessingKeys = {
        "overlayDateTime", "dateTimeColor",
        "dateTimeFormat", "dateTimePosX", "dateTimePosY",
        "equatorialGrid", "equatorialGridColor",
        "altAzGrid", "altAzGridColor",
        "constellation", "constellationColor", "constellationOverlay",
        "trackObjects", "trackObjectMinElevation", "trackObjectColor", "trackObjectFontScale",
        "gridLabelFontFamily", "gridLabelFontScale",
        "overlayText", "overlayTextString", "overlayTextColor",
        "overlayTextFontFamily", "overlayTextFontScale", "overlayTextPosX", "overlayTextPosY",
        "overlayFontFamily", "overlayFontScale",
        "motionBoxColor", "starColor", "plateSolveLabelMode",
        "overlaySpectrum", "spectrumDevice", "spectrumOffsetX", "spectrumOffsetY", "spectrumScale",
        "latitude", "longitude", "altitude", "azimuth", "elevation", "roll", "fov",
        "lensProjection", "lensCenterOffsetX", "lensCenterOffsetY", "lensDistortionK1", "owmAPIKey",
        "yoloBoxColor"
    };
    const bool postProcessChanged = force || std::any_of(kPostProcessingKeys.cbegin(), kPostProcessingKeys.cend(),
        [&settingsKeys](const QString& k) { return settingsKeys.contains(k); });
    const bool sourceChanged = force
        || settingsKeys.contains("cameraId")
        || settingsKeys.contains("cameraProtocol")
        || settingsKeys.contains("resolutionWidth")
        || settingsKeys.contains("resolutionHeight")
        || settingsKeys.contains("cameraBinX")
        || settingsKeys.contains("cameraBinY")
        || settingsKeys.contains("cameraNumX")
        || settingsKeys.contains("cameraNumY")
        || settingsKeys.contains("cameraStartX")
        || settingsKeys.contains("cameraStartY")
        || settingsKeys.contains("cameraGain")
        || settingsKeys.contains("cameraOffset")
        || settingsKeys.contains("cameraReadoutMode")
        || settingsKeys.contains("exposureTimeMs")
        || settingsKeys.contains("stackEnabled")
        || settingsKeys.contains("stackFrameCount")
        || settingsKeys.contains("stackMethod")
        || settingsKeys.contains("stackAlignmentMethod");

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (sourceChanged) {
        m_lastFrame = CameraPipelineFrame();
        m_preRecordVideoFrames.clear();
        m_preRecordBufferFlushed = false;
        resetRecordingLimits();
    }

    if (force
        || (settingsKeys.contains("saveImage") && m_settings.m_saveImage != wasSaveImage)
        || (settingsKeys.contains("saveVideo") && m_settings.m_saveVideo != wasSaveVideo))
    {
        resetRecordingLimits();
    }

    if (force || settingsKeys.contains("spectrumDevice")) {
        m_spectrumViewImage = QImage();
    }

    if (force || settingsKeys.contains("owmAPIKey") || settingsKeys.contains("latitude") || settingsKeys.contains("longitude"))
    {
        restartWeatherUpdates();
    }

    if (settingsKeys.contains("saveVideo") || settingsKeys.contains("videoFileName")
        || settingsKeys.contains("videoHwAcceleration") || settingsKeys.contains("videoPostProcess"))
    {
        closeVideoWriters();
        m_preRecordBufferFlushed = false;
    }

    if (force || settingsKeys.contains("videoPreRecordBufferSeconds")) {
        trimPreRecordBuffer();
    }

    if (postProcessChanged && !m_lastFrame.m_image.isNull()) {
        const QImage processed = applyPostProcessing(m_lastFrame);
        reportFrameToGUI(processed, m_lastFrame);
    }
}

void CameraPostProcessor::restartWeatherUpdates()
{
    if (m_weather)
    {
        disconnect(m_weather, &Weather::weatherUpdated, this, &CameraPostProcessor::weatherUpdated);
        delete m_weather;
        m_weather = nullptr;
    }

    m_weatherTemperature = NAN;
    m_weatherPressure = NAN;
    m_weatherHumidity = NAN;

    if (!m_settings.m_owmAPIKey.trimmed().isEmpty())
    {
        m_weather = Weather::create(m_settings.m_owmAPIKey.trimmed());
        if (m_weather)
        {
            connect(m_weather, &Weather::weatherUpdated, this, &CameraPostProcessor::weatherUpdated);
            m_weather->getWeatherPeriodically(m_settings.m_latitude, m_settings.m_longitude, 15);
        }
    }
}

void CameraPostProcessor::weatherUpdated(float temperature, float pressure, float humidity, float cloudiness, float windSpeed, float windDirection)
{
    m_weatherTemperature = temperature;
    m_weatherPressure = pressure;
    m_weatherHumidity = humidity;
    m_weatherCloudiness = cloudiness;
    m_weatherWindSpeed = windSpeed;
    m_weatherWindDirection = windDirection;

    if (!m_lastFrame.m_image.isNull())
    {
        const QImage processed = applyPostProcessing(m_lastFrame);
        reportFrameToGUI(processed, m_lastFrame);
    }
}

void CameraPostProcessor::updateTrackedMapObject(const QObject* pipeSource, SWGSDRangel::SWGMapItem* swgMapItem)
{
    if (!swgMapItem || (swgMapItem->getType() != 0)) {
        return;
    }

    const QString name = swgMapItem->getName() ? swgMapItem->getName()->trimmed() : QString();
    if (name.isEmpty()) {
        return;
    }

    const QString key = QStringLiteral("%1:%2").arg(pipeSource ? pipeSource->objectName() : QString(), name);
    const QString image = swgMapItem->getImage() ? *swgMapItem->getImage() : QString();

    if (image.isEmpty())
    {
        m_trackedMapObjects.remove(key);
    }
    else
    {
        TrackedMapObject object;
        object.m_label = (swgMapItem->getLabel() && !swgMapItem->getLabel()->trimmed().isEmpty())
            ? swgMapItem->getLabel()->trimmed()
            : name;
        object.m_label = object.m_label.replace("<br>", "\n");
        object.m_latitude = swgMapItem->getLatitude();
        object.m_longitude = swgMapItem->getLongitude();
        object.m_altitude = swgMapItem->getAltitude();

        if (swgMapItem->getPositionDateTime()) {
            object.m_positionDateTime = QDateTime::fromString(*swgMapItem->getPositionDateTime(), Qt::ISODateWithMs);
        }
        if (swgMapItem->getAvailableUntil()) {
            object.m_availableUntil = QDateTime::fromString(*swgMapItem->getAvailableUntil(), Qt::ISODateWithMs);
        }

        m_trackedMapObjects.insert(key, object);
    }
}

void CameraPostProcessor::submitFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame) {
        return;
    }

    bool schedule = false;
    {
        QMutexLocker locker(&m_frameMutex);
        if (m_pendingFrame) {
            qDebug() << "CameraPostProcessor: Dropping pending frame in favor of new frame";
        }
        m_pendingFrame = frame;
        if (!m_processingFrame)
        {
            m_processingFrame = true;
            schedule = true;
        }
    }

    if (schedule) {
        QMetaObject::invokeMethod(this, &CameraPostProcessor::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraPostProcessor::processNextFrame()
{
    CameraPipelineFramePtr frame;

    {
        QMutexLocker locker(&m_frameMutex);
        frame = m_pendingFrame;
        m_pendingFrame.reset();

        if (!frame)
        {
            m_processingFrame = false;
            return;
        }
    }

    processNewFrame(frame);

    bool schedule = false;
    {
        QMutexLocker locker(&m_frameMutex);
        if (m_pendingFrame) {
            schedule = true;
        } else {
            m_processingFrame = false;
        }
    }

    if (schedule) {
        QMetaObject::invokeMethod(this, &CameraPostProcessor::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraPostProcessor::processNewFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || !frame->hasImageData()) {
        return;
    }

    if (!frame->ensureCpuImageFromCuda()) {
        return;
    }

    m_captureDateTime = frame->m_captureDateTime.isValid() ? frame->m_captureDateTime : QDateTime::currentDateTime();
    const QImage& unprocessedImage = frame->m_unprocessedImage.isNull() ? frame->m_image : frame->m_unprocessedImage;
    const QImage processed = applyPostProcessing(*frame);

    m_lastFrame = *frame;

    reportFrameToGUI(processed, *frame);

    bool savedImageFrame = false;
    bool savedVideoFrame = false;

    if (m_captureActive && (m_settings.m_saveImage || frame->m_saveCurrentImage) && !m_settings.m_imageFileName.isEmpty())
    {
        if (shouldSaveRawMedia())
        {
            const QString rawFilename = createTimestampedOutputFilename(m_settings.m_imageFileName, true);
            qDebug() << "CameraPostProcessor: Saving raw image to" << rawFilename;
            unprocessedImage.save(rawFilename);
        }

        if (shouldSaveProcessedMedia())
        {
            const QString processedFilename = createTimestampedOutputFilename(m_settings.m_imageFileName, false);
            qDebug() << "CameraPostProcessor: Saving processed image to" << processedFilename;
            processed.save(processedFilename);
        }

        savedImageFrame = m_settings.m_saveImage;
    }

    if (m_captureActive && m_settings.m_saveVideo && !m_settings.m_videoFileName.isEmpty())
    {
        if (!m_preRecordBufferFlushed) {
            flushPreRecordFrames(unprocessedImage, processed);
        }

        if (shouldSaveRawMedia() && ensureVideoWriter(m_rawVideoWriter, m_settings.m_videoFileName, unprocessedImage, true)) {
            writeVideoFrame(m_rawVideoWriter, unprocessedImage);
            savedVideoFrame = true;
        }

        if (shouldSaveProcessedMedia() && ensureVideoWriter(m_processedVideoWriter, m_settings.m_videoFileName, processed, false)) {
            writeVideoFrame(m_processedVideoWriter, processed);
            savedVideoFrame = true;
        }
    }
    else if (m_captureActive && !m_settings.m_saveVideo)
    {
        appendPreRecordFrame(unprocessedImage, processed);
    }

    updateRecordingLimitsAfterFrame(savedImageFrame, savedVideoFrame);
}

void CameraPostProcessor::reportFrameToGUI(const QImage& image, const CameraPipelineFrame& frame)
{
    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportFrame::create(
            image,
            frame.m_histogramData,
            frame.m_stackCount,
            frame.m_stackQueuedCount,
            frame.m_stackDroppedCount,
            frame.m_plateSolved,
            frame.m_plateSolvedMatches,
            frame.m_plateSolveDetectedStarsConsidered,
            frame.m_plateSolveCatalogStarsLoaded,
            frame.m_plateSolveCatalogCandidateStars,
            frame.m_plateSolveOutlierStars,
            frame.m_plateSolveRmsError,
            frame.m_plateSolveMaxError,
            frame.m_plateSolveAzimuth,
            frame.m_plateSolveElevation,
            frame.m_plateSolveRoll,
            frame.m_plateSolveFov,
            frame.m_plateSolveCenterOffsetX,
            frame.m_plateSolveCenterOffsetY,
            frame.m_plateSolveDistortionK1,
            frame.m_plateSolveCatalogSource));
    }
}

void CameraPostProcessor::applyMotionOverlay(cv::Mat& bgrMat, const QVector<QRect>& motionBoxes) const
{
    PROFILER_START();
    const QColor& bc = m_settings.m_motionBoxColor;
    const cv::Scalar boxColor(bc.blue(), bc.green(), bc.red());
    for (const QRect& box : motionBoxes) {
        cv::Rect cvBox(box.x(), box.y(), box.width(), box.height());
        cv::rectangle(bgrMat, cvBox, boxColor, 2);
    }
    PROFILER_STOP(__FUNCTION__);
}

void CameraPostProcessor::applyDetectionOverlay(cv::Mat& bgrMat, const QVector<CameraPipelineDetection>& detections) const
{
    PROFILER_START();
    const QColor& bc = m_settings.m_yoloBoxColor;
    const cv::Scalar boxColor(bc.blue(), bc.green(), bc.red());
    const cv::Scalar textBg(0, 0, 0);

    for (const CameraPipelineDetection& detection : detections)
    {
        const cv::Rect box(detection.m_box.x(), detection.m_box.y(), detection.m_box.width(), detection.m_box.height());
        cv::rectangle(bgrMat, box, boxColor, 2);

        QString label = detection.m_label + QStringLiteral(" %1%").arg(static_cast<int>(detection.m_score * 100.0f + 0.5f));
        const std::string labelStd = label.toStdString();
        int baseLine = 0;
        const cv::Size textSize = cv::getTextSize(labelStd, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        const int labelY = std::max(box.y, textSize.height + 2);
        cv::rectangle(bgrMat,
                      cv::Point(box.x, labelY - textSize.height - 2),
                      cv::Point(box.x + textSize.width, labelY + baseLine),
                      textBg, cv::FILLED);
        cv::putText(bgrMat, labelStd, cv::Point(box.x, labelY), cv::FONT_HERSHEY_SIMPLEX, 0.5, boxColor, 1, cv::LINE_AA);
    }
    PROFILER_STOP(__FUNCTION__);
}

void CameraPostProcessor::applyStarOverlay(QImage& image, const QVector<CameraPipelineStarDetection>& starDetections) const
{
    PROFILER_START();

    if (starDetections.isEmpty()) {
        return;
    }

    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::TextAntialiasing);
    QFont font;
    if (!m_settings.m_gridLabelFontFamily.isEmpty()) {
        font.setFamily(m_settings.m_gridLabelFontFamily);
    }
    font.setPointSizeF(std::max(6.0, m_settings.m_gridLabelFontScale));
    painter.setFont(font);
    const QFontMetrics fontMetrics(font);

    for (const CameraPipelineStarDetection& detection : starDetections)
    {
        const QColor starColor = detection.m_solved
            ? m_settings.m_starColor
            : QColor(160, 160, 160);
        QPen pen(starColor);
        pen.setWidth(1);
        painter.setPen(pen);
        const QRectF box(detection.m_center.x() - 3.0, detection.m_center.y() - 3.0, 6.0, 6.0);
        painter.drawRect(box);

        if (detection.m_solved && !detection.m_projectedCenter.isNull())
        {
            QPen residualPen(starColor.lighter(125));
            residualPen.setStyle(Qt::DashLine);
            residualPen.setWidth(1);
            painter.setPen(residualPen);
            painter.drawLine(detection.m_center, detection.m_projectedCenter);
            painter.setPen(pen);
        }

        const QString solvedStarLabel = formatSolvedStarLabel(m_settings, detection);
        if (!solvedStarLabel.isEmpty()) {
            drawOutlinedLabel(painter, image.rect(), detection.m_center, solvedStarLabel, starColor, fontMetrics);
        }
    }

    PROFILER_STOP(__FUNCTION__);
}

void CameraPostProcessor::applySpectrumOverlay(cv::Mat& bgrMat) const
{
    PROFILER_START();
    QImage specSrc = m_spectrumViewImage;
    if (qAbs(m_settings.m_spectrumScale - 1.0) > 1e-4)
    {
        const int sw = static_cast<int>(specSrc.width() * m_settings.m_spectrumScale);
        const int sh = static_cast<int>(specSrc.height() * m_settings.m_spectrumScale);
        if (sw > 0 && sh > 0) {
            specSrc = specSrc.scaled(sw, sh, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        }
    }

    const QImage specRgb = specSrc.convertToFormat(QImage::Format_RGBA8888);
    const int dstW = bgrMat.cols;
    const int dstH = bgrMat.rows;
    const int ox = m_settings.m_spectrumOffsetX;
    const int oy = m_settings.m_spectrumOffsetY;
    const int sw = specRgb.width();
    const int sh = specRgb.height();

    const int srcX0 = std::max(0, -ox);
    const int srcY0 = std::max(0, -oy);
    const int srcX1 = std::min(sw, dstW - ox);
    const int srcY1 = std::min(sh, dstH - oy);

    for (int sy = srcY0; sy < srcY1; ++sy)
    {
        const uchar* srcRow = specRgb.constScanLine(sy);
        const int dy = oy + sy;
        uchar* dstRow = bgrMat.ptr<uchar>(dy);

        for (int sx = srcX0; sx < srcX1; ++sx)
        {
            const int srcPx = sx * 4;
            const uchar alpha = srcRow[srcPx + 3];
            if (alpha == 0) {
                continue;
            }
            const int dx = (ox + sx) * 3;
            if (alpha == 255)
            {
                dstRow[dx] = srcRow[srcPx + 2];
                dstRow[dx + 1] = srcRow[srcPx + 1];
                dstRow[dx + 2] = srcRow[srcPx];
            }
            else
            {
                const int a = alpha;
                const int invA = 255 - a;
                dstRow[dx] = static_cast<uchar>((srcRow[srcPx + 2] * a + dstRow[dx] * invA) / 255);
                dstRow[dx + 1] = static_cast<uchar>((srcRow[srcPx + 1] * a + dstRow[dx + 1] * invA) / 255);
                dstRow[dx + 2] = static_cast<uchar>((srcRow[srcPx] * a + dstRow[dx + 2] * invA) / 255);
            }
        }
    }
    PROFILER_STOP(__FUNCTION__);
}

const QImage& CameraPostProcessor::ensureRgb888(const QImage& image, QImage& convertedImage)
{
    if (image.format() == QImage::Format_RGB888) {
        return image;
    }

    convertedImage = image.convertToFormat(QImage::Format_RGB888);
    return convertedImage;
}

cv::Mat CameraPostProcessor::wrapRgb888Image(const QImage& image)
{
    return cv::Mat(image.height(), image.width(), CV_8UC3,
                   const_cast<uchar*>(image.constBits()),
                   static_cast<size_t>(image.bytesPerLine()));
}

QImage CameraPostProcessor::convertBgrToRgbImage(const cv::Mat& bgrMat)
{
    PROFILER_START();
    QImage result(bgrMat.cols, bgrMat.rows, QImage::Format_RGB888);
    cv::Mat rgbMat(result.height(), result.width(), CV_8UC3,
                   result.bits(),
                   static_cast<size_t>(result.bytesPerLine()));
    cv::cvtColor(bgrMat, rgbMat, cv::COLOR_BGR2RGB);
    return result;
}

void CameraPostProcessor::applyDateTimeOverlay(QImage& image) const
{
    PROFILER_START();
    const QString fmt = m_settings.m_dateTimeFormat.isEmpty()
                        ? QStringLiteral("yyyy-MM-dd hh:mm:ss")
                        : m_settings.m_dateTimeFormat;
    const QString text = m_captureDateTime.toString(fmt);
    QFont font;
    if (!m_settings.m_overlayFontFamily.isEmpty()) {
        font.setFamily(m_settings.m_overlayFontFamily);
    }
    font.setPointSizeF(m_settings.m_overlayFontScale);
    const QFontMetrics fm(font);
    QPainter painter(&image);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.setFont(font);
    painter.setPen(m_settings.m_dateTimeColor);
    const int x = m_settings.m_dateTimePosX;
    const int y = m_settings.m_dateTimePosY + fm.ascent();
    painter.drawText(x, y, text);
    PROFILER_STOP(__FUNCTION__);
}

QString CameraPostProcessor::expandOverlayTextTemplate() const
{
    QString overlayText = m_settings.m_overlayTextString;
    const auto replaceToken = [&overlayText](const QString& token, const QString& value)
    {
        overlayText.replace(token, value.toHtmlEscaped());
    };
    const auto weatherValueString = [](float value, int decimals) -> QString
    {
        return std::isnan(value) ? QStringLiteral("N/A") : QString::number(value, 'f', decimals);
    };

    replaceToken(QStringLiteral("${date}"), m_captureDateTime.date().toString(Qt::ISODate));
    replaceToken(QStringLiteral("${time}"), m_captureDateTime.time().toString(QStringLiteral("HH:mm:ss")));
    replaceToken(QStringLiteral("${exposure}"), QString::number(m_settings.m_exposureTimeMs, 'f', 3));
    replaceToken(QStringLiteral("${cameraId}"), m_settings.m_cameraId);
    replaceToken(QStringLiteral("${latitude}"), QString::number(m_settings.m_latitude, 'f', 6));
    replaceToken(QStringLiteral("${longitude}"), QString::number(m_settings.m_longitude, 'f', 6));
    replaceToken(QStringLiteral("${altitude}"), QString::number(m_settings.m_altitude, 'f', 2));
    replaceToken(QStringLiteral("${azimuth}"), QString::number(m_settings.m_azimuth, 'f', 2));
    replaceToken(QStringLiteral("${elevation}"), QString::number(m_settings.m_elevation, 'f', 2));
    replaceToken(QStringLiteral("${roll}"), QString::number(m_settings.m_roll, 'f', 2));
    replaceToken(QStringLiteral("${temp}"), weatherValueString(m_weatherTemperature, 1));
    replaceToken(QStringLiteral("${pressure}"), weatherValueString(m_weatherPressure, 1));
    replaceToken(QStringLiteral("${humidity}"), weatherValueString(m_weatherHumidity, 0));
    replaceToken(QStringLiteral("${humidty}"), weatherValueString(m_weatherHumidity, 0));
    replaceToken(QStringLiteral("${cloudiness}"), weatherValueString(m_weatherCloudiness, 0));
    replaceToken(QStringLiteral("${windSpeed}"), weatherValueString(m_weatherWindSpeed, 0));
    replaceToken(QStringLiteral("${windDirection}"), weatherValueString(m_weatherWindDirection, 0));

    return overlayText;
}

void CameraPostProcessor::applySkyGridOverlay(QImage& image) const
{
    PROFILER_START();

    const bool drawEquatorial = m_settings.m_equatorialGrid;
    const bool drawAltAz = m_settings.m_altAzGrid;
    if (!drawEquatorial && !drawAltAz) {
        return;
    }

    const SkyProjector projector = SkyProjector::create(m_settings, image.size());
    if (!projector.valid) {
        return;
    }

    const QDateTime utcDateTime = m_captureDateTime.toUTC();
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.setClipRect(image.rect());
    QFont font;
    if (!m_settings.m_gridLabelFontFamily.isEmpty()) {
        font.setFamily(m_settings.m_gridLabelFontFamily);
    }
    font.setPointSizeF(m_settings.m_gridLabelFontScale);
    painter.setFont(font);
    const QFontMetrics fontMetrics(font);

    if (drawAltAz)
    {
        painter.setPen(QPen(m_settings.m_altAzGridColor, 1.0));

        for (int altitude = -80; altitude <= 80; altitude += 10)
        {
            bool havePrevious = false;
            QPointF previousPoint;
            for (double azimuth = 0.0; azimuth <= 360.0 + 1e-6; azimuth += 2.0)
            {
                QPointF point;
                const bool ok = projector.projectAltAz(azimuth, static_cast<double>(altitude), point);
                if (ok && havePrevious && QLineF(previousPoint, point).length() <= std::hypot(static_cast<double>(image.width()), static_cast<double>(image.height())) * 2.0) {
                    painter.drawLine(previousPoint, point);
                }
                previousPoint = point;
                havePrevious = ok;
            }

            QPointF labelPoint;
            if (projector.projectAltAz(m_settings.m_azimuth, static_cast<double>(altitude), labelPoint)) {
                drawOutlinedLabel(painter, image.rect(), labelPoint, formatSignedDegrees(static_cast<double>(altitude)), m_settings.m_altAzGridColor, fontMetrics);
            }
        }

        for (int azimuth = 0; azimuth < 360; azimuth += 15)
        {
            bool havePrevious = false;
            QPointF previousPoint;
            for (double altitude = -10.0; altitude <= 90.0 + 1e-6; altitude += 2.0)
            {
                QPointF point;
                const bool ok = projector.projectAltAz(static_cast<double>(azimuth), altitude, point);
                if (ok && havePrevious && QLineF(previousPoint, point).length() <= std::hypot(static_cast<double>(image.width()), static_cast<double>(image.height())) * 2.0) {
                    painter.drawLine(previousPoint, point);
                }
                previousPoint = point;
                havePrevious = ok;
            }

            QPointF labelPoint;
            bool foundLabelPoint = false;
            for (double altitude = -10.0; altitude <= 90.0 + 1e-6; altitude += 2.0)
            {
                if (projector.projectAltAz(static_cast<double>(azimuth), altitude, labelPoint))
                {
                    foundLabelPoint = true;
                    break;
                }
            }

            if (foundLabelPoint) {
                drawOutlinedLabel(painter, image.rect(), labelPoint, formatAzimuthDegrees(static_cast<double>(azimuth)), m_settings.m_altAzGridColor, fontMetrics);
            }
        }
    }

    if (drawEquatorial)
    {
        painter.setPen(QPen(m_settings.m_equatorialGridColor, 1.0));

        for (int declination = -80; declination <= 80; declination += 10)
        {
            bool havePrevious = false;
            QPointF previousPoint;
            for (double rightAscension = 0.0; rightAscension <= 360.0 + 1e-6; rightAscension += 2.0)
            {
                double azimuth = 0.0;
                double elevation = 0.0;
                QPointF point;
                const bool ok = equatorialToAltAz(
                    rightAscension,
                    static_cast<double>(declination),
                    m_settings.m_latitude,
                    m_settings.m_longitude,
                    utcDateTime,
                    azimuth,
                    elevation)
                    && projector.projectAltAz(azimuth, elevation, point);

                if (ok && havePrevious && QLineF(previousPoint, point).length() <= std::hypot(static_cast<double>(image.width()), static_cast<double>(image.height())) * 2.0) {
                    painter.drawLine(previousPoint, point);
                }
                previousPoint = point;
                havePrevious = ok;
            }

            double labelAzimuth = 0.0;
            double labelElevation = 0.0;
            QPointF labelPoint;
            if (equatorialToAltAz(
                    greenwichMeanSiderealDegrees(utcDateTime) + m_settings.m_longitude,
                    static_cast<double>(declination),
                    m_settings.m_latitude,
                    m_settings.m_longitude,
                    utcDateTime,
                    labelAzimuth,
                    labelElevation)
                && projector.projectAltAz(labelAzimuth, labelElevation, labelPoint))
            {
                drawOutlinedLabel(painter, image.rect(), labelPoint, formatSignedDegrees(static_cast<double>(declination)), m_settings.m_equatorialGridColor, fontMetrics);
            }
        }

        for (int rightAscension = 0; rightAscension < 360; rightAscension += 15)
        {
            bool havePrevious = false;
            QPointF previousPoint;
            for (double declination = -80.0; declination <= 80.0 + 1e-6; declination += 2.0)
            {
                double azimuth = 0.0;
                double elevation = 0.0;
                QPointF point;
                const bool ok = equatorialToAltAz(
                    static_cast<double>(rightAscension),
                    declination,
                    m_settings.m_latitude,
                    m_settings.m_longitude,
                    utcDateTime,
                    azimuth,
                    elevation)
                    && projector.projectAltAz(azimuth, elevation, point);

                if (ok && havePrevious && QLineF(previousPoint, point).length() <= std::hypot(static_cast<double>(image.width()), static_cast<double>(image.height())) * 2.0) {
                    painter.drawLine(previousPoint, point);
                }
                previousPoint = point;
                havePrevious = ok;
            }

            double labelAzimuth = 0.0;
            double labelElevation = 0.0;
            QPointF labelPoint;
            if (equatorialToAltAz(
                    static_cast<double>(rightAscension),
                    std::clamp(static_cast<double>(m_settings.m_elevation), -60.0, 60.0),
                    m_settings.m_latitude,
                    m_settings.m_longitude,
                    utcDateTime,
                    labelAzimuth,
                    labelElevation)
                && projector.projectAltAz(labelAzimuth, labelElevation, labelPoint))
            {
                drawOutlinedLabel(painter, image.rect(), labelPoint, formatRightAscensionDegrees(static_cast<double>(rightAscension)), m_settings.m_equatorialGridColor, fontMetrics);
            }
        }
    }

    PROFILER_STOP(__FUNCTION__);
}

void CameraPostProcessor::applyConstellationOverlay(QImage& image) const
{
    PROFILER_START();

    if (!m_settings.m_constellation) {
        return;
    }

    const SkyProjector projector = SkyProjector::create(m_settings, image.size());
    if (!projector.valid) {
        return;
    }

    const QDateTime utcDateTime = plateSolveOverlayDateTime(m_settings, m_captureDateTime).toUTC();
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setClipRect(image.rect());
    painter.setPen(QPen(m_settings.m_constellationColor, 1.0));

    switch (m_settings.m_constellationOverlay)
    {
    case CameraSettings::ConstellationOverlayUrsaMajor:
        drawConstellationStars(painter, image, projector, utcDateTime, m_settings, kUrsaMajorStars);
        break;
    case CameraSettings::ConstellationOverlayOrion:
        drawConstellationStars(painter, image, projector, utcDateTime, m_settings, kOrionStars);
        break;
    case CameraSettings::ConstellationOverlayCrux:
        drawConstellationStars(painter, image, projector, utcDateTime, m_settings, kCruxStars);
        break;
    }

    PROFILER_STOP(__FUNCTION__);
}

void CameraPostProcessor::applyTrackedObjectOverlay(QImage& image) const
{
    PROFILER_START();

    if (!m_settings.m_trackObjects || m_trackedMapObjects.isEmpty()) {
        return;
    }

    const SkyProjector projector = SkyProjector::create(m_settings, image.size());
    if (!projector.valid) {
        return;
    }

    const QDateTime currentDateTime = m_captureDateTime.isValid() ? m_captureDateTime : QDateTime::currentDateTime();

    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.setClipRect(image.rect());
    QFont font;
    if (!m_settings.m_gridLabelFontFamily.isEmpty()) {
        font.setFamily(m_settings.m_gridLabelFontFamily);
    }
    font.setPointSizeF(m_settings.m_trackObjectFontScale);
    painter.setFont(font);
    const QFontMetrics fontMetrics(font);
    painter.setPen(QPen(m_settings.m_trackObjectColor, 2.0));

    for (auto it = m_trackedMapObjects.cbegin(); it != m_trackedMapObjects.cend(); ++it)
    {
        const TrackedMapObject& object = it.value();
        if (object.m_availableUntil.isValid() && (object.m_availableUntil < currentDateTime)) {
            continue;
        }

        AzEl azEl;
        azEl.setLocation(m_settings.m_latitude, m_settings.m_longitude, m_settings.m_altitude);
        azEl.setTarget(object.m_latitude, object.m_longitude, object.m_altitude);
        azEl.calculate();
        if (!std::isfinite(azEl.getAzimuth()) || !std::isfinite(azEl.getElevation()) || (azEl.getElevation() < m_settings.m_trackObjectMinElevation)) {
            continue;
        }

        QPointF point;
        if (!projector.projectAltAz(azEl.getAzimuth(), azEl.getElevation(), point)) {
            continue;
        }

        const QPoint labelPoint(static_cast<int>(std::lround(point.x())), static_cast<int>(std::lround(point.y())));
        if (!image.rect().adjusted(0, 0, -1, -1).contains(labelPoint)) {
            continue;
        }

        drawOutlinedLabel(painter, image.rect(), point, object.m_label, m_settings.m_trackObjectColor, fontMetrics);
    }

    PROFILER_STOP(__FUNCTION__);
}

void CameraPostProcessor::applyTextOverlay(QImage& image, const QString& overlayTextHtml) const
{
    PROFILER_START();
    QTextDocument overlayTextDocument;
    QFont font;
    if (!m_settings.m_overlayTextFontFamily.isEmpty()) {
        font.setFamily(m_settings.m_overlayTextFontFamily);
    }
    font.setPointSizeF(m_settings.m_overlayTextFontScale);
    overlayTextDocument.setDefaultFont(font);
    overlayTextDocument.setDefaultStyleSheet(QStringLiteral("* { color: %1; }").arg(m_settings.m_overlayTextColor.name()));
    overlayTextDocument.setHtml(QStringLiteral("<div>%1</div>").arg(overlayTextHtml));

    const int x = std::max(0, m_settings.m_overlayTextPosX);
    const qreal maxTextWidth = std::max(1, image.width() - x);
    overlayTextDocument.setTextWidth(maxTextWidth);

    QPainter painter(&image);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.save();
    painter.translate(x, m_settings.m_overlayTextPosY);
    overlayTextDocument.drawContents(&painter);
    painter.restore();
    PROFILER_STOP(__FUNCTION__);
}

QImage CameraPostProcessor::applyPostProcessing(const CameraPipelineFrame& frame)
{
    PROFILER_START();

    const QImage& input = frame.m_image;
    const bool needsSpectrumOverlay = m_settings.m_overlaySpectrum && !m_spectrumViewImage.isNull();
    const QString expandedOverlayText = expandOverlayTextTemplate();
    QTextDocument overlayTextDocument;
    overlayTextDocument.setHtml(expandedOverlayText);
    const bool needsTextOverlay = m_settings.m_overlayText && !overlayTextDocument.toPlainText().trimmed().isEmpty();
    const bool needsAny = m_settings.m_overlayDateTime
        || m_settings.m_equatorialGrid
        || m_settings.m_altAzGrid
        || m_settings.m_constellation
        || (m_settings.m_trackObjects && !m_trackedMapObjects.isEmpty())
        || needsTextOverlay
        || !frame.m_motionBoxes.isEmpty()
        || !frame.m_detections.isEmpty()
        || !frame.m_starDetections.isEmpty()
        || needsSpectrumOverlay;

    if (!needsAny) {
        return input;
    }

    QImage convertedRgb;
    const QImage& rgb = ensureRgb888(input, convertedRgb);
    cv::Mat mat = wrapRgb888Image(rgb);
    cv::Mat bgrMat;
    cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);

    if (!frame.m_motionBoxes.isEmpty()) { applyMotionOverlay(bgrMat, frame.m_motionBoxes); }
    if (!frame.m_detections.isEmpty()) { applyDetectionOverlay(bgrMat, frame.m_detections); }
    if (needsSpectrumOverlay) { applySpectrumOverlay(bgrMat); }

    QImage result = convertBgrToRgbImage(bgrMat);

    if (!frame.m_starDetections.isEmpty()) { applyStarOverlay(result, frame.m_starDetections); }
    if (m_settings.m_equatorialGrid || m_settings.m_altAzGrid) { applySkyGridOverlay(result); }
    if (m_settings.m_constellation) { applyConstellationOverlay(result); }
    if (m_settings.m_trackObjects && !m_trackedMapObjects.isEmpty()) { applyTrackedObjectOverlay(result); }
    if (m_settings.m_overlayDateTime) { applyDateTimeOverlay(result); }
    if (needsTextOverlay) { applyTextOverlay(result, expandedOverlayText); }

    PROFILER_STOP("CameraPostProcessor::applyPostProcessing");
    return result;
}

void CameraPostProcessor::setVideoRecordingEnabled(bool enabled)
{
    if (m_settings.m_saveVideo == enabled) {
        return;
    }

    m_settings.m_saveVideo = enabled;
    m_preRecordBufferFlushed = false;

    if (enabled) {
        m_videoRecordingStartDateTime = QDateTime::currentDateTimeUtc();
    } else {
        closeVideoWriters();
        m_videoRecordingStartDateTime = QDateTime();
    }

    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportSaveVideoState::create(enabled));
    }
}

void CameraPostProcessor::setImageRecordingEnabled(bool enabled)
{
    if (m_settings.m_saveImage == enabled) {
        return;
    }

    m_settings.m_saveImage = enabled;

    if (enabled) {
        m_recordedImageFrames = 0;
    }

    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportSaveImageState::create(enabled));
    }
}

void CameraPostProcessor::resetRecordingLimits()
{
    m_recordedImageFrames = 0;
    m_videoRecordingStartDateTime = m_settings.m_saveVideo ? QDateTime::currentDateTimeUtc() : QDateTime();
}

void CameraPostProcessor::updateRecordingLimitsAfterFrame(bool savedImageFrame, bool savedVideoFrame)
{
    if (savedImageFrame && (m_settings.m_imageRecordLimit > 0))
    {
        ++m_recordedImageFrames;

        if (m_recordedImageFrames >= m_settings.m_imageRecordLimit) {
            setImageRecordingEnabled(false);
        }
    }

    if (savedVideoFrame && (m_settings.m_videoRecordLimitSeconds > 0))
    {
        if (!m_videoRecordingStartDateTime.isValid()) {
            m_videoRecordingStartDateTime = QDateTime::currentDateTimeUtc();
        }

        if (m_videoRecordingStartDateTime.msecsTo(QDateTime::currentDateTimeUtc()) >= (m_settings.m_videoRecordLimitSeconds * 1000LL)) {
            setVideoRecordingEnabled(false);
        }
    }
}

QString CameraPostProcessor::createTimestampedOutputFilename(const QString& baseFileName, bool rawVariant)
{
    const QFileInfo fileInfo(baseFileName);
    const QString timestamp = QDateTime::currentDateTimeUtc().toString("yyyy-MM-ddTHH_mm_ss_zzz");
    const QString infix = rawVariant ? ".raw." : ".";
    return fileInfo.path() + "/" + fileInfo.baseName() + infix + timestamp + "." + fileInfo.suffix();
}

bool CameraPostProcessor::shouldSaveRawMedia() const
{
    return (m_settings.m_recordMode == CameraSettings::SavedMediaRaw)
        || (m_settings.m_recordMode == CameraSettings::SavedMediaBoth);
}

bool CameraPostProcessor::shouldSaveProcessedMedia() const
{
    return (m_settings.m_recordMode == CameraSettings::SavedMediaProcessed)
        || (m_settings.m_recordMode == CameraSettings::SavedMediaBoth);
}

void CameraPostProcessor::closeVideoWriters()
{
    if (m_rawVideoWriter.isOpened()) {
        m_rawVideoWriter.release();
    }
    if (m_processedVideoWriter.isOpened()) {
        m_processedVideoWriter.release();
    }
}

int CameraPostProcessor::preRecordBufferFrameLimit() const
{
    const int seconds = qBound(0, m_settings.m_videoPreRecordBufferSeconds, 60);
    if (seconds <= 0) {
        return 0;
    }

    return std::max(1, static_cast<int>(std::ceil(m_settings.getCaptureFrameRate() * seconds)));
}

void CameraPostProcessor::trimPreRecordBuffer()
{
    const int frameLimit = preRecordBufferFrameLimit();
    while (static_cast<int>(m_preRecordVideoFrames.size()) > frameLimit) {
        m_preRecordVideoFrames.pop_front();
    }

    if (frameLimit == 0) {
        m_preRecordVideoFrames.clear();
    }
}

void CameraPostProcessor::appendPreRecordFrame(const QImage& rawImage, const QImage& processedImage)
{
    if (preRecordBufferFrameLimit() <= 0) {
        m_preRecordVideoFrames.clear();
        return;
    }

    m_preRecordVideoFrames.push_back({rawImage.copy(), processedImage.copy()});
    trimPreRecordBuffer();
}

void CameraPostProcessor::flushPreRecordFrames(const QImage& currentRawImage, const QImage& currentProcessedImage)
{
    if (!m_settings.m_saveVideo || m_settings.m_videoFileName.isEmpty())
    {
        m_preRecordBufferFlushed = true;
        return;
    }

    if (shouldSaveRawMedia() && ensureVideoWriter(m_rawVideoWriter, m_settings.m_videoFileName, currentRawImage, true))
    {
        for (const BufferedVideoFrame& bufferedFrame : m_preRecordVideoFrames)
        {
            if (bufferedFrame.m_rawImage.size() == currentRawImage.size()) {
                writeVideoFrame(m_rawVideoWriter, bufferedFrame.m_rawImage);
            }
        }
    }

    if (shouldSaveProcessedMedia() && ensureVideoWriter(m_processedVideoWriter, m_settings.m_videoFileName, currentProcessedImage, false))
    {
        for (const BufferedVideoFrame& bufferedFrame : m_preRecordVideoFrames)
        {
            if (bufferedFrame.m_processedImage.size() == currentProcessedImage.size()) {
                writeVideoFrame(m_processedVideoWriter, bufferedFrame.m_processedImage);
            }
        }
    }

    m_preRecordVideoFrames.clear();
    m_preRecordBufferFlushed = true;
}

bool CameraPostProcessor::ensureVideoWriter(cv::VideoWriter& writer, const QString& baseFileName, const QImage& frameForSize, bool rawVariant)
{
    if (writer.isOpened()) {
        return true;
    }

    const QString filename = createTimestampedOutputFilename(baseFileName, rawVariant);
    const int fourcc = cv::VideoWriter::fourcc('a', 'v', 'c', '1');
    const std::vector<int> params = {
        cv::VIDEOWRITER_PROP_HW_ACCELERATION,
        m_settings.m_videoHwAcceleration ? cv::VIDEO_ACCELERATION_ANY : cv::VIDEO_ACCELERATION_NONE
    };

    writer.open(
        filename.toStdString(),
        fourcc,
        m_settings.getCaptureFrameRate(),
        cv::Size(frameForSize.width(), frameForSize.height()),
        params);

    if (writer.isOpened()) {
        qDebug() << "CameraPostProcessor opened:" << filename << "backend:" << QString::fromStdString(writer.getBackendName());
    } else {
        qWarning() << "CameraPostProcessor failed to open:" << filename;
    }

    return writer.isOpened();
}

void CameraPostProcessor::writeVideoFrame(cv::VideoWriter& writer, const QImage& frameToWrite)
{
    QImage convertedRgb;
    const QImage& rgb = ensureRgb888(frameToWrite, convertedRgb);
    cv::Mat mat = wrapRgb888Image(rgb);
    cv::Mat bgrMat;
    cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);
    writer.write(bgrMat);
}
