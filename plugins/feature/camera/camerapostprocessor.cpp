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
#include <limits>

#include <QDebug>
#include <QFont>
#include <QFontMetrics>
#include <QLineF>
#include <QPainter>
#include <QPolygonF>
#include <QRadialGradient>
#include <QTextDocument>
#include <QDateTime>
#include "SWGMapItem.h"
#include "util/azel.h"
#include "util/weather.h"
#include "util/profiler.h"
#include "maincore.h"
#include "cameraimageutils.h"
#include "camera.h"
#include "cameraworker.h"
#include "camerapostprocessor.h"
#include "camerarecorder.h"

MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgSpectrumFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgReportFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgClearTrackedObjectHeatMap, Message)
MESSAGE_CLASS_DEFINITION(CameraPostProcessor::MsgSaveCurrentImage, Message)

namespace {

const QStringList kTrackedObjectPipeURIs = {
    QStringLiteral("sdrangel.channel.adsbdemod"),
    QStringLiteral("sdrangel.feature.satellitetracker"),
    QStringLiteral("sdrangel.feature.startracker")
};

static constexpr int kTrackedObjectMaxTrackPoints = 256;
static constexpr double kTrackedObjectTrackMinDeltaDegrees = 1e-6;
static constexpr double kTrackedObjectTrackMinDeltaAltitudeMetres = 0.5;
static constexpr double kTrackedObjectHeatMapRadiusPixels = 18.0;
static constexpr double kTrackedObjectHeatMapLineWidthPixels = 10.0;

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
    // "Hide synthetic names": skip stars whose only name is the synthesized Gaia coordinate label
    // (catalogDisplayName routes generic Gaia stars through formatGaiaCoordinateLabel, which always
    // emits "Gaia J<coords>"); real catalogue names (HIP/HD/proper names) are kept.
    if (settings.m_plateSolveLabelHideSyntheticNames
        && detection.m_label.trimmed().startsWith(QStringLiteral("Gaia "), Qt::CaseInsensitive)) {
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

static void appendOutlinedPreviewTextLabel(QVector<CameraPostProcessor::PreviewTextLabel> *labels,
                                           const QString& text,
                                           const QPointF& point,
                                           const QColor& color,
                                           const QString& fontFamily,
                                           double fontPointSize)
{
    if (!labels || text.isEmpty()) {
        return;
    }

    CameraPostProcessor::PreviewTextLabel label;
    label.m_text = text;
    label.m_position = point;
    label.m_color = color;
    label.m_fontFamily = fontFamily;
    label.m_fontPointSize = fontPointSize;
    label.m_positionIsTopLeft = false;
    label.m_background = false;
    labels->append(label);
}

static void appendTopLeftPreviewTextLabel(QVector<CameraPostProcessor::PreviewTextLabel> *labels,
                                          const QString& text,
                                          const QPointF& topLeft,
                                          const QColor& color,
                                          const QString& fontFamily,
                                          double fontPointSize,
                                          bool background)
{
    if (!labels || text.isEmpty()) {
        return;
    }

    CameraPostProcessor::PreviewTextLabel label;
    label.m_text = text;
    label.m_position = topLeft;
    label.m_color = color;
    label.m_fontFamily = fontFamily;
    label.m_fontPointSize = fontPointSize;
    label.m_positionIsTopLeft = true;
    label.m_background = background;
    labels->append(label);
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
    if (settings.m_plateSolveUseCaptureDateTime)
    {
        if (captureDateTime.isValid()) {
            return captureDateTime;
        }

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
    m_nextStageQueue(nullptr),
    m_availableChannelOrFeatureHandler(kTrackedObjectPipeURIs, QStringList{QStringLiteral("mapitems")}),
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

}

void CameraPostProcessor::handleInputMessages()
{
    Message* message;

    Camera::discardQueuedProcessFramesOnCaptureActive(m_inputMessageQueue);

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
    if (Camera::MsgConfigureCamera::match(cmd))
    {
        const Camera::MsgConfigureCamera& cfg = (const Camera::MsgConfigureCamera&) cmd;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (Camera::MsgProcessFrame::match(cmd))
    {
        Camera::MsgProcessFrame& frameMsg = (Camera::MsgProcessFrame&) cmd;
        submitFrame(frameMsg.getFrame());
        return true;
    }
    else if (MsgSpectrumFrame::match(cmd))
    {
        MsgSpectrumFrame& frameMsg = (MsgSpectrumFrame&) cmd;
        m_spectrumViewImage = frameMsg.getImage();
        return true;
    }
    else if (MsgClearTrackedObjectHeatMap::match(cmd))
    {
        m_trackedObjectHeatMap = QImage();
        m_trackedObjectHeatMapSize = QSize();
        m_trackedObjectHeatMapLastPoints.clear();
        m_trackedObjectHeatMapSkipSeed = true;

        if (!m_lastFrame.m_image.isNull())
        {
            m_lastFrame.m_manualPreviewFrame = true;
            QVector<PreviewTextLabel> previewTextLabels;
            QVector<PreviewRectItem> previewRectItems;
            QVector<CameraPipelineTrackedObject> trackedObjects;
            const QImage preview = applyPostProcessing(m_lastFrame, false, &previewTextLabels, &previewRectItems, &trackedObjects);
            reportFrameToGUI(preview, m_lastFrame, previewTextLabels, previewRectItems, trackedObjects);
        }

        return true;
    }
    else if (MsgSaveCurrentImage::match(cmd))
    {
        saveCurrentImage();
        return true;
    }
    else if (MainCore::MsgMapItem::match(cmd))
    {
        const MainCore::MsgMapItem& msgMapItem = (const MainCore::MsgMapItem&) cmd;
        updateTrackedMapObject(msgMapItem.getPipeSource(), msgMapItem.getSWGMapItem());
        return true;
    }
    else if (Camera::MsgCaptureActive::match(cmd))
    {
        Camera::MsgCaptureActive& activeMsg = (Camera::MsgCaptureActive&) cmd;
        Camera::discardQueuedProcessFrames(m_inputMessageQueue);
        m_captureActive = activeMsg.isActive();
        m_captureEpoch = activeMsg.getCaptureEpoch();

        if (activeMsg.isActive())
        {
            m_lastFrame = CameraPipelineFrame();
        }
        QMutexLocker locker(&m_frameMutex);
        m_pendingFrames.clear();
        if (!m_captureActive) {
            m_processingFrame = false;
        }

        return true;
    }

    return false;
}

void CameraPostProcessor::saveCurrentImage()
{
    if (!m_nextStageQueue)
    {
        qWarning() << "CameraPostProcessor::saveCurrentImage: no recorder stage is available";
        return;
    }

    if (!m_lastFrame.hasImageData())
    {
        qWarning() << "CameraPostProcessor::saveCurrentImage: no current image is available";
        return;
    }

    CameraPipelineFramePtr frame(new CameraPipelineFrame(m_lastFrame));
    frame->m_manualPreviewFrame = true;
    frame->m_saveCurrentImage = true;
    m_nextStageQueue->push(Camera::MsgProcessFrame::create(frame));
}

void CameraPostProcessor::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraPostProcessor::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    static const QStringList kPostProcessingKeys = {
        "overlayDateTime", "dateTimeColor",
        "dateTimeFormat", "dateTimePosX", "dateTimePosY",
        "equatorialGrid", "equatorialGridColor",
        "altAzGrid", "altAzGridColor",
        "constellation", "constellationColor", "constellationOverlay",
        "trackObjects", "trackObjectTrails", "trackObjectHeatMap", "trackObjectMinElevation", "trackObjectColor", "trackObjectFontScale",
        "gridLabelFontFamily", "gridLabelFontScale",
        "overlayText", "overlayTextString", "overlayTextColor",
        "overlayTextFontFamily", "overlayTextFontScale", "overlayTextPosX", "overlayTextPosY",
        "overlayFontFamily", "overlayFontScale",
        "motionBoxColor", "starColor", "plateSolveLabelMode", "yoloEnabled",
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
    }

    if (sourceChanged
        || settingsKeys.contains("trackObjectMinElevation")
        || settingsKeys.contains("latitude")
        || settingsKeys.contains("longitude")
        || settingsKeys.contains("altitude")
        || settingsKeys.contains("azimuth")
        || settingsKeys.contains("elevation")
        || settingsKeys.contains("roll")
        || settingsKeys.contains("fov")
        || settingsKeys.contains("lensProjection")
        || settingsKeys.contains("lensCenterOffsetX")
        || settingsKeys.contains("lensCenterOffsetY")
        || settingsKeys.contains("lensDistortionK1"))
    {
        m_trackedObjectHeatMap = QImage();
        m_trackedObjectHeatMapSize = QSize();
        m_trackedObjectHeatMapLastPoints.clear();
        m_trackedObjectHeatMapSkipSeed = false;
    }

    if (force || settingsKeys.contains("spectrumDevice")) {
        m_spectrumViewImage = QImage();
    }

    if (force || settingsKeys.contains("owmAPIKey") || settingsKeys.contains("latitude") || settingsKeys.contains("longitude"))
    {
        restartWeatherUpdates();
    }

    if (postProcessChanged && !m_lastFrame.m_image.isNull()) {
        m_lastFrame.m_manualPreviewFrame = true;
        QVector<PreviewTextLabel> previewTextLabels;
        QVector<PreviewRectItem> previewRectItems;
        QVector<CameraPipelineTrackedObject> trackedObjects;
        const QImage preview = applyPostProcessing(m_lastFrame, false, &previewTextLabels, &previewRectItems, &trackedObjects);
        reportFrameToGUI(preview, m_lastFrame, previewTextLabels, previewRectItems, trackedObjects);
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
        QVector<PreviewTextLabel> previewTextLabels;
        QVector<PreviewRectItem> previewRectItems;
        QVector<CameraPipelineTrackedObject> trackedObjects;
        const QImage preview = applyPostProcessing(m_lastFrame, false, &previewTextLabels, &previewRectItems, &trackedObjects);
        reportFrameToGUI(preview, m_lastFrame, previewTextLabels, previewRectItems, trackedObjects);
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
        TrackedMapObject object = m_trackedMapObjects.value(key);
        object.m_name = name;
        object.m_label = (swgMapItem->getLabel() && !swgMapItem->getLabel()->trimmed().isEmpty())
            ? swgMapItem->getLabel()->trimmed()
            : name;
        object.m_label = object.m_label.replace("<br>", "\n");
        object.m_latitude = swgMapItem->getLatitude();
        object.m_longitude = swgMapItem->getLongitude();
        object.m_altitude = swgMapItem->getAltitude();

        if (swgMapItem->getAvailableUntil()) {
            object.m_availableUntil = QDateTime::fromString(*swgMapItem->getAvailableUntil(), Qt::ISODateWithMs);
        }
        else {
            object.m_availableUntil = QDateTime();
        }

        const bool appendTrackPoint = object.m_track.isEmpty()
            || (std::fabs(object.m_track.constLast().m_latitude - object.m_latitude) > kTrackedObjectTrackMinDeltaDegrees)
            || (std::fabs(object.m_track.constLast().m_longitude - object.m_longitude) > kTrackedObjectTrackMinDeltaDegrees)
            || (std::fabs(object.m_track.constLast().m_altitude - object.m_altitude) > kTrackedObjectTrackMinDeltaAltitudeMetres);
        if (appendTrackPoint)
        {
            object.m_track.append({
                object.m_latitude,
                object.m_longitude,
                object.m_altitude
            });
            while (object.m_track.size() > kTrackedObjectMaxTrackPoints) {
                object.m_track.removeFirst();
            }
        }

        m_trackedMapObjects.insert(key, object);
    }
}

void CameraPostProcessor::submitFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame) {
        return;
    }
    if (!Camera::acceptsPipelineFrame(frame, m_captureActive, m_captureEpoch)) {
        return;
    }

    bool schedule = false;
    {
        QMutexLocker locker(&m_frameMutex);
        m_pendingFrames.push_back(frame);
        // Keep only a small cushion. Under sustained overrun (queue full) drop the
        // oldest frame so latency stays bounded while we keep the most recent ones.
        while (static_cast<int>(m_pendingFrames.size()) > m_maxPendingFrames)
        {
            qDebug() << "CameraPostProcessor: Dropping pending frame, queue full";
            m_pendingFrames.pop_front();
        }
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
        if (m_pendingFrames.empty())
        {
            m_processingFrame = false;
            return;
        }
        frame = m_pendingFrames.front();
        m_pendingFrames.pop_front();
    }

    if (Camera::acceptsPipelineFrame(frame, m_captureActive, m_captureEpoch)) {
        processNewFrame(frame);
    }

    bool schedule = false;
    {
        QMutexLocker locker(&m_frameMutex);
        if (!m_pendingFrames.empty()) {
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
    if (!Camera::acceptsPipelineFrame(frame, m_captureActive, m_captureEpoch)) {
        return;
    }

    if (!frame->ensureCpuImageFromCuda()) {
        return;
    }

    m_captureDateTime = frame->m_captureDateTime.isValid() ? frame->m_captureDateTime : QDateTime::currentDateTime();
    QVector<PreviewTextLabel> previewTextLabels;
    QVector<PreviewRectItem> previewRectItems;
    QVector<CameraPipelineTrackedObject> trackedObjects;
    const QImage preview = applyPostProcessing(*frame, false, &previewTextLabels, &previewRectItems, &trackedObjects);
    if (!previewTextLabels.isEmpty() || !previewRectItems.isEmpty())
    {
        // Pooled deep copy of the preview so the rect/text items are drawn onto a
        // separate buffer from the one sent to the GUI (recycled, not malloc'd
        // per frame). Source composition reproduces copy()'s exact pixels.
        QImage processed = m_overlayImagePool.acquire(preview.width(), preview.height(), preview.format());
        if (!processed.isNull())
        {
            QPainter painter(&processed);
            painter.setCompositionMode(QPainter::CompositionMode_Source);
            painter.drawImage(0, 0, preview);
            painter.end();
        }
        else
        {
            processed = preview.copy();
        }
        applyPreviewRectItems(processed, previewRectItems);
        applyPreviewTextLabels(processed, previewTextLabels);
        frame->m_postProcessedImage = processed;
    }
    else
    {
        frame->m_postProcessedImage = preview;
    }

    m_lastFrame = *frame;

    if (!Camera::acceptsPipelineFrame(frame, m_captureActive, m_captureEpoch)) {
        return;
    }

    reportFrameToGUI(preview, *frame, previewTextLabels, previewRectItems, trackedObjects);

    if (m_nextStageQueue) {
        m_nextStageQueue->push(Camera::MsgProcessFrame::create(frame));
    }
}

void CameraPostProcessor::reportFrameToGUI(const QImage& image, const CameraPipelineFrame& frame, const QVector<PreviewTextLabel>& previewTextLabels, const QVector<PreviewRectItem>& previewRectItems, const QVector<CameraPipelineTrackedObject>& trackedObjects)
{
    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportFrame::create(
            image,
            frame.m_histogramData,
            frame.m_stack,
            frame.m_starDetections,
            frame.m_plateSolve,
            frame.m_motionBoxes,
            frame.m_detections,
            trackedObjects,
            frame.m_captureDateTime,
            frame.m_captureEpoch,
            frame.m_manualPreviewFrame,
            previewTextLabels,
            previewRectItems));
    }
}

void CameraPostProcessor::applyMotionOverlay(QImage& image, const QVector<QRect>& motionBoxes, bool drawBoxes, QVector<PreviewRectItem> *previewRectItems) const
{
    PROFILER_START();

    if (motionBoxes.isEmpty()) {
        return;
    }

    if (drawBoxes)
    {
        QPainter painter(&image);
        painter.setRenderHint(QPainter::Antialiasing);
        QPen pen(m_settings.m_motionBoxColor);
        pen.setWidth(2);
        painter.setPen(pen);
        painter.setBrush(Qt::NoBrush);

        for (const QRect& box : motionBoxes) {
            painter.drawRect(box);
        }
    }
    else if (previewRectItems)
    {
        previewRectItems->reserve(previewRectItems->size() + motionBoxes.size());
        for (const QRect& box : motionBoxes)
        {
            PreviewRectItem item;
            item.m_rect = QRectF(box);
            item.m_color = m_settings.m_motionBoxColor;
            item.m_lineWidth = 2.0;
            previewRectItems->append(item);
        }
    }

    PROFILER_STOP(__FUNCTION__);
}

void CameraPostProcessor::applyDetectionOverlay(QImage& image, const QVector<CameraPipelineDetection>& detections, bool drawLabels, QVector<PreviewTextLabel> *previewTextLabels, QVector<PreviewRectItem> *previewRectItems) const
{
    PROFILER_START();

    if (detections.isEmpty()) {
        return;
    }

    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::TextAntialiasing);
    QFont font = painter.font();
    font.setPointSizeF(std::max(6.0, font.pointSizeF() > 0.0 ? font.pointSizeF() : 9.0));
    painter.setFont(font);
    const QFontMetrics fontMetrics(font);
    QPen pen(m_settings.m_yoloBoxColor);
    pen.setWidth(2);

    for (const CameraPipelineDetection& detection : detections)
    {
        const QRect box = detection.m_box;
        if (drawLabels)
        {
            painter.setPen(pen);
            painter.setBrush(Qt::NoBrush);
            painter.drawRect(box);
        }
        else if (previewRectItems)
        {
            PreviewRectItem item;
            item.m_rect = QRectF(box);
            item.m_color = m_settings.m_yoloBoxColor;
            item.m_lineWidth = 2.0;
            previewRectItems->append(item);
        }

        const QString label = detection.m_label + QStringLiteral(" %1%").arg(static_cast<int>(detection.m_score * 100.0f + 0.5f));
        const QSize textSize = fontMetrics.size(Qt::TextSingleLine, label);
        QRect labelRect(
            box.left(),
            std::max(0, box.top() - textSize.height() - 4),
            textSize.width() + 6,
            textSize.height() + 4);
        if (labelRect.right() >= image.width()) {
            labelRect.moveRight(image.width() - 1);
        }
        if (labelRect.top() < 0) {
            labelRect.moveTop(std::min(image.height() - labelRect.height(), box.top()));
        }

        if (drawLabels)
        {
            const QPointF labelPoint(
                labelRect.left() - 4.0,
                labelRect.top() + fontMetrics.lineSpacing() + 4.0);
            drawOutlinedLabel(painter, image.rect(), labelPoint, label, m_settings.m_yoloBoxColor, fontMetrics);
        }
        else
        {
            appendTopLeftPreviewTextLabel(
                previewTextLabels,
                label,
                labelRect.topLeft(),
                m_settings.m_yoloBoxColor,
                font.family(),
                font.pointSizeF(),
                false);
        }
    }

    PROFILER_STOP(__FUNCTION__);
}

void CameraPostProcessor::applyStarOverlay(QImage& image, const QVector<CameraPipelineStarDetection>& starDetections, bool drawLabels, QVector<PreviewTextLabel> *previewTextLabels) const
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
        if (m_settings.m_showStarDetectionBoxes)
        {
            const QRectF box(detection.m_center.x() - 3.0, detection.m_center.y() - 3.0, 6.0, 6.0);
            painter.drawRect(box);
        }

        if (detection.m_solved && !detection.m_projectedCenter.isNull())
        {
            QPen residualPen(starColor.lighter(125));
            residualPen.setStyle(Qt::DashLine);
            residualPen.setWidth(1);
            painter.setPen(residualPen);
            painter.drawLine(detection.m_center, detection.m_projectedCenter);
            painter.setPen(pen);
        }

        if (drawLabels)
        {
            const QString solvedStarLabel = formatSolvedStarLabel(m_settings, detection);
            if (!solvedStarLabel.isEmpty()) {
                drawOutlinedLabel(painter, image.rect(), detection.m_center, solvedStarLabel, starColor, fontMetrics);
            }
        }
        else
        {
            appendOutlinedPreviewTextLabel(
                previewTextLabels,
                formatSolvedStarLabel(m_settings, detection),
                detection.m_center,
                starColor,
                font.family(),
                font.pointSizeF());
        }
    }

    PROFILER_STOP(__FUNCTION__);
}

void CameraPostProcessor::applyPreviewRectItems(QImage& image, const QVector<PreviewRectItem>& items) const
{
    PROFILER_START();

    if (items.isEmpty())
    {
        PROFILER_STOP(__FUNCTION__);
        return;
    }

    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setBrush(Qt::NoBrush);

    for (const PreviewRectItem& item : items)
    {
        QPen pen(item.m_color);
        pen.setWidthF(std::max(1.0, item.m_lineWidth));
        painter.setPen(pen);
        painter.drawRect(item.m_rect);
    }

    PROFILER_STOP(__FUNCTION__);
}

void CameraPostProcessor::applyPreviewTextLabels(QImage& image, const QVector<PreviewTextLabel>& labels) const
{
    PROFILER_START();

    if (labels.isEmpty())
    {
        PROFILER_STOP(__FUNCTION__);
        return;
    }

    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::TextAntialiasing);

    for (const PreviewTextLabel& label : labels)
    {
        QFont font;
        if (!label.m_fontFamily.isEmpty()) {
            font.setFamily(label.m_fontFamily);
        }
        font.setPointSizeF(std::max(6.0, label.m_fontPointSize));
        painter.setFont(font);
        const QFontMetrics fontMetrics(font);

        if (label.m_positionIsTopLeft)
        {
            const QStringList lines = label.m_text.split(QChar('\n'));
            int textWidth = 0;
            for (const QString& line : lines) {
                textWidth = std::max(textWidth, fontMetrics.horizontalAdvance(line));
            }
            QRect labelRect(
                qRound(label.m_position.x()),
                qRound(label.m_position.y()),
                textWidth + 6,
                lines.size() * fontMetrics.lineSpacing() + 4);

            if (label.m_background)
            {
                painter.setPen(Qt::NoPen);
                painter.setBrush(Qt::black);
                painter.drawRect(labelRect);
            }

            auto drawLines = [&](const QPoint& offset, const QColor& penColor)
            {
                painter.setPen(penColor);
                int baselineY = labelRect.top() + fontMetrics.ascent() + 2;
                for (const QString& line : lines)
                {
                    painter.drawText(labelRect.left() + 3 + offset.x(), baselineY + offset.y(), line);
                    baselineY += fontMetrics.lineSpacing();
                }
            };

            painter.setBrush(Qt::NoBrush);
            if (!label.m_background) {
                drawLines(QPoint(1, 1), Qt::black);
            }
            drawLines(QPoint(0, 0), label.m_color);
        }
        else
        {
            drawOutlinedLabel(painter, image.rect(), label.m_position, label.m_text, label.m_color, fontMetrics);
        }
    }

    PROFILER_STOP(__FUNCTION__);
}

void CameraPostProcessor::applySpectrumOverlay(QImage& image) const
{
    PROFILER_START();

    if (m_spectrumViewImage.isNull()) {
        return;
    }

    QImage specSrc = m_spectrumViewImage;
    if (qAbs(m_settings.m_spectrumScale - 1.0) > 1e-4)
    {
        const int sw = static_cast<int>(specSrc.width() * m_settings.m_spectrumScale);
        const int sh = static_cast<int>(specSrc.height() * m_settings.m_spectrumScale);
        if (sw > 0 && sh > 0) {
            specSrc = specSrc.scaled(sw, sh, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        }
    }

    QPainter painter(&image);
    painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
    painter.drawImage(m_settings.m_spectrumOffsetX, m_settings.m_spectrumOffsetY, specSrc);
    PROFILER_STOP(__FUNCTION__);
}

const QImage& CameraPostProcessor::ensureRgb888(const QImage& image, QImage& convertedImage)
{
    return CameraImageUtils::ensureRgb888(image, convertedImage);
}

cv::Mat CameraPostProcessor::wrapRgb888Image(const QImage& image)
{
    return CameraImageUtils::wrapRgb888Image(image);
}

void CameraPostProcessor::applyDateTimeOverlay(QImage& image, bool drawLabel, QVector<PreviewTextLabel> *previewTextLabels) const
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
    const int x = m_settings.m_dateTimePosX;
    const int y = m_settings.m_dateTimePosY;
    if (drawLabel)
    {
        QPainter painter(&image);
        painter.setRenderHint(QPainter::TextAntialiasing);
        painter.setFont(font);
        painter.setPen(m_settings.m_dateTimeColor);
        painter.drawText(x, y + fm.ascent(), text);
    }
    else
    {
        appendTopLeftPreviewTextLabel(
            previewTextLabels,
            text,
            QPointF(x, y),
            m_settings.m_dateTimeColor,
            font.family(),
            font.pointSizeF(),
            false);
    }
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

// Render only the grid LINES into a transparent overlay. This is the expensive
// part (thousands of antialiased trig-projected segments) and is cached by
// applySkyGridOverlay; the result is re-composited each frame until an input
// changes. Kept separate from label rendering, which is cheap and per-frame.
static void renderSkyGridLines(
    QImage& overlay,
    const SkyProjector& projector,
    const CameraSettings& settings,
    const QDateTime& utcDateTime,
    bool drawEquatorial,
    bool drawAltAz)
{
    QPainter painter(&overlay);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setClipRect(overlay.rect());
    const double maxSegment = std::hypot(static_cast<double>(overlay.width()), static_cast<double>(overlay.height())) * 2.0;

    if (drawAltAz)
    {
        painter.setPen(QPen(settings.m_altAzGridColor, 1.0));

        for (int altitude = -80; altitude <= 80; altitude += 10)
        {
            bool havePrevious = false;
            QPointF previousPoint;
            for (double azimuth = 0.0; azimuth <= 360.0 + 1e-6; azimuth += 2.0)
            {
                QPointF point;
                const bool ok = projector.projectAltAz(azimuth, static_cast<double>(altitude), point);
                if (ok && havePrevious && QLineF(previousPoint, point).length() <= maxSegment) {
                    painter.drawLine(previousPoint, point);
                }
                previousPoint = point;
                havePrevious = ok;
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
                if (ok && havePrevious && QLineF(previousPoint, point).length() <= maxSegment) {
                    painter.drawLine(previousPoint, point);
                }
                previousPoint = point;
                havePrevious = ok;
            }
        }
    }

    if (drawEquatorial)
    {
        painter.setPen(QPen(settings.m_equatorialGridColor, 1.0));

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
                    settings.m_latitude,
                    settings.m_longitude,
                    utcDateTime,
                    azimuth,
                    elevation)
                    && projector.projectAltAz(azimuth, elevation, point);

                if (ok && havePrevious && QLineF(previousPoint, point).length() <= maxSegment) {
                    painter.drawLine(previousPoint, point);
                }
                previousPoint = point;
                havePrevious = ok;
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
                    settings.m_latitude,
                    settings.m_longitude,
                    utcDateTime,
                    azimuth,
                    elevation)
                    && projector.projectAltAz(azimuth, elevation, point);

                if (ok && havePrevious && QLineF(previousPoint, point).length() <= maxSegment) {
                    painter.drawLine(previousPoint, point);
                }
                previousPoint = point;
                havePrevious = ok;
            }
        }
    }
}

// Render grid LABELS. Cheap (a handful of projected points) and dependent on
// drawLabels (bake into the image vs. collect for the preview text overlay), so
// it is always done per-frame rather than cached with the lines.
static void renderSkyGridLabels(
    QPainter& painter,
    const QRect& imageRect,
    const SkyProjector& projector,
    const CameraSettings& settings,
    const QDateTime& utcDateTime,
    bool drawEquatorial,
    bool drawAltAz,
    bool drawLabels,
    QVector<CameraPostProcessor::PreviewTextLabel> *previewTextLabels,
    const QFont& font,
    const QFontMetrics& fontMetrics)
{
    if (drawAltAz)
    {
        for (int altitude = -80; altitude <= 80; altitude += 10)
        {
            QPointF labelPoint;
            if (projector.projectAltAz(settings.m_azimuth, static_cast<double>(altitude), labelPoint)) {
                const QString label = formatSignedDegrees(static_cast<double>(altitude));
                if (drawLabels) {
                    drawOutlinedLabel(painter, imageRect, labelPoint, label, settings.m_altAzGridColor, fontMetrics);
                } else {
                    appendOutlinedPreviewTextLabel(previewTextLabels, label, labelPoint, settings.m_altAzGridColor, font.family(), font.pointSizeF());
                }
            }
        }

        for (int azimuth = 0; azimuth < 360; azimuth += 15)
        {
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
                const QString label = formatAzimuthDegrees(static_cast<double>(azimuth));
                if (drawLabels) {
                    drawOutlinedLabel(painter, imageRect, labelPoint, label, settings.m_altAzGridColor, fontMetrics);
                } else {
                    appendOutlinedPreviewTextLabel(previewTextLabels, label, labelPoint, settings.m_altAzGridColor, font.family(), font.pointSizeF());
                }
            }
        }
    }

    if (drawEquatorial)
    {
        for (int declination = -80; declination <= 80; declination += 10)
        {
            double labelAzimuth = 0.0;
            double labelElevation = 0.0;
            QPointF labelPoint;
            if (equatorialToAltAz(
                    greenwichMeanSiderealDegrees(utcDateTime) + settings.m_longitude,
                    static_cast<double>(declination),
                    settings.m_latitude,
                    settings.m_longitude,
                    utcDateTime,
                    labelAzimuth,
                    labelElevation)
                && projector.projectAltAz(labelAzimuth, labelElevation, labelPoint))
            {
                const QString label = formatSignedDegrees(static_cast<double>(declination));
                if (drawLabels) {
                    drawOutlinedLabel(painter, imageRect, labelPoint, label, settings.m_equatorialGridColor, fontMetrics);
                } else {
                    appendOutlinedPreviewTextLabel(previewTextLabels, label, labelPoint, settings.m_equatorialGridColor, font.family(), font.pointSizeF());
                }
            }
        }

        for (int rightAscension = 0; rightAscension < 360; rightAscension += 15)
        {
            double labelAzimuth = 0.0;
            double labelElevation = 0.0;
            QPointF labelPoint;
            if (equatorialToAltAz(
                    static_cast<double>(rightAscension),
                    std::clamp(static_cast<double>(settings.m_elevation), -60.0, 60.0),
                    settings.m_latitude,
                    settings.m_longitude,
                    utcDateTime,
                    labelAzimuth,
                    labelElevation)
                && projector.projectAltAz(labelAzimuth, labelElevation, labelPoint))
            {
                const QString label = formatRightAscensionDegrees(static_cast<double>(rightAscension));
                if (drawLabels) {
                    drawOutlinedLabel(painter, imageRect, labelPoint, label, settings.m_equatorialGridColor, fontMetrics);
                } else {
                    appendOutlinedPreviewTextLabel(previewTextLabels, label, labelPoint, settings.m_equatorialGridColor, font.family(), font.pointSizeF());
                }
            }
        }
    }
}

void CameraPostProcessor::applySkyGridOverlay(QImage& image, bool drawLabels, QVector<PreviewTextLabel> *previewTextLabels) const
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

    // Build the cache signature for the line overlay. The lines depend only on
    // the image size, grid colours and projection parameters; the equatorial
    // grid additionally moves with sidereal time, so the observer location and a
    // 1 s time bucket are folded in (and left zeroed when it is not drawn).
    SkyGridOverlayCache::Key key;
    key.m_size = image.size();
    key.m_drawEquatorial = drawEquatorial;
    key.m_drawAltAz = drawAltAz;
    key.m_altAzColor = m_settings.m_altAzGridColor.rgba();
    key.m_equatorialColor = m_settings.m_equatorialGridColor.rgba();
    key.m_lensProjection = static_cast<int>(m_settings.m_lensProjection);
    key.m_azimuth = m_settings.m_azimuth;
    key.m_elevation = m_settings.m_elevation;
    key.m_roll = m_settings.m_roll;
    key.m_fov = m_settings.m_fov;
    key.m_lensCenterOffsetX = m_settings.m_lensCenterOffsetX;
    key.m_lensCenterOffsetY = m_settings.m_lensCenterOffsetY;
    key.m_lensDistortionK1 = m_settings.m_lensDistortionK1;
    key.m_latitude = drawEquatorial ? static_cast<double>(m_settings.m_latitude) : 0.0;
    key.m_longitude = drawEquatorial ? static_cast<double>(m_settings.m_longitude) : 0.0;
    key.m_equatorialTimeBucket = drawEquatorial
        ? utcDateTime.toMSecsSinceEpoch() / SkyGridOverlayCache::m_equatorialQuantumMs
        : 0;

    if (!m_skyGridOverlayCache.m_valid || m_skyGridOverlayCache.m_key != key)
    {
        QImage overlay(image.size(), QImage::Format_ARGB32_Premultiplied);
        overlay.fill(Qt::transparent);
        renderSkyGridLines(overlay, projector, m_settings, utcDateTime, drawEquatorial, drawAltAz);
        m_skyGridOverlayCache.m_overlay = overlay;
        m_skyGridOverlayCache.m_key = key;
        m_skyGridOverlayCache.m_valid = true;
    }

    QPainter painter(&image);
    painter.setClipRect(image.rect());
    painter.drawImage(0, 0, m_skyGridOverlayCache.m_overlay);

    // Labels are cheap and depend on drawLabels, so draw them per-frame on top
    // of the cached lines (either baked into the image or collected for preview).
    painter.setRenderHint(QPainter::TextAntialiasing);
    QFont font;
    if (!m_settings.m_gridLabelFontFamily.isEmpty()) {
        font.setFamily(m_settings.m_gridLabelFontFamily);
    }
    font.setPointSizeF(m_settings.m_gridLabelFontScale);
    painter.setFont(font);
    const QFontMetrics fontMetrics(font);
    renderSkyGridLabels(painter, image.rect(), projector, m_settings, utcDateTime, drawEquatorial, drawAltAz, drawLabels, previewTextLabels, font, fontMetrics);

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

void CameraPostProcessor::applyTrackedObjectOverlay(QImage& image, bool drawLabels, QVector<PreviewTextLabel> *previewTextLabels, QVector<CameraPipelineTrackedObject> *trackedObjects)
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

    auto projectTrackPoint = [this, &projector](const TrackedMapObject::TrackPoint& trackPoint, QPointF& imagePoint) -> bool {
        AzEl trackAzEl;
        trackAzEl.setLocation(m_settings.m_latitude, m_settings.m_longitude, m_settings.m_altitude);
        trackAzEl.setTarget(trackPoint.m_latitude, trackPoint.m_longitude, trackPoint.m_altitude);
        trackAzEl.calculate();
        return std::isfinite(trackAzEl.getAzimuth())
            && std::isfinite(trackAzEl.getElevation())
            && (trackAzEl.getElevation() >= m_settings.m_trackObjectMinElevation)
            && projector.projectAltAz(trackAzEl.getAzimuth(), trackAzEl.getElevation(), imagePoint);
    };

    if (m_settings.m_trackObjectHeatMap)
    {
        if ((m_trackedObjectHeatMap.size() != image.size())
            || (m_trackedObjectHeatMap.format() != QImage::Format_ARGB32_Premultiplied)
            || (m_trackedObjectHeatMapSize != image.size()))
        {
            m_trackedObjectHeatMap = QImage(image.size(), QImage::Format_ARGB32_Premultiplied);
            m_trackedObjectHeatMap.fill(Qt::transparent);
            m_trackedObjectHeatMapSize = image.size();
            m_trackedObjectHeatMapLastPoints.clear();
        }

        QPainter heatPainter(&m_trackedObjectHeatMap);
        heatPainter.setRenderHint(QPainter::Antialiasing);
        heatPainter.setCompositionMode(QPainter::CompositionMode_Plus);
        const QRectF paddedImageRect = QRectF(image.rect()).adjusted(
            -kTrackedObjectHeatMapRadiusPixels,
            -kTrackedObjectHeatMapRadiusPixels,
            kTrackedObjectHeatMapRadiusPixels,
            kTrackedObjectHeatMapRadiusPixels);

        for (auto it = m_trackedMapObjects.cbegin(); it != m_trackedMapObjects.cend(); ++it)
        {
            const TrackedMapObject& object = it.value();
            if (object.m_availableUntil.isValid() && (object.m_availableUntil < currentDateTime)) {
                continue;
            }

            const QString objectKey = it.key();
            QPointF previousHeatPoint = m_trackedObjectHeatMapLastPoints.value(objectKey, QPointF(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()));
            bool hasPreviousHeatPoint = std::isfinite(previousHeatPoint.x()) && std::isfinite(previousHeatPoint.y());
            QVector<QPointF> validHeatPoints;

            for (const TrackedMapObject::TrackPoint& trackPoint : object.m_track)
            {
                QPointF heatPoint;
                if (projectTrackPoint(trackPoint, heatPoint) && paddedImageRect.contains(heatPoint)) {
                    validHeatPoints.append(heatPoint);
                }
            }

            if (validHeatPoints.isEmpty())
            {
                m_trackedObjectHeatMapLastPoints.remove(objectKey);
                continue;
            }

            if (!hasPreviousHeatPoint && m_trackedObjectHeatMapSkipSeed)
            {
                m_trackedObjectHeatMapLastPoints.insert(objectKey, validHeatPoints.last());
                continue;
            }

            int firstNewPointIndex = 0;
            if (hasPreviousHeatPoint)
            {
                firstNewPointIndex = validHeatPoints.size() - 1;

                for (int i = validHeatPoints.size() - 1; i >= 0; --i)
                {
                    if (QLineF(previousHeatPoint, validHeatPoints[i]).length() < 0.5)
                    {
                        firstNewPointIndex = i + 1;
                        break;
                    }
                }
            }

            QPolygonF projectedTrack;
            if (hasPreviousHeatPoint) {
                projectedTrack.append(previousHeatPoint);
            }
            for (int i = firstNewPointIndex; i < validHeatPoints.size(); ++i)
            {
                if (!projectedTrack.isEmpty() && (QLineF(projectedTrack.last(), validHeatPoints[i]).length() < 0.5)) {
                    continue;
                }
                projectedTrack.append(validHeatPoints[i]);
            }

            if (projectedTrack.size() > 1)
            {
                heatPainter.setBrush(Qt::NoBrush);
                QPen heatLinePen(QColor(255, 120, 0, 32), kTrackedObjectHeatMapLineWidthPixels);
                heatLinePen.setCapStyle(Qt::RoundCap);
                heatLinePen.setJoinStyle(Qt::RoundJoin);
                heatPainter.setPen(heatLinePen);
                heatPainter.drawPolyline(projectedTrack);

                QPen heatCorePen(QColor(255, 220, 0, 20), kTrackedObjectHeatMapLineWidthPixels * 0.45);
                heatCorePen.setCapStyle(Qt::RoundCap);
                heatCorePen.setJoinStyle(Qt::RoundJoin);
                heatPainter.setPen(heatCorePen);
                heatPainter.drawPolyline(projectedTrack);
            }

            heatPainter.setPen(Qt::NoPen);
            for (int i = firstNewPointIndex; i < validHeatPoints.size(); ++i)
            {
                const QPointF& heatPoint = validHeatPoints[i];
                QRadialGradient gradient(heatPoint, kTrackedObjectHeatMapRadiusPixels);
                gradient.setColorAt(0.0, QColor(255, 64, 0, 85));
                gradient.setColorAt(0.45, QColor(255, 210, 0, 42));
                gradient.setColorAt(1.0, QColor(255, 210, 0, 0));
                heatPainter.setBrush(gradient);
                heatPainter.drawEllipse(
                    heatPoint,
                    kTrackedObjectHeatMapRadiusPixels,
                    kTrackedObjectHeatMapRadiusPixels);
            }

            const QPointF newestHeatPoint = validHeatPoints.last();
            if (!hasPreviousHeatPoint || (QLineF(previousHeatPoint, newestHeatPoint).length() >= 0.5)) {
                m_trackedObjectHeatMapLastPoints.insert(objectKey, newestHeatPoint);
            }
        }
        m_trackedObjectHeatMapSkipSeed = false;
        heatPainter.end();

        painter.drawImage(0, 0, m_trackedObjectHeatMap);
    }

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

        if (m_settings.m_trackObjectTrails && (object.m_track.size() > 1))
        {
            QColor trackColor = m_settings.m_trackObjectColor;
            trackColor.setAlpha(180);
            painter.setPen(QPen(trackColor, 1.5));

            QPolygonF projectedTrack;
            auto flushProjectedTrack = [&painter, &projectedTrack]() {
                if (projectedTrack.size() > 1) {
                    painter.drawPolyline(projectedTrack);
                }
                projectedTrack.clear();
            };

            for (const TrackedMapObject::TrackPoint& trackPoint : object.m_track)
            {
                QPointF trackImagePoint;
                if (!projectTrackPoint(trackPoint, trackImagePoint)) {
                    flushProjectedTrack();
                    continue;
                }
                projectedTrack.append(trackImagePoint);
            }
            flushProjectedTrack();
            painter.setPen(QPen(m_settings.m_trackObjectColor, 2.0));
        }

        if (trackedObjects)
        {
            CameraPipelineTrackedObject trackedObject;
            trackedObject.m_name = object.m_name;
            trackedObject.m_label = object.m_label;
            trackedObject.m_position = point;
            trackedObject.m_azimuth = azEl.getAzimuth();
            trackedObject.m_elevation = azEl.getElevation();
            trackedObjects->append(trackedObject);
        }

        if (drawLabels) {
            drawOutlinedLabel(painter, image.rect(), point, object.m_label, m_settings.m_trackObjectColor, fontMetrics);
        } else {
            appendOutlinedPreviewTextLabel(
                previewTextLabels,
                object.m_label,
                point,
                m_settings.m_trackObjectColor,
                font.family(),
                font.pointSizeF());
        }
    }

    PROFILER_STOP(__FUNCTION__);
}

// overlayTextDocument must already have font, style-sheet, and HTML set (done in
// applyPostProcessing so the document is constructed only once per frame).
void CameraPostProcessor::applyTextOverlay(QImage& image, QTextDocument& overlayTextDocument) const
{
    PROFILER_START();
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

QImage CameraPostProcessor::applyPostProcessing(const CameraPipelineFrame& frame, bool drawPreviewText, QVector<PreviewTextLabel> *previewTextLabels, QVector<PreviewRectItem> *previewRectItems, QVector<CameraPipelineTrackedObject> *trackedObjects)
{
    PROFILER_START();

    const QImage& input = frame.m_image;
    const bool needsSpectrumOverlay = m_settings.m_overlaySpectrum && !m_spectrumViewImage.isNull();
    const QString expandedOverlayText = expandOverlayTextTemplate();
    // Build the overlay text document once with font/style/HTML used for both the
    // empty-check and rendering, so we only call QTextDocument::setHtml() once per frame.
    QTextDocument overlayTextDocument;
    bool needsTextOverlay = false;
    if (m_settings.m_overlayText && !expandedOverlayText.trimmed().isEmpty()) {
        QFont font;
        if (!m_settings.m_overlayTextFontFamily.isEmpty()) {
            font.setFamily(m_settings.m_overlayTextFontFamily);
        }
        font.setPointSizeF(m_settings.m_overlayTextFontScale);
        overlayTextDocument.setDefaultFont(font);
        overlayTextDocument.setDefaultStyleSheet(QStringLiteral("* { color: %1; }").arg(m_settings.m_overlayTextColor.name()));
        overlayTextDocument.setHtml(QStringLiteral("<div>%1</div>").arg(expandedOverlayText));
        needsTextOverlay = !overlayTextDocument.toPlainText().trimmed().isEmpty();
    }
    const bool needsAny = m_settings.m_overlayDateTime
        || m_settings.m_equatorialGrid
        || m_settings.m_altAzGrid
        || m_settings.m_constellation
        || (m_settings.m_trackObjects && !m_trackedMapObjects.isEmpty())
        || needsTextOverlay
        || !frame.m_motionBoxes.isEmpty()
        || (m_settings.m_yoloEnabled && !frame.m_detections.isEmpty())
        || !frame.m_starDetections.isEmpty()
        || needsSpectrumOverlay;

    if (!needsAny) {
        return input;
    }

    // Convert into a pooled RGB32 buffer (recycled across frames) instead of
    // letting convertToFormat allocate a fresh ~33 MB @4K buffer every frame.
    // CompositionMode_Source makes the blit a straight format-converting copy
    // (the pooled buffer's prior contents are fully overwritten), identical to
    // convertToFormat's result; the overlays are then painted onto it as before.
    QImage result = m_overlayImagePool.acquire(input.width(), input.height(), QImage::Format_RGB32);
    if (!result.isNull())
    {
        QPainter painter(&result);
        painter.setCompositionMode(QPainter::CompositionMode_Source);
        painter.drawImage(0, 0, input);
        painter.end();
    }
    else
    {
        result = input.convertToFormat(QImage::Format_RGB32);
    }
    if (!frame.m_motionBoxes.isEmpty()) { applyMotionOverlay(result, frame.m_motionBoxes, drawPreviewText, previewRectItems); }
    if (m_settings.m_yoloEnabled && !frame.m_detections.isEmpty()) { applyDetectionOverlay(result, frame.m_detections, drawPreviewText, previewTextLabels, previewRectItems); }
    if (needsSpectrumOverlay) { applySpectrumOverlay(result); }
    if (!frame.m_starDetections.isEmpty()) { applyStarOverlay(result, frame.m_starDetections, drawPreviewText, previewTextLabels); }
    if (m_settings.m_equatorialGrid || m_settings.m_altAzGrid) { applySkyGridOverlay(result, drawPreviewText, previewTextLabels); }
    if (m_settings.m_constellation) { applyConstellationOverlay(result); }
    if (m_settings.m_trackObjects && !m_trackedMapObjects.isEmpty()) { applyTrackedObjectOverlay(result, drawPreviewText, previewTextLabels, trackedObjects); }
    if (m_settings.m_overlayDateTime) { applyDateTimeOverlay(result, drawPreviewText, previewTextLabels); }
    if (needsTextOverlay) { applyTextOverlay(result, overlayTextDocument); }

    PROFILER_STOP("CameraPostProcessor::applyPostProcessing");
    return result;
}
