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
#include <cmath>

#include <QDebug>
#include <QFile>
#include <QMutableHashIterator>
#include <QTextStream>

#include "maincore.h"
#include "pipes/objectpipe.h"
#include "util/profiler.h"
#include "camera.h"
#include "cameraobjectdetector.h"
#include "SWGTargetAzimuthElevation.h"

MESSAGE_CLASS_DEFINITION(CameraObjectDetector::MsgReportObjectDetectionHistory, Message)
MESSAGE_CLASS_DEFINITION(CameraObjectDetector::MsgClearObjectDetectionHistory, Message)
MESSAGE_CLASS_DEFINITION(CameraObjectDetector::MsgReportTensorRtConversion, Message)

namespace {
class ProtoReader
{
public:
    ProtoReader(const uchar *data, int size) :
        m_data(data),
        m_size(size),
        m_pos(0)
    {}

    bool atEnd() const { return m_pos >= m_size; }

    bool readVarint(quint64& value)
    {
        value = 0;
        int shift = 0;

        while (m_pos < m_size && shift < 64)
        {
            const quint8 byte = m_data[m_pos++];
            value |= static_cast<quint64>(byte & 0x7f) << shift;

            if ((byte & 0x80) == 0) {
                return true;
            }

            shift += 7;
        }

        return false;
    }

    bool readSubMessage(ProtoReader& subMessage)
    {
        quint64 length = 0;

        if (!readVarint(length) || (length > static_cast<quint64>(m_size - m_pos))) {
            return false;
        }

        subMessage = ProtoReader(m_data + m_pos, static_cast<int>(length));
        m_pos += static_cast<int>(length);
        return true;
    }

    bool skipField(int wireType)
    {
        switch (wireType)
        {
        case 0:
        {
            quint64 value = 0;
            return readVarint(value);
        }
        case 1:
            return skipBytes(8);
        case 2:
        {
            quint64 length = 0;
            return readVarint(length) && skipBytes(static_cast<int>(length));
        }
        case 5:
            return skipBytes(4);
        default:
            return false;
        }
    }

private:
    bool skipBytes(int count)
    {
        if ((count < 0) || (count > (m_size - m_pos))) {
            return false;
        }

        m_pos += count;
        return true;
    }

    const uchar *m_data;
    int m_size;
    int m_pos;
};

bool parseOnnxTensorShape(ProtoReader& reader, QVector<int>& dims)
{
    while (!reader.atEnd())
    {
        quint64 tag = 0;

        if (!reader.readVarint(tag)) {
            return false;
        }

        const int fieldNumber = static_cast<int>(tag >> 3);
        const int wireType = static_cast<int>(tag & 0x7);

        if ((fieldNumber == 1) && (wireType == 2))
        {
            ProtoReader dimReader(nullptr, 0);

            if (!reader.readSubMessage(dimReader)) {
                return false;
            }

            int dimValue = 0;
            bool hasDimValue = false;

            while (!dimReader.atEnd())
            {
                quint64 dimTag = 0;

                if (!dimReader.readVarint(dimTag)) {
                    return false;
                }

                const int dimFieldNumber = static_cast<int>(dimTag >> 3);
                const int dimWireType = static_cast<int>(dimTag & 0x7);

                if ((dimFieldNumber == 1) && (dimWireType == 0))
                {
                    quint64 value = 0;

                    if (!dimReader.readVarint(value)) {
                        return false;
                    }

                    dimValue = static_cast<int>(value);
                    hasDimValue = true;
                }
                else if (!dimReader.skipField(dimWireType))
                {
                    return false;
                }
            }

            dims.append(hasDimValue ? dimValue : 0);
        }
        else if (!reader.skipField(wireType))
        {
            return false;
        }
    }

    return true;
}

bool parseOnnxTensorType(ProtoReader& reader, QVector<int>& dims)
{
    while (!reader.atEnd())
    {
        quint64 tag = 0;

        if (!reader.readVarint(tag)) {
            return false;
        }

        const int fieldNumber = static_cast<int>(tag >> 3);
        const int wireType = static_cast<int>(tag & 0x7);

        if ((fieldNumber == 2) && (wireType == 2))
        {
            ProtoReader shapeReader(nullptr, 0);
            return reader.readSubMessage(shapeReader) && parseOnnxTensorShape(shapeReader, dims);
        }

        if (!reader.skipField(wireType)) {
            return false;
        }
    }

    return false;
}

bool parseOnnxValueInfo(ProtoReader& reader, QVector<int>& dims)
{
    while (!reader.atEnd())
    {
        quint64 tag = 0;

        if (!reader.readVarint(tag)) {
            return false;
        }

        const int fieldNumber = static_cast<int>(tag >> 3);
        const int wireType = static_cast<int>(tag & 0x7);

        if ((fieldNumber == 2) && (wireType == 2))
        {
            ProtoReader typeReader(nullptr, 0);

            if (!reader.readSubMessage(typeReader)) {
                return false;
            }

            while (!typeReader.atEnd())
            {
                quint64 typeTag = 0;

                if (!typeReader.readVarint(typeTag)) {
                    return false;
                }

                const int typeFieldNumber = static_cast<int>(typeTag >> 3);
                const int typeWireType = static_cast<int>(typeTag & 0x7);

                if ((typeFieldNumber == 1) && (typeWireType == 2))
                {
                    ProtoReader tensorTypeReader(nullptr, 0);
                    return typeReader.readSubMessage(tensorTypeReader) && parseOnnxTensorType(tensorTypeReader, dims);
                }

                if (!typeReader.skipField(typeWireType)) {
                    return false;
                }
            }
        }
        else if (!reader.skipField(wireType))
        {
            return false;
        }
    }

    return false;
}

cv::Size readOnnxInputSize(const QString& modelPath)
{
    QFile modelFile(modelPath);

    if (!modelFile.open(QIODevice::ReadOnly)) {
        return cv::Size();
    }

    const QByteArray modelData = modelFile.readAll();
    ProtoReader modelReader(reinterpret_cast<const uchar *>(modelData.constData()), modelData.size());

    while (!modelReader.atEnd())
    {
        quint64 tag = 0;

        if (!modelReader.readVarint(tag)) {
            break;
        }

        const int fieldNumber = static_cast<int>(tag >> 3);
        const int wireType = static_cast<int>(tag & 0x7);

        if ((fieldNumber == 7) && (wireType == 2))
        {
            ProtoReader graphReader(nullptr, 0);

            if (!modelReader.readSubMessage(graphReader)) {
                break;
            }

            while (!graphReader.atEnd())
            {
                quint64 graphTag = 0;

                if (!graphReader.readVarint(graphTag)) {
                    return cv::Size();
                }

                const int graphFieldNumber = static_cast<int>(graphTag >> 3);
                const int graphWireType = static_cast<int>(graphTag & 0x7);

                if ((graphFieldNumber == 11) && (graphWireType == 2))
                {
                    ProtoReader inputReader(nullptr, 0);
                    QVector<int> dims;

                    if (!graphReader.readSubMessage(inputReader) || !parseOnnxValueInfo(inputReader, dims)) {
                        continue;
                    }

                    if (dims.size() >= 4)
                    {
                        const int width = dims.at(dims.size() - 1);
                        const int height = dims.at(dims.size() - 2);

                        if ((width > 0) && (height > 0)) {
                            return cv::Size(width, height);
                        }
                    }
                }
                else if (!graphReader.skipField(graphWireType))
                {
                    return cv::Size();
                }
            }
        }
        else if (!modelReader.skipField(wireType))
        {
            break;
        }
    }

    return cv::Size();
}

struct ObjectTargetVector
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

double degToRad(double degrees)
{
    return degrees * M_PI / 180.0;
}

double radToDeg(double radians)
{
    return radians * 180.0 / M_PI;
}

double normalizeDegrees(double degrees)
{
    double normalized = std::fmod(degrees, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }
    return normalized;
}

ObjectTargetVector vectorFromAltAz(double azimuthDegrees, double elevationDegrees)
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

double dot(const ObjectTargetVector& lhs, const ObjectTargetVector& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

ObjectTargetVector cross(const ObjectTargetVector& lhs, const ObjectTargetVector& rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x
    };
}

double length(const ObjectTargetVector& vector)
{
    return std::sqrt(dot(vector, vector));
}

ObjectTargetVector normalize(const ObjectTargetVector& vector)
{
    const double vectorLength = length(vector);
    if (vectorLength <= 0.0) {
        return {};
    }

    return {
        vector.x / vectorLength,
        vector.y / vectorLength,
        vector.z / vectorLength
    };
}

ObjectTargetVector rotateAroundAxis(const ObjectTargetVector& vector, const ObjectTargetVector& axis, double angleRadians)
{
    const double cosAngle = std::cos(angleRadians);
    const double sinAngle = std::sin(angleRadians);
    const ObjectTargetVector axisCrossVector = cross(axis, vector);
    const double axisDotVector = dot(axis, vector);

    return {
        vector.x * cosAngle + axisCrossVector.x * sinAngle + axis.x * axisDotVector * (1.0 - cosAngle),
        vector.y * cosAngle + axisCrossVector.y * sinAngle + axis.y * axisDotVector * (1.0 - cosAngle),
        vector.z * cosAngle + axisCrossVector.z * sinAngle + axis.z * axisDotVector * (1.0 - cosAngle)
    };
}

CameraSettings targetProjectionSettings(const CameraSettings& settings, const CameraPipelineFrame& frame)
{
    CameraSettings projectionSettings = settings;
    if (frame.m_plateSolve.m_solved)
    {
        projectionSettings.m_azimuth = frame.m_plateSolve.m_azimuth;
        projectionSettings.m_elevation = frame.m_plateSolve.m_elevation;
        projectionSettings.m_roll = frame.m_plateSolve.m_roll;
        projectionSettings.m_fov = frame.m_plateSolve.m_fov;
        projectionSettings.m_lensCenterOffsetX = frame.m_plateSolve.m_centerOffsetX;
        projectionSettings.m_lensCenterOffsetY = frame.m_plateSolve.m_centerOffsetY;
        projectionSettings.m_lensDistortionK1 = frame.m_plateSolve.m_distortionK1;
    }
    return projectionSettings;
}

bool pixelToAltAz(const CameraSettings& settings, const QSize& imageSize, const QPointF& pixel, double& azimuthDegrees, double& elevationDegrees)
{
    const int width = imageSize.width();
    const int height = imageSize.height();
    if ((width <= 0) || (height <= 0) || (settings.m_fov <= 0.0f)) {
        return false;
    }

    const double halfHorizontalFov = degToRad(settings.m_fov) * 0.5;
    if ((halfHorizontalFov <= 0.0) || (halfHorizontalFov >= M_PI * 0.5)) {
        return false;
    }

    const double azimuth = degToRad(settings.m_azimuth);
    const ObjectTargetVector center = normalize(vectorFromAltAz(settings.m_azimuth, settings.m_elevation));
    ObjectTargetVector right = normalize({std::cos(azimuth), -std::sin(azimuth), 0.0});
    ObjectTargetVector up = normalize(cross(right, center));
    if ((length(center) <= 0.0) || (length(right) <= 0.0) || (length(up) <= 0.0)) {
        return false;
    }

    const double rollRadians = degToRad(settings.m_roll);
    if (std::fabs(rollRadians) > 1e-9)
    {
        right = normalize(rotateAroundAxis(right, center, rollRadians));
        up = normalize(rotateAroundAxis(up, center, rollRadians));
    }

    const double principalPointX = static_cast<double>(width) * 0.5 + settings.m_lensCenterOffsetX;
    const double principalPointY = static_cast<double>(height) * 0.5 + settings.m_lensCenterOffsetY;
    const double aspect = static_cast<double>(height) / static_cast<double>(width);
    if (aspect <= 0.0) {
        return false;
    }

    const double normalizedX = (pixel.x() - principalPointX) / (0.5 * static_cast<double>(width));
    const double normalizedY = (principalPointY - pixel.y()) / (0.5 * static_cast<double>(height));
    double projectedX = normalizedX;
    double projectedY = normalizedY * aspect;

    if (std::fabs(settings.m_lensDistortionK1) > 1e-9)
    {
        const double distortedX = projectedX;
        const double distortedY = projectedY;
        for (int i = 0; i < 6; ++i)
        {
            const double radiusSquared = projectedX * projectedX + projectedY * projectedY;
            const double distortionScale = std::max(0.1, 1.0 + settings.m_lensDistortionK1 * radiusSquared);
            projectedX = distortedX / distortionScale;
            projectedY = distortedY / distortionScale;
        }
    }

    const double projectionRadius = std::sqrt(projectedX * projectedX + projectedY * projectedY);
    const double theta = [&]() -> double
    {
        switch (settings.m_lensProjection)
        {
        case CameraSettings::LensProjectionEquidistant:
            return projectionRadius * halfHorizontalFov;
        case CameraSettings::LensProjectionEquisolid:
            return 2.0 * std::asin(std::clamp(projectionRadius * std::sin(halfHorizontalFov * 0.5), -1.0, 1.0));
        case CameraSettings::LensProjectionRectilinear:
        default:
            return std::atan(projectionRadius * std::tan(halfHorizontalFov));
        }
    }();
    const double phi = std::atan2(projectedY, projectedX);
    const ObjectTargetVector tangent = normalize({
        right.x * std::cos(phi) + up.x * std::sin(phi),
        right.y * std::cos(phi) + up.y * std::sin(phi),
        right.z * std::cos(phi) + up.z * std::sin(phi)
    });
    const ObjectTargetVector target = normalize({
        center.x * std::cos(theta) + tangent.x * std::sin(theta),
        center.y * std::cos(theta) + tangent.y * std::sin(theta),
        center.z * std::cos(theta) + tangent.z * std::sin(theta)
    });

    if (length(target) <= 0.0) {
        return false;
    }

    azimuthDegrees = normalizeDegrees(radToDeg(std::atan2(target.x, target.y)));
    elevationDegrees = radToDeg(std::asin(std::clamp(target.z, -1.0, 1.0)));
    return std::isfinite(azimuthDegrees) && std::isfinite(elevationDegrees);
}

bool hasPlaybackPosition(const CameraPipelineFrame& frame)
{
    return (frame.m_playbackFrameNumber > 0) || (frame.m_playbackPositionMs >= 0);
}

bool hasPlaybackPosition(const CameraDetectionHistoryEntry& entry)
{
    return (entry.m_playbackFrameNumber > 0) || (entry.m_playbackPositionMs >= 0);
}

bool playbackFrameInRange(const CameraDetectionHistoryEntry& entry, int frameNumber)
{
    if (frameNumber <= 0 || entry.m_playbackFrameNumber <= 0) {
        return false;
    }

    const int lastFrameNumber = entry.m_lastPlaybackFrameNumber > 0
        ? entry.m_lastPlaybackFrameNumber
        : entry.m_playbackFrameNumber;
    const int rangeStart = std::min(entry.m_playbackFrameNumber, lastFrameNumber);
    const int rangeEnd = std::max(entry.m_playbackFrameNumber, lastFrameNumber);
    return (frameNumber >= rangeStart) && (frameNumber <= rangeEnd);
}

bool playbackPositionInRange(const CameraDetectionHistoryEntry& entry, qint64 positionMs)
{
    if (positionMs < 0 || entry.m_playbackPositionMs < 0) {
        return false;
    }

    const qint64 lastPositionMs = entry.m_lastPlaybackPositionMs >= 0
        ? entry.m_lastPlaybackPositionMs
        : entry.m_playbackPositionMs;
    const qint64 rangeStart = std::min(entry.m_playbackPositionMs, lastPositionMs);
    const qint64 rangeEnd = std::max(entry.m_playbackPositionMs, lastPositionMs);
    return (positionMs >= rangeStart) && (positionMs <= rangeEnd);
}

bool playbackRangesOverlap(const CameraDetectionHistoryEntry& lhs, const CameraDetectionHistoryEntry& rhs)
{
    if (!hasPlaybackPosition(lhs) || !hasPlaybackPosition(rhs)) {
        return false;
    }

    if ((lhs.m_playbackFrameNumber > 0) && (rhs.m_playbackFrameNumber > 0))
    {
        const int lhsLast = lhs.m_lastPlaybackFrameNumber > 0 ? lhs.m_lastPlaybackFrameNumber : lhs.m_playbackFrameNumber;
        const int rhsLast = rhs.m_lastPlaybackFrameNumber > 0 ? rhs.m_lastPlaybackFrameNumber : rhs.m_playbackFrameNumber;
        const int lhsStart = std::min(lhs.m_playbackFrameNumber, lhsLast);
        const int lhsEnd = std::max(lhs.m_playbackFrameNumber, lhsLast);
        const int rhsStart = std::min(rhs.m_playbackFrameNumber, rhsLast);
        const int rhsEnd = std::max(rhs.m_playbackFrameNumber, rhsLast);
        return lhsStart <= rhsEnd && rhsStart <= lhsEnd;
    }

    if ((lhs.m_playbackPositionMs >= 0) && (rhs.m_playbackPositionMs >= 0))
    {
        const qint64 lhsLast = lhs.m_lastPlaybackPositionMs >= 0 ? lhs.m_lastPlaybackPositionMs : lhs.m_playbackPositionMs;
        const qint64 rhsLast = rhs.m_lastPlaybackPositionMs >= 0 ? rhs.m_lastPlaybackPositionMs : rhs.m_playbackPositionMs;
        const qint64 lhsStart = std::min(lhs.m_playbackPositionMs, lhsLast);
        const qint64 lhsEnd = std::max(lhs.m_playbackPositionMs, lhsLast);
        const qint64 rhsStart = std::min(rhs.m_playbackPositionMs, rhsLast);
        const qint64 rhsEnd = std::max(rhs.m_playbackPositionMs, rhsLast);
        return lhsStart <= rhsEnd && rhsStart <= lhsEnd;
    }

    return false;
}
}

CameraObjectDetector::CameraObjectDetector(Camera *camera) :
    m_camera(camera),
    m_msgQueueToGUI(nullptr),
    m_msgQueueToFeature(nullptr),
    m_yoloInputSize(640, 640)
{
#ifdef CAMERA_TENSORRT_YOLO
    m_yoloTensorRt.setProgressCallback([this](bool active, const QString& modelPath, const QString& enginePath)
    {
        if (m_msgQueueToGUI) {
            m_msgQueueToGUI->push(MsgReportTensorRtConversion::create(active, modelPath, enginePath));
        }
    });
#endif
}

CameraObjectDetector::~CameraObjectDetector() = default;

bool CameraObjectDetector::handleStageMessage(const Message& cmd)
{
    if (MsgClearObjectDetectionHistory::match(cmd))
    {
        clearObjectDetectionHistory();
        return true;
    }

    return false;
}

void CameraObjectDetector::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraObjectDetector::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;
    CameraDetectionStage::applySettings(settings, settingsKeys, force);

    if (settingsKeys.contains("yoloModelPath") || (force && m_yoloLoadedModelPath != m_settings.m_yoloModelPath))
    {
        m_yoloNet = cv::dnn::Net();
#ifdef CAMERA_TENSORRT_YOLO
        m_yoloTensorRt.reset();
#endif
        m_yoloLoadedModelPath.clear();
        m_reportedErrorKeys.clear();
        m_appliedYoloDnnTarget = -1;
        m_yoloBatchedInferenceSupported = true;
    }

    if (settingsKeys.contains("yoloLabelsPath") || (force && m_yoloLoadedLabelsPath != m_settings.m_yoloLabelsPath))
    {
        m_yoloLabels.clear();
        m_yoloLoadedLabelsPath.clear();
        m_reportedErrorKeys.clear();
    }

    if ((force && !m_settings.m_yoloEnabled)
        || settingsKeys.contains("yoloEnabled")
        || settingsKeys.contains("yoloLabelsPath"))
    {
        clearObjectDetectionState();
    }
}

void CameraObjectDetector::captureActiveChanged(bool active)
{
    if (active) {
        clearObjectDetectionState();
    }
}

void CameraObjectDetector::processNewFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || !frame->hasImageData()) {
        return;
    }

    frame->m_detections.clear();

    if (!m_settings.m_yoloEnabled || m_settings.m_yoloModelPath.isEmpty())
    {
        const QDateTime detectionTime = frame->m_captureDateTime.isValid() ? frame->m_captureDateTime : QDateTime::currentDateTime();
        processObjectDetections(frame->m_detections, detectionTime, *frame);
        sendFirstObjectDetectionTarget(frame->m_detections, *frame);
        forwardFrame(frame);
        return;
    }

    if (!frame->ensureCpuImageFromCuda()) {
        return;
    }

    QImage convertedRgb;
    const QImage& rgb = ensureRgb888(frame->m_image, convertedRgb);
    cv::Mat mat = wrapRgb888Image(rgb);
    cv::Mat bgrMat;
    cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);
    const cv::Rect detectionRoi = resolveDetectionRoi(bgrMat.size());

    runYoloDetections(bgrMat, detectionRoi, frame->m_detections);

    const QDateTime detectionTime = frame->m_captureDateTime.isValid() ? frame->m_captureDateTime : QDateTime::currentDateTime();
    processObjectDetections(frame->m_detections, detectionTime, *frame);
    sendFirstObjectDetectionTarget(frame->m_detections, *frame);
    forwardFrame(frame);
}

void CameraObjectDetector::runYoloDetections(const cv::Mat& bgrMat, const cv::Rect& roi, QVector<CameraPipelineDetection>& detections)
{
    PROFILER_START();

    if (m_yoloLoadedLabelsPath != m_settings.m_yoloLabelsPath)
    {
        m_yoloLabels.clear();
        m_yoloLoadedLabelsPath.clear();

        if (!m_settings.m_yoloLabelsPath.isEmpty() && !(m_settings.m_yoloLabelsPath.startsWith("http://") || m_settings.m_yoloLabelsPath.startsWith("https://")))
        {
            QFile f(m_settings.m_yoloLabelsPath);
            if (f.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                QTextStream ts(&f);
                while (!ts.atEnd())
                {
                    const QString line = ts.readLine().trimmed();
                    if (!line.isEmpty()) {
                        m_yoloLabels.append(line);
                    }
                }
                m_yoloLoadedLabelsPath = m_settings.m_yoloLabelsPath;
            }
            else
            {
                qWarning() << "CameraObjectDetector::runYoloDetections: cannot open labels file:" << m_settings.m_yoloLabelsPath;
                reportErrorToFeature(
                    QStringLiteral("yoloLabels:%1").arg(m_settings.m_yoloLabelsPath),
                    tr("YOLO labels file load failed"),
                    tr("Failed to open YOLO labels file:\n%1").arg(m_settings.m_yoloLabelsPath));
            }
        }
    }

    const bool useTensorRt =
#ifdef CAMERA_TENSORRT_YOLO
        (m_settings.m_yoloDnnTarget == CameraSettings::TensorRT) || (m_settings.m_yoloDnnTarget == CameraSettings::TensorRT_FP16);
#else
        false;
#endif

    if ((m_yoloLoadedModelPath != m_settings.m_yoloModelPath) || (!useTensorRt && m_yoloNet.empty()))
    {
        m_yoloNet = cv::dnn::Net();
        m_yoloInputSize = cv::Size(640, 640);
        m_yoloLoadedModelPath.clear();
        // A newly-loaded net has default backend/target; force re-apply on next inference.
        m_appliedYoloDnnTarget = -1;
        m_yoloBatchedInferenceSupported = true;
#ifdef CAMERA_TENSORRT_YOLO
        m_yoloTensorRt.reset();
#endif

        if (!m_settings.m_yoloModelPath.isEmpty() && !(m_settings.m_yoloModelPath.startsWith("http://") || m_settings.m_yoloModelPath.startsWith("https://")))
        {
            const QString localFile = m_settings.m_yoloModelPath;
            try
            {
                const cv::Size modelInputSize = readOnnxInputSize(localFile);

                if ((modelInputSize.width > 0) && (modelInputSize.height > 0))
                {
                    m_yoloInputSize = modelInputSize;
                }
                else
                {
                    qWarning() << "CameraObjectDetector::runYoloDetections: unable to read model input size, using fallback 640x640 for"
                               << localFile;
                }

                if (!useTensorRt) {
                    m_yoloNet = cv::dnn::readNetFromONNX(localFile.toStdString());
                }

                m_yoloLoadedModelPath = m_settings.m_yoloModelPath;
                qDebug() << "CameraObjectDetector::runYoloDetections: loaded model" << localFile
                         << "with input size" << m_yoloInputSize.width << "x" << m_yoloInputSize.height;
            }
            catch (const cv::Exception& e)
            {
                qWarning() << "CameraObjectDetector::runYoloDetections: failed to load model:" << e.what();
                reportErrorToFeature(
                    QStringLiteral("yoloModelLoad:%1").arg(localFile),
                    tr("YOLO model load failed"),
                    tr("Failed to load YOLO model:\n%1\n\n%2").arg(localFile, QString::fromUtf8(e.what())));
                return;
            }
        }
        else
        {
            return;
        }
    }

    if (!useTensorRt && m_yoloNet.empty()) {
        return;
    }

    // Only push backend/target into the net when they change. The previous unconditional
    // setter on every frame can cause the network to rebuild its compute graph (especially
    // when transitioning between CPU and CUDA), and is a measurable cost even when the
    // values are identical.
    const int requestedTarget = static_cast<int>(m_settings.m_yoloDnnTarget);
    if (!useTensorRt && (m_appliedYoloDnnTarget != requestedTarget))
    {
        if (m_settings.m_yoloDnnTarget == CameraSettings::CUDA)
        {
            m_yoloNet.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            m_yoloNet.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        }
        else if (m_settings.m_yoloDnnTarget == CameraSettings::CUDA_FP16)
        {
            m_yoloNet.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            m_yoloNet.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
        }
        else
        {
            m_yoloNet.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
            m_yoloNet.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        }
        m_appliedYoloDnnTarget = requestedTarget;
        m_yoloBatchedInferenceSupported = true;
    }

    const QVector<cv::Rect> tileRects = makeYoloTiles(roi);

    // Letterbox each ROI/tile, batch inference where possible, then map detections back
    // to full-frame coordinates before global NMS removes overlap duplicates.
    const int targetW = m_yoloInputSize.width;
    const int targetH = m_yoloInputSize.height;
    if ((targetW <= 0) || (targetH <= 0) || tileRects.isEmpty()) {
        return;
    }

    std::vector<cv::Mat> letterboxes;
    std::vector<int> padXs;
    std::vector<int> padYs;
    std::vector<float> invScales;
    letterboxes.reserve(static_cast<size_t>(tileRects.size()));
    padXs.reserve(static_cast<size_t>(tileRects.size()));
    padYs.reserve(static_cast<size_t>(tileRects.size()));
    invScales.reserve(static_cast<size_t>(tileRects.size()));

    for (const cv::Rect& tileRect : tileRects)
    {
        cv::Mat tileMat = bgrMat(tileRect);
        const float scale = std::min(
            static_cast<float>(targetW) / static_cast<float>(std::max(1, tileMat.cols)),
            static_cast<float>(targetH) / static_cast<float>(std::max(1, tileMat.rows)));
        const int newW = std::max(1, static_cast<int>(std::round(tileMat.cols * scale)));
        const int newH = std::max(1, static_cast<int>(std::round(tileMat.rows * scale)));
        const int padX = (targetW - newW) / 2;
        const int padY = (targetH - newH) / 2;

        cv::Mat letterbox(targetH, targetW, tileMat.type(), cv::Scalar(114, 114, 114));
        cv::Mat resized;
        cv::resize(tileMat, resized, cv::Size(newW, newH), 0.0, 0.0, cv::INTER_LINEAR);
        resized.copyTo(letterbox(cv::Rect(padX, padY, newW, newH)));

        letterboxes.push_back(letterbox);
        padXs.push_back(padX);
        padYs.push_back(padY);
        invScales.push_back((scale > 0.0f) ? (1.0f / scale) : 1.0f);
    }

    const float confThresh = static_cast<float>(m_settings.m_yoloConfThreshold);
    const float nmsThresh = static_cast<float>(m_settings.m_yoloNmsThreshold);
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> classIds;

    auto decodeOutput = [&](cv::Mat output, int firstTile, int tileCount) -> bool
    {
        if ((output.dims == 3) && (output.size[0] == tileCount))
        {
            for (int localTileIndex = 0; localTileIndex < tileCount; ++localTileIndex)
            {
                const int tileIndex = firstTile + localTileIndex;
                cv::Mat det(output.size[1], output.size[2], output.type(), output.ptr(localTileIndex));
                decodeYoloDetections(det, tileRects[tileIndex], padXs[tileIndex], padYs[tileIndex], invScales[tileIndex], boxes, scores, classIds);
            }
            return true;
        }

        if ((output.dims == 2) && (tileCount > 1))
        {
            const bool isV8Style = output.rows < output.cols;
            if (isV8Style && ((output.cols % tileCount) == 0))
            {
                const int anchorsPerTile = output.cols / tileCount;
                for (int localTileIndex = 0; localTileIndex < tileCount; ++localTileIndex)
                {
                    const int tileIndex = firstTile + localTileIndex;
                    cv::Mat det = output.colRange(localTileIndex * anchorsPerTile, (localTileIndex + 1) * anchorsPerTile);
                    decodeYoloDetections(det, tileRects[tileIndex], padXs[tileIndex], padYs[tileIndex], invScales[tileIndex], boxes, scores, classIds);
                }
                return true;
            }

            if (!isV8Style && ((output.rows % tileCount) == 0))
            {
                const int anchorsPerTile = output.rows / tileCount;
                for (int localTileIndex = 0; localTileIndex < tileCount; ++localTileIndex)
                {
                    const int tileIndex = firstTile + localTileIndex;
                    cv::Mat det = output.rowRange(localTileIndex * anchorsPerTile, (localTileIndex + 1) * anchorsPerTile);
                    decodeYoloDetections(det, tileRects[tileIndex], padXs[tileIndex], padYs[tileIndex], invScales[tileIndex], boxes, scores, classIds);
                }
                return true;
            }

            qWarning() << "CameraObjectDetector::runYoloDetections: unable to split batched YOLO output"
                       << output.rows << "x" << output.cols << "tiles" << tileCount;
            return false;
        }

        if (tileCount == 1)
        {
            cv::Mat det = output;
            if (det.dims == 3) {
                det = det.reshape(1, det.size[1]);
            }
            decodeYoloDetections(det, tileRects[firstTile], padXs[firstTile], padYs[firstTile], invScales[firstTile], boxes, scores, classIds);
            return true;
        }

        return false;
    };

    bool useOpenCvDnn = !useTensorRt;
#ifdef CAMERA_TENSORRT_YOLO
    if (useTensorRt)
    {
        QString tensorRtError;
        std::vector<cv::Mat> tensorRtOutputs;
        const int maxBatch = std::max(1, std::min(16, static_cast<int>(letterboxes.size())));
        if (m_yoloTensorRt.ensureLoaded(
                m_settings.m_yoloModelPath,
                m_yoloInputSize,
                maxBatch,
                m_settings.m_yoloDnnTarget == CameraSettings::TensorRT_FP16,
                tensorRtError)
            && m_yoloTensorRt.infer(letterboxes, tensorRtOutputs, tensorRtError))
        {
            for (int tileIndex = 0; tileIndex < static_cast<int>(tensorRtOutputs.size()); ++tileIndex) {
                decodeOutput(tensorRtOutputs[static_cast<size_t>(tileIndex)], tileIndex, 1);
            }
            useOpenCvDnn = false;
        }
        else
        {
            qWarning() << "CameraObjectDetector::runYoloDetections: TensorRT inference unavailable, falling back to OpenCV DNN:"
                       << tensorRtError;
            m_yoloTensorRt.reset();

            if (m_yoloNet.empty())
            {
                try
                {
                    m_yoloNet = cv::dnn::readNetFromONNX(m_settings.m_yoloModelPath.toStdString());
                    m_appliedYoloDnnTarget = -1;
                }
                catch (const cv::Exception& e)
                {
                    qWarning() << "CameraObjectDetector::runYoloDetections: OpenCV fallback model load failed:" << e.what();
                    return;
                }
            }
            useOpenCvDnn = true;
        }
    }
#endif

    bool batchInferenceFailed = false;
    if (useOpenCvDnn && ((tileRects.size() == 1) || m_yoloBatchedInferenceSupported))
    {
        std::vector<cv::Mat> outputs;
        cv::Mat blob;
        cv::dnn::blobFromImages(letterboxes, blob, 1.0 / 255.0, m_yoloInputSize, cv::Scalar(), true, false);
        m_yoloNet.setInput(blob);

        try
        {
            m_yoloNet.forward(outputs, m_yoloNet.getUnconnectedOutLayersNames());
            if (!outputs.empty()) {
                decodeOutput(outputs[0], 0, tileRects.size());
            }
        }
        catch (const cv::Exception& e)
        {
            if (tileRects.size() <= 1)
            {
                qWarning() << "CameraObjectDetector::runYoloDetections: inference failed:" << e.what();
                return;
            }

            m_yoloBatchedInferenceSupported = false;
            batchInferenceFailed = true;
            qWarning() << "CameraObjectDetector::runYoloDetections: batched inference failed for this model/target; using per-tile inference:" << e.what();
        }
    }

    if (useOpenCvDnn && (batchInferenceFailed || ((tileRects.size() > 1) && !m_yoloBatchedInferenceSupported)))
    {
        for (int tileIndex = 0; tileIndex < tileRects.size(); ++tileIndex)
        {
            std::vector<cv::Mat> tileOutputs;
            cv::Mat tileBlob;
            cv::dnn::blobFromImage(letterboxes[tileIndex], tileBlob, 1.0 / 255.0, m_yoloInputSize, cv::Scalar(), true, false);
            m_yoloNet.setInput(tileBlob);

            try
            {
                m_yoloNet.forward(tileOutputs, m_yoloNet.getUnconnectedOutLayersNames());
            }
            catch (const cv::Exception& e)
            {
                qWarning() << "CameraObjectDetector::runYoloDetections: tile inference failed:" << tileIndex << e.what();
                continue;
            }

            if (!tileOutputs.empty()) {
                decodeOutput(tileOutputs[0], tileIndex, 1);
            }
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, scores, confThresh, nmsThresh, indices);

    detections.reserve(static_cast<qsizetype>(indices.size()));
    for (int idx : indices)
    {
        QString label;
        if (!m_yoloLabels.isEmpty() && classIds[idx] < m_yoloLabels.size()) {
            label = m_yoloLabels[classIds[idx]];
        } else {
            label = QStringLiteral("cls%1").arg(classIds[idx]);
        }

        CameraPipelineDetection detection;
        detection.m_box = QRect(boxes[idx].x, boxes[idx].y, boxes[idx].width, boxes[idx].height);
        if (intersectsExclusionRects(detection.m_box)) {
            continue;
        }
        detection.m_label = label;
        detection.m_score = scores[idx];
        detections.append(detection);
    }

    PROFILER_STOP(tileRects.size() > 1 ? "YOLO tiled" : "YOLO");
}

void CameraObjectDetector::decodeYoloDetections(const cv::Mat& det, const cv::Rect& tileRect, int padX, int padY, float invScale,
    std::vector<cv::Rect>& boxes, std::vector<float>& scores, std::vector<int>& classIds) const
{
    if (det.empty()) {
        return;
    }

    const float confThresh = static_cast<float>(m_settings.m_yoloConfThreshold);
    const bool isV8Style = (det.rows < det.cols);

    if (isV8Style)
    {
        cv::Mat detT;
        cv::transpose(det, detT);
        const int numAnchors = detT.rows;
        const int numClasses = detT.cols - 4;
        if (numClasses <= 0) {
            return;
        }

        for (int a = 0; a < numAnchors; ++a)
        {
            const float* row = detT.ptr<float>(a);
            float bestScore = 0.0f;
            int bestClass = 0;
            const float* classes = row + 4;
            for (int c = 0; c < numClasses; ++c)
            {
                if (classes[c] > bestScore) {
                    bestScore = classes[c];
                    bestClass = c;
                }
            }
            if (bestScore < confThresh) {
                continue;
            }

            const float cx = (row[0] - padX) * invScale;
            const float cy = (row[1] - padY) * invScale;
            const float w = row[2] * invScale;
            const float h = row[3] * invScale;

            boxes.push_back(cv::Rect(
                tileRect.x + static_cast<int>(cx - w / 2.0f),
                tileRect.y + static_cast<int>(cy - h / 2.0f),
                static_cast<int>(w),
                static_cast<int>(h)));
            scores.push_back(bestScore);
            classIds.push_back(bestClass);
        }
    }
    else
    {
        const int numAnchors = det.rows;
        const int numClasses = det.cols - 5;
        if (numClasses < 0) {
            return;
        }

        for (int a = 0; a < numAnchors; ++a)
        {
            const float* row = det.ptr<float>(a);
            const float objectness = row[4];
            if (objectness < confThresh) {
                continue;
            }

            float bestScore = 0.0f;
            int bestClass = 0;
            const float* classes = row + 5;
            for (int c = 0; c < numClasses; ++c)
            {
                const float s = objectness * classes[c];
                if (s > bestScore) {
                    bestScore = s;
                    bestClass = c;
                }
            }
            if (bestScore < confThresh) {
                continue;
            }

            const float cx = (row[0] - padX) * invScale;
            const float cy = (row[1] - padY) * invScale;
            const float w = row[2] * invScale;
            const float h = row[3] * invScale;

            boxes.push_back(cv::Rect(
                tileRect.x + static_cast<int>(cx - w / 2.0f),
                tileRect.y + static_cast<int>(cy - h / 2.0f),
                static_cast<int>(w),
                static_cast<int>(h)));
            scores.push_back(bestScore);
            classIds.push_back(bestClass);
        }
    }
}

QVector<cv::Rect> CameraObjectDetector::makeYoloTiles(const cv::Rect& roi) const
{
    const int tileW = std::min(m_yoloInputSize.width, roi.width);
    const int tileH = std::min(m_yoloInputSize.height, roi.height);
    QVector<cv::Rect> tiles;

    if ((tileW <= 0) || (tileH <= 0)) {
        return tiles;
    }

    if (!m_settings.m_yoloTileLargeImages || ((roi.width <= m_yoloInputSize.width) && (roi.height <= m_yoloInputSize.height)))
    {
        tiles.append(roi);
        return tiles;
    }

    const int overlapPercent = qBound(0, m_settings.m_yoloTileOverlapPercent, 90);
    auto makeStarts = [overlapPercent](int start, int length, int tileSize) -> QVector<int>
    {
        QVector<int> starts;
        if (length <= tileSize)
        {
            starts.append(start);
            return starts;
        }

        const int step = std::max(1, static_cast<int>(std::round(tileSize * (100 - overlapPercent) / 100.0)));
        const int last = start + length - tileSize;
        for (int pos = start; pos < last; pos += step) {
            starts.append(pos);
        }
        if (starts.isEmpty() || starts.last() != last) {
            starts.append(last);
        }
        return starts;
    };

    const QVector<int> xStarts = makeStarts(roi.x, roi.width, tileW);
    const QVector<int> yStarts = makeStarts(roi.y, roi.height, tileH);
    tiles.reserve(xStarts.size() * yStarts.size());

    for (int y : yStarts)
    {
        for (int x : xStarts) {
            tiles.append(cv::Rect(x, y, tileW, tileH));
        }
    }

    return tiles;
}

void CameraObjectDetector::clearObjectDetectionState()
{
    m_detectedObjectClasses.clear();
    m_pendingDisappearStates.clear();
}

void CameraObjectDetector::clearObjectDetectionHistory()
{
    m_activeObjectDetectionHistory.clear();
    m_completedObjectDetectionHistory.clear();
    reportObjectDetectionHistoryToGUI();
}

QList<CameraDetectionHistoryEntry> CameraObjectDetector::getObjectDetectionHistorySnapshot() const
{
    QList<CameraDetectionHistoryEntry> history = m_completedObjectDetectionHistory;
    for (auto it = m_activeObjectDetectionHistory.cbegin(); it != m_activeObjectDetectionHistory.cend(); ++it) {
        history.append(it.value());
    }

    std::sort(history.begin(), history.end(), [](const CameraDetectionHistoryEntry& lhs, const CameraDetectionHistoryEntry& rhs) {
        if (lhs.m_firstDetected != rhs.m_firstDetected) {
            return lhs.m_firstDetected > rhs.m_firstDetected;
        }

        return lhs.m_label < rhs.m_label;
    });

    return history;
}

void CameraObjectDetector::reportObjectDetectionHistoryToGUI() const
{
    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportObjectDetectionHistory::create(getObjectDetectionHistorySnapshot()));
    }
}

void CameraObjectDetector::reportErrorToFeature(const QString& errorKey, const QString& title, const QString& errorMessage)
{
    if (!m_msgQueueToFeature || m_reportedErrorKeys.contains(errorKey)) {
        return;
    }

    m_reportedErrorKeys.insert(errorKey);
    m_msgQueueToFeature->push(Camera::MsgReportError::create(title, errorMessage));
}

int CameraObjectDetector::findCompletedPlaybackHistoryIndex(const QString& className, const CameraPipelineFrame& frame) const
{
    if (!hasPlaybackPosition(frame)) {
        return -1;
    }

    for (int index = 0; index < m_completedObjectDetectionHistory.size(); ++index)
    {
        const CameraDetectionHistoryEntry& entry = m_completedObjectDetectionHistory.at(index);
        if (entry.m_label != className) {
            continue;
        }

        if (playbackFrameInRange(entry, frame.m_playbackFrameNumber)
            || playbackPositionInRange(entry, frame.m_playbackPositionMs))
        {
            return index;
        }
    }

    return -1;
}

int CameraObjectDetector::findCompletedPlaybackHistoryIndex(const CameraDetectionHistoryEntry& entry) const
{
    if (!hasPlaybackPosition(entry)) {
        return -1;
    }

    for (int index = 0; index < m_completedObjectDetectionHistory.size(); ++index)
    {
        const CameraDetectionHistoryEntry& completed = m_completedObjectDetectionHistory.at(index);
        if ((completed.m_label == entry.m_label) && playbackRangesOverlap(completed, entry)) {
            return index;
        }
    }

    return -1;
}

void CameraObjectDetector::updateHistoryEntryPlayback(CameraDetectionHistoryEntry& entry, const CameraPipelineFrame& frame) const
{
    if (frame.m_playbackFrameNumber > 0)
    {
        if ((entry.m_playbackFrameNumber <= 0) || (frame.m_playbackFrameNumber < entry.m_playbackFrameNumber)) {
            entry.m_playbackFrameNumber = frame.m_playbackFrameNumber;
        }
        if ((entry.m_lastPlaybackFrameNumber <= 0) || (frame.m_playbackFrameNumber > entry.m_lastPlaybackFrameNumber)) {
            entry.m_lastPlaybackFrameNumber = frame.m_playbackFrameNumber;
        }
    }

    if (frame.m_playbackPositionMs >= 0)
    {
        if ((entry.m_playbackPositionMs < 0) || (frame.m_playbackPositionMs < entry.m_playbackPositionMs)) {
            entry.m_playbackPositionMs = frame.m_playbackPositionMs;
        }
        if ((entry.m_lastPlaybackPositionMs < 0) || (frame.m_playbackPositionMs > entry.m_lastPlaybackPositionMs)) {
            entry.m_lastPlaybackPositionMs = frame.m_playbackPositionMs;
        }
    }
}

void CameraObjectDetector::mergeCompletedHistoryEntry(const CameraDetectionHistoryEntry& entry)
{
    const int existingIndex = findCompletedPlaybackHistoryIndex(entry);
    if (existingIndex < 0)
    {
        m_completedObjectDetectionHistory.append(entry);
        return;
    }

    CameraDetectionHistoryEntry& existing = m_completedObjectDetectionHistory[existingIndex];
    if (entry.m_firstDetected.isValid()
        && (!existing.m_firstDetected.isValid() || (entry.m_firstDetected < existing.m_firstDetected)))
    {
        existing.m_firstDetected = entry.m_firstDetected;
    }
    if (entry.m_disappeared.isValid()
        && (!existing.m_disappeared.isValid() || (entry.m_disappeared > existing.m_disappeared)))
    {
        existing.m_disappeared = entry.m_disappeared;
    }
    if (entry.m_playbackFrameNumber > 0
        && ((existing.m_playbackFrameNumber <= 0) || (entry.m_playbackFrameNumber < existing.m_playbackFrameNumber)))
    {
        existing.m_playbackFrameNumber = entry.m_playbackFrameNumber;
    }
    if (entry.m_lastPlaybackFrameNumber > 0
        && ((existing.m_lastPlaybackFrameNumber <= 0) || (entry.m_lastPlaybackFrameNumber > existing.m_lastPlaybackFrameNumber)))
    {
        existing.m_lastPlaybackFrameNumber = entry.m_lastPlaybackFrameNumber;
    }
    if (entry.m_playbackPositionMs >= 0
        && ((existing.m_playbackPositionMs < 0) || (entry.m_playbackPositionMs < existing.m_playbackPositionMs)))
    {
        existing.m_playbackPositionMs = entry.m_playbackPositionMs;
    }
    if (entry.m_lastPlaybackPositionMs >= 0
        && ((existing.m_lastPlaybackPositionMs < 0) || (entry.m_lastPlaybackPositionMs > existing.m_lastPlaybackPositionMs)))
    {
        existing.m_lastPlaybackPositionMs = entry.m_lastPlaybackPositionMs;
    }
    existing.m_peakConfidence = std::max(existing.m_peakConfidence, entry.m_peakConfidence);
}

void CameraObjectDetector::sendFirstObjectDetectionTarget(const QVector<CameraPipelineDetection>& detections, const CameraPipelineFrame& frame) const
{
    if (!m_camera || detections.isEmpty()) {
        return;
    }

    const CameraPipelineDetection& detection = detections.first();
    if (detection.m_box.isEmpty()) {
        return;
    }

    const QPointF center(
        static_cast<double>(detection.m_box.left()) + static_cast<double>(detection.m_box.width()) * 0.5,
        static_cast<double>(detection.m_box.top()) + static_cast<double>(detection.m_box.height()) * 0.5);
    const CameraSettings projectionSettings = targetProjectionSettings(m_settings, frame);
    double azimuth = 0.0;
    double elevation = 0.0;

    if (!pixelToAltAz(projectionSettings, frame.imageSize(), center, azimuth, elevation)) {
        return;
    }

    QList<ObjectPipe*> targetPipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_camera, "target", targetPipes);

    for (const ObjectPipe *pipe : targetPipes)
    {
        MessageQueue *messageQueue = qobject_cast<MessageQueue*>(pipe->m_element);
        if (!messageQueue) {
            continue;
        }

        SWGSDRangel::SWGTargetAzimuthElevation *swgTarget = new SWGSDRangel::SWGTargetAzimuthElevation();
        swgTarget->setName(new QString(detection.m_label.isEmpty() ? QStringLiteral("Camera object") : detection.m_label));
        swgTarget->setAzimuth(azimuth);
        swgTarget->setElevation(elevation);
        messageQueue->push(MainCore::MsgTargetAzimuthElevation::create(m_camera, swgTarget));
    }
}

void CameraObjectDetector::processObjectDetections(const QVector<CameraPipelineDetection>& detections, const QDateTime& now, CameraPipelineFrame& frame)
{
    QHash<QString, float> currentDetectedScores;
    QSet<QString> currentDetectedClasses;
    bool historyChanged = false;

    for (const CameraPipelineDetection& detection : detections)
    {
        if (detection.m_label.isEmpty()) {
            continue;
        }

        currentDetectedClasses.insert(detection.m_label);
        float& peakScore = currentDetectedScores[detection.m_label];
        peakScore = std::max(peakScore, detection.m_score);
    }

    for (const QString& className : currentDetectedClasses)
    {
        m_pendingDisappearStates.remove(className);

        const float currentPeakScore = currentDetectedScores.value(className);

        auto activeHistoryIt = m_activeObjectDetectionHistory.find(className);
        if (activeHistoryIt == m_activeObjectDetectionHistory.end())
        {
            CameraDetectionHistoryEntry entry;
            const int completedIndex = findCompletedPlaybackHistoryIndex(className, frame);
            if (completedIndex >= 0)
            {
                entry = m_completedObjectDetectionHistory.takeAt(completedIndex);
                entry.m_disappeared = QDateTime();
            }
            else
            {
                entry.m_label = className;
                entry.m_firstDetected = now;
            }
            updateHistoryEntryPlayback(entry, frame);
            entry.m_peakConfidence = std::max(entry.m_peakConfidence, currentPeakScore);
            activeHistoryIt = m_activeObjectDetectionHistory.insert(className, entry);
            historyChanged = true;
        }
        else
        {
            if (currentPeakScore > activeHistoryIt->m_peakConfidence)
            {
                activeHistoryIt->m_peakConfidence = currentPeakScore;
                historyChanged = true;
            }

            const qint64 previousStartMs = activeHistoryIt->m_playbackPositionMs;
            const qint64 previousLastMs = activeHistoryIt->m_lastPlaybackPositionMs;
            const int previousStartFrame = activeHistoryIt->m_playbackFrameNumber;
            const int previousLastFrame = activeHistoryIt->m_lastPlaybackFrameNumber;
            updateHistoryEntryPlayback(*activeHistoryIt, frame);
            historyChanged = historyChanged
                || (previousStartMs != activeHistoryIt->m_playbackPositionMs)
                || (previousLastMs != activeHistoryIt->m_lastPlaybackPositionMs)
                || (previousStartFrame != activeHistoryIt->m_playbackFrameNumber)
                || (previousLastFrame != activeHistoryIt->m_lastPlaybackFrameNumber);
        }

        if (!m_detectedObjectClasses.contains(className))
        {
            m_detectedObjectClasses.insert(className);
        }
    }

    for (const QString& className : m_detectedObjectClasses)
    {
        if (!currentDetectedClasses.contains(className) && !m_pendingDisappearStates.contains(className))
        {
            PendingDisappearState state;
            state.m_firstMissing = now;
            state.m_deadline = now;
            m_pendingDisappearStates.insert(className, state);
        }
    }

    QMutableHashIterator<QString, PendingDisappearState> it(m_pendingDisappearStates);
    while (it.hasNext())
    {
        it.next();

        if (currentDetectedClasses.contains(it.key())) {
            it.remove();
            continue;
        }

        if (it.value().m_deadline <= now)
        {
            m_detectedObjectClasses.remove(it.key());
            auto activeHistoryIt = m_activeObjectDetectionHistory.find(it.key());
            if (activeHistoryIt != m_activeObjectDetectionHistory.end())
            {
                activeHistoryIt->m_disappeared = it.value().m_firstMissing;
                mergeCompletedHistoryEntry(activeHistoryIt.value());
                m_activeObjectDetectionHistory.erase(activeHistoryIt);
                historyChanged = true;
            }
            it.remove();
        }
    }

    if (historyChanged) {
        reportObjectDetectionHistoryToGUI();
    }
}
