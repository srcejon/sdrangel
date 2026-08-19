///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#include "cameramediametadata.h"

#include <cmath>

#include <QImageWriter>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>

#include "camerapipelineframe.h"
#include "cameraobservationcontext.h"
#include "camerasettings.h"

namespace {

double finiteNumber(const QJsonObject& object, const QString& key, double fallback)
{
    const double value = object.value(key).toDouble(fallback);
    return std::isfinite(value) ? value : fallback;
}

}

QString CameraMediaMetadata::metadataKey()
{
    return QStringLiteral("SDRangel.Camera");
}

CameraMediaMetadata CameraMediaMetadata::fromFrame(
    const CameraSettings& settings,
    const CameraPipelineFrame& frame)
{
    CameraMediaMetadata metadata;
    const CameraSettings effectiveSettings = CameraObservationContext::projectionSettingsForFrame(settings, frame);

    metadata.m_valid = true;
    const QDateTime observationDateTime = CameraObservationContext::dateTimeForFrame(settings, frame);
    metadata.m_captureDateTimeUtc = observationDateTime.isValid()
        ? observationDateTime.toUTC()
        : QDateTime();
    metadata.m_latitude = effectiveSettings.m_latitude;
    metadata.m_longitude = effectiveSettings.m_longitude;
    metadata.m_altitude = effectiveSettings.m_altitude;
    metadata.m_azimuth = effectiveSettings.m_azimuth;
    metadata.m_elevation = effectiveSettings.m_elevation;
    metadata.m_roll = effectiveSettings.m_roll;
    metadata.m_fov = effectiveSettings.m_fov;
    metadata.m_lensProjection = static_cast<int>(effectiveSettings.m_lensProjection);
    metadata.m_lensCenterOffsetX = effectiveSettings.m_lensCenterOffsetX;
    metadata.m_lensCenterOffsetY = effectiveSettings.m_lensCenterOffsetY;
    metadata.m_lensDistortionK1 = effectiveSettings.m_lensDistortionK1;
    metadata.m_lensMirror = effectiveSettings.m_lensMirror;

    if (frame.m_imageTransform.isValid())
    {
        metadata.m_opticalSize = frame.m_imageTransform.m_opticalSize;
        metadata.m_opticalToImage = frame.m_imageTransform.m_opticalToImage;
        metadata.m_imageTransformValid = true;
    }

    return metadata;
}

CameraMediaMetadata CameraMediaMetadata::fromJson(
    const QByteArray& json,
    QString *errorMessage)
{
    CameraMediaMetadata metadata;
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(json, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        if (errorMessage) {
            *errorMessage = parseError.error != QJsonParseError::NoError
                ? parseError.errorString()
                : QStringLiteral("Camera metadata is not a JSON object");
        }
        return metadata;
    }

    const QJsonObject root = document.object();
    if (root.value(QStringLiteral("version")).toInt() != CurrentVersion)
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Unsupported camera metadata version");
        }
        return metadata;
    }

    const QJsonObject site = root.value(QStringLiteral("site")).toObject();
    const QJsonObject direction = root.value(QStringLiteral("direction")).toObject();
    const QJsonObject projection = root.value(QStringLiteral("projection")).toObject();
    const QJsonObject transform = root.value(QStringLiteral("imageTransform")).toObject();

    metadata.m_captureDateTimeUtc = QDateTime::fromString(
        root.value(QStringLiteral("captureDateTimeUtc")).toString(),
        Qt::ISODateWithMs);
    if (metadata.m_captureDateTimeUtc.isValid()) {
        metadata.m_captureDateTimeUtc = metadata.m_captureDateTimeUtc.toUTC();
    }
    metadata.m_latitude = finiteNumber(site, QStringLiteral("latitude"), 0.0);
    metadata.m_longitude = finiteNumber(site, QStringLiteral("longitude"), 0.0);
    metadata.m_altitude = finiteNumber(site, QStringLiteral("altitude"), 0.0);
    metadata.m_azimuth = finiteNumber(direction, QStringLiteral("azimuth"), 0.0);
    metadata.m_elevation = finiteNumber(direction, QStringLiteral("elevation"), 0.0);
    metadata.m_roll = finiteNumber(direction, QStringLiteral("roll"), 0.0);
    metadata.m_fov = finiteNumber(projection, QStringLiteral("fov"), 0.0);
    metadata.m_lensProjection = projection.value(QStringLiteral("type")).toInt(0);
    metadata.m_lensCenterOffsetX = finiteNumber(projection, QStringLiteral("centerOffsetX"), 0.0);
    metadata.m_lensCenterOffsetY = finiteNumber(projection, QStringLiteral("centerOffsetY"), 0.0);
    metadata.m_lensDistortionK1 = finiteNumber(projection, QStringLiteral("distortionK1"), 0.0);
    metadata.m_lensMirror = projection.value(QStringLiteral("mirror")).toBool(false);

    const int opticalWidth = transform.value(QStringLiteral("opticalWidth")).toInt();
    const int opticalHeight = transform.value(QStringLiteral("opticalHeight")).toInt();
    const QTransform opticalToImage(
        finiteNumber(transform, QStringLiteral("m11"), 1.0),
        finiteNumber(transform, QStringLiteral("m12"), 0.0),
        0.0,
        finiteNumber(transform, QStringLiteral("m21"), 0.0),
        finiteNumber(transform, QStringLiteral("m22"), 1.0),
        0.0,
        finiteNumber(transform, QStringLiteral("dx"), 0.0),
        finiteNumber(transform, QStringLiteral("dy"), 0.0),
        1.0);
    if ((opticalWidth > 0) && (opticalHeight > 0)
        && opticalToImage.isAffine() && opticalToImage.isInvertible())
    {
        metadata.m_opticalSize = QSize(opticalWidth, opticalHeight);
        metadata.m_opticalToImage = opticalToImage;
        metadata.m_imageTransformValid = true;
    }

    metadata.m_valid = true;
    return metadata;
}

CameraMediaMetadata CameraMediaMetadata::fromImage(
    const QImage& image,
    QString *errorMessage)
{
    const QString json = image.text(metadataKey());
    if (json.isEmpty()) {
        return CameraMediaMetadata();
    }
    return fromJson(json.toUtf8(), errorMessage);
}

QByteArray CameraMediaMetadata::toJson() const
{
    if (!m_valid) {
        return QByteArray();
    }

    QJsonObject site;
    site.insert(QStringLiteral("latitude"), m_latitude);
    site.insert(QStringLiteral("longitude"), m_longitude);
    site.insert(QStringLiteral("altitude"), m_altitude);

    QJsonObject direction;
    direction.insert(QStringLiteral("azimuth"), m_azimuth);
    direction.insert(QStringLiteral("elevation"), m_elevation);
    direction.insert(QStringLiteral("roll"), m_roll);

    QJsonObject projection;
    projection.insert(QStringLiteral("fov"), m_fov);
    projection.insert(QStringLiteral("type"), m_lensProjection);
    projection.insert(QStringLiteral("centerOffsetX"), m_lensCenterOffsetX);
    projection.insert(QStringLiteral("centerOffsetY"), m_lensCenterOffsetY);
    projection.insert(QStringLiteral("distortionK1"), m_lensDistortionK1);
    projection.insert(QStringLiteral("mirror"), m_lensMirror);

    QJsonObject root;
    root.insert(QStringLiteral("version"), CurrentVersion);
    if (m_captureDateTimeUtc.isValid()) {
        root.insert(QStringLiteral("captureDateTimeUtc"), m_captureDateTimeUtc.toString(Qt::ISODateWithMs));
    }
    root.insert(QStringLiteral("site"), site);
    root.insert(QStringLiteral("direction"), direction);
    root.insert(QStringLiteral("projection"), projection);

    if (m_imageTransformValid)
    {
        QJsonObject transform;
        transform.insert(QStringLiteral("opticalWidth"), m_opticalSize.width());
        transform.insert(QStringLiteral("opticalHeight"), m_opticalSize.height());
        transform.insert(QStringLiteral("m11"), m_opticalToImage.m11());
        transform.insert(QStringLiteral("m12"), m_opticalToImage.m12());
        transform.insert(QStringLiteral("m21"), m_opticalToImage.m21());
        transform.insert(QStringLiteral("m22"), m_opticalToImage.m22());
        transform.insert(QStringLiteral("dx"), m_opticalToImage.dx());
        transform.insert(QStringLiteral("dy"), m_opticalToImage.dy());
        root.insert(QStringLiteral("imageTransform"), transform);
    }

    return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

void CameraMediaMetadata::applyToFrame(
    CameraPipelineFrame& frame,
    qint64 captureOffsetMs) const
{
    if (!m_valid) {
        return;
    }

    frame.m_mediaMetadata = *this;
    frame.m_captureDirection.m_azimuth = static_cast<float>(m_azimuth);
    frame.m_captureDirection.m_elevation = static_cast<float>(m_elevation);
    frame.m_captureDirection.m_roll = static_cast<float>(m_roll);
    frame.m_captureDirection.m_valid = true;
    if (m_captureDateTimeUtc.isValid()) {
        frame.m_captureDateTime = m_captureDateTimeUtc.addMSecs(qMax<qint64>(0, captureOffsetMs));
    }

    CameraPipelineObservationContext& context = frame.m_observationContext;
    context.m_dateTime = frame.m_captureDateTime;
    context.m_latitude = static_cast<float>(m_latitude);
    context.m_longitude = static_cast<float>(m_longitude);
    context.m_altitude = static_cast<float>(m_altitude);
    context.m_azimuth = static_cast<float>(m_azimuth);
    context.m_elevation = static_cast<float>(m_elevation);
    context.m_roll = static_cast<float>(m_roll);
    context.m_fov = static_cast<float>(m_fov);
    context.m_lensProjection = m_lensProjection;
    context.m_lensCenterOffsetX = m_lensCenterOffsetX;
    context.m_lensCenterOffsetY = m_lensCenterOffsetY;
    context.m_lensDistortionK1 = m_lensDistortionK1;
    context.m_lensMirror = m_lensMirror;
    context.m_valid = true;

    applyImageTransform(frame);
}

void CameraMediaMetadata::applyImageTransform(CameraPipelineFrame& frame) const
{
    if (!m_valid || !m_imageTransformValid) {
        return;
    }

    frame.m_imageTransform.m_opticalSize = m_opticalSize;
    frame.m_imageTransform.m_opticalToImage = m_opticalToImage;
    frame.m_imageTransform.m_enabled = true;
}

void CameraMediaMetadata::applyProjectionSettings(CameraSettings& settings) const
{
    if (!m_valid) {
        return;
    }

    settings.m_latitude = static_cast<float>(m_latitude);
    settings.m_longitude = static_cast<float>(m_longitude);
    settings.m_altitude = static_cast<float>(m_altitude);
    settings.m_fov = static_cast<float>(m_fov);
    settings.m_lensProjection = static_cast<CameraSettings::LensProjection>(m_lensProjection);
    settings.m_lensCenterOffsetX = m_lensCenterOffsetX;
    settings.m_lensCenterOffsetY = m_lensCenterOffsetY;
    settings.m_lensDistortionK1 = m_lensDistortionK1;
    settings.m_lensMirror = m_lensMirror;
}

bool CameraMediaMetadata::writeImage(
    const QString& fileName,
    const QImage& image,
    const CameraMediaMetadata& metadata,
    QString *errorMessage)
{
    QSaveFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
    {
        if (errorMessage) {
            *errorMessage = file.errorString();
        }
        return false;
    }

    const QByteArray format = QFileInfo(fileName).suffix().toLatin1();
    QImageWriter writer(&file, format);
    QImage outputImage = image;
    if (metadata.isValid()) {
        const QString metadataJson = QString::fromUtf8(metadata.toJson());
        // A decoded image can still contain the previous value. Replace it on
        // the image as well as the writer so image plugins cannot restore the
        // stale text after setText() has supplied the updated value.
        outputImage.setText(metadataKey(), metadataJson);
        writer.setText(metadataKey(), metadataJson);
    }
    if (!writer.write(outputImage))
    {
        if (errorMessage) {
            *errorMessage = writer.errorString();
        }
        file.cancelWriting();
        return false;
    }
    if (file.commit()) {
        return true;
    }
    if (errorMessage) {
        *errorMessage = file.errorString();
    }
    return false;
}
