///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#include <cmath>

#include <QCoreApplication>
#include <QImage>
#include <QImageReader>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include "cameramediametadata.h"
#include "camerapipelineframe.h"

namespace {

bool nearlyEqual(double lhs, double rhs)
{
    return std::abs(lhs - rhs) < 1.0e-6;
}

QByteArray testMetadataJson(int lensProjection = 1)
{
    QJsonObject site;
    site.insert(QStringLiteral("latitude"), 51.5);
    site.insert(QStringLiteral("longitude"), -0.125);
    site.insert(QStringLiteral("altitude"), 72.0);

    QJsonObject direction;
    direction.insert(QStringLiteral("azimuth"), 123.5);
    direction.insert(QStringLiteral("elevation"), 47.25);
    direction.insert(QStringLiteral("roll"), -12.75);

    QJsonObject projection;
    projection.insert(QStringLiteral("fov"), 1.29);
    projection.insert(QStringLiteral("type"), lensProjection);
    projection.insert(QStringLiteral("centerOffsetX"), 4.5);
    projection.insert(QStringLiteral("centerOffsetY"), -3.25);
    projection.insert(QStringLiteral("distortionK1"), 0.0125);
    projection.insert(QStringLiteral("mirror"), true);

    QJsonObject transform;
    transform.insert(QStringLiteral("opticalWidth"), 3552);
    transform.insert(QStringLiteral("opticalHeight"), 3552);
    transform.insert(QStringLiteral("m11"), 0.5);
    transform.insert(QStringLiteral("m12"), 0.0);
    transform.insert(QStringLiteral("m21"), 0.0);
    transform.insert(QStringLiteral("m22"), 0.5);
    transform.insert(QStringLiteral("dx"), 144.0);
    transform.insert(QStringLiteral("dy"), 0.0);

    QJsonObject root;
    root.insert(QStringLiteral("version"), CameraMediaMetadata::CurrentVersion);
    root.insert(QStringLiteral("captureDateTimeUtc"), QStringLiteral("2026-07-29T12:34:56.789Z"));
    root.insert(QStringLiteral("site"), site);
    root.insert(QStringLiteral("direction"), direction);
    root.insert(QStringLiteral("projection"), projection);
    root.insert(QStringLiteral("imageTransform"), transform);
    return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

bool checkFrame(const CameraMediaMetadata& metadata, QString& error)
{
    CameraPipelineFrame frame;
    metadata.applyToFrame(frame, 1250);
    if (!frame.m_captureDirection.m_valid
        || !nearlyEqual(frame.m_captureDirection.m_azimuth, 123.5)
        || !nearlyEqual(frame.m_captureDirection.m_elevation, 47.25)
        || !nearlyEqual(frame.m_captureDirection.m_roll, -12.75))
    {
        error = QStringLiteral("Direction was not restored");
        return false;
    }
    if (frame.m_captureDateTime.toUTC().toString(Qt::ISODateWithMs)
        != QStringLiteral("2026-07-29T12:34:58.039Z"))
    {
        error = QStringLiteral("Capture date/time offset was not restored");
        return false;
    }
    if (!frame.m_observationContext.m_valid
        || frame.m_observationContext.m_dateTime != frame.m_captureDateTime
        || !nearlyEqual(frame.m_observationContext.m_latitude, 51.5)
        || !nearlyEqual(frame.m_observationContext.m_longitude, -0.125)
        || !nearlyEqual(frame.m_observationContext.m_altitude, 72.0)
        || !nearlyEqual(frame.m_observationContext.m_azimuth, 123.5)
        || !nearlyEqual(frame.m_observationContext.m_elevation, 47.25)
        || !nearlyEqual(frame.m_observationContext.m_roll, -12.75)
        || !nearlyEqual(frame.m_observationContext.m_fov, 1.29)
        || (frame.m_observationContext.m_lensProjection != 1)
        || !nearlyEqual(frame.m_observationContext.m_lensCenterOffsetX, 4.5)
        || !nearlyEqual(frame.m_observationContext.m_lensCenterOffsetY, -3.25)
        || !nearlyEqual(frame.m_observationContext.m_lensDistortionK1, 0.0125)
        || !frame.m_observationContext.m_lensMirror)
    {
        error = QStringLiteral("Observation context was not restored");
        return false;
    }
    if (!frame.m_imageTransform.isValid()
        || frame.m_imageTransform.m_opticalSize != QSize(3552, 3552)
        || !nearlyEqual(frame.m_imageTransform.m_opticalToImage.m11(), 0.5)
        || !nearlyEqual(frame.m_imageTransform.m_opticalToImage.dx(), 144.0))
    {
        error = QStringLiteral("Image transform was not restored");
        return false;
    }
    return true;
}

bool checkImageRoundTrip(
    const QString& fileName,
    const CameraMediaMetadata& metadata,
    QString& error)
{
    QImage image(32, 24, QImage::Format_RGB888);
    image.fill(Qt::darkBlue);
    if (!CameraMediaMetadata::writeImage(fileName, image, metadata, &error)) {
        return false;
    }

    QImageReader reader(fileName);
    const QImage decoded = reader.read();
    if (decoded.isNull())
    {
        error = reader.errorString();
        return false;
    }

    const CameraMediaMetadata decodedMetadata = CameraMediaMetadata::fromImage(decoded, &error);
    if (!decodedMetadata.isValid())
    {
        if (error.isEmpty()) {
            error = QStringLiteral("Embedded metadata was not returned by QImageReader");
        }
        return false;
    }
    return checkFrame(decodedMetadata, error);
}

bool checkImageMetadataReplacement(
    const QString& fileName,
    const CameraMediaMetadata& replacementMetadata,
    int expectedLensProjection,
    QString& error)
{
    QImage image;
    QString readerError;
    {
        QImageReader reader(fileName);
        image = reader.read();
        readerError = reader.errorString();
    }
    if (image.isNull())
    {
        error = readerError;
        return false;
    }
    if (!CameraMediaMetadata::writeImage(fileName, image, replacementMetadata, &error)) {
        return false;
    }

    QImageReader replacementReader(fileName);
    const QImage replacementImage = replacementReader.read();
    if (replacementImage.isNull())
    {
        error = replacementReader.errorString();
        return false;
    }
    const CameraMediaMetadata decodedMetadata = CameraMediaMetadata::fromImage(replacementImage, &error);
    if (!decodedMetadata.isValid())
    {
        if (error.isEmpty()) {
            error = QStringLiteral("Replacement metadata was not returned by QImageReader");
        }
        return false;
    }
    if (decodedMetadata.lensProjection() != expectedLensProjection)
    {
        error = QStringLiteral("Lens projection metadata was not replaced: expected %1, got %2")
            .arg(expectedLensProjection)
            .arg(decodedMetadata.lensProjection());
        return false;
    }
    return true;
}

}

int main(int argc, char **argv)
{
    QCoreApplication application(argc, argv);
    QString error;
    const CameraMediaMetadata metadata = CameraMediaMetadata::fromJson(testMetadataJson(), &error);
    if (!metadata.isValid() || !checkFrame(metadata, error))
    {
        qCritical().noquote() << "Camera media metadata JSON test failed:" << error;
        return 1;
    }

    QTemporaryDir directory;
    if (!directory.isValid())
    {
        qCritical() << "Camera media metadata test could not create a temporary directory";
        return 1;
    }

    for (const QString& suffix : {QStringLiteral("png"), QStringLiteral("jpg")})
    {
        error.clear();
        const QString fileName = directory.filePath(QStringLiteral("metadata.%1").arg(suffix));
        if (!checkImageRoundTrip(fileName, metadata, error))
        {
            qCritical().noquote() << "Camera media metadata" << suffix
                                  << "round-trip test failed:" << error;
            return 1;
        }

        error.clear();
        const CameraMediaMetadata replacementMetadata = CameraMediaMetadata::fromJson(
            testMetadataJson(2),
            &error);
        if (!replacementMetadata.isValid()
            || !checkImageMetadataReplacement(fileName, replacementMetadata, 2, error))
        {
            qCritical().noquote() << "Camera media metadata" << suffix
                                  << "replacement test failed:" << error;
            return 1;
        }
    }

    qInfo() << "Camera media metadata tests passed";
    return 0;
}
