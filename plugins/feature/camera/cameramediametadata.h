///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_FEATURE_CAMERA_MEDIA_METADATA_H_
#define INCLUDE_FEATURE_CAMERA_MEDIA_METADATA_H_

#include <QByteArray>
#include <QDateTime>
#include <QImage>
#include <QSize>
#include <QString>
#include <QTransform>

struct CameraPipelineFrame;
struct CameraSettings;

/**
 * \brief Versioned capture geometry stored in camera image and video files.
 *
 * PNG/JPEG files carry the compact JSON representation as image text. MP4 files
 * carry the same JSON as a file-level libavformat metadata value. The first-frame
 * values therefore describe the entire MP4 recording; per-frame telemetry can be
 * added later without changing the schema used for still images.
 */
class CameraMediaMetadata
{
public:
    static constexpr int CurrentVersion = 1;

    [[nodiscard]] static QString metadataKey();
    [[nodiscard]] static CameraMediaMetadata fromFrame(
        const CameraSettings& settings,
        const CameraPipelineFrame& frame);
    [[nodiscard]] static CameraMediaMetadata fromJson(
        const QByteArray& json,
        QString *errorMessage = nullptr);
    [[nodiscard]] static CameraMediaMetadata fromImage(
        const QImage& image,
        QString *errorMessage = nullptr);

    [[nodiscard]] bool isValid() const { return m_valid; }

    // Read-only access to the stored capture geometry. applyToFrame/applyProjectionSettings
    // cover the normal pipeline use, but consumers that build their own settings (e.g. the
    // plate-solver test harness, which fills unspecified CSV columns from the image) need the
    // individual values, including the pointing and capture time those helpers route through
    // CameraPipelineFrame.
    [[nodiscard]] const QDateTime& captureDateTimeUtc() const { return m_captureDateTimeUtc; }
    [[nodiscard]] double latitude() const { return m_latitude; }
    [[nodiscard]] double longitude() const { return m_longitude; }
    [[nodiscard]] double altitude() const { return m_altitude; }
    [[nodiscard]] double azimuth() const { return m_azimuth; }
    [[nodiscard]] double elevation() const { return m_elevation; }
    [[nodiscard]] double roll() const { return m_roll; }
    [[nodiscard]] double fov() const { return m_fov; }
    [[nodiscard]] int lensProjection() const { return m_lensProjection; }
    [[nodiscard]] double lensCenterOffsetX() const { return m_lensCenterOffsetX; }
    [[nodiscard]] double lensCenterOffsetY() const { return m_lensCenterOffsetY; }
    [[nodiscard]] double lensDistortionK1() const { return m_lensDistortionK1; }
    [[nodiscard]] bool lensMirror() const { return m_lensMirror; }

    [[nodiscard]] QByteArray toJson() const;
    void applyToFrame(CameraPipelineFrame& frame, qint64 captureOffsetMs = 0) const;
    void applyImageTransform(CameraPipelineFrame& frame) const;
    void applyProjectionSettings(CameraSettings& settings) const;

    [[nodiscard]] static bool writeImage(
        const QString& fileName,
        const QImage& image,
        const CameraMediaMetadata& metadata,
        QString *errorMessage = nullptr);

private:
    bool m_valid = false;
    QDateTime m_captureDateTimeUtc;
    double m_latitude = 0.0;
    double m_longitude = 0.0;
    double m_altitude = 0.0;
    double m_azimuth = 0.0;
    double m_elevation = 0.0;
    double m_roll = 0.0;
    double m_fov = 0.0;
    int m_lensProjection = 0;
    double m_lensCenterOffsetX = 0.0;
    double m_lensCenterOffsetY = 0.0;
    double m_lensDistortionK1 = 0.0;
    bool m_lensMirror = false;
    QSize m_opticalSize;
    QTransform m_opticalToImage;
    bool m_imageTransformValid = false;
};

#endif // INCLUDE_FEATURE_CAMERA_MEDIA_METADATA_H_
