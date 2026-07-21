///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2024 Jon Beniston, M7RCE <jon@beniston.com>                        //
//                                                                                   //
// This program is free software; you can redistribute it and/or modify             //
// it under the terms of the GNU General Public License as published by             //
// the Free Software Foundation as version 3 of the License, or                     //
// (at your option) any later version.                                              //
//                                                                                   //
// This program is distributed in the hope that it will be useful,                  //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                   //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                    //
// GNU General Public License V3 for more details.                                  //
//                                                                                   //
// You should have received a copy of the GNU General Public License                //
// along with this program. If not, see <http://www.gnu.org/licenses/>.             //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_CAMERASKYPROJECTOR_H
#define INCLUDE_CAMERASKYPROJECTOR_H

// Shared sky-to-image fisheye projector. Given a camera pose (azimuth/elevation/roll),
// field of view and lens model from CameraSettings, projects a sky direction expressed
// as azimuth/elevation onto an image pixel. Used both by the post-processor's sky
// overlays and by the cloud detector's sun/moon exclusion mask, so the two agree on
// exactly where a sky object lands on the frame.

#include <algorithm>
#include <cmath>

#include <QPointF>
#include <QSize>

#include "camerasettings.h"
#include "camerapipelineframe.h"

struct SkyVector
{
    double x;
    double y;
    double z;
};

inline double skyDegToRad(double value)
{
    static constexpr double kPi = 3.14159265358979323846;
    return value * kPi / 180.0;
}

inline double skyRadToDeg(double value)
{
    static constexpr double kPi = 3.14159265358979323846;
    return value * 180.0 / kPi;
}

inline double skyDot(const SkyVector& lhs, const SkyVector& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

inline SkyVector skyCross(const SkyVector& lhs, const SkyVector& rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x
    };
}

inline double skyLength(const SkyVector& vector)
{
    return std::sqrt(skyDot(vector, vector));
}

inline SkyVector skyNormalize(const SkyVector& vector)
{
    const double vectorLength = skyLength(vector);
    if (vectorLength <= 0.0) {
        return {0.0, 0.0, 0.0};
    }

    return {
        vector.x / vectorLength,
        vector.y / vectorLength,
        vector.z / vectorLength
    };
}

inline SkyVector skyRotateAroundAxis(const SkyVector& vector, const SkyVector& axis, double angleRadians)
{
    const double cosAngle = std::cos(angleRadians);
    const double sinAngle = std::sin(angleRadians);
    const SkyVector axisCrossVector = skyCross(axis, vector);
    const double axisDotVector = skyDot(axis, vector);

    return {
        vector.x * cosAngle + axisCrossVector.x * sinAngle + axis.x * axisDotVector * (1.0 - cosAngle),
        vector.y * cosAngle + axisCrossVector.y * sinAngle + axis.y * axisDotVector * (1.0 - cosAngle),
        vector.z * cosAngle + axisCrossVector.z * sinAngle + axis.z * axisDotVector * (1.0 - cosAngle)
    };
}

inline SkyVector skyVectorFromAltAz(double azimuthDegrees, double elevationDegrees)
{
    const double azimuth = skyDegToRad(azimuthDegrees);
    const double elevation = skyDegToRad(elevationDegrees);
    const double cosElevation = std::cos(elevation);

    return {
        cosElevation * std::sin(azimuth),
        cosElevation * std::cos(azimuth),
        std::sin(elevation)
    };
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
    // Handedness: the image is horizontally mirrored relative to the sky (m_lensMirror — up-looking
    // all-sky camera or star diagonal). The solved pose lives in the mirrored frame, so overlay
    // projection onto the displayed (original) image must reflect pixel x about the image centre.
    // Mirrors the solver's SkyProjector::mirrorX semantics exactly.
    bool mirrorX = false;
    CameraPipelineImageTransform imageTransform;

    static SkyProjector create(const CameraSettings& settings, const QSize& imageSize, const CameraPipelineImageTransform& transform = CameraPipelineImageTransform())
    {
        SkyProjector projector;
        projector.imageTransform = transform;
        const QSize projectionSize = transform.opticalSize(imageSize);
        projector.width = projectionSize.width();
        projector.height = projectionSize.height();
        projector.lensProjection = settings.m_lensProjection;

        if (projector.width <= 0 || projector.height <= 0 || settings.m_fov <= 0.0f) {
            return projector;
        }

        const double azimuth = skyDegToRad(settings.m_azimuth);
        projector.center = skyNormalize(skyVectorFromAltAz(settings.m_azimuth, settings.m_elevation));
        projector.right = skyNormalize({std::cos(azimuth), -std::sin(azimuth), 0.0});
        projector.up = skyNormalize(skyCross(projector.right, projector.center));
        if (skyLength(projector.right) <= 0.0 || skyLength(projector.up) <= 0.0) {
            return projector;
        }

        const double rollRadians = skyDegToRad(settings.m_roll);
        if (std::fabs(rollRadians) > 1e-9)
        {
            projector.right = skyNormalize(skyRotateAroundAxis(projector.right, projector.center, rollRadians));
            projector.up = skyNormalize(skyRotateAroundAxis(projector.up, projector.center, rollRadians));
        }

        const double halfHorizontalFov = skyDegToRad(settings.m_fov) * 0.5;
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
        projector.mirrorX = settings.m_lensMirror;
        projector.valid = projector.verticalScale > 0.0;
        return projector;
    }

    bool projectAltAz(double azimuthDegrees, double elevationDegrees, QPointF& point) const
    {
        if (!valid) {
            return false;
        }

        const SkyVector vector = skyVectorFromAltAz(azimuthDegrees, elevationDegrees);
        const double depth = skyDot(vector, center);
        if (depth <= 0.0) {
            return false;
        }

        const double planeX = skyDot(vector, right);
        const double planeY = skyDot(vector, up);
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

        QPointF opticalPoint(
            principalPointX + normalizedX * 0.5 * static_cast<double>(width),
            principalPointY - normalizedY * 0.5 * static_cast<double>(height));
        if (mirrorX) {
            opticalPoint.setX(static_cast<double>(width - 1) - opticalPoint.x());
        }
        point = imageTransform.mapOpticalToImage(opticalPoint);
        return true;
    }

    // Exact inverse of projectAltAz: image pixel to sky azimuth/elevation. Each step
    // mirrors the forward path in reverse order (image transform, mirror, principal
    // point/scales, distortion, projection radius, camera basis). The k1 distortion is
    // inverted iteratively; a handful of fixed-point steps converge for the small k1
    // magnitudes lens calibration produces.
    bool unprojectToAltAz(const QPointF& imagePoint, double& azimuthDegrees, double& elevationDegrees) const
    {
        if (!valid) {
            return false;
        }

        QPointF opticalPoint = imageTransform.mapImageToOptical(imagePoint);
        if (mirrorX) {
            opticalPoint.setX(static_cast<double>(width - 1) - opticalPoint.x());
        }
        const double normalizedX = (opticalPoint.x() - principalPointX) / (0.5 * static_cast<double>(width));
        const double normalizedY = (principalPointY - opticalPoint.y()) / (0.5 * static_cast<double>(height));
        double projectedX = normalizedX * horizontalScale;
        double projectedY = normalizedY * verticalScale;

        if (std::fabs(distortionK1) > 1e-9)
        {
            // Forward: distorted = undistorted * (1 + k1 * |undistorted|^2). Fixed-point
            // iteration from the distorted radius recovers the undistorted coordinates.
            const double distortedX = projectedX;
            const double distortedY = projectedY;
            double undistortedX = distortedX;
            double undistortedY = distortedY;
            for (int iteration = 0; iteration < 6; ++iteration)
            {
                const double radiusSquared = undistortedX * undistortedX + undistortedY * undistortedY;
                const double distortionScale = std::max(0.1, 1.0 + distortionK1 * radiusSquared);
                undistortedX = distortedX / distortionScale;
                undistortedY = distortedY / distortionScale;
            }
            projectedX = undistortedX;
            projectedY = undistortedY;
        }

        const double projectionRadius = std::hypot(projectedX, projectedY);
        double theta = 0.0;
        switch (lensProjection)
        {
        case CameraSettings::LensProjectionEquidistant:
            theta = projectionRadius * halfHorizontalFov;
            break;
        case CameraSettings::LensProjectionEquisolid:
        {
            const double halfSine = projectionRadius * std::sin(halfHorizontalFov * 0.5);
            if (halfSine > 1.0) {
                return false;
            }
            theta = 2.0 * std::asin(halfSine);
            break;
        }
        case CameraSettings::LensProjectionRectilinear:
        default:
            theta = std::atan(projectionRadius * std::tan(halfHorizontalFov));
            break;
        }
        static constexpr double kPi = 3.14159265358979323846;
        if (!std::isfinite(theta) || (theta >= kPi)) {
            return false;
        }

        const double phi = std::atan2(projectedY, projectedX);
        const double sinTheta = std::sin(theta);
        const SkyVector vector = {
            std::cos(theta) * center.x + sinTheta * (std::cos(phi) * right.x + std::sin(phi) * up.x),
            std::cos(theta) * center.y + sinTheta * (std::cos(phi) * right.y + std::sin(phi) * up.y),
            std::cos(theta) * center.z + sinTheta * (std::cos(phi) * right.z + std::sin(phi) * up.z)
        };
        elevationDegrees = skyRadToDeg(std::asin(std::clamp(vector.z, -1.0, 1.0)));
        azimuthDegrees = skyRadToDeg(std::atan2(vector.x, vector.y));
        if (azimuthDegrees < 0.0) {
            azimuthDegrees += 360.0;
        }
        return true;
    }
};

#endif // INCLUDE_CAMERASKYPROJECTOR_H
