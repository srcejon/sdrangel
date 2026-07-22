///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#include "meteormapgeometry.h"

#include <algorithm>
#include <cmath>

namespace {
    constexpr double EarthRadiusM = 6371008.8;
    constexpr double Pi = 3.14159265358979323846;

    double degreesToRadians(double degrees)
    {
        return degrees * Pi / 180.0;
    }

    double radiansToDegrees(double radians)
    {
        return radians * 180.0 / Pi;
    }

    double normalizedLongitude(double longitude)
    {
        return std::fmod(longitude + 540.0, 360.0) - 180.0;
    }
}

double MeteorMapGeometry::groundDistanceAtAltitude(double elevationDegrees, double altitudeM)
{
    if (!std::isfinite(elevationDegrees)
        || !std::isfinite(altitudeM)
        || (elevationDegrees < 0.0)
        || (elevationDegrees > 90.0)
        || (altitudeM <= 0.0))
    {
        return 0.0;
    }

    const double elevation = degreesToRadians(elevationDegrees);
    const double targetRadius = EarthRadiusM + altitudeM;
    const double radialComponent = EarthRadiusM * std::sin(elevation);
    const double slantDistance = -radialComponent
        + std::sqrt(radialComponent * radialComponent
            + targetRadius * targetRadius
            - EarthRadiusM * EarthRadiusM);
    const double centralAngle = std::atan2(
        slantDistance * std::cos(elevation),
        EarthRadiusM + slantDistance * std::sin(elevation));

    return EarthRadiusM * centralAngle;
}

std::vector<MeteorMapGeometry::Coordinate> MeteorMapGeometry::beamFootprint(
    double latitude,
    double longitude,
    double azimuthDegrees,
    double elevationDegrees,
    double horizontalBeamwidthDegrees,
    double verticalHPBWDegrees,
    double maxAltitudeM)
{
    std::vector<Coordinate> coordinates;

    if (!std::isfinite(latitude)
        || !std::isfinite(longitude)
        || !std::isfinite(azimuthDegrees)
        || !std::isfinite(elevationDegrees)
        || !std::isfinite(horizontalBeamwidthDegrees)
        || !std::isfinite(verticalHPBWDegrees)
        || !std::isfinite(maxAltitudeM)
        || (latitude < -90.0)
        || (latitude > 90.0)
        || (longitude < -180.0)
        || (longitude > 180.0)
        || (horizontalBeamwidthDegrees <= 0.0)
        || (verticalHPBWDegrees <= 0.0)
        || (maxAltitudeM <= 0.0))
    {
        return coordinates;
    }

    const double upperElevation = std::min(90.0, elevationDegrees + verticalHPBWDegrees / 2.0);

    if (upperElevation <= 0.0) {
        return coordinates;
    }

    const double lowerElevation = std::clamp(elevationDegrees - verticalHPBWDegrees / 2.0, 0.0, 90.0);
    const double outerDistanceM = groundDistanceAtAltitude(lowerElevation, maxAltitudeM);
    const double innerDistanceM = groundDistanceAtAltitude(std::max(0.0, upperElevation), maxAltitudeM);
    const double coverage = std::min(360.0, horizontalBeamwidthDegrees);
    const int segments = std::clamp((int) std::ceil(coverage / 5.0), 4, 144);
    const double firstBearing = azimuthDegrees - coverage / 2.0;
    const double bearingStep = coverage / segments;

    coordinates.reserve((segments + 1) * 2);

    for (int i = 0; i <= segments; ++i) {
        coordinates.push_back(destination(
            latitude,
            longitude,
            firstBearing + i * bearingStep,
            outerDistanceM,
            maxAltitudeM));
    }

    for (int i = segments; i >= 0; --i) {
        coordinates.push_back(destination(
            latitude,
            longitude,
            firstBearing + i * bearingStep,
            innerDistanceM,
            maxAltitudeM));
    }

    return coordinates;
}

MeteorMapGeometry::Coordinate MeteorMapGeometry::destination(
    double latitude,
    double longitude,
    double bearingDegrees,
    double groundDistanceM,
    double altitudeM)
{
    const double latitudeRadians = degreesToRadians(latitude);
    const double longitudeRadians = degreesToRadians(longitude);
    const double bearingRadians = degreesToRadians(bearingDegrees);
    const double centralAngle = groundDistanceM / EarthRadiusM;
    const double destinationLatitude = std::asin(
        std::sin(latitudeRadians) * std::cos(centralAngle)
        + std::cos(latitudeRadians) * std::sin(centralAngle) * std::cos(bearingRadians));
    const double destinationLongitude = longitudeRadians + std::atan2(
        std::sin(bearingRadians) * std::sin(centralAngle) * std::cos(latitudeRadians),
        std::cos(centralAngle) - std::sin(latitudeRadians) * std::sin(destinationLatitude));

    return {
        radiansToDegrees(destinationLatitude),
        normalizedLongitude(radiansToDegrees(destinationLongitude)),
        altitudeM
    };
}
