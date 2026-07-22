///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_METEORMAPGEOMETRY_H
#define INCLUDE_METEORMAPGEOMETRY_H

#include <vector>
#include <cstdint>

class MeteorMapGeometry
{
public:
    struct Coordinate
    {
        double m_latitude;
        double m_longitude;
        double m_altitudeM;
    };

    struct BeamDefinition
    {
        double m_latitude;
        double m_longitude;
        double m_altitudeM;
        double m_azimuthDegrees;
        double m_elevationDegrees;
        double m_horizontalBeamwidthDegrees;
        double m_verticalBeamwidthDegrees;
        double m_maxAltitudeM;
    };

    struct Mesh
    {
        std::vector<Coordinate> m_vertices;
        std::vector<std::uint32_t> m_triangleIndices;
        std::vector<Coordinate> m_footprint;

        bool isValid() const;
    };

    static double groundDistanceAtAltitude(double elevationDegrees, double altitudeM);
    static std::vector<Coordinate> beamFootprint(
        double latitude,
        double longitude,
        double azimuthDegrees,
        double elevationDegrees,
        double horizontalBeamwidthDegrees,
        double verticalHPBWDegrees,
        double maxAltitudeM);
    static Mesh beamVolume(const BeamDefinition& beam, int edgeSegments = 8);
    static Mesh beamIntersection(
        const BeamDefinition& first,
        const BeamDefinition& second,
        int edgeSegments = 8);

private:
    static Coordinate destination(
        double latitude,
        double longitude,
        double bearingDegrees,
        double groundDistanceM,
        double altitudeM);
};

#endif // INCLUDE_METEORMAPGEOMETRY_H
