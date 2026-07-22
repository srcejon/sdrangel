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

class MeteorMapGeometry
{
public:
    struct Coordinate
    {
        double m_latitude;
        double m_longitude;
        double m_altitudeM;
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

private:
    static Coordinate destination(
        double latitude,
        double longitude,
        double bearingDegrees,
        double groundDistanceM,
        double altitudeM);
};

#endif // INCLUDE_METEORMAPGEOMETRY_H
