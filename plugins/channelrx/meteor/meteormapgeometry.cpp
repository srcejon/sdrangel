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
#include <array>
#include <cmath>
#include <limits>

namespace {
    constexpr double EarthRadiusM = 6371008.8;
    constexpr double WGS84SemiMajorM = 6378137.0;
    constexpr double WGS84SemiMinorM = 6356752.314245;
    constexpr double WGS84EccentricitySquared = 6.6943799901413165e-3;
    constexpr double Pi = 3.14159265358979323846;

    struct Vector3
    {
        double x;
        double y;
        double z;

        Vector3 operator+(const Vector3& other) const { return {x + other.x, y + other.y, z + other.z}; }
        Vector3 operator-(const Vector3& other) const { return {x - other.x, y - other.y, z - other.z}; }
        Vector3 operator*(double scalar) const { return {x * scalar, y * scalar, z * scalar}; }
        Vector3 operator/(double scalar) const { return {x / scalar, y / scalar, z / scalar}; }
    };

    struct Plane
    {
        Vector3 origin;
        Vector3 inwardNormal;
    };

    using Face = std::vector<Vector3>;

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

    double dot(const Vector3& a, const Vector3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    Vector3 cross(const Vector3& a, const Vector3& b)
    {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }

    double length(const Vector3& value)
    {
        return std::sqrt(dot(value, value));
    }

    Vector3 normalized(const Vector3& value)
    {
        const double magnitude = length(value);
        return magnitude > 0.0 ? value / magnitude : Vector3 {0.0, 0.0, 0.0};
    }

    Vector3 geodeticToECEF(const MeteorMapGeometry::Coordinate& coordinate)
    {
        const double latitude = degreesToRadians(coordinate.m_latitude);
        const double longitude = degreesToRadians(coordinate.m_longitude);
        const double sinLatitude = std::sin(latitude);
        const double cosLatitude = std::cos(latitude);
        const double radius = WGS84SemiMajorM
            / std::sqrt(1.0 - WGS84EccentricitySquared * sinLatitude * sinLatitude);

        return {
            (radius + coordinate.m_altitudeM) * cosLatitude * std::cos(longitude),
            (radius + coordinate.m_altitudeM) * cosLatitude * std::sin(longitude),
            (radius * (1.0 - WGS84EccentricitySquared) + coordinate.m_altitudeM) * sinLatitude
        };
    }

    MeteorMapGeometry::Coordinate ecefToGeodetic(const Vector3& value)
    {
        const double longitude = std::atan2(value.y, value.x);
        const double horizontal = std::hypot(value.x, value.y);
        double latitude = std::atan2(value.z, horizontal * (1.0 - WGS84EccentricitySquared));
        double altitude = 0.0;

        for (int i = 0; i < 8; ++i)
        {
            const double sinLatitude = std::sin(latitude);
            const double radius = WGS84SemiMajorM
                / std::sqrt(1.0 - WGS84EccentricitySquared * sinLatitude * sinLatitude);
            altitude = horizontal / std::max(1.0e-12, std::cos(latitude)) - radius;
            latitude = std::atan2(
                value.z,
                horizontal * (1.0 - WGS84EccentricitySquared * radius / (radius + altitude)));
        }

        return {
            radiansToDegrees(latitude),
            normalizedLongitude(radiansToDegrees(longitude)),
            altitude
        };
    }

    bool validBeam(const MeteorMapGeometry::BeamDefinition& beam)
    {
        return std::isfinite(beam.m_latitude)
            && std::isfinite(beam.m_longitude)
            && std::isfinite(beam.m_altitudeM)
            && std::isfinite(beam.m_azimuthDegrees)
            && std::isfinite(beam.m_elevationDegrees)
            && std::isfinite(beam.m_horizontalBeamwidthDegrees)
            && std::isfinite(beam.m_verticalBeamwidthDegrees)
            && std::isfinite(beam.m_maxAltitudeM)
            && (beam.m_latitude >= -90.0)
            && (beam.m_latitude <= 90.0)
            && (beam.m_longitude >= -180.0)
            && (beam.m_longitude <= 180.0)
            && (beam.m_elevationDegrees >= 0.0)
            && (beam.m_elevationDegrees <= 90.0)
            && (beam.m_horizontalBeamwidthDegrees > 0.0)
            && (beam.m_verticalBeamwidthDegrees > 0.0)
            && (beam.m_maxAltitudeM > beam.m_altitudeM);
    }

    void beamFrame(
        const MeteorMapGeometry::BeamDefinition& beam,
        Vector3& forward,
        Vector3& right,
        Vector3& vertical)
    {
        const double latitude = degreesToRadians(beam.m_latitude);
        const double longitude = degreesToRadians(beam.m_longitude);
        const double azimuth = degreesToRadians(beam.m_azimuthDegrees);
        const double elevation = degreesToRadians(beam.m_elevationDegrees);
        const Vector3 east {-std::sin(longitude), std::cos(longitude), 0.0};
        const Vector3 north {
            -std::sin(latitude) * std::cos(longitude),
            -std::sin(latitude) * std::sin(longitude),
            std::cos(latitude)
        };
        const Vector3 up {
            std::cos(latitude) * std::cos(longitude),
            std::cos(latitude) * std::sin(longitude),
            std::sin(latitude)
        };

        forward = normalized(
            east * (std::sin(azimuth) * std::cos(elevation))
            + north * (std::cos(azimuth) * std::cos(elevation))
            + up * std::sin(elevation));
        right = normalized(east * std::cos(azimuth) - north * std::sin(azimuth));
        vertical = normalized(cross(right, forward));
    }

    std::array<Plane, 4> beamPlanes(const MeteorMapGeometry::BeamDefinition& beam)
    {
        Vector3 forward;
        Vector3 right;
        Vector3 vertical;
        beamFrame(beam, forward, right, vertical);
        const Vector3 origin = geodeticToECEF({beam.m_latitude, beam.m_longitude, beam.m_altitudeM});
        const double horizontalHalfAngle = degreesToRadians(
            std::clamp(beam.m_horizontalBeamwidthDegrees / 2.0, 0.01, 89.9));
        const double verticalHalfAngle = degreesToRadians(
            std::clamp(beam.m_verticalBeamwidthDegrees / 2.0, 0.01, 89.9));
        const double elevation = degreesToRadians(beam.m_elevationDegrees);
        const double lowerVerticalOffset = std::max(-verticalHalfAngle, -elevation);
        const double upperVerticalOffset = std::min(verticalHalfAngle, Pi / 2.0 - elevation);

        return {{
            {origin, normalized(forward * std::sin(horizontalHalfAngle) + right * std::cos(horizontalHalfAngle))},
            {origin, normalized(forward * std::sin(horizontalHalfAngle) - right * std::cos(horizontalHalfAngle))},
            {origin, normalized(forward * -std::sin(lowerVerticalOffset) + vertical * std::cos(lowerVerticalOffset))},
            {origin, normalized(forward * std::sin(upperVerticalOffset) - vertical * std::cos(upperVerticalOffset))}
        }};
    }

    Vector3 rayAtOffsets(
        const Vector3& forward,
        const Vector3& right,
        const Vector3& vertical,
        double horizontalOffset,
        double verticalOffset)
    {
        return normalized(
            forward
            + right * std::tan(horizontalOffset)
            + vertical * std::tan(verticalOffset));
    }

    bool rayToAltitude(
        const Vector3& origin,
        const Vector3& direction,
        double altitudeM,
        Vector3& intersection)
    {
        const double a = WGS84SemiMajorM + altitudeM;
        const double b = WGS84SemiMinorM + altitudeM;
        const double aa = (direction.x * direction.x + direction.y * direction.y) / (a * a)
            + direction.z * direction.z / (b * b);
        const double bb = 2.0 * ((origin.x * direction.x + origin.y * direction.y) / (a * a)
            + origin.z * direction.z / (b * b));
        const double cc = (origin.x * origin.x + origin.y * origin.y) / (a * a)
            + origin.z * origin.z / (b * b) - 1.0;
        const double discriminant = bb * bb - 4.0 * aa * cc;

        if (discriminant < 0.0) {
            return false;
        }

        const double root = std::sqrt(discriminant);
        const double first = (-bb - root) / (2.0 * aa);
        const double second = (-bb + root) / (2.0 * aa);
        const double distance = std::max(first, second);

        if (!(distance > 0.0) || !std::isfinite(distance)) {
            return false;
        }

        intersection = origin + direction * distance;
        return true;
    }

    void appendTriangle(
        MeteorMapGeometry::Mesh& mesh,
        std::uint32_t first,
        std::uint32_t second,
        std::uint32_t third,
        const Vector3& interior,
        const std::vector<Vector3>& vertices)
    {
        const Vector3 faceCenter = (vertices[first] + vertices[second] + vertices[third]) / 3.0;
        const Vector3 normal = cross(vertices[second] - vertices[first], vertices[third] - vertices[first]);

        if (dot(normal, faceCenter - interior) < 0.0) {
            std::swap(second, third);
        }

        mesh.m_triangleIndices.push_back(first);
        mesh.m_triangleIndices.push_back(second);
        mesh.m_triangleIndices.push_back(third);
    }

    Face clipFace(const Face& face, const Plane& plane, std::vector<Vector3>& intersections)
    {
        Face output;

        if (face.empty()) {
            return output;
        }

        Vector3 previous = face.back();
        double previousDistance = dot(previous - plane.origin, plane.inwardNormal);
        bool previousInside = previousDistance >= -0.01;

        for (const Vector3& current : face)
        {
            const double currentDistance = dot(current - plane.origin, plane.inwardNormal);
            const bool currentInside = currentDistance >= -0.01;

            if (currentInside != previousInside)
            {
                const double ratio = previousDistance / (previousDistance - currentDistance);
                const Vector3 intersection = previous + (current - previous) * ratio;
                output.push_back(intersection);
                intersections.push_back(intersection);
            }

            if (currentInside) {
                output.push_back(current);
            }

            previous = current;
            previousDistance = currentDistance;
            previousInside = currentInside;
        }

        return output;
    }

    void appendUnique(std::vector<Vector3>& values, const Vector3& value, double toleranceM = 0.25)
    {
        for (const Vector3& existing : values)
        {
            if (length(existing - value) <= toleranceM) {
                return;
            }
        }

        values.push_back(value);
    }

    std::vector<Face> clipPolyhedron(std::vector<Face> faces, const Plane& plane)
    {
        std::vector<Face> clippedFaces;
        std::vector<Vector3> intersections;

        for (const Face& face : faces)
        {
            Face clipped = clipFace(face, plane, intersections);
            if (clipped.size() >= 3) {
                clippedFaces.push_back(std::move(clipped));
            }
        }

        std::vector<Vector3> cap;
        for (const Vector3& intersection : intersections) {
            appendUnique(cap, intersection);
        }

        if (cap.size() >= 3)
        {
            Vector3 center {0.0, 0.0, 0.0};
            for (const Vector3& point : cap) {
                center = center + point;
            }
            center = center / (double) cap.size();

            const Vector3 outward = plane.inwardNormal * -1.0;
            const Vector3 reference = std::fabs(outward.z) < 0.8
                ? Vector3 {0.0, 0.0, 1.0}
                : Vector3 {1.0, 0.0, 0.0};
            const Vector3 axisX = normalized(cross(reference, outward));
            const Vector3 axisY = cross(outward, axisX);

            std::sort(cap.begin(), cap.end(), [&](const Vector3& first, const Vector3& second) {
                const Vector3 firstOffset = first - center;
                const Vector3 secondOffset = second - center;
                return std::atan2(dot(firstOffset, axisY), dot(firstOffset, axisX))
                    < std::atan2(dot(secondOffset, axisY), dot(secondOffset, axisX));
            });
            clippedFaces.push_back(std::move(cap));
        }

        return clippedFaces;
    }

    std::vector<MeteorMapGeometry::Coordinate> footprintFromVertices(
        const std::vector<MeteorMapGeometry::Coordinate>& vertices)
    {
        struct Point { double x; double y; MeteorMapGeometry::Coordinate coordinate; };
        std::vector<Point> points;

        if (vertices.empty()) {
            return {};
        }

        const double longitudeReference = vertices.front().m_longitude;
        points.reserve(vertices.size());
        for (const MeteorMapGeometry::Coordinate& coordinate : vertices)
        {
            double longitude = coordinate.m_longitude;
            while (longitude - longitudeReference > 180.0) longitude -= 360.0;
            while (longitude - longitudeReference < -180.0) longitude += 360.0;
            points.push_back({longitude, coordinate.m_latitude, coordinate});
        }

        std::sort(points.begin(), points.end(), [](const Point& first, const Point& second) {
            return (first.x < second.x) || ((first.x == second.x) && (first.y < second.y));
        });
        points.erase(std::unique(points.begin(), points.end(), [](const Point& first, const Point& second) {
            return (std::fabs(first.x - second.x) < 1.0e-9) && (std::fabs(first.y - second.y) < 1.0e-9);
        }), points.end());

        if (points.size() < 3) {
            return {};
        }

        auto turn = [](const Point& first, const Point& second, const Point& third) {
            return (second.x - first.x) * (third.y - first.y)
                - (second.y - first.y) * (third.x - first.x);
        };
        std::vector<Point> hull;
        for (const Point& point : points)
        {
            while (hull.size() >= 2 && turn(hull[hull.size() - 2], hull.back(), point) <= 0.0) {
                hull.pop_back();
            }
            hull.push_back(point);
        }
        const std::size_t lowerSize = hull.size();
        for (auto it = points.rbegin() + 1; it != points.rend(); ++it)
        {
            while (hull.size() > lowerSize && turn(hull[hull.size() - 2], hull.back(), *it) <= 0.0) {
                hull.pop_back();
            }
            hull.push_back(*it);
        }
        hull.pop_back();

        std::vector<MeteorMapGeometry::Coordinate> footprint;
        footprint.reserve(hull.size());
        for (const Point& point : hull)
        {
            MeteorMapGeometry::Coordinate coordinate = point.coordinate;
            coordinate.m_longitude = normalizedLongitude(point.x);
            coordinate.m_altitudeM = 0.0;
            footprint.push_back(coordinate);
        }
        return footprint;
    }
}

bool MeteorMapGeometry::Mesh::isValid() const
{
    if ((m_vertices.size() < 4)
        || m_triangleIndices.empty()
        || ((m_triangleIndices.size() % 3) != 0))
    {
        return false;
    }

    for (const Coordinate& vertex : m_vertices)
    {
        if (!std::isfinite(vertex.m_latitude)
            || !std::isfinite(vertex.m_longitude)
            || !std::isfinite(vertex.m_altitudeM)
            || (vertex.m_latitude < -90.0)
            || (vertex.m_latitude > 90.0)
            || (vertex.m_longitude < -180.0)
            || (vertex.m_longitude > 180.0))
        {
            return false;
        }
    }

    return std::all_of(
        m_triangleIndices.begin(),
        m_triangleIndices.end(),
        [this](std::uint32_t index) { return index < m_vertices.size(); });
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

MeteorMapGeometry::Mesh MeteorMapGeometry::beamVolume(const BeamDefinition& beam, int edgeSegments)
{
    Mesh mesh;

    if (!validBeam(beam)) {
        return mesh;
    }

    edgeSegments = std::clamp(edgeSegments, 2, 64);
    Vector3 forward;
    Vector3 right;
    Vector3 vertical;
    beamFrame(beam, forward, right, vertical);
    const Vector3 origin = geodeticToECEF({beam.m_latitude, beam.m_longitude, beam.m_altitudeM});
    const double horizontalHalfAngle = degreesToRadians(
        std::clamp(beam.m_horizontalBeamwidthDegrees / 2.0, 0.01, 89.9));
    const double verticalHalfAngle = degreesToRadians(
        std::clamp(beam.m_verticalBeamwidthDegrees / 2.0, 0.01, 89.9));
    const double elevation = degreesToRadians(beam.m_elevationDegrees);
    const double lowerVerticalOffset = std::max(-verticalHalfAngle, -elevation);
    const double upperVerticalOffset = std::min(verticalHalfAngle, Pi / 2.0 - elevation);
    std::vector<Vector3> vertices;
    std::vector<Vector3> perimeter;

    auto appendEdge = [&](double horizontalStart, double verticalStart, double horizontalEnd, double verticalEnd) {
        for (int i = 0; i < edgeSegments; ++i)
        {
            const double ratio = (double) i / edgeSegments;
            Vector3 point;
            if (!rayToAltitude(
                    origin,
                    rayAtOffsets(
                        forward,
                        right,
                        vertical,
                        horizontalStart + (horizontalEnd - horizontalStart) * ratio,
                        verticalStart + (verticalEnd - verticalStart) * ratio),
                    beam.m_maxAltitudeM,
                    point))
            {
                perimeter.clear();
                return;
            }
            perimeter.push_back(point);
        }
    };

    appendEdge(-horizontalHalfAngle, lowerVerticalOffset, horizontalHalfAngle, lowerVerticalOffset);
    appendEdge(horizontalHalfAngle, lowerVerticalOffset, horizontalHalfAngle, upperVerticalOffset);
    appendEdge(horizontalHalfAngle, upperVerticalOffset, -horizontalHalfAngle, upperVerticalOffset);
    appendEdge(-horizontalHalfAngle, upperVerticalOffset, -horizontalHalfAngle, lowerVerticalOffset);

    Vector3 capCenter;
    if (perimeter.size() != (std::size_t) edgeSegments * 4
        || !rayToAltitude(origin, forward, beam.m_maxAltitudeM, capCenter))
    {
        return {};
    }

    vertices.reserve(perimeter.size() + 2);
    vertices.push_back(origin);
    vertices.push_back(capCenter);
    vertices.insert(vertices.end(), perimeter.begin(), perimeter.end());
    mesh.m_vertices.reserve(vertices.size());
    for (const Vector3& vertex : vertices) {
        mesh.m_vertices.push_back(ecefToGeodetic(vertex));
    }

    const Vector3 interior = origin + (capCenter - origin) * 0.4;
    for (std::uint32_t i = 0; i < perimeter.size(); ++i)
    {
        const std::uint32_t current = i + 2;
        const std::uint32_t next = ((i + 1) % perimeter.size()) + 2;
        appendTriangle(mesh, 0, current, next, interior, vertices);
        appendTriangle(mesh, 1, next, current, interior, vertices);
    }

    mesh.m_footprint = beamFootprint(
        beam.m_latitude,
        beam.m_longitude,
        beam.m_azimuthDegrees,
        beam.m_elevationDegrees,
        beam.m_horizontalBeamwidthDegrees,
        beam.m_verticalBeamwidthDegrees,
        beam.m_maxAltitudeM);
    return mesh;
}

MeteorMapGeometry::Mesh MeteorMapGeometry::beamIntersection(
    const BeamDefinition& first,
    const BeamDefinition& second,
    int edgeSegments)
{
    const Mesh firstMesh = beamVolume(first, edgeSegments);
    Mesh intersection;

    if (!firstMesh.isValid() || !validBeam(second)) {
        return intersection;
    }

    std::vector<Vector3> sourceVertices;
    sourceVertices.reserve(firstMesh.m_vertices.size());
    for (const Coordinate& vertex : firstMesh.m_vertices) {
        sourceVertices.push_back(geodeticToECEF(vertex));
    }

    std::vector<Face> faces;
    faces.reserve(firstMesh.m_triangleIndices.size() / 3);
    for (std::size_t i = 0; i < firstMesh.m_triangleIndices.size(); i += 3)
    {
        faces.push_back({
            sourceVertices[firstMesh.m_triangleIndices[i]],
            sourceVertices[firstMesh.m_triangleIndices[i + 1]],
            sourceVertices[firstMesh.m_triangleIndices[i + 2]]
        });
    }

    for (const Plane& plane : beamPlanes(second))
    {
        faces = clipPolyhedron(std::move(faces), plane);
        if (faces.empty()) {
            return {};
        }
    }

    std::vector<Vector3> vertices;
    for (const Face& face : faces)
    {
        if (face.size() < 3) {
            continue;
        }

        std::vector<std::uint32_t> faceIndices;
        for (const Vector3& point : face)
        {
            std::uint32_t index = 0;
            for (; index < vertices.size(); ++index)
            {
                if (length(vertices[index] - point) <= 0.25) {
                    break;
                }
            }
            if (index == vertices.size()) {
                vertices.push_back(point);
            }
            faceIndices.push_back(index);
        }

        for (std::size_t i = 1; i + 1 < faceIndices.size(); ++i)
        {
            const Vector3 normal = cross(
                vertices[faceIndices[i]] - vertices[faceIndices[0]],
                vertices[faceIndices[i + 1]] - vertices[faceIndices[0]]);
            if (length(normal) > 0.01)
            {
                intersection.m_triangleIndices.push_back(faceIndices[0]);
                intersection.m_triangleIndices.push_back(faceIndices[i]);
                intersection.m_triangleIndices.push_back(faceIndices[i + 1]);
            }
        }
    }

    intersection.m_vertices.reserve(vertices.size());
    for (const Vector3& vertex : vertices) {
        intersection.m_vertices.push_back(ecefToGeodetic(vertex));
    }
    intersection.m_footprint = footprintFromVertices(intersection.m_vertices);

    if (!intersection.isValid()) {
        return {};
    }
    return intersection;
}
