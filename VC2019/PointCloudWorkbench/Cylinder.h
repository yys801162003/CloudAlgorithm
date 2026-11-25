#pragma once
#include "PointCloud.h"

namespace pc {
struct Segment { Vec3 from, to; };
struct Section {
    Vec3 center, axisPoint;
    Segment diameter, width;
    double maxDiameter=0, minWidth=0, equivalentDiameter=0, ellipticity=0, offset=0;
};
struct CylinderMeasurements {
    Segment lengthLine;
    double length=0;
    size_t validSections=0;
    Section maxDiameter, minWidth, minDiameter, maxEllipticity;
};
struct Cylinder {
    Vec3 center;       // Midpoint of the reconstructed finite axis segment.
    Vec3 axis;         // Unit vector in the input cloud's coordinates.
    double radius=0, height=0, threshold=0, rmse=0, inlierRatio=0, angularCoverage=0;
    std::vector<unsigned char> inliers; // One entry per original valid point.
    CylinderMeasurements measurements;
    std::vector<Vec3> vertices;
    std::vector<std::array<size_t,3>> triangles;
};
struct AxisAngle {
    Vec3 directionA, directionB;
    double degrees=0;
};
AxisAngle measureAxisAngle(const Cylinder& a, const Cylinder& b,
    const Transform* bToA=nullptr);
Cylinder reconstructCylinder(const Cloud& cloud, double distanceThreshold=0,
    const std::atomic_bool* cancel=nullptr, Progress progress={});
void exportCylinder(const std::filesystem::path& directory, const Cloud& source,
    const Cylinder& cylinder, const std::atomic_bool* cancel=nullptr);
}
