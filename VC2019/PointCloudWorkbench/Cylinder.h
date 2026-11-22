#pragma once
#include "PointCloud.h"

namespace pc {
struct Cylinder {
    Vec3 center;       // Midpoint of the reconstructed finite axis segment.
    Vec3 axis;         // Unit vector in the input cloud's coordinates.
    double radius=0, height=0, threshold=0, rmse=0, inlierRatio=0, angularCoverage=0;
    std::vector<unsigned char> inliers; // One entry per original valid point.
    std::vector<Vec3> vertices;
    std::vector<std::array<size_t,3>> triangles;
};
Cylinder reconstructCylinder(const Cloud& cloud, double distanceThreshold=0,
    const std::atomic_bool* cancel=nullptr, Progress progress={});
void exportCylinder(const std::filesystem::path& directory, const Cloud& source,
    const Cylinder& cylinder, const std::atomic_bool* cancel=nullptr);
}
