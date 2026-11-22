#pragma once
#include <array>
#include <atomic>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace pc {
struct Vec3 {
    double x = 0, y = 0, z = 0;
    double operator[](int axis) const { return axis == 0 ? x : axis == 1 ? y : z; }
};
Vec3 operator+(Vec3 a, Vec3 b);
Vec3 operator-(Vec3 a, Vec3 b);
Vec3 operator*(Vec3 a, double s);
double norm2(Vec3 a);
struct Cloud {
    std::vector<Vec3> points;
    size_t invalidPoints = 0;
};
struct Transform {
    std::array<double, 9> r{{1,0,0, 0,1,0, 0,0,1}};
    Vec3 t;
    Vec3 apply(Vec3 p) const;
};
struct Result {
    Transform bToA;
    std::vector<Vec3> alignedB;
    std::vector<unsigned char> differentA, differentB;
    double similarity = 0, coverageA = 0, coverageB = 0;
    double rmse = 0, threshold = 0;
    size_t matchedA = 0, matchedB = 0;
    int iterations = 0;
    bool hasMatchedPairs = false;
};
using Progress = std::function<void(const std::string&)>;
Cloud readPCD(const std::filesystem::path& file, const std::atomic_bool* cancel = nullptr);
Result compare(const Cloud& a, const Cloud& b, double differenceThreshold = 0,
    const std::atomic_bool* cancel = nullptr, Progress progress = {});
void exportResult(const std::filesystem::path& directory, const Cloud& a,
    const Result& result, const std::atomic_bool* cancel = nullptr);
}
