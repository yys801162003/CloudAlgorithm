// SPDX-License-Identifier: AGPL-3.0-only
#pragma once
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <atomic>
#include <filesystem>
#include <string>
#include <vector>

namespace smoke {
struct Image { cv::Mat bgr; int originalWidth=0, originalHeight=0; };
struct Box { cv::Rect rect; float score=0; int classId=0; };
struct Analysis {
    bool detected=false;
    double value=0; // CV: suspected area percent. DNN: highest smoke score [0,1].
    double milliseconds=0;
    cv::Mat annotated;
    std::vector<Box> boxes;
};
struct Letterbox { cv::Mat image; double scale=1; int left=0,top=0; };
Image loadImage(const std::filesystem::path& path);
Letterbox letterbox(const cv::Mat& image);
std::vector<Box> decodeYolo(const cv::Mat& output, cv::Size original,
    const Letterbox& geometry, float confidence, double& highestSmokeScore);
Analysis detectOpenCV(const cv::Mat& image, double minimumAreaPercent,
    const std::atomic_bool* cancel=nullptr);
class NeuralDetector {
    cv::dnn::Net network_;
    std::filesystem::path model_;
public:
    Analysis detect(const cv::Mat& image, const std::filesystem::path& model,
        float confidence, const std::atomic_bool* cancel=nullptr);
};
void exportAnalyses(const std::filesystem::path& directory, const std::filesystem::path& source,
    const Image& image, const Analysis* cvResult, const Analysis* neuralResult,
    double areaThreshold, double neuralThreshold);
}
