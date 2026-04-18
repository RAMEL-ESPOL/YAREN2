#include "filters/GlassesFilter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>

GlassesFilter::GlassesFilter(const std::string& assets_path) : FaceFilter(assets_path) {}

std::pair<int, int> GlassesFilter::getLandmarkIndices() const {
    const int LEFT_EYE = 33;
    const int RIGHT_EYE = 263; 
    
    return {LEFT_EYE, RIGHT_EYE};
}

FaceFilter::FilterParams GlassesFilter::getFilterParams() const {
    return {10, 300, 1.2, 20, 200, 1.0, 30, 500};
}

std::pair<int, int> GlassesFilter::calculatePosition(
    const cv::Mat& rotated_asset, 
    const std::vector<cv::Point2f>& landmarks
) const {
    const int LEFT_EYE = 33;
    const int RIGHT_EYE = 263; 

    int left_x = static_cast<int>(landmarks[LEFT_EYE].x);
    int left_y = static_cast<int>(landmarks[LEFT_EYE].y);
    int right_x = static_cast<int>(landmarks[RIGHT_EYE].x);
    int right_y = static_cast<int>(landmarks[RIGHT_EYE].y);

    int center_x = (left_x + right_x) / 2 - rotated_asset.cols / 2;
    int center_y = (left_y + right_y) / 2 - rotated_asset.rows / 2;
    return {center_x, center_y};
}