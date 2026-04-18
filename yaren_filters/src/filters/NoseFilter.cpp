#include "filters/NoseFilter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>

NoseFilter::NoseFilter(const std::string& assets_path) : FaceFilter(assets_path) {}

std::pair<int, int> NoseFilter::getLandmarkIndices() const {
    const int LEFT_NOSTRIL = 98;
    const int RIGHT_NOSTRIL = 327;
    
    return {LEFT_NOSTRIL, RIGHT_NOSTRIL};
}

FaceFilter::FilterParams NoseFilter::getFilterParams() const {
    return {10, 300, 1.5, 20, 200, 1.0, 20, 200};
}

std::pair<int, int> NoseFilter::calculatePosition(
    const cv::Mat& rotated_asset, 
    const std::vector<cv::Point2f>& landmarks
) const {
    const int CENTER_NOSE = 5;

    int nose_x = static_cast<int>(landmarks[CENTER_NOSE].x);
    int nose_y = static_cast<int>(landmarks[CENTER_NOSE].y);

    int center_x = nose_x - rotated_asset.cols / 2;
    int center_y = nose_y - rotated_asset.rows / 2;
    return {center_x, center_y};
}