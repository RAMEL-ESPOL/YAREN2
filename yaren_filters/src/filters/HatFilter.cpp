#include "filters/HatFilter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>

HatFilter::HatFilter(const std::string& assets_path) : FaceFilter(assets_path) {}

std::pair<int, int> HatFilter::getLandmarkIndices() const {
    const int FOREHEAD_RIGHT = 332;
    const int FOREHEAD_LEFT = 103; 
    
    return {FOREHEAD_LEFT, FOREHEAD_RIGHT};
}

FaceFilter::FilterParams HatFilter::getFilterParams() const {
    return {15, 300, 1.7, 30, 500, 0.75, 30, 500};
}

std::pair<int, int> HatFilter::calculatePosition(
    const cv::Mat& rotated_asset, 
    const std::vector<cv::Point2f>& landmarks
) const {
    const int FOREHEAD_LEFT = 103; 
    const int FOREHEAD_RIGHT = 332;

    int left_x = static_cast<int>(landmarks[FOREHEAD_LEFT].x);
    int left_y = static_cast<int>(landmarks[FOREHEAD_LEFT].y);
    int right_x = static_cast<int>(landmarks[FOREHEAD_RIGHT].x);
    int right_y = static_cast<int>(landmarks[FOREHEAD_RIGHT].y);

    int center_x = (left_x + right_x) / 2 - rotated_asset.cols / 2;
    int center_y = (left_y + right_y) / 2 - rotated_asset.rows / 2 - (rotated_asset.rows / 4);
    return {center_x, center_y};
}