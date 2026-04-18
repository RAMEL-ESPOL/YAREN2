#include "filters/MouthFilter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>

MouthFilter::MouthFilter(const std::string& assets_path) : FaceFilter(assets_path) {}

std::pair<int, int> MouthFilter::getLandmarkIndices() const {
    const int LEFT_MOUTH_CORNER = 61;
    const int RIGHT_MOUTH_CORNER = 291;
    
    return {LEFT_MOUTH_CORNER, RIGHT_MOUTH_CORNER};
}

FaceFilter::FilterParams MouthFilter::getFilterParams() const {
    return {10, 300, 1.3, 20, 300, 1.0, 10, 150};
}

std::pair<int, int> MouthFilter::calculatePosition(
    const cv::Mat& rotated_asset, 
    const std::vector<cv::Point2f>& landmarks
) const {
    const int LEFT_MOUTH_CORNER = 61;
    const int RIGHT_MOUTH_CORNER = 291;
    const int UPPER_LIP = 13;
    const int LOWER_LIP = 14;

    int left_x = static_cast<int>(landmarks[LEFT_MOUTH_CORNER].x);
    int right_x = static_cast<int>(landmarks[RIGHT_MOUTH_CORNER].x);
    int upper_y = static_cast<int>(landmarks[UPPER_LIP].y);
    int lower_y = static_cast<int>(landmarks[LOWER_LIP].y);

    int center_x = (left_x + right_x) / 2 - rotated_asset.cols / 2;
    int center_y = (upper_y + lower_y) / 2 - rotated_asset.rows / 2;
    return {center_x, center_y};
}