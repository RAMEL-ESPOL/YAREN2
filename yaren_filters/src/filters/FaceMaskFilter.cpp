#include "filters/FaceMaskFilter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>

FaceMaskFilter::FaceMaskFilter(const std::string& assets_path) : FaceFilter(assets_path) {}

std::pair<int, int> FaceMaskFilter::getLandmarkIndices() const {
    const int FACE_LEFT = 234;
    const int FACE_RIGHT = 454;
    
    return {FACE_LEFT, FACE_RIGHT};
}

FaceFilter::FilterParams FaceMaskFilter::getFilterParams() const {
    return {10, 300, 1.8, 100, 1000, 1.0, 100, 1000};
}

std::pair<int, int> FaceMaskFilter::calculatePosition(
    const cv::Mat& rotated_asset, 
    const std::vector<cv::Point2f>& landmarks
) const {
    const int FACE_LEFT = 234;
    const int FACE_RIGHT = 454;
    const int FACE_TOP = 10;
    const int FACE_BOTTOM = 152;

    int left_x = static_cast<int>(landmarks[FACE_LEFT].x);
    int right_x = static_cast<int>(landmarks[FACE_RIGHT].x);
    int top_y = static_cast<int>(landmarks[FACE_TOP].y);
    int bottom_y = static_cast<int>(landmarks[FACE_BOTTOM].y);

    int center_x = (left_x + right_x) / 2 - rotated_asset.cols / 2;
    int center_y = (top_y + bottom_y) / 2 - rotated_asset.rows / 2;
    return {center_x, center_y};
}