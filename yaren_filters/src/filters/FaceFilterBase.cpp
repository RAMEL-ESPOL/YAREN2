#include "filters/FaceFilterBase.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

using namespace cv;
namespace fs = std::filesystem;

std::vector<cv::Mat> FaceFilter::loadAssets(const std::string& path) {
    std::vector<cv::Mat> loaded_assets;
    for (const auto& entry : fs::directory_iterator(path)) {
        Mat img = imread(entry.path().string(), IMREAD_UNCHANGED);
        if (!img.empty() && img.cols > 0 && img.rows > 0) {
            loaded_assets.push_back(img);
        }
    }
    return loaded_assets;
}

bool FaceFilter::validLandmark(const cv::Point2f& point) {
    return std::isnan(point.x) || std::isinf(point.x) || std::isnan(point.y) || std::isinf(point.y);
}

bool FaceFilter::validatePosition(int x, int y, const cv::Size& asset_size, const cv::Size& frame_size) {
    const float PADDING_FACTOR = 0.3f;
    return (x > -asset_size.width * PADDING_FACTOR) &&
           (y > -asset_size.height * PADDING_FACTOR) &&
           (x + asset_size.width < frame_size.width * (1 + PADDING_FACTOR)) &&
           (y + asset_size.height < frame_size.height * (1 + PADDING_FACTOR)) &&
           (asset_size.width > 50) &&  
           (asset_size.height > 50);
}

FaceFilter::FaceFilter(const std::string& assets_path) {
    this->assets = loadAssets(assets_path);
}

cv::Mat FaceFilter::rotateImage(const cv::Mat& image, double angle) {
    if (image.empty() || angle > 360.0 || angle < -360.0) return cv::Mat();
    
    cv::Point2f center(image.cols / 2.0f, image.rows / 2.0f);
    cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle, 1.0);
    
    double sin = std::abs(rot_mat.at<double>(0, 1));
    double cos = std::abs(rot_mat.at<double>(0, 0));
    int new_w = static_cast<int>(image.rows * sin + image.cols * cos);
    int new_h = static_cast<int>(image.rows * cos + image.cols * sin);
    
    if (new_w > 1000 || new_h > 1000) return cv::Mat();
    
    rot_mat.at<double>(0, 2) += (new_w - image.cols) / 2.0;
    rot_mat.at<double>(1, 2) += (new_h - image.rows) / 2.0;
    
    cv::Mat rotated;
    cv::warpAffine(image, rotated, rot_mat, cv::Size(new_w, new_h));
    return rotated;
}

void FaceFilter::optimizedOverlay(cv::Mat& bg, const cv::Mat& overlay, int x, int y) {
    if (overlay.empty() || bg.empty() ||
        x >= bg.cols || y >= bg.rows ||
        x + overlay.cols <= 0 || y + overlay.rows <= 0) {
        return;
    }

    int y1 = std::max(y, 0);
    int y2 = std::min(y + overlay.rows, bg.rows);
    int x1 = std::max(x, 0);
    int x2 = std::min(x + overlay.cols, bg.cols);

    if (x1 >= x2 || y1 >= y2) return;

    cv::Mat bg_3channel;
    if (bg.channels() == 4) {
        cv::cvtColor(bg, bg_3channel, cv::COLOR_BGRA2BGR);
    } else if (bg.channels() == 3) {
        bg_3channel = bg;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("FaceFilter"), "Unsupported background image channels");
        return;
    }

    cv::Rect roi(x1, y1, x2 - x1, y2 - y1);
    cv::Mat bg_roi = bg_3channel(roi);

    cv::Rect overlay_roi(x1 - x, y1 - y, x2 - x1, y2 - y1);
    cv::Mat overlay_region = overlay(overlay_roi);

    std::vector<cv::Mat> overlay_channels(4);
    cv::split(overlay_region, overlay_channels);

    cv::Mat alpha;
    overlay_channels[3].convertTo(alpha, CV_32F, 1.0/255.0);
    cv::Mat inv_alpha = 1.0 - alpha;

    std::vector<cv::Mat> overlay_color_channels(3);
    for (int i = 0; i < 3; ++i) {
        overlay_color_channels[i] = overlay_channels[i].clone();
    }

    std::vector<cv::Mat> bg_channels(3);
    cv::split(bg_roi, bg_channels);

    for (int c = 0; c < 3; ++c) {
        cv::Mat bg_chan_float, ovl_chan_float;
        bg_channels[c].convertTo(bg_chan_float, CV_32F);
        overlay_color_channels[c].convertTo(ovl_chan_float, CV_32F);

        cv::Mat blended = bg_chan_float.mul(inv_alpha) + ovl_chan_float.mul(alpha);
        blended.convertTo(bg_channels[c], CV_8U);
    }

    cv::merge(bg_channels, bg_roi);
}

cv::Mat FaceFilter::applyFilterCommon(
    cv::Mat frame, 
    const std::vector<cv::Point2f>& landmarks, 
    const cv::Size& frame_size
) {
    if (frame.empty() || assets.empty() || landmarks.size() < 468) return frame;

    try {
        cv::Mat asset = assets[current_asset_idx].clone();
        if (current_asset_idx >= assets.size() || asset.channels() != 4) {
            RCLCPP_ERROR(rclcpp::get_logger("FaceFilter"), "Asset sin canal alpha");
            return frame;
        }

        auto [left_idx, right_idx] = getLandmarkIndices();
        cv::Point2f left_point = landmarks[left_idx];
        cv::Point2f right_point = landmarks[right_idx];

        if (validLandmark(left_point) || validLandmark(right_point)) {
            RCLCPP_ERROR(rclcpp::get_logger("FaceFilter"), "Landmarks inválidos (NaN/Inf)");
            return frame;
        }

        float dx = right_point.x - left_point.x;
        float dy = right_point.y - left_point.y;
        double angle = -std::atan2(dy, dx) * 180.0 / CV_PI;
        float distance = std::hypot(dx, dy);

        FilterParams params = getFilterParams();
        if (distance < params.min_distance || distance > params.max_distance) return frame;

        int asset_width = static_cast<int>(distance * params.width_factor);
        asset_width = std::clamp(asset_width, params.min_clamp_width, params.max_clamp_width);

        float aspect_ratio = static_cast<float>(asset.rows) / asset.cols;
        int asset_height = static_cast<int>(asset_width * aspect_ratio * params.height_factor);
        asset_height = std::clamp(asset_height, params.min_clamp_height, params.max_clamp_height);

        cv::resize(asset, asset, cv::Size(asset_width, asset_height));

        cv::Mat rotated = rotateImage(asset, angle);
        if (rotated.empty()) return frame;

        auto [center_x, center_y] = calculatePosition(rotated, landmarks);

        if (!validatePosition(center_x, center_y, rotated.size(), frame_size)) {
            return frame;
        }

        optimizedOverlay(frame, rotated, center_x, center_y);
        return frame;

    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("FaceFilter"), "Error: %s", e.what());
        return frame;
    }
}