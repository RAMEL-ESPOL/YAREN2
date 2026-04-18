#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <filesystem>

class FaceFilter {
protected:
    std::vector<cv::Mat> assets;
    size_t current_asset_idx = 0;

    bool validLandmark(const cv::Point2f& point);
    bool validatePosition(int x, int y, const cv::Size& asset_size, const cv::Size& frame_size);

public:
    struct FilterParams {
        int min_distance;
        int max_distance;
        float width_factor;
        int min_clamp_width;
        int max_clamp_width;
        float height_factor;
        int min_clamp_height;
        int max_clamp_height;
    };
    
    size_t getCurrentIndex() const { return current_asset_idx; }
    void setCurrentIndex(size_t idx) { current_asset_idx = idx % assets.size(); }
    size_t getAssetsSize() const { return assets.size(); }
    void incrementIndex() { current_asset_idx = (current_asset_idx + 1) % assets.size(); }
    void decrementIndex() { current_asset_idx = (current_asset_idx - 1 + assets.size()) % assets.size(); }

    std::vector<cv::Mat> loadAssets(const std::string& path);
    cv::Mat applyFilterCommon(cv::Mat frame, const std::vector<cv::Point2f>& landmarks, const cv::Size& frame_size);
    cv::Mat rotateImage(const cv::Mat& image, double angle);
    void optimizedOverlay(cv::Mat& bg, const cv::Mat& overlay, int x, int y);

    FaceFilter() = default;
    explicit FaceFilter(const std::string& assets_path);

    virtual std::pair<int, int> getLandmarkIndices() const = 0;
    virtual FilterParams getFilterParams() const = 0;
    virtual std::pair<int, int> calculatePosition(const cv::Mat& rotated_asset, const std::vector<cv::Point2f>& landmarks) const = 0;

    virtual cv::Mat applyFilter(cv::Mat frame, const std::vector<cv::Point2f>& landmarks, const cv::Size& frame_size) {
        return applyFilterCommon(frame, landmarks, frame_size);
    }
};