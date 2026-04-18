#pragma once
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <filesystem>

class AnimalFilter {
public:
    AnimalFilter(const std::string& animal_name);
    cv::Mat apply_filter(cv::Mat frame, const std::vector<cv::Point2f>& landmarks);

private:
    void load_masks(const std::string& animal_name);
    cv::Mat rotate_image(const cv::Mat& image, double angle);
    void optimized_overlay(cv::Mat& bg, const cv::Mat& overlay, int x, int y);
    float smooth_value(std::deque<float>& history, float new_value, size_t max_size = 5);

    cv::Mat mask_closed_closed_;
    cv::Mat mask_open_closed_;
    cv::Mat mask_closed_open_;
    cv::Mat mask_open_open_;
    
    std::deque<float> mouth_open_history_;
    std::deque<float> eyes_open_history_;
};