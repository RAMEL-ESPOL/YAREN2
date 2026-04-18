#ifndef POSE_DETECTOR_HPP
#define POSE_DETECTOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "yaren_interfaces/msg/pose_result.hpp"
#include "yaren_interfaces/msg/landmarks.hpp"
#include <chrono>
#include <memory>
#include <map>
#include <string>
#include <vector>
#include <cmath>

class YarenPoseDetector : public rclcpp::Node
{
public:
    YarenPoseDetector();

private:
    void handle_new_challenge(const std_msgs::msg::Int16::SharedPtr msg);
    bool is_above(const std::pair<float, float>& point1, const std::pair<float, float>& point2, float threshold = 30.0f);
    bool is_below(const std::pair<float, float>& point1, const std::pair<float, float>& point2, float threshold = 15.0f);
    bool is_at_same_height(const std::pair<float, float>& point1, const std::pair<float, float>& point2, float threshold = 30.0f);
    bool is_in_horizontal_range(const std::pair<float, float>& point1, const std::pair<float, float>& point2, float threshold = 30.0f);
    bool is_right_of(const std::pair<float, float>& point1, const std::pair<float, float>& point2, float threshold = 30.0f);
    bool is_left_of(const std::pair<float, float>& point1, const std::pair<float, float>& point2, float threshold = 30.0f);
    bool is_near(const std::pair<float, float>& point1, const std::pair<float, float>& point2, float threshold = 50.0f);
    bool detect_pose_actions(const std::vector<std::pair<float, float>>& keypoints);
    void detect_poses(const yaren_interfaces::msg::Landmarks::SharedPtr msg);
    
    rclcpp::Publisher<yaren_interfaces::msg::PoseResult>::SharedPtr pose_result_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr current_challenge_subscription_;
    rclcpp::Subscription<yaren_interfaces::msg::Landmarks>::SharedPtr pose_landmarks_subscription_;
    
    int current_challenge_;
    bool game_active_;
    
    int NOSE;
    int LEFT_EYE;
    int RIGHT_EYE;
    int LEFT_EAR;
    int RIGHT_EAR;
    int LEFT_SHOULDER;
    int RIGHT_SHOULDER;
    int LEFT_ELBOW;
    int RIGHT_ELBOW;
    int LEFT_WRIST;
    int RIGHT_WRIST;
    int LEFT_HIP;
    int RIGHT_HIP;
};

#endif