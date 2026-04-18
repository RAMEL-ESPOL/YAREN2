#ifndef GAME_MANAGER_HPP
#define GAME_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "yaren_interfaces/msg/pose_result.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <string>

enum class GameLevel {
    BASIC = 1,
    INTERMEDIATE = 2,
    ADVANCED = 3
};

class YarenGameManager : public rclcpp::Node
{
public:
    YarenGameManager();

private:
    void handle_audio_status(const std_msgs::msg::Bool::SharedPtr msg);
    void start_detection();
    void check_challenge_timeout();
    void handle_pose_result(const yaren_interfaces::msg::PoseResult::SharedPtr msg);
    void handle_successful_challenge();
    void handle_failed_challenge(const std::string& feedback_text);
    double get_current_time();
    void load_challenges_from_yaml();
    void load_intermediate_challenges_from_yaml();
    void load_advanced_challenges_from_yaml();
    void select_challenge();
    GameLevel get_current_level();
    void announce_level_up(GameLevel new_level);
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr current_challenge_publisher_;
    
    rclcpp::Subscription<yaren_interfaces::msg::PoseResult>::SharedPtr pose_result_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr audio_status_subscription_;
    
    rclcpp::TimerBase::SharedPtr challenge_timer_;

    std::vector<std::string> victory_texts_;
    std::vector<std::string> defeat_texts_;
    
    std::vector<YAML::Node> challenges_;
    std::vector<YAML::Node> intermediate_challenges_;
    std::vector<YAML::Node> advanced_challenges_;
    
    GameLevel current_level_;
    std::vector<int> current_sequence_;
    int current_sequence_step_;
    int expected_sequence_length_;
    int current_challenge_;
    int score_;
    int lives_;
    bool audio_playing_;
    bool detection_ongoing_;
    double challenge_timeout_;
    bool waiting_for_pose_;
    double correct_pose_start_time_;
    double correct_pose_duration_;
};

#endif