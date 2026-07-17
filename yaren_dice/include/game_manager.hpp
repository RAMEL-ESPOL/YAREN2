#ifndef GAME_MANAGER_HPP
#define GAME_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include "yaren_interfaces/msg/pose_result.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>

enum class GameLevel {
    BASIC,
    INTERMEDIATE,
    ADVANCED
};

class YarenGameManager : public rclcpp::Node
{
public:
    YarenGameManager();

private:
    void load_challenges_from_yaml();
    void load_intermediate_challenges_from_yaml();
    void load_advanced_challenges_from_yaml();
    void load_challenges_robot_from_yaml();

    void handle_language_change(const std_msgs::msg::Bool::SharedPtr msg);
    void handle_pose_result(const yaren_interfaces::msg::PoseResult::SharedPtr msg);
    void handle_audio_status(const std_msgs::msg::Bool::SharedPtr msg);
    
    void select_challenge();
    void start_detection();
    void check_challenge_timeout();
    void handle_successful_challenge();
    void handle_failed_challenge(const std::string& feedback_text);
    void end_game();
    void announce_level_up(GameLevel new_level);
    
    // Función para mover el robot (aplica la conversión del GUI a radianes)
    void move_robot(const std::vector<double>& raw_pose);

    double get_current_time();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr current_challenge_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
    
    rclcpp::Subscription<yaren_interfaces::msg::PoseResult>::SharedPtr pose_result_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr audio_status_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr language_subscription_;
    rclcpp::TimerBase::SharedPtr challenge_timer_;

    std::vector<YAML::Node> challenges_;
    std::vector<YAML::Node> intermediate_challenges_;
    std::vector<YAML::Node> advanced_challenges_;
    std::vector<YAML::Node> robot_challenges_;

    int current_challenge_;
    int score_;
    int lives_;
    
    // Contadores separados según el modo
    int challenges_played_; // Para "Con Ayuda" (Max 10 pasos totales)
    int level_score_;       // Para "Sin Ayuda" (Max 10 aciertos para subir de nivel)
    bool use_help_;         // Define en qué modo estamos jugando

    bool audio_playing_;
    bool detection_ongoing_;
    double challenge_timeout_;
    bool waiting_for_pose_;
    double correct_pose_start_time_;
    double correct_pose_duration_;
    
    GameLevel current_level_;
    std::vector<int> current_sequence_;
    int current_sequence_step_;
    int expected_sequence_length_;

    std::vector<std::string> victory_texts_es_;
    std::vector<std::string> victory_texts_en_;
    std::vector<std::string> defeat_texts_es_;
    std::vector<std::string> defeat_texts_en_;

    bool is_english_;
    bool game_initialized_;
    std::mutex language_mutex_;
    std::chrono::steady_clock::time_point game_start_time_;
};

#endif // GAME_MANAGER_HPP