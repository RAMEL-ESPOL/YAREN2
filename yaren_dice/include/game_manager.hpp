#ifndef GAME_MANAGER_HPP
#define GAME_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <yaren_interfaces/msg/pose_result.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>
#include <random>
#include <map>
#include <atomic>
#include <opencv2/opencv.hpp>

enum class GameLevel {
    BASIC,
    INTERMEDIATE,
    ADVANCED
};

// Estructura para los botones de las interfaces
struct IntroData {
    bool clicked;
    cv::Rect btn;
};

class YarenGameManager : public rclcpp::Node
{
public:
    YarenGameManager();

private:
    // ── Callbacks ──────────────────────────────────────────────────────────
    void handle_pose_result(const yaren_interfaces::msg::PoseResult::SharedPtr msg);
    void handle_audio_status(const std_msgs::msg::Bool::SharedPtr msg);
    void handle_language_change(const std_msgs::msg::Bool::SharedPtr msg);
    void handle_emotion(const std_msgs::msg::Int16::SharedPtr msg);
    void check_challenge_timeout();

    // ── Carga de YAMLs ─────────────────────────────────────────────────────
    void load_challenges_robot_from_yaml();
    void load_challenges_from_yaml();
    void load_intermediate_challenges_from_yaml();
    void load_advanced_challenges_from_yaml();

    // ── Lógica del juego ───────────────────────────────────────────────────
    void show_intro_screen();
    void show_control_panel();
    void select_challenge();
    void start_detection();
    void move_robot(const std::vector<double>& raw_pose);
    void end_game();
    void handle_successful_challenge();
    void handle_failed_challenge(const std::string& feedback_text);
    void announce_level_up(GameLevel new_level);

    // ── Utilidades ─────────────────────────────────────────────────────────
    double get_current_time();

    // ── Publishers / Subscribers ───────────────────────────────────────────
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr current_challenge_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ui_state_publisher_;

    rclcpp::Subscription<yaren_interfaces::msg::PoseResult>::SharedPtr pose_result_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr audio_status_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr language_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr emotion_subscription_;

    rclcpp::TimerBase::SharedPtr challenge_timer_;

    // ── Estado del juego ───────────────────────────────────────────────────
    int current_challenge_;
    int score_;
    int challenges_played_;
    int level_score_;
    int lives_;
    bool audio_playing_;
    bool detection_ongoing_;
    bool waiting_for_pose_;
    bool is_english_;
    bool game_initialized_;
    bool use_help_;
    bool is_session_;
    std::atomic<bool> session_aborted_{false};

    double challenge_timeout_;
    double correct_pose_start_time_;
    double correct_pose_duration_;

    GameLevel current_level_;
    std::vector<int> current_sequence_;
    int current_sequence_step_;
    int expected_sequence_length_;

    std::chrono::steady_clock::time_point game_start_time_;
    std::mutex language_mutex_;

    // ── Challenges cargados ────────────────────────────────────────────────
    std::vector<YAML::Node> robot_challenges_;
    std::vector<YAML::Node> challenges_;
    std::vector<YAML::Node> intermediate_challenges_;
    std::vector<YAML::Node> advanced_challenges_;

    // ── Textos de feedback ─────────────────────────────────────────────────
    std::vector<std::string> victory_texts_es_;
    std::vector<std::string> victory_texts_en_;
    std::vector<std::string> defeat_texts_es_;
    std::vector<std::string> defeat_texts_en_;

    // ── Variables para control del robot ────────────────────────────
    std::vector<double> pending_robot_pose_;
    bool has_pending_robot_pose_;
    double pending_detection_start_time_;
    bool robot_moving_ = false;
    double robot_move_end_time_ = 0.0;
    double robot_move_duration_ = 2.5;           
    double detection_delay_after_move_ = 0.5;    
    double audio_end_delay_ = 0.5;               
    double challenge_timeout_seconds_ = 20.0;    

    // ── Variables de Reporte Clínico ───────────────────────────────────────
    int total_attempts_;
    int successful_attempts_;
    int total_emotion_readings_;
    std::map<int, int> emotion_counts_;
    std::string patient_name_;
};

#endif // GAME_MANAGER_HPP