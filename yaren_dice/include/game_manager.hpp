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
    std::mutex language_mutex_;
    std::chrono::steady_clock::time_point game_start_time_;
    // ── Callbacks ──────────────────────────────────────────────
    void handle_audio_status(const std_msgs::msg::Bool::SharedPtr msg);
    void handle_pose_result(const yaren_interfaces::msg::PoseResult::SharedPtr msg);
    void handle_language_change(const std_msgs::msg::Bool::SharedPtr msg);  // 🔹 Agregado
    
    // ── Lógica del juego ───────────────────────────────────────
    void start_detection();
    void check_challenge_timeout();
    void handle_successful_challenge();
    void handle_failed_challenge(const std::string& feedback_text);
    double get_current_time();
    
    // ── Carga de desafíos ──────────────────────────────────────
    void load_challenges_from_yaml();
    void load_intermediate_challenges_from_yaml();
    void load_advanced_challenges_from_yaml();
    void select_challenge();
    
    // ── Niveles ────────────────────────────────────────────────
    GameLevel get_current_level();
    void announce_level_up(GameLevel new_level);

    // ── Publishers/Subscribers ─────────────────────────────────
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr current_challenge_publisher_;
    rclcpp::Subscription<yaren_interfaces::msg::PoseResult>::SharedPtr pose_result_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr audio_status_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr language_subscription_;  // 🔹 Agregado
    rclcpp::TimerBase::SharedPtr challenge_timer_;

    // ── Estado del juego ───────────────────────────────────────
    GameLevel current_level_ = GameLevel::BASIC;
    std::vector<int> current_sequence_;
    int current_sequence_step_ = 0;
    int expected_sequence_length_ = 1;
    int current_challenge_ = 0;
    int score_ = 0;
    int lives_ = 3;
    
    bool audio_playing_ = false;
    bool detection_ongoing_ = false;
    bool waiting_for_pose_ = false;
    bool game_initialized_;
    double challenge_timeout_ = 0.0;
    double correct_pose_start_time_ = 0.0;
    double correct_pose_duration_ = 0.5;
    
    // ── Idioma bilingüe ────────────────────────────────────────
    bool is_english_ = false;
    bool language_received_ = false;  // 🔹 NUEVO: ¿Ya recibimos el primer mensaje de idioma?

    // ── Textos de feedback (bilingües) ─────────────────────────
    std::vector<std::string> victory_texts_es_;
    std::vector<std::string> victory_texts_en_;
    std::vector<std::string> defeat_texts_es_;
    std::vector<std::string> defeat_texts_en_;
    
    // ── Desafíos cargados desde YAML ───────────────────────────
    std::vector<YAML::Node> challenges_;
    std::vector<YAML::Node> intermediate_challenges_;
    std::vector<YAML::Node> advanced_challenges_;
};

#endif // GAME_MANAGER_HPP