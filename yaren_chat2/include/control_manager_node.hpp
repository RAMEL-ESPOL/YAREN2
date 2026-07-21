#ifndef CONTROL_MANAGER_NODE_HPP_
#define CONTROL_MANAGER_NODE_HPP_

#include <memory>
#include <thread>
#include <mutex>
#include <string>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

class LifecycleNodesManager : public rclcpp::Node {
public:
    LifecycleNodesManager();

private:
    // ── Suscripciones y clientes ──────────────────────────────────────
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stt_status_sub_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr stt_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr llm_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr tts_state_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr yaren_ready_publisher_;

    void publish_yaren_ready();

    // ── Estado ────────────────────────────────────────────────────────
    bool stt_terminated_;
    bool lifecycle_busy_ = false;
    std::mutex state_lock_;
    std::thread init_thread_;

    // ── Transición síncrona con timeout ──────────────────────────────
    bool change_node_state(
        const std::string& node_name,
        uint8_t transition_id,
        std::chrono::seconds timeout = std::chrono::seconds(15)
    );

    // ── Métodos privados ──────────────────────────────────────────────
    void _configure_initial_nodes();   // ← lanza el hilo de configuración inicial
    void _initial_configuration();     // ← corre dentro del hilo
    void manage_node_lifecycle();      // ← lanza el hilo de transiciones
    void _manage_lifecycle_thread();   // ← corre dentro del hilo
    void stt_status_callback(const std_msgs::msg::Bool::SharedPtr msg);

    bool wait_for_service(
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client,
        const std::string& node_name,
        int timeout_sec
    );

    bool change_node_state_sync(const std::string& node_name, uint8_t transition_id);
};

#endif  // CONTROL_MANAGER_NODE_HPP_