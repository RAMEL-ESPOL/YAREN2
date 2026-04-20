#ifndef CONTROL_MANAGER_NODE_HPP_
#define CONTROL_MANAGER_NODE_HPP_

#include <memory>
#include <thread>
#include <mutex>
#include <string>
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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stt_status_sub_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr stt_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr llm_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr tts_state_client_;
    
    bool stt_terminated_;
    std::mutex state_lock_;

    void _configure_initial_nodes();
    void _initial_configuration();
    void stt_status_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void manage_node_lifecycle();
    void _manage_lifecycle_thread();
    void change_node_state(const std::string& node_name, uint8_t transition_id);
};

#endif // CONTROL_MANAGER_NODE_HPP_