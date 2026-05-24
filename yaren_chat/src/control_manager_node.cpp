#include "control_manager_node.hpp"

LifecycleNodesManager::LifecycleNodesManager() : Node("lifecycle_nodes_manager") {
    stt_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/stt_terminado", 10, 
        std::bind(&LifecycleNodesManager::stt_status_callback, this, std::placeholders::_1));
    
    stt_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
        "/stt_lifecycle_node/change_state");
    llm_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
        "/llm_lifecycle_node/change_state");
    tts_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
        "/tts_lifecycle_node/change_state");
    
    stt_terminated_ = false;
    
    _configure_initial_nodes();
}

void LifecycleNodesManager::_configure_initial_nodes() {
    std::thread t(&LifecycleNodesManager::_initial_configuration, this);
    t.detach();
}

void LifecycleNodesManager::_initial_configuration() {
    change_node_state("/stt_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    // std::this_thread::sleep_for(std::chrono::seconds(10));
    change_node_state("/stt_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    change_node_state("/llm_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    change_node_state("/tts_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
}

void LifecycleNodesManager::stt_status_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_lock_);
    stt_terminated_ = msg->data;
    manage_node_lifecycle();
}

void LifecycleNodesManager::manage_node_lifecycle() {
    std::thread t(&LifecycleNodesManager::_manage_lifecycle_thread, this);
    t.detach();
}

void LifecycleNodesManager::_manage_lifecycle_thread() {
    // 1. Leer el estado bajo el lock y soltarlo inmediatamente
    bool terminated;
    {
        std::lock_guard<std::mutex> lock(state_lock_);
        terminated = stt_terminated_;
    }

    // 2. Hacer las transiciones SIN el lock (son bloqueantes)
    if (terminated) {
        RCLCPP_INFO(get_logger(), "🔄 STT terminó → activando LLM y TTS");
        change_node_state("/llm_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        change_node_state("/tts_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        change_node_state("/stt_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    } else {
        RCLCPP_INFO(get_logger(), "🔄 TTS terminó → activando STT");
        change_node_state("/stt_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        change_node_state("/tts_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        change_node_state("/llm_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }
}

bool LifecycleNodesManager::change_node_state(
    const std::string& node_name, uint8_t transition_id,
    std::chrono::seconds timeout)
{
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_id;

    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client;
    if (node_name.find("stt") != std::string::npos)      client = stt_state_client_;
    else if (node_name.find("llm") != std::string::npos) client = llm_state_client_;
    else if (node_name.find("tts") != std::string::npos) client = tts_state_client_;
    else return false;

    if (!client->wait_for_service(timeout)) {
        RCLCPP_ERROR(get_logger(), "Service not available: %s", node_name.c_str());
        return false;
    }

    auto future = client->async_send_request(request);
    // Espera real con timeout — sin thread detached
    if (future.wait_for(timeout) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "Timeout en transición de %s", node_name.c_str());
        return false;
    }

    RCLCPP_INFO(get_logger(), "✅ Transición %d exitosa en %s",
                transition_id, node_name.c_str());
    return true;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleNodesManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}