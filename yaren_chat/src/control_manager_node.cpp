#include "control_manager_node.hpp"
#include <chrono>
#include <thread>

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
LifecycleNodesManager::LifecycleNodesManager() : Node("lifecycle_control_manager") {

    stt_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/stt_terminado", 10,
        std::bind(&LifecycleNodesManager::stt_status_callback, this, std::placeholders::_1));
    yaren_ready_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/yaren_ready", 10);
    stt_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
        "/stt_lifecycle_node/change_state");
    llm_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
        "/llm_lifecycle_node/change_state");
    tts_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
        "/tts_lifecycle_node/change_state");

    stt_terminated_ = false;

    // ✅ Un solo hilo de configuración inicial — sin detach para poder controlarlo
    init_thread_ = std::thread(&LifecycleNodesManager::_initial_configuration, this);
    init_thread_.detach();
}
void LifecycleNodesManager::publish_yaren_ready() {
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    yaren_ready_publisher_->publish(msg);
}
// ─────────────────────────────────────────────────────────────────────────────
// Configuración inicial — SECUENCIAL, una transición a la vez
// ─────────────────────────────────────────────────────────────────────────────
void LifecycleNodesManager::_initial_configuration() {
    RCLCPP_INFO(this->get_logger(), "🔧 Iniciando configuración secuencial...");

    // 1. Configurar LLM primero — es el más lento en responder
    //    Esperar hasta 30s a que el servicio exista (el LLM puede tardar en cargar)
    if (!wait_for_service(llm_state_client_, "/llm_lifecycle_node", 30)) return;
    if (!change_node_state_sync("/llm_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) return;
    RCLCPP_INFO(this->get_logger(), "✅ LLM configurado");

    // 2. Configurar TTS
    if (!wait_for_service(tts_state_client_, "/tts_lifecycle_node", 15)) return;
    if (!change_node_state_sync("/tts_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) return;
    RCLCPP_INFO(this->get_logger(), "✅ TTS configurado");

    // 3. Configurar y activar STT — arranca la escucha
    if (!wait_for_service(stt_state_client_, "/stt_lifecycle_node", 45)) return;
    if (!change_node_state_sync("/stt_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) return;
    RCLCPP_INFO(this->get_logger(), "✅ STT configurado");

    if (!change_node_state_sync("/stt_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) return;
    RCLCPP_INFO(this->get_logger(), "✅ STT activado — escuchando");
    publish_yaren_ready(); 
}

// ─────────────────────────────────────────────────────────────────────────────
// Callback: STT terminó de escuchar (/stt_terminado)
// ─────────────────────────────────────────────────────────────────────────────
void LifecycleNodesManager::stt_status_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(state_lock_);
        // Ignorar si ya hay un cambio en curso
        if (lifecycle_busy_) return;
        stt_terminated_ = msg->data;
        lifecycle_busy_ = true;
    }
    // Lanzar el hilo de gestión FUERA del lock para evitar deadlock
    std::thread t(&LifecycleNodesManager::_manage_lifecycle_thread, this);
    t.detach();
}

// ─────────────────────────────────────────────────────────────────────────────
// Gestión de lifecycle — SECUENCIAL, sin locks anidados
// ─────────────────────────────────────────────────────────────────────────────
void LifecycleNodesManager::_manage_lifecycle_thread() {
    bool terminated;
    {
        std::lock_guard<std::mutex> lock(state_lock_);
        terminated = stt_terminated_;
    }

    if (terminated) {
        // STT terminó de escuchar → LLM + TTS activos, STT desactivado
        RCLCPP_INFO(this->get_logger(), "🔄 STT terminó → activando LLM y TTS");
        change_node_state_sync("/llm_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        change_node_state_sync("/tts_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        change_node_state_sync("/stt_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    } else {
        // TTS terminó de hablar → STT activo, LLM + TTS desactivados
        RCLCPP_INFO(this->get_logger(), "🔄 TTS terminó → activando STT");
        change_node_state_sync("/stt_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        change_node_state_sync("/tts_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        change_node_state_sync("/llm_lifecycle_node", lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    
    }
    publish_yaren_ready();
    std::lock_guard<std::mutex> lock(state_lock_);
    lifecycle_busy_ = false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Esperar a que un servicio esté disponible (con timeout configurable)
// ─────────────────────────────────────────────────────────────────────────────
bool LifecycleNodesManager::wait_for_service(
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client,
    const std::string& node_name,
    int timeout_sec)
{
    RCLCPP_INFO(this->get_logger(), "⏳ Esperando servicio de %s...", node_name.c_str());
    if (!client->wait_for_service(std::chrono::seconds(timeout_sec))) {
        RCLCPP_ERROR(this->get_logger(),
            "❌ Service not available for %s (esperé %ds)",
            node_name.c_str(), timeout_sec);
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "✅ Servicio disponible: %s", node_name.c_str());
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Cambiar estado — SÍNCRONO (bloquea hasta recibir respuesta o timeout)
// ✅ FIX PRINCIPAL: sin hilos internos, ejecución garantizada en orden
// ─────────────────────────────────────────────────────────────────────────────
bool LifecycleNodesManager::change_node_state_sync(
    const std::string& node_name, uint8_t transition_id)
{
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client;
    
    // Determinar el timeout basado en el nodo
    // El STT necesita mucho más tiempo por la carga del modelo Vosk
    int timeout_sec = 15; 

    if (node_name.find("stt") != std::string::npos) {
        client = stt_state_client_;
        timeout_sec = 45; // Aumentado a 45s para evitar el error de Vosk
    }
    else if (node_name.find("llm") != std::string::npos) {
        client = llm_state_client_;
    }
    else if (node_name.find("tts") != std::string::npos) {
        client = tts_state_client_;
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "❌ Nodo desconocido: %s", node_name.c_str());
        return false;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_id;

    auto future = client->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "⏳ Solicitando transición %d para %s (timeout: %ds)...", 
                transition_id, node_name.c_str(), timeout_sec);

    // Esperar la respuesta con el timeout dinámico
    if (future.wait_for(std::chrono::seconds(timeout_sec)) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(),
            "❌ Timeout esperando respuesta de %s después de %ds (transición %d)",
            node_name.c_str(), timeout_sec, transition_id);
        return false;
    }

    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(this->get_logger(),
            "❌ Transición %d rechazada por %s",
            transition_id, node_name.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
        "✅ Transición %d exitosa en %s", transition_id, node_name.c_str());
    return true;
}
// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleNodesManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}