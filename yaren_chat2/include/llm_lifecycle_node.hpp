#ifndef LLM_LIFECYCLE_NODE_HPP
#define LLM_LIFECYCLE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <string>
#include <vector>
#include <regex>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mutex>
#include <unordered_map>

#include "llama.h"
#include "Yaren_interfaces/action/process_response.hpp"

namespace Yaren_chat {

class TextProcessor {
public:
    static std::vector<std::string> clean_text(const std::string& text);
};

class LLMLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
    using GoalHandleProcessResponse = rclcpp_action::ServerGoalHandle<Yaren_interfaces::action::ProcessResponse>;

    explicit LLMLifecycleNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LLMLifecycleNode();

private:
    rclcpp_action::Server<Yaren_interfaces::action::ProcessResponse>::SharedPtr action_server_;
    std::mutex llama_mutex_;
    
    std::string model_path_;
    llama_model* model_ = nullptr;
    llama_context* ctx_ = nullptr;
    llama_sampler* sampler_ = nullptr;
    const llama_vocab* vocab_ = nullptr;

    std::unordered_map<std::string, float> CONFIGURATIONS_ = {
        {"n_gpu_layers", 12.0},
        {"n_ctx", 4096.0},
        {"n_batch", 2048.0},
        {"n_threads", 4.0},
        {"min_p", 0.05},
        {"top_k", 40.0},
        {"top_p", 0.9},
        {"temp", 0.5},
        {"max_tokens_chat",300.0},
        {"max_tokens_summary", 200.0},
        {"token_buffer_size", 256.0},
        {"context_usage_threshold", 0.7}
    };
    
    struct ChatMessage {
        std::string role;
        std::string content;
    };
    std::vector<ChatMessage> conversation_history_;
    std::vector<ChatMessage> conversation_history_for_summary_;

    std::string system_prompt_base_ = "Eres Yaren, un asistente virtual amigable para niños que se encuentran en hospitales "
                                    "y tienen entre 7 a 12 años. Tienes la capacidad de mantener conversaciones "
                                    "naturales y amigables, y puedes responder preguntas sobre una variedad de temas. "
                                    "Tienes las siguientes reglas:"
                                    "- Brinda respuestas cortas y concisas. Eres un asistente virtual, no un monologo donde solo hablas tú."
                                    "- Siempre debes ser amigable y comprensivo."
                                    "- Nunca debes dar consejos médicos o diagnósticos."
                                    "- Siempre debes mantener la conversación dentro de un contexto apropiado para niños."
                                    "- Siempre debes ser positivo y alentador."
                                    "- Siempre debes recordar que estás hablando con un niño y adaptar tu lenguaje y tono en consecuencia."
                                    "- Siempre debes recordar que el niño puede estar en un entorno hospitalario y puede estar lidiando con emociones difíciles."
                                    "- Siempre debes recordar que el niño puede estar lidiando con emociones muy dificiles, y debes ser comprensivo y alentador."
                                    "- No debes inventar información o hacer suposiciones sobre la vida del niño."
                                    "- No debes inventar información acerca del hospital o el tratamiento del niño."
                                    "- No debes hacer suposiciones sobre la vida del niño, y siempre debes ser comprensivo y alentador."
                                    "Al iniciar una conversación, debes preguntarle al niño información sobre él mismo, como su nombre, edad y qué le gusta hacer.";

    std::string summary_prompt_ = "Basado en la conversación anterior, genera un resumen corto y conciso dividido por puntos con la siguiente información:\n"
                                "- Datos personales mencionados\n"
                                "- Temas tratados\n"
                                "- Sentimiento de la persona durante la conversación\n"
                                "- Último tema que estaban conversando\n"
                                "- Sobre qué se quedaron hablando\n\n"
                                "Mantén este resumen breve y directo al punto, en caso de que falte información de algunas de esas menciona que no hay info."; 

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& state);

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const yaren_interfaces::action::ProcessResponse::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleProcessResponse> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleProcessResponse> goal_handle);

    void execute_response_generation(const std::shared_ptr<GoalHandleProcessResponse> goal_handle);

    bool manage_context(int tokens_to_add);

    std::string generate_conversation_summary();
};

} // namespace yaren_chat

#endif // LLM_LIFECYCLE_NODE_HPP