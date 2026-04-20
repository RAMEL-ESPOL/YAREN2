#include "llm_lifecycle_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace yaren_chat {

std::vector<std::string> TextProcessor::clean_text(const std::string& text) {
    std::locale::global(std::locale(""));
    std::cout.imbue(std::locale());
        
    std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
    std::wstring wtext = converter.from_bytes(text);
    
    std::wregex punct_regex(L"[!¡?¿*,.:;()\\[\\]{}]");
    std::wstring wcleaned = std::regex_replace(wtext, punct_regex, L" ");
    
    std::wregex space_regex(L"\\s+");
    wcleaned = std::regex_replace(wcleaned, space_regex, L" ");
    
    std::string cleaned = converter.to_bytes(wcleaned);
        
    std::vector<std::string> sentences;
    std::regex sentence_regex("([.!?])\\s+");
    std::sregex_token_iterator iter(cleaned.begin(), cleaned.end(), sentence_regex, {-1, 0});
    std::sregex_token_iterator end;
    
    std::string sentence;
    for (; iter != end; ++iter) {
        sentence += iter->str();
        if (std::regex_search(iter->str(), sentence_regex)) {
            sentences.push_back(sentence);
            sentence.clear();
        }
    }
    if (!sentence.empty()) {
        sentences.push_back(sentence);
    }
    
    return sentences;
}

LLMLifecycleNode::LLMLifecycleNode(const rclcpp::NodeOptions& options)
    : LifecycleNode("llm_lifecycle_node", options) {
    
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("yaren_chat");
    model_path_ = pkg_share_dir + "/models/LLM/models--MaziyarPanahi--Llama-3.2-1B-Instruct-GGUF/snapshots/b64ae94264258a3d7516a34a8c54928d32b19869/Llama-3.2-1B-Instruct.Q4_K_M.gguf";
    
    RCLCPP_INFO(get_logger(), "Model path: %s", model_path_.c_str());
}

LLMLifecycleNode::~LLMLifecycleNode() {
    if (sampler_ != nullptr) {
        llama_sampler_free(sampler_);
    }
    if (ctx_ != nullptr) {
        llama_free(ctx_);
    }
    if (model_ != nullptr) {
        llama_model_free(model_);
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LLMLifecycleNode::on_configure(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(get_logger(), "Configuring LLM Node");
    
    try {
        ggml_backend_load_all();
        
        llama_model_params model_params = llama_model_default_params();
        model_params.n_gpu_layers = static_cast<int>(CONFIGURATIONS_["n_gpu_layers"]);
        
        model_ = llama_model_load_from_file(model_path_.c_str(), model_params);
        if (!model_) {
            RCLCPP_ERROR(get_logger(), "Error: unable to load model");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        
        vocab_ = llama_model_get_vocab(model_);
        
        llama_context_params ctx_params = llama_context_default_params();
        ctx_params.n_ctx = static_cast<int>(CONFIGURATIONS_["n_ctx"]);
        ctx_params.n_batch = static_cast<int>(CONFIGURATIONS_["n_batch"]);
        ctx_params.n_threads = static_cast<int>(CONFIGURATIONS_["n_threads"]);
        
        ctx_ = llama_init_from_model(model_, ctx_params);
        if (!ctx_) {
            RCLCPP_ERROR(get_logger(), "Error: failed to create llama_context");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        
        sampler_ = llama_sampler_chain_init(llama_sampler_chain_default_params());
        llama_sampler_chain_add(sampler_, llama_sampler_init_min_p(CONFIGURATIONS_["min_p"], 1));
        llama_sampler_chain_add(sampler_, llama_sampler_init_top_k(static_cast<int>(CONFIGURATIONS_["top_k"])));
        llama_sampler_chain_add(sampler_, llama_sampler_init_top_p(CONFIGURATIONS_["top_p"], 1));
        llama_sampler_chain_add(sampler_, llama_sampler_init_temp(CONFIGURATIONS_["temp"])); 
        llama_sampler_chain_add(sampler_, llama_sampler_init_dist(LLAMA_DEFAULT_SEED));
                        
        action_server_ = rclcpp_action::create_server<yaren_interfaces::action::ProcessResponse>(
            this,
            "/response_llama",
            std::bind(&LLMLifecycleNode::handle_goal, this, _1, _2),
            std::bind(&LLMLifecycleNode::handle_cancel, this, _1),
            std::bind(&LLMLifecycleNode::handle_accepted, this, _1));
            
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to configure: %s", e.what());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LLMLifecycleNode::on_activate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(get_logger(), "Activating LLM Node");
        
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LLMLifecycleNode::on_deactivate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(get_logger(), "Deactivating LLM Node");
        
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse LLMLifecycleNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const yaren_interfaces::action::ProcessResponse::Goal> goal)
{
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LLMLifecycleNode::handle_cancel(
    const std::shared_ptr<GoalHandleProcessResponse> goal_handle)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LLMLifecycleNode::handle_accepted(const std::shared_ptr<GoalHandleProcessResponse> goal_handle)
{
    std::thread{std::bind(&LLMLifecycleNode::execute_response_generation, this, _1), goal_handle}.detach();
}

void LLMLifecycleNode::execute_response_generation(const std::shared_ptr<GoalHandleProcessResponse> goal_handle)
{
    std::lock_guard<std::mutex> lock(llama_mutex_); 
    
    auto feedback = std::make_shared<yaren_interfaces::action::ProcessResponse::Feedback>();
    auto result = std::make_shared<yaren_interfaces::action::ProcessResponse::Result>();

    if (!model_ || !ctx_ || !sampler_) {
        result->completed = false;
        goal_handle->abort(result);
        return;
    }
    
    conversation_history_.clear();
    conversation_history_.push_back({"system", system_prompt_base_});
    conversation_history_for_summary_.push_back({"system", system_prompt_base_});

    conversation_history_.push_back({"user", goal_handle->get_goal()->input_text});
    conversation_history_for_summary_.push_back({"user", goal_handle->get_goal()->input_text});
    
    const char* tmpl = llama_model_chat_template(model_, NULL);
    std::vector<llama_chat_message> llama_messages;
    
    for (const auto& msg : conversation_history_) {
        llama_chat_message llm_msg;
        llm_msg.role = msg.role.c_str();
        llm_msg.content = msg.content.c_str();
        llama_messages.push_back(llm_msg);
    }
    
    std::vector<char> formatted(llama_n_ctx(ctx_));
    int len = llama_chat_apply_template(tmpl, llama_messages.data(), llama_messages.size(), true, formatted.data(), formatted.size());
    
    if (len < 0 || len > static_cast<int>(formatted.size())) {
        RCLCPP_ERROR(get_logger(), "Failed to apply the chat template");
        result->completed = false;
        goal_handle->abort(result);
        return;
    }
    
    std::string prompt(formatted.begin(), formatted.begin() + len);
    
    const int n_prompt_tokens = -llama_tokenize(vocab_, prompt.c_str(), prompt.size(), NULL, 0, true, true);
    manage_context(n_prompt_tokens);
    std::vector<llama_token> prompt_tokens(n_prompt_tokens);
    if (llama_tokenize(vocab_, prompt.c_str(), prompt.size(), prompt_tokens.data(), prompt_tokens.size(), llama_kv_self_used_cells(ctx_) == 0, true) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to tokenize the prompt");
        result->completed = false;
        goal_handle->abort(result);
        return;
    }
    
    llama_batch batch = llama_batch_get_one(prompt_tokens.data(), prompt_tokens.size());
    std::string buffer;
    std::string full_response;
    
    llama_token new_token_id;
    int max_tokens = static_cast<int>(CONFIGURATIONS_["max_tokens_chat"]); 
    int token_count = 0;
    
    RCLCPP_INFO(get_logger(), "Model: ");
    while (token_count < max_tokens) {
        int n_ctx = llama_n_ctx(ctx_);
        int n_ctx_used = llama_kv_self_used_cells(ctx_);

        if (n_ctx_used + batch.n_tokens > n_ctx) {
            RCLCPP_ERROR(get_logger(), "Context size exceeded");
            break;
        }
        
        if (goal_handle->is_canceling()) {
            result->completed = false;
            goal_handle->canceled(result);
            return;
        }
        
        if (llama_decode(ctx_, batch)) {
            RCLCPP_ERROR(get_logger(), "Failed to decode");
            break;
        }
        
        new_token_id = llama_sampler_sample(sampler_, ctx_, -1);
        
        if (llama_vocab_is_eog(vocab_, new_token_id)) {
            break;
        }
        
        char token_buf[static_cast<int>(CONFIGURATIONS_["token_buffer_size"])];
        int n = llama_token_to_piece(vocab_, new_token_id, token_buf, sizeof(token_buf), 0, true);
        if (n < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to convert token to piece");
            break;
        }
        
        std::string piece(token_buf, n);
        buffer += piece;
        full_response += piece;
        
        size_t sentence_end = 0;
        size_t buffer_len = buffer.length();

        while (sentence_end < buffer_len) {
            size_t next_end = buffer.find_first_of(".!?", sentence_end);
            
            if (next_end == std::string::npos) break;
            
            if (next_end + 1 > buffer_len || std::isspace(buffer[next_end + 1])) {
                sentence_end = next_end + 1;
            } else {
                sentence_end = next_end + 1;
                continue;
            }
            
            std::string sentence = buffer.substr(0, sentence_end);
            sentence.erase(sentence.find_last_not_of(" \t\n\r") + 1);  
            
            if (!sentence.empty()) {
                std::string clean_phrase = TextProcessor::clean_text(sentence).front();
                feedback->current_chunk = clean_phrase;
                feedback->is_last_chunk = false;
                std::cout << clean_phrase << std::endl;
                goal_handle->publish_feedback(feedback);
                
                buffer = buffer.substr(sentence_end);
                buffer_len = buffer.length();
                sentence_end = 0;  
            }
        }
        
        batch = llama_batch_get_one(&new_token_id, 1);
        token_count++;
    }
    
    feedback->is_last_chunk = true;
    goal_handle->publish_feedback(feedback);
    
    conversation_history_.push_back({"assistant", full_response});
    conversation_history_for_summary_.push_back({"assistant", full_response});
    
    result->completed = true;
    goal_handle->succeed(result);
}

bool LLMLifecycleNode::manage_context(int tokens_to_add) {
    int n_ctx = llama_n_ctx(ctx_);
    int n_ctx_used = llama_kv_self_used_cells(ctx_);
    
    if ((n_ctx_used + tokens_to_add) >= (n_ctx * CONFIGURATIONS_["context_usage_threshold"])) {        
        RCLCPP_INFO(get_logger(), "Context usage: %d/%d, generating summary...", n_ctx_used, n_ctx);

        std::string conversation_summary_ = generate_conversation_summary();
        
        llama_kv_self_clear(ctx_);
        llama_kv_self_seq_rm(ctx_, -1, -1, -1);

        if (!conversation_summary_.empty()) {
            RCLCPP_INFO(get_logger(), "Generated summary: %s", conversation_summary_.c_str());
            system_prompt_base_ = system_prompt_base_ + "\n\nResumen de la conversación hasta ahora:\n" + conversation_summary_;
        }

        return true;
    }
    
    return false;
}

std::string LLMLifecycleNode::generate_conversation_summary() {    
    RCLCPP_INFO(get_logger(), "Generating conversation summary...");

    std::vector<ChatMessage> limited_history;
    
    limited_history.push_back({"system", system_prompt_base_});
    
    const int MAX_MESSAGES_FOR_SUMMARY = 4; 
    int start_idx = std::max(1, static_cast<int>(conversation_history_for_summary_.size()) - MAX_MESSAGES_FOR_SUMMARY*2);
    
    for (int i = start_idx; i < static_cast<int>(conversation_history_for_summary_.size()); ++i) {
        if (conversation_history_for_summary_[i].role == "user") {
            limited_history.push_back({"user", conversation_history_for_summary_[i].content.c_str()});
        } else if (conversation_history_for_summary_[i].role == "assistant") {
            limited_history.push_back({"assistant", conversation_history_for_summary_[i].content.c_str()});
        }
    }

    limited_history.push_back({"system", system_prompt_base_});    
    limited_history.push_back({"user", summary_prompt_});

    const char* tmpl = llama_model_chat_template(model_, NULL);
    std::vector<llama_chat_message> llama_messages;
    
    for (const auto& msg : limited_history) {
        llama_chat_message llm_msg;
        llm_msg.role = msg.role.c_str();
        llm_msg.content = msg.content.c_str();
        llama_messages.push_back(llm_msg);
    }
    
    std::vector<char> formatted(llama_n_ctx(ctx_));
    int len = llama_chat_apply_template(tmpl, llama_messages.data(), llama_messages.size(), true, formatted.data(), formatted.size());
    
    if (len < 0) {
        RCLCPP_INFO(get_logger(), "Failed to apply the chat template for summary %d", len);
        return "";
    }
    
    std::string prompt(formatted.begin(), formatted.begin() + len);
    
    llama_kv_self_clear(ctx_);
    
    const int n_prompt_tokens = -llama_tokenize(vocab_, prompt.c_str(), prompt.size(), NULL, 0, true, true);
    std::vector<llama_token> prompt_tokens(n_prompt_tokens);
    if (llama_tokenize(vocab_, prompt.c_str(), prompt.size(), prompt_tokens.data(), prompt_tokens.size(), true, true) < 0) {
        RCLCPP_INFO(get_logger(), "Failed to tokenize the summary prompt");
        return "";
    }
    
    llama_batch batch = llama_batch_get_one(prompt_tokens.data(), prompt_tokens.size());
    std::string summary;
    
    llama_token new_token_id;
    int max_tokens = static_cast<int>(CONFIGURATIONS_["max_tokens_summary"]);
    int token_count = 0;
    
    while (token_count < max_tokens) {
        if (llama_decode(ctx_, batch)) {
            RCLCPP_INFO(get_logger(), "Failed to decode for summary");
            break;
        }
        
        new_token_id = llama_sampler_sample(sampler_, ctx_, -1);
        
        if (llama_vocab_is_eog(vocab_, new_token_id)) {
            break;
        }
        
        char token_buf[static_cast<int>(CONFIGURATIONS_["token_buffer_size"])];
        int n = llama_token_to_piece(vocab_, new_token_id, token_buf, sizeof(token_buf), 0, true);
        if (n < 0) {
            RCLCPP_INFO(get_logger(), "Failed to convert token to piece in summary");
            break;
        }
        
        std::string piece(token_buf, n);
        summary += piece;
        
        batch = llama_batch_get_one(&new_token_id, 1);
        token_count++;
    }
    
    RCLCPP_INFO(get_logger(), "Summary: %s", summary.c_str());
    return summary;
}

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<yaren_chat::LLMLifecycleNode>();
    
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}