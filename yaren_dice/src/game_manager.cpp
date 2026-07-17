#include "game_manager.hpp"
#include <thread>
#include <cstdlib>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>

using namespace std::chrono_literals;

YarenGameManager::YarenGameManager() : Node("yaren_game_manager")
{
    // Declarar y leer el parámetro del modo de juego
    this->declare_parameter("use_help", false);
    use_help_ = this->get_parameter("use_help").as_bool();

    RCLCPP_INFO(this->get_logger(), "Waiting 2 seconds for other nodes to initialize...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(this->get_logger(), "Starting game manager... Modo Ayuda: %s", use_help_ ? "ACTIVADO" : "DESACTIVADO");
    
    feedback_publisher_ = this->create_publisher<std_msgs::msg::String>("/game_feedback", 10);
    current_challenge_publisher_ = this->create_publisher<std_msgs::msg::Int16>("/current_challenge", 10);
    
    // Publisher para mover el robot
    trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 10);
    
    pose_result_subscription_ = this->create_subscription<yaren_interfaces::msg::PoseResult>(
        "/pose_result", 10, std::bind(&YarenGameManager::handle_pose_result, this, std::placeholders::_1));
    
    audio_status_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/audio_playing", 10, std::bind(&YarenGameManager::handle_audio_status, this, std::placeholders::_1));

    rclcpp::QoS qos_profile(1);
    qos_profile.transient_local();
    language_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/yaren/is_english", qos_profile, std::bind(&YarenGameManager::handle_language_change, this, std::placeholders::_1));
    
    current_challenge_ = 0;
    score_ = 0;
    challenges_played_ = 0; 
    level_score_ = 0;
    audio_playing_ = false;
    detection_ongoing_ = false;
    challenge_timeout_ = 0.0;
    waiting_for_pose_ = false;
    correct_pose_start_time_ = 0.0;
    correct_pose_duration_ = 0.5;
    current_level_ = GameLevel::BASIC;
    current_sequence_step_ = 0;
    expected_sequence_length_ = 1;
    is_english_ = false;
    game_initialized_ = false;  

    // Lógica condicional de carga y vidas
    if (use_help_) {
        lives_ = 5;
        load_challenges_robot_from_yaml();
    } else {
        lives_ = 3;
        load_challenges_from_yaml();
        load_intermediate_challenges_from_yaml();
        load_advanced_challenges_from_yaml();
    }
    
    victory_texts_es_ = {
        " ¡Muy bien! Has completado el desafío. Tu puntuación es ",
        " Increíble, has superado el desafío. Tu puntaje actual es ",
        " ¡Fantástico! Has logrado el desafío. Tu puntuación es "
    };

    victory_texts_en_ = {
        " Very good! You completed the challenge. Your score is ",
        " Incredible, you passed the challenge. Your current score is ",
        " Fantastic! You achieved the challenge. Your score is "
    };

    defeat_texts_es_ = {
        " ¡Oh no! Has fallado el desafío, no te preocupes, puedes intentarlo de nuevo. Tienes ",
        " Desafortunadamente, no has logrado el desafío, se que a la próxima lo harás mejor. Actualmente te quedan ",
        " No te preocupes puedes intentarlo de nuevo. Te quedan "
    };

    defeat_texts_en_ = {
        " Oh no! You failed the challenge, don't worry, you can try again. You have ",
        " Unfortunately, you didn't achieve the challenge, I know you'll do better next time. Currently you have ",
        " Don't worry, you can try again. You have "
    };
    
    challenge_timer_ = this->create_wall_timer(
        500ms, std::bind(&YarenGameManager::check_challenge_timeout, this));

    // Despierta el detector de posturas en Python
    std::thread([]() {
        std::system("ros2 run yaren_dice pose_detector &");
    }).detach();
}

void YarenGameManager::handle_language_change(const std_msgs::msg::Bool::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(language_mutex_);
    bool new_is_english = msg->data;
    
    if (game_initialized_ && new_is_english == is_english_) return;

    is_english_ = new_is_english;
    RCLCPP_INFO(this->get_logger(), "Game manager language updated to: %s", is_english_ ? "English" : "Español");

    if (!game_initialized_)
    {
        game_initialized_ = true;
        game_start_time_  = std::chrono::steady_clock::now();
        select_challenge();
        start_detection();
    }
    else
    {
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - game_start_time_).count();
            
        if (elapsed < 3.0) return;

        // Reset solo si no ha avanzado nada
        int start_lives = use_help_ ? 5 : 3;
        if (score_ == 0 && lives_ == start_lives)
        {
            waiting_for_pose_  = false;
            detection_ongoing_ = false;
            select_challenge();
            start_detection();
        }
    }
}

// ---------------- CARGA DE YAMLS ----------------

void YarenGameManager::load_challenges_robot_from_yaml()
{
    try {
        std::string yaml_path = ament_index_cpp::get_package_share_directory("yaren_dice") + "/config/challenges_robot.yaml";
        YAML::Node config = YAML::LoadFile(yaml_path);
        if (config["challenges"]) {
            for (const auto& challenge : config["challenges"]) robot_challenges_.push_back(challenge);
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error YAML: %s", e.what());
    }
}

void YarenGameManager::load_challenges_from_yaml()
{
    try {
        std::string yaml_path = ament_index_cpp::get_package_share_directory("yaren_dice") + "/config/challenges.yaml";
        YAML::Node config = YAML::LoadFile(yaml_path);
        if (config["challenges"]) {
            for (const auto& challenge : config["challenges"]) challenges_.push_back(challenge);
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error YAML: %s", e.what());
    }
}

void YarenGameManager::load_intermediate_challenges_from_yaml()
{
    try {
        std::string yaml_path = ament_index_cpp::get_package_share_directory("yaren_dice") + "/config/intermediate_challenges.yaml";
        YAML::Node config = YAML::LoadFile(yaml_path);
        if (config["intermediate_challenges"]) {
            for (const auto& challenge : config["intermediate_challenges"]) intermediate_challenges_.push_back(challenge);
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error YAML: %s", e.what());
    }
}

void YarenGameManager::load_advanced_challenges_from_yaml()
{
    try {
        std::string yaml_path = ament_index_cpp::get_package_share_directory("yaren_dice") + "/config/advanced_challenges.yaml";
        YAML::Node config = YAML::LoadFile(yaml_path);
        if (config["advanced_challenges"]) {
            for (const auto& challenge : config["advanced_challenges"]) advanced_challenges_.push_back(challenge);
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error YAML: %s", e.what());
    }
}

// ---------------- FIN CARGA DE YAMLS ----------------

void YarenGameManager::move_robot(const std::vector<double>& raw_pose)
{
    if (raw_pose.size() != 12) return;

    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = {
        "joint_1","joint_2","joint_3","joint_4",
        "joint_5","joint_6","joint_7","joint_8",
        "joint_9","joint_10","joint_11","joint_12"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (size_t i = 0; i < 12; ++i) {
        point.positions.push_back(raw_pose[i]);  // sin conversión
    }

    point.time_from_start.sec = 2;
    point.time_from_start.nanosec = 0;

    msg.points.push_back(point);
    trajectory_publisher_->publish(msg);
}

void YarenGameManager::announce_level_up(GameLevel new_level)
{
    auto feedback_msg = std::make_unique<std_msgs::msg::String>();
    if (new_level == GameLevel::INTERMEDIATE) {
        feedback_msg->data = is_english_ ? 
            "You reached 10 points! Do you want to continue with harder challenges? Let's go to the intermediate level, I give you 2 extra lives." : 
            "¡Llegaste a 10 puntos! ¿Quieres seguir jugando con desafíos más difíciles? Pasemos al nivel intermedio, te regalo 2 vidas extra.";
    } else if (new_level == GameLevel::ADVANCED) {
        feedback_msg->data = is_english_ ? 
            "Incredible! 10 more points! Ready for the advanced level? Take an extra life." : 
            "¡Increíble! ¡10 puntos más! ¿Listo para el nivel avanzado? Toma una vida extra.";
    } else {
        return;
    }
    feedback_publisher_->publish(std::move(feedback_msg));
}

void YarenGameManager::select_challenge()
{
    std::vector<YAML::Node>* current_challenges = nullptr;
    
    if (use_help_) 
    {
        current_challenges = &robot_challenges_;
        expected_sequence_length_ = 1;
    } 
    else 
    {
        switch (current_level_)
        {
            case GameLevel::BASIC:
                current_challenges = &challenges_;
                expected_sequence_length_ = 1;
                break;
            case GameLevel::INTERMEDIATE:
                current_challenges = &intermediate_challenges_;
                break;
            case GameLevel::ADVANCED:
                current_challenges = &advanced_challenges_;
                break;
        }
    }
    
    int random_index = rand() % current_challenges->size();
    YAML::Node selected_challenge = (*current_challenges)[random_index];
    
    // Mover al robot si estamos en el modo 'con ayuda'
    if (use_help_ && selected_challenge["robot_pose"]) {
        std::vector<double> raw_pose = selected_challenge["robot_pose"].as<std::vector<double>>();
        move_robot(raw_pose);
    }

    std::string challenge_text;
    std::string text_key = (is_english_ && selected_challenge["text_en"]) ? "text_en" : "text";

    if (use_help_ || current_level_ == GameLevel::BASIC)
    {
        current_challenge_ = selected_challenge["id"].as<int16_t>();
        current_sequence_.clear();
        current_sequence_.push_back(current_challenge_);
        expected_sequence_length_ = 1;
        current_sequence_step_ = 0;
        
        auto challenge_msg = std::make_unique<std_msgs::msg::Int16>();
        challenge_msg->data = current_challenge_;
        current_challenge_publisher_->publish(std::move(challenge_msg));

        std::vector<std::string> texts = selected_challenge[text_key].as<std::vector<std::string>>();
        challenge_text = texts[rand() % texts.size()];
    }
    else
    {
        current_sequence_ = selected_challenge["poses"].as<std::vector<int>>();
        expected_sequence_length_ = selected_challenge["sequence_length"].as<int>();
        current_sequence_step_ = 0;
        current_challenge_ = current_sequence_[0]; 
        
        auto challenge_msg = std::make_unique<std_msgs::msg::Int16>();
        challenge_msg->data = current_challenge_;
        current_challenge_publisher_->publish(std::move(challenge_msg));
        
        challenge_text = selected_challenge[text_key].as<std::string>();
    }

    auto feedback_msg = std::make_unique<std_msgs::msg::String>();
    feedback_msg->data = challenge_text;
    feedback_publisher_->publish(std::move(feedback_msg));
}

void YarenGameManager::handle_audio_status(const std_msgs::msg::Bool::SharedPtr msg)
{
    audio_playing_ = msg->data;
    if (!audio_playing_ && !detection_ongoing_) start_detection();
}

void YarenGameManager::start_detection()
{
    detection_ongoing_ = true;
    waiting_for_pose_ = true;
    challenge_timeout_ = get_current_time() + 20.0;
}

void YarenGameManager::check_challenge_timeout()
{
    std::lock_guard<std::mutex> lock(language_mutex_);
    if (!waiting_for_pose_ || challenge_timeout_ == 0.0) return;
    
    if (get_current_time() > challenge_timeout_)
    {
        int random_index = rand() % defeat_texts_es_.size();
        std::string defeat_text = is_english_ ? defeat_texts_en_[random_index] : defeat_texts_es_[random_index];
        handle_failed_challenge(defeat_text);
    }
}

void YarenGameManager::handle_pose_result(const yaren_interfaces::msg::PoseResult::SharedPtr msg){        
    std::lock_guard<std::mutex> lock(language_mutex_);
    if (!waiting_for_pose_ || audio_playing_) return;
            
    int received_challenge = msg->challenge;
    bool detected_poses = msg->detected_poses;
    
    if (received_challenge != current_challenge_) return;
    
    if (detected_poses)
    {
        if (correct_pose_start_time_ == 0.0)
        {
            correct_pose_start_time_ = get_current_time();
        }
        else if (get_current_time() - correct_pose_start_time_ >= correct_pose_duration_)
        {
            current_sequence_step_++;
            correct_pose_start_time_ = 0.0;
            
            if (current_sequence_step_ >= expected_sequence_length_)
            {
                handle_successful_challenge();
            }
            else
            {
                current_challenge_ = current_sequence_[current_sequence_step_];
                auto challenge_msg = std::make_unique<std_msgs::msg::Int16>();
                challenge_msg->data = current_challenge_;
                current_challenge_publisher_->publish(std::move(challenge_msg));
                
                auto feedback_msg = std::make_unique<std_msgs::msg::String>();
                feedback_msg->data = is_english_ ? "Good! Now the next pose in the sequence." : "¡Bien! Ahora la siguiente pose de la secuencia.";
                feedback_publisher_->publish(std::move(feedback_msg));
                
                challenge_timeout_ = get_current_time() + 20.0;
            }
        }
    }
    else
    {
        correct_pose_start_time_ = 0.0;
    }
}

void YarenGameManager::end_game()
{
    RCLCPP_INFO(this->get_logger(), "Juego terminado.");
    
    // Devolvemos el robot a la postura inicial si estábamos en modo ayuda
    if (use_help_) {
        move_robot({0.0, 0.0, 0.0, 0.00, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5});
    }

    auto game_over_msg = std::make_unique<std_msgs::msg::String>();
    game_over_msg->data = is_english_ ? 
        "It was really fun playing with you, the game is over. Your final score is " + std::to_string(score_) + "." :
        "Ha sido muy divertido jugar contigo, el juego termino. Tu puntuación final es " + std::to_string(score_) + ".";
    
    feedback_publisher_->publish(std::move(game_over_msg));
    rclcpp::shutdown();
}

void YarenGameManager::handle_successful_challenge()
{
    waiting_for_pose_ = false;
    detection_ongoing_ = false;
    correct_pose_start_time_ = 0.0;
    current_sequence_step_ = 0;
    score_++;
    
    auto feedback_msg = std::make_unique<std_msgs::msg::String>();
    int random_index = rand() % victory_texts_es_.size();
    std::string victory_text = is_english_ ? victory_texts_en_[random_index] : victory_texts_es_[random_index];
    
    if (use_help_ || current_level_ == GameLevel::BASIC) {
        feedback_msg->data = victory_text + std::to_string(score_) + ".";
    } else {
        std::string prefix = is_english_ ? "Incredible! You have completed the whole sequence. " : "¡Increíble! Has completado toda la secuencia. ";
        feedback_msg->data = prefix + victory_text + std::to_string(score_) + ".";
    }
    
    feedback_publisher_->publish(std::move(feedback_msg));
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // LÓGICA DE CONTINUACIÓN DEPENDIENDO DEL MODO
    if (use_help_) {
        challenges_played_++;
        if (challenges_played_ >= 10) {
            end_game();
            return;
        }
    } else {
        level_score_++;
        if (level_score_ >= 10) {
            level_score_ = 0;
            if (current_level_ == GameLevel::BASIC) {
                current_level_ = GameLevel::INTERMEDIATE;
                lives_ += 2;
                announce_level_up(current_level_);
                std::this_thread::sleep_for(std::chrono::seconds(6)); 
            } else if (current_level_ == GameLevel::INTERMEDIATE) {
                current_level_ = GameLevel::ADVANCED;
                lives_ += 1;
                announce_level_up(current_level_);
                std::this_thread::sleep_for(std::chrono::seconds(6)); 
            } else if (current_level_ == GameLevel::ADVANCED) {
                end_game();
                return;
            }
        }
    }

    select_challenge();
}

void YarenGameManager::handle_failed_challenge(const std::string& feedback_text)
{
    waiting_for_pose_ = false;
    detection_ongoing_ = false;
    correct_pose_start_time_ = 0.0;
    current_sequence_step_ = 0;
    lives_--;

    if (lives_ <= 0)
    {
        end_game();
        return;
    }
        
    auto feedback_msg = std::make_unique<std_msgs::msg::String>();
    std::string defeat_text = feedback_text + std::to_string(lives_);
    defeat_text += is_english_ ? (lives_ == 1 ? " attempt left." : " attempts left.") 
                               : (lives_ == 1 ? " intento." : " intentos.");
    
    feedback_msg->data = defeat_text;
    feedback_publisher_->publish(std::move(feedback_msg));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    if (use_help_) {
        challenges_played_++;
        if (challenges_played_ >= 10) {
            end_game();
            return;
        }
    }

    select_challenge();
}

double YarenGameManager::get_current_time()
{
    return static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count()) / 1000.0;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YarenGameManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}