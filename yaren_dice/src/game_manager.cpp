#include "game_manager.hpp"
#include <thread>
#include <cstdlib>
using namespace std::chrono_literals;

YarenGameManager::YarenGameManager() : Node("yaren_game_manager")
{
    RCLCPP_INFO(this->get_logger(), "Waiting 2 seconds for other nodes to initialize...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(this->get_logger(), "Starting game manager...");
    
    feedback_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/game_feedback", 10);
    current_challenge_publisher_ = this->create_publisher<std_msgs::msg::Int16>(
        "/current_challenge", 10);
    
    pose_result_subscription_ = this->create_subscription<yaren_interfaces::msg::PoseResult>(
        "/pose_result", 10,
        std::bind(&YarenGameManager::handle_pose_result, this, std::placeholders::_1));
    
    audio_status_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/audio_playing", 10,
        std::bind(&YarenGameManager::handle_audio_status, this, std::placeholders::_1));

    // Suscripción para saber en qué idioma está el robot
    rclcpp::QoS qos_profile(1);
    qos_profile.transient_local();
    language_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/yaren/is_english", qos_profile,
        std::bind(&YarenGameManager::handle_language_change, this, std::placeholders::_1));
    
    current_challenge_ = 0;
    score_ = 0;
    lives_ = 3;
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
    game_initialized_ = false;  // FIX: el juego no arranca hasta recibir el idioma

    load_challenges_from_yaml();
    load_intermediate_challenges_from_yaml();
    load_advanced_challenges_from_yaml();
    
    victory_texts_es_ = {
        " ¡Muy bien! Has completado el desafío. Tu puntuación es ",
        " Increíble, has superado el desafío. Tu puntaje actual es ",
        " ¡Fantástico! Has logrado el desafío. Tu puntuación es ",
        " Que bien lo hiciste! Has completado el desafío. Tu puntuación es ",
        " Eres el mejor jugador del mundo, sigue asi! Tu puntuación es ",
        " ¡Impresionante! Has superado el desafío. Tu puntuación es "
    };

    victory_texts_en_ = {
        " Very good! You completed the challenge. Your score is ",
        " Incredible, you passed the challenge. Your current score is ",
        " Fantastic! You achieved the challenge. Your score is ",
        " You did great! You completed the challenge. Your score is ",
        " You are the best player in the world, keep it up! Your score is ",
        " Awesome! You passed the challenge. Your score is "
    };

    defeat_texts_es_ = {
        " ¡Oh no! Has fallado el desafío, no te preocupes, puedes intentarlo de nuevo. Tienes ",
        " Desafortunadamente, no has logrado el desafío, se que a la próxima lo harás mejor. Actualmente te quedan ",
        " No te preocupes puedes intentarlo de nuevo, nadie te quitara tu puesto de campeon. Te quedan ",
        " No te desanimes sigue practicando, la practica hace al maestro. Te quedan ",
        " No te rindas! Puedes hacerlo mejor, todos fallamos alguna vez. Tienes "
    };

    defeat_texts_en_ = {
        " Oh no! You failed the challenge, don't worry, you can try again. You have ",
        " Unfortunately, you didn't achieve the challenge, I know you'll do better next time. Currently you have ",
        " Don't worry, you can try again, no one will take your champion spot. You have ",
        " Don't be discouraged, keep practicing, practice makes perfect. You have ",
        " Don't give up! You can do better, we all fail sometimes. You have "
    };
    
    challenge_timer_ = this->create_wall_timer(
        500ms, std::bind(&YarenGameManager::check_challenge_timeout, this));

    // FIX: NO llamamos select_challenge() ni start_detection() aquí.
    // El juego arranca en handle_language_change() cuando llega el idioma correcto.
    std::thread([]() {
        std::system("ros2 run yaren_dice pose_detector &");
    }).detach();
    RCLCPP_INFO(this->get_logger(), "Game manager ready, waiting for language topic...");
}

void YarenGameManager::handle_language_change(const std_msgs::msg::Bool::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(language_mutex_);
    
    bool new_is_english = msg->data;
    
    // Ignorar si el idioma no cambió realmente
    if (game_initialized_ && new_is_english == is_english_) return;

    is_english_ = new_is_english;
    RCLCPP_INFO(this->get_logger(), "Game manager language updated to: %s",
                is_english_ ? "English" : "Español");

    if (!game_initialized_)
    {
        game_initialized_ = true;
        game_start_time_  = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Starting game in correct language...");
        select_challenge();
        start_detection();
        RCLCPP_INFO(this->get_logger(), "Game started");
    }
    else
    {
        // Cambio real de idioma — solo resetear si no hay progreso
        // Y solo si pasaron al menos 3 segundos desde el inicio
        // (evita el doble-disparo del TRANSIENT_LOCAL)
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - game_start_time_).count();
            
        if (elapsed < 3.0)
        {
            RCLCPP_INFO(this->get_logger(),
                "Ignorando cambio de idioma transitorio (%.1fs desde inicio)", elapsed);
            return;
        }

        if (score_ == 0 && lives_ == 3)
        {
            RCLCPP_INFO(this->get_logger(),
                "Language changed before game progress, re-selecting challenge...");
            waiting_for_pose_  = false;
            detection_ongoing_ = false;
            select_challenge();
            start_detection();
        }
        else
        {
            // Juego en progreso: solo cambiar textos, no resetear
            RCLCPP_INFO(this->get_logger(),
                "Language changed mid-game, updating texts only.");
        }
    }
}
void YarenGameManager::load_challenges_from_yaml()
{
    try
    {
        std::string package_path = ament_index_cpp::get_package_share_directory("yaren_dice");
        std::string yaml_path = package_path + "/config/challenges.yaml";
        YAML::Node config = YAML::LoadFile(yaml_path);
        if (config["challenges"])
        {
            for (const auto& challenge : config["challenges"])
            {
                challenges_.push_back(challenge);
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "No challenges found in the YAML file.");
        }
    }
    catch (const YAML::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load challenges from YAML: %s", e.what());
    }
}

void YarenGameManager::load_intermediate_challenges_from_yaml()
{
    try
    {
        std::string package_path = ament_index_cpp::get_package_share_directory("yaren_dice");
        std::string yaml_path = package_path + "/config/intermediate_challenges.yaml";
        YAML::Node config = YAML::LoadFile(yaml_path);
        if (config["intermediate_challenges"])
        {
            for (const auto& challenge : config["intermediate_challenges"])
            {
                intermediate_challenges_.push_back(challenge);
            }
            RCLCPP_INFO(this->get_logger(), "Loaded %zu intermediate challenges", intermediate_challenges_.size());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "No intermediate challenges found in the YAML file.");
        }
    }
    catch (const YAML::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load intermediate challenges from YAML: %s", e.what());
    }
}

void YarenGameManager::load_advanced_challenges_from_yaml()
{
    try
    {
        std::string package_path = ament_index_cpp::get_package_share_directory("yaren_dice");
        std::string yaml_path = package_path + "/config/advanced_challenges.yaml";
        YAML::Node config = YAML::LoadFile(yaml_path);
        if (config["advanced_challenges"])
        {
            for (const auto& challenge : config["advanced_challenges"])
            {
                advanced_challenges_.push_back(challenge);
            }
            RCLCPP_INFO(this->get_logger(), "Loaded %zu advanced challenges", advanced_challenges_.size());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "No advanced challenges found in the YAML file.");
        }
    }
    catch (const YAML::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load advanced challenges from YAML: %s", e.what());
    }
}

GameLevel YarenGameManager::get_current_level()
{
    if (score_ >= 15) return GameLevel::ADVANCED;
    else if (score_ >= 7) return GameLevel::INTERMEDIATE;
    else return GameLevel::BASIC;
}

void YarenGameManager::announce_level_up(GameLevel new_level)
{
    auto feedback_msg = std::make_unique<std_msgs::msg::String>();
    if (new_level == GameLevel::INTERMEDIATE) {
        feedback_msg->data = is_english_ ? 
            "Congratulations! You have reached the intermediate level. Now you will have to do sequences of up to 3 movements." : 
            "¡Felicidades! Has alcanzado el nivel intermedio. Ahora tendrás que hacer secuencias de hasta 3 movimientos.";
    } else if (new_level == GameLevel::ADVANCED) {
        feedback_msg->data = is_english_ ? 
            "Incredible! You have reached the advanced level. Get ready for sequences of up to 5 movements." : 
            "¡Increíble! Has llegado al nivel avanzado. Prepárate para secuencias de hasta 5 movimientos.";
    } else {
        return;
    }
    
    feedback_publisher_->publish(std::move(feedback_msg));
    RCLCPP_INFO(this->get_logger(), "Level up announced: %d", static_cast<int>(new_level));
}

void YarenGameManager::select_challenge()
{
    GameLevel new_level = get_current_level();
    
    if (new_level != current_level_)
    {
        announce_level_up(new_level);
        current_level_ = new_level;
        std::this_thread::sleep_for(std::chrono::seconds(3)); 
    }
    
    std::vector<YAML::Node>* current_challenges = nullptr;
    
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
    
    if (current_challenges->empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No challenges available for current level.");
        return;
    }
    
    int random_index = rand() % current_challenges->size();
    YAML::Node selected_challenge = (*current_challenges)[random_index];
    
    std::string challenge_text;
    std::string text_key = (is_english_ && selected_challenge["text_en"]) ? "text_en" : "text";

    if (current_level_ == GameLevel::BASIC)
    {
        current_challenge_ = selected_challenge["id"].as<int16_t>();
        current_sequence_.clear();
        current_sequence_.push_back(current_challenge_);
        expected_sequence_length_ = 1;
        current_sequence_step_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Selected basic challenge ID: %d", current_challenge_);
        
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
        
        RCLCPP_INFO(this->get_logger(), "Selected sequence challenge with %d poses", expected_sequence_length_);
        
        auto challenge_msg = std::make_unique<std_msgs::msg::Int16>();
        challenge_msg->data = current_challenge_;
        current_challenge_publisher_->publish(std::move(challenge_msg));
        
        challenge_text = selected_challenge[text_key].as<std::string>();
    }

    // Siempre publicamos el texto del desafio al TTS, en todos los idiomas y niveles.
    auto feedback_msg = std::make_unique<std_msgs::msg::String>();
    feedback_msg->data = challenge_text;
    feedback_publisher_->publish(std::move(feedback_msg));
    
    RCLCPP_INFO(this->get_logger(), "Challenge text: %s", challenge_text.c_str());
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
    RCLCPP_INFO(this->get_logger(), "Detection phase started");
}

void YarenGameManager::check_challenge_timeout()
{
    std::lock_guard<std::mutex> lock(language_mutex_);
    if (!waiting_for_pose_ || challenge_timeout_ == 0.0) return;
    
    if (get_current_time() > challenge_timeout_)
    {
        RCLCPP_INFO(this->get_logger(), "Challenge timed out");
        int random_index = rand() % defeat_texts_es_.size();
        std::string defeat_text = is_english_ ?
            defeat_texts_en_[random_index] : defeat_texts_es_[random_index];
        handle_failed_challenge(defeat_text);
    }
}

void YarenGameManager::handle_pose_result(const yaren_interfaces::msg::PoseResult::SharedPtr msg){        
    std::lock_guard<std::mutex> lock(language_mutex_);
    if (!waiting_for_pose_ || audio_playing_) return;
            
    int received_challenge = msg->challenge;
    bool detected_poses = msg->detected_poses;
    
    if (received_challenge != current_challenge_)
    {
        RCLCPP_WARN(this->get_logger(), "Received challenge ID %d, but expected %d", received_challenge, current_challenge_);
        return;
    }
    
    if (detected_poses)
    {
        RCLCPP_INFO(this->get_logger(), "Correct pose detected for step %d/%d", current_sequence_step_ + 1, expected_sequence_length_);

        if (correct_pose_start_time_ == 0.0)
        {
            correct_pose_start_time_ = get_current_time();
            RCLCPP_INFO(this->get_logger(), "Correct pose detected, starting timer");
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
                
                RCLCPP_INFO(this->get_logger(), "Moving to next pose in sequence: %d", current_challenge_);
                challenge_timeout_ = get_current_time() + 20.0;
            }
        }
    }
    else
    {
        correct_pose_start_time_ = 0.0;
    }
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
    
    if (current_level_ == GameLevel::BASIC)
    {
        feedback_msg->data = victory_text + std::to_string(score_) + ".";
    }
    else
    {
        std::string prefix = is_english_ ? "Incredible! You have completed the whole sequence. " : "¡Increíble! Has completado toda la secuencia. ";
        feedback_msg->data = prefix + victory_text + std::to_string(score_) + ".";
    }
    
    feedback_publisher_->publish(std::move(feedback_msg));
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
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
        RCLCPP_INFO(this->get_logger(), "Juego terminado.");
        auto game_over_msg = std::make_unique<std_msgs::msg::String>();
        game_over_msg->data = is_english_ ? 
            "It was really fun playing with you, the game is over. Your final score is " + std::to_string(score_) + "." :
            "Ha sido muy divertido jugar contigo, el juego termino. Tu puntuación final es " + std::to_string(score_) + ".";
        
        feedback_publisher_->publish(std::move(game_over_msg));
        rclcpp::shutdown();
        return;
    }
        
    auto feedback_msg = std::make_unique<std_msgs::msg::String>();
    std::string defeat_text = feedback_text + std::to_string(lives_);
    
    if (lives_ == 1) {
        defeat_text += is_english_ ? " attempt left." : " intento.";
    } else {
        defeat_text += is_english_ ? " attempts left." : " intentos.";
    }
    
    feedback_msg->data = defeat_text;
    feedback_publisher_->publish(std::move(feedback_msg));
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
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