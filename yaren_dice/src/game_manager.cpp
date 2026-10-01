#include "game_manager.hpp"
#include <thread>
#include <cstdlib>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <algorithm>
#include <sstream>

using namespace std::chrono_literals;

YarenGameManager::YarenGameManager() : Node("yaren_game_manager")
{
    this->declare_parameter("use_help", false);
    use_help_ = this->get_parameter("use_help").as_bool();

    this->declare_parameter("is_session", false);
    is_session_ = this->get_parameter("is_session").as_bool();

    RCLCPP_INFO(this->get_logger(), "Waiting 2 seconds for other nodes to initialize...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(this->get_logger(), "Starting game manager... Modo Ayuda: %s | Modo Sesión: %s", 
                use_help_ ? "ACTIVADO" : "DESACTIVADO", is_session_ ? "ACTIVADO" : "DESACTIVADO");
    
    feedback_publisher_ = this->create_publisher<std_msgs::msg::String>("/game_feedback", 10);
    current_challenge_publisher_ = this->create_publisher<std_msgs::msg::Int16>("/current_challenge", 10);
    trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
    ui_state_publisher_ = this->create_publisher<std_msgs::msg::String>("/yaren/ui_state", 10);
    
    pose_result_subscription_ = this->create_subscription<yaren_interfaces::msg::PoseResult>(
        "/pose_result", 10, std::bind(&YarenGameManager::handle_pose_result, this, std::placeholders::_1));
    
    audio_status_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/audio_playing", 10, std::bind(&YarenGameManager::handle_audio_status, this, std::placeholders::_1));

    emotion_subscription_ = this->create_subscription<std_msgs::msg::Int16>(
        "/emotion", 10, std::bind(&YarenGameManager::handle_emotion, this, std::placeholders::_1));

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
    session_aborted_ = false;

    pending_detection_start_time_ = 0.0;
    has_pending_robot_pose_ = false;
    robot_moving_ = false;
    robot_move_end_time_ = 0.0;

    total_attempts_ = 0;
    successful_attempts_ = 0;
    total_emotion_readings_ = 0;
    emotion_counts_[3] = 0;
    emotion_counts_[4] = 0;
    emotion_counts_[5] = 0;
    emotion_counts_[6] = 0;

    if (use_help_ || is_session_) {
        lives_ = 5; 
        load_challenges_robot_from_yaml();
    } else {
        lives_ = 3;
        load_challenges_from_yaml();
        load_intermediate_challenges_from_yaml();
        load_advanced_challenges_from_yaml();
    }
    
    victory_texts_es_ = {" ¡Muy bien! Has completado el desafío. Tu puntuación es ", " Increíble, has superado el desafío. Tu puntaje actual es ", " ¡Fantástico! Has logrado el desafío. Tu puntuación es "};
    victory_texts_en_ = {" Very good! You completed the challenge. Your score is ", " Incredible, you passed the challenge. Your current score is ", " Fantastic! You achieved the challenge. Your score is "};
    defeat_texts_es_ = {" ¡Oh no! Has fallado el desafío, no te preocupes, puedes intentarlo de nuevo. Tienes ", " Desafortunadamente, no has logrado el desafío, se que a la próxima lo harás mejor. Actualmente te quedan ", " No te preocupes puedes intentarlo de nuevo. Te quedan "};
    defeat_texts_en_ = {" Oh no! You failed the challenge, don't worry, you can try again. You have ", " Unfortunately, you didn't achieve the challenge, I know you'll do better next time. Currently you have ", " Don't worry, you can try again. You have "};
    
    challenge_timer_ = this->create_wall_timer(
        500ms, std::bind(&YarenGameManager::check_challenge_timeout, this));

    std::thread([]() {
        std::system("ros2 run yaren_dice pose_detector &");
    }).detach();
}

void YarenGameManager::handle_emotion(const std_msgs::msg::Int16::SharedPtr msg)
{
    if (game_initialized_) {
        emotion_counts_[msg->data]++;
        total_emotion_readings_++;
    }
}

void YarenGameManager::handle_language_change(const std_msgs::msg::Bool::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(language_mutex_);
    bool new_is_english = msg->data;
    
    if (game_initialized_ && new_is_english == is_english_) return;

    is_english_ = new_is_english;

    if (!game_initialized_)
    {
        game_initialized_ = true;
        show_intro_screen(); // Muestra introducción y bloquea hasta dar clic
        
        if (is_session_) {
            show_control_panel(); // Lanza panel de interrupción para el especialista
        }
        
        game_start_time_  = std::chrono::steady_clock::now();
        select_challenge();
    }
    else
    {
        auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - game_start_time_).count();
        if (elapsed < 3.0) return;

        int start_lives = (use_help_ || is_session_) ? 5 : 3;
        if (score_ == 0 && lives_ == start_lives)
        {
            waiting_for_pose_  = false;
            detection_ongoing_ = false;
            robot_moving_ = false;
            robot_move_end_time_ = 0.0;
            select_challenge();
        }
    }
}

void YarenGameManager::show_intro_screen()
{
    std::string win_name = "Yaren Dice - Intro";
    cv::namedWindow(win_name, cv::WINDOW_NORMAL);
    cv::setWindowProperty(win_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::setWindowProperty(win_name, cv::WND_PROP_TOPMOST, 1);

    IntroData idata {false, cv::Rect(300, 370, 200, 52)};

    cv::setMouseCallback(win_name, [](int event, int x, int y, int, void* userdata) {
        if (event == cv::EVENT_LBUTTONDOWN) {
            IntroData* data = static_cast<IntroData*>(userdata);
            if (data->btn.contains(cv::Point(x, y))) {
                data->clicked = true;
            }
        }
    }, &idata);

    std::system("xdotool search --sync --name 'Yaren Dice - Intro' windowactivate --sync windowraise 2>/dev/null &");

    cv::Mat frame(480, 800, CV_8UC3, cv::Scalar(28, 12, 18));
    
    std::string title;
    std::vector<std::string> desc_lines;

    if (is_session_) {
        title = is_english_ ? "CLINICAL SESSION" : "SESION CLINICA";
        desc_lines = is_english_ ? 
            std::vector<std::string>{"Complete exactly 10 poses.", "Your facial expressions and concentration", "will be recorded for the clinical report."} :
            std::vector<std::string>{"Completa exactamente 10 poses.", "Tus expresiones faciales y concentracion", "seran evaluadas para el reporte clinico."};
    } else if (use_help_) {
        title = is_english_ ? "YAREN SAYS - WITH HELP" : "YAREN DICE - CON AYUDA";
        desc_lines = is_english_ ? 
            std::vector<std::string>{"Yaren will tell you and SHOW you the pose.", "Copy the robot's movement.", "You have 5 lives. Good luck!"} :
            std::vector<std::string>{"Yaren te dira y MOSTRARA la pose.", "Imita el movimiento del robot.", "Tienes 5 vidas. Buena suerte!"};
    } else {
        title = is_english_ ? "YAREN SAYS - NO HELP" : "YAREN DICE - SIN AYUDA";
        desc_lines = is_english_ ? 
            std::vector<std::string>{"Listen carefully to Yaren's instructions.", "The robot will NOT move.", "You have 3 lives and 3 difficulty levels."} :
            std::vector<std::string>{"Escucha atentamente las instrucciones.", "El robot NO se movera.", "Tienes 3 vidas y 3 niveles de dificultad."};
    }

    while (rclcpp::ok() && !idata.clicked) {
        frame.setTo(cv::Scalar(28, 12, 18));
        
        cv::rectangle(frame, cv::Rect(100, 50, 600, 390), cv::Scalar(46, 22, 30), cv::FILLED);
        cv::rectangle(frame, cv::Rect(100, 50, 600, 390), cv::Scalar(220, 80, 180), 2);

        int bl = 0;
        cv::Size ts = cv::getTextSize(title, cv::FONT_HERSHEY_DUPLEX, 1.0, 2, &bl);
        cv::putText(frame, title, cv::Point((800 - ts.width)/2, 120), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 235, 240), 2, cv::LINE_AA);

        int y_offset = 200;
        for (const auto& line : desc_lines) {
            cv::Size ls = cv::getTextSize(line, cv::FONT_HERSHEY_SIMPLEX, 0.7, 1, &bl);
            cv::putText(frame, line, cv::Point((800 - ls.width)/2, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(190, 150, 160), 1, cv::LINE_AA);
            y_offset += 35;
        }

        cv::rectangle(frame, idata.btn, cv::Scalar(160, 40, 100), cv::FILLED);
        cv::rectangle(frame, idata.btn, cv::Scalar(220, 80, 180), 2);
        
        std::string btn_lbl = is_english_ ? "START" : "COMENZAR";
        cv::Size bs = cv::getTextSize(btn_lbl, cv::FONT_HERSHEY_DUPLEX, 0.8, 2, &bl);
        cv::putText(frame, btn_lbl, cv::Point(idata.btn.x + (idata.btn.width - bs.width)/2, idata.btn.y + 34), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

        cv::imshow(win_name, frame);
        int key = cv::waitKey(16) & 0xFF;
        if (key == 27) { // ESC para salir si es necesario
            auto msg = std::make_unique<std_msgs::msg::String>();
            msg->data = "idle";
            ui_state_publisher_->publish(std::move(msg));
            rclcpp::shutdown();
            break;
        }
    }
    cv::destroyWindow(win_name);
    cv::waitKey(1);
}

void YarenGameManager::show_control_panel()
{
    std::thread([this]() {
        std::string win_name = "Control Panel";
        cv::namedWindow(win_name, cv::WINDOW_NORMAL);
        cv::resizeWindow(win_name, 320, 140);
        cv::moveWindow(win_name, 0, 0); // Panel flotante en la esquina superior izquierda
        cv::setWindowProperty(win_name, cv::WND_PROP_TOPMOST, 1);

        IntroData idata {false, cv::Rect(40, 35, 240, 60)};

        cv::setMouseCallback(win_name, [](int event, int x, int y, int, void* userdata) {
            if (event == cv::EVENT_LBUTTONDOWN) {
                IntroData* data = static_cast<IntroData*>(userdata);
                if (data->btn.contains(cv::Point(x, y))) {
                    data->clicked = true;
                }
            }
        }, &idata);

        cv::Mat frame(140, 320, CV_8UC3, cv::Scalar(35, 35, 45));
        cv::rectangle(frame, idata.btn, cv::Scalar(60, 40, 220), cv::FILLED);
        cv::rectangle(frame, idata.btn, cv::Scalar(100, 100, 255), 2, cv::LINE_AA);
        
        std::string txt = is_english_ ? "STOP SESSION NOW" : "DETENER SESION";
        int bl;
        cv::Size ts = cv::getTextSize(txt, cv::FONT_HERSHEY_DUPLEX, 0.65, 2, &bl);
        cv::putText(frame, txt, cv::Point(idata.btn.x + (idata.btn.width - ts.width)/2, idata.btn.y + 38), cv::FONT_HERSHEY_DUPLEX, 0.65, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

        while (rclcpp::ok() && !session_aborted_.load()) {
            cv::imshow(win_name, frame);
            int key = cv::waitKey(50) & 0xFF;
            
            // Si hacen clic en detener o presionan ESC en la ventanita
            if (idata.clicked || key == 27) {
                RCLCPP_INFO(this->get_logger(), "Sesión interrumpida manualmente por el especialista.");
                session_aborted_ = true; 
                break;
            }
        }
        cv::destroyWindow(win_name);
        cv::waitKey(1);
    }).detach();
}

// ---------------- CARGA DE YAMLS ----------------
void YarenGameManager::load_challenges_robot_from_yaml() {
    try {
        std::string yaml_path = ament_index_cpp::get_package_share_directory("yaren_dice") + "/config/challenges_robot.yaml";
        YAML::Node config = YAML::LoadFile(yaml_path);
        if (config["challenges"]) {
            for (const auto& challenge : config["challenges"]) robot_challenges_.push_back(challenge);
        }
    } catch (const YAML::Exception& e) { RCLCPP_ERROR(this->get_logger(), "Error YAML: %s", e.what()); }
}

void YarenGameManager::load_challenges_from_yaml() {
    try {
        std::string yaml_path = ament_index_cpp::get_package_share_directory("yaren_dice") + "/config/challenges.yaml";
        YAML::Node config = YAML::LoadFile(yaml_path);
        if (config["challenges"]) {
            for (const auto& challenge : config["challenges"]) challenges_.push_back(challenge);
        }
    } catch (const YAML::Exception& e) { RCLCPP_ERROR(this->get_logger(), "Error YAML: %s", e.what()); }
}

void YarenGameManager::load_intermediate_challenges_from_yaml() {
    try {
        std::string yaml_path = ament_index_cpp::get_package_share_directory("yaren_dice") + "/config/intermediate_challenges.yaml";
        YAML::Node config = YAML::LoadFile(yaml_path);
        if (config["intermediate_challenges"]) {
            for (const auto& challenge : config["intermediate_challenges"]) intermediate_challenges_.push_back(challenge);
        }
    } catch (const YAML::Exception& e) { RCLCPP_ERROR(this->get_logger(), "Error YAML: %s", e.what()); }
}

void YarenGameManager::load_advanced_challenges_from_yaml() {
    try {
        std::string yaml_path = ament_index_cpp::get_package_share_directory("yaren_dice") + "/config/advanced_challenges.yaml";
        YAML::Node config = YAML::LoadFile(yaml_path);
        if (config["advanced_challenges"]) {
            for (const auto& challenge : config["advanced_challenges"]) advanced_challenges_.push_back(challenge);
        }
    } catch (const YAML::Exception& e) { RCLCPP_ERROR(this->get_logger(), "Error YAML: %s", e.what()); }
}

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
    for (size_t i = 0; i < 12; ++i) point.positions.push_back(raw_pose[i]);

    point.time_from_start.sec = 2;
    point.time_from_start.nanosec = 0;

    msg.points.push_back(point);
    trajectory_publisher_->publish(msg);
    
    robot_moving_ = true;
    robot_move_end_time_ = get_current_time() + robot_move_duration_;
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
    has_pending_robot_pose_ = false;
    pending_robot_pose_.clear();
    robot_moving_ = false;
    robot_move_end_time_ = 0.0;

    std::vector<YAML::Node>* current_challenges = nullptr;
    
    if (use_help_ || is_session_) 
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
    
    if ((use_help_ || is_session_) && selected_challenge["robot_pose"]) {
        pending_robot_pose_ = selected_challenge["robot_pose"].as<std::vector<double>>();
        has_pending_robot_pose_ = true;
    }

    std::string challenge_text;
    std::string text_key = (is_english_ && selected_challenge["text_en"]) ? "text_en" : "text";

    if (use_help_ || is_session_ || current_level_ == GameLevel::BASIC)
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
    
    if (!audio_playing_ && !detection_ongoing_ && !robot_moving_)
    {
        if ((use_help_ || is_session_) && has_pending_robot_pose_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(audio_end_delay_ * 1000)));
            move_robot(pending_robot_pose_);
            has_pending_robot_pose_ = false;
            pending_robot_pose_.clear();
        }
        else if (!use_help_ && !is_session_)
        {
            start_detection();
        }
    }
    else if (audio_playing_)
    {
        pending_detection_start_time_ = 0.0;
    }
}

void YarenGameManager::start_detection()
{
    detection_ongoing_ = true;
    waiting_for_pose_ = true;
    challenge_timeout_ = get_current_time() + challenge_timeout_seconds_;
}

void YarenGameManager::check_challenge_timeout()
{
    std::lock_guard<std::mutex> lock(language_mutex_);

    // Capturamos si el especialista abortó la sesión vía el panel de control
    if (session_aborted_.load()) {
        session_aborted_ = false; // Resetear bandera para no entrar en bucle
        end_game();
        return;
    }
    
    if (robot_moving_ && get_current_time() >= robot_move_end_time_)
    {
        robot_moving_ = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(
            static_cast<int>(detection_delay_after_move_ * 1000)));
        start_detection();
    }
    
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
                
                challenge_timeout_ = get_current_time() + challenge_timeout_seconds_;
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
    RCLCPP_INFO(this->get_logger(), "Juego terminado. Calculando métricas y emitiendo reporte UI...");

    // Notificamos que la sesión cerró para que la ventana de control también se limpie
    session_aborted_ = true; 
    
    double concentration_index = 0.0;
    int success_rate = 0;
    if (total_attempts_ > 0) {
        concentration_index = (static_cast<double>(successful_attempts_) / total_attempts_) * 100.0;
        success_rate = static_cast<int>(concentration_index); 
    }

    double perc_alegria = 0, perc_tristeza = 0, perc_sorpresa = 0, perc_neutral = 0;
    int predominant_emotion_idx = 6; 
    std::string predominant_emotion_name = "NEUTRAL";

    if (total_emotion_readings_ > 0) {
        perc_alegria  = (static_cast<double>(emotion_counts_[3]) / total_emotion_readings_) * 100.0;
        perc_tristeza = (static_cast<double>(emotion_counts_[4]) / total_emotion_readings_) * 100.0;
        perc_sorpresa = (static_cast<double>(emotion_counts_[5]) / total_emotion_readings_) * 100.0;
        perc_neutral  = (static_cast<double>(emotion_counts_[6]) / total_emotion_readings_) * 100.0;

        int max_count = -1;
        for (const auto& pair : emotion_counts_) {
            if (pair.second > max_count) {
                max_count = pair.second;
                predominant_emotion_idx = pair.first;
            }
        }

        switch(predominant_emotion_idx) {
            case 3: predominant_emotion_name = "ALEGRIA"; break;
            case 4: predominant_emotion_name = "TRISTEZA"; break;
            case 5: predominant_emotion_name = "SORPRESA"; break;
            case 6: predominant_emotion_name = "NEUTRAL"; break;
        }
    }

    std::ostringstream json_payload;
    json_payload << "{"
                 << "\"action\": \"session_report\", "
                 << "\"poses_logradas\": " << successful_attempts_ << ", "
                 << "\"fallos\": " << (total_attempts_ - successful_attempts_) << ", "
                 << "\"tasa_exito\": " << success_rate << ", "
                 << "\"concentracion\": " << static_cast<int>(concentration_index) << ", "
                 << "\"alegria\": " << static_cast<int>(perc_alegria) << ", "
                 << "\"neutral\": " << static_cast<int>(perc_neutral) << ", "
                 << "\"sorpresa\": " << static_cast<int>(perc_sorpresa) << ", "
                 << "\"tristeza\": " << static_cast<int>(perc_tristeza) << ", "
                 << "\"emocion_predominante\": \"" << predominant_emotion_name << "\""
                 << "}";

    auto state_msg = std::make_unique<std_msgs::msg::String>();
    state_msg->data = json_payload.str();
    ui_state_publisher_->publish(std::move(state_msg));

    if (use_help_ || is_session_) {
        move_robot({0.0, 0.0, 0.0, 0.00, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5});
    }

    auto game_over_msg = std::make_unique<std_msgs::msg::String>();
    game_over_msg->data = is_english_ ? 
        "The session is over." :
        "La sesión ha finalizado.";
    
    feedback_publisher_->publish(std::move(game_over_msg));
    rclcpp::shutdown();
}

void YarenGameManager::handle_successful_challenge()
{
    waiting_for_pose_ = false;
    detection_ongoing_ = false;
    correct_pose_start_time_ = 0.0;
    current_sequence_step_ = 0;
    pending_detection_start_time_ = 0.0;
    robot_moving_ = false;
    robot_move_end_time_ = 0.0;
    
    score_++;
    successful_attempts_++; 
    total_attempts_++;      
    
    auto feedback_msg = std::make_unique<std_msgs::msg::String>();
    int random_index = rand() % victory_texts_es_.size();
    std::string victory_text = is_english_ ? victory_texts_en_[random_index] : victory_texts_es_[random_index];
    
    if (use_help_ || is_session_ || current_level_ == GameLevel::BASIC) {
        feedback_msg->data = victory_text + std::to_string(score_) + ".";
    } else {
        std::string prefix = is_english_ ? "Incredible! You have completed the whole sequence. " : "¡Increíble! Has completado toda la secuencia. ";
        feedback_msg->data = prefix + victory_text + std::to_string(score_) + ".";
    }
    
    feedback_publisher_->publish(std::move(feedback_msg));
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    if (use_help_ || is_session_) {
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
    pending_detection_start_time_ = 0.0;
    robot_moving_ = false;
    robot_move_end_time_ = 0.0;
    
    total_attempts_++; 

    // En sesión clínica NO descontamos vidas ni cortamos el juego
    if (is_session_) {
        auto feedback_msg = std::make_unique<std_msgs::msg::String>();
        feedback_msg->data = is_english_ ? "Don't worry, let's go with the next pose." : "No te preocupes, vamos con la siguiente pose.";
        feedback_publisher_->publish(std::move(feedback_msg));
        std::this_thread::sleep_for(std::chrono::seconds(2));

        challenges_played_++;
        if (challenges_played_ >= 10) {
            end_game();
            return;
        }
    } 
    else {
        lives_--;
        if (lives_ <= 0) {
            end_game();
            return;
        }
            
        auto feedback_msg = std::make_unique<std_msgs::msg::String>();
        std::string defeat_text = feedback_text + std::to_string(lives_);
        defeat_text += is_english_ ? (lives_ == 1 ? " attempt left." : " attempts left.") : (lives_ == 1 ? " intento." : " intentos.");
        
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