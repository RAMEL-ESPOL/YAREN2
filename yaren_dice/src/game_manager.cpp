#include "game_manager.hpp"
#include <thread>
#include <cstdlib>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

// ══════════════════════════════════════════════════════════════════════════════
//  OnScreenKeyboard — Teclado QWERTY táctil para la pantalla de Yaren
// ══════════════════════════════════════════════════════════════════════════════
class OnScreenKeyboard {
public:
    std::string text;
    bool visible { false };
    std::string prompt;
    bool isPassword_ { false };

    bool showNumbers_ { false };
    bool shiftActive_ { false };
    int  hovKey_      { -1 };
    cv::Point hovPt_  { 0, 0 };
    std::vector<cv::Rect> keyRects_;
    cv::Rect btnBackspace_{0,0,0,0}, btnNumToggle_{0,0,0,0};
    cv::Rect btnOk_{0,0,0,0}, btnCancel_{0,0,0,0};
    cv::Rect btnSpace_{0,0,0,0}, btnShift_{0,0,0,0};
    bool confirmed = false;
    bool cancelled = false;

    void show(const std::string& promptText, const std::string& initial = "") {
        prompt  = promptText;
        text    = initial;
        visible = true;
        showNumbers_ = false;
        hovKey_  = -1;
        confirmed = false;
        cancelled = false;
    }

    void hide() { visible = false; }

    const std::vector<std::vector<std::string>> ROWS_ALPHA = {
        {"q","w","e","r","t","y","u","i","o","p"},
        {"a","s","d","f","g","h","j","k","l"},
        {"z","x","c","v","b","n","m"},
        {}  // fila de controles
    };

    const std::vector<std::vector<std::string>> ROWS_NUM = {
        {"1","2","3","4","5","6","7","8","9","0"},
        {"-","/",":",";","(",")","$","&","@","\""},
        {".","_","#","!","?","=","+","<",">"},
        {}
    };

    const std::vector<std::vector<std::string>>& getCurrentKeys() {
        return showNumbers_ ? ROWS_NUM : ROWS_ALPHA;
    }

    int findKey(int x, int y) {
        for (int i=0;i<(int)keyRects_.size();++i)
            if (keyRects_[i].contains({x,y})) return i;
        return -1;
    }

    bool handleMouse(int ev, int x, int y) {
        if (!visible) return false;
        hovPt_ = {x, y};
        if (ev == cv::EVENT_MOUSEMOVE) {
            hovKey_ = findKey(x, y);
            return false;
        }
        if (ev != cv::EVENT_LBUTTONDOWN) return false;
        
        if (btnBackspace_.contains({x,y})) {
            if (!text.empty()) text.pop_back();
            return false;
        }
        if (btnNumToggle_.contains({x,y})) { showNumbers_ = !showNumbers_; return false; }
        if (btnOk_.contains({x,y})) { visible = false; confirmed = true; return true; }
        if (btnCancel_.contains({x,y})) { visible = false; cancelled = true; text = ""; return false; }
        if (btnSpace_.contains({x,y})) { text += ' '; return false; }
        if (btnShift_.contains({x,y})) { shiftActive_ = !shiftActive_; return false; }
        
        int ki = findKey(x, y);
        if (ki >= 0 && ki < (int)keyRects_.size()) {
            auto keys = getCurrentKeys();
            std::vector<std::string> flat;
            for(const auto& row : keys) for(const auto& k : row) flat.push_back(k);
            
            if (ki < (int)flat.size()) {
                std::string ch = flat[ki];
                if (shiftActive_) {
                    if (!ch.empty()) ch[0] = (char)std::toupper((unsigned char)ch[0]);
                    shiftActive_ = false;
                }
                text += ch;
            }
        }
        return false;
    }

    void render(cv::Mat& frame) {
        if (!visible) return;
        int W = frame.cols, H = frame.rows;

        cv::Mat ov = frame.clone();
        cv::rectangle(ov, {0,0,W,H}, cv::Scalar(2,6,16), cv::FILLED);
        cv::addWeighted(ov, 0.88, frame, 0.12, 0, frame);

        const int KBW = 760, KBH = 310;
        const int KBX = (W - KBW) / 2;
        const int KBY = H - KBH - 10;
        cv::rectangle(frame, {KBX, KBY, KBW, KBH}, cv::Scalar(8,14,28), cv::FILLED);
        cv::rectangle(frame, {KBX, KBY, KBW, KBH}, cv::Scalar(0,150,200), 2, cv::LINE_AA);

        int fieldY = KBY + 18;
        cv::putText(frame, prompt, {KBX+16, fieldY+14},
                    cv::FONT_HERSHEY_PLAIN, 0.95, cv::Scalar(80,180,220), 1, cv::LINE_AA);
        cv::Rect fieldRect{KBX+16, fieldY+20, KBW-32, 32};
        cv::rectangle(frame, fieldRect, cv::Scalar(4,12,26), cv::FILLED);
        cv::rectangle(frame, fieldRect, cv::Scalar(0,180,230), 1, cv::LINE_AA);
        
        std::string display = text;
        double t = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        if (std::fmod(t * 2.0, 1.0) < 0.5) display += "|";
        cv::putText(frame, display, {fieldRect.x+8, fieldRect.y+22},
                    cv::FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(220,235,255), 1, cv::LINE_AA);

        auto& ROWS = getCurrentKeys();
        keyRects_.clear();
        const int keyH = 44, keyGap = 5;
        int rowY = KBY + 75;

        for (int r = 0; r < 3; ++r) {
            const auto& row = ROWS[r];
            int n = (int)row.size();
            if(n == 0) continue;
            int keyW = (KBW - keyGap*(n+1)) / n;
            int rowX = KBX + (KBW - (keyW*n + keyGap*(n-1))) / 2;
            for (int c = 0; c < n; ++c) {
                int kx = rowX + c*(keyW+keyGap);
                cv::Rect kr{kx, rowY, keyW, keyH};
                keyRects_.push_back(kr);
                bool hov = ((int)keyRects_.size()-1 == hovKey_);
                cv::Scalar bg   = hov ? cv::Scalar(30,80,120) : cv::Scalar(14,24,42);
                cv::Scalar bord = hov ? cv::Scalar(0,220,255) : cv::Scalar(30,60,90);
                cv::rectangle(frame, kr, bg, cv::FILLED);
                cv::rectangle(frame, kr, bord, 1, cv::LINE_AA);
                std::string label = row[c];
                if (!showNumbers_ && shiftActive_ && !label.empty())
                    label[0] = (char)std::toupper((unsigned char)label[0]);
                int bl=0; cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_DUPLEX, 0.55, 1, &bl);
                cv::putText(frame, label, {kx+(keyW-ts.width)/2, rowY+keyH/2+7},
                            cv::FONT_HERSHEY_DUPLEX, 0.55, hov?cv::Scalar(255,255,255):cv::Scalar(180,200,220),
                            1, cv::LINE_AA);
            }
            rowY += keyH + keyGap;
        }

        int ctrlY  = rowY;
        int ctrlH  = keyH;

        btnNumToggle_ = {KBX+keyGap, ctrlY, 90, ctrlH};
        bool hNum = btnNumToggle_.contains(hovPt_);
        cv::rectangle(frame, btnNumToggle_, hNum?cv::Scalar(30,50,80):cv::Scalar(10,20,35), cv::FILLED);
        cv::rectangle(frame, btnNumToggle_, cv::Scalar(40,80,120), 1, cv::LINE_AA);
        cv::putText(frame, showNumbers_?"ABC":"123", {btnNumToggle_.x+18, ctrlY+ctrlH/2+7},
                    cv::FONT_HERSHEY_DUPLEX, 0.55, cv::Scalar(160,200,230), 1, cv::LINE_AA);

        if (!showNumbers_) {
            btnShift_ = {KBX+keyGap+95, ctrlY, 70, ctrlH};
            bool hSh = btnShift_.contains(hovPt_);
            cv::Scalar shBord = shiftActive_ ? cv::Scalar(0,220,120) : cv::Scalar(40,80,120);
            cv::rectangle(frame, btnShift_, hSh?cv::Scalar(20,50,30):cv::Scalar(10,20,35), cv::FILLED);
            cv::rectangle(frame, btnShift_, shBord, 1, cv::LINE_AA);
            cv::putText(frame, "SHIFT", {btnShift_.x+6, ctrlY+ctrlH/2+7},
                        cv::FONT_HERSHEY_DUPLEX, 0.42, shiftActive_?cv::Scalar(0,230,120):cv::Scalar(140,180,200),
                        1, cv::LINE_AA);
        } else {
            btnShift_ = {0,0,0,0};
        }

        int spX = KBX + (showNumbers_ ? 100+keyGap*2 : 175+keyGap*2);
        int spW = KBW - spX + KBX - 100 - keyGap*3 - 90;
        btnSpace_ = {spX, ctrlY, spW, ctrlH};
        bool hSp = btnSpace_.contains(hovPt_);
        cv::rectangle(frame, btnSpace_, hSp?cv::Scalar(25,50,80):cv::Scalar(12,20,38), cv::FILLED);
        cv::rectangle(frame, btnSpace_, cv::Scalar(30,70,110), 1, cv::LINE_AA);
        cv::putText(frame, "ESPACIO", {spX+spW/2-36, ctrlY+ctrlH/2+7},
                    cv::FONT_HERSHEY_DUPLEX, 0.46, cv::Scalar(120,160,190), 1, cv::LINE_AA);

        btnBackspace_ = {KBX+KBW-keyGap-90, ctrlY, 90, ctrlH};
        bool hBs = btnBackspace_.contains(hovPt_);
        cv::rectangle(frame, btnBackspace_, hBs?cv::Scalar(60,20,20):cv::Scalar(25,10,10), cv::FILLED);
        cv::rectangle(frame, btnBackspace_, cv::Scalar(140,40,40), 1, cv::LINE_AA);
        cv::putText(frame, "<-", {btnBackspace_.x+20, ctrlY+ctrlH/2+7},
                    cv::FONT_HERSHEY_DUPLEX, 0.55, cv::Scalar(220,100,100), 1, cv::LINE_AA);

        int btnRow2Y = ctrlY + ctrlH + keyGap;
        btnOk_ = {KBX+KBW-keyGap-200, btnRow2Y, 200, ctrlH-6};
        bool hOk = btnOk_.contains(hovPt_);
        cv::rectangle(frame, btnOk_, hOk?cv::Scalar(0,60,20):cv::Scalar(0,30,10), cv::FILLED);
        cv::rectangle(frame, btnOk_, cv::Scalar(0,180,80), hOk?2:1, cv::LINE_AA);
        cv::putText(frame, "GUARDAR", {btnOk_.x+28, btnRow2Y+ctrlH/2+1},
                    cv::FONT_HERSHEY_DUPLEX, 0.55, cv::Scalar(0,230,100), 1, cv::LINE_AA);

        btnCancel_ = {KBX+keyGap, btnRow2Y, 130, ctrlH-6};
        bool hCan = btnCancel_.contains(hovPt_);
        cv::rectangle(frame, btnCancel_, hCan?cv::Scalar(40,30,10):cv::Scalar(18,14,6), cv::FILLED);
        cv::rectangle(frame, btnCancel_, cv::Scalar(140,100,30), hCan?2:1, cv::LINE_AA);
        cv::putText(frame, "CANCELAR", {btnCancel_.x+4, btnRow2Y+ctrlH/2+1},
                    cv::FONT_HERSHEY_DUPLEX, 0.42, cv::Scalar(200,160,60), 1, cv::LINE_AA);
    }
};

// ══════════════════════════════════════════════════════════════════════════════

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
    patient_name_ = "";

    if (use_help_ || is_session_) {
        // En sesión las vidas no se usan (ver handle_failed_challenge); se deja el valor por consistencia
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

    // ── TECLADO PARA LA SESIÓN CLÍNICA ──
    OnScreenKeyboard keyboard;
    if (is_session_) {
        keyboard.show(is_english_ ? "Patient Name (Letters, numbers, _):" : "Nombre del Paciente (Letras, num, _):", "");
        patient_name_ = "";
    }

    IntroData idata {false, cv::Rect(300, 370, 200, 52)};

    // Estructura combinada para manejar ambos (botón y teclado)
    struct CallbackData {
        IntroData* idata;
        OnScreenKeyboard* kb;
    };
    CallbackData cbData {&idata, &keyboard};

    cv::setMouseCallback(win_name, [](int event, int x, int y, int, void* userdata) {
        CallbackData* data = static_cast<CallbackData*>(userdata);
        if (data->kb->visible) {
            data->kb->handleMouse(event, x, y);
        } else {
            if (event == cv::EVENT_LBUTTONDOWN && data->idata->btn.contains(cv::Point(x, y))) {
                data->idata->clicked = true;
            }
        }
    }, &cbData);

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

        // Si el teclado está visible (sesión activa y aún no ingresan nombre), dibujarlo encima.
        if (keyboard.visible) {
            keyboard.render(frame);
            if (keyboard.confirmed) {
                patient_name_ = keyboard.text.empty() ? "Paciente_Anonimo" : keyboard.text;
                keyboard.hide(); 
            } else if (keyboard.cancelled) {
                auto msg = std::make_unique<std_msgs::msg::String>();
                msg->data = "idle";
                ui_state_publisher_->publish(std::move(msg));
                rclcpp::shutdown();
                break;
            }
        }

        cv::imshow(win_name, frame);
        int key = cv::waitKey(16) & 0xFF;

        if (keyboard.visible && key != -1 && key != 255) {
            if (key == 8 || key == 127) {   // Backspace
                if (!keyboard.text.empty()) keyboard.text.pop_back();
            } else if (key == 13 || key == 10) { // Enter -> confirmar
                patient_name_ = keyboard.text.empty() ? "Paciente_Anonimo" : keyboard.text;
                keyboard.hide();
            } else if (key >= 32 && key <= 126) {
                char ch = static_cast<char>(key);
                if (std::isalnum(ch) || ch == '_') {
                    if (keyboard.text.size() < 20) keyboard.text += ch;
                }
            }
        }

        if (key == 27) { // ESC para salir
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

    // Duración de la sesión (desde que el usuario pulsó COMENZAR)
    double duracion_seg = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - game_start_time_).count();
    
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
                 << "\"paciente\": \"" << (patient_name_.empty() ? "Anonimo" : patient_name_) << "\", "
                 << "\"duracion_seg\": " << static_cast<int>(duracion_seg) << ", "
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

    // Deja que el reporte salga hacia el bridge antes de cerrar el nodo
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
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

    // En sesión clínica NO hay vidas y el fallo NO cuenta como desafío jugado:
    // el paciente repite con otra pose hasta lograr 10 aciertos.
    // El fallo queda registrado en total_attempts_ para el reporte.
    if (is_session_) {
        auto feedback_msg = std::make_unique<std_msgs::msg::String>();
        feedback_msg->data = is_english_ ? "Don't worry, let's try another pose." : "No te preocupes, intentemos otra pose.";
        feedback_publisher_->publish(std::move(feedback_msg));
        std::this_thread::sleep_for(std::chrono::seconds(2));
        // SIN challenges_played_++: solo cuentan los aciertos
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