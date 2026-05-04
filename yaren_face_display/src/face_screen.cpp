#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <thread>
#include <mutex>
#include <cstdlib>
#include <atomic>

namespace fs = std::filesystem;

// ═════════════════════════════════════════════════════════════
//  Estructura de ítem de menú / submenú
// ═════════════════════════════════════════════════════════════
struct MenuItem {
    std::string id;          
    std::string label;       
    std::string sublabel;    
    cv::Scalar  color;       
    cv::Rect    rect;        
    std::string cmd;         
    std::string stopCmd;     
    bool        hasSubMenu;  
};

// ═════════════════════════════════════════════════════════════
//  Nodo principal
// ═════════════════════════════════════════════════════════════
class VideoSynchronizer : public rclcpp::Node {
public:
    VideoSynchronizer() : Node("face_screen") {

        std::string pkgDir =
            ament_index_cpp::get_package_share_directory("yaren_face_display");

        // ── 1. Imágenes base ──────────────────────────────────
        eyesOpenImg    = cv::imread(pkgDir + "/faces/separate_parts_without_background/eyes_pairs/7.png",  cv::IMREAD_UNCHANGED);
        eyesClosedImg  = cv::imread(pkgDir + "/faces/separate_parts_without_background/eyes_pairs/3.png",  cv::IMREAD_UNCHANGED);
        mouthClosedImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/mouths/13.png", cv::IMREAD_UNCHANGED);
        mouthOpenImg   = cv::imread(pkgDir + "/faces/separate_parts_without_background/mouths/8.png",  cv::IMREAD_UNCHANGED);

        // ── 2. Secuencias de frames ───────────────────────────
        loadFrames(pkgDir + "/faces/transitions/blinking_frames", eyesFrames);
        loadFrames(pkgDir + "/faces/transitions/blinking_frames", mouthFrames);

        // ── 3. Estado inicial ─────────────────────────────────
        ttsActive         = false;
        isBlinking        = false;
        showMenu          = false;
        showSubMenu       = false;
        
        hoveredItem       = -1;
        hoveredSubItem    = -1;
        hoveredStopButton = false;
        hoveredExitButton = false;
        hoveredBackButton = false;

        activeMode        = "";
        activeStopCmd     = "";
        running           = true;
        lastBlinkTime     = std::chrono::system_clock::now();

        // ── 4. Suscripción TTS ────────────────────────────────
        ttsSubscription = this->create_subscription<std_msgs::msg::Bool>(
            "/audio_playing", 10,
            std::bind(&VideoSynchronizer::audioPlayingCallback, this, std::placeholders::_1));

        // ── 5. Publicadores ───────────────────────────────────
        faceScreenPublisher = this->create_publisher<sensor_msgs::msg::Image>("/face_screen", 10);
        modePublisher       = this->create_publisher<std_msgs::msg::String>("/yaren_mode", 10);

        // ── 6. Ventana OpenCV ─────────────────────────────────
        cv::namedWindow("Yaren Face", cv::WINDOW_NORMAL);
        cv::setWindowProperty("Yaren Face", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        cv::moveWindow("Yaren Face", 0, 0);
        cv::setMouseCallback("Yaren Face", VideoSynchronizer::mouseCallbackStatic, this);

        // ── 7. Menú principal (6 tarjetas 3×2) ───────────────
        menuItems = {
            {
                "yaren_mimic", "MIMIC", "Yaren te Imita",
                { 0, 229, 255 }, {},
                "ros2 launch yaren_arm_mimic yaren_mimic.launch.py &",
                "for pid in $(ps aux | grep -E 'yaren_mimic' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
                false
            },
            {
                "yaren_chat", "CHAT", "Conversar con Yaren",
                { 29, 233, 22 }, {},
                "ros2 launch yaren_chat yaren_chat.launch.py &",
                "for pid in $(ps aux | grep -E 'yaren_chat' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
                false
            },
            {
                "yaren_dice", "DICE", "Jugar Yaren Dice",
                { 64, 171, 255 }, {},
                "ros2 launch yaren_dice yaren_dice.launch.py &",
                "for pid in $(ps aux | grep -E 'yaren_dice' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
                false
            },
            {
                "yaren_movements", "MOVEMENTS", "Yaren se mueve >",
                { 251, 64, 224 }, {},
                "", "", 
                true
            },
            {
                "yaren_emotions", "EMOTIONS", "Yaren dectecta tu emocion",
                { 82, 82, 255 }, {},
                "ros2 launch yaren_emotions yaren_emotions.launch.py &",
                "for pid in $(ps aux | grep -E 'yaren_emotions' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
                false
            },
            {
                "yaren_filtros", "FILTROS", "Yaren te pone filros  >",
                { 105, 240, 174 }, {},
                "", "", 
                true
            },
        };

        // ── 8. Submenú MOVEMENTS: 3 rutinas ──────────────────
        subMenuMovements = {
            {
                "yaren_rutina1", "RUTINA 1", "Rutinas",
                { 251, 64, 224 }, {},
                "python3 src/YAREN2/yaren_movements/yaren_movements/yaren_rutina1.py &",
                "for pid in $(ps aux | grep -E 'yaren_rutina1' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
                false
            },
            {
                "yaren_rutina2", "RUTINA 2", "Rutina infinita",
                { 220, 80, 200 }, {},
                "python3 src/YAREN2/yaren_movements/yaren_movements/yaren_fullmovement.py &",
                "for pid in $(ps aux | grep -E 'yaren_fullmovement' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
                false
            },
            {
                "yaren_rutina3", "RUTINA 3", "movimiento libre",
                { 190, 100, 180 }, {},
                "python3 src/YAREN2/yaren_movements/yaren_movements/yaren_movement.py &",
                "for pid in $(ps aux | grep -E 'yaren_movement' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
                false
            },
        };

        // ── 9. Submenú FILTROS: animales y accesorios ─────────
        subMenuFiltros = {
            {
                "yaren_animales", "ANIMALES", "filtro animal",
                { 105, 240, 174 }, {},
                "ros2 launch yaren_filters yaren_animales.launch.py &",
                "for pid in $(ps aux | grep -E 'yaren_animales|animal_filter|face_landmark|csi_cam' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
                false
            },
            {
                "yaren_accesorios", "ACCESORIOS", "filtro accesorios",
                { 60, 200, 130 }, {},
                "ros2 launch yaren_filters yaren_accesorios.launch.py &",
                "for pid in $(ps aux | grep -E 'yaren_accesorios|face_filter|face_landmark|csi_cam' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done",
                false
            },
        };

        // ── 10. Hilo de renderizado ───────────────────────────
        renderThread = std::thread(&VideoSynchronizer::renderLoop, this);
        RCLCPP_INFO(get_logger(), "face_screen listo — menu + submenus integrados.");
    }

    ~VideoSynchronizer() {
        running = false;
        if (renderThread.joinable()) renderThread.join();
        cv::destroyAllWindows();
        
        if (!activeStopCmd.empty()) {
            std::system(activeStopCmd.c_str());
        }
    }

    void drawWindow() {
        std::lock_guard<std::mutex> lock(frameMutex);
        if (!latestFrame.empty()) {
            cv::imshow("Yaren Face", latestFrame);
            int key = cv::waitKey(1);
            if (key == 27) {   
                if (showSubMenu) {
                    showSubMenu    = false;
                    hoveredSubItem = -1;
                } else if (showMenu) {
                    showMenu          = false;
                    hoveredItem       = -1;
                } else {
                    running = false;
                    rclcpp::shutdown();
                }
            }
        }
    }

    static void mouseCallbackStatic(int event, int x, int y,
                                    int /*flags*/, void* userdata) {
        static_cast<VideoSynchronizer*>(userdata)->handleMouse(event, x, y);
    }

private:
    void loadFrames(const std::string& dir, std::vector<cv::Mat>& frames) {
        if (!fs::exists(dir)) return;
        std::vector<std::string> files;
        for (const auto& e : fs::directory_iterator(dir))
            if (e.path().extension() == ".png")
                files.push_back(e.path().string());
        std::sort(files.begin(), files.end());
        for (const auto& f : files) {
            cv::Mat m = cv::imread(f);
            if (!m.empty()) frames.push_back(m);
        }
    }

    void audioPlayingCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        ttsActive = msg->data;
    }

    // ═════════════════════════════════════════════════════════
    //  Gestión del mouse
    // ═════════════════════════════════════════════════════════
    void handleMouse(int event, int x, int y) {

        // ── Capa 3: SUBMENÚ ───────────────────────────────────
        if (showSubMenu) {
            if (event == cv::EVENT_MOUSEMOVE) {
                hoveredBackButton = backButtonRect.contains({x,y});
                hoveredSubItem = -1;
                for (int i = 0; i < (int)activeSubMenu.size(); ++i)
                    if (activeSubMenu[i].rect.contains({x,y})) { hoveredSubItem=i; break; }
            }
            if (event == cv::EVENT_LBUTTONDOWN) {
                // 1. Botón VOLVER
                if (backButtonRect.contains({x,y})) {
                    showSubMenu = false;
                    hoveredBackButton = false;
                    return;
                }
                // 2. Tarjetas
                for (int i = 0; i < (int)activeSubMenu.size(); ++i) {
                    if (activeSubMenu[i].rect.contains({x,y})) {
                        executeMode(activeSubMenu[i]);
                        return;
                    }
                }
            }
            return;
        }

        // ── Capa 2: MENÚ PRINCIPAL ────────────────────────────
        if (showMenu) {
            if (event == cv::EVENT_MOUSEMOVE) {
                hoveredStopButton = (!activeMode.empty() && stopButtonRect.contains({x,y}));
                hoveredExitButton = exitButtonRect.contains({x,y});
                
                hoveredItem = -1;
                for (int i = 0; i < (int)menuItems.size(); ++i)
                    if (menuItems[i].rect.contains({x,y})) { hoveredItem=i; break; }
            }
            if (event == cv::EVENT_LBUTTONDOWN) {
                // 1. Botón SALIR
                if (exitButtonRect.contains({x,y})) {
                    showMenu = false;
                    hoveredExitButton = false;
                    return;
                }
                // 2. Botón APAGAR MODO
                if (!activeMode.empty() && stopButtonRect.contains({x,y})) {
                    RCLCPP_INFO(get_logger(), "Apagando modo manual: %s", activeMode.c_str());
                    if (!activeStopCmd.empty()) {
                        std::system(activeStopCmd.c_str()); 
                    }
                    activeMode        = "";
                    activeStopCmd     = "";
                    showMenu          = false; 
                    hoveredStopButton = false;
                    
                    auto msg = std_msgs::msg::String();
                    msg.data = "idle";
                    modePublisher->publish(msg);
                    return; 
                }
                // 3. Tarjetas Principales
                for (int i = 0; i < (int)menuItems.size(); ++i) {
                    if (menuItems[i].rect.contains({x,y})) {
                        if (menuItems[i].hasSubMenu)
                            openSubMenu(menuItems[i].id, menuItems[i].color);
                        else
                            executeMode(menuItems[i]);
                        return; 
                    }
                }
            }
            return;
        }

        // ── Capa 1: CARA (idle) ───────────────────────────────
        if (event == cv::EVENT_LBUTTONDOWN) {
            showMenu          = true;
            hoveredItem       = -1;
            hoveredStopButton = false;
            hoveredExitButton = false;
        }
    }

    void openSubMenu(const std::string& parentId, const cv::Scalar& parentColor) {
        subMenuTitle       = (parentId == "yaren_movements") ? "MOVEMENTS" : "FILTROS";
        subMenuAccentColor = parentColor;
        activeSubMenu      = (parentId == "yaren_movements") ? subMenuMovements : subMenuFiltros;
        hoveredSubItem     = -1;
        showSubMenu        = true;
    }

    void executeMode(const MenuItem& item) {
        showMenu          = false;
        showSubMenu       = false;
        hoveredItem       = -1;
        hoveredSubItem    = -1;
        hoveredStopButton = false;
        hoveredExitButton = false;
        hoveredBackButton = false;

        if (!activeMode.empty() && !activeStopCmd.empty()) {
            RCLCPP_INFO(get_logger(), "Auto-apagando modo anterior para iniciar nuevo: %s", activeMode.c_str());
            std::system(activeStopCmd.c_str());
        }

        activeMode    = item.id;
        activeStopCmd = item.stopCmd; 

        auto msg = std_msgs::msg::String();
        msg.data = activeMode;
        modePublisher->publish(msg);

        if (!item.cmd.empty()) {
            RCLCPP_INFO(get_logger(), "Iniciando Nuevo Modo: %s", item.cmd.c_str());
            std::system(item.cmd.c_str());
        }
    }

    cv::Mat getFaceFrame() {
        auto now = std::chrono::system_clock::now();
        cv::Mat res = cv::Mat::zeros(eyesOpenImg.size(), CV_8UC3);

        double sinceBlink = std::chrono::duration<double>(now - lastBlinkTime).count();
        if (!isBlinking && sinceBlink > 4.0) {
            isBlinking = true; blinkStartTime = now; lastBlinkTime = now;
        }
        cv::Mat eye = isBlinking ? eyesClosedImg : eyesOpenImg;
        if (isBlinking && std::chrono::duration<double>(now - blinkStartTime).count() > 0.2)
            isBlinking = false;
            
        overlayImage(res, eye);
        overlayImage(res, ttsActive ? mouthOpenImg : mouthClosedImg);
        return res;
    }

    // ═════════════════════════════════════════════════════════
    //  Renderizado del MENÚ PRINCIPAL
    // ═════════════════════════════════════════════════════════
    void renderMenu(cv::Mat& frame) {
        int W = frame.cols, H = frame.rows;

        cv::Mat ov = frame.clone();
        cv::rectangle(ov, {0,0,W,H}, cv::Scalar(5,13,26), cv::FILLED);
        cv::addWeighted(ov, 0.88, frame, 0.12, 0, frame);

        const int COLS=3, ROWS=2;
        const int CW = std::min(220, W/COLS - 20);
        const int CH = std::min(150, H/(ROWS+2));
        const int G  = 14;
        const int TW = COLS*CW + (COLS-1)*G;
        const int TH = ROWS*CH + (ROWS-1)*G;
        const int SX = (W-TW)/2;
        const int SY = (H-TH)/2 + 20;

        drawCentered(frame, "SELECCIONA UN MODO", W, SY-28, cv::FONT_HERSHEY_DUPLEX, 0.65, cv::Scalar(0,160,200), 1);

        for (int i = 0; i < (int)menuItems.size(); ++i) {
            int col=i%COLS, row=i/COLS;
            menuItems[i].rect = { SX+col*(CW+G), SY+row*(CH+G), CW, CH };
            drawCard(frame, menuItems[i], hoveredItem==i, menuItems[i].hasSubMenu);
        }

        // Layout de Botones (SALIR y APAGAR)
        int btnHeight = 40;
        int btnY = SY + TH + 30;

        if (!activeMode.empty()) {
            // Si hay modo activo: Dibujamos APAGAR y SALIR uno al lado del otro
            int stopWidth = 300;
            int exitWidth = 150;
            int gap = 20;
            int totalWidth = stopWidth + gap + exitWidth;
            int startX = (W - totalWidth) / 2;

            stopButtonRect = { startX, btnY, stopWidth, btnHeight };
            exitButtonRect = { startX + stopWidth + gap, btnY, exitWidth, btnHeight };

            // Dibujar botón de APAGAR
            cv::Scalar stopColor = hoveredStopButton ? cv::Scalar(40, 40, 220) : cv::Scalar(20, 20, 180);
            cv::rectangle(frame, stopButtonRect, stopColor, cv::FILLED);
            cv::rectangle(frame, stopButtonRect, cv::Scalar(100, 100, 255), 1, cv::LINE_AA);
            drawTextInRect(frame, "APAGAR: " + activeMode, stopButtonRect, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        } else {
            // Si NO hay modo activo: Reseteamos APAGAR y centramos SALIR
            stopButtonRect = {0, 0, 0, 0};
            int exitWidth = 150;
            exitButtonRect = { (W - exitWidth) / 2, btnY, exitWidth, btnHeight };
        }

        // Dibujar botón de SALIR (Se dibuja siempre)
        cv::Scalar exitColor = hoveredExitButton ? cv::Scalar(60, 60, 60) : cv::Scalar(30, 30, 35);
        cv::rectangle(frame, exitButtonRect, exitColor, cv::FILLED);
        cv::rectangle(frame, exitButtonRect, cv::Scalar(120, 120, 120), 1, cv::LINE_AA);
        drawTextInRect(frame, "SALIR", exitButtonRect, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    }

    // ═════════════════════════════════════════════════════════
    //  Renderizado del SUBMENÚ
    // ═════════════════════════════════════════════════════════
    void renderSubMenu(cv::Mat& frame) {
        int W = frame.cols, H = frame.rows;

        cv::Mat ov = frame.clone();
        cv::rectangle(ov, {0,0,W,H}, cv::Scalar(3,8,18), cv::FILLED);
        cv::addWeighted(ov, 0.92, frame, 0.08, 0, frame);

        int N = (int)activeSubMenu.size();
        const int CW = std::min(200, W/std::max(N,1) - 24);
        const int CH = 170;
        const int G  = 18;
        const int TW = N*CW + (N-1)*G;
        const int SX = (W-TW)/2;
        const int SY = (H-CH)/2;

        drawCentered(frame, "YAREN  >  " + subMenuTitle, W, SY-40, cv::FONT_HERSHEY_DUPLEX, 0.7, subMenuAccentColor, 1);
        cv::line(frame, {SX, SY-22}, {SX+TW, SY-22}, cv::Scalar(subMenuAccentColor[0]*.4, subMenuAccentColor[1]*.4, subMenuAccentColor[2]*.4), 1, cv::LINE_AA);

        for (int i = 0; i < N; ++i) {
            activeSubMenu[i].rect = { SX+i*(CW+G), SY, CW, CH };
            drawCard(frame, activeSubMenu[i], hoveredSubItem==i, false);
        }

        // DIBUJAR BOTÓN FIJO "VOLVER"
        int bbw = 150, bbh = 40;
        backButtonRect = { (W - bbw) / 2, SY + CH + 35, bbw, bbh };

        cv::Scalar backColor = hoveredBackButton ? cv::Scalar(60, 60, 60) : cv::Scalar(30, 30, 35);
        cv::rectangle(frame, backButtonRect, backColor, cv::FILLED);
        cv::rectangle(frame, backButtonRect, cv::Scalar(120, 120, 120), 1, cv::LINE_AA);
        drawTextInRect(frame, "VOLVER", backButtonRect, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    }

    void drawCard(cv::Mat& frame, const MenuItem& item, bool hovered, bool arrow) {
        int cx=item.rect.x, cy=item.rect.y, cw=item.rect.width, ch=item.rect.height;
        cv::Scalar a = item.color;

        cv::rectangle(frame, item.rect, hovered ? cv::Scalar(20,30,45) : cv::Scalar(10,18,31), cv::FILLED);
        cv::rectangle(frame, item.rect, hovered ? a : cv::Scalar(a[0]*.35,a[1]*.35,a[2]*.35), hovered ? 2 : 1, cv::LINE_AA);

        cv::line(frame,{cx+4,cy+4},{cx+16,cy+4}, a, 1, cv::LINE_AA);
        cv::line(frame,{cx+4,cy+4},{cx+4,cy+16}, a, 1, cv::LINE_AA);

        int icx=cx+cw/2, icy=cy+ch/2-18;
        cv::circle(frame,{icx,icy},22, cv::Scalar(a[0]*.3,a[1]*.3,a[2]*.3),1,cv::LINE_AA);
        cv::circle(frame,{icx,icy},hovered?9:6,a,cv::FILLED,cv::LINE_AA);

        if (arrow) {
            std::vector<cv::Point> pts = {{cx+cw-22,icy-7},{cx+cw-12,icy},{cx+cw-22,icy+7}};
            cv::polylines(frame,pts,false,a,1,cv::LINE_AA);
        }

        int bl=0;
        cv::Size ls=cv::getTextSize(item.label,cv::FONT_HERSHEY_DUPLEX,0.55,1,&bl);
        cv::putText(frame,item.label,{cx+(cw-ls.width)/2,cy+ch-30}, cv::FONT_HERSHEY_DUPLEX,0.55, hovered ? a : cv::Scalar(a[0]*.7,a[1]*.7,a[2]*.7), 1,cv::LINE_AA);

        cv::Size ss=cv::getTextSize(item.sublabel,cv::FONT_HERSHEY_PLAIN,0.85,1,&bl);
        cv::putText(frame,item.sublabel,{cx+(cw-ss.width)/2,cy+ch-12}, cv::FONT_HERSHEY_PLAIN,0.85,cv::Scalar(90,110,130),1,cv::LINE_AA);
    }

    // Centra texto en una posición Y dada (Toda la pantalla)
    void drawCentered(cv::Mat& frame, const std::string& txt, int W, int y, int font, double scale, const cv::Scalar& color, int thick) {
        int bl=0;
        cv::Size s=cv::getTextSize(txt,font,scale,thick,&bl);
        cv::putText(frame,txt,{(W-s.width)/2,y},font,scale,color,thick,cv::LINE_AA);
    }

    // Centra texto matemáticamente dentro de un rectángulo
    void drawTextInRect(cv::Mat& frame, const std::string& txt, const cv::Rect& rect, int font, double scale, const cv::Scalar& color, int thick) {
        int bl = 0;
        cv::Size s = cv::getTextSize(txt, font, scale, thick, &bl);
        cv::Point pt(rect.x + (rect.width - s.width) / 2, rect.y + (rect.height + s.height) / 2 - 2);
        cv::putText(frame, txt, pt, font, scale, color, thick, cv::LINE_AA);
    }

    void renderLoop() {
        rclcpp::Rate rate(30);
        while (running && rclcpp::ok()) {
            cv::Mat frame = getFaceFrame();

            if      (showSubMenu) renderSubMenu(frame);
            else if (showMenu)    renderMenu(frame);

            if (!frame.empty()) {
                { std::lock_guard<std::mutex> lock(frameMutex); latestFrame=frame.clone(); }
                auto msg=cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",frame).toImageMsg();
                faceScreenPublisher->publish(*msg);
            }
            rate.sleep();
        }
    }

    void overlayImage(cv::Mat& bg, const cv::Mat& fg) {
        if (fg.empty()) return;
        if (fg.channels()==4) {
            for (int y=0;y<bg.rows;++y)
                for (int x=0;x<bg.cols;++x) {
                    const cv::Vec4b& f=fg.at<cv::Vec4b>(y,x);
                    float a=f[3]/255.f;
                    if (a>0) {
                        cv::Vec3b& b=bg.at<cv::Vec3b>(y,x);
                        for (int c=0;c<3;++c) b[c]=(uchar)(f[c]*a+b[c]*(1.f-a));
                    }
                }
        } else { fg.copyTo(bg); }
    }

    cv::Mat eyesOpenImg, eyesClosedImg, mouthOpenImg, mouthClosedImg;
    std::vector<cv::Mat> eyesFrames, mouthFrames;

    std::atomic<bool> ttsActive;
    std::atomic<bool> isBlinking;
    std::atomic<bool> running;
    std::atomic<bool> showMenu;
    std::atomic<bool> showSubMenu;
    
    std::atomic<int>  hoveredItem;
    std::atomic<int>  hoveredSubItem;
    
    std::atomic<bool> hoveredStopButton;
    std::atomic<bool> hoveredExitButton;
    std::atomic<bool> hoveredBackButton;
    
    std::string       activeMode;
    std::string       activeStopCmd;
    
    cv::Rect          stopButtonRect;
    cv::Rect          exitButtonRect;
    cv::Rect          backButtonRect;

    std::chrono::system_clock::time_point lastBlinkTime, blinkStartTime;

    std::vector<MenuItem> menuItems;           
    std::vector<MenuItem> subMenuMovements;    
    std::vector<MenuItem> subMenuFiltros;      
    std::vector<MenuItem> activeSubMenu;       
    std::string           subMenuTitle;
    cv::Scalar            subMenuAccentColor;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr  ttsSubscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr faceScreenPublisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   modePublisher;

    std::thread renderThread;
    std::mutex  frameMutex;
    cv::Mat     latestFrame;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoSynchronizer>();
    rclcpp::Rate rate(30);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->drawWindow();
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}