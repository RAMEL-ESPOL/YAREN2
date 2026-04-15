#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <map>

// Tu rutina de habla personalizada
static const std::vector<int> CUSTOM_ROUTINE = {0, 2, 8, 5, 6, 4, 3, 5, 6, 1};

// Configuración de tiempos
static constexpr double FRAME_DURATION = 0.20;  // Velocidad de habla
static constexpr double LISTEN_DURATION = 3.0;  // Tiempo que se queda escuchando
static constexpr int LISTEN_MOUTH_ID = 13;      // <--- La boca que querías para escuchar

class YarenConversationFace : public rclcpp::Node {
public:
    YarenConversationFace() : Node("yaren_face_node") {
        std::string pkgDir = ament_index_cpp::get_package_share_directory("yaren_face_display");
        std::string mouthDir = pkgDir + "/faces/separate_parts_without_background/mouths/";

        // 1. Ojos fijos (archivo 7.png)
        eyesImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/eyes_pairs/7.png", cv::IMREAD_UNCHANGED);

        // 2. Cargar frames necesarios (incluyendo el 13)
        std::vector<int> needed = {0, 1, 2, 3, 4, 5, 6, 8, LISTEN_MOUTH_ID};
        for (int n : needed) {
            std::string path = mouthDir + std::to_string(n) + ".png";
            cv::Mat img = cv::imread(path, cv::IMREAD_UNCHANGED);
            if (!img.empty()) {
                mouthMap[n] = img;
            } else {
                RCLCPP_WARN(this->get_logger(), "No se pudo cargar la boca: %s", path.c_str());
            }
        }

        // Estado inicial
        isListening = false;
        routineIdx = 0;
        lastStateChange = std::chrono::system_clock::now();
        running = true;

        facePublisher = this->create_publisher<sensor_msgs::msg::Image>("/face_screen", 10);
          // 1. Crear la ventana con nombre exacto
    cv::namedWindow("Yaren Face", cv::WINDOW_NORMAL);

    // 2. Quitar bordes y decoraciones de la ventana (Modo nativo)
    cv::setWindowProperty("Yaren Face", cv::WND_PROP_AUTOSIZE, cv::WINDOW_AUTOSIZE);

    // 3. Forzar Pantalla Completa
    cv::setWindowProperty("Yaren Face", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    // 4. (Opcional) Mover la ventana a la esquina 0,0 para asegurar que cubra todo
    cv::moveWindow("Yaren Face", 0, 0);
        // 7. Hilo de procesamiento de frames
        renderThread = std::thread(&YarenConversationFace::renderLoop, this);

        RCLCPP_INFO(this->get_logger(), "Initialized Yaren Conversation Node");
    }


~YarenConversationFace() {
        running = false;
        if (renderThread.joinable()) {
            renderThread.join();
        }
        // Estas líneas van DENTRO de las llaves del destructor
        cv::destroyWindow("Yaren Face"); 
        cv::destroyAllWindows();        
    }

    void updateWindow() {
        std::lock_guard<std::mutex> lock(frameMutex);
        if (!latestFrame.empty()) {
            cv::imshow("Yaren Face", latestFrame);
            
            // Capturar la tecla ESC (código 27)
            int key = cv::waitKey(1);
            if (key == 27) { 
                RCLCPP_INFO(this->get_logger(), "Cerrando ventana de Yaren...");
                running = false; 
                rclcpp::shutdown(); 
            }
        }
    }

private:
    void overlayImage(cv::Mat& bg, const cv::Mat& fg) {
        if (fg.empty() || fg.channels() != 4) return;
        for (int y = 0; y < std::min(bg.rows, fg.rows); ++y) {
            for (int x = 0; x < std::min(bg.cols, fg.cols); ++x) {
                cv::Vec4b& fP = const_cast<cv::Mat&>(fg).at<cv::Vec4b>(y, x);
                float alpha = fP[3] / 255.0f;
                if (alpha > 0) {
                    cv::Vec3b& bP = bg.at<cv::Vec3b>(y, x);
                    for (int c = 0; c < 3; ++c)
                        bP[c] = static_cast<uchar>(fP[c] * alpha + bP[c] * (1.0f - alpha));
                }
            }
        }
    }

    void renderLoop() {
        rclcpp::Rate rate(30);
        while (running && rclcpp::ok()) {
            auto now = std::chrono::system_clock::now();
            double elapsed = std::chrono::duration<double>(now - lastStateChange).count();

            int currentMouthNum = 0;

            if (isListening) {
                // --- ESTADO: ESCUCHANDO ---
                currentMouthNum = LISTEN_MOUTH_ID; // Muestra la boca 13
                if (elapsed >= LISTEN_DURATION) {
                    isListening = false;
                    routineIdx = 0;
                    lastStateChange = now;
                }
            } else {
                // --- ESTADO: HABLANDO ---
                if (elapsed >= FRAME_DURATION) {
                    routineIdx++;
                    lastStateChange = now;
                    
                    if (routineIdx >= CUSTOM_ROUTINE.size()) {
                        isListening = true; // Al terminar la rutina, pasa a escuchar
                        lastStateChange = now;
                        currentMouthNum = LISTEN_MOUTH_ID;
                    }
                }
                
                if (!isListening) {
                    currentMouthNum = CUSTOM_ROUTINE[routineIdx];
                }
            }

            // Composición final
            cv::Mat canvas = cv::Mat::zeros(eyesImg.size(), CV_8UC3);
            overlayImage(canvas, eyesImg);
            
            if (mouthMap.count(currentMouthNum)) {
                overlayImage(canvas, mouthMap[currentMouthNum]);
            } else if (mouthMap.count(0)) {
                overlayImage(canvas, mouthMap[0]); // Respaldo por si falla la carga
            }

            {
                std::lock_guard<std::mutex> lock(frameMutex);
                latestFrame = canvas.clone();
            }

            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", canvas).toImageMsg();
            facePublisher->publish(*msg);

            rate.sleep();
        }
    }

    cv::Mat eyesImg;
    std::map<int, cv::Mat> mouthMap;
    size_t routineIdx;
    bool isListening;
    std::chrono::system_clock::time_point lastStateChange;
    
    bool running;
    std::thread renderThread;
    std::mutex frameMutex;
    cv::Mat latestFrame;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr facePublisher;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YarenConversationFace>();
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->updateWindow();
    }
    rclcpp::shutdown();
    return 0;
}