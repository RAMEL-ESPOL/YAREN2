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

// Configuración de la rutina y tiempos
static const std::vector<int> TALK_ROUTINE = {0, 2, 8, 5, 6, 4, 3, 5, 6, 1};
static constexpr double FRAME_DURATION = 0.15;
static constexpr double WAIT_DURATION = 2.0; // Tiempo para la boca 13
static constexpr int LISTEN_MOUTH_ID = 13;

enum class FaceState { TALKING, LISTENING, ALT_FACE };

class YarenFullConversation : public rclcpp::Node {
public:
    YarenFullConversation() : Node("yaren_face_node") {
        std::string pkgDir = ament_index_cpp::get_package_share_directory("yaren_face_display");
        std::string mouthDir = pkgDir + "/faces/separate_parts_without_background/mouths/";

        // 1. Ojos Base
        eyesImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/eyes_pairs/7.png", cv::IMREAD_UNCHANGED);

        // 2. Bocas
        std::vector<int> needed = {0, 1, 2, 3, 4, 5, 6, 8, LISTEN_MOUTH_ID};
        for (int n : needed) {
            mouthMap[n] = cv::imread(mouthDir + std::to_string(n) + ".png", cv::IMREAD_UNCHANGED);
        }

        // 3. Caras Completas
        std::vector<std::string> faceFiles = {"thinking.png", "angry.png", "sad.png", "happy.png"};
        for (const auto& f : faceFiles) {
            cv::Mat img = cv::imread(pkgDir + "/faces/" + f, cv::IMREAD_UNCHANGED);
            if (!img.empty()) altFaces.push_back(img);
        }

        currentState = FaceState::TALKING;
        routineIdx = 0;
        altFaceIdx = 0;
        lastChange = std::chrono::system_clock::now();
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
        renderThread = std::thread(&YarenFullConversation::renderLoop, this);

        RCLCPP_INFO(this->get_logger(), "Initialized Yaren Conversation Node");
    }


    ~YarenFullConversation() {
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
        if (fg.empty()) return;
        if (fg.channels() != 4) {
            cv::resize(fg, fg, bg.size());
            fg.copyTo(bg);
            return;
        }
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
            double elapsed = std::chrono::duration<double>(now - lastChange).count();
            cv::Mat canvas = cv::Mat::zeros(eyesImg.size(), CV_8UC3);

            switch (currentState) {
                case FaceState::TALKING:
                    overlayImage(canvas, eyesImg);
                    if (elapsed >= FRAME_DURATION) {
                        routineIdx++;
                        lastChange = now;
                        if (routineIdx >= TALK_ROUTINE.size()) {
                            currentState = FaceState::LISTENING; // Toca escuchar
                        }
                    }
                    overlayImage(canvas, mouthMap[TALK_ROUTINE[routineIdx % TALK_ROUTINE.size()]]);
                    break;

                case FaceState::LISTENING:
                    // Muestra boca 13 (Escuchando)
                    overlayImage(canvas, eyesImg);
                    overlayImage(canvas, mouthMap[LISTEN_MOUTH_ID]);
                    if (elapsed >= WAIT_DURATION) {
                        currentState = FaceState::ALT_FACE; // Toca cambiar cara
                        lastChange = now;
                    }
                    break;

                case FaceState::ALT_FACE:
                    // Muestra cara completa (angry, thinking, etc.)
                    overlayImage(canvas, altFaces[altFaceIdx]);
                    if (elapsed >= WAIT_DURATION) {
                        altFaceIdx = (altFaceIdx + 1) % altFaces.size();
                        currentState = FaceState::TALKING; // Vuelve a hablar
                        routineIdx = 0;
                        lastChange = now;
                    }
                    break;
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
    std::vector<cv::Mat> altFaces;
    FaceState currentState;
    size_t routineIdx, altFaceIdx;
    std::chrono::system_clock::time_point lastChange;
    bool running;
    std::thread renderThread;
    std::mutex frameMutex;
    cv::Mat latestFrame;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr facePublisher;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YarenFullConversation>();
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->updateWindow();
    }
    rclcpp::shutdown();
    return 0;
}