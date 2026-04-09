#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>

namespace fs = std::filesystem;

class VideoSynchronizer : public rclcpp::Node {
public:
    VideoSynchronizer() : Node("face_screen") {
        std::string pkgDir = ament_index_cpp::get_package_share_directory("yaren_face_screen");

        // 1. Cargar imágenes base (IMPORTANTE: IMREAD_UNCHANGED para soportar transparencia)
        eyesOpenImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/eyes_pairs/7.png", cv::IMREAD_UNCHANGED);
        eyesClosedImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/eyes_pairs/3.png", cv::IMREAD_UNCHANGED);
        mouthClosedImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/mouths/13.png", cv::IMREAD_UNCHANGED);
        mouthOpenImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/mouths/8.png", cv::IMREAD_UNCHANGED);

        // 2. Cargar Caras Alternativas (También con UNCHANGED)
        std::vector<std::string> altFacePaths = {
            pkgDir + "/faces/angry.png",
            pkgDir + "/faces/game_over.png",
            pkgDir + "/faces/mal_humor.png",
            pkgDir + "/faces/meh.png",
            pkgDir + "/faces/money.png",
            pkgDir + "/faces/open_mouth.png",
            pkgDir + "/faces/ready.png",
            pkgDir + "/faces/sad.png",
            pkgDir + "/faces/thinking.png",
            pkgDir + "/faces/tongue_out.png"
            
        };

        for (const auto& path : altFacePaths) {
            cv::Mat img = cv::imread(path, cv::IMREAD_UNCHANGED);
            if (!img.empty()) {
                altFaces.push_back(img);
            } else {
                RCLCPP_WARN(this->get_logger(), "No se pudo cargar: %s", path.c_str());
            }
        }

        // 3. Estado inicial
        ttsActive = false;
        isBlinking = false;
        showingAltFace = false;
        blinkCount = 0; 
        currentAltFaceIndex = 0;
        lastBlinkTime = std::chrono::system_clock::now();
        running = true;

        // ROS y Ventana
        ttsSubscription = this->create_subscription<std_msgs::msg::Bool>(
            "/audio_playing", 10, std::bind(&VideoSynchronizer::audioPlayingCallback, this, std::placeholders::_1));
        faceScreenPublisher = this->create_publisher<sensor_msgs::msg::Image>("/face_screen", 10);

        // 1. Crear la ventana con nombre exacto
    cv::namedWindow("Yaren Face", cv::WINDOW_NORMAL);

    // 2. Quitar bordes y decoraciones de la ventana (Modo nativo)
    cv::setWindowProperty("Yaren Face", cv::WND_PROP_AUTOSIZE, cv::WINDOW_AUTOSIZE);

    // 3. Forzar Pantalla Completa
    cv::setWindowProperty("Yaren Face", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    // 4. (Opcional) Mover la ventana a la esquina 0,0 para asegurar que cubra todo
    cv::moveWindow("Yaren Face", 0, 0);
        // 7. Hilo de procesamiento de frames
        renderThread = std::thread(&VideoSynchronizer::renderLoop, this);

        RCLCPP_INFO(this->get_logger(), "Initialized Video Synchronizer Node");
    }

    ~VideoSynchronizer() {
        running = false;
        if (renderThread.joinable()) renderThread.join();
        cv::destroyAllWindows();
    }

private:
    // Función esencial para mezclar capas transparentes sobre el fondo negro
    void overlayImage(cv::Mat& background, const cv::Mat& foreground) {
        if (foreground.empty()) return;
        if (foreground.channels() == 4) {
            for (int y = 0; y < background.rows; ++y) {
                for (int x = 0; x < background.cols; ++x) {
                    if (y >= foreground.rows || x >= foreground.cols) continue;
                    cv::Vec4b& fgPixel = const_cast<cv::Mat&>(foreground).at<cv::Vec4b>(y, x);
                    float alpha = fgPixel[3] / 255.0f;
                    if (alpha > 0) {
                        cv::Vec3b& bgPixel = background.at<cv::Vec3b>(y, x);
                        for (int c = 0; c < 3; ++c) {
                            bgPixel[c] = static_cast<uchar>(fgPixel[c] * alpha + bgPixel[c] * (1.0f - alpha));
                        }
                    }
                }
            }
        } else {
            foreground.copyTo(background);
        }
    }

    cv::Mat getAnimationFrame() {
        auto now = std::chrono::system_clock::now();
        
        // 1. LIENZO NEGRO BASE
        cv::Mat result = cv::Mat::zeros(eyesOpenImg.size(), CV_8UC3);

        // Calculamos cuánto tiempo ha pasado desde el último evento
        double sinceBlink = std::chrono::duration<double>(now - lastBlinkTime).count();

        // 2. Lógica de Cara Alternativa (Prioridad alta para que no se interrumpa)
        if (showingAltFace) {
            double elapsedAlt = std::chrono::duration<double>(now - altFaceStartTime).count();
            
            if (elapsedAlt > 1.0) { // Duración de la cara alternativa
                showingAltFace = false;
                // RESET CRÍTICO: Ponemos el último parpadeo como "ahora" para que 
                // tenga que esperar otros 4 segundos antes de volver a hacer nada.
                lastBlinkTime = now; 
                currentAltFaceIndex = (currentAltFaceIndex + 1) % altFaces.size();
                return result; // Retornamos negro un frame o dejamos que pase al estado normal
            }
            
            // Si la cara alternativa tiene canales alfa, usamos overlay, si no, copyTo
            if (altFaces[currentAltFaceIndex].channels() == 4) {
                overlayImage(result, altFaces[currentAltFaceIndex]);
            } else {
                altFaces[currentAltFaceIndex].copyTo(result);
            }
            return result; // Si mostramos cara alternativa, no dibujamos ojos/boca normales
        }

        // 3. Lógica de Parpadeo
        if (!isBlinking && sinceBlink > 1.0) { 
            isBlinking = true;
            blinkStartTime = now;
        }

        cv::Mat currentEye;
        if (isBlinking) {
            double elapsedBlink = std::chrono::duration<double>(now - blinkStartTime).count();
            
            if (elapsedBlink > 0.2) {
                isBlinking = false;
                lastBlinkTime = now;
                blinkCount++; 
                
                if (blinkCount >= 1 && !altFaces.empty()) {
                    showingAltFace = true;     
                    altFaceStartTime = now;    
                    blinkCount = 0;
                    // Retornamos rápido para empezar a mostrar la cara nueva en el siguiente frame
                    return result; 
                }
            }
            currentEye = eyesClosedImg;
        } 
        else {
            currentEye = eyesOpenImg; 
        }

        // 4. Lógica de boca (Solo si no hay cara alternativa activa)
        cv::Mat currentMouth = ttsActive ? mouthOpenImg : mouthClosedImg;

        // 5. DIBUJAR SOBRE EL NEGRO
        overlayImage(result, currentEye);
        overlayImage(result, currentMouth);
        
        return result;
    }

    void renderLoop() {
        rclcpp::Rate rate(30);
        while (running && rclcpp::ok()) {
            cv::Mat frame = getAnimationFrame();
            if (!frame.empty()) {
                {
                    std::lock_guard<std::mutex> lock(frameMutex);
                    latestFrame = frame.clone();
                }
                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                faceScreenPublisher->publish(*msg);
            }
            rate.sleep();
        }
    }

    void audioPlayingCallback(const std_msgs::msg::Bool::SharedPtr msg) { ttsActive = msg->data; }

public:
    void drawWindow() {
        std::lock_guard<std::mutex> lock(frameMutex);
        if (!latestFrame.empty()) {
            cv::imshow("Yaren Face", latestFrame);
            cv::waitKey(1);
        }
    }

private:
    cv::Mat eyesOpenImg, eyesClosedImg, mouthOpenImg, mouthClosedImg;
    std::vector<cv::Mat> altFaces;
    size_t currentAltFaceIndex; 
    bool ttsActive, isBlinking, showingAltFace, running;
    int blinkCount; 
    std::chrono::system_clock::time_point lastBlinkTime, blinkStartTime, altFaceStartTime;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ttsSubscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr faceScreenPublisher;
    std::thread renderThread;
    std::mutex frameMutex;
    cv::Mat latestFrame;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoSynchronizer>();
    rclcpp::Rate loop_rate(30);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->drawWindow();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}