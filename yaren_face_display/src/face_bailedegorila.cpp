#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <SDL2/SDL.h>        // <-- Nueva inclusión de SDL2
#include <SDL2/SDL_mixer.h>  // <-- Nueva inclusión de SDL_mixer
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
        std::string pkgDir = ament_index_cpp::get_package_share_directory("yaren_face_display");

        // 1. Cargar imágenes base
        eyesOpenImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/eyes_pairs/7.png", cv::IMREAD_UNCHANGED);
        eyesClosedImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/eyes_pairs/3.png", cv::IMREAD_UNCHANGED);
        mouthClosedImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/mouths/13.png", cv::IMREAD_UNCHANGED);
        mouthOpenImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/mouths/8.png", cv::IMREAD_UNCHANGED);

        // 2. Cargar Caras Alternativas
        std::vector<std::string> altFacePaths = {
            pkgDir + "/faces/money.png",
            pkgDir + "/faces/open_mouth.png",
            pkgDir + "/faces/ready.png",
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
    
        // 3. Inicializar el sistema de audio asíncrono con SDL2
        audioCargado = false;
        bgm = nullptr;

        if (SDL_Init(SDL_INIT_AUDIO) < 0) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo inicializar SDL Audio: %s", SDL_GetError());
        } else {
            // Configurar el mixer: 44100Hz, formato estándar, 2 canales (estéreo), 2048 bytes de buffer
            if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 2048) < 0) {
                RCLCPP_ERROR(this->get_logger(), "No se pudo inicializar SDL_mixer: %s", Mix_GetError());
            } else {
                // CAMBIA LA RUTA AQUÍ AL NOMBRE/UBICACIÓN REAL DE TU MP3
                std::string rutaMusica = "/home/roberto/robotis_ws/src/YAREN2/yaren_radio/audios/CantaJuego - El Baile del Gorila.mp3";
                bgm = Mix_LoadMUS(rutaMusica.c_str());
                
                if (!bgm) {
                    RCLCPP_WARN(this->get_logger(), "No se encontró el archivo MP3 en: %s. Error: %s", rutaMusica.c_str(), Mix_GetError());
                } else {
                    audioCargado = true;
                    // El segundo argumento es el número de bucles (-1 significa reproducción infinita)
                    Mix_PlayMusic(bgm, -1);
                    RCLCPP_INFO(this->get_logger(), "🎵 Música de fondo iniciada asíncronamente.");
                }
            }
        }

        // Estado inicial
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

        cv::namedWindow("Yaren Face", cv::WINDOW_NORMAL);
        cv::setWindowProperty("Yaren Face", cv::WND_PROP_AUTOSIZE, cv::WINDOW_AUTOSIZE);
        cv::setWindowProperty("Yaren Face", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        cv::moveWindow("Yaren Face", 0, 0);

        // Hilo de procesamiento de frames
        renderThread = std::thread(&VideoSynchronizer::renderLoop, this);

        RCLCPP_INFO(this->get_logger(), "Initialized Video Synchronizer Node with Audio");
    }

    ~VideoSynchronizer() {
        running = false;
        if (renderThread.joinable()) renderThread.join();
        
        // Detener la música y cerrar los subsistemas de audio de manera limpia
        if (audioCargado) {
            Mix_HaltMusic();
            if (bgm) Mix_FreeMusic(bgm);
            Mix_CloseAudio();
        }
        SDL_Quit();

        cv::destroyAllWindows();
    }

private:
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
        cv::Mat result = cv::Mat::zeros(eyesOpenImg.size(), CV_8UC3);
        double sinceBlink = std::chrono::duration<double>(now - lastBlinkTime).count();

        if (showingAltFace) {
            double elapsedAlt = std::chrono::duration<double>(now - altFaceStartTime).count();
            if (elapsedAlt > 1.0) {
                showingAltFace = false;
                lastBlinkTime = now; 
                currentAltFaceIndex = (currentAltFaceIndex + 1) % altFaces.size();
                return result;
            }
            if (altFaces[currentAltFaceIndex].channels() == 4) {
                overlayImage(result, altFaces[currentAltFaceIndex]);
            } else {
                altFaces[currentAltFaceIndex].copyTo(result);
            }
            return result;
        }

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
                    return result; 
                }
            }
            currentEye = eyesClosedImg;
        } else {
            currentEye = eyesOpenImg; 
        }

        cv::Mat currentMouth = ttsActive ? mouthOpenImg : mouthClosedImg;
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

    // Variables de control para el audio
    bool audioCargado;
    Mix_Music* bgm;
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