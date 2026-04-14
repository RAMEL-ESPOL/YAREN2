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

namespace fs = std::filesystem;

class VideoSynchronizer : public rclcpp::Node {
public:
    VideoSynchronizer() : Node("face_screen") {
        std::string pkgDir = ament_index_cpp::get_package_share_directory("yaren_face_display");

        // 1. Cargar imágenes base (Asegúrate de que estos nombres existan en /imgs)
        eyesOpenImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/eyes_pairs/7.png", cv::IMREAD_UNCHANGED);
        eyesClosedImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/eyes_pairs/3.png", cv::IMREAD_UNCHANGED);
        mouthClosedImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/mouths/13.png", cv::IMREAD_UNCHANGED);
        mouthOpenImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/mouths/8.png", cv::IMREAD_UNCHANGED);

        // 2. Cargar secuencias de frames
        loadFrames(pkgDir + "/faces/transitions/blinking_frames", eyesFrames);
        loadFrames(pkgDir + "/faces/transitions/blinking_frames", mouthFrames);

        // 3. Inicializar variables de estado
        ttsActive = false;
        isBlinking = false;
        lastBlinkTime = std::chrono::system_clock::now();
        running = true;

        // 4. Suscripción al audio
        ttsSubscription = this->create_subscription<std_msgs::msg::Bool>(
            "/audio_playing", 10,
            std::bind(&VideoSynchronizer::audioPlayingCallback, this, std::placeholders::_1));

        // 5. Publicador de imagen (para RViz o web)
        faceScreenPublisher = this->create_publisher<sensor_msgs::msg::Image>("/face_screen", 10);

            // --- Dentro del constructor de VideoSynchronizer ---

    // 1. Crear la ventana con nombre exacto
    cv::namedWindow("Yaren Face", cv::WINDOW_NORMAL);

    // 2. Quitar bordes y decoraciones de la ventana (Modo nativo)
    cv::setWindowProperty("Yaren Face", cv::WINDOW_FULLSCREEN, cv::WINDOW_FULLSCREEN);

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
    // --- LÓGICA DE CARGA ---
    void loadFrames(const std::string& dir, std::vector<cv::Mat>& frames) {
        if (!fs::exists(dir)) {
            RCLCPP_ERROR(this->get_logger(), "Directorio no existe: %s", dir.c_str());
            return;
        }
        std::vector<std::string> files;
        for (const auto& entry : fs::directory_iterator(dir)) {
            if (entry.path().extension() == ".png") files.push_back(entry.path().string());
        }
        std::sort(files.begin(), files.end());
        for (const auto& f : files) {
            cv::Mat frame = cv::imread(f);
            if (!frame.empty()) frames.push_back(frame);
        }
        RCLCPP_INFO(this->get_logger(), "Cargados %zu frames de %s", frames.size(), dir.c_str());
    }

    // --- CALLBACKS ---
    void audioPlayingCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        ttsActive = msg->data;
    }

    // --- LÓGICA DE ANIMACIÓN ---
    double easeInOut(double x) {
        return x < 0.5 ? 2 * x * x : 1 - std::pow(-2 * x + 2, 2) / 2;
    }

        cv::Mat getAnimationFrame() {
        auto now = std::chrono::system_clock::now();
        
        // 1. Crear lienzo negro (3 canales para mostrar en pantalla)
        // Usamos el tamaño de eyesOpenImg pero forzamos 3 canales (CV_8UC3)
        cv::Mat result = cv::Mat::zeros(eyesOpenImg.size(), CV_8UC3);

        // Lógica de parpadeo (se mantiene igual)
        double sinceBlink = std::chrono::duration<double>(now - lastBlinkTime).count();
        if (!isBlinking && sinceBlink > 4.0) {
            isBlinking = true;
            blinkStartTime = now;
            lastBlinkTime = now;
        }

        cv::Mat currentEye;
        if (isBlinking) {
            double elapsed = std::chrono::duration<double>(now - blinkStartTime).count();
            if (elapsed > 0.2) isBlinking = false;
            currentEye = eyesClosedImg;
        } else {
            currentEye = eyesOpenImg;
        }

        cv::Mat currentMouth = ttsActive ? mouthOpenImg : mouthClosedImg;

        // 2. Superponer capas sobre el fondo negro
        overlayImage(result, currentEye);
        overlayImage(result, currentMouth);

        return result;
}

    // --- BUCLE DE RENDERIZADO ---
    void renderLoop() {
        rclcpp::Rate rate(30);
        while (running && rclcpp::ok()) {
            cv::Mat frame = getAnimationFrame();
            if (!frame.empty()) {
                {
                    std::lock_guard<std::mutex> lock(frameMutex);
                    latestFrame = frame.clone();
                }
                // Publicar a ROS
                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                faceScreenPublisher->publish(*msg);
            }
            rate.sleep();
        }
    }
void overlayImage(cv::Mat& background, const cv::Mat& foreground) {
    if (foreground.empty()) return;

    // Si la imagen tiene canal Alfa (4 canales)
    if (foreground.channels() == 4) {
        for (int y = 0; y < background.rows; ++y) {
            for (int x = 0; x < background.cols; ++x) {
                cv::Vec4b& fgPixel = const_cast<cv::Mat&>(foreground).at<cv::Vec4b>(y, x);
                float alpha = fgPixel[3] / 255.0f; // Opacidad de 0.0 a 1.0

                if (alpha > 0) {
                    cv::Vec3b& bgPixel = background.at<cv::Vec3b>(y, x);
                    // Mezcla BGR basada en el alfa
                    for (int c = 0; c < 3; ++c) {
                        bgPixel[c] = static_cast<uchar>(fgPixel[c] * alpha + bgPixel[c] * (1.0f - alpha));
                    }
                }
            }
        }
    } else {
        // Si no tiene alfa, simplemente copiar (fallback)
        foreground.copyTo(background);
    }
}
public:
    // Esta función debe llamarse desde el hilo principal (main)
    void drawWindow() {
        std::lock_guard<std::mutex> lock(frameMutex);
        if (!latestFrame.empty()) {
            cv::imshow("Yaren Face", latestFrame);
            cv::waitKey(1);
        }
    }

private:
    cv::Mat eyesOpenImg, eyesClosedImg, mouthOpenImg, mouthClosedImg;
    std::vector<cv::Mat> eyesFrames, mouthFrames;
    bool ttsActive, isBlinking, running;
    std::chrono::system_clock::time_point lastBlinkTime, blinkStartTime;
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ttsSubscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr faceScreenPublisher;
    
    std::thread renderThread;
    std::mutex frameMutex;
    cv::Mat latestFrame;
};

// --- MAIN CORREGIDO PARA EVITAR TRABAS ---
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoSynchronizer>();

    rclcpp::Rate loop_rate(30);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node); // Procesa mensajes de ROS
        node->drawWindow();      // OpenCV dibuja en el hilo principal
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}