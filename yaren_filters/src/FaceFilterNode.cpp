#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaren_interfaces/msg/landmarks.hpp"
#include "filters/GlassesFilter.hpp"
#include "filters/MouthFilter.hpp"
#include "filters/NoseFilter.hpp"
#include "filters/HatFilter.hpp"
#include "filters/FaceMaskFilter.hpp"
#include <atomic>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <opencv2/highgui.hpp> // NECESARIO PARA MOSTRAR VENTANA

using namespace std::chrono_literals;
using namespace message_filters;
using ImageMsg = sensor_msgs::msg::Image;
using Landmarks = yaren_interfaces::msg::Landmarks;
typedef sync_policies::ApproximateTime<ImageMsg, Landmarks> ApproximateTimePolicy;

class FaceFilterNode : public rclcpp::Node {
public:
    FaceFilterNode() : Node("face_filter_node"), 
                      mask_mode_(false),
                      running_(true) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        
        std::string assets_path = ament_index_cpp::get_package_share_directory("yaren_filters") + "/imgs";
        glasses_filter_ = std::make_shared<GlassesFilter>(assets_path + "/glasses");
        mouth_filter_ = std::make_shared<MouthFilter>(assets_path + "/mouths");
        nose_filter_ = std::make_shared<NoseFilter>(assets_path + "/noses");
        hat_filter_ = std::make_shared<HatFilter>(assets_path + "/hats");
        face_mask_filter_ = std::make_shared<FaceMaskFilter>(assets_path + "/faces");
        
        image_sub_.subscribe(this, "image_raw");
        landmarks_sub_.subscribe(this, "face_landmarks");
        
        sync_ = std::make_shared<message_filters::Synchronizer<ApproximateTimePolicy>>(
            ApproximateTimePolicy(10), image_sub_, landmarks_sub_
        );
        sync_->registerCallback(
            std::bind(&FaceFilterNode::callback, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2)
        );
        
        image_pub_ = image_transport::create_publisher(this, "filtered_image", qos.get_rmw_qos_profile());
        
        // COMENTA ESTAS DOS LÍNEAS:
        // keyboard_thread_ = std::thread(&FaceFilterNode::keyboardListener, this);
        // keyboard_thread_.detach(); 
        
        RCLCPP_INFO(this->get_logger(), "Nodo inicializado. Modo normal activado");
    }

    // Destructor simplificado (el hilo está detached)
    ~FaceFilterNode() {
        running_ = false;
        cv::destroyAllWindows();
    }

private:
    void callback(const ImageMsg::ConstSharedPtr& img_msg, 
                  const Landmarks::ConstSharedPtr& landmarks_msg) {
        try {
            cv::Mat frame;
            try {
                frame = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            
            // 1. Procesar Landmarks
            std::vector<cv::Point2f> landmarks;
            for (const auto& point : landmarks_msg->landmarks) {
                landmarks.emplace_back(
                    point.x * frame.cols,
                    point.y * frame.rows
                );
            }
            
            // 2. Lógica de filtros y espejo
            if (!landmarks.empty()) {
                std::lock_guard<std::mutex> lock(mutex_);
                cv::flip(frame, frame, 1); // Efecto espejo para PC
                
                if(mask_mode_) {
                    frame = face_mask_filter_->applyFilter(frame, landmarks, frame.size());
                } else {
                    frame = glasses_filter_->applyFilter(frame, landmarks, frame.size());
                    frame = mouth_filter_->applyFilter(frame, landmarks, frame.size());
                    frame = nose_filter_->applyFilter(frame, landmarks, frame.size());
                    frame = hat_filter_->applyFilter(frame, landmarks, frame.size());
                }
            } else {
                 cv::flip(frame, frame, 1);
            }

            // --- 3. VISUALIZACIÓN Y CONTROL POR TECLADO ---
            cv::imshow("Face Filter", frame);
            int key = cv::waitKey(1); // Captura la tecla en la ventana activa

            if (key > 0) {
                std::lock_guard<std::mutex> lock(mutex_);
                char c = (char)key;
                
                // Lógica de cambio de modo y selección de accesorios
                if (c == 'n' || c == 'm') {
                    // Si ya estamos en modo máscara, m/n cambian la cara. 
                    // Si no, activamos el modo máscara.
                    if (!mask_mode_) {
                        mask_mode_ = true;
                        RCLCPP_INFO(get_logger(), "Modo Máscara Activado");
                    }
                    
                    if (c == 'n') decrementIndex(face_mask_filter_);
                    else incrementIndex(face_mask_filter_);
                } 
                else if (c == ' ') { // Barra espaciadora para volver al modo normal
                    mask_mode_ = false;
                    RCLCPP_INFO(get_logger(), "Modo Accesorios Activado");
                }
                else if (!mask_mode_) {
                    // Solo procesamos estas teclas si NO estamos en modo máscara completa
                    handleNormalMode(c);
                }
            }
            // ----------------------------------------------
            
            auto output_msg = cv_bridge::CvImage(img_msg->header, "bgr8", frame).toImageMsg();
            image_pub_.publish(output_msg);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error en callback: %s", e.what());
        }
    }

    void keyboardListener() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        // Usamos select para no bloquear y poder salir con Ctrl+C limpiamente
        while (rclcpp::ok() && running_) {
            fd_set set;
            struct timeval timeout;
            FD_ZERO(&set);
            FD_SET(STDIN_FILENO, &set);
            timeout.tv_sec = 0;
            timeout.tv_usec = 100000; // 100ms

            int res = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
            if (res > 0) {
                char key = getchar();
                std::lock_guard<std::mutex> lock(mutex_);
                
                if(mask_mode_) {
                    handleMaskMode(key);
                } else {
                    handleNormalMode(key);
                }
            }
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

    void handleNormalMode(char key) {
        switch(key) {
            case 'e': incrementIndex(hat_filter_); break;
            case 'q': decrementIndex(hat_filter_); break;
            case 'w': incrementIndex(nose_filter_); break;
            case 's': decrementIndex(nose_filter_); break;
            case 'd': incrementIndex(glasses_filter_); break;
            case 'a': decrementIndex(glasses_filter_); break;
            case 'c': incrementIndex(mouth_filter_); break;
            case 'z': decrementIndex(mouth_filter_); break;
            case 'n': case 'm': 
                mask_mode_ = true;
                RCLCPP_INFO(get_logger(), "Modo máscara activado");
                break;
        }
    }

    void handleMaskMode(char key) {
        switch(key) {
            case 'n': decrementIndex(face_mask_filter_); break;
            case 'm': incrementIndex(face_mask_filter_); break;
            default: 
                mask_mode_ = false;
                RCLCPP_INFO(get_logger(), "Modo normal activado");
                break;
        }
    }

    template<typename T>
    void incrementIndex(std::shared_ptr<T> filter) {
        if(filter->getAssetsSize() > 0) {
            filter->incrementIndex();
            RCLCPP_INFO(get_logger(), "Índice actualizado: %zu", filter->getCurrentIndex());
        }
    }

    template<typename T>
    void decrementIndex(std::shared_ptr<T> filter) {
        if(filter->getAssetsSize() > 0) {
            filter->decrementIndex();
            RCLCPP_INFO(get_logger(), "Índice actualizado: %zu", filter->getCurrentIndex());
        }
    }

    std::atomic<bool> mask_mode_;
    std::atomic<bool> running_;
    std::thread keyboard_thread_;
    std::mutex mutex_;

    std::shared_ptr<GlassesFilter> glasses_filter_;
    std::shared_ptr<MouthFilter> mouth_filter_;
    std::shared_ptr<NoseFilter> nose_filter_;
    std::shared_ptr<HatFilter> hat_filter_;
    std::shared_ptr<FaceMaskFilter> face_mask_filter_;
    
    image_transport::Publisher image_pub_;
    Subscriber<ImageMsg> image_sub_;
    Subscriber<Landmarks> landmarks_sub_;
    std::shared_ptr<message_filters::Synchronizer<ApproximateTimePolicy>> sync_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FaceFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
