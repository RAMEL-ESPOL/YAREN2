#include "AnimalFaceNode.hpp"
#include <termios.h>
#include <unistd.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h> // Necesario para la conversion correcta

AnimalFaceNode::AnimalFaceNode() : Node("animal_face_node"), current_filter_("bear") {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    
    image_sub_.subscribe(this, "image_raw");
    landmarks_sub_.subscribe(this, "face_landmarks");
    
    sync_ = std::make_shared<Synchronizer<ApproximateTimePolicy>>(
        ApproximateTimePolicy(10), image_sub_, landmarks_sub_);
    
    sync_->registerCallback(&AnimalFaceNode::callback, this);
    
    image_pub_ = image_transport::create_publisher(this, "filtered_image", qos.get_rmw_qos_profile());
    
    running_ = true;
    keyboard_thread_ = std::thread(&AnimalFaceNode::keyboard_listener, this);
    keyboard_thread_.detach();
    
    RCLCPP_INFO(get_logger(), "Nodo de filtros animales inicializado");
    RCLCPP_INFO(get_logger(), "Teclas: 1-Oso | 2-Gato | 3-Mono");
}

void AnimalFaceNode::callback(const ImageMsg::ConstSharedPtr& img_msg, 
                            const Landmarks::ConstSharedPtr& landmarks_msg) {
    try {
        // --- CORRECCIÓN AQUÍ ---
        // Antes intentabas convertir YUV manualmente. Ahora leemos BGR directo.
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        // Procesar landmarks
        std::vector<cv::Point2f> landmarks;
        for(const auto& point : landmarks_msg->landmarks) {
            landmarks.emplace_back(
                point.x * frame.cols,
                point.y * frame.rows
            );
        }
        
        {
            std::lock_guard<std::mutex> lock(filter_mutex_);
            // Espejo horizontal
            cv::flip(frame, frame, 1);
            frame = current_filter_.apply_filter(frame, landmarks);
        }
        
        // --- VISUALIZACIÓN ---
        cv::imshow("Animal Filter", frame);
        cv::waitKey(1); 
        // ---------------------

        auto output_msg = cv_bridge::CvImage(
            img_msg->header, "bgr8", frame).toImageMsg();
        image_pub_.publish(output_msg);
        
    } catch(const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error en callback: %s", e.what());
    }
}

void AnimalFaceNode::keyboard_listener() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while(rclcpp::ok() && running_) {
        fd_set set;
        struct timeval timeout;
        FD_ZERO(&set);
        FD_SET(STDIN_FILENO, &set);
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; // 100ms

        int res = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
        if (res > 0) {
            int key = getchar();
            if(key_bindings_.find(key) != key_bindings_.end()) {
                std::lock_guard<std::mutex> lock(filter_mutex_);
                change_filter(key_bindings_[key]);
                RCLCPP_INFO(get_logger(), "Filtro cambiado a: %s", key_bindings_[key].c_str());
            }
        }
    }
    
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

void AnimalFaceNode::change_filter(const std::string& animal) {
    try {
        current_filter_ = AnimalFilter(animal);
    } catch(const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error cambiando filtro: %s", e.what());
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AnimalFaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}