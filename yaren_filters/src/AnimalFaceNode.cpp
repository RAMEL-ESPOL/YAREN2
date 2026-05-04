#include "AnimalFaceNode.hpp"
// Se eliminaron <termios.h> y <unistd.h> porque ya no leeremos de la consola
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

AnimalFaceNode::AnimalFaceNode() : Node("animal_face_node"), current_filter_("bear") {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    
    image_sub_.subscribe(this, "image_raw");
    landmarks_sub_.subscribe(this, "face_landmarks");
    
    sync_ = std::make_shared<Synchronizer<ApproximateTimePolicy>>(
        ApproximateTimePolicy(10), image_sub_, landmarks_sub_);
    
    sync_->registerCallback(&AnimalFaceNode::callback, this);
    
    image_pub_ = image_transport::create_publisher(this, "filtered_image", qos.get_rmw_qos_profile());
    
    running_ = true;
    
    // NOTA: Se eliminó el hilo del teclado de la terminal. 
    // Ahora OpenCV manejará las teclas directamente en su ventana.
    
    RCLCPP_INFO(get_logger(), "Nodo de filtros animales inicializado");
    RCLCPP_INFO(get_logger(), "Presiona en la ventana de video: 1-Oso | 2-Gato | 3-Mono");
}

void AnimalFaceNode::callback(const ImageMsg::ConstSharedPtr& img_msg, 
                            const Landmarks::ConstSharedPtr& landmarks_msg) {
    try {
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
        
        // --- VISUALIZACIÓN Y LECTURA DE TECLADO ---
        cv::imshow("Animal Filter", frame);
        
        // waitKey devuelve el código ASCII de la tecla presionada.
        // Se le aplica una máscara (& 0xFF) para evitar caracteres raros según el SO.
        int key = cv::waitKey(1) & 0xFF; 
        
        // Si se presionó una tecla válida en tu mapa (ej. '1', '2', '3')
        if (key != 255 && key_bindings_.find(key) != key_bindings_.end()) {
            std::lock_guard<std::mutex> lock(filter_mutex_);
            change_filter(key_bindings_[key]);
            RCLCPP_INFO(get_logger(), "Filtro cambiado a: %s", key_bindings_[key].c_str());
        }
        // ------------------------------------------

        auto output_msg = cv_bridge::CvImage(
            img_msg->header, "bgr8", frame).toImageMsg();
        image_pub_.publish(output_msg);
        
    } catch(const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error en callback: %s", e.what());
    }
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