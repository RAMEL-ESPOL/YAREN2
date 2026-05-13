#pragma once
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include "mask/AnimalFilter.hpp"
#include "yaren_interfaces/msg/landmarks.hpp"
#include <mutex>
#include <atomic>
#include <string>
#include <opencv2/core.hpp>

class AnimalFaceNode : public rclcpp::Node
{
public:
    AnimalFaceNode();

    // Cambia filtro de forma thread-safe (llamado desde main)
    void set_filter(const std::string& animal);

    // main() lee el último frame procesado para mostrarlo
    bool get_last_frame(cv::Mat& out);

    // true cuando el usuario hizo clic en la ventana de cámara
    std::atomic<bool> cam_clicked_{ false };

    // === NUEVO: Publicador para avisar al face_screen ===
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using Landmarks = yaren_interfaces::msg::Landmarks;
    using ApproximateTimePolicy =
        message_filters::sync_policies::ApproximateTime<ImageMsg, Landmarks>;
    using Synchronizer = message_filters::Synchronizer<ApproximateTimePolicy>;

    message_filters::Subscriber<ImageMsg> image_sub_;
    message_filters::Subscriber<Landmarks> landmarks_sub_;
    std::shared_ptr<Synchronizer> sync_;
    image_transport::Publisher image_pub_;

    AnimalFilter current_filter_;
    std::mutex filter_mutex_; // protege current_filter_ Y last_frame_
    cv::Mat last_frame_; 
    bool has_frame_{ false };

    void callback(const ImageMsg::ConstSharedPtr& img_msg,
                  const Landmarks::ConstSharedPtr& landmarks_msg);
};