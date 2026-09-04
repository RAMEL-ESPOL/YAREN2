#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaren_interfaces/msg/landmarks.hpp>
#include <opencv2/core.hpp>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <memory>
#include <chrono>
#include <string>
#include "mask/AnimalFilter.hpp"

using ImageMsg  = sensor_msgs::msg::Image;
using Landmarks = yaren_interfaces::msg::Landmarks;
using ApproximateTimePolicy =
    message_filters::sync_policies::ApproximateTime<ImageMsg, Landmarks>;
using Synchronizer   = message_filters::Synchronizer<ApproximateTimePolicy>;
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Forward declarations para no incluir las clases completas aquí
class TongueDetector;
class VirtualBackground;

class AnimalFaceNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit AnimalFaceNode();

    // ── Lifecycle callbacks ────────────────────────────────────
    CallbackReturn on_configure (const rclcpp_lifecycle::State&) override;
    CallbackReturn on_activate  (const rclcpp_lifecycle::State&) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_cleanup   (const rclcpp_lifecycle::State&) override;
    CallbackReturn on_shutdown  (const rclcpp_lifecycle::State&) override;

    // ── API pública ────────────────────────────────────────────
    void set_filter(const std::string& animal);
    bool get_last_frame(cv::Mat& out);

    std::atomic<bool> is_english_       { false };
    std::atomic<bool> language_received_{ false };
    std::atomic<bool> cam_clicked_      { false };

private:
    void callback(const ImageMsg::ConstSharedPtr&,
                  const Landmarks::ConstSharedPtr&);
    void run_ui();
    void load_animal_assets(const std::string& animal);
    void stop_sound();
    void play_sound(bool turn_on);

    message_filters::Subscriber<ImageMsg,  rclcpp_lifecycle::LifecycleNode> image_sub_;
    message_filters::Subscriber<Landmarks, rclcpp_lifecycle::LifecycleNode> landmarks_sub_;
    std::shared_ptr<Synchronizer> sync_;

    rclcpp::Publisher<ImageMsg>::SharedPtr image_pub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr  language_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   mode_pub_;

    AnimalFilter current_filter_;
    std::mutex   filter_mutex_;
    cv::Mat      last_frame_;
    bool         has_frame_{ false };

    std::vector<cv::Mat> previews_;
    pid_t sound_pid_ = -1;

    std::thread       ui_thread_;
    std::atomic<bool> ui_running_{ false };

    std::unique_ptr<TongueDetector>    tongue_detector_;
    std::unique_ptr<VirtualBackground> virtual_bg_;
    std::string                        current_sound_path_;
};