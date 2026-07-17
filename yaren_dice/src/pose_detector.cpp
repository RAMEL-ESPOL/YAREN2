#include "pose_detector.hpp"

using namespace std::chrono_literals;

YarenPoseDetector::YarenPoseDetector() : Node("yaren_pose_detector")
{
    pose_result_publisher_ = this->create_publisher<yaren_interfaces::msg::PoseResult>(
        "/pose_result", 10);
    
    current_challenge_subscription_ = this->create_subscription<std_msgs::msg::Int16>(
        "/current_challenge", 10,
        std::bind(&YarenPoseDetector::handle_new_challenge, this, std::placeholders::_1));
    
    pose_landmarks_subscription_ = this->create_subscription<yaren_interfaces::msg::Landmarks>(
        "/pose_landmarks", 10,
        std::bind(&YarenPoseDetector::detect_poses, this, std::placeholders::_1));
    
    current_challenge_ = 0;
    game_active_ = false;
    
    NOSE = 0;
    LEFT_EYE = 1;
    RIGHT_EYE = 2;
    LEFT_EAR = 3;
    RIGHT_EAR = 4;
    LEFT_SHOULDER = 5;
    RIGHT_SHOULDER = 6;
    LEFT_ELBOW = 7;
    RIGHT_ELBOW = 8;
    LEFT_WRIST = 9;
    RIGHT_WRIST = 10;
    LEFT_HIP = 11;
    RIGHT_HIP = 12;
        
    RCLCPP_INFO(this->get_logger(), "Yaren Pose Detector started successfully");
}

void YarenPoseDetector::handle_new_challenge(const std_msgs::msg::Int16::SharedPtr msg)
{
    current_challenge_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "New challenge received: %d", current_challenge_);
}

bool YarenPoseDetector::is_above(const std::pair<float, float>& point1, const std::pair<float, float>& point2, float threshold)
{
    return (point2.second - point1.second) > threshold;
}

bool YarenPoseDetector::is_below(const std::pair<float, float>& point1, const std::pair<float, float>& point2, float threshold)
{
    return (0.0f < std::abs(point1.second - point2.second) && std::abs(point1.second - point2.second) <= threshold);
}

bool YarenPoseDetector::is_at_same_height(const std::pair<float, float>& point1, const std::pair<float, float>& point2, float threshold)
{
    return (0.0f < std::abs(point1.second - point2.second) && std::abs(point1.second - point2.second) < threshold);
}

bool YarenPoseDetector::is_in_horizontal_range(const std::pair<float, float>& point1, const std::pair<float, float>& point2, float threshold)
{
    return (0.0f < std::abs(point1.first - point2.first) && std::abs(point1.first - point2.first) < threshold);
}

bool YarenPoseDetector::is_right_of(const std::pair<float, float>& point1, const std::pair<float, float>& point2, float threshold)
{
    return (point2.first - point1.first) > threshold;
}

bool YarenPoseDetector::is_left_of(const std::pair<float, float>& point1, const std::pair<float, float>& point2, float threshold)
{
    return (point1.first - point2.first) > threshold;
}

bool YarenPoseDetector::is_near(const std::pair<float, float>& point1, const std::pair<float, float>& point2, float threshold)
{
    float distance = std::sqrt(std::pow(point1.first - point2.first, 2) + std::pow(point1.second - point2.second, 2));
    return distance < threshold;
}

bool YarenPoseDetector::detect_pose_actions(const std::vector<std::pair<float, float>>& keypoints)
{
    if (current_challenge_ == 0) return false;
    if (keypoints.size() < 13) return false;

    // Umbral dinámico basado en el ancho entre hombros
    float shoulder_width = std::abs(keypoints[LEFT_SHOULDER].first - keypoints[RIGHT_SHOULDER].first);
    float vertical_threshold   = shoulder_width * 0.5f;   // para "arriba"
    float forward_h_threshold  = shoulder_width * 0.6f;   // separación horizontal brazo al frente
    float forward_v_threshold  = shoulder_width * 0.4f;   // diferencia vertical brazo al frente
    float side_h_threshold     = shoulder_width * 0.7f;   // separación horizontal brazo a un lado

    switch (current_challenge_)
    {
        // --- BRAZO DERECHO ARRIBA ---
        case 1:
            return keypoints[RIGHT_WRIST].second != 0 &&
                   keypoints[RIGHT_ELBOW].second != 0 &&
                   is_above(keypoints[RIGHT_WRIST],  keypoints[RIGHT_SHOULDER]) &&
                   is_above(keypoints[RIGHT_ELBOW],  keypoints[RIGHT_SHOULDER]);

        // --- BRAZO IZQUIERDO ARRIBA ---
        case 2:
            return keypoints[LEFT_WRIST].second != 0 &&
                   keypoints[LEFT_ELBOW].second != 0 &&
                   is_above(keypoints[LEFT_WRIST],  keypoints[LEFT_SHOULDER]) &&
                   is_above(keypoints[LEFT_ELBOW],  keypoints[LEFT_SHOULDER]);

        // --- AMBOS BRAZOS ARRIBA ---
        case 3:
            return keypoints[RIGHT_WRIST].second != 0 && keypoints[RIGHT_ELBOW].second != 0 &&
                   keypoints[LEFT_WRIST].second  != 0 && keypoints[LEFT_ELBOW].second  != 0 &&
                   is_above(keypoints[RIGHT_WRIST],  keypoints[RIGHT_SHOULDER]) &&
                   is_above(keypoints[RIGHT_ELBOW],  keypoints[RIGHT_SHOULDER]) &&
                   is_above(keypoints[LEFT_WRIST],   keypoints[LEFT_SHOULDER])  &&
                   is_above(keypoints[LEFT_ELBOW],   keypoints[LEFT_SHOULDER]);

        // --- BRAZO DERECHO HACIA DELANTE ---
        // Muñeca y codo al mismo nivel vertical que el hombro (±threshold),
        // y la muñeca está más al frente (en 2D: cerca horizontalmente del hombro)
        case 4:
            return keypoints[RIGHT_WRIST].second != 0 &&
                   std::abs(keypoints[RIGHT_WRIST].second  - keypoints[RIGHT_SHOULDER].second) < forward_v_threshold &&
                   std::abs(keypoints[RIGHT_ELBOW].second  - keypoints[RIGHT_SHOULDER].second) < forward_v_threshold &&
                   // la muñeca NO está muy lejos lateralmente (está al frente, no al lado)
                   std::abs(keypoints[RIGHT_WRIST].first   - keypoints[RIGHT_SHOULDER].first)  < forward_h_threshold;

        // --- BRAZO IZQUIERDO HACIA DELANTE ---
        case 5:
            return keypoints[LEFT_WRIST].second != 0 &&
                   std::abs(keypoints[LEFT_WRIST].second  - keypoints[LEFT_SHOULDER].second) < forward_v_threshold &&
                   std::abs(keypoints[LEFT_ELBOW].second  - keypoints[LEFT_SHOULDER].second) < forward_v_threshold &&
                   std::abs(keypoints[LEFT_WRIST].first   - keypoints[LEFT_SHOULDER].first)  < forward_h_threshold;

        // --- AMBOS BRAZOS HACIA DELANTE ---
        case 6:
        {
            bool right_fwd = keypoints[RIGHT_WRIST].second != 0 &&
                             std::abs(keypoints[RIGHT_WRIST].second - keypoints[RIGHT_SHOULDER].second) < forward_v_threshold &&
                             std::abs(keypoints[RIGHT_WRIST].first  - keypoints[RIGHT_SHOULDER].first)  < forward_h_threshold;
            bool left_fwd  = keypoints[LEFT_WRIST].second != 0 &&
                             std::abs(keypoints[LEFT_WRIST].second  - keypoints[LEFT_SHOULDER].second)  < forward_v_threshold &&
                             std::abs(keypoints[LEFT_WRIST].first   - keypoints[LEFT_SHOULDER].first)   < forward_h_threshold;
            return right_fwd && left_fwd;
        }

        // --- BRAZO DERECHO A UN LADO ---
        // Muñeca al mismo nivel vertical que el hombro Y separada horizontalmente
        case 7:
            return keypoints[RIGHT_WRIST].second != 0 &&
                   std::abs(keypoints[RIGHT_WRIST].second - keypoints[RIGHT_SHOULDER].second) < forward_v_threshold &&
                   // la muñeca está claramente a la derecha del hombro derecho
                   (keypoints[RIGHT_SHOULDER].first - keypoints[RIGHT_WRIST].first) > side_h_threshold;

        // --- BRAZO IZQUIERDO A UN LADO ---
        case 8:
            return keypoints[LEFT_WRIST].second != 0 &&
                   std::abs(keypoints[LEFT_WRIST].second - keypoints[LEFT_SHOULDER].second) < forward_v_threshold &&
                   // la muñeca está claramente a la izquierda del hombro izquierdo
                   (keypoints[LEFT_WRIST].first - keypoints[LEFT_SHOULDER].first) > side_h_threshold;

        // --- BRAZOS EXTENDIDOS A LOS LADOS ---
        case 9:
        {
            bool right_side = keypoints[RIGHT_WRIST].second != 0 &&
                              std::abs(keypoints[RIGHT_WRIST].second - keypoints[RIGHT_SHOULDER].second) < forward_v_threshold &&
                              (keypoints[RIGHT_SHOULDER].first - keypoints[RIGHT_WRIST].first) > side_h_threshold;
            bool left_side  = keypoints[LEFT_WRIST].second != 0 &&
                              std::abs(keypoints[LEFT_WRIST].second - keypoints[LEFT_SHOULDER].second) < forward_v_threshold &&
                              (keypoints[LEFT_WRIST].first - keypoints[LEFT_SHOULDER].first) > side_h_threshold;
            return right_side && left_side;
        }

        // --- BRAZO DERECHO ARRIBA + BRAZO IZQUIERDO A UN LADO ---
        case 10:
        {
            bool right_up  = keypoints[RIGHT_WRIST].second != 0 &&
                             keypoints[RIGHT_ELBOW].second != 0 &&
                             is_above(keypoints[RIGHT_WRIST], keypoints[RIGHT_SHOULDER]) &&
                             is_above(keypoints[RIGHT_ELBOW], keypoints[RIGHT_SHOULDER]);
            bool left_side = keypoints[LEFT_WRIST].second != 0 &&
                             std::abs(keypoints[LEFT_WRIST].second - keypoints[LEFT_SHOULDER].second) < forward_v_threshold &&
                             (keypoints[LEFT_WRIST].first - keypoints[LEFT_SHOULDER].first) > side_h_threshold;
            return right_up && left_side;
        }

        default:
            break;
    }
    return false;
}

void YarenPoseDetector::detect_poses(const yaren_interfaces::msg::Landmarks::SharedPtr msg)
{
    if (current_challenge_ == 0) return;
    
    std::vector<std::pair<float, float>> person_keypoints;
    for (const auto& landmark : msg->landmarks) 
    {
        float x = landmark.x;  
        float y = landmark.y;  
        person_keypoints.emplace_back(x, y);
    }
    
    auto result_msg = std::make_unique<yaren_interfaces::msg::PoseResult>();
    result_msg->challenge = current_challenge_;
    result_msg->detected_poses = detect_pose_actions(person_keypoints);
    result_msg->timestamp = this->now();
    
    pose_result_publisher_->publish(std::move(result_msg));
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YarenPoseDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
