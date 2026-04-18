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
    if (current_challenge_ == 0) {
        return false;
    }
    
    int thershold_bt_shoulders = (keypoints[LEFT_SHOULDER].first - keypoints[RIGHT_SHOULDER].first) / 3;
    int thershold_vertical = (keypoints[NOSE].second - keypoints[LEFT_EYE].second) * 2;
    int thershold_touch_eye = (keypoints[LEFT_EYE].first - keypoints[RIGHT_EYE].first) / 1;
    int thershold_touch_elbow = (keypoints[NOSE].second - keypoints[LEFT_EYE].second) * 4;
    
    switch (current_challenge_)
    {
        case 1:
            if (is_above(keypoints[RIGHT_WRIST], keypoints[RIGHT_SHOULDER]) &&
                is_above(keypoints[RIGHT_ELBOW], keypoints[RIGHT_SHOULDER]) &&
                keypoints[RIGHT_WRIST].second != 0 && keypoints[RIGHT_ELBOW].second != 0)
            {
                return true;
            }
            break;
            
        case 2:
            if (is_above(keypoints[LEFT_WRIST], keypoints[LEFT_SHOULDER]) &&
                is_above(keypoints[LEFT_ELBOW], keypoints[LEFT_SHOULDER]) &&
                keypoints[LEFT_WRIST].second != 0 && keypoints[LEFT_ELBOW].second != 0)
            {
                return true;
            }
            break;
            
        case 3:
        {
            bool arm_left_up = (is_above(keypoints[LEFT_WRIST], keypoints[LEFT_SHOULDER]) &&
                               is_above(keypoints[LEFT_ELBOW], keypoints[LEFT_SHOULDER]) &&
                               keypoints[LEFT_WRIST].second != 0 && keypoints[LEFT_ELBOW].second != 0);
                               
            bool arm_right_up = (is_above(keypoints[RIGHT_WRIST], keypoints[RIGHT_SHOULDER]) &&
                                is_above(keypoints[RIGHT_ELBOW], keypoints[RIGHT_SHOULDER]) &&
                                keypoints[RIGHT_WRIST].second != 0 && keypoints[RIGHT_ELBOW].second != 0);
                                
            if (arm_left_up && arm_right_up)
            {
                return true;
            }
            break;
        }
            
        case 4:
            if (is_at_same_height(keypoints[RIGHT_WRIST], keypoints[RIGHT_SHOULDER], 100.0f) &&
                is_in_horizontal_range(keypoints[RIGHT_WRIST], keypoints[RIGHT_SHOULDER], 100.0f))
            {
                return true;
            }
            break;
            
        case 5:
            if (is_at_same_height(keypoints[LEFT_WRIST], keypoints[LEFT_SHOULDER], 100.0f) &&
                is_in_horizontal_range(keypoints[LEFT_WRIST], keypoints[LEFT_SHOULDER], 100.0f))
            {
                return true;
            }
            break;
            
        case 6:
        {
            bool arm_left_forward = (is_at_same_height(keypoints[LEFT_WRIST], keypoints[LEFT_SHOULDER], 100.0f) &&
                                    is_in_horizontal_range(keypoints[LEFT_WRIST], keypoints[LEFT_SHOULDER], 100.0f));
                                    
            bool arm_right_forward = (is_at_same_height(keypoints[RIGHT_WRIST], keypoints[RIGHT_SHOULDER], 100.0f) &&
                                     is_in_horizontal_range(keypoints[RIGHT_WRIST], keypoints[RIGHT_SHOULDER], 100.0f));
                                     
            if (arm_left_forward && arm_right_forward)
            {
                return true;
            }
        }
        break;
        
        case 7:
            if (is_above(keypoints[RIGHT_WRIST], keypoints[RIGHT_SHOULDER]) &&
                is_above(keypoints[LEFT_WRIST], keypoints[LEFT_SHOULDER]) &&
                is_left_of(keypoints[RIGHT_WRIST], keypoints[RIGHT_SHOULDER], thershold_bt_shoulders) &&
                is_right_of(keypoints[LEFT_WRIST], keypoints[LEFT_SHOULDER], thershold_bt_shoulders))
            {
                return true;
            }
            break;

        case 8:
            if (is_near(keypoints[RIGHT_WRIST], keypoints[NOSE], thershold_vertical))
            {
                return true;
            }
            break;
            
        case 9:
            if (is_near(keypoints[LEFT_WRIST], keypoints[NOSE], thershold_vertical))
            {
                return true;
            }
            break;
            
        case 10:
            if (is_near(keypoints[LEFT_WRIST], keypoints[LEFT_EYE], thershold_touch_eye))
            {
                return true;
            }
            break;
            
        case 11:
            if (is_near(keypoints[RIGHT_WRIST], keypoints[LEFT_EYE], thershold_touch_eye))
            {
                return true;
            }
            break;

        case 12:
            if (is_near(keypoints[RIGHT_WRIST], keypoints[RIGHT_EYE], thershold_touch_eye))
            {
                return true;
            }
            break;
            
        case 13:
            if (is_near(keypoints[LEFT_WRIST], keypoints[RIGHT_EYE], thershold_touch_eye))
            {
                return true;
            }
            break;
            
        case 14:
            if (is_near(keypoints[RIGHT_WRIST], keypoints[RIGHT_EAR], thershold_vertical))
            {
                return true;
            }
            break;
            
        case 15:
            if (is_near(keypoints[LEFT_WRIST], keypoints[RIGHT_EAR], thershold_vertical))
            {
                return true;
            }
            break;
            
        case 16:
        {
            bool wrist_right = is_near(keypoints[RIGHT_WRIST], keypoints[RIGHT_EAR], thershold_vertical);
            bool wrist_left = is_near(keypoints[LEFT_WRIST], keypoints[RIGHT_EAR], thershold_vertical);
            
            if (wrist_right && wrist_left)
            {
                return true;
            }
        }
        break;
        
        case 17:
            if (is_near(keypoints[LEFT_WRIST], keypoints[LEFT_EAR], thershold_vertical))
            {
                return true;
            }
            break;
            
        case 18:
            if (is_near(keypoints[RIGHT_WRIST], keypoints[LEFT_EAR], thershold_vertical))
            {
                return true;
            }
            break;
            
        case 19:
        {
            bool wrist_left = is_near(keypoints[LEFT_WRIST], keypoints[LEFT_EAR], thershold_vertical);
            bool wrist_right = is_near(keypoints[RIGHT_WRIST], keypoints[LEFT_EAR], thershold_vertical);
            
            if (wrist_left && wrist_right)
            {
                return true;
            }
        }
        break;
            
        case 20:
            if (is_near(keypoints[RIGHT_WRIST], keypoints[LEFT_SHOULDER], thershold_vertical))
            {
                return true;
            }
            break;
            
        case 21:
            if (is_near(keypoints[LEFT_WRIST], keypoints[RIGHT_SHOULDER], thershold_vertical))
            {
                return true;
            }
            break;
            
        case 22:
            if (is_near(keypoints[RIGHT_WRIST], keypoints[LEFT_ELBOW], thershold_touch_elbow) &&
                keypoints[LEFT_ELBOW].second != 0 && keypoints[RIGHT_WRIST].second != 0)
            {
                return true;
            }
            break;
            
        case 23:
            if (is_near(keypoints[LEFT_WRIST], keypoints[RIGHT_ELBOW], thershold_touch_elbow) &&
                keypoints[RIGHT_ELBOW].second != 0 && keypoints[LEFT_WRIST].second != 0)
            {
                return true;
            }
            break;
            
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
