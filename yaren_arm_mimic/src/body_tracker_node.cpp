#include "body_tracker_node.hpp"
#include <cmath>

BodyTrackerNode::BodyTrackerNode() : Node("body_tracker_node"), last_detection_valid(false) {
    subscription_ = this->create_subscription<yaren_interfaces::msg::BodyPoints>(
        "body_points", 10, 
        std::bind(&BodyTrackerNode::bodyPointsCallback, this, std::placeholders::_1));
    
    publisher_ = this->create_publisher<yaren_interfaces::msg::BodyPosition>("body_tracker", 10);
    
    RCLCPP_INFO(this->get_logger(), "Body tracker node initialized");
}

float BodyTrackerNode::smoothAngle(float new_angle, float prev_angle, float alpha) {
    return alpha * new_angle + (1 - alpha) * prev_angle;
}

float BodyTrackerNode::radian2Euler(float radian) {
    return radian * 180.0 / M_PI;
}

float BodyTrackerNode::calculateAngleWithVertical(float shoulder_x, float shoulder_y, float elbow_x, float elbow_y) {
    float v_x = elbow_x - shoulder_x;
    float v_y = elbow_y - shoulder_y;
    
    return radian2Euler(atan2(-v_x, v_y));
}

float BodyTrackerNode::calculateAngleWithVerticalZY(float shoulder_z, float shoulder_y, float elbow_z, float elbow_y) {
    float v_z = elbow_z - shoulder_z;
    float v_y = elbow_y - shoulder_y;
    
    return radian2Euler(atan2(-v_z, v_y));
}

float BodyTrackerNode::calculateRelativeAngle(float shoulder_x, float shoulder_y,
                           float elbow_x, float elbow_y,
                           float wrist_x, float wrist_y) {
    float v_x = wrist_x - elbow_x;
    float v_y = wrist_y - elbow_y;
    float u_x = elbow_x - shoulder_x;
    float u_y = elbow_y - shoulder_y;
    
    float det_v_u = u_x * v_y - u_y * v_x;
    float dot_v_u = u_x * v_x + u_y * v_y;
    
    return radian2Euler(atan2(det_v_u, dot_v_u));
}

float BodyTrackerNode::calculateRelativeAngleZY(float shoulder_z, float shoulder_y,
                             float elbow_z, float elbow_y,
                             float wrist_z, float wrist_y) {
    float v_z = wrist_z - elbow_z;
    float v_y = wrist_y - elbow_y;
    float u_z = elbow_z - shoulder_z;
    float u_y = elbow_y - shoulder_y;
    
    float det_v_u = u_z * v_y - u_y * v_z;
    float dot_v_u = u_z * v_z + u_y * v_y;
    
    return radian2Euler(atan2(det_v_u, dot_v_u));
}

void BodyTrackerNode::bodyPointsCallback(const yaren_interfaces::msg::BodyPoints::SharedPtr msg) {
    yaren_interfaces::msg::BodyPosition arm_msg;
    arm_msg.is_valid = false;

    if (!msg->is_detected) {
        if (last_detection_valid) {
            last_valid_arm_msg.is_valid = false;
            publisher_->publish(last_valid_arm_msg);
            RCLCPP_WARN(this->get_logger(), "No detection! Using last valid angles but marking as invalid.");
        }
        last_detection_valid = false;
        return;
    }
    
    // Usar la notación de punto (.x, .y, .z) para extraer los datos de Point32
    float angle_shoulder_right_elbow_YX = calculateAngleWithVertical(
        msg->right_shoulder.x, msg->right_shoulder.y,
        msg->right_elbow.x, msg->right_elbow.y
    );
    
    float angle_elbow_right_wrist_YX = calculateRelativeAngle(
        msg->right_shoulder.x, msg->right_shoulder.y,
        msg->right_elbow.x, msg->right_elbow.y,
        msg->right_wrist.x, msg->right_wrist.y
    );
    
    float angle_shoulder_left_elbow_YX = -calculateAngleWithVertical(
        msg->left_shoulder.x, msg->left_shoulder.y,
        msg->left_elbow.x, msg->left_elbow.y
    );
    
    float angle_elbow_left_wrist_YX = calculateRelativeAngle(
        msg->left_shoulder.x, msg->left_shoulder.y,
        msg->left_elbow.x, msg->left_elbow.y,
        msg->left_wrist.x, msg->left_wrist.y
    );
    
    float angle_shoulder_right_elbow_ZY = calculateAngleWithVerticalZY(
        msg->right_shoulder.z, msg->right_shoulder.y,
        msg->right_elbow.z, msg->right_elbow.y
    );
    
    float angle_elbow_right_wrist_ZY = calculateRelativeAngleZY(
        msg->right_shoulder.z, msg->right_shoulder.y,
        msg->right_elbow.z, msg->right_elbow.y,
        msg->right_wrist.z, msg->right_wrist.y
    );
    
    float angle_shoulder_left_elbow_ZY = calculateAngleWithVerticalZY(
        msg->left_shoulder.z, msg->left_shoulder.y,
        msg->left_elbow.z, msg->left_elbow.y
    );
    
    float angle_elbow_left_wrist_ZY = calculateRelativeAngleZY(
        msg->left_shoulder.z, msg->left_shoulder.y,
        msg->left_elbow.z, msg->left_elbow.y,
        msg->left_wrist.z, msg->left_wrist.y
    );
    
    arm_msg.right_shoulder_elbow_yx = angle_shoulder_right_elbow_YX;
    arm_msg.right_elbow_wrist_yx = angle_elbow_right_wrist_YX;
    arm_msg.left_shoulder_elbow_yx = angle_shoulder_left_elbow_YX;
    arm_msg.left_elbow_wrist_yx = angle_elbow_left_wrist_YX;
    
    arm_msg.right_shoulder_elbow_zy = angle_shoulder_right_elbow_ZY;
    arm_msg.right_elbow_wrist_zy = angle_elbow_right_wrist_ZY;
    arm_msg.left_shoulder_elbow_zy = angle_shoulder_left_elbow_ZY;
    arm_msg.left_elbow_wrist_zy = angle_elbow_left_wrist_ZY;
    
    // Coordenadas directas de las muñecas
    arm_msg.right_wrist_x = msg->right_wrist.x;
    arm_msg.right_wrist_y = msg->right_wrist.y;
    arm_msg.left_wrist_x = msg->left_wrist.x;
    arm_msg.left_wrist_y = msg->left_wrist.y;

    // Pasar las rotaciones de las palmas
    arm_msg.right_palm_rotation = msg->right_palm_rotation;
    arm_msg.left_palm_rotation = msg->left_palm_rotation;

    last_valid_arm_msg = arm_msg;
    last_detection_valid = true;
    arm_msg.is_valid = true;
    
    publisher_->publish(arm_msg);
    
    // RCLCPP_INFO(this->get_logger(), "Sending body position message..."); 
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BodyTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}