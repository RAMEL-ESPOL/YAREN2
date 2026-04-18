#include "body_tracker_node.hpp"
#include <cmath>

BodyTrackerNode::BodyTrackerNode()
: Node("body_tracker_node"), last_detection_valid(false)
{
    subscription_ = this->create_subscription<yaren_interfaces::msg::BodyPoints>(
        "body_points", 10,
        std::bind(&BodyTrackerNode::bodyPointsCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<yaren_interfaces::msg::BodyPosition>("body_tracker", 10);

    publish_timer_ = this->create_wall_timer(publish_period_, std::bind(&BodyTrackerNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Body tracker node initialized (publish period: %ld ms)", publish_period_.count());
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
    last_body_points_ = msg;
    has_body_points_ = true;
}

void BodyTrackerNode::timerCallback() {
    if (!has_body_points_) return;

    auto msg = last_body_points_;
    yaren_interfaces::msg::BodyPosition arm_msg = last_valid_arm_msg;
    arm_msg.is_valid = false;

    if (!msg->is_detected) {
        if (last_detection_valid) {
            last_valid_arm_msg.is_valid = false;
            publisher_->publish(last_valid_arm_msg);
            RCLCPP_WARN(this->get_logger(),
                        "No detection! Using last valid angles but marking as invalid.");
        }
        last_detection_valid = false;
        return;
    }

    float new_right_shoulder_elbow_yx = truncf(calculateAngleWithVertical(
        msg->right_shoulder.x, msg->right_shoulder.y,
        msg->right_elbow.x,    msg->right_elbow.y));

    float new_right_elbow_wrist_yx = truncf(calculateRelativeAngle(
        msg->right_shoulder.x, msg->right_shoulder.y,
        msg->right_elbow.x,    msg->right_elbow.y,
        msg->right_wrist.x,    msg->right_wrist.y));

    float new_left_shoulder_elbow_yx = truncf(-calculateAngleWithVertical(
        msg->left_shoulder.x,  msg->left_shoulder.y,
        msg->left_elbow.x,     msg->left_elbow.y));

    float new_left_elbow_wrist_yx = truncf(-calculateRelativeAngle(
        msg->left_shoulder.x,  msg->left_shoulder.y,
        msg->left_elbow.x,     msg->left_elbow.y,
        msg->left_wrist.x,     msg->left_wrist.y));

    float new_right_shoulder_elbow_zy = truncf(calculateAngleWithVerticalZY(
        msg->right_shoulder.z, msg->right_shoulder.y,
        msg->right_elbow.z,    msg->right_elbow.y));

    float new_right_elbow_wrist_zy = truncf(calculateRelativeAngleZY(
        msg->right_shoulder.z, msg->right_shoulder.y,
        msg->right_elbow.z,    msg->right_elbow.y,
        msg->right_wrist.z,    msg->right_wrist.y));

    float new_left_shoulder_elbow_zy = truncf(calculateAngleWithVerticalZY(
        msg->left_shoulder.z,  msg->left_shoulder.y,
        msg->left_elbow.z,     msg->left_elbow.y));

    float new_left_elbow_wrist_zy = truncf(calculateRelativeAngleZY(
        msg->left_shoulder.z,  msg->left_shoulder.y,
        msg->left_elbow.z,     msg->left_elbow.y,
        msg->left_wrist.z,     msg->left_wrist.y));

    bool anyChanged = false;    

    if (std::fabs(new_right_shoulder_elbow_yx - last_valid_arm_msg.right_shoulder_elbow_yx) > angle_variation_threshold_) {
        arm_msg.right_shoulder_elbow_yx = new_right_shoulder_elbow_yx;
        anyChanged = true;
    }
    if (std::fabs(new_right_elbow_wrist_yx - last_valid_arm_msg.right_elbow_wrist_yx) > angle_variation_threshold_) {
        arm_msg.right_elbow_wrist_yx = new_right_elbow_wrist_yx;
        anyChanged = true;
    }
    if (std::fabs(new_left_shoulder_elbow_yx - last_valid_arm_msg.left_shoulder_elbow_yx) > angle_variation_threshold_) {
        arm_msg.left_shoulder_elbow_yx = new_left_shoulder_elbow_yx;
        anyChanged = true;
    }
    if (std::fabs(new_left_elbow_wrist_yx - last_valid_arm_msg.left_elbow_wrist_yx) > angle_variation_threshold_) {
        arm_msg.left_elbow_wrist_yx = new_left_elbow_wrist_yx;
        anyChanged = true;
    }
    if (std::fabs(new_right_shoulder_elbow_zy - last_valid_arm_msg.right_shoulder_elbow_zy) > angle_variation_threshold_) {
        arm_msg.right_shoulder_elbow_zy = new_right_shoulder_elbow_zy;
        anyChanged = true;
    }
    if (std::fabs(new_right_elbow_wrist_zy - last_valid_arm_msg.right_elbow_wrist_zy) > angle_variation_threshold_) {
        arm_msg.right_elbow_wrist_zy = new_right_elbow_wrist_zy;
        anyChanged = true;
    }
    if (std::fabs(new_left_shoulder_elbow_zy - last_valid_arm_msg.left_shoulder_elbow_zy) > angle_variation_threshold_) {
        arm_msg.left_shoulder_elbow_zy = new_left_shoulder_elbow_zy;
        anyChanged = true;
    }
    if (std::fabs(new_left_elbow_wrist_zy - last_valid_arm_msg.left_elbow_wrist_zy) > angle_variation_threshold_) {
        arm_msg.left_elbow_wrist_zy = new_left_elbow_wrist_zy;
        anyChanged = true;
    }

    if (!anyChanged) {
        return;
    }

    arm_msg.right_wrist_x = msg->right_wrist.x;
    arm_msg.right_wrist_y = msg->right_wrist.y;
    arm_msg.left_wrist_x = msg->left_wrist.x;
    arm_msg.left_wrist_y = msg->left_wrist.y;

    last_valid_arm_msg = arm_msg;
    last_detection_valid = true;
    arm_msg.is_valid = true;

    publisher_->publish(arm_msg);
    
    RCLCPP_INFO(this->get_logger(), "Sending body position message...");
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BodyTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}