#include "body_tracker_node.hpp"
#include <cmath>

BodyTrackerNode::BodyTrackerNode()
: rclcpp_lifecycle::LifecycleNode("body_tracker_node") {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BodyTrackerNode::on_configure(const rclcpp_lifecycle::State&)
{
    publisher_ = this->create_publisher<yaren_interfaces::msg::BodyPosition>(
        "body_tracker", 10);
    RCLCPP_INFO(get_logger(), "body_tracker_node configurado.");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BodyTrackerNode::on_activate(const rclcpp_lifecycle::State& state)
{
    LifecycleNode::on_activate(state);
    subscription_ = this->create_subscription<yaren_interfaces::msg::BodyPoints>(
        "body_points", 10,
        std::bind(&BodyTrackerNode::bodyPointsCallback, this, std::placeholders::_1));
    last_detection_valid = false;
    RCLCPP_INFO(get_logger(), "body_tracker_node activo.");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BodyTrackerNode::on_deactivate(const rclcpp_lifecycle::State& state)
{
    LifecycleNode::on_deactivate(state);
    subscription_.reset();
    RCLCPP_INFO(get_logger(), "body_tracker_node desactivado.");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BodyTrackerNode::on_cleanup(const rclcpp_lifecycle::State&)
{
    publisher_.reset();
    RCLCPP_INFO(get_logger(), "body_tracker_node limpiado.");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BodyTrackerNode::on_shutdown(const rclcpp_lifecycle::State&)
{
    subscription_.reset();
    publisher_.reset();
    return CallbackReturn::SUCCESS;
}

// ── Métodos de cálculo (sin cambios) ─────────────────────────────────────────

float BodyTrackerNode::smoothAngle(float new_angle, float prev_angle, float alpha) {
    return alpha * new_angle + (1 - alpha) * prev_angle;
}

float BodyTrackerNode::radian2Euler(float radian) {
    return radian * 180.0 / M_PI;
}

float BodyTrackerNode::calculateAngleWithVertical(
    float sx, float sy, float ex, float ey) {
    return radian2Euler(atan2(-(ex - sx), ey - sy));
}

float BodyTrackerNode::calculateAngleWithVerticalZY(
    float sz, float sy, float ez, float ey) {
    return radian2Euler(atan2(-(ez - sz), ey - sy));
}

float BodyTrackerNode::calculateRelativeAngle(
    float sx, float sy, float ex, float ey, float wx, float wy) {
    float vx = wx - ex, vy = wy - ey;
    float ux = ex - sx, uy = ey - sy;
    return radian2Euler(atan2(ux*vy - uy*vx, ux*vx + uy*vy));
}

float BodyTrackerNode::calculateRelativeAngleZY(
    float sz, float sy, float ez, float ey, float wz, float wy) {
    float vz = wz - ez, vy = wy - ey;
    float uz = ez - sz, uy = ey - sy;
    return radian2Euler(atan2(uz*vy - uy*vz, uz*vz + uy*vy));
}

void BodyTrackerNode::bodyPointsCallback(
    const yaren_interfaces::msg::BodyPoints::SharedPtr msg)
{
    yaren_interfaces::msg::BodyPosition arm_msg;
    arm_msg.is_valid = false;

    if (!msg->is_detected) {
        if (last_detection_valid) {
            last_valid_arm_msg.is_valid = false;
            publisher_->publish(last_valid_arm_msg);
        }
        last_detection_valid = false;
        return;
    }

    arm_msg.right_shoulder_elbow_yx = calculateAngleWithVertical(
        msg->right_shoulder.x, msg->right_shoulder.y,
        msg->right_elbow.x,    msg->right_elbow.y);
    arm_msg.right_elbow_wrist_yx = calculateRelativeAngle(
        msg->right_shoulder.x, msg->right_shoulder.y,
        msg->right_elbow.x,    msg->right_elbow.y,
        msg->right_wrist.x,    msg->right_wrist.y);
    arm_msg.left_shoulder_elbow_yx = -calculateAngleWithVertical(
        msg->left_shoulder.x, msg->left_shoulder.y,
        msg->left_elbow.x,    msg->left_elbow.y);
    arm_msg.left_elbow_wrist_yx = calculateRelativeAngle(
        msg->left_shoulder.x, msg->left_shoulder.y,
        msg->left_elbow.x,    msg->left_elbow.y,
        msg->left_wrist.x,    msg->left_wrist.y);
    arm_msg.right_shoulder_elbow_zy = calculateAngleWithVerticalZY(
        msg->right_shoulder.z, msg->right_shoulder.y,
        msg->right_elbow.z,    msg->right_elbow.y);
    arm_msg.right_elbow_wrist_zy = calculateRelativeAngleZY(
        msg->right_shoulder.z, msg->right_shoulder.y,
        msg->right_elbow.z,    msg->right_elbow.y,
        msg->right_wrist.z,    msg->right_wrist.y);
    arm_msg.left_shoulder_elbow_zy = calculateAngleWithVerticalZY(
        msg->left_shoulder.z, msg->left_shoulder.y,
        msg->left_elbow.z,    msg->left_elbow.y);
    arm_msg.left_elbow_wrist_zy = calculateRelativeAngleZY(
        msg->left_shoulder.z, msg->left_shoulder.y,
        msg->left_elbow.z,    msg->left_elbow.y,
        msg->left_wrist.z,    msg->left_wrist.y);

    arm_msg.right_wrist_x      = msg->right_wrist.x;
    arm_msg.right_wrist_y      = msg->right_wrist.y;
    arm_msg.left_wrist_x       = msg->left_wrist.x;
    arm_msg.left_wrist_y       = msg->left_wrist.y;
    arm_msg.right_palm_rotation = msg->right_palm_rotation;
    arm_msg.left_palm_rotation  = msg->left_palm_rotation;
    arm_msg.is_valid            = true;

    last_valid_arm_msg   = arm_msg;
    last_detection_valid = true;
    publisher_->publish(arm_msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BodyTrackerNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}