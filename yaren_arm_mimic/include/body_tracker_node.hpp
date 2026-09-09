#ifndef BODY_TRACKER_NODE_HPP
#define BODY_TRACKER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "yaren_interfaces/msg/body_points.hpp"
#include "yaren_interfaces/msg/body_position.hpp"

class BodyTrackerNode : public rclcpp_lifecycle::LifecycleNode {
public:
    BodyTrackerNode();

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State&);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State&);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State&);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State&);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State&);

private:
    bool last_detection_valid{false};
    yaren_interfaces::msg::BodyPosition last_valid_arm_msg;

    float smoothAngle(float new_angle, float prev_angle, float alpha = 0.2);
    float radian2Euler(float radian);
    float calculateAngleWithVertical(float sx, float sy, float ex, float ey);
    float calculateAngleWithVerticalZY(float sz, float sy, float ez, float ey);
    float calculateRelativeAngle(float sx, float sy, float ex, float ey, float wx, float wy);
    float calculateRelativeAngleZY(float sz, float sy, float ez, float ey, float wz, float wy);
    void bodyPointsCallback(const yaren_interfaces::msg::BodyPoints::SharedPtr msg);

    rclcpp::Subscription<yaren_interfaces::msg::BodyPoints>::SharedPtr subscription_;
    rclcpp_lifecycle::LifecyclePublisher<yaren_interfaces::msg::BodyPosition>::SharedPtr publisher_;
};

#endif