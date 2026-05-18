#ifndef BODY_TRACKER_NODE_HPP
#define BODY_TRACKER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "yaren_interfaces/msg/body_points.hpp"
#include "yaren_interfaces/msg/body_position.hpp"
#include "kinematic_validator.hpp"

class BodyTrackerNode : public rclcpp::Node {
public:
    BodyTrackerNode();

private:
    bool last_detection_valid;
    yaren_interfaces::msg::BodyPosition last_valid_arm_msg;
    
    // Validador cinemático con parámetros antropométricos
    kinematic_validator::HumanLimbParams human_params_;
    kinematic_validator::RobotJointLimits robot_limits_;
    
    // Estados previos para suavizado
    kinematic_validator::SphericalAngles last_right_shoulder_angles_;
    kinematic_validator::SphericalAngles last_right_elbow_angles_;
    kinematic_validator::SphericalAngles last_left_shoulder_angles_;
    kinematic_validator::SphericalAngles last_left_elbow_angles_;
    
    // Métodos antiguos (deprecados pero mantenidos para compatibilidad)
    float smoothAngle(float new_angle, float prev_angle, float alpha = 0.2);
    float radian2Euler(float radian);
    float calculateAngleWithVertical(float shoulder_x, float shoulder_y, float elbow_x, float elbow_y);
    float calculateAngleWithVerticalZY(float shoulder_z, float shoulder_y, float elbow_z, float elbow_y);
    float calculateShoulderTilt(float left_shoulder_x, float left_shoulder_y,
                               float right_shoulder_x, float right_shoulder_y);
    float calculateRelativeAngle(float shoulder_x, float shoulder_y,
                               float elbow_x, float elbow_y,
                               float wrist_x, float wrist_y);
    float calculateRelativeAngleZY(float shoulder_z, float shoulder_y,
                                 float elbow_z, float elbow_y,
                                 float wrist_z, float wrist_y);
    
    // NUEVOS: Métodos robustos con validación 3D
    bool validateArmGeometry(
        const kinematic_validator::Vector3& shoulder,
        const kinematic_validator::Vector3& elbow,
        const kinematic_validator::Vector3& wrist,
        const kinematic_validator::HumanLimbParams& params,
        float& upper_arm_length,
        float& forearm_length
    );
    
    kinematic_validator::SphericalAngles calculateRobustShoulderAngles(
        const kinematic_validator::Vector3& shoulder,
        const kinematic_validator::Vector3& elbow
    );
    
    kinematic_validator::SphericalAngles calculateRobustElbowAngles(
        const kinematic_validator::Vector3& shoulder,
        const kinematic_validator::Vector3& elbow,
        const kinematic_validator::Vector3& wrist
    );
    
    void bodyPointsCallback(const yaren_interfaces::msg::BodyPoints::SharedPtr msg);
    
    rclcpp::Subscription<yaren_interfaces::msg::BodyPoints>::SharedPtr subscription_;
    rclcpp::Publisher<yaren_interfaces::msg::BodyPosition>::SharedPtr publisher_;
};

#endif