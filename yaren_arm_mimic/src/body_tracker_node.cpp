#include "body_tracker_node.hpp"
#include <cmath>

BodyTrackerNode::BodyTrackerNode() : Node("body_tracker_node"), last_detection_valid(false) {
    subscription_ = this->create_subscription<yaren_interfaces::msg::BodyPoints>(
        "body_points", 10, 
        std::bind(&BodyTrackerNode::bodyPointsCallback, this, std::placeholders::_1));
    
    publisher_ = this->create_publisher<yaren_interfaces::msg::BodyPosition>("body_tracker", 10);
    
    // Inicializar estados previos
    last_right_shoulder_angles_ = {0, 0, 0};
    last_right_elbow_angles_ = {0, 0, 0};
    last_left_shoulder_angles_ = {0, 0, 0};
    last_left_elbow_angles_ = {0, 0, 0};
    
    RCLCPP_INFO(this->get_logger(), "Body tracker node initialized with 3D kinematic validation");
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

float BodyTrackerNode::calculateShoulderTilt(float left_shoulder_x, float left_shoulder_y,
                                             float right_shoulder_x, float right_shoulder_y) {
    float v_x = right_shoulder_x - left_shoulder_x;
    float v_y = right_shoulder_y - left_shoulder_y;
    
    return radian2Euler(atan2(v_x, v_y));
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

/**
 * NUEVOS MÉTODOS: Validación 3D robusto con corrección de perspectiva
 */

bool BodyTrackerNode::validateArmGeometry(
    const kinematic_validator::Vector3& shoulder,
    const kinematic_validator::Vector3& elbow,
    const kinematic_validator::Vector3& wrist,
    const kinematic_validator::HumanLimbParams& params,
    float& upper_arm_length,
    float& forearm_length
) {
    // Detectar singularidades (codo demasiado cerca del hombro)
    if (kinematic_validator::detectSingularity(elbow, shoulder, 0.02f)) {
        RCLCPP_WARN(this->get_logger(), "Singularity detected: elbow too close to shoulder");
        return false;
    }
    
    // Calcular longitudes de miembros
    upper_arm_length = (elbow - shoulder).magnitude();
    forearm_length = (wrist - elbow).magnitude();
    
    // Validar longitudes
    auto upper_arm_check = kinematic_validator::validateLimbLength(
        upper_arm_length,
        params.upper_arm_min,
        params.upper_arm_max
    );
    
    auto forearm_check = kinematic_validator::validateLimbLength(
        forearm_length,
        params.forearm_min,
        params.forearm_max
    );
    
    if (!upper_arm_check.is_valid) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Upper arm validation failed: %s (length: %.3f)",
            upper_arm_check.error_message.c_str(), upper_arm_length);
        return false;
    }
    
    if (!forearm_check.is_valid) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Forearm validation failed: %s (length: %.3f)",
            forearm_check.error_message.c_str(), forearm_length);
        return false;
    }
    
    return true;
}

kinematic_validator::SphericalAngles BodyTrackerNode::calculateRobustShoulderAngles(
    const kinematic_validator::Vector3& shoulder,
    const kinematic_validator::Vector3& elbow
) {
    // Vector desde hombro a codo (relativo)
    kinematic_validator::Vector3 shoulder_vec = elbow - shoulder;
    
    // Calcular ángulos esféricos (corrección de perspectiva)
    kinematic_validator::SphericalAngles angles = 
        kinematic_validator::calculateSphericalAngles(shoulder_vec);
    
    return angles;
}

kinematic_validator::SphericalAngles BodyTrackerNode::calculateRobustElbowAngles(
    const kinematic_validator::Vector3& shoulder,
    const kinematic_validator::Vector3& elbow,
    const kinematic_validator::Vector3& wrist
) {
    // Vector desde hombro a codo (referencia)
    kinematic_validator::Vector3 upper_arm = elbow - shoulder;
    
    // Vector desde codo a muñeca
    kinematic_validator::Vector3 forearm = wrist - elbow;
    
    // Ángulo relativo entre forearm y upper arm
    // Se calcula usando el plano definido por ambos vectores
    float upper_mag = upper_arm.magnitude();
    float fore_mag = forearm.magnitude();
    
    if (upper_mag < 1e-6f || fore_mag < 1e-6f) {
        return kinematic_validator::SphericalAngles{0, 0, 0};
    }
    
    kinematic_validator::Vector3 upper_normalized = upper_arm.normalized();
    kinematic_validator::Vector3 fore_normalized = forearm.normalized();
    
    // Ángulo entre vectores (flexión de codo)
    float cos_angle = upper_normalized.dot(fore_normalized);
    cos_angle = std::max(-1.0f, std::min(1.0f, cos_angle));  // Clamp a [-1, 1]
    float elbow_flex = std::acos(cos_angle);
    
    // Convertir a ángulos esféricos relativos
    kinematic_validator::SphericalAngles angles = 
        kinematic_validator::calculateSphericalAngles(forearm);
    
    // El ángulo de flexión va en el pitch relativo
    angles.pitch = elbow_flex - M_PI;  // Restar π porque 0 es extendido, π es flexionado
    
    return angles;
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
    
    // Convertir Point32 a Vector3
    auto right_shoulder = kinematic_validator::point32ToVector3(
        msg->right_shoulder.x, msg->right_shoulder.y, msg->right_shoulder.z);
    auto right_elbow = kinematic_validator::point32ToVector3(
        msg->right_elbow.x, msg->right_elbow.y, msg->right_elbow.z);
    auto right_wrist = kinematic_validator::point32ToVector3(
        msg->right_wrist.x, msg->right_wrist.y, msg->right_wrist.z);
    
    auto left_shoulder = kinematic_validator::point32ToVector3(
        msg->left_shoulder.x, msg->left_shoulder.y, msg->left_shoulder.z);
    auto left_elbow = kinematic_validator::point32ToVector3(
        msg->left_elbow.x, msg->left_elbow.y, msg->left_elbow.z);
    auto left_wrist = kinematic_validator::point32ToVector3(
        msg->left_wrist.x, msg->left_wrist.y, msg->left_wrist.z);
    
    // RAMA DERECHA: Validación 3D
    float right_upper_len, right_fore_len;
    bool right_geometry_valid = validateArmGeometry(
        right_shoulder, right_elbow, right_wrist,
        human_params_,
        right_upper_len, right_fore_len
    );
    
    // RAMA IZQUIERDA: Validación 3D
    float left_upper_len, left_fore_len;
    bool left_geometry_valid = validateArmGeometry(
        left_shoulder, left_elbow, left_wrist,
        human_params_,
        left_upper_len, left_fore_len
    );
    
    // Si al menos un brazo es válido, proceder
    if (!right_geometry_valid && !left_geometry_valid) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Both arms failed geometry validation");
        return;
    }
    
    // Calcular ángulos ROBUSTOS con validación 3D
    kinematic_validator::SphericalAngles right_shoulder_angles{0, 0, 0};
    kinematic_validator::SphericalAngles right_elbow_angles{0, 0, 0};
    
    if (right_geometry_valid) {
        right_shoulder_angles = calculateRobustShoulderAngles(right_shoulder, right_elbow);
        right_elbow_angles = calculateRobustElbowAngles(right_shoulder, right_elbow, right_wrist);
        
        // Aplicar suavizado exponencial
        right_shoulder_angles = kinematic_validator::smoothSphericalAngles(
            right_shoulder_angles, last_right_shoulder_angles_, 0.2f);
        right_elbow_angles = kinematic_validator::smoothSphericalAngles(
            right_elbow_angles, last_right_elbow_angles_, 0.2f);
        
        last_right_shoulder_angles_ = right_shoulder_angles;
        last_right_elbow_angles_ = right_elbow_angles;
    }
    
    kinematic_validator::SphericalAngles left_shoulder_angles{0, 0, 0};
    kinematic_validator::SphericalAngles left_elbow_angles{0, 0, 0};
    
    if (left_geometry_valid) {
        left_shoulder_angles = calculateRobustShoulderAngles(left_shoulder, left_elbow);
        left_elbow_angles = calculateRobustElbowAngles(left_shoulder, left_elbow, left_wrist);
        
        // Aplicar suavizado exponencial
        left_shoulder_angles = kinematic_validator::smoothSphericalAngles(
            left_shoulder_angles, last_left_shoulder_angles_, 0.2f);
        left_elbow_angles = kinematic_validator::smoothSphericalAngles(
            left_elbow_angles, last_left_elbow_angles_, 0.2f);
        
        last_left_shoulder_angles_ = left_shoulder_angles;
        last_left_elbow_angles_ = left_elbow_angles;
    }
    
    // Convertir a grados para compatibilidad con sistema actual
    auto right_shoulder_deg = kinematic_validator::radiansToDegreesSpherical(right_shoulder_angles);
    auto right_elbow_deg = kinematic_validator::radiansToDegreesSpherical(right_elbow_angles);
    auto left_shoulder_deg = kinematic_validator::radiansToDegreesSpherical(left_shoulder_angles);
    auto left_elbow_deg = kinematic_validator::radiansToDegreesSpherical(left_elbow_angles);
    
    // Mapear ángulos esféricos a articulaciones del robot
    // Para compatibilidad con el formato existente:
    // right_shoulder_elbow_zy <- pitch del hombro
    // right_shoulder_elbow_yx <- yaw del hombro
    // right_elbow_wrist_zy <- pitch del codo
    // right_elbow_wrist_yx <- yaw del codo
    
    arm_msg.right_shoulder_elbow_zy = right_shoulder_deg.pitch;
    arm_msg.right_shoulder_elbow_yx = right_shoulder_deg.yaw;
    arm_msg.right_elbow_wrist_zy = right_elbow_deg.pitch;
    arm_msg.right_elbow_wrist_yx = right_elbow_deg.yaw;
    
    arm_msg.left_shoulder_elbow_zy = left_shoulder_deg.pitch;
    arm_msg.left_shoulder_elbow_yx = -left_shoulder_deg.yaw;  // Invertir para brazo izquierdo
    arm_msg.left_elbow_wrist_zy = left_elbow_deg.pitch;
    arm_msg.left_elbow_wrist_yx = left_elbow_deg.yaw;
    
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
    
    // Log de depuración (una vez por segundo)
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Right shoulder: pitch=%.2f° yaw=%.2f° | Left shoulder: pitch=%.2f° yaw=%.2f°",
        right_shoulder_deg.pitch, right_shoulder_deg.yaw,
        left_shoulder_deg.pitch, left_shoulder_deg.yaw);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BodyTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}