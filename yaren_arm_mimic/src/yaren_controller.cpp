#include "yaren_controller.hpp"
#include <cmath>
#include <algorithm>

DualArmTrajectoryController::DualArmTrajectoryController() : Node("body_trajectory_controller") {
    joint_limits_["joint_2"] = std::make_pair(-0.3, 0.3);  
    
    joint_limits_["joint_5"] = std::make_pair(-0.75, 1.50);
    joint_limits_["joint_6"] = std::make_pair(0.05, 0.9848);
    joint_limits_["joint_7"] = std::make_pair(-0.6981, 0.6981);
    joint_limits_["joint_8"] = std::make_pair(0.15, 1.50);
    
    joint_limits_["joint_9"] = std::make_pair(-0.75, 1.50);
    joint_limits_["joint_10"] = std::make_pair(0.05, 0.9848);
    joint_limits_["joint_11"] = std::make_pair(-0.6981, 0.6981);
    joint_limits_["joint_12"] = std::make_pair(0.15, 1.50);
    
    right_joints_ = {"joint_5", "joint_6", "joint_7", "joint_8"};
    left_joints_ = {"joint_9", "joint_10", "joint_11", "joint_12"};
    
    last_right_pos_ = calculateMidpoints(right_joints_);
    last_left_pos_ = calculateMidpoints(left_joints_);
    
    // Inicializar posiciones actuales para la interpolación fluida
    current_right_pos_ = last_right_pos_;
    current_left_pos_ = last_left_pos_;
    
    torso_tilt_ = 0.0;
    
    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, "/joint_trajectory_controller/follow_joint_trajectory");
        
    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting 5 seconds");
        throw std::runtime_error("Action server not available");
    }
    
    subscription_ = this->create_subscription<yaren_interfaces::msg::BodyPosition>(
        "body_tracker", 10, 
        std::bind(&DualArmTrajectoryController::armTrackerCallback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(300), 
        std::bind(&DualArmTrajectoryController::sendTrajectoryGoal, this));
    
    all_joints_ = {
        "joint_1", "joint_2", "joint_3", "joint_4",
        "joint_5", "joint_6", "joint_7", "joint_8",
        "joint_9", "joint_10", "joint_11", "joint_12"
    };
    
    new_data_available_ = false;
    goal_sent_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Dual arm trajectory controller initialized with improved kinematic limits");
}

std::map<std::string, float> DualArmTrajectoryController::calculateMidpoints(const std::vector<std::string>& joints) {
    std::map<std::string, float> result;
    for (const auto& j : joints) {
        result[j] = (joint_limits_[j].first + joint_limits_[j].second) / 2.0;
    }
    return result;
}

float DualArmTrajectoryController::euler2Radian(float euler) {
    return euler * M_PI / 180.0;
}

float DualArmTrajectoryController::limitJointPosition(const std::string& joint, float position) {
    if (joint_limits_.find(joint) != joint_limits_.end()) {
        float lower = joint_limits_[joint].first;
        float upper = joint_limits_[joint].second;
        return std::max(lower, std::min(upper, position));
    }
    return position;
}

/**
 * NUEVA FUNCIÓN: Validación interdependiente de límites articulares
 * 
 * Implementa restricciones cinemáticas adicionales:
 * - Cuando elevation es muy baja, azimuth debe ser limitado (configuración mecánica)
 * - Cuando elbow rotation es extremo, las restricciones de wrist son más severas
 * - Detecta configuraciones "imposibles" del robot
 */
std::map<std::string, float> DualArmTrajectoryController::validateInterdependentLimits(
    const std::map<std::string, float>& positions,
    const std::vector<std::string>& arm_joints
) {
    std::map<std::string, float> validated = positions;
    
    // Índices: [0]=azimuth(joint_5/9), [1]=elevation(joint_6/10), 
    //          [2]=elbow(joint_7/11), [3]=wrist(joint_8/12)
    
    float azimuth = validated[arm_joints[0]];
    float elevation = validated[arm_joints[1]];
    float elbow = validated[arm_joints[2]];
    float wrist = validated[arm_joints[3]];
    
    // RESTRICCIÓN 1: Cuando elevation es muy baja (<0.2 rad), 
    // azimuth debe estar en rango central para evitar colisiones
    if (elevation < 0.2f) {
        float azimuth_min = -0.3f;
        float azimuth_max = 0.3f;
        if (azimuth < azimuth_min || azimuth > azimuth_max) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Arm elevation too low (%.3f) - restricting azimuth to [%.3f, %.3f]",
                elevation, azimuth_min, azimuth_max);
            azimuth = std::max(azimuth_min, std::min(azimuth_max, azimuth));
            validated[arm_joints[0]] = azimuth;
        }
    }
    
    // RESTRICCIÓN 2: Validar que no hay "bloqueos" cinemáticos
    // El brazo no puede pasar a través del cuerpo
    // Si elevation es negativa (imposible), forzar a 0.05 (mínimo)
    if (elevation < 0.05f) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Shoulder elevation below minimum (%.3f) - clamping to minimum 0.05",
            elevation);
        elevation = 0.05f;
        validated[arm_joints[1]] = elevation;
    }
    
    // RESTRICCIÓN 3: Cuando elbow está en rango extremo (>0.5 rad),
    // aplicar restricciones adicionales en wrist para estabilidad
    if (std::abs(elbow) > 0.5f) {
        // En posiciones de codo extremas, wrist debe moverse con cuidado
        float wrist_min = 0.25f;
        float wrist_max = 1.2f;
        if (wrist < wrist_min || wrist > wrist_max) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Wrist angle conflicting with elbow extremes - correcting");
            wrist = std::max(wrist_min, std::min(wrist_max, wrist));
            validated[arm_joints[3]] = wrist;
        }
    }
    
    // RESTRICCIÓN 4: Detección de singularidad cercana
    // Si elevation está muy cerca del máximo, el robot entra en singularidad
    if (elevation > 0.9f) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Approaching shoulder singularity at elevation %.3f - limiting",
            elevation);
        elevation = 0.9f;
        validated[arm_joints[1]] = elevation;
    }
    
    return validated;
}

std::map<std::string, float> DualArmTrajectoryController::processArmData(const std::array<float, 4>& angles, 
                                          const std::vector<std::string>& arm_joints, 
                                          const bool is_right) {
    std::map<std::string, float> positions;

    // Convertir ángulos de grados a radianes
    positions[arm_joints[0]] = euler2Radian(angles[0]);
    positions[arm_joints[1]] = euler2Radian(angles[1]);
    positions[arm_joints[2]] = euler2Radian(angles[2]); 
    positions[arm_joints[3]] = euler2Radian(angles[3]);  
    
    // Aplicar límites básicos primero
    for (const auto& joint : arm_joints) {
        positions[joint] = limitJointPosition(joint, positions[joint]);
    }
    
    // NUEVO: Aplicar validación interdependiente
    positions = validateInterdependentLimits(positions, arm_joints);
    
    // Verificación final de límites (por si la validación interdependiente generó valores fuera de rango)
    for (const auto& joint : arm_joints) {
        positions[joint] = limitJointPosition(joint, positions[joint]);
    }
    
    return positions;
}

void DualArmTrajectoryController::armTrackerCallback(const yaren_interfaces::msg::BodyPosition::SharedPtr msg) {
    if (!msg->is_valid) {
        RCLCPP_DEBUG(this->get_logger(), "Invalid data. Maintaining position.");
        return;
    }

    // Se eliminó la lógica de shoulder_tilt_angle ya que no existe en el mensaje
    torso_tilt_ = 0.0;
            
    std::array<float, 4> right_angles = {
        msg->right_shoulder_elbow_zy,
        msg->right_shoulder_elbow_yx,
        msg->right_elbow_wrist_zy,
        msg->right_elbow_wrist_yx
    };
    
    last_right_pos_ = processArmData(right_angles, right_joints_, true);
    
    std::array<float, 4> left_angles = {
        msg->left_shoulder_elbow_zy,
        msg->left_shoulder_elbow_yx,
        msg->left_elbow_wrist_zy,
        msg->left_elbow_wrist_yx
    };
    
    last_left_pos_ = processArmData(left_angles, left_joints_, false);
    
    new_data_available_ = true;
}

void DualArmTrajectoryController::goal_response_callback(const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Goal accepted by server, waiting for result");
        goal_sent_ = true;
    }
}

void DualArmTrajectoryController::feedback_callback(
    GoalHandleFollowJointTrajectory::SharedPtr,
    const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
    // Mantener la terminal limpia de exceso de logs
}

void DualArmTrajectoryController::result_callback(const GoalHandleFollowJointTrajectory::WrappedResult & result) {
    goal_sent_ = false;
    
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Unknown result code");
            break;
    }
}

void DualArmTrajectoryController::sendTrajectoryGoal() {
    if (!new_data_available_ || goal_sent_) {
        return;
    }
    
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = all_joints_;
    
    int steps = 15; // Matriz de 15 pasos para interpolación fluida
    float total_duration = 1.0; // Duración total del movimiento en segundos
    
    for (int i = 1; i <= steps; ++i) {
        float t = static_cast<float>(i) / steps;
        
        // Curva Ease-in / Ease-out (suavizado cúbico)
        float ease_t = t * t * (3.0f - 2.0f * t); 
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        
        // Interpolación brazo derecho
        float j5 = current_right_pos_["joint_5"] + (last_right_pos_["joint_5"] - current_right_pos_["joint_5"]) * ease_t;
        float j6 = current_right_pos_["joint_6"] + (last_right_pos_["joint_6"] - current_right_pos_["joint_6"]) * ease_t;
        float j7 = current_right_pos_["joint_7"] + (last_right_pos_["joint_7"] - current_right_pos_["joint_7"]) * ease_t;
        float j8 = current_right_pos_["joint_8"] + (last_right_pos_["joint_8"] - current_right_pos_["joint_8"]) * ease_t;
        
        // Interpolación brazo izquierdo
        float j9  = current_left_pos_["joint_9"]  + (last_left_pos_["joint_9"]  - current_left_pos_["joint_9"])  * ease_t;
        float j10 = current_left_pos_["joint_10"] + (last_left_pos_["joint_10"] - current_left_pos_["joint_10"]) * ease_t;
        float j11 = current_left_pos_["joint_11"] + (last_left_pos_["joint_11"] - current_left_pos_["joint_11"]) * ease_t;
        float j12 = current_left_pos_["joint_12"] + (last_left_pos_["joint_12"] - current_left_pos_["joint_12"]) * ease_t;
        
        point.positions = {
            0.0,                   
            torso_tilt_,           
            0.0, 0.0,              
            j5, j6, j7, j8,               
            j9, j10, j11, j12                
        };
        
        point.velocities.resize(point.positions.size(), 0.0);
        
        long long nanoseconds = static_cast<long long>(total_duration * t * 1e9);
        point.time_from_start = rclcpp::Duration(std::chrono::nanoseconds(nanoseconds));
        
        goal_msg.trajectory.points.push_back(point);
    }
    
    // Guardar la última meta enviada como el nuevo estado "actual"
    current_right_pos_ = last_right_pos_;
    current_left_pos_ = last_left_pos_;
    
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
    
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = 
        std::bind(&DualArmTrajectoryController::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = 
        std::bind(&DualArmTrajectoryController::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = 
        std::bind(&DualArmTrajectoryController::result_callback, this, std::placeholders::_1);
        
    trajectory_client_->async_send_goal(goal_msg, send_goal_options);
    
    new_data_available_ = false;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualArmTrajectoryController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}