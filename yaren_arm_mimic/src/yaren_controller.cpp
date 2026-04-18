#include "yaren_controller.hpp"

DualArmTrajectoryController::DualArmTrajectoryController() : Node("body_trajectory_controller") {
    joint_limits_["joint_5"] = std::pair<float, float>(-6.2, 6.2);
    joint_limits_["joint_6"] = std::pair<float, float>(-6.2, 6.2);
    joint_limits_["joint_7"] = std::pair<float, float>(-6.2, 6.2);
    joint_limits_["joint_8"] = std::pair<float, float>(-6.2, 6.2);

    joint_limits_["joint_9"] = std::pair<float, float>(-6.2, 6.2);
    joint_limits_["joint_10"] = std::pair<float, float>(-6.2, 6.2);
    joint_limits_["joint_11"] = std::pair<float, float>(-6.2, 6.2);
    joint_limits_["joint_12"] = std::pair<float, float>(-6.2, 6.2);

    right_joints_ = {"joint_5", "joint_6", "joint_7", "joint_8"};
    left_joints_ = {"joint_9", "joint_10", "joint_11", "joint_12"};
    
    last_right_pos_ = calculateMidpoints(right_joints_);
    last_left_pos_ = calculateMidpoints(left_joints_);
    
    torso_tilt_ = 0.0;
    
    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, "/joint_trajectory_controller/follow_joint_trajectory");
    
    subscription_ = this->create_subscription<yaren_interfaces::msg::BodyPosition>(
        "body_tracker", 10, 
        std::bind(&DualArmTrajectoryController::armTrackerCallback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), 
        std::bind(&DualArmTrajectoryController::sendTrajectoryGoal, this));    

    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_WARN(this->get_logger(), "Action server not available after waiting 10 seconds");
        //throw std::runtime_error("Action server not available");
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Action server available.");
    }
    /*
    subscription_ = this->create_subscription<yaren_interfaces::msg::BodyPosition>(
        "body_tracker", 10, 
        std::bind(&DualArmTrajectoryController::armTrackerCallback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), 
        std::bind(&DualArmTrajectoryController::sendTrajectoryGoal, this));
    */
    all_joints_ = {
        "joint_1", "joint_2", "joint_3", "joint_4",
        "joint_5", "joint_6", "joint_7","joint_8",
        "joint_9", "joint_10", "joint_11", "joint_12"
    };
    
    new_data_available_ = false;
    bool new_data_available_;    
    RCLCPP_INFO(this->get_logger(), "Dual arm trajectory controller initialized");
}

std::map<std::string, float> DualArmTrajectoryController::calculateMidpoints(const std::vector<std::string>& joints) {
    std::map<std::string, float> result;
    for (const auto& j : joints) {
        result[j] = (joint_limits_[j].first + joint_limits_[j].second) / 2.0;
    }
    return result;
}


float DualArmTrajectoryController::euler2Radian(float euler) {
    return euler;  // Si body_points_detector ya publica en radianes
}


float DualArmTrajectoryController::limitJointPosition(const std::string& joint, float position) {
    if (joint_limits_.find(joint) != joint_limits_.end()) {
        float lower = joint_limits_[joint].first;
        float upper = joint_limits_[joint].second;
        return std::max(lower, std::min(upper, position));
    }
    return position;
}

std::map<std::string, float> DualArmTrajectoryController::processArmData(
    const std::array<float, 4>& angles, 
    const std::vector<std::string>& arm_joints, 
    const bool is_right) 
{
    std::map<std::string, float> positions;

    if (is_right) {
        // joint_4: hombro rotación ZY
        positions[arm_joints[0]] = euler2Radian(angles[0]);
        // joint_5: hombro rotación YX (rpy=π,0,π en URDF)
        positions[arm_joints[1]] = euler2Radian(angles[1]);
        // joint_6: codo rotación YX
        positions[arm_joints[2]] = euler2Radian(angles[2]);
        // joint_7: codo rotación ZY (rpy=+1.5708 en URDF)
        positions[arm_joints[3]] = euler2Radian(angles[3]);
    } else {
        // joint_9: rpy=0,0,0 vs joint_5: rpy=π,0,π → mismo eje pero frame distinto
        positions[arm_joints[0]] = euler2Radian(angles[0]);
        // joint_10: rpy=0,0,-π/2 igual que joint_6 → invertir por simetría especular
        positions[arm_joints[1]] = euler2Radian(-angles[1]);
        // joint_11: rpy=-1.5708 vs joint_7: rpy=+1.5708 → invertir signo
        positions[arm_joints[2]] = euler2Radian(-angles[2]);
        // joint_12: similar a joint_8
        positions[arm_joints[3]] = euler2Radian(angles[3]);
    }

    for (const auto& joint : arm_joints) {
        positions[joint] = limitJointPosition(joint, positions[joint]);
    }
    return positions;
}
void DualArmTrajectoryController::armTrackerCallback(const yaren_interfaces::msg::BodyPosition::SharedPtr msg) {
    if (!msg->is_valid) {
        RCLCPP_INFO(this->get_logger(), "Datos inválidos. Manteniendo posición.");
        return;
    }
            
    std::array<float, 4> right_angles = {
        msg->right_shoulder_elbow_zy,
        msg->right_shoulder_elbow_yx,
        msg->right_elbow_wrist_yx,
        msg->right_elbow_wrist_zy
    };
    
    last_right_pos_ = processArmData(right_angles, right_joints_, true);
    
    std::array<float, 4> left_angles = {
        msg->left_shoulder_elbow_zy,
        msg->left_shoulder_elbow_yx,
        msg->left_elbow_wrist_yx,
        msg->left_elbow_wrist_zy
    };
    
    last_left_pos_ = processArmData(left_angles, left_joints_, false);
    
    new_data_available_ = true;
}

void DualArmTrajectoryController::goal_response_callback(
    const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle) 
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
        // ELIMINAR: goal_sent_ = true;  ← ya no se usa
    }
}

void DualArmTrajectoryController::feedback_callback(
    GoalHandleFollowJointTrajectory::SharedPtr,
    const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Received feedback");
}

void DualArmTrajectoryController::result_callback(
    const GoalHandleFollowJointTrajectory::WrappedResult & result) 
{
    // ELIMINAR: goal_sent_ = false;  ← ya no se usa
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "Unknown result code");
            break;
    }
}
void DualArmTrajectoryController::sendTrajectoryGoal() {
    if (!new_data_available_) {
        return;
    }
    // ELIMINAR el bloqueo por goal_sent_
    // ANTES: if (!new_data_available_ || goal_sent_) return;
    // Ahora simplemente continúa y sobreescribe el goal anterior

    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = all_joints_;
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {
        0.0,
        0.0,
        0.0,
        0.0,
        last_right_pos_["joint_5"],
        last_right_pos_["joint_6"],
        last_right_pos_["joint_7"],
        last_right_pos_["joint_8"],
        last_left_pos_["joint_9"],
        last_left_pos_["joint_10"],
        last_left_pos_["joint_11"],
        last_left_pos_["joint_12"],
    };
    
    point.velocities.resize(point.positions.size(), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(0.15); // ← CORREGIDO

    goal_msg.trajectory.points.push_back(point);
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(0.5); // ← dar tolerancia

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
