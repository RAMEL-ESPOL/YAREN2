#include "yaren_controller.hpp"
#include <cmath>

// ── Mapea un valor de [in_min, in_max] a [out_min, out_max] ──────────────────
static float mapRange(float value, float in_min, float in_max,
                                   float out_min, float out_max) {
    float clamped = std::max(in_min, std::min(in_max, value));
    return out_min + (clamped - in_min) / (in_max - in_min) * (out_max - out_min);
}

DualArmTrajectoryController::DualArmTrajectoryController() : Node("body_trajectory_controller") {
    // ── Límites físicos del robot (rad) — TUS LÍMITES EXACTOS ──
    joint_limits_["joint_2"]  = std::make_pair(0.0, 0.5235f);
    joint_limits_["joint_5"]  = std::make_pair(0.0f, 3.0f);
    joint_limits_["joint_6"]  = std::make_pair(0.0f, 1.0472f);
    joint_limits_["joint_7"]  = std::make_pair(-0.7853f, 0.0f);
    joint_limits_["joint_8"]  = std::make_pair(0.1745f, 1.5708f);
    joint_limits_["joint_9"]  = std::make_pair(-3.0f, 0.0f);
    joint_limits_["joint_10"] = std::make_pair(0.0f, 1.0472f);
    joint_limits_["joint_11"] = std::make_pair(0.0f, 0.7853f);
    joint_limits_["joint_12"] = std::make_pair(0.1745f, 1.5708f);

    right_joints_ = {"joint_5", "joint_6", "joint_7", "joint_8"};
    left_joints_  = {"joint_9", "joint_10", "joint_11", "joint_12"};

    last_right_pos_    = calculateMidpoints(right_joints_);
    last_left_pos_     = calculateMidpoints(left_joints_);
    current_right_pos_ = last_right_pos_;
    current_left_pos_  = last_left_pos_;
    torso_tilt_        = 0.0f;

    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, "/joint_trajectory_controller/follow_joint_trajectory");

    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server no disponible");
        throw std::runtime_error("Action server not available");
    }

    subscription_ = this->create_subscription<yaren_interfaces::msg::BodyPosition>(
        "body_tracker", 10,
        std::bind(&DualArmTrajectoryController::armTrackerCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&DualArmTrajectoryController::sendTrajectoryGoal, this));

    all_joints_ = {
        "joint_1", "joint_2", "joint_3", "joint_4",
        "joint_5", "joint_6", "joint_7", "joint_8",
        "joint_9", "joint_10", "joint_11", "joint_12"
    };

    new_data_available_ = false;
    goal_sent_          = false;

    RCLCPP_INFO(this->get_logger(), "DualArmTrajectoryController inicializado");
}

std::map<std::string, float>
DualArmTrajectoryController::calculateMidpoints(const std::vector<std::string>& joints) {
    std::map<std::string, float> result;
    for (const auto& j : joints)
        result[j] = (joint_limits_[j].first + joint_limits_[j].second) / 2.0f;
    return result;
}

float DualArmTrajectoryController::euler2Radian(float euler) {
    return euler * static_cast<float>(M_PI) / 180.0f;
}

float DualArmTrajectoryController::limitJointPosition(const std::string& joint, float position) {
    auto it = joint_limits_.find(joint);
    if (it != joint_limits_.end())
        return std::max(it->second.first, std::min(it->second.second, position));
    return position;
}

// ─────────────────────────────────────────────────────────────────────────────
//  MAPEO CORREGIDO — Respeta TUS límites exactos
// ─────────────────────────────────────────────────────────────────────────────
std::map<std::string, float>
DualArmTrajectoryController::processArmData(const std::array<float, 4>& angles,
                                             const std::vector<std::string>& arm_joints,
                                             const bool is_right) {
    std::map<std::string, float> positions;

    // ── angles[0] = hombro ZY (elevación lateral) ──
    // Humano: 0° (abajo) .. +160° (arriba)
    if (is_right) {
        // joint_5: 0.0 a 3.0 rad (solo positivo)
        float j_shoulder_zy = mapRange(angles[0], 0.0f, 160.0f,
                                       joint_limits_[arm_joints[0]].first,   // 0.0
                                       joint_limits_[arm_joints[0]].second); // 3.0
        positions[arm_joints[0]] = limitJointPosition(arm_joints[0], j_shoulder_zy);
    } else {
        // joint_9: -3.0 a 0.0 rad (solo negativo, invertido)
        float j_shoulder_zy = mapRange(angles[0], 0.0f, 160.0f,
                                       joint_limits_[arm_joints[0]].second,   // -3.0
                                       joint_limits_[arm_joints[0]].first); // 0.0
        positions[arm_joints[0]] = limitJointPosition(arm_joints[0], j_shoulder_zy);
    }

    // ── angles[1] = hombro YX (flexión/extensión) ──
    // Humano: 0° (neutro) .. +90° (adelante)
    // Robot: 0.0 .. 1.0472 rad (solo positivo, usar valor absoluto)
    float j_shoulder_yx = mapRange(std::abs(angles[1]), 0.0f, 90.0f,
                                   joint_limits_[arm_joints[1]].first,   // 0.0
                                   joint_limits_[arm_joints[1]].second); // 1.0472
    positions[arm_joints[1]] = limitJointPosition(arm_joints[1], j_shoulder_yx);

    // ── angles[2] = codo ZY (flexión codo) ──
    // Humano: 0° (recto) .. +90° (doblado)
    if (is_right) {
        // joint_7: -0.7853 a 0.0 rad (solo negativo, mapeo invertido)
        // 0° humano → 0.0 rad, 90° humano → -0.7853 rad
        float j_elbow_zy = mapRange(angles[2], 0.0f, 90.0f,
                                    joint_limits_[arm_joints[2]].second, // 0.0
                                    joint_limits_[arm_joints[2]].first); // -0.7853
        positions[arm_joints[2]] = limitJointPosition(arm_joints[2], j_elbow_zy);
    } else {
        // joint_11: 0.0 a 0.7853 rad (solo positivo)
        float j_elbow_zy = mapRange(angles[2], 0.0f, 90.0f,
                                    joint_limits_[arm_joints[2]].first,   // 0.0
                                    joint_limits_[arm_joints[2]].second); // 0.7853
        positions[arm_joints[2]] = limitJointPosition(arm_joints[2], j_elbow_zy);
    }

    // ── angles[3] = codo YX (rotación muñeca) ──
    // Humano: 0° .. +150°
    // Robot: 0.1745 .. 1.5708 rad
    float j_elbow_yx = mapRange(std::abs(angles[3]), 0.0f, 150.0f,
                                joint_limits_[arm_joints[3]].first,   // 0.1745
                                joint_limits_[arm_joints[3]].second); // 1.5708
    positions[arm_joints[3]] = limitJointPosition(arm_joints[3], j_elbow_yx);

    return positions;
}

void DualArmTrajectoryController::armTrackerCallback(
    const yaren_interfaces::msg::BodyPosition::SharedPtr msg)
{
    if (!msg->is_valid) return;

    torso_tilt_ = 0.0f;

    // ── Brazo derecho ──
    // joint_5: 0 a 3.0 rad (positivo)
    // joint_7: -0.7853 a 0.0 rad (negativo)
    std::array<float, 4> right_angles = {
        msg->right_shoulder_elbow_zy,    // ZY: positivo para elevación
        msg->right_shoulder_elbow_yx,    // YX: positivo para adelante
        msg->right_elbow_wrist_zy,       // ZY: positivo para doblar
        msg->right_elbow_wrist_yx        // YX: positivo para rotar
    };
    last_right_pos_ = processArmData(right_angles, right_joints_, true);

    // ── Brazo izquierdo ──
    // joint_9: -3.0 a 0.0 rad (negativo)
    // joint_11: 0.0 a 0.7853 rad (positivo)
    std::array<float, 4> left_angles = {
        msg->left_shoulder_elbow_zy,     // ZY: positivo para elevación
        msg->left_shoulder_elbow_yx,     // YX: positivo para adelante
        msg->left_elbow_wrist_zy,        // ZY: positivo para doblar
        msg->left_elbow_wrist_yx         // YX: positivo para rotar
    };
    last_left_pos_ = processArmData(left_angles, left_joints_, false);

    new_data_available_ = true;
}

void DualArmTrajectoryController::goal_response_callback(
    const GoalHandleFollowJointTrajectory::SharedPtr& goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal rechazado");
        goal_sent_ = false;
    } else {
        goal_sent_ = true;
    }
}

void DualArmTrajectoryController::feedback_callback(
    GoalHandleFollowJointTrajectory::SharedPtr,
    const std::shared_ptr<const FollowJointTrajectory::Feedback>) {}

void DualArmTrajectoryController::result_callback(
    const GoalHandleFollowJointTrajectory::WrappedResult& result)
{
    goal_sent_ = false;
    if (result.code == rclcpp_action::ResultCode::ABORTED)
        RCLCPP_WARN(this->get_logger(), "Goal abortado");
}

void DualArmTrajectoryController::sendTrajectoryGoal() {
    if (!new_data_available_ || goal_sent_) return;

    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = all_joints_;

    const int   steps          = 10;
    const float total_duration = 0.4f;

    for (int i = 1; i <= steps; ++i) {
        float t      = static_cast<float>(i) / steps;
        float ease_t = t * t * (3.0f - 2.0f * t);

        trajectory_msgs::msg::JointTrajectoryPoint point;

        float j5  = current_right_pos_["joint_5"]  + (last_right_pos_["joint_5"]  - current_right_pos_["joint_5"])  * ease_t;
        float j6  = current_right_pos_["joint_6"]  + (last_right_pos_["joint_6"]  - current_right_pos_["joint_6"])  * ease_t;
        float j7  = current_right_pos_["joint_7"]  + (last_right_pos_["joint_7"]  - current_right_pos_["joint_7"])  * ease_t;
        float j8  = current_right_pos_["joint_8"]  + (last_right_pos_["joint_8"]  - current_right_pos_["joint_8"])  * ease_t;
        float j9  = current_left_pos_["joint_9"]   + (last_left_pos_["joint_9"]   - current_left_pos_["joint_9"])   * ease_t;
        float j10 = current_left_pos_["joint_10"]  + (last_left_pos_["joint_10"]  - current_left_pos_["joint_10"])  * ease_t;
        float j11 = current_left_pos_["joint_11"]  + (last_left_pos_["joint_11"]  - current_left_pos_["joint_11"])  * ease_t;
        float j12 = current_left_pos_["joint_12"]  + (last_left_pos_["joint_12"]  - current_left_pos_["joint_12"])  * ease_t;

        // ✅ SIN INVERSIÓN - Usar valores directamente (processArmData ya maneja la dirección)
        point.positions = {0.0f, torso_tilt_, 0.0f, 0.0f,
                           j5, j6, j7, j8, j9, j10, j11, j12};
        point.velocities.resize(point.positions.size(), 0.0);

        auto ns = static_cast<long long>(total_duration * t * 1e9);
        point.time_from_start = rclcpp::Duration(std::chrono::nanoseconds(ns));
        goal_msg.trajectory.points.push_back(point);
    }

    current_right_pos_ = last_right_pos_;
    current_left_pos_  = last_left_pos_;
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);

    auto opts = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    opts.goal_response_callback =
        std::bind(&DualArmTrajectoryController::goal_response_callback, this, std::placeholders::_1);
    opts.feedback_callback =
        std::bind(&DualArmTrajectoryController::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    opts.result_callback =
        std::bind(&DualArmTrajectoryController::result_callback, this, std::placeholders::_1);

    trajectory_client_->async_send_goal(goal_msg, opts);
    new_data_available_ = false;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<DualArmTrajectoryController>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Error fatal: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}