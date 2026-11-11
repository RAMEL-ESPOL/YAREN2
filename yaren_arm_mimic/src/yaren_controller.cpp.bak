#include "yaren_controller.hpp"
#include <cmath>

static float mapRange(float value, float in_min, float in_max,
                      float out_min, float out_max) {
    float clamped = std::max(in_min, std::min(in_max, value));
    return out_min + (clamped - in_min) / (in_max - in_min) * (out_max - out_min);
}

DualArmTrajectoryController::DualArmTrajectoryController()
: rclcpp_lifecycle::LifecycleNode("body_trajectory_controller") {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualArmTrajectoryController::on_configure(const rclcpp_lifecycle::State&)
{
    joint_limits_["joint_2"]  = {0.0f,    0.5235f};
    joint_limits_["joint_5"]  = {0.0f,    3.0f};
    joint_limits_["joint_6"]  = {0.0f,    1.0472f};
    joint_limits_["joint_7"]  = {-0.7853f,0.0f};
    joint_limits_["joint_8"]  = {0.1745f, 1.5708f};
    joint_limits_["joint_9"]  = {-3.0f,   0.0f};
    joint_limits_["joint_10"] = {0.0f,    1.0472f};
    joint_limits_["joint_11"] = {0.0f,    0.7853f};
    joint_limits_["joint_12"] = {0.1745f, 1.5708f};

    right_joints_ = {"joint_5","joint_6","joint_7","joint_8"};
    left_joints_  = {"joint_9","joint_10","joint_11","joint_12"};
    all_joints_   = {"joint_1","joint_2","joint_3","joint_4",
                     "joint_5","joint_6","joint_7","joint_8",
                     "joint_9","joint_10","joint_11","joint_12"};

    last_right_pos_    = calculateMidpoints(right_joints_);
    last_left_pos_     = calculateMidpoints(left_joints_);
    current_right_pos_ = last_right_pos_;
    current_left_pos_  = last_left_pos_;

    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, "/joint_trajectory_controller/follow_joint_trajectory");

    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(get_logger(), "Action server no disponible");
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "yaren_controller configurado.");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualArmTrajectoryController::on_activate(const rclcpp_lifecycle::State& state)
{
    LifecycleNode::on_activate(state);

    subscription_ = this->create_subscription<yaren_interfaces::msg::BodyPosition>(
        "body_tracker", 10,
        std::bind(&DualArmTrajectoryController::armTrackerCallback,
                  this, std::placeholders::_1));

    mimic_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/mimic/enabled", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            mimic_enabled_ = msg->data;
            RCLCPP_INFO(get_logger(), "Mimic %s", mimic_enabled_ ? "ACTIVADO" : "DESACTIVADO");
        });

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&DualArmTrajectoryController::sendTrajectoryGoal, this));

    RCLCPP_INFO(get_logger(), "yaren_controller activo.");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualArmTrajectoryController::on_deactivate(const rclcpp_lifecycle::State& state)
{
    LifecycleNode::on_deactivate(state);
    timer_.reset();
    subscription_.reset();
    mimic_enable_sub_.reset();
    mimic_enabled_ = false;
    RCLCPP_INFO(get_logger(), "yaren_controller desactivado.");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualArmTrajectoryController::on_cleanup(const rclcpp_lifecycle::State&)
{
    trajectory_client_.reset();
    joint_limits_.clear();
    RCLCPP_INFO(get_logger(), "yaren_controller limpiado.");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DualArmTrajectoryController::on_shutdown(const rclcpp_lifecycle::State&)
{
    timer_.reset();
    subscription_.reset();
    mimic_enable_sub_.reset();
    trajectory_client_.reset();
    return CallbackReturn::SUCCESS;
}

// ── Métodos de cálculo (sin cambios) ─────────────────────────────────────────

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

float DualArmTrajectoryController::limitJointPosition(
    const std::string& joint, float position) {
    auto it = joint_limits_.find(joint);
    if (it != joint_limits_.end())
        return std::max(it->second.first, std::min(it->second.second, position));
    return position;
}

std::map<std::string, float>
DualArmTrajectoryController::processArmData(const std::array<float,4>& angles,
                                             const std::vector<std::string>& arm_joints,
                                             bool is_right) {
    std::map<std::string, float> positions;
    if (is_right) {
        positions[arm_joints[0]] = limitJointPosition(arm_joints[0],
            mapRange(angles[0], 0.0f, 160.0f,
                     joint_limits_[arm_joints[0]].first,
                     joint_limits_[arm_joints[0]].second));
    } else {
        positions[arm_joints[0]] = limitJointPosition(arm_joints[0],
            mapRange(angles[0], 0.0f, 160.0f,
                     joint_limits_[arm_joints[0]].second,
                     joint_limits_[arm_joints[0]].first));
    }
    positions[arm_joints[1]] = limitJointPosition(arm_joints[1],
        mapRange(std::abs(angles[1]), 0.0f, 90.0f,
                 joint_limits_[arm_joints[1]].first,
                 joint_limits_[arm_joints[1]].second));
    if (is_right) {
        positions[arm_joints[2]] = limitJointPosition(arm_joints[2],
            mapRange(angles[2], 0.0f, 90.0f,
                     joint_limits_[arm_joints[2]].second,
                     joint_limits_[arm_joints[2]].first));
    } else {
        positions[arm_joints[2]] = limitJointPosition(arm_joints[2],
            mapRange(angles[2], 0.0f, 90.0f,
                     joint_limits_[arm_joints[2]].first,
                     joint_limits_[arm_joints[2]].second));
    }
    positions[arm_joints[3]] = limitJointPosition(arm_joints[3],
        mapRange(std::abs(angles[3]), 0.0f, 150.0f,
                 joint_limits_[arm_joints[3]].first,
                 joint_limits_[arm_joints[3]].second));
    return positions;
}

void DualArmTrajectoryController::armTrackerCallback(
    const yaren_interfaces::msg::BodyPosition::SharedPtr msg)
{
    if (!msg->is_valid || !mimic_enabled_) return;
    torso_tilt_ = 0.0f;
    last_right_pos_ = processArmData(
        {msg->right_shoulder_elbow_zy, msg->right_shoulder_elbow_yx,
         msg->right_elbow_wrist_zy,    msg->right_elbow_wrist_yx},
        right_joints_, true);
    last_left_pos_ = processArmData(
        {msg->left_shoulder_elbow_zy, msg->left_shoulder_elbow_yx,
         msg->left_elbow_wrist_zy,    msg->left_elbow_wrist_yx},
        left_joints_, false);
    new_data_available_ = true;
}

void DualArmTrajectoryController::goal_response_callback(
    const GoalHandleFollowJointTrajectory::SharedPtr& goal_handle) {
    goal_sent_ = (goal_handle != nullptr);
    if (!goal_handle)
        RCLCPP_ERROR(get_logger(), "Goal rechazado");
}

void DualArmTrajectoryController::feedback_callback(
    GoalHandleFollowJointTrajectory::SharedPtr,
    const std::shared_ptr<const FollowJointTrajectory::Feedback>) {}

void DualArmTrajectoryController::result_callback(
    const GoalHandleFollowJointTrajectory::WrappedResult& result) {
    goal_sent_ = false;
    if (result.code == rclcpp_action::ResultCode::ABORTED)
        RCLCPP_WARN(get_logger(), "Goal abortado");
}

void DualArmTrajectoryController::sendTrajectoryGoal() {
    if (!new_data_available_ || goal_sent_) return;
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = all_joints_;
    const int steps = 10;
    const float total_duration = 0.4f;
    for (int i = 1; i <= steps; ++i) {
        float t      = static_cast<float>(i) / steps;
        float ease_t = t * t * (3.0f - 2.0f * t);
        trajectory_msgs::msg::JointTrajectoryPoint point;
        auto interp = [&](const std::map<std::string,float>& cur,
                          const std::map<std::string,float>& tgt,
                          const std::string& j) {
            return cur.at(j) + (tgt.at(j) - cur.at(j)) * ease_t;
        };
        point.positions = {
            0.0f, torso_tilt_, 0.0f, 0.0f,
            interp(current_right_pos_, last_right_pos_, "joint_5"),
            interp(current_right_pos_, last_right_pos_, "joint_6"),
            interp(current_right_pos_, last_right_pos_, "joint_7"),
            interp(current_right_pos_, last_right_pos_, "joint_8"),
            interp(current_left_pos_,  last_left_pos_,  "joint_9"),
            interp(current_left_pos_,  last_left_pos_,  "joint_10"),
            interp(current_left_pos_,  last_left_pos_,  "joint_11"),
            interp(current_left_pos_,  last_left_pos_,  "joint_12"),
        };
        point.velocities.resize(point.positions.size(), 0.0);
        point.time_from_start = rclcpp::Duration(
            std::chrono::nanoseconds(static_cast<long long>(total_duration * t * 1e9)));
        goal_msg.trajectory.points.push_back(point);
    }
    current_right_pos_ = last_right_pos_;
    current_left_pos_  = last_left_pos_;
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
    auto opts = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    opts.goal_response_callback =
        std::bind(&DualArmTrajectoryController::goal_response_callback,
                  this, std::placeholders::_1);
    opts.feedback_callback =
        std::bind(&DualArmTrajectoryController::feedback_callback,
                  this, std::placeholders::_1, std::placeholders::_2);
    opts.result_callback =
        std::bind(&DualArmTrajectoryController::result_callback,
                  this, std::placeholders::_1);
    trajectory_client_->async_send_goal(goal_msg, opts);
    new_data_available_ = false;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<DualArmTrajectoryController>();
        rclcpp::spin(node->get_node_base_interface());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Error fatal: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}