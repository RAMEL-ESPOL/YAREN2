#ifndef YAREN_CONTROLLER_HPP
#define YAREN_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "yaren_interfaces/msg/body_position.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cmath>
#include <map>
#include <string>
#include <vector>

class DualArmTrajectoryController : public rclcpp_lifecycle::LifecycleNode {
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory =
        rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    DualArmTrajectoryController();

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
    std::map<std::string, float> calculateMidpoints(const std::vector<std::string>&);
    float euler2Radian(float euler);
    float limitJointPosition(const std::string& joint, float position);
    std::map<std::string, float> processArmData(const std::array<float,4>&,
                                                 const std::vector<std::string>&,
                                                 bool is_right);
    void armTrackerCallback(const yaren_interfaces::msg::BodyPosition::SharedPtr);
    void goal_response_callback(const GoalHandleFollowJointTrajectory::SharedPtr&);
    void feedback_callback(GoalHandleFollowJointTrajectory::SharedPtr,
                           const std::shared_ptr<const FollowJointTrajectory::Feedback>);
    void result_callback(const GoalHandleFollowJointTrajectory::WrappedResult&);
    void sendTrajectoryGoal();

    std::map<std::string, std::pair<float,float>> joint_limits_;
    std::map<std::string, float> last_right_pos_, last_left_pos_;
    std::map<std::string, float> current_right_pos_, current_left_pos_;
    std::vector<std::string> right_joints_, left_joints_, all_joints_;
    float torso_tilt_{0.0f};
    bool new_data_available_{false};
    bool goal_sent_{false};
    bool mimic_enabled_{false};

    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
    rclcpp::Subscription<yaren_interfaces::msg::BodyPosition>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mimic_enable_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif