#ifndef YAREN_CONTROLLER_HPP
#define YAREN_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "yaren_interfaces/msg/body_position.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <cmath>
#include <map>
#include <string>
#include <vector>

class DualArmTrajectoryController : public rclcpp::Node {
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    DualArmTrajectoryController();

private:
    std::map<std::string, float> calculateMidpoints(const std::vector<std::string>& joints);
    float euler2Radian(float euler);
    float limitJointPosition(const std::string& joint, float position);
    std::map<std::string, float> processArmData(const std::array<float, 4>& angles, 
                                              const std::vector<std::string>& arm_joints, 
                                              const bool is_right);
    void armTrackerCallback(const yaren_interfaces::msg::BodyPosition::SharedPtr msg);
    void goal_response_callback(const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle);
    void feedback_callback(GoalHandleFollowJointTrajectory::SharedPtr,
                          const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback);
    void result_callback(const GoalHandleFollowJointTrajectory::WrappedResult & result);
    void sendTrajectoryGoal();
    
    std::map<std::string, std::pair<float, float>> joint_limits_;
    std::map<std::string, float> last_right_pos_;
    std::map<std::string, float> last_left_pos_;
    float torso_tilt_;
    std::map<std::string, float> current_right_pos_;
    std::map<std::string, float> current_left_pos_;
    std::vector<std::string> right_joints_;
    std::vector<std::string> left_joints_;
    std::vector<std::string> all_joints_;
    
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
    rclcpp::Subscription<yaren_interfaces::msg::BodyPosition>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool new_data_available_;
    bool goal_sent_;
};

#endif