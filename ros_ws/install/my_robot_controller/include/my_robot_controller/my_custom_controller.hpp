#ifndef MY_ROBOT_CONTROLLER__MY_CUSTOM_CONTROLLER_HPP_
#define MY_ROBOT_CONTROLLER__MY_CUSTOM_CONTROLLER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace my_robot_controller
{
class MyCustomController : public controller_interface::ControllerInterface
{
public:
  MyCustomController() = default;
  ~MyCustomController() override = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory =
    rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  static constexpr std::size_t kJointCount = 6U;
  using JointPositionArray = std::array<double, kJointCount>;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

  bool validate_configured_joints(std::string & error_message) const;
  bool load_limit_parameters(std::string & error_message);
  bool validate_trajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    std::string & error_message) const;
  bool validate_start_segment(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const JointPositionArray & start_positions,
    std::string & error_message) const;
  bool validate_interfaces() const;
  bool validate_command_positions(
    const JointPositionArray & positions,
    std::string & error_message) const;

  JointPositionArray read_current_positions_locked() const;
  bool command_positions(const JointPositionArray & positions);
  void clear_active_goal_locked();
  std::shared_ptr<FollowJointTrajectory::Result> make_result(
    int32_t error_code,
    const std::string & error_string) const;
  void abort_goal(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> & goal_handle,
    const std::string & error_string) const;
  void cancel_goal(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> & goal_handle,
    const std::string & error_string) const;
  void succeed_goal(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> & goal_handle) const;

  std::vector<std::string> joint_names_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

  mutable std::mutex trajectory_mutex_;
  bool is_active_{false};
  bool has_active_trajectory_{false};
  bool cancel_requested_{false};
  std::shared_ptr<GoalHandleFollowJointTrajectory> active_goal_handle_;
  std::uint64_t active_goal_sequence_{0};
  std::uint64_t next_goal_sequence_{0};
  std::uint64_t latest_valid_goal_sequence_{0};
  std::shared_ptr<const trajectory_msgs::msg::JointTrajectory> active_trajectory_;
  JointPositionArray active_start_positions_{};
  JointPositionArray hold_positions_{};
  JointPositionArray position_lower_limits_{{-3.14, -1.57, -1.57, -1.57, -1.46, -3.14}};
  JointPositionArray position_upper_limits_{{3.14, 1.57, 1.57, 1.57, 1.57, 3.14}};
  double max_velocity_{4.5};
  // Conservative rad/s^2 default for optional trajectory accelerations.
  double max_acceleration_{20.0};
  double min_segment_duration_{0.1};
  rclcpp::Time active_start_time_{0, 0, RCL_ROS_TIME};
};
}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__MY_CUSTOM_CONTROLLER_HPP_
