#include "my_robot_controller/my_custom_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{
constexpr double kNanosecondsPerSecond = 1000000000.0;
constexpr double kLimitTolerance = 1.0e-9;
constexpr double kStationaryTolerance = 1.0e-9;
constexpr double kHardMaxVelocity = 4.5;
constexpr std::size_t kExpectedJointCount = 6U;

const std::vector<std::string> & required_joint_names()
{
  static const std::vector<std::string> names = {
    "Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"};
  return names;
}

const std::array<double, kExpectedJointCount> & hard_lower_limits()
{
  static const std::array<double, kExpectedJointCount> limits = {
    {-3.14, -1.57, -1.57, -1.57, -1.46, -3.14}};
  return limits;
}

const std::array<double, kExpectedJointCount> & hard_upper_limits()
{
  static const std::array<double, kExpectedJointCount> limits = {
    {3.14, 1.57, 1.57, 1.57, 1.57, 3.14}};
  return limits;
}

std::string join_joint_names(const std::vector<std::string> & names)
{
  std::ostringstream stream;
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    if (i > 0U)
    {
      stream << ", ";
    }
    stream << names[i];
  }
  return stream.str();
}

std::int64_t duration_to_nanoseconds(const builtin_interfaces::msg::Duration & duration)
{
  return rclcpp::Duration(duration).nanoseconds();
}

double nanoseconds_to_seconds(const std::int64_t nanoseconds)
{
  return static_cast<double>(nanoseconds) / kNanosecondsPerSecond;
}

std::int64_t seconds_to_nanoseconds(const double seconds)
{
  return static_cast<std::int64_t>(std::ceil(seconds * kNanosecondsPerSecond));
}

std::vector<double> limit_array_to_vector(const std::array<double, kExpectedJointCount> & values)
{
  return std::vector<double>(values.begin(), values.end());
}

double interpolate_ratio(
  const std::int64_t elapsed_ns,
  const std::int64_t start_ns,
  const std::int64_t end_ns)
{
  if (end_ns <= start_ns)
  {
    return 1.0;
  }

  const double ratio = static_cast<double>(elapsed_ns - start_ns) /
    static_cast<double>(end_ns - start_ns);
  return std::max(0.0, std::min(1.0, ratio));
}

bool copy_limit_vector_to_array(
  const std::vector<double> & values,
  const std::string & parameter_name,
  std::array<double, kExpectedJointCount> & output,
  std::string & error_message)
{
  if (values.size() != output.size())
  {
    error_message = parameter_name + " must contain " + std::to_string(output.size()) +
      " values, got " + std::to_string(values.size());
    return false;
  }

  for (std::size_t i = 0; i < output.size(); ++i)
  {
    if (!std::isfinite(values[i]))
    {
      error_message = parameter_name + " contains a non-finite value for " +
        required_joint_names()[i];
      return false;
    }
    output[i] = values[i];
  }

  return true;
}

bool validate_position_limit_arrays(
  const std::array<double, kExpectedJointCount> & lower_limits,
  const std::array<double, kExpectedJointCount> & upper_limits,
  std::string & error_message)
{
  const auto & hard_lower = hard_lower_limits();
  const auto & hard_upper = hard_upper_limits();
  const auto & joint_names = required_joint_names();

  for (std::size_t i = 0; i < lower_limits.size(); ++i)
  {
    if (lower_limits[i] > upper_limits[i])
    {
      error_message = "position limits for " + joint_names[i] +
        " have lower limit greater than upper limit";
      return false;
    }

    if (lower_limits[i] < (hard_lower[i] - kLimitTolerance))
    {
      error_message = "position_lower_limits for " + joint_names[i] +
        " is below hard safety limit " + std::to_string(hard_lower[i]);
      return false;
    }

    if (upper_limits[i] > (hard_upper[i] + kLimitTolerance))
    {
      error_message = "position_upper_limits for " + joint_names[i] +
        " is above hard safety limit " + std::to_string(hard_upper[i]);
      return false;
    }
  }

  return true;
}

bool validate_position_vector(
  const std::vector<double> & positions,
  const std::vector<std::string> & joint_names,
  const std::array<double, kExpectedJointCount> & lower_limits,
  const std::array<double, kExpectedJointCount> & upper_limits,
  const std::size_t point_index,
  std::string & error_message)
{
  if (positions.size() != joint_names.size())
  {
    error_message = "point " + std::to_string(point_index) + " has " +
      std::to_string(positions.size()) + " positions, expected " +
      std::to_string(joint_names.size());
    return false;
  }

  for (std::size_t joint_index = 0; joint_index < positions.size(); ++joint_index)
  {
    if (!std::isfinite(positions[joint_index]))
    {
      error_message = "point " + std::to_string(point_index) +
        " contains a non-finite position for " + joint_names[joint_index];
      return false;
    }

    if (
      (positions[joint_index] < (lower_limits[joint_index] - kLimitTolerance)) ||
      (positions[joint_index] > (upper_limits[joint_index] + kLimitTolerance)))
    {
      error_message = "point " + std::to_string(point_index) + " position for " +
        joint_names[joint_index] + " is outside [" +
        std::to_string(lower_limits[joint_index]) + ", " +
        std::to_string(upper_limits[joint_index]) + "]";
      return false;
    }
  }

  return true;
}

bool validate_joint_vector_field(
  const std::vector<double> & values,
  const std::string & field_name,
  const std::vector<std::string> & joint_names,
  const std::size_t point_index,
  const bool enforce_absolute_limit,
  const double absolute_limit,
  std::string & error_message)
{
  if (values.empty())
  {
    return true;
  }

  if (values.size() != joint_names.size())
  {
    error_message = "point " + std::to_string(point_index) + " has " +
      std::to_string(values.size()) + " " + field_name + ", expected " +
      std::to_string(joint_names.size());
    return false;
  }

  for (std::size_t joint_index = 0; joint_index < values.size(); ++joint_index)
  {
    if (!std::isfinite(values[joint_index]))
    {
      error_message = "point " + std::to_string(point_index) + " contains a non-finite " +
        field_name + " value for " + joint_names[joint_index];
      return false;
    }

    if (
      enforce_absolute_limit &&
      (std::abs(values[joint_index]) > (absolute_limit + kLimitTolerance)))
    {
      error_message = "point " + std::to_string(point_index) + " " + field_name +
        " value for " + joint_names[joint_index] + " exceeds limit " +
        std::to_string(absolute_limit);
      return false;
    }
  }

  return true;
}

bool validate_segment_timing_and_velocity(
  const std::vector<double> & start_positions,
  const std::vector<double> & end_positions,
  const std::vector<std::string> & joint_names,
  const std::int64_t duration_ns,
  const double max_velocity,
  const double min_segment_duration,
  const bool allow_zero_duration_stationary,
  const std::string & segment_name,
  std::string & error_message)
{
  if ((start_positions.size() != joint_names.size()) || (end_positions.size() != joint_names.size()))
  {
    error_message = segment_name + " has a vector size mismatch";
    return false;
  }

  bool has_motion = false;
  for (std::size_t joint_index = 0; joint_index < joint_names.size(); ++joint_index)
  {
    if (!std::isfinite(start_positions[joint_index]) || !std::isfinite(end_positions[joint_index]))
    {
      error_message = segment_name + " contains a non-finite position for " +
        joint_names[joint_index];
      return false;
    }

    if (std::abs(end_positions[joint_index] - start_positions[joint_index]) >
      kStationaryTolerance)
    {
      has_motion = true;
    }
  }

  if (duration_ns < 0)
  {
    error_message = segment_name + " has a negative duration";
    return false;
  }

  if (duration_ns == 0)
  {
    if (has_motion || !allow_zero_duration_stationary)
    {
      error_message = segment_name + " has zero duration for commanded motion";
      return false;
    }
    return true;
  }

  const std::int64_t minimum_duration_ns = seconds_to_nanoseconds(min_segment_duration);
  if (has_motion && (minimum_duration_ns > 0) && (duration_ns < minimum_duration_ns))
  {
    error_message = segment_name + " duration " +
      std::to_string(nanoseconds_to_seconds(duration_ns)) +
      " is shorter than minimum segment duration " +
      std::to_string(min_segment_duration);
    return false;
  }

  const double duration_seconds = nanoseconds_to_seconds(duration_ns);
  for (std::size_t joint_index = 0; joint_index < joint_names.size(); ++joint_index)
  {
    const double velocity =
      std::abs(end_positions[joint_index] - start_positions[joint_index]) / duration_seconds;
    if (velocity > (max_velocity + kLimitTolerance))
    {
      error_message = segment_name + " velocity for " + joint_names[joint_index] +
        " exceeds " + std::to_string(max_velocity) + " rad/s";
      return false;
    }
  }

  return true;
}

std::array<double, kExpectedJointCount> sample_trajectory_command(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::array<double, kExpectedJointCount> & start_positions,
  const std::array<double, kExpectedJointCount> & fallback_positions,
  const rclcpp::Time & start_time,
  const rclcpp::Time & time,
  bool & trajectory_finished)
{
  std::array<double, kExpectedJointCount> command = fallback_positions;
  trajectory_finished = false;

  if (trajectory.points.empty())
  {
    return command;
  }

  std::int64_t elapsed_ns = (time - start_time).nanoseconds();
  if (elapsed_ns < 0)
  {
    elapsed_ns = 0;
  }

  const auto & points = trajectory.points;
  const std::int64_t first_time_ns = duration_to_nanoseconds(points.front().time_from_start);

  if (elapsed_ns <= first_time_ns)
  {
    const double ratio = first_time_ns > 0 ?
      static_cast<double>(elapsed_ns) / static_cast<double>(first_time_ns) : 1.0;

    for (std::size_t i = 0; i < command.size(); ++i)
    {
      command[i] =
        start_positions[i] + ((points.front().positions[i] - start_positions[i]) * ratio);
    }

    trajectory_finished = (points.size() == 1U) && (elapsed_ns >= first_time_ns);
    return command;
  }

  for (std::size_t point_index = 1U; point_index < points.size(); ++point_index)
  {
    const std::int64_t segment_start_ns =
      duration_to_nanoseconds(points[point_index - 1U].time_from_start);
    const std::int64_t segment_end_ns =
      duration_to_nanoseconds(points[point_index].time_from_start);

    if (elapsed_ns <= segment_end_ns)
    {
      const double ratio = interpolate_ratio(elapsed_ns, segment_start_ns, segment_end_ns);

      for (std::size_t i = 0; i < command.size(); ++i)
      {
        const double start_position = points[point_index - 1U].positions[i];
        const double end_position = points[point_index].positions[i];
        command[i] = start_position + ((end_position - start_position) * ratio);
      }

      trajectory_finished =
        (point_index == (points.size() - 1U)) && (elapsed_ns >= segment_end_ns);
      return command;
    }
  }

  for (std::size_t i = 0; i < command.size(); ++i)
  {
    command[i] = points.back().positions[i];
  }
  trajectory_finished = true;

  return command;
}
}  // namespace

namespace my_robot_controller
{
controller_interface::CallbackReturn MyCustomController::on_init()
{
  try
  {
    auto_declare<std::vector<std::string>>("joints", required_joint_names());
    auto_declare<std::vector<double>>(
      "position_lower_limits", limit_array_to_vector(position_lower_limits_));
    auto_declare<std::vector<double>>(
      "position_upper_limits", limit_array_to_vector(position_upper_limits_));
    auto_declare<double>("max_velocity", max_velocity_);
    auto_declare<double>("max_acceleration", max_acceleration_);
    auto_declare<double>("min_segment_duration", min_segment_duration_);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyCustomController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  std::string error_message;
  if (!validate_configured_joints(error_message))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error_message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!load_limit_parameters(error_message))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error_message.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    get_node(), "~/follow_joint_trajectory",
    std::bind(&MyCustomController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MyCustomController::handle_cancel, this, std::placeholders::_1),
    std::bind(&MyCustomController::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured FollowJointTrajectory action '~/follow_joint_trajectory' for joints: %s",
    join_joint_names(joint_names_).c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MyCustomController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(joint_names_.size());

  for (const auto & joint : joint_names_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }

  return config;
}

controller_interface::InterfaceConfiguration MyCustomController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(joint_names_.size());

  for (const auto & joint : joint_names_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }

  return config;
}

controller_interface::CallbackReturn MyCustomController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!validate_interfaces())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  JointPositionArray initial_hold_positions{};

  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    hold_positions_ = read_current_positions_locked();
    active_start_positions_ = hold_positions_;
    active_start_time_ = get_node()->now();
    clear_active_goal_locked();
    std::string error_message;
    if (!validate_command_positions(hold_positions_, error_message))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Refusing activation with unsafe hold position: %s",
        error_message.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    initial_hold_positions = hold_positions_;
    is_active_ = true;
  }

  return command_positions(initial_hold_positions) ?
         controller_interface::CallbackReturn::SUCCESS :
         controller_interface::CallbackReturn::ERROR;
}

controller_interface::CallbackReturn MyCustomController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::shared_ptr<GoalHandleFollowJointTrajectory> goal_to_abort;

  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    is_active_ = false;
    goal_to_abort = active_goal_handle_;
    clear_active_goal_locked();
  }

  abort_goal(goal_to_abort, "Controller deactivated before trajectory completion");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyCustomController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::shared_ptr<GoalHandleFollowJointTrajectory> goal_to_abort;

  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    is_active_ = false;
    goal_to_abort = active_goal_handle_;
    clear_active_goal_locked();
    active_trajectory_.reset();
  }

  abort_goal(goal_to_abort, "Controller cleaned up before trajectory completion");
  action_server_.reset();
  joint_names_.clear();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MyCustomController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & /*period*/)
{
  JointPositionArray command{};
  bool should_command = false;
  bool should_sample_trajectory = false;
  std::shared_ptr<const trajectory_msgs::msg::JointTrajectory> trajectory_to_sample;
  JointPositionArray sample_start_positions{};
  JointPositionArray sample_fallback_positions{};
  rclcpp::Time sample_start_time{0, 0, RCL_ROS_TIME};
  std::shared_ptr<GoalHandleFollowJointTrajectory> sampled_goal_handle;
  std::shared_ptr<GoalHandleFollowJointTrajectory> goal_to_abort;
  std::shared_ptr<GoalHandleFollowJointTrajectory> goal_to_cancel;
  std::shared_ptr<GoalHandleFollowJointTrajectory> goal_to_succeed;
  std::string abort_reason;

  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    command = hold_positions_;

    if (is_active_)
    {
      should_command = true;

      if (has_active_trajectory_)
      {
        if (cancel_requested_)
        {
          goal_to_cancel = active_goal_handle_;
          clear_active_goal_locked();
        }
        else if (!active_trajectory_)
        {
          goal_to_abort = active_goal_handle_;
          abort_reason = "Active trajectory state is empty";
          clear_active_goal_locked();
        }
        else
        {
          should_sample_trajectory = true;
          trajectory_to_sample = active_trajectory_;
          sample_start_positions = active_start_positions_;
          sample_fallback_positions = hold_positions_;
          sample_start_time = active_start_time_;
          sampled_goal_handle = active_goal_handle_;
        }
      }
    }
  }

  if (should_sample_trajectory && trajectory_to_sample)
  {
    bool trajectory_finished = false;
    const JointPositionArray sampled_command = sample_trajectory_command(
      *trajectory_to_sample, sample_start_positions, sample_fallback_positions,
      sample_start_time, time, trajectory_finished);
    std::string command_error;

    std::lock_guard<std::mutex> lock(trajectory_mutex_);

    if (!is_active_)
    {
      should_command = false;
    }
    else if (
      !has_active_trajectory_ || (active_goal_handle_ != sampled_goal_handle) ||
      (active_trajectory_ != trajectory_to_sample))
    {
      command = hold_positions_;
      should_command = true;
    }
    else if (cancel_requested_)
    {
      goal_to_cancel = active_goal_handle_;
      clear_active_goal_locked();
      command = hold_positions_;
      should_command = true;
    }
    else if (!validate_command_positions(sampled_command, command_error))
    {
      goal_to_abort = active_goal_handle_;
      abort_reason = "Unsafe trajectory sample rejected: " + command_error;
      clear_active_goal_locked();
      command = hold_positions_;
      should_command = true;
    }
    else
    {
      command = sampled_command;
      hold_positions_ = command;
      should_command = true;

      if (trajectory_finished)
      {
        goal_to_succeed = active_goal_handle_;
        clear_active_goal_locked();
      }
    }
  }

  if (should_command)
  {
    if (!command_positions(command) && !goal_to_abort)
    {
      abort_reason = "Unsafe hold command rejected by final command check";
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      goal_to_abort = active_goal_handle_;
      clear_active_goal_locked();
    }
  }

  abort_goal(goal_to_abort, abort_reason);
  cancel_goal(goal_to_cancel, "Trajectory goal canceled");
  succeed_goal(goal_to_succeed);

  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse MyCustomController::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  if (!goal)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Rejecting trajectory goal: goal message is null");
    return rclcpp_action::GoalResponse::REJECT;
  }

  JointPositionArray start_positions{};

  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (!is_active_)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Rejecting trajectory goal: controller is inactive");
      return rclcpp_action::GoalResponse::REJECT;
    }

    start_positions = read_current_positions_locked();
  }

  std::string error_message;
  if (!validate_trajectory(goal->trajectory, error_message))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Rejecting trajectory goal: %s", error_message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!validate_start_segment(goal->trajectory, start_positions, error_message))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Rejecting trajectory goal: %s", error_message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MyCustomController::handle_cancel(
  const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  std::lock_guard<std::mutex> lock(trajectory_mutex_);

  if (has_active_trajectory_ && (active_goal_handle_ == goal_handle))
  {
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  return rclcpp_action::CancelResponse::REJECT;
}

void MyCustomController::handle_accepted(
  std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  if (!goal)
  {
    abort_goal(goal_handle, "Trajectory goal became null before acceptance");
    return;
  }

  auto trajectory =
    std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal->trajectory);
  std::shared_ptr<GoalHandleFollowJointTrajectory> goal_to_abort;
  bool accepted_while_inactive = false;
  std::uint64_t goal_sequence = 0;
  std::string error_message;

  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    ++next_goal_sequence_;
    goal_sequence = next_goal_sequence_;
  }

  if (!validate_trajectory(*trajectory, error_message))
  {
    abort_goal(goal_handle, error_message);
    return;
  }

  JointPositionArray start_positions{};

  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);

    if (!is_active_)
    {
      accepted_while_inactive = true;
    }
    else
    {
      start_positions = read_current_positions_locked();
    }
  }

  if (accepted_while_inactive)
  {
    abort_goal(goal_handle, "Controller became inactive before accepting trajectory");
    return;
  }

  if (!validate_start_segment(*trajectory, start_positions, error_message))
  {
    abort_goal(goal_handle, error_message);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);

    if (!is_active_)
    {
      accepted_while_inactive = true;
    }
    else if (goal_sequence < latest_valid_goal_sequence_)
    {
      goal_to_abort = goal_handle;
      error_message = "Trajectory goal superseded by a newer goal";
    }
    else
    {
      latest_valid_goal_sequence_ = goal_sequence;
      if (has_active_trajectory_ && (active_goal_handle_ != goal_handle))
      {
        goal_to_abort = active_goal_handle_;
      }

      active_goal_handle_ = goal_handle;
      active_trajectory_ = trajectory;
      active_goal_sequence_ = goal_sequence;
      active_start_positions_ = start_positions;
      active_start_time_ = get_node()->now();
      has_active_trajectory_ = true;
      cancel_requested_ = false;
    }
  }

  if (accepted_while_inactive)
  {
    abort_goal(goal_handle, "Controller became inactive before accepting trajectory");
    return;
  }

  abort_goal(
    goal_to_abort,
    error_message.empty() ? "Trajectory goal preempted by a newer goal" : error_message);
}

bool MyCustomController::validate_configured_joints(std::string & error_message) const
{
  const auto & required = required_joint_names();
  if (joint_names_ != required)
  {
    error_message = "Controller joints must be exactly [" + join_joint_names(required) +
      "], but configured joints are [" + join_joint_names(joint_names_) + "]";
    return false;
  }

  return true;
}

bool MyCustomController::load_limit_parameters(std::string & error_message)
{
  JointPositionArray lower_limits{};
  JointPositionArray upper_limits{};

  if (!copy_limit_vector_to_array(
      get_node()->get_parameter("position_lower_limits").as_double_array(),
      "position_lower_limits", lower_limits, error_message))
  {
    return false;
  }

  if (!copy_limit_vector_to_array(
      get_node()->get_parameter("position_upper_limits").as_double_array(),
      "position_upper_limits", upper_limits, error_message))
  {
    return false;
  }

  if (!validate_position_limit_arrays(lower_limits, upper_limits, error_message))
  {
    return false;
  }

  const double max_velocity = get_node()->get_parameter("max_velocity").as_double();
  if (
    !std::isfinite(max_velocity) || (max_velocity <= 0.0) ||
    (max_velocity > (kHardMaxVelocity + kLimitTolerance)))
  {
    error_message = "max_velocity must be finite and in (0, " +
      std::to_string(kHardMaxVelocity) + "]";
    return false;
  }

  const double max_acceleration = get_node()->get_parameter("max_acceleration").as_double();
  if (!std::isfinite(max_acceleration) || (max_acceleration <= 0.0))
  {
    error_message = "max_acceleration must be finite and positive";
    return false;
  }

  const double min_segment_duration =
    get_node()->get_parameter("min_segment_duration").as_double();
  if (!std::isfinite(min_segment_duration) || (min_segment_duration < 0.0))
  {
    error_message = "min_segment_duration must be finite and non-negative";
    return false;
  }

  position_lower_limits_ = lower_limits;
  position_upper_limits_ = upper_limits;
  max_velocity_ = max_velocity;
  max_acceleration_ = max_acceleration;
  min_segment_duration_ = min_segment_duration;

  return true;
}

bool MyCustomController::validate_trajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  std::string & error_message) const
{
  if (trajectory.points.empty())
  {
    error_message = "trajectory has no points";
    return false;
  }

  if (trajectory.joint_names != joint_names_)
  {
    error_message = "trajectory joint_names must exactly match configured joints [" +
      join_joint_names(joint_names_) + "], got [" + join_joint_names(trajectory.joint_names) + "]";
    return false;
  }

  std::int64_t previous_time_ns = -1;
  for (std::size_t point_index = 0; point_index < trajectory.points.size(); ++point_index)
  {
    const auto & point = trajectory.points[point_index];

    if (!validate_position_vector(
        point.positions, joint_names_, position_lower_limits_, position_upper_limits_,
        point_index, error_message))
    {
      return false;
    }

    if (!validate_joint_vector_field(
        point.velocities, "velocity", joint_names_, point_index, true, max_velocity_,
        error_message))
    {
      return false;
    }

    if (!validate_joint_vector_field(
        point.accelerations, "acceleration", joint_names_, point_index, true, max_acceleration_,
        error_message))
    {
      return false;
    }

    if (!validate_joint_vector_field(
        point.effort, "effort", joint_names_, point_index, false, 0.0, error_message))
    {
      return false;
    }

    const std::int64_t point_time_ns = duration_to_nanoseconds(point.time_from_start);
    if (point_time_ns < 0)
    {
      error_message = "point " + std::to_string(point_index) +
        " has a negative time_from_start";
      return false;
    }

    if ((point_index > 0U) && (point_time_ns <= previous_time_ns))
    {
      error_message = "trajectory point times must be strictly increasing";
      return false;
    }

    if (point_index > 0U)
    {
      const std::int64_t segment_duration_ns = point_time_ns - previous_time_ns;
      const std::string segment_name =
        "trajectory segment " + std::to_string(point_index - 1U) + "->" +
        std::to_string(point_index);
      if (!validate_segment_timing_and_velocity(
          trajectory.points[point_index - 1U].positions, point.positions, joint_names_,
          segment_duration_ns, max_velocity_, min_segment_duration_, false, segment_name,
          error_message))
      {
        return false;
      }
    }

    previous_time_ns = point_time_ns;
  }

  return true;
}

bool MyCustomController::validate_start_segment(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const JointPositionArray & start_positions,
  std::string & error_message) const
{
  if (trajectory.points.empty())
  {
    error_message = "trajectory has no points";
    return false;
  }

  if (trajectory.joint_names != joint_names_)
  {
    error_message = "trajectory joint_names must exactly match configured joints [" +
      join_joint_names(joint_names_) + "], got [" + join_joint_names(trajectory.joint_names) + "]";
    return false;
  }

  if (!validate_command_positions(start_positions, error_message))
  {
    error_message = "start state is unsafe: " + error_message;
    return false;
  }

  const std::int64_t first_time_ns =
    duration_to_nanoseconds(trajectory.points.front().time_from_start);
  return validate_segment_timing_and_velocity(
    limit_array_to_vector(start_positions), trajectory.points.front().positions, joint_names_,
    first_time_ns, max_velocity_, min_segment_duration_, true, "start segment", error_message);
}

bool MyCustomController::validate_command_positions(
  const JointPositionArray & positions,
  std::string & error_message) const
{
  const auto & names =
    (joint_names_.size() == positions.size()) ? joint_names_ : required_joint_names();

  if (names.size() != positions.size())
  {
    error_message = "command vector size does not match expected joint count";
    return false;
  }

  for (std::size_t joint_index = 0; joint_index < positions.size(); ++joint_index)
  {
    if (!std::isfinite(positions[joint_index]))
    {
      error_message = "command for " + names[joint_index] + " is non-finite";
      return false;
    }

    if (
      (positions[joint_index] < (position_lower_limits_[joint_index] - kLimitTolerance)) ||
      (positions[joint_index] > (position_upper_limits_[joint_index] + kLimitTolerance)))
    {
      error_message = "command for " + names[joint_index] + " is outside [" +
        std::to_string(position_lower_limits_[joint_index]) + ", " +
        std::to_string(position_upper_limits_[joint_index]) + "]";
      return false;
    }
  }

  return true;
}

bool MyCustomController::validate_interfaces() const
{
  if (command_interfaces_.size() != joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu position command interfaces, got %zu",
      joint_names_.size(), command_interfaces_.size());
    return false;
  }

  if (state_interfaces_.size() != joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu position state interfaces, got %zu",
      joint_names_.size(), state_interfaces_.size());
    return false;
  }

  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    const std::string expected_name = joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION;

    if (command_interfaces_[i].get_name() != expected_name)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Command interface %zu is '%s', expected '%s'", i,
        command_interfaces_[i].get_name().c_str(), expected_name.c_str());
      return false;
    }

    if (state_interfaces_[i].get_name() != expected_name)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "State interface %zu is '%s', expected '%s'", i,
        state_interfaces_[i].get_name().c_str(), expected_name.c_str());
      return false;
    }
  }

  return true;
}

MyCustomController::JointPositionArray MyCustomController::read_current_positions_locked() const
{
  JointPositionArray positions = hold_positions_;

  for (std::size_t i = 0; i < positions.size(); ++i)
  {
    if (i < state_interfaces_.size())
    {
      const double value = state_interfaces_[i].get_value();
      if (std::isfinite(value))
      {
        positions[i] = value;
      }
    }
  }

  return positions;
}

bool MyCustomController::command_positions(const JointPositionArray & positions)
{
  std::string error_message;
  if (!validate_command_positions(positions, error_message))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Refusing unsafe command: %s", error_message.c_str());
    return false;
  }

  if (command_interfaces_.size() != positions.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu", positions.size(),
      command_interfaces_.size());
    return false;
  }

  for (std::size_t i = 0; i < positions.size(); ++i)
  {
    command_interfaces_[i].set_value(positions[i]);
  }

  return true;
}

void MyCustomController::clear_active_goal_locked()
{
  has_active_trajectory_ = false;
  cancel_requested_ = false;
  active_goal_handle_.reset();
  active_goal_sequence_ = 0;
  active_trajectory_.reset();
}

std::shared_ptr<MyCustomController::FollowJointTrajectory::Result> MyCustomController::make_result(
  const int32_t error_code,
  const std::string & error_string) const
{
  auto result = std::make_shared<FollowJointTrajectory::Result>();
  result->error_code = error_code;
  result->error_string = error_string;
  return result;
}

void MyCustomController::abort_goal(
  const std::shared_ptr<GoalHandleFollowJointTrajectory> & goal_handle,
  const std::string & error_string) const
{
  if (!goal_handle || !goal_handle->is_active())
  {
    return;
  }

  try
  {
    goal_handle->abort(make_result(FollowJointTrajectory::Result::INVALID_GOAL, error_string));
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(get_node()->get_logger(), "Failed to abort trajectory goal: %s", e.what());
  }
}

void MyCustomController::cancel_goal(
  const std::shared_ptr<GoalHandleFollowJointTrajectory> & goal_handle,
  const std::string & error_string) const
{
  if (!goal_handle || !goal_handle->is_active())
  {
    return;
  }

  try
  {
    goal_handle->canceled(make_result(FollowJointTrajectory::Result::SUCCESSFUL, error_string));
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(get_node()->get_logger(), "Failed to cancel trajectory goal: %s", e.what());
  }
}

void MyCustomController::succeed_goal(
  const std::shared_ptr<GoalHandleFollowJointTrajectory> & goal_handle) const
{
  if (!goal_handle || !goal_handle->is_active())
  {
    return;
  }

  try
  {
    goal_handle->succeed(make_result(FollowJointTrajectory::Result::SUCCESSFUL, ""));
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(get_node()->get_logger(), "Failed to succeed trajectory goal: %s", e.what());
  }
}

}  // namespace my_robot_controller

PLUGINLIB_EXPORT_CLASS(
  my_robot_controller::MyCustomController,
  controller_interface::ControllerInterface)
