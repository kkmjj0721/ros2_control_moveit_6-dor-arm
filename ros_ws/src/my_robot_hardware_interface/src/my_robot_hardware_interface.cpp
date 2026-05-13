#include "my_robot_hardware_interface/my_robot_hardware_interface.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <exception>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_robot_hardware_interface
{
namespace
{

constexpr const char * kLoggerName = "MyRobotHardwareInterface";
constexpr std::array<const char *, 7> kExpectedJointNames = {
  {"Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6", "Joint_Gripper"}};
constexpr std::array<double, 7> kFallbackPositionMins = {
  {-3.14, -1.57, -1.57, -1.57, -1.46, -3.14, -0.5}};
constexpr std::array<double, 7> kFallbackPositionMaxs = {
  {3.14, 1.57, 1.57, 1.57, 1.57, 3.14, 0.18}};

int expected_joint_index(const std::string & joint_name)
{
  for (size_t i = 0; i < kExpectedJointNames.size(); ++i)
  {
    if (joint_name == kExpectedJointNames[i])
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}

std::string expected_joint_names()
{
  std::string names;
  for (size_t i = 0; i < kExpectedJointNames.size(); ++i)
  {
    if (i > 0)
    {
      names += ", ";
    }
    names += kExpectedJointNames[i];
  }
  return names;
}

bool parse_finite_double(const std::string & value, double & parsed_value)
{
  if (value.empty())
  {
    return false;
  }

  try
  {
    size_t parsed_chars = 0;
    parsed_value = std::stod(value, &parsed_chars);
    return std::isfinite(parsed_value) && std::all_of(
      value.begin() + static_cast<std::string::difference_type>(parsed_chars), value.end(),
      [](const char c) { return std::isspace(static_cast<unsigned char>(c)) != 0; });
  }
  catch (const std::exception &)
  {
    return false;
  }
}

bool parse_initial_value(const std::string & value, double & parsed_value)
{
  return value.empty() || parse_finite_double(value, parsed_value);
}

}  // namespace

hardware_interface::CallbackReturn MyRobotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  const auto logger = rclcpp::get_logger(kLoggerName);

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_FATAL(logger, "Base SystemInterface initialization failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kExpectedJointNames.size())
  {
    RCLCPP_FATAL(
      logger,
      "Expected %zu joints (%s), but ros2_control provided %zu joints.",
      kExpectedJointNames.size(), expected_joint_names().c_str(), info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.assign(info_.joints.size(), 0.0);
  hw_velocities_.assign(info_.joints.size(), 0.0);
  hw_commands_.assign(info_.joints.size(), 0.0);
  last_valid_commands_.assign(info_.joints.size(), 0.0);
  joint_position_mins_.assign(info_.joints.size(), 0.0);
  joint_position_maxs_.assign(info_.joints.size(), 0.0);
  velocity_state_enabled_.assign(info_.joints.size(), 0U);

  std::array<bool, kExpectedJointNames.size()> seen_joints = {};
  size_t velocity_state_count = 0;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const hardware_interface::ComponentInfo & joint = info_.joints[i];
    const int joint_index = expected_joint_index(joint.name);
    if (joint_index < 0)
    {
      RCLCPP_FATAL(
        logger,
        "Unsupported joint '%s'. Expected joints are: %s.",
        joint.name.c_str(), expected_joint_names().c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (seen_joints[static_cast<size_t>(joint_index)])
    {
      RCLCPP_FATAL(logger, "Joint '%s' is declared more than once.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    seen_joints[static_cast<size_t>(joint_index)] = true;

    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        logger,
        "Joint '%s' declares %zu command interfaces; exactly one '%s' command interface is supported.",
        joint.name.c_str(), joint.command_interfaces.size(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        logger,
        "Joint '%s' declares unsupported command interface '%s'; only '%s' is supported.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    const hardware_interface::InterfaceInfo & command_interface = joint.command_interfaces[0];
    const size_t expected_index = static_cast<size_t>(joint_index);
    double position_min = kFallbackPositionMins[expected_index];
    double position_max = kFallbackPositionMaxs[expected_index];

    if (!command_interface.min.empty() && !parse_finite_double(command_interface.min, position_min))
    {
      RCLCPP_FATAL(
        logger,
        "Joint '%s' has invalid '%s' command min '%s'. Expected a finite floating-point number.",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION, command_interface.min.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!command_interface.max.empty() && !parse_finite_double(command_interface.max, position_max))
    {
      RCLCPP_FATAL(
        logger,
        "Joint '%s' has invalid '%s' command max '%s'. Expected a finite floating-point number.",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION, command_interface.max.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (position_min > position_max)
    {
      RCLCPP_FATAL(
        logger,
        "Joint '%s' has invalid '%s' command limits: min %.6f is greater than max %.6f.",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION, position_min, position_max);
      return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_position_state = false;
    bool has_velocity_state = false;
    double initial_position = 0.0;

    for (const hardware_interface::InterfaceInfo & state_interface : joint.state_interfaces)
    {
      if (state_interface.name == hardware_interface::HW_IF_POSITION)
      {
        if (has_position_state)
        {
          RCLCPP_FATAL(
            logger, "Joint '%s' declares duplicate '%s' state interfaces.",
            joint.name.c_str(), hardware_interface::HW_IF_POSITION);
          return hardware_interface::CallbackReturn::ERROR;
        }

        has_position_state = true;
        if (!parse_initial_value(state_interface.initial_value, initial_position))
        {
          RCLCPP_FATAL(
            logger,
            "Joint '%s' has invalid '%s' initial_value '%s'. Expected a floating-point number.",
            joint.name.c_str(), hardware_interface::HW_IF_POSITION, state_interface.initial_value.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
      else if (state_interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        if (has_velocity_state)
        {
          RCLCPP_FATAL(
            logger, "Joint '%s' declares duplicate '%s' state interfaces.",
            joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
          return hardware_interface::CallbackReturn::ERROR;
        }

        has_velocity_state = true;
      }
      else
      {
        RCLCPP_FATAL(
          logger,
          "Joint '%s' declares unsupported state interface '%s'; supported state interfaces are '%s' and '%s'.",
          joint.name.c_str(), state_interface.name.c_str(), hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (!has_position_state)
    {
      RCLCPP_FATAL(
        logger, "Joint '%s' must declare a '%s' state interface.",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (initial_position < position_min || initial_position > position_max)
    {
      RCLCPP_FATAL(
        logger,
        "Joint '%s' has initial position %.6f outside configured command limits [%.6f, %.6f].",
        joint.name.c_str(), initial_position, position_min, position_max);
      return hardware_interface::CallbackReturn::ERROR;
    }

    hw_positions_[i] = initial_position;
    hw_commands_[i] = initial_position;
    last_valid_commands_[i] = initial_position;
    joint_position_mins_[i] = position_min;
    joint_position_maxs_[i] = position_max;
    velocity_state_enabled_[i] = has_velocity_state ? 1U : 0U;
    if (has_velocity_state)
    {
      ++velocity_state_count;
    }
  }

  RCLCPP_INFO(
    logger,
    "Configured simulated position hardware interface for %zu joints; %zu velocity state interfaces exported.",
    info_.joints.size(), velocity_state_count);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyRobotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Activating simulated hardware interface.");

  for (size_t i = 0; i < hw_positions_.size(); ++i)
  {
    hw_commands_[i] = hw_positions_[i];
    last_valid_commands_[i] = hw_positions_[i];
    hw_velocities_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Simulated hardware interface activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyRobotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Simulated hardware interface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MyRobotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyRobotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const auto logger = rclcpp::get_logger(kLoggerName);
  const double period_seconds = period.seconds();
  const auto restore_last_valid_commands = [this]()
  {
    for (size_t j = 0; j < hw_commands_.size(); ++j)
    {
      hw_commands_[j] = last_valid_commands_[j];
    }
  };

  for (size_t i = 0; i < hw_commands_.size(); ++i)
  {
    const double command = hw_commands_[i];

    if (!std::isfinite(command))
    {
      RCLCPP_ERROR(
        logger,
        "Rejected non-finite simulated position command for joint '%s'. State remains unchanged.",
        info_.joints[i].name.c_str());
      restore_last_valid_commands();
      return hardware_interface::return_type::ERROR;
    }

    if (command < joint_position_mins_[i] || command > joint_position_maxs_[i])
    {
      RCLCPP_ERROR(
        logger,
        "Rejected simulated position command %.6f for joint '%s'; allowed range is [%.6f, %.6f]. "
        "State remains unchanged.",
        command, info_.joints[i].name.c_str(), joint_position_mins_[i], joint_position_maxs_[i]);
      restore_last_valid_commands();
      return hardware_interface::return_type::ERROR;
    }
  }

  for (size_t i = 0; i < hw_positions_.size(); ++i)
  {
    const double previous_position = hw_positions_[i];
    hw_positions_[i] = hw_commands_[i];
    last_valid_commands_[i] = hw_commands_[i];

    if (velocity_state_enabled_[i] != 0U)
    {
      hw_velocities_[i] =
        period_seconds > 0.0 ? (hw_positions_[i] - previous_position) / period_seconds : 0.0;
    }
  }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> MyRobotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    for (const hardware_interface::InterfaceInfo & state_interface : info_.joints[i].state_interfaces)
    {
      if (state_interface.name == hardware_interface::HW_IF_POSITION)
      {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
      }
      else if (state_interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MyRobotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}
}   // namespace my_robot_hardware_interface

// 将其注册为插件
PLUGINLIB_EXPORT_CLASS(
  my_robot_hardware_interface::MyRobotHardwareInterface,
  hardware_interface::SystemInterface
)
