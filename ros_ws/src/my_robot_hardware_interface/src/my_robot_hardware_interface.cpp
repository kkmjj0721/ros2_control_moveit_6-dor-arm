#include "my_robot_hardware_interface/my_robot_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_robot_hardware_interface
{

hardware_interface::CallbackReturn MyRobotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
    // 调用父类初始化
    if (hardware_interface::SystemInterface::on_init(info) !=hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 7 个控制关节
    hw_states_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    // 检查 xacro 中的接口配置是否正确匹配 (只要 position)
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 1 ||
            joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
        RCLCPP_FATAL(
            rclcpp::get_logger("MyRobotHardwareInterface"),
            "Joint '%s' has %zu command interfaces found. 1 expected (position).",
            joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyRobotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MyRobotHardwareInterface"), "Activating... please wait.");

  // 同步初始状态
  for (size_t i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("MyRobotHardwareInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyRobotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MyRobotHardwareInterface"), "Deactivating...");

  RCLCPP_INFO(rclcpp::get_logger("MyRobotHardwareInterface"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MyRobotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyRobotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> MyRobotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MyRobotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
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