#include "my_robot_controller/my_custom_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_robot_controller
{
controller_interface::CallbackReturn MyCustomController::on_init()
{
    // 初始化，从参数服务器获取 joint 名字
    try {
        auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    } catch (const std::exception & e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyCustomController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MyCustomController::command_interface_configuration() const
{
  // 声明我们需要向 Hardware Interface 发送哪些命令 (例如: 位置)
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

controller_interface::InterfaceConfiguration MyCustomController::state_interface_configuration() const
{
  // 声明我们需要从 Hardware Interface 读取哪些状态 (例如: 位置)
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

controller_interface::CallbackReturn MyCustomController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyCustomController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MyCustomController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 编写你的控制算法逻辑
  // 例如：读取当前位置，计算误差，写入新命令
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    // 简单示例：将读取到的状态直接作为命令写回 (仅作演示，实际需要控制算法)
    // double current_position = state_interfaces_[i].get_value();
    // command_interfaces_[i].set_value(current_position + 0.01); 
  }
  return controller_interface::return_type::OK;
}

}  // namespace my_robot_controller

// 导出为插件 (非常重要)
PLUGINLIB_EXPORT_CLASS(
  my_robot_controller::MyCustomController,
  controller_interface::ControllerInterface)