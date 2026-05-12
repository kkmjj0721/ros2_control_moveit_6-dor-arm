#ifndef MY_ROBOT_CONTROLLER__MY_CUSTOM_CONTROLLER_HPP_
#define MY_ROBOT_CONTROLLER__MY_CUSTOM_CONTROLLER_HPP_

#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_robot_controller
{
class MyCustomController : public controller_interface::ControllerInterface
{
public:
    MyCustomController() = default;
    ~MyCustomController() = default;

    // 生命周期回调函数
    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // 定义接口配置函数
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    // 定义控制更新函数
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
private:
    std::vector<std::string> joint_names_;    
};
}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__MY_CUSTOM_CONTROLLER_HPP_