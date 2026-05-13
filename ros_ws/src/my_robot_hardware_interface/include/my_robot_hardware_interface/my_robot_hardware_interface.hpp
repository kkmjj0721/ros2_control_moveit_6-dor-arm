#ifndef MY_ROBOT_HARDWARE_INTERFACE_HPP_
#define MY_ROBOT_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace my_robot_hardware_interface
{

class MyRobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MyRobotHardwareInterface)
    // 生命周期节点接口
    hardware_interface::CallbackReturn 
        on_init(const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn 
        on_activate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn 
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // 读写接口
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    // 暴露状态和命令接口给Controller Manager
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
    std::vector<double> hw_commands_;
    std::vector<double> last_valid_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> joint_position_mins_;
    std::vector<double> joint_position_maxs_;
    std::vector<uint8_t> velocity_state_enabled_;

};

}  // namespace my_robot_hardware_interface

#endif  
