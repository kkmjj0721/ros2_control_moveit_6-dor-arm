cmake_minimum_required(VERSION 3.8)
project(my_robot_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(my_robot_hardware_interface SHARED
  src/my_robot_hardware_interface.cpp
)
target_include_directories(my_robot_hardware_interface PRIVATE include)
ament_target_dependencies(my_robot_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

install(TARGETS my_robot_hardware_interface
  DESTINATION lib
)
install(FILES my_robot_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
