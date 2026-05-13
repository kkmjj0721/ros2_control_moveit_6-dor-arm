# ros2_control_moveit_6-dor-arm
使用ros2control+moveit2的六自由度机械臂的夹取仿真实现

## Build and source the workspace:

```
colcon build
source install/setup.bash
```




## Usage:

- To display the robot model in RViz:

  ```
  ros2 launch my_robot_description display.launch.py
  ```

- To run demo：

  ```
  ros2 launch my_robot_moveit_config demo.launch.py
  ros2 topic echo /display_planned_path
  ```

  - To view the hardware_interface:

    ```
    ros2 control list_hardware_interfaces
    ```

  - To view the controller:

    ```
    ros2 control list_controllers
    ```

  - To view the action server:

    ```
    ros2 action list | grep follow_joint_trajectory
    ```

  - To view the joint state:

    ```
    ros2 topic echo /joint_states
    ```

  - open the girpper:

    ```
    ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
        trajectory: {
          joint_names: ['Joint_Gripper'],
          points: [
            {positions: [-0.5], time_from_start: {sec: 2, nanosec: 0}}
          ]
        }
      }"			
    ```

  - close the girpper:

    ```
    ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
        trajectory: {
          joint_names: ['Joint_Gripper'],
          points: [
            {positions: [0.18], time_from_start: {sec: 2, nanosec: 0}}
          ]
        }
      }"
    ```

  - To moveit the arm:							

    ```
    ros2 action send_goal /my_custom_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
        trajectory: {
          joint_names: ['Joint_1','Joint_2','Joint_3','Joint_4','Joint_5','Joint_6'],
          points: [
            {
              positions: [0.3, -0.3, 0.4, 0.5, 0.2, 0.6],
              time_from_start: {sec: 4, nanosec: 0}
            }
          ]
        }
      }"
    ```

    

