import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 获取包所在的路径
    pkg_name = 'my_robot_description'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. 定位到你的 xacro 文件 
    xacro_file_path = os.path.join(pkg_share, 'urdf/mock_components', 'my_robot_description.urdf.xacro')

    # 2. 使用 xacro 命令解析文件
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file_path]), 
        value_type=str
    )

    # 4. 配置节点
    # Node 1: Robot State Publisher (负责发布 TF 坐标变换)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Node 2: Joint State Publisher GUI (提供一个控制关节角度的滑动条界面)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Node 3: RViz2 (可视化界面)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # 5. 返回 LaunchDescription，启动所有节点
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
