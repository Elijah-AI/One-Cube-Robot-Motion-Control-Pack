"""启动自制夹爪节点 ds485_node。

用法：
  ros2 launch ds485_driver ds485_driver.launch.py
  ros2 launch ds485_driver ds485_driver.launch.py config:=/path/to/my.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('ds485_driver'), 'config', 'ds485_driver.yaml')

    config_arg = DeclareLaunchArgument(
        'config', default_value=default_config,
        description='ds485_node 参数文件路径')

    safety_arg = DeclareLaunchArgument(
        'safety_config', default_value=os.path.join(
            get_package_share_directory('ds485_driver'), 'config', 'gripper_safety.yaml'),
        description='两侧轨迹、接触和热保护标定')

    control_arg = DeclareLaunchArgument('control', default_value='position',
                                        choices=['position', 'torque'],
                                        description='位置轨迹或电机恒定 PWM')

    level_arg = DeclareLaunchArgument('log_level', default_value='info',
                                      description='节点日志等级，抓通信报文时使用 debug')
    node = Node(
        package='ds485_driver',
        executable='ds485_node',
        name='ds485_node',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', ['ds485_node:=', LaunchConfiguration('log_level')]],
        parameters=[LaunchConfiguration('config'), LaunchConfiguration('safety_config'),
                    {'control_strategy': LaunchConfiguration('control')}],
    )
    return LaunchDescription([config_arg, safety_arg, control_arg, level_arg, node])
