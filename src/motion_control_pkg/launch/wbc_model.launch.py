"""Read-only observer. Does not launch the robot driver or send commands."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description = get_package_share_directory('onecube_robot_description')
    package = get_package_share_directory('motion_control_pkg')
    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value=os.path.join(
            description, 'urdf', 'onecube_robot_description.urdf')),
        DeclareLaunchArgument('config', default_value=os.path.join(
            package, 'config', 'wbc_model.yaml')),
        Node(package='motion_control_pkg', executable='wbc_model_node',
             name='wbc_model', output='screen', parameters=[
                 LaunchConfiguration('config'),
                 {'urdf_path': LaunchConfiguration('urdf_path')}]),
    ])
