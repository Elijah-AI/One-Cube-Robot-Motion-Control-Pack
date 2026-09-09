import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    default_config_path = os.path.join(
        get_package_share_directory('task_manager'),
        'config',
        'task_config.yaml'
    )
    
    config_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Path to tasks configuration file'
    )
    
    task_manager_node = Node(
        package='task_manager',
        executable='task_manager_node',
        name='task_manager_node',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path')
        }]
    )
    
    return LaunchDescription([
        config_arg,
        task_manager_node
    ])