from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_name = LaunchConfiguration('topic_name')
    action_name = LaunchConfiguration('action_name')
    output_dir = LaunchConfiguration('output_dir')

    return LaunchDescription([
        DeclareLaunchArgument('topic_name', default_value='/fusion/synced_robot_vision'),
        DeclareLaunchArgument('action_name', default_value='/vision/calibration_control'),
        DeclareLaunchArgument('output_dir', default_value=''),
        Node(
            package='vision_calibration',
            executable='calibration_node',
            name='calibration_node',
            output='screen',
            parameters=[
                {'topic_name': topic_name},
                {'action_name': action_name},
                {'output_dir': output_dir},
                {'board_cols': 11},
                {'board_rows': 8},
                {'square_size': 0.015},
                {'ransac_threshold': 0.002},
                {'ransac_sample_size': 8},
                {'ransac_n_iter': 5000},
                {'rotation_weight': 0.2},
                {'refine_iterations': 30},
            ],
        ),
    ])
