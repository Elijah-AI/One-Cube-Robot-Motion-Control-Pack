from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_detection_pose_cloud',
            executable='detection_pose_cloud_server',
            name='detection_pose_cloud_server',
            output='screen',
        ),
    ])
