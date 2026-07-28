import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'color_width',
            default_value='1280',
            description='Color image width'
        ),
        DeclareLaunchArgument(
            'color_height',
            default_value='720',
            description='Color image height'
        ),
        DeclareLaunchArgument(
            'color_fps',
            default_value='30',
            description='Color stream frame rate'
        ),
        DeclareLaunchArgument(
            'depth_width',
            default_value='1280',
            description='Depth image width'
        ),
        DeclareLaunchArgument(
            'depth_height',
            default_value='720',
            description='Depth image height'
        ),
        DeclareLaunchArgument(
            'depth_fps',
            default_value='30',
            description='Depth stream frame rate'
        ),
        DeclareLaunchArgument(
            'serial_number',
            default_value='',
            description='Camera serial number (empty = auto-select first device)'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='camera_link',
            description='Frame ID for the RGBD message header'
        ),
        DeclareLaunchArgument(
            'color_frame_id',
            default_value='camera_color_optical_frame',
            description='Frame ID for color camera'
        ),
        DeclareLaunchArgument(
            'depth_frame_id',
            default_value='camera_depth_optical_frame',
            description='Frame ID for depth camera'
        ),
        DeclareLaunchArgument(
            'bypass_lock',
            default_value='false',
            description='Bypass cross-process device lock'
        ),
        Node(
            package='camera_driver',
            executable='camera_driver_node',
            name='camera_driver_node',
            output='screen',
            parameters=[{
                'color_width': LaunchConfiguration('color_width'),
                'color_height': LaunchConfiguration('color_height'),
                'color_fps': LaunchConfiguration('color_fps'),
                'depth_width': LaunchConfiguration('depth_width'),
                'depth_height': LaunchConfiguration('depth_height'),
                'depth_fps': LaunchConfiguration('depth_fps'),
                'serial_number': LaunchConfiguration('serial_number'),
                'frame_id': LaunchConfiguration('frame_id'),
                'color_frame_id': LaunchConfiguration('color_frame_id'),
                'depth_frame_id': LaunchConfiguration('depth_frame_id'),
                'bypass_lock': LaunchConfiguration('bypass_lock'),
            }],
        ),
    ])
