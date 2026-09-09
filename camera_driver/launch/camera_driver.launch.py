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
            default_value='3',
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
            default_value='3',
            description='Depth stream frame rate'
        ),
        DeclareLaunchArgument(
            'serial_number',
            default_value='',
            description='Camera serial number (empty = auto-select first device)'
        ),
        DeclareLaunchArgument(
            'camera_id_sn_map',
            default_value='["CPC8563000CJ","CPC85630003B"]',
            description=(
                'Multi-camera SN -> camera_id mapping (string array, index = camera_id): '
                'camera_id 0=head (CPC8563000PW), 1=waist (CPC8563000X6); '
                'when set, all listed cameras are opened individually.'
            )
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
        DeclareLaunchArgument(
            'enable_align',
            default_value='true',
            description=(
                'Enable D2C alignment (depth -> color). '
                'With align_mode=HW the pipeline performs hardware D2C; with align_mode=SW '
                'the in-node reprojection is used. Both aligned paths publish camera_rgbd_pointcloud '
                'in the color optical frame, pixel-aligned with the color image.'
            )
        ),
        DeclareLaunchArgument(
            'align_mode',
            default_value='SW',
            description=(
                'Image registration mode: OFF | HW (pipeline hardware D2C) | SW (in-node software D2C '
                'reprojection). In HW/SW modes camera_rgbd_pointcloud is reconstructed with the color '
                'intrinsics and is pixel-aligned with the color image. '
                'Default HW matches orbbec_camera (e.g. gemini_330_series.launch.py) and is the '
                'recommended mode for Gemini 330 series devices.'
            )
        ),
        DeclareLaunchArgument(
            'align_target_stream',
            default_value='COLOR',
            description='Alignment target stream: COLOR (D2C, depth -> color) | DEPTH (C2D, color -> depth)'
        ),
        DeclareLaunchArgument(
            'pc_keep_invalid_points',
            default_value='true',
            description=(
                'Keep all point cloud pixels: invalid depth points are represented as NaN '
                '(point cloud width = image resolution). Set to false to filter invalid points out.'
            )
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
                'camera_id_sn_map': LaunchConfiguration('camera_id_sn_map'),
                'frame_id': LaunchConfiguration('frame_id'),
                'color_frame_id': LaunchConfiguration('color_frame_id'),
                'depth_frame_id': LaunchConfiguration('depth_frame_id'),
                'bypass_lock': LaunchConfiguration('bypass_lock'),
                'enable_align': LaunchConfiguration('enable_align'),
                'align_mode': LaunchConfiguration('align_mode'),
                'align_target_stream': LaunchConfiguration('align_target_stream'),
                'pc_keep_invalid_points': LaunchConfiguration('pc_keep_invalid_points'),
            }],
        ),
    ])
