# Launch file for the Berxel Hawk camera_driver_node.
#
# 本节点与 camera_driver (Orbbec) 功能完全一致，仅底层图像数据来源不同
# （Berxel Hawk SDK 替代 Orbbec SDK）。因此本 launch 与
# camera_driver/launch/camera_driver.launch.py 提供完全一致的参数名与默认值，
# 便于两套驱动无缝替换（例如把上层系统里对 camera_driver.launch.py 的启动
# 改为本文件即可，参数命令不变）。
#
# Usage:
#   # 默认参数（color/depth 1280x800 @ 5fps；iHawk100RS 的 color 仅支持 1280x800@5..45，
#   # 节点启动时若请求模式不被支持会按 SDK 列表自动校正为最近可用值。与
#   # config/berxel_camera_driver.yaml 一致）
#   ros2 launch berxel_camera_driver berxel_camera_driver.launch.py
#
#   # 用与 camera_driver 相同的参数覆盖分辨率 / 帧率 / 设备 / 对齐模式等
#   ros2 launch berxel_camera_driver berxel_camera_driver.launch.py \
#       color_width:=640 color_height:=480 color_fps:=30 \
#       depth_width:=640 depth_height:=480 depth_fps:=30 \
#       align_mode:=OFF serial_number:=<serial>
#
#   # 或直接加载整份 ROS 参数 yaml（此时上面的 inline 参数被忽略）
#   ros2 launch berxel_camera_driver berxel_camera_driver.launch.py \
#       config_file:=/abs/path/to/berxel_camera_driver.yaml
#
# 包内自带模板参数文件：config/berxel_camera_driver.yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_node(context):
    """按 camera_driver/launch/camera_driver.launch.py 构建 inline 参数节点。

    除与 camera_driver 相同的参数外，再补充本节点额外支持的参数
    （pointcloud_format / pointcloud_base_frame_id / enable_host_time_sync /
    time_sync_period），这些在 camera_driver launch 中不暴露、使用其节点默认值。

    camera_id_sn_map 允许为空（= 不指定 SN，按枚举顺序自动打开第一台）。launch_ros
    会把该参数字符串按 yaml 解析成数组，而空数组 '[]' 会被 evaluate_parameters
    判为非法（“不允许空序列”），因此为空时不再显式下发该参数，让节点使用内建默认
    std::vector<std::string>{} —— 语义与空数组完全一致。
    """
    parameters = {
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
        'enable_align': LaunchConfiguration('enable_align'),
        'align_mode': LaunchConfiguration('align_mode'),
        'align_target_stream': LaunchConfiguration('align_target_stream'),
        'pointcloud_format': LaunchConfiguration('pointcloud_format'),
        'pc_keep_invalid_points': LaunchConfiguration('pc_keep_invalid_points'),
        'pointcloud_base_frame_id': LaunchConfiguration('pointcloud_base_frame_id'),
        'enable_host_time_sync': LaunchConfiguration('enable_host_time_sync'),
        'time_sync_period': LaunchConfiguration('time_sync_period'),
    }
    camera_sn_map = context.perform_substitution(
        LaunchConfiguration('camera_id_sn_map')).strip()
    if camera_sn_map and camera_sn_map not in ('[]', '{}'):
        # 仅当用户提供了非空 SN 数组时才下发，交由 launch_ros 解析为 string array
        parameters['camera_id_sn_map'] = LaunchConfiguration('camera_id_sn_map')
    return Node(
        package='berxel_camera_driver',
        executable='camera_driver_node',
        name='camera_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameters],
    )


def _launch_node(context):
    """运行时决定用 inline 参数还是 config 文件。

    若显式提供了 config_file（非空）则整份加载该 ROS 参数 yaml；
    否则使用与 camera_driver launch 相同的 inline 参数（可逐项覆盖）。
    """
    config_file = context.perform_substitution(LaunchConfiguration('config_file'))
    if config_file and config_file.strip():
        return [Node(
            package='berxel_camera_driver',
            executable='camera_driver_node',
            name='camera_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[config_file],
        )]
    return [_create_node(context)]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('color_width', default_value='1280',
                              description='Color image width (Berxel iHawk100RS: 1280x800@5..45)'),
        DeclareLaunchArgument('color_height', default_value='800',
                              description='Color image height (Berxel iHawk100RS: 1280x800@5..45)'),
        DeclareLaunchArgument('color_fps', default_value='5',
                              description='Color stream frame rate (unsupported values are auto-corrected)'),
        DeclareLaunchArgument('depth_width', default_value='1280',
                              description='Depth image width'),
        DeclareLaunchArgument('depth_height', default_value='800',
                              description='Depth image height'),
        DeclareLaunchArgument('depth_fps', default_value='5',
                              description='Depth stream frame rate (unsupported values are auto-corrected)'),
        DeclareLaunchArgument('serial_number', default_value='',
                              description='Camera serial number (empty = auto-select first device)'),
        DeclareLaunchArgument(
            'camera_id_sn_map',
            default_value='[]',
            description=(
                'Multi-camera SN -> camera_id mapping (string array, index = camera_id): '
                'camera_id 0=head, 1=waist, ... When set, the listed cameras are opened '
                'individually by SN. When empty, device enumeration order is used.'
            ),
        ),
        DeclareLaunchArgument('frame_id', default_value='camera_link',
                              description='Frame ID for the SyncedRobotVision header'),
        DeclareLaunchArgument('color_frame_id', default_value='camera_color_optical_frame',
                              description='Frame ID for color camera'),
        DeclareLaunchArgument('depth_frame_id', default_value='camera_depth_optical_frame',
                              description='Frame ID for depth camera'),
        DeclareLaunchArgument('bypass_lock', default_value='false',
                              description='Bypass cross-process device lock'),
        DeclareLaunchArgument(
            'enable_align',
            default_value='true',
            description=(
                'Enable D2C alignment (depth -> color). The Berxel SDK performs hardware '
                'registration; kept for backward compatibility (align_mode takes precedence).'
            ),
        ),
        DeclareLaunchArgument(
            'align_mode',
            default_value='HW',
            description=(
                'Image registration mode: OFF | HW | SW. The Berxel SDK only performs '
                'depth->color alignment in hardware (setRegistrationEnable), so both HW and '
                'SW map to hardware registration; OFF disables it.'
            ),
        ),
        DeclareLaunchArgument(
            'align_target_stream',
            default_value='COLOR',
            description='Alignment target stream: COLOR (D2C; Berxel only supports D2C)',
        ),
        DeclareLaunchArgument(
            'pointcloud_format',
            default_value='rgb_point',
            description='Point cloud output: rgb_point (xyz float32 + rgb uint32) | point (xyz only)',
        ),
        DeclareLaunchArgument(
            'pc_keep_invalid_points',
            default_value='true',
            description=(
                'Keep all point cloud pixels: invalid depth points are represented as NaN '
                '(point cloud width = image width). Set to false to filter invalid points out.'
            ),
        ),
        DeclareLaunchArgument('pointcloud_base_frame_id', default_value='base_link',
                              description='Frame_id of the robot-base (transformed) point cloud'),
        DeclareLaunchArgument(
            'enable_host_time_sync',
            default_value='true',
            description='Sync the camera internal clock to the host system time (setSystemClock)'),
        DeclareLaunchArgument('time_sync_period', default_value='60.0',
                              description='Host time re-sync period in seconds'),
        # config_file: 提供后整份加载该 ROS 参数 yaml，否则使用上面的 inline 参数
        DeclareLaunchArgument('config_file', default_value='',
                              description='Path to a ROS parameters yaml file (optional)'),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_launch_node)])
