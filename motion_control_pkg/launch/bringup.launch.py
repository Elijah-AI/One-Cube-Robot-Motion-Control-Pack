import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('onecube_robot_description').find('onecube_robot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'onecube_robot_description.urdf')
    mesh_dir = os.path.dirname(pkg_share)   #父级目录
    
    # driver_package_dir = get_package_share_directory('miracle_driver')
    # param_file = "/opt/tq/motion_control/config/motion_control_config.yaml"
    motion_control_pkg = FindPackageShare('motion_control_pkg').find('motion_control_pkg')
    param_file = os.path.join(motion_control_pkg, 'launch', 'config', 'motion_control_config.yaml')
    active_arm_configuration = LaunchConfiguration('active_arm_configuration')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'active_arm_configuration',
                default_value='dual_arm',
                description=(
                    'Both Marvin arm channels are enabled. left_only/right_only '
                    'modes are no longer supported.'
                ),
            ),
            Node(
                package='motion_control_pkg',
                namespace="motion_control_ns",           #节点和参数的命名空间匹配，代码中获取时，无需加命名空间
                executable='motion_control_node',
                name='motion_control',
                output='screen',
                parameters=[
                    param_file,
                    {
                        'urdf_path': urdf_file,
                        'mesh_dir': mesh_dir,
                        'active_arm_configuration': active_arm_configuration,
                    }
                ],
            )
            # Node(
            #     package='miracle_driver',
            #     executable='joint_state_merger',
            #     name='joint_state_merger',
            #     output='screen',
            # ),

            #TF
            # Node(
            #     package='robot_state_publisher',
            #     executable='robot_state_publisher',
            #     name='robot_state_publisher',
            #     output='screen',
            #     parameters=[{
            #         'publish_frequency': 50.0, 
            #         'robot_description': open(urdf_file).read()}],
            # ),

            # Node(
            #     package='miracle_motion_middleware',
            #     executable='miracle_hardware_interface',
            #     name='miracle_hardware_interface',
            #     output='screen',
            # )
        ]
    )
