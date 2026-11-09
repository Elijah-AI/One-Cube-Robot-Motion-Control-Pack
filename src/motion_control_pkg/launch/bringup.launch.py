import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('onecube_robot_description').find('onecube_robot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'onecube_robot_description.urdf')
    mesh_dir = os.path.dirname(pkg_share)   #父级目录
    
    # driver_package_dir = get_package_share_directory('miracle_driver')
    
    motion_control_pkg = FindPackageShare('motion_control_pkg').find('motion_control_pkg')
    head_home_file = os.path.join(motion_control_pkg, 'config', 'head_startup_home.yaml')
    # param_file = os.path.join(motion_control_pkg, 'launch', 'config', 'motion_control_config.yaml')
    param_file = "/opt/onecube/motion_control/config/motion_control_config.yaml"

    driver_config = LaunchConfiguration('driver_config')
    
    
    active_arm_configuration = LaunchConfiguration('active_arm_configuration')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'publish_robot_tf', default_value='true', choices=['true', 'false'],
                description='Publish the measured robot TF tree and joint-name frames',
            ),
            DeclareLaunchArgument(
                'planning_worker_count',
                default_value='2',
                choices=['1', '2', '4'],
                description='Planning workers: 1 serial, 2/4 parallel validation; dual-arm IK uses two',
            ),
            DeclareLaunchArgument(
                'driver_config',
                default_value=os.environ.get(
                    'DRIVER_SDK_CONFIG',
                    '/opt/onecube/driver_sdk/configuration.xml'),
                description='Driver SDK XML configuration file',
            ),
            DeclareLaunchArgument(
                'active_arm_configuration',
                default_value='dual_arm',
                description=(
                    'Both Marvin arm channels are enabled. left_only/right_only '
                    'modes are no longer supported.'
                ),
            ),
            DeclareLaunchArgument(
                'collision_pointcloud_topic',
                default_value='/fusion/synced_robot_vision',
                description='SyncedRobotVision topic used only by collision mode 2',
            ),
            Node(
                package='motion_control_pkg',
                namespace="motion_control_ns",           #节点和参数的命名空间匹配，代码中获取时，无需加命名空间
                executable='motion_control_node',
                name='motion_control',
                output='screen',
                # Infinite grace: a failed measured shutdown must retain PDO and
                # feedback, never be converted to SIGKILL by launch's 5+5 s default.
                # 1e309 parses as +inf in both launch's typed float and expression.
                sigterm_timeout='1e309',
                sigkill_timeout='1e309',
                parameters=[
                    head_home_file,
                    param_file,
                    {
                        'urdf_path': urdf_file, 
                        'mesh_dir': mesh_dir, 
                        'EtherCAT_bus_config': driver_config,
                        'active_arm_configuration': active_arm_configuration,
                        'planning_worker_count': ParameterValue(
                            LaunchConfiguration('planning_worker_count'), value_type=int),
                        'collision_pointcloud_topic': LaunchConfiguration('collision_pointcloud_topic'),
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    motion_control_pkg, 'launch', 'robot_tf.launch.py')),
                condition=IfCondition(LaunchConfiguration('publish_robot_tf')),
                launch_arguments={'urdf_path': urdf_file}.items(),
            ),
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
