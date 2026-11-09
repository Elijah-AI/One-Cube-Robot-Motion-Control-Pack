"""Publish the URDF TF tree from measured joint feedback, without drivers."""
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def description_with_joint_frames(description):
    """Alias each joint name to its child link (the frame after joint motion).

    Only the TF publisher's in-memory description is extended. Planning geometry
    and the original URDF stay authoritative and unchanged.
    """
    robot = ET.fromstring(description)
    links = {link.attrib['name'] for link in robot.findall('link')}
    joints = list(robot.findall('joint'))
    joint_names = {joint.attrib['name'] for joint in joints}
    for joint in joints:
        name = joint.attrib['name']
        child = joint.find('child').attrib['link']
        if name in links:
            if name == child:
                continue
            raise ValueError(f'Joint TF alias {name!r} conflicts with an existing link')
        alias_joint = '__joint_tf_alias_' + name
        if alias_joint in joint_names:
            raise ValueError(f'Joint TF alias joint {alias_joint!r} already exists')
        ET.SubElement(robot, 'link', name=name)
        alias = ET.SubElement(robot, 'joint', name=alias_joint, type='fixed')
        ET.SubElement(alias, 'parent', link=child)
        ET.SubElement(alias, 'child', link=name)
        ET.SubElement(alias, 'origin', xyz='0 0 0', rpy='0 0 0')
        links.add(name)
        joint_names.add(alias_joint)
    return ET.tostring(robot, encoding='unicode')


def _setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    with open(get('urdf_path'), encoding='utf-8') as stream:
        description = stream.read()
    if get('publish_joint_frames').lower() == 'true':
        description = description_with_joint_frames(description)
    frequency = float(get('publish_frequency'))
    if not math.isfinite(frequency) or not 0 < frequency <= 1000:
        raise ValueError('publish_frequency must be finite and in (0, 1000] Hz')
    target, source = get('target_frame'), get('source_frame')
    if bool(target) != bool(source):
        raise ValueError('Set both target_frame and source_frame, or leave both empty')
    nodes = [Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='motion_control_state_publisher', namespace='motion_control_ns',
        output='screen', parameters=[{
            'robot_description': ParameterValue(description, value_type=str),
            'publish_frequency': frequency,
            'frame_prefix': ParameterValue(get('frame_prefix'), value_type=str),
            'ignore_timestamp': False,
        }], remappings=[('joint_states', get('joint_state_topic'))],
    )]
    if target:
        nodes.append(Node(
            package='motion_control_pkg', executable='joint_tf_pair',
            name='joint_tf_pair', namespace='motion_control_ns', output='screen',
            parameters=[{
                'target_frame': ParameterValue(target, value_type=str),
                'source_frame': ParameterValue(source, value_type=str),
                'publish_frequency': ParameterValue(
                    LaunchConfiguration('pair_publish_frequency'), value_type=float),
                'max_age_sec': ParameterValue(
                    LaunchConfiguration('pair_max_age_sec'), value_type=float),
            }], remappings=[('/Control/joint_transform', get('transform_topic'))],
        ))
    return nodes


def generate_launch_description():
    model = os.path.join(get_package_share_directory('onecube_robot_description'),
                         'urdf', 'onecube_robot_description.urdf')
    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value=model),
        DeclareLaunchArgument('joint_state_topic', default_value='/wbc/joint_states',
                              description='Measured JointState input; never filled with zeros'),
        DeclareLaunchArgument('publish_frequency', default_value='50.0',
                              description='Maximum dynamic TF publication frequency in Hz'),
        DeclareLaunchArgument('publish_joint_frames', default_value='true',
                              choices=['true', 'false'],
                              description='Add joint-name aliases for the moving child-link frames'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument('target_frame', default_value='',
                              description='Optional reference frame for the pair output'),
        DeclareLaunchArgument('source_frame', default_value='',
                              description='Optional frame expressed in target_frame'),
        DeclareLaunchArgument('transform_topic', default_value='/Control/joint_transform'),
        DeclareLaunchArgument('pair_publish_frequency', default_value='20.0'),
        DeclareLaunchArgument('pair_max_age_sec', default_value='0.15'),
        OpaqueFunction(function=_setup),
    ])
