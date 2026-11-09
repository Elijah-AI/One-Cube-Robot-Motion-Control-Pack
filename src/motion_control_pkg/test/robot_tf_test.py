#!/usr/bin/python3
"""Hardware-free integration test of real RSP, TF2, aliases and pair output.

Runs on a private localhost DDS domain. It never launches motion_control_node.
"""
import importlib.util
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET

# Set DDS isolation before importing RMW bindings as well as before spawning ROS.
os.environ['ROS_DOMAIN_ID'] = os.environ.get('ROBOT_TF_TEST_DOMAIN_ID', '187')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.pop('ROS_DISCOVERY_SERVER', None)

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


PACKAGE = Path(sys.argv[1]).resolve()
sys.argv = [sys.argv[0]]
MODEL = PACKAGE.parent / 'onecube_robot_description/urdf/onecube_robot_description.urdf'
USE_INSTALLED = os.environ.get('ROBOT_TF_TEST_USE_INSTALLED') == '1'
if USE_INSTALLED:
    MODEL = Path(get_package_share_directory('onecube_robot_description')) / 'urdf/onecube_robot_description.urdf'
spec = importlib.util.spec_from_file_location('robot_tf_launch', PACKAGE / 'launch/robot_tf.launch.py')
launch_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_module)


def transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    out = np.eye(4)
    out[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                   [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                   [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    out[:3, 3] = [t.x, t.y, t.z]
    return out


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*cross + (1-math.cos(angle))*(cross@cross)


def independent_fk(positions):
    """Compose URDF origins and axes without RSP/KDL/TF2 or production FK."""
    root = ET.parse(MODEL).getroot()
    frames = {'base_link': np.eye(4)}
    remaining = list(root.findall('joint'))
    while remaining:
        advanced = []
        for joint in remaining:
            name = joint.get('name')
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent not in frames or (joint.get('type') != 'fixed' and name not in positions):
                continue
            origin = joint.find('origin')
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0]*3
            r, p, y = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0]*3
            local = np.eye(4)
            local[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
            local[:3, 3] = xyz
            if joint.get('type') in ('revolute', 'continuous'):
                axis = [float(v) for v in joint.find('axis').get('xyz').split()]
                local[:3, :3] = local[:3, :3] @ rotation(axis, positions[name])
            elif joint.get('type') == 'prismatic':
                axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
                local[:3, 3] += local[:3, :3] @ (axis * positions[name])
            frames[child] = frames[parent] @ local
            frames[name] = frames[child]
            advanced.append(joint)
        if not advanced:
            break
        remaining = [j for j in remaining if j not in advanced]
    return frames


class RobotTFTest(unittest.TestCase):
    def test_tf_integration(self):
        original = MODEL.read_bytes()
        extended = launch_module.description_with_joint_frames(original.decode())
        root = ET.fromstring(extended)
        names = [link.get('name') for link in root.findall('link')]
        self.assertEqual(len(names), len(set(names)))
        self.assertIn('Joint7_L', names)
        self.assertEqual(MODEL.read_bytes(), original)
        conflict = '<robot name="bad"><link name="a"/><link name="b"/><joint name="a" type="fixed"><parent link="a"/><child link="b"/></joint></robot>'
        with self.assertRaises(ValueError):
            launch_module.description_with_joint_frames(conflict)

        with tempfile.TemporaryDirectory(prefix='robot_tf_test_') as directory:
            temp = Path(directory)
            prefix = temp / 'install'
            marker = prefix / 'share/ament_index/resource_index/packages/motion_control_pkg'
            marker.parent.mkdir(parents=True)
            marker.touch()
            (marker.parent / 'onecube_robot_description').touch()
            model_dir = prefix / 'share/onecube_robot_description/urdf'
            model_dir.mkdir(parents=True)
            shutil.copy2(MODEL, model_dir)
            launch_dir = prefix / 'share/motion_control_pkg/launch'
            launch_dir.mkdir(parents=True)
            shutil.copy2(PACKAGE / 'launch/robot_tf.launch.py', launch_dir)
            lib = prefix / 'lib/motion_control_pkg'
            lib.mkdir(parents=True)
            shutil.copy2(PACKAGE / 'scripts/joint_tf_pair', lib)
            env = dict(os.environ)
            if not USE_INSTALLED:
                env['AMENT_PREFIX_PATH'] = str(prefix) + ':' + env.get('AMENT_PREFIX_PATH', '')
            env['ROS_LOG_DIR'] = str(temp / 'ros_logs')
            log = (temp / 'launch.log').open('w+')
            command = ['ros2', 'launch', 'motion_control_pkg', 'robot_tf.launch.py',
                       'urdf_path:=' + str(MODEL), 'joint_state_topic:=/tf_test/joint_states',
                       'publish_frequency:=100.0', 'target_frame:=Joint3_L',
                       'source_frame:=head_pitch_joint', 'transform_topic:=/tf_test/pair',
                       'pair_max_age_sec:=0.15']
            process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            node = None
            try:
                rclpy.init(domain_id=int(os.environ['ROS_DOMAIN_ID']))
                node = Node('robot_tf_integration_test')
                buffer = Buffer(node=node)
                listener = TransformListener(buffer, node)
                joint_pub = node.create_publisher(JointState, '/tf_test/joint_states', qos_profile_sensor_data)
                pair_messages, dynamic_messages = [], []
                pair_sub = node.create_subscription(TransformStamped, '/tf_test/pair', pair_messages.append, 50)
                tf_sub = node.create_subscription(TFMessage, '/tf', dynamic_messages.append, 100)

                def spin_until(predicate, timeout=8., positions=None):
                    deadline = time.monotonic()+timeout
                    while time.monotonic() < deadline:
                        self.assertIsNone(process.poll(), 'TF launch exited early')
                        if positions is not None:
                            msg = JointState()
                            msg.header.stamp = node.get_clock().now().to_msg()
                            msg.name = list(reversed(positions))
                            msg.position = [positions[n] for n in msg.name]
                            joint_pub.publish(msg)
                        rclpy.spin_once(node, timeout_sec=.015)
                        if predicate():
                            return True
                    return False

                self.assertTrue(spin_until(lambda: buffer.can_transform('base_link', 'base_front_camera_link', Time())))
                self.assertFalse(buffer.can_transform('base_link', 'Joint7_L', Time()))
                self.assertFalse(pair_messages, 'Dynamic pair output fabricated without feedback')
                print('PASS static TF available before measurements; dynamic TF withheld')

                joints = ['Joint'+str(j)+'_'+side for side in ['L', 'R'] for j in range(1, 8)]
                joints += ['leg_joint1', 'leg_joint2', 'leg_joint3', 'body_joint', 'head_yaw_joint', 'head_pitch_joint']
                pairs = [('base_link', 'Joint7_L'), ('Joint2_L', 'Joint6_R'),
                         ('leg_joint2', 'head_pitch_joint'), ('Joint3_L', 'head_pitch_joint')]
                for k in range(3):
                    positions = {j:math.sin(i*.73+k)*.15 for i, j in enumerate(joints)}
                    expected = independent_fk(positions)
                    expected_pair = np.linalg.inv(expected['Joint3_L']) @ expected['head_pitch_joint']
                    before = len(pair_messages)
                    self.assertTrue(spin_until(lambda: len(pair_messages)>before and np.allclose(
                        transform_matrix(pair_messages[-1].transform), expected_pair,
                        rtol=0, atol=1e-9), positions=positions))
                    for target, source in pairs:
                        actual = buffer.lookup_transform(target, source, Time())
                        reverse = buffer.lookup_transform(source, target, Time())
                        np.testing.assert_allclose(transform_matrix(actual.transform),
                                                   np.linalg.inv(expected[target]) @ expected[source], rtol=0, atol=1e-9)
                        np.testing.assert_allclose(transform_matrix(actual.transform) @ transform_matrix(reverse.transform), np.eye(4), rtol=0, atol=1e-9)
                    alias = buffer.lookup_transform('Link7_L', 'Joint7_L', Time())
                    np.testing.assert_allclose(transform_matrix(alias.transform), np.eye(4), rtol=0, atol=1e-12)
                    self.assertEqual(pair_messages[-1].header.frame_id, 'Joint3_L')
                    self.assertEqual(pair_messages[-1].child_frame_id, 'head_pitch_joint')
                print('PASS nonzero measured joints, independent FK, cross-branch transforms and inverses')
                self.assertFalse(buffer.can_transform('base_link', 'base_front_wheel_1_yaw_joint', Time()))
                self.assertFalse(buffer.can_transform('base_link', 'gripper_l_joint1', Time()))
                print('PASS missing steering/gripper feedback not replaced with zeros')

                # Drain in-flight packets, then ensure no timer refreshes dynamic TF.
                spin_until(lambda: False, timeout=.45)
                pair_count, dynamic_count = len(pair_messages), len(dynamic_messages)
                old_stamp = buffer.lookup_transform('base_link', 'Joint7_L', Time()).header.stamp
                spin_until(lambda: False, timeout=.30)
                self.assertEqual(len(pair_messages), pair_count, 'Stale TF leaked onto pair output')
                self.assertEqual(len(dynamic_messages), dynamic_count, 'Dynamic TF renewed without measurements')
                self.assertEqual(buffer.lookup_transform('base_link', 'Joint7_L', Time()).header.stamp, old_stamp)
                print('PASS stale feedback stops pair output and preserves the last acquisition stamp')
                self.assertTrue(spin_until(lambda: len(pair_messages)>pair_count, positions=positions))
                print('PASS publication resumes when fresh feedback arrives')
            except Exception:
                if node is not None:
                    print('NODES', node.get_node_names_and_namespaces(), file=sys.stderr)
                    print('TOPICS', node.get_topic_names_and_types(), file=sys.stderr)
                    print('JOINT_SUBSCRIPTIONS', node.get_subscriptions_info_by_topic('/tf_test/joint_states'), file=sys.stderr)
                    print('PAIR_MESSAGES', len(pair_messages), 'TF_MESSAGES', len(dynamic_messages), file=sys.stderr)
                log.flush()
                print((temp / 'launch.log').read_text(), file=sys.stderr)
                raise
            finally:
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=3)
                log.close()


if __name__ == '__main__':
    unittest.main()
