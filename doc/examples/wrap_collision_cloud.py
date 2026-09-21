#!/usr/bin/env python3
"""Wrap a live PointCloud2 in SyncedRobotVision; no TF or motion commands.

Use a dedicated output topic and select it with collision_pointcloud_topic.
The input must already contain correctly framed environment obstacle points.
"""
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from com_interfaces.msg import SyncedRobotVision


def wrap_cloud(cloud):
    message = SyncedRobotVision()
    message.header = cloud.header
    message.camera_rgbd_pointcloud = cloud
    message.data_valid = True
    return message


class CloudWrapper(Node):
    def __init__(self, input_topic, output_topic):
        super().__init__('collision_cloud_wrapper')
        self.publisher = self.create_publisher(
            SyncedRobotVision, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info(f'Wrapping {input_topic} -> {output_topic}; no frame transform')

    def on_cloud(self, cloud):
        # Preserve the acquisition timestamp and actual coordinate frame.
        # The motion server performs detailed XYZ/layout/model-frame validation.
        if not cloud.data or not cloud.header.frame_id:
            return
        self.publisher.publish(wrap_cloud(cloud))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, help='sensor_msgs/msg/PointCloud2 topic')
    parser.add_argument('--output', default='/collision/synced_robot_vision')
    args = parser.parse_args()
    if args.input == args.output:
        parser.error('input and output topics must differ (their message types differ)')
    rclpy.init(args=[])
    node = CloudWrapper(args.input, args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
