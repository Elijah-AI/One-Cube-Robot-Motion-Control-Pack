#!/usr/bin/env python3
"""
订阅 SyncedRobotVision 话题，保存 camera_rgbd_color（Image→BMP）
和 camera_rgbd_pointcloud（PointCloud2→PLY）到指定目录。

用法:
  # 默认话题 /fusion/synced_robot_vision, 输出 /tmp/saved_vision
  python3 save_synced_vision.py

  # 指定话题和输出目录
  python3 save_synced_vision.py --topic /fusion/synced_robot_vision \
      --output-dir /home/wyq/test_ws/src/vision_calibration/data_cali/test

  # 只保存指定相机 (默认所有相机都存)
  python3 save_synced_vision.py --camera-id 0

  # 限制保存帧数后自动退出 (例如只存 10 帧)
  python3 save_synced_vision.py --max-frames 10

依赖: rclpy, sensor_msgs_py, cv_bridge, numpy, opencv
"""
import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, PointCloud2
from com_interfaces.msg import SyncedRobotVision
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2


def save_image(bridge: CvBridge, img: Image, path: str) -> bool:
    """将 sensor_msgs/Image 保存为 BMP"""
    import cv2
    try:
        cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        return cv2.imwrite(path, cv_img)
    except Exception as e:
        print(f"[ERROR] 保存图像失败 {path}: {e}")
        return False


def save_pointcloud(cloud: PointCloud2, path: str) -> bool:
    """
    将 sensor_msgs/PointCloud2 保存为 PLY (ascii) 格式。
    纯 numpy 实现，不依赖 open3d。
    自动判断是否为彩色点云：有 rgb/rgba 字段则保存彩色，否则只存 xyz。
    """
    try:
        # 读取所有字段（不指定 field_names，避免字段名不匹配报错）
        pts = list(point_cloud2.read_points(cloud, skip_nans=True))
        if len(pts) == 0:
            print(f"[WARN] 点云为空: {path}")
            return False
        pts = np.array(pts)
        names = pts.dtype.names  # 结构化数组字段名

        xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)

        # 动态查找颜色字段：rgb / rgba（float32 打包的 uint32）或 r/g/b 分量
        rgb_field = None
        for cand in ("rgb", "rgba"):
            if cand in names:
                rgb_field = cand
                break

        has_rgb = rgb_field is not None
        if has_rgb:
            # float32 字节重解释为 uint32（不能用 astype，会丢数据）
            rgb_uint = np.ascontiguousarray(pts[rgb_field]).view(np.uint32)
            r = ((rgb_uint >> 16) & 0xFF).astype(np.uint8)
            g = ((rgb_uint >> 8) & 0xFF).astype(np.uint8)
            b = (rgb_uint & 0xFF).astype(np.uint8)
            # 判断是否真的是彩色（rgb 不全为 0）
            is_color = bool(r.any() or g.any() or b.any())
        else:
            # 检查是否有分开的 r/g/b 字段
            if "r" in names and "g" in names and "b" in names:
                r = pts["r"].astype(np.uint8)
                g = pts["g"].astype(np.uint8)
                b = pts["b"].astype(np.uint8)
                has_rgb = True
                is_color = bool(r.any() or g.any() or b.any())
            else:
                is_color = False

        # 写 PLY (ascii)，用 np.savetxt 批量写入提升性能
        n = xyz.shape[0]
        header = "ply\nformat ascii 1.0\n" \
                 f"element vertex {n}\n" \
                 "property float x\n" \
                 "property float y\n" \
                 "property float z\n"
        if has_rgb and is_color:
            header += "property uchar red\n" \
                      "property uchar green\n" \
                      "property uchar blue\n"
            data = np.column_stack([xyz, r, g, b])
            fmt = ["%.6f", "%.6f", "%.6f", "%d", "%d", "%d"]
        else:
            data = xyz
            fmt = ["%.6f", "%.6f", "%.6f"]
            # 如果有 rgb 字段但全为 0，不写颜色
            if has_rgb and not is_color:
                print(f"[INFO] 点云有 rgb 字段但颜色全为 0, 保存为无颜色 PLY")

        np.savetxt(path, data, fmt=fmt, header=header + "end_header\n", comments="")
        return True
    except Exception as e:
        print(f"[ERROR] 保存点云失败 {path}: {e}")
        return False


class VisionSaver(Node):
    def __init__(self, topic: str, output_dir: str,
                 camera_id: int = -1, max_frames: int = 0,
                 skip_invalid: bool = False, save_cloud: bool = True):
        super().__init__("vision_saver")
        self.bridge = CvBridge()
        self.output_dir = output_dir
        self.camera_id = camera_id
        self.max_frames = max_frames
        self.skip_invalid = skip_invalid
        self.save_cloud = save_cloud
        self.count = 0

        os.makedirs(output_dir, exist_ok=True)

        # 用 SensorDataQoS 匹配传感器数据
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(
            SyncedRobotVision, topic, self.cb, qos)
        self.get_logger().info(
            f"订阅 {topic}, 输出目录 {output_dir}, "
            f"camera_id 过滤={'所有' if camera_id<0 else camera_id}, "
            f"max_frames={max_frames if max_frames>0 else '不限'}, "
            f"保存点云={'是' if save_cloud else '否'}")

    def cb(self, msg: SyncedRobotVision):
        # camera_id 过滤
        if self.camera_id >= 0 and msg.camera_id != self.camera_id:
            return
        # data_valid 过滤：仅在 --skip-invalid 时跳过无效帧
        # if self.skip_invalid and not msg.data_valid:
        #     return

        self.count += 1
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            now = time.time()
            stamp_sec, stamp_nsec = int(now), int((now % 1) * 1e9)
        else:
            stamp_sec, stamp_nsec = stamp.sec, stamp.nanosec
        ts = f"{stamp_sec}_{stamp_nsec:09d}"
        img_path = os.path.join(self.output_dir, f"image_{ts}.bmp")
        pc_path = os.path.join(self.output_dir, f"cloud_{ts}.ply")
        if os.path.exists(img_path):
            ts = f"{ts}_{self.count:04d}"
            img_path = os.path.join(self.output_dir, f"image_{ts}.bmp")
            pc_path = os.path.join(self.output_dir, f"cloud_{ts}.ply")

        t0 = time.time()
        ok_img = save_image(self.bridge, msg.camera_rgbd_color, img_path)
        ok_pc = save_pointcloud(msg.camera_rgbd_pointcloud, pc_path) \
            if self.save_cloud else None
        dt = time.time() - t0

        # 状态判断：不保存点云时只看图像
        if self.save_cloud:
            status = "OK" if (ok_img and ok_pc) else "FAIL"
            pc_str = f"✓" if ok_pc else f"✗"
            pc_file = f", {os.path.basename(pc_path)}"
        else:
            status = "OK" if ok_img else "FAIL"
            pc_str = "-"
            pc_file = ""
        self.get_logger().info(
            f"[{status}] #{self.count} cam={msg.camera_id} "
            f"img={'✓' if ok_img else '✗'} pc={pc_str} "
            f"({dt:.2f}s) -> {os.path.basename(img_path)}{pc_file}")

        if self.max_frames > 0 and self.count >= self.max_frames:
            self.get_logger().info(f"已达 max_frames={self.max_frames}, 退出")
            raise SystemExit(0)


def main():
    parser = argparse.ArgumentParser(
        description="保存 SyncedRobotVision 的图像和点云")
    parser.add_argument("--topic", default="/fusion/synced_robot_vision",
                        help="话题名 (默认 /fusion/synced_robot_vision)")
    parser.add_argument("--output-dir", default="/home/neardi/ros2_ws/src/vision_calibration/test/save",
                        help="输出目录 (默认 /tmp/saved_vision)")
    parser.add_argument("--camera-id", type=int, default=0,
                        help="只保存指定相机 (-1=所有, 0=头部, 1=腰部)")
    parser.add_argument("--max-frames", type=int, default=2,
                        help="保存帧数上限 (0=不限, Ctrl+C 退出)")
    parser.add_argument("--skip-invalid", action="store_true", default=False,
                        help="跳过 data_valid=false 的帧 (默认不跳过, 全部保存)")
    parser.add_argument("--no-cloud", action="store_true", default=False,
                        help="不保存点云, 只保存图像 (默认同时保存图像和点云)")
    args = parser.parse_args()

    rclpy.init()
    node = VisionSaver(
        topic=args.topic,
        output_dir=args.output_dir,
        camera_id=args.camera_id,
        max_frames=args.max_frames,
        skip_invalid=args.skip_invalid,
        save_cloud=not args.no_cloud,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.get_logger().info(f"共保存 {node.count} 帧, 输出目录 {node.output_dir}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
# 同时保存图像和点云（默认）
# python3 save_synced_vision.py --max-frames 5

# 只保存图像，不保存点云
# python3 ~/ros2_ws/src/vision_calibration/test/save_synced_vision.py --no-cloud --max-frames 5