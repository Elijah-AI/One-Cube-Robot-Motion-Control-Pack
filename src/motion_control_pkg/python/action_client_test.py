import argparse
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2

from com_interfaces.action import MoveJoints, MovePose


def _parse_list(arg: Optional[str]) -> List[str]:
    if not arg:
        return []
    return [x for x in (s.strip() for s in arg.split(",")) if x]


def _parse_float_list(arg: Optional[str]) -> List[float]:
    if not arg:
        return []
    return [float(x) for x in _parse_list(arg)]


def _clamp_speed(v: float) -> float:
    return max(0.0, min(1.0, float(v)))


class ActionTestClient(Node):
    def __init__(self) -> None:
        super().__init__("motion_control_action_test_client")
        self._move_pose_client = ActionClient(self, MovePose, "/Control/move_pose")
        self._move_joints_client = ActionClient(self, MoveJoints, "/Control/move_joints")
        self._cancel_timer = None

    def wait_servers(self, timeout_sec: float) -> bool:
        ok_pose = self._move_pose_client.wait_for_server(timeout_sec=timeout_sec)
        ok_joints = self._move_joints_client.wait_for_server(timeout_sec=timeout_sec)
        if not ok_pose:
            self.get_logger().error("MovePose action server not available")
        if not ok_joints:
            self.get_logger().error("MoveJoints action server not available")
        return ok_pose and ok_joints

    def send_move_pose(
        self,
        task_id: str,
        arm_id: int,
        frame_id: str,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        enable_collision_check: bool,
        speed: float,
        use_force_control: bool,
        bound_size: List[float],
        hand_action_type: List[int],
        point_cloud_frame_id: str,
        timeout_ms: int,
        cancel_after_sec: float,
    ) -> None:
        goal = MovePose.Goal()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = frame_id
        goal.task_id = task_id
        goal.arm_id = int(arm_id)
        goal.enable_collision_check = bool(enable_collision_check)
        goal.speed = _clamp_speed(speed)
        goal.use_force_control = bool(use_force_control)
        goal.timeout_ms = int(timeout_ms)

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = frame_id
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.x = float(qx)
        ps.pose.orientation.y = float(qy)
        ps.pose.orientation.z = float(qz)
        ps.pose.orientation.w = float(qw)
        goal.target_pose = ps

        goal.bound_size = [float(v) for v in bound_size]
        goal.hand_action_type = [int(v) for v in hand_action_type]
        goal.env_point_cloud = PointCloud2()
        goal.point_cloud_frame_id = str(point_cloud_frame_id)

        def feedback_cb(msg) -> None:
            fb = getattr(msg, "feedback", msg)
            self.get_logger().info(
                f"MovePose feedback: progress={fb.progress_rate:.1f} step='{fb.current_step_info}' status='{fb.task_status}'"
            )

        send_future = self._move_pose_client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error("MovePose send goal failed")
            return
        if not goal_handle.accepted:
            self.get_logger().warn("MovePose goal rejected")
            return

        if cancel_after_sec and cancel_after_sec > 0.0:
            if self._cancel_timer is not None:
                self._cancel_timer.cancel()
                self._cancel_timer = None

            def do_cancel() -> None:
                if self._cancel_timer is not None:
                    self._cancel_timer.cancel()
                    self._cancel_timer = None
                self.get_logger().warn(f"Canceling MovePose goal after {cancel_after_sec:.3f}s")
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda f: self.get_logger().info(f"Cancel response: {f.result()}"))

            self._cancel_timer = self.create_timer(float(cancel_after_sec), do_cancel)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"MovePose result: success={result.success} error='{result.error_message}' exec_time={result.execution_time:.3f} final_err={result.final_distance_error:.6f}"
        )

    def send_move_joints(
        self,
        task_id: str,
        arm_id: int,
        joint_name: List[str],
        position: List[float],
        enable_collision_check: bool,
        speed: float,
        timeout_ms: int,
        cancel_after_sec: float,
    ) -> None:
        if len(joint_name) != len(position):
            self.get_logger().error("joint_name length must equal position length")
            return

        goal = MoveJoints.Goal()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = ""
        goal.task_id = task_id
        goal.arm_id = int(arm_id)
        goal.joint_name = list(joint_name)
        goal.position = [float(v) for v in position]
        goal.enable_collision_check = bool(enable_collision_check)
        goal.speed = _clamp_speed(speed)
        goal.timeout_ms = int(timeout_ms)

        def feedback_cb(msg) -> None:
            fb = getattr(msg, "feedback", msg)
            self.get_logger().info(
                f"MoveJoints feedback: progress={fb.progress_rate:.1f} step='{fb.current_step_info}' status='{fb.task_status}'"
            )

        send_future = self._move_joints_client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error("MoveJoints send goal failed")
            return
        if not goal_handle.accepted:
            self.get_logger().warn("MoveJoints goal rejected")
            return

        if cancel_after_sec and cancel_after_sec > 0.0:
            if self._cancel_timer is not None:
                self._cancel_timer.cancel()
                self._cancel_timer = None

            def do_cancel() -> None:
                if self._cancel_timer is not None:
                    self._cancel_timer.cancel()
                    self._cancel_timer = None
                self.get_logger().warn(f"Canceling MoveJoints goal after {cancel_after_sec:.3f}s")
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda f: self.get_logger().info(f"Cancel response: {f.result()}"))

            self._cancel_timer = self.create_timer(float(cancel_after_sec), do_cancel)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"MoveJoints result: success={result.success} error='{result.error_message}' exec_time={result.execution_time:.3f} final_err={result.final_distance_error:.6f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(prog="action_client_test.py")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_pose = sub.add_parser("move_pose")
    p_pose.add_argument("--task-id", default="test_move_pose")
    p_pose.add_argument("--arm-id", type=int, required=True)
    p_pose.add_argument("--frame-id", default="base_link")
    p_pose.add_argument("--x", type=float, required=True)
    p_pose.add_argument("--y", type=float, required=True)
    p_pose.add_argument("--z", type=float, required=True)
    p_pose.add_argument("--qx", type=float, default=0.0)
    p_pose.add_argument("--qy", type=float, default=0.0)
    p_pose.add_argument("--qz", type=float, default=0.0)
    p_pose.add_argument("--qw", type=float, default=1.0)
    p_pose.add_argument("--collision", action="store_true")
    p_pose.add_argument("--speed", type=float, default=1.0)
    p_pose.add_argument("--force", action="store_true")
    p_pose.add_argument("--bound-size", default="")
    p_pose.add_argument("--hand-action-type", default="")
    p_pose.add_argument("--point-cloud-frame-id", default="")
    p_pose.add_argument("--timeout-ms", type=int, default=5000)
    p_pose.add_argument("--cancel-after-sec", type=float, default=0.0)

    p_joints = sub.add_parser("move_joints")
    p_joints.add_argument("--task-id", default="test_move_joints")
    p_joints.add_argument("--arm-id", type=int, required=True)
    p_joints.add_argument("--joint-name", required=True)
    p_joints.add_argument("--position", required=True)
    p_joints.add_argument("--collision", action="store_true")
    p_joints.add_argument("--speed", type=float, default=1.0)
    p_joints.add_argument("--timeout-ms", type=int, default=5000)
    p_joints.add_argument("--cancel-after-sec", type=float, default=0.0)

    p_common = sub.add_parser("wait")
    p_common.add_argument("--timeout-sec", type=float, default=5.0)

    args = parser.parse_args()

    rclpy.init()
    node = ActionTestClient()

    if args.cmd == "wait":
        node.wait_servers(timeout_sec=args.timeout_sec)
        node.destroy_node()
        rclpy.shutdown()
        return

    if not node.wait_servers(timeout_sec=5.0):
        node.destroy_node()
        rclpy.shutdown()
        return

    if args.cmd == "move_pose":
        bound_size = _parse_float_list(args.bound_size)
        hand_action_type = [int(v) for v in _parse_list(args.hand_action_type)]
        node.send_move_pose(
            task_id=args.task_id,
            arm_id=args.arm_id,
            frame_id=args.frame_id,
            x=args.x,
            y=args.y,
            z=args.z,
            qx=args.qx,
            qy=args.qy,
            qz=args.qz,
            qw=args.qw,
            enable_collision_check=args.collision,
            speed=args.speed,
            use_force_control=args.force,
            bound_size=bound_size,
            hand_action_type=hand_action_type,
            point_cloud_frame_id=args.point_cloud_frame_id,
            timeout_ms=args.timeout_ms,
            cancel_after_sec=args.cancel_after_sec,
        )
    elif args.cmd == "move_joints":
        node.send_move_joints(
            task_id=args.task_id,
            arm_id=args.arm_id,
            joint_name=_parse_list(args.joint_name),
            position=_parse_float_list(args.position),
            enable_collision_check=args.collision,
            speed=args.speed,
            timeout_ms=args.timeout_ms,
            cancel_after_sec=args.cancel_after_sec,
        )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
