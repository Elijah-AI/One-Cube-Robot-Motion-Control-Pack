#!/usr/bin/env python3

import argparse
import time
from typing import Any

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from com_interfaces.action import MoveJoints, MovePose


class AlternatingActionClient(Node):
    def __init__(self) -> None:
        super().__init__("alternating_left_arm_action_client")
        self._move_pose_client = ActionClient(
            self, MovePose, "/Control/move_pose"
        )
        self._move_joints_client = ActionClient(
            self, MoveJoints, "/Control/move_joints"
        )

    def wait_for_servers(self, timeout_sec: float) -> bool:
        if not self._move_pose_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("MovePose action server is not available")
            return False
        if not self._move_joints_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("MoveJoints action server is not available")
            return False
        return True

    def send_move_pose(self) -> bool:
        goal = MovePose.Goal()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "base_link"
        goal.task_id = "MANUAL_LEFT_POSE_01"
        goal.arm_id = MovePose.Goal.ARM_LEFT
        goal.control_mode = MovePose.Goal.CONTROL_POSITION

        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.header.frame_id = "head_pitch_link"
        goal.target_pose.pose.position.x = 0.809191
        goal.target_pose.pose.position.y = -0.238137
        goal.target_pose.pose.position.z = -0.202854
        goal.target_pose.pose.orientation.x = -0.657273
        goal.target_pose.pose.orientation.y = 0.239505
        goal.target_pose.pose.orientation.z = -0.714153
        goal.target_pose.pose.orientation.w = -0.0247918

        goal.bound_size = [0.0, 0.0, 0.1]
        goal.enable_collision_check = False
        goal.use_force_control = False
        goal.enable_move_line = False
        goal.speed = 0.2
        goal.timeout_ms = 45000

        return self._send_goal_and_wait(
            self._move_pose_client, goal, "MovePose"
        )

    def send_move_joints(self) -> bool:
        goal = MoveJoints.Goal()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "base_link"
        goal.task_id = "MANUAL_LEFT_JOINT_01"
        goal.arm_id = MoveJoints.Goal.ARM_LEFT
        goal.control_mode = MoveJoints.Goal.CONTROL_POSITION
        goal.joint_name = [
            "Joint1_L",
            "Joint2_L",
            "Joint3_L",
            "Joint4_L",
            "Joint5_L",
            "Joint6_L",
            "Joint7_L",
        ]
        goal.position = [1.7, -1.2, -1.7, -1.6, 0.0, 0.0, 0.0]
        goal.enable_collision_check = False
        goal.speed = 0.2
        goal.timeout_ms = 30000

        return self._send_goal_and_wait(
            self._move_joints_client, goal, "MoveJoints"
        )

    def _send_goal_and_wait(
        self, client: ActionClient, goal: Any, action_name: str
    ) -> bool:
        def feedback_callback(feedback_message: Any) -> None:
            feedback = feedback_message.feedback
            self.get_logger().info(
                f"{action_name} feedback: progress={feedback.progress_rate:.1f}% "
                f"status='{feedback.task_status}' "
                f"step='{feedback.current_step_info}'"
            )

        self.get_logger().info(f"Sending {action_name} goal")
        send_future = client.send_goal_async(
            goal, feedback_callback=feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error(f"Failed to send {action_name} goal")
            return False
        if not goal_handle.accepted:
            self.get_logger().error(f"{action_name} goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_response = result_future.result()
        if result_response is None:
            self.get_logger().error(f"Failed to receive {action_name} result")
            return False

        result = result_response.result
        log_message = (
            f"{action_name} result: success={result.success}, "
            f"error='{result.error_message}', "
            f"execution_time={result.execution_time:.3f}s, "
            f"final_distance_error={result.final_distance_error:.6f}"
        )
        if result.success:
            self.get_logger().info(log_message)
        else:
            self.get_logger().error(log_message)
        return bool(result.success)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Alternately send left MovePose and MoveJoints goals"
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Delay between completed goals in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=0,
        help="Number of pose/joints cycles; 0 runs continuously",
    )
    parser.add_argument(
        "--server-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for each action server",
    )
    args = parser.parse_args()

    if args.interval < 0.0:
        parser.error("--interval must be non-negative")
    if args.cycles < 0:
        parser.error("--cycles must be non-negative")
    if args.server_timeout <= 0.0:
        parser.error("--server-timeout must be positive")

    rclpy.init()
    node = AlternatingActionClient()
    try:
        if not node.wait_for_servers(args.server_timeout):
            return

        completed_cycles = 0
        while rclpy.ok() and (
            args.cycles == 0 or completed_cycles < args.cycles
        ):
            node.get_logger().info(
                f"Starting alternating cycle {completed_cycles + 1}"
            )
            if not node.send_move_pose():
                node.get_logger().error(
                    "MovePose failed; stopping alternating action client"
                )
                break
            time.sleep(args.interval)

            if not rclpy.ok():
                break
            if not node.send_move_joints():
                node.get_logger().error(
                    "MoveJoints failed; stopping alternating action client"
                )
                break
            completed_cycles += 1

            if args.cycles == 0 or completed_cycles < args.cycles:
                time.sleep(args.interval)
    except KeyboardInterrupt:
        node.get_logger().info("Alternating action client stopped")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
