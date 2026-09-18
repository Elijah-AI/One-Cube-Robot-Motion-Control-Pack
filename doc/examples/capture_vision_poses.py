#!/usr/bin/env python3
"""Capture one DetectionPoseCloud result, or sort a saved result; never sends motion goals."""
import argparse
import copy
import json
import math
from pathlib import Path
import time
import uuid


def sorted_targets(targets, order):
    rows = copy.deepcopy(targets)
    if not rows:
        raise ValueError("No detected targets to save or sort")
    if order != "confidence-desc":
        frames = {row["pose"]["header"]["frame_id"] for row in rows}
        if len(frames) != 1 or not next(iter(frames)):
            raise ValueError("Coordinate sorting requires one common, nonempty frame_id")

    def key(row):
        p = row["pose"]["pose"]["position"]
        if order == "confidence-desc":
            values = (-float(row["confidence"]),)
        elif order == "z-desc-x-asc":
            values = (-float(p["z"]), float(p["x"]), float(p["y"]))
        else:
            values = (float(p[order[0]]),)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Nonfinite sort key in detection result")
        return values

    return sorted(rows, key=key)  # stable: equal keys preserve the detector order


def capture(args):
    import rclpy
    from rclpy.action import ActionClient
    from action_msgs.msg import GoalStatus
    from rosidl_runtime_py.convert import message_to_ordereddict
    from com_interfaces.action import DetectionPoseCloud

    rclpy.init()
    node = rclpy.create_node("vision_pose_capture_" + uuid.uuid4().hex[:8])
    client = ActionClient(node, DetectionPoseCloud, "/vision/detection_pose_cloud")
    handle = None
    finished = False
    try:
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/vision/detection_pose_cloud server is unavailable")
        goal = DetectionPoseCloud.Goal()
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.task_id = "capture_" + uuid.uuid4().hex
        goal.target_obj_name = args.target
        goal.expected_frame_id = args.frame
        goal.priority = args.priority
        goal.type = args.type
        deadline = time.monotonic() + args.timeout

        def wait(future):
            rclpy.spin_until_future_complete(
                node, future, timeout_sec=max(0.0, deadline - time.monotonic()))
            if not future.done():
                raise TimeoutError("Vision action exceeded the capture timeout")
            return future.result()

        handle = wait(client.send_goal_async(goal))
        if handle is None or not handle.accepted:
            raise RuntimeError("Vision goal was rejected")
        response = wait(handle.get_result_async())
        finished = True
        result = response.result
        if response.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            raise RuntimeError("Vision failed: " + result.error_message)
        data = dict(message_to_ordereddict(result))
        targets = data.pop("detected_targets")
        for index, row in enumerate(targets):
            row["source_index"] = index
        data.update(task_id=goal.task_id, targets=targets)
        return data
    finally:
        if handle is not None and handle.accepted and not finished:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=2.0)
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, help="Sort existing JSON without contacting ROS")
    parser.add_argument("--output", type=Path, required=True, help="New JSON file; existing files are not overwritten")
    parser.add_argument("--sort", choices=["confidence-desc", "x-asc", "y-asc", "z-asc", "z-desc-x-asc"], default="confidence-desc")
    parser.add_argument("--target", default="lefta")
    parser.add_argument("--frame", default="base_link", help="Requested frame; inspect the returned target frame")
    parser.add_argument("--priority", type=int, choices=range(1, 26), default=10)
    parser.add_argument("--type", type=int, choices=[0, 1], default=0)
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    if args.output.exists():
        parser.error("--output already exists; choose a new file")
    data = json.loads(args.input.read_text()) if args.input else capture(args)
    data["sorted_targets"] = sorted_targets(data["targets"], args.sort)
    data["sort_order"] = args.sort
    text = json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    with args.output.open("x") as stream:
        stream.write(text)
    print(f"Saved {len(data['targets'])} targets to {args.output}; no motion goals sent")


if __name__ == "__main__":
    main()
