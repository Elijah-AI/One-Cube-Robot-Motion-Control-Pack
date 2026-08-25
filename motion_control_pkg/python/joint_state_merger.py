import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from builtin_interfaces.msg import Time
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import (
    Pose,
    PoseArray,
    PoseStamped,
    Quaternion,
    Point,
)
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from com_interfaces.msg import MotorStatus


@dataclass
class _CachedJointState:
    names: List[str]
    positions: List[float]
    velocities: List[float]
    efforts: List[float]
    stamp_ns: int
    source: str


@dataclass
class _EndEffectorSample:
    stamp_ns: int
    frame_id: str
    pose: Pose


class JointStateMergerNode(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_merger")

        self.declare_parameter(
            "motor_status_topic", "/Control/MotorInfo")
        self.declare_parameter(
            "head_joint_topic", "/head_joint_states")
        self.declare_parameter(
            "leg_joint_topic", "/leg_joint_states")

        self.declare_parameter(
            "joint_states_topic", "/joint_states")
        self.declare_parameter(
            "end_effector_1_topic", "/end_effector_1_pose")
        self.declare_parameter(
            "end_effector_2_topic", "/end_effector_2_pose")

        self.declare_parameter(
            "end_effector_1_path_topic",
            "/end_effector_1_path")
        self.declare_parameter(
            "end_effector_2_path_topic",
            "/end_effector_2_path")
        self.declare_parameter(
            "end_effector_1_poses_topic",
            "/end_effector_1_poses")
        self.declare_parameter(
            "end_effector_2_poses_topic",
            "/end_effector_2_poses")
        self.declare_parameter(
            "end_effector_trajectory_markers_topic",
            "/end_effector_trajectory_markers")

        self.declare_parameter(
            "end_effector_default_frame", "base_link")
        self.declare_parameter(
            "publish_rate_hz", 60.0)
        self.declare_parameter(
            "max_staleness_sec", 0.1)

        self.declare_parameter(
            "trajectory_history_max_points", 100000)
        self.declare_parameter(
            "trajectory_history_max_seconds", 120.0)
        self.declare_parameter(
            "end_effector_1_color",
            [1.0, 0.2, 0.2, 1.0])
        self.declare_parameter(
            "end_effector_2_color",
            [0.2, 0.5, 1.0, 1.0])
        self.declare_parameter(
            "trajectory_marker_namespace",
            "end_effector_trajectory")
        self.declare_parameter(
            "trajectory_line_width", 0.005)
        self.declare_parameter(
            "trajectory_point_scale", 0.01)
        self.declare_parameter(
            "end_effector_pose_min_delta_m", 0.0)

        self.declare_parameter(
            "action_move_pose_topic_base", "/Control/move_pose")
        self.declare_parameter(
            "action_move_joints_topic_base", "/Control/move_joints")
        self.declare_parameter(
            "record_trajectory_only_during_action", True)
        self.declare_parameter(
            "action_status_staleness_sec", 2.0)
        self.declare_parameter(
            "action_tail_window_sec", 1.0)

        # ------------------------------------------------------------------
        motor_topic = self.get_parameter(
            "motor_status_topic").get_parameter_value().string_value
        head_topic = self.get_parameter(
            "head_joint_topic").get_parameter_value().string_value
        leg_topic = self.get_parameter(
            "leg_joint_topic").get_parameter_value().string_value

        js_topic = self.get_parameter(
            "joint_states_topic").get_parameter_value().string_value
        ee1_topic = self.get_parameter(
            "end_effector_1_topic").get_parameter_value().string_value
        ee2_topic = self.get_parameter(
            "end_effector_2_topic").get_parameter_value().string_value

        ee1_path_topic = self.get_parameter(
            "end_effector_1_path_topic").get_parameter_value().string_value
        ee2_path_topic = self.get_parameter(
            "end_effector_2_path_topic").get_parameter_value().string_value
        ee1_poses_topic = self.get_parameter(
            "end_effector_1_poses_topic").get_parameter_value().string_value
        ee2_poses_topic = self.get_parameter(
            "end_effector_2_poses_topic").get_parameter_value().string_value
        ee_markers_topic = self.get_parameter(
            "end_effector_trajectory_markers_topic"
        ).get_parameter_value().string_value

        self._default_ee_frame = self.get_parameter(
            "end_effector_default_frame").get_parameter_value().string_value

        rate_hz = self.get_parameter(
            "publish_rate_hz").get_parameter_value().double_value
        if rate_hz <= 0.0:
            self.get_logger().warn(
                f"Invalid publish_rate_hz={rate_hz}, falling back to 60.0 Hz")
            rate_hz = 60.0
        self._publish_interval_ns = int((1.0 / rate_hz) * 1e9)
        self._max_staleness_ns = int(
            self.get_parameter(
                "max_staleness_sec").get_parameter_value().double_value * 1e9)

        history_max_points = int(
            self.get_parameter(
                "trajectory_history_max_points").get_parameter_value().integer_value)
        if history_max_points <= 0:
            self.get_logger().warn(
                f"Invalid trajectory_history_max_points={history_max_points},"
                " falling back to 100000")
            history_max_points = 100000
        self._history_max_points = history_max_points

        history_max_seconds = float(
            self.get_parameter(
                "trajectory_history_max_seconds"
            ).get_parameter_value().double_value)
        if history_max_seconds < 0.0:
            self.get_logger().warn(
                "Invalid trajectory_history_max_seconds="
                f"{history_max_seconds}, falling back to 120.0 s")
            history_max_seconds = 120.0
        self._history_max_ns = int(history_max_seconds * 1e9)

        self._ee_min_delta_m = float(
            self.get_parameter(
                "end_effector_pose_min_delta_m"
            ).get_parameter_value().double_value)
        if self._ee_min_delta_m < 0.0:
            self._ee_min_delta_m = 0.0

        action_move_pose_base = self.get_parameter(
            "action_move_pose_topic_base").get_parameter_value().string_value
        action_move_joints_base = self.get_parameter(
            "action_move_joints_topic_base").get_parameter_value().string_value
        self._record_only_during_action = bool(
            self.get_parameter(
                "record_trajectory_only_during_action"
            ).get_parameter_value().bool_value)
        staleness_sec = float(
            self.get_parameter(
                "action_status_staleness_sec"
            ).get_parameter_value().double_value)
        if staleness_sec <= 0.0:
            self.get_logger().warn(
                f"Invalid action_status_staleness_sec={staleness_sec},"
                " falling back to 2.0 s")
            staleness_sec = 2.0
        self._action_status_staleness_ns = int(staleness_sec * 1e9)

        self._move_pose_status_topic = (
            action_move_pose_base.rstrip("/") + "/_action/status")
        self._move_joints_status_topic = (
            action_move_joints_base.rstrip("/") + "/_action/status")
        self._action_move_pose_active = False
        self._action_move_joints_active = False
        self._action_move_pose_stamp_ns: int = 0
        self._action_move_joints_stamp_ns: int = 0
        self._action_running_previous = False
        self._tail_window_active = False
        self._tail_window_end_ns: int = 0

        tail_sec = float(
            self.get_parameter(
                "action_tail_window_sec"
            ).get_parameter_value().double_value)
        if tail_sec < 0.0:
            self.get_logger().warn(
                f"Invalid action_tail_window_sec={tail_sec}, falling back to 1.0 s")
            tail_sec = 1.0
        self._action_tail_window_ns = int(tail_sec * 1e9)

        self._marker_ns = self.get_parameter(
            "trajectory_marker_namespace").get_parameter_value().string_value
        self._ee_color_1 = self._get_color_param("end_effector_1_color")
        self._ee_color_2 = self._get_color_param("end_effector_2_color")
        self._line_width = float(
            self.get_parameter(
                "trajectory_line_width").get_parameter_value().double_value)
        self._point_scale = float(
            self.get_parameter(
                "trajectory_point_scale").get_parameter_value().double_value)

        # ------------------------------------------------------------------
        self._lock = threading.Lock()
        self._arm_cache: Optional[_CachedJointState] = None
        self._head_cache: Optional[_CachedJointState] = None
        self._leg_cache: Optional[_CachedJointState] = None

        self._ee_history: List[Deque[_EndEffectorSample]] = [
            deque(maxlen=self._history_max_points),
            deque(maxlen=self._history_max_points),
        ]
        self._ee_last_frame_ids: List[str] = ["", ""]
        self._ee_history_changed: List[bool] = [False, False]
        self._ee_last_published_stamp_ns: int = 0

        self._last_publish_ns = 0
        self._received_arm_first = False
        self._received_head_first = False
        self._received_leg_first = False

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        subscriber_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        latched_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        self._arm_sub = self.create_subscription(
            MotorStatus, motor_topic, self._on_motor_status, subscriber_qos)
        self._head_sub = self.create_subscription(
            JointState, head_topic, self._on_head_joint, sensor_qos)
        self._leg_sub = self.create_subscription(
            JointState, leg_topic, self._on_leg_joint, sensor_qos)
        self._move_pose_status_sub = self.create_subscription(
            GoalStatusArray,
            self._move_pose_status_topic,
            self._on_move_pose_status,
            subscriber_qos)
        self._move_joints_status_sub = self.create_subscription(
            GoalStatusArray,
            self._move_joints_status_topic,
            self._on_move_joints_status,
            subscriber_qos)

        self._js_pub = self.create_publisher(
            JointState, js_topic, subscriber_qos)
        self._ee1_pub = self.create_publisher(
            PoseStamped, ee1_topic, subscriber_qos)
        self._ee2_pub = self.create_publisher(
            PoseStamped, ee2_topic, subscriber_qos)

        self._ee1_path_pub = self.create_publisher(
            Path, ee1_path_topic, latched_qos)
        self._ee2_path_pub = self.create_publisher(
            Path, ee2_path_topic, latched_qos)
        self._ee1_poses_pub = self.create_publisher(
            PoseArray, ee1_poses_topic, latched_qos)
        self._ee2_poses_pub = self.create_publisher(
            PoseArray, ee2_poses_topic, latched_qos)
        self._ee_markers_pub = self.create_publisher(
            MarkerArray, ee_markers_topic, latched_qos)

        publish_period_sec = 1.0 / max(rate_hz * 2.0, 100.0)
        self._timer = self.create_timer(
            publish_period_sec, self._on_publish_timer)

        self.get_logger().info(
            "JointState merger initialized:\n"
            f"  subscribe arm : {motor_topic}\n"
            f"  subscribe head: {head_topic}\n"
            f"  subscribe leg : {leg_topic}\n"
            f"  publish /joint_states: {js_topic}\n"
            f"  publish ee pose: {ee1_topic} / {ee2_topic}\n"
            f"  publish ee path: {ee1_path_topic} / {ee2_path_topic}\n"
            f"  publish ee poses: {ee1_poses_topic} / {ee2_poses_topic}\n"
            f"  publish markers: {ee_markers_topic}\n"
            f"  target rate {rate_hz:.1f} Hz, "
            f"max staleness {self._max_staleness_ns/1e6:.0f} ms\n"
            f"  ee trajectory window: "
            f"max {self._history_max_points} points / "
            f"{self._history_max_ns/1e9:.1f} seconds\n"
            f"  record during action only: {self._record_only_during_action}\n"
            f"  action status staleness: {self._action_status_staleness_ns/1e9:.1f} s\n"
            f"  action tail window: {self._action_tail_window_ns/1e9:.1f} s\n"
            f"  move_pose status: {self._move_pose_status_topic}\n"
            f"  move_joints status: {self._move_joints_status_topic}")

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def _get_color_param(self, name: str) -> ColorRGBA:
        raw = self.get_parameter(name).get_parameter_value().double_array_value
        if raw is None:
            raw = []
        arr = list(raw)
        while len(arr) < 4:
            arr.append(1.0 if len(arr) == 3 else 0.0)
        def _clamp(x: float) -> float:
            return max(0.0, min(1.0, float(x)))
        return ColorRGBA(
            r=_clamp(arr[0]),
            g=_clamp(arr[1]),
            b=_clamp(arr[2]),
            a=_clamp(arr[3]))

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------
    def _on_motor_status(self, msg: MotorStatus) -> None:
        try:
            n = len(msg.joint_names)
            ok_len = (
                len(msg.joint_positions) == n and
                len(msg.joint_velocities) == n and
                len(msg.joint_efforts) == n
            )
            if n > 0 and not ok_len:
                self.get_logger().error(
                    "MotorStatus arrays have inconsistent lengths: "
                    f"names={n}, pos={len(msg.joint_positions)}, "
                    f"vel={len(msg.joint_velocities)}, "
                    f"eff={len(msg.joint_efforts)}; message ignored")
                return
            stamp_ns = self._stamp_to_ns(msg.header.stamp)
            if n > 0:
                cache = _CachedJointState(
                    names=list(msg.joint_names),
                    positions=list(msg.joint_positions),
                    velocities=list(msg.joint_velocities),
                    efforts=list(msg.joint_efforts),
                    stamp_ns=stamp_ns,
                    source="dual_arm")
                with self._lock:
                    self._arm_cache = cache
                if not self._received_arm_first:
                    self._received_arm_first = True
                    self.get_logger().info(
                        f"First dual-arm MotorStatus received: {n} joints")

            self._process_end_effectors(msg, stamp_ns)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Failed to process MotorStatus: {exc}", exc_info=True)

    def _on_head_joint(self, msg: JointState) -> None:
        try:
            n = len(msg.name)
            if len(msg.position) != n or \
               len(msg.velocity) != n or \
               len(msg.effort) != n:
                self.get_logger().error(
                    "head_joint_states arrays have inconsistent lengths: "
                    f"names={n}, pos={len(msg.position)}, "
                    f"vel={len(msg.velocity)}, "
                    f"eff={len(msg.effort)}; message ignored")
                return
            stamp_ns = self._stamp_to_ns(msg.header.stamp)
            cache = _CachedJointState(
                names=list(msg.name),
                positions=list(msg.position),
                velocities=list(msg.velocity),
                efforts=list(msg.effort),
                stamp_ns=stamp_ns,
                source="head")
            with self._lock:
                self._head_cache = cache
            if not self._received_head_first:
                self._received_head_first = True
                self.get_logger().info(
                    f"First head JointState received: {n} joints")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Failed to process head JointState: {exc}", exc_info=True)

    def _on_leg_joint(self, msg: JointState) -> None:
        try:
            n = len(msg.name)
            if len(msg.position) != n or \
               len(msg.velocity) != n or \
               len(msg.effort) != n:
                self.get_logger().error(
                    "leg_joint_states arrays have inconsistent lengths: "
                    f"names={n}, pos={len(msg.position)}, "
                    f"vel={len(msg.velocity)}, "
                    f"eff={len(msg.effort)}; message ignored")
                return
            stamp_ns = self._stamp_to_ns(msg.header.stamp)
            cache = _CachedJointState(
                names=list(msg.name),
                positions=list(msg.position),
                velocities=list(msg.velocity),
                efforts=list(msg.effort),
                stamp_ns=stamp_ns,
                source="leg")
            with self._lock:
                self._leg_cache = cache
            if not self._received_leg_first:
                self._received_leg_first = True
                self.get_logger().info(
                    f"First leg JointState received: {n} joints")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Failed to process leg JointState: {exc}", exc_info=True)

    def _on_move_pose_status(self, msg: GoalStatusArray) -> None:
        try:
            stamp_ns = time.monotonic_ns()
            active = self._goal_status_array_active(msg)
            with self._lock:
                self._action_move_pose_active = active
                self._action_move_pose_stamp_ns = stamp_ns
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Failed to process move_pose status: {exc}")

    def _on_move_joints_status(self, msg: GoalStatusArray) -> None:
        try:
            stamp_ns = time.monotonic_ns()
            active = self._goal_status_array_active(msg)
            with self._lock:
                self._action_move_joints_active = active
                self._action_move_joints_stamp_ns = stamp_ns
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Failed to process move_joints status: {exc}")

    @staticmethod
    def _goal_status_array_active(msg: GoalStatusArray) -> bool:
        GOAL_STATUS_ACCEPTED = 1
        GOAL_STATUS_EXECUTING = 2
        GOAL_STATUS_CANCELING = 3
        active_set = {
            GOAL_STATUS_ACCEPTED,
            GOAL_STATUS_EXECUTING,
            GOAL_STATUS_CANCELING,
        }
        for status in msg.status_list:
            if int(status.status) in active_set:
                return True
        return False

    def _is_action_running_locked(self, now_ns: int) -> bool:
        if not self._record_only_during_action:
            return True
        stale = self._action_status_staleness_ns
        move_pose_ok = self._action_move_pose_active and (
            now_ns - self._action_move_pose_stamp_ns <= stale)
        move_joints_ok = self._action_move_joints_active and (
            now_ns - self._action_move_joints_stamp_ns <= stale)
        active = move_pose_ok or move_joints_ok
        if active:
            if self._tail_window_active:
                self._tail_window_active = False
                self._tail_window_end_ns = 0
            return True
        if self._tail_window_active:
            return now_ns <= self._tail_window_end_ns
        return False

    # ------------------------------------------------------------------
    # End-effectors: publish current stamp + build trajectory history
    # ------------------------------------------------------------------
    def _process_end_effectors(self, msg: MotorStatus, stamp_ns: int) -> None:
        n_ee = len(msg.end_effector_pose)
        if n_ee == 0:
            return
        frame_id = msg.header.frame_id or self._default_ee_frame
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
            stamp_ns = self._stamp_to_ns(stamp)

        def _copy_pose(src: Pose) -> Pose:
            dst = Pose()
            dst.position.x = float(src.position.x)
            dst.position.y = float(src.position.y)
            dst.position.z = float(src.position.z)
            dst.orientation.x = float(src.orientation.x)
            dst.orientation.y = float(src.orientation.y)
            dst.orientation.z = float(src.orientation.z)
            dst.orientation.w = float(src.orientation.w)
            return dst

        def _build_ps(index: int, pose: Pose) -> PoseStamped:
            out = PoseStamped()
            out.header = Header(stamp=stamp, frame_id=frame_id)
            out.pose = pose
            return out

        missing_ee2_warned_once = getattr(
            self, "_missing_ee2_warned", False)

        for idx in range(2):
            if idx < n_ee:
                pose = _copy_pose(msg.end_effector_pose[idx])
            else:
                if idx == 1 and not missing_ee2_warned_once:
                    self._missing_ee2_warned = True
                    self.get_logger().warn(
                        "MotorStatus only provides "
                        f"{n_ee} end_effector_pose; "
                        "end_effector_2 history and trajectory will be "
                        "incomplete until the array length >= 2")
                continue
            ps = _build_ps(idx, pose)
            try:
                if idx == 0:
                    self._ee1_pub.publish(ps)
                else:
                    self._ee2_pub.publish(ps)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"Failed to publish ee{idx+1} pose: {exc}")

            with self._lock:
                now_ns = time.monotonic_ns()
                if not self._is_action_running_locked(now_ns):
                    continue
                history = self._ee_history[idx]
                drop = False
                if self._ee_min_delta_m > 0.0 and len(history) > 0:
                    last = history[-1].pose.position
                    dx = pose.position.x - last.x
                    dy = pose.position.y - last.y
                    dz = pose.position.z - last.z
                    if dx * dx + dy * dy + dz * dz < \
                       self._ee_min_delta_m * self._ee_min_delta_m:
                        drop = True
                if not drop:
                    history.append(_EndEffectorSample(
                        stamp_ns=stamp_ns,
                        frame_id=frame_id,
                        pose=pose))
                    self._ee_last_frame_ids[idx] = frame_id
                    self._ee_history_changed[idx] = True

    # ------------------------------------------------------------------
    # Publish merged JointState + end-effector trajectories on timer
    # ------------------------------------------------------------------
    def _on_publish_timer(self) -> None:
        try:
            now_ns = time.monotonic_ns()
            do_publish = (
                now_ns - self._last_publish_ns >= self._publish_interval_ns
            )
            history_changed = False
            histories_snapshot: List[List[_EndEffectorSample]] = [[]] * 2
            frame_ids: List[str] = ["", ""]
            if do_publish:
                with self._lock:
                    stale = self._action_status_staleness_ns
                    pose_active = self._action_move_pose_active and (
                        now_ns - self._action_move_pose_stamp_ns <= stale)
                    joints_active = self._action_move_joints_active and (
                        now_ns - self._action_move_joints_stamp_ns <= stale)
                    strict_active_now = pose_active or joints_active

                    if strict_active_now and not self._action_running_previous:
                        self._tail_window_active = False
                        self._tail_window_end_ns = 0
                        for idx in range(2):
                            self._ee_history[idx].clear()
                            self._ee_history_changed[idx] = True
                        self.get_logger().info(
                            "MovePose/MoveJoints action started; "
                            "end-effector trajectory histories cleared for"
                            " a new recording session")
                    elif self._action_running_previous and not strict_active_now:
                        if self._action_tail_window_ns > 0:
                            self._tail_window_active = True
                            self._tail_window_end_ns = (
                                now_ns + self._action_tail_window_ns)
                            self.get_logger().info(
                                "MovePose/MoveJoints action ended; "
                                "recording tail window for "
                                f"{self._action_tail_window_ns/1e9:.2f} s")
                        else:
                            self._tail_window_active = False
                            self._tail_window_end_ns = 0

                    action_running_now = self._is_action_running_locked(now_ns)
                    self._action_running_previous = action_running_now
                    if self._tail_window_active and now_ns > self._tail_window_end_ns:
                        self._tail_window_active = False
                        self._action_running_previous = False
                        self.get_logger().info(
                            "Tail window elapsed; end-effector recording"
                            " stopped until next action")

                    arm = self._arm_cache
                    head = self._head_cache
                    leg = self._leg_cache
                    for idx in range(2):
                        if self._history_max_ns > 0:
                            h = self._ee_history[idx]
                            cutoff_ns = now_ns - self._history_max_ns
                            while h and h[0].stamp_ns < cutoff_ns:
                                h.popleft()
                                self._ee_history_changed[idx] = True
                        if self._ee_history_changed[idx]:
                            history_changed = True
                            self._ee_history_changed[idx] = False
                        histories_snapshot[idx] = list(self._ee_history[idx])
                        frame_ids[idx] = (
                            self._ee_last_frame_ids[idx] or self._default_ee_frame
                        )
            else:
                return

            if history_changed:
                self._publish_end_effector_trajectories(
                    histories_snapshot, frame_ids, self.get_clock().now().to_msg())

            caches: List[_CachedJointState] = []
            stale_sources: List[str] = []
            for source_cache, source_name in (
                (arm, "dual_arm"),
                (head, "head"),
                (leg, "leg"),
            ):
                if source_cache is None:
                    continue
                age = now_ns - source_cache.stamp_ns
                if self._max_staleness_ns > 0 and age > self._max_staleness_ns:
                    stale_sources.append(
                        f"{source_name}({age/1e6:.0f}ms)")
                    continue
                caches.append(source_cache)

            if caches:
                merged = self._merge_caches(caches)
                merged.header.stamp = self.get_clock().now().to_msg()
                self._js_pub.publish(merged)
            else:
                if arm is None and head is None and leg is None:
                    self._last_publish_ns = now_ns
                    return
                if getattr(self, "_warned_empty_once", None) != stale_sources:
                    self._warned_empty_once = stale_sources
                    if stale_sources:
                        self.get_logger().warn(
                            "All cached sources stale; skipping /joint_states"
                            " publish: " + ", ".join(stale_sources))
                self._last_publish_ns = now_ns
                return

            self._last_publish_ns = now_ns
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Failed to publish merged data: {exc}", exc_info=True)

    # ------------------------------------------------------------------
    # End-effector trajectory publishers
    # ------------------------------------------------------------------
    def _publish_end_effector_trajectories(
        self,
        histories: List[List[_EndEffectorSample]],
        frame_ids: List[str],
        stamp_msg,
    ) -> None:
        def _build_path(index: int) -> Optional[Path]:
            samples = histories[index]
            if not samples:
                return None
            path = Path()
            path.header = Header(
                stamp=stamp_msg, frame_id=frame_ids[index]
                or self._default_ee_frame)
            for sample in samples:
                ps = PoseStamped()
                ps.header = Header(
                    stamp=self._ns_to_stamp(sample.stamp_ns),
                    frame_id=sample.frame_id or self._default_ee_frame)
                ps.pose = sample.pose
                path.poses.append(ps)
            return path

        def _build_pose_array(index: int) -> Optional[PoseArray]:
            samples = histories[index]
            if not samples:
                return None
            arr = PoseArray()
            arr.header = Header(
                stamp=stamp_msg,
                frame_id=frame_ids[index] or self._default_ee_frame)
            for sample in samples:
                arr.poses.append(sample.pose)
            return arr

        try:
            path1 = _build_path(0)
            if path1 is not None:
                self._ee1_path_pub.publish(path1)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Failed to publish ee1 path: {exc}")

        try:
            path2 = _build_path(1)
            if path2 is not None:
                self._ee2_path_pub.publish(path2)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Failed to publish ee2 path: {exc}")

        try:
            pa1 = _build_pose_array(0)
            if pa1 is not None:
                self._ee1_poses_pub.publish(pa1)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Failed to publish ee1 poses: {exc}")

        try:
            pa2 = _build_pose_array(1)
            if pa2 is not None:
                self._ee2_poses_pub.publish(pa2)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Failed to publish ee2 poses: {exc}")

        try:
            marker_array = self._build_markers(
                histories, frame_ids, stamp_msg)
            self._ee_markers_pub.publish(marker_array)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Failed to publish ee trajectory markers: {exc}")

    def _build_markers(
        self,
        histories: List[List[_EndEffectorSample]],
        frame_ids: List[str],
        stamp_msg,
    ) -> MarkerArray:
        MARKER_ACTION_ADD = 0
        MARKER_ACTION_DELETEALL = 3
        MARKER_LINE_STRIP = 4
        MARKER_SPHERE_LIST = 7
        array = MarkerArray()
        seen_ids = set()

        common_header_frames = set()
        for idx in range(2):
            samples = histories[idx]
            if not samples:
                continue
            common_header_frames.add(
                frame_ids[idx] or self._default_ee_frame)
        if common_header_frames:
            clear = Marker()
            clear.header = Header(
                stamp=stamp_msg,
                frame_id=next(iter(common_header_frames)))
            clear.ns = self._marker_ns
            clear.id = 0
            clear.action = MARKER_ACTION_DELETEALL
            clear.frame_locked = True
            array.markers.append(clear)
            seen_ids.add((clear.ns, clear.id))

        for idx in range(2):
            color = self._ee_color_1 if idx == 0 else self._ee_color_2
            samples = histories[idx]
            frame = frame_ids[idx] or self._default_ee_frame
            header = Header(stamp=stamp_msg, frame_id=frame)

            if not samples:
                continue

            uid = 10000 + idx * 10

            line = Marker()
            line.header = header
            line.ns = self._marker_ns
            line.id = uid + 1
            if (line.ns, line.id) in seen_ids:
                continue
            seen_ids.add((line.ns, line.id))
            line.type = MARKER_LINE_STRIP
            line.action = MARKER_ACTION_ADD
            line.frame_locked = True
            line.scale.x = max(0.0001, self._line_width)
            line.color = color
            line.pose.orientation.w = 1.0
            for s in samples:
                line.points.append(Point(
                    x=float(s.pose.position.x),
                    y=float(s.pose.position.y),
                    z=float(s.pose.position.z)))
            array.markers.append(line)

            points = Marker()
            points.header = header
            points.ns = self._marker_ns
            points.id = uid + 2
            if (points.ns, points.id) in seen_ids:
                continue
            seen_ids.add((points.ns, points.id))
            points.type = MARKER_SPHERE_LIST
            points.action = MARKER_ACTION_ADD
            points.frame_locked = True
            s = max(0.0001, self._point_scale)
            points.scale.x = s
            points.scale.y = s
            points.scale.z = s
            points.color = color
            points.pose.orientation.w = 1.0
            for s_ in samples:
                points.points.append(Point(
                    x=float(s_.pose.position.x),
                    y=float(s_.pose.position.y),
                    z=float(s_.pose.position.z)))
            array.markers.append(points)

        return array

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _ns_to_stamp(stamp_ns: int) -> Time:
        secs = int(stamp_ns // 1_000_000_000)
        nanos = int(stamp_ns - secs * 1_000_000_000)
        return Time(sec=secs, nanosec=nanos)

    @staticmethod
    def _merge_caches(
        caches: List[_CachedJointState],
    ) -> JointState:
        seen: Dict[str, int] = {}
        merged_names: List[str] = []
        merged_positions: List[float] = []
        merged_velocities: List[float] = []
        merged_efforts: List[float] = []

        for cache in caches:
            for i, name in enumerate(cache.names):
                if not name:
                    continue
                if name in seen:
                    idx = seen[name]
                    merged_positions[idx] = cache.positions[i]
                    merged_velocities[idx] = cache.velocities[i]
                    merged_efforts[idx] = cache.efforts[i]
                else:
                    seen[name] = len(merged_names)
                    merged_names.append(name)
                    merged_positions.append(cache.positions[i])
                    merged_velocities.append(cache.velocities[i])
                    merged_efforts.append(cache.efforts[i])

        msg = JointState()
        msg.name = merged_names
        msg.position = merged_positions
        msg.velocity = merged_velocities
        msg.effort = merged_efforts
        return msg


def main() -> None:
    rclpy.init()
    node = JointStateMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("JointState merger stopped by user")
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(
            f"JointState merger crashed: {exc}", exc_info=True)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
