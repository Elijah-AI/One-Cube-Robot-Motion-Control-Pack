"""通过真实 ROS 服务和两条 PTY 验证电机模式、正负 PWM 及停止应答。"""
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
from ds485_interfaces.msg import GripperHealth, GripperStop
from ds485_interfaces.srv import ServoMoveAngle, ServoSetLimits, ServoSetSwitch, ServoSetTorque

from test_ros_pty import Device


class Motor(Device):
    def __init__(self, ack):
        self.pwm = 0
        self.commands = []
        self.drop_next_ack = False
        self.reject_next = False
        self.stuck_feedback = False
        self.fault = 0
        super().__init__(ack)

    def data(self, addr):
        if addr == 0x49:
            return struct.pack("<h", 9 if self.stuck_feedback else self.pwm)
        return super().data(addr)

    def process(self, frame):
        if not self.blackout and self.fault and frame[4:6] == bytes([2, 1]):
            self.sequence += 1
            body = bytes([1, 5, 2, 1, self.fault, self.sequence % 256])
            os.write(self.master, bytes([0xF9, 0xF5]) + body + bytes([~sum(body) & 255]))
            return
        if not self.blackout and frame[4] == 3:
            addr, payload = frame[5], frame[6:-1]
            if addr == 0x6E:
                pwm, = struct.unpack("<h", payload)
                if pwm:
                    assert self.reg[0x5A] == bytes([2])
                if pwm and self.reject_next:
                    self.reject_next = False
                    body = bytes([1, 2, 0x40])
                    os.write(self.master, bytes([0xF9, 0xF5]) + body + bytes([~sum(body) & 255]))
                    return
                self.pwm = pwm
                self.commands.append((time.monotonic(), pwm))
                if pwm and self.drop_next_ack:
                    self.drop_next_ack = False
                    return
            if addr == 0x64 and self.reg[0x5A] == bytes([2]):
                assert payload != bytes([2]), "电机模式固件拒绝位置保持"
            if addr == 0x64 and payload == bytes([1]):
                self.pwm = 0
        super().process(frame)


def run(binary, directory, devices):
    params = {"control_strategy": "torque", "state_rate_hz": 40.0}
    for side, device in zip(("left", "right"), devices):
        params[side] = dict(port_type="serial", serial_device=device.path, gap_ms=2, timeout_ms=50)
    path = Path(directory, "motor.yaml")
    path.write_text(yaml.safe_dump({"ds485_node": {"ros__parameters": params}}))
    log = tempfile.TemporaryFile(mode="w+")
    process = subprocess.Popen([binary, "--ros-args", "--params-file", str(path)], stdout=log, stderr=log)
    node = rclpy.create_node("motor_bench")
    client = node.create_client(ServoSetTorque, "/ds485/servo/set_torque")
    limits = node.create_client(ServoSetLimits, "/ds485/servo/set_limits")
    switch = node.create_client(ServoSetSwitch, "/ds485/servo/set_switch")
    move = node.create_client(ServoMoveAngle, "/ds485/servo/move_angle")
    stop = node.create_publisher(GripperStop, "/ds485/gripper/stop", 10)
    states = {}
    node.create_subscription(GripperHealth, "/ds485/gripper/health",
                             lambda msg: states.update({msg.side: msg}), 20)

    def until(condition, timeout=6):
        deadline = time.monotonic() + timeout
        while not condition():
            assert process.poll() is None and time.monotonic() < deadline, states
            rclpy.spin_once(node, timeout_sec=.01)

    def torque(side, pwm, ms=0):
        return client.call_async(ServoSetTorque.Request(side=side, pwm=pwm, duration_ms=ms))

    def result(future):
        until(future.done)
        return future.result()

    left, right = devices
    busy = None
    try:
        assert client.wait_for_service(timeout_sec=5)
        until(lambda: all(i in states and states[i].feedback_valid for i in (0, 1)))
        assert not left.writes and not right.writes
        if "--embedded" in sys.argv:
            from std_srvs.srv import SetBool, Trigger
            busy = node.create_client(Trigger, "/offline/left_busy")
            assert busy.wait_for_service(timeout_sec=3)
            assert not result(busy.call_async(Trigger.Request())).success
            gate = node.create_client(SetBool, "/offline/gripper_admit")
            assert gate.wait_for_service(timeout_sec=3)
            refused = result(torque(0, 120, 200))
            assert not refused.success and not refused.sent and refused.message == "HOST_DENIED"
            assert not left.writes and not right.writes
            # A stop remains available even while the host rejects new motion.
            assert result(torque(1, 0)).stop_confirmed
            assert result(gate.call_async(SetBool.Request(data=True))).success
        assert not result(torque(0, -1001)).success
        assert not result(torque(2, 1)).success
        assert not result(move.call_async(ServoMoveAngle.Request(side=0, angle_deg=10.0))).success
        assert not left.commands and not right.commands

        finite = torque(0, 100, 200)
        until(lambda: left.pwm == 100)
        continuous = result(torque(1, -175))
        assert continuous.success and continuous.sent and not continuous.acknowledged
        pulse = result(finite)
        assert pulse.success and pulse.sent and pulse.acknowledged and pulse.stop_confirmed, pulse
        interval = 1000 * (left.commands[1][0] - left.commands[0][0])
        assert 170 < interval < 260, interval
        assert [p for _, p in left.commands] == [100, 0]
        assert left.reg[0x5A] == bytes([0]) and left.reg[0x64] == bytes([1])
        assert right.pwm == -175
        assert result(torque(1, 80)).success
        manual = result(torque(1, 0))
        assert manual.success and manual.stop_confirmed and not manual.acknowledged
        assert [p for _, p in right.commands] == [-175, 80, 0]

        finite = torque(0, -100, 3000)
        until(lambda: left.pwm == -100 and states[0].phase == "TORQUE_ACTIVE" and
              stop.get_subscription_count() > 0)
        stop.publish(GripperStop(side=0))
        interrupted = result(finite)
        assert not interrupted.success and interrupted.stop_confirmed
        assert "PULSE_INTERRUPTED" in interrupted.message

        until(lambda: states[0].reason == "STOP_REQUESTED" and states[0].stop_confirmed)
        left.drop_next_ack = True
        unknown = result(torque(0, 77, 200))
        assert not unknown.success and unknown.sent and not unknown.acknowledged, unknown
        assert unknown.stop_confirmed and left.pwm == 0
        assert sum(p == 77 for _, p in left.commands) == 1
        left.reject_next = True
        rejected = result(torque(0, 78))
        assert rejected.sent and not rejected.acknowledged and not rejected.success
        assert rejected.stop_confirmed and "0x40" in rejected.message

        assert result(torque(0, 90)).success
        left.fault = 4
        until(lambda: states[0].phase == "TORQUE_FAULT" and states[0].stop_confirmed)
        assert "SERVO_FAULT" in states[0].reason and left.pwm == 0
        assert not result(torque(0, 85)).success
        left.fault = 0
        assert result(torque(0, 90)).success
        if busy:
            assert result(busy.call_async(Trigger.Request())).success
        left.stuck_feedback = True
        failed = result(torque(0, 0))
        assert not failed.success and not failed.stop_confirmed, failed
        if busy:
            assert result(busy.call_async(Trigger.Request())).success
        left.stuck_feedback = False
        assert result(torque(0, 0)).stop_confirmed
        if busy:
            assert not result(busy.call_async(Trigger.Request())).success
        assert result(torque(1, 66)).success
        right.blackout = True
        until(lambda: states[1].phase == "STOP_UNCONFIRMED")
        right.blackout = False
        assert result(torque(1, 0)).stop_confirmed
        assert sum(p == 66 for _, p in right.commands) == 1
        for device in devices:
            assert all(addr in (0x5A, 0x64, 0x6E) for addr, _ in device.writes)
        explicit = ServoSetLimits.Request(side=0, pwm_limit=-1, current_limit=-1, speed_limit=123)
        assert result(limits.call_async(explicit)).success
        assert left.reg[0x5D] == struct.pack("<H", 123)
        assert left.reg[0x5C] == struct.pack("<H", 1000)
        assert result(torque(0, 64)).success
        assert result(switch.call_async(ServoSetSwitch.Request(side=0, mode=0))).success
        assert left.pwm == 0 and left.reg[0x64] == bytes([0])
        if busy:
            # The production host fences motion and queues a brake before waiting for idle.
            host_stop = node.create_client(Trigger, "/offline/host_stop")
            assert host_stop.wait_for_service(timeout_sec=3)
            assert result(torque(0, 45)).success
            assert result(host_stop.call_async(Trigger.Request())).success
            until(lambda: left.pwm == 0 and states[0].reason == "HOST_STOP" and states[0].stop_confirmed)
            assert not result(busy.call_async(Trigger.Request())).success
            assert not result(torque(0, 15)).success
            assert result(gate.call_async(SetBool.Request(data=True))).success
        assert result(torque(0, 45)).success
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=5) == 0 and left.pwm == 0
        assert left.reg[0x64] == bytes([1]) and left.reg[0x5A] == bytes([0])
        print(f"双侧恒定 PWM、抢占、丢应答、反馈故障及退出制动通过，200 ms 脉冲实测 {interval:.1f} ms")
    except BaseException:
        log.seek(0)
        print("".join(log.readlines()[-30:]), file=sys.stderr)
        raise
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
        node.destroy_node()
        log.close()


if __name__ == "__main__":
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_DOMAIN_ID"] = str(105 + os.getpid() % 70)
    with tempfile.TemporaryDirectory() as directory:
        os.environ["ROS_LOG_DIR"] = directory
        rclpy.init(args=[])
        devices = [Motor(True), Motor(False)]
        try:
            run(sys.argv[1], directory, devices)
        finally:
            for device in devices:
                device.close()
            rclpy.shutdown()
