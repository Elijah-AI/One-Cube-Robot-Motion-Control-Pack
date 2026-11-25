"""ROS 2 Humble 集成验收。两条 PTY 各模拟一个 ID=1 的设备，测试值仅在临时文件中生效。"""
import os
import pty
import select
import signal
import statistics
import struct
import subprocess
import sys
import tempfile
import threading
import time
import tty
from pathlib import Path

import rclpy
import yaml
from ds485_interfaces.msg import GripperCommand, GripperHealth, GripperResult, GripperStop
from ds485_interfaces.srv import ServoMoveAngle, ServoReadRegister, ServoWriteRegister
from std_srvs.srv import Trigger


def u16(value):
    return struct.pack("<H", value)


class Device:
    def __init__(self, ack):
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave)
        tty.setraw(self.slave)
        # 上次连接遗留的合法位置帧，新节点必须先丢弃。
        stale=bytes.fromhex("01 07 02 46 F1 06 00 00")
        os.write(self.master, bytes.fromhex("F9 F5")+stale+bytes([~sum(stale)&255]))
        self.alive = True
        self.blackout = False
        self.ignore_targets = False
        self.contact_at = None
        self.angle = 0
        self.target = 0
        self.temp = 25
        self.sequence = 0
        self.targets = []
        self.writes = []
        self.ack = ack
        self.reg = {0x04: b"\0", 0x11: bytes([8 if ack else 0]), 0x13: u16(1000),
                    0x14: u16(1000), 0x19: struct.pack("<h", -1800), 0x1A: u16(1800),
                    0x1C: u16(800), 0x32: b"\x0f", 0x33: b"\0", 0x34: b"T",
                    0x3D: bytes([65, 5]), 0x3E: u16(900)+bytes([3]), 0x3F: u16(700)+bytes([20]),
                    0x5A: b"\0", 0x5B: u16(1000), 0x5C: u16(1000), 0x5D: u16(200), 0x64: b"\1"}
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def data(self, addr):
        load = self.contact_at is not None and self.target > self.contact_at
        values = {0x46: ("i", self.angle), 0x47: ("h", 0), 0x48: ("h", 70 if load else 5),
                  0x49: ("h", 150 if load else 20), 0x4A: ("h", self.temp), 0x4B: ("h", 24000),
                  0x4C: ("i", self.target-self.angle)}
        if addr in values:
            fmt, value = values[addr]
            return struct.pack("<"+fmt, value)
        return self.reg[addr]

    def process(self, frame):
        if sum(frame[2:]) & 255 != 255 or frame[2] != 1:
            return
        if self.blackout:
            return
        cmd, addr = frame[4:6]
        data = frame[6:-1]
        if cmd == 2:
            if addr == 1:
                self.sequence += 1
                payload = bytes([0, self.sequence % 256])
            elif addr == 0x59:
                payload = b"".join(self.data(a) for a in data)
            else:
                payload = self.data(addr)
            body = bytes([1, 3+len(payload), cmd, addr])+payload
        else:
            self.writes.append((addr, data))
            if addr == 0x69:
                target, = struct.unpack("<h", data)
                self.targets.append((time.monotonic(), target))
                if self.ignore_targets:
                    return
                self.target=target
                self.angle = min(self.contact_at, self.target) if self.contact_at is not None else self.target
            elif addr == 0x64:
                self.target = self.angle
            self.reg[addr] = data
            if not self.ack:
                return
            body = bytes([1, 2, 0])
        reply = bytes([0xF9,0xF5])+body+bytes([(~sum(body)) & 255])
        os.write(self.master, reply)

    def loop(self):
        buf = bytearray()
        while self.alive:
            if not select.select([self.master], [], [], .1)[0]:
                continue
            buf.extend(os.read(self.master, 512))
            while len(buf) >= 4:
                n = 4+buf[3]
                if buf[:2] != bytes([0xF9, 0xFF]):
                    del buf[0]
                elif len(buf) < n:
                    break
                else:
                    self.process(buf[:n])
                    del buf[:n]

    def close(self):
        if not self.alive:
            return
        self.alive = False
        self.thread.join()
        os.close(self.master)
        os.close(self.slave)


def simulated_profile():
    return dict(limits_verified=True, interpolation_verified=True, control_frequency_hz=40,
                feedback_frequency_hz=40, health_frequency_hz=10, open_position_deg=0.0,
                close_limit_deg=10.0, close_velocity_deg_s=15.0, close_acceleration_deg_s2=200.0,
                max_following_error_deg=1.0, pwm_free=20, pwm_contact_threshold=100,
                pwm_contact_delta_threshold=70, pwm_close_limit=400, pwm_hold_limit=200,
                pwm_hard_limit=600, current_contact_ma=50, current_close_limit_ma=300,
                current_hold_limit_ma=120, current_hard_limit_ma=500, temperature_warning_c=48,
                temperature_stop_c=60, contact_error_deg=0.3, contact_min_fraction=0.1,
                contact_max_fraction=0.9, contact_confirm_ms=60, startup_guard_ms=30, filter_ms=6,
                feedback_timeout_ms=100, motion_timeout_ms=4000, max_close_duration_ms=3000,
                max_hold_duration_ms=3000, hold_overload_duration_ms=100, hold_drift_deg=1.0,
                serial_response_timeout_ms=35)


def run(binary, directory, devices, verified):
    config = {"trace_hex": True, "allow_raw_write": True, "state_rate_hz": 10.0}
    for name, device in zip(("left", "right"), devices):
        safety = simulated_profile()
        safety["limits_verified"] = verified
        alias=Path(directory,name)
        alias.unlink(missing_ok=True)
        alias.symlink_to(device.path)
        config[name] = dict(port_type="serial", serial_device=str(alias), servo_id=1, gap_ms=0,
                            tolerance_deg=0.2, safety=safety)
    params = Path(directory, "bench.yaml")
    params.write_text(yaml.safe_dump({"ds485_node": {"ros__parameters": config}}))
    log = tempfile.TemporaryFile(mode="w+")
    command=[binary,"--ros-args","--params-file",str(params),"--log-level","ds485_node:=debug"]
    if not verified:
        # 明确使用台架配置，避免现场 YAML 覆盖本用例的未标定标志。
        command=["ros2","launch","ds485_driver","ds485_driver.launch.py","config:="+str(params),"safety_config:="+str(params),"log_level:=debug"]
    process=subprocess.Popen(command, stdout=log, stderr=log)
    node = rclpy.create_node("pty_client")
    reports, health = {}, {}
    node.create_subscription(GripperResult, "ds485/gripper/result",
                             lambda m: reports.update({(m.id, m.side): m}), 10)
    node.create_subscription(GripperHealth, "ds485/gripper/health",
                             lambda m: health.update({m.side: m}), 20)
    pub = node.create_publisher(GripperCommand, "ds485/gripper/command", 10)
    stop = node.create_publisher(GripperStop, "ds485/gripper/stop", 10)

    def until(pred, timeout=5):
        end = time.monotonic()+timeout
        while not pred():
            assert process.poll() is None, "控制节点提前退出"
            assert time.monotonic()<end, (reports, health)
            rclpy.spin_once(node, timeout_sec=.014)

    def service(kind, name, request, wait=True):
        client = node.create_client(kind, "ds485/"+name)
        assert client.wait_for_service(timeout_sec=2)
        f = client.call_async(request)
        if not wait:
            return f
        until(f.done)
        return f.result()

    def goal(serial, side, pos):
        pub.publish(GripperCommand(id=serial, side=side, position=pos))

    def result(serial, side):
        until(lambda: (serial, side) in reports)
        return reports[serial, side]

    try:
        until(lambda: pub.get_subscription_count() and all(i in health and health[i].feedback_valid for i in (0,1)))
        assert health[0].actual_deg==0 and health[1].actual_deg==0
        if not verified:
            goal(1, 2, 255)
            for side in (0, 1):
                assert "CALIBRATION_REQUIRED" in result(1, side).message
            assert all(not dev.targets for dev in devices)
            return
        goal(2, 2, 128)
        for side in (0, 1):
            out = result(2, side)
            assert out.message == "ARRIVED", out
            assert len(devices[side].targets)>6
            assert all(abs(b[1]-a[1])<=5 for a,b in zip(devices[side].targets, devices[side].targets[1:]))
            gaps = [1000*(b[0]-a[0]) for a,b in zip(devices[side].targets, devices[side].targets[1:])]
            print(f"PTY side={side} frames={len(gaps)+1} gap_min_ms={min(gaps):.1f} "
                  f"gap_p50_ms={statistics.median(gaps):.1f} gap_max_ms={max(gaps):.1f}")
        req = ServoReadRegister.Request(side=1, address=0x32, length=1)
        assert list(service(ServoReadRegister,"servo/read_register",req).data)==[15]
        wr = ServoWriteRegister.Request(side=0,address=0x66,data=[100,0,2,0])
        assert not service(ServoWriteRegister,"servo/write_register",wr).success
        moving = ServoMoveAngle.Request(side=1,angle_deg=1.0,speed_dps=8,wait=True)
        done = service(ServoMoveAngle,"servo/move_angle",moving)
        assert done.success and abs(done.final_angle_deg-1.)<=.2
        goal(3,1,255)
        assert result(3,1).message=="EMPTY_CLOSED"
        devices[0].contact_at=58
        goal(4,0,255)
        assert result(4,0).message=="GRASPED"
        frozen=len(devices[0].targets)
        until(lambda: health[0].phase=="HOLDING")
        busy=ServoReadRegister.Request(side=0,address=0x32,length=1)
        assert service(ServoReadRegister,"servo/read_register",busy).message=="BUSY"
        assert devices[0].reg[0x5B]==u16(200) and devices[0].reg[0x5C]==u16(120)
        devices[0].temp=65
        until(lambda: health[0].phase=="FAULT")
        assert health[0].stop_confirmed and health[0].has_command_id and health[0].command_id==4
        assert len(devices[0].targets)==frozen
        goal(5,0,255)
        assert not result(5,0).success
        devices[0].temp=25
        devices[0].contact_at=None
        assert service(Trigger,"gripper/left/recover",Trigger.Request()).success
        assert len(devices[0].targets)==frozen
        goal(6,0,0)
        until(lambda: len(devices[0].targets)>frozen+1)
        goal(7,0,255)
        assert result(6,0).result==GripperResult.RESULT_PREEMPTED
        stop.publish(GripperStop(side=0))
        assert result(7,0).result==GripperResult.RESULT_CANCELED
        n=len(devices[1].targets)
        goal(8,1,0)
        until(lambda: len(devices[1].targets)>n+1)
        devices[1].blackout=True
        assert "STOP_UNCONFIRMED" in result(8,1).message
        until(lambda: health[1].phase=="FAULT" and not health[1].stop_confirmed)
        n=len(devices[1].targets)
        devices[1].blackout=False
        until(lambda: health[1].feedback_valid)
        assert health[1].phase=="FAULT" and len(devices[1].targets)==n
        assert all(addr!=0x66 for dev in devices for addr, _ in dev.writes)
        devices[1].close()
        until(lambda: not health[1].feedback_valid)
        devices[1]=Device(False)
        alias=Path(directory,"right")
        alias.unlink()
        alias.symlink_to(devices[1].path)
        assert service(Trigger,"gripper/right/recover",Trigger.Request()).success
        assert not devices[1].targets
        devices[1].ignore_targets=True
        no_wait=ServoMoveAngle.Request(side=1,angle_deg=8.0,speed_dps=8,wait=False)
        assert service(ServoMoveAngle,"servo/move_angle",no_wait).message=="ACCEPTED"
        until(lambda: "TARGET_UNCONFIRMED" in health[1].reason)
        assert health[1].stop_confirmed
        print("PTY 双侧、无写 ACK、空抓、接触保持、过温、抢占、取消及失联恢复验证通过")
    except BaseException:
        log.seek(0)
        print("".join(log.readlines()[-35:]), file=sys.stderr)
        raise
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(4)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        log.close()


if __name__=="__main__":
    os.environ["ROS_LOCALHOST_ONLY"]="1"
    os.environ["ROS_DOMAIN_ID"]=str(105+os.getpid()%70)
    os.environ["RMW_IMPLEMENTATION"]="rmw_fastrtps_cpp"
    with tempfile.TemporaryDirectory() as directory:
        os.environ["ROS_LOG_DIR"]=directory
        rclpy.init(args=[])
        try:
            for calibrated in (False, True):
                devices=[Device(True), Device(False)]
                try:
                    run(sys.argv[1], directory, devices, calibrated)
                finally:
                    for device in devices:
                        device.close()
        finally:
            rclpy.shutdown()

