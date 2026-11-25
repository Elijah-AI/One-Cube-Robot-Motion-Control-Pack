"""一条 PTY 模拟同一总线上的多台舵机，经真实 ROS 服务验证一侧多台恒定 PWM：
0x83 一帧同时启动、0x04/0x84 预存后广播、逐台写、启动确认、同步开关的单播补发、
单台故障整侧停止、按 servo_id 的维护服务、退出制动，以及多台配置的启动检查。"""
import os
import pty
import select
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time
import tty
from pathlib import Path

import rclpy
import rclpy.executors
import yaml
from ds485_interfaces.msg import GripperHealth, ServoState
from ds485_interfaces.srv import ServoReadRegister, ServoSetLimits, ServoSetSwitch, ServoSetTorque

MULTI_WRITE, STAGE, APPLY, BROADCAST = 0x83, 0x04, 0x84, 0xFE


def u16(value):
    return struct.pack("<H", value)


def i16(value):
    return struct.pack("<h", value)


class Servo:
    """总线上的一台从站。寄存器初值与 test_ros_pty.Device 相同。"""

    def __init__(self, bus, sid, ack):
        self.bus, self.id, self.ack = bus, sid, ack
        self.pwm = 0
        self.fault = 0
        self.sequence = 0
        self.silent = False      # 不执行也不应答任何帧
        self.deaf_sync = False   # 收不到广播同步帧，单播正常
        self.staged = None
        self.commands = []       # (总线帧序号, 时刻, pwm, 功能码)
        self.reg = {0x04: b"\0", 0x11: bytes([8 if ack else 0]), 0x13: u16(1000),
                    0x14: u16(1000), 0x19: i16(-1800), 0x1A: u16(1800),
                    0x1C: u16(800), 0x32: b"\x0f", 0x33: b"\0", 0x34: b"T",
                    0x3D: bytes([65, 5]), 0x3E: u16(900) + bytes([3]), 0x3F: u16(700) + bytes([20]),
                    0x5A: b"\0", 0x5B: u16(1000), 0x5C: u16(1000), 0x5D: u16(200), 0x64: b"\1"}

    def data(self, addr):
        values = {0x46: ("i", 0), 0x47: ("h", 0), 0x48: ("h", 5), 0x49: ("h", self.pwm),
                  0x4A: ("h", 25), 0x4B: ("h", 24000), 0x4C: ("i", 0)}
        if addr in values:
            fmt, value = values[addr]
            return struct.pack("<" + fmt, value)
        return self.reg[addr]

    def apply(self, addr, data, cmd):
        """普通写、0x83 与 0x84 的执行效果相同。"""
        data = bytes(data)
        if addr == 0x6E:
            pwm, = struct.unpack("<h", data)
            if pwm and self.reg[0x5A] != b"\x02":
                self.bus.errors.append(f"ID {self.id} 未进入电机模式即收到 PWM {pwm}")
            self.pwm = pwm
            self.commands.append((self.bus.count, time.monotonic(), pwm, cmd))
        elif addr == 0x64:
            if data == b"\x02" and self.reg[0x5A] == b"\x02":
                self.bus.errors.append(f"ID {self.id} 电机模式固件拒绝位置保持")
            if data in (b"\x00", b"\x01"):
                self.pwm = 0
        self.reg[addr] = data

    def reply(self, body):
        os.write(self.bus.master, bytes([0xF9, 0xF5]) + body + bytes([~sum(body) & 255]))

    def unicast(self, frame):
        cmd, addr, data = frame[4], frame[5], frame[6:-1]
        if self.silent:
            return
        if cmd == STAGE:
            self.staged = (addr, bytes(data))
            return
        # 0x84 之前的其他指令清除预存
        self.staged = None
        if cmd == 2:
            if addr == 1:
                self.sequence += 1
                payload = bytes([self.fault, self.sequence % 256])
            elif addr == 0x59:
                payload = b"".join(self.data(a) for a in data)
            else:
                payload = self.data(addr)
            self.reply(bytes([self.id, 3 + len(payload), cmd, addr]) + payload)
        elif cmd == 3:
            self.apply(addr, data, cmd)
            if self.ack:
                self.reply(bytes([self.id, 2, 0]))
        else:
            self.bus.errors.append(f"ID {self.id} 未模拟的功能码 {frame.hex(' ')}")


class Bus:
    """一条 RS485 总线：按帧中 ID 分发，广播同步帧由各台按协议同时执行且不应答。"""

    def __init__(self, ids, ack):
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave)
        tty.setraw(self.slave)
        self.errors = []
        self.frames = []
        self.count = 0
        self.servos = {sid: Servo(self, sid, ack) for sid in ids}
        self.alive = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def __getitem__(self, sid):
        return self.servos[sid]

    def sent(self, cmd, addr=None):
        return [f for f in self.frames if f[4] == cmd and (addr is None or f[5] == addr)]

    def writes(self, sid, addr):
        return [f for f in self.frames if f[2] == sid and f[4] == 3 and f[5] == addr]

    def process(self, frame):
        frame = bytes(frame)
        if sum(frame[2:]) & 255 != 255:
            self.errors.append("校验和错误 " + frame.hex(" "))
            return
        self.count += 1
        self.frames.append(frame)
        sid, cmd = frame[2], frame[4]
        if sid != BROADCAST:
            if sid in self.servos:
                self.servos[sid].unicast(frame)
            return
        listeners = [s for s in self.servos.values() if not s.silent and not s.deaf_sync]
        if cmd == MULTI_WRITE:
            addr, body = frame[5], frame[6:-1]
            width, items = body[0], body[1:]
            if not width or len(items) % (width + 1):
                self.errors.append("0x83 长度错误 " + frame.hex(" "))
                return
            for k in range(0, len(items), width + 1):
                servo = self.servos.get(items[k])
                if servo in listeners:
                    servo.staged = None
                    servo.apply(addr, items[k + 1:k + 1 + width], cmd)
        elif cmd == APPLY:
            for servo in listeners:
                if servo.staged:
                    servo.apply(*servo.staged, cmd)
                servo.staged = None
        else:
            self.errors.append("未模拟的广播 " + frame.hex(" "))

    def loop(self):
        buf = bytearray()
        while self.alive:
            if not select.select([self.master], [], [], .1)[0]:
                continue
            buf.extend(os.read(self.master, 512))
            while len(buf) >= 4:
                n = 4 + buf[3]
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


def side(bus, **extra):
    config = dict(port_type="serial", serial_device=bus.path, gap_ms=2, timeout_ms=50)
    config.update(extra)
    return config


def write_params(directory, name, params):
    path = Path(directory, name + ".yaml")
    path.write_text(yaml.safe_dump({"ds485_node": {"ros__parameters": params}}, allow_unicode=True))
    return path


class Bench:
    """一个 ds485_node 进程。每个实例用独立 ROS 域，避免上一进程的服务发现残留。"""

    def __init__(self, binary, directory, name, params, domain):
        path = write_params(directory, name, params)
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
        self.log = tempfile.TemporaryFile(mode="w+")
        self.process = subprocess.Popen([binary, "--ros-args", "--params-file", str(path)],
                                        stdout=self.log, stderr=self.log, env=env)
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=domain)
        self.node = rclpy.create_node("multi_bench_" + name, context=self.context)
        self.executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.set_torque = self.node.create_client(ServoSetTorque, "/ds485/servo/set_torque")
        self.limits = self.node.create_client(ServoSetLimits, "/ds485/servo/set_limits")
        self.switch = self.node.create_client(ServoSetSwitch, "/ds485/servo/set_switch")
        self.read = self.node.create_client(ServoReadRegister, "/ds485/servo/read_register")
        self.health = {}
        self.states = set()
        self.node.create_subscription(GripperHealth, "/ds485/gripper/health",
                                      lambda m: self.health.update({(m.side, m.servo_id): m}), 50)
        self.node.create_subscription(ServoState, "/ds485/state",
                                      lambda m: self.states.add((m.side, m.servo_id)), 50)

    def until(self, condition, timeout=6):
        deadline = time.monotonic() + timeout
        while not condition():
            assert self.process.poll() is None, "节点提前退出"
            assert time.monotonic() < deadline, {k: (v.phase, v.reason) for k, v in self.health.items()}
            self.executor.spin_once(timeout_sec=.01)

    def call(self, client, request):
        assert client.wait_for_service(timeout_sec=5)
        future = client.call_async(request)
        self.until(future.done)
        return future.result()

    def torque(self, side_index, pwm, ms=0):
        return self.call(self.set_torque, ServoSetTorque.Request(side=side_index, pwm=pwm, duration_ms=ms))

    def ready(self, keys):
        self.until(lambda: all(k in self.health and self.health[k].feedback_valid for k in keys))
        self.until(lambda: set(keys) <= self.states)

    def dump(self):
        self.log.seek(0)
        print("".join(self.log.readlines()[-40:]), file=sys.stderr)

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=5)
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)
        self.log.close()


def synchronized(binary, directory, domain):
    """左侧 0x83（第二台反向安装），右侧 0x04/0x84 且写指令无应答。"""
    left, right = Bus([1, 2], ack=True), Bus([1, 2], ack=False)
    params = {"control_strategy": "torque", "state_rate_hz": 40.0,
              "left": side(left, servo_ids=[1, 2], servo_directions=[1, -1], sync_write="multi_id",
                           start_confirm_ms=200),
              "right": side(right, servo_ids=[1, 2], sync_write="staged", start_confirm_ms=200)}
    bench = Bench(binary, directory, "sync", params, domain)
    try:
        bench.ready([(0, 1), (0, 2), (1, 1), (1, 2)])
        assert not any(f[4] != 2 for bus in (left, right) for f in bus.frames), "启动阶段只允许读"

        # 0x83：电机模式逐台切换并读回后，一帧携带两台各自方向的 PWM
        r = bench.torque(0, 300)
        assert r.success and r.sent and not r.acknowledged, r
        bench.until(lambda: left[1].pwm == 300 and left[2].pwm == -300)
        frames = left.sent(MULTI_WRITE, 0x6E)
        assert len(frames) == 1, frames
        assert frames[0][6:-1] == bytes([2, 1]) + i16(300) + bytes([2]) + i16(-300)
        assert left[1].commands[-1][0] == left[2].commands[-1][0], "两台须由同一帧启动"
        assert not left.writes(1, 0x6E) and not left.writes(2, 0x6E)
        start = left.frames.index(frames[0])
        for sid in (1, 2):
            assert any(f[2] == sid and f[4] == 3 and f[5] == 0x5A and f[6:-1] == b"\x02" for f in left.frames[:start])
        bench.until(lambda: bench.health[(0, 1)].reason == "PWM_START_CONFIRMED")

        r = bench.torque(0, 0)
        assert r.success and r.stop_confirmed, r
        for s in left.servos.values():
            assert s.pwm == 0 and s.reg[0x64] == b"\x01" and s.reg[0x5A] == b"\x00", s.id
        assert left.sent(MULTI_WRITE, 0x6E)[-1][6:-1] == bytes([2, 1, 0, 0, 2, 0, 0])
        assert left.sent(MULTI_WRITE, 0x64)[-1][6:-1] == bytes([1, 1, 1, 2, 1])
        # 同步帧之后逐台单播补零输出和制动，不依赖开关读回推断是否收到同步帧
        for sid in (1, 2):
            assert [f[6:-1] for f in left.writes(sid, 0x6E)] == [i16(0)], sid
            assert [f[6:-1] for f in left.writes(sid, 0x64)] == [b"\x01"], sid
        bench.until(lambda: all(bench.health[(0, i)].stop_confirmed for i in (1, 2)))

        # 0x04 逐台预存 + 0x84 广播执行；脉冲结束的零输出同样同步
        r = bench.torque(1, 200, 300)
        assert r.success and r.sent and not r.acknowledged and r.stop_confirmed, r
        assert "START_UNVERIFIED" not in r.message, r
        assert [f[2] for f in right.sent(STAGE, 0x6E)[:2]] == [1, 2]
        assert len(right.sent(APPLY)) >= 2
        for s in right.servos.values():
            assert [(c[2], c[3]) for c in s.commands] == [(200, APPLY), (0, APPLY), (0, 3)], s.commands
            assert s.reg[0x64] == b"\x01" and s.reg[0x5A] == b"\x00"
        assert right[1].commands[0][0] == right[2].commands[0][0]
        interval = 1000 * (right[1].commands[1][1] - right[1].commands[0][1])
        assert 260 < interval < 400, interval

        # 第二台收不到同步帧：确认窗内读不到其输出，整侧停止
        left[2].deaf_sync = True
        heard1, heard2 = len(left[1].commands), len(left[2].commands)
        r = bench.torque(0, 250)
        assert r.success and r.sent, r
        bench.until(lambda: "START_UNCONFIRMED ID 2" in bench.health[(0, 1)].reason and
                    all(bench.health[(0, i)].stop_confirmed for i in (1, 2)))
        assert [(c[2], c[3]) for c in left[1].commands[heard1:]] == [(250, MULTI_WRITE), (0, MULTI_WRITE), (0, 3)]
        assert [(c[2], c[3]) for c in left[2].commands[heard2:]] == [(0, 3)], left[2].commands
        assert left[1].pwm == 0 and left[2].pwm == 0 and left[2].reg[0x5A] == b"\x00"

        # 收不到同步帧的一台仍由单播完成开关切换，并逐台读回
        before = len(left.frames)
        r = bench.call(bench.switch, ServoSetSwitch.Request(side=0, mode=0))
        assert r.success and r.message == "SWITCH_VERIFIED", r
        assert left[1].reg[0x64] == b"\x00" and left[2].reg[0x64] == b"\x00"
        assert [f[6:-1] for f in left.frames[before:] if f[4] == MULTI_WRITE and f[5] == 0x64] == \
            [bytes([1, 1, 1, 2, 1]), bytes([1, 1, 0, 2, 0])]
        for sid in (1, 2):
            assert [f[6:-1] for f in left.frames[before:] if f[2] == sid and f[4] == 3 and f[5] == 0x64] == \
                [b"\x01", b"\x00"], sid
        left[2].deaf_sync = False

        # 已确认输出的一台掉出输出（复位或漏收），连续两轮读不到后整侧停止
        r = bench.torque(0, 130)
        assert r.success, r
        bench.until(lambda: bench.health[(0, 1)].reason == "PWM_START_CONFIRMED")
        left[2].pwm = 0
        bench.until(lambda: "OUTPUT_LOST ID 2" in bench.health[(0, 1)].reason and
                    all(bench.health[(0, i)].stop_confirmed for i in (1, 2)))
        assert left[1].pwm == 0 and all(s.reg[0x5A] == b"\x00" for s in left.servos.values())

        # 任一台保护：整侧停止，故障期间拒绝新输出
        r = bench.torque(0, 150)
        assert r.success, r
        bench.until(lambda: bench.health[(0, 1)].reason == "PWM_START_CONFIRMED")
        assert left[1].pwm == 150 and left[2].pwm == -150
        left[2].fault = 4
        bench.until(lambda: bench.health[(0, 1)].phase == "TORQUE_FAULT" and
                    all(bench.health[(0, i)].stop_confirmed for i in (1, 2)))
        assert "SERVO_FAULT ID 2" in bench.health[(0, 1)].reason, bench.health[(0, 1)].reason
        assert left[1].pwm == 0 and left[2].pwm == 0
        r = bench.torque(0, 140)
        assert not r.success and "SERVO_FAULT ID 2" in r.message, r
        assert not any(abs(c[2]) == 140 for s in left.servos.values() for c in s.commands)
        left[2].fault = 0
        bench.until(lambda: bench.health[(0, 1)].phase == "TORQUE_IDLE")

        # 维护服务：servo_id=-1 时限幅作用于整侧，指定 ID 只作用于该台
        def limits(speed, **extra):
            return bench.call(bench.limits, ServoSetLimits.Request(
                side=0, pwm_limit=-1, current_limit=-1, speed_limit=speed, **extra))
        assert limits(123).success
        assert left[1].reg[0x5D] == u16(123) and left[2].reg[0x5D] == u16(123)
        assert limits(77, servo_id=2).success
        assert left[1].reg[0x5D] == u16(123) and left[2].reg[0x5D] == u16(77)
        assert limits(50, servo_id=9).message == "INVALID_SERVO_ID"
        r = bench.call(bench.read, ServoReadRegister.Request(side=0, address=0x5D, length=2, servo_id=2))
        assert r.success and bytes(r.data) == u16(77), r
        r = bench.call(bench.read, ServoReadRegister.Request(side=0, address=0x5D, length=2))
        assert r.success and bytes(r.data) == u16(123), r

        # 退出时整侧制动并恢复模式
        assert bench.torque(0, 120).success
        bench.until(lambda: left[1].pwm == 120 and left[2].pwm == -120)
        bench.process.send_signal(signal.SIGINT)
        assert bench.process.wait(timeout=5) == 0
        for bus in (left, right):
            for s in bus.servos.values():
                assert s.pwm == 0 and s.reg[0x64] == b"\x01" and s.reg[0x5A] == b"\x00", s.id
            assert all(f[5] in (0x5A, 0x5D, 0x64, 0x6E) for f in bus.frames if f[4] in (3, STAGE, MULTI_WRITE))
            assert not bus.errors, bus.errors
        return interval
    except BaseException:
        bench.dump()
        raise
    finally:
        bench.close()
        left.close()
        right.close()


def sequential(binary, directory, domain):
    """左侧逐台写且校验应答；右侧由 servo_count 从 servo_id=5 连续编号。"""
    left, right = Bus([1, 2], ack=True), Bus([5, 6], ack=True)
    params = {"control_strategy": "torque", "state_rate_hz": 40.0,
              "left": side(left, servo_ids=[1, 2], sync_write="sequential"),
              "right": side(right, servo_id=5, servo_count=2)}
    bench = Bench(binary, directory, "sequential", params, domain)
    try:
        bench.ready([(0, 1), (0, 2), (1, 5), (1, 6)])
        r = bench.torque(0, 100, 200)
        assert r.success and r.acknowledged and r.stop_confirmed, r
        assert not left.sent(MULTI_WRITE) and not left.sent(STAGE) and not left.sent(APPLY)
        nonzero = [f[2] for f in left.frames if f[4] == 3 and f[5] == 0x6E and f[6:-1] != b"\0\0"]
        assert nonzero == [1, 2], nonzero
        # 逐台写停止时一台失联：其余舵机仍收到零输出和制动，整侧报告未确认；恢复后再次停止完成
        r = bench.torque(0, 70)
        assert r.success and r.acknowledged, r
        bench.until(lambda: left[1].pwm == 70 and left[2].pwm == 70)
        left[1].silent = True
        r = bench.torque(0, 0)
        assert not r.success and not r.stop_confirmed, r
        assert left[2].pwm == 0 and left[2].reg[0x64] == b"\x01", "一台失联时另一台仍须收到零输出和制动"
        assert left[1].pwm == 70
        left[1].silent = False
        r = bench.torque(0, 0)
        assert r.success and r.stop_confirmed, r
        assert left[1].pwm == 0 and all(s.reg[0x5A] == b"\x00" for s in left.servos.values())

        r = bench.torque(1, -90)
        assert r.success and not r.acknowledged, r
        bench.until(lambda: right[5].pwm == -90 and right[6].pwm == -90)
        assert right[5].commands[-1][0] == right[6].commands[-1][0]
        r = bench.torque(1, 0)
        assert r.success and r.stop_confirmed, r
        for bus in (left, right):
            assert not bus.errors, bus.errors
    except BaseException:
        bench.dump()
        raise
    finally:
        bench.close()
        left.close()
        right.close()


def rejected(binary, directory, domain):
    """多台配置错误时节点在访问总线前退出。"""
    bus = Bus([1, 2], ack=True)
    cases = [
        ("position", dict(servo_ids=[1, 2]), "position 模式每侧只支持 1 台舵机"),
        ("torque", dict(servo_ids=[1, 1]), "互不重复"),
        ("torque", dict(servo_ids=[1, 2], servo_directions=[1]), "servo_directions 须与舵机数量相同"),
        ("torque", dict(servo_ids=[1, 2], servo_directions=[1, 0]), "只能填 1 或 -1"),
        ("torque", dict(servo_ids=[1, 2], sync_write="fast"), "sync_write 只能是"),
        ("torque", dict(servo_ids=[1, 2], servo_count=3), "不一致"),
        ("torque", dict(servo_count=9), "servo_count 范围"),
        ("torque", dict(servo_ids=[1, 2], start_confirm_ms=0), "start_confirm_ms 不能小于"),
        ("torque", dict(servo_ids=[1, 2], sync_write="staged", start_confirm_ms=20), "start_confirm_ms 不能小于"),
        ("torque", dict(servo_ids=[1, 2], gap_ms=1), "gap_ms 不能小于"),
    ]
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    try:
        for i, (strategy, extra, expected) in enumerate(cases):
            params = {"control_strategy": strategy, "left": side(bus, **extra), "right": {"enabled": False}}
            path = write_params(directory, f"bad{i}", params)
            done = subprocess.run([binary, "--ros-args", "--params-file", str(path)], env=env,
                                  capture_output=True, encoding="utf-8", errors="replace", timeout=10)
            output = done.stdout + done.stderr
            assert done.returncode == 1 and expected in output, (extra, done.returncode, output[-600:])
        assert not bus.frames and not bus.errors
    finally:
        bus.close()


if __name__ == "__main__":
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    base = 175 + (os.getpid() % 18) * 3
    with tempfile.TemporaryDirectory() as directory:
        os.environ["ROS_LOG_DIR"] = directory
        rejected(sys.argv[1], directory, base)
        pulse = synchronized(sys.argv[1], directory, base + 1)
        sequential(sys.argv[1], directory, base + 2)
    print(f"一侧多台：0x83 同帧启动、0x04/0x84、逐台写、启动确认、开关补发、故障联停、退出制动及配置检查通过，"
          f"300 ms 同步脉冲实测 {pulse:.1f} ms")
