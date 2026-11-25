"""Exercise the GX12-01 profile with the embedded driver on a simulated right bus."""
import os
import sys
import tempfile
from pathlib import Path

import yaml
from std_srvs.srv import SetBool
from test_multi_torque_pty import Bench, Bus


def run(binary, config_path, directory):
    params = yaml.safe_load(Path(config_path).read_text())["ds485_node"]["ros__parameters"]
    bus = Bus([1, 2], ack=True)
    params["right"].update(port_type="serial", serial_device=bus.path)
    bench = Bench(binary, directory, "embedded", params, 150 + os.getpid() % 20)
    try:
        bench.ready([(1, 1), (1, 2)])
        assert not bench.torque(0, 100).success
        assert bench.torque(1, 100).message == "HOST_DENIED"
        gate = bench.node.create_client(SetBool, "/test_gripper_admit")
        assert bench.call(gate, SetBool.Request(data=True)).success
        result = bench.torque(1, 100)
        assert result.success and result.sent and not result.acknowledged, result
        bench.until(lambda: bench.health[(1, 1)].reason == "PWM_START_CONFIRMED")
        assert (bus[1].pwm, bus[2].pwm) == (-100, 100)
        assert bench.call(gate, SetBool.Request(data=False)).success
        bench.until(lambda: all(bench.health[(1, sid)].stop_confirmed for sid in (1, 2)))
        assert bus[1].pwm == bus[2].pwm == 0
        assert bench.torque(1, 0).stop_confirmed
    finally:
        bench.close()
        bus.close()


if __name__ == "__main__":
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    with tempfile.TemporaryDirectory() as directory:
        os.environ["ROS_LOG_DIR"] = directory
        run(sys.argv[1], sys.argv[2], directory)
