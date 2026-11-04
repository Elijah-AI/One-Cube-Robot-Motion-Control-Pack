#!/usr/bin/python3
"""Exercise command routing and atomic requests without ROS or hardware."""
import contextlib
import io
import runpy
import sys
import types
from unittest.mock import patch

cli = runpy.run_path(sys.argv[1])
calls = []


class Parameter:
    def __init__(self, name, value):
        self.name, self.value = name, value

    def to_parameter_msg(self):
        return self


class Node:
    def create_client(self, service_type, name):
        class Client:
            def wait_for_service(self, **kwargs):
                return True

            def call_async(self, request):
                calls.append((name, [(p.name, p.value) for p in request.parameters]))
                result = types.SimpleNamespace(successful=True, reason="accepted")
                return types.SimpleNamespace(done=lambda: True,
                    result=lambda: types.SimpleNamespace(result=result))
        return Client()

    def create_subscription(self, message_type, name, callback, qos):
        callback(types.SimpleNamespace(data="arm_error=OFF body_error=OFF"))
        return object()

    def destroy_subscription(self, subscription):
        pass

    def destroy_node(self):
        pass


modules = {
    "rclpy": types.SimpleNamespace(init=lambda **kw: None,
        create_node=lambda name: Node(), shutdown=lambda: None,
        spin_once=lambda *a, **kw: None,
        spin_until_future_complete=lambda *a, **kw: None),
    "rclpy.parameter": types.SimpleNamespace(Parameter=Parameter),
    "rclpy.qos": types.SimpleNamespace(QoSProfile=lambda **kw: kw,
        DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1),
        ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1)),
    "rcl_interfaces.srv": types.SimpleNamespace(SetParametersAtomically=
        types.SimpleNamespace(Request=types.SimpleNamespace)),
    "std_msgs.msg": types.SimpleNamespace(String=object),
}
groups = 0
with patch.dict(sys.modules, modules), contextlib.redirect_stdout(io.StringIO()):
    for command in ("arm_error", "body_error"):
        for args in (["0.02", "0.08"], ["on", "0.02"], ["0.03", "0.1"]):
            assert cli["main"](args, command) == 0
            expected_limit = float(args[0]) if args[0] != "on" else .02
            expected_hard = .1 if args[0] == "0.03" else .08
            assert calls[-1] == ("/Control/" + command,
                [("enabled", True), ("position_error_rad", expected_limit),
                 ("hard_error_rad", expected_hard)])
        groups += 1
        assert cli["main"](["off"], command) == 0
        assert calls[-1] == ("/Control/" + command, [("enabled", False)])
        groups += 1
        count = len(calls)
        assert cli["main"](["status"], command) == 0 and len(calls) == count
        groups += 1
        for args in (["nan", ".08"], [".02", "inf"], ["-.01"], [".09", ".08"], ["0"]):
            with contextlib.redirect_stderr(io.StringIO()):
                try:
                    cli["main"](args, command)
                except SystemExit as error:
                    assert error.code == 2
                else:
                    raise AssertionError("invalid limits accepted")
            assert len(calls) == count
        groups += 1
print("PASS error_option_cli groups=" + str(groups) + "; no ROS or hardware")
