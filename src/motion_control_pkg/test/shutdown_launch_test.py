#!/usr/bin/env python3
"""Exercise real bringup escalation settings with a harmless ROS timer fixture."""
import importlib.util
import pathlib
import sys
from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

fixture, launch_path = sys.argv[1:]
spec = importlib.util.spec_from_file_location('bringup_under_test', launch_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
results = []
processes = []
def node_fixture(**kwargs):
    process = ExecuteProcess(
        cmd=[fixture, '--hold'], name='offline_shutdown_fixture', output='screen',
        sigterm_timeout=kwargs['sigterm_timeout'], sigkill_timeout=kwargs['sigkill_timeout'])
    processes.append(process)
    return process
module.Node = node_fixture
launch_description = module.generate_launch_description()
assert len(processes) == 1
launch_description.add_action(RegisterEventHandler(OnProcessExit(
    target_action=processes[0], on_exit=lambda event, context: results.append(event.returncode))))
launch_description.add_action(TimerAction(period=1.5, actions=[EmitEvent(event=Shutdown(reason='offline Ctrl+C equivalent'))]))
service = LaunchService(noninteractive=True)
service.include_launch_description(launch_description)
rc = service.run()
assert rc == 0 and results == [0], (rc, results)
print('PASS_LAUNCH_NO_FORCED_KILL: feedback fixture survived beyond default 10 s')
