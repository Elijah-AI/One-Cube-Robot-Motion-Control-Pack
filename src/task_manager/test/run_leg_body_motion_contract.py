"""Compile the contract test against the production objects already built by CMake.

No source implementation is duplicated or substituted. Robot/task main functions
are excluded; the executable contains only an isolated ROS Action capture server.
"""
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

build = Path(sys.argv[1]).resolve()
source = Path(sys.argv[2]).resolve()
commands = json.loads((build / 'compile_commands.json').read_text())
command = shlex.split(next(c['command'] for c in commands if c['file'].endswith('/task_action_client.cpp')))
command[command.index('-o') + 1] = str(build / 'leg_body_motion_contract.o')
command[-1] = str(source / 'test/test_leg_body_motion_contract.cpp')
subprocess.run(command, check=True, cwd=build)
link = shlex.split((build / 'CMakeFiles/task_manager_node.dir/link.txt').read_text())
link[link.index('-o') + 1] = str(build / 'test_leg_body_motion_contract')
link = [a for a in link if not a.endswith('/task_manager_node.cpp.o') and not a.endswith('/task_manager_legacy_safety.cpp.o')]
link.insert(1, str(build / 'leg_body_motion_contract.o'))
subprocess.run(link, check=True, cwd=build)
env = dict(os.environ, ROS_DOMAIN_ID='222', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
# ROS loads the selected middleware type support with dlopen by basename; use
# the exact interface prefix CMake selected, including with a bare ctest shell.
cache = (build / 'CMakeCache.txt').read_text().splitlines()
interface_dir = next(line.split('=', 1)[1] for line in cache if line.startswith('com_interfaces_DIR:'))
interface_prefix = Path(interface_dir).parents[2]
env['LD_LIBRARY_PATH'] = str(interface_prefix / 'lib') + os.pathsep + env.get('LD_LIBRARY_PATH', '')
env['AMENT_PREFIX_PATH'] = str(interface_prefix) + os.pathsep + env.get('AMENT_PREFIX_PATH', '')
subprocess.run([str(build / 'test_leg_body_motion_contract'), str(build / 'leg_body_motion_parser_case.yaml')],
               env=env, cwd=build, check=True, timeout=30)
