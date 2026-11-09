import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

build=Path(sys.argv[1]).resolve();source=Path(sys.argv[2]).resolve()
commands=json.loads((build/'compile_commands.json').read_text())
command=shlex.split(next(c['command'] for c in commands if c['file'].endswith('/task_action_client.cpp')))
command[command.index('-o')+1]=str(build/'ik_leg_recovery_test.o')
command[-1]=str(source/'test/test_ik_leg_recovery.cpp')
subprocess.run(command,check=True,cwd=build)
link=shlex.split((build/'CMakeFiles/task_manager_node.dir/link.txt').read_text())
link[link.index('-o')+1]=str(build/'test_ik_leg_recovery')
link=[a for a in link if not a.endswith('/task_manager_node.cpp.o') and not a.endswith('/task_manager_legacy_safety.cpp.o')]
link.insert(1,str(build/'ik_leg_recovery_test.o'))
subprocess.run(link,check=True,cwd=build)
cache=(build/'CMakeCache.txt').read_text().splitlines()
interface=Path(next(line.split('=',1)[1] for line in cache if line.startswith('com_interfaces_DIR:'))).parents[2]
env=dict(os.environ,ROS_DOMAIN_ID='221',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
env['LD_LIBRARY_PATH']=str(interface/'lib')+os.pathsep+env.get('LD_LIBRARY_PATH','')
env['AMENT_PREFIX_PATH']=str(interface)+os.pathsep+env.get('AMENT_PREFIX_PATH','')
subprocess.run([str(build/'test_ik_leg_recovery'),str(source/'config/tasks/laptop_assembly_two_piece.yaml')],
    env=env,cwd=build,check=True,timeout=120)
