#!/usr/bin/env python3
"""编译并运行离线测试，不加载 SDK 动态库，不连接机器人。"""
import argparse
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--build-dir', type=Path, required=True)
    args = parser.parse_args()
    source = args.source.resolve()
    build = args.build_dir.resolve()
    build.mkdir(parents=True, exist_ok=True)
    tests = Path(__file__).resolve().parent
    gripper = source / 'src/common/gripper'
    header = source / 'include/motion_control_pkg/gripper'
    text = (source / 'src/dual_arm_module.cpp').read_text()
    progress_start = text.index('float_t DualArmModule::get_progress_1000(')
    progress_end = text.index('void DualArmModule::control_gripper_result', progress_start)
    helper_start = text.index('static uint8_t ds485_pwm_to_255')
    helper_end = text.index('void DualArmModule::load_ds485_gripper_parameters', helper_start)
    action_start = text.index('void DualArmModule::execution_loop_control_ds485_gripper', helper_end)
    action_end = text.index('void DualArmModule::heart_beat_cb', action_start)
    extracted = text[progress_start:progress_end] + text[helper_start:helper_end] + text[action_start:action_end]
    (build / 'ds485_extracted_action.inc').write_text(extracted)
    common = ['c++', '-std=c++17', '-Wall', '-Wextra', '-pthread',
              '-I' + str(build), '-I' + str(tests), '-I' + str(header),
              '-I' + str(source / 'include'),
              '-I' + str(source / 'include/motion_control_pkg/MarvinSDK')]
    cases = {
        'protocol': [tests / 'test_ds485_gripper.cpp', tests / 'fake_ds485_servo.cpp'],
        'combined': [tests / 'test_combined.cpp'],
        'action': [tests / 'test_action.cpp', tests / 'fake_ds485_servo.cpp',
                   gripper / 'Ds485_Marvin_Transport.cpp'],
    }
    for name, files in cases.items():
        executable = build / ('test_' + name)
        command = common + [str(p) for p in files + [gripper / 'Ds485_Gripper.cpp']]
        command += ['-o', str(executable)]
        subprocess.run(command, check=True)
        result = subprocess.run([str(executable)], check=False, text=True, capture_output=True)
        (build / (name + '.log')).write_text(result.stdout + result.stderr)
        if result.returncode:
            print(result.stdout + result.stderr, flush=True)
            result.check_returncode()
        print(result.stdout.strip().splitlines()[-1], flush=True)


if __name__ == '__main__':
    main()
