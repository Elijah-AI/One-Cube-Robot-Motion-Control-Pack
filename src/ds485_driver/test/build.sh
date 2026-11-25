#!/bin/sh
# 不用 ROS 时的离线测试一键编译 + 运行（只测第 2、3 层，不需要机械臂/舵机/MarvinSDK）。
# 需要 g++（Ubuntu: sudo apt install g++）。
#   sh test/build.sh
set -e
cd "$(dirname "$0")/.."

mkdir -p build_test
g++ -std=c++17 -pthread -Wall -Wextra -O0 -g \
    -Iinclude -Itest \
    test/test_bus_worker.cpp \
    test/test_trajectory.cpp \
    test/test_transaction.cpp \
    test/test_servo_group.cpp \
    test/test_ds485.cpp \
    test/fake_servo_port.cpp \
    src/servo/ds485_protocol.cpp \
    src/servo/serial_transaction.cpp \
    src/servo/ds485_protection.cpp \
    src/servo/ds485_servo.cpp \
    src/servo/ds485_servo_group.cpp \
    src/gripper/gripper_controller.cpp \
    src/rs485/bus_worker.cpp \
    src/gripper/ds485_gripper.cpp \
    -o build_test/ds485_offline_test

./build_test/ds485_offline_test
