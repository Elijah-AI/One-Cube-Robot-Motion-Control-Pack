#!/usr/bin/env bash
# Offline only: compile executables, run algorithm/Fake-device tests; never launch robot nodes.
set -eo pipefail
merge_repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
merge_build_root="${MERGE_BUILD_ROOT:-$merge_repo_root/build}"
merge_install_root="${MERGE_INSTALL_ROOT:-$merge_repo_root/install}"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ -n "${MERGE_DEPENDENCY_PREFIX:-}" ]]; then
    export CMAKE_PREFIX_PATH="$MERGE_DEPENDENCY_PREFIX:${CMAKE_PREFIX_PATH:-}"
fi
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=218
export ROS_LOG_DIR="$merge_build_root/ros-logs"
export TASK_MANAGER_REAL_CONFIG_PATH="$merge_repo_root/task_manager/config/task_config.yaml"
mkdir -p "$ROS_LOG_DIR"

for merge_package in com_interfaces onecube_robot_description motion_control_pkg task_manager robot_time_sync_node safety_monitor camera_driver; do
    mkdir -p "$merge_build_root/$merge_package"
    printf 'Building %s (one worker)\n' "$merge_package"
    cmake -S "$merge_repo_root/$merge_package" -B "$merge_build_root/$merge_package" \
        -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS_RELEASE='-O2 -DNDEBUG' \
        -DCMAKE_CXX_FLAGS= -DCMAKE_EXE_LINKER_FLAGS= \
        -DPython3_EXECUTABLE=/usr/bin/python3 \
        -DCMAKE_INSTALL_PREFIX="$merge_install_root/$merge_package" \
        > "$merge_build_root/$merge_package/configure.log" 2>&1
    cmake --build "$merge_build_root/$merge_package" --parallel 1 \
        > "$merge_build_root/$merge_package/build.log" 2>&1
    cmake --install "$merge_build_root/$merge_package" \
        > "$merge_build_root/$merge_package/install.log" 2>&1
    export CMAKE_PREFIX_PATH="$merge_install_root/$merge_package:${CMAKE_PREFIX_PATH:-}"
    source "$merge_install_root/$merge_package/share/$merge_package/local_setup.bash"
done

for merge_package in com_interfaces motion_control_pkg task_manager; do
    printf 'Testing %s (offline)\n' "$merge_package"
    ctest --test-dir "$merge_build_root/$merge_package" --output-on-failure \
        | tee "$merge_build_root/$merge_package/ctest.log"
done
colcon test-result --test-result-base "$merge_build_root" --verbose
