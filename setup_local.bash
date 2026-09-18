# Source the independently installed packages after a native CMake build.
# This changes only the current shell environment; it starts no ROS nodes.
source /opt/ros/humble/setup.bash
_ylf_wbc_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -d "${_ylf_wbc_workspace}/install/native_deps/lib" ]; then
  export LD_LIBRARY_PATH="${_ylf_wbc_workspace}/install/native_deps/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
for _ylf_wbc_package in com_interfaces onecube_robot_description camera_driver motion_control_pkg safety_monitor task_manager vision_calibration vision_detection_pose_cloud; do
  _ylf_wbc_setup="${_ylf_wbc_workspace}/install/${_ylf_wbc_package}/share/${_ylf_wbc_package}/local_setup.bash"
  if [ -f "${_ylf_wbc_setup}" ]; then
    source "${_ylf_wbc_setup}"
  fi
done
unset _ylf_wbc_workspace _ylf_wbc_package _ylf_wbc_setup
