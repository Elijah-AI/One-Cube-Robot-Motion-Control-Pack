#ifndef TASK_MANAGER__CURRENT_TCP_HEIGHT_HPP_
#define TASK_MANAGER__CURRENT_TCP_HEIGHT_HPP_

#include "task_manager/vision_pose_utils.hpp"
#include "com_interfaces/msg/motor_status.hpp"

namespace task_act
{
// Intersect the calibrated slot's visual-Z line with the measured base-Z plane.
// Keep the visual X/Y slot coordinates and TCP orientation unchanged.
inline bool resolve_current_tcp_height(
  const geometry_msgs::msg::PoseStamped& reference,
  const task_cfg::MovePoseParam& param,
  const com_interfaces::msg::MotorStatus& state,
  geometry_msgs::msg::PoseStamped& target,
  double& local_height, std::string& error)
{
  const auto& offset = param.vision_pose_offset;
  if (!offset.keep_current_tcp_height || param.arm_id > 1 ||
      param.frame_id != "base_link" || offset.translation_frame != "vision" ||
      state.header.frame_id != param.frame_id || state.end_effector_pose.size() != 2U ||
      state.joint_positions.size() != 14U || state.joint_velocities.size() != 14U ||
      !std::isfinite(offset.minimum_z) || offset.minimum_z < 0.0) {
    error = "current TCP height requires complete single-arm base_link feedback";
    return false;
  }
  geometry_msgs::msg::PoseStamped current;
  current.header = state.header;
  current.pose = state.end_effector_pose[param.arm_id];
  if (!valid_vision_pose(current, param.frame_id, error)) return false;
  for (std::size_t i = 7U * param.arm_id; i < 7U * (param.arm_id + 1U); ++i) {
    if (!std::isfinite(state.joint_positions[i]) ||
        !std::isfinite(state.joint_velocities[i]) ||
        std::abs(state.joint_velocities[i]) > 0.01) {
      error = "current TCP height requires finite, stopped arm feedback";
      return false;
    }
  }
  auto resolved = offset;
  resolved.translation[2] = 0.0;
  if (!offset_vision_pose(reference, resolved, param.frame_id, target, error)) return false;
  const auto& q = reference.pose.orientation;
  const double norm2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  const double vertical = 1.0 - 2.0*(q.x*q.x + q.y*q.y)/norm2;
  if (vertical <= 0.1) {
    error = "visual +Z must point upward to resolve current TCP height";
    return false;
  }
  local_height = (current.pose.position.z - target.pose.position.z) / vertical;
  if (!std::isfinite(local_height) || local_height <= offset.minimum_z) {
    error = "current TCP height is not above minimum_z; cannot descend to placement target";
    return false;
  }
  resolved.translation[2] = local_height;
  return offset_vision_pose(reference, resolved, param.frame_id, target, error);
}
}  // namespace task_act
#endif
