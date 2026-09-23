#ifndef TASK_MANAGER__VISION_POSE_UTILS_HPP_
#define TASK_MANAGER__VISION_POSE_UTILS_HPP_

#include <cmath>
#include <string>
#include "task_manager/task_config_parser.hpp"
#include "com_interfaces/action/detection_pose_cloud.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace task_act
{
inline bool valid_vision_pose(
  const geometry_msgs::msg::PoseStamped& pose, const std::string& frame,
  std::string& error)
{
  if (frame.empty() || pose.header.frame_id != frame) {
    error = "vision target frame '" + pose.header.frame_id + "' differs from required '" +
      frame + "'; transform the measured pose before returning it";
    return false;
  }
  const auto& p = pose.pose.position;
  const auto& q = pose.pose.orientation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(norm) || norm < 1e-12) {
    error = "vision target contains an invalid position or quaternion";
    return false;
  }
  return true;
}

inline bool validate_vision_result(
  const com_interfaces::action::DetectionPoseCloud::Result& result,
  const task_cfg::VisionActionParam& param, std::string& error)
{
  if (!result.success) {error = "vision detection failed"; return false;}
  if (param.require_single_target && result.detected_targets.size() != 1) {
    error = "vision detection must return exactly one corner target";
    return false;
  }
  if (param.require_expected_frame) {
    if (result.detected_targets.empty()) {error = "vision detection returned no targets"; return false;}
    for (const auto& target : result.detected_targets) {
      if (!valid_vision_pose(target.pose, param.expected_frame_id, error)) {return false;}
    }
  }
  return true;
}

// Copy the captured reference; never accumulate offsets in the cached result.
// "vision" rotates translation by the original measured orientation, before TCP alignment.
inline bool offset_vision_pose(
  const geometry_msgs::msg::PoseStamped& corner, const task_cfg::VisionPoseOffset& offset,
  const std::string& frame,
  geometry_msgs::msg::PoseStamped& target, std::string& error)
{
  if (!offset.enabled || offset.translation.size() != 3 || offset.rotation.size() != 4 ||
      (offset.translation_frame != "base" && offset.translation_frame != "vision")) {
    error = "invalid vision_pose_offset configuration"; return false;
  }
  if (!valid_vision_pose(corner, frame, error)) {return false;}
  for (double v : offset.translation) {
    if (!std::isfinite(v)) {error = "non-finite vision translation"; return false;}
  }
  double offset_norm = 0.0;
  for (double v : offset.rotation) {offset_norm += v*v;}
  if (!std::isfinite(offset_norm) || offset_norm < 1e-12) {
    error = "invalid offset quaternion"; return false;
  }
  target = corner;
  const auto& q = corner.pose.orientation;
  const double q_norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  const double r_norm = std::sqrt(offset_norm);
  const double x = q.x/q_norm, y = q.y/q_norm, z = q.z/q_norm, w = q.w/q_norm;
  const double dx = offset.translation[0], dy = offset.translation[1], dz = offset.translation[2];
  if (offset.translation_frame == "vision") {
    target.pose.position.x += (1-2*(y*y+z*z))*dx + 2*(x*y-z*w)*dy + 2*(x*z+y*w)*dz;
    target.pose.position.y += 2*(x*y+z*w)*dx + (1-2*(x*x+z*z))*dy + 2*(y*z-x*w)*dz;
    target.pose.position.z += 2*(x*z-y*w)*dx + 2*(y*z+x*w)*dy + (1-2*(x*x+y*y))*dz;
  } else {
    target.pose.position.x += dx;
    target.pose.position.y += dy;
    target.pose.position.z += dz;
  }
  const double rx = offset.rotation[0]/r_norm, ry = offset.rotation[1]/r_norm;
  const double rz = offset.rotation[2]/r_norm, rw = offset.rotation[3]/r_norm;
  auto& out = target.pose.orientation;
  // Hamilton product q_corner * q_offset (xyzw).
  out.x = w*rx + x*rw + y*rz - z*ry;
  out.y = w*ry - x*rz + y*rw + z*rx;
  out.z = w*rz + x*ry - y*rx + z*rw;
  out.w = w*rw - x*rx - y*ry - z*rz;
  return valid_vision_pose(target, frame, error);
}

inline bool offset_vision_pose(
  const geometry_msgs::msg::PoseStamped& reference, const task_cfg::MovePoseParam& param,
  geometry_msgs::msg::PoseStamped& target, std::string& error)
{
  if (param.use_config_pose ||
      (param.arm_id != 0 && param.arm_id != 1 && param.arm_id != 5 && param.arm_id != 6)) {
    error = "vision_pose_offset requires a single-arm vision target"; return false;
  }
  return offset_vision_pose(reference, param.vision_pose_offset, param.frame_id, target, error);
}
}  // namespace task_act
#endif  // TASK_MANAGER__VISION_POSE_UTILS_HPP_
