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

// Copy the captured corner; never accumulate offsets in the cached result.
// Translation is expressed in the required frame, rotation in the corner's local frame.
inline bool offset_vision_pose(
  const geometry_msgs::msg::PoseStamped& corner, const task_cfg::MovePoseParam& param,
  geometry_msgs::msg::PoseStamped& target, std::string& error)
{
  const auto& offset = param.vision_pose_offset;
  if (!offset.enabled || param.use_config_pose || param.arm_id > 1 ||
      offset.translation.size() != 3 || offset.rotation.size() != 4) {
    error = "invalid vision_pose_offset configuration"; return false;
  }
  if (!valid_vision_pose(corner, param.frame_id, error)) {return false;}
  double offset_norm = 0.0;
  for (double v : offset.rotation) {offset_norm += v*v;}
  if (!std::isfinite(offset_norm) || offset_norm < 1e-12) {
    error = "invalid offset quaternion"; return false;
  }
  target = corner;
  target.pose.position.x += offset.translation[0];
  target.pose.position.y += offset.translation[1];
  target.pose.position.z += offset.translation[2];
  const auto& q = corner.pose.orientation;
  const double q_norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  const double r_norm = std::sqrt(offset_norm);
  const double x = q.x/q_norm, y = q.y/q_norm, z = q.z/q_norm, w = q.w/q_norm;
  const double rx = offset.rotation[0]/r_norm, ry = offset.rotation[1]/r_norm;
  const double rz = offset.rotation[2]/r_norm, rw = offset.rotation[3]/r_norm;
  auto& out = target.pose.orientation;
  // Hamilton product q_corner * q_offset (xyzw).
  out.x = w*rx + x*rw + y*rz - z*ry;
  out.y = w*ry - x*rz + y*rw + z*rx;
  out.z = w*rz + x*ry - y*rx + z*rw;
  out.w = w*rw - x*rx - y*ry - z*rz;
  return valid_vision_pose(target, param.frame_id, error);
}
}  // namespace task_act
#endif  // TASK_MANAGER__VISION_POSE_UTILS_HPP_
