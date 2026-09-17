#ifndef MOTION_CONTROL_PKG_MOTION_POSE_SPACE_CONTROL_HPP_
#define MOTION_CONTROL_PKG_MOTION_POSE_SPACE_CONTROL_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "traj_common_util.h"

namespace motion_control_pkg::motion
{

enum class PoseTrajectoryKind
{
  kRuckigPointToPoint,
  kToppraMultiWaypoint
};

struct PoseWaypointPath
{
  bool valid{false};
  PoseTrajectoryKind kind{PoseTrajectoryKind::kRuckigPointToPoint};
  std::vector<Waypoint> waypoints;
  std::string error_message;
};

// 这里只整理既有 IK/OMPL 的输出，不生成或平滑路径。连续重复点被去除；
// 单一有效状态复制成起终两点，交给 Ruckig 处理零位移点到点请求。
inline PoseWaypointPath preparePoseWaypointPath(
  const std::vector<std::vector<double>> & active_arm_path,
  std::size_t expected_dof = 7U,
  double duplicate_tolerance = 1.0e-12)
{
  PoseWaypointPath result;
  if (active_arm_path.empty() || expected_dof == 0U ||
    !std::isfinite(duplicate_tolerance) || duplicate_tolerance < 0.0)
  {
    result.error_message = "pose path or path settings are invalid";
    return result;
  }

  for (std::size_t index = 0; index < active_arm_path.size(); ++index) {
    const auto & point = active_arm_path[index];
    if (point.size() != expected_dof ||
      !std::all_of(point.begin(), point.end(), [](double value) {
        return std::isfinite(value);
      }))
    {
      result.error_message =
        "pose path point " + std::to_string(index) +
        " has an invalid dimension or non-finite value";
      result.waypoints.clear();
      return result;
    }

    bool duplicate = false;
    if (!result.waypoints.empty()) {
      duplicate = true;
      const auto & previous = result.waypoints.back().positions;
      for (std::size_t joint = 0; joint < expected_dof; ++joint) {
        if (std::abs(previous[joint] - point[joint]) > duplicate_tolerance) {
          duplicate = false;
          break;
        }
      }
    }
    if (!duplicate) {
      Waypoint waypoint;
      waypoint.positions = point;
      result.waypoints.push_back(std::move(waypoint));
    }
  }

  if (result.waypoints.empty()) {
    result.error_message = "pose path contains no finite active-arm state";
    return result;
  }
  if (result.waypoints.size() == 1U) {
    result.waypoints.push_back(result.waypoints.front());
  }

  // 架构冻结：单目标的起终两点交给 Ruckig；OMPL 产生的三点及以上路径
  // 整体交给 TOPPRA 时间参数化，禁止逐段调用 Ruckig 再拼接。
  result.kind = result.waypoints.size() == 2U ?
    PoseTrajectoryKind::kRuckigPointToPoint :
    PoseTrajectoryKind::kToppraMultiWaypoint;
  result.valid = true;
  return result;
}

}  // namespace motion_control_pkg::motion

#endif  // MOTION_CONTROL_PKG_MOTION_POSE_SPACE_CONTROL_HPP_
