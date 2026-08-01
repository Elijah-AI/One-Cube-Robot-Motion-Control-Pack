#ifndef MOTION_CONTROL_PKG_EXECUTION_SYNCHRONIZED_TRAJECTORY_HPP_
#define MOTION_CONTROL_PKG_EXECUTION_SYNCHRONIZED_TRAJECTORY_HPP_

#include <string>
#include <vector>

#include "motion_control_pkg/execution/dual_arm_command_coordinator.hpp"

namespace motion_control_pkg::execution
{

struct SynchronizedTrajectoryPair
{
  bool success{false};
  std::string error_message;
  std::vector<JointCommand> left;
  std::vector<JointCommand> right;
};

/**
 * @brief 将较短轨迹按归一化时间线性重采样到较长轨迹的 4 ms 采样数。
 *
 * 起终点保持精确一致；只拉长较短轨迹，不加快较长轨迹。调用方仍需在
 * 防碰撞规划后使用成对路点时间参数化，避免重采样破坏碰撞路径对应关系。
 */
SynchronizedTrajectoryPair synchronizeTrajectoryPair(
  const std::vector<JointCommand> & left,
  const std::vector<JointCommand> & right);

}  // namespace motion_control_pkg::execution

#endif  // MOTION_CONTROL_PKG_EXECUTION_SYNCHRONIZED_TRAJECTORY_HPP_
