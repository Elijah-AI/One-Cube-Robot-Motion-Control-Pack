#ifndef MOTION_CONTROL_PKG_MOTION_JOINT_SPACE_CONTROL_HPP_
#define MOTION_CONTROL_PKG_MOTION_JOINT_SPACE_CONTROL_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "motion_control_pkg/execution/joint_command.hpp"
#include "motion_control_pkg/sdk/dual_arm_device_interface.hpp"

namespace motion_control_pkg::motion
{

using JointVector =
  std::array<double, motion_control_pkg::sdk::kArmDof>;
using CcsJoint67Table = std::array<std::array<double, 3>, 4>;

constexpr double kRadiansToDegrees =
  57.295779513082320876798154814105;
constexpr double kDegreesToRadians =
  0.01745329251994329576923690768489;

struct JointTargetValidation
{
  bool valid{false};
  std::string error_message;
};

struct PositionLimitIntersection
{
  bool valid{false};
  std::vector<double> lower;
  std::vector<double> upper;
  std::string error_message;
};

struct ScaledMotionLimits
{
  bool valid{false};
  std::vector<double> velocity;
  std::vector<double> acceleration;
  std::vector<double> jerk;
  std::string error_message;
};

inline JointTargetValidation validateSpeedScale(double speed_scale)
{
  if (!std::isfinite(speed_scale) || speed_scale <= 0.0 ||
    speed_scale > 1.0)
  {
    return {
      false, "speed must be finite and in the range (0, 1]"};
  }
  return {true, {}};
}

/**
 * @brief 求厂商 CCS 限位与 URDF/Pinocchio 限位的逐关节交集。
 *
 * 规划与下发都使用这一个交集，避免只满足模型限位却越过控制器软限位，
 * 或只满足控制器配置却越过当前 URDF 的关节范围。
 */
inline PositionLimitIntersection intersectPositionLimits(
  const std::vector<double> & sdk_lower,
  const std::vector<double> & sdk_upper,
  const std::vector<double> & model_lower,
  const std::vector<double> & model_upper)
{
  const auto has_arm_dof = [](const std::vector<double> & values) {
      return values.size() == motion_control_pkg::sdk::kArmDof;
    };
  if (!has_arm_dof(sdk_lower) || !has_arm_dof(sdk_upper) ||
    !has_arm_dof(model_lower) || !has_arm_dof(model_upper))
  {
    return {
      false, {}, {},
      "all joint limit vectors must contain exactly seven values"};
  }

  PositionLimitIntersection result;
  result.lower.resize(motion_control_pkg::sdk::kArmDof);
  result.upper.resize(motion_control_pkg::sdk::kArmDof);
  for (std::size_t index = 0;
    index < motion_control_pkg::sdk::kArmDof; ++index)
  {
    const std::array<double, 4> configured{
      sdk_lower[index], sdk_upper[index],
      model_lower[index], model_upper[index]};
    if (!std::all_of(configured.begin(), configured.end(), [](double value) {
        return std::isfinite(value);
      }) || sdk_lower[index] >= sdk_upper[index] ||
      model_lower[index] >= model_upper[index])
    {
      result.error_message =
        "configured limits for joint " + std::to_string(index + 1U) +
        " are invalid";
      result.lower.clear();
      result.upper.clear();
      return result;
    }

    result.lower[index] = std::max(sdk_lower[index], model_lower[index]);
    result.upper[index] = std::min(sdk_upper[index], model_upper[index]);
    if (result.lower[index] >= result.upper[index]) {
      result.error_message =
        "SDK and model limits do not overlap for joint " +
        std::to_string(index + 1U);
      result.lower.clear();
      result.upper.clear();
      return result;
    }
  }
  result.valid = true;
  return result;
}

/**
 * @brief 按 Action 的 (0, 1] 速度比例缩放已加载的控制器运动约束。
 *
 * 这里只缩放现成约束，实际点到点轨迹仍由 Ruckig 求解，不在业务层生成
 * 插值曲线。速度为零无法形成可执行的有限时长轨迹，因此显式拒绝。
 */
inline ScaledMotionLimits scaleMotionLimits(
  const std::vector<double> & velocity,
  const std::vector<double> & acceleration,
  const std::vector<double> & jerk,
  double speed_scale)
{
  const auto speed_validation = validateSpeedScale(speed_scale);
  if (!speed_validation.valid) {
    return {false, {}, {}, {}, speed_validation.error_message};
  }
  if (velocity.size() != motion_control_pkg::sdk::kArmDof ||
    acceleration.size() != motion_control_pkg::sdk::kArmDof ||
    jerk.size() != motion_control_pkg::sdk::kArmDof)
  {
    return {
      false, {}, {}, {},
      "motion limit vectors must contain exactly seven values"};
  }

  ScaledMotionLimits result;
  result.velocity.resize(motion_control_pkg::sdk::kArmDof);
  result.acceleration.resize(motion_control_pkg::sdk::kArmDof);
  result.jerk.resize(motion_control_pkg::sdk::kArmDof);
  for (std::size_t index = 0;
    index < motion_control_pkg::sdk::kArmDof; ++index)
  {
    if (!std::isfinite(velocity[index]) || velocity[index] <= 0.0 ||
      !std::isfinite(acceleration[index]) || acceleration[index] <= 0.0 ||
      !std::isfinite(jerk[index]) || jerk[index] <= 0.0)
    {
      result.error_message =
        "configured motion limit for joint " +
        std::to_string(index + 1U) + " is invalid";
      result.velocity.clear();
      result.acceleration.clear();
      result.jerk.clear();
      return result;
    }
    result.velocity[index] = velocity[index] * speed_scale;
    result.acceleration[index] = acceleration[index] * speed_scale;
    result.jerk[index] = jerk[index] * speed_scale;
  }
  result.valid = true;
  return result;
}

/**
 * @brief 校验 CCS 配置中的 J6/J7 二次包络。
 *
 * 表格四行依次为 J6 正向上界、J6 负向上界、J6 负向下界、J6 正向下界；
 * 系数及自变量均按厂商配置的“度”解释。该逻辑复用现有 CCS 约束定义，
 * 不以人为固定角度替代配置文件。
 */
inline bool passesCcsJoint67Constraint(
  const JointVector & joints_rad,
  const CcsJoint67Table & table_deg)
{
  if (!std::all_of(joints_rad.begin(), joints_rad.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    return false;
  }
  for (const auto & row : table_deg) {
    if (!std::all_of(row.begin(), row.end(), [](double value) {
        return std::isfinite(value);
      }))
    {
      return false;
    }
  }

  const double joint6_deg = joints_rad[5] * kRadiansToDegrees;
  const double joint7_deg = joints_rad[6] * kRadiansToDegrees;
  const auto polynomial = [joint6_deg](const std::array<double, 3> & row) {
      return row[0] * joint6_deg * joint6_deg +
             row[1] * joint6_deg + row[2];
    };
  constexpr double kComparisonToleranceDeg = 1e-9;

  if (joint6_deg > 1.0) {
    return joint7_deg <= polynomial(table_deg[0]) + kComparisonToleranceDeg &&
           joint7_deg >= polynomial(table_deg[3]) - kComparisonToleranceDeg;
  }
  if (joint6_deg < -1.0) {
    return joint7_deg <= polynomial(table_deg[1]) + kComparisonToleranceDeg &&
           joint7_deg >= polynomial(table_deg[2]) - kComparisonToleranceDeg;
  }
  return true;
}

inline JointTargetValidation validateJointTarget(
  const JointVector & target_rad,
  const std::vector<double> & lower_rad,
  const std::vector<double> & upper_rad,
  const CcsJoint67Table & joint67_table_deg)
{
  if (lower_rad.size() != motion_control_pkg::sdk::kArmDof ||
    upper_rad.size() != motion_control_pkg::sdk::kArmDof)
  {
    return {
      false, "joint limit vectors must contain exactly seven values"};
  }

  constexpr double kLimitToleranceRad = 1e-12;
  for (std::size_t index = 0;
    index < motion_control_pkg::sdk::kArmDof; ++index)
  {
    if (!std::isfinite(target_rad[index])) {
      return {
        false, "joint " + std::to_string(index + 1U) + " is not finite"};
    }
    if (!std::isfinite(lower_rad[index]) ||
      !std::isfinite(upper_rad[index]) ||
      lower_rad[index] >= upper_rad[index])
    {
      return {
        false, "configured limits for joint " +
        std::to_string(index + 1U) + " are invalid"};
    }
    if (target_rad[index] < lower_rad[index] - kLimitToleranceRad ||
      target_rad[index] > upper_rad[index] + kLimitToleranceRad)
    {
      std::ostringstream message;
      message << "joint " << (index + 1U) << " target=" << target_rad[index]
              << " rad is outside [" << lower_rad[index] << ", "
              << upper_rad[index] << "] rad";
      return {false, message.str()};
    }
  }

  if (!passesCcsJoint67Constraint(target_rad, joint67_table_deg)) {
    return {false, "target violates the configured CCS J6/J7 constraint"};
  }
  return {true, {}};
}

/**
 * @brief 对 Ruckig 的每个离散采样点重复安全约束检查。
 *
 * 仅验证起点和终点不足以证明非线性的 J6/J7 包络在中间过程始终成立，
 * 所以下发前必须对完整 4 ms 采样序列做同一校验。
 */
inline JointTargetValidation validateTrajectoryPositions(
  const std::vector<std::vector<double>> & positions_rad,
  const std::vector<double> & lower_rad,
  const std::vector<double> & upper_rad,
  const CcsJoint67Table & joint67_table_deg)
{
  if (positions_rad.empty()) {
    return {false, "planned trajectory contains no samples"};
  }
  for (std::size_t sample = 0; sample < positions_rad.size(); ++sample) {
    if (positions_rad[sample].size() != motion_control_pkg::sdk::kArmDof) {
      return {
        false, "planned trajectory sample " + std::to_string(sample) +
        " does not contain seven joints"};
    }
    JointVector point{};
    std::copy(positions_rad[sample].begin(), positions_rad[sample].end(),
      point.begin());
    const auto validation = validateJointTarget(
      point, lower_rad, upper_rad, joint67_table_deg);
    if (!validation.valid) {
      return {
        false, "planned trajectory sample " + std::to_string(sample) +
        " is invalid: " + validation.error_message};
    }
  }
  return {true, {}};
}

inline bool trajectoryToVendorDegrees(
  const std::vector<std::vector<double>> & positions_rad,
  std::vector<motion_control_pkg::execution::JointCommand> & commands_deg,
  std::string & error_message)
{
  commands_deg.clear();
  commands_deg.reserve(positions_rad.size());
  for (std::size_t sample = 0; sample < positions_rad.size(); ++sample) {
    if (positions_rad[sample].size() != motion_control_pkg::sdk::kArmDof) {
      error_message =
        "planned trajectory sample " + std::to_string(sample) +
        " does not contain seven joints";
      commands_deg.clear();
      return false;
    }
    motion_control_pkg::execution::JointCommand command{};
    for (std::size_t joint = 0;
      joint < motion_control_pkg::sdk::kArmDof; ++joint)
    {
      if (!std::isfinite(positions_rad[sample][joint])) {
        error_message =
          "planned trajectory contains a non-finite joint sample";
        commands_deg.clear();
        return false;
      }
      // MarvinSDK 的关节命令边界使用度；规划器内部始终保持弧度。
      command[joint] = positions_rad[sample][joint] * kRadiansToDegrees;
    }
    commands_deg.push_back(command);
  }
  if (commands_deg.empty()) {
    error_message = "planned trajectory contains no samples";
    return false;
  }
  error_message.clear();
  return true;
}

inline double maximumJointErrorRadians(
  const std::array<double, motion_control_pkg::sdk::kArmDof> & feedback_deg,
  const JointVector & target_rad)
{
  double maximum_error = 0.0;
  for (std::size_t index = 0;
    index < motion_control_pkg::sdk::kArmDof; ++index)
  {
    if (!std::isfinite(feedback_deg[index]) ||
      !std::isfinite(target_rad[index]))
    {
      return std::numeric_limits<double>::infinity();
    }
    maximum_error = std::max(
      maximum_error,
      std::abs(feedback_deg[index] * kDegreesToRadians - target_rad[index]));
  }
  return maximum_error;
}

}  // namespace motion_control_pkg::motion

#endif  // MOTION_CONTROL_PKG_MOTION_JOINT_SPACE_CONTROL_HPP_
