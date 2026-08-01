#ifndef MOTION_CONTROL_PKG_EXECUTION_ARM_ACTION_UTILS_HPP_
#define MOTION_CONTROL_PKG_EXECUTION_ARM_ACTION_UTILS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "motion_control_pkg/sdk/dual_arm_device_interface.hpp"

namespace motion_control_pkg::execution
{

constexpr std::uint8_t kLeftArmId = 0;
constexpr std::uint8_t kRightArmId = 1;
constexpr std::uint8_t kBothArmsId = 2;

inline std::optional<sdk::ArmChannel> armChannelFromId(std::uint8_t arm_id)
{
  if (arm_id == kLeftArmId) {
    return sdk::ArmChannel::kLeft;
  }
  if (arm_id == kRightArmId) {
    return sdk::ArmChannel::kRight;
  }
  return std::nullopt;
}

inline bool singleArmGoalEnabled(
  sdk::ActiveArmConfiguration configuration, std::uint8_t arm_id)
{
  const auto channel = armChannelFromId(arm_id);
  return channel && sdk::isArmEnabled(configuration, *channel);
}

struct JointGoalValidationResult
{
  bool valid{false};
  std::array<double, sdk::kArmDof> ordered_positions{};
  std::string error_message;
};

inline const std::array<std::string, sdk::kArmDof> & jointNames(
  sdk::ArmChannel channel)
{
  static const std::array<std::string, sdk::kArmDof> left_names{
    "Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L",
    "Joint5_L", "Joint6_L", "Joint7_L"};
  static const std::array<std::string, sdk::kArmDof> right_names{
    "Joint1_R", "Joint2_R", "Joint3_R", "Joint4_R",
    "Joint5_R", "Joint6_R", "Joint7_R"};
  return channel == sdk::ArmChannel::kLeft ? left_names : right_names;
}

/**
 * @brief 校验七关节名称并按框架标准顺序重排目标角。
 *
 * 本函数只负责不依赖运行时配置的名称、数量和有限值校验。关节限位及
 * J6/J7 耦合约束由 MoveJoints 执行入口使用当前 CCS/URDF 配置再次校验。
 */
inline JointGoalValidationResult validateAndOrderJointGoal(
  sdk::ArmChannel channel,
  const std::vector<std::string> & names,
  const std::vector<double> & positions)
{
  JointGoalValidationResult result;
  if (names.size() != sdk::kArmDof || positions.size() != sdk::kArmDof) {
    result.error_message =
      "MoveJoints requires exactly seven joint names and seven positions";
    return result;
  }

  const std::set<std::string> unique_names(names.begin(), names.end());
  if (unique_names.size() != sdk::kArmDof) {
    result.error_message = "MoveJoints joint names must be unique";
    return result;
  }

  const auto & expected_names = jointNames(channel);
  for (std::size_t expected_index = 0;
    expected_index < expected_names.size(); ++expected_index)
  {
    const auto found =
      std::find(names.begin(), names.end(), expected_names[expected_index]);
    if (found == names.end()) {
      result.error_message =
        "MoveJoints is missing or contains a wrong-side joint: " +
        expected_names[expected_index];
      return result;
    }
    const auto source_index =
      static_cast<std::size_t>(std::distance(names.begin(), found));
    if (!std::isfinite(positions[source_index])) {
      result.error_message = "MoveJoints positions must all be finite";
      return result;
    }
    result.ordered_positions[expected_index] = positions[source_index];
  }

  result.valid = true;
  return result;
}

}  // namespace motion_control_pkg::execution

#endif  // MOTION_CONTROL_PKG_EXECUTION_ARM_ACTION_UTILS_HPP_
