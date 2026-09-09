#ifndef MOTION_CONTROL_PKG_EXECUTION_IMPEDANCE_CONFIGURATION_HPP_
#define MOTION_CONTROL_PKG_EXECUTION_IMPEDANCE_CONFIGURATION_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "motion_control_pkg/sdk/dual_arm_device_interface.hpp"

namespace motion_control_pkg::execution
{

inline motion_control_pkg::sdk::ImpedanceParameters declareImpedanceParameters(
  rclcpp::Node & node,
  const std::string & prefix,
  const std::vector<double> & default_stiffness,
  const std::vector<double> & default_damping,
  int default_velocity_ratio,
  int default_acceleration_ratio)
{
  auto stiffness = node.declare_parameter<std::vector<double>>(
    prefix + "_stiffness", default_stiffness);
  auto damping = node.declare_parameter<std::vector<double>>(
    prefix + "_damping", default_damping);
  const auto velocity_ratio = node.declare_parameter<std::int64_t>(
    prefix + "_velocity_ratio", default_velocity_ratio);
  const auto acceleration_ratio = node.declare_parameter<std::int64_t>(
    prefix + "_acceleration_ratio", default_acceleration_ratio);

  if (stiffness.size() != motion_control_pkg::sdk::kArmDof ||
      damping.size() != motion_control_pkg::sdk::kArmDof)
  {
    throw std::invalid_argument(
      prefix + " stiffness and damping must each contain seven values");
  }
  if (velocity_ratio < 1 || velocity_ratio > 100 ||
      acceleration_ratio < 1 || acceleration_ratio > 100)
  {
    throw std::invalid_argument(
      prefix + " velocity/acceleration ratios must be in [1,100]");
  }

  motion_control_pkg::sdk::ImpedanceParameters parameters;
  parameters.velocity_ratio = static_cast<int>(velocity_ratio);
  parameters.acceleration_ratio = static_cast<int>(acceleration_ratio);
  for (std::size_t index = 0; index < motion_control_pkg::sdk::kArmDof;
      ++index)
  {
    if (!std::isfinite(stiffness[index]) || stiffness[index] < 0.0 ||
        !std::isfinite(damping[index]) || damping[index] < 0.0 ||
        damping[index] > 1.0)
    {
      throw std::invalid_argument(
        prefix + " requires finite K>=0 and damping in [0,1]");
    }
    parameters.stiffness[index] = stiffness[index];
    parameters.damping[index] = damping[index];
  }
  return parameters;
}

inline motion_control_pkg::sdk::ToolParameters declareToolParameters(
  rclcpp::Node & node,
  const std::string & prefix,
  const Eigen::Isometry3d & flange_to_tool_tcp,
  const std::vector<double> & default_dynamics)
{
  auto dynamics = node.declare_parameter<std::vector<double>>(
    prefix + "_tool_dynamics", default_dynamics);
  if (dynamics.size() != motion_control_pkg::sdk::kToolDynamicsSize) {
    throw std::invalid_argument(
      prefix + "_tool_dynamics must contain M, MCP_X, MCP_Y, MCP_Z, "
      "I_XX, I_XY, I_XZ, I_YY, I_YZ, I_ZZ");
  }

  motion_control_pkg::sdk::ToolParameters parameters;
  const auto & translation = flange_to_tool_tcp.translation();
  parameters.kinematics[0] = translation.x() * 1000.0;
  parameters.kinematics[1] = translation.y() * 1000.0;
  parameters.kinematics[2] = translation.z() * 1000.0;

  // 与 MarvinSDK::FX_Matrix42XYZABCDEG 相同的 Rz(C)*Ry(B)*Rx(A) 分解。
  const auto & rotation = flange_to_tool_tcp.linear();
  const double planar = std::hypot(rotation(0, 0), rotation(1, 0));
  const double angle_y = std::atan2(-rotation(2, 0), planar);
  double angle_x = 0.0;
  double angle_z = 0.0;
  if (planar <= 1.0e-12) {
    angle_x = angle_y > 0.0 ?
      std::atan2(rotation(0, 1), rotation(1, 1)) :
      -std::atan2(rotation(0, 1), rotation(1, 1));
  } else {
    angle_x = std::atan2(rotation(2, 1), rotation(2, 2));
    angle_z = std::atan2(rotation(1, 0), rotation(0, 0));
  }
  constexpr double kRadiansToDegrees =
    57.295779513082320876798154814105;
  parameters.kinematics[3] = angle_x * kRadiansToDegrees;
  parameters.kinematics[4] = angle_y * kRadiansToDegrees;
  parameters.kinematics[5] = angle_z * kRadiansToDegrees;

  std::copy(dynamics.begin(), dynamics.end(), parameters.dynamics.begin());
  if (!motion_control_pkg::sdk::isValidToolParameters(parameters)) {
    throw std::invalid_argument(
      prefix + "_tool_dynamics must be finite with M>0 and a physically "
      "valid inertia tensor");
  }
  return parameters;
}

}  // namespace motion_control_pkg::execution

#endif  // MOTION_CONTROL_PKG_EXECUTION_IMPEDANCE_CONFIGURATION_HPP_
