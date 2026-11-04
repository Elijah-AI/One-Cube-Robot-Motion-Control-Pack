#pragma once
#include "motion_control_pkg/execution/precision_policy.hpp"
#include <rclcpp/rclcpp.hpp>

namespace motion_control_pkg::execution {
// Shared node: WBC may read the same immutable leg policy after the leg module.
inline PrecisionPolicy precisionParameters(rclcpp::Node& node, const std::string& prefix) {
  PrecisionPolicy p;
  rcl_interfaces::msg::ParameterDescriptor fixed; fixed.read_only = true;
  const auto real = [&](const char* key, double fallback) {
    const std::string name = prefix + "." + key;
    return node.has_parameter(name) ? node.get_parameter(name).as_double() :
        node.declare_parameter<double>(name, fallback, fixed);
  };
  p.completion.position_tolerance = real("goal_tolerance_rad", p.completion.position_tolerance);
  p.completion.velocity_tolerance = real("stopped_velocity_rad_s", p.completion.velocity_tolerance);
  p.completion.stable_seconds = real("stable_time_sec", p.completion.stable_seconds);
  p.completion.stall_seconds = real("stall_time_sec", p.completion.stall_seconds);
  p.completion.progress_rad = real("minimum_progress_rad", p.completion.progress_rad);
  p.completion_timeout = real("completion_timeout_sec", p.completion_timeout);
  const auto enabled = prefix + ".approach_enabled";
  p.approach_enabled = node.has_parameter(enabled) ? node.get_parameter(enabled).as_bool() :
      node.declare_parameter<bool>(enabled, p.approach_enabled, fixed);
  p.approach_distance = real("approach_distance_rad", p.approach_distance);
  p.speed_scale = real("approach_speed_scale", p.speed_scale);
  p.velocity_cap = real("approach_velocity_rad_s", p.velocity_cap);
  p.acceleration_cap = real("approach_acceleration_rad_s2", p.acceleration_cap);
  p.jerk_cap = real("approach_jerk_rad_s3", p.jerk_cap);
  p.validate();
  return p;
}
}  // namespace motion_control_pkg::execution
