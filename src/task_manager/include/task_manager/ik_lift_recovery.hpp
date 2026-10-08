#pragma once

#include <cmath>
#include <string>

namespace task_act {
// These messages originate only in WholeBodyModule::plan_trajectory(), before
// prepare/dispatch. In particular, a measured-completion timeout is NOT IK.
inline bool is_pre_motion_whole_body_ik_failure(const std::string& message,
    bool aborted, bool success, bool execution_observed,
    bool timing_valid, double motion_time) {
  if (!aborted || success || execution_observed) return false;
  if (timing_valid && (!std::isfinite(motion_time) || motion_time != 0)) return false;
  const char* prefixes[] = {
      "failed to solve dual-arm whole-body IK at Cartesian substep ",
      "failed to solve whole-body IK for target pose",
      "whole-body weighted IK failed at Cartesian line waypoint ",
      "failed to find a collision-free dual-arm whole-body IK solution for the target pose"};
  for (const auto* prefix : prefixes)
    if (message.rfind(prefix, 0) == 0) return true;
  return false;
}
}  // namespace task_act
