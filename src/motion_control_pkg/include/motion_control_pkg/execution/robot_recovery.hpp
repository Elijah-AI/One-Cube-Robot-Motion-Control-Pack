#pragma once
#include "motion_control_pkg/sdk/drive_control.hpp"
#include <array>
#include <functional>
#include <string>

namespace motion_control_pkg::execution {
struct RobotRecoveryResult {
  bool success{}, arms_cleared{}, lower_body_cleared{}, head_cleared{};
  bool drives_enabled{}, motion_ready{};
  std::string message;
};
struct RobotRecoveryHooks {
  // Order: arms, waist/legs, head. All groups are attempted in each clear phase.
  std::array<std::function<sdk::DriveControlResult(bool)>,3> drives;
  std::function<sdk::DriveControlResult()> enable_arms, enable_body;
  std::function<bool(std::string&)> reset_latch;
  std::function<bool()> canceled;
};
inline RobotRecoveryResult recoverRobot(const RobotRecoveryHooks& hooks, bool enable) {
  RobotRecoveryResult out;
  const std::array<const char*,3> names{{"arms","waist/legs","head"}};
  const auto phase=[&](bool clear) {
    bool all=true;
    for (unsigned i=0;i<3;++i) {
      if (hooks.canceled()) {out.message+="recovery canceled; ";return false;}
      const auto result=hooks.drives[i](clear);
      all=result.success() && all;
      out.message+=std::string(names[i])+(clear?" clear: ":" disable: ")+result.message+"; ";
      if (clear) {
        if(i==0)out.arms_cleared=result.success();
        if(i==1)out.lower_body_cleared=result.success();
        if(i==2)out.head_cleared=result.success();
      }
    }
    return all && !hooks.canceled();
  };
  if (!phase(false) || !phase(true)) return out;
  if (!enable) {
    out.success=true;out.message+="cleared; drives disabled and software fence retained";return out;
  }
  const auto enable_all=[&] {
    if(hooks.canceled())return false;
    auto arm=hooks.enable_arms();out.message+=arm.message+"; ";
    if(!arm.success() || hooks.canceled())return false;
    auto body=hooks.enable_body();out.message+=body.message+"; ";
    if(!body.success() || hooks.canceled())return false;
    std::string error;
    if(!hooks.reset_latch(error) || hooks.canceled()) {out.message+=error;return false;}
    out.message+=error+"; ready for NEW goals; no previous trajectory resumed";
    return true;
  };
  if(!enable_all()) {
    for(const auto& stop:hooks.drives) {
      const auto result=stop(false);out.message+="rollback disable: "+result.message+"; ";
    }
    return out;
  }
  out.success=out.drives_enabled=out.motion_ready=true;
  return out;
}
} // namespace motion_control_pkg::execution
