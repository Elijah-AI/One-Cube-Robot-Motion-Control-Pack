#include "motion_control_pkg/execution/shutdown_request.hpp"
#pragma once
#include "motion_control_pkg/execution/motion_safety.hpp"
#include "motion_control_pkg/loong_driver_sdk.h"
#include <array>
#include <algorithm>
#include <optional>

namespace motion_control_pkg::execution {
class EthercatCommandSafety {
public:
  EthercatCommandSafety(const std::vector<std::uint16_t>& ids,
                       const std::vector<DriverSDK::motorTargetStruct>& targets) : safety_(motionSafety()) {
    if (shutdownRequested()) {error_="orderly shutdown in progress";return;}
    if (!safety_) {valid_=true;return;}
    if (ids.size()!=targets.size() || ids.empty()) {error_="empty motor list or target count mismatch";return;}
    for (auto id:ids) {
      if (id>=26 && id<=29) mask_|=kLegSafety;
      else if (id==30 || id==31) mask_|=kHeadSafety;
      else {error_="unsupported motor_id="+std::to_string(id);return;}
    }
    const auto collect=[&](const auto& expected,std::vector<double>& q) {
      for(auto id:expected) {
        const auto it=std::find(ids.begin(),ids.end(),id);
        if(it==ids.end()){error_="missing motor_id="+std::to_string(id);return false;}
        const auto n=static_cast<std::size_t>(it-ids.begin());
        if(targets[n].enabled!=1){error_="safety command requires enabled=1 for motor_id="+std::to_string(id);return false;}
        q.push_back(targets[n].pos);
      }return true;
    };
    if ((mask_&kLegSafety) && !collect(std::array<std::uint16_t,4>{{29,28,27,26}},legs_))return;
    if ((mask_&kHeadSafety) && !collect(std::array<std::uint16_t,2>{{31,30}},head_))return;
    if(ids.size()!=legs_.size()+head_.size()){error_="duplicate or unexpected motor entries";return;}
    permit_.emplace(safety_->command(mask_,error_));
    valid_=static_cast<bool>(*permit_) &&
      (!(mask_&kLegSafety)||permit_->validate(kLegSafety,legs_)) &&
      (!(mask_&kHeadSafety)||permit_->validate(kHeadSafety,head_));
    if (!valid_ && error_.empty()) error_="joint command violates URDF bounds or dimensions";
  }
  explicit operator bool() const {return valid_;}
  const std::string& error() const {return error_;}
  bool committed(){
    if(!permit_)return valid_;
    return (!(mask_&kLegSafety)||permit_->committed(kLegSafety,legs_)) &&
      (!(mask_&kHeadSafety)||permit_->committed(kHeadSafety,head_));
  }
  void failed(){if(permit_)permit_->failed("EtherCAT target submission failed");}
private:
  std::shared_ptr<MotionSafety> safety_;
  std::optional<MotionSafety::CommandPermit> permit_;
  SafetyMask mask_{};
  std::vector<double> legs_,head_;
  bool valid_{false};
  std::string error_;
};
}  // namespace motion_control_pkg::execution
