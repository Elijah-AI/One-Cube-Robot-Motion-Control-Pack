#pragma once

#include "motion_control_pkg/ethercat_bus_manager.hpp"
#include "motion_control_pkg/execution/joint_completion.hpp"
#include "motion_control_pkg/execution/preplanned_dispatch.hpp"
#include "motion_control_pkg/execution/motion_safety.hpp"
#include "motion_control_pkg/motion/precision_approach.hpp"
#include "motion_control_pkg/sdk/ethercat_feedback_validity.hpp"

namespace motion_control_pkg::execution {
struct JointCompletionFeedback {
  std::vector<double> position, velocity;
  std::uint64_t sequence{0};
  JointCompletionGuard::Time acquired{};
};

inline bool readEthercatChainFeedback(const std::shared_ptr<miracle::EthercatBusManager>& bus,
    const std::vector<std::uint16_t>& ids, JointCompletionFeedback& out, std::string& error,
    bool require_enabled = false) {
  std::vector<DriverSDK::motorActualStruct> data;
  JointCompletionGuard::Time acquired;
  if (!bus || !bus->getTimedMotorActualSnapshot(data, acquired, out.sequence)) {
    error = "EtherCAT feedback unavailable"; return false;
  }
  const auto now = JointCompletionGuard::Clock::now();
  if (out.sequence == 0 || acquired == JointCompletionGuard::Time{} || acquired > now ||
      now - acquired >= std::chrono::milliseconds(100)) {
    error = "EtherCAT feedback stale or invalid"; return false;
  }
  out.position.clear(); out.velocity.clear();
  out.acquired = acquired;
  for (auto id : ids) {
    if (id >= data.size()) { error = "missing EtherCAT motor"; return false; }
    const auto& motor = data[id];
    if (!sdk::ethercatFeedbackValid(motor) || !std::isfinite(motor.pos) || !std::isfinite(motor.vel)) {
      error = "invalid EtherCAT motor feedback: " + std::to_string(id); return false;
    }
    if (require_enabled && !sdk::ethercatFeedbackOperational(motor)) {
      error = "EtherCAT motor not operation-enabled: id=" + std::to_string(id) +
          " alias=" + std::to_string(id+1) + " status=" + std::to_string(motor.statusWord);
      return false;
    }
    out.position.push_back(motor.pos); out.velocity.push_back(motor.vel);
  }
  error.clear(); return !ids.empty();
}

// This loop is also used by hardware-free tests with advancing measured frames.
// No command is sent during settling: EthercatBusManager keeps the original
// last setpoint active. Waiting never resets/re-enables a drive or changes modes.
template<class Read, class Check, class Timing = SteadyDispatchTiming>
bool waitForJointCompletion(const std::vector<double>& target,
    JointCompletionGuard::Time deadline, const PrecisionPolicy& policy,
    Read read, Check check, double& position_error, std::string& error, Timing timing = {}) {
  using Guard = JointCompletionGuard;
  policy.validate();
  const auto started = timing.now();
  const auto timeout = std::chrono::duration_cast<Guard::Clock::duration>(
      std::chrono::duration<double>(policy.completion_timeout));
  Guard guard(target, started, std::min(deadline, started + timeout), false, policy.completion);
  while (true) {
    JointCompletionFeedback measured;
    if (!check(error) || !read(measured,error)) return false;
    const auto now = timing.now();
    if (measured.acquired == Guard::Time{} || measured.acquired > now ||
        now - measured.acquired >= std::chrono::milliseconds(100)) {
      error = "completion feedback stale or invalid"; return false;
    }
    const auto state = guard.update(measured.sequence,measured.position,measured.velocity,now,error);
    position_error = guard.positionError();
    if (state == Guard::Status::Failed) return false;
    if (state == Guard::Status::Settled) return true;
    timing.sleep_until(now + std::chrono::milliseconds(10));
  }
}

// No hardware construction here. The gate is a preplanned zero-velocity
// approach point, not a reset of the command to an arbitrary measured position.
template<class Send, class Read, class Check, class Publish, class Timing = SteadyDispatchTiming>
bool executePrecisionTrajectory(const std::vector<std::vector<double>>& path,
    Send send, Read read, Check check, Publish publish,
    JointCompletionGuard::Time deadline, double& position_error, std::string& error,
    const PrecisionPolicy& policy, std::size_t gate = motion::kNoApproachGate, Timing timing = {}) {
  using Guard = JointCompletionGuard;
  policy.validate();
  position_error = std::numeric_limits<double>::quiet_NaN();
  if (path.empty() || path.front().empty()) { error = "empty EtherCAT trajectory"; return false; }
  for (const auto& q : path)
    if (q.size() != path.front().size() || !std::all_of(q.begin(),q.end(),[](double x){return std::isfinite(x);})) {
      error = "invalid EtherCAT trajectory sample"; return false;
    }
  const bool staged = gate != motion::kNoApproachGate;
  if (staged && (gate == 0 || gate >= path.size()-1)) {error="invalid precision approach gate";return false;}
  JointCompletionFeedback measured;
  if (!check(error) || !read(measured,error)) return false;
  if (measured.position.size()!=path.front().size() || measured.velocity.size()!=path.front().size() ||
      measured.sequence==0 || measured.acquired==Guard::Time{} || measured.acquired>timing.now() ||
      timing.now()-measured.acquired>=std::chrono::milliseconds(100)) {
    error="invalid initial precision feedback";return false;
  }
  for (std::size_t j=0;j<path.front().size();++j)
    if (!std::isfinite(measured.position[j]) || !std::isfinite(measured.velocity[j]) ||
        std::abs(measured.position[j]-path.front()[j]) > kPlanningStartToleranceRad ||
        std::abs(measured.velocity[j]) > policy.completion.velocity_tolerance) {
      error="EtherCAT chain moved during planning or is not stationary";return false;
    }
  const auto budget = [&](std::size_t intervals, unsigned holds) {
    return intervals*kDualArmControlPeriodSeconds + holds*policy.completion.stable_seconds <
        std::chrono::duration<double>(deadline-timing.now()).count();
  };
  if (!budget(path.size()-1,staged?2:1)) {
    error="planned EtherCAT precision motion and settling exceed remaining timeout; no samples sent";return false;
  }
  auto cancel=std::make_shared<CancellationToken>();
  const auto stream = [&](std::size_t first,std::size_t count,bool delayed) {
    const auto start=timing.now()+(delayed?kDualArmControlPeriod:std::chrono::microseconds(0));
    return dispatchPreplannedSamples(count,start,cancel,[&](std::size_t i) {
      if (!check(error) || timing.now()>=deadline) {if(error.empty())error="precision motion timeout";return false;}
      if (!send(path[first+i])) {error="EtherCAT motor command rejected";return false;}
      // This callback is a latest-value handoff; ROS publication belongs to
      // the caller's independent progress timer, never this 4 ms sample.
      publish(95.0F*(first+i+1)/path.size());
      return true;
    },error,timing);
  };
  if (!stream(0,staged?gate+1:path.size(),false)) return false;
  if (staged) {
    publish(95.0F*(gate+1)/path.size());
    auto approach_policy=policy;
    // This is a coarse handover check, deliberately separate from final accuracy.
    approach_policy.completion.position_tolerance=kPlanningStartToleranceRad;
    double handover_error = 0;
    const auto gate_read = [&](JointCompletionFeedback& state, std::string& reason) {
      if (!read(state,reason)) return false;
      position_error = std::numeric_limits<double>::quiet_NaN();
      if (state.position.size()==path.back().size()) {
        position_error=0;
        for (std::size_t j=0;j<state.position.size();++j) {
          if (!std::isfinite(state.position[j])) {position_error=std::numeric_limits<double>::quiet_NaN();break;}
          position_error=std::max(position_error,std::abs(state.position[j]-path.back()[j]));
        }
      }
      return true;
    };
    if (!waitForJointCompletion(path[gate],deadline,approach_policy,gate_read,check,handover_error,error,timing)) {
      error="precision approach handover: "+error;return false;
    }
    if (!budget(path.size()-1-gate,1)) {error="insufficient timeout for precision suffix after handover";return false;}
    // No new action, no retries: send the validated suffix once, continuing from
    // the last commanded (and now settled) approach point.
    if (!stream(gate+1,path.size()-1-gate,true)) return false;
  }
  publish(95.0F);
  return waitForJointCompletion(path.back(),deadline,policy,read,check,position_error,error,timing);
}

template<class Send, class Check, class Publish>
bool executeEthercatTrajectory(const std::vector<std::vector<double>>& path,
    const std::shared_ptr<miracle::EthercatBusManager>& bus,
    const std::vector<std::uint16_t>& ids, Send send, Check check, Publish publish,
    JointCompletionGuard::Time deadline, double& position_error, std::string& error,
    const PrecisionPolicy& policy = {}, std::size_t gate = motion::kNoApproachGate) {
  if (path.empty() || path.front().size()!=ids.size()) {error="invalid EtherCAT path dimensions";return false;}
  return executePrecisionTrajectory(path,send,[&](JointCompletionFeedback& measured,std::string& reason) {
    // Streaming and completion still require these axes to remain enabled.
    return readEthercatChainFeedback(bus,ids,measured,reason,true);
  },[&](std::string& reason) {
    if (!check(reason)) return false;
    const auto safety=motionSafety();
    return !safety || safety->ready(reason);
  },publish,deadline,position_error,error,policy,gate);
}
}  // namespace motion_control_pkg::execution
