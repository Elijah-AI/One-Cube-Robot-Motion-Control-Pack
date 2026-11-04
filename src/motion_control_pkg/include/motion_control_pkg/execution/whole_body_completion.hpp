#pragma once

#include "motion_control_pkg/execution/joint_completion.hpp"
#include <array>

namespace motion_control_pkg::execution {
struct WholeBodyCompletionFeedback {
  sdk::DualArmDeviceState arms;  // degrees, degrees/s
  std::array<double, 4> leg_positions{}, leg_velocities{};  // rad, rad/s
  std::uint64_t leg_sequence{0};
};

class WholeBodyCompletionGuard {
public:
  using Clock = JointCompletionGuard::Clock;
  using Time = Clock::time_point;
  using Status = JointCompletionGuard::Status;
  WholeBodyCompletionGuard(std::vector<double> left, std::vector<double> right,
      std::vector<double> legs, Time started, Time deadline, JointCompletionOptions leg_options = {})
      : legs_(std::move(legs), started, deadline, false, leg_options) {
    if (!left.empty()) arms_[0].emplace(std::move(left), started, deadline);
    if (!right.empty()) arms_[1].emplace(std::move(right), started, deadline);
  }
  Status update(const WholeBodyCompletionFeedback& measured, Time now, std::string& error,
                bool task_reached = true) {
    if (!arms_[0] && !arms_[1]) { error = "whole-body completion requires an arm"; return Status::Failed; }
    bool settled = true;
    std::string waiting;
    for (std::size_t arm = 0; arm < 2; ++arm) {
      if (!arms_[arm]) continue;
      const auto status = arms_[arm]->update(measured.arms.arms[arm], now, error, task_reached);
      if (status == Status::Failed) { error = "arm " + std::to_string(arm) + ": " + error; return status; }
      if (status != Status::Settled) { settled = false; waiting += "arm " + std::to_string(arm) + ": " + error + "; "; }
    }
    const auto status = legs_.update(measured.leg_sequence,
        {measured.leg_positions.begin(), measured.leg_positions.end()},
        {measured.leg_velocities.begin(), measured.leg_velocities.end()}, now, error, true, task_reached);
    if (status == Status::Failed) { error = "leg/waist: " + error; return status; }
    if (status != Status::Settled) { settled = false; waiting += "leg/waist: " + error; }
    error = settled ? "" : waiting;
    return settled ? Status::Settled : Status::Waiting;
  }
private:
  std::array<std::optional<ArmCompletionGuard>, 2> arms_;
  JointCompletionGuard legs_;
};
}  // namespace motion_control_pkg::execution
