#pragma once

#include "motion_control_pkg/sdk/dual_arm_device_interface.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace motion_control_pkg::execution {

struct WholeBodyCompletionFeedback {
  sdk::DualArmDeviceState arms;  // SDK positions/velocities are degrees and degrees/s.
  std::array<double, 4> leg_positions{}, leg_velocities{};  // rad, rad/s
  std::uint64_t leg_sequence{0};
};

// Completion must be based on new controller frames, not repeated polls of a
// cached low_speed bit. Used even when static COM feedback is disabled.
class WholeBodyCompletionGuard {
public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  enum class Status { Waiting, Settled, Failed };

  WholeBodyCompletionGuard(std::vector<double> left, std::vector<double> right,
      std::vector<double> legs, Time started, Time deadline)
    : targets_{std::move(left), std::move(right)}, legs_(std::move(legs)),
      deadline_(deadline), last_fresh_{started, started, started} {}

  Status update(const WholeBodyCompletionFeedback& feedback, Time now, std::string& reason) {
    if (now >= deadline_) {
      reason = "whole-body completion timed out: " + last_reason_;
      return Status::Failed;
    }
    if (legs_.size() != 4 || (targets_[0].empty() && targets_[1].empty()))
      return fail("invalid whole-body completion targets", reason);
    bool fresh = true, stopped = true;
    double max_position_error = 0, max_velocity = 0;
    std::vector<double> positions;
    std::string moving;
    for (std::size_t arm = 0; arm < 2; ++arm) {
      if (targets_[arm].empty()) continue;
      if (targets_[arm].size() != 7) return fail("invalid arm completion target", reason);
      const auto& state = feedback.arms.arms[arm];
      const std::string side = arm == 0 ? "left arm" : "right arm";
      if (state.error_code != 0 || state.current_state >= 100)
        return fail(side + " completion fault: error_code=" + std::to_string(state.error_code) +
            ", current_state=" + std::to_string(state.current_state), reason);
      const int sequence = state.feedback_sequence;
      if (sequence < 0 || sequence > 1000000 ||
          (sequence < arm_sequences_[arm] && !(arm_sequences_[arm] > 900000 && sequence < 100000)))
        return fail(side + " completion feedback sequence invalid or reversed", reason);
      const bool advanced = sequence != arm_sequences_[arm];
      fresh = fresh && advanced && arm_sequences_[arm] >= 0;
      if (advanced) { last_fresh_[arm] = now; arm_sequences_[arm] = sequence; }
      if (now - last_fresh_[arm] >= std::chrono::milliseconds(100))
        return fail(side + " completion feedback stopped advancing", reason);
      if (!state.low_speed) { stopped = false; moving += side + " low_speed=false; "; }
      constexpr double radians = 0.017453292519943295;
      for (std::size_t j = 0; j < 7; ++j) {
        positions.push_back(state.joint_position[j] * radians);
        if (!measure(state.joint_position[j] * radians, state.joint_velocity[j] * radians,
                     targets_[arm][j], max_position_error, max_velocity))
          return fail(side + " non-finite completion feedback or target", reason);
      }
    }
    if (feedback.leg_sequence == 0 || feedback.leg_sequence < leg_sequence_)
      return fail("leg/waist completion feedback sequence invalid or reversed", reason);
    const bool advanced = feedback.leg_sequence != leg_sequence_;
    fresh = fresh && advanced && leg_sequence_ != 0;
    if (advanced) { last_fresh_[2] = now; leg_sequence_ = feedback.leg_sequence; }
    if (now - last_fresh_[2] >= std::chrono::milliseconds(100))
      return fail("leg/waist completion feedback stopped advancing", reason);
    for (std::size_t j = 0; j < 4; ++j) {
      positions.push_back(feedback.leg_positions[j]);
      if (!measure(feedback.leg_positions[j], feedback.leg_velocities[j], legs_[j],
                   max_position_error, max_velocity))
        return fail("non-finite leg/waist completion feedback or target", reason);
    }
    // Also require measured positions to stop changing. This prevents a zero
    // velocity field or early low_speed bit from hiding residual movement.
    if (fresh) {
      if (previous_stamp_) {
        const double dt = std::chrono::duration<double>(now - *previous_stamp_).count();
        if (dt <= 0) return fail("completion sampling clock did not advance", reason);
        for (std::size_t j = 0; j < positions.size(); ++j)
          max_velocity = std::max(max_velocity, std::abs(positions[j]-previous_positions_[j])/dt);
      }
      previous_positions_ = std::move(positions);
      previous_stamp_ = now;
    }

    stopped = stopped && max_position_error <= 0.01 && max_velocity <= 0.01;
    last_reason_ = moving + "max_position_error_rad=" + std::to_string(max_position_error) +
        ", max_velocity_rad_s=" + std::to_string(max_velocity);
    reason = last_reason_;
    if (!stopped) { stable_since_.reset(); stable_samples_ = 0; return Status::Waiting; }
    if (!fresh) return Status::Waiting;
    if (!stable_since_) stable_since_ = now;
    ++stable_samples_;
    if (stable_samples_ < 5 || now - *stable_since_ < std::chrono::milliseconds(100))
      return Status::Waiting;
    reason.clear();
    return Status::Settled;
  }

private:
  static bool measure(double position, double velocity, double target, double& error, double& speed) {
    if (!std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(target)) return false;
    error = std::max(error, std::abs(position - target));
    speed = std::max(speed, std::abs(velocity));
    return true;
  }
  static Status fail(const std::string& message, std::string& reason) {
    reason = message; return Status::Failed;
  }
  std::array<std::vector<double>, 2> targets_;
  std::vector<double> legs_;
  Time deadline_;
  std::array<Time, 3> last_fresh_;
  std::array<int, 2> arm_sequences_{{-1, -1}};
  std::uint64_t leg_sequence_{0};
  std::optional<Time> stable_since_;
  std::optional<Time> previous_stamp_;
  std::vector<double> previous_positions_;
  unsigned stable_samples_{0};
  std::string last_reason_{"waiting for fresh stopped feedback"};
};
}  // namespace motion_control_pkg::execution
