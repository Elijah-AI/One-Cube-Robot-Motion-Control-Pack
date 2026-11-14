#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "motion_control_pkg/sdk/dual_arm_device_interface.hpp"
#include "motion_control_pkg/execution/control_period.hpp"
#include "motion_control_pkg/execution/precision_policy.hpp"

namespace motion_control_pkg::execution {

inline constexpr double kGoalJointToleranceRad = 0.01;
inline constexpr double kPlanningStartToleranceRad = 0.01;
inline constexpr double kGoalVelocityToleranceRadSec = 0.01;
inline constexpr double kGoalTranslationToleranceM = 0.003;
inline constexpr double kGoalRotationToleranceRad = 0.02;
inline constexpr auto kGoalSettlingDuration = std::chrono::milliseconds(100);

inline bool hasCompletionBudget(std::size_t samples, double remaining_seconds,
                                double stable_seconds = .1) {
  return samples > 0 && std::isfinite(remaining_seconds) &&
      std::isfinite(stable_seconds) && stable_seconds >= .1 &&
      (samples - 1) * kDualArmControlPeriodSeconds +
          stable_seconds < remaining_seconds;
}

// Shared by ordinary arms, EtherCAT chains and whole-body completion. Only
// advancing device frames count towards settling; repeated polls cannot succeed.
class JointCompletionGuard {
public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  enum class Status { Waiting, Settled, Failed };

  JointCompletionGuard(std::vector<double> target, Time started, Time deadline,
                       bool sdk_sequence = false, JointCompletionOptions options = {})
      : target_(std::move(target)), deadline_(deadline), last_fresh_(started),
        sdk_sequence_(sdk_sequence), options_(options) { options_.validate(); }

  Status update(std::uint64_t sequence, const std::vector<double>& q,
      const std::vector<double>& v, Time now, std::string& reason,
      bool stopped = true, bool task_reached = true) {
    if (now >= deadline_) return fail("completion timed out: " + diagnostics(), reason);
    if (target_.empty() || q.size() != target_.size() || v.size() != q.size())
      return fail("invalid completion dimensions", reason);
    if ((!sdk_sequence_ && sequence == 0) || (sdk_sequence_ && sequence > 1000000) ||
        (sequence_ && sequence < *sequence_ &&
         !(sdk_sequence_ && *sequence_ > 900000 && sequence < 100000)))
      return fail("completion feedback sequence invalid or reversed", reason);
    const bool fresh = !sequence_ || sequence != *sequence_;
    if (now < last_fresh_ || now - last_fresh_ >= std::chrono::milliseconds(100))
      return fail("completion feedback stopped advancing", reason);
    double max_speed = 0;
    error_ = 0;
    std::size_t worst_joint = 0;
    for (std::size_t j = 0; j < q.size(); ++j) {
      if (!std::isfinite(q[j]) || !std::isfinite(v[j]) || !std::isfinite(target_[j]))
        return fail("non-finite completion feedback or target", reason);
      const double error = std::abs(q[j] - target_[j]);
      if (error > error_) {error_ = error; worst_joint = j;}
      max_speed = std::max(max_speed, std::abs(v[j]));
    }
    if (fresh && sequence_) {
      const double dt = std::chrono::duration<double>(now - last_fresh_).count();
      if (dt <= 0) return fail("completion sampling clock did not advance", reason);
      for (std::size_t j = 0; j < q.size(); ++j)
        max_speed = std::max(max_speed, std::abs(q[j] - previous_[j]) / dt);
    }
    if (fresh) {
      previous_ = q;
      sequence_ = sequence;
      last_fresh_ = now;
    }
    last_reason_ = "max_position_error_rad=" + std::to_string(error_) +
        ", worst_joint_index=" + std::to_string(worst_joint) +
        ", target_rad=" + std::to_string(target_[worst_joint]) +
        ", actual_rad=" + std::to_string(q[worst_joint]) +
        ", tolerance_rad=" + std::to_string(options_.position_tolerance) +
        ", max_velocity_rad_s=" + std::to_string(max_speed) +
        ", task_reached=" + std::to_string(task_reached) +
        ", stopped=" + std::to_string(stopped);
    reason = last_reason_;
    // Detect a stationary residual, not normal deceleration. Repeated polls
    // neither advance progress nor make the no-progress timer eligible.
    if (fresh && options_.stall_seconds > 0) {
      if (!stopped || max_speed > options_.velocity_tolerance || error_ <= options_.position_tolerance) {
        progress_since_.reset();
      } else if (!progress_since_ || progress_error_ - error_ >= options_.progress_rad) {
        progress_since_ = now; progress_error_ = error_;
      } else if (std::chrono::duration<double>(now - *progress_since_).count() >= options_.stall_seconds) {
        return fail("completion stalled at original target (no automatic offset or retry): " + last_reason_, reason);
      }
    }
    if (!stopped || !task_reached || error_ > options_.position_tolerance ||
        max_speed > options_.velocity_tolerance) {
      stable_since_.reset(); stable_samples_ = 0;
      return Status::Waiting;
    }
    if (fresh) {
      if (!stable_since_) stable_since_ = now;
      ++stable_samples_;
    }
    if (stable_samples_ < 5 || !stable_since_ ||
        std::chrono::duration<double>(last_fresh_ - *stable_since_).count() < options_.stable_seconds)
      return Status::Waiting;
    reason.clear();
    return Status::Settled;
  }
  double positionError() const { return error_; }
  std::string diagnostics() const {
    const double stable_seconds = stable_since_ ?
        std::chrono::duration<double>(last_fresh_ - *stable_since_).count() : 0.0;
    return last_reason_ + ", velocity_tolerance_rad_s=" + std::to_string(options_.velocity_tolerance) +
        ", stable_samples=" + std::to_string(stable_samples_) +
        ", stable_seconds=" + std::to_string(stable_seconds) +
        ", required_stable_seconds=" + std::to_string(options_.stable_seconds);
  }

private:
  static Status fail(const std::string& message, std::string& reason) {
    reason = message; return Status::Failed;
  }
  std::vector<double> target_, previous_;
  Time deadline_, last_fresh_;
  bool sdk_sequence_;
  JointCompletionOptions options_;
  std::optional<std::uint64_t> sequence_;
  std::optional<Time> stable_since_;
  std::optional<Time> progress_since_;
  double progress_error_{0};
  unsigned stable_samples_{0};
  double error_{0};
  std::string last_reason_{"waiting for fresh measured feedback"};
};

class ArmCompletionGuard {
public:
  using Clock = JointCompletionGuard::Clock;
  using Status = JointCompletionGuard::Status;
  ArmCompletionGuard(std::vector<double> target, Clock::time_point start,
                     Clock::time_point deadline) : guard_(std::move(target), start, deadline, true) {}

  Status update(const sdk::ArmFeedback& state, Clock::time_point now,
                std::string& error, bool task_reached = true) {
    if (!sdk::armFeedbackOperational(state) || state.feedback_sequence < 0) {
      error = "invalid arm completion state: error_code=" + std::to_string(state.error_code) +
          ", state=" + std::to_string(state.current_state);
      return Status::Failed;
    }
    constexpr double radians = 0.017453292519943295;
    std::vector<double> q(7), v(7);
    for (std::size_t j = 0; j < 7; ++j) {
      q[j] = state.joint_position[j] * radians;
      v[j] = state.joint_velocity[j] * radians;
    }
    return guard_.update(state.feedback_sequence, q, v, now, error, state.low_speed, task_reached);
  }
  double positionError() const { return guard_.positionError(); }
  std::string diagnostics() const { return guard_.diagnostics(); }
private:
  JointCompletionGuard guard_;
};
}  // namespace motion_control_pkg::execution
