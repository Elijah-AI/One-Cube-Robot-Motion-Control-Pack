#pragma once

#include "motion_control_pkg/execution/joint_completion.hpp"
#include <array>
#include <limits>

namespace motion_control_pkg::execution {

struct ArmTcpCompletionError {
  double translation_m{0.0};
  double rotation_rad{0.0};
  bool valid() const {
    return std::isfinite(translation_m) && translation_m >= 0.0 &&
        std::isfinite(rotation_rad) && rotation_rad >= 0.0;
  }
  bool reached() const {
    return valid() && translation_m <= kGoalTranslationToleranceM &&
        rotation_rad <= kGoalRotationToleranceRad;
  }
};

// Both arms must be settled in the current snapshot. Never latch an earlier
// success while the other arm is still moving. Use the Action's remaining
// deadline; a second, hidden three-second deadline must not cut it short.
class DualArmCompletionGuard {
public:
  using Clock = ArmCompletionGuard::Clock;
  using Time = Clock::time_point;
  using Status = ArmCompletionGuard::Status;
  using TcpErrors = std::array<ArmTcpCompletionError, 2>;

  DualArmCompletionGuard(std::vector<double> left, std::vector<double> right,
                        Time started, Time deadline)
      : arms_{{ArmCompletionGuard(std::move(left), started, deadline),
               ArmCompletionGuard(std::move(right), started, deadline)}},
        started_(started), deadline_(deadline) {}

  Status update(const sdk::DualArmDeviceState& measured, Time now,
                std::string& reason, const std::optional<TcpErrors>& tcp = std::nullopt) {
    // Preserve both previous observations at the deadline. Calling the left
    // guard first would report its timeout even if only the right was waiting.
    if (now >= deadline_) {
      reason = timeoutReason(now);
      return Status::Failed;
    }
    last_observed_ = now;
    tcp_ = tcp;
    bool failed = false;
    bool settled = true;
    for (std::size_t arm = 0; arm < arms_.size(); ++arm) {
      std::string error;
      status_[arm] = arms_[arm].update(measured.arms[arm], now, error,
                                      !tcp || (*tcp)[arm].reached());
      detail_[arm] = status_[arm] == Status::Failed ? error : arms_[arm].diagnostics();
      if (tcp && !(*tcp)[arm].valid()) {
        status_[arm] = Status::Failed;
        detail_[arm] = "invalid TCP completion error; " + detail_[arm];
      }
      failed = failed || status_[arm] == Status::Failed;
      if (status_[arm] == Status::Settled) {
        if (!settled_at_[arm]) settled_at_[arm] = now;
      } else {
        settled_at_[arm].reset();
        settled = false;
      }
    }
    if (failed) {
      reason = "dual-arm completion feedback failed: " + diagnostics(now);
      return Status::Failed;
    }
    reason = settled ? "" : diagnostics(now);
    return settled ? Status::Settled : Status::Waiting;
  }

  std::string timeoutReason(Time now) const {
    return "dual-arm completion timed out: " + diagnostics(now);
  }

  double completionSkewSeconds() const {
    if (!settled_at_[0] || !settled_at_[1]) return std::numeric_limits<double>::quiet_NaN();
    return std::abs(std::chrono::duration<double>(*settled_at_[0] - *settled_at_[1]).count());
  }

private:
  static const char* statusName(Status status) {
    return status == Status::Settled ? "settled" : status == Status::Failed ? "failed" : "waiting";
  }
  std::string diagnostics(Time now) const {
    std::string result = "completion_elapsed_sec=" +
        std::to_string(std::chrono::duration<double>(now - started_).count()) +
        ", completion_budget_sec=" +
        std::to_string(std::chrono::duration<double>(deadline_ - started_).count()) +
        ", last_snapshot_age_sec=" + (last_observed_ ?
        std::to_string(std::chrono::duration<double>(now - *last_observed_).count()) : "unavailable");
    for (std::size_t arm = 0; arm < arms_.size(); ++arm) {
      result += std::string(arm == 0 ? "; left{" : "; right{") +
          "last_status=" + statusName(status_[arm]) + ", " + detail_[arm];
      if (tcp_) {
        result += ", tcp_position_error_m=" + std::to_string((*tcp_)[arm].translation_m) +
            ", tcp_position_tolerance_m=" + std::to_string(kGoalTranslationToleranceM) +
            ", tcp_rotation_error_rad=" + std::to_string((*tcp_)[arm].rotation_rad) +
            ", tcp_rotation_tolerance_rad=" + std::to_string(kGoalRotationToleranceRad);
      }
      result += "}";
    }
    return result;
  }
  std::array<ArmCompletionGuard, 2> arms_;
  Time started_, deadline_;
  std::optional<Time> last_observed_;
  std::array<Status, 2> status_{{Status::Waiting, Status::Waiting}};
  std::array<std::string, 2> detail_{{"waiting for fresh measured feedback", "waiting for fresh measured feedback"}};
  std::array<std::optional<Time>, 2> settled_at_;
  std::optional<TcpErrors> tcp_;
};
}  // namespace motion_control_pkg::execution
