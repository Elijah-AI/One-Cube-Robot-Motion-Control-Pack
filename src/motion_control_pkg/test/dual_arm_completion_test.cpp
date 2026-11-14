#include "motion_control_pkg/execution/dual_arm_completion.hpp"
#include <iostream>
#include <stdexcept>

using namespace motion_control_pkg::execution;
using Guard = DualArmCompletionGuard;
using Status = Guard::Status;
using namespace std::chrono_literals;

static void check(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
static bool has(const std::string& text, const char* token) {
  return text.find(token) != std::string::npos;
}
struct Fixture {
  Guard::Time start{1s};
  motion_control_pkg::sdk::DualArmDeviceState measured;
  Guard::TcpErrors tcp{};
  Fixture() {
    for (auto& arm : measured.arms) {
      arm.current_state = 1;
      arm.low_speed = true;
      arm.feedback_sequence = 100;
    }
  }
  void advance() { for (auto& arm : measured.arms) ++arm.feedback_sequence; }
  Guard make(std::chrono::milliseconds timeout) {
    return Guard(std::vector<double>(7, 0), std::vector<double>(7, 0), start, start + timeout);
  }
};

int main() try {
  unsigned cases = 0;
  const auto run = [&](const char* name, auto test) {
    test(); ++cases; std::cout << "PASS " << name << '\n';
  };
  run("both arms still require 100 ms of advancing feedback", [] {
    Fixture f; auto guard = f.make(1s); std::string reason;
    for (int ms = 0; ms <= 100; ms += 20) {
      f.advance();
      const auto status = guard.update(f.measured, f.start + std::chrono::milliseconds(ms), reason, f.tcp);
      check(status == (ms == 100 ? Status::Settled : Status::Waiting), "premature or missing settlement");
    }
    check(reason.empty() && guard.completionSkewSeconds() == 0, "success result changed");
  });
  run("late right TCP can settle after 3 s within the Action deadline", [] {
    Fixture f; auto guard = f.make(5s); std::string reason;
    for (int ms = 0; ms <= 3300; ms += 20) {
      f.advance(); f.tcp[1].translation_m = ms < 3200 ? 0.004 : 0.0;
      const auto status = guard.update(f.measured, f.start + std::chrono::milliseconds(ms), reason, f.tcp);
      check(status == (ms == 3300 ? Status::Settled : Status::Waiting), "hidden 3 s deadline or lost stable window");
    }
    check(std::abs(guard.completionSkewSeconds() - 3.2) < 1e-9, "completion skew lost");
  });
  run("timeout identifies right TCP blocker and settled left", [] {
    Fixture f; auto guard = f.make(1200ms); std::string reason;
    f.tcp[1].translation_m = 0.004;
    for (int ms = 0; ms < 1200; ms += 20) {
      f.advance();
      check(guard.update(f.measured, f.start + std::chrono::milliseconds(ms), reason, f.tcp) == Status::Waiting,
            "unreached TCP completed");
    }
    check(guard.update(f.measured, f.start + 1200ms, reason, f.tcp) == Status::Failed, "deadline ignored");
    check(has(reason, "left{last_status=settled") && has(reason, "right{last_status=waiting") &&
          has(reason, "tcp_position_error_m=0.004000") && has(reason, "stable_samples=0") &&
          has(reason, "completion_budget_sec=1.200000") && has(reason, "last_snapshot_age_sec=0.020000"),
          "timeout masked right TCP or omitted freshness/stability diagnostics");
  });
  run("timeout also identifies left orientation blocker", [] {
    Fixture f; auto guard = f.make(200ms); std::string reason;
    f.tcp[0].rotation_rad = 0.03;
    for (int ms = 0; ms < 200; ms += 20) {
      f.advance(); guard.update(f.measured, f.start + std::chrono::milliseconds(ms), reason, f.tcp);
    }
    check(guard.update(f.measured, f.start + 200ms, reason, f.tcp) == Status::Failed, "orientation accepted");
    check(has(reason, "left{last_status=waiting") && has(reason, "right{last_status=settled") &&
          has(reason, "tcp_rotation_error_rad=0.030000"), "left orientation not diagnosed");
  });
  run("settled left must be checked again while waiting for right", [] {
    Fixture f; auto guard = f.make(1s); std::string reason;
    for (int ms = 0; ms <= 400; ms += 20) {
      f.advance();
      f.tcp[1].translation_m = ms < 200 ? 0.004 : 0.0;
      f.tcp[0].translation_m = ms >= 200 && ms < 300 ? 0.004 : 0.0;
      const auto status = guard.update(f.measured, f.start + std::chrono::milliseconds(ms), reason, f.tcp);
      check(status == (ms == 400 ? Status::Settled : Status::Waiting), "historical left success was latched");
    }
  });
  run("joint position velocity and SDK low-speed gates stay enforced", [] {
    for (int kind = 0; kind < 3; ++kind) {
      Fixture f; auto guard = f.make(200ms); std::string reason;
      if (kind == 0) f.measured.arms[1].joint_position[2] = 0.02 / 0.017453292519943295;
      if (kind == 1) f.measured.arms[1].joint_velocity[2] = 0.02 / 0.017453292519943295;
      if (kind == 2) f.measured.arms[1].low_speed = false;
      for (int ms = 0; ms < 200; ms += 20) {
        f.advance();
        check(guard.update(f.measured, f.start + std::chrono::milliseconds(ms), reason) == Status::Waiting,
              "position/velocity/low-speed gate bypassed");
      }
      check(guard.update(f.measured, f.start + 200ms, reason) == Status::Failed, "joint deadline ignored");
      check(has(reason, "right{last_status=waiting") && has(reason, "stable_samples=0"), "joint blocker omitted");
    }
  });
  run("frozen right feedback fails while left continues updating", [] {
    Fixture f; auto guard = f.make(5s); std::string reason;
    for (int ms = 0; ms <= 100; ms += 20) {
      ++f.measured.arms[0].feedback_sequence;
      const auto status = guard.update(f.measured, f.start + std::chrono::milliseconds(ms), reason);
      check(status == (ms == 100 ? Status::Failed : Status::Waiting), "frozen feedback accepted");
    }
    check(has(reason, "right{last_status=failed") && has(reason, "feedback stopped advancing"), "stale side omitted");
  });
  run("SDK faults and transitions fail with both sides in diagnostics", [] {
    for (int kind = 0; kind < 2; ++kind) {
      Fixture f; auto guard = f.make(5s); std::string reason;
      if (kind == 0) f.measured.arms[1].error_code = 7;
      else f.measured.arms[1].current_state = 103;
      check(guard.update(f.measured, f.start, reason) == Status::Failed, "SDK fault/transition accepted");
      check(has(reason, "left{") && has(reason, "right{last_status=failed"), "one side was omitted");
    }
  });
  run("invalid TCP and nonfinite joint values fail", [] {
    for (int kind = 0; kind < 3; ++kind) {
      Fixture f; auto guard = f.make(5s); std::string reason;
      if (kind == 0) f.tcp[1].rotation_rad = std::numeric_limits<double>::quiet_NaN();
      if (kind == 1) f.tcp[1].translation_m = -0.1;
      if (kind == 2) f.measured.arms[1].joint_position[2] = std::numeric_limits<double>::infinity();
      check(guard.update(f.measured, f.start, reason, f.tcp) == Status::Failed, "nonfinite value accepted");
    }
  });
  run("reversed sequences fail and valid SDK wrap still settles", [] {
    Fixture f; auto guard = f.make(1s); std::string reason;
    guard.update(f.measured, f.start, reason);
    --f.measured.arms[1].feedback_sequence;
    check(guard.update(f.measured, f.start + 20ms, reason) == Status::Failed, "sequence reversal accepted");
    Fixture wrap; auto wrapped = wrap.make(1s);
    for (int i = 0; i <= 5; ++i) {
      for (auto& arm : wrap.measured.arms) arm.feedback_sequence = i == 0 ? 999999 : i - 1;
      const auto status = wrapped.update(wrap.measured, wrap.start + 20ms * i, reason);
      check(status == (i == 5 ? Status::Settled : Status::Waiting), "valid wrap rejected");
    }
  });
  run("short remaining Action budget is never extended", [] {
    Fixture f; auto guard = f.make(150ms); std::string reason;
    for (int ms = 0; ms < 150; ms += 20) {
      f.advance(); f.tcp[1].translation_m = ms < 100 ? 0.004 : 0.0;
      check(guard.update(f.measured, f.start + std::chrono::milliseconds(ms), reason, f.tcp) == Status::Waiting,
            "insufficient stable duration accepted");
    }
    check(guard.update(f.measured, f.start + 150ms, reason, f.tcp) == Status::Failed, "Action budget extended");
    check(has(reason, "stable_seconds=0.040000"), "partial stable duration omitted");
  });
  run("deadline before first feedback reports unavailable snapshot", [] {
    Fixture f; auto guard = f.make(0ms); std::string reason;
    check(guard.update(f.measured, f.start, reason) == Status::Failed, "expired Action accepted");
    check(has(reason, "last_snapshot_age_sec=unavailable") && has(reason, "left{") && has(reason, "right{"),
          "missing snapshot diagnostics");
  });
  std::cout << "PASS dual-arm completion cases=" << cases << "; hardware_access=false\n";
  return 0;
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n'; return 1;
}
