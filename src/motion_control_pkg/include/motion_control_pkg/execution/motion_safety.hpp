#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace motion_control_pkg::execution {
using SafetyClock = std::chrono::steady_clock;
using SafetyMask = unsigned;
inline constexpr SafetyMask kLeftSafety = 1, kRightSafety = 2, kLegSafety = 4, kHeadSafety = 8;
inline constexpr SafetyMask kArmSafety = kLeftSafety | kRightSafety;
inline constexpr SafetyMask kBodySafety = kLegSafety | kHeadSafety;
inline constexpr SafetyMask kAllSafety = 15;

struct SafetyJoint {
  std::string name;
  double lower{}, upper{}, velocity{};
};
struct SafetySample {
  std::vector<double> position, velocity;
  SafetyClock::time_point acquired{};
  std::uint64_t sequence{};
  bool wraps_at_million{false};
  // Optional controller output echo in radians; never used as the safety target.
  std::vector<double> controller_command;
};
struct SafetyPolicy {
  double tracking_error{0.02}, hard_error{0.08}, tracking_dwell{0.08};
  double max_feedback_age{0.1}, stop_timeout{3.0};
  bool arm_error_enabled{false};
  double arm_error_limit_rad{0.02}, arm_error_hard_limit_rad{0.08};
  bool body_error_enabled{false};
};
struct HeadStartupHomePolicy;

// Shared command fence and independent measured-state monitor. A software fence
// freezes future setpoint updates; it is NOT a drive Quick Stop, brake or STO.
// No automatic drive reset, mode change, disable or re-enable is performed here.
class MotionSafety : public std::enable_shared_from_this<MotionSafety> {
public:
  using Read = std::function<bool(SafetySample&, std::string&)>;
  explicit MotionSafety(SafetyPolicy policy = {});
  ~MotionSafety();
  MotionSafety(const MotionSafety&) = delete;
  MotionSafety& operator=(const MotionSafety&) = delete;
  void addChannel(SafetyMask channel, std::vector<SafetyJoint> joints, Read read);
  void start();
  void stop();
  // Deterministic offline tests can drive acquisition explicitly, without a worker.
  void poll(SafetyClock::time_point now = SafetyClock::now());
  // One startup attempt; only its owning worker may send head-only commands.
  // Other channels keep their URDF limits. No fault clear or drive enable.
  void configureHeadStartupHome(const HeadStartupHomePolicy& policy);
  bool beginHeadStartupHome(std::vector<std::string>& names,
      std::vector<double>& positions, std::string& error);
  bool endHeadStartupHome(bool success, std::string& error);
  // Constraints run on validated measured snapshots, never the planning model.
  using StateConstraint = std::function<bool(const std::vector<std::string>&,
      const std::vector<double>&, std::string&)>;
  std::uint64_t addStateConstraint(SafetyMask required, StateConstraint check, std::string& error);
  void removeStateConstraint(std::uint64_t id);

  class CommandPermit {
  public:
    CommandPermit(CommandPermit&&) noexcept = default;
    explicit operator bool() const { return allowed_; }
    bool validate(SafetyMask channel, const std::vector<double>& radians);
    bool committed(SafetyMask channel, const std::vector<double>& radians);
    void failed(const std::string& reason);
  private:
    friend class MotionSafety;
    CommandPermit(MotionSafety&, SafetyMask, std::string&, bool configuration = false);
    MotionSafety* owner_;
    std::unique_lock<std::mutex> lock_;
    SafetyMask channels_{};
    bool allowed_{false};
    SafetyClock::time_point acquired_{};
  };
  CommandPermit command(SafetyMask channels, std::string& error);
  CommandPermit configuration(SafetyMask channels, std::string& error);
  void requestStop(const std::string& reason);
  // Independent idle-only options for kArmSafety and kBodySafety (legs/waist/head).
  // Both default OFF. Position-error trips stop the
  // current execution and release the fence only after measured standstill and
  // all old executions exit. Other fault classes retain their software latch.
  bool setErrorTrackingEnabled(SafetyMask group, bool enabled, double error_rad,
      double hard_error_rad, std::string& detail);
  bool reset(std::string& error);
  bool ready(std::string& error) const;
  bool latched() const;
  bool workersIdle() const;
  std::string status() const;
  void enterExecution();
  void leaveExecution();
  bool finish(SafetyMask channels, bool success, std::string& detail);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Installed only by the production composition root. Planning-only/offline
// instances have no implicit hardware or background threads.
void installMotionSafety(const std::shared_ptr<MotionSafety>& safety);
std::shared_ptr<MotionSafety> motionSafety();

class MotionSafetyScope {
public:
  explicit MotionSafetyScope(SafetyMask channels);
  ~MotionSafetyScope();
  MotionSafetyScope(const MotionSafetyScope&) = delete;
  bool finish(bool success, std::string& detail);
  static bool beforeTerminal(bool success, std::string& detail);
private:
  std::shared_ptr<MotionSafety> safety_;
  SafetyMask channels_;
  bool finished_{false};
  MotionSafetyScope* previous_;
  static thread_local MotionSafetyScope* current_;
};
}  // namespace motion_control_pkg::execution
