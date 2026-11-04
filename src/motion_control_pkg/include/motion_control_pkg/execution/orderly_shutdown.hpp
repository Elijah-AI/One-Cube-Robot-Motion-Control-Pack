#pragma once
#include "motion_control_pkg/sdk/drive_control.hpp"
#include <functional>
#include <exception>
#include <algorithm>

namespace motion_control_pkg::execution {
// Single owner (main thread). The executor, PDO worker and acquisition remain
// alive until fresh post-command feedback has passed both confirmation windows.
class OrderlyShutdown {
public:
  using Clock = sdk::DriveClock;
  enum class Phase { Stopping, Disabling, Releasing, Complete, FailedHold };
  struct Hooks {
    std::function<bool()> idle;
    std::function<bool(std::vector<sdk::DriveAxis>&, std::string&, bool)> read;
    std::function<bool(std::string&)> disable;
    std::function<bool(std::string&)> release;
  };
  OrderlyShutdown(Hooks hooks, std::size_t expected_axes, Clock::time_point now)
    : hooks_(std::move(hooks)), expected_axes_(expected_axes), since_(now), window_(now) {}

  void tick(Clock::time_point now) try {
    if (phase_ == Phase::Complete || phase_ == Phase::FailedHold) return;
    const bool releasing=phase_==Phase::Releasing;
    const bool disabling=phase_!=Phase::Stopping;
    std::vector<sdk::DriveAxis> axes;
    const bool read_ok=hooks_.read(axes,detail_,disabling);
    const bool idle=hooks_.idle();
    // Arm acquisition stamps are assigned by the read hook. Validate against
    // the time AFTER reading, never against the earlier tick invocation time.
    const auto observed=std::max(now,Clock::now());
    const bool count_ok=axes.size()==expected_axes_;
    if (!read_ok || !idle || !count_ok) axes.clear();
    // Preflight requires stationary feedback regardless of servo mode. It does
    // not require drives to be enabled, or mask a fault as healthy motion data.
    if (!disabling) for (auto& a:axes) a.reset_allowed=true;
    const bool confirmed=read_ok && idle && count_ok && (expected_axes_==0 ||
        window_.update(axes,false,!disabling,observed));
    if (!confirmed) {
      // Always invalidate a window on failed reads / outstanding workers.
      if (axes.empty()) window_.update({},false,!disabling,observed);
      if (!idle) detail_="accepted/planning/executing/tool/startup workers still active";
      if (observed-since_ >= (disabling ? std::chrono::seconds(5) : std::chrono::seconds(10)))
        fail(std::string(disabling ? "disable confirmation timeout: " : "standstill timeout: ")+detail_);
      return;
    }
    if (releasing) {
      // Recheck fresh disabled feedback immediately before releasing handles,
      // including when the main thread was descheduled after confirmation.
      if (hooks_.release(detail_)) {phase_=Phase::Complete;detail_="drivers released";}
      else fail("driver release failed: "+detail_);
    } else if (disabling) {
      phase_=Phase::Releasing;
      detail_="disabled + stationary for 500 ms; brake_feedback=UNAVAILABLE";
    } else if (hooks_.disable(detail_)) {
      phase_=Phase::Disabling;
      since_=std::max(now,Clock::now());
      // A successful command submission never counts as disabled feedback.
      window_=sdk::DriveFeedbackWindow(since_,std::chrono::milliseconds(500));
    } else fail("disable submission failed: "+detail_);
  } catch (const std::exception& error) { fail(std::string("shutdown exception: ")+error.what()); }
    catch (...) { fail("unknown shutdown exception"); }
  Phase phase() const { return phase_; }
  bool complete() const { return phase_==Phase::Complete; }
  std::string status() const {
    const char* name=phase_==Phase::Stopping ? "STOPPING" :
      phase_==Phase::Disabling ? "DISABLING" :
      phase_==Phase::Releasing ? "DISABLED_STATIONARY_CONFIRMED" :
      phase_==Phase::Complete ? "COMPLETE" : "FAILED_HOLD";
    return std::string("shutdown=")+name+" admission=closed brake_feedback=UNAVAILABLE; "+detail_;
  }
private:
  void fail(const std::string& detail) { phase_=Phase::FailedHold; detail_=detail; }
  Hooks hooks_;
  std::size_t expected_axes_;
  Phase phase_{Phase::Stopping};
  Clock::time_point since_;
  sdk::DriveFeedbackWindow window_;
  std::string detail_{"waiting for fresh measured standstill"};
};
}  // namespace motion_control_pkg::execution
