#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace motion_control_pkg::sdk {
using DriveClock = std::chrono::steady_clock;
struct DriveControlResult {
  bool command_issued{false};
  bool feedback_confirmed{false};
  std::string message;
  bool success() const { return command_issued && feedback_confirmed; }
};
struct DriveAxis {
  double position{}, velocity{};  // radians, rad/s
  std::uint64_t sequence{};
  DriveClock::time_point acquired{};
  bool disabled{}, fault{}, reset_allowed{};
  bool wraps_at_million{false};
};

// Confirmation requires multiple DEVICE frames, not multiple application reads.
// Also bound displacement: a stuck velocity field must not prove standstill.
class DriveFeedbackWindow {
public:
  explicit DriveFeedbackWindow(DriveClock::time_point after,
      std::chrono::milliseconds dwell=std::chrono::milliseconds(100)) : after_(after), dwell_(dwell) {}
  bool update(const std::vector<DriveAxis>& axes, bool clearing, bool preflight,
              DriveClock::time_point now) {
    if (axes.empty()) { reset(); return false; }
    for (const auto& a : axes) {
      if (!a.sequence || a.acquired < after_ || a.acquired > now ||
          now-a.acquired > std::chrono::milliseconds(100) ||
          !std::isfinite(a.position) || !std::isfinite(a.velocity) ||
          std::abs(a.velocity) > .01 ||
          !(preflight ? a.reset_allowed : a.disabled) ||
          (clearing && !preflight && a.fault)) {
        reset(); return false;
      }
    }
    if (last_.size() != axes.size()) {
      last_=origin_=axes; count_=1; return false;
    }
    bool advancing=true;
    for (std::size_t i=0; i<axes.size(); ++i) {
      const auto& a=axes[i]; const auto& b=last_[i];
      const bool wrapped=a.wraps_at_million && b.sequence>900000 && a.sequence<100000;
      if ((!wrapped && a.sequence<b.sequence) || a.acquired<b.acquired ||
          std::abs(a.position-origin_[i].position)>.001) {
        reset(); return false;
      }
      advancing=advancing && a.sequence!=b.sequence && a.acquired>b.acquired;
    }
    if (!advancing) return false;
    last_=axes; ++count_;
    if (count_<5) return false;
    for (std::size_t i=0; i<axes.size(); ++i)
      if (axes[i].acquired-origin_[i].acquired<dwell_) return false;
    return true;
  }
private:
  void reset() { last_.clear(); origin_.clear(); count_=0; }
  DriveClock::time_point after_;
  std::chrono::milliseconds dwell_;
  std::vector<DriveAxis> last_, origin_;
  unsigned count_{0};
};

inline bool waitDriveFeedback(
    const std::function<bool(std::vector<DriveAxis>&,std::string&)>& read,
    const std::function<bool()>& canceled, bool clearing, bool preflight,
    DriveClock::time_point after, std::string& detail,
    std::chrono::milliseconds timeout=std::chrono::milliseconds(2000)) {
  DriveFeedbackWindow window(after);
  const auto end=DriveClock::now()+timeout;
  while (DriveClock::now()<end) {
    if (canceled()) { detail="superseded by a newer emergency stop"; return false; }
    std::vector<DriveAxis> axes;
    if (!read(axes,detail)) axes.clear();
    if (window.update(axes,clearing,preflight,DriveClock::now())) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  detail="fresh disabled/standstill feedback not confirmed: "+detail;
  return false;
}
}  // namespace motion_control_pkg::sdk
