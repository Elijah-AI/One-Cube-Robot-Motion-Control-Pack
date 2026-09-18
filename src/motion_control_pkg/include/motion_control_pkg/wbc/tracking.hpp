#pragma once
#include "motion_control_pkg/wbc/model.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>

namespace motion_control_pkg::wbc {
// Policy, not robot attributes. Input timestamps share one acquisition clock.
struct TrackingPolicy {
  double position_error{.015}, hard_error{.08}, dwell_sec{.08}, max_age_sec{.15};
};

// Compares measured q to the last command SENT before acquisition, not to a
// future trajectory sample or the latest command at callback time. Commands
// are position holds. Transport delay and finite servo bandwidth remain error.
// Single-owner / externally synchronized. Failure is latched until reset().
class PositionTrackingGuard {
public:
  explicit PositionTrackingGuard(std::vector<JointSpec> joints, TrackingPolicy p = {})
      : joints_(std::move(joints)), policy_(p) {
    if (joints_.empty() || !std::isfinite(p.position_error) || !std::isfinite(p.hard_error) ||
        !std::isfinite(p.dwell_sec) || !std::isfinite(p.max_age_sec) || p.position_error<=0 ||
        p.hard_error<p.position_error || p.dwell_sec<=0 || p.max_age_sec<=0)
      throw std::invalid_argument("invalid tracking policy");
    reset();
  }
  void reset() { history_.clear(); since_.assign(joints_.size(),-1); last_stamp_=0; peak_=0; error_.clear(); }
  bool command(double stamp, const std::vector<double>& q) {
    if (!error_.empty()) return false;
    if (!std::isfinite(stamp) || stamp<=0 || q.size()!=joints_.size() ||
        (!history_.empty() && stamp<=history_.back().stamp)) return fail("invalid command clock or dimensions");
    for (std::size_t j=0;j<q.size();++j)
      if (!std::isfinite(q[j]) || q[j]<joints_[j].lower-1e-8 || q[j]>joints_[j].upper+1e-8)
        return fail("command outside URDF: "+joints_[j].name);
    history_.push_back({stamp,q});
    // Retain the predecessor of the entire allowed feedback-age interval.
    while (history_.size()>2 && history_[1].stamp<stamp-policy_.max_age_sec-.01) history_.pop_front();
    return true;
  }
  bool observe(double stamp, double now, const std::vector<double>& q, const std::vector<double>& v) {
    if (!error_.empty()) return false;
    if (history_.empty()) return true;
    if (!std::isfinite(stamp) || !std::isfinite(now) || stamp<=0 || stamp>now || now-stamp>policy_.max_age_sec)
      return fail("tracking feedback clock or age invalid");
    // Pre-dispatch samples are not compared to commands that did not exist.
    if (stamp<history_.front().stamp) return true;
    if (stamp<last_stamp_) return fail("tracking feedback time reversed");
    if (stamp==last_stamp_) return true;  // replay cannot advance dwell
    if (q.size()!=joints_.size() || v.size()!=q.size()) return fail("tracking feedback dimensions invalid");
    auto ref=history_.begin();
    for (auto it=history_.begin(); it!=history_.end() && it->stamp<=stamp; ++it) ref=it;
    for (std::size_t j=0;j<q.size();++j) {
      const auto& spec=joints_[j];
      if (!std::isfinite(q[j]) || !std::isfinite(v[j]) || q[j]<spec.lower-1e-6 || q[j]>spec.upper+1e-6 ||
          std::abs(v[j])>spec.velocity+1e-6) return fail("tracking feedback violates URDF: "+spec.name);
      const double e=std::abs(q[j]-ref->q[j]); peak_=std::max(peak_,e);
      if (e>policy_.hard_error) return fail("hard position tracking error: "+spec.name);
      if (e<=policy_.position_error) since_[j]=-1;
      else {
        if (since_[j]<0) since_[j]=stamp;
        if (stamp-since_[j]>=policy_.dwell_sec) return fail("sustained position tracking error: "+spec.name);
      }
    }
    last_stamp_=stamp; return true;
  }
  const std::string& error() const {return error_;}
  double peak() const {return peak_;}
private:
  bool fail(const std::string& error) {if (error_.empty()) error_=error; return false;}
  struct Command {double stamp; std::vector<double> q;};
  std::vector<JointSpec> joints_; TrackingPolicy policy_; std::deque<Command> history_;
  std::vector<double> since_; double last_stamp_{},peak_{}; std::string error_;
};
}  // namespace motion_control_pkg::wbc
