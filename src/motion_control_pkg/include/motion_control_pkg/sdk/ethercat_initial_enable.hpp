#pragma once

#include "motion_control_pkg/sdk/ethercat_feedback_validity.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace motion_control_pkg::sdk {
using EnableClock = std::chrono::steady_clock;
struct EthercatEnableSample {
  std::vector<DriverSDK::motorActualStruct> motors;
  EnableClock::time_point acquired{};
  std::uint64_t sequence{};
};
struct EnableTiming {
  auto now() const { return EnableClock::now(); }
  void wait() const { std::this_thread::sleep_for(std::chrono::milliseconds(4)); }
};

// One attempt per motor and bus lifetime. Only the explicit motion/startup
// caller may use this handshake; the bus worker never initiates an enable.
// Hold the initial measured position, never chase changing feedback or clear
// faults. The caller fences every hold write with the ordinary safety permit.
class EthercatInitialEnable {
public:
  template<class Read, class Hold, class Disable, class Check, class Timing = EnableTiming>
  bool run(const std::vector<std::uint16_t>& ids, double timeout,
      Read read, Hold hold, Disable disable, Check check, std::string& error, Timing timing = {}) {
    if (ids.empty() || !std::isfinite(timeout) || timeout <= 0 ||
        std::set<std::uint16_t>(ids.begin(),ids.end()).size()!=ids.size()) {
      error="invalid initial enable request";return false;
    }
    for (auto id:ids) if (attempted_.count(id)) {
      error="automatic motor re-enable refused: alias="+std::to_string(id+1);return false;
    }
    EthercatEnableSample sample;
    const auto valid=[&]() {
      const auto now=timing.now();
      if (!sample.sequence || sample.acquired==EnableClock::time_point{} ||
          sample.acquired>now || now-sample.acquired>=std::chrono::milliseconds(100)) {
        error="initial enable requires fresh PDO feedback";return false;
      }
      for (auto id:ids) if (id>=sample.motors.size() ||
          !ethercatFeedbackValid(sample.motors[id]) || !std::isfinite(sample.motors[id].pos) ||
          !std::isfinite(sample.motors[id].vel)) {
        error="invalid initial enable feedback: alias="+std::to_string(id+1);return false;
      }
      return true;
    };
    if (!check(error) || !read(sample,error) || !valid()) return false;
    std::vector<DriverSDK::motorTargetStruct> targets(ids.size());
    for (std::size_t i=0;i<ids.size();++i) {
      if (std::abs(sample.motors[ids[i]].vel)>.01) {
        error="initial enable requires stationary motors";return false;
      }
      targets[i].pos=sample.motors[ids[i]].pos;targets[i].enabled=1;
    }
    attempted_.insert(ids.begin(),ids.end());
    bool success=false;
    struct Cleanup { Disable& disable; bool& success; ~Cleanup(){if(!success)disable();} } cleanup{disable,success};
    const auto deadline=timing.now()+std::chrono::duration_cast<EnableClock::duration>(std::chrono::duration<double>(timeout));
    auto previous=sample.acquired, stable=EnableClock::time_point{};
    auto sequence=sample.sequence;
    std::vector<bool> observed_enabled(ids.size(),false);
    unsigned stable_frames=0;
    while (timing.now()<deadline) {
      if (!check(error) || !hold(targets,error)) return false;
      timing.wait();
      if (!check(error) || !read(sample,error) || !valid()) return false;
      if (sample.sequence<sequence || sample.acquired<previous ||
          ((sample.sequence==sequence)!=(sample.acquired==previous))) {
        error="initial enable PDO sequence or timestamp regressed";return false;
      }
      if (sample.sequence==sequence) continue;
      sequence=sample.sequence;previous=sample.acquired;
      bool all_enabled=true;
      for (std::size_t i=0;i<ids.size();++i) {
        const auto& motor=sample.motors[ids[i]];
        if (std::abs(motor.pos-targets[i].pos)>.01 || std::abs(motor.vel)>.01) {
          error="motor moved during initial enable: alias="+std::to_string(ids[i]+1)+
            " measured="+std::to_string(motor.pos)+" hold="+std::to_string(targets[i].pos)+
            " delta="+std::to_string(motor.pos-targets[i].pos)+
            " velocity="+std::to_string(motor.vel)+" (limits: abs(delta)<=0.01 rad, abs(velocity)<=0.01 rad/s)";
          return false;
        }
        const bool enabled=ethercatFeedbackOperational(motor);
        if (observed_enabled[i] && !enabled) {
          error="motor lost enable during initial handshake: alias="+std::to_string(ids[i]+1);return false;
        }
        observed_enabled[i]=enabled;all_enabled=all_enabled && enabled;
      }
      if (!all_enabled) {stable={};stable_frames=0;continue;}
      if (!stable_frames++) stable=sample.acquired;
      if (stable_frames>=5 && sample.acquired-stable>=std::chrono::milliseconds(600)) {
        success=true;error.clear();return true;
      }
    }
    error="initial motor enable timed out";return false;
  }
private:
  std::set<std::uint16_t> attempted_;
};
}  // namespace motion_control_pkg::sdk
