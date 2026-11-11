#pragma once
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <jsoncpp/json/json.h>

namespace motion_control_pkg::motion {
using IkStop = std::function<bool()>;
using IkClock = std::chrono::steady_clock;
struct IkRecoveryOptions {
  bool enabled{true};
  std::size_t seeds{24}, workers{8};
  double seed_timeout_s{0.5}, request_timeout_s{2.0};
  std::vector<int> cpus; // Empty: detect E cores from hybrid SMT topology.
  std::string worker_executable;
};
struct IkRecoveryStats {
  std::uint64_t requests{0}, successes{0}, cancelled{0}, failures{0};
  unsigned attempted{0}, timed_out{0};
  double elapsed_s{0};
  std::vector<int> cpus;
  std::string status{"disabled"};
};

// Per-caller cancellation/deadline scope; never installed globally for another
// action. Worker threads must receive an explicit stop callback as before.
inline thread_local const IkStop* ik_planning_stop = nullptr;
class IkPlanningStopScope {
 public:
  explicit IkPlanningStopScope(IkStop stop):stop_(std::move(stop)),previous_(ik_planning_stop) { ik_planning_stop=&stop_; }
  ~IkPlanningStopScope() { ik_planning_stop=previous_; }
  IkPlanningStopScope(const IkPlanningStopScope&)=delete;
 private:
  IkStop stop_; const IkStop* previous_;
};
inline bool ikPlanningStopped() { return ik_planning_stop && *ik_planning_stop && (*ik_planning_stop)(); }
inline thread_local bool ik_recovery_suppressed = false;
class IkRecoverySuppressionScope {
 public:
  IkRecoverySuppressionScope():previous_(ik_recovery_suppressed) { ik_recovery_suppressed=true; }
  ~IkRecoverySuppressionScope() { ik_recovery_suppressed=previous_; }
 private: bool previous_;
};
inline thread_local bool ik_single_seed_kernel = false;
class IkSingleSeedScope {
 public:
  IkSingleSeedScope():previous_(ik_single_seed_kernel) { ik_single_seed_kernel=true; }
  ~IkSingleSeedScope() { ik_single_seed_kernel=previous_; }
 private: bool previous_;
};

class IkRecoveryClient {
 public:
  IkRecoveryClient(IkRecoveryOptions options, std::string urdf, std::string meshes,
                   std::string model_signature);
  ~IkRecoveryClient();
  IkRecoveryClient(const IkRecoveryClient&)=delete;
  bool solve(Json::Value request, const std::function<bool(const Json::Value&)>& accept,
             const IkStop& stop);
  IkRecoveryStats stats() const;
  const IkRecoveryOptions& options() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
std::vector<int> selectIkRecoveryCpus(std::size_t workers, const std::vector<int>& requested,
                                    const std::string& sys_root="/sys/devices/system/cpu");
} // namespace motion_control_pkg::motion

class MiracleMAlgo;
// Only the hardware-free broker uses this adapter. It cannot submit motion.
struct IkRecoveryAccess {
  static std::string signature(const MiracleMAlgo& algo);
  static Json::Value scene(const MiracleMAlgo& algo);
  static void applyScene(MiracleMAlgo& algo,const Json::Value& scene);
  static Json::Value evaluate(MiracleMAlgo& algo,const Json::Value& request,
                             const Json::Value& seed,const motion_control_pkg::motion::IkStop& stop);
};
