#pragma once
#include <cstdint>
#include <mutex>

namespace task_mgr {
// One handoff to the ROS executor, never a queue of future robot runs.
class ModbusTriggerGate {
public:
  void set_idle(bool idle) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = 0;
    accepting_ = idle;
  }
  bool submit(uint32_t address) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!accepting_ || address == 0) return false;
    accepting_ = false;
    pending_ = address;
    return true;
  }
  bool take(uint32_t& address) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_ == 0) return false;
    address = pending_;
    pending_ = 0;
    return true;
  }
private:
  std::mutex mutex_;
  bool accepting_{false};
  uint32_t pending_{0};
};
}  // namespace task_mgr
