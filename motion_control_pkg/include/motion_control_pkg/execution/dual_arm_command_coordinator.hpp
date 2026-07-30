#ifndef MOTION_CONTROL_PKG_EXECUTION_DUAL_ARM_COMMAND_COORDINATOR_HPP_
#define MOTION_CONTROL_PKG_EXECUTION_DUAL_ARM_COMMAND_COORDINATOR_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "motion_control_pkg/execution/control_period.hpp"
#include "motion_control_pkg/sdk/sdk_command_gate.hpp"

namespace motion_control_pkg::execution
{

using JointCommand =
  std::array<double, motion_control_pkg::sdk::kArmDof>;

enum class TrajectoryExecutionState
{
  kQueued,
  kRunning,
  kSucceeded,
  kCanceled,
  kFailed,
};

inline bool isTerminal(TrajectoryExecutionState state)
{
  return state == TrajectoryExecutionState::kSucceeded ||
         state == TrajectoryExecutionState::kCanceled ||
         state == TrajectoryExecutionState::kFailed;
}

struct TrajectoryExecutionSnapshot
{
  TrajectoryExecutionState state{TrajectoryExecutionState::kFailed};
  std::size_t samples_sent{0};
  std::size_t sample_count{0};
  std::string error_message;
};

namespace detail
{

struct SharedTrajectoryState
{
  mutable std::mutex mutex;
  std::condition_variable condition;
  std::atomic<bool> cancel_requested{false};
  std::shared_ptr<std::atomic<bool>> coupled_cancel_requested;
  TrajectoryExecutionState state{TrajectoryExecutionState::kQueued};
  std::size_t samples_sent{0};
  std::size_t sample_count{0};
  std::string error_message;
};

}  // namespace detail

/**
 * @brief 单侧轨迹的协作式控制句柄。
 *
 * requestCancel() 只阻止协调器继续发送后续采样点，不等价于厂商软急停
 * 或物理急停。
 */
class TrajectoryTicket
{
public:
  TrajectoryTicket() = default;

  bool valid() const noexcept {return static_cast<bool>(state_);}
  bool requestCancel();
  TrajectoryExecutionSnapshot snapshot() const;
  bool waitFor(std::chrono::steady_clock::duration timeout) const;

private:
  friend class DualArmCommandCoordinator;
  explicit TrajectoryTicket(
    std::shared_ptr<detail::SharedTrajectoryState> state)
  : state_(std::move(state))
  {
  }

  std::shared_ptr<detail::SharedTrajectoryState> state_;
};

struct TrajectoryEnqueueResult
{
  bool success{false};
  std::string error_message;
  TrajectoryTicket ticket;
};

struct SynchronizedTrajectoryEnqueueResult
{
  bool success{false};
  std::string error_message;
  TrajectoryTicket left_ticket;
  TrajectoryTicket right_ticket;
};

/**
 * @brief 以单一 4 ms 线程协调左右臂 MarvinSDK 全局事务。
 *
 * 每个周期最多执行一次 OnClearSet -> A/B 有效点 -> OnSetSend。左右轨迹
 * 各自推进索引，因此取消一侧不会清空或终止另一侧轨迹。
 */
class DualArmCommandCoordinator
{
public:
  explicit DualArmCommandCoordinator(
    motion_control_pkg::sdk::SdkCommandGate & command_gate,
    std::chrono::steady_clock::duration period = kDualArmControlPeriod);
  ~DualArmCommandCoordinator();

  bool start();
  void stop();

  TrajectoryEnqueueResult enqueueTrajectory(
    motion_control_pkg::sdk::ArmChannel channel,
    std::vector<JointCommand> points);

  /**
   * @brief 在同一临界区内预留左右槽位，保证双臂无单侧提前入队窗口。
   */
  SynchronizedTrajectoryEnqueueResult enqueueSynchronizedPair(
    std::vector<JointCommand> left_points,
    std::vector<JointCommand> right_points);

  std::chrono::steady_clock::duration period() const noexcept
  {
    return period_;
  }

private:
  struct ArmSlot
  {
    std::vector<JointCommand> points;
    std::size_t next_index{0};
    std::shared_ptr<detail::SharedTrajectoryState> state;
    bool active{false};
  };

  void run();
  void processOneCycle();
  void finishSlot(
    ArmSlot & slot, TrajectoryExecutionState state,
    const std::string & error_message = {});
  void cancelActiveTrajectories(const std::string & reason);

  motion_control_pkg::sdk::SdkCommandGate & command_gate_;
  const std::chrono::steady_clock::duration period_;
  std::array<ArmSlot, motion_control_pkg::sdk::kArmCount> slots_;
  std::mutex slots_mutex_;
  std::mutex wait_mutex_;
  std::condition_variable wait_condition_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stopped_{false};
  std::thread worker_;
};

}  // namespace motion_control_pkg::execution

#endif  // MOTION_CONTROL_PKG_EXECUTION_DUAL_ARM_COMMAND_COORDINATOR_HPP_
