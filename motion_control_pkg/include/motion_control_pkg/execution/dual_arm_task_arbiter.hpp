#ifndef MOTION_CONTROL_PKG_EXECUTION_DUAL_ARM_TASK_ARBITER_HPP_
#define MOTION_CONTROL_PKG_EXECUTION_DUAL_ARM_TASK_ARBITER_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "motion_control_pkg/sdk/dual_arm_device_interface.hpp"
#include "motion_control_pkg/execution/execution_control.hpp"

namespace motion_control_pkg::execution
{

enum class ArmTaskScope
{
  kLeft,
  kRight,
  kDual,
};

struct ArmTaskArbitrationState
{
  explicit ArmTaskArbitrationState(
    motion_control_pkg::sdk::ActiveArmConfiguration active_configuration)
  : configuration(active_configuration)
  {
  }

  std::mutex mutex;
  std::array<bool, motion_control_pkg::sdk::kArmCount> occupied{false, false};
  bool dual_task_active{false};
  motion_control_pkg::sdk::ActiveArmConfiguration configuration;
};

/**
 * @brief 一次单臂或双臂任务的独占资源令牌。
 *
 * 令牌不可复制，只能移动。最后一个有效令牌离开作用域时自动释放其占用，
 * 因此成功、取消、失败和异常路径共享同一套资源恢复语义。
 */
class ArmTaskLease
{
public:
  ArmTaskLease() = default;
  ~ArmTaskLease();

  ArmTaskLease(ArmTaskLease && other) noexcept;
  ArmTaskLease & operator=(ArmTaskLease && other) noexcept;

  ArmTaskLease(const ArmTaskLease &) = delete;
  ArmTaskLease & operator=(const ArmTaskLease &) = delete;

  bool valid() const noexcept {return static_cast<bool>(state_);}
  ArmTaskScope scope() const noexcept {return scope_;}
  void reset() noexcept;

private:
  friend class DualArmTaskArbiter;
  ArmTaskLease(
    std::shared_ptr<ArmTaskArbitrationState> state,
    ArmTaskScope scope) noexcept;

  std::shared_ptr<ArmTaskArbitrationState> state_;
  ArmTaskScope scope_{ArmTaskScope::kLeft};
};

struct ArmTaskAvailability
{
  bool available{false};
  std::string error_message;

  explicit operator bool() const noexcept {return available;}
};

struct ArmTaskAcquireResult
{
  bool success{false};
  std::string error_message;
  ArmTaskLease lease;

  explicit operator bool() const noexcept {return success;}
};

struct ArmTaskArbitrationSnapshot
{
  bool left_occupied{false};
  bool right_occupied{false};
  bool dual_task_active{false};
};

/**
 * @brief 双臂执行共享的停止请求和唯一终态。
 *
 * 后续同步轨迹执行器由左右任一侧报告故障时，两个通道观察到同一个
 * stopRequested()；tryFail/tryCancel/trySucceed 中只有第一个调用能够提交
 * Action 终态。
 */
class DualArmTaskTermination
{
public:
  bool tryFail(
    motion_control_pkg::sdk::ArmChannel failed_channel,
    const std::string & reason);
  bool tryCancel(const std::string & reason);
  bool trySucceed();

  bool stopRequested() const noexcept {return stop_requested_.load();}
  TerminalState state() const noexcept {return terminal_.state();}
  std::string errorMessage() const;

private:
  bool tryStop(TerminalState state, const std::string & reason);

  TerminalStateMachine terminal_;
  std::atomic<bool> stop_requested_{false};
  mutable std::mutex error_mutex_;
  std::string error_message_;
};

/**
 * @brief 对单臂和双臂 Action 进行统一、原子的资源仲裁。
 *
 * canAcquire() 只用于 Goal 回调的快速预检查；Action 被接受后必须再次调用
 * tryAcquire()。后者在同一临界区完成检查和占用，负责消除并发 Goal 竞争。
 */
class DualArmTaskArbiter
{
public:
  explicit DualArmTaskArbiter(
    motion_control_pkg::sdk::ActiveArmConfiguration configuration);

  ArmTaskAvailability canAcquire(ArmTaskScope scope) const;
  ArmTaskAcquireResult tryAcquire(ArmTaskScope scope);
  ArmTaskArbitrationSnapshot snapshot() const;

private:
  friend class ArmTaskLease;

  static std::array<bool, motion_control_pkg::sdk::kArmCount> maskFor(
    ArmTaskScope scope) noexcept;
  static ArmTaskAvailability checkAvailabilityLocked(
    const ArmTaskArbitrationState & state, ArmTaskScope scope);

  std::shared_ptr<ArmTaskArbitrationState> state_;
};

}  // namespace motion_control_pkg::execution

#endif  // MOTION_CONTROL_PKG_EXECUTION_DUAL_ARM_TASK_ARBITER_HPP_
