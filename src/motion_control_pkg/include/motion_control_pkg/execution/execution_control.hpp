#ifndef MOTION_CONTROL_PKG_EXECUTION_EXECUTION_CONTROL_HPP_
#define MOTION_CONTROL_PKG_EXECUTION_EXECUTION_CONTROL_HPP_

#include <atomic>
#include <chrono>

namespace motion_control_pkg::execution
{

enum class TerminalState
{
  kActive,
  kSucceeded,
  kAborted,
  kCanceled,
};

/**
 * @brief 保证一个执行任务最多进入一次终态。
 */
class TerminalStateMachine
{
public:
  bool tryFinish(TerminalState terminal_state)
  {
    if (terminal_state == TerminalState::kActive) {
      return false;
    }
    auto expected = TerminalState::kActive;
    return state_.compare_exchange_strong(expected, terminal_state);
  }

  TerminalState state() const noexcept {return state_.load();}

private:
  std::atomic<TerminalState> state_{TerminalState::kActive};
};

/**
 * @brief 使用单调时钟实现超时，系统时间校准不会改变期限。
 */
class SteadyDeadline
{
public:
  explicit SteadyDeadline(std::chrono::steady_clock::duration timeout)
  : deadline_(std::chrono::steady_clock::now() + timeout)
  {
  }

  double remainingSeconds() const noexcept
  {
    return std::chrono::duration<double>(deadline_ - std::chrono::steady_clock::now()).count();
  }

  std::chrono::steady_clock::time_point timePoint() const noexcept { return deadline_; }

  bool expired() const noexcept
  {
    return std::chrono::steady_clock::now() >= deadline_;
  }

private:
  std::chrono::steady_clock::time_point deadline_;
};

/**
 * @brief 协作式取消标记。
 *
 * 取消只阻止后续轨迹采样点继续下发，不等价于厂商软急停或物理急停。
 */
class CancellationToken
{
public:
  void request() noexcept {requested_.store(true);}
  bool requested() const noexcept {return requested_.load();}

private:
  std::atomic<bool> requested_{false};
};

/**
 * @brief 在线程所有退出路径上恢复模块忙闲标志。
 *
 * 最多管理同一任务占用的两个资源；单臂任务只传入第一个指针。
 */
class ScopedBusyFlagReset
{
public:
  explicit ScopedBusyFlagReset(
    std::atomic<bool> * first, std::atomic<bool> * second = nullptr)
  : first_(first), second_(second)
  {
  }

  ~ScopedBusyFlagReset()
  {
    if (first_ != nullptr) {
      first_->store(false);
    }
    if (second_ != nullptr) {
      second_->store(false);
    }
  }

  ScopedBusyFlagReset(const ScopedBusyFlagReset &) = delete;
  ScopedBusyFlagReset & operator=(const ScopedBusyFlagReset &) = delete;

private:
  std::atomic<bool> * first_;
  std::atomic<bool> * second_;
};

}  // namespace motion_control_pkg::execution

#endif  // MOTION_CONTROL_PKG_EXECUTION_EXECUTION_CONTROL_HPP_
