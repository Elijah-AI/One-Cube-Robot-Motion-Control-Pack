#ifndef MOTION_CONTROL_PKG_EXECUTION_ACTION_TERMINAL_GUARD_HPP_
#define MOTION_CONTROL_PKG_EXECUTION_ACTION_TERMINAL_GUARD_HPP_

#include <memory>
#include <utility>

#include "motion_control_pkg/execution/execution_control.hpp"

namespace motion_control_pkg::execution
{

/**
 * @brief 对 ROS 2 Action GoalHandle 的单终态保护。
 *
 * 模板不绑定具体 Action 类型，可在后续迁移 MovePose、MoveJoints 与末端工具
 * Action 时复用，同时不改变 RobotControlNode 的既有职责边界。
 */
template<typename GoalHandleT, typename ResultT>
class ActionTerminalGuard
{
public:
  explicit ActionTerminalGuard(std::shared_ptr<GoalHandleT> goal_handle)
  : goal_handle_(std::move(goal_handle))
  {
  }

  bool succeed(const std::shared_ptr<ResultT> & result)
  {
    if (!terminal_.tryFinish(TerminalState::kSucceeded)) {
      return false;
    }
    goal_handle_->succeed(result);
    return true;
  }

  bool abort(const std::shared_ptr<ResultT> & result)
  {
    if (!terminal_.tryFinish(TerminalState::kAborted)) {
      return false;
    }
    goal_handle_->abort(result);
    return true;
  }

  bool cancel(const std::shared_ptr<ResultT> & result)
  {
    if (!terminal_.tryFinish(TerminalState::kCanceled)) {
      return false;
    }
    goal_handle_->canceled(result);
    return true;
  }

  TerminalState state() const noexcept {return terminal_.state();}

private:
  std::shared_ptr<GoalHandleT> goal_handle_;
  TerminalStateMachine terminal_;
};

}  // namespace motion_control_pkg::execution

#endif  // MOTION_CONTROL_PKG_EXECUTION_ACTION_TERMINAL_GUARD_HPP_
