#include "motion_control_pkg/gripper/ds485_ros_bridge.hpp"
#include "motion_control_pkg/execution/action_terminal_guard.hpp"
#include "motion_control_pkg/execution/gripper_command_validation.hpp"
#include "motion_control_pkg/execution/shutdown_request.hpp"
#include <algorithm>
#include <cmath>

namespace motion_control_pkg::gripper {
using CommandMsg = ds485_interfaces::msg::GripperCommand;
using ResultMsg = ds485_interfaces::msg::GripperResult;
using StateMsg = ds485_interfaces::msg::ServoState;
namespace {
template <class Pub, class Msg> void publish(const Pub &pub, const Msg &msg) {
  try {
    if (rclcpp::ok())
      pub->publish(msg);
  } catch (const rclcpp::exceptions::RCLError &) {
  }
}
uint8_t scaled(int value, int maximum) {
  return maximum > 0 ? static_cast<uint8_t>(std::clamp(std::abs(value) * 255 / maximum, 0, 255))
                     : 0;
}
} // namespace
Ds485RosBridge::Ds485RosBridge(rclcpp::Node::SharedPtr node,
                               const ds485_driver::DriverConfig &config,
                               Ds485Executor::PortFactory factory)
    : node_(node) {
  const int poll = config.poll_ms;
  const double hz = config.state_rate_hz;
  default_timeout_ms_ = config.default_timeout_ms;
  auto qos = rclcpp::QoS(10).reliable().durability_volatile();
  result_pub_ = node->create_publisher<ResultMsg>("/ds485/gripper/result", qos);
  state_pub_ = node->create_publisher<StateMsg>("/ds485/state", qos);
  executor_ = std::make_unique<Ds485Executor>(
      config.sides, std::move(factory),
      [this](const auto &c, const auto &out) { publishResult(c, out); },
      [this](unsigned side, const auto &state) { publishState(side, state); }, poll, hz,
      [] { return execution::shutdownRequested(); },
      [logger = node->get_logger()](unsigned side, const std::string &message) {
        RCLCPP_WARN(logger, "DS485 %s: %s", side == 0 ? "left" : "right", message.c_str());
      });
  command_sub_ = node->create_subscription<CommandMsg>(
      "/ds485/gripper/command", qos, [this](CommandMsg::ConstSharedPtr msg) { onCommand(*msg); });
  stop_sub_ = node->create_subscription<ds485_interfaces::msg::GripperStop>(
      "/ds485/gripper/stop", qos, [this](ds485_interfaces::msg::GripperStop::ConstSharedPtr msg) {
        executor_->stop(msg->side);
      });
}
Ds485RosBridge::Ds485RosBridge(rclcpp::Node::SharedPtr node, const std::string& device_config,
                               const std::array<bool, 2>& gripper_tools) : node_(node) {
  torque_ = std::make_unique<ds485_driver::EmbeddedTorque>(
      node->get_node_base_interface()->get_context(), device_config, gripper_tools,
      [this](unsigned side, const std::function<void()>& dispatch, std::string& error) {
        return admit(side, dispatch, error);
      });
}
bool Ds485RosBridge::admit(unsigned side, const std::function<void()>& dispatch, std::string& error) const {
  auto guard = std::atomic_load(&admission_);
  if (guard && *guard) return (*guard)(side, dispatch, error);
  error = "motion admission is not ready";
  return false;
}
Ds485RosBridge::~Ds485RosBridge() {
  shutdown();
}
void Ds485RosBridge::start() {
  // The motion module has connected its SDK; driver start() only attaches to it.
  if (torque_) torque_->start();
  else executor_->start();
}
void Ds485RosBridge::shutdown() {
  if (torque_) torque_->shutdown();
  if (executor_) executor_->shutdown();
}
void Ds485RosBridge::publishResult(const Ds485Executor::Command &c,
                                   const Ds485Executor::Outcome &out) {
  ResultMsg msg;
  msg.id = c.id;
  msg.side = c.side;
  msg.result = out.result;
  msg.success = out.success();
  msg.message = out.message;
  msg.final_position = out.state.position;
  msg.final_angle_deg = out.state.servo.angle / 10.0F;
  publish(result_pub_, msg);
  if (!msg.success) {
    if (auto node = node_.lock())
      RCLCPP_WARN(node->get_logger(), "DS485 topic id=%u side=%u result=%u: %s", msg.id, msg.side,
                  msg.result, msg.message.c_str());
  }
}
void Ds485RosBridge::publishState(unsigned side, const Ds485Executor::State &state) {
  StateMsg msg;
  msg.side = side;
  msg.online = state.online;
  msg.busy = state.busy;
  msg.calibrated = state.calibrated;
  msg.error = state.error;
  if (state.online) {
    const auto &s = state.servo;
    msg.state_raw = s.state;
    msg.moving = s.moving;
    msg.fault = s.fault;
    msg.fault_text = s.fault ? ds485::FaultText(s.state) : "";
    msg.angle_deg = s.angle / 10.0F;
    msg.speed_dps = s.speed;
    msg.current_ma = s.current;
    msg.pwm = s.pwm;
    msg.position = state.position;
  }
  const std::string warning =
      !state.online
          ? "offline: " + state.error
          : (state.servo.fault ? "servo protection: " + ds485::FaultText(state.servo.state)
                               : state.error);
  if (warning != last_state_warning_[side]) {
    last_state_warning_[side] = warning;
    if (!warning.empty()) {
      if (auto node = node_.lock())
        RCLCPP_WARN(node->get_logger(), "DS485 side=%u: %s", side, warning.c_str());
    }
  }
  publish(state_pub_, msg);
}
void Ds485RosBridge::onCommand(const CommandMsg &msg) {
  std::vector<Ds485Executor::Command> commands;
  if (msg.side <= 2)
    for (unsigned i = 0; i < 2; ++i)
      if (msg.side == 2 || msg.side == i) {
        Ds485Executor::Command c;
        c.id = msg.id;
        c.side = i;
        c.position = msg.position;
        c.torque = msg.torque;
        c.speed = msg.speed;
        c.timeout_ms = msg.timeout_ms ? msg.timeout_ms : default_timeout_ms_;
        commands.push_back(c);
      }
  std::string error;
  std::vector<Ds485Executor::Handle> handles;
  bool accepted = false;
  if (msg.side > 2)
    error = "side must be 0, 1 or 2";
  else if (!admit(
               msg.side, [&] { accepted = executor_->submit(commands, handles, error); }, error) &&
           error.empty())
    error = "motion admission refused";
  if (accepted)
    return;
  if (commands.empty()) {
    Ds485Executor::Command c;
    c.id = msg.id;
    c.side = msg.side;
    commands.push_back(c);
  }
  for (const auto &c : commands) {
    Ds485Executor::Outcome out;
    out.done = true;
    out.result = Ds485Executor::Failed;
    out.message = error;
    if (c.side < 2)
      out.state = executor_->state(c.side);
    publishResult(c, out);
  }
}
bool Ds485RosBridge::validGoal(const Action::Goal &goal, std::string &error) const {
  if (torque_) {
    error = "POSITION_MODE_REQUIRED: control_gripper requires DS485 position strategy";
    return false;
  }
  const auto valid = execution::validateControlGripperGoal(goal);
  if (!valid) {
    error = valid.error_message;
    return false;
  }
  if (goal.timeout_ms > UINT32_MAX) {
    error = "DS485 timeout exceeds uint32 milliseconds";
    return false;
  }
  for (unsigned i = 0; i < 2; ++i)
    if (goal.gripper_id == 2 || goal.gripper_id == i) {
      const auto &c = executor_->config(i);
      if (!c.enabled || c.gripper.open_angle == c.gripper.close_angle) {
        error = "gripper disabled or uncalibrated";
        return false;
      }
      if (goal.com_port[i] != c.channel) {
        error = "com_port must match configured shared DS485 channel";
        return false;
      }
    }
  return true;
}
void Ds485RosBridge::runAction(std::shared_ptr<GoalHandle> handle) {
  execution::ActionTerminalGuard<GoalHandle, Action::Result> terminal(handle);
  auto result = std::make_shared<Action::Result>();
  result->final_position.assign(2, 0);
  result->final_torque.assign(2, 0);
  const auto finish = [&](int code) {
    try {
      if (code == 1)
        terminal.succeed(result);
      else if (code == 2)
        terminal.cancel(result);
      else
        terminal.abort(result);
    } catch (const std::exception &e) {
      if (auto node = node_.lock())
        RCLCPP_ERROR(node->get_logger(),
                     "DS485 Action terminal delivery failed after worker completion: %s", e.what());
    }
  };
  const auto goal = handle->get_goal();
  std::vector<Ds485Executor::Command> commands;
  std::vector<Ds485Executor::Handle> tickets;
  if (!validGoal(*goal, result->error_message)) {
    finish(0);
    return;
  }
  for (unsigned i = 0; i < 2; ++i)
    if (goal->gripper_id == 2 || goal->gripper_id == i) {
      Ds485Executor::Command c;
      c.side = i;
      c.action = true;
      c.position = goal->position[i];
      c.torque = goal->max_torque[i];
      c.speed = goal->speed[i];
      c.timeout_ms = goal->timeout_ms;
      commands.push_back(c);
    }
  if (!executor_->submit(commands, tickets, result->error_message)) {
    finish(0);
    return;
  }
  const auto initial =
      std::array<uint8_t, 2>{executor_->state(0).position, executor_->state(1).position};
  bool cancel_sent = false;
  try {
    while (true) {
      bool all_done = true, failed = false;
      for (const auto &t : tickets) {
        const auto out = executor_->outcome(t);
        all_done &= out.done;
        failed |= out.done && !out.success();
      }
      if (!cancel_sent && (handle->is_canceling() || execution::shutdownRequested() || failed)) {
        executor_->cancel(tickets);
        cancel_sent = true;
      }
      if (all_done)
        break;
      auto fb = std::make_shared<Action::Feedback>();
      fb->current_position.assign(2, 0);
      fb->current_torque.assign(2, 0);
      fb->current_speed.assign(2, 0);
      float progress = 0;
      for (const auto &c : commands) {
        const auto state = executor_->state(c.side);
        fb->current_position[c.side] = state.position;
        fb->current_torque[c.side] = scaled(state.servo.pwm, 1000);
        fb->current_speed[c.side] = scaled(state.servo.speed, state.max_speed);
        const int distance = std::abs(int(c.position) - initial[c.side]);
        progress +=
            distance ? std::clamp(100.0F * (distance - std::abs(int(c.position) - state.position)) /
                                      distance,
                                  0.0F, 99.0F)
                     : 99.0F;
      }
      fb->progress_rate = progress / commands.size();
      if (auto node = node_.lock())
        fb->header.stamp = node->now();
      handle->publish_feedback(fb);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  } catch (const std::exception &e) {
    // Keep ownership until the worker has handled cancellation, even if ROS publication fails.
    executor_->cancel(tickets);
    for (const auto &t : tickets)
      while (!executor_->outcome(t).done)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    result->error_message = std::string("Action transport error: ") + e.what();
    finish(0);
    return;
  }
  result->success = true;
  for (const auto &t : tickets) {
    const auto out = executor_->outcome(t);
    const auto side = t->command.side;
    result->final_position[side] = out.state.position;
    result->final_torque[side] = scaled(out.state.servo.pwm, 1000);
    if (!out.success()) {
      result->success = false;
      result->error_message += (result->error_message.empty() ? "" : "; ") +
                               std::string(side == 0 ? "left: " : "right: ") + out.message;
    }
  }
  if (!result->success) {
    if (auto node = node_.lock())
      RCLCPP_WARN(node->get_logger(), "DS485 Action task=%s: %s", goal->task_id.c_str(),
                  result->error_message.c_str());
  }
  if (handle->is_canceling()) {
    result->success = false;
    finish(2);
  } else if (result->success)
    finish(1);
  else
    finish(0);
}
} // namespace motion_control_pkg::gripper
