#include "ds485_driver/gripper_controller.h"
#include "ds485_driver/marvin_rs485_port.h"
#include <cmath>
#include <stdexcept>

namespace ds485_driver {
using Clock = std::chrono::steady_clock;

GripperController::GripperController(const std::array<Config, 2> &configs, PortFactory factory,
                                     ResultCallback result, StateCallback state, int poll_ms,
                                     double hz, std::function<bool()> shutdown_requested,
                                     FaultLogger fault_logger)
    : configs_(configs), result_callback_(std::move(result)), state_callback_(std::move(state)),
      poll_ms_(poll_ms), state_hz_(hz), shutdown_requested_(std::move(shutdown_requested)) {
  if (poll_ms < 1 || poll_ms > 1000 || !std::isfinite(hz) || hz < 0 || hz > 100 ||
      (hz > 0 && hz < 0.1))
    throw std::invalid_argument("DS485 poll_ms must be 1..1000 and state_hz 0..100");
  use_shared_sdk_ = !factory;
  if (!factory)
    factory = [](unsigned side, uint8_t channel) {
      return std::make_unique<MarvinRs485Port>(side == 0 ? 'A' : 'B', channel);
    };
  for (unsigned i = 0; i < 2; ++i) {
    const auto &c = configs_[i];
    auto &s = sides_[i];
    if (!c.enabled)
      continue;
    if (c.id > 250 || c.channel < 1 || c.channel > 3)
      throw std::invalid_argument("invalid DS485 servo ID or channel");
    s.port = factory(i, c.channel);
    if (!s.port)
      throw std::invalid_argument("null DS485 port");
    s.servo = std::make_unique<Ds485Servo>(s.port.get(), c.id);
    s.servo->SetTiming(c.gap_ms, c.reply_timeout_ms);
    if (!s.servo->SetProtectionConfig(c.protection))
      throw std::invalid_argument(std::string(i == 0 ? "left: " : "right: ") + s.servo->LastError());
    if (fault_logger)
      s.servo->SetFaultLogger([fault_logger, i](const std::string &message) {
        fault_logger(i, message);
      });
    s.gripper = std::make_unique<Ds485Gripper>(s.servo.get());
    s.gripper->SetConfig(c.gripper);
    s.state.calibrated = c.gripper.open_angle != c.gripper.close_angle;
  }
}
GripperController::~GripperController() {
  shutdown();
}
void GripperController::start() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (started_)
      throw std::logic_error("DS485 executor already started");
    started_ = running_ = true;
  }
  try {
    if (use_shared_sdk_) {
      MarvinLink::Attach();
      attached_ = true;
    }
    for (unsigned i = 0; i < 2; ++i)
      if (configs_[i].enabled)
        sides_[i].worker = std::thread(&GripperController::worker, this, i);
  } catch (...) {
    shutdown();
    throw;
  }
}
bool GripperController::submit(const std::vector<Command> &commands, std::vector<Handle> &handles,
                               std::string &error) {
  std::lock_guard<std::mutex> lock(mutex_);
  handles.clear();
  if (!running_ || (shutdown_requested_ && shutdown_requested_())) {
    error = "DS485 executor is shutting down";
    return false;
  }
  if (commands.empty() || commands.size() > 2) {
    error = "invalid command batch";
    return false;
  }
  unsigned mask = 0;
  for (const auto &c : commands) {
    if (c.side > 1 || (mask & (1U << c.side)) || c.timeout_ms <= 0 || c.timeout_ms > UINT32_MAX) {
      error = "invalid DS485 side, duplicate side or timeout";
      return false;
    }
    mask |= 1U << c.side;
    const auto &s = sides_[c.side];
    if (!configs_[c.side].enabled || !s.state.calibrated) {
      error = "gripper disabled or uncalibrated";
      return false;
    }
    if (s.hold_failed || s.hold) {
      error = "gripper stop pending or unconfirmed";
      return false;
    }
    if (s.queue.size() >= 32) {
      error = "gripper command queue full";
      return false;
    }
    const auto conflicts = [&](const Handle &t) { return t && (c.action || t->command.action); };
    if (conflicts(s.current)) {
      error = "gripper is owned by another command interface";
      return false;
    }
    for (const auto &t : s.queue)
      if (conflicts(t)) {
        error = "gripper is owned by another command interface";
        return false;
      }
  }
  for (const auto &c : commands) {
    auto &s = sides_[c.side];
    if (s.current && !s.current->cancel)
      s.current->cancel = Preempted;
    for (auto &t : s.queue)
      if (!t->cancel)
        t->cancel = Preempted;
    auto t = std::make_shared<Ticket>(c);
    t->deadline = Clock::now() + std::chrono::milliseconds(c.timeout_ms);
    s.queue.push_back(t);
    handles.push_back(t);
  }
  error.clear();
  cv_.notify_all();
  return true;
}
void GripperController::cancel(const std::vector<Handle> &handles) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto &t : handles)
    if (t && !t->outcome.done)
      t->cancel = Canceled;
  cv_.notify_all();
}
void GripperController::stop(unsigned side) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_ || side > 2)
    return;
  for (unsigned i = 0; i < 2; ++i)
    if ((side == 2 || side == i) && configs_[i].enabled) {
      auto &s = sides_[i];
      // Active execution performs its own hold; an idle bus needs an explicit hold.
      s.hold = !s.current;
      if (s.current)
        s.current->cancel = Canceled;
      for (auto &t : s.queue)
        t->cancel = Canceled;
    }
  cv_.notify_all();
}
void GripperController::requestShutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  running_ = false;
  for (auto &s : sides_) {
    if (s.current)
      s.current->cancel = Canceled;
    for (auto &t : s.queue)
      t->cancel = Canceled;
  }
  cv_.notify_all();
}
void GripperController::shutdown() {
  requestShutdown();
  for (auto &s : sides_)
    if (s.worker.joinable())
      s.worker.join();
  if (attached_) {
    MarvinLink::Release();
    attached_ = false;
  }
}
bool GripperController::busy(unsigned i) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto &s = sides_.at(i);
  return bool(s.current) || !s.queue.empty() || s.hold || s.hold_failed;
}
GripperController::Outcome GripperController::outcome(const Handle &t) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return t->outcome;
}
GripperController::State GripperController::state(unsigned i) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return sides_.at(i).state;
}
bool GripperController::interrupted(const Handle &t, uint8_t &code, std::string &message) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (t->cancel || !running_ || (shutdown_requested_ && shutdown_requested_())) {
    code = t->cancel ? t->cancel : static_cast<uint8_t>(Canceled);
    message =
        code == Preempted ? "replaced by newer topic command" : "stop/cancel/shutdown requested";
    return true;
  }
  if (Clock::now() >= t->deadline) {
    code = Timeout;
    message = "gripper command timed out";
    return true;
  }
  return false;
}
bool GripperController::hold(unsigned i, std::string &message) {
  auto &s = sides_[i];
  // Do not re-enable a protected servo while handling cancel/stop/shutdown.
  uint8_t state_bytes[DS485_FRAME_MAX]{};
  if (!s.servo->ReadReg(DS485_REG_STATE, state_bytes, 2)) {
    message += "; hold UNCONFIRMED (cannot read protection state): " + s.servo->LastError();
  } else if (state_bytes[0] & DS485_STATE_FAULT_MASK) {
    message += "; hold refused: servo protection active, inspection required";
  } else if (s.gripper->Stop()) {
    message += "; hold acknowledged";
    return true;
  } else
    message += "; hold FAILED: " + s.gripper->LastError();
  std::lock_guard<std::mutex> lock(mutex_);
  s.hold_failed = true;
  return false;
}
void GripperController::finish(unsigned i, const Handle &t, uint8_t code,
                               const std::string &message) {
  Outcome out;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (t->outcome.done)
      return;
    t->outcome.done = true;
    t->outcome.result = code;
    t->outcome.message = message;
    t->outcome.state = sides_[i].state;
    out = t->outcome;
    if (sides_[i].current == t)
      sides_[i].current.reset();
  }
  cv_.notify_all();
  if (result_callback_ && !t->command.action)
    result_callback_(t->command, out);
}
void GripperController::updateState(unsigned i, bool online, const ServoStatus &status,
                                    const std::string &error) {
  State out;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto &s = sides_[i];
    s.state.online = online;
    s.state.error = error;
    s.state.busy = bool(s.current) || !s.queue.empty() || s.hold || s.hold_failed;
    if (online) {
      s.state.servo = status;
      s.state.position = s.gripper->AngleToPosition(status.angle);
    }
    s.state.max_speed = s.gripper->MaxSpeed();
    out = s.state;
  }
  if (state_callback_)
    state_callback_(i, out);
}
void GripperController::execute(unsigned i, const Handle &t) {
  auto &s = sides_[i];
  uint8_t code = 0;
  std::string message;
  if (interrupted(t, code, message)) {
    finish(i, t, code, message);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (s.hold_failed) {
      code = Failed;
      message = "previous hold failed; inspection required";
    }
  }
  if (code) {
    finish(i, t, code, message);
    return;
  }
  if (!s.gripper->IsReady() && !s.gripper->Init()) {
    finish(i, t, Failed, "gripper initialization failed: " + s.gripper->LastError());
    return;
  }
  if (s.servo->SpeedLimit() == 0) {
    finish(i, t, Failed, "servo speed limit 0x5D is unavailable or zero; motion refused");
    return;
  }
  if (interrupted(t, code, message)) {
    finish(i, t, code, message);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    t->outcome.started = true;
  }
  bool sent = false;
  const auto &c = t->command;
  if (!s.gripper->StartMove(c.position, c.torque, c.speed, &sent)) {
    message = "command failed: " + s.gripper->LastError();
    if (sent) {
      uint8_t state[DS485_FRAME_MAX]{};
      if (s.servo->ReadReg(DS485_REG_STATE, state, 2) && (state[0] & DS485_STATE_PROTECTION_MASK)) {
        ServoStatus status{};
        const bool online = s.servo->ReadStatus(&status);
        updateState(i, online, status, online ? "" : s.servo->LastError());
        finish(i, t, Fault, message);
        return;
      }
      hold(i, message);
    }
    finish(i, t, Failed, message);
    return;
  }
  while (true) {
    if (interrupted(t, code, message)) {
      if (!hold(i, message))
        code = Failed;
      finish(i, t, code, message);
      return;
    }
    ServoStatus status{};
    bool have = false;
    const int result = s.gripper->Update(&status, &have);
    updateState(i, have, status, have ? "" : s.gripper->LastError());
    if (result == GRIPPER_FAULT) {
      finish(i, t, Fault, s.gripper->LastError());
      return;
    }
    // A stop arriving during a read must be honored before reporting success.
    if (interrupted(t, code, message)) {
      if (!hold(i, message))
        code = Failed;
      finish(i, t, code, message);
      return;
    }
    if (result == GRIPPER_ARRIVED || result == GRIPPER_BLOCKED) {
      finish(i, t, result == GRIPPER_ARRIVED ? Arrived : Blocked,
             result == GRIPPER_ARRIVED ? "target reached" : "blocked; holding object");
      return;
    }
    if (result == GRIPPER_ERROR) {
      message = s.gripper->LastError();
      hold(i, message);
      finish(i, t, Failed, message);
      return;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, std::chrono::milliseconds(poll_ms_), [&] { return t->cancel || !running_; });
  }
}
void GripperController::worker(unsigned i) {
  auto &s = sides_[i];
  auto next_state = Clock::now();
  bool initial_read = true;
  bool check_on_connect = true;
  try {
    while (true) {
      if (shutdown_requested_ && shutdown_requested_())
        requestShutdown();
      Handle t;
      bool do_hold = false, read_state = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (s.hold)
          do_hold = true;
        else if (!s.queue.empty()) {
          t = s.queue.front();
          s.queue.pop_front();
          s.current = t;
        } else if (!running_)
          break;
        else if (initial_read || (state_hz_ > 0 && Clock::now() >= next_state))
          read_state = true;
        else {
          cv_.wait_for(lock, std::chrono::milliseconds(poll_ms_));
          continue;
        }
      }
      if (do_hold) {
        std::string error = "explicit stop";
        const bool ok = hold(i, error);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          s.hold = false;
        }
        ServoStatus status{};
        const bool read = s.servo->ReadStatus(&status);
        updateState(i, read, status, ok ? "" : error);
      } else if (t)
        execute(i, t);
      else if (read_state) {
        ServoStatus status{};
        const bool ok = s.servo->ReadStatus(&status);
        initial_read = false;
        std::string error;
        if (!ok) {
          check_on_connect = true;
          error = s.servo->LastError();
        } else if (check_on_connect) {
          check_on_connect = false;
          if (!s.servo->Probe() || !s.servo->EnsureProtection())
            error = "protection configuration: " + s.servo->LastError();
        }
        updateState(i, ok, status, error);
        if (state_hz_ > 0)
          next_state = Clock::now() +
                       std::chrono::milliseconds(ok ? static_cast<int>(1000.0 / state_hz_) : 2000);
      }
    }
  } catch (const std::exception &e) {
    std::string message = std::string("DS485 worker exception: ") + e.what();
    Handle active;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active = s.current;
      s.hold_failed = true;
      s.hold = false;
    }
    if (active) {
      hold(i, message);
      finish(i, active, Failed, message);
    }
    while (true) {
      Handle t;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (s.queue.empty())
          break;
        t = s.queue.front();
        s.queue.pop_front();
      }
      finish(i, t, Failed, message);
    }
  }
}
} // namespace ds485_driver
