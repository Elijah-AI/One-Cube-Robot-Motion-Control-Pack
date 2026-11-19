#pragma once

#include "ds485_driver/ds485_gripper.h"
#include <array>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace ds485_driver {

// One executor owns both buses. Only its workers may touch the servo objects.
class GripperController {
public:
  enum Result : uint8_t {
    Arrived = 1,
    Blocked = 2,
    Fault = 3,
    Timeout = 4,
    Failed = 5,
    Canceled = 6,
    Preempted = 7
  };
  struct Config {
    bool enabled = true;
    uint8_t id = 1, channel = 2;
    int gap_ms = 10, reply_timeout_ms = 300;
    GripperConfig gripper;
    ServoProtectionConfig protection;
  };
  struct Command {
    uint32_t id = 0;
    uint8_t side = 0, position = 0, torque = 0, speed = 0;
    int64_t timeout_ms = 10000;
    bool action = false;
  };
  struct State {
    bool online = false, busy = false, calibrated = false;
    uint8_t position = 0;
    uint16_t max_speed = 0;
    ServoStatus servo{};
    std::string error;
  };
  struct Outcome {
    bool done = false, started = false;
    uint8_t result = 0;
    std::string message;
    State state;
    bool success() const {
      return result == Arrived || result == Blocked;
    }
  };
  struct Ticket {
    const Command command;
    explicit Ticket(const Command &value) : command(value) {}

  private:
    friend class GripperController;
    Outcome outcome;
    uint8_t cancel = 0;
    std::chrono::steady_clock::time_point deadline;
  };
  using Handle = std::shared_ptr<Ticket>;
  using PortFactory = std::function<std::unique_ptr<Rs485Port>(unsigned, uint8_t)>;
  using ResultCallback = std::function<void(const Command &, const Outcome &)>;
  using FaultLogger = std::function<void(unsigned, const std::string &)>;
  using StateCallback = std::function<void(unsigned, const State &)>;

  GripperController(const std::array<Config, 2> &, PortFactory = {}, ResultCallback = {},
                    StateCallback = {}, int poll_ms = 20, double state_hz = 10.0,
                    std::function<bool()> shutdown_requested = {}, FaultLogger fault_logger = {});
  ~GripperController();
  // Production: the host must connect its SDK first. This only Attach-es and never calls Connect.
  // With an injected PortFactory (offline tests), no SDK operation is performed.
  void start();
  // A batch is admitted atomically. Actions never preempt; topics only preempt topics.
  // Returns acceptance immediately; outcome(handle) returns state/result directly in C++.
  // Command.action=true reserves that side against topic-like preemption.
  bool submit(const std::vector<Command> &, std::vector<Handle> &, std::string &);
  void cancel(const std::vector<Handle> &);
  void stop(unsigned side); // 0/1/2; also holds an idle gripper
  void requestShutdown();
  void shutdown(); // joins workers before the owner releases its SDK
  bool busy(unsigned side) const;
  Outcome outcome(const Handle &) const;
  State state(unsigned side) const;
  const Config &config(unsigned side) const {
    return configs_.at(side);
  }

private:
  struct Side {
    std::unique_ptr<Rs485Port> port;
    std::unique_ptr<Ds485Servo> servo;
    std::unique_ptr<Ds485Gripper> gripper;
    std::deque<Handle> queue;
    Handle current;
    State state;
    bool hold = false, hold_failed = false;
    std::thread worker;
  };
  void worker(unsigned);
  void execute(unsigned, const Handle &);
  bool interrupted(const Handle &, uint8_t &, std::string &) const;
  bool hold(unsigned, std::string &);
  void finish(unsigned, const Handle &, uint8_t, const std::string &);
  void updateState(unsigned, bool, const ServoStatus &, const std::string &);
  std::array<Config, 2> configs_;
  std::array<Side, 2> sides_;
  ResultCallback result_callback_;
  StateCallback state_callback_;
  int poll_ms_;
  double state_hz_;
  std::function<bool()> shutdown_requested_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool running_ = false, started_ = false;
  bool use_shared_sdk_ = false, attached_ = false;
};
} // namespace ds485_driver
