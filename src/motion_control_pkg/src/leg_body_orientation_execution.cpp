#include "miracle_lower_body_module.hpp"
#include "motion_control_pkg/execution/action_terminal_guard.hpp"
#include "motion_control_pkg/execution/stream_timing.hpp"
#include "motion_control_pkg/motion/leg_body_orientation.hpp"
#include "motion_control_pkg/motion/scoped_collision_scene.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace miracle {
namespace {
constexpr double kStationaryVelocity = 0.02;  // rad/s
constexpr double kFeedbackAge = 0.1;          // s
using Clock = std::chrono::steady_clock;
using motion_control_pkg::motion::LegJoints;

double maximumError(const LegJoints& a, const LegJoints& b) {
  double result = 0;
  for (std::size_t i = 0; i < a.size(); ++i) result = std::max(result, std::abs(a[i] - b[i]));
  return result;
}
bool stationary(const LegJoints& velocities) {
  return std::all_of(velocities.begin(), velocities.end(),
      [](double v) { return std::isfinite(v) && std::abs(v) <= kStationaryVelocity; });
}
}

bool MiracleLowerBodyModule::read_body_orientation_feedback(LegJoints& positions,
    LegJoints& velocities, uint64_t& sequence, std::string& error) {
  std::vector<DriverSDK::motorActualStruct> motors;
  Clock::time_point acquired;
  if (!m_bus_manager || !m_bus_manager->getTimedMotorActualSnapshot(motors, acquired, sequence)) {
    error = "Body-orientation MoveJ requires measured leg feedback";
    return false;
  }
  const auto now = Clock::now();
  if (sequence == 0 || acquired == Clock::time_point{} || acquired > now ||
      std::chrono::duration<double>(now - acquired).count() > kFeedbackAge || m_jointID.size() != 4) {
    error = "Body-orientation MoveJ leg feedback is stale or invalid";
    return false;
  }
  for (std::size_t i = 0; i < 4; ++i) {
    if (m_jointID[i] >= motors.size()) {
      error = "Body-orientation MoveJ is missing a leg motor";
      return false;
    }
    const auto& motor = motors[m_jointID[i]];
    if (motor.statusWord == 0 || motor.statusWord == 0xffff ||
        (motor.statusWord & 0x0008) || motor.errorCode ||
        !std::isfinite(motor.pos) || !std::isfinite(motor.vel)) {
      error = "Invalid leg motor feedback: motor " + std::to_string(m_jointID[i]);
      return false;
    }
    positions[i] = motor.pos;  // DriverSDK already applies joint direction and zero calibration.
    velocities[i] = motor.vel;
  }
  error.clear();
  return true;
}

void MiracleLowerBodyModule::execution_loop_body_orientation(
    std::shared_ptr<GoalHandleMoveJoints> goal_handle) {
  struct Release {
    std::atomic<bool>& busy;
    bool& running;
    ~Release() { running = false; busy.store(false); }
  } release{is_executing_leg_, m_traj_run};
  motion_control_pkg::execution::ActionTerminalGuard<GoalHandleMoveJoints, MoveJoints::Result> terminal(goal_handle);
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveJoints::Result>();
  result->final_distance_error = std::numeric_limits<double>::infinity();
  const auto started = Clock::now();
  auto motion_started = Clock::time_point{};
  double planning_time = 0;
  const auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - started).count(); };
  const auto remaining = [&] { return static_cast<double>(goal->timeout_ms) / 1000.0 - elapsed(); };
  const auto update_timing = [&] {
    result->timing_breakdown_valid = true;
    result->ik_time = 0;
    result->planning_time = planning_time;
    result->motion_time = motion_started == Clock::time_point{} ? 0.0 :
        std::chrono::duration<double>(Clock::now() - motion_started).count();
    result->execution_time = elapsed();
  };
  const auto check_request = [&] {
    if (goal_handle->is_canceling()) throw std::runtime_error("Body-orientation MoveJ canceled");
    if (!rclcpp::ok()) throw std::runtime_error("ROS shutdown during body-orientation MoveJ");
    if (remaining() <= 0) throw std::runtime_error("Body-orientation MoveJ timeout");
  };
  const auto publish = [&](uint8_t state, float progress, const std::string& text) {
    auto feedback = std::make_shared<MoveJoints::Feedback>();
    feedback->dual_task_status = state;
    feedback->task_status = text;
    feedback->current_step_info = text;
    feedback->progress_rate = progress;
    if (auto node = lockNode(__func__)) feedback->header.stamp = node->now();
    goal_handle->publish_feedback(feedback);
  };
  try {
    if (!m_galgo || goal->control_mode != MoveJoints::Goal::CONTROL_POSITION || goal->timeout_ms <= 0)
      throw std::invalid_argument("Body-orientation MoveJ requires a model, position control and positive timeout_ms");
    check_request();
    LegJoints start{}, velocity{}, measured{};
    uint64_t sequence = 0;
    std::string error;
    if (!read_body_orientation_feedback(start, velocity, sequence, error)) throw std::runtime_error(error);
    if (!stationary(velocity)) throw std::runtime_error("Leg must be stationary before body-orientation MoveJ");
    motion_control_pkg::motion::LegBodyOrientationPlanner planner(m_galgo->getModel());
    publish(MoveJoints::Feedback::STATUS_PLANNING, 0, "Planning leg MoveJ with fixed body orientation");
    const auto planning_started = Clock::now();
    const auto trajectory = planner.plan(start, goal->joint_name, goal->position,
        goal->speed, 1.0 / TRAJ_FREQ, remaining());
    // Preserve the route's configured collision behavior, including environment mode 2.
    motion_control_pkg::motion::checkChainCollision(*m_galgo, goal->enable_collision_check,
        ID_LEG, trajectory.positions);
    planning_time = std::chrono::duration<double>(Clock::now() - planning_started).count();
    check_request();

    uint64_t previous_sequence = sequence;
    const auto read_and_check = [&] {
      if (!read_body_orientation_feedback(measured, velocity, sequence, error)) throw std::runtime_error(error);
      if (sequence < previous_sequence) throw std::runtime_error("Leg feedback sequence moved backwards");
      previous_sequence = sequence;
      if (planner.orientationError(measured, trajectory.reference_orientation) > body_orientation_tracking_tolerance_rad_)
        throw std::runtime_error("Measured body orientation exceeded leg_body_orientation_tracking_tolerance_rad");
      result->final_distance_error = maximumError(measured, trajectory.target);
    };
    read_and_check();
    if (maximumError(measured, start) > JOINT_THRESHOLD || !stationary(velocity))
      throw std::runtime_error("Leg moved during body-orientation planning; resend from its current state");
    if (trajectory.duration == 0) {
      result->success = true;
      result->error_message = "Leg already at target; body orientation unchanged";
      update_timing(); terminal.succeed(result); return;
    }
    if (remaining() <= trajectory.duration) throw std::runtime_error("Insufficient timeout remaining for leg trajectory");
    if (!m_bus_manager->enableMotors(m_jointID, std::min(5.0, remaining())))
      throw std::runtime_error("Leg motor enable safety check failed");
    check_request();
    read_and_check();
    if (maximumError(measured, start) > JOINT_THRESHOLD || !stationary(velocity))
      throw std::runtime_error("Leg moved during enable; body-orientation trajectory was not dispatched");
    if (remaining() <= trajectory.duration) throw std::runtime_error("Insufficient timeout after motor enable");

    RCLCPP_INFO(rclcpp::get_logger("leg_module"),
        "Body-orientation MoveJ: compensate J%zu, target=[%.6f,%.6f,%.6f,%.6f], duration=%.3fs, FK error=%.9f rad",
        trajectory.compensated_joint + 1, trajectory.target[0], trajectory.target[1], trajectory.target[2],
        trajectory.target[3], trajectory.duration, trajectory.max_orientation_error);
    m_traj_run = true;
    motion_started = Clock::now();
    auto tick = motion_started;
    uint64_t final_command_sequence = sequence;
    constexpr int64_t period_ns = TRAJ_CYCLE_US * 1000LL;
    for (std::size_t i = 0; i < trajectory.positions.size(); ++i) {
      std::this_thread::sleep_until(tick);
      check_request();
      const auto dispatch_started = Clock::now();
      read_and_check();
      if (!setMulJointPos(trajectory.positions[i], m_jointID)) throw std::runtime_error("Leg motor command rejected");
      final_command_sequence = sequence;
      if (i % 25 == 0)
        publish(MoveJoints::Feedback::STATUS_EXECUTING, 95.0F * i / trajectory.positions.size(),
            "Executing synchronized leg MoveJ with fixed body orientation");
      const auto ns = [](Clock::time_point t) { return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count(); };
      tick = Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::nanoseconds(
          motion_control_pkg::execution::nextStreamTick(ns(tick), ns(dispatch_started), ns(Clock::now()), period_ns))));
    }
    publish(MoveJoints::Feedback::STATUS_WAITING_FOR_COMPLETION, 95, "Waiting for measured leg target and body orientation");
    while (true) {
      std::this_thread::sleep_for(std::chrono::microseconds(TRAJ_CYCLE_US));
      check_request();
      read_and_check();
      if (sequence > final_command_sequence && result->final_distance_error <= JOINT_REACH_THRESHOLD && stationary(velocity)) break;
    }
    result->success = true;
    result->error_message = "Leg target reached with body orientation preserved";
    update_timing(); terminal.succeed(result);
  } catch (const std::exception& error) {
    result->success = false;
    result->error_message = error.what();
    update_timing();
    if (goal_handle->is_canceling()) terminal.cancel(result);
    else terminal.abort(result);
  } catch (...) {
    result->success = false;
    result->error_message = "Unknown body-orientation MoveJ failure";
    update_timing();
    if (goal_handle->is_canceling()) terminal.cancel(result);
    else terminal.abort(result);
  }
}
}  // namespace miracle
