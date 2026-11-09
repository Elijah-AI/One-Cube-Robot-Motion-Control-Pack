#include "motion_control_pkg/execution/latest_progress.hpp"
#include "motion_control_pkg/execution/joint_completion.hpp"
#include "miracle_lower_body_module.hpp"
#include "com_interfaces/leg_body_motion_goal.hpp"
#include "motion_control_pkg/motion/leg_body_motion.hpp"
#include "motion_control_pkg/motion/scoped_collision_scene.hpp"
#include "motion_control_pkg/execution/action_terminal_guard.hpp"
#include "motion_control_pkg/execution/stream_timing.hpp"

#include <map>
#include <set>
#include <sstream>
#include <limits>
#include <stdexcept>

namespace miracle {
namespace {
using Clock = std::chrono::steady_clock;
using motion_control_pkg::motion::LegJoints;
constexpr double kStationary = 0.02;
constexpr double kHeldJointTolerance = 0.01;
constexpr double kBodyPositionTracking = 0.01;  // m, servo tracking guard
constexpr double kBodyPositionCompletion = 0.001;
const std::array<std::string, 4> kLegNames{{"leg_joint1", "leg_joint2", "leg_joint3", "body_joint"}};
double jointError(const LegJoints& a, const LegJoints& b) {
  double error = 0;
  for (std::size_t j = 0; j < 4; ++j) error = std::max(error, std::abs(a[j] - b[j]));
  return error;
}
double orientationError(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle();
}
}

void MiracleLowerBodyModule::execution_loop_body_motion(std::shared_ptr<GoalHandleMoveJoints> handle) {
  struct Release {
    MiracleLowerBodyModule& owner;
    bool upper_reserved{false};
    ~Release() {
      if (upper_reserved) owner.release_upper_body_();
      owner.m_traj_run = false;
      owner.is_executing_leg_.store(false);
    }
  } release{*this};
  motion_control_pkg::execution::ActionTerminalGuard<GoalHandleMoveJoints, MoveJoints::Result> terminal(handle);
  const auto goal = handle->get_goal();
  const auto action_precision = motion_control_pkg::execution::actionPrecisionPolicy(
      precision_policy_, goal->precision_arrival);
  auto result = std::make_shared<MoveJoints::Result>();
  motion_control_pkg::execution::MotionSafetyScope safety_scope(4U);
  result->final_distance_error = std::numeric_limits<double>::infinity();
  const auto started = Clock::now();
  auto motion_started = Clock::time_point{};
  double planning_time = 0;
  const auto remaining = [&] {
    return goal->timeout_ms / 1000.0 - std::chrono::duration<double>(Clock::now() - started).count();
  };
  const auto timing = [&] {
    result->timing_breakdown_valid = true;
    result->planning_time = planning_time;
    result->ik_time = 0;  // Body IK is included in the leg planning stage.
    result->motion_time = motion_started == Clock::time_point{} ? 0 :
        std::chrono::duration<double>(Clock::now() - motion_started).count();
    result->execution_time = std::chrono::duration<double>(Clock::now() - started).count();
  };
  const auto check = [&] {
    if (handle->is_canceling()) throw std::runtime_error("Leg body motion canceled");
    if (!rclcpp::ok()) throw std::runtime_error("ROS shutdown during leg body motion");
    if (remaining() <= 0) throw std::runtime_error("Leg body motion timeout");
  };
  const auto publish = [&](uint8_t state, float progress, const std::string& message) {
    auto feedback = std::make_shared<MoveJoints::Feedback>();
    feedback->dual_task_status = state;
    feedback->task_status = feedback->current_step_info = message;
    feedback->progress_rate = progress;
    if (auto node = lockNode(__func__)) feedback->header.stamp = node->now();
    handle->publish_feedback(feedback);
  };
  try {
    std::string error;
    if (!com_interfaces::validateLegBodyMotionGoal(goal->arm_id, goal->leg_motion_mode,
        goal->body_tilt_deg, goal->body_height_m, goal->joint_name, goal->position, goal->speed,
        goal->timeout_ms, goal->control_mode, goal->acceleration_scale, goal->jerk_scale, false, error))
      throw std::invalid_argument(error);
    if (!m_galgo || !reserve_upper_body_ || !release_upper_body_ || !read_upper_body_)
      throw std::runtime_error("Leg body motion requires a model and upper-body resource/feedback callbacks");
    if (!goal->header.frame_id.empty() && goal->header.frame_id != "base_link")
      throw std::invalid_argument("Leg body targets must use base_link coordinates");
    if (auto node = lockNode(__func__)) {
      bool dynamic = false;
      if (node->has_parameter("wbc_dynamic_zmp_enabled")) node->get_parameter("wbc_dynamic_zmp_enabled", dynamic);
      if (dynamic) throw std::runtime_error("Leg body motion requires static balance mode; dynamic ZMP planning is not supported on this route");
    }
    check();
    if (!reserve_upper_body_(error)) throw std::runtime_error(error);
    release.upper_reserved = true;
    const auto& model = m_galgo->getModel();
    using Snapshot = std::map<std::string, std::pair<double, double>>;
    const auto readFull = [&]() {
      sensor_msgs::msg::JointState state;
      if (!read_upper_body_(state, error)) throw std::runtime_error(error);
      auto node = lockNode(__func__);
      const double stamp = rclcpp::Time(state.header.stamp).seconds();
      const double now = node ? node->now().seconds() : -1;
      if (!node || stamp <= 0 || stamp > now || now - stamp > 0.1 || state.name.size() != 20 ||
          state.position.size() != state.name.size() || state.velocity.size() != state.name.size())
        throw std::runtime_error("Body motion requires complete, fresh measured arm/leg/waist/head feedback");
      Snapshot snapshot;
      for (std::size_t j = 0; j < state.name.size(); ++j) {
        if (!model.existJointName(state.name[j]) ||
            model.joints[model.getJointId(state.name[j])].nq() != 1 ||
            !std::isfinite(state.position[j]) || !std::isfinite(state.velocity[j]) ||
            !snapshot.emplace(state.name[j], std::make_pair(state.position[j], state.velocity[j])).second)
          throw std::runtime_error("Invalid or duplicate measured body-motion joint: " + state.name[j]);
      }
      for (const auto& name : kLegNames)
        if (!snapshot.count(name)) throw std::runtime_error("Missing measured joint: " + name);
      for (const auto& name : {"head_yaw_joint", "head_pitch_joint"})
        if (!snapshot.count(name)) throw std::runtime_error("Missing measured joint: " + std::string(name));
      for (const auto& side : {"L", "R"})
        for (int joint = 1; joint <= 7; ++joint) {
          const auto name = "Joint" + std::to_string(joint) + "_" + side;
          if (!snapshot.count(name)) throw std::runtime_error("Missing measured joint: " + name);
        }
      return snapshot;
    };
    const auto initial = readFull();
    for (const auto& entry : initial)
      if (std::abs(entry.second.second) > kStationary)
        throw std::runtime_error("All body-motion joints must start stationary: " + entry.first);
    LegJoints start{}, measured{}, velocity{};
    uint64_t sequence = 0;
    if (!read_body_orientation_feedback(start, velocity, sequence, error)) throw std::runtime_error(error);
    for (std::size_t j = 0; j < 4; ++j)
      if (std::abs(start[j] - initial.at(kLegNames[j]).first) > JOINT_THRESHOLD)
        throw std::runtime_error("Leg and full-body feedback are inconsistent");
    motion_control_pkg::motion::LegBodyMotionPlanner planner(model);
    publish(MoveJoints::Feedback::STATUS_PLANNING, 0, "Planning URDF body tilt/vertical path");
    const auto planning_started = Clock::now();
    const auto trajectory = planner.plan(start,
        static_cast<motion_control_pkg::motion::LegBodyMotionMode>(goal->leg_motion_mode),
        goal->body_tilt_deg, goal->body_height_m, goal->speed, 1.0 / TRAJ_FREQ, remaining());
    std::vector<double> fixed_q;
    if (!m_galgo->getFullJointState(fixed_q)) throw std::runtime_error("Missing model joint state");
    for (const auto& entry : initial)
      fixed_q.at(model.joints[model.getJointId(entry.first)].idx_q()) = entry.second.first;
    if (m_galgo->staticBalance()) {
      const auto policy = m_galgo->staticBalance()->statePolicy();
      for (const auto& joint : m_galgo->staticBalance()->model().joints())
        if (!joint.locked && !initial.count(joint.name) && !policy.assumed_positions.count(joint.name))
          throw std::runtime_error("Body COM validation lacks measured or explicitly assumed joint: " + joint.name);
      for (const auto& entry : policy.assumed_positions)
        if (model.existJointName(entry.first))
          fixed_q.at(model.joints[model.getJointId(entry.first)].idx_q()) = entry.second;
    }
    std::vector<std::vector<double>> full_path;
    full_path.reserve(trajectory.joints.positions.size());
    for (const auto& point : trajectory.joints.positions) {
      auto full = fixed_q;
      for (std::size_t j = 0; j < 4; ++j)
        full.at(model.joints[model.getJointId(kLegNames[j])].idx_q()) = point[j];
      full_path.push_back(std::move(full));
    }
    // COM checks remain enabled even when geometric collision checking is off.
    if (!m_galgo->validateStaticBalancePath(full_path, error, 1.0 / TRAJ_FREQ)) throw std::runtime_error(error);
    {
      motion_control_pkg::motion::ScopedCollisionScene scene(*m_galgo, goal->enable_collision_check, fixed_q);
      for (const auto& q : full_path)
        if (!m_galgo->isFullQValid(Eigen::Map<const Eigen::VectorXd>(q.data(), q.size())))
          throw std::runtime_error("Body path rejected by joint limits or collision checks");
    }
    planning_time = std::chrono::duration<double>(Clock::now() - planning_started).count();
    LegJoints commanded = start;
    uint64_t previous_sequence = sequence;
    const auto readAndCheck = [&]() {
      check();
      const auto state = readFull();
      for (const auto& entry : initial) {
        const bool moving_leg = entry.first == "leg_joint3" ||
            (trajectory.vertical && (entry.first == "leg_joint1" || entry.first == "leg_joint2"));
        if (!moving_leg && (std::abs(state.at(entry.first).first - entry.second.first) > kHeldJointTolerance ||
            std::abs(state.at(entry.first).second) > kStationary))
          throw std::runtime_error("Held joint moved during body motion: " + entry.first);
      }
      if (!read_body_orientation_feedback(measured, velocity, sequence, error)) throw std::runtime_error(error);
      if (sequence < previous_sequence) throw std::runtime_error("Leg feedback sequence moved backwards");
      previous_sequence = sequence;
      const auto actual = planner.pose(measured);
      const auto expected = trajectory.vertical ? trajectory.start_body : planner.pose(commanded);
      if (orientationError(actual, expected) > body_orientation_tracking_tolerance_rad_)
        throw std::runtime_error("Measured body orientation exceeded its tracking tolerance");
      if (trajectory.vertical &&
          (actual.translation().head<2>() - trajectory.start_body.translation().head<2>()).norm() > kBodyPositionTracking)
        throw std::runtime_error("Measured body X/Y moved off the vertical path");
      result->final_distance_error = jointError(measured, trajectory.joints.target);
      auto full = fixed_q;
      for (const auto& entry : state)
        full.at(model.joints[model.getJointId(entry.first)].idx_q()) = entry.second.first;
      if (!m_galgo->isStaticBalanceValid(Eigen::Map<const Eigen::VectorXd>(full.data(), full.size())))
        throw std::runtime_error("Measured body motion violated its static COM constraint");
      return actual;
    };
    readAndCheck();
    if (jointError(measured, start) > JOINT_THRESHOLD)
      throw std::runtime_error("Leg moved during body planning; resend from measured state");
    if (!motion_control_pkg::execution::hasCompletionBudget(trajectory.joints.positions.size(), remaining(), action_precision.completion.stable_seconds))
      throw std::runtime_error("Insufficient body-motion timeout for trajectory and settling");
    if (trajectory.joints.duration != 0) {
      if (!m_bus_manager->enableMotors(m_jointID, std::min(5.0, remaining())))
        throw std::runtime_error("Leg motor enable failed");
      readAndCheck();
      if (jointError(measured, start) > JOINT_THRESHOLD ||
          std::any_of(velocity.begin(), velocity.end(), [](double v) { return std::abs(v) > kStationary; }))
        throw std::runtime_error("Leg moved during enable; body path was not dispatched");
      if (!motion_control_pkg::execution::hasCompletionBudget(trajectory.joints.positions.size(), remaining(), action_precision.completion.stable_seconds))
        throw std::runtime_error("Insufficient timeout for trajectory and settling after enable");
      publish(MoveJoints::Feedback::STATUS_EXECUTING,0,"Executing body tilt/vertical trajectory");
      motion_control_pkg::execution::LatestProgressPublisher progress(
          [&](float value){publish(MoveJoints::Feedback::STATUS_EXECUTING,value,"Executing body tilt/vertical trajectory");});
      motion_started = Clock::now();
      auto tick = motion_started;
      m_traj_run = true;
      constexpr int64_t period_ns = TRAJ_CYCLE_US * 1000LL;
      try {
        for (std::size_t n = 0; n < trajectory.joints.positions.size(); ++n) {
          std::this_thread::sleep_until(tick);
          check();
          if (progress.failed()) throw std::runtime_error(progress.error());
          const auto dispatch_started = Clock::now();
          readAndCheck();
          const auto& point = trajectory.joints.positions[n];
          if (!setMulJointPos(point, m_jointID)) throw std::runtime_error("Body-motion leg command rejected");
          std::copy(point.begin(), point.end(), commanded.begin());
          progress.update(95.0F * n / trajectory.joints.positions.size());
          const auto ns = [](Clock::time_point t) {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
          };
          tick = Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::nanoseconds(
              motion_control_pkg::execution::nextStreamTick(ns(tick), ns(dispatch_started), ns(Clock::now()), period_ns))));
        }
      } catch (const std::exception& e) {
        std::string detail=e.what();
        motion_control_pkg::execution::MotionSafetyScope::beforeTerminal(false,detail);
        progress.stop();throw std::runtime_error(detail);
      } catch (...) {
        std::string detail="unknown body trajectory execution exception";
        motion_control_pkg::execution::MotionSafetyScope::beforeTerminal(false,detail);
        progress.stop();throw std::runtime_error(detail);
      }
      progress.stop();
      if (progress.failed()) throw std::runtime_error(progress.error());
    }
    using Guard = motion_control_pkg::execution::JointCompletionGuard;
    const auto settling = Clock::now();
    Guard guard({trajectory.joints.target.begin(), trajectory.joints.target.end()}, settling,
        std::min(started + std::chrono::milliseconds(goal->timeout_ms), settling +
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(action_precision.completion_timeout))),
        false, action_precision.completion);
    publish(MoveJoints::Feedback::STATUS_WAITING_FOR_COMPLETION, 95, "Checking measured body pose and held joints");
    while (true) {
      std::this_thread::sleep_for(std::chrono::microseconds(TRAJ_CYCLE_US));
      const auto actual = readAndCheck();
      const bool reached = result->final_distance_error <= JOINT_REACH_THRESHOLD &&
          orientationError(actual, trajectory.target_body) <= JOINT_REACH_THRESHOLD &&
          (!trajectory.vertical || (actual.translation() - trajectory.target_body.translation()).norm() <= kBodyPositionCompletion) &&
          std::all_of(velocity.begin(), velocity.end(), [](double v) { return std::abs(v) <= kStationary; });
      const auto status = guard.update(sequence, {measured.begin(), measured.end()},
          {velocity.begin(), velocity.end()}, Clock::now(), error, true, reached);
      if (status == Guard::Status::Failed) throw std::runtime_error(error);
      if (status == Guard::Status::Settled) break;
    }
    result->success = true;
    std::ostringstream description;
    description << "Body motion reached: body_link Z=" << planner.pose(measured).translation().z()
                << " m, inclination=" << planner.tiltDegrees(measured) << " deg";
    result->error_message = description.str();
    timing(); terminal.succeed(result);
  } catch (const std::exception& exception) {
    result->success = false;
    result->error_message = exception.what();
    timing();
    if (handle->is_canceling()) terminal.cancel(result); else terminal.abort(result);
  } catch (...) {
    result->success = false;
    result->error_message = "Unknown leg body motion failure";
    timing();
    if (handle->is_canceling()) terminal.cancel(result); else terminal.abort(result);
  }
}
}  // namespace miracle
