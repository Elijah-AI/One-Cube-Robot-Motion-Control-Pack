#include "whole_body_module.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <optional>

#include "motion_control_pkg/execution/control_period.hpp"
#include "motion_control_pkg/kinematics/transform_utils.hpp"

namespace
{

Eigen::Isometry3d interpolatePose(
    const Eigen::Isometry3d& start,
    const Eigen::Isometry3d& target,
    double alpha)
{
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond start_rotation(start.rotation());
    Eigen::Quaterniond target_rotation(target.rotation());
    result.linear() = start_rotation.slerp(alpha, target_rotation).normalized().toRotationMatrix();
    result.translation() =
        (1.0 - alpha) * start.translation() + alpha * target.translation();
    return result;
}

void addBoundOffset(
    geometry_msgs::msg::Pose& pose,
    const std::vector<float>& bound_size)
{
    if (bound_size.size() == 3U) {
        pose.position.x += bound_size[0];
        pose.position.y += bound_size[1];
        pose.position.z += bound_size[2];
    }
}

void addBoundOffset(
    geometry_msgs::msg::Pose& pose,
    const std::array<float, 3>& bound_size)
{
    pose.position.x += bound_size[0];
    pose.position.y += bound_size[1];
    pose.position.z += bound_size[2];
}

bool planRuckigSamplesWholeBody(
    MiracleMAlgo& miracle_algo,
    const std::vector<double>& start,
    const std::vector<double>& target,
    const std::vector<double>& velocity_limits,
    const std::vector<double>& acceleration_limits,
    const std::vector<double>& jerk_limits,
    std::vector<std::vector<double>>& trajectory,
    std::string& error_message)
{
    if (start.empty() || start.size() != target.size() ||
        start.size() != velocity_limits.size() ||
        start.size() != acceleration_limits.size() ||
        start.size() != jerk_limits.size()) {
        error_message = "whole-body Ruckig input dimensions do not match";
        return false;
    }
    const std::vector<double> zero_velocity(start.size(), 0.0);
    const std::vector<double> zero_acceleration(start.size(), 0.0);
    if (!miracle_algo.ruckigPlan(
            motion_control_pkg::execution::kDualArmControlPeriodSeconds,
            velocity_limits, acceleration_limits, jerk_limits,
            start, zero_velocity, zero_acceleration, target, trajectory)) {
        error_message = "whole-body Ruckig planning failed";
        return false;
    }
    error_message.clear();
    return true;
}

bool planToppraSamplesWholeBody(
    MiracleMAlgo& miracle_algo,
    const std::vector<std::vector<double>>& waypoints,
    const std::vector<double>& velocity_limits,
    const std::vector<double>& acceleration_limits,
    std::vector<std::vector<double>>& trajectory,
    std::string& error_message)
{
    if (waypoints.size() < 2U || velocity_limits.empty() ||
        velocity_limits.size() != acceleration_limits.size()) {
        error_message = "whole-body TOPPRA input dimensions are invalid";
        return false;
    }
    if (!miracle_algo.toppraPlan(
            waypoints,
            motion_control_pkg::execution::kDualArmControlPeriodSeconds,
            velocity_limits, acceleration_limits, trajectory)) {
        error_message = "whole-body TOPPRA planning failed";
        return false;
    }
    error_message.clear();
    return true;
}

bool buildCartesianLineScalars(
    MiracleMAlgo& miracle_algo,
    double distance,
    double speed,
    std::vector<double>& scalars,
    std::string& error_message)
{
    scalars.clear();
    if (!std::isfinite(distance) || distance < 0.0) {
        error_message = "invalid whole-body Cartesian line distance";
        return false;
    }
    if (distance <= 1e-8) {
        scalars = {0.0, 1.0};
        return true;
    }
    const double motion_rate = std::clamp(speed, 0.01, 1.0);
    std::vector<std::vector<double>> scalar_trajectory;
    if (!miracle_algo.ruckigPlan(
            motion_control_pkg::execution::kDualArmControlPeriodSeconds,
            {std::max(1e-3, motion_rate * CARTES_V_MAX)},
            {std::max(1e-3, motion_rate * CARTES_A_MAX)},
            {std::max(1e-3, motion_rate * CARTES_J_MAX)},
            {0.0}, {0.0}, {0.0}, {distance}, scalar_trajectory)) {
        error_message = "failed to time-parameterize whole-body Cartesian line";
        return false;
    }
    if (scalar_trajectory.empty()) {
        error_message = "whole-body Cartesian line planner returned no samples";
        return false;
    }
    if (scalar_trajectory.front().size() != 1U) {
        error_message = "whole-body Cartesian line planner returned an invalid sample";
        return false;
    }
    if (std::abs(scalar_trajectory.front().front()) > 1e-8) {
        scalars.push_back(0.0);
    }
    for (const auto& sample : scalar_trajectory) {
        if (sample.size() != 1U || !std::isfinite(sample.front())) {
            error_message = "whole-body Cartesian line planner returned an invalid sample";
            return false;
        }
        scalars.push_back(std::clamp(sample.front() / distance, 0.0, 1.0));
    }
    if (scalars.empty() || scalars.back() < 1.0 - 1e-8) {
        scalars.push_back(1.0);
    } else {
        scalars.back() = 1.0;
    }
    return true;
}

bool makeWholeBodyActiveWaypoint(
    MiracleMAlgo& miracle_algo,
    const std::vector<double>& full_q,
    bool use_left,
    bool use_right,
    std::vector<double>& active_q)
{
    active_q.clear();
    std::vector<double> arm_q;
    if (use_left) {
        if (!miracle_algo.extractSingleArmJoints(true, full_q, arm_q)) {
            return false;
        }
        active_q.insert(active_q.end(), arm_q.begin(), arm_q.end());
    }
    if (use_right) {
        if (!miracle_algo.extractSingleArmJoints(false, full_q, arm_q)) {
            return false;
        }
        active_q.insert(active_q.end(), arm_q.begin(), arm_q.end());
    }
    std::vector<double> leg_q;
    if (!miracle_algo.extractLegJoints(full_q, leg_q)) {
        return false;
    }
    active_q.insert(active_q.end(), leg_q.begin(), leg_q.end());
    return !active_q.empty();
}

bool splitWholeBodyActiveTrajectory(
    const std::vector<std::vector<double>>& active_trajectory,
    bool use_left,
    bool use_right,
    std::vector<std::vector<double>>& left_path,
    std::vector<std::vector<double>>& right_path,
    std::vector<std::vector<double>>& leg_path,
    std::string& error_message)
{
    left_path.clear();
    right_path.clear();
    leg_path.clear();
    const std::size_t arm_dof = 7U;
    const std::size_t leg_dof = 4U;
    const std::size_t expected =
        (use_left ? arm_dof : 0U) + (use_right ? arm_dof : 0U) + leg_dof;
    if (active_trajectory.empty() || expected == 0U) {
        error_message = "whole-body trajectory is empty";
        return false;
    }
    for (const auto& sample : active_trajectory) {
        if (sample.size() != expected) {
            error_message = "whole-body trajectory sample dimensions do not match";
            return false;
        }
        std::size_t offset = 0U;
        if (use_left) {
            left_path.emplace_back(sample.begin() + static_cast<std::ptrdiff_t>(offset),
                                   sample.begin() + static_cast<std::ptrdiff_t>(offset + arm_dof));
            offset += arm_dof;
        }
        if (use_right) {
            right_path.emplace_back(sample.begin() + static_cast<std::ptrdiff_t>(offset),
                                    sample.begin() + static_cast<std::ptrdiff_t>(offset + arm_dof));
            offset += arm_dof;
        }
        leg_path.emplace_back(sample.begin() + static_cast<std::ptrdiff_t>(offset),
                              sample.end());
    }
    return true;
}

}  // namespace

WholeBodyModule::WholeBodyModule(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<MiracleMAlgo>& miracle_algo,
    const std::shared_ptr<DualArmModule>& dual_arm_module,
    const std::shared_ptr<miracle::MiracleLowerBodyModule>& lower_body_module,
    std::string planning_frame,
    Eigen::Isometry3d left_model_end_to_tool_tcp,
    Eigen::Isometry3d right_model_end_to_tool_tcp)
: node_(node),
  miracle_algo_(miracle_algo),
  dual_arm_module_(dual_arm_module),
  lower_body_module_(lower_body_module),
  planning_frame_(std::move(planning_frame)),
  left_model_end_to_tool_tcp_(left_model_end_to_tool_tcp),
  right_model_end_to_tool_tcp_(right_model_end_to_tool_tcp)
{
}

WholeBodyModule::~WholeBodyModule()
{
    if (execution_thread_.joinable()) {
        execution_thread_.join();
    }
}

void WholeBodyModule::start_execution_move_pose(
    std::shared_ptr<GoalHandleMovePose> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    const bool need_left = goal && (goal->arm_id == WHOLE_BODY_LEFT ||
        goal->arm_id == WHOLE_BODY_BOTH);
    const bool need_right = goal && (goal->arm_id == WHOLE_BODY_RIGHT ||
        goal->arm_id == WHOLE_BODY_BOTH);
    if (!goal || (!need_left && !need_right)) {
        auto result = std::make_shared<MovePose::Result>();
        result->success = false;
        result->error_message = "unsupported whole-body arm_id";
        goal_handle->abort(result);
        return;
    }

    bool expected = false;
    if (!is_executing_.compare_exchange_strong(expected, true)) {
        auto result = std::make_shared<MovePose::Result>();
        result->success = false;
        result->error_message = "whole-body module is already executing";
        goal_handle->abort(result);
        return;
    }

    std::string resource_error;
    if (!dual_arm_module_->reserve_preplanned_trajectory_resources(
            need_left, need_right, resource_error)) {
        is_executing_.store(false);
        auto result = std::make_shared<MovePose::Result>();
        result->success = false;
        result->error_message = "whole-body arm resource reservation failed: " +
            resource_error;
        goal_handle->abort(result);
        return;
    }
    if (!lower_body_module_->reserve_preplanned_trajectory_resource(resource_error)) {
        dual_arm_module_->release_preplanned_trajectory_resources(
            need_left, need_right);
        is_executing_.store(false);
        auto result = std::make_shared<MovePose::Result>();
        result->success = false;
        result->error_message = "whole-body leg resource reservation failed: " +
            resource_error;
        goal_handle->abort(result);
        return;
    }

    if (execution_thread_.joinable()) {
        execution_thread_.join();
    }
    try {
        execution_thread_ = std::thread(
            &WholeBodyModule::execution_loop, this, goal_handle);
    } catch (const std::exception& error) {
        dual_arm_module_->release_preplanned_trajectory_resources(
            need_left, need_right);
        lower_body_module_->release_preplanned_trajectory_resource();
        is_executing_.store(false);
        auto result = std::make_shared<MovePose::Result>();
        result->success = false;
        result->error_message =
            std::string("failed to create whole-body worker: ") + error.what();
        goal_handle->abort(result);
    }
}

bool WholeBodyModule::normalize_target(
    const geometry_msgs::msg::PoseStamped& input,
    const std::vector<double>& full_q,
    geometry_msgs::msg::PoseStamped& output,
    std::string& error_message) const
{
    const auto normalized =
        motion_control_pkg::kinematics::validateAndNormalizePoseStamped(input);
    if (!normalized.success) {
        error_message = normalized.error_message;
        return false;
    }
    if (normalized.pose.header.frame_id == planning_frame_) {
        output = normalized.pose;
        output.header.frame_id = planning_frame_;
        return true;
    }
    Eigen::Isometry3d planning_to_source;
    if (!miracle_algo_->getFramePoseAtConfiguration(
        planning_frame_, normalized.pose.header.frame_id, full_q,
        planning_to_source))
    {
        error_message = "failed to transform whole-body target frame into planning frame";
        return false;
    }
    const Eigen::Isometry3d source_to_target =
        motion_control_pkg::kinematics::poseToIsometry(normalized.pose.pose);
    output = normalized.pose;
    output.header.frame_id = planning_frame_;
    output.pose = motion_control_pkg::kinematics::isometryToPose(
        planning_to_source * source_to_target);
    const auto output_validation =
        motion_control_pkg::kinematics::validateAndNormalizePoseStamped(output);
    if (!output_validation.success) {
        error_message = output_validation.error_message;
        return false;
    }
    return true;
}

bool WholeBodyModule::plan_trajectory(
    const std::shared_ptr<const MovePose::Goal>& goal,
    const std::vector<double>& start_q,
    std::vector<std::vector<double>>& left_path,
    std::vector<std::vector<double>>& right_path,
    std::vector<std::vector<double>>& leg_path,
    std::string& error_message) const
{
    left_path.clear();
    right_path.clear();
    leg_path.clear();

    const bool use_left = goal->arm_id == WHOLE_BODY_LEFT ||
        goal->arm_id == WHOLE_BODY_BOTH;
    const bool use_right = goal->arm_id == WHOLE_BODY_RIGHT ||
        goal->arm_id == WHOLE_BODY_BOTH;
    geometry_msgs::msg::PoseStamped left_target;
    geometry_msgs::msg::PoseStamped right_target;
    if (use_left && !normalize_target(goal->arm_id == WHOLE_BODY_BOTH ?
        goal->left_target_pose : goal->target_pose, start_q, left_target,
        error_message)) {
        return false;
    }
    if (use_right && !normalize_target(goal->arm_id == WHOLE_BODY_BOTH ?
        goal->right_target_pose : goal->target_pose, start_q, right_target,
        error_message)) {
        return false;
    }
    if (use_left) {
        if (goal->arm_id == WHOLE_BODY_BOTH) {
            addBoundOffset(left_target.pose, goal->left_bound_size);
        } else {
            addBoundOffset(left_target.pose, goal->bound_size);
        }
    }
    if (use_right) {
        if (goal->arm_id == WHOLE_BODY_BOTH) {
            addBoundOffset(right_target.pose, goal->right_bound_size);
        } else {
            addBoundOffset(right_target.pose, goal->bound_size);
        }
    }

    const auto target_to_root = [&](const geometry_msgs::msg::PoseStamped& target,
                                    bool is_left,
                                    Eigen::Isometry3d& root_to_end) {
        const Eigen::Isometry3d planning_to_end =
            motion_control_pkg::kinematics::targetTcpToModelEndPose(
            target.pose, is_left ? left_model_end_to_tool_tcp_ :
                                  right_model_end_to_tool_tcp_);
        return miracle_algo_->transformPoseToModelRoot(
            planning_frame_, start_q, planning_to_end, root_to_end);
    };

    std::optional<Eigen::Isometry3d> left_goal;
    std::optional<Eigen::Isometry3d> right_goal;
    Eigen::Isometry3d left_root_goal = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d right_root_goal = Eigen::Isometry3d::Identity();
    if (use_left) {
        if (!target_to_root(left_target, true, left_root_goal)) {
            error_message = "failed to transform left whole-body target to model root";
            return false;
        }
        left_goal = left_root_goal;
    }
    if (use_right) {
        if (!target_to_root(right_target, false, right_root_goal)) {
            error_message = "failed to transform right whole-body target to model root";
            return false;
        }
        right_goal = right_root_goal;
    }

    Eigen::Isometry3d left_start = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d right_start = Eigen::Isometry3d::Identity();
    if (use_left && !miracle_algo_->getFramePoseAtConfiguration(
        "base_link", "gripper_base_l_link", start_q, left_start)) {
        error_message = "failed to obtain left arm start pose";
        return false;
    }
    if (use_right && !miracle_algo_->getFramePoseAtConfiguration(
        "base_link", "gripper_base_r_link", start_q, right_start)) {
        error_message = "failed to obtain right arm start pose";
        return false;
    }

    const double speed = std::clamp(static_cast<double>(goal->speed), 0.01, 1.0);
    std::vector<std::vector<double>> full_waypoints;

    if (goal->enable_move_line) {
        // One scalar Ruckig trajectory provides a common progress parameter for
        // both arms.  The same alpha is used for both Cartesian poses and the
        // two goals are solved in one WLS call, so the legs are shared.
        double max_distance = 0.0;
        if (use_left) {
            max_distance = std::max(
                max_distance, (left_root_goal.translation() - left_start.translation()).norm());
        }
        if (use_right) {
            max_distance = std::max(
                max_distance, (right_root_goal.translation() - right_start.translation()).norm());
        }
        std::vector<double> scalars;
        if (!buildCartesianLineScalars(
                *miracle_algo_, max_distance, speed, scalars, error_message)) {
            return false;
        }
        Eigen::VectorXd q_current = miracle_algo_->toEigenConfig(start_q);
        for (std::size_t sample_index = 0; sample_index < scalars.size(); ++sample_index) {
            const double alpha = scalars[sample_index];
            if (sample_index == 0U && alpha <= 1e-8) {
                full_waypoints.push_back(start_q);
                continue;
            }
            const auto left_step = use_left ?
                std::optional<Eigen::Isometry3d>(interpolatePose(
                    left_start, left_root_goal, alpha)) : std::nullopt;
            const auto right_step = use_right ?
                std::optional<Eigen::Isometry3d>(interpolatePose(
                    right_start, right_root_goal, alpha)) : std::nullopt;
            Eigen::VectorXd q_next;
            if (!miracle_algo_->solveWholeBodyIK(
                q_current, left_step, right_step, q_next)) {
                error_message = "whole-body weighted IK failed at Cartesian line waypoint " +
                    std::to_string(sample_index) + "/" +
                    std::to_string(scalars.size() - 1U);
                return false;
            }
            if (goal->enable_collision_check && !miracle_algo_->isFullQValid(q_next)) {
                error_message = "whole-body Cartesian line waypoint violates collision constraints";
                return false;
            }
            full_waypoints.emplace_back(q_next.data(), q_next.data() + q_next.size());
            q_current = std::move(q_next);
        }
        if (full_waypoints.size() < 2U) {
            error_message = "whole-body Cartesian line planner returned fewer than two waypoints";
            return false;
        }
    } else {
        // The target is solved once with both arms present.  This is important:
        // two independent IK calls could produce different leg configurations.
        Eigen::VectorXd target_eigen;
        if (!miracle_algo_->solveWholeBodyIK(
                miracle_algo_->toEigenConfig(start_q), left_goal, right_goal,
                target_eigen)) {
            error_message = "failed to solve whole-body IK for target pose";
            return false;
        }
        const std::vector<double> target_q(
            target_eigen.data(), target_eigen.data() + target_eigen.size());

        if (goal->enable_collision_check) {
            if (!miracle_algo_->planWholeBodyJointSpace(
                    start_q, target_q, use_left, use_right, full_waypoints)) {
                error_message = "whole-body OMPL collision planning failed";
                return false;
            }
        } else {
            full_waypoints = {start_q, target_q};
        }
    }

    if (goal->enable_move_line) {
        std::vector<std::vector<double>> active_trajectory;
        for (const auto& full_q : full_waypoints) {
            std::vector<double> active_q;
            if (!makeWholeBodyActiveWaypoint(
                    *miracle_algo_, full_q, use_left, use_right, active_q)) {
                error_message = "failed to extract whole-body Cartesian line waypoint";
                return false;
            }
            active_trajectory.push_back(std::move(active_q));
        }
        if (!splitWholeBodyActiveTrajectory(
                active_trajectory, use_left, use_right,
                left_path, right_path, leg_path, error_message)) {
            return false;
        }
        error_message.clear();
        return true;
    }

    std::vector<std::vector<double>> active_waypoints;
    active_waypoints.reserve(full_waypoints.size());
    for (const auto& full_q : full_waypoints) {
        std::vector<double> active_q;
        if (!makeWholeBodyActiveWaypoint(
                *miracle_algo_, full_q, use_left, use_right, active_q)) {
            error_message = "failed to extract whole-body planning waypoint";
            return false;
        }
        active_waypoints.push_back(std::move(active_q));
    }

    const double arm_velocity = std::max(1e-3, speed * CARTES_V_MAX);
    const double arm_acceleration = std::max(1e-3, speed * CARTES_A_MAX);
    const double arm_jerk = std::max(1e-3, speed * CARTES_J_MAX);
    const double leg_velocity = std::max(1e-3, 1.5 * speed);
    const double leg_acceleration = std::max(1e-3, 5.0 * speed);
    const double leg_jerk = std::max(1e-3, 10.0 * speed);
    std::vector<double> velocity_limits;
    std::vector<double> acceleration_limits;
    std::vector<double> jerk_limits;
    const auto append_limits = [&](std::size_t count, double velocity,
                                   double acceleration, double jerk) {
        velocity_limits.insert(velocity_limits.end(), count, velocity);
        acceleration_limits.insert(acceleration_limits.end(), count, acceleration);
        jerk_limits.insert(jerk_limits.end(), count, jerk);
    };
    const auto append_arm_limits = [&](bool is_left) {
        std::vector<double> velocity;
        std::vector<double> acceleration;
        std::vector<double> jerk;
        if (dual_arm_module_ && dual_arm_module_->get_joint_motion_limits(
                is_left, velocity, acceleration, jerk)) {
            for (std::size_t index = 0; index < velocity.size(); ++index) {
                velocity_limits.push_back(std::max(1e-3, speed * velocity[index]));
                acceleration_limits.push_back(std::max(1e-3, speed * acceleration[index]));
                jerk_limits.push_back(std::max(1e-3, speed * jerk[index]));
            }
        } else {
            append_limits(7U, arm_velocity, arm_acceleration, arm_jerk);
        }
    };
    if (use_left) {
        append_arm_limits(true);
    }
    if (use_right) {
        append_arm_limits(false);
    }
    append_limits(4U, leg_velocity, leg_acceleration, leg_jerk);

    std::vector<std::vector<double>> active_trajectory;
    if (active_waypoints.size() == 2U) {
        if (!planRuckigSamplesWholeBody(
                *miracle_algo_, active_waypoints.front(), active_waypoints.back(),
                velocity_limits, acceleration_limits, jerk_limits,
                active_trajectory, error_message)) {
            return false;
        }
    } else {
        if (!planToppraSamplesWholeBody(
                *miracle_algo_, active_waypoints, velocity_limits,
                acceleration_limits, active_trajectory, error_message)) {
            return false;
        }
    }

    // TOPPRA may generate samples between OMPL waypoints.  Rebuild complete
    // configurations and validate them when collision checking was requested.
    if (goal->enable_collision_check) {
        for (const auto& active_q : active_trajectory) {
            std::vector<double> full_q;
            if (!miracle_algo_->mergeWholeBodyJoints(
                    start_q, use_left, use_right, active_q, full_q) ||
                !miracle_algo_->isFullQValid(miracle_algo_->toEigenConfig(full_q))) {
                error_message = "time-parameterized whole-body path violates collision constraints";
                return false;
            }
        }
    }
    if (!splitWholeBodyActiveTrajectory(
            active_trajectory, use_left, use_right,
            left_path, right_path, leg_path, error_message)) {
        return false;
    }
    error_message.clear();
    return true;
}

void WholeBodyModule::execution_loop(
    std::shared_ptr<GoalHandleMovePose> goal_handle)
{
    motion_control_pkg::execution::ScopedBusyFlagReset busy_reset(
        &is_executing_);
    motion_control_pkg::execution::ActionTerminalGuard<
        GoalHandleMovePose, MovePose::Result> terminal(goal_handle);
    const auto started_at = std::chrono::steady_clock::now();
    const auto goal = goal_handle->get_goal();
    const bool need_left = goal->arm_id == WHOLE_BODY_LEFT ||
        goal->arm_id == WHOLE_BODY_BOTH;
    const bool need_right = goal->arm_id == WHOLE_BODY_RIGHT ||
        goal->arm_id == WHOLE_BODY_BOTH;
    const auto release_child_resources = [&]() {
        dual_arm_module_->release_preplanned_trajectory_resources(
            need_left, need_right);
        lower_body_module_->release_preplanned_trajectory_resource();
    };
    auto result = std::make_shared<MovePose::Result>();
    result->success = false;
    auto feedback = std::make_shared<MovePose::Feedback>();
    const auto publish_feedback = [&](float progress, const std::string& step) {
        feedback->task_status = "RUNNING";
        feedback->progress_rate = std::clamp(progress, 0.0F, 100.0F);
        feedback->current_step_info = step;
        if (auto node = node_.lock()) {
            feedback->header.stamp = node->get_clock()->now();
        }
        goal_handle->publish_feedback(feedback);
    };
    const auto finish_time = [&]() {
        result->execution_time = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started_at).count();
    };

    if (goal_handle->is_canceling()) {
        result->error_message = "whole-body MovePose canceled before planning";
        finish_time();
        terminal.cancel(result);
        release_child_resources();
        return;
    }
    std::vector<double> start_q;
    if (!miracle_algo_->getFullJointState(start_q)) {
        result->error_message = "failed to obtain whole-body start state";
        finish_time();
        terminal.abort(result);
        release_child_resources();
        return;
    }
    publish_feedback(5.0F, "whole-body weighted IK planning");
    std::vector<std::vector<double>> left_path;
    std::vector<std::vector<double>> right_path;
    std::vector<std::vector<double>> leg_path;
    std::string planning_error;
    if (!plan_trajectory(
        goal, start_q, left_path, right_path, leg_path, planning_error)) {
        result->error_message = planning_error;
        finish_time();
        terminal.abort(result);
        release_child_resources();
        return;
    }
    if (goal->timeout_ms > 0 &&
        std::chrono::steady_clock::now() - started_at >=
        std::chrono::milliseconds(goal->timeout_ms)) {
        result->error_message = "whole-body MovePose timed out during planning";
        finish_time();
        terminal.abort(result);
        release_child_resources();
        return;
    }
    if (goal_handle->is_canceling()) {
        result->error_message = "whole-body MovePose canceled after planning";
        finish_time();
        terminal.cancel(result);
        release_child_resources();
        return;
    }

    publish_feedback(30.0F, "dispatching preplanned arm and leg trajectories");
    auto cancel = std::make_shared<motion_control_pkg::execution::CancellationToken>();
    const auto start_time = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(100);
    std::string arm_error;
    std::string leg_error;
    std::future<bool> arm_future;
    std::future<bool> leg_future;
    try {
        arm_future = std::async(
            std::launch::async,
            [this, &left_path, &right_path, goal, start_time, cancel, &arm_error]() {
                return dual_arm_module_->execute_preplanned_trajectory(
                    left_path, right_path, goal->control_mode, start_time,
                    cancel, arm_error);
            });
        leg_future = std::async(
            std::launch::async,
            [this, &leg_path, start_time, cancel, &leg_error]() {
                return lower_body_module_->execute_preplanned_trajectory(
                    leg_path, start_time, cancel, leg_error);
            });
    } catch (const std::exception& error) {
        cancel->request();
        if (arm_future.valid()) {
            arm_future.wait();
        }
        if (leg_future.valid()) {
            leg_future.wait();
        }
        result->error_message =
            std::string("failed to start whole-body child execution: ") +
            error.what();
        finish_time();
        terminal.abort(result);
        release_child_resources();
        return;
    }

    std::optional<bool> arm_ok;
    std::optional<bool> leg_ok;
    bool child_failed = false;
    bool timed_out = false;
    while (!arm_ok || !leg_ok)
    {
        if (!arm_ok && arm_future.wait_for(std::chrono::milliseconds(20)) ==
            std::future_status::ready) {
            arm_ok = arm_future.get();
            if (!*arm_ok) {
                child_failed = true;
                cancel->request();
            }
        }
        if (!leg_ok && leg_future.wait_for(std::chrono::milliseconds(0)) ==
            std::future_status::ready) {
            leg_ok = leg_future.get();
            if (!*leg_ok) {
                child_failed = true;
                cancel->request();
            }
        }
        if (goal_handle->is_canceling()) {
            cancel->request();
        }
        if (goal->timeout_ms > 0 &&
            std::chrono::steady_clock::now() - started_at >=
            std::chrono::milliseconds(goal->timeout_ms))
        {
            timed_out = true;
            cancel->request();
            result->error_message = "whole-body MovePose timed out";
        }
        publish_feedback(30.0F, "executing synchronized whole-body trajectory");
    }
    finish_time();

    if (goal_handle->is_canceling()) {
        if (result->error_message.empty()) {
            result->error_message = "whole-body MovePose canceled";
        }
        terminal.cancel(result);
    } else if (child_failed || timed_out || !*arm_ok || !*leg_ok) {
        if (result->error_message.empty()) {
            result->error_message = !*arm_ok ? arm_error : leg_error;
        }
        terminal.abort(result);
    } else {
        result->success = true;
        result->error_message.clear();
        result->final_distance_error = 0.0;
        publish_feedback(100.0F, "whole-body MovePose complete");
        terminal.succeed(result);
    }
    release_child_resources();
}
