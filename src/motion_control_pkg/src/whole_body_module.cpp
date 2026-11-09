#include "motion_control_pkg/execution/latest_progress.hpp"
#include "motion_control_pkg/execution/precision_parameters.hpp"
#include "motion_control_pkg/motion/scoped_collision_scene.hpp"
#include "motion_control_pkg/motion/tcp_level_plan.hpp"
#include "com_interfaces/tcp_level_goal.hpp"
#include "com_interfaces/rigid_grasp_goal.hpp"
#include "com_interfaces/precision_arrival_goal.hpp"
#include "whole_body_module.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <optional>

#include "motion_control_pkg/execution/control_period.hpp"
#include "motion_control_pkg/motion/cartesian_line_sampling.hpp"
#include "motion_control_pkg/motion/trajectory_derivative_limits.hpp"
#include "motion_control_pkg/kinematics/transform_utils.hpp"

namespace
{
motion_control_pkg::motion::RigidGraspOptions graspOptions(const MovePose::Goal& goal) {
    return {goal.keep_grasp_rigid,goal.grasp_position_tolerance_m,goal.grasp_orientation_tolerance_rad};
}
Eigen::Isometry3d graspFrame(const Eigen::Isometry3d& left,const Eigen::Isometry3d& right) {
    Eigen::Isometry3d frame=left;
    frame.translation()=.5*(left.translation()+right.translation());return frame;
}

class HardwareWholeBodyBackend final : public motion_control_pkg::execution::WholeBodyExecutionBackend {
public:
    HardwareWholeBodyBackend(std::shared_ptr<DualArmModule> arms,
                             std::shared_ptr<miracle::MiracleLowerBodyModule> legs)
      : arms_(std::move(arms)),legs_(std::move(legs)) {}
    bool reserve_arms(bool l,bool r,std::string& e) override {return arms_->reserve_preplanned_trajectory_resources(l,r,e);}
    bool reserve_legs(std::string& e) override {return legs_->reserve_preplanned_trajectory_resource(e);}
    void release_arms(bool l,bool r) noexcept override {arms_->release_preplanned_trajectory_resources(l,r);}
    void release_legs() noexcept override {legs_->release_preplanned_trajectory_resource();}
    bool arm_limits(bool l,std::vector<double>& v,std::vector<double>& a,std::vector<double>& j) const override {
        return arms_->get_joint_motion_limits(l,v,a,j);
    }
    bool prepare_dispatch(const Path& l,const Path& r,const Path& p,std::uint8_t mode,std::string& e) override {
        return arms_->prepare_preplanned_dispatch(l,r,mode,e) && legs_->prepare_preplanned_dispatch(p,e);
    }
    bool execute_arms(const Path&,const Path&,std::uint8_t,Time start,const Cancel& cancel,std::string& e) override {
        return arms_->dispatch_preplanned(start,cancel,e);
    }
    bool execute_legs(const Path& p,Time start,const Cancel& cancel,std::string& e) override {
        return legs_->dispatch_preplanned(p,start,cancel,e);
    }
    bool read_completion_feedback(motion_control_pkg::execution::WholeBodyCompletionFeedback& state,
                                  std::string& error) override {
        return arms_->read_preplanned_completion_feedback(state.arms,error) &&
            legs_->read_preplanned_completion_feedback(state.leg_positions,state.leg_velocities,
                                                        state.leg_sequence,error);
    }
private:
    std::shared_ptr<DualArmModule> arms_;
    std::shared_ptr<miracle::MiracleLowerBodyModule> legs_;
};

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
    const std::vector<double>& jerk_limits,
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
            velocity_limits, acceleration_limits, trajectory,
            MiracleMAlgo::ToppraSolverMode::kStableCartesian,
            std::numeric_limits<double>::infinity(), jerk_limits)) {
        error_message = "whole-body TOPPRA planning failed";
        return false;
    }
    error_message.clear();
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
    Eigen::Isometry3d right_model_end_to_tool_tcp,
    std::shared_ptr<motion_control_pkg::execution::WholeBodyExecutionBackend> backend)
: node_(node),
  miracle_algo_(miracle_algo),
  dual_arm_module_(dual_arm_module),
  lower_body_module_(lower_body_module),
  backend_(std::move(backend)),
  planning_frame_(std::move(planning_frame)),
  left_model_end_to_tool_tcp_(left_model_end_to_tool_tcp),
  right_model_end_to_tool_tcp_(right_model_end_to_tool_tcp)
{
    if (backend_ && (dual_arm_module_ || lower_body_module_))
        throw std::invalid_argument("injected WBC backend cannot be mixed with hardware modules");
    if (!backend_ && dual_arm_module_ && lower_body_module_)
        backend_=std::make_shared<HardwareWholeBodyBackend>(dual_arm_module_,lower_body_module_);
    leg_precision_policy_ = motion_control_pkg::execution::precisionParameters(*node, "leg_precision");
    rcl_interfaces::msg::ParameterDescriptor fixed; fixed.read_only = true;
    const bool enabled = node->declare_parameter<bool>("wbc_static_com_enabled", false, fixed);
    dynamic_zmp_enabled_ = node->declare_parameter<bool>("wbc_dynamic_zmp_enabled", false, fixed);
    if (dynamic_zmp_enabled_ && !enabled)
        throw std::invalid_argument("dynamic ZMP requires static COM gating");
    const bool assumptions = node->declare_parameter<bool>("wbc_stationary_horizontal_contact_confirmed", false, fixed);
    if (enabled) {
        if (!assumptions) throw std::invalid_argument("static COM requires stationary horizontal four-wheel-contact confirmation");
        motion_control_pkg::wbc::StaticBalanceOptions options;
        options.support_links = node->declare_parameter<std::vector<std::string>>("wbc_support_links",
            {"base_front_wheel_1_pitch_link", "base_rear_wheel_1_pitch_link", "base_front_wheel_2_link", "base_rear_wheel_2_link"}, fixed);
        options.unknown_steering = node->declare_parameter<std::vector<std::string>>("wbc_unknown_steering_joints",
            {"base_front_wheel_1_yaw_joint", "base_rear_wheel_1_yaw_joint"}, fixed);
        options.inset = node->declare_parameter<double>("wbc_support_inset_m", .02, fixed);
        options.steering_uncertainty_margin_enabled = node->declare_parameter<bool>(
            "wbc_steering_uncertainty_margin_enabled", true, fixed);
        options.com_weight = node->declare_parameter<double>("wbc_com_weight", .01, fixed);
        miracle_algo_->configureStaticBalance(options);
        auto policy = miracle_algo_->staticBalance()->statePolicy();
        // Dynamic prediction cannot use the Stage-2 assumed steering state.
        // Keep the conservative region, but require both actual steering angles.
        if (dynamic_zmp_enabled_) {
            policy.assumed_positions.clear();
            reference_options_.maximum_duration = node->declare_parameter<double>("wbc_reference_max_duration_sec", 60.0, fixed);
            reference_options_.maximum_zmp_scale = node->declare_parameter<double>("wbc_reference_max_zmp_scale", 10.0, fixed);
            if (!std::isfinite(reference_options_.maximum_duration) || reference_options_.maximum_duration<=0 ||
                !std::isfinite(reference_options_.maximum_zmp_scale) || reference_options_.maximum_zmp_scale<1)
                throw std::invalid_argument("invalid WBC dynamic reference budget");
        }
        policy.differentiate_joints = node->declare_parameter<std::vector<std::string>>("wbc_differentiate_joints",
            {"leg_joint1", "leg_joint2", "leg_joint3", "body_joint"}, fixed);
        balance_state_ = std::make_unique<motion_control_pkg::wbc::WholeBodyState>(miracle_algo_->staticBalance()->model().joints(), policy);
        balance_age_ = node->declare_parameter<double>("wbc_max_state_age_sec", .15, fixed);
        balance_skew_ = node->declare_parameter<double>("wbc_max_state_skew_sec", .06, fixed);
        balance_state_->snapshot(0, balance_age_, balance_skew_);
        const auto topic = node->declare_parameter<std::string>("wbc_joint_state_topic", "/wbc/joint_states", fixed);
        rclcpp::SubscriptionOptions feedback_options;
        if (dual_arm_module_)
            feedback_options.callback_group = dual_arm_module_->measuredFeedbackCallbackGroup();
        else
            feedback_options.callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        balance_subscription_ = node->create_subscription<sensor_msgs::msg::JointState>(topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
                auto node = node_.lock(); if (!node) return;
                std::string error;
                const double stamp = rclcpp::Time(msg->header.stamp).seconds();
                if (stamp > node->now().seconds() || !balance_state_->update(msg->name, msg->position, msg->velocity, stamp, error))
                    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000, "WBC feedback rejected: %s", error.c_str());
            }, feedback_options);
    }
}

bool WholeBodyModule::read_balance_state(std::vector<double>& q, std::string& error) const {
    if (!balance_state_) return miracle_algo_->getFullJointState(q);
    const auto node = node_.lock();
    if (!node) { error = "WBC node unavailable"; return false; }
    const auto state = balance_state_->snapshotNow([&]{return node->now().seconds();}, balance_age_, balance_skew_);
    if (dynamic_zmp_enabled_ && (!state.velocity_valid || !state.assumed.empty())) {
        error = "dynamic WBC requires complete fresh measured q/v including steering";
        for (const auto& name:state.missing) error += "; missing="+name;
        for (const auto& name:state.stale) error += "; stale="+name;
        if (state.missing.empty() && state.stale.empty()) error += "; velocity estimate invalid or assumed state present";
        return false;
    }
    if (!state.position_valid) {
        error = "WBC feedback incomplete, stale or time-skewed";
        for (const auto& name : state.missing) error += "; missing=" + name;
        for (const auto& name : state.stale) error += "; stale=" + name;
        error += "; oldest_age_sec=" + std::to_string(node->now().seconds()-state.oldest_stamp);
        error += "; skew_sec=" + std::to_string(state.newest_stamp-state.oldest_stamp);
        return false;
    }
    if (!miracle_algo_->balanceStateToFullQ(state, q)) {
        error = "WBC feedback-to-model joint mapping failed";
        return false;
    }
    if (!miracle_algo_->isStaticBalanceValid(miracle_algo_->toEigenConfig(q))) {
        error = "measured state violates configured static COM constraint"; return false;
    }
    return true;
}

bool WholeBodyModule::check_balance_feedback(const std::vector<double>& start, bool use_left,
    bool use_right, bool before_dispatch, std::string& error) const {
    if (!balance_state_) return true;
    std::vector<double> current, expected, active;
    if (!read_balance_state(current, error)) return false;
    if (dynamic_zmp_enabled_ && before_dispatch) {
        const auto node = node_.lock();
        if (!node) {error = "WBC node unavailable"; return false;}
        const auto state = balance_state_->snapshotNow([&]{return node->now().seconds();}, balance_age_, balance_skew_);
        if (!state.velocity_valid || std::any_of(state.velocities.begin(),state.velocities.end(),
            [](double v){return !std::isfinite(v) || std::abs(v)>.01;})) {
            error = "dynamic WBC requires resting joints before dispatch"; return false;
        }
    }
    expected = start;
    if (!before_dispatch && (!makeWholeBodyActiveWaypoint(*miracle_algo_, current, use_left, use_right, active) ||
        !miracle_algo_->mergeWholeBodyJoints(start, use_left, use_right, active, expected))) {
        error = "cannot validate frozen WBC joints"; return false;
    }
    for (std::size_t i = 0; i < current.size(); ++i)
        if (std::abs(current[i]-expected[i]) > .01) {
            error = before_dispatch ? "WBC start state changed during planning" : "uncommanded WBC joint moved";
            return false;
        }
    return true;
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
    if (!backend_) {
        auto result = std::make_shared<MovePose::Result>();
        result->success = false;
        result->error_message = "whole-body execution backends are not attached (planning-only instance)";
        goal_handle->abort(result);
        return;
    }
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
    if (!backend_->reserve_arms(
            need_left, need_right, resource_error)) {
        is_executing_.store(false);
        auto result = std::make_shared<MovePose::Result>();
        result->success = false;
        result->error_message = "whole-body arm resource reservation failed: " +
            resource_error;
        goal_handle->abort(result);
        return;
    }
    if (!backend_->reserve_legs(resource_error)) {
        backend_->release_arms(
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
        execution_thread_ = std::thread([this,goal_handle,need_left,need_right] {
            try {execution_loop(goal_handle);}
            catch (const std::exception& e) {
                backend_->release_arms(need_left,need_right); backend_->release_legs();
                is_executing_.store(false);
                if (goal_handle->is_active()) {
                    auto result=std::make_shared<MovePose::Result>(); result->success=false;
                    result->error_message=std::string("whole-body worker exception: ")+e.what();
                    try {goal_handle->abort(result);} catch (const std::exception&) {}
                }
            }
        });
    } catch (const std::exception& error) {
        backend_->release_arms(
            need_left, need_right);
        backend_->release_legs();
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
    RCLCPP_INFO(
        rclcpp::get_logger("whole_body_module"),
        "normalize_target: before frame=[%s] pos=[%.6f,%.6f,%.6f] quat=[%.6f,%.6f,%.6f,%.6f]; "
        "after frame=[%s] pos=[%.6f,%.6f,%.6f] quat=[%.6f,%.6f,%.6f,%.6f]",
        normalized.pose.header.frame_id.c_str(),
        normalized.pose.pose.position.x, normalized.pose.pose.position.y,
        normalized.pose.pose.position.z,
        normalized.pose.pose.orientation.x, normalized.pose.pose.orientation.y,
        normalized.pose.pose.orientation.z, normalized.pose.pose.orientation.w,
        output.header.frame_id.c_str(),
        output.pose.position.x, output.pose.position.y, output.pose.position.z,
        output.pose.orientation.x, output.pose.orientation.y,
        output.pose.orientation.z, output.pose.orientation.w);
    
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
    std::string& error_message,
    std::optional<Eigen::Isometry3d>& left_target_tcp,
    std::optional<Eigen::Isometry3d>& right_target_tcp,
    motion_control_pkg::wbc::ReferenceResult* reference) const
{
    left_path.clear();
    right_path.clear();
    leg_path.clear();
    if (reference) *reference = {};
    if (!goal || (goal->arm_id!=WHOLE_BODY_LEFT && goal->arm_id!=WHOLE_BODY_RIGHT && goal->arm_id!=WHOLE_BODY_BOTH)) {
        error_message = "unsupported whole-body planning goal"; return false;
    }
    if (!com_interfaces::validatePrecisionArrivalGoal(goal->precision_arrival,
            goal->arm_id, true, goal->control_mode, error_message)) return false;
    if (!com_interfaces::validateTcpLevelGoal(goal->keep_tcp_level, goal->tcp_level_tolerance_rad,
            goal->arm_id, true, goal->control_mode, error_message)) return false;
    if (!com_interfaces::validateRigidGraspGoal(*goal,error_message)) return false;
    if(goal->keep_grasp_rigid && dynamic_zmp_enabled_) {
        error_message="rigid grasp currently requires fixed-base kinematic WBC (dynamic ZMP disabled)";return false;
    }
    if (goal->whole_body_waist_only && goal->enable_move_line) {
        error_message = "whole_body_waist_only requires MoveP (enable_move_line=false)";
        return false;
    }
    auto collision_scene = motion_control_pkg::motion::makeCollisionScene(
        *miracle_algo_, goal->enable_collision_check, start_q,
        goal->env_point_cloud, error_message);
    if (!collision_scene) return false;

    const bool use_left = goal->arm_id == WHOLE_BODY_LEFT ||
        goal->arm_id == WHOLE_BODY_BOTH;
    const bool use_right = goal->arm_id == WHOLE_BODY_RIGHT ||
        goal->arm_id == WHOLE_BODY_BOTH;
    motion_control_pkg::motion::TcpLevelPlan level(goal->keep_tcp_level,
        miracle_algo_->getModel(), start_q, use_left, use_right, true,
        left_model_end_to_tool_tcp_, right_model_end_to_tool_tcp_, goal->tcp_level_tolerance_rad,graspOptions(*goal));
    if (!level.initialize(error_message)) return false;
    std::vector<double> frozen_leg_q;
    if (goal->whole_body_waist_only &&
        (!miracle_algo_->extractLegJoints(start_q, frozen_leg_q) || frozen_leg_q.size() != 4U)) {
        error_message = "cannot obtain waist-only starting leg configuration";
        return false;
    }
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

    // 记录规划系下的目标 TCP 位姿，供执行结束后计算最终误差。
    if (use_left) {
        left_target_tcp = motion_control_pkg::kinematics::poseToIsometry(
            left_target.pose);
    }
    if (use_right) {
        right_target_tcp = motion_control_pkg::kinematics::poseToIsometry(
            right_target.pose);
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

    if ((use_left && !level.target(true, left_root_goal * left_model_end_to_tool_tcp_, error_message)) ||
        (use_right && !level.target(false, right_root_goal * right_model_end_to_tool_tcp_, error_message))) return false;
    if(!level.targets(left_root_goal*left_model_end_to_tool_tcp_,
                      right_root_goal*right_model_end_to_tool_tcp_,error_message)) return false;
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

    if (!std::isfinite(goal->speed) || goal->speed <= 0 || goal->speed > 1 ||
        !std::isfinite(goal->acceleration_scale) || goal->acceleration_scale < 0 || goal->acceleration_scale > 1 ||
        !std::isfinite(goal->jerk_scale) || goal->jerk_scale < 0 || goal->jerk_scale > 1) {
        error_message = "invalid whole-body speed/acceleration/jerk scale"; return false;
    }
    const double speed = goal->speed;
    const double acceleration_scale = goal->acceleration_scale == 0 ? 1 : goal->acceleration_scale;
    const double jerk_scale = goal->jerk_scale == 0 ? 1 : goal->jerk_scale;
    std::vector<std::vector<double>> full_waypoints;
    const auto object_start=graspFrame(left_start*left_model_end_to_tool_tcp_,right_start*right_model_end_to_tool_tcp_);
    const auto object_goal=graspFrame(left_root_goal*left_model_end_to_tool_tcp_,right_root_goal*right_model_end_to_tool_tcp_);
    const Eigen::Isometry3d object_left=object_start.inverse()*left_start*left_model_end_to_tool_tcp_;
    const Eigen::Isometry3d object_right=object_start.inverse()*right_start*right_model_end_to_tool_tcp_;
    const auto rigid_steps=[&](double alpha) {
        const auto object=interpolatePose(object_start,object_goal,alpha);
        return std::array<Eigen::Isometry3d,2>{
            object*object_left*left_model_end_to_tool_tcp_.inverse(),
            object*object_right*right_model_end_to_tool_tcp_.inverse()};
    };

    if (goal->enable_move_line) {
        // Sample TCP geometry independently of time. Both arms use one path
        // parameter; orientation-only goals receive their own sampling budget.
        std::size_t segments = 1;
        for (bool left : {true, false}) {
            if ((left && !use_left) || (!left && !use_right)) continue;
            const auto& tool = left ? left_model_end_to_tool_tcp_ : right_model_end_to_tool_tcp_;
            const auto sampling = motion_control_pkg::motion::computeCartesianLineSegmentCount(
                (left ? left_start : right_start) * tool,
                (left ? left_root_goal : right_root_goal) * tool,
                motion_control_pkg::motion::kCartesianIkTranslationStepMeters,
                motion_control_pkg::motion::kCartesianIkRotationStepRadians);
            if (!sampling.valid) { error_message = sampling.error_message; return false; }
            segments = std::max(segments, sampling.segments);
        }
        std::vector<double> scalars;
        for (std::size_t i = 0; i <= segments; ++i)
            scalars.push_back(static_cast<double>(i) / segments);
        Eigen::VectorXd q_current = miracle_algo_->toEigenConfig(start_q);
        for (std::size_t sample_index = 0; sample_index < scalars.size(); ++sample_index) {
            const double alpha = scalars[sample_index];
            if (sample_index == 0U && alpha <= 1e-8) {
                full_waypoints.push_back(start_q);
                continue;
            }
            auto left_step = use_left ?
                std::optional<Eigen::Isometry3d>(interpolatePose(
                    left_start * left_model_end_to_tool_tcp_, left_root_goal * left_model_end_to_tool_tcp_, alpha) *
                    left_model_end_to_tool_tcp_.inverse()) : std::nullopt;
            auto right_step = use_right ?
                std::optional<Eigen::Isometry3d>(interpolatePose(
                    right_start * right_model_end_to_tool_tcp_, right_root_goal * right_model_end_to_tool_tcp_, alpha) *
                    right_model_end_to_tool_tcp_.inverse()) : std::nullopt;
            if(goal->keep_grasp_rigid) {
                const auto coupled=rigid_steps(alpha);left_step=coupled[0];right_step=coupled[1];
            }
            Eigen::VectorXd q_next;
            if (!miracle_algo_->solveWholeBodyIK(
                q_current, left_step, right_step, q_next,
                std::array<double, 4>{1.0, 1.0, 1.0, 0.2},
                0.0, 10.0, 3U, false)) {
                error_message = "whole-body weighted IK failed at Cartesian line waypoint " +
                    std::to_string(sample_index) + "/" +
                    std::to_string(scalars.size() - 1U);
                return false;
            }
            full_waypoints.emplace_back(q_next.data(), q_next.data() + q_next.size());
            q_current = std::move(q_next);
        }
        if (full_waypoints.size() < 2U) {
            error_message = "whole-body Cartesian line planner returned fewer than two waypoints";
            return false;
        }
        if (goal->enable_collision_check) {
            for (std::size_t waypoint_index = 0;
                 waypoint_index < full_waypoints.size(); ++waypoint_index) {
                if (!miracle_algo_->isFullQValid(
                        miracle_algo_->toEigenConfig(full_waypoints[waypoint_index]))) {
                    error_message =
                        "whole-body Cartesian line trajectory violates collision constraints at waypoint " +
                        std::to_string(waypoint_index) + "/" +
                        std::to_string(full_waypoints.size() - 1U);
                    return false;
                }
            }
        }
    } else {
        // The target is solved once with both arms present.  This is important:
        // two independent IK calls could produce different leg configurations.
        Eigen::VectorXd target_eigen;
        if (use_left && use_right) {
            // A large simultaneous pose change can leave the local WLS solver
            // outside the convergence basin.  Move both Cartesian goals through
            // the same alpha values and feed each result into the next solve.
            // Collision validity is checked below on the resulting target/path;
            // keeping the IK solve free of the soft collision barrier avoids
            // rejecting every step when the current model is already near a
            // configured collision pair.
            const double translation_distance = std::max(
                (left_root_goal.translation() - left_start.translation()).norm(),
                (right_root_goal.translation() - right_start.translation()).norm());
            const double left_angle = Eigen::Quaterniond(left_start.rotation()).angularDistance(
                Eigen::Quaterniond(left_root_goal.rotation()));
            const double right_angle = Eigen::Quaterniond(right_start.rotation()).angularDistance(
                Eigen::Quaterniond(right_root_goal.rotation()));
            const double rotation_distance = std::max(left_angle, right_angle);
            const std::size_t translation_steps = static_cast<std::size_t>(std::ceil(
                translation_distance / 0.08));
            const std::size_t rotation_steps = static_cast<std::size_t>(std::ceil(
                rotation_distance / 0.35));
            const std::size_t steps = std::clamp<std::size_t>(
                std::max<std::size_t>({1U, translation_steps, rotation_steps}), 1U, 20U);

            Eigen::VectorXd q_current = miracle_algo_->toEigenConfig(start_q);
            for (std::size_t step = 1U; step <= steps; ++step) {
                const double alpha = static_cast<double>(step) /
                    static_cast<double>(steps);
                auto left_step = std::optional<Eigen::Isometry3d>(
                    interpolatePose(left_start, left_root_goal, alpha));
                auto right_step = std::optional<Eigen::Isometry3d>(
                    interpolatePose(right_start, right_root_goal, alpha));
                if(goal->keep_grasp_rigid) {
                    const auto coupled=rigid_steps(alpha);left_step=coupled[0];right_step=coupled[1];
                }
                Eigen::VectorXd q_next;
                if (!miracle_algo_->solveWholeBodyIK(
                        q_current, left_step, right_step, q_next,
                        std::array<double, 4>{1.0, 1.0, 1.0, 0.2},
                        0.0, 10.0, 5U, false, goal->whole_body_waist_only)) {
                    error_message =
                        "failed to solve dual-arm whole-body IK at Cartesian "
                        "substep " + std::to_string(step) + "/" +
                        std::to_string(steps);
                    return false;
                }
                q_current = std::move(q_next);
            }
            target_eigen = std::move(q_current);
            if (goal->enable_collision_check &&
                !miracle_algo_->isFullQValid(target_eigen)) {
                // The unconstrained staged solve can select an elbow branch
                // that reaches the Cartesian target through the torso.  Re-solve
                // only the final dual pose with the collision barrier and
                // multiple arm/leg seeds, while retaining one shared leg state.
                Eigen::VectorXd collision_free_target;
                if (!miracle_algo_->solveWholeBodyIK(
                        target_eigen, left_goal, right_goal,
                        collision_free_target,
                        std::array<double, 4>{1.0, 1.0, 1.0, 0.2},
                        0.0, 5.0, 15U, true, goal->whole_body_waist_only)) {
                    error_message =
                        "failed to find a collision-free dual-arm whole-body "
                        "IK solution for the target pose";
                    return false;
                }
                target_eigen = std::move(collision_free_target);
            }
        } else if (!miracle_algo_->solveWholeBodyIK(
                       miracle_algo_->toEigenConfig(start_q), left_goal, right_goal,
                       target_eigen, std::array<double, 4>{1.0, 1.0, 1.0, 0.2},
                       0.0, 10.0, 10U, true, goal->whole_body_waist_only)) {
            error_message = "failed to solve whole-body IK for target pose";
            return false;
        }
        const std::vector<double> target_q(
            target_eigen.data(), target_eigen.data() + target_eigen.size());

        if (goal->enable_collision_check) {
            if (!miracle_algo_->planWholeBodyJointSpace(
                    start_q, target_q, use_left, use_right, full_waypoints,
                    goal->whole_body_waist_only)) {
                error_message = "whole-body OMPL collision planning failed";
                return false;
            }
        } else {
            full_waypoints = {start_q, target_q};
        }
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

    // Keep the existing four-axis lower-body wire format: J1/J2/J3 hold the
    // measured start, J4 moves. Never repair a genuinely moving leg trajectory
    // by overwriting it after IK; only remove floating-point interpolation noise.
    if ((!goal->enable_move_line || goal->keep_grasp_rigid) && !level.project(active_waypoints, error_message)) return false;
    const auto enforce_frozen_legs = [&](std::vector<std::vector<double>>& path) {
        if (!goal->whole_body_waist_only) return true;
        const std::size_t offset = (use_left ? 7U : 0U) + (use_right ? 7U : 0U);
        for (auto& sample : path) {
            if (sample.size() != offset + 4U) {
                error_message = "invalid waist-only trajectory dimensions";
                return false;
            }
            for (std::size_t k = 0; k < 3U; ++k) {
                if (!std::isfinite(sample[offset + k]) ||
                    !std::isfinite(frozen_leg_q[k]) ||
                    std::abs(sample[offset + k] - frozen_leg_q[k]) > 1e-10) {
                    error_message = "waist-only trajectory changes leg_joint1/2/3";
                    return false;
                }
                sample[offset + k] = frozen_leg_q[k];
            }
        }
        return true;
    };
    if (!enforce_frozen_legs(active_waypoints)) return false;

    std::vector<double> velocity_limits, acceleration_limits, jerk_limits;
    const auto append_arm_limits = [&](bool is_left) {
        std::vector<double> velocity(7, CARTES_V_MAX), acceleration(7, CARTES_A_MAX), jerk(7, CARTES_J_MAX);
        if (backend_ && !backend_->arm_limits(is_left, velocity, acceleration, jerk)) {
            error_message = "cannot obtain whole-body arm motion limits"; return false;
        }
        if (velocity.size() != 7 || acceleration.size() != 7 || jerk.size() != 7) {
            error_message = "invalid whole-body arm motion limit dimensions"; return false;
        }
        for (std::size_t i = 0; i < 7; ++i) {
            if (!std::isfinite(velocity[i]) || velocity[i] <= 0 ||
                !std::isfinite(acceleration[i]) || acceleration[i] <= 0 ||
                !std::isfinite(jerk[i]) || jerk[i] <= 0) {
                error_message = "invalid whole-body arm motion limits"; return false;
            }
            velocity_limits.push_back(speed * velocity[i]);
            acceleration_limits.push_back(speed * acceleration_scale * acceleration[i]);
            jerk_limits.push_back(speed * jerk_scale * jerk[i]);
        }
        return true;
    };
    if ((use_left && !append_arm_limits(true)) || (use_right && !append_arm_limits(false))) return false;
    velocity_limits.insert(velocity_limits.end(), 4, 1.5 * speed);
    acceleration_limits.insert(acceleration_limits.end(), 4, 5.0 * speed * acceleration_scale);
    jerk_limits.insert(jerk_limits.end(), 4, 10.0 * speed * jerk_scale);
    // URDF velocity limits remain authoritative; acceleration and jerk are
    // planner policies because ordinary URDF does not define either quantity.
    {
        const auto& model = miracle_algo_->getModel();
        std::vector<double> full_limits(model.nq);
        for (std::size_t id = 1; id < model.joints.size(); ++id)
            full_limits[model.joints[id].idx_q()] = model.velocityLimit[model.joints[id].idx_v()];
        std::vector<double> active_limits;
        if (!makeWholeBodyActiveWaypoint(*miracle_algo_, full_limits, use_left, use_right, active_limits) ||
            active_limits.size() != velocity_limits.size()) {
            error_message = "cannot map URDF whole-body velocity limits"; return false;
        }
        for (std::size_t i = 0; i < active_limits.size(); ++i) {
            if (!std::isfinite(active_limits[i]) || !(active_limits[i] > 0)) { error_message = "active URDF joint has zero velocity limit"; return false; }
            velocity_limits[i] = std::min(velocity_limits[i], speed * active_limits[i]);
        }
    }

    std::vector<std::vector<double>> active_trajectory;
    if (active_waypoints.size() == 2U && !goal->enable_move_line) {
        if (!planRuckigSamplesWholeBody(
                *miracle_algo_, active_waypoints.front(), active_waypoints.back(),
                velocity_limits, acceleration_limits, jerk_limits,
                active_trajectory, error_message)) {
            return false;
        }
    } else if (std::all_of(active_waypoints.begin(), active_waypoints.end(),
                   [&](const auto& q) { return q == active_waypoints.front(); })) {
        active_trajectory = {active_waypoints.front(), active_waypoints.back()};
    } else {
        if (!planToppraSamplesWholeBody(
                *miracle_algo_, active_waypoints, velocity_limits,
                acceleration_limits, jerk_limits, active_trajectory, error_message)) {
            return false;
        }
    }

    if (!enforce_frozen_legs(active_trajectory)) return false;
    std::optional<motion_control_pkg::wbc::ReferenceResult> prepared_reference;
    if (dynamic_zmp_enabled_) {
        // Ruckig returns one point for a no-op goal. Use an explicit stationary
        // interval so it receives the same reference/state validation.
        if (active_trajectory.size()==1) active_trajectory.push_back(active_trajectory.front());
        namespace wbc = motion_control_pkg::wbc;
        const auto& balance = *miracle_algo_->staticBalance();
        const auto& legacy = miracle_algo_->getModel();
        auto state = balance.model().neutralSnapshot();
        std::vector<std::size_t> indices;
        for (const auto& spec : balance.model().joints()) {
            if (!legacy.existJointName(spec.name)) {error_message="WBC full model mapping failed"; return false;}
            indices.push_back(legacy.joints[legacy.getJointId(spec.name)].idx_q());
        }
        const auto ordered = [&](const std::vector<double>& q) {
            std::vector<double> out;
            for (auto i:indices) out.push_back(q.at(i));
            return out;
        };
        const auto legacy_q = [&](const std::vector<double>& q) {
            auto out=start_q;
            for (std::size_t j=0;j<indices.size();++j) out.at(indices[j])=q.at(j);
            return out;
        };
        state.positions=ordered(start_q); // Planned zero-v/a start; feedback rest is separately gated.
        std::vector<std::vector<double>> complete;
        for (const auto& active:active_trajectory) {
            std::vector<double> full;
            if (!miracle_algo_->mergeWholeBodyJoints(start_q,use_left,use_right,active,full)) {
                error_message="cannot rebuild dynamic WBC trajectory"; return false;
            }
            complete.push_back(ordered(full));
        }
        // Frozen coordinates still require positive mathematical retimer limits;
        // their URDF-locked/unchanged positions are checked independently.
        const std::vector<double> frozen_limits(start_q.size(),1);
        std::vector<double> full_v,full_a,full_j;
        if (!miracle_algo_->mergeWholeBodyJoints(frozen_limits,use_left,use_right,velocity_limits,full_v) ||
            !miracle_algo_->mergeWholeBodyJoints(frozen_limits,use_left,use_right,acceleration_limits,full_a) ||
            !miracle_algo_->mergeWholeBodyJoints(frozen_limits,use_left,use_right,jerk_limits,full_j)) {
            error_message="cannot map dynamic WBC derivative limits"; return false;
        }
        auto v=ordered(full_v), a=ordered(full_a), j=ordered(full_j);
        for (std::size_t k=0;k<v.size();++k) if (!balance.model().joints()[k].locked)
            v[k]=std::min(v[k],balance.model().joints()[k].velocity);
        const auto check_geometry = [&](const std::vector<double>& q, std::string& error) {
            const auto full=legacy_q(q);
            if (goal->enable_collision_check && !miracle_algo_->isFullQValid(miracle_algo_->toEigenConfig(full))) {
                error="resampled WBC reference violates collision constraints"; return false;
            }
            if (!goal->enable_move_line) return true;
            for (bool left:{true,false}) {
                if ((left && !use_left) || (!left && !use_right)) continue;
                Eigen::Isometry3d begin, actual;
                const auto link=left?"gripper_base_l_link":"gripper_base_r_link";
                if (!miracle_algo_->getFramePoseAtConfiguration("base_link",link,start_q,begin) ||
                    !miracle_algo_->getFramePoseAtConfiguration("base_link",link,full,actual)) {
                    error="cannot validate resampled MoveL FK"; return false;
                }
                const auto& tool=left?left_model_end_to_tool_tcp_:right_model_end_to_tool_tcp_;
                begin=begin*tool;
                actual=actual*tool;
                const Eigen::Isometry3d end=(left?left_root_goal:right_root_goal)*tool;
                const Eigen::Vector3d delta=end.translation()-begin.translation();
                double alpha=0;
                if (delta.squaredNorm()>1e-12)
                    alpha=std::clamp((actual.translation()-begin.translation()).dot(delta)/delta.squaredNorm(),0.0,1.0);
                else {
                    const double angle=Eigen::AngleAxisd(begin.rotation().transpose()*end.rotation()).angle();
                    if (angle>1e-8) alpha=std::clamp(Eigen::AngleAxisd(begin.rotation().transpose()*actual.rotation()).angle()/angle,0.0,1.0);
                }
                const auto expected=interpolatePose(begin,end,alpha);
                if ((actual.translation()-expected.translation()).norm()>.003 ||
                    Eigen::AngleAxisd(expected.rotation().transpose()*actual.rotation()).angle()>.02) {
                    error="resampled MoveL exceeds 3 mm / 0.02 rad line tolerance"; return false;
                }
            }
            return true;
        };
        auto prepared=wbc::prepareReference(balance,state,complete,v,a,j,reference_options_,check_geometry);
        if (!prepared.valid) {error_message="dynamic WBC: "+prepared.error; return false;}
        active_trajectory.clear();
        for (const auto& sample:prepared.samples) {
            std::vector<double> active;
            if (!makeWholeBodyActiveWaypoint(*miracle_algo_,legacy_q(sample.positions),use_left,use_right,active)) {
                error_message="cannot split prepared WBC reference"; return false;
            }
            active_trajectory.push_back(std::move(active));
        }
        RCLCPP_INFO(rclcpp::get_logger("whole_body_module"),
            "dynamic WBC reference: %.3f s, %zu samples, minimum predicted ZMP margin %.6f m",
            prepared.duration,prepared.samples.size(),prepared.minimum_zmp_margin);
        prepared_reference=std::move(prepared);
    }
    if (!enforce_frozen_legs(active_trajectory)) return false;

    if (!motion_control_pkg::motion::validateCommandDerivatives(active_trajectory,
            motion_control_pkg::execution::kDualArmControlPeriodSeconds,
            velocity_limits, acceleration_limits, jerk_limits, error_message)) return false;
    if (!level.validate(active_trajectory, error_message)) return false;
    std::vector<std::vector<double>> balance_path;
    for (const auto& active_q : active_trajectory) {
        std::vector<double> full_q;
        if (!miracle_algo_->mergeWholeBodyJoints(start_q, use_left, use_right, active_q, full_q) ||
            (goal->enable_collision_check && !miracle_algo_->isFullQValid(miracle_algo_->toEigenConfig(full_q)))) {
            error_message = "time-parameterized whole-body path violates collision or joint constraints";
            return false;
        }
        const auto& model = miracle_algo_->getModel();
        for (std::size_t j = 0; j < full_q.size(); ++j) {
            if (!std::isfinite(full_q[j]) || full_q[j] < model.lowerPositionLimit[j] - 1e-8 ||
                full_q[j] > model.upperPositionLimit[j] + 1e-8) {
                error_message = "retimed whole-body trajectory violates URDF position limits"; return false;
            }
        }
        if (goal->enable_move_line && goal->keep_grasp_rigid) {
            Eigen::Isometry3d actual_left,actual_right;
            if(!miracle_algo_->getFramePoseAtConfiguration("base_link","gripper_base_l_link",full_q,actual_left)||
               !miracle_algo_->getFramePoseAtConfiguration("base_link","gripper_base_r_link",full_q,actual_right)) {
                error_message="cannot validate rigid object MoveL FK";return false;
            }
            const auto actual=graspFrame(actual_left*left_model_end_to_tool_tcp_,actual_right*right_model_end_to_tool_tcp_);
            const auto deviation=motion_control_pkg::motion::evaluateCartesianLineSample(object_start,object_goal,actual);
            if(!deviation.valid||deviation.position_error_m>.003||deviation.orientation_error_rad>.02) {
                error_message="rigid object MoveL exceeds 3 mm / 0.02 rad object-frame line tolerance";return false;
            }
        } else if (goal->enable_move_line) {
            for (bool left : {true, false}) {
                if ((left && !use_left) || (!left && !use_right)) continue;
                const auto& tool = left ? left_model_end_to_tool_tcp_ : right_model_end_to_tool_tcp_;
                Eigen::Isometry3d actual;
                if (!miracle_algo_->getFramePoseAtConfiguration("base_link",
                        left ? "gripper_base_l_link" : "gripper_base_r_link", full_q, actual)) {
                    error_message = "cannot validate retimed whole-body MoveL FK"; return false;
                }
                const auto deviation = motion_control_pkg::motion::evaluateCartesianLineSample(
                    (left ? left_start : right_start) * tool,
                    (left ? left_root_goal : right_root_goal) * tool, actual * tool);
                // The evaluator clamps progress to the finite segment before
                // measuring error, so endpoint overshoot is already bounded in
                // metres/radians. A percentage bound spuriously rejects tiny moves.
                if (!deviation.valid || deviation.position_error_m > 0.003 ||
                    deviation.orientation_error_rad > 0.02) {
                    error_message = "retimed whole-body MoveL exceeds 3 mm / 0.02 rad TCP line tolerance: sample=" +
                        std::to_string(balance_path.size()) + ", position_m=" + std::to_string(deviation.position_error_m) +
                        ", rotation_rad=" + std::to_string(deviation.orientation_error_rad) +
                        ", progress=" + std::to_string(deviation.progress);
                    return false;
                }
            }
        }
        balance_path.push_back(std::move(full_q));
    }
    if (!miracle_algo_->validateStaticBalancePath(balance_path, error_message,
            motion_control_pkg::execution::kDualArmControlPeriodSeconds)) return false;
    if (!splitWholeBodyActiveTrajectory(
            active_trajectory, use_left, use_right,
            left_path, right_path, leg_path, error_message)) {
        return false;
    }
    error_message.clear();
    if (reference && prepared_reference) *reference=std::move(*prepared_reference);
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
    const auto action_precision = motion_control_pkg::execution::actionPrecisionPolicy(
        leg_precision_policy_, goal->precision_arrival);
    const bool need_left = goal->arm_id == WHOLE_BODY_LEFT ||
        goal->arm_id == WHOLE_BODY_BOTH;
    const bool need_right = goal->arm_id == WHOLE_BODY_RIGHT ||
        goal->arm_id == WHOLE_BODY_BOTH;
    const auto release_child_resources = [&]() {
        backend_->release_arms(
            need_left, need_right);
        backend_->release_legs();
    };
    auto result = std::make_shared<MovePose::Result>();
    motion_control_pkg::execution::MotionSafetyScope safety_scope((need_left?1U:0U)|(need_right?2U:0U)|4U);
    result->success = false;
    auto feedback = std::make_shared<MovePose::Feedback>();
    const auto publish_feedback = [&](float progress, const std::string& step,
                                      std::uint8_t phase=MovePose::Feedback::STATUS_EXECUTING) {
        feedback->task_status = progress>=100.0F ? "SUCCEEDED" : "RUNNING";
        feedback->progress_rate = std::clamp(progress, 0.0F, 100.0F);
        feedback->dual_task_status = phase;
        feedback->left_progress_rate = need_left ? feedback->progress_rate : 0.0F;
        feedback->right_progress_rate = need_right ? feedback->progress_rate : 0.0F;
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
    if (!read_balance_state(start_q, result->error_message)) {
        if (result->error_message.empty()) result->error_message = "failed to obtain whole-body start state";
        finish_time();
        terminal.abort(result);
        release_child_resources();
        return;
    }
    publish_feedback(5.0F, "whole-body weighted IK planning",MovePose::Feedback::STATUS_PLANNING);
    std::vector<std::vector<double>> left_path;
    std::vector<std::vector<double>> right_path;
    std::vector<std::vector<double>> leg_path;
    std::string planning_error;
    std::optional<Eigen::Isometry3d> left_target_tcp;
    std::optional<Eigen::Isometry3d> right_target_tcp;
    motion_control_pkg::wbc::ReferenceResult reference;
    if (!plan_trajectory(
        goal, start_q, left_path, right_path, leg_path, planning_error,
        left_target_tcp, right_target_tcp, backend_->needs_reference() ? &reference : nullptr)) {
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

    if (backend_->needs_reference() && (!reference.valid || !miracle_algo_->staticBalance() ||
        !backend_->prepare(miracle_algo_->staticBalance()->model().neutralSnapshot().names,reference,result->error_message))) {
        if (result->error_message.empty()) result->error_message="backend requires a validated dynamic reference";
        finish_time(); terminal.abort(result); release_child_resources(); return;
    }
    if (!check_balance_feedback(start_q, need_left, need_right, true, result->error_message)) {
        finish_time(); terminal.abort(result); release_child_resources(); return;
    }
    const std::size_t arm_samples=need_left ? left_path.size() : right_path.size();
    if (arm_samples!=leg_path.size() || (need_left && need_right && left_path.size()!=right_path.size())) {
        result->error_message="whole-body arm/leg trajectory sample counts differ";
        finish_time(); terminal.abort(result); release_child_resources(); return;
    }
    const auto enough_time = [&] {
        if (goal->timeout_ms <= 0) return true;
        // Include the common future start and the minimum measured settling window.
        const double remaining = goal->timeout_ms / 1000.0 - 0.01 -
            std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at).count();
        return motion_control_pkg::execution::hasCompletionBudget(arm_samples, remaining);
    };
    if (!enough_time()) {
        result->error_message = "whole-body motion and settling exceed remaining timeout; no samples sent";
        finish_time(); terminal.abort(result); release_child_resources(); return;
    }
    publish_feedback(20.0F,"preparing arm and leg trajectories");
    motion_control_pkg::motion::TcpLevelPlan level_monitor(goal->keep_tcp_level,
        miracle_algo_->getModel(), start_q, need_left, need_right, true,
        left_model_end_to_tool_tcp_, right_model_end_to_tool_tcp_, goal->tcp_level_tolerance_rad,graspOptions(*goal));
    if (!level_monitor.initialize(result->error_message) ||
        !level_monitor.startMonitoring(result->error_message)) {
        finish_time(); terminal.abort(result); release_child_resources(); return;
    }
    if (!backend_->prepare_dispatch(left_path,right_path,leg_path,goal->control_mode,result->error_message)) {
        finish_time(); terminal.abort(result); release_child_resources(); return;
    }
    if (goal_handle->is_canceling() || (goal->timeout_ms>0 &&
        std::chrono::steady_clock::now()-started_at>=std::chrono::milliseconds(goal->timeout_ms))) {
        result->error_message="whole-body MovePose canceled or timed out during preparation";
        finish_time();
        if (goal_handle->is_canceling()) terminal.cancel(result); else terminal.abort(result);
        release_child_resources(); return;
    }
    if (!check_balance_feedback(start_q,need_left,need_right,true,result->error_message)) {
        finish_time(); terminal.abort(result); release_child_resources(); return;
    }
    if (!enough_time()) {
        result->error_message = "insufficient timeout after whole-body preparation; no samples sent";
        finish_time(); terminal.abort(result); release_child_resources(); return;
    }
    publish_feedback(30.0F, "dispatching preplanned arm and leg trajectories");
    auto cancel = std::make_shared<motion_control_pkg::execution::CancellationToken>();
    // Observation must not delay cancellation/balance supervision or child start.
    motion_control_pkg::execution::LatestProgressPublisher stream_progress([&](float value) {
        publish_feedback(value,cancel->requested() ? "stopping synchronized whole-body trajectory" :
            "executing synchronized whole-body trajectory",cancel->requested() ?
            MovePose::Feedback::STATUS_STOPPING : MovePose::Feedback::STATUS_EXECUTING);
    });
    if (stream_progress.failed()) {
        result->error_message=stream_progress.error();stream_progress.stop();
        finish_time();terminal.abort(result);release_child_resources();return;
    }
    // Preserve HHworkspace's two-phase dispatch: all preparation is complete.
    const auto start_time = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(10);
    std::string arm_error;
    std::string leg_error;
    std::future<bool> arm_future;
    std::future<bool> leg_future;
    // Declared after futures: request cancellation BEFORE their destructors
    // join async workers if feedback publication or another operation throws.
    struct CancelWorkersOnExit {
        std::shared_ptr<motion_control_pkg::execution::CancellationToken> token;
        ~CancelWorkersOnExit() {token->request();}
    } cancel_workers_on_exit{cancel};
    try {
        arm_future = std::async(
            std::launch::async,
            [this, &left_path, &right_path, goal, start_time, cancel, &arm_error]() {
                return backend_->execute_arms(
                    left_path, right_path, goal->control_mode, start_time,
                    cancel, arm_error);
            });
        leg_future = std::async(
            std::launch::async,
            [this, &leg_path, start_time, cancel, &leg_error]() {
                return backend_->execute_legs(
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
        stream_progress.stop();
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
        if (stream_progress.failed()) {
            child_failed=true;cancel->request();
            if(result->error_message.empty())result->error_message=stream_progress.error();
        }
        if (!check_balance_feedback(start_q, need_left, need_right, false, result->error_message)) {
            child_failed = true; cancel->request();
        }
        if (!arm_ok && arm_future.wait_for(std::chrono::milliseconds(20)) ==
            std::future_status::ready) {
            try {arm_ok = arm_future.get();}
            catch (const std::exception& e) {arm_ok=false; arm_error=std::string("arm backend exception: ")+e.what();}
            if (!*arm_ok) {
                child_failed = true;
                cancel->request();
            }
        }
        if (!leg_ok && leg_future.wait_for(std::chrono::milliseconds(arm_ok ? 20 : 0)) ==
            std::future_status::ready) {
            try {leg_ok = leg_future.get();}
            catch (const std::exception& e) {leg_ok=false; leg_error=std::string("leg backend exception: ")+e.what();}
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
        stream_progress.update(30.0F);
    }
    if (child_failed || timed_out || goal_handle->is_canceling())
        safety_scope.finish(false,result->error_message);
    stream_progress.stop();
    if (stream_progress.failed()) {
        child_failed=true;cancel->request();
        if(result->error_message.empty())result->error_message=stream_progress.error();
    }
    std::vector<double> final_q;
    // A successful stream only means the last command was sent. Keep all
    // resources reserved until measured feedback has continuously settled.
    if (!child_failed && !timed_out && !goal_handle->is_canceling()) {
        using Guard = motion_control_pkg::execution::WholeBodyCompletionGuard;
        const auto settling_started = Guard::Clock::now();
        auto deadline = settling_started + std::chrono::seconds(3);
        if (goal->timeout_ms > 0)
            deadline = std::min(deadline, started_at + std::chrono::milliseconds(goal->timeout_ms));
        Guard guard(need_left ? left_path.back() : std::vector<double>{},
                    need_right ? right_path.back() : std::vector<double>{},
                    leg_path.back(), settling_started, deadline, action_precision.completion);
        publish_feedback(95.0F, "waiting for measured arm and waist feedback to settle");
        while (!goal_handle->is_canceling()) {
            if (!rclcpp::ok()) {
                child_failed = true;
                result->error_message = "ROS shutdown while waiting for whole-body completion";
                break;
            }
            if (!check_balance_feedback(start_q,need_left,need_right,false,result->error_message)) {
                child_failed = true;
                break;
            }
            motion_control_pkg::execution::WholeBodyCompletionFeedback measured;
            if (!backend_->read_completion_feedback(measured,result->error_message)) {
                child_failed = true;
                result->error_message = "WBC completion feedback: " + result->error_message;
                break;
            }
            // Evaluate TCP goals from the same measured frame used for joint
            // settling, including when COM gating is disabled.
            std::vector<double> measured_active;
            constexpr double radians = 0.017453292519943295;
            for (std::size_t arm = 0; arm < 2; ++arm) {
                if ((arm == 0 && !need_left) || (arm == 1 && !need_right)) continue;
                for (double q : measured.arms.arms[arm].joint_position) measured_active.push_back(q * radians);
            }
            measured_active.insert(measured_active.end(), measured.leg_positions.begin(), measured.leg_positions.end());
            if (!miracle_algo_->mergeWholeBodyJoints(start_q, need_left, need_right, measured_active, final_q)) {
                child_failed = true; result->error_message = "cannot reconstruct measured completion state"; break;
            }
            bool tcp_reached = true;
            for (bool left : {true, false}) {
                if ((left && !need_left) || (!left && !need_right)) continue;
                Eigen::Isometry3d actual;
                if (!miracle_algo_->getFramePoseAtConfiguration(planning_frame_,
                        left ? "gripper_base_l_link" : "gripper_base_r_link", final_q, actual)) {
                    child_failed = true; result->error_message = "invalid measured completion FK"; break;
                }
                actual = actual * (left ? left_model_end_to_tool_tcp_ : right_model_end_to_tool_tcp_);
                const auto& target = left ? left_target_tcp : right_target_tcp;
                if (!target || !actual.matrix().allFinite()) {
                    child_failed = true; result->error_message = "non-finite measured TCP or missing goal"; break;
                }
                const double position_error = (actual.translation() - target->translation()).norm();
                const double angle_error = motion_control_pkg::motion::quaternionAngularDistance(
                    Eigen::Quaterniond(actual.rotation()), Eigen::Quaterniond(target->rotation()));
                if (left) result->left_final_distance_error_m = position_error;
                else result->right_final_distance_error_m = position_error;
                tcp_reached = tcp_reached && position_error <= motion_control_pkg::execution::kGoalTranslationToleranceM &&
                    angle_error <= motion_control_pkg::execution::kGoalRotationToleranceRad;
            }
            if (child_failed) break;
            const auto status = guard.update(measured,Guard::Clock::now(),result->error_message,tcp_reached);
            if (status == Guard::Status::Failed) { child_failed = true; break; }
            if (status == Guard::Status::Settled) {
                RCLCPP_INFO(rclcpp::get_logger("whole_body_module"),
                    "WBC precision_arrival=%s settled after %.3f s (leg_tolerance=%.6f rad, stable=%.3f s)",
                    goal->precision_arrival ? "ON" : "OFF",
                    std::chrono::duration<double>(Guard::Clock::now()-settling_started).count(),
                    action_precision.completion.position_tolerance, action_precision.completion.stable_seconds);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (goal_handle->is_canceling())
            result->error_message = "whole-body MovePose canceled while waiting for measured completion";
    }
    finish_time();

    if (!child_failed && !timed_out && !goal_handle->is_canceling() && balance_state_) {
        if (!read_balance_state(final_q,result->error_message)) child_failed=true;
        else {
            std::vector<double> final_active,expected,measured_leg;
            if (!miracle_algo_->extractLegJoints(final_q, measured_leg) ||
                measured_leg.size() != leg_path.back().size()) {
                child_failed=true; result->error_message="cannot extract final measured WBC leg state";
            }
            if (need_left) final_active.insert(final_active.end(),left_path.back().begin(),left_path.back().end());
            if (need_right) final_active.insert(final_active.end(),right_path.back().begin(),right_path.back().end());
            // Exclude legs from the fixed arm/held-joint tolerance below. Their
            // original commanded target is checked separately with this Goal's
            // selected tolerance; no command or goal is changed here.
            final_active.insert(final_active.end(),measured_leg.begin(),measured_leg.end());
            if (!child_failed && !miracle_algo_->mergeWholeBodyJoints(start_q,need_left,need_right,final_active,expected)) {
                child_failed=true; result->error_message="cannot reconstruct final WBC command";
            } else if (!child_failed) for (std::size_t i=0;i<final_q.size();++i) if (
                    !std::isfinite(final_q[i]) || std::abs(final_q[i]-expected[i])>.01) {
                child_failed=true; result->error_message="final measured WBC arm/held position differs from reference"; break;
            }
            if (!child_failed) {
                for (std::size_t i=0;i<measured_leg.size();++i) {
                    if (!std::isfinite(measured_leg[i]) ||
                        std::abs(measured_leg[i]-leg_path.back()[i]) > action_precision.completion.position_tolerance) {
                        child_failed=true; result->error_message="final measured WBC leg position exceeds selected completion tolerance"; break;
                    }
                }
            }
        }
    }
    if (goal_handle->is_canceling()) {
        if (result->error_message.empty()) {
            result->error_message = !arm_error.empty() ? arm_error : !leg_error.empty() ? leg_error : "whole-body MovePose canceled";
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
        result->final_distance_error = std::max(
            result->left_final_distance_error_m, result->right_final_distance_error_m);
        publish_feedback(100.0F, "whole-body MovePose complete");
        terminal.succeed(result);
    }
    release_child_resources();
}
