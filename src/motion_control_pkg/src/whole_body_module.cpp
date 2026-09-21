#include "motion_control_pkg/motion/scoped_collision_scene.hpp"
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
    auto collision_scene = motion_control_pkg::motion::makeCollisionScene(
        *miracle_algo_, goal->enable_collision_check, start_q,
        goal->env_point_cloud, error_message);
    if (!collision_scene) return false;

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
                const auto left_step = std::optional<Eigen::Isometry3d>(
                    interpolatePose(left_start, left_root_goal, alpha));
                const auto right_step = std::optional<Eigen::Isometry3d>(
                    interpolatePose(right_start, right_root_goal, alpha));
                Eigen::VectorXd q_next;
                if (!miracle_algo_->solveWholeBodyIK(
                        q_current, left_step, right_step, q_next,
                        std::array<double, 4>{1.0, 1.0, 1.0, 0.2},
                        0.0, 10.0, 5U, false)) {
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
                        0.0, 5.0, 15U, true)) {
                    error_message =
                        "failed to find a collision-free dual-arm whole-body "
                        "IK solution for the target pose";
                    return false;
                }
                target_eigen = std::move(collision_free_target);
            }
        } else if (!miracle_algo_->solveWholeBodyIK(
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

    if (goal->enable_move_line && !dynamic_zmp_enabled_) {
        if (!miracle_algo_->validateStaticBalancePath(full_waypoints, error_message,
                motion_control_pkg::execution::kDualArmControlPeriodSeconds)) return false;
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
        if (backend_ && backend_->arm_limits(
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
    // URDF velocity limits remain authoritative; acceleration and jerk are
    // planner policies because ordinary URDF does not define either quantity.
    if (miracle_algo_->staticBalance()) {
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
            if (!(active_limits[i] > 0)) { error_message = "active URDF joint has zero velocity limit"; return false; }
            velocity_limits[i] = std::min(velocity_limits[i], speed * active_limits[i]);
        }
    }

    std::vector<std::vector<double>> active_trajectory;
    if (goal->enable_move_line) {
        active_trajectory = active_waypoints;
    } else if (active_waypoints.size() == 2U) {
        if (!planRuckigSamplesWholeBody(
                *miracle_algo_, active_waypoints.front(), active_waypoints.back(),
                velocity_limits, acceleration_limits, jerk_limits,
                active_trajectory, error_message)) {
            return false;
        }
    } else {
        if (!planToppraSamplesWholeBody(
                *miracle_algo_, active_waypoints, velocity_limits,
                acceleration_limits, jerk_limits, active_trajectory, error_message)) {
            return false;
        }
    }

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

    // TOPPRA may generate samples between OMPL waypoints.  Rebuild complete
    // configurations and validate them when collision checking was requested.
    std::vector<std::vector<double>> balance_path;
    if (goal->enable_collision_check || miracle_algo_->staticBalance()) {
        for (const auto& active_q : active_trajectory) {
            std::vector<double> full_q;
            if (!miracle_algo_->mergeWholeBodyJoints(
                    start_q, use_left, use_right, active_q, full_q) ||
                (goal->enable_collision_check && !miracle_algo_->isFullQValid(miracle_algo_->toEigenConfig(full_q)))) {
                error_message = "time-parameterized whole-body path violates collision constraints";
                return false;
            }
            balance_path.push_back(std::move(full_q));
        }
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
    publish_feedback(20.0F,"preparing arm and leg trajectories");
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
    publish_feedback(30.0F, "dispatching preplanned arm and leg trajectories");
    auto cancel = std::make_shared<motion_control_pkg::execution::CancellationToken>();
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
        publish_feedback(30.0F, cancel->requested() ? "stopping synchronized whole-body trajectory" : "executing synchronized whole-body trajectory",
            cancel->requested() ? MovePose::Feedback::STATUS_STOPPING : MovePose::Feedback::STATUS_EXECUTING);
    }
    finish_time();

    std::vector<double> final_q;
    if (!child_failed && !timed_out && !goal_handle->is_canceling() && balance_state_) {
        if (!read_balance_state(final_q,result->error_message)) child_failed=true;
        else {
            std::vector<double> final_active,expected;
            if (need_left) final_active.insert(final_active.end(),left_path.back().begin(),left_path.back().end());
            if (need_right) final_active.insert(final_active.end(),right_path.back().begin(),right_path.back().end());
            final_active.insert(final_active.end(),leg_path.back().begin(),leg_path.back().end());
            if (!miracle_algo_->mergeWholeBodyJoints(start_q,need_left,need_right,final_active,expected)) {
                child_failed=true; result->error_message="cannot reconstruct final WBC command";
            } else for (std::size_t i=0;i<final_q.size();++i) if (std::abs(final_q[i]-expected[i])>.01) {
                child_failed=true; result->error_message="final measured WBC position differs from reference"; break;
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
        // 参照 dual_arm_module 计算 TCP 最终位置误差：FK 求当前 TCP，与目标 TCP 比较。
        const auto compute_tcp_error =
            [&](const std::optional<Eigen::Isometry3d>& target_tcp,
                const char* end_frame,
                const Eigen::Isometry3d& model_end_to_tool_tcp) -> double {
            if (!target_tcp) {
                return 0.0;
            }
            Eigen::Matrix4d planning_to_model_end;
            bool fk_ok=false;
            if (balance_state_) {
                Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
                fk_ok=miracle_algo_->getFramePoseAtConfiguration(planning_frame_,end_frame,final_q,pose);
                planning_to_model_end=pose.matrix();
            } else fk_ok=miracle_algo_->getForwardKinematics(planning_frame_,end_frame,planning_to_model_end);
            if (!fk_ok || !planning_to_model_end.allFinite())
            {
                RCLCPP_WARN(rclcpp::get_logger("whole_body_module"),
                    "FK to %s failed or non-finite; TCP error set to NaN",
                    end_frame);
                return std::numeric_limits<double>::quiet_NaN();
            }
            const geometry_msgs::msg::Pose actual_tcp_pose =
                motion_control_pkg::kinematics::composeToolTcpPose(
                planning_to_model_end, model_end_to_tool_tcp);
            const Eigen::Isometry3d actual_tcp =
                motion_control_pkg::kinematics::poseToIsometry(actual_tcp_pose);
            return (actual_tcp.translation() -
                target_tcp->translation()).norm();
        };
        result->left_final_distance_error_m =
            need_left ?
            compute_tcp_error(left_target_tcp, "gripper_base_l_link",
                left_model_end_to_tool_tcp_) : 0.0;
        result->right_final_distance_error_m =
            need_right ?
            compute_tcp_error(right_target_tcp, "gripper_base_r_link",
                right_model_end_to_tool_tcp_) : 0.0;
        result->final_distance_error = std::max(
            result->left_final_distance_error_m,
            result->right_final_distance_error_m);
        if (balance_state_ && (!std::isfinite(result->left_final_distance_error_m) ||
            !std::isfinite(result->right_final_distance_error_m))) {
            result->success=false; result->error_message="final measured TCP FK is invalid";
            terminal.abort(result); release_child_resources(); return;
        }
        publish_feedback(100.0F, "whole-body MovePose complete");
        terminal.succeed(result);
    }
    release_child_resources();
}
