#include "motion_control_pkg/motion/scoped_collision_scene.hpp"
#include "motion_control_pkg/execution/stream_timing.hpp"
#include "dual_arm_module.hpp"
#include "miracle_m_algo.hpp"
#include "motion_control_pkg/execution/arm_action_utils.hpp"
#include "motion_control_pkg/execution/control_period.hpp"
#include "motion_control_pkg/execution/preplanned_dispatch.hpp"
#include "motion_control_pkg/execution/execution_control.hpp"
#include "motion_control_pkg/execution/impedance_configuration.hpp"
#include "motion_control_pkg/execution/joint_command.hpp"
#include "motion_control_pkg/kinematics/transform_utils.hpp"
#include "motion_control_pkg/motion/joint_space_control.hpp"
#include "motion_control_pkg/motion/cartesian_line_sampling.hpp"
#include "motion_control_pkg/motion/pose_space_control.hpp"
#include "motion_control_pkg/suck/neardi47_gpio.h"

#include <std_msgs/msg/header.hpp>
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <limits>
#include <memory>

#include <iostream>
#include <stdexcept>

namespace
{

void sleepUntilTimespec(const timespec& timestamp)
{
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &timestamp, nullptr) == EINTR) {
    }
}

std::int64_t monotonicNanoseconds()
{
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    return static_cast<std::int64_t>(now.tv_sec)*1000000000LL + now.tv_nsec;
}
std::int64_t asNanoseconds(const timespec& t)
{
    return static_cast<std::int64_t>(t.tv_sec)*1000000000LL + t.tv_nsec;
}
struct StreamTimingReport
{
    motion_control_pkg::execution::StreamTiming stats;
    ~StreamTimingReport()
    {
        RCLCPP_INFO(rclcpp::get_logger("dual_arm_module"),
            "arm stream timing: samples=%zu mean_gap_us=%.3f max_gap_us=%.3f "
            "max_lateness_us=%.3f late_over_1ms=%zu",
            stats.samples, stats.samples > 1 ? double(stats.gap_sum)/(stats.samples-1)/1000 : 0,
            double(stats.maximum_gap)/1000, double(stats.maximum_lateness)/1000, stats.late_samples);
    }
};
void scheduleNextSample(timespec& next, std::int64_t started)
{
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        motion_control_pkg::execution::kDualArmControlPeriod).count();
    const auto ns = motion_control_pkg::execution::nextStreamTick(
        asNanoseconds(next), started, monotonicNanoseconds(), period);
    next.tv_sec = ns / 1000000000LL;
    next.tv_nsec = ns % 1000000000LL;
    sleepUntilTimespec(next);
}

template <typename GoalHandleT>
bool streamSingleArmTrajectory(
    motion_control_pkg::sdk::SdkCommandGate& sdk_command_gate,
    const std::shared_ptr<GoalHandleT>& goal_handle,
    motion_control_pkg::sdk::ArmChannel channel,
    const std::vector<motion_control_pkg::execution::JointCommand>& command_points,
    motion_control_pkg::execution::SteadyDeadline& deadline,
    const char* action_name,
    const std::function<void(float, const std::string&)>& publish_feedback,
    const std::function<void(const std::string&)>& abort_goal,
    const std::function<void(const std::string&)>& cancel_goal)
{
    if (command_points.empty()) {
        abort_goal("trajectory must contain at least one sample");
        return false;
    }
    const std::size_t planned_samples = command_points.size();
    if ((planned_samples-1)*motion_control_pkg::execution::kDualArmControlPeriodSeconds > deadline.remainingSeconds()) {
        abort_goal("Planned motion exceeds remaining action timeout; no trajectory samples sent");
        return false;
    }
    StreamTimingReport timing;
    timespec next_cycle{};
    clock_gettime(CLOCK_MONOTONIC, &next_cycle);
    auto next_feedback = std::chrono::steady_clock::now();
    const char* arm_name =
        channel == motion_control_pkg::sdk::ArmChannel::kLeft ? "left" : "right";
    for (std::size_t index = 0; index < command_points.size(); ++index) {
        if (goal_handle->is_canceling()) {
            cancel_goal(
                std::string(action_name) +
                " goal canceled by client; future samples suppressed");
            return false;
        }
        if (deadline.expired()) {
            abort_goal(std::string(action_name) + " command streaming timed out");
            return false;
        }
        const auto sample_started = monotonicNanoseconds();
        timing.stats.observe(asNanoseconds(next_cycle), sample_started, 4000000LL);
        const auto send_result =
            sdk_command_gate.sendJointPosition(channel, command_points[index]);
        if (!send_result) {
            abort_goal(send_result.error_message);
            return false;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_feedback) {
            const float command_progress =
                static_cast<float>(index + 1U) /
                static_cast<float>(command_points.size());
            publish_feedback(
                25.0F + 70.0F * command_progress,
                std::string(arm_name) + " arm streaming commands at 4 ms");
            next_feedback = now + std::chrono::milliseconds(50);
        }
        if (index + 1U < command_points.size()) {
            scheduleNextSample(next_cycle, sample_started);
        }
    }
    return true;
}

template <typename GoalHandleT>
bool streamDualArmTrajectory(
    motion_control_pkg::sdk::SdkCommandGate& sdk_command_gate,
    const std::shared_ptr<GoalHandleT>& goal_handle,
    const std::vector<motion_control_pkg::execution::JointCommand>& left_commands,
    const std::vector<motion_control_pkg::execution::JointCommand>& right_commands,
    motion_control_pkg::execution::SteadyDeadline& deadline,
    const char* action_name,
    std::uint8_t executing_status,
    const char* feedback_step,
    const std::function<void(std::uint8_t, float, float, const std::string&)>& publish_feedback,
    const std::function<void(const std::string&)>& abort_goal,
    const std::function<void(const std::string&)>& cancel_goal)
{
    if (left_commands.empty() || right_commands.empty()) {
        abort_goal("both synchronized trajectories must contain samples");
        return false;
    }
    if (left_commands.size() != right_commands.size()) {
        abort_goal("dual-arm synchronized trajectories must have equal sample counts");
        return false;
    }
    const std::size_t planned_samples = left_commands.size();
    if ((planned_samples-1)*motion_control_pkg::execution::kDualArmControlPeriodSeconds > deadline.remainingSeconds()) {
        abort_goal("Planned motion exceeds remaining action timeout; no trajectory samples sent");
        return false;
    }
    StreamTimingReport timing;
    timespec next_cycle{};
    clock_gettime(CLOCK_MONOTONIC, &next_cycle);
    auto next_feedback = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < left_commands.size(); ++index) {
        if (goal_handle->is_canceling()) {
            cancel_goal(
                std::string(action_name) +
                " canceled; future paired samples suppressed");
            return false;
        }
        if (deadline.expired()) {
            abort_goal(std::string(action_name) + " command streaming timed out");
            return false;
        }
        const auto sample_started = monotonicNanoseconds();
        timing.stats.observe(asNanoseconds(next_cycle), sample_started, 4000000LL);
        const auto send_result = sdk_command_gate.sendJointPositions(
            left_commands[index], right_commands[index]);
        if (!send_result) {
            abort_goal(send_result.error_message);
            return false;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_feedback) {
            const float progress =
                20.0F + 70.0F * static_cast<float>(index + 1U) /
                static_cast<float>(left_commands.size());
            publish_feedback(
                executing_status, progress, progress, feedback_step);
            next_feedback = now + std::chrono::milliseconds(50);
        }
        if (index + 1U < left_commands.size()) {
            scheduleNextSample(next_cycle, sample_started);
        }
    }
    return true;
}

bool planRuckigSamples(
    MiracleMAlgo& miracle_algo,
    const std::vector<double>& start,
    const std::vector<double>& target,
    const std::vector<double>& velocity_limits,
    const std::vector<double>& acceleration_limits,
    const std::vector<double>& jerk_limits,
    std::vector<std::vector<double>>& trajectory,
    std::string& error_message)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("dual_arm_module"),
        "Using Ruckig point-to-point planner, dof=" << start.size()
        << ", dt=" << motion_control_pkg::execution::kDualArmControlPeriodSeconds);
    if (start.empty() || start.size() != target.size() ||
        start.size() != velocity_limits.size() ||
        start.size() != acceleration_limits.size() ||
        start.size() != jerk_limits.size())
    {
        error_message = "Ruckig planning input dimensions do not match";
        return false;
    }
    const std::vector<double> zero_velocity(start.size(), 0.0);
    const std::vector<double> zero_acceleration(start.size(), 0.0);
    if (!miracle_algo.ruckigPlan(
            motion_control_pkg::execution::kDualArmControlPeriodSeconds,
            velocity_limits, acceleration_limits, jerk_limits,
            start, zero_velocity, zero_acceleration, target, trajectory))
    {
        error_message = "MiracleMAlgo::ruckigPlan failed";
        return false;
    }
    error_message.clear();
    return true;
}

bool planToppraSamples(
    MiracleMAlgo& miracle_algo,
    const std::vector<std::vector<double>>& waypoints,
    const std::vector<double>& velocity_limits,
    const std::vector<double>& acceleration_limits,
    const std::vector<double>& jerk_limits,
    std::vector<std::vector<double>>& trajectory,
    std::string& error_message,
    MiracleMAlgo::ToppraSolverMode solver_mode =
        MiracleMAlgo::ToppraSolverMode::kStableCartesian,
    double maximum_duration_seconds =
        std::numeric_limits<double>::infinity())
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("dual_arm_module"),
        "Using TOPPRA multi-waypoint planner, waypoints=" << waypoints.size()
        << ", dof=" << velocity_limits.size()
        << ", dt=" << motion_control_pkg::execution::kDualArmControlPeriodSeconds);
    if (waypoints.size() < 2U || velocity_limits.empty() ||
        velocity_limits.size() != acceleration_limits.size())
    {
        error_message = "TOPPRA planning input dimensions are invalid";
        return false;
    }
    if (!miracle_algo.toppraPlan(
            waypoints,
            motion_control_pkg::execution::kDualArmControlPeriodSeconds,
            velocity_limits, acceleration_limits, trajectory, solver_mode,
            maximum_duration_seconds, jerk_limits))
    {
        error_message = "MiracleMAlgo::toppraPlan failed";
        return false;
    }
    error_message.clear();
    return true;
}

std::vector<std::vector<double>> extractWaypointPositions(
    const std::vector<Waypoint>& waypoints)
{
    std::vector<std::vector<double>> positions;
    positions.reserve(waypoints.size());
    for (const auto& waypoint : waypoints) {
        positions.push_back(waypoint.positions);
    }
    return positions;
}

}  // namespace

DualArmModule::DualArmModule(
    const std::string& arm_name,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<motion_control_pkg::sdk::DualArmDeviceInterface> arm_device,
    std::shared_ptr<MiracleMAlgo> miracle_algo,
    motion_control_pkg::sdk::ActiveArmConfiguration arm_configuration,
    std::string sdk_config_path,
    std::string planning_frame,
    Eigen::Isometry3d left_model_end_to_tool_tcp,
    Eigen::Isometry3d right_model_end_to_tool_tcp)
: name_(arm_name),
  node_(node),
  is_executing_left_arm_(false),
  is_executing_right_arm_(false),
  is_executing_left_tool_(false),
  is_executing_right_tool_(false),
  arm_device_(std::move(arm_device)),
  miracle_algo_(std::move(miracle_algo)),
  arm_configuration_(arm_configuration),
  sdk_config_path_(std::move(sdk_config_path)),
  planning_frame_(std::move(planning_frame)),
  left_model_end_to_tool_tcp_(std::move(left_model_end_to_tool_tcp)),
  right_model_end_to_tool_tcp_(std::move(right_model_end_to_tool_tcp))
{
    auto shared_node = lockNode(__func__);
    if (!shared_node) {
        return;
    }
    if (!arm_device_ || !miracle_algo_) {
        throw std::invalid_argument(
            "DualArmModule requires injected SDK adapter and MiracleMAlgo");
    }
    acceleration_scale_ = shared_node->declare_parameter<double>("arm_acceleration_scale", 1.0);
    jerk_scale_ = shared_node->declare_parameter<double>("arm_jerk_scale", 1.0);
    if (!std::isfinite(acceleration_scale_) || acceleration_scale_ <= 0 || acceleration_scale_ > 1 ||
        !std::isfinite(jerk_scale_) || jerk_scale_ <= 0 || jerk_scale_ > 1) {
        throw std::invalid_argument("arm acceleration/jerk scales must be in (0,1]");
    }
    const std::vector<double> joint_stiffness{
        5.0, 5.0, 5.0, 4.0, 3.0, 3.0, 2.0};
    const std::vector<double> joint_damping{
        0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2};
    const std::vector<double> cartesian_stiffness{
        10000.0, 10000.0, 10000.0, 600.0, 600.0, 600.0, 20.0};
    const std::vector<double> cartesian_damping{
        0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1.0};
    joint_impedance_parameters_[0] =
        motion_control_pkg::execution::declareImpedanceParameters(
        *shared_node, "left_joint_impedance", joint_stiffness,
        joint_damping, 10, 10);
    joint_impedance_parameters_[1] =
        motion_control_pkg::execution::declareImpedanceParameters(
        *shared_node, "right_joint_impedance", joint_stiffness,
        joint_damping, 10, 10);
    cartesian_impedance_parameters_[0] =
        motion_control_pkg::execution::declareImpedanceParameters(
        *shared_node, "left_cartesian_impedance", cartesian_stiffness,
        cartesian_damping, 20, 20);
    cartesian_impedance_parameters_[1] =
        motion_control_pkg::execution::declareImpedanceParameters(
        *shared_node, "right_cartesian_impedance", cartesian_stiffness,
        cartesian_damping, 20, 20);

    const std::vector<double> left_tool_dynamics{
        1.7, -0.36326579, -174.45006830, -0.57521107,
        0.005674265845, 0.000000029998, -0.000035481733,
        0.001812184129, 0.000014284801, 0.004925221043};
    const std::vector<double> right_tool_dynamics{
        0.35746, -0.36386, -128.866, -0.79,
        0.000598, 0.0, 0.0, 0.000177, 0.000016, 0.000602};
    tool_parameters_[0] = motion_control_pkg::execution::declareToolParameters(
        *shared_node, "left", left_model_end_to_tool_tcp_, left_tool_dynamics);
    tool_parameters_[1] = motion_control_pkg::execution::declareToolParameters(
        *shared_node, "right", right_model_end_to_tool_tcp_, right_tool_dynamics);

    sdk_command_gate_ =
        std::make_unique<motion_control_pkg::sdk::SdkCommandGate>(
        *arm_device_, arm_configuration_);

    RCLCPP_INFO(
        rclcpp::get_logger("dual_arm_module"),
        "Constructing dual arm module without hardware connection...");

    // 初始化 PickAndPlace Action Server
    // this->pick_and_place_action_server_ = rclcpp_action::create_server<PickAndPlace>(
    //     this,
    //     "/Control/pick_and_place",
    //     std::bind(&DualArmModule::handle_goal_pick_and_place, this, std::placeholders::_1, std::placeholders::_2),
    //     std::bind(&DualArmModule::handle_cancel_pick_and_place, this, std::placeholders::_1),
    //     std::bind(&DualArmModule::handle_accepted_pick_and_place, this, std::placeholders::_1));

    // // 初始化 HardwareReset Action Server
    // this->hardware_reset_action_server_ = rclcpp_action::create_server<HardwareReset>(
    //     this,
        
    //     "/Control/hardware_reset",
    //     std::bind(&DualArmModule::handle_goal_hardware_reset, this, std::placeholders::_1, std::placeholders::_2),
    //     std::bind(&DualArmModule::handle_cancel_hardware_reset, this, std::placeholders::_1),
    //     std::bind(&DualArmModule::handle_accepted_hardware_reset, this, std::placeholders::_1));


    // // 初始化 HardwareCheck Action Server
    // this->hardware_Check_action_server_ = rclcpp_action::create_server<HardwareCheck>(
    //     this,
    //     "/Control/hardware_check",
    //     std::bind(&DualArmModule::handle_goal_hardware_check, this, std::placeholders::_1, std::placeholders::_2),
    //     std::bind(&DualArmModule::handle_cancel_hardware_check, this, std::placeholders::_1),
    //     std::bind(&DualArmModule::handle_accepted_hardware_check, this, std::placeholders::_1));

   
    heart_beat_pub_ = shared_node->create_publisher<HeartBeat>("/SafeNode/heartbeat", 10);
    device_status_pub_ = shared_node->create_publisher<NodeStatus>("/SafeNode/status", 10);
    // this->status = STATE_INIT;
    // this->error_flag = 0;

    hear_beat_timer_ = shared_node->create_wall_timer(1000ms,
    std::bind(&DualArmModule::heart_beat_cb, this));
    auto ds_msg = com_interfaces::msg::NodeStatus();
    ds_msg.node_id = MOTION_CONTROL;
    ds_msg.status = STATE_INIT ;//node_->status;
    ds_msg.header.stamp = shared_node->get_clock()->now();
    ds_msg.error_flag = 0;
    device_status_pub_->publish(ds_msg);

    r_q_lower_limits_.resize(arm_dof_), 
    r_q_upper_limits_.resize(arm_dof_),  
    r_v_limit_.resize(arm_dof_),
    r_a_limit_.resize(arm_dof_),
    r_jerk_limit_.resize(arm_dof_),
    l_q_lower_limits_.resize(arm_dof_),
    l_q_upper_limits_.resize(arm_dof_),
    l_v_limit_.resize(arm_dof_),
    l_a_limit_.resize(arm_dof_),
    l_jerk_limit_.resize(arm_dof_);
   
    this->motor_status_pub_ = shared_node->create_publisher<MotorStatus>("/Control/MotorInfo",10);
    measured_feedback_channel_ = std::make_unique<motion_control_pkg::wbc::FeedbackChannel>(
        shared_node, [this](sensor_msgs::msg::JointState& output, std::string& error) {
            return readMeasuredFeedback(output, error);
        });
    joint_feedback_timer_ = shared_node->create_wall_timer(10ms,std::bind(&DualArmModule::motor_statu_cb, this));
    motor_status_msg_ = com_interfaces::msg::MotorStatus();
    init_motor_status();

    // 吸盘 GPIO 提前到构造时初始化，避免首次 ControlSuck 动作才申请 /dev/gpiochip4。
    // 失败仅记录错误不抛出：动作执行路径仍会按 suck_gpio_initialized_ 标志重试。 高电平表示关闭吸盘。
    if (neardi47_gpio_init(true, true) < 0) {
        const int error_number = errno;
        RCLCPP_ERROR(
            rclcpp::get_logger("dual_arm_module"),
            "Failed to initialize /dev/gpiochip4 during construction: %s",
            std::strerror(error_number));
    } else {
        suck_gpio_initialized_ = true;
    }

    // joint_command_timer_ = create_wall_timer(250ms,
    // std::bind(&DualArmModule::joint_command_cb, this));

    head_status_pub_=shared_node->create_publisher<MotorStatus>("/Control/HeadPose", 10);
    head_feedback_timer_ = shared_node->create_wall_timer(33ms, std::bind(&DualArmModule::head_status_cb, this));
    head_status_msg_ = com_interfaces::msg::MotorStatus();

    sub_pointcloudNode_ = std::make_shared<rclcpp::Node>("pointcloud_sub_node");
    if (!shared_node->has_parameter("collision_pointcloud_topic")) {
        shared_node->declare_parameter<std::string>("collision_pointcloud_topic", "/fusion/synced_robot_vision");
    }
    collision_pointcloud_topic_ = shared_node->get_parameter("collision_pointcloud_topic").as_string();

    // 建立持久订阅,后台线程持续接收
    collision_cloud_sub_ = sub_pointcloudNode_->create_subscription<RobotVision>(
        collision_pointcloud_topic_, rclcpp::SensorDataQoS(),
        [this](const RobotVision::SharedPtr msg) {
            if (!msg || msg->camera_rgbd_pointcloud.data.empty()) return;
            {
                std::lock_guard<std::mutex> lk(latest_cloud_mutex_);
                latest_cloud_ = msg->camera_rgbd_pointcloud;
                latest_cloud_ready_ = true;
            }
            latest_cloud_cv_.notify_one();
        });
    cloud_spin_running_ = true;
    cloud_spin_thread_ = std::thread([this] {
        while (cloud_spin_running_ && rclcpp::ok()) {
            rclcpp::spin_some(sub_pointcloudNode_);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });
}

bool DualArmModule::initialize(std::string& error_message)
{
    // 先完成纯本地配置和运动学初始化，失败时不建立任何机器人连接。
    l_init_arm_params_ = init_arm_params();
    if (!l_init_arm_params_) {
        error_message = "Failed to load or initialize Marvin kinematics config: " +
            sdk_config_path_;
        return false;
    }
    if (!init_set_tcp(error_message)) {
        return false;
    }

    const auto connect_result =
        sdk_command_gate_->connect({192, 168, 1, 190});
    if (!connect_result) {
        error_message = connect_result.error_message;
        return false;
    }
    sdk_connected_ = true;
    sdk_command_gate_->disableLogging();

    if (!set_arm_control_mode(error_message)) {
        const auto release_result = sdk_command_gate_->release();
        sdk_connected_ = false;
        if (!release_result) {
            error_message += "; " + release_result.error_message;
        }
        return false;
    }

    init_robot_link_ = true;
    error_message.clear();
    return true;
}

rclcpp::Node::SharedPtr DualArmModule::lockNode(const char *context) const
{
    auto node = node_.lock();
    if (!node) {
        RCLCPP_ERROR(rclcpp::get_logger("dual_arm_module"), "Node expired in %s", context);
    }
    return node;
}

motion_control_pkg::sdk::SdkCommandResult
DualArmModule::configureGoalControlMode(
    std::uint8_t control_mode,
    motion_control_pkg::sdk::ArmControlMode impedance_mode,
    const std::vector<motion_control_pkg::sdk::ArmChannel>& channels)
{
    using motion_control_pkg::sdk::ArmControlMode;
    using motion_control_pkg::sdk::ArmControlModeCommand;
    using motion_control_pkg::sdk::SdkCommandResult;

    if (control_mode > 1U) {
        return SdkCommandResult::failure(
            "control_mode must be CONTROL_POSITION(0) or "
            "CONTROL_IMPEDANCE(1)");
    }
    if (impedance_mode != ArmControlMode::kJointImpedance &&
        impedance_mode != ArmControlMode::kCartesianImpedance)
    {
        return SdkCommandResult::failure(
            "goal requested an unsupported impedance mode");
    }

    std::vector<ArmControlModeCommand> commands;
    commands.reserve(channels.size());
    for (const auto channel : channels) {
        const auto index = static_cast<std::size_t>(channel);
        if (index >= motion_control_pkg::sdk::kArmCount) {
            return SdkCommandResult::failure(
                "goal requested an invalid arm channel");
        }
        ArmControlModeCommand command;
        command.channel = channel;
        if (control_mode == 0U) {
            command.mode = ArmControlMode::kPosition;
            command.impedance.velocity_ratio = 100;
            command.impedance.acceleration_ratio = 100;
        } else {
            command.mode = impedance_mode;
            command.tool = tool_parameters_[index];
            command.impedance =
                impedance_mode == ArmControlMode::kJointImpedance ?
                joint_impedance_parameters_[index] :
                cartesian_impedance_parameters_[index];
        }
        commands.push_back(command);
    }
    return sdk_command_gate_->configureControlModes(commands);
}

DualArmModule::~DualArmModule()
{
    // 安全退出机制：等待正在执行的线程结束
    if (execution_thread_move_left_pose_.joinable()) {
        execution_thread_move_left_pose_.join();
    }
    if (execution_thread_move_right_pose_.joinable()) {
        execution_thread_move_right_pose_.join();
    }
    if (execution_thread_move_left_joint_.joinable()) {
        execution_thread_move_left_joint_.join();
    }
    if (execution_thread_move_right_joint_.joinable()) {
        execution_thread_move_right_joint_.join();
    }
    if (execution_thread_move_dual_joints_.joinable()) {
        execution_thread_move_dual_joints_.join();
    }
    if (execution_thread_move_dual_pose_.joinable()) {
        execution_thread_move_dual_pose_.join();
    }
    gripper_worker_threads_.joinAll();
    if (execution_thread_control_suck_.joinable()) {
        execution_thread_control_suck_.join();
    }
    if (suck_gpio_initialized_) {
        if (neardi47_gpio_write(true, true) < 0) {
            RCLCPP_ERROR(
                rclcpp::get_logger("ControlSuck"),
                "Failed to switch suction GPIO outputs off during shutdown: %s",
                std::strerror(errno));
        }
        neardi47_gpio_deinit();
        suck_gpio_initialized_ = false;
    }
    if (sdk_connected_ && sdk_command_gate_) {
        const auto release_result = sdk_command_gate_->release();
        if (!release_result) {
            RCLCPP_ERROR(
                rclcpp::get_logger("dual_arm_module"),
                "Dual arm SDK release failed: %s",
                release_result.error_message.c_str());
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("dual_arm_module"),"%s Deconstruct successed", __func__);
}

bool DualArmModule::is_idle(int module_id) const  //根据模块标号返回单个模块是否空闲状态
{
    if(module_id == ID_LEFT_ARM)
    {
        return !is_executing_left_arm_.load();
    }
    else if(module_id == ID_RIGHT_ARM)
    {
        return !is_executing_right_arm_.load();
    }
    else if(module_id == ID_LEFT_TOOL)
    {
        return !is_executing_left_tool_.load();
    }
    else if(module_id == ID_RIGHT_TOOL)
    {
        return !is_executing_right_tool_.load();
    }
    else
    {
        return false;
    }
}

bool DualArmModule::tryClaimArms(
    bool need_left, bool need_right, std::string& error_message)
{
    std::lock_guard<std::mutex> lock(arm_claim_mtx_);
    if (need_left &&
        !motion_control_pkg::sdk::isArmEnabled(
            arm_configuration_, motion_control_pkg::sdk::ArmChannel::kLeft))
    {
        error_message = "left arm is disabled by active_arm_configuration";
        return false;
    }
    if (need_right &&
        !motion_control_pkg::sdk::isArmEnabled(
            arm_configuration_, motion_control_pkg::sdk::ArmChannel::kRight))
    {
        error_message = "right arm is disabled by active_arm_configuration";
        return false;
    }
    if (need_left && is_executing_left_arm_.load()) {
        error_message = "left arm is already occupied by another task";
        return false;
    }
    if (need_right && is_executing_right_arm_.load()) {
        error_message = "right arm is already occupied by another task";
        return false;
    }
    if (need_left) {
        is_executing_left_arm_.store(true);
    }
    if (need_right) {
        is_executing_right_arm_.store(true);
    }
    error_message.clear();
    return true;
}

void DualArmModule::releaseArms(bool release_left, bool release_right) noexcept
{
    std::lock_guard<std::mutex> lock(arm_claim_mtx_);
    if (release_left) {
        is_executing_left_arm_.store(false);
    }
    if (release_right) {
        is_executing_right_arm_.store(false);
    }
}

void DualArmModule::start_execution_move_pose(
    std::shared_ptr<GoalHandleMovePose> goal_handle,
    geometry_msgs::msg::PoseStamped planning_target)
{
    std::string claim_error;
    const auto goal = goal_handle->get_goal();
    const bool need_left = goal->arm_id == ARM_LEFT;
    const bool need_right = goal->arm_id == ARM_RIGHT;
    if (!tryClaimArms(need_left, need_right, claim_error)) {
        auto result = std::make_shared<MovePose::Result>();
        result->success = ACTION_FAIL;
        result->error_message = claim_error;
        goal_handle->abort(result);
        return;
    }
    // 标记为正在执行,
    if(goal->arm_id==ARM_LEFT)
    {
        if (execution_thread_move_left_pose_.joinable()) {
            execution_thread_move_left_pose_.join();
        }
        is_executing_left_arm_.store(true);
        try {
            execution_thread_move_left_pose_ = std::thread(
                [this, goal_handle, planning_target = std::move(planning_target)]() mutable {
                    execution_loop_move_left_pose(
                        goal_handle, std::move(planning_target));
                });
        } catch (const std::exception& error) {
            releaseArms(true, false);
            auto result = std::make_shared<MovePose::Result>();
            result->success = ACTION_FAIL;
            result->error_message =
                std::string("Failed to start left MovePose worker: ") + error.what();
            goal_handle->abort(result);
        }
    }
    else if(goal->arm_id==ARM_RIGHT)
    {
        if (execution_thread_move_right_pose_.joinable()) {
            execution_thread_move_right_pose_.join();
        }
        is_executing_right_arm_.store(true);
        try {
            execution_thread_move_right_pose_ = std::thread(
                [this, goal_handle, planning_target = std::move(planning_target)]() mutable {
                    execution_loop_move_right_pose(
                        goal_handle, std::move(planning_target));
                });
        } catch (const std::exception& error) {
            releaseArms(false, true);
            auto result = std::make_shared<MovePose::Result>();
            result->success = ACTION_FAIL;
            result->error_message =
                std::string("Failed to start right MovePose worker: ") + error.what();
            goal_handle->abort(result);
        }
    }
}

void DualArmModule::execution_loop_move_left_pose(
    std::shared_ptr<GoalHandleMovePose> goal_handle,
    geometry_msgs::msg::PoseStamped planning_target)
{
    motion_control_pkg::execution::ScopedBusyFlagReset busy_reset(
        &is_executing_left_arm_);
    execution_loop_move_pose_impl(
        goal_handle, motion_control_pkg::sdk::ArmChannel::kLeft,
        planning_target);
}

void DualArmModule::execution_loop_move_right_pose(
    std::shared_ptr<GoalHandleMovePose> goal_handle,
    geometry_msgs::msg::PoseStamped planning_target)
{
    motion_control_pkg::execution::ScopedBusyFlagReset busy_reset(
        &is_executing_right_arm_);
    execution_loop_move_pose_impl(
        goal_handle, motion_control_pkg::sdk::ArmChannel::kRight,
        planning_target);
}

void DualArmModule::start_execution_move_joints(
    std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    const bool need_left = goal->arm_id == ARM_LEFT;
    const bool need_right = goal->arm_id == ARM_RIGHT;
    std::string claim_error;
    if (!tryClaimArms(need_left, need_right, claim_error)) {
        auto result = std::make_shared<MoveJoints::Result>();
        result->success = ACTION_FAIL;
        result->error_message = claim_error;
        goal_handle->abort(result);
        return;
    }
    if (goal->arm_id == ARM_LEFT) {
        if (execution_thread_move_left_joint_.joinable()) {
            execution_thread_move_left_joint_.join();
        }
        is_executing_left_arm_.store(true);
        try {
            execution_thread_move_left_joint_ = std::thread(
                [this, goal_handle]() mutable {
                    execution_loop_move_left_joint(goal_handle);
                });
        } catch (const std::exception& error) {
            releaseArms(true, false);
            auto result = std::make_shared<MoveJoints::Result>();
            result->success = ACTION_FAIL;
            result->error_message =
                std::string("Failed to start left MoveJoints worker: ") + error.what();
            goal_handle->abort(result);
        }
    } else if (goal->arm_id == ARM_RIGHT) {
        if (execution_thread_move_right_joint_.joinable()) {
            execution_thread_move_right_joint_.join();
        }
        is_executing_right_arm_.store(true);
        try {
            execution_thread_move_right_joint_ = std::thread(
                [this, goal_handle]() mutable {
                    execution_loop_move_right_joint(goal_handle);
                });
        } catch (const std::exception& error) {
            releaseArms(false, true);
            auto result = std::make_shared<MoveJoints::Result>();
            result->success = ACTION_FAIL;
            result->error_message =
                std::string("Failed to start right MoveJoints worker: ") + error.what();
            goal_handle->abort(result);
        }
    }
}

void DualArmModule::execution_loop_move_left_joint(
    std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
    motion_control_pkg::execution::ScopedBusyFlagReset busy_reset(
        &is_executing_left_arm_);
    execution_loop_move_joint_impl(
        goal_handle, motion_control_pkg::sdk::ArmChannel::kLeft);
}

void DualArmModule::execution_loop_move_right_joint(
    std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
    motion_control_pkg::execution::ScopedBusyFlagReset busy_reset(
        &is_executing_right_arm_);
    execution_loop_move_joint_impl(
        goal_handle, motion_control_pkg::sdk::ArmChannel::kRight);
}

void DualArmModule::start_execution_move_dual_arm_joints(
    std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
    std::string claim_error;
    if (!tryClaimArms(true, true, claim_error)) {
        auto result = std::make_shared<MoveJoints::Result>();
        result->success = false;
        result->error_message = claim_error;
        goal_handle->abort(result);
        return;
    }
    if (execution_thread_move_dual_joints_.joinable()) {
        execution_thread_move_dual_joints_.join();
    }
    is_executing_left_arm_.store(true);
    is_executing_right_arm_.store(true);
    try {
        execution_thread_move_dual_joints_ = std::thread(
            [this, goal_handle]() mutable {
                execution_loop_move_dual_arm_joints(goal_handle);
            });
    } catch (const std::exception& error) {
        releaseArms(true, true);
        auto result = std::make_shared<MoveJoints::Result>();
        result->success = false;
        result->error_message =
            std::string("Failed to start dual-arm worker: ") + error.what();
        goal_handle->abort(result);
    }
}

void DualArmModule::execution_loop_move_dual_arm_joints(
    std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
    motion_control_pkg::execution::ScopedBusyFlagReset busy_reset(
        &is_executing_left_arm_, &is_executing_right_arm_);
    motion_control_pkg::execution::ActionTerminalGuard<
        GoalHandleMoveJoints, MoveJoints::Result> terminal(
        goal_handle);
    const auto started_at = std::chrono::steady_clock::now();
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveJoints::Result>();
    auto feedback = std::make_shared<MoveJoints::Feedback>();
    result->success = false;
    result->left_final_distance_error_rad =
        std::numeric_limits<double>::quiet_NaN();
    result->right_final_distance_error_rad =
        std::numeric_limits<double>::quiet_NaN();
    result->final_distance_error =
        std::numeric_limits<double>::quiet_NaN();
    result->completion_time_skew_sec =
        std::numeric_limits<double>::quiet_NaN();

    const auto update_time = [&]() {
        result->execution_time = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started_at).count();
    };
    const auto abort_goal = [&](const std::string& message) {
        result->success = false;
        result->error_message = message;
        update_time();
        RCLCPP_ERROR(
            rclcpp::get_logger("dual_arm_module"),
            "Dual-arm MoveJoints failed: %s",
            result->error_message.c_str());
        terminal.abort(result);
    };
    const auto cancel_goal = [&](const std::string& message) {
        result->success = false;
        result->error_message = message;
        update_time();
        terminal.cancel(result);
    };
    const auto publish_feedback = [this, &goal_handle, &feedback](
        std::uint8_t status, float left, float right,
        const std::string& step) {
        feedback->dual_task_status = status;
        feedback->task_status = "RUNNING";
        feedback->left_progress_rate = std::clamp(left, 0.0F, 100.0F);
        feedback->right_progress_rate = std::clamp(right, 0.0F, 100.0F);
        feedback->progress_rate =
            std::min(feedback->left_progress_rate,
            feedback->right_progress_rate);
        feedback->current_step_info = step;
        if (auto node = lockNode(__func__)) {
            feedback->header.stamp = node->get_clock()->now();
        }
        goal_handle->publish_feedback(feedback);
    };

    if (arm_configuration_ !=
        motion_control_pkg::sdk::ActiveArmConfiguration::kDualArm ||
        goal->synchronization_mode !=
        MoveJoints::Goal::SYNC_START_AND_FINISH)
    {
        abort_goal(
            "MoveJoints requires dual_arm configuration and "
            "SYNC_START_AND_FINISH mode");
        return;
    }
    const auto speed_validation =
        motion_control_pkg::motion::validateSpeedScale(goal->speed);
    if (!speed_validation.valid || goal->timeout_ms <= 0) {
        abort_goal(speed_validation.valid ?
            "MoveJoints timeout_ms must be positive" :
            speed_validation.error_message);
        return;
    }
    const auto left_goal_validation =
        motion_control_pkg::execution::validateAndOrderJointGoal(
        motion_control_pkg::sdk::ArmChannel::kLeft,
        std::vector<std::string>(goal->left_joint_name.begin(),
        goal->left_joint_name.end()),
        std::vector<double>(goal->left_position.begin(),
        goal->left_position.end()));
    const auto right_goal_validation =
        motion_control_pkg::execution::validateAndOrderJointGoal(
        motion_control_pkg::sdk::ArmChannel::kRight,
        std::vector<std::string>(goal->right_joint_name.begin(),
        goal->right_joint_name.end()),
        std::vector<double>(goal->right_position.begin(),
        goal->right_position.end()));
    if (!left_goal_validation.valid || !right_goal_validation.valid) {
        abort_goal(left_goal_validation.valid ?
            right_goal_validation.error_message :
            left_goal_validation.error_message);
        return;
    }
    if (!init_robot_link_ || !l_init_arm_params_) {
        abort_goal("Dual-arm robot link or kinematics is not initialized");
        return;
    }
    if (goal_handle->is_canceling()) {
        cancel_goal("MoveJoints canceled before planning");
        return;
    }
    motion_control_pkg::execution::SteadyDeadline deadline(
        std::chrono::milliseconds(goal->timeout_ms));
    publish_feedback(
        MoveJoints::Feedback::STATUS_PLANNING, 0.0F, 0.0F,
        "reading one synchronized dual-arm state snapshot");

    motion_control_pkg::sdk::DualArmDeviceState initial_state;
    const auto state_result = sdk_command_gate_->readState(initial_state);
    if (!state_result) {
        abort_goal(state_result.error_message);
        return;
    }
    for (std::size_t arm = 0; arm < 2U; ++arm) {
        const auto& arm_state = initial_state.arms[arm];
        if (arm_state.error_code != 0 || arm_state.current_state == 100 ||
            !arm_state.low_speed)
        {
            abort_goal(
                std::string(arm == 0U ? "left" : "right") +
                " arm is not stationary and error-free before planning");
            return;
        }
    }
    {
        std::lock_guard<std::mutex> lock(arm_status_mtx_);
        dual_arm_state_ = initial_state;
    }
    std::vector<double> left_current;
    std::vector<double> right_current;
    std::string model_error;
    if (!updateArmModelState(
        initial_state, left_current, right_current, model_error))
    {
        abort_goal(model_error);
        return;
    }
    std::vector<double> full_start_q;
    if (!miracle_algo_->getFullJointState(full_start_q)) {
        abort_goal("Failed to obtain the full Pinocchio start state");
        return;
    }
    auto collision_scene = motion_control_pkg::motion::makeCollisionScene(
        *miracle_algo_, goal->enable_collision_check, full_start_q,
        sensor_msgs::msg::PointCloud2{}, model_error);
    if (!collision_scene) { abort_goal(model_error); return; }


    const motion_control_pkg::motion::JointVector left_target =
        left_goal_validation.ordered_positions;
    const motion_control_pkg::motion::JointVector right_target =
        right_goal_validation.ordered_positions;
    motion_control_pkg::motion::JointVector left_start{};
    motion_control_pkg::motion::JointVector right_start{};
    std::copy(left_current.begin(), left_current.end(), left_start.begin());
    std::copy(right_current.begin(), right_current.end(), right_start.begin());

    struct ArmPlanningData
    {
        motion_control_pkg::motion::PositionLimitIntersection limits;
        motion_control_pkg::motion::ScaledMotionLimits motion;
        motion_control_pkg::motion::CcsJoint67Table joint67{};
    };
    std::array<ArmPlanningData, 2> arm_data;
    const std::array<motion_control_pkg::motion::JointVector, 2> starts{
        left_start, right_start};
    const std::array<motion_control_pkg::motion::JointVector, 2> targets{
        left_target, right_target};
    for (std::size_t arm = 0; arm < arm_data.size(); ++arm) {
        std::vector<double> model_lower(arm_dof_);
        std::vector<double> model_upper(arm_dof_);
        if (!miracle_algo_->getChainJointsLimt(
            static_cast<int>(arm), model_lower, model_upper))
        {
            abort_goal("Failed to obtain dual-arm Pinocchio joint limits");
            return;
        }
        arm_data[arm].limits =
            motion_control_pkg::motion::intersectPositionLimits(
            arm == 0U ? l_q_lower_limits_ : r_q_lower_limits_,
            arm == 0U ? l_q_upper_limits_ : r_q_upper_limits_,
            model_lower, model_upper);
        arm_data[arm].motion = motion_control_pkg::motion::scaleMotionLimits(
            arm == 0U ? l_v_limit_ : r_v_limit_,
            arm == 0U ? l_a_limit_ : r_a_limit_,
            arm == 0U ? l_jerk_limit_ : r_jerk_limit_, goal->speed, goal->acceleration_scale, goal->jerk_scale);
        if (!arm_data[arm].limits.valid || !arm_data[arm].motion.valid) {
            abort_goal(arm_data[arm].limits.valid ?
                arm_data[arm].motion.error_message :
                arm_data[arm].limits.error_message);
            return;
        }
        for (std::size_t row = 0; row < arm_data[arm].joint67.size(); ++row) {
            for (std::size_t column = 0;
                column < arm_data[arm].joint67[row].size(); ++column)
            {
                arm_data[arm].joint67[row][column] = bd_[arm][row][column];
            }
        }
        const auto start_check =
            motion_control_pkg::motion::validateJointTarget(
            starts[arm], arm_data[arm].limits.lower,
            arm_data[arm].limits.upper, arm_data[arm].joint67);
        const auto target_check =
            motion_control_pkg::motion::validateJointTarget(
            targets[arm], arm_data[arm].limits.lower,
            arm_data[arm].limits.upper, arm_data[arm].joint67);
        if (!start_check.valid || !target_check.valid) {
            abort_goal(
                std::string(arm == 0U ? "left" : "right") +
                " arm joint constraint failed: " +
                (start_check.valid ? target_check.error_message :
                start_check.error_message));
            return;
        }
    }

    std::vector<std::vector<double>> dual_path;
    if (goal->enable_collision_check) {
        std::vector<double> dual_goal;
        dual_goal.insert(dual_goal.end(), left_target.begin(), left_target.end());
        dual_goal.insert(dual_goal.end(), right_target.begin(), right_target.end());
        std::vector<std::vector<double>> full_path;
        if (!miracle_algo_->planDualArmJointSpace(
            full_start_q, dual_goal, full_path))
        {
            abort_goal(
                "Existing Pinocchio/Coal/OMPL dual-arm collision planner failed");
            return;
        }
        dual_path.reserve(full_path.size());
        for (const auto& full_q : full_path) {
            std::vector<double> left;
            std::vector<double> right;
            if (!miracle_algo_->extractSingleArmJoints(true, full_q, left) ||
                !miracle_algo_->extractSingleArmJoints(false, full_q, right))
            {
                abort_goal("Dual-arm collision planner returned an invalid waypoint");
                return;
            }
            left.insert(left.end(), right.begin(), right.end());
            dual_path.push_back(std::move(left));
        }
    } else {
        dual_path = {
            std::vector<double>(left_start.begin(), left_start.end()),
            std::vector<double>(left_target.begin(), left_target.end())};
        dual_path[0].insert(
            dual_path[0].end(), right_start.begin(), right_start.end());
        dual_path[1].insert(
            dual_path[1].end(), right_target.begin(), right_target.end());
    }
    if (dual_path.size() < 2U) {
        abort_goal("Dual-arm planner returned fewer than two waypoints");
        return;
    }

    std::vector<double> combined_velocity;
    std::vector<double> combined_acceleration;
    std::vector<double> combined_jerk;
    for (const auto& data : arm_data) {
        combined_velocity.insert(
            combined_velocity.end(),
            data.motion.velocity.begin(), data.motion.velocity.end());
        combined_acceleration.insert(
            combined_acceleration.end(),
            data.motion.acceleration.begin(), data.motion.acceleration.end());
        combined_jerk.insert(
            combined_jerk.end(),
            data.motion.jerk.begin(), data.motion.jerk.end());
    }
    std::vector<std::vector<double>> planned_positions;
    std::string planning_error;
    const bool planned = dual_path.size() == 2U ?
        planRuckigSamples(
            *miracle_algo_, dual_path.front(), dual_path.back(),
            combined_velocity, combined_acceleration, combined_jerk,
            planned_positions, planning_error) :
        planToppraSamples(
            *miracle_algo_, dual_path, combined_velocity, combined_acceleration, combined_jerk,
            planned_positions, planning_error);
    if (!planned || planned_positions.empty()) {
        abort_goal("Dual-arm 14-DOF time parameterization failed: " +
            planning_error);
        return;
    }

    std::array<std::vector<std::vector<double>>, 2> trajectories;
    for (const auto& point : planned_positions) {
        if (point.size() != 2U * static_cast<std::size_t>(arm_dof_)) {
            abort_goal("Dual-arm trajectory sample does not contain 14 joints");
            return;
        }
        trajectories[0].emplace_back(point.begin(), point.begin() + arm_dof_);
        trajectories[1].emplace_back(point.begin() + arm_dof_, point.end());
    }
    for (std::size_t arm = 0; arm < trajectories.size(); ++arm) {
        const auto validation =
            motion_control_pkg::motion::validateTrajectoryPositions(
            trajectories[arm], arm_data[arm].limits.lower,
            arm_data[arm].limits.upper, arm_data[arm].joint67);
        if (!validation.valid) {
            abort_goal(
                std::string(arm == 0U ? "left" : "right") +
                " synchronized trajectory is invalid: " +
                validation.error_message);
            return;
        }
    }

    // OMPL 路点经 TOPPRA 参数化后，再用同一个 Pinocchio/Coal 接口检查
    // 每个 4 ms 联合采样点；不引入第二套防碰撞算法。
    if (goal->enable_collision_check) {
        bool collision_free = true;
        for (std::size_t sample = 0;
            sample < trajectories[0].size(); ++sample)
        {
            if (miracle_algo_->updateChainJointsAngle(
                "left_arm", trajectories[0][sample]) != 0 ||
                miracle_algo_->updateChainJointsAngle(
                "right_arm", trajectories[1][sample]) != 0)
            {
                collision_free = false;
                break;
            }
            std::vector<double> full_sample;
            if (!miracle_algo_->getFullJointState(full_sample) ||
                !miracle_algo_->isFullQValid(
                miracle_algo_->toEigenConfig(full_sample)))
            {
                collision_free = false;
                break;
            }
        }
        (void)miracle_algo_->updateChainJointsAngle("left_arm", left_current);
        (void)miracle_algo_->updateChainJointsAngle("right_arm", right_current);
        if (!collision_free) {
            abort_goal(
                "Pinocchio/Coal rejected a time-parameterized dual-arm sample");
            return;
        }
    }

    collision_scene.reset();
    std::vector<motion_control_pkg::execution::JointCommand> left_commands;
    std::vector<motion_control_pkg::execution::JointCommand> right_commands;
    std::string conversion_error;
    if (!motion_control_pkg::motion::trajectoryToVendorDegrees(
        trajectories[0], left_commands, conversion_error) ||
        !motion_control_pkg::motion::trajectoryToVendorDegrees(
        trajectories[1], right_commands, conversion_error))
    {
        abort_goal(conversion_error);
        return;
    }
    if (goal_handle->is_canceling()) {
        cancel_goal("MoveJoints canceled before atomic submission");
        return;
    }
    if (deadline.expired()) {
        abort_goal("MoveJoints timed out during planning");
        return;
    }
    publish_feedback(
        MoveJoints::Feedback::STATUS_EXECUTING, 15.0F, 15.0F,
        goal->control_mode == MoveJoints::Goal::CONTROL_IMPEDANCE ?
        "switching A/B to joint impedance mode" :
        "switching A/B to position mode");
    const auto mode_result = configureGoalControlMode(
        goal->control_mode,
        motion_control_pkg::sdk::ArmControlMode::kJointImpedance,
        {motion_control_pkg::sdk::ArmChannel::kLeft,
         motion_control_pkg::sdk::ArmChannel::kRight});
    if (!mode_result) {
        abort_goal(mode_result.error_message);
        return;
    }
    if (goal_handle->is_canceling()) {
        cancel_goal("MoveJoints canceled during control-mode transition");
        return;
    }
    if (deadline.expired()) {
        abort_goal("MoveJoints timed out during control-mode transition");
        return;
    }
    publish_feedback(
        MoveJoints::Feedback::STATUS_EXECUTING, 20.0F, 20.0F,
        "submitting synchronized A/B batches at 4 ms");
    if (!streamDualArmTrajectory(
        *sdk_command_gate_, goal_handle, left_commands, right_commands, deadline,
        "MoveJoints", MoveJoints::Feedback::STATUS_EXECUTING,
        "streaming one atomic dual-arm batch per cycle",
        publish_feedback, abort_goal, cancel_goal))
    {
        return;
    }

    publish_feedback(
        MoveJoints::Feedback::STATUS_WAITING_FOR_COMPLETION,
        95.0F, 95.0F, "waiting for both arms to report low speed");
    std::array<std::optional<std::chrono::steady_clock::time_point>, 2>
        completed_at;
    while (!completed_at[0] || !completed_at[1]) {
        if (goal_handle->is_canceling()) {
            cancel_goal(
                "MoveJoints canceled during completion wait; no "
                "physical emergency stop was issued");
            return;
        }
        if (deadline.expired()) {
            abort_goal("MoveJoints completion wait timed out");
            return;
        }
        motion_control_pkg::sdk::DualArmDeviceState completion;
        const auto completion_result = sdk_command_gate_->readState(completion);
        if (!completion_result) {
            abort_goal(completion_result.error_message);
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        for (std::size_t arm = 0; arm < 2U; ++arm) {
            if (completion.arms[arm].error_code != 0 ||
                completion.arms[arm].current_state == 100)
            {
                abort_goal(
                    std::string(arm == 0U ? "left" : "right") +
                    " arm entered an SDK error during completion wait");
                return;
            }
            if (!completed_at[arm] && completion.arms[arm].low_speed) {
                completed_at[arm] = now;
            }
        }
        result->left_final_distance_error_rad =
            motion_control_pkg::motion::maximumJointErrorRadians(
            completion.arms[0].joint_position, left_target);
        result->right_final_distance_error_rad =
            motion_control_pkg::motion::maximumJointErrorRadians(
            completion.arms[1].joint_position, right_target);
        if (!completed_at[0] || !completed_at[1]) {
            std::this_thread::sleep_for(
                motion_control_pkg::execution::kDualArmControlPeriod);
        }
    }
    result->completion_time_skew_sec = std::chrono::duration<double>(
        *completed_at[0] - *completed_at[1]).count();
    result->completion_time_skew_sec =
        std::abs(result->completion_time_skew_sec);
    result->final_distance_error = std::max(
        result->left_final_distance_error_rad,
        result->right_final_distance_error_rad);
    result->success = true;
    result->error_message.clear();
    update_time();
    publish_feedback(
        MoveJoints::Feedback::STATUS_WAITING_FOR_COMPLETION,
        100.0F, 100.0F, "dual-arm joint motion completed");
    terminal.succeed(result);
}

void DualArmModule::start_execution_move_dual_arm_pose(
    std::shared_ptr<GoalHandleMovePose> goal_handle,
    geometry_msgs::msg::PoseStamped left_planning_target,
    geometry_msgs::msg::PoseStamped right_planning_target,
    std::chrono::steady_clock::time_point request_started_at)
{
    std::string claim_error;
    if (!tryClaimArms(true, true, claim_error)) {
        auto result = std::make_shared<MovePose::Result>();
        result->success = false;
        result->error_message = claim_error;
        goal_handle->abort(result);
        return;
    }
    if (execution_thread_move_dual_pose_.joinable()) {
        execution_thread_move_dual_pose_.join();
    }
    is_executing_left_arm_.store(true);
    is_executing_right_arm_.store(true);
    try {
        execution_thread_move_dual_pose_ = std::thread(
            [this, goal_handle,
            left_planning_target = std::move(left_planning_target),
            right_planning_target = std::move(right_planning_target),
            request_started_at]() mutable {
                execution_loop_move_dual_arm_pose(
                    goal_handle, std::move(left_planning_target),
                    std::move(right_planning_target), request_started_at);
            });
    } catch (const std::exception& error) {
        releaseArms(true, true);
        auto result = std::make_shared<MovePose::Result>();
        result->success = false;
        result->error_message =
            std::string("Failed to start dual-arm pose worker: ") +
            error.what();
        goal_handle->abort(result);
    }
}

void DualArmModule::execution_loop_move_dual_arm_pose(
    std::shared_ptr<GoalHandleMovePose> goal_handle,
    geometry_msgs::msg::PoseStamped left_planning_target,
    geometry_msgs::msg::PoseStamped right_planning_target,
    std::chrono::steady_clock::time_point request_started_at)
{
    motion_control_pkg::execution::ScopedBusyFlagReset busy_reset(
        &is_executing_left_arm_, &is_executing_right_arm_);
    motion_control_pkg::execution::ActionTerminalGuard<
        GoalHandleMovePose, MovePose::Result> terminal(
        goal_handle);
    const auto started_at = request_started_at;
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MovePose::Result>();
    auto feedback = std::make_shared<MovePose::Feedback>();
    result->success = false;
    result->left_final_distance_error_m =
        std::numeric_limits<double>::quiet_NaN();
    result->right_final_distance_error_m =
        std::numeric_limits<double>::quiet_NaN();
    result->final_distance_error =
        std::numeric_limits<double>::quiet_NaN();
    result->completion_time_skew_sec =
        std::numeric_limits<double>::quiet_NaN();

    const auto update_time = [&]() {
        result->execution_time = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started_at).count();
    };
    const auto abort_goal = [&](const std::string& message) {
        result->success = false;
        result->error_message = message;
        update_time();
        RCLCPP_ERROR(
            rclcpp::get_logger("dual_arm_module"),
            "Dual-arm MovePose failed: %s",
            result->error_message.c_str());
        terminal.abort(result);
    };
    const auto cancel_goal = [&](const std::string& message) {
        result->success = false;
        result->error_message = message;
        update_time();
        terminal.cancel(result);
    };
    const auto publish_feedback = [this, &goal_handle, &feedback](
        std::uint8_t status, float left, float right,
        const std::string& step) {
        feedback->dual_task_status = status;
        feedback->task_status = "RUNNING";
        feedback->left_progress_rate = std::clamp(left, 0.0F, 100.0F);
        feedback->right_progress_rate = std::clamp(right, 0.0F, 100.0F);
        feedback->progress_rate = std::min(
            feedback->left_progress_rate, feedback->right_progress_rate);
        feedback->current_step_info = step;
        if (auto node = lockNode(__func__)) {
            feedback->header.stamp = node->get_clock()->now();
        }
        goal_handle->publish_feedback(feedback);
    };

    if (arm_configuration_ !=
        motion_control_pkg::sdk::ActiveArmConfiguration::kDualArm ||
        goal->synchronization_mode !=
        MovePose::Goal::SYNC_START_AND_FINISH ||
        goal->timeout_ms <= 0)
    {
        abort_goal("MovePose goal violates the synchronized dual-arm contract");
        return;
    }
    const auto speed_validation =
        motion_control_pkg::motion::validateSpeedScale(goal->speed);
    if (!speed_validation.valid) {
        abort_goal(speed_validation.error_message);
        return;
    }
    
    if (!init_robot_link_ || !l_init_arm_params_) {
        abort_goal("Dual-arm robot link or kinematics is not initialized");
        return;
    }
    if (goal_handle->is_canceling()) {
        cancel_goal("MovePose canceled before planning");
        return;
    }
    const auto requested_timeout = std::chrono::milliseconds(goal->timeout_ms);
    const auto elapsed_before_planning =
        std::chrono::steady_clock::now() - started_at;
    if (elapsed_before_planning >= requested_timeout) {
        abort_goal("MovePose overall timeout expired before planning");
        return;
    }
    motion_control_pkg::execution::SteadyDeadline deadline(
        requested_timeout - elapsed_before_planning);
    publish_feedback(
        MovePose::Feedback::STATUS_PLANNING, 0.0F, 0.0F,
        "reading one synchronized dual-arm state snapshot");

    motion_control_pkg::sdk::DualArmDeviceState initial_state;
    const auto state_result = sdk_command_gate_->readState(initial_state);
    if (!state_result) {
        abort_goal(state_result.error_message);
        return;
    }
    for (std::size_t arm = 0; arm < 2U; ++arm) {
        const auto& arm_state = initial_state.arms[arm];
        if (arm_state.error_code != 0 || arm_state.current_state == 100 ||
            !arm_state.low_speed)
        {
            abort_goal(
                std::string(arm == 0U ? "left" : "right") +
                " arm is not stationary and error-free before pose planning");
            return;
        }
    }
    {
        std::lock_guard<std::mutex> lock(arm_status_mtx_);
        dual_arm_state_ = initial_state;
    }
    std::vector<double> left_current;
    std::vector<double> right_current;
    std::string model_error;
    if (!updateArmModelState(
        initial_state, left_current, right_current, model_error))
    {
        abort_goal(model_error);
        return;
    }
    std::vector<double> full_start_q;
    if (!miracle_algo_->getFullJointState(full_start_q)) {
        abort_goal("Failed to obtain the full Pinocchio start state");
        return;
    }
    auto collision_scene = motion_control_pkg::motion::makeCollisionScene(
        *miracle_algo_, goal->enable_collision_check, full_start_q,
        goal->env_point_cloud, model_error);
    if (!collision_scene) { abort_goal(model_error); return; }


    geometry_msgs::msg::PoseStamped normalized_left;
    geometry_msgs::msg::PoseStamped normalized_right;
    std::string conversion_error;
    if (!normalizePoseTargetToPlanningFrame(
        left_planning_target, normalized_left, conversion_error))
    {
        abort_goal("left target: " + conversion_error);
        return;
    }
    if (!normalizePoseTargetToPlanningFrame(
        right_planning_target, normalized_right, conversion_error))
    {
        abort_goal("right target: " + conversion_error);
        return;
    }

    //加上偏移量
    normalized_left.pose.position.x +=  goal->left_bound_size[0];
    normalized_left.pose.position.y +=  goal->left_bound_size[1];
    normalized_left.pose.position.z +=  goal->left_bound_size[2];
    normalized_right.pose.position.x +=  goal->right_bound_size[0];
    normalized_right.pose.position.y +=  goal->right_bound_size[1];
    normalized_right.pose.position.z +=  goal->right_bound_size[2];

    const auto& effective_left = normalized_left;
    const auto& effective_right = normalized_right;

    const Eigen::Isometry3d left_planning_to_end =
        motion_control_pkg::kinematics::targetTcpToModelEndPose(
        effective_left.pose, left_model_end_to_tool_tcp_);
    const Eigen::Isometry3d right_planning_to_end =
        motion_control_pkg::kinematics::targetTcpToModelEndPose(
        effective_right.pose, right_model_end_to_tool_tcp_);
    Eigen::Isometry3d left_root_to_end = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d right_root_to_end = Eigen::Isometry3d::Identity();
    if (!miracle_algo_->transformPoseToModelRoot(
        planning_frame_, full_start_q, left_planning_to_end,
        left_root_to_end) ||
        !miracle_algo_->transformPoseToModelRoot(
        planning_frame_, full_start_q, right_planning_to_end,
        right_root_to_end))
    {
        abort_goal("Failed to transform dual TCP targets into the model root frame");
        return;
    }

    std::vector<double> dual_goal;
    std::vector<std::vector<double>> full_path;
    std::vector<std::vector<double>> planned_positions;
    
    struct ArmPlanningData
    {
        motion_control_pkg::motion::PositionLimitIntersection limits;
        motion_control_pkg::motion::ScaledMotionLimits motion;
        motion_control_pkg::motion::CcsJoint67Table joint67{};
    };
    std::array<ArmPlanningData, 2> arm_data;

    auto init_arm_planning_data = [&](std::size_t arm_index) -> bool {
        std::vector<double> model_lower(arm_dof_);
        std::vector<double> model_upper(arm_dof_);
        if (!miracle_algo_->getChainJointsLimt(
            static_cast<int>(arm_index), model_lower, model_upper))
        {
            return false;
        }
        arm_data[arm_index].limits =
            motion_control_pkg::motion::intersectPositionLimits(
            arm_index == 0U ? l_q_lower_limits_ : r_q_lower_limits_,
            arm_index == 0U ? l_q_upper_limits_ : r_q_upper_limits_,
            model_lower, model_upper);
        arm_data[arm_index].motion =
            motion_control_pkg::motion::scaleMotionLimits(
            arm_index == 0U ? l_v_limit_ : r_v_limit_,
            arm_index == 0U ? l_a_limit_ : r_a_limit_,
            arm_index == 0U ? l_jerk_limit_ : r_jerk_limit_, goal->speed, goal->acceleration_scale, goal->jerk_scale);
        if (!arm_data[arm_index].limits.valid || !arm_data[arm_index].motion.valid) {
            return false;
        }
        for (std::size_t row = 0; row < arm_data[arm_index].joint67.size(); ++row) {
            for (std::size_t column = 0;
                column < arm_data[arm_index].joint67[row].size(); ++column)
            {
                arm_data[arm_index].joint67[row][column] = bd_[arm_index][row][column];
            }
        }
        return true;
    };

    if (goal->enable_move_line)
    {
        if (!init_arm_planning_data(0) || !init_arm_planning_data(1)) {
            abort_goal("Failed to initialize dual MoveL limits"); return;
        }
        std::vector<double> velocity, acceleration, jerk;
        for (const auto& data : arm_data) {
            velocity.insert(velocity.end(), data.motion.velocity.begin(), data.motion.velocity.end());
            acceleration.insert(acceleration.end(), data.motion.acceleration.begin(), data.motion.acceleration.end());
            jerk.insert(jerk.end(), data.motion.jerk.begin(), data.motion.jerk.end());
        }
        double translation_step = motion_control_pkg::motion::kCartesianIkTranslationStepMeters;
        double rotation_step = motion_control_pkg::motion::kCartesianIkRotationStepRadians;
        bool valid = false;
        std::string error;
        for (int attempt = 0; attempt < 3 && !valid; ++attempt) {
            std::array<std::vector<std::vector<double>>, 2> coarse;
            std::array<Eigen::Isometry3d, 2> starts;
            const std::array<Eigen::Isometry3d, 2> targets{left_root_to_end, right_root_to_end};
            bool ik_ok = true;
            for (std::size_t arm = 0; arm < 2; ++arm) {
                double ik_seconds = 0;
                const bool ok = miracle_algo_->planSingleArmCartesianIkWaypoints(arm == 0, full_start_q,
                    targets[arm], translation_step, rotation_step, coarse[arm], starts[arm], &ik_seconds);
                result->ik_time += ik_seconds;
                ik_ok = ik_ok && ok;
            }
            if (!ik_ok) {error = "dual MoveL continuous IK failed";}
            else {
                // Match normalized Cartesian progress before one 14-DOF
                // parameterization. Neither arm is padded with an early stop.
                const std::size_t count = std::max(coarse[0].size(), coarse[1].size());
                std::vector<std::vector<double>> combined;
                for (std::size_t i = 0; i < count; ++i) {
                    std::vector<double> q;
                    for (std::size_t arm = 0; arm < 2; ++arm) {
                        const double x = count <= 1 ? 0.0 :
                            static_cast<double>(i)*(coarse[arm].size()-1)/(count-1);
                        const auto a = static_cast<std::size_t>(std::floor(x));
                        const auto b = std::min(a+1, coarse[arm].size()-1);
                        for (std::size_t j = 0; j < 7; ++j) {
                            q.push_back(coarse[arm][a][j]+(x-a)*(coarse[arm][b][j]-coarse[arm][a][j]));
                        }
                    }
                    combined.push_back(std::move(q));
                }
                valid = planToppraSamples(*miracle_algo_, combined, velocity, acceleration, jerk,
                    planned_positions, error, MiracleMAlgo::ToppraSolverMode::kStableCartesian,
                    static_cast<double>(goal->timeout_ms)/1000.0);
                for (std::size_t arm = 0; arm < 2 && valid; ++arm) {
                    std::vector<std::vector<double>> samples;
                    for (const auto& q : planned_positions) {
                        samples.emplace_back(q.begin()+arm*7, q.begin()+(arm+1)*7);
                    }
                    double position_error = 0, rotation_error = 0;
                    valid = miracle_algo_->validateSingleArmCartesianTrajectory(arm == 0, full_start_q,
                        starts[arm], targets[arm], samples,
                        motion_control_pkg::motion::kCartesianLinePositionToleranceMeters,
                        motion_control_pkg::motion::kCartesianLineOrientationToleranceRadians,
                        position_error, rotation_error, error);
                }
            }
            if (goal_handle->is_canceling()) {cancel_goal("Dual MoveL canceled during planning"); return;}
            if (deadline.expired()) {abort_goal("Dual MoveL timed out during planning"); return;}
            translation_step *= 0.5; rotation_step *= 0.5;
        }
        if (!valid) {abort_goal("Jerk-limited dual MoveL failed: " + error); return;}
    }
    else
    {
        if (goal->enable_collision_check) {
            if (!miracle_algo_->planDualArmCartesianGoal(
            full_start_q, left_root_to_end, right_root_to_end, full_path))
            {
                abort_goal(
                    "Existing dual-arm Pinocchio IK/Coal/OMPL pose planner failed");
                return;
            }

            if (full_path.empty() ||
                !miracle_algo_->extractSingleArmJoints(
                true, full_path.back(), dual_goal))
            {
                abort_goal("Dual-arm pose planner returned an invalid left goal");
                return;
            }
            std::vector<double> right_goal;
            if (!miracle_algo_->extractSingleArmJoints(
                false, full_path.back(), right_goal))
            {
                abort_goal("Dual-arm pose planner returned an invalid right goal");
                return;
            }
            dual_goal.insert(dual_goal.end(), right_goal.begin(), right_goal.end());
        } else if (!miracle_algo_->solveDualArmCartesianIk(
            full_start_q, left_root_to_end, right_root_to_end, dual_goal))
        {
            abort_goal("Existing Pinocchio dual-arm IK failed");
            return;
        }
        if (dual_goal.size() != 14U) {
            abort_goal("Dual-arm IK did not return fourteen arm joints");
            return;
        }

        motion_control_pkg::motion::JointVector left_start{};
        motion_control_pkg::motion::JointVector right_start{};
        motion_control_pkg::motion::JointVector left_target{};
        motion_control_pkg::motion::JointVector right_target{};
        std::copy(left_current.begin(), left_current.end(), left_start.begin());
        std::copy(right_current.begin(), right_current.end(), right_start.begin());
        std::copy(dual_goal.begin(), dual_goal.begin() + arm_dof_,
            left_target.begin());
        std::copy(dual_goal.begin() + arm_dof_, dual_goal.end(),
            right_target.begin());

        if (!init_arm_planning_data(0) || !init_arm_planning_data(1)) {
            abort_goal("Failed to initialize arm planning data");
            return;
        }

        const std::array<motion_control_pkg::motion::JointVector, 2> starts{
            left_start, right_start};
        const std::array<motion_control_pkg::motion::JointVector, 2> targets{
            left_target, right_target};
        
        for (std::size_t arm = 0; arm < arm_data.size(); ++arm) {
            const auto start_check =
                motion_control_pkg::motion::validateJointTarget(
                starts[arm], arm_data[arm].limits.lower,
                arm_data[arm].limits.upper, arm_data[arm].joint67);
            const auto target_check =
                motion_control_pkg::motion::validateJointTarget(
                targets[arm], arm_data[arm].limits.lower,
                arm_data[arm].limits.upper, arm_data[arm].joint67);
            if (!start_check.valid || !target_check.valid) {
                abort_goal(
                    std::string(arm == 0U ? "left" : "right") +
                    " arm IK constraint failed: " +
                    (start_check.valid ? target_check.error_message :
                    start_check.error_message));
                return;
            }
        }

        std::vector<std::vector<double>> dual_path;
        if (goal->enable_collision_check) {
            dual_path.reserve(full_path.size());
            for (const auto& full_q : full_path) {
                std::vector<double> left;
                std::vector<double> right;
                if (!miracle_algo_->extractSingleArmJoints(true, full_q, left) ||
                    !miracle_algo_->extractSingleArmJoints(false, full_q, right))
                {
                    abort_goal("Dual-arm pose path contains an invalid full-body waypoint");
                    return;
                }
                left.insert(left.end(), right.begin(), right.end());
                dual_path.push_back(std::move(left));
            }
        } else {
            dual_path = {
                std::vector<double>(left_start.begin(), left_start.end()),
                std::vector<double>(left_target.begin(), left_target.end())};
            dual_path[0].insert(
                dual_path[0].end(), right_start.begin(), right_start.end());
            dual_path[1].insert(
                dual_path[1].end(), right_target.begin(), right_target.end());
        }
        if (dual_path.size() < 2U) {
            abort_goal("Dual-arm pose planner returned fewer than two waypoints");
            return;
        }

        std::vector<double> combined_velocity;
        std::vector<double> combined_acceleration;
        std::vector<double> combined_jerk;
        for (const auto& data : arm_data) {
            combined_velocity.insert(
                combined_velocity.end(),
                data.motion.velocity.begin(), data.motion.velocity.end());
            combined_acceleration.insert(
                combined_acceleration.end(),
                data.motion.acceleration.begin(), data.motion.acceleration.end());
            combined_jerk.insert(
                combined_jerk.end(),
                data.motion.jerk.begin(), data.motion.jerk.end());
        }
        
        std::string planning_error;
        const bool planned = dual_path.size() == 2U ?
            planRuckigSamples(
                *miracle_algo_, dual_path.front(), dual_path.back(),
                combined_velocity, combined_acceleration, combined_jerk,
                planned_positions, planning_error) :
            planToppraSamples(
                *miracle_algo_, dual_path, combined_velocity, combined_acceleration, combined_jerk,
                planned_positions, planning_error);
        if (!planned || planned_positions.empty()) {
            abort_goal("Dual-arm pose 14-DOF time parameterization failed: " +
                planning_error);
            return;
        }
    }

    // 【关键防御】再次检查 planned_positions
    if (planned_positions.empty()) {
        abort_goal("Trajectory planning resulted in empty positions");
        return;
    }

    std::array<std::vector<std::vector<double>>, 2> trajectories;
    for (const auto& point : planned_positions) {
        // 防御性检查：point 大小必须正确
        if (point.size() != 2U * static_cast<std::size_t>(arm_dof_)) {
            abort_goal("Dual-arm pose trajectory sample does not contain 14 joints");
            return;
        }
        trajectories[0].emplace_back(point.begin(), point.begin() + arm_dof_);
        trajectories[1].emplace_back(point.begin() + arm_dof_, point.end());
    }
    
    // 如果 trajectories 为空（理论上不可能，因为上面检查了 planned_positions 非空），则报错
    if (trajectories[0].empty() || trajectories[1].empty()) {
        abort_goal("Generated trajectories are empty");
        return;
    }

    for (std::size_t arm = 0; arm < trajectories.size(); ++arm) {
        const auto validation =
            motion_control_pkg::motion::validateTrajectoryPositions(
            trajectories[arm], arm_data[arm].limits.lower,
            arm_data[arm].limits.upper, arm_data[arm].joint67);
        if (!validation.valid) {
            abort_goal(
                std::string(arm == 0U ? "left" : "right") +
                " pose trajectory is invalid: " + validation.error_message);
            return;
        }
    }
    

    if (goal->enable_collision_check) {
        bool collision_free = true;
        for (std::size_t sample = 0;
            sample < trajectories[0].size(); ++sample)
        {
            if (deadline.expired() ||
                miracle_algo_->updateChainJointsAngle(
                "left_arm", trajectories[0][sample]) != 0 ||
                miracle_algo_->updateChainJointsAngle(
                "right_arm", trajectories[1][sample]) != 0)
            {
                collision_free = false;
                break;
            }
            std::vector<double> full_sample;
            if (!miracle_algo_->getFullJointState(full_sample) ||
                !miracle_algo_->isFullQValid(
                miracle_algo_->toEigenConfig(full_sample)))
            {
                collision_free = false;
                break;
            }
        }
        (void)miracle_algo_->updateChainJointsAngle("left_arm", left_current);
        (void)miracle_algo_->updateChainJointsAngle("right_arm", right_current);
        if (!collision_free) {
            abort_goal(deadline.expired() ?
                "MovePose timed out during sampled collision checking" :
                "Pinocchio/Coal rejected a time-parameterized pose sample");
            return;
        }
    }

    collision_scene.reset();
    std::vector<motion_control_pkg::execution::JointCommand> left_commands;
    std::vector<motion_control_pkg::execution::JointCommand> right_commands;
    {
        std::string convert_msg;
        if (!motion_control_pkg::motion::trajectoryToVendorDegrees(
            trajectories[0], left_commands, convert_msg) ||
            !motion_control_pkg::motion::trajectoryToVendorDegrees(
            trajectories[1], right_commands, convert_msg))
        {
            abort_goal(convert_msg);
            return;
        }
    }
    if (goal_handle->is_canceling()) {
        cancel_goal("MovePose canceled before atomic submission");
        return;
    }
    if (deadline.expired()) {
        abort_goal("MovePose timed out during planning");
        return;
    }
    publish_feedback(
        MovePose::Feedback::STATUS_EXECUTING, 15.0F, 15.0F,
        goal->control_mode == MovePose::Goal::CONTROL_IMPEDANCE ?
        "switching A/B to Cartesian impedance mode" :
        "switching A/B to position mode");
    const auto mode_result = configureGoalControlMode(
        goal->control_mode,
        motion_control_pkg::sdk::ArmControlMode::kCartesianImpedance,
        {motion_control_pkg::sdk::ArmChannel::kLeft,
         motion_control_pkg::sdk::ArmChannel::kRight});
    if (!mode_result) {
        abort_goal(mode_result.error_message);
        return;
    }
    if (goal_handle->is_canceling()) {
        cancel_goal("MovePose canceled during control-mode transition");
        return;
    }
    if (deadline.expired()) {
        abort_goal("MovePose timed out during control-mode transition");
        return;
    }
    publish_feedback(
        MovePose::Feedback::STATUS_EXECUTING, 20.0F, 20.0F,
        "submitting synchronized pose trajectory batches at 4 ms");
    if (!streamDualArmTrajectory(
        *sdk_command_gate_, goal_handle, left_commands, right_commands, deadline,
        "MovePose", MovePose::Feedback::STATUS_EXECUTING,
        "streaming one atomic dual-arm pose batch per cycle",
        publish_feedback, abort_goal, cancel_goal))
    {
        return;
    }

    publish_feedback(
        MovePose::Feedback::STATUS_WAITING_FOR_COMPLETION,
        95.0F, 95.0F, "waiting for both TCP targets to settle");
    const Eigen::Isometry3d left_target_tcp =
        motion_control_pkg::kinematics::poseToIsometry(
        effective_left.pose);
    const Eigen::Isometry3d right_target_tcp =
        motion_control_pkg::kinematics::poseToIsometry(
        effective_right.pose);
    std::array<std::optional<std::chrono::steady_clock::time_point>, 2>
        completed_at;
    while (!completed_at[0] || !completed_at[1]) {
        if (goal_handle->is_canceling()) {
            cancel_goal(
                "MovePose canceled during completion wait; no "
                "physical emergency stop was issued");
            return;
        }
        if (deadline.expired()) {
            abort_goal("MovePose completion wait timed out");
            return;
        }
        motion_control_pkg::sdk::DualArmDeviceState completion;
        const auto completion_result = sdk_command_gate_->readState(completion);
        if (!completion_result) {
            abort_goal(completion_result.error_message);
            return;
        }
        for (std::size_t arm = 0; arm < 2U; ++arm) {
            if (completion.arms[arm].error_code != 0 ||
                completion.arms[arm].current_state == 100)
            {
                abort_goal(
                    std::string(arm == 0U ? "left" : "right") +
                    " arm entered an SDK error during pose completion");
                return;
            }
        }
        if (!updateArmModelState(
            completion, left_current, right_current, model_error))
        {
            abort_goal(model_error);
            return;
        }
        geometry_msgs::msg::Pose left_tcp_pose;
        geometry_msgs::msg::Pose right_tcp_pose;
        if (!computeToolTcpPose(
            "gripper_base_l_link", left_model_end_to_tool_tcp_,
            left_tcp_pose, model_error) ||
            !computeToolTcpPose(
            "gripper_base_r_link", right_model_end_to_tool_tcp_,
            right_tcp_pose, model_error))
        {
            abort_goal(model_error);
            return;
        }
        result->left_final_distance_error_m =
            (motion_control_pkg::kinematics::poseToIsometry(left_tcp_pose).
            translation() - left_target_tcp.translation()).norm();
        result->right_final_distance_error_m =
            (motion_control_pkg::kinematics::poseToIsometry(right_tcp_pose).
            translation() - right_target_tcp.translation()).norm();
        result->final_distance_error = std::max(
            result->left_final_distance_error_m,
            result->right_final_distance_error_m);
        if (!std::isfinite(result->left_final_distance_error_m) ||
            !std::isfinite(result->right_final_distance_error_m))
        {
            abort_goal("Dual-arm FK returned a non-finite TCP error");
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        for (std::size_t arm = 0; arm < 2U; ++arm) {
            if (!completed_at[arm] && completion.arms[arm].low_speed) {
                completed_at[arm] = now;
            }
        }
        if (!completed_at[0] || !completed_at[1]) {
            std::this_thread::sleep_for(
                motion_control_pkg::execution::kDualArmControlPeriod);
        }
    }
    result->completion_time_skew_sec = std::abs(
        std::chrono::duration<double>(
        *completed_at[0] - *completed_at[1]).count());
    result->success = true;
    result->error_message.clear();
    update_time();
    publish_feedback(
        MovePose::Feedback::STATUS_WAITING_FOR_COMPLETION,
        100.0F, 100.0F, "dual-arm TCP motion completed");
    terminal.succeed(result);
}

void DualArmModule::execution_loop_move_joint_impl(
    std::shared_ptr<GoalHandleMoveJoints> goal_handle,
    motion_control_pkg::sdk::ArmChannel channel)
{
    motion_control_pkg::execution::ActionTerminalGuard<
        GoalHandleMoveJoints, MoveJoints::Result> terminal(goal_handle);
    const auto start_time = std::chrono::steady_clock::now();
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveJoints::Feedback>();
    auto result = std::make_shared<MoveJoints::Result>();
    result->success = ACTION_FAIL;
    result->timing_breakdown_valid = true;
    result->ik_time = 0.0;
    result->planning_time = 0.0;
    result->motion_time = 0.0;
    result->final_distance_error =
        std::numeric_limits<double>::quiet_NaN();

    bool planning_active = false;
    bool motion_active = false;
    std::chrono::steady_clock::time_point planning_started_at;
    std::chrono::steady_clock::time_point motion_started_at;
    const auto update_phase_times = [&]() {
        const auto now = std::chrono::steady_clock::now();
        if (planning_active) {
            result->planning_time = std::chrono::duration<double>(
                now - planning_started_at).count();
        }
        if (motion_active) {
            result->motion_time = std::chrono::duration<double>(
                now - motion_started_at).count();
        }
    };
    const auto update_execution_time = [&]() {
        update_phase_times();
        result->execution_time = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time).count();
    };
    const auto abort_goal = [&](const std::string& message) {
        result->success = ACTION_FAIL;
        result->error_message = message;
        update_execution_time();
        RCLCPP_ERROR(
            rclcpp::get_logger("dual_arm_module"),
            "MoveJoints (%s arm) failed: %s",
            channel == motion_control_pkg::sdk::ArmChannel::kLeft ?
            "left" : "right",
            result->error_message.c_str());
        terminal.abort(result);
    };
    const auto cancel_goal = [&](const std::string& message) {
        result->success = ACTION_FAIL;
        result->error_message = message;
        update_execution_time();
        terminal.cancel(result);
    };
    const auto publish_feedback = [&goal_handle, &feedback, this](
        float progress, const std::string& step) {
        feedback->task_status = progress >= 100.0F ? "SUCCEEDED" : "RUNNING";
        feedback->progress_rate = std::clamp(progress, 0.0F, 100.0F);
        feedback->current_step_info = step;
        if (auto node = lockNode(__func__)) {
            feedback->header.stamp = node->get_clock()->now();
        }
        goal_handle->publish_feedback(feedback);
    };

    const auto expected_channel =
        motion_control_pkg::execution::armChannelFromId(goal->arm_id);
    if (!expected_channel || *expected_channel != channel) {
        abort_goal("MoveJoints requires exactly one matching arm");
        return;
    }

    const auto validation =
        motion_control_pkg::execution::validateAndOrderJointGoal(
        channel, goal->joint_name, goal->position);
    if (!validation.valid) {
        abort_goal(validation.error_message);
        return;
    }
    const auto& base_velocity =
        channel == motion_control_pkg::sdk::ArmChannel::kLeft ?
        l_v_limit_ : r_v_limit_;
    const auto& base_acceleration =
        channel == motion_control_pkg::sdk::ArmChannel::kLeft ?
        l_a_limit_ : r_a_limit_;
    const auto& base_jerk =
        channel == motion_control_pkg::sdk::ArmChannel::kLeft ?
        l_jerk_limit_ : r_jerk_limit_;
    const auto scaled_limits =
        motion_control_pkg::motion::scaleMotionLimits(
        base_velocity, base_acceleration, base_jerk,
        static_cast<double>(goal->speed), goal->acceleration_scale, goal->jerk_scale);
    if (!scaled_limits.valid) {
        abort_goal(scaled_limits.error_message);
        return;
    }
    if (goal_handle->is_canceling()) {
        cancel_goal("MoveJoints goal canceled before execution");
        return;
    }
    if (!init_robot_link_) {
        abort_goal("Robot link is not initialized");
        return;
    }

    const auto timeout = std::chrono::milliseconds(
        goal->timeout_ms > 0 ? goal->timeout_ms : 5000);
    motion_control_pkg::execution::SteadyDeadline deadline(timeout);
    const std::size_t arm_index = static_cast<std::size_t>(channel);
    const char* arm_name =
        channel == motion_control_pkg::sdk::ArmChannel::kLeft ?
        "left" : "right";

    motion_control_pkg::sdk::DualArmDeviceState initial_state;
    const auto initial_state_result =
        sdk_command_gate_->readState(initial_state);
    if (!initial_state_result) {
        abort_goal(initial_state_result.error_message);
        return;
    }
    {
        std::lock_guard<std::mutex> lock(arm_status_mtx_);
        dual_arm_state_ = initial_state;
    }
    const auto& selected_initial_state = initial_state.arms[arm_index];
    if (selected_initial_state.error_code != 0 ||
        selected_initial_state.current_state == 100)
    {
        abort_goal(
            std::string(arm_name) +
            " arm reports an SDK state error: error_code=" +
            std::to_string(selected_initial_state.error_code) +
            ", current_state=" +
            std::to_string(selected_initial_state.current_state));
        return;
    }
    if (!selected_initial_state.low_speed) {
        abort_goal(std::string(arm_name) +
            " arm is not stationary before MoveJoints planning");
        return;
    }

    std::vector<double> current_radians;
    std::string conversion_error;
    if (!motion_control_pkg::kinematics::degreesToRadians(
        selected_initial_state.joint_position, current_radians,
        conversion_error, std::string(arm_name) + " arm"))
    {
        abort_goal(conversion_error);
        return;
    }
    motion_control_pkg::motion::JointVector current_joint{};
    std::copy(current_radians.begin(), current_radians.end(),
        current_joint.begin());
    const motion_control_pkg::motion::JointVector target_joint =
        validation.ordered_positions;
    result->final_distance_error =
        motion_control_pkg::motion::maximumJointErrorRadians(
        selected_initial_state.joint_position, target_joint);
    publish_feedback(5.0F, std::string(arm_name) + " arm state acquired");

    // Pinocchio 负责模型关节限位，CCS 配置负责控制器软限位；实际目标取交集。
    std::vector<double> model_lower(motion_control_pkg::sdk::kArmDof);
    std::vector<double> model_upper(motion_control_pkg::sdk::kArmDof);
    if (!miracle_algo_->getChainJointsLimt(
        channel == motion_control_pkg::sdk::ArmChannel::kLeft ? 0 : 1,
        model_lower, model_upper))
    {
        abort_goal(std::string("Failed to obtain ") + arm_name +
            " arm Pinocchio joint limits");
        return;
    }
    const auto& sdk_lower =
        channel == motion_control_pkg::sdk::ArmChannel::kLeft ?
        l_q_lower_limits_ : r_q_lower_limits_;
    const auto& sdk_upper =
        channel == motion_control_pkg::sdk::ArmChannel::kLeft ?
        l_q_upper_limits_ : r_q_upper_limits_;
    const auto effective_limits =
        motion_control_pkg::motion::intersectPositionLimits(
        sdk_lower, sdk_upper, model_lower, model_upper);
    if (!effective_limits.valid) {
        abort_goal(effective_limits.error_message);
        return;
    }

    motion_control_pkg::motion::CcsJoint67Table joint67_table{};
    for (std::size_t row = 0; row < joint67_table.size(); ++row) {
        for (std::size_t column = 0;
            column < joint67_table[row].size(); ++column)
        {
            // bd_ 来自当前包的 ccs_m6_40.MvKDCfg，不使用其他工程的常量副本。
            joint67_table[row][column] = bd_[arm_index][row][column];
        }
    }
    const auto current_validation =
        motion_control_pkg::motion::validateJointTarget(
        current_joint, effective_limits.lower, effective_limits.upper,
        joint67_table);
    if (!current_validation.valid) {
        abort_goal(std::string(arm_name) +
            " arm current state is outside configured constraints: " +
            current_validation.error_message);
        return;
    }
    const auto target_validation =
        motion_control_pkg::motion::validateJointTarget(
        target_joint, effective_limits.lower, effective_limits.upper,
        joint67_table);
    if (!target_validation.valid) {
        abort_goal(std::string(arm_name) +
            " arm target is invalid: " + target_validation.error_message);
        return;
    }
    publish_feedback(15.0F, std::string(arm_name) +
        " arm joint constraints validated");

    planning_started_at = std::chrono::steady_clock::now();
    planning_active = true;
    std::vector<std::vector<double>> planned_positions;
    if (!planRuckigSamples(
            *miracle_algo_,
            std::vector<double>(current_joint.begin(), current_joint.end()),
            std::vector<double>(target_joint.begin(), target_joint.end()),
            scaled_limits.velocity, scaled_limits.acceleration,
            scaled_limits.jerk, planned_positions, conversion_error))
    {
        abort_goal(std::string(arm_name) +
            " arm Ruckig point-to-point planning failed: " + conversion_error);
        return;
    }
    if (goal->enable_collision_check) {
        std::vector<double> left_current, right_current;
        if (!updateArmModelState(initial_state, left_current, right_current, conversion_error)) {
            abort_goal(conversion_error); return;
        }
        try {
            motion_control_pkg::motion::checkChainCollision(*miracle_algo_, goal->enable_collision_check,
                channel == motion_control_pkg::sdk::ArmChannel::kLeft ? ID_LEFT_ARM : ID_RIGHT_ARM,
                planned_positions);
        } catch (const std::exception& e) { abort_goal(e.what()); return; }
    }
    const auto trajectory_validation =
        motion_control_pkg::motion::validateTrajectoryPositions(
        planned_positions, effective_limits.lower, effective_limits.upper,
        joint67_table);
    if (!trajectory_validation.valid) {
        abort_goal(trajectory_validation.error_message);
        return;
    }

    std::vector<motion_control_pkg::execution::JointCommand> command_points;
    if (!motion_control_pkg::motion::trajectoryToVendorDegrees(
        planned_positions, command_points, conversion_error))
    {
        abort_goal(conversion_error);
        return;
    }
    update_phase_times();
    planning_active = false;
    publish_feedback(25.0F, std::string(arm_name) +
        " arm Ruckig trajectory ready");

    if (goal_handle->is_canceling()) {
        cancel_goal("MoveJoints goal canceled before command submission");
        return;
    }
    if (deadline.expired()) {
        abort_goal("MoveJoints timed out before command submission");
        return;
    }
    publish_feedback(
        20.0F, std::string(arm_name) +
        (goal->control_mode == MoveJoints::Goal::CONTROL_IMPEDANCE ?
        " arm switching to joint impedance mode" :
        " arm switching to position mode"));
    const auto mode_result = configureGoalControlMode(
        goal->control_mode,
        motion_control_pkg::sdk::ArmControlMode::kJointImpedance,
        {channel});
    if (!mode_result) {
        abort_goal(mode_result.error_message);
        return;
    }
    if (goal_handle->is_canceling()) {
        cancel_goal("MoveJoints canceled during control-mode transition");
        return;
    }
    if (deadline.expired()) {
        abort_goal("MoveJoints timed out during control-mode transition");
        return;
    }
    motion_started_at = std::chrono::steady_clock::now();
    motion_active = true;
    if (!streamSingleArmTrajectory(
        *sdk_command_gate_, goal_handle, channel, command_points, deadline,
        "MoveJoints",
        publish_feedback, abort_goal, cancel_goal))
    {
        return;
    }

    // 所有离散点提交后，用新鲜 SDK 状态等待“低速到位”，并计算最大关节误差。
    auto next_feedback = std::chrono::steady_clock::now();
    while (true) {
        if (goal_handle->is_canceling()) {
            cancel_goal(
                "MoveJoints goal canceled during completion wait; "
                "no physical emergency stop was issued");
            return;
        }
        if (deadline.expired()) {
            abort_goal("MoveJoints completion wait timed out");
            return;
        }

        motion_control_pkg::sdk::DualArmDeviceState completion_state;
        const auto completion_state_result =
            sdk_command_gate_->readState(completion_state);
        if (!completion_state_result) {
            abort_goal(completion_state_result.error_message);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(arm_status_mtx_);
            dual_arm_state_ = completion_state;
        }
        const auto& selected_completion_state =
            completion_state.arms[arm_index];
        if (selected_completion_state.error_code != 0 ||
            selected_completion_state.current_state == 100)
        {
            abort_goal(
                std::string(arm_name) +
                " arm entered an SDK state error: error_code=" +
                std::to_string(selected_completion_state.error_code) +
                ", current_state=" +
                std::to_string(selected_completion_state.current_state));
            return;
        }
        result->final_distance_error =
            motion_control_pkg::motion::maximumJointErrorRadians(
            selected_completion_state.joint_position, target_joint);
        if (!std::isfinite(result->final_distance_error)) {
            abort_goal(std::string(arm_name) +
                " arm returned non-finite completion feedback");
            return;
        }
        if (selected_completion_state.low_speed) {
            result->success = ACTION_SUCCESS;
            result->error_message.clear();
            update_execution_time();
            publish_feedback(100.0F, std::string(arm_name) +
                " arm MoveJoints complete");
            terminal.succeed(result);
            RCLCPP_INFO(
                rclcpp::get_logger("dual_arm_module"),
                "%s arm MoveJoints succeeded; max joint error=%.9f rad",
                arm_name, result->final_distance_error);
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= next_feedback) {
            publish_feedback(99.0F, std::string(arm_name) +
                " arm waiting for low-speed completion");
            next_feedback = now + std::chrono::milliseconds(50);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void DualArmModule::execution_loop_move_pose_impl(
    std::shared_ptr<GoalHandleMovePose> goal_handle,
    motion_control_pkg::sdk::ArmChannel channel,
    const geometry_msgs::msg::PoseStamped& planning_target)
{
    motion_control_pkg::execution::ActionTerminalGuard<
        GoalHandleMovePose, MovePose::Result> terminal(goal_handle);
    const auto start_time = std::chrono::steady_clock::now();
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MovePose::Feedback>();
    auto result = std::make_shared<MovePose::Result>();
    result->success = ACTION_FAIL;
    result->timing_breakdown_valid = true;
    result->ik_time = 0.0;
    result->planning_time = 0.0;
    result->motion_time = 0.0;
    result->final_distance_error =
        std::numeric_limits<double>::quiet_NaN();

    bool planning_active = false;
    bool motion_active = false;
    std::chrono::steady_clock::time_point planning_started_at;
    std::chrono::steady_clock::time_point motion_started_at;
    const auto update_phase_times = [&]() {
        const auto now = std::chrono::steady_clock::now();
        if (planning_active) {
            const double planning_and_ik = std::chrono::duration<double>(
                now - planning_started_at).count();
            result->planning_time = std::max(
                0.0, planning_and_ik - result->ik_time);
        }
        if (motion_active) {
            result->motion_time = std::chrono::duration<double>(
                now - motion_started_at).count();
        }
    };
    const auto update_execution_time = [&]() {
        update_phase_times();
        result->execution_time = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time).count();
    };
    const auto abort_goal = [&](const std::string& message) {
        result->success = ACTION_FAIL;
        result->error_message = message;
        update_execution_time();
        RCLCPP_ERROR(
            rclcpp::get_logger("dual_arm_module"),
            "MovePose (%s arm) failed: %s",
            channel == motion_control_pkg::sdk::ArmChannel::kLeft ?
            "left" : "right",
            result->error_message.c_str());
        terminal.abort(result);
    };
    const auto cancel_goal = [&](const std::string& message) {
        result->success = ACTION_FAIL;
        result->error_message = message;
        update_execution_time();
        terminal.cancel(result);
    };
    const auto publish_feedback = [&goal_handle, &feedback, this](
        float progress, const std::string& step) {
        feedback->task_status = progress >= 100.0F ? "SUCCEEDED" : "RUNNING";
        feedback->progress_rate = std::clamp(progress, 0.0F, 100.0F);
        feedback->current_step_info = step;
        if (auto node = lockNode(__func__)) {
            feedback->header.stamp = node->get_clock()->now();
        }
        goal_handle->publish_feedback(feedback);
    };

    const auto expected_channel =
        motion_control_pkg::execution::armChannelFromId(goal->arm_id);
    if (!expected_channel || *expected_channel != channel) {
        abort_goal("MovePose requires exactly one matching arm");
        return;
    }

    geometry_msgs::msg::PoseStamped normalized_target;
    std::string conversion_error;
    if (!normalizePoseTargetToPlanningFrame(
        planning_target, normalized_target, conversion_error))
    {
        abort_goal(conversion_error);
        return;
    }

    //加上偏移量
    std::array<float, 3> safe_bound_size = {0.0f, 0.0f, 0.0f}; // 默认值

    if (!goal->bound_size.empty()) {
        if (goal->bound_size.size() >= 3) {
            safe_bound_size[0] = goal->bound_size[0];
            safe_bound_size[1] = goal->bound_size[1];
            safe_bound_size[2] = goal->bound_size[2];
        } else {
            RCLCPP_WARN(rclcpp::get_logger("DualArmModule"), "bound_size has less than 3 elements, using defaults for missing ones.");
            for (size_t i = 0; i < goal->bound_size.size(); ++i) {
                safe_bound_size[i] = goal->bound_size[i];
            }
        }
    } else {
        RCLCPP_INFO(rclcpp::get_logger("DualArmModule"), "bound_size is empty, using default [0, 0, 0].");
    }
    normalized_target.pose.position.x +=  safe_bound_size[0];
    normalized_target.pose.position.y +=  safe_bound_size[1];
    normalized_target.pose.position.z +=  safe_bound_size[2];

    //使用固定姿态，测试y轴180度,z轴90度，固定轴
    // normalized_target.pose.orientation.x = -0.7071068;
    // normalized_target.pose.orientation.y = 0.7071068;
    // normalized_target.pose.orientation.z = 0.0;
    // normalized_target.pose.orientation.w = 0.0;

    const auto& effective_target = normalized_target;

    const std::size_t arm_index = static_cast<std::size_t>(channel);
    const bool is_left =
        channel == motion_control_pkg::sdk::ArmChannel::kLeft;
    const char* arm_name = is_left ? "left" : "right";
    const char* model_end_frame = is_left ?
        "gripper_base_l_link" : "gripper_base_r_link";
    const auto& model_end_to_tool_tcp = is_left ?
        left_model_end_to_tool_tcp_ : right_model_end_to_tool_tcp_;

    const auto scaled_limits =
        motion_control_pkg::motion::scaleMotionLimits(
        is_left ? l_v_limit_ : r_v_limit_,
        is_left ? l_a_limit_ : r_a_limit_,
        is_left ? l_jerk_limit_ : r_jerk_limit_,
        static_cast<double>(goal->speed), goal->acceleration_scale, goal->jerk_scale);
    if (!scaled_limits.valid) {
        abort_goal(scaled_limits.error_message);
        return;
    }
    if (goal->use_force_control) {
        abort_goal("MovePose force control is not implemented in stage 7");
        return;
    }
    if (!goal->hand_action_type.empty()) {
        abort_goal("MovePose hand actions are not implemented before the tool stage");
        return;
    }

    if (!l_init_arm_params_) {
        abort_goal("Robot kinematics parameters are not initialized");
        return;
    }
    if (!init_robot_link_) {
        abort_goal("Robot link is not initialized");
        return;
    }
    if (goal_handle->is_canceling()) {
        cancel_goal("MovePose goal canceled before execution");
        return;
    }

    const auto timeout = std::chrono::milliseconds(
        goal->timeout_ms > 0 ? goal->timeout_ms : 5000);
    motion_control_pkg::execution::SteadyDeadline deadline(timeout);
    RCLCPP_INFO(
        rclcpp::get_logger("dual_arm_module"),
        "Starting %s arm MovePose using Pinocchio%s and existing trajectory libraries",
        arm_name, goal->enable_collision_check ? "/Coal/OMPL" : " IK");

    motion_control_pkg::sdk::DualArmDeviceState initial_state;
    const auto state_result = sdk_command_gate_->readState(initial_state);
    if (!state_result) {
        abort_goal(state_result.error_message);
        return;
    }
    {
        std::lock_guard<std::mutex> lock(arm_status_mtx_);
        dual_arm_state_ = initial_state;
    }
    const auto& selected_initial_state = initial_state.arms[arm_index];
    if (selected_initial_state.error_code != 0 ||
        selected_initial_state.current_state == 100)
    {
        abort_goal(
            std::string(arm_name) +
            " arm reports an SDK state error: error_code=" +
            std::to_string(selected_initial_state.error_code) +
            ", current_state=" +
            std::to_string(selected_initial_state.current_state));
        return;
    }
    if (!selected_initial_state.low_speed) {
        abort_goal(std::string(arm_name) +
            " arm is not stationary before MovePose planning");
        return;
    }

    std::vector<double> left_joint_radians;
    std::vector<double> right_joint_radians;
    std::string model_error;
    if (!updateArmModelState(
        initial_state, left_joint_radians, right_joint_radians, model_error))
    {
        abort_goal(model_error);
        return;
    }
    std::vector<double> full_start_q;
    if (!miracle_algo_->getFullJointState(full_start_q)) {
        abort_goal("Failed to obtain the Pinocchio full-body start state");
        return;
    }
    std::vector<double> current_arm_q;
    if (!miracle_algo_->extractSingleArmJoints(
        is_left, full_start_q, current_arm_q))
    {
        abort_goal(std::string("Failed to extract ") + arm_name +
            " arm joints by model name");
        return;
    }
    publish_feedback(5.0F, std::string(arm_name) + " arm state acquired");

    // Action 目标始终表示工具 TCP；先反解固定 end_T_tcp，再把中央
    // planning_frame 目标交给同一 Pinocchio 模型转换到模型根坐标系。
    const Eigen::Isometry3d planning_to_model_end =
        motion_control_pkg::kinematics::targetTcpToModelEndPose(
        effective_target.pose, model_end_to_tool_tcp);
    Eigen::Isometry3d root_to_model_end = Eigen::Isometry3d::Identity();
    if (!miracle_algo_->transformPoseToModelRoot(
        planning_frame_, full_start_q, planning_to_model_end,
        root_to_model_end))
    {
        abort_goal("Failed to transform the MovePose target into the model root frame");
        return;
    }

    // Pinocchio 模型限位与当前包 CCS 软限位取交集；OMPL 路点与时间参数化
    // 后的全部 4 ms 采样点都会经过同一套约束检查。
    std::vector<double> model_lower(motion_control_pkg::sdk::kArmDof);
    std::vector<double> model_upper(motion_control_pkg::sdk::kArmDof);
    if (!miracle_algo_->getChainJointsLimt(
        is_left ? 0 : 1, model_lower, model_upper))
    {
        abort_goal(std::string("Failed to obtain ") + arm_name +
            " arm Pinocchio joint limits");
        return;
    }
    const auto effective_limits =
        motion_control_pkg::motion::intersectPositionLimits(
        is_left ? l_q_lower_limits_ : r_q_lower_limits_,
        is_left ? l_q_upper_limits_ : r_q_upper_limits_,
        model_lower, model_upper);
    if (!effective_limits.valid) {
        abort_goal(effective_limits.error_message);
        return;
    }
    motion_control_pkg::motion::CcsJoint67Table joint67_table{};
    for (std::size_t row = 0; row < joint67_table.size(); ++row) {
        for (std::size_t column = 0;
            column < joint67_table[row].size(); ++column)
        {
            joint67_table[row][column] = bd_[arm_index][row][column];
        }
    }
    motion_control_pkg::motion::JointVector current_joint{};
    std::copy(current_arm_q.begin(), current_arm_q.end(),
        current_joint.begin());
    const auto current_validation =
        motion_control_pkg::motion::validateJointTarget(
        current_joint, effective_limits.lower, effective_limits.upper,
        joint67_table);
    if (!current_validation.valid) {
        abort_goal(std::string(arm_name) +
            " arm current state is outside configured constraints: " +
            current_validation.error_message);
        return;
    }

    std::vector<std::vector<double>> active_arm_path;
    std::vector<std::vector<double>> planned_positions;
    std::vector<motion_control_pkg::execution::JointCommand> command_points;
    const char* planner_name = "Unknown";
    planning_started_at = std::chrono::steady_clock::now();
    planning_active = true;
    auto collision_scene = motion_control_pkg::motion::makeCollisionScene(
        *miracle_algo_, goal->enable_collision_check, full_start_q,
        goal->env_point_cloud, conversion_error);
    if (!collision_scene) { abort_goal(conversion_error); return; }
    std::vector<std::vector<double>> via_joint_targets;
    std::vector<Eigen::Isometry3d> via_tcp_targets;
    if (!goal->enable_move_line) {
        if (!goal->via_poses.empty()) {
            active_arm_path.push_back(current_arm_q);
            std::vector<double> seed = full_start_q;
            std::vector<Eigen::Isometry3d> targets;
            for (const auto& via : goal->via_poses) {
                geometry_msgs::msg::PoseStamped normalized;
                if (!normalizePoseTargetToPlanningFrame(via, normalized, conversion_error)) {
                    abort_goal(conversion_error); return;
                }
                if (goal->bound_size.size() == 3) {
                    normalized.pose.position.x += goal->bound_size[0];
                    normalized.pose.position.y += goal->bound_size[1];
                    normalized.pose.position.z += goal->bound_size[2];
                }
                via_tcp_targets.push_back(motion_control_pkg::kinematics::poseToIsometry(normalized.pose));
                Eigen::Isometry3d root_target;
                if (!miracle_algo_->transformPoseToModelRoot(planning_frame_, full_start_q,
                    motion_control_pkg::kinematics::targetTcpToModelEndPose(normalized.pose, model_end_to_tool_tcp),
                    root_target)) {abort_goal("Failed to transform MoveP via TCP"); return;}
                targets.push_back(root_target);
            }
            targets.push_back(root_to_model_end);
            for (std::size_t index = 0; index < targets.size(); ++index) {
                if (goal_handle->is_canceling()) {cancel_goal("Continuous MoveP canceled during planning"); return;}
                if (deadline.expired()) {abort_goal("Continuous MoveP timed out during planning"); return;}
                if (goal->enable_collision_check) {
                    std::vector<std::vector<double>> segment;
                    double ik_seconds = 0;
                    if (!miracle_algo_->planSingleArmCartesianGoal(is_left, seed, targets[index], segment, &ik_seconds) || segment.empty()) {
                        abort_goal("Continuous MoveP IK/OMPL segment failed"); return;
                    }
                    result->ik_time += ik_seconds;
                    for (std::size_t k = 1; k < segment.size(); ++k) {
                        std::vector<double> q;
                        if (!miracle_algo_->extractSingleArmJoints(is_left, segment[k], q)) {
                            abort_goal("Invalid continuous MoveP waypoint"); return;
                        }
                        active_arm_path.push_back(std::move(q));
                    }
                    seed = segment.back();
                } else {
                    std::vector<double> q;
                    const auto ik_start = std::chrono::steady_clock::now();
                    const bool ok = miracle_algo_->solveSingleArmCartesianIk(is_left, seed, targets[index], q);
                    result->ik_time += std::chrono::duration<double>(std::chrono::steady_clock::now()-ik_start).count();
                    if (!ok) {abort_goal("Continuous MoveP IK failed"); return;}
                    active_arm_path.push_back(q);
                    std::vector<double> next;
                    if (!miracle_algo_->mergeSingleArmJoints(is_left, seed, q, next)) {
                        abort_goal("Continuous MoveP state merge failed"); return;
                    }
                    seed = std::move(next);
                }
                if (index < goal->via_poses.size()) {via_joint_targets.push_back(active_arm_path.back());}
            }
        } else if (goal->enable_collision_check) {
            // 复用 MiracleMAlgo 内部 Pinocchio IK、Coal 状态有效性与 OMPL
            // RRTConnect；旧的恒真 OMPLWrapper 不再进入 MovePose 主链。
            std::vector<std::vector<double>> full_ompl_path;
            if (!miracle_algo_->planSingleArmCartesianGoal(
                is_left, full_start_q, root_to_model_end, full_ompl_path,
                &result->ik_time))
            {
                abort_goal(std::string(arm_name) +
                    " arm Pinocchio/Coal/OMPL pose planning failed");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("DualArmModule"), "进入执行move_pose_碰撞规划成功");
            active_arm_path.reserve(full_ompl_path.size());
            for (const auto& full_q : full_ompl_path) {
                std::vector<double> arm_q;
                if (!miracle_algo_->extractSingleArmJoints(
                    is_left, full_q, arm_q))
                {
                    abort_goal("OMPL returned an invalid full-body waypoint");
                    return;
                }
                active_arm_path.push_back(std::move(arm_q));
            }
            publish_feedback(15.0F, std::string(arm_name) +
                " arm collision-checked OMPL path ready");
        } else {
            // 无碰撞模式仍使用现成 Pinocchio IK。单目标形成唯一的起终两点，
            // 必须交给 Ruckig，不通过业务层插值。
            std::vector<double> ik_goal;
            const auto ik_started_at = std::chrono::steady_clock::now();
            const bool ik_ok = miracle_algo_->solveSingleArmCartesianIk(
                is_left, full_start_q, root_to_model_end, ik_goal);
            result->ik_time += std::chrono::duration<double>(
                std::chrono::steady_clock::now() - ik_started_at).count();
            if (!ik_ok)
            {
                abort_goal(std::string(arm_name) + " arm Pinocchio IK failed");
                return;
            }
            active_arm_path = {current_arm_q, std::move(ik_goal)};
            publish_feedback(15.0F, std::string(arm_name) +
                " arm Pinocchio IK target ready");
        }

        const auto pose_path =
            motion_control_pkg::motion::preparePoseWaypointPath(active_arm_path);
        if (!pose_path.valid) {
            abort_goal(pose_path.error_message);
            return;
        }
        const std::vector<std::vector<double>> waypoint_positions =
            extractWaypointPositions(pose_path.waypoints);
        const auto waypoint_validation =
            motion_control_pkg::motion::validateTrajectoryPositions(
            waypoint_positions, effective_limits.lower, effective_limits.upper,
            joint67_table);
        if (!waypoint_validation.valid) {
            abort_goal("IK/OMPL path violates configured constraints: " +
                waypoint_validation.error_message);
            return;
        }

        const bool use_ruckig = pose_path.kind ==
            motion_control_pkg::motion::PoseTrajectoryKind::kRuckigPointToPoint;
        planner_name = use_ruckig ? "Ruckig" : "TOPPRA";
        std::string planner_error;
    
        const bool planned = use_ruckig ?
            planRuckigSamples(
                *miracle_algo_, waypoint_positions.front(), waypoint_positions.back(),
                scaled_limits.velocity, scaled_limits.acceleration,
                scaled_limits.jerk, planned_positions, planner_error) :
            planToppraSamples(
                *miracle_algo_, waypoint_positions, scaled_limits.velocity,
                scaled_limits.acceleration, scaled_limits.jerk, planned_positions, planner_error);
        if (!planned) {
            abort_goal(std::string(arm_name) + " arm " + planner_name +
                " time parameterization failed: " + planner_error);
            return;
        }   
    }
    else {  
        planner_name = "CartesianLine/TOPPRA";
        constexpr std::size_t kMaxCartesianLineRefinementAttempts = 3U;
        double translation_step =
            motion_control_pkg::motion::kCartesianIkTranslationStepMeters;
        double rotation_step =
            motion_control_pkg::motion::kCartesianIkRotationStepRadians;
        std::string line_error;
        bool line_planned = false;
        for (std::size_t attempt = 0U;
            attempt < kMaxCartesianLineRefinementAttempts; ++attempt)
        {
            active_arm_path.clear();
            planned_positions.clear();
            Eigen::Isometry3d actual_start_pose = Eigen::Isometry3d::Identity();
            double attempt_ik_time = 0.0;
            const bool ik_planned =
                miracle_algo_->planSingleArmCartesianIkWaypoints(
                is_left, full_start_q, root_to_model_end,
                translation_step, rotation_step, active_arm_path,
                actual_start_pose, &attempt_ik_time);
            result->ik_time += attempt_ik_time;
            if (!ik_planned) {
                line_error = "coarse continuous IK failed";
            } else {
                const auto coarse_validation =
                    motion_control_pkg::motion::validateTrajectoryPositions(
                    active_arm_path, effective_limits.lower,
                    effective_limits.upper, joint67_table);
                if (!coarse_validation.valid) {
                    line_error =
                        "coarse Cartesian IK path violates configured constraints: " +
                        coarse_validation.error_message;
                } else {
                    std::string toppra_error;
                    if (!planToppraSamples(
                        *miracle_algo_, active_arm_path, scaled_limits.velocity,
                        scaled_limits.acceleration, scaled_limits.jerk, planned_positions,
                        toppra_error,
                        MiracleMAlgo::ToppraSolverMode::kStableCartesian,
                        std::chrono::duration<double>(timeout).count()))
                    {
                        line_error = "TOPPRA failed: " + toppra_error;
                    } else {
                        double max_position_error_m = 0.0;
                        double max_orientation_error_rad = 0.0;
                        if (miracle_algo_->validateSingleArmCartesianTrajectory(
                            is_left, full_start_q, actual_start_pose,
                            root_to_model_end, planned_positions,
                            motion_control_pkg::motion::
                            kCartesianLinePositionToleranceMeters,
                            motion_control_pkg::motion::
                            kCartesianLineOrientationToleranceRadians,
                            max_position_error_m, max_orientation_error_rad,
                            line_error))
                        {
                            RCLCPP_INFO(
                                rclcpp::get_logger("dual_arm_module"),
                                "Cartesian line validated: attempt=%zu, IK waypoints=%zu, 4 ms samples=%zu, max_position_error=%.6f m, max_orientation_error=%.6f rad, ik_time=%.6f s",
                                attempt + 1U, active_arm_path.size(),
                                planned_positions.size(), max_position_error_m,
                                max_orientation_error_rad, attempt_ik_time);
                            line_planned = true;
                            break;
                        }
                    }
                }
            }

            if (attempt + 1U < kMaxCartesianLineRefinementAttempts) {
                RCLCPP_WARN(
                    rclcpp::get_logger("dual_arm_module"),
                    "Cartesian line attempt failed at IK steps %.6f m / %.6f rad (%s); refining sampling",
                    translation_step, rotation_step, line_error.c_str());
                translation_step *= 0.5;
                rotation_step *= 0.5;
            }
        }
        if (!line_planned) {
            abort_goal("Cartesian line planning failed: " + line_error);
            return;
        }
    }

    if (goal->enable_collision_check) {
        for (const auto& q : planned_positions) {
            std::vector<double> full;
            if (deadline.expired() || !miracle_algo_->mergeSingleArmJoints(is_left, full_start_q, q, full) ||
                !miracle_algo_->isFullQValid(miracle_algo_->toEigenConfig(full))) {
                abort_goal("Retimed MoveP/MoveL sample failed collision check or timed out"); return;
            }
        }
    }
    for (std::size_t via = 0; via < via_joint_targets.size(); ++via) {
        const auto distance = [&](const std::vector<double>& q) {
            double d = 0;
            for (std::size_t j = 0; j < q.size(); ++j) {d += std::pow(q[j]-via_joint_targets[via][j], 2);}
            return d;
        };
        const auto nearest = std::min_element(planned_positions.begin(), planned_positions.end(),
            [&](const auto& a, const auto& b) {return distance(a) < distance(b);});
        std::vector<double> full;
        Eigen::Isometry3d end_pose;
        if (nearest == planned_positions.end() ||
            !miracle_algo_->mergeSingleArmJoints(is_left, full_start_q, *nearest, full) ||
            !miracle_algo_->getFramePoseAtConfiguration(planning_frame_, model_end_frame, full, end_pose)) {
            abort_goal("Continuous MoveP via FK failed"); return;
        }
        const auto tcp = end_pose * model_end_to_tool_tcp;
        if ((tcp.translation()-via_tcp_targets[via].translation()).norm() >
                motion_control_pkg::motion::kCartesianLinePositionToleranceMeters ||
            Eigen::AngleAxisd(tcp.rotation().transpose()*via_tcp_targets[via].rotation()).angle() >
                motion_control_pkg::motion::kCartesianLineOrientationToleranceRadians) {
            abort_goal("Continuous MoveP misses via TCP tolerance after retiming"); return;
        }
    }
    const auto trajectory_validation =
        motion_control_pkg::motion::validateTrajectoryPositions(
        planned_positions, effective_limits.lower, effective_limits.upper,
        joint67_table);
    if (!trajectory_validation.valid) {
        abort_goal(trajectory_validation.error_message);
        return;
    }
    if (goal->enable_collision_check) {
        // Validate the actual 4 ms samples, including MoveL and TOPPRA interpolation.
        const auto& model = miracle_algo_->getModel();
        for (const auto& arm_sample : planned_positions) {
            Eigen::VectorXd full_sample = miracle_algo_->toEigenConfig(full_start_q);
            for (std::size_t joint = 0; joint < arm_sample.size(); ++joint) {
                const std::string name = "Joint" + std::to_string(joint + 1U) + (is_left ? "_L" : "_R");
                full_sample[model.joints[model.getJointId(name)].idx_q()] = arm_sample[joint];
            }
            if (deadline.expired() || !miracle_algo_->isFullQValid(full_sample)) {
                abort_goal("MovePose sampled collision validation failed or timed out");
                return;
            }
        }
    }
    collision_scene.reset();
    if (!motion_control_pkg::motion::trajectoryToVendorDegrees(
        planned_positions, command_points, conversion_error))
    {
        abort_goal(conversion_error);
        return;
    }
    update_phase_times();
    planning_active = false;
    publish_feedback(25.0F, std::string(arm_name) + " arm " + planner_name +
        " 4 ms trajectory ready");

    if (goal_handle->is_canceling()) {
        cancel_goal("MovePose goal canceled before command submission");
        return;
    }
    if (deadline.expired()) {
        abort_goal("MovePose timed out before command submission");
        return;
    }
    publish_feedback(
        20.0F, std::string(arm_name) +
        (goal->control_mode == MovePose::Goal::CONTROL_IMPEDANCE ?
        " arm switching to Cartesian impedance mode" :
        " arm switching to position mode"));
    const auto mode_result = configureGoalControlMode(
        goal->control_mode,
        motion_control_pkg::sdk::ArmControlMode::kCartesianImpedance,
        {channel});
    if (!mode_result) {
        abort_goal(mode_result.error_message);
        return;
    }
    if (goal_handle->is_canceling()) {
        cancel_goal("MovePose canceled during control-mode transition");
        return;
    }
    if (deadline.expired()) {
        abort_goal("MovePose timed out during control-mode transition");
        return;
    }
    motion_started_at = std::chrono::steady_clock::now();
    motion_active = true;
    if (!streamSingleArmTrajectory(
        *sdk_command_gate_, goal_handle, channel, command_points, deadline,
        "MovePose",
        publish_feedback, abort_goal, cancel_goal))
    {
        return;
    }

    // Action 终态只由这条共享左右臂链产生：持续读取新鲜 SDK 状态，更新
    // Pinocchio FK，并以工具 TCP 平移误差（米）填充 final_distance_error。
    auto next_feedback = std::chrono::steady_clock::now();
    const Eigen::Isometry3d target_tcp =
        motion_control_pkg::kinematics::poseToIsometry(effective_target.pose);
    while (true) {
        if (goal_handle->is_canceling()) {
            cancel_goal(
                "MovePose goal canceled during completion wait; "
                "no physical emergency stop was issued");
            return;
        }
        if (deadline.expired()) {
            abort_goal("MovePose completion wait timed out");
            return;
        }

        motion_control_pkg::sdk::DualArmDeviceState completion_state;
        const auto completion_result =
            sdk_command_gate_->readState(completion_state);
        if (!completion_result) {
            abort_goal(completion_result.error_message);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(arm_status_mtx_);
            dual_arm_state_ = completion_state;
        }
        const auto& selected_completion_state =
            completion_state.arms[arm_index];
        if (selected_completion_state.error_code != 0 ||
            selected_completion_state.current_state == 100)
        {
            abort_goal(
                std::string(arm_name) +
                " arm entered an SDK state error: error_code=" +
                std::to_string(selected_completion_state.error_code) +
                ", current_state=" +
                std::to_string(selected_completion_state.current_state));
            return;
        }

        if (!updateArmModelState(
            completion_state, left_joint_radians, right_joint_radians,
            model_error))
        {
            abort_goal(model_error);
            return;
        }
        geometry_msgs::msg::Pose current_tcp_pose;
        if (!computeToolTcpPose(
            model_end_frame, model_end_to_tool_tcp, current_tcp_pose,
            model_error))
        {
            abort_goal(model_error);
            return;
        }
        const Eigen::Isometry3d current_tcp =
            motion_control_pkg::kinematics::poseToIsometry(current_tcp_pose);
        result->final_distance_error =
            (current_tcp.translation() - target_tcp.translation()).norm();
        if (!std::isfinite(result->final_distance_error)) {
            abort_goal(std::string(arm_name) +
                " arm returned non-finite completion feedback");
            return;
        }

        if (selected_completion_state.low_speed) {
            result->success = ACTION_SUCCESS;
            result->error_message.clear();
            update_execution_time();
            publish_feedback(100.0F, std::string(arm_name) +
                " arm MovePose complete");
            terminal.succeed(result);
            RCLCPP_INFO(
                rclcpp::get_logger("dual_arm_module"),
                "%s arm MovePose succeeded via %s; TCP translation error=%.9f m",
                arm_name, planner_name, result->final_distance_error);
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= next_feedback) {
            publish_feedback(99.0F, std::string(arm_name) +
                " arm waiting for low-speed completion");
            next_feedback = now + std::chrono::milliseconds(50);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

bool DualArmModule::start_execution_control_gripper(
    std::shared_ptr<GoalHandleControlGripper> goal_handle,
    std::string& error_message)
{
    const auto goal = goal_handle->get_goal();
    const bool need_left = goal->gripper_id == GRIPPER_LEFT ||
        goal->gripper_id == GRIPPER_BOTH;
    const bool need_right = goal->gripper_id == GRIPPER_RIGHT ||
        goal->gripper_id == GRIPPER_BOTH;
    if (!need_left && !need_right) {
        error_message = "unsupported gripper_id";
        return false;
    }

    std::lock_guard<std::mutex> lock(control_tool_start_mutex_);
    if ((need_left && is_executing_left_tool_.load()) ||
        (need_right && is_executing_right_tool_.load()))
    {
        error_message = "a requested tool is still executing another goal";
        return false;
    }

    if (need_left) {
        is_executing_left_tool_.store(true);
    }
    if (need_right) {
        is_executing_right_tool_.store(true);
    }

    try {
        const auto worker_slot = need_left && need_right ?
            motion_control_pkg::execution::GripperWorkerSlot::kBoth :
            (need_left ?
                motion_control_pkg::execution::GripperWorkerSlot::kLeft :
                motion_control_pkg::execution::GripperWorkerSlot::kRight);
        gripper_worker_threads_.start(
            worker_slot,
            &DualArmModule::execution_loop_control_gripper, this, goal_handle);
    } catch (const std::exception& exception) {
        if (need_left) {
            is_executing_left_tool_.store(false);
        }
        if (need_right) {
            is_executing_right_tool_.store(false);
        }
        error_message = std::string("failed to create gripper worker: ") +
            exception.what();
        return false;
    }

    error_message.clear();
    return true;
}

bool DualArmModule::prepare_preplanned_dispatch(
    const std::vector<std::vector<double>>& left_positions,
    const std::vector<std::vector<double>>& right_positions,
    std::uint8_t control_mode,
    std::string& error_message)
{
    prepared_dispatch_ = PreparedDispatch{};
    const bool need_left = !left_positions.empty();
    const bool need_right = !right_positions.empty();
    if (!need_left && !need_right) {
        error_message = "preplanned dual-arm trajectory is empty";
        return false;
    }
    if ((need_left && left_positions.front().size() != 7U) ||
        (need_right && right_positions.front().size() != 7U))
    {
        error_message = "preplanned arm trajectory must contain seven joints";
        return false;
    }
    if (need_left && need_right && left_positions.size() != right_positions.size()) {
        error_message = "preplanned dual-arm trajectories must have equal sample counts";
        return false;
    }

    if (need_left && !motion_control_pkg::motion::trajectoryToVendorDegrees(
        left_positions, prepared_dispatch_.left_commands, error_message)) {
        return false;
    }
    if (need_right && !motion_control_pkg::motion::trajectoryToVendorDegrees(
        right_positions, prepared_dispatch_.right_commands, error_message)) {
        return false;
    }

    prepared_dispatch_.need_left = need_left;
    prepared_dispatch_.need_right = need_right;
    if (need_left) {
        prepared_dispatch_.channels.push_back(motion_control_pkg::sdk::ArmChannel::kLeft);
    }
    if (need_right) {
        prepared_dispatch_.channels.push_back(motion_control_pkg::sdk::ArmChannel::kRight);
    }
    const auto mode_result = configureGoalControlMode(
        control_mode,
        motion_control_pkg::sdk::ArmControlMode::kCartesianImpedance,
        prepared_dispatch_.channels);
    if (!mode_result) {
        error_message = mode_result.error_message;
        return false;
    }
    prepared_dispatch_.valid = true;
    error_message.clear();
    return true;
}

bool DualArmModule::dispatch_preplanned(
    const std::chrono::steady_clock::time_point& start_time,
    const std::shared_ptr<motion_control_pkg::execution::CancellationToken>& cancel,
    std::string& error_message)
{
    if (!prepared_dispatch_.valid) {
        error_message = "dispatch_preplanned called without successful prepare";
        if (cancel) cancel->request();
        return false;
    }
    prepared_dispatch_.valid = false;
    if (cancel && cancel->requested()) {
        error_message = "preplanned trajectory canceled before execution";
        return false;
    }

    const std::size_t sample_count = prepared_dispatch_.need_left
        ? prepared_dispatch_.left_commands.size()
        : prepared_dispatch_.right_commands.size();
    return motion_control_pkg::execution::dispatchPreplannedSamples(
        sample_count, start_time, cancel, [&](std::size_t index) {
        const auto send_result = sdk_command_gate_->sendJointPositions(
            prepared_dispatch_.need_left
                ? std::optional<motion_control_pkg::execution::JointCommand>(
                      prepared_dispatch_.left_commands[index])
                : std::nullopt,
            prepared_dispatch_.need_right
                ? std::optional<motion_control_pkg::execution::JointCommand>(
                      prepared_dispatch_.right_commands[index])
                : std::nullopt);
        if (!send_result) {
            error_message = send_result.error_message;
            return false;
        }
        return true;
    }, error_message);
}

bool DualArmModule::execute_preplanned_trajectory(
    const std::vector<std::vector<double>>& left_positions,
    const std::vector<std::vector<double>>& right_positions,
    std::uint8_t control_mode,
    const std::chrono::steady_clock::time_point& start_time,
    const std::shared_ptr<motion_control_pkg::execution::CancellationToken>& cancel,
    std::string& error_message)
{
    if (cancel && cancel->requested()) {
        error_message = "preplanned trajectory canceled before preparation";
        return false;
    }
    if (!prepare_preplanned_dispatch(
            left_positions, right_positions, control_mode, error_message)) {
        return false;
    }
    return dispatch_preplanned(start_time, cancel, error_message);
}

bool DualArmModule::reserve_preplanned_trajectory_resources(
    bool need_left, bool need_right, std::string& error_message)
{
    return tryClaimArms(need_left, need_right, error_message);
}

void DualArmModule::release_preplanned_trajectory_resources(
    bool release_left, bool release_right) noexcept
{
    releaseArms(release_left, release_right);
}

bool DualArmModule::get_joint_motion_limits(
    bool is_left,
    std::vector<double>& velocity,
    std::vector<double>& acceleration,
    std::vector<double>& jerk) const
{
    const auto& source_velocity = is_left ? l_v_limit_ : r_v_limit_;
    const auto& source_acceleration = is_left ? l_a_limit_ : r_a_limit_;
    const auto& source_jerk = is_left ? l_jerk_limit_ : r_jerk_limit_;
    if (source_velocity.size() != static_cast<std::size_t>(arm_dof_) ||
        source_acceleration.size() != static_cast<std::size_t>(arm_dof_) ||
        source_jerk.size() != static_cast<std::size_t>(arm_dof_) ||
        !std::all_of(source_velocity.begin(), source_velocity.end(),
            [](double value) { return std::isfinite(value) && value > 0.0; }) ||
        !std::all_of(source_acceleration.begin(), source_acceleration.end(),
            [](double value) { return std::isfinite(value) && value > 0.0; }) ||
        !std::all_of(source_jerk.begin(), source_jerk.end(),
            [](double value) { return std::isfinite(value) && value > 0.0; })) {
        velocity.clear();
        acceleration.clear();
        jerk.clear();
        return false;
    }
    velocity = source_velocity;
    acceleration = source_acceleration;
    jerk = source_jerk;
    return true;
}

bool DualArmModule::start_execution_control_suck(
    std::shared_ptr<GoalHandleControlSuck> goal_handle,
    std::string& error_message)
{
    const auto goal = goal_handle->get_goal();
    const bool need_left = goal->suck_id == ControlSuck::Goal::SUCK_LEFT ||
        goal->suck_id == ControlSuck::Goal::SUCK_BOTH;
    const bool need_right = goal->suck_id == ControlSuck::Goal::SUCK_RIGHT ||
        goal->suck_id == ControlSuck::Goal::SUCK_BOTH;
    if (!need_left && !need_right) {
        error_message = "unsupported suck_id";
        return false;
    }

    std::lock_guard<std::mutex> lock(control_tool_start_mutex_);
    if ((need_left && is_executing_left_tool_.load()) ||
        (need_right && is_executing_right_tool_.load()))
    {
        error_message = "a requested tool is still executing another goal";
        return false;
    }

    if (execution_thread_control_suck_.joinable()) {
        execution_thread_control_suck_.join();
    }

    if (need_left) {
        is_executing_left_tool_.store(true);
    }
    if (need_right) {
        is_executing_right_tool_.store(true);
    }

    try {
        execution_thread_control_suck_ = std::thread(
            &DualArmModule::execution_loop_control_suck, this, goal_handle);
    } catch (const std::exception& exception) {
        if (need_left) {
            is_executing_left_tool_.store(false);
        }
        if (need_right) {
            is_executing_right_tool_.store(false);
        }
        error_message = std::string("failed to create suck worker: ") +
            exception.what();
        return false;
    }

    error_message.clear();
    return true;
}

void DualArmModule::execution_loop_control_suck(
    std::shared_ptr<GoalHandleControlSuck> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    const bool need_left = goal->suck_id == ControlSuck::Goal::SUCK_LEFT ||
        goal->suck_id == ControlSuck::Goal::SUCK_BOTH;
    const bool need_right = goal->suck_id == ControlSuck::Goal::SUCK_RIGHT ||
        goal->suck_id == ControlSuck::Goal::SUCK_BOTH;

    std::atomic<bool> * first_busy_flag = need_left ?
        &is_executing_left_tool_ : &is_executing_right_tool_;
    std::atomic<bool> * second_busy_flag = need_left && need_right ?
        &is_executing_right_tool_ : nullptr;
    motion_control_pkg::execution::ScopedBusyFlagReset busy_reset(
        first_busy_flag, second_busy_flag);
    ControlSuckTerminalGuard terminal(goal_handle);
    motion_control_pkg::execution::SteadyDeadline deadline(
        std::chrono::milliseconds(goal->timeout_ms));

    auto feedback = std::make_shared<ControlSuck::Feedback>();
    auto result = std::make_shared<ControlSuck::Result>();
    const auto publish_feedback = [&](float progress) {
        feedback->progress_rate = progress;
        if (auto node = lockNode(__func__)) {
            feedback->header.stamp = node->now();
        }
        goal_handle->publish_feedback(feedback);
    };
    const auto abort_gpio = [&](const std::string& message) {
        result->success = ACTION_FAIL;
        result->error_message = message;
        RCLCPP_ERROR(
            rclcpp::get_logger("ControlSuck"), "%s", message.c_str());
        terminal.abort(result);
    };

    publish_feedback(0.0F);
    if (deadline.expired()) {
        abort_gpio("ControlSuck timed out before GPIO initialization");
        return;
    }

    std::lock_guard<std::mutex> gpio_lock(suck_gpio_mutex_);

    if (!suck_gpio_initialized_) {
        if (neardi47_gpio_init(true, true) < 0) {
            const int error_number = errno;
            abort_gpio(
                std::string("Failed to initialize /dev/gpiochip4: ") +
                std::strerror(error_number));
            return;
        }
        suck_gpio_initialized_ = true;
    }

    publish_feedback(50.0F);
    if (deadline.expired()) {
        abort_gpio("ControlSuck timed out before setting GPIO output");
        return;
    }

    int write_result = 0;
    if (need_left && need_right) {
        write_result = neardi47_gpio_write(!goal->active[0], !goal->active[1]);
    } else if (need_left) {
        write_result = neardi47_gpio4_b2_write(!goal->active[0]);
    } else {
        write_result = neardi47_gpio4_b0_write(!goal->active[1]);
    }
    if (write_result < 0) {
        const int error_number = errno;
        neardi47_gpio_deinit();
        suck_gpio_initialized_ = false;
        abort_gpio(
            std::string("Failed to set suction GPIO output: ") +
            std::strerror(error_number));
        return;
    }

    if (need_left) {
        left_suck_active_ = goal->active[0];
    }
    if (need_right) {
        right_suck_active_ = goal->active[1];
    }

    publish_feedback(100.0F);
    result->success = ACTION_SUCCESS;
    result->error_message.clear();
    terminal.succeed(result);
}

void DualArmModule::execution_loop_control_gripper(std::shared_ptr<GoalHandleControlGripper> goal_handle)
{
    //添加机器人开/合抓逻辑   示例
    const auto goal = goal_handle->get_goal();
    std::atomic<bool> * first_busy_flag = nullptr;
    std::atomic<bool> * second_busy_flag = nullptr;
    if (goal->gripper_id == GRIPPER_LEFT) {
        first_busy_flag = &is_executing_left_tool_;
    } else if (goal->gripper_id == GRIPPER_RIGHT) {
        first_busy_flag = &is_executing_right_tool_;
    } else if (goal->gripper_id == GRIPPER_BOTH) {
        first_busy_flag = &is_executing_left_tool_;
        second_busy_flag = &is_executing_right_tool_;
    }
    motion_control_pkg::execution::ScopedBusyFlagReset busy_reset(
        first_busy_flag, second_busy_flag);
    ControlGripperTerminalGuard terminal(goal_handle);
    const auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(goal->timeout_ms);

    auto feedback = std::make_shared<ControlGripper::Feedback>();
    auto result = std::make_shared<ControlGripper::Result>();
    feedback->current_position.assign(2U, 0U);
    feedback->current_torque.assign(2U, 0U);
    feedback->current_speed.assign(2U, 0U);
    result->final_position.assign(2U, 0U);
    result->final_torque.assign(2U, 0U);
    std::shared_ptr<Jodell_Control> jodell_control = std::make_shared<Jodell_Control>();

    Jodell_status rt_status[2] = {{}, {}};
    int init_position[2] = {0,0};
    

    if (goal->gripper_id == GRIPPER_LEFT)
    {
        jodell_control->Jodell_modbus_config_A(goal->com_port[0], 9);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
        bool ret = jodell_control->Jodell_Status_A(&rt_status[GRIPPER_LEFT]);
        if (ret)
        {
            init_position[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].position;
        }
        else
        {
            result->success = ACTION_FAIL;
            result->error_message = "Get gripper_A status failed";
            RCLCPP_ERROR(rclcpp::get_logger("Jodell_Status_A"), "Get gripper_A status failed");
            terminal.abort(result);
            return;
        }

        if (!jodell_control->Jodell_Move_Abs_A(
                goal->position[GRIPPER_LEFT],
                goal->max_torque[GRIPPER_LEFT],
                goal->speed[GRIPPER_LEFT]))
        {
            result->success = ACTION_FAIL;
            result->error_message = "Send gripper_A move command failed";
            terminal.abort(result);
            return;
        }
        while (true)
        {

            // 休眠20ms

            rclcpp::sleep_for(std::chrono::milliseconds(20));

            if (goal_handle->is_canceling()) {
                result->success = ACTION_FAIL;
                result->error_message = "ControlGripper goal canceled by client";
                terminal.cancel(result);
                return;
            }

            /* 超时检查 */
            if (std::chrono::steady_clock::now() - start_time >= timeout)
            {
                RCLCPP_ERROR(rclcpp::get_logger("timeout"), "timeout exit");
                result->success = ACTION_FAIL;
                result->error_message = "time_out";
                result->final_position[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].position;
                result->final_position[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].position;
                result->final_torque[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].torque;
                result->final_torque[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].torque;
                terminal.abort(result);
                return;
            }
            ret = jodell_control->Jodell_Status_A(&rt_status[GRIPPER_LEFT]);

            if (!ret)
            {
                result->success = ACTION_FAIL;
                result->error_message = "Get gripper_A status failed while moving";
                terminal.abort(result);
                return;
            }
            else
            {
                log_jodell_status(rclcpp::get_logger("Jodell_Status_A"), rt_status[GRIPPER_LEFT]);
                if(rt_status[GRIPPER_LEFT].gto == 0)
                {
                    control_gripper_result(
                        rt_status, goal->gripper_id, result, terminal);
                    break;
                }
                else
                {
                    feedback->current_position[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].position;
                    feedback->current_torque[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].torque;
                    feedback->current_speed[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].speed;
                    feedback->progress_rate = get_progress_1000(init_position[GRIPPER_LEFT], goal->position[GRIPPER_LEFT], rt_status[GRIPPER_LEFT].position);
                    if (auto node = lockNode(__func__)) {
                        feedback->header.stamp = node->now();
                    }
                    goal_handle->publish_feedback(feedback);
                }
            }
            

        }
    } 
    else if (goal->gripper_id == GRIPPER_RIGHT)
    {
        jodell_control->Jodell_modbus_config_B(goal->com_port[1], 9);
        bool ret = jodell_control->Jodell_Status_B(&rt_status[GRIPPER_RIGHT]);
        if (ret)
        {
            init_position[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].position;
        }
        else
        {
            result->success = ACTION_FAIL;
            result->error_message = "Get gripper_B status failed";
            RCLCPP_ERROR(rclcpp::get_logger("Jodell_Status_B"), "Get gripper_B status failed");
            terminal.abort(result);
            return;
        }
        if (!jodell_control->Jodell_Move_Abs_B(
                goal->position[GRIPPER_RIGHT],
                goal->max_torque[GRIPPER_RIGHT],
                goal->speed[GRIPPER_RIGHT]))
        {
            result->success = ACTION_FAIL;
            result->error_message = "Send gripper_B move command failed";
            terminal.abort(result);
            return;
        }

        while (true)
        {

            // 休眠20ms

            rclcpp::sleep_for(std::chrono::milliseconds(20));

            if (goal_handle->is_canceling()) {
                result->success = ACTION_FAIL;
                result->error_message = "ControlGripper goal canceled by client";
                terminal.cancel(result);
                return;
            }

            /* 超时检查 */
            if (std::chrono::steady_clock::now() - start_time >= timeout)
            {
                RCLCPP_INFO(rclcpp::get_logger("timeout"), "timeout exit");
                result->success = ACTION_FAIL;
                result->error_message = "time_out";
                terminal.abort(result);
                return;
            }


            ret = jodell_control->Jodell_Status_B(&rt_status[GRIPPER_RIGHT]);

            if (!ret)
            {
                result->success = ACTION_FAIL;
                result->error_message = "Get gripper_B status failed while moving";
                terminal.abort(result);
                return;
            }
            else
            {
                if(rt_status[GRIPPER_RIGHT].gto == 0)
                {
                    control_gripper_result(
                        rt_status, goal->gripper_id, result, terminal);
                    break;
                }
                else
                {
                    feedback->current_position[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].position;
                    feedback->current_torque[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].torque;
                    feedback->current_speed[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].speed;
                    feedback->progress_rate = get_progress_1000(init_position[GRIPPER_RIGHT], goal->position[GRIPPER_RIGHT], rt_status[GRIPPER_RIGHT].position);
                    if (auto node = lockNode(__func__)) {
                        feedback->header.stamp = node->now();
                    }
                    goal_handle->publish_feedback(feedback);
                }
            }
           
            

        }
    }
    else if (goal->gripper_id == GRIPPER_BOTH)
    {
        jodell_control->Jodell_modbus_config_A(goal->com_port[0], 9);

        jodell_control->Jodell_modbus_config_B(goal->com_port[1], 9);
        bool ret = jodell_control->Jodell_Status_A(&rt_status[GRIPPER_LEFT]);
        if (ret)
        {
            init_position[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].position;
        }
        else
        {
            result->success = ACTION_FAIL;
            result->error_message = "Get gripper_A status failed";
            RCLCPP_ERROR(rclcpp::get_logger("Jodell_Status_A"), "Get gripper_A status failed");
            terminal.abort(result);
            return;
        }

        ret = jodell_control->Jodell_Status_B(&rt_status[GRIPPER_RIGHT]);
        if (ret)
        {
            init_position[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].position;
        }
        else
        {
            result->success = ACTION_FAIL;
            result->error_message = "Get gripper_B status failed";
            RCLCPP_ERROR(rclcpp::get_logger("Jodell_Status_B"), "Get gripper_B status failed");
            terminal.abort(result);
            return;
        }

        if (!jodell_control->Jodell_Move_Abs_A(
                goal->position[GRIPPER_LEFT],
                goal->max_torque[GRIPPER_LEFT],
                goal->speed[GRIPPER_LEFT]))
        {
            result->success = ACTION_FAIL;
            result->error_message = "Send gripper_A move command failed";
            terminal.abort(result);
            return;
        }
        if (!jodell_control->Jodell_Move_Abs_B(
                goal->position[GRIPPER_RIGHT],
                goal->max_torque[GRIPPER_RIGHT],
                goal->speed[GRIPPER_RIGHT]))
        {
            result->success = ACTION_FAIL;
            result->error_message =
                "Send gripper_B move command failed after gripper_A command";
            terminal.abort(result);
            return;
        }

        while (true)
        {

            // 休眠20ms

            rclcpp::sleep_for(std::chrono::milliseconds(20));

            if (goal_handle->is_canceling()) {
                result->success = ACTION_FAIL;
                result->error_message = "ControlGripper goal canceled by client";
                terminal.cancel(result);
                return;
            }

            /* 超时检查 */
            if (std::chrono::steady_clock::now() - start_time >= timeout)
            {
                RCLCPP_INFO(rclcpp::get_logger("timeout"), "timeout exit");
                result->success = ACTION_FAIL;
                result->error_message = "time_out";
                terminal.abort(result);
                return;
            }


            ret = jodell_control->Jodell_Status_A(&rt_status[GRIPPER_LEFT]);
            ret &= jodell_control->Jodell_Status_B(&rt_status[GRIPPER_RIGHT]);
            if (!ret)
            {
                result->success = ACTION_FAIL;
                result->error_message =
                    "Get gripper status failed while moving both grippers";
                terminal.abort(result);
                return;
            }
            else
            {
                if(rt_status[GRIPPER_LEFT].gto == 0 && rt_status[GRIPPER_RIGHT].gto == 0)
                {
                    control_gripper_result(
                        rt_status, goal->gripper_id, result, terminal);
                    break;
                }
                else
                {
                    feedback->current_position[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].position;
                    feedback->current_torque[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].torque;
                    feedback->current_speed[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].speed;
                    feedback->current_position[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].position;
                    feedback->current_torque[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].torque;
                    feedback->current_speed[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].speed;
                    const float left_progress = get_progress_1000(
                        init_position[GRIPPER_LEFT], goal->position[GRIPPER_LEFT],
                        rt_status[GRIPPER_LEFT].position);
                    const float right_progress = get_progress_1000(
                        init_position[GRIPPER_RIGHT], goal->position[GRIPPER_RIGHT],
                        rt_status[GRIPPER_RIGHT].position);
                    feedback->progress_rate =
                        0.5F * (left_progress + right_progress);
                    if (auto node = lockNode(__func__)) {
                        feedback->header.stamp = node->now();
                    }
                    goal_handle->publish_feedback(feedback);
                }
            }
           
            

        }
    }
    else
    {
        result->success = ACTION_FAIL;
        result->error_message = "Unsupported gripper_id";
        terminal.abort(result);
        return;
    }
    /***测试****/
    // Eigen::Matrix4d T_test = Eigen::Matrix4d::Identity();
    // const bool fk_test_ok = miracle_algo_->getForwardKinematics(
    //      "head_pitch_link", "body_link",T_test);
    // if (!fk_test_ok) {
    //     RCLCPP_ERROR(
    //         rclcpp::get_logger("DualArmModule_FKTest"),
    //         "Failed to get forward kinematics from body_link to "
    //         "head_pitch_link");
    // } else {
    //     RCLCPP_INFO(
    //         rclcpp::get_logger("DualArmModule_FKTest"),
    //         "T_test (head_pitch_link -> body_link): "
    //         "[[%.9f, %.9f, %.9f, %.9f], "
    //         "[%.9f, %.9f, %.9f, %.9f], "
    //         "[%.9f, %.9f, %.9f, %.9f], "
    //         "[%.9f, %.9f, %.9f, %.9f]]",
    //         T_test(0, 0), T_test(0, 1), T_test(0, 2), T_test(0, 3),
    //         T_test(1, 0), T_test(1, 1), T_test(1, 2), T_test(1, 3),
    //         T_test(2, 0), T_test(2, 1), T_test(2, 2), T_test(2, 3),
    //         T_test(3, 0), T_test(3, 1), T_test(3, 2), T_test(3, 3));
    // }


}

// -----------------------------------------//

 bool DualArmModule::init_arm_params()
 {
    bool res = false;
    std::string mutable_config_path = sdk_config_path_;
    if (LOADMvCfg(
        mutable_config_path.data(), type_, grv_, dh_, pnva_, bd_, mass_,
        mcp_, i_) == FX_FALSE)
    {
       RCLCPP_ERROR(
           rclcpp::get_logger("dual_arm_module"),
           "Failed to load Marvin config: %s", sdk_config_path_.c_str());
       return res;
    }
    if (FX_Robot_Init_Type(0, type_[0]) == FX_FALSE ||
        FX_Robot_Init_Type(1, type_[1]) == FX_FALSE)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("dual_arm_module"),
            "Failed to initialize Marvin robot types");
        return res;
    }
    if (FX_Robot_Init_Kine(0, dh_[0]) == FX_FALSE ||
        FX_Robot_Init_Kine(1, dh_[1]) == FX_FALSE)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("dual_arm_module"),
            "Failed to initialize Marvin kinematics");
        return res;
    }
    if (FX_Robot_Init_Lmt(0, pnva_[0], bd_[0]) == FX_FALSE ||
        FX_Robot_Init_Lmt(1, pnva_[1], bd_[1]) == FX_FALSE)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("dual_arm_module"),
            "Failed to initialize Marvin joint limits");
        return res;
    }

    for(int i =0;i<arm_dof_;i++)
    {

      l_q_lower_limits_[i] = pnva_[0][i][1]*motion_control_pkg::kinematics::kDegreesToRadians;
      r_q_lower_limits_[i] = pnva_[1][i][1]*motion_control_pkg::kinematics::kDegreesToRadians;

      l_q_upper_limits_[i] = pnva_[0][i][0]*motion_control_pkg::kinematics::kDegreesToRadians;
      r_q_upper_limits_[i] = pnva_[1][i][0]*motion_control_pkg::kinematics::kDegreesToRadians;

    //   std::cout<<"i:="<<i<<","<<"l:="<< l_q_lower_limits_[i]<<","<<l_q_upper_limits_[i]<<std::endl;
    //   std::cout<<"i:="<<i<<","<<"r:="<< r_q_lower_limits_[i]<<","<<r_q_upper_limits_[i]<<std::endl;

      l_v_limit_[i] = pnva_[0][i][2]*motion_control_pkg::kinematics::kDegreesToRadians;
      r_v_limit_[i] = pnva_[1][i][2]*motion_control_pkg::kinematics::kDegreesToRadians;

    //   std::cout<<"i:="<<i<<","<<"v:="<< l_v_limit_[i]<<","<<r_v_limit_[i]<<std::endl;
      
      l_a_limit_[i] = pnva_[0][i][3]*motion_control_pkg::kinematics::kDegreesToRadians * acceleration_scale_;
      r_a_limit_[i] = pnva_[1][i][3]*motion_control_pkg::kinematics::kDegreesToRadians * acceleration_scale_;

    //   std::cout<<"i:="<<i<<","<<"a:="<< l_a_limit_[i]<<","<<r_a_limit_[i]<<std::endl;
      
    

      // 加速度在上一行已经由度制转换为 rad/s²；jerk 只按倍数生成，
      // 不得再次乘 DEG2RAD，否则约束会被错误缩小约 57.3 倍。
      l_jerk_limit_[i] = 5.0 * pnva_[0][i][3] * motion_control_pkg::kinematics::kDegreesToRadians * jerk_scale_;
      r_jerk_limit_[i] = 5.0 * pnva_[1][i][3] * motion_control_pkg::kinematics::kDegreesToRadians * jerk_scale_;
      
    //   std::cout<<"i:="<<i<<","<<"j:="<< l_jerk_limit_[i]<<","<<r_jerk_limit_[i]<<std::endl;
    }
    res = true;
    return res;
 }

bool DualArmModule::init_set_tcp(std::string& error_message)
{
    // 复用配置中的同一份 end_T_tcp：既用于状态 FK，也用于厂商 IK 的工具设置，
    // 避免保留另一组硬编码 TCP 参数。
    const auto set_tool = [&error_message](
        int arm_index,
        const char* arm_name,
        const Eigen::Isometry3d& model_end_to_tool_tcp) {
          Matrix4 vendor_tool{};
          const auto vendor_matrix =
              motion_control_pkg::kinematics::toVendorMillimeterMatrix(
              model_end_to_tool_tcp);
          for (std::size_t row = 0; row < 4U; ++row) {
            for (std::size_t column = 0; column < 4U; ++column) {
              vendor_tool[row][column] = vendor_matrix[row][column];
            }
          }
          if (FX_Robot_Tool_Set(arm_index, vendor_tool) == FX_FALSE) {
            error_message = std::string("Failed to configure ") + arm_name +
              " tool transform in Marvin kinematics";
            return false;
          }
          return true;
        };

    if (motion_control_pkg::sdk::isArmEnabled(
        arm_configuration_, motion_control_pkg::sdk::ArmChannel::kLeft) &&
        !set_tool(0, "left", left_model_end_to_tool_tcp_))
    {
        return false;
    }
    if (motion_control_pkg::sdk::isArmEnabled(
        arm_configuration_, motion_control_pkg::sdk::ArmChannel::kRight) &&
        !set_tool(1, "right", right_model_end_to_tool_tcp_))
    {
        return false;
    }

    error_message.clear();
    return true;
}



bool DualArmModule::set_arm_control_mode(std::string& error_message)
{
    const auto result = sdk_command_gate_->initializePositionMode(100, 60);
    if (!result) {
        error_message = result.error_message;
        return false;
    }
    error_message.clear();
    return true;
}



void DualArmModule::log_jodell_status(rclcpp::Logger logger, const Jodell_status& status) {
    RCLCPP_INFO(logger, "=== Jodell Status [0] ===");
    
    // 打印故障位
    RCLCPP_INFO(logger, "Faults: Active[%d] Ctrl[%d] Comm[%d] OC[%d] Volt[%d] En[%d] Temp[%d] Self[%d]",
                status.err_active, status.err_control, status.err_comm,
                status.err_over_current, status.err_voltage, status.err_enable,
                status.err_over_temp, status.err_self);

    // 打印实时数据 (注意 uint8_t/int8_t 需要强转为 int 才能打印数字，否则可能打印字符)
    RCLCPP_INFO(logger, "Data: Pos[%d] Spd[%d] Torque[%d] BusV[%d] Temp[%d C]",
                (int)status.position, (int)status.speed, (int)status.torque,
                (int)status.busvoltage, (int)status.temperature);

    // 打印夹爪状态
    RCLCPP_INFO(logger, "Gripper: Stat[%d] Mode[%d] GTO[%d] State[%d] Obj[%d]",
                (int)status.status, (int)status.mode, (int)status.gto,
                (int)status.state, (int)status.obj);
}



/**
 * @brief 千分制进度 0~1000，支持目标小于初始（反向运动）
 * @param init_val 初始 0~255
 * @param tar_val 目标 0~255
 * @param curr_val 当前 0~255
 * @return progress [0,100.0]
 */
float_t DualArmModule::get_progress_1000(int init_val, int tar_val, int curr_val)
{
    int delta_total = tar_val - init_val;
    // 起止一样直接满载
    if (delta_total == 0)
    {
        return 99.0;
    }

    int delta_curr = curr_val - init_val;
    float_t rate = static_cast<double>(delta_curr) / delta_total;
    float_t progress = rate * 100.0;

    // 限幅0~1000
    if (progress < 0.0) progress = 0.0;
    if (progress > 100.0) progress = 100.0;

    return progress;
}

void DualArmModule::control_gripper_result(
    Jodell_status *rt_status,
    std::uint8_t gripper_id,
    std::shared_ptr<ControlGripper::Result> result,
    ControlGripperTerminalGuard& terminal)
{
    result->final_position.assign(2U, 0U);
    result->final_torque.assign(2U, 0U);
    result->error_message.clear();

    const auto append_error = [&result](
        const char* gripper_name, const char* fault_name) {
          if (!result->error_message.empty()) {
              result->error_message += ", ";
          }
          result->error_message += gripper_name;
          result->error_message += ":";
          result->error_message += fault_name;
        };
    const auto collect_status = [&](std::size_t index, const char* name) {
          const auto& status = rt_status[index];
          result->final_position[index] = status.position;
          result->final_torque[index] = status.torque;
          if (status.err_active) {
              append_error(name, "err_active");
          }
          if (status.err_control) {
              append_error(name, "err_control");
          }
          if (status.err_comm) {
              append_error(name, "err_comm");
          }
          if (status.err_over_current) {
              append_error(name, "err_over_current");
          }
          if (status.err_voltage) {
              append_error(name, "err_voltage");
          }
          if (status.err_enable) {
              append_error(name, "err_enable");
          }
          if (status.err_over_temp) {
              append_error(name, "err_over_temp");
          }
          if (status.err_self) {
              append_error(name, "err_self");
          }
        };

    if (gripper_id == GRIPPER_LEFT || gripper_id == GRIPPER_BOTH) {
        collect_status(GRIPPER_LEFT, "left");
    }
    if (gripper_id == GRIPPER_RIGHT || gripper_id == GRIPPER_BOTH) {
        collect_status(GRIPPER_RIGHT, "right");
    }

    if (!result->error_message.empty()) {
        result->success = ACTION_FAIL;
        RCLCPP_ERROR(
            rclcpp::get_logger("control_gripper_result"), "%s",
            result->error_message.c_str());
        terminal.abort(result);
    } else {
        result->success = ACTION_SUCCESS;
        terminal.succeed(result);
    }
}
void DualArmModule::heart_beat_cb()
{
    //每隔一秒发送一次心跳报文
    auto node = lockNode(__func__);
    if (!node) {
        return;
    }
    auto hb_msg = com_interfaces::msg::HeartBeat();
    hb_msg.node_id = MOTION_CONTROL;
    hb_msg.header.stamp = node->get_clock()->now();
    heart_beat_pub_->publish(hb_msg);
}
void DualArmModule::init_motor_status()
{
    motor_status_msg_.joint_names.clear();
    motor_status_msg_.joint_positions.clear();
    motor_status_msg_.joint_velocities.clear();
    motor_status_msg_.joint_efforts.clear();
    motor_status_msg_.end_effector_pose.clear();
    for(const auto&name :MotorName)
    {
       motor_status_msg_.joint_names.push_back(name);
       motor_status_msg_.joint_positions.push_back(0.0);
       motor_status_msg_.joint_velocities.push_back(0.0);
       motor_status_msg_.joint_efforts.push_back(0.0);
    }
    geometry_msgs::msg::Pose identity_pose;
    identity_pose.orientation.w = 1.0;
    motor_status_msg_.end_effector_pose.push_back(identity_pose);
    motor_status_msg_.end_effector_pose.push_back(identity_pose);
}

bool DualArmModule::updateArmModelState(
    const motion_control_pkg::sdk::DualArmDeviceState& state,
    std::vector<double>& left_radians,
    std::vector<double>& right_radians,
    std::string& error_message)
{
    // 两条链必须来自同一份 SDK 状态快照，避免左右臂 FK 时刻不一致。
    if (!motion_control_pkg::kinematics::degreesToRadians(
        state.arms[0].joint_position, left_radians, error_message,
        "left arm"))
    {
        return false;
    }
    if (!motion_control_pkg::kinematics::degreesToRadians(
        state.arms[1].joint_position, right_radians, error_message,
        "right arm"))
    {
        return false;
    }

    std::string left_chain = "left_arm";
    std::string right_chain = "right_arm";
    if (miracle_algo_->updateChainJointsAngle(
        left_chain, left_radians) != 0)
    {
        error_message =
            "Failed to update left_arm Pinocchio joint state from Marvin feedback";
        return false;
    }
    if (miracle_algo_->updateChainJointsAngle(
        right_chain, right_radians) != 0)
    {
        error_message =
            "Failed to update right_arm Pinocchio joint state from Marvin feedback";
        return false;
    }

    error_message.clear();
    return true;
}

bool DualArmModule::computeToolTcpPose(
    const std::string& model_end_frame,
    const Eigen::Isometry3d& model_end_to_tool_tcp,
    geometry_msgs::msg::Pose& tool_tcp_pose,
    std::string& error_message)
{
    // 机械臂内部固定链全部由 URDF/Pinocchio 求解，不再维护安装矩阵副本。
    Eigen::Matrix4d planning_to_model_end;
    if (!miracle_algo_->getForwardKinematics(
        planning_frame_, model_end_frame, planning_to_model_end))
    {
        error_message = "Pinocchio FK failed from " + planning_frame_ +
            " to " + model_end_frame;
        return false;
    }
    if (!planning_to_model_end.allFinite() ||
        !model_end_to_tool_tcp.matrix().allFinite())
    {
        error_message = "Pinocchio FK or configured tool TCP contains non-finite values";
        return false;
    }

    tool_tcp_pose = motion_control_pkg::kinematics::composeToolTcpPose(
        planning_to_model_end, model_end_to_tool_tcp);
    error_message.clear();
    return true;
}

bool DualArmModule::normalizePoseTargetToPlanningFrame(
    const geometry_msgs::msg::PoseStamped& input,
    geometry_msgs::msg::PoseStamped& output,
    std::string& error_message)
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
        error_message.clear();
        return true;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("dual_arm_module"),
        "[normalizePoseTargetToPlanningFrame] before: frame_id="
        << normalized.pose.header.frame_id
        << ", xyz=[" << normalized.pose.pose.position.x << ","
        << normalized.pose.pose.position.y << ","
        << normalized.pose.pose.position.z << "], quat=["
        << normalized.pose.pose.orientation.x << ","
        << normalized.pose.pose.orientation.y << ","
        << normalized.pose.pose.orientation.z << ","
        << normalized.pose.pose.orientation.w << "]");
    std::vector<double> full_q;
    if (!miracle_algo_->getFullJointState(full_q)) {
        error_message =
            "Failed to obtain the Pinocchio full-body state before pose frame conversion";
        return false;
    }
    Eigen::Matrix4d T_planning_source = Eigen::Matrix4d::Identity();
    if (!miracle_algo_->getForwardKinematics(
        planning_frame_, normalized.pose.header.frame_id, T_planning_source))
    {
        error_message =
            "Pinocchio FK failed for pose conversion from " +
            planning_frame_ + " to " + normalized.pose.header.frame_id;
        return false;
    }
    Eigen::Isometry3d T_planning_source_iso;
    T_planning_source_iso.matrix() = T_planning_source;
    if (!T_planning_source_iso.matrix().allFinite()) {
        error_message =
            "Pinocchio FK pose conversion matrix contains non-finite values";
        return false;
    }
    const Eigen::Isometry3d source_to_point =
        motion_control_pkg::kinematics::poseToIsometry(
        normalized.pose.pose);
    const Eigen::Isometry3d planning_to_point =
        T_planning_source_iso * source_to_point;
    output.header = normalized.pose.header;
    output.header.frame_id = planning_frame_;
    output.pose = motion_control_pkg::kinematics::isometryToPose(
        planning_to_point);
    const auto final_check =
        motion_control_pkg::kinematics::validateAndNormalizePoseStamped(output);
    if (!final_check.success) {
        error_message =
            "Pose after internal frame conversion is invalid: " +
            final_check.error_message;
        return false;
    }
    output = final_check.pose;
    output.header.frame_id = planning_frame_;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("dual_arm_module"),
        "[normalizePoseTargetToPlanningFrame] after : frame_id="
        << output.header.frame_id
        << ", xyz=[" << output.pose.position.x << ","
        << output.pose.position.y << ","
        << output.pose.position.z << "], quat=["
        << output.pose.orientation.x << ","
        << output.pose.orientation.y << ","
        << output.pose.orientation.z << ","
        << output.pose.orientation.w << "]");
    error_message.clear();
    return true;
}

bool DualArmModule::readMeasuredFeedback(
    sensor_msgs::msg::JointState& output, std::string& error)
{
    const auto node = lockNode(__func__);
    if (!node || !init_robot_link_ || !feedback_bus_) {
        error = "hardware feedback sources not initialized";
        return false;
    }
    // Dedicated acquisition path: no model update, FK, collision scene or
    // MotorInfo fallback. A planner can hold the model mutex for seconds.
    const auto arm_acquired_ros = node->now();
    const auto arm_acquired_steady = std::chrono::steady_clock::now();
    motion_control_pkg::sdk::DualArmDeviceState arms;
    const auto result = sdk_command_gate_->readState(arms);
    if (!result) {
        error = "arm SDK read failed: " + result.error_message;
        return false;
    }
    std::vector<DriverSDK::motorActualStruct> motors;
    std::chrono::steady_clock::time_point bus_acquired;
    uint64_t bus_sequence = 0;
    if (!feedback_bus_->getTimedMotorActualSnapshot(motors, bus_acquired, bus_sequence)) {
        error = "no valid EtherCAT acquisition";
        return false;
    }
    const auto bus_older_ns = std::max<int64_t>(0,
        std::chrono::duration_cast<std::chrono::nanoseconds>(arm_acquired_steady-bus_acquired).count());
    return measured_feedback_.build(arms, motors, bus_sequence,
        arm_acquired_ros.nanoseconds()-bus_older_ns, node->now().nanoseconds(), output, error);
}

void DualArmModule::motor_statu_cb()
{
    auto node = lockNode(__func__);
    if (!node) {
      return;
    }
    motor_status_msg_.header.frame_id = planning_frame_;
    // 查验连接是否成功
    if (!init_robot_link_) {
      init_motor_status();
      motor_status_msg_.header.frame_id = planning_frame_;
      motor_status_msg_.header.stamp = node->get_clock()->now();
      motor_status_pub_->publish(motor_status_msg_);
    } else {
      motion_control_pkg::sdk::DualArmDeviceState latest_state;
      const auto state_result = sdk_command_gate_->readState(latest_state);
      if (!state_result) {
        RCLCPP_ERROR(
            node->get_logger(), "Read dual arm state failed: %s",
            state_result.error_message.c_str());
        init_motor_status();
        motor_status_msg_.header.frame_id = planning_frame_;
        motor_status_msg_.header.stamp = node->get_clock()->now();
        motor_status_pub_->publish(motor_status_msg_);
        return;
      }
      {
        std::lock_guard<std::mutex> lock(arm_status_mtx_);
        dual_arm_state_ = latest_state;
      }
      std::vector<double> left_joint_radians;
      std::vector<double> right_joint_radians;
      std::string model_error;
      if (!updateArmModelState(
          latest_state, left_joint_radians, right_joint_radians, model_error))
      {
        RCLCPP_ERROR(
            node->get_logger(), "Dual-arm model update failed: %s",
            model_error.c_str());
        motor_status_msg_.header.stamp = node->get_clock()->now();
        motor_status_pub_->publish(motor_status_msg_);
        return;
      }

      std::vector<double> left_velocity_radians;
      std::vector<double> right_velocity_radians;
      if (!motion_control_pkg::kinematics::degreesToRadians(
          latest_state.arms[0].joint_velocity, left_velocity_radians,
          model_error, "left arm velocity") ||
          !motion_control_pkg::kinematics::degreesToRadians(
          latest_state.arms[1].joint_velocity, right_velocity_radians,
          model_error, "right arm velocity"))
      {
        RCLCPP_ERROR(
            node->get_logger(), "Dual-arm velocity conversion failed: %s",
            model_error.c_str());
        motor_status_msg_.header.stamp = node->get_clock()->now();
        motor_status_pub_->publish(motor_status_msg_);
        return;
      }

      for (int i = 0; i < 7; i++)
      {
         motor_status_msg_.joint_positions[i] = left_joint_radians[i];
         motor_status_msg_.joint_positions[i+7] = right_joint_radians[i];
         motor_status_msg_.joint_velocities[i] = left_velocity_radians[i];
         motor_status_msg_.joint_velocities[i+7] = right_velocity_radians[i];
         motor_status_msg_.joint_efforts[i] = latest_state.arms[0].joint_effort[i];
         motor_status_msg_.joint_efforts[i+7] = latest_state.arms[1].joint_effort[i];
      }

      geometry_msgs::msg::Pose left_tcp_pose;
      geometry_msgs::msg::Pose right_tcp_pose;
      const bool left_fk_ok = computeToolTcpPose(
          "gripper_base_l_link", left_model_end_to_tool_tcp_,
          left_tcp_pose, model_error);
      if (!left_fk_ok) {
        RCLCPP_ERROR(
            node->get_logger(), "Left arm FK failed: %s", model_error.c_str());
      }
      const bool right_fk_ok = computeToolTcpPose(
          "gripper_base_r_link", right_model_end_to_tool_tcp_,
          right_tcp_pose, model_error);
      if (!right_fk_ok) {
        RCLCPP_ERROR(
            node->get_logger(), "Right arm FK failed: %s", model_error.c_str());
      }
      if (left_fk_ok) {
        motor_status_msg_.end_effector_pose[0] = left_tcp_pose;
      }
      if (right_fk_ok) {
        motor_status_msg_.end_effector_pose[1] = right_tcp_pose;
      }
      motor_status_msg_.header.stamp = node->get_clock()->now();
      motor_status_pub_->publish(motor_status_msg_);
    }
}

void DualArmModule::head_status_cb()
{
    head_status_msg_.end_effector_pose.clear(); 
    head_status_msg_.header.frame_id = planning_frame_;
    std::string model_end_frame = "head_pitch_link";

    auto node = lockNode(__func__);
    if (!node) {
      return;
    }
    // 查验连接是否成功
    if (!init_robot_link_) {
      init_motor_status();
      head_status_msg_.header.frame_id = planning_frame_;
      head_status_msg_.header.stamp = node->get_clock()->now();
      head_status_pub_->publish(head_status_msg_);
      return;
    }
    Eigen::Matrix4d T_source_target = Eigen::Matrix4d::Identity();
    if (!miracle_algo_->getForwardKinematics(planning_frame_, model_end_frame, T_source_target))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get forward kinematics");
        return;
    }

    Eigen::Matrix3d rotation_matrix = T_source_target.block<3,3>(0,0);
    Eigen::Quaterniond quat(rotation_matrix);
    Eigen::Vector3d translation = T_source_target.block<3,1>(0,3);
    geometry_msgs::msg::Pose identity_pose;
    identity_pose.orientation.x = quat.x();
    identity_pose.orientation.y = quat.y();
    identity_pose.orientation.z = quat.z();
    identity_pose.orientation.w = quat.w();
    identity_pose.position.x = translation.x();
    identity_pose.position.y = translation.y();
    identity_pose.position.z = translation.z();
    head_status_msg_.end_effector_pose.push_back(identity_pose);
    head_status_msg_.header.stamp = node->get_clock()->now();
    head_status_pub_->publish(head_status_msg_);

}

sensor_msgs::msg::PointCloud2 DualArmModule::loadPointCloud(const std::string& topic_name) {
    if (topic_name.empty()) return sensor_msgs::msg::PointCloud2{};
    std::unique_lock<std::mutex> lk(latest_cloud_mutex_);
    // 等待最新帧;若已有缓存就直接用,最多等 5 秒
    if (!latest_cloud_ready_) {
        latest_cloud_cv_.wait_for(lk, std::chrono::seconds(5),
                                  [this] { return latest_cloud_ready_; });
    }
    if (!latest_cloud_ready_) {
        throw std::runtime_error("Timeout waiting for synced robot vision on topic: " + topic_name);
    }
    latest_cloud_ready_ = false;  // 取走后下次等新帧(如需"最新"语义可去掉这行)
    return latest_cloud_;
}

void DualArmModule::joint_command_cb()
{

}
