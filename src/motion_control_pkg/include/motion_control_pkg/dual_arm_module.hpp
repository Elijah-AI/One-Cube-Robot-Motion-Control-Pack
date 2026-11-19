#ifndef DUAL_ARM_MODULE_HPP
#define DUAL_ARM_MODULE_HPP


#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "main_node.hpp"
// 双臂通信经统一 Adapter/Gate 访问，公共模块不再直接依赖 MarvinSDK.h。
#include "motion_control_pkg/execution/action_terminal_guard.hpp"
#include "motion_control_pkg/execution/execution_control.hpp"
#include "motion_control_pkg/execution/gripper_worker_threads.hpp"
#include "motion_control_pkg/execution/joint_command.hpp"
#include "motion_control_pkg/sdk/sdk_command_gate.hpp"
#include "motion_control_pkg/wbc/measured_feedback.hpp"
#include "motion_control_pkg/wbc/feedback_channel.hpp"
#include "motion_control_pkg/ethercat_bus_manager.hpp"
#include "FxRobot.h"
#include "Modbus_Jodell.h"
#include "motion_control_pkg/gripper/ds485_ros_bridge.hpp"

using namespace std::chrono_literals;
using Matrix4 = double[4][4];

class MiracleMAlgo;


class DualArmModule
{
public:
    bool uses_ds485_gripper() const { return gripper_type_ == "ds485"; }
    void setGripperAdmission(motion_control_pkg::gripper::Ds485RosBridge::Admission guard) {
        if (ds485_bridge_) ds485_bridge_->setAdmission(std::move(guard));
    }
    void stopActiveGripperTorque() {
        if (ds485_bridge_) ds485_bridge_->stopActiveTorque();
    }
    bool toolActionBusy(unsigned side) const {
        return (side != 1 && is_executing_left_tool_.load()) ||
               (side != 0 && is_executing_right_tool_.load());
    }
    bool validGripperGoal(const ControlGripper::Goal& goal, std::string& error) const {
        return !ds485_bridge_ || ds485_bridge_->validGoal(goal,error);
    }

    motion_control_pkg::sdk::DriveControlResult setTeachMode(unsigned mask, bool enable,
        const std::function<bool()>& canceled);
    bool expectedModeTransition(unsigned arm,const motion_control_pkg::sdk::ArmFeedback& feedback) const {
        return sdk_command_gate_ && arm<2 && sdk_command_gate_->expectedModeTransition(
            static_cast<motion_control_pkg::sdk::ArmChannel>(arm),feedback);
    }
    bool beginMeasuredFeedbackRecoveryEpoch() {
        const auto node=lockNode(__func__);
        if (!node) return false;
        std::lock_guard<std::mutex> lock(measured_feedback_mutex_);
        return measured_feedback_.beginRecoveryEpoch(node->now().nanoseconds());
    }
    motion_control_pkg::sdk::DriveControlResult recoverPositionMode(
        const std::function<bool(const motion_control_pkg::sdk::DualArmDeviceState&,std::string&)>& check,
        const std::function<bool()>& canceled,
        const std::function<bool(const motion_control_pkg::sdk::DualArmDeviceState&,std::string&)>& settling_check = {}) {
        if (!sdk_command_gate_ || !sdk_connected_) return {false,false,"arm SDK is not connected"};
        return sdk_command_gate_->recoverPositionMode(check,canceled,settling_check);
    }
    motion_control_pkg::sdk::DriveControlResult controlDrives(bool left, bool right, bool clear_fault, bool recovery_stop=false,
        const std::function<bool()>& cancel_clear={}) {
        if (!sdk_command_gate_ || !sdk_connected_)
            return {false,false,"arm SDK is not connected"};
        return sdk_command_gate_->controlDrives(left,right,clear_fault,recovery_stop,cancel_clear);
    }
    bool shutdownDriverConnected() const { return sdk_connected_; }
    bool readShutdownFeedback(std::vector<motion_control_pkg::sdk::DriveAxis>& axes, std::string& error) {
        return sdk_connected_ && sdk_command_gate_->readShutdownFeedback(axes,error);
    }
    bool disableForShutdown(std::string& error) {
        if (!sdk_connected_) return true;
        const auto result=sdk_command_gate_->disableForShutdown();
        error=result.error_message; return static_cast<bool>(result);
    }
    bool releaseAfterShutdown(std::string& error) {
        if (ds485_bridge_) ds485_bridge_->shutdown();
        if (!sdk_connected_) return true;
        const auto result=sdk_command_gate_->release();
        error=result.error_message;
        if (result) sdk_connected_=false;
        return static_cast<bool>(result);
    }
    sensor_msgs::msg::PointCloud2 collisionPointCloud() { return loadPointCloud(collision_pointcloud_topic_); }


    explicit DualArmModule(
        const std::string& name,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<motion_control_pkg::sdk::DualArmDeviceInterface> arm_device,
        std::shared_ptr<MiracleMAlgo> miracle_algo,
        motion_control_pkg::sdk::ActiveArmConfiguration arm_configuration,
        std::string sdk_config_path,
        std::string planning_frame,
        Eigen::Isometry3d left_model_end_to_tool_tcp,
        Eigen::Isometry3d right_model_end_to_tool_tcp);

    ~DualArmModule();

    // 设备连接与模式切换显式执行，构造函数不产生硬件通信。
    bool initialize(std::string& error_message);
    void attachMeasuredFeedbackBus(std::shared_ptr<miracle::EthercatBusManager> bus) {
        feedback_bus_=std::move(bus);
    }
    bool read_body_motion_feedback(sensor_msgs::msg::JointState& state, std::string& error) {
        // Planning, first-tick and settling reads can observe the same PDO.
        // Advancement belongs to the publisher and completion/safety guards.
        return readMeasuredFeedback(state, error, false);
    }
    rclcpp::CallbackGroup::SharedPtr measuredFeedbackCallbackGroup() const {
        return measured_feedback_channel_->callbackGroup();
    }

    // 暴露给上层的状态检查
    bool is_idle(int module_id) const;
    
    // 暴露给上层的任务入口, 一个action対应一个接口
    void start_execution_move_pose(
        std::shared_ptr<GoalHandleMovePose> goal_handle,
        geometry_msgs::msg::PoseStamped planning_target);

    void start_execution_move_joints(
        std::shared_ptr<GoalHandleMoveJoints> goal_handle);

    void start_execution_move_dual_arm_joints(
        std::shared_ptr<GoalHandleMoveJoints> goal_handle);

    void start_execution_move_dual_arm_pose(
        std::shared_ptr<GoalHandleMovePose> goal_handle,
        geometry_msgs::msg::PoseStamped left_planning_target,
        geometry_msgs::msg::PoseStamped right_planning_target,
        std::chrono::steady_clock::time_point request_started_at);

    bool start_execution_control_gripper(
        std::shared_ptr<GoalHandleControlGripper> goal_handle,
        std::string& error_message);

    // 执行全身模块已经完成 IK 和轨迹规划的弧度制关节轨迹。
    // 调用者必须先通过 reserve_preplanned_trajectory_resources() 预约资源，
    // 并在执行结束后通过 release_preplanned_trajectory_resources() 释放资源。
    bool execute_preplanned_trajectory(
        const std::vector<std::vector<double>>& left_positions,
        const std::vector<std::vector<double>>& right_positions,
        std::uint8_t control_mode,
        const std::chrono::steady_clock::time_point& start_time,
        const std::shared_ptr<motion_control_pkg::execution::CancellationToken>& cancel,
        std::string& error_message);

    // 预规划轨迹下发拆分为两阶段：
    // 1) prepare_preplanned_dispatch 同步完成单位转换与控制模式配置，
    //    产出的命令缓存在 prepared_dispatch_ 中；控制模式配置会访问硬件；
    // 2) dispatch_preplanned 在 start_time 到来后按 4ms 节拍循环下发。
    // 调用者应在两路 prepare 均成功后再统一设定 start_time，避免准备阶段
    // 耗时吃掉启动预算导致首帧失同步（曾用 now+100ms 硬编码预算不可靠）。
    bool prepare_preplanned_dispatch(
        const std::vector<std::vector<double>>& left_positions,
        const std::vector<std::vector<double>>& right_positions,
        std::uint8_t control_mode,
        std::string& error_message);
    bool dispatch_preplanned(
        const std::chrono::steady_clock::time_point& start_time,
        const std::shared_ptr<motion_control_pkg::execution::CancellationToken>& cancel,
        std::string& error_message);

    // Read controller frames without model/FK access for WBC completion gating.
    bool read_preplanned_completion_feedback(
        motion_control_pkg::sdk::DualArmDeviceState& state, std::string& error);

    // Reserve arm execution flags before whole-body planning starts.
    bool reserve_preplanned_trajectory_resources(
        bool need_left, bool need_right, std::string& error_message);
    void release_preplanned_trajectory_resources(
        bool release_left, bool release_right) noexcept;

    // 返回与现有 MovePose 时间参数化相同的关节运动限幅（弧度制）。
    bool get_joint_motion_limits(
        bool is_left,
        std::vector<double>& velocity,
        std::vector<double>& acceleration,
        std::vector<double>& jerk) const;
    bool start_execution_control_suck(
        std::shared_ptr<GoalHandleControlSuck> goal_handle,
        std::string& error_message);


    void log_jodell_status(rclcpp::Logger logger, const Jodell_status& status);
    
    
private:
    //一个action対应一个接口
    void execution_loop_move_left_pose(
        std::shared_ptr<GoalHandleMovePose> goal_handle,
        geometry_msgs::msg::PoseStamped planning_target);
    void execution_loop_move_right_pose(
        std::shared_ptr<GoalHandleMovePose> goal_handle,
        geometry_msgs::msg::PoseStamped planning_target);
    void execution_loop_move_pose_impl(
        std::shared_ptr<GoalHandleMovePose> goal_handle,
        motion_control_pkg::sdk::ArmChannel channel,
        const geometry_msgs::msg::PoseStamped& planning_target);

    void execution_loop_move_left_joint(
        std::shared_ptr<GoalHandleMoveJoints> goal_handle);
    void execution_loop_move_right_joint(
        std::shared_ptr<GoalHandleMoveJoints> goal_handle);
    void execution_loop_move_joint_impl(
        std::shared_ptr<GoalHandleMoveJoints> goal_handle,
        motion_control_pkg::sdk::ArmChannel channel);
    void execution_loop_move_dual_arm_joints(
        std::shared_ptr<GoalHandleMoveJoints> goal_handle);
    void execution_loop_move_dual_arm_pose(
        std::shared_ptr<GoalHandleMovePose> goal_handle,
        geometry_msgs::msg::PoseStamped left_planning_target,
        geometry_msgs::msg::PoseStamped right_planning_target,
        std::chrono::steady_clock::time_point request_started_at);

    void execution_loop_control_gripper(std::shared_ptr<GoalHandleControlGripper> goal_handle);
    // 自制夹爪（德晟舵机）执行流程，gripper_type 参数为 "ds485" 时使用
    void execution_loop_control_ds485_gripper(std::shared_ptr<GoalHandleControlGripper> goal_handle);
    // 从 ROS 参数读取自制夹爪配置，构造时调用一次
    void load_ds485_gripper_parameters(rclcpp::Node& node);
    void execution_loop_control_suck(std::shared_ptr<GoalHandleControlSuck> goal_handle);

    rclcpp::Node::SharedPtr lockNode(const char *context) const;


    std::string name_;
    rclcpp::Node::WeakPtr node_;
    std::thread execution_thread_move_left_pose_;
    std::thread execution_thread_move_right_pose_;
    std::thread execution_thread_move_left_joint_;
    std::thread execution_thread_move_right_joint_;
    std::thread execution_thread_move_dual_joints_;
    std::thread execution_thread_move_dual_pose_;
    motion_control_pkg::execution::GripperWorkerThreads gripper_worker_threads_;
    std::thread execution_thread_control_suck_;
    std::mutex control_tool_start_mutex_;
    std::mutex suck_gpio_mutex_;

    std::atomic<bool> is_executing_left_arm_;
    std::atomic<bool> is_executing_right_arm_;
    std::atomic<bool> is_executing_left_tool_;
    std::atomic<bool> is_executing_right_tool_;

    // 夹爪类型："jodell" 外购 Jodell 夹爪（默认）；"ds485" 德晟舵机自制夹爪
    std::string gripper_type_{"jodell"};
    std::unique_ptr<motion_control_pkg::gripper::Ds485RosBridge> ds485_bridge_;

    rclcpp::Subscription<RobotVision>::SharedPtr collision_cloud_sub_;
    std::mutex latest_cloud_mutex_;
    sensor_msgs::msg::PointCloud2 latest_cloud_;       // 缓存最新一帧
    std::condition_variable latest_cloud_cv_;
    bool latest_cloud_ready_{false};
    std::thread cloud_spin_thread_;                    // 后台 spin
    std::atomic<bool> cloud_spin_running_{false};

    // --- PickAndPlace Action Handlers ---
    // rclcpp_action::GoalResponse handle_goal_pick_and_place(
    //     const rclcpp_action::GoalUUID & uuid,
    //     std::shared_ptr<const PickAndPlace::Goal> goal);

    // rclcpp_action::CancelResponse handle_cancel_pick_and_place(
    //     const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);

    // void handle_accepted_pick_and_place(
    //     const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);

    // void execute_pick_and_place(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);

    // --- HardwareReset Action Handlers ---
    // rclcpp_action::GoalResponse handle_goal_hardware_reset(
    //     const rclcpp_action::GoalUUID & uuid,
    //     std::shared_ptr<const HardwareReset::Goal> goal);

    // rclcpp_action::CancelResponse handle_cancel_hardware_reset(
    //     const std::shared_ptr<GoalHandleHardwareReset> goal_handle);

    // void handle_accepted_hardware_reset(
    //     const std::shared_ptr<GoalHandleHardwareReset> goal_handle);

    // void execute_hardware_reset(const std::shared_ptr<GoalHandleHardwareReset> goal_handle);

    // rclcpp_action::GoalResponse handle_goal_hardware_check(
    //     const rclcpp_action::GoalUUID & uuid,
    //     std::shared_ptr<const HardwareCheck::Goal> goal);

    // rclcpp_action::CancelResponse handle_cancel_hardware_check(
    //     const std::shared_ptr<GoalHandleHardwareCheck> goal_handle);

    // void handle_accepted_hardware_check(
    //     const std::shared_ptr<GoalHandleHardwareCheck> goal_handle);

    // void execute_hardware_check(const std::shared_ptr<GoalHandleHardwareCheck> goal_handle);

    // rclcpp_action::GoalResponse handle_goal_move_pose(
    //     const rclcpp_action::GoalUUID & uuid,
    //     std::shared_ptr<const MovePose::Goal> goal);

    // rclcpp_action::CancelResponse handle_cancel_move_pose(
    //     const std::shared_ptr<GoalHandleMovePose> goal_handle);

    // void handle_accepted_move_pose(
    //     const std::shared_ptr<GoalHandleMovePose> goal_handle);

    // void execute_move_pose(const std::shared_ptr<GoalHandleMovePose> goal_handle);
    bool init_arm_params();
    bool init_set_tcp(std::string& error_message);
    // rclcpp_action::GoalResponse handle_goal_adjust_base_pose(
    //     const rclcpp_action::GoalUUID & uuid,
    //     std::shared_ptr<const AdjustBasePose::Goal> goal);

    // rclcpp_action::CancelResponse handle_cancel_adjust_base_pose(
    //     const std::shared_ptr<GoalHandleAdjustBasePose> goal_handle);

    // void handle_accepted_adjust_base_pose(
    //     const std::shared_ptr<GoalHandleAdjustBasePose> goal_handle);

    // void execute_adjust_base_pose(const std::shared_ptr<GoalHandleAdjustBasePose> goal_handle);



    using ControlGripperTerminalGuard =
        motion_control_pkg::execution::ActionTerminalGuard<
        GoalHandleControlGripper, ControlGripper::Result>;
    using ControlSuckTerminalGuard =
        motion_control_pkg::execution::ActionTerminalGuard<
        GoalHandleControlSuck, ControlSuck::Result>;
    void control_gripper_result(
        Jodell_status *rt_status,
        std::uint8_t gripper_id,
        std::shared_ptr<ControlGripper::Result> result,
        ControlGripperTerminalGuard& terminal);

    float_t get_progress_1000(int init_val, int tar_val, int curr_val);

    void heart_beat_cb();
    void motor_statu_cb();
    bool readMeasuredFeedback(sensor_msgs::msg::JointState& output, std::string& error,
                              bool require_advancement);
    void head_status_cb();
    void init_motor_status();

    sensor_msgs::msg::PointCloud2 loadPointCloud(const std::string& topic_name);
    // 将一次 SDK 状态快照转换为弧度，并按现有左右臂链名更新 Pinocchio。
    bool updateArmModelState(
        const motion_control_pkg::sdk::DualArmDeviceState& state,
        std::vector<double>& left_radians,
        std::vector<double>& right_radians,
        std::string& error_message);
      bool tryClaimArms(bool need_left, bool need_right, std::string& error_message);
      void releaseArms(bool release_left, bool release_right) noexcept;
    // 使用现有 MiracleMAlgo FK，再拼接配置中的模型末端到工具 TCP 变换。
    bool computeToolTcpPose(
        const std::string& model_end_frame,
        const Eigen::Isometry3d& model_end_to_tool_tcp,
        geometry_msgs::msg::Pose& tool_tcp_pose,
        std::string& error_message);
    // 将外部传入的任意 frame_id PoseStamped 通过 Pinocchio 内部 FK 转换
    // 到 planning_frame_；等价于原 PoseFrameTransformer，但不依赖外部 TF。
    bool normalizePoseTargetToPlanningFrame(
        const geometry_msgs::msg::PoseStamped& input,
        geometry_msgs::msg::PoseStamped& output,
        std::string& error_message);
    motion_control_pkg::sdk::SdkCommandResult configureGoalControlMode(
        std::uint8_t control_mode,
        motion_control_pkg::sdk::ArmControlMode impedance_mode,
        const std::vector<motion_control_pkg::sdk::ArmChannel>& channels);
    void joint_command_cb();
    bool set_arm_control_mode(std::string& error_message);
    // Action ServersHeartBeat
    // rclcpp_action::Server<PickAndPlace>::SharedPtr pick_and_place_action_server_;
    // rclcpp_action::Server<HardwareReset>::SharedPtr hardware_reset_action_server_;
    // rclcpp_action::Server<HardwareCheck>::SharedPtr hardware_Check_action_server_;
    rclcpp_action::Server<MovePose>::SharedPtr move_pose_action_server_;
    rclcpp_action::Server<ControlGripper>::SharedPtr control_gripper_action_server_;
    // rclcpp_action::Server<AdjustBasePose>::SharedPtr adjust_base_pose_action_server_;
    rclcpp::Publisher<HeartBeat>::SharedPtr heart_beat_pub_;
    rclcpp::Publisher<NodeStatus>::SharedPtr device_status_pub_;
    rclcpp::Publisher<MotorStatus>::SharedPtr motor_status_pub_;
    std::unique_ptr<motion_control_pkg::wbc::FeedbackChannel> measured_feedback_channel_;
    std::shared_ptr<miracle::EthercatBusManager> feedback_bus_;
    std::mutex measured_feedback_mutex_;
    motion_control_pkg::wbc::MeasuredFeedbackBuilder measured_feedback_{
        std::array<uint16_t,6>{{LEG_JOINT1,LEG_JOINT2,LEG_JOINT3,BODY_JOINT,HEAD_YAW_MOTOR_ID,HEAD_PITCH_MOTOR_ID}}};
    rclcpp::Publisher<MotorStatus>::SharedPtr head_status_pub_;
    MotorStatus motor_status_msg_;
    MotorStatus head_status_msg_;
    rclcpp::TimerBase::SharedPtr hear_beat_timer_;
    rclcpp::TimerBase::SharedPtr joint_feedback_timer_;
    rclcpp::TimerBase::SharedPtr head_feedback_timer_;
    //rclcpp::TimerBase::SharedPtr joint_command_timer_; 中心控制节点 RobotControlNode.cpp
    //天机Dural Arm data strcture
    FX_INT32L type_[2];
    FX_DOUBLE grv_[2][3];
    FX_DOUBLE dh_[2][8][4];
    FX_DOUBLE pnva_[2][7][4];
    FX_DOUBLE bd_[2][4][3];
    FX_DOUBLE mass_[2][7];
    FX_DOUBLE mcp_[2][7][3];
    FX_DOUBLE i_[2][7][6];
    std::shared_ptr<motion_control_pkg::sdk::DualArmDeviceInterface> arm_device_;
    std::unique_ptr<motion_control_pkg::sdk::SdkCommandGate> sdk_command_gate_;
    // prepare_preplanned_dispatch 产出、dispatch_preplanned 消费的命令缓存。
    // prepare 在主线程同步写入，dispatch 在 async 线程读取；async 启动点
    // 提供 happens-before，无需额外加锁。每次 prepare 清空重填。
    struct PreparedDispatch {
        std::vector<motion_control_pkg::execution::JointCommand> left_commands;
        std::vector<motion_control_pkg::execution::JointCommand> right_commands;
        std::vector<motion_control_pkg::sdk::ArmChannel> channels;
        bool need_left = false;
        bool need_right = false;
        bool valid = false;
    } prepared_dispatch_;
    std::shared_ptr<MiracleMAlgo> miracle_algo_;
    motion_control_pkg::sdk::ActiveArmConfiguration arm_configuration_;
    std::string sdk_config_path_;
    double acceleration_scale_{1.0};
    double jerk_scale_{1.0};
    std::string planning_frame_;
    Eigen::Isometry3d left_model_end_to_tool_tcp_;
    Eigen::Isometry3d right_model_end_to_tool_tcp_;
    std::array<
        motion_control_pkg::sdk::ImpedanceParameters,
        motion_control_pkg::sdk::kArmCount> joint_impedance_parameters_;
    std::array<
        motion_control_pkg::sdk::ImpedanceParameters,
        motion_control_pkg::sdk::kArmCount> cartesian_impedance_parameters_;
    std::array<
        motion_control_pkg::sdk::ToolParameters,
        motion_control_pkg::sdk::kArmCount> tool_parameters_;
    motion_control_pkg::sdk::DualArmDeviceState dual_arm_state_;
    double r_joints_command_[7];//右臂关节命令
    double l_joints_command_[7];//左臂关节命令
    std::atomic<bool> sdk_connected_{false};
    bool init_robot_link_ = false;
    bool suck_gpio_initialized_ = false;
    bool left_suck_active_ = false;
    bool right_suck_active_ = false;
    mutable std::mutex arm_status_mtx_;
      mutable std::mutex arm_claim_mtx_;
      const int arm_dof_ = 7;
    std::vector<double> r_q_lower_limits_,l_q_lower_limits_;
    std::vector<double> r_q_upper_limits_,l_q_upper_limits_;
    std::vector<double> r_v_limit_,l_v_limit_;
    std::vector<double> r_a_limit_,l_a_limit_; 
    std::vector<double> r_jerk_limit_,l_jerk_limit_;
    bool l_init_arm_params_ = false;

    rclcpp::Node::SharedPtr sub_pointcloudNode_;
    std::mutex pointcloud_subscription_mutex_;
    std::string collision_pointcloud_topic_;
};

#endif // DUAL_ARM_MODULE_HPP
