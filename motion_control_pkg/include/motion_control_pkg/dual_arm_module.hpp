#ifndef DUAL_ARM_MODULE_HPP
#define DUAL_ARM_MODULE_HPP


#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "main_node.hpp"
#include "traj_planner.h"
#include "planner_factory.h"
#include "toppra_planner.h"
#include "ruckig_planner.h"

// 双臂通信经统一 Adapter/Gate 访问，公共模块不再直接依赖 MarvinSDK.h。
#include "motion_control_pkg/execution/action_terminal_guard.hpp"
#include "motion_control_pkg/execution/arm_action_utils.hpp"
#include "motion_control_pkg/execution/control_period.hpp"
#include "motion_control_pkg/execution/dual_arm_command_coordinator.hpp"
#include "motion_control_pkg/execution/dual_arm_task_arbiter.hpp"
#include "motion_control_pkg/execution/execution_control.hpp"
#include "motion_control_pkg/execution/synchronized_trajectory.hpp"
#include "motion_control_pkg/kinematics/transform_utils.hpp"
#include "motion_control_pkg/motion/joint_space_control.hpp"
#include "motion_control_pkg/motion/pose_space_control.hpp"
#include "motion_control_pkg/sdk/sdk_command_gate.hpp"
#include "FxRobot.h"
#include "Modbus_Jodell.h"
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <memory>

#define DEG2RAD M_PI/180.0
#define RAD2DEG 180.0/M_PI

using namespace std::chrono_literals;
using Matrix4 = double[4][4];

class MiracleMAlgo;


class DualArmModule
{
public:


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

    // 暴露给上层的状态检查
    bool is_idle(int module_id) const;
    
    // 暴露给上层的任务入口, 一个action対应一个接口
    void start_execution_move_pose(
        std::shared_ptr<GoalHandleMovePose> goal_handle,
        geometry_msgs::msg::PoseStamped planning_target,
        motion_control_pkg::execution::ArmTaskLease task_lease);

    void start_execution_move_joints(
        std::shared_ptr<GoalHandleMoveJoints> goal_handle,
        motion_control_pkg::execution::ArmTaskLease task_lease);

    void start_execution_move_dual_arm_joints(
        std::shared_ptr<GoalHandleMoveJoints> goal_handle,
        motion_control_pkg::execution::ArmTaskLease task_lease);

    void start_execution_move_dual_arm_pose(
        std::shared_ptr<GoalHandleMovePose> goal_handle,
        geometry_msgs::msg::PoseStamped left_planning_target,
        geometry_msgs::msg::PoseStamped right_planning_target,
        std::chrono::steady_clock::time_point request_started_at,
        motion_control_pkg::execution::ArmTaskLease task_lease);

    void start_execution_control_gripper(std::shared_ptr<GoalHandleControlGripper> goal_handle);


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

    rclcpp::Node::SharedPtr lockNode(const char *context) const;


    std::string name_;
    rclcpp::Node::WeakPtr node_;
    std::thread execution_thread_move_left_pose_;
    std::thread execution_thread_move_right_pose_;
    std::thread execution_thread_move_left_joint_;
    std::thread execution_thread_move_right_joint_;
    std::thread execution_thread_move_dual_joints_;
    std::thread execution_thread_move_dual_pose_;
    std::thread execution_thread_control_gripper_;

    std::atomic<bool> is_executing_left_arm_;
    std::atomic<bool> is_executing_right_arm_;
    std::atomic<bool> is_executing_left_tool_;
    std::atomic<bool> is_executing_right_tool_;

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
    void control_gripper_result(
        Jodell_status *rt_status,
        std::shared_ptr<ControlGripper::Result> result,
        ControlGripperTerminalGuard& terminal);

    float_t get_progress_1000(int init_val, int tar_val, int curr_val);

    void heart_beat_cb();
    void motor_statu_cb();
    void init_motor_status();
    // 将一次 SDK 状态快照转换为弧度，并按现有左右臂链名更新 Pinocchio。
    bool updateArmModelState(
        const motion_control_pkg::sdk::DualArmDeviceState& state,
        std::vector<double>& left_radians,
        std::vector<double>& right_radians,
        std::string& error_message);
    // 使用现有 MiracleMAlgo FK，再拼接配置中的模型末端到工具 TCP 变换。
    bool computeToolTcpPose(
        const std::string& model_end_frame,
        const Eigen::Isometry3d& model_end_to_tool_tcp,
        geometry_msgs::msg::Pose& tool_tcp_pose,
        std::string& error_message);
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
    MotorStatus motor_status_msg_;
    rclcpp::TimerBase::SharedPtr hear_beat_timer_;
    rclcpp::TimerBase::SharedPtr joint_feedback_timer_;
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
    std::unique_ptr<
        motion_control_pkg::execution::DualArmCommandCoordinator>
        command_coordinator_;
    std::shared_ptr<MiracleMAlgo> miracle_algo_;
    motion_control_pkg::sdk::ActiveArmConfiguration arm_configuration_;
    std::string sdk_config_path_;
    std::string planning_frame_;
    Eigen::Isometry3d left_model_end_to_tool_tcp_;
    Eigen::Isometry3d right_model_end_to_tool_tcp_;
    motion_control_pkg::sdk::DualArmDeviceState dual_arm_state_;
    double r_joints_command_[7];//右臂关节命令
    double l_joints_command_[7];//左臂关节命令
    bool sdk_connected_ = false;
    bool init_robot_link_ = false;
    mutable std::mutex arm_status_mtx_;
      const int arm_dof_ = 7;
    std::vector<double> r_q_lower_limits_,l_q_lower_limits_;
    std::vector<double> r_q_upper_limits_,l_q_upper_limits_;
    std::vector<double> r_v_limit_,l_v_limit_;
    std::vector<double> r_a_limit_,l_a_limit_; 
    std::vector<double> r_jerk_limit_,l_jerk_limit_;
    bool l_init_arm_params_ = false;
    // 左右臂分别持有规划器，禁止把左臂逐关节限制复用于右臂。
    std::unique_ptr<TrajectoryPlanner> l_ruckig_planner_;
    std::unique_ptr<TrajectoryPlanner> r_ruckig_planner_;
    std::unique_ptr<TrajectoryPlanner> l_toppra_planner_;
    std::unique_ptr<TrajectoryPlanner> r_toppra_planner_;
    
};

#endif // DUAL_ARM_MODULE_HPP
