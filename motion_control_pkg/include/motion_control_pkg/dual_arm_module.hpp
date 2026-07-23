#ifndef DUAL_ARM_MODULE_HPP
#define DUAL_ARM_MODULE_HPP


#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "main_node.hpp"
#include "traj_planner.h"
#include "ompl_wrapper.h"
#include "planner_factory.h"
#include "toppra_planner.h"
#include "ruckig_planner.h"

//引入天机SDK头文件
#include "MarvinSDK.h" 
#include "FxRobot.h"
#include "Modbus_Jodell.h"
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <memory>

#define CONTROL_TIME 0.002
#define DEG2RAD M_PI/180.0
#define RAD2DEG 180.0/M_PI

using namespace std::chrono_literals;
using Matrix4 = double[4][4];



class DualArmModule
{
public:


    explicit DualArmModule(const std::string& name, rclcpp::Node::SharedPtr node);

    ~DualArmModule();

    // 暴露给上层的状态检查
    bool is_idle(int module_id) const;
    
    // 暴露给上层的任务入口, 一个action対应一个接口
    void start_execution_move_pose(std::shared_ptr<GoalHandleMovePose> goal_handle);

    void start_execution_control_gripper(std::shared_ptr<GoalHandleControlGripper> goal_handle);


    void log_jodell_status(rclcpp::Logger logger, const Jodell_status& status);
    
    
private:
    //一个action対应一个接口
    void execution_loop_move_left_pose(std::shared_ptr<GoalHandleMovePose> goal_handle);
    void execution_loop_move_right_pose(std::shared_ptr<GoalHandleMovePose> goal_handle);
    void execution_loop_move_dual_pose(std::shared_ptr<GoalHandleMovePose> goal_handle);
    void execution_loop_move_pose_impl(std::shared_ptr<GoalHandleMovePose> goal_handle);

    void execution_loop_control_gripper(std::shared_ptr<GoalHandleControlGripper> goal_handle);

    rclcpp::Node::SharedPtr lockNode(const char *context) const;


    std::string name_;
    rclcpp::Node::WeakPtr node_;
    std::thread execution_thread_move_left_pose_;
    std::thread execution_thread_move_right_pose_;
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
    void init_set_tcp();
    // rclcpp_action::GoalResponse handle_goal_adjust_base_pose(
    //     const rclcpp_action::GoalUUID & uuid,
    //     std::shared_ptr<const AdjustBasePose::Goal> goal);

    // rclcpp_action::CancelResponse handle_cancel_adjust_base_pose(
    //     const std::shared_ptr<GoalHandleAdjustBasePose> goal_handle);

    // void handle_accepted_adjust_base_pose(
    //     const std::shared_ptr<GoalHandleAdjustBasePose> goal_handle);

    // void execute_adjust_base_pose(const std::shared_ptr<GoalHandleAdjustBasePose> goal_handle);



    void control_gripper_result(Jodell_status *rt_status, std::shared_ptr<ControlGripper::Result> result, const std::shared_ptr<GoalHandleControlGripper> goal_handle);

    float_t get_progress_1000(int init_val, int tar_val, int curr_val);

    void heart_beat_cb();
    void motor_statu_cb();
    void init_motor_status();
  
    geometry_msgs::msg::Pose matrix_to_pose(const Matrix4& mat);
    void pose_to_matrix(const geometry_msgs::msg::Pose& pose,Matrix4& mat);
    void joint_command_cb();
    bool check_robot_statu_error();
    void set_arm_control_mode();
    void matrix_multiply(const Matrix4& a, const Matrix4& b, Matrix4& result);
    void print_matrix(Matrix4 a); 
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
    Eigen::Matrix4d result_matrix;
    //天机Dural Arm data strcture
    FX_INT32L type_[2];
    FX_DOUBLE grv_[2][3];
    FX_DOUBLE dh_[2][8][4];
    FX_DOUBLE pnva_[2][7][4];
    FX_DOUBLE bd_[2][4][3];
    FX_DOUBLE mass_[2][7];
    FX_DOUBLE mcp_[2][7][3];
    FX_DOUBLE i_[2][7][6];
    Matrix4 r_tool_; //右臂工具设置
    Matrix4 l_tool_; //左臂工具设置
    Matrix4 l_install_base_inv_;//左臂安装设置
    Matrix4 r_install_base_inv_;//右臂安装设置
    FX_DOUBLE r_current_joint_[7];
    FX_DOUBLE l_current_joint_[7];
    FX_InvKineSolvePara r_sp_;//右臂逆运学数据结构
    FX_InvKineSolvePara l_sp_;//左臂逆运学数据结构
    DCSS dcss_;  //初始化订阅双臂数据的结构体
    double r_joints_command_[7];//右臂关节命令
    double l_joints_command_[7];//左臂关节命令
    geometry_msgs::msg::Pose l_end_effector_pose,r_end_effector_pose;
    bool init_robot_link_;
    bool arm_status_error_ = false;
    mutable std::mutex arm_status_mtx_;
      const int arm_dof_ = 7;
    std::vector<double> r_q_lower_limits_,l_q_lower_limits_;
    std::vector<double> r_q_upper_limits_,l_q_upper_limits_;
    std::vector<double> r_v_limit_,l_v_limit_;
    std::vector<double> r_a_limit_,l_a_limit_; 
    std::vector<double> r_jerk_limit_,l_jerk_limit_;
    TrajectoryResult l_current_traj_;
    rclcpp::Time l_traj_start_traj_;
    bool l_traj_active_ = false;
    int l_step_ = 0;
    bool l_init_arm_params_ ;
    std::unique_ptr<TrajectoryPlanner> traj_ptr_;
    
};

#endif // DUAL_ARM_MODULE_HPP
