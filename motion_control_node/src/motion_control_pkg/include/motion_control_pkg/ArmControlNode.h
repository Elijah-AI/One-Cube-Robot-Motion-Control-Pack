#ifndef ARM_CONTROL_NODE_HPP
#define ARM_CONTROL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "Modbus_Jodell.h"
// 引入自动生成的Action头文件
#include "com_interfaces/action/pick_and_place.hpp"
#include "com_interfaces/action/hardware_reset.hpp"
#include "com_interfaces/action/hardware_check.hpp"
#include "com_interfaces/action/move_arm.hpp"
#include "com_interfaces/action/control_gripper.hpp"
#include "com_interfaces/action/adjust_base_pose.hpp"
#include "com_interfaces/msg/heart_beat.hpp"
#include "com_interfaces/msg/node_status.hpp"
#include "com_interfaces/msg/motor_status.hpp"
#include "traj_planner.h"
#include "ompl_wrapper.h"
#include "planner_factory.h"
#include "toppra_planner.h"
#include "ruckig_planner.h"
//引入天机SDK头文件
#include "MarvinSDK.h" 
#include "FxRobot.h"
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

enum
{
    GRIPPER_LEFT =0,
    GRIPPER_RIGHT,
    GRIPPER_BOTH
};

enum
{
    ARM_LEFT =0,
    ARM_RIGHT,
    ARM_BOTH
};

enum
{
    ACTION_FAIL = 0,
    ACTION_SUCCESS,
    
};


enum
{
    TASK_MANAGER        = 1,    // 任务管理节点
    M4_BRIDGE           = 2,    // M4桥接节点
    MOTION_CONTROL      = 3,    // 运控节点
    CAMERA_DRIVER       = 4,    // 相机驱动节点
    DATA_PREPROCESS     = 5,    // 数据预处理节点
    IMU_DRIVER          = 6,    // IMU驱动节点
    VISION_ALGORITHM    = 7,    // 视觉算法节点
    SORT_NODE           = 8,    // 排序节点
    SIX_AXIS_FORCE      = 10,   // 六维力驱动节点
    VLA_NODE            = 11    // VLA节点


};


enum
{
    STATE_UNKNOWN     = 0,   // 未知状态
    STATE_INIT        = 1,   // 初始化
    STATE_SELFCHECK   = 2,   // 自检
    STATE_RESET       = 3,   // 复位
    STATE_IDLE        = 5,   // 空闲
    STATE_RESET_DONE  = 6,   // 复位完成
    STATE_RUNNING     = 7,   // 运行中
    STATE_ERROR       = 8    // 异常故障
}DeviceState;


const std::vector<std::string> MotorName = {"Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L", "Joint5_L",
                                            "Joint6_L", "Joint7_L", "Joint1_R", "Joint2_R", "Joint3_R",
                                            "Joint4_R", "Joint5_R", "Joint6_R", "Joint7_R"}; 

class ArmControlNode : public rclcpp::Node
{
public:
    using PickAndPlace = com_interfaces::action::PickAndPlace;
    using GoalHandlePickAndPlace = rclcpp_action::ServerGoalHandle<PickAndPlace>;

    using HardwareReset = com_interfaces::action::HardwareReset;
    using GoalHandleHardwareReset = rclcpp_action::ServerGoalHandle<HardwareReset>;

    using HardwareCheck = com_interfaces::action::HardwareCheck;
    using GoalHandleHardwareCheck = rclcpp_action::ServerGoalHandle<HardwareCheck>;

    using MoveArm = com_interfaces::action::MoveArm;
    using GoalHandleMoveArm = rclcpp_action::ServerGoalHandle<MoveArm>;

    using ControlGripper = com_interfaces::action::ControlGripper;
    using GoalHandleControlGripper = rclcpp_action::ServerGoalHandle<ControlGripper>;

    using AdjustBasePose = com_interfaces::action::AdjustBasePose;
    using GoalHandleAdjustBasePose = rclcpp_action::ServerGoalHandle<AdjustBasePose>;

    using HeartBeat = com_interfaces::msg::HeartBeat;
    using NodeStatus = com_interfaces::msg::NodeStatus;
 
    using MotorStatus = com_interfaces::msg::MotorStatus;

    explicit ArmControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    void log_jodell_status(rclcpp::Logger logger, const Jodell_status& status);
    
private:
    // --- PickAndPlace Action Handlers ---
    rclcpp_action::GoalResponse handle_goal_pick_and_place(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PickAndPlace::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel_pick_and_place(
        const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);

    void handle_accepted_pick_and_place(
        const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);

    void execute_pick_and_place(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);

    // --- HardwareReset Action Handlers ---
    rclcpp_action::GoalResponse handle_goal_hardware_reset(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const HardwareReset::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel_hardware_reset(
        const std::shared_ptr<GoalHandleHardwareReset> goal_handle);

    void handle_accepted_hardware_reset(
        const std::shared_ptr<GoalHandleHardwareReset> goal_handle);

    void execute_hardware_reset(const std::shared_ptr<GoalHandleHardwareReset> goal_handle);

    rclcpp_action::GoalResponse handle_goal_hardware_check(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const HardwareCheck::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel_hardware_check(
        const std::shared_ptr<GoalHandleHardwareCheck> goal_handle);

    void handle_accepted_hardware_check(
        const std::shared_ptr<GoalHandleHardwareCheck> goal_handle);

    void execute_hardware_check(const std::shared_ptr<GoalHandleHardwareCheck> goal_handle);

    rclcpp_action::GoalResponse handle_goal_move_arm(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveArm::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel_move_arm(
        const std::shared_ptr<GoalHandleMoveArm> goal_handle);

    void handle_accepted_move_arm(
        const std::shared_ptr<GoalHandleMoveArm> goal_handle);

    void execute_move_arm(const std::shared_ptr<GoalHandleMoveArm> goal_handle);
    bool init_arm_params();
    void init_set_tcp();
    rclcpp_action::GoalResponse handle_goal_adjust_base_pose(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const AdjustBasePose::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel_adjust_base_pose(
        const std::shared_ptr<GoalHandleAdjustBasePose> goal_handle);

    void handle_accepted_adjust_base_pose(
        const std::shared_ptr<GoalHandleAdjustBasePose> goal_handle);

    void execute_adjust_base_pose(const std::shared_ptr<GoalHandleAdjustBasePose> goal_handle);

    rclcpp_action::GoalResponse handle_goal_control_gripper(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ControlGripper::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel_control_gripper(
        const std::shared_ptr<GoalHandleControlGripper> goal_handle);

    void handle_accepted_control_gripper(
        const std::shared_ptr<GoalHandleControlGripper> goal_handle);

    void execute_control_gripper(const std::shared_ptr<GoalHandleControlGripper> goal_handle);

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
    rclcpp_action::Server<PickAndPlace>::SharedPtr pick_and_place_action_server_;
    rclcpp_action::Server<HardwareReset>::SharedPtr hardware_reset_action_server_;
    rclcpp_action::Server<HardwareCheck>::SharedPtr hardware_Check_action_server_;
    rclcpp_action::Server<MoveArm>::SharedPtr move_arm_action_server_;
    rclcpp_action::Server<ControlGripper>::SharedPtr control_gripper_action_server_;
    rclcpp_action::Server<AdjustBasePose>::SharedPtr adjust_base_pose_action_server_;
    rclcpp::Publisher<HeartBeat>::SharedPtr heart_beat_pub_;
    rclcpp::Publisher<NodeStatus>::SharedPtr device_status_pub_;
    rclcpp::Publisher<MotorStatus>::SharedPtr motor_status_pub_;
    MotorStatus motor_status_msg_;
    rclcpp::TimerBase::SharedPtr hear_beat_timer_;
    rclcpp::TimerBase::SharedPtr joint_feedback_timer_;
    //rclcpp::TimerBase::SharedPtr joint_command_timer_;
    uint8_t status;
    int64_t error_flag;
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

#endif // ARM_CONTROL_NODE_HPP
