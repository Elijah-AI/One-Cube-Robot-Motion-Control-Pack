#ifndef MAIN_NODE_HPP
#define MAIN_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/header.hpp>
// 引入自动生成的Action头文件
#include "com_interfaces/action/pick_and_place.hpp"
#include "com_interfaces/action/hardware_reset.hpp"
#include "com_interfaces/action/hardware_check.hpp"
#include "com_interfaces/action/move_pose.hpp"
#include "com_interfaces/action/control_gripper.hpp"
#include "com_interfaces/action/adjust_base_pose.hpp"
#include "com_interfaces/msg/heart_beat.hpp"
#include "com_interfaces/msg/node_status.hpp"
#include "com_interfaces/msg/motor_status.hpp"
#include "com_interfaces/action/move_joints.hpp"
#include "com_interfaces/action/control_suck.hpp"

#include "jsoncpp/json/json.h"

using PickAndPlace = com_interfaces::action::PickAndPlace;
using GoalHandlePickAndPlace = rclcpp_action::ServerGoalHandle<PickAndPlace>;

using HardwareReset = com_interfaces::action::HardwareReset;
using GoalHandleHardwareReset = rclcpp_action::ServerGoalHandle<HardwareReset>;

using HardwareCheck = com_interfaces::action::HardwareCheck;
using GoalHandleHardwareCheck = rclcpp_action::ServerGoalHandle<HardwareCheck>;

using MovePose = com_interfaces::action::MovePose;
using GoalHandleMovePose = rclcpp_action::ServerGoalHandle<MovePose>;

using ControlSuck = com_interfaces::action::ControlSuck;
using GoalHandleControlSuck = rclcpp_action::ServerGoalHandle<ControlSuck>;

using MoveJoints = com_interfaces::action::MoveJoints;
using GoalHandleMoveJoints = rclcpp_action::ServerGoalHandle<MoveJoints>;

using ControlGripper = com_interfaces::action::ControlGripper;
using GoalHandleControlGripper = rclcpp_action::ServerGoalHandle<ControlGripper>;

using AdjustBasePose = com_interfaces::action::AdjustBasePose;
using GoalHandleAdjustBasePose = rclcpp_action::ServerGoalHandle<AdjustBasePose>;

using HeartBeat = com_interfaces::msg::HeartBeat;
using NodeStatus = com_interfaces::msg::NodeStatus;

using MotorStatus = com_interfaces::msg::MotorStatus;

//EtherCAT从站ID号
#define LEG_JOINT1            4
#define LEG_JOINT2            5
#define LEG_JOINT3            6
#define LEG_JOINT4            7

#define HEAD_YAW_MOTOR_ID     1
#define HEAD_PITCH_MOTOR_ID   2


//节点间通信枚举
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
    ARM_BOTH,
    HEAD,
    LEG
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


//运控节点内，各部件模块枚举0：left_arm - 1：right_arm - 3：head - 4：leg
enum
{
    ID_LEFT_ARM = 0,
    ID_RIGHT_ARM = 1,
    ID_HEAD = 3,
    ID_LEG = 4,
    ID_LEFT_TOOL = 5,
    ID_RIGHT_TOOL = 6,
};



const std::vector<std::string> MotorName = {"Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L", "Joint5_L",
                                            "Joint6_L", "Joint7_L", "Joint1_R", "Joint2_R", "Joint3_R",
                                            "Joint4_R", "Joint5_R", "Joint6_R", "Joint7_R"}; 



#endif // MAIN_NODE_HPP
