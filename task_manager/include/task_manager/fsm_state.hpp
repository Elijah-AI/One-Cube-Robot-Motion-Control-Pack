// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Author: gene.wang
// Date: 2026
//
// Description:
// 有限状态机（FSM）状态定义头文件。
// 定义系统运行过程中的所有状态枚举（SystemState）和错误位掩码（ErrorBitMask），
// 以及状态与字符串之间的转换函数。

#ifndef TASK_MANAGER__FSM_STATE_HPP_
#define TASK_MANAGER__FSM_STATE_HPP_

#include <cstdint>
#include <string>

namespace task_fsm
{
/// @brief 系统运行状态枚举
enum class SystemState : int8_t
{
  UNKNOWN    = 0,  ///< 未知状态
  INIT       = 1,  ///< 初始化中
  SELF_CHECK = 2,  ///< 自检中
  RESET      = 3,  ///< 复位中
  RESET_DONE = 4,  ///< 复位完成
  IDLE       = 5,  ///< 空闲（待命）
  RUNNING    = 6,  ///< 任务执行中
  ERROR      = 7   ///< 错误状态
};

/// @brief 错误标志位掩码定义
enum ErrorBitMask : uint64_t
{
  ERR_EMERGENCY          = 1ULL << 0,  ///< 紧急停止
  ERR_MODBUS_FAULT       = 1ULL << 1,  ///< Modbus 通信故障
  ERR_VISION_TIMEOUT     = 1ULL << 2,  ///< 视觉检测超时
  ERR_ARM_PLAN_FAIL      = 1ULL << 3,  ///< 机械臂规划失败
  ERR_GRIPPER_FAULT      = 1ULL << 4,  ///< 夹爪故障
  ERR_CONFIG_PARSE_FAIL  = 1ULL << 5,  ///< 配置文件解析失败
  ERR_BT_EXEC_ERROR      = 1ULL << 6,  ///< 行为树执行错误
  ERR_SERVICE_TIMEOUT    = 1ULL << 7   ///< 服务调用超时
};

/// @brief 将状态枚举转换为可读字符串
/// @param state 系统状态枚举值
/// @return 对应的字符串描述
std::string state_to_str(SystemState state);

/// @brief 将字符串转换为状态枚举
/// @param s 状态字符串
/// @return 对应的系统状态枚举值，无法识别时返回 UNKNOWN
SystemState str_to_state(const std::string& s);
}

#endif // TASK_MANAGER__FSM_STATE_HPP_
