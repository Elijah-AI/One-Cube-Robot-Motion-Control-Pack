// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Author: gene.wang
// Date: 2026
//
// Description:
// 有限状态机（FSM）状态转换函数实现。
// 提供 SystemState 枚举与字符串之间的双向转换功能。

#include "task_manager/fsm_state.hpp"

namespace task_fsm
{
/// @brief 将系统状态枚举转换为可读字符串
/// @param state 系统状态枚举值
/// @return 对应的字符串描述
std::string state_to_str(SystemState state)
{
  switch(state)
  {
    case SystemState::UNKNOWN:    return "UNKNOWN";
    case SystemState::INIT:       return "INIT";
    case SystemState::SELF_CHECK: return "SELF_CHECK";
    case SystemState::RESET:      return "RESET";
    case SystemState::RESET_DONE: return "RESET_DONE";
    case SystemState::IDLE:       return "IDLE";
    case SystemState::RUNNING:    return "RUNNING";
    case SystemState::ERROR:      return "ERROR";
    case SystemState::EMERGENCY:  return "EMERGENCY";
    default:                      return "INVALID";
  }
}

/// @brief 将字符串转换为系统状态枚举
/// @param s 状态字符串
/// @return 对应的系统状态枚举值，无法识别时返回 UNKNOWN
SystemState str_to_state(const std::string& s)
{
  if(s == "UNKNOWN")    return SystemState::UNKNOWN;
  if(s == "INIT")       return SystemState::INIT;
  if(s == "SELF_CHECK") return SystemState::SELF_CHECK;
  if(s == "RESET")      return SystemState::RESET;
  if(s == "RESET_DONE") return SystemState::RESET_DONE;
  if(s == "IDLE")       return SystemState::IDLE;
  if(s == "RUNNING")    return SystemState::RUNNING;
  if(s == "ERROR")      return SystemState::ERROR;
  if(s == "EMERGENCY")  return SystemState::EMERGENCY;
  return SystemState::UNKNOWN;
}
}
