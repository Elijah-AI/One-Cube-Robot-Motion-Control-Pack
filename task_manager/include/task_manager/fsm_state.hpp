#ifndef TASK_MANAGER__FSM_STATE_HPP_
#define TASK_MANAGER__FSM_STATE_HPP_

#include <cstdint>
#include <string>
#include <cstdint>

namespace task_fsm
{
enum class SystemState : int8_t
{
  UNKNOWN = 0,
  INIT = 1,
  SELF_CHECK = 2,
  RESET = 3,
  RESET_DONE = 4,
  IDLE = 5,
  RUNNING = 6,
  ERROR = 7
};

enum ErrorBitMask : uint64_t
{
  ERR_EMERGENCY          = 1ULL << 0,
  ERR_MODBUS_FAULT       = 1ULL << 1,
  ERR_VISION_TIMEOUT     = 1ULL << 2,
  ERR_ARM_PLAN_FAIL      = 1ULL << 3,
  ERR_GRIPPER_FAULT      = 1ULL << 4,
  ERR_CONFIG_PARSE_FAIL  = 1ULL << 5,
  ERR_BT_EXEC_ERROR      = 1ULL << 6,
  ERR_SERVICE_TIMEOUT    = 1ULL << 7
};

std::string state_to_str(SystemState state);
SystemState str_to_state(const std::string& s);
}

#endif