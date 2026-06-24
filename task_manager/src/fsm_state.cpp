#include "task_manager/fsm_state.hpp"

namespace task_fsm
{
std::string state_to_str(SystemState state)
{
  switch(state)
  {
    case SystemState::UNKNOWN: return "UNKNOWN";
    case SystemState::INIT: return "INIT";
    case SystemState::SELF_CHECK: return "SELF_CHECK";
    case SystemState::RESET: return "RESET";
    case SystemState::RESET_DONE: return "RESET_DONE";
    case SystemState::IDLE: return "IDLE";
    case SystemState::RUNNING: return "RUNNING";
    case SystemState::ERROR: return "ERROR";
    default: return "INVALID";
  }
}

SystemState str_to_state(const std::string& s)
{
  if(s == "UNKNOWN") return SystemState::UNKNOWN;
  if(s == "INIT") return SystemState::INIT;
  if(s == "SELF_CHECK") return SystemState::SELF_CHECK;
  if(s == "RESET") return SystemState::RESET;
  if(s == "RESET_DONE") return SystemState::RESET_DONE;
  if(s == "IDLE") return SystemState::IDLE;
  if(s == "RUNNING") return SystemState::RUNNING;
  return SystemState::UNKNOWN;
}
}