// Copyright (c) 2026 Robot Control System
// All rights reserved.
//
// Description:
// FSM 状态转换单元测试。
// 测试 SystemState 枚举与字符串之间的双向转换，
// 包括新增的 EMERGENCY 状态。

#include <gtest/gtest.h>
#include "task_manager/fsm_state.hpp"

using namespace task_fsm;

// ============================================================================
// state_to_str 测试
// ============================================================================

TEST(FsmStateTest, StateToStrKnownStates)
{
  EXPECT_EQ(state_to_str(SystemState::UNKNOWN),    "UNKNOWN");
  EXPECT_EQ(state_to_str(SystemState::INIT),       "INIT");
  EXPECT_EQ(state_to_str(SystemState::SELF_CHECK), "SELF_CHECK");
  EXPECT_EQ(state_to_str(SystemState::RESET),      "RESET");
  EXPECT_EQ(state_to_str(SystemState::RESET_DONE), "RESET_DONE");
  EXPECT_EQ(state_to_str(SystemState::IDLE),       "IDLE");
  EXPECT_EQ(state_to_str(SystemState::RUNNING),    "RUNNING");
  EXPECT_EQ(state_to_str(SystemState::ERROR),      "ERROR");
  EXPECT_EQ(state_to_str(SystemState::EMERGENCY),  "EMERGENCY");
}

TEST(FsmStateTest, StateToStrInvalidState)
{
  // 测试无效枚举值返回 "INVALID"
  EXPECT_EQ(state_to_str(static_cast<SystemState>(99)), "INVALID");
  EXPECT_EQ(state_to_str(static_cast<SystemState>(-1)), "INVALID");
}

// ============================================================================
// str_to_state 测试
// ============================================================================

TEST(FsmStateTest, StrToStateKnownStrings)
{
  EXPECT_EQ(str_to_state("UNKNOWN"),    SystemState::UNKNOWN);
  EXPECT_EQ(str_to_state("INIT"),       SystemState::INIT);
  EXPECT_EQ(str_to_state("SELF_CHECK"), SystemState::SELF_CHECK);
  EXPECT_EQ(str_to_state("RESET"),      SystemState::RESET);
  EXPECT_EQ(str_to_state("RESET_DONE"), SystemState::RESET_DONE);
  EXPECT_EQ(str_to_state("IDLE"),       SystemState::IDLE);
  EXPECT_EQ(str_to_state("RUNNING"),    SystemState::RUNNING);
  EXPECT_EQ(str_to_state("ERROR"),      SystemState::ERROR);
  EXPECT_EQ(str_to_state("EMERGENCY"),  SystemState::EMERGENCY);
}

TEST(FsmStateTest, StrToStateUnknownString)
{
  EXPECT_EQ(str_to_state(""),            SystemState::UNKNOWN);
  EXPECT_EQ(str_to_state("INVALID"),     SystemState::UNKNOWN);
  EXPECT_EQ(str_to_state("emergency"),   SystemState::UNKNOWN);  // 大小写敏感
  EXPECT_EQ(str_to_state("Emergency"),   SystemState::UNKNOWN);
}

// ============================================================================
// 往返转换测试
// ============================================================================

TEST(FsmStateTest, RoundTripConversion)
{
  // 验证 state -> string -> state 的往返一致性
  std::vector<SystemState> all_states = {
    SystemState::UNKNOWN,
    SystemState::INIT,
    SystemState::SELF_CHECK,
    SystemState::RESET,
    SystemState::RESET_DONE,
    SystemState::IDLE,
    SystemState::RUNNING,
    SystemState::ERROR,
    SystemState::EMERGENCY
  };

  for (auto state : all_states)
  {
    std::string str = state_to_str(state);
    SystemState back = str_to_state(str);
    EXPECT_EQ(back, state) << "Round-trip failed for state string: " << str;
  }
}

// ============================================================================
// EMERGENCY 状态特定测试
// ============================================================================

TEST(FsmStateTest, EmergencyStateHasCorrectNumericValue)
{
  // 验证 EMERGENCY = 8，确保与其他状态的数值无冲突
  EXPECT_EQ(static_cast<int8_t>(SystemState::EMERGENCY), 8);

  // 确保不存在数值冲突
  std::set<int8_t> values;
  values.insert(static_cast<int8_t>(SystemState::UNKNOWN));
  values.insert(static_cast<int8_t>(SystemState::INIT));
  values.insert(static_cast<int8_t>(SystemState::SELF_CHECK));
  values.insert(static_cast<int8_t>(SystemState::RESET));
  values.insert(static_cast<int8_t>(SystemState::RESET_DONE));
  values.insert(static_cast<int8_t>(SystemState::IDLE));
  values.insert(static_cast<int8_t>(SystemState::RUNNING));
  values.insert(static_cast<int8_t>(SystemState::ERROR));
  values.insert(static_cast<int8_t>(SystemState::EMERGENCY));

  EXPECT_EQ(values.size(), 9u) << "State enum values must be unique";
}

// ============================================================================
// ErrorBitMask 测试
// ============================================================================

TEST(FsmStateTest, ErrorBitMaskValues)
{
  // 验证错误位掩码定义
  EXPECT_EQ(static_cast<uint64_t>(ERR_EMERGENCY),     1ULL << 0);
  EXPECT_EQ(static_cast<uint64_t>(ERR_MODBUS_FAULT),   1ULL << 1);
  EXPECT_EQ(static_cast<uint64_t>(ERR_VISION_TIMEOUT), 1ULL << 2);
  EXPECT_EQ(static_cast<uint64_t>(ERR_ARM_PLAN_FAIL),  1ULL << 3);
  EXPECT_EQ(static_cast<uint64_t>(ERR_GRIPPER_FAULT),  1ULL << 4);

  // 验证位掩码互不重叠
  uint64_t all_bits = ERR_EMERGENCY | ERR_MODBUS_FAULT | ERR_VISION_TIMEOUT
                    | ERR_ARM_PLAN_FAIL | ERR_GRIPPER_FAULT
                    | ERR_CONFIG_PARSE_FAIL | ERR_BT_EXEC_ERROR | ERR_SERVICE_TIMEOUT;
  int bit_count = __builtin_popcountll(all_bits);
  EXPECT_EQ(bit_count, 8) << "All error bit masks should occupy distinct bits";
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
