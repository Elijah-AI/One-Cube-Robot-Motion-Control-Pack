#include "motion_control_pkg/sdk/marvin_dual_arm_sdk_adapter.hpp"

#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>

#if defined(__aarch64__)
// 厂商头文件只允许出现在 Adapter 实现中，公共控制接口不暴露 DCSS。
#include "MarvinSDK.h"
#endif

namespace motion_control_pkg::sdk
{

#if defined(__aarch64__)

bool MarvinDualArmSdkAdapter::checkCompatibility()
{
  int byte_order = -1;
  const int result = CheckSDKTypeCompat(&byte_order);
  return result >= 0 && (byte_order == 0 || byte_order == 1);
}

bool MarvinDualArmSdkAdapter::connect(const std::array<unsigned char, 4> & ip)
{
  return OnLinkTo(ip[0], ip[1], ip[2], ip[3]);
}

bool MarvinDualArmSdkAdapter::release()
{
  return OnRelease();
}

void MarvinDualArmSdkAdapter::disableLogging()
{
  OnLogOff();
  OnLocalLogOff();
}

bool MarvinDualArmSdkAdapter::readState(DualArmDeviceState & state)
{
  DCSS vendor_state{};
  if (!OnGetBuf(&vendor_state)) {
    return false;
  }

  for (std::size_t arm = 0; arm < kArmCount; ++arm) {
    auto & output = state.arms[arm];
    output.current_state = vendor_state.m_State[arm].m_CurState;
    output.error_code = vendor_state.m_State[arm].m_ERRCode;
    output.low_speed = vendor_state.m_Out[arm].m_LowSpdFlag == 1;
    for (std::size_t joint = 0; joint < kArmDof; ++joint) {
      output.joint_position[joint] = vendor_state.m_Out[arm].m_FB_Joint_Pos[joint];
      output.joint_velocity[joint] = vendor_state.m_Out[arm].m_FB_Joint_Vel[joint];
      output.joint_effort[joint] = vendor_state.m_Out[arm].m_FB_Joint_SToq[joint];
    }
  }
  return true;
}

bool MarvinDualArmSdkAdapter::beginCommandBatch()
{
  return OnClearSet();
}

bool MarvinDualArmSdkAdapter::clearServoError(ArmChannel channel)
{
  if (channel == ArmChannel::kLeft) {
    OnClearErr_A();
  } else {
    OnClearErr_B();
  }
  // 当前 SDK 的清错接口无返回值；事务最终结果由 OnSetSend 提供。
  return true;
}

bool MarvinDualArmSdkAdapter::setPositionMode(ArmChannel channel)
{
  return channel == ArmChannel::kLeft ?
         OnSetTargetState_A(1) : OnSetTargetState_B(1);
}

bool MarvinDualArmSdkAdapter::setJointLimits(
  ArmChannel channel, int velocity_ratio, int acceleration_ratio)
{
  return channel == ArmChannel::kLeft ?
         OnSetJointLmt_A(velocity_ratio, acceleration_ratio) :
         OnSetJointLmt_B(velocity_ratio, acceleration_ratio);
}

bool MarvinDualArmSdkAdapter::setJointPosition(
  ArmChannel channel, const std::array<double, kArmDof> & joints)
{
  // 厂商接口历史签名不是 const，使用局部副本保持上层命令不可变。
  auto vendor_joints = joints;
  return channel == ArmChannel::kLeft ?
         OnSetJointCmdPos_A(vendor_joints.data()) :
         OnSetJointCmdPos_B(vendor_joints.data());
}

bool MarvinDualArmSdkAdapter::commitCommandBatch()
{
  return OnSetSend();
}

#else

namespace
{

constexpr int kPositionModeState = 1;
constexpr double kDegreesToRadians = 0.017453292519943295;

struct FakeAdapterRuntime
{
  std::mutex mutex;
  DualArmDeviceState state{};
  std::array<std::array<double, kArmDof>, kArmCount> pending_joints{};
  std::array<bool, kArmCount> pending_joint_valid{{false, false}};
  bool connected{false};
  bool batch_open{false};
  std::size_t batch_id{0};
};

FakeAdapterRuntime & runtime()
{
  static FakeAdapterRuntime instance;
  return instance;
}

std::size_t channelIndex(ArmChannel channel) noexcept
{
  return static_cast<std::size_t>(channel);
}

const char * channelName(ArmChannel channel) noexcept
{
  return channel == ArmChannel::kLeft ? "left_arm" : "right_arm";
}

void printJoints(
  ArmChannel channel, std::size_t batch_id,
  const std::array<double, kArmDof> & joints)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6);
  stream << "[FakeDualArmSdkAdapter] batch=" << batch_id
         << " channel=" << channelName(channel)
         << " joints_rad=[";
  for (std::size_t index = 0; index < joints.size(); ++index) {
    if (index != 0U) {
      stream << ", ";
    }
    stream << (joints[index] * kDegreesToRadians);
  }
  stream << "]";
  std::cout << stream.str() << std::endl;
}

}  // namespace

bool MarvinDualArmSdkAdapter::checkCompatibility()
{
  return true;
}

bool MarvinDualArmSdkAdapter::connect(const std::array<unsigned char, 4> & ip)
{
  auto & fake = runtime();
  std::lock_guard<std::mutex> lock(fake.mutex);
  fake.connected = true;
  fake.batch_open = false;
  fake.pending_joint_valid.fill(false);
  fake.batch_id = 0;
  for (auto & arm : fake.state.arms) {
    arm.current_state = kPositionModeState;
    arm.error_code = 0;
    // x86 fake 设备默认视为静止可规划，否则 MoveJoints/MovePose 会在前置检查中中止。
    arm.low_speed = true;
    arm.joint_position.fill(0.0);
    arm.joint_velocity.fill(0.0);
    arm.joint_effort.fill(0.0);
  }

  std::cout << "[FakeDualArmSdkAdapter] connect to "
            << static_cast<int>(ip[0]) << "."
            << static_cast<int>(ip[1]) << "."
            << static_cast<int>(ip[2]) << "."
            << static_cast<int>(ip[3]) << std::endl;
  return true;
}

bool MarvinDualArmSdkAdapter::release()
{
  auto & fake = runtime();
  std::lock_guard<std::mutex> lock(fake.mutex);
  fake.connected = false;
  fake.batch_open = false;
  fake.pending_joint_valid.fill(false);
  std::cout << "[FakeDualArmSdkAdapter] release" << std::endl;
  return true;
}

void MarvinDualArmSdkAdapter::disableLogging()
{
  std::cout << "[FakeDualArmSdkAdapter] disableLogging ignored" << std::endl;
}

bool MarvinDualArmSdkAdapter::readState(DualArmDeviceState & state)
{
  auto & fake = runtime();
  std::lock_guard<std::mutex> lock(fake.mutex);
  if (!fake.connected) {
    return false;
  }
  state = fake.state;
  return true;
}

bool MarvinDualArmSdkAdapter::beginCommandBatch()
{
  auto & fake = runtime();
  std::lock_guard<std::mutex> lock(fake.mutex);
  if (!fake.connected || fake.batch_open) {
    return false;
  }
  fake.batch_open = true;
  fake.pending_joint_valid.fill(false);
  ++fake.batch_id;
  return true;
}

bool MarvinDualArmSdkAdapter::clearServoError(ArmChannel channel)
{
  auto & fake = runtime();
  std::lock_guard<std::mutex> lock(fake.mutex);
  if (!fake.connected || !fake.batch_open) {
    return false;
  }
  auto & arm = fake.state.arms[channelIndex(channel)];
  arm.error_code = 0;
  // arm.low_speed = false;
  return true;
}

bool MarvinDualArmSdkAdapter::setPositionMode(ArmChannel channel)
{
  auto & fake = runtime();
  std::lock_guard<std::mutex> lock(fake.mutex);
  if (!fake.connected || !fake.batch_open) {
    return false;
  }
  fake.state.arms[channelIndex(channel)].current_state = kPositionModeState;
  return true;
}

bool MarvinDualArmSdkAdapter::setJointLimits(
  ArmChannel channel, int velocity_ratio, int acceleration_ratio)
{
  auto & fake = runtime();
  std::lock_guard<std::mutex> lock(fake.mutex);
  if (!fake.connected || !fake.batch_open) {
    return false;
  }

  std::cout << "[FakeDualArmSdkAdapter] " << channelName(channel)
            << " limits velocity_ratio=" << velocity_ratio
            << " acceleration_ratio=" << acceleration_ratio << std::endl;
  return true;
}

bool MarvinDualArmSdkAdapter::setJointPosition(
  ArmChannel channel, const std::array<double, kArmDof> & joints)
{
  auto & fake = runtime();
  std::lock_guard<std::mutex> lock(fake.mutex);
  if (!fake.connected || !fake.batch_open) {
    return false;
  }
  fake.pending_joints[channelIndex(channel)] = joints;
  fake.pending_joint_valid[channelIndex(channel)] = true;
  return true;
}

bool MarvinDualArmSdkAdapter::commitCommandBatch()
{
  auto & fake = runtime();
  std::lock_guard<std::mutex> lock(fake.mutex);
  if (!fake.connected || !fake.batch_open) {
    return false;
  }

  for (std::size_t index = 0; index < kArmCount; ++index) {
    if (!fake.pending_joint_valid[index]) {
      continue;
    }
    auto & arm = fake.state.arms[index];
    arm.joint_position = fake.pending_joints[index];
    arm.joint_velocity.fill(0.0);
    arm.joint_effort.fill(0.0);
    arm.low_speed = true; 
    printJoints(
      index == channelIndex(ArmChannel::kLeft) ? ArmChannel::kLeft :
      ArmChannel::kRight,
      fake.batch_id, fake.pending_joints[index]);
  }

  fake.batch_open = false;
  fake.pending_joint_valid.fill(false);
  return true;
}

#endif

}  // namespace motion_control_pkg::sdk
