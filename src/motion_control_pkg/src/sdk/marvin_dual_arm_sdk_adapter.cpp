#include "motion_control_pkg/sdk/marvin_dual_arm_sdk_adapter.hpp"

#include <iostream>
#include <mutex>
#include <cstdio>

#if defined(__aarch64__) || defined(ONECUBE_USE_REAL_MARVIN_SDK)
// 厂商头文件只允许出现在 Adapter 实现中，公共控制接口不暴露 DCSS。
#include "MarvinSDK.h"
#endif

namespace motion_control_pkg::sdk
{

#if defined(__aarch64__) || defined(ONECUBE_USE_REAL_MARVIN_SDK)

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
    output.feedback_sequence = vendor_state.m_Out[arm].m_OutFrameSerial;
    output.current_state = vendor_state.m_State[arm].m_CurState;
    output.error_code = vendor_state.m_State[arm].m_ERRCode;
    output.low_speed = vendor_state.m_Out[arm].m_LowSpdFlag == 1;
    output.impedance_type = vendor_state.m_In[arm].m_ImpType;
    output.drag_space = vendor_state.m_In[arm].m_DragSpType;
    output.tip_di = vendor_state.m_Out[arm].m_TipDI;
    output.controller_joint_command.emplace();
    for (std::size_t joint = 0; joint < kArmDof; ++joint) {
      output.joint_position[joint] = vendor_state.m_Out[arm].m_FB_Joint_Pos[joint];
      output.joint_velocity[joint] = vendor_state.m_Out[arm].m_FB_Joint_Vel[joint];
      output.joint_effort[joint] = vendor_state.m_Out[arm].m_FB_Joint_SToq[joint];
      (*output.controller_joint_command)[joint] = vendor_state.m_Out[arm].m_FB_Joint_Cmd[joint];
    }
    for (std::size_t index = 0; index < kToolKinematicsSize; ++index) {
      output.tool_kinematics[index] = vendor_state.m_In[arm].m_ToolKine[index];
    }
    for (std::size_t index = 0; index < kToolDynamicsSize; ++index) {
      output.tool_dynamics[index] = vendor_state.m_In[arm].m_ToolDyn[index];
    }
  }
  return true;
}

bool MarvinDualArmSdkAdapter::beginCommandBatch()
{
  return OnClearSet();
}

bool MarvinDualArmSdkAdapter::emergencyStop(bool left, bool right)
{
  if (!left && !right) return false;
  EStop(left && right ? "AB" : left ? "A" : "B");
  return true;  // void vendor API: readback, not this return, proves the outcome.
}

bool MarvinDualArmSdkAdapter::readServoError(ArmChannel channel, std::size_t joint, long& error)
{
  if (joint>=kArmDof || (channel!=ArmChannel::kLeft && channel!=ArmChannel::kRight)) return false;
  char name[30]{};
  std::snprintf(name,sizeof(name),"SERVO%dERR%zu",channel==ArmChannel::kLeft ? 0 : 1,joint);
  error=-1;
  return OnGetIntPara(name,&error)==0;
}

bool MarvinDualArmSdkAdapter::clearServoError(ArmChannel channel)
{
  if (channel == ArmChannel::kLeft) {
    OnClearErr_A();
  } else {
    OnClearErr_B();
  }
  // RESET uses OnSetIntPara directly. Only fresh readback can confirm clearing.
  return true;
}

bool MarvinDualArmSdkAdapter::setServoOff(ArmChannel channel)
{
  return channel == ArmChannel::kLeft ? OnSetTargetState_A(0) : OnSetTargetState_B(0);
}

bool MarvinDualArmSdkAdapter::setDragSpace(ArmChannel channel, int space)
{
  if (space < 0 || space > 5) return false;
  return channel == ArmChannel::kLeft ? OnSetDragSpace_A(space) : OnSetDragSpace_B(space);
}

bool MarvinDualArmSdkAdapter::setPositionMode(ArmChannel channel)
{
  return channel == ArmChannel::kLeft ?
         OnSetTargetState_A(1) : OnSetTargetState_B(1);
}

bool MarvinDualArmSdkAdapter::setTorqueMode(ArmChannel channel)
{
  return channel == ArmChannel::kLeft ?
         OnSetTargetState_A(3) : OnSetTargetState_B(3);
}

bool MarvinDualArmSdkAdapter::setJointLimits(
  ArmChannel channel, int velocity_ratio, int acceleration_ratio)
{
  return channel == ArmChannel::kLeft ?
         OnSetJointLmt_A(velocity_ratio, acceleration_ratio) :
         OnSetJointLmt_B(velocity_ratio, acceleration_ratio);
}

bool MarvinDualArmSdkAdapter::setTool(
  ArmChannel channel, const ToolParameters & parameters)
{
  auto vendor_kinematics = parameters.kinematics;
  auto vendor_dynamics = parameters.dynamics;
  return channel == ArmChannel::kLeft ?
         OnSetTool_A(vendor_kinematics.data(), vendor_dynamics.data()) :
         OnSetTool_B(vendor_kinematics.data(), vendor_dynamics.data());
}

bool MarvinDualArmSdkAdapter::setJointImpedanceGains(
  ArmChannel channel,
  const std::array<double, kArmDof> & stiffness,
  const std::array<double, kArmDof> & damping)
{
  auto vendor_stiffness = stiffness;
  auto vendor_damping = damping;
  return channel == ArmChannel::kLeft ?
         OnSetJointKD_A(vendor_stiffness.data(), vendor_damping.data()) :
         OnSetJointKD_B(vendor_stiffness.data(), vendor_damping.data());
}

bool MarvinDualArmSdkAdapter::setCartesianImpedanceGains(
  ArmChannel channel,
  const std::array<double, kArmDof> & stiffness,
  const std::array<double, kArmDof> & damping)
{
  auto vendor_stiffness = stiffness;
  auto vendor_damping = damping;
  return channel == ArmChannel::kLeft ?
         OnSetCartKD_A(vendor_stiffness.data(), vendor_damping.data()) :
         OnSetCartKD_B(vendor_stiffness.data(), vendor_damping.data());
}

bool MarvinDualArmSdkAdapter::setImpedanceType(
  ArmChannel channel, ArmControlMode mode)
{
  if (mode != ArmControlMode::kJointImpedance &&
    mode != ArmControlMode::kCartesianImpedance)
  {
    return false;
  }
  const int vendor_type = static_cast<int>(mode);
  return channel == ArmChannel::kLeft ?
         OnSetImpType_A(vendor_type) : OnSetImpType_B(vendor_type);
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

bool MarvinDualArmSdkAdapter::setDragSpace(ArmChannel, int) { return false; }

bool MarvinDualArmSdkAdapter::emergencyStop(bool, bool)
{
  // No physical command/acknowledgement on the pseudo-driver platform.
  return false;
}

bool MarvinDualArmSdkAdapter::readServoError(ArmChannel, std::size_t, long&)
{
  return false;
}

namespace
{

constexpr int kPositionModeState = 1;
constexpr int kTorqueModeState = 3;

struct FakeAdapterRuntime
{
  std::mutex mutex;
  DualArmDeviceState state{};
  std::array<std::array<double, kArmDof>, kArmCount> pending_joints{};
  std::array<bool, kArmCount> pending_joint_valid{{false, false}};
  bool connected{false};
  bool batch_open{false};
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
  for (auto & arm : fake.state.arms) {
    arm.current_state = kPositionModeState;
    arm.error_code = 0;
    // x86 fake 设备默认视为静止可规划，否则 MoveJoints/MovePose 会在前置检查中中止。
    arm.low_speed = true;
    arm.joint_position.fill(0.0);
    arm.joint_velocity.fill(0.0);
    arm.joint_effort.fill(0.0);
    arm.tool_kinematics.fill(0.0);
    arm.tool_dynamics.fill(0.0);
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

bool MarvinDualArmSdkAdapter::setServoOff(ArmChannel channel)
{
  auto& fake=runtime();
  std::lock_guard<std::mutex> lock(fake.mutex);
  if (!fake.connected || !fake.batch_open) return false;
  auto& arm=fake.state.arms[channelIndex(channel)];
  arm.current_state=0; arm.low_speed=true; arm.joint_velocity.fill(0.0);
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

bool MarvinDualArmSdkAdapter::setTorqueMode(ArmChannel channel)
{
  auto & fake = runtime();
  std::lock_guard<std::mutex> lock(fake.mutex);
  if (!fake.connected || !fake.batch_open) {
    return false;
  }
  fake.state.arms[channelIndex(channel)].current_state = kTorqueModeState;
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

bool MarvinDualArmSdkAdapter::setTool(
  ArmChannel channel, const ToolParameters & parameters)
{
  auto & fake = runtime();
  std::lock_guard<std::mutex> lock(fake.mutex);
  if (!fake.connected || !fake.batch_open ||
      !isValidToolParameters(parameters))
  {
    return false;
  }
  auto & arm = fake.state.arms[channelIndex(channel)];
  arm.tool_kinematics = parameters.kinematics;
  arm.tool_dynamics = parameters.dynamics;
  return true;
}

bool MarvinDualArmSdkAdapter::setJointImpedanceGains(
  ArmChannel channel,
  const std::array<double, kArmDof> & stiffness,
  const std::array<double, kArmDof> & damping)
{
  (void)channel;
  (void)stiffness;
  (void)damping;
  auto & fake = runtime();
  std::lock_guard<std::mutex> lock(fake.mutex);
  return fake.connected && fake.batch_open;
}

bool MarvinDualArmSdkAdapter::setCartesianImpedanceGains(
  ArmChannel channel,
  const std::array<double, kArmDof> & stiffness,
  const std::array<double, kArmDof> & damping)
{
  return setJointImpedanceGains(channel, stiffness, damping);
}

bool MarvinDualArmSdkAdapter::setImpedanceType(
  ArmChannel channel, ArmControlMode mode)
{
  (void)channel;
  auto & fake = runtime();
  std::lock_guard<std::mutex> lock(fake.mutex);
  return fake.connected && fake.batch_open &&
         (mode == ArmControlMode::kJointImpedance ||
          mode == ArmControlMode::kCartesianImpedance);
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
  }

  fake.batch_open = false;
  fake.pending_joint_valid.fill(false);
  return true;
}

#endif

}  // namespace motion_control_pkg::sdk
