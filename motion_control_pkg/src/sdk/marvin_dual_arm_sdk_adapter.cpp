#include "motion_control_pkg/sdk/marvin_dual_arm_sdk_adapter.hpp"

// 厂商头文件只允许出现在 Adapter 实现中，公共控制接口不暴露 DCSS。
#include "MarvinSDK.h"

namespace motion_control_pkg::sdk
{

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

}  // namespace motion_control_pkg::sdk
