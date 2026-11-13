#ifndef MOTION_CONTROL_PKG_SDK_MARVIN_DUAL_ARM_SDK_ADAPTER_HPP_
#define MOTION_CONTROL_PKG_SDK_MARVIN_DUAL_ARM_SDK_ADAPTER_HPP_

#include "motion_control_pkg/sdk/dual_arm_device_interface.hpp"

namespace motion_control_pkg::sdk
{

/**
 * @brief MarvinSDK 到统一双臂设备接口的薄适配层。
 *
 * 本头文件故意不包含 MarvinSDK.h；厂商头文件仅在 .cpp 中出现，防止其
 * 宏、结构体和 ABI 约束污染新整机框架的公共头文件。
 */
class MarvinDualArmSdkAdapter final : public DualArmDeviceInterface
{
public:
  bool checkCompatibility() override;
  bool connect(const std::array<unsigned char, 4> & ip) override;
  bool release() override;
  void disableLogging() override;
  bool readState(DualArmDeviceState & state) override;
  bool emergencyStop(bool left, bool right) override;
  bool readServoError(ArmChannel channel, std::size_t joint, long& error) override;
  bool beginCommandBatch() override;
  bool clearServoError(ArmChannel channel) override;
  bool setDragSpace(ArmChannel channel, int space) override;
  bool setServoOff(ArmChannel channel) override;
  bool setPositionMode(ArmChannel channel) override;
  bool setTorqueMode(ArmChannel channel) override;
  bool setJointLimits(
    ArmChannel channel, int velocity_ratio, int acceleration_ratio) override;
  bool setTool(
    ArmChannel channel, const ToolParameters & parameters) override;
  bool setJointImpedanceGains(
    ArmChannel channel,
    const std::array<double, kArmDof> & stiffness,
    const std::array<double, kArmDof> & damping) override;
  bool setCartesianImpedanceGains(
    ArmChannel channel,
    const std::array<double, kArmDof> & stiffness,
    const std::array<double, kArmDof> & damping) override;
  bool setImpedanceType(
    ArmChannel channel, ArmControlMode mode) override;
  bool setJointPosition(
    ArmChannel channel,
    const std::array<double, kArmDof> & joints) override;
  bool commitCommandBatch() override;
};

}  // namespace motion_control_pkg::sdk

#endif  // MOTION_CONTROL_PKG_SDK_MARVIN_DUAL_ARM_SDK_ADAPTER_HPP_
