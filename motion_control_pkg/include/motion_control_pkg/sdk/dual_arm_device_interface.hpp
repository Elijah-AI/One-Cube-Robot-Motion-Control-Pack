#ifndef MOTION_CONTROL_PKG_SDK_DUAL_ARM_DEVICE_INTERFACE_HPP_
#define MOTION_CONTROL_PKG_SDK_DUAL_ARM_DEVICE_INTERFACE_HPP_

#include <array>
#include <cstddef>
#include <optional>
#include <string>

namespace motion_control_pkg::sdk
{

constexpr std::size_t kArmCount = 2;
constexpr std::size_t kArmDof = 7;

enum class ArmChannel : std::size_t
{
  kLeft = 0,
  kRight = 1,
};

enum class ActiveArmConfiguration
{
  kDualArm,
};

inline std::optional<ActiveArmConfiguration> parseActiveArmConfiguration(
  const std::string & value)
{
  if (value == "dual_arm") {
    return ActiveArmConfiguration::kDualArm;
  }
  return std::nullopt;
}

inline bool isArmEnabled(
  ActiveArmConfiguration configuration, ArmChannel channel)
{
  (void)configuration;
  (void)channel;
  return true;
}

struct ArmFeedback
{
  std::array<double, kArmDof> joint_position{};
  std::array<double, kArmDof> joint_velocity{};
  std::array<double, kArmDof> joint_effort{};
  int current_state{0};
  int error_code{0};
  bool low_speed{false};
};

struct DualArmDeviceState
{
  std::array<ArmFeedback, kArmCount> arms{};
};

/**
 * @brief 双臂设备的厂商无关接口。
 *
 * 上层只依赖这里定义的数据与操作，MarvinSDK 的类型和头文件被限制在
 * Adapter 实现内部。这样既不改变原框架的模块边界，也避免后续更换匹配
 * SDK 版本时把 ABI 细节扩散到控制逻辑。
 */
class DualArmDeviceInterface
{
public:
  virtual ~DualArmDeviceInterface() = default;

  virtual bool checkCompatibility() = 0;
  virtual bool connect(const std::array<unsigned char, 4> & ip) = 0;
  virtual bool release() = 0;
  virtual void disableLogging() = 0;
  virtual bool readState(DualArmDeviceState & state) = 0;

  // 厂商 SDK 规定：以下设置必须位于 begin/commit 事务之间。
  virtual bool beginCommandBatch() = 0;
  virtual bool clearServoError(ArmChannel channel) = 0;
  virtual bool setPositionMode(ArmChannel channel) = 0;
  virtual bool setJointLimits(
    ArmChannel channel, int velocity_ratio, int acceleration_ratio) = 0;
  virtual bool setJointPosition(
    ArmChannel channel, const std::array<double, kArmDof> & joints) = 0;
  virtual bool commitCommandBatch() = 0;
};

}  // namespace motion_control_pkg::sdk

#endif  // MOTION_CONTROL_PKG_SDK_DUAL_ARM_DEVICE_INTERFACE_HPP_
