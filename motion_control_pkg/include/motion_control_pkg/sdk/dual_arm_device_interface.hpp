#ifndef MOTION_CONTROL_PKG_SDK_DUAL_ARM_DEVICE_INTERFACE_HPP_
#define MOTION_CONTROL_PKG_SDK_DUAL_ARM_DEVICE_INTERFACE_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>

namespace motion_control_pkg::sdk
{

constexpr std::size_t kArmCount = 2;
constexpr std::size_t kArmDof = 7;
constexpr std::size_t kToolKinematicsSize = 6;
constexpr std::size_t kToolDynamicsSize = 10;

enum class ArmChannel : std::size_t
{
  kLeft = 0,
  kRight = 1,
};

enum class ArmControlMode : int
{
  kPosition = 0,
  kJointImpedance = 1,
  kCartesianImpedance = 2,
};

struct ImpedanceParameters
{
  std::array<double, kArmDof> stiffness{};
  std::array<double, kArmDof> damping{};
  int velocity_ratio{10};
  int acceleration_ratio{10};
};

struct ToolParameters
{
  // SDK 顺序：XYZABC，平移单位 mm，转角单位 deg。
  std::array<double, kToolKinematicsSize> kinematics{};
  // SDK 顺序：M, MCP_X, MCP_Y, MCP_Z, I_XX, I_XY, I_XZ,
  // I_YY, I_YZ, I_ZZ；MCP 相对法兰，单位 mm。
  std::array<double, kToolDynamicsSize> dynamics{};
};

inline bool isValidToolParameters(const ToolParameters & parameters)
{
  for (const double value : parameters.kinematics) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (const double value : parameters.dynamics) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  if (parameters.dynamics[0] <= 0.0) {
    return false;
  }

  const double i_xx = parameters.dynamics[4];
  const double i_xy = parameters.dynamics[5];
  const double i_xz = parameters.dynamics[6];
  const double i_yy = parameters.dynamics[7];
  const double i_yz = parameters.dynamics[8];
  const double i_zz = parameters.dynamics[9];
  constexpr double kTolerance = 1.0e-12;
  if (i_xx < 0.0 || i_yy < 0.0 || i_zz < 0.0 ||
    i_xx + i_yy + kTolerance < i_zz ||
    i_xx + i_zz + kTolerance < i_yy ||
    i_yy + i_zz + kTolerance < i_xx)
  {
    return false;
  }
  const double xy_minor = i_xx * i_yy - i_xy * i_xy;
  const double xz_minor = i_xx * i_zz - i_xz * i_xz;
  const double yz_minor = i_yy * i_zz - i_yz * i_yz;
  const double determinant =
    i_xx * (i_yy * i_zz - i_yz * i_yz) -
    i_xy * (i_xy * i_zz - i_xz * i_yz) +
    i_xz * (i_xy * i_yz - i_xz * i_yy);
  return xy_minor >= -kTolerance && xz_minor >= -kTolerance &&
         yz_minor >= -kTolerance && determinant >= -kTolerance;
}

struct ArmControlModeCommand
{
  ArmChannel channel{ArmChannel::kLeft};
  ArmControlMode mode{ArmControlMode::kPosition};
  ImpedanceParameters impedance{};
  ToolParameters tool{};
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
  std::array<double, kToolKinematicsSize> tool_kinematics{};
  std::array<double, kToolDynamicsSize> tool_dynamics{};
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
  virtual bool setTorqueMode(ArmChannel channel) = 0;
  virtual bool setJointLimits(
    ArmChannel channel, int velocity_ratio, int acceleration_ratio) = 0;
  virtual bool setTool(
    ArmChannel channel, const ToolParameters & parameters) = 0;
  virtual bool setJointImpedanceGains(
    ArmChannel channel,
    const std::array<double, kArmDof> & stiffness,
    const std::array<double, kArmDof> & damping) = 0;
  virtual bool setCartesianImpedanceGains(
    ArmChannel channel,
    const std::array<double, kArmDof> & stiffness,
    const std::array<double, kArmDof> & damping) = 0;
  virtual bool setImpedanceType(
    ArmChannel channel, ArmControlMode mode) = 0;
  virtual bool setJointPosition(
    ArmChannel channel, const std::array<double, kArmDof> & joints) = 0;
  virtual bool commitCommandBatch() = 0;
};

}  // namespace motion_control_pkg::sdk

#endif  // MOTION_CONTROL_PKG_SDK_DUAL_ARM_DEVICE_INTERFACE_HPP_
