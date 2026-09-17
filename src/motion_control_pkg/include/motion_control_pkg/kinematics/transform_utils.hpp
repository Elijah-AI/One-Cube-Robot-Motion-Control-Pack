#ifndef MOTION_CONTROL_PKG_KINEMATICS_TRANSFORM_UTILS_HPP_
#define MOTION_CONTROL_PKG_KINEMATICS_TRANSFORM_UTILS_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer_interface.h>

namespace motion_control_pkg::kinematics
{

constexpr double kDegreesToRadians =
  3.141592653589793238462643383279502884 / 180.0;

struct PoseTransformResult
{
  bool success{false};
  geometry_msgs::msg::PoseStamped pose;
  std::string error_message;
};

struct ToolTransformResult
{
  bool success{false};
  Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
  std::string error_message;
};

// 校验 Action 入口的 PoseStamped，并只对合法的非单位四元数做归一化。
// 位置单位沿用 ROS 约定为米；frame_id 和时间戳保持不变，供 tf2 查询使用。
inline PoseTransformResult validateAndNormalizePoseStamped(
  const geometry_msgs::msg::PoseStamped & input)
{
  PoseTransformResult result;
  result.pose = input;

  if (input.header.frame_id.empty()) {
    result.error_message =
      "MovePose target_pose.header.frame_id must not be empty";
    return result;
  }

  const auto & position = input.pose.position;
  const auto & orientation = input.pose.orientation;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
    !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
    !std::isfinite(orientation.w))
  {
    result.error_message =
      "MovePose target_pose contains a non-finite position or orientation";
    return result;
  }

  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (!std::isfinite(norm) || norm <= 1.0e-12) {
    result.error_message =
      "MovePose target_pose quaternion norm must be greater than zero";
    return result;
  }

  result.pose.pose.orientation.x /= norm;
  result.pose.pose.orientation.y /= norm;
  result.pose.pose.orientation.z /= norm;
  result.pose.pose.orientation.w /= norm;
  result.success = true;
  return result;
}

class PoseFrameTransformer
{
public:
  PoseFrameTransformer(
    const tf2_ros::BufferInterface & buffer,
    std::string planning_frame,
    double timeout_seconds)
  : buffer_(buffer),
    planning_frame_(std::move(planning_frame)),
    timeout_(tf2::durationFromSec(timeout_seconds))
  {
  }

  PoseTransformResult transformToPlanningFrame(
    const geometry_msgs::msg::PoseStamped & input) const
  {
    auto normalized = validateAndNormalizePoseStamped(input);
    if (!normalized.success) {
      return normalized;
    }
    if (planning_frame_.empty()) {
      normalized.success = false;
      normalized.error_message = "Dual-arm planning frame must not be empty";
      return normalized;
    }

    if (normalized.pose.header.frame_id == planning_frame_) {
      normalized.pose.header.frame_id = planning_frame_;
      return normalized;
    }

    PoseTransformResult result;
    try {
      // 非零时间戳按请求时刻查询；零时间戳由 tf2 解释为最新可用变换。
      result.pose = buffer_.transform(
        normalized.pose, planning_frame_, timeout_);
    } catch (const tf2::TransformException & exception) {
      result.error_message =
        "TF transform from " + normalized.pose.header.frame_id + " to " +
        planning_frame_ + " failed: " + exception.what();
      return result;
    }

    result = validateAndNormalizePoseStamped(result.pose);
    if (!result.success) {
      result.error_message =
        "TF output in " + planning_frame_ + " is invalid: " +
        result.error_message;
      return result;
    }
    result.pose.header.frame_id = planning_frame_;
    return result;
  }

  const std::string & planningFrame() const {return planning_frame_;}

private:
  const tf2_ros::BufferInterface & buffer_;
  std::string planning_frame_;
  tf2::Duration timeout_;
};

// 工具参数固定为“模型末端坐标系 -> 工具 TCP”，平移单位为米。
// Eigen 负责四元数归一化和旋转矩阵生成，这里不重复实现姿态算法。
inline ToolTransformResult parseToolTransform(
  const std::vector<double> & xyz_quaternion,
  const std::string & parameter_name)
{
  ToolTransformResult result;
  if (xyz_quaternion.size() != 7U) {
    result.error_message = parameter_name +
      " must contain [x, y, z, qx, qy, qz, qw]";
    return result;
  }
  for (double value : xyz_quaternion) {
    if (!std::isfinite(value)) {
      result.error_message = parameter_name + " contains a non-finite value";
      return result;
    }
  }

  Eigen::Quaterniond quaternion(
    xyz_quaternion[6], xyz_quaternion[3], xyz_quaternion[4],
    xyz_quaternion[5]);
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() <= 1.0e-12) {
    result.error_message = parameter_name +
      " quaternion norm must be greater than zero";
    return result;
  }
  quaternion.normalize();

  result.transform = Eigen::Isometry3d::Identity();
  result.transform.translation() = Eigen::Vector3d(
    xyz_quaternion[0], xyz_quaternion[1], xyz_quaternion[2]);
  result.transform.linear() = quaternion.toRotationMatrix();
  result.success = true;
  return result;
}

inline Eigen::Isometry3d poseToIsometry(
  const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y,
    pose.orientation.z);
  quaternion.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  transform.linear() = quaternion.toRotationMatrix();
  return transform;
}

inline geometry_msgs::msg::Pose isometryToPose(
  const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  Eigen::Quaterniond quaternion(transform.rotation());
  quaternion.normalize();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

inline geometry_msgs::msg::Pose composeToolTcpPose(
  const Eigen::Matrix4d & planning_to_model_end,
  const Eigen::Isometry3d & model_end_to_tool_tcp)
{
  Eigen::Isometry3d planning_to_end = Eigen::Isometry3d::Identity();
  planning_to_end.matrix() = planning_to_model_end;
  // 变换方向固定为 planning_T_end * end_T_tcp，防止工具偏移反向。
  return isometryToPose(planning_to_end * model_end_to_tool_tcp);
}

// MovePose 的目标语义是工具 TCP 位姿，而 Pinocchio IK 的目标帧是模型末端。
// 这里只做既有工具固定变换的方向转换：planning_T_end =
// planning_T_tcp * inverse(end_T_tcp)，不在业务层实现任何运动学算法。
inline Eigen::Isometry3d targetTcpToModelEndPose(
  const geometry_msgs::msg::Pose & planning_target_tcp,
  const Eigen::Isometry3d & model_end_to_tool_tcp)
{
  return poseToIsometry(planning_target_tcp) *
         model_end_to_tool_tcp.inverse();
}

using HomogeneousMatrix = std::array<std::array<double, 4>, 4>;

inline HomogeneousMatrix toVendorMillimeterMatrix(
  const Eigen::Isometry3d & transform_metres)
{
  HomogeneousMatrix result{};
  const Eigen::Matrix4d matrix = transform_metres.matrix();
  for (std::size_t row = 0; row < 4U; ++row) {
    for (std::size_t column = 0; column < 4U; ++column) {
      result[row][column] = matrix(
        static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column));
    }
  }
  // ROS/Pinocchio 内部始终使用米，仅在厂商 SDK 的矩阵边界转换为毫米。
  result[0][3] *= 1000.0;
  result[1][3] *= 1000.0;
  result[2][3] *= 1000.0;
  return result;
}

template<std::size_t SizeT>
inline bool degreesToRadians(
  const std::array<double, SizeT> & degrees,
  std::vector<double> & radians,
  std::string & error_message,
  const std::string & context)
{
  // MarvinSDK 反馈为度；MiracleMAlgo/Pinocchio 的关节状态统一使用弧度。
  radians.resize(SizeT);
  for (std::size_t index = 0; index < SizeT; ++index) {
    if (!std::isfinite(degrees[index])) {
      error_message = context + " contains non-finite joint feedback";
      radians.clear();
      return false;
    }
    radians[index] = degrees[index] * kDegreesToRadians;
  }
  error_message.clear();
  return true;
}

}  // namespace motion_control_pkg::kinematics

#endif  // MOTION_CONTROL_PKG_KINEMATICS_TRANSFORM_UTILS_HPP_
