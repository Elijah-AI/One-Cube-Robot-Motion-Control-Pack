#ifndef VISION_DETECTION_POSE_CLOUD__VISION_DATA_SUBSCRIBER_HPP_
#define VISION_DETECTION_POSE_CLOUD__VISION_DATA_SUBSCRIBER_HPP_

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "com_interfaces/msg/synced_robot_vision.hpp"

namespace vision_detection_pose_cloud
{

// ============================================================================
// VisionDataSubscriber
//   专门订阅 com_interfaces/msg/SyncedRobotVision 的数据缓冲器。
//   按 camera_id 分别缓存最新一帧 (支持多相机: 0=头部, 1=腰部, 2+预留),
//   每个 camera_id 独立缓存, 互不覆盖。
//
//   设计要点:
//     1) 持有 rclcpp::Node 的原始指针 (非拥有), 由 DetectionPoseCloudServer 传入;
//     2) 订阅使用 SensorDataQoS (BestEffort), 适合高频图像流, 丢帧不阻塞;
//     3) 缓存区 std::unordered_map<uint8_t, SyncedRobotVision>, mutex 保护;
//     4) 彩色图与点云必须一起取 (同帧同步), 避免分别取到不同帧导致位姿错位;
//        因此只提供 getImageAndCloud() 原子接口, 同时返回时间戳。
// ============================================================================
class VisionDataSubscriber
{
public:
  explicit VisionDataSubscriber(rclcpp::Node * node);
  ~VisionDataSubscriber() = default;

  // 指定 camera_id 是否已收到至少一帧
  bool hasData(uint8_t camera_id) const;

  // 等待指定 camera_id 的数据到达 (阻塞轮询; 依赖主线程 executor spin)
  bool waitForData(uint8_t camera_id, double timeout_sec) const;

  // 同时获取指定 camera_id 的彩色图 + 点云 + 时间戳 (同帧同步, 一次加锁)
  // stamp: 来自 SyncedRobotVision.header.stamp
  // 返回 false 表示该 camera_id 无数据, 或图像/点云为空
  bool getImageAndCloud(
    uint8_t camera_id,
    sensor_msgs::msg::Image & image,
    sensor_msgs::msg::PointCloud2 & cloud,
    builtin_interfaces::msg::Time & stamp) const;

  // 诊断信息: 列出所有已缓存 camera_id 及其帧摘要
  std::string getDebugInfo(uint8_t camera_id) const;
  std::string getDebugInfoAll() const;

  // 获取指定 camera_id 的相机内参 (CameraInfo)
  bool getCameraInfo(uint8_t camera_id, sensor_msgs::msg::CameraInfo & info) const;

  // 获取指定 camera_id 的完整 SyncedRobotVision 帧
  bool getLatestFrame(uint8_t camera_id, com_interfaces::msg::SyncedRobotVision & frame) const;

  // 所有已缓存的 camera_id 列表
  std::vector<uint8_t> getCachedCameraIds() const;

  // 指定 camera_id 最新数据的时间戳
  bool getLatestStamp(uint8_t camera_id, builtin_interfaces::msg::Time & stamp) const;

private:
  void onSyncedVision(const com_interfaces::msg::SyncedRobotVision::SharedPtr msg);

  rclcpp::Subscription<com_interfaces::msg::SyncedRobotVision>::SharedPtr sub_;
  mutable std::mutex data_mutex_;
  // 按 camera_id 缓存最新一帧; 每个 id 独立, 互不覆盖
  std::unordered_map<uint8_t, com_interfaces::msg::SyncedRobotVision> frames_by_cam_;
  // 已缓存过的 camera_id 集合 (用于首帧日志去重, 避免每帧都打印)
  std::unordered_map<uint8_t, bool> first_logged_;
};

}  // namespace vision_detection_pose_cloud

#endif  // VISION_DETECTION_POSE_CLOUD__VISION_DATA_SUBSCRIBER_HPP_
