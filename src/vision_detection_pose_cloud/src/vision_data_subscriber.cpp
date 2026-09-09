#include "vision_detection_pose_cloud/vision_data_subscriber.hpp"

#include <chrono>
#include <sstream>

namespace vision_detection_pose_cloud
{

// ============================================================================
// 构造: 在传入的 Node 上创建订阅
// ============================================================================
VisionDataSubscriber::VisionDataSubscriber(rclcpp::Node * node)
{
  // 话题名作为参数, 可在 launch/命令行中覆盖
  std::string topic = node->declare_parameter<std::string>(
    "vision_topic", "/fusion/synced_robot_vision");

  // SensorDataQoS: BestEffort + 浅拷贝, 适合高频图像流, 丢帧不阻塞
  sub_ = node->create_subscription<com_interfaces::msg::SyncedRobotVision>(
    topic,
    rclcpp::SensorDataQoS(),
    std::bind(&VisionDataSubscriber::onSyncedVision, this, std::placeholders::_1));

  RCLCPP_INFO(
    node->get_logger(),
    "VisionDataSubscriber: 订阅 %s (SensorDataQoS), 按 camera_id 分别缓存, 等待相机数据...",
    topic.c_str());
}

// ============================================================================
// 订阅回调: 按 camera_id 缓存最新一帧 (整帧浅拷贝)
// ============================================================================
void VisionDataSubscriber::onSyncedVision(
  const com_interfaces::msg::SyncedRobotVision::SharedPtr msg)
{
  bool first_for_this_cam = false;
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    first_for_this_cam = (frames_by_cam_.find(msg->camera_id) == frames_by_cam_.end());
    frames_by_cam_[msg->camera_id] = *msg;  // 覆盖该 camera_id 的旧帧
  }
  // 每个 camera_id 首帧到达时打印一次, 便于确认各相机数据链路
  if (first_for_this_cam) {
    RCLCPP_INFO(
      rclcpp::get_logger("VisionDataSubscriber"),
      "✅ camera_id=%u 首帧到达: data_valid=%d | 图像 %ux%u %s (%zu bytes) | "
        "点云 %ux%u point_step=%u | target_label='%s'",
      msg->camera_id, msg->data_valid ? 1 : 0,
      msg->camera_rgbd_color.width, msg->camera_rgbd_color.height,
      msg->camera_rgbd_color.encoding.c_str(),
      msg->camera_rgbd_color.data.size(),
      msg->camera_rgbd_pointcloud.width, msg->camera_rgbd_pointcloud.height,
      msg->camera_rgbd_pointcloud.point_step,
      msg->target_label.c_str());
  }
}

// ============================================================================
// 访问接口 (按 camera_id)
// ============================================================================
bool VisionDataSubscriber::hasData(uint8_t camera_id) const
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  return frames_by_cam_.find(camera_id) != frames_by_cam_.end();
}

bool VisionDataSubscriber::waitForData(uint8_t camera_id, double timeout_sec) const
{
  if (hasData(camera_id)) {
    return true;
  }
  // 主线程 rclcpp::spin() 驱动回调, 此处轮询
  auto start = std::chrono::steady_clock::now();
  while (!hasData(camera_id)) {
    if (std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count() > timeout_sec)
    {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return true;
}

bool VisionDataSubscriber::getImageAndCloud(
  uint8_t camera_id,
  sensor_msgs::msg::Image & image,
  sensor_msgs::msg::PointCloud2 & cloud,
  builtin_interfaces::msg::Time & stamp) const
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  auto it = frames_by_cam_.find(camera_id);
  if (it == frames_by_cam_.end()) {
    return false;  // 该 camera_id 无数据
  }
  const auto & frame = it->second;
  // data_valid 字段为"预留", 发布端可能未实现; 按实际数据判断
  if (!frame.data_valid) {
    RCLCPP_WARN_ONCE(
      rclcpp::get_logger("VisionDataSubscriber"),
      "camera_id=%u data_valid=false (发布端可能未实现此标志); 按实际数据判断",
      camera_id);
  }
  // 一次加锁, 同帧拷贝; 保证 image/cloud/stamp 来自同一帧
  image = frame.camera_rgbd_color;
  cloud = frame.camera_rgbd_pointcloud;
  stamp = frame.header.stamp;
  // 实际有效性: 图像有像素 + 点云有点
  const size_t cloud_points =
    static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
  return !image.data.empty() && image.width > 0 && image.height > 0 &&
         cloud_points > 0 && !cloud.data.empty();
}

std::string VisionDataSubscriber::getDebugInfo(uint8_t camera_id) const
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  std::ostringstream oss;
  auto it = frames_by_cam_.find(camera_id);
  if (it == frames_by_cam_.end()) {
    oss << "camera_id=" << static_cast<int>(camera_id) << " has_data=0 (从未收到)";
  } else {
    const auto & f = it->second;
    oss << "camera_id=" << static_cast<int>(camera_id)
        << " has_data=1 data_valid=" << (f.data_valid ? 1 : 0)
        << " img=" << f.camera_rgbd_color.width << "x"
        << f.camera_rgbd_color.height
        << " " << f.camera_rgbd_color.encoding
        << " bytes=" << f.camera_rgbd_color.data.size()
        << " cloud=" << f.camera_rgbd_pointcloud.width << "x"
        << f.camera_rgbd_pointcloud.height
        << " step=" << f.camera_rgbd_pointcloud.point_step
        << " stamp=" << f.header.stamp.sec << "." << f.header.stamp.nanosec
        << " target_label='" << f.target_label << "'";
  }
  return oss.str();
}

std::string VisionDataSubscriber::getDebugInfoAll() const
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  std::ostringstream oss;
  if (frames_by_cam_.empty()) {
    oss << "无任何缓存数据 (从未收到 SyncedRobotVision)";
  } else {
    oss << "已缓存 " << frames_by_cam_.size() << " 个相机: ";
    bool first = true;
    for (const auto & kv : frames_by_cam_) {
      if (!first) { oss << " | "; }
      first = false;
      const auto & f = kv.second;
      oss << "cam" << static_cast<int>(kv.first)
          << "=" << f.camera_rgbd_color.width << "x" << f.camera_rgbd_color.height
          << " " << f.camera_rgbd_color.encoding
          << " cloud=" << f.camera_rgbd_pointcloud.width << "x"
          << f.camera_rgbd_pointcloud.height;
    }
  }
  return oss.str();
}

bool VisionDataSubscriber::getCameraInfo(
  uint8_t camera_id, sensor_msgs::msg::CameraInfo & info) const
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  auto it = frames_by_cam_.find(camera_id);
  if (it == frames_by_cam_.end()) {
    return false;
  }
  info = it->second.camera_info;
  return true;
}

bool VisionDataSubscriber::getLatestFrame(
  uint8_t camera_id, com_interfaces::msg::SyncedRobotVision & frame) const
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  auto it = frames_by_cam_.find(camera_id);
  if (it == frames_by_cam_.end()) {
    return false;
  }
  frame = it->second;
  return true;
}

std::vector<uint8_t> VisionDataSubscriber::getCachedCameraIds() const
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  std::vector<uint8_t> ids;
  ids.reserve(frames_by_cam_.size());
  for (const auto & kv : frames_by_cam_) {
    ids.push_back(kv.first);
  }
  return ids;
}

bool VisionDataSubscriber::getLatestStamp(
  uint8_t camera_id, builtin_interfaces::msg::Time & stamp) const
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  auto it = frames_by_cam_.find(camera_id);
  if (it == frames_by_cam_.end()) {
    return false;
  }
  stamp = it->second.header.stamp;
  return true;
}

}  // namespace vision_detection_pose_cloud
