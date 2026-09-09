#ifndef VISION_DETECTION_POSE_CLOUD__DETECTION_POSE_CLOUD_SERVER_HPP_
#define VISION_DETECTION_POSE_CLOUD__DETECTION_POSE_CLOUD_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "com_interfaces/action/detection_pose_cloud.hpp"
#include "com_interfaces/msg/target_detection.hpp"
#include "vision_detection_pose_cloud/vision_data_subscriber.hpp"

// NPU 检测库接口
#include "sort_dectect.h"

namespace vision_detection_pose_cloud
{

using DetectionPoseCloud = com_interfaces::action::DetectionPoseCloud;
using GoalHandleDPC = rclcpp_action::ServerGoalHandle<DetectionPoseCloud>;

// 状态码定义（与 action 文件一致）
enum class StatusCode : uint8_t
{
  OK = 0,
  FAILED = 1,
  TIMEOUT = 2,
  BUSY = 3,
};

class DetectionPoseCloudServer : public rclcpp::Node
{
public:
  DetectionPoseCloudServer();
  ~DetectionPoseCloudServer() override = default;

private:
  // Action server 回调
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const DetectionPoseCloud::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleDPC> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleDPC> goal_handle);

  // 任务执行主体（独立线程）
  void execute(const std::shared_ptr<GoalHandleDPC> goal_handle);

  // 辅助函数
  void publish_feedback(
    const std::shared_ptr<GoalHandleDPC> & gh,
    const std::string & status,
    float progress,
    const std::string & info);

  com_interfaces::msg::TargetDetection build_detection_msg(
    const std::shared_ptr<const DetectionPoseCloud::Goal> & goal,
    const geometry_msgs::msg::PoseStamped & pose,
    float confidence);
  // 由 Result6D (gripper 系, m, rad) 构造 TargetDetection 消息
  com_interfaces::msg::TargetDetection build_detection_msg(
    const std::shared_ptr<const DetectionPoseCloud::Goal> & goal,
    const Result6D & result6d);

  std_msgs::msg::Header make_header(const std::string & frame_id) const;

  // 视觉管线函数
  // image 与 cloud 由 execute() 在任务起始处通过 getImageAndCloud() 原子取一次,
  // 保证两个管线阶段使用同一帧相机数据, 避免分别取到不同帧导致位姿错位
  // 输出 results: 检测到的多目标 6D 位姿 (gripper 系, m, ZYX rad)
  bool detect_target(
    const std::string & target_name,
    const sensor_msgs::msg::Image & image,
    const sensor_msgs::msg::PointCloud2 & cloud,
    std::vector<Result6D> & results,
    std::string & info);

  // 保存彩色图 + 点云到文件
  //   image_path: 保存为 PNG (通过 cv_bridge 转换 -> cv::imwrite)
  //   cloud_path: 保存为 PLY (通过 pcl::io::savePLYFileBinary)
  //   save_colored: 若 true 且点云已含 rgb 字段, PLY 自动保留颜色
  //                 (SyncedRobotVision.camera_rgbd_pointcloud 是 RGB 点云, 默认带色)
  bool save_image_and_cloud(
    const sensor_msgs::msg::Image & image,
    const sensor_msgs::msg::PointCloud2 & cloud,
    const std::string & image_path,
    const std::string & cloud_path,
    bool save_colored = true);

  // 调试: 从彩色点云中提取颜色, 保存为图像 (用于核对点云颜色)
  //   要求: 点云含 rgb/rgba 字段 + 有序(height>1) + width×height == expected_w×expected_h
  //   有序且数量正确: 还原为 expected_w×expected_h 图像 (点(row,col)->像素(row,col))
  //   无序或数量不符: 跳过并返回诊断信息 (需相机内参投影才能还原, 此处不做)
  //   expected_w/expected_h: 期望图像尺寸 (通常来自彩色图 image.width/height)
  bool extract_color_image_from_cloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    int expected_w,
    int expected_h,
    const std::string & color_img_path,
    std::string & info);

  // 成员
  rclcpp_action::Server<DetectionPoseCloud>::SharedPtr action_server_;
  rclcpp::Publisher<com_interfaces::msg::TargetDetection>::SharedPtr detection_pub_;

  // 相机数据订阅器: 始终缓存最新一帧 SyncedRobotVision (彩色图 + 点云 + 内参)
  std::unique_ptr<VisionDataSubscriber> vision_sub_;

  // 并行任务计数（仅用于日志展示并行度；真实视觉资源互斥请按需加锁）
  std::atomic<int> active_tasks_count_{0};

  // NPU 检测配置 (slotDetection 参数; 当前库内用硬编码, 保留默认即可)
  ParaConfig para_config_;
  // DecodeInfo 不作为成员复用 (多任务并行时竞争), 由 detect_target 局部构造

  // NPU 互斥锁: slotDetection 内部使用全局单例 ObjectRegionDetector,
  // 多个 action goal 并行线程调用时必须串行化, 避免竞争 rknn 上下文
  std::mutex npu_mutex_;
};

}  // namespace vision_detection_pose_cloud

#endif  // VISION_DETECTION_POSE_CLOUD__DETECTION_POSE_CLOUD_SERVER_HPP_
