#include "vision_detection_pose_cloud/detection_pose_cloud_server.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sys/stat.h>
#include <cerrno>
#include <cstring>
#include <cmath>

namespace vision_detection_pose_cloud
{
namespace {
// Site confirmed: algorithm poses are already in base coordinates.
constexpr const char* kDetectionFrame = "base_link";
}


// ============================================================================
// 构造函数
// ============================================================================

DetectionPoseCloudServer::DetectionPoseCloudServer()
: rclcpp::Node("detection_pose_cloud_server")
{
  // 创建 Action Server，监听 /vision/detection_pose_cloud
  action_server_ = rclcpp_action::create_server<DetectionPoseCloud>(
    this,
    "/vision/detection_pose_cloud",
    std::bind(&DetectionPoseCloudServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DetectionPoseCloudServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&DetectionPoseCloudServer::handle_accepted, this, std::placeholders::_1));

  // 发布 TargetDetection 话题（rclcpp publisher 线程安全，可被多个并行任务同时调用）
  detection_pub_ = this->create_publisher<com_interfaces::msg::TargetDetection>(
    "/vision/target_detection", 10);

  // 相机数据订阅器: 始终缓存最新一帧 SyncedRobotVision
  vision_sub_ = std::make_unique<VisionDataSubscriber>(this);

  RCLCPP_INFO(this->get_logger(), "✅ Action server ready (parallel): /vision/detection_pose_cloud");
}

// ============================================================================
// Action 回调
// ============================================================================

rclcpp_action::GoalResponse
DetectionPoseCloudServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const DetectionPoseCloud::Goal> goal)
{
  const int prio = goal->priority;

  // 1. 优先级合法性校验 (1-25)
  if (prio < 1 || prio > 25) {
    RCLCPP_WARN(this->get_logger(), "拒绝任务: 优先级 %d 越界 (要求 1-25)", prio);
    return rclcpp_action::GoalResponse::REJECT;
  }

  // 2. 任务名称非空校验
  if (goal->target_obj_name.empty()) {
    RCLCPP_WARN(this->get_logger(), "拒绝任务: target_obj_name 为空");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // 3. 并行模式：不拒绝、不抢占，所有合法任务直接接受并行执行
  //    优先级字段仅作为客户端调度提示，服务端按到达顺序派发独立线程
  (void)prio;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
DetectionPoseCloudServer::handle_cancel(
  const std::shared_ptr<GoalHandleDPC> /*goal_handle*/)
{
  RCLCPP_INFO(this->get_logger(), "收到取消请求, 允许取消");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
DetectionPoseCloudServer::handle_accepted(const std::shared_ptr<GoalHandleDPC> goal_handle)
{
  // 并行模式：每个合法 goal 直接派发独立工作线程，互不抢占
  // 工作线程持有 goal_handle 的 shared_ptr 副本，对象生命周期安全
  std::thread(
    std::bind(&DetectionPoseCloudServer::execute, this, std::placeholders::_1),
    goal_handle).detach();
}

// ============================================================================
// 任务执行主体
// ============================================================================

void
DetectionPoseCloudServer::execute(const std::shared_ptr<GoalHandleDPC> goal_handle)
{
  auto logger = this->get_logger();
  const auto goal = goal_handle->get_goal();
  const auto t_start = std::chrono::steady_clock::now();

  // 工具函数（lambda 需先定义后使用）
  auto elapsed = [&]() -> double {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
  };
  auto is_canceling = [&]() -> bool {
    return goal_handle->is_canceling();
  };
  auto frame_id = []() -> std::string {
    return kDetectionFrame;
  };
  if (!goal->expected_frame_id.empty() && goal->expected_frame_id != kDetectionFrame) {
    RCLCPP_WARN(logger, "Vision output is already %s; requested frame %s is not transformed",
      kDetectionFrame, goal->expected_frame_id.c_str());
  }

  const int prev_count = active_tasks_count_.fetch_add(1) + 1;
  RCLCPP_INFO(
    logger, "▶ 任务开始 task_id=%s target=%s type=%d priority=%d frame=%s (并行度=%d)",
    goal->task_id.c_str(), goal->target_obj_name.c_str(),
    goal->type, goal->priority, frame_id().c_str(), prev_count);
  auto dec_on_exit = [&]() {
    active_tasks_count_.fetch_sub(1);
  };

  auto result = std::make_shared<DetectionPoseCloud::Result>();
  result->success = false;
  result->status_code = static_cast<uint8_t>(StatusCode::FAILED);  // 默认 FAILED
  auto finish_fail = [&](uint8_t code, const std::string & msg) {
    result->header = make_header(frame_id());
    result->status_code = code;
    result->error_message = msg;
    result->execution_time = elapsed();
    goal_handle->abort(result);
    dec_on_exit();
  };
  auto finish_cancel = [&]() {
    result->header = make_header(frame_id());
    result->error_message = "任务被取消";
    result->execution_time = elapsed();
    goal_handle->canceled(result);
    dec_on_exit();
  };

  // ---------- STAGE 1: DETECTING ----------
  publish_feedback(goal_handle, "DETECTING", 5.0f, "正在检测目标物: " + goal->target_obj_name);
  std::string info;

  // 一次性原子取彩色图 + 点云 (同帧同步), 供后续 detect / generate_point_cloud 共用
  // 按 goal->camera_id 指定相机 (0=头部, 1=腰部); 同时取时间戳
  //  const uint8_t camera_id = goal->camera_id;
  const uint8_t camera_id = 0;
  sensor_msgs::msg::Image cur_image;
  sensor_msgs::msg::PointCloud2 cur_cloud;
  builtin_interfaces::msg::Time cur_stamp;
  if (!vision_sub_) {
    finish_fail(static_cast<uint8_t>(StatusCode::FAILED),
      "检测失败: vision_sub_ 未初始化 (内部错误)");
    return;
  }
  // 等待指定 camera_id 数据到达 (最多 5 秒); 主线程 rclcpp::spin() 驱动回调填入数据
  RCLCPP_INFO(logger, "等待 camera_id=%u 数据 (最多 5s)...", camera_id);
  if (!vision_sub_->waitForData(camera_id, 5.0)) {
    RCLCPP_ERROR(logger,
      "检测失败: 5s 内未收到 camera_id=%u 的 SyncedRobotVision", camera_id);
    RCLCPP_ERROR(logger, "已缓存相机: %s",
      vision_sub_->getDebugInfoAll().c_str());
    RCLCPP_ERROR(logger,
      "排查: 1) 相机发布节点是否启动? 2) 话题名是否匹配 (默认 /fusion/synced_robot_vision)? "
      "3) QoS 是否兼容 (订阅用 SensorDataQoS=BestEffort)? "
      "4) camera_id 是否正确 (0=头部, 1=腰部)? "
      "用 'ros2 topic echo --once /fusion/synced_robot_vision --field camera_id' 确认发布端 id");
    finish_fail(static_cast<uint8_t>(StatusCode::TIMEOUT),
      "检测失败: 5s 内未收到 camera_id=" + std::to_string(camera_id) +
      " 的相机数据");
    return;
  }
  // 取同帧图像 + 点云 + 时间戳; 若实际数据为空 (图像无像素/点云无点) 才失败
  // 注: data_valid=false 不再硬拦截 (该字段为"预留", 发布端可能未实现)
  if (!vision_sub_->getImageAndCloud(camera_id, cur_image, cur_cloud, cur_stamp)) {
    RCLCPP_ERROR(logger, "收到数据但图像/点云为空, 诊断: %s",
      vision_sub_->getDebugInfo(camera_id).c_str());
    finish_fail(static_cast<uint8_t>(StatusCode::FAILED),
      "检测失败: 图像或点云为空, 诊断: " + vision_sub_->getDebugInfo(camera_id));
    return;
  }
  RCLCPP_INFO(logger, "取到有效相机数据: %s",
    vision_sub_->getDebugInfo(camera_id).c_str());

  //图像和点云保存 (彩色图 + 点云, 同帧); 文件名含 camera_id + 时间戳避免并行任务覆盖
  const std::string ts = std::to_string(cur_stamp.sec) + "_" +
                         std::to_string(cur_stamp.nanosec);
  const std::string img_path = goal->target_obj_name + "_cam" +
    std::to_string(camera_id) + "_" + ts + ".bmp";
  const std::string pcd_path = goal->target_obj_name + "_cam" +
    std::to_string(camera_id) + "_" + ts + ".ply";
  RCLCPP_INFO(logger, "相机数据时间戳: %d.%09d", cur_stamp.sec, cur_stamp.nanosec);
  // save_image_and_cloud(cur_image, cur_cloud, img_path, pcd_path, true);

  // ---------- STAGE 1: DETECTING + ESTIMATING_POSE ----------
  // detect_target 内部完成: RKNN 检测 + 点云拟合 + 6D 位姿 + 手眼变换
  // 输出 results: 多目标 6D 位姿 (base_link 系, m, ZYX rad)
  std::vector<Result6D> detect_results;
  if (!detect_target(goal->target_obj_name, cur_image, cur_cloud, detect_results, info)) {
    finish_fail(static_cast<uint8_t>(StatusCode::TIMEOUT), "检测失败: " + info);
    return;
  }
  if (is_canceling()) { finish_cancel(); return; }

  // ---------- STAGE 2: BUILD TARGETS ----------
  publish_feedback(goal_handle, "ESTIMATING_POSE", 50.0f, "正在封装多目标结果");
  result->detected_targets.clear();
  result->detected_targets.reserve(detect_results.size());
  for (const auto & r : detect_results) {
    auto det = build_detection_msg(goal, r);
    result->detected_targets.push_back(det);
    // 发布到 TargetDetection 话题（rclcpp publisher 线程安全，可并行调用）
    detection_pub_->publish(det);
  }
  if (is_canceling()) { finish_cancel(); return; }

  // ---------- STAGE 3: FINISHED ----------
  publish_feedback(goal_handle, "FINISHED", 100.0f, "任务完成");

  result->header = make_header(frame_id());
  result->header.stamp = cur_stamp;  // 用相机数据采集时间戳，而非节点当前时刻
  result->success = true;
  result->status_code = static_cast<uint8_t>(StatusCode::OK);
  result->error_message = "";
  result->total_detected_count = static_cast<int32_t>(detect_results.size());
  result->execution_time = elapsed();
  goal_handle->succeed(result);
  dec_on_exit();

  RCLCPP_INFO(
    logger, "✅ 任务完成 task_id=%s success=true targets=%d time=%.3fs",
    goal->task_id.c_str(), result->total_detected_count, result->execution_time);
}

// ============================================================================
// 辅助函数
// ============================================================================

void
DetectionPoseCloudServer::publish_feedback(
  const std::shared_ptr<GoalHandleDPC> & gh,
  const std::string & status,
  float progress,
  const std::string & info)
{
  auto fb = std::make_shared<DetectionPoseCloud::Feedback>();
  fb->task_status = status;
  fb->progress_rate = progress;
  fb->current_step_info = info;
  fb->header.stamp = this->now();
  fb->header.frame_id = kDetectionFrame;
  gh->publish_feedback(fb);
}

std_msgs::msg::Header
DetectionPoseCloudServer::make_header(const std::string & frame_id) const
{
  std_msgs::msg::Header h;
  h.stamp = this->now();
  h.frame_id = frame_id.empty() ? std::string(kDetectionFrame) : frame_id;
  return h;
}

com_interfaces::msg::TargetDetection
DetectionPoseCloudServer::build_detection_msg(
  const std::shared_ptr<const DetectionPoseCloud::Goal> & goal,
  const geometry_msgs::msg::PoseStamped & pose,
  float confidence)
{
  com_interfaces::msg::TargetDetection det;
  det.obj_name = goal->target_obj_name;
  det.pose = pose;
  det.confidence = confidence;
  det.tool_type = (goal->type == 1) ? "gripper" : "suction";
  return det;
}

// 由 Result6D (base_link 系, m, ZYX rad) 构造 TargetDetection 消息
// ZYX 欧拉角 -> 四元数：q = Rz(yaw)*Ry(pitch)*Rx(roll) 对应的四元数
com_interfaces::msg::TargetDetection
DetectionPoseCloudServer::build_detection_msg(
  const std::shared_ptr<const DetectionPoseCloud::Goal> & goal,
  const Result6D & result6d)
{
  com_interfaces::msg::TargetDetection det;
  det.obj_name = goal->target_obj_name;

  // 位置 (m, base_link 系)
  det.pose.header.stamp = this->now();
  det.pose.header.frame_id = kDetectionFrame;
  det.pose.pose.position.x = result6d.pose_xyz.x();
  det.pose.pose.position.y = result6d.pose_xyz.y();
  det.pose.pose.position.z = result6d.pose_xyz.z();

  // ZYX 内禀欧拉角 (roll, pitch, yaw) -> 四元数
  // q = qz(yaw) * qy(pitch) * qx(roll)
  float roll = result6d.pose_rpy.x();
  float pitch = result6d.pose_rpy.y();
  float yaw = result6d.pose_rpy.z();
  float cr = std::cos(roll * 0.5f),  sr = std::sin(roll * 0.5f);
  float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
  float cy = std::cos(yaw * 0.5f),   sy = std::sin(yaw * 0.5f);
  det.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
  det.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
  det.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
  det.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;

  det.confidence = result6d.confidence;
  det.tool_type = (goal->type == 1) ? "gripper" : "suction";
  return det;
}

// ============================================================================
// 视觉管线函数（RKNN 检测 + 点云拟合 + 6D 位姿 + 手眼变换）
// ============================================================================

bool
DetectionPoseCloudServer::detect_target(
  const std::string & target_name,
  const sensor_msgs::msg::Image & image,
  const sensor_msgs::msg::PointCloud2 & cloud,
  std::vector<Result6D> & results,
  std::string & info)
{
  results.clear();
  // image 与 cloud 已由 execute() 通过 getImageAndCloud() 原子取一次, 保证同帧
  auto logger = this->get_logger();

  // -------------------- 1. sensor_msgs::Image -> cv::Mat (BGR) --------------------
  cv::Mat img_in;
  try {
    auto img_msg = std::make_shared<sensor_msgs::msg::Image>(image);
    std::string enc = image.encoding.empty() ? std::string("bgr8") : image.encoding;
    auto cv_br = cv_bridge::toCvShare(img_msg, enc);
    if (!cv_br || cv_br->image.empty()) {
      info = "图像转换失败 (cv_bridge 返回空)";
      RCLCPP_WARN(logger, "detect_target: %s", info.c_str());
      return false;
    }
    // slotDetection 内部用 OpenCV 处理, 统一转 bgr8 (OpenCV 默认)
    if (enc == "rgb8") {
      cv::cvtColor(cv_br->image, img_in, cv::COLOR_RGB2BGR);
    } else {
      img_in = cv_br->image.clone();  // 拷贝, 避免修改原始消息缓冲
    }
  } catch (const cv_bridge::Exception & e) {
    info = std::string("cv_bridge 异常: ") + e.what();
    RCLCPP_WARN(logger, "detect_target: %s", info.c_str());
    return false;
  } catch (const cv::Exception & e) {
    info = std::string("OpenCV 异常: ") + e.what();
    RCLCPP_WARN(logger, "detect_target: %s", info.c_str());
    return false;
  }

  // -------------------- 2. sensor_msgs::PointCloud2 -> pcl::PointCloud<PointXYZ> ----
  // 源点云含 RGB 字段, slotDetection 要求 pcl::PointCloud<pcl::PointXYZ> (仅 xyz)
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  try {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_in);
  } catch (const std::exception & e) {
    info = std::string("点云转换异常: ") + e.what();
    RCLCPP_WARN(logger, "detect_target: %s", info.c_str());
    return false;
  }
  if (cloud_in->empty()) {
    info = "点云转换后为空 (可能无 xyz 字段)";
    RCLCPP_WARN(logger, "detect_target: %s", info.c_str());
    return false;
  }

  // -------------------- 3. 调用 NPU 检测 slotDetection --------------------
  // ParaConfig: 当前库内未实际读取 (slotDetection 内用硬编码 LocalizationInfo),
  //             传成员默认值即可; DecodeInfo 局部构造接收结果
  // npu_mutex_: 串行化, 避免并行 goal 线程竞争全局单例 ObjectRegionDetector
  DecodeInfo decode_info;
  RCLCPP_INFO(logger,
    "detect_target('%s'): 调用 slotDetection | 图像 %dx%d | 点云 %zu 点 (organized=%d)",
    target_name.c_str(), img_in.cols, img_in.rows, cloud_in->size(),
    cloud_in->height > 1 ? 1 : 0);

  {
    std::lock_guard<std::mutex> lk(npu_mutex_);
    slotDetection(img_in, cloud_in, &para_config_, &decode_info, target_name);
  }

  // -------------------- 4. 解析结果 --------------------
  // decode_info.loc_ok      : 1=定位成功, 0=失败
  // decode_info.output_result: vector<Result6D>, 每个含 id/confidence/pose_xyz[m, base_link 系]/pose_rpy[rad, ZYX]
  // decode_info.relia        : 可靠性标志
  if (decode_info.loc_ok != 1) {
    info = "slotDetection 定位失败 (loc_ok=" + std::to_string(decode_info.loc_ok) +
      ", relia=" + std::to_string(decode_info.relia) + ")";
    RCLCPP_WARN(logger, "detect_target: %s", info.c_str());
    return false;
  }

  // 将检测到的多目标结果输出到 results (base_link 系, m, ZYX rad)
  results = decode_info.output_result;

  RCLCPP_INFO(logger, "detect_target('%s'): 检测到 %zu 个目标 (relia=%d)",
    target_name.c_str(), results.size(), decode_info.relia);
  for (size_t i = 0; i < results.size(); ++i) {
    const auto & r = results[i];
    RCLCPP_INFO(logger,
      "  [%zu] id=%d conf=%.3f pos=(%.3f, %.3f, %.3f)[m, base_link] rpy=(%.1f, %.1f, %.1f)[deg]",
      i, r.id, r.confidence,
      r.pose_xyz.x(), r.pose_xyz.y(), r.pose_xyz.z(),
      r.pose_rpy.x() * 180.0 / M_PI, r.pose_rpy.y() * 180.0 / M_PI, r.pose_rpy.z() * 180.0 / M_PI);
  }

  info = "检测完成: " + std::to_string(results.size()) +
    " 个目标 (base_link 坐标系, m)";
  return !results.empty();
}

// ============================================================================
// save_image_and_cloud
//   保存彩色图为 PNG, 点云为 PLY (可选保留颜色)
//   image 与 cloud 必须来自同一帧 (由 execute() 通过 getImageAndCloud 取出)
// ============================================================================

namespace {

// 确保目标目录存在 (递归创建, 权限 0755)
bool ensure_dir_exists(const std::string & path)
{
  if (path.empty()) {
    return true;  // 当前目录
  }
  std::string dir;
  for (size_t i = 0; i < path.size(); ++i) {
    dir += path[i];
    if (path[i] == '/') {
      // 跳过开头的 '/' (绝对路径) 或连续 '/'
      if (dir.size() <= 1) {
        continue;
      }
      if (::mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) {
        return false;
      }
    }
  }
  return true;
}

}  // anonymous namespace

bool
DetectionPoseCloudServer::save_image_and_cloud(
  const sensor_msgs::msg::Image & image,
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & image_path,
  const std::string & cloud_path,
  bool save_colored)
{
  // 提取目录并创建
  auto ensure_parent = [](const std::string & file_path) -> bool {
    auto pos = file_path.find_last_of('/');
    if (pos == std::string::npos) {
      return true;  // 无目录, 写到当前目录
    }
    return ensure_dir_exists(file_path.substr(0, pos + 1));
  };

  if (!ensure_parent(image_path) || !ensure_parent(cloud_path)) {
    RCLCPP_WARN(this->get_logger(), "save_image_and_cloud: 创建目录失败: %s",
      std::strerror(errno));
  }

  bool image_ok = false;
  bool cloud_ok = false;

  // -------------------- 1. 保存彩色图为 PNG --------------------
  if (image.width > 0 && image.height > 0 && !image.data.empty()) {
    try {
      // cv_bridge::toCvShare 要求 ConstPtr; 这里从 const 引用构造一个
      auto img_msg = std::make_shared<sensor_msgs::msg::Image>(image);
      // encoding: 若消息自带则用自带 ("rgb8"/"bgr8"/"mono8"), 否则按 bgr8 解释
      std::string enc = image.encoding.empty() ? std::string("bgr8") : image.encoding;
      auto cv_br = cv_bridge::toCvShare(img_msg, enc);
      if (cv_br && !cv_br->image.empty()) {
        // cv::imwrite 按 BGR 排序写 PNG (OpenCV 默认); 若源是 rgb8 需先转 BGR
        // 否则红蓝通道互换, 颜色失真
        cv::Mat img_out;
        if (enc == "rgb8" || enc == "rgba8") {
          cv::cvtColor(cv_br->image, img_out, cv::COLOR_RGB2BGR);
        } else {
          img_out = cv_br->image;  // bgr8/mono8 无需转换
        }
        if (cv::imwrite(image_path, img_out)) {
          image_ok = true;
          RCLCPP_INFO(this->get_logger(),
            "已保存图像: %s (%dx%d %s→bgr8)", image_path.c_str(),
            img_out.cols, img_out.rows, enc.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "cv::imwrite 失败: %s", image_path.c_str());
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "save_image_and_cloud: cv_bridge 转换结果为空");
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN(this->get_logger(), "save_image_and_cloud: cv_bridge 异常: %s", e.what());
    } catch (const cv::Exception & e) {
      RCLCPP_WARN(this->get_logger(), "save_image_and_cloud: OpenCV 异常: %s", e.what());
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "save_image_and_cloud: 图像数据为空, 跳过保存");
  }

  // -------------------- 2. 保存点云为 PLY --------------------
  // SyncedRobotVision.camera_rgbd_pointcloud 是 RGB 点云 (含 rgb 字段);
  // pcl::io::savePLYFileBinary 会保留所有字段, 包括颜色。
  // save_colored=false 时, 可改为保存纯几何 (此处仅做开关, 字段是否含色由源决定)
  if (cloud.width > 0 && cloud.height > 0 && cloud.row_step > 0) {
    try {
      // sensor_msgs/PointCloud2 -> pcl::PCLPointCloud2
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(cloud, pcl_pc2);

      // 若要求彩色但点云无 rgb 字段, 仅记录警告 (不阻断保存)
      if (save_colored) {
        bool has_rgb = false;
        for (const auto & f : pcl_pc2.fields) {
          if (f.name == "rgb" || f.name == "rgba") {
            has_rgb = true;
            break;
          }
        }
        if (!has_rgb) {
          RCLCPP_WARN(this->get_logger(),
            "save_image_and_cloud: save_colored=true 但点云无 rgb 字段, "
            "PLY 将不含颜色 (如需投影图像着色请扩展此函数)");
        }
      }

      // PCL 1.12: savePLYFileBinary 无 PCLPointCloud2 重载, 用 savePLYFile(binary=true)
      int rc = pcl::io::savePLYFile(
        cloud_path, pcl_pc2,
        Eigen::Vector4f::Zero(),
        Eigen::Quaternionf::Identity(),
        /*binary=*/true);
      if (rc == 0) {
        cloud_ok = true;
        RCLCPP_INFO(this->get_logger(),
          "已保存点云: %s (%ux%u points, point_step=%u%s)",
          cloud_path.c_str(),
          pcl_pc2.width, pcl_pc2.height, pcl_pc2.point_step,
          save_colored ? ", 含颜色" : "");
      } else {
        RCLCPP_WARN(this->get_logger(), "savePLYFile(binary) 失败 (rc=%d): %s",
          rc, cloud_path.c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "save_image_and_cloud: PCL 异常: %s", e.what());
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "save_image_and_cloud: 点云数据为空, 跳过保存");
  }

  // 调试: 若点云已保存且含颜色, 从点云提取颜色再存一张图 (便于核对点云颜色)
  if (cloud_ok && save_colored) {
    // 由 cloud_path 派生颜色图路径: xxx_cloud.ply -> xxx_cloud_color.png
    std::string color_path = cloud_path;
    auto dot = color_path.find_last_of('.');
    if (dot != std::string::npos) {
      color_path = color_path.substr(0, dot) + "_color.bmp";
    } else {
      color_path += "_color.png";
    }
    std::string ext_info;
    extract_color_image_from_cloud(cloud, image.width, image.height, color_path, ext_info);
  }

  return image_ok && cloud_ok;
}

// ============================================================================
// extract_color_image_from_cloud
//   调试函数: 从彩色点云提取颜色, 保存为图像 (核对点云颜色是否正确)
//   要求: 点云含 rgb/rgba 字段 + 有序(height>1) + width×height == expected_w×expected_h
//   有序且数量正确: 还原为 expected_w×expected_h 图像 (点(row,col)->像素(row,col))
//   无序或数量不符: 跳过并返回诊断 (需相机内参投影才能还原, 此处不做)
// ============================================================================
bool
DetectionPoseCloudServer::extract_color_image_from_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  int expected_w,
  int expected_h,
  const std::string & color_img_path,
  std::string & info)
{
  auto logger = this->get_logger();

  // 1. 检查是否有 rgb/rgba 字段
  bool has_rgb = false;
  for (const auto & f : cloud.fields) {
    if (f.name == "rgb" || f.name == "rgba") {
      has_rgb = true;
      break;
    }
  }
  if (!has_rgb) {
    info = "点云无 rgb/rgba 字段, 跳过颜色图提取";
    RCLCPP_DEBUG(logger, "extract_color_image: %s", info.c_str());
    return false;
  }

  // 2. 校验点云总点数 == expected_w × expected_h
  //    有序(1280x720) 或 扁平(921600x1) 只要总数匹配, 即可按行主序重塑为图像:
  //      线性索引 i -> 像素 (row=i/expected_w, col=i%expected_w)
  //    (前提: 发布端按行主序填充, 即先存第一行从左到右, 再第二行...)
  const uint32_t cw = cloud.width;
  const uint32_t ch = cloud.height;
  const size_t expected_count =
    static_cast<size_t>(expected_w) * static_cast<size_t>(expected_h);
  const size_t cloud_count = static_cast<size_t>(cw) * static_cast<size_t>(ch);

  if (cloud_count != expected_count) {
    info = "点云数量 " + std::to_string(cw) + "x" + std::to_string(ch) +
      "=" + std::to_string(cloud_count) + " 与期望 " +
      std::to_string(expected_w) + "x" + std::to_string(expected_h) +
      "=" + std::to_string(expected_count) + " 不符, 跳过";
    RCLCPP_WARN(logger, "extract_color_image: %s", info.c_str());
    return false;
  }

  // 3. 数量正确: 转为 pcl::PointCloud<pcl::PointXYZRGB> (PCL 自动解包 rgb)
  pcl::PointCloud<pcl::PointXYZRGB> rgb_cloud;
  try {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, rgb_cloud);
  } catch (const std::exception & e) {
    info = std::string("PointXYZRGB 转换异常: ") + e.what();
    RCLCPP_WARN(logger, "extract_color_image: %s", info.c_str());
    return false;
  }
  if (rgb_cloud.size() != expected_count) {
    info = "PointXYZRGB 数量 " + std::to_string(rgb_cloud.size()) +
      " 与期望 " + std::to_string(expected_count) + " 不符";
    RCLCPP_WARN(logger, "extract_color_image: %s", info.c_str());
    return false;
  }

  // 4. 构造 expected_h × expected_w 图像 (点(row,col) -> 像素(row,col))
  cv::Mat color_img(expected_h, expected_w, CV_8UC3, cv::Scalar(0, 0, 0));
  // PointXYZRGB 的 .r/.g/.b 是正确 RGB; OpenCV imwrite 需 BGR -> Vec3b(b,g,r)
  bool all_zero = true;
  size_t valid_count = 0;
  for (int r = 0; r < expected_h; ++r) {
    for (int c = 0; c < expected_w; ++c) {
      const auto & p = rgb_cloud.points[
        static_cast<size_t>(r) * expected_w + static_cast<size_t>(c)];
      // 跳过 NaN 点 (有序点云中无效深度点常被置 NaN), 留黑色
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      color_img.at<cv::Vec3b>(r, c) = cv::Vec3b(p.b, p.g, p.r);
      ++valid_count;
      if (p.r || p.g || p.b) { all_zero = false; }
    }
  }

  // 5. 保存
  bool ok = false;
  try {
    if (cv::imwrite(color_img_path, color_img)) {
      ok = true;
      RCLCPP_INFO(logger,
        "已保存点云颜色图: %s (%dx%d, organized=1, valid=%zu/%zu%s)",
        color_img_path.c_str(), expected_w, expected_h,
        valid_count, expected_count,
        all_zero ? ", 颜色全零(可能 rgb 字段解析失败)" : "");
    } else {
      RCLCPP_WARN(logger, "extract_color_image: cv::imwrite 失败: %s",
        color_img_path.c_str());
    }
  } catch (const cv::Exception & e) {
    RCLCPP_WARN(logger, "extract_color_image: OpenCV 异常: %s", e.what());
  }

  info = "点云颜色图: " + std::to_string(expected_w) + "x" +
    std::to_string(expected_h) + " organized=1 valid=" +
    std::to_string(valid_count) + "/" + std::to_string(expected_count) +
    " all_zero=" + std::string(all_zero ? "1" : "0");
  return ok && !all_zero;
}

}  // namespace vision_detection_pose_cloud
