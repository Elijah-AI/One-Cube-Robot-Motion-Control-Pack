#include "task_manager/task_action_client.hpp"
#include "task_manager/log_helper.hpp"
#include <random>
#include <sstream>
#include <uuid/uuid.h>
#include <cmath>
#include <iomanip>
#include <utility>

// 使用 com_interfaces 中的 Action 定义
#include "com_interfaces/action/vision_detection.hpp"
#include "com_interfaces/action/control_gripper.hpp"
#include "com_interfaces/action/move_pose.hpp"
#include "com_interfaces/action/move_joints.hpp"
#include "com_interfaces/action/control_suck.hpp"

namespace task_act
{
TaskActionClient::TaskActionClient(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // 使用正确的 Action 类型
  vision_client_ = rclcpp_action::create_client<VisionDet>(node_, "/vision/detection_pose_cloud");
  arm_client_ = rclcpp_action::create_client<MovePose>(node_, "/Control/move_pose");
  gripper_client_ = rclcpp_action::create_client<CtrlGripper>(node_, "/Control/control_gripper");
  move_joints_client_ = rclcpp_action::create_client<MoveJoints>(node_, "/Control/move_joints");
  control_suck_client_ = rclcpp_action::create_client<CtrlSuck>(node_, "/Control/control_suck");
  // HardwareCheck 客户端（用于视觉自检 /vision/Check_self）
  hw_check_client_ = rclcpp_action::create_client<HardWareCheck>(node_, "/vision/Check_self");
  // HardwareReset 客户端（用于视觉复位 /vision/Reset）
  hw_reset_client_ = rclcpp_action::create_client<HardWareReset>(node_, "/vision/Reset");
  // 运控自检/复位独立客户端（必须为成员变量，否则回调无法触发）
  control_check_client_ = rclcpp_action::create_client<HardWareCheck>(node_, "/Control/Checkself");
  control_reset_client_ = rclcpp_action::create_client<HardWareReset>(node_, "/Control/Reset");
  std::shared_future<GoalHandleCtrlGripper> gripper_send_future_{};
  reset_state();

  // 初始化所有资源锁为未占用状态
  for (size_t i = 0; i < static_cast<size_t>(ResourceType::COUNT); i++)
  {
    resource_locks_[i].store(false, std::memory_order_release);
  }
}

// ============================================================================
// ActionContext 访问方法
// ============================================================================

const ActionContext& TaskActionClient::get_ctx(const std::string& state_key) const
{
    // 自动创建默认上下文
    std::lock_guard<std::mutex> lock(ctx_mutex_);
    auto it = action_contexts_.find(state_key);
    if (it == action_contexts_.end())
    {
        // 插入默认上下文
        action_contexts_.emplace(state_key, ActionContext());
        it = action_contexts_.find(state_key);
    }
    return it->second;
}

ActionContext& TaskActionClient::get_ctx(const std::string& state_key)
{
    std::lock_guard<std::mutex> lock(ctx_mutex_);
    return action_contexts_[state_key];
}

void TaskActionClient::begin_task_timing()
{
  std::lock_guard<std::mutex> lock(action_timing_mutex_);
  action_timing_records_.clear();
  active_action_timing_index_.clear();
  action_timing_active_ = true;
}

void TaskActionClient::start_action_timing(
  const std::string& state_key, const std::string& action_type)
{
  std::lock_guard<std::mutex> lock(action_timing_mutex_);
  if (!action_timing_active_)
  {
    return;
  }

  AtomicActionTimingRecord record;
  record.sequence = action_timing_records_.size() + 1U;
  record.state_key = state_key;
  record.action_type = action_type;
  record.started_at = std::chrono::steady_clock::now();
  action_timing_records_.push_back(std::move(record));
  active_action_timing_index_[state_key] = action_timing_records_.size() - 1U;
}

void TaskActionClient::finish_action_timing(
  const std::string& state_key,
  const std::string& result,
  bool timing_breakdown_valid,
  double ik_time,
  double planning_time,
  double motion_time,
  bool use_client_total_as_motion)
{
  std::lock_guard<std::mutex> lock(action_timing_mutex_);
  if (!action_timing_active_)
  {
    return;
  }

  const auto index_it = active_action_timing_index_.find(state_key);
  if (index_it == active_action_timing_index_.end() ||
      index_it->second >= action_timing_records_.size())
  {
    return;
  }

  auto& record = action_timing_records_[index_it->second];
  if (record.completed)
  {
    return;
  }

  record.finished_at = std::chrono::steady_clock::now();
  record.completed = true;
  record.result = result;
  const bool finite_breakdown =
    std::isfinite(ik_time) && ik_time >= 0.0 &&
    std::isfinite(planning_time) && planning_time >= 0.0 &&
    std::isfinite(motion_time) && motion_time >= 0.0;
  record.timing_breakdown_valid = timing_breakdown_valid && finite_breakdown;
  if (record.timing_breakdown_valid)
  {
    record.ik_time = ik_time;
    record.planning_time = planning_time;
    record.motion_time = motion_time;
  }
  if (use_client_total_as_motion)
  {
    record.timing_breakdown_valid = true;
    record.ik_time = 0.0;
    record.planning_time = 0.0;
    record.motion_time = std::chrono::duration<double>(
      record.finished_at - record.started_at).count();
  }
  active_action_timing_index_.erase(index_it);
}

void TaskActionClient::log_task_timing_summary(
  const std::string& task_name, const std::string& task_result)
{
  std::vector<AtomicActionTimingRecord> records;
  const auto summary_time = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(action_timing_mutex_);
    action_timing_active_ = false;
    for (auto& record : action_timing_records_)
    {
      if (!record.completed)
      {
        record.finished_at = summary_time;
        record.result = "INCOMPLETE";
      }
    }
    records = action_timing_records_;
    active_action_timing_index_.clear();
  }

  LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::ACTION,
    "===== Atomic Action Timing Summary: task=" << task_name
    << ", result=" << task_result << ", count=" << records.size()
    << " =====");

  double summed_client_total = 0.0;
  for (const auto& record : records)
  {
    const double client_total = std::max(
      0.0, std::chrono::duration<double>(
        record.finished_at - record.started_at).count());
    summed_client_total += client_total;

    std::ostringstream line;
    line << "[" << std::setw(2) << std::setfill('0') << record.sequence << "]"
         << " state_key=" << record.state_key
         << ", action=" << record.action_type
         << ", result=" << record.result
         << std::fixed << std::setprecision(6)
         << ", client_total=" << client_total << "s";
    if (record.timing_breakdown_valid)
    {
      line << ", motion_time=" << record.motion_time << "s"
           << ", ik_time=" << record.ik_time << "s"
           << ", planning_time=" << record.planning_time << "s";
    }
    else
    {
      line << ", motion_time=N/A, ik_time=N/A, planning_time=N/A";
    }
    LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, line.str());
  }

  LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::ACTION,
    "===== Atomic Action Timing End: summed_client_total="
    << std::fixed << std::setprecision(6) << summed_client_total
    << "s (parallel actions may overlap) =====");
}

// ============================================================================
// call_vision_detect
// ============================================================================

ActionCallResult
TaskActionClient::call_vision_detect(
  const std::string& task_id, const task_cfg::VisionActionParam& param,
  const std::string& state_key)
{

  LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::VISION, "===== Vision Detect Start =====");
  
  // 参数校验
  if (task_id.empty())
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "task_id is empty");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }
  if (param.target_obj_name.empty())
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "target_obj_name is empty");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }

  auto goal = VisionDet::Goal();
  goal.header.stamp = node_->now();
  goal.task_id = task_id;
  goal.target_obj_name = param.target_obj_name;
  goal.expected_frame_id = param.expected_frame_id;
  goal.need_6d_pose = param.need_6d_pose;
  goal.need_env_point_cloud = param.need_env_point_cloud;
  goal.point_cloud_downsample_resolution = param.downsample_res;
  goal.priority = param.priority;
  goal.type = param.type;
  
  LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "Goal: task_id=" << goal.task_id
    << ", target=" << goal.target_obj_name
    << ", frame_id=" << goal.expected_frame_id
    << ", need_6d=" << (goal.need_6d_pose ? "true" : "false")
    << ", need_cloud=" << (goal.need_env_point_cloud ? "true" : "false")
    << ", downsample=" << goal.point_cloud_downsample_resolution
    << ", priority=" << goal.priority
    << ", type=" << goal.type);
  
  // 增加等待时间和重试机制
  int max_retries = 3;
  int retry_count = 0;
  bool server_ready = false;
  
  while (retry_count < max_retries && !server_ready) {
    LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "Waiting for vision action server (attempt " << (retry_count + 1) << "/" << max_retries << ")...");
    
    server_ready = vision_client_->wait_for_action_server(std::chrono::seconds(5));
    
    if (!server_ready) {
      retry_count++;
      if (retry_count < max_retries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
  
  if (!server_ready) {
    LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision action server timeout after " << max_retries << " attempts");
    return ActionCallResult::FAIL_SERVER_TIMEOUT;
  }
  
  LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "Vision action server connected successfully");

  // 5. 配置异步回调选项 (SendGoalOptions)
  auto send_goal_options = rclcpp_action::Client<VisionDet>::SendGoalOptions();
  
  // 目标响应回调：使用 lambda 捕获 state_key，更新正确的 context
  send_goal_options.goal_response_callback =
    [this, state_key](GoalHandleVisionDetection::SharedPtr goal_handle)
    {
      auto goal = goal_handle.get();
      if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Goal was rejected by server [" << state_key << "]");
        finish_action_timing(state_key, "GOAL_REJECTED");
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
      } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "Goal accepted by server [" << state_key << "], waiting for result...");
      }
    };
    
  // 反馈回调：服务器周期性发送进度时触发
  send_goal_options.feedback_callback =
    std::bind(&TaskActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    
  // 结果回调：使用 lambda 捕获 state_key，更新正确的 context
  send_goal_options.result_callback =
    [this, state_key](const GoalHandleVisionDetection::WrappedResult & result)
    {
      const bool succeeded = result.result && result.result->success;
      finish_action_timing(
        state_key, succeeded ? "SUCCESS" : "FAILURE");
      if (!result.result)
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION,
          "Vision detect returned an empty result [" << state_key << "]");
        complete_action_context(
          state_key, ActionRunState::FAILURE,
          ActionCallResult::FAIL_GENERAL);
        return;
      }
      auto& ctx = get_ctx(state_key);

      if(result.result->success)
      {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::VISION, "Vision detect succeeded [" << state_key << "]");
        print_vision_result(node_->get_logger(), *result.result);

        // ========== 保存视觉检测结果到 context，供后续动作使用 ==========
        ctx.vision_result = *result.result;
        ctx.has_vision_result = true;

        ctx.state = ActionRunState::SUCCESS;
        ctx.call_result = ActionCallResult::SUCCESS;
      }
      else
      {
        // 失败时标记无有效结果
        ctx.has_vision_result = false;

        if (result.result->status_code == 3)
        {
          LOG_WARN(node_->get_logger(), task_log::Tag::VISION, "Vision detect server busy [" << state_key << "]: " << result.result->error_message);
          ctx.call_result = ActionCallResult::FAIL_SERVER_BUSY;
        }
        else
        {
          LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision detect failed [" << state_key << "]: " << result.result->error_message);
          ctx.call_result = ActionCallResult::FAIL_GENERAL;
        }
        ctx.state = ActionRunState::FAILURE;
      }

      // 释放该 action 占用的资源（遍历所有 context）
      {
        std::lock_guard<std::mutex> lock(ctx_mutex_);
        for (auto& [key, ctx_inner] : action_contexts_)
        {
          if (!ctx_inner.acquired_resources.empty())
          {
            for (const auto& res : ctx_inner.acquired_resources)
            {
              release_resource(res);
            }
            ctx_inner.acquired_resources.clear();
          }
        }
      }
    };
  auto send_goal_future = vision_client_->async_send_goal(goal, send_goal_options);
  LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::VISION, "===== Vision Detect End =====");
  return ActionCallResult::SUCCESS;

}
void TaskActionClient::goal_response_callback(GoalHandleVisionDetection::SharedPtr goal_handle)
{
    auto goal = goal_handle.get();
    if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Goal was rejected by server");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
    } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "Goal accepted by server, waiting for result...");
    }
}

void TaskActionClient::feedback_callback(GoalHandleVisionDetection::SharedPtr, const std::shared_ptr<const VisionDet::Feedback> feedback)
{
  LOG_DEBUG(node_->get_logger(), task_log::Tag::VISION, "Feedback: progress=" << (feedback ? std::to_string(feedback->progress_rate) : "N/A"));
}

void TaskActionClient::result_callback(const GoalHandleVisionDetection::WrappedResult & result)
{
  if(result.result->success)
  {
    LOG_SUCCESS(node_->get_logger(), task_log::Tag::VISION, "Vision detect succeeded");
    print_vision_result(node_->get_logger(), *result.result);
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::SUCCESS;
    ctx.call_result = ActionCallResult::SUCCESS;
    
  }
  else
  {
    // 根据错误码区分失败原因
    if (result.result->status_code == 3)
    {
      LOG_WARN(node_->get_logger(), task_log::Tag::VISION, "Vision detect server busy: " << result.result->error_message);
      auto& ctx = get_ctx("default");
      ctx.call_result = ActionCallResult::FAIL_SERVER_BUSY;
    }
    else
    {
      LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision detect failed: " << result.result->error_message);
      auto& ctx = get_ctx("default");
      ctx.call_result = ActionCallResult::FAIL_GENERAL;
    }
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::FAILURE;
  }

  // 释放该 action 占用的资源（遍历所有 context，释放 vision 类型的资源）
  {
    std::lock_guard<std::mutex> lock(ctx_mutex_);
    for (auto& [key, ctx] : action_contexts_)
    {
      if (!ctx.acquired_resources.empty())
      {
        for (const auto& res : ctx.acquired_resources)
        {
          release_resource(res);
        }
        ctx.acquired_resources.clear();
      }
    }
  }
}

void TaskActionClient::print_vision_result(rclcpp::Logger logger, const VisionDet::Result & res)
{
  LOG_INFO_TITLE(logger, task_log::Tag::VISION, "===== VisionDetection Result =====");
  LOG_INFO(logger, task_log::Tag::VISION, "success: " << (res.success ? "true" : "false"));

  // status_code 文字解析
  std::string status_str;
  switch (res.status_code)
  {
    case 0: status_str = "OK"; break;
    case 1: status_str = "FAILED (hardware fault)"; break;
    case 2: status_str = "TIMEOUT"; break;
    case 3: status_str = "BUSY (high priority task running)"; break;
    default: status_str = "Unknown(" + std::to_string(res.status_code) + ")";
  }
  LOG_INFO(logger, task_log::Tag::VISION, "status_code: " << status_str);
  LOG_INFO(logger, task_log::Tag::VISION, "error_message: " << res.error_message);
  LOG_INFO(logger, task_log::Tag::VISION, "execution_time: " << res.execution_time << " s");

  // 6D PoseStamped
  const auto & pose = res.target_6d_pose;
  LOG_INFO(logger, task_log::Tag::VISION, "target_6d_pose.frame_id: " << pose.header.frame_id);
  LOG_INFO(logger, task_log::Tag::VISION, "target_6d_pose.position: x=" << pose.pose.position.x
    << ", y=" << pose.pose.position.y << ", z=" << pose.pose.position.z);
  LOG_INFO(logger, task_log::Tag::VISION, "target_6d_pose.orientation: qx=" << pose.pose.orientation.x
    << ", qy=" << pose.pose.orientation.y << ", qz=" << pose.pose.orientation.z << ", qw=" << pose.pose.orientation.w);
  LOG_INFO(logger, task_log::Tag::VISION, "pose_confidence: " << res.pose_confidence);

  // 点云信息（只打印基础信息，不打印海量点）
  LOG_INFO(logger, task_log::Tag::VISION, "env_point_cloud.frame_id: " << res.env_point_cloud.header.frame_id
    << ", width=" << res.env_point_cloud.width << ", height=" << res.env_point_cloud.height
    << ", point_step=" << res.env_point_cloud.point_step);
  LOG_INFO(logger, task_log::Tag::VISION, "point_cloud_frame_id: " << res.point_cloud_frame_id);

  // 额外打印 XYZABC 格式（四元数转欧拉角）
  print_vision_result_xyzabc(logger, res);
}

// ============================================================================
// 四元数转欧拉角（静态工具函数）
// ============================================================================
void TaskActionClient::quaternion_to_rpy(double qx, double qy, double qz, double qw,
                                          double& roll, double& pitch, double& yaw)
{
    // roll (绕 X 轴)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (绕 Y 轴)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);

    // yaw (绕 Z 轴)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    // 转换为度
    roll  = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw   = yaw * 180.0 / M_PI;
}

// ============================================================================
// print_vision_result_xyzabc - 打印视觉检测结果的 XYZABC 格式
// ============================================================================
void TaskActionClient::print_vision_result_xyzabc(rclcpp::Logger logger, const VisionDet::Result & res)
{
  const auto & pose = res.target_6d_pose;
  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double z = pose.pose.position.z;
  double qx = pose.pose.orientation.x;
  double qy = pose.pose.orientation.y;
  double qz = pose.pose.orientation.z;
  double qw = pose.pose.orientation.w;

  double roll_deg, pitch_deg, yaw_deg;
  quaternion_to_rpy(qx, qy, qz, qw, roll_deg, pitch_deg, yaw_deg);

  LOG_INFO_TITLE(logger, task_log::Tag::VISION, "===== 6D Pose (XYZABC) =====");
  LOG_INFO(logger, task_log::Tag::VISION, "Frame ID: " << pose.header.frame_id);
  LOG_INFO(logger, task_log::Tag::VISION, "X (mm): " << std::fixed << std::setprecision(2) << x * 1000.0
    << ", Y (mm): " << y * 1000.0 << ", Z (mm): " << z * 1000.0);
  LOG_INFO(logger, task_log::Tag::VISION, "A (RotX/deg): " << std::fixed << std::setprecision(2) << roll_deg
    << ", B (RotY/deg): " << pitch_deg << ", C (RotZ/deg): " << yaw_deg);
  LOG_INFO(logger, task_log::Tag::VISION, "XYZABC: [" << std::fixed << std::setprecision(2)
    << x * 1000.0 << ", " << y * 1000.0 << ", " << z * 1000.0 << ", "
    << roll_deg << ", " << pitch_deg << ", " << yaw_deg << "]");
  LOG_INFO(logger, task_log::Tag::VISION, "Confidence: " << std::fixed << std::setprecision(2) << res.pose_confidence * 100.0 << "%");
}



// ============================================================================
// call_move_pose
// ============================================================================

ActionCallResult TaskActionClient::call_move_pose(
  const std::string& task_id, const task_cfg::MovePoseParam& param,
  const geometry_msgs::msg::PoseStamped& target_pose,
  const sensor_msgs::msg::PointCloud2& cloud, const std::string& frame_id,
  const std::string& state_key)
{
  // 参数校验
  if (task_id.empty())
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "task_id is empty");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }
  if (param.arm_id != MovePose::Goal::ARM_LEFT && param.arm_id != MovePose::Goal::ARM_RIGHT &&
      param.arm_id != MovePose::Goal::ARM_BOTH && param.arm_id != MovePose::Goal::HEAD &&
      param.arm_id != MovePose::Goal::LEG)
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "invalid arm_id: " << static_cast<int>(param.arm_id));
    return ActionCallResult::FAIL_PARAM_INVALID;
  }

  auto goal = MovePose::Goal();
  goal.header.stamp = node_->now();
  goal.header.frame_id = param.frame_id;

  goal.task_id = task_id;

  // arm_id 直接使用 uint8，与 com_interfaces 保持一致
  goal.arm_id = param.arm_id;

  goal.enable_collision_check = param.enable_collision_check;

  //   com_interfaces/action/MovePose.action 定义：
  //   ARM_LEFT/ARM_RIGHT/HEAD/LEG → 使用 target_pose + bound_size
  //   ARM_BOTH → 使用 left_target_pose/left_bound_size + right_target_pose/right_bound_size + synchronization_mode
  if (param.arm_id == MovePose::Goal::ARM_BOTH)
  {
    // 填充左臂目标位姿（YAML 中为 [x, y, z, qx, qy, qz, qw] 浮点数组）

    if (param.use_config_pose)
    {
      if (param.left_target_pose.size() >= 7)
      {
        goal.left_target_pose.header.stamp = node_->now();
        goal.left_target_pose.header.frame_id = param.frame_id;
        goal.left_target_pose.pose.position.x    = param.left_target_pose[0];
        goal.left_target_pose.pose.position.y    = param.left_target_pose[1];
        goal.left_target_pose.pose.position.z    = param.left_target_pose[2];
        goal.left_target_pose.pose.orientation.x = param.left_target_pose[3];
        goal.left_target_pose.pose.orientation.y = param.left_target_pose[4];
        goal.left_target_pose.pose.orientation.z = param.left_target_pose[5];
        goal.left_target_pose.pose.orientation.w = param.left_target_pose[6];
      }
      // 填充右臂目标位姿
      if (param.right_target_pose.size() >= 7)
      {
        goal.right_target_pose.header.stamp = node_->now();
        goal.right_target_pose.header.frame_id = param.frame_id;
        goal.right_target_pose.pose.position.x    = param.right_target_pose[0];
        goal.right_target_pose.pose.position.y    = param.right_target_pose[1];
        goal.right_target_pose.pose.position.z    = param.right_target_pose[2];
        goal.right_target_pose.pose.orientation.x = param.right_target_pose[3];
        goal.right_target_pose.pose.orientation.y = param.right_target_pose[4];
        goal.right_target_pose.pose.orientation.z = param.right_target_pose[5];
        goal.right_target_pose.pose.orientation.w = param.right_target_pose[6];
      }
    }
    else
    {
      // 如果不使用配置位姿，则直接使用传入的 target_pose
      goal.left_target_pose = target_pose;
      goal.right_target_pose = target_pose;
      // 强制使用配置文件中定义的 frame_id（配置了非空 frame_id 时）
      if (!param.frame_id.empty())
      {
        goal.left_target_pose.header.frame_id = param.frame_id;
        goal.right_target_pose.header.frame_id = param.frame_id;
      }
    }

    // 填充左右包围盒尺寸（固定长度 3）
    for (size_t i = 0; i < param.left_bound_size.size() && i < goal.left_bound_size.size(); ++i)
    {
      goal.left_bound_size[i] = param.left_bound_size[i];
    }

    for (size_t i = 0; i < param.right_bound_size.size() && i < goal.right_bound_size.size(); ++i)
    {
      goal.right_bound_size[i] = param.right_bound_size[i];
    }

    goal.synchronization_mode = param.synchronization_mode;
  }
  else
  {
    if (param.use_config_pose)
    {
      goal.target_pose = target_pose;
      if (param.config_pose.size() >= 7)
      {
        goal.target_pose.pose.position.x    = param.config_pose[0];
        goal.target_pose.pose.position.y    = param.config_pose[1];
        goal.target_pose.pose.position.z    = param.config_pose[2];
        goal.target_pose.pose.orientation.x = param.config_pose[3];
        goal.target_pose.pose.orientation.y = param.config_pose[4];
        goal.target_pose.pose.orientation.z = param.config_pose[5];
        goal.target_pose.pose.orientation.w = param.config_pose[6];
      }
      else
      {
        LOG_WARN(node_->get_logger(), task_log::Tag::ARM, "config_pose size < 7, using target_pose directly");
      }
    }
    else
    {
      goal.target_pose = target_pose;
    }
    // 若配置文件中定义了 frame_id 则强制使用，否则保留 target_pose 自带的 frame_id
    if (!param.frame_id.empty())
    {
      goal.target_pose.header.frame_id = param.frame_id;
    }
    if (!param.bound_size.empty())
    {
      goal.bound_size.resize(param.bound_size.size());
      for (size_t i = 0; i < param.bound_size.size(); ++i)
      {
        goal.bound_size[i] = param.bound_size[i];
      }
    }
  }
  goal.use_force_control = param.use_force_control;

  goal.speed = param.speed;
  if (!param.hand_action_type.empty())
  {
    goal.hand_action_type = param.hand_action_type;
  }
  goal.env_point_cloud = cloud;
  goal.point_cloud_frame_id = frame_id;
  goal.timeout_ms = param.timeout_ms;
  goal.enable_move_line = param.enable_move_line;

  // ========== 参数打印（实际发送的 Goal） ==========
  {
    std::ostringstream oss;
    oss << "MovePose Goal: task_id=" << goal.task_id
        << "\n arm_id=" << static_cast<int>(goal.arm_id)
        << "\n collision_check=" << (goal.enable_collision_check ? "true" : "false")
        << "\n force_control=" << (goal.use_force_control ? "true" : "false")
        << "\n enable_move_line=" << (goal.enable_move_line ? "true" : "false")
        << "\n speed=" << goal.speed
        << "\n timeout_ms=" << goal.timeout_ms
        << "\n cloud_frame=" << goal.point_cloud_frame_id
        << "\n cloud_size=" << goal.env_point_cloud.data.size()
        << "\n Target.head.frame_id=" << goal.target_pose.header.frame_id
        << "\n";
    if (goal.arm_id == MovePose::Goal::ARM_BOTH)
    {

      oss << "\n  [ARM_BOTH] sync_mode=" << static_cast<int>(goal.synchronization_mode);
      oss << "\n  left_target_pose=("
          << goal.left_target_pose.pose.position.x << ","
          << goal.left_target_pose.pose.position.y << ","
          << goal.left_target_pose.pose.position.z << ","
          << goal.left_target_pose.pose.orientation.x << ","
          << goal.left_target_pose.pose.orientation.y << ","
          << goal.left_target_pose.pose.orientation.z << ","
          << goal.left_target_pose.pose.orientation.w << ")";
      oss << "\n  right_target_pose=("
          << goal.right_target_pose.pose.position.x << ","
          << goal.right_target_pose.pose.position.y << ","
          << goal.right_target_pose.pose.position.z << ","
          << goal.right_target_pose.pose.orientation.x << ","
          << goal.right_target_pose.pose.orientation.y << ","
          << goal.right_target_pose.pose.orientation.z << ","
          << goal.right_target_pose.pose.orientation.w << ")";
      oss << "\n  left_bound_size=(";
      for (size_t i = 0; i < goal.left_bound_size.size(); ++i)
        oss << (i ? "," : "") << goal.left_bound_size[i];
      oss << ")\nright_bound_size=(";
      for (size_t i = 0; i < goal.right_bound_size.size(); ++i)
        oss << (i ? "," : "") << goal.right_bound_size[i];
      oss << ")";
    }
    else
    {
      oss << "\n  use_config_pose=" << (param.use_config_pose ? "true" : "false");
      oss << "\n  frame_id=(";
      oss << goal.target_pose.header.frame_id << ")";
      oss << "\n  target_pose=("
          << goal.target_pose.pose.position.x << ","
          << goal.target_pose.pose.position.y << ","
          << goal.target_pose.pose.position.z << ","
          << goal.target_pose.pose.orientation.x << ","
          << goal.target_pose.pose.orientation.y << ","
          << goal.target_pose.pose.orientation.z << ","
          << goal.target_pose.pose.orientation.w << ")";
      oss << "\n  bound_size=(";
      for (size_t i = 0; i < goal.bound_size.size(); ++i)
        oss << (i ? "," : "") << goal.bound_size[i];
      oss << ")";
    }
    if (!goal.hand_action_type.empty())
    {
      oss << "\n  hand_action_type=[";
      for (size_t i = 0; i < goal.hand_action_type.size(); ++i)
        oss << (i ? "," : "") << static_cast<int>(goal.hand_action_type[i]);
      oss << "]";
    }
    LOG_INFO(node_->get_logger(), task_log::Tag::ARM, oss.str());
  }

  // 等待服务端就绪
  int max_retries = 3;
  int retry_count = 0;
  bool server_ready = false;

  while (retry_count < max_retries && !server_ready) {
    LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "Waiting for MovePose action server (attempt " << (retry_count + 1) << "/" << max_retries << ")...");

    server_ready = arm_client_->wait_for_action_server(std::chrono::seconds(5));

    if (!server_ready) {
      retry_count++;
      if (retry_count < max_retries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }

  if (!server_ready) {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MovePose action server timeout after " << max_retries << " attempts");
    return ActionCallResult::FAIL_SERVER_TIMEOUT;
  }

  LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "MovePose action server connected successfully");

  auto send_goal_options = rclcpp_action::Client<MovePose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this, state_key](GoalHandleMovePose::SharedPtr goal_handle)
    {
      auto goal = goal_handle.get();
      if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Goal was rejected by server [" << state_key << "]");
        finish_action_timing(state_key, "GOAL_REJECTED");
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
      } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "Goal accepted by server [" << state_key << "], waiting for result...");
      }
    };

  send_goal_options.feedback_callback =
    std::bind(&TaskActionClient::arm_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

  send_goal_options.result_callback =
    [this, state_key](const GoalHandleMovePose::WrappedResult & result)
    {
      const bool succeeded = result.result && result.result->success;
      finish_action_timing(
        state_key, succeeded ? "SUCCESS" : "FAILURE",
        result.result && result.result->timing_breakdown_valid,
        result.result ? result.result->ik_time : 0.0,
        result.result ? result.result->planning_time : 0.0,
        result.result ? result.result->motion_time : 0.0);
      if (!result.result)
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM,
          "MovePose returned an empty result [" << state_key << "]");
        complete_action_context(
          state_key, ActionRunState::FAILURE,
          ActionCallResult::FAIL_GENERAL);
        return;
      }
      if(result.result->success)
      {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::ARM, "MovePose succeeded [" << state_key << "]");
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::SUCCESS;
        ctx.call_result = ActionCallResult::SUCCESS;
      }
      else
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MovePose failed [" << state_key << "]: " << result.result->error_message);
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GENERAL;
      }

      // 释放该 action 占用的资源
      {
        std::lock_guard<std::mutex> lock(ctx_mutex_);
        for (auto& [key, ctx] : action_contexts_)
        {
          if (!ctx.acquired_resources.empty())
          {
            for (const auto& res : ctx.acquired_resources)
            {
              release_resource(res);
            }
            ctx.acquired_resources.clear();
          }
        }
      }
    };

  arm_client_->async_send_goal(goal, send_goal_options);
  return ActionCallResult::SUCCESS;
}

// ============================================================================
// call_move_joints
// ============================================================================

ActionCallResult TaskActionClient::call_move_joints(
  const std::string& task_id, const task_cfg::MoveJointsParam& param,
  const std::string& state_key)
{
  if (task_id.empty())
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "task_id is empty");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }
  if (param.arm_id != MoveJoints::Goal::ARM_LEFT && param.arm_id != MoveJoints::Goal::ARM_RIGHT &&
      param.arm_id != MoveJoints::Goal::ARM_BOTH && param.arm_id != MoveJoints::Goal::HEAD &&
      param.arm_id != MoveJoints::Goal::LEG)
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "invalid arm_id: " << static_cast<int>(param.arm_id));
    return ActionCallResult::FAIL_PARAM_INVALID;
  }

  if (param.arm_id == MoveJoints::Goal::ARM_BOTH)
  {
    constexpr size_t kArmJointCount = 7U;
    if (param.left_joint_name.size() != kArmJointCount ||
        param.left_position.size() != kArmJointCount ||
        param.right_joint_name.size() != kArmJointCount ||
        param.right_position.size() != kArmJointCount)
    {
      LOG_ERROR(node_->get_logger(), task_log::Tag::ARM,
        "ARM_BOTH requires exactly 7 joint names and positions per arm");
      return ActionCallResult::FAIL_PARAM_INVALID;
    }
  }
  else
  {
    const size_t expected_joint_count =
      (param.arm_id == MoveJoints::Goal::HEAD) ? 2U :
      (param.arm_id == MoveJoints::Goal::LEG) ? 4U : 7U;
    if (param.joint_name.size() != expected_joint_count ||
        param.position.size() != expected_joint_count)
    {
      LOG_ERROR(node_->get_logger(), task_log::Tag::ARM,
        "arm_id=" << static_cast<int>(param.arm_id) << " requires exactly "
        << expected_joint_count << " joint names and positions, got names="
        << param.joint_name.size() << ", positions=" << param.position.size());
      return ActionCallResult::FAIL_PARAM_INVALID;
    }
  }

  auto goal = MoveJoints::Goal();
  goal.header.stamp = node_->now();
  goal.task_id = task_id;

  // arm_id 直接使用 uint8，与 com_interfaces 保持一致
  goal.arm_id = param.arm_id;

  // com_interfaces/action/MoveJoints.action 定义：
  //   ARM_LEFT/ARM_RIGHT/HEAD/LEG → 使用 joint_name + position（可变长 vector）
  //   ARM_BOTH → 使用 left_joint_name/left_position + right_joint_name/right_position（固定长 array<7>）
  if (param.arm_id == MoveJoints::Goal::ARM_BOTH)
  {
    for (size_t i = 0; i < param.left_joint_name.size() && i < goal.left_joint_name.size(); ++i)
      goal.left_joint_name[i] = param.left_joint_name[i];
    for (size_t i = 0; i < param.left_position.size() && i < goal.left_position.size(); ++i)
      goal.left_position[i] = param.left_position[i];
    for (size_t i = 0; i < param.right_joint_name.size() && i < goal.right_joint_name.size(); ++i)
      goal.right_joint_name[i] = param.right_joint_name[i];
    for (size_t i = 0; i < param.right_position.size() && i < goal.right_position.size(); ++i)
      goal.right_position[i] = param.right_position[i];
    goal.synchronization_mode = param.synchronization_mode;
  }
  else
  {
    goal.joint_name = param.joint_name;
    goal.position = param.position;
  }
  goal.enable_collision_check = param.enable_collision_check;
  goal.speed = param.speed;
  goal.timeout_ms = param.timeout_ms;

  // ========== 参数打印（实际发送的 Goal） ==========
  {
    std::ostringstream oss;
    oss << "MoveJoints Goal: task_id=" << goal.task_id
        << ", arm_id=" << static_cast<int>(goal.arm_id)
        << ", collision_check=" << (goal.enable_collision_check ? "true" : "false")
        << ", speed=" << goal.speed
        << ", timeout_ms=" << goal.timeout_ms;
    if (goal.arm_id == MoveJoints::Goal::ARM_BOTH)
    {
      oss << "\n  [ARM_BOTH] sync_mode=" << static_cast<int>(goal.synchronization_mode);
      oss << "\n  left_joint_name=[";
      for (size_t i = 0; i < goal.left_joint_name.size(); ++i)
        oss << (i ? "," : "") << goal.left_joint_name[i];
      oss << "]";
      oss << "\n  left_position=[";
      for (size_t i = 0; i < goal.left_position.size(); ++i)
        oss << (i ? "," : "") << goal.left_position[i];
      oss << "]";
      oss << "\n  right_joint_name=[";
      for (size_t i = 0; i < goal.right_joint_name.size(); ++i)
        oss << (i ? "," : "") << goal.right_joint_name[i];
      oss << "]";
      oss << "\n  right_position=[";
      for (size_t i = 0; i < goal.right_position.size(); ++i)
        oss << (i ? "," : "") << goal.right_position[i];
      oss << "]";
    }
    else
    {
      oss << "\n  joint_name=[";
      for (size_t i = 0; i < goal.joint_name.size(); ++i)
        oss << (i ? "," : "") << goal.joint_name[i];
      oss << "]";
      oss << "\n  position=[";
      for (size_t i = 0; i < goal.position.size(); ++i)
        oss << (i ? "," : "") << goal.position[i];
      oss << "]";
    }
    LOG_INFO(node_->get_logger(), task_log::Tag::ARM, oss.str());
  }

  // 等待服务端就绪
  int max_retries = 3;
  int retry_count = 0;
  bool server_ready = false;

  while (retry_count < max_retries && !server_ready) {
    LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "Waiting for MoveJoints action server (attempt " << (retry_count + 1) << "/" << max_retries << ")...");

    server_ready = move_joints_client_->wait_for_action_server(std::chrono::seconds(5));

    if (!server_ready) {
      retry_count++;
      if (retry_count < max_retries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }

  if (!server_ready) {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MoveJoints action server timeout after " << max_retries << " attempts");
    return ActionCallResult::FAIL_SERVER_TIMEOUT;
  }

  LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "MoveJoints action server connected successfully");

  auto send_goal_options = rclcpp_action::Client<MoveJoints>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this, state_key](GoalHandleMoveJoints::SharedPtr goal_handle)
    {
      auto goal = goal_handle.get();
      if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MoveJoints goal rejected by server [" << state_key << "]");
        finish_action_timing(state_key, "GOAL_REJECTED");
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
      } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "MoveJoints goal accepted by server [" << state_key << "], waiting for result...");
      }
    };

  send_goal_options.feedback_callback =
    std::bind(&TaskActionClient::move_joints_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

  send_goal_options.result_callback =
    [this, state_key](const GoalHandleMoveJoints::WrappedResult & result)
    {
      const bool succeeded = result.result && result.result->success;
      finish_action_timing(
        state_key, succeeded ? "SUCCESS" : "FAILURE",
        result.result && result.result->timing_breakdown_valid,
        result.result ? result.result->ik_time : 0.0,
        result.result ? result.result->planning_time : 0.0,
        result.result ? result.result->motion_time : 0.0);
      if (!result.result)
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM,
          "MoveJoints returned an empty result [" << state_key << "]");
        complete_action_context(
          state_key, ActionRunState::FAILURE,
          ActionCallResult::FAIL_GENERAL);
        return;
      }
      auto& ctx = get_ctx(state_key);
      if(result.result->success)
      {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::ARM, "MoveJoints succeeded [" << state_key << "]");
        ctx.state = ActionRunState::SUCCESS;
        ctx.call_result = ActionCallResult::SUCCESS;
      }
      else
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MoveJoints failed [" << state_key << "]: " << result.result->error_message);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GENERAL;
      }

      // 只释放当前 Goal 的资源；不能影响仍在运行的并行兄弟动作。
      release_acquired_resources(ctx);
    };

  move_joints_client_->async_send_goal(goal, send_goal_options);
  return ActionCallResult::SUCCESS;
}

// ============================================================================
// call_control_suck
// ============================================================================

ActionCallResult TaskActionClient::call_control_suck(
  const std::string& task_id, const task_cfg::SuckParam& param,
  const std::string& state_key)
{
  if (task_id.empty())
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, "task_id is empty (control_suck)");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }

  auto goal = CtrlSuck::Goal();
  goal.header.stamp = node_->now();
  goal.task_id = task_id;

  // suck_id 直接使用 uint8，与 com_interfaces 保持一致
  if (param.suck_id != CtrlSuck::Goal::SUCK_LEFT && param.suck_id != CtrlSuck::Goal::SUCK_RIGHT &&
      param.suck_id != CtrlSuck::Goal::SUCK_BOTH)
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, "invalid suck_id: " << static_cast<int>(param.suck_id));
    return ActionCallResult::FAIL_PARAM_INVALID;
  }
  goal.suck_id = param.suck_id;

  goal.active = param.active;
  goal.timeout_ms = param.timeout_ms;

  // ========== 参数打印（实际发送的 Goal） ==========
  {
    std::ostringstream oss;
    oss << "ControlSuck Goal: task_id=" << goal.task_id
        << ", suck_id=" << static_cast<int>(goal.suck_id)
        << ", active=[";
    for (size_t i = 0; i < goal.active.size(); ++i)
      oss << (i ? "," : "") << (goal.active[i] ? "true" : "false");
    oss << "], timeout_ms=" << goal.timeout_ms;
    LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, oss.str());
  }

  // 等待服务端就绪
  if (!control_suck_client_->wait_for_action_server(std::chrono::seconds(2))) {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck action server not available");
    return ActionCallResult::FAIL_SERVER_TIMEOUT;
  }

  auto send_goal_options = rclcpp_action::Client<CtrlSuck>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this, state_key](GoalHandleCtrlSuck::SharedPtr goal_handle)
    {
      auto goal = goal_handle.get();
      if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck goal rejected by server [" << state_key << "]");
        finish_action_timing(state_key, "GOAL_REJECTED");
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
      } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck goal accepted by server [" << state_key << "], waiting for result...");
      }
    };

  send_goal_options.result_callback =
    [this, state_key](const GoalHandleCtrlSuck::WrappedResult & result)
    {
      const bool succeeded = result.result && result.result->success;
      finish_action_timing(
        state_key, succeeded ? "SUCCESS" : "FAILURE",
        false, 0.0, 0.0, 0.0, true);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      if (!result.result)
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION,
          "ControlSuck returned an empty result [" << state_key << "]");
        complete_action_context(
          state_key, ActionRunState::FAILURE,
          ActionCallResult::FAIL_GENERAL);
        return;
      }
      if(result.result->success)
      {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck succeeded [" << state_key << "]");
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::SUCCESS;
        ctx.call_result = ActionCallResult::SUCCESS;
      }
      else
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck failed [" << state_key << "]: " << result.result->error_message);
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GENERAL;
      }

      {
        std::lock_guard<std::mutex> lock(ctx_mutex_);
        for (auto& [key, ctx] : action_contexts_)
        {
          if (!ctx.acquired_resources.empty())
          {
            for (const auto& res : ctx.acquired_resources)
            {
              release_resource(res);
            }
            ctx.acquired_resources.clear();
          }
        }
      }
    };

  control_suck_client_->async_send_goal(goal, send_goal_options);
  return ActionCallResult::SUCCESS;
}

// ============================================================================
// arm 回调（默认 key 版本，保留兼容性）
// ============================================================================

void TaskActionClient::arm_goal_response_callback(GoalHandleMovePose::SharedPtr goal_handle)
{
    auto goal = goal_handle.get();
    if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Goal was rejected by server");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
    } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "Goal accepted by server, waiting for result...");
    }
}

void TaskActionClient::arm_feedback_callback(GoalHandleMovePose::SharedPtr, const std::shared_ptr<const MovePose::Feedback> feedback)
{
  LOG_DEBUG(node_->get_logger(), task_log::Tag::ARM, "Feedback: progress=" << (feedback ? std::to_string(feedback->progress_rate) : "N/A"));
}

void TaskActionClient::arm_result_callback(const GoalHandleMovePose::WrappedResult & result)
{
  if(result.result->success)
  {
    LOG_SUCCESS(node_->get_logger(), task_log::Tag::ARM, "MovePose succeeded");
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::SUCCESS;
    ctx.call_result = ActionCallResult::SUCCESS;
  }
  else
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MovePose failed: " << result.result->error_message);
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::FAILURE;
    ctx.call_result = ActionCallResult::FAIL_GENERAL;
  }

  {
    std::lock_guard<std::mutex> lock(ctx_mutex_);
    for (auto& [key, ctx] : action_contexts_)
    {
      if (!ctx.acquired_resources.empty())
      {
        for (const auto& res : ctx.acquired_resources)
        {
          release_resource(res);
        }
        ctx.acquired_resources.clear();
      }
    }
  }
}

// ============================================================================
// move_joints 回调（默认 key 版本）
// ============================================================================

void TaskActionClient::move_joints_goal_response_callback(GoalHandleMoveJoints::SharedPtr goal_handle)
{
    auto goal = goal_handle.get();
    if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MoveJoints goal rejected by server");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
    } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "MoveJoints goal accepted by server, waiting for result...");
    }
}

void TaskActionClient::move_joints_feedback_callback(GoalHandleMoveJoints::SharedPtr, const std::shared_ptr<const MoveJoints::Feedback> feedback)
{
  LOG_DEBUG(node_->get_logger(), task_log::Tag::ARM, "MoveJoints feedback: progress=" << (feedback ? std::to_string(feedback->progress_rate) : "N/A"));
}

void TaskActionClient::move_joints_result_callback(const GoalHandleMoveJoints::WrappedResult & result)
{
  if(result.result->success)
  {
    LOG_SUCCESS(node_->get_logger(), task_log::Tag::ARM, "MoveJoints succeeded");
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::SUCCESS;
    ctx.call_result = ActionCallResult::SUCCESS;
  }
  else
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "MoveJoints failed: " << result.result->error_message);
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::FAILURE;
    ctx.call_result = ActionCallResult::FAIL_GENERAL;
  }

  {
    std::lock_guard<std::mutex> lock(ctx_mutex_);
    for (auto& [key, ctx] : action_contexts_)
    {
      if (!ctx.acquired_resources.empty())
      {
        for (const auto& res : ctx.acquired_resources)
        {
          release_resource(res);
        }
        ctx.acquired_resources.clear();
      }
    }
  }
}

// ============================================================================
// control_suck 回调（默认 key 版本）
// ============================================================================

void TaskActionClient::control_suck_goal_response_callback(GoalHandleCtrlSuck::SharedPtr goal_handle)
{
    auto goal = goal_handle.get();
    if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck goal rejected by server");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
    } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck goal accepted by server, waiting for result...");
    }
}

void TaskActionClient::control_suck_feedback_callback(GoalHandleCtrlSuck::SharedPtr, const std::shared_ptr<const CtrlSuck::Feedback> feedback)
{
  LOG_DEBUG(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck feedback: progress=" << (feedback ? std::to_string(feedback->progress_rate) : "N/A"));
}

void TaskActionClient::control_suck_result_callback(const GoalHandleCtrlSuck::WrappedResult & result)
{
  if(result.result->success)
  {
    LOG_SUCCESS(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck succeeded");
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::SUCCESS;
    ctx.call_result = ActionCallResult::SUCCESS;
  }
  else
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, "ControlSuck failed: " << result.result->error_message);
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::FAILURE;
    ctx.call_result = ActionCallResult::FAIL_GENERAL;
  }

  {
    std::lock_guard<std::mutex> lock(ctx_mutex_);
    for (auto& [key, ctx] : action_contexts_)
    {
      if (!ctx.acquired_resources.empty())
      {
        for (const auto& res : ctx.acquired_resources)
        {
          release_resource(res);
        }
        ctx.acquired_resources.clear();
      }
    }
  }
}


// ============================================================================
// call_gripper
// ============================================================================

ActionCallResult TaskActionClient::call_gripper(
  const std::string& task_id, const task_cfg::GripperParam& param,
  const std::string& state_key)
{

  LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::GRIPPER, "===== Gripper Control Start =====");

  // 在访问动态数组前完成校验和整体赋值。
  CtrlGripper::Goal goal;
  std::string goal_error;
  const auto build_result = detail::build_gripper_goal(
    task_id, param, goal, goal_error);
  if (!action_call_result_is_ok(build_result))
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER,
      "Invalid gripper goal: " << goal_error);
    return build_result;
  }
  goal.header.stamp = node_->now();

  // 1. 等待 Server 就绪
  if(!gripper_client_->wait_for_action_server(std::chrono::seconds(2))) {
    LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER, "Gripper action server not available");
    return ActionCallResult::FAIL_SERVER_TIMEOUT;
  }

  LOG_INFO(node_->get_logger(), task_log::Tag::GRIPPER, "Goal: task_id=" << task_id
    << ", gripper_id=" << (int)goal.gripper_id
    << ", position=[" << (int)goal.position[0] << "," << (int)goal.position[1] << "]"
    << ", block=" << (goal.block_until_complete ? "true" : "false")
    << ", timeout=" << goal.timeout_ms << "ms");
  
  // 增加等待时间和重试机制
  int max_retries = 3;
  int retry_count = 0;
  bool server_ready = false;
  
  while (retry_count < max_retries && !server_ready) {
    LOG_INFO(node_->get_logger(), task_log::Tag::GRIPPER, "Waiting for gripper action server (attempt " << (retry_count + 1) << "/" << max_retries << ")...");
    
    server_ready = gripper_client_->wait_for_action_server(std::chrono::seconds(5));
    
    if (!server_ready) {
      retry_count++;
      if (retry_count < max_retries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
  
  if (!server_ready) {
    LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER, "gripper action server timeout after " << max_retries << " attempts");
    return ActionCallResult::FAIL_SERVER_TIMEOUT;
  }
  
  LOG_INFO(node_->get_logger(), task_log::Tag::GRIPPER, "Gripper action server connected successfully");



  // 5. 配置异步回调选项 (SendGoalOptions)
  auto send_goal_options = rclcpp_action::Client<CtrlGripper>::SendGoalOptions();
  
  // 目标响应回调：使用 lambda 捕获 state_key，更新正确的 context
  send_goal_options.goal_response_callback =
    [this, state_key](GoalHandleCtrlGripper::SharedPtr goal_handle)
    {
      auto goal = goal_handle.get();
      if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER, "Goal was rejected by server [" << state_key << "]");
        finish_action_timing(state_key, "GOAL_REJECTED");
        complete_action_context(
          state_key, ActionRunState::FAILURE,
          ActionCallResult::FAIL_GOAL_REJECTED);
      } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::GRIPPER, "Goal accepted by server [" << state_key << "], waiting for result...");
      }
    };
    
  // 反馈回调：服务器周期性发送进度时触发
  send_goal_options.feedback_callback =
    std::bind(&TaskActionClient::gripper_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

  // 结果回调：使用 lambda 捕获 state_key，更新正确的 context
  send_goal_options.result_callback =
    [this, state_key](const GoalHandleCtrlGripper::WrappedResult & result)
    {
      const bool succeeded = result.result && result.result->success;
      finish_action_timing(
        state_key, succeeded ? "SUCCESS" : "FAILURE",
        false, 0.0, 0.0, 0.0, true);
      if (!result.result)
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER,
          "Gripper control returned an empty result [" << state_key << "]");
        complete_action_context(
          state_key, ActionRunState::FAILURE,
          ActionCallResult::FAIL_GENERAL);
      }
      else if (result.result->success &&
        (result.result->final_position.size() < 2U ||
        result.result->final_torque.size() < 2U))
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER,
          "Gripper control returned incomplete final arrays [" << state_key
          << "]: position_size=" << result.result->final_position.size()
          << ", torque_size=" << result.result->final_torque.size());
        complete_action_context(
          state_key, ActionRunState::FAILURE,
          ActionCallResult::FAIL_GENERAL);
      }
      else if(result.result->success)
      {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::GRIPPER, "Gripper control succeeded [" << state_key << "]: final_position=["
          << (int)result.result->final_position[0] << "," << (int)result.result->final_position[1]
          << "], final_torque=[" << (int)result.result->final_torque[0] << "," << (int)result.result->final_torque[1] << "]");
        complete_action_context(
          state_key, ActionRunState::SUCCESS,
          ActionCallResult::SUCCESS);
      }
      else
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER, "Gripper control failed [" << state_key << "]: " << result.result->error_message);
        complete_action_context(
          state_key, ActionRunState::FAILURE,
          ActionCallResult::FAIL_GENERAL);
      }
    };
  gripper_send_future_ = gripper_client_->async_send_goal(goal, send_goal_options);
  return ActionCallResult::SUCCESS;
}

rclcpp::Time TaskActionClient::get_ros_now_time() const
{
  return node_->now();
}


rclcpp::Time TaskActionClient::get_action_start_time(const std::string& state_key) const
{
  return get_ctx(state_key).start_time;
}

uint32_t TaskActionClient::get_action_execution_time(const std::string& state_key) const
{
  return get_ctx(state_key).execution_time_ms;
}

ActionCallResult TaskActionClient::get_last_call_result(const std::string& state_key) const
{
  return get_ctx(state_key).call_result;
}


void TaskActionClient::gripper_goal_response_callback(GoalHandleCtrlGripper::SharedPtr goal_handle)
{
    auto goal = goal_handle.get();
    if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER, "Goal was rejected by server");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
    } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::GRIPPER, "Goal accepted by server, waiting for result...");
    }
}

void TaskActionClient::gripper_feedback_callback(GoalHandleCtrlGripper::SharedPtr, const std::shared_ptr<const GoalHandleCtrlGripper::Feedback> feedback)
{
  LOG_DEBUG(node_->get_logger(), task_log::Tag::GRIPPER, "Feedback: progress=" << (feedback ? std::to_string(feedback->progress_rate) : "N/A"));
}

void TaskActionClient::gripper_result_callback(const GoalHandleCtrlGripper::WrappedResult & result)
{
  //StopTimer();
  if(result.result->success)
  {
    LOG_SUCCESS(node_->get_logger(), task_log::Tag::GRIPPER, "Gripper control succeeded: final_position=["
      << (int)result.result->final_position[0] << "," << (int)result.result->final_position[1]
      << "], final_torque=[" << (int)result.result->final_torque[0] << "," << (int)result.result->final_torque[1] << "]");
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::SUCCESS;
    ctx.call_result = ActionCallResult::SUCCESS;
    
  }
  else
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER, "Gripper control failed: " << result.result->error_message);
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::FAILURE;
    ctx.call_result = ActionCallResult::FAIL_GENERAL;
  }

  // 释放该 action 占用的资源（遍历所有 context）
  {
    std::lock_guard<std::mutex> lock(ctx_mutex_);
    for (auto& [key, ctx] : action_contexts_)
    {
      if (!ctx.acquired_resources.empty())
      {
        for (const auto& res : ctx.acquired_resources)
        {
          release_resource(res);
        }
        ctx.acquired_resources.clear();
      }
    }
  }
}

// ============================================================================
// send_goal（串行版本，使用 "default" key）
// ============================================================================

ActionCallResult TaskActionClient::send_goal(const task_cfg::AtomicAction& act)
{
    const std::string state_key = "default";
    return send_goal(act, state_key);
}

// ============================================================================
// dispatch_action：动作类型分发执行（send_goal 共享逻辑）
// ============================================================================

ActionCallResult TaskActionClient::dispatch_action(
    const task_cfg::AtomicAction& act,
    const std::string& task_id,
    ActionContext& ctx,
    const std::string& state_key)
{
    const std::string& act_type = act.action_type;
    const std::string log_prefix = (state_key == "default") ? "" : "[Branch " + state_key + "] ";

    if (act_type == "vision_detect")
    {
        ctx.state = ActionRunState::RUNNING;
        ctx.execution_time_ms = 60000;
        ActionCallResult res = call_vision_detect(task_id, act.vision_param, state_key);
        if (action_call_result_is_ok(res))
        {
            LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal vision_detect: " << action_call_result_to_str(res));
            ctx.state = ActionRunState::RUNNING;
            return ActionCallResult::SUCCESS;
        }
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = res;
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal vision_detect failed: " << action_call_result_to_str(res));
        return res;
    }
    else if (act_type == "move_pose" || act_type == "move_arm")
    {
        ctx.state = ActionRunState::RUNNING;
        ctx.execution_time_ms = static_cast<uint32_t>(act.arm_param.timeout_ms > 0 ?
            act.arm_param.timeout_ms : 60000);

        // 尝试从序列级 key 获取同序列 vision_detect 的结果
        geometry_msgs::msg::PoseStamped target_pose = pose;
        sensor_msgs::msg::PointCloud2 env_cloud = cloud;
        std::string cloud_frame = "base_link";

        std::string seq_key = derive_seq_key(state_key);
        if (!seq_key.empty())
        {
            VisionDet::Result vis_res;
            if (get_vision_result_from_seq_key(seq_key, vis_res))
            {
                LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, log_prefix
                    << "Using seq-level vision result from [" << seq_key << "] for move_pose");
                target_pose = vis_res.target_6d_pose;
                cloud_frame = vis_res.point_cloud_frame_id;
                if (!vis_res.env_point_cloud.data.empty())
                {
                    env_cloud = vis_res.env_point_cloud;
                }
            }
            else
            {
                LOG_WARN(node_->get_logger(), task_log::Tag::ACTION, log_prefix
                    << "No seq-level vision result at [" << seq_key << "] for move_pose, using config_pose if set");
            }
        }

        ActionCallResult res = call_move_pose(task_id, act.arm_param, target_pose, env_cloud, cloud_frame, state_key);
        if (action_call_result_is_ok(res))
        {
            LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal move_pose: " << action_call_result_to_str(res));
            ctx.state = ActionRunState::RUNNING;
            return ActionCallResult::SUCCESS;
        }
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = res;
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal move_pose failed: " << action_call_result_to_str(res));
        return res;
    }
    else if (act_type == "move_joints")
    {
        ctx.state = ActionRunState::RUNNING;
        ctx.execution_time_ms = static_cast<uint32_t>(act.move_joints_param.timeout_ms > 0 ?
            act.move_joints_param.timeout_ms : 60000);

        // move_joints 不使用视觉结果注入（关节目标由 YAML 配置直接指定）
        ActionCallResult res = call_move_joints(task_id, act.move_joints_param, state_key);
        if (action_call_result_is_ok(res))
        {
            LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal move_joints: " << action_call_result_to_str(res));
            ctx.state = ActionRunState::RUNNING;
            return ActionCallResult::SUCCESS;
        }
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = res;
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal move_joints failed: " << action_call_result_to_str(res));
        return res;
    }
    else if (act_type == "control_suck")
    {
        ctx.state = ActionRunState::RUNNING;
        ctx.execution_time_ms = static_cast<uint32_t>(act.suck_param.timeout_ms > 0 ?
            act.suck_param.timeout_ms : 30000);
        ActionCallResult res = call_control_suck(task_id, act.suck_param, state_key);
        if (action_call_result_is_ok(res))
        {
            LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal control_suck: " << action_call_result_to_str(res));
            ctx.state = ActionRunState::RUNNING;
            return ActionCallResult::SUCCESS;
        }
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = res;
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal control_suck failed: " << action_call_result_to_str(res));
        return res;
    }
    else if (act_type == "gripper_ctrl")
    {
        ctx.state = ActionRunState::RUNNING;
        ctx.execution_time_ms = act.gripper_param.timeout_ms > 0 ?
            static_cast<uint32_t>(act.gripper_param.timeout_ms) : 30000;
        LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal gripper_ctrl: position_size=" << act.gripper_param.position.size());
        ActionCallResult res = call_gripper(task_id, act.gripper_param, state_key);
        if (action_call_result_is_ok(res))
        {
            LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal gripper_ctrl: " << action_call_result_to_str(res));
            ctx.state = ActionRunState::RUNNING;
            return ActionCallResult::SUCCESS;
        }
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = res;
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal gripper_ctrl failed: " << action_call_result_to_str(res));
        return res;
    }
    else if (act_type == "vision_check_self")
    {
        ctx.state = ActionRunState::RUNNING;
        ctx.execution_time_ms = act.vision_check_self_param.timeout_ms > 0 ?
            static_cast<uint32_t>(act.vision_check_self_param.timeout_ms) : 60000;
        ActionCallResult res = call_vision_check_self(task_id, act.vision_check_self_param, state_key);
        if (action_call_result_is_ok(res))
        {
            LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal vision_check_self: " << action_call_result_to_str(res));
            ctx.state = ActionRunState::RUNNING;
            return ActionCallResult::SUCCESS;
        }
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = res;
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal vision_check_self failed: " << action_call_result_to_str(res));
        return res;
    }
    else if (act_type == "control_check_self")
    {
        ctx.state = ActionRunState::RUNNING;
        ctx.execution_time_ms = act.control_check_self_param.timeout_ms > 0 ?
            static_cast<uint32_t>(act.control_check_self_param.timeout_ms) : 60000;
        ActionCallResult res = call_control_check_self(task_id, act.control_check_self_param, state_key);
        if (action_call_result_is_ok(res))
        {
            LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal control_check_self: " << action_call_result_to_str(res));
            ctx.state = ActionRunState::RUNNING;
            return ActionCallResult::SUCCESS;
        }
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = res;
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal control_check_self failed: " << action_call_result_to_str(res));
        return res;
    }
    else if (act_type == "vision_reset")
    {
        ctx.state = ActionRunState::RUNNING;
        ctx.execution_time_ms = act.vision_reset_param.timeout_ms > 0 ?
            static_cast<uint32_t>(act.vision_reset_param.timeout_ms) : 60000;
        ActionCallResult res = call_vision_reset(task_id, act.vision_reset_param, state_key);
        if (action_call_result_is_ok(res))
        {
            LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal vision_reset: " << action_call_result_to_str(res));
            ctx.state = ActionRunState::RUNNING;
            return ActionCallResult::SUCCESS;
        }
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = res;
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal vision_reset failed: " << action_call_result_to_str(res));
        return res;
    }
    else if (act_type == "control_reset")
    {
        ctx.state = ActionRunState::RUNNING;
        ctx.execution_time_ms = act.control_reset_param.timeout_ms > 0 ?
            static_cast<uint32_t>(act.control_reset_param.timeout_ms) : 60000;
        ActionCallResult res = call_control_reset(task_id, act.control_reset_param, state_key);
        if (action_call_result_is_ok(res))
        {
            LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal control_reset: " << action_call_result_to_str(res));
            ctx.state = ActionRunState::RUNNING;
            return ActionCallResult::SUCCESS;
        }
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = res;
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal control_reset failed: " << action_call_result_to_str(res));
        return res;
    }
    else
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "Unknown action type: " << act_type);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_UNKNOWN_TYPE;
        return ActionCallResult::FAIL_UNKNOWN_TYPE;
    }
}

// ============================================================================
// 并行分支专用：带 state_key 的重载函数
// ============================================================================

bool TaskActionClient::is_action_running(const std::string& state_key) const
{
    return get_ctx(state_key).state == ActionRunState::RUNNING;
}

bool TaskActionClient::is_action_success(const std::string& state_key) const
{
    return get_ctx(state_key).state == ActionRunState::SUCCESS;
}

bool TaskActionClient::is_action_failure(const std::string& state_key) const
{
    return get_ctx(state_key).state == ActionRunState::FAILURE;
}

void TaskActionClient::reset_action_state(const std::string& state_key)
{
    auto& ctx = get_ctx(state_key);
    ctx.state = ActionRunState::IDLE;
    ctx.call_result = ActionCallResult::SUCCESS;
    ctx.start_time = rclcpp::Time(0, 0);
    // 清除之前缓存的 vision 检测结果
    ctx.has_vision_result = false;
    ctx.vision_result = VisionDet::Result();
    // 释放该 context 占用的资源，防止资源锁泄漏导致下次任务无法获取资源
    release_acquired_resources(ctx);
}

void TaskActionClient::reset_all_states()
{
    std::lock_guard<std::mutex> lock(ctx_mutex_);
    for (auto& [key, ctx] : action_contexts_)
    {
        ctx.state = ActionRunState::IDLE;
        ctx.call_result = ActionCallResult::SUCCESS;
        ctx.start_time = rclcpp::Time(0, 0);
        // 清除之前缓存的 vision 检测结果
        ctx.has_vision_result = false;
        ctx.vision_result = VisionDet::Result();
        // 释放该 context 占用的所有资源
        for (const auto& res : ctx.acquired_resources)
        {
            release_resource(res);
        }
        ctx.acquired_resources.clear();
    }
}

// ============================================================================
// vision_detect 结果查询接口实现
// ============================================================================

bool TaskActionClient::get_vision_result(const std::string& state_key, VisionDet::Result& out_result) const
{
    const auto& ctx = get_ctx(state_key);
    if (!ctx.has_vision_result)
    {
        return false;
    }
    out_result = ctx.vision_result;
    return true;
}

bool TaskActionClient::has_vision_result(const std::string& state_key) const
{
    return get_ctx(state_key).has_vision_result;
}

const ActionContext& TaskActionClient::get_context(const std::string& state_key) const
{
    return get_ctx(state_key);
}

// ============================================================================
// 序列级视觉结果存储实现（跨 action 共享）
// ============================================================================

void TaskActionClient::store_vision_result_at_seq_key(const std::string& seq_key, const VisionDet::Result& result)
{
    if (seq_key.empty())
    {
        return;
    }
    std::lock_guard<std::mutex> lock(seq_vision_mutex_);
    seq_vision_results_[seq_key] = result;
    LOG_INFO(node_->get_logger(), task_log::Tag::VISION,
        "Stored vision result at seq_key [" << seq_key
        << "], target=" << result.target_6d_pose.header.frame_id);
}

bool TaskActionClient::get_vision_result_from_seq_key(const std::string& seq_key, VisionDet::Result& out_result) const
{
    if (seq_key.empty())
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(seq_vision_mutex_);
    auto it = seq_vision_results_.find(seq_key);
    if (it == seq_vision_results_.end())
    {
        return false;
    }
    out_result = it->second;
    return true;
}

bool TaskActionClient::has_vision_result_at_seq_key(const std::string& seq_key) const
{
    if (seq_key.empty())
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(seq_vision_mutex_);
    return seq_vision_results_.find(seq_key) != seq_vision_results_.end();
}

void TaskActionClient::clear_all_seq_vision_results()
{
    std::lock_guard<std::mutex> lock(seq_vision_mutex_);
    seq_vision_results_.clear();
    LOG_DEBUG(node_->get_logger(), task_log::Tag::VISION, "Cleared all seq-level vision results");
}

ActionCallResult TaskActionClient::send_goal(const task_cfg::AtomicAction& act, const std::string& state_key)
{
    // 先重置该分支状态
    auto& ctx = get_ctx(state_key);
    ctx.state = ActionRunState::IDLE;
    ctx.call_result = ActionCallResult::SUCCESS;

    const std::string& act_type = act.action_type;

    // 生成唯一任务 ID
    uuid_t uuid;
    uuid_generate(uuid);
    char buf[40];
    uuid_unparse(uuid, buf);
    std::string current_task_id_ = std::string(buf);

    // ========== 资源锁检查：尝试获取所需资源 ==========
    auto required_res = get_required_resources(act_type, act);
    std::vector<ResourceType> acquired_res;
    for (const auto& res : required_res)
    {
        if (!acquire_resource(res))
        {
            for (const auto& acquired : acquired_res)
            {
                release_resource(acquired);
            }
            LOG_WARN(node_->get_logger(), task_log::Tag::ACTION,
                "[Branch " << state_key << "] send_goal " << act_type
                << " failed: resource " << resource_type_to_str(res) << " is busy, will retry");
            ctx.state = ActionRunState::IDLE;
            ctx.call_result = ActionCallResult::FAIL_RESOURCE_BUSY;
            return ActionCallResult::FAIL_RESOURCE_BUSY;
        }
        acquired_res.push_back(res);
    }
    ctx.acquired_resources = std::move(acquired_res);
    ctx.start_time = node_->now();
    start_action_timing(state_key, act_type);

    // ========== 动作类型分发 ==========
    ActionCallResult ret = dispatch_action(act, current_task_id_, ctx, state_key);

    // 如果发送失败（非资源锁原因），释放已获取的资源
    if (!action_call_result_is_ok(ret))
    {
        finish_action_timing(
            state_key, action_call_result_to_str(ret));
        release_acquired_resources(ctx);
    }

    return ret;
}

// ============================================================================
// 资源锁实现
// ============================================================================

void TaskActionClient::release_acquired_resources(ActionContext& ctx)
{
    for (const auto& res : ctx.acquired_resources)
    {
        release_resource(res);
    }
    ctx.acquired_resources.clear();
}

void TaskActionClient::complete_action_context(
    const std::string& state_key,
    ActionRunState state,
    ActionCallResult call_result)
{
    const auto resources = detail::complete_context(
      action_contexts_, ctx_mutex_, state_key, state, call_result);
    for (const auto& resource : resources)
    {
        release_resource(resource);
    }
}

bool TaskActionClient::acquire_resource(ResourceType res)
{
    size_t idx = static_cast<size_t>(res);
    if (idx >= static_cast<size_t>(ResourceType::COUNT))
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, "acquire_resource: invalid resource index " << idx);
        return false;
    }
    // atomic 的 test-and-set：期望 false（未占用），设置为 true（占用）
    bool expected = false;
    bool acquired = resource_locks_[idx].compare_exchange_strong(expected, true, std::memory_order_acq_rel);
    if (acquired)
    {
        LOG_DEBUG(node_->get_logger(), task_log::Tag::ACTION, "Resource " << resource_type_to_str(res) << " acquired");
    }
    else
    {
        LOG_DEBUG(node_->get_logger(), task_log::Tag::ACTION, "Resource " << resource_type_to_str(res) << " is busy");
    }
    return acquired;
}

void TaskActionClient::release_resource(ResourceType res)
{
    size_t idx = static_cast<size_t>(res);
    if (idx >= static_cast<size_t>(ResourceType::COUNT))
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, "release_resource: invalid resource index " << idx);
        return;
    }
    resource_locks_[idx].store(false, std::memory_order_release);
    LOG_DEBUG(node_->get_logger(), task_log::Tag::ACTION, "Resource " << resource_type_to_str(res) << " released");
}

bool TaskActionClient::is_resource_busy(ResourceType res) const
{
    size_t idx = static_cast<size_t>(res);
    if (idx >= static_cast<size_t>(ResourceType::COUNT))
    {
        return true; // 无效资源视为 busy
    }
    return resource_locks_[idx].load(std::memory_order_acquire);
}

std::vector<ResourceType> TaskActionClient::get_required_resources(
    const std::string& act_type, const task_cfg::AtomicAction& act) const
{
    std::vector<ResourceType> resources;
    if (act_type == "vision_detect" || act_type == "vision_check_self" || act_type == "vision_reset")
    {
        resources.push_back(ResourceType::VISION);
    }
    else if (act_type == "control_check_self" || act_type == "control_reset")
    {
        // 运控自检和复位占用左右臂资源
        resources.push_back(ResourceType::LEFT_ARM);
        resources.push_back(ResourceType::RIGHT_ARM);
    }
    else if (act_type == "move_pose" || act_type == "move_arm")
    {
        // arm_id 使用 com_interfaces 命名常量: ARM_LEFT=0, ARM_RIGHT=1, ARM_BOTH=2
        const auto& aid = act.arm_param.arm_id;
        if (aid == MovePose::Goal::ARM_LEFT || aid == MovePose::Goal::ARM_BOTH)
        {
            resources.push_back(ResourceType::LEFT_ARM);
        }
        if (aid == MovePose::Goal::ARM_RIGHT || aid == MovePose::Goal::ARM_BOTH)
        {
            resources.push_back(ResourceType::RIGHT_ARM);
        }
        if (aid == MovePose::Goal::HEAD)
        {
            resources.push_back(ResourceType::HEAD);
        }
        if (aid == MovePose::Goal::LEG)
        {
            resources.push_back(ResourceType::LOWER_BODY);
        }
    }
    else if (act_type == "move_joints")
    {
        const auto& aid = act.move_joints_param.arm_id;
        if (aid == MoveJoints::Goal::ARM_LEFT || aid == MoveJoints::Goal::ARM_BOTH)
        {
            resources.push_back(ResourceType::LEFT_ARM);
        }
        if (aid == MoveJoints::Goal::ARM_RIGHT || aid == MoveJoints::Goal::ARM_BOTH)
        {
            resources.push_back(ResourceType::RIGHT_ARM);
        }
        if (aid == MoveJoints::Goal::HEAD)
        {
            resources.push_back(ResourceType::HEAD);
        }
        if (aid == MoveJoints::Goal::LEG)
        {
            resources.push_back(ResourceType::LOWER_BODY);
        }
    }
    else if (act_type == "gripper_ctrl")
    {
        if (act.gripper_param.gripper_id == 0)
        {
            resources.push_back(ResourceType::LEFT_GRIPPER);
        }
        else if (act.gripper_param.gripper_id == 1)
        {
            resources.push_back(ResourceType::RIGHT_GRIPPER);
        }
    }
    // control_suck 不占用互斥资源（可与其他 action 并行）
    return resources;
}

// ============================================================================
// call_vision_check_self - 调用视觉自检（/vision/Checkself）
// ============================================================================
ActionCallResult TaskActionClient::call_vision_check_self(
    const std::string& task_id, const task_cfg::VisionCheckSelfParam& param,
    const std::string& state_key)
{
    LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::VISION, "===== Vision CheckSelf Start =====");

    if (task_id.empty())
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "task_id is empty");
        return ActionCallResult::FAIL_PARAM_INVALID;
    }

    // 等待 server 就绪
    if (!hw_check_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision CheckSelf action server not available");
        return ActionCallResult::FAIL_SERVER_TIMEOUT;
    }

    auto goal = HardWareCheck::Goal();
    goal.header.stamp = node_->now();
    goal.task_id = task_id;
    goal.timeout_ms = param.timeout_ms > 0 ? param.timeout_ms : 15000;
    goal.priority = 1;

    LOG_INFO(node_->get_logger(), task_log::Tag::VISION,
        "VisionCheckSelf Goal: task_id=" << goal.task_id
        << ", timeout_ms=" << goal.timeout_ms << ", priority=" << (int)goal.priority);

    auto send_goal_options = rclcpp_action::Client<HardWareCheck>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this, state_key](GoalHandleHardWareCheck::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision CheckSelf goal rejected [" << state_key << "]");
                finish_action_timing(state_key, "GOAL_REJECTED");
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::FAILURE;
                ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
            }
            else
            {
                LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "Vision CheckSelf goal accepted [" << state_key << "]");
            }
        };

    send_goal_options.feedback_callback =
        std::bind(&TaskActionClient::hw_check_feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback =
        [this, state_key](const GoalHandleHardWareCheck::WrappedResult& result)
        {
            finish_action_timing(
                state_key,
                result.result && result.result->success ? "SUCCESS" : "FAILURE");
            if (result.result->success)
            {
                LOG_SUCCESS(node_->get_logger(), task_log::Tag::VISION,
                    "Vision CheckSelf succeeded [" << state_key << "], execution_time=" << result.result->execution_time << "s");
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::SUCCESS;
                ctx.call_result = ActionCallResult::SUCCESS;
            }
            else
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::VISION,
                    "Vision CheckSelf failed [" << state_key << "]: " << result.result->error_message);
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::FAILURE;
                ctx.call_result = ActionCallResult::FAIL_GENERAL;
            }

            // 释放资源
            {
                std::lock_guard<std::mutex> lock(ctx_mutex_);
                for (auto& [key, ctx] : action_contexts_)
                {
                    if (!ctx.acquired_resources.empty())
                    {
                        for (const auto& res : ctx.acquired_resources)
                        {
                            release_resource(res);
                        }
                        ctx.acquired_resources.clear();
                    }
                }
            }
        };

    hw_check_client_->async_send_goal(goal, send_goal_options);
    LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::VISION, "===== Vision CheckSelf End =====");
    return ActionCallResult::SUCCESS;
}

// ============================================================================
// call_control_check_self - 调用运控自检（/Control/Checkself）
// ============================================================================
ActionCallResult TaskActionClient::call_control_check_self(
    const std::string& task_id, const task_cfg::ControlCheckSelfParam& param,
    const std::string& state_key)
{
    LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::ARM, "===== Control CheckSelf Start =====");

    if (task_id.empty())
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "task_id is empty");
        return ActionCallResult::FAIL_PARAM_INVALID;
    }

    // 使用成员变量 control_check_client_（已在构造函数中初始化）
    if (!control_check_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Control CheckSelf action server not available");
        return ActionCallResult::FAIL_SERVER_TIMEOUT;
    }

    auto goal = HardWareCheck::Goal();
    goal.header.stamp = node_->now();
    goal.task_id = task_id;
    goal.timeout_ms = param.timeout_ms > 0 ? param.timeout_ms : 15000;
    goal.priority = 1;
    // 设置 com_port 参数（如果配置了）
    if (param.com_port.size() >= 2)
    {
        goal.com_port[0] = param.com_port[0];
        goal.com_port[1] = param.com_port[1];
    }

    LOG_INFO(node_->get_logger(), task_log::Tag::ARM,
        "ControlCheckSelf Goal: task_id=" << goal.task_id
        << ", timeout_ms=" << goal.timeout_ms << ", priority=" << (int)goal.priority
        << ", com_port=[" << (int)goal.com_port[0] << "," << (int)goal.com_port[1] << "]");

    auto send_goal_options = rclcpp_action::Client<HardWareCheck>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this, state_key](GoalHandleHardWareCheck::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Control CheckSelf goal rejected [" << state_key << "]");
                finish_action_timing(state_key, "GOAL_REJECTED");
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::FAILURE;
                ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
            }
            else
            {
                LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "Control CheckSelf goal accepted [" << state_key << "]");
            }
        };

    send_goal_options.feedback_callback =
        [this](GoalHandleHardWareCheck::SharedPtr, const std::shared_ptr<const HardWareCheck::Feedback> feedback)
        {
            if (feedback)
            {
                LOG_DEBUG(node_->get_logger(), task_log::Tag::ARM,
                    "Control CheckSelf feedback: " << feedback->progress_percent << "% - " << feedback->current_step);
            }
        };

    send_goal_options.result_callback =
        [this, state_key](const GoalHandleHardWareCheck::WrappedResult& result)
        {
            finish_action_timing(
                state_key,
                result.result && result.result->success ? "SUCCESS" : "FAILURE");
            if (result.result->success)
            {
                LOG_SUCCESS(node_->get_logger(), task_log::Tag::ARM,
                    "Control CheckSelf succeeded [" << state_key << "], execution_time=" << result.result->execution_time << "s");
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::SUCCESS;
                ctx.call_result = ActionCallResult::SUCCESS;
            }
            else
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::ARM,
                    "Control CheckSelf failed [" << state_key << "]: " << result.result->error_message);
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::FAILURE;
                ctx.call_result = ActionCallResult::FAIL_GENERAL;
            }

            // 释放资源
            {
                std::lock_guard<std::mutex> lock(ctx_mutex_);
                for (auto& [key, ctx] : action_contexts_)
                {
                    if (!ctx.acquired_resources.empty())
                    {
                        for (const auto& res : ctx.acquired_resources)
                        {
                            release_resource(res);
                        }
                        ctx.acquired_resources.clear();
                    }
                }
            }
        };

    control_check_client_->async_send_goal(goal, send_goal_options);
    LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::ARM, "===== Control CheckSelf End =====");
    return ActionCallResult::SUCCESS;
}

// ============================================================================
// call_vision_reset - 调用视觉复位（/vision/Reset）
// ============================================================================
ActionCallResult TaskActionClient::call_vision_reset(
    const std::string& task_id, const task_cfg::VisionResetParam& param,
    const std::string& state_key)
{
    LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::VISION, "===== Vision Reset Start =====");

    if (task_id.empty())
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "task_id is empty");
        return ActionCallResult::FAIL_PARAM_INVALID;
    }

    if (!hw_reset_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision Reset action server not available");
        return ActionCallResult::FAIL_SERVER_TIMEOUT;
    }

    auto goal = HardWareReset::Goal();
    goal.header.stamp = node_->now();
    goal.task_id = task_id;
    goal.timeout_ms = param.timeout_ms > 0 ? param.timeout_ms : 15000;
    goal.priority = 1;

    LOG_INFO(node_->get_logger(), task_log::Tag::VISION,
        "VisionReset Goal: task_id=" << goal.task_id
        << ", timeout_ms=" << goal.timeout_ms << ", priority=" << (int)goal.priority);

    auto send_goal_options = rclcpp_action::Client<HardWareReset>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this, state_key](GoalHandleHardWareReset::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision Reset goal rejected [" << state_key << "]");
                finish_action_timing(state_key, "GOAL_REJECTED");
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::FAILURE;
                ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
            }
            else
            {
                LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "Vision Reset goal accepted [" << state_key << "]");
            }
        };

    send_goal_options.feedback_callback =
        std::bind(&TaskActionClient::hw_reset_feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback =
        [this, state_key](const GoalHandleHardWareReset::WrappedResult& result)
        {
            finish_action_timing(
                state_key,
                result.result && result.result->success ? "SUCCESS" : "FAILURE");
            if (result.result->success)
            {
                LOG_SUCCESS(node_->get_logger(), task_log::Tag::VISION,
                    "Vision Reset succeeded [" << state_key << "], duration=" << result.result->duration_sec << "s");
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::SUCCESS;
                ctx.call_result = ActionCallResult::SUCCESS;
            }
            else
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::VISION,
                    "Vision Reset failed [" << state_key << "]: " << result.result->error_message);
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::FAILURE;
                ctx.call_result = ActionCallResult::FAIL_GENERAL;
            }

            // 释放资源
            {
                std::lock_guard<std::mutex> lock(ctx_mutex_);
                for (auto& [key, ctx] : action_contexts_)
                {
                    if (!ctx.acquired_resources.empty())
                    {
                        for (const auto& res : ctx.acquired_resources)
                        {
                            release_resource(res);
                        }
                        ctx.acquired_resources.clear();
                    }
                }
            }
        };

    hw_reset_client_->async_send_goal(goal, send_goal_options);
    LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::VISION, "===== Vision Reset End =====");
    return ActionCallResult::SUCCESS;
}

// ============================================================================
// call_control_reset - 调用运控复位（/Control/Reset）
// ============================================================================
ActionCallResult TaskActionClient::call_control_reset(
    const std::string& task_id, const task_cfg::ControlResetParam& param,
    const std::string& state_key)
{
    LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::ARM, "===== Control Reset Start =====");

    if (task_id.empty())
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "task_id is empty");
        return ActionCallResult::FAIL_PARAM_INVALID;
    }

    // 使用成员变量 control_reset_client_（已在构造函数中初始化）
    if (!control_reset_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Control Reset action server not available");
        return ActionCallResult::FAIL_SERVER_TIMEOUT;
    }

    auto goal = HardWareReset::Goal();
    goal.header.stamp = node_->now();
    goal.task_id = task_id;
    goal.timeout_ms = param.timeout_ms > 0 ? param.timeout_ms : 15000;
    goal.priority = 1;

    LOG_INFO(node_->get_logger(), task_log::Tag::ARM,
        "ControlReset Goal: task_id=" << goal.task_id
        << ", timeout_ms=" << goal.timeout_ms << ", priority=" << (int)goal.priority);

    auto send_goal_options = rclcpp_action::Client<HardWareReset>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this, state_key](GoalHandleHardWareReset::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Control Reset goal rejected [" << state_key << "]");
                finish_action_timing(state_key, "GOAL_REJECTED");
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::FAILURE;
                ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
            }
            else
            {
                LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "Control Reset goal accepted [" << state_key << "]");
            }
        };

    send_goal_options.feedback_callback =
        [this](GoalHandleHardWareReset::SharedPtr, const std::shared_ptr<const HardWareReset::Feedback> feedback)
        {
            if (feedback)
            {
                LOG_DEBUG(node_->get_logger(), task_log::Tag::ARM,
                    "Control Reset feedback: " << feedback->progress_percent << "% - " << feedback->current_step);
            }
        };

    send_goal_options.result_callback =
        [this, state_key](const GoalHandleHardWareReset::WrappedResult& result)
        {
            finish_action_timing(
                state_key,
                result.result && result.result->success ? "SUCCESS" : "FAILURE");
            if (result.result->success)
            {
                LOG_SUCCESS(node_->get_logger(), task_log::Tag::ARM,
                    "Control Reset succeeded [" << state_key << "], duration=" << result.result->duration_sec << "s");
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::SUCCESS;
                ctx.call_result = ActionCallResult::SUCCESS;
            }
            else
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::ARM,
                    "Control Reset failed [" << state_key << "]: " << result.result->error_message);
                auto& ctx = get_ctx(state_key);
                ctx.state = ActionRunState::FAILURE;
                ctx.call_result = ActionCallResult::FAIL_GENERAL;
            }

            // 释放资源
            {
                std::lock_guard<std::mutex> lock(ctx_mutex_);
                for (auto& [key, ctx] : action_contexts_)
                {
                    if (!ctx.acquired_resources.empty())
                    {
                        for (const auto& res : ctx.acquired_resources)
                        {
                            release_resource(res);
                        }
                        ctx.acquired_resources.clear();
                    }
                }
            }
        };

    control_reset_client_->async_send_goal(goal, send_goal_options);
    LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::ARM, "===== Control Reset End =====");
    return ActionCallResult::SUCCESS;
}

// ============================================================================
// HardwareCheck 回调函数
// ============================================================================
void TaskActionClient::hw_check_goal_response_callback(GoalHandleHardWareCheck::SharedPtr goal_handle)
{
    if (!goal_handle)
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "HardwareCheck goal was rejected by server");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
    }
    else
    {
        LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "HardwareCheck goal accepted by server, waiting for result...");
    }
}

void TaskActionClient::hw_check_feedback_callback(
    GoalHandleHardWareCheck::SharedPtr,
    const std::shared_ptr<const HardWareCheck::Feedback> feedback)
{
    if (feedback)
    {
        LOG_DEBUG(node_->get_logger(), task_log::Tag::VISION,
            "HardwareCheck feedback: " << feedback->progress_percent << "% - " << feedback->current_step);
    }
}

void TaskActionClient::hw_check_result_callback(const GoalHandleHardWareCheck::WrappedResult& result)
{
    if (result.result->success)
    {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::VISION,
            "HardwareCheck succeeded, execution_time=" << result.result->execution_time << "s");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::SUCCESS;
        ctx.call_result = ActionCallResult::SUCCESS;
    }
    else
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION,
            "HardwareCheck failed: " << result.result->error_message);
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GENERAL;
    }

    // 释放资源
    {
        std::lock_guard<std::mutex> lock(ctx_mutex_);
        for (auto& [key, ctx] : action_contexts_)
        {
            if (!ctx.acquired_resources.empty())
            {
                for (const auto& res : ctx.acquired_resources)
                {
                    release_resource(res);
                }
                ctx.acquired_resources.clear();
            }
        }
    }
}

// ============================================================================
// HardwareReset 回调函数
// ============================================================================
void TaskActionClient::hw_reset_goal_response_callback(GoalHandleHardWareReset::SharedPtr goal_handle)
{
    if (!goal_handle)
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "HardwareReset goal was rejected by server");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
    }
    else
    {
        LOG_INFO(node_->get_logger(), task_log::Tag::VISION, "HardwareReset goal accepted by server, waiting for result...");
    }
}

void TaskActionClient::hw_reset_feedback_callback(
    GoalHandleHardWareReset::SharedPtr,
    const std::shared_ptr<const HardWareReset::Feedback> feedback)
{
    if (feedback)
    {
        LOG_DEBUG(node_->get_logger(), task_log::Tag::VISION,
            "HardwareReset feedback: " << feedback->progress_percent << "% - " << feedback->current_step);
    }
}

void TaskActionClient::hw_reset_result_callback(const GoalHandleHardWareReset::WrappedResult& result)
{
    if (result.result->success)
    {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::VISION,
            "HardwareReset succeeded, duration=" << result.result->duration_sec << "s");
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::SUCCESS;
        ctx.call_result = ActionCallResult::SUCCESS;
    }
    else
    {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION,
            "HardwareReset failed: " << result.result->error_message);
        auto& ctx = get_ctx("default");
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GENERAL;
    }

    // 释放资源
    {
        std::lock_guard<std::mutex> lock(ctx_mutex_);
        for (auto& [key, ctx] : action_contexts_)
        {
            if (!ctx.acquired_resources.empty())
            {
                for (const auto& res : ctx.acquired_resources)
                {
                    release_resource(res);
                }
                ctx.acquired_resources.clear();
            }
        }
    }
}

} // namespace task_act
