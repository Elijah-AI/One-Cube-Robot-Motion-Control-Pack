#include "task_manager/task_action_client.hpp"
#include "task_manager/log_helper.hpp"
#include <random>
#include <sstream>
#include <uuid/uuid.h>
#include <cmath>
#include <iomanip>

// 使用 com_interfaces 中的 Action 定义
#include "com_interfaces/action/vision_detection.hpp"
#include "com_interfaces/action/control_gripper.hpp"
#include "com_interfaces/action/move_arm.hpp"

namespace task_act
{
TaskActionClient::TaskActionClient(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // 使用正确的 Action 类型
  vision_client_ = rclcpp_action::create_client<VisionDet>(node_, "/vision/detection_pose_cloud");
  arm_client_ = rclcpp_action::create_client<MoveArm>(node_, "/Control/move_arm");
  gripper_client_ = rclcpp_action::create_client<CtrlGripper>(node_, "/Control/control_gripper");
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
      if(result.result->success)
      {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::VISION, "Vision detect succeeded [" << state_key << "]");
        print_vision_result(node_->get_logger(), *result.result);
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::SUCCESS;
        ctx.call_result = ActionCallResult::SUCCESS;
      }
      else
      {
        if (result.result->status_code == 3)
        {
          LOG_WARN(node_->get_logger(), task_log::Tag::VISION, "Vision detect server busy [" << state_key << "]: " << result.result->error_message);
          auto& ctx = get_ctx(state_key);
          ctx.call_result = ActionCallResult::FAIL_SERVER_BUSY;
        }
        else
        {
          LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision detect failed [" << state_key << "]: " << result.result->error_message);
          auto& ctx = get_ctx(state_key);
          ctx.call_result = ActionCallResult::FAIL_GENERAL;
        }
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::FAILURE;
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
// call_move_arm
// ============================================================================

ActionCallResult TaskActionClient::call_move_arm(
  const std::string& task_id, const task_cfg::MoveArmParam& param,
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
  if (param.arm_id.empty())
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "arm_id is empty");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }

  auto goal = MoveArm::Goal();
  goal.header.stamp = node_->now();
  goal.task_id = task_id;

  if (param.arm_id == "arm0")
  {
    goal.arm_id = 0;
  }
  else if (param.arm_id == "arm1")
  {
    goal.arm_id = 1;
  }
  else
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "unknown arm_id: " << param.arm_id);
    return ActionCallResult::FAIL_PARAM_INVALID;
  }
  
  goal.use_planning = param.use_planning;
  goal.enable_collision_check = param.collision_check;

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
  
  goal.safety_distance = param.safety_distance;
  goal.env_point_cloud = cloud;
  goal.point_cloud_frame_id = frame_id;


  // 增加等待时间和重试机制
  int max_retries = 3;
  int retry_count = 0;
  bool server_ready = false;
  
  while (retry_count < max_retries && !server_ready) {
    LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "Waiting for ArmMove action server (attempt " << (retry_count + 1) << "/" << max_retries << ")...");
    
    server_ready = arm_client_->wait_for_action_server(std::chrono::seconds(5));
    
    if (!server_ready) {
      retry_count++;
      if (retry_count < max_retries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
  
  if (!server_ready) {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "move arm action server timeout after " << max_retries << " attempts");
    return ActionCallResult::FAIL_SERVER_TIMEOUT;
  }
  
  LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "move arm action server connected successfully");



  // 5. 配置异步回调选项 (SendGoalOptions)
  auto send_goal_options = rclcpp_action::Client<MoveArm>::SendGoalOptions();
  
  // 目标响应回调：使用 lambda 捕获 state_key，更新正确的 context
  send_goal_options.goal_response_callback =
    [this, state_key](GoalHandleMoveArm::SharedPtr goal_handle)
    {
      auto goal = goal_handle.get();
      if (!goal) {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Goal was rejected by server [" << state_key << "]");
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
      } else {
        LOG_INFO(node_->get_logger(), task_log::Tag::ARM, "Goal accepted by server [" << state_key << "], waiting for result...");
      }
    };
    
  // 反馈回调：服务器周期性发送进度时触发
  send_goal_options.feedback_callback =
    std::bind(&TaskActionClient::arm_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    
  // 结果回调：使用 lambda 捕获 state_key，更新正确的 context
  send_goal_options.result_callback =
    [this, state_key](const GoalHandleMoveArm::WrappedResult & result)
    {
      if(result.result->success)
      {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::ARM, "Move arm succeeded [" << state_key << "]");
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::SUCCESS;
        ctx.call_result = ActionCallResult::SUCCESS;
      }
      else
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Move arm failed [" << state_key << "]: " << result.result->error_message);
        auto& ctx = get_ctx(state_key);
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
    };
  auto send_goal_future = arm_client_->async_send_goal(goal, send_goal_options);
  return ActionCallResult::SUCCESS;
 
}

void TaskActionClient::arm_goal_response_callback(GoalHandleMoveArm::SharedPtr goal_handle)
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

void TaskActionClient::arm_feedback_callback(GoalHandleMoveArm::SharedPtr, const std::shared_ptr<const MoveArm::Feedback> feedback)
{
  LOG_DEBUG(node_->get_logger(), task_log::Tag::ARM, "Feedback: progress=" << (feedback ? std::to_string(feedback->progress_rate) : "N/A"));
}

void TaskActionClient::arm_result_callback(const GoalHandleMoveArm::WrappedResult & result)
{
  if(result.result->success)
  {
    LOG_SUCCESS(node_->get_logger(), task_log::Tag::ARM, "Move arm succeeded");
    auto& ctx = get_ctx("default");
    ctx.state = ActionRunState::SUCCESS;
    ctx.call_result = ActionCallResult::SUCCESS;
    
  }
  else
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Move arm failed: " << result.result->error_message);
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
// call_gripper
// ============================================================================

ActionCallResult TaskActionClient::call_gripper(
  const std::string& task_id, const task_cfg::GripperParam& param,
  const std::string& state_key)
{

  LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::GRIPPER, "===== Gripper Control Start =====");
  
  // 参数校验
  if (task_id.empty())
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER, "task_id is empty");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }
  if (param.position.size() < 2)
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER, "position size < 2");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }

  // 1. 等待 Server 就绪
  if(!gripper_client_->wait_for_action_server(std::chrono::seconds(2))) {
    LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER, "Gripper action server not available");
    return ActionCallResult::FAIL_SERVER_TIMEOUT;
  }

  // 2. 构造 Goal
  auto goal = CtrlGripper::Goal();
  goal.header.stamp = node_->now();
  goal.task_id = task_id;
  goal.gripper_id = param.gripper_id;
  goal.position[0] = param.position[0];
  goal.position[1] = param.position[1];
  goal.speed[0] = param.speed[0];
  goal.speed[1] = param.speed[1];
  goal.max_torque[0] = param.max_torque[0];
  goal.max_torque[1] = param.max_torque[1];
  goal.com_port[0] = param.com_port[0];
  goal.com_port[1] = param.com_port[1];
  goal.block_until_complete = param.block;
  goal.timeout_ms = param.timeout_ms;

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
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
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
      if(result.result->success)
      {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::GRIPPER, "Gripper control succeeded [" << state_key << "]: final_position=["
          << (int)result.result->final_position[0] << "," << (int)result.result->final_position[1]
          << "], final_torque=[" << (int)result.result->final_torque[0] << "," << (int)result.result->final_torque[1] << "]");
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::SUCCESS;
        ctx.call_result = ActionCallResult::SUCCESS;
      }
      else
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::GRIPPER, "Gripper control failed [" << state_key << "]: " << result.result->error_message);
        auto& ctx = get_ctx(state_key);
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
    else if (act_type == "move_arm")
    {
        ctx.state = ActionRunState::RUNNING;
        ctx.execution_time_ms = 60000;
        geometry_msgs::msg::PoseStamped dummy_pose = pose;
        sensor_msgs::msg::PointCloud2 dummy_cloud = cloud;
        ActionCallResult res = call_move_arm(task_id, act.arm_param, dummy_pose, dummy_cloud, "base_link", state_key);
        if (action_call_result_is_ok(res))
        {
            LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal move_arm: " << action_call_result_to_str(res));
            ctx.state = ActionRunState::RUNNING;
            return ActionCallResult::SUCCESS;
        }
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = res;
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal move_arm failed: " << action_call_result_to_str(res));
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
        // 释放该 context 占用的所有资源
        for (const auto& res : ctx.acquired_resources)
        {
            release_resource(res);
        }
        ctx.acquired_resources.clear();
    }
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
    for (const auto& res : required_res)
    {
        if (!acquire_resource(res))
        {
            LOG_WARN(node_->get_logger(), task_log::Tag::ACTION,
                "[Branch " << state_key << "] send_goal " << act_type
                << " failed: resource " << resource_type_to_str(res) << " is busy, will retry");
            ctx.state = ActionRunState::IDLE;
            ctx.call_result = ActionCallResult::FAIL_RESOURCE_BUSY;
            return ActionCallResult::FAIL_RESOURCE_BUSY;
        }
    }
    ctx.acquired_resources = required_res;
    ctx.start_time = node_->now();

    // ========== 动作类型分发 ==========
    ActionCallResult ret = dispatch_action(act, current_task_id_, ctx, state_key);

    // 如果发送失败（非资源锁原因），释放已获取的资源
    if (!action_call_result_is_ok(ret))
    {
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
    else if (act_type == "move_arm")
    {
        if (act.arm_param.arm_id == "arm0")
        {
            resources.push_back(ResourceType::LEFT_ARM);
        }
        else if (act.arm_param.arm_id == "arm1")
        {
            resources.push_back(ResourceType::RIGHT_ARM);
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
        "Goal: task_id=" << goal.task_id << ", timeout_ms=" << goal.timeout_ms);

    auto send_goal_options = rclcpp_action::Client<HardWareCheck>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this, state_key](GoalHandleHardWareCheck::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision CheckSelf goal rejected [" << state_key << "]");
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
        "Goal: task_id=" << goal.task_id << ", timeout_ms=" << goal.timeout_ms);

    auto send_goal_options = rclcpp_action::Client<HardWareCheck>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this, state_key](GoalHandleHardWareCheck::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Control CheckSelf goal rejected [" << state_key << "]");
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
        "Goal: task_id=" << goal.task_id << ", timeout_ms=" << goal.timeout_ms);

    auto send_goal_options = rclcpp_action::Client<HardWareReset>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this, state_key](GoalHandleHardWareReset::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "Vision Reset goal rejected [" << state_key << "]");
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
        "Goal: task_id=" << goal.task_id << ", timeout_ms=" << goal.timeout_ms);

    auto send_goal_options = rclcpp_action::Client<HardWareReset>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this, state_key](GoalHandleHardWareReset::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                LOG_ERROR(node_->get_logger(), task_log::Tag::ARM, "Control Reset goal rejected [" << state_key << "]");
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
