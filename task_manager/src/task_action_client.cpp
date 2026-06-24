#include "task_manager/task_action_client.hpp"
#include <random>
#include <sstream>
#include <uuid/uuid.h>

// 添加正确的 Action 类型头文件
#include "vision_detection_action/action/vision_detection.hpp"

#include "motion_control_pkg/action/control_gripper.hpp"
#include "motion_control_pkg/action/move_arm.hpp"

namespace task_act
{
TaskActionClient::TaskActionClient(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // 使用正确的 Action 类型
  vision_client_ = rclcpp_action::create_client<VisionDet>(node_, "/vision/detection_pose_cloud");
  arm_client_ = rclcpp_action::create_client<MoveArm>(node_, "/Control/move_arm");
  gripper_client_ = rclcpp_action::create_client<CtrlGripper>(node_, "/Control/control_gripper");
  std::shared_future<GoalHandleCtrlGripper> gripper_send_future_{};
  reset_state();

}

bool
TaskActionClient::call_vision_detect(
  const std::string& task_id, const task_cfg::VisionActionParam& param)
{

  RCLCPP_INFO(node_->get_logger(), "---------call_vision_detect Start--------");
  auto goal = vision_detection_action::action::VisionDetection::Goal();
  goal.header.stamp = node_->now();
  goal.task_id = task_id;
  goal.target_obj_name = param.target_obj_name;
  goal.expected_frame_id = param.expected_frame_id;
  goal.need_6d_pose = param.need_6d_pose;
  goal.need_env_point_cloud = param.need_env_point_cloud;
  goal.point_cloud_downsample_resolution = param.downsample_res;
  goal.priority = param.priority;
  goal.type = param.type;
  
  RCLCPP_INFO(node_->get_logger(), "=== Goal Parameters ===");
  RCLCPP_INFO(node_->get_logger(), "Header Stamp: sec=%d, nanosec=%u", 
              goal.header.stamp.sec, 
              goal.header.stamp.nanosec);
  RCLCPP_INFO(node_->get_logger(), "Task ID: %s", goal.task_id.c_str());
  RCLCPP_INFO(node_->get_logger(), "Target Object Name: %s", goal.target_obj_name.c_str());
  RCLCPP_INFO(node_->get_logger(), "Expected Frame ID: %s", goal.expected_frame_id.c_str());
  RCLCPP_INFO(node_->get_logger(), "Need 6D Pose: %s", goal.need_6d_pose ? "true" : "false");
  RCLCPP_INFO(node_->get_logger(), "Need Env PointCloud: %s", goal.need_env_point_cloud ? "true" : "false");
  RCLCPP_INFO(node_->get_logger(), "PointCloud Downsample Resolution: %.4f", goal.point_cloud_downsample_resolution);
  RCLCPP_INFO(node_->get_logger(), "Priority: %d", goal.priority);
  RCLCPP_INFO(node_->get_logger(), "Type: %d", goal.type);
  
  // 增加等待时间和重试机制
  int max_retries = 3;
  int retry_count = 0;
  bool server_ready = false;
  
  while (retry_count < max_retries && !server_ready) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for vision action server (attempt %d/%d)...", 
                retry_count + 1, max_retries);
    
    server_ready = vision_client_->wait_for_action_server(std::chrono::seconds(5));
    
    if (!server_ready) {
      retry_count++;
      if (retry_count < max_retries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
  
  if (!server_ready) {
    RCLCPP_ERROR(node_->get_logger(), "Vision action server timeout after %d attempts", max_retries);
    RCLCPP_INFO(node_->get_logger(), "---------call_vision_detect End--------");
    return false;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Vision action server connected successfully");
  


  // 5. 配置异步回调选项 (SendGoalOptions)
  auto send_goal_options = rclcpp_action::Client<VisionDet>::SendGoalOptions();
  
  // 目标响应回调：当服务器接受或拒绝目标时触发
  send_goal_options.goal_response_callback =
    std::bind(&TaskActionClient::goal_response_callback, this, std::placeholders::_1);
    
  // 反馈回调：服务器周期性发送进度时触发
  send_goal_options.feedback_callback =
    std::bind(&TaskActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    
  // 结果回调：任务完成时触发
  send_goal_options.result_callback =
    std::bind(&TaskActionClient::result_callback, this, std::placeholders::_1);
  auto send_goal_future = vision_client_->async_send_goal(goal, send_goal_options);
  current_state_ = ActionRunState::RUNNING;
  task_start_time_ = rclcpp::Time(goal.header.stamp);
  execution_time_ = 10000;
  return true;

}
void TaskActionClient::goal_response_callback(GoalHandleVisionDetection::SharedPtr goal_handle)
{
    auto goal = goal_handle.get();
    if (!goal) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        current_state_ = ActionRunState::RUNNING;
    } else {
        RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result...");
    }
}

void TaskActionClient::feedback_callback(GoalHandleVisionDetection::SharedPtr, const std::shared_ptr<const VisionDet::Feedback> feedback)
{
  RCLCPP_INFO(node_->get_logger(), "Received feedback: %.2f", feedback->progress_rate);
}

void TaskActionClient::result_callback(const GoalHandleVisionDetection::WrappedResult & result)
{
  if(result.result->success)
  {
    RCLCPP_INFO(node_->get_logger(), "---------call_vision_detect End: success--------");
    print_vision_result(node_->get_logger(), *result.result);
    current_state_ = ActionRunState::SUCCESS;
    
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "---------call_vision_detect End: failed:%s--------", result.result->error_message.c_str());
    current_state_ = ActionRunState::FAILURE;
  }
}

void TaskActionClient::print_vision_result(rclcpp::Logger logger, const VisionDet::Result & res)
{
  RCLCPP_INFO(logger, "==================== VisionDetection Result ====================");
  RCLCPP_INFO(logger, "success: %s", res.success ? "true" : "false");

  // Header
  RCLCPP_INFO(logger, "header.stamp: sec=%d, nanosec=%u, frame_id=%s",
              res.header.stamp.sec,
              res.header.stamp.nanosec,
              res.header.frame_id.c_str());

  // status_code 文字解析
  std::string status_str;
  switch (res.status_code)
  {
    case 0: status_str = "0: OK"; break;
    case 1: status_str = "1: FAILED (硬件故障)"; break;
    case 2: status_str = "2: TIMEOUT (超时未响应)"; break;
    case 3: status_str = "3: BUSY (正在执行其他高优先级任务)"; break;
    default: status_str = std::to_string(res.status_code) + ": Unknown code";
  }
  RCLCPP_INFO(logger, "status_code: %s", status_str.c_str());

  RCLCPP_INFO(logger, "error_message: [%s]", res.error_message.c_str());
  RCLCPP_INFO(logger, "execution_time: %.4f s", res.execution_time);

  // 6D PoseStamped
  const auto & pose = res.target_6d_pose;
  RCLCPP_INFO(logger, "target_6d_pose.frame_id: %s", pose.header.frame_id.c_str());
  RCLCPP_INFO(logger, "target_6d_pose.position: x=%.4f, y=%.4f, z=%.4f",
              pose.pose.position.x,
              pose.pose.position.y,
              pose.pose.position.z);
  RCLCPP_INFO(logger, "target_6d_pose.orientation: qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",
              pose.pose.orientation.x,
              pose.pose.orientation.y,
              pose.pose.orientation.z,
              pose.pose.orientation.w);

  RCLCPP_INFO(logger, "pose_confidence: %.4f", res.pose_confidence);

  // 点云信息（只打印基础信息，不打印海量点）
  RCLCPP_INFO(logger, "env_point_cloud.header.frame_id: %s", res.env_point_cloud.header.frame_id.c_str());
  RCLCPP_INFO(logger, "env_point_cloud width=%u, height=%u, point_step=%u",
              res.env_point_cloud.width,
              res.env_point_cloud.height,
              res.env_point_cloud.point_step);
  RCLCPP_INFO(logger, "point_cloud_frame_id: %s", res.point_cloud_frame_id.c_str());
  RCLCPP_INFO(logger, "================================================================\n");
}

bool TaskActionClient::call_move_arm(
  const std::string& task_id, const task_cfg::MoveArmParam& param,
  const geometry_msgs::msg::PoseStamped& target_pose,
  const sensor_msgs::msg::PointCloud2& cloud, const std::string& frame_id)
{
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
    goal.arm_id = 2;
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
    RCLCPP_INFO(node_->get_logger(), "Waiting for ArmMove action server (attempt %d/%d)...", 
                retry_count + 1, max_retries);
    
    server_ready = arm_client_->wait_for_action_server(std::chrono::seconds(5));
    
    if (!server_ready) {
      retry_count++;
      if (retry_count < max_retries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
  
  if (!server_ready) {
    RCLCPP_ERROR(node_->get_logger(), "move arm action server timeout after %d attempts", max_retries);
    RCLCPP_INFO(node_->get_logger(), "---------call_move_arm End--------");
    return false;
  }
  
  RCLCPP_INFO(node_->get_logger(), "move arm action server connected successfully");
  


  // 5. 配置异步回调选项 (SendGoalOptions)
  auto send_goal_options = rclcpp_action::Client<MoveArm>::SendGoalOptions();
  
  // 目标响应回调：当服务器接受或拒绝目标时触发
  send_goal_options.goal_response_callback =
    std::bind(&TaskActionClient::arm_goal_response_callback, this, std::placeholders::_1);
    
  // 反馈回调：服务器周期性发送进度时触发
  send_goal_options.feedback_callback =
    std::bind(&TaskActionClient::arm_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    
  // 结果回调：任务完成时触发
  send_goal_options.result_callback =
    std::bind(&TaskActionClient::arm_result_callback, this, std::placeholders::_1);
  auto send_goal_future = arm_client_->async_send_goal(goal, send_goal_options);
  current_state_ = ActionRunState::RUNNING;
  task_start_time_ = rclcpp::Time(goal.header.stamp);
  execution_time_ = 10000;
  return true;
 
}

void TaskActionClient::arm_goal_response_callback(GoalHandleMoveArm::SharedPtr goal_handle)
{
    auto goal = goal_handle.get();
    if (!goal) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        current_state_ = ActionRunState::RUNNING;
    } else {
        RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result...");
    }
}

void TaskActionClient::arm_feedback_callback(GoalHandleMoveArm::SharedPtr, const std::shared_ptr<const MoveArm::Feedback> feedback)
{
  RCLCPP_INFO(node_->get_logger(), "arm Received feedback: %.2f", feedback->progress_rate);
}

void TaskActionClient::arm_result_callback(const GoalHandleMoveArm::WrappedResult & result)
{
  if(result.result->success)
  {
    RCLCPP_INFO(node_->get_logger(), "---------arm_result_callback End: success--------");
    current_state_ = ActionRunState::SUCCESS;
    
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "---------arm_result_callback End: failed--------");
    current_state_ = ActionRunState::FAILURE;
  }
}


bool TaskActionClient::call_gripper(
  const std::string& task_id, const task_cfg::GripperParam& param)
{

  RCLCPP_INFO(node_->get_logger(), "--------call_gripper start---------");
  // 1. 等待 Server 就绪
  if(!gripper_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper action server not available");
    return false;
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

  RCLCPP_INFO(node_->get_logger(), "Sending Gripper Goal: Task ID=%s", task_id.c_str());
  RCLCPP_INFO(node_->get_logger(), "Header Stamp: sec=%d, nanosec=%u", 
              goal.header.stamp.sec, 
              goal.header.stamp.nanosec);
  // 增加等待时间和重试机制
  int max_retries = 3;
  int retry_count = 0;
  bool server_ready = false;
  
  while (retry_count < max_retries && !server_ready) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for gripper action server (attempt %d/%d)...", 
                retry_count + 1, max_retries);
    
    server_ready = gripper_client_->wait_for_action_server(std::chrono::seconds(5));
    
    if (!server_ready) {
      retry_count++;
      if (retry_count < max_retries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
  
  if (!server_ready) {
    RCLCPP_ERROR(node_->get_logger(), "gripper action server timeout after %d attempts", max_retries);
    RCLCPP_INFO(node_->get_logger(), "---------call_gripper End--------");
    return false;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Vision action server connected successfully");
  


  // 5. 配置异步回调选项 (SendGoalOptions)
  auto send_goal_options = rclcpp_action::Client<CtrlGripper>::SendGoalOptions();
  
  // 目标响应回调：当服务器接受或拒绝目标时触发
  send_goal_options.goal_response_callback =
    std::bind(&TaskActionClient::gripper_goal_response_callback, this, std::placeholders::_1);
    
  // 反馈回调：服务器周期性发送进度时触发
  send_goal_options.feedback_callback =
    std::bind(&TaskActionClient::gripper_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    
  // 结果回调：任务完成时触发
  send_goal_options.result_callback =
    std::bind(&TaskActionClient::gripper_result_callback, this, std::placeholders::_1);
  gripper_send_future_ = gripper_client_->async_send_goal(goal, send_goal_options);
  current_state_ = ActionRunState::RUNNING;
  //StartTimer(goal.timeout_ms);
  task_start_time_ = rclcpp::Time(goal.header.stamp);


  execution_time_ = goal.timeout_ms;
  return true;
}
rclcpp::Time TaskActionClient::get_ros_now_time() const
{
  return node_->now();
}


rclcpp::Time TaskActionClient::get_task_start_time() const
{
  return task_start_time_;
}

uint32_t TaskActionClient::get_task_execution_time() const
{
  return execution_time_;
}


void TaskActionClient::gripper_goal_response_callback(GoalHandleCtrlGripper::SharedPtr goal_handle)
{
    auto goal = goal_handle.get();
    if (!goal) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        current_state_ = ActionRunState::RUNNING;
    } else {
        RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result...");
    }
}

void TaskActionClient::gripper_feedback_callback(GoalHandleCtrlGripper::SharedPtr, const std::shared_ptr<const GoalHandleCtrlGripper::Feedback> feedback)
{
  RCLCPP_INFO(node_->get_logger(), "gripper Received feedback: %.2f", feedback->progress_rate);
}

void TaskActionClient::gripper_result_callback(const GoalHandleCtrlGripper::WrappedResult & result)
{
  //StopTimer();
  if(result.result->success)
  {
    RCLCPP_INFO(node_->get_logger(), "---------gripper_result_callback End: success--------\nfinal_position[%d %d]. final_torque[%d %d]", 
    result.result->final_position[0], result.result->final_position[1], result.result->final_torque[0], result.result->final_torque[1]);
    current_state_ = ActionRunState::SUCCESS;
    
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "---------gripper_result_callback End: failed--------\nerror msg: %s", 
    result.result->error_message.c_str());
    current_state_ = ActionRunState::FAILURE;
  }
}

bool TaskActionClient::send_goal(const task_cfg::AtomicAction& act)
{
  // 先重置状态
    reset_state();

    const std::string& act_type = act.action_type;
    uuid_t uuid;
    uuid_generate(uuid);
    char buf[40];
    uuid_unparse(uuid, buf);
    std::string current_task_id_ = std::string(buf);
    bool ret = false;

    // 根据动作类型分支调用
    if (act_type == "vision_detect")
    {
        current_state_ = ActionRunState::RUNNING;
        
        bool res = call_vision_detect(current_task_id_, act.vision_param);

        if (res)
        {
          RCLCPP_INFO(node_->get_logger(), "Detect vision result: true");
          current_state_ = ActionRunState::RUNNING;

          ret = true;
        } 
        else 
        {
          current_state_ = ActionRunState::FAILURE;

          ret = false;
        }
        
    }
    else if (act_type == "move_arm")
    {
      current_state_ = ActionRunState::RUNNING;
      // 此处根据你实际业务补充 pose / pointcloud 等参数
      geometry_msgs::msg::PoseStamped dummy_pose = pose;
      sensor_msgs::msg::PointCloud2 dummy_cloud = cloud;
        
      bool res = call_move_arm(current_task_id_, act.arm_param, dummy_pose, dummy_cloud, "base_link");
      if (res)
      {
        RCLCPP_INFO(node_->get_logger(), "move arm result: true");
        current_state_ = ActionRunState::RUNNING;
        ret = true;
      }
      else
      {
          current_state_ = ActionRunState::FAILURE;
      }
    }
    else if (act_type == "gripper_ctrl")
    {
      current_state_ = ActionRunState::RUNNING;
      RCLCPP_INFO(node_->get_logger(), "---------act.gripper_param position size:%ld--------------", act.gripper_param.position.size());    
      bool res = call_gripper(current_task_id_, act.gripper_param);
      if (res)
      {
        RCLCPP_INFO(node_->get_logger(), "gripper control result: true");
        current_state_ = ActionRunState::RUNNING;
        ret = true;
      }
      else
      {
        current_state_ = ActionRunState::FAILURE;
      }
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unknown  action type: %s", act_type.c_str());
      current_state_ = ActionRunState::FAILURE;
      ret = false;
    }

    return ret;
}
}
