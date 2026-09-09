#include "task_manager/task_action_client.hpp"
#include "task_manager/log_helper.hpp"
#include <algorithm>
#include <random>
#include <sstream>
#include <uuid/uuid.h>
#include <cmath>
#include <iomanip>
#include <thread>
#include <chrono>

// 使用 com_interfaces 中的 Action 定义
#include "com_interfaces/action/detection_pose_cloud.hpp"
#include "com_interfaces/action/control_gripper.hpp"
#include "com_interfaces/action/move_pose.hpp"
#include "com_interfaces/action/move_joints.hpp"
#include "com_interfaces/action/control_suck.hpp"

namespace task_act
{

// ============================================================================
// call_calibration_control - 调用视觉标定控制（/vision/calibration_control）
// ============================================================================
ActionCallResult TaskActionClient::call_calibration_control(
  const std::string& task_id, const task_cfg::CalibrationControlParam& param,
  const std::string& state_key)
{
  if (task_id.empty())
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "task_id is empty (calibration_control)");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }

  // 校验命令类型
  if (param.command != "CHECK_STATUS" && param.command != "TRIGGER_CAPTURE" &&
      param.command != "EXECUTE_CALIBRATION")
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::VISION,
      "invalid calibration command: " << param.command << " [" << state_key << "]");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }

  auto goal = CalibrationControl::Goal();
  goal.header.stamp = node_->now();
  goal.camera_id = param.camera_id;
  goal.arm_id = param.arm_id;
  goal.command = param.command;
  goal.auto_execute = param.auto_execute;
  goal.mode = param.mode;

  LOG_INFO(node_->get_logger(), task_log::Tag::VISION,
    "CalibrationControl Goal [" << state_key << "]: camera_id=" << static_cast<int>(goal.camera_id)
    << ", arm_id=" << static_cast<int>(goal.arm_id)
    << ", command=" << goal.command
    << ", auto_execute=" << (goal.auto_execute ? "true" : "false")
    << ", mode=" << goal.mode);

  // 等待服务端就绪（最多重试 3 次）
  bool server_ready = false;
  for (int i = 0; i < 3 && !server_ready; ++i)
  {
    server_ready = calibration_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!server_ready && i < 2)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
  if (!server_ready)
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::VISION,
      "CalibrationControl action server not available (attempted 3 times) [" << state_key << "]");
    return ActionCallResult::FAIL_SERVER_TIMEOUT;
  }

  auto send_goal_options = rclcpp_action::Client<CalibrationControl>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this, state_key](GoalHandleCalibrationControl::SharedPtr goal_handle)
    {
      auto goal = goal_handle.get();
      if (!goal)
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION,
          "CalibrationControl goal rejected by server [" << state_key << "]");
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GOAL_REJECTED;
      }
      else
      {
        LOG_INFO(node_->get_logger(), task_log::Tag::VISION,
          "CalibrationControl goal accepted by server [" << state_key << "], waiting for result...");
        // 保存活动的 goal handle，供抢占/急停时取消远端动作
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_calibration_goal_ = goal_handle;
      }
    };

  send_goal_options.result_callback =
    [this, state_key, command = std::string(param.command)](
      const GoalHandleCalibrationControl::WrappedResult & result)
    {
      if (result.result->success)
      {
        LOG_SUCCESS(node_->get_logger(), task_log::Tag::VISION,
          "CalibrationControl succeeded [" << state_key << "], command=" << command
          << ", total_captures=" << result.result->total_captures);
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::SUCCESS;
        ctx.call_result = ActionCallResult::SUCCESS;
      }
      else
      {
        LOG_ERROR(node_->get_logger(), task_log::Tag::VISION,
          "CalibrationControl failed [" << state_key << "]: " << result.result->error_message);
        auto& ctx = get_ctx(state_key);
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = ActionCallResult::FAIL_GENERAL;
      }

      // 释放该 action 占用的资源（遍历所有上下文）
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

      // 清除活动的 CalibrationControl goal handle
      {
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_calibration_goal_.reset();
      }
    };

  calibration_client_->async_send_goal(goal, send_goal_options);
  return ActionCallResult::SUCCESS;
}


// ============================================================================
// call_auto_calibrate - 自动标定入口（启动后台编排线程）
// ============================================================================
ActionCallResult TaskActionClient::call_auto_calibrate(
  const std::string& task_id, const task_cfg::AutoCalibrateParam& param,
  const std::string& state_key)
{
  if (task_id.empty())
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::VISION, "task_id is empty (auto_calibrate)");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }
  if (param.pose_pool.empty() || param.sample_num <= 0)
  {
    LOG_ERROR(node_->get_logger(), task_log::Tag::VISION,
      "auto_calibrate param invalid: pose_pool_size=" << param.pose_pool.size()
      << ", sample_num=" << param.sample_num << " [" << state_key << "]");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }

  auto& ctx = get_ctx(state_key);
  ctx.state = ActionRunState::RUNNING;
  ctx.call_result = ActionCallResult::SUCCESS;
  ctx.execution_time_ms = static_cast<uint32_t>(param.timeout_ms > 0 ? param.timeout_ms : 600000);

  // 清除可能残留的取消标志，启动后台编排线程（参数按值拷贝进线程，避免引用悬垂）
  auto_calibrate_cancel_.store(false, std::memory_order_release);

  LOG_INFO_TITLE(node_->get_logger(), task_log::Tag::VISION,
    "===== Auto Calibrate Start [" << state_key << "] =====");

  std::thread([this, task_id, param, state_key]()
  {
    auto_calibrate_worker(task_id, param, state_key);
  }).detach();

  return ActionCallResult::SUCCESS;
}


// ============================================================================
// wait_step_complete - 等待自动标定子步骤到达终态
// ============================================================================
bool TaskActionClient::wait_step_complete(
    const std::string& step_key, uint32_t timeout_ms, bool& out_success)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (rclcpp::ok())
  {
    // 被打断（急停 / 复位 / 新任务抢占）：通知后台线程退出
    if (auto_calibrate_cancel_.load(std::memory_order_acquire))
    {
      LOG_WARN(node_->get_logger(), task_log::Tag::VISION,
        "Auto-calibrate interrupted, stop waiting step [" << step_key << "]");
      return false;
    }

    const ActionRunState st = get_ctx(step_key).state;
    if (st == ActionRunState::SUCCESS)
    {
      out_success = true;
      return true;
    }
    if (st == ActionRunState::FAILURE)
    {
      out_success = false;
      return true;
    }
    if (std::chrono::steady_clock::now() > deadline)
    {
      LOG_ERROR(node_->get_logger(), task_log::Tag::VISION,
        "Auto-calibrate step [" << step_key << "] wait timeout (" << timeout_ms << " ms)");
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  LOG_WARN(node_->get_logger(), task_log::Tag::VISION,
    "rclcpp::ok() false, stop waiting step [" << step_key << "]");
  return false;
}


// ============================================================================
// auto_calibrate_worker - 自动标定后台编排线程主体
//   流程：CHECK_STATUS → 按顺序取前 sample_num 组 →
//         循环 { move_joints 关节空间到位 + TRIGGER_CAPTURE 采集 } → EXECUTE_CALIBRATION
// ============================================================================
void TaskActionClient::auto_calibrate_worker(
    const std::string& task_id, const task_cfg::AutoCalibrateParam& param,
    const std::string& state_key)
{
  auto logger = node_->get_logger();

  // 汇报最终结果（若已被打断则不再触碰主上下文，避免干扰打断后的新任务）
  auto finish = [&](bool ok, ActionCallResult cr)
  {
    if (auto_calibrate_cancel_.load(std::memory_order_acquire))
    {
      LOG_WARN(logger, task_log::Tag::VISION,
        "Auto-calibrate aborted by cancel flag, skip final report [" << state_key << "]");
      return;
    }
    auto& ctx = get_ctx(state_key);
    ctx.state = ok ? ActionRunState::SUCCESS : ActionRunState::FAILURE;
    ctx.call_result = cr;
    LOG_INFO(logger, task_log::Tag::VISION,
      "Auto-calibrate finished: " << (ok ? "SUCCESS" : "FAILURE") << " [" << state_key << "]");
  };

  uint32_t step_no = 0;

  // ========== 0. 头部初始到位：自动标定开始时先将头部 move_joints 到指定位置（可选） ==========
  if (param.head_move_enable)
  {
    if (auto_calibrate_cancel_.load(std::memory_order_acquire))
    {
      finish(false, ActionCallResult::FAIL_GENERAL);
      return;
    }

    if (param.head_position.empty())
    {
      LOG_ERROR(logger, task_log::Tag::VISION,
        "Auto-calibrate head_move_enable=true but head_position is empty, abort");
      finish(false, ActionCallResult::FAIL_PARAM_INVALID);
      return;
    }

    // 头部关节名：优先使用配置 head_joint_name；未配置时回退默认头部双关节
    std::vector<std::string> head_names = param.head_joint_name;
    if (head_names.empty())
    {
      head_names = {"head_yaw_joint", "head_pitch_joint"};
    }
    if (head_names.size() != param.head_position.size())
    {
      LOG_ERROR(logger, task_log::Tag::VISION,
        "Auto-calibrate head config mismatch: head_joint_name.size()=" << head_names.size()
        << " != head_position.size()=" << param.head_position.size());
      finish(false, ActionCallResult::FAIL_PARAM_INVALID);
      return;
    }

    const std::string step_key = state_key + "_head_move";
    auto& sctx = get_ctx(step_key);
    sctx.state = ActionRunState::RUNNING;
    sctx.call_result = ActionCallResult::SUCCESS;
    const uint32_t head_timeout_ms = static_cast<uint32_t>(
      param.head_move_timeout_ms > 0 ? param.head_move_timeout_ms : 15000);
    sctx.execution_time_ms = head_timeout_ms;

    task_cfg::MoveJointsParam hparam;
    hparam.arm_id = MoveJoints::Goal::HEAD;   // 3 = HEAD，见 com_interfaces/action/MoveJoints.action
    hparam.joint_name = head_names;
    // 将 head_position（float）转为 move_joints 所需的 double 关节角
    hparam.position.assign(param.head_position.begin(), param.head_position.end());
    hparam.enable_collision_check = param.enable_collision_check;
    hparam.speed = param.head_speed;
    hparam.timeout_ms = param.head_move_timeout_ms;

    ActionCallResult r = call_move_joints(task_id, hparam, step_key);
    if (!action_call_result_is_ok(r))
    {
      LOG_ERROR(logger, task_log::Tag::VISION,
        "Auto-calibrate head move send failed: " << action_call_result_to_str(r));
      finish(false, r);
      return;
    }
    bool head_ok = false;
    if (!wait_step_complete(step_key, head_timeout_ms, head_ok))
    {
      finish(false, ActionCallResult::FAIL_TIMEOUT);
      return;
    }
    if (!head_ok)
    {
      LOG_ERROR(logger, task_log::Tag::VISION,
        "Auto-calibrate head move failed to reach target position");
      finish(false, ActionCallResult::FAIL_GENERAL);
      return;
    }
    reset_action_state(step_key);
  }

  // ========== 1. CHECK_STATUS：请求标定（检查相机/数据是否就绪并锁定 arm_id）==========
  {
    const std::string step_key = state_key + "_check";
    auto& sctx = get_ctx(step_key);
    sctx.state = ActionRunState::RUNNING;
    sctx.call_result = ActionCallResult::SUCCESS;

    task_cfg::CalibrationControlParam check_param;
    check_param.camera_id = param.camera_id;
    check_param.arm_id = param.arm_id;
    check_param.command = "CHECK_STATUS";
    check_param.auto_execute = false;
    check_param.mode = param.mode;
    check_param.timeout_ms = param.capture_timeout_ms;

    ActionCallResult r = call_calibration_control(task_id, check_param, step_key);
    if (!action_call_result_is_ok(r))
    {
      LOG_ERROR(logger, task_log::Tag::VISION,
        "Auto-calibrate CHECK_STATUS send failed: " << action_call_result_to_str(r));
      finish(false, r);
      return;
    }
    bool ok = false;
    if (!wait_step_complete(step_key, static_cast<uint32_t>(check_param.timeout_ms), ok))
    {
      finish(false, ActionCallResult::FAIL_TIMEOUT);
      return;
    }
    if (!ok)
    {
      LOG_ERROR(logger, task_log::Tag::VISION, "Auto-calibrate CHECK_STATUS returned failure");
      finish(false, ActionCallResult::FAIL_GENERAL);
      return;
    }
    reset_action_state(step_key);
  }

  if (auto_calibrate_cancel_.load(std::memory_order_acquire))
  {
    finish(false, ActionCallResult::FAIL_GENERAL);
    return;
  }

  // ========== 2. 按 pose_pool 原始顺序取前 sample_num 组目标位姿 ==========
  std::vector<std::vector<float>> selected;
  {
    const auto& pool = param.pose_pool;
    const uint32_t need = static_cast<uint32_t>(std::min<int32_t>(
      param.sample_num, static_cast<int32_t>(pool.size())));
    for (uint32_t i = 0; i < need; ++i)
    {
      selected.push_back(pool[i]);   // 保持 pose_pool 原有顺序，逐组执行
    }
    LOG_INFO(logger, task_log::Tag::VISION,
      "Auto-calibrate executes " << selected.size() << " poses in order from pool of " << pool.size());
    if (selected.empty())
    {
      LOG_ERROR(logger, task_log::Tag::VISION, "Auto-calibrate pose_pool empty");
      finish(false, ActionCallResult::FAIL_PARAM_INVALID);
      return;
    }
  }

  // ========== 3. 循环：move_joints 关节空间到位 + TRIGGER_CAPTURE 采集 ==========
  for (size_t i = 0; i < selected.size(); ++i)
  {
    if (auto_calibrate_cancel_.load(std::memory_order_acquire))
    {
      finish(false, ActionCallResult::FAIL_GENERAL);
      return;
    }

    const auto& pose_vals = selected[i];
    if (pose_vals.size() < 7)
    {
      LOG_ERROR(logger, task_log::Tag::VISION,
        "Auto-calibrate pose #" << i << " has < 7 elements, skip");
      continue;
    }

    // ---- 3.1 move_joints（按标定配置臂关节空间移动到位，默认右臂）----
    {
      const std::string step_key = state_key + "_move_" + std::to_string(step_no++);
      auto& sctx = get_ctx(step_key);
      sctx.state = ActionRunState::RUNNING;
      sctx.call_result = ActionCallResult::SUCCESS;
      sctx.execution_time_ms = static_cast<uint32_t>(param.move_timeout_ms > 0 ? param.move_timeout_ms : 15000);

      // 关节名：优先使用配置 joint_name；未配置（或不足 7 个）则按臂回退默认名
      //   Joint1_R..Joint7_R（右臂）/ Joint1_L..Joint7_L（左臂）
      std::vector<std::string> joint_names = param.joint_name;
      if (joint_names.size() < 7)
      {
        joint_names.clear();
        const bool right_arm = (param.arm_id == MoveJoints::Goal::ARM_RIGHT);
        const char* suffix = right_arm ? "_R" : "_L";
        for (int k = 1; k <= 7; ++k)
        {
          joint_names.emplace_back(std::string("Joint") + std::to_string(k) + suffix);
        }
      }

      task_cfg::MoveJointsParam jparam;
      // 移动跟随标定配置臂（0=左臂 ARM_LEFT，1=右臂 ARM_RIGHT，与 CalibrationControl 编码一致）
      jparam.arm_id = (param.arm_id == MoveJoints::Goal::ARM_RIGHT)
                          ? MoveJoints::Goal::ARM_RIGHT : MoveJoints::Goal::ARM_LEFT;
      jparam.joint_name = joint_names;
      // pose_vals 为该组 7 个关节角（单位 rad，顺序与 joint_name 对应）
      for (size_t k = 0; k < 7; ++k)
      {
        jparam.position.push_back(pose_vals[k]);
      }
      jparam.enable_collision_check = param.enable_collision_check;
      jparam.speed = param.speed;
      jparam.timeout_ms = param.move_timeout_ms;

      ActionCallResult r = call_move_joints(task_id, jparam, step_key);
      if (!action_call_result_is_ok(r))
      {
        LOG_ERROR(logger, task_log::Tag::VISION,
          "Auto-calibrate move_joints #" << i << " send failed: " << action_call_result_to_str(r));
        finish(false, r);
        return;
      }
      bool ok = false;
      if (!wait_step_complete(step_key, static_cast<uint32_t>(jparam.timeout_ms), ok))
      {
        finish(false, ActionCallResult::FAIL_TIMEOUT);
        return;
      }
      if (!ok)
      {
        LOG_ERROR(logger, task_log::Tag::VISION, "Auto-calibrate move_joints #" << i << " failed to reach target");
        finish(false, ActionCallResult::FAIL_GENERAL);
        return;
      }
      reset_action_state(step_key);
    }

    // ---- 3.1b 到位稳定等待：move_joints 完成后等待 post_move_delay_ms 再触发采集 ----
    if (param.post_move_delay_ms > 0)
    {
      const auto deadline = std::chrono::steady_clock::now() +
                            std::chrono::milliseconds(param.post_move_delay_ms);
      LOG_INFO(logger, task_log::Tag::VISION,
        "Auto-calibrate waiting " << param.post_move_delay_ms
        << " ms after move_joints #" << i << " before capture");
      while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
      {
        if (auto_calibrate_cancel_.load(std::memory_order_acquire))
        {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      if (auto_calibrate_cancel_.load(std::memory_order_acquire))
      {
        finish(false, ActionCallResult::FAIL_GENERAL);
        return;
      }
    }

    // ---- 3.2 TRIGGER_CAPTURE（通知视觉节点采集当前末端位姿）----
    {
      const std::string step_key = state_key + "_cap_" + std::to_string(step_no++);
      auto& sctx = get_ctx(step_key);
      sctx.state = ActionRunState::RUNNING;
      sctx.call_result = ActionCallResult::SUCCESS;

      task_cfg::CalibrationControlParam cap_param;
      cap_param.camera_id = param.camera_id;
      cap_param.arm_id = param.arm_id;
      cap_param.command = "TRIGGER_CAPTURE";
      cap_param.auto_execute = false;    // 全部采集完成后由 EXECUTE_CALIBRATION 统一计算
      cap_param.mode = param.mode;
      cap_param.timeout_ms = param.capture_timeout_ms;

      ActionCallResult r = call_calibration_control(task_id, cap_param, step_key);
      if (!action_call_result_is_ok(r))
      {
        LOG_ERROR(logger, task_log::Tag::VISION,
          "Auto-calibrate TRIGGER_CAPTURE #" << i << " send failed: " << action_call_result_to_str(r));
        finish(false, r);
        return;
      }
      bool ok = false;
      if (!wait_step_complete(step_key, static_cast<uint32_t>(cap_param.timeout_ms), ok))
      {
        finish(false, ActionCallResult::FAIL_TIMEOUT);
        return;
      }
      if (!ok)
      {
        LOG_ERROR(logger, task_log::Tag::VISION,
          "Auto-calibrate TRIGGER_CAPTURE #" << i << " returned failure");
        finish(false, ActionCallResult::FAIL_GENERAL);
        return;
      }
      reset_action_state(step_key);
    }
  }

  if (auto_calibrate_cancel_.load(std::memory_order_acquire))
  {
    finish(false, ActionCallResult::FAIL_GENERAL);
    return;
  }

  // ========== 4. EXECUTE_CALIBRATION：通知视觉节点进行标定计算 ==========
  {
    const std::string step_key = state_key + "_execute";
    auto& sctx = get_ctx(step_key);
    sctx.state = ActionRunState::RUNNING;
    sctx.call_result = ActionCallResult::SUCCESS;

    task_cfg::CalibrationControlParam exec_param;
    exec_param.camera_id = param.camera_id;
    exec_param.arm_id = param.arm_id;
    exec_param.command = "EXECUTE_CALIBRATION";
    exec_param.auto_execute = false;
    exec_param.mode = param.mode;
    exec_param.timeout_ms = param.timeout_ms;

    ActionCallResult r = call_calibration_control(task_id, exec_param, step_key);
    if (!action_call_result_is_ok(r))
    {
      LOG_ERROR(logger, task_log::Tag::VISION,
        "Auto-calibrate EXECUTE_CALIBRATION send failed: " << action_call_result_to_str(r));
      finish(false, r);
      return;
    }
    bool ok = false;
    if (!wait_step_complete(step_key, static_cast<uint32_t>(exec_param.timeout_ms), ok))
    {
      finish(false, ActionCallResult::FAIL_TIMEOUT);
      return;
    }
    if (!ok)
    {
      LOG_ERROR(logger, task_log::Tag::VISION, "Auto-calibrate EXECUTE_CALIBRATION returned failure");
      finish(false, ActionCallResult::FAIL_GENERAL);
      return;
    }
    reset_action_state(step_key);
  }

  finish(true, ActionCallResult::SUCCESS);
}

} // namespace task_act
