// task_action_client_compound.cpp
// move_compound（复合动作）实现：在单个动作内部并行驱动多个 move_pose / move_joints 单体动作
//
// 设计要点：
//   1. 每个子动作与普通 move_pose / move_joints 完全一致：参数结构复用 MovePoseParam /
//      MoveJointsParam（配置解析即复用 parse_move_pose_params / parse_move_joints_params），
//      目标位姿注入（序列级视觉结果 / 成员 pose）逻辑也与普通 move_pose 保持一致。
//   2. 每个子动作在独立子 state_key 上调用 call_move_pose / call_move_joints（均为异步发送，
//      立即返回），各 Action Server（/Control/move_pose、/Control/move_joints）相互独立，
//      因此可在单个动作内部实现真正并行执行。
//   3. 后台 worker 线程并发轮询各子上下文，直到 全部成功 / 任一失败 / 整体超时 / 被取消。
//   4. 限制：同一复合动作中若包含多个同一种动作 Server 的子动作（如两个独立 move_pose），
//      Action Server 单执行语义会拒绝后到 goal（此时该子上下文会置 FAILURE，整体判失败）；
//      典型双臂+腿场景建议使用双臂 ARM_BOTH 单 goal + 腿部/头部 move_joints（不同 Server）并行。

#include "task_manager/task_action_client.hpp"
#include "task_manager/log_helper.hpp"
#include <algorithm>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

// 使用 com_interfaces 中的 Action 定义
#include "com_interfaces/action/move_pose.hpp"
#include "com_interfaces/action/move_joints.hpp"

namespace task_act
{

// ============================================================================
// call_compound_move - 复合动作入口（启动后台编排线程）
// ============================================================================
ActionCallResult TaskActionClient::call_compound_move(
    const std::string& task_id, const task_cfg::CompoundMoveParam& param,
    const std::string& state_key)
{
  auto logger = node_->get_logger();

  if (task_id.empty())
  {
    LOG_ERROR(logger, task_log::Tag::ACTION, "task_id is empty (move_compound)");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }
  if (param.sub_actions.empty())
  {
    LOG_ERROR(logger, task_log::Tag::ACTION,
      "move_compound param invalid: sub_actions is empty [" << state_key << "]");
    return ActionCallResult::FAIL_PARAM_INVALID;
  }

  auto& ctx = get_ctx(state_key);
  ctx.state = ActionRunState::RUNNING;
  ctx.call_result = ActionCallResult::SUCCESS;
  ctx.execution_time_ms = static_cast<uint32_t>(param.timeout_ms > 0 ? param.timeout_ms : 30000);

  // 清除可能残留的取消标志，启动后台编排线程（参数按值拷贝进线程，避免引用悬垂）
  compound_move_cancel_.store(false, std::memory_order_release);

  LOG_INFO_TITLE(logger, task_log::Tag::ACTION,
    "===== move_compound Start [" << state_key << "]: " << param.sub_actions.size()
    << " sub-action(s), timeout=" << ctx.execution_time_ms << " ms =====");

  std::thread([this, task_id, param, state_key]()
  {
    compound_move_worker(task_id, param, state_key);
  }).detach();

  return ActionCallResult::SUCCESS;
}


// ============================================================================
// compound_move_worker - 复合动作后台编排线程主体
//   流程：对每个子动作异步发送 goal（各自子 ctx）→ 并发等待全部子 ctx 到达终态
// ============================================================================
void TaskActionClient::compound_move_worker(
    const std::string& task_id, const task_cfg::CompoundMoveParam& param,
    const std::string& state_key)
{
  auto logger = node_->get_logger();
  const auto& subs = param.sub_actions;

  // 汇报最终结果（若已被打断则不再触碰主上下文，避免干扰打断后的新任务）
  auto finish = [&](bool ok, ActionCallResult cr)
  {
    if (compound_move_cancel_.load(std::memory_order_acquire))
    {
      LOG_WARN(logger, task_log::Tag::ACTION,
        "move_compound aborted by cancel flag, skip final report [" << state_key << "]");
      return;
    }
    auto& ctx = get_ctx(state_key);
    ctx.state = ok ? ActionRunState::SUCCESS : ActionRunState::FAILURE;
    ctx.call_result = cr;
    complete_action_context(state_key, ctx.state, cr);
    finish_action_timing(state_key, ok ? "SUCCESS" : "FAILURE");
    LOG_INFO(logger, task_log::Tag::ACTION,
      "move_compound finished: " << (ok ? "SUCCESS" : "FAILURE")
      << " (" << action_call_result_to_str(cr) << ") [" << state_key << "]");
  };

  // Cancel all children of this compound action without touching other branches.
  auto cancel_pending_goals = [this, state_key]()
  {
    cancel_action_goals(state_key + "_sub_");
  };

  // ========== 1. 对每个子动作异步发送 goal（各 Server 独立，天然并行）==========
  for (size_t i = 0; i < subs.size(); ++i)
  {
    const auto& sub = subs[i];
    const std::string sub_key = state_key + "_sub_" + std::to_string(i);
    auto& sctx = get_ctx(sub_key);
    sctx.state = ActionRunState::RUNNING;
    sctx.call_result = ActionCallResult::SUCCESS;
    sctx.execution_time_ms = static_cast<uint32_t>(param.timeout_ms > 0 ? param.timeout_ms : 30000);

    if (sub.action_type == "move_pose" || sub.action_type == "move_arm")
    {
      // 目标位姿注入：与普通 move_pose dispatch 完全一致——优先使用同序列 vision 结果，
      // 否则回退到成员 pose/cloud（use_config_pose=true 时由 call_move_pose 内部取配置位姿）
      geometry_msgs::msg::PoseStamped target_pose = pose;
      sensor_msgs::msg::PointCloud2 env_cloud = cloud;
      std::string cloud_frame = "base_link";
      bool has_vision_target = false;
      const std::string seq_key = derive_seq_key(state_key);
      if (!seq_key.empty())
      {
        VisionDet::Result vis_res;
        if (get_vision_result_from_seq_key(seq_key, vis_res))
        {
          LOG_INFO(logger, task_log::Tag::ACTION,
            "move_compound sub #" << i << ": using seq-level vision result from [" << seq_key << "]");
          if (!vis_res.detected_targets.empty())
          {
            target_pose = vis_res.detected_targets[0].pose;
            has_vision_target = vis_res.success;
            cloud_frame = target_pose.header.frame_id;
          }
        }
      }

      ActionCallResult r = sub.move_pose_param.vision_pose_offset.enabled && !has_vision_target ?
          ActionCallResult::FAIL_PARAM_INVALID : call_move_pose(task_id, sub.move_pose_param,
          target_pose, env_cloud, cloud_frame, sub_key);
      if (action_call_result_is_ok(r))
      {
        LOG_INFO(logger, task_log::Tag::ACTION,
          "move_compound sub #" << i << " [" << sub.action_type
          << "] arm_id=" << static_cast<int>(sub.move_pose_param.arm_id)
          << " goal sent [" << sub_key << "]");
      }
      else
      {
        LOG_ERROR(logger, task_log::Tag::ACTION,
          "move_compound sub #" << i << " [" << sub.action_type
          << "] send failed: " << action_call_result_to_str(r));
        sctx.state = ActionRunState::FAILURE;
        sctx.call_result = r;
      }
    }
    else if (sub.action_type == "move_joints")
    {
      ActionCallResult r = call_move_joints(task_id, sub.move_joints_param, sub_key);
      if (action_call_result_is_ok(r))
      {
        LOG_INFO(logger, task_log::Tag::ACTION,
          "move_compound sub #" << i << " [move_joints] arm_id="
          << static_cast<int>(sub.move_joints_param.arm_id)
          << " joints=" << sub.move_joints_param.joint_name.size()
          << " goal sent [" << sub_key << "]");
      }
      else
      {
        LOG_ERROR(logger, task_log::Tag::ACTION,
          "move_compound sub #" << i << " [move_joints] send failed: "
          << action_call_result_to_str(r));
        sctx.state = ActionRunState::FAILURE;
        sctx.call_result = r;
      }
    }
    else
    {
      LOG_ERROR(logger, task_log::Tag::ACTION,
        "move_compound sub #" << i << " unsupported sub action_type: " << sub.action_type);
      sctx.state = ActionRunState::FAILURE;
      sctx.call_result = ActionCallResult::FAIL_UNKNOWN_TYPE;
    }
  }

  // ========== 2. 并发等待全部子动作完成（全部成功才成功；任一失败/超时即判失败）==========
  const uint32_t timeout_ms = static_cast<uint32_t>(param.timeout_ms > 0 ? param.timeout_ms : 30000);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (rclcpp::ok())
  {
    // 被打断（急停 / 复位 / 新任务抢占）：通知后台线程退出
    if (compound_move_cancel_.load(std::memory_order_acquire))
    {
      LOG_WARN(logger, task_log::Tag::ACTION,
        "move_compound interrupted, stop waiting [" << state_key << "]");
      return;
    }

    bool all_success = true;
    bool any_failure = false;
    for (size_t i = 0; i < subs.size(); ++i)
    {
      const ActionRunState st = get_ctx(state_key + "_sub_" + std::to_string(i)).state;
      if (st == ActionRunState::FAILURE)
      {
        any_failure = true;
        break;
      }
      if (st != ActionRunState::SUCCESS)
      {
        all_success = false;
      }
    }
    if (any_failure)
    {
      LOG_ERROR(logger, task_log::Tag::ACTION,
        "move_compound sub action failed, cancelling pending goals [" << state_key << "]");
      cancel_pending_goals();
      finish(false, ActionCallResult::FAIL_GENERAL);
      return;
    }
    if (all_success)
    {
      finish(true, ActionCallResult::SUCCESS);
      return;
    }
    if (std::chrono::steady_clock::now() > deadline)
    {
      LOG_ERROR(logger, task_log::Tag::ACTION,
        "move_compound wait timeout (" << timeout_ms << " ms), cancelling pending goals [" << state_key << "]");
      cancel_pending_goals();
      finish(false, ActionCallResult::FAIL_TIMEOUT);
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  LOG_WARN(logger, task_log::Tag::ACTION,
    "rclcpp::ok() false, stop move_compound worker [" << state_key << "]");
}

} // namespace task_act
