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
  // 视觉标定控制客户端（/vision/calibration_control）
  calibration_client_ = rclcpp_action::create_client<CalibrationControl>(node_, "/vision/calibration_control");
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

    // ========== 执行前延迟：解析 delay_ms 参数，若大于 0 则先延迟再执行 ==========
    if (act.delay_ms > 0)
    {
        LOG_INFO(node_->get_logger(), task_log::Tag::ACTION,
            log_prefix << "Action [" << act_type << "] will delay " << act.delay_ms
            << " ms before execution");
        std::this_thread::sleep_for(std::chrono::milliseconds(act.delay_ms));
    }

    if (act_type == "vision_detect")
    {
        ctx.state = ActionRunState::RUNNING;
        ctx.execution_time_ms = 60000;
        ActionCallResult res = call_vision_detect(task_id, act.vision_param, state_key);
        if (action_call_result_is_ok(res))
        {
            LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal vision_detect: " << action_call_result_to_str(res));
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
                if (!vis_res.detected_targets.empty())
                {
                    target_pose = vis_res.detected_targets[0].pose;
                    cloud_frame = target_pose.header.frame_id;
                }
                // 新接口 DetectionPoseCloud 不再输出环境点云，env_cloud 保持为空
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
            return ActionCallResult::SUCCESS;
        }
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = res;
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal control_reset failed: " << action_call_result_to_str(res));
        return res;
    }
    else if (act_type == "calibration_control")
    {
        ctx.state = ActionRunState::RUNNING;
        ctx.execution_time_ms = static_cast<uint32_t>(act.calibration_param.timeout_ms > 0 ?
            act.calibration_param.timeout_ms : 60000);
        ActionCallResult res = call_calibration_control(task_id, act.calibration_param, state_key);
        if (action_call_result_is_ok(res))
        {
            LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal calibration_control: " << action_call_result_to_str(res));
            return ActionCallResult::SUCCESS;
        }
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = res;
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal calibration_control failed: " << action_call_result_to_str(res));
        return res;
    }
    else if (act_type == "auto_calibrate")
    {
        ctx.state = ActionRunState::RUNNING;
        ctx.execution_time_ms = static_cast<uint32_t>(act.auto_calibrate_param.timeout_ms > 0 ?
            act.auto_calibrate_param.timeout_ms : 600000);
        // call_auto_calibrate 启动后台编排线程后立即返回（最终结果写入 ctx.state）
        ActionCallResult res = call_auto_calibrate(task_id, act.auto_calibrate_param, state_key);
        if (action_call_result_is_ok(res))
        {
            LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal auto_calibrate: " << action_call_result_to_str(res));
            return ActionCallResult::SUCCESS;
        }
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = res;
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal auto_calibrate failed: " << action_call_result_to_str(res));
        return res;
    }
    else if (act_type == "move_compound")
    {
        ctx.state = ActionRunState::RUNNING;
        ctx.execution_time_ms = static_cast<uint32_t>(act.compound_move_param.timeout_ms > 0 ?
            act.compound_move_param.timeout_ms : 30000);
        // call_compound_move 启动后台编排线程后立即返回（最终结果写入 ctx.state）
        ActionCallResult res = call_compound_move(task_id, act.compound_move_param, state_key);
        if (action_call_result_is_ok(res))
        {
            LOG_INFO(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal move_compound: " << action_call_result_to_str(res));
            return ActionCallResult::SUCCESS;
        }
        ctx.state = ActionRunState::FAILURE;
        ctx.call_result = res;
        LOG_ERROR(node_->get_logger(), task_log::Tag::ACTION, log_prefix << "send_goal move_compound failed: " << action_call_result_to_str(res));
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

    // 清空所有活动 goal handle（防止服务器无响应/崩溃等极端情况下残留，
    {
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        active_vision_goal_.clear();
        active_arm_goal_.clear();
        active_move_joints_goal_.clear();
        active_gripper_goal_.clear();
        active_suck_goal_.clear();
        active_vision_check_goal_.clear();
        active_control_check_goal_.clear();
        active_vision_reset_goal_.clear();
        active_control_reset_goal_.clear();
        active_calibration_goal_.clear();
    }

    // 通知后台自动标定编排线程退出（复位/新任务抢占场景）
    auto_calibrate_cancel_.store(true, std::memory_order_release);
    // 通知后台复合动作编排线程退出（复位/新任务抢占场景）
    compound_move_cancel_.store(true, std::memory_order_release);
}


void TaskActionClient::cancel_all_actions()
{
    auto_calibrate_cancel_.store(true, std::memory_order_release);
    compound_move_cancel_.store(true, std::memory_order_release);
    cancel_action_goals("");
}

void TaskActionClient::cancel_action_goals(const std::string& state_key_prefix)
{
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    auto cancel_matching = [&state_key_prefix](auto client, auto& goals)
    {
        for (auto it = goals.begin(); it != goals.end(); )
        {
            if (!state_key_prefix.empty() && it->first.compare(0, state_key_prefix.size(), state_key_prefix) != 0)
            {
                ++it;
                continue;
            }
            const auto& goal = it->second;
            if (goal && (goal->get_status() == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
                         goal->get_status() == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
                         goal->get_status() == action_msgs::msg::GoalStatus::STATUS_CANCELING))
            {
                client->async_cancel_goal(goal);
            }
            it = goals.erase(it);
        }
    };
    cancel_matching(arm_client_, active_arm_goal_);
    cancel_matching(move_joints_client_, active_move_joints_goal_);
    cancel_matching(gripper_client_, active_gripper_goal_);
    cancel_matching(control_suck_client_, active_suck_goal_);
    cancel_matching(vision_client_, active_vision_goal_);
    cancel_matching(hw_check_client_, active_vision_check_goal_);
    cancel_matching(hw_reset_client_, active_vision_reset_goal_);
    cancel_matching(control_check_client_, active_control_check_goal_);
    cancel_matching(control_reset_client_, active_control_reset_goal_);
    cancel_matching(calibration_client_, active_calibration_goal_);
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
        << "], targets=" << result.total_detected_count);
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
        // arm_id 使用 com_interfaces 命名常量: ARM_LEFT=0, ARM_RIGHT=1, ARM_BOTH=2, HEAD=3, LEG=4
        const auto& aid = act.arm_param.arm_id;
        if (aid == MovePose::Goal::ARM_LEFT || aid == MovePose::Goal::ARM_BOTH || aid == MovePose::Goal::WHOLE_BODY_LEFT || aid == MovePose::Goal::WHOLE_BODY_BOTH)
        {
            resources.push_back(ResourceType::LEFT_ARM);
        }
        if (aid == MovePose::Goal::ARM_RIGHT || aid == MovePose::Goal::ARM_BOTH || aid == MovePose::Goal::WHOLE_BODY_RIGHT || aid == MovePose::Goal::WHOLE_BODY_BOTH)
        {
            resources.push_back(ResourceType::RIGHT_ARM);
        }
        if (aid == MovePose::Goal::HEAD)
        {
            resources.push_back(ResourceType::HEAD);
        }
        // LEG 统一表示腿 / 腰部（腰部关节 16-18 由 move_joints 以 arm_id=LEG 派发）
        if (aid == MovePose::Goal::LEG || aid == MovePose::Goal::WHOLE_BODY_LEFT || aid == MovePose::Goal::WHOLE_BODY_RIGHT || aid == MovePose::Goal::WHOLE_BODY_BOTH)
        {
            resources.push_back(ResourceType::LEG);
        }
        // MovePose.hand_action_type 非空 → 追加灵巧手资源（与手臂侧对应）
        if (!act.arm_param.hand_action_type.empty())
        {
            if (aid == MovePose::Goal::ARM_LEFT || aid == MovePose::Goal::ARM_BOTH || aid == MovePose::Goal::WHOLE_BODY_LEFT || aid == MovePose::Goal::WHOLE_BODY_BOTH)
            {
                resources.push_back(ResourceType::LEFT_HAND);
            }
            if (aid == MovePose::Goal::ARM_RIGHT || aid == MovePose::Goal::ARM_BOTH || aid == MovePose::Goal::WHOLE_BODY_RIGHT || aid == MovePose::Goal::WHOLE_BODY_BOTH)
            {
                resources.push_back(ResourceType::RIGHT_HAND);
            }
        }
    }
    else if (act_type == "move_joints")
    {
        // arm_id 使用 com_interfaces 命名常量: ARM_LEFT=0, ARM_RIGHT=1, ARM_BOTH=2, HEAD=3, LEG=4
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
        // LEG 统一表示腿 / 腰部（腿部 leg_joint1-3 + 腰部 body_joint 均以 arm_id=LEG 派发）
        if (aid == MoveJoints::Goal::LEG)
        {
            resources.push_back(ResourceType::LEG);
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
    else if (act_type == "control_suck")
    {
        // suck_id: 0=SUCK_LEFT, 1=SUCK_RIGHT, 2=SUCK_BOTH
        const auto& sid = act.suck_param.suck_id;
        if (sid == 0 || sid == 2)
        {
            resources.push_back(ResourceType::LEFT_SUCK);
        }
        if (sid == 1 || sid == 2)
        {
            resources.push_back(ResourceType::RIGHT_SUCK);
        }
    }
    else if (act_type == "calibration_control")
    {
        // 视觉标定服务占用视觉链路
        resources.push_back(ResourceType::VISION);
    }
    else if (act_type == "auto_calibrate")
    {
        // 自动标定占用视觉链路 + 标定机械臂（arm_id：0=左臂 ARM_LEFT，1=右臂 ARM_RIGHT）
        resources.push_back(ResourceType::VISION);
        if (act.auto_calibrate_param.arm_id == MovePose::Goal::ARM_RIGHT)
        {
            resources.push_back(ResourceType::RIGHT_ARM);
        }
        else
        {
            resources.push_back(ResourceType::LEFT_ARM);
        }
    }
    else if (act_type == "move_compound")
    {
        // 复合动作：聚合所有子动作占用的资源（去重，避免重复 acquire 同一把锁）
        auto add_if_missing = [&resources](ResourceType res)
        {
            for (const auto& r : resources)
            {
                if (r == res)
                {
                    return;
                }
            }
            resources.push_back(res);
        };
        for (const auto& sub : act.compound_move_param.sub_actions)
        {
            if (sub.action_type == "move_pose" || sub.action_type == "move_arm")
            {
                const auto& aid = sub.move_pose_param.arm_id;
                if (aid == MovePose::Goal::ARM_LEFT || aid == MovePose::Goal::ARM_BOTH || aid == MovePose::Goal::WHOLE_BODY_LEFT || aid == MovePose::Goal::WHOLE_BODY_BOTH)
                {
                    add_if_missing(ResourceType::LEFT_ARM);
                }
                if (aid == MovePose::Goal::ARM_RIGHT || aid == MovePose::Goal::ARM_BOTH || aid == MovePose::Goal::WHOLE_BODY_RIGHT || aid == MovePose::Goal::WHOLE_BODY_BOTH)
                {
                    add_if_missing(ResourceType::RIGHT_ARM);
                }
                if (aid == MovePose::Goal::HEAD)
                {
                    add_if_missing(ResourceType::HEAD);
                }
                if (aid == MovePose::Goal::LEG || aid == MovePose::Goal::WHOLE_BODY_LEFT || aid == MovePose::Goal::WHOLE_BODY_RIGHT || aid == MovePose::Goal::WHOLE_BODY_BOTH)
                {
                    add_if_missing(ResourceType::LEG);
                }
                if (!sub.move_pose_param.hand_action_type.empty())
                {
                    if (aid == MovePose::Goal::ARM_LEFT || aid == MovePose::Goal::ARM_BOTH || aid == MovePose::Goal::WHOLE_BODY_LEFT || aid == MovePose::Goal::WHOLE_BODY_BOTH)
                    {
                        add_if_missing(ResourceType::LEFT_HAND);
                    }
                    if (aid == MovePose::Goal::ARM_RIGHT || aid == MovePose::Goal::ARM_BOTH || aid == MovePose::Goal::WHOLE_BODY_RIGHT || aid == MovePose::Goal::WHOLE_BODY_BOTH)
                    {
                        add_if_missing(ResourceType::RIGHT_HAND);
                    }
                }
            }
            else if (sub.action_type == "move_joints")
            {
                const auto& aid = sub.move_joints_param.arm_id;
                if (aid == MoveJoints::Goal::ARM_LEFT || aid == MoveJoints::Goal::ARM_BOTH)
                {
                    add_if_missing(ResourceType::LEFT_ARM);
                }
                if (aid == MoveJoints::Goal::ARM_RIGHT || aid == MoveJoints::Goal::ARM_BOTH)
                {
                    add_if_missing(ResourceType::RIGHT_ARM);
                }
                if (aid == MoveJoints::Goal::HEAD)
                {
                    add_if_missing(ResourceType::HEAD);
                }
                if (aid == MoveJoints::Goal::LEG)
                {
                    add_if_missing(ResourceType::LEG);
                }
            }
        }
    }
    return resources;
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

} // namespace task_act
