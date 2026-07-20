#ifndef TASK_ACTION_CLIENT_HPP_
#define TASK_ACTION_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <string>
#include <map>
#include <mutex>
#include <atomic>
#include <cmath>

#include "task_config_parser.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// 使用 com_interfaces 中的 Action 定义
#include "com_interfaces/action/vision_detection.hpp"
#include "com_interfaces/action/control_gripper.hpp"
#include "com_interfaces/action/move_arm.hpp"
#include "com_interfaces/action/hardware_check.hpp"
#include "com_interfaces/action/hardware_reset.hpp"


namespace task_act
{
using VisionDet = com_interfaces::action::VisionDetection;
using MoveArm = com_interfaces::action::MoveArm;
using CtrlGripper = com_interfaces::action::ControlGripper;
using HardWareCheck = com_interfaces::action::HardwareCheck;
using HardWareReset = com_interfaces::action::HardwareReset;
using GoalHandleVisionDetection = rclcpp_action::ClientGoalHandle<VisionDet>;
using GoalHandleMoveArm = rclcpp_action::ClientGoalHandle<MoveArm>;
using GoalHandleCtrlGripper = rclcpp_action::ClientGoalHandle<CtrlGripper>;
using GoalHandleHardWareCheck = rclcpp_action::ClientGoalHandle<HardWareCheck>;
using GoalHandleHardWareReset = rclcpp_action::ClientGoalHandle<HardWareReset>;

// 动作全局状态枚举
enum class ActionRunState
{
    IDLE,       // 空闲
    RUNNING,    // 执行中
    SUCCESS,    // 执行成功
    FAILURE     // 执行失败
};

/// @brief Action 调用结果枚举
/// 替代原有的 bool 返回值，精确表达调用结果的各种情况
enum class ActionCallResult
{
    SUCCESS,                ///< 调用成功，goal 已发送
    FAIL_SERVER_BUSY,       ///< 服务端繁忙（资源被占用）
    FAIL_SERVER_TIMEOUT,    ///< 服务端连接超时
    FAIL_GOAL_REJECTED,     ///< 目标被服务端拒绝
    FAIL_PARAM_INVALID,     ///< 参数无效
    FAIL_UNKNOWN_TYPE,      ///< 未知动作类型
    FAIL_TIMEOUT,           ///< 执行超时
    FAIL_RESOURCE_BUSY,     ///< 本地资源被其他 action 占用
    FAIL_GENERAL            ///< 通用失败
};

/// @brief 将 ActionCallResult 转换为可读字符串
inline const char* action_call_result_to_str(ActionCallResult r)
{
    switch (r)
    {
        case ActionCallResult::SUCCESS:             return "SUCCESS";
        case ActionCallResult::FAIL_SERVER_BUSY:    return "FAIL_SERVER_BUSY";
        case ActionCallResult::FAIL_SERVER_TIMEOUT: return "FAIL_SERVER_TIMEOUT";
        case ActionCallResult::FAIL_GOAL_REJECTED:  return "FAIL_GOAL_REJECTED";
        case ActionCallResult::FAIL_PARAM_INVALID:  return "FAIL_PARAM_INVALID";
        case ActionCallResult::FAIL_UNKNOWN_TYPE:   return "FAIL_UNKNOWN_TYPE";
        case ActionCallResult::FAIL_TIMEOUT:        return "FAIL_TIMEOUT";
        case ActionCallResult::FAIL_RESOURCE_BUSY:  return "FAIL_RESOURCE_BUSY";
        case ActionCallResult::FAIL_GENERAL:        return "FAIL_GENERAL";
        default:                                    return "UNKNOWN";
    }
}

/// @brief 将 ActionCallResult 映射为 BTStatus 兼容的布尔语义
/// true 表示成功，false 表示失败
inline bool action_call_result_is_ok(ActionCallResult r)
{
    return r == ActionCallResult::SUCCESS;
}

enum class ArmAndGripperSide
{
    SIDE_LEFT  = 0,
    SIDE_RIGHT = 1
};

/// @brief 硬件资源类型枚举
/// 定义系统中可被互斥访问的硬件资源
enum class ResourceType
{
    VISION,         ///< 视觉相机
    LEFT_ARM,       ///< 左机械臂
    RIGHT_ARM,      ///< 右机械臂
    LEFT_GRIPPER,   ///< 左夹爪
    RIGHT_GRIPPER,  ///< 右夹爪
    COUNT           ///< 资源总数（哨兵）
};

/// @brief 将 ResourceType 转换为可读字符串
inline const char* resource_type_to_str(ResourceType r)
{
    switch (r)
    {
        case ResourceType::VISION:        return "VISION";
        case ResourceType::LEFT_ARM:      return "LEFT_ARM";
        case ResourceType::RIGHT_ARM:     return "RIGHT_ARM";
        case ResourceType::LEFT_GRIPPER:  return "LEFT_GRIPPER";
        case ResourceType::RIGHT_GRIPPER: return "RIGHT_GRIPPER";
        default:                          return "UNKNOWN";
    }
}

/// @brief 每个 action 的独立上下文（状态、启动时间、超时时间、调用结果、占用的资源）
struct ActionContext
{
    ActionRunState state{ActionRunState::IDLE};
    ActionCallResult call_result{ActionCallResult::SUCCESS};
    rclcpp::Time start_time{0, 0};
    uint32_t execution_time_ms{60000};  // 默认 60 秒超时
    std::vector<ResourceType> acquired_resources;  ///< 该 action 占用的资源列表
};

class TaskActionClient
{
public:
    explicit TaskActionClient(rclcpp::Node::SharedPtr node);

    // 原有接口：单独调用各类 Action，返回类型由 bool 改为 ActionCallResult
    // state_key 参数用于并行分支，标识该 action 所属的上下文 key，默认 "default"
    ActionCallResult call_vision_detect(
        const std::string& task_id, const task_cfg::VisionActionParam& param,
        const std::string& state_key = "default");

    ActionCallResult call_move_arm(
        const std::string& task_id, const task_cfg::MoveArmParam& param,
        const geometry_msgs::msg::PoseStamped& target_pose,
        const sensor_msgs::msg::PointCloud2& cloud, const std::string& frame_id,
        const std::string& state_key = "default");

    ActionCallResult call_gripper(
        const std::string& task_id, const task_cfg::GripperParam& param,
        const std::string& state_key = "default");
    
    ActionCallResult check_vision_self(
        const std::string& task_id, const task_cfg::GripperParam& param,
        const std::string& state_key = "default");

    // ========= HardwareCheck 相关接口 =========
    /// @brief 调用视觉自检（/vision/Checkself）
    ActionCallResult call_vision_check_self(
        const std::string& task_id, const task_cfg::VisionCheckSelfParam& param,
        const std::string& state_key = "default");

    /// @brief 调用运控自检（/Control/Checkself）
    ActionCallResult call_control_check_self(
        const std::string& task_id, const task_cfg::ControlCheckSelfParam& param,
        const std::string& state_key = "default");

    // ========= HardwareReset 相关接口 =========
    /// @brief 调用视觉复位（/vision/Reset）
    ActionCallResult call_vision_reset(
        const std::string& task_id, const task_cfg::VisionResetParam& param,
        const std::string& state_key = "default");

    /// @brief 调用运控复位（/Control/Reset）
    ActionCallResult call_control_reset(
        const std::string& task_id, const task_cfg::ControlResetParam& param,
        const std::string& state_key = "default");

    // 统一入口：根据 AtomicAction 分发执行
    ActionCallResult send_goal(const task_cfg::AtomicAction& act);
    // ========= 并行分支专用：带 state_key 的重载 =========
    /// @brief 使用 state_key 发送 goal（并行分支专用，不干扰其他 action 状态）
    ActionCallResult send_goal(const task_cfg::AtomicAction& act, const std::string& state_key);

    // ========= 基于 state_key 的状态查询接口 =========
    /// @brief 查询指定 state_key 的动作是否正在运行
    bool is_action_running(const std::string& state_key) const;
    /// @brief 查询指定 state_key 的动作是否成功
    bool is_action_success(const std::string& state_key) const;
    /// @brief 查询指定 state_key 的动作是否失败
    bool is_action_failure(const std::string& state_key) const;
    /// @brief 重置指定 state_key 的状态
    void reset_action_state(const std::string& state_key);

    /// @brief 重置所有 state_key 的状态并释放所有资源
    void reset_all_states();

    // ========= 默认（串行）状态查询接口（给行为树使用） =========
    ActionRunState get_action_state() const { return get_ctx("default").state; }
    bool is_action_running() const  { return get_ctx("default").state == ActionRunState::RUNNING; }
    bool is_action_success() const  { return get_ctx("default").state == ActionRunState::SUCCESS; }
    bool is_failure() const  { return get_ctx("default").state == ActionRunState::FAILURE; }
    bool is_action_idle() const  { return get_ctx("default").state == ActionRunState::IDLE; }
    void reset_state()       { reset_action_state("default"); }

    /// @brief 获取指定 state_key 的启动时间
    rclcpp::Time get_action_start_time(const std::string& state_key = "default") const;
    /// @brief 获取指定 state_key 的超时时间（毫秒）
    uint32_t get_action_execution_time(const std::string& state_key = "default") const;

    /// @brief 打印视觉检测结果（含原始四元数）
    void print_vision_result(rclcpp::Logger logger, const VisionDet::Result & res);

    /// @brief 打印视觉检测结果的 XYZABC 格式（四元数转欧拉角）
    /// 将 target_6d_pose 中的位置 (x,y,z) 和四元数转换为 RPY 欧拉角 (A=X轴旋转, B=Y轴旋转, C=Z轴旋转)
    void print_vision_result_xyzabc(rclcpp::Logger logger, const VisionDet::Result & res);

    /// @brief 将四元数转换为欧拉角 (roll, pitch, yaw)，单位：度
    /// @param qx, qy, qz, qw 四元数分量
    /// @param roll 输出：绕 X 轴旋转角度（度）
    /// @param pitch 输出：绕 Y 轴旋转角度（度）
    /// @param yaw 输出：绕 Z 轴旋转角度（度）
    static void quaternion_to_rpy(double qx, double qy, double qz, double qw,
                                   double& roll, double& pitch, double& yaw);

    rclcpp::Time get_ros_now_time() const;

    /// @brief 获取指定 state_key 的最近一次调用结果
    ActionCallResult get_last_call_result(const std::string& state_key = "default") const;

    // ========= 资源锁接口 =========
    /// @brief 尝试获取指定资源，返回 true 表示成功获取
    bool acquire_resource(ResourceType res);
    /// @brief 释放指定资源
    void release_resource(ResourceType res);
    /// @brief 检查指定资源是否被占用
    bool is_resource_busy(ResourceType res) const;

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_future<std::shared_ptr<GoalHandleCtrlGripper> >  gripper_send_future_;
    void goal_response_callback(GoalHandleVisionDetection::SharedPtr goal_handle);
    void feedback_callback(GoalHandleVisionDetection::SharedPtr, const std::shared_ptr<const VisionDet::Feedback> feedback);
    void result_callback(const GoalHandleVisionDetection::WrappedResult & result);

    void arm_goal_response_callback(GoalHandleMoveArm::SharedPtr goal_handle);
    void arm_feedback_callback(GoalHandleMoveArm::SharedPtr, const std::shared_ptr<const MoveArm::Feedback> feedback);
    void arm_result_callback(const GoalHandleMoveArm::WrappedResult & result);


    void gripper_goal_response_callback(GoalHandleCtrlGripper::SharedPtr goal_handle);
    void gripper_feedback_callback(GoalHandleCtrlGripper::SharedPtr, const std::shared_ptr<const GoalHandleCtrlGripper::Feedback> feedback);
    void gripper_result_callback(const GoalHandleCtrlGripper::WrappedResult & result);

    // ========= HardwareCheck 回调 =========
    void hw_check_goal_response_callback(GoalHandleHardWareCheck::SharedPtr goal_handle);
    void hw_check_feedback_callback(GoalHandleHardWareCheck::SharedPtr, const std::shared_ptr<const HardWareCheck::Feedback> feedback);
    void hw_check_result_callback(const GoalHandleHardWareCheck::WrappedResult & result);

    // ========= HardwareReset 回调 =========
    void hw_reset_goal_response_callback(GoalHandleHardWareReset::SharedPtr goal_handle);
    void hw_reset_feedback_callback(GoalHandleHardWareReset::SharedPtr, const std::shared_ptr<const HardWareReset::Feedback> feedback);
    void hw_reset_result_callback(const GoalHandleHardWareReset::WrappedResult & result);

    /// @brief 根据动作类型和参数确定所需的资源列表
    std::vector<ResourceType> get_required_resources(
        const std::string& act_type, const task_cfg::AtomicAction& act) const;

    /// @brief 释放 ActionContext 中已获取的所有资源
    /// @param ctx 动作上下文
    void release_acquired_resources(ActionContext& ctx);

    /// @brief 动作类型分发执行（send_goal 共享逻辑）
    /// @param act 原子动作
    /// @param task_id 任务 ID
    /// @param ctx 动作上下文引用
    /// @param state_key 状态 key（用于日志前缀）
    /// @return 动作调用结果
    ActionCallResult dispatch_action(
        const task_cfg::AtomicAction& act,
        const std::string& task_id,
        ActionContext& ctx,
        const std::string& state_key);

    // 五类 Action 客户端
    rclcpp_action::Client<VisionDet>::SharedPtr vision_client_;
    rclcpp_action::Client<MoveArm>::SharedPtr arm_client_;
    rclcpp_action::Client<CtrlGripper>::SharedPtr gripper_client_;
    rclcpp_action::Client<HardWareCheck>::SharedPtr hw_check_client_;
    rclcpp_action::Client<HardWareReset>::SharedPtr hw_reset_client_;
    // 运控自检/复位独立客户端（必须为成员变量，不能是局部变量，否则回调无法触发）
    rclcpp_action::Client<HardWareCheck>::SharedPtr control_check_client_;
    rclcpp_action::Client<HardWareReset>::SharedPtr control_reset_client_;

    // ========= 基于 state_key 的独立 action 上下文管理 =========
    /// @brief 获取指定 state_key 的上下文（const 版本）
    const ActionContext& get_ctx(const std::string& state_key) const;
    /// @brief 获取指定 state_key 的上下文（可变版本，自动创建）
    ActionContext& get_ctx(const std::string& state_key);

    /// @brief 所有 action 的独立上下文映射表
    mutable std::map<std::string, ActionContext> action_contexts_;
    /// @brief 上下文访问互斥锁
    mutable std::mutex ctx_mutex_;

    // ========= 资源锁状态管理 =========
    /// @brief 每个资源的占用标志（true=被占用）
    std::atomic<bool> resource_locks_[static_cast<size_t>(ResourceType::COUNT)];

    geometry_msgs::msg::PoseStamped pose;
    sensor_msgs::msg::PointCloud2 cloud;
};

} // namespace task_act

#endif // TASK_ACTION_CLIENT_HPP_
