#ifndef TASK_ACTION_CLIENT_HPP_
#define TASK_ACTION_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <string>

#include "task_config_parser.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// 你的 Action 消息头
//#include "task_manager/action/vision_detection.hpp"
#include "vision_detection_action/action/vision_detection.hpp"

#include "motion_control_pkg/action/control_gripper.hpp"
#include "motion_control_pkg/action/move_arm.hpp"


namespace task_act
{
using VisionDet = vision_detection_action::action::VisionDetection;
using MoveArm = motion_control_pkg::action::MoveArm;
using CtrlGripper = motion_control_pkg::action::ControlGripper;
using GoalHandleVisionDetection = rclcpp_action::ClientGoalHandle<VisionDet>;
using GoalHandleMoveArm = rclcpp_action::ClientGoalHandle<MoveArm>;
using GoalHandleCtrlGripper = rclcpp_action::ClientGoalHandle<CtrlGripper>;
// 动作全局状态枚举
enum class ActionRunState
{
    IDLE,       // 空闲
    RUNNING,    // 执行中
    SUCCESS,    // 执行成功
    FAILURE     // 执行失败
};

enum class ArmAndGripperSide
{
    SIDE_LEFT  = 0,
    SIDE_RIGHT = 1
};

class TaskActionClient
{
public:
    explicit TaskActionClient(rclcpp::Node::SharedPtr node);

    // 原有接口：单独调用各类 Action
    bool call_vision_detect(
        const std::string& task_id, const task_cfg::VisionActionParam& param);

    bool call_move_arm(
        const std::string& task_id, const task_cfg::MoveArmParam& param,
        const geometry_msgs::msg::PoseStamped& target_pose,
        const sensor_msgs::msg::PointCloud2& cloud, const std::string& frame_id);

    bool call_gripper(
        const std::string& task_id, const task_cfg::GripperParam& param);

    // 统一入口：根据 AtomicAction 分发执行
    bool send_goal(const task_cfg::AtomicAction& act);
    void StartTimer(uint32_t ms);
    void StopTimer();
    // ========= 新增：状态查询接口（给行为树使用） =========
    ActionRunState get_action_state() const { return current_state_; }
    bool is_action_running() const  { return current_state_ == ActionRunState::RUNNING; }
    bool is_action_success() const  { return current_state_ == ActionRunState::SUCCESS; }
    bool is_failure() const  { return current_state_ == ActionRunState::FAILURE; }
    bool is_action_idle() const  { return current_state_ == ActionRunState::IDLE; }
    void reset_state()       { current_state_ = ActionRunState::IDLE; }
    void print_vision_result(rclcpp::Logger logger, const VisionDet::Result & res);
    rclcpp::Time get_ros_now_time() const;
    rclcpp::Time get_task_start_time() const;
    uint32_t get_task_execution_time() const;
    void action_time_out_cb();
private:
    rclcpp::Node::SharedPtr node_;
    std::shared_future<std::shared_ptr<GoalHandleCtrlGripper> >  gripper_send_future_;
    //std::future<std::shared_ptr<rclcpp_action::ClientGoalHandle<motion_control_pkg::action::ControlGripper>>> gripper_send_future_;
    //std::future<typename rclcpp_action::ClientGoalHandle<CtrlGripper>::SharedPtr> gripper_send_future_;
    void goal_response_callback(GoalHandleVisionDetection::SharedPtr goal_handle);
    void feedback_callback(GoalHandleVisionDetection::SharedPtr, const std::shared_ptr<const VisionDet::Feedback> feedback);
    void result_callback(const GoalHandleVisionDetection::WrappedResult & result);

    void arm_goal_response_callback(GoalHandleMoveArm::SharedPtr goal_handle);
    void arm_feedback_callback(GoalHandleMoveArm::SharedPtr, const std::shared_ptr<const MoveArm::Feedback> feedback);
    void arm_result_callback(const GoalHandleMoveArm::WrappedResult & result);


    void gripper_goal_response_callback(GoalHandleCtrlGripper::SharedPtr goal_handle);
    void gripper_feedback_callback(GoalHandleCtrlGripper::SharedPtr, const std::shared_ptr<const GoalHandleCtrlGripper::Feedback> feedback);
    void gripper_result_callback(const GoalHandleCtrlGripper::WrappedResult & result);
    // 三类 Action 客户端
    rclcpp_action::Client<VisionDet>::SharedPtr vision_client_;
    rclcpp_action::Client<MoveArm>::SharedPtr arm_client_;
    rclcpp_action::Client<CtrlGripper>::SharedPtr gripper_client_;

    // 动作执行超时定时器
    //rclcpp::TimerBase::SharedPtr action_time_out_timer_;

    rclcpp::Time task_start_time_{0, 0};   // 任务启动时间戳
    uint32_t execution_time_;         // 任务执行时间
    // ========= 新增：全局动作状态 =========
    ActionRunState current_state_;
    
    geometry_msgs::msg::PoseStamped pose;
    sensor_msgs::msg::PointCloud2 cloud;
};

} // namespace task_act

#endif // TASK_ACTION_CLIENT_HPP_