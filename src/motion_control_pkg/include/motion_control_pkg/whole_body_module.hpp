#ifndef MOTION_CONTROL_PKG_WHOLE_BODY_MODULE_HPP_
#define MOTION_CONTROL_PKG_WHOLE_BODY_MODULE_HPP_

#include "motion_control_pkg/execution/precision_policy.hpp"
#include <Eigen/Geometry>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <sensor_msgs/msg/joint_state.hpp>

#include "dual_arm_module.hpp"
#include "main_node.hpp"
#include "miracle_lower_body_module.hpp"
#include "miracle_head_module.hpp"
#include "miracle_m_algo.hpp"
#include "motion_control_pkg/execution/action_terminal_guard.hpp"
#include "motion_control_pkg/execution/execution_control.hpp"
#include "motion_control_pkg/wbc/trajectory.hpp"
#include "motion_control_pkg/execution/whole_body_backend.hpp"

class WholeBodyModule
{
public:
    WholeBodyModule(
        const rclcpp::Node::SharedPtr& node,
        const std::shared_ptr<MiracleMAlgo>& miracle_algo,
        const std::shared_ptr<DualArmModule>& dual_arm_module,
        const std::shared_ptr<miracle::MiracleLowerBodyModule>& lower_body_module,
        std::string planning_frame,
        Eigen::Isometry3d left_model_end_to_tool_tcp,
        Eigen::Isometry3d right_model_end_to_tool_tcp,
        std::shared_ptr<motion_control_pkg::execution::WholeBodyExecutionBackend> backend = nullptr,
        std::shared_ptr<miracle::MiracleHeadModule> head = nullptr);

    ~WholeBodyModule();

    bool is_idle() const noexcept { return !is_executing_.load(); }

    void start_execution_move_pose(
        std::shared_ptr<GoalHandleMovePose> goal_handle);
    void start_execution_move_joints(std::shared_ptr<GoalHandleMoveJoints> goal_handle);

    // Read-only planning/feedback seams for hardware-free integration tests.
    // plan_trajectory never dispatches commands. Dynamic mode callers must
    // obtain complete state via read_balance_state and confirm rest using
    // check_balance_feedback(..., before_dispatch=true) before dispatch.
    bool read_balance_state(std::vector<double>& q, std::string& error) const;
    bool check_balance_feedback(const std::vector<double>& start, bool use_left,
                               bool use_right, bool before_dispatch, std::string& error) const;
private:
    motion_control_pkg::execution::PrecisionPolicy leg_precision_policy_;
    bool dynamic_zmp_enabled_{false};
    motion_control_pkg::wbc::ReferenceOptions reference_options_;
    std::unique_ptr<motion_control_pkg::wbc::WholeBodyState> balance_state_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr balance_subscription_;
    double balance_age_{.15}, balance_skew_{.06};
    template<class Action> void start_execution(std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> goal_handle);
    template<class Action> void execution_loop(std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> goal_handle);
    bool read_leg_compensation_state(std::vector<double>& q,std::string& error,
                                    const std::vector<double>* expected_start=nullptr) const;

    bool normalize_target(
        const geometry_msgs::msg::PoseStamped& input,
        const std::vector<double>& full_q,
        geometry_msgs::msg::PoseStamped& output,
        std::string& error_message) const;

public:
    bool plan_leg_trajectory(const std::shared_ptr<const MoveJoints::Goal>& goal,
        const std::vector<double>& start_q,std::vector<std::vector<double>>& left_path,
        std::vector<std::vector<double>>& right_path,std::vector<std::vector<double>>& leg_path,
        std::string& error) const;
    bool plan_trajectory(
        const std::shared_ptr<const MovePose::Goal>& goal,
        const std::vector<double>& start_q,
        std::vector<std::vector<double>>& left_path,
        std::vector<std::vector<double>>& right_path,
        std::vector<std::vector<double>>& leg_path,
        std::string& error_message,
        std::optional<Eigen::Isometry3d>& left_target_tcp,
        std::optional<Eigen::Isometry3d>& right_target_tcp,
        motion_control_pkg::wbc::ReferenceResult* reference = nullptr) const;

private:
    rclcpp::Node::WeakPtr node_;
    std::shared_ptr<MiracleMAlgo> miracle_algo_;
    std::shared_ptr<DualArmModule> dual_arm_module_;
    std::shared_ptr<miracle::MiracleLowerBodyModule> lower_body_module_;
    std::shared_ptr<motion_control_pkg::execution::WholeBodyExecutionBackend> backend_;
    std::string planning_frame_;
    Eigen::Isometry3d left_model_end_to_tool_tcp_;
    Eigen::Isometry3d right_model_end_to_tool_tcp_;

    std::thread execution_thread_;
    std::atomic<bool> is_executing_{false};
};

#endif  // MOTION_CONTROL_PKG_WHOLE_BODY_MODULE_HPP_
