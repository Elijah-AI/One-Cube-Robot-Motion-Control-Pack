#ifndef MOTION_CONTROL_PKG_WHOLE_BODY_MODULE_HPP_
#define MOTION_CONTROL_PKG_WHOLE_BODY_MODULE_HPP_

#include <Eigen/Geometry>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "dual_arm_module.hpp"
#include "main_node.hpp"
#include "miracle_lower_body_module.hpp"
#include "miracle_m_algo.hpp"
#include "motion_control_pkg/execution/action_terminal_guard.hpp"
#include "motion_control_pkg/execution/execution_control.hpp"

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
        Eigen::Isometry3d right_model_end_to_tool_tcp);

    ~WholeBodyModule();

    bool is_idle() const noexcept { return !is_executing_.load(); }

    void start_execution_move_pose(
        std::shared_ptr<GoalHandleMovePose> goal_handle);

private:
    void execution_loop(std::shared_ptr<GoalHandleMovePose> goal_handle);

    bool normalize_target(
        const geometry_msgs::msg::PoseStamped& input,
        const std::vector<double>& full_q,
        geometry_msgs::msg::PoseStamped& output,
        std::string& error_message) const;

    bool plan_trajectory(
        const std::shared_ptr<const MovePose::Goal>& goal,
        const std::vector<double>& start_q,
        std::vector<std::vector<double>>& left_path,
        std::vector<std::vector<double>>& right_path,
        std::vector<std::vector<double>>& leg_path,
        std::string& error_message,
        std::optional<Eigen::Isometry3d>& left_target_tcp,
        std::optional<Eigen::Isometry3d>& right_target_tcp) const;

    rclcpp::Node::WeakPtr node_;
    std::shared_ptr<MiracleMAlgo> miracle_algo_;
    std::shared_ptr<DualArmModule> dual_arm_module_;
    std::shared_ptr<miracle::MiracleLowerBodyModule> lower_body_module_;
    std::string planning_frame_;
    Eigen::Isometry3d left_model_end_to_tool_tcp_;
    Eigen::Isometry3d right_model_end_to_tool_tcp_;

    std::thread execution_thread_;
    std::atomic<bool> is_executing_{false};
};

#endif  // MOTION_CONTROL_PKG_WHOLE_BODY_MODULE_HPP_
