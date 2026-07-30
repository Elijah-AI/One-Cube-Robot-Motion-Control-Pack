#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "main_node.hpp"
#include "dual_arm_module.hpp"
#include "miracle_m_algo.hpp"
#include "motion_control_pkg/execution/arm_action_utils.hpp"
#include "motion_control_pkg/execution/dual_arm_task_arbiter.hpp"
#include "motion_control_pkg/kinematics/transform_utils.hpp"
#include "motion_control_pkg/motion/joint_space_control.hpp"
#include "motion_control_pkg/sdk/marvin_dual_arm_sdk_adapter.hpp"


using namespace miracle;

class RobotControlNode : public rclcpp::Node {
public:
    RobotControlNode() : Node("robot_control_node") {}

    void init() {
        // 机器人模型类初始化
        std::string urdf_path = "onecube_rotbot.urdf";
        std::string mesh_dir =  "package";
        this->declare_parameter("urdf_path", "onecube_rotbot.urdf");
        this->declare_parameter("mesh_dir", "package");

        if(this->get_parameter("urdf_path", urdf_path)) {
            RCLCPP_INFO(rclcpp::get_logger("robot_control_node"), "Param urdf_path is: %s !", urdf_path.c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("robot_control_node"),"Failed to get parameter urdf_path");
        }
        if(this->get_parameter("mesh_dir", mesh_dir)) {
            RCLCPP_INFO(rclcpp::get_logger("robot_control_node"), "Param mesh_dir is: %s !", mesh_dir.c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("robot_control_node"),"Failed to get parameter mesh_dir");
        }

        miracleAlgo_ptr_ = std::make_shared<MiracleMAlgo>(urdf_path, mesh_dir);

        std::string planning_frame = "base_link";
        double tf_lookup_timeout_sec = 0.2;
        std::string left_arm_tool = "gripper";
        std::string right_arm_tool = "suck";
        std::vector<double> left_arm_tool_tf{
            0.0, 0.0, 0.1785, 0.0, 0.0, 0.0, 1.0};
        std::vector<double> right_arm_tool_tf{
            0.0, 0.0, 0.1785, 0.0, 0.0, 0.0, 1.0};
        this->declare_parameter("dual_arm_planning_frame", planning_frame);
        this->declare_parameter(
            "tf_lookup_timeout_sec", tf_lookup_timeout_sec);
        this->declare_parameter("left_arm_tool", left_arm_tool);
        this->declare_parameter("right_arm_tool", right_arm_tool);
        this->declare_parameter("left_arm_tool_tf", left_arm_tool_tf);
        this->declare_parameter("right_arm_tool_tf", right_arm_tool_tf);
        this->get_parameter("dual_arm_planning_frame", planning_frame);
        this->get_parameter("tf_lookup_timeout_sec", tf_lookup_timeout_sec);
        this->get_parameter("left_arm_tool", left_arm_tool);
        this->get_parameter("right_arm_tool", right_arm_tool);
        this->get_parameter("left_arm_tool_tf", left_arm_tool_tf);
        this->get_parameter("right_arm_tool_tf", right_arm_tool_tf);

        if (planning_frame.empty() ||
            !miracleAlgo_ptr_->getModel().existFrame(planning_frame)) {
            throw std::invalid_argument(
                "dual_arm_planning_frame must name a frame in the robot model");
        }
        if (!std::isfinite(tf_lookup_timeout_sec) ||
            tf_lookup_timeout_sec < 0.0) {
            throw std::invalid_argument(
                "tf_lookup_timeout_sec must be finite and non-negative");
        }
        const auto left_tool_transform =
            motion_control_pkg::kinematics::parseToolTransform(
            left_arm_tool_tf, "left_arm_tool_tf");
        if (!left_tool_transform.success) {
            throw std::invalid_argument(left_tool_transform.error_message);
        }
        const auto right_tool_transform =
            motion_control_pkg::kinematics::parseToolTransform(
            right_arm_tool_tf, "right_arm_tool_tf");
        if (!right_tool_transform.success) {
            throw std::invalid_argument(right_tool_transform.error_message);
        }

        // 沿用总节点负责统一入口的现有分层：这里只补入唯一 TF2
        // Buffer/Listener；双臂模块只接收已转换到 planning_frame 的目标。
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_->setUsingDedicatedThread(true);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
            *tf_buffer_, shared_from_this(), true);
        pose_frame_transformer_ = std::make_unique<
            motion_control_pkg::kinematics::PoseFrameTransformer>(
            *tf_buffer_, planning_frame, tf_lookup_timeout_sec);

        // 双臂 SDK 与整机算法对象由主节点统一创建并注入，模块构造阶段不连接硬件。
        std::string active_arm_configuration = "dual_arm";
        this->declare_parameter(
            "active_arm_configuration", active_arm_configuration);
        this->get_parameter(
            "active_arm_configuration", active_arm_configuration);
        const auto parsed_arm_configuration =
            motion_control_pkg::sdk::parseActiveArmConfiguration(
            active_arm_configuration);
        if (!parsed_arm_configuration) {
            throw std::invalid_argument(
                "active_arm_configuration must be dual_arm; single-arm-only "
                "modes have been removed");
        }
        active_arm_configuration_ = *parsed_arm_configuration;
        task_arbiter_ = std::make_unique<
            motion_control_pkg::execution::DualArmTaskArbiter>(
            active_arm_configuration_);

        const std::string sdk_config_path =
            ament_index_cpp::get_package_share_directory("motion_control_pkg") +
            "/config/ccs_m6_40.MvKDCfg";
        dual_arm_device_ =
            std::make_shared<
            motion_control_pkg::sdk::MarvinDualArmSdkAdapter>();
        dual_arm_module_ = std::make_shared<DualArmModule>(
            "dual_arm", shared_from_this(), dual_arm_device_,
            miracleAlgo_ptr_, active_arm_configuration_, sdk_config_path,
            planning_frame, left_tool_transform.transform,
            right_tool_transform.transform);

        std::string dual_arm_initialization_error;
        if (!dual_arm_module_->initialize(dual_arm_initialization_error)) {
            throw std::runtime_error(
                "Dual arm initialization failed: " +
                dual_arm_initialization_error);
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Dual arm initialized with configuration=%s, planning_frame=%s, "
            "left_tool=%s, right_tool=%s and SDK config=%s",
            active_arm_configuration.c_str(), planning_frame.c_str(),
            left_arm_tool.c_str(), right_arm_tool.c_str(),
            sdk_config_path.c_str());

        // 2. 创建全局唯一的 Action Server
        move_pose_action_server_ = rclcpp_action::create_server<MovePose>(
            this,
            "/Control/move_pose",
            std::bind(&RobotControlNode::handle_goal_move_pose, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RobotControlNode::handle_cancel_move_pose, this, std::placeholders::_1),
            std::bind(&RobotControlNode::handle_accepted_move_pose, this, std::placeholders::_1)
        );
    
        move_joints_action_server_ = rclcpp_action::create_server<MoveJoints>(
            this,
            "/Control/move_joints",
            std::bind(&RobotControlNode::handle_goal_move_joints, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RobotControlNode::handle_cancel_move_joints, this, std::placeholders::_1),
            std::bind(&RobotControlNode::handle_accepted_move_joints, this, std::placeholders::_1)
        );

        control_gripper_action_server_ = rclcpp_action::create_server<ControlGripper>(
            this,
            "/Control/control_gripper",
            std::bind(&RobotControlNode::handle_goal_control_gripper, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RobotControlNode::handle_cancel_control_gripper, this, std::placeholders::_1),
            std::bind(&RobotControlNode::handle_accepted_control_gripper, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Robot Control Action Server is ready.");
    }

private:
    std::shared_ptr<MiracleMAlgo> miracleAlgo_ptr_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<motion_control_pkg::kinematics::PoseFrameTransformer>
    pose_frame_transformer_;
    std::shared_ptr<motion_control_pkg::sdk::DualArmDeviceInterface>
    dual_arm_device_;
    motion_control_pkg::sdk::ActiveArmConfiguration
    active_arm_configuration_{
        motion_control_pkg::sdk::ActiveArmConfiguration::kDualArm};
    std::unique_ptr<motion_control_pkg::execution::DualArmTaskArbiter>
    task_arbiter_;

    std::shared_ptr<DualArmModule> dual_arm_module_;
    rclcpp_action::Server<MovePose>::SharedPtr move_pose_action_server_;
    rclcpp_action::Server<MoveJoints>::SharedPtr move_joints_action_server_;
    rclcpp_action::Server<ControlGripper>::SharedPtr control_gripper_action_server_;

    // ================== 核心分发逻辑 ==================

    rclcpp_action::GoalResponse handle_goal_move_pose(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MovePose::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received movement request for part_id: %u", goal->arm_id);

        if (goal->arm_id == ARM_LEFT || goal->arm_id == ARM_RIGHT) {
            const auto pose_validation =
                motion_control_pkg::kinematics::validateAndNormalizePoseStamped(
                goal->target_pose);
            if (!pose_validation.success) {
                RCLCPP_WARN(
                    this->get_logger(), "%s",
                    pose_validation.error_message.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
            const auto speed_validation =
                motion_control_pkg::motion::validateSpeedScale(
                static_cast<double>(goal->speed));
            if (!speed_validation.valid) {
                RCLCPP_WARN(
                    this->get_logger(), "%s",
                    speed_validation.error_message.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
            if (goal->use_force_control) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "MovePose force control is reserved for a later stage.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            if (!goal->hand_action_type.empty()) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "MovePose hand actions are reserved for the tool stage.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            if (!goal->bound_size.empty()) {
                if (goal->bound_size.size() != 3U ||
                    !std::all_of(
                        goal->bound_size.begin(), goal->bound_size.end(),
                        [](float value) {
                            return std::isfinite(value) && value > 0.0F;
                        }))
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "MovePose bound_size must be empty or contain three "
                        "finite positive dimensions in metres.");
                    return rclcpp_action::GoalResponse::REJECT;
                }
            }
            const auto& point_cloud = goal->env_point_cloud;
            if (!point_cloud.data.empty() || point_cloud.width != 0U ||
                point_cloud.height != 0U)
            {
                // 当前依赖中没有已接入 MiracleMAlgo/Coal 的 PointCloud2
                // 场景导入器；显式拒绝，避免把未参与碰撞检测的数据静默忽略。
                RCLCPP_WARN(
                    this->get_logger(),
                    "MovePose environment point clouds are unavailable until "
                    "an existing collision-world importer is integrated.");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        if (goal->arm_id == ARM_BOTH) {
            if (goal->synchronization_mode !=
                MovePose::Goal::SYNC_START_AND_FINISH ||
                goal->timeout_ms <= 0)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "ARM_BOTH MovePose requires SYNC_START_AND_FINISH and "
                    "a positive timeout_ms");
                return rclcpp_action::GoalResponse::REJECT;
            }
            const auto speed = motion_control_pkg::motion::validateSpeedScale(
                static_cast<double>(goal->speed));
            const auto left_pose =
                motion_control_pkg::kinematics::validateAndNormalizePoseStamped(
                goal->left_target_pose);
            const auto right_pose =
                motion_control_pkg::kinematics::validateAndNormalizePoseStamped(
                goal->right_target_pose);
            if (!speed.valid || !left_pose.success || !right_pose.success) {
                const std::string error = !speed.valid ? speed.error_message :
                    (!left_pose.success ? left_pose.error_message :
                    right_pose.error_message);
                RCLCPP_WARN(this->get_logger(), "%s", error.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
            if (goal->use_force_control || !goal->hand_action_type.empty()) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "ARM_BOTH MovePose force/tool actions are not implemented");
                return rclcpp_action::GoalResponse::REJECT;
            }
            const auto& point_cloud = goal->env_point_cloud;
            if (!point_cloud.data.empty() || point_cloud.width != 0U ||
                point_cloud.height != 0U)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "ARM_BOTH MovePose point clouds are not connected to the "
                    "collision world");
                return rclcpp_action::GoalResponse::REJECT;
            }
            const auto unsupported_bound = [](const auto& bound) {
                return std::any_of(bound.begin(), bound.end(), [](float value) {
                    return !std::isfinite(value) || value != 0.0F;
                });
            };
            if (unsupported_bound(goal->left_bound_size) ||
                unsupported_bound(goal->right_bound_size))
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "ARM_BOTH MovePose non-zero tool envelopes require an "
                    "attached collision-object interface");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        // 【路由检查】：根据指令中的参数，检查对应模块是否繁忙
        if (goal->arm_id == ARM_LEFT) {
            const auto availability = task_arbiter_->canAcquire(
                motion_control_pkg::execution::ArmTaskScope::kLeft);
            if (!availability) {
                RCLCPP_WARN(
                    this->get_logger(), "%s",
                    availability.error_message.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        } 
        else if (goal->arm_id == ARM_RIGHT) {
            const auto availability = task_arbiter_->canAcquire(
                motion_control_pkg::execution::ArmTaskScope::kRight);
            if (!availability) {
                RCLCPP_WARN(
                    this->get_logger(), "%s",
                    availability.error_message.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else if (goal->arm_id == ARM_BOTH) {
            const auto availability = task_arbiter_->canAcquire(
                motion_control_pkg::execution::ArmTaskScope::kDual);
            if (!availability) {
                RCLCPP_WARN(
                    this->get_logger(), "%s",
                    availability.error_message.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else {
            RCLCPP_WARN(
                this->get_logger(),
                "MovePose accepts ARM_LEFT, ARM_RIGHT, or ARM_BOTH only.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_move_pose(
        const std::shared_ptr<GoalHandleMovePose> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received move pose cancel request.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_move_pose(const std::shared_ptr<GoalHandleMovePose> goal_handle)
    {
        const auto goal = goal_handle->get_goal();

        // 【任务分发】：将 goal_handle 丢给对应的部件模块去执行
        if (goal->arm_id == ARM_LEFT || goal->arm_id == ARM_RIGHT) {
            const auto scope = goal->arm_id == ARM_LEFT ?
                motion_control_pkg::execution::ArmTaskScope::kLeft :
                motion_control_pkg::execution::ArmTaskScope::kRight;
            auto acquisition = task_arbiter_->tryAcquire(scope);
            if (!acquisition) {
                auto result = std::make_shared<MovePose::Result>();
                result->success = ACTION_FAIL;
                result->error_message =
                    "MovePose lost the arm arbitration race: " +
                    acquisition.error_message;
                goal_handle->abort(result);
                return;
            }
            if (goal_handle->is_canceling()) {
                auto result = std::make_shared<MovePose::Result>();
                result->success = ACTION_FAIL;
                result->error_message =
                    "MovePose goal canceled before TF transformation";
                goal_handle->canceled(result);
                return;
            }
            const auto transformed =
                pose_frame_transformer_->transformToPlanningFrame(
                goal->target_pose);
            if (!transformed.success) {
                auto result = std::make_shared<MovePose::Result>();
                result->success = ACTION_FAIL;
                result->error_message = transformed.error_message;
                RCLCPP_ERROR(
                    this->get_logger(), "%s", result->error_message.c_str());
                goal_handle->abort(result);
                return;
            }
            RCLCPP_INFO(
                this->get_logger(),
                "Dispatching dual-arm target transformed from %s to %s.",
                goal->target_pose.header.frame_id.c_str(),
                transformed.pose.header.frame_id.c_str());
            dual_arm_module_->start_execution_move_pose(
                goal_handle, transformed.pose, std::move(acquisition.lease));
        } else if (goal->arm_id == ARM_BOTH) {
            const auto request_started_at = std::chrono::steady_clock::now();
            auto acquisition = task_arbiter_->tryAcquire(
                motion_control_pkg::execution::ArmTaskScope::kDual);
            if (!acquisition) {
                auto result = std::make_shared<MovePose::Result>();
                result->success = ACTION_FAIL;
                result->error_message =
                    "ARM_BOTH MovePose lost the arbitration race: " +
                    acquisition.error_message;
                goal_handle->abort(result);
                return;
            }
            if (goal_handle->is_canceling()) {
                auto result = std::make_shared<MovePose::Result>();
                result->success = ACTION_FAIL;
                result->error_message =
                    "ARM_BOTH MovePose canceled before TF transformation";
                goal_handle->canceled(result);
                return;
            }
            const auto left = pose_frame_transformer_->transformToPlanningFrame(
                goal->left_target_pose);
            if (!left.success) {
                auto result = std::make_shared<MovePose::Result>();
                result->success = ACTION_FAIL;
                result->error_message = "left target: " + left.error_message;
                goal_handle->abort(result);
                return;
            }
            if (goal_handle->is_canceling()) {
                auto result = std::make_shared<MovePose::Result>();
                result->success = ACTION_FAIL;
                result->error_message =
                    "ARM_BOTH MovePose canceled between TF transforms";
                goal_handle->canceled(result);
                return;
            }
            const auto right = pose_frame_transformer_->transformToPlanningFrame(
                goal->right_target_pose);
            if (!right.success) {
                auto result = std::make_shared<MovePose::Result>();
                result->success = ACTION_FAIL;
                result->error_message = "right target: " + right.error_message;
                goal_handle->abort(result);
                return;
            }
            if (std::chrono::steady_clock::now() - request_started_at >=
                std::chrono::milliseconds(goal->timeout_ms))
            {
                auto result = std::make_shared<MovePose::Result>();
                result->success = ACTION_FAIL;
                result->error_message =
                    "ARM_BOTH MovePose timeout expired during TF transforms";
                result->execution_time = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - request_started_at).count();
                goal_handle->abort(result);
                return;
            }
            dual_arm_module_->start_execution_move_dual_arm_pose(
                goal_handle, left.pose, right.pose, request_started_at,
                std::move(acquisition.lease));
        }
    }

    // ==============================================================


     rclcpp_action::GoalResponse handle_goal_move_joints(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveJoints::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received move joints request for part_id: %u", goal->arm_id);

        if (goal->arm_id == ARM_LEFT || goal->arm_id == ARM_RIGHT) {
            const auto speed_validation =
                motion_control_pkg::motion::validateSpeedScale(
                static_cast<double>(goal->speed));
            if (!speed_validation.valid) {
                RCLCPP_WARN(
                    this->get_logger(), "%s",
                    speed_validation.error_message.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
            // MoveJoints 的碰撞模式尚未接入 Stage 7 的 MiracleMAlgo
            // Pinocchio/Coal/OMPL 链，不能回退到旧的恒真碰撞包装器。
            if (goal->enable_collision_check) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Collision-checked MoveJoints is not enabled before the "
                    "validated Pinocchio/OMPL collision pipeline migration.");
                return rclcpp_action::GoalResponse::REJECT;
            }
        } else if (goal->arm_id == ARM_BOTH) {
            if (goal->synchronization_mode !=
                MoveJoints::Goal::SYNC_START_AND_FINISH ||
                goal->timeout_ms <= 0)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "ARM_BOTH MoveJoints requires SYNC_START_AND_FINISH and "
                    "a positive timeout_ms");
                return rclcpp_action::GoalResponse::REJECT;
            }
            const auto speed = motion_control_pkg::motion::validateSpeedScale(
                static_cast<double>(goal->speed));
            if (!speed.valid) {
                RCLCPP_WARN(
                    this->get_logger(), "%s", speed.error_message.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
            const std::vector<std::string> left_names(
                goal->left_joint_name.begin(), goal->left_joint_name.end());
            const std::vector<double> left_positions(
                goal->left_position.begin(), goal->left_position.end());
            const std::vector<std::string> right_names(
                goal->right_joint_name.begin(), goal->right_joint_name.end());
            const std::vector<double> right_positions(
                goal->right_position.begin(), goal->right_position.end());
            const auto left_validation =
                motion_control_pkg::execution::validateAndOrderJointGoal(
                motion_control_pkg::sdk::ArmChannel::kLeft,
                left_names, left_positions);
            const auto right_validation =
                motion_control_pkg::execution::validateAndOrderJointGoal(
                motion_control_pkg::sdk::ArmChannel::kRight,
                right_names, right_positions);
            if (!left_validation.valid || !right_validation.valid) {
                const auto& error = left_validation.valid ?
                    right_validation.error_message :
                    left_validation.error_message;
                RCLCPP_WARN(this->get_logger(), "%s", error.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        // 【路由检查】：根据指令中的参数，检查对应模块是否繁忙
        if (goal->arm_id == ARM_LEFT) {
            const auto validation =
                motion_control_pkg::execution::validateAndOrderJointGoal(
                motion_control_pkg::sdk::ArmChannel::kLeft,
                goal->joint_name, goal->position);
            if (!validation.valid) {
                RCLCPP_WARN(
                    this->get_logger(), "%s",
                    validation.error_message.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
            const auto availability = task_arbiter_->canAcquire(
                motion_control_pkg::execution::ArmTaskScope::kLeft);
            if (!availability) {
                RCLCPP_WARN(
                    this->get_logger(), "%s",
                    availability.error_message.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        } 
        else if (goal->arm_id == ARM_RIGHT) {
            const auto validation =
                motion_control_pkg::execution::validateAndOrderJointGoal(
                motion_control_pkg::sdk::ArmChannel::kRight,
                goal->joint_name, goal->position);
            if (!validation.valid) {
                RCLCPP_WARN(
                    this->get_logger(), "%s",
                    validation.error_message.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
            const auto availability = task_arbiter_->canAcquire(
                motion_control_pkg::execution::ArmTaskScope::kRight);
            if (!availability) {
                RCLCPP_WARN(
                    this->get_logger(), "%s",
                    availability.error_message.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else if (goal->arm_id == ARM_BOTH) {
            const auto availability = task_arbiter_->canAcquire(
                motion_control_pkg::execution::ArmTaskScope::kDual);
            if (!availability) {
                RCLCPP_WARN(
                    this->get_logger(), "%s",
                    availability.error_message.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else {
            RCLCPP_WARN(
                this->get_logger(),
                "MoveJoints accepts ARM_LEFT, ARM_RIGHT, or ARM_BOTH only.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_move_joints(
        const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received move joints cancel request.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_move_joints(const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
    {
        const auto goal = goal_handle->get_goal();

        // 【任务分发】：将 goal_handle 丢给对应的部件模块去执行
        if (goal->arm_id == ARM_LEFT || goal->arm_id == ARM_RIGHT) {
            const auto scope = goal->arm_id == ARM_LEFT ?
                motion_control_pkg::execution::ArmTaskScope::kLeft :
                motion_control_pkg::execution::ArmTaskScope::kRight;
            auto acquisition = task_arbiter_->tryAcquire(scope);
            if (!acquisition) {
                auto result = std::make_shared<MoveJoints::Result>();
                result->success = ACTION_FAIL;
                result->error_message =
                    "MoveJoints lost the arm arbitration race: " +
                    acquisition.error_message;
                goal_handle->abort(result);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Dispatching joints task to Dual Arm.");
            dual_arm_module_->start_execution_move_joints(
                goal_handle, std::move(acquisition.lease));
        } else if (goal->arm_id == ARM_BOTH) {
            auto acquisition = task_arbiter_->tryAcquire(
                motion_control_pkg::execution::ArmTaskScope::kDual);
            if (!acquisition) {
                auto result = std::make_shared<MoveJoints::Result>();
                result->success = ACTION_FAIL;
                result->error_message =
                    "ARM_BOTH MoveJoints lost the arbitration race: " +
                    acquisition.error_message;
                goal_handle->abort(result);
                return;
            }
            dual_arm_module_->start_execution_move_dual_arm_joints(
                goal_handle, std::move(acquisition.lease));
        }
    }

    //====================================================================

    rclcpp_action::GoalResponse handle_goal_control_gripper(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ControlGripper::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received movement request for gripper_id: %u", goal->gripper_id);

        RCLCPP_WARN(
            this->get_logger(),
            "ControlGripper is fail-closed because the tool-control stage has "
            "not completed offline or real-robot validation.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    rclcpp_action::CancelResponse handle_cancel_control_gripper(
        const std::shared_ptr<GoalHandleControlGripper> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received control gripper cancel request.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_control_gripper(
        const std::shared_ptr<GoalHandleControlGripper> goal_handle)
    {
        // Goal requests are rejected above. Keep a defensive terminal path so
        // an accidental direct callback invocation can never reach legacy
        // Modbus commands before the tool-control stage is validated.
        auto result = std::make_shared<ControlGripper::Result>();
        result->success = ACTION_FAIL;
        result->error_message =
            "ControlGripper is unavailable before tool-stage validation";
        goal_handle->abort(result);
    }

};






int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControlNode>();

    //调用 init 初始化模块和 Action Server，安全传递 shared_from_this
    node->init();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
