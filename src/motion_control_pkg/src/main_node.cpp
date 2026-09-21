#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "main_node.hpp"
#include "dual_arm_module.hpp"
#include "ethercat_bus_manager.hpp"
#include "miracle_head_module.hpp"
#include "miracle_lower_body_module.hpp"
#include "miracle_m_algo.hpp"
#include "whole_body_module.hpp"
#include "motion_control_pkg/execution/arm_action_utils.hpp"
#include "motion_control_pkg/execution/gripper_command_validation.hpp"
#include "motion_control_pkg/execution/suck_command_validation.hpp"
#include "motion_control_pkg/kinematics/transform_utils.hpp"
#include "motion_control_pkg/motion/joint_space_control.hpp"
#include "motion_control_pkg/sdk/marvin_dual_arm_sdk_adapter.hpp"


using namespace miracle;

class RobotControlNode : public rclcpp::Node {
public:
    RobotControlNode() : Node("robot_control_node") {}

    void init() {
        // 1. 创建各个身体部位的控制模块
        // 传入 shared_from_this() 让各模块都能使用这个 Node 发日志/建定时器

        std::string etherat_bus_config = " ";
        this->declare_parameter("EtherCAT_bus_config", " ");
        
        if(this->get_parameter("EtherCAT_bus_config", etherat_bus_config)) {
            RCLCPP_INFO(rclcpp::get_logger("robot_control_node"), "Param EtherCAT_bus_config is: %s !", etherat_bus_config.c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("robot_control_node"),"Failed to get parameter EtherCAT_bus_config");
        }
        ethercat_bus_manager_ptr_ = EthercatBusManager::getOrCreate(etherat_bus_config.c_str());
        if (!ethercat_bus_manager_ptr_ || !ethercat_bus_manager_ptr_->isReady()) {
            throw std::runtime_error("EtherCAT bus manager failed its startup safety checks");
        }

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

        //创建头部模块
        head_module_ = std::make_shared<MiracleHeadModule>("head", shared_from_this());
        head_module_->initDevices(ethercat_bus_manager_ptr_);
        head_module_->initGmcaModule(miracleAlgo_ptr_);

        //创建下肢模块
        lower_body_module_ = std::make_shared<MiracleLowerBodyModule>("leg", shared_from_this());
        lower_body_module_->initDevices(ethercat_bus_manager_ptr_);
        lower_body_module_->initGmcaModule(miracleAlgo_ptr_);

        std::string planning_frame = "base_link";
        std::vector<double> left_arm_tool_tf{
            0.0, 0.0, 0.1785, 0.0, 0.0, 0.0, 1.0};
        std::vector<double> right_arm_tool_tf{
            0.0, 0.0, 0.1785, 0.0, 0.0, 0.0, 1.0};
        this->declare_parameter("dual_arm_planning_frame", planning_frame);
        this->declare_parameter("left_arm_tool", left_arm_tool_);
        this->declare_parameter("right_arm_tool", right_arm_tool_);
        this->declare_parameter("left_arm_tool_tf", left_arm_tool_tf);
        this->declare_parameter("right_arm_tool_tf", right_arm_tool_tf);
        this->get_parameter("dual_arm_planning_frame", planning_frame);
        this->get_parameter("left_arm_tool", left_arm_tool_);
        this->get_parameter("right_arm_tool", right_arm_tool_);
        this->get_parameter("left_arm_tool_tf", left_arm_tool_tf);
        this->get_parameter("right_arm_tool_tf", right_arm_tool_tf);

        const auto is_supported_tool = [](const std::string& tool) {
            return tool == "gripper" || tool == "suck" || tool == "none";
        };
        if (!is_supported_tool(left_arm_tool_) ||
            !is_supported_tool(right_arm_tool_))
        {
            throw std::invalid_argument(
                "left_arm_tool/right_arm_tool must be gripper, suck, or none");
        }

        if (planning_frame.empty() ||
            !miracleAlgo_ptr_->getModel().existFrame(planning_frame)) {
            throw std::invalid_argument(
                "dual_arm_planning_frame must name a frame in the robot model");
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

        // 双臂 SDK 与整机算法对象由主节点统一创建并注入，模块构造阶段不连接硬件。
        // 目标位姿的 frame 转换下沉到 DualArmModule 执行接口内部，通过 Pinocchio
        // FK 完成，不依赖外部 TF 发布。
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
        const std::string sdk_config_path =
            ament_index_cpp::get_package_share_directory("motion_control_pkg") +
            "/config/ccs_m6_40.MvKDCfg";
        dual_arm_device_ =
            std::make_shared<
            motion_control_pkg::sdk::MarvinDualArmSdkAdapter>();
        
        //创建双臂模块
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
        dual_arm_module_->attachMeasuredFeedbackBus(ethercat_bus_manager_ptr_);
        RCLCPP_INFO(
            this->get_logger(),
            "Dual arm initialized with configuration=%s, planning_frame=%s, "
            "left_tool=%s, right_tool=%s and SDK config=%s",
            active_arm_configuration.c_str(), planning_frame.c_str(),
            left_arm_tool_.c_str(), right_arm_tool_.c_str(),
            sdk_config_path.c_str());

        miracleAlgo_ptr_->setCollisionCloudProvider(
            [weak = std::weak_ptr<DualArmModule>(dual_arm_module_)] {
                const auto module = weak.lock();
                return module ? module->collisionPointCloud() : sensor_msgs::msg::PointCloud2{};
            });

        whole_body_module_ = std::make_shared<WholeBodyModule>(
            shared_from_this(), miracleAlgo_ptr_, dual_arm_module_,
            lower_body_module_, planning_frame, left_tool_transform.transform,
            right_tool_transform.transform);

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

        control_suck_action_server_ = rclcpp_action::create_server<ControlSuck>(
            this,
            "/Control/control_suck",
            std::bind(&RobotControlNode::handle_goal_control_suck, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RobotControlNode::handle_cancel_control_suck, this, std::placeholders::_1),
            std::bind(&RobotControlNode::handle_accepted_control_suck, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Robot Control Action Server is ready.");
    }

private:
    std::shared_ptr<MiracleMAlgo> miracleAlgo_ptr_;
    std::shared_ptr<EthercatBusManager> ethercat_bus_manager_ptr_;
    std::shared_ptr<MiracleHeadModule> head_module_;
    std::shared_ptr<MiracleLowerBodyModule> lower_body_module_;

    std::shared_ptr<motion_control_pkg::sdk::DualArmDeviceInterface>
    dual_arm_device_;
    motion_control_pkg::sdk::ActiveArmConfiguration
    active_arm_configuration_{
        motion_control_pkg::sdk::ActiveArmConfiguration::kDualArm};
    std::shared_ptr<DualArmModule> dual_arm_module_;
    std::shared_ptr<WholeBodyModule> whole_body_module_;
    std::string left_arm_tool_{"suck"};
    std::string right_arm_tool_{"gripper"};
    rclcpp_action::Server<MovePose>::SharedPtr move_pose_action_server_;
    rclcpp_action::Server<MoveJoints>::SharedPtr move_joints_action_server_;
    rclcpp_action::Server<ControlGripper>::SharedPtr control_gripper_action_server_;
    rclcpp_action::Server<ControlSuck>::SharedPtr control_suck_action_server_;

    // ================== 核心分发逻辑 ==================

    rclcpp_action::GoalResponse handle_goal_move_pose(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MovePose::Goal> goal)
    {
        if (goal->enable_collision_check > 2) {
            RCLCPP_WARN(this->get_logger(), "enable_collision_check must be 0, 1 or 2");
            return rclcpp_action::GoalResponse::REJECT;
        }
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received movement request for part_id: %u", goal->arm_id);
        if (miracleAlgo_ptr_->staticBalance() && !whole_body_module_->is_idle()) {
            RCLCPP_WARN(this->get_logger(), "Static COM whole-body operation owns all movable body joints");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (!goal->via_poses.empty()) {
            if (goal->arm_id > ARM_RIGHT || goal->enable_move_line || goal->via_poses.size() > 64) {
                RCLCPP_WARN(this->get_logger(), "via_poses supports single-arm MoveP only, at most 64 poses");
                return rclcpp_action::GoalResponse::REJECT;
            }
            for (const auto& via : goal->via_poses) {
                if (!motion_control_pkg::kinematics::validateAndNormalizePoseStamped(via).success) {
                    return rclcpp_action::GoalResponse::REJECT;
                }
            }
        }

        if (!std::isfinite(goal->acceleration_scale) || goal->acceleration_scale < 0 || goal->acceleration_scale > 1 ||
            !std::isfinite(goal->jerk_scale) || goal->jerk_scale < 0 || goal->jerk_scale > 1 ||
            (goal->arm_id > ARM_BOTH && (goal->acceleration_scale != 0 || goal->jerk_scale != 0))) {
            RCLCPP_WARN(this->get_logger(), "Acceleration/jerk scales require arm goals and values in [0,1]");
            return rclcpp_action::GoalResponse::REJECT;
        }


        if (goal->control_mode != MovePose::Goal::CONTROL_POSITION &&
            goal->control_mode != MovePose::Goal::CONTROL_IMPEDANCE) {
            RCLCPP_WARN(
                this->get_logger(),
                "MovePose control_mode must be CONTROL_POSITION or "
                "CONTROL_IMPEDANCE.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->arm_id == ARM_LEFT || goal->arm_id == ARM_RIGHT ||
            goal->arm_id == WHOLE_BODY_LEFT || goal->arm_id == WHOLE_BODY_RIGHT) {
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
                            return std::isfinite(value);
                        }))
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "MovePose bound_size must be empty or contain three "
                        "finite dimensions in metres.");
                    return rclcpp_action::GoalResponse::REJECT;
                }
            }
            // const auto& point_cloud = goal->env_point_cloud;
            // if (!point_cloud.data.empty() || point_cloud.width != 0U ||
            //     point_cloud.height != 0U)
            // {
            //     // 当前依赖中没有已接入 MiracleMAlgo/Coal 的 PointCloud2
            //     // 场景导入器；显式拒绝，避免把未参与碰撞检测的数据静默忽略。
            //     RCLCPP_WARN(
            //         this->get_logger(),
            //         "MovePose environment point clouds are unavailable until "
            //         "an existing collision-world importer is integrated.");
            //     return rclcpp_action::GoalResponse::REJECT;
            // }
        }

        if (goal->arm_id == ARM_BOTH || goal->arm_id == WHOLE_BODY_BOTH) {
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
            // const auto& point_cloud = goal->env_point_cloud;
            // if (!point_cloud.data.empty() || point_cloud.width != 0U ||
            //     point_cloud.height != 0U)
            // {
            //     RCLCPP_WARN(
            //         this->get_logger(),
            //         "ARM_BOTH MovePose point clouds are not connected to the "
            //         "collision world");
            //     return rclcpp_action::GoalResponse::REJECT;
            // }
            const auto unsupported_bound = [](const auto& bound) {
                return std::any_of(bound.begin(), bound.end(), [](float value) {
                    return !std::isfinite(value);
                });
            };
            if (unsupported_bound(goal->left_bound_size) ||
                unsupported_bound(goal->right_bound_size))
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "ARM_BOTH MovePose left/right bound_size invalid");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        // 【路由检查】：根据指令中的参数，检查对应模块是否繁忙
        if (goal->arm_id == WHOLE_BODY_LEFT ||
            goal->arm_id == WHOLE_BODY_RIGHT ||
            goal->arm_id == WHOLE_BODY_BOTH) {
            if (goal->timeout_ms <= 0 || !whole_body_module_->is_idle()) {
                RCLCPP_WARN(this->get_logger(), "Whole-body module is busy or timeout is invalid");
                return rclcpp_action::GoalResponse::REJECT;
            }
            const bool needs_left = goal->arm_id == WHOLE_BODY_LEFT ||
                goal->arm_id == WHOLE_BODY_BOTH;
            const bool needs_right = goal->arm_id == WHOLE_BODY_RIGHT ||
                goal->arm_id == WHOLE_BODY_BOTH;
            if (miracleAlgo_ptr_->staticBalance() &&
                (!head_module_->is_idle(ID_HEAD) || !dual_arm_module_->is_idle(ID_LEFT_ARM) ||
                 !dual_arm_module_->is_idle(ID_RIGHT_ARM))) {
                RCLCPP_WARN(this->get_logger(), "Static COM requires head and both arms idle before planning");
                return rclcpp_action::GoalResponse::REJECT;
            }
            if ((needs_left && !dual_arm_module_->is_idle(ID_LEFT_ARM)) ||
                (needs_right && !dual_arm_module_->is_idle(ID_RIGHT_ARM)) ||
                !lower_body_module_->is_idle(ID_LEG))
            {
                RCLCPP_WARN(this->get_logger(), "Whole-body resources are busy");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else if (goal->arm_id == ARM_LEFT) {
            if (!dual_arm_module_->is_idle(ID_LEFT_ARM)) {
                RCLCPP_WARN(this->get_logger(), "Left arm is busy!");
                return rclcpp_action::GoalResponse::REJECT;
            }
        } 
        else if (goal->arm_id == ARM_RIGHT) {
            if (!dual_arm_module_->is_idle(ID_RIGHT_ARM)) {
                RCLCPP_WARN(this->get_logger(), "Right arm is busy!");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else if (goal->arm_id == ARM_BOTH) {
            if (!dual_arm_module_->is_idle(ID_LEFT_ARM) ||
                !dual_arm_module_->is_idle(ID_RIGHT_ARM))
            {
                RCLCPP_WARN(this->get_logger(), "Dual arms are busy!");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else if (goal->arm_id == LEG) { 
            if (!lower_body_module_->is_idle(ID_LEG)) {
                RCLCPP_WARN(this->get_logger(), "Leg is busy!");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Unknown arm_id or part ID!");
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

        // 全身模式由协调模块统一规划，再把轨迹分发给手臂和腿部。
        if (goal->arm_id == WHOLE_BODY_LEFT ||
            goal->arm_id == WHOLE_BODY_RIGHT ||
            goal->arm_id == WHOLE_BODY_BOTH) {
            whole_body_module_->start_execution_move_pose(goal_handle);
        }
        // 【任务分发】：将 goal_handle 丢给对应的部件模块去执行
        else if (goal->arm_id == ARM_LEFT || goal->arm_id == ARM_RIGHT) {
            if (goal_handle->is_canceling()) {
                auto result = std::make_shared<MovePose::Result>();
                result->success = ACTION_FAIL;
                result->error_message =
                    "MovePose goal canceled before dispatch";
                goal_handle->canceled(result);
                return;
            }
            RCLCPP_INFO(
                this->get_logger(),
                "Dispatching single-arm MovePose target with frame_id=%s",
                goal->target_pose.header.frame_id.c_str());
            dual_arm_module_->start_execution_move_pose(
                goal_handle, goal->target_pose);
        } else if (goal->arm_id == ARM_BOTH) {
            const auto request_started_at = std::chrono::steady_clock::now();
            if (goal_handle->is_canceling()) {
                auto result = std::make_shared<MovePose::Result>();
                result->success = ACTION_FAIL;
                result->error_message =
                    "ARM_BOTH MovePose canceled before dispatch";
                goal_handle->canceled(result);
                return;
            }
            RCLCPP_INFO(
                this->get_logger(),
                "Dispatching ARM_BOTH MovePose: left_frame=%s right_frame=%s",
                goal->left_target_pose.header.frame_id.c_str(),
                goal->right_target_pose.header.frame_id.c_str());
            dual_arm_module_->start_execution_move_dual_arm_pose(
                goal_handle, goal->left_target_pose, goal->right_target_pose,
                request_started_at);
        }
        else if (goal->arm_id == LEG) {
            RCLCPP_INFO(this->get_logger(), "Dispatching task to leg.");
            lower_body_module_->start_execution_move_pose(goal_handle);
        }
    }

    // ==============================================================


     rclcpp_action::GoalResponse handle_goal_move_joints(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveJoints::Goal> goal)
    {
        if (goal->enable_collision_check > 2) {
            RCLCPP_WARN(this->get_logger(), "enable_collision_check must be 0, 1 or 2");
            return rclcpp_action::GoalResponse::REJECT;
        }
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received move joints request for part_id: %u", goal->arm_id);
        if (miracleAlgo_ptr_->staticBalance() && !whole_body_module_->is_idle()) {
            RCLCPP_WARN(this->get_logger(), "Joint command rejected during static COM whole-body operation");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (!std::isfinite(goal->acceleration_scale) || goal->acceleration_scale < 0 || goal->acceleration_scale > 1 ||
            !std::isfinite(goal->jerk_scale) || goal->jerk_scale < 0 || goal->jerk_scale > 1 ||
            (goal->arm_id > ARM_BOTH && (goal->acceleration_scale != 0 || goal->jerk_scale != 0))) {
            RCLCPP_WARN(this->get_logger(), "Acceleration/jerk scales require arm goals and values in [0,1]");
            return rclcpp_action::GoalResponse::REJECT;
        }


        if (goal->control_mode != MoveJoints::Goal::CONTROL_POSITION &&
            goal->control_mode != MoveJoints::Goal::CONTROL_IMPEDANCE) {
            RCLCPP_WARN(
                this->get_logger(),
                "MoveJoints control_mode must be CONTROL_POSITION or "
                "CONTROL_IMPEDANCE.");
            return rclcpp_action::GoalResponse::REJECT;
        }

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
            if (!dual_arm_module_->is_idle(ID_LEFT_ARM)) {
                RCLCPP_WARN(this->get_logger(), "Left arm is busy!");
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
            if (!dual_arm_module_->is_idle(ID_RIGHT_ARM)) {
                RCLCPP_WARN(this->get_logger(), "Right arm is busy!");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else if (goal->arm_id == ARM_BOTH) {
            if (!dual_arm_module_->is_idle(ID_LEFT_ARM) ||
                !dual_arm_module_->is_idle(ID_RIGHT_ARM))
            {
                RCLCPP_WARN(this->get_logger(), "Dual arms are busy!");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else if (goal->arm_id == LEG) { 
            if (!lower_body_module_->is_idle(ID_LEG)) {
                RCLCPP_WARN(this->get_logger(), "Leg is busy!");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else if (goal->arm_id == HEAD) {
            if (!head_module_->is_idle(ID_HEAD)) {
                RCLCPP_WARN(this->get_logger(), "Head is busy!");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Unknown arm_id or part ID!");
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
            RCLCPP_INFO(this->get_logger(), "Dispatching joints task to Dual Arm.");
            dual_arm_module_->start_execution_move_joints(
                goal_handle);
        } else if (goal->arm_id == ARM_BOTH) {
            dual_arm_module_->start_execution_move_dual_arm_joints(
                goal_handle);
        }
        else if (goal->arm_id == LEG) {
            RCLCPP_INFO(this->get_logger(), "Dispatching joints task to leg.");
            lower_body_module_->start_execution_move_joints(goal_handle);
        }
        else if (goal->arm_id == HEAD) {
            RCLCPP_INFO(this->get_logger(), "Dispatching joints task to head.");
            head_module_->start_execution_move_joints(goal_handle);
        }
    }

    //====================================================================

    rclcpp_action::GoalResponse handle_goal_control_gripper(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ControlGripper::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received movement request for gripper_id: %u", goal->gripper_id);

        const auto validation =
            motion_control_pkg::execution::validateControlGripperGoal(*goal);
        if (!validation) {
            RCLCPP_WARN(
                this->get_logger(), "Rejecting ControlGripper goal: %s",
                validation.error_message.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        const bool needs_left =
            goal->gripper_id == ControlGripper::Goal::GRIPPER_LEFT ||
            goal->gripper_id == ControlGripper::Goal::GRIPPER_BOTH;
        const bool needs_right =
            goal->gripper_id == ControlGripper::Goal::GRIPPER_RIGHT ||
            goal->gripper_id == ControlGripper::Goal::GRIPPER_BOTH;
        if ((needs_left && left_arm_tool_ != "gripper") ||
            (needs_right && right_arm_tool_ != "gripper"))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Rejecting ControlGripper goal: configured tools are "
                "left=%s, right=%s.",
                left_arm_tool_.c_str(), right_arm_tool_.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        if ((needs_left && !dual_arm_module_->is_idle(ID_LEFT_TOOL)) ||
            (needs_right && !dual_arm_module_->is_idle(ID_RIGHT_TOOL)))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Rejecting ControlGripper goal because a requested tool is busy.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_control_gripper(
        const std::shared_ptr<GoalHandleControlGripper> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_WARN(
            this->get_logger(),
            "Rejecting ControlGripper cancel request because a validated "
            "physical stop command is not available for the Jodell gripper.");
        return rclcpp_action::CancelResponse::REJECT;
    }

    void handle_accepted_control_gripper(
        const std::shared_ptr<GoalHandleControlGripper> goal_handle)
    {
        std::string start_error;
        if (!dual_arm_module_->start_execution_control_gripper(
                goal_handle, start_error))
        {
            auto result = std::make_shared<ControlGripper::Result>();
            result->success = ACTION_FAIL;
            result->final_position.assign(2U, 0U);
            result->final_torque.assign(2U, 0U);
            result->error_message =
                "ControlGripper lost the tool-resource race: " + start_error;
            RCLCPP_ERROR(
                this->get_logger(), "%s", result->error_message.c_str());
            goal_handle->abort(result);
        }
    }

    rclcpp_action::GoalResponse handle_goal_control_suck(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ControlSuck::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(
            this->get_logger(), "Received suction request for suck_id: %u",
            goal->suck_id);

        const auto validation =
            motion_control_pkg::execution::validateControlSuckGoal(*goal);
        if (!validation) {
            RCLCPP_WARN(
                this->get_logger(), "Rejecting ControlSuck goal: %s",
                validation.error_message.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        const bool needs_left =
            goal->suck_id == ControlSuck::Goal::SUCK_LEFT ||
            goal->suck_id == ControlSuck::Goal::SUCK_BOTH;
        const bool needs_right =
            goal->suck_id == ControlSuck::Goal::SUCK_RIGHT ||
            goal->suck_id == ControlSuck::Goal::SUCK_BOTH;
        if ((needs_left && left_arm_tool_ != "suck") ||
            (needs_right && right_arm_tool_ != "suck"))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Rejecting ControlSuck goal: configured tools are "
                "left=%s, right=%s.",
                left_arm_tool_.c_str(), right_arm_tool_.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        if ((needs_left && !dual_arm_module_->is_idle(ID_LEFT_TOOL)) ||
            (needs_right && !dual_arm_module_->is_idle(ID_RIGHT_TOOL)))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Rejecting ControlSuck goal because a requested tool is busy.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_control_suck(
        const std::shared_ptr<GoalHandleControlSuck> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_WARN(
            this->get_logger(),
            "Rejecting ControlSuck cancel request because the GPIO command "
            "completes immediately.");
        return rclcpp_action::CancelResponse::REJECT;
    }

    void handle_accepted_control_suck(
        const std::shared_ptr<GoalHandleControlSuck> goal_handle)
    {
        std::string start_error;
        if (!dual_arm_module_->start_execution_control_suck(
                goal_handle, start_error))
        {
            auto result = std::make_shared<ControlSuck::Result>();
            result->success = ACTION_FAIL;
            result->error_message =
                "ControlSuck lost the tool-resource race: " + start_error;
            RCLCPP_ERROR(
                this->get_logger(), "%s", result->error_message.c_str());
            goal_handle->abort(result);
        }
    }

};




int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControlNode>();

    try {
        //调用 init 初始化模块和 Action Server，安全传递 shared_from_this
        node->init();
        // Default callbacks may wait for the model lock held by a planner.
        // The separate acquisition/WBC subscription group must keep running.
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2);
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("robot_control_node"), "Startup failed: %s", e.what());
        node.reset();
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
