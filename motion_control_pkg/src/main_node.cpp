#include <rclcpp/rclcpp.hpp>
#include "main_node.hpp"
#include "dual_arm_module.hpp"
#include "ethercat_bus_manager.hpp"
#include "miracle_head_module.hpp"
#include "miracle_lower_body_module.hpp"


using namespace miracle;

class RobotControlNode : public rclcpp::Node {
public:
    RobotControlNode() : Node("robot_control_node") {}

    void init() {
        // 1. 创建各个身体部位的控制模块
        // 传入 shared_from_this() 让各模块都能使用这个 Node 发日志/建定时器
        dual_arm_module_ = std::make_shared<DualArmModule>("dual_arm", shared_from_this());

        std::string etherat_bus_config = " ";
        this->declare_parameter("EtherCAT_bus_config", " ");
        
        if(this->get_parameter("EtherCAT_bus_config", etherat_bus_config)) {
            RCLCPP_INFO(rclcpp::get_logger("robot_control_node"), "Param EtherCAT_bus_config is: %s !", etherat_bus_config.c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("robot_control_node"),"Failed to get parameter EtherCAT_bus_config");
        }
        ethercat_bus_manager_ptr_ = EthercatBusManager::getOrCreate(etherat_bus_config.c_str());

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
    std::shared_ptr<EthercatBusManager> ethercat_bus_manager_ptr_;

    std::shared_ptr<DualArmModule> dual_arm_module_;
    std::shared_ptr<MiracleHeadModule> head_module_;
    std::shared_ptr<MiracleLowerBodyModule> lower_body_module_;

    rclcpp_action::Server<MovePose>::SharedPtr move_pose_action_server_;
    rclcpp_action::Server<MoveJoints>::SharedPtr move_joints_action_server_;
    rclcpp_action::Server<ControlGripper>::SharedPtr control_gripper_action_server_;

    // ================== 核心分发逻辑 ==================

    rclcpp_action::GoalResponse handle_goal_move_pose(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MovePose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received movement request for part_id: %u", goal->arm_id);

        // 【路由检查】：根据指令中的参数，检查对应模块是否繁忙
        if (goal->arm_id == ARM_LEFT) {
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
        else if(goal->arm_id == ARM_BOTH) {
            if (!dual_arm_module_->is_idle(ID_LEFT_ARM) || !dual_arm_module_->is_idle(ID_RIGHT_ARM)) {
                RCLCPP_WARN(this->get_logger(), "At least one arm is busy!");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else if (goal->arm_id == LEG) { 
            if (!lower_body_module_->is_idle(ID_LEG)) {
                RCLCPP_WARN(this->get_logger(), "Leg is busy!");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        // else if (goal->arm_id == HEAD) { 未加
        //     if (!head_module_->is_idle(ID_HEAD)) {
        //         RCLCPP_WARN(this->get_logger(), "Head is busy!");
        //         return rclcpp_action::GoalResponse::REJECT;
        //     }
        // }
        else {
            RCLCPP_ERROR(this->get_logger(), "Unknown arm_id or part ID!");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_move_pose(
        const std::shared_ptr<GoalHandleMovePose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received move pose cancel request.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_move_pose(const std::shared_ptr<GoalHandleMovePose> goal_handle)
    {
        const auto goal = goal_handle->get_goal();

        // 【任务分发】：将 goal_handle 丢给对应的部件模块去执行
        if (goal->arm_id == ARM_LEFT || goal->arm_id == ARM_RIGHT || goal->arm_id == ARM_BOTH) {
            RCLCPP_INFO(this->get_logger(), "Dispatching task to Dual Arm.");
            dual_arm_module_->start_execution_move_pose(goal_handle);
        } 
        else if (goal->arm_id == LEG) {
            RCLCPP_INFO(this->get_logger(), "Dispatching task to leg.");
            lower_body_module_->start_execution_move_pose(goal_handle);
        }
        // else if (goal->arm_id == HEAD) {
        //     head_module_->start_execution_move_pose(goal_handle);
        // }
    }

    // ==============================================================


     rclcpp_action::GoalResponse handle_goal_move_joints(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveJoints::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received move joints request for part_id: %u", goal->arm_id);

        // 【路由检查】：根据指令中的参数，检查对应模块是否繁忙
        if (goal->arm_id == ARM_LEFT) {
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
        else if(goal->arm_id == ARM_BOTH) {
            if (!dual_arm_module_->is_idle(ID_LEFT_ARM) || !dual_arm_module_->is_idle(ID_RIGHT_ARM)) {
                RCLCPP_WARN(this->get_logger(), "At least one arm is busy!");
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
        RCLCPP_INFO(this->get_logger(), "Received move joints cancel request.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_move_joints(const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
    {
        const auto goal = goal_handle->get_goal();

        // 【任务分发】：将 goal_handle 丢给对应的部件模块去执行
        if (goal->arm_id == ARM_LEFT || goal->arm_id == ARM_RIGHT || goal->arm_id == ARM_BOTH) {
            RCLCPP_INFO(this->get_logger(), "Dispatching joints task to Dual Arm.");
            // dual_arm_module_->start_execution_move_joints(goal_handle);
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

        RCLCPP_INFO(this->get_logger(), "Received movement request for gripper_id: %u", goal->gripper_id);

        // 【路由检查】：TODO: 检查对应手臂是否安装有対应执行器，根据配置文件
        if (goal->gripper_id == GRIPPER_LEFT) {
            if (!dual_arm_module_->is_idle(ID_LEFT_TOOL)) {
                RCLCPP_WARN(this->get_logger(), "Left gripper is busy!");
                return rclcpp_action::GoalResponse::REJECT;
            }
        } 
        else if (goal->gripper_id == GRIPPER_RIGHT) {
            if (!dual_arm_module_->is_idle(ID_RIGHT_TOOL)) {
                RCLCPP_WARN(this->get_logger(), "Right gripper is busy!");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else if(goal->gripper_id == GRIPPER_BOTH) {
            if (!dual_arm_module_->is_idle(ID_LEFT_TOOL) || !dual_arm_module_->is_idle(ID_RIGHT_TOOL)) {
                RCLCPP_WARN(this->get_logger(), "At least one gripper is busy!");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Unknown gripper_id or part ID!");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_control_gripper(
        const std::shared_ptr<GoalHandleControlGripper> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received control gripper cancel request.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_control_gripper(
        const std::shared_ptr<GoalHandleControlGripper> goal_handle)
    {
        const auto goal = goal_handle->get_goal();

        // 【任务分发】：将 goal_handle 丢给对应的部件模块去执行
        if (goal->gripper_id == GRIPPER_LEFT || goal->gripper_id == GRIPPER_RIGHT || goal->gripper_id == GRIPPER_BOTH) {
            RCLCPP_INFO(this->get_logger(), "Dispatching task to Dual Arm Gripper.");
            dual_arm_module_->start_execution_control_gripper(goal_handle);
        } 
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
