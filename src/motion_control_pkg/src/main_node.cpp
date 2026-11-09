#include "motion_control_pkg/execution/shutdown_request.hpp"
#include "motion_control_pkg/execution/orderly_shutdown.hpp"
#include "motion_control_pkg/execution/motion_safety.hpp"
#include "motion_control_pkg/execution/ethercat_trajectory.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <sstream>

#include "main_node.hpp"
#include "com_interfaces/leg_joint_goal.hpp"
#include "com_interfaces/leg_body_motion_goal.hpp"
#include "com_interfaces/tcp_level_goal.hpp"
#include "com_interfaces/rigid_grasp_goal.hpp"
#include "com_interfaces/precision_arrival_goal.hpp"
#include "dual_arm_module.hpp"
#include "ethercat_bus_manager.hpp"
#include "com_interfaces/srv/drive_control.hpp"
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
    ~RobotControlNode() override {
        if (motion_safety_) motion_safety_->requestStop("control node shutdown");
        if (head_startup_thread_.joinable()) head_startup_thread_.join();
    }


    void init() {
        shutdown_pub_=create_publisher<std_msgs::msg::String>(
            "/Control/shutdown_state",rclcpp::QoS(1).transient_local());
        shutdown_timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{
            std_msgs::msg::String msg;msg.data=shutdownStatus();shutdown_pub_->publish(msg);
        });
        if (motion_control_pkg::execution::shutdownRequested()) return;
        // 1. 创建各个身体部位的控制模块
        // 传入 shared_from_this() 让各模块都能使用这个 Node 发日志/建定时器

        std::string etherat_bus_config = " ";
        this->declare_parameter("EtherCAT_bus_config", " ");
        
        if(this->get_parameter("EtherCAT_bus_config", etherat_bus_config)) {
            RCLCPP_INFO(rclcpp::get_logger("robot_control_node"), "Param EtherCAT_bus_config is: %s !", etherat_bus_config.c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("robot_control_node"),"Failed to get parameter EtherCAT_bus_config");
        }
        ethercat_attempted_=true;
        ethercat_bus_manager_ptr_ = EthercatBusManager::getOrCreate(etherat_bus_config.c_str());
        if (motion_control_pkg::execution::shutdownRequested()) return;
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
        rcl_interfaces::msg::ParameterDescriptor planning_workers_descriptor;
        planning_workers_descriptor.description = "Startup planning workers: 1 serial or 2/4 parallel";
        planning_workers_descriptor.read_only = true;
        const int planning_workers = this->declare_parameter<int>(
            "planning_worker_count", 2, planning_workers_descriptor);
        if (planning_workers != 1 && planning_workers != 2 && planning_workers != 4)
            throw std::invalid_argument("planning_worker_count must be 1, 2 or 4");
        miracleAlgo_ptr_->configurePlanningWorkers(static_cast<std::size_t>(planning_workers));
        RCLCPP_INFO(get_logger(), "Planning workers: %d (private caches, shared immutable model)", planning_workers);

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

        if (motion_control_pkg::execution::shutdownRequested()) return;
        std::string dual_arm_initialization_error;
        if (!dual_arm_module_->initialize(dual_arm_initialization_error)) {
            throw std::runtime_error(
                "Dual arm initialization failed: " +
                dual_arm_initialization_error);
        }
        dual_arm_module_->attachMeasuredFeedbackBus(ethercat_bus_manager_ptr_);
        lower_body_module_->set_body_motion_callbacks(
            [arms = dual_arm_module_, head = head_module_](std::string& error) {
                if (!arms->reserve_preplanned_trajectory_resources(true, true, error)) return false;
                if (!head->reserve_body_motion_resource(error)) {
                    arms->release_preplanned_trajectory_resources(true, true);
                    return false;
                }
                return true;
            },
            [arms = dual_arm_module_, head = head_module_] {
                head->release_body_motion_resource();
                arms->release_preplanned_trajectory_resources(true, true);
            },
            [arms = dual_arm_module_](sensor_msgs::msg::JointState& state, std::string& error) {
                return arms->read_body_motion_feedback(state, error);
            });
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

        configureMotionSafety();

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

        if (head_startup_policy_.enabled && !motion_control_pkg::execution::shutdownRequested()) {
            head_startup_finished_=false;
            head_startup_thread_=std::thread([this] {
                std::string error;
                const bool ok=head_module_->runStartupHome(head_startup_policy_,motion_safety_,error);
                head_startup_finished_=true;
                if(ok)RCLCPP_INFO(get_logger(),"Head startup home complete; normal URDF limits restored");
                else RCLCPP_ERROR(get_logger(),"Head startup home failed: %s",error.c_str());
            });
        }
        RCLCPP_INFO(this->get_logger(), "Robot Control Action Server is ready; motion admission follows safety/startup state.");
    }

    void beginShutdown() {
        std::lock_guard<std::mutex> lock(admission_mutex_);
        motion_control_pkg::execution::requestShutdown();
        if (shutdown_) return;
        if (motion_safety_) motion_safety_->requestStop("orderly process shutdown");
        const std::size_t axes=(ethercat_attempted_ ? 6U : 0U)+
            ((dual_arm_module_ && dual_arm_module_->shutdownDriverConnected()) ? 14U : 0U);
        using Shutdown=motion_control_pkg::execution::OrderlyShutdown;
        Shutdown::Hooks hooks;
        hooks.idle=[this] {return shutdownResourcesIdle();};
        hooks.read=[this](auto& values,auto& error,bool after_disable) {
            if (ethercat_attempted_ && (!ethercat_bus_manager_ptr_ ||
                !ethercat_bus_manager_ptr_->readShutdownFeedback(values,error,after_disable))) return false;
            const std::string bus_detail=error;
            if (dual_arm_module_ && dual_arm_module_->shutdownDriverConnected()) {
                std::string arm_detail;
                const bool arm_ok=dual_arm_module_->readShutdownFeedback(values,arm_detail);
                error=bus_detail+"; arms:"+arm_detail;
                if (!arm_ok) return false;
            }
            return true;
        };
        hooks.disable=[this](std::string& error) {
            // Submit both commands even if one backend reports failure; never
            // release either backend on an unconfirmed result.
            bool bus_ok=!ethercat_attempted_;
            if (ethercat_bus_manager_ptr_) bus_ok=ethercat_bus_manager_ptr_->requestShutdownDisable(error);
            std::string arm_error;
            const bool arm_ok=!dual_arm_module_ || dual_arm_module_->disableForShutdown(arm_error);
            if (!arm_ok) error+="; "+arm_error;
            return bus_ok && arm_ok;
        };
        hooks.release=[this](std::string& error) {
            if (head_startup_thread_.joinable()) head_startup_thread_.join();
            // Readers are serialized by the SDK gate; after release they fail
            // closed without touching the vendor handle. PDO joins precede IgH release.
            if (dual_arm_module_ && !dual_arm_module_->releaseAfterShutdown(error)) return false;
            return !ethercat_bus_manager_ptr_ || ethercat_bus_manager_ptr_->releaseAfterShutdown(error);
        };
        shutdown_=std::make_unique<Shutdown>(std::move(hooks),axes,Shutdown::Clock::now());
        setShutdownStatus(shutdown_->status());
        RCLCPP_WARN(get_logger(),"Orderly shutdown requested: admission closed; keeping ROS/feedback alive until confirmed");
    }

    bool advanceShutdown() {
        const auto previous=shutdown_->phase();
        shutdown_->tick(motion_control_pkg::sdk::DriveClock::now());
        const auto status=shutdown_->status();
        if (status!=shutdownStatus()) {
            setShutdownStatus(status);
            if (shutdown_->phase()==motion_control_pkg::execution::OrderlyShutdown::Phase::FailedHold)
                RCLCPP_ERROR(get_logger(),"%s; drivers/feedback retained; repeated signals cannot force completion",status.c_str());
            else if (previous!=shutdown_->phase()) RCLCPP_INFO(get_logger(),"%s",status.c_str());
        }
        return shutdown_->complete();
    }

private:
    std::mutex admission_mutex_;
    mutable std::mutex shutdown_status_mutex_;
    std::string shutdown_status_{"shutdown=RUNNING admission=open brake_feedback=UNAVAILABLE"};
    std::unique_ptr<motion_control_pkg::execution::OrderlyShutdown> shutdown_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shutdown_pub_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;
    std::atomic<bool> head_startup_finished_{true};
    bool ethercat_attempted_{false};
    std::string shutdownStatus() const {
        std::lock_guard<std::mutex> lock(shutdown_status_mutex_);return shutdown_status_;
    }
    void setShutdownStatus(const std::string& status) {
        {std::lock_guard<std::mutex> lock(shutdown_status_mutex_);shutdown_status_=status;}
        if (shutdown_pub_) {std_msgs::msg::String msg;msg.data=status;shutdown_pub_->publish(msg);}
    }
    bool shutdownResourcesIdle() const {
        return head_startup_finished_ && (!motion_safety_ || motion_safety_->workersIdle()) &&
            (!whole_body_module_ || whole_body_module_->is_idle()) &&
            (!head_module_ || head_module_->is_idle(ID_HEAD)) &&
            (!lower_body_module_ || lower_body_module_->is_idle(ID_LEG)) &&
            (!dual_arm_module_ || (dual_arm_module_->is_idle(ID_LEFT_ARM) &&
             dual_arm_module_->is_idle(ID_RIGHT_ARM) && dual_arm_module_->is_idle(ID_LEFT_TOOL) &&
             dual_arm_module_->is_idle(ID_RIGHT_TOOL)));
    }
    motion_control_pkg::execution::HeadStartupHomePolicy head_startup_policy_;
    std::thread head_startup_thread_;
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

    std::shared_ptr<motion_control_pkg::execution::MotionSafety> motion_safety_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motion_safety_pub_;
    rclcpp::TimerBase::SharedPtr motion_safety_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_motion_service_, reset_motion_fault_service_;
    rclcpp::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr arm_error_service_, body_error_service_;
    rclcpp::CallbackGroup::SharedPtr drive_control_group_;
    rclcpp::Service<com_interfaces::srv::DriveControl>::SharedPtr drive_control_service_;
    std::mutex drive_clear_mutex_;

    bool motionResourcesIdle() const {
        return whole_body_module_->is_idle() && head_module_->is_idle(ID_HEAD) &&
            lower_body_module_->is_idle(ID_LEG) && dual_arm_module_->is_idle(ID_LEFT_ARM) &&
            dual_arm_module_->is_idle(ID_RIGHT_ARM);
    }

    void configureDriveControl() {
        using Service=com_interfaces::srv::DriveControl;
        drive_control_group_=create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        drive_control_service_=create_service<Service>("/Control/drive_control",
            [this](const std::shared_ptr<Service::Request> request,
                   std::shared_ptr<Service::Response> response) {
                if (motion_control_pkg::execution::shutdownRequested()) {
                    response->message="orderly shutdown in progress; drive maintenance refused"; return;
                }
                if (request->target>Service::Request::LOWER_BODY ||
                    request->operation>Service::Request::CLEAR_FAULT) {
                    response->message="invalid target or operation"; return;
                }
                const bool clear=request->operation==Service::Request::CLEAR_FAULT;
                std::unique_lock<std::mutex> clear_lock(drive_clear_mutex_,std::defer_lock);
                if (clear && (!clear_lock.try_lock() || !motionResourcesIdle())) {
                    response->message="clear refused: maintenance or motion resources busy"; return;
                }
                // Fence every coupled trajectory before touching a selected drive.
                // This includes WBC compensation and targets from other clients.
                motion_safety_->requestStop(clear ? "explicit online drive clear" : "online drive emergency stop");
                if (clear && !motionResourcesIdle()) {
                    response->message="clear refused: motion worker still leaving execution"; return;
                }
                motion_control_pkg::sdk::DriveControlResult result;
                if (request->target==Service::Request::LOWER_BODY) {
                    result=ethercat_bus_manager_ptr_->controlLowerBodyDrives(clear);
                } else {
                    result=dual_arm_module_->controlDrives(
                        request->target!=Service::Request::RIGHT_ARM,
                        request->target!=Service::Request::LEFT_ARM,clear);
                }
                response->success=result.success();
                response->command_issued=result.command_issued;
                response->feedback_confirmed=result.feedback_confirmed;
                response->message=result.message;
            },rmw_qos_profile_services_default,drive_control_group_);
    }

    void configureMotionSafety() {
        using namespace motion_control_pkg::execution;
        SafetyPolicy policy;
        policy.tracking_error=declare_parameter("motion_tracking_error_rad",policy.tracking_error);
        policy.hard_error=declare_parameter("motion_tracking_hard_error_rad",policy.hard_error);
        policy.tracking_dwell=declare_parameter("motion_tracking_dwell_sec",policy.tracking_dwell);
        policy.max_feedback_age=declare_parameter("motion_feedback_max_age_sec",policy.max_feedback_age);
        policy.stop_timeout=declare_parameter("motion_stop_confirmation_timeout_sec",policy.stop_timeout);
        motion_safety_=std::make_shared<MotionSafety>(policy);
        const auto limits=[&](const std::vector<std::string>& names) {
            std::vector<SafetyJoint> result;
            const auto& model=miracleAlgo_ptr_->getModel();
            for(const auto& name:names) {
                if(!model.existJointName(name)) throw std::runtime_error("missing safety joint: "+name);
                const auto& joint=model.joints[model.getJointId(name)];
                result.push_back({name,model.lowerPositionLimit[joint.idx_q()],
                    model.upperPositionLimit[joint.idx_q()],model.velocityLimit[joint.idx_v()]});
            }
            return result;
        };
        for(unsigned arm=0;arm<2;++arm) {
            std::vector<std::string> names;
            for(unsigned j=1;j<=7;++j)names.push_back("Joint"+std::to_string(j)+(arm==0?"_L":"_R"));
            motion_safety_->addChannel(1U<<arm,limits(names),
                [weak=std::weak_ptr<DualArmModule>(dual_arm_module_),arm](SafetySample& sample,std::string& error) {
                    const auto module=weak.lock();
                    motion_control_pkg::sdk::DualArmDeviceState state;
                    if(!module || !module->read_preplanned_completion_feedback(state,error))return false;
                    const auto& a=state.arms[arm];
                    if(a.feedback_sequence<0 || !motion_control_pkg::sdk::armFeedbackOperational(a)) {
                        error="arm driver fault or invalid state: arm="+std::to_string(arm)+
                            " error="+std::to_string(a.error_code)+" state="+std::to_string(a.current_state);return false;
                    }
                    constexpr double radians=0.017453292519943295;
                    for(unsigned j=0;j<7;++j){sample.position.push_back(a.joint_position[j]*radians);sample.velocity.push_back(a.joint_velocity[j]*radians);}
                    if(a.controller_joint_command)
                        for(double q:*a.controller_joint_command)sample.controller_command.push_back(q*radians);
                    sample.sequence=a.feedback_sequence;sample.acquired=a.acquired;sample.wraps_at_million=true;
                    return true;
                });
        }
        const auto bus_reader=[weak=std::weak_ptr<EthercatBusManager>(ethercat_bus_manager_ptr_)](std::vector<std::uint16_t> ids) {
            return [weak,ids=std::move(ids)](SafetySample& sample,std::string& error) {
                JointCompletionFeedback measured;
                if(!readEthercatChainFeedback(weak.lock(),ids,measured,error))return false;
                sample.position=std::move(measured.position);sample.velocity=std::move(measured.velocity);
                sample.acquired=measured.acquired;sample.sequence=measured.sequence;return true;
            };
        };
        motion_safety_->addChannel(kLegSafety,limits({"leg_joint1","leg_joint2","leg_joint3","body_joint"}),
            bus_reader({LEG_JOINT1,LEG_JOINT2,LEG_JOINT3,BODY_JOINT}));
        const auto head_limits=limits({"head_yaw_joint","head_pitch_joint"});
        motion_safety_->addChannel(kHeadSafety,head_limits,
            bus_reader({HEAD_YAW_MOTOR_ID,HEAD_PITCH_MOTOR_ID}));
        rcl_interfaces::msg::ParameterDescriptor fixed;fixed.read_only=true;
        auto& home=head_startup_policy_;
        home.enabled=declare_parameter("head_startup_home.enabled",true,fixed);
        home.lower=declare_parameter<std::vector<double>>("head_startup_home.recovery_lower_rad",
            {head_limits[0].lower,-0.7},fixed);
        home.upper=declare_parameter<std::vector<double>>("head_startup_home.recovery_upper_rad",
            {head_limits[0].upper,head_limits[1].upper},fixed);
        home.velocity=declare_parameter("head_startup_home.velocity_rad_s",home.velocity,fixed);
        home.acceleration=declare_parameter("head_startup_home.acceleration_rad_s2",home.acceleration,fixed);
        home.jerk=declare_parameter("head_startup_home.jerk_rad_s3",home.jerk,fixed);
        home.feedback_wait=declare_parameter("head_startup_home.feedback_wait_sec",home.feedback_wait,fixed);
        home.timeout=declare_parameter("head_startup_home.timeout_sec",home.timeout,fixed);
        motion_safety_->configureHeadStartupHome(home);
        installMotionSafety(motion_safety_);
        motion_safety_->start();
        motion_safety_pub_=create_publisher<std_msgs::msg::String>("/Control/motion_safety",rclcpp::QoS(1).transient_local());
        motion_safety_timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{
            std_msgs::msg::String status;status.data=motion_safety_->status()+"; "+shutdownStatus();motion_safety_pub_->publish(status);
        });
        stop_motion_service_=create_service<std_srvs::srv::Trigger>("/Control/stop_motion",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
                motion_safety_->requestStop("explicit stop service request");
                response->success=true;response->message="command fence installed; measured stop confirmation is asynchronous; "+motion_safety_->status();
            });
        reset_motion_fault_service_=create_service<std_srvs::srv::Trigger>("/Control/reset_motion_fault",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
                // Resource reservations precede worker startup. Include workers
                // that have been accepted but have not entered MotionSafetyScope
                // yet (and head/leg workers that are still planning).
                if (!whole_body_module_->is_idle() || !head_module_->is_idle(ID_HEAD) ||
                    !lower_body_module_->is_idle(ID_LEG) || !dual_arm_module_->is_idle(ID_LEFT_ARM) ||
                    !dual_arm_module_->is_idle(ID_RIGHT_ARM)) {
                    response->success=false;
                    response->message="motion resources still reserved by an accepted/planning/executing worker";
                    return;
                }
                response->success=motion_safety_->reset(response->message);
            });
        // Independent runtime switches; both start OFF on every process start.
        const auto error_service=[this](const std::string& name,SafetyMask group) {
          return create_service<rcl_interfaces::srv::SetParametersAtomically>(
            "/Control/"+name,
            [this,group](const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> request,
                   std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> response) {
                std::lock_guard<std::mutex> admission_lock(admission_mutex_);
                if (motion_control_pkg::execution::shutdownRequested()) {
                    response->result.reason="orderly shutdown in progress";return;
                }
                if (!motionResourcesIdle()) {
                    response->result.reason="error tracking option requires idle motion resources";return;
                }
                bool enabled=false;double limit=0,hard=0;unsigned seen=0;
                using Type=rcl_interfaces::msg::ParameterType;
                for (const auto& parameter:request->parameters) {
                    unsigned bit=0;
                    if(parameter.name=="enabled" && parameter.value.type==Type::PARAMETER_BOOL) {
                        bit=1;enabled=parameter.value.bool_value;
                    } else if(parameter.name=="position_error_rad" && parameter.value.type==Type::PARAMETER_DOUBLE) {
                        bit=2;limit=parameter.value.double_value;
                    } else if(parameter.name=="hard_error_rad" && parameter.value.type==Type::PARAMETER_DOUBLE) {
                        bit=4;hard=parameter.value.double_value;
                    }
                    if(!bit || (seen&bit)) {
                        response->result.reason="unknown, duplicate or wrongly typed error tracking option";return;
                    }
                    seen|=bit;
                }
                if (!(seen&1) || (enabled && seen!=7) || (!enabled && seen!=1)) {
                    response->result.reason="enable requires enabled + position_error_rad + hard_error_rad; disable requires enabled only";
                    return;
                }
                response->result.successful=motion_safety_->setErrorTrackingEnabled(
                    group,enabled,limit,hard,response->result.reason);
                if(response->result.successful) RCLCPP_INFO(get_logger(),"%s",response->result.reason.c_str());
            });
        };
        arm_error_service_=error_service("arm_error",kArmSafety);
        body_error_service_=error_service("body_error",kBodySafety);
        RCLCPP_INFO(get_logger(),"arm_error and body_error default OFF; enable: <command> <position_error_rad> <hard_error_rad>; body_error covers legs, waist and head");
        configureDriveControl();
    }

    // ================== 核心分发逻辑 ==================

    rclcpp_action::GoalResponse handle_goal_move_pose(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MovePose::Goal> goal)
    {
        std::lock_guard<std::mutex> admission_lock(admission_mutex_);
        if (motion_control_pkg::execution::shutdownRequested()) return rclcpp_action::GoalResponse::REJECT;
        std::string safety_error;
        if (!motion_safety_ || !motion_safety_->ready(safety_error)) {
            RCLCPP_WARN(get_logger(),"Motion admission refused: %s",safety_error.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

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
        std::string constraint_error;
        if (!com_interfaces::validatePrecisionArrivalGoal(goal->precision_arrival,
                goal->arm_id, true, goal->control_mode, constraint_error)) {
            RCLCPP_WARN(get_logger(), "%s", constraint_error.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (!com_interfaces::validateRigidGraspGoal(*goal,constraint_error)) {
            RCLCPP_WARN(get_logger(), "%s", constraint_error.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (!com_interfaces::validateTcpLevelGoal(goal->keep_tcp_level, goal->tcp_level_tolerance_rad,
                goal->arm_id, true, goal->control_mode, constraint_error)) {
            RCLCPP_WARN(get_logger(), "%s", constraint_error.c_str());
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

        if (goal->whole_body_waist_only &&
            ((goal->arm_id != WHOLE_BODY_LEFT && goal->arm_id != WHOLE_BODY_RIGHT &&
              goal->arm_id != WHOLE_BODY_BOTH) || goal->enable_move_line)) {
            RCLCPP_WARN(this->get_logger(),
                "whole_body_waist_only requires whole-body MoveP (arm_id=5/6/7, enable_move_line=false)");
            return rclcpp_action::GoalResponse::REJECT;
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
        std::lock_guard<std::mutex> admission_lock(admission_mutex_);
        if (motion_control_pkg::execution::shutdownRequested()) {
            auto result=std::make_shared<MovePose::Result>();
            result->success=ACTION_FAIL;result->error_message="orderly shutdown before dispatch";
            goal_handle->abort(result);return;
        }

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
        std::lock_guard<std::mutex> admission_lock(admission_mutex_);
        if (motion_control_pkg::execution::shutdownRequested()) return rclcpp_action::GoalResponse::REJECT;
        std::ostringstream names, positions; positions.precision(9);
        for(std::size_t j=0;j<goal->joint_name.size();++j)names << (j?",":"") << goal->joint_name[j];
        for(std::size_t j=0;j<goal->position.size();++j)positions << (j?",":"") << goal->position[j];
        RCLCPP_INFO(get_logger(),
            "MoveJoints goal task_id=%s arm_id=%u control_mode=%u speed=%.9g acceleration_scale=%.9g "
            "jerk_scale=%.9g collision_check=%u timeout_ms=%u names=[%s] positions_rad=[%s]",
            goal->task_id.c_str(),static_cast<unsigned>(goal->arm_id),static_cast<unsigned>(goal->control_mode),
            goal->speed,goal->acceleration_scale,goal->jerk_scale,
            static_cast<unsigned>(goal->enable_collision_check),static_cast<unsigned>(goal->timeout_ms),
            names.str().c_str(),positions.str().c_str());
        std::string safety_error;
        if (!motion_safety_ || !motion_safety_->ready(safety_error)) {
            RCLCPP_WARN(get_logger(),"Motion admission refused: %s",safety_error.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        std::string body_error;
        if (!com_interfaces::validatePrecisionArrivalGoal(goal->precision_arrival,
                goal->arm_id, false, goal->control_mode, body_error)) {
            RCLCPP_WARN(get_logger(), "%s", body_error.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (!com_interfaces::validateTcpLevelGoal(goal->keep_tcp_level, goal->tcp_level_tolerance_rad,
                goal->arm_id, false, goal->control_mode, body_error)) {
            RCLCPP_WARN(get_logger(), "%s", body_error.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (!com_interfaces::validateLegBodyMotionGoal(goal->arm_id, goal->leg_motion_mode,
                goal->body_tilt_deg, goal->body_height_m, goal->joint_name, goal->position,
                goal->speed, goal->timeout_ms, goal->control_mode, goal->acceleration_scale,
                goal->jerk_scale, false, body_error)) {
            RCLCPP_WARN(this->get_logger(), "Invalid leg body motion: %s", body_error.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->leg_motion_mode != MoveJoints::Goal::LEG_JOINT_TARGETS &&
            !goal->header.frame_id.empty() && goal->header.frame_id != "base_link") {
            RCLCPP_WARN(this->get_logger(), "Leg body targets use base_link coordinates");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->leg_motion_mode != MoveJoints::Goal::LEG_JOINT_TARGETS &&
            (!dual_arm_module_->is_idle(ID_LEFT_ARM) || !dual_arm_module_->is_idle(ID_RIGHT_ARM) ||
             !head_module_->is_idle(ID_HEAD))) {
            RCLCPP_WARN(this->get_logger(), "Body motion requires idle upper-body resources");
            return rclcpp_action::GoalResponse::REJECT;
        }
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
            if (goal->joint_name.size() == 2 || goal->position.size() == 2 ||
                goal->joint_name.size() == 3 || goal->position.size() == 3) {
                com_interfaces::LegJointSelection selection;
                std::string error;
                if (!com_interfaces::validateLegOrientationGoal(goal->joint_name, goal->position, selection, error) ||
                    !std::isfinite(goal->speed) || goal->speed <= 0 || goal->speed > 1 ||
                    goal->timeout_ms <= 0 || goal->control_mode != MoveJoints::Goal::CONTROL_POSITION) {
                    RCLCPP_WARN(this->get_logger(), "Invalid body-orientation leg MoveJ: %s; require speed (0,1], position mode and positive timeout", error.c_str());
                    return rclcpp_action::GoalResponse::REJECT;
                }
            }
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
        std::lock_guard<std::mutex> admission_lock(admission_mutex_);
        if (motion_control_pkg::execution::shutdownRequested()) {
            auto result=std::make_shared<MoveJoints::Result>();
            result->success=ACTION_FAIL;result->error_message="orderly shutdown before dispatch";
            goal_handle->abort(result);return;
        }

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
        std::lock_guard<std::mutex> admission_lock(admission_mutex_);
        if (motion_control_pkg::execution::shutdownRequested()) return rclcpp_action::GoalResponse::REJECT;
        if (head_startup_policy_.enabled) {
            std::string error;
            if (!motion_safety_->ready(error)) {
                RCLCPP_WARN(get_logger(),"Tool admission refused during startup/fault: %s",error.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
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
        std::lock_guard<std::mutex> admission_lock(admission_mutex_);
        if (motion_control_pkg::execution::shutdownRequested()) {
            auto result=std::make_shared<ControlGripper::Result>();
            result->success=ACTION_FAIL;result->error_message="orderly shutdown before dispatch";
            goal_handle->abort(result);return;
        }

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
        std::lock_guard<std::mutex> admission_lock(admission_mutex_);
        if (motion_control_pkg::execution::shutdownRequested()) return rclcpp_action::GoalResponse::REJECT;
        if (head_startup_policy_.enabled) {
            std::string error;
            if (!motion_safety_->ready(error)) {
                RCLCPP_WARN(get_logger(),"Tool admission refused during startup/fault: %s",error.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
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
        std::lock_guard<std::mutex> admission_lock(admission_mutex_);
        if (motion_control_pkg::execution::shutdownRequested()) {
            auto result=std::make_shared<ControlSuck::Result>();
            result->success=ACTION_FAIL;result->error_message="orderly shutdown before dispatch";
            goal_handle->abort(result);return;
        }

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
    using namespace motion_control_pkg::execution;
    installShutdownSignalHandlers();
    // Default rclcpp handlers invalidate the context before measured stop can
    // finish. Ctrl+C / SIGTERM now request the same coordinator exactly once.
    rclcpp::init(argc,argv,rclcpp::InitOptions{},rclcpp::SignalHandlerOptions::None);
    auto node=std::make_shared<RobotControlNode>();
    int result=0;
    try { node->init(); }
    catch (const std::exception& error) {
        RCLCPP_ERROR(node->get_logger(),"Startup failed; entering orderly shutdown: %s",error.what());
        result=1;requestShutdown();
    }
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{},2);
    executor.add_node(node);
    std::thread spin([&] {
        try {executor.spin();}
        catch (const std::exception& error) {
            RCLCPP_ERROR(node->get_logger(),"Executor failed: %s",error.what());
            requestShutdown();
        }
    });
    while (!shutdownRequested()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    node->beginShutdown();
    while (!node->advanceShutdown()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    executor.cancel();
    spin.join();
    executor.remove_node(node);
    rclcpp::shutdown();
    node.reset();
    return result;
}
