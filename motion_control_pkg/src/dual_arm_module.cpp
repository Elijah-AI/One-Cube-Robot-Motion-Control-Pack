#include "dual_arm_module.hpp"
#include <std_msgs/msg/header.hpp>
#include <memory>

#include <iostream>

DualArmModule::DualArmModule(const std::string& arm_name, rclcpp::Node::SharedPtr node)
: name_(arm_name), node_(node), is_executing_left_arm_(false), is_executing_right_arm_(false), is_executing_left_tool_(false), is_executing_right_tool_(false)
{
    auto shared_node = lockNode(__func__);
    if (!shared_node) {
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("dual_arm_module"), "Initializing duaml arm module...");

    // 初始化 PickAndPlace Action Server
    // this->pick_and_place_action_server_ = rclcpp_action::create_server<PickAndPlace>(
    //     this,
    //     "/Control/pick_and_place",
    //     std::bind(&DualArmModule::handle_goal_pick_and_place, this, std::placeholders::_1, std::placeholders::_2),
    //     std::bind(&DualArmModule::handle_cancel_pick_and_place, this, std::placeholders::_1),
    //     std::bind(&DualArmModule::handle_accepted_pick_and_place, this, std::placeholders::_1));

    // // 初始化 HardwareReset Action Server
    // this->hardware_reset_action_server_ = rclcpp_action::create_server<HardwareReset>(
    //     this,
        
    //     "/Control/hardware_reset",
    //     std::bind(&DualArmModule::handle_goal_hardware_reset, this, std::placeholders::_1, std::placeholders::_2),
    //     std::bind(&DualArmModule::handle_cancel_hardware_reset, this, std::placeholders::_1),
    //     std::bind(&DualArmModule::handle_accepted_hardware_reset, this, std::placeholders::_1));


    // // 初始化 HardwareCheck Action Server
    // this->hardware_Check_action_server_ = rclcpp_action::create_server<HardwareCheck>(
    //     this,
    //     "/Control/hardware_check",
    //     std::bind(&DualArmModule::handle_goal_hardware_check, this, std::placeholders::_1, std::placeholders::_2),
    //     std::bind(&DualArmModule::handle_cancel_hardware_check, this, std::placeholders::_1),
    //     std::bind(&DualArmModule::handle_accepted_hardware_check, this, std::placeholders::_1));

   
    heart_beat_pub_ = shared_node->create_publisher<HeartBeat>("/SafeNode/heartbeat", 10);
    device_status_pub_ = shared_node->create_publisher<NodeStatus>("/SafeNode/status", 10);
    // this->status = STATE_INIT;
    // this->error_flag = 0;

    hear_beat_timer_ = shared_node->create_wall_timer(1000ms,
    std::bind(&DualArmModule::heart_beat_cb, this));
    auto ds_msg = com_interfaces::msg::NodeStatus();
    ds_msg.node_id = MOTION_CONTROL;
    ds_msg.status = STATE_INIT ;//node_->status;
    ds_msg.header.stamp = shared_node->get_clock()->now();
    ds_msg.error_flag = 0;
    device_status_pub_->publish(ds_msg);

    r_q_lower_limits_.resize(arm_dof_), 
    r_q_upper_limits_.resize(arm_dof_),  
    r_v_limit_.resize(arm_dof_),
    r_a_limit_.resize(arm_dof_),
    r_jerk_limit_.resize(arm_dof_),
    l_q_lower_limits_.resize(arm_dof_),
    l_q_upper_limits_.resize(arm_dof_),
    l_v_limit_.resize(arm_dof_),
    l_a_limit_.resize(arm_dof_),
    l_jerk_limit_.resize(arm_dof_);
    l_init_arm_params_ = init_arm_params();
    init_set_tcp();
    std::cout<<"l_init_arm_params"<<static_cast<int>(l_init_arm_params_)<<std::endl;
    init_robot_link_ = OnLinkTo(192,168,1,190);
    std::cout<<"3*********"<<std::endl;
    if(init_robot_link_)
    {
       OnLogOff();
       OnLocalLogOff(); 
       set_arm_control_mode();
       std::cout<<"4*********"<<std::endl;
    }
   
    this->motor_status_pub_ = shared_node->create_publisher<MotorStatus>("/Control/MotorInfo",10);
    joint_feedback_timer_ = shared_node->create_wall_timer(250ms,
    std::bind(&DualArmModule::motor_statu_cb, this));
    motor_status_msg_ = com_interfaces::msg::MotorStatus();
    init_motor_status();
    MotionConstraint cons;
    cons.max_vel = l_v_limit_;
    cons.max_acc = l_a_limit_;
    cons.max_jerk = l_jerk_limit_;
    traj_ptr_ = PlannerFactory::createPlanner(PlannerType::RUCKIG);
    traj_ptr_->init(arm_dof_, cons);
    traj_ptr_->setCycleTime(CONTROL_TIME); 
    //
    l_install_base_inv_[0][0] = 1.0; l_install_base_inv_[0][1] = 0.0; l_install_base_inv_[0][2] = 0.0; l_install_base_inv_[0][3] = 0.0;
    l_install_base_inv_[1][0] = 0.0; l_install_base_inv_[1][1] = 0.0; l_install_base_inv_[1][2] = -1.0; l_install_base_inv_[1][3] = 1121.0;
    l_install_base_inv_[2][0] = 0.0; l_install_base_inv_[2][1] = 1.0; l_install_base_inv_[2][2] = 0.0; l_install_base_inv_[2][3] = -40.0;
    l_install_base_inv_[3][0] = 0.0; l_install_base_inv_[3][1] = 0.0; l_install_base_inv_[3][2] = 0.0; l_install_base_inv_[3][3] = 1.0;

    r_install_base_inv_[0][0] = -1.0;r_install_base_inv_[0][1] = 0.0;r_install_base_inv_[0][2] = 0.0; r_install_base_inv_[0][3] = 0.0;
    r_install_base_inv_[1][0] = 0.0; r_install_base_inv_[1][1] = 0.0;r_install_base_inv_[1][2] = 1.0; r_install_base_inv_[1][3] = -1121.0;
    r_install_base_inv_[2][0] = 0.0; r_install_base_inv_[2][1] = 1.0;r_install_base_inv_[2][2] = 0.0; r_install_base_inv_[2][3] = 40.0;
    r_install_base_inv_[3][0] = 0.0; r_install_base_inv_[3][1] = 0.0;r_install_base_inv_[3][2] = 0.0; r_install_base_inv_[3][3] = 1.0;
 
    

    // joint_command_timer_ = create_wall_timer(250ms,
    // std::bind(&DualArmModule::joint_command_cb, this));
}

rclcpp::Node::SharedPtr DualArmModule::lockNode(const char *context) const
{
    auto node = node_.lock();
    if (!node) {
        RCLCPP_ERROR(rclcpp::get_logger("dual_arm_module"), "Node expired in %s", context);
    }
    return node;
}

DualArmModule::~DualArmModule()
{
    // 安全退出机制：等待正在执行的线程结束
    if (execution_thread_move_pose_.joinable()) {
        execution_thread_move_pose_.join();
    }
    if (execution_thread_control_gripper_.joinable()) {
        execution_thread_control_gripper_.join();
    }
    RCLCPP_INFO(rclcpp::get_logger("dual_arm_module"),"%s Deconstruct successed", __func__);
}

bool DualArmModule::is_idle(int module_id) const  //根据模块标号返回单个模块是否空闲状态
{
    if(module_id == ID_LEFT_ARM)
    {
        return !is_executing_left_arm_.load();
    }
    else if(module_id == ID_RIGHT_ARM)
    {
        return !is_executing_right_arm_.load();
    }
    else if(module_id == ID_LEFT_TOOL)
    {
        return !is_executing_left_tool_.load();
    }
    else if(module_id == ID_RIGHT_TOOL)
    {
        return !is_executing_right_tool_.load();
    }
    else
    {
        return false;
    }
}

void DualArmModule::start_execution_move_pose(std::shared_ptr<GoalHandleMovePose> goal_handle)
{
    // 如果之前有线程，等待
    if (execution_thread_move_pose_.joinable()) {
        execution_thread_move_pose_.join();
    }
    
    const auto goal = goal_handle->get_goal();
    // 标记为正在执行,
    if(goal->arm_id==ARM_LEFT)
    {
        is_executing_left_arm_.store(true);
    }
    else if(goal->arm_id==ARM_RIGHT)
    {
        is_executing_right_arm_.store(true);
    }
    else if(goal->arm_id==ARM_BOTH)
    {
        is_executing_left_arm_.store(true);
        is_executing_right_arm_.store(true);
    }

    // 启动线程（不再使用 detach）
    execution_thread_move_pose_ = std::thread(&DualArmModule::execution_loop_move_pose, this, goal_handle);
}

void DualArmModule::execution_loop_move_pose(std::shared_ptr<GoalHandleMovePose> goal_handle)
{
    RCLCPP_INFO(rclcpp::get_logger("dual_arm_module"), "Received request to cancel MovePose goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MovePose::Feedback>();
    auto result = std::make_shared<MovePose::Result>();
    std::cout<<"execute_move_pose***"<<std::endl;
    std::cout<<"l_init_arm_params_ := "<<l_init_arm_params_<<std::endl;
    if(!l_init_arm_params_)
    {
        result->success = ACTION_FAIL;
        result->error_message = "Init Robot Arm Paramters failed";
        goal_handle->abort(result);
        is_executing_left_arm_.store(false);
        is_executing_right_arm_.store(false);
        return;
    }  
    RCLCPP_INFO(rclcpp::get_logger("dual_arm_module"), "l init left arm_params");
    if(!init_robot_link_)
    {        
        result->success = ACTION_FAIL;
        result->error_message = " Robot Link failed";
        goal_handle->abort(result);
        is_executing_left_arm_.store(false);
        is_executing_right_arm_.store(false);
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("dual_arm_module"), "init robot link ");
    OnGetBuf(&dcss_);
    int arm_error_a=dcss_.m_State[0].m_ERRCode;
    int arm_state_a=dcss_.m_State[0].m_CurState;
    if (arm_error_a!=0 || arm_state_a==100)
    {
        result->success = ACTION_FAIL;
        result->error_message = " left arm exits error";
        goal_handle->abort(result);
        is_executing_left_arm_.store(false);
        is_executing_right_arm_.store(false);
        return;
    }

    int arm_error_b=dcss_.m_State[1].m_ERRCode;
    int arm_state_b=dcss_.m_State[1].m_CurState;
    if (arm_error_b!=0 || arm_state_b==100)
    {
        result->success = ACTION_FAIL;
        result->error_message = " right arm exits error";
        goal_handle->abort(result);
        is_executing_left_arm_.store(false);
        is_executing_right_arm_.store(false);
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("dual_arm_module"), "arm no error ");

          RCLCPP_INFO(rclcpp::get_logger("dual_arm_module"), " robot no servo errors ");

    for(int i = 0; i < arm_dof_; i++)
    {
        l_current_joint_[i] = dcss_.m_Out[0].m_FB_Joint_Pos[i] ;
        r_current_joint_[i] = dcss_.m_Out[1].m_FB_Joint_Pos[i] ;
        l_sp_.m_Input_IK_RefJoint[i] = l_current_joint_[i];
        r_sp_.m_Input_IK_RefJoint[i] = r_current_joint_[i];
    }
    std::cout<<"start pos:="<<l_current_joint_[0]<<","<<l_current_joint_[1]
      <<","<<l_current_joint_[2]<<","<<l_current_joint_[3]
      <<","<<l_current_joint_[4]<<","<<l_current_joint_[5]
      <<","<<l_current_joint_[6]<<std::endl;
    if(goal->enable_collision_check)
    {
        auto tar_pose = goal->target_pose.pose;
        geometry_msgs::msg::Pose pose;
        auto safety_distance = goal->bound_size;
        pose.position.x = tar_pose.position.x + safety_distance[0];
        pose.position.y = tar_pose.position.y + safety_distance[1];
        pose.position.z = tar_pose.position.z + safety_distance[2];
        pose.orientation.x = tar_pose.orientation.x;
        pose.orientation.y = tar_pose.orientation.y;
        pose.orientation.z = tar_pose.orientation.z;
        pose.orientation.w = tar_pose.orientation.w;
        Matrix4 l_tar_pos,r_tar_pos;
        
        if(goal->arm_id==ARM_LEFT)
        {      
          pose_to_matrix(pose,l_tar_pos);
          Matrix4 l_tcp_;
          matrix_multiply(l_install_base_inv_,l_tar_pos,l_tcp_);
          print_matrix(l_tcp_);
          for (int i = 0; i < 4; i++)
          {
                for (int j = 0; j < 4; j++)
                {
                    l_sp_.m_Input_IK_TargetTCP[i][j] = l_tcp_[i][j];
                }
          }
        //Todo: 任务节点添加目标构型的关节角度
        // double Ret_joints[7]={0.0,50.0,-30.0,30.0,40.0,10.0,10.0};
        // for (long i = 0; i < arm_dof_; i++)
        //   {
        //     l_sp_.m_Input_IK_RefJoint[i] = Ret_joints[i];
        //   }
            l_sp_.m_Input_IK_ZSPType=1;
            l_sp_.m_Input_IK_ZSPPara[0]=l_tar_pos[0][0];
            l_sp_.m_Input_IK_ZSPPara[1]=l_tar_pos[1][0];
            l_sp_.m_Input_IK_ZSPPara[2]=l_tar_pos[2][0];
            l_sp_.m_Input_ZSP_Angle = 0.0;
          if (FX_Robot_Kine_IK(0, &l_sp_) == FX_FALSE)
          {
              result->success = ACTION_FAIL;
              result->error_message = "left arm inverse kinamatics error";
              RCLCPP_ERROR(rclcpp::get_logger("dual_arm_module"), "left arm inverse kinamatics error");
              goal_handle->abort(result);
              is_executing_left_arm_.store(false);
              return;
          }
          RCLCPP_INFO(rclcpp::get_logger("dual_arm_module"), " arm inverse kinamatics no errors ");
          double tar_joints[7]={10.0,10.0,20.0,30.0,40.0,10.0,10.0};
          for (long i = 0; i < arm_dof_; i++)
          {
             tar_joints[i] = l_sp_.m_Output_RetJoint[i];
          }
          std::cout<<"tar_joints:="<<tar_joints[0]<<","<<tar_joints[1]<<","<<tar_joints[2]<<","<<tar_joints[3]<<","<<tar_joints[4]<<","<<tar_joints[5]<<","<<tar_joints[6]<<std::endl;
          //运动规划
          auto& ompl = OMPLWrapper::instance();         
          std::vector<double>q_start(7);
          std::vector<double>q_goal(7);
          for(int i=0;i<arm_dof_;i++)
          {
             q_start[i] = l_current_joint_[i]*DEG2RAD;
             q_goal[i] = tar_joints[i]*DEG2RAD;
          }
         ompl.setJointLimits(l_q_lower_limits_, l_q_upper_limits_);
         ompl.setPlanningGroup(PlanningGroup::LEFT_ARM);
         ompl.setPlanner(OmplPlanner::RRTConnect);
         ompl.setEnableColliderCheck(goal->enable_collision_check);  //enable collide check or not
         auto res = ompl.plan(q_start, q_goal);        
         if(res.success)
         {
            l_current_traj_  = traj_ptr_->plan(res.wayPoints);     
            std::cout<<"ompl res:="<<res.wayPoints.size()<<std::endl;
            if (l_current_traj_.success)
            {
                while(true)
                {   
                    int points = l_current_traj_.positions.size();
                    std::cout<<"toppra res:="<<points<<std::endl;
                    arm_status_mtx_.lock();
                    l_traj_active_ = (arm_status_error_==false)?true:false;
                    arm_status_mtx_.unlock();
                    RCLCPP_INFO(rclcpp::get_logger("dual_arm_module"), " l_traj_active_ = %s\n", l_traj_active_ ? "true" : "false");
                    if(l_traj_active_)
                    {
                        for(int i =0;i<points;i++)
                        {
                            // 检查是否收到了取消请求
                            if (goal_handle->is_canceling()) {
                                // ... 执行机械臂急停操作 ...
                                result->success = ACTION_FAIL;
                                result->error_message = " Goal canceled by client";
                                goal_handle->canceled(result);
                                RCLCPP_INFO(rclcpp::get_logger("dual_arm_module"), "[%s] Goal canceled by client.", name_.c_str());
                                
                                // 结束执行前，释放繁忙标记
                                is_executing_left_arm_.store(false);
                                return;
                            }
                            //下发运动点位
                            double joints_a[7] = {0.0};
                            for(int j=0;j<arm_dof_;j++)
                            {
                                joints_a[j] = l_current_traj_.positions[i][j]*RAD2DEG;
                            }
                            OnClearSet();
                            OnSetJointCmdPos_A(joints_a);
                            OnSetSend();
                            l_step_++;
                            feedback->task_status  = 1;
                            feedback->progress_rate = static_cast<double>(l_step_)/points * 100.0;
                            if (feedback->progress_rate > 100.0)
                            {
                                feedback->progress_rate = 100.0;
                            }
                            else if (feedback->progress_rate < 0.0)
                            {
                                feedback->progress_rate = 0.0;
                            }

                            RCLCPP_INFO(rclcpp::get_logger("dual_arm_module"), " feedback->progress_rate:%.1f, %d, %d\n", feedback->progress_rate, l_step_, points);

                            feedback->current_step_info = "joint command send";
                            if (auto node = lockNode(__func__)) {
                                feedback->header.stamp = node->get_clock()->now();
                            }
                            goal_handle->publish_feedback(feedback);   //要加限流反馈
                            rclcpp::sleep_for(std::chrono::milliseconds(2));
                        } 
                        while(1)
                        {
                            arm_status_mtx_.lock();
                            //RCLCPP_INFO(node_->get_logger(), "dcss_.m_Out[0].m_LowSpdFlag = %d", dcss_.m_Out[0].m_LowSpdFlag);
                            if(dcss_.m_Out[0].m_LowSpdFlag == 1 && dcss_.m_Out[1].m_LowSpdFlag)
                            {
                                arm_status_mtx_.unlock();
                                break;
                            }
                            arm_status_mtx_.unlock();

                            
                        }
                        
                        RCLCPP_INFO(rclcpp::get_logger("dual_arm_module"), " goal_handle->succeed(result)");
                        result->success = ACTION_SUCCESS;
                        goal_handle->succeed(result);
                        // 正常结束，恢复空闲状态
                        is_executing_left_arm_.store(false);
                        break;                       
                    }else{
                        feedback->task_status  = 1;
                        feedback->progress_rate = l_step_/points;
                        feedback->current_step_info = "joint command send";
                        if (auto node = lockNode(__func__)) {
                            feedback->header.stamp = node->get_clock()->now();
                        }
                        goal_handle->publish_feedback(feedback);
                    }
                }

            }else{
                result->success = ACTION_FAIL;
                result->error_message = " traj plan failed";
                goal_handle->abort(result);
                is_executing_left_arm_.store(false);
                return; 
            }
         }else{

            result->success = ACTION_FAIL;
            result->error_message = " ompl plan failed";
            goal_handle->abort(result);
            is_executing_left_arm_.store(false);
            return;
         }
        }else if(goal->arm_id==ARM_RIGHT)
        {
          pose_to_matrix(pose,r_tar_pos);
         for (int i = 0; i < 4; i++)
          {
            for (int j = 0; j < 4; j++)
            {
              r_sp_.m_Input_IK_TargetTCP[i][j] = r_tar_pos[i][j];
            }
          }

         if (FX_Robot_Kine_IK(1, &r_sp_) == FX_FALSE)
          {
              result->success = ACTION_FAIL;
              result->error_message = "right arm inverse kinamatics error";
              goal_handle->abort(result);
              is_executing_right_arm_.store(false);
              return;
          }

        }else if(goal->arm_id==ARM_BOTH)
        {
          
           pose_to_matrix(pose,l_tar_pos);
           geometry_msgs::msg::Pose r_pose;
           r_pose.position.x = pose.position.x + safety_distance[3];
           r_pose.position.y = pose.position.y + safety_distance[4];
           r_pose.position.z = pose.position.z + safety_distance[5];
           r_pose.orientation.x = tar_pose.orientation.x;
           r_pose.orientation.y = tar_pose.orientation.y;
           r_pose.orientation.z = tar_pose.orientation.z;
           r_pose.orientation.w = tar_pose.orientation.w;
           pose_to_matrix(pose,r_tar_pos);

        }
    }
}

void DualArmModule::start_execution_control_gripper(std::shared_ptr<GoalHandleControlGripper> goal_handle)
{
    // 如果之前有执行完的遗留线程，先清理掉
    if (execution_thread_control_gripper_.joinable()) {
        execution_thread_control_gripper_.join();
    }

    const auto goal = goal_handle->get_goal();
    // 标记为正在执行
    if(goal->gripper_id == GRIPPER_LEFT)
    {
        is_executing_left_tool_.store(true);
    }
    else if(goal->gripper_id==GRIPPER_RIGHT)
    {
        is_executing_right_tool_.store(true);
    }
    else if(goal->gripper_id==GRIPPER_BOTH)
    {
        is_executing_left_tool_.store(true);
        is_executing_right_tool_.store(true);
    }

    // 启动线程（不再使用 detach）
    execution_thread_control_gripper_ = std::thread(&DualArmModule::execution_loop_control_gripper, this, goal_handle);
}

void DualArmModule::execution_loop_control_gripper(std::shared_ptr<GoalHandleControlGripper> goal_handle)
{
    //添加机器人开/合抓逻辑   示例
    const auto goal = goal_handle->get_goal();
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    // 起始时间：把msg里的stamp转成rclcpp::Time
    rclcpp::Time start_time(goal->header.stamp);
    const int64_t timeout_ns = static_cast<int64_t>(goal->timeout_ms) * 1000000LL;

    auto feedback = std::make_shared<ControlGripper::Feedback>();
    auto result = std::make_shared<ControlGripper::Result>();
    std::shared_ptr<Jodell_Control> jodell_control = std::make_shared<Jodell_Control>();

    Jodell_status rt_status[2] = {{}, {}};
    int init_position[2] = {0,0};
    

    if (goal->gripper_id == GRIPPER_LEFT)
    {
        jodell_control->Jodell_modbus_config_A(goal->com_port[0], 9);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
        bool ret = jodell_control->Jodell_Status_A(&rt_status[GRIPPER_LEFT]);
        if (ret)
        {
            init_position[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].position;
        }
        else
        {
            result->success = ACTION_FAIL;
            result->error_message = "Get gripper_A status failed";
            RCLCPP_ERROR(rclcpp::get_logger("Jodell_Status_A"), "Get gripper_A status failed");
            goal_handle->abort(result);
            is_executing_left_tool_.store(false);
            return;
        }

        //左爪
        jodell_control->Jodell_Move_Abs_A(goal->position[GRIPPER_LEFT], goal->max_torque[GRIPPER_LEFT], goal->speed[GRIPPER_LEFT]);
        while (true)
        {

            // 休眠20ms

            rclcpp::sleep_for(std::chrono::milliseconds(20));

            /* 超时检查 */
            rclcpp::Time now = ros_clock.now();
            int64_t diff_ns = (now - start_time).nanoseconds();

            if(diff_ns >= timeout_ns)
            {
                RCLCPP_ERROR(rclcpp::get_logger("timeout"), "timeout exit");
                result->success = ACTION_FAIL;
                result->error_message = "time_out";
                result->final_position[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].position;
                result->final_position[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].position;
                result->final_torque[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].torque;
                result->final_torque[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].torque;
                result->success = ACTION_SUCCESS;
                goal_handle->abort(result);
                is_executing_left_tool_.store(false);
                break;
            }
            ret = jodell_control->Jodell_Status_A(&rt_status[GRIPPER_LEFT]);

            if (ret)
            {
                log_jodell_status(rclcpp::get_logger("Jodell_Status_A"), rt_status[GRIPPER_LEFT]);
                if(rt_status[GRIPPER_LEFT].gto == 0)
                {
                    control_gripper_result(rt_status, result, goal_handle);
                    is_executing_left_tool_.store(false);
                    break;
                }
                else
                {
                    feedback->current_position[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].position;
                    feedback->current_torque[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].torque;
                    feedback->current_speed[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].speed;
                    feedback->progress_rate = get_progress_1000(init_position[GRIPPER_LEFT], goal->position[GRIPPER_LEFT], rt_status[GRIPPER_LEFT].position);
                    goal_handle->publish_feedback(feedback);
                }
            }
            

        }
    } 
    else if (goal->gripper_id == GRIPPER_RIGHT)
    {
        jodell_control->Jodell_modbus_config_B(goal->com_port[1], 9);
        bool ret = jodell_control->Jodell_Status_B(&rt_status[GRIPPER_RIGHT]);
        if (ret)
        {
            init_position[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].position;
        }
        else
        {
            result->success = ACTION_FAIL;
            result->error_message = "Get gripper_B status failed";
            RCLCPP_ERROR(rclcpp::get_logger("Jodell_Status_B"), "Get gripper_B status failed");
            goal_handle->abort(result);
            is_executing_right_tool_.store(false);
            return;
        }
        jodell_control->Jodell_Move_Abs_B(goal->position[GRIPPER_RIGHT], goal->max_torque[GRIPPER_RIGHT], goal->speed[GRIPPER_RIGHT]);

        while (true)
        {

            // 休眠20ms

            rclcpp::sleep_for(std::chrono::milliseconds(20));

            /* 超时检查 */
            rclcpp::Time now = ros_clock.now();
            int64_t diff_ns = (now - start_time).nanoseconds();

            if(diff_ns >= timeout_ns)
            {
                RCLCPP_INFO(rclcpp::get_logger("timeout"), "timeout exit");
                result->success = ACTION_FAIL;
                result->error_message = "time_out";
                goal_handle->abort(result);
                is_executing_right_tool_.store(false);
                break;
            }


            ret = jodell_control->Jodell_Status_A(&rt_status[GRIPPER_RIGHT]);

            if (ret)
            {
                if(rt_status[GRIPPER_RIGHT].gto == 0)
                {
                    control_gripper_result(rt_status, result, goal_handle);
                    is_executing_right_tool_.store(false);
                    break;
                }
                else
                {
                    feedback->current_position[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].position;
                    feedback->current_torque[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].torque;
                    feedback->current_speed[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].speed;
                    feedback->progress_rate = get_progress_1000(init_position[GRIPPER_RIGHT], goal->position[GRIPPER_RIGHT], rt_status[GRIPPER_RIGHT].position);
                    goal_handle->publish_feedback(feedback);
                }
            }
           
            

        }
    }
    else if (goal->gripper_id == GRIPPER_BOTH)
    {
        jodell_control->Jodell_modbus_config_A(goal->com_port[0], 9);

        jodell_control->Jodell_modbus_config_B(goal->com_port[1], 9);
        bool ret = jodell_control->Jodell_Status_A(&rt_status[GRIPPER_LEFT]);
        if (ret)
        {
            init_position[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].position;
        }
        else
        {
            result->success = ACTION_FAIL;
            result->error_message = "Get gripper_A status failed";
            RCLCPP_ERROR(rclcpp::get_logger("Jodell_Status_A"), "Get gripper_A status failed");
            //goal_handle->abort(result);
            //return;
        }

        ret = jodell_control->Jodell_Status_B(&rt_status[GRIPPER_RIGHT]);
        if (ret)
        {
            init_position[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].position;
        }
        else
        {
            result->success = ACTION_FAIL;
            result->error_message = "Get gripper_B status failed";
            RCLCPP_ERROR(rclcpp::get_logger("Jodell_Status_B"), "Get gripper_B status failed");
            //goal_handle->abort(result);
            //return;
        }

        jodell_control->Jodell_Move_Abs_A(goal->position[GRIPPER_LEFT], goal->max_torque[GRIPPER_LEFT], goal->speed[GRIPPER_LEFT]);
        
        jodell_control->Jodell_Move_Abs_B(goal->position[GRIPPER_RIGHT], goal->max_torque[GRIPPER_RIGHT], goal->speed[GRIPPER_RIGHT]);

        while (true)
        {

            // 休眠20ms

            rclcpp::sleep_for(std::chrono::milliseconds(20));

            /* 超时检查 */
            rclcpp::Time now = ros_clock.now();
            int64_t diff_ns = (now - start_time).nanoseconds();

            if(diff_ns >= timeout_ns)
            {
                RCLCPP_INFO(rclcpp::get_logger("timeout"), "timeout exit");
                result->success = ACTION_FAIL;
                result->error_message = "time_out";
                goal_handle->abort(result);
                is_executing_left_tool_.store(false);
                is_executing_right_tool_.store(false);
                break;
            }


            ret = jodell_control->Jodell_Status_A(&rt_status[GRIPPER_LEFT]);
            ret &= jodell_control->Jodell_Status_B(&rt_status[GRIPPER_RIGHT]);
            if (ret)
            {
                if(rt_status[GRIPPER_LEFT].gto == 0 && rt_status[GRIPPER_RIGHT].gto == 0)
                {
                    control_gripper_result(rt_status, result, goal_handle);
                    is_executing_left_tool_.store(false);
                    is_executing_right_tool_.store(false);
                    break;
                }
                else
                {
                    feedback->current_position[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].position;
                    feedback->current_torque[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].torque;
                    feedback->current_speed[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].speed;
                    feedback->current_position[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].position;
                    feedback->current_torque[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].torque;
                    feedback->current_speed[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].speed;
                    feedback->progress_rate = get_progress_1000(init_position[GRIPPER_RIGHT], goal->position[GRIPPER_RIGHT], rt_status[GRIPPER_RIGHT].position);
                    goal_handle->publish_feedback(feedback);
                }
            }
           
            

        }
    }
    else
    {
        is_executing_left_tool_.store(false);
        is_executing_right_tool_.store(false);
        return;
    }


}

// -----------------------------------------//

 bool DualArmModule::init_arm_params()
 {
    bool res = false;
    if (LOADMvCfg((char*)"/config/ccs_m6_40.MvKDCfg", type_, grv_, dh_, pnva_, bd_, mass_, mcp_, i_) == FX_FALSE)
    {
         std::cout<<"1*****"<<std::endl;
       return res;
    }
    if (FX_Robot_Init_Type(0, type_[0])&&FX_Robot_Init_Type(1, type_[1]) == FX_FALSE )
    {
         std::cout<<"11*****"<<std::endl;
        return res;
    }
    if(FX_Robot_Init_Kine(0, dh_[0])&&FX_Robot_Init_Kine(1, dh_[1]) == FX_FALSE )
    {
         std::cout<<"12*****"<<std::endl;
        return res;
    }
    if(FX_Robot_Init_Lmt(0, pnva_[0], bd_[0])&&FX_Robot_Init_Lmt(1, pnva_[1], bd_[1]) == FX_FALSE )
    {
         std::cout<<"13*****"<<std::endl;
        return res;
    }

    std::cout<<"14*****"<<std::endl;
    for(int i =0;i<arm_dof_;i++)
    {

      l_q_lower_limits_[i] = pnva_[0][i][1]*DEG2RAD;
      r_q_lower_limits_[i] = pnva_[1][i][1]*DEG2RAD;

      l_q_upper_limits_[i] = pnva_[0][i][0]*DEG2RAD;
      r_q_upper_limits_[i] = pnva_[1][i][0]*DEG2RAD;

      std::cout<<"i:="<<i<<","<<"l:="<< l_q_lower_limits_[i]<<","<<l_q_upper_limits_[i]<<std::endl;
      std::cout<<"i:="<<i<<","<<"r:="<< r_q_lower_limits_[i]<<","<<r_q_upper_limits_[i]<<std::endl;

      l_v_limit_[i] = pnva_[0][i][2]*DEG2RAD;
      r_v_limit_[i] = pnva_[1][i][2]*DEG2RAD;

      std::cout<<"i:="<<i<<","<<"v:="<< pnva_[0][i][2]<<std::endl;
      
      l_a_limit_[i] = pnva_[0][i][3]*DEG2RAD;
      r_a_limit_[i] = pnva_[1][i][3]*DEG2RAD;

      std::cout<<"i:="<<i<<","<<"a:="<< pnva_[0][i][3]<<std::endl;
    

      l_jerk_limit_[i] = 5.0*l_a_limit_[i]*DEG2RAD;
      r_jerk_limit_[i] = 5.0*r_a_limit_[i]*DEG2RAD;

    }
     std::cout<<"15*****"<<std::endl;
    res = true;
    return res;
 }

 void DualArmModule::init_set_tcp()
 {

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (i == j)
            {
                l_tool_[i][j] = 1.0;
                r_tool_[i][j] = 1.0;
            }
            else
            {
                l_tool_[i][j] = 0.0;
                r_tool_[i][j] = 0.0;
            }

        }
    }
    l_tool_[2][3] = 178.5;
    r_tool_[2][3] = 178.5;

    if (FX_Robot_Tool_Set(0, l_tool_) == FX_FALSE)
    {
        printf("Robot Set Left Tool Error\n");
    }
    else
    {
        printf("Robot Set Left Tool Success\n");
    }

    if (FX_Robot_Tool_Set(1, r_tool_) == FX_FALSE)
    {
        printf("Robot Set Right Tool Error\n");
    }
    else
    {
        printf("Robot Set Right Tool Success\n");
    }

 }



void DualArmModule::set_arm_control_mode()
{
    OnClearSet();
    OnSetTargetState_A(1) ;
    OnSetJointLmt_A(10, 10) ;
    OnSetTargetState_B(1) ;
    OnSetJointLmt_B(10, 10) ;
    OnSetSend();
    rclcpp::sleep_for(std::chrono::milliseconds(1));
}



void DualArmModule::log_jodell_status(rclcpp::Logger logger, const Jodell_status& status) {
    RCLCPP_INFO(logger, "=== Jodell Status [0] ===");
    
    // 打印故障位
    RCLCPP_INFO(logger, "Faults: Active[%d] Ctrl[%d] Comm[%d] OC[%d] Volt[%d] En[%d] Temp[%d] Self[%d]",
                status.err_active, status.err_control, status.err_comm,
                status.err_over_current, status.err_voltage, status.err_enable,
                status.err_over_temp, status.err_self);

    // 打印实时数据 (注意 uint8_t/int8_t 需要强转为 int 才能打印数字，否则可能打印字符)
    RCLCPP_INFO(logger, "Data: Pos[%d] Spd[%d] Torque[%d] BusV[%d] Temp[%d C]",
                (int)status.position, (int)status.speed, (int)status.torque,
                (int)status.busvoltage, (int)status.temperature);

    // 打印夹爪状态
    RCLCPP_INFO(logger, "Gripper: Stat[%d] Mode[%d] GTO[%d] State[%d] Obj[%d]",
                (int)status.status, (int)status.mode, (int)status.gto,
                (int)status.state, (int)status.obj);
}



/**
 * @brief 千分制进度 0~1000，支持目标小于初始（反向运动）
 * @param init_val 初始 0~255
 * @param tar_val 目标 0~255
 * @param curr_val 当前 0~255
 * @return progress [0,100.0]
 */
float_t DualArmModule::get_progress_1000(int init_val, int tar_val, int curr_val)
{
    int delta_total = tar_val - init_val;
    // 起止一样直接满载
    if (delta_total == 0)
    {
        return 99.0;
    }

    int delta_curr = curr_val - init_val;
    float_t rate = static_cast<double>(delta_curr) / delta_total;
    float_t progress = rate * 100.0;

    // 限幅0~1000
    if (progress < 0.0) progress = 0.0;
    if (progress > 100.0) progress = 100.0;

    return progress;
}

void DualArmModule::control_gripper_result(Jodell_status *rt_status, std::shared_ptr<ControlGripper::Result> result, const std::shared_ptr<GoalHandleControlGripper> goal_handle)
{
    result->success = ACTION_SUCCESS;
    if (rt_status[GRIPPER_LEFT].err_active || rt_status[GRIPPER_RIGHT].err_active)
    {
        result->success = ACTION_FAIL;
        result->error_message = "err_active";
    }
    
    if (rt_status[GRIPPER_LEFT].err_control || rt_status[GRIPPER_RIGHT].err_control)
    {
        result->success = ACTION_FAIL;
        if (result->error_message.empty())
        {
            result->error_message = "err_control";
        } 
        else 
        {
            result->error_message = +", err_active";
        }
        
    }

    if (rt_status[GRIPPER_LEFT].err_comm || rt_status[GRIPPER_RIGHT].err_comm)
    {
        result->success = ACTION_FAIL;
        if (result->error_message.empty())
        {
            result->error_message = "err_comm";
        } 
        else 
        {
            result->error_message = +", err_comm";
        }
        
    }

    if (rt_status[GRIPPER_LEFT].err_over_current || rt_status[GRIPPER_RIGHT].err_over_current)
    {
        result->success = ACTION_FAIL;
        if (result->error_message.empty())
        {
            result->error_message = "err_over_current";
        } 
        else 
        {
            result->error_message = +", err_over_current";
        }
        
    }

    if (rt_status[GRIPPER_LEFT].err_voltage || rt_status[GRIPPER_RIGHT].err_voltage)
    {
        result->success = ACTION_FAIL;
        if (result->error_message.empty())
        {
            result->error_message = "err_voltage";
        } 
        else 
        {
            result->error_message = +", err_voltage";
        }
        
    }


    if (rt_status[GRIPPER_LEFT].err_voltage || rt_status[GRIPPER_RIGHT].err_voltage)
    {
        result->success = ACTION_FAIL;
        if (result->error_message.empty())
        {
            result->error_message = "err_voltage";
        } 
        else 
        {
            result->error_message = +", err_voltage";
        }
        
    }


    if (rt_status[GRIPPER_LEFT].err_enable || rt_status[GRIPPER_RIGHT].err_voltage)
    {
        result->success = ACTION_FAIL;
        if (result->error_message.empty())
        {
            result->error_message = "err_enable";
        } 
        else 
        {
            result->error_message = +", err_enable";
        }
        
    }


    if (rt_status[GRIPPER_LEFT].err_over_temp || rt_status[GRIPPER_RIGHT].err_voltage)
    {
        result->success = ACTION_FAIL;
        if (result->error_message.empty())
        {
            result->error_message = "err_over_temp";
        } 
        else 
        {
            result->error_message = +", err_over_temp";
        }
        
    }


    if (rt_status[GRIPPER_LEFT].err_self || rt_status[GRIPPER_RIGHT].err_self)
    {
        result->success = ACTION_FAIL;
        if (result->error_message.empty())
        {
            result->error_message = "err_self";
        } 
        else 
        {
            result->error_message = +", err_self";
        }
        
    }

    result->final_position[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].position;
    result->final_position[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].position;
    result->final_torque[GRIPPER_LEFT] = rt_status[GRIPPER_LEFT].torque;
    result->final_torque[GRIPPER_RIGHT] = rt_status[GRIPPER_RIGHT].torque;
    if(!result->error_message.empty())
    {
        result->success = ACTION_FAIL;
        RCLCPP_ERROR(rclcpp::get_logger("control_gripper_result"), "%s", result->error_message.c_str());
        goal_handle->abort(result);
    }
    else
    {
        result->success = ACTION_SUCCESS;
        
        goal_handle->succeed(result);
    }
    
    return;
}
void DualArmModule::heart_beat_cb()
{
    //每隔一秒发送一次心跳报文
    auto node = lockNode(__func__);
    if (!node) {
        return;
    }
    auto hb_msg = com_interfaces::msg::HeartBeat();
    hb_msg.node_id = MOTION_CONTROL;
    hb_msg.header.stamp = node->get_clock()->now();
    heart_beat_pub_->publish(hb_msg);
}
void DualArmModule::init_motor_status()
{
    for(const auto&name :MotorName)
    {
       motor_status_msg_.joint_names.push_back(name);
       motor_status_msg_.joint_positions.push_back(0.0);
       motor_status_msg_.joint_velocities.push_back(0.0);
       motor_status_msg_.joint_efforts.push_back(0.0);
    }
    l_end_effector_pose.position.x = 0.0;
    l_end_effector_pose.position.y = 0.0;
    l_end_effector_pose.position.z = 0.0;
    l_end_effector_pose.orientation.w = 1.0;
    l_end_effector_pose.orientation.x = 0.0;
    l_end_effector_pose.orientation.y = 0.0;
    l_end_effector_pose.orientation.z = 0.0;
    motor_status_msg_.end_effector_pose.push_back(l_end_effector_pose);    

    r_end_effector_pose.position.x = 0.0;
    r_end_effector_pose.position.y = 0.0;
    r_end_effector_pose.position.z = 0.0;
    r_end_effector_pose.orientation.w = 1.0;
    r_end_effector_pose.orientation.x = 0.0;
    r_end_effector_pose.orientation.y = 0.0;
    r_end_effector_pose.orientation.z = 0.0;
    motor_status_msg_.end_effector_pose.push_back(r_end_effector_pose);  
}

geometry_msgs::msg::Pose DualArmModule::matrix_to_pose(const Matrix4& mat)
{
    geometry_msgs::msg::Pose pose;

    pose.position.x = mat[0][3];
    pose.position.y = mat[1][3];
    pose.position.z = mat[2][3];
 
    double r00 = mat[0][0], r01 = mat[0][1], r02 = mat[0][2];
    double r10 = mat[1][0], r11 = mat[1][1], r12 = mat[1][2];
    double r20 = mat[2][0], r21 = mat[2][1], r22 = mat[2][2];
 
    double qw, qx, qy, qz;
    double trace = r00 + r11 + r22;

    if (trace > 0) {
        double s = 0.5 / sqrt(trace + 1.0);
        qw = 0.25 / s;
        qx = (r21 - r12) * s;
        qy = (r02 - r20) * s;
        qz = (r10 - r01) * s;
    } else {
        if (r00 > r11 && r00 > r22) {
            double s = 2.0 * sqrt(1.0 + r00 - r11 - r22);
            qw = (r21 - r12) / s;
            qx = 0.25 * s;
            qy = (r01 + r10) / s;
            qz = (r02 + r20) / s;
        } else if (r11 > r22) {
            double s = 2.0 * sqrt(1.0 + r11 - r00 - r22);
            qw = (r02 - r20) / s;
            qx = (r01 + r10) / s;
            qy = 0.25 * s;
            qz = (r12 + r21) / s;
        } else {
            double s = 2.0 * sqrt(1.0 + r22 - r00 - r11);
            qw = (r10 - r01) / s;
            qx = (r02 + r20) / s;
            qy = (r12 + r21) / s;
            qz = 0.25 * s;
        }
    }

    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;

    return pose;

}

void DualArmModule::pose_to_matrix(const geometry_msgs::msg::Pose& pose,Matrix4& mat)
{
     // 提取位置
    double x = pose.position.x;
    double y = pose.position.y;
    double z = pose.position.z;

    // 提取四元数并归一化（确保单位四元数）
    double qx = pose.orientation.x;
    double qy = pose.orientation.y;
    double qz = pose.orientation.z;
    double qw = pose.orientation.w;
    double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (norm > 0.0) {
        qx /= norm; qy /= norm; qz /= norm; qw /= norm;
    }

    // 计算旋转矩阵元素
    double xx = qx * qx, yy = qy * qy, zz = qz * qz;
    double xy = qx * qy, xz = qx * qz, yz = qy * qz;
    double wx = qw * qx, wy = qw * qy, wz = qw * qz;

    mat[0][0] = 1.0 - 2.0*(yy + zz);
    mat[0][1] = 2.0*(xy - wz);
    mat[0][2] = 2.0*(xz + wy);
    mat[0][3] = x;

    mat[1][0] = 2.0*(xy + wz);
    mat[1][1] = 1.0 - 2.0*(xx + zz);
    mat[1][2] = 2.0*(yz - wx);
    mat[1][3] = y;

    mat[2][0] = 2.0*(xz - wy);
    mat[2][1] = 2.0*(yz + wx);
    mat[2][2] = 1.0 - 2.0*(xx + yy);
    mat[2][3] = z;

    // 最后一行
    mat[3][0] = 0.0;
    mat[3][1] = 0.0;
    mat[3][2] = 0.0;
    mat[3][3] = 1.0;

}

bool DualArmModule::check_robot_statu_error()
{
    bool res = false;
//     int arm_error_a=dcss_.m_State[0].m_ERRCode;
//     int arm_state_a=dcss_.m_State[0].m_CurState;
//     if (arm_error_a!=0 || arm_state_a==100)
//     {
//         res = true;
//         return res;
//     }

//     int arm_error_b=dcss_.m_State[1].m_ERRCode;
//     int arm_state_b=dcss_.m_State[1].m_CurState;
//     if (arm_error_b!=0 || arm_state_b==100)
//     {
//         res = true;
//         return res;
//     }

//     long ErrCode_A[7]={};
//     long ErrCode_B[7]={};
//     OnGetServoErr_A(ErrCode_A);
//     OnGetServoErr_B(ErrCode_B);
//     bool allZero_a = true;
//     bool allZero_b = true;
//     for (int i = 0; i < 7; ++i) 
//     {
//         if (ErrCode_A[i] != 0) {
//             allZero_a = false;
//             break;
//         }
//     }
//     for (int i = 0; i < 7; ++i) 
//     {
//         if (ErrCode_B[i] != 0) {
//             allZero_b = false;
//             break;
//         }
//     }
//    if(allZero_a)
//    {
//        res = true;
//         return res;
//    }

//    if(allZero_b)
//    {
//        res = true;
//         return res;
//    }

    return res;
}

void DualArmModule::motor_statu_cb()
{       
   auto node = lockNode(__func__);
   if (!node) {
     return;
   }
   // 查验连接是否成功
    if (!init_robot_link_) {
     init_motor_status();
     motor_status_msg_.header.stamp = node->get_clock()->now();
     motor_status_pub_->publish(motor_status_msg_);
    }else
    {
      arm_status_mtx_.lock();
      OnGetBuf(&dcss_);
      arm_status_mtx_.unlock();
      //dcss_[0].
      auto is_error = check_robot_statu_error();
      if(is_error)
      {
        arm_status_mtx_.lock();
        arm_status_error_ = true;
        arm_status_mtx_.unlock();
      }
      double current_joints_a[7]={0.0};
      double current_joints_b[7]={0.0};
      for (int i = 0; i < 7; i++)
      {
         current_joints_a[i] = dcss_.m_Out[0].m_FB_Joint_Pos[i];
         current_joints_b[i] = dcss_.m_Out[1].m_FB_Joint_Pos[i];
         motor_status_msg_.joint_positions[i] = dcss_.m_Out[0].m_FB_Joint_Pos[i];
         motor_status_msg_.joint_positions[i+7] = dcss_.m_Out[1].m_FB_Joint_Pos[i];
         motor_status_msg_.joint_velocities[i] = dcss_.m_Out[0].m_FB_Joint_Vel[i];
         motor_status_msg_.joint_velocities[i+7] = dcss_.m_Out[1].m_FB_Joint_Vel[i];
         motor_status_msg_.joint_efforts[i] = dcss_.m_Out[0].m_FB_Joint_SToq[i];
         motor_status_msg_.joint_efforts[i+7] = dcss_.m_Out[1].m_FB_Joint_SToq[i];
      }
        Matrix4 l_end_pos,r_end_pos;
        FX_Robot_Kine_FK(0, current_joints_a, l_end_pos);
        FX_Robot_Kine_FK(1, current_joints_b, r_end_pos);
        
        motor_status_msg_.end_effector_pose[0] = matrix_to_pose(l_end_pos);
        motor_status_msg_.end_effector_pose[1] = matrix_to_pose(r_end_pos);
        motor_status_msg_.header.stamp = node->get_clock()->now();
        motor_status_pub_->publish(motor_status_msg_);
    }
}

void DualArmModule::joint_command_cb()
{

}

 void DualArmModule:: matrix_multiply(const Matrix4& a, const Matrix4& b, Matrix4& result)
 {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result[i][j] = 0;
            for (int k = 0; k < 4; ++k) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
 }
  void DualArmModule::print_matrix(Matrix4 a)
  {
       for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            std::cout << std::fixed << std::setprecision(2) << std::setw(8) << a[i][j] << " ";
        }
        std::cout << std::endl;
    }
  }
