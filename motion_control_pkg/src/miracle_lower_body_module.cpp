/*
#####################################################################################
#
# Copyright (c) 2024 Miracle, Inc. All Rights Reserved.
#
# This software contains confidential and proprietary information of Miracle, Inc.
# ("Confidential Information"). You shall not disclose such Confidential Information
# and shall use it only in accordance with the terms of the license agreement you
# entered into with Miracle, Inc.
#
# UNAUTHORIZED COPYING, USE, OR DISTRIBUTION OF THIS SOFTWARE, OR ANY PORTION OR
# DERIVATIVE THEREOF, IS STRICTLY PROHIBITED. IF YOU HAVE RECEIVED THIS SOFTWARE IN
# ERROR, PLEASE NOTIFY GALBOT, INC. IMMEDIATELY AND DELETE IT FROM YOUR SYSTEM.
#
#####################################################################################
#
# Description: Miracle LowerBodyModule src file
# Version: 1.0
# Date:
# Author: 
# Note: This library is for Miracle driver package only.
#
#####################################################################################
#
# Revision History:
#
# Date       Version  Author       Description
# 2025-8-8   1.0.0    lv
# ---------- -------- ------------ -----------------------------------------------
#
#
*/

#include <mutex>
#include <sched.h>
#include <sys/syscall.h>
#include <thread>
#include <unistd.h>

#include "miracle_lower_body_module.hpp"

// using json = Json;
using namespace miracle;

#define LEG_JOINT_LIMIT_ADJUST_RAD JOINT_LIMIT_ADJUST_RAD
#define LEG_JOINT_MAX_OVER_RAD JOINT_MAX_OVER_RAD

/* accelebraion time max */
#define TRAJECTORY_BUFF_MAX_MS 100
#define TRAJECTORY_LOCK_CPU_NO 0


namespace miracle {
    extern std::string robot_serial_no;
    // static std::mutex sys_mutex;
    // static std::mutex leg_mutex;

    // void *lowerbodyFollowTrajectoryThreadHandle(void *ptr) {

    //     MiracleLowerBodyModule *body_module = (MiracleLowerBodyModule *)ptr;
    //     body_module->followTrajectoryHandle();
    //     pthread_exit(0);
    //     return NULL;
    // }

    MiracleLowerBodyModule::MiracleLowerBodyModule(const char *module_name, const rclcpp::Node::SharedPtr node) : 
                            MiracleModule(module_name, node) {
        auto node_ptr = lockNode(__func__);
        if (!node_ptr) {
            return;
        }
        // function init
        // m_function_map["get_follow_trajectory_status"] = std::bind(&MiracleLowerBodyModule::_getFollowTrajectoryState, this, std::placeholders::_1);
        // m_function_map["follow_trajectory"] = std::bind(&MiracleLowerBodyModule::__setFollowTrajectory, this, std::placeholders::_1);
        // m_function_map["stop_follow_trajectory"] = std::bind(&MiracleLowerBodyModule::_stopFollowTrajectory, this, std::placeholders::_1);
        // m_function_map["get_leg_joint_angles"] = std::bind(&MiracleLowerBodyModule::_getJointAngles, this, std::placeholders::_1);
        // m_function_map["set_leg_joint_angles"] = std::bind(&MiracleLowerBodyModule::__setJointAngles, this, std::placeholders::_1);
        // m_function_map["set_pose"] = std::bind(&MiracleLowerBodyModule::_setPose, this, std::placeholders::_1);


        // data init
        m_module_status.resize(LOWER_BODY_STATUS_MAX);
        m_module_enabled.resize(LEG_JOINTNUM);
        m_joints_min_pos.resize(LEG_JOINTNUM);
        m_joints_max_pos.resize(LEG_JOINTNUM);
        m_joints_cur_pos.resize(LEG_JOINTNUM);
        m_joints_cur_speed.resize(LEG_JOINTNUM);
        m_joints_cur_current.resize(LEG_JOINTNUM);
        m_traj_buffer_pos.resize(LEG_JOINTNUM);
        has_logged_error.resize(LEG_JOINTNUM);

        for (int i = 0; i < LEG_JOINTNUM; i++) {
            m_module_status[i] = 0;
            m_module_enabled[i] = 0;
            m_joints_cur_pos[i] = 0.0;
            m_joints_cur_speed[i] = 0.0;
            m_joints_cur_current[i] = 0.0;
            has_logged_error[i] = false;
        }
    
        m_jointID = {LEG_JOINT1, LEG_JOINT2, LEG_JOINT3, LEG_JOINT4};

        // node_ptr->declare_parameter("leg_chain_start", std::string());
        // node_ptr->declare_parameter("leg_chain_end", std::string());
        node_ptr->declare_parameter("leg_joint_zero", std::vector<int>{0, 0, 0, 0});
        node_ptr->declare_parameter("leg_joint_dir", std::vector<int>{1, 1, 1, 1});


        // node_ptr->get_parameter("leg_chain_start", m_chain_start);
        // node_ptr->get_parameter("leg_chain_end", m_chain_end);
        
        std::vector<long int> temp_joint_zero, temp_joint_dir;

        if(node_ptr->get_parameter("leg_joint_zero", temp_joint_zero)) {
            RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"Got leg joint zero: %ld, %ld, %ld, %ld", temp_joint_zero[0], temp_joint_zero[1], 
                        temp_joint_zero[2], temp_joint_zero[3]);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"Failed to get parameter leg joint zero");
        }

        if(node_ptr->get_parameter("leg_joint_dir", temp_joint_dir)) {
            RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"Got leg joint dir: %ld, %ld, %ld, %ld", temp_joint_dir[0], temp_joint_dir[1], 
                        temp_joint_dir[2], temp_joint_dir[3]);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"Failed to get parameter leg joint dir");
        }

        m_joint_zero.clear();
        m_joint_dir.clear();
        for (long int value : temp_joint_zero) {
            m_joint_zero.push_back(static_cast<int32_t>(value));
        }
        
        for (long int value : temp_joint_dir) {
            m_joint_dir.push_back(static_cast<int8_t>(value));
        }

        // m_cloud_collision_run_stop = false;
        is_executing_leg_.store(false);
    }

    MiracleLowerBodyModule::~MiracleLowerBodyModule() {
        deinitDevices();

        m_recv_thread_stop = true;
        if (m_ethercat_recv_handler && m_ethercat_recv_handler->joinable()) {
            m_ethercat_recv_handler->join();
        }

        if (execution_thread_move_pose_.joinable()) {
            execution_thread_move_pose_.join();
        }
        if (execution_thread_move_joints_.joinable()) {
            execution_thread_move_joints_.join();
        }

        RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s -%s Deconstruct successed", m_module_name, __func__);
    }
    /*
    ** init status msg basic info
    */
    // void MiracleLowerBodyModule::initStatusMsg(robot_msg::msg::MiracleModuleStatus &status_msg) {
    //     status_msg.name.resize(LOWER_BODY_STATUS_MAX);
    //     status_msg.status.resize(LOWER_BODY_STATUS_MAX);
    //     for (int i = 0; i < LEG_JOINTNUM; i++) {
    //         status_msg.name[i] = "joint" + std::to_string(i+1);
    //     }
    //     status_msg.name[LOWER_BODY_STATUS_WORK_STATE] = "work_state";
    //     // TODO:add other state handle
    // }

    void MiracleLowerBodyModule::initGmcaModule(std::shared_ptr<MiracleMAlgo> &miracleAlgo) {
        m_galgo = miracleAlgo;
        m_galgo->getChainJointsLimt(4, m_joints_min_pos, m_joints_max_pos);
        RCLCPP_INFO(rclcpp::get_logger("Lower body module"),"%s m_joints_min_pos:[%f,%f,%f,%f] m_joints_max_pos:[%f,%f,%f,%f]", m_module_name, m_joints_min_pos[0], m_joints_min_pos[1],
                m_joints_min_pos[2], m_joints_min_pos[3], m_joints_max_pos[0], m_joints_max_pos[1], m_joints_max_pos[2], m_joints_max_pos[3]);

    }

    void MiracleLowerBodyModule::initDevices(std::shared_ptr<EthercatBusManager> &bus_manager) {
        RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s +%s", m_module_name, __func__);
        // int len = strlen(com_name);
        // memcpy(m_bus_name, com_name, len > sizeof(m_bus_name) ? sizeof(m_bus_name) : len);

        m_inited = true;
        
        // m_power_manager = power_manager;
        m_bus_manager = bus_manager;

        recvDataThread();

        RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s -%s", m_module_name, __func__);
    }


    void MiracleLowerBodyModule::deinitDevices(void) {

        m_inited = false;

        RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s -%s", m_module_name, __func__);
    }

    
    void MiracleLowerBodyModule::recvDataThread() {
        if (m_ethercat_recv_handler && m_ethercat_recv_handler->joinable()) {
            m_ethercat_recv_handler->join();
        }
        m_ethercat_recv_handler = std::make_shared<std::thread>(std::bind(&MiracleLowerBodyModule::lowerBodyRevCallBack, this));

    }

    void MiracleLowerBodyModule::lowerBodyRevCallBack() {

        int loop_cnt = 0, loop_cnt2 = 0;
        while(!m_recv_thread_stop) {

            getJointPosAndVel(m_joints_cur_pos, m_joints_cur_speed, m_joints_cur_current, m_module_status);
            // m_module_status[LOWER_BODY_STATUS_WORK_STATE] = 0;
    
    
            // RCLCPP_INFO(rclcpp::get_logger("Lower body module"),"%s angle: %f:%f:%f:%f", m_module_name, m_joints_cur_pos[0], m_joints_cur_pos[1], m_joints_cur_pos[2],
            //         m_joints_cur_pos[3]);
            if (m_galgo) {
                m_galgo->updateChainJointsAngle(m_gmca_chain_name, m_joints_cur_pos);
              //  m_galgo->updateChainJointsVelocity(m_gmca_chain_name, m_joints_cur_speed);
            }
            
            //48ms周期
            if(loop_cnt == 0) {
                if (m_joints_data_cb) {
                    m_joints_data_cb(m_joints_cur_pos, m_joints_cur_speed, m_joints_cur_current);
                }
            }

            //500ms周期
            if(loop_cnt2 == 0) {
                
                if (m_module_status_cb) {
                    m_module_status_cb(m_module_status);
                }

            }

            loop_cnt++;
            loop_cnt = loop_cnt%12;
            loop_cnt2++;
            loop_cnt2 = loop_cnt2%125;
            usleep(TRAJ_CYCLE_US);
        }

    }

    bool MiracleLowerBodyModule::getJointPosAndVel(std::vector<double> &actu_pos, std::vector<double> &actu_vel, std::vector<double> &actu_curr, std::vector<uint64_t> &error_code) {
        
        std::vector<DriverSDK::motorActualStruct> status_data;
        if (!m_bus_manager || !m_bus_manager->getMotorActualSnapshot(status_data)) {
            RCLCPP_WARN(rclcpp::get_logger("lower_body_module")," %s: status_data is empty!", m_module_name);
            return false;
        }

        try {
            for(size_t i = 0; i < actu_pos.size(); i++) {

                // int32_t cur_pos = status_data.at(m_jointID[i]).actualPosition;
                // int32_t cur_vel = status_data.at(m_jointID[i]).actualVelocity;
                
                // double rad_pos = m_joint_dir[i] * (cur_pos-m_joint_zero[i]) * (2*M_PI) / m_cnt_per_cycle;
                // double rad_vel = m_joint_dir[i] * cur_vel * (2*M_PI) / m_cnt_per_cycle;

                // actu_pos[i] = rad_pos;
                // actu_vel[i] = rad_vel;            
                actu_pos[i] = status_data[m_jointID[i]].pos;
                actu_vel[i] = status_data[m_jointID[i]].vel;
                actu_curr[i] = status_data[m_jointID[i]].tor;
                
                error_code[i] = status_data[m_jointID[i]].errorCode;
                m_module_enabled[i] = status_data[m_jointID[i]].statusWord;
            }
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("leg_module"),"%s %s get motor data error %s", m_module_name, __func__, e.what());
        }

        
        return true;
    }

    
    bool MiracleLowerBodyModule::setMulJointPos(const std::vector<double> pos, std::vector<uint16_t> ethcat_id) {

        if(pos.size() != ethcat_id.size()) {
            return false;
        }
        std::vector<DriverSDK::motorTargetStruct> motor_target_data(pos.size());
        for (size_t i = 0; i < pos.size(); i++) {
            motor_target_data[i].pos = static_cast<float>(pos[i]);
            motor_target_data[i].vel = 0.0f;
            motor_target_data[i].tor = 0.0f;
            motor_target_data[i].kp = 0.0f;
            motor_target_data[i].kd = 0.0f;
            motor_target_data[i].enabled = 1;
        }
        if (!m_bus_manager || !m_bus_manager->updateMotorTargets(ethcat_id, motor_target_data)) {
            RCLCPP_ERROR(rclcpp::get_logger("leg_module"),"%s %s updateMotorTargets failed", m_module_name, __func__);
            return false;
        }
        
        return true;
    }

    /*
    ** set bms data callback for upper
    // */
    // void MiracleLowerBodyModule::setBmsDataCB(miracleBmsCB cb) {
    //     m_bms_cb = cb;
    // }

    /*
    ** if leg is moving, then this will return true
    **/
    bool MiracleLowerBodyModule::isMoving(void) {
        return m_traj_run;
    }

    //此模板函数要不同 Action 文件（.action）中，下面用到的字段名称必须一模一样
    template <typename ActionT>
    void MiracleLowerBodyModule::followTrajectoryHandle(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle) {

        auto result = std::make_shared<typename ActionT::Result>();
        auto feedback = std::make_shared<typename ActionT::Feedback>();

        m_traj_point_index = 0;
        m_traj_point_cnt = m_traj_pos.size();
        int loop_counter = 0;
        
        RCLCPP_INFO(rclcpp::get_logger("leg_module"),"%s +%s total_cnt %d", m_module_name, __func__, m_traj_point_cnt);
        while (m_traj_point_index < m_traj_point_cnt) {
            if (goal_handle->is_canceling()) {
                // ... 急停操作 ...
                result->success = ACTION_FAIL;
                result->error_message = " Goal canceled by client";
                goal_handle->canceled(result);
                RCLCPP_INFO(rclcpp::get_logger("leg_module"),"%s Goal canceled by client.", m_module_name);
                // 结束执行前，释放繁忙标记
                is_executing_leg_.store(false);
                return;
            }
            for (int i = 0; i < LEG_JOINTNUM; i++) {
                m_traj_buffer_pos[i] = float(m_traj_pos[m_traj_point_index][i]);
            }

            m_traj_point_index++;

            RCLCPP_INFO(rclcpp::get_logger("leg_module"),"%s %d:%d, check:[%f:%f] cur[%f:%f]", m_module_name, m_traj_point_index, m_traj_point_cnt,
                        m_traj_buffer_pos[0], m_traj_buffer_pos[1], m_joints_cur_pos[0], m_joints_cur_pos[1]);

            setMulJointPos(m_traj_buffer_pos, m_jointID);
            
            //限流反馈
            if (loop_counter % 100 == 0) {
                feedback->task_status  = 1;
                feedback->progress_rate = static_cast<float>(m_traj_point_index)/m_traj_point_cnt * 100.0;

                feedback->current_step_info = "joint command";
                if (auto node_ptr = lockNode(__func__)) {
                    feedback->header.stamp = node_ptr->get_clock()->now();
                }
                goal_handle->publish_feedback(feedback);
            }
            loop_counter++;

            usleep(TRAJ_CYCLE_US);   //按ethercat周期发送
        }

        result->success = ACTION_SUCCESS;
        goal_handle->succeed(result);
        // 正常结束，恢复空闲状态
        is_executing_leg_.store(false);
        RCLCPP_INFO(rclcpp::get_logger("leg_module"),"%s -%s %d", m_module_name, __func__, m_traj_point_index);
    }

    // bool MiracleLowerBodyModule::isRunMode(CallbackResult &r) {
    //     if (!m_inited || !m_bus_manager) { // || isError()) {  //TODO 待修改
    //         r.status_code = 20506;
    //         r.message = "腿关节模块未初始化";
    //         return false;
    //     }
    //     return true;
    // }

    bool MiracleLowerBodyModule::isMotorError(){
        for(int i = 0; i < LEG_JOINTNUM; i++) {
            if(m_module_status[i]) {
                return true;
            }
        }
        return false;

    }

    bool MiracleLowerBodyModule::checkJointsAngleOk(std::vector<double> angles) {

        for(int i = 0; i < LEG_JOINTNUM; i++) {
            if((angles[i] >= m_joints_min_pos[i]) && (angles[i] <= m_joints_max_pos[i])) {
                continue;
            } else {
                return false;
            }
        }
        return true; 
    }

    bool MiracleLowerBodyModule::is_idle(int module_id) const
    {
        if(module_id == ID_LEG)
        {
            return !is_executing_leg_.load();
        }
        else
        {
            return false;
        }
    }

    void MiracleLowerBodyModule::start_execution_move_joints(std::shared_ptr<GoalHandleMoveJoints> goal_handle)
    {
        // 如果之前有线程，等待
        if (execution_thread_move_joints_.joinable()) {
            execution_thread_move_joints_.join();
        }
    
        is_executing_leg_.store(true);

        execution_thread_move_joints_ = std::thread(&MiracleLowerBodyModule::execution_loop_move_joints, this, goal_handle);
    }


    void MiracleLowerBodyModule::start_execution_move_pose(std::shared_ptr<GoalHandleMovePose> goal_handle)
    {
        // 如果之前有线程，等待
        if (execution_thread_move_pose_.joinable()) {
            execution_thread_move_pose_.join();
        }
    
        is_executing_leg_.store(true);

        execution_thread_move_pose_ = std::thread(&MiracleLowerBodyModule::execution_loop_move_pose, this, goal_handle);
    }


    void MiracleLowerBodyModule::execution_loop_move_joints(std::shared_ptr<GoalHandleMoveJoints> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveJoints::Feedback>();
        auto result = std::make_shared<MoveJoints::Result>();

        // if (checkPowerState(result)) {
        //     return result;
        // }
        float motion_rate = goal->speed;
        std::vector<double> angles = goal->position;            //rad

        RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s %s MotionRate: %f", m_module_name, __func__, motion_rate);
        
        bool enable_collision = goal->enable_collision_check;

        if (motion_rate > 1.0 || motion_rate < 0.0) {
            RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s motion_rate set error, need to set between (0.0, 1.0)", m_module_name, __func__);
            result->success = ACTION_FAIL;
            result->error_message = "speed out of range";
            goal_handle->abort(result);
            is_executing_leg_.store(false);  
            return ;
        }
        if(angles.size() != LEG_JOINTNUM) {
            RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s param error: joint_angles size is not match", m_module_name, __func__);
            result->success = ACTION_FAIL;
            result->error_message = "joint_angles size error";
            goal_handle->abort(result);
            is_executing_leg_.store(false);  
            return ;
        }

        if (!checkJointsAngleOk(angles)) {
            RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s param error: joint_angles out of range", m_module_name, __func__);
            result->success = ACTION_FAIL;
            result->error_message = "joint angles out of range";
            goal_handle->abort(result);
            is_executing_leg_.store(false);  
        }

        double plan_dt = 1.0 / TRAJ_FREQ;
        double max_velocity = 1.5 * motion_rate;
        double max_acc = 5 * motion_rate;
        double max_jerk = 10 * motion_rate;

        // current info
        std::vector<double> joint_cur_pos(LEG_JOINTNUM);
        std::vector<double> joint_cur_vel(LEG_JOINTNUM);
        std::vector<double> joint_cur_acc(LEG_JOINTNUM);

        for (int i = 0; i < LEG_JOINTNUM; i++) {
            joint_cur_pos[i] = m_joints_cur_pos[i];
            joint_cur_vel[i] = m_joints_cur_speed[i];
            joint_cur_acc[i] = 0.0;                    //...关节没有
        }
        m_traj_point_index = 0;
        RCLCPP_WARN(rclcpp::get_logger("lower_body_module"),"%s %s %d cur_vel:[%f,%f]", m_module_name, __func__, m_traj_point_index, joint_cur_vel[0], joint_cur_vel[1]);
        //     max_acc *= 1.5; 未停止直接重新规划时用
        //     max_jerk *= 1.5;
        // } else {
        bool pos_same = true;
        for (int i = 0; i < LEG_JOINTNUM; i++) {
            if (abs(angles[i] - m_joints_cur_pos[i]) > JOINT_THRESHOLD) {
                pos_same = false;
                break;
            }
        }
        if (pos_same) {
            RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s %s %f from [%f,%f] to [%f,%f] almost same", m_module_name, __func__, motion_rate, m_joints_cur_pos[0], m_joints_cur_pos[1],
                    angles[0], angles[1]);
            result->success = ACTION_SUCCESS;
            result->error_message = "joint angles already at target";
            goal_handle->succeed(result);
            is_executing_leg_.store(false);  
            return;
        }

        try {

            RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s %s %f from [%f,%f,%f,%f] to [%f,%f,%f,%f]", m_module_name, __func__, motion_rate, m_joints_cur_pos[0], m_joints_cur_pos[1], m_joints_cur_pos[2], m_joints_cur_pos[3],
                  angles[0], angles[1], angles[2], angles[3]);

            std::vector<std::vector<double>> traj_pos_new;
            std::vector<std::vector<double>> traj_vel_new;
            std::vector<std::vector<double>> traj_acc_new;

            int all_point_ok = m_galgo->computeMoveJTrajectory<LEG_JOINTNUM>(ID_LEG, plan_dt, max_velocity, max_acc, max_jerk, joint_cur_pos, joint_cur_vel,
                                                               joint_cur_acc, angles, traj_pos_new, traj_vel_new, traj_acc_new);

            if (0 != all_point_ok) {
                RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s Error %d for movej", m_module_name, __func__, all_point_ok);
                result->success = ACTION_FAIL;
                result->error_message = "leg joint trajectory failed, error:" + std::to_string(all_point_ok);
                goal_handle->abort(result);
                is_executing_leg_.store(false);  
                return;
            }

            m_traj_pos = traj_pos_new;
            m_traj_vel = traj_vel_new;
            m_traj_acc = traj_acc_new;
            followTrajectoryHandle(goal_handle);

        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s format %s err:%s", m_module_name, __func__, e.what());
            result->success = ACTION_FAIL;
            result->error_message = "leg module parameter error:" + std::string(e.what());
            goal_handle->abort(result);
            is_executing_leg_.store(false);  
            return;
        } 

    }

    void MiracleLowerBodyModule::execution_loop_move_pose(std::shared_ptr<GoalHandleMovePose> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MovePose::Feedback>();
        auto result = std::make_shared<MovePose::Result>();

        // if (checkPowerState(result)) {
        //     return result;
        // }
        float motion_rate = goal->speed;
        auto& tar_pose = goal->target_pose.pose;

        //腿部4自由度限制，y方向只能为0.0
        std::vector<double> target_pose = {tar_pose.position.x, 0.0, tar_pose.position.z, 
                                          tar_pose.orientation.x, tar_pose.orientation.y, tar_pose.orientation.z, tar_pose.orientation.w};

        bool enable_collision = goal->enable_collision_check;
        
        RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s %s MotionRate: %f", m_module_name, __func__, motion_rate);
        
        if (motion_rate > 1.0 || motion_rate < 0.0) {
            RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s motion_rate set error, need to set between (0.0, 1.0)", m_module_name, __func__);
            result->success = ACTION_FAIL;
            result->error_message = "speed out of range";
            goal_handle->abort(result);
            is_executing_leg_.store(false);  
            return ;
        }
        if(target_pose[3] == 0.0 && target_pose[4] == 0.0 && target_pose[5] == 0.0 && target_pose[6] == 0.0) {
            RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s param error: pose is invalid", m_module_name, __func__);
            result->success = ACTION_FAIL;
            result->error_message = "pose invalid";
            goal_handle->abort(result);
            is_executing_leg_.store(false);  
            return ;
        }

        double plan_dt = 1.0 / TRAJ_FREQ;
        double max_velocity = 1.5 * motion_rate;
        double max_acc = 5 * motion_rate;
        double max_jerk = 10 * motion_rate;

        // current info
        std::vector<double> joint_cur_pos(LEG_JOINTNUM);
        std::vector<double> joint_cur_vel(LEG_JOINTNUM);
        std::vector<double> joint_cur_acc(LEG_JOINTNUM);

        for (int i = 0; i < LEG_JOINTNUM; i++) {
            joint_cur_pos[i] = m_joints_cur_pos[i];
            joint_cur_vel[i] = m_joints_cur_speed[i];
            joint_cur_acc[i] = 0.0;                    //...关节没有
        }

        RCLCPP_WARN(rclcpp::get_logger("lower_body_module"),"%s %s %d cur_vel:[%f,%f, %f, %f]", m_module_name, __func__, m_traj_point_index, joint_cur_vel[0], joint_cur_vel[1], joint_cur_vel[2], joint_cur_vel[3]);
        //     max_acc *= 1.5; 未停止直接重新规划时用
        //     max_jerk *= 1.5;
        // } else {
        try {

            RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s %s %f pose [%f,%f,%f,%f,%f,%f,%f]", m_module_name, __func__, motion_rate, target_pose[0], target_pose[1], target_pose[2], target_pose[3],
                  target_pose[4], target_pose[5], target_pose[6]);

            std::vector<std::vector<double>> traj_pos_new;
            std::vector<std::vector<double>> traj_vel_new;
            std::vector<std::vector<double>> traj_acc_new;

            int all_point_ok = m_galgo->computeMovePTrajectory<LEG_JOINTNUM>(ID_LEG, plan_dt, max_velocity, max_acc, max_jerk, joint_cur_pos, joint_cur_vel,
                                                               joint_cur_acc, target_pose, traj_pos_new, traj_vel_new, traj_acc_new);

            if (0 != all_point_ok) {
                RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s Error %d for moveP", m_module_name, __func__, all_point_ok);
                result->success = ACTION_FAIL;
                result->error_message = "leg joint trajectory failed, error:" + std::to_string(all_point_ok);
                goal_handle->abort(result);
                is_executing_leg_.store(false);  
                return;
            }

            m_traj_pos = traj_pos_new;
            m_traj_vel = traj_vel_new;
            m_traj_acc = traj_acc_new;
            followTrajectoryHandle(goal_handle);

            //测试
            Eigen::Matrix4d T;
            if (m_galgo->getForwardKinematics("base_link", "body_link", T)) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("lower_body_module"), "T_base_body=\n" << T);
            }

        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s format %s err:%s", m_module_name, __func__, e.what());
            result->success = ACTION_FAIL;
            result->error_message = "leg module parameter error:" + std::string(e.what());
            goal_handle->abort(result);
            is_executing_leg_.store(false);  
            return;
        } 

    }



    // CallbackResult MiracleLowerBodyModule::_setPose(const Json::Value &params) {
    //     RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s CALLING %s", m_module_name, __func__);
    //     CallbackResult result;
    //     if (!isRunMode(result)) {
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s isRunMode fail %s", m_module_name, result.message.c_str());
    //         return result;
    //     }
    //     if (checkPowerState(result)) {
    //         return result;
    //     }


    //     double motion_rate = 0.0;

    //     // target info
    //     // bool is_tcp_pose = false;
    //     std::vector<double> target_pose;
    //     std::string chain_startl, chain_endl;
    //     std::vector<std::vector<double>> way_point_poses = {};
    //     bool nearest = false;

    //     if (!params.isMember("parameters") || !params["parameters"].isObject()) {
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s param error: missing parameters", m_module_name, __func__);
    //         result.status_code = 20546;
    //         result.message = "腿关节参数解析错误";
    //         return result;
    //     }

    //     const Json::Value &parameters = params["parameters"];

    //     if (parameters.isMember("nearest") && parameters["nearest"].isBool()) {
    //         nearest = parameters["nearest"].asBool();
    //     } else {
    //         RCLCPP_WARN(rclcpp::get_logger("lower_body_module"),"%s %s no nearest param", m_module_name, __func__);
    //     }

    //     if (parameters.isMember("chain_start") && parameters["chain_start"].isString()) {
    //         chain_startl = parameters["chain_start"].asString();
    //     } else {
    //         chain_startl = m_chain_start;
    //         RCLCPP_WARN(rclcpp::get_logger("lower_body_module"),"%s %s no chain_start param", m_module_name, __func__);
    //     }

    //     if (parameters.isMember("chain_end") && parameters["chain_end"].isString()) {
    //         chain_endl = parameters["chain_end"].asString();
    //     } else {
    //         chain_endl = m_chain_end;
    //         RCLCPP_WARN(rclcpp::get_logger("lower_body_module"),"%s %s no chain_end param", m_module_name, __func__);
    //     }

    //     if (!parameters.isMember("motion_rate") || !parameters["motion_rate"].isNumeric()) {
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s param error: missing motion_rate", m_module_name, __func__);
    //         result.status_code = 20546;
    //         result.message = "腿关节参数解析错误";
    //         return result;
    //     }
    //     motion_rate = parameters["motion_rate"].asDouble();
    //     if (motion_rate > 1.0 || motion_rate < 0.0) {
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s motion_rate set error, need to set between (0.0, 1.0)", m_module_name, __func__);
    //         result.status_code = 20544;
    //         result.message = "腿关节速度超出合法范围";
    //         return result;
    //     }

    //     if (!parameters.isMember("pose") || !parameters["pose"].isArray()) {
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s param error: missing pose", m_module_name, __func__);
    //         result.status_code = 20546;
    //         result.message = "腿关节参数解析错误";
    //         return result;
    //     }
    //     const Json::Value &recv_pose = parameters["pose"];
    //     int pose_size = static_cast<int>(recv_pose.size());
    //     if (pose_size != 7) {
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s param pose size error %d", m_module_name, __func__, pose_size);
    //         result.status_code = 20545;
    //         result.message = "设置的位姿参数数量不匹配";
    //         return result;
    //     }

    //     target_pose.resize(pose_size);
    //     for (Json::ArrayIndex i = 0; i < recv_pose.size(); ++i) {
    //         target_pose[i] = recv_pose[i].asDouble();
    //     }

    //     if (parameters.isMember("way_point_poses")) {
    //         const Json::Value &recv_way_point_poses = parameters["way_point_poses"];
    //         if (!recv_way_point_poses.isNull() && recv_way_point_poses.isArray()) {
    //             int way_point_poses_cnt = static_cast<int>(recv_way_point_poses.size());
    //             way_point_poses.resize(way_point_poses_cnt);
    //             for (Json::ArrayIndex index = 0; index < recv_way_point_poses.size(); ++index) {
    //                 int point_data_len = static_cast<int>(recv_way_point_poses[index].size());
    //                 if (point_data_len != pose_size) {
    //                     RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s param way_point_poses index%d len error:%d", m_module_name, static_cast<int>(index), point_data_len);
    //                     result.status_code = 20545;
    //                     result.message = "设置的位姿参数数量不匹配" + std::to_string(static_cast<int>(index));
    //                     return result;
    //                 }
    //                 way_point_poses[index].resize(point_data_len);
    //                 for (int j = 0; j < point_data_len; ++j) {
    //                     way_point_poses[index][j] = recv_way_point_poses[index][j].asDouble();
    //                 }
    //             }
    //         }
    //     }


    //     // schedule parameter
    //     double plan_dt = 1.0 / TRAJ_FREQ;
    //     double max_velocity = 1.0 * motion_rate;
    //     double max_acc = 1.5 * motion_rate;
    //     double max_jerk = 5 * motion_rate;

    //     // current info
    //     std::vector<double> joint_cur_pos(LEG_JOINTNUM);
    //     std::vector<double> joint_cur_vel(LEG_JOINTNUM);
    //     std::vector<double> joint_cur_acc(LEG_JOINTNUM);

    //     // handle last traj vel and acc
    //     pthread_mutex_lock(&m_traj_mutex);
    //     if (m_traj_run) {
    //         // this is used to stop follow run
    //         // m_traj_cur_stop = true;

    //         // if (m_traj_vel.size() > m_traj_point_index) {
    //         //     for (int i = 0; i < LEG_JOINTNUM; i++) {
    //         //         joint_cur_pos[i] = m_traj_buffer_pos[i];
    //         //         joint_cur_vel[i] = m_traj_vel[m_traj_point_index][i];
    //         //         joint_cur_acc[i] = m_traj_acc[m_traj_point_index][i];
    //         //     }
    //         //     RCLCPP_WARN(rclcpp::get_logger("lower_body_module"),"%s %s %d cur_vel:[%f,%f,%f,%f] cur_acc:[%f,%f,%f,%f]", m_module_name, __func__, m_traj_point_index, joint_cur_vel[0],
    //         //           joint_cur_vel[1], joint_cur_vel[2], joint_cur_vel[3], joint_cur_acc[0],joint_cur_acc[1], joint_cur_acc[2], joint_cur_acc[3]);
    //         // } else {
    //         //     // last traj must be zero vel an acc
    //         //     for (int i = 0; i < LEG_JOINTNUM; i++) {
    //         //         joint_cur_pos[i] = m_traj_buffer_pos[i];
    //         //         joint_cur_vel[i] = 0.0;
    //         //         joint_cur_acc[i] = 0.0;
    //         //     }
    //         // }
    //         for (int i = 0; i < LEG_JOINTNUM; i++) {
    //             joint_cur_pos[i] = m_joints_cur_pos[i];
    //             joint_cur_vel[i] = m_joints_cur_speed[i];
    //             joint_cur_acc[i] = 0.0;                    //有没有
    //         }
    //         max_acc *= 1.5;
    //         max_jerk *= 1.5;
    //     } else {
    //         for (int i = 0; i < LEG_JOINTNUM; i++) {
    //             joint_cur_pos[i] = m_joints_cur_pos[i];
    //             joint_cur_vel[i] = 0.0;
    //             joint_cur_acc[i] = 0.0;
    //         }
    //     }
    //     pthread_mutex_unlock(&m_traj_mutex);

    //     try {
    //         RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s %s %f pose [%f,%f,%f,%f,%f,%f]", m_module_name, __func__, motion_rate, target_pose[0], target_pose[1], target_pose[2], target_pose[3],
    //               target_pose[4], target_pose[5]);

    //         std::vector<std::vector<double>> traj_pos_new;
    //         std::vector<std::vector<double>> traj_vel_new;
    //         std::vector<std::vector<double>> traj_acc_new;

    //         int all_point_ok;
    //         if (0 == way_point_poses.size()) {
    //             all_point_ok = m_galgo->computeMovePTrajectory<LEG_JOINTNUM>(chain_startl, chain_endl, plan_dt, max_velocity, max_acc, max_jerk, joint_cur_pos, joint_cur_vel,
    //                                                            joint_cur_acc, target_pose, traj_pos_new, traj_vel_new, traj_acc_new, nearest);
    //         } else {
    //             RCLCPP_WARN(rclcpp::get_logger("lower_body_module"),"%s %s way_point param is not support in move_pose", m_module_name, __func__);
    //             result.status_code = 1;
    //             result.message = "now not support way_point";
    //             return result;
    //         }
    //         if (0 != all_point_ok) {
    //             RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s Error %d for movep", m_module_name, __func__, all_point_ok);
    //             result.status_code = 20543;
    //             result.message = "位姿设置失败" + std::to_string(all_point_ok);
    //             return result;
    //         }

    //         // int pos_cnt = traj_pos_new.size();
    //         // for (int i = 0; i < pos_cnt; i++) {
    //         //     RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"------------------------------------[%f,%f,%f,%f,%f,%f,%f]", float(traj_pos_new[i][0]), float(traj_pos_new[i][1]),
    //         //           float(traj_pos_new[i][2]), float(traj_pos_new[i][3]), float(traj_pos_new[i][4]), float(traj_pos_new[i][5]), float(traj_pos_new[i][6]));
    //         // }

    //         pthread_mutex_lock(&m_traj_mutex);
    //         // Generate the trajectory within the control loop
    //         m_traj_pos = traj_pos_new;
    //         m_traj_vel = traj_vel_new;
    //         m_traj_acc = traj_acc_new;
    //         if (!m_traj_run) {

    //             m_traj_run = true;
    //             // excute thread
    //             pthread_t Leg_tid;
    //             pthread_attr_t attr;
    //             struct sched_param param;
    //             param.sched_priority = 40;

    //             pthread_attr_init(&attr);
    //             pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    //             pthread_attr_setschedpolicy(&attr, SCHED_RR);

    //             pthread_attr_setschedparam(&attr, &param);

    //             if (0 != pthread_create(&Leg_tid, &attr, lowerbodyFollowTrajectoryThreadHandle, this)) {
    //                 m_traj_run = false;
    //                 pthread_mutex_unlock(&m_traj_mutex);
    //                 result.status_code = 1;
    //                 result.message = "Error create thread";
    //                 return result;
    //             }
    //         } else {
    //             // notify follow thread to stop
    //             m_traj_point_index = 0; // add this for bug when stop a new trajectory
    //             m_traj_run_new = true;
    //         }
    //         pthread_mutex_unlock(&m_traj_mutex);

    //         result.status_code = 0;
    //         result.message = "执行成功";
    //         return result;
    //     } catch (const std::exception &e) {
    //         pthread_mutex_unlock(&m_traj_mutex);
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s excute err:%s", m_module_name, __func__, e.what());
    //         result.status_code = 20546;
    //         result.message = "相关关节参数解析错误" + std::string(e.what());
    //         return result;
    //     }
    // }
    


    // CallbackResult MiracleLowerBodyModule::__handleMoveJTrajectory(const std::vector<double> joint_target_pos, double motion_rate) {

    //     CallbackResult result;

    //     // schedule parameter
    //     double plan_dt = 1.0 / TRAJ_FREQ;
    //     double max_velocity = 1.0 * motion_rate;
    //     double max_acc = 1.5 * motion_rate;
    //     double max_jerk = 5 * motion_rate;

    //     // current info
    //     std::vector<double> joint_cur_pos(LEG_JOINTNUM);
    //     std::vector<double> joint_cur_vel(LEG_JOINTNUM);
    //     std::vector<double> joint_cur_acc(LEG_JOINTNUM);

    //     // handle last traj vel and acc
    //     pthread_mutex_lock(&m_traj_mutex);
    //     if (m_traj_run) {
    //         // this is used to stop follow run
    //         // m_traj_cur_stop = true;

    //         // if (m_traj_vel.size() > m_traj_point_index) {
    //         //     for (int i = 0; i < LEG_JOINTNUM; i++) {
    //         //         joint_cur_pos[i] = m_traj_buffer_pos[i];
    //         //         joint_cur_vel[i] = m_traj_vel[m_traj_point_index][i];
    //         //         joint_cur_acc[i] = m_traj_acc[m_traj_point_index][i];
    //         //     }
    //         //     RCLCPP_WARN(rclcpp::get_logger("lower_body_module"),"%s %s %d cur_vel:[%f,%f,%f,%f] cur_acc:[%f,%f,%f,%f]", m_module_name, __func__, m_traj_point_index, joint_cur_vel[0], joint_cur_vel[1],
    //         //           joint_cur_vel[2], joint_cur_vel[3], joint_cur_acc[0], joint_cur_acc[1], joint_cur_acc[2], joint_cur_acc[3]);
    //         // } else {
    //         //     // last traj must be zero vel an acc
    //         //     for (int i = 0; i < LEG_JOINTNUM; i++) {
    //         //         joint_cur_pos[i] = m_traj_buffer_pos[i];
    //         //         joint_cur_vel[i] = 0.0;
    //         //         joint_cur_acc[i] = 0.0;
    //         //     }
    //         // }
    //         for (int i = 0; i < LEG_JOINTNUM; i++) {
    //             joint_cur_pos[i] = m_joints_cur_pos[i];
    //             joint_cur_vel[i] = m_joints_cur_speed[i];
    //             joint_cur_acc[i] = 0.0;                    //有没有
    //         }
    //         max_acc *= 1.5;
    //         max_jerk *= 1.5;
    //     } else {
    //         bool pos_same = true;
    //         for (int i = 0; i < LEG_JOINTNUM; i++) {
    //             if (abs(joint_target_pos[i] - m_joints_cur_pos[i]) > JOINT_THRESHOLD) {
    //                 pos_same = false;
    //                 break;
    //             }
    //         }
    //         if (pos_same) {
    //             pthread_mutex_unlock(&m_traj_mutex);
    //             RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s %s %f from [%f,%f,%f,%f] to [%f,%f,%f,%f] almost same", m_module_name, __func__, motion_rate, m_joints_cur_pos[0],
    //                   m_joints_cur_pos[1], m_joints_cur_pos[2], m_joints_cur_pos[3], joint_target_pos[0], joint_target_pos[1], joint_target_pos[2],
    //                   joint_target_pos[3]);
    //             result.status_code = 0;
    //             result.message = "执行成功";
    //             return result;
    //         }
    //         for (int i = 0; i < LEG_JOINTNUM; i++) {
    //             joint_cur_pos[i] = m_joints_cur_pos[i];
    //             joint_cur_vel[i] = 0.0;
    //             joint_cur_acc[i] = 0.0;
    //         }
    //     }
    //     pthread_mutex_unlock(&m_traj_mutex);

    //     try {

    //         RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s %s %f from [%f,%f,%f,%f] to [%f,%f,%f,%f]", m_module_name, __func__, motion_rate, m_joints_cur_pos[0], m_joints_cur_pos[1],
    //               m_joints_cur_pos[2], m_joints_cur_pos[3], joint_target_pos[0], joint_target_pos[1], joint_target_pos[2], joint_target_pos[3]);

    //         std::vector<std::vector<double>> traj_pos_new;
    //         std::vector<std::vector<double>> traj_vel_new;
    //         std::vector<std::vector<double>> traj_acc_new;

    //         int all_point_ok = m_galgo->computeMoveJTrajectory<LEG_JOINTNUM>(plan_dt, max_velocity, max_acc, max_jerk, joint_cur_pos, joint_cur_vel,
    //                                                         joint_cur_acc, joint_target_pos, traj_pos_new, traj_vel_new, traj_acc_new,m_module_name);

    //         if (0 != all_point_ok) {
    //             RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s Error %d for movej", m_module_name, __func__, all_point_ok);
    //             result.status_code = 20502;
    //             result.message = "腿关节设置失败" + std::to_string(all_point_ok);
    //             return result;
    //         }

    //         pthread_mutex_lock(&m_traj_mutex);
    //         m_traj_pos = traj_pos_new;
    //         m_traj_vel = traj_vel_new;
    //         m_traj_acc = traj_acc_new;
    //         if (!m_traj_run) {

    //             m_traj_run = true;
    //             // excute thread
    //             pthread_t lowerBody_tid;
    //             pthread_attr_t attr;
    //             struct sched_param param;
    //             param.sched_priority = 40;

    //             pthread_attr_init(&attr);
    //             pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    //             pthread_attr_setschedpolicy(&attr, SCHED_RR);
    //             pthread_attr_setschedparam(&attr, &param);
    //             if (0 != pthread_create(&lowerBody_tid, &attr, lowerbodyFollowTrajectoryThreadHandle, this)) {
    //                 m_traj_run = false;
    //                 pthread_mutex_unlock(&m_traj_mutex);
    //                 result.status_code = 1;
    //                 result.message = "Error create thread";
    //                 return result;
    //             }
    //         } else {
    //             // notify follow thread to stop
    //             m_traj_point_index = 0; // add this for bug when stop a new trajectory
    //             m_traj_run_new = true;
    //         }
    //         pthread_mutex_unlock(&m_traj_mutex);

    //         result.status_code = 0;
    //         result.message = "执行成功";
    //         return result;
    //     } catch (const std::exception &e) {
    //         pthread_mutex_unlock(&m_traj_mutex);
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s format %s err:%s", m_module_name, __func__, e.what());
    //         result.status_code = 20505;
    //         result.message = "腿关节参数解析错误" + std::string(e.what());
    //         return result;
    //     }
    // }


    // CallbackResult MiracleLowerBodyModule::__setJointAngles(const Json::Value &params) {
    //     RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s CALLING %s", m_module_name, __func__);
    //     CallbackResult result;

    //     if (!isRunMode(result)) {
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s isRunMode fail %s", m_module_name, result.message.c_str());
    //         return result;
    //     }
    //     if (checkPowerState(result)) {
    //         return result;
    //     }

    //     std::vector<double> joint_target_pos(LEG_JOINTNUM);
    //     double motion_rate = 0.1;
    //     if (!params.isMember("parameters") || !params["parameters"].isObject()) {
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s param error: missing parameters", m_module_name, __func__);
    //         result.status_code = 20505;
    //         result.message = "腿关节参数解析错误";
    //         return result;
    //     }
    //     const Json::Value &parameters = params["parameters"];

    //     if (!parameters.isMember("speed") || !parameters["speed"].isNumeric()) {
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s param error: missing speed", m_module_name, __func__);
    //         result.status_code = 20505;
    //         result.message = "腿关节参数解析错误";
    //         return result;
    //     }
    //     motion_rate = parameters["speed"].asDouble();
    //     if (motion_rate > 1.0 || motion_rate < 0.0) {
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s motion_rate set error, need to set between (0.0, 1.0)", m_module_name, __func__);
    //         result.status_code = 20503;
    //         result.message = "腿关节速度超出合法范围";
    //         return result;
    //     }

    //     if (!parameters.isMember("joint_angles") || !parameters["joint_angles"].isArray()) {
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s param error: missing joint_angles", m_module_name, __func__);
    //         result.status_code = 20505;
    //         result.message = "腿关节参数解析错误";
    //         return result;
    //     }
    //     const Json::Value &joint_angles = parameters["joint_angles"];
    //     if (static_cast<unsigned int>(joint_angles.size()) < LEG_JOINTNUM) {
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s %s param error: joint_angles size mismatch", m_module_name, __func__);
    //         result.status_code = 20505;
    //         result.message = "腿关节参数解析错误";
    //         return result;
    //     }

    //     for (unsigned int i = 0; i < LEG_JOINTNUM; ++i) {
    //         joint_target_pos[i] = joint_angles[static_cast<Json::ArrayIndex>(i)].asDouble();
    //         if ((joint_target_pos[i] > m_joints_max_pos[i] + LEG_JOINT_MAX_OVER_RAD) || (joint_target_pos[i] < m_joints_min_pos[i] - LEG_JOINT_MAX_OVER_RAD)) {
    //             RCLCPP_WARN(rclcpp::get_logger("leg_module"),"%s %s joint%d %f over limit", m_module_name, __func__, i, joint_target_pos[i]);
    //             result.status_code = 20504;
    //             result.message = "腿关节" + std::to_string(i) + "角度" + std::to_string(double(joint_target_pos[i])) + "超出物理限制";
    //             return result;
    //         }
    //     }

    //     // int status = _checkLegSafety(joint_target_pos);
    //     // if (status != 0) {
    //     //     RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s format leg safety err: %d", m_module_name, status);
    //     //     result.status_code = 1;
    //     //     result.message = "Error: check leg safety error" + std::to_string(status);
    //     //     return result;
    //     // }

    //     return __handleMoveJTrajectory(joint_target_pos, motion_rate);
    // }

    // CallbackResult MiracleLowerBodyModule::_getJointAngles(const Json::Value &params) {
    //     RCLCPP_DEBUG(rclcpp::get_logger("lower_body_module"),"%s CALLING %s", m_module_name, __func__);
    //     CallbackResult result;
    //     if (!isRunMode(result)) {
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s isRunMode fail %s", m_module_name, result.message.c_str());
    //         return result;
    //     }

    //     Json::Value data;
    //     Json::Value angles(Json::arrayValue);
    //     for (int i = 0; i < LEG_JOINTNUM; i++) {
            
    //         // if (m_joints_cur_pos[i] > m_joints_max_pos[i]) {
    //         //     if ((m_joints_cur_pos[i] - m_joints_max_pos[i]) > LEG_JOINT_MAX_OVER_RAD) {
    //         //         angles.push_back(m_joints_cur_pos[i]);
    //         //     } else {
    //         //         angles.push_back(m_joints_max_pos[i]);
    //         //     }
    //         // } else if (m_joints_cur_pos[i] < m_joints_min_pos[i]) {
    //         //     if ((m_joints_min_pos[i] - m_joints_cur_pos[i]) > LEG_JOINT_MAX_OVER_RAD) {
    //         //         angles.push_back(m_joints_cur_pos[i]);
    //         //     } else {
    //         //         angles.push_back(m_joints_min_pos[i]);
    //         //     }
    //         // } else {
    //         //     angles.push_back(m_joints_cur_pos[i]);
    //         // }
    //         angles.append(m_joints_cur_pos[i]);
    //     }
    //     data["joint_angles"] = angles;
    //     result.status_code = 0;
    //     result.data = data;
    //     result.message = "leg joint angle retrieved successfully.";
    //     return result;
    // }

    // CallbackResult MiracleLowerBodyModule::_getFollowTrajectoryState(const Json::Value &params) {
    //     RCLCPP_DEBUG(rclcpp::get_logger("lower_body_module"),"%s CALLING %s", m_module_name, __func__);
    //     CallbackResult result;
    //     if (!isRunMode(result)) {
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s isRunMode fail %s", m_module_name, result.message.c_str());
    //         return result;
    //     }
    //     try {
    //         Json::Value data;
    //         // if (m_6dof_collision) {
    //         //     data["follow_trajectory_status"] = 0;
    //         //     data["stop_reason"] = 1; // collision stop
    //         // } else {
    //             data["follow_trajectory_status"] = isMoving();
    //             data["stop_reason"] = 0;
    //         // }
    //         result.data = data;
    //     } catch (const std::exception &e) {
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"Error generating lowerBody follow trajectory response: %s", e.what());
    //         result.status_code = 20603;
    //         result.message = "轨迹状态获取失败" + std::string(e.what());
    //         return result;
    //     }

    //     result.status_code = 0;
    //     result.message = "执行成功";
    //     return result;
    // }
    

    // CallbackResult MiracleLowerBodyModule::__setFollowTrajectory(const Json::Value &params) {
    //     RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s CALLING %s", m_module_name, __func__);
    //     CallbackResult result;

    //     // if (!isRunMode(result)) {
    //     //     RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s isRunMode fail %s", m_module_name, result.message.c_str());
    //     //     return result;
    //     // }
    //     // if (checkPowerState(result)) {
    //     //     return result;
    //     // }

    //     // double traj_freq;
    //     // json traj_pos;
    //     // try {
    //     //     traj_freq = params.at("parameters").at("freq");
    //     //     traj_pos = params.at("parameters").at("follow_list");
    //     //     int total_pos_cnt = traj_pos.size();
    //     //     for (int i = 0; i < total_pos_cnt; i++) {
    //     //         for (int j = 0; j < LEG_JOINTNUM; j++) {
    //     //             if ((traj_pos[i][j] > m_joints_max_pos[j] + LEG_JOINT_MAX_OVER_RAD) || (traj_pos[i][j] < m_joints_min_pos[j] - LEG_JOINT_MAX_OVER_RAD)) {
    //     //                 RCLCPP_WARN(rclcpp::get_logger("lower_body_module"),"%s %s %d point 0 over limit", m_module_name, __func__, i);
    //     //                 result.status_code = 1;
    //     //                 result.message = "Error: point" + std::to_string(i) + "joint" + std::to_string(j) + "over limit";
    //     //                 return result;
    //     //             }
    //     //         }
    //     //     }
    //     // } catch (const std::exception &e) {
    //     //     RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s format %s err:%s", m_module_name, e.what());
    //     //     result.status_code = 1;
    //     //     result.message = "Error: " + std::string(e.what());
    //     //     return result;
    //     // }

    //     // pthread_mutex_lock(&m_traj_mutex);
    //     // m_traj_freq = traj_freq;
    //     // m_traj_pos = traj_pos;

    //     // if (!m_traj_run) {
    //     //     m_traj_run = true;
    //     //     // excute thread
    //     //     pthread_t lowerBody_tid;
    //     //     pthread_attr_t attr;
    //     //     pthread_attr_init(&attr);
    //     //     pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    //     //     if (0 != pthread_create(&lowerBody_tid, &attr, lowerbodyFollowTrajectoryThreadHandle, this)) {
    //     //         m_traj_run = false;
    //     //         pthread_mutex_unlock(&m_traj_mutex);
    //     //         result.status_code = 1;
    //     //         result.message = "Error create thread";
    //     //         return result;
    //     //     }
    //     // } else {
    //     //     // notify follow thread to stop
    //     //     m_traj_point_index = 0; // add this for bug when stop a new trajectory
    //     //     m_traj_run_new = true;
    //     // }
    //     // pthread_mutex_unlock(&m_traj_mutex);

    //     // result.status_code = 0;
    //     // result.message = "follow trajectory start successfully.";

    //     result.status_code = 1;
    //     result.message = "Leg moudle follow trajectory do not realize.";
    //     return result;
    // }

    // CallbackResult MiracleLowerBodyModule::_stopFollowTrajectory(const Json::Value &params) {
    //     RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s CALLING %s", m_module_name, __func__);
    //     CallbackResult result;
    //     result.status_code = 0;

    //     if (!isRunMode(result)) {
    //         RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s isRunMode fail %s", m_module_name, result.message.c_str());
    //         return result;
    //     }
    //     result.message = "lower body stop follow trajectory successfully.";

    //     pthread_mutex_lock(&m_traj_mutex);
    //     if (m_traj_run) {             //原为!m_traj_run
    //         m_traj_run_stop = true;
    //     }
    //     pthread_mutex_unlock(&m_traj_mutex);
    //     int retry_times = 0;
    //     while (m_traj_run) {          //原为!m_traj_run
    //         retry_times++;
    //         if (retry_times > 30) {
    //             result.status_code = 1;
    //             result.message = "lower body stop follow trajectory overtime.";
    //             break;
    //         }
    //         usleep(20000);
    //     }
    //     return result;
    // }

} // namespace miracle
