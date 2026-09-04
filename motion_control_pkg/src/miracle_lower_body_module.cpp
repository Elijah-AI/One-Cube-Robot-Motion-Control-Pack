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
#include <algorithm>

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
    
        m_jointID = {LEG_JOINT1, LEG_JOINT2, LEG_JOINT3, BODY_JOINT};

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

        joint_state_pub_ = node_ptr->create_publisher<sensor_msgs::msg::JointState>("/leg_joint_states", 10);
        joint_state_msg_.name = {"leg_joint1", "leg_joint2", "leg_joint3", "body_joint"};
        joint_state_msg_.position.resize(LEG_JOINTNUM, 0.0);
        joint_state_msg_.velocity.resize(LEG_JOINTNUM, 0.0);
        joint_state_msg_.effort.resize(LEG_JOINTNUM, 0.0);

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

    void MiracleLowerBodyModule::publishJointState() {
        if (!joint_state_pub_) {
            return;
        }

        auto node_ptr = lockNode(__func__);
        if (!node_ptr) {
            return;
        }

        joint_state_msg_.header.stamp = node_ptr->get_clock()->now();
        joint_state_msg_.position = m_joints_cur_pos;
        joint_state_msg_.velocity = m_joints_cur_speed;
        joint_state_msg_.effort = m_joints_cur_current;
        joint_state_pub_->publish(joint_state_msg_);
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
                publishJointState();
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
    void MiracleLowerBodyModule::followTrajectoryHandle(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle,
        const std::chrono::steady_clock::time_point& action_started_at,
        double ik_time_seconds,
        double planning_time_seconds) {

        auto result = std::make_shared<typename ActionT::Result>();
        auto feedback = std::make_shared<typename ActionT::Feedback>();
        const auto motion_started_at = std::chrono::steady_clock::now();
        const auto update_timing = [&]() {
            const auto now = std::chrono::steady_clock::now();
            result->timing_breakdown_valid = true;
            result->ik_time = std::max(0.0, ik_time_seconds);
            result->planning_time = std::max(0.0, planning_time_seconds);
            result->motion_time = std::chrono::duration<double>(
                now - motion_started_at).count();
            result->execution_time = std::chrono::duration<double>(
                now - action_started_at).count();
        };

        m_traj_point_index = 0;
        m_traj_point_cnt = m_traj_pos.size();
        int loop_counter = 0;
        
        RCLCPP_INFO(rclcpp::get_logger("leg_module"),"%s +%s total_cnt %d", m_module_name, __func__, m_traj_point_cnt);
        while (m_traj_point_index < m_traj_point_cnt) {
            if (goal_handle->is_canceling()) {
                // ... 急停操作 ...
                result->success = ACTION_FAIL;
                result->error_message = " Goal canceled by client";
                update_timing();
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

            if (!setMulJointPos(m_traj_buffer_pos, m_jointID)) {
                result->success = ACTION_FAIL;
                result->error_message = "leg motor command rejected";
                update_timing();
                goal_handle->abort(result);
                is_executing_leg_.store(false);
                return;
            }
            
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

        double final_error = 0.0;
        std::vector<DriverSDK::motorActualStruct> actual_data;
        if (m_bus_manager && m_bus_manager->getMotorActualSnapshot(actual_data)) {
            for (int i = 0; i < LEG_JOINTNUM; ++i) {
                const auto &actual = actual_data[m_jointID[i]];
                final_error = std::max(
                    final_error,
                    std::abs(static_cast<double>(m_traj_buffer_pos[i]) - actual.pos));
            }
        } else {
            for (int i = 0; i < LEG_JOINTNUM; ++i) {
                final_error = std::max(
                    final_error,
                    std::abs(static_cast<double>(m_traj_buffer_pos[i]) - m_joints_cur_pos[i]));
            }
        }
        result->final_distance_error = final_error;
        update_timing();
        if (final_error > JOINT_REACH_THRESHOLD) {
            result->success = ACTION_FAIL;
            result->error_message = "legs failed to reach final position";
            goal_handle->abort(result);
        } else {
            result->success = ACTION_SUCCESS;
            result->error_message = "legs reached target";
            goal_handle->succeed(result);
        }

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

    bool MiracleLowerBodyModule::execute_preplanned_trajectory(
        const std::vector<std::vector<double>>& positions,
        const std::chrono::steady_clock::time_point& start_time,
        const std::shared_ptr<motion_control_pkg::execution::CancellationToken>& cancel,
        std::string& error_message)
    {
        if (positions.empty()) {
            error_message = "preplanned leg trajectory is empty";
            return false;
        }
        for (const auto& point : positions) {
            if (point.size() != LEG_JOINTNUM ||
                !std::all_of(point.begin(), point.end(), [](double value) {
                    return std::isfinite(value);
                }))
            {
                error_message = "preplanned leg trajectory must contain four finite joints";
                return false;
            }
        }
        if (cancel && cancel->requested()) {
            error_message = "preplanned leg trajectory canceled before execution";
            return false;
        }

        for (const auto& point : positions) {
            if (!checkJointsAngleOk(point)) {
                error_message = "preplanned leg trajectory violates joint limits";
                return false;
            }
        }
        if (!m_bus_manager || !m_bus_manager->enableMotors(m_jointID)) {
            error_message = "leg motor enable safety check failed";
            return false;
        }

        m_traj_run = true;
        std::this_thread::sleep_until(start_time);
        for (std::size_t index = 0; index < positions.size(); ++index) {
            if (cancel && cancel->requested()) {
                error_message = "preplanned leg trajectory canceled";
                m_traj_run = false;
                return false;
            }
            m_traj_buffer_pos = positions[index];
            RCLCPP_INFO(
                rclcpp::get_logger("leg_module"),
                "%s preplanned command %zu/%zu: joint1=%.9f, joint2=%.9f, "
                "joint3=%.9f, body_joint=%.9f",
                m_module_name, index + 1U, positions.size(),
                m_traj_buffer_pos[0], m_traj_buffer_pos[1],
                m_traj_buffer_pos[2], m_traj_buffer_pos[3]);
            if (!setMulJointPos(m_traj_buffer_pos, m_jointID)) {
                error_message = "leg motor command rejected";
                m_traj_run = false;
                return false;
            }
            if (index + 1U < positions.size()) {
                std::this_thread::sleep_until(
                    start_time + std::chrono::milliseconds(4) *
                    static_cast<long long>(index + 1U));
            }
        }
        m_traj_run = false;
        error_message.clear();
        return true;
    }

    bool MiracleLowerBodyModule::reserve_preplanned_trajectory_resource(
        std::string& error_message)
    {
        bool expected = false;
        if (!is_executing_leg_.compare_exchange_strong(expected, true)) {
            error_message = "leg is already occupied by another task";
            return false;
        }
        error_message.clear();
        return true;
    }

    void MiracleLowerBodyModule::release_preplanned_trajectory_resource() noexcept
    {
        is_executing_leg_.store(false);
    }


    void MiracleLowerBodyModule::execution_loop_move_joints(std::shared_ptr<GoalHandleMoveJoints> goal_handle)
    {
        const auto action_started_at = std::chrono::steady_clock::now();
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
            return;
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

        if (!m_bus_manager || !m_bus_manager->enableMotors(m_jointID)) {
            RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),
                         "%s refused to start: motor enable safety check failed", m_module_name);
            result->success = ACTION_FAIL;
            result->error_message = "leg motor enable safety check failed";
            goal_handle->abort(result);
            is_executing_leg_.store(false);
            return;
        }
        for (int i = 0; i < LEG_JOINTNUM; i++) {
            joint_cur_pos[i] = m_joints_cur_pos[i];
            joint_cur_vel[i] = m_joints_cur_speed[i];
        }

        try {

            RCLCPP_INFO(rclcpp::get_logger("lower_body_module"),"%s %s %f from [%f,%f,%f,%f] to [%f,%f,%f,%f]", m_module_name, __func__, motion_rate, m_joints_cur_pos[0], m_joints_cur_pos[1], m_joints_cur_pos[2], m_joints_cur_pos[3],
                  angles[0], angles[1], angles[2], angles[3]);

            std::vector<std::vector<double>> traj_pos_new;
            std::vector<std::vector<double>> traj_vel_new;
            std::vector<std::vector<double>> traj_acc_new;

            const auto planning_started_at = std::chrono::steady_clock::now();
            int all_point_ok = m_galgo->computeMoveJTrajectory<LEG_JOINTNUM>(ID_LEG, plan_dt, max_velocity, max_acc, max_jerk, joint_cur_pos, joint_cur_vel,
                                                               joint_cur_acc, angles, traj_pos_new, traj_vel_new, traj_acc_new);
            const double planning_time_seconds = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - planning_started_at).count();

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
            followTrajectoryHandle(
                goal_handle, action_started_at, 0.0, planning_time_seconds);

        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s format %s err:%s", m_module_name, __func__, e.what());
            result->success = ACTION_FAIL;
            result->error_message = "leg module parameter error:" + std::string(e.what());
            if (goal_handle->is_active()) {
                goal_handle->abort(result);
            }
            is_executing_leg_.store(false);  
            return;
        } 

    }

    void MiracleLowerBodyModule::execution_loop_move_pose(std::shared_ptr<GoalHandleMovePose> goal_handle)
    {
        const auto action_started_at = std::chrono::steady_clock::now();
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

        if (!m_bus_manager || !m_bus_manager->enableMotors(m_jointID)) {
            RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),
                         "%s refused to start: motor enable safety check failed", m_module_name);
            result->success = ACTION_FAIL;
            result->error_message = "leg motor enable safety check failed";
            goal_handle->abort(result);
            is_executing_leg_.store(false);
            return;
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

            double ik_time_seconds = 0.0;
            const auto planning_started_at = std::chrono::steady_clock::now();
            int all_point_ok = m_galgo->computeMovePTrajectory<LEG_JOINTNUM>(ID_LEG, plan_dt, max_velocity, max_acc, max_jerk, joint_cur_pos, joint_cur_vel,
                                                               joint_cur_acc, target_pose, traj_pos_new, traj_vel_new, traj_acc_new,
                                                               &ik_time_seconds);
            const double planning_and_ik_seconds = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - planning_started_at).count();
            const double planning_time_seconds = std::max(
                0.0, planning_and_ik_seconds - ik_time_seconds);

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
            followTrajectoryHandle(
                goal_handle, action_started_at, ik_time_seconds,
                planning_time_seconds);

            //测试
            Eigen::Matrix4d T;
            if (m_galgo->getForwardKinematics("base_link", "body_link", T)) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("lower_body_module"), "T_base_body=\n" << T);
            }

        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("lower_body_module"),"%s format %s err:%s", m_module_name, __func__, e.what());
            result->success = ACTION_FAIL;
            result->error_message = "leg module parameter error:" + std::string(e.what());
            if (goal_handle->is_active()) {
                goal_handle->abort(result);
            }
            is_executing_leg_.store(false);  
            return;
        } 

    }


} // namespace miracle
