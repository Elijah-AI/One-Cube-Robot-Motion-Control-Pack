#include "motion_control_pkg/execution/latest_progress.hpp"
#include "motion_control_pkg/execution/precision_parameters.hpp"
#include "motion_control_pkg/motion/precision_approach.hpp"
#include "motion_control_pkg/execution/motion_safety.hpp"
#include "motion_control_pkg/execution/ethercat_trajectory.hpp"
#include "motion_control_pkg/motion/scoped_collision_scene.hpp"
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
# Description: Miracle Head src file
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

#include "miracle_head_module.hpp"
#include "ethercat_bus_manager.hpp"
// #include "log.h"
#include <mutex>
#include <thread>


// using json = Json;
using namespace miracle;

#define HEAD_JOINT_LIMIT_ADJUST_RAD JOINT_LIMIT_ADJUST_RAD

/* accelebraion time max */
#define TRAJECTORY_BUFF_MAX_MS 100

#define HEAD_YAW_STEP1_RAD (11 * DEGREE_TO_RAD)
#define HEAD_YAW_STEP2_RAD (90 * DEGREE_TO_RAD)
#define HEAD_PITCH_STEP1_RAD (21 * DEGREE_TO_RAD)
#define HEAD_PITCH_STEP2_RAD (15 * DEGREE_TO_RAD)


namespace miracle {
    extern std::string robot_serial_no;
    // static std::mutex sys_mutex;
    // static std::mutex head_mutex;

    // void *followTrajThreadHandle(void *ptr) {

    //     MiracleHeadModule *module_ptr = (MiracleHeadModule *)ptr;
    //     module_ptr->followTrajectoryHandle();
    //     pthread_exit(0);
    //     return NULL;
    // }

    MiracleHeadModule::MiracleHeadModule(const char *module_name, const rclcpp::Node::SharedPtr node) : MiracleModule(module_name, node) {
        auto node_ptr = lockNode(__func__);
        if (!node_ptr) {
            return;
        }
        precision_policy_ = motion_control_pkg::execution::precisionParameters(*node_ptr, "head_precision");
        // function init
        // m_function_map["get_follow_trajectory_status"] = std::bind(&MiracleHeadModule::_getFollowTrajectoryState, this, std::placeholders::_1);
        // m_function_map["set_head_joint_angles"] = std::bind(&MiracleHeadModule::__setJointAngles, this, std::placeholders::_1);
        // m_function_map["get_head_joint_angles"] = std::bind(&MiracleHeadModule::__getJointAngles, this, std::placeholders::_1);
        // m_function_map["get_joints_angles_limit"] = std::bind(&MiracleHeadModule::__getJointAnglesLimit, this, std::placeholders::_1);
        // m_function_map["stop_follow_trajectory"] = std::bind(&MiracleHeadModule::__stopFollowTrajectory, this, std::placeholders::_1);

        // m_head = NULL;
        m_module_status.resize(HEAD_STATUS_MAX);
        m_module_enabled.resize(HEAD_JOINTNUM);
        m_joints_min_pos.resize(HEAD_JOINTNUM);
        m_joints_max_pos.resize(HEAD_JOINTNUM);        
        m_joints_cur_pos.resize(HEAD_JOINTNUM);
        m_joints_cur_speed.resize(HEAD_JOINTNUM);
        m_joints_cur_current.resize(HEAD_JOINTNUM);
        m_traj_buffer_pos.resize(HEAD_JOINTNUM);
        has_logged_error.resize(HEAD_JOINTNUM);
        for (int i = 0; i < HEAD_JOINTNUM; i++) {
            m_module_status[i] = 0;
            m_module_enabled[i] = 0;
            m_joints_cur_pos[i] = 0.0;
            m_joints_cur_speed[i] = 0.0;
            m_joints_cur_current[i] = 0.0;
            has_logged_error[i] = false;
        }

        // m_ipc = NULL;
        m_jointID = {HEAD_YAW_MOTOR_ID, HEAD_PITCH_MOTOR_ID};
    
        std::vector<long int> temp_joint_zero, temp_joint_dir;
        // node_ptr->declare_parameter("head_chain_start", std::string());
        // node_ptr->declare_parameter("head_chain_end", std::string());
        node_ptr->declare_parameter("head_joint_zero", std::vector<int>{0, 0});
        node_ptr->declare_parameter("head_joint_dir", std::vector<int>{1, 1});


        // node_ptr->get_parameter("head_chain_start", m_chain_start);
        // node_ptr->get_parameter("head_chain_end", m_chain_end);

        if(node_ptr->get_parameter("head_joint_zero", temp_joint_zero)) {
            RCLCPP_INFO(rclcpp::get_logger("head_module"),"Got head joint zero: %ld, %ld", temp_joint_zero[0], temp_joint_zero[1]);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("head_module"),"Failed to get parameter head joint zero");
        }

        if(node_ptr->get_parameter("head_joint_dir", temp_joint_dir)) {
            RCLCPP_INFO(rclcpp::get_logger("head_module"),"Got head joint dir: %ld, %ld", temp_joint_dir[0], temp_joint_dir[1]);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("head_module"),"Failed to get parameter head joint dir");
        }
        
        m_joint_zero.clear();
        m_joint_dir.clear();
        for (long int value : temp_joint_zero) {
            m_joint_zero.push_back(static_cast<int32_t>(value));
        }
        
        for (long int value : temp_joint_dir) {
            m_joint_dir.push_back(static_cast<int8_t>(value));
        }

        joint_state_pub_ = node_ptr->create_publisher<sensor_msgs::msg::JointState>("/head_joint_states", 10);
        joint_state_msg_.name = {"head_yaw_joint", "head_pitch_joint"};
        joint_state_msg_.position.resize(HEAD_JOINTNUM, 0.0);
        joint_state_msg_.velocity.resize(HEAD_JOINTNUM, 0.0);
        joint_state_msg_.effort.resize(HEAD_JOINTNUM, 0.0);

        // m_cloud_collision_run_stop = false;
        is_executing_head_.store(false);
    }

    MiracleHeadModule::~MiracleHeadModule() {
        deinitDevices();

        m_recv_thread_stop = true;
        if (m_ethercat_recv_handler && m_ethercat_recv_handler->joinable()) {
            m_ethercat_recv_handler->join();
        }

        if (execution_thread_move_joints_.joinable()) {
            execution_thread_move_joints_.join();
        }

        RCLCPP_INFO(rclcpp::get_logger("head_module"),"%s -%s Deconstruct successed", m_module_name, __func__);
    }

    /*
    ** init status msg basic info
    */
    // void MiracleHeadModule::initStatusMsg(robot_msg::msg::MiracleModuleStatus &status_msg) {
    //     status_msg.name.resize(HEAD_STATUS_MAX);
    //     status_msg.status.resize(HEAD_STATUS_MAX);
    //     for (int i = 0; i < HEAD_JOINTNUM; i++) {
    //         status_msg.name[i] = "joint" + std::to_string(i+1);
    //     }
    //     status_msg.name[HEAD_STATUS_WORK_STATE] = "work_state";
    //     // TODO:add other state handle
    // }

    /*
    ** for gmca
    **/
    void MiracleHeadModule::initGmcaModule(std::shared_ptr<MiracleMAlgo> &miracleAlgo) {
        m_galgo = miracleAlgo;
        m_galgo->getChainJointsLimt(3, m_joints_min_pos, m_joints_max_pos);
        RCLCPP_INFO(rclcpp::get_logger("Head module"),"%s m_joints_min_pos:[%f,%f] m_joints_max_pos:[%f,%f]", m_module_name, m_joints_min_pos[0], m_joints_min_pos[1],
                m_joints_max_pos[0], m_joints_max_pos[1]);
    }

    void MiracleHeadModule::initDevices(std::shared_ptr<EthercatBusManager> &bus_manager) {
        RCLCPP_INFO(rclcpp::get_logger("head_module"),"%s +%s", m_module_name, __func__);
        // int len = strlen(com_name);
        // memcpy(m_bus_name, com_name, len > sizeof(m_bus_name) ? sizeof(m_bus_name) : len);

        //     // std::unique_lock<std::mutex> lock(sys_mutex);

        m_inited = true;
        m_bus_manager = bus_manager;
        // EthercatBusManager::getOrCreate(ethercat_controller);

        recvDataThread();

        RCLCPP_INFO(rclcpp::get_logger("head_module"),"%s -%s", m_module_name, __func__);
    }


    void MiracleHeadModule::deinitDevices(void) {
        
        m_inited = false;


        RCLCPP_INFO(rclcpp::get_logger("head_module"),"%s -%s", m_module_name, __func__);
    }

    

    void MiracleHeadModule::recvDataThread() {
        if (m_ethercat_recv_handler && m_ethercat_recv_handler->joinable()) {
            m_ethercat_recv_handler->join();
        }
        m_ethercat_recv_handler = std::make_shared<std::thread>(std::bind(&MiracleHeadModule::headRevCallBack, this));

    }

    void MiracleHeadModule::publishJointState() {
        if (!joint_state_pub_) {
            return;
        }

        auto node_ptr = lockNode(__func__);
        if (!node_ptr) {
            return;
        }

        motion_control_pkg::execution::JointCompletionFeedback measured;
        std::string error;
        if (!motion_control_pkg::execution::readEthercatChainFeedback(m_bus_manager,m_jointID,measured,error))return;
        const auto age=std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now()-measured.acquired).count();
        joint_state_msg_.header.stamp = node_ptr->now()-rclcpp::Duration::from_nanoseconds(age);
        joint_state_msg_.position = measured.position;
        joint_state_msg_.velocity = measured.velocity;
        joint_state_msg_.effort.clear();
        joint_state_pub_->publish(joint_state_msg_);
    }

    void MiracleHeadModule::headRevCallBack() {
        
        int loop_cnt = 0;
        int loop_cnt2 = 0;
        while(!m_recv_thread_stop) {

            if (!getJointPosAndVel(m_joints_cur_pos, m_joints_cur_speed, m_module_status)) { usleep(TRAJ_CYCLE_US); continue; }
            // m_module_status[HEAD_STATUS_WORK_STATE] = 0;
    
            // RCLCPP_INFO(rclcpp::get_logger("head module"),"%s angle: %f:%f:%f:%f", m_module_name, m_joints_cur_pos[0], m_joints_cur_pos[1], m_joints_cur_pos[2],
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

    void MiracleHeadModule::updateStopState(bool stop){
        // if (!m_bus_manager || !m_bus_manager->batchSetSoftstop(stop)) {
        //     RCLCPP_WARN(rclcpp::get_logger("head_module"),"%s %s batchSetSoftstop failed", m_module_name, __func__);
        // }
    }
    
    bool MiracleHeadModule::getJointPosAndVel(std::vector<double> &actu_pos, std::vector<double> &actu_vel, std::vector<uint64_t> &error_code) {
        
        std::vector<DriverSDK::motorActualStruct> status_data;
        std::chrono::steady_clock::time_point acquired;
        std::uint64_t sequence=0;
        if (!m_bus_manager || !m_bus_manager->getTimedMotorActualSnapshot(status_data,acquired,sequence) ||
            sequence==0 || acquired>std::chrono::steady_clock::now() ||
            std::chrono::steady_clock::now()-acquired>=std::chrono::milliseconds(100)) {
            static rclcpp::Clock feedback_log_clock(RCL_STEADY_TIME);
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("head_module"),feedback_log_clock,2000,
                "%s: timestamped EtherCAT feedback unavailable or stale",m_module_name);
            return false;
        }

        for (auto id:m_jointID) {
            if (id>=status_data.size())return false;
            const auto& m=status_data[id];
            if (!std::isfinite(m.pos)||!std::isfinite(m.vel)||
                !motion_control_pkg::sdk::ethercatFeedbackValid(m))return false;
        }
        try {
            for(size_t i = 0; i < actu_pos.size(); i++) {

                // int32_t cur_pos = status_data.at(m_jointID[i]).actualPosition;
                // int32_t cur_vel = status_data.at(m_jointID[i]).actualVelocity;
                // double rad_pos = m_joint_dir[i] * (cur_pos-m_joint_zero[i]) * (2*M_PI) / m_cnt_per_cycle;
                // double rad_vel = m_joint_dir[i] * cur_vel * (2*M_PI) / m_cnt_per_cycle;
                
                actu_pos[i] = status_data[m_jointID[i]].pos;
                actu_vel[i] = status_data[m_jointID[i]].vel;
                // m_joints_cur_current[i] = status_data[m_jointID[i]].tor;

                error_code[i] = status_data[m_jointID[i]].errorCode;
                m_module_enabled[i] = status_data[m_jointID[i]].statusWord;
    
            }

            // for(int i = 0; i < m_module_enabled.size(); i++){
            //     if(m_module_enabled[i] != 4 && !has_logged_error[i]){
            //         RCLCPP_ERROR(rclcpp::get_logger("head_module")," %s: motor %d is breakdown!", m_module_name, i);
            //         has_logged_error[i] = true; // 标志为已记录过，下次不用再调用了，除非恢复过正常
            //         m_module_status.back() |= (1ULL << 0);  // 设置第0位为1
            //         m_module_status.back() |= (1ULL << i+5); 
            //         // motor_not_enabled = true; // 用于判断是否执行动作，true则不执行
            //     }
            //     else if (has_logged_error[i]) {
            //         if(m_module_enabled[i]==4){    
            //             has_logged_error[i] = false;
            //             m_module_status.back() &= ~(1ULL << i+5);
            //         }
            //         if(std::all_of(m_module_enabled.begin(), m_module_enabled.end(), [](uint64_t val) { return val == 4; })){
            //             m_module_status.back() &= ~(1ULL << 0);
            //             // motor_not_enabled = false;
            //         }
            //     }
            // }
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s %s get motor data error %s", m_module_name, __func__, e.what());
            return false;
        }

        
        return true;
    }


    /**
     * @brief 下发多个关节位置
     * 
     * @param pos 
     * @return true 
     * @return false 
     */
    bool MiracleHeadModule::setMulJointPos(const std::vector<double> pos, std::vector<uint16_t> ethcat_id) {

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
            RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s %s updateMotorTargets failed", m_module_name, __func__);
            return false;
        }
        
        return true;
    }

    bool MiracleHeadModule::checkJointsAngleOk(std::vector<double> angles) {
        if((angles[0] >= m_joints_min_pos[0]) && (angles[0] <= m_joints_max_pos[0]) && (angles[1] >= m_joints_min_pos[1]) && (angles[1] <= m_joints_max_pos[1])) {
            return true;
        } else {
            return false;
        }

    }

    bool MiracleHeadModule::is_idle(int module_id) const  //根据模块标号返回单个模块是否空闲状态
    {
        if(module_id == ID_HEAD)
        {
            return !is_executing_head_.load();
        }
        else
        {
            return false;
        }
    }

    void MiracleHeadModule::start_execution_move_joints(std::shared_ptr<GoalHandleMoveJoints> goal_handle)
    {
        std::string error;
        if (!reserve_body_motion_resource(error)) {
            auto result = std::make_shared<MoveJoints::Result>();
            result->success = false;
            result->error_message = error;
            goal_handle->abort(result);
            return;
        }
        try {
        // 如果之前有线程，等待
        if (execution_thread_move_joints_.joinable()) {
            execution_thread_move_joints_.join();
        }
        
        // const auto goal = goal_handle->get_goal();
        // 标记为正在执行,
    

        // 启动线程（不再使用 detach）
        execution_thread_move_joints_ = std::thread(&MiracleHeadModule::execution_loop_move_joints, this, goal_handle);
        } catch (const std::exception& exception) {
            release_body_motion_resource();
            auto result = std::make_shared<MoveJoints::Result>();
            result->success = false;
            result->error_message = exception.what();
            goal_handle->abort(result);
        }
    }

    bool MiracleHeadModule::reserve_body_motion_resource(std::string& error)
    {
        bool expected = false;
        if (!is_executing_head_.compare_exchange_strong(expected, true)) {
            error = "Head is occupied by another motion";
            return false;
        }
        error.clear();
        return true;
    }

    void MiracleHeadModule::execution_loop_move_joints(std::shared_ptr<GoalHandleMoveJoints> goal_handle)
    {
        const auto action_started_at = std::chrono::steady_clock::now();
        const auto goal = goal_handle->get_goal();
        const auto action_precision = motion_control_pkg::execution::actionPrecisionPolicy(
            precision_policy_, goal->precision_arrival);
        auto feedback = std::make_shared<MoveJoints::Feedback>();
        auto result = std::make_shared<MoveJoints::Result>();

        // if (checkPowerState(result)) {
        //     return result;
        // }
        float motion_rate = goal->speed;
        std::vector<double> angles = goal->position;            //rad

        if (motion_rate > 1.0 || motion_rate < 0.0) {
            RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s %s motion_rate set error, need to set between (0.0, 1.0)", m_module_name, __func__);
            result->success = ACTION_FAIL;
            result->error_message = "speed out of range";
            goal_handle->abort(result);
            is_executing_head_.store(false);  
            return ;
        }
        if(angles.size() != HEAD_JOINTNUM) {
            RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s %s param error: joint_angles size is not match", m_module_name, __func__);
            result->success = ACTION_FAIL;
            result->error_message = "joint_angles size error";
            goal_handle->abort(result);
            is_executing_head_.store(false);  
            return ;
        }

        if (!checkJointsAngleOk(angles)) {
            RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s %s param error: joint_angles out of range", m_module_name, __func__);
            result->success = ACTION_FAIL;
            result->error_message = "joint angles out of range";
            goal_handle->abort(result);
            is_executing_head_.store(false);  
            return;
        }

        double plan_dt = 1.0 / TRAJ_FREQ;
        double max_velocity = 1.5 * motion_rate;
        double max_acc = 5 * motion_rate;
        double max_jerk = 10 * motion_rate;

        // current info
        std::vector<double> joint_cur_pos(HEAD_JOINTNUM);
        std::vector<double> joint_cur_vel(HEAD_JOINTNUM);
        std::vector<double> joint_cur_acc(HEAD_JOINTNUM);

        for (int i = 0; i < HEAD_JOINTNUM; i++) {
            joint_cur_pos[i] = m_joints_cur_pos[i];
            joint_cur_vel[i] = m_joints_cur_speed[i];
            joint_cur_acc[i] = 0.0;                    //...关节没有
        }
        m_traj_point_index = 0;
        RCLCPP_WARN(rclcpp::get_logger("Head_module"),"%s %s %d cur_vel:[%f,%f]", m_module_name, __func__, m_traj_point_index, joint_cur_vel[0], joint_cur_vel[1]);
        //     max_acc *= 1.5; 未停止直接重新规划时用
        //     max_jerk *= 1.5;
        // } else {
        if (!m_bus_manager || !m_bus_manager->enableMotors(m_jointID)) {
            RCLCPP_ERROR(rclcpp::get_logger("head_module"),
                         "%s refused to start: motor enable safety check failed", m_module_name);
            result->success = ACTION_FAIL;
            result->error_message = "head motor enable safety check failed";
            goal_handle->abort(result);
            is_executing_head_.store(false);
            return;
        }
        for (int i = 0; i < HEAD_JOINTNUM; i++) {
            joint_cur_pos[i] = m_joints_cur_pos[i];
            joint_cur_vel[i] = m_joints_cur_speed[i];
        }

        try {

            RCLCPP_INFO(rclcpp::get_logger("head_module"),"%s %s %f from [%f,%f] to [%f,%f]", m_module_name, __func__, motion_rate, m_joints_cur_pos[0], m_joints_cur_pos[1], angles[0],
                  angles[1]);

            std::vector<std::vector<double>> traj_pos_new;
            std::vector<std::vector<double>> traj_vel_new;
            std::vector<std::vector<double>> traj_acc_new;

            motion_control_pkg::execution::JointCompletionFeedback measured;
            std::string measured_error;
            if (!motion_control_pkg::execution::readEthercatChainFeedback(m_bus_manager, m_jointID, measured, measured_error))
                throw std::runtime_error(measured_error);
            if (std::any_of(measured.velocity.begin(), measured.velocity.end(), [](double v) {
                    return std::abs(v) > motion_control_pkg::execution::kGoalVelocityToleranceRadSec; }))
                throw std::runtime_error("Joint chain must be stationary before MoveJ");
            joint_cur_pos = measured.position;
            joint_cur_vel.assign(joint_cur_pos.size(), 0.0);
            joint_cur_acc.assign(joint_cur_pos.size(), 0.0);
            motion_control_pkg::motion::PrecisionApproach approach;
            std::string precision_error;
            const bool precision_ok = motion_control_pkg::motion::planPrecisionApproach(
                joint_cur_pos, angles, action_precision,
                [&](const auto& from, const auto& to, bool slow, auto& p, auto& v, auto& a, std::string& error) {
                    const double scale = slow ? action_precision.speed_scale : 1.0;
                    const int code = m_galgo->computeMoveJTrajectory<HEAD_JOINTNUM>(ID_HEAD, plan_dt,
                        slow ? std::min(max_velocity*scale,action_precision.velocity_cap) : max_velocity,
                        slow ? std::min(max_acc*scale,action_precision.acceleration_cap) : max_acc,
                        slow ? std::min(max_jerk*scale,action_precision.jerk_cap) : max_jerk,
                        from, std::vector<double>(from.size(),0.0), std::vector<double>(from.size(),0.0),
                        to, p, v, a, motion_rate*scale);
                    if (code != 0) error = "precision Ruckig planning failed: " + std::to_string(code);
                    return code == 0;
                }, approach, precision_error);
            if (!precision_ok) throw std::runtime_error(precision_error);
            precision_gate_ = approach.gate;
            traj_pos_new = std::move(approach.positions);
            traj_vel_new = std::move(approach.velocities);
            traj_acc_new = std::move(approach.accelerations);
            std::vector<double> urdf_velocity;
            if (!m_galgo->getChainVelocityLimits(ID_HEAD, urdf_velocity))
                throw std::runtime_error("cannot validate precision trajectory velocity limits");
            for (auto& v : urdf_velocity) v=std::min(v*motion_rate,max_velocity);
            if (!motion_control_pkg::motion::validateCommandDerivatives(traj_pos_new,plan_dt,urdf_velocity,
                    std::vector<double>(HEAD_JOINTNUM,max_acc),std::vector<double>(HEAD_JOINTNUM,max_jerk),precision_error))
                throw std::runtime_error(precision_error);
            RCLCPP_INFO(rclcpp::get_logger("head_precision"),
                "MoveJ precision_arrival=%s: goal_tolerance=%.6f rad, slow_suffix=%d, samples=%zu, planned_motion=%.3f s",
                goal->precision_arrival ? "ON" : "OFF", action_precision.completion.position_tolerance,
                precision_gate_ != motion_control_pkg::motion::kNoApproachGate, traj_pos_new.size(),
                (traj_pos_new.size()-1)*plan_dt);




            motion_control_pkg::motion::checkChainCollision(*m_galgo, goal->enable_collision_check, ID_HEAD, traj_pos_new);
            m_traj_pos = traj_pos_new;
            m_traj_vel = traj_vel_new;
            m_traj_acc = traj_acc_new;
            followTrajectoryHandle(goal_handle, action_started_at);

        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s format %s err:%s", m_module_name, __func__, e.what());
            result->success = ACTION_FAIL;
            result->error_message = "head module parameter error:" + std::string(e.what());
            if (goal_handle->is_active()) {
                goal_handle->abort(result);
            }
            is_executing_head_.store(false);  
            return;
        } 

    }

    void MiracleHeadModule::followTrajectoryHandle(std::shared_ptr<GoalHandleMoveJoints> goal_handle,
        const std::chrono::steady_clock::time_point& action_started_at) {
        using namespace motion_control_pkg::execution;
        auto result = std::make_shared<MoveJoints::Result>();
        MotionSafetyScope safety_scope(8U);
        auto feedback = std::make_shared<MoveJoints::Feedback>();
        const auto goal = goal_handle->get_goal();
        const auto action_precision = motion_control_pkg::execution::actionPrecisionPolicy(
            precision_policy_, goal->precision_arrival);
        const auto started = JointCompletionGuard::Clock::now();
        const auto deadline = action_started_at + std::chrono::milliseconds(goal->timeout_ms);
        const auto check = [&](std::string& error) {
            if (goal_handle->is_canceling()) { error = "head trajectory canceled"; return false; }
            if (!rclcpp::ok()) { error = "ROS shutdown during head trajectory"; return false; }
            if (goal->timeout_ms <= 0 || JointCompletionGuard::Clock::now() >= deadline) {
                error = "head trajectory timeout"; return false;
            }
            return true;
        };
        const auto publish = [&](float progress) {
            feedback->progress_rate = progress;
            feedback->dual_task_status = progress < 95 ? MoveJoints::Feedback::STATUS_EXECUTING :
                MoveJoints::Feedback::STATUS_WAITING_FOR_COMPLETION;
            feedback->task_status = progress < 95 ? "executing" : "waiting for measured settling";
            feedback->current_step_info = feedback->task_status;
            if (auto node = lockNode(__func__)) feedback->header.stamp = node->now();
            goal_handle->publish_feedback(feedback);
        };
        {
            LatestProgressPublisher progress(publish);
            const auto progress_check = [&](std::string& error) {
                if (!check(error)) return false;
                if (progress.failed()) {error=progress.error();return false;}
                return true;
            };
            try {
                result->success = executeEthercatTrajectory(m_traj_pos, m_bus_manager, m_jointID,
                    [&](const std::vector<double>& q) { return setMulJointPos(q, m_jointID); },
                    progress_check, [&](float value){progress.update(value);}, deadline, result->final_distance_error, result->error_message, action_precision, precision_gate_);
            } catch (const std::exception& e) {
                result->error_message=e.what();
                safety_scope.finish(false,result->error_message);
                progress.stop();throw std::runtime_error(result->error_message);
            } catch (...) {
                result->error_message="unknown precision trajectory execution exception";
                safety_scope.finish(false,result->error_message);
                progress.stop();throw std::runtime_error(result->error_message);
            }
            // Fence/confirm a failed motion before waiting for observer I/O.
            if (!result->success) safety_scope.finish(false,result->error_message);
            progress.stop();
            if (progress.failed() && result->success) {
                result->success=false;result->error_message=progress.error();
            }
        }
        result->execution_time = std::chrono::duration<double>(JointCompletionGuard::Clock::now() - action_started_at).count();
        result->timing_breakdown_valid = true;
        result->ik_time = 0.0;
        result->planning_time = std::chrono::duration<double>(started - action_started_at).count();
        result->motion_time = std::chrono::duration<double>(JointCompletionGuard::Clock::now() - started).count();
        result->success = safety_scope.finish(result->success && !goal_handle->is_canceling(), result->error_message);
        if (goal_handle->is_canceling()) goal_handle->canceled(result);
        else if (result->success) { publish(100.0F); goal_handle->succeed(result); }
        else goal_handle->abort(result);
        is_executing_head_.store(false);
    }

    // bool MiracleHeadModule::isRunMode(CallbackResult &r) {
    //     if (!m_bus_manager) { // || isMotorError()) !m_inited || 
    //         r.status_code = 20516;
    //         r.message = "头关节模块未初始化";
    //         return false;
    //     }
    //     return true;
    // }

    bool MiracleHeadModule::isMotorError(){
        for(int i = 0; i < HEAD_JOINTNUM; i++) {
            if(m_module_status[i]) {
                return true;
            }
        }
        return false;

    }

    /*
    ** if  is moving, then this will return true
    **/
    bool MiracleHeadModule::isMoving(void) {
        return m_traj_run;
    }

    // CallbackResult MiracleHeadModule::__handleMoveJTrajectory(const std::vector<double> joint_target_pos, double motion_rate) {

    //     CallbackResult result;

    //     // schedule parameter
    //     double plan_dt = 1.0 / TRAJ_FREQ;
    //     double max_velocity = 1.5 * motion_rate;
    //     double max_acc = 5 * motion_rate;
    //     double max_jerk = 10 * motion_rate;

    //     // current info
    //     std::vector<double> joint_cur_pos(HEAD_JOINTNUM);
    //     std::vector<double> joint_cur_vel(HEAD_JOINTNUM);
    //     std::vector<double> joint_cur_acc(HEAD_JOINTNUM);

    //     // handle last traj vel and acc
    //     pthread_mutex_lock(&m_traj_mutex);
    //     if (m_traj_run) {
    //         // this is used to stop follow run
    //         // m_traj_cur_stop = true;

    //         // if (m_traj_vel.size() > m_traj_point_index) {
    //         //     for (int i = 0; i < HEAD_JOINTNUM; i++) {
    //         //         joint_cur_pos[i] = m_traj_buffer_pos[i];
    //         //         joint_cur_vel[i] = m_traj_vel[m_traj_point_index][i];
    //         //         joint_cur_acc[i] = m_traj_acc[m_traj_point_index][i];
    //         //     }
    //         //     RCLCPP_WARN(rclcpp::get_logger("head_module"),"%s %s %d cur_vel:[%f,%f] cur_acc:[%f,%f]", m_module_name, __func__, m_traj_point_index, joint_cur_vel[0], joint_cur_vel[1],
    //         //           joint_cur_acc[0], joint_cur_acc[1]);
    //         // } else {
    //         //     // last traj must be zero vel an acc
    //         //     for (int i = 0; i < HEAD_JOINTNUM; i++) {
    //         //         joint_cur_pos[i] = m_traj_buffer_pos[i];
    //         //         joint_cur_vel[i] = 0.0;
    //         //         joint_cur_acc[i] = 0.0;
    //         //     }
    //         // }

    //         for (int i = 0; i < HEAD_JOINTNUM; i++) {
    //             joint_cur_pos[i] = m_joints_cur_pos[i];
    //             joint_cur_vel[i] = m_joints_cur_speed[i];
    //             joint_cur_acc[i] = 0.0;                    //钛虎关节没有
    //         }

    //         RCLCPP_WARN(rclcpp::get_logger("Head_module"),"%s %s %d cur_vel:[%f,%f]", m_module_name, __func__, m_traj_point_index, joint_cur_vel[0],
    //                   joint_cur_vel[1]);
    //         max_acc *= 1.5;
    //         max_jerk *= 1.5;
    //     } else {
    //         bool pos_same = true;
    //         for (int i = 0; i < HEAD_JOINTNUM; i++) {
    //             if (abs(joint_target_pos[i] - m_joints_cur_pos[i]) > JOINT_THRESHOLD) {
    //                 pos_same = false;
    //                 break;
    //             }
    //         }
    //         if (pos_same) {
    //             pthread_mutex_unlock(&m_traj_mutex);
    //             RCLCPP_INFO(rclcpp::get_logger("head_module"),"%s %s %f from [%f,%f] to [%f,%f] almost same", m_module_name, __func__, motion_rate, m_joints_cur_pos[0], m_joints_cur_pos[1],
    //                   joint_target_pos[0], joint_target_pos[1]);
    //             result.status_code = 0;
    //             result.message = "执行成功";
    //             return result;
    //         }
    //         for (int i = 0; i < HEAD_JOINTNUM; i++) {
    //             joint_cur_pos[i] = m_joints_cur_pos[i];
    //             joint_cur_vel[i] = 0.0;
    //             joint_cur_acc[i] = 0.0;
    //         }
    //     }
    //     pthread_mutex_unlock(&m_traj_mutex);

    //     try {

    //         RCLCPP_INFO(rclcpp::get_logger("head_module"),"%s %s %f from [%f,%f] to [%f,%f]", m_module_name, __func__, motion_rate, m_joints_cur_pos[0], m_joints_cur_pos[1], joint_target_pos[0],
    //               joint_target_pos[1]);

    //         std::vector<std::vector<double>> traj_pos_new;
    //         std::vector<std::vector<double>> traj_vel_new;
    //         std::vector<std::vector<double>> traj_acc_new;

    //         int all_point_ok = m_galgo->computeMoveJTrajectory<HEAD_JOINTNUM>(plan_dt, max_velocity, max_acc, max_jerk, joint_cur_pos, joint_cur_vel,
    //                                                            joint_cur_acc, joint_target_pos, traj_pos_new, traj_vel_new, traj_acc_new,m_module_name);

    //         if (0 != all_point_ok) {
    //             RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s %s Error %d for movej", m_module_name, __func__, all_point_ok);
    //             result.status_code = 20512;
    //             result.message = "头关节设置失败" + std::to_string(all_point_ok);
    //             return result;
    //         }

    //         pthread_mutex_lock(&m_traj_mutex);
    //         m_traj_pos = traj_pos_new;
    //         m_traj_vel = traj_vel_new;
    //         m_traj_acc = traj_acc_new;
    //         if (!m_traj_run) {

    //             m_traj_run = true;
    //             // excute thread
    //             pthread_t follow_tid;
    //             pthread_attr_t attr;
    //             struct sched_param param;
    //             param.sched_priority = 40;

    //             pthread_attr_init(&attr);
    //             pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    //             pthread_attr_setschedpolicy(&attr, SCHED_RR);
    //             pthread_attr_setschedparam(&attr, &param);
    //             if (0 != pthread_create(&follow_tid, &attr, followTrajThreadHandle, this)) {
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
    //         RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s format %s err:%s", m_module_name, __func__, e.what());
    //         result.status_code = 20515;
    //         result.message = "头关节参数解析错误" + std::string(e.what());
    //         return result;
    //     }
    // }


    // CallbackResult MiracleHeadModule::__setJointAngles(const Json::Value &params) {
    //     RCLCPP_INFO(rclcpp::get_logger("head_module"),"%s CALLING %s", m_module_name, __func__);
    //     CallbackResult result;
    //     if (!isRunMode(result)) {
    //         RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s isRunMode fail %s", m_module_name, result.message.c_str());
    //         return result;
    //     }

    //     if (checkPowerState(result)) {
    //         return result;
    //     }
    //     float motion_rate;
    //     std::vector<double> angles;
    //     if (!params.isMember("parameters") || !params["parameters"].isObject()) {
    //         RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s %s param error: missing parameters", m_module_name, __func__);
    //         result.status_code = 20515;
    //         result.message = "头关节参数解析错误";
    //         return result;
    //     }
    //     const Json::Value &parameters = params["parameters"];
    //     if (!parameters.isMember("speed") || !parameters["speed"].isNumeric()) {
    //         RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s %s param error: missing speed", m_module_name, __func__);
    //         result.status_code = 20515;
    //         result.message = "头关节参数解析错误";
    //         return result;
    //     }
    //     motion_rate = parameters["speed"].asFloat();
    //     if (motion_rate > 1.0 || motion_rate < 0.0) {
    //         RCLCPP_ERROR(rclcpp::get_logger("arm_module"),"%s %s motion_rate set error, need to set between (0.0, 1.0)", m_module_name, __func__);
    //         result.status_code = 20513;
    //         result.message = "头关节速度超出合法范围";
    //         return result;
    //     }
    //     if (!parameters.isMember("joint_angles") || !parameters["joint_angles"].isArray()) {
    //         RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s %s param error: missing joint_angles", m_module_name, __func__);
    //         result.status_code = 20515;
    //         result.message = "头关节参数解析错误";
    //         return result;
    //     }
    //     const Json::Value &joint_angles = parameters["joint_angles"];
    //     if (static_cast<int>(joint_angles.size()) != HEAD_JOINTNUM) {
    //         RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s %s param error: joint_angles size mismatch", m_module_name, __func__);
    //         result.status_code = 20515;
    //         result.message = "头关节参数解析错误";
    //         return result;
    //     }
    //     angles.reserve(joint_angles.size());
    //     for (Json::ArrayIndex i = 0; i < joint_angles.size(); ++i) {
    //         angles.push_back(joint_angles[i].asDouble());
    //     }

    //     // if (m_head) {
    //     if ((abs(m_joints_cur_pos[0] - angles[0]) > 0.001) || (abs(m_joints_cur_pos[1] - angles[1]) > 0.001)) {

    //         if (!checkJointsAngleOk(angles)) {
    //             result.status_code = 20514;
    //             result.message = "头关节角度超出物理限制";
    //             RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s angles param error", m_module_name);
    //         } else {

    //             return __handleMoveJTrajectory(angles, motion_rate);
    //         }
    //     } else {
    //         result.status_code = 0;
    //         result.message = "执行成功";
    //     }
    //     // } else {
    //     //     RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s %s m_head NULL", m_module_name, __func__);
    //     //     result.status_code = 1;
    //     //     result.message = "Error : head instance NULL";
    //     // }
    //     return result;
    // }

    // CallbackResult MiracleHeadModule::__getJointAngles(const Json::Value &params) {
    //     RCLCPP_INFO(rclcpp::get_logger("head_module"),"%s CALLING %s", m_module_name, __func__);
    //     CallbackResult result;

    //     if (m_inited && m_bus_manager) {
    //         try {
    //             Json::Value data;
    //             Json::Value angles(Json::arrayValue);
    //             for (int i = 0; i < HEAD_JOINTNUM; i++) {
                
    //                 angles.append(m_joints_cur_pos[i]);
    //             }
    //             data["joint_angles"] = angles;
    //             result.data = data;
    //         } catch (const std::exception &e) {
    //             RCLCPP_ERROR(rclcpp::get_logger("head_module"),"Error generating head raw response: %s", e.what());
    //             result.status_code = 1;
    //             result.message = "Error: " + std::string(e.what());
    //             return result;
    //         }
            
    //         if (isRunMode(result)) {
    //             result.status_code = 0;
    //             result.message = "head raw retrieved successfully.";
    //         }
    //         return result;
    //     } else {
    //         RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s %s m_bus_manager NULL", m_module_name, __func__);
    //         result.status_code = 1;
    //         result.message = "Error : head module not initialized";
    //         return result;
    //     }
        
    //     return result;
    // }

    // CallbackResult MiracleHeadModule::__getJointAnglesLimit(const Json::Value &params) {
    //     RCLCPP_INFO(rclcpp::get_logger("head_module"),"%s CALLING %s", m_module_name, __func__);
    //     CallbackResult result;
    //     result.status_code = 0;

    //     try {
    //         Json::Value data;
    //         Json::Value angles_limit(Json::arrayValue);
    //         data["limit_step_cnt"] = 2;
            
    //         Json::Value angles_limit1(Json::arrayValue);
    //         angles_limit1.append(m_joints_min_pos[0]);
    //         angles_limit1.append(m_joints_min_pos[1]);
            
    //         Json::Value angles_limit2(Json::arrayValue);
    //         angles_limit2.append(m_joints_max_pos[0]);
    //         angles_limit2.append(m_joints_max_pos[1]);
            
    //         angles_limit.append(angles_limit1);
    //         angles_limit.append(angles_limit2);
            
    //         data["joint_angles_limit"] = angles_limit;
    //         result.data = data;
    //     } catch (const std::exception &e) {
    //         RCLCPP_ERROR(rclcpp::get_logger("head_module"),"Error generating head raw response: %s", e.what());
    //         result.status_code = 1;
    //         result.message = "Error: " + std::string(e.what());
    //         return result;
    //     }
    //     return result;
    // }

    // CallbackResult MiracleHeadModule::_getFollowTrajectoryState(const Json::Value &params) {
    //     RCLCPP_DEBUG(rclcpp::get_logger("head_module"),"%s CALLING %s", m_module_name, __func__);
    //     CallbackResult result;
    //     try {
    //         Json::Value data;
    //         data["follow_trajectory_status"] = isMoving();
    //         data["stop_reason"] = 0;
    //         result.data = data;
    //     } catch (const std::exception &e) {
    //         RCLCPP_ERROR(rclcpp::get_logger("head_module"),"Error generating state raw response: %s", e.what());
    //         result.status_code = 1;
    //         result.message = "Error: " + std::string(e.what());
    //         return result;
    //     }

    //     if (isRunMode(result)) {
    //         result.status_code = 0;
    //         result.message = "state retrieved successfully.";
    //     }
    //     return result;
    // }

    // CallbackResult MiracleHeadModule::__stopFollowTrajectory(const Json::Value &params) {
    //     RCLCPP_INFO(rclcpp::get_logger("head_module"),"%s CALLING %s", m_module_name, __func__);
    //     CallbackResult result;
    //     result.status_code = 0;
    //     if (!isRunMode(result)) {
    //         RCLCPP_ERROR(rclcpp::get_logger("head_module"),"%s check_module_running fail %s", __func__, result.message.c_str());
    //         return result;
    //     }
        
    //     result.message = "Head stop follow trajectory successfully.";

    //     m_cloud_collision_run_stop = true;
    //     m_set_pose_stop = true;

    //     pthread_mutex_lock(&m_traj_mutex);
    //     if (m_traj_run) {      //原为!m_traj_run
    //         m_traj_run_stop = true;
    //     }
    //     pthread_mutex_unlock(&m_traj_mutex);
    //     int retry_times = 0;
    //     while (m_traj_run) { //原为!m_traj_run
    //         retry_times++;
    //         if (retry_times > 30) {
    //             result.status_code = 1;
    //             result.message = "Head stop follow trajectory overtime.";
    //             break;
    //         }
    //         usleep(20000);  //20ms
    //     }
    //     return result;
    // }
  


} // namespace miracle
