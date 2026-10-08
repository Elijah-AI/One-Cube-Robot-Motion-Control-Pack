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
# ERROR, PLEASE NOTIFY MIRACLE, INC. IMMEDIATELY AND DELETE IT FROM YOUR SYSTEM.
#
#####################################################################################
#
# Description: Miracle Head head file
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

#ifndef MIRACLE_HEAD_MODULE_HPP
#define MIRACLE_HEAD_MODULE_HPP


#include <iostream>
#include <vector>

#include "sensor_msgs/msg/joint_state.hpp"
#include "jsoncpp/json/json.h"
#include "miracle_module.hpp"
#include "miracle_m_algo.hpp"
#include "main_node.hpp"
#include "rclcpp/rclcpp.hpp"


// using json = Json;

namespace miracle {

    typedef enum {
        HEAD_STATUS_WORK_STATE = HEAD_JOINTNUM,
        HEAD_STATUS_MAX,
    } HEAD_STATUS_ENUM;

    /**
     * @brief 头部子模块
     * 
     */
    class MiracleHeadModule : public MiracleModule {
    public:
        MiracleHeadModule(const char *module_name, const rclcpp::Node::SharedPtr node);
        
        ~MiracleHeadModule();
        /*
        ** init status msg basic info
        */

        /**
         * @brief 实现接口，初始化status_msg中的基本信息
         * 
         * @param status_msg 
         */
        // void initStatusMsg(robot_msg::msg::MiracleModuleStatus &status_msg);

        /*
        ** init all extra device and system for ota
        */
        void initDevices(std::shared_ptr<EthercatBusManager> &bus_manager);

        /*
        ** deinit all extra device and system
        ** this is not thread saft, must call in same thread with cmd handle
        */

        /**
         * @brief
         * 
         */
        void deinitDevices(void);

        void recvDataThread(void);
        /*
        ** head function callback
        */
        void headRevCallBack();

        /*
        ** if leg is moving, then this will return true
        **/
        bool isMoving(void);

        /*
        ** use to follow handle
        */
        void followTrajectoryHandle(std::shared_ptr<GoalHandleMoveJoints> goal_handle);

        /*
        ** for gmca
        **/
        void initGmcaModule(std::shared_ptr<MiracleMAlgo> &miracleAlgo);

        void updateStopState(bool stop);
        
        bool getJointPosAndVel(std::vector<double> &actu_pos, std::vector<double> &actu_vel, std::vector<uint64_t> &error_code);
    
        /**
         * @brief 下发多个关节位置
         * 
         * @param pos 
         * @return true 
         * @return false 
         */
        bool setMulJointPos(const std::vector<double> pos, std::vector<uint16_t> ethcat_id);


        // 暴露给上层的状态检查
        bool is_idle(int module_id) const;
        bool reserve_body_motion_resource(std::string& error);
        void release_body_motion_resource() noexcept { is_executing_head_.store(false); }
    
        // 暴露给上层的任务入口, 一个action対应一个接口
        void start_execution_move_joints(std::shared_ptr<GoalHandleMoveJoints> goal_handle);

    private:
        //一个action対应一个接口
        void execution_loop_move_joints(std::shared_ptr<GoalHandleMoveJoints> goal_handle);
        void publishJointState();
        
        bool checkJointsAngleOk(std::vector<double> angles);

        /*
        ** head ready for work
        */
        // bool isRunMode(CallbackResult &r);
        bool isMotorError();

        // CallbackResult __setJointAngles(const Json::Value &params);
        // CallbackResult __getJointAngles(const Json::Value &params);
        // CallbackResult __getJointAnglesLimit(const Json::Value &params);
        // CallbackResult _getFollowTrajectoryState(const Json::Value &params);
        // CallbackResult __stopFollowTrajectory(const Json::Value &params);

        // CallbackResult __handleMoveJTrajectory(const std::vector<double> joint_target_pos, double motion_rate);

        // head info
        // Head *m_head;
        int32_t m_cnt_per_cycle = 3342336;     //钛虎模组编码器负载端编码器一圈脉冲数 16位，65536*51

        std::vector<int32_t> m_joint_zero;    //关节零点
        std::vector<int8_t>  m_joint_dir;     //关节方向
        std::vector<uint16_t> m_jointID;

        std::thread execution_thread_move_joints_;
        std::atomic<bool> is_executing_head_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        sensor_msgs::msg::JointState joint_state_msg_;

    };
} // namespace miracle

#endif // MIRACLE_HEAD_MODULE_HPP
