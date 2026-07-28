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
# Description: Miracle LowerBodyModule head file
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
*/

#ifndef MIRACLE_LOWER_BODY_MODULE_HPP
#define MIRACLE_LOWER_BODY_MODULE_HPP


#include <cmath>
#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include "sensor_msgs/msg/joint_state.hpp"
#include "miracle_module.hpp"
#include "jsoncpp/json/json.h"
#include "main_node.hpp"
// #include "miracle_chassis_module.hpp"



// using json = Json;

namespace miracle {

    typedef enum {
        LOWER_BODY_STATUS_WORK_STATE = LEG_JOINTNUM,
        LOWER_BODY_STATUS_MAX,
    } LOWER_BODY_STATUS_ENUM;

    // typedef void (*miracleBmsCB)(ChassisPower::BmsType bms_data);

    /**
     * @brief 下肢子模块
     * 
     */
    class MiracleLowerBodyModule : public MiracleModule {
    public:
        MiracleLowerBodyModule(const char *module_name, const rclcpp::Node::SharedPtr node);

        ~MiracleLowerBodyModule();
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

        /**
         * @brief 初始化外部设备和系统
         * 
         * @param com_name 
         * @param power_manager 
         */
        void initDevices(std::shared_ptr<EthercatBusManager> &bus_manager);

        /*
        ** deinit all extra device and system
        ** this is not thread saft, must call in same thread with cmd handle
        */
       
        /**
         * @brief 反初始化外部设备和系统，非线程安全，
         * 
         */
        void deinitDevices(void);

        void recvDataThread();

        /**
         * @brief 下肢数据处理回调函数， 形参可根据实际调整
         * 
         * @param c 
         * @param f 
         * @param buf 
         */
        void lowerBodyRevCallBack();

        /*
        ** set bms data callback for upper
        */


        /*
        ** if leg is moving, then this will return true
        **/

        /**
         * @brief 判断下肢是否在运动
         * 
         * @return true 
         * @return false 
         */
        bool isMoving(void);

        /*
        ** use to lowerBody follow function
        */


        template <typename ActionT>
        void followTrajectoryHandle(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle);
        /*
        ** for gmca
        **/

        /**
         * @brief 初始化底层运动学动力学类
         * 
         * @param miracleAlgo 
         */
        void initGmcaModule(std::shared_ptr<MiracleMAlgo> &miracleAlgo);
        


        /**
         * @brief 获取当前关节位置
         * 
         * @return std::vector<float> 
         */
        bool getJointPosAndVel(std::vector<double> &actu_pos, std::vector<double> &actu_vel, std::vector<double> &actu_curr, std::vector<uint64_t> &error_code);
    
        
        bool setMulJointPos(const std::vector<double> pos, std::vector<uint16_t> ethcat_id);

        // CallbackResult __setJointAngles(const Json::Value &params);          ///< 设置关节角度，置于function_map表中 
        
        // CallbackResult __setFollowTrajectory(const Json::Value &params);     ///< 跟随关节轨迹点，置于function_map表中

        bool is_idle(int module_id) const;
    
        // 暴露给上层的任务入口, 一个action対应一个接口
        void start_execution_move_joints(std::shared_ptr<GoalHandleMoveJoints> goal_handle);

        void start_execution_move_pose(std::shared_ptr<GoalHandleMovePose> goal_handle);

    private:
           //一个action対应一个接口
        void execution_loop_move_joints(std::shared_ptr<GoalHandleMoveJoints> goal_handle);

        void execution_loop_move_pose(std::shared_ptr<GoalHandleMovePose> goal_handle);
        void publishJointState();


        // bool isRunMode(CallbackResult &r);
        bool isMotorError();
        bool checkJointsAngleOk(std::vector<double> angles);

        // CallbackResult _setPose(const Json::Value &params);                    ///< 设置笛卡尔空间位姿

        // CallbackResult _getFollowTrajectoryState(const Json::Value &params);   ///< 获取轨迹跟随的状态

        // CallbackResult _stopFollowTrajectory(const Json::Value &params);       ///< 停止轨迹跟随

        // CallbackResult _getJointAngles(const Json::Value &params);            ///< 获取关节角度

        /**
         * @brief 笛卡尔空间直线插补
         * 
         * @param start 
         * @param end 
         * @param point_num 
         * @return std::vector<std::vector<double>>  插补的点轨迹
         */
        // std::vector<std::vector<double>> _cartesianPositionLinearInterpolation(const std::vector<double> start, const std::vector<double> end,
        //                                                                        unsigned int point_num);
        

        // CallbackResult __handleMoveJTrajectory(const std::vector<double> joint_target_pos, double motion_rate);  ///< 被__setJointAngles() 调用


        int32_t m_cnt_per_cycle = 6619136;       //钛虎模组编码器负载端编码器一圈脉冲数 16位，65536*101

        std::vector<int32_t> m_joint_zero;    //关节零点
        std::vector<int8_t>  m_joint_dir;     //关节方向
        std::vector<uint16_t> m_jointID;

        std::thread execution_thread_move_pose_;
        std::thread execution_thread_move_joints_;
        std::atomic<bool> is_executing_leg_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        sensor_msgs::msg::JointState joint_state_msg_;
        
        // devices
        // Leg *m_leg;    ///< 底层机械臂类，用于控制电机模组




        // struct {
        //     std::vector<double> pos_upper_limit = {0.99, 3.1415, 2.35, 1.57};
        //     std::vector<double> pos_lower_limit = {0.0, 0.0, 0.0, -1.57};
        //     std::vector<double> max_vel = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        //     std::vector<double> max_acc = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
        //     std::vector<double> max_jerk = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
        // } JointPlannerLimit;

        // enum LegStatus {
        //     E_NO_ERROR = 0,
        //     E_PITCH_DANGER = 1,
        //     E_CAL_COM_FAIL = 2,
        //     E_IK_FAIL = 3,
        // };

        // double mass_center_limit_ = 0.25;   ///< 机器人质心半径极限

    };
} // namespace miracle

#endif // MIRACLE_LOWER_BODY_MODULE_HPP
