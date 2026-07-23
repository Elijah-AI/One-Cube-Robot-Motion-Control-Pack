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
# Description: Miracle module head file
# Version:
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
#ifndef MIRACLE_MODULE_HPP
#define MIRACLE_MODULE_HPP

#include <functional>
#include <memory>
#include <string>
#include <sys/time.h>
#include <thread>
#include <time.h>
#include <vector>
#include <atomic> 
#include <map>
#include <chrono>
#include <mutex>
#include "ethercat_bus_manager.hpp"
#include "miracle_m_algo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "jsoncpp/json/json.h"


namespace miracle {

    #define MIRACLE_MODULE_NAME_MAX_LEN 64
    #define MIRACLE_PI 3.1415926536
    #define RAD_TO_DEGREE 57.295779513
    #define DEGREE_TO_RAD 0.0174532925

    #define JOINT_THRESHOLD 0.002
    #define JOINT_LIMIT_ADJUST_RAD 0.03
    #define JOINT_MAX_OVER_RAD 0.01

    #define MAX_RETRY_TIMES 6

    #define HAND_JOINTNUM     7
    #define ARM_JOINTNUM     7
    #define LEG_JOINTNUM     4
    #define HEAD_JOINTNUM    2
    #define CHASSIS_JOINTNUM 2
    #define CHASSIS_ULTRASONICNUM 8

    typedef enum {
        MODULE_WORK_NORMAL = 0,
        MODULE_WORK_LOCAL_CONTROL = 1,

        MODULE_WORK_MAX,
    } MiracleModuleWorkState;

    typedef void (*miracleJointsDataCB)(std::vector<double> joints_pos, std::vector<double> joints_vel, std::vector<double> joints_current);

    typedef void (*miracleModuleStatusCB)(std::vector<uint64_t> status);

    typedef void (*miracleUltrasonicDataCB)(std::vector<double> ultrasonic_state);

    struct CallbackResult {
            /**
            * @brief Status code indicating the execution status of the callback.
            *
            * The value of the status code corresponds to different execution states,
            * such as success, failure, error, etc. The default status code is 2,
            * which can be modified as needed.
            * 0: Success
            * 1: Failure
            * 2: Default empty status code
            * 3: Reserved
            * 4: Reserved
            * 5: Reserved
            */
            unsigned int status_code;

            /**
            * @brief Message describing the detailed information about the execution of the callback.
            *
            * This message includes additional explanations about the execution status,
            * error descriptions, etc. If no message needs to be conveyed, it can be left empty.
            * It can also serve as a string data field.
            */
            std::string message;

            /**
            * @brief JSON object containing additional data.
            *
            * This JSON object can be used to store any data relevant to the execution
            * result of the callback. If no additional data needs to be passed, it can
            * remain an empty JSON object.
            */
            Json::Value data;

            /**
            * @brief Constructor to initialize a CallbackResult object.
            *
            * @param status_code Status code, default is 2.
            * @param message Message description, default is an empty string.
            * @param data JSON data, default is an empty JSON object.
            */
            CallbackResult(unsigned int status_code = 2, const std::string &message = "", const Json::Value &data = Json::Value())
                : status_code(status_code), message(message), data(data) {
            }
    };
} // namespace miracle


namespace miracle {

    /**
     * @brief MiracleModule基类
     * 
     */
    class MiracleModule {
    public:
        MiracleModule(const char *module_name, const rclcpp::Node::SharedPtr node);

        /*
        ** set joints position data callback for upper
        */

        /**
         * @brief 设置关节位置速度电流回调函数
         * 
         * @param cb 
         */
        void setJointsDataCB(miracleJointsDataCB cb);

        /*
        ** set module status callback for upper
        */

        /**
         * @brief 设置模块状态回调函数
         * 
         * @param cb 
         */
        void setModuleStatusCB(miracleModuleStatusCB cb);

        /**
         * @brief 设置雷达状态回调函数
         * 
         * @param cb 
         */
        void setUltrasonicDataCB(miracleUltrasonicDataCB cb);

    protected:
        rclcpp::Node::SharedPtr lockNode(const char *context) const;

    public:

        /*
        ** use to judge whether module is match cmd
        */

        /**
         * @brief 判断模块类成员中的模块名称与参数是否匹配
         * 
         * @param module_name 
         * @return true 
         * @return false 
         */
        bool moduleMatch(const std::string module_name);

        /*
        ** modude function run
        */

        /**
         * @brief 寻找相应函数并运行
         * 
         * @param key  map中的key
         * @param json_data 传入的json数据
         * @return CallbackResult  返回结构体
         */
        CallbackResult tryExecuteFunction(const std::string &key, const Json::Value &json_data);

        /*
        ** whether this module is ininted
        */

        /**
         * @brief 判断模块是否初始化
         * 
         * @return true 
         * @return false 
         */
        bool isInited(void) {
            return m_inited;
        }
     

        /*
        ** power state handle
        */

        /**
         * @brief 检查电源状态
         * 
         * @param r 
         * @return true 电源状态异常
         * @return false 电源状态正常
         */
        // bool checkPowerState(CallbackResult &r);

        /**
         * @brief 检查模块运行状态
         * 
         * @param r 
         * @return true 正常
         * @return false 异常
         */
        // bool check_module_running(CallbackResult &r);

        //bool async_upgrade(std::string &file, std::string &version, CallbackResult &result);
        //bool upgrade(std::string file_path);

        // CallbackResult __get_version(const Json::Value &params);                            ///< 获取版本号
        // CallbackResult __get_sn(const Json::Value &params);                                 ///< 获取sn号
        // CallbackResult __upgrade_firmware(const Json::Value &params);                       ///< 升级mcu固件
        // CallbackResult _getWorkMode(const Json::Value &params);                             ///< 获取模块工作模式

        /*
        ** init status msg basic info
        */

        /**
         * @brief 接口函数，初始化status_msg的基础信息
         * 
         * @param status_msg 
         */
        // virtual void initStatusMsg(robot_msg::msg::MiracleModuleStatus &status_msg) = 0;

        /*
        ** init all extra device and system 
        */

        /**
         * @brief 接口函数，初始化所以外部设备
         * 
         * @param com_name 
         * @param power_manager 
         */
        virtual void initDevices(std::shared_ptr<EthercatBusManager> &bus_manager) = 0;

        /*
        ** deinit all extra device and system
        ** 
        */

        /**
         * @brief 接口函数，反初始化外部设备，
         * 
         */
        virtual void deinitDevices(void) = 0;

    
        virtual void recvDataThread(void) = 0;


        // for workmode
        // void setWorkmode(uint32_t mode);                               ///< 设置模块工作模式

        // state, used for json server function match
        bool m_inited;                                                 ///< 模块初始化状态


        // module state
        // uint8_t m_module_state;                                       ///< 模块状态

        // module data effect
        // bool m_module_data_effect;                                   ///< 模块执行器数据有无状态

        // module work mode
        uint32_t m_module_work_mode;                                 ///< 模块工作模式

        // module name
        char m_module_name[MIRACLE_MODULE_NAME_MAX_LEN];  ///< 模块名，例如left_arm, right_arm, leg, base ,head
        // char m_bus_name[MIRACLE_BUS_NAME_MAX_LEN];        ///< 总线名称

        // version
        // char m_version[64];                            ///< 版本号
                   
        // sn
        // char m_sn[64];                                ///< sn号

        // System *m_sys;                               ///< 系统信息类
        // Iap *m_iap;                                  ///< 升级信息类

        // function map for module
        std::map<std::string, std::function<CallbackResult(const Json::Value &)>> m_function_map;   ///< 模块函数map


        // bus state check thread
        std::shared_ptr<std::thread> m_ethercat_recv_handler;   //获取数据线程
        std::atomic<bool>  m_recv_thread_stop;

        // module state
        std::vector<uint64_t> m_module_status;           ///< 模块电机状态，错误码
        miracleModuleStatusCB m_module_status_cb;        ///< 模块状态回调
        std::vector<int> m_module_enabled;               // 电机使能状态
        std::vector<bool> has_logged_error;



        // joint info

        /**
         * @brief 关节信息相关变量
         * 
         */
        std::vector<double> m_joints_min_pos;
        std::vector<double> m_joints_max_pos;
        // record current joint angle
        std::vector<double> m_joints_cur_pos;
        std::vector<double> m_joints_cur_speed;
        std::vector<double> m_joints_cur_current;
        std::vector<double> m_joints_last_pos;
        std::vector<double> m_ultrasonic_value;

        std::vector<int32_t> m_recv_cnt;

        miracleJointsDataCB m_joints_data_cb;        ///< 关节数据回调
        miracleUltrasonicDataCB m_ultrasonic_data_cb;

        // for power manager
        bool m_no_power;

    protected:
        std::shared_ptr<EthercatBusManager> m_bus_manager;
        virtual ~MiracleModule();

    public:
        // std::shared_ptr<ChestPower> m_chest_power;

        rclcpp::Node::WeakPtr m_node;

        // 这里的MiracleMAlgo指机械臂运动学动力学模型类
        std::shared_ptr<MiracleMAlgo> m_galgo;  
        std::string m_gmca_chain_name;         ///< urdf中连杆链名，例如left_arm, right_arm
        std::string m_chain_start, m_chain_end;

        
        // for follow trajectory

        /**
         * @brief 轨迹控制相关变量 
         * 
         */
        bool m_traj_run;
    //     bool m_traj_cur_stop;
        bool m_traj_run_new;
        bool m_traj_run_stop;

        bool m_set_pose_stop;
        bool m_is_set_pose;

        bool m_cloud_collision_run_stop;

        bool motor_not_enabled = false;
   
    //     int m_traj_freq;
    //     int m_traj_buf_total_cnt;
        int m_traj_point_cnt;
        int m_traj_point_index;
        int delta_index = 5;
        std::vector<double> m_traj_buffer_pos;

        std::vector<std::vector<double>> m_traj_pos;
        std::vector<std::vector<double>> m_traj_vel;
        std::vector<std::vector<double>> m_traj_acc;
        pthread_mutex_t m_traj_mutex;
    };
} // namespace miracle
#endif
