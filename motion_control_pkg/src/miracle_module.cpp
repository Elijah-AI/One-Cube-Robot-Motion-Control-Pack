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
# Description: Miracle module src file
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

#include "miracle_module.hpp"
#include <cstring>

using namespace miracle;

namespace miracle {

    MiracleModule::MiracleModule(const char *module_name, const rclcpp::Node::SharedPtr node) {
        memset(m_module_name, 0, sizeof(m_module_name));
        if (module_name) {
            size_t len = strlen(module_name);
            memcpy(m_module_name, module_name, len > sizeof(m_module_name) ? sizeof(m_module_name) : len);
        }

        m_function_map.clear();

        m_inited = false;


        // m_module_state = 1; //System::HEART_NONE;

        // m_module_data_effect = false;

        // work mode
        // m_module_work_mode = 0;


        m_joints_data_cb = NULL;
        m_module_status_cb = NULL;
        m_ultrasonic_data_cb=NULL;

        m_no_power = false;
        m_bus_manager = nullptr;


        // for gmca
        m_galgo = NULL;
        m_gmca_chain_name = m_module_name;
        m_node = node;

        m_ethercat_recv_handler = NULL;   //获取数据线程
        m_recv_thread_stop = false;

        // for follow trajectory
        m_traj_run = false;
        // m_traj_cur_stop = false;
        m_traj_run_new = false;
        m_traj_run_stop = false;
        // m_traj_freq = 50;
        m_traj_point_cnt = 0;
        m_traj_point_index = 0;
        m_traj_mutex = PTHREAD_MUTEX_INITIALIZER;
    }

    MiracleModule::~MiracleModule() = default;

    rclcpp::Node::SharedPtr MiracleModule::lockNode(const char *context) const {
        auto node = m_node.lock();
        if (!node) {
            RCLCPP_ERROR(rclcpp::get_logger("miracle_module"), "Node expired in %s", context);
        }
        return node;
    }

    /*
    ** set joints position data callback for upper
    */
    void MiracleModule::setJointsDataCB(miracleJointsDataCB cb) {
        m_joints_data_cb = cb;
    }

    /*
    ** set module status callback for upper
    */
    void MiracleModule::setModuleStatusCB(miracleModuleStatusCB cb) {
        m_module_status_cb = cb;
    }

    bool MiracleModule::moduleMatch(const std::string module_name) {
        if (m_inited && (0 == module_name.compare(m_module_name))) {
            return true;
        }
        return false;
    }

    /*
    ** set ultrasonic data callback for upper
    */
    void MiracleModule::setUltrasonicDataCB(miracleUltrasonicDataCB cb) {
        m_ultrasonic_data_cb = cb;
    }

    CallbackResult MiracleModule::tryExecuteFunction(const std::string &key, const Json::Value &json_data) {
        CallbackResult result;
        if (m_function_map.find(key) != m_function_map.end()) {
            result = m_function_map[key](json_data);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("miracle_module"),"%s unsupport function %s", m_module_name, key.c_str());
            result.status_code = 1;
            result.message = "Error: Unknown command type";
        }
        return result;
    }


    // bool MiracleModule::checkPowerState(CallbackResult &r) {   //TODO
    //     // if (m_no_power) {
    //     //     RCLCPP_WARN(rclcpp::get_logger("miracle_module"),"%s no motor power", m_module_name);
    //     //     r.status_code = 1;
    //     //     r.message = "Error: no motor power";
    //     //     return true;
    //     // }
    //     // if (m_chassis_power && m_chassis_power->isBatteryLow()) {
    //     //     RCLCPP_WARN(rclcpp::get_logger("miracle_module"),"%s battery low", m_module_name);
    //     //     r.status_code = 1;
    //     //     r.message = "Error: battery low";
    //     //     return true;
    //     // }
    //     return false;
    // }

    
} // namespace miracle
