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
# ERROR, PLEASE NOTIFY Miracle, INC. IMMEDIATELY AND DELETE IT FROM YOUR SYSTEM.
#
#####################################################################################
#
# Description: Miracle gmca lib use
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
#ifndef MIRACLE_MOTION_ALGO_HPP_
#define MIRACLE_MOTION_ALGO_HPP_

#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <array>
#include <Eigen/Dense>
#include <mutex>
#include <optional>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>

#include "jsoncpp/json/json.h"
#include "rclcpp/rclcpp.hpp"
#include "ruckig/ruckig.hpp"
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
// #include <urdf/model.h>
// #include "pinocchio/spatial/se3.hpp"
// #include <chrono>
// #include "toppra/toppra.hpp"

#define JOINTS_THRESHOLD 0.002
#define CARTES_V_MAX 1.8
#define CARTES_A_MAX 7.2
#define CARTES_J_MAX 36.0

typedef Eigen::VectorXd VectorXd;
typedef Eigen::MatrixXd MatrixXd;

typedef Eigen::Vector3d Vector3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 3> Matrix33d;
typedef Eigen::Matrix<double, 2, 1> VectorHead;
typedef Eigen::Matrix<double, 4, 1> VectorLeg;
typedef Eigen::Matrix<double, 7, 1> VectorArm;
typedef Eigen::Matrix<double, 7, 1> vectorpose;   //x,y,z+四元数

// using json = nlohmann::json;

//碰撞检测
const std::vector<std::pair<std::string, std::string>> collision_pairs = {
    {"Link1_L", "gripper_base_r_link"},
  
    {"Link2_L", "head_pitch_link"},
    {"Link2_L", "gripper_base_r_link"},

    {"Link3_L", "head_pitch_link"},
    {"Link3_L", "body_link"},
    {"Link3_L", "leg_link3"},
    {"Link3_L", "gripper_base_r_link"},

    {"Link4_L", "head_pitch_link"},
    {"Link4_L", "body_link"},
    {"Link4_L", "leg_link3"},
    {"Link4_L", "leg_link2"},
    {"Link4_L", "leg_link1"},
    {"Link4_L", "Link3_R"},
    {"Link4_L", "Link4_R"},
    {"Link4_L", "Link5_R"},
    {"Link4_L", "Link6_R"},
    {"Link4_L", "Link7_R"}, 
    {"Link4_L", "gripper_base_r_link"},

    {"Link5_L", "head_pitch_link"},
    {"Link5_L", "body_link"},
    {"Link5_L", "leg_link3"},
    {"Link5_L", "leg_link2"},
    {"Link5_L", "leg_link1"},
    {"Link5_L", "Link3_R"},
    {"Link5_L", "Link4_R"},
    {"Link5_L", "Link5_R"},
    {"Link5_L", "Link6_R"},
    {"Link5_L", "Link7_R"},
    {"Link5_L", "gripper_base_r_link"},

    {"Link6_L", "head_pitch_link"},
    {"Link6_L", "body_link"},
    {"Link6_L", "leg_link3"},
    {"Link6_L", "leg_link2"},
    {"Link6_L", "leg_link1"},
    {"Link6_L", "Link3_R"},
    {"Link6_L", "Link4_R"},
    {"Link6_L", "Link5_R"},
    {"Link6_L", "Link6_R"}, 
    {"Link6_L", "Link7_R"},
    {"Link6_L", "gripper_base_r_link"},   
    {"Link6_L", "base_link"},


    {"Link7_L", "head_pitch_link"},
    {"Link7_L", "body_link"},
    {"Link7_L", "leg_link3"},
    {"Link7_L", "leg_link2"},
    {"Link7_L", "leg_link1"},
    {"Link7_L", "Link3_R"},
    {"Link7_L", "Link4_R"},
    {"Link7_L", "Link5_R"},
    {"Link7_L", "Link6_R"},
    {"Link7_L", "Link7_R"},
    {"Link7_L", "gripper_base_r_link"}, 
    {"Link7_L", "base_link"},

    
    {"gripper_base_l_link", "head_pitch_link"},
    {"gripper_base_l_link", "body_link"},
    {"gripper_base_l_link", "leg_link3"},
    {"gripper_base_l_link", "leg_link2"},
    {"gripper_base_l_link", "leg_link1"},
    {"gripper_base_l_link", "Link1_R"},
    {"gripper_base_l_link", "Link2_R"},
    {"gripper_base_l_link", "Link3_R"},
    {"gripper_base_l_link", "Link4_R"},
    {"gripper_base_l_link", "Link5_R"},
    {"gripper_base_l_link", "Link6_R"},
    {"gripper_base_l_link", "Link7_R"},
    {"gripper_base_l_link", "gripper_base_r_link"}, 
    {"gripper_base_l_link", "base_link"},

    {"Link2_R", "head_pitch_link"},

    {"Link3_R", "head_pitch_link"},
    {"Link3_R", "body_link"},
    {"Link3_R", "leg_link3"},

    {"Link4_R", "head_pitch_link"},
    {"Link4_R", "body_link"},
    {"Link4_R", "leg_link3"},
    {"Link4_R", "leg_link2"},
    {"Link4_R", "leg_link1"},

    {"Link5_R", "head_pitch_link"},
    {"Link5_R", "body_link"},
    {"Link5_R", "leg_link3"},
    {"Link5_R", "leg_link2"},
    {"Link5_R", "leg_link1"},

    {"Link6_R", "head_pitch_link"},
    {"Link6_R", "body_link"},
    {"Link6_R", "leg_link3"},
    {"Link6_R", "leg_link2"},
    {"Link6_R", "leg_link1"},
    {"Link6_R", "base_link"},

    {"Link7_R", "head_pitch_link"},
    {"Link7_R", "body_link"},
    {"Link7_R", "leg_link3"},
    {"Link7_R", "leg_link2"},
    {"Link7_R", "leg_link1"},
    {"Link7_R", "base_link"},

    {"gripper_base_r_link", "head_pitch_link"},
    {"gripper_base_r_link", "body_link"},
    {"gripper_base_r_link", "leg_link3"},
    {"gripper_base_r_link", "leg_link2"},
    {"gripper_base_r_link", "leg_link1"},
    {"gripper_base_r_link", "base_link"},
};


const std::vector<std::string> arm_collision_detect= {//手臂与目标点云碰撞检测。 
    "Link3_L",
    "Link4_L",
    "Link5_L",
    "Link6_L",
    "Link7_L",

    "Link3_R",
    "Link4_R",
    "Link5_R",
    "Link6_R",
    "Link7_R",
};



/**
 * @brief 运动学，动力学中间层类, 需要增加路径规划，轨迹规划等功能。
 * 
 */
class MiracleMAlgo {
public:

    /**
     * @brief 构造函数
     * 
     * @param model_name 
     * @param cali_config_path 
     * @param description_path 
     */
    MiracleMAlgo(const std::string& urdf_path, const std::string& mesh_dir);

    ~MiracleMAlgo() = default;
    /*
    ** set joints limit adjust for chain
    **/
   
    /**
     * @brief 获取所选链上的关节角度范围
     * 
     * @param module_id   模块ID - 0：left_arm - 1：right_arm - 3：head - 4：leg
     * @param min_angles  输出各个关节的最小角，单位：rad
     * @param max_angles  输出各个关节的最大角，单位：rad
     * @return true 
     * @return false 
     */
    bool getChainJointsLimt(int module_id, std::vector<double> &min_angles, std::vector<double> &max_angles);

    
    std::vector<double> getLegJoints(void);
    /*
    ** update chain joints angle(rad)
    **/

    /**
     * @brief 更新所选链上的关节角度 rad
     * 
     * @param module_name   模块名称 
     * @param cur_joint_angles  模块角度 
     * @return int 7
     */
    int updateChainJointsAngle(
        const std::string &module_name,
        const std::vector<double> &cur_joint_angles);
    /*
    ** update chain joints velocity(rad/s)
    **/

    // Set the collision pairs explicitly
    void setupInternalCollisionPairs();

    // --- Collision Settings ---
    // Add an external collision box
    void addExternalCollisionBox(const std::string& name, 
                                 const Eigen::Vector3d& size, 
                                 const Eigen::Isometry3d& pose);
                                 
    // Remove an external collision object by name
    void removeExternalCollisionObject(const std::string& name);
                                 
    // Filter adjacent links to prevent "invalid start state" due to permanent self-collision
    // void disableAdjacentLinkCollisions();
    // Manually disable collision between two geometries
    void disableCollisionPair(const std::string& geom1_name, const std::string& geom2_name);

    // --- Dual Arm Planning ---
    // Plan path in joint space (start/path use full-body q; goal uses 14D arm joints: left[0..6], right[7..13])
    bool planDualArmJointSpace(const std::vector<double>& start_q,
                               const std::vector<double>& goal_q,
                               std::vector<std::vector<double>>& path);

    // 全身关节空间避障规划。goal_q 为完整模型配置，活动关节按
    // 左臂(7)、右臂(7)、腿部(4) 的顺序由 use_left/use_right 选择。
    bool planWholeBodyJointSpace(const std::vector<double>& start_q,
                                 const std::vector<double>& goal_q,
                                 bool use_left,
                                 bool use_right,
                                 std::vector<std::vector<double>>& path);

    // Plan to Cartesian goals for both arms
    bool planDualArmCartesianGoal(const std::vector<double>& start_q,
                                  const Eigen::Isometry3d& left_goal,
                                  const Eigen::Isometry3d& right_goal,
                                  std::vector<std::vector<double>>& path);

    // Plan Cartesian straight-line paths for both arms
    bool planDualArmCartesianPath(const std::vector<double>& start_q,
                                  const Eigen::Isometry3d& left_goal,
                                  const Eigen::Isometry3d& right_goal,
                                  std::vector<std::vector<double>>& path,
                                  double step_size = 0.01);

    bool planDualArmCartesianLine(const std::vector<double>& start_q,
                                           const std::vector<double>& left_end_poseture,
                                           const std::vector<double>& right_end_poseture,
                                           const double& dt,const double& v_max,
                                           const double& a_max, const double& j_max,
                                           std::vector<std::vector<double>>& path);

    // --- Single Arm Planning ---
    // start_q/path use full-body q. The inactive joints remain fixed during planning.
    bool planSingleArmJointSpace(bool is_left,
                                 const std::vector<double>& start_q,
                                 const std::vector<double>& goal_q, // Active arm's 7D goal
                                 std::vector<std::vector<double>>& path);

    bool planSingleArmCartesianGoal(bool is_left,
                                    const std::vector<double>& start_q,
                                    const Eigen::Isometry3d& goal,
                                    std::vector<std::vector<double>>& path);

    // Stage 7 只暴露现有 Pinocchio IK 与按名称建立的 q 索引适配器。
    // 业务层不得复制 IK、固定切片或直接访问内部全身状态。
    bool solveSingleArmCartesianIk(bool is_left,
                                   const std::vector<double>& start_q,
                                   const Eigen::Isometry3d& goal,
                                   std::vector<double>& goal_q);
    // 仅适配现有 solveDualIK：输出 left[0..6], right[7..13]，不复制 IK。
    bool solveDualArmCartesianIk(const std::vector<double>& start_q,
                                 const Eigen::Isometry3d& left_goal,
                                 const Eigen::Isometry3d& right_goal,
                                 std::vector<double>& goal_q);
    // 全身位置逆解：只激活请求的手臂和腿部关节，腿部通过加权最小二乘
    // 尽量保持在 q_init 附近。leg_weights 顺序为
    // leg_joint1、leg_joint2、leg_joint3、body_joint；left_goal/right_goal
    // 至少提供一个。
    bool solveWholeBodyIK(
        const Eigen::VectorXd& q_init,
        const std::optional<Eigen::Isometry3d>& left_goal,
        const std::optional<Eigen::Isometry3d>& right_goal,
        Eigen::VectorXd& q_res,
        const std::array<double, 4>& leg_weights =
            std::array<double, 4>{100.0, 100.0, 100.0, 20.0},
        double body_pitch_limit_rad = 30.0 * 0.017453292519943295);
    bool extractSingleArmJoints(bool is_left,
                                const std::vector<double>& full_q,
                                std::vector<double>& arm_q) const;
    bool extractLegJoints(const std::vector<double>& full_q,
                          std::vector<double>& leg_q) const;
    // 将时间参数化后的活动关节向量写回完整配置。活动关节顺序与
    // planWholeBodyJointSpace 相同，reference_full_q 中的其他关节保持不变。
    bool mergeWholeBodyJoints(const std::vector<double>& reference_full_q,
                              bool use_left,
                              bool use_right,
                              const std::vector<double>& active_joints,
                              std::vector<double>& full_q) const;
    bool getFullJointState(std::vector<double>& full_q) const;

    // Pinocchio IK 使用模型根坐标系目标。若中央 planning_frame 不是根帧，
    // 仍由同一模型按当前全身 q 完成坐标变换，不引入第二套 TF/运动学实现。
    bool transformPoseToModelRoot(const std::string& source_frame,
                                  const std::vector<double>& full_q,
                                  const Eigen::Isometry3d& source_to_target,
                                  Eigen::Isometry3d& root_to_target);

    bool planSingleArmCartesianPath(bool is_left,
                                    const std::vector<double>& start_q,
                                    const Eigen::Isometry3d& goal,
                                    std::vector<std::vector<double>>& path,
                                    double step_size = 0.01);

    bool planSingleArmCartesianLine(bool is_left, const std::vector<double>& start_q,
                                    const std::vector<double>& end_poseture,
                                    const double& dt,const double& v_max,
                                    const double& a_max, const double& j_max,
                                    std::vector<std::vector<double>>& path);

    const pinocchio::Model& getModel() const { return model_; }

    bool getForwardKinematics(const std::string& source_frame,
                              const std::string& target_frame,
                              Eigen::Matrix4d& T_source_target);
    bool getFramePoseAtConfiguration(const std::string& source_frame,
                                     const std::string& target_frame,
                                     const std::vector<double>& full_q,
                                     Eigen::Isometry3d& source_to_target);

    // Accept full-body q only，全身关节角度格式转换
    Eigen::VectorXd toEigenConfig(const std::vector<double>& full_q) const;
    /**
     * @brief 检查全身关节角度是否有碰撞
     * 
     * @param full_q  全身关节角度
     * @return true  有效
     * @return false 无效
     */
    bool isFullQValid(const Eigen::VectorXd& full_q);
    /**
     * @brief 二次插值
     * 
     * @param raw_pos  输入位置
     * @param pos      输出位置
     * @param plan_dt  轨迹规划时间步长
     * @param interp_dt  插值时间步长
     * @return int 0：成功，其他：失败
     * */
    int quadraticInterpolation(const std::vector<std::vector<double>> &raw_pos, std::vector<std::vector<double>> &pos, double plan_dt, double interp_dt = 0.004);

    /*
    ** for movej
    **/

    /**
     * @brief 关节空间轨迹规划
     * 
     * @param module_id    模块id,0：left_arm - 1：right_arm - 3：head - 4：leg
     * @param plan_dt      轨迹规划时间步长
     * @param max_vel      最大速度
     * @param max_acc      最大加速度
     * @param max_jerk     最大加加速度
     * @param joint_cur_pos 当前关节位置
     * @param joint_cur_vel 当前关节速度
     * @param joint_cur_acc 当前关节加速度
     * @param joint_tgt_pos 关节空间目标位置
     * @param traj_pos       输出轨迹位置
     * @param traj_vel       输出轨迹速度
     * @param traj_acc       输出轨迹加速度
     * @return int           0：成功，其他：失败,-1是自碰撞 ，-2是轨迹规划失败
     */
    template <int num_joints>
    int computeMoveJTrajectory(int module_id, const double plan_dt, const double max_velocity, const double max_acc, const double max_jerk, const std::vector<double> joint_cur_pos, 
                              const std::vector<double> joint_cur_vel,  const std::vector<double> joint_cur_acc, const std::vector<double> joint_target_pos, 
                              std::vector<std::vector<double>>& traj_pos_new, std::vector<std::vector<double>>& traj_vel_new, 
                              std::vector<std::vector<double>>& traj_acc_new) {   //,const std::string &module_name=" " 
        // int num_joints = joint_cur_pos.size();
        // 根据当前模块id和joint_target_pos更新q_target中的角度
        Eigen::VectorXd q_target = toEigenConfig(_m_full_joint_state_msg);
        
        const std::vector<size_t> *q_indices_ptr = nullptr;
        if (module_id == 0) {
            q_indices_ptr = &left_arm_q_indices_;
        } else if (module_id == 1) {
            q_indices_ptr = &right_arm_q_indices_;
        } else if (module_id == 3) {
            q_indices_ptr = &head_q_indices_;
        } else if (module_id == 4) {
            q_indices_ptr = &leg_q_indices_;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("computeMoveJ Trajectory"), "Unsupported module_id: %d", module_id);
            return -3;
        }

        if (q_indices_ptr->size() != joint_target_pos.size()) {
            RCLCPP_ERROR(rclcpp::get_logger("computeMoveJ Trajectory"), "joint_cur_pos size mismatch: module_id=%d expected=%zu got=%zu",
                         module_id, q_indices_ptr->size(), joint_target_pos.size());
            return -4;
        }

        for (size_t i = 0; i < q_indices_ptr->size(); ++i) {
            q_target[static_cast<Eigen::Index>((*q_indices_ptr)[i])] = joint_target_pos[i];
        }

        if(!isFullQValid(q_target)) {
            RCLCPP_ERROR(rclcpp::get_logger("computeMoveJ Trajectory"), "q_target result collison happened");
            return -1;
        }

        ruckig::InputParameter<num_joints> input;
        for(size_t i = 0; i < num_joints; ++i) {
            input.current_position[i] = joint_cur_pos[i];
            input.current_velocity[i] = joint_cur_vel[i];
            input.current_acceleration[i] = joint_cur_acc[i];
            // input.current_acceleration = {0.0, 2.5, -0.5};
        
            input.target_position[i] = joint_target_pos[i];
            input.target_velocity[i] = 0.0;
            // input.target_acceleration = {0.0, 0.0, 0.5};

            input.max_velocity[i] = max_velocity;
            input.max_acceleration[i] = max_acc;
            input.max_jerk[i] = max_jerk;
        }

        ruckig::Ruckig<num_joints> otg;
        ruckig::Trajectory<num_joints> trajectory;

        ruckig::Result result = otg.calculate(input, trajectory); // Working = 0,  The trajectory is calculated normally
        if (result < 0) {//= ruckig::Result::ErrorInvalidInput) {
            
            RCLCPP_ERROR(rclcpp::get_logger("computeMoveJ Trajectory"),"%s Ruckig  calculate error %d",  __func__, (int)result);
            return -2;
        }
        
        // Get duration of the trajectory
        double total_time = trajectory.get_duration();
        std::array<double, num_joints> new_position, new_velocity, new_acceleration;
        // std::cout << "Trajectory duration: " << trajectory.get_duration() << " [s]." << std::endl;

        for(double new_time = 0.0; new_time < total_time; new_time+=plan_dt) {
            trajectory.at_time(new_time, new_position, new_velocity, new_acceleration);



            std::vector<double> vec_pos{new_position.begin(), new_position.end()};
            std::vector<double> vec_vel{new_velocity.begin(), new_velocity.end()};
            std::vector<double> vec_acc{new_acceleration.begin(), new_acceleration.end()};
                        
            traj_pos_new.emplace_back(vec_pos);
            traj_vel_new.emplace_back(vec_vel);
            traj_acc_new.emplace_back(vec_acc);    

        }    
        
        // trajectory.at_time(total_time, new_position, new_velocity, new_acceleration);
        // std::vector<double> vec_pos{new_position.begin(), new_position.end()};
        // std::vector<double> vec_vel{new_velocity.begin(), new_velocity.end()};
        // std::vector<double> vec_acc{new_acceleration.begin(), new_acceleration.end()};

        // traj_pos_new.emplace_back(vec_pos);
        // traj_vel_new.emplace_back(vec_vel);
        // traj_acc_new.emplace_back(vec_acc);



        // std::cout << "Position at time " << new_time << " [s]: " << join(new_position) << std::endl;

        // // Get some info about the position extrema of the trajectory
        // std::array<Bound, 3> position_extrema = trajectory.get_position_extrema();
        // std::cout << "Position extremas for DoF 4 are " << position_extrema[2].min << " (min) to " << position_extrema[2].max << " (max)" << std::endl;
        
        return 0;
    }


    /*
    ** for movep
    **/

    /**
     * @brief 笛卡尔空间轨迹规划
     * 
     * @param module_id   模块id,0：left_arm - 1：right_arm - 3：head - 4：leg
     * @param plan_dt     轨迹规划时间步长
     * @param max_vel     最大速度
     * @param max_acc     最大加速度
     * @param max_jerk    最大加加速度
     * @param joint_cur_pos 当前关节位置
     * @param joint_cur_vel 当前关节速度
     * @param joint_cur_acc 当前关节加速度
     * @param tgt_pose      笛卡尔空间目标位姿,位置加四元素
     * @param is_tcp_pose   是否为tcp位姿
     * @param traj_pos      输出轨迹位置
     * @param traj_vel      输出轨迹速度
     * @param traj_acc      输出轨迹加速度
     * @param nearest       求逆解选项
     * @return int          0：成功，其他：失败
     */
    template <int num_joints>
    int computeMovePTrajectory(int module_id, const double plan_dt, const double max_velocity, const double max_acc, 
                              const double max_jerk, const std::vector<double> joint_cur_pos, const std::vector<double> joint_cur_vel,
                              const std::vector<double> joint_cur_acc, std::vector<double> tgt_pose, std::vector<std::vector<double>>& traj_pos_new,
                              std::vector<std::vector<double>>& traj_vel_new, std::vector<std::vector<double>> &traj_acc_new) {   //, bool nearest=true
    
        Eigen::VectorXd q_init = toEigenConfig(_m_full_joint_state_msg);
        Eigen::VectorXd q_res;
        // 根据当前模块id和joint_cur_pos更新q_init中的角度
        const std::vector<size_t> *q_indices_ptr = nullptr;
        if (module_id == 0) {
            q_indices_ptr = &left_arm_q_indices_;
        } else if (module_id == 1) {
            q_indices_ptr = &right_arm_q_indices_;
        } else if (module_id == 3) {
            q_indices_ptr = &head_q_indices_;
        } else if (module_id == 4) {
            q_indices_ptr = &leg_q_indices_;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("computeMove PTrajectory"), "Unsupported module_id: %d", module_id);
            return -5;
        }

        if (q_indices_ptr->size() != joint_cur_pos.size()) {
            RCLCPP_ERROR(rclcpp::get_logger("computeMove PTrajectory"), "joint_cur_pos size mismatch: module_id=%d expected=%zu got=%zu",
                         module_id, q_indices_ptr->size(), joint_cur_pos.size());
            return -5;
        }

        for (size_t i = 0; i < q_indices_ptr->size(); ++i) {
            q_init[static_cast<Eigen::Index>((*q_indices_ptr)[i])] = joint_cur_pos[i];
        }

        std::vector<double> target_q(num_joints);
        //tgt_pose为7维向量，位置加四元数，转换为goal变量
        Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
        goal.translation() = Eigen::Vector3d(tgt_pose[0], tgt_pose[1], tgt_pose[2]);
        Eigen::Quaterniond q(tgt_pose[6], tgt_pose[3], tgt_pose[4], tgt_pose[5]); // (w, x, y, z)
        q.normalize();
        goal.linear() = q.toRotationMatrix();
        
        
        if(!solveIK(q_init, goal, q_res, module_id)) {
            RCLCPP_ERROR(rclcpp::get_logger("computeMove PTrajectory"), "ik failed: module_id: %d", module_id);
            return -4;
        }
        
        if(!isFullQValid(q_res)) {
            RCLCPP_ERROR(rclcpp::get_logger("computeMove PTrajectory"), "q_res result collison happened");
            return -5;
        }

        for (size_t j = 0; j < num_joints; j++) {
            target_q[j] = q_res[static_cast<Eigen::Index>((*q_indices_ptr)[j])];
        }
        
        bool pos_same = true;
        for (int i = 0; i < num_joints; i++) {
            if (abs(target_q[i] - joint_cur_pos[i]) > JOINTS_THRESHOLD) {
                pos_same = false;
                break;
            }
        }
        if (pos_same) {
            RCLCPP_INFO(rclcpp::get_logger("computeMove PTrajectory"),"%s %lf from [%lf,%lf] to [%lf,%lf] almost same",  __func__, plan_dt, joint_cur_pos[0], joint_cur_pos[1],
                    target_q[0], target_q[1]); 
            return -6;
        }
        // rc = tracik_solver->CartToJnt(start_q, end_pose, target_q);

        for (size_t j = 0; j < num_joints; j++) {
            RCLCPP_INFO(rclcpp::get_logger("computeMove PTrajectory"),"Target joint%ld: %f", j, target_q[j]);
        }

        ruckig::InputParameter<num_joints> input;
        for(size_t i = 0; i < num_joints; ++i) {
            input.current_position[i] = joint_cur_pos[i];
            input.current_velocity[i] = joint_cur_vel[i];
            input.current_acceleration[i] = joint_cur_acc[i];
            // input.current_acceleration = {0.0, 2.5, -0.5};
        
            input.target_position[i] = target_q[i];
            input.target_velocity[i] = 0.0;
            // input.target_acceleration = {0.0, 0.0, 0.5};
    
            input.max_velocity[i] = max_velocity;
            input.max_acceleration[i] = max_acc;
            input.max_jerk[i] = max_jerk;

            // target_collision_detect.emplace_back(target_q[i]);
        }
        
        ruckig::Ruckig<num_joints> otg;
        ruckig::Trajectory<num_joints> trajectory;
    
        ruckig::Result result = otg.calculate(input, trajectory); // Working = 0,  The trajectory is calculated normally
        if (result < 0) {// == ruckig::Result::ErrorInvalidInput) {
            
            RCLCPP_ERROR(rclcpp::get_logger("computeMove PTrajectory"),"%s Ruckig  calculate error %d",  __func__, (int)result);
            return -7;
        }
    
        // Get duration of the trajectory
        double total_time = trajectory.get_duration();
        std::array<double, num_joints> new_position, new_velocity, new_acceleration;
        // std::cout << "Trajectory duration: " << trajectory.get_duration() << " [s]." << std::endl;


        for(double new_time = 0.0; new_time < total_time; new_time+=plan_dt) {
            trajectory.at_time(new_time, new_position, new_velocity, new_acceleration);
    
            std::vector<double> vec_pos{new_position.begin(), new_position.end()};
            std::vector<double> vec_vel{new_velocity.begin(), new_velocity.end()};
            std::vector<double> vec_acc{new_acceleration.begin(), new_acceleration.end()};
    

            traj_pos_new.emplace_back(vec_pos);
            traj_vel_new.emplace_back(vec_vel);
            traj_acc_new.emplace_back(vec_acc);    
    
        }    
        
        trajectory.at_time(total_time, new_position, new_velocity, new_acceleration);
        std::vector<double> vec_pos{new_position.begin(), new_position.end()};
        std::vector<double> vec_vel{new_velocity.begin(), new_velocity.end()};
        std::vector<double> vec_acc{new_acceleration.begin(), new_acceleration.end()};
    
        traj_pos_new.emplace_back(vec_pos);
        traj_vel_new.emplace_back(vec_vel);
        traj_acc_new.emplace_back(vec_acc);
        
        return 0;
    }

    bool ruckigPlan(
        double dt,
        const std::vector<double>& max_velocity,
        const std::vector<double>& max_acc,
        const std::vector<double>& max_jerk,
        const std::vector<double>& joint_cur_pos,
        const std::vector<double>& joint_cur_vel,
        const std::vector<double>& joint_cur_acc,
        const std::vector<double>& joint_target_pos,
        std::vector<std::vector<double>>& res_trajectory);

    bool toppraPlan(
        const std::vector<std::vector<double>>& ompl_path,
        double dt,
        const std::vector<double>& max_velocity,
        const std::vector<double>& max_acc,
        std::vector<std::vector<double>>& res_trajectory);
    
    // std::shared_ptr<GMCA::GMCA> robot; ///< 运动学，动力学底层类，封装为库

    // std::shared_ptr<GMCAParam> robot; ///< for compile
    // toppra::Vector position0{7}, position1{7}, position2{7}; //for compile


private:
    // Helper to create OMPL setup dynamically based on active joints
    std::shared_ptr<ompl::geometric::SimpleSetup> createPlannerSetup(const std::vector<size_t>& active_indices, 
                                                                     const Eigen::VectorXd& fixed_full_q);
    /**
     * @brief 检查OMPL状态是否有效
     * 
     * @param state  OMPL状态
     * @param active_indices  活动关节索引
     * @param fixed_full_q  全身固定关节关节角度
     * @return true  有效
     * @return false 无效
     */
    bool isStateValid(const ompl::base::State* state, 
                      const std::vector<size_t>& active_indices, 
                      const Eigen::VectorXd& fixed_full_q);

    // Solve Inverse Kinematics (mode: 0=left, 1=right, 3=head, 4=leg)
    bool solveIK(const Eigen::VectorXd& q_init,
                 const Eigen::Isometry3d& goal,
                 Eigen::VectorXd& q_res,
                 int mode);

    bool solveDualIK(const Eigen::VectorXd& q_init,
                     const Eigen::Isometry3d& left_goal,
                     const Eigen::Isometry3d& right_goal,
                     Eigen::VectorXd& q_res);
    
    Eigen::Quaterniond slerp_ruckig(const Eigen::Quaterniond& q0,
                                    const Eigen::Quaterniond& q1,
                                    double t);
    std::string urdf_path_;
    std::string mesh_dir_;

    pinocchio::Model model_;
    pinocchio::GeometryModel geom_model_;
    std::shared_ptr<pinocchio::Data> data_;
    std::shared_ptr<pinocchio::GeometryData> geom_data_;
    // Pinocchio Data/GeometryData 是可变工作区。FK、IK、碰撞和 OMPL
    // 有效性回调共享这些对象，必须通过同一把可重入锁串行访问。
    mutable std::recursive_mutex pinocchio_data_mutex_;

    // Indices in the pinocchio full configuration vector
    std::vector<size_t> left_arm_q_indices_;
    std::vector<size_t> right_arm_q_indices_;
    std::vector<size_t> head_q_indices_;
    std::vector<size_t> leg_q_indices_;
    std::vector<int> left_arm_v_indices_;
    std::vector<int> right_arm_v_indices_;
    std::vector<int> head_v_indices_;
    std::vector<int> leg_v_indices_;

    pinocchio::FrameIndex left_ee_frame_id_;
    pinocchio::FrameIndex right_ee_frame_id_;
    pinocchio::FrameIndex head_ee_frame_id_;
    pinocchio::FrameIndex leg_ee_frame_id_;

    Eigen::VectorXd default_q_; // To keep non-arm joints at default values

    // List of names of external collision objects added
    std::vector<std::string> external_objects_;

    // rclcpp::Node::SharedPtr node_;
    // std::string urdf_param_name_;

    //碰撞检测
    // urdf::Model robot_model; 

    //各关节角度信息
    mutable std::mutex upadate_joinst_state_msg;
    std::vector<double>  _m_full_joint_state_msg;
    std::vector<double>  _m_left_arm_joint_state_msg;
    std::vector<double>  _m_chassis_joint_state_msg;
    std::vector<double>  _m_right_arm_joint_state_msg;
    std::vector<double>  _m_lower_body_joint_state_msg;
    std::vector<double>  _m_head_joint_state_msg;


};

#endif
