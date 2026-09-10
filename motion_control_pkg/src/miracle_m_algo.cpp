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
# Description: Miracle gmca lib use
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

#include "miracle_m_algo.hpp"
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <limits>
#include <random>
#include <sstream>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>
#include <coal/collision.h>
#include <coal/shape/geometric_shapes.h>
// 二次规划(QP)求解器：全身逆解采用 qpOASES 进行速度级数值优化
#include <qpOASES.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include "sensor_msgs/msg/joint_state.hpp"
#include "motion_control_pkg/motion/cartesian_line_sampling.hpp"
#include "motion_control_pkg/motion/toppra_path_preparation.hpp"

#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/parametrizer/const_accel.hpp>
#include <toppra/toppra.hpp>
#include <toppra/algorithm.hpp>
#include <toppra/constraint.hpp>
#include <toppra/geometric_path.hpp>
#include <toppra/parametrizer/spline.hpp>
#include <toppra/solver/seidel.hpp>
// 或者如果有开启的话：#ifdef BUILD_WITH_qpOASES
#include <toppra/solver/qpOASES-wrapper.hpp>

namespace
{

    constexpr double kRadiansToDegrees =
        180.0 / 3.141592653589793238462643383279502884;
    constexpr double kDegreesToRadians = 1.0 / kRadiansToDegrees;
    constexpr double kCcsJoint6ActivationDeg = 20.0;
    constexpr double kCcsJoint67OffsetDeg = 109.5;
    constexpr double kCcsJoint67Slope = 1.025;
    constexpr double kCcsComparisonToleranceDeg = 1.0e-9;
    constexpr double kCcsNullspaceGain = 0.05;
    constexpr int kIkLineSearchAttempts = 12;
    constexpr std::array<double, 5> kSingleArmFallbackJoint6SeedsDeg{
        0.0, 30.0, -30.0, 50.0, -50.0};
    constexpr std::array<std::array<double, 2>, 5>
        kDualArmFallbackJoint6SeedsDeg{{
            {0.0, 0.0},
            {30.0, 30.0},
            {-30.0, -30.0},
            {30.0, -30.0},
            {-30.0, 30.0},
        }};

    bool hasSevenArmJointIndices(
        const Eigen::VectorXd& q,
        const std::vector<size_t>& q_indices)
    {
        return q_indices.size() == 7U &&
            std::all_of(q_indices.begin(), q_indices.end(), [&q](size_t index) {
                return index < static_cast<size_t>(q.size());
            });
    }

    double simplifiedCcsJoint7LimitDeg(double joint6_deg)
    {
        return kCcsJoint67OffsetDeg -
            kCcsJoint67Slope * std::abs(joint6_deg);
    }

    bool passesSimplifiedCcsJoint67Constraint(
        const Eigen::VectorXd& q,
        const std::vector<size_t>& q_indices)
    {
        if (!hasSevenArmJointIndices(q, q_indices)) {
            return false;
        }
        const double joint6_deg =
            q[static_cast<Eigen::Index>(q_indices[5])] * kRadiansToDegrees;
        const double joint7_deg =
            q[static_cast<Eigen::Index>(q_indices[6])] * kRadiansToDegrees;
        if (!std::isfinite(joint6_deg) || !std::isfinite(joint7_deg)) {
            return false;
        }
        if (std::abs(joint6_deg) <= kCcsJoint6ActivationDeg) {
            return true;
        }
        const double joint7_limit_deg = simplifiedCcsJoint7LimitDeg(joint6_deg);
        return joint7_limit_deg >= 0.0 &&
            std::abs(joint7_deg) <=
            joint7_limit_deg + kCcsComparisonToleranceDeg;
    }

    bool projectToSimplifiedCcsJoint67Constraint(
    Eigen::VectorXd& q,
    const std::vector<size_t>& q_indices)
    {
        if (!hasSevenArmJointIndices(q, q_indices)) {
            return false;
        }
        const Eigen::Index joint6_index =
            static_cast<Eigen::Index>(q_indices[5]);
        const Eigen::Index joint7_index =
            static_cast<Eigen::Index>(q_indices[6]);
        const double joint6_deg = q[joint6_index] * kRadiansToDegrees;
        double joint7_deg = q[joint7_index] * kRadiansToDegrees;
        if (!std::isfinite(joint6_deg) || !std::isfinite(joint7_deg)) {
            return false;
        }
        if (std::abs(joint6_deg) <= kCcsJoint6ActivationDeg) {
            return true;
        }
        const double joint7_limit_deg = simplifiedCcsJoint7LimitDeg(joint6_deg);
        if (joint7_limit_deg < 0.0) {
            return false;
        }
        joint7_deg = std::clamp(
            joint7_deg, -joint7_limit_deg, joint7_limit_deg);
        q[joint7_index] = joint7_deg * kDegreesToRadians;
        return true;
    }

    bool setSimplifiedCcsJoint67Seed(
        Eigen::VectorXd& q,
        const std::vector<size_t>& q_indices,
        double joint6_deg)
    {
        if (!hasSevenArmJointIndices(q, q_indices) ||
            !std::isfinite(joint6_deg))
        {
            return false;
        }
        q[static_cast<Eigen::Index>(q_indices[5])] =
            joint6_deg * kDegreesToRadians;
        q[static_cast<Eigen::Index>(q_indices[6])] = 0.0;
        return passesSimplifiedCcsJoint67Constraint(q, q_indices);
    }

    void addCcsJoint67CenteringVelocity(
        const Eigen::VectorXd& q,
        const std::vector<size_t>& q_indices,
        const std::vector<int>& v_indices,
        Eigen::VectorXd& centering_velocity)
    {
        if (!hasSevenArmJointIndices(q, q_indices) ||
            v_indices.size() != 7U ||
            centering_velocity.size() <= 0)
        {
            return;
        }
        const int joint6_v_index = v_indices[5];
        const int joint7_v_index = v_indices[6];
        if (joint6_v_index < 0 || joint7_v_index < 0 ||
            joint6_v_index >= centering_velocity.size() ||
            joint7_v_index >= centering_velocity.size())
        {
            return;
        }
        centering_velocity[joint6_v_index] =
            -kCcsNullspaceGain *
            q[static_cast<Eigen::Index>(q_indices[5])];
        centering_velocity[joint7_v_index] =
            -kCcsNullspaceGain *
            q[static_cast<Eigen::Index>(q_indices[6])];
    }
}  // namespace


MiracleMAlgo::MiracleMAlgo(const std::string& urdf_path, const std::string& mesh_dir)
    :urdf_path_(urdf_path), mesh_dir_(mesh_dir) {

    // std::filesystem::path urdf_path;
    // try {
    //     std::string share_dir = ament_index_cpp::get_package_share_directory("miracle_description_v2");
    //     urdf_path = std::filesystem::path(share_dir) / "urdf" / "miracle_v2.urdf";
    //     //RCLCPP_INFO(rclcpp::get_logger("kdl"), "URDF file: %s", urdf_path.c_str());
    // } catch (const std::exception &e) {
    //     std::cerr << "Package not found: " << e.what() << std::endl;
    // }
    
    //  if (!robot_model.initFile(urdf_path)) {
    //     RCLCPP_ERROR(rclcpp::get_logger("kdl"), "Failed to parse URDF file!");
    // }
    pinocchio::urdf::buildModel(urdf_path_, model_);
    pinocchio::urdf::buildGeom(model_, urdf_path_, pinocchio::COLLISION, geom_model_, mesh_dir_);
    
    data_ = std::make_shared<pinocchio::Data>(model_);
    geom_data_ = std::make_shared<pinocchio::GeometryData>(geom_model_);

    std::vector<std::string> left_joint_names = {
        "Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L",
        "Joint5_L", "Joint6_L", "Joint7_L"
    };
    std::vector<std::string> right_joint_names = {
        "Joint1_R", "Joint2_R", "Joint3_R", "Joint4_R",
        "Joint5_R", "Joint6_R", "Joint7_R"
    };
    std::vector<std::string> head_joint_names = {"head_yaw_joint", "head_pitch_joint"};
    std::vector<std::string> leg_joint_names = {"leg_joint1", "leg_joint2", "leg_joint3", "body_joint"};

    const std::string left_ee_frame_primary = "gripper_base_l_link";
    const std::string left_ee_frame_fallback = "gripper_base_l_link";
    const std::string right_ee_frame_primary = "gripper_base_r_link";
    const std::string right_ee_frame_fallback = "gripper_base_r_link";
    const std::string head_ee_frame_primary = "head_camera_link";
    const std::string head_ee_frame_fallback = "head_pitch_link";
    const std::string leg_ee_frame_primary = "body_link";
    const std::string leg_ee_frame_fallback = "base_link";

    left_arm_q_indices_.resize(7);
    right_arm_q_indices_.resize(7);
    left_arm_v_indices_.resize(7);
    right_arm_v_indices_.resize(7);
    head_q_indices_.clear();
    leg_q_indices_.clear();
    head_v_indices_.clear();
    leg_v_indices_.clear();

    for(int i=0; i<7; ++i) {
        if(model_.existJointName(left_joint_names[i])) {
            pinocchio::JointIndex jid = model_.getJointId(left_joint_names[i]);
            left_arm_q_indices_[i] = model_.joints[jid].idx_q();
            left_arm_v_indices_[i] = model_.joints[jid].idx_v();
        } else {
            RCLCPP_WARN(rclcpp::get_logger("MiracleMAlgo"), "joint %s not found!", left_joint_names[i].c_str());
        }
        if(model_.existJointName(right_joint_names[i])) {
            pinocchio::JointIndex jid = model_.getJointId(right_joint_names[i]);
            right_arm_q_indices_[i] = model_.joints[jid].idx_q();
            right_arm_v_indices_[i] = model_.joints[jid].idx_v();
        } else {
            RCLCPP_WARN(rclcpp::get_logger("MiracleMAlgo"), "joint %s not found!", right_joint_names[i].c_str());
        }
    }
    for(const auto& name : head_joint_names) {
        if(model_.existJointName(name)) {
            pinocchio::JointIndex jid = model_.getJointId(name);
            head_q_indices_.push_back(model_.joints[jid].idx_q());
            head_v_indices_.push_back(model_.joints[jid].idx_v());
        } else {
            RCLCPP_WARN(rclcpp::get_logger("MiracleMAlgo"), "joint %s not found!", name.c_str());
        }
    }
    for(const auto& name : leg_joint_names) {
        if(model_.existJointName(name)) {
            pinocchio::JointIndex jid = model_.getJointId(name);
            leg_q_indices_.push_back(model_.joints[jid].idx_q());
            leg_v_indices_.push_back(model_.joints[jid].idx_v());
        } else {
            RCLCPP_WARN(rclcpp::get_logger("MiracleMAlgo"), "joint %s not found!", name.c_str());
        }
    }

    default_q_ = pinocchio::neutral(model_);
    _m_full_joint_state_msg.assign(default_q_.data(), default_q_.data() + default_q_.size());

    if(model_.existFrame(left_ee_frame_primary)) {
        left_ee_frame_id_ = model_.getFrameId(left_ee_frame_primary);
    } else {
        left_ee_frame_id_ = model_.getFrameId(left_ee_frame_fallback);
    }
    
    if(model_.existFrame(right_ee_frame_primary)) {
        right_ee_frame_id_ = model_.getFrameId(right_ee_frame_primary);
    } else {
        right_ee_frame_id_ = model_.getFrameId(right_ee_frame_fallback);
    }
    
    if(model_.existFrame(head_ee_frame_primary)) {
        head_ee_frame_id_ = model_.getFrameId(head_ee_frame_primary);
        RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"), "Head EE frame: %s (id=%u)", head_ee_frame_primary.c_str(), static_cast<unsigned int>(head_ee_frame_id_));
    } else {
        head_ee_frame_id_ = model_.getFrameId(head_ee_frame_fallback);
        RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"), "Head EE frame: %s (id=%u)", head_ee_frame_fallback.c_str(), static_cast<unsigned int>(head_ee_frame_id_));
    }
    
    if(model_.existFrame(leg_ee_frame_primary)) {
        leg_ee_frame_id_ = model_.getFrameId(leg_ee_frame_primary);
        RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"), "Leg EE frame: %s (id=%u)", leg_ee_frame_primary.c_str(), static_cast<unsigned int>(leg_ee_frame_id_));
    } else {
        leg_ee_frame_id_ = model_.getFrameId(leg_ee_frame_fallback);
        RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"), "Leg EE frame fallback: %s (id=%u)", leg_ee_frame_fallback.c_str(), static_cast<unsigned int>(leg_ee_frame_id_));
    }
   
    // Setup custom collision pairs specified by user
    setupInternalCollisionPairs();

    // node_ = std::make_shared<rclcpp::Node>("pointcloud_sub_node");
}

void MiracleMAlgo::setupInternalCollisionPairs() {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    // Clear all existing collision pairs first
    geom_model_.collisionPairs.clear();
    
    int added_pairs = 0;
    
    // Add internal collision pairs
    for (const auto& pair : collision_pairs) {
        // Pinocchio might append "_0" or "_1" or "_2" if there are multiple meshes per link.
        // Also it uses the mesh name from the URDF which might not be exactly the link name.
        // Let's iterate over all geometry objects to find a match by substring instead.
        std::vector<pinocchio::GeomIndex> geoms1, geoms2;
        
        for (size_t i = 0; i < geom_model_.geometryObjects.size(); ++i) {
            const std::string& geom_name = geom_model_.geometryObjects[i].name;
            // check if geom_name starts with the link name + "_"
            if (geom_name.find(pair.first + "_") == 0 || geom_name == pair.first) {
                geoms1.push_back(i);
            }
            if (geom_name.find(pair.second + "_") == 0 || geom_name == pair.second) {
                geoms2.push_back(i);
            }
        }
        
        if (!geoms1.empty() && !geoms2.empty()) {
            for (auto id1 : geoms1) {
                for (auto id2 : geoms2) {
                    geom_model_.addCollisionPair(pinocchio::CollisionPair(id1, id2));
                    added_pairs++;
                }
            }
        } else {
            // std::cerr << "Warning: Geometry for collision pair (" << pair.first << ", " << pair.second << ") not found!" << std::endl;
        }
    }
    
    geom_data_ = std::make_shared<pinocchio::GeometryData>(geom_model_);
    RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"), "Internal collision pairs configured. Total active pairs: %d", added_pairs);
}


void MiracleMAlgo::disableCollisionPair(const std::string& geom1_name, const std::string& geom2_name) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    if(!geom_model_.existGeometryName(geom1_name) || !geom_model_.existGeometryName(geom2_name)) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "Geometry not found! geom1=%s geom2=%s", geom1_name.c_str(), geom2_name.c_str());
        return;
    }
    pinocchio::GeomIndex id1 = geom_model_.getGeometryId(geom1_name);
    pinocchio::GeomIndex id2 = geom_model_.getGeometryId(geom2_name);
    
    geom_model_.removeCollisionPair(pinocchio::CollisionPair(id1, id2));
    geom_data_ = std::make_shared<pinocchio::GeometryData>(geom_model_);
}

void MiracleMAlgo::addExternalCollisionBox(const std::string& name, 
                                              const Eigen::Vector3d& size, 
                                              const Eigen::Isometry3d& pose) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    // Prevent adding duplicates
    if (std::find(external_objects_.begin(), external_objects_.end(), name) != external_objects_.end()) {
        RCLCPP_WARN(rclcpp::get_logger("MiracleMAlgo"), "External object '%s' already exists.", name.c_str());
        return;
    }

    std::shared_ptr<coal::Box> box = std::make_shared<coal::Box>(size.x(), size.y(), size.z());
    pinocchio::SE3 placement(pose.rotation(), pose.translation());
    pinocchio::GeometryObject obj(name, 0, placement, box); 
    
    pinocchio::GeomIndex obj_id = geom_model_.addGeometryObject(obj);
    external_objects_.push_back(name);
    
    // Only add collision pairs related to this new object
    int added_pairs = 0;
    for (const auto& arm_link : arm_collision_detect) {
        for (size_t i = 0; i < geom_model_.geometryObjects.size(); ++i) {
            const std::string& geom_name = geom_model_.geometryObjects[i].name;
            if (geom_name.find(arm_link + "_") == 0 || geom_name == arm_link) {
                // Check if pair already exists
                bool exists = false;
                for (const auto& pair : geom_model_.collisionPairs) {
                    if ((pair.first == i && pair.second == obj_id) || 
                        (pair.first == obj_id && pair.second == i)) {
                        exists = true;
                        break;
                    }
                }
                if (!exists) {
                    geom_model_.addCollisionPair(pinocchio::CollisionPair(i, obj_id));
                    added_pairs++;
                }
            }
        }
    }
    
    geom_data_ = std::make_shared<pinocchio::GeometryData>(geom_model_);
    RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"), "Added external object '%s'. New collision pairs: %d. Total active pairs: %zu",
                name.c_str(), added_pairs, geom_model_.collisionPairs.size());
}

std::shared_ptr<coal::OcTree> MiracleMAlgo::buildOcTreeFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                                        double resolution,
                                                                        Eigen::Vector3d& out_center) {
  if (!cloud || cloud->empty())
    throw std::runtime_error("Input cloud is empty");
  if (!std::isfinite(resolution) || resolution <= 0.0)
    throw std::runtime_error("Octree resolution must be finite and positive");

  // 1. 计算 AABB 与中心
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double min_z = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  double max_z = std::numeric_limits<double>::lowest();
  for (const auto& p : cloud->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    min_x = std::min(min_x, (double)p.x); max_x = std::max(max_x, (double)p.x);
    min_y = std::min(min_y, (double)p.y); max_y = std::max(max_y, (double)p.y);
    min_z = std::min(min_z, (double)p.z); max_z = std::max(max_z, (double)p.z);
  }
  if (min_x > max_x)
    throw std::runtime_error("No valid points");

  const double center_x = (min_x + max_x) / 2.0;
  const double center_y = (min_y + max_y) / 2.0;
  const double center_z = (min_z + max_z) / 2.0;
  out_center << center_x, center_y, center_z;

  // 2. 构建 octomap::OcTree,插入点云(平移到以 center 为原点)
  auto octree = std::make_shared<octomap::OcTree>(resolution);
  // 默认 occupancy 阈值 0.5,可以直接用 true 强制占据
  for (const auto& p : cloud->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    octree->updateNode(
        octomap::point3d((float)(p.x - center_x),
                         (float)(p.y - center_y),
                         (float)(p.z - center_z)),
        true);
  }
  // 同步内部节点的占据状态(必须!)
  octree->updateInnerOccupancy();

  // 3. 统计占据叶节点数(诊断)
  size_t leaf_count = 0;
  for (auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
    if (it->getOccupancy() > octree->getOccupancyThres()) leaf_count++;
  }
  RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"),
              "[OcTree] resolution=%.4f, occupied leaves=%zu",
              resolution, leaf_count);
  if (leaf_count == 0)
    throw std::runtime_error("OcTree has 0 occupied leaves");

  // 4. 包装成 coal::OcTree
  //    coal::OcTree 内部持有 shared_ptr<const octomap::OcTree>,生命周期安全
  std::shared_ptr<const octomap::OcTree> const_octree = octree;
  auto coal_octree = std::make_shared<coal::OcTree>(const_octree);
  return coal_octree;
}

void MiracleMAlgo::addExternalCollisionPointcloud(const std::string& name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

  std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);

  // 1cm 体素,与 MoveIt 默认一致
  const double resolution = 0.01;

  try {
    // 1. 加载点云
    // auto cloud = loadPointCloud("/fusion/synced_robot_vision");
    RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"),
                "[PCD] loaded %zu points", cloud ? cloud->size() : 0U);

    // 2. 构建 coal::OcTree
    auto start_time_bvh = std::chrono::high_resolution_clock::now();
    Eigen::Vector3d cloud_center;
    auto coal_octree = buildOcTreeFromCloud(cloud, resolution, cloud_center);
    // Validate/build before replacing the previous scene. Failure must reach the caller.
    if (std::find(external_objects_.begin(), external_objects_.end(), name)
        != external_objects_.end()) {
      removeExternalCollisionObject(name);
    }
    RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"),
                "[OcTree] built, cloud_center=(%.3f,%.3f,%.3f)",
                cloud_center.x(), cloud_center.y(), cloud_center.z());

    auto end_time_bvh = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_ms_bvh = end_time_bvh - start_time_bvh;
    std::cout<<"改进体素化+bvh耗时:"<<elapsed_ms_bvh.count()<<"ms"<<std::endl;

    // 4. placement = pose (OcTree 内部坐标已以原点为中心)
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = cloud_center;
    Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0); 
    pose.linear() = q.toRotationMatrix(); 
    pinocchio::SE3 placement(pose.rotation(), pose.translation());

    // 5. 加到 GeometryModel
    pinocchio::GeometryObject obj(name, 0, placement, coal_octree);
    pinocchio::GeomIndex obj_id = geom_model_.addGeometryObject(obj);
    external_objects_.push_back(name);

    // 6. 加碰撞对(逻辑保持不变)
    int added_pairs = 0;
    for (const auto& arm_link : arm_collision_detect) {
      for (size_t i = 0; i < geom_model_.geometryObjects.size(); ++i) {
        const std::string& geom_name = geom_model_.geometryObjects[i].name;
        if (geom_name.find(arm_link + "_") == 0 || geom_name == arm_link) {
          bool exists = false;
          for (const auto& pair : geom_model_.collisionPairs) {
            if ((pair.first == i && pair.second == obj_id) ||
                (pair.first == obj_id && pair.second == i)) {
              exists = true; break;
            }
          }
          if (!exists) {
            geom_model_.addCollisionPair(pinocchio::CollisionPair(i, obj_id));
            added_pairs++;
          }
        }
      }
    }

    geom_data_ = std::make_shared<pinocchio::GeometryData>(geom_model_);

    RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"),
                "Added external object '%s' as OcTree. "
                "New pairs: %d. Total: %zu",
                name.c_str(), added_pairs, geom_model_.collisionPairs.size());

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"),
                 "Failed to add pointcloud collision '%s': %s",
                 name.c_str(), e.what());
    throw;
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MiracleMAlgo::pointCloudToModelRoot(
    const sensor_msgs::msg::PointCloud2& message,
    const std::vector<double>& full_q)
{
    if (message.header.frame_id.empty() || message.width == 0U ||
        message.height == 0U || message.point_step == 0U ||
        message.is_bigendian ||
        static_cast<std::uint64_t>(message.row_step) <
            static_cast<std::uint64_t>(message.width) * message.point_step ||
        message.data.size() < static_cast<std::uint64_t>(message.row_step) * message.height) {
        throw std::runtime_error("Invalid point cloud frame, dimensions or byte layout");
    }
    for (const char* coordinate : {"x", "y", "z"}) {
        const auto field = std::find_if(message.fields.begin(), message.fields.end(),
            [coordinate](const auto& item) { return item.name == coordinate; });
        if (field == message.fields.end() || field->count != 1U ||
            field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
            static_cast<std::uint64_t>(field->offset) + sizeof(float) > message.point_step) {
            throw std::runtime_error("Point cloud requires valid float32 x/y/z fields");
        }
    }
    Eigen::Isometry3d root_from_cloud = Eigen::Isometry3d::Identity();
    if (!transformPoseToModelRoot(message.header.frame_id, full_q,
            Eigen::Isometry3d::Identity(), root_from_cloud)) {
        throw std::runtime_error("Unknown point cloud frame: " + message.header.frame_id);
    }
    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::fromROSMsg(message, input);
    auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    output->reserve(input.size());
    for (const auto& point : input) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
        const Eigen::Vector3d transformed = root_from_cloud * Eigen::Vector3d(point.x, point.y, point.z);
        if (!transformed.allFinite()) throw std::runtime_error("Nonfinite point cloud transform");
        output->push_back(pcl::PointXYZ(transformed.x(), transformed.y(), transformed.z()));
    }
    if (output->empty()) throw std::runtime_error("Point cloud contains no finite XYZ points");
    return output;
}

void MiracleMAlgo::removeExternalCollisionObject(const std::string& name) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    auto it = std::find(external_objects_.begin(), external_objects_.end(), name);
    if (it != external_objects_.end()) {
        external_objects_.erase(it);
        
        if (geom_model_.existGeometryName(name)) {
            size_t old_pairs_size = geom_model_.collisionPairs.size();
            
            // Remove from GeometryModel (this automatically updates collisionPairs and their indices!)
            geom_model_.removeGeometryObject(name);
            geom_data_ = std::make_shared<pinocchio::GeometryData>(geom_model_);
            
            size_t removed_pairs = old_pairs_size - geom_model_.collisionPairs.size();
            
            RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"), "Removed external collision object '%s'. Removed pairs: %zu. Total active pairs: %zu",
                        name.c_str(), removed_pairs, geom_model_.collisionPairs.size());
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("MiracleMAlgo"), "External collision object '%s' not found.", name.c_str());
    }
}

Eigen::VectorXd MiracleMAlgo::toEigenConfig(const std::vector<double>& full_q) const {
    Eigen::VectorXd q = default_q_;
    if (full_q.size() == static_cast<size_t>(model_.nq)) {
        for (int i = 0; i < model_.nq; ++i) {
            q[i] = full_q[static_cast<size_t>(i)];
        }
        return q;
    }
    RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "toEigenConfig: unsupported input size: %zu, expected %d", full_q.size(), model_.nq);
    return q;
}

bool MiracleMAlgo::isFullQValid(const Eigen::VectorXd& full_q) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    if (full_q.size() != model_.nq || !full_q.allFinite()) {
        std::cerr << "[isFullQValid] Unsupported input size: " << full_q.size()
                  << ". Expected " << model_.nq << std::endl;
        return false;
    }

    for (int i = 0; i < model_.nq; ++i) {
        if (full_q[i] < model_.lowerPositionLimit[i] || full_q[i] > model_.upperPositionLimit[i]) {
            std::cout << "[isFullQValid] Joint limit violated at q[" << i << "]: val=" << full_q[i]
                      << ", limits=[" << model_.lowerPositionLimit[i] << ", "
                      << model_.upperPositionLimit[i] << "]" << std::endl;
            return false;
        }
    }

    // The mobile base must not pitch beyond the configured 30 degree safety
    // limit.  Keep this check in the common full-state validator so that
    // sampled OMPL and time-parameterized states use the same constraint.
    // if (model_.existFrame("body_link")) {
    //     pinocchio::forwardKinematics(model_, *data_, full_q);
    //     pinocchio::updateFramePlacements(model_, *data_);
    //     const Eigen::Matrix3d& rotation =
    //         data_->oMf[model_.getFrameId("body_link")].rotation();
    //     const double pitch = std::atan2(
    //         -rotation(2, 0), std::hypot(rotation(0, 0), rotation(1, 0)));
    //     if (pitch > 30.0 * 0.017453292519943295) {   // 只关注正方向
    //         return false;
    //     }
    // }

    pinocchio::updateGeometryPlacements(model_, *data_, geom_model_, *geom_data_, full_q);
    if (pinocchio::computeCollisions(geom_model_, *geom_data_, false)) {
        std::ostringstream coll_pairs;
        for (size_t k = 0; k < geom_model_.collisionPairs.size(); ++k) {
            if (geom_data_->collisionResults[k].isCollision()) {
                const auto& pair = geom_model_.collisionPairs[k];
                coll_pairs << "  "
                           << geom_model_.geometryObjects[pair.first].name
                           << " <--> "
                           << geom_model_.geometryObjects[pair.second].name
                           << std::endl;
            }
        }
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"),
                     "[isFullQValid] Collision detected. Pairs:\n%s",
                     coll_pairs.str().c_str());
        return false;
    }

    return true;
}

std::shared_ptr<ompl::geometric::SimpleSetup> MiracleMAlgo::createPlannerSetup(
    const std::vector<size_t>& active_indices, 
    const Eigen::VectorXd& fixed_full_q) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    
    int dim = active_indices.size();
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(dim));
    ompl::base::RealVectorBounds bounds(dim);
    
    for(int i=0; i<dim; ++i) {
        // Find joint ID that corresponds to this q index
        for(size_t j = 1; j < model_.joints.size(); ++j) {
            if(model_.joints[j].idx_q() == (int)active_indices[i]) {
                bounds.setLow(i, model_.lowerPositionLimit[model_.joints[j].idx_q()]);
                bounds.setHigh(i, model_.upperPositionLimit[model_.joints[j].idx_q()]);
                // Debug bounds
                // std::cout << "Joint idx_q " << model_.joints[j].idx_q() << " limits: " << bounds.low[i] << " to " << bounds.high[i] << std::endl;
                break;
            }
        }
    }
    space->setBounds(bounds);

    auto ss = std::make_shared<ompl::geometric::SimpleSetup>(space);
    ss->setStateValidityChecker([this, active_indices, fixed_full_q](const ompl::base::State* state) { 
        return isStateValid(state, active_indices, fixed_full_q); 
    });
    
    auto planner(std::make_shared<ompl::geometric::RRTConnect>(ss->getSpaceInformation()));
    // Set a larger range to avoid OMPL complaining or skip valid checks for debugging
    // planner->setRange(0.5);
    ss->setPlanner(planner);
    
    return ss;
}

bool MiracleMAlgo::isStateValid(const ompl::base::State* state, 
                                   const std::vector<size_t>& active_indices, 
                                   const Eigen::VectorXd& fixed_full_q) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    const auto* rs = state->as<ompl::base::RealVectorStateSpace::StateType>();
    Eigen::VectorXd q = fixed_full_q;
    for(size_t i=0; i<active_indices.size(); ++i) {
        q[active_indices[i]] = rs->values[i];
    }
    
    pinocchio::updateGeometryPlacements(model_, *data_, geom_model_, *geom_data_, q);
    bool collision = pinocchio::computeCollisions(geom_model_, *geom_data_, true);
    
    // Clean up all debug prints from isStateValid and single arm planning
    if (collision) {
        // debug print
        for(size_t k = 0; k < geom_model_.collisionPairs.size(); ++k) {
            if(geom_data_->collisionResults[k].isCollision()) {
                RCLCPP_WARN(rclcpp::get_logger("MiracleMAlgo"),
                    "Collision during planning: %s <--> %s",
                    geom_model_.geometryObjects[geom_model_.collisionPairs[k].first].name.c_str(),
                    geom_model_.geometryObjects[geom_model_.collisionPairs[k].second].name.c_str());
            }
        }
        
    }
    
    // Check joint limits
    // Wait, the index passed to upperPositionLimit is wrong!
    // model_.lowerPositionLimit is indexed by idx_q(), not by the pinocchio joint ID!
    // Let's fix this in isStateValid:
    for(size_t i=0; i<active_indices.size(); ++i) {
        if (rs->values[i] < model_.lowerPositionLimit[active_indices[i]] || 
            rs->values[i] > model_.upperPositionLimit[active_indices[i]]) {
            RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "Joint limit violated for index %zu: val=%f, limits=[%f, %f]",
                         active_indices[i], rs->values[i],
                         model_.lowerPositionLimit[active_indices[i]],
                         model_.upperPositionLimit[active_indices[i]]);
            return false;
        }
    }

    // if (model_.existFrame("body_link")) {
    //     pinocchio::forwardKinematics(model_, *data_, q);
    //     pinocchio::updateFramePlacements(model_, *data_);
    //     const Eigen::Matrix3d& rotation =
    //         data_->oMf[model_.getFrameId("body_link")].rotation();
    //     const double pitch = std::atan2(
    //         -rotation(2, 0), std::hypot(rotation(0, 0), rotation(1, 0)));
    //     if (std::abs(pitch) > 30.0 * 0.017453292519943295) {
    //         return false;
    //     }
    // }
    
    return !collision;
}

// --- Dual Arm Implementations ---
bool MiracleMAlgo::planDualArmJointSpace(const std::vector<double>& start_q,
                                            const std::vector<double>& goal_q,
                                            std::vector<std::vector<double>>& path) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    if (start_q.size() != static_cast<size_t>(model_.nq)) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "planDualArmJointSpace: unsupported start_q size: %zu, expected %d", start_q.size(), model_.nq);
        return false;
    }
    if (goal_q.size() != 14) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "planDualArmJointSpace: unsupported goal_q size: %zu, expected 14", goal_q.size());
        return false;
    }
    std::vector<size_t> active_indices;
    active_indices.insert(active_indices.end(), left_arm_q_indices_.begin(), left_arm_q_indices_.end());
    active_indices.insert(active_indices.end(), right_arm_q_indices_.begin(), right_arm_q_indices_.end());
    
    Eigen::VectorXd fixed_full_q = toEigenConfig(start_q);
    auto ss = createPlannerSetup(active_indices, fixed_full_q);
    
    ompl::base::ScopedState<> start(ss->getStateSpace());
    ompl::base::ScopedState<> goal(ss->getStateSpace());
    for(int i=0; i<7; ++i) {
        start[i] = start_q[left_arm_q_indices_[static_cast<size_t>(i)]];
        start[7 + i] = start_q[right_arm_q_indices_[static_cast<size_t>(i)]];
        goal[i] = goal_q[static_cast<size_t>(i)];
        goal[7 + i] = goal_q[static_cast<size_t>(7 + i)];
    }
    // debug start state explicitly using our function
    RCLCPP_INFO_STREAM(rclcpp::get_logger("MiracleMAlgo"),
                       "Validating dual arm start state using isStateValid: "
                           << isStateValid(start.get(), active_indices, fixed_full_q));
              
    ss->setStartAndGoalStates(start, goal);
    
    if (ss->solve(5.0)) {
        ss->simplifySolution();
        ompl::geometric::PathGeometric& p = ss->getSolutionPath();
        if (p.getStateCount() > 2) {
            p.interpolate(p.getStateCount() * 10);
        }
        
        path.clear();
        for(std::size_t i = 0; i < p.getStateCount(); ++i) {
            const auto* state = p.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
            std::vector<double> q = start_q;
            for(int j=0; j<7; ++j) {
                q[left_arm_q_indices_[static_cast<size_t>(j)]] = state->values[j];
                q[right_arm_q_indices_[static_cast<size_t>(j)]] = state->values[7 + j];
            }
            path.push_back(q);
        }
        return true;
    }
    return false;
}

bool MiracleMAlgo::planWholeBodyJointSpace(
    const std::vector<double>& start_q,
    const std::vector<double>& goal_q,
    bool use_left,
    bool use_right,
    std::vector<std::vector<double>>& path)
{
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    path.clear();
    if (start_q.size() != static_cast<std::size_t>(model_.nq) ||
        goal_q.size() != static_cast<std::size_t>(model_.nq) ||
        (!use_left && !use_right) || leg_q_indices_.empty()) {
        RCLCPP_ERROR(
            rclcpp::get_logger("MiracleMAlgo"),
            "planWholeBodyJointSpace: invalid configuration size or active joints");
        return false;
    }

    std::vector<size_t> active_indices;
    if (use_left) {
        active_indices.insert(
            active_indices.end(), left_arm_q_indices_.begin(), left_arm_q_indices_.end());
    }
    if (use_right) {
        active_indices.insert(
            active_indices.end(), right_arm_q_indices_.begin(), right_arm_q_indices_.end());
    }
    active_indices.insert(
        active_indices.end(), leg_q_indices_.begin(), leg_q_indices_.end());

    const Eigen::VectorXd fixed_full_q = toEigenConfig(start_q);
    auto setup = createPlannerSetup(active_indices, fixed_full_q);
    ompl::base::ScopedState<> start(setup->getStateSpace());
    ompl::base::ScopedState<> target(setup->getStateSpace());
    for (std::size_t index = 0; index < active_indices.size(); ++index) {
        start[index] = start_q[active_indices[index]];
        target[index] = goal_q[active_indices[index]];
    }
    const bool start_valid = isStateValid(start.get(), active_indices, fixed_full_q);
    const bool target_valid = isStateValid(target.get(), active_indices, fixed_full_q);
    if (!start_valid || !target_valid) {
        const auto report_invalid_state = [&](const char* label,
                                               const ompl::base::State* state) {
            Eigen::VectorXd full_q = fixed_full_q;
            const auto* values = state->as<
                ompl::base::RealVectorStateSpace::StateType>();
            for (std::size_t index = 0; index < active_indices.size(); ++index) {
                full_q[active_indices[index]] = values->values[index];
            }
            pinocchio::updateGeometryPlacements(
                model_, *data_, geom_model_, *geom_data_, full_q);
            if (pinocchio::computeCollisions(geom_model_, *geom_data_, false)) {
                for (std::size_t pair_index = 0;
                     pair_index < geom_model_.collisionPairs.size(); ++pair_index) {
                    if (geom_data_->collisionResults[pair_index].isCollision()) {
                        const auto& pair = geom_model_.collisionPairs[pair_index];
                        RCLCPP_ERROR(
                            rclcpp::get_logger("MiracleMAlgo"),
                            "planWholeBodyJointSpace: %s collision: %s <--> %s",
                            label,
                            geom_model_.geometryObjects[pair.first].name.c_str(),
                            geom_model_.geometryObjects[pair.second].name.c_str());
                    }
                }
            } else {
                RCLCPP_ERROR(
                    rclcpp::get_logger("MiracleMAlgo"),
                    "planWholeBodyJointSpace: %s rejected by joint/body constraints",
                    label);
            }
        };
        if (!start_valid) {
            report_invalid_state("start", start.get());
        }
        if (!target_valid) {
            report_invalid_state("goal", target.get());
        }
        RCLCPP_ERROR(
            rclcpp::get_logger("MiracleMAlgo"),
            "planWholeBodyJointSpace: start or goal state is invalid");
        return false;
    }
    bool same_state = true;
    for (std::size_t index = 0; index < active_indices.size(); ++index) {
        if (std::abs(start[index] - target[index]) > 1e-10) {
            same_state = false;
            break;
        }
    }
    if (same_state) {
        path = {start_q, goal_q};
        return true;
    }
    setup->setStartAndGoalStates(start, target);
    if (!setup->solve(5.0)) {
        RCLCPP_ERROR(
            rclcpp::get_logger("MiracleMAlgo"),
            "planWholeBodyJointSpace: OMPL failed to find a path");
        return false;
    }

    setup->simplifySolution();
    ompl::geometric::PathGeometric& solution = setup->getSolutionPath();
    if (solution.getStateCount() > 2U) {
        solution.interpolate(solution.getStateCount() * 10U);
    }
    path.reserve(solution.getStateCount());
    for (std::size_t state_index = 0; state_index < solution.getStateCount(); ++state_index) {
        const auto* state = solution.getState(state_index)->as<
            ompl::base::RealVectorStateSpace::StateType>();
        std::vector<double> full_q = start_q;
        for (std::size_t joint_index = 0; joint_index < active_indices.size(); ++joint_index) {
            full_q[active_indices[joint_index]] = state->values[joint_index];
        }
        path.push_back(std::move(full_q));
    }
    return path.size() >= 2U;
}

// --- Single Arm Implementations ---
bool MiracleMAlgo::planSingleArmJointSpace(bool is_left,
                                              const std::vector<double>& start_q,
                                              const std::vector<double>& goal_q,
                                              std::vector<std::vector<double>>& path) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    if (start_q.size() != static_cast<size_t>(model_.nq)) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "planSingleArmJointSpace: unsupported start_q size: %zu, expected %d", start_q.size(), model_.nq);
        return false;
    }
    if (goal_q.size() != 7) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "planSingleArmJointSpace: unsupported goal_q size: %zu, expected 7", goal_q.size());
        return false;
    }
    const std::vector<size_t>& active_indices = is_left ? left_arm_q_indices_ : right_arm_q_indices_;
    // Also, we need to make sure the start_q that OMPL evaluates has the fixed arm set to [0, 0.5, ...]
    // wait, in isStateValid, we use fixed_full_q:
    //    Eigen::VectorXd q = fixed_full_q;
    //    for(size_t i=0; i<active_indices.size(); ++i) q[active_indices[i]] = rs->values[i];
    // If fixed_full_q is [0]*14, it might still collide!
    // So start_q must be expanded to a collision free full-body state.
    Eigen::VectorXd fixed_full_q = toEigenConfig(start_q);
    
    auto ss = createPlannerSetup(active_indices, fixed_full_q); //fixed_full_q开始状态全身关节角
    
    ompl::base::ScopedState<> start(ss->getStateSpace());
    ompl::base::ScopedState<> goal(ss->getStateSpace());
    
    // If user passed a 14D goal_q, extract the 7D part. If they passed 7D, use it directly.
    for(int i=0; i<7; ++i) {
        start[i] = start_q[active_indices[static_cast<size_t>(i)]];
        goal[i] = goal_q[static_cast<size_t>(i)];
    }
    
    // Validate the start state explicitly using our function
    if (!isStateValid(start.get(), active_indices, fixed_full_q)) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "Start state is explicitly invalid in our check!");
        return false;
    }
    
    ss->setStartAndGoalStates(start, goal);
    
    if (ss->solve(5.0)) {
        ss->simplifySolution();
        ompl::geometric::PathGeometric& p = ss->getSolutionPath();
        if (p.getStateCount() > 2) {
            p.interpolate(p.getStateCount() * 10);  //10倍插值
        }
        
        path.clear();
        for(std::size_t i = 0; i < p.getStateCount(); ++i) {
            const auto* state = p.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
            std::vector<double> current_dual_q = start_q; // keep inactive joints fixed
            for(int j=0; j<7; ++j) {
                current_dual_q[active_indices[static_cast<size_t>(j)]] = state->values[j];
            }
            path.push_back(current_dual_q);
        }
        // savePathToTxt(path, "/home/cxs/桌面/pose.txt");
        return true;
    }
    return false;
}

bool MiracleMAlgo::getForwardKinematics(const std::string& source_frame,
                                        const std::string& target_frame,
                                        Eigen::Matrix4d& T_source_target)
{
    // if (_m_full_joint_state_msg.size() != static_cast<size_t>(model_.nq)) {
    //     std::lock_guard<std::mutex> lock(upadate_joinst_state_msg);
    //     _m_full_joint_state_msg.assign(default_q_.data(), default_q_.data() + default_q_.size());
    // }

    Eigen::VectorXd q(model_.nq);
    {
        std::lock_guard<std::mutex> lock(upadate_joinst_state_msg);
        if (_m_full_joint_state_msg.size() !=
            static_cast<size_t>(model_.nq)) {
            RCLCPP_ERROR(
                rclcpp::get_logger("MiracleMAlgo"),
                "getForwardKinematics: full joint state size=%zu, expected=%d",
                _m_full_joint_state_msg.size(), model_.nq);
            return false;
        }
        q = Eigen::Map<const Eigen::VectorXd>(_m_full_joint_state_msg.data(), static_cast<long>(model_.nq));
    }

    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);

    if (!model_.existFrame(source_frame)) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "getForwardKinematics: source_frame not found: %s", source_frame.c_str());
        return false;
    }
    if (!model_.existFrame(target_frame)) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "getForwardKinematics: target_frame not found: %s", target_frame.c_str());
        return false;
    }

    const pinocchio::FrameIndex src_id = model_.getFrameId(source_frame);
    const pinocchio::FrameIndex tgt_id = model_.getFrameId(target_frame);

    pinocchio::forwardKinematics(model_, *data_, q);
    pinocchio::updateFramePlacements(model_, *data_);

    const pinocchio::SE3 oMs = data_->oMf[src_id];
    const pinocchio::SE3 oMt = data_->oMf[tgt_id];
    const pinocchio::SE3 sMt = oMs.actInv(oMt);

    T_source_target = sMt.toHomogeneousMatrix();
    return true;
}

bool MiracleMAlgo::getFramePoseAtConfiguration(
    const std::string& source_frame,
    const std::string& target_frame,
    const std::vector<double>& full_q,
    Eigen::Isometry3d& source_to_target)
{
    if (full_q.size() != static_cast<size_t>(model_.nq) ||
        !model_.existFrame(source_frame) || !model_.existFrame(target_frame)) {
        return false;
    }
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    const Eigen::VectorXd q = toEigenConfig(full_q);
    pinocchio::forwardKinematics(model_, *data_, q);
    pinocchio::updateFramePlacements(model_, *data_);
    const auto source_id = model_.getFrameId(source_frame);
    const auto target_id = model_.getFrameId(target_frame);
    source_to_target = Eigen::Isometry3d::Identity();
    source_to_target.matrix() =
        data_->oMf[source_id].actInv(data_->oMf[target_id]).toHomogeneousMatrix();
    return source_to_target.matrix().allFinite();
}



// --- Inverse Kinematics ---
bool MiracleMAlgo::solveIK(const Eigen::VectorXd& q_init,
                              const Eigen::Isometry3d& goal,
                              Eigen::VectorXd& q_res,
                              int mode) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    SingleArmIkWorkspace workspace;
    return solveIKWithLockedData(q_init, goal, q_res, mode, workspace);
}

bool MiracleMAlgo::solveIKWithLockedData(
    const Eigen::VectorXd& q_init,
    const Eigen::Isometry3d& goal,
    Eigen::VectorXd& q_res,
    int mode,
    SingleArmIkWorkspace& workspace)
{
    double eps = 5e-5;
    const int IT_MAX = 1000;
    const double DT = 0.5;
    const double damp = 1e-4;

    pinocchio::SE3 oMdes(goal.rotation(), goal.translation());

    // RCLCPP_INFO_STREAM(rclcpp::get_logger("MiracleMAlgo"),
    //     "[SolveIK] oMdes translation: " << oMdes.translation().transpose()
    //     << "\n[SolveIK] oMdes rotation:\n" << oMdes.rotation());

    pinocchio::FrameIndex ee_id = 0;
    const std::vector<size_t>* active_q = nullptr;
    const std::vector<int>* active_v = nullptr;
    const char* mode_name = "";
    if (mode == 0) {
        ee_id = left_ee_frame_id_;
        active_q = &left_arm_q_indices_;
        active_v = &left_arm_v_indices_;
        mode_name = "Left arm";
    } else if (mode == 1) {
        ee_id = right_ee_frame_id_;
        active_q = &right_arm_q_indices_;
        active_v = &right_arm_v_indices_;
        mode_name = "Right arm";
    } else if (mode == 3) {
        eps = 1e-3;
        ee_id = head_ee_frame_id_;
        active_v = &head_v_indices_;
        mode_name = "Head";
    } else if (mode == 4) {
        eps = 5e-4;
        ee_id = leg_ee_frame_id_;
        active_v = &leg_v_indices_;
        mode_name = "Leg";
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "solveIK: unsupported mode: %d", mode);
        return false;
    }

    std::vector<char> active_mask(static_cast<size_t>(model_.nv), 0);
    auto mark_active = [&active_mask, this](const std::vector<int>& idxs) {
        for(int idx : idxs) {
            if(idx >= 0 && idx < model_.nv) {
                active_mask[static_cast<size_t>(idx)] = 1;
            }
        }
    };
    mark_active(*active_v);

    q_res = q_init;
    if (active_q != nullptr &&
        !passesSimplifiedCcsJoint67Constraint(q_res, *active_q))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MiracleMAlgo"),
            "solveIK: %s initial state violates simplified CCS J6/J7 constraint",
            mode_name);
        return false;
    }
    workspace.jacobian.resize(6, model_.nv);
    workspace.error.resize(6);
    workspace.jjt.resize(6, 6);
    workspace.velocity.resize(model_.nv);
    workspace.damped_pseudo_inverse.resize(model_.nv, 6);
    workspace.nullspace_projector.resize(model_.nv, model_.nv);
    workspace.centering_velocity.resize(model_.nv);
    workspace.candidate.resize(model_.nq);
    workspace.identity6.setIdentity(6, 6);
    Eigen::MatrixXd& J_tmp = workspace.jacobian;

    bool success = false;
    size_t next_fallback_seed = 0U;

    // std::cout << "ee id: " << ee_id << std::endl;

    // std::vector<std::string> q_names(static_cast<size_t>(model_.nq), "<unnamed>");
    // for (pinocchio::JointIndex jid = 1; jid < model_.joints.size(); ++jid) {
    //     const int idx_q = model_.joints[jid].idx_q();
    //     const int nq = model_.joints[jid].nq();
    //     for (int k = 0; k < nq; ++k) {
    //         const int q_idx = idx_q + k;
    //         if (q_idx >= 0 && q_idx < model_.nq) {
    //             q_names[static_cast<size_t>(q_idx)] = model_.names[jid];
    //         }
    //     }
    // }
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("MiracleMAlgo"), "[SolveIK] q_res: "
    //     << [&]() {
    //         std::ostringstream oss;
    //         for (int j = 0; j < model_.nq; ++j) {
    //             if (j > 0) {
    //                 oss << '\n';
    //             }
    //             oss << q_names[static_cast<size_t>(j)] << "=" << q_res[j];
    //         }
    //         return oss.str();
    //     }());

    Eigen::VectorXd& err = workspace.error;
    for (int i=0; i<IT_MAX; i++) {
        // pinocchio::forwardKinematics(model_, *data_, q_res);
        // pinocchio::updateFramePlacements(model_, *data_);
        pinocchio::computeJointJacobians(model_, *data_, q_res);
        pinocchio::updateFramePlacements(model_, *data_);
        
        const pinocchio::SE3 iMd = data_->oMf[ee_id].actInv(oMdes);
        err = pinocchio::log6(iMd).toVector();
        //打印误差
        // RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"), "[SolveIK] err norm: %f", err.norm());

        if (err.norm() < eps &&
            (active_q == nullptr ||
            passesSimplifiedCcsJoint67Constraint(q_res, *active_q)))
        {
            success = true;
            RCLCPP_DEBUG(rclcpp::get_logger("MiracleMAlgo"), "solveIK: %s IK converged in %d iterations. Error norm: %f", mode_name, i, err.norm());
            break;
        }

        J_tmp.setZero();
        pinocchio::getFrameJacobian(model_, *data_, ee_id, pinocchio::LOCAL, J_tmp);
        // pinocchio::computeFrameJacobian(model_, *data_, q_res, ee_id, pinocchio::LOCAL, J_tmp);

        // Mask out non-active joints该掩码用于过滤掉非活动关节，只考虑活动关节的Jacobian
        for(int j=0; j<model_.nv; ++j) {
            if(!active_mask[static_cast<size_t>(j)]) {
                J_tmp.col(j).setZero();
            }
        }

        Eigen::MatrixXd& JJt = workspace.jjt;
        JJt.noalias() = J_tmp * J_tmp.transpose();
        JJt.diagonal().array() += damp;

        const auto jj_solver = JJt.ldlt();
        Eigen::VectorXd& v = workspace.velocity;
        v.noalias() = J_tmp.transpose() * jj_solver.solve(err);
        if (active_q != nullptr) {
            // Keep the Cartesian task primary while using arm redundancy to
            // bias J6/J7 toward the center of the simplified CCS envelope.
            Eigen::MatrixXd& damped_pseudo_inverse =
                workspace.damped_pseudo_inverse;
            damped_pseudo_inverse.noalias() =
                J_tmp.transpose() * jj_solver.solve(workspace.identity6);
            Eigen::MatrixXd& nullspace_projector =
                workspace.nullspace_projector;
            nullspace_projector.setIdentity();
            nullspace_projector.noalias() -= damped_pseudo_inverse * J_tmp;
            Eigen::VectorXd& centering_velocity =
                workspace.centering_velocity;
            centering_velocity.setZero();
            addCcsJoint67CenteringVelocity(
                q_res, *active_q, *active_v, centering_velocity);
            v += nullspace_projector * centering_velocity;

            const double current_error_norm = err.norm();
            bool step_accepted = false;
            // Project every trial into the hard CCS envelope and only accept
            // it after recomputing FK and confirming that pose error falls.
            for (int attempt = 0;
                attempt < kIkLineSearchAttempts; ++attempt)
            {
                const double step_scale = std::ldexp(1.0, -attempt);
                Eigen::VectorXd& candidate = workspace.candidate;
                candidate = pinocchio::integrate(
                    model_, q_res, v * (DT * step_scale));
                for (int j = 0; j < model_.nq; ++j) {
                    candidate[j] = std::clamp(
                        candidate[j], model_.lowerPositionLimit[j],
                        model_.upperPositionLimit[j]);
                }
                if (!projectToSimplifiedCcsJoint67Constraint(
                    candidate, *active_q) ||
                    !passesSimplifiedCcsJoint67Constraint(
                    candidate, *active_q))
                {
                    continue;
                }

                pinocchio::forwardKinematics(model_, *data_, candidate);
                pinocchio::updateFramePlacements(model_, *data_);
                const pinocchio::SE3 candidate_iMd =
                    data_->oMf[ee_id].actInv(oMdes);
                const double candidate_error_norm =
                    pinocchio::log6(candidate_iMd).toVector().norm();
                if (std::isfinite(candidate_error_norm) &&
                    candidate_error_norm < current_error_norm)
                {
                    q_res = candidate;
                    step_accepted = true;
                    break;
                }
            }
            if (!step_accepted) {
                bool reseeded = false;
                while (next_fallback_seed <
                    kSingleArmFallbackJoint6SeedsDeg.size() && !reseeded)
                {
                    q_res = q_init;
                    const double seed_joint6_deg =
                        kSingleArmFallbackJoint6SeedsDeg[next_fallback_seed++];
                    reseeded = setSimplifiedCcsJoint67Seed(
                        q_res, *active_q, seed_joint6_deg);
                    for (int j = 0; reseeded && j < model_.nq; ++j) {
                        q_res[j] = std::clamp(
                            q_res[j], model_.lowerPositionLimit[j],
                            model_.upperPositionLimit[j]);
                    }
                    reseeded = reseeded &&
                        projectToSimplifiedCcsJoint67Constraint(
                        q_res, *active_q) &&
                        passesSimplifiedCcsJoint67Constraint(
                        q_res, *active_q);
                    if (reseeded) {
                        RCLCPP_WARN(
                            rclcpp::get_logger("MiracleMAlgo"),
                            "solveIK: %s retrying with J6 seed %.1f deg and J7 seed 0 deg",
                            mode_name, seed_joint6_deg);
                    }
                }
                if (reseeded) {
                    continue;
                }
                RCLCPP_ERROR(
                    rclcpp::get_logger("MiracleMAlgo"),
                    "solveIK: %s could not find a CCS-valid descending step at iteration %d",
                    mode_name, i);
                return false;
            }
        } else {
            q_res = pinocchio::integrate(model_, q_res, v * DT);
            // 头部和下肢维持原有独立关节限位处理。
            for (int j = 0; j < model_.nq; ++j) {
                q_res[j] = std::clamp(
                    q_res[j], model_.lowerPositionLimit[j],
                    model_.upperPositionLimit[j]);
            }
        }
    }
    
    if (!success) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "solveIK: failed to converge after %d iterations. Error norm: %f", IT_MAX, err.norm());
    }
    return success;
}

bool MiracleMAlgo::solveDualIK(const Eigen::VectorXd& q_init,
                                  const Eigen::Isometry3d& left_goal,
                                  const Eigen::Isometry3d& right_goal,
                                  Eigen::VectorXd& q_res) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    const double eps = 5e-5;
    const int IT_MAX = 1000;
    const double DT = 0.5;
    const double damp = 1e-4;

    pinocchio::SE3 oMdes_left(left_goal.rotation(), left_goal.translation());
    pinocchio::SE3 oMdes_right(right_goal.rotation(), right_goal.translation());

    std::vector<char> active_mask(static_cast<size_t>(model_.nv), 0);
    auto mark_active = [&active_mask, this](const std::vector<int>& idxs) {
        for(int idx : idxs) {
            if(idx >= 0 && idx < model_.nv) {
                active_mask[static_cast<size_t>(idx)] = 1;
            }
        }
    };
    mark_active(left_arm_v_indices_);
    mark_active(right_arm_v_indices_);

    q_res = q_init;
    if (!passesSimplifiedCcsJoint67Constraint(
        q_res, left_arm_q_indices_) ||
        !passesSimplifiedCcsJoint67Constraint(
        q_res, right_arm_q_indices_))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MiracleMAlgo"),
            "solveDualIK: initial state violates simplified CCS J6/J7 constraint");
        return false;
    }
    Eigen::MatrixXd J_left(6, model_.nv);
    Eigen::MatrixXd J_right(6, model_.nv);

    bool success = false;
    size_t next_fallback_seed = 0U;
    Eigen::VectorXd err_left(6);
    Eigen::VectorXd err_right(6);
    for (int i=0; i<IT_MAX; i++) {
        // pinocchio::forwardKinematics(model_, *data_, q_res);
        pinocchio::computeJointJacobians(model_, *data_, q_res);
        pinocchio::updateFramePlacements(model_, *data_);

        const pinocchio::SE3 iMd_left = data_->oMf[left_ee_frame_id_].actInv(oMdes_left);
        const pinocchio::SE3 iMd_right = data_->oMf[right_ee_frame_id_].actInv(oMdes_right);
        err_left = pinocchio::log6(iMd_left).toVector();
        err_right = pinocchio::log6(iMd_right).toVector();
        if (err_left.norm() < eps && err_right.norm() < eps &&
            passesSimplifiedCcsJoint67Constraint(
            q_res, left_arm_q_indices_) &&
            passesSimplifiedCcsJoint67Constraint(
            q_res, right_arm_q_indices_))
        {
            success = true;
            RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"), "solveDualIK: converged in %d iterations. Left err: %f, Right err: %f", i, err_left.norm(), err_right.norm());
            break;
        }

        J_left.setZero();
        J_right.setZero();
        pinocchio::getFrameJacobian(model_, *data_, left_ee_frame_id_, pinocchio::LOCAL, J_left);
        pinocchio::getFrameJacobian(model_, *data_, right_ee_frame_id_, pinocchio::LOCAL, J_right);
        // pinocchio::computeFrameJacobian(model_, *data_, q_res, left_ee_frame_id_, pinocchio::LOCAL, J_left);
        // pinocchio::computeFrameJacobian(model_, *data_, q_res, right_ee_frame_id_, pinocchio::LOCAL, J_right);

        Eigen::MatrixXd J(12, model_.nv);
        J << J_left, J_right;
        Eigen::VectorXd err(12);
        err << err_left, err_right;

        // Mask out non-active joints该掩码用于过滤掉非活动关节，只考虑活动关节的Jacobian
        for(int j=0; j<model_.nv; ++j) {
            if(!active_mask[static_cast<size_t>(j)]) {
                J.col(j).setZero();
            }
        }

        Eigen::MatrixXd JJt = J * J.transpose();
        JJt.diagonal().array() += damp;

        const auto jj_solver = JJt.ldlt();
        Eigen::VectorXd v = J.transpose() * jj_solver.solve(err);
        // Each arm contributes one redundant direction; center both wrist
        // pairs without changing the primary dual Cartesian task.
        const Eigen::MatrixXd damped_pseudo_inverse =
            J.transpose() * jj_solver.solve(
            Eigen::MatrixXd::Identity(12, 12));
        const Eigen::MatrixXd nullspace_projector =
            Eigen::MatrixXd::Identity(model_.nv, model_.nv) -
            damped_pseudo_inverse * J;
        Eigen::VectorXd centering_velocity =
            Eigen::VectorXd::Zero(model_.nv);
        addCcsJoint67CenteringVelocity(
            q_res, left_arm_q_indices_, left_arm_v_indices_,
            centering_velocity);
        addCcsJoint67CenteringVelocity(
            q_res, right_arm_q_indices_, right_arm_v_indices_,
            centering_velocity);
        v += nullspace_projector * centering_velocity;

        const double current_error_norm = err.norm();
        bool step_accepted = false;
        // A dual-arm trial is accepted only if both CCS envelopes hold and
        // the combined left/right Cartesian error decreases.
        for (int attempt = 0;
            attempt < kIkLineSearchAttempts; ++attempt)
        {
            const double step_scale = std::ldexp(1.0, -attempt);
            Eigen::VectorXd candidate = pinocchio::integrate(
                model_, q_res, v * (DT * step_scale));
            for (int j = 0; j < model_.nq; ++j) {
                candidate[j] = std::clamp(
                    candidate[j], model_.lowerPositionLimit[j],
                    model_.upperPositionLimit[j]);
            }
            if (!projectToSimplifiedCcsJoint67Constraint(
                candidate, left_arm_q_indices_) ||
                !projectToSimplifiedCcsJoint67Constraint(
                candidate, right_arm_q_indices_) ||
                !passesSimplifiedCcsJoint67Constraint(
                candidate, left_arm_q_indices_) ||
                !passesSimplifiedCcsJoint67Constraint(
                candidate, right_arm_q_indices_))
            {
                continue;
            }

            pinocchio::forwardKinematics(model_, *data_, candidate);
            pinocchio::updateFramePlacements(model_, *data_);
            const Eigen::VectorXd candidate_err_left = pinocchio::log6(
                data_->oMf[left_ee_frame_id_].actInv(oMdes_left)).toVector();
            const Eigen::VectorXd candidate_err_right = pinocchio::log6(
                data_->oMf[right_ee_frame_id_].actInv(oMdes_right)).toVector();
            const double candidate_error_norm = std::sqrt(
                candidate_err_left.squaredNorm() +
                candidate_err_right.squaredNorm());
            if (std::isfinite(candidate_error_norm) &&
                candidate_error_norm < current_error_norm)
            {
                q_res = std::move(candidate);
                step_accepted = true;
                break;
            }
        }
        if (!step_accepted) {
            bool reseeded = false;
            while (next_fallback_seed <
                kDualArmFallbackJoint6SeedsDeg.size() && !reseeded)
            {
                q_res = q_init;
                const auto& seed =
                    kDualArmFallbackJoint6SeedsDeg[next_fallback_seed++];
                reseeded = setSimplifiedCcsJoint67Seed(
                    q_res, left_arm_q_indices_, seed[0]) &&
                    setSimplifiedCcsJoint67Seed(
                    q_res, right_arm_q_indices_, seed[1]);
                for (int j = 0; reseeded && j < model_.nq; ++j) {
                    q_res[j] = std::clamp(
                        q_res[j], model_.lowerPositionLimit[j],
                        model_.upperPositionLimit[j]);
                }
                reseeded = reseeded &&
                    projectToSimplifiedCcsJoint67Constraint(
                    q_res, left_arm_q_indices_) &&
                    projectToSimplifiedCcsJoint67Constraint(
                    q_res, right_arm_q_indices_) &&
                    passesSimplifiedCcsJoint67Constraint(
                    q_res, left_arm_q_indices_) &&
                    passesSimplifiedCcsJoint67Constraint(
                    q_res, right_arm_q_indices_);
                if (reseeded) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("MiracleMAlgo"),
                        "solveDualIK: retrying with left/right J6 seeds %.1f/%.1f deg and J7 seeds 0 deg",
                        seed[0], seed[1]);
                }
            }
            if (reseeded) {
                continue;
            }
            RCLCPP_ERROR(
                rclcpp::get_logger("MiracleMAlgo"),
                "solveDualIK: could not find a CCS-valid descending step at iteration %d",
                i);
            return false;
        }
    }

    if (!success) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "solveDualIK: failed to converge after %d iterations. Left err: %f, Right err: %f", IT_MAX, err_left.norm(), err_right.norm());
    }
    //打印出ik结果，横置打印,只打印左右臂的关节角度
    RCLCPP_INFO_STREAM(rclcpp::get_logger("MiracleMAlgo"), "[SolveIK] Left arm IK result: " << q_res(left_arm_q_indices_).transpose());
    //std::cout << "[SolveIK] Right arm IK result: " << q_res(right_arm_q_indices_).transpose() << std::endl;
    return success;
}

// =============================================================================
// 全身运动学逆解：基于速度级二次规划(QP)的方法
// -----------------------------------------------------------------------------
// 数学模型(每步迭代求解一次 QP)：
//   min  0.5 * dq' H dq + dq' g
//   s.t. lb <= dq <= ub               (a) 关节位置限位约束 -> 位移边界
//        lbA <= A dq <= ubA           (b) 自碰撞避免约束(最近点距离雅可比)
// 其中：
//   H = 2 (J'J + diag(W)) ,  g = -2 J' e      (任务跟踪 + 姿态正则)
//   J 为活动关节(被请求手臂 + 腿部)到末端速度的雅可比，e 为 log6 位姿误差
//   W 为姿态正则权重：手臂小(自由运动)、腿部大(保持在 q_init 附近)
// 单臂模式仅激活对应手臂 + 腿部；双臂模式激活双臂 + 腿部。
// 采用 qpOASES 求解；并以回溯线搜索保证关节限位/自碰撞不被违反(线性化安全网)。
// =============================================================================
bool MiracleMAlgo::solveWholeBodyIKOnce(
    const Eigen::VectorXd& q_init,
    const std::optional<Eigen::Isometry3d>& left_goal,
    const std::optional<Eigen::Isometry3d>& right_goal,
    Eigen::VectorXd& q_res,
    const std::array<double, 4>& leg_weights,
    double body_pitch_limit_rad,
    std::chrono::steady_clock::time_point deadline,
    bool enable_collision_avoidance)
{
    // pinocchio 工作区(Data/GeometryData)非线程安全，且与 FK/碰撞回调共享，
    // 必须通过可重入锁串行访问。
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);

    // --- 输入校验 ---
    if (q_init.size() != model_.nq) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"),
                     "solveWholeBodyIKOnce: q_init size=%d, expected=%d",
                     static_cast<int>(q_init.size()), model_.nq);
        return false;
    }
    if (!left_goal && !right_goal) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"),
                     "solveWholeBodyIKOnce: 至少需提供一个手臂目标位姿");
        return false;
    }
    if (left_goal && !left_goal->matrix().allFinite()) return false;
    if (right_goal && !right_goal->matrix().allFinite()) return false;

    // --- 构造活动关节速度索引表 ---
    // 活动关节 = 被请求的末端手臂(每臂 7) + 腿部/躯干(4)。
    // 腿部参与以保证手臂可达性并通过加权最小二乘保持在 q_init 附近；头部不参与。
    std::vector<int> active_v;
    active_v.reserve(left_arm_v_indices_.size() + right_arm_v_indices_.size() +
                     leg_v_indices_.size());
    auto push_unique = [&](const std::vector<int>& idxs) {
        for (int idx : idxs) {
            if (std::find(active_v.begin(), active_v.end(), idx) == active_v.end())
                active_v.push_back(idx);
        }
    };
    if (left_goal)  push_unique(left_arm_v_indices_);
    if (right_goal) push_unique(right_arm_v_indices_);
    push_unique(leg_v_indices_);
    const int n_active = static_cast<int>(active_v.size());
    if (n_active <= 0) return false;

    // 末端目标列表(单臂或双臂)
    struct EEGoal { pinocchio::FrameIndex frame_id; pinocchio::SE3 oMdes; };
    std::vector<EEGoal> goals;
    if (left_goal)
        goals.push_back({left_ee_frame_id_,
                         pinocchio::SE3(left_goal->rotation(), left_goal->translation())});
    if (right_goal)
        goals.push_back({right_ee_frame_id_,
                         pinocchio::SE3(right_goal->rotation(), right_goal->translation())});
    const int n_ee = static_cast<int>(goals.size());
    const int n_task = 6 * n_ee;  // 任务空间维度

    // --- 活动速度索引 -> 关节(q)索引及位置限位的映射 ---
    std::vector<int> active_q(n_active);
    Eigen::VectorXd lower_pos(n_active), upper_pos(n_active);
    for (int j = 0; j < n_active; ++j) {
        const int v_idx = active_v[j];
        int q_idx = v_idx;  // 单自由度关节 idx_q == idx_v，作为回退
        for (pinocchio::JointIndex jid = 1; jid < model_.joints.size(); ++jid) {
            if (model_.joints[jid].idx_v() == v_idx) {
                q_idx = model_.joints[jid].idx_q();
                break;
            }
        }
        active_q[j] = q_idx;
        lower_pos[j] = model_.lowerPositionLimit[q_idx];
        upper_pos[j] = model_.upperPositionLimit[q_idx];
    }

    // --- 姿态正则权重(对角) ---
    // 手臂小权重(自由运动)、腿部按 leg_weights(大权重,保持原位)。
    const double arm_reg = 5.0e-2;  // 用户指定: 0.05 (5:1 leg:arm 比值)
    Eigen::VectorXd W = Eigen::VectorXd::Zero(n_active);
    for (int j = 0; j < n_active; ++j) {
        bool is_leg = false;
        for (size_t k = 0; k < leg_v_indices_.size(); ++k) {
            if (leg_v_indices_[k] == active_v[j]) {
                W[j] = std::max(1.0e-6, leg_weights[k]);
                is_leg = true;
                break;
            }
        }
        if (!is_leg) W[j] = arm_reg;
    }

    // body_joint(躯干俯仰)索引(leg_v_indices_ 末项按构造顺序为 body_joint)
    int body_q_idx = -1;
    if (!leg_v_indices_.empty()) {
        const int body_v_idx = leg_v_indices_.back();
        for (int j = 0; j < n_active; ++j) {
            if (active_v[j] == body_v_idx) { body_q_idx = active_q[j]; break; }
        }
    }

    // --- 迭代参数 ---
    const double eps = 1.4e-4;     // 收敛阈值(6D误差范数: 均匀分布时 ~0.1mm + ~0.008°)
    const int IT_MAX = 600;          // 最大迭代次数
    // 自适应步长: 接近目标时误差小、二阶非线性占比大，大步长会过冲形成极限环
    // (表现为 err 在 mm 级震荡不收敛)，按误差分级收紧步长。
    auto step_limit = [](double err_norm) {
        if (err_norm < 0.01) return 0.01;
        if (err_norm < 0.05) return 0.05;
        return 0.30;
    };
    const double safety_margin = 0.01;  // 关节限位安全余量(rad)
    const double d_safe = 0.04;      // 自碰撞安全距离阈值(m)
    const double coll_gain = 1.0;   // 自碰撞约束增益(=1:线性化下不越过安全距离)

    // 反对称矩阵：skew(r) * w == r x w
    auto skew = [](const Eigen::Vector3d& r) {
        Eigen::Matrix3d s;
        s << 0.0, -r.z(), r.y(),
             r.z(), 0.0, -r.x(),
             -r.y(), r.x(), 0.0;
        return s;
    };

    q_res = q_init;
    Eigen::VectorXd q = q_init;
    bool success = false;

    Eigen::MatrixXd J_full(n_task, n_active);    // 任务雅可比(活动列)
    Eigen::MatrixXd J_ee(6, model_.nv);           // 末端帧全雅可比(LOCAL)
    Eigen::MatrixXd J_joint(6, model_.nv);       // 关节全雅可比(WORLD,用于最近点)
    Eigen::VectorXd err(n_task);

    for (int iter = 0; iter < IT_MAX; ++iter) {
        if (std::chrono::steady_clock::now() > deadline) {
            RCLCPP_WARN(rclcpp::get_logger("MiracleMAlgo"),
                        "solveWholeBodyIKOnce: 迭代超时(iter=%d)", iter);
            break;
        }

        // 1) 运动学与雅可比(computeJointJacobians 内部已执行 FK)
        pinocchio::computeJointJacobians(model_, *data_, q);
        pinocchio::updateFramePlacements(model_, *data_);

        // 2) 任务误差与雅可比(LOCAL 帧，与 log6 误差一致)
        for (int e = 0; e < n_ee; ++e) {
            const pinocchio::SE3 iMd = data_->oMf[goals[e].frame_id].actInv(goals[e].oMdes);
            err.segment<6>(6 * e) = pinocchio::log6(iMd).toVector();
            J_ee.setZero();
            pinocchio::getFrameJacobian(model_, *data_, goals[e].frame_id,
                                        pinocchio::LOCAL, J_ee);
            // 抽取活动速度列，组成活动雅可比
            for (int j = 0; j < n_active; ++j)
                J_full.block(6 * e, j, 6, 1) = J_ee.col(active_v[j]);
        }

        // if (iter == 0) {
        //     std::ostringstream qdiag;
        //     for (size_t k = 0; k < leg_v_indices_.size(); ++k) {
        //         const int v = leg_v_indices_[k];
        //         int qi = v;
        //         for (pinocchio::JointIndex jid = 1; jid < model_.joints.size(); ++jid)
        //             if (model_.joints[jid].idx_v() == v) { qi = model_.joints[jid].idx_q(); break; }
        //         qdiag << " L" << k << "=" << q[qi];
        //     }
        //     for (size_t k = 0; k < left_arm_v_indices_.size() && (bool)left_goal; ++k) {
        //         const int v = left_arm_v_indices_[k];
        //         int qi = v;
        //         for (pinocchio::JointIndex jid = 1; jid < model_.joints.size(); ++jid)
        //             if (model_.joints[jid].idx_v() == v) { qi = model_.joints[jid].idx_q(); break; }
        //         qdiag << " A" << k << "=" << q[qi];
        //     }
        //     // 直接打印 q.head(min(18,nq)) 原始值，验证映射
        //     std::ostringstream qraw;
        //     const int raw_n = std::min(18, (int)q.size());
        //     for (int i = 0; i < raw_n; ++i) qraw << " q[" << i << "]=" << q[i];
        //     qraw << " sum=" << q.sum();
        //     RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"),
        //                 "solveWholeBodyIKOnce: iter=0 err_norm=%.6f (eps=%.3f), mapped_q: %s",
        //                 err.norm(), eps, qdiag.str().c_str());
        //     RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"),
        //                 "solveWholeBodyIKOnce: iter=0 raw_q: %s; nq=%d nv=%d",
        //                 qraw.str().c_str(), (int)model_.nq, (int)model_.nv);
        //     // 打印 base_link 在 universe 系的 placement，验证 FK 原点
        //     if (model_.existFrame("base_link")) {
        //         const pinocchio::SE3& b = data_->oMf[model_.getFrameId("base_link")];
        //         RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"),
        //                     "  base_link in universe: p=[%.5f,%.5f,%.5f]; q=[%.5f,%.5f,%.5f,%.5f]",
        //                     b.translation().x(), b.translation().y(), b.translation().z(),
        //                     Eigen::Quaterniond(b.rotation()).x(),
        //                     Eigen::Quaterniond(b.rotation()).y(),
        //                     Eigen::Quaterniond(b.rotation()).z(),
        //                     Eigen::Quaterniond(b.rotation()).w());
        //     }
        //     // 打印前 5 个 frame 名（看 model frames 顺序）
        //     {
        //         std::ostringstream os;
        //         const int nf = std::min(8, (int)model_.frames.size());
        //         for (int f = 0; f < nf; ++f)
        //             os << " f" << f << "=" << model_.frames[f].name
        //                << "(pj=" << model_.frames[f].parentJoint << ")";
        //         RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"),
        //                     "  frames[0..%d]: %s", nf-1, os.str().c_str());
        //     }
        //     RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"),
        //                 "solveWholeBodyIKOnce: iter=0 seed_err=%.6f eps=%.3f; n_ee=%d n_active=%d",
        //                 err.norm(), eps, n_ee, n_active);
        //     for (int e = 0; e < n_ee; ++e) {
        //         const Eigen::Vector3d p_actual = data_->oMf[goals[e].frame_id].translation();
        //         const Eigen::Vector3d p_target = goals[e].oMdes.translation();
        //         const Eigen::Quaterniond q_actual(data_->oMf[goals[e].frame_id].rotation());
        //         const Eigen::Quaterniond q_target(goals[e].oMdes.rotation());
        //         RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"),
        //                     "  EE%d: actual_p=[%.5f,%.5f,%.5f] target_p=[%.5f,%.5f,%.5f]; actual_q=[%.5f,%.5f,%.5f,%.5f] target_q=[%.5f,%.5f,%.5f,%.5f]",
        //                     e, p_actual.x(), p_actual.y(), p_actual.z(),
        //                     p_target.x(), p_target.y(), p_target.z(),
        //                     q_actual.x(), q_actual.y(), q_actual.z(), q_actual.w(),
        //                     q_target.x(), q_target.y(), q_target.z(), q_target.w());
        //     }
        // }

        if (iter % 100 == 0 && iter > 0) {
            RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"),
                        "solveWholeBodyIKOnce: iter=%d err_norm=%.6f",
                        iter, err.norm());
        }

        if (err.norm() < eps) { success = true; break; }

        // 3) 更新几何体位姿并计算自碰撞距离(若启用)
        //    仅对距离 < d_safe 的碰撞对构造不等式约束，保证 QP 规模精简。
        std::vector<Eigen::RowVectorXd> A_rows;  // 每行 1 x n_active
        std::vector<double> lbA_rows, ubA_rows;
        if (enable_collision_avoidance) {
            pinocchio::updateGeometryPlacements(model_, *data_, geom_model_,
                                                *geom_data_, q);
            pinocchio::computeDistances(geom_model_, *geom_data_);
            const auto& pairs = geom_model_.collisionPairs;
            for (size_t k = 0; k < pairs.size(); ++k) {
                const auto& dr = geom_data_->distanceResults[k];
                if (!std::isfinite(dr.min_distance)) continue;
                const double d = dr.min_distance;
                if (d >= d_safe) continue;  // 距离足够远，无需约束

                const Eigen::Vector3d& p1 = dr.nearest_points[0];  // 属于 obj0
                const Eigen::Vector3d& p2 = dr.nearest_points[1];  // 属于 obj1
                // coal 约定：normal 由 o1 指向 o2，即 (p2 - p1)/d
                Eigen::Vector3d nrm = dr.normal;
                if (!nrm.allFinite() || nrm.norm() < 1.0e-9) {
                    nrm = p2 - p1;
                    if (nrm.norm() < 1.0e-9) continue;
                }
                nrm.normalize();
                if (!nrm.allFinite()) continue;

                const auto& cp = pairs[k];
                const auto& go0 = geom_model_.geometryObjects[cp.first];
                const auto& go1 = geom_model_.geometryObjects[cp.second];
                const pinocchio::JointIndex j0 = go0.parentJoint;
                const pinocchio::JointIndex j1 = go1.parentJoint;

                // 最近点线速度雅可比(世界系)：
                //   v_p = v_origin + w x (p - origin) = (Jv - skew(r) Jw) dq
                // (pinocchio Jacobian 行序：topRows(3)=LINEAR, bottomRows(3)=ANGULAR)
                Eigen::MatrixXd Jp0 = Eigen::MatrixXd::Zero(3, model_.nv);
                Eigen::MatrixXd Jp1 = Eigen::MatrixXd::Zero(3, model_.nv);
                if (j0 > 0 && j0 < model_.joints.size()) {
                    J_joint.setZero();
                    pinocchio::getJointJacobian(model_, *data_, j0,
                                                pinocchio::WORLD, J_joint);
                    const Eigen::Vector3d o0 = data_->oMi[j0].translation();
                    const Eigen::Vector3d r0 = p1 - o0;
                    Jp0 = J_joint.topRows(3) - skew(r0) * J_joint.bottomRows(3);
                }
                if (j1 > 0 && j1 < model_.joints.size()) {
                    J_joint.setZero();
                    pinocchio::getJointJacobian(model_, *data_, j1,
                                                pinocchio::WORLD, J_joint);
                    const Eigen::Vector3d o1 = data_->oMi[j1].translation();
                    const Eigen::Vector3d r1 = p2 - o1;
                    Jp1 = J_joint.topRows(3) - skew(r1) * J_joint.bottomRows(3);
                }
                // 距离变化率 d_dot = n . (v_p2 - v_p1) = n'(Jp1 - Jp0) dq
                const Eigen::RowVectorXd A_full = nrm.transpose() * (Jp1 - Jp0);
                Eigen::RowVectorXd A_act(1, n_active);
                for (int j = 0; j < n_active; ++j) A_act[j] = A_full[active_v[j]];
                if (!A_act.allFinite()) continue;

                A_rows.push_back(A_act);
                lbA_rows.push_back(-coll_gain * (d - d_safe));  // 强制不越过安全距离
                ubA_rows.push_back(std::numeric_limits<double>::infinity());
            }
        }

        // 3.5) 机械臂 6/7 轴组合干涉约束(CCS)：|j6|>20° 时 |j7| <= 110.5 - 1.025*|j6| (度)。
        //      与自碰撞约束相同的一阶线性化：裕量 slack = L(|j6|) - |j7| 不足时，
        //      限制 dq 不得进一步逼近干涉边界；|j6|<=20° 时约束未激活(与判定函数一致)。
        {
            const double ccs_margin_rad = 0.5 * kDegreesToRadians;  // 裕量约 0.5°
            const auto arm_ccs_rows = [&](const std::vector<int>& arm_v_indices) {
                if (arm_v_indices.size() != 7U) return;
                int col6 = -1, col7 = -1;
                for (int j = 0; j < n_active; ++j) {
                    if (active_v[j] == arm_v_indices[5]) col6 = j;
                    else if (active_v[j] == arm_v_indices[6]) col7 = j;
                }
                if (col6 < 0 || col7 < 0) return;
                const double j6_rad = q[active_q[col6]];
                const double j7_rad = q[active_q[col7]];
                if (!std::isfinite(j6_rad) || !std::isfinite(j7_rad)) return;
                if (std::abs(j6_rad * kRadiansToDegrees) <= kCcsJoint6ActivationDeg)
                    return;  // 约束未激活
                // L_rad = offset_rad - slope*|j6_rad|；slack = L_rad - |j7_rad|
                const double offset_rad = kCcsJoint67OffsetDeg * kDegreesToRadians;
                const double slack =
                    offset_rad - kCcsJoint67Slope * std::abs(j6_rad) -
                    std::abs(j7_rad);
                Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(n_active);
                row[col6] = -kCcsJoint67Slope * (j6_rad >= 0.0 ? 1.0 : -1.0);
                row[col7] = -(j7_rad >= 0.0 ? 1.0 : -1.0);
                if (!row.allFinite()) return;
                A_rows.push_back(row);
                lbA_rows.push_back(-coll_gain * (slack - ccs_margin_rad));
                ubA_rows.push_back(std::numeric_limits<double>::infinity());
            };
            if (left_goal)  arm_ccs_rows(left_arm_v_indices_);
            if (right_goal) arm_ccs_rows(right_arm_v_indices_);
        }

        // 4) 组装 QP 目标：min 0.5 dq'H dq + dq'g
        //    H = 2 (J'J + diag(W) + 1e-9) ;  g = -2 J' e
        Eigen::MatrixXd H = 2.0 * (J_full.transpose() * J_full);
        H.diagonal().array() += 2.0 * W.array();
        H.diagonal().array() += 2.0e-9;  // 数值稳定，保证正定
        Eigen::VectorXd g = -2.0 * (J_full.transpose() * err);

        // 5) 盒约束(关节位置限位 -> 对 dq 的位移边界)
        const double step_now = step_limit(err.norm());
        Eigen::VectorXd lb = Eigen::VectorXd::Zero(n_active);
        Eigen::VectorXd ub = Eigen::VectorXd::Zero(n_active);
        for (int j = 0; j < n_active; ++j) {
            const double qj = q[active_q[j]];
            double lo = lower_pos[j] + safety_margin - qj;
            double up = upper_pos[j] - safety_margin - qj;
            lo = std::max(lo, -step_now);
            up = std::min(up,  step_now);
            if (lo > up) { const double mid = 0.5 * (lo + up); lo = mid; up = mid; }
            lb[j] = lo; ub[j] = up;
        }
        // body_joint 俯仰限位：将躯干关节角度限制在 [-body_pitch_limit, +body_pitch_limit]
        // if (body_q_idx >= 0) {
        //     for (int j = 0; j < n_active; ++j) {
        //         if (active_q[j] != body_q_idx) continue;
        //         const double qj = q[body_q_idx];
        //         double lo = std::max(-body_pitch_limit_rad, lower_pos[j]) - qj;
        //         double up = std::min( body_pitch_limit_rad, upper_pos[j]) - qj;
        //         lo = std::max(lo, lb[j]);
        //         up = std::min(up, ub[j]);
        //         if (lo > up) { const double mid = 0.5 * (lo + up); lo = mid; up = mid; }
        //         lb[j] = lo; ub[j] = up;
        //         break;
        //     }
        // }

        // 6) 求解 QP(qpOASES)。激活的碰撞对数随位形动态变化，逐次重建 QP。
        Eigen::VectorXd dq = Eigen::VectorXd::Zero(n_active);
        bool qp_ok = false;
        const int nC = static_cast<int>(A_rows.size());

        // 行主序拷贝矩阵
        std::vector<qpOASES::real_t> H_arr(static_cast<size_t>(n_active * n_active));
        std::vector<qpOASES::real_t> g_arr(static_cast<size_t>(n_active));
        std::vector<qpOASES::real_t> lb_arr(static_cast<size_t>(n_active));
        std::vector<qpOASES::real_t> ub_arr(static_cast<size_t>(n_active));
        for (int j = 0; j < n_active; ++j) {
            g_arr[j] = g[j];
            lb_arr[j] = lb[j];
            ub_arr[j] = ub[j];
            for (int i = 0; i < n_active; ++i)
                H_arr[static_cast<size_t>(i * n_active + j)] = H(i, j);
        }
        std::vector<qpOASES::real_t> A_arr, lbA_arr, ubA_arr;
        if (nC > 0) {
            A_arr.resize(static_cast<size_t>(nC * n_active));
            lbA_arr.resize(static_cast<size_t>(nC));
            ubA_arr.resize(static_cast<size_t>(nC));
            for (int r = 0; r < nC; ++r) {
                lbA_arr[r] = lbA_rows[r];
                ubA_arr[r] = ubA_rows[r];
                for (int j = 0; j < n_active; ++j)
                    A_arr[static_cast<size_t>(r * n_active + j)] = A_rows[r][j];
            }
        }

        try {
            qpOASES::Options options;
            options.setToFast();               // 实时性优先的预设
            options.printLevel = qpOASES::PL_NONE;

            qpOASES::QProblem qp(n_active, nC, qpOASES::HST_POSDEF, qpOASES::BT_TRUE);
            qp.setOptions(options);
            qpOASES::int_t nWSR = 50;
            qpOASES::real_t cputime = 0.02;    // 单步最大 CPU 时间(s)
            const qpOASES::returnValue res = qp.init(
                H_arr.data(), g_arr.data(),
                (nC > 0 ? A_arr.data() : nullptr),
                lb_arr.data(), ub_arr.data(),
                (nC > 0 ? lbA_arr.data() : nullptr),
                (nC > 0 ? ubA_arr.data() : nullptr),
                nWSR, &cputime);

            if (res == qpOASES::SUCCESSFUL_RETURN ||
                res == qpOASES::RET_MAX_NWSR_REACHED) {
                std::vector<qpOASES::real_t> xbuf(static_cast<size_t>(n_active));
                if (qp.getPrimalSolution(xbuf.data()) == qpOASES::SUCCESSFUL_RETURN) {
                    for (int j = 0; j < n_active; ++j) dq[j] = xbuf[j];
                    qp_ok = true;
                }
            }
        } catch (...) {
            qp_ok = false;
        }

        if (!qp_ok) {
            // QP 不可行(约束冲突/奇异位形)：以阻尼最小二乘回退，保证鲁棒性
            Eigen::MatrixXd JJt = J_full * J_full.transpose();
            JJt.diagonal().array() += 1.0e-4;
            const Eigen::VectorXd v = J_full.transpose() * JJt.ldlt().solve(err);
            for (int j = 0; j < n_active; ++j)
                dq[j] = std::clamp(v[j], lb[j], ub[j]);
        }

        // 7) 回溯线搜索：保证不违反关节限位/自碰撞(线性化近似的安全网)
        //    若 8 级步长缩小(α=1,0.5,...,1/128)均不通过自碰撞判定(暂时性的中间自碰撞)，
        //    仍接受 α = 1/256 的极小步前进，以避免腿部大角度旋转时被"临时自碰撞"卡死。
        //    解的有效性在收敛后以 isFullQValid 再严格校验(仅在成功分支)。
        double alpha = 1.0;
        Eigen::VectorXd q_next;
        bool accepted = false;
        for (int bt = 0; bt < 8; ++bt, alpha *= 0.5) {
            Eigen::VectorXd dq_full = Eigen::VectorXd::Zero(model_.nv);
            for (int j = 0; j < n_active; ++j) dq_full[active_v[j]] = alpha * dq[j];
            q_next = pinocchio::integrate(model_, q, dq_full);
            // 关节限位裁剪
            for (int j = 0; j < model_.nq; ++j) {
                if (q_next[j] < model_.lowerPositionLimit[j])
                    q_next[j] = model_.lowerPositionLimit[j];
                if (q_next[j] > model_.upperPositionLimit[j])
                    q_next[j] = model_.upperPositionLimit[j];
            }
            bool valid = true;
            if (enable_collision_avoidance) {
                pinocchio::updateGeometryPlacements(model_, *data_, geom_model_,
                                                     *geom_data_, q_next);
                if (pinocchio::computeCollisions(geom_model_, *geom_data_, false))
                    valid = false;
            }
            // 6/7 轴组合干涉约束硬校验：防止步进落入干涉区(含 |j6| 穿越 20° 的情形)
            if (valid && left_goal &&
                !passesSimplifiedCcsJoint67Constraint(q_next, left_arm_q_indices_))
                valid = false;
            if (valid && right_goal &&
                !passesSimplifiedCcsJoint67Constraint(q_next, right_arm_q_indices_))
                valid = false;
            if (valid) { accepted = true; break; }
        }
        if (!accepted) {
            // 所有 8 级步长均碰撞，仍向前走一小步(1/256 倍率)，以渡过暂时性的自碰撞。
            Eigen::VectorXd dq_full = Eigen::VectorXd::Zero(model_.nv);
            const double alpha_min = alpha;  // 此时 alpha = (0.5)^8 = 1/256
            for (int j = 0; j < n_active; ++j) dq_full[active_v[j]] = alpha_min * dq[j];
            q_next = pinocchio::integrate(model_, q, dq_full);
            for (int j = 0; j < model_.nq; ++j) {
                q_next[j] = std::clamp(q_next[j],
                                       model_.lowerPositionLimit[j],
                                       model_.upperPositionLimit[j]);
            }
        }
        q = q_next;

        // 8) 躯干俯仰安全检查：若 FK 俯仰超限，将 body_joint 回退(保守)
        // if (body_q_idx >= 0 && model_.existFrame("body_link")) {
        //     pinocchio::forwardKinematics(model_, *data_, q);
        //     pinocchio::updateFramePlacements(model_, *data_);
        //     const Eigen::Matrix3d& R =
        //         data_->oMf[model_.getFrameId("body_link")].rotation();
        //     const double pitch = std::atan2(-R(2, 0),
        //                                     std::hypot(R(0, 0), R(1, 0)));
        //     if (std::abs(pitch) > body_pitch_limit_rad) {
        //         q[body_q_idx] = std::clamp(q[body_q_idx],
        //                                    -body_pitch_limit_rad,
        //                                     body_pitch_limit_rad);
        //     }
        // }
    }

    q_res = q;

    // 终点有效性检查：收敛后若仍处自碰撞，视为失败(已含可重入锁)
    if (success && enable_collision_avoidance) {
        if (!isFullQValid(q)) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("MiracleMAlgo"),
                         "solveWholeBodyIK:  逆解成功，但碰撞检测失败，q_res="
                         << q_res.transpose());
            success = false;
        }
        
    }
    // 终点 6/7 轴组合干涉约束校验(独立于自碰撞开关，始终生效)
    if (success) {
        if (left_goal &&
            !passesSimplifiedCcsJoint67Constraint(q, left_arm_q_indices_)) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("MiracleMAlgo"),
                             "solveWholeBodyIK:  逆解成功，但 左臂6/7 轴组合干涉约束校验失败，q_res="
                             << q_res.transpose());
                success = false;
            }
        if (success && right_goal &&
            !passesSimplifiedCcsJoint67Constraint(q, right_arm_q_indices_)) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("MiracleMAlgo"),
                             "solveWholeBodyIK:  逆解成功，但 右臂6/7 轴组合干涉约束校验失败，q_res="
                             << q_res.transpose());
                success = false;
            }
    }
    return success;
}

// =============================================================================
// 全身逆解对外接口：多种子 + 超时控制 + 错误处理
// -----------------------------------------------------------------------------
// 输入：q_init 初始关节状态；left_goal/right_goal 目标位姿(至少其一)；
//       leg_weights 腿部姿态权重；body_pitch_limit_rad 躯干俯仰限位；
//       timeout_seconds 超时；max_seed_attempts 额外随机种子次数；
//       enable_collision_avoidance 是否启用自碰撞避免约束。
// 输出：q_res 求得的关节角度；返回 true=有效解，false=无解/奇异/超时。
// =============================================================================
bool MiracleMAlgo::solveWholeBodyIK(
    const Eigen::VectorXd& q_init,
    const std::optional<Eigen::Isometry3d>& left_goal,
    const std::optional<Eigen::Isometry3d>& right_goal,
    Eigen::VectorXd& q_res,
    const std::array<double, 4>& leg_weights,
    double body_pitch_limit_rad,
    double timeout_seconds,
    std::size_t max_seed_attempts,
    bool enable_collision_avoidance)
{
    // --- 输入校验 ---
    if (q_init.size() != model_.nq) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"),
                     "solveWholeBodyIK: q_init size=%d, expected=%d",
                     static_cast<int>(q_init.size()), model_.nq);
        return false;
    }
    if (!left_goal && !right_goal) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"),
                     "solveWholeBodyIK: 至少需提供一个手臂目标位姿");
        return false;
    }
    if (left_goal && !left_goal->matrix().allFinite()) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "solveWholeBodyIK: left_goal 非有限");
        return false;
    }
    if (right_goal && !right_goal->matrix().allFinite()) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "solveWholeBodyIK: right_goal 非有限");
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<long long>(timeout_seconds * 1000.0));

    // 种子 0：初始关节状态
    if (solveWholeBodyIKOnce(q_init, left_goal, right_goal, q_res,
                             leg_weights, body_pitch_limit_rad,
                             deadline, enable_collision_avoidance)) {
        return true;
    }

    // 无额外种子请求：直接返回失败(奇异位形/无解)
    if (max_seed_attempts == 0U) return false;

    // 额外随机种子：手臂 + 腿部 联合扰动。
    // 注意：腿部关节大角度联动是达到远距离目标的必要条件（例如零位 y≈1.1 的身体
    // 需要旋到 y≈0.1 附近），因此必须在种子中采样腿部初值，以避免初始姿态把
    // 优化困在"腿几乎不动、臂无法到达"的局部极小。
    std::mt19937 rng(12345u);  // 固定种子，保证可复现
    std::uniform_real_distribution<double> arm_dist(-3.14159, 3.14159);  // ±π: 覆盖不同肘部分支
    // 腿部扰动范围：按 leg 限位的 ±30% 均匀采样（足够覆盖多种折叠/展开姿态）
    std::uniform_real_distribution<double> leg_dist(0.0, 1.0);

    // 收集被请求手臂的 q 索引
    std::vector<int> arm_q;
    auto collect_arm_q = [&](const std::vector<int>& v_idxs) {
        for (int v : v_idxs) {
            int q_idx = v;
            for (pinocchio::JointIndex jid = 1; jid < model_.joints.size(); ++jid) {
                if (model_.joints[jid].idx_v() == v) { q_idx = model_.joints[jid].idx_q(); break; }
            }
            arm_q.push_back(q_idx);
        }
    };
    if (left_goal)  collect_arm_q(left_arm_v_indices_);
    if (right_goal) collect_arm_q(right_arm_v_indices_);

    // 收集腿部的 q 索引（与 leg_weights 顺序一致）
    std::vector<int> leg_q;
    leg_q.reserve(leg_v_indices_.size());
    for (int v : leg_v_indices_) {
        int q_idx = v;
        for (pinocchio::JointIndex jid = 1; jid < model_.joints.size(); ++jid) {
            if (model_.joints[jid].idx_v() == v) { q_idx = model_.joints[jid].idx_q(); break; }
        }
        leg_q.push_back(q_idx);
    }

    for (std::size_t s = 0U; s < max_seed_attempts; ++s) {
        if (std::chrono::steady_clock::now() > deadline) break;
        Eigen::VectorXd seed = q_init;
        for (int q_idx : arm_q) {
            const double lo = model_.lowerPositionLimit[q_idx];
            const double up = model_.upperPositionLimit[q_idx];
            seed[q_idx] = std::clamp(q_init[q_idx] + arm_dist(rng), lo, up);
        }
        // 腿部在限位的 [30%, 100%] 区间内随机（偏向上限=展开方向），不越界
        for (size_t k = 0; k < leg_q.size(); ++k) {
            const int q_idx = leg_q[k];
            const double lo = model_.lowerPositionLimit[q_idx];
            const double up = model_.upperPositionLimit[q_idx];
            const double range = up - lo;
            const double sample = lo + range * (0.30 + 0.70 * leg_dist(rng));
            seed[q_idx] = sample;  // 已在 [lo, up] 内，无需 clamp
        }
        if (solveWholeBodyIKOnce(seed, left_goal, right_goal, q_res,
                                leg_weights, body_pitch_limit_rad,
                                deadline, enable_collision_avoidance)) {
            return true;
        }
    }

    RCLCPP_WARN(rclcpp::get_logger("MiracleMAlgo"),
                "solveWholeBodyIK: 所有种子均未收敛(无解/奇异/超时)");
    return false;
}



bool MiracleMAlgo::planDualArmCartesianGoal(const std::vector<double>& start_q,
                                               const Eigen::Isometry3d& left_goal,
                                               const Eigen::Isometry3d& right_goal,
                                               std::vector<std::vector<double>>& path) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    Eigen::VectorXd q_init = toEigenConfig(start_q);
    Eigen::VectorXd q_res;
    if(!solveDualIK(q_init, left_goal, right_goal, q_res)) return false;
    
    std::vector<double> goal_q(14);
    for (int i = 0; i < 7; ++i) {
        goal_q[static_cast<size_t>(i)] = q_res[left_arm_q_indices_[static_cast<size_t>(i)]];
        goal_q[static_cast<size_t>(7 + i)] = q_res[right_arm_q_indices_[static_cast<size_t>(i)]];
    }
    return planDualArmJointSpace(start_q, goal_q, path);
}

bool MiracleMAlgo::planSingleArmCartesianGoal(bool is_left,
                                                 const std::vector<double>& start_q,
                                                 const Eigen::Isometry3d& goal,
                                                 std::vector<std::vector<double>>& path,
                                                 double* ik_time_seconds) {
    if (ik_time_seconds != nullptr) {
        *ik_time_seconds = 0.0;
    }
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    if (start_q.size() != static_cast<size_t>(model_.nq)) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"),
                     "planSingleArmCartesianGoal: unsupported start_q size: %zu, expected %d",
                     start_q.size(), model_.nq);
        return false;
    }
    Eigen::VectorXd q_init = toEigenConfig(start_q);
    Eigen::VectorXd q_res;
    const auto ik_started_at = std::chrono::steady_clock::now();
    const bool ik_ok = solveIK(q_init, goal, q_res, is_left ? 0 : 1);
    if (ik_time_seconds != nullptr) {
        *ik_time_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - ik_started_at).count();
    }
    if (!ik_ok) return false;
    
    const std::vector<size_t>& active_indices = is_left ? left_arm_q_indices_ : right_arm_q_indices_;
    std::vector<double> goal_q(7);
    for (int i = 0; i < 7; ++i) {
        goal_q[static_cast<size_t>(i)] = q_res[active_indices[static_cast<size_t>(i)]];
    }
    return planSingleArmJointSpace(is_left, start_q, goal_q, path);
}

bool MiracleMAlgo::solveSingleArmCartesianIk(
    bool is_left,
    const std::vector<double>& start_q,
    const Eigen::Isometry3d& goal,
    std::vector<double>& goal_q) {
    goal_q.clear();
    if (start_q.size() != static_cast<size_t>(model_.nq) ||
        !goal.matrix().allFinite()) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"),
                     "solveSingleArmCartesianIk: invalid full q or goal pose");
        return false;
    }

    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    const Eigen::VectorXd q_init = toEigenConfig(start_q);
    Eigen::VectorXd q_result;
    if (!solveIK(q_init, goal, q_result, is_left ? 0 : 1)) {
        return false;
    }

    const auto& indices = is_left ? left_arm_q_indices_ : right_arm_q_indices_;
    goal_q.resize(indices.size());
    for (size_t index = 0; index < indices.size(); ++index) {
        goal_q[index] = q_result[indices[index]];
    }
    return true;
}

bool MiracleMAlgo::solveDualArmCartesianIk(
    const std::vector<double>& start_q,
    const Eigen::Isometry3d& left_goal,
    const Eigen::Isometry3d& right_goal,
    std::vector<double>& goal_q) {
    goal_q.clear();
    if (start_q.size() != static_cast<size_t>(model_.nq) ||
        !left_goal.matrix().allFinite() ||
        !right_goal.matrix().allFinite()) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"),
                     "solveDualArmCartesianIk: invalid full q or goal pose");
        return false;
    }

    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    Eigen::VectorXd q_result;
    if (!solveDualIK(toEigenConfig(start_q), left_goal, right_goal, q_result)) {
        return false;
    }
    goal_q.resize(14U);
    for (std::size_t index = 0; index < 7U; ++index) {
        goal_q[index] = q_result[left_arm_q_indices_[index]];
        goal_q[7U + index] = q_result[right_arm_q_indices_[index]];
    }
    return true;
}

bool MiracleMAlgo::extractSingleArmJoints(
    bool is_left,
    const std::vector<double>& full_q,
    std::vector<double>& arm_q) const {
    arm_q.clear();
    if (full_q.size() != static_cast<size_t>(model_.nq)) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"),
                     "extractSingleArmJoints: unsupported full q size: %zu, expected %d",
                     full_q.size(), model_.nq);
        return false;
    }
    const auto& indices = is_left ? left_arm_q_indices_ : right_arm_q_indices_;
    arm_q.resize(indices.size());
    for (size_t index = 0; index < indices.size(); ++index) {
        arm_q[index] = full_q[indices[index]];
    }
    return true;
}

bool MiracleMAlgo::extractLegJoints(
    const std::vector<double>& full_q,
    std::vector<double>& leg_q) const
{
    leg_q.clear();
    if (full_q.size() != static_cast<size_t>(model_.nq)) {
        return false;
    }
    leg_q.reserve(leg_q_indices_.size());
    for (const std::size_t index : leg_q_indices_) {
        if (index >= full_q.size()) {
            leg_q.clear();
            return false;
        }
        leg_q.push_back(full_q[index]);
    }
    return true;
}

bool MiracleMAlgo::mergeWholeBodyJoints(
    const std::vector<double>& reference_full_q,
    bool use_left,
    bool use_right,
    const std::vector<double>& active_joints,
    std::vector<double>& full_q) const
{
    full_q.clear();
    if (reference_full_q.size() != static_cast<std::size_t>(model_.nq) ||
        (!use_left && !use_right)) {
        return false;
    }
    const std::size_t expected_size =
        (use_left ? left_arm_q_indices_.size() : 0U) +
        (use_right ? right_arm_q_indices_.size() : 0U) +
        leg_q_indices_.size();
    if (active_joints.size() != expected_size) {
        return false;
    }

    full_q = reference_full_q;
    std::size_t offset = 0U;
    const auto assign = [&](const std::vector<std::size_t>& indices) {
        for (std::size_t index = 0; index < indices.size(); ++index) {
            full_q[indices[index]] = active_joints[offset + index];
        }
        offset += indices.size();
    };
    if (use_left) {
        assign(left_arm_q_indices_);
    }
    if (use_right) {
        assign(right_arm_q_indices_);
    }
    assign(leg_q_indices_);
    return true;
}

bool MiracleMAlgo::getFullJointState(std::vector<double>& full_q) const {
    std::lock_guard<std::mutex> lock(upadate_joinst_state_msg);
    if (_m_full_joint_state_msg.size() != static_cast<size_t>(model_.nq)) {
        full_q.clear();
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"),
                     "getFullJointState: internal state size=%zu, expected=%d",
                     _m_full_joint_state_msg.size(), model_.nq);
        return false;
    }
    full_q = _m_full_joint_state_msg;
    return true;
}

bool MiracleMAlgo::transformPoseToModelRoot(
    const std::string& source_frame,
    const std::vector<double>& full_q,
    const Eigen::Isometry3d& source_to_target,
    Eigen::Isometry3d& root_to_target) {
    if (full_q.size() != static_cast<size_t>(model_.nq) ||
        !source_to_target.matrix().allFinite() ||
        !model_.existFrame(source_frame)) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"),
                     "transformPoseToModelRoot: invalid q, source frame or target pose");
        return false;
    }

    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    const Eigen::VectorXd q = toEigenConfig(full_q);
    pinocchio::forwardKinematics(model_, *data_, q);
    pinocchio::updateFramePlacements(model_, *data_);
    const auto source_id = model_.getFrameId(source_frame);
    Eigen::Isometry3d root_to_source = Eigen::Isometry3d::Identity();
    root_to_source.matrix() = data_->oMf[source_id].toHomogeneousMatrix();
    root_to_target = root_to_source * source_to_target;
    return root_to_target.matrix().allFinite();
}

bool MiracleMAlgo::planDualArmCartesianPath(const std::vector<double>& start_q,
                                               const Eigen::Isometry3d& left_goal,
                                               const Eigen::Isometry3d& right_goal,
                                               std::vector<std::vector<double>>& path,
                                               double step_size) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    Eigen::VectorXd q_current = toEigenConfig(start_q);
    pinocchio::forwardKinematics(model_, *data_, q_current);
    pinocchio::updateFramePlacements(model_, *data_);
    
    pinocchio::SE3 left_start = data_->oMf[left_ee_frame_id_];
    pinocchio::SE3 right_start = data_->oMf[right_ee_frame_id_];
    pinocchio::SE3 left_end(left_goal.rotation(), left_goal.translation());
    pinocchio::SE3 right_end(right_goal.rotation(), right_goal.translation());
    
    double d_pos = std::max((left_end.translation() - left_start.translation()).norm(),
                            (right_end.translation() - right_start.translation()).norm());
    int steps = std::max(10, static_cast<int>(d_pos / step_size));
    
    path.clear();
    path.push_back(start_q);
    
    for(int i=1; i<=steps; ++i) {
        double alpha = static_cast<double>(i) / steps;
        pinocchio::SE3 left_interp = pinocchio::SE3::Interpolate(left_start, left_end, alpha);
        pinocchio::SE3 right_interp = pinocchio::SE3::Interpolate(right_start, right_end, alpha);
        
        Eigen::VectorXd q_next;
        Eigen::Isometry3d lg, rg;
        lg.translation() = left_interp.translation(); lg.linear() = left_interp.rotation();
        rg.translation() = right_interp.translation(); rg.linear() = right_interp.rotation();
        
        if(!solveDualIK(q_current, lg, rg, q_next)) {
            RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "PlanDualArmCartesianPath: failed to solve IK at step %d/%d", i, steps);
            return false;
        }
        
        pinocchio::updateGeometryPlacements(model_, *data_, geom_model_, *geom_data_, q_next);
        if(pinocchio::computeCollisions(geom_model_, *geom_data_, true)) {
            RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "PlanDualArmCartesianPath: collision detected at step %d/%d", i, steps);
            return false;
        }
        
        q_current = q_next;
        std::vector<double> current_q(static_cast<size_t>(model_.nq));
        for(int j=0; j<model_.nq; ++j) {
            current_q[static_cast<size_t>(j)] = q_current[j];
        }
        path.push_back(current_q);
    }
    return true;
}

bool MiracleMAlgo::planDualArmCartesianLine(const std::vector<double>& start_q,
                                           const std::vector<double>& left_end_poseture,
                                           const std::vector<double>& right_end_poseture,
                                           const double& dt,const double& v_max,
                                           const double& a_max, const double& j_max,
                                           std::vector<std::vector<double>>& path) { 
    
    std::vector<std::vector<double>> left_path,right_path;

    bool left_success = false;
    bool right_success = false;

    // 1. 定义线程任务 Lambda
    auto left_task = [&]() {
        left_success = planSingleArmCartesianLine(true, start_q, left_end_poseture, dt, v_max, a_max, j_max, left_path);
    };

    auto right_task = [&]() {
        right_success = planSingleArmCartesianLine(false, start_q, right_end_poseture, dt, v_max, a_max, j_max, right_path);
    };

    // 2. 创建并立即启动线程
    std::thread t_left(left_task);
    std::thread t_right(right_task);

    // 3. 等待两个线程执行完毕
    t_left.join();
    t_right.join();

    // 4. 检查执行结果
    if (!left_success) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "planDualArmCartesianLine: failed to plan left arm path");
        return false;
    }
    if (!right_success) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "planDualArmCartesianLine: failed to plan right arm path");
        return false;
    }

    std::vector<double> full_q;
    
    // size_t sync_size = std::max(left_path.size(), right_path.size());
    // int min_points_size = std::min(left_path.size(),right_path.size());
    path.clear();
    if(left_path.size()>=right_path.size()){
        for(size_t i=0; i<right_path.size(); ++i) {
            full_q=left_path[i];
            full_q.insert(full_q.end(),right_path[i].begin(),right_path[i].end());
            path.push_back(full_q);
        }
        for(size_t i=right_path.size(); i<left_path.size(); ++i){
            full_q=left_path[i];
            full_q.insert(full_q.end(),right_path.back().begin(),right_path.back().end());
            path.push_back(full_q);
        }
    }
    else{
        for(size_t i=0; i<left_path.size(); ++i) {
            full_q=left_path[i];
            full_q.insert(full_q.end(),right_path[i].begin(),right_path[i].end());
            path.push_back(full_q);
        }
        for(size_t i=left_path.size(); i<right_path.size(); ++i){
            full_q=left_path.back();
            full_q.insert(full_q.end(),right_path[i].begin(),right_path[i].end());
            path.push_back(full_q);
        }
    }
    
        
    RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"), "shuangbi guihua wancheng inside");
    return true;
}

bool MiracleMAlgo::planSingleArmCartesianPath(bool is_left,
                                                 const std::vector<double>& start_q,
                                                 const Eigen::Isometry3d& goal,
                                                 std::vector<std::vector<double>>& path,
                                                 double step_size) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    Eigen::VectorXd q_current = toEigenConfig(start_q);
    pinocchio::forwardKinematics(model_, *data_, q_current);
    pinocchio::updateFramePlacements(model_, *data_);
    
    pinocchio::FrameIndex ee_id = is_left ? left_ee_frame_id_ : right_ee_frame_id_;
    pinocchio::SE3 start_pose = data_->oMf[ee_id];
    pinocchio::SE3 end_pose(goal.rotation(), goal.translation());
    
    double d_pos = (end_pose.translation() - start_pose.translation()).norm();
    int steps = std::max(10, static_cast<int>(d_pos / step_size));
    
    path.clear();
    path.push_back(start_q);
    
    for(int i=1; i<=steps; ++i) {
        double alpha = static_cast<double>(i) / steps;
        pinocchio::SE3 interp = pinocchio::SE3::Interpolate(start_pose, end_pose, alpha);
        
        Eigen::VectorXd q_next;
        Eigen::Isometry3d g;
        g.translation() = interp.translation(); g.linear() = interp.rotation();
        
        if(!solveIK(q_current, g, q_next, is_left ? 0 : 1)) {
            RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "PlanSingleArmCartesianPath: failed to solve IK at step %d/%d", i, steps);
            return false;
        }
        
        pinocchio::updateGeometryPlacements(model_, *data_, geom_model_, *geom_data_, q_next);
        if(pinocchio::computeCollisions(geom_model_, *geom_data_, true)) {
            RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "PlanSingleArmCartesianPath: collision detected at step %d/%d", i, steps);
            return false;
        }
        
        q_current = q_next;
        std::vector<double> current_q(static_cast<size_t>(model_.nq));
        for(int j=0; j<model_.nq; ++j) {
            current_q[static_cast<size_t>(j)] = q_current[j];
        }
        path.push_back(current_q);
    }
    return true;
}

bool MiracleMAlgo::planSingleArmCartesianIkWaypoints(
    bool is_left,
    const std::vector<double>& start_q,
    const Eigen::Isometry3d& goal,
    double max_translation_step_m,
    double max_rotation_step_rad,
    std::vector<std::vector<double>>& arm_waypoints,
    Eigen::Isometry3d& actual_start_pose,
    double* ik_time_seconds)
{
    arm_waypoints.clear();
    actual_start_pose = Eigen::Isometry3d::Identity();
    if (ik_time_seconds != nullptr) {
        *ik_time_seconds = 0.0;
    }
    if (start_q.size() != static_cast<std::size_t>(model_.nq) ||
        !goal.matrix().allFinite())
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MiracleMAlgo"),
            "planSingleArmCartesianIkWaypoints: invalid full q or goal");
        return false;
    }

    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    Eigen::VectorXd q_current = toEigenConfig(start_q);
    pinocchio::forwardKinematics(model_, *data_, q_current);
    pinocchio::updateFramePlacements(model_, *data_);

    const pinocchio::FrameIndex ee_id =
        is_left ? left_ee_frame_id_ : right_ee_frame_id_;
    const pinocchio::SE3 start_se3 = data_->oMf[ee_id];
    actual_start_pose.matrix() = start_se3.toHomogeneousMatrix();
    const auto sampling =
        motion_control_pkg::motion::computeCartesianLineSegmentCount(
        actual_start_pose, goal, max_translation_step_m,
        max_rotation_step_rad);
    if (!sampling.valid) {
        RCLCPP_ERROR(
            rclcpp::get_logger("MiracleMAlgo"),
            "planSingleArmCartesianIkWaypoints: %s",
            sampling.error_message.c_str());
        return false;
    }
    // 保持旧 Move_L 的最短平移保护，避免把数值噪声放大成轨迹。
    if (sampling.translation_distance_m < 1.0e-2) {
        RCLCPP_ERROR(
            rclcpp::get_logger("MiracleMAlgo"),
            "planSingleArmCartesianIkWaypoints: translation %.6f m is shorter than 0.01 m",
            sampling.translation_distance_m);
        return false;
    }

    const auto& indices =
        is_left ? left_arm_q_indices_ : right_arm_q_indices_;
    if (indices.size() != 7U) {
        RCLCPP_ERROR(
            rclcpp::get_logger("MiracleMAlgo"),
            "planSingleArmCartesianIkWaypoints: active arm does not have seven joints");
        return false;
    }
    const auto append_active_arm = [&](const Eigen::VectorXd& full_q) {
        std::vector<double> arm_q;
        arm_q.reserve(indices.size());
        for (const std::size_t index : indices) {
            if (index >= static_cast<std::size_t>(full_q.size())) {
                return false;
            }
            arm_q.push_back(full_q[static_cast<Eigen::Index>(index)]);
        }
        arm_waypoints.push_back(std::move(arm_q));
        return true;
    };

    arm_waypoints.reserve(sampling.segments + 1U);
    if (!append_active_arm(q_current)) {
        arm_waypoints.clear();
        return false;
    }

    SingleArmIkWorkspace workspace;
    Eigen::VectorXd q_next(model_.nq);
    for (std::size_t segment = 1U; segment <= sampling.segments; ++segment) {
        const double alpha = static_cast<double>(segment) /
            static_cast<double>(sampling.segments);
        const pinocchio::SE3 interpolated = pinocchio::SE3::Interpolate(
            start_se3, pinocchio::SE3(goal.rotation(), goal.translation()), alpha);
        Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
        target.matrix() = interpolated.toHomogeneousMatrix();

        const auto ik_started_at = std::chrono::steady_clock::now();
        const bool ik_ok = solveIKWithLockedData(
            q_current, target, q_next, is_left ? 0 : 1, workspace);
        if (ik_time_seconds != nullptr) {
            *ik_time_seconds += std::chrono::duration<double>(
                std::chrono::steady_clock::now() - ik_started_at).count();
        }
        if (!ik_ok) {
            RCLCPP_ERROR(
                rclcpp::get_logger("MiracleMAlgo"),
                "planSingleArmCartesianIkWaypoints: IK failed at segment %zu/%zu",
                segment, sampling.segments);
            arm_waypoints.clear();
            return false;
        }
        q_current = q_next;
        if (!append_active_arm(q_current)) {
            arm_waypoints.clear();
            return false;
        }
    }

    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "Cartesian line coarse IK ready: segments=%zu, translation=%.6f m, rotation=%.6f rad, step<=%.6f m/%.6f rad, ik_time=%.6f s",
        sampling.segments, sampling.translation_distance_m,
        sampling.rotation_distance_rad, max_translation_step_m,
        max_rotation_step_rad,
        ik_time_seconds != nullptr ? *ik_time_seconds : 0.0);
    return true;
}

bool MiracleMAlgo::validateSingleArmCartesianTrajectory(
    bool is_left,
    const std::vector<double>& reference_full_q,
    const Eigen::Isometry3d& start_pose,
    const Eigen::Isometry3d& goal_pose,
    const std::vector<std::vector<double>>& arm_trajectory,
    double position_tolerance_m,
    double orientation_tolerance_rad,
    double& max_position_error_m,
    double& max_orientation_error_rad,
    std::string& error_message)
{
    max_position_error_m = 0.0;
    max_orientation_error_rad = 0.0;
    error_message.clear();
    const auto fk_validation_started_at = std::chrono::steady_clock::now();
    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "Cartesian line FK validation: begin arm=%s samples=%zu position_tolerance=%.9f m orientation_tolerance=%.9f rad",
        is_left ? "left" : "right", arm_trajectory.size(),
        position_tolerance_m, orientation_tolerance_rad);
    const auto log_fk_validation_end = [&](bool valid) {
        const double elapsed_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - fk_validation_started_at).count();
        RCLCPP_INFO(
            rclcpp::get_logger("MiracleMAlgo"),
            "Cartesian line FK validation: end arm=%s result=%s samples=%zu elapsed=%.6f s max_position_error=%.9f m max_orientation_error=%.9f rad error=%s",
            is_left ? "left" : "right", valid ? "success" : "failure",
            arm_trajectory.size(), elapsed_seconds, max_position_error_m,
            max_orientation_error_rad,
            error_message.empty() ? "none" : error_message.c_str());
    };
    if (reference_full_q.size() != static_cast<std::size_t>(model_.nq) ||
        arm_trajectory.empty() || !start_pose.matrix().allFinite() ||
        !goal_pose.matrix().allFinite() ||
        !std::isfinite(position_tolerance_m) || position_tolerance_m <= 0.0 ||
        !std::isfinite(orientation_tolerance_rad) || orientation_tolerance_rad <= 0.0)
    {
        error_message = "Cartesian line FK validation inputs are invalid";
        log_fk_validation_end(false);
        return false;
    }

    const auto& indices =
        is_left ? left_arm_q_indices_ : right_arm_q_indices_;
    if (indices.size() != 7U) {
        error_message = "Cartesian line FK validation requires seven arm joints";
        log_fk_validation_end(false);
        return false;
    }

    const double translation_distance =
        (goal_pose.translation() - start_pose.translation()).norm();
    if (!std::isfinite(translation_distance) || translation_distance < 1.0e-2) {
        error_message = "Cartesian line FK validation translation is too short";
        log_fk_validation_end(false);
        return false;
    }
    const double progress_tolerance =
        std::max(1.0e-6, position_tolerance_m / translation_distance);

    const auto fk_lock_started_at = std::chrono::steady_clock::now();
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    const double fk_lock_wait_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - fk_lock_started_at).count();
    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "Cartesian line FK validation: Pinocchio lock acquired arm=%s wait=%.6f s",
        is_left ? "left" : "right", fk_lock_wait_seconds);
    Eigen::VectorXd full_q = toEigenConfig(reference_full_q);
    const pinocchio::FrameIndex ee_id =
        is_left ? left_ee_frame_id_ : right_ee_frame_id_;
    double previous_progress = -progress_tolerance;
    Eigen::Isometry3d final_pose = Eigen::Isometry3d::Identity();
    for (std::size_t sample_index = 0U;
        sample_index < arm_trajectory.size(); ++sample_index)
    {
        const auto& arm_q = arm_trajectory[sample_index];
        if (arm_q.size() != indices.size()) {
            error_message = "Cartesian line FK validation arm sample dimension mismatch";
            log_fk_validation_end(false);
            return false;
        }
        for (std::size_t joint = 0U; joint < indices.size(); ++joint) {
            full_q[static_cast<Eigen::Index>(indices[joint])] = arm_q[joint];
        }
        pinocchio::forwardKinematics(model_, *data_, full_q);
        pinocchio::updateFramePlacements(model_, *data_);
        Eigen::Isometry3d sample_pose = Eigen::Isometry3d::Identity();
        sample_pose.matrix() = data_->oMf[ee_id].toHomogeneousMatrix();
        const auto sample_error =
            motion_control_pkg::motion::evaluateCartesianLineSample(
            start_pose, goal_pose, sample_pose);
        if (!sample_error.valid) {
            error_message = sample_error.error_message;
            log_fk_validation_end(false);
            return false;
        }

        max_position_error_m = std::max(
            max_position_error_m, sample_error.position_error_m);
        max_orientation_error_rad = std::max(
            max_orientation_error_rad, sample_error.orientation_error_rad);
        if (sample_error.progress < -progress_tolerance ||
            sample_error.progress > 1.0 + progress_tolerance)
        {
            std::ostringstream stream;
            stream << "Cartesian line progress out of range at sample "
                   << sample_index << ": " << sample_error.progress;
            error_message = stream.str();
            log_fk_validation_end(false);
            return false;
        }
        if (sample_error.progress + progress_tolerance < previous_progress) {
            std::ostringstream stream;
            stream << "Cartesian line progress regressed at sample "
                   << sample_index << ": previous=" << previous_progress
                   << " current=" << sample_error.progress;
            error_message = stream.str();
            log_fk_validation_end(false);
            return false;
        }
        if (sample_error.position_error_m > position_tolerance_m ||
            sample_error.orientation_error_rad > orientation_tolerance_rad)
        {
            std::ostringstream stream;
            stream << "Cartesian line FK tolerance exceeded at sample "
                   << sample_index << ": position_error="
                   << sample_error.position_error_m << " m, orientation_error="
                   << sample_error.orientation_error_rad << " rad";
            error_message = stream.str();
            log_fk_validation_end(false);
            return false;
        }
        previous_progress = std::max(previous_progress, sample_error.progress);
        final_pose = sample_pose;
    }

    const double endpoint_position_error =
        (final_pose.translation() - goal_pose.translation()).norm();
    const double endpoint_orientation_error =
        motion_control_pkg::motion::quaternionAngularDistance(
        Eigen::Quaterniond(final_pose.rotation()),
        Eigen::Quaterniond(goal_pose.rotation()));
    if (endpoint_position_error > position_tolerance_m ||
        endpoint_orientation_error > orientation_tolerance_rad)
    {
        std::ostringstream stream;
        stream << "Cartesian line endpoint mismatch: position_error="
               << endpoint_position_error << " m, orientation_error="
               << endpoint_orientation_error << " rad";
        error_message = stream.str();
        log_fk_validation_end(false);
        return false;
    }
    log_fk_validation_end(true);
    return true;
}

Eigen::Quaterniond MiracleMAlgo::slerp_ruckig(const Eigen::Quaterniond& q0,
                                       const Eigen::Quaterniond& q1,
                                       double t)
{
    // 计算点积（使用 coeffs，顺序为 [x, y, z, w]）
    double dot = q0.coeffs().dot(q1.coeffs());

    Eigen::Quaterniond q1_local = q1;

    // 防止反向，取最短圆弧
    if (dot < 0.0)
    {
        q1_local.coeffs() = -q1_local.coeffs();
        dot = -dot;
    }

    // 限制点积在 [-1, 1]，防止数值误差导致 acos 越界
    dot = std::clamp(dot, -1.0, 1.0);

    double theta_0    = std::acos(dot);
    double theta      = theta_0 * t;
    double sin_theta0 = std::sin(theta_0);

    // 四元数几乎相同，直接返回起始四元数
    if (sin_theta0 < 1e-10)
    {
        return q0;
    }

    double s0 = std::cos(theta) - dot * std::sin(theta) / sin_theta0;
    double s1 = std::sin(theta) / sin_theta0;

    Eigen::Quaterniond result;
    result.coeffs() = s0 * q0.coeffs() + s1 * q1_local.coeffs();
    return result;
}

bool MiracleMAlgo::planSingleArmCartesianLine(bool is_left, const std::vector<double>& start_q,
                                                    const std::vector<double>& end_poseture,
                                                    const double& dt,const double& v_max,
                                                    const double& a_max, const double& j_max,
                                                    std::vector<std::vector<double>>& path,
                                                    double* ik_time_seconds)
{
    if (ik_time_seconds != nullptr) {
        *ik_time_seconds = 0.0;
    }
    // std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    pinocchio::Data local_data(model_); 
    pinocchio::GeometryData local_geom_data(geom_model_);
    Eigen::VectorXd q_current = toEigenConfig(start_q);
    pinocchio::forwardKinematics(model_, local_data, q_current);
    pinocchio::updateFramePlacements(model_, local_data);
    
    pinocchio::FrameIndex ee_id = is_left ? left_ee_frame_id_ : right_ee_frame_id_;
    pinocchio::SE3 start_poseture = data_->oMf[ee_id];
    Eigen::Vector3d start_pose=start_poseture.translation();
    Eigen::Vector3d end_pose(end_poseture[0], end_poseture[1], end_poseture[2]);
    // RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"), "end pose: %f,%f,%f", 
    //                                 end_poseture[0],end_poseture[1],end_poseture[2]);
    Eigen::Vector3d delta_X = end_pose - start_pose;
    double d_pos = delta_X.norm();
    if (d_pos < 1e-2)
    {
        std::cout << "起点终点重合！" << std::endl;
        return false;
    }

    Eigen::Quaterniond start_quat(start_poseture.rotation());
    Eigen::Quaterniond end_quat(end_poseture[6], end_poseture[3], end_poseture[4], end_poseture[5]);

    // ===================== Ruckig 一维规划器 =====================
    ruckig::Ruckig<1> otg(dt);
    ruckig::InputParameter<1> input;
    ruckig::OutputParameter<1> output;

    // 初始状态
    input.current_position = {0.0};
    input.current_velocity = {0.0};
    input.current_acceleration = {0.0};

    // 目标状态
    input.target_position = {d_pos};
    input.target_velocity = {0.0};
    input.target_acceleration = {0.0};

    // 运动约束
    input.max_velocity = {v_max};
    input.max_acceleration = {a_max};
    input.max_jerk = {j_max};

    double t = 0.0;
    // double s0 = input.current_position[0];
    // double lambda0 = s0 / d_pos;

    std::vector<double> start_p={start_pose[0], start_pose[1], start_pose[2], start_quat.x(), start_quat.y(), start_quat.z(), start_quat.w()};
    std::vector<std::vector<double>>pose_path;
    pose_path.push_back(start_p);
    // path.push_back(start_q);

    while (true)
    {
        ruckig::Result result = otg.update(input, output);

        if (result != ruckig::Result::Working)
        {
            break;
        }

        t += dt;

        double s_k   = output.new_position[0];
        // double ds_k  = output.new_velocity[0];
        // double dds_k = output.new_acceleration[0];
        double lambda_k = s_k / d_pos;

        // 位置线性插补
        Eigen::Vector3d xk = start_pose + lambda_k * delta_X;

        // 姿态 Slerp 插补
        Eigen::Quaterniond qk = slerp_ruckig(start_quat, end_quat, lambda_k);

        std::vector<double>point(7);
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.linear() = qk.toRotationMatrix(); // 设置旋转部分
        pose.translation() = xk; 
    
        point={xk.x(),xk.y(),xk.z(),qk.x(),qk.y(),qk.z(),qk.w()};
        pose_path.push_back(point);
        
        output.pass_to_input(input);
    }
    path.clear();
    std::vector<double> current_full_q = start_q;
    std::vector<double>point_q(7);
    for(size_t i=0;i<pose_path.size();i++){
        Eigen::Vector3d xk(pose_path[i][0],pose_path[i][1],pose_path[i][2]);
        Eigen::Quaterniond qk(pose_path[i][6],pose_path[i][3],pose_path[i][4],pose_path[i][5]);
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.linear() = qk.toRotationMatrix(); // 设置旋转部分
        pose.translation() = xk;
        std::vector<double> q_next;
        const auto ik_started_at = std::chrono::steady_clock::now();
        const bool ik_ok = solveSingleArmCartesianIk(
            is_left, current_full_q, pose, q_next);
        if (ik_time_seconds != nullptr) {
            *ik_time_seconds += std::chrono::duration<double>(
                std::chrono::steady_clock::now() - ik_started_at).count();
        }
        if (!ik_ok) {
            RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "PlanSingleArmCartesianPath: failed to solve IK at %zu point",i+1);
            return false;
        }

        const auto& indices = is_left ? left_arm_q_indices_ : right_arm_q_indices_;
        for (size_t index = 0; index < indices.size(); ++index) {
            current_full_q[indices[index]] = q_next[index];
        }
        path.push_back(q_next);
        
    }
    return true;
}

bool MiracleMAlgo::ruckigPlan(
    double dt,
    const std::vector<double>& max_velocity,
    const std::vector<double>& max_acc,
    const std::vector<double>& max_jerk,
    const std::vector<double>& joint_cur_pos,
    const std::vector<double>& joint_cur_vel,
    const std::vector<double>& joint_cur_acc,
    const std::vector<double>& joint_target_pos,
    std::vector<std::vector<double>>& res_trajectory)
{
    const std::size_t dof = joint_target_pos.size();
    if (dof == 0U || !std::isfinite(dt) || dt <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "ruckigPlan: invalid dt or empty target");
        return false;
    }
    if (joint_cur_pos.size() != dof || joint_cur_vel.size() != dof ||
        joint_cur_acc.size() != dof || max_velocity.size() != dof ||
        max_acc.size() != dof || max_jerk.size() != dof)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MiracleMAlgo"),
            "ruckigPlan: vector size mismatch dof=%zu cur_pos=%zu cur_vel=%zu cur_acc=%zu vel=%zu acc=%zu jerk=%zu",
            dof, joint_cur_pos.size(), joint_cur_vel.size(), joint_cur_acc.size(),
            max_velocity.size(), max_acc.size(), max_jerk.size());
        return false;
    }

    ruckig::InputParameter<ruckig::DynamicDOFs> input(dof);
    for (std::size_t index = 0; index < dof; ++index) {
        input.current_position[index] = joint_cur_pos[index];
        input.current_velocity[index] = joint_cur_vel[index];
        input.current_acceleration[index] = joint_cur_acc[index];
        input.target_position[index] = joint_target_pos[index];
        input.target_velocity[index] = 0.0;
        input.target_acceleration[index] = 0.0;
        input.max_velocity[index] = max_velocity[index];
        input.max_acceleration[index] = max_acc[index];
        input.max_jerk[index] = max_jerk[index];
    }

    ruckig::Ruckig<ruckig::DynamicDOFs> otg(dof);
    ruckig::Trajectory<ruckig::DynamicDOFs> trajectory(dof);
    const ruckig::Result result = otg.calculate(input, trajectory);
    if (result < 0) {
        RCLCPP_ERROR(
            rclcpp::get_logger("MiracleMAlgo"),
            "ruckigPlan: calculate error %d", static_cast<int>(result));
        return false;
    }

    res_trajectory.clear();
    const double total_time = trajectory.get_duration();
    std::vector<double> new_position(dof);
    std::vector<double> new_velocity(dof);
    std::vector<double> new_acceleration(dof);
    const std::size_t total_steps =
        static_cast<std::size_t>(std::max(1, static_cast<int>(std::ceil(total_time / dt))));
    for (std::size_t i = 0; i < total_steps; ++i) {
        const double new_time =
            i + 1U == total_steps ? total_time : static_cast<double>(i) * dt;
        trajectory.at_time(new_time, new_position, new_velocity, new_acceleration);
        res_trajectory.emplace_back(new_position.begin(), new_position.end());
    }
    return true;
}

bool MiracleMAlgo::toppraPlan(
    const std::vector<std::vector<double>>& ompl_path,
    double dt,
    const std::vector<double>& max_velocity,
    const std::vector<double>& max_acc,
    std::vector<std::vector<double>>& res_trajectory,
    ToppraSolverMode solver_mode,
    double maximum_duration_seconds)
{
    const int dof = static_cast<int>(max_velocity.size());
    if (ompl_path.size() < 2 || dof <= 0 || !std::isfinite(dt) || dt <= 0.0 ||
        std::isnan(maximum_duration_seconds) || maximum_duration_seconds <= 0.0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "toppraPlan: invalid path or dof");
        return false;
    }
    if (max_acc.size() != static_cast<std::size_t>(dof)) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "toppraPlan: acceleration size mismatch");
        return false;
    }
    for (int joint = 0; joint < dof; ++joint) {
        if (!std::isfinite(max_velocity[static_cast<std::size_t>(joint)]) ||
            max_velocity[static_cast<std::size_t>(joint)] <= 0.0 ||
            !std::isfinite(max_acc[static_cast<std::size_t>(joint)]) ||
            max_acc[static_cast<std::size_t>(joint)] <= 0.0)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("MiracleMAlgo"),
                "toppraPlan: limits must be finite and positive");
            return false;
        }
    }

    const bool stable_cartesian_mode =
        solver_mode == ToppraSolverMode::kStableCartesian;
    const auto prepared =
        motion_control_pkg::motion::prepareToppraWaypointPath(
        ompl_path, static_cast<std::size_t>(dof),
        stable_cartesian_mode ?
        motion_control_pkg::motion::kToppraDuplicateWaypointToleranceRad : 0.0,
        stable_cartesian_mode);
    if (!prepared.valid) {
        RCLCPP_ERROR(
            rclcpp::get_logger("MiracleMAlgo"),
            "toppraPlan: path preparation failed: %s",
            prepared.error_message.c_str());
        return false;
    }
    
    // 1. waypoint -> toppra::Vectors
    toppra::Vectors pos_list;
    const int nPt = static_cast<int>(prepared.waypoints.size());
    for (const auto& wp : prepared.waypoints)
    {
        toppra::Vector p(dof);
        for (int j = 0; j < dof; ++j)
            p(j) = wp[j];
        pos_list.push_back(p);
    }

    // 路径参数节点
    toppra::Vector breakPts(nPt);
    for (int i = 0; i < nPt; ++i) {
        breakPts(i) = prepared.breakpoints[static_cast<std::size_t>(i)];
    }
    // TOPPRA already enforces zero temporal path speed at both ends.  A
    // clamped geometric spline also forces q'(s)=0 and creates a double-zero
    // endpoint that can collapse the penultimate squared path velocity and
    // inflate the last segment to many hours.  Keep legacy callers unchanged,
    // but use zero-curvature natural boundaries for stable Cartesian MoveL.
    const char* boundary_condition_name = stable_cartesian_mode ?
        motion_control_pkg::motion::kToppraStableCartesianBoundaryCondition :
        "clamped";
    std::array<toppra::BoundaryCond,2> bc{
        toppra::BoundaryCond(boundary_condition_name),
        toppra::BoundaryCond(boundary_condition_name)
    };

    // 三次样条生成路径
    auto path = std::make_shared<toppra::PiecewisePolyPath>(
        toppra::PiecewisePolyPath::CubicSpline(pos_list, breakPts, bc)
    );

    // 2. 约束组装
    toppra::Vector velLimitLower = Eigen::VectorXd::Zero(dof);
    toppra::Vector velLimitUpper = Eigen::VectorXd::Zero(dof);
    toppra::Vector accLimitLower = Eigen::VectorXd::Zero(dof);
    toppra::Vector accLimitUpper = Eigen::VectorXd::Zero(dof);
    for(int j = 0; j < dof; ++j) {
        velLimitLower(j) = -max_velocity[static_cast<std::size_t>(j)];
        velLimitUpper(j) = max_velocity[static_cast<std::size_t>(j)];
        accLimitLower(j) = -max_acc[static_cast<std::size_t>(j)];
        accLimitUpper(j) = max_acc[static_cast<std::size_t>(j)];
    }
    toppra::LinearConstraintPtrs cons;
    cons.emplace_back(std::make_shared<toppra::constraint::LinearJointVelocity>(velLimitLower, velLimitUpper));
    cons.emplace_back(std::make_shared<toppra::constraint::LinearJointAcceleration>(accLimitLower, accLimitUpper));
    for (auto& c : cons)
        c->discretizationType(toppra::DiscretizationType::Interpolation);  //Collocation, Interpolation问题规模更大,精度更高

    // 3. TOPPRA求解
    toppra::algorithm::TOPPRA opt(cons, path);
    // The downloaded libtoppra exports qpOASESWrapper but was built without
    // qpOASES support, so invoking it throws at runtime.  The stable MoveL
    // mode therefore keeps the available Seidel backend and fixes the input
    // pathology (near-duplicate points and ill-conditioned arc-length knots)
    // before the solver is entered.
    const char* solver_name = stable_cartesian_mode ? "SeidelStable" : "Seidel";
    opt.solver(std::make_shared<toppra::solver::Seidel>());

    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "toppraPlan: solve begin solver=%s boundary=%s input_waypoints=%zu retained_waypoints=%zu removed=%zu min_joint_step=%.9f rad max_joint_step=%.9f rad",
        solver_name, boundary_condition_name, ompl_path.size(), prepared.waypoints.size(),
        prepared.removed_waypoints, prepared.minimum_joint_step_rad,
        prepared.maximum_joint_step_rad);

    const auto solve_started_at = std::chrono::steady_clock::now();
    const auto ret = opt.computePathParametrization();
    const double solve_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - solve_started_at).count();
    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "toppraPlan: solve end solver=%s result=%d elapsed=%.6f s",
        solver_name, static_cast<int>(ret), solve_seconds);
    if (ret != toppra::ReturnCode::OK)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "toppraPlan: solve failed");
        return false;
    }

    // 4. 生成参数化
    const auto& data = opt.getParameterizationData();
    const auto path_speed_from_squared = [](double squared_path_velocity) {
        return std::isfinite(squared_path_velocity) && squared_path_velocity >= 0.0 ?
            std::sqrt(squared_path_velocity) :
            std::numeric_limits<double>::quiet_NaN();
    };
    const auto log_gridpoint_diagnostics =
        [&](const char* context, Eigen::Index index) {
            if (index < 0 || index >= data.gridpoints.size() ||
                index >= data.parametrization.size())
            {
                RCLCPP_WARN(
                    rclcpp::get_logger("MiracleMAlgo"),
                    "toppraPlan: gridpoint diagnostics context=%s index=%lld unavailable gridpoints=%lld parametrization=%lld",
                    context, static_cast<long long>(index),
                    static_cast<long long>(data.gridpoints.size()),
                    static_cast<long long>(data.parametrization.size()));
                return;
            }

            const double gridpoint = data.gridpoints(index);
            const double squared_path_velocity = data.parametrization(index);
            const double path_velocity =
                path_speed_from_squared(squared_path_velocity);
            const toppra::Vector q_prime = path->eval_single(gridpoint, 1);
            const toppra::Vector q_double_prime = path->eval_single(gridpoint, 2);
            const bool controllable_available =
                index < data.controllable_sets.rows() &&
                data.controllable_sets.cols() >= 2;
            const double controllable_lower = controllable_available ?
                data.controllable_sets(index, 0) :
                std::numeric_limits<double>::quiet_NaN();
            const double controllable_upper = controllable_available ?
                data.controllable_sets(index, 1) :
                std::numeric_limits<double>::quiet_NaN();

            RCLCPP_INFO(
                rclcpp::get_logger("MiracleMAlgo"),
                "toppraPlan: gridpoint diagnostics context=%s index=%lld gridpoint=%.17g squared_path_velocity=%.17g path_velocity=%.17g q_prime_norm=%.17g q_double_prime_norm=%.17g controllable_set=[%.17g,%.17g]",
                context, static_cast<long long>(index), gridpoint,
                squared_path_velocity, path_velocity, q_prime.norm(),
                q_double_prime.norm(), controllable_lower, controllable_upper);
        };

    Eigen::Index minimum_parametrization_index = -1;
    Eigen::Index minimum_positive_parametrization_index = -1;
    Eigen::Index minimum_interior_parametrization_index = -1;
    double minimum_parametrization = std::numeric_limits<double>::infinity();
    double minimum_positive_parametrization =
        std::numeric_limits<double>::infinity();
    double minimum_interior_parametrization =
        std::numeric_limits<double>::infinity();
    std::size_t nonfinite_parametrization_count = 0U;
    std::size_t negative_parametrization_count = 0U;
    for (Eigen::Index index = 0; index < data.parametrization.size(); ++index) {
        const double value = data.parametrization(index);
        if (!std::isfinite(value)) {
            ++nonfinite_parametrization_count;
            continue;
        }
        if (value < 0.0) {
            ++negative_parametrization_count;
        }
        if (value < minimum_parametrization) {
            minimum_parametrization = value;
            minimum_parametrization_index = index;
        }
        if (value > 0.0 && value < minimum_positive_parametrization) {
            minimum_positive_parametrization = value;
            minimum_positive_parametrization_index = index;
        }
        if (index > 0 && index + 1 < data.parametrization.size() &&
            value < minimum_interior_parametrization)
        {
            minimum_interior_parametrization = value;
            minimum_interior_parametrization_index = index;
        }
    }
    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "toppraPlan: parametrization summary count=%lld min=%.17g min_index=%lld min_positive=%.17g min_positive_index=%lld min_interior=%.17g min_interior_index=%lld nonfinite=%zu negative=%zu",
        static_cast<long long>(data.parametrization.size()),
        minimum_parametrization,
        static_cast<long long>(minimum_parametrization_index),
        minimum_positive_parametrization,
        static_cast<long long>(minimum_positive_parametrization_index),
        minimum_interior_parametrization,
        static_cast<long long>(minimum_interior_parametrization_index),
        nonfinite_parametrization_count, negative_parametrization_count);
    if (minimum_parametrization_index >= 0) {
        const Eigen::Index neighborhood_begin =
            std::max<Eigen::Index>(0, minimum_parametrization_index - 2);
        const Eigen::Index neighborhood_end =
            std::min<Eigen::Index>(data.parametrization.size() - 1,
                minimum_parametrization_index + 2);
        for (Eigen::Index index = neighborhood_begin;
            index <= neighborhood_end; ++index)
        {
            log_gridpoint_diagnostics("minimum_neighborhood", index);
        }
    }
    if (minimum_positive_parametrization_index >= 0 &&
        minimum_positive_parametrization_index != minimum_parametrization_index)
    {
        log_gridpoint_diagnostics(
            "minimum_positive", minimum_positive_parametrization_index);
    }
    if (minimum_interior_parametrization_index >= 0 &&
        minimum_interior_parametrization_index != minimum_parametrization_index &&
        minimum_interior_parametrization_index !=
            minimum_positive_parametrization_index)
    {
        log_gridpoint_diagnostics(
            "minimum_interior", minimum_interior_parametrization_index);
    }
    //在时间域拟合一条新的 q(t) 三次样条（更平滑，但可能偏离原 path）
    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "toppraPlan: spline construction begin gridpoints=%lld parametrization=%lld",
        static_cast<long long>(data.gridpoints.size()),
        static_cast<long long>(data.parametrization.size()));
    const auto spline_started_at = std::chrono::steady_clock::now();
    toppra::parametrizer::Spline param(path, data.gridpoints, data.parametrization);
    const double spline_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - spline_started_at).count();
    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "toppraPlan: spline construction end elapsed=%.6f s", spline_seconds);

    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "toppraPlan: validate begin");
    const auto validate_started_at = std::chrono::steady_clock::now();
    const bool parameterization_valid = param.validate();
    const double validate_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - validate_started_at).count();
    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "toppraPlan: validate end result=%s elapsed=%.6f s",
        parameterization_valid ? "success" : "failure", validate_seconds);
    if (!parameterization_valid) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "toppraPlan: validate failed.");    
        return false;
    }
    // 总时间 & 按dt采样
    const toppra::Vector& raw_time = param.getTimes();
    double totalT = raw_time.tail(1)(0);
    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "toppraPlan: duration totalT=%.9f s raw_time_count=%lld dt=%.9f s",
        totalT, static_cast<long long>(raw_time.size()), dt);

    const Eigen::Index segment_point_count = std::min<Eigen::Index>(
        raw_time.size(),
        std::min<Eigen::Index>(
            data.gridpoints.size(), data.parametrization.size()));
    Eigen::Index maximum_raw_time_delta_segment = -1;
    double maximum_raw_time_delta = -std::numeric_limits<double>::infinity();
    for (Eigen::Index end_index = 1;
        end_index < segment_point_count; ++end_index)
    {
        const Eigen::Index begin_index = end_index - 1;
        const double gridpoint_delta =
            data.gridpoints(end_index) - data.gridpoints(begin_index);
        const double begin_path_velocity = path_speed_from_squared(
            data.parametrization(begin_index));
        const double end_path_velocity = path_speed_from_squared(
            data.parametrization(end_index));
        const double average_path_velocity =
            0.5 * (begin_path_velocity + end_path_velocity);
        const double raw_time_delta =
            raw_time(end_index) - raw_time(begin_index);
        RCLCPP_INFO(
            rclcpp::get_logger("MiracleMAlgo"),
            "toppraPlan: segment begin_index=%lld end_index=%lld gridpoint_begin=%.17g gridpoint_end=%.17g gridpoint_delta=%.17g squared_path_velocity_begin=%.17g squared_path_velocity_end=%.17g average_path_velocity=%.17g deltaT=%.17g",
            static_cast<long long>(begin_index),
            static_cast<long long>(end_index),
            data.gridpoints(begin_index), data.gridpoints(end_index),
            gridpoint_delta, data.parametrization(begin_index),
            data.parametrization(end_index), average_path_velocity,
            raw_time_delta);
        if (std::isfinite(raw_time_delta) &&
            raw_time_delta > maximum_raw_time_delta)
        {
            maximum_raw_time_delta = raw_time_delta;
            maximum_raw_time_delta_segment = begin_index;
        }
    }
    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "toppraPlan: maximum raw_time delta segment=%lld begin_index=%lld end_index=%lld deltaT=%.17g",
        static_cast<long long>(maximum_raw_time_delta_segment),
        static_cast<long long>(maximum_raw_time_delta_segment),
        static_cast<long long>(maximum_raw_time_delta_segment + 1),
        maximum_raw_time_delta);
    if (maximum_raw_time_delta_segment >= 0) {
        log_gridpoint_diagnostics(
            "maximum_raw_time_delta_begin",
            maximum_raw_time_delta_segment);
        log_gridpoint_diagnostics(
            "maximum_raw_time_delta_end",
            maximum_raw_time_delta_segment + 1);
    }

    if (!std::isfinite(totalT) || totalT <= 0.0 ||
        totalT / dt > static_cast<double>(std::numeric_limits<int>::max()))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MiracleMAlgo"),
            "toppraPlan: invalid trajectory duration");
        return false;
    }
    if (std::isfinite(maximum_duration_seconds) &&
        totalT > maximum_duration_seconds)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MiracleMAlgo"),
            "toppraPlan: trajectory duration %.9f s exceeds planning limit %.9f s; refusing %lld samples",
            totalT, maximum_duration_seconds,
            static_cast<long long>(std::ceil(totalT / dt)));
        return false;
    }

    int sample_cnt = static_cast<int>(std::ceil(totalT / dt));
    sample_cnt = std::max(1, sample_cnt);
    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "toppraPlan: sample count=%d totalT=%.9f s dt=%.9f s",
        sample_cnt, totalT, dt);

    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "toppraPlan: sample time grid construction begin samples=%d",
        sample_cnt);
    const auto sample_grid_started_at = std::chrono::steady_clock::now();
    toppra::Vector samp_t(sample_cnt);
    for (int i = 0; i < sample_cnt; ++i)
    {
        samp_t(i) = i + 1 == sample_cnt ? totalT : static_cast<double>(i) * dt;
    }
    const double sample_grid_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - sample_grid_started_at).count();
    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "toppraPlan: sample time grid construction end samples=%d elapsed=%.6f s",
        sample_cnt, sample_grid_seconds);

    // 批量求值 pos/vel/acc
    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "toppraPlan: param.eval begin samples=%d order=0", sample_cnt);
    const auto eval_started_at = std::chrono::steady_clock::now();
    auto pos_buf = param.eval(samp_t, 0);
    const double eval_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - eval_started_at).count();
    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "toppraPlan: param.eval end samples=%zu elapsed=%.6f s",
        pos_buf.size(), eval_seconds);
    // auto vel_buf = param.eval(samp_t, 1);
    // auto acc_buf = param.eval(samp_t, 2);

    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "toppraPlan: trajectory materialization begin samples=%d dof=%d",
        sample_cnt, dof);
    const auto materialization_started_at = std::chrono::steady_clock::now();
    res_trajectory.clear();
    // 5. 填充统一结构体 
    res_trajectory.resize(sample_cnt, std::vector<double>(dof));


    for (int i = 0; i < sample_cnt; ++i)
    {
        for (int j = 0; j < dof; ++j)
        {
            res_trajectory[i][j] = pos_buf[i](j);
            // res.velocities[i][j] = vel_buf[i](j);
            // res.accelerations[i][j] = acc_buf[i](j);
        }
    }
    const double materialization_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - materialization_started_at).count();
    RCLCPP_INFO(
        rclcpp::get_logger("MiracleMAlgo"),
        "toppraPlan: trajectory materialization end samples=%zu elapsed=%.6f s",
        res_trajectory.size(), materialization_seconds);
    return true;
}
//--------------------------------------------------------------//

bool MiracleMAlgo::getChainJointsLimt(int module_id, std::vector<double> &min_angles, std::vector<double> &max_angles) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    const std::vector<size_t> *q_indices = nullptr;
    if (module_id == 0) {
        q_indices = &left_arm_q_indices_;
    } else if (module_id == 1) {
        q_indices = &right_arm_q_indices_;
    } else if (module_id == 3) {
        q_indices = &head_q_indices_;
    } else if (module_id == 4) {
        q_indices = &leg_q_indices_;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "Unsupported module_id: %d", module_id);
        return false;
    }

    if (min_angles.size() != q_indices->size() || max_angles.size() != q_indices->size()) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "Chain joint size mismatch. expected=%zu got=%zu", min_angles.size(), q_indices->size());
        return false;
    }

    for (size_t i = 0; i < q_indices->size(); ++i) {
        const size_t idx = (*q_indices)[i];
        if (idx >= static_cast<size_t>(model_.lowerPositionLimit.size()) || idx >= static_cast<size_t>(model_.upperPositionLimit.size())) {
            RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "Joint limit index out of range. idx=%zu", idx);
            return false;
        }
        min_angles[i] = model_.lowerPositionLimit[static_cast<Eigen::Index>(idx)];
        max_angles[i] = model_.upperPositionLimit[static_cast<Eigen::Index>(idx)];
    }

    return true;
}

std::vector<double> MiracleMAlgo::getLegJoints() {
    // std::vector<double> joint(LEG_JOINTNUM, 0.0);
    // if (!_m_lower_body_joint_state_msg.empty()) {
    return _m_lower_body_joint_state_msg;
    // } else {
    //     return joint;
    // }
}

int MiracleMAlgo::updateChainJointsAngle(
    const std::string &module_name,
    const std::vector<double> &cur_joint_angles) {
    
    std::vector<double> *msg = nullptr;
    const std::vector<size_t> *q_indices = nullptr;
    if (module_name == "left_arm") {
        msg = &_m_left_arm_joint_state_msg;
        q_indices = &left_arm_q_indices_;
    } else if (module_name == "right_arm") {
        msg = &_m_right_arm_joint_state_msg;
        q_indices = &right_arm_q_indices_;
    } else if (module_name == "leg") {
        msg = &_m_lower_body_joint_state_msg; 
        q_indices = &leg_q_indices_;
    } else if (module_name == "base") {
        msg = &_m_chassis_joint_state_msg;
    } else if (module_name == "head") {
        msg = &_m_head_joint_state_msg;
        q_indices = &head_q_indices_;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("MiracleMAlgo"), "Unknown chain name: %s", module_name.c_str());
        return -1;
    }

    if(msg == nullptr){
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "Internal error: msg pointer is null");
        return -1;
    }
    if (q_indices) {
        if (q_indices->size() != cur_joint_angles.size()) {
            RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "Joint size mismatch for module %s: state=%zu, indices=%zu",
                         module_name.c_str(), cur_joint_angles.size(), q_indices->size());
            return -1;
        }
    }
    for (const double angle : cur_joint_angles) {
        if (!std::isfinite(angle)) {
            RCLCPP_ERROR(
                rclcpp::get_logger("MiracleMAlgo"),
                "Non-finite joint angle for module %s", module_name.c_str());
            return -1;
        }
    }

    std::lock_guard<std::mutex> lock(upadate_joinst_state_msg);
    if (q_indices) {
        if (_m_full_joint_state_msg.size() != static_cast<size_t>(model_.nq)) {
            _m_full_joint_state_msg.assign(default_q_.data(), default_q_.data() + default_q_.size());
        }
        for (size_t i = 0; i < q_indices->size(); ++i) {
            const size_t idx = (*q_indices)[i];
            if (idx >= _m_full_joint_state_msg.size()) {
                RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "Full joint state index out of range for module %s: idx=%zu size=%zu",
                             module_name.c_str(), idx, _m_full_joint_state_msg.size());
                return -1;
            }
            _m_full_joint_state_msg[idx] = cur_joint_angles[i];
        }
    }
    msg->assign(cur_joint_angles.begin(), cur_joint_angles.end());

    return 0;
}


int MiracleMAlgo::quadraticInterpolation(const std::vector<std::vector<double>> &raw_pos, std::vector<std::vector<double>> &pos, double plan_dt, double interp_dt) {

    if (raw_pos.size() < 2) {
        return -1; //至少需要2个点才能进行插值
    }
    if (plan_dt <= 0.0) {
        return -2;
    }
    pos.clear();

    const size_t total_pos_cnt = raw_pos.size();
    const size_t num_joints = raw_pos[0].size();
    for (size_t i = 0; i < total_pos_cnt; ++i) {
        if (raw_pos[i].size() != num_joints) {
            return -3;
        }
    }

    if(plan_dt <= interp_dt) {

        pos.reserve(total_pos_cnt);

        for (size_t i = 0; i < total_pos_cnt; ++i) {
            pos.push_back(raw_pos[i]);
        }

        RCLCPP_INFO(rclcpp::get_logger("m algo"), "plan_dt is less than interp_dt plan_dt: %f, interp_dt: %f", plan_dt, interp_dt);

        return 0;

    }
    //预留空间
    // const double interp_dt = 1.0/TRAJ_FREQ; // 默认 4ms
    size_t estimated_points = static_cast<size_t>((total_pos_cnt - 1) * plan_dt / interp_dt) + 1;
    pos.reserve(estimated_points);

    std::vector<std::vector<double>> velocities(total_pos_cnt, std::vector<double>(num_joints));

    RCLCPP_INFO(rclcpp::get_logger("m algo"), "raw_pos points number is: %ld, estimated_points number is: %ld", total_pos_cnt, estimated_points);

    // 中心差分法
    for (size_t i = 1; i < total_pos_cnt - 1; ++i) {
        for (size_t j = 0; j < num_joints; ++j) {
            // v_i = (p_{i+1} - p_{i-1}) / (2 * dt)
            velocities[i][j] = (raw_pos[i + 1][j] - raw_pos[i - 1][j]) / (2.0 * plan_dt);
        }
    }

    // 第一个点,前向差分
    for (size_t j = 0; j < num_joints; ++j) {
        // v_0 = (p_1 - p_0) / dt
        velocities[0][j] = (raw_pos[1][j] - raw_pos[0][j]) / plan_dt;
    }

    //最后一个点,后向差分法
    size_t last_idx = total_pos_cnt - 1;
    for (size_t j = 0; j < num_joints; ++j) {
        // v_n = (p_n - p_{n-1}) / dt
        velocities[last_idx][j] = (raw_pos[last_idx][j] - raw_pos[last_idx - 1][j]) / plan_dt;
    }

    for (size_t i = 0; i < total_pos_cnt - 1; ++i) {
        const std::vector<double>& p0 = raw_pos[i];      // 段的起始位置
        const std::vector<double>& v0 = velocities[i];   // 段的起始速度
        const std::vector<double>& v1 = velocities[i+1]; // 段的结束速度

        // a = (v_end - v_start) / dt
        std::vector<double> a(num_joints);
        for (size_t j = 0; j < num_joints; ++j) {
            a[j] = (v1[j] - v0[j]) / plan_dt;
        }

        // 在 [0, plan_dt] 区间内以 interp_dt (4ms) 为步长进行插值
        for (double t = 0.0; t < plan_dt; t += interp_dt) {
            std::vector<double> interpolated_point(num_joints);
            for (size_t j = 0; j < num_joints; ++j) {
                // p(t) = p0 + v0*t + 0.5*a*t^2
                interpolated_point[j] = p0[j] + v0[j] * t + 0.5 * a[j] * t * t;
            }
            pos.push_back(interpolated_point);
        }
    }

    //添加最后一个点
    pos.push_back(raw_pos[last_idx]);

    return 0; 
}

void MiracleMAlgo::savePathToTxt(const std::vector<std::vector<double> >& out_path, const std::string& filename)
{
    if (out_path.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("DualArmModule"), "Path is empty, nothing to save.");
        return;
    }

    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("DualArmModule"), "Failed to open file: %s", filename.c_str());
        return;
    }

    // 设置浮点数格式：固定小数点，保留 6 位小数
    outFile << std::fixed << std::setprecision(6);

    // 获取自由度 (假设所有路径点维度一致)
    int dof = static_cast<int>(out_path[0].size());

    for (size_t i = 0; i < out_path.size(); ++i) {
        const std::vector<double>& q = out_path[i];
        
        // 安全检查：确保维度一致
        if (static_cast<int>(q.size()) != dof) {
            RCLCPP_WARN(rclcpp::get_logger("DualArmModule"), 
                        "Dimension mismatch at waypoint %zu. Skipping.", i);
            continue;
        }

        for (int j = 0; j < dof; ++j) {
            outFile << q[j];
            // 如果不是该行的最后一个元素，则添加逗号
            if (j < dof - 1) {
                outFile << ","; 
            }
        }
        // 【关键】每个轨迹点（每一行）结束后换行
        outFile << std::endl;
    }

    outFile.close();
    RCLCPP_INFO(rclcpp::get_logger("DualArmModule"), 
                "Path saved to %s (%zu waypoints).", filename.c_str(), out_path.size());
}
