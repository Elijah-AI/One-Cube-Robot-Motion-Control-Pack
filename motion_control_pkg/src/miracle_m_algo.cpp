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
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/collision/collision.hpp>
#include <coal/collision.h>
#include <coal/shape/geometric_shapes.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include "sensor_msgs/msg/joint_state.hpp"

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
    if (full_q.size() != model_.nq) {
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

    pinocchio::updateGeometryPlacements(model_, *data_, geom_model_, *geom_data_, full_q);
    if (pinocchio::computeCollisions(geom_model_, *geom_data_, false)) {
        std::cout << "[isFullQValid] Collision detected. Pairs:" << std::endl;
        for (size_t k = 0; k < geom_model_.collisionPairs.size(); ++k) {
            if (geom_data_->collisionResults[k].isCollision()) {
                const auto& pair = geom_model_.collisionPairs[k];
                std::cout << "  "
                          << geom_model_.geometryObjects[pair.first].name
                          << " <--> "
                          << geom_model_.geometryObjects[pair.second].name
                          << std::endl;
            }
        }
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
        /*
        for(size_t k = 0; k < geom_model_.collisionPairs.size(); ++k) {
            if(geom_data_->collisionResults[k].isCollision()) {
                std::cout << "Collision during planning: "
                          << geom_model_.geometryObjects[geom_model_.collisionPairs[k].first].name << " <--> "
                          << geom_model_.geometryObjects[geom_model_.collisionPairs[k].second].name << std::endl;
            }
        }
        */
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



// --- Inverse Kinematics ---
bool MiracleMAlgo::solveIK(const Eigen::VectorXd& q_init,
                              const Eigen::Isometry3d& goal,
                              Eigen::VectorXd& q_res,
                              int mode) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    double eps = 1e-5;
    const int IT_MAX = 2000;
    const double DT = 0.5;
    const double damp = 1e-4;

    pinocchio::SE3 oMdes(goal.rotation(), goal.translation());

    // RCLCPP_INFO_STREAM(rclcpp::get_logger("MiracleMAlgo"),
    //     "[SolveIK] oMdes translation: " << oMdes.translation().transpose()
    //     << "\n[SolveIK] oMdes rotation:\n" << oMdes.rotation());

    pinocchio::FrameIndex ee_id = 0;
    const std::vector<int>* active_v = nullptr;
    const char* mode_name = "";
    if (mode == 0) {
        ee_id = left_ee_frame_id_;
        active_v = &left_arm_v_indices_;
        mode_name = "Left arm";
    } else if (mode == 1) {
        ee_id = right_ee_frame_id_;
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
    Eigen::MatrixXd J_tmp(6, model_.nv);

    bool success = false;

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


    for (int i=0; i<IT_MAX; i++) {
        // pinocchio::forwardKinematics(model_, *data_, q_res);
        // pinocchio::updateFramePlacements(model_, *data_);
        pinocchio::computeJointJacobians(model_, *data_, q_res);
        pinocchio::updateFramePlacements(model_, *data_);
        
        const pinocchio::SE3 iMd = data_->oMf[ee_id].actInv(oMdes);
        Eigen::VectorXd err = pinocchio::log6(iMd).toVector();
        //打印误差
        // RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"), "[SolveIK] err norm: %f", err.norm());

        if (err.norm() < eps) {
            success = true;
            RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"), "solveIK: %s IK converged in %d iterations. Error norm: %f", mode_name, i, err.norm());
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

        Eigen::MatrixXd JJt = J_tmp * J_tmp.transpose();
        JJt.diagonal().array() += damp;
        
        Eigen::VectorXd v = J_tmp.transpose() * JJt.ldlt().solve(err);
        q_res = pinocchio::integrate(model_, q_res, v * DT);
        
        // Ensure q_res stays within limits(假设只有1自由度简单关节。如果有浮动基，不能这样遍历!)
        for (int j = 0; j < model_.nq; ++j) {
            if (q_res[j] < model_.lowerPositionLimit[j]) {
                q_res[j] = model_.lowerPositionLimit[j];
                // RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"),"%s Joint %d is below lower limit", __func__, j);
            }
            if (q_res[j] > model_.upperPositionLimit[j]){
                q_res[j] = model_.upperPositionLimit[j];
                // RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"),"%s Joint %d is above upper limit", __func__, j);
            } 
        }
    }
    
    if (!success) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "solveIK: failed to converge after %d iterations.", IT_MAX);
    }
    return success;
}

bool MiracleMAlgo::solveDualIK(const Eigen::VectorXd& q_init,
                                  const Eigen::Isometry3d& left_goal,
                                  const Eigen::Isometry3d& right_goal,
                                  Eigen::VectorXd& q_res) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    const double eps = 1e-5;
    const int IT_MAX = 2000;
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
    Eigen::MatrixXd J_left(6, model_.nv);
    Eigen::MatrixXd J_right(6, model_.nv);

    bool success = false;

    for (int i=0; i<IT_MAX; i++) {
        pinocchio::forwardKinematics(model_, *data_, q_res);
        pinocchio::updateFramePlacements(model_, *data_);

        const pinocchio::SE3 iMd_left = data_->oMf[left_ee_frame_id_].actInv(oMdes_left);
        const pinocchio::SE3 iMd_right = data_->oMf[right_ee_frame_id_].actInv(oMdes_right);
        Eigen::VectorXd err_left = pinocchio::log6(iMd_left).toVector();
        Eigen::VectorXd err_right = pinocchio::log6(iMd_right).toVector();
        if (err_left.norm() < eps && err_right.norm() < eps) {
            success = true;
            RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"), "solveDualIK: converged in %d iterations. Left err: %f, Right err: %f", i, err_left.norm(), err_right.norm());
            break;
        }

        J_left.setZero();
        J_right.setZero();
        pinocchio::computeFrameJacobian(model_, *data_, q_res, left_ee_frame_id_, pinocchio::LOCAL, J_left);
        pinocchio::computeFrameJacobian(model_, *data_, q_res, right_ee_frame_id_, pinocchio::LOCAL, J_right);

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

        Eigen::VectorXd v = J.transpose() * JJt.ldlt().solve(err);
        q_res = pinocchio::integrate(model_, q_res, v * DT);

        for (int j = 0; j < model_.nq; ++j) {
            if (q_res[j] < model_.lowerPositionLimit[j]) q_res[j] = model_.lowerPositionLimit[j];
            if (q_res[j] > model_.upperPositionLimit[j]) q_res[j] = model_.upperPositionLimit[j];
        }
    }

    if (!success) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "solveDualIK: failed to converge after %d iterations.", IT_MAX);
    }
    //打印出ik结果，横置打印,只打印左右臂的关节角度
    RCLCPP_INFO_STREAM(rclcpp::get_logger("MiracleMAlgo"), "[SolveIK] Left arm IK result: " << q_res(left_arm_q_indices_).transpose());
    //std::cout << "[SolveIK] Right arm IK result: " << q_res(right_arm_q_indices_).transpose() << std::endl;
    return success;
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
                                                 std::vector<std::vector<double>>& path) {
    std::lock_guard<std::recursive_mutex> data_lock(pinocchio_data_mutex_);
    if (start_q.size() != static_cast<size_t>(model_.nq)) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"),
                     "planSingleArmCartesianGoal: unsupported start_q size: %zu, expected %d",
                     start_q.size(), model_.nq);
        return false;
    }
    Eigen::VectorXd q_init = toEigenConfig(start_q);
    Eigen::VectorXd q_res;
    if(!solveIK(q_init, goal, q_res, is_left ? 0 : 1)) return false;
    
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
                                                    std::vector<std::vector<double>>& path)
{
    Eigen::VectorXd q_current = toEigenConfig(start_q);
    pinocchio::forwardKinematics(model_, *data_, q_current);
    pinocchio::updateFramePlacements(model_, *data_);
    
    pinocchio::FrameIndex ee_id = is_left ? left_ee_frame_id_ : right_ee_frame_id_;
    pinocchio::SE3 start_poseture = data_->oMf[ee_id];
    Eigen::Vector3d start_pose=start_poseture.translation();
    Eigen::Vector3d end_pose(end_poseture[0], end_poseture[1], end_poseture[2]);
    RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"), "end pose: %f,%f,%f", 
                                    end_poseture[0],end_poseture[1],end_poseture[2]);
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

    std::vector<double> current_full_q = start_q;
    std::vector<double>point_q(7);
    for(size_t i=0;i<pose_path.size();i++){
        RCLCPP_INFO(rclcpp::get_logger("MiracleMAlgo"), "path i: %f,%f,%f,%f,%f,%f,%f", 
                                    pose_path[i][0],pose_path[i][1],pose_path[i][2],pose_path[i][3],pose_path[i][4],pose_path[i][5],pose_path[i][6]);
        Eigen::Vector3d xk(pose_path[i][0],pose_path[i][1],pose_path[i][2]);
        Eigen::Quaterniond qk(pose_path[i][6],pose_path[i][3],pose_path[i][4],pose_path[i][5]);
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.linear() = qk.toRotationMatrix(); // 设置旋转部分
        pose.translation() = xk;
        std::vector<double> q_next;
        if(!solveSingleArmCartesianIk(is_left, current_full_q, pose, q_next)) {
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
    std::vector<std::vector<double>>& res_trajectory)
{
    const int dof = static_cast<int>(max_velocity.size());
    if (ompl_path.size() < 2 || dof <= 0 || !std::isfinite(dt) || dt <= 0.0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "toppraPlan: invalid path or dof");
        return false;
    }
    if (max_acc.size() != static_cast<std::size_t>(dof)) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "toppraPlan: acceleration size mismatch");
        return false;
    }
    for (const auto& wp : ompl_path) {
        if (wp.size() != static_cast<std::size_t>(dof)) {
            RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "toppraPlan: waypoint size mismatch");
            return false;
        }
    }
    
    // 1. waypoint -> toppra::Vectors
    toppra::Vectors pos_list;
    int nPt = ompl_path.size();
    for (const auto& wp : ompl_path)
    {
        toppra::Vector p(dof);
        for (int j = 0; j < dof; ++j)
            p(j) = wp[j];
        pos_list.push_back(p);
    }

    // 路径参数节点
    toppra::Vector breakPts(nPt);
    breakPts(0) = 0.0;
    for (int i = 1; i < nPt; ++i) {
        breakPts(i) = breakPts(i - 1) + (pos_list[i] - pos_list[i - 1]).norm();
    }
    // clamped 起止零速度,一阶为0
    std::array<toppra::BoundaryCond,2> bc{
        toppra::BoundaryCond("clamped"),
        toppra::BoundaryCond("clamped")
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
    // opt.setN(500);  求解时间急剧上升
    // auto solver = std::make_shared<toppra::solver::qpOASESWrapper>();   //二次规划求解速度更快
    auto solver = std::make_shared<toppra::solver::Seidel>();
    opt.solver(solver);

    auto ret = opt.computePathParametrization();
    if (ret != toppra::ReturnCode::OK)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "toppraPlan: solve failed");
        return false;
    }

    // 4. 生成参数化
    const auto& data = opt.getParameterizationData();
    //在时间域拟合一条新的 q(t) 三次样条（更平滑，但可能偏离原 path）
    toppra::parametrizer::Spline param(path, data.gridpoints, data.parametrization);
    if (!param.validate()) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "toppraPlan: validate failed.");    
        return false;
    }
    // 总时间 & 按dt采样
    const toppra::Vector& raw_time = param.getTimes();
    double totalT = raw_time.tail(1)(0);

    int sample_cnt = static_cast<int>(std::ceil(totalT / dt));
    sample_cnt = std::max(1, sample_cnt);

    toppra::Vector samp_t(sample_cnt);
    for (int i = 0; i < sample_cnt; ++i)
    {
        samp_t(i) = i + 1 == sample_cnt ? totalT : static_cast<double>(i) * dt;
    }

    // 批量求值 pos/vel/acc
    auto pos_buf = param.eval(samp_t, 0);
    // auto vel_buf = param.eval(samp_t, 1);
    // auto acc_buf = param.eval(samp_t, 2);

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
