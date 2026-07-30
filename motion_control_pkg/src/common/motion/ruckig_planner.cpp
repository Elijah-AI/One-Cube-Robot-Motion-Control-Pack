#include "ruckig_planner.h"
#include "motion_control_pkg/execution/trajectory_sampling.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

bool RuckigPlanner::init(int dof, const MotionConstraint& constraint)
{
    if (dof <= 0 || !std::isfinite(dt_) || dt_ <= 0.0) {
        std::cerr << "[Ruckig] Invalid DOF or cycle time: "
                  << dof << ", dt=" << dt_ << std::endl;
        return false;
    }
    if (constraint.max_vel.size() != static_cast<std::size_t>(dof) ||
        constraint.max_acc.size() != static_cast<std::size_t>(dof) ||
        constraint.max_jerk.size() != static_cast<std::size_t>(dof)) {
        std::cerr << "[Ruckig] Constraint dimension mismatch." << std::endl;
        return false;
    }

    dof_ = dof;
    constraint_ = constraint;

    try {
        // 1. 初始化 Input/Output/Ruckig 对象，必须传入 dof
        in_ = std::make_unique<ruckig::InputParameter<ruckig::DynamicDOFs>>(dof);
        out_ = std::make_unique<ruckig::OutputParameter<ruckig::DynamicDOFs>>(dof);
        
        // 初始化 Ruckig 实例，传入 dof 和 dt
        rk_ = std::make_unique<ruckig::Ruckig<ruckig::DynamicDOFs>>(dof, dt_);
    } catch (const std::exception& e) {
        std::cerr << "[Ruckig] Initialization failed: " << e.what() << std::endl;
        return false;
    }

    // 2. 设置约束和初始状态
    for (int i = 0; i < dof; ++i) {
        in_->max_velocity[i] = constraint.max_vel[i];
        in_->max_acceleration[i] = constraint.max_acc[i];
        in_->max_jerk[i] = constraint.max_jerk[i];
        
        // 初始状态归零
        in_->current_position[i] = 0.0;
        in_->current_velocity[i] = 0.0;
        in_->current_acceleration[i] = 0.0;
        
        // 目标状态归零
        in_->target_position[i] = 0.0;
        in_->target_velocity[i] = 0.0;
        in_->target_acceleration[i] = 0.0;
    }

    return true;
}

TrajectoryResult RuckigPlanner::plan(const std::vector<Waypoint>& waypoints)
{
    TrajectoryResult res;
    res.success = false;
    
    if (!rk_ || !in_ || !out_) {
        std::cerr << "[Ruckig] Error: Planner not initialized (null pointers)." << std::endl;
        return res;
    }

    // Ruckig 在双臂架构中只负责单个点到点目标。OMPL 等多点路径必须交给
    // TOPPRA，禁止在这里逐段拼接并产生边界重复采样。
    if (waypoints.size() != 2) {
        std::cerr << "[Ruckig] Error: Point-to-point planning requires exactly "
                  << "2 waypoints." << std::endl;
        return res;
    }
    
    for (const auto& wp : waypoints) {
        if (wp.positions.size() != static_cast<size_t>(dof_)) {
            std::cerr << "[Ruckig] Error: Waypoint dimension mismatch. Expected " 
                      << dof_ << ", got " << wp.positions.size() << std::endl;
            return res;
        }
    }
  
    for (int i = 0; i < dof_; ++i) {
        in_->current_position[i] = waypoints.front().positions[i];
        in_->current_velocity[i] = 0.0;
        in_->current_acceleration[i] = 0.0;
        in_->target_position[i] = waypoints.back().positions[i];
        in_->target_velocity[i] = 0.0;
        in_->target_acceleration[i] = 0.0;
    }

    const ruckig::Result ruckig_result =
        rk_->calculate(*in_, out_->trajectory);
    if (ruckig_result != ruckig::Result::Working &&
        ruckig_result != ruckig::Result::Finished) {
        std::cerr << "[Ruckig] Calculate failed, code: "
                  << int(ruckig_result) << std::endl;
        return res;
    }

    const double total_time = out_->trajectory.get_duration();
    const auto sample_times =
        motion_control_pkg::execution::makeInclusiveSampleTimes(
        total_time, dt_);
    if (sample_times.empty()) {
        std::cerr << "[Ruckig] Failed to create 4 ms sample times." << std::endl;
        return res;
    }

    res.total_time = total_time;
    res.times = sample_times;
    res.positions.resize(sample_times.size(), std::vector<double>(dof_));
    res.velocities.resize(sample_times.size(), std::vector<double>(dof_));
    res.accelerations.resize(sample_times.size(), std::vector<double>(dof_));
    res.jerks.resize(
        sample_times.size(), std::vector<double>(dof_, 0.0));

    for (std::size_t index = 0; index < sample_times.size(); ++index) {
        out_->trajectory.at_time(
            sample_times[index], res.positions[index],
            res.velocities[index], res.accelerations[index]);
    }

    // 明确把唯一的最后一个采样点固定为请求目标，避免浮点求值造成终点漂移。
    res.positions.back() = waypoints.back().positions;
    res.success = true;
    return res;
}
