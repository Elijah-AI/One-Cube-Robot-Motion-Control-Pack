#include "ruckig_planner.h"
#include <cmath>
#include <algorithm>
#include <iostream>

bool RuckigPlanner::init(int dof, const MotionConstraint& constraint)
{
    if (dof <= 0) {
        std::cerr << "[Ruckig] Invalid DOF: " << dof << std::endl;
        return false;
    }

    dof_ = dof;

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

    if (waypoints.size() < 2) {
        std::cerr << "[Ruckig] Error: Need at least 2 waypoints." << std::endl;
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
     
        in_->current_position[i] = waypoints[0].positions[i];
    }

    double total_t = 0.0;  

    using TrajPoint = std::tuple<double, std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>>;
    std::vector<TrajPoint> seg_buf;

    for (size_t seg_idx = 1; seg_idx < waypoints.size(); seg_idx++)
    {
        // 1. 设置本段目标位置
        for (int i = 0; i < dof_; ++i) {
            in_->target_position[i] = waypoints[seg_idx].positions[i];
        }

        // 2. 计算轨迹
        ruckig::Result res_rk = rk_->calculate(*in_, out_->trajectory);
        

        if (res_rk != ruckig::Result::Working && res_rk != ruckig::Result::Finished) {
            std::cerr << "[Ruckig] Calculate failed at seg " << seg_idx 
                      << ", code: " << int(res_rk) << std::endl;
            return res; 
        }

        double seg_dur = out_->trajectory.get_duration();
        
        if (seg_dur < 1e-6) {
            // 如果时间为0，跳过或仅添加终点
            continue;
        }

        // 3. 采样轨迹
        // 采样点数 = 持续时间 / dt + 1
        int seg_sample = static_cast<int>(std::ceil(seg_dur / dt_)) + 1;

        for (int s = 0; s < seg_sample; ++s)
        {
            double t = s * dt_;
            if (t > seg_dur) t = seg_dur;

            std::vector<double> q(dof_);
            std::vector<double> dq(dof_);
            std::vector<double> ddq(dof_);
            std::vector<double> dddq(dof_);
            
            size_t section = 0;
            
            // 获取状态
            out_->trajectory.at_time(t, q, dq, ddq, dddq, section);

            seg_buf.emplace_back(total_t + t, q, dq, ddq, dddq);
        }

        total_t += seg_dur;

        // 4. 状态接力：更新下一段的初始状态
        // 【修正4】移除 current_jerk，使用 4 参数版本更新 pos, vel, acc
        size_t end_section = 0;
        out_->trajectory.at_time(seg_dur, 
                                 in_->current_position, 
                                 in_->current_velocity, 
                                 in_->current_acceleration);
    }

    // 5. 构建返回结果
    res.success = true;
    res.total_time = total_t;
    
    // 将 seg_buf 转换为你的 TrajectoryResult 格式
    // 假设 TrajectoryResult 有 std::vector<std::vector<double>> positions, velocities...
    double dt = dt_;

    int sample_cnt = static_cast<int>(std::ceil(total_t / dt)) + 1;
    sample_cnt = std::max(1, sample_cnt);
    std::vector<double> samp_t(sample_cnt);
    for (int i = 0; i < sample_cnt; ++i)
    {
        double t = i * dt;
        if (t > total_t) t = total_t;
        samp_t[i] = t;
    }
    
    size_t num_points = seg_buf.size();
    if (num_points > 0) {
        res.positions.resize(num_points);
        res.velocities.resize(num_points);
        res.accelerations.resize(num_points);
        res.jerks.resize(num_points); // 如果有 jerk 字段
        res.times.resize(num_points);
        for (size_t i = 0; i < num_points; ++i) {
            res.positions[i] = std::get<1>(seg_buf[i]);       
            res.velocities[i] = std::get<2>(seg_buf[i]);
            res.accelerations[i] = std::get<3>(seg_buf[i]);
            res.jerks[i] = std::get<4>(seg_buf[i]);
            res.times[i] = samp_t[i];
        }
    }
   
    return res;
}

