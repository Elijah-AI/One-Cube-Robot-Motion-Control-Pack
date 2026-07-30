#include "toppra_planner.h"
#include "motion_control_pkg/execution/trajectory_sampling.hpp"
#include <toppra/algorithm.hpp>
#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/parametrizer/const_accel.hpp>
//#ifdef BUILD_WITH_qpOASES
#include <toppra/solver/qpOASES-wrapper.hpp>
//#endif
#ifdef BUILD_WITH_GLPK
#include <toppra/solver/glpk-wrapper.hpp>
#endif
#include <toppra/solver/seidel.hpp>
#include <toppra/toppra.hpp>
#include <toppra/parametrizer/spline.hpp>
#include <cmath>
#include <iostream>

bool ToppraPlanner::init(int dof, const MotionConstraint& constraint)
{
    if (dof <= 0 || !std::isfinite(dt_) || dt_ <= 0.0 ||
        constraint.max_vel.size() != static_cast<std::size_t>(dof) ||
        constraint.max_acc.size() != static_cast<std::size_t>(dof)) {
        std::cerr << "[Toppra] invalid DOF, cycle time or constraints\n";
        return false;
    }
    dof_ = dof;
    constraint_ = constraint;

    // 速度/加速度转Eigen，max_vel取正负，jerk toppra不使用直接丢弃
    vel_min_.resize(dof);
    vel_max_.resize(dof);
    acc_min_.resize(dof);
    acc_max_.resize(dof);

    for (int i = 0; i < dof; ++i)
    {
        vel_min_(i) = -constraint_.max_vel[i];
        vel_max_(i) =  constraint_.max_vel[i];
        acc_min_(i) = -constraint_.max_acc[i];
        acc_max_(i) =  constraint_.max_acc[i];
    }
    return true;
}

TrajectoryResult ToppraPlanner::plan(const std::vector<Waypoint>& waypoints)
{
    TrajectoryResult res;
    if (waypoints.size() < 2 || dof_ <= 0)
    {
        std::cerr << "[Toppra] waypoint count <2 or invalid dof\n";
        return res;
    }

    // 1. waypoint -> toppra::Vectors
    toppra::Vectors pos_list;
    int nPt = waypoints.size();
    for (const auto& wp : waypoints)
    {
        if (wp.positions.size() != static_cast<std::size_t>(dof_)) {
            std::cerr << "[Toppra] waypoint dimension mismatch\n";
            return res;
        }
        toppra::Vector p(dof_);
        for (int j = 0; j < dof_; ++j)
            p(j) = wp.positions[j];
        pos_list.push_back(p);
    }

    // 归一化断点 [0,1]
    toppra::Vector breakPts(nPt);
    for (int i = 0; i < nPt; ++i)
        breakPts(i) = static_cast<double>(i) / (nPt - 1);

    // clamped 起止零速度
    std::array<toppra::BoundaryCond,2> bc{
        toppra::BoundaryCond("clamped"),
        toppra::BoundaryCond("clamped")
    };

    // 三次样条生成路径
    auto path = std::make_shared<toppra::PiecewisePolyPath>(
        toppra::PiecewisePolyPath::CubicSpline(pos_list, breakPts, bc)
    );

    // 2. 约束组装
    toppra::LinearConstraintPtrs cons;
    cons.emplace_back(std::make_shared<toppra::constraint::LinearJointVelocity>(vel_min_, vel_max_));
    cons.emplace_back(std::make_shared<toppra::constraint::LinearJointAcceleration>(acc_min_, acc_max_));
    for (auto& c : cons)
        c->discretizationType(toppra::DiscretizationType::Collocation);

    // 3. TOPPRA求解
    toppra::algorithm::TOPPRA opt(cons, path);
    opt.setN(grid_num_);//默认为100
    auto solver = std::make_shared<toppra::solver::Seidel>();
    opt.solver(solver);

    auto ret = opt.computePathParametrization();
    if (ret != toppra::ReturnCode::OK)
    {
        std::cerr << "[Toppra] solve failed\n";
        return res;
    }

    // 4. 生成参数化
    const auto& data = opt.getParameterizationData();
    toppra::parametrizer::Spline param(path, data.gridpoints, data.controllable_sets.col(1));
    if (!param.validate())
        return res;

    // 总时间 & 按dt采样
    const toppra::Vector& raw_time = param.getTimes();
    double totalT = raw_time.tail(1)(0);
    const auto sample_times =
        motion_control_pkg::execution::makeInclusiveSampleTimes(
        totalT, dt_);
    if (sample_times.empty()) {
        std::cerr << "[Toppra] failed to create 4 ms sample times\n";
        return res;
    }
    const int sample_cnt = static_cast<int>(sample_times.size());
    toppra::Vector samp_t(sample_cnt);
    for (int i = 0; i < sample_cnt; ++i) {
        samp_t(i) = sample_times[static_cast<std::size_t>(i)];
    }

    // 批量求值 pos/vel/acc
    auto pos_buf = param.eval(samp_t, 0);
    auto vel_buf = param.eval(samp_t, 1);
    auto acc_buf = param.eval(samp_t, 2);

    // 5. 填充统一结构体 TrajectoryResult
    res.success = true;
    res.total_time = totalT;
    res.times.resize(sample_cnt);
    res.positions.resize(sample_cnt, std::vector<double>(dof_));
    res.velocities.resize(sample_cnt, std::vector<double>(dof_));
    res.accelerations.resize(sample_cnt, std::vector<double>(dof_));
    // Toppra无jerk，全部填0
    res.jerks.resize(sample_cnt, std::vector<double>(dof_, 0.0));

    for (int i = 0; i < sample_cnt; ++i)
    {
        res.times[i] = samp_t(i);
        for (int j = 0; j < dof_; ++j)
        {
            res.positions[i][j] = pos_buf[i](j);
            res.velocities[i][j] = vel_buf[i](j);
            res.accelerations[i][j] = acc_buf[i](j);
        }
    }
    // 多点轨迹同样显式保留输入路径的最终目标点。
    res.positions.back() = waypoints.back().positions;
    return res;
}
