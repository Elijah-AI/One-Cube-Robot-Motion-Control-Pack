#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "traj_common_util.h"
#include "motion_control_pkg/execution/control_period.hpp"
#include <vector>
#include <memory>

// 规划器类型
enum class PlannerType
{
    TOPPRA,      // 时间最优（离线）
    RUCKIG       // Jerk最优（在线/实时）
};

// 运动约束（两者通用）
struct MotionConstraint
{
    std::vector<double> max_vel;    // 最大速度
    std::vector<double> max_acc;    // 最大加速度
    std::vector<double> max_jerk;   // 最大jerk（Ruckig用，Toppra自动忽略）
};

// 轨迹结果（统一输出）
struct TrajectoryResult
{
    bool success = false;           // 是否成功
    double total_time = 0.0;        // 总时间
    std::vector<double> times;      // 时间序列
    std::vector<std::vector<double>> positions;  // 位置
    std::vector<std::vector<double>> velocities; // 速度
    std::vector<std::vector<double>> accelerations; // 加速度
    std::vector<std::vector<double>> jerks; // 加加速度（Ruckig）
};

// 统一轨迹规划基类
class TrajectoryPlanner
{
public:
    virtual ~TrajectoryPlanner() = default;

    // 初始化：自由度 + 约束
    virtual bool init(int dof, const MotionConstraint& constraint) = 0;

    // 规划接口（统一！）
    virtual TrajectoryResult plan(const std::vector<Waypoint>& waypoints) = 0;

    // 双臂规划默认使用统一 4 ms 周期；需要其他周期的模块必须显式设置。
    void setCycleTime(double time) { dt_ = time; }

protected:
    int dof_ = 0;
    double dt_ =
        motion_control_pkg::execution::kDualArmControlPeriodSeconds;
    MotionConstraint constraint_;
};


#endif // TRAJECTORY_PLANNER_H
