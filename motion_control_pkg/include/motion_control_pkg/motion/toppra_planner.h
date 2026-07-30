#ifndef TOPPRA_PLANNER_H
#define TOPPRA_PLANNER_H

#include "traj_planner.h"
#include <toppra/toppra.hpp>
#include <Eigen/Core>

class ToppraPlanner : public TrajectoryPlanner
{
public:
    bool init(int dof, const MotionConstraint& constraint) override;
    TrajectoryResult plan(const std::vector<Waypoint>& waypoints) override;

private:
    // 内部Eigen缓存
    Eigen::VectorXd vel_min_;
    Eigen::VectorXd vel_max_;
    Eigen::VectorXd acc_min_;
    Eigen::VectorXd acc_max_;

    // toppra网格离散数
    int grid_num_ = 100;
};

#endif
