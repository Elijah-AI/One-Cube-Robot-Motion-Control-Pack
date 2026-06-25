#ifndef PLANNER_FACTORY_H
#define PLANNER_FACTORY_H

#include "traj_planner.h"
#include <memory>

// 工厂类：创建规划器（上层动态调用）
class PlannerFactory
{
public:
    static std::unique_ptr<TrajectoryPlanner> createPlanner(PlannerType type);
};


#endif
