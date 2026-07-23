#include "planner_factory.h"
#include "toppra_planner.h"
#include "ruckig_planner.h"

std::unique_ptr<TrajectoryPlanner> PlannerFactory::createPlanner(PlannerType type)
{
    switch(type)
    {
        case PlannerType::TOPPRA:
            return std::make_unique<ToppraPlanner>();
        case PlannerType::RUCKIG:
            // 后续Ruckig派生类在此添加
            return std::make_unique<RuckigPlanner>();
        default:
            return nullptr;
    }
}
