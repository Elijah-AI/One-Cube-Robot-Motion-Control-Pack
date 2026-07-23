#ifndef RUCKIG_PLANNER_H
#define RUCKIG_PLANNER_H

#include "traj_planner.h" 
#include <ruckig/ruckig.hpp>
#include <memory>
#include <vector>

class RuckigPlanner : public TrajectoryPlanner
{
public:
    RuckigPlanner() = default;
    ~RuckigPlanner() override = default;

    bool init(int dof, const MotionConstraint& constraint) override;
    TrajectoryResult plan(const std::vector<Waypoint>& waypoints) override;

private:
    int dof_ = 0;
    std::unique_ptr<ruckig::Ruckig<ruckig::DynamicDOFs>> rk_;
    std::unique_ptr<ruckig::InputParameter<ruckig::DynamicDOFs>> in_;
    std::unique_ptr<ruckig::OutputParameter<ruckig::DynamicDOFs>> out_;
};

#endif

