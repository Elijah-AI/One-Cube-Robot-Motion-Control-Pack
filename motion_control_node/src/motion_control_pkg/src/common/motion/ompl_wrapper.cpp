#include "ompl_wrapper.h"
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/est/EST.h>

OMPLWrapper& OMPLWrapper::instance() {
    static OMPLWrapper ins;
    return ins;
}

void OMPLWrapper::setPlanningGroup(PlanningGroup group) {
    group_ = group;
    active_indices_.clear();

    switch (group) {
        case PlanningGroup::LEFT_ARM:
            active_indices_ = {0,1,2,3,4,5,6};
            break;
        case PlanningGroup::RIGHT_ARM:
            active_indices_ = {7,8,9,10,11,12,13};
            break;
        case PlanningGroup::DUAL_ARM:
            active_indices_ = {0,1,2,3,4,5,6,7,8,9,10,11,12,13};
            break;
        case PlanningGroup::HEAD:
            active_indices_ = {14,15};
            break;
        case PlanningGroup::BODY:
            active_indices_ = {16,17,18,19};
            break;
    }
    active_dof_ = active_indices_.size();
    createStateSpace();
}

void OMPLWrapper::setJointLimits(const std::vector<double>& lower, const std::vector<double>& upper) {
    lower_limits_ = lower;
    upper_limits_ = upper;
}

void OMPLWrapper::createStateSpace() {
    space_ = std::make_shared<ompl::base::RealVectorStateSpace>(active_dof_);
    auto* space = space_->as<ompl::base::RealVectorStateSpace>();
    ompl::base::RealVectorBounds bounds(active_dof_);

    for (int i = 0; i < active_dof_; ++i) {
        int idx = active_indices_[i];
        bounds.setLow(i, lower_limits_[idx]);
        bounds.setHigh(i, upper_limits_[idx]);
    }

    space->setBounds(bounds);
    ss_ = std::make_shared<ompl::geometric::SimpleSetup>(space_);

    ss_->setStateValidityChecker([this](const ompl::base::State* s) {
        return isStateValid(s);
    });
}

bool OMPLWrapper::isStateValid(const ompl::base::State* state) {

    return true;
}

void OMPLWrapper::setPlanningTime(double time)
{
   solve_time_ = time;
}

void OMPLWrapper::setPlanner(OmplPlanner type, double max_time) {
    switch (type) {
        case OmplPlanner::RRT:
            ss_->setPlanner(std::make_shared<ompl::geometric::RRT>(ss_->getSpaceInformation()));
            break;
        case OmplPlanner::RRTConnect:
            ss_->setPlanner(std::make_shared<ompl::geometric::RRTConnect>(ss_->getSpaceInformation()));
            break;
        case OmplPlanner::PRM:
            ss_->setPlanner(std::make_shared<ompl::geometric::PRM>(ss_->getSpaceInformation()));
            break;
        case OmplPlanner::EST:
            ss_->setPlanner(std::make_shared<ompl::geometric::EST>(ss_->getSpaceInformation()));
            break;
    }
    setPlanningTime(max_time);
}

void OMPLWrapper::setEnableColliderCheck(bool enable)
{
    collide_check_ = enable;
}

OmplPlanResult OMPLWrapper::plan(
    const std::vector<double>& q_start,
    const std::vector<double>& q_goal
) 
{
    OmplPlanResult res;
   // current_obstacles_ = obstacles;

    // 安全检查
    if (!space_ || !ss_ || active_dof_ == 0 || active_indices_.empty()) {
        return res;
    }
    if (q_start.size() != q_goal.size()) {
        return res;
    }
    bool is_partial = (q_start.size() == active_dof_);  
    if (!is_partial) {
        return res;
    }

    ompl::base::ScopedState<> ompl_start(space_);
    ompl::base::ScopedState<> ompl_goal(space_);

    std::vector<double> q_full_rest;

    for (int i = 0; i < active_dof_; ++i) {
        int idx = active_indices_[i];
        if (is_partial) {
            ompl_start[i] = q_start[i];
            ompl_goal[i]  = q_goal[i];
        } else {
            ompl_start[i] = q_start[idx];
            ompl_goal[i]  = q_goal[idx];
            q_full_rest[idx] = q_start[idx]; 
        }
    }

    ss_->setStartState(ompl_start);
    ss_->setGoalState(ompl_goal);
    ss_->setup();

   ompl::base::PlannerStatus status = ss_->solve(solve_time_);
    if (!status) {
        return res;
    }
     ss_->simplifySolution();
     auto path = ss_->getSolutionPath();
    if (path.getStateCount() > 0) {
        const auto* first_state = path.getState(0);
        const auto* last_state = path.getState(path.getStateCount() - 1);
     
        double dist_first_to_start = space_->distance(first_state, ompl_start.get());
        double dist_last_to_start = space_->distance(last_state, ompl_start.get());
        
        if (dist_last_to_start < dist_first_to_start) {
           // std::cout << "Detected reversed path. Reversing..." << std::endl;
            path.reverse();
        }
    }
  for (size_t i = 0; i < path.getStateCount(); ++i) 
  { 
    const auto* s = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
     std::vector<double> q_partial(active_dof_);
    for (int j = 0; j < active_dof_; ++j) {
        q_partial[j] = (*s)[j];
    }
    
    Waypoint wp;
    wp.positions.resize(active_dof_);
    for (int j = 0; j < active_dof_; ++j) {
        wp.positions[j] = q_partial[j];
    }
    res.wayPoints.push_back(wp);
   }
    res.success = true;
    res.planning_time = ss_->getLastPlanComputationTime();
    return res;
}