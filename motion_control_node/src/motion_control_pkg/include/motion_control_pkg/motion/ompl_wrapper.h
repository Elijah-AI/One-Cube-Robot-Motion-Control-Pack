#ifndef OMPL_WRAPPER_H
#define OMPL_WRAPPER_H

#include "traj_common_util.h"
#include <vector>
#include <memory>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>

// 规划算法
enum class OmplPlanner {
    RRT,
    RRTConnect,  
    PRM,
    EST
};

// 规划部位
enum class PlanningGroup {
    LEFT_ARM,    // 0-6
    RIGHT_ARM,   // 7-13
    DUAL_ARM,    // 0-13 
    HEAD,        //14-15
    BODY         // 16-19
};

// 规划结果
struct OmplPlanResult {
    bool success = false;
    double planning_time = 0.0;
    std::vector<Waypoint> wayPoints; 
};

class OMPLWrapper {
public:
    static OMPLWrapper& instance();
    // 设置规划部位
    void setPlanningGroup(PlanningGroup group);
    // 设置关节限位
    void setJointLimits(const std::vector<double>& lower, const std::vector<double>& upper);
    // 设置算法
    void setPlanner(OmplPlanner type, double max_time = 1.0);
    void setPlanningTime(double time);
    void setEnableColliderCheck(bool enable);
    // 执行规划
    OmplPlanResult plan(
        const std::vector<double>& q_start,
        const std::vector<double>& q_goal
    );
private:
    OMPLWrapper() = default;
    // 内部
    void createStateSpace();
    bool isStateValid(const ompl::base::State* state);
    PlanningGroup group_;
    ompl::geometric::SimpleSetupPtr ss_;
    ompl::base::StateSpacePtr space_;
    std::vector<double> lower_limits_, upper_limits_;
    double solve_time_;
    int active_dof_;
    std::vector<int> active_indices_;
    bool collide_check_;
};
#endif
