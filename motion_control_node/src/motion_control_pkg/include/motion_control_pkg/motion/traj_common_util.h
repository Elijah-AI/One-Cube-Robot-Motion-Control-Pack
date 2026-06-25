#ifndef TRAJ_COMMON_UTIL_H
#define TRAJ_COMMON_UTIL_H

#include <vector>

// 路点（位置 + 时间）
struct Waypoint
{
    std::vector<double> positions;  // 路点位置
    double time = 0.0;              // 时间戳（Toppra自动优化）
};


#endif
