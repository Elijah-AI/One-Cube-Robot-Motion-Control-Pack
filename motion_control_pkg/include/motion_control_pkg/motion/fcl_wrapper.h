#ifndef FCL_WRAPPER_H
#define FCL_WRAPPER_H

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

// FCL 核心头文件
#include <fcl/fcl.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>

// 碰撞结果结构体
struct CollisionResult
{
    bool is_collision = false;    // 是否碰撞
    int contact_num = 0;          // 接触点数量
    std::vector<Eigen::Vector3d> contacts;  // 接触点
    std::vector<Eigen::Vector3d> normals;   // 接触法向
    std::vector<double> depths;             // 穿透深度
};

// 距离结果结构体
struct DistanceResult
{
    bool valid = false;           // 计算是否有效
    double min_distance = 0.0;    // 最短距离
    Eigen::Vector3d nearest_pt1;  // 物体1最近点
    Eigen::Vector3d nearest_pt2;  // 物体2最近点
};

// FCL 碰撞检测封装类（单例模式，全局使用一个实例即可）
class FCLWrapper
{
public:
    static FCLWrapper& instance();

    // ==================== 几何体类型 ====================
    using GeometryPtr = std::shared_ptr<fcl::CollisionGeometryd>;

    // 创建几何体（自动管理内存）
    GeometryPtr createSphere(double radius);
    GeometryPtr createBox(double x, double y, double z);
    GeometryPtr createCylinder(double radius, double height);
    GeometryPtr createPoint(const Eigen::Vector3d& p);
    GeometryPtr createLine(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
    
    // 三角网格（vertices: N*3, triangles: M*3）
    GeometryPtr createMesh(const std::vector<Eigen::Vector3d>& vertices,
                           const std::vector<Eigen::Vector3i>& triangles);

    // ==================== 核心检测接口 ====================
    // 1. 仅检测是否碰撞（最快）
    bool isCollide(GeometryPtr geom1, const Eigen::Isometry3d& pose1,
                   GeometryPtr geom2, const Eigen::Isometry3d& pose2);

    // 2. 详细碰撞信息（接触点、法向、深度）
    CollisionResult collideDetail(GeometryPtr geom1, const Eigen::Isometry3d& pose1,
                                  GeometryPtr geom2, const Eigen::Isometry3d& pose2);

    // 3. 最短距离计算
    DistanceResult distance(GeometryPtr geom1, const Eigen::Isometry3d& pose1,
                            GeometryPtr geom2, const Eigen::Isometry3d& pose2);

private:
    FCLWrapper() = default;
    ~FCLWrapper() = default;
    FCLWrapper(const FCLWrapper&) = delete;
    FCLWrapper& operator=(const FCLWrapper&) = delete;
};

#endif // FCL_WRAPPER_H
