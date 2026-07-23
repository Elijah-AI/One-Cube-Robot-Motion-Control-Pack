#include "fcl_wrapper.h"

FCLWrapper& FCLWrapper::instance()
{
    static FCLWrapper ins;
    return ins;
}

// ==================== 几何体创建 ====================
FCLWrapper::GeometryPtr FCLWrapper::createSphere(double radius)
{
    return std::make_shared<fcl::Sphered>(radius);
}

FCLWrapper::GeometryPtr FCLWrapper::createBox(double x, double y, double z)
{
    return std::make_shared<fcl::Boxd>(x, y, z);
}

FCLWrapper::GeometryPtr FCLWrapper::createCylinder(double radius, double height)
{
    return std::make_shared<fcl::Cylinderd>(radius, height);
}

FCLWrapper::GeometryPtr FCLWrapper::createPoint(const Eigen::Vector3d& p)
{
    auto geom = std::make_shared<fcl::Sphered>(1e-6);
    return geom;
}

FCLWrapper::GeometryPtr FCLWrapper::createLine(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    Eigen::Vector3d dir = b - a;
    double len = dir.norm();
    auto geom = std::make_shared<fcl::Capsuled>(1e-6, len);
    return geom;
}

FCLWrapper::GeometryPtr FCLWrapper::createMesh(const std::vector<Eigen::Vector3d>& vertices,
                                               const std::vector<Eigen::Vector3i>& triangles)
{
    auto mesh = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
    // 1. 转换顶点: Eigen::Vector3d -> fcl::Vector3d
    std::vector<fcl::Vector3d> fcl_vertices;
    fcl_vertices.reserve(vertices.size());
    for (const auto& v : vertices) {
        fcl_vertices.emplace_back(v.x(), v.y(), v.z());
    }

    // 2. 转换三角形: Eigen::Vector3i -> fcl::Triangle
    // 注意：addSubModel 的第二个参数是 std::vector<fcl::Triangle>
    std::vector<fcl::Triangle> fcl_triangles;
    fcl_triangles.reserve(triangles.size());
    for (const auto& t : triangles) {
        // fcl::Triangle 构造函数接受三个 int 索引
        fcl_triangles.emplace_back(t.x(), t.y(), t.z());
    }

    // 3. 构建模型
    mesh->beginModel();
    // 现在参数类型完全匹配：
    // const std::vector<fcl::Vector3d>& ps
    // const std::vector<fcl::Triangle>& ts
    mesh->addSubModel(fcl_vertices, fcl_triangles);
    mesh->endModel();

    return mesh;
}

// ==================== 碰撞检测 ====================
bool FCLWrapper::isCollide(GeometryPtr geom1, const Eigen::Isometry3d& pose1,
                           GeometryPtr geom2, const Eigen::Isometry3d& pose2)
{
    fcl::CollisionObjectd obj1(geom1, pose1);
    fcl::CollisionObjectd obj2(geom2, pose2);

    fcl::CollisionRequestd req;
    fcl::CollisionResultd res;
    fcl::collide(&obj1, &obj2, req, res);

    return res.isCollision();
}

CollisionResult FCLWrapper::collideDetail(GeometryPtr geom1, const Eigen::Isometry3d& pose1,
                                          GeometryPtr geom2, const Eigen::Isometry3d& pose2)
{
    fcl::CollisionObjectd obj1(geom1, pose1);
    fcl::CollisionObjectd obj2(geom2, pose2);

    fcl::CollisionRequestd req;
    req.enable_contact = true;    // 开启接触点记录
    req.num_max_contacts = 100;   // 最大接触点数量

    fcl::CollisionResultd res;
    fcl::collide(&obj1, &obj2, req, res);

    CollisionResult result;
    result.is_collision = res.isCollision();
    result.contact_num = res.numContacts();

    for (size_t i = 0; i < res.numContacts(); ++i) {
        const auto& c = res.getContact(i);
        result.contacts.push_back(c.pos);
        result.normals.push_back(c.normal);
        result.depths.push_back(c.penetration_depth);
    }

    return result;
}

// ==================== 距离计算 ====================
DistanceResult FCLWrapper::distance(GeometryPtr geom1, const Eigen::Isometry3d& pose1,
                                    GeometryPtr geom2, const Eigen::Isometry3d& pose2)
{
    fcl::CollisionObjectd obj1(geom1, pose1);
    fcl::CollisionObjectd obj2(geom2, pose2);

    fcl::DistanceRequestd req;
    req.enable_nearest_points = true;  // 开启最近点记录
    req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

    fcl::DistanceResultd res;
    fcl::distance(&obj1, &obj2, req, res);

    DistanceResult result;
    result.valid = true;
    result.min_distance = res.min_distance;
    result.nearest_pt1 = res.nearest_points[0];
    result.nearest_pt2 = res.nearest_points[1];

    return result;
}
