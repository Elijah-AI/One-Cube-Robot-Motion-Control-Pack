#pragma once

#include "miracle_m_algo.hpp"
#include <memory>
#include <stdexcept>

namespace motion_control_pkg::motion {

// Serialize scene/mode changes with every Pinocchio query. Destroy before dispatch.
class CollisionModeGuard {
public:
    CollisionModeGuard(MiracleMAlgo& algo, uint8_t mode)
        : algo_(algo), lock_(algo.lockCollisionScene()), previous_(algo.setCollisionCheckMode(mode)) {}
    ~CollisionModeGuard() { algo_.setCollisionCheckMode(previous_); }
    CollisionModeGuard(const CollisionModeGuard&) = delete;
    CollisionModeGuard& operator=(const CollisionModeGuard&) = delete;
private:
    MiracleMAlgo& algo_;
    std::unique_lock<std::recursive_mutex> lock_;
    uint8_t previous_;
};

class ScopedCollisionScene {
public:
    ScopedCollisionScene(MiracleMAlgo& algo, uint8_t mode,
                         const std::vector<double>& start,
                         sensor_msgs::msg::PointCloud2 cloud = sensor_msgs::msg::PointCloud2())
        : algo_(algo), guard_(algo, mode) {
        if (mode != 2) return;
        if (cloud.data.empty()) cloud = algo.collisionCloud();
        if (cloud.data.empty())
            throw std::runtime_error("Collision mode 2 requires an external point cloud; none received");
        const auto points = algo.pointCloudToModelRoot(cloud, start);
        if (!points || points->empty())
            throw std::runtime_error("External collision point cloud contains no usable points");
        algo.addExternalCollisionPointcloud(object_name_, points);
        installed_ = true;
    }
    ~ScopedCollisionScene() {
        if (installed_) algo_.removeExternalCollisionObject(object_name_);
    }
private:
    MiracleMAlgo& algo_;
    CollisionModeGuard guard_;
    bool installed_{false};
    // Stable scene object name, independent of the ROS topic and frame_id.
    const std::string object_name_{"box"};
};

inline std::unique_ptr<ScopedCollisionScene> makeCollisionScene(
    MiracleMAlgo& algo, uint8_t mode, const std::vector<double>& start,
    const sensor_msgs::msg::PointCloud2& cloud, std::string& error) {
    try {
        return std::make_unique<ScopedCollisionScene>(algo, mode, start, cloud);
    } catch (const std::exception& e) {
        error = e.what();
        return {};
    }
}

inline void checkChainCollision(MiracleMAlgo& algo, uint8_t mode, int module_id,
                               const std::vector<std::vector<double>>& samples,
                               const sensor_msgs::msg::PointCloud2& cloud = sensor_msgs::msg::PointCloud2()) {
    if (mode == 0) return;
    std::vector<double> start;
    if (!algo.getFullJointState(start)) throw std::runtime_error("Missing full joint state for collision check");
    ScopedCollisionScene scene(algo, mode, start, cloud);
    if (!algo.validateChainSamples(module_id, samples))
        throw std::runtime_error("Trajectory rejected by collision or model joint limits");
}
}  // namespace motion_control_pkg::motion
