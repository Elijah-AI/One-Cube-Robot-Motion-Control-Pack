#include "motion_control_pkg/miracle_m_algo.hpp"
#include "motion_control_pkg/motion/planning_worker_pool.hpp"
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

struct MiracleMAlgo::PlanningWorkspaces {
    PlanningWorkspaces(const pinocchio::Model& model, std::size_t count)
        : data(count), ik(count), geometry(count), pool(count) {
        for (auto& item : data) item = std::make_unique<pinocchio::Data>(model);
    }
    std::vector<std::unique_ptr<pinocchio::Data>> data;
    std::vector<SingleArmIkWorkspace> ik;
    std::vector<std::unique_ptr<pinocchio::GeometryData>> geometry;
    std::uint64_t geometry_revision{std::numeric_limits<std::uint64_t>::max()};
    // Last member: join before releasing any workspace.
    motion_control_pkg::motion::PlanningWorkerPool pool;
};

void MiracleMAlgo::configurePlanningWorkers(std::size_t count) {
    if (count != 1 && count != 2 && count != 4)
        throw std::invalid_argument("planning worker count must be 1, 2 or 4");
    std::lock_guard<std::recursive_mutex> lock(pinocchio_data_mutex_);
    if (count == 1) planning_workspaces_.reset();
    else if (!planning_workspaces_ || planning_workspaces_->data.size() != count) {
        auto candidate = std::make_shared<PlanningWorkspaces>(model_, count);
        planning_workspaces_ = std::move(candidate);
    }
    planning_worker_count_ = count;
}

MiracleMAlgo::PlanningWorkspaces& MiracleMAlgo::planningWorkspaces() {
    if (!planning_workspaces_)
        planning_workspaces_ = std::make_shared<PlanningWorkspaces>(model_, planning_worker_count_);
    return *planning_workspaces_;
}

bool MiracleMAlgo::planDualArmCartesianIkWaypoints(
    const std::vector<double>& start_q,
    const std::array<Eigen::Isometry3d, 2>& goals,
    double translation_step, double rotation_step,
    std::array<std::vector<std::vector<double>>, 2>& waypoints,
    std::array<Eigen::Isometry3d, 2>& starts,
    double& elapsed_seconds, const PlanningStop& stop)
{
    // The caller may already hold ScopedCollisionScene. Only the coordinator
    // takes this recursive lock; neither worker calls any locking public API.
    std::lock_guard<std::recursive_mutex> lock(pinocchio_data_mutex_);
    const auto begin = std::chrono::steady_clock::now();
    std::array<bool, 2> ok{};
    auto* workers = planning_worker_count_ > 1 ? &planningWorkspaces() : nullptr;
    const auto job = [&](std::size_t arm) {
        SingleArmIkWorkspace serial_workspace;
        double worker_seconds = 0;
        ok[arm] = planSingleArmCartesianIkWaypointsWithData(arm == 0, start_q, goals[arm],
            translation_step, rotation_step, waypoints[arm], starts[arm], &worker_seconds,
            workers ? *workers->data[arm] : *data_,
            workers ? workers->ik[arm] : serial_workspace, stop);
    };
    try {
        if (workers) workers->pool.run(job, 2);
        else { job(0); job(1); }
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "Dual IK worker failed: %s", error.what());
        ok = {};
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "Dual IK worker failed: unknown exception");
        ok = {};
    }
    elapsed_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    if (!ok[0] || !ok[1] || (stop && stop())) {
        waypoints = {};
        return false;
    }
    return true;
}

bool MiracleMAlgo::getFramePosesAtConfigurations(
    const std::string& source_frame, const std::string& target_frame,
    const std::vector<std::vector<double>>& configurations,
    std::vector<Eigen::Isometry3d>& poses)
{
    std::lock_guard<std::recursive_mutex> lock(pinocchio_data_mutex_);
    poses.clear();
    if (configurations.empty() || !model_.existFrame(source_frame) || !model_.existFrame(target_frame))
        return false;
    for (const auto& q : configurations)
        if (q.size() != static_cast<std::size_t>(model_.nq) ||
            !std::all_of(q.begin(), q.end(), [](double v) { return std::isfinite(v); })) return false;
    const auto source = model_.getFrameId(source_frame), target = model_.getFrameId(target_frame);
    poses.resize(configurations.size(), Eigen::Isometry3d::Identity());
    // Avoid scheduling overhead on short paths. No per-sample thread creation.
    auto* workers = planning_worker_count_ > 1 && configurations.size() >= 32*planning_worker_count_ ? &planningWorkspaces() : nullptr;
    const std::size_t count = workers ? workers->data.size() : 1;
    std::array<bool, 4> valid{true, true, true, true};
    const auto job = [&](std::size_t worker) {
        auto& data = workers ? *workers->data[worker] : *data_;
        for (std::size_t i = configurations.size()*worker/count;
             i < configurations.size()*(worker+1)/count; ++i) {
            const auto q = toEigenConfig(configurations[i]);
            pinocchio::forwardKinematics(model_, data, q);
            pinocchio::updateFramePlacements(model_, data);
            poses[i].matrix() = data.oMf[source].actInv(data.oMf[target]).toHomogeneousMatrix();
            valid[worker] = valid[worker] && poses[i].matrix().allFinite();
        }
    };
    try {
        if (workers) workers->pool.run(job); else job(0);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "Batch FK worker failed: %s", error.what());
        poses.clear(); return false;
    } catch (...) {
        poses.clear(); return false;
    }
    if (!std::all_of(valid.begin(), valid.end(), [](bool ok) { return ok; })) { poses.clear(); return false; }
    return true;
}

bool MiracleMAlgo::validateFullConfigurations(
    const std::vector<std::vector<double>>& configurations,
    std::size_t& first_invalid, const PlanningStop& stop)
{
    std::lock_guard<std::recursive_mutex> lock(pinocchio_data_mutex_);
    first_invalid = 0;
    if (configurations.empty()) return false;
    applyCollisionCheckMode();
    auto* workers = planning_worker_count_ > 1 && configurations.size() >= 32*planning_worker_count_ ? &planningWorkspaces() : nullptr;
    if (workers) {
        if (workers->geometry_revision != geometry_revision_) {
            for (auto& geometry : workers->geometry)
                geometry = std::make_unique<pinocchio::GeometryData>(geom_model_);
            workers->geometry_revision = geometry_revision_;
        }
        for (auto& geometry : workers->geometry) {
            geometry->activeCollisionPairs = geom_data_->activeCollisionPairs;
            geometry->collisionRequests = geom_data_->collisionRequests;
        }
    }
    // Bytes, not vector<bool>: workers must never write into the same packed word.
    std::vector<std::uint8_t> valid(configurations.size(), 0);
    const std::size_t count = workers ? workers->data.size() : 1;
    const auto job = [&](std::size_t worker) {
        auto& data = workers ? *workers->data[worker] : *data_;
        auto& geometry = workers ? *workers->geometry[worker] : *geom_data_;
        for (std::size_t i = configurations.size()*worker/count;
             i < configurations.size()*(worker+1)/count; ++i) {
            if (stop && stop()) break;
            const auto& q = configurations[i];
            // toEigenConfig intentionally accepts some shorter legacy inputs;
            // a batch query requires a complete configuration, never padding.
            if (q.size() == static_cast<std::size_t>(model_.nq))
                valid[i] = isFullQValidWithData(toEigenConfig(q), data, geometry);
        }
    };
    try {
        if (workers) workers->pool.run(job); else job(0);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("MiracleMAlgo"), "Batch collision worker failed: %s", error.what());
        return false;
    } catch (...) {
        return false;
    }
    const auto invalid = std::find(valid.begin(), valid.end(), 0);
    first_invalid = static_cast<std::size_t>(invalid - valid.begin());
    return invalid == valid.end() && !(stop && stop());
}
