#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "motion_control_pkg/execution/control_period.hpp"
#include "motion_control_pkg/execution/execution_control.hpp"

namespace motion_control_pkg::execution
{
// Preparation must finish before the caller chooses the common start time.
template<class SendSample>
bool dispatchPreplannedSamples(
    std::size_t count, std::chrono::steady_clock::time_point start,
    const std::shared_ptr<CancellationToken>& cancel,
    SendSample send, std::string& error)
{
    if (count == 0U) {
        error = "preplanned trajectory is empty";
        if (cancel) cancel->request();
        return false;
    }
    for (std::size_t index = 0; index < count; ++index) {
        if (cancel && cancel->requested()) {
            error = "preplanned trajectory canceled";
            return false;
        }
        std::this_thread::sleep_until(start + kDualArmControlPeriod * index);
        if (cancel && cancel->requested()) {
            error = "preplanned trajectory canceled";
            return false;
        }
        bool sent = false;
        try {
            sent = send(index);
        } catch (const std::exception& exception) {
            error = std::string("preplanned dispatch exception: ") + exception.what();
        } catch (...) {
            error = "unknown preplanned dispatch exception";
        }
        if (!sent) {
            if (cancel) cancel->request();
            return false;
        }
    }
    error.clear();
    return true;
}

inline bool splitWholeBodyActiveTrajectory(
    const std::vector<std::vector<double>>& trajectory, bool left, bool right,
    std::vector<std::vector<double>>& left_path,
    std::vector<std::vector<double>>& right_path,
    std::vector<std::vector<double>>& leg_path, std::string& error)
{
    left_path.clear(); right_path.clear(); leg_path.clear();
    const std::size_t expected = (left ? 7U : 0U) + (right ? 7U : 0U) + 4U;
    if ((!left && !right) || trajectory.empty()) {
        error = "whole-body trajectory requires an arm and at least one sample";
        return false;
    }
    for (const auto& sample : trajectory) {
        if (sample.size() != expected || !std::all_of(sample.begin(), sample.end(),
                [](double value) { return std::isfinite(value); })) {
            error = "whole-body trajectory requires matching finite joint samples";
            return false;
        }
    }
    for (const auto& sample : trajectory) {
        auto joint = sample.begin();
        if (left) { left_path.emplace_back(joint, joint + 7); joint += 7; }
        if (right) { right_path.emplace_back(joint, joint + 7); joint += 7; }
        leg_path.emplace_back(joint, sample.end());
    }
    error.clear();
    return true;
}
}  // namespace motion_control_pkg::execution
