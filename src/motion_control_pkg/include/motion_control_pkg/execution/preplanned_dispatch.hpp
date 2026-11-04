#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "motion_control_pkg/execution/control_period.hpp"
#include "motion_control_pkg/execution/execution_control.hpp"

namespace motion_control_pkg::execution
{
struct SteadyDispatchTiming {
    auto now() const { return std::chrono::steady_clock::now(); }
    void sleep_until(std::chrono::steady_clock::time_point time) const { std::this_thread::sleep_until(time); }
};

// Preparation must finish before the caller chooses the common start time.
template<class SendSample, class Timing = SteadyDispatchTiming>
bool dispatchPreplannedSamples(
    std::size_t count, std::chrono::steady_clock::time_point start,
    const std::shared_ptr<CancellationToken>& cancel,
    SendSample send, std::string& error, Timing timing = {})
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
        const auto scheduled = start + kDualArmControlPeriod * index;
        timing.sleep_until(scheduled);
        if (cancel && cancel->requested()) {
            error = "preplanned trajectory canceled";
            return false;
        }
        // A whole-body schedule has one clock. Never shift one child clock or
        // drain a backlog independently: cancel the shared group on a missed tick.
        const auto woke = timing.now();
        const auto missed_tick = [&](const char* phase, auto observed) {
            if (observed - scheduled < kDualArmControlPeriod)
                return false;
            const auto us=[](auto duration) {
                return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
            };
            std::ostringstream detail;
            detail << "preplanned dispatch missed 4 ms deadline at sample " << index
                << " phase=" << phase << " wake_late_us=" << us(woke-scheduled)
                << " sample_us=" << us(observed-woke) << " late_us=" << us(observed-scheduled);
            error = detail.str();
            if (cancel) cancel->request();
            return true;
        };
        if (missed_tick("before_sample",woke)) return false;
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
        if (missed_tick("after_sample",timing.now())) return false;
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
