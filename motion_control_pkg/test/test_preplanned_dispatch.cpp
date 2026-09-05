#include <gtest/gtest.h>
#include <future>
#include <limits>
#include <numeric>
#include <stdexcept>
#include "motion_control_pkg/execution/preplanned_dispatch.hpp"

using namespace motion_control_pkg::execution;
using namespace std::chrono_literals;

TEST(PreplannedDispatch, SplitsAllThreeWholeBodyModesWithoutChangingSamples)
{
    for (int mode = 0; mode < 3; ++mode) {
        const bool left = mode != 1, right = mode != 0;
        std::vector<double> sample(mode == 2 ? 18U : 11U);
        std::iota(sample.begin(), sample.end(), 0.0);
        std::vector<std::vector<double>> l, r, leg;
        std::string error;
        ASSERT_TRUE(splitWholeBodyActiveTrajectory({sample, sample}, left, right, l, r, leg, error));
        EXPECT_EQ(l.size(), left ? 2U : 0U);
        EXPECT_EQ(r.size(), right ? 2U : 0U);
        EXPECT_EQ(leg.size(), 2U);
        std::vector<double> joined;
        if (left) joined.insert(joined.end(), l[0].begin(), l[0].end());
        if (right) joined.insert(joined.end(), r[0].begin(), r[0].end());
        joined.insert(joined.end(), leg[0].begin(), leg[0].end());
        EXPECT_EQ(joined, sample);
    }
}

TEST(PreplannedDispatch, RejectsBadDimensionsNanAndEmptyArmSelection)
{
    std::vector<std::vector<double>> l, r, leg;
    std::string error;
    EXPECT_FALSE(splitWholeBodyActiveTrajectory({}, true, true, l, r, leg, error));
    EXPECT_FALSE(splitWholeBodyActiveTrajectory({{0,0,0,0}}, false, false, l, r, leg, error));
    std::vector<double> sample(11, 0.0);
    sample[5] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(splitWholeBodyActiveTrajectory({sample}, true, false, l, r, leg, error));
    EXPECT_TRUE(l.empty()); EXPECT_TRUE(leg.empty());
    EXPECT_FALSE(splitWholeBodyActiveTrajectory({std::vector<double>(11), {0.0}}, true, false, l, r, leg, error));
    EXPECT_TRUE(l.empty()); EXPECT_TRUE(leg.empty());
}

TEST(PreplannedDispatch, TwoPreparedStreamsUseCommonStartAndFourMillisecondSamples)
{
    const auto start = std::chrono::steady_clock::now() + 20ms;
    auto cancel = std::make_shared<CancellationToken>();
    auto run = [&]() {
        std::vector<std::chrono::steady_clock::time_point> times;
        std::string error;
        const bool ok = dispatchPreplannedSamples(4, start, cancel, [&](std::size_t index) {
            EXPECT_EQ(times.size(), index);
            times.push_back(std::chrono::steady_clock::now());
            return true;
        }, error);
        EXPECT_TRUE(ok) << error;
        return times;
    };
    auto arms = std::async(std::launch::async, run);
    auto legs = std::async(std::launch::async, run);
    for (const auto& times : {arms.get(), legs.get()}) {
        ASSERT_EQ(times.size(), 4U);
        for (std::size_t i = 0; i < times.size(); ++i)
            EXPECT_GE(times[i], start + 4ms * i);
    }
}

TEST(PreplannedDispatch, CancellationBeforeStartSendsNothing)
{
    auto cancel = std::make_shared<CancellationToken>();
    cancel->request();
    int sent = 0;
    std::string error;
    EXPECT_FALSE(dispatchPreplannedSamples(3, std::chrono::steady_clock::now(), cancel,
        [&](std::size_t) { ++sent; return true; }, error));
    EXPECT_EQ(sent, 0);
}

TEST(PreplannedDispatch, ChildFailureCancelsPeerAndSuppressesRemainingSamples)
{
    auto cancel = std::make_shared<CancellationToken>();
    int sent = 0, peer_sent = 0;
    std::string error;
    EXPECT_FALSE(dispatchPreplannedSamples(3, std::chrono::steady_clock::now(), cancel,
        [&](std::size_t) { ++sent; error = "fake device rejected sample"; return false; }, error));
    EXPECT_EQ(sent, 1);
    EXPECT_TRUE(cancel->requested());
    EXPECT_FALSE(dispatchPreplannedSamples(3, std::chrono::steady_clock::now(), cancel,
        [&](std::size_t) { ++peer_sent; return true; }, error));
    EXPECT_EQ(peer_sent, 0);
}

TEST(PreplannedDispatch, DeviceExceptionCancelsPeer)
{
    auto cancel = std::make_shared<CancellationToken>();
    std::string error;
    EXPECT_FALSE(dispatchPreplannedSamples(3, std::chrono::steady_clock::now(), cancel,
        [](std::size_t) -> bool { throw std::runtime_error("fake send error"); }, error));
    EXPECT_TRUE(cancel->requested());
    EXPECT_NE(error.find("fake send error"), std::string::npos);
}
