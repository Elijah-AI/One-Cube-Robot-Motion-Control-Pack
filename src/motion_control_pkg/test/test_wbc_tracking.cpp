#include "motion_control_pkg/wbc/tracking.hpp"
#include <gtest/gtest.h>
namespace wbc=motion_control_pkg::wbc;
namespace {
auto specs() {return std::vector<wbc::JointSpec>{{"a",-1,1,2,3,false},{"b",-1,1,2,3,false}};}
}
TEST(WbcTracking, AcquisitionTimeSelectsHistoricalCommand) {
  wbc::PositionTrackingGuard g(specs());
  ASSERT_TRUE(g.command(10,{0,0})); ASSERT_TRUE(g.command(10.05,{.1,0}));
  EXPECT_TRUE(g.observe(10.02,10.10,{0,0},{0,0})); EXPECT_DOUBLE_EQ(g.peak(),0);
  EXPECT_TRUE(g.observe(10.06,10.11,{.1,0},{0,0}));
}
TEST(WbcTracking, PersistentErrorLatchesAndResetIsExplicit) {
  wbc::PositionTrackingGuard g(specs()); g.command(10,{0,0});
  EXPECT_TRUE(g.observe(10.01,10.01,{.02,0},{0,0}));
  EXPECT_FALSE(g.observe(10.10,10.10,{.02,0},{0,0}));
  EXPECT_FALSE(g.observe(10.11,10.11,{0,0},{0,0}));
  g.reset(); EXPECT_TRUE(g.command(11,{0,0})); EXPECT_TRUE(g.observe(11.01,11.01,{0,0},{0,0}));
}
TEST(WbcTracking, TransientAndDifferentJointsDoNotAccumulate) {
  wbc::PositionTrackingGuard g(specs()); g.command(10,{0,0});
  EXPECT_TRUE(g.observe(10.01,10.01,{.02,0},{0,0}));
  EXPECT_TRUE(g.observe(10.05,10.05,{0,.02},{0,0}));
  EXPECT_TRUE(g.observe(10.10,10.10,{.02,0},{0,0}));
  EXPECT_TRUE(g.observe(10.15,10.15,{0,0},{0,0}));
}
TEST(WbcTracking, ReplayDoesNotAdvanceDwellAndReverseFails) {
  wbc::PositionTrackingGuard g(specs()); g.command(10,{0,0});
  EXPECT_TRUE(g.observe(10.01,10.01,{.02,0},{0,0}));
  EXPECT_TRUE(g.observe(10.01,10.10,{.02,0},{0,0}));
  EXPECT_FALSE(g.observe(10.005,10.11,{0,0},{0,0}));
}
TEST(WbcTracking, HardErrorAndUrdfVelocityRejectImmediately) {
  wbc::PositionTrackingGuard g(specs()); g.command(10,{0,0});
  EXPECT_FALSE(g.observe(10.01,10.01,{.09,0},{0,0}));
  g.reset(); g.command(10,{0,0}); EXPECT_FALSE(g.observe(10.01,10.01,{0,0},{2.1,0}));
}
TEST(WbcTracking, MalformedStaleAndFutureFeedbackFail) {
  for (int fault=0;fault<4;++fault) {
    wbc::PositionTrackingGuard g(specs()); g.command(10,{0,0});
    EXPECT_FALSE(g.observe(fault==0?9.8:10.01, fault==1?10:10.02,
        fault==2?std::vector<double>{0}:std::vector<double>{0,0},
        fault==3?std::vector<double>{NAN,0}:std::vector<double>{0,0}));
  }
}
TEST(WbcTracking, CommandValidationAndHistoryRetention) {
  wbc::PositionTrackingGuard g(specs());
  for (int i=0;i<200;++i) ASSERT_TRUE(g.command(10+i*.004,{i*.001,0}));
  EXPECT_TRUE(g.observe(10.65,10.79,{.162,0},{0,0})); EXPECT_NEAR(g.peak(),0,1e-12);
  EXPECT_FALSE(g.command(10.8,{2,0}));
  g.reset(); g.command(11,{0,0}); EXPECT_FALSE(g.command(10,{0,0}));
}
