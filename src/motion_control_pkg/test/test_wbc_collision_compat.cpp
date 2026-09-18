#include <gtest/gtest.h>
#include "motion_control_pkg/motion/scoped_collision_scene.hpp"

TEST(WbcCollisionCompatibility, DisablingCollisionDoesNotDisableComProtection) {
  const std::string description = WBC_TEST_DESCRIPTION;
  MiracleMAlgo algo(description + "/urdf/onecube_robot_description.urdf", description);
  motion_control_pkg::wbc::StaticBalanceOptions options;
  options.support_links = {"base_front_wheel_1_pitch_link", "base_rear_wheel_1_pitch_link",
                           "base_front_wheel_2_link", "base_rear_wheel_2_link"};
  options.unknown_steering = {"base_front_wheel_1_yaw_joint", "base_rear_wheel_1_yaw_joint"};
  algo.configureStaticBalance(options);
  const auto neutral = algo.staticBalance()->model().neutralSnapshot();
  std::vector<double> safe;
  ASSERT_TRUE(algo.balanceStateToFullQ(neutral, safe));
  ASSERT_TRUE(algo.isStaticBalanceValid(algo.toEigenConfig(safe)));
  std::vector<double> unsafe;
  const auto& joints = algo.staticBalance()->model().joints();
  for (std::size_t j = 0; j < joints.size() && unsafe.empty(); ++j) {
    if (joints[j].name.find("leg_joint") != 0) continue;
    for (int step = 0; step <= 20; ++step) {
      auto state = neutral;
      state.positions[j] = joints[j].lower + (joints[j].upper - joints[j].lower) * step / 20.;
      std::vector<double> q;
      ASSERT_TRUE(algo.balanceStateToFullQ(state, q));
      if (!algo.isStaticBalanceValid(algo.toEigenConfig(q))) { unsafe = q; break; }
    }
  }
  ASSERT_FALSE(unsafe.empty()) << "URDF fixture must include a posture outside the support region";
  for (uint8_t mode : {0, 1, 2}) {
    motion_control_pkg::motion::CollisionModeGuard guard(algo, mode);
    EXPECT_TRUE(algo.isStaticBalanceValid(algo.toEigenConfig(safe)));
    EXPECT_FALSE(algo.isStaticBalanceValid(algo.toEigenConfig(unsafe)));
    std::string error;
    EXPECT_FALSE(algo.validateStaticBalancePath({unsafe}, error));
    EXPECT_FALSE(error.empty());
  }
  // A WBC model does not make an absent visual collision cloud acceptable.
  algo.setCollisionCloudProvider([] { return sensor_msgs::msg::PointCloud2{}; });
  EXPECT_THROW(motion_control_pkg::motion::ScopedCollisionScene(algo, 2, safe), std::runtime_error);
}
