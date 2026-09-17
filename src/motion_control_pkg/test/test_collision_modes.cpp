#include <gtest/gtest.h>
#include "motion_control_pkg/motion/scoped_collision_scene.hpp"
#include <pcl_conversions/pcl_conversions.h>

using motion_control_pkg::motion::ScopedCollisionScene;
using motion_control_pkg::motion::CollisionModeGuard;

TEST(CollisionModes, SeparatesSelfExternalAndDisabled) {
  MiracleMAlgo algo(TEST_URDF, "");
  Eigen::VectorXd q(1); q << 1.0;
  algo.setCollisionCheckMode(1);
  ASSERT_TRUE(algo.isFullQValid(q));
  auto obstacle = Eigen::Isometry3d::Identity();
  algo.addExternalCollisionBox("test_obstacle", Eigen::Vector3d::Ones(), obstacle);
  EXPECT_TRUE(algo.isFullQValid(q)); // self mode ignores existing external geometry
  algo.setCollisionCheckMode(2);
  EXPECT_FALSE(algo.isFullQValid(q));
  algo.setCollisionCheckMode(0);
  EXPECT_TRUE(algo.isFullQValid(q));
  q[0] = 0.0; // the two robot links overlap
  EXPECT_TRUE(algo.isFullQValid(q));
  algo.setCollisionCheckMode(1);
  EXPECT_FALSE(algo.isFullQValid(q));
  algo.setCollisionCheckMode(0);
  q[0] = 3.0;
  EXPECT_FALSE(algo.isFullQValid(q)); // off does not disable joint limits
  EXPECT_THROW(algo.setCollisionCheckMode(3), std::invalid_argument);
}

TEST(CollisionModes, PointCloudIsRequiredOnlyInModeTwoAndSceneRestores) {
  MiracleMAlgo algo(TEST_URDF, "");
  const std::vector<double> start{1.0};
  int calls = 0;
  algo.setCollisionCloudProvider([&] { ++calls; return sensor_msgs::msg::PointCloud2{}; });
  algo.setCollisionCheckMode(1);
  { ScopedCollisionScene scene(algo, 0, start); }
  { ScopedCollisionScene scene(algo, 1, start); }
  EXPECT_EQ(calls, 0);
  EXPECT_THROW(ScopedCollisionScene(algo, 2, start), std::runtime_error);
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(algo.setCollisionCheckMode(1), 1); // throwing constructor restores mode
  pcl::PointCloud<pcl::PointXYZ> points;
  points.push_back(pcl::PointXYZ(0, 0, 0));
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(points, cloud);
  cloud.header.frame_id = "Link3_L";
  {
    ScopedCollisionScene scene(algo, 2, start, cloud);
    EXPECT_FALSE(algo.isFullQValid(algo.toEigenConfig(start)));
  }
  EXPECT_EQ(calls, 1); // explicit cloud bypasses provider
  EXPECT_EQ(algo.setCollisionCheckMode(2), 1);
  EXPECT_TRUE(algo.isFullQValid(algo.toEigenConfig(start))); // temporary cloud removed
}
