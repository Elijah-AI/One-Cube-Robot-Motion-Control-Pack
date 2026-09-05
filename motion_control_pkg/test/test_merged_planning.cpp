#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/serialization.hpp>
#include "com_interfaces/msg/synced_robot_vision.hpp"
#include "motion_control_pkg/miracle_m_algo.hpp"

namespace
{
class MergedPlanning : public ::testing::Test
{
protected:
    static std::unique_ptr<MiracleMAlgo> algo;
    static void SetUpTestSuite() {
        algo = std::make_unique<MiracleMAlgo>(
            MERGE_MODEL_ROOT "/onecube_robot_description/urdf/onecube_robot_description.urdf",
            MERGE_MODEL_ROOT);
    }
    static void TearDownTestSuite() { algo.reset(); }
    std::vector<double> neutral() {
        return std::vector<double>(algo->getModel().nq, 0.0);
    }
    void set(std::vector<double>& q, const std::string& joint, double value) {
        const auto& model = algo->getModel();
        q[model.joints[model.getJointId(joint)].idx_q()] = value;
    }
    Eigen::Isometry3d pose(const std::vector<double>& q, const std::string& frame) {
        Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
        EXPECT_TRUE(algo->getFramePoseAtConfiguration("base_link", frame, q, result));
        return result;
    }
    sensor_msgs::msg::PointCloud2 cloudMessage(const std::string& frame) {
        sensor_msgs::msg::PointCloud2 message;
        message.header.frame_id = frame;
        sensor_msgs::PointCloud2Modifier modifier(message);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(1);
        *sensor_msgs::PointCloud2Iterator<float>(message, "x") = 0.1F;
        *sensor_msgs::PointCloud2Iterator<float>(message, "y") = 0.2F;
        *sensor_msgs::PointCloud2Iterator<float>(message, "z") = 0.3F;
        return message;
    }
};
std::unique_ptr<MiracleMAlgo> MergedPlanning::algo;

TEST_F(MergedPlanning, HeadPoseFollowsHeadAndLegState)
{
    auto q = neutral();
    const auto initial = pose(q, "head_pitch_link");
    set(q, "head_pitch_joint", 0.2);
    set(q, "leg_joint1", 0.1);
    const auto moved = pose(q, "head_pitch_link");
    EXPECT_GT((initial.matrix() - moved.matrix()).norm(), 0.05);
    EXPECT_NEAR(moved.rotation().determinant(), 1.0, 1e-10);
    EXPECT_TRUE(moved.matrix().allFinite());
    Eigen::Matrix4d missing = Eigen::Matrix4d::Identity();
    EXPECT_FALSE(algo->getForwardKinematics("base_link", "missing_head", missing));
}

TEST_F(MergedPlanning, CameraCloudIsTransformedIntoModelRoot)
{
    const auto q = neutral();
    auto message = cloudMessage("head_pitch_link");
    const auto converted = algo->pointCloudToModelRoot(message, q);
    ASSERT_EQ(converted->size(), 1U);
    const Eigen::Vector3d expected = pose(q, "head_pitch_link") * Eigen::Vector3d(0.1F, 0.2F, 0.3F);
    EXPECT_NEAR(converted->front().x, expected.x(), 1e-6);
    EXPECT_NEAR(converted->front().y, expected.y(), 1e-6);
    EXPECT_NEAR(converted->front().z, expected.z(), 1e-6);
    message.header.frame_id = "missing_camera";
    EXPECT_THROW(algo->pointCloudToModelRoot(message, q), std::runtime_error);
}

TEST_F(MergedPlanning, RejectsMalformedAndNonfiniteClouds)
{
    auto message = cloudMessage("base_link");
    const auto q = neutral();
    message.data.pop_back();
    EXPECT_THROW(algo->pointCloudToModelRoot(message, q), std::runtime_error);
    message = cloudMessage("base_link");
    message.fields.pop_back();
    EXPECT_THROW(algo->pointCloudToModelRoot(message, q), std::runtime_error);
    message = cloudMessage("base_link");
    *sensor_msgs::PointCloud2Iterator<float>(message, "y") = std::numeric_limits<float>::quiet_NaN();
    EXPECT_THROW(algo->pointCloudToModelRoot(message, q), std::runtime_error);
    message = cloudMessage("");
    EXPECT_THROW(algo->pointCloudToModelRoot(message, q), std::runtime_error);
}

TEST_F(MergedPlanning, OctreeRejectsEmptyCloudAndInvalidResolution)
{
    Eigen::Vector3d center;
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    EXPECT_THROW(algo->buildOcTreeFromCloud(nullptr, 0.01, center), std::runtime_error);
    EXPECT_THROW(algo->buildOcTreeFromCloud(cloud, 0.01, center), std::runtime_error);
    cloud->push_back(pcl::PointXYZ(1.0F, 2.0F, 3.0F));
    EXPECT_THROW(algo->buildOcTreeFromCloud(cloud, 0.0, center), std::runtime_error);
    cloud->push_back(pcl::PointXYZ(1.0F, std::numeric_limits<float>::infinity(), 3.0F));
    EXPECT_NO_THROW(algo->buildOcTreeFromCloud(cloud, 0.01, center));
    EXPECT_TRUE(center.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
    EXPECT_THROW(algo->addExternalCollisionPointcloud("invalid_cloud", nullptr), std::runtime_error);
}

TEST_F(MergedPlanning, OccupiedCloudBlocksOmplAndRemovalRestoresScene)
{
    const auto q = neutral();
    ASSERT_TRUE(algo->isFullQValid(algo->toEigenConfig(q)));
    std::vector<double> left;
    ASSERT_TRUE(algo->extractSingleArmJoints(true, q, left));
    std::vector<std::vector<double>> path;
    ASSERT_TRUE(algo->planSingleArmJointSpace(true, q, left, path));
    // Dense occupied cube around a checked arm link, then a far-away replacement.
    auto obstacle = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const Eigen::Vector3d center = pose(q, "Link4_L").translation();
    for (int x = -10; x <= 10; ++x)
        for (int y = -10; y <= 10; ++y)
            for (int z = -10; z <= 10; ++z)
                obstacle->push_back(pcl::PointXYZ(center.x() + x * 0.01,
                    center.y() + y * 0.01, center.z() + z * 0.01));
    algo->addExternalCollisionPointcloud("test_environment", obstacle);
    EXPECT_FALSE(algo->isFullQValid(algo->toEigenConfig(q)));
    EXPECT_FALSE(algo->planSingleArmJointSpace(true, q, left, path));
    EXPECT_THROW(algo->addExternalCollisionPointcloud("test_environment", nullptr), std::runtime_error);
    EXPECT_FALSE(algo->isFullQValid(algo->toEigenConfig(q)));
    auto far = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    far->push_back(pcl::PointXYZ(100.0, 100.0, 100.0));
    algo->addExternalCollisionPointcloud("test_environment", far);
    EXPECT_TRUE(algo->isFullQValid(algo->toEigenConfig(q)));
    algo->removeExternalCollisionObject("test_environment");
    EXPECT_TRUE(algo->isFullQValid(algo->toEigenConfig(q)));
}

TEST_F(MergedPlanning, WholeBodyIkSupportsLeftRightAndBothGoals)
{
    auto start = neutral();
    set(start, "Joint2_L", 0.3);
    set(start, "Joint2_R", -0.3);
    for (int mode = 0; mode < 3; ++mode) {
        auto target = start;
        if (mode != 1) set(target, "Joint1_L", 0.04);
        if (mode != 0) set(target, "Joint1_R", -0.04);
        const std::optional<Eigen::Isometry3d> left = mode != 1
            ? std::optional<Eigen::Isometry3d>(pose(target, "gripper_base_l_link")) : std::nullopt;
        const std::optional<Eigen::Isometry3d> right = mode != 0
            ? std::optional<Eigen::Isometry3d>(pose(target, "gripper_base_r_link")) : std::nullopt;
        Eigen::VectorXd result;
        ASSERT_TRUE(algo->solveWholeBodyIK(algo->toEigenConfig(start), left, right,
            result, {1.0, 1.0, 1.0, 0.2}, 0.5236, 3.0, 0, false)) << mode;
        const std::vector<double> solved(result.data(), result.data() + result.size());
        if (left) { EXPECT_LT((pose(solved, "gripper_base_l_link").matrix() - left->matrix()).norm(), 4e-4); }
        if (right) { EXPECT_LT((pose(solved, "gripper_base_r_link").matrix() - right->matrix()).norm(), 4e-4); }
        for (const auto& name : {"head_yaw_joint", "head_pitch_joint"}) {
            const auto index = algo->getModel().joints[algo->getModel().getJointId(name)].idx_q();
            EXPECT_DOUBLE_EQ(result[index], start[index]);
        }
        EXPECT_GE(result.minCoeff(), -3.2);
        for (Eigen::Index i = 0; i < result.size(); ++i) {
            EXPECT_GE(result[i], algo->getModel().lowerPositionLimit[i]);
            EXPECT_LE(result[i], algo->getModel().upperPositionLimit[i]);
        }
    }
}

TEST_F(MergedPlanning, RelaxedBodyPitchStillKeepsUrdfAndCcsLimits)
{
    auto start = neutral();
    set(start, "leg_joint1", 0.8);
    const auto body = pose(start, "body_link");
    const auto& r = body.rotation();
    EXPECT_GT(std::abs(std::atan2(-r(2, 0), std::hypot(r(0, 0), r(1, 0)))), 0.5236);
    EXPECT_TRUE(algo->isFullQValid(algo->toEigenConfig(start)));
    Eigen::VectorXd result;
    ASSERT_TRUE(algo->solveWholeBodyIK(algo->toEigenConfig(start),
        pose(start, "gripper_base_l_link"), std::nullopt, result,
        {1.0, 1.0, 1.0, 0.2}, 0.5236, 2.0, 0, false));
    EXPECT_TRUE(result.isApprox(algo->toEigenConfig(start), 1e-10));
    auto invalid = start;
    set(invalid, "leg_joint1", 10.0);
    EXPECT_FALSE(algo->isFullQValid(algo->toEigenConfig(invalid)));
    set(invalid, "leg_joint1", std::numeric_limits<double>::quiet_NaN());
    EXPECT_FALSE(algo->isFullQValid(algo->toEigenConfig(invalid)));
    set(start, "Joint6_L", 1.0);
    set(start, "Joint7_L", 1.0);
    EXPECT_FALSE(algo->solveWholeBodyIK(algo->toEigenConfig(start),
        pose(start, "gripper_base_l_link"), std::nullopt, result,
        {1.0, 1.0, 1.0, 0.2}, 0.5236, 2.0, 0, false));
}

TEST_F(MergedPlanning, WholeBodyIkRejectsMissingGoalsAndInvalidState)
{
    const auto start = neutral();
    Eigen::VectorXd result;
    EXPECT_FALSE(algo->solveWholeBodyIK(algo->toEigenConfig(start), std::nullopt, std::nullopt, result));
    EXPECT_FALSE(algo->solveWholeBodyIK(Eigen::VectorXd::Zero(1),
        pose(start, "gripper_base_l_link"), std::nullopt, result));
}

TEST_F(MergedPlanning, SyncedVisionSerializesNewCloudAlongsideExistingFields)
{
    com_interfaces::msg::SyncedRobotVision input, output;
    input.camera_rgbd_pointcloud = cloudMessage("base_link");
    input.target_label = "box";
    input.joint_positions = {0.1, 0.2};
    input.data_valid = true;
    rclcpp::Serialization<com_interfaces::msg::SyncedRobotVision> serializer;
    rclcpp::SerializedMessage bytes;
    serializer.serialize_message(&input, &bytes);
    serializer.deserialize_message(&bytes, &output);
    EXPECT_EQ(output, input);
    EXPECT_EQ(algo->pointCloudToModelRoot(output.camera_rgbd_pointcloud, neutral())->size(), 1U);
}
}  // namespace
