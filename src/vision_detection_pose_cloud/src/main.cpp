#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "vision_detection_pose_cloud/detection_pose_cloud_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vision_detection_pose_cloud::DetectionPoseCloudServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
