#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "vision_calibration/calibration_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vision_calibration::CalibrationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
