#include <rclcpp/rclcpp.hpp>
#include "ArmControlNode.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
