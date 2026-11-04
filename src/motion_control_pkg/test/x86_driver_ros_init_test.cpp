#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "MarvinSDK.h"

int main(int argc, char **argv)
{
    int byte_order = -1;
    if (CheckSDKTypeCompat(&byte_order) < 0 || (byte_order != 0 && byte_order != 1)) {
        std::cerr << "Marvin SDK ABI compatibility check failed\n";
        return 1;
    }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("x86_driver_link_smoke_test");
    std::cout << "ROS node initialized with the real SDK loaded; no hardware initialization\n";
    node.reset();
    rclcpp::shutdown();
    return 0;
}
