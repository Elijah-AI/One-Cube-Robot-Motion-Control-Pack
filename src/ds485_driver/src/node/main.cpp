#include "ds485_node.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Ds485Node>();
    int code = 0;
    try
    {
        if (node->Start())
        {
            rclcpp::spin(node);
        }
        else
        {
            code = 1;
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "%s", e.what());
        code = 1;
    }
    node->Shutdown();
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return code;
}
