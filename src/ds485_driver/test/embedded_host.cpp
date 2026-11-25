// PTY test host. The link spy replaces the Marvin transport in this executable.
#include "ds485_driver/embedded_torque.h"
#include <std_srvs/srv/set_bool.hpp>
#include <atomic>

int main(int argc, char **argv) {
    if (argc != 4 || std::string(argv[2]) != "--params-file") return 2;
    rclcpp::init(0, nullptr);
    auto host = std::make_shared<rclcpp::Node>("test_gx12_01_host");
    std::atomic<bool> admitted{false};
    ds485_driver::EmbeddedTorque gripper(host->get_node_base_interface()->get_context(),
        argv[3], {false, true},
        [&](unsigned side, const std::function<void()>& submit, std::string& error) {
            if (side != 1 || !admitted) {
                error = "HOST_DENIED";
                return false;
            }
            submit();
            return true;
        });
    using Gate = std_srvs::srv::SetBool;
    auto gate = host->create_service<Gate>("test_gripper_admit",
        [&](Gate::Request::SharedPtr request, Gate::Response::SharedPtr response) {
            admitted = request->data;
            if (!admitted) gripper.requestStop();
            response->success = true;
        });
    gripper.start();
    rclcpp::spin(host);
    gripper.shutdown();
}
