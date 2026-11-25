#pragma once
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <functional>
#include <memory>
#include <string>

namespace ds485_driver {
using GripperAdmission = std::function<bool(unsigned, const std::function<void()>&, std::string&)>;

// The owner connects MarvinSDK before start() and releases it after shutdown().
class EmbeddedTorque {
public:
    EmbeddedTorque(rclcpp::Context::SharedPtr context, const std::string& device_config,
                   const std::array<bool, 2>& gripper_tools, GripperAdmission admission);
    ~EmbeddedTorque();
    void start();
    void requestStop();
    void shutdown();
    bool busy(unsigned side) const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
