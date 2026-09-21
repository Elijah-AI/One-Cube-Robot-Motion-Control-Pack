#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <functional>
#include <string>
#include <utility>

namespace motion_control_pkg::wbc {

// This group must run alongside the default model/FK callbacks on an executor
// with at least two workers. Acquisition must never access the planning model.
class FeedbackChannel {
public:
    using Acquire = std::function<bool(sensor_msgs::msg::JointState&, std::string&)>;

    FeedbackChannel(const rclcpp::Node::SharedPtr& node, Acquire acquire)
        : group_(node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)) {
        publisher_ = node->create_publisher<sensor_msgs::msg::JointState>(
            "/wbc/joint_states", rclcpp::SensorDataQoS());
        const rclcpp::Node::WeakPtr weak_node = node;
        timer_ = node->create_wall_timer(std::chrono::milliseconds(10),
            [this, weak_node, acquire = std::move(acquire)] {
                const auto node = weak_node.lock();
                if (!node) return;
                sensor_msgs::msg::JointState message;
                std::string error;
                if (acquire(message, error)) {
                    publisher_->publish(message);
                } else {
                    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                        "Measured WBC feedback withheld: %s", error.c_str());
                }
            }, group_);
    }

    rclcpp::CallbackGroup::SharedPtr callbackGroup() const { return group_; }

private:
    rclcpp::CallbackGroup::SharedPtr group_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace motion_control_pkg::wbc
