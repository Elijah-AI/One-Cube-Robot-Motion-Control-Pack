#include "ds485_node.h"
#include <chrono>
#include <thread>

namespace ds485_driver {
struct EmbeddedTorque::Impl {
    std::shared_ptr<Ds485Node> node;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::atomic<bool> stopping{false};
    std::thread thread;
    bool started{false};

    static rclcpp::ExecutorOptions executorOptions(const rclcpp::Context::SharedPtr& context) {
        rclcpp::ExecutorOptions options;
        options.context = context;
        return options;
    }
    Impl(rclcpp::Context::SharedPtr context, const std::string& config,
         const std::array<bool, 2>& tools, GripperAdmission admission)
        : executor(executorOptions(context)) {
        if (!admission) throw std::invalid_argument("embedded DS485 requires motion admission");
        rclcpp::NodeOptions options;
        options.context(context).use_global_arguments(false);
        options.arguments({"--ros-args", "--params-file", config});
        options.append_parameter_override("control_strategy", std::string("torque"));
        if (!tools[0]) options.append_parameter_override("left.enabled", false);
        if (!tools[1]) options.append_parameter_override("right.enabled", false);
        node = std::make_shared<Ds485Node>(options, std::move(admission));
    }
};

EmbeddedTorque::EmbeddedTorque(rclcpp::Context::SharedPtr context, const std::string& config,
                               const std::array<bool, 2>& tools, GripperAdmission admission)
    : impl_(std::make_unique<Impl>(std::move(context),config,tools,std::move(admission))) {}
EmbeddedTorque::~EmbeddedTorque() { shutdown(); }
void EmbeddedTorque::start() {
    auto& impl = *impl_;
    if (impl.started) return;
    impl.stopping.store(false);
    impl.started = true;
    try {
        if (!impl.node->Start()) throw std::runtime_error("embedded DS485 torque startup failed");
        impl.executor.add_node(impl.node);
        impl.thread = std::thread([&impl] {
            const auto context = impl.node->get_node_base_interface()->get_context();
            while (!impl.stopping.load() && rclcpp::ok(context))
                impl.executor.spin_once(std::chrono::milliseconds(25));
        });
    } catch (...) {
        shutdown();
        throw;
    }
}
void EmbeddedTorque::requestStop() {
    impl_->node->StopActiveTorque();
}
void EmbeddedTorque::shutdown() {
    auto& impl = *impl_;
    if (!impl.started) return;
    impl.stopping.store(true);
    if (impl.thread.joinable()) {
        // spin_once is bounded and a ROS signal may already have invalidated the context.
        impl.thread.join();
        if (rclcpp::ok(impl.node->get_node_base_interface()->get_context()))
            impl.executor.remove_node(impl.node);
    }
    impl.node->Shutdown();
    impl.started = false;
}
bool EmbeddedTorque::busy(unsigned side) const {
    return side < 2 && impl_->node->TorqueBusy(side);
}
}
