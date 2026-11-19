#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ds485_interfaces/msg/gripper_command.hpp"
#include "ds485_interfaces/msg/gripper_result.hpp"
#include "ds485_interfaces/msg/gripper_stop.hpp"
#include "ds485_interfaces/msg/gripper_health.hpp"
#include "ds485_interfaces/msg/servo_state.hpp"
#include "ds485_interfaces/srv/servo_get_info.hpp"
#include "ds485_interfaces/srv/servo_move_angle.hpp"
#include "ds485_interfaces/srv/servo_read_register.hpp"
#include "ds485_interfaces/srv/servo_set_limits.hpp"
#include "ds485_interfaces/srv/servo_set_switch.hpp"
#include "ds485_interfaces/srv/servo_set_torque.hpp"
#include "ds485_interfaces/srv/servo_write_register.hpp"
#include "ds485_driver/bus_worker.h"
#include "ds485_driver/ds485_gripper.h"
#include "ds485_driver/trajectory_controller.h"
#include "ds485_driver/embedded_torque.h"
#include <atomic>
#include <optional>
#include <set>

namespace srv = ds485_interfaces::srv;
using namespace ds485_interfaces::msg;
using ds485::Phase;
using Worker = ds485::BusWorker;
using Reply = std::function<void(uint8_t, const std::string &, const ServoStatus &)>;
struct Goal
{
    std::optional<uint32_t> id;
    bool absolute = false, grasp = false;
    int32_t angle = 0;
    uint8_t position = 0, torque = 0, speed = 0;
    uint16_t speed_dps = 0;
    uint32_t timeout = 0;
    Reply finish, accepted;
};
struct TorqueSession
{
    // 本侧总线线程从首次准备输出到制动并恢复模式期间保留进入时的模式。
    std::optional<uint8_t> original_mode;
    // 非零输出成功发送后置位，开始停止时清零。制动是否已证实另见 stop_confirmed。
    bool active = false;
    // 最近一条命令或异常的处理结果，由总线线程发布到健康话题。
    std::string reason;
};
struct Side
{
    std::atomic<unsigned> queued_pwm{0};
    std::atomic<bool> output_owned{false};
    int index = 0;
    bool enabled = false;
    std::string name, endpoint;
    std::unique_ptr<Rs485Port> port;
    std::unique_ptr<Ds485Servo> servo;
    std::unique_ptr<Ds485Gripper> gripper;
    std::unique_ptr<ds485::GripperController> control;
    Worker bus;
    TorqueSession torque;
    // 以下由本侧总线线程独占。任务结果发送后清空 goal，仍由状态机继续监控 HOLDING。
    std::optional<Goal> goal;
    // 总线线程从新话题任务复制。保持和故障期间保留，以便关联成功后出现的异常。
    std::optional<uint32_t> last_topic_id;
    ServoStatus sample{};
    bool first = true, inventory = false, stop_confirmed = false, touched = false, warm = false;
    int pwm_cap = 0, gap_ms = 0;
    double last_state = -1e15, last_health = -1e15, last_warning = -1e15, last_valid_ms = -1;
    unsigned suppressed = 0;
    std::string last_error;
    Phase published = Phase::Uninitialized;
};
class Ds485Node : public rclcpp::Node
{
  public:
    explicit Ds485Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                       ds485_driver::GripperAdmission admission = {})
        : Node("ds485_node", "/", options), admission_(std::move(admission)) {}
    bool TorqueBusy(unsigned side) const {
        const auto& s = sides_[side];
        return s.queued_pwm.load() != 0 || s.output_owned.load();
    }
    bool Start();
    void StopActiveTorque();
    void Shutdown();

  private:
    template <class T> T Setting(const std::string &key, const T &fallback)
    {
        rcl_interfaces::msg::ParameterDescriptor d;
        d.read_only = true;
        return declare_parameter<T>(key, fallback, d);
    }
    int Int(const std::string &key, int fallback);
    ds485::SafetyProfile LoadSafety(
        const std::string &prefix, double open, double close, double tolerance, int timeout);
    bool LoadSide(int index, const std::string &name);
    bool Inventory(Side &s);
    void Routes();
    void TorqueRoute();
    void TorqueCommand(Side &s, const srv::ServoSetTorque::Request &q, srv::ServoSetTorque::Response &r);
    bool PrepareMotor(Side &s, std::string &why);
    bool StopMotor(Side &s, const std::string &why, srv::ServoSetTorque::Response *response = nullptr);
    void TorqueTick(Side &s);
    const char *TorquePhase(const Side &s) const;
    template <class Service, class F> void ServiceRoute(const std::string &name, F callback);
    template <class Service, class F> void Maintenance(const std::string &name, F op);
    void Command(const GripperCommand &msg);
    void Enqueue(Side &s, Goal goal);
    void Begin(Side &s, Goal goal);
    void Tick(Side &s);
    void Context(Side &s, const std::string &source, double deadline = 0, bool stop = false,
        double ack_window_ms = 0);
    bool Feedback(Side &s);
    bool SendTarget(Side &s, int32_t target);
    bool StopHardware(Side &s, uint8_t mode);
    void Stop(Side &s, uint8_t mode, bool cancel);
    void Fault(Side &s, const std::string &why, bool communication = false);
    void Finish(Side &s, uint8_t result, const std::string &reason);
    void Publish(Side &s, bool force = false);
    void State(Side &s, ServoState &msg);
    void Result(uint32_t id, uint8_t side, uint8_t code, const std::string &why, const ServoStatus &sample);
    Side sides_[2];
    bool use_marvin_ = false, allow_raw_write_ = false, raw_hex_ = false;
    ds485_driver::GripperAdmission admission_;  // Set only when embedded in the motion controller.
    bool sdk_attached_ = false;
    bool torque_mode_ = false;
    int default_timeout_ms_ = 10000;
    double state_rate_hz_ = 10;
    std::set<std::string> endpoints_;
    rclcpp::Publisher<ServoState>::SharedPtr states_;
    rclcpp::Publisher<GripperResult>::SharedPtr results_;
    rclcpp::Publisher<GripperHealth>::SharedPtr health_;
    rclcpp::Subscription<GripperCommand>::SharedPtr command_;
    rclcpp::Subscription<GripperStop>::SharedPtr stop_;
    std::vector<rclcpp::ServiceBase::SharedPtr> services_;
};
