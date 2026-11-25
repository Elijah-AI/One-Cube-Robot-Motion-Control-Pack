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
#include "ds485_driver/ds485_servo_group.h"
#include "ds485_driver/gripper_controller.h"
#include "ds485_driver/embedded_torque.h"
#include <atomic>
#include <optional>
#include <set>
#include <vector>

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
// 一侧总线上的一台从站。以下字段只由本侧总线线程访问。
struct Motor
{
    std::unique_ptr<Ds485Servo> servo;
    // 恒定 PWM 时乘到请求值上。同侧镜像安装的舵机配 -1，使同一请求产生同向夹持。
    int direction = 1;
    // 电机模式专用反馈。position 模式每侧只有一台，沿用 Side::sample。
    ServoStatus sample{};
    double last_valid_ms = -1;
    std::string last_error;
    // 从首次准备输出到制动并恢复模式期间保留进入时的 0x5A。
    std::optional<uint8_t> original_mode;
    // 本次输出后在反馈中读到与请求同向的非零 PWM。
    bool started = false;
    // 已启动后连续几轮读不到同向输出（复位、漏收或被固件截断）。多台时达到 2 轮整侧停止。
    int idle_rounds = 0;
    bool stop_confirmed = false;
};
struct TorqueSession
{
    // 非零输出成功发送后置位，开始停止时清零。制动是否已证实另见 stop_confirmed。
    bool active = false;
    // 最近一条命令或异常的处理结果，由总线线程发布到健康话题。
    std::string reason;
    int16_t pwm = 0;          // 当前请求，未乘方向
    double sent_ms = -1;      // 运动帧交给传输层的时刻
    bool verify_start = false; // 多台无应答同步帧：等待每台反馈出同向 PWM
    int rounds = 0;            // 本次输出后完整读完整侧反馈的轮数；未满 2 轮不判启动失败
};
struct Side
{
    std::atomic<unsigned> queued_pwm{0};
    std::atomic<bool> output_owned{false};
    int index = 0;
    bool enabled = false;
    std::string name, endpoint;
    std::unique_ptr<Rs485Port> port;
    // 同一总线的全部舵机共用一个事务管理器，隔离窗、迟到包清理和统计按整条总线计算。
    std::unique_ptr<ds485::SerialTransactionManager> transactions;
    // 配置顺序。motors[0] 是 position 模式的舵机和维护服务的默认对象。
    std::vector<Motor> motors;
    Ds485Servo *servo = nullptr; // motors[0].servo
    std::unique_ptr<ds485::Ds485ServoGroup> group;
    int start_confirm_ms = 0;
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
    // zero 非空时，同步方式下对其中为 true 的舵机在单播开关前先单播零输出。
    bool StopMotors(Side &s, uint8_t mode, const std::vector<bool> *zero = nullptr);
    static bool MotorFeedback(Motor &m);
    std::string MonitorMotors(Side &s);
    void TorqueTick(Side &s);
    const char *TorquePhase(const Side &s) const;
    static bool ModePending(const Side &s);
    static Motor *FindMotor(Side &s, int servo_id);
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
    void State(Side &s, const ServoStatus &sample, uint8_t servo_id, const std::string &error, ServoState &msg);
    void Health(Side &s, const ServoStatus &sample, uint8_t servo_id, double last_valid_ms,
        const std::string &error, bool stop_confirmed, bool warning, double now);
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
