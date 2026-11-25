#include "ds485_node.h"
#include <rcutils/logging.h>
#include "ds485_driver/marvin_rs485_port.h"
#include "ds485_driver/serial_rs485_port.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <sys/stat.h>
namespace
{
// 单帧 0x83 可容纳更多，但每多一台，停止确认和逐台读反馈都要多占一轮总线。
constexpr int kMaxServosPerSide = 8;
// 多台同步帧启动确认的下限，至少覆盖固件起转和一轮反馈。
constexpr int kMinStartConfirmMs = 50;
// 多台共线的最小发送间隔。
constexpr int kMinSharedGapMs = 2;
int32_t Tenth(double angle)
{
    return static_cast<int32_t>(std::lround(angle * 10));
}
} // namespace
int Ds485Node::Int(const std::string &key, int fallback)
{
    const auto value = Setting<int64_t>(key, fallback);
    if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max())
    {
        throw std::invalid_argument(key + " 超出整数范围");
    }
    return static_cast<int>(value);
}
ds485::SafetyProfile Ds485Node::LoadSafety(
    const std::string &p, double open, double close, double tolerance, int timeout)
{
    ds485::SafetyProfile cfg;
    cfg.open_position_deg = open;
    cfg.close_limit_deg = close;
    cfg.tolerance_deg = tolerance;
    cfg.serial_response_timeout_ms = timeout;
    cfg.limits_verified = Setting<bool>(p + "limits_verified", cfg.limits_verified);
    cfg.interpolation_verified = Setting<bool>(p + "interpolation_verified", cfg.interpolation_verified);
    cfg.enable_auto_preload = Setting<bool>(p + "enable_auto_preload", cfg.enable_auto_preload);
    cfg.control_mode = Int(p + "control_mode", cfg.control_mode);
    cfg.control_frequency_hz = Int(p + "control_frequency_hz", cfg.control_frequency_hz);
    cfg.feedback_frequency_hz = Int(p + "feedback_frequency_hz", cfg.feedback_frequency_hz);
    cfg.health_frequency_hz = Int(p + "health_frequency_hz", cfg.health_frequency_hz);
    cfg.close_direction = Int(p + "close_direction", cfg.close_direction);
    cfg.pwm_contact_threshold = Int(p + "pwm_contact_threshold", cfg.pwm_contact_threshold);
    cfg.pwm_contact_delta_threshold =
        Int(p + "pwm_contact_delta_threshold", cfg.pwm_contact_delta_threshold);
    cfg.pwm_free = Int(p + "pwm_free", cfg.pwm_free);
    cfg.pwm_close_limit = Int(p + "pwm_close_limit", cfg.pwm_close_limit);
    cfg.pwm_hold_limit = Int(p + "pwm_hold_limit", cfg.pwm_hold_limit);
    cfg.pwm_hard_limit = Int(p + "pwm_hard_limit", cfg.pwm_hard_limit);
    cfg.current_contact_ma = Int(p + "current_contact_ma", cfg.current_contact_ma);
    cfg.current_close_limit_ma = Int(p + "current_close_limit_ma", cfg.current_close_limit_ma);
    cfg.current_hold_limit_ma = Int(p + "current_hold_limit_ma", cfg.current_hold_limit_ma);
    cfg.current_hard_limit_ma = Int(p + "current_hard_limit_ma", cfg.current_hard_limit_ma);
    cfg.temperature_warning_c = Int(p + "temperature_warning_c", cfg.temperature_warning_c);
    cfg.temperature_stop_c = Int(p + "temperature_stop_c", cfg.temperature_stop_c);
    cfg.contact_confirm_ms = Int(p + "contact_confirm_ms", cfg.contact_confirm_ms);
    cfg.startup_guard_ms = Int(p + "startup_guard_ms", cfg.startup_guard_ms);
    cfg.filter_ms = Int(p + "filter_ms", cfg.filter_ms);
    cfg.feedback_timeout_ms = Int(p + "feedback_timeout_ms", cfg.feedback_timeout_ms);
    cfg.serial_response_timeout_ms = Int(p + "serial_response_timeout_ms", cfg.serial_response_timeout_ms);
    cfg.read_max_retries = Int(p + "read_max_retries", cfg.read_max_retries);
    cfg.motion_timeout_ms = Int(p + "motion_timeout_ms", cfg.motion_timeout_ms);
    cfg.max_close_duration_ms = Int(p + "max_close_duration_ms", cfg.max_close_duration_ms);
    cfg.max_hold_duration_ms = Int(p + "max_hold_duration_ms", cfg.max_hold_duration_ms);
    cfg.hold_overload_duration_ms = Int(p + "hold_overload_duration_ms", cfg.hold_overload_duration_ms);
    cfg.max_feedback_failures = Int(p + "max_feedback_failures", cfg.max_feedback_failures);
    cfg.open_position_deg = Setting<double>(p + "open_position_deg", cfg.open_position_deg);
    cfg.close_limit_deg = Setting<double>(p + "close_limit_deg", cfg.close_limit_deg);
    cfg.close_velocity_deg_s = Setting<double>(p + "close_velocity_deg_s", cfg.close_velocity_deg_s);
    cfg.close_acceleration_deg_s2 =
        Setting<double>(p + "close_acceleration_deg_s2", cfg.close_acceleration_deg_s2);
    cfg.max_following_error_deg =
        Setting<double>(p + "max_following_error_deg", cfg.max_following_error_deg);
    cfg.contact_error_deg = Setting<double>(p + "contact_error_deg", cfg.contact_error_deg);
    cfg.contact_min_fraction = Setting<double>(p + "contact_min_fraction", cfg.contact_min_fraction);
    cfg.contact_max_fraction = Setting<double>(p + "contact_max_fraction", cfg.contact_max_fraction);
    cfg.hold_drift_deg = Setting<double>(p + "hold_drift_deg", cfg.hold_drift_deg);
    cfg.unload_step_deg = Setting<double>(p + "unload_step_deg", cfg.unload_step_deg);
    cfg.max_unload_deg = Setting<double>(p + "max_unload_deg", cfg.max_unload_deg);
    cfg.fault_stop_policy = Setting<std::string>(p + "fault_stop_policy", cfg.fault_stop_policy);
    return cfg;
}
bool Ds485Node::LoadSide(int index, const std::string &name)
{
    Side &s = sides_[index];
    s.name = name;
    s.index = index;
    const std::string p = name + ".";

    s.enabled = Setting<bool>(p + "enabled", true);
    std::string port_type = Setting<std::string>(p + "port_type", "marvin");
    std::string arm = Setting<std::string>(p + "arm", index == 0 ? "A" : "B");
    int64_t channel = Setting<int64_t>(p + "channel", MARVIN_CH_COM1);
    std::string device = Setting<std::string>(p + "serial_device", "/dev/ttyUSB0");
    int64_t baud = Setting<int64_t>(p + "serial_baud", 115200);
    int64_t servo_id = Setting<int64_t>(p + "servo_id", 1);
    // 本侧从站：servo_ids 明确列出；未列出时由 servo_count 从 servo_id 起连续编号。0 = 未指定。
    const int64_t servo_count = Setting<int64_t>(p + "servo_count", 0);
    const auto servo_ids = Setting<std::vector<int64_t>>(p + "servo_ids", std::vector<int64_t>{});
    const auto directions = Setting<std::vector<int64_t>>(p + "servo_directions", std::vector<int64_t>{});
    const std::string sync_write = Setting<std::string>(p + "sync_write", "multi_id");
    const int64_t start_confirm_ms = Setting<int64_t>(p + "start_confirm_ms", 200);
    int64_t gap_ms = Setting<int64_t>(p + "gap_ms", 10);
    int64_t timeout_ms = Setting<int64_t>(p + "timeout_ms", 300);

    double open_deg = Setting<double>(p + "open_deg", 0.0);
    double close_deg = Setting<double>(p + "close_deg", 0.0);
    int64_t default_pwm = Setting<int64_t>(p + "default_pwm", 1000);
    int64_t max_speed = Setting<int64_t>(p + "max_speed", 0);
    int64_t default_speed = Setting<int64_t>(p + "default_speed", 0);
    double tolerance_deg = Setting<double>(p + "tolerance_deg", 1.0);
    double still_band_deg = Setting<double>(p + "still_band_deg", 0.3);
    int64_t blocked_samples = Setting<int64_t>(p + "blocked_samples", 10);

    const std::string protection_key = p + "protection.";
    ServoProtectionConfig protection;
    protection.auto_recover = Setting<bool>(protection_key + "auto_recover", true);
    protection.voltage_high_v = Setting<int64_t>(protection_key + "voltage_high_v", -1);
    protection.voltage_low_v = Setting<int64_t>(protection_key + "voltage_low_v", -1);
    protection.temperature_trip_c = Setting<int64_t>(protection_key + "temperature_trip_c", -1);
    protection.temperature_hysteresis_c = Setting<int64_t>(protection_key + "temperature_hysteresis_c", -1);
    protection.current_ma = Setting<int64_t>(protection_key + "current_ma", -1);
    protection.current_detect_ms = Setting<int64_t>(protection_key + "current_detect_ms", -1);
    protection.stall_pwm = Setting<int64_t>(protection_key + "stall_pwm", -1);
    protection.stall_detect_ms = Setting<int64_t>(protection_key + "stall_detect_ms", -1);

    ds485::SafetyProfile cfg;
    if (torque_mode_)
    {
        cfg.serial_response_timeout_ms = timeout_ms;
    }
    else
    {
        cfg = LoadSafety(p + "safety.", open_deg, close_deg, tolerance_deg, timeout_ms);
        open_deg = cfg.open_position_deg;
        close_deg = cfg.close_limit_deg;
    }
    if (!s.enabled)
    {
        RCLCPP_INFO(get_logger(), "[%s] 已在配置中关闭", name.c_str());
        return true;
    }

    if (admission_) {
        const auto &overrides = get_node_parameters_interface()->get_parameter_overrides();
        const auto first = overrides.lower_bound(p);
        if (first == overrides.end() || first->first.compare(0, p.size(), p) != 0) {
            RCLCPP_ERROR(get_logger(), "Missing ds485_node configuration for %s", name.c_str());
            return false;
        }
        if (port_type == "marvin" && (arm != (index == 0 ? "A" : "B") || channel < 2 || channel > 3)) {
            RCLCPP_ERROR(get_logger(), "[%s] shared SDK requires its matching arm and COM1/COM2", name.c_str());
            return false;
        }
    }

    // ---------- 参数检查 ----------
    if (servo_count < 0 || servo_count > kMaxServosPerSide)
    {
        RCLCPP_ERROR(get_logger(), "[%s] servo_count 范围为 0~%d（0 表示按 servo_ids 或 1 台）", name.c_str(),
            kMaxServosPerSide);
        return false;
    }
    std::vector<int64_t> ids = servo_ids;
    if (ids.empty())
    {
        for (int64_t i = 0; i < std::max<int64_t>(servo_count, 1); ++i)
        {
            ids.push_back(servo_id + i);
        }
    }
    else if (servo_count > 0 && servo_count != static_cast<int64_t>(ids.size()))
    {
        RCLCPP_ERROR(get_logger(), "[%s] servo_count=%ld 与 servo_ids 的 %zu 台不一致", name.c_str(),
            static_cast<long>(servo_count), ids.size());
        return false;
    }
    if (ids.size() > static_cast<size_t>(kMaxServosPerSide))
    {
        RCLCPP_ERROR(get_logger(), "[%s] 每侧最多 %d 台舵机", name.c_str(), kMaxServosPerSide);
        return false;
    }
    std::set<int64_t> unique;
    for (int64_t id : ids)
    {
        if (id < 0 || id > DS485_ID_MAX || !unique.insert(id).second)
        {
            RCLCPP_ERROR(get_logger(), "[%s] 舵机 ID 必须为 0~250 且互不重复", name.c_str());
            return false;
        }
    }
    if (!directions.empty() && directions.size() != ids.size())
    {
        RCLCPP_ERROR(get_logger(), "[%s] servo_directions 须与舵机数量相同", name.c_str());
        return false;
    }
    for (int64_t direction : directions)
    {
        if (direction != 1 && direction != -1)
        {
            RCLCPP_ERROR(get_logger(), "[%s] servo_directions 只能填 1 或 -1", name.c_str());
            return false;
        }
    }
    ds485::SyncWrite sync = ds485::SyncWrite::MultiId;
    if (!ds485::ParseSyncWrite(sync_write, &sync))
    {
        RCLCPP_ERROR(get_logger(), "[%s] sync_write 只能是 multi_id、staged 或 sequential", name.c_str());
        return false;
    }
    if (start_confirm_ms < 0 || start_confirm_ms > 5000)
    {
        RCLCPP_ERROR(get_logger(), "[%s] start_confirm_ms 范围 0~5000", name.c_str());
        return false;
    }
    if (ids.size() > 1 && sync != ds485::SyncWrite::Sequential && start_confirm_ms < kMinStartConfirmMs)
    {
        // 同步帧没有应答，启动确认是发现某台漏收的唯一途径，不允许关闭。
        RCLCPP_ERROR(get_logger(),
            "[%s] 多台使用 %s 时 start_confirm_ms 不能小于 %d（同步帧无应答，须靠反馈确认每台启动）。"
            "不需要确认时改用 sync_write: sequential",
            name.c_str(), sync_write.c_str(), kMinStartConfirmMs);
        return false;
    }
    if (ids.size() > 1 && gap_ms < kMinSharedGapMs)
    {
        // 多台共线时，上一台回包结束到下一帧之间要留出收发器换向时间。
        RCLCPP_ERROR(get_logger(), "[%s] 一侧多台时 gap_ms 不能小于 %d", name.c_str(), kMinSharedGapMs);
        return false;
    }
    if (!torque_mode_ && ids.size() > 1)
    {
        // 位置轨迹、接触和保持按单台标定，多台同轴时不能只驱动其中一台。
        RCLCPP_ERROR(get_logger(),
            "[%s] position 模式每侧只支持 1 台舵机，当前配置 %zu 台。多台请用 control:=torque，"
            "或把 servo_ids 改为单台",
            name.c_str(), ids.size());
        return false;
    }
    if (gap_ms < 0 || gap_ms > 1000 || timeout_ms < 10 || timeout_ms > 5000)
    {
        RCLCPP_ERROR(get_logger(), "[%s] gap_ms 范围 0~1000，timeout_ms 范围 10~5000", name.c_str());
        return false;
    }
    if (default_pwm < 1 || default_pwm > 1000 || max_speed < 0 || max_speed > 65535 || default_speed < 0 ||
        default_speed > 65535 || blocked_samples < 1)
    {
        RCLCPP_ERROR(get_logger(), "[%s] default_pwm 范围 1~1000，速度范围 0~65535，blocked_samples >= 1",
            name.c_str());
        return false;
    }

    if (!std::isfinite(open_deg) || !std::isfinite(close_deg) || open_deg < -3276.8 || open_deg > 3276.7 ||
        close_deg < -3276.8 || close_deg > 3276.7 || !std::isfinite(tolerance_deg) || tolerance_deg < 0 ||
        tolerance_deg > 6553 || !std::isfinite(still_band_deg) || still_band_deg < 0 ||
        still_band_deg > 6553 || cfg.serial_response_timeout_ms < 2 ||
        cfg.serial_response_timeout_ms > 5000 || cfg.read_max_retries < 0 || cfg.read_max_retries > 1 ||
        cfg.health_frequency_hz < 1 || cfg.health_frequency_hz > 100 || cfg.control_frequency_hz < 20 ||
        cfg.control_frequency_hz > 100)
    {
        RCLCPP_ERROR(get_logger(), "[%s] INVALID_CONFIGURATION", name.c_str());
        return false;
    }
    // ---------- 第 1 层：总线 ----------
    if (port_type == "marvin")
    {
        if ((arm != "A" && arm != "B") || channel < 1 || channel > 3)
        {
            RCLCPP_ERROR(
                get_logger(), "[%s] arm 只能是 A/B，channel 只能是 1(CANFD)/2(COM1)/3(COM2)", name.c_str());
            return false;
        }
        // SDK 的接收 FIFO 按臂共享，两个通道也不能各设一个读取者。
        s.endpoint = "marvin:" + arm;
        s.port.reset(new MarvinRs485Port(arm[0], (int)channel));
        use_marvin_ = true;
    }
    else if (port_type == "serial")
    {
        struct stat info{};
        if (::stat(device.c_str(), &info) == 0 && S_ISCHR(info.st_mode))
        {
            s.endpoint = "serial:" + std::to_string(info.st_rdev);
        }
        else
        {
            s.endpoint = "serial:" + device;
        }
        SerialRs485Port *serial = new SerialRs485Port();
        s.port.reset(serial);
        if (!serial->Open(device, (int)baud))
        {
            RCLCPP_ERROR(
                get_logger(), "[%s] SERIAL_OPEN_FAILED %s", name.c_str(), serial->LastError().c_str());
            return false;
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "[%s] port_type 只能是 marvin 或 serial", name.c_str());
        return false;
    }

    if (!endpoints_.insert(s.endpoint).second)
    {
        RCLCPP_ERROR(get_logger(), "DUPLICATE_BUS %s", s.endpoint.c_str());
        return false;
    }
    // ---------- 第 2 层：舵机 ----------
    // 同一总线的全部舵机共用事务管理器：一问一答、超时隔离和统计都按整条总线计算。
    s.transactions = std::make_unique<ds485::SerialTransactionManager>(s.port.get());
    s.gap_ms = static_cast<int>(gap_ms);
    std::vector<Ds485Servo *> members;
    std::string id_text;
    for (size_t i = 0; i < ids.size(); ++i)
    {
        Motor m;
        m.servo.reset(new Ds485Servo(s.port.get(), static_cast<uint8_t>(ids[i]), s.transactions.get()));
        m.direction = directions.empty() ? 1 : static_cast<int>(directions[i]);
        m.servo->SetTiming(s.gap_ms, cfg.serial_response_timeout_ms);
        members.push_back(m.servo.get());
        id_text += (id_text.empty() ? "" : ",") + std::to_string(ids[i]) + (m.direction < 0 ? "(-)" : "");
        s.motors.push_back(std::move(m));
    }
    s.servo = s.motors[0].servo.get();
    // 单台时普通写能校验短应答，同步写没有意义。
    s.group = std::make_unique<ds485::Ds485ServoGroup>(
        members, ids.size() == 1 ? ds485::SyncWrite::Sequential : sync);
    s.start_confirm_ms = static_cast<int>(start_confirm_ms);
    s.control = std::make_unique<ds485::GripperController>(cfg);
    auto &transactions = *s.transactions;
    transactions.Options(cfg.read_max_retries, raw_hex_);
    transactions.SharedBus(s.motors.size() > 1);
    transactions.Logger(
        [this, &s](bool warning, const std::string &line)
    {
        const char *phase = torque_mode_ ? "TORQUE" : ds485::Name(s.control->State());
        if (!warning)
        {
            RCLCPP_DEBUG(get_logger(), "[%s] state=%s %s", s.name.c_str(), phase, line.c_str());
            return;
        }
        const double now = s.port->NowMs();
        if (now - s.last_warning >= 1000 || raw_hex_)
        {
            RCLCPP_WARN(get_logger(), "[%s] state=%s suppressed=%u %s", s.name.c_str(), phase, s.suppressed,
                line.c_str());
            s.last_warning = now;
            s.suppressed = 0;
        }
        else
        {
            ++s.suppressed;
        }
    }, [this]
    { return rcutils_logging_logger_is_enabled_for(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG); });
    for (auto &m : s.motors)
    {
        if (!torque_mode_ && !m.servo->SetProtectionConfig(protection))
        {
            RCLCPP_ERROR(get_logger(), "[%s] %s", name.c_str(), m.servo->LastError().c_str());
            return false;
        }
        // 保护日志正文已带舵机 ID。
        m.servo->SetFaultLogger([this, name](const std::string &message)
        { RCLCPP_WARN(get_logger(), "[%s] %s", name.c_str(), message.c_str()); });
    }

    // ---------- 第 3 层：夹爪 ----------
    GripperConfig c;
    c.open_angle = Tenth(open_deg);
    c.close_angle = Tenth(close_deg);
    c.default_pwm = (uint16_t)default_pwm;
    c.max_speed = (uint16_t)max_speed;
    c.default_speed = (uint16_t)default_speed;
    c.tolerance = Tenth(tolerance_deg);
    c.still_band = Tenth(still_band_deg);
    c.blocked_samples = (int)blocked_samples;
    s.gripper.reset(new Ds485Gripper(s.servo));
    s.gripper->SetConfig(c);

    RCLCPP_INFO(get_logger(),
        "[%s] 总线 %s，舵机 ID %s（%zu 台，同步方式 %s，启动确认 %d ms），张开 %.1f° / 闭合 %.1f°，默认 PWM %d",
        name.c_str(), s.port->Name().c_str(), id_text.c_str(), s.motors.size(), ds485::Name(s.group->Mode()),
        s.motors.size() > 1 ? s.start_confirm_ms : 0, open_deg, close_deg, (int)default_pwm);
    return true;
}

bool Ds485Node::Start()
{
    std::string ip = Setting<std::string>("marvin_ip", "192.168.1.190");
    const auto strategy = Setting<std::string>("control_strategy", "position");
    if (strategy != "position" && strategy != "torque")
    {
        RCLCPP_ERROR(get_logger(), "control_strategy 只支持 position 或 torque");
        return false;
    }
    torque_mode_ = strategy == "torque";
    state_rate_hz_ = Setting<double>("state_rate_hz", 10.);
    Int("poll_ms", 20); // 兼容旧参数文件，轨迹周期由 control_frequency_hz 决定。
    default_timeout_ms_ = Int("default_timeout_ms", 10000);
    allow_raw_write_ = Setting<bool>("allow_raw_write", false);
    raw_hex_ = Setting<bool>("trace_hex", false);
    if (!std::isfinite(state_rate_hz_) || state_rate_hz_ < 0 || state_rate_hz_ > 100 ||
        default_timeout_ms_ < 1)
    {
        return false;
    }
    if (!LoadSide(0, "left") || !LoadSide(1, "right") || (!sides_[0].enabled && !sides_[1].enabled))
    {
        return false;
    }
    std::string why;
    if (use_marvin_) {
        if (admission_) {
            MarvinLink::Attach();
        } else if (!MarvinLink::Connect(ip, &why)) {
            RCLCPP_ERROR(get_logger(), "SERIAL_OPEN_FAILED %s", why.c_str());
            return false;
        }
        sdk_attached_ = true;
    }
    if (!why.empty())
    {
        RCLCPP_WARN(get_logger(), "%s", why.c_str());
    }
    states_ = create_publisher<ServoState>("ds485/state", 10);
    results_ = create_publisher<GripperResult>("ds485/gripper/result", 10);
    health_ = create_publisher<GripperHealth>("ds485/gripper/health", 10);
    Routes();
    for (auto &s : sides_)
    {
        if (!s.enabled)
        {
            continue;
        }
        s.bus.Start([this, &s] { Tick(s); }, [this, &s]
        {
            double hz = s.control->Monitoring()
                            ? s.control->cfg.control_frequency_hz
                            : std::max(state_rate_hz_, double(s.control->cfg.health_frequency_hz));
            return 1000. / hz;
        });
    }
    return true;
}
void Ds485Node::Shutdown()
{
    for (auto &s : sides_)
    {
        s.bus.RequestClose([this, &s]
        {
            if (torque_mode_)
            {
                if (s.touched || ModePending(s))
                {
                    StopMotor(s, "SHUTDOWN");
                }
                return;
            }
            if (s.touched || s.control->Monitoring())
            {
                Stop(s, s.control->cfg.fault_stop_policy == "brake" ? 1 : 2, true);
            }
        });
    }
    for (auto &s : sides_)
    {
        s.bus.Join();
    }
    if (sdk_attached_)
    {
        MarvinLink::Release();
        sdk_attached_ = false;
    }
}
