#include "ds485_node.h"
#include <algorithm>
#include <cmath>

namespace
{
std::string Tag(const Motor &m)
{
    return "ID " + std::to_string(m.servo->Id()) + " ";
}

// 日志用：每台的角度和 PWM
std::string Samples(const Side &s)
{
    std::string text;
    for (const auto &m : s.motors)
    {
        text += " id" + std::to_string(m.servo->Id()) + "(angle_d10=" + std::to_string(m.sample.angle) +
                ",pwm=" + std::to_string(m.sample.pwm) + ")";
    }
    return text;
}
} // namespace

bool Ds485Node::ModePending(const Side &s)
{
    return std::any_of(
        s.motors.begin(), s.motors.end(), [](const Motor &m) { return m.original_mode.has_value(); });
}

Motor *Ds485Node::FindMotor(Side &s, int servo_id)
{
    if (servo_id < 0)
    {
        return s.motors.empty() ? nullptr : &s.motors[0];
    }
    for (auto &m : s.motors)
    {
        if (m.servo->Id() == servo_id)
        {
            return &m;
        }
    }
    return nullptr;
}

const char *Ds485Node::TorquePhase(const Side &s) const
{
    if (s.torque.active)
    {
        return "TORQUE_ACTIVE";
    }
    if (s.touched)
    {
        return "STOP_UNCONFIRMED";
    }
    if (ModePending(s))
    {
        return "MODE_RESTORE_PENDING";
    }
    for (const auto &m : s.motors)
    {
        if (!m.sample.valid)
        {
            return "OFFLINE";
        }
    }
    for (const auto &m : s.motors)
    {
        if (m.sample.fault)
        {
            return "TORQUE_FAULT";
        }
    }
    return "TORQUE_IDLE";
}

bool Ds485Node::MotorFeedback(Motor &m)
{
    if (!m.servo->ReadStatus(&m.sample))
    {
        m.last_error = m.servo->LastError();
        return false;
    }
    m.last_valid_ms = m.sample.sampled_ms;
    m.last_error.clear();
    return true;
}

std::string Ds485Node::MonitorMotors(Side &s)
{
    // 逐台读取。任何一台失联或保护都返回原因，由调用者同时停止整侧，避免单台继续顶住机构。
    const bool multi = s.motors.size() > 1;
    for (auto &m : s.motors)
    {
        if (!MotorFeedback(m))
        {
            return "FEEDBACK_LOST " + m.last_error;
        }
        if (m.sample.fault)
        {
            return "SERVO_FAULT " + Tag(m) + ds485::FaultText(m.sample.state);
        }
        const int expected = s.torque.pwm * m.direction;
        if (!s.torque.active || expected == 0)
        {
            continue;
        }
        if (m.sample.pwm != 0 && (m.sample.pwm > 0) == (expected > 0))
        {
            m.started = true;
            m.idle_rounds = 0;
        }
        else if (multi && m.started && ++m.idle_rounds >= 2)
        {
            // 已确认输出的一台连续两轮读不到同向 PWM：可能复位或漏收了更新帧，另一台不能单独继续。
            return "OUTPUT_LOST " + Tag(m) + "pwm=" + std::to_string(m.sample.pwm);
        }
    }
    if (!s.torque.active)
    {
        return "";
    }
    ++s.torque.rounds;
    if (!s.torque.verify_start)
    {
        return "";
    }
    // 同步帧没有应答。每台都要在确认窗内读到同向的非零 PWM，否则整侧停止。
    std::string missing;
    for (const auto &m : s.motors)
    {
        if (!m.started)
        {
            missing += (missing.empty() ? "" : ",") + std::to_string(m.servo->Id());
        }
    }
    if (missing.empty())
    {
        s.torque.verify_start = false;
        s.torque.reason = "PWM_START_CONFIRMED";
        RCLCPP_INFO(get_logger(), "[%s] MOTOR_START_CONFIRMED after_ms=%.1f%s", s.name.c_str(),
            s.port->NowMs() - s.torque.sent_ms, Samples(s).c_str());
        Publish(s, true);
        return "";
    }
    // 总线慢时一轮反馈可能超过确认窗。时间到且至少完整读过两轮仍未见输出才判失败，
    // 读不到反馈本身另按 FEEDBACK_LOST 停止。
    if (s.port->NowMs() - s.torque.sent_ms >= s.start_confirm_ms && s.torque.rounds >= 2)
    {
        return "START_UNCONFIRMED ID " + missing;
    }
    return "";
}

bool Ds485Node::StopMotors(Side &s, uint8_t mode, const std::vector<bool> *zero)
{
    // 同步方式先用一帧让整侧同时切换开关。同步帧无应答，而运行中开关寄存器可能本来就是 1，
    // 读回无法证明某台收到了这一帧，所以随后总是逐台单播：会话中的舵机先补零输出，再写开关并读回。
    // 零输出与开关写入都可重复执行。逐台写方式即单台流程：写入后读回，不重发。
    // 每台都要取得两次静止反馈才算停止。
    const auto &cfg = s.control->cfg;
    const size_t n = s.motors.size();
    const bool sync = s.group->Mode() != ds485::SyncWrite::Sequential;
    // 同步方式每台多一帧零输出，预算按每台的写、读回和静止确认放宽。
    const double due = s.port->NowMs() + std::max(150, (sync ? 8 : 6) * (cfg.serial_response_timeout_ms + 12)) *
                                             static_cast<double>(n);
    Context(s, "stop", due, true);
    if (sync)
    {
        const auto r = s.group->WriteSwitch(mode, nullptr, true);
        if (!r.ok)
        {
            RCLCPP_WARN(get_logger(), "[%s] SYNC_SWITCH mode=%u %s", s.name.c_str(), mode, r.error.c_str());
        }
    }
    // 先给每台写完零输出和开关，再逐台读回：一台失联时，其余舵机不必等它的读回超时才收到停止。
    std::vector<bool> failed(n, false);
    for (size_t i = 0; i < n; ++i)
    {
        Motor &m = s.motors[i];
        if (sync && zero && i < zero->size() && (*zero)[i])
        {
            bool sent = false;
            if (!m.servo->WriteMotorPwm(0, &sent))
            {
                RCLCPP_WARN(get_logger(), "[%s] MOTOR_ZERO_UNICAST id=%u sent=%d %s", s.name.c_str(),
                    m.servo->Id(), sent, m.servo->LastError().c_str());
            }
        }
        uint8_t ack = 0;
        bool sent = false;
        const bool ok = m.servo->WriteReg(DS485_REG_SWITCH, &mode, 1, &sent, &ack);
        // 保护短应答本身也能证明设备收帧。用随后两类读回判断刹停，保护状态不等于刹停失败。
        failed[i] = !(ok || sent);
    }
    for (size_t i = 0; i < n; ++i)
    {
        uint8_t actual = 0;
        failed[i] = failed[i] || !(s.motors[i].servo->ReadU8(DS485_REG_SWITCH, &actual) && actual == mode);
    }
    std::vector<int> settled(n, 0);
    for (;;)
    {
        bool pending = false;
        for (size_t i = 0; i < n; ++i)
        {
            Motor &m = s.motors[i];
            if (failed[i] || settled[i] >= 2)
            {
                continue;
            }
            if (!MotorFeedback(m))
            {
                failed[i] = true;
                continue;
            }
            bool stationary = !m.sample.moving && m.sample.speed == 0;
            if (mode == DS485_SWITCH_BRAKE)
            {
                stationary = stationary && m.sample.pwm == 0;
            }
            if (mode == DS485_SWITCH_HOLD)
            {
                stationary = stationary && std::abs(m.sample.position_error / 10.) <= cfg.tolerance_deg;
            }
            settled[i] = stationary ? settled[i] + 1 : 0;
            pending = pending || settled[i] < 2;
        }
        if (!pending || s.port->NowMs() >= due)
        {
            break;
        }
        s.port->SleepMs(2);
    }
    bool all = true;
    for (size_t i = 0; i < n; ++i)
    {
        Motor &m = s.motors[i];
        m.stop_confirmed = !failed[i] && settled[i] >= 2;
        if (!m.stop_confirmed)
        {
            all = false;
            m.sample.valid = false;
            RCLCPP_ERROR(get_logger(), "[%s] STOP_UNCONFIRMED id=%u mode=%u %s", s.name.c_str(), m.servo->Id(),
                mode, m.servo->LastError().c_str());
        }
    }
    s.stop_confirmed = all;
    s.touched = !all;
    return all;
}

bool Ds485Node::PrepareMotor(Side &s, std::string &why)
{
    Context(s, "motor_prepare");
    for (auto &m : s.motors)
    {
        if (!MotorFeedback(m))
        {
            why = m.last_error;
            return false;
        }
        if (m.sample.fault)
        {
            why = "SERVO_FAULT " + Tag(m) + ds485::FaultText(m.sample.state);
            return false;
        }
    }
    if (s.torque.active)
    {
        return true;
    }
    if (s.touched || ModePending(s))
    {
        why = "PREVIOUS_STOP_INCOMPLETE";
        return false;
    }
    for (const auto &m : s.motors)
    {
        if (m.sample.moving || m.sample.speed != 0)
        {
            why = "ALREADY_MOVING " + Tag(m);
            return false;
        }
    }
    // 先读全部舵机，确认都能进入电机模式后再登记原模式，避免只改动其中几台。
    std::vector<uint8_t> modes;
    bool brake = false;
    for (auto &m : s.motors)
    {
        uint8_t output = 0;
        if (!m.servo->Probe() || !m.servo->ReadU8(DS485_REG_SWITCH, &output))
        {
            why = m.servo->LastError();
            return false;
        }
        const uint8_t mode = m.servo->Mode();
        if (mode > 2)
        {
            why = "UNKNOWN_CONTROL_MODE " + Tag(m);
            return false;
        }
        modes.push_back(mode);
        brake = brake || output != DS485_SWITCH_BRAKE;
    }
    for (size_t i = 0; i < s.motors.size(); ++i)
    {
        s.motors[i].original_mode = modes[i];
    }
    if (brake && !StopMotors(s, DS485_SWITCH_BRAKE))
    {
        why = "PREPARE_BRAKE_UNCONFIRMED";
        return false;
    }
    Context(s, "motor_prepare");
    s.touched = true;
    s.stop_confirmed = false;
    for (auto &m : s.motors)
    {
        m.stop_confirmed = false;
    }
    // 0x5A 不在 0x83 开放的地址内，逐台写入并读回。运动帧发出前各台均已就绪，不影响同时启动。
    for (auto &m : s.motors)
    {
        if (*m.original_mode != 2 && !m.servo->WriteU8Check(DS485_REG_MODE, 2))
        {
            why = m.servo->LastError();
            return false;
        }
    }
    // 0x64=2 用于舵机位置保持。左侧固件在电机模式下拒绝该操作，直接由 0x6E 启动。
    return true;
}

bool Ds485Node::StopMotor(Side &s, const std::string &why, srv::ServoSetTorque::Response *response)
{
    // 制动优先。零输出帧的应答可能丢失，仍独立发 0x64 并用开关及两次新反馈确认。
    std::vector<bool> session(s.motors.size(), false);
    bool any = false;
    for (size_t i = 0; i < s.motors.size(); ++i)
    {
        session[i] = s.motors[i].original_mode.has_value();
        any = any || session[i];
    }
    s.torque.active = false;
    s.torque.verify_start = false;
    Context(s, "motor_zero", 0, true, 40);
    ds485::SyncResult zero;
    if (any)
    {
        // 逐台写时一台失联也继续给其余舵机发零输出；同步方式的单播补零在 StopMotors 中。
        zero = s.group->WriteMotorPwm(std::vector<int16_t>(s.motors.size(), 0), &session, true);
        if (!zero.ok)
        {
            RCLCPP_WARN(get_logger(), "[%s] MOTOR_ZERO %s", s.name.c_str(), zero.error.c_str());
        }
    }
    if (response)
    {
        response->sent = zero.sent;
        response->acknowledged = zero.ok && zero.acknowledged;
    }
    const bool stopped = StopMotors(s, DS485_SWITCH_BRAKE, &session);
    bool restored = stopped;
    std::string restore_error;
    if (stopped && any)
    {
        Context(s, "motor_restore", 0, true);
        for (auto &m : s.motors)
        {
            if (!m.original_mode)
            {
                continue;
            }
            const uint8_t original = *m.original_mode;
            uint8_t current = 0;
            bool ok = m.servo->ReadU8(DS485_REG_MODE, &current);
            if (ok && current != original)
            {
                ok = m.servo->WriteU8Check(DS485_REG_MODE, original);
            }
            if (ok)
            {
                m.original_mode.reset();
            }
            else
            {
                restored = false;
                restore_error += " " + m.servo->LastError();
            }
        }
    }
    s.output_owned.store(!stopped || !restored);
    s.torque.reason = why;
    if (!stopped)
    {
        s.torque.reason += " STOP_UNCONFIRMED";
    }
    else if (!restored)
    {
        s.torque.reason += " MODE_RESTORE_UNCONFIRMED" + restore_error;
    }
    RCLCPP_INFO(get_logger(), "[%s] MOTOR_STOP reason=%s brake_confirmed=%d mode_restored=%d%s",
        s.name.c_str(), s.torque.reason.c_str(), stopped, restored, Samples(s).c_str());
    Publish(s, true);
    return stopped && restored;
}

void Ds485Node::StopActiveTorque()
{
    for (auto &s : sides_) {
        if (!s.enabled || !TorqueBusy(s.index)) continue;
        Worker::Work stop;
        stop.run = [this, &s] {
            if (s.touched || ModePending(s) || s.output_owned.load())
                StopMotor(s, "HOST_STOP");
        };
        s.bus.Submit(Worker::Priority::Stop, std::move(stop));
    }
}

void Ds485Node::TorqueTick(Side &s)
{
    Context(s, "motor_feedback");
    if (s.torque.active)
    {
        const auto why = MonitorMotors(s);
        if (!why.empty() && !s.bus.Interrupted())
        {
            StopMotor(s, why);
            return;
        }
    }
    else
    {
        // 空闲时逐台更新，单台失联不影响其余舵机的状态发布。
        for (auto &m : s.motors)
        {
            if (s.bus.Interrupted())
            {
                break;
            }
            MotorFeedback(m);
        }
    }
    Publish(s);
}

void Ds485Node::TorqueCommand(Side &s, const srv::ServoSetTorque::Request &q, srv::ServoSetTorque::Response &r)
{
    if (q.pwm == 0)
    {
        r.success = StopMotor(s, "STOP_REQUESTED", &r);
        r.stop_confirmed = s.stop_confirmed;
        r.message = s.torque.reason;
        return;
    }
    std::string why;
    bool ok = PrepareMotor(s, why);
    ds485::SyncResult out;
    if (ok)
    {
        // 整侧一帧启动：0x83 由各舵机同时执行；单台时为普通写并校验短应答。实际 PWM 另见 0x49。
        std::vector<int16_t> pwm;
        for (const auto &m : s.motors)
        {
            pwm.push_back(static_cast<int16_t>(q.pwm * m.direction));
        }
        Context(s, "motor_pwm", 0, false, q.duration_ms);
        out = s.group->WriteMotorPwm(pwm);
        r.sent = out.sent;
        r.acknowledged = out.ok && out.acknowledged;
        ok = out.ok;
        if (!ok)
        {
            why = out.error;
        }
    }
    if (!ok)
    {
        if (ModePending(s) || s.touched)
        {
            StopMotor(s, why);
            r.stop_confirmed = s.stop_confirmed;
            why = s.torque.reason;
        }
        r.message = why;
        return;
    }
    const double started = out.sent_ms;
    s.touched = true;
    s.stop_confirmed = false;
    s.output_owned.store(true);
    s.torque.active = true;
    s.torque.pwm = q.pwm;
    s.torque.sent_ms = started;
    for (auto &m : s.motors)
    {
        m.started = false;
        m.idle_rounds = 0;
        m.stop_confirmed = false;
    }
    s.torque.rounds = 0;
    s.torque.verify_start = s.motors.size() > 1 && s.start_confirm_ms > 0;
    s.torque.reason = r.acknowledged ? "PWM_ACKNOWLEDGED" : "PWM_SENT_NO_ACK";
    RCLCPP_INFO(get_logger(), "[%s] MOTOR_PWM requested=%d duration_ms=%u sent_ms=%.1f ack=%d sync=%s servos=%zu",
        s.name.c_str(), q.pwm, q.duration_ms, started, r.acknowledged, ds485::Name(s.group->Mode()),
        s.motors.size());
    Publish(s, true);
    if (q.duration_ms == 0)
    {
        r.success = true;
        r.message = s.torque.reason;
        return;
    }

    // 此侧工作线程维护脉冲寿命。新目标、显式停止和节点退出均可打断等候；另一侧独立运行。
    // 预留下一帧收发间隔。每次反馈都受同一截止时间约束，应答延迟不累加到输出时长。
    const double stop_at = started + q.duration_ms - std::min<double>(s.gap_ms, q.duration_ms);
    // 一轮反馈逐台读取状态和 0x59，台数越多需要的余量越大；同步帧后另有一段静默。
    const bool sync = s.group->Mode() != ds485::SyncWrite::Sequential;
    const double round = (2. * s.gap_ms + 5) * static_cast<double>(s.motors.size()) +
                         (sync ? std::max(ds485::SerialTransactionManager::kNoReplySettleMs, s.gap_ms) : 0);
    double next_sample = started;
    why = "PULSE_COMPLETE";
    while (s.port->NowMs() < stop_at && !s.bus.Interrupted())
    {
        const double now = s.port->NowMs();
        if (now >= next_sample && stop_at - now > round)
        {
            Context(s, "motor_pulse_feedback", stop_at);
            const auto fault = MonitorMotors(s);
            if (!fault.empty())
            {
                // 到期或抢占造成的读失败按原因结束；保护和未启动始终如实报告。
                const bool lost = fault.rfind("FEEDBACK_LOST", 0) == 0;
                if (!lost || (s.port->NowMs() < stop_at && !s.bus.Interrupted()))
                {
                    why = fault;
                }
                break;
            }
            Publish(s);
            next_sample = s.port->NowMs() + 20;
        }
        else
        {
            s.port->SleepMs(1);
        }
    }
    if (s.bus.Interrupted())
    {
        why = "PULSE_INTERRUPTED";
    }
    const bool unverified = s.torque.verify_start;
    ok = StopMotor(s, why);
    r.success = ok && why == "PULSE_COMPLETE";
    r.stop_confirmed = s.stop_confirmed;
    r.message = s.torque.reason;
    if (unverified && why == "PULSE_COMPLETE")
    {
        // 脉冲短于一轮反馈时无法逐台读到输出，结果中注明。
        r.message += " START_UNVERIFIED";
    }
}
