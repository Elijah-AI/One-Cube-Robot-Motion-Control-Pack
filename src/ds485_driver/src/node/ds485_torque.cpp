#include "ds485_node.h"
#include <algorithm>

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
    if (s.torque.original_mode)
    {
        return "MODE_RESTORE_PENDING";
    }
    if (!s.sample.valid)
    {
        return "OFFLINE";
    }
    return s.sample.fault ? "TORQUE_FAULT" : "TORQUE_IDLE";
}

bool Ds485Node::PrepareMotor(Side &s, std::string &why)
{
    Context(s, "motor_prepare");
    if (!Feedback(s))
    {
        why = s.last_error;
        return false;
    }
    if (s.sample.fault)
    {
        why = "SERVO_FAULT " + ds485::FaultText(s.sample.state);
        return false;
    }
    if (s.torque.active)
    {
        return true;
    }
    if (s.touched || s.torque.original_mode)
    {
        why = "PREVIOUS_STOP_INCOMPLETE";
        return false;
    }
    if (s.sample.moving || s.sample.speed != 0)
    {
        why = "ALREADY_MOVING";
        return false;
    }
    uint8_t output = 0;
    if (!s.servo->Probe() || !s.servo->ReadU8(DS485_REG_SWITCH, &output))
    {
        why = s.servo->LastError();
        return false;
    }
    const uint8_t mode = s.servo->Mode();
    if (mode > 2)
    {
        why = "UNKNOWN_CONTROL_MODE";
        return false;
    }
    s.torque.original_mode = mode;
    if (output != DS485_SWITCH_BRAKE && !StopHardware(s, DS485_SWITCH_BRAKE))
    {
        why = "PREPARE_BRAKE_UNCONFIRMED";
        return false;
    }
    Context(s, "motor_prepare");
    s.touched = true;
    s.stop_confirmed = false;
    if (mode != 2 && !s.servo->WriteU8Check(DS485_REG_MODE, 2))
    {
        why = s.servo->LastError();
        return false;
    }
    // 0x64=2 用于舵机位置保持。左侧固件在电机模式下拒绝该操作，直接由 0x6E 启动。
    return true;
}

bool Ds485Node::StopMotor(Side &s, const std::string &why, srv::ServoSetTorque::Response *response)
{
    // 制动优先。零输出帧的应答可能丢失，仍独立发 0x64 并用开关及两次新反馈确认。
    const bool session = s.torque.original_mode.has_value();
    s.torque.active = false;
    Context(s, "motor_zero", 0, true, 40);
    bool sent = false, zero_ok = false;
    if (session)
    {
        zero_ok = s.servo->WriteMotorPwm(0, &sent);
        if (!zero_ok)
        {
            RCLCPP_WARN(get_logger(), "[%s] MOTOR_ZERO %s", s.name.c_str(), s.servo->LastError().c_str());
        }
    }
    if (response)
    {
        response->sent = sent;
        response->acknowledged = zero_ok && s.servo->WriteAckEnabled();
    }
    const bool stopped = StopHardware(s, DS485_SWITCH_BRAKE);
    bool restored = stopped;
    if (stopped && session)
    {
        Context(s, "motor_restore", 0, true);
        const uint8_t original = *s.torque.original_mode;
        uint8_t current = 0;
        restored = s.servo->ReadU8(DS485_REG_MODE, &current);
        if (restored && current != original)
        {
            restored = s.servo->WriteU8Check(DS485_REG_MODE, original);
        }
        if (restored)
        {
            s.torque.original_mode.reset();
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
        s.torque.reason += " MODE_RESTORE_UNCONFIRMED " + s.servo->LastError();
    }
    RCLCPP_INFO(get_logger(),
        "[%s] MOTOR_STOP reason=%s brake_confirmed=%d mode_restored=%d angle_d10=%d pwm=%d",
        s.name.c_str(), s.torque.reason.c_str(), stopped, restored, s.sample.angle, s.sample.pwm);
    Publish(s, true);
    return stopped && restored;
}

void Ds485Node::StopActiveTorque()
{
    for (auto &s : sides_) {
        if (!s.enabled || !TorqueBusy(s.index)) continue;
        Worker::Work stop;
        stop.run = [this, &s] {
            if (s.touched || s.torque.original_mode || s.output_owned.load())
                StopMotor(s, "HOST_STOP");
        };
        s.bus.Submit(Worker::Priority::Stop, std::move(stop));
    }
}

void Ds485Node::TorqueTick(Side &s)
{
    Context(s, "motor_feedback");
    const bool ok = Feedback(s);
    if (s.torque.active && !s.bus.Interrupted() && (!ok || s.sample.fault))
    {
        const auto why = ok ? "SERVO_FAULT " + ds485::FaultText(s.sample.state) : "FEEDBACK_LOST " + s.last_error;
        StopMotor(s, why);
        return;
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
    if (ok)
    {
        // Marvin Send 表示 SDK 发送槽已受理。短应答由协议层校验，实际 PWM 另见 0x49。
        Context(s, "motor_pwm", 0, false, q.duration_ms);
        ok = s.servo->WriteMotorPwm(q.pwm, &r.sent);
        r.acknowledged = ok && s.servo->WriteAckEnabled();
        if (!ok)
        {
            why = s.servo->LastError();
        }
    }
    if (!ok)
    {
        if (s.torque.original_mode || s.touched)
        {
            StopMotor(s, why);
            r.stop_confirmed = s.stop_confirmed;
            why = s.torque.reason;
        }
        r.message = why;
        return;
    }
    const double started = s.servo->LastTargetSendMs();
    s.touched = true;
    s.stop_confirmed = false;
    s.output_owned.store(true);
    s.torque.active = true;
    s.torque.reason = r.acknowledged ? "PWM_ACKNOWLEDGED" : "PWM_SENT_NO_ACK";
    RCLCPP_INFO(get_logger(), "[%s] MOTOR_PWM requested=%d duration_ms=%u sent_ms=%.1f ack=%d",
        s.name.c_str(), q.pwm, q.duration_ms, started, r.acknowledged);
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
    double next_sample = started;
    why = "PULSE_COMPLETE";
    while (s.port->NowMs() < stop_at && !s.bus.Interrupted())
    {
        const double now = s.port->NowMs();
        if (now >= next_sample && stop_at - now > 2 * s.gap_ms + 5)
        {
            Context(s, "motor_pulse_feedback", stop_at);
            if (!Feedback(s))
            {
                if (s.port->NowMs() < stop_at && !s.bus.Interrupted())
                {
                    why = "FEEDBACK_LOST " + s.last_error;
                }
                break;
            }
            if (s.sample.fault)
            {
                why = "SERVO_FAULT " + ds485::FaultText(s.sample.state);
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
    ok = StopMotor(s, why);
    r.success = ok && why == "PULSE_COMPLETE";
    r.stop_confirmed = s.stop_confirmed;
    r.message = s.torque.reason;
}
