#include "ds485_node.h"
#include <algorithm>
#include <cmath>
#include <limits>

void Ds485Node::Context(Side &s, const std::string &source, double deadline, bool stop, double ack_window_ms)
{
    ds485::TransactionContext cx;
    cx.source = source;
    cx.queued_ms = s.bus.QueuedMs();
    cx.deadline_ms = deadline;
    cx.stop = stop;
    cx.ack_window_ms = ack_window_ms;
    if (!stop && s.control->Monitoring())
    {
        cx.response_limit_ms = 1000. / s.control->cfg.feedback_frequency_hz;
    }
    if (!stop)
    {
        cx.interrupted = [&s] { return s.bus.Interrupted(); };
    }
    s.servo->Transactions().Context(cx);
}

bool Ds485Node::Feedback(Side &s)
{
    if (!s.servo->ReadStatus(&s.sample))
    {
        s.last_error = s.servo->LastError();
        s.inventory = false;
        return false;
    }
    s.last_valid_ms = s.sample.sampled_ms;
    s.last_error.clear();
    return true;
}

bool Ds485Node::Inventory(Side &s)
{
    s.inventory = false;
    if (!s.servo->Probe())
    {
        s.last_error = s.servo->LastError();
        return false;
    }
    std::string registers;
    bool complete = s.servo->InspectProtection(&registers);
    RCLCPP_INFO(get_logger(), "[%s] PROTECTION_INVENTORY%s", s.name.c_str(), registers.c_str());
    if (!complete)
    {
        s.last_error = "INVENTORY_INCOMPLETE " + registers;
        return false;
    }
    if (!s.servo->EnsureProtection())
    {
        s.last_error = s.servo->LastError();
        return false;
    }
    if (!s.gripper->Init())
    {
        s.last_error = s.gripper->LastError();
        return false;
    }
    s.inventory = Feedback(s);
    return s.inventory;
}

void Ds485Node::Enqueue(Side &s, Goal g)
{
    const auto reject = g.finish;
    Worker::Work work;
    work.run = [this, &s, g = std::move(g)]() mutable { Begin(s, std::move(g)); };
    work.reject = [reject](const char *reason)
    {
        const std::string why = reason;
        uint8_t result = GripperResult::RESULT_FAILED;
        if (why == "PREEMPTED")
        {
            result = GripperResult::RESULT_PREEMPTED;
        }
        if (why == "CANCELED")
        {
            result = GripperResult::RESULT_CANCELED;
        }
        if (reject)
        {
            reject(result, why, {});
        }
    };
    s.bus.Submit(Worker::Priority::Goal, std::move(work));
}

void Ds485Node::Finish(Side &s, uint8_t result, const std::string &reason)
{
    auto g = std::move(s.goal);
    s.goal.reset();
    if (g && g->finish)
    {
        g->finish(result, reason, s.sample);
    }
}

void Ds485Node::Begin(Side &s, Goal g)
{
    auto &ctl = *s.control;
    const auto &cfg = ctl.cfg;
    const auto reject = [&](const std::string &why)
    {
        if (g.finish)
        {
            g.finish(GripperResult::RESULT_FAILED, why, s.sample);
        }
    };
    if (s.goal)
    {
        // 先确认旧任务已停止，再为新任务从实际位置重新起步。
        const bool stopped = StopHardware(s, cfg.fault_stop_policy == "brake" ? 1 : 2);
        ctl.Cancel(stopped);
        Finish(s, stopped ? GripperResult::RESULT_PREEMPTED : GripperResult::RESULT_FAULT,
            stopped ? "PREEMPTED" : ctl.Reason());
    }
    if (ctl.Latched())
    {
        reject(ctl.Reason());
        return;
    }
    std::string error = cfg.Validate();
    if (!error.empty())
    {
        reject(error);
        return;
    }
    // 从首笔 I/O 起登记归属，抢占中的读失败留给随后执行的 stop/新命令结算。
    s.first = false;
    s.goal = std::move(g);
    const auto invalid = [&](const std::string &why)
    {
        if (!s.bus.Interrupted())
        {
            Finish(s, GripperResult::RESULT_FAILED, why);
        }
    };
    Context(s, "start");
    if (!s.inventory && !Inventory(s))
    {
        invalid(s.last_error);
        return;
    }
    if (!Feedback(s))
    {
        if (!s.bus.Interrupted() && ctl.Monitoring())
        {
            Fault(s, "FEEDBACK_STALE " + s.last_error, true);
        }
        else
        {
            invalid(s.last_error);
        }
        return;
    }
    if (s.sample.moving || s.sample.speed != 0)
    {
        Fault(s, "UNEXPECTED_MOTION");
        return;
    }
    error = ctl.Check(s.sample, s.port->NowMs());
    if (!error.empty())
    {
        Fault(s, error);
        return;
    }
    const auto &job = *s.goal;
    int32_t target = job.absolute ? job.angle : s.gripper->PositionToAngle(job.position);
    const double angle = target / 10.;
    if (angle < std::min(cfg.open_position_deg, cfg.close_limit_deg) ||
        angle > std::max(cfg.open_position_deg, cfg.close_limit_deg))
    {
        invalid("TARGET_OUT_OF_RANGE");
        return;
    }
    if (ctl.State() == Phase::Holding &&
        (target - s.sample.angle) * (cfg.close_limit_deg > cfg.open_position_deg ? 1 : -1) >= 0)
    {
        invalid("ALREADY_HOLDING");
        return;
    }
    const double speed = job.absolute ? (job.speed_dps ? job.speed_dps : s.gripper->MaxSpeed())
                                      : s.gripper->SpeedToDps(job.speed);
    if (speed > s.gripper->MaxSpeed() || speed <= 0)
    {
        invalid("SPEED_OUT_OF_RANGE");
        return;
    }
    s.pwm_cap = std::min<int>(s.gripper->TorqueToPwm(job.torque), cfg.pwm_close_limit);
    int current_cap = cfg.current_close_limit_ma;
    if (ctl.State() == Phase::Holding)
    {
        // 张开时保留保持阶段的限幅，避免尚未解除预紧就先提高输出。
        s.pwm_cap = std::min(s.pwm_cap, cfg.pwm_hold_limit);
        current_cap = cfg.current_hold_limit_ma;
    }
    const int timeout =
        std::min<uint32_t>(job.timeout ? job.timeout : default_timeout_ms_, cfg.motion_timeout_ms);
    s.last_topic_id = job.id;
    s.stop_confirmed = false;
    s.touched = true;
    const auto start_deadline = s.port->NowMs() + timeout;
    Context(s, "prepare", start_deadline);
    if (!s.servo->PrepareTrajectory(
            s.pwm_cap, current_cap, std::ceil(std::min(speed, cfg.close_velocity_deg_s))) ||
        !Feedback(s))
    {
        if (!s.bus.Interrupted())
        {
            Fault(s, s.servo->LastError(), true);
        }
        return;
    }
    const double now = s.port->NowMs();
    if (!ctl.Start(s.sample, target, speed, s.goal->grasp, now, static_cast<int>(start_deadline - now)))
    {
        Fault(s, ctl.Reason());
        return;
    }
    if (!SendTarget(s, s.sample.angle))
    {
        return;
    }
    if (s.goal && s.goal->accepted)
    {
        s.goal->accepted(GripperResult::RESULT_ARRIVED, "ACCEPTED", s.sample);
        s.goal->accepted = {};
        s.goal->finish = {};
    }
    Publish(s, true);
}

bool Ds485Node::SendTarget(Side &s, int32_t target)
{
    auto &ctl = *s.control;
    const auto &cfg = ctl.cfg;
    const double q = target / 10.;
    if (!s.sample.valid || q < std::min(cfg.open_position_deg, cfg.close_limit_deg) ||
        q > std::max(cfg.open_position_deg, cfg.close_limit_deg) || target < -32768 || target > 32767)
    {
        Fault(s, "INTERPOLATION_TARGET_INVALID");
        return false;
    }
    if (std::abs(q - s.sample.angle / 10.) > cfg.max_following_error_deg + 1e-8)
    {
        Fault(s, "FOLLOW_ERROR");
        return false;
    }
    double due = s.sample.sampled_ms + cfg.feedback_timeout_ms;
    const bool changes_hold = ctl.State() == Phase::Holding && std::abs(q - ctl.CommandDeg()) > 0.05;
    if ((ctl.Moving() || changes_hold) && ctl.LastSend() >= 0)
    {
        due = std::min(due, ctl.LastSend() + 50.);
        // 速度由相邻 0x69 的真实发送间隔决定，过早的时隙只等待至 100 Hz 下界。
        while (s.port->NowMs() < ctl.LastSend() + 10 && s.port->NowMs() < due && !s.bus.Interrupted())
        {
            s.port->SleepMs(1);
        }
    }
    if (s.bus.Interrupted())
    {
        return false;
    }
    if (s.port->NowMs() >= due)
    {
        Fault(s, "CONTROL_CADENCE_OR_FEEDBACK_DEADLINE", true);
        return false;
    }
    Context(s, "target", due);
    if (!s.servo->WriteInterpolated(target))
    {
        if (!s.bus.Interrupted())
        {
            Fault(s, s.servo->LastError(), true);
        }
        return false;
    }
    ctl.Sent(target, s.servo->LastTargetSendMs());
    return true;
}

bool Ds485Node::StopHardware(Side &s, uint8_t mode)
{
    // 停止优先通过一个新事务发出。写返回、开关读回、运动状态停止都必须确认。
    const auto &cfg = s.control->cfg;
    const double due = s.port->NowMs() + std::max(150, 6 * (cfg.serial_response_timeout_ms + 12));
    Context(s, "stop", due, true);
    uint8_t ack = 0, actual = 0;
    bool sent = false;
    bool ok = s.servo->WriteReg(DS485_REG_SWITCH, &mode, 1, &sent, &ack);
    // 保护短应答本身也能证明设备收帧。用随后两类读回判断刹停，保护状态不等于刹停失败。
    if (ok || (sent && (torque_mode_ || (ack & DS485_STATE_PROTECTION_MASK))))
    {
        ok = s.servo->ReadU8(DS485_REG_SWITCH, &actual) && actual == mode;
    }
    int settled = 0;
    if (ok)
    {
        do
        {
            ok = Feedback(s);
            if (!ok)
            {
                break;
            }
            bool stationary = !s.sample.moving && s.sample.speed == 0;
            if (torque_mode_ && mode == DS485_SWITCH_BRAKE)
            {
                stationary = stationary && s.sample.pwm == 0;
            }
            if (mode == DS485_SWITCH_HOLD)
            {
                stationary = stationary && std::abs(s.sample.position_error / 10.) <= cfg.tolerance_deg;
            }
            settled = stationary ? settled + 1 : 0;
            if (settled >= 2)
            {
                break;
            }
            s.port->SleepMs(2);
        } while (s.port->NowMs() < due);
    }
    s.stop_confirmed = ok && settled == 2;
    s.touched = !s.stop_confirmed;
    if (!s.stop_confirmed)
    {
        s.sample.valid = false;
        RCLCPP_ERROR(get_logger(), "[%s] STOP_UNCONFIRMED mode=%u %s", s.name.c_str(), mode,
            s.servo->LastError().c_str());
    }
    return s.stop_confirmed;
}

void Ds485Node::Stop(Side &s, uint8_t mode, bool cancel)
{
    const bool ok = StopHardware(s, mode);
    s.control->Cancel(ok);
    if (cancel)
    {
        Finish(s, ok ? GripperResult::RESULT_CANCELED : GripperResult::RESULT_FAULT,
            ok ? "CANCELED" : s.control->Reason());
    }
    Publish(s, true);
}

void Ds485Node::Fault(Side &s, const std::string &why, bool communication)
{
    const std::string cause = why;
    const auto before = s.control->Latched() ? s.published : s.control->State();
    const auto evidence = s.sample;
    s.control->Fail(cause, communication);
    RCLCPP_ERROR(get_logger(),
        "[%s] GRIPPER_FAULT from=%s reason=%s valid=%d age_ms=%.1f q_actual=%.1f q_cmd=%.1f "
        "error_deg=%.1f pwm=%d current_ma=%d temp_c=%d voltage_mv=%d",
        s.name.c_str(), ds485::Name(before), cause.c_str(), evidence.valid,
        s.last_valid_ms < 0 ? -1. : s.port->NowMs() - s.last_valid_ms, evidence.angle / 10.,
        s.control->CommandDeg(), evidence.position_error / 10., evidence.pwm, evidence.current,
        evidence.temperature, evidence.voltage);
    if (!StopHardware(s, s.control->cfg.fault_stop_policy == "brake" ? 1 : 2))
    {
        s.control->StopUnconfirmed();
    }
    uint8_t code = GripperResult::RESULT_FAULT;
    if (s.stop_confirmed && cause == "MOTION_TIMEOUT")
    {
        code = GripperResult::RESULT_TIMEOUT;
    }
    Finish(s, code, s.control->Reason());
    Publish(s, true);
}

void Ds485Node::Tick(Side &s)
{
    if (torque_mode_)
    {
        TorqueTick(s);
        return;
    }
    auto &ctl = *s.control;
    const auto &cfg = ctl.cfg;
    if (s.first)
    {
        s.first = false;
        Context(s, "initialization");
        if (Inventory(s))
        {
            if (s.sample.moving || s.sample.speed != 0)
            {
                Fault(s, "STARTUP_MOTION_DETECTED");
                return;
            }
            ctl.Ready(s.sample, s.port->NowMs());
        }
        Publish(s, true);
        return;
    }
    const bool active = ctl.Monitoring();
    double due = 0;
    if (active)
    {
        due = std::min(ctl.SampleTime() + cfg.feedback_timeout_ms,
            s.port->NowMs() + 1000. / cfg.feedback_frequency_hz);
    }
    if (ctl.Moving() && ctl.LastSend() >= 0)
    {
        due = std::min(due, ctl.LastSend() + 50.);
    }
    Context(s, active ? "control_feedback" : "health", due);
    const bool valid = Feedback(s);
    if (s.bus.Interrupted())
    {
        return;
    }
    const double now = s.port->NowMs();
    if (!valid)
    {
        if (active)
        {
            ctl.Lost(now, s.last_error);
            Fault(s, ctl.Reason(), true);
            return;
        }
        if (ctl.State() == Phase::Ready || ctl.State() == Phase::Empty || ctl.State() == Phase::Recovering)
        {
            // 空闲失联也撤销就绪状态，重连后由显式恢复重新核对硬件。
            ctl.Lost(now, s.last_error);
        }
    }
    else if (active)
    {
        auto decision = ctl.Step(s.sample, now);
        switch (decision.action)
        {
        case ds485::Action::Target:
            SendTarget(s, decision.target);
            break;
        case ds485::Action::HoldLimits:
        {
            const auto contact = s.sample;
            RCLCPP_INFO(get_logger(), "[%s] CONTACT_DETECTED q_actual=%.1f q_cmd=%.1f pwm=%d current_ma=%d",
                s.name.c_str(), contact.angle / 10., ctl.CommandDeg(), contact.pwm, contact.current);
            Context(s, "hold_limits", contact.sampled_ms + cfg.feedback_timeout_ms);
            if (!s.servo->SetCurrentLimit(cfg.current_hold_limit_ma) ||
                !s.servo->SetPwmLimit(std::min(s.pwm_cap, cfg.pwm_hold_limit)) || !Feedback(s))
            {
                if (!s.bus.Interrupted())
                {
                    Fault(s, "HOLD_LIMITS_UNCONFIRMED " + s.servo->LastError(), true);
                }
                return;
            }
            std::string why = ctl.Check(s.sample, s.port->NowMs());
            if (!why.empty())
            {
                Fault(s, why);
                return;
            }
            double echoed = (double(s.sample.angle) + s.sample.position_error) / 10.;
            if (s.sample.sampled_ms <= contact.sampled_ms ||
                std::abs(echoed - ctl.CommandDeg()) > cfg.tolerance_deg ||
                std::abs(s.sample.angle / 10. - contact.angle / 10.) > cfg.hold_drift_deg)
            {
                Fault(s, "HOLD_TARGET_UNCONFIRMED", true);
                return;
            }
            ctl.Held(s.port->NowMs());
            const bool grasp = s.goal && s.goal->grasp;
            Finish(s, grasp ? GripperResult::RESULT_BLOCKED : GripperResult::RESULT_FAILED,
                grasp ? "GRASPED" : "CONTACT_BEFORE_TARGET");
            Publish(s, true);
            break;
        }
        case ds485::Action::Arrived:
            s.touched = false;
            Finish(s, GripperResult::RESULT_ARRIVED, "ARRIVED");
            break;
        case ds485::Action::Empty:
            // 空抓到端点后消除残余跟踪误差，避免以闭合限流长期顶住机构。
            if (!StopHardware(s, cfg.fault_stop_policy == "brake" ? 1 : 2))
            {
                ctl.StopUnconfirmed();
                Finish(s, GripperResult::RESULT_FAULT, ctl.Reason());
            }
            else
            {
                Finish(s, GripperResult::RESULT_FAILED, "EMPTY_CLOSED");
            }
            break;
        case ds485::Action::Stop:
            Fault(s, ctl.Reason(), ctl.State() == Phase::Recovering);
            return;
        case ds485::Action::None:
            break;
        }
    }
    else if (!ctl.Latched() && s.inventory && cfg.Validate().empty())
    {
        const auto why = ctl.Check(s.sample, now);
        if (!why.empty())
        {
            Fault(s, why);
            return;
        }
    }
    Publish(s);
}
