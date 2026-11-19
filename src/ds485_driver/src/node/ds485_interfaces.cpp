#include "ds485_node.h"
#include <algorithm>
#include <cmath>

namespace
{
template <class F> void Deliver(F send, const rclcpp::Logger &logger)
{
    try
    {
        if (rclcpp::ok())
        {
            send();
        }
    }
    catch (const rclcpp::exceptions::RCLError &e)
    {
        if (rclcpp::ok())
        {
            RCLCPP_ERROR(logger, "ROS_DELIVERY_FAILED %s", e.what());
        }
    }
}
} // namespace
void Ds485Node::State(Side &s, ServoState &m)
{
    const auto &v = s.sample;
    m.side = s.index;
    m.online = v.valid;
    m.busy = s.torque.active || s.goal.has_value() || s.control->Monitoring();
    m.calibrated = s.gripper->Config().open_angle != s.gripper->Config().close_angle;
    m.error = v.valid ? "" : s.last_error;
    m.fault = s.control->Latched() || (v.valid && v.fault);
    if (s.control->Latched())
    {
        m.fault_text = s.control->Reason();
    }
    else if (v.valid && v.fault)
    {
        m.fault_text = ds485::FaultText(v.state);
    }
    if (!v.valid)
    {
        return;
    }
    m.state_raw = v.state;
    m.moving = v.moving;
    m.angle_deg = v.angle / 10.;
    m.speed_dps = v.speed;
    m.current_ma = v.current;
    m.pwm = v.pwm;
    m.position = s.gripper->AngleToPosition(v.angle);
}

void Ds485Node::Publish(Side &s, bool force)
{
    if (!rclcpp::ok())
    {
        return;
    }
    const double now = s.port->NowMs();
    const auto &c = *s.control;
    const bool transition = s.published != c.State();
    if (transition)
    {
        RCLCPP_INFO(get_logger(), "[%s] %s -> %s reason=%s q_cmd=%.1f", s.name.c_str(),
            ds485::Name(s.published), ds485::Name(c.State()), c.Reason().c_str(), c.CommandDeg());
        s.published = c.State();
    }
    const bool warning = s.sample.valid && c.cfg.temperature_warning_c > 0 &&
                         s.sample.temperature >= c.cfg.temperature_warning_c;
    if (warning && !s.warm)
    {
        RCLCPP_WARN(
            get_logger(), "[%s] TEMPERATURE_WARNING temp_c=%d", s.name.c_str(), s.sample.temperature);
    }
    if (s.sample.valid)
    {
        s.warm = warning;
    }
    if (force || transition || (state_rate_hz_ > 0 && now - s.last_state >= 1000. / state_rate_hz_))
    {
        ServoState m;
        State(s, m);
        Deliver([&] { states_->publish(m); }, get_logger());
        s.last_state = now;
    }
    if (!force && !transition && now - s.last_health < 1000. / c.cfg.health_frequency_hz)
    {
        return;
    }
    s.last_health = now;
    const auto &stats = s.servo->Transactions().Stats();
    GripperHealth h;
    h.side = s.index;
    h.has_command_id = s.last_topic_id.has_value();
    h.command_id = s.last_topic_id.value_or(0);
    h.phase = ds485::Name(c.State());
    h.reason = c.Reason();
    if (torque_mode_)
    {
        h.phase = TorquePhase(s);
        h.reason = s.torque.reason;
    }
    if (!s.sample.valid && !s.last_error.empty())
    {
        h.reason += " " + s.last_error;
    }
    h.feedback_valid = s.sample.valid;
    h.sample_monotonic_ms = s.last_valid_ms;
    h.feedback_age_ms = s.last_valid_ms < 0 ? -1 : now - s.last_valid_ms;
    h.command_deg = c.CommandDeg();
    h.temperature_warning = warning;
    h.stop_confirmed = s.stop_confirmed;
    if (s.sample.valid)
    {
        h.actual_deg = s.sample.angle / 10.;
        h.error_deg = s.sample.position_error / 10.;
        h.pwm = s.sample.pwm;
        h.current_ma = s.sample.current;
        h.temperature_c = s.sample.temperature;
        h.voltage_mv = s.sample.voltage;
    }
    h.transactions_ok = stats.ok;
    h.transactions_failed = stats.failed;
    h.read_retries = stats.retries;
    h.checksums_failed = stats.checksums;
    h.timeouts = stats.timeouts;
    h.max_consecutive_failures = stats.max_consecutive;
    h.latency_p50_ms = stats.Percentile(.5);
    h.latency_p95_ms = stats.Percentile(.95);
    h.latency_p99_ms = stats.Percentile(.99);
    h.latency_max_ms = stats.maximum;
    h.max_feedback_age_ms = c.MaxAge();
    h.max_cycle_jitter_ms = c.MaxJitter();
    h.peak_pwm = c.PeakPwm();
    h.peak_current_ma = c.PeakCurrent();
    Deliver([&] { health_->publish(h); }, get_logger());
}

void Ds485Node::Result(
    uint32_t id, uint8_t side, uint8_t result, const std::string &why, const ServoStatus &status)
{
    if (!rclcpp::ok())
    {
        return;
    }
    GripperResult m;
    m.id = id;
    m.side = side;
    m.result = result;
    m.message = why;
    m.success = result == GripperResult::RESULT_BLOCKED || result == GripperResult::RESULT_ARRIVED;
    if (status.valid && side < 2)
    {
        m.final_position = sides_[side].gripper->AngleToPosition(status.angle);
        m.final_angle_deg = status.angle / 10.;
    }
    Deliver([&] { results_->publish(m); }, get_logger());
    RCLCPP_INFO(get_logger(), "[side=%u] id=%u result=%u %s", side, id, result, why.c_str());
}

void Ds485Node::Command(const GripperCommand &m)
{
    if (m.side > 2)
    {
        Result(m.id, m.side, GripperResult::RESULT_FAILED, "INVALID_SIDE", {});
        return;
    }
    for (auto &s : sides_)
    {
        if (m.side != 2 && m.side != s.index)
        {
            continue;
        }
        if (!s.enabled)
        {
            Result(m.id, s.index, GripperResult::RESULT_FAILED, "SIDE_DISABLED", {});
            continue;
        }
        if (torque_mode_)
        {
            Result(m.id, s.index, GripperResult::RESULT_FAILED, "POSITION_MODE_REQUIRED", {});
            continue;
        }
        Goal g;
        g.grasp = m.position == 255;
        g.id = m.id;
        g.position = m.position;
        g.torque = m.torque;
        g.speed = m.speed;
        g.timeout = m.timeout_ms;
        g.finish = [this, id = m.id, side = s.index](uint8_t result, const std::string &why,
                       const ServoStatus &sample) { Result(id, side, result, why, sample); };
        Enqueue(s, std::move(g));
    }
}

// Humble 的延后应答签名。executor 接收请求后即让出线程，只有总线线程运行实际 I/O。
template <class Service, class F> void Ds485Node::ServiceRoute(const std::string &name, F cb)
{
    using Endpoint = rclcpp::Service<Service>;
    auto handle = [cb, logger = get_logger()](typename Endpoint::SharedPtr endpoint,
                      std::shared_ptr<rmw_request_id_t> header, typename Service::Request::SharedPtr req)
    {
        auto res = std::make_shared<typename Service::Response>();
        auto send = [endpoint, header, res, logger]
        { Deliver([&] { endpoint->send_response(*header, *res); }, logger); };
        cb(req, res, send);
    };
    services_.push_back(create_service<Service>(name, handle));
}
template <class Service, class F> void Ds485Node::Maintenance(const std::string &name, F op)
{
    ServiceRoute<Service>("ds485/servo/" + name, [this, op, name](auto req, auto res, auto send)
    {
        if (req->side >= 2 || !sides_[req->side].enabled)
        {
            res->message = "INVALID_SIDE";
            send();
            return;
        }
        Side &s = sides_[req->side];
        Worker::Work work;
        work.run = [this, &s, op, name, req, res, send]
        {
            if (s.torque.active || (s.touched && torque_mode_) || s.control->Monitoring() || s.goal)
            {
                res->message = "BUSY";
            }
            else
            {
                Context(s, name);
                op(s, *req, *res);
            }
            send();
        };
        work.reject = [res, send](const char *why)
        {
            res->message = why;
            send();
        };
        const auto submit = [&] { s.bus.Submit(Worker::Priority::Service, std::move(work)); };
        if (!admission_ || name == "get_info" || name == "read_register") submit();
        else if (!admission_(req->side, submit, res->message)) send();
    });
}

void Ds485Node::TorqueRoute()
{
    ServiceRoute<srv::ServoSetTorque>("ds485/servo/set_torque", [this](auto q, auto r, auto send)
    {
        if (!torque_mode_ || q->side > 1 || !sides_[q->side].enabled ||
            q->pwm < -DS485_PWM_MAX || q->pwm > DS485_PWM_MAX)
        {
            r->message = !torque_mode_ ? "TORQUE_MODE_REQUIRED" : "INVALID_SIDE_OR_PWM";
            send();
            return;
        }
        Side &s = sides_[q->side];
        Worker::Work work;
        work.run = [this, &s, q, r, send]
        {
            TorqueCommand(s, *q, *r);
            if (q->pwm != 0) --s.queued_pwm;
            send();
        };
        work.reject = [&s, q, r, send](const char *why)
        {
            if (q->pwm != 0) --s.queued_pwm;
            r->message = why;
            send();
        };
        if (q->pwm == 0) {
            s.bus.Submit(Worker::Priority::Stop, std::move(work));
            return;
        }
        const auto submit = [&] {
            ++s.queued_pwm;
            s.bus.Submit(Worker::Priority::Goal, std::move(work));
        };
        if (!admission_) submit();
        else if (!admission_(q->side, submit, r->message)) send();
    });
}

void Ds485Node::Routes()
{
    TorqueRoute();
    command_ = create_subscription<GripperCommand>(
        "ds485/gripper/command", 10, [this](GripperCommand::SharedPtr msg) { Command(*msg); });
    stop_ = create_subscription<GripperStop>("ds485/gripper/stop", 10, [this](GripperStop::SharedPtr msg)
    {
        for (auto &s : sides_)
        {
            if (s.enabled && (msg->side == 2 || msg->side == s.index))
            {
                Worker::Work w;
                w.run = [this, &s]
                {
                    if (torque_mode_)
                    {
                        StopMotor(s, "STOP_REQUESTED");
                    }
                    else
                    {
                        Stop(s, 2, true);
                    }
                };
                s.bus.Submit(Worker::Priority::Stop, std::move(w));
            }
        }
    });
    Maintenance<srv::ServoReadRegister>("read_register", [](Side &s, const auto &q, auto &r)
    {
        uint8_t data[DS485_FRAME_MAX];
        int n = 0;
        if (q.request.size() > DS485_FRAME_MAX - 7)
        {
            r.message = "INVALID_REQUEST";
            return;
        }
        r.success = s.servo->ReadReg(q.address, data, q.length, q.request.data(), q.request.size(), &n);
        r.tx_hex = s.servo->LastTx();
        r.rx_hex = s.servo->LastRx();
        r.message = r.success ? "ok" : s.servo->LastError();
        if (r.success)
        {
            r.data.assign(data, data + n);
        }
    });
    Maintenance<srv::ServoWriteRegister>("write_register", [this](Side &s, const auto &q, auto &r)
    {
        if (!allow_raw_write_)
        {
            r.message = "RAW_WRITE_DISABLED";
            return;
        }
        if (q.data.empty() || q.data.size() > DS485_FRAME_MAX - 7)
        {
            r.message = "INVALID_REQUEST";
            return;
        }
        // 生产节点的调试服务也不能通过原始寄存器绕开轨迹、限幅和硬件保护。
        if (q.address >= 0x59 || q.address == 0x02 || q.address == 0x03 || q.address == 0x46 ||
            q.address == 0x34)
        {
            r.message = "USE_CONTROLLED_INTERFACE";
            return;
        }
        if (q.address == 0x32)
        {
            uint8_t old = 0;
            if (q.data.size() != 1)
            {
                r.message = "INVALID_LENGTH";
                return;
            }
            if (!s.servo->ReadU8(0x32, &old))
            {
                r.message = s.servo->LastError();
                return;
            }
            if ((old & ~q.data[0]) != 0 || ((old ^ q.data[0]) & 0xf0) != 0)
            {
                r.message = "PROTECTION_BITS_REQUIRED";
                return;
            }
            r.success = s.servo->WriteU8Check(0x32, q.data[0]);
        }
        else
        {
            r.success = s.servo->WriteReg(q.address, q.data.data(), q.data.size());
        }
        s.inventory = false;
        r.tx_hex = s.servo->LastTx();
        r.rx_hex = s.servo->LastRx();
        r.message = r.success ? "WRITE_ACCEPTED" : s.servo->LastError();
    });
    Maintenance<srv::ServoSetLimits>("set_limits", [this](Side &s, const auto &q, auto &r)
    {
        if (q.pwm_limit < -1 || q.pwm_limit > 1000 || q.speed_limit < -1 || q.speed_limit > 65535 ||
            (torque_mode_ && q.speed_limit == 0) ||
            q.current_limit < -1 || q.current_limit > 65535)
        {
            r.message = "INVALID_LIMIT";
            return;
        }
        const auto &cfg = s.control->cfg;
        if (!torque_mode_ && (!cfg.Validate().empty() || q.pwm_limit > cfg.pwm_close_limit ||
            q.current_limit > cfg.current_close_limit_ma || q.speed_limit > std::ceil(cfg.close_velocity_deg_s) ||
            q.pwm_limit == 0 || q.current_limit == 0 || q.speed_limit == 0))
        {
            r.message = "CALIBRATION_OR_LIMIT";
            return;
        }
        s.inventory = false;
        r.success = (q.pwm_limit < 0 || s.servo->SetPwmLimit(q.pwm_limit)) &&
                    (q.speed_limit < 0 || s.servo->SetSpeedLimit(q.speed_limit)) &&
                    (q.current_limit < 0 || s.servo->SetCurrentLimit(q.current_limit));
        r.message = r.success ? "LIMITS_VERIFIED" : s.servo->LastError();
    });
    Maintenance<srv::ServoGetInfo>("get_info", [this](Side &s, const auto &, auto &r)
    {
        ServoInfo info{};
        r.bus_name = s.port->Name();
        r.success = s.servo->ReadInfo(&info) && Feedback(s);
        if (!r.success)
        {
            r.message = s.servo->LastError();
            r.state.side = s.index;
            r.state.error = r.message;
            return;
        }
        State(s, r.state);
        r.sys_config = info.sys_config;
        r.write_ack = info.write_ack;
        r.min_angle_deg = info.min_angle / 10.;
        r.max_angle_deg = info.max_angle / 10.;
        r.mode = info.mode;
        r.torque_switch = info.torque_switch;
        r.pwm_limit = info.pwm_limit;
        r.current_limit = info.current_limit;
        r.speed_limit = info.speed_limit;
        r.max_pwm = info.max_pwm;
        r.temperature = info.temperature;
        r.voltage = info.voltage / 1000.;
        r.message = "ok";
    });
    ServiceRoute<srv::ServoMoveAngle>("ds485/servo/move_angle", [this](auto q, auto r, auto send)
    {
        if (q->side >= 2 || !sides_[q->side].enabled || !std::isfinite(q->angle_deg) ||
            q->angle_deg < -3276.8 || q->angle_deg > 3276.7)
        {
            r->message = "INVALID_GOAL";
            send();
            return;
        }
        if (torque_mode_)
        {
            r->message = "POSITION_MODE_REQUIRED";
            send();
            return;
        }
        Goal g;
        g.absolute = true;
        g.angle = std::lround(q->angle_deg * 10.);
        g.speed_dps = q->speed_dps;
        g.timeout = q->timeout_ms ? q->timeout_ms : 5000;
        g.finish = [r, send](uint8_t code, const std::string &why, const ServoStatus &v)
        {
            r->success = code == GripperResult::RESULT_ARRIVED;
            r->message = why;
            if (v.valid)
            {
                r->final_angle_deg = v.angle / 10.;
            }
            send();
        };
        if (!q->wait)
        {
            g.accepted = g.finish;
        }
        Enqueue(sides_[q->side], std::move(g));
    });
    ServiceRoute<srv::ServoSetSwitch>("ds485/servo/set_switch", [this](auto q, auto r, auto send)
    {
        if (q->side >= 2 || !sides_[q->side].enabled || q->mode > 2)
        {
            r->message = "INVALID_SWITCH";
            send();
            return;
        }
        auto &s = sides_[q->side];
        Worker::Work w;
        w.run = [this, &s, q, r, send]
        {
            if (torque_mode_)
            {
                r->success = StopMotor(s, "STOP_REQUESTED");
                r->message = s.torque.reason;
                if (r->success && q->mode != DS485_SWITCH_BRAKE)
                {
                    // 显式释放或锁舵先终止电机输出并恢复原模式，再核对请求的开关状态。
                    r->success = StopHardware(s, q->mode);
                    r->message = r->success ? "SWITCH_VERIFIED" : "SWITCH_UNCONFIRMED";
                }
                send();
                return;
            }
            Stop(s, q->mode, true);
            r->success = s.stop_confirmed;
            r->message = r->success ? "SWITCH_VERIFIED" : "STOP_UNCONFIRMED";
            send();
        };
        w.reject = [r, send](const char *why)
        {
            r->message = why;
            send();
        };
        const auto submit = [&] { s.bus.Submit(Worker::Priority::Stop, std::move(w)); };
        if (!admission_ || q->mode == DS485_SWITCH_BRAKE) submit();
        else if (!admission_(q->side, submit, r->message)) send();
    });
    for (auto &s : sides_)
    {
        if (s.enabled)
        {
            ServiceRoute<std_srvs::srv::Trigger>("ds485/gripper/" + s.name + "/recover",
                [this, &s](auto, auto r, auto send)
            {
                Worker::Work w;
                w.run = [this, &s, r, send]
                {
                    if (s.torque.active || s.control->Monitoring() || s.goal)
                    {
                        r->message = "BUSY";
                        send();
                        return;
                    }
                    if (!s.port->RecoverLink())
                    {
                        r->message = "SERIAL_OPEN_FAILED " + s.port->LastError();
                        send();
                        return;
                    }
                    if (torque_mode_)
                    {
                        r->success = StopMotor(s, "RECOVERY_STOP");
                        r->message = s.torque.reason;
                        send();
                        return;
                    }
                    Context(s, "explicit_recovery");
                    if (s.touched && !StopHardware(s, s.control->cfg.fault_stop_policy == "brake" ? 1 : 2))
                    {
                        r->message = "STOP_UNCONFIRMED";
                        send();
                        return;
                    }
                    Context(s, "explicit_recovery");
                    if (!Inventory(s))
                    {
                        r->message = s.last_error;
                    }
                    else if (s.sample.moving || s.sample.speed != 0)
                    {
                        r->message = "STILL_MOVING";
                    }
                    else
                    {
                        r->success = s.control->Ready(s.sample, s.port->NowMs());
                        r->message = s.control->Reason();
                        if (r->success)
                        {
                            s.last_topic_id.reset();
                        }
                    }
                    Publish(s, true);
                    send();
                };
                w.reject = [r, send](const char *why)
                {
                    r->message = why;
                    send();
                };
                s.bus.Submit(Worker::Priority::Service, std::move(w));
            });
        }
    }
}
