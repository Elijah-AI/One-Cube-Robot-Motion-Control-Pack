#include "ds485_driver/gripper_controller.h"
#include <algorithm>
#include <cmath>
namespace ds485
{
namespace
{
bool finite_range(double v, double lo, double hi)
{
    return std::isfinite(v) && v >= lo && v <= hi;
}
} // namespace
std::string SafetyProfile::Validate() const
{
    if (!limits_verified || !interpolation_verified)
    {
        return "CALIBRATION_REQUIRED limits_verified/interpolation_verified";
    }
    if (enable_auto_preload)
    {
        return "AUTO_PRELOAD_UNSUPPORTED";
    }
    if (control_mode != 1 || control_frequency_hz < 20 || control_frequency_hz > 100 ||
        feedback_frequency_hz != control_frequency_hz || health_frequency_hz < 1 ||
        health_frequency_hz > feedback_frequency_hz)
    {
        return "INVALID_MODE_OR_FREQUENCY";
    }
    if (fault_stop_policy != "lock" && fault_stop_policy != "brake")
    {
        return "INVALID_STOP_POLICY";
    }
    if (!finite_range(open_position_deg, -3276.8, 3276.7) ||
        !finite_range(close_limit_deg, -3276.8, 3276.7) ||
        std::abs(open_position_deg - close_limit_deg) < 0.1 ||
        (close_direction != 0 && close_direction != (close_limit_deg > open_position_deg ? 1 : -1)))
    {
        return "INVALID_MECHANICAL_LIMIT";
    }
    if (!finite_range(tolerance_deg, 0.1, std::abs(open_position_deg - close_limit_deg) / 2) ||
        !finite_range(close_velocity_deg_s, 0.1, 65535) ||
        !finite_range(close_acceleration_deg_s2, 0.1, 1e7) ||
        !finite_range(max_following_error_deg, 0.1, std::abs(open_position_deg - close_limit_deg)))
    {
        return "MISSING_TRAJECTORY_LIMITS";
    }
    if (pwm_free < 0 || pwm_contact_threshold <= 0 || pwm_contact_delta_threshold <= 0 ||
        pwm_contact_delta_threshold > 1000 || pwm_free >= pwm_contact_threshold ||
        pwm_contact_threshold > pwm_close_limit || pwm_hold_limit <= 0 ||
        pwm_hold_limit > pwm_close_limit || pwm_close_limit > pwm_hard_limit || pwm_hard_limit > 1000)
    {
        return "MISSING_PWM_LIMITS";
    }
    if (current_contact_ma <= 0 || current_contact_ma > current_close_limit_ma ||
        current_hold_limit_ma <= 0 || current_hold_limit_ma > current_close_limit_ma ||
        current_close_limit_ma > current_hard_limit_ma || current_hard_limit_ma > 32767 ||
        temperature_warning_c <= 0 || temperature_stop_c <= temperature_warning_c ||
        temperature_stop_c > 255)
    {
        return "MISSING_CURRENT_OR_TEMPERATURE_LIMITS";
    }
    if (!finite_range(contact_error_deg, 0.1, max_following_error_deg) ||
        !finite_range(contact_min_fraction, 0, 1) || !finite_range(contact_max_fraction, 0, 1) ||
        contact_min_fraction >= contact_max_fraction || contact_max_fraction >= 1 ||
        contact_confirm_ms <= 0 || startup_guard_ms < 0 || filter_ms <= 0)
    {
        return "MISSING_CONTACT_CALIBRATION";
    }
    if (feedback_timeout_ms <= 0 || serial_response_timeout_ms <= 0 || serial_response_timeout_ms > 5000 ||
        read_max_retries < 0 || read_max_retries > 1 || max_feedback_failures < 1 ||
        motion_timeout_ms <= 0 || max_close_duration_ms <= 0 || max_hold_duration_ms <= 0 ||
        hold_overload_duration_ms <= 0)
    {
        return "MISSING_TIME_LIMITS";
    }
    if (!finite_range(hold_drift_deg, 0.1, std::abs(open_position_deg - close_limit_deg)) ||
        !finite_range(max_unload_deg, 0, hold_drift_deg) ||
        !finite_range(unload_step_deg, 0, max_unload_deg) ||
        ((unload_step_deg == 0) != (max_unload_deg == 0)) || (unload_step_deg > 0 && unload_step_deg < 0.1))
    {
        return "INVALID_HOLD_UNLOAD";
    }
    return {};
}
const char *Name(Phase p)
{
    switch (p)
    {
    case Phase::Uninitialized:
        return "UNINITIALIZED";
    case Phase::Ready:
        return "READY";
    case Phase::Opening:
        return "OPENING";
    case Phase::Closing:
        return "CLOSING";
    case Phase::Contact:
        return "CONTACT_DETECTED";
    case Phase::Holding:
        return "HOLDING";
    case Phase::Releasing:
        return "RELEASING";
    case Phase::Empty:
        return "EMPTY_CLOSED";
    case Phase::Recovering:
        return "RECOVERING";
    case Phase::Fault:
        return "FAULT";
    }
    return "UNKNOWN";
}
void GripperController::Transition(Phase p, const std::string &why)
{
    phase_ = p;
    reason_ = why;
}
bool GripperController::Moving() const
{
    return phase_ == Phase::Closing || phase_ == Phase::Opening || phase_ == Phase::Releasing;
}
bool GripperController::Monitoring() const
{
    return Moving() || phase_ == Phase::Holding || phase_ == Phase::Contact;
}
bool GripperController::Latched() const
{
    return phase_ == Phase::Fault || phase_ == Phase::Recovering;
}
std::string GripperController::Check(const ServoStatus &s, double now) const
{
    if (!s.valid || now < s.sampled_ms || now - s.sampled_ms > cfg.feedback_timeout_ms)
    {
        return "FEEDBACK_STALE";
    }
    if (s.fault)
    {
        return "DEVICE_FAULT " + ds485::FaultText(s.state);
    }
    if (s.temperature >= cfg.temperature_stop_c)
    {
        return "OVER_TEMPERATURE";
    }
    if (std::abs(int(s.pwm)) >= cfg.pwm_hard_limit)
    {
        return "PWM_HARD_LIMIT";
    }
    if (std::abs(int(s.current)) >= cfg.current_hard_limit_ma)
    {
        return "CURRENT_HARD_LIMIT";
    }
    const double q = s.angle / 10.;
    if (q < std::min(cfg.open_position_deg, cfg.close_limit_deg) - cfg.tolerance_deg ||
        q > std::max(cfg.open_position_deg, cfg.close_limit_deg) + cfg.tolerance_deg)
    {
        return "MECHANICAL_LIMIT";
    }
    return {};
}
bool GripperController::Ready(const ServoStatus &s, double now)
{
    auto error = cfg.Validate();
    if (error.empty())
    {
        error = Check(s, now);
    }
    if (!error.empty())
    {
        reason_ = error;
        return false;
    }
    sample_ = s.sampled_ms;
    missed_ = 0;
    Transition(Phase::Ready, "READY");
    return true;
}
bool GripperController::Start(
    const ServoStatus &s, int32_t target, double speed, bool grasp, double now, int timeout_ms)
{
    if (Latched())
    {
        return false;
    }
    bool releasing = phase_ == Phase::Holding;
    if (!Ready(s, now))
    {
        return false;
    }
    goal_ = target / 10.;
    if (!finite_range(goal_, std::min(cfg.open_position_deg, cfg.close_limit_deg),
            std::max(cfg.open_position_deg, cfg.close_limit_deg)) ||
        !finite_range(speed, 0.01, 65535) || timeout_ms <= 0)
    {
        reason_ = "INVALID_GOAL";
        return false;
    }
    cmd_ = planned_ = s.angle / 10.;
    warm_ = s.temperature >= cfg.temperature_warning_c;
    bool closing = Direction() * (goal_ - cmd_) >= 0;
    speed_ = std::min(speed, cfg.close_velocity_deg_s);
    grasp_ = grasp;
    start_ = now;
    deadline_ =
        now + std::min(timeout_ms, closing ? std::min(cfg.motion_timeout_ms, cfg.max_close_duration_ms)
                                           : cfg.motion_timeout_ms);
    velocity_ = 0;
    last_send_ = contact_since_ = overload_since_ = -1;
    arrivals_ = 0;
    pwm_ = peak_pwm_ = std::abs(int(s.pwm));
    current_ = peak_current_ = std::abs(int(s.current));
    unload_ = max_age_ = max_jitter_ = 0;
    Transition(closing ? Phase::Closing : (releasing ? Phase::Releasing : Phase::Opening), "NEW_GOAL");
    return true;
}
Decision GripperController::Fail(const std::string &why, bool communication)
{
    Transition(communication ? Phase::Recovering : Phase::Fault, why);
    return {Action::Stop, 0};
}
Decision GripperController::Lost(double now, const std::string &why)
{
    ++missed_;
    if (sample_ >= 0)
    {
        max_age_ = std::max(max_age_, now - sample_);
    }
    if (phase_ == Phase::Fault)
    {
        return {};
    }
    if (phase_ == Phase::Recovering)
    {
        if (missed_ >= unsigned(cfg.max_feedback_failures) || max_age_ >= cfg.feedback_timeout_ms)
        {
            Transition(Phase::Fault, "FEEDBACK_STALE " + why);
        }
        return {};
    }
    return Fail("FEEDBACK_STALE " + why, true);
}
void GripperController::Sent(int32_t target, double now)
{
    cmd_ = target / 10.;
    last_send_ = now;
}
void GripperController::Held(double now)
{
    hold_start_ = now;
    Transition(Phase::Holding, "GRASPED");
}
void GripperController::Cancel(bool confirmed)
{
    if (!confirmed)
    {
        StopUnconfirmed();
        return;
    }
    if (!Latched())
    {
        Transition(Phase::Ready, "CANCELED");
    }
}
void GripperController::StopUnconfirmed()
{
    if (reason_.find("STOP_UNCONFIRMED") == std::string::npos)
    {
        reason_ += " STOP_UNCONFIRMED";
    }
    phase_ = Phase::Fault;
}
Decision GripperController::Step(const ServoStatus &s, double now)
{
    if (!Monitoring())
    {
        return {};
    }
    const double previous = sample_;
    if (!s.valid || s.sampled_ms <= previous || now - s.sampled_ms > cfg.feedback_timeout_ms)
    {
        return Lost(now, "NO_FRESH_SAMPLE");
    }
    sample_ = s.sampled_ms;
    max_age_ = std::max(max_age_, now - sample_);
    peak_pwm_ = std::max(peak_pwm_, double(std::abs(int(s.pwm))));
    peak_current_ = std::max(peak_current_, double(std::abs(int(s.current))));
    auto error = Check(s, now);
    if (!error.empty())
    {
        return Fail(error, error == "FEEDBACK_STALE");
    }
    warm_ = s.temperature >= cfg.temperature_warning_c;
    const double period = 1000. / cfg.control_frequency_hz;
    const double elapsed = sample_ - previous;
    max_jitter_ = std::max(max_jitter_, std::abs(elapsed - period));
    const double dt = std::min(elapsed, period) / 1000.;
    const double alpha = 1 - std::exp(-elapsed / cfg.filter_ms);
    pwm_ += alpha * (std::abs(int(s.pwm)) - pwm_);
    current_ += alpha * (std::abs(int(s.current)) - current_);
    const double q = s.angle / 10.;
    const double sign = Direction();
    if (last_send_ >= 0 && std::abs((double(s.angle) + s.position_error) / 10. - cmd_) > cfg.tolerance_deg)
    {
        return Fail("TARGET_UNCONFIRMED", true);
    }
    if (Moving() && std::abs(cmd_ - q) > cfg.max_following_error_deg + 1e-8)
    {
        return Fail("FOLLOW_ERROR");
    }
    if (phase_ == Phase::Holding)
    {
        if (now - hold_start_ >= cfg.max_hold_duration_ms)
        {
            return Fail("HOLD_DURATION");
        }
        if (std::abs(q - hold_anchor_) > cfg.hold_drift_deg)
        {
            return Fail("HOLD_SLIP");
        }
        if (pwm_ < cfg.pwm_hold_limit && current_ < cfg.current_hold_limit_ma)
        {
            overload_since_ = -1;
            return {};
        }
        if (overload_since_ < 0)
        {
            overload_since_ = now;
        }
        if (now - overload_since_ < cfg.hold_overload_duration_ms)
        {
            return {};
        }
        if (!cfg.unload_step_deg)
        {
            return Fail("CANNOT_HOLD_SAFELY");
        }
        // 停留后首次卸载先发送冻结点重建 0x69 节拍。下一周期仍超载时才卸载。
        if (last_send_ < 0 || now - last_send_ > 50)
        {
            return {Action::Target, static_cast<int32_t>(std::lround(cmd_ * 10))};
        }
        double next = std::round((cmd_ - sign * cfg.unload_step_deg) * 10) / 10.;
        const double step = std::abs(next - cmd_);
        if (unload_ + step > cfg.max_unload_deg + 1e-8)
        {
            return Fail("CANNOT_HOLD_SAFELY");
        }
        if ((next - cfg.open_position_deg) * sign < 0)
        {
            return Fail("UNLOAD_LIMIT");
        }
        unload_ += step;
        overload_since_ = now;
        return {Action::Target, static_cast<int32_t>(std::lround(next * 10))};
    }
    if (now >= deadline_)
    {
        return Fail("MOTION_TIMEOUT");
    }
    if (last_send_ >= 0 && now - last_send_ > 50)
    {
        return Fail("CONTROL_CADENCE_LOST", true);
    }
    const double span = cfg.close_limit_deg - cfg.open_position_deg;
    const double fraction = (q - cfg.open_position_deg) / span;
    const bool closing = phase_ == Phase::Closing;
    bool contact =
        closing && now - start_ >= cfg.startup_guard_ms && fraction >= cfg.contact_min_fraction &&
        fraction <= cfg.contact_max_fraction &&
        (pwm_ >= cfg.pwm_contact_threshold || pwm_ - cfg.pwm_free >= cfg.pwm_contact_delta_threshold) &&
        (current_ >= cfg.current_contact_ma || sign * s.position_error / 10. >= cfg.contact_error_deg);
    if (contact && std::abs(q - cfg.open_position_deg) > cfg.tolerance_deg &&
        std::abs(q - cfg.close_limit_deg) > cfg.tolerance_deg)
    {
        if (contact_since_ < 0)
        {
            contact_since_ = sample_;
        }
        if (sample_ - contact_since_ >= cfg.contact_confirm_ms)
        {
            hold_anchor_ = q;
            Transition(Phase::Contact, "PWM_CONTACT");
            return {Action::HoldLimits, 0};
        }
        // 疑似接触期间维持上一目标，只确认负载，不继续增加预紧。
        return {Action::Target, static_cast<int32_t>(std::lround(cmd_ * 10))};
    }
    contact_since_ = -1;
    if (std::abs(cmd_ - goal_) < 0.05 && std::abs(q - goal_) <= cfg.tolerance_deg && !s.moving)
    {
        if (++arrivals_ >= 2)
        {
            if (grasp_)
            {
                Transition(Phase::Empty, "EMPTY_CLOSED");
                return {Action::Empty, 0};
            }
            Transition(Phase::Ready, "ARRIVED");
            return {Action::Arrived, 0};
        }
    }
    else
    {
        arrivals_ = 0;
    }
    const double direction = closing ? sign : -sign;
    const double remaining = std::max(0., direction * (goal_ - planned_));
    const double braking = std::sqrt(2 * cfg.close_acceleration_deg_s2 * remaining);
    const double desired = std::min(speed_, braking);
    const double dv = cfg.close_acceleration_deg_s2 * dt;
    velocity_ = std::clamp(desired, std::max(0., velocity_ - dv), velocity_ + dv);
    const double room = cfg.max_following_error_deg - direction * (planned_ - q);
    planned_ += direction * std::min({velocity_ * dt, remaining, std::max(0., room)});
    // 向实际位置方向取整，量化本身不能突破领先量或机械限位。
    double ticks = direction > 0 ? std::floor(planned_ * 10 + 1e-8) : std::ceil(planned_ * 10 - 1e-8);
    if (!finite_range(ticks, -32768, 32767))
    {
        return Fail("TARGET_OUT_OF_RANGE");
    }
    return {Action::Target, static_cast<int32_t>(ticks)};
}
} // namespace ds485
