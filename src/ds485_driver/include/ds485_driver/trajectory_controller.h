#pragma once
#include "ds485_driver/ds485_servo.h"
#include <string>

namespace ds485
{
struct SafetyProfile
{
    bool limits_verified = false, interpolation_verified = false, enable_auto_preload = false;
    int control_mode = 1, control_frequency_hz = 25, feedback_frequency_hz = 25, health_frequency_hz = 2;
    int close_direction = 0; // 0 由实际标定端点推导
    double open_position_deg = 0, close_limit_deg = 0, tolerance_deg = 0;
    double close_velocity_deg_s = -1, close_acceleration_deg_s2 = -1, max_following_error_deg = -1;
    int pwm_contact_threshold = -1, pwm_contact_delta_threshold = -1, pwm_free = -1;
    int pwm_close_limit = -1, pwm_hold_limit = -1, pwm_hard_limit = -1;
    int current_contact_ma = -1, current_close_limit_ma = -1, current_hold_limit_ma = -1,
        current_hard_limit_ma = -1;
    int temperature_warning_c = -1, temperature_stop_c = -1;
    double contact_error_deg = -1, contact_min_fraction = -1, contact_max_fraction = -1;
    int contact_confirm_ms = -1, startup_guard_ms = -1, filter_ms = -1;
    int feedback_timeout_ms = -1, serial_response_timeout_ms = 300, read_max_retries = 0;
    int motion_timeout_ms = -1, max_close_duration_ms = -1, max_hold_duration_ms = -1;
    int hold_overload_duration_ms = -1, max_feedback_failures = 3;
    double hold_drift_deg = -1, unload_step_deg = 0, max_unload_deg = 0;
    std::string fault_stop_policy = "lock";
    std::string Validate() const;
};
enum class Phase
{
    Uninitialized,
    Ready,
    Opening,
    Closing,
    Contact,
    Holding,
    Releasing,
    Empty,
    Recovering,
    Fault
};
const char *Name(Phase p);
enum class Action
{
    None,
    Target,
    HoldLimits,
    Arrived,
    Empty,
    Stop
};
struct Decision
{
    Action action = Action::None;
    int32_t target = 0; // 仅 Target 有效，单位 0.1°
};
class GripperController
{
  public:
    explicit GripperController(const SafetyProfile &config) : cfg(config) {}
    const SafetyProfile cfg;
    Phase State() const
    {
        return phase_;
    }
    const std::string &Reason() const
    {
        return reason_;
    }
    double CommandDeg() const
    {
        return cmd_;
    }
    double LastSend() const
    {
        return last_send_;
    }
    double SampleTime() const
    {
        return sample_;
    }
    double PeakPwm() const
    {
        return peak_pwm_;
    }
    double PeakCurrent() const
    {
        return peak_current_;
    }
    double MaxAge() const
    {
        return max_age_;
    }
    double MaxJitter() const
    {
        return max_jitter_;
    }
    bool Warm() const
    {
        return warm_;
    }
    bool Moving() const;
    bool Monitoring() const;
    bool Latched() const;
    std::string Check(const ServoStatus &s, double now) const;
    bool Ready(const ServoStatus &s, double now);
    bool Start(const ServoStatus &s, int32_t target, double speed, bool grasp, double now, int timeout_ms);
    Decision Step(const ServoStatus &s, double now);
    Decision Lost(double now, const std::string &why);
    Decision Fail(const std::string &why, bool communication = false);
    void Sent(int32_t target, double now); // 传输受理后记录，下一次新反馈再核对目标
    void Held(double now);
    void Cancel(bool confirmed);
    void StopUnconfirmed();

  private:
    void Transition(Phase state, const std::string &reason);
    double Direction() const
    {
        return cfg.close_limit_deg > cfg.open_position_deg ? 1. : -1.;
    }
    // 此状态只归该侧总线线程。Start 从实测位置建立新轨迹，Sent 才提交发送目标；故障只由明确恢复离开。
    Phase phase_ = Phase::Uninitialized;
    std::string reason_ = "NOT_READY";
    // Start 以 s.angle 起步。Step 计算 planned_，Sent 只在传输层受理后更新 cmd_ 和 last_send_。
    double goal_ = 0, cmd_ = 0, planned_ = 0, speed_ = 0, velocity_ = 0, start_ = 0, deadline_ = 0;
    // sample_ 是最近参与控制的整批反馈时间。任务重建时丢弃旧样本，重复或过期样本不能推进目标。
    double sample_ = -1, last_send_ = -1, contact_since_ = -1, overload_since_ = -1, hold_start_ = 0;
    // 接触确认时保存机械位置。保持期间只累加已规划的卸载量，下一条新任务清零。
    double hold_anchor_ = 0, unload_ = 0, pwm_ = 0, current_ = 0, peak_pwm_ = 0, peak_current_ = 0,
           max_age_ = 0, max_jitter_ = 0;
    unsigned arrivals_ = 0, missed_ = 0;
    bool grasp_ = false, warm_ = false;
};
} // namespace ds485
