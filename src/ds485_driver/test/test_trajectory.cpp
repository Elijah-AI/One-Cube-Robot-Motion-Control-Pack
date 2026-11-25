#include "ds485_driver/gripper_controller.h"
#include <cassert>
#include <cmath>
#include <cstdio>
using namespace ds485;
namespace
{
SafetyProfile bench(bool reverse = false)
{
    SafetyProfile p;
    p.limits_verified = p.interpolation_verified = true;
    p.open_position_deg = reverse ? 10 : 0;
    p.close_limit_deg = reverse ? 0 : 10;
    p.tolerance_deg = 0.2;
    p.close_velocity_deg_s = 10;
    p.close_acceleration_deg_s2 = 200;
    p.max_following_error_deg = 1;
    p.pwm_free = 20;
    p.pwm_contact_threshold = 100;
    p.pwm_contact_delta_threshold = 70;
    p.pwm_close_limit = 400;
    p.pwm_hold_limit = 200;
    p.pwm_hard_limit = 600;
    p.current_contact_ma = 50;
    p.current_close_limit_ma = 300;
    p.current_hold_limit_ma = 120;
    p.current_hard_limit_ma = 500;
    p.temperature_warning_c = 50;
    p.temperature_stop_c = 60;
    p.contact_error_deg = .5;
    p.contact_min_fraction = .1;
    p.contact_max_fraction = .9;
    p.contact_confirm_ms = 80;
    p.startup_guard_ms = 40;
    p.filter_ms = 10;
    p.feedback_timeout_ms = 100;
    p.motion_timeout_ms = 3000;
    p.max_close_duration_ms = 2000;
    p.max_hold_duration_ms = 1000;
    p.hold_overload_duration_ms = 80;
    p.hold_drift_deg = 1;
    return p;
}
struct Rig
{
    GripperController c;
    ServoStatus s{};
    int cmd;
    explicit Rig(SafetyProfile p = bench(), bool grasp = true)
        : c(p), cmd(std::lround(p.open_position_deg * 10))
    {
        s.angle = cmd;
        s.valid = true;
        s.temperature = 25;
        s.sampled_ms = 100;
        assert(c.Start(s, std::lround(p.close_limit_deg * 10), 10, grasp, 100, 2000));
        c.Sent(cmd, 100);
    }
    Decision tick(bool track = true)
    {
        if (track)
        {
            s.angle = cmd;
        }
        s.position_error = cmd - s.angle;
        s.sampled_ms += 40;
        auto d = c.Step(s, s.sampled_ms);
        if (d.action == Action::Target)
        {
            cmd = d.target;
            c.Sent(cmd, s.sampled_ms);
        }
        return d;
    }
    void contact()
    {
        while (cmd < 30)
        {
            tick();
        }
        s.pwm = 150;
        s.current = 70;
        tick();
        const int frozen = cmd;
        tick(false);
        assert(cmd == frozen && tick(false).action == Action::HoldLimits);
        c.Held(s.sampled_ms);
        assert(c.State() == Phase::Holding);
    }
};
} // namespace
void TestTrajectory()
{
    assert(!SafetyProfile{}.Validate().empty());
    for (bool reverse : {false, true})
    {
        Rig r(bench(reverse));
        for (int i = 0; i < 48 && r.c.Moving(); ++i)
        {
            const int old = r.cmd;
            r.tick();
            assert(std::abs(r.cmd - old) <= 4 && std::abs(r.cmd - r.s.angle) <= 10);
            assert(r.cmd >= 0 && r.cmd <= 100);
        }
        assert(r.c.State() == Phase::Empty && r.c.Reason() == "EMPTY_CLOSED");
    }
    Rig absolute(bench(), false);
    while (absolute.c.Moving())
    {
        absolute.tick();
    }
    assert(absolute.c.State() == Phase::Ready);
    Rig stalled;
    while (stalled.c.Moving())
    {
        stalled.tick(false);
        assert(stalled.cmd <= 10);
    }
    assert(stalled.c.Reason() == "MOTION_TIMEOUT");
    Rig lag;
    while (lag.cmd < 25)
    {
        lag.tick();
    }
    lag.s.angle = lag.cmd - 15;
    lag.s.pwm = 150;
    lag.s.current = 70;
    assert(lag.tick(false).action == Action::Stop && lag.c.Reason() == "FOLLOW_ERROR");
    Rig hard;
    hard.s.pwm = -600;
    assert(hard.tick().action == Action::Stop && hard.c.Reason() == "PWM_HARD_LIMIT");
    Rig heat;
    heat.s.temperature = 51;
    heat.tick();
    assert(heat.c.Warm());
    heat.s.temperature = 60;
    assert(heat.tick().action == Action::Stop);
    assert(!heat.c.Start(heat.s, 100, 10, true, heat.s.sampled_ms, 2000));
    Rig amp;
    amp.s.current = -501;
    assert(amp.tick().action == Action::Stop);
    Rig flag;
    flag.s.fault = true;
    flag.s.state = 4;
    assert(flag.tick().action == Action::Stop);
    Rig aging;
    aging.s.valid = false;
    assert(aging.tick().action == Action::Stop && aging.c.State() == Phase::Recovering);
    aging.s.valid = true;
    assert(aging.tick().action == Action::None);
    aging.c.Lost(300, "RX_NO_BYTES");
    assert(aging.c.State() == Phase::Fault);
    Rig late;
    late.s.sampled_ms += 51;
    assert(late.c.Step(late.s, late.s.sampled_ms).action == Action::Stop);
    Rig freeze;
    freeze.contact();
    const int frozen = freeze.cmd;
    freeze.s.pwm = 201;
    freeze.s.current = 130;
    for (int i = 0; i < 8 && freeze.c.Monitoring(); ++i)
    {
        freeze.tick(false);
        assert(freeze.cmd == frozen);
    }
    assert(freeze.c.Reason() == "CANNOT_HOLD_SAFELY");
    auto unload = bench();
    unload.unload_step_deg = .2;
    unload.max_unload_deg = .4;
    Rig holding(unload);
    holding.contact();
    const int anchor = holding.cmd;
    holding.s.current = 180;
    for (int i = 0; i < 15 && holding.c.Monitoring(); ++i)
    {
        holding.tick(false);
        assert(holding.cmd <= anchor);
    }
    assert(holding.cmd == anchor - 4 && holding.c.State() == Phase::Fault);
    Rig slip;
    slip.contact();
    slip.s.angle -= 15;
    assert(slip.tick(false).action == Action::Stop);
    Rig duration;
    duration.contact();
    duration.s.pwm = 0;
    duration.s.current = 0;
    while (duration.c.Monitoring())
    {
        duration.tick(false);
    }
    assert(duration.c.Reason() == "HOLD_DURATION");
    Rig cancel;
    cancel.c.Cancel(false);
    assert(cancel.c.Reason().find("STOP_UNCONFIRMED") != std::string::npos);
    std::puts("轨迹、反向机构、硬保护、保持及故障锁存模拟通过");
}
