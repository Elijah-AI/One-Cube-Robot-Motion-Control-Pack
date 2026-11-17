#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <thread>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "Ds485_Marvin_Transport.h"
#include "fake_ds485_servo.h"
#include "MarvinSDK.h"
#include "motion_control_pkg/execution/action_terminal_guard.hpp"

// This harness runs without an installed robot motion-safety monitor.
bool motion_control_pkg::execution::MotionSafetyScope::beforeTerminal(bool success, std::string&) {
    return success;
}

enum Injection { none, right_send_failure, right_ack_loss, status_failure, protection, client_cancel, hold_rejected, mid_move_timeout, orderly_shutdown, cancel_before_init };
static Fake_Ds485_Bus bus;
static Injection injected;
static bool canceling;

static bool send(int side, unsigned char *data, long size, long channel)
{
    const bool move = data[4] == DS485_CMD_WRITE && data[5] == DS485_REG_MOVE_SPEED;
    if (move && side == DS485_SIDE_RIGHT && (injected == right_send_failure || injected == hold_rejected)) return false;
    if (injected == status_failure && bus.servo[side].moving &&
        (data[5] == DS485_REG_ANGLE || data[5] == DS485_REG_COMBINED)) return false;
    const bool accepted = bus.Send(side, channel, data, size);
    if (move && side == DS485_SIDE_RIGHT && injected == right_ack_loss) bus.Clear(side);
    return accepted;
}

bool OnClearChDataA() { return bus.Clear(DS485_SIDE_LEFT); }
bool OnClearChDataB() { return bus.Clear(DS485_SIDE_RIGHT); }
bool OnSetChDataA(unsigned char *data, long size, long ch) { return send(DS485_SIDE_LEFT, data, size, ch); }
bool OnSetChDataB(unsigned char *data, long size, long ch) { return send(DS485_SIDE_RIGHT, data, size, ch); }
long OnGetChDataA(unsigned char *data, long *ch) { return bus.Receive(DS485_SIDE_LEFT, *ch, data, 256); }
long OnGetChDataB(unsigned char *data, long *ch) { return bus.Receive(DS485_SIDE_RIGHT, *ch, data, 256); }

namespace rclcpp {
struct Logger {};
Logger get_logger(const char *) { return {}; }
void sleep_for(std::chrono::milliseconds ms) {
    bus.Sleep_ms(ms.count());
    if (injected == protection) {
        bus.servo[0].fault_bits = 0x08;
        bus.servo[0].moving = false;
    }
    if (injected == mid_move_timeout) std::this_thread::sleep_for(std::chrono::milliseconds(1050));
    if (injected == client_cancel) canceling = true;
}
}
#define RCLCPP_ERROR(logger, ...) ((void)(logger))
#define RCLCPP_INFO(logger, ...) ((void)(logger))
#define RCLCPP_WARN(logger, ...) ((void)(logger))
namespace motion_control_pkg::execution { bool shutdownRequested() { return injected == orderly_shutdown; } }

enum { GRIPPER_LEFT, GRIPPER_RIGHT, GRIPPER_BOTH };
enum { ACTION_FAIL, ACTION_SUCCESS };
struct ControlGripper {
    struct Result { int success{}; std::string error_message; std::vector<uint8_t> final_position, final_torque; };
    struct Feedback {
        struct { int stamp; } header;
        std::vector<uint8_t> current_position, current_torque, current_speed;
        float progress_rate{};
    };
    struct Goal {
        int gripper_id{GRIPPER_BOTH}, timeout_ms{10000};
        std::vector<uint8_t> com_port{2, 3}, position{255, 255}, max_torque{10, 10}, speed{100, 100};
    };
};
struct GoalHandleControlGripper {
    std::shared_ptr<ControlGripper::Goal> goal = std::make_shared<ControlGripper::Goal>();
    std::shared_ptr<ControlGripper::Result> result;
    char outcome{'?'};
    auto get_goal() { return goal; }
    bool is_canceling() { return canceling; }
    void publish_feedback(const std::shared_ptr<ControlGripper::Feedback>& feedback) {
        assert(feedback->progress_rate >= 0.0 && feedback->progress_rate <= 100.0);
        assert(feedback->current_position.size() == 2);
    }
    void set_terminal(char state, const std::shared_ptr<ControlGripper::Result>& value) {
        assert(outcome == '?');
        outcome = state;
        result = value;
    }
    void abort(const std::shared_ptr<ControlGripper::Result>& value) { set_terminal('a', value); }
    void canceled(const std::shared_ptr<ControlGripper::Result>& value) { set_terminal('c', value); }
    void succeed(const std::shared_ptr<ControlGripper::Result>& value) { set_terminal('s', value); }
};
using ControlGripperTerminalGuard = motion_control_pkg::execution::ActionTerminalGuard<
    GoalHandleControlGripper, ControlGripper::Result>;
struct DualArmModule {
    std::atomic<bool> is_executing_left_tool_{}, is_executing_right_tool_{};
    Ds485_config ds485_config_[2] = {
        {2, 1, 0, 900, 90, 30, 10, 3, 10, 0, 20},
        {3, 1, 0, 900, 90, 30, 10, 3, 10, 0, 20}
    };
    struct Node { int now() { return 0; } };
    Node *lockNode(const char *) { return nullptr; }
    float_t get_progress_1000(int, int, int);
    void execution_loop_control_ds485_gripper(std::shared_ptr<GoalHandleControlGripper>);
};
#include "ds485_extracted_action.inc"


static void run(const char* name, Injection which, char outcome, bool hold_left, bool hold_right,
                int timeout = 10000, int side = GRIPPER_BOTH)
{
    bus.Reset_Servo(0);
    bus.Reset_Servo(1);
    bus.servo[0].torque_switch = DS485_SWITCH_HOLD;
    bus.servo[1].torque_switch = DS485_SWITCH_HOLD;
    bus.servo[0].reject_switch_write = which == hold_rejected;
    injected = which;
    canceling = which == cancel_before_init;
    DualArmModule module;
    auto goal = std::make_shared<GoalHandleControlGripper>();
    goal->goal->timeout_ms = timeout;
    goal->goal->gripper_id = side;
    module.is_executing_left_tool_ = side != GRIPPER_RIGHT;
    module.is_executing_right_tool_ = side != GRIPPER_LEFT;
    module.execution_loop_control_ds485_gripper(goal);

    assert(goal->outcome == outcome);
    assert(goal->result);
    assert(goal->result->success == (outcome == 's' ? ACTION_SUCCESS : ACTION_FAIL));
    assert(!module.is_executing_left_tool_ && !module.is_executing_right_tool_);
    assert((bus.servo[0].last_write_addr == DS485_REG_SWITCH) == hold_left);
    assert((bus.servo[1].last_write_addr == DS485_REG_SWITCH) == hold_right);
    if (hold_left) assert(bus.servo[0].moving == (which == hold_rejected));
    if (hold_right) assert(!bus.servo[1].moving);

    if (outcome == 's') {
        if (side != GRIPPER_RIGHT) {
            assert(bus.servo[0].last_channel == 2);
            assert(goal->result->final_position[0] == 255);
        } else {
            assert(bus.servo[0].frames_ok == 0);
        }
        if (side != GRIPPER_LEFT) {
            assert(bus.servo[1].last_channel == 3);
            assert(goal->result->final_position[1] == 255);
        } else {
            assert(bus.servo[1].frames_ok == 0);
        }
    }
    if (timeout == 0) {
        assert(bus.servo[0].frames_ok == 0);
        assert(bus.servo[1].frames_ok == 0);
    }
    if (timeout < 10000) assert(goal->result->error_message == "time_out");
    if (which == hold_rejected) {
        assert(goal->result->error_message.find("right ds485 move command failed") != std::string::npos);
        assert(goal->result->error_message.find("left hold failed") != std::string::npos);
    }
    printf("PASS %s\n", name);
}

int main()
{
    run("dual success and A/B mapping", none, 's', false, false);
    run("left-only success and no right traffic", none, 's', false, false, 10000, GRIPPER_LEFT);
    run("right-only success and no left traffic", none, 's', false, false, 10000, GRIPPER_RIGHT);
    run("right command rejected before send", right_send_failure, 'a', true, false);
    run("right command sent but ACK lost", right_ack_loss, 'a', true, true);
    run("status read fails mid-motion", status_failure, 'a', true, true);
    run("left protection fault stops only the other side", protection, 'a', false, true);
    run("client cancellation mid-motion", client_cancel, 'c', true, true);
    run("failed hold is visible in returned error", hold_rejected, 'a', true, false);
    run("expired deadline before bus access", none, 'a', false, false, 0);
    run("expired deadline after successful send", mid_move_timeout, 'a', true, true, 1000);
    run("shutdown refuses bus access", orderly_shutdown, 'a', false, false);
    run("cancel before init refuses bus access", cancel_before_init, 'c', false, false);
    puts("DS485 Action: 13 scenarios passed");
}
