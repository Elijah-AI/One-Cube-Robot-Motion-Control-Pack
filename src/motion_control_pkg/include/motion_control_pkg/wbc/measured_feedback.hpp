#pragma once

#include "motion_control_pkg/sdk/dual_arm_device_interface.hpp"
#include "motion_control_pkg/sdk/ethercat_feedback_validity.hpp"
#include "motion_control_pkg/loong_driver_sdk.h"
#include <sensor_msgs/msg/joint_state.hpp>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include <utility>

namespace motion_control_pkg::wbc {

// Builds only measured arm/body/head states. It never invents steering/locked
// coordinates. One conservative timestamp covers both hardware sources.
class MeasuredFeedbackBuilder {
public:
    explicit MeasuredFeedbackBuilder(std::array<uint16_t, 6> motor_ids)
        : motor_ids_(motor_ids) {}

    // Snapshot readers may reuse a still-fresh physical frame. They must not
    // consume the publisher's advancement history or refresh acquisition time.
    bool buildSnapshot(const sdk::DualArmDeviceState& arms,
               const std::vector<DriverSDK::motorActualStruct>& motors,
               uint64_t motor_sequence, int64_t acquired_ns, int64_t now_ns,
               sensor_msgs::msg::JointState& output, std::string& error) const {
        MeasuredFeedbackBuilder snapshot(motor_ids_);
        return snapshot.build(arms, motors, motor_sequence, acquired_ns, now_ns, output, error);
    }

    bool build(const sdk::DualArmDeviceState& arms,
               const std::vector<DriverSDK::motorActualStruct>& motors,
               uint64_t motor_sequence, int64_t acquired_ns, int64_t now_ns,
               sensor_msgs::msg::JointState& output, std::string& error) {
        output = sensor_msgs::msg::JointState{};
        const auto fail = [&](const std::string& why) { error = why; return false; };
        if (acquired_ns <= 0 || acquired_ns > now_ns || now_ns-acquired_ns > 100000000LL)
            return fail("hardware acquisition clock/age invalid");
        if (acquired_ns <= last_stamp_ || motor_sequence == 0 || motor_sequence <= last_motor_sequence_)
            return fail("hardware sample did not advance");
        for (std::size_t a=0; a<2; ++a) {
            const auto& arm=arms.arms[a];
            if (!sequenceAdvanced(arm.feedback_sequence, last_arm_sequence_[a]))
                return fail("arm "+std::to_string(a)+" frame did not advance: "+
                    std::to_string(last_arm_sequence_[a])+" -> "+std::to_string(arm.feedback_sequence));
            if (!sdk::armFeedbackOperational(arm))
                return fail("arm controller reports error or transition state");
            for (std::size_t j=0; j<7; ++j)
                if (!std::isfinite(arm.joint_position[j]) || !std::isfinite(arm.joint_velocity[j]) ||
                    !std::isfinite(arm.joint_effort[j])) return fail("non-finite arm feedback");
        }
        for (auto id:motor_ids_) {
            if (id>=motors.size()) return fail("missing body/head motor");
            const auto& m=motors[id];
            if (!sdk::ethercatFeedbackValid(m))
                return fail("motor "+std::to_string(id)+" invalid status="+
                    std::to_string(m.statusWord)+" error="+std::to_string(m.errorCode));
            if (!std::isfinite(m.pos) || !std::isfinite(m.vel) || !std::isfinite(m.tor))
                return fail("non-finite body/head feedback");
        }
        sensor_msgs::msg::JointState message;
        message.header.stamp.sec=static_cast<int32_t>(acquired_ns/1000000000LL);
        message.header.stamp.nanosec=static_cast<uint32_t>(acquired_ns%1000000000LL);
        message.header.frame_id="base_link";
        constexpr double deg_to_rad=0.017453292519943295;
        for (std::size_t a=0; a<2; ++a) {
            for (std::size_t j=0; j<7; ++j) {
                message.name.push_back("Joint"+std::to_string(j+1)+(a==0 ? "_L" : "_R"));
                message.position.push_back(arms.arms[a].joint_position[j]*deg_to_rad);
                message.velocity.push_back(arms.arms[a].joint_velocity[j]*deg_to_rad);
                message.effort.push_back(arms.arms[a].joint_effort[j]);
            }
        }
        const std::array<const char*,6> names{{"leg_joint1","leg_joint2","leg_joint3",
            "body_joint","head_yaw_joint","head_pitch_joint"}};
        for (std::size_t j=0; j<6; ++j) {
            const auto& m=motors[motor_ids_[j]];
            message.name.push_back(names[j]);
            message.position.push_back(m.pos); // DriverSDK already applies polarity/zero/gear scaling.
            message.velocity.push_back(m.vel);
            message.effort.push_back(m.tor);
        }
        last_stamp_=acquired_ns;
        last_motor_sequence_=motor_sequence;
        for (std::size_t a=0; a<2; ++a) last_arm_sequence_[a]=arms.arms[a].feedback_sequence;
        output=std::move(message); error.clear(); return true;
    }

private:
    static bool sequenceAdvanced(int value,int previous) {
        if (value<0 || value>1000000) return false;
        if (previous<0 || value>previous) return true;
        // Vendor frame counter wraps near one million. A general reset/reversal
        // is not treated as a fresh sample; reconnect/restart must requalify it.
        return previous>900000 && value<100000;
    }
    std::array<uint16_t,6> motor_ids_;
    std::array<int,2> last_arm_sequence_{{-1,-1}};
    uint64_t last_motor_sequence_{0};
    int64_t last_stamp_{0};
};
}  // namespace motion_control_pkg::wbc
