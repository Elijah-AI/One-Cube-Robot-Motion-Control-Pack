#include "loong_driver_sdk.h"

#include <algorithm>
#include <cstddef>

namespace DriverSDK {

DriverSDK &DriverSDK::instance()
{
    static DriverSDK sdk;
    return sdk;
}

bool DriverSDK::init(char const *xml_file)
{
    std::lock_guard<std::mutex> lock(mutex_);
    config_path_ = xml_file != nullptr ? xml_file : "";
    ensureMotorStorageLocked(static_cast<std::size_t>(motor_count_));
    return true;
}

bool DriverSDK::start(const std::string &eni0_file,
                      const std::string &eni1_file,
                      int priority,
                      int interval,
                      std::int64_t cycle_time_0,
                      std::int64_t cycle_time_1,
                      std::int64_t shift_time)
{
    (void)priority;
    (void)interval;
    (void)cycle_time_0;
    (void)cycle_time_1;
    (void)shift_time;

    std::lock_guard<std::mutex> lock(mutex_);
    config_path_ = eni0_file + ";" + eni1_file;
    ensureMotorStorageLocked(static_cast<std::size_t>(motor_count_));
    return true;
}

int DriverSDK::getTotalMotorNr() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return motor_count_;
}

bool DriverSDK::setMode(const std::vector<char> &mode)
{
    std::lock_guard<std::mutex> lock(mutex_);
    mode_ = mode;
    if (!mode.empty()) {
        motor_count_ = static_cast<int>(mode.size());
        ensureMotorStorageLocked(mode.size());
    }
    return true;
}

bool DriverSDK::getMotorActual(std::vector<motorActualStruct> &motor_actual_data)
{
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotorStorageLocked(motor_actual_data.empty()
                                 ? static_cast<std::size_t>(motor_count_)
                                 : motor_actual_data.size());
    motor_actual_data = actual_;
    return true;
}

bool DriverSDK::setMotorTarget(const std::vector<motorTargetStruct> &motor_target_data)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!motor_target_data.empty()) {
        motor_count_ = static_cast<int>(motor_target_data.size());
        ensureMotorStorageLocked(motor_target_data.size());
        target_ = motor_target_data;
    }
    return true;
}

bool DriverSDK::advance()
{
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotorStorageLocked(static_cast<std::size_t>(motor_count_));
    for (std::size_t i = 0; i < actual_.size() && i < target_.size(); ++i) {
        if (target_[i].enabled != 0U) {
            actual_[i].pos = target_[i].pos;
            actual_[i].vel = target_[i].vel;
            actual_[i].tor = target_[i].tor;
            actual_[i].statusWord = 4;
        } else {
            actual_[i].vel = 0.0f;
            actual_[i].tor = 0.0f;
            actual_[i].statusWord = 0;
        }
        actual_[i].errorCode = 0;
    }
    return true;
}

void DriverSDK::ensureMotorStorageLocked(std::size_t motor_count)
{
    const std::size_t normalized_count =
        std::max<std::size_t>(motor_count, static_cast<std::size_t>(1));

    actual_.resize(normalized_count);
    target_.resize(normalized_count);
    if (mode_.size() < normalized_count) {
        mode_.resize(normalized_count, 0);
    }
}

}  // namespace DriverSDK
