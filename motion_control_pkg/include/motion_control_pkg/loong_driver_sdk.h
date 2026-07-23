#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace DriverSDK {

struct motorActualStruct {
    float pos = 0.0f;
    float vel = 0.0f;
    float tor = 0.0f;
    std::uint64_t errorCode = 0;
    std::uint16_t statusWord = 0;
};

struct motorTargetStruct {
    float pos = 0.0f;
    float vel = 0.0f;
    float tor = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    std::uint8_t enabled = 0;
};

class DriverSDK {
public:
    DriverSDK() = default;

    static DriverSDK &instance();

    bool init(char const *xml_file);
    bool start(const std::string &eni0_file,
               const std::string &eni1_file,
               int priority,
               int interval,
               std::int64_t cycle_time_0,
               std::int64_t cycle_time_1,
               std::int64_t shift_time);

    int getTotalMotorNr() const;
    bool setMode(const std::vector<char> &mode);
    bool getMotorActual(std::vector<motorActualStruct> &motor_actual_data);
    bool setMotorTarget(const std::vector<motorTargetStruct> &motor_target_data);
    bool advance();

private:
    void ensureMotorStorageLocked(std::size_t motor_count);

    mutable std::mutex mutex_;
    int motor_count_ = 32;
    std::vector<char> mode_;
    std::vector<motorActualStruct> actual_;
    std::vector<motorTargetStruct> target_;
    std::string config_path_;
};

}  // namespace DriverSDK
