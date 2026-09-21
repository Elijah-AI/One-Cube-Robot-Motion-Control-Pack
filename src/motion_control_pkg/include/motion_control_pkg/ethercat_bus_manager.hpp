#ifndef ETHERCAT_BUS_MANAGER_HPP
#define ETHERCAT_BUS_MANAGER_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "loong_driver_sdk.h"


namespace miracle {

#define TRAJ_FREQ      250.0
#define TRAJ_CYCLE_US  4000

class EthercatBusManager {
public:
    static std::shared_ptr<EthercatBusManager> getOrCreate(char const *xmlFile);

    EthercatBusManager(const EthercatBusManager &) = delete;
    EthercatBusManager &operator=(const EthercatBusManager &) = delete;

    ~EthercatBusManager();

    bool isReady() const noexcept;
    bool getMotorActualSnapshot(std::vector<DriverSDK::motorActualStruct> &motor_actual_data) const;
    // A successful SDK acquisition only; callers must retain its age and sequence.
    bool getTimedMotorActualSnapshot(std::vector<DriverSDK::motorActualStruct>& data,
        std::chrono::steady_clock::time_point& acquired, uint64_t& sequence) const;
    bool enableMotors(const std::vector<uint16_t> &motor_ids, double timeout_seconds = 5.0);
    bool updateMotorTargets(const std::vector<uint16_t> &motor_ids,
                            const std::vector<DriverSDK::motorTargetStruct> &motor_target_data);
    // bool batchSetSoftstop(bool stop);

private:
    explicit EthercatBusManager(char const *xmlFile);

    void run();
    bool waitForCommunication();
    bool initializeTargetBuffer();
    void disableAll();

    DriverSDK::DriverSDK& controller_;
    int motor_number_{0};
    std::vector<int> active_motors_;

    mutable std::mutex actual_mutex_;
    mutable std::mutex target_mutex_;
    std::mutex enable_mutex_;
    std::vector<DriverSDK::motorActualStruct> motor_actual_snapshot_;
    std::chrono::steady_clock::time_point actual_acquired_{};
    uint64_t actual_sequence_{0};
    bool actual_read_valid_{false};
    std::vector<DriverSDK::motorTargetStruct> motor_target_buffer_;
    std::vector<bool> motor_enable_ready_;

    std::shared_ptr<std::thread> worker_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> ready_{false};

    static std::mutex instance_mutex_;
    static std::weak_ptr<EthercatBusManager> instance_;
};

}  // namespace miracle

#endif  // ETHERCAT_BUS_MANAGER_HPP
