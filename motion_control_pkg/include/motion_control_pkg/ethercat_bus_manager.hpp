#ifndef ETHERCAT_BUS_MANAGER_HPP
#define ETHERCAT_BUS_MANAGER_HPP

#include <atomic>
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

    bool getMotorActualSnapshot(std::vector<DriverSDK::motorActualStruct> &motor_actual_data) const;
    bool updateMotorTargets(const std::vector<uint16_t> &motor_ids,
                            const std::vector<DriverSDK::motorTargetStruct> &motor_target_data);
    // bool batchSetSoftstop(bool stop);

private:
    explicit EthercatBusManager(char const *xmlFile);

    void run();
    void initializeTargetBuffer();

    DriverSDK::DriverSDK& controller_;
    int motor_number_;

    mutable std::mutex actual_mutex_;
    mutable std::mutex target_mutex_;
    std::vector<DriverSDK::motorActualStruct> motor_actual_snapshot_;
    std::vector<DriverSDK::motorTargetStruct> motor_target_buffer_;
    bool clear_error_command_sent_{false};

    std::shared_ptr<std::thread> worker_;
    std::atomic<bool> stop_{false};

    static std::mutex instance_mutex_;
    static std::weak_ptr<EthercatBusManager> instance_;
};

}  // namespace miracle

#endif  // ETHERCAT_BUS_MANAGER_HPP
