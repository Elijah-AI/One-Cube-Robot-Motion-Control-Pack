#include "ethercat_bus_manager.hpp"

#include <algorithm>

#include "rclcpp/rclcpp.hpp"

namespace miracle {

namespace {

constexpr int kExpectedMotorCount = 32;
constexpr unsigned short kStatusSwitchOnDisabled = 0x0040U;
constexpr unsigned short kStatusOperationEnabled = 0x0037U;

std::vector<int> buildActiveMotors()
{
    std::vector<int> active_motors;
    active_motors.reserve(static_cast<std::size_t>(kExpectedMotorCount));
    for (int i = 0; i < kExpectedMotorCount; ++i) {
        active_motors.push_back(i);
    }
    return active_motors;
}

}  // namespace

std::mutex EthercatBusManager::instance_mutex_;
std::weak_ptr<EthercatBusManager> EthercatBusManager::instance_;

std::shared_ptr<EthercatBusManager> EthercatBusManager::getOrCreate(char const *xmlFile)
{
    std::lock_guard<std::mutex> lock(instance_mutex_);
    auto instance = instance_.lock();
    if (instance) {
        return instance;
    }

    instance = std::shared_ptr<EthercatBusManager>(new EthercatBusManager(xmlFile));
    instance_ = instance;
    return instance;
}

EthercatBusManager::EthercatBusManager(char const *xmlFile)
    : controller_(DriverSDK::DriverSDK::instance())
{
    (void)xmlFile;

    motor_number_ = kExpectedMotorCount;
    active_motors_ = buildActiveMotors();

    motor_actual_snapshot_.resize(static_cast<std::size_t>(motor_number_));
    motor_target_buffer_.resize(static_cast<std::size_t>(motor_number_));
    motor_enable_ready_.assign(static_cast<std::size_t>(motor_number_), false);

    for (int i = 0; i < motor_number_; ++i) {
        motor_actual_snapshot_[i].pos = 0.0f;
        motor_actual_snapshot_[i].vel = 0.0f;
        motor_actual_snapshot_[i].tor = 0.0f;
        motor_actual_snapshot_[i].statusWord = kStatusSwitchOnDisabled;
        motor_actual_snapshot_[i].errorCode = 0U;

        motor_target_buffer_[i].pos = 0.0f;
        motor_target_buffer_[i].vel = 0.0f;
        motor_target_buffer_[i].tor = 0.0f;
        motor_target_buffer_[i].kp = 0.0f;
        motor_target_buffer_[i].kd = 0.0f;
        motor_target_buffer_[i].enabled = 0;
    }

    ready_ = true;
    RCLCPP_INFO(rclcpp::get_logger("ethercat_bus_manager"),
                "Using fake EtherCAT bus manager on pseudo driver platform");
}

EthercatBusManager::~EthercatBusManager()
{
    ready_ = false;
    stop_ = true;
    disableAll();
}

void EthercatBusManager::run() {}

bool EthercatBusManager::waitForCommunication()
{
    return true;
}

bool EthercatBusManager::initializeTargetBuffer()
{
    return true;
}

bool EthercatBusManager::isReady() const noexcept
{
    return ready_.load();
}

bool EthercatBusManager::getMotorActualSnapshot(
    std::vector<DriverSDK::motorActualStruct> &motor_actual_data) const
{
    std::lock_guard<std::mutex> lock(actual_mutex_);
    if (motor_actual_snapshot_.empty()) {
        return false;
    }
    motor_actual_data = motor_actual_snapshot_;
    return true;
}

bool EthercatBusManager::enableMotors(const std::vector<uint16_t> &motor_ids, double timeout_seconds)
{
    if (!ready_ || motor_ids.empty() || timeout_seconds <= 0.0) {
        return false;
    }

    std::lock_guard<std::mutex> enable_lock(enable_mutex_);
    std::lock_guard<std::mutex> target_lock(target_mutex_);
    std::lock_guard<std::mutex> actual_lock(actual_mutex_);

    for (uint16_t motor_id : motor_ids) {
        if (motor_id >= motor_target_buffer_.size() ||
            std::find(active_motors_.begin(), active_motors_.end(), motor_id) == active_motors_.end()) {
            RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                         "Motor index %u is not active in fake bus manager",
                         static_cast<unsigned int>(motor_id));
            return false;
        }
    }

    for (uint16_t motor_id : motor_ids) {
        motor_target_buffer_[motor_id].enabled = 1;
        motor_enable_ready_[motor_id] = true;
        motor_actual_snapshot_[motor_id].statusWord = kStatusOperationEnabled;
        motor_actual_snapshot_[motor_id].errorCode = 0U;
        motor_actual_snapshot_[motor_id].vel = 0.0f;
        motor_actual_snapshot_[motor_id].tor = 0.0f;
        motor_actual_snapshot_[motor_id].pos = motor_target_buffer_[motor_id].pos;
    }

    return true;
}

bool EthercatBusManager::updateMotorTargets(
    const std::vector<uint16_t> &motor_ids,
    const std::vector<DriverSDK::motorTargetStruct> &motor_target_data)
{
    if (!ready_ || motor_ids.size() != motor_target_data.size()) {
        return false;
    }

    std::lock_guard<std::mutex> target_lock(target_mutex_);
    std::lock_guard<std::mutex> actual_lock(actual_mutex_);
    if (motor_target_buffer_.empty()) {
        return false;
    }

    for (size_t i = 0; i < motor_ids.size(); ++i) {
        const uint16_t motor_id = motor_ids[i];
        if (motor_id >= motor_target_buffer_.size() ||
            std::find(active_motors_.begin(), active_motors_.end(), motor_id) == active_motors_.end() ||
            (motor_target_data[i].enabled != 0 && motor_target_data[i].enabled != 1) ||
            (motor_target_data[i].enabled == 1 && !motor_enable_ready_[motor_id])) {
            return false;
        }
    }

    for (size_t i = 0; i < motor_ids.size(); ++i) {
        const uint16_t motor_id = motor_ids[i];
        motor_target_buffer_[motor_id] = motor_target_data[i];
        motor_actual_snapshot_[motor_id].pos = motor_target_data[i].pos;
        motor_actual_snapshot_[motor_id].vel = motor_target_data[i].vel;
        motor_actual_snapshot_[motor_id].tor = motor_target_data[i].tor;
        motor_actual_snapshot_[motor_id].errorCode = 0U;

        if (motor_target_data[i].enabled == 0) {
            motor_enable_ready_[motor_id] = false;
            motor_actual_snapshot_[motor_id].statusWord = kStatusSwitchOnDisabled;
        } else {
            motor_actual_snapshot_[motor_id].statusWord = kStatusOperationEnabled;
        }
    }

    return true;
}

void EthercatBusManager::disableAll()
{
    std::lock_guard<std::mutex> target_lock(target_mutex_);
    std::lock_guard<std::mutex> actual_lock(actual_mutex_);

    for (int motor_id : active_motors_) {
        motor_target_buffer_[motor_id].enabled = 0;
        motor_target_buffer_[motor_id].vel = 0.0f;
        motor_target_buffer_[motor_id].tor = 0.0f;
        motor_enable_ready_[motor_id] = false;

        motor_actual_snapshot_[motor_id].vel = 0.0f;
        motor_actual_snapshot_[motor_id].tor = 0.0f;
        motor_actual_snapshot_[motor_id].statusWord = kStatusSwitchOnDisabled;
        motor_actual_snapshot_[motor_id].errorCode = 0U;
    }
}

}  // namespace miracle
