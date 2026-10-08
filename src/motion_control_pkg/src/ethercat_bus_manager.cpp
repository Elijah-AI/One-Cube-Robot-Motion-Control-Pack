#include "ethercat_bus_manager.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <time.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
// #include "json_server.hpp"

namespace miracle {

namespace {

constexpr int kExpectedMotorCount = 32;
constexpr int kCommunicationStableCycles = 25;
constexpr int kCommunicationTimeoutCycles = 25 * 1000000 / TRAJ_CYCLE_US;
constexpr int kBrakeReleaseHoldCycles = 600000 / TRAJ_CYCLE_US;
constexpr int kShutdownDisableCycles = 100;
constexpr int kStartupFaultResetCycles = 200000 / TRAJ_CYCLE_US;
constexpr int kStartupFaultResetTimeoutCycles = 2000000 / TRAJ_CYCLE_US;
// Same EtherCAT aliases as LEG_JOINT1/2/3 and BODY_JOINT in main_node.hpp.
constexpr std::array<int, 4> kLowerBodyMotorIds{{30 - 1, 29 - 1, 28 - 1, 27 - 1}};

bool communicationValid(unsigned short status_word) {
    return status_word != 0U && status_word != 0xffffU;
}

bool operationEnabled(unsigned short status_word) {
    return (status_word & 0x007fU) == 0x0037U;
}

bool faultPresent(const DriverSDK::motorActualStruct &actual) {
    return actual.errorCode != 0U || (actual.statusWord & 0x0008U) != 0U;
}

// Startup only: no worker is running, so its fault-to-disable guard cannot
// overwrite the SDK's enabled=-1 reset request. Never enable a motor here.
bool clearStartupLowerBodyFaults(
    DriverSDK::DriverSDK &controller, const std::vector<int> &active_motors,
    const std::vector<DriverSDK::motorTargetStruct> &initial_targets) {
    std::vector<int> motor_ids;
    for (int id : kLowerBodyMotorIds) {
        if (std::find(active_motors.begin(), active_motors.end(), id) != active_motors.end()) {
            motor_ids.push_back(id);
        }
    }
    if (motor_ids.empty()) {
        return true;
    }

    auto motor_target_data = initial_targets;
    for (auto &target : motor_target_data) {
        target.enabled = 0;
    }
    std::vector<DriverSDK::motorActualStruct> actual_data(initial_targets.size());
    const auto read_feedback = [&]() {
        if (controller.getMotorActual(actual_data) != 0 ||
            actual_data.size() != motor_target_data.size()) {
            return false;
        }
        for (int id : motor_ids) {
            if (static_cast<size_t>(id) >= actual_data.size() ||
                !communicationValid(actual_data[id].statusWord) ||
                !std::isfinite(actual_data[id].pos)) {
                return false;
            }
            motor_target_data[id].pos = actual_data[id].pos;
        }
        return true;
    };
    if (!read_feedback()) {
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                     "Lower-body startup fault reset requires valid motor feedback");
        return false;
    }
    for (int id : motor_ids) {
        motor_target_data[id].enabled = -1;
        RCLCPP_INFO(rclcpp::get_logger("ethercat_bus_manager"),
                    "Startup fault reset alias %u: enabled=-1, status=0x%04x error=0x%04x",
                    static_cast<unsigned int>(id + 1),
                    static_cast<unsigned int>(actual_data[id].statusWord),
                    static_cast<unsigned int>(actual_data[id].errorCode));
    }
    bool reset_sent = true;
    for (int cycle = 0; cycle < kStartupFaultResetCycles; ++cycle) {
        if (controller.setMotorTarget(motor_target_data) != 0) {
            reset_sent = false;
            break;
        }
        usleep(TRAJ_CYCLE_US);
        if (!read_feedback()) {
            reset_sent = false;
            break;
        }
    }
    // Release the reset command even on error. Normal enableMotors() remains
    // responsible for the later enable/brake-release handshake.
    for (int id : motor_ids) {
        motor_target_data[id].enabled = 0;
    }
    const bool reset_released = controller.setMotorTarget(motor_target_data) == 0;
    if (!reset_sent || !reset_released) {
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                     "Lower-body startup fault reset failed to exchange motor data");
        return false;
    }

    int stable_cycles = 0;
    for (int cycle = 0; cycle < kStartupFaultResetTimeoutCycles; ++cycle) {
        usleep(TRAJ_CYCLE_US);
        const bool feedback_valid = read_feedback();
        bool faults_cleared = feedback_valid;
        if (feedback_valid) {
            for (int id : motor_ids) {
                faults_cleared = faults_cleared && !faultPresent(actual_data[id]);
            }
        }
        if (controller.setMotorTarget(motor_target_data) != 0) {
            break;
        }
        stable_cycles = faults_cleared ? stable_cycles + 1 : 0;
        if (stable_cycles >= kCommunicationStableCycles) {
            RCLCPP_INFO(rclcpp::get_logger("ethercat_bus_manager"),
                        "Lower-body startup fault reset confirmed; enabled=0 until normal enable");
            return true;
        }
    }
    for (int id : motor_ids) {
        if (static_cast<size_t>(id) < actual_data.size()) {
            RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                         "Startup fault reset not confirmed for alias %u: status=0x%04x error=0x%04x",
                         static_cast<unsigned int>(id + 1),
                         static_cast<unsigned int>(actual_data[id].statusWord),
                         static_cast<unsigned int>(actual_data[id].errorCode));
        }
    }
    return false;
}

std::vector<unsigned short> defaultMaxCurrent() {
    std::vector<unsigned short> values(kExpectedMotorCount, 1000);
    for (int alias : {18, 19, 25, 26, 31, 32}) {
        values[alias - 1] = 600;
    }
    return values;
}

}  // namespace

std::mutex EthercatBusManager::instance_mutex_;
std::weak_ptr<EthercatBusManager> EthercatBusManager::instance_;

std::shared_ptr<EthercatBusManager> EthercatBusManager::getOrCreate(char const *xmlFile) {
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
    : controller_(DriverSDK::DriverSDK::instance()) {
    controller_.setMaxCurr(defaultMaxCurrent());
    std::vector<char> mode(kExpectedMotorCount, static_cast<char>(8));
    if (controller_.setMode(mode) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"), "Failed to set motor modes");
        return;
    }

    controller_.init(xmlFile);
    motor_number_ = controller_.getTotalMotorNr();
    if (motor_number_ != kExpectedMotorCount) {
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                     "Expected %d logical motors, got %d", kExpectedMotorCount, motor_number_);
        return;
    }

    active_motors_ = controller_.getActiveMotors();
    if (active_motors_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"), "No active motor found");
        return;
    }
    if (!waitForCommunication() || !initializeTargetBuffer()) {
        return;
    }
    if (!clearStartupLowerBodyFaults(controller_, active_motors_, motor_target_buffer_) ||
        !initializeTargetBuffer()) {
        return;
    }

    ready_ = true;
    worker_ = std::make_shared<std::thread>(&EthercatBusManager::run, this);
}

EthercatBusManager::~EthercatBusManager() {
    ready_ = false;
    stop_ = true;
    if (worker_ && worker_->joinable()) {
        worker_->join();
    }
    disableAll();
}

bool EthercatBusManager::waitForCommunication() {
    std::vector<DriverSDK::motorActualStruct> motor_actual_data(motor_number_);
    int stable_cycles = 0;
    for (int cycle = 0; cycle < kCommunicationTimeoutCycles; ++cycle) {
        controller_.advance();
        usleep(TRAJ_CYCLE_US);
        if (controller_.getMotorActual(motor_actual_data) != 0) {
            stable_cycles = 0;
            continue;
        }

        bool valid = true;
        for (int motor_id : active_motors_) {
            if (motor_id < 0 || motor_id >= motor_number_ ||
                !communicationValid(motor_actual_data[motor_id].statusWord)) {
                valid = false;
                break;
            }
        }
        stable_cycles = valid ? stable_cycles + 1 : 0;
        if (stable_cycles >= kCommunicationStableCycles) {
            return true;
        }
    }

    RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                 "PDO communication did not become stable within %.1f seconds",
                 static_cast<double>(kCommunicationTimeoutCycles) * TRAJ_CYCLE_US / 1000000.0);
    return false;
}

bool EthercatBusManager::initializeTargetBuffer() {
    std::vector<DriverSDK::motorActualStruct> motor_actual_data(motor_number_);
    const auto acquired = std::chrono::steady_clock::now();
    if (controller_.getMotorActual(motor_actual_data) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                     "Failed to read initial motor state");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(actual_mutex_);
        motor_actual_snapshot_ = motor_actual_data;
        actual_acquired_ = acquired;
        ++actual_sequence_;
        actual_read_valid_ = true;
    }
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        motor_target_buffer_.resize(motor_number_);
        motor_enable_ready_.assign(motor_number_, false);
        for (size_t i = 0; i < motor_actual_data.size(); ++i) {
            motor_target_buffer_[i].pos = motor_actual_data[i].pos;
            motor_target_buffer_[i].vel = 0.0f;
            motor_target_buffer_[i].tor = 0.0f;
            motor_target_buffer_[i].kp = 0.0f;
            motor_target_buffer_[i].kd = 0.0f;
            motor_target_buffer_[i].enabled = 0;
        }
    }
    return true;
}

void EthercatBusManager::run() {

    std::vector<DriverSDK::motorActualStruct> actual_data(motor_number_);
    std::vector<DriverSDK::motorTargetStruct> target_data(motor_number_);
    timespec next_wakeup_time{};
    clock_gettime(CLOCK_MONOTONIC, &next_wakeup_time);

    while (!stop_) {
      
        const auto acquired = std::chrono::steady_clock::now();
        const int actual_result = controller_.getMotorActual(actual_data);
        {
            std::lock_guard<std::mutex> lock(actual_mutex_);
            actual_read_valid_ = actual_result == 0;
            if (actual_read_valid_) {
                motor_actual_snapshot_ = actual_data;
                actual_acquired_ = acquired;
                ++actual_sequence_;
            }
        }
        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            for (int motor_id : active_motors_) {
                if (actual_result != 0 || faultPresent(actual_data[motor_id]) ||
                    !communicationValid(actual_data[motor_id].statusWord)) {
                    motor_target_buffer_[motor_id].enabled = 0;
                    motor_enable_ready_[motor_id] = false;
                } else if (!operationEnabled(actual_data[motor_id].statusWord)) {
                    if (motor_enable_ready_[motor_id]) {
                        motor_target_buffer_[motor_id].enabled = 0;
                    }
                    motor_enable_ready_[motor_id] = false;
                }
            }
            target_data = motor_target_buffer_;
        }

        if (!target_data.empty()) {
            if (controller_.setMotorTarget(target_data) != 0) {
                RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                             "setMotorTarget failed");
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ethercat_bus_manager"), "No motor target set");
            controller_.advance();
        }

        next_wakeup_time.tv_nsec += TRAJ_CYCLE_US * 1000;
        while (next_wakeup_time.tv_nsec >= 1000000000L) {
            next_wakeup_time.tv_nsec -= 1000000000L;
            ++next_wakeup_time.tv_sec;
        }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup_time, nullptr);
    }
}

bool EthercatBusManager::isReady() const noexcept {
    return ready_.load();
}

bool EthercatBusManager::getMotorActualSnapshot(std::vector<DriverSDK::motorActualStruct> &motor_actual_data) const {
    std::lock_guard<std::mutex> lock(actual_mutex_);
    if (motor_actual_snapshot_.empty()) {
        return false;
    }
    motor_actual_data = motor_actual_snapshot_;
    return true;
}

bool EthercatBusManager::getTimedMotorActualSnapshot(
    std::vector<DriverSDK::motorActualStruct>& data,
    std::chrono::steady_clock::time_point& acquired, uint64_t& sequence) const {
    std::lock_guard<std::mutex> lock(actual_mutex_);
    if (!actual_read_valid_ || motor_actual_snapshot_.empty() || actual_sequence_ == 0)
        return false;
    data = motor_actual_snapshot_;
    acquired = actual_acquired_;
    sequence = actual_sequence_;
    return true;
}

bool EthercatBusManager::enableMotors(const std::vector<uint16_t> &motor_ids,
                                      double timeout_seconds) {
    if (!ready_ || motor_ids.empty() || timeout_seconds <= 0.0) {
        return false;
    }

    std::lock_guard<std::mutex> enable_lock(enable_mutex_);
    std::vector<uint16_t> pending_ids;
    std::vector<DriverSDK::motorActualStruct> actual_data;
    if (!getMotorActualSnapshot(actual_data)) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        for (uint16_t motor_id : motor_ids) {
            if (motor_id >= motor_target_buffer_.size() ||
                std::find(active_motors_.begin(), active_motors_.end(), motor_id) == active_motors_.end()) {
                RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                             "Motor index %u is not active", static_cast<unsigned int>(motor_id));
                return false;
            }
            if (!motor_enable_ready_[motor_id]) {
                pending_ids.push_back(motor_id);
            }
        }
    }
    if (pending_ids.empty()) {
        return true;
    }

    for (uint16_t motor_id : pending_ids) {
        const auto &actual = actual_data[motor_id];
        if (!communicationValid(actual.statusWord) || faultPresent(actual)) {
            RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                         "Refusing to enable alias %u: status=0x%04x error=0x%04x",
                         static_cast<unsigned int>(motor_id + 1),
                         static_cast<unsigned int>(actual.statusWord),
                         static_cast<unsigned int>(actual.errorCode));
            return false;
        }
    }

    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        for (uint16_t motor_id : pending_ids) {
            motor_target_buffer_[motor_id].pos = actual_data[motor_id].pos;
            motor_target_buffer_[motor_id].vel = 0.0f;
            motor_target_buffer_[motor_id].tor = 0.0f;
            motor_target_buffer_[motor_id].kp = 0.0f;
            motor_target_buffer_[motor_id].kd = 0.0f;
            motor_target_buffer_[motor_id].enabled = 1;
        }
    }

    const int timeout_cycles = static_cast<int>(timeout_seconds * 1000000.0 / TRAJ_CYCLE_US);
    int stable_cycles = 0;
    for (int cycle = 0; cycle < timeout_cycles && !stop_; ++cycle) {
        usleep(TRAJ_CYCLE_US);
        if (!getMotorActualSnapshot(actual_data)) {
            stable_cycles = 0;
            continue;
        }

        bool all_enabled = true;
        for (uint16_t motor_id : pending_ids) {
            const auto &actual = actual_data[motor_id];
            if (!communicationValid(actual.statusWord) || faultPresent(actual)) {
                RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                             "Enable aborted for alias %u: status=0x%04x error=0x%04x",
                             static_cast<unsigned int>(motor_id + 1),
                             static_cast<unsigned int>(actual.statusWord),
                             static_cast<unsigned int>(actual.errorCode));
                all_enabled = false;
                stable_cycles = -1;
                break;
            }
            all_enabled = all_enabled && operationEnabled(actual.statusWord);
        }

        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            for (uint16_t motor_id : pending_ids) {
                motor_target_buffer_[motor_id].pos = actual_data[motor_id].pos;
            }
        }

        if (stable_cycles < 0) {
            break;
        }
        stable_cycles = all_enabled ? stable_cycles + 1 : 0;
        if (stable_cycles >= kBrakeReleaseHoldCycles) {
            std::lock_guard<std::mutex> lock(target_mutex_);
            for (uint16_t motor_id : pending_ids) {
                motor_enable_ready_[motor_id] = true;
            }
            return true;
        }
    }

    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        for (uint16_t motor_id : pending_ids) {
            motor_target_buffer_[motor_id].enabled = 0;
            motor_enable_ready_[motor_id] = false;
        }
    }
    RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                 "Motor enable timed out or was aborted");
    return false;
}

bool EthercatBusManager::updateMotorTargets(const std::vector<uint16_t> &motor_ids,
                                            const std::vector<DriverSDK::motorTargetStruct> &motor_target_data) {
    if (!ready_ || motor_ids.size() != motor_target_data.size()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(target_mutex_);
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
    }
    return true;
}

void EthercatBusManager::disableAll() {
    if (motor_number_ <= 0 || motor_target_buffer_.empty()) {
        return;
    }

    std::vector<DriverSDK::motorActualStruct> actual_data(motor_number_);
    std::vector<DriverSDK::motorTargetStruct> target_data(motor_number_);
    for (int cycle = 0; cycle < kShutdownDisableCycles; ++cycle) {
        controller_.getMotorActual(actual_data);
        for (int i = 0; i < motor_number_; ++i) {
            target_data[i].pos = actual_data[i].pos;
            target_data[i].vel = 0.0f;
            target_data[i].tor = 0.0f;
            target_data[i].kp = 0.0f;
            target_data[i].kd = 0.0f;
            target_data[i].enabled = 0;
        }
        controller_.setMotorTarget(target_data);
        usleep(TRAJ_CYCLE_US);
    }
}

// bool EthercatBusManager::batchSetSoftstop(bool stop) {
//     if (!controller_) {
//         return false;
//     }
//     controller_->batchSetSoftstop(stop);
//     return true;
// }

}  // namespace miracle
