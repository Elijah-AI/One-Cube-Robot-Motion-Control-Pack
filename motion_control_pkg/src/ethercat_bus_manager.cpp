#include "ethercat_bus_manager.hpp"

#include <time.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
// #include "json_server.hpp"

namespace miracle {

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
    
    controller_.init(xmlFile);  //init先调用
    motor_number_ = controller_.getTotalMotorNr();
    if (motor_number_ <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"), "No motor found");
        return;
    }

    std::vector<char> mode(motor_number_, static_cast<char>(8));  //每个电机模式为8
    controller_.setMode(mode);

    initializeTargetBuffer();
    worker_ = std::make_shared<std::thread>(&EthercatBusManager::run, this);
}

EthercatBusManager::~EthercatBusManager() {
    stop_ = true;
    if (worker_ && worker_->joinable()) {
        worker_->join();
    }
}

void EthercatBusManager::initializeTargetBuffer() {
    std::vector<DriverSDK::motorActualStruct> motor_actual_data(motor_number_);
    motor_target_buffer_.resize(motor_number_);
    
    controller_.advance();
    usleep(TRAJ_CYCLE_US);
    controller_.advance();
    usleep(TRAJ_CYCLE_US);

    controller_.getMotorActual(motor_actual_data);
    {
        std::lock_guard<std::mutex> lock(actual_mutex_);
        motor_actual_snapshot_ = motor_actual_data;
    }
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        for (size_t i = 0; i < motor_actual_data.size(); ++i) {
            motor_target_buffer_[i].pos = motor_actual_data[i].pos;
            motor_target_buffer_[i].vel = 0.0f;
            motor_target_buffer_[i].tor = 0.0f;
            motor_target_buffer_[i].kp = 0.0f;
            motor_target_buffer_[i].kd = 0.0f;
            motor_target_buffer_[i].enabled = -1;
        }
    }
}

void EthercatBusManager::run() {

    std::vector<DriverSDK::motorActualStruct> actual_data(motor_number_);
    std::vector<DriverSDK::motorTargetStruct> target_data(motor_number_);
    timespec next_wakeup_time{};
    clock_gettime(CLOCK_MONOTONIC, &next_wakeup_time);

    while (!stop_) {
      
        bool need_reset_enable_flag = false;

        controller_.getMotorActual(actual_data);
        {
            std::lock_guard<std::mutex> lock(actual_mutex_);
            motor_actual_snapshot_ = actual_data;
        }
        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            target_data = motor_target_buffer_;
        }

        if (!target_data.empty()) {
            controller_.setMotorTarget(target_data);
            need_reset_enable_flag = !clear_error_command_sent_;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ethercat_bus_manager"), "No motor target set");
        }

        if (need_reset_enable_flag) {
            std::lock_guard<std::mutex> lock(target_mutex_);
            for (auto &target : motor_target_buffer_) {
                if (target.enabled == -1) {
                    target.enabled = 1;     //启动电机,释放刹车，500ms后才可以动作
                }
            }
            clear_error_command_sent_ = true;
        }
        controller_.advance();

        next_wakeup_time.tv_nsec += TRAJ_CYCLE_US * 1000;
        while (next_wakeup_time.tv_nsec >= 1000000000L) {
            next_wakeup_time.tv_nsec -= 1000000000L;
            ++next_wakeup_time.tv_sec;
        }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup_time, nullptr);
    }
}

bool EthercatBusManager::getMotorActualSnapshot(std::vector<DriverSDK::motorActualStruct> &motor_actual_data) const {
    std::lock_guard<std::mutex> lock(actual_mutex_);
    if (motor_actual_snapshot_.empty()) {
        return false;
    }
    motor_actual_data = motor_actual_snapshot_;
    return true;
}

bool EthercatBusManager::updateMotorTargets(const std::vector<uint16_t> &motor_ids,
                                            const std::vector<DriverSDK::motorTargetStruct> &motor_target_data) {
    if (motor_ids.size() != motor_target_data.size()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(target_mutex_);
    if (motor_target_buffer_.empty()) {
        return false;
    }

    for (size_t i = 0; i < motor_ids.size(); ++i) {
        const uint16_t motor_id = motor_ids[i];
        if (motor_id >= motor_target_buffer_.size()) {
            return false;
        }
        motor_target_buffer_[motor_id] = motor_target_data[i];
    }
    return true;
}

// bool EthercatBusManager::batchSetSoftstop(bool stop) {
//     if (!controller_) {
//         return false;
//     }
//     controller_->batchSetSoftstop(stop);
//     return true;
// }

}  // namespace miracle
