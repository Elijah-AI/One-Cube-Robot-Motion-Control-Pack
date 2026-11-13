#include "motion_control_pkg/execution/ethercat_command_safety.hpp"
#include "ethercat_bus_manager.hpp"
#include "motion_control_pkg/execution/stream_timing.hpp"
#include "motion_control_pkg/sdk/ethercat_feedback_api.hpp"
#include "motion_control_pkg/sdk/ethercat_feedback_validity.hpp"
#include "motion_control_pkg/sdk/ethercat_startup_readiness.hpp"
#include <dlfcn.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <limits>
#include <sstream>
#include <time.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
// #include "json_server.hpp"

namespace miracle {

namespace {

constexpr int kExpectedMotorCount = 32;
constexpr int kCommunicationStableCycles = 25;
// Six slaves may each take 5 s to synchronize DC. Bound the real elapsed
// startup time, then require continuous OP/complete PDO before any reset.
constexpr auto kCommunicationTimeout = std::chrono::seconds(60);
constexpr int kShutdownDisableCycles = 100;
constexpr int kStartupFaultResetCycles = 200000 / TRAJ_CYCLE_US;
constexpr int kStartupFaultResetTimeoutCycles = 2000000 / TRAJ_CYCLE_US;
// Same EtherCAT aliases as LEG_JOINT1/2/3 and BODY_JOINT in main_node.hpp.
constexpr std::array<int, 4> kLowerBodyMotorIds{{30 - 1, 29 - 1, 28 - 1, 27 - 1}};
// Include head aliases 31/32: PREOP/OP loss can leave 0xff32 latched there too.
constexpr std::array<int, 6> kStartupFaultResetMotorIds{{29,28,27,26,31,30}};

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
bool clearStartupEthercatFaults(
    DriverSDK::DriverSDK &controller, const std::vector<int> &active_motors,
    const std::vector<DriverSDK::motorTargetStruct> &initial_targets) {
    std::vector<int> motor_ids;
    for (int id : kStartupFaultResetMotorIds) {
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
                     "EtherCAT startup fault reset requires valid motor feedback");
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
        if (motion_control_pkg::execution::shutdownRequested()) { reset_sent=false; break; }
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
                     "EtherCAT startup fault reset failed to exchange motor data");
        return false;
    }

    int stable_cycles = 0;
    for (int cycle = 0; cycle < kStartupFaultResetTimeoutCycles; ++cycle) {
        if (motion_control_pkg::execution::shutdownRequested()) return false;
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
                        "EtherCAT startup fault reset confirmed; enabled=0 until normal enable");
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
    // Check release capability before claiming any master or enabling drives.
    if (!dlsym(RTLD_DEFAULT,"onecube_shutdown_ethercat_v1"))
        throw std::runtime_error("matching DriverSDK shutdown API is required");
    if (!dlsym(RTLD_DEFAULT,"onecube_begin_head_pitch_recovery_v1") ||
        !dlsym(RTLD_DEFAULT,"onecube_end_head_pitch_recovery_v1"))
        throw std::runtime_error("matching DriverSDK scoped head startup recovery API is required");
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
    const bool communication_ready=waitForCommunication();
    if (!initializeTargetBuffer()) return;
    bool startup_ok=communication_ready && !motion_control_pkg::execution::shutdownRequested();
    if (startup_ok)
        startup_ok=clearStartupEthercatFaults(controller_, active_motors_, motor_target_buffer_) &&
                   initializeTargetBuffer();
    // Even a canceled/failed startup retains a disabled PDO worker for measured
    // shutdown. It must never fall through to normal motion admission.
    ready_ = startup_ok && !motion_control_pkg::execution::shutdownRequested();
    drive_maintenance_available_=std::all_of(kLowerBodyMotorIds.begin(),kLowerBodyMotorIds.end(),
        [&](int id){return std::find(active_motors_.begin(),active_motors_.end(),id)!=active_motors_.end();});
    head_maintenance_available_=std::find(active_motors_.begin(),active_motors_.end(),31)!=active_motors_.end() &&
        std::find(active_motors_.begin(),active_motors_.end(),30)!=active_motors_.end();
    worker_ = std::make_shared<std::thread>(&EthercatBusManager::run, this);
}

EthercatBusManager::~EthercatBusManager() {
    ready_ = false;
    stop_ = true;
    if (worker_ && worker_->joinable()) {
        worker_->join();
    }
    if (!released_) disableAll();
}

bool EthercatBusManager::waitForCommunication() {
    const auto logger = rclcpp::get_logger("ethercat_bus_manager");
    const auto read_bus = reinterpret_cast<OnecubeReadEthercatBusStateV1>(
        dlsym(RTLD_DEFAULT, "onecube_read_ethercat_bus_state_v1"));
    const auto read_frames = reinterpret_cast<OnecubeReadEthercatFeedbackV1>(
        dlsym(RTLD_DEFAULT, "onecube_read_ethercat_feedback_v1"));
    if (!read_bus || !read_frames) {
        RCLCPP_ERROR(logger, "DriverSDK lacks OP/PDO readiness API; install the matching SDK before startup");
        return false;
    }
    std::vector<DriverSDK::motorActualStruct> actual(motor_number_);
    std::vector<OnecubeEthercatFeedbackV1> frames(motor_number_);
    OnecubeEthercatBusStateV1 bus{};
    const auto started = std::chrono::steady_clock::now();
    auto next_log = started;
    RCLCPP_INFO(logger, "Waiting up to 60 s for all 6 EtherCAT slaves OP and complete PDO stable for 2 s before startup reset/home");
    while (rclcpp::ok() && !motion_control_pkg::execution::shutdownRequested() &&
           std::chrono::steady_clock::now()-started < kCommunicationTimeout) {
        controller_.advance();
        usleep(TRAJ_CYCLE_US);
        const bool read_ok = controller_.getMotorActual(actual)==0 &&
            read_bus(0, &bus, sizeof(bus))==1 &&
            read_frames(frames.data(),frames.size(),sizeof(OnecubeEthercatFeedbackV1))==motor_number_;
        timespec stamp{};
        clock_gettime(CLOCK_MONOTONIC, &stamp);
        const auto now_ns = static_cast<std::int64_t>(stamp.tv_sec)*1000000000LL+stamp.tv_nsec;
        bool valid = read_ok && motion_control_pkg::sdk::ethercatStartupReady(bus,frames,now_ns);
        for (int id : active_motors_) {
            if (id<0 || id>=motor_number_ || !communicationValid(actual[id].statusWord)) valid=false;
        }
        const auto now = std::chrono::steady_clock::now();
        const double stable_sec = bus.ready_since_ns>0 ?
            static_cast<double>(bus.acquired_ns-bus.ready_since_ns)/1e9 : 0.0;
        if (valid) {
            RCLCPP_INFO(logger, "EtherCAT startup ready: all 6 slaves OP, PDO complete %u/%u, stable %.3f s, waited %.3f s",
                bus.complete_domains,bus.domain_count,stable_sec,std::chrono::duration<double>(now-started).count());
            return true;
        }
        if (now>=next_log) {
            RCLCPP_INFO(logger, "Waiting for EtherCAT OP: slaves=%u/%u AL=0x%02x link=%u PDO=%u/%u stable=%.3f s",
                bus.responding_slaves,bus.expected_slaves,bus.al_states,bus.link_up,
                bus.complete_domains,bus.domain_count,stable_sec);
            next_log=now+std::chrono::seconds(5);
        }
    }
    RCLCPP_ERROR(logger, "EtherCAT startup stopped: OP/PDO not stable within 60 s or shutdown requested; no startup reset/home sent");
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
    const auto read_bus=reinterpret_cast<OnecubeReadEthercatBusStateV1>(
        dlsym(RTLD_DEFAULT,"onecube_read_ethercat_bus_state_v1"));
    const auto read_device_frame=reinterpret_cast<OnecubeReadEthercatFeedbackV1>(
        dlsym(RTLD_DEFAULT,"onecube_read_ethercat_feedback_v1"));
    std::vector<OnecubeEthercatFeedbackV1> device_frames(static_cast<std::size_t>(motor_number_));
    if (!read_device_frame)
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
            "DriverSDK lacks PDO acquisition timestamps; measured motion feedback is unavailable until the matching SDK patch is installed");


    std::vector<DriverSDK::motorActualStruct> actual_data(motor_number_);
    std::vector<DriverSDK::motorTargetStruct> target_data(motor_number_);
    timespec next_wakeup_time{};
    clock_gettime(CLOCK_MONOTONIC, &next_wakeup_time);
    const auto nanoseconds = [](const timespec& t) {
        return static_cast<std::int64_t>(t.tv_sec) * 1000000000LL + t.tv_nsec;
    };

    while (!stop_) {
        timespec cycle_started{};
        clock_gettime(CLOCK_MONOTONIC, &cycle_started);
        const int actual_result = controller_.getMotorActual(actual_data);
        bool timestamped = actual_result==0 && read_device_frame &&
            read_device_frame(device_frames.data(),device_frames.size(),sizeof(OnecubeEthercatFeedbackV1))==motor_number_;
        OnecubeEthercatBusStateV1 bus{};
        timespec bus_checked{};
        const bool bus_read = read_bus && read_bus(0,&bus,sizeof(bus))==1;
        clock_gettime(CLOCK_MONOTONIC,&bus_checked);
        // Keep the head preflight gated if OP/PDO is lost after constructor
        // readiness but before the asynchronous startup-home thread begins.
        timestamped = timestamped && bus_read &&
            motion_control_pkg::sdk::ethercatStartupReady(bus,device_frames,nanoseconds(bus_checked));
        const bool raw_frames_valid=timestamped;
        std::int64_t oldest_ns=std::numeric_limits<std::int64_t>::max();
        std::uint64_t oldest_sequence=std::numeric_limits<std::uint64_t>::max();
        // These are the six EtherCAT joints consumed by this robot's motion
        // modules; other logical indices can belong to inactive/CAN devices.
        for (int id:{29,28,27,26,31,30}) {
            if (!timestamped || static_cast<std::size_t>(id)>=device_frames.size()) {timestamped=false;break;}
            const auto& frame=device_frames[id];
            // Disabled drives still provide measured position for startup and
            // idle monitoring. Faults and invalid status still reject the view;
            // command dispatch checks operation-enabled independently.
            if (!frame.sequence || frame.acquired_ns<=0 ||
                !motion_control_pkg::sdk::ethercatFeedbackValid(frame.status,frame.error)) {timestamped=false;break;}
            oldest_ns=std::min(oldest_ns,frame.acquired_ns);
            oldest_sequence=std::min(oldest_sequence,frame.sequence);
        }
        if (!timestamped) {
            std::ostringstream detail;
            detail << "SDK_read=" << actual_result << " raw_frames_valid=" << raw_frames_valid
                << " bus_read=" << bus_read << " slaves=" << bus.responding_slaves << '/' << bus.expected_slaves
                << " AL=0x" << std::hex << bus.al_states << std::dec
                << " PDO=" << bus.complete_domains << '/' << bus.domain_count;
            for (int id:{29,28,27,26,31,30}) {
                if (static_cast<std::size_t>(id)>=device_frames.size()) continue;
                const auto& f=device_frames[id];
                detail << " alias=" << id+1 << " status=0x" << std::hex << f.status
                    << " error=0x" << f.error << std::dec << " frame=" << f.sequence
                    << " position_rad=" << f.position << " velocity_rad_s=" << f.velocity
                    << " age_ms=" << (nanoseconds(bus_checked)-f.acquired_ns)/1e6;
            }
            static rclcpp::Clock feedback_log_clock(RCL_STEADY_TIME);
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ethercat_bus_manager"),feedback_log_clock,2000,
                "Measured EtherCAT feedback rejected: %s",detail.str().c_str());
        }
        if (timestamped) for (std::size_t id=0;id<device_frames.size();++id) {
            const auto& frame=device_frames[id];
            if (!frame.sequence) continue;
            actual_data[id].pos=frame.position;actual_data[id].vel=frame.velocity;actual_data[id].tor=frame.effort;
            actual_data[id].statusWord=frame.status;actual_data[id].errorCode=frame.error;
        }
        {
            std::lock_guard<std::mutex> lock(actual_mutex_);
            actual_read_valid_ = actual_result == 0;
            actual_device_timestamped_ = timestamped;
            if (actual_read_valid_) motor_actual_snapshot_ = actual_data;
            if (timestamped && oldest_sequence != actual_sequence_) {
                motor_timed_snapshot_ = actual_data;
                actual_acquired_=std::chrono::steady_clock::time_point(std::chrono::nanoseconds(oldest_ns));
                actual_sequence_=oldest_sequence;
            }
        }
        motion_control_pkg::sdk::EthercatDriveMaintenance::WriteTicket maintenance_ticket,head_ticket,shutdown_ticket;
        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            for (int motor_id : active_motors_) {
                // Log the transition once, not every bus cycle. Values from a
                // failed SDK read are retained data, not fresh feedback.
                const auto &actual = actual_data[motor_id];
                const char *reason = actual_result != 0 ? "feedback_read_failed" :
                    faultPresent(actual) ? "motor_fault" :
                    !communicationValid(actual.statusWord) ? "invalid_status_word" :
                    !operationEnabled(actual.statusWord) ? "operation_not_enabled" : nullptr;
                if (reason && motor_enable_ready_[motor_id]) {
                    RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                        "Motor enable readiness lost: reason=%s motor_id=%d alias=%d "
                        "sdk_read_result=%d feedback_valid=%d status=0x%04x error=0x%04x "
                        "actual_pos=%.9f actual_vel=%.9f target_pos=%.9f target_enabled=%d",
                        reason, motor_id, motor_id + 1, actual_result, actual_result == 0,
                        static_cast<unsigned int>(actual.statusWord),
                        static_cast<unsigned int>(actual.errorCode),
                        static_cast<double>(actual.pos), static_cast<double>(actual.vel),
                        static_cast<double>(motor_target_buffer_[motor_id].pos),
                        static_cast<int>(motor_target_buffer_[motor_id].enabled));
                }
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
            // Apply AFTER the normal fault-to-disable guard, otherwise it would
            // erase enabled=-1. No other thread writes the DriverSDK target.
            maintenance_ticket=drive_maintenance_.update(
                device_frames,raw_frames_valid,motor_target_buffer_,motor_enable_ready_);
            head_ticket=head_maintenance_.update(
                device_frames,raw_frames_valid,motor_target_buffer_,motor_enable_ready_);
            // Shutdown has priority over clear/reset and includes head axes.
            shutdown_ticket=shutdown_maintenance_.update(
                device_frames,raw_frames_valid,motor_target_buffer_,motor_enable_ready_);
            target_data = motor_target_buffer_;
        }

        if (!target_data.empty()) {
            const bool written=controller_.setMotorTarget(target_data)==0;
            drive_maintenance_.written(maintenance_ticket,written);
            head_maintenance_.written(head_ticket,written);
            shutdown_maintenance_.written(shutdown_ticket,written);
            if (!written) {
                RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                             "setMotorTarget failed");
                if (const auto safety=motion_control_pkg::execution::motionSafety())
                    safety->requestStop("EtherCAT SDK setMotorTarget failed");
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ethercat_bus_manager"), "No motor target set");
            controller_.advance();
        }

        timespec completed{};
        clock_gettime(CLOCK_MONOTONIC, &completed);
        // This worker refreshes a held target, not a queued trajectory. A slow
        // bus transaction must not be followed by a burst of repeated writes.
        const auto next = motion_control_pkg::execution::nextStreamTick(
            nanoseconds(next_wakeup_time), nanoseconds(cycle_started),
            nanoseconds(completed), TRAJ_CYCLE_US * 1000LL);
        next_wakeup_time.tv_sec = next / 1000000000LL;
        next_wakeup_time.tv_nsec = next % 1000000000LL;

        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup_time, nullptr) == EINTR) {
        }
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
    if (!actual_read_valid_ || !actual_device_timestamped_ || motor_actual_snapshot_.empty() || actual_sequence_ == 0)
        return false;
    data = motor_timed_snapshot_;
    acquired = actual_acquired_;
    sequence = actual_sequence_;
    return true;
}

bool EthercatBusManager::beginHeadPitchStartupRecovery(
    double lower, double initial, double timeout, std::string& error) {
    using Begin = int (*)(double,double,double);
    const auto begin=reinterpret_cast<Begin>(dlsym(RTLD_DEFAULT,"onecube_begin_head_pitch_recovery_v1"));
    const auto monitor=motion_control_pkg::execution::motionSafety();
    if (motion_control_pkg::execution::shutdownRequested() || !ready_ || stop_ ||
        !monitor || !monitor->ready(error) || !begin || begin(lower,initial,timeout)!=1) {
        if(error.empty())error="SDK refused scoped head pitch startup recovery";
        return false;
    }
    return true;
}

void EthercatBusManager::endHeadPitchStartupRecovery() noexcept {
    using End = void (*)();
    const auto end=reinterpret_cast<End>(dlsym(RTLD_DEFAULT,"onecube_end_head_pitch_recovery_v1"));
    if(end)end();
}

motion_control_pkg::sdk::DriveControlResult EthercatBusManager::recoverAllDrives(
    const std::function<bool(const motion_control_pkg::sdk::EthercatEnableSample&,std::string&)>& check,
    const std::function<bool()>& canceled) {
    using namespace motion_control_pkg::sdk;
    DriveControlResult result;
    std::unique_lock<std::mutex> enable_lock(enable_mutex_,std::try_to_lock);
    if (!enable_lock.owns_lock() || !ready_ || stop_ || !drive_maintenance_available_ ||
        !head_maintenance_available_ || !check || canceled()) {
        result.message="whole-body recovery unavailable or busy";return result;
    }
    const std::vector<std::uint16_t> ids{29,28,27,26,31,30};
    const auto safe=[&](std::string& error) {
        if (motion_control_pkg::execution::shutdownRequested() || canceled() || stop_ || !ready_) {
            error="whole-body recovery canceled";return false;
        }
        EthercatEnableSample sample;
        return getTimedMotorActualSnapshot(sample.motors,sample.acquired,sample.sequence) && check(sample,error);
    };
    if (!safe(result.message)) return result;
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        if (canceled() || !drive_maintenance_.releaseAfterConfirmedClear() ||
            !head_maintenance_.releaseAfterConfirmedClear()) {
            drive_maintenance_.requestShutdownDisable();
            head_maintenance_.requestShutdownDisable();
            result.message="both body and head must have confirmed clears before enabling";return result;
        }
    }
    // A fresh handshake instance is reserved for this explicit service call;
    // the ordinary one-shot startup/automatic-enable policy stays unchanged.
    EthercatInitialEnable recovery_enable;
    result.feedback_confirmed=recovery_enable.run(ids,5.0,
        [&](EthercatEnableSample& sample,std::string& error) {
            return getTimedMotorActualSnapshot(sample.motors,sample.acquired,sample.sequence) && check(sample,error);
        },
        [&](const auto& targets,std::string& error) {
            if (!safe(error)) return false;
            std::lock_guard<std::mutex> lock(target_mutex_);
            if (canceled() || motion_control_pkg::execution::shutdownRequested()) return false;
            for (std::size_t i=0;i<ids.size();++i) motor_target_buffer_[ids[i]]=targets[i];
            result.command_issued=true;return true;
        },
        [&] {
            std::lock_guard<std::mutex> lock(target_mutex_);
            for (auto id:ids) {motor_target_buffer_[id].enabled=0;motor_enable_ready_[id]=false;}
            drive_maintenance_.requestShutdownDisable();head_maintenance_.requestShutdownDisable();
        },safe,result.message);
    if (result.feedback_confirmed) {
        std::lock_guard<std::mutex> lock(target_mutex_);
        if (canceled() || motion_control_pkg::execution::shutdownRequested()) {
            result.feedback_confirmed=false;
            drive_maintenance_.requestShutdownDisable();head_maintenance_.requestShutdownDisable();
        } else {
            for (auto id:ids) motor_enable_ready_[id]=true;
        }
    }
    result.message=(result.feedback_confirmed?"head and waist/legs enabled at measured positions; ":
        "body/head recovery enable unconfirmed; ")+result.message;
    return result;
}

bool EthercatBusManager::enableMotors(const std::vector<uint16_t> &motor_ids,
                                      double timeout_seconds) {
    if (motion_control_pkg::execution::shutdownRequested() || !ready_ || stop_ || motor_ids.empty() || !std::isfinite(timeout_seconds) || timeout_seconds <= 0.0)
        return false;
    const auto monitor = motion_control_pkg::execution::motionSafety();
    if (monitor && monitor->latched()) return false;

    std::lock_guard<std::mutex> enable_lock(enable_mutex_);
    bool all_ready = true;
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        for (auto id : motor_ids) {
            if (id >= motor_target_buffer_.size() ||
                std::find(active_motors_.begin(), active_motors_.end(), id) == active_motors_.end()) {
                RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                             "Initial enable rejected: inactive motor id=%u", static_cast<unsigned>(id));
                return false;
            }
            all_ready = all_ready && motor_enable_ready_[id];
        }
    }
    if (all_ready) return true;

    std::string error;
    const bool enabled = initial_enable_.run(motor_ids, timeout_seconds,
        [&](motion_control_pkg::sdk::EthercatEnableSample& sample, std::string& reason) {
            if (getTimedMotorActualSnapshot(sample.motors, sample.acquired, sample.sequence)) return true;
            reason = "initial enable requires timestamped PDO feedback"; return false;
        },
        [&](const std::vector<DriverSDK::motorTargetStruct>& targets, std::string& reason) {
            motion_control_pkg::execution::EthercatCommandSafety permit(motor_ids, targets);
            if (!permit) {reason = permit.error(); return false;}
            std::lock_guard<std::mutex> lock(target_mutex_);
            if (stop_ || !ready_) {reason = "bus stopped during initial enable"; permit.failed(); return false;}
            for (std::size_t i = 0; i < motor_ids.size(); ++i)
                motor_target_buffer_[motor_ids[i]] = targets[i];
            if (!permit.committed()) {reason = "initial enable safety commit failed"; return false;}
            return true;
        },
        [&] {
            // Failed handshake releases its enable request. No fault reset,
            // mode change or second attempt is issued.
            std::lock_guard<std::mutex> lock(target_mutex_);
            for (auto id : motor_ids) {
                motor_target_buffer_[id].enabled = 0;
                motor_enable_ready_[id] = false;
            }
        },
        [&](std::string& reason) {
            if (motion_control_pkg::execution::shutdownRequested() || stop_ || !ready_) {reason = "bus unavailable during initial enable"; return false;}
            return !monitor || monitor->ready(reason);
        }, error);
    if (!enabled) {
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                     "Initial motor enable refused/failed: %s", error.c_str());
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        for (auto id : motor_ids) motor_enable_ready_[id] = true;
    }
    RCLCPP_INFO(rclcpp::get_logger("ethercat_bus_manager"),
                "Initial motor enable confirmed for %zu axes; held position, fresh enabled PDOs for 600 ms",
                motor_ids.size());
    return true;
}

bool EthercatBusManager::updateMotorTargets(const std::vector<uint16_t> &motor_ids,
                                            const std::vector<DriverSDK::motorTargetStruct> &motor_target_data) {
    motion_control_pkg::execution::EthercatCommandSafety safety(motor_ids,motor_target_data);
    if (!safety) {
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                     "updateMotorTargets rejected: safety gate: %s (ids=%zu targets=%zu)",
                     safety.error().c_str(), motor_ids.size(), motor_target_data.size());
        return false;
    }

    if (!ready_ || motor_ids.size() != motor_target_data.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                     "updateMotorTargets rejected: ready=%d ids=%zu targets=%zu",
                     ready_.load(), motor_ids.size(), motor_target_data.size());
        safety.failed(); return false;
    }

    std::lock_guard<std::mutex> lock(target_mutex_);
    if (motion_control_pkg::execution::shutdownRequested() || motor_target_buffer_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                     "updateMotorTargets rejected: empty target buffer");
        safety.failed(); return false;
    }

    for (size_t i = 0; i < motor_ids.size(); ++i) {
        const uint16_t motor_id = motor_ids[i];
        const auto &target = motor_target_data[i];
        const char *reason = motor_id >= motor_target_buffer_.size() ? "motor_index_out_of_range" :
            std::find(active_motors_.begin(), active_motors_.end(), motor_id) == active_motors_.end() ? "motor_not_active" :
            (target.enabled != 0 && target.enabled != 1) ? "invalid_enable_value" :
            (target.enabled == 1 && !motor_enable_ready_[motor_id]) ? "motor_enable_not_ready" : nullptr;
        if (reason) {
            RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                         "updateMotorTargets rejected: reason=%s index=%zu motor_id=%u alias=%u "
                         "target_pos=%.9f target_enabled=%d buffer_size=%zu",
                         reason, i, static_cast<unsigned int>(motor_id),
                         static_cast<unsigned int>(motor_id + 1),
                         static_cast<double>(target.pos), static_cast<int>(target.enabled),
                         motor_target_buffer_.size());
            safety.failed(); return false;
        }
    }
    for (size_t i = 0; i < motor_ids.size(); ++i) {
        const uint16_t motor_id = motor_ids[i];
        motor_target_buffer_[motor_id] = motor_target_data[i];
    }
    const bool committed = safety.committed();
    if (!committed) {
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_bus_manager"),
                     "updateMotorTargets rejected: safety commit failed after target buffer update");
    }
    return committed;
}

bool EthercatBusManager::requestShutdownDisable(std::string& error) {
    if (!motion_control_pkg::execution::shutdownRequested() || !worker_ || stop_) {
        error="shutdown PDO worker unavailable"; return false;
    }
    shutdown_maintenance_.requestShutdownDisable();
    return true;  // Only queued. readShutdownFeedback(true) requires actual write + newer PDO.
}

bool EthercatBusManager::releaseAfterShutdown(std::string& error) {
    if (released_) return true;
    using Release=int(*)();
    const auto release=reinterpret_cast<Release>(dlsym(RTLD_DEFAULT,"onecube_shutdown_ethercat_v1"));
    if (!release) {error="DriverSDK release API unavailable"; return false;}
    ready_=false; stop_=true;
    if (worker_ && worker_->joinable()) worker_->join();
    { std::lock_guard<std::mutex> lock(actual_mutex_);
      actual_read_valid_=false; actual_device_timestamped_=false; }
    if (release()!=1) {error="DriverSDK EtherCAT release failed"; return false;}
    released_=true;
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
