#pragma once

#include <array>
#include <mutex>
#include <memory>
#include "motion_control_pkg/loong_driver_sdk.h"
#include "motion_control_pkg/sdk/drive_control.hpp"
#include "motion_control_pkg/sdk/ethercat_feedback_api.hpp"

namespace motion_control_pkg::sdk {
// Only the existing bus worker writes DriverSDK. Requests change this mailbox.
// The maintenance override persists at disabled after either operation.
class EthercatDriveMaintenance {
public:
  static constexpr std::array<int,4> ids{{29,28,27,26}};
  explicit EthercatDriveMaintenance(std::vector<int> selected={29,28,27,26})
    : selected_(std::move(selected)) {}
  struct Receipt { bool issued{false}, reset_issued{false}; };
  struct WriteTicket {
    std::uint64_t epoch{};
    bool reset{};
    std::shared_ptr<Receipt> receipt;
  };
  DriveControlResult control(bool clear_fault, bool recovery_stop=false,
      const std::function<bool()>& cancel_clear={});
  WriteTicket update(const std::vector<OnecubeEthercatFeedbackV1>& frames, bool valid,
                     std::vector<DriverSDK::motorTargetStruct>& targets,
                     std::vector<bool>& enable_ready);
  void written(WriteTicket ticket, bool success);
  void requestShutdownDisable();
  // Only explicit recovery may release a successfully confirmed clear override.
  bool releaseAfterConfirmedClear();
  bool readShutdownFeedback(std::vector<DriveAxis>& axes, std::string& detail,
                            bool require_written) { return read(axes,detail,!require_written); }
private:
  const std::vector<int> selected_;
  bool read(std::vector<DriveAxis>& axes, std::string& detail, bool preflight);
  std::mutex mutex_, clear_mutex_;
  std::vector<OnecubeEthercatFeedbackV1> frames_;
  bool frames_valid_{false}, active_{false}, clearing_{false};
  bool reset_failed_{false};
  bool clear_confirmed_{false};
  std::shared_ptr<Receipt> receipt_;
  std::uint64_t epoch_{0};
  DriveClock::time_point reset_until_{}, write_time_{};
};
}  // namespace motion_control_pkg::sdk
