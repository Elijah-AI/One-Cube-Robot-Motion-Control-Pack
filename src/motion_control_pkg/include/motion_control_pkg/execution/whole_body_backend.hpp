#pragma once
#include "motion_control_pkg/execution/execution_control.hpp"
#include "motion_control_pkg/execution/whole_body_completion.hpp"
#include "motion_control_pkg/wbc/trajectory.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <cstdint>
#include <memory>

namespace motion_control_pkg::execution {
// Transport/resource boundary only. Planning, state guards, futures and Action
// terminal transitions stay in WholeBodyModule for BOTH real and virtual use.
// Implementations must make cancellation bounded and must not throw from release.
class WholeBodyExecutionBackend {
public:
  using Path=std::vector<std::vector<double>>;
  using Time=std::chrono::steady_clock::time_point;
  using Cancel=std::shared_ptr<CancellationToken>;
  virtual ~WholeBodyExecutionBackend()=default;
  virtual bool reserve_arms(bool left,bool right,std::string& error)=0;
  virtual bool reserve_legs(std::string& error)=0;
  virtual void release_arms(bool left,bool right) noexcept=0;
  virtual void release_legs() noexcept=0;
  virtual bool reserve_head(std::string& error) {error="rigid LEG MoveJ requires a head reservation";return false;}
  virtual void release_head() noexcept {}
  virtual bool read_body_state(sensor_msgs::msg::JointState&,std::string& error) {
    error="rigid LEG MoveJ needs fresh measured arm/leg/head feedback";return false;
  }
  virtual bool arm_limits(bool left,std::vector<double>& v,std::vector<double>& a,std::vector<double>& j) const=0;
  virtual bool needs_reference() const {return false;}
  virtual bool prepare(const std::vector<std::string>&,const wbc::ReferenceResult&,std::string&) {return true;}
  // Hardware mode changes and motor enables finish before the common start
  // timestamp is selected. Virtual transports may have nothing to prepare.
  virtual bool prepare_dispatch(const Path&,const Path&,const Path&,std::uint8_t,std::string&) {return true;}
  virtual bool execute_arms(const Path& left,const Path& right,std::uint8_t mode,
                           Time start,const Cancel& cancel,std::string& error)=0;
  virtual bool execute_legs(const Path& path,Time start,const Cancel& cancel,std::string& error)=0;
  // Return actual controller feedback; unsupported backends fail closed.
  virtual bool read_completion_feedback(WholeBodyCompletionFeedback&, std::string& error) {
    error = "whole-body backend has no measured completion feedback";
    return false;
  }
};
}  // namespace motion_control_pkg::execution
