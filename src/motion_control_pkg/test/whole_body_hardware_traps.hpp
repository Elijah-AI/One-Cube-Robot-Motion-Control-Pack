#pragma once
#include "motion_control_pkg/whole_body_module.hpp"
#include "motion_control_pkg/miracle_head_module.hpp"
#include <stdexcept>
// Hardware entry points are traps; this executable links no vendor drivers.
[[noreturn]] static void hardware() {throw std::runtime_error("UNEXPECTED HARDWARE ACCESS");}
using Path=std::vector<std::vector<double>>;
using Time=std::chrono::steady_clock::time_point;
using Cancel=std::shared_ptr<motion_control_pkg::execution::CancellationToken>;
bool DualArmModule::reserve_preplanned_trajectory_resources(bool,bool,std::string&) {hardware();}
void DualArmModule::release_preplanned_trajectory_resources(bool,bool) noexcept {std::terminate();}
bool DualArmModule::get_joint_motion_limits(bool,std::vector<double>&,std::vector<double>&,std::vector<double>&) const {hardware();}
bool DualArmModule::prepare_preplanned_dispatch(const Path&,const Path&,std::uint8_t,std::string&) {hardware();}
bool DualArmModule::dispatch_preplanned(const Time&,const Cancel&,std::string&) {hardware();}
bool DualArmModule::read_preplanned_completion_feedback(motion_control_pkg::sdk::DualArmDeviceState&,std::string&) {hardware();}
bool DualArmModule::readMeasuredFeedback(sensor_msgs::msg::JointState&,std::string&,bool) {hardware();}
namespace miracle {
bool MiracleHeadModule::reserve_body_motion_resource(std::string&) {hardware();}
bool MiracleLowerBodyModule::reserve_preplanned_trajectory_resource(std::string&) {hardware();}
void MiracleLowerBodyModule::release_preplanned_trajectory_resource() noexcept {std::terminate();}
bool MiracleLowerBodyModule::prepare_preplanned_dispatch(const Path&,std::string&) {hardware();}
bool MiracleLowerBodyModule::dispatch_preplanned(const Path&,const Time&,const Cancel&,std::string&) {hardware();}
bool MiracleLowerBodyModule::read_body_orientation_feedback(std::array<double,4>&,std::array<double,4>&,uint64_t&,std::string&) {hardware();}
}
