#include "ds485_driver/gripper_config.h"
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace ds485_driver {
namespace {
void Require(bool condition, const std::string &message) {
  if (!condition)
    throw std::invalid_argument(message);
}
int32_t Angle(double degrees, const std::string &name) {
  Require(std::isfinite(degrees) && degrees >= -3276.8 && degrees <= 3276.7,
          name + " must be in -3276.8..3276.7 degrees");
  return static_cast<int32_t>(std::lround(degrees * 10.0));
}
} // namespace

bool LoadGripperConfig(const std::string &path, DriverConfig &output, std::string &error) {
  try {
    DriverConfig config;
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node params = root["ds485_node"]["ros__parameters"];
    Require(params.IsMap(), "expected ds485_node.ros__parameters mapping");
    if (params["poll_ms"])
      config.poll_ms = params["poll_ms"].as<int>();
    if (params["state_rate_hz"])
      config.state_rate_hz = params["state_rate_hz"].as<double>();
    if (params["default_timeout_ms"])
      config.default_timeout_ms = params["default_timeout_ms"].as<int64_t>();
    Require(config.poll_ms >= 1 && config.poll_ms <= 1000, "poll_ms must be 1..1000");
    Require(std::isfinite(config.state_rate_hz) &&
                (config.state_rate_hz == 0 ||
                 (config.state_rate_hz >= 0.1 && config.state_rate_hz <= 100)),
            "state_rate_hz must be 0 or 0.1..100");
    Require(config.default_timeout_ms > 0 && config.default_timeout_ms <= UINT32_MAX,
            "default_timeout_ms must be 1..4294967295");

    for (unsigned side = 0; side < 2; ++side) {
      const std::string name = side == 0 ? "left" : "right";
      const YAML::Node values = params[name];
      Require(values.IsMap(), name + " configuration is missing");
      GripperController::Config &c = config.sides[side];
      if (values["enabled"])
        c.enabled = values["enabled"].as<bool>();
      if (!c.enabled)
        continue;

      const std::string port =
          values["port_type"] ? values["port_type"].as<std::string>() : "marvin";
      const std::string arm =
          values["arm"] ? values["arm"].as<std::string>() : (side == 0 ? "A" : "B");
      Require(port == "marvin", name + ": integrated mode requires port_type=marvin");
      Require(arm == (side == 0 ? "A" : "B"), name + ": arm must be A for left or B for right");
      const int channel = values["channel"] ? values["channel"].as<int>() : 2;
      const int id = values["servo_id"] ? values["servo_id"].as<int>() : 1;
      Require(channel == 2 || channel == 3, name + ": channel must be 2 (COM1) or 3 (COM2)");
      Require(id >= 0 && id <= 250, name + ": servo_id must be 0..250");
      c.channel = static_cast<uint8_t>(channel);
      c.id = static_cast<uint8_t>(id);
      if (values["gap_ms"])
        c.gap_ms = values["gap_ms"].as<int>();
      if (values["timeout_ms"])
        c.reply_timeout_ms = values["timeout_ms"].as<int>();
      Require(c.gap_ms >= 0 && c.gap_ms <= 1000, name + ": gap_ms must be 0..1000");
      Require(c.reply_timeout_ms >= 10 && c.reply_timeout_ms <= 5000,
              name + ": timeout_ms must be 10..5000");

      if (const auto protection = values["protection"]) {
        Require(protection.IsMap(), name + ".protection must be a mapping");
        auto &p = c.protection;
        if (protection["auto_recover"])
          p.auto_recover = protection["auto_recover"].as<bool>();
        if (protection["voltage_high_v"])
          p.voltage_high_v = protection["voltage_high_v"].as<int64_t>();
        if (protection["voltage_low_v"])
          p.voltage_low_v = protection["voltage_low_v"].as<int64_t>();
        if (protection["temperature_trip_c"])
          p.temperature_trip_c = protection["temperature_trip_c"].as<int64_t>();
        if (protection["temperature_hysteresis_c"])
          p.temperature_hysteresis_c = protection["temperature_hysteresis_c"].as<int64_t>();
        if (protection["current_ma"])
          p.current_ma = protection["current_ma"].as<int64_t>();
        if (protection["current_detect_ms"])
          p.current_detect_ms = protection["current_detect_ms"].as<int64_t>();
        if (protection["stall_pwm"])
          p.stall_pwm = protection["stall_pwm"].as<int64_t>();
        if (protection["stall_detect_ms"])
          p.stall_detect_ms = protection["stall_detect_ms"].as<int64_t>();
      }

      c.gripper.open_angle = Angle(values["open_deg"].as<double>(), name + ".open_deg");
      c.gripper.close_angle = Angle(values["close_deg"].as<double>(), name + ".close_deg");
      Require(c.gripper.open_angle != c.gripper.close_angle,
              name + ": open_deg and close_deg must differ");
      const int pwm = values["default_pwm"] ? values["default_pwm"].as<int>() : 1000;
      const int maximum = values["max_speed"] ? values["max_speed"].as<int>() : 0;
      const int speed = values["default_speed"] ? values["default_speed"].as<int>() : 0;
      Require(pwm >= 1 && pwm <= 1000, name + ": default_pwm must be 1..1000");
      Require(maximum >= 0 && maximum <= 65535 && speed >= 0 && speed <= 65535,
              name + ": max_speed/default_speed must be 0..65535 deg/s");
      c.gripper.default_pwm = static_cast<uint16_t>(pwm);
      c.gripper.max_speed = static_cast<uint16_t>(maximum);
      c.gripper.default_speed = static_cast<uint16_t>(speed);
      const double tolerance = values["tolerance_deg"] ? values["tolerance_deg"].as<double>() : 1.0;
      const double still_band =
          values["still_band_deg"] ? values["still_band_deg"].as<double>() : 0.3;
      Require(tolerance >= 0.1 && still_band >= 0.0, name + ": invalid tolerance/still band");
      c.gripper.tolerance = Angle(tolerance, name + ".tolerance_deg");
      c.gripper.still_band = Angle(still_band, name + ".still_band_deg");
      if (values["blocked_samples"])
        c.gripper.blocked_samples = values["blocked_samples"].as<int>();
      Require(c.gripper.blocked_samples >= 2 && c.gripper.blocked_samples <= 1000000,
              name + ": blocked_samples must be 2..1000000");
    }
    output = config;
    error.clear();
    return true;
  } catch (const std::exception &e) {
    error = e.what();
    return false;
  }
}
} // namespace ds485_driver
