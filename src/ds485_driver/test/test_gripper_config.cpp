#include "ds485_driver/gripper_config.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#define CHECK(x)                                                                                   \
  do {                                                                                             \
    if (!(x))                                                                                      \
      throw std::runtime_error(std::string(#x) + ":" + std::to_string(__LINE__));                  \
  } while (0)
int main(int argc, char **argv) {
  std::string temporary = "/tmp/ds485_config_test_" + std::to_string(getpid()) + ".yaml";
  int checked = 0;
  try {
    CHECK(argc == 3);
    const int expected_current_ma = std::stoi(argv[2]);
    ds485_driver::DriverConfig config;
    std::string error;
    CHECK(ds485_driver::LoadGripperConfig(argv[1], config, error));
    CHECK(config.sides[0].gripper.open_angle == 2);
    CHECK(config.sides[0].gripper.close_angle == 981);
    CHECK(config.sides[1].gripper.open_angle == 579);
    CHECK(config.sides[1].gripper.close_angle == 1676);
    CHECK(config.sides[0].channel == 2 && config.sides[1].channel == 2);
    const auto &left = config.sides[0].protection;
    const auto &right = config.sides[1].protection;
    CHECK(left.auto_recover && left.voltage_high_v == 30 && left.voltage_low_v == 5);
    CHECK(left.temperature_trip_c == 60 && left.temperature_hysteresis_c == 5);
    CHECK(left.current_ma == expected_current_ma && left.current_detect_ms == 200);
    CHECK(left.stall_pwm == 600 && left.stall_detect_ms == 3000);
    CHECK(right.auto_recover && right.voltage_high_v == -1 && right.current_ma == -1);
    ++checked;
    const auto original = YAML::LoadFile(argv[1]);
    auto reject = [&](const YAML::Node &tree) {
      {
        std::ofstream file(temporary);
        file << tree;
      }
      config.poll_ms = 777;
      CHECK(!ds485_driver::LoadGripperConfig(temporary, config, error));
      CHECK(!error.empty());
      CHECK(config.poll_ms == 777);
      ++checked;
    };
    {
      auto tree = YAML::Clone(original);
      tree["ds485_node"]["ros__parameters"]["left"]["protection"]["current_ma"] = "invalid";
      reject(tree);
    }
    const char *bad_fields[] = {"servo_id", "channel",     "gap_ms",    "timeout_ms",
                                "open_deg", "default_pwm", "max_speed", "blocked_samples"};
    const double bad_values[] = {300, 1, -1, 0, 99999, 0, -1, 1};
    for (int i = 0; i < 8; ++i) {
      auto tree = YAML::Clone(original);
      tree["ds485_node"]["ros__parameters"]["left"][bad_fields[i]] = bad_values[i];
      reject(tree);
    }
    {
      auto tree = YAML::Clone(original);
      tree["ds485_node"]["ros__parameters"]["right"]["arm"] = "A";
      reject(tree);
    }
    {
      auto tree = YAML::Clone(original);
      tree["ds485_node"]["ros__parameters"]["left"]["port_type"] = "serial";
      reject(tree);
    }
    {
      auto tree = YAML::Clone(original);
      tree["ds485_node"]["ros__parameters"]["poll_ms"] = 0;
      reject(tree);
    }
    {
      auto tree = YAML::Clone(original);
      tree["ds485_node"]["ros__parameters"]["state_rate_hz"] = -1;
      reject(tree);
    }
    {
      auto tree = YAML::Clone(original);
      tree["ds485_node"]["ros__parameters"]["default_timeout_ms"] = 0;
      reject(tree);
    }
    CHECK(!ds485_driver::LoadGripperConfig(temporary + ".missing", config, error));
    ++checked;
    std::cout << "DS485 independent configuration: " << checked << " scenarios PASS\n";
  } catch (const std::exception &e) {
    unlink(temporary.c_str());
    std::cerr << e.what() << '\n';
    return 1;
  }
  unlink(temporary.c_str());
  return 0;
}
