#include "motion_control_pkg/sdk/ethercat_feedback_validity.hpp"
#include <iostream>

int main() {
  using namespace motion_control_pkg::sdk;
  struct Case { unsigned short status, error; bool feedback, operational; };
  const Case cases[] = {
    {0x0231, 0, true, false}, {0x0331, 0, true, false},
    {0x0040, 0, true, false}, {0x0021, 0, true, false},
    {0x0023, 0, true, false}, {0x0037, 0, true, true},
    {0x0237, 0, true, true}, {0x0337, 0, true, true},
    {0x0000, 0, false, false}, {0xffff, 0, false, false},
    {0x0008, 0, false, false}, {0x0339, 0, false, false},
    {0x003f, 0, false, false}, {0x0037, 0x7121, false, false},
    {0x0331, 1, false, false}
  };
  unsigned checked = 0;
  for (const auto& c : cases) {
    DriverSDK::motorActualStruct motor{};
    motor.statusWord = c.status;
    motor.errorCode = c.error;
    if (ethercatFeedbackValid(c.status, c.error) != c.feedback ||
        ethercatFeedbackValid(motor) != c.feedback ||
        ethercatFeedbackOperational(motor) != c.operational) {
      std::cerr << "feedback/operation gate mismatch: status=" << c.status
                << " error=" << c.error << '\n';
      return 1;
    }
    ++checked;
  }
  std::cout << "PASS: " << checked << " disabled/enabled/fault feedback cases\n";
}
