#include "model_report.hpp"
#include <filesystem>
#include <iostream>

int main(int argc, char** argv) {
  using namespace motion_control_pkg::wbc;
  if (argc < 6) {
    std::cerr << "Usage: wbc_model_audit URDF --neutral SUPPORT_LINK SUPPORT_LINK SUPPORT_LINK [SUPPORT_LINK ...]\n"
                 "Offline URDF neutral configuration only; does not initialize ROS or hardware.\n";
    return 2;
  }
  try {
    if (std::string(argv[2]) != "--neutral") throw std::invalid_argument("explicit --neutral required");
    const std::string path = std::filesystem::canonical(argv[1]).string();
    CentroidalModel model(path);
    SupportModel support(path, std::vector<std::string>(argv + 3, argv + argc));
    const auto state = model.neutralSnapshot();
    const auto com = model.evaluate(state);
    const auto footprint = support.evaluate(com, 0.02);
    auto report = modelReport(model, state, &com, &footprint);
    report["urdf_path"] = path;
    report["support_inset_m"] = 0.02;
    report["joints"] = Json::Value(Json::arrayValue);
    for (const auto& j : model.joints()) {
      Json::Value item;
      item["name"] = j.name; item["lower"] = j.lower; item["upper"] = j.upper;
      item["velocity_limit"] = j.velocity; item["effort_limit"] = j.effort;
      item["locked_by_urdf"] = j.locked;
      report["joints"].append(item);
    }
    Json::StreamWriterBuilder writer;
    writer["indentation"] = "  ";
    std::cout << Json::writeString(writer, report) << '\n';
    return footprint.geometry_valid ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "WBC model audit failed: " << e.what() << '\n';
    return 1;
  }
}
