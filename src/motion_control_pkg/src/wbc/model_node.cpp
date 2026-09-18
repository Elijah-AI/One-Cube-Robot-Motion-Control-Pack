#include "model_report.hpp"
#include "motion_control_pkg/wbc/static_balance.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <filesystem>
#include <functional>

namespace motion_control_pkg::wbc {
// Deliberately consumes stamped JointState only. The current MotorInfo producer
// can publish zero-filled fallback packets on a failed SDK read; it therefore
// cannot be treated as a trustworthy whole-body acquisition stream.
class ModelNode : public rclcpp::Node {
public:
  ModelNode() : Node("wbc_model") {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;  // all cached model/policy parameters require restart
    const auto path = declare_parameter<std::string>("urdf_path", "", fixed);
    model_ = std::make_unique<CentroidalModel>(path);
    urdf_path_ = std::filesystem::canonical(path).string();
    StatePolicy policy;
    support_ = std::make_unique<SupportModel>(path,
      declare_parameter<std::vector<std::string>>("support_links", std::vector<std::string>{}, fixed),
      declare_parameter<std::vector<std::string>>("package_dirs", std::vector<std::string>{}, fixed));
    age_ = declare_parameter<double>("max_state_age_sec", 0.1, fixed);
    skew_ = declare_parameter<double>("max_state_skew_sec", 0.02, fixed);
    inset_ = declare_parameter<double>("support_inset_m", 0.02, fixed);
    const auto unknown = declare_parameter<std::vector<std::string>>("unknown_steering_joints", std::vector<std::string>{}, fixed);
    if (!unknown.empty()) {
      StaticBalanceOptions options;
      options.support_links = get_parameter("support_links").as_string_array();
      options.unknown_steering = unknown; options.inset = inset_;
      balance_ = std::make_unique<StaticBalance>(path, options, get_parameter("package_dirs").as_string_array());
      policy = balance_->statePolicy();
    }
    policy.differentiate_joints = declare_parameter<std::vector<std::string>>("differentiate_joints", std::vector<std::string>{}, fixed);
    policy.filter_time_constant = declare_parameter<double>("velocity_filter_tau_sec", .03, fixed);
    state_ = std::make_unique<WholeBodyState>(model_->joints(), policy);
    tolerance_ = declare_parameter<double>("support_coplanarity_tolerance_m", 0.002, fixed);
    const auto rate = declare_parameter<double>("publish_rate_hz", 20.0, fixed);
    const auto topics = declare_parameter<std::vector<std::string>>("joint_state_topics", {"/joint_states"}, fixed);
    if (!std::isfinite(rate) || rate <= 0 || rate > 250 || topics.empty())
      throw std::invalid_argument("invalid publish rate or empty state topics");
    // Validate policies at startup, even when no observation has arrived.
    state_->snapshot(0.0, age_, skew_);
    support_->evaluate(model_->evaluate(model_->neutralSnapshot()), inset_, tolerance_);
    for (const auto& topic : topics) {
      subscriptions_.push_back(create_subscription<sensor_msgs::msg::JointState>(
        topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          const double stamp = rclcpp::Time(msg->header.stamp).seconds();
          std::string error;
          if (stamp > now().seconds() ||
              !state_->update(msg->name, msg->position, msg->velocity, stamp, error)) {
            if (error.empty()) error = "future acquisition timestamp";
            last_error_ = error;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.c_str());
          } else {
            last_error_.clear();
          }
        }));
    }
    report_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
    com_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("~/com", 10);
    polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>("~/support_polygon", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&ModelNode::tick, this));
  }
private:
  void tick() {
    const auto time = now();
    if (last_clock_ > time.seconds()) {
      state_->clear();
      last_error_ = "clock moved backwards; observations cleared";
    }
    last_clock_ = time.seconds();
    const auto snapshot = state_->snapshot(time.seconds(), age_, skew_);
    Json::Value report = modelReport(*model_, snapshot, nullptr, nullptr);
    geometry_msgs::msg::PolygonStamped polygon;
    polygon.header.frame_id = model_->rootFrame();
    polygon.header.stamp = time;
    try {
      if (snapshot.position_valid) {
        const auto com = model_->evaluate(snapshot);
        auto footprint = balance_ ? balance_->region() : support_->evaluate(com, inset_, tolerance_);
        if (balance_) footprint.signed_margin = balance_->evaluate(snapshot).margin;
        report = modelReport(*model_, snapshot, &com, &footprint);
        geometry_msgs::msg::PointStamped point;
        point.header.frame_id = model_->rootFrame();
        point.header.stamp = rclcpp::Time(static_cast<int64_t>(snapshot.oldest_stamp * 1e9), get_clock()->get_clock_type());
        point.point.x = com.com.x(); point.point.y = com.com.y(); point.point.z = com.com.z();
        com_pub_->publish(point);
        polygon.header.stamp = point.header.stamp;
        if (footprint.geometry_valid) {
          for (const auto& vertex : footprint.polygon) {
            geometry_msgs::msg::Point32 p;
            p.x = vertex.x(); p.y = vertex.y(); p.z = footprint.plane_z;
            polygon.polygon.points.push_back(p);
          }
        }
      }
    } catch (const std::exception& e) {
      report["position_valid"] = false;
      report["velocity_valid"] = false;
      report["model_error"] = e.what();
    }
    // An empty polygon clears stale support visualization when input expires.
    polygon_pub_->publish(polygon);
    report["last_packet_error"] = last_error_;
    report["urdf_path"] = urdf_path_;
    report["support_inset_m"] = inset_;
    if (balance_) {
      report["steering_com_uncertainty_m"] = balance_->bounds().com_xy;
      report["steering_support_uncertainty_m"] = balance_->bounds().support_xy;
      report["support_inset_effective_m"] = inset_ + balance_->bounds().com_xy + balance_->bounds().support_xy;
      report["com_is_nominal_steering_estimate"] = true;
    }
    std_msgs::msg::String msg;
    Json::StreamWriterBuilder writer; writer["indentation"] = "";
    msg.data = Json::writeString(writer, report);
    report_pub_->publish(msg);
  }
  std::string urdf_path_, last_error_;
  double age_{}, skew_{}, inset_{}, tolerance_{}, last_clock_{};
  std::unique_ptr<CentroidalModel> model_;
  std::unique_ptr<WholeBodyState> state_;
  std::unique_ptr<SupportModel> support_;
  std::unique_ptr<StaticBalance> balance_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> subscriptions_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr com_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace motion_control_pkg::wbc

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<motion_control_pkg::wbc::ModelNode>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("wbc_model"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
