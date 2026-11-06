#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class GpsNode : public rclcpp::Node {
public:
  GpsNode() : Node("gps_node") {
    sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::NavSatFix::SharedPtr m) {
        if (m->status.status < 0) {  // STATUS_NO_FIX = -1
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No GPS fix");
          return;
        }
        RCLCPP_INFO(get_logger(), "lat %.7f lon %.7f alt %.1f",
                    m->latitude, m->longitude, m->altitude);
      });
  }
private:
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsNode>());
  rclcpp::shutdown();
}