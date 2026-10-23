#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class GpsListener : public rclcpp::Node {
public:
  GpsListener() : Node("gps_listener") {
    sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "fix", 10,
      [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (msg->status.status < 0) {
          RCLCPP_WARN(get_logger(), "No fix yet");
          return;
        }
        RCLCPP_INFO(get_logger(), "lat %.6f  lon %.6f  alt %.1f",
                    msg->latitude, msg->longitude, msg->altitude);
      });
  }
private:
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsListener>());
  rclcpp::shutdown();
  return 0;
}