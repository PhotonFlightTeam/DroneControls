#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class GpsListener : public rclcpp::Node {
public:
  GpsListener() : Node("gps_listener") {
    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("gps/readings", 10);
    sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "fix", 10,
      [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
     std_msgs::msg::Float64MultiArray out;
        out.data = {msg->latitude, msg->longitude, msg->altitude};
        pub_->publish(out);
   });
  }
private:
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsListener>());
  rclcpp::shutdown();
  return 0;
}