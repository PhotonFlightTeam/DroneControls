#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class GpsListener : public rclcpp::Node {
public:
  GpsListener() : Node("gps_listener") {
    const float FIXED_ALTITUDE = 215.1000f; // measured in meters from shoreline
    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("gps/readings", 10);
    sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "fix", 10,
      [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
     std_msgs::msg::Float64MultiArray out;
        const double gps_timestamp = msg->header.stamp.sec
          + static_cast<double>(msg->header.stamp.nanosec) / 1e9;
          // used to be msg->altitude big issue: gps altitude is very noisy 
          // SUGGESTED FUTURE FIX: use a barometer/magnometer
          // opted for fixed altitude because it atleast enforces consistency for the point cloud data 
        out.data = {msg->latitude, msg->longitude, FIXED_ALTITUDE, gps_timestamp};
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