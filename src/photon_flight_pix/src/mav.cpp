#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

/*
Reads the Cube+ (ArduPilot) GPS and IMU through MAVROS and republishes them in the same
format as gpscapture.cpp and imucapture.cpp so downstream nodes don't care where the data came from.
Run mavros first, e.g.
  ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyACM0:921600
MAVROS publishes
  /mavros/global_position/global  GPS fix (NavSatFix)
  /mavros/imu/data_raw            raw accelerometer + gyro straight from the autopilot, no orientation
Acceleration is in m/s^2 and angular velocity in rad/s, both in the FCU body frame
(z reads ~ +/-9.8 when sitting still)

Publishes
  gps/readings  Float64MultiArray {lat, lon, altitude, timestamp}
  /imu/data     sensor_msgs/Imu
Don't run this alongside gpscapture / imucapture, they publish on the same topics
*/
class PixNode : public rclcpp::Node {
public:
  PixNode() : Node("pix_node") {
    gps_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("gps/readings", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::NavSatFix::SharedPtr m) {
        if (m->status.status < 0) {  // STATUS_NO_FIX = -1
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No GPS fix");
          return;
        }
        std_msgs::msg::Float64MultiArray out;
        const double gps_timestamp = m->header.stamp.sec
          + static_cast<double>(m->header.stamp.nanosec) / 1e9;
        out.data = {m->latitude, m->longitude, m->altitude, gps_timestamp};
        gps_pub_->publish(out);
      });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/mavros/imu/data_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr m) {
        sensor_msgs::msg::Imu message;
        message.header.stamp = m->header.stamp;
        message.header.frame_id = "imu_link";

        message.linear_acceleration = m->linear_acceleration;
        message.angular_velocity = m->angular_velocity;

        // -1 in the first covariance entry means "no orientation provided" (REP 145)
        message.orientation_covariance[0] = -1;
        imu_pub_->publish(message);
      });
  }
private:
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PixNode>());
  rclcpp::shutdown();
}
