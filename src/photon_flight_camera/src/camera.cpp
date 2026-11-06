#include <chrono>
#include <ctime>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class CameraSubscriber : public rclcpp::Node
{
public:
  CameraSubscriber()
  : Node("camera_subscriber"), camera_(0)
  {
    subscription_ = create_subscription<std_msgs::msg::Bool>(
      "take_picture",
      10,
      std::bind(
        &CameraSubscriber::picture_callback,
        this,
        std::placeholders::_1));

    if (!camera_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Could not open camera 0");
    } else {
      RCLCPP_INFO(get_logger(), "Camera opened successfully");
    }
  }

private:
  void picture_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!message->data) {
      return;
    }

    if (!camera_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Camera is not available");
      return;
    }

    cv::Mat frame;
    camera_ >> frame;

    if (frame.empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to capture frame");
      return;
    }

    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);

    std::ostringstream filename;
    filename << "picture_"
             << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S")
             << ".jpg";

    if (!cv::imwrite(filename.str(), frame)) {
      RCLCPP_ERROR(get_logger(), "Failed to save %s", filename.str().c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "Saved picture: %s", filename.str().c_str());
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  cv::VideoCapture camera_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraSubscriber>());
  rclcpp::shutdown();
  return 0;
}