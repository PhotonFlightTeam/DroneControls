#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <cstdlib>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class InternetPublisher : public rclcpp::Node
{
  public:
    InternetPublisher()
    : Node("internet_publisher_node"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::Bool>("internet_connection", 10);
      timer_ = this->create_wall_timer(
      10ms, std::bind(&InternetPublisher::internet_callback, this));
    }

  private:
    void internet_callback()
    {
      std::string host = "8.8.8.8";
      std::string pingCommand = "ping -c 1 " + host;

      int result = system(pingCommand.c_str()); // run ping command send one packet
      auto message = std_msgs::msg::Bool();

      if (result == 0) {
        message.data = true;
      } else {
        message.data = false;
      }
    
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InternetPublisher>());
  rclcpp::shutdown();
  return 0;
}