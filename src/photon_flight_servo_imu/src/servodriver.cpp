#include <chrono> 
#include <memory> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <lgpio.h>

/*
SOURCES: 
ROS2: Subscriber + Publisher Nodes
https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

lgpio for interacting with gpios for c++ and raspi
https://abyz.me.uk/lg/lgpio.html#lgGpiochipOpen

I'm leaving alot of comments because I have never used ros2 or this libarary before
*/

class ServoDriverNode : public rclcpp::Node 
{
    public: 
        ServoDriverNode()
        : Node("servo_driver_node")
        {
            //opens gpio device 
            default_gpio = lgGpiochipOpen(0);
            if (default_gpio < 0) {
                RCLPP_ERROR(this->get_logger(), "GPIO FAILED TO OPEN");
                return;
            }
            // NOTE: Pi 4B PWM pins; 12, 13, 18 and 19
            servo_pin = 18;

            lgGpioClaimOutput(
                // sets line flag for gpio                
                default_gpio, 
                //0 sets default behavior for pin that servos need (line flag)
                0,
                // pin we're claiming for output
                servo_pin,
                // starts the pin at LOW when claimed
                0,
            );
       
            RCLCPP_INFO(this->get_logger(), "Servo pin set at %d", servo_pin); 
        }

        ~ServoDriverNode() 
        {
            if (default_gpio >= 0) {
                lgGpiochipClose(default_gpio);
            }
        }
}