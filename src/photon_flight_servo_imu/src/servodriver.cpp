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

https://raspberrypi.stackexchange.com/questions/119440/how-to-control-a-servo-via-libgpiod-on-a-raspberry-pi

I'm leaving alot of comments because I have never used ros2 or this library before
*/
using namespace std::chrono_literals;

class ServoDriverNode : public rclcpp::Node 
{
    public: 
    ServoDriverNode()
    : Node("servo_driver_node")
    {
        /*
        NOTE:
        No subscriber/publisher definition in this class since this node acts purely as an isolated computation node for the servo 
        */
       
       //opens gpio device 
       default_gpio = lgGpiochipOpen(0);
       if (default_gpio < 0) {
           RCLCPP_ERROR(this->get_logger(), "GPIO FAILED TO OPEN");
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
            0
        );
        
        RCLCPP_INFO(this->get_logger(), "Servo pin set at %d", servo_pin); 
        // define timer and bind the callback to the object so it upates every 5 seconds (we will adjust this during testing)
        // a "walltimer" is the actual elapsed time measured by the system clock 
        timer_ = this->create_wall_timer(
            50ms, std::bind(&ServoDriverNode::update_angle_callback, this)
        );
    }
    
    ~ServoDriverNode() 
    {
        if (default_gpio >= 0) {
            lgGpiochipClose(default_gpio);
        }
    }
    
    private:
    void update_angle_callback()
    {
        //the width of PWM input determines the angle of our servo
        pulse_width += direction;
        /*
        500 = 0 degrees 
        2500 = 180 degrees
        */
       
       if (pulse_width <= 500) {
           pulse_width = 500; 
           direction = 50;
        } else if (pulse_width >= 2500) {
            pulse_width = 2500;
            direction = -50;
        }
    }
    int pulse_width;
    int direction;
    int default_gpio;
    int servo_pin;
    /*
    shared_ptrs are smart pointers that share ownership of one or multiple objects, specifically here it manages
    the timer object we created earlier this allows multiple parts of our program to share the timer safely
    */
    rclcpp::TimerBase::SharedPtr timer_;
    
};

int main(int argc, char* argv[])
{
    // initalizes ros2
    rclcpp::init(argc, argv);
    //detects type of class
    auto node = std::make_shared<ServoDriverNode>();
    // spin starts processing data from node
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
