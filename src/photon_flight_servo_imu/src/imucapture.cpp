#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cstdint>

/*
For this I employed Gemini 4 to help list out all of the data registers for the MPU-6000 / 6050 and what to write to them for starting and shutting them off
https://naylampmechatronics.com/img/cms/000033/MPU-6000-Register-Map1.pdf
kept the data sheet handy to cross refernece what was written to further understand
*/

/*Fun fact I2C only requires two signal wires SDA and SCL, since each I2C device has its own address you can communicate to multiple addresses
using the same pins on the host processor
*/
class IMUPublisher : public rclcpp::Node {
public:
    IMUPublisher() : Node("imu_publisher_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        // open i2c with standard linux file operations 
        i2c_connection = open("/dev/i2c-1", O_RDWR);
        if (i2c_connection < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open I2C bus /dev/i2c-1!");
    }   

        /*
        https://www.man7.org/linux/man-pages/man2/ioctl.2.html
        icotl() sets the behavior of this specfic file so that every read() and write() communicates over i2c to that specifc hex address 
        0x68 is the default for MPU
        */
        ioctl(i2c_connection, I2C_SLAVE, 0x68);

        // 0x6B is the power management register, set to 1 by default
        // I2C protocol demands we first identify the target we want to write to first and then the data we want to write into it 
        char config[2] = {0x6B, 0x00};

        // wakes up our IMU
        write(i2c_connection, config, 2);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&IMUPublisher::imu_callback, this));
    }

    ~IMUPublisher() {
        if (i2c_connection >= 0) close(i2c_connection);
    }

private:
    void imu_callback() {
        /*
        Buffer size here dictates what 
        measurements are read into what buffer
        */
      char start_accel_register = 0x3B;
      char start_gyro_register = 0x43;
      char accel_buffer[6];
      char gyro_buffer[6];
      int16_t accel_store[3];
      int16_t gyro_store[3];

      /*
      constexpr allows for compile time evaluation of expressions and functions
      improves program effiency and helps reduce runtime overhead

      IMUs dont put out  gs for acceleration, in the manual it outputs digital counts 
      to get the amount gs for the digital counts we divide by 16384 (in the manual this is the default) since 16384 = 1 g of force
      Gyroscope is the same way but if we want it in range of degrees we divide by 131
      */
      constexpr double ACCEL_SCALE = 16384.0;
      constexpr double GYRO_SCALE = 131.0;
      constexpr double GRAVITY = 9.80665;
      // convert to radians since most ros2 nodes play nice with them
      constexpr double DEG_TO_RAD = 0.01745329251;


      // turns on acceleration and gyroscope data registers
      // we dont need a pointer to each register since the IMUs registers have pointers to each of their adjacent register
      write(i2c_connection, &start_accel_register, 1); // point to this register
      read(i2c_connection, accel_buffer, 6); // read from it
      write(i2c_connection, &start_gyro_register, 1);
      read(i2c_connection, gyro_buffer, 6);

      /*
      HUGE TLDR essentially I2C can only send 8 byte values at a time 
      but gyro and accelerometer values are 16 signed byte values 
      so I2C splits them into two 8 byte values
      First is the most significant byte (holds the sign and a portion of the total magnitude)
      Second is the low byte (least significant byte)
      */

      // convert each to big endian or 16 signed bits
    for (int i = 0; i < 3; i++) {
        // static casting to ensure no sign issues
        accel_store[i] = static_cast<int16_t>((accel_buffer[i * 2] << 8) | accel_buffer[(i * 2) + 1]);
        gyro_store[i] = static_cast<int16_t>((gyro_buffer[i * 2] << 8) | gyro_buffer[(i * 2) + 1]);
    }

        auto message = sensor_msgs::msg::Imu();
        // lets us know the time the node was sent 
        message.header.stamp = this->now();
        message.header.frame_id = "imu_link";

        message.linear_acceleration.x = (accel_store[0] / ACCEL_SCALE) * GRAVITY;
        message.linear_acceleration.y = (accel_store[1] / ACCEL_SCALE) * GRAVITY;
        message.linear_acceleration.z = (accel_store[2] / ACCEL_SCALE) * GRAVITY;

        message.angular_velocity.x = (gyro_store[0] / GYRO_SCALE) * DEG_TO_RAD;
        message.angular_velocity.y = (gyro_store[1] / GYRO_SCALE) * DEG_TO_RAD;
        message.angular_velocity.z = (gyro_store[2] / GYRO_SCALE) * DEG_TO_RAD;

        for (auto accel : accel_store) {
            auto new_accel = (accel / ACCEL_SCALE) * GRAVITY;
            RCLCPP_INFO(this->get_logger(), "Acceleration: %.3f", new_accel);
        }


        for (auto gyro : gyro_store) {
            auto new_gyro = (gyro / GYRO_SCALE) * DEG_TO_RAD;
            RCLCPP_INFO(this->get_logger(), "GyroScope: %.3f", new_gyro);
        }

        // closure to wake up imu if connection drops
        auto wake_up = [&]() {
            uint8_t w[2] = {0x6B, 0x01};
            return write(this->i2c_connection, w, 2) == 2;
        };

        

        publisher_->publish(message);

        wake_up();

    }
    int i2c_connection;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUPublisher>());
    rclcpp::shutdown();
    return 0;
}