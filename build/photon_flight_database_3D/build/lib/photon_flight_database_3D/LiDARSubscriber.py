import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LiDARSubscriber(Node):


    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_listener_callback,
            10)
        self.subscription  
      
    def lidar_listener_callback(self, msg):
        
            ranges = np.array(msg.ranges)
            indices = np.arange(len(ranges))
            # apparently instead of an angle array we get the minimumn angle (-3.14) and an angle increment (size of each step away from the minimum) so we need to calculate the angle ourselves
            # https://docs.ros.org/en/humble/p/sensor_msgs/msg/LaserScan.html

            angles = msg.angle_min + (indices * msg.angle_increment)
            valid_mask = np.isfinite(ranges) # filter out bad scans

            valid_ranges = ranges[valid_mask]
            valid_angles = angles[valid_mask]

            scan_matrix = np.column_stack((valid_angles, valid_ranges)) #stack into matrix

            if len(scan_matrix) > 0:
                self.get_logger().info(f"Matrix shape: {scan_matrix.shape} (Valid points: {len(scan_matrix)})")

            first_angle = scan_matrix[0, 0]
            first_radius = scan_matrix[0, 1]
            self.get_logger().info(f"First point -> Angle: {math.degrees(first_angle):.2f}°, Radius: {first_radius:.2f}m")

            



def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LiDARSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()