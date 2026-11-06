from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

def setup(context):
    nodes = []
    if input('Start IMU? [y/n] ').strip().lower() == 'y':
        nodes.append(Node(package='my_imu_pkg', executable='imu_publisher_node'))
    if input('Start lidar? [y/n] ').strip().lower() == 'y':
        nodes.append(Node(package='my_lidar_pkg', executable='lidar_subscriber',
                          output='screen'))
    return nodes

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=setup)])