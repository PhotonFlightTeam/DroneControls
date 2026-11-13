from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    lidar_frame = LaunchConfiguration('lidar_frame')
    gps_port = LaunchConfiguration('gps_port')
    gps_baudrate = LaunchConfiguration('gps_baudrate')

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB0',
            description='Serial port used by the RPLiDAR'),
        DeclareLaunchArgument(
            'lidar_baudrate',
            default_value='115200', # old 115200
            description='Baud rate used by the RPLiDAR'),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='laser',
            description='TF frame assigned to the RPLiDAR'),
        DeclareLaunchArgument(
            'gps_port',
            default_value='/dev/ttyACM0',
            description='Serial port used by the NMEA GPS'),
        DeclareLaunchArgument(
            'gps_baudrate',
            default_value='9600',
            description='Baud rate used by the NMEA GPS'),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': lidar_port,
                'serial_baudrate': lidar_baudrate,
                'frame_id': lidar_frame,
                'angle_compensate': True,
            }],
            output='screen'),

        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_serial_driver',
            parameters=[{
                'port': gps_port,
                'baud': gps_baudrate,
            }],
            output='screen'),

        Node(
            package='photon_flight_servo_imu',
            executable='gps_listener',
            name='gps_listener',
            output='screen'),

        Node(
            package='photon_flight_servo_imu',
            executable='imu_publisher_node',
            name='imu_publisher_node',
            output='screen'),

        Node(
            package='photon_flight_database_3D',
            executable='lidar_subscriber',
            name='lidar_subscriber',
            output='screen'),
    ])
