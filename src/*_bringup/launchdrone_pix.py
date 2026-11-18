from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Same as flight_capture.launch.py but with use_pix:=true (default) the Cube+ (through mavros) replaces
# the NMEA gps driver, gps_listener and imu_publisher_node. pix_node republishes mavros data on
# gps/readings and /imu/data. use_pix:=false brings back the original gps and imu nodes instead


def generate_launch_description():
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    lidar_frame = LaunchConfiguration('lidar_frame')
    gps_port = LaunchConfiguration('gps_port')
    gps_baudrate = LaunchConfiguration('gps_baudrate')
    fcu_url = LaunchConfiguration('fcu_url')
    use_pix = LaunchConfiguration('use_pix')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_pix',
            default_value='true',
            description='true: Cube+ gps/imu through mavros + pix_node, '
                        'false: original NMEA gps and I2C imu nodes'),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB0',
            description='Serial port used by the RPLiDAR'),
        DeclareLaunchArgument(
            'lidar_baudrate',
            default_value='115200',
            description='Baud rate used by the RPLiDAR'),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='laser',
            description='TF frame assigned to the RPLiDAR'),
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyACM0:921600',
            description='Cube+ connection for mavros, device:baud (use_pix:=true)'),
        DeclareLaunchArgument(
            'gps_port',
            default_value='/dev/ttyACM0',
            description='Serial port used by the NMEA GPS (use_pix:=false)'),
        DeclareLaunchArgument(
            'gps_baudrate',
            default_value='9600',
            description='Baud rate used by the NMEA GPS (use_pix:=false)'),

        # Cube+ path: mavros_node with the ArduPilot plugin list and config (apm.launch ships with mavros)
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mavros'), 'launch', 'apm.launch'])),
            launch_arguments={'fcu_url': fcu_url}.items(),
            condition=IfCondition(use_pix)),

        Node(
            package='photon_flight_pix',
            executable='mav',
            name='pix_node',
            output='screen',
            condition=IfCondition(use_pix)),

        # Original path
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_serial_driver',
            parameters=[{
                'port': gps_port,
                'baud': gps_baudrate,
            }],
            output='screen',
            condition=UnlessCondition(use_pix)),

        Node(
            package='photon_flight_servo_imu',
            executable='gps_listener',
            name='gps_listener',
            output='screen',
            condition=UnlessCondition(use_pix)),

        Node(
            package='photon_flight_servo_imu',
            executable='imu_publisher_node',
            name='imu_publisher_node',
            output='screen',
            condition=UnlessCondition(use_pix)),

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
            package='photon_flight_database_3D',
            executable='lidar_subscriber',
            name='lidar_subscriber',
            output='screen'),
    ])
