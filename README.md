# DroneControls

DroneControls is a ROS 2 workspace for collecting flight sensor data, recording RPLiDAR scans, and converting recorded 2D LiDAR flights into a 3D point cloud.

The project combines LiDAR, IMU, GPS, camera, internet-status, servo, and PX4/MAVROS-related nodes into a single workspace for drone research and flight testing.

## What it does

- Records synchronized angle and range measurements from an RPLiDAR.
- Associates LiDAR points with accelerometer, gyroscope, timestamp, and GPS readings.
- Saves validated flight matrices at shutdown as NumPy `.npy` files.
- Estimates roll, pitch, and yaw from IMU data.
- Generates a LAS point cloud with `laspy`.
- Provides camera capture, servo control, internet-status, GPS, IMU, and PX4/MAVROS utility nodes.

## Current data flow

```text
RPLiDAR (/scan)
      |
      +--> LiDARSubscriber
GPS (/gps/readings) --/       |
IMU (/imu/data) ------+       +--> 2D_Flight_Matrix_N.npy
                                      |
                                      v
                              LiDAR3DConversion
                                      |
                                      v
                      3D_Flight_PointCloud.las
```

## Flight matrix format

Each flight matrix is stored as one rectangular NumPy 2D array after valid LiDAR scan blocks are vertically stacked.

The current expected layout is **12 columns**:

| Columns | Data |
|---|---|
| 0 | LiDAR angle |
| 1 | LiDAR range |
| 2–4 | Linear acceleration (x, y, z) |
| 5–7 | Angular velocity (x, y, z) |
| 8 | Gyroscope/IMU timestamp |
| 9–11 | GPS latitude, longitude, altitude |

Matrix shape:

```text
(total LiDAR samples, 12)
```

The expected column count is calculated from the IMU and GPS arrays plus the two LiDAR columns. Scans with missing or inconsistent sensor data are discarded during shutdown saving.

## Repository layout

| Path | Description |
|---|---|
| `src/photon_flight_database_3D` | LiDAR recording and 3D conversion nodes |
| `src/photon_flight_servo_imu` | Servo GPIO, IMU, and GPS ROS 2 nodes |
| `src/photon_flight_camera` | Camera trigger and image capture node |
| `src/photon_flight_internet` | Internet connectivity publisher |
| `src/photon_flight_pix` | PX4/MAVROS utility node |
| `src/rplidar_ros` | RPLiDAR ROS 2 driver |

## Requirements

- Ubuntu with ROS 2 (the project uses ROS 2 C++ APIs and Python `rclpy`)
- A supported RPLiDAR and RPLiDAR ROS 2 driver
- The `nmea_navsat_driver` package for serial NMEA GPS input
- An MPU-style IMU exposed through I2C
- GPS data published through ROS 2
- Python dependencies:
  - `numpy`
  - `laspy`
  - optional LAZ support through `laspy[lazrs]`
  - `psycopg2` only for database functionality
- Hardware packages such as OpenCV and `lgpio` for the nodes that use them

Install the Python package dependencies with:

```bash
sudo rosdep init       # only needed if rosdep has not been initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install numpy 'laspy[lazrs]' psycopg2-binary
```

## Build

From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

For a clean rebuild:

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## Run the flight capture and conversion

The exact device serial port and topics may differ by hardware configuration.

1. Source the workspace and start the RPLiDAR driver:

```bash
source install/setup.bash
ros2 run rplidar_ros rplidar_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser \
  -p angle_compensate:=true
```

2. In another terminal, start the serial NMEA GPS driver:

```bash
source install/setup.bash
ros2 run nmea_navsat_driver nmea_serial_driver --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baud:=9600
```

Then convert its `sensor_msgs/NavSatFix` output into `gps/readings` with the project GPS listener:

```bash
source install/setup.bash
ros2 run photon_flight_servo_imu gps_listener
```

3. In another terminal, start the IMU publisher:

```bash
source install/setup.bash
ros2 run photon_flight_servo_imu imu_publisher_node
```

5. Start the flight data recorder:

```bash
ros2 run photon_flight_database_3D lidar_subscriber
```

6. Stop the recorder with `Ctrl+C`. It saves the latest valid flight data as a numbered file:

```text
2D_Flight_Matrix_0.npy
2D_Flight_Matrix_1.npy
2D_Flight_Matrix_2.npy
```

7. Generate a LAS point cloud. The conversion node automatically loads the matrix with the highest numbered flight file in the current directory:

```bash
ros2 run photon_flight_database_3D Conversion_node
```

The resulting cloud is written to:

```text
3D_Flight_PointCloud.las
```

## Run individual nodes

```bash
# Camera
ros2 run photon_flight_camera camera_subscriber

# Internet connectivity
ros2 run photon_flight_internet internet_publisher_node

# Servo
ros2 run photon_flight_servo_imu servo_driver

# IMU and GPS
ros2 run photon_flight_servo_imu imu_publisher_node
ros2 run photon_flight_servo_imu gps_listener

# RPLiDAR client utilities
ros2 run rplidar_ros rplidar_client
```

## Useful ROS 2 commands

```bash
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /imu/data
ros2 topic echo /gps/readings
ros2 topic hz /scan
```

## Development status

This repository is an active flight-system prototype. Hardware-specific paths, package metadata, topic names, and database configuration may require changes for a new device or ROS 2 installation.

Known areas under development include:

- Completing database synchronization and query logic
- Launching the workspace consistently from one bringup package
- Adding automated tests for sensor synchronization and matrix validation
- Improving GPS/IMU calibration and timing synchronization
- Better handling of low-quality or missing sensor samples

Do not fly or operate hardware based only on this software until sensor calibration, fail-safe behavior, and flight safety have been verified by qualified personnel.

## Contributing

1. Create a feature branch from the current default branch.
2. Keep hardware-specific configuration isolated and documented.
3. Preserve sensor units and column indexes in flight matrices.
4. Run a workspace build before opening a pull request.
5. Document any ROS 2 topic or launch-command changes.

## License

Package license files are incomplete or marked TODO in portions of this repository. Confirm the desired project license before distributing or publishing derivative work.

## Acknowledgments

- ROS 2 and the RPLiDAR ROS package ecosystem
- NumPy and LAS/`laspy` tooling
- The Photon Flight Team contributors and collaborators
