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

`/gps/readings` and `/imu/data` come from either the custom NMEA GPS and I2C IMU nodes or from a Cube+ through MAVROS and `pix_node`. See [Choose the GPS and IMU source](#choose-the-gps-and-imu-source).

## Flight matrix format

Each flight matrix is stored as one rectangular NumPy 2D array after valid LiDAR scan blocks are vertically stacked.

The current expected layout is **13 columns**:

| Columns | Data |
|---|---|
| 0 | LiDAR angle |
| 1 | LiDAR range |
| 2–4 | Linear acceleration (x, y, z) |
| 5–7 | Angular velocity (x, y, z) |
| 8 | Gyroscope/IMU timestamp |
| 9–11 | GPS latitude, longitude, altitude |
| 12 | GPS timestamp |

Matrix shape:

```text
(total LiDAR samples, 13)
```

The expected column count is calculated from the IMU and GPS arrays plus the two LiDAR columns. Scans with missing or inconsistent sensor data are discarded during shutdown saving.

During LAS conversion, GPS latitude and longitude are interpolated to each
scan's IMU time and converted to local east/north offsets in meters. GPS
altitude is intentionally ignored because its variation can create large
vertical artifacts; vertical placement currently comes from the LiDAR/IMU
orientation only. The resulting cloud is relative to the first available GPS
fix, not georeferenced to an absolute coordinate system. If the flight has no
GPS fixes, conversion retains the LiDAR/IMU-only positions.

The converter aligns the initial mean accelerometer direction with +Z, then
estimates gyro bias from candidate samples with near-gravity acceleration and
low angular rate (falling back to the initial readings if no such samples
exist). It propagates orientation with the gyro and corrects tilt with the
accelerometer only when acceleration magnitude is near gravity. Gyro
integration is skipped across timestamp gaps over 0.5 seconds, which are
reported because orientation across those gaps is unknown. Yaw remains
gyro-only and can drift without an absolute heading reference. Scans with less
than 340 degrees of measured angular coverage are reported but retained in the
output.

## Repository layout

| Path | Description |
|---|---|
| `src/photon_flight_database_3D` | LiDAR recording and 3D conversion nodes |
| `src/photon_flight_servo_imu` | Servo GPIO, IMU, and GPS ROS 2 nodes |
| `src/photon_flight_camera` | Camera trigger and image capture node |
| `src/photon_flight_internet` | Internet connectivity publisher |
| `src/photon_flight_pix` | Cube+ (ArduPilot) GPS and IMU node using MAVROS |
| `src/*_bringup` | Standalone launch files (`launchdrone.py`, `launchdrone_pix.py`) |
| `src/rplidar_ros` | RPLiDAR ROS 2 driver |

## Requirements

- Ubuntu with ROS 2 (the project uses ROS 2 C++ APIs and Python `rclpy`)
- A supported RPLiDAR and RPLiDAR ROS 2 driver
- GPS and IMU data from one of two sources (see [Choose the GPS and IMU source](#choose-the-gps-and-imu-source)):
  - Custom: the `nmea_navsat_driver` package for serial NMEA GPS input and an MPU-style IMU exposed through I2C
  - MAVLink: a Cube+ running ArduPilot and the `mavros` package
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

1. Source the workspace and start all flight capture nodes together:

```bash
source install/setup.bash
ros2 launch photon_flight_database_3D flight_capture.launch.py
```

The launch file defaults to `/dev/ttyUSB0` for the RPLiDAR and `/dev/ttyACM0` for the GPS. Override either device or baud rate when needed:

```bash
ros2 launch photon_flight_database_3D flight_capture.launch.py \
  lidar_port:=/dev/ttyUSB1 \
  lidar_baudrate:=115200 \
  lidar_frame:=laser \
  gps_port:=/dev/ttyACM1 \
  gps_baudrate:=9600
```

It starts the RPLiDAR node, NMEA serial GPS driver, GPS-to-array converter, IMU publisher, and LiDAR flight recorder. The launch-file configuration is equivalent to:

```bash
ros2 run rplidar_ros rplidar_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser \
  -p angle_compensate:=true

ros2 run nmea_navsat_driver nmea_serial_driver --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baud:=9600

ros2 run photon_flight_servo_imu gps_listener
ros2 run photon_flight_servo_imu imu_publisher_node
ros2 run photon_flight_database_3D lidar_subscriber
```

2. Stop the recorder with `Ctrl+C`. It saves the latest valid flight data as a numbered file:

```text
2D_Flight_Matrix_0.npy
2D_Flight_Matrix_1.npy
2D_Flight_Matrix_2.npy
```

3. Generate a LAS point cloud. The conversion node automatically loads the matrix with the highest numbered flight file in the current directory:

```bash
ros2 run photon_flight_database_3D Conversion_node
```

The resulting cloud is written to:

```text
3D_Flight_PointCloud.las
```

## Choose the GPS and IMU source

GPS and accelerometer/gyroscope data can come from either of two sources. Both publish on the same topics (`/gps/readings` and `/imu/data`) in the same format, so the recorder and converter work with either. **Only run one source at a time**, because they publish to the same topics.

| Source | Nodes | Hardware |
|---|---|---|
| Custom (original) | `nmea_serial_driver`, `gps_listener`, `imu_publisher_node` | NMEA serial GPS and an MPU-style I2C IMU |
| MAVLink (Cube+) | `mavros_node`, `pix_node` | Cube+ running ArduPilot |

`src/*_bringup/launchdrone_pix.py` starts the LiDAR nodes plus whichever source you pick with the `use_pix` argument. It is a standalone launch file (not installed with a package), so pass its path to `ros2 launch`:

```bash
# Cube+ through MAVROS (default, use_pix:=true)
ros2 launch src/*_bringup/launchdrone_pix.py

# Original NMEA GPS and I2C IMU
ros2 launch src/*_bringup/launchdrone_pix.py use_pix:=false
```

| Argument | Default | Used when | Description |
|---|---|---|---|
| `use_pix` | `true` | always | `true` for the Cube+, `false` for the custom GPS and IMU |
| `fcu_url` | `/dev/ttyACM0:921600` | `use_pix:=true` | Cube+ connection for MAVROS, `device:baud` |
| `gps_port` | `/dev/ttyACM0` | `use_pix:=false` | NMEA GPS serial port |
| `gps_baudrate` | `9600` | `use_pix:=false` | NMEA GPS baud rate |
| `lidar_port` | `/dev/ttyUSB0` | always | RPLiDAR serial port |
| `lidar_baudrate` | `115200` | always | RPLiDAR baud rate |
| `lidar_frame` | `laser` | always | TF frame assigned to the RPLiDAR |

### Cube+ (MAVLink) source

`pix_node` (`src/photon_flight_pix/src/mav.cpp`, executable `mav`) subscribes to MAVROS and republishes in the custom nodes' format:

| MAVROS topic (input) | Published topic (output) | Format |
|---|---|---|
| `/mavros/global_position/global` | `gps/readings` | `Float64MultiArray` `{lat, lon, altitude, timestamp}`; nothing is published without a GPS fix |
| `/mavros/imu/data_raw` | `/imu/data` | `sensor_msgs/Imu` with linear acceleration (m/s²) and angular velocity (rad/s), frame `imu_link`, no orientation |

Notes:

- Timestamps come from MAVROS (the autopilot), not from the Pi's clock as in `imu_publisher_node`.
- Unlike `gps_listener`, `pix_node` publishes the GPS altitude as reported rather than a fixed altitude. LAS conversion ignores altitude, so this does not affect the point cloud.
- MAVROS is started through its own `apm.launch`, which loads the ArduPilot plugin list and configuration. To run it by hand:

```bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600
ros2 run photon_flight_pix mav
```

## Run individual nodes

```bash
# Camera
ros2 run photon_flight_camera camera_subscriber

# Internet connectivity
ros2 run photon_flight_internet internet_publisher_node

# Servo
ros2 run photon_flight_servo_imu servo_driver

# IMU and GPS (custom source)
ros2 run photon_flight_servo_imu imu_publisher_node
ros2 run photon_flight_servo_imu gps_listener

# IMU and GPS (Cube+ source, needs MAVROS running)
ros2 run photon_flight_pix mav

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
