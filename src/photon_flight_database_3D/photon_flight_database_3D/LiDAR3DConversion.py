import rclpy
from rclpy.node import Node
import numpy as np
import glob
import re

from photon_flight_database_3D.gps_utils import gps_offsets_per_scan

# credit and learning https://electronic-python.readthedocs.io/en/latest/vpython/yaw_pitch_roll.html


def rotation_to_z(g):
    """Smallest rotation taking vector g onto +z (Rodrigues)."""
    magnitude = np.linalg.norm(g)
    if not np.isfinite(magnitude) or magnitude == 0:
        raise RuntimeError('Cannot align an invalid accelerometer reference')
    g = g / magnitude
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(g, z)
    c = float(g @ z)
    if c < -0.999999:
        return np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1.0 + c)


def ema(x, t, tau):
    """Time-aware exponential moving average along axis 0."""
    if tau <= 0:
        return x
    out = np.empty_like(x, dtype=float)
    out[0] = x[0]
    for k in range(1, len(x)):
        a = 1.0 - np.exp(-(t[k] - t[k - 1]) / tau)
        out[k] = out[k - 1] + a * (x[k] - out[k - 1])
    return out


class ConversionNode(Node):

    COLUMN_T, COLUMN_ANGLE, COLUMN_RANGE = 8, 0, 1
    COLUMN_ACCEL, COLUMN_GYRO, COLUMN_GPS = slice(2, 5), slice(5, 8), slice(9, 13)
    BIAS_SECONDS = 3.0
    MAX_GYRO_INTEGRATION_GAP = 0.5
    GRAVITY = 9.80665
    ACCEL_GRAVITY_TOLERANCE = 0.2
    ATTITUDE_CORRECTION_TAU = 2.0
    STATIONARY_GYRO_THRESHOLD = 0.1

    def __init__(self):
        super().__init__('Conversion_node')
        self.loaded_matrix = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.timestamp_gaps = []
        self.scan_coverage_degrees = None
        self.gyro_bias_sample_count = 0
        self.gyro_bias_fallback = False

    def latest_flight_matrix(self, directory='.'):
        pattern = re.compile(r"2D_Flight_Matrix_(\d+)\.npy$")
        best_num, best_path = -1, None

        for path in glob.glob(f"{directory}/2D_Flight_Matrix_*.npy"):
            match = pattern.search(path)
            if match and int(match.group(1)) > best_num:
                best_num, best_path = int(match.group(1)), path

        return best_path

    def load_matrix(self, best_path=None):
        if best_path is None:
            best_path = self.latest_flight_matrix()
        if best_path is None:
            raise FileNotFoundError('No 2D_Flight_Matrix_*.npy file found')
        self.loaded_matrix = np.load(best_path)

    def calculate_pitch_roll_yaw(self):
        if self.loaded_matrix is None:
            raise RuntimeError('Load a flight matrix before calculating orientation')

        m = self.loaded_matrix

        # A negative change marks a new LiDAR scan boundary.
        starts = np.flatnonzero(np.r_[True, np.diff(m[:, self.COLUMN_ANGLE]) < 0])
        ends = np.r_[starts[1:], len(m)]
        self.scan_coverage_degrees = np.array([
            np.degrees(np.ptp(m[start:end, self.COLUMN_ANGLE]))
            for start, end in zip(starts, ends)
        ])
        scan_indices = np.arange(len(starts))
        scan_times = m[starts, self.COLUMN_T]
        valid_times = np.isfinite(scan_times)
        if not valid_times.any():
            raise RuntimeError(
                'The flight matrix contains no valid IMU timestamps')
        t = np.interp(
            scan_indices, scan_indices[valid_times], scan_times[valid_times])
        if np.any(np.diff(t) < 0):
            raise RuntimeError('IMU timestamps go backwards between scans')
        t -= t[0]

        accel = m[starts][:, self.COLUMN_ACCEL]
        gyro = m[starts][:, self.COLUMN_GYRO]
        for readings, sensor_name in (
                (accel, 'accelerometer'), (gyro, 'gyroscope')):
            for axis in range(readings.shape[1]):
                valid_readings = np.isfinite(readings[:, axis])
                if not valid_readings.any():
                    raise RuntimeError(
                        'The flight matrix contains no valid '
                        f'{sensor_name} readings')
                readings[:, axis] = np.interp(
                    scan_indices, scan_indices[valid_readings],
                    readings[valid_readings, axis])

        n_ref = max(int(np.sum(t < self.BIAS_SECONDS)), 3)
        alignment = rotation_to_z(accel[:n_ref].mean(axis=0))
        accel = accel @ alignment.T
        gyro = gyro @ alignment.T
        stationary_candidates = (
            (np.abs(np.linalg.norm(accel, axis=1) - self.GRAVITY)
             <= self.ACCEL_GRAVITY_TOLERANCE * self.GRAVITY)
            & (np.linalg.norm(gyro, axis=1) <= self.STATIONARY_GYRO_THRESHOLD)
        )
        self.gyro_bias_sample_count = int(
            np.count_nonzero(stationary_candidates))
        self.gyro_bias_fallback = self.gyro_bias_sample_count < 3
        if self.gyro_bias_fallback:
            bias = np.median(gyro[:n_ref], axis=0)
        else:
            bias = np.median(gyro[stationary_candidates], axis=0)
        a_b = ema(accel, t, 0.2)
        w_b = gyro - bias
        roll = np.zeros(len(starts))
        pitch = np.zeros(len(starts))
        yaw = np.zeros(len(starts))
        self.timestamp_gaps = []

        for index in range(1, len(starts)):
            dt = t[index] - t[index - 1]
            if dt > self.MAX_GYRO_INTEGRATION_GAP:
                self.timestamp_gaps.append((index, dt))
                roll[index] = roll[index - 1]
                pitch[index] = pitch[index - 1]
                yaw[index] = yaw[index - 1]
                continue

            sin_roll = np.sin(roll[index - 1])
            cos_roll = np.cos(roll[index - 1])
            cos_pitch = np.cos(pitch[index - 1])
            safe_cos_pitch = np.copysign(
                max(abs(cos_pitch), 1e-6), cos_pitch)
            tan_pitch = np.sin(pitch[index - 1]) / safe_cos_pitch
            gyro_x, gyro_y, gyro_z = (
                w_b[index - 1] + w_b[index]) / 2.0

            roll_rate = (
                gyro_x + sin_roll * tan_pitch * gyro_y
                + cos_roll * tan_pitch * gyro_z)
            pitch_rate = cos_roll * gyro_y - sin_roll * gyro_z
            yaw_rate = (
                sin_roll / safe_cos_pitch * gyro_y
                + cos_roll / safe_cos_pitch * gyro_z)
            roll_predicted = roll[index - 1] + roll_rate * dt
            pitch_predicted = pitch[index - 1] + pitch_rate * dt
            yaw[index] = yaw[index - 1] + yaw_rate * dt

            accel_magnitude = np.linalg.norm(a_b[index])
            if abs(accel_magnitude - self.GRAVITY) <= (
                    self.ACCEL_GRAVITY_TOLERANCE * self.GRAVITY):
                measured_roll = np.arctan2(a_b[index, 1], a_b[index, 2])
                measured_pitch = np.arctan2(
                    -a_b[index, 0],
                    np.hypot(a_b[index, 1], a_b[index, 2]))
                correction = 1.0 - np.exp(
                    -dt / self.ATTITUDE_CORRECTION_TAU)
                roll_error = np.arctan2(
                    np.sin(measured_roll - roll_predicted),
                    np.cos(measured_roll - roll_predicted))
                pitch_error = np.arctan2(
                    np.sin(measured_pitch - pitch_predicted),
                    np.cos(measured_pitch - pitch_predicted))
                roll[index] = roll_predicted + correction * roll_error
                pitch[index] = pitch_predicted + correction * pitch_error
            else:
                roll[index] = roll_predicted
                pitch[index] = pitch_predicted

        self.roll, self.pitch, self.yaw = roll, pitch, yaw
        return roll, pitch, yaw

    def save_las(self, output_path='3D_Flight_PointCloud.las'):
        try:
            import laspy
        except ImportError as error:
            raise RuntimeError(
                'LAS export requires laspy. Install it with: '
                'python3 -m pip install laspy[lazrs]'
            ) from error

        if self.loaded_matrix is None:
            raise RuntimeError('Load a flight matrix before creating a LAS file')
        if self.roll is None or self.pitch is None or self.yaw is None:
            raise RuntimeError('Calculate flight orientation before creating a LAS file')

        m = self.loaded_matrix
        starts = np.flatnonzero(
            np.r_[True, np.diff(m[:, self.COLUMN_ANGLE]) < 0])
        scan_index = np.searchsorted(starts, np.arange(len(m)), side='right') - 1

        roll = np.asarray(self.roll)
        pitch = np.asarray(self.pitch)
        yaw = np.asarray(self.yaw)
        if len(roll) != len(starts) or len(pitch) != len(starts) or len(yaw) != len(starts):
            raise RuntimeError('Orientation data does not match the detected scans')

        angle = m[:, self.COLUMN_ANGLE]
        distance = m[:, self.COLUMN_RANGE]
        valid = np.isfinite(angle) & np.isfinite(distance) & (distance >= 0)
        valid_indices = np.flatnonzero(valid)
        if not len(valid_indices):
            raise RuntimeError('The flight matrix contains no valid LiDAR points')

        point_scan = scan_index[valid_indices]
        cr = np.cos(roll[point_scan])
        sr = np.sin(roll[point_scan])
        cp = np.cos(pitch[point_scan])
        sp = np.sin(pitch[point_scan])
        cy = np.cos(yaw[point_scan])
        sy = np.sin(yaw[point_scan])

        point_angle = angle[valid_indices]
        point_distance = distance[valid_indices]
        x_body = point_distance * np.cos(point_angle)
        y_body = point_distance * np.sin(point_angle)
        z_body = np.zeros(len(valid_indices))

        # Apply roll, then pitch, then yaw around the drone's 3D pose.
        y1 = cr * y_body - sr * z_body
        z1 = sr * y_body + cr * z_body
        x1 = cp * x_body + sp * z1
        z2 = -sp * x_body + cp * z1

        x = cy * x1 - sy * y1
        y = sy * x1 + cy * y1
        z = z2

        gps_offsets = gps_offsets_per_scan(m, starts)
        x += gps_offsets[point_scan, 0]
        y += gps_offsets[point_scan, 1]
        z += gps_offsets[point_scan, 2]

        if not all(
                np.isfinite(coordinates).all()
                for coordinates in (x, y, z)):
            raise RuntimeError(
                'Point coordinates contain non-finite values; refusing to '
                'write an invalid LAS file')

        header = laspy.LasHeader(point_format=3, version='1.3')
        header.offset = [np.floor(minimum) for minimum in (x.min(), y.min(), z.min())]
        header.scale = [0.001, 0.001, 0.001]
        cloud = laspy.LasData(header)
        cloud.x = x
        cloud.y = y
        cloud.z = z
        cloud.write(output_path)
        return output_path, len(cloud.points)


def main(args=None):
    rclpy.init(args=args)
    node = ConversionNode()
    node.load_matrix()
    roll, pitch, yaw = node.calculate_pitch_roll_yaw()
    if node.timestamp_gaps:
        largest_gap = max(gap for _, gap in node.timestamp_gaps)
        node.get_logger().warning(
            f"Skipped gyro integration across {len(node.timestamp_gaps)} "
            f"timestamp gap(s); largest gap was {largest_gap:.2f}s. "
            "Orientation across those gaps is unknown.")
    if node.gyro_bias_fallback:
        node.get_logger().warning(
            "No low-motion IMU samples were found for gyro-bias estimation; "
            "using the initial readings as a fallback.")
    else:
        node.get_logger().info(
            f"Estimated gyro bias from "
            f"{node.gyro_bias_sample_count} low-motion samples.")
    yaw_span = np.degrees(np.ptp(yaw))
    if yaw_span > 360.0:
        node.get_logger().warning(
            f"Gyro-only yaw spans {yaw_span:.1f} degrees. Without an absolute "
            "heading reference, yaw drift or repeated turns cannot be "
            "distinguished.")
    if node.scan_coverage_degrees is not None:
        incomplete = np.count_nonzero(node.scan_coverage_degrees < 340.0)
        if incomplete:
            node.get_logger().warning(
                f"{incomplete} of {len(node.scan_coverage_degrees)} scans "
                "cover less than 340 degrees; these scans were retained.")
    try:
        output_path, point_count = node.save_las()
        node.get_logger().info(
            f"Saved {point_count} points to {output_path}")
    except RuntimeError as error:
        node.get_logger().error(str(error))
    node.get_logger().info(
        f"{len(roll)} scans, final yaw {np.degrees(yaw[-1]):.1f}°")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()