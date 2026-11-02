import rclpy
from rclpy.node import Node
import numpy as np
import glob
import re

# credit and learning https://electronic-python.readthedocs.io/en/latest/vpython/yaw_pitch_roll.html


def rotation_to_z(g):
    """Smallest rotation taking vector g onto +z (Rodrigues)."""
    g = g / np.linalg.norm(g)
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
    COLUMN_ACCEL, COLUMN_GYRO, COLUMN_GPS = slice(2, 5), slice(5, 8), slice(9, 12)
    BIAS_SECONDS = 3.0

    def __init__(self):
        super().__init__('Conversion_node')
        self.loaded_matrix = None
        self.roll = None
        self.pitch = None
        self.yaw = None

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
        t = m[starts, self.COLUMN_T] - m[starts[0], self.COLUMN_T]
        accel = m[starts][:, self.COLUMN_ACCEL]
        gyro = m[starts][:, self.COLUMN_GYRO]

        n_ref = max(int(np.sum(t < self.BIAS_SECONDS)), 3)
        rotation_to_z(accel[:n_ref].mean(axis=0))
        bias = gyro[:n_ref].mean(axis=0)

        # Smooth acceleration readings and correct gyro bias.
        a_b = ema(accel, t, 0.2)
        w_b = gyro - bias

        roll = np.arctan2(a_b[:, 1], a_b[:, 2])
        pitch = np.arctan2(-a_b[:, 0], np.hypot(a_b[:, 1], a_b[:, 2]))
        yaw = np.concatenate((
            [0.0],
            np.cumsum(0.5 * (w_b[1:, 2] + w_b[:-1, 2]) * np.diff(t))))

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