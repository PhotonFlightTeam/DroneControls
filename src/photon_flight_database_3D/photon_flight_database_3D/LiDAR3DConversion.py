import rclpy
from rclpy.node import Node
import numpy as np
import glob 
import re

from lidar_to_pointcloud import rotation_to_z, ema

#credit and learning https://electronic-python.readthedocs.io/en/latest/vpython/yaw_pitch_roll.html

class ConversionNode(Node): 

    COLUMN_T, COLUMN_ANGLE, COLUMN_RANGE = 0, 1, 2
    COLUMN_ACCEL, COLUMN_GYRO, COLUMN_GPS = slice(3,6), slice(6,9), slice(9,12)
    #offset inccorect reading
    BIAS_SECOND = 3.0

    def __init__(self): 
        super.__init__('Conversion_node')
          
        self.loaded_matrix = None
 
 def latest_flight_matrix(directory="."):
    pattern = re.compile(r"2D_Flight_Matrix_(\d+)\.npy$")
    best_num, best_path = -1, None

    for path in glob.glob(f"{directory}/2D_Flight_Matrix_*.npy"):
        match = pattern.search(path)
        if match and int(match.group(1)) > best_num:
            best_num, best_path = int(match.group(1)), path

    return best_path

def load_matrix(self, best_path):
    self.loaded_matrix = np.load(best_path)

def calculate_pitch_roll_yaw(self):
    matrix = self.load_matrix

    # flatnonzero flattens the angle column
    # np.dff subtracts angle from later angle, gives change in angle from row to row
    # gives us true scan boundaries
    starts = np.flatnonzero(np.r_[True, np.diff(m[:, self.COL_ANGLE]) < 0])
    # this builds an array of scans times in seconds since first scan
    t = m[starts, self.COLUMN_T] - m[starts[0], self.COLUMN_T]
    accel = m[starts][:, self.COLUMN_ACCEL]
    gyro = m[starts][:, self.COL_GYRO]

    
    n_ref = max(int(np.sum(t < self.BIAS_SECONDS)), 3)
    R_align = rotation_to_z(accel[:n_ref].mean(axis=0))
    bias = gyro[:n_ref].mean(axis=0)

    #smooth acceleration readings
    #ema is exponential moving average, smoothing filter
    a_b = ema(accel, t, 0.2)
    # correct bias for gyro 
    w_b = (gyro - bias)  

    roll = np.arctan2(a_b[:, 1], a_b[:, 2])
    pitch = np.arctan2(-a_b[:, 0], np.hypot(a_b[:, 1], a_b[:, 2]))


  #  roll  = np.arctan2(a_b[:, 1], a_b[:, 2])
  # pitch = np.arctan2(-a_b[:, 0], np.hypot(a_b[:, 1], a_b[:, 2]))