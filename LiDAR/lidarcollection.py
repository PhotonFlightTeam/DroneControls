from os import path
from datetime import datetime
from rplidar import RPLidar
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from sys import exit

BAUD_RATE: int = 115200
TIMEOUT: int = 1
DEVICE_PATH: str =  '/dev/ttyUSB0'


def check_connection() -> bool: 
    if path.exists(DEVICE_PATH): 
        print(f"Found RPLidar on path: {DEVICE_PATH}")
        return True
    else: 
        print(f"RPLidar unit not found, try checking the device path or the connection") 
        print(f"ls /dev/tty* in terminal to check all talk ports")
        return False 

#TODO: Update line function


if __name__ == "__main__": 

    if check_connection() == True:
         #instantiate lidar instance with baud, port its connected and the timeout
         lidar = RPLidar(port=DEVICE_PATH, baudrate=BAUD_RATE, timeout=TIMEOUT)
         # Check status of lidar unit
         if lidar.get_health() == 'Good':  

            lidar.start_motor()
            lidar.start_motor()

#TODO: Iterate over scans, put in numpy array 
#Find a way to save lidar scans 
#Plot and create figure that scans will appear over 


    else:
        exit(1)