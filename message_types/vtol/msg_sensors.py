"""
msg_sensors
    - messages type for output of sensors
    
part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        2/16/2019 - RWB
        3/12/2024 - RWB
"""
import numpy as np


class MsgSensors:
    '''
        Message class for the sensors to carry measurement data
        
        Attributes:
            gyro_x: gyroscope measurement along body x axis
            gyro_y: gyroscope measurement along body y axis
            gyro_z: gyroscope measurement along body z axis
            accel_x: accelerometer measurement along body x axis
            accel_y: accelerometer measurement along body y axis
            accel_z: accelerometer measurement along body z axis
            mag_x: magnetometer measurement along body x axis
            mag_y: magnetometer measurement along body y axis
            mag_z: magnetometer measurement along body z axis
            static_pressure: static pressure
            diff_pressure: differential pressure
            gps_n: gps north
            gps_e: gps east
            gps_h: gps altitude
            gps_Vg: gps ground speed
            gps_course: gps course angle            
    '''
    def __init__(self):
        self.gyro_x = float(0)
        self.gyro_y = float(0)
        self.gyro_z = float(0)
        self.accel_x = float(0)
        self.accel_y = float(0)
        self.accel_z = float(0)
        self.mag_x = float(0)
        self.mag_y = float(0)
        self.mag_z = float(0)
        self.static_pressure = float(0)
        self.diff_pressure = float(0)
        self.gps_n = float(0)
        self.gps_e = float(0)
        self.gps_h = float(0)
        self.gps_Vg = float(0)
        self.gps_course = np.radians(0)