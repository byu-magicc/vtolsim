"""
msg_sensors
    - messages type for output of sensors
    
part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        2/16/2019 - RWB
"""


class MsgSensors:
    '''Message type for sensor measurements'''
    def __init__(self):
        self.gyro_x = float(0)  # gyroscope along body x axis
        self.gyro_y = float(0)  # gyroscope along body y axis
        self.gyro_z = float(0)  # gyroscope along body z axis
        self.accel_x = float(0)  # specific acceleration along body x axis
        self.accel_y = float(0)  # specific acceleration along body y axis
        self.accel_z = float(0)  # specific acceleration along body z axis
        self.mag_x = float(0)  # magnetic field along body x axis
        self.mag_y = float(0)  # magnetic field along body y axis
        self.mag_z = float(0)  # magnetic field along body z axis
        self.abs_pressure = float(0)  # absolute pressure
        self.diff_pressure = float(0)  # differential pressure
        self.gps_n = float(0)  # gps north
        self.gps_e = float(0)  # gps east
        self.gps_h = float(0)  # gps altitude
        self.gps_Vg = float(0)  # gps ground speed
        self.gps_course = float(0)  # gps course angle