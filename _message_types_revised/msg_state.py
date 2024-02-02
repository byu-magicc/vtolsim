"""
msgState 
    - messages type for state, that will be passed between blocks in the architecture
    - Update history:  
        5/3/2021 - RWB
        6/8/2021 - RWB
        4/20/2022 - RWB
        11/16/2023 - RWB
"""
import numpy as np
from tools.rotations import rotation_to_euler

class MsgState:
    def __init__(self):
        self.pos = np.array([[0.], [0.], [0.]])  # inertial NED position in meters
        self.vel = np.array([[0.], [0.], [0.]])  # inertial velocity in body frame in meters
        self.R = np.identity(3)  # rotation matrix, inertial to body
        self.omega = np.array([[0.], [0.], [0.]])  # angular velocity in body frame in rad/sec
        self.gyro_bias = np.array([[0.], [0.], [0.]])  # gyro bias in rad/sec
        self.motor_angle = np.array([[0.], [0.]])  # right/left angles of motors
        self.Va = 0.  # airspeed
        self.alpha = 0.  # angle of attach
        self.beta = 0.  # sideslip angle

    def add_to_position(self, n=0, e=0, d=0):
        self.pos = self.pos + np.array([[n], [e], [d]])

    def euler_angles(self):
        phi, theta, psi = rotation_to_euler(self.R)
        return phi, theta, psi
