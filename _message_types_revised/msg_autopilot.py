"""
- messages type for input to the autopilot
- Update history:
    2/5/2019 - R.W. Beard
    11/27/2023 - R.W. Beard
"""

import numpy as np

class MsgAutopilot:
    def __init__(self):
        self.airspeed = 0.0  # commanded airspeed m/s
        self.course = 0.0  # commanded course angle in rad
        self.altitude = 0.0  # commanded altitude in m
        self.phi_feedforward = 0.0  # feedforward command for roll angle

        #commands for hover controller
        self.roll = 0.0
        self.pitch = 0.0
        self.pn = 0.0
        self.pe = 0.0

        # Jake's hover control
        self.pos = np.zeros((3, 1))  # commanded position in m
        self.vel = np.zeros((3, 1))  # commanded velocity in m/s
        self.accel = np.zeros((3, 1))  # commanded acceleration in m/s/s
        self.heading = 0.0  # commanded heading in rad

    def __str__(self):
        return f'airspeed_command: {self.airspeed}\n' + \
            f'course_command: {self.course}\n' + \
            f'altitude_command: {self.altitude}\n' + \
            f'phi_feedforward: {self.phi_feedforward}\n' + \
            f'roll_command: {self.roll}\n' + \
            f'pitch_command: {self.pitch}\n' + \
            f'pn_command: {self.pn}\n' + \
            f'pe_command: {self.pe}\n' + \
            f'pos: {self.pos.T}.T\n' + \
            f'vel: {self.vel.T}.T\n' + \
            f'accel: {self.accel.T}.T\n' + \
            f'heading: {self.heading}'