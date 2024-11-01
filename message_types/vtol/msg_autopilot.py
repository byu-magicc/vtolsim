"""
msg_autopilot
    - messages type for input to the autopilot

part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        2/5/2019 - RWB
"""


class MsgAutopilot:
    def __init__(self):
        self.airspeed = 0.0  # commanded airspeed m/s
        self.course = 0.0  # commanded course angle in rad
        self.altitude = 0.0  # commanded altitude in m
        self.phi_feedforward = 0.0  # feedforward command for roll angle

        #commands for hover controller
        self.pn = 0.0
        self.pe = 0.0
        self.heading = 0.0
        self.vn = 0.0
        self.roll = 0.0
        self.pitch = 0.0
