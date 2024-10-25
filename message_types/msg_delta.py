"""
msg_delta
    - messages type for input to the aircraft
    
part of mavsim
    - Beard & McLain, PUP, 2012
    - Last update:
        2/27/2020 - RWB
        4/6/2022 - RWB
"""
import numpy as np


class MsgDelta:
    '''
        Message class for control commands to the aircraft
    '''
    def __init__(self,
                 elevator=0.0,
                 aileron=0.0,
                 rudder=0.0,
                 forwardThrottle=0.5,
                 verticalThrottle_1=0.5,
                 verticalThrottle_2=0.5,
                 verticalThrottle_3=0.5,
                 verticalThrottle_4=0.5,
                 azimuth_cmd=0.0,
                 elevation_cmd=0.0):
        self.elevator = float(elevator)  # elevator command
        self.aileron = float(aileron)  # aileron command
        self.rudder = float(rudder)  # rudder command
        self.forwardThrottle = float(forwardThrottle)  # forward throttle command
        self.verticalThrottle_1 = float(verticalThrottle_1) #vertical throttle 1 command
        self.verticalThrottle_2 = float(verticalThrottle_2) #vertical throttle 2 command
        self.verticalThrottle_3 = float(verticalThrottle_3) #vertical throttle 3 command
        self.verticalThrottle_4 = float(verticalThrottle_4) #vertical throttle 4 command
        self.gimbal_az = float(azimuth_cmd)  # azimuth command for gimbal
        self.gimbal_el = float(elevation_cmd)  # elevation command for gimbal

    def to_array(self)->np.ndarray:
        return np.array([[self.elevator],
                         [self.aileron],
                         [self.rudder],
                         [self.forwardThrottle],
                         [self.verticalThrottle_1],
                         [self.verticalThrottle_2],
                         [self.verticalThrottle_3],
                         [self.verticalThrottle_4],
                         [self.gimbal_az],
                         [self.gimbal_el],
                         ])

    def from_array(self, u:np.ndarray):
        self.elevator = u.item(0)
        self.aileron = u.item(1)
        self.rudder = u.item(2)
        self.forwardThrottle = u.item(3)
        self.verticalThrottle_1 = u.item(4)
        self.verticalThrottle_2 = u.item(5)
        self.verticalThrottle_3 = u.item(6)
        self.verticalThrottle_4 = u.item(7)
        self.gimbal_az = u.item(8)
        self.gimbal_el = u.item(9)

    def print(self):
        print('elevator=', self.elevator,
              'aileron=', self.aileron,
              'rudder=', self.rudder,
              'forward Throttle=', self.forwardThrottle,
              'vertical Throttle 1=', self.verticalThrottle_1,
              'vertical Throttle 2=', self.verticalThrottle_2,
              'vertical Throttle 3=', self.verticalThrottle_3,
              'vertical Throttle 4=', self.verticalThrottle_4,
              'azimuth_cmd=', self.gimbal_az,
              'elevation_cmd=', self.gimbal_el)


