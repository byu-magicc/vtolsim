"""
msgState 
    - messages type for state, that will be passed between blocks in the architecture
    
part of mavPySim 
    - Beard & McLain, PUP, 2012
    - Update history:  
        1/9/2019 - RWB
        3/30/2022 - RWB
"""
import numpy as np


class MsgState:
    '''
    Message class that defines the state of the aircraft

    Attributes
    ----------
    north : float
        inertial north position in meters
    east : float
        inertial east position in meters
    altitude : float
        inertial altitude in meters
    phi : float
        roll angle in radians
    theta : float
        pitch angle in radians
    psi : float
        yaw angle in radians
    Va : float
        airspeed in meters/sec
    alpha : float
        angle of attack in radians
    beta : float
        sideslip angle in radians
    p : float
        roll rate in radians/sec
    q : float
        pitch rate in radians/sec
    r : float
        yaw rate in radians/sec
    Vg : float
        groundspeed in meters/sec
    gamma : float
        flight path angle in radians
    chi : float
        course angle in radians
    wn : float
        inertial windspeed in north direction in meters/sec
    we : float
        inertial windspeed in east direction in meters/sec
    bx : float
        gyro bias along roll axis in radians/sec
    by : float
        gyro bias along pitch axis in radians/sec
    bz : float
        gyro bias along yaw axis in radians/sec
    gimbal_az : float
        gimbal azimuth angle
    gimbal_el : float
        gimbal elevation angle
    
    Methods
    -------
    __add__(self, other)
        Overload the addition '+' operator
    __sub__(self, other):
        Overload the subtraction '-' operator
    __rmul__(self, other: float):
        Overload right multiply by a scalar
    '''    
    def __init__(self):
        #positional portion of the true state
        self.north = float(0.) 
        self.east = float(0.)      
        self.altitude = float(100.)
        # body frame velocity of the plane
        self.u = float(0.)
        self.v = float(0.)
        self.w = float(0.)
        #roll, pitch, and yaw
        self.phi = float(0.)     
        self.theta = float(0.)
        self.psi = float(0.)
        self.Va = float(25.)
        self.alpha = float(0.)
        self.beta = float(0.)
        #omega
        self.p = float(0.)
        self.q = float(0.)
        self.r = float(0.)
        self.Vg = float(25.)
        self.gamma = float(0.)
        self.chi = float(0.)
        self.wn = float(0.)
        self.we = float(0.)
        self.bx = float(0.)
        self.by = float(0.)
        self.bz = float(0.)
        self.gimbal_az = float(0.)
        self.gimbal_el = float(np.radians(-90))

    def __add__(self, other):
        '''Overload the addition '+' operator'''
        out = MsgState()
        out.north = self.north + other.north
        out.east = self.east + other.east
        out.altitude = self.altitude + other.altitude
        out.phi = self.phi + other.phi
        out.theta = self.theta + other.theta
        out.psi = self.psi + other.psi
        out.Va = self.Va + other.Va
        out.alpha = self.alpha + other.alpha
        out.beta = self.beta + other.beta
        out.p = self.p + other.p
        out.q = self.q + other.q
        out.r = self.r + other.r
        out.Vg = self.Vg + other.Vg
        out.gamma = self.gamma + other.gamma
        out.chi = self.chi + other.chi
        out.wn = self.wn + other.wn
        out.we = self.we + other.we
        out.bx = self.bx + other.bx
        out.by = self.by + other.by
        out.bz = self.bz + other.bz
        out.gimbal_az = self.gimbal_az + other.gimbal_az
        out.gimbal_el = self.gimbal_el + other.gimbal_el
        return out

    def __sub__(self, other):
        '''Overload the subtraction '-' operator'''
        out = MsgState()
        out.north = self.north - other.north
        out.east = self.east - other.east
        out.altitude = self.altitude - other.altitude
        out.phi = self.phi - other.phi
        out.theta = self.theta - other.theta
        out.psi = self.psi - other.psi
        out.Va = self.Va - other.Va
        out.alpha = self.alpha - other.alpha
        out.beta = self.beta - other.beta
        out.p = self.p - other.p
        out.q = self.q - other.q
        out.r = self.r - other.r
        out.Vg = self.Vg - other.Vg
        out.gamma = self.gamma - other.gamma
        out.chi = self.chi - other.chi
        out.wn = self.wn - other.wn
        out.we = self.we - other.we
        out.bx = self.bx - other.bx
        out.by = self.by - other.by
        out.bz = self.bz - other.bz
        out.gimbal_az = self.gimbal_az - other.gimbal_az
        out.gimbal_el = self.gimbal_el - other.gimbal_el
        return out

    def __rmul__(self, other: float):
        '''Overload right multiply by a scalar'''
        out = MsgState()
        out.north = other * self.north
        out.east = other * self.east
        out.altitude = other * self.altitude
        out.phi = other * self.phi
        out.theta = other * self.theta
        out.psi = other * self.psi
        out.Va = other * self.Va
        out.alpha = other * self.alpha
        out.beta = other * self.beta
        out.p = other * self.p
        out.q = other * self.q
        out.r = other * self.r
        out.Vg = other * self.Vg
        out.gamma = other * self.gamma
        out.chi = other * self.chi
        out.wn = other * self.wn
        out.we = other * self.we
        out.bx = other * self.bx
        out.by = other * self.by
        out.bz = other * self.bz
        out.gimbal_az = other * self.gimbal_az
        out.gimbal_el = other * self.gimbal_el
        return out
