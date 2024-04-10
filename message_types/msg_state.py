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
from tools.rotations import rotation_to_euler, euler_to_rotation
from message_types.msg_state_old import MsgState as MsgStateOld

class MsgState:
    '''
    Message class that defines the state of the aircraft

    Attributes
    ----------
    pos : np.ndarray (3x1)
        inertial NED position in meters
    vel : np.ndarray (3x1)
        inertial velocity in body frame in m/s
    R : np.ndarray (3x1)
        rotation matrix, body to inertial
    omega : np.ndarray (3x1)
        angular velocity in body frame in rad/sec
    gyro_bias : np.ndarray (3x1)
        gyro bias in rad/sec
    motor_angle : np.ndarray (2x1)
        right/left angles of motors in rad
    Va : float
        airspeed in m/s
    alpha : float
        angle of attach in rad
    beta : float
        sideslip angle in rad
    
    Methods
    -------
    add_to_position(n, e, d) :
        add (n,e,d) to the current position
    euler_angles() :
        returns the euler angles phi, theta, psi
    old_format() : 
        converts to MsgStateOld format    
   
    TO DO:  add these
    __add__(self, other)
        Overload the addition '+' operator
    __sub__(self, other):
        Overload the subtraction '-' operator
    __rmul__(self, other: float):
        Overload right multiply by a scalar
    '''
    def __init__(self, 
                 old=None, 
                 pos: np.ndarray=np.array([[0.], [0.], [0.]]), 
                 vel: np.ndarray=np.array([[0.], [0.], [0.]]), 
                 R: np.ndarray=np.identity(3), 
                 omega: np.ndarray=np.array([[0.], [0.], [0.]]), 
                 gyro_bias: np.ndarray=np.array([[0.], [0.], [0.]]), 
                 motor_angle: np.ndarray=np.array([[0.], [0.]]),  
                 Va: float=0.,  
                 alpha: float=0.,  
                 beta: float=0.,  
                 ):
        if old==None:
            self.pos = pos  
            self.vel = vel  
            self.R = R  
            self.omega = omega  
            self.gyro_bias = gyro_bias
            self.motor_angle = motor_angle
            self.Va = Va
            self.alpha = alpha
            self.beta = beta
        else:
            self.pos = np.array([[old.pn], [old.pe], [-old.h]])  
            self.vel = np.array([[old.u], [old.v], [old.w]]) 
            self.R = euler_to_rotation(old.phi, old.theta, old.psi)  
            self.omega = np.array([[old.p], [old.q], [old.r]])  
            self.gyro_bias = np.array([[old.bx], [old.by], [old.bz]])  
            self.motor_angle = np.array([[old.right_rotor], [old.left_rotor]]) 
            self.Va = old.Va  
            self.alpha = old.alpha  
            self.beta = old.beta

    def add_to_position(self, n=0, e=0, d=0):
        self.pos = self.pos + np.array([[n], [e], [d]])

    def euler_angles(self)->tuple[float, float, float]:
        phi, theta, psi = rotation_to_euler(self.R)
        return phi, theta, psi
    
    def old_format(self)->MsgStateOld:    
        old = MsgStateOld()
        old.pn = self.pos[0,0]      # inertial north position in meters
        old.pe = self.pos[1,0]      # inertial east position in meters
        old.h = -self.pos[2,0]       # inertial altitude in meters
        phi, theta, psi = rotation_to_euler(self.R)
        old.phi = phi     # roll angle in radians
        old.theta = theta   # pitch angle in radians
        old.psi = psi     # yaw angle in radians
        old.Va = self.Va      # airspeed in meters/sec
        old.alpha = self.alpha   # angle of attack in radians
        old.beta = self.beta    # sideslip angle in radians
        old.p = self.omega[0,0]       # roll rate in radians/sec
        old.q = self.omega[1,0]       # pitch rate in radians/sec
        old.r = self.omega[2,0]       # yaw rate in radians/sec
        old.Vg = self.Va      # groundspeed in meters/sec
        old.gamma = old.phi + self.alpha   # flight path angle in radians
        old.chi = np.arctan2(self.vel[1,0], self.vel[0,0])     # course angle in radians
        old.wn = 0.      # inertial windspeed in north direction in meters/sec
        old.we = 0.      # inertial windspeed in east direction in meters/sec
        old.bx = self.gyro_bias[0,0]      # gyro bias along roll axis in radians/sec
        old.by = self.gyro_bias[1,0]      # gyro bias along pitch axis in radians/sec
        old.bz = self.gyro_bias[2,0]      # gyro bias along yaw axis in radians/sec
        old.right_rotor = self.motor_angle[0,0]  # angle of the right motor in radians
        old.left_rotor = self.motor_angle[1,0]  # angle of the left motor in radians
        old.u = self.vel[0,0]       # inertial velocity resolved along body x-axis in m/s
        old.v = self.vel[1,0]       # inertial velocity resolved along body y-axis in m/s
        old.w = self.vel[2,0]       # inertial velocity resolved along body z-axis in m/s
        return old
    