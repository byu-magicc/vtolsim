#creates the state class for the quad plane

import numpy as np
from tools.rotations import rotation_to_euler, euler_to_rotation




#creates the state message class, which contains all of the state information for the quadplane
class MsgState:

    #creates the initialization functions
    def __init__(self,
                 pos: np.ndarray=np.array([[0.0], [0.0], [0.0]]),
                 vel: np.ndarray=np.array([[0.0], [0.0], [0.0]]),
                 R: np.ndarray=np.identity(3),
                 omega: np.ndarray=np.array([[0.0], [0.0], [0.0]]),
                 gyro_bias: np.ndarray=np.array([[0.0], [0.0], [0.0]]),
                 Va: float=0.0,
                 alpha: float=0.0,  
                 beta: float=0.0,
                 Vg: float=0.0,
                 chi: float=0.0):
        
        #saves the position
        self.pos = pos
        #saves the velocity
        self.vel = vel
        #saves the R
        self.R = R
        #saves the omega
        self.omega = omega
        #saves the gyro bias
        self.gyro_bias = gyro_bias
        #saves the airspeed
        self.Va = Va
        #saves the alpha or angle of attack
        self.alpha = alpha
        #saves the beta or sideslip angle
        self.beta = beta
        #saves the groundspeed
        self.Vg = Vg
        #saves the course angle
        self.chi = chi

    #creates function to add to the position
    def add_to_position(self, n=0, e=0, d=0):
        self.pos = self.pos + np.array([[n], [e], [d]])


    #gets the euler angles from the quaternion
    def euler_angles(self)->tuple[float, float, float]:
        phi, theta, psi = rotation_to_euler(self.R)
        return phi, theta, psi
    

    