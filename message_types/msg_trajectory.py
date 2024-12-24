#This file implements a message class to communicate trajectory information and setpoints.
import numpy as np
#creates the message trajectory class



#This trajectory message class, as presently constituted works by storing the desired
#position, velocity, acceleration and course angle for a single instant of time.
#TODO I will likely change this in the future to change to contain all the timesteps for a trajectory
#***Important: Reminder that all of vectors are in the inertial frame ONLY, which means that when comparing
#With another velocity or acceleration vector in another function, that vector has to be rotated into the 
#The inertial frame as well, as they are often in the body frame.
class MsgTrajectory:

    #creates the init function
    #Arguments:
    #1. pos_d: Desired 3D position of the aircraft in the inertial frame
    #2. vel_d: Desired 3D velocity of the aircraft in the inertial frame
    #3. accel_d: Desired 3D velocity of the aircraft in the inertial frame
    #4. psi: the course angle of the aircraft, which is the angle of the x vector
    #        which has been projected onto the horizontal plane in the inertial frame
    def __init__(self, 
                 pos_des_inertial: np.ndarray, 
                 vel_des_inertial: np.ndarray, 
                 accel_des_inertial: np.ndarray,
                 psi: float):
        #stores the position, velocity, and acceleration
        
        #stores the position desired in the inertial frame
        self.pos_des_inertial = pos_des_inertial

        self.vel_des_inertial = vel_des_inertial
        self.accel_des_inertial = accel_des_inertial
        #stores the course angle
        self.psi = psi