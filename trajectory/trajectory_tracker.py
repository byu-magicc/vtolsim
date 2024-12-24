import numpy as np
import sys
sys.path.append('..')

import parameters.anaconda_parameters as QUAD
import parameters.geometric_control_parameters as CTRL
from copy import copy

from message_types.msg_state import MsgState
from message_types.msg_trajectory import MsgTrajectory

from tools.rotations import *

class TrajectoryTracker:

    #creates the initialization function
    def __init__(self):
        self.pos_errs = []
        self.vel_errs = []
        self.F_ds = []
        self.R = []

    #creates the update function
    #arguments:
    #1. State: the vector low level state of the plane
    #2. trajectory: the current desired 
    #position, velocity, acceleration, and yaw from the B Spline generator
    def update(self, state: MsgState, trajectory: MsgTrajectory):

        #gets the current position of the body in the 
        #gets the actual state position

        #TODO figure out whether 
        pos_inertial = np.array([[state.north],
                                 [state.east],
                                 [state.altitude]])
        
        #gets the actual current velocity in the body frame
        vel_body = np.array([[state.u],
                             [state.v],
                             [state.w]])
        
        #gets the rotation matrix to rotate from body back to inertial 
        Rotation = euler_to_rotation(phi=state.phi, theta=state.theta, psi=state.psi)

        #gets the actual velocity in the inertial frame
        vel_inertial = Rotation @ vel_body

        #gets the desired position, in the inertial frame
        pos_des_inertial = trajectory.pos_des_inertial
        #gets the desired velocity in the inertial frame
        vel_des_inertial = trajectory.vel_des_inertial
        #gets the desired acceleration in the inertial frame
        accel_des_inertial = trajectory.accel_des_inertial


        #gets the positional error in the inertial frame
        pos_error_inertial = pos_inertial - pos_des_inertial

        #gets the velocity error in the inertial frame
        vel_error_inertial = vel_inertial - vel_des_inertial


        #gets the acceleration 

