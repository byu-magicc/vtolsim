#/usr/bin/python3

import numpy as np
import sys
sys.path.append('..')

import parameters.anaconda_parameters as QUAD
import parameters.geometric_control_parameters as GEO_CTRL
from copy import copy

from message_types.msg_state import MsgState
from message_types.msg_trajectory import MsgTrajectory

from tools.rotations import *

class PitchFreeTrajectoryTracker():

    def __init__(self):
        self.pos_errs = []
        self.vel_errs = []
        self.F_ds = []
        self.R = []


        self.counter = 0
        pass

    def update(self, state: MsgState, trajectory: MsgTrajectory):
        
        # position error
        #the actual position (pn, pe, pd)
        pos_inert = np.array([[state.north],
                          [state.east],
                          [-state.altitude]])
        #the commanded position (pn, pe, pd)
        pos_ref_inert = trajectory.pos_des_inertial
        #position error (pn, pe, pd)
        pos_err = pos_ref_inert - pos_inert

        # velocity error
        #actual body frame velocity (u, v, w)
        vel_body = np.array([[state.u],
                          [state.v],
                          [state.w]])
        #desired body frame velocity (u, v, w)
        vel_ref_inertial = trajectory.vel_des_inertial
        #gets the rotation from the the body to the inertial
        R_body2inert = euler_to_rotation(phi=state.phi, theta=state.theta, psi=state.psi)
        #gets the Rotatiom from inertial to body
        R_inert2body = R_body2inert.T
        #gets the velocity in the body grame
        vel_ref_body = R_inert2body @ vel_ref_inertial
        #error body frame velocity (u, v, w)
        vel_err_body = vel_ref_body - vel_body


        #lets go through and get the velocity error in the inertial, and try that and see if that changes anything.
        #gets the velocity actual of the craft in the inertial frame
        vel_inert = R_body2inert @ vel_body
        #gets the velocity  error in the inertial frame
        vel_err_inert = vel_ref_inertial - vel_inert


        # heading error
        #gets the desired psi trajectory
        psi_r = trajectory.psi
        #
        x_di = np.array([[np.cos(psi_r), np.sin(psi_r), 0]]).T

        # desired force vector computation
        acc_r = trajectory.accel_des_inertial
        e3 = np.array([[0, 0, 1]]).T
        f_di = QUAD.mass * (acc_r - QUAD.gravity*e3 + GEO_CTRL.Kp @ pos_err + GEO_CTRL.Kd @ vel_err_inert)



        # Desired rotation and body force computation
        y_di = np.cross(x_di.reshape(-1), f_di.reshape(-1)).reshape((3,1)) / \
            np.linalg.norm(np.cross(x_di.reshape(-1), f_di.reshape(-1)).reshape((3,1)))
        z_di = np.cross(x_di.reshape(-1), y_di.reshape(-1)).reshape((3,1))


        Fx_d = (x_di.T @ f_di).item(0)
        Fz_d = (z_di.T @ f_di).item(0)
        F_d = np.array([[Fx_d, Fz_d]]).T

        R = np.concatenate((x_di, y_di, z_di), axis=1)

        #appends all the information
        (self.pos_errs).append(copy(pos_err))
        (self.vel_errs).append(copy(vel_err_body))
        (self.F_ds).append(copy(F_d))
        (self.R).append(copy(R))

        if self.counter % 50 == 0:
            potato = 0


        self.counter += 1

        return F_d, R
    
    #helper function to get all of the information buildup
    def getInfo(self):
        return self.pos_errs, self.vel_errs, self.F_ds, self.R