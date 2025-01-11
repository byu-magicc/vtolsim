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

from spatialmath.base import skewa, skew

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




        #in order to obtain the correct solution to the rotation matrix, we need to solve for both
        #the positive and negative solution


        #primarily, we obtain the F_x desired
        Fx_des = (x_di.T @ f_di).item(0)

        #gets the main vector for the F_z desired
        Fz_des_vector = f_di - Fx_des * x_di


        #------------------------------------Positive Solution Section------------------------------

        #gets the positive solution for the magnitude of the Fz_desired
        Fz_des_posSln = np.linalg.norm(Fz_des_vector)

        #gets the full length 2 desired forces
        F_des_posSln = np.array([[Fx_des],
                                 [Fz_des_posSln]])

        #gets the z hat desired vector, expressed in the inertial frame, positive solution
        z_di_posSln = Fz_des_vector / Fz_des_posSln

        #gets the y di positive solution using the cross product matrix
        y_di_posSln = skew(z_di_posSln) @ x_di

        #puts together the Rotation positive solution matrix
        R_des2inert_posSln = np.concatenate((x_di, y_di_posSln, z_di_posSln), axis=1)

        #gets the rotation from the desired to the body matrix
        R_des2body_posSln = R_inert2body @ R_des2inert_posSln

        #gets the axis and angle for the positive solution.
        axis_posSln, angle_posSln = rotation_to_axisAngle(R=R_des2body_posSln)


        #------------------------------------Negative Solution Section------------------------------
        #gets the negative solution for the magnitude of the Fz_desired
        Fz_des_negSln = -np.linalg.norm(Fz_des_vector)

        #gets the full length 2 desired forces
        F_des_negSln = np.array([[Fx_des],
                                 [Fz_des_negSln]])

        #gets the z hat desired vector,  expressed in the inertial frame, negative solution
        z_di_negSln = Fz_des_vector / Fz_des_negSln

        #gets the y_di negative solution using the cross product matrix
        y_di_negSln = skew(z_di_negSln) @ x_di

        #puts together the Rotation negative solution matrix
        R_des2inert_negSln = np.concatenate((x_di, y_di_negSln, z_di_negSln), axis=1)

        #gets the rotation from the desired to the body matrix
        R_des2body_negSln = R_inert2body @ R_des2inert_negSln

        #gets the axis angle for the negative solution
        axis_negSln, angle_negSln = rotation_to_axisAngle(R=R_des2body_negSln)


        #------------comparison section--------------------------------------------------
        
        #checks which angle has the greater absolute value
        #case the positive solution is greater than the negative solution in absolute value terms
        if np.abs(angle_posSln) >= np.abs(angle_negSln):
            #sets the desired Force vector
            F_des = F_des_negSln
            #sets the Rotation for the negative solution
            R_des2inert = R_des2inert_negSln
        #case we go with the positive solution
        else:
            F_des = F_des_posSln
            R_des2inert = R_des2inert_posSln
            






        #appends all the information
        (self.pos_errs).append(copy(pos_err))
        (self.vel_errs).append(copy(vel_err_body))
        (self.F_ds).append(copy(F_des))
        (self.R).append(copy(R_des2inert))

        if self.counter % 50 == 0:
            potato = 0


        self.counter += 1

        return F_des, R_des2inert
    
    #helper function to get all of the information buildup
    def getInfo(self):
        return self.pos_errs, self.vel_errs, self.F_ds, self.R