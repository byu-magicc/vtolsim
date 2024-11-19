#/usr/bin/python3

import numpy as np
import sys
sys.path.append('..')

import parameters.anaconda_parameters as QUAD
import parameters.geometric_control_parameters as CTRL

class PitchFreeTrajectoryTracker():

    def __init__(self):
        self.pos_errs = []
        self.vel_errs = []
        self.F_ds = []
        self.R = []
        pass

    def update(self, state, trajectory):
        
        # position error
        #the actual position (pn, pe, pd)
        pos_i = state[0:3,:]
        #the commanded position (pn, pe, pd)
        pos_r = trajectory[0:3,0].reshape((3,1))
        #position error (pn, pe, pd)
        pos_err = pos_i - pos_r

        # velocity error
        #actual body frame velocity (u, v, w)
        vel_i = state[3:6,:]
        #desired body frame velocity (u, v, w)
        vel_r = trajectory[0:3,1].reshape((3,1))
        #error body frame velocity (u, v, w)
        vel_err = vel_i - vel_r

        # heading error
        #gets the desired psi trajectory
        psi_r = trajectory[3,0]
        #
        x_di = np.array([[np.cos(psi_r), np.sin(psi_r), 0]]).T

        # desired force vector computation
        acc_r = trajectory[0:3,2].reshape((3,1))
        e3 = np.array([[0, 0, 1]]).T
        f_di = QUAD.mass * (acc_r - QUAD.gravity*e3 - CTRL.Kp @ pos_err - CTRL.Kd @ vel_err)

        # Desired rotation and body force computation
        y_di = np.cross(x_di.reshape(-1), f_di.reshape(-1)).reshape((3,1)) / \
            np.linalg.norm(np.cross(x_di.reshape(-1), f_di.reshape(-1)).reshape((3,1)))
        z_di = np.cross(x_di.reshape(-1), y_di.reshape(-1)).reshape((3,1))


        Fx_d = (x_di.T @ f_di).item(0)
        Fz_d = (z_di.T @ f_di).item(0)
        F_d = np.array([[Fx_d, Fz_d]]).T

        R = np.concatenate((x_di, y_di, z_di), axis=1)

        #appends all the information
        (self.pos_errs).append(pos_err)
        (self.vel_errs).append(vel_err)
        (self.F_ds).append(F_d)
        (self.R).append(R)

        return F_d, R
    
    #helper function to get all of the information buildup
    def getInfo(self):
        return self.pos_errs, self.vel_errs, self.F_ds, self.R