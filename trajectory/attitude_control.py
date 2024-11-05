#!/usr/bin/python3

import numpy as np
import sys
sys.path.append('..')
import parameters.geometric_control_parameters as CTRL

class AttitudeControl():

    def __init__(self):
        pass

    def Pa(self, R):
        return .5 * (R - R.T)

    def inv_skew(self, R):
        return np.array([[-R[1,2], R[0,2], -R[0,1]]]).T

    def update(self, R_b2i, R_d2i):
        R_d2b = R_b2i.T @ R_d2i
        omega_c = CTRL.omega_Kp @ self.inv_skew(self.Pa(R_d2b))
        return omega_c