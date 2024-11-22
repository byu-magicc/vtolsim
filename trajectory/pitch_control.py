#!/usr/bin/python3

import numpy as np
import sys
from scipy.linalg import expm
sys.path.append('..')
import parameters.geometric_control_parameters as CTRL
import parameters.anaconda_parameters as QUAD
from tools.rotations import hat, euler_to_rotation

class PitchControl():

    def __init__(self):
        #creates the arrays to store the needed information
        
        pass

    def exp(self, omega):
        omega_skew = hat(omega)
        theta = np.linalg.norm(omega)
        return np.eye(3) + omega_skew * np.sin(theta)/theta + \
            omega_skew**2 * (1-np.cos(theta)) / theta**2

    def update(self, thrust_input, R_d2i, v_body):
        if v_body.item(0) == 0:
            alpha = 0
        else:
            alpha = np.arctan(v_body.item(1)/v_body.item(0))
        Va = np.linalg.norm(v_body)
        sigma = (1 + np.exp(-QUAD.M*(alpha-QUAD.alpha0)) + np.exp(QUAD.M*(alpha+QUAD.alpha0))) / \
            ((1 + np.exp(-QUAD.M*(alpha-QUAD.alpha0))) * (1 + np.exp(QUAD.M*(alpha+QUAD.alpha0)))) 
        CL = (1-sigma)*(QUAD.C_L_0+QUAD.C_L_alpha*alpha) + \
            sigma*2*np.sign(alpha)*(np.sin(alpha)**2)*np.cos(alpha)
        CD = QUAD.C_D_p + ((QUAD.C_L_0 + QUAD.C_L_alpha*alpha)**2) / (np.pi*QUAD.e*QUAD.AR)
        # compute Lift and Drag Forces
        F_lift = (1/2)*QUAD.rho*(Va**2)*QUAD.S_wing*CL
        F_drag = (1/2)*QUAD.rho*(Va**2)*QUAD.S_wing*CD

        f_body = np.array([[np.cos(alpha), -np.sin(alpha)],
                           [np.sin(alpha), np.cos(alpha)]]) @ \
                               np.array([[-F_drag], [-F_lift]])
        T_d = thrust_input-f_body
        # print(T_d)

        # R_p2d = Euler2Rotation(0, np.radians(12), 0)
        # R_p2i = R_p2d @ R_d2i
        # T_d = (R_p2d @ np.array([[T_d.item(0), 0, T_d.item(1)]]).T)[(0, 2), :]
        # # print(f'T: {T.T}')
        # print(f'T_d: {T_d.T}')
        # return T_d, R_p2i
        
        # return T_d, R_d2i
        return self.solve_pitch(T_d, R_d2i, Va)

    def solve_pitch(self, T, R_d2i, Va):
        # if Va < 2:
        #     return T, R_d2i

        thrust_ang = np.arctan2(-T.item(1), T.item(0))
        # print(np.degrees(thrust_ang))
        # give up if it's in this quadrant
        # if thrust_ang < -np.pi / 2:
        #     return T, R_d2i
        # elif thrust_ang > np.pi / 2:
        #     theta_p = thrust_ang - np.pi/2
        # elif thrust_ang < 0:
        #     theta_p = thrust_ang
        # # if in first quadrant, no need to change
        # else:
        #     return T, R_d2i
        default = 10
        if thrust_ang > np.radians(90+default):
            theta_p = thrust_ang - np.radians(90+default)
        # elif Va > 5: 
        #     theta_p = np.radians(5)
        # else:
        #     theta_p = np.radians(Va)
        else: theta_p = np.radians(default)

        # print(f'theta_p: {np.degrees(theta_p)}')
        
        # R_p2d = self.exp(theta_p*np.array([[0, 1, 0]]).T)
        R_p2d = expm(-hat(theta_p*np.array([0., 1., 0.]))).T #Euler2Rotation(0, theta_p, 0)
        R_p2i =  R_d2i @ R_p2d
        # print(np.array([[T.item(0), 0, T.item(1)]]) @ R_p2d)
        T_d = (R_p2d @ np.array([[T.item(0), 0, T.item(1)]]).T)[(0, 2), :]
        # print(f'T: {T.T}')
        # print(f'T_d: {T_d.T}')
        return T_d, R_p2i