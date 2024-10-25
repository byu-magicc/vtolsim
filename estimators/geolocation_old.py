"""
target geolocation algorithm
    - Beard & McLain, PUP, 2012
    - Updated:
        4/1/2022 - RWB
        4/6/2022 - RWB
        7/13/2023 - RWB
"""
import numpy as np
import parameters.simulation_parameters as SIM
import parameters.camera_parameters as CAM
from tools.rotations import euler_to_rotation
from estimators.filters import ExtendedKalmanFilterContinuousDiscrete


class Geolocation:
    def __init__(self, ts: float=0.01):
        self.ekf = ExtendedKalmanFilterContinuousDiscrete(
            f=self.f, 
            Q = 0.01 * np.diag([
                (1)**2,  # north position
                (1)**2,  # east position
                (1)**2,  # down position
                (10)**2,  # north velocity
                (10)**2,  # east velocity
                (10)**2,  # down velocity
                (3)**2,  # distance to target L
                ]),
            P0= 0.1*np.diag([
                10**2,  # north position
                10**2,  # east position
                10**2,  # down position
                10**2,  # north velocity
                10**2,  # east velocity
                10**2,  # down velocity
                10**2,  # distance to target L
                ]), 
            xhat0=np.array([[
                0.,  # north position
                0.,  # east position
                0.,  # down position
                0.,  # north velocity
                0.,  # east velocity
                0.,  # down velocity
                100.,  # distance to target L
                ]]).T, 
            Qu=0.01*np.diag([
                1**2, # mav north position
                1**2, # mav east position
                1**2, # mav down position
                1**2, # mav north velocity
                1**2, # mav east velocity
                1**2, # mav down velocity
                ]), 
            Ts=ts,
            N=10
        )
        self.R = .1 * np.diag([1.0, 1.0, 1.0, 1.0])

    def update(self, mav, pixels):
        # system input
        u = np.array([
            [mav.north], # mav position
            [mav.east], 
            [-mav.altitude],
            [mav.Vg * np.cos(mav.chi)], # mav velocity
            [mav.Vg * np.sin(mav.chi)], 
            [0.],
            ])    
        xhat, P = self.ekf.propagate_model(u)
        # update with pixel measurement
        y = pixels
        self.process_measurements(mav, pixels)
        xhat, P = self.ekf.measurement_update(
            y=y, 
            u=u,
            h=self.h,
            R=self.R)
        return xhat[0:3, :]  # return estimated NED position

    def f(self, x:np.ndarray, u:np.ndarray)->np.ndarray:
        # system dynamics for propagation model: xdot = f(x, u)
        target_position = x[0:3]
        target_velocity = x[3:6]
        L = x[6,0]
        mav_position = u[0:3]
        mav_velocity = u[3:6]
        target_position_dot = target_velocity
        target_velocity_dot = np.zeros((3,1))
        L_dot = ((target_position - mav_position).T @ (target_velocity - mav_velocity)) / L
        xdot = np.concatenate((target_position_dot, target_velocity_dot, L_dot), axis=0)
        return xdot

    def h(self, x:np.ndarray, u:np.ndarray)->np.ndarray:
        # measurement model y
        target_position = x[0:3]
        L = x[6:7]
        y = np.concatenate((target_position, L), axis=0)
        return y

    def process_measurements(self, mav, pixels):
        h = mav.altitude
        mav_position = np.array([[mav.north], [mav.east], [-h]])
        ell = np.array([[pixels.pixel_x], [pixels.pixel_y], [CAM.f]])
        ell_c = ell / np.linalg.norm(ell)
        R_b_i = euler_to_rotation(mav.phi, mav.theta, mav.psi)
        R_g_b = euler_to_rotation(0, mav.gimbal_el, mav.gimbal_az)
        R_c_g = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        ell_i = R_b_i @ R_g_b @ R_c_g @ ell_c
        L = h / ell_i.item(2)
        target_position = mav_position + L * ell_i
        y = np.concatenate((target_position, np.array([[L]])), axis=0)
        return y
