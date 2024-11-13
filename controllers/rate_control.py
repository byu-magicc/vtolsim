#This is the file that implements the rate controller

import sys
sys.path.append('..')
import numpy as np
import parameters.low_level_parameters as LP
from controllers.pid_control import PidControl


class RateControl:
    def __init__(self, ts_control):
        self.p_ctrl = PidControl(kp=LP.p_kp, ki=LP.p_ki, kd=LP.p_kd, Ts=ts_control, limit=np.inf)
        self.q_ctrl = PidControl(kp=LP.q_kp, ki=LP.q_ki, kd=LP.q_kd, Ts=ts_control, limit=np.inf)
        self.r_ctrl = PidControl(kp=LP.r_kp, ki=LP.r_ki, kd=LP.r_kd, Ts=ts_control, limit=np.inf)

    def update(self, omega_d, omega, Ts=None):
        tau_x_d = self.p_ctrl.update(omega_d[0], omega[0], Ts)
        tau_y_d = self.q_ctrl.update(omega_d[1], omega[1], Ts)
        tau_z_d = self.r_ctrl.update(omega_d[2], omega[2], Ts)

        return np.array([tau_x_d, tau_y_d, tau_z_d]).reshape(-1)
