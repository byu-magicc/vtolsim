import numpy as np
from controllers.pid_control import PidControl


class RateControl:
    def __init__(self, ts_control):
        # Control gains
        # p-channel
        p_kp = 0.50  
        p_ki = 1.00
        p_kd = 0.001
        # q-channel
        q_kp = 0.70  
        q_ki = 1.00
        q_kd = 0.001
        # r-channel
        r_kp = 0.50  
        r_ki = 1.00
        r_kd = 0.001
        self.p_ctrl = PidControl(kp=p_kp, ki=p_ki, kd=p_kd, Ts=ts_control, limit=np.inf)
        self.q_ctrl = PidControl(kp=q_kp, ki=q_ki, kd=q_kd, Ts=ts_control, limit=np.inf)
        self.r_ctrl = PidControl(kp=r_kp, ki=r_ki, kd=r_kd, Ts=ts_control, limit=np.inf)

    def update(self, omega_d, omega, Ts=None):
        tau_x_d = self.p_ctrl.update(omega_d[0], omega[0], Ts)
        tau_y_d = self.q_ctrl.update(omega_d[1], omega[1], Ts)
        tau_z_d = self.r_ctrl.update(omega_d[2], omega[2], Ts)
        return np.array([tau_x_d, tau_y_d, tau_z_d]).reshape(-1)
