import numpy as np
from controllers.quad.pid_control import PidControl

class AttitudeControl:
    def __init__(self, ts_control):
        # control gains:  roll
        phi_kp = 4.0
        phi_ki = 0.0
        phi_kd = 0.2
        # control gains:  pitch
        theta_kp = 4.0
        theta_ki = 0.0
        theta_kd = 0.2
        # control gains:  yaw
        psi_kp = 0.5
        psi_ki = 0.0
        psi_kd = 0.1
        self.phi_control = PidControl(kp=phi_kp, ki=phi_ki, kd=phi_kd, Ts=ts_control, limit=np.inf)
        self.theta_control = PidControl(kp=theta_kp, ki=theta_ki, kd=theta_kd, Ts=ts_control, limit=np.inf)
        self.psi_control = PidControl(kp=psi_kp, ki=psi_ki, kd=psi_kd, Ts=ts_control, limit=np.inf)

    def update(self, cmd, state):
        p_c = self.phi_control.update(cmd.item(0), state.phi)
        q_c = self.theta_control.update(cmd.item(1), state.theta)
        r_c = self.psi_control.update(cmd.item(2), state.psi)
        return np.array([[p_c],[q_c],[r_c]])

    def sat(self, val, low, high):
        if val < low:
            val = low
        elif val > high:
            val = high
        return val
