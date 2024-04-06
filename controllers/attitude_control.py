import numpy as np
from controllers.pid_control import PidControl


#attitude control controls the roll, pitch, and yaw of the vehicle
#as in phi, thete, and psi
class AttitudeControl:
    def __init__(self, ts_control):
        # control gains:  roll (phi)
        phi_kp = 4.0
        phi_ki = 0.0
        phi_kd = 0.2
        # control gains:  pitch (theta)
        theta_kp = 4.0
        theta_ki = 0.0
        theta_kd = 0.2
        # control gains:  yaw (psi)
        psi_kp = 0.5
        psi_ki = 0.0
        psi_kd = 0.1

        #instantiates the phi control loop, which is a PID control
        self.phi_control = PidControl(kp=phi_kp, ki=phi_ki, kd=phi_kd, Ts=ts_control, limit=np.inf)
        #instantiates the theta control loop, which is a PID control instance
        self.theta_control = PidControl(kp=theta_kp, ki=theta_ki, kd=theta_kd, Ts=ts_control, limit=np.inf)
        #instantiates the psi control loop, which is a PID control
        self.psi_control = PidControl(kp=psi_kp, ki=psi_ki, kd=psi_kd, Ts=ts_control, limit=np.inf)

    #update function for attitude control

    #Arguments:
    #1. cmd: a length 3 vector which contains the phi, theta, and psi commands respectively
    #2. state: state vector of the vtol. 

    #Returns:
    #1. the commanded p, q, and r, which are the commanded derivatives of roll, pitch, and yaw respectively

    def update(self, cmd, state):
        p_c = self.phi_control.update(cmd.item(0), state.phi)
        q_c = self.theta_control.update(cmd.item(1), state.theta)
        r_c = self.psi_control.update(cmd.item(2), state.psi)
        return np.array([[p_c],[q_c],[r_c]])

    #defines the saturation function
    def sat(self, val, low, high):
        if val < low:
            val = low
        elif val > high:
            val = high
        return val
