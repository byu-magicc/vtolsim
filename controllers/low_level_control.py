import numpy as np
from controllers.pid_control import PidControl
#from message_types.msg_controls import MsgControls
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
import parameters.control_allocation_parameters as CP
import math


class LowLevelControl:
    def __init__(self, 
                 M: float=0.5, # M defines the transition point for mixing
                 Va0: float=0., # airspeed
                 ts: float=0.01,  # sample rate
                 ):
        # control gains: p-channel
        p_kp = 0.15
        p_ki = 0.03
        p_kd = 0.00
        # control gains: q-channel
        q_kp = 0.08
        q_ki = 0.03
        q_kd = 0.0
        # control gains: r-channel
        r_kp = 0.1
        r_ki = 0.03
        r_kd = 0.01
        self.M = M
        self.Va0 = Va0
        self.p_ctrl = PidControl(kp=p_kp, ki=p_ki, kd=p_kd, Ts=ts, limit=np.inf)
        self.q_ctrl = PidControl(kp=q_kp, ki=q_ki, kd=q_kd, Ts=ts, limit=np.inf)
        self.r_ctrl = PidControl(kp=r_kp, ki=r_ki, kd=r_kd, Ts=ts, limit=np.inf)
        self.mixer = CP.mixer
        #self.output = MsgControls()
        self.output = MsgDelta()
        self.limits = CP.limits           
        self.alpha = 0.99
        self.elevon_right = 0.0
        self.elevon_left = 0.0  
        self.elevon_k = 5000

    def update(self, 
               f_d: np.ndarray, # desired force 2x1 vector 
               omega_d: np.ndarray,  # desired angular velocity 3x1 vector
               state: MsgState,  # vtol state 
               sigma: float=None, # mixing parameter
               ):
        tau_d = np.array([
            [self.p_ctrl.update(omega_d.item(0), state.omega.item(0))],
            [self.q_ctrl.update(omega_d.item(1), state.omega.item(1))],
            [self.r_ctrl.update(omega_d.item(2), state.omega.item(2))],
        ]) 
        wrench = np.concatenate((f_d.reshape(2,1), tau_d), axis=0)

        if sigma is None: # use sigma from airspeed
            sigma = self.compute_sigma(state.Va)

        scale = np.ones((5,7))
        scale[2:4,0:5] = 1.0-sigma
        scale[2:4,5:] = sigma
        scaled_mixer = self.mixer * scale  # element wise

        zeta = np.zeros((7,1))
        for i in range(zeta.shape[0]):
            zeta[i] = np.dot(wrench.T, scaled_mixer[:,i]).item(0)

        servo_right = math.atan2(zeta.item(2), zeta.item(1))
        servo_left = math.atan2(zeta.item(4), zeta.item(3))
        throttle_right = math.sqrt(zeta.item(1)**2 + zeta.item(2)**2)
        throttle_left = math.sqrt(zeta.item(3)**2 + zeta.item(4)**2)
        if state.Va < .01:
            self.elevon_right = 0.0
            self.elevon_left = 0.0
        else:
            self.elevon_right = self.alpha*self.elevon_right + (1 - self.alpha)*zeta.item(5)*self.elevon_k/state.Va**2
            self.elevon_left =  self.alpha*self.elevon_left + (1- self.alpha)*zeta.item(6)*self.elevon_k/state.Va**2

        delta = np.array([[zeta.item(0)],[throttle_right],[throttle_left],[servo_right],[servo_left],[self.elevon_right],[self.elevon_left]])
        for i in range(delta.shape[0]):
            delta[i] = self.sat(delta.item(i), self.limits.item(0,i), self.limits.item(1,i))

        self.output.elevator =  delta.item(6) + delta.item(5)
        self.output.aileron = delta.item(6) - delta.item(5)
        self.output.rudder = np.radians(0.0)  
        self.output.throttle_right = delta.item(1) 
        self.output.throttle_left = delta.item(2)
        self.output.throttle_rear = delta.item(0)
        self.output.motor_right = delta.item(3)
        self.output.motor_left = delta.item(4)
        return self.output

    #returns value between 0-1; sigmoid based on Va centered around Va0
    def compute_sigma(self, Va):
        sig = 1.0/(1 + np.exp(-self.M*(Va - self.Va0)))
        return sig

    def sat(self, val, low, high):
        if val < low:
            val = low
        elif val > high:
            val = high
        return val
