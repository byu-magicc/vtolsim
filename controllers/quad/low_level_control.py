#This file implements the controller for the lower levels of the system
import numpy as np
from controllers.quad.pid_control import PidControl

#imports the delta message type
from message_types.quad.msg_delta import MsgDelta
#imports the state message
from message_types.quad.msg_state import MsgState
import parameters.control_allocation_parameters as CP
import math



#creates the Low Level Control class
class LowLevelControl:

    #creates the initialization function
    def __init__(self,
                 M: float=0.5,
                 Va0: float=0.0,
                 ts: float=0.01):
        
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

    #creates the update function
    def update(self, f_d: np.ndarray,#desired force 2x1 vector
                     omega_d: np.ndarray, #desired angular velocity 3x1 vector
                     state: MsgState, #Quad state
                     sigma: float=None): #mixing parameter
        
        #gets the tau desired vector from the omega desired input and the actual omega
        tau_d = np.array([
            [self.p_ctrl.update(omega_d.item(0), state.omega.item(0))],
            [self.q_ctrl.update(omega_d.item(1), state.omega.item(1))],
            [self.r_ctrl.update(omega_d.item(2), state.omega.item(2))],
        ]) 
        wrench = np.concatenate((f_d, tau_d), axis=0)

        if sigma is None:
            sigma = self.compute_sigma(state.Va)

        scale = np.ones((5,7))
        scale[2:4,0:5] = 1.0-sigma
        scale[2:4,5:] = sigma
        scaled_mixer = self.mixer * scale  # element wise

        zeta = np.zeros((7,1))
        for i in range(zeta.shape[0]):
            zeta[i] = np.dot(wrench.T, scaled_mixer[:,i]).item(0)