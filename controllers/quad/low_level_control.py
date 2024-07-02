#This file implements the controller for the lower levels of the system
import numpy as np
from controllers.quad.pid_control import PControl

#imports the delta message type
from message_types.quad.msg_delta import MsgDelta
#imports the state message
from message_types.quad.msg_state import MsgState
import parameters.control_allocation_parameters as CP
import math

import parameters.simulation_parameters as SIM

#imports the vtol dynamics for 
from models.quad.quad_dynamics import QuadDynamics


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

        #stores the time sample rate of the simulation parameters
        self.Ts = SIM.ts_control

        self.p_ctrl = PControl(kp=p_kp, Ts=self.Ts)
        self.q_ctrl = PControl(kp=q_kp, Ts=self.Ts)
        self.r_ctrl = PControl(kp=r_kp, Ts=self.Ts)
        self.mixer = CP.mixer
        #self.output = MsgControls()
        self.output = MsgDelta()
        self.limits = CP.limits
        self.alpha = 0.99

        #instantiates a quadDynamics instance
        quad = QuadDynamics(SIM.ts_simulation)

    #creates the update function
    def update(self, f_d: np.ndarray,#desired force 2x1 vector
                     omega_d: np.ndarray, #desired angular velocity 3x1 vector
                     state: MsgState, #Quad state
                     sigma: float=None): #mixing parameter
        
        #gets the tau desired vector from the omega desired input and the actual omega
        tau_d = np.array([
            [self.p_ctrl.update(omega_d.item(0), state.omega.item(0))],
            [self.q_ctrl.update(omega_d.item(1), state.omega.item(1))],
            [self.r_ctrl.update(omega_d.item(2), state.omega.item(2))]
        ])

        #gets the wrench desired, which is the generalized version of forces and torques
        Wrench_D = np.clongdouble((f_d, tau_d), axis=0)

        



