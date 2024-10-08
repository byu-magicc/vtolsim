#this file implements a low level controller for the 

#imports the forces moments derivatives function, which we will use to test
from forces_torques_derivatives import forces_moments_derivatives

import numpy as np
from controllers.quad.pid_control import PControl

#imports the delta message type
from message_types.quad.msg_delta import MsgDelta
#imports the state message
from message_types.quad.msg_state import MsgState

import math

import parameters.quad.simulation_parameters as SIM

import parameters.quad.anaconda_parameters as QUAD

import parameters.quad.control_allocation_parameters as CAP

from tools.rotations import quaternion_to_rotation

import scipy.optimize as spo

import copy

from scipy.optimize import minimize

#imports the class for the wrench calculation
from forces_torques_derivatives import wrenchCalculation


#instantiates the dynamics for the quadplane
from models.quad.quad_dynamics import QuadDynamics

#saves the trim
from models.quad.trimValues import trimDelta

import time




#creates the low level control class, where we 
class LowLevelControl_successiveControl:

    #creates the initialization function
    def __init__(self,
                 M: float=0.5,
                 Va0: float=0.0,
                 ts: float=0.01):
        
        #instantiates the wrench calculation class
        self.wrenchCalculator = wrenchCalculation()

        #creates instance of the state class
        self.state = MsgState()

        #stores the wind

        #saves the wind
        self.wind = np.array([[0.0], [0.0], [0.0]])       

        # control gains: p-channel
        p_kp = 0.15

        # control gains: q-channel
        q_kp = 0.08

        # control gains: r-channel
        r_kp = 0.1       


        self.M = M
        self.Va0 = Va0

        #stores the time sample rate of the simulation parameters
        self.Ts = SIM.ts_control

        self.p_ctrl = PControl(kp=p_kp, Ts=self.Ts)
        self.q_ctrl = PControl(kp=q_kp, Ts=self.Ts)
        self.r_ctrl = PControl(kp=r_kp, Ts=self.Ts)
        self.output = MsgDelta()
        self.alpha = 0.99

        #stores the previous solution for the delta c portion
        self.delta_c_previous_solution = CAP.init_plane_control

        #stores the previous solution for the delta r portion
        self.delta_t_previous_solution = CAP.init_quad_control

        #stores the whole previous solution
        self.previous_solution = np.ndarray((8,1))





    #creates the update function
    def update(self, f_d: np.ndarray,#desired force 2x1 vector
                     omega_d: np.ndarray, #desired angular velocity 3x1 vector
                     state: MsgState, #Quad state
                     wind: np.ndarray): #the wind in the inertial frame
        
        #stores the state
        self.state = state

        #stores the wind
        self.wind = wind

        #get the desired torque vector from:
        #1. The desired omega input and
        #2. The actual omega input
        #by updating the proportional controllers for each variable
        tau_d = np.array([[self.p_ctrl.update(omega_d.item(0), state.omega.item(0))],
                          [self.q_ctrl.update(omega_d.item(1), state.omega.item(1))],
                          [self.r_ctrl.update(omega_d.item(2), state.omega.item(2))]])
        
        #gets the wrench desired 
        wrenchDesired = np.concatenate((f_d, tau_d), axis=0)

        #gets the delta solution
        delta = self.computeOptimization(wrenchDesired=wrenchDesired)

        #returns the delta
        return delta
        

    #creates a function that gets the delta output from the desired wrench
    #and the current state
    def computeOptimization(self, wrenchDesired: np.ndarray):

        #creates the x0
        x0_delta_c = self.delta_c_previous_solution

        #calls the minimization function from the 
        delta_c_result = minimize(fun=self.objectiveFunction,
                                  x0=x0_delta_c,
                                  args=(wrenchDesired),
                                  bounds=CAP.actuatorBounds_delta_c,
                                  jac=True,
                                  options={'maxiter': CAP.max_iter})
        

        delta_c = delta_c_result.x
        elevator = delta_c.item(0)
        aileron = delta_c.item(1)
        rudder = delta_c.item(2)
        forwardThrottle = delta_c.item(3)
        #based on the delta_c result, we construct the next bounds
        delta_t_bounds = [(elevator, elevator),
                          (aileron, aileron),
                          (rudder, rudder),
                          (forwardThrottle, forwardThrottle),
                          (0.0, 1.0),
                          (0.0, 1.0),
                          (0.0, 1.0),
                          (0.0, 1.0)]
        
        #gets the four previous solutions for the delta solution
        v1_prev = self.previous_solution.item(4)
        v2_prev = self.previous_solution.item(5)
        v3_prev = self.previous_solution.item(6)
        v4_prev = self.previous_solution.item(7)

        #creates the x0 for the delta_t
        x0_delta_t = np.array([elevator,
                               aileron,
                               rudder,
                               forwardThrottle,
                               v1_prev,
                               v2_prev,
                               v3_prev,
                               v4_prev])

        #based on these minimizations, we then get the delta_t result
        delta_t_result = minimize(fun=self.objectiveFunction,
                                  x0=x0_delta_t,
                                  args=(wrenchDesired),
                                  bounds=delta_t_bounds,
                                  jac=True,
                                  options={'maxiter': CAP.max_iter})
        

        #gets the final retuls
        deltaFinal_array = delta_t_result.x

        #gets the delta
        deltaFinal = MsgDelta()
        deltaFinal.from_array(deltaFinal_array)

        #returns the delta final
        return deltaFinal


    #defines the objective function
    def objectiveFunction(self, deltaArray: np.ndarray, wrenchDesired: np.ndarray):

        #gets the delta message
        deltaMessage = MsgDelta()
        deltaMessage.from_array(delta_array=deltaArray)


        #saves the mixing matrix to mix the moments with the forces with the right weights
        K_Wrench = CAP.K_Wrench

        #gets the wrench and the wrench Jacobian
        wrench_actual, wrench_actualJacobian = \
            self.wrenchCalculator.forces_moments_derivatives(delta=deltaMessage,
                                                             state=self.state)

        #gets the wrench error
        wrenchError = wrenchDesired - wrench_actual

        #gets the objective, which is the magnitude of the wrench error,
        #with the scaling factor of the K_Tau matrix
        #(1x1) = (1x1) * (1x5) * (5x5) * (5x1)
        objective = 0.5 * wrenchError.T @ K_Wrench @ wrenchError



        #gets the gradient of the objective function 
        # (A vector of the derivative of the objective function with respect to
        # each of the 8 delta control inputs)
        objective_gradient = -wrench_actualJacobian @ K_Wrench @ wrenchError

        #returns the objective and the objective gradient
        return objective, objective_gradient