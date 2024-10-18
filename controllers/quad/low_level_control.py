#this file implements a low level controller for the 


import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))



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
from controllers.quad.forces_torques_derivatives import wrenchCalculation


#instantiates the dynamics for the quadplane
from models.quad.quad_dynamics import QuadDynamics

#saves the trim
from models.quad.trimValues import trimDelta

import time

import copy

#creates a low level control class, where we find the whole delta
# which creates the min norm error for
class LowLevelControl_simultaneousControl:

    #creates the initialization function
    def __init__(self,
                 ts: float=0.01,
                 torqueControl = False):
        
        #instantiates the wrench calculation class
        self.wrenchCalculator = wrenchCalculation()

        #creates instance of the state class
        self.state = MsgState()

        #saves the time sample rate
        self.Ts = ts

        #saves the torque control variable. torqueControl determines whether we will directly control
        #the torque, or whether we will use a desire roll, pitch and yaw rate to find the desired torque
        self.torqueControl = torqueControl

        #saves the wind
        self.wind = np.array([[0.0], [0.0], [0.0]])       

        # control gains: p-channel
        p_kp = 0.15

        # control gains: q-channel
        q_kp = 0.08

        # control gains: r-channel
        r_kp = 0.1       


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


        #creates the Error vector between the desired and the actual to see what's happening
        self.error = np.ndarray((5,0))

        #store the wrench actual
        self.wrenchActual = np.ndarray((5,1))



        self.printerCounter = 0

        self.objectiveCounter = 0




    #creates the update function
    def update(self, f_desired: np.ndarray,#desired force 2x1 vector
                     state: MsgState, #Quad state
                     wind: np.ndarray, #the wind in the inertial frame
                     omega_d: np.ndarray, #desired angular velocity 3x1 vector
                     tau_desired: np.ndarray): #the desired torque array
        
        #stores the state
        self.state = state

        #stores the wind
        self.wind = wind



        #case, we are doing direct torque control
        if self.torqueControl:
            tau_d = tau_desired
        #otherwise we are going to use the omegas thing
        else:
            #get the desired torque vector from:
            #1. The desired omega input and
            #2. The actual omega input
            #by updating the proportional controllers for each variable
            tau_d = np.array([[self.p_ctrl.update(omega_d.item(0), state.omega.item(0))],
                              [self.q_ctrl.update(omega_d.item(1), state.omega.item(1))],
                              [self.r_ctrl.update(omega_d.item(2), state.omega.item(2))]])
        
        #reshapes f_d and tau_d
        f_d = f_desired.reshape(2,1)

        tau_d = tau_d.reshape(3,1)

        #gets the wrench desired 
        wrenchDesired = np.concatenate((f_d, tau_d), axis=0)

        #gets the delta solution
        delta = self.computeOptimization(wrenchDesired=wrenchDesired)


        self.printerCounter += 1
        #returns the delta
        return delta
        

    #creates a function that gets the delta output from the desired wrench
    #and the current state
    def computeOptimization(self, wrenchDesired: np.ndarray):

        #creates the x0
        x0_delta_c = np.concatenate((self.delta_c_previous_solution, np.array([0.0, 0.0, 0.0, 0.0])))

        self.objectiveCounter = 0
        #calls the minimization function from the 
        delta_result = minimize(fun=self.objectiveFunction,
                                x0=x0_delta_c,
                                args=(wrenchDesired),
                                bounds=CAP.actuatorBounds_delta_c,
                                jac=False,
                                options={'maxiter': CAP.max_iter})
        
        deltaArray = delta_result.x
        #saves the previous solution
        self.previous_solution = deltaArray


        #gets the delta
        deltaFinal = MsgDelta()
        deltaFinal.from_array(deltaArray)

        #saves the wrench error
        wrenchError = wrenchDesired - self.wrenchActual
        self.error = np.concatenate((self.error, wrenchError), axis=1)        

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
        wrenchError = wrench_actual - wrenchDesired


        #gets the objective, which is the magnitude of the wrench error,
        #with the scaling factor of the K_Tau matrix
        #(1x1) = (1x1) * (1x5) * (5x5) * (5x1)
        objective = 0.5 * wrenchError.T @ K_Wrench @ wrenchError
        
        #saves the actual wrench
        self.wrenchActual = wrench_actual


        #gets the gradient of the objective function 
        # (A vector of the derivative of the objective function with respect to
        # each of the 8 delta control inputs)
        objective_gradient = -wrench_actualJacobian @ K_Wrench @ wrenchError

        self.objectiveCounter += 1
        #returns the objective and the objective gradient
        return objective#TODO, objective_gradient
    
    #defines function to get wrench error
    def getWrenchError(self):
        return self.error


#creates the low level control class, where we first find the delta c 
# and then the delta t vertical vector after that
class LowLevelControl_successiveControl:

    #creates the initialization function
    def __init__(self,
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
        x0_delta_c = np.concatenate((self.delta_c_previous_solution, np.array([0.0, 0.0, 0.0, 0.0])))

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
    

#creates a low level control class, where we use the actual QuadDynamics file to
#implement the force and torque calculator. Then we can try to see how fishy things are
class LowLevelControl_reference:

    #creates the initialization function
    def __init__(self,
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


        #stores the time sample rate of the simulation parameters
        self.Ts = SIM.ts_control

        self.p_ctrl = PControl(kp=p_kp, Ts=self.Ts)
        self.q_ctrl = PControl(kp=q_kp, Ts=self.Ts)
        self.r_ctrl = PControl(kp=r_kp, Ts=self.Ts)
        self.output = MsgDelta()
        self.alpha = 0.99

        #store the quadDynamics object
        self.quad = QuadDynamics(ts=SIM.ts_simulation)

        #stores the previous solution for the delta c portion
        self.delta_c_previous_solution = CAP.init_plane_control

        #stores the previous solution for the delta r portion
        self.delta_t_previous_solution = CAP.init_quad_control

        #stores the whole previous solution
        self.previous_solution = np.ndarray((8,1))


        #creates the Error vector between the desired and the actual to see what's happening
        self.error = np.ndarray((5,0))

        #store the wrench actual
        self.wrenchActual = np.ndarray((5,1))




    #creates the update function
    def update(self, f_d: np.ndarray,#desired force 2x1 vector
                     omega_d: np.ndarray, #desired angular velocity 3x1 vector
                     state: MsgState, #Quad state
                     wind: np.ndarray, #the wind in the inertial frame
                     quad: QuadDynamics): #the quaddynamics object, which we will use to perform our dynamics calculations
        
        #stores the state
        self.state = state

        #stores the wind
        self.wind = wind

        #copies the quad 
        self.quad = copy.copy(quad)

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
        x0_delta_c = np.concatenate((self.delta_c_previous_solution, np.array([0.0, 0.0, 0.0, 0.0])))

        #calls the minimization function from the 
        delta_result = minimize(fun=self.objectiveFunction,
                                x0=x0_delta_c,
                                args=(wrenchDesired),
                                bounds=CAP.actuatorBounds_delta_c,
                                jac=False,
                                options={'maxiter': CAP.max_iter})
        

        deltaArray = delta_result.x
        #saves the previous solution
        self.previous_solution = deltaArray

        #gets the delta
        deltaFinal = MsgDelta()
        deltaFinal.from_array(deltaArray)

        #saves the wrench error
        wrenchError = wrenchDesired - self.wrenchActual
        self.error = np.concatenate((self.error, wrenchError), axis=1)        

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
        #TODO wrench_actual, wrench_actualJacobian = \
        #    self.wrenchCalculator.forces_moments_derivatives(delta=deltaMessage,
        #                                                     state=self.state)
        
        #calculates the fullwrenchActual (includes fy) from the copy of the quad dynamics file
        fullWrenchActual = self.quad._forces_moments(delta=deltaMessage)

        #shortens it to get rid of f_y, which we don't care about
        wrench_actual = np.ndarray((5,1))
        wrench_actual[0][0] = fullWrenchActual[0][0]
        wrench_actual[1][0] = fullWrenchActual[2][0]
        wrench_actual[2][0] = fullWrenchActual[3][0]
        wrench_actual[3][0] = fullWrenchActual[4][0]
        wrench_actual[4][0] = fullWrenchActual[5][0]

        #gets the wrench error
        wrenchError = wrenchDesired - wrench_actual

        #gets the objective, which is the magnitude of the wrench error,
        #with the scaling factor of the K_Tau matrix
        #(1x1) = (1x1) * (1x5) * (5x5) * (5x1)
        objective = 0.5 * wrenchError.T @ K_Wrench @ wrenchError
        
        #saves the actual wrench
        self.wrenchActual = wrench_actual


        #gets the gradient of the objective function 
        # (A vector of the derivative of the objective function with respect to
        # each of the 8 delta control inputs)
        #TODO objective_gradient = -wrench_actualJacobian @ K_Wrench @ wrenchError

        #returns the objective and the objective gradient
        return objective#TODO, objective_gradient
    
    #defines function to get wrench error
    def getWrenchError(self):
        return self.error



  
        
