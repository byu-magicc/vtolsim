#This file implements the controller for the lower levels of the system
import numpy as np
from controllers.quad.pid_control import PControl

#imports the delta message type
from message_types.quad.msg_delta import MsgDelta
#imports the state message
from message_types.quad.msg_state import MsgState

import math

import parameters.quad.simulation_parameters as SIM

import parameters.quad.anaconda_parameters as QUAD

from tools.rotations import quaternion_to_rotation

import scipy.optimize as spo

import copy

#instantiates the dynamics for the quadplane
from models.quad.quad_dynamics import QuadDynamics

#saves the trim
from models.quad.trimValues import trimDelta


#creates the Low Level Control class for successive control, which
#FIRST finds the delta_c* and SECOND finds the delta_t*
class LowLevelControl_successiveControl:

    #creates the initialization function
    def __init__(self,
                 M: float=0.5,
                 Va0: float=0.0,
                 ts: float=0.01):
        

        #creates an instance of the Quad dynamics for the control piece
        self.quad = QuadDynamics(ts=SIM.ts_control)
        
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
        #self.output = MsgControls()
        self.output = MsgDelta()
        self.alpha = 0.99


        #saves the weights for the mixing matrix to mix the forces and torques for the error
        Fx_bar = 100.0
        Fy_bar = 100.0
        Fz_bar = 100.0
        Mx_bar = 50.0
        My_bar = 50.0
        Mz_bar = 50.0

        #creates the weighting matrix
        self.weightingMatrix = np.diag([1/Fx_bar, 1/Fy_bar, 1/Fz_bar, 1/Mx_bar, 1/My_bar, 1/Mz_bar])


        #creates the desired wrench vector
        self.wrenchDesired = np.ndarray((6,1))

        #defines the delta_a, e, and r and t forward bounds
        delta_a_bound = (-1.0, 1.0)
        delta_e_bound = (-1.0, 1.0)
        delta_r_bound = (-1.0, 1.0)
        delta_t_forward_bound = (0.0, 1.0)
        
        #puts them together
        self.delta_c_bounds = (delta_a_bound, delta_e_bound, delta_r_bound, delta_t_forward_bound)

        #defines the bounds for the vertical thrusters
        delta_t_vertical_1_bound = (0.0, 1.0)
        delta_t_vertical_2_bound = (0.0, 1.0)
        delta_t_vertical_3_bound = (0.0, 1.0)
        delta_t_vertical_4_bound = (0.0, 1.0)

        #puts these together
        self.delta_t_bounds = (delta_t_vertical_1_bound, delta_t_vertical_2_bound, delta_t_vertical_3_bound, delta_t_vertical_4_bound)


        #creates the state message
        self.state = MsgState()

        #creates the initial guess for the delta c portion
        self.x0_delta_c = (0.0, 0.0, 0.0, 0.5)

        #stores the delta c star
        self.delta_c_star_array = np.array((4,1))

        #creates the initial guess for the delta r portion
        self.x0_delta_r = (0.5, 0.5, 0.5, 0.5)

        #saves the wind
        self.wind = np.array([[0.0], [0.0], [0.0]])

        #saves an instance of the quad dynamics class
        self.quad = QuadDynamics(ts=ts)



    #creates the update function
    def update(self, f_d: np.ndarray,#desired force 2x1 vector
                     omega_d: np.ndarray, #desired angular velocity 3x1 vector
                     state: MsgState, #Quad state
                     quad: QuadDynamics,
                     wind: np.ndarray): #the wind in the inertial frame
        
        #saves a copy of the quad dynamics class being passed in
        self.quad = copy.copy(quad)

        #stores the state
        self.state = state

        #stores the wind
        self.wind = wind

        #gets the tau desired vector from the omega desired input and the actual omega
        tau_d = np.array([
            [self.p_ctrl.update(omega_d.item(0), state.omega.item(0))],
            [self.q_ctrl.update(omega_d.item(1), state.omega.item(1))],
            [self.r_ctrl.update(omega_d.item(2), state.omega.item(2))]
        ])

        #gets the wrench desired, which is the generalized version of forces and torques
        self.wrenchDesired = np.concatenate((f_d, tau_d), axis=0)


        #gets delta c star
        delta_c_solution = spo.minimize(fun=self.getWrenchSESurfaces, x0=self.x0_delta_c, method='Nelder-Mead', bounds=self.delta_c_bounds)

        self.delta_c_star_array = delta_c_solution.x

        #gets the full delta which minimizes error given a desired wrench,
        #as a length 8 array
        delta_t_solution = spo.minimize(fun=self.getWrenchSERotors, x0=self.x0_delta_r, method='Nelder-Mead', bounds=self.delta_t_bounds)

        delta_r_star_array = delta_t_solution.x

        #converts that array into a delta message
        deltaOutput = MsgDelta(elevator=self.delta_c_star_array[0],
                               aileron=self.delta_c_star_array[1],
                               rudder=self.delta_c_star_array[2],
                               forwardThrottle=self.delta_c_star_array[3],
                               verticalThrottle_1=delta_r_star_array[0],
                               verticalThrottle_2=delta_r_star_array[1],
                               verticalThrottle_3=delta_r_star_array[2],
                               verticalThrottle_4=delta_r_star_array[3])
        
        #returns the delta output
        return deltaOutput


    #creates function that gets the wrench squared error based on the
    #standard plane model, with just the control surfaces and the 
    #forward oriented throttle
    def getWrenchSESurfaces(self, delta_c_array: np.ndarray):

        #takes the delta array and converts it into a delta message
        deltaMessage = MsgDelta(elevator=delta_c_array[0],
                                aileron=delta_c_array[1],
                                rudder=delta_c_array[2],
                                forwardThrottle=delta_c_array[3])
        
        #gets the calculated wrench
        calculatedWrench = self.quad._forces_moments(delta=deltaMessage)

        #gets the wrench error
        wrenchError = self.wrenchDesired - calculatedWrench

        #gets the mean squared error using the weighting matrix
        MSError = (wrenchError.T @ self.weightingMatrix @ wrenchError)[0][0]

        #returns the Mean squared error
        return MSError
    
    #creates a function that gets the wrench squared error based on the
    #quadrotors
    def getWrenchSERotors(self, delta_r_array: np.ndarray):

        #takes the delta array and converts it into a delta message
        #all the while saving the delta_c_star, which has already been calculated 
        #for this particular iteration
        deltaMessage = MsgDelta(elevator=(self.delta_c_star_array)[0],
                                aileron=(self.delta_c_star_array)[0],
                                rudder=(self.delta_c_star_array)[0],
                                forwardThrottle=(self.delta_c_star_array)[0],
                                verticalThrottle_1=delta_r_array[0],
                                verticalThrottle_2=delta_r_array[1],
                                verticalThrottle_3=delta_r_array[2],
                                verticalThrottle_4=delta_r_array[3])
        
        #gets the calculated wrench
        calculatedWrench = self.quad._forces_moments(delta=deltaMessage)

        #gets the wrench error
        wrenchError = self.wrenchDesired - calculatedWrench

        #gets the mean squared error using the weighting matrix
        MSError = (wrenchError.T @ self.weightingMatrix @ wrenchError)[0][0]

        #returns the Mean squared error
        return MSError






#creates the unseperated low level control system, 
# which means we find the delta_c and delta_t at the same time
class LowLevelControl_simultaneousControl:

    #creates the init function
    def __init__(self,
                 M: float=0.5,
                 Va0: float=0.0,
                 ts: float=0.01):
        
        #creates an instance of the Quad dynamics for the control piece
        self.quad = QuadDynamics(ts=SIM.ts_control)
        
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
        #self.output = MsgControls()
        self.output = MsgDelta()
        self.alpha = 0.99


        #saves the weights for the mixing matrix to mix the forces and torques for the error
        Fx_bar = 100.0
        Fy_bar = 100.0
        Fz_bar = 100.0
        Mx_bar = 50.0
        My_bar = 50.0
        Mz_bar = 50.0

        #creates the weighting matrix
        self.weightingMatrix = np.diag([1/Fx_bar, 1/Fy_bar, 1/Fz_bar, 1/Mx_bar, 1/My_bar, 1/Mz_bar])


        #creates the desired wrench vector
        self.wrenchDesired = np.ndarray((6,1))

        #creates the bounds, which are all lumped together in this one
        #defines the delta_a, e, and r and t forward bounds
        delta_a_bound = (-1.0, 1.0)
        delta_e_bound = (-1.0, 1.0)
        delta_r_bound = (-1.0, 1.0)
        delta_t_forward_bound = (0.0, 1.0)
        #defines the bounds for the vertical thrusters
        delta_t_vertical_1_bound = (0.0, 1.0)
        delta_t_vertical_2_bound = (0.0, 1.0)
        delta_t_vertical_3_bound = (0.0, 1.0)
        delta_t_vertical_4_bound = (0.0, 1.0)

        #puts them together
        self.delta_bounds = (delta_a_bound, delta_e_bound, delta_r_bound, delta_t_forward_bound,\
                             delta_t_vertical_1_bound, delta_t_vertical_2_bound, delta_t_vertical_3_bound, delta_t_vertical_4_bound)

        #creates the state message
        self.state = MsgState()

        #creates the initial guess
        self.x0_delta = (0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5)

    #creates the update function
    def update(self,
               f_d: np.ndarray, #desired force vector
               omega_d: np.ndarray, #desired omega vector
               state: MsgState, #current state
               quad: QuadDynamics, #the whole quad state and dynamics
               wind: np.ndarray): #current wind conditions
        
        #stores the state in the self variable
        self.state = state

        #stores the quad dynamics copy
        self.quad = copy.copy(quad)

        #stores the wind in the wind vector
        self.wind = wind

        #gets the desired torques from the p, q, r controls using proportional control loop
        tau_d = np.array([[self.p_ctrl.update(omega_d.item(0), state.omega.item(0))],
                          [self.q_ctrl.update(omega_d.item(1), state.omega.item(1))],
                          [self.r_ctrl.update(omega_d.item(2), state.omega.item(2))]])
        

        #gets the wrench desired by concatenation
        self.wrenchDesired = np.concatenate((f_d, tau_d), axis=0)

        #gets the delta output
        delta_solution = spo.minimize(fun=self.getWrench, x0=self.x0_delta, method='Nelder-Mead', bounds=self.delta_bounds)

        #gets the delta array
        deltaArray = delta_solution.x

        #converts that array into a delta message
        deltaOutput = MsgDelta(elevator=deltaArray[0],
                               aileron=deltaArray[1],
                               rudder=deltaArray[2],
                               forwardThrottle=deltaArray[3],
                               verticalThrottle_1=deltaArray[4],
                               verticalThrottle_2=deltaArray[5],
                               verticalThrottle_3=deltaArray[6],
                               verticalThrottle_4=deltaArray[7])
        
        #returns the delta output array
        return deltaOutput


    #creates the function to get the wrench by modifying the deltas
    def getWrench(self, deltaArray: np.ndarray):
        #takes the delta array and creates a delta message class from it
        deltaMessage = MsgDelta(elevator=deltaArray[0],
                                aileron=deltaArray[1],
                                rudder=deltaArray[2],
                                forwardThrottle=deltaArray[3],
                                verticalThrottle_1=deltaArray[4],
                                verticalThrottle_2=deltaArray[5],
                                verticalThrottle_3=deltaArray[6],
                                verticalThrottle_4=deltaArray[7])

        #gets the calculated wrench
        calculatedWrench = self.quad._forces_moments(delta=deltaMessage)

        #gets the wrench error
        wrenchError = self.wrenchDesired - calculatedWrench

        #gets the mean squared error using the weighting matrix
        MSError = (wrenchError.T @ self.weightingMatrix @ wrenchError)[0][0]

        #returns the mean squared error
        return MSError


        


#creates the low level control system for just the delta_t
