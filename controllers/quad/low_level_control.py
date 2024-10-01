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

from controllers.quad.JacobianCalculation import getJacobians

#instantiates the dynamics for the quadplane
from models.quad.quad_dynamics import QuadDynamics

#saves the trim
from models.quad.trimValues import trimDelta

import time


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
        
        #creates the state message
        self.state = MsgState()

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

        #creates the initial guess for the delta c portion
        self.x0_delta_c = (0.0, 0.0, 0.0, 0.5)

        #stores the delta c star
        self.delta_c_star_array = np.array((4,1))

        #creates the initial guess for the delta r portion
        self.x0_delta_t = (0.5, 0.5, 0.5, 0.5)

        #saves the wind
        self.wind = np.array([[0.0], [0.0], [0.0]])


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
        delta_t_solution = spo.minimize(fun=self.getWrenchSERotors, x0=self.x0_delta_t, method='Nelder-Mead', bounds=self.delta_t_bounds)

        delta_t_star_array = delta_t_solution.x

        #converts that array into a delta message
        deltaOutput = MsgDelta(elevator=self.delta_c_star_array[0],
                               aileron=self.delta_c_star_array[1],
                               rudder=self.delta_c_star_array[2],
                               forwardThrottle=self.delta_c_star_array[3],
                               verticalThrottle_1=delta_t_star_array[0],
                               verticalThrottle_2=delta_t_star_array[1],
                               verticalThrottle_3=delta_t_star_array[2],
                               verticalThrottle_4=delta_t_star_array[3])
        
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
    def getWrenchSERotors(self, delta_t_array: np.ndarray):

        #takes the delta array and converts it into a delta message
        #all the while saving the delta_c_star, which has already been calculated 
        #for this particular iteration
        deltaMessage = MsgDelta(elevator=(self.delta_c_star_array)[0],
                                aileron=(self.delta_c_star_array)[0],
                                rudder=(self.delta_c_star_array)[0],
                                forwardThrottle=(self.delta_c_star_array)[0],
                                verticalThrottle_1=delta_t_array[0],
                                verticalThrottle_2=delta_t_array[1],
                                verticalThrottle_3=delta_t_array[2],
                                verticalThrottle_4=delta_t_array[3])
        
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


        #creates the weighting submatrices
        self.Q_F = np.diag([1/Fx_bar, 1/Fy_bar, 1/Fz_bar])
        self.Q_M = np.diag([1/Mx_bar, 1/My_bar, 1/Mz_bar])

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


        #creates the array to store the completion time

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
        delta_solution = spo.minimize(fun=self.getWrench, 
                                      x0=self.x0_delta, 
                                      method='Nelder-Mead', 
                                      bounds=self.delta_bounds)
        
        #sets the x0 guess to the current delta solution
        self.x0_delta = delta_solution.x

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


    #######TODO########## I need to fix this
    #creates a function to compute the objective gradient
    def getObjectiveGradient(self, delta_in: np.ndarray):
        

        #converts the delta_in as an 8x1 array to a delta message
        deltaMessage = MsgDelta(elevator=delta_in[0],
                                aileron=delta_in[1],
                                rudder=delta_in[2],
                                forwardThrottle=delta_in[3],
                                verticalThrottle_1=delta_in[4],
                                verticalThrottle_2=delta_in[5],
                                verticalThrottle_3=delta_in[6],
                                verticalThrottle_4=delta_in[7])

        #gets the groundspeed vector in the body frame
        groundspeed = self.state.vel
        #first calculates the airspeed
        airspeed = groundspeed - self.wind[0:3]

        #gets the Jacobians
        F_Jacobian, M_Jacobian = getJacobians(delta=deltaMessage, state=self.state, airspeed=airspeed)

        #gets the desired forces and torques
        F_d = self.wrenchDesired[1:3]
        M_d = self.wrenchDesired[3:6]

        #gets the forces and torques based on the delta input
        wrenchActual = self.quad._forces_moments(delta=deltaMessage)
        F_actual = wrenchActual[1:3]
        M_actual = wrenchActual[3:6]

        #gets the whole gradient objective
        gr_obj = F_Jacobian*2*self.Q_F*(F_actual-F_d) + M_Jacobian*2*self.Q_M*(M_actual - M_d)

        return gr_obj




        
#creates the low level control system for just the vertical delta t's
class LowLevelControl_vertical:

#creates the initialization function
    def __init__(self,
                 M: float=0.5,
                 Va0: float=0.0,
                 ts: float=0.01):
        
        #creates an instance of the Quad dynamics for the control piece
        self.quad = QuadDynamics(ts=SIM.ts_control)

        #creates the state message
        self.state = MsgState()
        
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

        #defines the bounds for the vertical thrusters
        delta_t_vertical_1_bound = (0.0, 1.0)
        delta_t_vertical_2_bound = (0.0, 1.0)
        delta_t_vertical_3_bound = (0.0, 1.0)
        delta_t_vertical_4_bound = (0.0, 1.0)

        #puts these together
        self.delta_t_bounds = (delta_t_vertical_1_bound, delta_t_vertical_2_bound, delta_t_vertical_3_bound, delta_t_vertical_4_bound)

        #creates the initial guess for the delta r portion
        self.x0_delta_t = (0.5, 0.5, 0.5, 0.5)

        #saves the wind
        self.wind = np.array([[0.0], [0.0], [0.0]])

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
        tau_d = np.array([[self.p_ctrl.update(omega_d.item(0), state.omega.item(0))],
                          [self.q_ctrl.update(omega_d.item(1), state.omega.item(1))],
                          [self.r_ctrl.update(omega_d.item(2), state.omega.item(2))]])

        #gets the wrench desired, which is the generalized version of forces and torques
        self.wrenchDesired = np.concatenate((f_d, tau_d), axis=0)

        #gets the delta solution
        delta_t_solution = spo.minimize(fun=self.getWrench, x0=self.x0_delta_t, method='Nelder-Mead', bounds=self.delta_t_bounds)
        delta_t_star_array = delta_t_solution.x

        #converts that array into a delta message
        deltaOutput = MsgDelta(verticalThrottle_1=delta_t_star_array[0],
                               verticalThrottle_2=delta_t_star_array[1],
                               verticalThrottle_3=delta_t_star_array[2],
                               verticalThrottle_4=delta_t_star_array[3])
        
        #returns the delta output
        return deltaOutput
    
    #creates the get wrench function
    def getWrench(self, delta_t: np.ndarray):

        #creates the delta message
        deltaMessage = MsgDelta(verticalThrottle_1=delta_t[0],
                                verticalThrottle_2=delta_t[1],
                                verticalThrottle_3=delta_t[2],
                                verticalThrottle_4=delta_t[3])
        
        #gets the calculated wrench
        calculatedWrench = self.quad._forces_moments(delta=deltaMessage)

        #gets the wrench error
        wrenchError = self.wrenchDesired - calculatedWrench

        #gets the mean squared error using the weighting matrix
        MSError = (wrenchError.T @ self.weightingMatrix @ wrenchError)[0][0]

        #returns the Mean squared error
        return MSError
    

#low Level controller for control surfaces
class LowLevelControl_Surfaces:

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
        self.weightingMatrix = np.diag([1/(Fx_bar**2), 1/(Fy_bar**2), 1/(Fz_bar**2), 1/(Mx_bar**2), 1/(My_bar**2), 1/(Mz_bar**2)])


        #creates the desired wrench vector
        self.wrenchDesired = np.ndarray((6,1))

        #creates the bounds, which are all lumped together in this one
        #defines the delta_a, e, and r and t forward bounds
        delta_e_bound = (-1.0, 1.0)
        delta_a_bound = (-1.0, 1.0)
        delta_r_bound = (-1.0, 1.0)
        delta_t_forward_bound = (0.0, 1.0)

        #puts them together
        self.delta_c_bounds = (delta_e_bound, delta_a_bound, delta_r_bound, delta_t_forward_bound)


        #creates the initial guess for the delta c portion
        self.x0_delta_c = (0.0, 0.0, 0.0, 0.5)

        #saves the wind
        self.wind = np.array([[0.0], [0.0], [0.0]])

    #creates the update function
    def update(self, f_d: np.ndarray,#desired force 2x1 vector, Fx, Fz desired
                     state: MsgState, #Quad state
                     quad: QuadDynamics, #passes in instance of the quadplane dynamics class
                     tau_input: bool = False,#passes in input to set whether we are being input straight torque, or omegas and then need to calculate torques
                     wind: np.ndarray = np.zeros((3,1)), #the wind in the inertial frame
                     omega_d: np.ndarray = np.zeros((2,1)), #desired angular velocity p, and q
                     tau_d: np.ndarray = np.zeros((2,1))):# desired torque
        
        #saves a copy of the quad dynamics class being passed in
        self.quad = copy.copy(quad)

        #stores the state
        self.state = state

        #stores the wind
        self.wind = wind

        #creates the desired tau
        tau_desired = np.zeros((3,1))

        #case tau directly input
        if tau_input:
            tau_desired = tau_d
        #case omegas to tau
        else:
            #gets the tau desired vector from the omega desired input and the actual omega
            tau_desired = np.array([[self.p_ctrl.update(omega_d.item(0), state.omega.item(0))],
                                    [self.q_ctrl.update(omega_d.item(1), state.omega.item(1))],
                                    [self.r_ctrl.update(omega_d.item(2), state.omega.item(2))]])

        #gets the wrench desired, which is the generalized version of forces and torques
        self.wrenchDesired = np.concatenate((f_d, tau_desired), axis=0)

        #gets the delta c solution
        delta_c_solution = spo.minimize(fun=self.getWrench, x0=self.x0_delta_c, method='Nelder-Mead', bounds=self.delta_c_bounds)

        delta_c_array = delta_c_solution.x

        deltaOutput = MsgDelta(elevator=delta_c_array[0],
                               aileron=delta_c_array[1],
                               rudder=delta_c_array[2],
                               forwardThrottle=delta_c_array[3])


        return deltaOutput
    

    #creates the get wrench function
    def getWrench(self, delta_c_array: np.ndarray):

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

        #returns the mean squared error
        return MSError
    

#creates the shortened low level controller for control surfaces only
#Controls: Fx, Fz, Mx, and My, because we are not controlling in the y direction
class LowLevelControl_SurfacesShortened:

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
        #self.output = MsgControls()
        self.output = MsgDelta()
        self.alpha = 0.99



        #saves the weights for the mixing matrix to mix the forces and torques for the error
        Fx_bar = 100.0
        Fz_bar = 100.0
        Mx_bar = 50.0
        My_bar = 50.0

        #creates the weighting matrix
        self.weightingMatrix = np.diag([1/(Fx_bar**2), 1/(Fz_bar**2), 1/(Mx_bar**2), 1/(My_bar**2)])

        #creates the desired wrench vector
        self.wrenchDesired = np.ndarray((4,1))

        #creates the bounds
        delta_e_bound = (-1.0, 1.0)
        delta_a_bound = (-1.0, 1.0)
        delta_t_forward_bound = (0.0, 1.0)


        self.delta_c_bounds = (delta_e_bound, delta_a_bound, delta_t_forward_bound)

        #creates the initial guess for the delta c portion
        self.x0_delta_c = (0.0, 0.0, 0.5)

        #saves the wind
        self.wind = np.array([[0.0], [0.0], [0.0]])


    #creates the update function
    def update(self, f_d: np.ndarray,#desired force 2x1 vector, Fx, Fz desired
                     state: MsgState, #Quad state
                     quad: QuadDynamics, #passes in instance of the quadplane dynamics class
                     tau_input: bool = False,#passes in input to set whether we are being input straight torque, or omegas and then need to calculate torques
                     wind: np.ndarray = np.zeros((3,1)), #the wind in the inertial frame
                     omega_d: np.ndarray = np.zeros((2,1)), #desired angular velocity p, and q
                     tau_d: np.ndarray = np.zeros((2,1))):# desired torque

        
        #saves a copy of the quad dynamics class being passed in
        self.quad = copy.copy(quad)

        #stores the state
        self.state = state

        #stores the wind
        self.wind = wind

        tau_desired = np.zeros((2,1))

        #chooses whether to take the input tau or the input omegas
        #case tau input directly
        if tau_input:
            tau_desired = tau_d

        #case input omegas and use p control
        else:
            #gets the tau desired vector from the omega desired input and the actual omega
            tau_desired = np.array([
                [self.p_ctrl.update(omega_d.item(0), state.omega.item(0))],
                [self.q_ctrl.update(omega_d.item(1), state.omega.item(1))]])




        #gets the wrench desired
        self.wrenchDesired = np.concatenate((f_d, tau_desired), axis=0)

        #gets the solution
        delta_c_solution = spo.minimize(fun=self.getWrench, x0=self.x0_delta_c, method='Nelder-Mead', bounds=self.delta_c_bounds)

        delta_c_array = delta_c_solution.x

        deltaOutput = MsgDelta(elevator=delta_c_array[0],
                               aileron=delta_c_array[1],
                               forwardThrottle=delta_c_array[2])


        return deltaOutput


    #creates the get wrench function
    def getWrench(self, delta_c_array: np.ndarray):

        #takes the delta array and converts it into a delta message
        deltaMessage = MsgDelta(elevator=delta_c_array[0],
                                aileron=delta_c_array[1],
                                forwardThrottle=delta_c_array[2])

        #gets the calculated wrench
        calculatedWrench = self.quad._forces_moments(delta=deltaMessage)

        shortenedCalculatedWrench = np.array([[calculatedWrench[0][0]],
                                              [calculatedWrench[2][0]],
                                              [calculatedWrench[3][0]],
                                              [calculatedWrench[4][0]]])

        #gets the wrench error
        wrenchError = self.wrenchDesired - shortenedCalculatedWrench

        #gets the mean squared error using the weighting matrix
        MSError = (wrenchError.T @ self.weightingMatrix @ wrenchError)[0][0]

        #returns the mean squared error
        return MSError
    






