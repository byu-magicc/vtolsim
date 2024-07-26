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

import parameters.quad.anaconda_parameters as QUAD

from tools.rotations import quaternion_to_rotation

import scipy.optimize as spo


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



        #defines the delta_a, e, and r bounds
        delta_a_bound = (-1.0, 1.0)
        delta_e_bound = (-1.0, 1.0)
        delta_r_bound = (-1.0, 1.0)
        

        #puts them together
        delta_c_bounds = (delta_a_bound, delta_e_bound, delta_r_bound)







    #creates function to get the forces and torques
    def getForcesTorques(self, state: MsgState, delta: MsgDelta)->np.ndarray:


        #gets the three control surfaces
        elevator = delta.elevator
        aileron = delta.aileron
        rudder = delta.rudder

        #gets the R matrix
        R = quaternion_to_rotation(state[6:10])

        #gets p, q, and r
        p = state.pos.item(0)
        q = state.pos.item(1)
        r = state.pos.item(2)

        #gets the gravitational force
        f_g = QUAD.mass * QUAD.gravity * R.T @ np.array([[0.], [0.], [1.]])

        #gets each portion of the gravitational force
        fx = f_g.item(0)
        fy = f_g.item(1)
        fz = f_g.item(2)
        #intermediate variables
        qbar = 0.5 * QUAD.rho * self._Va**2
        ca = np.cos(state.alpha)
        sa = np.sin(state.alpha)

        if self._Va > 1:
            p_nondim = p * QUAD.b / (2 * self._Va)  # nondimensionalize p
            q_nondim = q * QUAD.c / (2 * self._Va)  # nondimensionalize q
            r_nondim = r * QUAD.b / (2 * self._Va)  # nondimensionalize r
        else:
            p_nondim = 0.0
            q_nondim = 0.0
            r_nondim = 0.0

        # compute Lift and Drag coefficients
        tmp1 = np.exp(-QUAD.M * (state.alpha - QUAD.alpha0))
        tmp2 = np.exp(QUAD.M * (state.alpha + QUAD.alpha0))
        sigma = (1 + tmp1 + tmp2) / ((1 + tmp1) * (1 + tmp2))

        CL = (1 - sigma) * (QUAD.C_L_0 + QUAD.C_L_alpha * state.alpha) \
             + sigma * 2 * np.sign(state.alpha) * sa**2 * ca
        CD = (1 - sigma) \
            * (QUAD.C_D_p + \
               ((QUAD.C_L_0 + QUAD.C_L_alpha * state.alpha)**2)\
                /(np.pi * QUAD.e * QUAD.AR)) \
            + sigma * 2 * np.sign(state.alpha) * sa

        # compute Lift and Drag Forces
        F_lift = qbar * QUAD.S_wing * (CL + QUAD.C_L_q * q_nondim + QUAD.C_L_delta_e * elevator)
        F_drag = qbar * QUAD.S_wing * (CD + QUAD.C_D_q * q_nondim + QUAD.C_D_delta_e * elevator)

        # compute longitudinal forces in body frame
        fx += - ca * F_drag + sa * F_lift
        fz += - sa * F_drag - ca * F_lift
        # compute lateral forces in body frame
        fy += qbar * QUAD.S_wing * (
                  QUAD.C_Y_0
                + QUAD.C_Y_beta * self._beta
                + QUAD.C_Y_p * p_nondim
                + QUAD.C_Y_r * r_nondim
                + QUAD.C_Y_delta_a * aileron
                + QUAD.C_Y_delta_r * rudder)
        
        # compute logitudinal torque in body frame
        My = qbar * QUAD.S_wing * QUAD.c * (
                QUAD.C_m_0
                + QUAD.C_m_alpha * self._alpha
                + QUAD.C_m_q * q_nondim
                + QUAD.C_m_delta_e * elevator
        )
        # compute lateral torques in body frame
        Mx  = qbar * QUAD.S_wing * QUAD.b * (
                QUAD.C_ell_0
                + QUAD.C_ell_beta * self._beta
                + QUAD.C_ell_p * p_nondim
                + QUAD.C_ell_r * r_nondim
                + QUAD.C_ell_delta_a * aileron
                + QUAD.C_ell_delta_r * rudder
        )
        Mz = qbar * QUAD.S_wing * QUAD.b * (
                QUAD.C_n_0 + QUAD.C_n_beta * self._beta
                + QUAD.C_n_p * p_nondim
                + QUAD.C_n_r * r_nondim
                + QUAD.C_n_delta_a * aileron
                + QUAD.C_n_delta_r * rudder
        )
        

        #############################################################################################################
        #Forces and moments from the forward prop

        #gets the airspeed through the rear, main propulsion propellor
        Va_forward_prop = np.array([[1.0], [0.0], [0.0]]).T @ self.v_air

        #gets the thrust and the moment from the forward prop
        Thrust_Forward, Prop_Moment_Forward = self._motor_thrust_torque(Va_forward_prop, delta.forwardThrottle)

        #gets the force on the airplane from the forward propeller
        #In this case, it is in the positive x direction, so we multiply by the unit vector in the direction the thrust is facing
        Force_Forward = Thrust_Forward * np.array([[1.0], [0.0], [0.0]])

        #gets the moment, which is a little more complicated
        ##############################*******************************************************************************************
        #I may need to change this to account for the direction of the prop rotation
        #I am currently choosing it to rotate clockwise, when at the back, looking to the front of the airplane
        Moment_Forward = Prop_Moment_Forward*np.array([[1.0],[0.0],[0.0]]) + np.cross(QUAD.forward_rotor_pos.T, Force_Forward.T).T

        #adds each component to the whole forces and moment variables
        fx += Force_Forward.item(0)
        fy += Force_Forward.item(1)
        fz += Force_Forward.item(2)

        Mx += Moment_Forward.item(0)
        My += Moment_Forward.item(1)
        Mz += Moment_Forward.item(2)
        #############################################################################################################



        #############################################################################################################
        #Forces and moments from the rear port propeller
        Va_rear_port_prop = np.array([[0.0],[0.0],[-1.0]]).T @ self.v_air

        #gets the thrust and the moment from the rear port propeller
        Thrust_RearPort, Prop_Moment_RearPort = self._motor_thrust_torque(Va_rear_port_prop, delta.verticalThrottle_1)

        #gets the RearPort Force
        Force_RearPort = Thrust_RearPort*np.array([[0.0],[0.0],[-1.0]])

        #gets the rear port moment
        Moment_RearPort = Prop_Moment_RearPort*np.array([[0.0],[0.0],[-1.0]]) + np.cross(QUAD.vertical_rotor_1_pos.T, Force_RearPort.T).T

        #adds each component to the whole forces and moment variables
        fx += Force_RearPort.item(0)
        fy += Force_RearPort.item(1)
        fz += Force_RearPort.item(2)

        Mx += Moment_RearPort.item(0)
        My += Moment_RearPort.item(1)
        Mz += Moment_RearPort.item(2)
        #############################################################################################################

        #############################################################################################################
        #Forces and moments from the front port propeller
        Va_front_port_prop = np.array([[0.0],[0.0],[-1.0]]).T @ self.v_air

        #gets the thrust and the moment from the rear port propeller
        Thrust_FrontPort, Prop_Moment_FrontPort = self._motor_thrust_torque(Va_front_port_prop, delta.verticalThrottle_2)

        #gets the RearPort Force
        Force_FrontPort = Thrust_FrontPort*np.array([[0.0],[0.0],[-1.0]])

        #gets the rear port moment
        Moment_FrontPort = Prop_Moment_FrontPort*np.array([[0.0],[0.0],[-1.0]]) + np.cross(QUAD.vertical_rotor_2_pos.T, Force_FrontPort.T).T

        #adds each component to the whole forces and moment variables
        fx += Force_FrontPort.item(0)
        fy += Force_FrontPort.item(1)
        fz += Force_FrontPort.item(2)

        Mx += Moment_FrontPort.item(0)
        My += Moment_FrontPort.item(1)
        Mz += Moment_FrontPort.item(2)

        #############################################################################################################

        #############################################################################################################
        #Forces and moments from the front starboard propeller
        Va_front_starboard_prop = np.array([[0.0],[0.0],[-1.0]]).T @ self.v_air

        #gets the thrust and the moment from the rear port propeller
        Thrust_FrontStarboard, Prop_Moment_FrontStarboard = self._motor_thrust_torque(Va_front_starboard_prop, delta.verticalThrottle_3)

        #gets the RearStarboard Force
        Force_FrontStarboard = Thrust_FrontStarboard*np.array([[0.0],[0.0],[-1.0]])

        #gets the rear starboard moment
        Moment_FrontStarboard = Prop_Moment_FrontStarboard*np.array([[0.0],[0.0],[-1.0]]) + np.cross(QUAD.vertical_rotor_3_pos.T, Force_FrontStarboard.T).T

        #adds each component to the whole forces and moment variables
        fx += Force_FrontStarboard.item(0)
        fy += Force_FrontStarboard.item(1)
        fz += Force_FrontStarboard.item(2)

        Mx += Moment_FrontStarboard.item(0)
        My += Moment_FrontStarboard.item(1)
        Mz += Moment_FrontStarboard.item(2)

        #############################################################################################################

        #############################################################################################################
        #Forces and moments from the rear starboard propeller

        #Forces and moments from the front starboard propeller
        Va_rear_starboard_prop = np.array([[0.0],[0.0],[-1.0]]).T @ self.v_air

        #gets the thrust and the moment from the rear port propeller
        Thrust_RearStarboard, Prop_Moment_RearStarboard = self._motor_thrust_torque(Va_rear_starboard_prop, delta.verticalThrottle_4)

        #gets the RearStarboard Force
        Force_RearStarboard = Thrust_RearStarboard*np.array([[0.0],[0.0],[-1.0]])

        #gets the rear starboard moment
        Moment_RearStarboard = Prop_Moment_RearStarboard*np.array([[0.0],[0.0],[-1.0]]) + np.cross(QUAD.vertical_rotor_4_pos.T, Force_RearStarboard.T).T

        #adds each component to the whole forces and moment variables
        fx += Force_RearStarboard.item(0)
        fy += Force_RearStarboard.item(1)
        fz += Force_RearStarboard.item(2)

        Mx += Moment_RearStarboard.item(0)
        My += Moment_RearStarboard.item(1)
        Mz += Moment_RearStarboard.item(2)

        #############################################################################################################

        #returns the forces
        return np.array([[fx, fy, fz, Mx, My, Mz]]).T



    #gets the motor thrust and torque
    def _motor_thrust_torque(self, Va: float, delta_t: float)->tuple[float, float]:


        #gets the coefficients for the propeller

        C_Q0 = QUAD.C_Q0
        C_Q1 = QUAD.C_Q1
        C_T0 = QUAD.C_T0
        C_Q2 = QUAD.C_Q2
        C_T1 = QUAD.C_T1
        C_T2 = QUAD.C_T2
        D_prop = QUAD.D_prop
        KQ = QUAD.KQ
        R_motor = QUAD.R_motor
        i0 = QUAD.i0


        #gets the voltage in, based on the delta_t
        V_in = QUAD.V_max * delta_t
        # Quadratic formula to solve for motor speed
        a = C_Q0 * QUAD.rho * np.power(D_prop, 5) \
            / ((2.*np.pi)**2)
        b = (C_Q1 * QUAD.rho * np.power(D_prop, 4)
             / (2.*np.pi)) * Va + KQ**2/R_motor
        c = C_Q2 * QUAD.rho * np.power(D_prop, 3) \
            * Va**2 - (KQ / R_motor) * V_in + KQ * i0        
        # Consider only positive root
        Omega_op = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)
        # compute advance ratio
        J_op = 2 * np.pi * Va / (Omega_op * D_prop)
        # compute non-dimensionalized coefficients of thrust and torque
        C_T = C_T2 * J_op**2 + C_T1 * J_op + C_T0
        C_Q = C_Q2 * J_op**2 + C_Q1 * J_op + C_Q0
        # add thrust and torque due to propeller
        n = Omega_op / (2 * np.pi)
        T_p = QUAD.rho * n**2 * np.power(D_prop, 4) * C_T
        Q_p = QUAD.rho * n**2 * np.power(D_prop, 5) * C_Q
        return T_p, Q_p


