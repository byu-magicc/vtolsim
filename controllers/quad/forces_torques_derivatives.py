#This file implements the functions that get the forces, and torques of the system,
#as well as the derivatives of those terms.

import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))

import sympy as sp

import numpy as np

import parameters.quad.anaconda_parameters as QUAD
from message_types.quad.msg_delta import MsgDelta
from message_types.quad.msg_state import MsgState


class wrenchCalculation:
    #creates the initialization function
    def __init__(self):

        #creates the counter
        self.counter = 0

    
    #function that returns the forces, moments, force derivatives, and moment derivatives
    #This will be useful for the jacobian
    #arguments: 
    #1. delta: MsgDelta class
    #2. state: MsgState class
    #Returns:
    #1. wrench Output, based on the current state
    #2. Wrench Jacobian, based on the delta inputs
    def forces_moments_derivatives(self, delta: MsgDelta, state: MsgState):




        #gets the whole state out
        pos = state.pos
        #gets the body frame velocity
        velocity = state.vel
        #gets the rotation matrix
        R = state.R
        #gets the rotational rates
        omega = state.omega

        #splits up the omega
        p = omega.item(0)
        q = omega.item(1)
        r = omega.item(2)


        #gets the individual variables in the state
        #airspeed
        Va = state.Va
        #gets the whole air velocity
        v_air = state.v_air
        #angle of attack
        alpha = state.alpha
        #sideslip angle
        beta = state.beta
        #course angle
        chi = state.chi

        #gets the airspeed relative to the rotors
        rotorAirspeed = np.array([v_air.item(0),
                                  -v_air.item(2),
                                  -v_air.item(2),
                                  -v_air.item(2),
                                  -v_air.item(2)])


        #creates the body frame forces and moments variables, which will be iteratively 
        #added to over the course of things
        fx = 0.0
        fz = 0.0

        Mx = 0.0
        My = 0.0
        Mz = 0.0



        #gets the gravitational force
        f_g = QUAD.mass * QUAD.gravity * R.T @ np.array([[0.], [0.], [1.]])

        #gets each portion of the gravitational force in the body frame
        #and adds it to creates the three body frame forces
        #will iteratively add to each of the components
        fg_x = f_g.item(0)
        fg_z = f_g.item(2)

        #adds these to the iterative body frame forces
        fx += fg_x
        fz += fg_z

        #gets the wrench achieved
        wrenchAchieved = self.force_torque_achieved(delta=delta, state=state)

        #adds the components of the wrench achieved to the total wrench
        fx += wrenchAchieved.item(0)
        fz += wrenchAchieved.item(1)
        Mx += wrenchAchieved.item(2)
        My += wrenchAchieved.item(3)
        Mz += wrenchAchieved.item(4)

        wrenchOutput = np.array([[fx],[fz],[Mx],[My],[Mz]])

        #creates breakpoint to see where things go.
        if (self.counter % 10 == 0):
            a = 0

        #gets the Jacobian for the output
        wrenchJacobian = self.wrenchJacobian(delta=delta, state=state)


        #increments the counter
        self.counter += 1

        return wrenchOutput, wrenchJacobian




    #creates a function that obtains the total forces and torques achieved
    def force_torque_achieved(self, delta: MsgDelta, state: MsgState):

        #calls the rotor thrust torque derivative function to get the thrust and torque achieved
        #creates the individual components for the forces and torques

        #body frame x force
        fx = 0.0
        fz = 0.0
        Mx = 0.0
        My = 0.0
        Mz = 0.0


        #this is just a list of each of the thrusts, not the body frame portions
        rotor_thrust, rotor_torque, rotor_thrust_der, rotor_torque_der =\
                     self.rotor_thrust_torque_der(delta=delta, state=state, dir=QUAD.propDirections)


        #############################################################################################################
        #Forces and moments from the forward prop


        #iterates through and adds the resultant moments and forces to the total forces and moments
        for i in range(QUAD.num_rotors):

            #gets the forward force
            Force = rotor_thrust[i]*QUAD.normalVectors[i]

            #gets the propeller moment forward
            Prop_Moment_Forward = rotor_torque[i]*QUAD.normalVectors[i]

            #gets the total moment
            Moment = Prop_Moment_Forward + rotor_thrust[i]*QUAD.leverMoments[i]

            fx += Force.item(0)
            fz += Force.item(2)

            Mx += Moment.item(0)
            My += Moment.item(1)
            Mz += Moment.item(2)

        if self.counter % 10 == 0:
            a = 0

        #gets the aerodynamic forces and torques
        #these are already in body frame
        aero_forces, aero_torques = self.aerodynamic_force_torque(delta=delta, state=state)


        #splits up the aerodynamic forces into the various components 
        fx_aerodynamic = aero_forces.item(0)
        fz_aerodynamic = aero_forces.item(2)
        #gets the aerodynamic moments
        Mx_aerodynamic = aero_torques.item(0)
        My_aerodynamic = aero_torques.item(1)
        Mz_aerodynamic = aero_torques.item(2)

        #adds the aerodynamic forces and moments to the total forces and moments
        fx += fx_aerodynamic
        fz += fz_aerodynamic
        Mx += Mx_aerodynamic
        My += My_aerodynamic
        Mz += Mz_aerodynamic



        #creates the wrench return vector
        wrench_achieved = np.array([[fx],[fz],[Mx],[My],[Mz]])
        return wrench_achieved


    #creates a function that obtains the Jacobian of the wrench
    def wrenchJacobian(self, delta: MsgDelta, state: MsgState):
        #gets  the constant scaling gamma coefficient
        Gamma = (1/2)*QUAD.rho * ((state.Va)**2) * QUAD.S_wing

        alpha = state.alpha

        alphaRotation = np.array([[-np.cos(alpha), np.sin(alpha)],
                                  [-np.sin(alpha), -np.cos(alpha)]])

        LiftDragVec = np.array([[QUAD.C_D_delta_e],[QUAD.C_L_delta_e]])

        elevatorForceCoefficients = Gamma*alphaRotation @ LiftDragVec

        #gets the forces, moments and derivatives for the individual rotors
        rotor_forces, rotor_moments, rotor_force_ders, rotor_moment_ders =\
                                     self.rotor_thrust_torque_der(delta=delta, state=state, dir=QUAD.propDirections)

        #gets the Fx gradient
        Fx_Gradient = [elevatorForceCoefficients[0][0],#delta e
                       0,#delta a
                       0,#delta r
                       QUAD.forward_rotor_normal[0][0]*rotor_force_ders[0],#delta tf
                       QUAD.vertical_rotor_1_normal[0][0]*rotor_force_ders[1],#delta t v1
                       QUAD.vertical_rotor_2_normal[0][0]*rotor_force_ders[2],#delta t v2
                       QUAD.vertical_rotor_3_normal[0][0]*rotor_force_ders[3],#delta t v3
                       QUAD.vertical_rotor_4_normal[0][0]*rotor_force_ders[4]#delta t v4
                       ]

        #gets the Fz gradient
        Fz_Gradient = [elevatorForceCoefficients[1][0],#delta e
                       0,#delta a
                       0,#delta r
                       QUAD.forward_rotor_normal[2][0]*rotor_force_ders[0],#delta tf,#delta t forward
                       QUAD.vertical_rotor_1_normal[2][0]*rotor_force_ders[1],#delta t v1
                       QUAD.vertical_rotor_2_normal[2][0]*rotor_force_ders[2],#delta t v2
                       QUAD.vertical_rotor_3_normal[2][0]*rotor_force_ders[3],#delta t v3
                       QUAD.vertical_rotor_4_normal[2][0]*rotor_force_ders[4]#delta t v4
                       ]

        #gets the Mx gradient
        Mx_Gradient = [0.0,#delta e
                       Gamma*QUAD.b*QUAD.C_ell_delta_a,
                       Gamma*QUAD.b*QUAD.C_ell_delta_r,
                       QUAD.forward_rotor_normal[2][0]*rotor_moment_ders[0],
                       QUAD.leverMomentV1[0][0]*rotor_force_ders[1],
                       QUAD.leverMomentV2[0][0]*rotor_force_ders[2],
                       QUAD.leverMomentV3[0][0]*rotor_force_ders[3],
                       QUAD.leverMomentV4[0][0]*rotor_force_ders[4]
                       ]

        My_Gradient = [Gamma * QUAD.c * QUAD.C_m_delta_e,
                       0,
                       0,
                       0.0,
                       QUAD.leverMomentV1[1][0]*rotor_force_ders[1],
                       QUAD.leverMomentV2[1][0]*rotor_force_ders[2],
                       QUAD.leverMomentV3[1][0]*rotor_force_ders[3],
                       QUAD.leverMomentV4[1][0]*rotor_force_ders[4]
                       ]

        Mz_Gradient = [0.0,
                       Gamma*QUAD.b*QUAD.C_n_delta_a,
                       Gamma*QUAD.b*QUAD.C_n_delta_r,
                       0.0,
                       QUAD.vertical_rotor_1_normal[2][0]*rotor_moment_ders[1],
                       QUAD.vertical_rotor_2_normal[2][0]*rotor_moment_ders[2],
                       QUAD.vertical_rotor_3_normal[2][0]*rotor_moment_ders[3],
                       QUAD.vertical_rotor_4_normal[2][0]*rotor_moment_ders[4]]

        #returns the complete gradient, which we need to transpose for it to be accurate
        return np.array([Fx_Gradient, Fz_Gradient, Mx_Gradient, My_Gradient, Mz_Gradient]).T

    #function that gets the thrust, moment, thrust derivative, and moment derivative of the system
    #Arguments:
    #1. delta_t: vector of throttle inputs (0.0, 1.0) for the individual rotor
    #2. Va: vector of the airspeeds through the various rotors
    #3. dir: the direction of rotation for each of the rotors
    #Returns:
    #1. thrust: vector of all 5 thrusts
    #2. torque: vector of all 5 moments
    #3. thrust_der: vector of all 5 thrust derivatives
    #4. torque_der: vector of all 5 moment derivatives

    #Order: 1. Forward, 2. Port Front, 3. Port Rear, 4 Starboard Rear, 5 Starboard Front
    def rotor_thrust_torque_der(self, delta: MsgDelta, state: MsgState, dir: np.ndarray)->tuple[list, list, list, list]:
        #gets the air velocity from the state
        v_air = state.v_air

        #gets the v_air in the different directions
        v_air_x = v_air[0][0]
        v_air_z = v_air[2][0]

        #gets the v_air from the rotor perspective
        v_air_rotors = np.array([v_air_x,
                                 -v_air_z,
                                 -v_air_z,
                                 -v_air_z,
                                 -v_air_z])


        #obtains the five delta throttles and fixes things
        delta_t = np.array([delta.forwardThrottle,
                            delta.verticalThrottle_1,
                            delta.verticalThrottle_2,
                            delta.verticalThrottle_3,
                            delta.verticalThrottle_4])

        thrust = list()
        torque = list()
        thrust_der = list()
        torque_der = list()
        for i in range(5):
            # compute thrust and torque due to propeller  (See addendum by McLain)
            # grab motor/prop params
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
            # map delta_t throttle command(0 to 1) into motor input voltage
            V_in = QUAD.V_max * delta_t[i]
            V_in_der = QUAD.V_max
            # Quadratic formula to solve for motor speed
            a = C_Q0 * QUAD.rho * np.power(D_prop, 5) \
                / ((2.*np.pi)**2)
            b = (C_Q1 * QUAD.rho * np.power(D_prop, 4)
                / (2.*np.pi)) * v_air_rotors[i] + KQ**2/R_motor
            c = C_Q2 * QUAD.rho * np.power(D_prop, 3) \
                * (v_air_rotors[i])**2 - (KQ / R_motor) * V_in + KQ * i0
            c_der = (KQ / R_motor) * V_in_der
            # Consider only positive root
            Omega_op = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)
            Omega_op_der = c_der / np.sqrt(b**2 - 4*a*c)
            # compute advance ratio
            J_op = 2 * np.pi * v_air_rotors[i] / (Omega_op * D_prop)
            J_op_der = -2 * np.pi * v_air_rotors[i] * Omega_op_der / (Omega_op**2 * D_prop)
            # compute non-dimensionalized coefficients of thrust and torque
            C_T = C_T2 * J_op**2 + C_T1 * J_op + C_T0
            C_Q = C_Q2 * J_op**2 + C_Q1 * J_op + C_Q0
            C_T_der = 2 * C_T2 * J_op * J_op_der + C_T1 * J_op_der
            C_Q_der = 2 * C_Q2 * J_op * J_op_der + C_Q1 * J_op_der
            # add thrust and torque due to propeller
            n = Omega_op / (2 * np.pi)
            T_p = QUAD.rho * n**2 * np.power(D_prop, 4) * C_T
            Q_p = QUAD.rho * n**2 * np.power(D_prop, 5) * C_Q
            T_p_der = QUAD.rho * Omega_op * Omega_op_der * np.power(D_prop, 4) * C_T / (2 * np.pi**2) + \
                QUAD.rho * Omega_op**2 * np.power(D_prop, 4) * C_T_der / (2 * np.pi)**2
            Q_p_der = QUAD.rho * Omega_op * Omega_op_der * np.power(D_prop, 5) * C_Q / (2 * np.pi**2) + \
                QUAD.rho * Omega_op**2 * np.power(D_prop, 5) * C_Q_der / (2 * np.pi)**2

            thrust.append(T_p)
            torque.append(Q_p)
            thrust_der.append(T_p_der)
            torque_der.append(Q_p_der)

        return thrust, torque, thrust_der, torque_der


    #function that gets the total force and torque achieved
    def aerodynamic_force_torque(self, delta: MsgDelta, state: MsgState)->tuple[np.ndarray, np.ndarray]:


        #gets the three control surfaces
        elevator = delta.elevator
        aileron = delta.aileron
        rudder = delta.rudder

        #gets the whole state out
        pos = state.pos
        #gets the body frame velocity
        velocity = state.vel
        #gets the rotation matrix
        R = state.R
        #gets the rotational rates
        omega = state.omega

        #splits up the omega
        p = omega.item(0)
        q = omega.item(1)
        r = omega.item(2)


        #gets the individual variables in the state
        #airspeed
        Va = state.Va
        #gets the whole air velocity
        v_air = state.v_air
        #angle of attack
        alpha = state.alpha
        #sideslip angle
        beta = state.beta
        #course angle
        chi = state.chi

        #intermediate variables
        qbar = 0.5 * QUAD.rho * Va**2
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        if Va > 1:
            p_nondim = p * QUAD.b / (2 * Va)  # nondimensionalize p
            q_nondim = q * QUAD.c / (2 * Va)  # nondimensionalize q
            r_nondim = r * QUAD.b / (2 * Va)  # nondimensionalize r
        else:
            p_nondim = 0.0
            q_nondim = 0.0
            r_nondim = 0.0

        # compute Lift and Drag coefficients
        tmp1 = np.exp(-QUAD.M * (alpha - QUAD.alpha0))
        tmp2 = np.exp(QUAD.M * (alpha + QUAD.alpha0))
        sigma = (1 + tmp1 + tmp2) / ((1 + tmp1) * (1 + tmp2))

        CL = (1 - sigma) * (QUAD.C_L_0 + QUAD.C_L_alpha * alpha) \
             + sigma * 2 * np.sign(alpha) * sa**2 * ca
        CD = QUAD.C_D_p + ((QUAD.C_L_0 + QUAD.C_L_alpha * alpha)**2)/(np.pi * QUAD.e * QUAD.AR)
    
        # compute Lift and Drag Forces in the inertial frame
        F_lift = qbar * QUAD.S_wing * (CL + QUAD.C_L_q * q_nondim + QUAD.C_L_delta_e * elevator)
        F_drag = qbar * QUAD.S_wing * (CD + QUAD.C_D_q * q_nondim + QUAD.C_D_delta_e * elevator)

        # compute longitudinal forces in body frame
        fx_aerodynamic = - ca * F_drag + sa * F_lift
        fz_aerodynamic = - sa * F_drag - ca * F_lift
        # compute lateral forces in body frame
        fy_aerodynamic = qbar * QUAD.S_wing * (
                                QUAD.C_Y_0
                              + QUAD.C_Y_beta * beta
                              + QUAD.C_Y_p * p_nondim
                              + QUAD.C_Y_r * r_nondim
                              + QUAD.C_Y_delta_a * aileron
                              + QUAD.C_Y_delta_r * rudder)

        # compute logitudinal torque in body frame
        My_aerodynamic = qbar * QUAD.S_wing * QUAD.c * (
                QUAD.C_m_0
                + QUAD.C_m_alpha * alpha
                + QUAD.C_m_q * q_nondim
                + QUAD.C_m_delta_e * elevator)
        # compute lateral torques in body frame
        Mx_aerodynamic  = qbar * QUAD.S_wing * QUAD.b * (
                QUAD.C_ell_0
                + QUAD.C_ell_beta * beta
                + QUAD.C_ell_p * p_nondim
                + QUAD.C_ell_r * r_nondim
                + QUAD.C_ell_delta_a * aileron
                + QUAD.C_ell_delta_r * rudder)
        Mz_aerodynamic = qbar * QUAD.S_wing * QUAD.b * (
                QUAD.C_n_0 + QUAD.C_n_beta * beta
                + QUAD.C_n_p * p_nondim
                + QUAD.C_n_r * r_nondim
                + QUAD.C_n_delta_a * aileron
                + QUAD.C_n_delta_r * rudder)

        #gets the forces and moments in the body frame
        forcesBodyFrame = np.array([[fx_aerodynamic],[fy_aerodynamic],[fz_aerodynamic]])
        momentsBodyFrame = np.array([[Mx_aerodynamic],[My_aerodynamic],[Mz_aerodynamic]])

        if self.counter % 10 == 0:
            a = 0



        return forcesBodyFrame, momentsBodyFrame


    #function that converts delta message to delta array
    def delta_message_to_array(self, deltaMessage: MsgDelta)->np.ndarray:

        temp = np.array([deltaMessage.elevator,
                         deltaMessage.aileron,
                         deltaMessage.rudder,
                         deltaMessage.forwardThrottle,
                         deltaMessage.verticalThrottle_1,
                         deltaMessage.verticalThrottle_2,
                         deltaMessage.verticalThrottle_3,
                         deltaMessage.verticalThrottle_4])
        
        return temp
    
    #function that converts delta array to delta message
    def delta_array_to_message(self, deltaArray: np.ndarray)->MsgDelta:

        deltaMessage = MsgDelta(elevator=deltaArray.item(0),
                                aileron=deltaArray.item(1),
                                rudder=deltaArray.item(2),
                                forwardThrottle=deltaArray.item(3),
                                verticalThrottle_1=deltaArray.item(4),
                                verticalThrottle_2=deltaArray.item(5),
                                verticalThrottle_3=deltaArray.item(6),
                                verticalThrottle_4=deltaArray.item(7))
        
        return deltaMessage
