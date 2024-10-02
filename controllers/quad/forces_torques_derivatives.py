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



#function that returns the forces, moments, force derivatives, and moment derivatives
#This will be useful for the jacobian

def forces_moments_derivatives(delta: MsgDelta, state: MsgState):


    #gets the three control surfaces
    elevator = delta.elevator
    aileron = delta.aileron
    rudder = delta.rudder

    delta_c = np.array([elevator, aileron, rudder])

    #gets the 5 rotor throttles
    forward_delta_t = delta.forwardThrottle

    #4 vertical deltas
    vertical_1_delta_t = delta.verticalThrottle_1
    vertical_2_delta_t = delta.verticalThrottle_2
    vertical_3_delta_t = delta.verticalThrottle_3
    vertical_4_delta_t = delta.verticalThrottle_4

    #creates a vector of the delta_t
    delta_t = np.array([forward_delta_t, 
                        vertical_1_delta_t, 
                        vertical_2_delta_t, 
                        vertical_3_delta_t,
                        vertical_4_delta_t])
    


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
    fy = 0.0
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
    fg_y = f_g.item(1)
    fg_z = f_g.item(2)

    #adds these to the iterative body frame forces
    fx += fg_x
    fy += fg_y
    fz += fg_z

    forcesAerodynamic, momentsAerodynamic, aerodynamic_force_der, aerodynamic_torque_der =  \
                    aerodynamic_force_torque_der(delta=delta_c, state=state)


    #adds the aerodynamic forces and moments to the iterative variables
    fx_aerodynamic = forcesAerodynamic[0][0]
    fy_aerodynamic = forcesAerodynamic[1][0]
    fz_aerodynamic = forcesAerodynamic[2][0]

    Mx_aerodynamic = momentsAerodynamic[0][0]
    My_aerodynamic = momentsAerodynamic[1][0]
    Mz_aerodynamic = momentsAerodynamic[2][0]





    #calls the function for the Forces moments and derivatives function for each of the Rotors
    motorThrusts, motorTorques, motorThrustsDer, motorTorquesDer = \
                        rotor_thrust_torque_der(delta=delta_t, 
                                                v_air=rotorAirspeed,
                                                dir=QUAD.propDirections)
    

    #adds the thrusts and torques to the body frame forces and torques
    #first adds the forward propeller force
    fx += motorThrusts[0]
    Mx += motorTorques[0]

    #adds the thrusts from the 
    #needs to subtract because the forces are in the -z direction, same with the torques as a convention
    fz = fz - (motorThrusts[1] + motorThrusts[2] + motorThrusts[3] + motorThrusts[4])
    Mz = Mz - (motorTorques[1] + motorTorques[2] + motorTorques[3] + motorTorques[4])














    








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
def rotor_thrust_torque_der(delta: np.ndarray, v_air: np.ndarray, dir: np.ndarray)->tuple[list, list, list, list]:
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
        V_in = QUAD.V_max * delta[i]
        V_in_der = QUAD.V_max
        # Quadratic formula to solve for motor speed
        a = C_Q0 * QUAD.rho * np.power(D_prop, 5) \
            / ((2.*np.pi)**2)
        b = (C_Q1 * QUAD.rho * np.power(D_prop, 4)
            / (2.*np.pi)) * v_air[i] + KQ**2/R_motor
        c = C_Q2 * QUAD.rho * np.power(D_prop, 3) \
            * (v_air[i])**2 - (KQ / R_motor) * V_in + KQ * i0
        c_der = (KQ / R_motor) * V_in_der
        # Consider only positive root
        Omega_op = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)
        Omega_op_der = c_der / np.sqrt(b**2 - 4*a*c)
        # compute advance ratio
        J_op = 2 * np.pi * v_air[i] / (Omega_op * D_prop)
        J_op_der = -2 * np.pi * v_air[i] * Omega_op_der / (Omega_op**2 * D_prop)
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
        # Flip moment sign for certain rotors
        Q_p *= -dir[i]
        Q_p_der *= -dir[i]

        thrust.append(T_p)
        torque.append(Q_p)
        thrust_der.append(T_p_der)
        torque_der.append(Q_p_der)

    return thrust, torque, thrust_der, torque_der





#function that gets the aerodynamic forces torques and derivatives
def aerodynamic_force_torque_der(delta: np.ndarray, state: MsgState)->tuple[np.ndarray, np.ndarray]:

    #gets the three control surfaces
    elevator = delta[0]
    aileron = delta[1]
    rudder = delta[2]

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
    Mx_aerodynamic = qbar * QUAD.S_wing * QUAD.c * (
            QUAD.C_m_0
            + QUAD.C_m_alpha * alpha
            + QUAD.C_m_q * q_nondim
            + QUAD.C_m_delta_e * elevator)
    # compute lateral torques in body frame
    My_aerodynamic  = qbar * QUAD.S_wing * QUAD.b * (
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

    return forcesBodyFrame, momentsBodyFrame
