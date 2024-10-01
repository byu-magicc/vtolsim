#This file implements the functions that get the forces, and torques of the system,
#as well as the derivatives of those terms.

import sympy as sp

import numpy as np

import parameters.quad.anaconda_parameters as QUAD




#function that gets the thrust, moment, thrust derivative, and moment derivative of the system
#Arguments:
#1. delta_t: vector of throttle inputs (0.0, 1.0) for the individual rotor
#2. Va: vector of the airspeeds through the various rotors
#3. dir: the direction of rotation for each of the rotors
def rotor_thrust_torque_der(delta: np.ndarray, Va: np.ndarray, dir: np.ndarray)->tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
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
            / (2.*np.pi)) * Va[i] + KQ**2/R_motor
        c = C_Q2 * QUAD.rho * np.power(D_prop, 3) \
            * (Va[i])**2 - (KQ / R_motor) * V_in + KQ * i0
        c_der = (KQ / R_motor) * V_in_der
        # Consider only positive root
        Omega_op = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)
        Omega_op_der = c_der / np.sqrt(b**2 - 4*a*c)
        # compute advance ratio
        J_op = 2 * np.pi * Va[i] / (Omega_op * D_prop)
        J_op_der = -2 * np.pi * Va[i] * Omega_op_der / (Omega_op**2 * D_prop)
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


