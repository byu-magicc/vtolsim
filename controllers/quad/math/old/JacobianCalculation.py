#this file implements the calculations for the jacobian


from message_types.quad.msg_state import MsgState
from message_types.quad.msg_delta import MsgDelta
import numpy as np
import parameters.quad.anaconda_parameters as QUAD
import sympy as sp

#defines the function to get the thrust and torque gradients
#arguments: 
#1. Va: the Airspeed, which is a scalar, and needs to be calculated as the airspeed perpendicular to the direction of travel
#2. delta_t: the input for the thrust and torque function
#3. C: 6x1 matrix of C_Q and C_T coefficients
#4. Electrical: 4x1 matrix of electrical characteristics of the motor
#5. Aerodynamics: 2x1 Matrix of characteristics for the aerodynamic system operation
#returns:
#1: Partial of T with respect to delta_t
#2: Partial of Q with respect to delta_t

def getThrustTorqueGradient(Va_in: float, delta_t_in: float):
    ##########################################################################################
    #symbolic section
    #creates the symbols for the C constants
    C_Q0, C_Q1, C_Q2, C_T0, C_T1, C_T2 = sp.symbols('C_{Q0}, C_{Q1}, C_{Q2}, C_{T0}, C_{T1}, C_{T2}', real=True)

    V_max, KQ, R_motor, i0 = sp.symbols('V_{max},K_Q, R_{motor}, i_0', real=True)

    D_prop, rho, pi, delta_t = sp.symbols('D_{prop}, rho, pi, {\delta}_{t}', real=True)

    Va = sp.symbols('V_a', real=True)

    #sets V_in
    V_in = V_max*delta_t

    a = C_Q0*rho*(D_prop**5)/((2*pi)**2)

    b = (C_Q1*rho*(D_prop**4)/(2*pi)) * Va + (KQ**2)/R_motor

    c = C_Q2*rho*(D_prop**3)*(Va**2) - (KQ/R_motor) * V_in + KQ*i0

    #solves the quadratic equation
    Omega_op = (-b + sp.sqrt(b**2 - 4*a*c))/(2*a)

    #computes the advance ratio
    J_op = 2*pi*Va/(Omega_op*D_prop)

    #gets the full t and q coefficients
    C_T = C_T2*J_op**2 + C_T1*J_op + C_T0
    C_Q = C_Q2*J_op**2 + C_Q1*J_op + C_Q0

    #adds thrust and torque due to the propeller
    n = Omega_op/(2*pi)

    #gets the thrust and moment
    T_p = rho*(n**2)*(D_prop**4)*C_T
    Q_p = rho*(n**2)*(D_prop**5)*C_Q



    #gets the partial derivative of Thrust with respect to delta_t
    partial_T_delta_t = sp.diff(T_p, delta_t)

    #gets the partial derivative of Q with respect to delta_t
    partial_Q_delta_t = sp.diff(Q_p, delta_t)

    #end symbolic section
    ##########################################################################################


    ##########################################################################################
    #creates vector that contains all of the symbols used in this sympy array thing
    symbols = (Va, delta_t,\
               C_Q0, C_Q1, C_Q2, C_T0, C_T1, C_T2,\
               V_max, KQ, R_motor, i0,\
               D_prop, rho, pi)
    

    #creates the function for the T partial numerical function
    partial_T = sp.lambdify(symbols, partial_T_delta_t, modules='numpy')

    #creates the function for the M partial numerical function
    partial_Q = sp.lambdify(symbols, partial_Q_delta_t, modules='numpy')


    #gets the numerical partial_T
    partial_T_numerical = partial_T(Va_in, delta_t_in,\
                                    QUAD.C_Q0, QUAD.C_Q1, QUAD.C_Q2, QUAD.C_T0, QUAD.C_T1, QUAD.C_T2,\
                                    QUAD.V_max, QUAD.KQ, QUAD.R_motor, QUAD.i0,\
                                    QUAD.D_prop, QUAD.rho, np.pi)

    partial_Q_numerical = partial_Q(Va_in, delta_t_in,\
                                    QUAD.C_Q0, QUAD.C_Q1, QUAD.C_Q2, QUAD.C_T0, QUAD.C_T1, QUAD.C_T2,\
                                    QUAD.V_max, QUAD.KQ, QUAD.R_motor, QUAD.i0,\
                                    QUAD.D_prop, QUAD.rho, np.pi)
    
    ##########################################################################################
    return partial_T_numerical, partial_Q_numerical

#gets the jacobian of the Force matrix
def getJacobians(delta: MsgDelta, state: MsgState, airspeed: MsgDelta):

    #defines the airspeed in the x direction
    Va_x = airspeed[0][0]

    #defines the airspeed in the minus z direction
    #(this means that if the airspeed is -15 in the z direction, the airspeed in the minus z direction is 15)
    #(if airspeed in the z direction is 15, then airspeed in minus z direction is -15, and so forth)
    Va_z_minus = -1.0*airspeed[2][0]


    rho = QUAD.rho
    Va = state.Va
    S = QUAD.S_wing
    b = QUAD.b
    c = QUAD.c


    alpha = state.alpha
    sa = np.sin(state.alpha)
    ca = np.cos(state.alpha)

    #breaks up the jacobian and then adds it together so it's less messy
    ForceFixedWingJacobian = np.array([[0.5*rho*(Va**2)*S*(sa*QUAD.C_L_delta_e - ca*QUAD.C_D_delta_e), 0, 0, 0, 0, 0, 0, 0],
                                       [0, 0.5*rho*(Va**2)*S*QUAD.C_Y_delta_a, 0.5*rho*(Va**2)*S*QUAD.C_Y_delta_r, 0, 0, 0, 0, 0],
                                       [0.5*rho*(Va**2)*S*(-ca*QUAD.C_L_delta_e - sa*QUAD.C_D_delta_e), 0, 0, 0, 0, 0, 0, 0]])

    delta_t_f = delta.forwardThrottle
    delta_t_v1 = delta.verticalThrottle_1
    delta_t_v2 = delta.verticalThrottle_2
    delta_t_v3 = delta.verticalThrottle_3
    delta_t_v4 = delta.verticalThrottle_4


    #gets the partial of T and Q with respect to each of the delta_t's
    d_T_d_delta_t_f, d_Q_d_delta_t_f = getThrustTorqueGradient(Va_in=Va_x, delta_t_in=delta_t_f)
    d_T_d_delta_t_v1, d_Q_d_delta_t_v1 = getThrustTorqueGradient(Va_in=Va_z_minus, delta_t_in=delta_t_v1)
    d_T_d_delta_t_v2, d_Q_d_delta_t_v2 = getThrustTorqueGradient(Va_in=Va_z_minus, delta_t_in=delta_t_v2)
    d_T_d_delta_t_v3, d_Q_d_delta_t_v3 = getThrustTorqueGradient(Va_in=Va_z_minus, delta_t_in=delta_t_v3)
    d_T_d_delta_t_v4, d_Q_d_delta_t_v4 = getThrustTorqueGradient(Va_in=Va_z_minus, delta_t_in=delta_t_v4)


    ForceThrustJacobian = np.array([[0, 0, 0, d_T_d_delta_t_f, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, d_T_d_delta_t_v1, d_T_d_delta_t_v2, d_T_d_delta_t_v3, d_T_d_delta_t_v4]])
    

    MomentFixedWingJacobian = np.array([[0, 0.5*rho*(Va**2)*S*b*QUAD.C_ell_delta_a, 0.5*rho*(Va**2)*S*b*QUAD.C_ell_delta_r, 0, 0, 0, 0, 0],
                                        [0.5*rho*(Va**2)*S*c*QUAD.C_m_delta_e, 0, 0, 0, 0, 0, 0, 0],
                                        [0, 0.5*rho*(Va**2)*S*b*QUAD.C_n_delta_a, 0.5*rho*(Va**2)*S*b*QUAD.C_n_delta_r, 0, 0, 0, 0, 0]])


    MomentThrustJacobian = np.array([[0, 0, 0, d_Q_d_delta_t_f, 0.5*d_T_d_delta_t_v1, 0.5*d_T_d_delta_t_v2, -0.5*d_T_d_delta_t_v3, -0.5*d_T_d_delta_t_v4],
                                     [0, 0, 0, 0, 0.5*d_T_d_delta_t_v1, -0.5*d_T_d_delta_t_v2, -0.5*d_T_d_delta_t_v3, 0.5*d_T_d_delta_t_v4],
                                     [0, 0, 0, 0, -d_Q_d_delta_t_v1, -d_Q_d_delta_t_v2, -d_Q_d_delta_t_v3, -d_Q_d_delta_t_v4]])
    
    ForceJacobian = ForceFixedWingJacobian + ForceThrustJacobian

    MomentJacobian = MomentFixedWingJacobian + MomentThrustJacobian


    return ForceJacobian, MomentJacobian



