#%%

#this file finds the derivative of the thrust and moment with respect to delta_t
import sympy as sp
from IPython.display import Latex, display
import numpy as np

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

def getThrustTorqueGradient(Va_in: float, delta_t_in: float, C: np.ndarray, Electrical: np.ndarray, Aerodynamics: np.ndarray):
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
                                    C[0][0], C[1][0], C[2][0], C[3][0], C[4][0], C[5][0],
                                    Electrical[0][0], Electrical[1][0], Electrical[2][0], Electrical[3][0],\
                                    Aerodynamics[0][0], Aerodynamics[1][0], np.pi)

    partial_Q_numerical = partial_Q(Va_in, delta_t_in,\
                                    C[0][0], C[1][0], C[2][0], C[3][0], C[4][0], C[5][0],
                                    Electrical[0][0], Electrical[1][0], Electrical[2][0], Electrical[3][0],\
                                    Aerodynamics[0][0], Aerodynamics[1][0], np.pi)
    
    ##########################################################################################
    return partial_T_numerical, partial_Q_numerical
