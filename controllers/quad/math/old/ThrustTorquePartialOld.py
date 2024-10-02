#%%

#this file finds the derivative of the thrust and moment with respect to delta_t
import sympy as sp
from IPython.display import Latex, display


#creates the symbols for the C constants
C_Q0, C_Q1, C_Q2, C_T0, C_T1, C_T2 = sp.symbols('C_{Q0}, C_{Q1}, C_{Q2}, C_{T0}, C_{T1}, C_{T2}', real=True)

D_prop, KQ, R_motor, i0 = sp.symbols('D_{prop}, K_Q, R_{motor}, i_0', real=True)

V_max, rho, pi, delta_t = sp.symbols('V_{max}, rho, pi, {\delta}_{t}', real=True)

Va = sp.symbols('V_a', real=True)

#sets V_in
V_in = V_max*delta_t

a = C_Q0*rho*(D_prop**5)/((2*pi)**2)

b = (C_Q1*rho*(D_prop**4)/(2*pi)) * Va + (KQ**2)/R_motor

c = C_Q2*rho*(D_prop**3)*(Va**2) - (KQ/R_motor) * V_in + KQ*i0


a_sym, b_sym, c_sym = sp.symbols('a, b, c', real=True)


#solves the quadratic equation
Omega_op = (-b_sym + sp.sqrt(b_sym**2 - 4*a_sym*c_sym))/(2*a_sym)

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


#Gets the derivative of T_p and Q_p with respect to delta_t


########################################################################
#Thrust Gradient
#gets the partial derivative of the Thrust with respect to the c coefficient
#where the C coefficient
partial_F_C = sp.diff(T_p, c_sym)

print('partial of F with respect to C')
display(partial_F_C)

partial_C_delta_t = sp.diff(c, delta_t)
print('partial of C with respect to delta_t')
display(partial_C_delta_t)

print("partial of F with respect to C")
print(sp.latex(partial_F_C))

print("a")
print(sp.latex(a))

print("b")
print(sp.latex(b))

print("c")
print(sp.latex(c))

print("partial of C with respect to delta")
print(partial_C_delta_t)

print('partial T p')
partial_T_p = sp.diff(T_p, delta_t)
########################################################################


#########################################################################
#Aerodynamic Moment Gradient

#gets the partial derivative of the moment with respect to the c coefficient
partial_M_C = sp.diff(Q_p, c_sym)

print('partial of M with respect to C')
display(partial_M_C)

print('partial of M Latex')
print(sp.latex(partial_M_C))


print('partial Q p')
partial_Q_p = sp.diff(Q_p, delta_t)
display(partial_Q_p)
#########################################################################

#converts the 



# %%
