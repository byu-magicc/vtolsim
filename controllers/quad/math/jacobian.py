#%%
#calculates the aerodynamic derivatives
import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[3]))

import sympy as sp

import parameters.quad.anaconda_parameters as QUAD
from IPython.display import display

C_L_0 = sp.symbols('C_{L_0}')
C_D_0 = sp.symbols('C_{E_0}')
C_m_0 = sp.symbols('C_{m_0}')
C_L_alpha = sp.symbols('C_{L_{\\alpha}}')
C_D_alpha = sp.symbols('C_{D_{\\alpha}}')
C_m_alpha = sp.symbols('C_{m_{\\alpha}}')
C_L_q = sp.symbols('C_{L_q}')
C_D_q = sp.symbols('C_{D_q}')
C_m_q = sp.symbols('C_{m_q}')
C_L_delta_e = sp.symbols('C_{L_{\\delta_e}}')
C_D_delta_e = sp.symbols('C_{D_{\\delta_e}}')
C_m_delta_e = sp.symbols('C_{m_{\\delta_e}}')

C_Y_0 = sp.symbols('C_{Y_0}')
C_ell_0 = sp.symbols('C_{ell_0}')
C_n_0 = sp.symbols('C_{n_0}')
C_Y_beta = sp.symbols('C_{Y_{\\beta}}')
C_ell_beta = sp.symbols('C_{ell_{\\beta}}')
C_n_beta = sp.symbols('C_{n_{\\beta}}')
C_Y_p = sp.symbols('C_{Y_p}')
C_ell_p = sp.symbols('C_{ell_p}')
C_n_p = sp.symbols('C_{n_p}')
C_Y_r = sp.symbols('C_{Y_r}')
C_ell_r = sp.symbols('C_{ell_r}')
C_n_r = sp.symbols('C_{n_r}')
C_Y_delta_a = sp.symbols('C_{Y_{\\delta_a}}')
C_ell_delta_a = sp.symbols('C_{ell_{\\delta_a}}')
C_n_delta_a = sp.symbols('C_{n_{\\delta_a}}')
C_Y_delta_r = sp.symbols('C_{Y_{\\delta_r}}')
C_ell_delta_r = sp.symbols('C_{ell_{\\delta_r}}')
C_n_delta_r = sp.symbols('C_{n_{\\delta_r}}')

display(C_L_delta_e)


#gets the three control surfaces
delta_e = sp.symbols('delta_e')
delta_a = sp.symbols('delta_a')
delta_r = sp.symbols('delta_r')

#creates the delta vector
delta_c = sp.Matrix([delta_e, delta_a, delta_r])

#splits up the omega
p = sp.symbols('p')
q = sp.symbols('q')
r = sp.symbols('r')
#gets the individual variables in the state
#airspeed
Va = sp.symbols('V_a')

#angle of attack
alpha = sp.symbols('alpha')
#sideslip angle
beta = sp.symbols('beta')

rho = sp.symbols('rho')

#intermediate variables
qbar = 0.5 * rho * Va**2
ca = sp.cos(alpha)
sa = sp.sin(alpha)

S_wing = sp.symbols('S_{wing}')


b = sp.symbols('b')
c = sp.symbols('c')

display(C_L_alpha)

# compute Lift and Drag Forces in the inertial frame
F_lift = qbar * S_wing * (C_L_0 + alpha*C_L_alpha + C_L_q*q + C_L_delta_e*delta_e)

F_drag = qbar * S_wing * (C_D_0 + alpha*C_D_alpha + C_D_q*q + C_D_delta_e*delta_e)

# compute longitudinal forces in body frame
fx_aerodynamic = - ca * F_drag + sa * F_lift
fz_aerodynamic = - sa * F_drag - ca * F_lift
# compute lateral forces in body frame
fy_aerodynamic = qbar * S_wing * (
                        C_Y_0
                      + C_Y_beta * beta
                      + C_Y_p * p
                      + C_Y_r * r
                      + C_Y_delta_a * delta_a
                      + C_Y_delta_r * delta_r)
# compute logitudinal torque in body frame
My_aerodynamic = qbar * S_wing * c * (
        C_m_0 + C_m_alpha * alpha + C_m_q * q + C_m_delta_e * delta_e)
# compute lateral torques in body frame
Mx_aerodynamic  = qbar * S_wing * b * (
        C_ell_0
        + C_ell_beta * beta
        + C_ell_p * p
        + C_ell_r * r
        + C_ell_delta_a * delta_a
        + C_ell_delta_r * delta_r)
Mz_aerodynamic = qbar * S_wing * b * (
        C_n_0 + C_n_beta * beta
        + C_n_p * p
        + C_n_r * r
        + C_n_delta_a * delta_a
        + C_n_delta_r * delta_r)


#creates the Wrench Vector

wrenchAerodynamic = sp.Matrix([[fx_aerodynamic],
                               [fy_aerodynamic],
                               [fz_aerodynamic],
                               [Mx_aerodynamic],
                               [My_aerodynamic],
                               [Mz_aerodynamic]])


display(wrenchAerodynamic)

#gets the jacobian of the wrench vector
wrenchJacobian = wrenchAerodynamic.jacobian(delta_c)
print('Wrench Jacobian: ')
display(wrenchJacobian)

# %%
