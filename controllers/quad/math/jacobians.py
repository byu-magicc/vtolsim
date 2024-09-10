#this file finds and calculates the gradient vector from the objective function for the quadplane
#%%

import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[3]))

import numpy as np
import sympy as sp
from IPython.display import Latex, display

#imports the QUAD parts
import parameters.quad.anaconda_parameters as QUAD

#defines some constants used for all of these
rho, Va, S, b, c, m, g = sp.symbols('rho, V_a, S, b, c, m, g', real=True)

alpha, beta, p, q, r, phi, theta = sp.symbols('alpha, beta, p, q, r, phi, theta', real=True)


#defines the delta variables
delta_a, delta_e, delta_r, delta_t = sp.symbols('delta_a, delta_e, delta_r, delta_t', real = True)

#defines the delta vector
deltaVector = sp.Matrix([[delta_a], [delta_e], [delta_r], [delta_t]])

#gets the lift coefficients
CL_0, CL_alpha, CL_q, CL_delta_e = sp.symbols('C_{L_0}, C_{L_{\\alpha}}, C_{L_q}, C_{L_{\\delta_e}}')

#gets the drag coefficients
CD_0, CD_alpha, CD_q, CD_delta_e = sp.symbols('C_{D_0}, C_{D_{\\alpha}}, C_{D_q}, C_{D_{\\delta_e}}')


#gets the scaling factor
ForcesScalingFactor = (1/2)*rho*(Va**2)*S

#gets the Fx alpha matrix
FxAlphaMatrix = sp.Matrix([[sp.sin(alpha), -sp.cos(alpha)]])


LiftDragMixing = sp.Matrix([[1.0, alpha, (c*q)/(2*Va), delta_e, 0, 0, 0, 0],
                            [0, 0, 0, 0, 1.0, alpha, (c*q)/(2*Va), delta_e]])


LiftDragCoefficientsVector = sp.Matrix([[CL_0], [CL_alpha], [CL_q], [CL_delta_e], [CD_0], [CD_alpha], [CD_q], [CD_delta_e]])

#defines the Forward thrust component function
ForwardThrust = sp.Function('T')(delta_t)
#gets the Fx gravity term
Fx_Gravity = -m*g*sp.sin(theta)

#gets the Whole Function Fx
Fx = (ForcesScalingFactor*FxAlphaMatrix*LiftDragMixing*LiftDragCoefficientsVector)[0] + ForwardThrust + Fx_Gravity
print("Fx complete: ")
display(Fx)

#gets the z alpha matrix for forces
FzAlphaMatrix = sp.Matrix([[-sp.cos(alpha), -sp.sin(alpha)]]) 

#gets the Fz gravity
Fz_gravity = m*g*sp.cos(theta)*sp.cos(phi)

#gets the z forces
Fz = (ForcesScalingFactor*FzAlphaMatrix*LiftDragMixing*LiftDragCoefficientsVector)[0] + Fz_gravity
print("Fz complete: ")
display(Fz)

#############################################################################
#gets the Y forces
#defines the Y coefficients
C_Y_0, C_Y_beta, C_Y_p, C_Y_r, C_Y_delta_a, C_Y_delta_r = sp.symbols('C_{Y_0}, C_{Y_\\beta}, C_{Y_p}, C_{Y_r}, C_{Y_{\\delta_a}}, C_{Y_{\\delta_r}}')

#gets the Y Mixing matrix
YMixingMatrix = sp.Matrix([[1.0, beta, ((b*p)/(2*Va)), ((b*r)/(2*Va)), delta_a, delta_r]])


#gets the Y Coefficients vector
YCoefficientsVector = sp.Matrix([[C_Y_0], [C_Y_beta], [C_Y_p], [C_Y_r], [C_Y_delta_a], [C_Y_delta_r]])

#gets the Y coefficients gravity
Fy_Gravity = m*g*sp.cos(theta)*sp.sin(phi)


#gets the Forces function Fy
Fy = (ForcesScalingFactor*YMixingMatrix*YCoefficientsVector)[0] + Fy_Gravity
#############################################################################



###################################################################################################
#moments section


#l moments section
#gets the l scaling factor
lScalingFactor = (1/2)*rho*(Va**2)*S*b
#gets the m scaling factor
mScalingFactor = (1/2)*rho*(Va**2)*S*c
#gets the n scaling factor
nScalingFactor = (1/2)*rho*(Va**2)*S*b


#gets the l mixing matrix
lMixingMatrix = sp.Matrix([[1.0, beta, ((b*p)/(2*Va)), ((b*r)/(2*Va)), delta_a, delta_r]])
#gets the m mixing matrix
mMixingMatrix = sp.Matrix([[1.0, alpha, ((c*q)/(2*Va)), delta_e]])
#gets the n mixing matrix
nMixingMatrix = sp.Matrix([[1.0, beta, ((b*p)/(2*Va)), ((b*r)/(2*Va)), delta_a, delta_r]])



#gets the l coefficients
C_l_0, C_l_beta, C_l_p, C_l_r, C_l_delta_a, C_l_delta_r = sp.symbols('C_{l_0}, C_{l_\\beta}, C_{l_p}, C_{l_r}, C_{l_{\\delta_a}}, C_{l_{\\delta_r}}')

#gets the m coefficients
C_m_0, C_m_alpha, C_m_q, C_m_delta_e = sp.symbols('C_{m_0}, C_{m_\\alpha}, C_{m_q}, C_{m_{\\delta_e}}')

#gets the n coefficients
C_n_0, C_n_beta, C_n_p, C_n_r, C_n_delta_a, C_n_delta_r = sp.symbols('C_{n_0}, C_{n_\\beta}, C_{n_p}, C_{n_r}, C_{n_{\\delta_a}}, C_{n_{\\delta_r}}')


#creates the coefficient matricies
#l coefficient vector
lCoefficientVector = sp.Matrix([[C_l_0],[C_l_beta], [C_l_p], [C_l_r], [C_l_delta_a], [C_l_delta_r]])
#m coefficient vector
mCoefficientVector = sp.Matrix([[C_m_0], [C_m_alpha], [C_m_q], [C_m_delta_e]])
#n coeficient vector
nCoefficientVector = sp.Matrix([[C_n_0], [C_n_beta], [C_n_p], [C_n_r], [C_n_delta_a], [C_n_delta_r]])


#moments from forward prop
MomentForward = sp.Function('M')(delta_t)


#adds everything together for each moment
Mx = (lScalingFactor*lMixingMatrix*lCoefficientVector)[0] + MomentForward

My = (mScalingFactor*mMixingMatrix*mCoefficientVector)[0]
print("My Complete")
display(My)
Mz = (nScalingFactor*nMixingMatrix*nCoefficientVector)[0]


#finds the partials of Fx, Fz, and My with respect to delta_e
Fx_dot = sp.diff(Fx, delta_e)
print("Fx dot")
display(Fx_dot)
Fz_dot = sp.diff(Fz, delta_e)
print("Fz dot")
display(Fz_dot)
My_dot = sp.diff(My, delta_e)
print("My dot")
display(My_dot)


###############################################################################################################
#Fx section

#constructs the Fx_delta_e variable
LiftDrag_delta_e_Coefficients = sp.Matrix([[CL_delta_e], [CD_delta_e]])
LiftDrag_delta_e_Mixing = sp.Matrix([[delta_e, 0],
                               [0, delta_e]])

#gets the stuff for the residue
LiftDrag_residue_Coefficients = sp.Matrix([[CL_0], [CL_alpha], [CL_q], [CD_0], [CD_alpha], [CD_q]])
LiftDrag_residue_Mixing = sp.Matrix([[1.0, alpha, (c*q)/(2*Va), 0, 0, 0],
                                     [0, 0, 0, 1.0, alpha, (c*q)/(2*Va)]])


#gets the m delta e coefficients
m_delta_e_Coefficients = sp.Matrix([[C_m_delta_e]])
#gets the m delta e mixing matrix
m_delta_e_Mixing = sp.Matrix([[delta_e]])

#gets the Residue matrix for the m moments
m_residue_Coefficients = sp.Matrix([[C_m_0], [C_m_alpha], [C_m_q]])
#gets the mixing matrix for that
m_residue_Mixing = sp.Matrix([[1.0, alpha, ((c*q)/(2*Va))]])



#constructs the Fx residue
Fx_residue = (ForcesScalingFactor*FxAlphaMatrix*LiftDrag_residue_Mixing*LiftDrag_residue_Coefficients)[0] + ForwardThrust + Fx_Gravity
print("Fx residue: ")
display(Fx_residue)

#constructs the Fz residue
Fz_residue = (ForcesScalingFactor*FzAlphaMatrix*LiftDrag_residue_Mixing*LiftDrag_residue_Coefficients)[0] + Fz_gravity
print("Fz residue: ")
display(Fz_residue)

#constructs the My residue 
My_residue = (mScalingFactor*m_residue_Mixing*m_residue_Coefficients)[0]
print("My residue: ")
display(My_residue)


#########################################################################################
#constucts the delta e components
Fx_delta_e = (ForcesScalingFactor*FxAlphaMatrix*LiftDrag_delta_e_Mixing*LiftDrag_delta_e_Coefficients)[0]
print("Fx delta e")
display(Fx_delta_e)

Fz_delta_e = (ForcesScalingFactor*FzAlphaMatrix*LiftDrag_delta_e_Mixing*LiftDrag_delta_e_Coefficients)[0]
print("Fz delta e")
display(Fz_delta_e)

#constructs the My delta e
My_delta_e = (mScalingFactor*m_delta_e_Mixing*m_delta_e_Coefficients)[0]
print("My delta e")
display(My_delta_e)


#gets the derivative of the objective function
objective_dot = ((Fx_residue*Fx_dot + Fz_residue*Fz_dot + My_residue*My_dot)+(Fx_delta_e*Fx_dot + Fz_delta_e*Fz_dot + My_delta_e*My_dot))

display(objective_dot)





# %%
