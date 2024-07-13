#%%
#This file is a sympy file that performs math operations to help me significantly with the math equations


from IPython.display import Latex, display
import sympy as sp


#defines some constants used for all of these
rho, Va, S, b, c, m, g = sp.symbols('rho, V_a, S, b, c, m, g', real=True)
display(rho, Va)


#defines variables that are part of the state of the system
alpha, beta, p, q, r, phi, theta = sp.symbols('alpha, beta, p, q, r, phi, theta', real=True)
display(beta)

alpha2 = alpha**2

#defines the delta variables
delta_e, delta_a, delta_r = sp.symbols('delta_e, delta_a, delta_r', real = True)
display(delta_e)

#defines the delta thrust variables
delta_t1, delta_t2, delta_t3, delta_t4, delta_t5 = sp.symbols('\delta_{t1}, \delta_{t2}, \delta_{t3}, \delta_{t4}, \delta_{t5}')
display(delta_t2)


#############################################################################
#defines the variables for the for the Fx components

#gets the lift coefficients
CL_0, CL_alpha, CL_alpha2, CL_q, CL_delta_e = sp.symbols('C_{L_0}, C_{L_{\\alpha}}, C_{L_{\\alpha^2}}, C_{L_q}, C_{L_{\\delta_e}}')

#gets the drag coefficients
CD_0, CD_alpha, CD_alpha2, CD_q, CD_delta_e = sp.symbols('C_{D_0}, C_{D_{\\alpha}}, C_{D_{\\alpha^2}}, C_{D_q}, C_{D_{\\delta_e}}')

#gets the scaling factor
ForcesScalingFactor = (1/2)*rho*(Va**2)*S

#gets the Fx alpha matrix
FxAlphaMatrix = sp.Matrix([[sp.sin(alpha), -sp.cos(alpha)]])
display(FxAlphaMatrix)



LiftDragMixing = sp.Matrix([[1.0, alpha, alpha2, (c*q)/(2*Va), delta_e, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 1.0, alpha, alpha2, (c*q)/(2*Va), delta_e]])



LiftDragCoefficientsVector = sp.Matrix([[CL_0], [CL_alpha], [CL_alpha2], [CL_q], [CL_delta_e], [CD_0], [CD_alpha], [CD_alpha2], [CD_q], [CD_delta_e]])


#defines the Forward thrust component function
ForwardThrust = sp.Function('T')(delta_t5)
display(ForwardThrust)
#gets the Fx gravity term
Fx_Gravity = -m*g*sp.sin(theta)

#gets the Whole Function Fx
Fx = (ForcesScalingFactor*FxAlphaMatrix*LiftDragMixing*LiftDragCoefficientsVector)[0] + ForwardThrust + Fx_Gravity
display(Fx)
#gets everything for Fz

#gets the z alpha matrix for forces
FzAlphaMatrix = sp.Matrix([[-sp.cos(alpha), -sp.sin(alpha)]]) 

#gets the thrust functions for the vertical rotors
VerticalThrust1 = sp.Function('T')(delta_t1)
VerticalThrust2 = sp.Function('T')(delta_t2)
VerticalThrust3 = sp.Function('T')(delta_t3)
VerticalThrust4 = sp.Function('T')(delta_t4)

#gets the Fz thrust
Fz_Thrust = -VerticalThrust1 - VerticalThrust2 - VerticalThrust3 - VerticalThrust4

#gets the Fz gravity
Fz_gravity = m*g*sp.cos(theta)*sp.cos(phi)

Fz = (ForcesScalingFactor*FzAlphaMatrix*LiftDragMixing*LiftDragCoefficientsVector)[0] + Fz_Thrust + Fz_gravity
#############################################################################

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


#creates the Moments from the prop rotation of the five props
#moments from forward prop
MomentForward = sp.Function('M')(delta_t5)
#moments from vertical props
MomentVertical1 = sp.Function('M')(delta_t1)
MomentVertical2 = sp.Function('M')(delta_t2)
MomentVertical3 = sp.Function('M')(delta_t3)
MomentVertical4 = sp.Function('M')(delta_t4)

#gets the vertical moments sum
verticalMoments = MomentVertical1 + MomentVertical2 + MomentVertical3 + MomentVertical4

############################****************************************************************************************************************************************
#creates the variables for the positions of the four vertical props
#prop 1 position variables
p_x1, p_y1, p_z1 = sp.symbols('p_{x1}, p_{y1}, p_{z1}')
p_x2, p_y2, p_z2 = sp.symbols('p_{x2}, p_{y2}, p_{z2}')
p_x3, p_y3, p_z3 = sp.symbols('p_{x3}, p_{y3}, p_{z3}')
p_x4, p_y4, p_z4 = sp.symbols('p_{x4}, p_{y4}, p_{z4}')

#gets the prop position 1 vector
p_1 = sp.Matrix([[p_x1], [p_y1], [p_z1]])
p_2 = sp.Matrix([[p_x2], [p_y2], [p_z2]])
p_3 = sp.Matrix([[p_x3], [p_y3], [p_z3]])
p_4 = sp.Matrix([[p_x4], [p_y4], [p_z4]])



#adds everything together for each moment
Mx = (lScalingFactor*lMixingMatrix*lCoefficientVector)[0] + MomentForward

My = (mScalingFactor*mMixingMatrix*mCoefficientVector)[0]

Mz = (nScalingFactor*nMixingMatrix*nCoefficientVector)[0] + verticalMoments



###################################################################################################


# %%
