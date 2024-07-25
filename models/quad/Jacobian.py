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
delta_a, delta_e, delta_r = sp.symbols('delta_a, delta_e, delta_r', real = True)
display(delta_e)

#defines the delta thrust variables
delta_t1, delta_t2, delta_t3, delta_t4, delta_t5 = sp.symbols('\delta_{t1}, \delta_{t2}, \delta_{t3}, \delta_{t4}, \delta_{t5}')
display(delta_t2)


#defines the delta vector
deltaVector = sp.Matrix([[delta_a], [delta_e], [delta_r], [delta_t1], [delta_t2], [delta_t3], [delta_t4], [delta_t5]])
display(deltaVector)

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
#display(Fx)
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

#gets the Thrust Moments
ThrustMoment1 = sp.Matrix([[-p_y1*VerticalThrust1],[p_x1*VerticalThrust1],[0]])
ThrustMoment2 = sp.Matrix([[-p_y2*VerticalThrust2],[p_x2*VerticalThrust2],[0]])
ThrustMoment3 = sp.Matrix([[-p_y3*VerticalThrust3],[p_x3*VerticalThrust3],[0]])
ThrustMoment4 = sp.Matrix([[-p_y4*VerticalThrust4],[p_x4*VerticalThrust4],[0]])


#gets the sum of the x components of the thrust moments
ThrustMomentsX = ThrustMoment1[0] + ThrustMoment2[0] + ThrustMoment3[0] + ThrustMoment4[0]
ThrustMomentsY = ThrustMoment1[1] + ThrustMoment2[1] + ThrustMoment3[1] + ThrustMoment4[1]
ThrustMomentsZ = ThrustMoment1[2] + ThrustMoment2[2] + ThrustMoment3[2] + ThrustMoment4[2]

#adds everything together for each moment
Mx = (lScalingFactor*lMixingMatrix*lCoefficientVector)[0] + MomentForward + ThrustMomentsX

My = (mScalingFactor*mMixingMatrix*mCoefficientVector)[0] + ThrustMomentsY

Mz = (nScalingFactor*nMixingMatrix*nCoefficientVector)[0] + verticalMoments + ThrustMomentsZ

#gets the whole wrench vector
Wrench = sp.Matrix([[Fx],[Fy],[Fz],[Mx],[My],[Mz]])


#gets the Jacobian of the wrench with respect to the delta vector
Wrench_Jacobian = Wrench.jacobian(deltaVector)
Wrench_Jacobian_Transposed = sp.transpose(Wrench_Jacobian)
display(Wrench_Jacobian_Transposed)

wrenchJacobianLatex = sp.latex(Wrench_Jacobian)
wrenchJacobianTransposedLatex = sp.latex(Wrench_Jacobian_Transposed)

file = open("JacobianOutput.txt", "w")
file.write(wrenchJacobianTransposedLatex)
file.close


newFile = open("JacobianOutput.txt", "r")
output = newFile.read()
print(output)
newFile.close()




###################################################################################################

###################################################################################################
#this section creates the symbolic form of the motor thrust functions
C_Q0, C_Q1, C_Q2, D_prop, V_max, delta_ti = sp.symbols('C_{Q0}, C_{Q1}, C_{Q2}, D_{p}, V_{max}, \\delta_{ti}')
Va_i, KQ, C_T0, C_T1, C_T2, R_motor, i0 = sp.symbols('Va_{i}, K_Q, C_{T0}, C_{T1}, C_{T2}, R_{motor}, i_0')
display(Va_i)

#defines the Vin
Vin = V_max*delta_ti

#gets the a_prop
a_prop = ((C_Q0*rho*(D_prop**5))/((2*sp.pi)**2))
display(a_prop)
#gets the b_prop
b_prop = (rho*(D_prop**4))/(2*sp.pi)*C_Q1*Va_i + (KQ**2)/(R_motor) 
display(b_prop)

#gets the c prop
c_prop = rho*(D_prop**3)*C_Q2*(Va_i**2) - (KQ/R_motor)*Vin + KQ*i0
display(c_prop)

#gets the angular velocity of the propeller
Omega_p = ((-b_prop + sp.sqrt(b_prop**2 - 4*a_prop*c_prop))/(2*a_prop))
display(Omega_p)

#computes the advance ratio
J_op = 2*sp.pi*Va_i/(Omega_p*D_prop)

#gets the coefficient of thrust
C_T = C_T2*(J_op**2) + C_T1*J_op + C_T0
C_Q = C_Q2*(J_op**2) + C_Q2*J_op + C_T0

#gets n (whatever that is)
n = Omega_p / (2*sp.pi)
#gets the thrust output for a particular propeller
T_pi = rho*(n**2)*(D_prop**4)*C_T
Q_pi = rho*(n**2)*(D_prop**5)*C_Q

#gets the derivative of T_pi with respect to delta_ti
T_p_partial = sp.diff(T_pi, delta_ti)
#display(T_p_partial)


###################################################################################################

# %%
