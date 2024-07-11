#This file contains the parameters for the anaconda aircraft


import numpy as np
from tools.rotations import euler_to_quaternion


######################################################################################
                #Initial Conditions
######################################################################################

#initial conditions for the MAV
pn0 = 0.  # initial north position
pe0 = 0. # initial east position
pd0 = 0.  # initial down position
u0 = 0. # initial velocity along body x-axis
v0 = 0.  # initial velocity along body y-axis
w0 = 0. # initial velocity along body z-axis
phi0 = 0. # initial roll angle
theta0 =  0.  # initial pitch angle
psi0 = 0. # initial yaw angle
p0 = 0  # initial roll rate
q0 = 0  # initial pitch rate
r0 = 0  # initial yaw rate
Va0 = np.sqrt(u0**2+v0**2+w0**2)
#   Quaternion State
e = euler_to_quaternion(phi0, theta0, psi0)
e0 = e.item(0)
e1 = e.item(1)
e2 = e.item(2)
e3 = e.item(3)



######################################################################################
                #Physical Parameters
######################################################################################
mass = 1.0 #kg
Jx = 0.0165#0.008383 #kg m^2
Jy = 0.025#0.006114
Jz = 0.0282#0.01435
Jxz = 0.000048#0.000024
S_wing = 0.2589#0.19#0.55
b = 1.4224#0.65#2.8956
c = 0.3305#0.25#0.18994

#other physical parameters
S_prop = 0.2027
rho = 1.2682
e = 0.9
e_oswald = e
AR = (b**2) / S_wing
AR_wing = AR
gravity = 9.81

#sets the physical positions of the props. That is, where their bases are located.
#in units of meters

#sets the location of the forward prop
forward_rotor_pos = np.array([[-0.3], [0.0], [0.0]])

#sets the locations of the vertical propellers
vertical_rotor_1_pos = np.array([[-0.5], [-0.5], [0.0]])
vertical_rotor_2_pos = np.array([[0.5], [-0.5], [0.0]])
vertical_rotor_3_pos = np.array([[0.5], [0.5], [0.0]])
vertical_rotor_4_pos = np.array([[-0.5], [0.5], [0.0]])

#some necessary parameters
M = 50.0
alpha0 = np.deg2rad(15) #0.47
epsilon = 0.16


#######################################################################################
#Coefficients
C_L_0 = 0.005
C_D_0 = 0.0022
C_m_0 = 0.0
C_L_alpha = 2.819
C_D_alpha = 0.03
C_m_alpha = -0.185
C_L_q = 3.242
C_D_q = 0.0
C_m_q = -1.093
C_L_delta_e = 0.2
C_D_delta_e = 0.005
C_m_delta_e = -0.05

C_D_p = 0.0

C_Y_0 = 0.0
C_ell_0 = 0.0
C_n_0 = 0.0

C_Y_beta = -0.318
C_ell_beta = -0.032
C_n_beta = 0.112
C_Y_p = 0.078
C_ell_p = -0.207
C_n_p = -0.053
C_Y_r = 0.288
C_ell_r = 0.036
C_n_r = -0.104
C_Y_delta_a = 0.000536
C_ell_delta_a = 0.018
C_n_delta_a = -0.00328#-0.011
C_Y_delta_r = 0.0
C_ell_delta_r = 0.0
C_n_delta_r = 0.0



######################################################################################
                #  Propeller parameters
######################################################################################

# # Prop parameters
D_prop = 20*(0.0254)     # prop diameter in m


# Motor parameters
K_V = 145.                   # from datasheet RPM/V
KQ = (1. / K_V) * 60. / (2. * np.pi)  # KQ in N-m/A, V-s/rad
R_motor = 0.042              # ohms
i0 = 1.5                     # no-load (zero-torque) current (A)

k_force = 65.0
k_moment = 5.0

# Inputs
# ncells = 12.
# V_max = 3.7 * ncells  # max voltage for specified number of battery cells
# Inputs
ncells = 3.
V_max = 3.7 * ncells  # max voltage for specified number of battery cells


# Coeffiecients from prop_data fit
C_Q2 = -0.01664
C_Q1 = 0.004970
C_Q0 = 0.005230
C_T2 = -0.1079
C_T1 = -0.06044
C_T0 = 0.09357






######################################################################################
                #   Calculation Variables
######################################################################################
#   gamma parameters pulled from page 36 (dynamics)
gamma = Jx * Jz - (Jxz**2)
gamma1 = (Jxz * (Jx - Jy + Jz)) / gamma
gamma2 = (Jz * (Jz - Jy) + (Jxz**2)) / gamma
gamma3 = Jz / gamma
gamma4 = Jxz / gamma
gamma5 = (Jz - Jx) / Jy
gamma6 = Jxz / Jy
gamma7 = ((Jx - Jy) * Jx + (Jxz**2)) / gamma
gamma8 = Jx / gamma
