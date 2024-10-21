#This file contains the parameters for the anaconda aircraft


import numpy as np
from tools.rotations import euler_to_quaternion


######################################################################################
                #Initial Conditions
######################################################################################
#   Initial conditions for QUADplane
north0 = 0.  # initial north position
east0 = 0.  # initial east position
down0 = -100.0  # initial down position
u0 = 25.  # initial velocity along body x-axis
v0 = 0.  # initial velocity along body y-axis
w0 = 0.  # initial velocity along body z-axis
phi0 = 0.  # initial roll angle
theta0 = 0.  # initial pitch angle
psi0 = 0.0  # initial yaw angle
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
mass = 11.0 #kg
Jx = 0.8244 #kg m^2
Jy = 1.135
Jz = 1.759
Jxz = 0.1204
S_wing = 0.55 #surface are of the wing
b = 2.8956 #wingspan
c = 0.18994 #mean chord of the QUAD Wing

#other physical parameters
S_prop = 0.2027 #surface area of the propellor
rho = 1.2682 #air density
e = 0.9
AR = (b**2) / S_wing
gravity = 9.81

num_rotors = 5

#sets the physical positions of the props. That is, where their bases are located.
#in units of meters

#sets the location of the forward prop
forward_rotor_pos = np.array([[-0.3], [0.0], [0.0]])

#sets the locations of the vertical propellers
vertical_rotor_1_pos = np.array([[0.5], [-0.5], [0.0]])#Port Front
vertical_rotor_2_pos = np.array([[-0.5], [-0.5], [0.0]])#Port Rear
vertical_rotor_3_pos = np.array([[-0.5], [0.5], [0.0]])#Starboard Rear
vertical_rotor_4_pos = np.array([[0.5], [0.5], [0.0]])#Starboard Front

#creates the array of all rotor positions
rotorPositions = [forward_rotor_pos, vertical_rotor_1_pos, vertical_rotor_2_pos, vertical_rotor_3_pos, vertical_rotor_4_pos]


#defines the normal vector for each propeller
forward_rotor_normal = np.array([[1.0], [0.0], [0.0]])

#sets the locations of the vertical propellers
vertical_rotor_1_normal = np.array([[0.0], [0.0], [-1.0]])#Port Front
vertical_rotor_2_normal = np.array([[0.0], [0.0], [-1.0]])#Port Rear
vertical_rotor_3_normal = np.array([[0.0], [0.0], [-1.0]])#Starboard Rear
vertical_rotor_4_normal = np.array([[0.0], [0.0], [-1.0]])#Starboard Front
#array of all normal vectors
normalVectors = [forward_rotor_normal, vertical_rotor_1_normal, vertical_rotor_2_normal, vertical_rotor_3_normal, vertical_rotor_4_normal, ]


#finds the cross products between the positions of each rotor,
#and the normal vector for each
leverMomentForward = (np.cross(forward_rotor_pos.T, forward_rotor_normal.T)).T
leverMomentV1 = (np.cross(vertical_rotor_1_pos.T, vertical_rotor_1_normal.T)).T
leverMomentV2 = (np.cross(vertical_rotor_2_pos.T, vertical_rotor_2_normal.T)).T
leverMomentV3 = (np.cross(vertical_rotor_3_pos.T, vertical_rotor_3_normal.T)).T
leverMomentV4 = (np.cross(vertical_rotor_4_pos.T, vertical_rotor_4_normal.T)).T

#gets the list of the lever moments
leverMoments = [leverMomentForward,leverMomentV1,leverMomentV2,leverMomentV3,leverMomentV4]

#defines the directions of each of the propellors
propDirections = np.array([-1.0,#forward propellor
                           1.0, #front port propeller
                           -1.0, #rear port propeller
                           1.0, #rear starboard propeller
                           -1.0]) #front starboard propeller


#some necessary parameters
M = 50.0
alpha0 = np.deg2rad(15) #0.47
epsilon = 0.16




#######################################################################################
#Coefficients
C_L_0 = 0.23#
C_D_0 = 0.043#
C_m_0 = 0.0135
C_L_alpha = 5.61#
C_D_alpha = 0.03#
C_m_alpha = -2.74
C_L_q = 7.95#
C_D_q = 0.0#
C_m_q = -38.21
C_L_delta_e = -0.13#
C_D_delta_e = 0.0135#
C_m_delta_e = -0.99
M = 50.0
alpha0 = 0.47
epsilon = 0.16
C_D_p = 0.0



C_Y_0 = 0.0#
C_ell_0 = 0.0
C_n_0 = 0.0
C_Y_beta = -0.98#
C_ell_beta = -0.13
C_n_beta = 0.073
C_Y_p = 0.0#
C_ell_p = -0.51
C_n_p = 0.069
C_Y_r = 0.0#
C_ell_r = 0.25
C_n_r = -0.095
C_Y_delta_a = 0.075#
C_ell_delta_a = 0.17
C_n_delta_a = -0.011
C_Y_delta_r = 0.19#
C_ell_delta_r = 0.0024
C_n_delta_r = -0.069



######################################################################################
                #  Propeller parameters
######################################################################################

# # Prop parameters
D_prop = 20*(0.0254)     # prop diameter in m


# Motor parameters
KV_rpm_per_volt = 145.                            # Motor speed constant from datasheet in RPM/V
KV = (1. / KV_rpm_per_volt) * 60. / (2. * np.pi)  # Back-emf constant, KV in V-s/rad
KQ = KV                                           # Motor torque constant, KQ in N-m/A
R_motor = 0.042              # ohms
i0 = 1.5                     # no-load (zero-torque) current (A)

k_force = 65.0
k_moment = 5.0


# Inputs
ncells = 12.
V_max = 3.7 * ncells  # max voltage for specified number of battery cells

#sets the maximum thrust
Tmax = 40

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

#   C values defines on pag 62
C_p_0         = gamma3 * C_ell_0      + gamma4 * C_n_0
C_p_beta      = gamma3 * C_ell_beta   + gamma4 * C_n_beta
C_p_p         = gamma3 * C_ell_p      + gamma4 * C_n_p
C_p_r         = gamma3 * C_ell_r      + gamma4 * C_n_r
C_p_delta_a    = gamma3 * C_ell_delta_a + gamma4 * C_n_delta_a
C_p_delta_r    = gamma3 * C_ell_delta_r + gamma4 * C_n_delta_r
C_r_0         = gamma4 * C_ell_0      + gamma8 * C_n_0
C_r_beta      = gamma4 * C_ell_beta   + gamma8 * C_n_beta
C_r_p         = gamma4 * C_ell_p      + gamma8 * C_n_p
C_r_r         = gamma4 * C_ell_r      + gamma8 * C_n_r
C_r_delta_a    = gamma4 * C_ell_delta_a + gamma8 * C_n_delta_a
C_r_delta_r    = gamma4 * C_ell_delta_r + gamma8 * C_n_delta_r
