"""
convergence_parameters
    - This file contains the aerodynamic parameters for the convergence aircraft.
    (https://www.horizonhobby.com/product/airplanes/airplanes-14501--1/bind-n-fly/convergence-vtol-bnf-basic-efl11050)
    - The current numbers are just the aerosonde parameters, with a few modifications
    - Need to do some sysid to correctly identify the convergence parameters.

    - Update history:
        5/8/2019 - R.W. Beard
"""
import numpy as np
from tools.rotations import euler_to_quaternion, quaternion_to_rotation

######################################################################################
                #   Initial Conditions
######################################################################################
#   Initial conditions for vtol
pos0 = np.array([[0.], [0.], [0.]])  # initial NED position
vel0 = np.array([[0.], [0.], [0.]])  # initial inertial velocity in body frame
phi0 = 0.0  # initial roll angle
theta0 = 0.0  # initial pitch angle
psi0 = 0.0  # initial yaw angle
quat0 = euler_to_quaternion(phi0, theta0, psi0)  # initial attitude
omega0 = np.array([[0.], [0.], [0.]])  # initial angular velocity\
gyro_bias0 = np.array([[0.], [0.], [0.]])

rotor_angle_right0 = np.radians(90) # angle of right rotor
rotor_angle_left0 = np.radians(90) # angle of left rotor

Tmax = 22.6

######################################################################################
                #   Physical Parameters
######################################################################################
######Convergence#######
mass = 1.0 #kg
Jx = 0.0165#0.008383 #kg m^2
Jy = 0.025#0.006114
Jz = 0.0282#0.01435
Jxz = 0.000048#0.000024
J = np.array([[Jx, 0, -Jxz], [0, Jy, 0], [-Jxz, 0, Jz]])
Jinv = np.linalg.inv(J)
S_wing = 0.2589#0.19#0.55
b = 1.4224#0.65#2.8956
c = 0.3305#0.25#0.18994

########Zagi########
S_prop = 0.2027
rho = 1.2682
e = 0.9
e_oswald = e
AR = (b**2) / S_wing
AR_wing = AR
gravity = 9.81

# physical position of rotors in units of meters
rear_rotor_pos = np.array([[-0.24], [0.0], [0.0]])
right_rotor_pos = np.array([[0.12], [0.2], [0.0]])
left_rotor_pos = np.array([[0.12], [-0.2], [0.0]])

######################################################################################
                #   Longitudinal Coefficients
######################################################################################
######Convergence#######
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

M = 50.0
alpha0 = np.deg2rad(15) #0.47
epsilon = 0.16
C_D_p = 0.015# Parasitic Drag


######################################################################################
                #   Lateral Coefficients
######################################################################################
C_Y_0 = 0.0
C_ell_0 = 0.0
C_n_0 = 0.0

######Convergence#######
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
                #   Propeller thrust / torque parameters (see addendum by McLain)
######################################################################################
######Aeorsonde#######
# # Prop parameters
D_prop = 20*(0.0254)     # prop diameter in m

# Motor parameters
K_V = 145.                   # from datasheet RPM/V
KQ = (1. / K_V) * 60. / (2. * np.pi)  # KQ in N-m/A, V-s/rad
R_motor = 0.042              # ohms
i0 = 1.5                     # no-load (zero-torque) current (A)

k_force = 65.0
k_moment = 5.0

# Coeffiecients from prop_data fit
C_Q2 = -0.01664
C_Q1 = 0.004970
C_Q0 = 0.005230
C_T2 = -0.1079
C_T1 = -0.06044
C_T0 = 0.09357

############################# Rear ##################################
######Convergence#######
# Prop parameters
D_prop_rear = 5.5*(0.0254)    # prop diameter in m

# Motor parameters
K_V_rear = 1550.                   # from datasheet RPM/V
KQ_rear = (1. / K_V_rear) * 60. / (2. * np.pi)  # KQ in N-m/A, V-s/rad
R_motor_rear = 0.4              # ohms
i0_rear = 0.6                     # no-load (zero-torque) current (A)

# Inputs
ncells = 6.
V_max = 3.7 * ncells  # max voltage for specified number of battery cells

# Coeffiecients from prop_data fit
C_Q2_rear = -0.0368
C_Q1_rear = 0.0292
C_Q0_rear = 0.0216
C_T2_rear = -0.1921
C_T1_rear = 0.0505
C_T0_rear = 0.2097

############################# Front Left/Right ##################################
######Convergence#######
# Prop parameters
D_prop_front = 7.0*(0.0254)#20*(0.0254)     # prop diameter in m

# Motor parameters
K_V_front = 1450.                   # from datasheet RPM/V
KQ_front = (1. / K_V_front) * 60. / (2. * np.pi)  # KQ in N-m/A, V-s/rad
R_motor_front = 0.3              # ohms
i0_front = 0.83                     # no-load (zero-torque) current (A)

# Coeffiecients from prop_data fit
C_Q2_front = -0.0216
C_Q1_front = 0.0129
C_Q0_front = 0.0088
C_T2_front = -0.1480
C_T1_front = 0.0144
C_T0_front = 0.1167

######################################################################################
                #   Front motor servo parameters
######################################################################################

#First order servo model alpha_dot = k*(delta-alpha)
k_servo = 10.0
servo_min = np.radians(0.0)
servo_max = np.radians(115.0)

######################################################################################
                #   Elevon parameters
######################################################################################
elevon_min = np.radians(-45.0)
elevon_max = np.radians(45.0)

######################################################################################
                #   Calculated Variables
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
