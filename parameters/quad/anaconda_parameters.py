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
