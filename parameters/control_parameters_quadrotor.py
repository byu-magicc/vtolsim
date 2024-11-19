"""
quadSim
    - Chapter 14 assignment for Beard & McLain, PUP, 2012
    - Update history:
        7/7/2021 - RWB
        4/26/2022 - RWB
        7/13/2023 - RWB
"""
import numpy as np
import parameters.anaconda_parameters as QUAD

gravity = QUAD.gravity  # gravity constant
mass = QUAD.mass
Cd_prop = QUAD.C_drag_prop
rho = QUAD.rho

# ----------saturation limits-------------
xy_torque_limit = 200.0
z_torque_limit = 100.0
roll_pitch_angle_limit = np.radians(30)

# ----------roll loop-------------
wn_roll = 10.0
zeta_roll = 0.707
roll_kp = QUAD.Jx * wn_roll**2
roll_kd = QUAD.Jx * 2 * zeta_roll * wn_roll

# ----------pitch loop-------------
wn_pitch = wn_roll
zeta_pitch = zeta_roll
pitch_kp = QUAD.Jy * wn_pitch**2
pitch_kd = QUAD.Jy * 2 * zeta_pitch * wn_pitch


# ----------yaw loop-------------
wn_yaw = 2.
zeta_yaw = 0.707
yaw_kp = QUAD.Jz * wn_yaw**2
yaw_kd = QUAD.Jz * 2 * zeta_yaw * wn_yaw
yaw_ki = 1.0

# ----------north loop-------------
wn_north = 0.5
zeta_north = 0.707
north_kp = wn_north**2
north_kd = 2 * zeta_north * wn_north
north_ki = 0.05

# ----------east loop-------------
wn_east = wn_north
zeta_east = zeta_north
east_kp = wn_east**2
east_kd = 2 * zeta_east * wn_east
east_ki = north_ki

# ----------down loop-------------
wn_down = 2.0 * wn_north
zeta_down = 0.707
down_kp = wn_down**2
down_kd = 2 * zeta_down * wn_down
down_ki = 0.01
