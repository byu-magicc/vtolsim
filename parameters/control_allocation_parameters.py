import numpy as np

# weighting matrix for minimization of difference between desired and achieved force/torque
K_Tau = np.eye(5)

# weighting matrix for minimization of difference between optimal and necessary actuator setpoint
def K_delta(airspeed):
    return np.eye(7) * np.array([[1, 1, 10, 3, 3, 0.0, 0.0]]).T * 1e-6 * airspeed**2

# initial actuator guess
init_actuators = np.array([0.7556, 0.7556, 0.9235, 1.3837, 1.4544, 0.0, 0.0])

# minimum-energy actuator setpoints
actuators_desired = np.zeros(7)

# max iterations for nonlinear solver
max_iter = 15

mixer = np.array([  #convention for elevon flaps is down is positive
    [   0.0,   1.0,   0.0,   1.0,   0.0,   0.0,   0.0],  #Fx
    [   1.1,   0.0,   0.9,   0.0,   0.9,   0.0,   0.0],  #Fz
    [   0.0,   0.0,  -1.0,   0.0,   1.0,  -1.0,   1.0],  #Tx
    [  -1.0,   0.0,   1.0,   0.0,   1.0,  -1.0,  -1.0],  #Ty
    [   0.0,  -1.0,   0.0,   1.0,   0.0,   0.0,   0.0], #Tz
])
#will later be scaled by sig(Va)
#              t_re, t_ri*c(dr), t_ri*s(dr), t_l*c(dl), t_l*s(dl), er, el
#
# mixer = np.array([[   0.0,   1.0,   0.0,   1.0,   0.0,   0.0,   0.0],  #Fx
#                   [   1.1,   0.0,   0.9,   0.0,   0.9,   0.0,   0.0],  #Fz
#                   [   0.0,   0.0,  -1.0,   0.0,   1.0,  -1.0,   1.0],  #Tx
#                   [  -1.0,   0.0,   1.0,   0.0,   1.0,  -3.0,  -3.0],  #Ty
#                   [   0.0,  -1.0,   0.0,   1.0,   0.0,   0.0,   0.0]]) #Tz
limits = np.array([
    [0.0, 1.0],
    [0.0, 1.0],
    [0.0, 1.0],
    [np.radians(0.0), np.radians(115.0)],
    [np.radians(0.0), np.radians(115.0)],
    [np.radians(-45.0), np.radians(45.0)],
    [np.radians(-45.0), np.radians(45.0)],
]).T            
