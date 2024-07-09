
#%%

import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[3]))

import scipy.optimize as spo
import numpy as np


from message_types.quad.msg_delta import MsgDelta
from message_types.quad.msg_state import MsgState

from tools.rotations import quaternion_to_rotation

import parameters.quad.anaconda_parameters as QUAD


import parameters.simulation_parameters as SIM

from controllers.quad.pid_control import PControl

from controllers.quad.tests.optimizationTestClass import OptimizationTest

kp = 0.15
Ts=SIM.ts_control


#creates an arbitrary desired wrench
Wrench_D = np.array([[0.0],
                     [0.0],
                     [0.0],
                     [0.0],
                     [0.0],
                     [0.0]])



#defines the delta_a, e, and r bounds
delta_a_bound = (-1.0, 1.0)
delta_e_bound = (-1.0, 1.0)
delta_r_bound = (-1.0, 1.0)
delta_t_forward = (0.0, 0.0)
delta_t_vertical_1 = (0.0, 0.0)
delta_t_vertical_2 = (0.0, 0.0)
delta_t_vertical_3 = (0.0, 0.0)
delta_t_vertical_4 = (0.0, 0.0)


#puts them together
delta_bounds = (delta_a_bound, delta_e_bound, delta_r_bound, delta_t_forward, delta_t_vertical_1, delta_t_vertical_2, delta_t_vertical_3, delta_t_vertical_4)


pos = np.array([[0.00421417], [0.00023018], [0.00346456]])
vel = np.array([[25.0], [0.00517804], [-0.01385525]])
R = np.array([[0.9999275], [0.00511143], [0.00059805], [0.01088625]])
omega = np.array([[-0.02315855], [0.02411054], [0.43522908]])



#instantiates the optimization test class
optimizer = OptimizationTest(ts = SIM.ts_simulation)

optimizer._update_state(pos=pos,
                        vel=vel,
                        R=R,
                        omega=omega)


#creates the initial delta with all zeros input
delta_initial = MsgDelta()

wind = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

optimizer.update(delta=delta_initial, wind=wind)

x0 = [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

#gets the solution
spo.minimize(optimizer.meanSquaredError,x0, method='SLSQP', bounds=delta_bounds)

#x_output = solution.x
#print(x_output)

#gets the forces and torques from the x output
#forcesTorques = 



# %%
