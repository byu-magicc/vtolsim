"""
Parameters for geometric controller
"""
import numpy as np
from geometric_control.optimal_pitch import AERO_TYPE, OPTIMAL_PITCH_METHOD

Kp = np.diag([3., 3., 5.])
Kd = np.diag([2., 2., 2.])
Ki = 0.*np.diag([.1, .1, .1])
omega_Kp = 5.*np.diag([1., 1., 1.])

perr_d0_sat = 5.

delta_theta_max = .0005 # rad/step

aero_type = AERO_TYPE.BLENDED_2

optimal_pitch_method = OPTIMAL_PITCH_METHOD.Sampled
