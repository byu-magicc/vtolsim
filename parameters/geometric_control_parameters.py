"""
Parameters for geometric controller
"""

import sys
sys.path.append('..')
import numpy as np


Kp = np.diag([3., 3., 5.])
Kd = np.diag([2., 2., 2.])
Ki = 0.*np.diag([.1, .1, .1])
omega_Kp = 5.*np.diag([1., 1., 1.])


#copy of the original tuning parameters:
'''
Kp = np.diag([3., 3., 5.])
Kd = np.diag([2., 2., 2.])
Ki = 0.*np.diag([.1, .1, .1])
omega_Kp = 5.*np.diag([1., 1., 1.])
#'''



#other set of references, which I do not as of yet understand.
# Kp = .1*np.diag([3., 3., 5.])
# Kd = .1*np.diag([2., 2., 2.])
# Ki = 0.*np.diag([.1, .1, .1])
# omega_Kp = 0.1*5.*np.diag([1., 1., 1.])

perr_d0_sat = 5.

delta_theta_max = .0005 # rad/step