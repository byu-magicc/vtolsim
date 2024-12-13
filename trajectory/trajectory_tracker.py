import numpy as np
import sys
sys.path.append('..')

import parameters.anaconda_parameters as QUAD
import parameters.geometric_control_parameters as CTRL
from copy import copy

from message_types.msg_state import MsgState
from message_types.msg_trajectory

class TrajectoryTracker:

    #creates the initialization function
    def __init__(self):
        self.pos_errs = []
        self.vel_errs = []
        self.F_ds = []
        self.R = []

    #creates the update function
    #arguments:
    #1. State: the vector low level state of the plane
    #2. trajectory: the current desired 
    #position, velocity, acceleration, and yaw from the B Spline generator
    def update(self, state: np.ndarray, trajectory: np.ndarray):

        #gets the current position of the body in the 