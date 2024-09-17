#this file computes our objective gradient for use in the optimization algorithm
from ThrustTorquePartial import getThrustTorqueGradient

import sympy as sp
import numpy as np
import copy

from message_types.quad.msg_state import MsgState
from models.quad.quad_dynamics import QuadDynamics

#function that gets the gradient of the objective function
#arguments:
#1. Airspeed: 3x1 array, airspeed of the aircraft in the body frame, relative to the airmass it is passing through
#2. State: the current state of the system
#3. a copy of the current class

def getObjectiveGrad(Airspeed: np.ndarray, state: MsgState, quad_in: QuadDynamics):

    #makes a copy of the quad
    quad = copy.copy(quad_in)

    

