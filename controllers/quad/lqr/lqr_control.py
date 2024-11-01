#creates the two seperate lqr controllers. I am going to do two things as preliminaries to making the 
#whole simulation thing run. In this case, I am going to create two LQR controllers, one for VTOL and
#one for the standard plane. I first want to get the simulation running on each of them individually
#before I start building the rest of the controller

import numpy as np
from scipy.linalg import solve_continuous_are, inv
from tools.transfer_function import TransferFunction
import parameters.quad.fixed_wing_lqr_parameters as AP


#creates an LQR controller 
#the purpose of this controller is to provide us with a desired force and torque
#which will in turn be fed into the low level controller.

class LqrControl:
    def __init__(self, ts_control):

        #saves the time sample
        self.Ts = ts_control
        #creates a 


