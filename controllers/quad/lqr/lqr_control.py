import numpy as np
import scipy
from message_types.msg_convert import *
from controllers.quad.lqr.lqr_dynamics import es_jacobians
from tools.quaternions import state_boxMinus, state_boxPlus



#creates the Lqr control class
class LqrControl:

    #creates the initialization fnction
    def __init__(self, Ts):
        self.Ts = Ts

        #saves the number of actuators
        numActuators = 8
        #stores the previous input
        self.u_prev = np.zeros((numActuators, 1))
        #saves the alpha
        self.alpha = 0.5
        #saves the epsilon
        self.epsilon = 0.001

        #creates the Q matrix
        self.Q = np.diag([1/10.0, # north position
                          1/10.0, # east position
                          1/10.0, # down position
                          1/10.0, # x body velocity
                          1/10.0, # y body velocity
                          1/10.0, # z body velocity
                          2.0,    # q_tilde 0
                          2.0,    # q_tilde 1
                          2.0])   # q_tilde 2
        
        #creates the R matrix
        self.R = np.diag([1.0,
                          1.0,
                          1.0,
                          1.0,
                          1.0,
                          1.0,
                          1.0,
                          1.0])
        
    #creates the update function
    def update(self, x, x_desired, u_desired, df_traj):

        #gets the error for the state
        x_tilde = state_boxMinus(x_desired, x)

        #gets the u_tilde
        u_tilde = u_desired - self.u_prev

        