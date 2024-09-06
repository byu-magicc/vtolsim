import numpy as np
from numpy import concatenate
from scipy.linalg import solve_continuous_are, norm
from tools.dirty_derivative import DirtyDerivative
from tools.rotations import vee, hat, logR, leftJacobianInv
from message_types.quad.msg_state import MsgState
from message_types.quad.msg_delta import MsgDelta
from message_types.quad.msg_autopilot_quadRotor import MsgAutopilotQuadRotor



#creates  the autopilot message class
class Autopilot:

    #creates the init function
    def __init__(self, ts_control: float):
        self.Ts = ts_control
        #creates the position integrator
        self.integrator_pos = np.zeros((3, 1))
        #creates the error delayed by 1
        self.pos_error_delay_1 = np.zeros((3, 1))

        #computes the lqr gains
        Id = np.eye(3)
        Zero = np.zeros((3,3))

        #creates the A matrix
        A = np.concatenate((np.concatenate((Zero, Id, Zero), axis=1),
                            np.concatenate((Zero, Zero, Zero), axis=1),
                            np.concatenate((Id, Zero, Zero), axis=1)), axis=0)
        
        B = np.concatenate((Zero, Id, Zero), axis=0)
        #sets the Lqr gains
        qp = 10.0
        qd = 20.0
        qi = 0.1
        qf = 1.0
        #creates the lqr K matrix
        Q = np.diag([qp, qp, qp, qd, qd, qd, qi, qi, qi])
        R = qf * np.diag([1., 1., 1.])
        P = solve_continuous_are(A, B, Q, R)
        self.K_lqr = (1/qf) * B.T @ P

        #creates the dirty derivatives
        self.Rdot = DirtyDerivative(Ts=self.Ts, tau=5*self.Ts)
        self.omegadot = DirtyDerivative(Ts=self.Ts, tau=5*self.Ts)
        # gains for Jake Johnson's controller
        self.Kr = 20*np.diag([1., 1., 1.])
        self.Komega = 20*np.diag([1., 1., 1.])


    #creates the update function
    def update(self, trajectory, state)
