import numpy as np
from numpy import concatenate
from scipy.linalg import solve_continuous_are, norm
from tools.dirty_derivative import DirtyDerivative
from tools.rotations import vee, hat, logR, leftJacobianInv
from message_types.quad.msg_state import MsgState
from message_types.quad.msg_delta import MsgDelta
from message_types.quad.msg_autopilot_quadRotor import MsgAutopilotQuadRotor

import parameters.quad.anaconda_parameters as QUAD



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
    def update(self, trajectory: MsgAutopilotQuadRotor, state: MsgState):

        #computes the errors
        pos_error = state.pos - trajectory.pos
        vel_error = state.vel - trajectory.vel
        e3 = np.array([[0.0],
                       [0.0],
                       [1.0]])
        
        #updates the positional integrator
        self.integrator_pos = self.integrator_pos + self.Ts/2 * (pos_error + self.pos_error_delay_1)
        self.pos_error_delay_1 = pos_error

        x_tilde = np.concatenate((pos_error, vel_error, self.integrator_pos), axis=0)

        #gets the desired force with the actual desired force, and then terms to cancel out
        #the gravity and the state drift term
        # TODO in the other controller, the QUAD.mass term was not there for the trajectory term.
        #I was assuming that accel just refers to acceleration and that it needs to be scaled
        #by mass to get the appropriate force
        f_d = trajectory.accel*QUAD.mass \
            - QUAD.mass * QUAD.gravity * e3 \
            -self.K_lqr @ x_tilde
        
        #gets the thrust
        thrust = saturate(norm(f_d), 0, QUAD.Tmax)
        k_d = - f_d / (norm(f_d)+0.01)
        s_d = np.array([[np.cos(trajectory.heading)], [np.sin(trajectory.heading)], [0.]])
        j_d = np.cross(k_d.T, s_d.T).T
        j_d = j_d / norm(j_d)
        i_d = np.cross(j_d.T, k_d.T).T
        R_d = concatenate((i_d, j_d, k_d), axis=1)
        # compute derivative of R
        R_d_dot = self.Rdot.update(R_d)
        omega_d = vee(0.5*(R_d.T @ R_d_dot - R_d_dot.T @ R_d))
        omega_d_dot = self.omegadot.update(omega_d)
        r_tilde = vee(logR(state.rot.T @ R_d))
        omega_tilde = state.rot.T @ R_d @ omega_d - state.omega
        torque = hat(state.omega) @ QUAD.J @ state.omega \
                 + QUAD.J @ omega_d_dot \
                 + leftJacobianInv(r_tilde).T @ self.Kr @ r_tilde \
                 + self.Komega @ omega_tilde

        # construct output and commanded states
        delta = MsgDelta(force=thrust, torque=torque)
        commanded_state = MsgState(pos=trajectory.pos,
                                   vel=trajectory.vel,
                                   rot=R_d,
                                   omega=omega_d)
        
        return delta, commanded_state





def saturate(input: float, low_limit: float, up_limit: float):
    if input <= low_limit:
        output = low_limit
    elif input >= up_limit:
        output = up_limit
    else:
        output = input
    return output


