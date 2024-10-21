#implements the dynamics for the quadplane
import numpy as np
from message_types.msg_state import MsgState
import parameters.anaconda_parameters as QUAD
from tools.rotations import quaternion_to_rotation
from tools.rotations import quaternion_to_euler
from message_types.msg_delta import MsgDelta


class QuadDynamics:
    #creates the initialization function
    def __init__(self, Ts: float):
        self._ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        # We will also need a variety of other elements that are functions of the _state and the wind.
        # self.true_state is a 19x1 vector that is estimated and used by the autopilot to control the aircraft:
        # true_state = [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        self._state = np.array([[QUAD.north0],  # (0)
                               [QUAD.east0],   # (1)
                               [QUAD.down0],   # (2)
                               [QUAD.u0],    # (3)
                               [QUAD.v0],    # (4)
                               [QUAD.w0],    # (5)
                               [QUAD.e0],    # (6)
                               [QUAD.e1],    # (7)
                               [QUAD.e2],    # (8)
                               [QUAD.e3],    # (9)
                               [QUAD.p0],    # (10)
                               [QUAD.q0],    # (11)
                               [QUAD.r0]    # (12)
                               ])
        # initialize true_state message
        self.true_state = MsgState()

        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec
        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        self._Va = QUAD.u0
        self._alpha = 0
        self._beta = 0
        # update velocity data and forces and moments
        self._update_velocity_data()
        self._forces_moments(delta=MsgDelta())
        # update the message class for the true state
        self._update_true_state()




    #creates the rk4 step
    def _rk4_step(self, forces_moments: np.ndarray):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = self._f(self._state[0:13], forces_moments)
        k2 = self._f(self._state[0:13] + time_step/2.*k1, forces_moments)
        k3 = self._f(self._state[0:13] + time_step/2.*k2, forces_moments)
        k4 = self._f(self._state[0:13] + time_step*k3, forces_moments)
        self._state[0:13] += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)
        # normalize the quaternion
        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[6][0] = self._state.item(6)/normE
        self._state[7][0] = self._state.item(7)/normE
        self._state[8][0] = self._state.item(8)/normE
        self._state[9][0] = self._state.item(9)/normE


    #creates the derivatives function
    def _f(self, state: np.ndarray, forces_moments: np.ndarray)->np.ndarray:
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        # north = state.item(0)
        # east = state.item(1)
        # down = state.item(2)
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        e0 = state.item(6)
        e1 = state.item(7)
        e2 = state.item(8)
        e3 = state.item(9)
        p = state.item(10)
        q = state.item(11)
        r = state.item(12)
        #   extract forces/moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)
        l = forces_moments.item(3)
        m = forces_moments.item(4)
        n = forces_moments.item(5)

        # position kinematics
        pos_dot = quaternion_to_rotation(state[6:10]) @ state[3:6]
        north_dot = pos_dot.item(0)
        east_dot = pos_dot.item(1)
        down_dot = pos_dot.item(2)

        # position dynamics
        u_dot = r*v - q*w + fx/QUAD.mass
        v_dot = p*w - r*u + fy/QUAD.mass
        w_dot = q*u - p*v + fz/QUAD.mass

        # rotational kinematics
        e0_dot = 0.5 * (-p*e1 - q*e2 - r*e3)
        e1_dot = 0.5 * (p*e0 + r*e2 - q*e3)
        e2_dot = 0.5 * (q*e0 - r*e1 + p*e3)
        e3_dot = 0.5 * (r*e0 + q*e1 -p*e2)

        # rotatonal dynamics
        p_dot = QUAD.gamma1*p*q - QUAD.gamma2*q*r + QUAD.gamma3*l + QUAD.gamma4*n
        q_dot = QUAD.gamma5*p*r - QUAD.gamma6*(p**2-r**2) + m/QUAD.Jy
        r_dot = QUAD.gamma7*p*q - QUAD.gamma1*q*r + QUAD.gamma4*l + QUAD.gamma8*n

        # collect the derivative of the states
        x_dot = np.array([[north_dot, east_dot, down_dot, u_dot, v_dot, w_dot,
                           e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot]]).T
        return x_dot
    

    def _update_true_state(self):
        # update the class structure for the true state:
        #   [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        phi, theta, psi = quaternion_to_euler(self._state[6:10])
        pdot = quaternion_to_rotation(self._state[6:10]) @ self._state[3:6]
        self.true_state.pos[0][0] = self._state.item(0)
        self.true_state.pos[0][0] = self._state.item(1)
        self.true_state.pos[0][0] = self._state.item(2)
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta
        self.true_state.Vg = np.linalg.norm(pdot)
        self.true_state.chi = np.arctan2(pdot.item(1), pdot.item(0))
        self.true_state.omega[0][0] = self._state.item(10)
        self.true_state.omega[0][0] = self._state.item(11)
        self.true_state.omega[0][0] = self._state.item(12)
        self.true_state.wn = self._wind.item(0)
        self.true_state.we = self._wind.item(1)
        self.true_state.bx = 0
        self.true_state.by = 0
        self.true_state.bz = 0
        self.true_state.gimbal_az = 0
        self.true_state.gimbal_el = 0