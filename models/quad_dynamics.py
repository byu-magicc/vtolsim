import numpy as np
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
import parameters.anaconda_parameters as QUAD
from tools.rotations import quaternion_to_rotation, quaternion_to_euler


#defines the dynamics for the quadplane
class QuadDynamics:
    #creates the init function
    def __init__(self, Ts: float):
        self.ts_simulation = Ts


        #creates the state vector
        self.state_vector = np.array([[QUAD.north0],
                                      [QUAD.east0],
                                      [QUAD.down0],
                                      [QUAD.u0],
                                      [QUAD.v0],
                                      [QUAD.w0],
                                      [QUAD.e0],
                                      [QUAD.e1],
                                      [QUAD.e2],
                                      [QUAD.e3],
                                      [QUAD.p0],
                                      [QUAD.q0],
                                      [QUAD.r0]])
        
        #creates variable for the true state
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

        #saves the forces and the moments
        self.forces_moments = np.zeros((6, 1))


    #creates the update function
    def update(self, delta: MsgDelta, wind: np.ndarray):
        self.forces_moments = self._forces_moments(delta)




    #creates function to obtain the forces and moments for the system
    def _forces_moments(self, delta: MsgDelta) -> np.ndarray: 
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        #phi, theta, psi = quaternion_to_euler(self._state[6:10])
        p = self._state.item(10)
        q = self._state.item(11)
        r = self._state.item(12)

        # compute gravitational forces
        R = quaternion_to_rotation(self._state[6:10]) # rotation from body to world frame
        f_g = R.T @ np.array([[0.], [0.], [QUAD.mass * QUAD.gravity]])
        fx = f_g.item(0)
        fy = f_g.item(1)
        fz = f_g.item(2)
        if self.counter % 21 == 0:
            a = 0

        # intermediate variables
        qbar = 0.5 * QUAD.rho * self._Va**2
        ca = np.cos(self._alpha)
        sa = np.sin(self._alpha)
        p_nondim = p * QUAD.b / (2 * self._Va)  # nondimensionalize p
        q_nondim = q * QUAD.c / (2 * self._Va)  # nondimensionalize q
        r_nondim = r * QUAD.b / (2 * self._Va)  # nondimensionalize r

        # compute Lift and Drag coefficients
        tmp1 = np.exp(-QUAD.M * (self._alpha - QUAD.alpha0))
        tmp2 = np.exp(QUAD.M * (self._alpha + QUAD.alpha0))
        sigma = (1 + tmp1 + tmp2) / ((1 + tmp1) * (1 + tmp2))
        CL = (1 - sigma) * (QUAD.C_L_0 + QUAD.C_L_alpha * self._alpha) \
             + sigma * 2 * np.sign(self._alpha) * sa**2 * ca
        CD = QUAD.C_D_p + ((QUAD.C_L_0 + QUAD.C_L_alpha * self._alpha)**2)/(np.pi * QUAD.e * QUAD.AR)
        # compute Lift and Drag Forces
        F_lift = qbar * QUAD.S_wing * (
                CL
                + QUAD.C_L_q * q_nondim
                + QUAD.C_L_delta_e * delta.elevator
        )
        F_drag = qbar * QUAD.S_wing * (
                CD
                + QUAD.C_D_q * q_nondim
                + QUAD.C_D_delta_e * delta.elevator
        )
        # compute longitudinal forces in body frame
        fx = fx - ca * F_drag + sa * F_lift
        fz = fz - sa * F_drag - ca * F_lift
        # compute lateral forces in body frame
        fy += qbar * QUAD.S_wing * (
                QUAD.C_Y_0
                + QUAD.C_Y_beta * self._beta
                + QUAD.C_Y_p * p_nondim
                + QUAD.C_Y_r * r_nondim
                + QUAD.C_Y_delta_a * delta.aileron
                + QUAD.C_Y_delta_r * delta.rudder
        )

        if self.counter % 21 == 0:
            a = 0
        # compute logitudinal torque in body frame
        My = qbar * QUAD.S_wing * QUAD.c * (
                QUAD.C_m_0
                + QUAD.C_m_alpha * self._alpha
                + QUAD.C_m_q * q_nondim
                + QUAD.C_m_delta_e * delta.elevator
        )

        if self.counter % 20 == 0:
            a=0
        # compute lateral torques in body frame
        Mx = qbar * QUAD.S_wing * QUAD.b * (
                QUAD.C_ell_0
                + QUAD.C_ell_beta * self._beta
                + QUAD.C_ell_p * p_nondim
                + QUAD.C_ell_r * r_nondim
                + QUAD.C_ell_delta_a * delta.aileron
                + QUAD.C_ell_delta_r * delta.rudder
        )
        Mz = qbar * QUAD.S_wing * QUAD.b * (
                QUAD.C_n_0 + QUAD.C_n_beta * self._beta
                + QUAD.C_n_p * p_nondim
                + QUAD.C_n_r * r_nondim
                + QUAD.C_n_delta_a * delta.aileron
                + QUAD.C_n_delta_r * delta.rudder
        )

        thrust_prop, torque_prop = self._motor_thrust_torque(self._Va, delta.throttle)
        fx += thrust_prop
        Mx += -torque_prop

        self._forces[0] = fx
        self._forces[1] = fy
        self._forces[2] = fz

        if self.counter % 1000 == 0:
            thrust = thrust_prop
            a = 0

        self.counter += 1

        return np.array([[fx, fy, fz, Mx, My, Mz]]).T


    #creates the rk4 function
    def _rk4_step(self, forces_moments: np.ndarray):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self.ts_simulation
        k1 = self._f(self.state_vector[0:13], forces_moments)
        k2 = self._f(self.state_vector[0:13] + time_step/2.*k1, forces_moments)
        k3 = self._f(self.state_vector[0:13] + time_step/2.*k2, forces_moments)
        k4 = self._f(self.state_vector[0:13] + time_step*k3, forces_moments)
        self.state_vector[0:13] += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)
        # normalize the quaternion
        e0 = self.state_vector.item(6)
        e1 = self.state_vector.item(7)
        e2 = self.state_vector.item(8)
        e3 = self.state_vector.item(9)
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self.state_vector[6][0] = self.state_vector.item(6)/normE
        self.state_vector[7][0] = self.state_vector.item(7)/normE
        self.state_vector[8][0] = self.state_vector.item(8)/normE
        self.state_vector[9][0] = self.state_vector.item(9)/normE


    #creates the function to obtain the state derivatives
    def _f(self, state_vector: np.ndarray, forces_moments: np.ndarray)->np.ndarray:
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        # north = state.item(0)
        # east = state.item(1)
        # down = state.item(2)
        u = state_vector.item(3)
        v = state_vector.item(4)
        w = state_vector.item(5)
        e0 = state_vector.item(6)
        e1 = state_vector.item(7)
        e2 = state_vector.item(8)
        e3 = state_vector.item(9)
        p = state_vector.item(10)
        q = state_vector.item(11)
        r = state_vector.item(12)
        #   extract forces/moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)
        l = forces_moments.item(3)
        m = forces_moments.item(4)
        n = forces_moments.item(5)

        # position kinematics
        pos_dot = quaternion_to_rotation(state_vector[6:10]) @ state_vector[3:6]
        north_dot = pos_dot.item(0)
        east_dot = pos_dot.item(1)
        down_dot = pos_dot.item(2)

        # position dynamics
        u_dot = r*v - q*w + fx/QUAD.mass
        v_dot = p*w - r*u + fy/QUAD.mass
        w_dot = q*u - p*v + fz/QUAD.mass

        self.udot = u_dot
        self.vdot = v_dot
        self.wdot = w_dot

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



    #creates helper function to get the state vector
    def getStateVector(self):
        return self.state_vector