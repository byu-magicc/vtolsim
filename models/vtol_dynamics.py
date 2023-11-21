"""
vtolDynamics
    - this file implements the dynamic equations of motion for VTOL
    - use unit quaternion for the attitude state

    - Update history:
        5/8/2019 - R.W. Beard
        11/20/2023 - RWB
"""
import numpy as np
import numpy as np
import parameters.convergence_parameters as VTOL
from tools.rotations import quaternion_to_rotation, quaternion_to_euler
from tools.rotations import hat, quat_hat

# load message types
from message_types.msg_state import MsgState

class VtolDynamics:
    def __init__(self, Ts):
        self._ts = Ts  # simulation sample rate
        # internal state (quaternion)
        self._state = np.array([
            [VTOL.pos0.item(0)],  # [0]  north position
            [VTOL.pos0.item(1)],  # [1]  east position
            [VTOL.pos0.item(2)],  # [2]  down position
            [VTOL.vel0.item(0)],   # [3]  velocity along body x-axis
            [VTOL.vel0.item(1)],   # [4]  velocity along body y-axis
            [VTOL.vel0.item(2)],   # [5]  velocity along body z-axis
            [VTOL.quat0.item(0)],  # [6]  quaternion - scalar part
            [VTOL.quat0.item(1)],  # [7]  quaternion - vector-x
            [VTOL.quat0.item(2)],  # [8]  quaternion - vector-y
            [VTOL.quat0.item(3)],  # [9]  quaternion - vector-z
            [VTOL.omega0.item(0)], # [10]  roll rate
            [VTOL.omega0.item(1)], # [11]  pitch rate
            [VTOL.omega0.item(2)], # [12]  yaw rate
            [VTOL.rotor_angle_right0], # [13] pitch angle of right motor
            [VTOL.rotor_angle_left0],  # [14] pitch angle of left motor
        ])
        self.state = MsgState()
        self._update_true_state()
        self._forces = np.array([[0.], [0.], [0.]])

        # # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec



    ###################################
    # public functions
    def update(self, delta, wind):
        '''
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_ar, delta_al, delta_tb, delta_tr, delta_tl) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        '''
        # get forces and moments acting on rigid bod
        force_torque = self._forces_moments(delta)
        # Integrate ODE using Runge-Kutta RK4 algorithm
        k1 = self._f(self._state, force_torque, delta)
        k2 = self._f(self._state + self._ts/2.*k1, force_torque, delta)
        k3 = self._f(self._state + self._ts/2.*k2, force_torque, delta)
        k4 = self._f(self._state + self._ts*k3, force_torque, delta)
        self._state += self._ts/6 * (k1 + 2*k2 + 2*k3 + k4)
        # normalize the quaternion
        norm_quat = np.linalg.norm(self._state[6:10])
        self._state[6][0] /= norm_quat
        self._state[7][0] /= norm_quat
        self._state[8][0] /= norm_quat
        self._state[9][0] /= norm_quat
        # update the message class for the true state
        self._update_true_state(wind)

    def set_state(self, new_state):
        self._state = new_state
        self._update_true_state()

    ###################################
    # private functions
    def _f(self, state, force_torque, delta):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        pos = state[0:3]
        vel = state[3:6]
        quat = state[6:10]
        omega = state[10:13]
        motor_angle = state[13:15]
        R = quaternion_to_rotation(quat)  # body to inertial
        # computer forces and torques
        force = force_torque[0:3]
        torque = force_torque[3:6]
        # equations or motion
        pos_dot = R @ vel
        vel_dot = np.cross(vel.T, omega.T).T + force / VTOL.mass
        quat_dot = 0.5 * quat_hat(omega) @ quat
        omega_dot = VTOL.Jinv @ (-hat(omega) @ VTOL.J @ omega + torque)
        motor_cmd = np.array([[delta.motor_right], [delta.motor_left]])
        motor_angle_dot = VTOL.k_servo * (motor_cmd - motor_angle)
        # collect the derivative of the states
        x_dot = np.concatenate((pos_dot, vel_dot, 
                                quat_dot, omega_dot, 
                                motor_angle_dot), axis=0)
        return x_dot

    def _forces_moments(self, delta):
        """
        return the forces on the VTOL based on the state, wind, and control surfaces
        :return: Forces and torques on the VTOL np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        u = self._state.item(3)
        v = self._state.item(4)
        w = self._state.item(5)
        phi, theta, psi = quaternion_to_euler(self._state[6:10])
        p = self._state.item(10)
        q = self._state.item(11)
        r = self._state.item(12)
        # compute gravitaional forces
        # rotation from body to world frame
        R = quaternion_to_rotation(self._state[6:10]) # rotation from body to world frame
        f_g = R.T @ np.array([[0.], [0.], [VTOL.mass * VTOL.gravity]])
        fx = f_g.item(0)
        fy = f_g.item(1)
        fz = f_g.item(2)
        # intermediate variables
        qbar = 0.5 * VTOL.rho * self.state.Va**2
        ca = np.cos(self.state.alpha)
        sa = np.sin(self.state.alpha)
        if self.state.Va > 1:
            p_nondim = p * VTOL.b / (2 * self.state.Va)  # nondimensionalize p
            q_nondim = q * VTOL.c / (2 * self.state.Va)  # nondimensionalize q
            r_nondim = r * VTOL.b / (2 * self.state.Va)  # nondimensionalize r
        else:
            p_nondim = 0.0
            q_nondim = 0.0
            r_nondim = 0.0
        # compute Lift and Drag coefficients
        tmp1 = np.exp(-VTOL.M * (self.state.alpha - VTOL.alpha0))
        tmp2 = np.exp(VTOL.M * (self.state.alpha + VTOL.alpha0))
        sigma = (1 + tmp1 + tmp2) / ((1 + tmp1) * (1 + tmp2))
        CL = (1 - sigma) * (VTOL.C_L_0 + VTOL.C_L_alpha * self.state.alpha) \
             + sigma * 2 * np.sign(self.state.alpha) * sa**2 * ca
        CD = (1 - sigma) * (VTOL.C_D_p + ((VTOL.C_L_0 + VTOL.C_L_alpha * self.state.alpha)**2)/(np.pi * VTOL.e * VTOL.AR)) \
            + sigma * 2 * np.sign(self.state.alpha) * sa
        # compute Lift and Drag Forces
        F_lift = qbar * VTOL.S_wing * (
                CL
                + VTOL.C_L_q * q_nondim
                + VTOL.C_L_delta_e * delta.elevator
        )
        F_drag = qbar * VTOL.S_wing * (
                CD
                + VTOL.C_D_q * q_nondim
                + VTOL.C_D_delta_e * delta.elevator
        )
        # compute longitudinal forces in body frame
        fx += -ca * F_drag + sa * F_lift
        fz += -sa * F_drag - ca * F_lift
        # compute lateral forces in body frame
        fy += qbar * VTOL.S_wing * (
                VTOL.C_Y_0
                + VTOL.C_Y_beta * self.state.beta
                + VTOL.C_Y_p * p_nondim
                + VTOL.C_Y_r * r_nondim
                + VTOL.C_Y_delta_a * delta.aileron
                + VTOL.C_Y_delta_r * delta.rudder
        )
        force_elevator = np.array(
            [[qbar * VTOL.S_wing * delta.elevator * (-ca * VTOL.C_D_delta_e + sa * VTOL.C_L_delta_e)],
             [0.0],
             [qbar * VTOL.S_wing * delta.elevator * (-sa * VTOL.C_D_delta_e - ca * VTOL.C_L_delta_e)]])
        # compute logitudinal torque in body frame
        My = qbar * VTOL.S_wing * VTOL.c * (
                VTOL.C_m_0
                + VTOL.C_m_alpha * self.state.alpha
                + VTOL.C_m_q * q_nondim
                + VTOL.C_m_delta_e * delta.elevator
        )
        # compute lateral torques in body frame
        Mx  = qbar * VTOL.S_wing * VTOL.b * (
                VTOL.C_ell_0
                + VTOL.C_ell_beta * self.state.beta
                + VTOL.C_ell_p * p_nondim
                + VTOL.C_ell_r * r_nondim
                + VTOL.C_ell_delta_a * delta.aileron
                + VTOL.C_ell_delta_r * delta.rudder
        )
        Mz = qbar * VTOL.S_wing * VTOL.b * (
                VTOL.C_n_0 + VTOL.C_n_beta * self.state.beta
                + VTOL.C_n_p * p_nondim
                + VTOL.C_n_r * r_nondim
                + VTOL.C_n_delta_a * delta.aileron
                + VTOL.C_n_delta_r * delta.rudder
        )
        Mx_surface = qbar * VTOL.S_wing * VTOL.b * (
                VTOL.C_ell_delta_a * delta.aileron
                + VTOL.C_ell_delta_r * delta.rudder)
        My_surface = qbar * VTOL.S_wing * VTOL.c * (VTOL.C_m_delta_e * delta.elevator)
        Mz_surface = qbar * VTOL.S_wing * VTOL.b * (
                VTOL.C_n_delta_a * delta.aileron
                + VTOL.C_n_delta_r * delta.rudder)
        # force and torque from rear motor
        Va_rear = np.array([[0.0], [0.0], [-1.0]]).T @ self.v_air
        Thrust, Moment = self._motor_thrust_torque(Va_rear, delta.throttle_rear, True)
        Force_rear = Thrust * np.array([[0.0],
                                        [0.0],
                                        [-1.0]])
        Moment_rear = Moment * np.array([[0.0],
                                         [0.0],
                                         [1.0]]) \
                       + np.cross(VTOL.rear_rotor_pos.T, Force_rear.T).T
        fx += Force_rear.item(0)
        fy += Force_rear.item(1)
        fz += Force_rear.item(2)
        Mx += Moment_rear.item(0)
        My += Moment_rear.item(1)
        Mz += Moment_rear.item(2)
        # thrust and torque from the right motor
        Va_right = np.array([[np.cos(self._state.item(13))],
                            [0.0],
                            [-np.sin(self._state.item(13))]]).T @ self.v_air
        Thrust, Moment = self._motor_thrust_torque(Va_right, delta.throttle_right, False)
        Force_right = Thrust * np.array([[np.cos(self._state.item(13))],
                                         [0.0],
                                         [-np.sin(self._state.item(13))]])
        Moment_right = Moment * np.array([[-np.cos(self._state.item(13))],
                                          [0.0],
                                          [np.sin(self._state.item(13))]]) \
                       + np.cross(VTOL.right_rotor_pos.T, Force_right.T).T
        fx += Force_right.item(0)
        fy += Force_right.item(1)
        fz += Force_right.item(2)
        Mx += Moment_right.item(0)
        My += Moment_right.item(1)
        Mz += Moment_right.item(2)
        # thrust and torque from the left motor
        Va_left = np.array([[np.cos(self._state.item(14))],
                            [0.0],
                            [-np.sin(self._state.item(14))]]).T @ self.v_air
        Thrust, Moment = self._motor_thrust_torque(Va_left, delta.throttle_left, False)
        Force_left = Thrust * np.array([[np.cos(self._state.item(14))],
                                        [0.0],
                                        [-np.sin(self._state.item(14))]])
        Moment_left = Moment * np.array([[np.cos(self._state.item(14))],
                                         [0.0],
                                         [-np.sin(self._state.item(14))]]) \
                       + np.cross(VTOL.left_rotor_pos.T, Force_left.T).T
        fx += Force_left.item(0)
        fy += Force_left.item(1)
        fz += Force_left.item(2)
        Mx += Moment_left.item(0)
        My += Moment_left.item(1)
        Mz += Moment_left.item(2)
        self._forces[0] = fx
        self._forces[1] = fy
        self._forces[2] = fz
        return np.array([[fx, fy, fz, Mx, My, Mz]]).T

    def _motor_thrust_torque(self, Va, delta_t, is_rear):
        # compute thrust and torque due to propeller  
        # grab motor/prop params
        if is_rear:
            C_Q0 = VTOL.C_Q0_rear
            C_Q1 = VTOL.C_Q1_rear
            C_T0 = VTOL.C_T0_rear
            C_Q2 = VTOL.C_Q2_rear
            C_T1 = VTOL.C_T1_rear
            C_T2 = VTOL.C_T2_rear
            D_prop = VTOL.D_prop_rear
            KQ = VTOL.KQ_rear
            R_motor = VTOL.R_motor_rear
            i0 = VTOL.i0_rear
        else:
            C_Q0 = VTOL.C_Q0_front
            C_Q1 = VTOL.C_Q1_front
            C_Q2 = VTOL.C_Q2_front
            C_T0 = VTOL.C_T0_front
            C_T1 = VTOL.C_T1_front
            C_T2 = VTOL.C_T2_front
            D_prop = VTOL.D_prop_front
            KQ = VTOL.KQ_front
            R_motor = VTOL.R_motor_front
            i0 = VTOL.i0_front
        # map delta_t throttle command(0 to 1) into motor input voltage
        V_in = VTOL.V_max * delta_t
        # Quadratic formula to solve for motor speed
        a = C_Q0 * VTOL.rho * np.power(D_prop, 5) \
            / ((2.*np.pi)**2)
        b = (C_Q1 * VTOL.rho * np.power(D_prop, 4)
             / (2.*np.pi)) * Va + KQ**2/R_motor
        c = C_Q2 * VTOL.rho * np.power(D_prop, 3) \
            * Va**2 - (KQ / R_motor) * V_in + KQ * i0
        # Consider only positive root
        Omega_op = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)
        # compute advance ratio
        J_op = 2 * np.pi * Va / (Omega_op * D_prop)
        # compute non-dimensionalized coefficients of thrust and torque
        C_T = C_T2 * J_op**2 + C_T1 * J_op + C_T0
        C_Q = C_Q2 * J_op**2 + C_Q1 * J_op + C_Q0
        # add thrust and torque due to propeller
        n = Omega_op / (2 * np.pi)
        thrust_prop = VTOL.rho * n**2 * np.power(D_prop, 4) * C_T
        torque_prop = VTOL.rho * n**2 * np.power(D_prop, 5) * C_Q
        return thrust_prop, torque_prop

    def _update_true_state(self, wind=np.zeros((6,1))):
        # update the state message:
        self.state.pos = self._state[0:3]
        self.state.vel = self._state[3:6]
        self.state.R = quaternion_to_rotation(self._state[6:10])
        self.state.omega = self._state[10:13]
        self.state.gyro_bias = np.array([[VTOL.gyro_bias0.item(0)],
                                         [VTOL.gyro_bias0.item(1)],
                                         [VTOL.gyro_bias0.item(2)]])
        self.motor_angle = self._state[13:15]
        steady_state_wind = wind[0:3]
        gust = wind[3:6]
        # convert wind vector from world to body frame
        wind_body_frame = self.state.R.T @ steady_state_wind  # rotate steady state wind to body frame
        wind_body_frame += gust  # add the gust
        self._wind = self.state.R @ wind_body_frame  # wind in the world frame
        # velocity vector relative to the airmass
        self.v_air = self._state[3:6] - wind_body_frame
        # compute airspeed
        self.state.Va = np.linalg.norm( self.v_air )
        ur = self.v_air.item(0)
        vr = self.v_air.item(1)
        wr = self.v_air.item(2)
        # compute angle of attack
        if ur==0:
            self.state.alpha = np.sign(wr)*np.pi/2.
        else:
            self.state.alpha = np.arctan(wr/ur)
        # compute sideslip angle
        #tmp = np.sqrt(ur**2 + wr**2)
        if self.state.Va==0:
            self.state.beta = np.sign(vr)*np.pi/2.
        else:
            self.state.beta = np.arcsin(vr/self.state.Va)



