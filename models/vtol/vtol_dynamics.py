"""
vtolDynamics
    - this file implements the dynamic equations of motion for VTOL
    - use unit quaternion for the attitude state

    - Update history:
        5/8/2019 - R.W. Beard
        3/12/2024 - RWBeard
"""
import numpy as np
import parameters.vtol.convergence_parameters as VTOL
import parameters.vtol.sensor_parameters as SENSOR
from tools.rotations import quaternion_to_rotation, quaternion_to_euler, euler_to_rotation
from tools.quaternions import *
from message_types.vtol.msg_convert import *
from message_types.vtol.msg_state import MsgState
from message_types.vtol.msg_sensors import MsgSensors
from message_types.vtol.msg_delta import MsgDelta

class VtolDynamics:
    '''
        Vtol dynamics - implements the dynamic model for VTOL
    '''
    def __init__(self, ts: float):
        self._ts = ts
        # set initial states based on parameter file
        # _state is the 15x1 internal state of the aircraft that is being propagated:
        self._state = np.array([
            [VTOL.pn0],  # [0]  north position
            [VTOL.pe0],  # [1]  east position
            [VTOL.pd0],  # [2]  down position
            [VTOL.u0],   # [3]  velocity along body x-axis
            [VTOL.v0],   # [4]  velocity along body y-axis
            [VTOL.w0],   # [5]  velocity along body z-axis
            [VTOL.e0],   # [6]  quaternion - scalar part
            [VTOL.e1],   # [7]  quaternion - vector-x
            [VTOL.e2],   # [8]  quaternion - vector-y
            [VTOL.e3],   # [9]  quaternion - vector-z
            [VTOL.p0],   # [10]  roll rate
            [VTOL.q0],   # [11]  pitch rate
            [VTOL.r0],   # [12]  yaw rate
            [VTOL.right_rotor],  # [13] pitch angle of right motor
            [VTOL.left_rotor],   # [14] pitch angle of left motor
            ])
        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec
        self.v_air = self._state[3:6]
        self._Va = 0
        self._alpha = 0
        self._beta = 0
        self._update_velocity_data()
        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        # initialize true_state message
        self.true_state = MsgState()
        # initialize the sensors message
        self._sensors = MsgSensors()
        # random walk parameters for GPS
        self._gps_eta_n = 0.
        self._gps_eta_e = 0.
        self._gps_eta_h = 0.
        # timer so that gps only updates every ts_gps seconds
        self._t_gps = 999.  # large value ensures gps updates at initial time.

    ###################################
    # public functions
    def update(self, delta: MsgDelta, wind: np.ndarray):
        '''
            Integrate the differential equations defining dynamics, update sensors
            
            delta: control inputs
            wind = [steady_state'; gust']'
                steady_state: (3x1) constant wind in inertial coordinates
                gust: (3x1) gusts in body frame
        '''
        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)
        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts
        k1 = self._derivatives(self._state, forces_moments, delta)
        k2 = self._derivatives(self._state + time_step/2.*k1, forces_moments, delta)
        k3 = self._derivatives(self._state + time_step/2.*k2, forces_moments, delta)
        k4 = self._derivatives(self._state + time_step*k3, forces_moments, delta)
        self._state += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)
        # normalize the quaternion
        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[6] = self._state.item(6)/normE
        self._state[7] = self._state.item(7)/normE
        self._state[8] = self._state.item(8)/normE
        self._state[9] = self._state.item(9)/normE
        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)
        # update the message class for the true state
        self._update_true_state()

    def sensors(self)->MsgSensors:
        '''
            Return value of sensors on MAV: 
                gyros, 
                accels, 
                static_pressure, 
                dynamic_pressure, 
                GPS
        '''
        phi, theta, psi = quaternion_to_euler(self._state[6:10])
        pdot = quaternion_to_rotation(self._state[6:10]) @ self._state[3:6]
        # simulate rate gyros(units are rad / sec)
        self._sensors.gyro_x = self._state.item(10) \
            + np.random.normal(SENSOR.gyro_x_bias, SENSOR.gyro_sigma)
        self._sensors.gyro_y = self._state.item(11) \
            + np.random.normal(SENSOR.gyro_y_bias, SENSOR.gyro_sigma)
        self._sensors.gyro_z = self._state.item(12) \
            + np.random.normal(SENSOR.gyro_z_bias, SENSOR.gyro_sigma)
        # simulate accelerometers(units of g)
        self._sensors.accel_x = self._forces.item(0)/VTOL.mass \
            + VTOL.gravity*np.sin(theta) \
            + np.random.normal(0., SENSOR.accel_sigma)
        self._sensors.accel_y = self._forces.item(1)/VTOL.mass \
            - VTOL.gravity*np.cos(theta)*np.sin(phi) \
            + np.random.normal(0., SENSOR.accel_sigma)
        self._sensors.accel_z = self._forces.item(2)/VTOL.mass \
            - VTOL.gravity*np.cos(theta)*np.cos(phi) \
            + np.random.normal(0., SENSOR.accel_sigma)
        # simulate magnetometers
        # magnetic field in provo has magnetic declination of 12.5 degrees
        # and magnetic inclination of 66 degrees
        R_mag = euler_to_rotation(0.0, np.radians(-66), np.radians(12.5))
        # magnetic field in inertial frame: unit vector
        mag_inertial = R_mag.T @ np.array([[1.0], [0.0], [0.0]])
        R = quaternion_to_rotation(self._state[6:10]) # body to inertial
        # magnetic field in body frame: unit vector
        mag_body = R.T @ mag_inertial
        self._sensors.mag_x = mag_body.item(0) \
            + np.random.normal(0., SENSOR.mag_sigma)
        self._sensors.mag_y = mag_body.item(1) \
            + np.random.normal(0., SENSOR.mag_sigma)
        self._sensors.mag_z = mag_body.item(2) \
            + np.random.normal(0., SENSOR.mag_sigma)
        # simulate pressure sensors
        self._sensors.static_pressure = \
            -VTOL.rho*VTOL.gravity*self._state.item(2) \
            + np.random.normal(0., SENSOR.static_pres_sigma)
        self._sensors.diff_pressure = 0.5 * VTOL.rho * self._Va**2 \
            + np.random.normal(0., SENSOR.diff_pres_sigma)
        # simulate GPS sensor
        if self._t_gps >= SENSOR.ts_gps:
            self._gps_eta_n = \
                np.exp(-SENSOR.gps_beta*SENSOR.ts_gps)*self._gps_eta_n \
                + np.random.normal(0., SENSOR.gps_n_sigma)
            self._gps_eta_e = \
                np.exp(-SENSOR.gps_beta*SENSOR.ts_gps)*self._gps_eta_e \
                + np.random.normal(0., SENSOR.gps_e_sigma)
            self._gps_eta_h = \
                np.exp(-SENSOR.gps_beta*SENSOR.ts_gps)*self._gps_eta_h \
                + np.random.normal(0., SENSOR.gps_h_sigma)
            self._sensors.gps_n = self._state.item(0) + self._gps_eta_n
            self._sensors.gps_e = self._state.item(1) + self._gps_eta_e
            self._sensors.gps_h = -self._state.item(2) + self._gps_eta_h
            self._sensors.gps_Vg = np.linalg.norm(self._state[3:6]) \
                + np.random.normal(0., SENSOR.gps_Vg_sigma)
            self._sensors.gps_course = np.arctan2(pdot.item(1), pdot.item(0)) \
                + np.random.normal(0., SENSOR.gps_course_sigma)
            self._t_gps = 0.
        else:
            self._t_gps += self._ts
        return self._sensors

    def external_set_state(self, new_state: np.ndarray):
        '''
        Allow external function to set the internal state
        '''
        self._state = new_state
        self._update_true_state()

    ###################################
    # private functions
    def _derivatives(self, 
                     state: np.ndarray, 
                     forces_moments: np.ndarray, 
                     delta: MsgDelta):
        '''
            Implements equations of motion xdot = f(x, u)
        '''
        # extract the states
        # pn = state.item(0)
        # pe = state.item(1)
        # pd = state.item(2)
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
        right_motor = state.item(13)
        left_motor = state.item(14)
        #   extract forces/moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)
        Mx = forces_moments.item(3)
        My = forces_moments.item(4)
        Mz = forces_moments.item(5)
        # position kinematics
        pos_dot = quaternion_to_rotation(state[6:10]) @ state[3:6]
        pn_dot = pos_dot.item(0)
        pe_dot = pos_dot.item(1)
        pd_dot = pos_dot.item(2)
        # position dynamics
        u_dot = r*v - q*w + fx/VTOL.mass
        v_dot = p*w - r*u + fy/VTOL.mass
        w_dot = q*u - p*v + fz/VTOL.mass
        # rotational kinematics
        e0_dot = 0.5 * (-p*e1 - q*e2 - r*e3)
        e1_dot = 0.5 * (p*e0 + r*e2 - q*e3)
        e2_dot = 0.5 * (q*e0 - r*e1 + p*e3)
        e3_dot = 0.5 * (r*e0 + q*e1 -p*e2)
        # rotatonal dynamics
        p_dot = VTOL.gamma1*p*q - VTOL.gamma2*q*r + VTOL.gamma3*Mx + VTOL.gamma4*Mz
        q_dot = VTOL.gamma5*p*r - VTOL.gamma6*(p**2-r**2) + My/VTOL.Jy
        r_dot = VTOL.gamma7*p*q - VTOL.gamma1*q*r + VTOL.gamma4*Mx + VTOL.gamma8*Mz
        # motor servo dynamics
        right_motor_dot = VTOL.k_servo*(delta.motor_right - right_motor)
        left_motor_dot = VTOL.k_servo*(delta.motor_left - left_motor)
        # collect the derivative of the states
        x_dot = np.array([[pn_dot, pe_dot, pd_dot, 
                           u_dot, v_dot, w_dot,
                           e0_dot, e1_dot, e2_dot, e3_dot,
                           p_dot, q_dot, r_dot, 
                           right_motor_dot, left_motor_dot]]).T
        return x_dot

    def _update_velocity_data(self, wind: np.ndarray=np.zeros((6,1))):
        '''
        Updates the airspeed, angle-of-attack, and sideslip angles
        Also updates the incident wind on the body used to calculate thrust
        '''
        steady_state = wind[0:3]
        gust = wind[3:6]
        # convert wind vector from world to body frame
        R = quaternion_to_rotation(self._state[6:10]) # rotation from body to world frame
        wind_body_frame = R.T @ steady_state  # rotate steady state wind to body frame
        wind_body_frame += gust  # add the gust
        self._wind = R @ wind_body_frame  # wind in the world frame
        # velocity vector relative to the airmass
        self.v_air = self._state[3:6] - wind_body_frame
        # compute airspeed
        self._Va = np.linalg.norm( self.v_air )
        ur = self.v_air.item(0)
        vr = self.v_air.item(1)
        wr = self.v_air.item(2)
        # compute angle of attack
        if ur==0:
            self._alpha = np.sign(wr)*np.pi/2.
        else:
            self._alpha = np.arctan(wr/ur)
        # compute sideslip angle
        #tmp = np.sqrt(ur**2 + wr**2)
        if self._Va==0:
            self._beta = np.sign(vr)*np.pi/2.
        else:
            self._beta = np.arcsin(vr/self._Va)

    def _forces_moments(self, delta: MsgDelta)->np.ndarray:
        '''
            computes the external forces and torques experienced by eVTOL
            returns: np.array([[Fx, Fy, Fz, Mx, My, Mz]]).T
        '''
        elevator = delta.elevator
        aileron = delta.aileron
        rudder = delta.rudder
        R = quaternion_to_rotation(self._state[6:10])
        p = self._state.item(10)
        q = self._state.item(11)
        r = self._state.item(12)
        # compute gravitational forces
        f_g = VTOL.mass * VTOL.gravity * R.T @ np.array([[0.], [0.], [1.]])
        fx = f_g.item(0)
        fy = f_g.item(1)
        fz = f_g.item(2)
        # intermediate variables
        qbar = 0.5 * VTOL.rho * self._Va**2
        ca = np.cos(self._alpha)
        sa = np.sin(self._alpha)
        if self._Va > 1:
            p_nondim = p * VTOL.b / (2 * self._Va)  # nondimensionalize p
            q_nondim = q * VTOL.c / (2 * self._Va)  # nondimensionalize q
            r_nondim = r * VTOL.b / (2 * self._Va)  # nondimensionalize r
        else:
            p_nondim = 0.0
            q_nondim = 0.0
            r_nondim = 0.0
        # compute Lift and Drag coefficients
        tmp1 = np.exp(-VTOL.M * (self._alpha - VTOL.alpha0))
        tmp2 = np.exp(VTOL.M * (self._alpha + VTOL.alpha0))
        sigma = (1 + tmp1 + tmp2) / ((1 + tmp1) * (1 + tmp2))
        CL = (1 - sigma) * (VTOL.C_L_0 + VTOL.C_L_alpha * self._alpha) \
             + sigma * 2 * np.sign(self._alpha) * sa**2 * ca
        CD = (1 - sigma) \
            * (VTOL.C_D_p + \
               ((VTOL.C_L_0 + VTOL.C_L_alpha * self._alpha)**2)\
                /(np.pi * VTOL.e * VTOL.AR)) \
            + sigma * 2 * np.sign(self._alpha) * sa
        # compute Lift and Drag Forces
        F_lift = qbar * VTOL.S_wing * (
                CL
                + VTOL.C_L_q * q_nondim
                + VTOL.C_L_delta_e * elevator
        )
        F_drag = qbar * VTOL.S_wing * (
                CD
                + VTOL.C_D_q * q_nondim
                + VTOL.C_D_delta_e * elevator
        )
        # compute longitudinal forces in body frame
        fx += - ca * F_drag + sa * F_lift
        fz += - sa * F_drag - ca * F_lift
        # compute lateral forces in body frame
        fy += qbar * VTOL.S_wing * (
                VTOL.C_Y_0
                + VTOL.C_Y_beta * self._beta
                + VTOL.C_Y_p * p_nondim
                + VTOL.C_Y_r * r_nondim
                + VTOL.C_Y_delta_a * aileron
                + VTOL.C_Y_delta_r * rudder
        )
        # I don't think these lines are needed
        # force_elevator = np.array(
        #     [[qbar * VTOL.S_wing * elevator * (-ca * VTOL.C_D_delta_e + sa * VTOL.C_L_delta_e)],
        #      [0.0],
        #      [qbar * VTOL.S_wing * elevator * (-sa * VTOL.C_D_delta_e - ca * VTOL.C_L_delta_e)]])
        # compute logitudinal torque in body frame
        My = qbar * VTOL.S_wing * VTOL.c * (
                VTOL.C_m_0
                + VTOL.C_m_alpha * self._alpha
                + VTOL.C_m_q * q_nondim
                + VTOL.C_m_delta_e * elevator
        )
        # compute lateral torques in body frame
        Mx  = qbar * VTOL.S_wing * VTOL.b * (
                VTOL.C_ell_0
                + VTOL.C_ell_beta * self._beta
                + VTOL.C_ell_p * p_nondim
                + VTOL.C_ell_r * r_nondim
                + VTOL.C_ell_delta_a * aileron
                + VTOL.C_ell_delta_r * rudder
        )
        Mz = qbar * VTOL.S_wing * VTOL.b * (
                VTOL.C_n_0 + VTOL.C_n_beta * self._beta
                + VTOL.C_n_p * p_nondim
                + VTOL.C_n_r * r_nondim
                + VTOL.C_n_delta_a * aileron
                + VTOL.C_n_delta_r * rudder
        )
        # I don't think these are needed either
        # Mx_surface = qbar * VTOL.S_wing * VTOL.b * (
        #         VTOL.C_ell_delta_a * aileron
        #         + VTOL.C_ell_delta_r * rudder)
        # My_surface = qbar * VTOL.S_wing * VTOL.c * (VTOL.C_m_delta_e * elevator)
        # Mz_surface = qbar * VTOL.S_wing * VTOL.b * (
        #         VTOL.C_n_delta_a * aileron
        #         + VTOL.C_n_delta_r * rudder)
        #
        # force and torque from rear motor
        Va_rear = np.array([[0.0], [0.0], [-1.0]]).T @ self.v_air
        Thrust, Moment = self._motor_thrust_torque(Va_rear, 
                                                   delta.throttle_rear, 
                                                   True)
        Force_rear = Thrust * np.array([[0.0], [0.0], [-1.0]])
        Moment_rear = Moment * np.array([[0.0], [0.0], [1.0]]) \
            + np.cross(VTOL.rear_rotor_pos.T, Force_rear.T).T
        fx += Force_rear.item(0)
        fy += Force_rear.item(1)
        fz += Force_rear.item(2)
        Mx += Moment_rear.item(0)
        My += Moment_rear.item(1)
        Mz += Moment_rear.item(2)
        # thrust and torque from the right motor
        ur = np.array([
            [np.cos(self._state.item(13))],
            [0.0],
            [-np.sin(self._state.item(13))]])
        Va_right = ur.T @ self.v_air
        Thrust, Moment = self._motor_thrust_torque(Va_right, 
                                                   delta.throttle_right, 
                                                   False)
        Force_right = Thrust * ur
        Moment_right = -Moment * ur \
            + np.cross(VTOL.right_rotor_pos.T, Force_right.T).T

        fx += Force_right.item(0)
        fy += Force_right.item(1)
        fz += Force_right.item(2)
        Mx += Moment_right.item(0)
        My += Moment_right.item(1)
        Mz += Moment_right.item(2)
        # thrust and torque from the left motor
        ul = np.array([
            [np.cos(self._state.item(14))],
            [0.0],
            [-np.sin(self._state.item(14))]])
        Va_left = ul.T @ self.v_air
        Thrust, Moment = self._motor_thrust_torque(Va_left, 
                                                   delta.throttle_left, 
                                                   False)
        Force_left = Thrust * ul
        Moment_left = -Moment * ul + np.cross(VTOL.left_rotor_pos.T, Force_left.T).T
        fx += Force_left.item(0)
        fy += Force_left.item(1)
        fz += Force_left.item(2)
        Mx += Moment_left.item(0)
        My += Moment_left.item(1)
        Mz += Moment_left.item(2)
        self._forces[0] = fx
        self._forces[1] = fy
        self._forces[2] = fz
        # why is this needed
        # self.total_thrust = (Force_right + Force_left + Force_rear + force_elevator).reshape(-1)
        # self.total_torque = np.array([Mx_rotor + Mx_surface, My_rotor + My_surface, Mz_rotor + Mz_surface])
        return np.array([[fx, fy, fz, Mx, My, Mz]]).T

    def _motor_thrust_torque(self, 
                             Va: float, 
                             delta_t: float, 
                             is_rear: bool)->tuple[float, float]:
        '''
            compute thrust and torque due to propeller
            eVTOL will experience negative torque
            :returns: [Thrust, Torque]
        '''
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
        T_p = VTOL.rho * n**2 * np.power(D_prop, 4) * C_T
        Q_p = VTOL.rho * n**2 * np.power(D_prop, 5) * C_Q
        return T_p, Q_p

    def _update_true_state(self):
        '''update the class structure for the true state '''
        self.true_state.pos = self._state[0:3]
        self.true_state.vel = self._state[3:6]
        self.true_state.R = quaternion_to_rotation(self._state[6:10])
        self.true_state.omega = self._state[10:13]
        self.true_state.gyro_bias = np.array([
            [SENSOR.gyro_x_bias],
            [SENSOR.gyro_y_bias],
            [SENSOR.gyro_z_bias]])  
        self.true_state.motor_angle = self._state[13:15]
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta
        # old state format
        # phi, theta, psi = quaternion_to_euler(self._state[6:10])
        # pdot = quaternion_to_rotation(self._state[6:10]) @ self._state[3:6]
        # self.true_state.pn = self._state.item(0)
        # self.true_state.pe = self._state.item(1)
        # self.true_state.h = -self._state.item(2)
        # self.true_state.Va = self._Va
        # self.true_state.alpha = self._alpha
        # self.true_state.beta = self._beta
        # self.true_state.phi = phi
        # self.true_state.theta = theta
        # self.true_state.psi = psi
        # self.true_state.Vg = np.linalg.norm(pdot)
        # if(self.true_state.Vg > .01):
        #     self.true_state.gamma = np.arcsin(pdot.item(2) / self.true_state.Vg)
        # else:
        #     self.true_state.gamma = 0.0
        # self.true_state.chi = np.arctan2(pdot.item(1), pdot.item(0))
        # self.true_state.p = self._state.item(10)
        # self.true_state.q = self._state.item(11)
        # self.true_state.r = self._state.item(12)
        # self.true_state.wn = self._wind.item(0)
        # self.true_state.we = self._wind.item(1)
        # self.true_state.bx = SENSOR.gyro_x_bias
        # self.true_state.by = SENSOR.gyro_y_bias
        # self.true_state.bz = SENSOR.gyro_z_bias
        # self.true_state.u = self._state.item(3)
        # self.true_state.v = self._state.item(4)
        # self.true_state.w = self._state.item(5)
        # self.true_state.right_rotor = self._state.item(13)
        # self.true_state.left_rotor = self._state.item(14)