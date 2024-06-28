
#quad dynamics
#-this file implements the dynamic equations of motion for the quad plane
#uses unit quaternion for the attitude state


import numpy as np
import parameters.quad.anaconda_parameters as QUAD
import parameters.sensor_parameters as SENSOR
from tools.rotations import quaternion_to_rotation, quaternion_to_euler, euler_to_rotation
from tools.quaternions import *
from message_types.msg_convert import *
from message_types.quad.msg_state import MsgState
from message_types.msg_sensors import MsgSensors
from message_types.quad.msg_delta import MsgDelta


#creates the QuadDynamics class
class QuadDynamics:

    #creates the initialization function
    def __init__(self, ts: float):
        self._ts = ts


        #creates the state array and initializes them to the original positions
        self._state = np.array([
            [QUAD.pn0],  # [0]  north position
            [QUAD.pe0],  # [1]  east position
            [QUAD.pd0],  # [2]  down position
            [QUAD.u0],   # [3]  velocity along body x-axis
            [QUAD.v0],   # [4]  velocity along body y-axis
            [QUAD.w0],   # [5]  velocity along body z-axis
            [QUAD.e0],   # [6]  quaternion - scalar part
            [QUAD.e1],   # [7]  quaternion - vector-x
            [QUAD.e2],   # [8]  quaternion - vector-y
            [QUAD.e3],   # [9]  quaternion - vector-z
            [QUAD.p0],   # [10]  roll rate
            [QUAD.q0],   # [11]  pitch rate
            [QUAD.r0],   # [12]  yaw rate
        ])

        #stores the wind
        self._wind = np.array([[0.0], [0.0], [0.0]])
        #stores the original airspeed
        self.v_air = self._state[3:6]
        self._Va = 0.0
        self._alpha = 0.0
        self._beta = 0.0
        self._update_velocity_data()

        #stores the forces
        self._forces = np.array([[0.0], [0.0], [0.0]])
        #initializes the true state message
        self.true_state = MsgState()
        #initializes the sensors message
        self._sensors = MsgSensors()
        # random walk parameters for GPS
        self._gps_eta_n = 0.
        self._gps_eta_e = 0.
        self._gps_eta_h = 0.
        # timer so that gps only updates every ts_gps seconds
        self._t_gps = 999.  # large value ensures gps updates at initial time.


    #creates the update function for the system
    def update(self, delta: MsgDelta, wind: np.ndarray):

        #calls to get the forces and the moments of the system
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


    #creates the sensors function
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
        self._sensors.accel_x = self._forces.item(0)/QUAD.mass \
            + QUAD.gravity*np.sin(theta) \
            + np.random.normal(0., SENSOR.accel_sigma)
        self._sensors.accel_y = self._forces.item(1)/QUAD.mass \
            - QUAD.gravity*np.cos(theta)*np.sin(phi) \
            + np.random.normal(0., SENSOR.accel_sigma)
        self._sensors.accel_z = self._forces.item(2)/QUAD.mass \
            - QUAD.gravity*np.cos(theta)*np.cos(phi) \
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
            -QUAD.rho*QUAD.gravity*self._state.item(2) \
            + np.random.normal(0., SENSOR.static_pres_sigma)
        self._sensors.diff_pressure = 0.5 * QUAD.rho * self._Va**2 \
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
        u_dot = r*v - q*w + fx/QUAD.mass
        v_dot = p*w - r*u + fy/QUAD.mass
        w_dot = q*u - p*v + fz/QUAD.mass
        # rotational kinematics
        e0_dot = 0.5 * (-p*e1 - q*e2 - r*e3)
        e1_dot = 0.5 * (p*e0 + r*e2 - q*e3)
        e2_dot = 0.5 * (q*e0 - r*e1 + p*e3)
        e3_dot = 0.5 * (r*e0 + q*e1 -p*e2)
        # rotatonal dynamics
        p_dot = QUAD.gamma1*p*q - QUAD.gamma2*q*r + QUAD.gamma3*Mx + QUAD.gamma4*Mz
        q_dot = QUAD.gamma5*p*r - QUAD.gamma6*(p**2-r**2) + My/QUAD.Jy
        r_dot = QUAD.gamma7*p*q - QUAD.gamma1*q*r + QUAD.gamma4*Mx + QUAD.gamma8*Mz
        # collect the derivative of the states
        x_dot = np.array([[pn_dot, pe_dot, pd_dot, 
                           u_dot, v_dot, w_dot,
                           e0_dot, e1_dot, e2_dot, e3_dot,
                           p_dot, q_dot, r_dot]]).T
        return x_dot


    #creates the update velocity data function
    def _update_velocity_data(self, wind: np.ndarray=np.zeros((6,1))):

        #gets the steady state wind
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


    #creates the forces moments function
    def _forces_moments(self, delta: MsgDelta)->np.ndarray:


        #gets the three control surfaces
        elevator = delta.elevator
        aileron = delta.aileron
        rudder = delta.rudder

        #gets the R matrix
        R = quaternion_to_rotation(self._state[6:10])

        #gets p, q, and r
        p = self._state.item(10)
        q = self._state.item(11)
        r = self._state.item(12)

        #gets the gravitational force
        f_g = QUAD.mass * QUAD.gravity * R.T @ np.array([[0.], [0.], [1.]])

        #gets each portion of the gravitational force
        fx = f_g.item(0)
        fy = f_g.item(1)
        fz = f_g.item(2)
        #intermediate variables
        qbar = 0.5 * QUAD.rho * self._Va**2
        ca = np.cos(self._alpha)
        sa = np.sin(self._alpha)

        if self._Va > 1:
            p_nondim = p * QUAD.b / (2 * self._Va)  # nondimensionalize p
            q_nondim = q * QUAD.c / (2 * self._Va)  # nondimensionalize q
            r_nondim = r * QUAD.b / (2 * self._Va)  # nondimensionalize r
        else:
            p_nondim = 0.0
            q_nondim = 0.0
            r_nondim = 0.0

        # compute Lift and Drag coefficients
        tmp1 = np.exp(-QUAD.M * (self._alpha - QUAD.alpha0))
        tmp2 = np.exp(QUAD.M * (self._alpha + QUAD.alpha0))
        sigma = (1 + tmp1 + tmp2) / ((1 + tmp1) * (1 + tmp2))

        CL = (1 - sigma) * (QUAD.C_L_0 + QUAD.C_L_alpha * self._alpha) \
             + sigma * 2 * np.sign(self._alpha) * sa**2 * ca
        CD = (1 - sigma) \
            * (QUAD.C_D_p + \
               ((QUAD.C_L_0 + QUAD.C_L_alpha * self._alpha)**2)\
                /(np.pi * QUAD.e * QUAD.AR)) \
            + sigma * 2 * np.sign(self._alpha) * sa

        # compute Lift and Drag Forces
        F_lift = qbar * QUAD.S_wing * (CL + QUAD.C_L_q * q_nondim + QUAD.C_L_delta_e * elevator)
        F_drag = qbar * QUAD.S_wing * (CD + QUAD.C_D_q * q_nondim + QUAD.C_D_delta_e * elevator)

        # compute longitudinal forces in body frame
        fx += - ca * F_drag + sa * F_lift
        fz += - sa * F_drag - ca * F_lift
        # compute lateral forces in body frame
        fy += qbar * QUAD.S_wing * (
                  QUAD.C_Y_0
                + QUAD.C_Y_beta * self._beta
                + QUAD.C_Y_p * p_nondim
                + QUAD.C_Y_r * r_nondim
                + QUAD.C_Y_delta_a * aileron
                + QUAD.C_Y_delta_r * rudder)
        
        # compute logitudinal torque in body frame
        My = qbar * QUAD.S_wing * QUAD.c * (
                QUAD.C_m_0
                + QUAD.C_m_alpha * self._alpha
                + QUAD.C_m_q * q_nondim
                + QUAD.C_m_delta_e * elevator
        )
        # compute lateral torques in body frame
        Mx  = qbar * QUAD.S_wing * QUAD.b * (
                QUAD.C_ell_0
                + QUAD.C_ell_beta * self._beta
                + QUAD.C_ell_p * p_nondim
                + QUAD.C_ell_r * r_nondim
                + QUAD.C_ell_delta_a * aileron
                + QUAD.C_ell_delta_r * rudder
        )
        Mz = qbar * QUAD.S_wing * QUAD.b * (
                QUAD.C_n_0 + QUAD.C_n_beta * self._beta
                + QUAD.C_n_p * p_nondim
                + QUAD.C_n_r * r_nondim
                + QUAD.C_n_delta_a * aileron
                + QUAD.C_n_delta_r * rudder
        )
        

        #############################################################################################################
        #Forces and moments from the forward prop

        #gets the airspeed through the rear, main propulsion propellor
        Va_forward_prop = np.array([[1.0], [0.0], [0.0]]).T @ self.v_air

        #gets the thrust and the moment from the forward prop
        Thrust_Forward, Prop_Moment_Forward = self._motor_thrust_torque(Va_forward_prop, delta.forwardThrottle)

        #gets the force on the airplane from the forward propeller
        #In this case, it is in the positive x direction, so we multiply by the unit vector in the direction the thrust is facing
        Force_Forward = Thrust_Forward * np.array([[1.0], [0.0], [0.0]])

        #gets the moment, which is a little more complicated
        ##############################*******************************************************************************************
        #I may need to change this to account for the direction of the prop rotation
        #I am currently choosing it to rotate clockwise, when at the back, looking to the front of the airplane
        Moment_Forward = Prop_Moment_Forward*np.array([[1.0],[0.0],[0.0]]) + np.cross(QUAD.forward_rotor_pos.T, Force_Forward.T).T

        #adds each component to the whole forces and moment variables
        fx += Force_Forward.item(0)
        fy += Force_Forward.item(1)
        fz += Force_Forward.item(2)

        Mx += Moment_Forward.item(0)
        My += Moment_Forward.item(1)
        Mz += Moment_Forward.item(2)
        #############################################################################################################



        #############################################################################################################
        #Forces and moments from the rear port propeller
        Va_rear_port_prop = np.array([[0.0],[0.0],[-1.0]]).T @ self.v_air

        #gets the thrust and the moment from the rear port propeller
        Thrust_RearPort, Prop_Moment_RearPort = self._motor_thrust_torque(Va_rear_port_prop, delta.verticalThrottle_1)

        #gets the RearPort Force
        Force_RearPort = Thrust_RearPort*np.array([[0.0],[0.0],[-1.0]])

        #gets the rear port moment
        Moment_RearPort = Prop_Moment_RearPort*np.array([[0.0],[0.0],[-1.0]]) + np.cross(QUAD.vertical_rotor_1_pos.T, Force_RearPort.T).T

        #adds each component to the whole forces and moment variables
        fx += Force_RearPort.item(0)
        fy += Force_RearPort.item(1)
        fz += Force_RearPort.item(2)

        Mx += Moment_RearPort.item(0)
        My += Moment_RearPort.item(1)
        Mz += Moment_RearPort.item(2)
        #############################################################################################################

        #############################################################################################################
        #Forces and moments from the front port propeller
        Va_front_port_prop = np.array([[0.0],[0.0],[-1.0]]).T @ self.v_air

        #gets the thrust and the moment from the rear port propeller
        Thrust_FrontPort, Prop_Moment_FrontPort = self._motor_thrust_torque(Va_front_port_prop, delta.verticalThrottle_2)

        #gets the RearPort Force
        Force_FrontPort = Thrust_FrontPort*np.array([[0.0],[0.0],[-1.0]])

        #gets the rear port moment
        Moment_FrontPort = Prop_Moment_FrontPort*np.array([[0.0],[0.0],[-1.0]]) + np.cross(QUAD.vertical_rotor_2_pos.T, Force_FrontPort.T).T

        #adds each component to the whole forces and moment variables
        fx += Force_FrontPort.item(0)
        fy += Force_FrontPort.item(1)
        fz += Force_FrontPort.item(2)

        Mx += Moment_FrontPort.item(0)
        My += Moment_FrontPort.item(1)
        Mz += Moment_FrontPort.item(2)

        #############################################################################################################

        #############################################################################################################
        #Forces and moments from the front starboard propeller
        Va_front_starboard_prop = np.array([[0.0],[0.0],[-1.0]]).T @ self.v_air

        #gets the thrust and the moment from the rear port propeller
        Thrust_FrontStarboard, Prop_Moment_FrontStarboard = self._motor_thrust_torque(Va_front_starboard_prop, delta.verticalThrottle_3)

        #gets the RearStarboard Force
        Force_FrontStarboard = Thrust_FrontStarboard*np.array([[0.0],[0.0],[-1.0]])

        #gets the rear starboard moment
        Moment_FrontStarboard = Prop_Moment_FrontStarboard*np.array([[0.0],[0.0],[-1.0]]) + np.cross(QUAD.vertical_rotor_3_pos.T, Force_FrontStarboard.T).T

        #adds each component to the whole forces and moment variables
        fx += Force_FrontStarboard.item(0)
        fy += Force_FrontStarboard.item(1)
        fz += Force_FrontStarboard.item(2)

        Mx += Moment_FrontStarboard.item(0)
        My += Moment_FrontStarboard.item(1)
        Mz += Moment_FrontStarboard.item(2)

        #############################################################################################################

        #############################################################################################################
        #Forces and moments from the rear starboard propeller

        #Forces and moments from the front starboard propeller
        Va_rear_starboard_prop = np.array([[0.0],[0.0],[-1.0]]).T @ self.v_air

        #gets the thrust and the moment from the rear port propeller
        Thrust_RearStarboard, Prop_Moment_RearStarboard = self._motor_thrust_torque(Va_rear_starboard_prop, delta.verticalThrottle_4)

        #gets the RearStarboard Force
        Force_RearStarboard = Thrust_RearStarboard*np.array([[0.0],[0.0],[-1.0]])

        #gets the rear starboard moment
        Moment_RearStarboard = Prop_Moment_RearStarboard*np.array([[0.0],[0.0],[-1.0]]) + np.cross(QUAD.vertical_rotor_4_pos.T, Force_RearStarboard.T).T

        #adds each component to the whole forces and moment variables
        fx += Force_RearStarboard.item(0)
        fy += Force_RearStarboard.item(1)
        fz += Force_RearStarboard.item(2)

        Mx += Moment_RearStarboard.item(0)
        My += Moment_RearStarboard.item(1)
        Mz += Moment_RearStarboard.item(2)

        #############################################################################################################

        #returns the forces
        return np.array([[fx, fy, fz, Mx, My, Mz]]).T


    
    def _motor_thrust_torque(self, Va: float, delta_t: float)->tuple[float, float]:


        #gets the coefficients for the propeller

        C_Q0 = QUAD.C_Q0
        C_Q1 = QUAD.C_Q1
        C_T0 = QUAD.C_T0
        C_Q2 = QUAD.C_Q2
        C_T1 = QUAD.C_T1
        C_T2 = QUAD.C_T2
        D_prop = QUAD.D_prop
        KQ = QUAD.KQ
        R_motor = QUAD.R_motor
        i0 = QUAD.i0


        #gets the voltage in, based on the delta_t
        V_in = QUAD.V_max * delta_t
        # Quadratic formula to solve for motor speed
        a = C_Q0 * QUAD.rho * np.power(D_prop, 5) \
            / ((2.*np.pi)**2)
        b = (C_Q1 * QUAD.rho * np.power(D_prop, 4)
             / (2.*np.pi)) * Va + KQ**2/R_motor
        c = C_Q2 * QUAD.rho * np.power(D_prop, 3) \
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
        T_p = QUAD.rho * n**2 * np.power(D_prop, 4) * C_T
        Q_p = QUAD.rho * n**2 * np.power(D_prop, 5) * C_Q
        return T_p, Q_p


    #function to update the true state
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
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta    
