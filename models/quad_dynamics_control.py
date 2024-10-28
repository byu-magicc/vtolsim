"""
mavDynamics 
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state
    
mavsim_python
    - Beard & McLain, PUP, 2012
    - Update history:  
        2/24/2020 - RWB
        7/13/2023 - RWB
"""
import numpy as np
from models.quad_dynamics import QuadDynamics
# load message types
#from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
import parameters.anaconda_parameters as QUAD
from tools.rotations import quaternion_to_rotation, quaternion_to_euler


class QuadDynamicsControl(QuadDynamics):
    def __init__(self, Ts: float):
        super().__init__(Ts)
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

        #creates an airspeed vector and initializes it to the groundspeed portion
        self.v_air = self._state[3:6]


    ###################################
    # public functions
    def update(self, delta: MsgDelta, wind: np.ndarray):
        '''
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        '''
        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)
        super()._rk4_step(forces_moments)
        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)
        # update the message class for the true state
        self._update_true_state()

    ###################################
    # private functions
    def _update_velocity_data(self, wind: np.ndarray=np.zeros((6,1))):
        '''
        update the airspeed, angle-of-attack, and side-slip angles using wind data
        '''
        steady_state = wind[0:3]
        gust = wind[3:6]
        # convert wind vector from body to world frame
        R = quaternion_to_rotation(self._state[6:10]) # passive rotation from body to world frame
        wind_body_frame = R.T @ steady_state  # rotate steady state wind to body frame
        wind_body_frame += gust  # add the gust
        self._wind = R @ wind_body_frame  # wind in the world frame
        # velocity vector relative to the airmass
        self.v_air = self._state[3:6] - wind_body_frame
        ur = self.v_air.item(0)
        vr = self.v_air.item(1)
        wr = self.v_air.item(2)
        # compute airspeed
        self._Va = np.sqrt(ur**2 + vr**2 + wr**2)
        # compute angle of attack
        if ur == 0:
            self._alpha = np.sign(wr)*np.pi/2.
        else:
            self._alpha = np.arctan(wr/ur)
        # compute sideslip angle
        tmp = np.sqrt(ur**2 + wr**2)
        if tmp == 0:
            self._beta = np.sign(vr)*np.pi/2.
        else:
            self._beta = np.arcsin(vr/tmp)

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
        # compute logitudinal torque in body frame
        My = qbar * QUAD.S_wing * QUAD.c * (
                QUAD.C_m_0
                + QUAD.C_m_alpha * self._alpha
                + QUAD.C_m_q * q_nondim
                + QUAD.C_m_delta_e * delta.elevator
        )
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


        #creates the array of the throttles
        throttles = np.array([[delta.forwardThrottle],
                              [delta.verticalThrottle_1],
                              [delta.verticalThrottle_2],
                              [delta.verticalThrottle_3],
                              [delta.verticalThrottle_4]])


        #iterates through the five propellers and obtains their respective contribution to the dynamics
        for i in range(QUAD.num_rotors):

            #gets the airspeed through the current propeller
            Va_Prop = ((QUAD.normalVectors)[i]).T @ self.v_air

            #gets the rotor thrusts and moment
            thrust_rotor, moment_rotor = self._motor_thrust_torque(Va_Prop, throttles[i][0])

            #gets the force, the thrust times the unit vector in the out direction
            Force_rotor = thrust_rotor*((QUAD.normalVectors)[i])

            #obtains the aerodynamic moment
            aero_moment = ((QUAD.propDirections)[i])*moment_rotor*((QUAD.rotorPositions)[i])

            #obtains the lever moment
            lever_moment = thrust_rotor*((QUAD.leverArms)[i])

            #gets the total moment
            total_moment = lever_moment + aero_moment

            #adds the forces and moments
            fx += Force_rotor.item(0)
            fy += Force_rotor.item(1)
            fz += Force_rotor.item(2)
            #does the same for the moments
            Mx += total_moment.item(0)
            My += total_moment.item(1)
            Mz += total_moment.item(2)



        self._forces[0] = fx
        self._forces[1] = fy
        self._forces[2] = fz
        return np.array([[fx, fy, fz, Mx, My, Mz]]).T

    def _motor_thrust_torque(self, Va: float, delta_t: float) -> tuple[float, float]:
        '''
        compute thrust and torque due to propeller
        '''
        # map delta_t throttle command(0 to 1) into motor input voltage
        v_in = QUAD.V_max * delta_t
        # Quadratic formula to solve for motor speed
        a = QUAD.C_Q0 * QUAD.rho * np.power(QUAD.D_prop, 5) \
            / ((2.*np.pi)**2)
        b = (QUAD.C_Q1 * QUAD.rho * np.power(QUAD.D_prop, 4)
             / (2.*np.pi)) * Va + QUAD.KQ * QUAD.KV / QUAD.R_motor
        c = QUAD.C_Q2 * QUAD.rho * np.power(QUAD.D_prop, 3) \
            * Va**2 - (QUAD.KQ / QUAD.R_motor) * v_in + QUAD.KQ * QUAD.i0
       
        # Angular speed of propeller
        omega_p = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)
        # compute advance ratio
        J_p = 2 * np.pi * Va / (omega_p * QUAD.D_prop)
        # compute non-dimensionalized coefficients of thrust and torque
        C_T = QUAD.C_T2 * J_p**2 + QUAD.C_T1 * J_p + QUAD.C_T0
        C_Q = QUAD.C_Q2 * J_p**2 + QUAD.C_Q1 * J_p + QUAD.C_Q0
        # compute propeller thrust and torque
        n = omega_p / (2 * np.pi)
        thrust_prop = QUAD.rho * n**2 * np.power(QUAD.D_prop, 4) * C_T
        torque_prop = QUAD.rho * n**2 * np.power(QUAD.D_prop, 5) * C_Q

        return thrust_prop, torque_prop

    def _update_true_state(self):
        '''
        update the class structure for the true state:
           [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        '''
        phi, theta, psi = quaternion_to_euler(self._state[6:10])
        pdot = quaternion_to_rotation(self._state[6:10]) @ self._state[3:6]
        #updates the north east and altitude positions
        self.true_state.north = self._state.item(0)
        self.true_state.east = self._state.item(1)
        self.true_state.altitude = -self._state.item(2)
        #updates the body frame velocity
        self.true_state.u = self._state.item(3)
        self.true_state.v = self._state.item(4)
        self.true_state.w = self._state.item(5)

        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta
        self.true_state.phi = phi
        self.true_state.theta = theta
        self.true_state.psi = psi
        self.true_state.Vg = np.linalg.norm(pdot)
        self.true_state.gamma = np.arcsin(pdot.item(2) / self.true_state.Vg)
        self.true_state.chi = np.arctan2(pdot.item(1), pdot.item(0))
        self.true_state.p = self._state.item(10)
        self.true_state.q = self._state.item(11)
        self.true_state.r = self._state.item(12)
        self.true_state.wn = self._wind.item(0)
        self.true_state.we = self._wind.item(1)
        self.true_state.bx = 0
        self.true_state.by = 0
        self.true_state.bz = 0
        self.true_state.gimbal_az = 0
        self.true_state.gimbal_el = 0
