import numpy as np
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
import parameters.anaconda_parameters as QUAD
from tools.rotations import quaternion_to_rotation, quaternion_to_euler
import parameters.sensor_parameters as SENSOR


#defines the dynamics for the quadplane
class QuadDynamics:
    #creates the init function
    def __init__(self, Ts: float):
        self.ts_simulation = Ts


        #creates the state vector
        self._state = np.array([[QUAD.pn0],
                                [QUAD.pe0],
                                [QUAD.pd0],
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
        self._Va = 0.0
        self._alpha = 0.0
        self._beta = 0.0
        self._Vg = 0.0
        self._chi = 0.0

        # update velocity data and forces and moments
        self._update_velocity_data()
        self._forces_moments(delta=MsgDelta())
        # update the message class for the true state
        self._update_true_state()

        #creates the variable to store the air velocity
        self.v_air = self._state[3:6]

        #saves the forces and the moments
        self.forces_moments = np.zeros((6, 1))


    #creates the update function
    def update(self, delta: MsgDelta, wind: np.ndarray):
        #gets the forces and moments
        #calls to get the forces and the moments of the system
        forces_moments = self._forces_moments(delta)
        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self.ts_simulation
        k1 = self._derivatives(self._state, forces_moments)
        k2 = self._derivatives(self._state + time_step/2.*k1, forces_moments)
        k3 = self._derivatives(self._state + time_step/2.*k2, forces_moments)
        k4 = self._derivatives(self._state + time_step*k3, forces_moments)
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

        return forces_moments




    #creates function to obtain the forces and moments for the system
    def _forces_moments(self, delta: MsgDelta) -> np.ndarray: 
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """

        #creates the forces and moments variables
        fx = 0
        fy = 0
        fz = 0
        Mx = 0
        My = 0
        Mz = 0

        #phi, theta, psi = quaternion_to_euler(self._state[6:10])
        p = self._state.item(10)
        q = self._state.item(11)
        r = self._state.item(12)

        # compute gravitational forces
        R = quaternion_to_rotation(self._state[6:10]) # rotation from body to world frame
        f_g = R.T @ np.array([[0.], [0.], [QUAD.mass * QUAD.gravity]])
        #adds the gravitational forces to the fx, fy, and fz
        fx += f_g.item(0)
        fy += f_g.item(1)
        fz += f_g.item(2)


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
        # compute longitudinal forces in body frame, 
        # adds the aerodynamic forces to the fx and fz components
        fx += fx - ca * F_drag + sa * F_lift
        fz += fz - sa * F_drag - ca * F_lift
        #adds to the fy component
        # compute lateral forces in body frame
        fy += qbar * QUAD.S_wing * (
                QUAD.C_Y_0
                + QUAD.C_Y_beta * self._beta
                + QUAD.C_Y_p * p_nondim
                + QUAD.C_Y_r * r_nondim
                + QUAD.C_Y_delta_a * delta.aileron
                + QUAD.C_Y_delta_r * delta.rudder
        )

        #adds the Aerodynamic Moments
        # compute logitudinal torque in body frame
        My += qbar * QUAD.S_wing * QUAD.c * (
                QUAD.C_m_0
                + QUAD.C_m_alpha * self._alpha
                + QUAD.C_m_q * q_nondim
                + QUAD.C_m_delta_e * delta.elevator
        )

        # compute lateral torques in body frame
        Mx += qbar * QUAD.S_wing * QUAD.b * (
                QUAD.C_ell_0
                + QUAD.C_ell_beta * self._beta
                + QUAD.C_ell_p * p_nondim
                + QUAD.C_ell_r * r_nondim
                + QUAD.C_ell_delta_a * delta.aileron
                + QUAD.C_ell_delta_r * delta.rudder
        )
        Mz += qbar * QUAD.S_wing * QUAD.b * (
                QUAD.C_n_0 + QUAD.C_n_beta * self._beta
                + QUAD.C_n_p * p_nondim
                + QUAD.C_n_r * r_nondim
                + QUAD.C_n_delta_a * delta.aileron
                + QUAD.C_n_delta_r * delta.rudder
        )

        #creates the vector containing the 5 throttles
        throttles = np.array([[delta.forwardThrottle],
                              [delta.verticalThrottle_1],
                              [delta.verticalThrottle_2],
                              [delta.verticalThrottle_3],
                              [delta.verticalThrottle_4]])

        #iterates through the five propellers, (Forward Prop and 4 vertical props)
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

        #saves the three forces
        self._forces[0] = fx
        self._forces[1] = fy
        self._forces[2] = fz

        #returns the whole wrench
        return np.array([[fx, fy, fz, Mx, My, Mz]]).T
    
    #creates the function that obtains the thrust and torque from the motor
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



    #creates the rk4 function
    def _rk4_step(self, forces_moments: np.ndarray):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self.ts_simulation
        k1 = self._derivatives(self._state[0:13], forces_moments)
        k2 = self._derivatives(self._state[0:13] + time_step/2.*k1, forces_moments)
        k3 = self._derivatives(self._state[0:13] + time_step/2.*k2, forces_moments)
        k4 = self._derivatives(self._state[0:13] + time_step*k3, forces_moments)
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


    #creates the function to obtain the state derivatives
    def _derivatives(self, state_vector: np.ndarray, forces_moments: np.ndarray)->np.ndarray:
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

    #defines the function to update the velocity data
    def _update_velocity_data(self, wind: np.ndarray=np.zeros((6,1))):

        #gets the steady state wind
        steady_state = wind[0:3]
        gust = wind[3:6]
        # convert wind vector from world to body frame
        R = quaternion_to_rotation(self._state[6:10]) # rotation from body to world frame
        wind_body_frame = R.T @ steady_state  # rotate steady state wind to body frame
        wind_body_frame += gust  # add the gust
        self._wind = R @ wind_body_frame  # wind in the world frame
        #gets the velocity in the body frame
        velocity_body_frame = self._state[3:6]
        # velocity vector relative to the airmass
        self.v_air = velocity_body_frame - wind_body_frame
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

        #computes the groundspeed
        self._Vg = np.linalg.norm(velocity_body_frame)
        
        #computes the course angle
        #gets the world frame velocity
        velocity_world_frame = R @ velocity_body_frame

        self._chi = np.arctan2(velocity_world_frame.item(1), velocity_world_frame.item(0))


    #function to update the true state
    def _update_true_state(self):
        #updates the actual position
        self.true_state.pos = self._state[0:3]
        #updates the velocity of the object
        self.true_state.vel = self._state[3:6]
        #updates the rotation matrix
        self.true_state.R = quaternion_to_rotation(self._state[6:10])
        #updates the p,q,r values
        self.true_state.omega = self._state[10:13]
        #updates the gyro biases
        self.true_state.gyro_bias = np.array([
            [SENSOR.gyro_x_bias],
            [SENSOR.gyro_y_bias],
            [SENSOR.gyro_z_bias]])
        #saves the airspeed magnitude
        self.true_state.Va = self._Va
        #saves the alpha
        self.true_state.alpha = self._alpha
        #saves the beta
        self.true_state.beta = self._beta
        self.true_state.Vg = self._Vg
        self.true_state.chi = self._chi

    #creates helper function to get the state vector
    def getStateVector(self):
        return self._state