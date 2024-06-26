
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
from message_types.msg_delta import MsgDelta


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
        self._update_velocity_data(wind=self._wind)



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
