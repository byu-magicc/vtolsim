"""
quadrotor_dynamics
    - this file implements the dynamic equations of motion
    - use unit quaternion for the attitude state

    - Update history:
        3/19/2020 - RWB
        4/19/2022 - RWB
        7/13/2023 - RWB
"""
import numpy as np
import parameters.quadrotor_parameters as QUAD
from tools.rotations import quaternion_to_rotation
from tools.rotations import hat, quat_hat

# load message types
from message_types.msg_state import MsgState

class QuadrotorDynamics:
    def __init__(self, Ts):
        self.ts_simulation = Ts
        # internal state (quaternion)
        self._state = np.array([
            [QUAD.pos0.item(0)],
            [QUAD.pos0.item(1)],
            [QUAD.pos0.item(2)],
            [QUAD.vel0.item(0)],
            [QUAD.vel0.item(1)],
            [QUAD.vel0.item(2)],
            [QUAD.quat0.item(0)],
            [QUAD.quat0.item(1)],
            [QUAD.quat0.item(2)],
            [QUAD.quat0.item(3)],
            [QUAD.omega0.item(0)],
            [QUAD.omega0.item(1)],
            [QUAD.omega0.item(2)],
        ])
        # visible state (rotation)
        self.state = MsgState()
        self.update_true_state()
        self.state.gyro_bias = np.array([[QUAD.gyro_bias0.item(0)],
                                         [QUAD.gyro_bias0.item(1)],
                                         [QUAD.gyro_bias0.item(2)]])
        self._forces = np.array([[0.], [0.], [0.]])

    ###################################
    # public functions
    def update(self, delta):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self.ts_simulation
        x1 = self.f(self._state, delta)
        x2 = self.f(self._state + time_step/2.*x1, delta)
        x3 = self.f(self._state + time_step/2.*x2, delta)
        x4 = self.f(self._state + time_step*x3, delta)
        self._state = self._state + time_step/6 * (x1 + 2*x2 + 2*x3 + x4)
        # normalize the quaternion
        norm_quat = np.linalg.norm(self._state[6:10])
        self._state[6][0] /= norm_quat
        self._state[7][0] /= norm_quat
        self._state[8][0] /= norm_quat
        self._state[9][0] /= norm_quat
        # update the message class for the true state
        self.update_true_state()

    def f(self, state, delta):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        pos = state[0:3]
        vel = state[3:6]
        quat = state[6:10]
        omega = state[10:13]
        # R = quaternion_to_rotation(quat)  # body to inertial
        # computer forces and torques
        forces, torques = self.forces_torques(state, delta)

        # equations of motion
        pos_dot = vel
        vel_dot = forces / QUAD.mass
        # rotational equations of motion
        quat_dot = 0.5 * quat_hat(omega) @ quat
        omega_dot = QUAD.Jinv @ (-hat(omega) @ QUAD.J @ omega + torques)
        # collect the derivative of the states
        x_dot = np.concatenate((pos_dot, vel_dot, quat_dot, omega_dot), axis=0)
        return x_dot

    def forces_torques(self, state, delta):
        """
        return the forces in the inertial frame, and
         moments in the body frame
        """
        vel = state[3:6]
        quat = state[6:10]
        R = quaternion_to_rotation(quat)  # body to inertial
        e3 = np.array([[0.], [0.], [1.]])
        D = QUAD.gravity * QUAD.Cd_prop * np.diag([1, 1, 0])

        # compute forces on quadrotor
        gravity_force = QUAD.mass * QUAD.gravity * e3
        drag_force = -QUAD.mass * R @ D @ R.T @ vel
        thrust_force = - delta.thrust * R @ e3

        # total forces in inertial frame
        self._forces = gravity_force + drag_force + thrust_force
        # return forces and moments
        return self._forces, delta.torque

    def update_true_state(self):
        # update the state message:
        self.state.pos = self._state[0:3]
        self.state.vel = self._state[3:6]
        self.state.R = quaternion_to_rotation(self._state[6:10])
        self.state.omega = self._state[10:13]

    def set_state(self, new_state):
        self._state = new_state

