#creates the geometric controller for the airplane

import numpy as np
from message_types.msg_trajectory import MsgTrajectory
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
import parameters.quad.anaconda_parameters as QUAD
from tools.differentiators import RotationDerivative
from tools.lie_group import vee_SO3, antisymmetric

from scipy.linalg import expm
from tools.rotations import quaternion_to_euler, quaternion_to_rotation, vee, hat
import parameters.quad.geometric_control_parameters as GEOM
from controllers.quad.geometric_control.optimal_pitch import compute_thrust, find_pitch_thrust, find_thrust_from_theta
from controllers.quad.low_level_control import LowLevelControl_successiveControl



#creates the Geometric Controller class
class GeometricController:

    def __init__(self,
                 time_step=0.01,
                 seed_optimizer=True,
                 constrain_pitch_rate=True,
                 theta_0=0.0):
        
        self.differentiate_R = RotationDerivative()
        self.seed_optimizer = seed_optimizer
        self.constrain_pitch_rate = constrain_pitch_rate
        self.T_opt_prev = None
        self.theta_opt_prev = None
        self.theta_cmd_prev = theta_0
        #initialize low-level controllers
        self.low_ctrl = LowLevelControl_successiveControl(
            M=0.5, 
            Va0=0.0, 
            ts=time_step)
        
    #creates the update function
    def update(self,
               trajectory: MsgTrajectory,
               state: MsgState)->tuple[MsgDelta, MsgState]:
        # compute desired force and rotation matrix to follow trajectory
        F_d, R_d2i = self.trajectory_follower(trajectory, state)
        # numerically differentiate the rotation matrix R_d to get desired angular velocity
        omega_d = self.differentiate_R.update(R_d2i)
        # find Va and gamma
        vd_d = R_d2i.T @ trajectory.velocity
        gamma = np.arctan2(-vd_d[2], vd_d[0])
        Va = np.linalg.norm(vd_d)
        # find optimal thrust and pitch
        theta_opt_p2d, T_opt_d_p = find_pitch_thrust(
            vd_d, 
            F_d, 
            previous_theta=self.theta_opt_prev, 
            model=GEOM.aero_type, 
            method=GEOM.optimal_pitch_method)
        # filter theta
        if (self.constrain_pitch_rate 
            and np.abs(self.theta_cmd_prev - theta_opt_p2d)>GEOM.delta_theta_max):
                theta_p2d = self.theta_cmd_prev \
                    + np.sign(theta_opt_p2d - self.theta_cmd_prev)*GEOM.delta_theta_max
                # make sure thrust matches pitch angle
                T_d_p = find_thrust_from_theta(vd_d, 
                                               F_d, 
                                               theta_p2d, 
                                               model=GEOM.aero_type)
        else:
            T_d_p = T_opt_d_p
            theta_p2d = theta_opt_p2d
        self.T_opt_prev = T_opt_d_p
        self.theta_opt_prev = theta_opt_p2d
        self.theta_cmd_prev = theta_p2d
        # y_d2i = R_d2i[:,1]
        # R_p2d = expm(-hat(theta_p2d*y_d2i)).T
        R_p2d = expm(-hat(theta_p2d*np.array([0., 1., 0.]))).T
        R_p2i = R_d2i @ R_p2d
        omega_p2i_p = R_p2d.T @ omega_d
        # omega_p2i_p = omega_d
        B = np.array([[1., 0.], [0., 0.], [0., 1.]])
        q_b2i = state[6:10].reshape(-1)
        R_b2i = quaternion_to_rotation(q_b2i)
        T_d_in_b = B.T @ R_b2i.T @ R_d2i @ R_p2d @ B @ T_d_p
        # attitude control
        omega_c = self.attitude_controller(state, R_p2i, omega_p2i_p)
        delta = self.low_ctrl.update(T_d_in_b, omega_c, state)
        commanded_state = MsgState()
        commanded_state.pos = trajectory.position
        commanded_state.vel = R_p2i.T @ trajectory.velocity
        commanded_state.R = R_p2i
        commanded_state.omega = omega_c
        commanded_state.motor_angle = np.array([
            [delta.motor_left],[delta.motor_right]])
        return delta, commanded_state


    def trajectory_follower(self,
                            trajectory: MsgTrajectory, 
                            state: MsgState, )->tuple[np.ndarray, np.ndarray]:
        '''
        Compute desired force and rotation matrix to follow trajectory.  
        Implements WillisBeard21.pdf equations 6-11
        '''
        pos_error = state.pos - trajectory.position
        vel_error = state.R @ state.vel - trajectory.velocity
        e3 = np.array([[0.], [0.], [1.]])
        f_d = QUAD.mass * ( 
                trajectory.acceleration 
                - QUAD.gravity*e3 
                - GEOM.Kp @ pos_error 
                - GEOM.Kd @ vel_error 
                )
        # desired body x axis in inertial frame
        x_d = np.array([
            [np.cos(trajectory.heading)],
            [np.sin(trajectory.heading)],
            [0.0]])
        # desired body y axis in inertial frame
        y_d = np.cross(x_d.T, f_d.T).T
        norm_y_d = np.linalg.norm(y_d)
        if norm_y_d>0:
            y_d = y_d / norm_y_d
        else:
            y_d = np.cross(x_d.T, e3.T).T
        # desired body z axis in inertial frame
        z_d = np.cross(x_d.T, y_d.T).T
        # desired rotation matrix from body to inertial
        R_d = np.column_stack((x_d, y_d, z_d))
        # desired force in body x-z plane
        F_d = np.array([
            [x_d.T @ f_d],
            [z_d.T @ f_d],
            ])
        return F_d, R_d
    

    def attitude_controller(
        self,
        state: MsgState, 
        R_d: np.ndarray, 
        omega_d: np.ndarray,
        )->np.ndarray:
        '''
        Compute the commanded angular velocity to converge to a desired rotation matrix
        Implements WillisBeard21.pdf equation 27
        '''
        R_d2b = state.R.T @ R_d
        if 0.5*(np.trace(np.eye(3) - R_d2b)) >= 2:
            print("Attitude controller assumptions violated")
            print("R_d2b = ", R_d2b)
        else:
            # compute commanded angular velocity
            omega_c = R_d2b @ omega_d + GEOM.omega_Kp @ vee_SO3(antisymmetric(R_d2b))
        return omega_c