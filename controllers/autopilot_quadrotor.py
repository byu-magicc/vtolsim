#implements a quadrotor autopilot controller

import numpy as np
import parameters.control_parameters_quadrotor as AP
from tools.rotations import euler_to_rotation, rotation_to_euler
from tools.wrap import wrap
from controllers.pid_control import PidControl
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta


class Autopilot_Quadrotor:

    def __init__(self, ts_control):
        # instantiate attitude control loops
        self.roll_ctrl = PidControl(
                                kp=AP.roll_kp,
                                ki=0,
                                kd=AP.roll_kd,
                                limit=AP.xy_torque_limit)
        self.pitch_ctrl = PidControl(
                                kp=AP.pitch_kp,
                                ki=0,
                                kd=AP.pitch_kd,
                                limit=AP.xy_torque_limit)
        self.yaw_ctrl = PidControl(
                                kp=AP.yaw_kp,
                                ki=AP.yaw_ki,
                                kd=AP.yaw_kd,
                                Ts=ts_control,
                                limit=AP.z_torque_limit)
        # instantiate lateral controllers
        self.north_ctrl = PidControl(
                                kp=AP.north_kp,
                                ki=AP.north_ki,
                                kd=AP.north_kd,
                                limit=100000)#AP.roll_pitch_angle_limit)
        self.east_ctrl = PidControl(
                                kp=AP.east_kp,
                                ki=AP.east_ki,
                                kd=AP.east_kd,
                                limit=100000)#AP.roll_pitch_angle_limit)
        # instantiate longitudinal controllers
        self.down_ctrl = PidControl(
                                kp=AP.down_kp,
                                ki=AP.down_ki,
                                kd=AP.down_kd,
                                limit=100000)
        self.commanded_state = MsgState()

    #creates the update function
    def update(self, cmd: MsgState, state: MsgState):
        phi = state.phi
        theta = state.theta
        psi = state.psi

        #creates the rotation from the body frame to the inertial frame
        R_body_to_inertial = euler_to_rotation(phi=phi, theta=theta, psi=psi)

        #because we represent the velocities in the Body frame,
        #we need to rotate from body frame to inertial frame for the velocities
        
        #creates the body frame commanded actual velocity vector
        v_body_commanded = np.array([[cmd.u],
                                     [cmd.v],
                                     [cmd.w]])

        #creates the body frame actual velocity vector
        v_body_actual = np.array([[state.u],
                                  [state.v],
                                  [state.w]])
        
        #gets the two v vectors in the inertial frame
        v_inertial_commanded = R_body_to_inertial @ v_body_commanded

        v_inertial_actual = R_body_to_inertial @ v_body_actual

        

        #lateral Autopilot Section
        #north positional controller
        u_n = self.north_ctrl.update_with_ff(y_ref=cmd.north, #north command postion
                                             y=state.north, #north actual position
                                             y_ref_dot=v_inertial_commanded.item(0))