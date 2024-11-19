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