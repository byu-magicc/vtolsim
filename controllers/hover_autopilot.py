"""
- Update History:
    2/6/2019 - R.W. Beard
    11/27/2023 - R.W. Beard
"""
import numpy as np
from controllers.pid_control import PidControl, PdControlWithRate
from tools.rotations import rotation_to_euler

import parameters.convergence_parameters as VTOL
from tools.transfer_function import transferFunction
from tools.wrap import wrap
from message_types.msg_state import msgState
from message_types.msg_controls import msgControls
from hover_controller.compute_delta import compute_delta
import math


class HoverController:
    def __init__(self, ts_control):
        self.gravity = VTOL.gravity  # gravity constant
        rho = VTOL.rho  # density of air
        sigma = 0.05  # low pass filter gain for derivative
        #----------roll gains-------------
        roll_kp = 0.18
        roll_kd = 0.07
        #----------pitch gains-------------
        pitch_kp = 0.15
        pitch_kd = 0.05
        #----------altitude gains-------------
        altitude_kp = -2.5
        altitude_ki = -0.3
        altitude_kd = -1.2
        #-----------position north gains------
        pn_kp = 1.5
        pn_ki = 0.2
        pn_kd = 2.5
        #-----------position east gains------
        pe_kp = 0.55
        pe_ki = 0.01
        pe_kd = 1.3
        #---------heading gains---------------
        yaw_kp = -0.5
        yaw_ki = -0.05
        yaw_kd = -0.5
        self.roll_ctrl = PdControlWithRate(
            kp=roll_kp,
            kd=roll_kd,
            Ts=ts_control,
            limit=np.inf)
        self.pitch_ctrl = PdControlWithRate(
            kp=pitch_kp,
            kd=pitch_kd,
            Ts=ts_control,
            limit=np.inf)
        self.pn_ctrl = PidControl(
            kp=pn_kp,
            ki=pn_ki,
            kd=pn_kd,
            Ts=ts_control,
            limit=np.inf)
        self.pe_ctrl = PidControl(
            kp=pe_kp,
            ki=pe_ki,
            kd=pe_kd,
            Ts=ts_control,
            limit=np.inf)
        self.altitude_ctrl = PidControl(
            kp=altitude_kp,
            ki=altitude_ki,
            kd=altitude_kd,
            Ts=ts_control,
            limit=np.inf)
        self.heading_ctrl = PidControl(
            kp=yaw_kp,
            ki=yaw_ki,
            kd=yaw_kd,
            Ts=ts_control,
            limit=np.inf,
            ispsi=True)
        self.commanded_state = msgState()

    def update(self, command, state):
        roll, pitch, yaw = rotation_to_euler(state.R)
        # outer-loops: inertial position, altitude, heading loops
        un_cmd = self.pn_ctrl.update(command.pn, state.pos[0,0])
        ue_cmd = self.pe_ctrl.update(command.pe, state.pos[1,0])
        roll_cmd = (-un_cmd * np.sin(yaw) + ue_cmd * np.cos(yaw))/self.gravity
        pitch_cmd = (-un_cmd * np.cos(yaw) - ue_cmd * np.sin(yaw))/self.gravity
        roll_cmd = self.saturate(roll_cmd, -30*np.pi/180, 30*np.pi/180);
        pitch_cmd = self.saturate(pitch_cmd, -20*np.pi/180, 20*np.pi/180);
        h_cmd = self.altitude_ctrl.update(command.altitude, -pos[2,0])
        T = VTOL.mass * (VTOL.gravity + h_cmd)
        # inner-loops: roll, pitch, heading loops, trottle command
        tau_x = self.roll_ctrl.update(roll_cmd, roll, state.omega[0,0])
        tau_y = self.pitch_ctrl.update(pitch_cmd, pitch, state.omega[1,0])
        tau_z = self.heading_ctrl.update_with_rate(command.heading, 
                                                   yaw, state.omega[2,0])
        
###HERE
        # delta_command = compute_delta(np.array([[0.0],[-F],[tau_phi],[tau_theta],[tau_psi]]))

        #testing ROSFlight mixer strategy
        delta_command = np.zeros((5,1))
        delta_command[0] = 1.08*F - 1.333*tau_theta
        delta_command[1] = 0.92*F - tau_phi + 0.667*tau_theta
        delta_command[2] = 0.92*F + tau_phi + 0.667*tau_theta
        delta_command[3] = -tau_psi + np.pi/2.0
        delta_command[4] = tau_psi + np.pi/2.0

        for i in range(3):
            if delta_command[i] < 0.0:
                delta_command[i] = 0.0
            elif delta_command[i] > 1.0:
                delta_command[i] = 1.0

        for i in range(3,5):
            if delta_command[i] < np.radians(-15.0):
                delta_command[i] = np.radians(-15.0)
            elif delta_command[i] > np.radians(115.0):
                delta_command[i] = np.radians(115.0)

        # print(delta_command)
        # adfa
        # construct output and commanded states
        delta = msgControls()
        delta.elevon_right = 0.0
        delta.elevon_left = 0.0
        delta.throttle_rear = delta_command.item(0)
        delta.throttle_right = delta_command.item(1)
        delta.throttle_left = delta_command.item(2)
        delta.servo_right = delta_command.item(3)
        delta.servo_left = delta_command.item(4)

        self.commanded_state.h = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = phi_c
        self.commanded_state.theta = theta_c
        self.commanded_state.chi = cmd.yaw_command
        self.commanded_state.pn = cmd.pn_command
        self.commanded_state.pe = cmd.pe_command
        return delta, self.commanded_state

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output
