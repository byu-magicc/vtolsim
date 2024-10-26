"""
autopilot block for mavsim_python - Total Energy Control System
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/14/2020 - RWB
        7/13/2023 - RWB
"""
import numpy as np
import parameters.control_parameters as AP
import parameters.anaconda_parameters as QUAD
from controllers.pi_control import PIControl
from controllers.pd_control_with_rate import PDControlWithRate
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
from tools.transfer_function import TransferFunction
from tools.wrap import wrap


class Autopilot:
    def __init__(self, ts_control):
        # instantiate lateral controllers
        self.roll_from_aileron = PDControlWithRate(
                        kp=AP.roll_kp,
                        kd=AP.roll_kd,
                        limit=np.radians(45))
        self.course_from_roll = PIControl(
                        kp=AP.course_kp,
                        ki=AP.course_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.yaw_damper = TransferFunction(
                        num=np.array([[AP.yaw_damper_kr, 0]]),
                        den=np.array([[1, AP.yaw_damper_p_wo]]),
                        Ts=ts_control)

        # instantiate TECS controllers
        self.pitch_from_elevator = PDControlWithRate(
                        kp=AP.pitch_kp,
                        kd=AP.pitch_kd,
                        limit=np.radians(45))
        # throttle gains (unitless)
        self.E_kp = 1
        self.E_ki = .5
        # pitch gains
        self.L_kp = 1
        self.L_ki = .1
        # saturated altitude error
        self.h_error_max = 50.  # meters
        self.E_integrator = 0.
        self.L_integrator = 0.
        self.E_error_d1 = 0.
        self.L_error_d1 = 0.
        self.delta_t_d1 = 0.
        self.theta_c_d1 = 0.
        self.theta_c_max = np.radians(30)
        self.Ts = ts_control
        self.commanded_state = MsgState()

    def update(self, cmd, state):

        # lateral autopilot
        chi_c = wrap(cmd.course_command, state.chi)
        phi_c = self.saturate(
            cmd.phi_feedforward + self.course_from_roll.update(chi_c, state.chi),
            -np.radians(30), np.radians(30))
        delta_a = self.roll_from_aileron.update(phi_c, state.phi, state.p)
        delta_r = self.yaw_damper.update(state.r)

        # longitudinal TECS autopilot
        # error in kinetic energy
        K_error = 0.5 * QUAD.mass * (cmd.airspeed_command**2 - state.Va**2)
        K_ref = 0.5 * QUAD.mass * cmd.airspeed_command**2

        # (saturated) error in potential energy
        U_error = QUAD.mass * QUAD.gravity * \
                  self.saturate(cmd.altitude_command - state.altitude,
                                -self.h_error_max, self.h_error_max)

        # (normalized) error in total energy and energy difference
        E_error = (K_error + U_error) / K_ref
        L_error = (U_error - K_error) / K_ref

        #  update the integrator(with anti - windup)
        if (self.delta_t_d1 > 0.) and (self.delta_t_d1 < 1.):
            self.E_integrator = self.E_integrator \
                                + (self.Ts / 2) * (E_error + self.E_error_d1)

        if (self.theta_c_d1 > -self.theta_c_max) and (self.theta_c_d1 < self.theta_c_max):
            self.L_integrator = self.L_integrator \
                                + (self.Ts / 2) * (L_error + self.L_error_d1)

        delta_t = self.saturate(self.E_kp * E_error
                                + self.E_ki * self.E_integrator, 0, 1)
        theta_c = self.saturate(self.L_kp * L_error
                                + self.L_ki * self.L_integrator,
                                -self.theta_c_max, self.theta_c_max)
        delta_e = self.pitch_from_elevator.update(theta_c, state.theta, state.q)
        self.E_error_d1 = E_error
        self.L_error_d1 = L_error
        self.delta_t_d1 = delta_t
        self.theta_c_d1 = theta_c

        # construct output and commanded states
        delta = MsgDelta(elevator=delta_e,
                         aileron=delta_a,
                         rudder=delta_r,
                         forwardThrottle=delta_t)
        self.commanded_state.altitude = cmd.altitude_command                 
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = phi_c
        self.commanded_state.theta = theta_c
        self.commanded_state.chi = cmd.course_command
        return delta, self.commanded_state

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output
