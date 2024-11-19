"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/10/22 - RWB
        7/13/2023 - RWB
        3/5/2024 - RWB
"""
import numpy as np
from scipy.linalg import solve_continuous_are, inv
import parameters.control_parameters_airplane as AP
import models.model_coef as M
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
from tools.transfer_function import TransferFunction
from tools.wrap import wrap

def saturate(input, low_limit, up_limit):
    if input <= low_limit:
        output = low_limit
    elif input >= up_limit:
        output = up_limit
    else:
        output = input
    return output


class Autopilot:
    def __init__(self, ts_control):
        self.Ts = ts_control
        self.yaw_damper = TransferFunction(
                        num=np.array([[AP.yaw_damper_kr, 0]]),
                        den=np.array([[1, AP.yaw_damper_p_wo]]),
                        Ts=ts_control)
        # initialize integrators and delay variables
        self.integratorCourse = 0
        self.integratorAltitude = 0
        self.integratorAirspeed = 0
        self.errorCourseD1 = 0
        self.errorAltitudeD1 = 0
        self.errorAirspeedD1 = 0
        # compute LQR gains
        CrLat = np.array([[0, 0, 0, 0, 1.0]])
        AAlat = np.concatenate((
                    np.concatenate((M.A_lat, np.zeros((5,1))), axis=1),
                    np.concatenate((CrLat, np.zeros((1,1))), axis=1)),
                    axis=0)
        BBlat = np.concatenate((M.B_lat[np.ix_([0,1,2,3,4],[0])], np.zeros((1,1))), axis=0)
        Qlat = np.diag([.001, 1, 1, 1, 1, 0.1]) # v, p, r, phi, chi, intChi
        Rlat = np.diag([1]) # aileron
        Plat = solve_continuous_are(AAlat, BBlat, Qlat, Rlat)
        self.Klat = inv(Rlat) @ BBlat.T @ Plat
        CrLon = np.array([[0, 0, 0, 0, 1.0], [1/AP.Va0, 1/AP.Va0, 0, 0, 0]])
        AAlon = np.concatenate((
                    np.concatenate((M.A_lon, np.zeros((5,2))), axis=1),
                    np.concatenate((CrLon, np.zeros((2,2))), axis=1)),
                    axis=0)
        BBlon = np.concatenate((M.B_lon, np.zeros((2, 2))), axis=0)
        Qlon = np.diag([1, 1, 1, .01, 1, 0.1, 1]) # u, w, q, theta, h, intH, intVa
        Rlon = np.diag([0.1, 1])  # e, t
        Plon = solve_continuous_are(AAlon, BBlon, Qlon, Rlon)
        self.Klon = inv(Rlon) @ BBlon.T @ Plon
        self.commanded_state = MsgState()

    def update(self, cmd, state):
        # lateral autopilot
        errorAirspeed = state.Va - cmd.airspeed_command
        chi_c = wrap(cmd.course_command, state.chi)
        errorCourse = saturate(state.chi - chi_c, -np.radians(35), np.radians(35))
        if (abs(errorCourse)<np.radians(10)):
            self.integratorCourse = self.integratorCourse + (self.Ts/2) * (errorCourse + self.errorCourseD1)
        self.errorCourseD1 = errorCourse
        xLat = np.array([[errorAirspeed * np.sin(state.beta)],  # v
                         [state.p], 
                         [state.r], 
                         [state.phi], 
                         [errorCourse],
                         [self.integratorCourse]])
        tmp = -self.Klat @ xLat
        delta_a = saturate(tmp.item(0), -np.radians(30), np.radians(30))
        delta_r = saturate(self.yaw_damper.update(state.r), 
                           -np.radians(30), np.radians(30))

        # longitudinal autopilot
        altitude_c = saturate(cmd.altitude_command,
                              state.altitude - 0.2*AP.altitude_zone,
                              state.altitude + 0.2*AP.altitude_zone)
        errorAltitude = state.altitude - altitude_c
        if abs(errorAltitude) < 0.2*AP.altitude_zone:
            self.integratorAltitude = self.integratorAltitude \
                                    + (self.Ts/2) * (errorAltitude + self.errorAltitudeD1)
        self.errorAltitudeD1 = errorAltitude
        if abs(errorAirspeed) < 5:
            self.integratorAirspeed = self.integratorAirspeed \
                                    + (self.Ts/2) * (errorAirspeed + self.errorAirspeedD1)
        self.errorAirspeedD1 = errorAirspeed
        xLon = np.array([[errorAirspeed * np.cos(state.alpha)],  # u
                      [errorAirspeed * np.sin(state.alpha)],  # w
                      [state.q],
                      [state.theta],
                      [errorAltitude],
                      [self.integratorAltitude],
                      [self.integratorAirspeed]])
        tmp = -self.Klon @ xLon
        delta_e = saturate(tmp.item(0), -np.radians(30), np.radians(30))
        delta_t = saturate(tmp.item(1), 0.0, 1.0)

        # construct control outputs and commanded states
        delta = MsgDelta(elevator=delta_e,
                         aileron=delta_a,
                         rudder=delta_r,
                         forwardThrottle=delta_t)
        self.commanded_state.altitude = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = 0
        self.commanded_state.theta = 0
        self.commanded_state.chi = cmd.course_command
        return delta, self.commanded_state

