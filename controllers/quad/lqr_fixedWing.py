#this file implements the fixed wing lqr controller for the quadplane,
#that is, an lqr controller for when we are just in fixed wing mode, which is
#identical to the mavsim Lqr controller.

#uses rate damping controller

import numpy as np
from scipy.linalg import solve_continuous_are, inv
import parameters.quad.control_parameters as AP
import models.quad.model_coef as M
from message_types.quad.msg_state import MsgState
from message_types.quad.msg_delta import MsgDelta
from tools.transfer_function import TransferFunction
from tools.wrap import wrap
from message_types.quad.msg_autopilot_fixedWing import MsgAutopilotFixedWing
from message_types.quad.msg_state import MsgState

from tools.rotations import euler_to_rotation

#imports the dirty derivative
from tools.differentiators import DirtyDerivative

def saturate(input, low_limit, up_limit):
    if input <= low_limit:
        output = low_limit
    elif input >= up_limit:
        output = up_limit
    else:
        output = input
    return output


#instantiates the fixed wing Autopilot
class Autopilot:
    #creates the initialization function
    def __init__(self, ts_control: float):
        self.Ts = ts_control
        self.yaw_damper = TransferFunction(num=np.array([[AP.yaw_damper_kr, 0]]),
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
        # compute LQR gains
        CrLat = np.array([[0, 0, 0, 0, 1.0]])
        AAlat = np.concatenate((
                    np.concatenate((M.A_lat, np.zeros((5,1))), axis=1),
                    np.concatenate((CrLat, np.zeros((1,1))), axis=1)),
                    axis=0)
        BBlat = np.concatenate((M.B_lat[np.ix_([0,1,2,3,4],[0])], np.zeros((1,1))), axis=0)
        Qlat = np.diag([0.001, 1, 1, 1, 1, 0.1]) # v, p, r, phi, chi, intChi
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

        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        self.counter = 0
        #creates the dirty derivatives for the altitude, airspeed, and course errors
        self.airspeedErrorDiff = DirtyDerivative(Ts=ts_control)

    #creates the update function
    def update(self, cmd: MsgAutopilotFixedWing, state: MsgState):

        #gets the airspeed error
        errorAirspeed = state.Va - cmd.airspeed_command
        #gets the chi command
        chi_c = wrap(cmd.course_command, state.chi)
        #gets the course error
        errorCourse = saturate(state.chi - chi_c, -np.radians(35), np.radians(35))
        #integrator antiwindup
        if (abs(errorCourse)<np.radians(10)):
            self.integratorCourse = self.integratorCourse + (self.Ts/2) * (errorCourse + self.errorCourseD1)

        #saves the course error delayed by one
        self.errorCourseD1 = errorCourse

        #gets the roll, pitch, and yaw
        phi, theta, psi = state.euler_angles()

        #gets the p, q, and r
        p = (state.omega)[0][0]
        q = (state.omega)[1][0]
        r = (state.omega)[2][0]

        #gets the xLat, that is the lateral state
        xLat = np.array([[errorAirspeed*np.sin(state.beta)],
                         [p],
                         [r],
                         [phi],
                         [errorCourse],
                         [self.integratorCourse]])
        
        temp = -self.Klat @ xLat
        #saturates the delta_a command
        delta_a = saturate(temp.item(0), -np.radians(30), np.radians(30))
        #saturates the delta_r command
        delta_r = saturate(self.yaw_damper.update(r), -np.radians(30), np.radians(30))

        #gets the altitude
        altitude = -(state.pos)[2][0]
        #longitudinal autopilot
        altitude_c = saturate(cmd.altitude_command, 
                              altitude - 0.2*AP.altitude_zone,
                              altitude + 0.2*AP.altitude_zone)
        #gets the altitude error
        errorAltitude = altitude - altitude_c

        #antiwindup integrator
        if abs(errorAltitude) < 0.2*AP.altitude_zone:
            self.integratorAltitude += (self.Ts/2)*(errorAltitude + self.errorAltitudeD1)
        #saves the error altitude delayed by one
        self.errorAltitudeD1 = errorAltitude


        #gets the derivative of the airspeed
        #antiwindup scheme for the airspeed integrator
        #only integrates when the derivative is lower than a threshold
        if abs(errorAirspeed < 5.0):
            self.integratorAirspeed += (self.Ts/2)*(errorAirspeed + self.errorAirspeedD1)
        #saves the error airspeed for delayed function
        self.errorAirspeedD1 = errorAirspeed



        xLon = np.array([[errorAirspeed * np.cos(state.alpha)],  # u
                      [errorAirspeed * np.sin(state.alpha)],  # w
                      [q],
                      [theta],
                      [errorAltitude],
                      [self.integratorAltitude],
                      [self.integratorAirspeed]])
        tmp = -self.Klon @ xLon
        delta_e = saturate(tmp.item(0), -np.radians(30), np.radians(30))
        delta_t = saturate(tmp.item(1), 0.0, 1.0)

        #upates p from the mav state
        self.p = p

        # construct control outputs and commanded states
        delta = MsgDelta(elevator=delta_e,
                         aileron=delta_a,
                         rudder=delta_r,
                         forwardThrottle=delta_t)
        self.commanded_state.pos[2][0] = -cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.chi = cmd.course_command

        if self.counter % 10 == 0:
            a = 0

        self.counter += 1
        return delta, self.commanded_state


    #gets the current errors for the autopilot
    def getErrors(self):
        #returns the course error, altitudeError, and airspeed error
        return self.errorCourseD1, self.p
