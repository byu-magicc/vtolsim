"""
mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/21/2019 - RWB
        2/24/2020 - RWB
        7/13/2023 - RWB
        3/20/2024 - RWB
"""
import numpy as np
from math import sin, cos
from message_types.msg_state import MsgState
from message_types.msg_path import MsgPath
from message_types.msg_autopilot_fixedWing import MsgAutopilot
from tools.wrap import wrap
from tools.saturate import saturate


class PathFollower:
    """
    Path follower

    Attributes
    ----------
        chi_inf : float
            approach angle for large distance from straight-line path
        k_path : float
            path gain for straight-line path following
        k_orbit : float
            path gain for orbit following
        gravity : float
        autopilot_commands : MsgAutopilot
            message sent to autopilot
    
    Methods
    -------
    def update(path, state) :
        update the path follower
        Parameters
        ----------
            path: MsgPath
                defines the path
            state : MsgState
                state of aircraft
        Returns
        -------
        commands : MsgAutopilot 
            commands to the autopilot
    _follow_straight_line(path, state) :
        called when path.mode=='line'
        follow a straight line
    _follow_orbit(path, state) :
        called when path.mode=='orbit'
        follow a constant altitude orbit
    _follow_helix(path, state) :
        called when path.mode=='helix'
        follow a climbing helix
    """  

    def __init__(self):
        self.chi_inf = np.radians(50)  # approach angle for large distance from straight-line path
        self.k_path = 0.05  # path gain for straight-line path following
        self.k_orbit = 5 #10.0  # path gain for orbit following
        self.gravity = 9.8
        self._helix_spiral_index = 0
        self._helix_varphi = 0
        self.k_helix_phi = 1
        self.k_helix_grad = 60*np.diag([1,1,1])
        self.k_helix_cross = 15
        self.autopilot_commands = MsgAutopilot()  # message sent to autopilot

    def update(self, 
               path: MsgPath, 
               state: MsgState)->MsgAutopilot:
        if path.type == 'line':
            self._follow_straight_line(path, state)
        elif path.type == 'orbit':
            self._follow_orbit(path, state)
        elif path.type == 'helix':
            self._follow_helix(path, state)
        return self.autopilot_commands

    def _follow_straight_line(self, 
                              path: MsgPath, 
                              state: MsgState):
        self.autopilot_commands.mode = 'airspeed_altitude_course'
        self.autopilot_commands.airspeed_command = path.airspeed
        chi_q = np.arctan2(path.line_direction.item(1),
                           path.line_direction.item(0))
        chi_q = wrap(chi_q, state.chi)
        ep = np.array([[state.north], [state.east], [-state.altitude]]) - path.line_origin
        # path_error = -sin(chi_q) * (state.north - path.line_origin.item(0)) \
        #              + cos(chi_q) * (state.east - path.line_origin.item(1))
        path_error = -sin(chi_q) * ep.item(0) + cos(chi_q) * ep.item(1)
        # course command
        self.autopilot_commands.course_command \
            = chi_q - self.chi_inf * (2 / np.pi) * np.arctan(self.k_path * path_error)
        # altitude command
        n = np.cross(np.array([[0, 0, 1]]), path.line_direction.T).T
        n = n / np.linalg.norm(n)
        s = ep - (ep.T @ n) * n
        self.autopilot_commands.altitude_command \
            = -path.line_origin.item(2) \
              - np.sqrt((s.item(0))**2 + (s.item(1))**2) * path.line_direction.item(2) \
              / np.sqrt(path.line_direction.item(0)**2 + path.line_direction.item(1)**2)
        # feedforward roll angle for straight line is zero
        self.autopilot_commands.phi_feedforward = 0.0

    def _follow_orbit(self, 
                      path: MsgPath, 
                      state: MsgState):
        self.autopilot_commands.mode = 'airspeed_altitude_course'
        if path.orbit_direction == 'CW':
            direction = 1.0
        if path.orbit_direction == 'CCW':
            direction = -1.0
        # airspeed command
        self.autopilot_commands.airspeed_command = path.airspeed
        # distance from orbit center
        d = np.sqrt((state.north - path.orbit_center.item(0))**2
                    + (state.east - path.orbit_center.item(1))**2)
        # compute wrapped version of angular position on orbit
        varphi = np.arctan2(state.east - path.orbit_center.item(1),
                            state.north - path.orbit_center.item(0))
        varphi = wrap(varphi, state.chi)
        # compute normalized orbit error
        orbit_error = (d - path.orbit_radius) / path.orbit_radius
        # course command
        self.autopilot_commands.course_command \
            = varphi + direction * (np.pi/2.0 + np.arctan(self.k_orbit * orbit_error))
        # altitude command
        self.autopilot_commands.altitude_command = -path.orbit_center.item(2)
        # roll feedforward command
        if orbit_error < 10:
            # self.autopilot_commands.phi_feedforward \
            #     = direction * np.arctan(path.airspeed**2 / self.gravity / path.orbit_radius)
            self.autopilot_commands.phi_feedforward \
                = direction * np.arctan(state.Vg** 2 / self.gravity / path.orbit_radius / np.cos(state.chi-state.psi))
        else:
            self.autopilot_commands.phi_feedforward = 0.0

    def _follow_helix(self, 
                      path: MsgPath, 
                      state: MsgState):
        #self.autopilot_commands.mode = 'airspeed_climbrate_roll'
        if path.orbit_direction == 'CW':
            direction = 1.0
        if path.orbit_direction == 'CCW':
            direction = -1.0
        # airspeed command
        self.autopilot_commands.airspeed_command = path.airspeed
        # alpha1: cylinder
        e_n = state.north - path.orbit_center.item(0)
        e_e = state.east - path.orbit_center.item(1)
        alpha1 = (e_n/path.orbit_radius)**2 + (e_e/path.orbit_radius)**2 - 1.0
        gradAlpha1 = np.array([[
            2*e_n/path.orbit_radius,
            2*e_e/path.orbit_radius,
            0.]])
        # alpha2: spiral plane
        # angular position on spiral, with logic to unwrap the angles
        varphi = np.arctan2(e_e, e_n) - path.helix_start_angle      
        if varphi-self._helix_varphi>np.pi:
            self._helix_spiral_index = self._helix_spiral_index-1
        if varphi-self._helix_varphi<-np.pi:
            self._helix_spiral_index = self._helix_spiral_index+1
        self._helix_varphi = varphi
        # definition of alpha2 and gradient
        alpha2 = (-state.altitude-path.orbit_center.item(2))/path.orbit_radius \
             + direction * np.tan(path.helix_climb_angle) \
                * (varphi + 2 * np.pi * self._helix_spiral_index)
        d = max(e_n**2 + e_e**2, 0.1)  # distance squared from spiral center
        gradAlpha2 = np.array([[
            -direction * np.tan(path.helix_climb_angle) * e_e / d,
            direction * np.tan(path.helix_climb_angle) * e_n / d,
            1/path.orbit_radius]])         
        # compute the commanded velocity vector
        V = 0.5 * (alpha1**2 + alpha2**2)
        gradV = alpha1 * gradAlpha1 + alpha2 * gradAlpha2
        u = -self.k_helix_grad * gradV \
            + direction * self.k_helix_cross * np.cross(gradAlpha1, gradAlpha2)
        # noralize to ensure velocity vector has length Va_d
        norm_u = np.linalg.norm(u)
        if norm_u != 0:
            u = path.airspeed * u / norm_u
 
        # desired course command
        chi_c = wrap(np.arctan2(u.item(1), u.item(0)), state.chi) 
        self.autopilot_commands.course_command = chi_c
        print('chi_c=', chi_c  * 180/np.pi)
        # commanded flight path angle
        commanded_flight_path_angle \
            =-saturate(np.arcsin(u.item(2)/path.airspeed), 
                        -np.radians(20), np.radians(20))
        # commanded altitude:  
        #   Using hdot = k_p(h^c - h) = -V sin(gamma^c) we get
        #   h^c = h + (1/k_p) V sin(gamma^c)
        one_over_kp = 0.2  # total guess
        self.autopilot_commands.altitude_command = state.altitude \
            - one_over_kp * state.Va * np.sin(commanded_flight_path_angle)

#        self.autopilot_commands.mode = 'airspeed_climbrate_roll'


        # roll feedforward command
        if d < 10:
            # self.autopilot_commands.phi_feedforward \
            #     = direction * np.arctan(path.airspeed**2 / self.gravity / path.orbit_radius)
            # self.autopilot_commands.phi_feedforward \
            #     = direction * np.arctan(state.Vg** 2 / self.gravity / path.orbit_radius / np.cos(state.chi-state.psi))
            self.autopilot_commands.phi_feedforward \
                = direction * np.arctan(state.Vg** 2 / self.gravity / path.orbit_radius)
        else:
            self.autopilot_commands.phi_feedforward = 0.0



