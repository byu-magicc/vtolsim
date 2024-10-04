"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
        7/13/2023 - RWB
        3/20/2024 - RWB
"""
import numpy as np
import parameters.quad.control_parameters as AP
from controllers.quad.pi_control import PIControl
from controllers.quad.pid_control import PidControl
from controllers.quad.pd_control_with_rate import PDControlWithRate
#from controllers.tf_control import TFControl
from message_types.quad.msg_state import MsgState
from message_types.quad.msg_delta import MsgDelta
from message_types.quad.msg_autopilot_fixedWing import MsgAutopilotFixedWing
from tools.transfer_function import TransferFunction
from tools.wrap import wrap
from tools.saturate import saturate
from tools.rotations import rotation_to_euler, euler_to_rotation


class Autopilot:
    """
    Low-level autopilot using successive loop closure

    Attributes
    ----------
        roll_from_aileron : PDControlWithRate
            roll hold using ailerons
        course_from_roll : PIControl
            course hold by commanding roll
        yaw_damper : TransferFunction
            yaw damper to remove adverse yaw
        pitch_from_elevator : PDControlWithRate
            pitch attitude hold using elevator
        altitude_from_pitch : PIControl
            altitude hold by commanding pitch
        climbrate_from_pitch : PIDControl
            hold a climb rate (hdot) by commanding pitch
        airspeed_from_throttle : PIControl
            airspeed hold by commanding throttle
        commanded_state : MsgState
            commanded state message
    
    Methods
    -------
    update(cmd, state) : 
        update the autopilot
        Parameters
        ----------
        cmd : MsgAutopilot
            commands to the autopilot
        state: MsgState
            state of the aircraft
        Returns
        -------
        delta : MsgDelta 
            control signals
        commanded_states : MsgState 
            commanded states
    _update_airspeed_altitude_course(cmd, state) :
        called when cmd.mode=='airspeed_altitude_course'
        controls airspeed, airspeed, and course angle
    _update_airspeed_climbrate_roll(cmd, state) :
        called when cmd.mode=='airspeed_climbrate_roll'
        controls airspeed, climbrate (hdot), and roll angle
    """  
    def __init__(self, ts_control: float):
        # instantiate lateral-directional controllers
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
        #self.yaw_damper = TFControl(
        #                 k=AP.yaw_damper_kr,
        #                 n0=0.0,
        #                 n1=1.0,
        #                 d0=AP.yaw_damper_p_wo,
        #                 d1=1,
        #                 Ts=ts_control)
        # instantiate longitudinal controllers
        self.pitch_from_elevator = PDControlWithRate(
                        kp=AP.pitch_kp,
                        kd=AP.pitch_kd,
                        limit=np.radians(45))
        self.altitude_from_pitch = PIControl(
                        kp=AP.altitude_kp,
                        ki=AP.altitude_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.climbrate_from_pitch = PidControl(
                        kp=AP.climbrate_kp,
                        ki=AP.climbrate_ki,
                        kd=AP.climbrate_kd,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.airspeed_from_throttle = PIControl(
                        kp=AP.airspeed_throttle_kp,
                        ki=AP.airspeed_throttle_ki,
                        Ts=ts_control,
                        limit=1.0)
        self.commanded_state = MsgState()

    def update(self, 
               cmd: MsgAutopilotFixedWing, 
               state: MsgState):
        if cmd.mode=='airspeed_altitude_course':
            delta = self._update_airspeed_altitude_course(cmd, state)
        elif cmd.mode=='airspeed_climbrate_roll':
            delta = self._update_airspeed_climbrate_roll(cmd, state)
        return delta, self.commanded_state



    #we usually use this one
    def _update_airspeed_altitude_course(self,
                                         cmd: MsgAutopilotFixedWing,
                                         state: MsgState)->MsgDelta:
        
        #gets the state Rotation matrix
        state_R = state.R

        #gets the state roll pitch and yaw
        state_phi, state_theta, state_psi = rotation_to_euler(R=state_R)

        #gets the altitude, the negative of the position[2][0], which is the down position
        state_altitude = -(state.pos)[2][0]
        
        state_p = (state.omega)[0][0]
        state_q = (state.omega)[1][0]
        state_r = (state.omega)[2][0]

        # lateral autopilot
        chi_c = wrap(cmd.course_command, state.chi)
        phi_c = saturate(
                cmd.phi_feedforward + self.course_from_roll.update(chi_c, state.chi),
                -np.radians(30), np.radians(30))
        delta_a = self.roll_from_aileron.update(phi_c, state_phi, state_p)
        delta_r = self.yaw_damper.update(state_r)
        # longitudinal autopilot
        # saturate the altitude command
        altitude_c = saturate(cmd.altitude_command,
                              state_altitude - AP.altitude_zone,
                              state_altitude + AP.altitude_zone)
        theta_c = self.altitude_from_pitch.update(altitude_c, state_altitude)
        delta_e = self.pitch_from_elevator.update(theta_c, state_theta, state_q)
        delta_t = self.airspeed_from_throttle.update(cmd.airspeed_command, state.Va)
        delta_t = saturate(delta_t, 0.0, 1.0)
        # construct control outputs and commanded states
        delta = MsgDelta(elevator=delta_e,
                         aileron=delta_a,
                         rudder=delta_r,
                         forwardThrottle=delta_t)
        
        #gets the euler to rotation of the 

        #gets the commanded state rotation
        commanded_state_R = euler_to_rotation(phi=phi_c, theta=theta_c, psi=state_psi)
        

        (self.commanded_state.pos)[2][0] = -cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.R = commanded_state_R
        self.commanded_state.chi = cmd.course_command
        return delta





    def _update_airspeed_climbrate_roll(self,
                                        cmd: MsgAutopilotFixedWing,
                                        state: MsgState)->tuple[MsgDelta, MsgState]:
        
        #gets the state Rotation matrix
        state_R = state.R

        #gets the state roll pitch and yaw
        state_phi, state_theta, state_psi = rotation_to_euler(R=state_R)

        #gets the altitude, the negative of the position[2][0], which is the down position
        state_altitude = -(state.pos)[2][0]
        
        state_p = (state.omega)[0][0]
        state_q = (state.omega)[1][0]
        state_r = (state.omega)[2][0]        
        
        # lateral autopilot
        phi_c = cmd.roll_command
        delta_a = self.roll_from_aileron.update(phi_c, state.phi, state_p)
        delta_r = self.yaw_damper.update(state.r)
        # longitudinal autopilot
        climbrate = state.Va * np.sin(state.gamma)
        theta_c = self.climbrate_from_pitch.update(cmd.climb_rate_command, climbrate)
        delta_e = self.pitch_from_elevator.update(theta_c, state_theta, state_q)
        delta_t = self.airspeed_from_throttle.update(cmd.airspeed_command, state.Va)
        delta_t = saturate(delta_t, 0.0, 1.0)
        # construct control outputs and commanded states
        delta = MsgDelta(elevator=delta_e,
                         aileron=delta_a,
                         rudder=delta_r,
                         throttle=delta_t)
        self.commanded_state.gamma = np.arcsin(cmd.climb_rate_command/cmd.airspeed_command)
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = phi_c
        self.commanded_state.theta = theta_c
        self.commanded_state.chi = cmd.course_command
        return delta

