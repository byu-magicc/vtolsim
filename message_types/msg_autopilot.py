"""
msg_autopilot
    - messages type for input to the autopilot
    
part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        2/5/2019 - RWB
        3/20/2024 - RWB
"""


class MsgAutopilot:
    """
    Message type for communicating with low-level autopilot

    Attributes
    ----------
        mode : str 
            autopilot mode.  
            Current options:
                'airspeed_altitude_course'
                    command airspeed, alitude, and course
                'airspeed_climbrate_roll'
                    command airspeed, climbrate, and roll
        airspeed_command : float
            commanded airspeed in m/s
        course_command : float 
            commanded course in radians
            used in 'airspeed_altitude_course' mode
        altitude_command : float
            commanded altitude in meters
            used in 'airspeed_altitude_course' mode
        climb_rate_command : float
            commanded climb_rate (h_dot) in m/s
            used in 'airspeed_climbrate_roll' mode
        roll_command : float
            commanded roll angle in radians
            used in 'airspeed_climbrate_roll' mode
        phi_feedforward : float
            feedforward command for roll angle in radians.  
            Used in orbit following.
    
    Methods
    -------
    """  
    def __init__(self):
        self.mode = 'airspeed_altitude_course'
        #self.mode = 'airspeed_climbrate_roll'
        self.airspeed_command = float(0.0) 
        self.course_command = float(0.0)  
        self.altitude_command = float(0.0)  
        self.climb_rate_command = float(0.0) 
        self.roll_command = float(0.0)  
        self.phi_feedforward = float(0.0)  
