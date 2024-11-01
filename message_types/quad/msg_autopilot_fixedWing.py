#creates the autopilot message for fixed wing mode

class MsgAutopilotFixedWing:

    #creates the init function
    def __init__(self):
        self.mode = 'airspeed_altitude_course'
        #self.mode = 'airspeed_climbrate_roll'
        self.airspeed_command = float(0.0)
        self.course_command = float(0.0)
        self.altitude_command = float(0.0)
        self.climb_rate_command = float(0.0)
        self.roll_command = float(0.0)
        self.phi_feedforward = float(0.0)