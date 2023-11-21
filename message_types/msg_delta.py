class MsgDelta:
    def __init__(self):
        self.elevator = 0.0  # elevator
        self.aileron = 0.0  # aileron
        self.rudder = 0.0  # rudder
        self.throttle_right = 0.0  # throttle for right motor 
        self.throttle_left = 0.0  # throttle for left motor
        self.throttle_rear = 0.0  # throttle for back motor
        self.motor_right = 0.0  # right motor commanded angle
        self.motor_left = 0.0  # left motor commanded angle
        # self.elevon_right = 0.0  # right elevon angle in radians
        # self.elevon_left = 0.0  # left elevon in radians
        # # need to be able to command left and right motor angles
        # self.servo_right = 0.0 # commanded right servo angle in radians
        # self.servo_left = 0.0 # commanded left servo angle in radians


