"""
msg_controls
    - messages type for low level control surfaces

    - Last update:
        5/8/2019 - R.W. Beard
"""


class MsgControls:
    def __init__(self):
        self.elevon_right = 0.0  # right elevon angle in radians
        self.elevon_left = 0.0  # left elevon in radians
        self.throttle_rear = 0.0  # commanded throttle for rear rotor
        self.throttle_right = 0.0  # commanded throttle for right rotor
        self.throttle_left = 0.0  # commanded throttle for left rotor
        # need to be able to command left and right motor angles
        self.servo_right = 0.0 # commanded right servo angle in radians
        self.servo_left = 0.0 # commanded left servo angle in radians

