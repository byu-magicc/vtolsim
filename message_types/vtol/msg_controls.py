"""
msg_controls
    - messages type for low level control surfaces

    - Last update:
        5/8/2019 - R.W. Beard
"""
import numpy as np

class MsgControls:
    '''
        Message class that defines the control inputs to the eVTOL

        Attributes:
            elevon_right: right elevon angle in radians
            elevon_left: left elevon in radians
            throttle_rear: throttle for rear rotor in [0, 1]
            throttle_right: throttle for right rotor in [0, 1]
            throttle_left:  throttle for left rotor in [0, 1]
            servo_right: right servo angle in radians
            servo_left: left servo angle in radians
    '''
    def __init__(self):
        self.elevon_right = np.radians(0.0)  
        self.elevon_left = np.radians(0.0)  
        self.throttle_rear = float(0.0)  
        self.throttle_right = float(0.0)  
        self.throttle_left = float(0.0) 
        self.servo_right = np.radians(0.0)
        self.servo_left = np.radians(0.0)

