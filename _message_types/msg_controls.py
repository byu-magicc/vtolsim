"""
msg_delta
    - messages type for low level control surfaces

    - Last update:
        5/8/2019 - R.W. Beard
        11/20/2023 - R.W. Beard
"""
import numpy as np

class msgControls:
    def __init__(self):
        self.elevator = 0.0  # elevator
        self.aileron = 0.0  # aileron
        self.rudder = 0.0  # rudder
        self.trottle = np.array([[0.0],  # trottle for right motor 
                                 [0.0],  # trottle for left motor
                                 [0.0]]) # trottle for back motor
        self.motor_angle_cmd = np.array([[0.0],   # right motor commanded angle
                                         [0.0]])  # left motor commanded angle
        # self.elevon_right = 0.0  # right elevon angle in radians
        # self.elevon_left = 0.0  # left elevon in radians
        # self.throttle_rear = 0.0  # commanded throttle for rear rotor
        # self.throttle_right = 0.0  # commanded throttle for right rotor
        # self.throttle_left = 0.0  # commanded throttle for left rotor
        # # need to be able to command left and right motor angles
        # self.servo_right = 0.0 # commanded right servo angle in radians
        # self.servo_left = 0.0 # commanded left servo angle in radians
    
    # def __str__(self):
    #     to_str = ''
    #     to_str += 'elevon_right: {}\n'.format(self.elevon_right)
    #     to_str += 'elevon_left: {}\n'.format(self.elevon_left)
    #     to_str += 'throttle_rear: {}\n'.format(self.throttle_rear)
    #     to_str += 'throttle_right: {}\n'.format(self.throttle_right)
    #     to_str += 'throttle_left: {}\n'.format(self.throttle_left)
    #     to_str += 'servo_right: {}\n'.format(self.servo_right)
    #     to_str += 'servo_left: {}\n'.format(self.servo_left)
    #     return to_str
