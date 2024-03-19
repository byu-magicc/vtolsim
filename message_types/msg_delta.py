import numpy as np
from message_types.msg_controls import MsgControls

class MsgDelta:
    '''
        Message class that defines the control inputs to the eVTOL

        Attributes:
            elevator: elevator angle in radians
            aileron: aileron in radians
            rudder: rudder in radians
            throttle_right: throttle for right rotor in [0, 1]
            throttle_left:  throttle for left rotor in [0, 1]
            throttle_rear: throttle for rear rotor in [0, 1]
            motor_right: right motor angle in radians
            motor_left: left motor angle in radians
    '''
    def __init__(self, old: MsgControls=None):
        if old==None:
            self.elevator = np.radians(0.0)  
            self.aileron = np.radians(0.0)  
            self.rudder = np.radians(0.0)  
            self.throttle_right = float(0.0)   
            self.throttle_left = float(0.0)  
            self.throttle_rear = float(0.0)  
            self.motor_right = np.radians(0.0)  
            self.motor_left = np.radians(0.0)  
        else:
            self.elevator =  old.elevon_left + old.elevon_right
            self.aileron = old.elevon_left - old.elevon_right
            self.rudder = np.radians(0.0)  
            self.throttle_right = old.throttle_right 
            self.throttle_left = old.throttle_left
            self.throttle_rear = old.throttle_rear
            self.motor_right = old.servo_right
            self.motor_left = old.servo_left

    def old_format(self)->MsgControls:    
        old = MsgControls()
        old.elevon_right = 0.5 * (self.elevator - self.aileron)  
            # right elevon angle in radians
        old.elevon_left = 0.5 * (self.elevator + self.aileron)  
            # left elevon in radians
        old.throttle_rear = self.throttle_rear  
            # commanded throttle for rear rotor
        old.throttle_right = self.throttle_right  
            # commanded throttle for right rotor
        old.throttle_left = self.throttle_left  
            # commanded throttle for left rotor
        old.servo_right = self.motor_right 
            # commanded right servo angle in radians
        old.servo_left = self.motor_left 
            # commanded left servo angle in radians
        return old
    
    def from_array(self, delta_array: np.ndarray):
        self.elevator =  delta_array.item(0)
        self.aileron = delta_array.item(1)
        self.rudder = delta_array.item(2)  
        self.throttle_right = delta_array.item(3) 
        self.throttle_left = delta_array.item(4)
        self.throttle_rear = delta_array.item(5)
        self.motor_right = delta_array.item(6)
        self.motor_left = delta_array.item(7)

    def to_array(self)->np.ndarray:
        '''
            Convert MsgDelta structure to array:
            :output: np.array([[
                elevator, 
                aileron, 
                rudder,
                throttle_right,
                throttle_left,
                throttle_rear,
                motor_right,
                motor_left,
                ]]).T
        '''
        delta = np.zeros((8,1))
        delta[0,0] = self.elevator
        delta[1,0] = self.aileron
        delta[2,0] = self.rudder  
        delta[3,0] = self.throttle_right
        delta[4,0] = self.throttle_left
        delta[5,0] = self.throttle_rear
        delta[6,0] = self.motor_right
        delta[7,0] = self.motor_left
        return delta

    def __add__(self, other):
        '''Overload the addition '+' operator'''
        out = MsgDelta()
        out.elevator = self.elevator + other.elevator
        out.aileron = self.aileron + other.aileron  
        out.rudder = self.rudder + other.rudder  
        out.throttle_right = self.throttle_right + other.throttle_right  
        out.throttle_left = self.throttle_left + other.throttle_left  
        out.throttle_rear = self.throttle_rear + other.throttle_rear  
        out.motor_right = self.motor_right + other.motor_right  
        out.motor_left = self.motor_left + other.motor_left  
        return out

    def __sub__(self, other):
        '''Overload the subtraction '-' operator'''
        out = MsgDelta()
        out.elevator = self.elevator - other.elevator
        out.aileron = self.aileron - other.aileron  
        out.rudder = self.rudder - other.rudder  
        out.throttle_right = self.throttle_right - other.throttle_right  
        out.throttle_left = self.throttle_left - other.throttle_left  
        out.throttle_rear = self.throttle_rear - other.throttle_rear  
        out.motor_right = self.motor_right - other.motor_right  
        out.motor_left = self.motor_left - other.motor_left  
        return out
    
    def __rmul__(self, other):
        '''Overload right multiply by a scalar'''
        out = MsgDelta()
        out.elevator = other * self.elevator
        out.aileron = other * self.aileron 
        out.rudder = other * self.rudder  
        out.throttle_right = other * self.throttle_right  
        out.throttle_left = other * self.throttle_left  
        out.throttle_rear = other * self.throttle_rear  
        out.motor_right = other * self.motor_right  
        out.motor_left = other * self.motor_left  
        return out

