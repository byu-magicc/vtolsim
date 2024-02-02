from message_types.msg_controls import MsgControls

class MsgDelta:
    def __init__(self, old=None):
        if old==None:
            self.elevator = 0.0  # elevator
            self.aileron = 0.0  # aileron
            self.rudder = 0.0  # rudder
            self.throttle_right = 0.0  # throttle for right motor 
            self.throttle_left = 0.0  # throttle for left motor
            self.throttle_rear = 0.0  # throttle for back motor
            self.motor_right = 0.0  # right motor commanded angle
            self.motor_left = 0.0  # left motor commanded angle
        else:
            self.elevator =  old.elevon_left + old.elevon_right
            self.aileron = old.elevon_left - old.elevon_right
            self.rudder = 0.0  
            self.throttle_right = old.throttle_right 
            self.throttle_left = old.throttle_left
            self.throttle_rear = old.throttle_rear
            self.motor_right = old.servo_right
            self.motor_left = old.servo_left

    def old_format(self):    
        old = MsgControls()
        old.elevon_right = 0.5 * (self.elevator - self.aileron)  # right elevon angle in radians
        old.elevon_left = 0.5 * (self.elevator + self.aileron)  # left elevon in radians
        old.throttle_rear = self.throttle_rear  # commanded throttle for rear rotor
        old.throttle_right = self.throttle_right  # commanded throttle for right rotor
        old.throttle_left = self.throttle_left  # commanded throttle for left rotor
        old.servo_right = self.motor_right # commanded right servo angle in radians
        old.servo_left = self.motor_left # commanded left servo angle in radians
        return old

