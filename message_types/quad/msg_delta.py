import numpy as np

#the controls are as follows:
#elevator
#aileron
#rudder
#forward throttle
#vertical throttle 1 - Port Rear
#vertical throttle 2 - Port Front
#vertical throttle 3 - Starboard Front
#vertical throttle 4 - Starboard Rear


#defines the deltas for the controls
class MsgDelta:

    #creates the initialization function
    def __init__(self,
                 elevator=0.0,
                 aileron=0.0,
                 rudder=0.0,
                 forwardThrottle=0.0,
                 verticalThrottle_1=0.0,
                 verticalThrottle_2=0.0,
                 verticalThrottle_3=0.0,
                 verticalThrottle_4=0.0):
        #control surfaces are defines between -1.0 to 1.0
        #elevator like normal
        self.elevator = float(elevator)
        #aileron like normal
        self.aileron = float(aileron)
        #rudder like normal
        self.rudder = float(rudder)

        #throttles are defined between 0.0 to 1.0

        #sets the forward throttle, which is the main propulsion throttle
        self.forwardThrottle = float(forwardThrottle)
        #sets the vertical throttle 1, which is the port rear throttle
        self.verticalThrottle_1 = float(verticalThrottle_1)
        #sets the vertical throttle 2, which is the port front throttle
        self.verticalThrottle_2 = float(verticalThrottle_2)
        #sets the vertical throttle 3, which is the starboard front throttle
        self.verticalThrottle_3 = float(verticalThrottle_3)
        #sets the vertical throttle 4, which is the starboard rear throttle
        self.verticalThrottle_4 = float(verticalThrottle_4)


    #creates the helper function to go to from array to set the deltas
    def from_array(self, delta_array: np.ndarray):
        self.elevator = delta_array.item(0)
        self.aileron = delta_array.item(1)
        self.rudder = delta_array.item(2)

        self.forwardThrottle = delta_array.item(3)
        self.verticalThrottle_1 = delta_array.item(4)
        self.verticalThrottle_2 = delta_array.item(5)
        self.verticalThrottle_3 = delta_array.item(6)
        self.verticalThrottle_4 = delta_array.item(7)


    #sends the information to an array
    def to_array(self)->np.ndarray:

        #creates the delta
        delta = np.zeros((8,1))
        delta[0,0] = self.elevator
        delta[1,0] = self.aileron
        delta[2,0] = self.rudder

        delta[3,0] = self.forwardThrottle
        delta[4,0] = self.verticalThrottle_1
        delta[5,0] = self.verticalThrottle_2
        delta[6,0] = self.verticalThrottle_3
        delta[7,0] = self.verticalThrottle_4

        #returns the delta
        return delta