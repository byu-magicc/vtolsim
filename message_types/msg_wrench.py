#this file implements the message for the wrench. Since it is so important to constantly be reading in and out wrenches
#then, it is high time we devote a message class to the subject

#the big problem I need to solve is how to keep the length 6 and length 5 wrenches straight.s

import numpy as np

class Msg_Wrench:

    def __init__(self):
        #initializes all of the force and moment values to zero
        self.F_x = 0.0
        self.F_y = 0.0
        self.F_z = 0.0
        self.M_x = 0.0
        self.M_y = 0.0
        self.M_z = 0.0

    #defines the getter and setter functions for the length 6 vector
    def getWrench_full(self):
        return np.array([[self.F_x], [self.F_y], [self.F_z], [self.M_x], [self.M_y], [self.M_z]])

    def setWrench_full(self, wrenchInput: np.ndarray):
        self.F_x = wrenchInput.item(0)
        self.F_y = wrenchInput.item(1)
        self.F_z = wrenchInput.item(2)

        self.M_x = wrenchInput.item(3)
        self.M_y = wrenchInput.item(4)
        self.M_z = wrenchInput.item(5)

    #defines a setter and getter function for the length 5 vector
    def getWrench_five(self):
        return np.array([[self.F_x], [self.F_z], [self.M_x], [self.M_y], [self.M_z]])

    def setWrench_full(self, wrenchInput: np.ndarray):
        self.F_x = wrenchInput.item(0)
        self.F_z = wrenchInput.item(1)

        self.M_x = wrenchInput.item(2)
        self.M_y = wrenchInput.item(3)
        self.M_z = wrenchInput.item(4)
