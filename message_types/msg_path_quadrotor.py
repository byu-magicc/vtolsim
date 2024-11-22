#this file implements the path message for the quadrotor flight

import numpy as np


class MsgPathQuadrotor:
    def __init__(self):
        self.position = np.array([[0.], [0.], [0.]])
        self.velocity = np.array([[0.], [0.], [0.]])
        self.acceleration = np.array([[0.], [0.], [0.]])
        self.heading = 0.