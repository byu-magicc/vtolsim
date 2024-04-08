"""
    - Update history:
        7/1/2021 - RWB
        6/21/2022 - RWB
        4/8/2024 - RWB
"""
import numpy as np


class MsgTrajectory:
    '''
        Trajectory message, defining the instanteous position, velocity, acceleration, and heading
    '''
    def __init__(self, 
                 position: np.ndarray=np.array([[0.], [0.], [0.]]),
                 velocity: np.ndarray=np.array([[0.], [0.], [0.]]),
                 acceleration: np.ndarray=np.array([[0.], [0.], [0.]]),
                 heading: float=0.,
                 ):
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.heading = heading
