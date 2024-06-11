"""
hover_at_point.py
    - Update history:
        5/14/2024 - RWB
    - hover at a point
"""
import numpy as np
from message_types.msg_trajectory import MsgTrajectory


class Planner:
    def __init__(self, 
                 position: np.ndarray=np.array([[0.], [0.], [-10.]]), 
                 heading: float=0.,
                 ):
        self.traj = MsgTrajectory(
            position=position,
            velocity=np.array([[0.], [0.], [0.]]),
            acceleration=np.array([[0.],[0.], [0.]]),
            heading=heading,
            heading_rate=0.)

    def update(self, 
               t: float=0. # current time,
               ) -> MsgTrajectory:
        return self.traj
    
    def set_position(self, 
            position: np.ndarray,
            ):
        self.position=position

    def set_heading(self, 
            heading: floaw,
            ):
        self.heading=heading