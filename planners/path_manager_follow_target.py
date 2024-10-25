"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - RWB
        3/30/2022 - RWB
        4/1/2022 - RWB
        7/13/2023 - RWB
"""
import numpy as np
from message_types.msg_path import MsgPath


class PathManager:
    def __init__(self):
        # message sent to path follower
        self.path = MsgPath()
        self.manager_requests_waypoints = True

    def update(self, target_position):
        self.path.set(
            type='orbit',
            airspeed=25,
            orbit_center=np.array([
                [target_position.item(0)],
                [target_position.item(1)],
                [-200.]]),
            orbit_radius=200.,
            orbit_direction='CW',
        )
        return self.path
