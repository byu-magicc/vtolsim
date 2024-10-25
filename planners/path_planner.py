# path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/3/2019 - RWB
#         3/30/2022 - RWB
#         7/13/2023 - RWB
#         4/1/2024 - RWB
import numpy as np
from message_types.msg_waypoints import MsgWaypoints
from message_types.msg_state import MsgState
from message_types.msg_world_map import MsgWorldMap
from planners.rrt_straight_line import RRTStraightLine
from planners.rrt_dubins import RRTDubins


class PathPlanner:
    '''
        Path planner

        Attributes
        ----------
        waypoints : MsgWaypoints
            a set of waypoints defining the (smoothed) planned path
        waypoints_not_smooth : MsgWaypoints
            a set of waypoints defining the non-smoothed path from RRT
        tree : MsgWaypoints
            a set of waypoints defining the RRT tree
        _type : str
            contains the method used for planning
                'simple_straight' - a simple set of straight-line waypoints
                'simple_dubins' - a simple set of dubins waypoints
                'rrt_straight' - RRT using straight-line planning
                'rrt_dubins' - RRT using Dubins planning

        Methods
        -------
        update(world_map, state, radius)
            : update the planner using the current world map, 
            : the state of the MAV, and the turn radius
    '''
    def __init__(self, type: str='rrt_dubins'):
        # waypoints definition
        self.waypoints = MsgWaypoints()
        if type == 'rrt_straight':
            self.rrt_straight_line = RRTStraightLine()
        if type == 'rrt_dubins':
            self.rrt_dubins = RRTDubins()
        self._type = type

    def update(self, 
               world_map: MsgWorldMap, 
               state: MsgState, 
               radius: float):
        print('planning...')
        if self._type == 'simple_straight':
            Va = 25
            self.waypoints.type = 'fillet'
            self.waypoints.add(np.array([[0, 0, -100]]).T, Va, np.inf, np.inf, 0, 0)
            self.waypoints.add(np.array([[1000, 0, -100]]).T, Va, np.inf, np.inf, 0, 0)
            self.waypoints.add(np.array([[0, 1000, -100]]).T, Va, np.inf, np.inf, 0, 0)
            self.waypoints.add(np.array([[1000, 1000, -100]]).T, Va, np.inf, np.inf, 0, 0)

        elif self._type == 'simple_dubins':
            Va = 25
            self.waypoints.type = 'dubins'
            self.waypoints.add(np.array([[0, 0, -100]]).T, Va, np.radians(0), np.inf, 0, 0)
            self.waypoints.add(np.array([[1000, 0, -100]]).T, Va, np.radians(45), np.inf, 0, 0)
            self.waypoints.add(np.array([[0, 1000, -100]]).T, Va, np.radians(45), np.inf, 0, 0)
            self.waypoints.add(np.array([[1000, 1000, -100]]).T, Va, np.radians(-135), np.inf, 0, 0)

        elif self._type == 'rrt_straight':
            desired_airspeed = 25
            desired_altitude = 100
            # start pose is current pose
            start_pose = np.array([[state.north], 
                                   [state.east], 
                                   [-desired_altitude]])
            # desired end pose
            if np.linalg.norm(start_pose[0:2]) < world_map.city_width / 2:
                end_pose = np.array([[world_map.city_width], 
                                     [world_map.city_width],
                                     [-desired_altitude]])
            else:  # or to the bottom-left corner of world_map
                end_pose = np.array([[0], [0], [-desired_altitude]])
            self.waypoints = self.rrt_straight_line.update(
                start_pose, 
                end_pose, 
                desired_airspeed, 
                world_map)
            self.waypoints_not_smooth = self.rrt_straight_line.waypoints_not_smoothed
            self.tree = self.rrt_straight_line.tree
            
        elif self._type == 'rrt_dubins':
            desired_airspeed = 25
            desired_altitude = 100
            # start pose is current pose
            start_pose = np.array([[state.north], [state.east],
                                   [-desired_altitude], [state.chi]])
            # desired end pose
            # either plan to the top-right corner of world_map
            if np.linalg.norm(start_pose[0:2]) < world_map.city_width / 2:
                end_pose = np.array([[world_map.city_width], [world_map.city_width],
                                     [-desired_altitude], [state.chi]])
            else:  # or to the bottom-left corner of world_map
                end_pose = np.array([[0], [0], [-desired_altitude], [state.chi]])
            self.waypoints = self.rrt_dubins.update(
                start_pose, 
                end_pose,
                desired_airspeed, 
                world_map, 
                radius)
            self.waypoints_not_smooth = self.rrt_dubins.waypoint_not_smooth
            self.tree = self.rrt_dubins.tree
        else:
            print("Error in Path Planner: Undefined planner type.")
        self.waypoints.plot_updated = False
        print('...done planning.')
        return self.waypoints
