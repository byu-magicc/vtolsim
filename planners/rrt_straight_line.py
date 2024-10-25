# rrt straight line path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/3/2019 - Brady Moon
#         4/11/2019 - RWB
#         3/31/2020 - RWB
#         4/1/2024 - RWB
import numpy as np
from message_types.msg_waypoints import MsgWaypoints
from message_types.msg_world_map import MsgWorldMap


class RRTStraightLine:
    '''
        Implements RRT straight line planner

        Attributes
        ----------
        segment_length : float
            the length extended at each iteration of RRT
        waypoints_not_smooth : MsgWaypoints
            a set of waypoints defining the non-smoothed path from RRT
        tree : MsgWaypoints
            a set of waypoints defining the RRT tree

        Methods
        -------
        update(start_pose, end_pose, Va, world_map, radius)
            : plans a path through the world defined by world_map
            : from start_pose to end_pose at velocity Va, 
            : with turning radius
        extend_tree(tree, end_pose, Va, world_map)
            : extend the tree by one edge into feasible region
            : returns a flag if new node is close to end_pose
    '''
    def __init__(self):
        self.segment_length = 300 # standard length of path segments

    def update(self, 
               start_pose: np.ndarray, 
               end_pose: np.ndarray, 
               Va: float, 
               world_map: MsgWorldMap)->MsgWaypoints:
        tree = MsgWaypoints()
        #tree.type = 'straight_line'
        tree.type = 'fillet'
        # add the start pose to the tree
        tree.add(ned=start_pose, airspeed=Va)
        # check to see if start_pose connects directly to end_pose
        if (distance(start_pose, end_pose) < self.segment_length) \
                and (collision(start_pose, end_pose, world_map) is False):
            tree.add(ned=end_pose, airspeed=Va,
                     cost=distance(start_pose, end_pose),
                     parent=1, connect_to_goal=1)
        else:
            num_paths = 0
            while num_paths < 5:
                flag = self.extend_tree(tree, end_pose, Va, world_map)
                num_paths = num_paths + flag
        # find path with minimum cost to end_node
        waypoints_not_smooth = find_minimum_path(tree, end_pose)
        waypoints = smooth_path(waypoints_not_smooth, world_map)
        self.waypoints_not_smoothed = waypoints_not_smooth
        self.tree = tree
        return waypoints

    def extend_tree(self, 
                    tree: MsgWaypoints, 
                    end_pose: np.ndarray, 
                    Va: float, 
                    world_map: MsgWorldMap)->bool:
        # extend tree by randomly selecting pose and extending tree toward that pose
        flag1 = False
        while flag1 is False:
            random_pose_ = random_pose(world_map, end_pose.item(2))
            # find leaf on tree that is closest to random_pose
            tmp = tree.ned-np.tile(random_pose_, (1, tree.num_waypoints))
            tmp1 = np.diag(tmp.T @ tmp)
            idx = np.argmin(tmp1)
            dist = tmp1.item(idx)
            L = np.min([np.sqrt(dist), self.segment_length])
            cost = tree.cost.item(idx) + L
            tmp = random_pose_-column(tree.ned, idx)
            new_pose = column(tree.ned, idx) + L * (tmp / np.linalg.norm(tmp))

            if collision(column(tree.ned, idx), new_pose, world_map) is False:
                tree.add(ned=new_pose, airspeed=Va, cost=cost,
                         parent=idx, connect_to_goal=0)
                flag1 = True
            # check to see if new node connects directly to end_node
            if (distance(new_pose, end_pose) < self.segment_length)\
                    and (collision(new_pose, end_pose, world_map) is False):
                tree.connect_to_goal[-1] = True  # mark node as connecting to end.
                flag = True
            else:
                flag = False
        return flag
        
    # def process_app(self):
    #     self.planner_viewer.process_app()

def smooth_path(waypoints, world_map):
    # smooth the waypoint path
    smooth = [0]  # add the first waypoint
    ptr = 1
    while ptr <= waypoints.num_waypoints - 2:
        pose_s = column(waypoints.ned, smooth[-1])
        pose_e = column(waypoints.ned, ptr + 1)
        if (collision(pose_s, pose_e, world_map) == True):
            smooth.append(ptr)
        ptr += 1
    smooth.append(waypoints.num_waypoints - 1)
    # construct smooth waypoint path
    smooth_waypoints = MsgWaypoints()
    for i in smooth:
        smooth_waypoints.add(column(waypoints.ned, i),
                             waypoints.airspeed.item(i),
                             np.inf,
                             np.inf,
                             np.inf,
                             np.inf)
    smooth_waypoints.type = waypoints.type
    return smooth_waypoints


def find_minimum_path(tree, end_pose):
    # find the lowest cost path to the end node
    # find nodes that connect to end_node
    connecting_nodes = []
    for i in range(tree.num_waypoints):
        if tree.connect_to_goal.item(i) == 1:
            connecting_nodes.append(i)
    # find minimum cost last node
    idx = np.argmin(tree.cost[connecting_nodes])
    # construct lowest cost path order
    path = [connecting_nodes[idx]]  # last node that connects to end node
    parent_node = tree.parent.item(connecting_nodes[idx])
    while parent_node >= 1:
        path.insert(0, int(parent_node))
        parent_node = tree.parent.item(int(parent_node))
    path.insert(0, 0)
    # construct waypoint path
    waypoints = MsgWaypoints()
    for i in path:
        waypoints.add(column(tree.ned, i),
                      tree.airspeed.item(i),
                      np.inf,
                      np.inf,
                      np.inf,
                      np.inf)
    waypoints.add(end_pose,
                  tree.airspeed[-1],
                  np.inf,
                  np.inf,
                  np.inf,
                  np.inf)
    waypoints.type = tree.type
    return waypoints


def random_pose(world_map, pd):
    # generate a random pose
    pn = world_map.city_width * np.random.rand()
    pe = world_map.city_width * np.random.rand()
    pose = np.array([[pn], [pe], [pd]])
    return pose


def distance(start_pose, end_pose):
    # compute distance between start and end pose
    d = np.linalg.norm(start_pose - end_pose)
    return d


def collision(start_pose, end_pose, world_map):
    # check to see of path from start_pose to end_pose colliding with map
    collision_flag = False
    points = points_along_path(start_pose, end_pose, 100)
    for i in range(points.shape[1]):
        if height_above_ground(world_map, column(points, i)) <= 0:
            collision_flag = True
    return collision_flag


def height_above_ground(world_map, point):
    # find the altitude of point above ground level
    point_height = -point.item(2)
    tmp = np.abs(point.item(0)-world_map.building_north)
    d_n = np.min(tmp)
    idx_n = np.argmin(tmp)
    tmp = np.abs(point.item(1)-world_map.building_east)
    d_e = np.min(tmp)
    idx_e = np.argmin(tmp)
    if (d_n<world_map.building_width) and (d_e<world_map.building_width):
        map_height = world_map.building_height[idx_n, idx_e]
    else:
        map_height = 0
    h_agl = point_height - map_height
    return h_agl

def points_along_path(start_pose, end_pose, N):
    # returns points along path separated by Del
    points = start_pose
    q = (end_pose - start_pose)
    L = np.linalg.norm(q)
    q = q / L
    w = start_pose
    for i in range(1, N):
        w = w + (L / N) * q
        points = np.append(points, w, axis=1)
    return points


def column(A, i):
    # extracts the ith column of A and return column vector
    tmp = A[:, i]
    col = tmp.reshape(A.shape[0], 1)
    return col