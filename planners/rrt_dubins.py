# rrt dubins path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/16/2019 - RWB
import numpy as np
from message_types.msg_waypoints import MsgWaypoints
from planners.dubins_parameters import DubinsParameters


class RRTDubins:
    def __init__(self):
        self.segment_length = 450  # standard length of path segments
        self.dubins_path = DubinsParameters()

    def update(self, start_pose, end_pose, Va, world_map, radius):
        self.segment_length = 4 * radius
        tree = MsgWaypoints()
        tree.type = 'dubins'
        # add the start pose to the tree
        tree.add(ned=start_pose[0:3], airspeed=Va, course=start_pose.item(3))
        # check to see if start_pose connects directly to end_pose
        waypoints = MsgWaypoints()
        waypoints_not_smooth = MsgWaypoints()
        if distance(start_pose, end_pose) <= 4 * radius \
            and distance(start_pose, end_pose) > self.segment_length \
            and self.collision(start_pose, end_pose, world_map, radius) is False:
            tree.add(ned=end_pose[0:3], airspeed=Va, course=end_pose.item(3),
                     cost=distance(start_pose, end_pose), parent=1, connect_to_goal=1)
        else:
            num_paths = 0
            while num_paths < 3:
                flag = self.extendTree(tree, end_pose, Va, world_map, radius)
                num_paths = num_paths + flag
        # find path with minimum cost to end_node
        waypoints_not_smooth = findMinimumPath(tree, end_pose)
        waypoints = self.smoothPath(waypoints_not_smooth, world_map, radius)
        self.waypoint_not_smooth = waypoints_not_smooth
        self.tree = tree
        return waypoints

    def extendTree(self, tree, end_pose, Va, world_map, radius):
        # extend tree by randomly selecting pose and extending tree toward that pose
        flag1 = False
        while flag1 is False:
            random_pose = randomPose(world_map, end_pose.item(2))
            # find leaf on tree that is closest to new_pose
            tmp = tree.ned-np.tile(random_pose[0:3], (1, tree.num_waypoints))
            tmp1 = np.diag(tmp.T @ tmp)
            idx = np.argmin(tmp1)
            dist = np.sqrt(tmp1.item(idx))
            L = np.max([np.min([dist, self.segment_length]), 3*radius])
            cost = tree.cost.item(idx) + L
            tmp = random_pose[0:3]-column(tree.ned, idx)
            new_ned = column(tree.ned, idx) + L * (tmp / np.linalg.norm(tmp))
            new_chi = np.arctan2(new_ned.item(1) - tree.ned[1, idx],
                                 new_ned.item(0) - tree.ned[0, idx])
            new_pose = np.concatenate((new_ned, np.array([[new_chi]])), axis=0)
            tree_pose = np.concatenate((column(tree.ned, idx), np.array([[tree.course.item(idx)]])), axis=0)
            flag = False
            if self.collision(tree_pose, new_pose, world_map, radius) is False:
                tree.add(ned=new_pose[0:3], airspeed=Va, course=new_chi,
                         cost=cost, parent=idx, connect_to_goal=0)
                flag1=True
                # check to see if new node connects directly to end_node
                if distance(new_pose, end_pose) >= 3*radius \
                    and distance(new_pose, end_pose) < self.segment_length \
                    and self.collision(new_pose, end_pose, world_map, radius) is False:
                    tree.connect_to_goal[-1] = 1  # mark node as connecting to end.
                    flag = True
        return flag

    def collision(self, start_pose, end_pose, world_map, radius):
        # check to see of path from start_pose to end_pose colliding with world_map
        collision_flag = False
        #Del = 0.05
        self.dubins_path.update(start_pose[0:3], start_pose.item(3), end_pose[0:3], end_pose.item(3), radius)
        points = self.dubins_path.compute_points()
        for i in range(points.shape[0]):
            if heightAboveGround(world_map, column(points.T, i)) <= 5:
                collision_flag = True
        return collision_flag

    def process_app(self):
        self.planner_viewer.process_app()

    def smoothPath(self, waypoints, world_map, radius):
        # smooth the waypoint path
        smooth = [0]  # add the first waypoint
        ptr = 1
        while ptr <= waypoints.num_waypoints - 2:
            start_pose = np.concatenate((column(waypoints.ned, smooth[-1]), np.array([[waypoints.course[smooth[-1]]]])),
                                        axis=0)
            end_pose = np.concatenate((column(waypoints.ned, ptr + 1), np.array([[waypoints.course[ptr + 1]]])), axis=0)
            if self.collision(start_pose, end_pose, world_map, radius) is True \
                    and distance(start_pose, end_pose) >= 2 * radius:
                smooth.append(ptr)
            ptr += 1
        smooth.append(waypoints.num_waypoints - 1)
        # construct smooth waypoint path
        smooth_waypoints = MsgWaypoints()
        for i in smooth:
            smooth_waypoints.add(column(waypoints.ned, i),
                                 waypoints.airspeed.item(i),
                                 waypoints.course.item(i),
                                 np.inf,
                                 np.inf,
                                 np.inf)
        smooth_waypoints.type = waypoints.type
        return smooth_waypoints


def findMinimumPath(tree, end_pose):
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
                      tree.course.item(i),
                      np.inf,
                      np.inf,
                      np.inf)
    waypoints.add(end_pose[0:3],
                  tree.airspeed[-1],
                  end_pose.item(3),
                  np.inf,
                  np.inf,
                  np.inf)
    waypoints.type = tree.type
    return waypoints


def distance(start_pose, end_pose):
    # compute distance between start and end pose
    d = np.linalg.norm(start_pose[0:3] - end_pose[0:3])
    return d


def heightAboveGround(world_map, point):
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


def randomPose(world_map, pd):
    # generate a random pose
    pn = world_map.city_width * np.random.rand()
    pe = world_map.city_width * np.random.rand()
    chi = 0
    pose = np.array([[pn], [pe], [pd], [chi]])
    return pose


def mod(x):
    # force x to be between 0 and 2*pi
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x


def column(A, i):
    # extracts the ith column of A and return column vector
    tmp = A[:, i]
    col = tmp.reshape(A.shape[0], 1)
    return col
