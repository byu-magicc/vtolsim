#This file implements the generators that create the trajectories for plane
import numpy as np

from message_types.msg_trajectory import MsgTrajectory

class TrajectoryGenerator():

    def __init__(self):
        self.segments = []
        self.end_time = 0.0

    def add_line_segment(self, line_segment):
        self.segments.append([line_segment, self.end_time, self.end_time + line_segment.time])
        self.end_time = self.end_time + line_segment.time

    def traj_msg(self, t):
        i = 0
        while t > self.segments[i][2]:
            i += 1
            assert i < len(self.segments)
        segment_t = t - self.segments[i][1]
        return self.segments[i][0].traj_msg(segment_t)

    def position(self, t):
        i = 0
        while t > self.segments[i][2]:
            i += 1
            assert i < len(self.segments)
        segment_t = t - self.segments[i][1]
        return self.segments[i][0].position(segment_t)

    def get_position_pts(self, time_scale):
        t = 0
        positions = np.ndarray((3,1))
        while t < self.end_time:
            position = self.position(t)
            positions = np.append(positions, position, axis=1)
            t += time_scale
        return positions



#creates the line segment class.
class LineSegment():

    def __init__(self, start_pos, start_vel, end_pos, end_vel):
        self.start_pos = start_pos
        start_vel_mag = start_vel
        self.end_pos = end_pos
        end_vel_mag = end_vel
        self.direction = (end_pos - start_pos) / np.linalg.norm(end_pos - start_pos)
        self.start_vel = self.direction * start_vel
        self.end_vel = self.direction * end_vel
        dist = np.linalg.norm(end_pos - start_pos)
        self.time = 2*dist/(start_vel_mag + end_vel_mag)
        self.acc = (self.end_vel - self.start_vel) / self.time
    
    def position(self, t):
        return .5*(self.velocity(t)-self.start_vel)*t + self.start_vel*t + self.start_pos

    def velocity(self, t):
        return (self.end_vel - self.start_vel) * t / self.time + self.start_vel

    def acceleration(self, t):
        return self.acc

    def traj_msg(self, t):
        pos = self.position(t)
        vel = self.velocity(t)
        accel = self.acceleration(t)
        yaw = 0
        data = np.concatenate((pos, vel, accel), axis=1)
        return np.concatenate((np.concatenate((data, np.zeros((3,2))), axis=1), np.zeros((1,5))), axis=0)

    def get_position_pts(self, time_scale):
        t = 0
        positions = np.array([[],[],[]])
        while t < self.time:
            position = self.position(t)
            positions = np.append(positions, position, axis=1)
            t += time_scale
        return positions


    def __str__(self):
        str = f'time: {self.time}\n' + \
            f'acceleration: {self.acceleration}\n'
        return str