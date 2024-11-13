#/usr/bin/python3
import sys
import numpy as np

sys.path.append('..')
from trajectory.trajectoryGenerator import LineSegment, TrajectoryGenerator


lineFlight = TrajectoryGenerator()
lineFlight.add_line_segment(LineSegment(
    start_pos=np.array([[0, 0, 0]]).T, 
    start_vel=0, 
    end_pos=np.array([[50, 0, -20]]).T, 
    end_vel=5))
lineFlight.add_line_segment(LineSegment(
    start_pos=np.array([[50, 0, -20]]).T, 
    start_vel=5, 
    end_pos=np.array([[150, 0, -20]]).T, 
    end_vel=10))
lineFlight.add_line_segment(LineSegment(
    start_pos=np.array([[150, 0, -20]]).T, 
    start_vel=10, 
    end_pos=np.array([[250, 0, -20]]).T, 
    end_vel=10))
lineFlight.add_line_segment(LineSegment(
    start_pos=np.array([[250, 0, -20]]).T, 
    start_vel=10, 
    end_pos=np.array([[350, 0, -20]]).T, 
    end_vel=5))
lineFlight.add_line_segment(LineSegment(
    start_pos=np.array([[350, 0, -20]]).T, 
    start_vel=5, 
    end_pos=np.array([[400, 0, 0]]).T, 
    end_vel=0))