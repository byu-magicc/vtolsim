#/usr/bin/python3
import sys
import numpy as np

sys.path.append('..')
from trajectory.trajectoryGenerator import LineSegment, TrajectoryGenerator


VTOL_Line_Flight = TrajectoryGenerator()
VTOL_Line_Flight.add_line_segment(LineSegment(
    start_pos=np.array([[0, 0, 0]]).T, 
    start_vel=0, 
    end_pos=np.array([[50, 0, -20]]).T, 
    end_vel=5))
VTOL_Line_Flight.add_line_segment(LineSegment(
    start_pos=np.array([[50, 0, -20]]).T, 
    start_vel=5, 
    end_pos=np.array([[150, 0, -20]]).T, 
    end_vel=10))
VTOL_Line_Flight.add_line_segment(LineSegment(
    start_pos=np.array([[150, 0, -20]]).T, 
    start_vel=10, 
    end_pos=np.array([[250, 0, -20]]).T, 
    end_vel=10))
VTOL_Line_Flight.add_line_segment(LineSegment(
    start_pos=np.array([[250, 0, -20]]).T, 
    start_vel=10, 
    end_pos=np.array([[350, 0, -20]]).T, 
    end_vel=5))
VTOL_Line_Flight.add_line_segment(LineSegment(
    start_pos=np.array([[350, 0, -20]]).T, 
    start_vel=5, 
    end_pos=np.array([[400, 0, 0]]).T, 
    end_vel=0))





#creates the straight line flight
straight_line_flight = TrajectoryGenerator()
straight_line_flight.add_line_segment(LineSegment(
    start_pos=np.array([[0, 0, -100]]).T,
    start_vel=25,
    end_pos=np.array([[1000, 0, -100]]).T,
    end_vel=25
))