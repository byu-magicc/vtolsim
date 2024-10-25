"""
msg_path
    - messages type for input to path follower
    
part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        3/11/2019 - RWB
        3/20/2024 - RWB
"""
import numpy as np


class MsgPath:
    '''
        Message class that defines a path
        'line' paths are defined by
            airspeed - desired airspeed along the line
            line_origin - origin of the straight path line (r)
            line_direction - direction of line -unit vector- (q)
        'orbit' paths are defined by
            airspeed - desired airspeed along the orbit
            orbit_center - center of the orbit (c)
            orbit_radius - radius of the orbit (rho)
            orbit_direction - orbit direction: 'CW'==clockwise, 'CCW'==counter clockwise
        'helix' paths are defined by
            airspeed - desired airspeed along helical path
            orbit_center - center of the helix
            orbit_radius - radius of the helix
            orbit_direction - helix direction: 'CW'==clockwise, 'CCW'==counter clockwise
            helix_start_angle - start angle from north of helical path
            helix_climb_angle - climb angle of helix
        plot_updated is for drawing purposes
    '''
    def __init__(self, 
                 type: str='line',
                 airspeed: float=25,
                 line_origin: np.ndarray=np.array([[0.0, 0.0, 0.0]]).T,
                 line_direction: np.ndarray=np.array([[1.0, 0.0, 0.0]]).T,
                 orbit_center: np.ndarray=np.array([[0.0, 0.0, 0.0]]).T,
                 orbit_radius: float=50,
                 orbit_direction: str='CW',
                 helix_start_angle: float=0,
                 helix_climb_angle: float=0):
        self.type = type
        self.airspeed = airspeed
        self.line_origin = line_origin
        self.line_direction = line_direction/np.linalg.norm(line_direction)
        self.orbit_center = orbit_center
        self.orbit_radius = orbit_radius
        self.orbit_direction = orbit_direction
        self.helix_start_angle = helix_start_angle
        self.helix_climb_angle = helix_climb_angle

        # flag that indicates that path has been plotted
        self.plot_updated = bool(False)

    def set(self,
            type: str='line',
            airspeed: float=25,
            line_origin: np.ndarray=np.array([[0.], [0.], [0.]]),
            line_direction: np.ndarray=np.array([[1.], [0.], [0.]]),
            orbit_center: np.ndarray=np.array([[0.], [0.], [0.]]),
            orbit_radius: float=100,
            orbit_direction: str='CW',
            helix_start_angle: float=0,
            helix_climb_angle: float=0,
            ):
        if type=='line':
            self.type = type
            self.airspeed = airspeed
            self.line_origin = line_origin
            self.line_direction = line_direction/np.linalg.norm(line_direction)
            self.plot_updated=False
        if type=='orbit':
            self.type = type
            self.airspeed = airspeed
            self.orbit_center = orbit_center
            self.orbit_radius = orbit_radius
            self.orbit_direction = orbit_direction
            self.plot_updated=False
        if type=='helix':
            self.type = type
            self.airspeed = airspeed
            self.orbit_center = orbit_center
            self.orbit_radius = orbit_radius
            self.orbit_direction = orbit_direction
            self.helix_start_angle = helix_start_angle
            self.helix_climb_angle = helix_climb_angle
            self.plot_updated=False
