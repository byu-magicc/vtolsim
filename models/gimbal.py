"""
point_gimbal
    - point gimbal at target
part of mavsim
    - Beard & McLain, PUP, 2012
    - Update history:  
        3/31/2022 - RWB
        7/13/2023 - RWB
"""
import numpy as np
from tools.rotations import euler_to_rotation
import parameters.camera_parameters as CAM


class Gimbal:
    def pointAtGround(self, mav):
        az_d = 0
        el_d = np.radians(-90)
        # proportional control for gimbal
        u_az = CAM.k_az * (az_d - mav.gimbal_az)
        u_el = CAM.k_el * (el_d - mav.gimbal_el)
        return( np.array([[u_az], [u_el]]) )
            
    def pointAtPosition(self, mav, target_position):
        # line-of-sight vector in the inertial frame
        mav_position = np.array([[mav.north], [mav.east], [-mav.altitude]])
        ell_i = target_position - mav_position
        # rotate line-of-sight vector into body frame and normalize
        R_b2i = euler_to_rotation(mav.phi, mav.theta, mav.psi) # body to inertial
        ell_b = R_b2i.T @ ell_i
        ell_b = ell_b / np.linalg.norm(ell_b)
        return( self.pointAlongVector(ell_b, mav.gimbal_az, mav.gimbal_el) )

    def pointAlongVector(self, ell, azimuth, elevation):
        # point gimbal so that optical axis aligns with unit vector ell
        # ell is assumed to be aligned in the body frame
        # given current azimuth and elevation angles of the gimbal
        # compute control inputs to align gimbal
        az_d = np.arctan2(ell.item(1), ell.item(0))
        el_d = np.arctan2(-ell.item(2), np.sqrt(ell.item(0)**2 + ell.item(1)**2))
        # proportional control for gimbal
        u_az = CAM.k_az * (az_d - azimuth)
        u_el = CAM.k_el * (el_d - elevation)
        return( np.array([[u_az], [u_el]]) )




