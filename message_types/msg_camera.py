"""
msg_camera
    - messages type output of camera
    
part of mavsim
    - Beard & McLain, PUP, 2012
    - Updated:
        4/1/2022 - RWB
"""
import parameters.camera_parameters as CAM


class MsgCamera:
    '''
        Message type for receiving data from a simulated camera 
            (pixel_x, pixel_y) is the location of an object in the image plane
            (size_x, size_y) is the size of the image plane in pixels
            fov is the camera field-of-view
            focal_length is the cameras focal length 
    '''
    def __init__(self):
        self.pixel_x = int(0)
        self.pixel_y = int(0)
        self.size_x = int(CAM.pix)
        self.size_y = int(CAM.pix)
        self.fov = float(CAM.fov)
        self.focal_length = float(CAM.f)
