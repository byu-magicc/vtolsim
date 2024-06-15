#function to put together a three dimensional drawing of the quadplane

import numpy as np
import pyqtgraph.opengl as gl
from tools.rotations import euler_to_rotation

#defines the draw quadplane class
class DrawQuadplane():

    #initialization function
    def __init__(self, state, window, scale=1.0):

        self.unit_length = scale
        #creates the coordinates
        vtol_position = state.pos
        #boty to inertial rotation
        R_bi = state.R
        #convert to north east down for rendering
        self.R_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        # colors
        self.mygrey1 = np.array([0.8, 0.8, 0.8, 1])  # light
        self.mygrey2 = np.array([0.6, 0.6, 0.6, 1])
        self.mygrey3 = np.array([0.5, 0.5, 0.5, 1])
        self.mygrey4 = np.array([0.3, 0.3, 0.3, 1])  # dark




    #creates function to get the fuselage points 
    def get_fuselage_points(self):
        #defines the points of the Quadplane fuselage
        #gets the unit length shorthand
        ul = self.unit_length

        #creates a bunch of helper variables to make the drawing cleaner
        x_bodyForward = 3.0*ul
        x_bodyBackward = -3.0*ul


        y_bodySide = 1.0*ul

        #creates the points array
        points = np.array([[x_bodyForward, y_bodySide, -1.0],#0
                           [x_bodyForward, y_bodySide, 1.0],#1
                           [x_bodyBackward, y_bodySide, 1.0],#2
                           [x_bodyBackward, y_bodySide, -1.0],#3
                           [4.0, 0.0, 0.0],#4
                           [x_bodyForward, -y_bodySide, -1.0],#5
                           [x_bodyForward, -y_bodySide, 1.0],#6
                           [x_bodyBackward, -y_bodySide, 1.0],#7
                           [x_bodyBackward, -y_bodySide, -1.0]])#8
        

        #creates the index array, which defines the meshes
        index = np.array([[0, 1, 2],
                          ])
