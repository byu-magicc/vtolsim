#This file implements the problem of drawing up a quadplane for the simulation

import numpy as np
import pyqtgraph.opengl as gl
from tools.rotations import euler_to_rotation



class DrawQuadplane():

    #defines the initialization function
    def __init__(self, state, window, scale=1.0):


        #saves the unit length
        self.unit_length = scale
        #gets the position of the vtol
        quad_position = state.pos
        #gets the body to inertial rotation matrix
        R_bi = state.R
        #converts to north east down for rendering
        self.R_ned = np.array([[0, 1, 0],
                               [1, 0, 0],
                               [0, 0, -1]])
        # colors
        self.mygrey1 = np.array([0.8, 0.8, 0.8, 1])  # light
        self.mygrey2 = np.array([0.6, 0.6, 0.6, 1])
        self.mygrey3 = np.array([0.5, 0.5, 0.5, 1])
        self.mygrey4 = np.array([0.3, 0.3, 0.3, 1])  # dark

        #gets the points, indecies, and colors from the helper function
        self.fuselage_points, self.fuselage_index, self.fuselage_meshColors = self.get_body_points()       




    #defines the get body points function
    def get_body_points(self):
        #saves the unit length in shorthand
        ul = self.unit_length
        #gets the forward length
        self.bodyForward_x = 3.0*ul
        self.bodyBack_x = -3.0*ul
        #the y dimensions
        self.bodyRight_y = 1.0*ul
        self.bodyLeft_y = -1.0*ul
        #the z dimensions
        self.bodyDown_z = 1.0*ul
        self.bodyUp_z = -1.0*ul
        #nose specifications
        self.noseLength_x = 4.0*ul

        #creates the point vector
        points = np.array([[self.bodyForward_x, self.bodyRight_y, self.bodyUp_z],#point 0
                           [self.bodyForward_x, self.bodyRight_y, self.bodyDown_z],#point 1
                           [self.bodyBack_x, self.bodyRight_y, self.bodyDown_z],#point 2
                           [self.bodyBack_x, self.bodyRight_y, self.bodyUp_z],#point 3
                           [self.noseLength_x, 0.0, 0.0],#point 4
                           [self.bodyForward_x, self.bodyLeft_y, self.bodyUp_z],#point 5
                           [self.bodyForward_x, self.bodyLeft_y, self.bodyDown_z],#point 6
                           [self.bodyBack_x, self.bodyLeft_y, self.bodyDown_z],#point 7
                           [self.bodyBack_x, self.bodyLeft_y, self.bodyUp_z]])#point 8
        
        #creates the indecies for the meshes
        indecies = np.array([[0, 1, 2],
                             [0, 2, 3],
                             [0, 3, 5],
                             [3, 5, 8],
                             [5, 6, 8],
                             [6, 7, 8],
                             [1, 6, 7],
                             [1, 2, 7],
                             [2, 3, 7],
                             [3, 7, 8],
                             ])