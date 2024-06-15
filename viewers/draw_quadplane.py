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

        #creates the fuselage object 
        self.quad_fuselage = self.add_object(self.fuselage_points,
                                             self.fuselage_index,
                                             self.fuselage_meshColors,
                                             R_bi,
                                             quad_position)
        #adds the item to the window
        window.addItem(self.quad_fuselage)


    def add_object(self, points, index, colors, R, position):
        rotated_points = self.rotate_points(points, R)
        translated_points = self.translate_points(rotated_points, position)
        translated_points = self.R_ned @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points, index)
        object = gl.GLMeshItem(
            vertexes=mesh,  # defines the triangular mesh (Nx3x3)
            vertexColors=colors,  # defines mesh colors (Nx1)
            drawEdges=True,  # draw edges between mesh elements
            smooth=False,  # speeds up rendering
            computeNormals=False)  # speeds up rendering
        return object

    def update_object(self, object, points, index, colors, R, position):
        rotated_points = self.rotate_points(points, R)
        translated_points = self.translate_points(rotated_points, position)
        translated_points = self.R_ned @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points, index)
        object.setMeshData(vertexes=mesh, vertexColors=colors)
        return object

    def rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points
        return rotated_points

    def translate_points(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(translation, np.ones([1, points.shape[1]]))
        return translated_points

    def points_to_mesh(self, points, index):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        points = points.T
        mesh = np.array([[points[index[0,0]],points[index[0,1]],points[index[0,2]]]])
        for i in range(1, index.shape[0]):
            tmp = np.array([[points[index[i,0]], points[index[i,1]], points[index[i,2]]]])
            mesh = np.concatenate((mesh, tmp), axis=0)
        return mesh

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
        
        #creates the indicies for the meshes
        indicies = np.array([[0, 1, 2],
                             [0, 2, 3],
                             [0, 3, 5],
                             [3, 5, 8],
                             [5, 6, 8],
                             [6, 7, 8],
                             [1, 6, 7],
                             [1, 2, 7],
                             [2, 3, 7],
                             [3, 7, 8],
                             [0, 1, 4],
                             [0, 4, 5],
                             [1, 4, 6],
                             [4, 5, 6]])
        
        #creates the mesh colors array
        #   define the colors for each face of triangular mesh
        meshColors = np.empty((22, 3, 4), dtype=np.float32)
        meshColors[0] = self.mygrey1  # top
        meshColors[1] = self.mygrey1  # top
        meshColors[2] = self.mygrey1  # top
        meshColors[3] = self.mygrey1  # top
        meshColors[4] = self.mygrey1  # right side
        meshColors[5] = self.mygrey1  # right side
        meshColors[6] = self.mygrey1  # right side
        meshColors[7] = self.mygrey1  # right side
        meshColors[8] = self.mygrey1  # left side
        meshColors[9] = self.mygrey1  # left side
        meshColors[10] = self.mygrey1  # left side
        meshColors[11] = self.mygrey1  # left side
        meshColors[12] = self.mygrey3  # back
        meshColors[13] = self.mygrey3  # back

        #returns the points, the indicies, and the mesh colors
        return points, indicies, meshColors
    

    #creates the get left wing points
    def get_left_wing_points(self):
        
        #gets the unit length
        ul = self.unit_length
        #gets the front x
        front_x = 1.0*ul
        #gets the back x
        back_x = -2.0*ul
        #gets the aileron pocket x
        aileron_pocket_x = -1.0*ul

        #gets the inside y
        inside_y = -1.0*ul
        #gets the outside y
        outside_y = -8.0*ul
        #inside aileron y
        inside_aileron_y = -4.0*ul
        #outside aileron y
        outside_aileron_y = -7.0*ul

        #top z
        top_z = -1.0*ul
        #bottom z
        bottom_z = 1.0*ul

        #creates the points
        points = np.array([[front_x, inside_y, top_z],#point 0
                           [front_x, inside_aileron_y, top_z],#point 1
                           [front_x, outside_aileron_y, top_z],#point 2
                           [front_x, outside_y, top_z],#point 3
                           [back_x, outside_y, top_z],#point 4
                           [back_x, outside_aileron_y, top_z],#point 5
                           [aileron_pocket_x, outside_aileron_y, top_z],#point 6
                           [aileron_pocket_x, inside_aileron_y, top_z],#point 7
                           [back_x, inside_aileron_y, top_z],#point 8
                           [back_x, inside_y, top_z],#point 9
                           [front_x, inside_y, bottom_z],#point 10
                           [front_x, inside_aileron_y, bottom_z], #point 11
                           [front_x, outside_aileron_y, bottom_z], #point 12
                           [front_x, outside_y, bottom_z],#point 13
                           [back_x, outside_y, bottom_z],#point 14
                           [back_x, outside_aileron_y, bottom_z],#point 15
                           [aileron_pocket_x, outside_aileron_y, bottom_z],#point 16
                           [aileron_pocket_x, inside_aileron_y, bottom_z],#point 17
                           [back_x, inside_aileron_y, bottom_z],#point 18
                           [back_x, inside_y, bottom_z]])#point 19
        
        #creates the meshes indicies
        indicies = np.array([[0, 8, 9],
                             [0, 1, 8],
                             [1, 2, 6],
                             [1, 6, 7],
                             [2, 3, 4],
                             [2, 4, 5],
                             [10, 18, 19],
                             [10, 11, 18],
                             [11, 12, 16],
                             [11, 16, 17],
                             [12, 13, 14],
                             [12, 14, 15],
                             [0, 9, 10],
                             [9, 10, 19],
                             [9, 8, 18],
                             [9, 18, 19],
                             [8, 7, 17],
                             [8, 17, 18],
                             [7, 6, 16],
                             [7, 16, 17],
                             [6, 5, 16],
                             [5, 15, 16],
                             [4, 5, 14],
                             [5, 14, 15],
                             [3, 4, 13],
                             [4, 13, 14],
                             [0, 3, 13],
                             [0, 10, 13]])
        
        #creates the mesh colors array
        meshColors = np.empty((18, 3, 4), dtype=np.float32)
        meshColors[0] = self.mygrey1  # top
        meshColors[1] = self.mygrey1  # top
        meshColors[2] = self.mygrey1  # top
        meshColors[3] = self.mygrey1  # top
        meshColors[4] = self.mygrey1  # right side
        meshColors[5] = self.mygrey1  # right side
        meshColors[6] = self.mygrey1  # right side
        meshColors[7] = self.mygrey1  # right side
        meshColors[8] = self.mygrey1  # left side
        meshColors[9] = self.mygrey1  # left side
        meshColors[10] = self.mygrey1  # left side
        meshColors[11] = self.mygrey1  # left side
        meshColors[12] = self.mygrey3  # back
        meshColors[13] = self.mygrey3  # back
        meshColors[14] = self.mygrey2  # front top
        meshColors[15] = self.mygrey2  # front top
        meshColors[16] = self.mygrey2  # front middle
        meshColors[17] = self.mygrey2  # front middle
        meshColors[18] = self.mygrey4  # bottom
        meshColors[19] = self.mygrey4  # bottom
        meshColors[20] = self.mygrey4  # bottom
        meshColors[21] = self.mygrey4  # bottom
        meshColors[22] = self.mygrey2  # front top
        meshColors[23] = self.mygrey2  # front middle
        meshColors[24] = self.mygrey2  # front middle
        meshColors[25] = self.mygrey4  # bottom
        meshColors[26] = self.mygrey4  # bottom
        meshColors[27] = self.mygrey4  # bottom

        return points, indicies, meshColors


    #creates the get left wing points
    def get_right_wing_points(self):
        
        #gets the unit length
        ul = self.unit_length
        #gets the front x
        front_x = 1.0*ul
        #gets the back x
        back_x = -2.0*ul
        #gets the aileron pocket x
        aileron_pocket_x = -1.0*ul

        #gets the inside y
        inside_y = 1.0*ul
        #gets the outside y
        outside_y = 8.0*ul
        #inside aileron y
        inside_aileron_y = 4.0*ul
        #outside aileron y
        outside_aileron_y = 7.0*ul

        #top z
        top_z = -1.0*ul
        #bottom z
        bottom_z = 1.0*ul

        #creates the points
        points = np.array([[front_x, inside_y, top_z],#point 0
                           [front_x, inside_aileron_y, top_z],#point 1
                           [front_x, outside_aileron_y, top_z],#point 2
                           [front_x, outside_y, top_z],#point 3
                           [back_x, outside_y, top_z],#point 4
                           [back_x, outside_aileron_y, top_z],#point 5
                           [aileron_pocket_x, outside_aileron_y, top_z],#point 6
                           [aileron_pocket_x, inside_aileron_y, top_z],#point 7
                           [back_x, inside_aileron_y, top_z],#point 8
                           [back_x, inside_y, top_z],#point 9
                           [front_x, inside_y, bottom_z],#point 10
                           [front_x, inside_aileron_y, bottom_z], #point 11
                           [front_x, outside_aileron_y, bottom_z], #point 12
                           [front_x, outside_y, bottom_z],#point 13
                           [back_x, outside_y, bottom_z],#point 14
                           [back_x, outside_aileron_y, bottom_z],#point 15
                           [aileron_pocket_x, outside_aileron_y, bottom_z],#point 16
                           [aileron_pocket_x, inside_aileron_y, bottom_z],#point 17
                           [back_x, inside_aileron_y, bottom_z],#point 18
                           [back_x, inside_y, bottom_z]])#point 19
        
        #creates the meshes indicies
        indicies = np.array([[0, 8, 9],
                             [0, 1, 8],
                             [1, 2, 6],
                             [1, 6, 7],
                             [2, 3, 4],
                             [2, 4, 5],
                             [10, 18, 19],
                             [10, 11, 18],
                             [11, 12, 16],
                             [11, 16, 17],
                             [12, 13, 14],
                             [12, 14, 15],
                             [0, 9, 10],
                             [9, 10, 19],
                             [9, 8, 18],
                             [9, 18, 19],
                             [8, 7, 17],
                             [8, 17, 18],
                             [7, 6, 16],
                             [7, 16, 17],
                             [6, 5, 16],
                             [5, 15, 16],
                             [4, 5, 14],
                             [5, 14, 15],
                             [3, 4, 13],
                             [4, 13, 14],
                             [0, 3, 13],
                             [0, 10, 13]])
        
        #creates the mesh colors array
        meshColors = np.empty((18, 3, 4), dtype=np.float32)
        meshColors[0] = self.mygrey1  # top
        meshColors[1] = self.mygrey1  # top
        meshColors[2] = self.mygrey1  # top
        meshColors[3] = self.mygrey1  # top
        meshColors[4] = self.mygrey1  # right side
        meshColors[5] = self.mygrey1  # right side
        meshColors[6] = self.mygrey1  # right side
        meshColors[7] = self.mygrey1  # right side
        meshColors[8] = self.mygrey1  # left side
        meshColors[9] = self.mygrey1  # left side
        meshColors[10] = self.mygrey1  # left side
        meshColors[11] = self.mygrey1  # left side
        meshColors[12] = self.mygrey3  # back
        meshColors[13] = self.mygrey3  # back
        meshColors[14] = self.mygrey2  # front top
        meshColors[15] = self.mygrey2  # front top
        meshColors[16] = self.mygrey2  # front middle
        meshColors[17] = self.mygrey2  # front middle
        meshColors[18] = self.mygrey4  # bottom
        meshColors[19] = self.mygrey4  # bottom
        meshColors[20] = self.mygrey4  # bottom
        meshColors[21] = self.mygrey4  # bottom
        meshColors[22] = self.mygrey2  # front top
        meshColors[23] = self.mygrey2  # front middle
        meshColors[24] = self.mygrey2  # front middle
        meshColors[25] = self.mygrey4  # bottom
        meshColors[26] = self.mygrey4  # bottom
        meshColors[27] = self.mygrey4  # bottom


        return points, indicies, meshColors

