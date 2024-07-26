#This file implements the problem of drawing up a quadplane for the simulation

import numpy as np
import pyqtgraph.opengl as gl
from tools.rotations import euler_to_rotation


from message_types.msg_state import MsgState

import pyqtgraph.opengl as gl


class DrawQuadplane():

    #defines the initialization function
    def __init__(self, state: MsgState, window: gl.GLViewWidget, scale=1.0):


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

        ################################################################################
        #fuselage
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
        ################################################################################


        ################################################################################
        #left wing
        self.leftWingPoints, self.left_wing_indicies, self.leftWing_meshColors = self.get_left_wing_points()
        #sets the left wing location
        self.leftWingLocation = np.array([[1.0*self.unit_length], [-1.0*self.unit_length], [0.0*self.unit_length]])
        #creates the left wing object
        self.quad_leftWing = self.add_object(self.leftWingPoints, 
                                             self.left_wing_indicies, 
                                             self.leftWing_meshColors,
                                             R_bi,
                                             quad_position + R_bi @ self.leftWingLocation)
        window.addItem(self.quad_leftWing)
        ################################################################################


        ################################################################################
        #right wing
        self.rightWingPoints, self.rightWingIndicies, self.rightWing_meshColors = self.get_right_wing_points()
        #creates the right wing location
        self.rightWingLocation = np.array([[1.0*self.unit_length], [1.0*self.unit_length], [0.0*self.unit_length]])
        #creates the right wing object
        self.quad_rightWing = self.add_object(self.rightWingPoints,
                                              self.rightWingIndicies,
                                              self.rightWing_meshColors,
                                              R_bi,
                                              quad_position + R_bi @ self.rightWingLocation)
        #adds the right wing to the window
        window.addItem(self.quad_rightWing)
        ################################################################################


        ################################################################################
        #left wpar
        self.leftSparPoints, self.leftSparIndicies, self.leftSpar_meshColors = self.get_left_spar_points()
        #left spar position
        self.leftSparLocation = np.array([[5.0*self.unit_length], [-4.0*self.unit_length], [0.0*self.unit_length]])
        #creates the left spar object
        self.quad_leftSpar = self.add_object(self.leftSparPoints,
                                             self.leftSparIndicies,
                                             self.leftSpar_meshColors,
                                             R_bi,
                                             quad_position + R_bi @ self.leftSparLocation)
        #adds the item
        window.addItem(self.quad_leftSpar)
        ################################################################################


        ################################################################################
        #right spar portion
        self.rightSparPoints, self.rightSparIndicies, self.rightSpar_meshColors = self.get_right_spar_points()
        #right spar location
        self.rightSparLocation = np.array([[5.0*self.unit_length], [4.0*self.unit_length], [0.0*self.unit_length]])
        #creates the right spar object
        self.quad_rightspar = self.add_object(self.rightSparPoints,
                                              self.rightSparIndicies,
                                              self.rightSpar_meshColors,
                                              R_bi,
                                              quad_position + R_bi @ self.rightSparLocation)
        #addsthe item
        window.addItem(self.quad_rightspar)
        ################################################################################

        ################################################################################
        #left vertical stabilizer
        self.leftVerticalStabilizerPoints, self.leftVerticalStabilizerIndicies, self.leftVerticalStabilizer_meshColors = self.get_left_vertical_stabilizer_points()
        #vertical stabilizer location
        self.leftVerticalStabilizerLocation = np.array([[-8.0*self.unit_length], [-4.0*self.unit_length], [0.0*self.unit_length]])
        #creates the vertical stabilizer object
        self.quad_leftVerticalStabilizer = self.add_object(self.leftVerticalStabilizerPoints,
                                                           self.leftVerticalStabilizerIndicies,
                                                           self.leftVerticalStabilizer_meshColors,
                                                           R_bi,
                                                           quad_position + R_bi @ self.leftVerticalStabilizerLocation)

        #adds the item
        window.addItem(self.quad_leftVerticalStabilizer)
        ################################################################################

        ################################################################################
        #right vertical stabilizer
        self.rightVerticalStabilizerPoints, self.rightVerticalStabilizerIndicies, self.rightVerticalStabilizer_meshColors = self.get_right_vertical_stabilizer_points()
        #right vertical stabilizer location
        self.rightVerticalStabilizerLocation = np.array([[-8.0*self.unit_length], [4.0*self.unit_length], [0.0*self.unit_length]])
        #creates the vertical stabilizer object
        self.quad_rightVerticalStabilizer = self.add_object(self.rightVerticalStabilizerPoints,
                                                           self.rightVerticalStabilizerIndicies,
                                                           self.rightVerticalStabilizer_meshColors,
                                                           R_bi,
                                                           quad_position + R_bi @ self.rightVerticalStabilizerLocation)

        #adds the item
        window.addItem(self.quad_rightVerticalStabilizer)
        ################################################################################


        ################################################################################
        #horizontal stabilizer
        self.horizontalStabilizerPoints, self.horizontalStabilizerIndicies, self.horizontalStabilizer_meshColors = self.get_horizontal_stabilizer_points()
        #position of the horizontal stabilizer
        self.horizontalStabilizerLocation = np.array([[-8.0*self.unit_length], [0.0*self.unit_length], [0.0*self.unit_length]])
        #creates the vertical stabilizer object
        self.quad_horizontalStabilizer = self.add_object(self.horizontalStabilizerPoints,
                                                           self.horizontalStabilizerIndicies,
                                                           self.horizontalStabilizer_meshColors,
                                                           R_bi,
                                                           quad_position + R_bi @ self.horizontalStabilizerLocation)

        #adds the item
        window.addItem(self.quad_horizontalStabilizer)        
        ################################################################################

        ################################################################################
        #adding the motors section
        self.motor_points, self.motor_indicies, self.motor_meshColors = self.get_motor_points()

        #port front motor section
        self.portFrontMotorPos = np.array([[5.8], [-4.0], [-0.3]])

        self.portFrontMotor = self.add_object(self.motor_points, 
                                              self.motor_indicies,
                                              self.motor_meshColors,
                                              R_bi,
                                              quad_position + R_bi @ self.portFrontMotorPos)

        window.addItem(self.portFrontMotor)

        #port rear motor section
        self.portRearMotorPos = np.array([[-5.0], [-4.0], [-0.3]])

        self.portRearMotor = self.add_object(self.motor_points, 
                                              self.motor_indicies,
                                              self.motor_meshColors,
                                              R_bi,
                                              quad_position + R_bi @ self.portRearMotorPos)

        window.addItem(self.portRearMotor)

        #starboard rear motor section
        self.starboardRearMotorPos = np.array([[-5.0], [4.0], [-0.3]])

        self.starboardRearMotor = self.add_object(self.motor_points, 
                                              self.motor_indicies,
                                              self.motor_meshColors,
                                              R_bi,
                                              quad_position + R_bi @ self.starboardRearMotorPos)

        window.addItem(self.starboardRearMotor)

        #starboard front motor section
        self.starboardFrontMotorPos = np.array([[5.8], [4.0], [-0.3]])

        self.starboardFrontMotor = self.add_object(self.motor_points, 
                                                   self.motor_indicies,
                                                   self.motor_meshColors,
                                                   R_bi,
                                                   quad_position + R_bi @ self.starboardFrontMotorPos)

        window.addItem(self.starboardFrontMotor)

        #creates the rotation for the R rotor
        self.R_motor = euler_to_rotation(0.0, np.pi/2.0, 0.0)
        self.forwardPropulsionMotorPos = np.array([[-3.0], [0.0], [-0.5]])

        self.forwardPropulsionMotor = self.add_object(self.motor_points,
                                                      self.motor_indicies,
                                                      self.motor_meshColors,
                                                      R_bi @ self.R_motor,
                                                      quad_position + R_bi @ self.forwardPropulsionMotorPos)
        window.addItem(self.forwardPropulsionMotor)
        ################################################################################



        ################################################################################       
        self.rotor_points, self.rotor_indicies, self.rotor_meshColors = self.get_rotor_points()
        
        #creates the port front rotor
        self.portFront_rotorPosition = np.array([[5.8], [-4.0], [-0.5]])
        #adds the right rotor object
        self.portFront_rotor = self.add_object(self.rotor_points,
                                           self.rotor_indicies,
                                           self.rotor_meshColors,
                                           R_bi,
                                           quad_position + R_bi @ self.portFront_rotorPosition)
        window.addItem(self.portFront_rotor)

        #creates the port rear rotor
        self.portRear_rotorPosition = np.array([[-5.0], [-4.0], [-0.5]])

        #adds the port rear rotor object
        self.portRear_rotor = self.add_object(self.rotor_points,
                                              self.rotor_indicies,
                                              self.rotor_meshColors,
                                              R_bi,
                                              quad_position + R_bi @ self.portRear_rotorPosition)
        window.addItem(self.portRear_rotor)


        #creates the port rear rotor
        self.starboardRear_rotorPosition = np.array([[-5.0], [4.0], [-0.5]])

        #adds the port rear rotor object
        self.starboardRear_rotor = self.add_object(self.rotor_points,
                                              self.rotor_indicies,
                                              self.rotor_meshColors,
                                              R_bi,
                                              quad_position + R_bi @ self.starboardRear_rotorPosition)
        window.addItem(self.starboardRear_rotor)

        #creates the port rear rotor
        self.starboardFront_rotorPosition = np.array([[5.8], [4.0], [-0.5]])

        #adds the port rear rotor object
        self.starboardFront_rotor = self.add_object(self.rotor_points,
                                              self.rotor_indicies,
                                              self.rotor_meshColors,
                                              R_bi,
                                              quad_position + R_bi @ self.starboardFront_rotorPosition)
        window.addItem(self.starboardFront_rotor)

        window.addItem(self.starboardFront_rotor)


        #creates the propulsion rotor

        #creates the rotation for the R rotor
        self.R_rotor = euler_to_rotation(0.0, np.pi/2.0, 0.0)

        self.forwardPropulsion_rotorPosition = np.array([[-3.2], [0.0], [-0.5]])

        self.forwardPropulsion_rotor = self.add_object(self.rotor_points,
                                                       self.rotor_indicies,
                                                       self.rotor_meshColors,
                                                       R_bi @ self.R_rotor,
                                                       quad_position + R_bi @ self.forwardPropulsion_rotorPosition)
        window.addItem(self.forwardPropulsion_rotor)
        ################################################################################




    #creates the update function
    def update(self, state: MsgState):

        #gets the North, East, Down position of the aircraft
        quad_position = state.pos
        R_bi = state.R
        self.quad_fuselage = self.update_object(self.quad_fuselage,
                                            self.fuselage_points,
                                            self.fuselage_index,
                                            self.fuselage_meshColors,
                                            R_bi,
                                            quad_position)
        #updates the left wing
        self.quad_leftWing = self.update_object(self.quad_leftWing,
                                                self.leftWingPoints,
                                                self.left_wing_indicies,
                                                self.leftWing_meshColors,
                                                R_bi,
                                                quad_position + R_bi @ self.leftWingLocation)
        
        #updates the right wing
        self.quad_rightWing = self.update_object(self.quad_rightWing,
                                                 self.rightWingPoints,
                                                 self.rightWingIndicies,
                                                 self.rightWing_meshColors,
                                                 R_bi,
                                                 quad_position + R_bi @ self.rightWingLocation)
        
        #updates the left spar
        self.quad_leftSpar = self.update_object(self.quad_leftSpar,
                                                self.leftSparPoints,
                                                self.leftSparIndicies,
                                                self.leftSpar_meshColors,
                                                R_bi,
                                                quad_position + R_bi @ self.leftSparLocation)
        
        #updates the right spar
        self.quad_rightspar = self.update_object(self.quad_rightspar,
                                                 self.rightSparPoints,
                                                 self.rightSparIndicies,
                                                 self.leftSpar_meshColors,
                                                 R_bi,
                                                 quad_position + R_bi @ self.rightSparLocation)
        
        #updates the left vertical stabilizer
        self.quad_leftVerticalStabilizer = self.update_object(self.quad_leftVerticalStabilizer,
                                                              self.leftVerticalStabilizerPoints,
                                                              self.leftVerticalStabilizerIndicies,
                                                              self.leftVerticalStabilizer_meshColors,
                                                              R_bi,
                                                              quad_position + R_bi @ self.leftVerticalStabilizerLocation)
        
        #updatesthe right vertical stabilizer
        self.quad_rightVerticalStabilizer = self.update_object(self.quad_rightVerticalStabilizer,
                                                               self.rightVerticalStabilizerPoints,
                                                               self.rightVerticalStabilizerIndicies,
                                                               self.rightVerticalStabilizer_meshColors,
                                                               R_bi,
                                                               quad_position + R_bi @ self.rightVerticalStabilizerLocation)
        
        #updates the horizontal stabilizer
        self.quad_horizontalStabilizer = self.update_object(self.quad_horizontalStabilizer,
                                                            self.horizontalStabilizerPoints,
                                                            self.horizontalStabilizerIndicies,
                                                            self.horizontalStabilizer_meshColors,
                                                            R_bi,
                                                            quad_position + R_bi @ self.horizontalStabilizerLocation)
        
        #updates the front port motor
        self.portFrontMotor = self.update_object(self.portFrontMotor,
                                                 self.motor_points,
                                                 self.motor_indicies,
                                                 self.motor_meshColors,
                                                 R_bi,
                                                 quad_position + R_bi @ self.portFrontMotorPos)
        
        self.portRearMotor = self.update_object(self.portRearMotor,
                                                self.motor_points,
                                                self.motor_indicies,
                                                self.motor_meshColors,
                                                R_bi,
                                                quad_position + R_bi @ self.portRearMotorPos)
        
        self.starboardRearMotor = self.update_object(self.starboardRearMotor,
                                                     self.motor_points,
                                                     self.motor_indicies,
                                                     self.motor_meshColors,
                                                     R_bi,
                                                     quad_position + R_bi @ self.starboardRearMotorPos)

        self.starboardFrontMotor = self.update_object(self.starboardFrontMotor,
                                                      self.motor_points,
                                                      self.motor_indicies,
                                                      self.motor_meshColors,
                                                      R_bi,
                                                      quad_position + R_bi @ self.starboardFrontMotorPos)

        self.forwardPropulsionMotor = self.update_object(self.forwardPropulsionMotor,
                                                  self.motor_points,
                                                  self.motor_indicies,
                                                  self.motor_meshColors,
                                                  R_bi @ self.R_motor,
                                                  quad_position + R_bi @ self.forwardPropulsionMotorPos)


        self.portFront_rotor = self.update_object(self.portFront_rotor,
                                                  self.rotor_points,
                                                  self.rotor_indicies,
                                                  self.rotor_meshColors,
                                                  R_bi,
                                                  quad_position + R_bi @ self.portFront_rotorPosition)

        self.portRear_rotor = self.update_object(self.portRear_rotor,
                                                  self.rotor_points,
                                                  self.rotor_indicies,
                                                  self.rotor_meshColors,
                                                  R_bi,
                                                  quad_position + R_bi @ self.portRear_rotorPosition)
        
        self.starboardRear_rotor = self.update_object(self.starboardRear_rotor,
                                                  self.rotor_points,
                                                  self.rotor_indicies,
                                                  self.rotor_meshColors,
                                                  R_bi,
                                                  quad_position + R_bi @ self.starboardRear_rotorPosition)
        
        self.starboardFront_rotor = self.update_object(self.starboardFront_rotor,
                                                  self.rotor_points,
                                                  self.rotor_indicies,
                                                  self.rotor_meshColors,
                                                  R_bi,
                                                  quad_position + R_bi @ self.starboardFront_rotorPosition)

        self.forwardPropulsion_rotor = self.update_object(self.forwardPropulsion_rotor,
                                                          self.rotor_points,
                                                          self.rotor_indicies,
                                                          self.rotor_meshColors,
                                                          R_bi @ self.R_rotor,
                                                          quad_position + R_bi @ self.forwardPropulsion_rotorPosition)

    #function to add object
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

    #function to update object
    def update_object(self, object, points, index, colors, R, position):
        rotated_points = self.rotate_points(points, R)
        translated_points = self.translate_points(rotated_points, position)
        translated_points = self.R_ned @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points, index)
        object.setMeshData(vertexes=mesh, vertexColors=colors)
        return object

    #function to rotate the object
    def rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points
        return rotated_points

    #function to translate the object
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
        points = np.transpose(np.array([[self.bodyForward_x, self.bodyRight_y, self.bodyUp_z],#point 0
                                        [self.bodyForward_x, self.bodyRight_y, self.bodyDown_z],#point 1
                                        [self.bodyBack_x, self.bodyRight_y, self.bodyDown_z],#point 2
                                        [self.bodyBack_x, self.bodyRight_y, self.bodyUp_z],#point 3
                                        [self.noseLength_x, 0.0, 0.0],#point 4
                                        [self.bodyForward_x, self.bodyLeft_y, self.bodyUp_z],#point 5
                                        [self.bodyForward_x, self.bodyLeft_y, self.bodyDown_z],#point 6
                                        [self.bodyBack_x, self.bodyLeft_y, self.bodyDown_z],#point 7
                                        [self.bodyBack_x, self.bodyLeft_y, self.bodyUp_z]]))#point 8
        
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
        front_x = 0.0*ul
        #gets the back x
        back_x = -3.0*ul


        #gets the inside y
        inside_y = 0.0*ul
        #gets the outside y
        outside_y = -8.0*ul


        #top z
        top_z = -0.25*ul
        #bottom z
        bottom_z = 0.0*ul

        #creates the points
        points = np.transpose(np.array([[front_x, inside_y, top_z],#point 0
                                        [front_x, outside_y, top_z],#point 1
                                        [back_x, outside_y, top_z],#point 2
                                        [back_x, inside_y, top_z],#point 3
                                        [front_x, inside_y, bottom_z],#point 4
                                        [front_x, outside_y, bottom_z],#point 5
                                        [back_x, outside_y, bottom_z],#point 6
                                        [back_x, inside_y, bottom_z]]))#point 7
        
        #creates the meshes indicies
        indicies = np.array([[0, 1, 2],#mesh 0
                             [0, 2, 3],#mesh 1
                             [0, 1, 4],#mesh 2
                             [1, 4, 5],#mesh 3
                             [1, 2, 5],#mesh 4
                             [2, 5, 6],#mesh 5
                             [2, 3, 6],#mesh 6
                             [3, 6, 7],#mesh 7
                             [0, 3, 4],#mesh 8
                             [3, 4, 7],#mesh 9
                             [4, 5, 6],#mesh 10
                             [4, 6, 7]])#mesh 11
        
        #creates the mesh colors array
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
        meshColors[0] = self.mygrey1  #
        meshColors[1] = self.mygrey1  #
        meshColors[2] = self.mygrey1  #
        meshColors[3] = self.mygrey1  #
        meshColors[4] = self.mygrey1  #
        meshColors[5] = self.mygrey1  #
        meshColors[6] = self.mygrey1  #
        meshColors[7] = self.mygrey1  #
        meshColors[8] = self.mygrey1  #
        meshColors[9] = self.mygrey1  #
        meshColors[10] = self.mygrey1 #
        meshColors[11] = self.mygrey1 #

        return points, indicies, meshColors

    #creates the get left wing points
    def get_right_wing_points(self):
        
        #gets the unit length
        ul = self.unit_length
        #gets the front x
        front_x = 0.0*ul
        #gets the back x
        back_x = -3.0*ul

        #gets the inside y
        inside_y = 0.0*ul
        #gets the outside y
        outside_y = 8.0*ul


        #top z
        top_z = -0.25*ul
        #bottom z
        bottom_z = 0.0*ul

        #creates the points
        points = np.transpose(np.array([[front_x, inside_y, top_z],#point 0
                                        [front_x, outside_y, top_z],#point 1
                                        [back_x, outside_y, top_z],#point 2
                                        [back_x, inside_y, top_z],#point 3
                                        [front_x, inside_y, bottom_z],#point 4
                                        [front_x, outside_y, bottom_z],#point 5
                                        [back_x, outside_y, bottom_z],#point 6
                                        [back_x, inside_y, bottom_z]]))#point 7
        
        #creates the meshes indicies
        indicies = np.array([[0, 1, 2],#mesh 0
                             [0, 2, 3],#mesh 1
                             [0, 1, 4],#mesh 2
                             [1, 4, 5],#mesh 3
                             [1, 2, 5],#mesh 4
                             [2, 5, 6],#mesh 5
                             [2, 3, 6],#mesh 6
                             [3, 6, 7],#mesh 7
                             [0, 3, 4],#mesh 8
                             [3, 4, 7],#mesh 9
                             [4, 5, 6],#mesh 10
                             [4, 6, 7]])#mesh 11
        
        #creates the mesh colors array
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
        meshColors[0] = self.mygrey1  #
        meshColors[1] = self.mygrey1  #
        meshColors[2] = self.mygrey1  #
        meshColors[3] = self.mygrey1  #
        meshColors[4] = self.mygrey1  #
        meshColors[5] = self.mygrey1  #
        meshColors[6] = self.mygrey1  #
        meshColors[7] = self.mygrey1  #
        meshColors[8] = self.mygrey1  #
        meshColors[9] = self.mygrey1  #
        meshColors[10] = self.mygrey1 #
        meshColors[11] = self.mygrey1 #


        return points, indicies, meshColors

    #creates the left spar function
    def get_left_spar_points(self):
        #saves the unit length
        ul = self.unit_length
        
        #x positions
        front_x = 1.0*ul
        back_x = -15.0*ul
        #y positions
        inside_y = -0.15*ul
        outside_y = 0.15*ul

        #z positions
        top_z = -0.15*ul
        bottom_z = 0.15*ul

        #creates the points vector
        points = np.transpose(np.array([[front_x, inside_y, top_z],# point 0
                                        [front_x, outside_y, top_z],# point 1
                                        [back_x, outside_y, top_z],# point 2
                                        [back_x, inside_y, top_z],# point 3
                                        [front_x, inside_y, bottom_z],# point 4
                                        [front_x, outside_y, bottom_z],# point 5
                                        [back_x, outside_y, bottom_z],# point 6
                                        [back_x, inside_y, bottom_z]]))# point 7
        
        indicies = np.array([[0, 1, 2],
                             [0, 2, 3],
                             [0, 3, 4],
                             [3, 4, 7],
                             [4, 5, 6],
                             [4, 6, 7],
                             [1, 2, 6],
                             [1, 5, 6],
                             [0, 1, 4],
                             [1, 4, 5],
                             [2, 3, 7],
                             [2, 6, 7]])
        
        #creates the mesh colors
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
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

        #returns the points and the indicies
        return points, indicies, meshColors

    #creates the left spar function
    def get_right_spar_points(self):
        #saves the unit length
        ul = self.unit_length
        
        #x positions
        front_x = 1.0*ul
        back_x = -15.0*ul
        #y positions
        inside_y = -0.15*ul
        outside_y = 0.15*ul

        #z positions
        top_z = -0.15*ul
        bottom_z = 0.15*ul

        #creates the points vector
        points = np.transpose(np.array([[front_x, inside_y, top_z],# point 0
                                        [front_x, outside_y, top_z],# point 1
                                        [back_x, outside_y, top_z],# point 2
                                        [back_x, inside_y, top_z],# point 3
                                        [front_x, inside_y, bottom_z],# point 4
                                        [front_x, outside_y, bottom_z],# point 5
                                        [back_x, outside_y, bottom_z],# point 6
                                        [back_x, inside_y, bottom_z]]))# point 7
        
        indicies = np.array([[0, 1, 2],
                             [0, 2, 3],
                             [0, 3, 4],
                             [3, 4, 7],
                             [4, 5, 6],
                             [4, 6, 7],
                             [1, 2, 6],
                             [1, 5, 6],
                             [0, 1, 4],
                             [1, 4, 5],
                             [2, 3, 7],
                             [2, 6, 7]])
        
        #creates the mesh colors
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
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

        #returns the points and the indicies
        return points, indicies, meshColors

    #creates the left vertical stabilizer
    def get_left_vertical_stabilizer_points(self):

        #gets the unit length
        ul = self.unit_length

        #x coordinates
        forward_x = 0.0*ul
        rear_x = -2.0*ul

        #y coordinates
        inside_y = 0.12*ul
        outside_y = -0.12*ul

        #z coordinates
        bottom_z = 0.0*ul
        top_z = -2.0*ul

        #creates the points
        points = np.transpose(np.array([[forward_x, outside_y, bottom_z],#point 0
                                        [rear_x, outside_y, bottom_z],#point 1
                                        [rear_x, outside_y, top_z],#point 2
                                        [forward_x, outside_y, top_z],#point 3
                                        [forward_x, inside_y, bottom_z],#point 4
                                        [rear_x, inside_y, bottom_z],#point 5
                                        [rear_x, inside_y, top_z],#point 6
                                        [forward_x, inside_y, top_z]]))#point 7
        
        #creates the indicies
        indicies = np.array([[0, 1, 3],
                             [1, 2, 3],
                             [0, 3, 7],
                             [0, 4, 7],
                             [3, 2, 7],
                             [2, 6, 7],
                             [1, 2, 5],
                             [2, 5, 6],
                             [0, 1, 4],
                             [1, 4, 5],
                             [4, 5, 6],
                             [4, 6, 7]])
        

        #creates the meshes
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
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

        #returns the vectors
        return points, indicies, meshColors
    
    #creates the right vertical stabilizer
    def get_right_vertical_stabilizer_points(self):

        #gets the unit length
        ul = self.unit_length

        #x coordinates
        forward_x = 0.0*ul
        rear_x = -2.0*ul

        #y coordinates
        inside_y = 0.12*ul
        outside_y = -0.12*ul

        #z coordinates
        bottom_z = 0.0*ul
        top_z = -2.0*ul

        #creates the points
        points = np.array([[forward_x, outside_y, bottom_z],#point 0
                           [rear_x, outside_y, bottom_z],#point 1
                           [rear_x, outside_y, top_z],#point 2
                           [forward_x, outside_y, top_z],#point 3
                           [forward_x, inside_y, bottom_z],#point 4
                           [rear_x, inside_y, bottom_z],#point 5
                           [rear_x, inside_y, top_z],#point 6
                           [forward_x, inside_y, top_z]]).T#point 7
        
        #creates the indicies
        indicies = np.array([[0, 1, 3],
                             [1, 2, 3],
                             [0, 3, 7],
                             [0, 4, 7],
                             [3, 2, 7],
                             [2, 6, 7],
                             [1, 2, 5],
                             [2, 5, 6],
                             [0, 1, 4],
                             [1, 4, 5],
                             [4, 5, 6],
                             [4, 6, 7]])
        

        #creates the meshes
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
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

        #returns the vectors
        return points, indicies, meshColors

    #creates the horizontal stabilizer
    def get_horizontal_stabilizer_points(self):
        #gets the unit length
        ul = self.unit_length

        #saves the x values
        front_x = 0.0*ul
        back_x = -2.0*ul
        #saves the y values
        left_y = -4.0*ul
        right_y = 4.0*ul
        #saves the z values
        top_z = -0.12*ul
        bottom_z = 0.12*ul


        #creates the points
        points = np.array([[front_x, right_y, top_z],#point 0
                           [back_x, right_y, top_z],#point 1
                           [back_x, left_y, top_z],#point 2
                           [front_x, left_y, top_z],#point 3
                           [front_x, right_y, bottom_z],#point 4
                           [back_x, right_y, bottom_z],#point 5
                           [back_x, left_y, bottom_z],#point 6
                           [front_x, left_y, bottom_z]]).T#point 7
        
        #creates the indicies for the meshes
        indicies = np.array([[0, 1, 2],
                             [0, 2, 3],
                             [0, 1, 4],
                             [1, 4, 5],
                             [1, 2, 5],
                             [2, 5, 6],
                             [2, 3, 6],
                             [3, 6, 7],
                             [0, 3, 4],
                             [3, 4, 7],
                             [4, 5, 6],
                             [4, 6, 7]])
        
        #creates the mesh colors
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
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
        return points, indicies, meshColors
    

    def get_motor_points(self):
        #gets the unit length
        ul = self.unit_length

        height = 0.3*ul
        width = 0.4*ul
        length = 0.4*ul


        #defines the points
        points = np.array([[length/2.0, width/2.0, -height/2.0], #0
                           [-length/2.0, width/2.0, -height/2.0], #1
                           [-length/2.0, -width/2.0, -height/2.0], #2
                           [length/2.0, -width/2.0, -height/2.0], #3
                           [length/2.0, width/2.0, height/2.0], #4
                           [-length/2.0, width/2.0, height/2.0], #5
                           [-length/2.0, -width/2.0, height/2.0], #6
                           [length/2.0, -width/2.0, height/2.0]]).T #7
        
        indecies = np.array([[0, 1, 2],
                             [0, 2, 3],
                             [0, 1, 4],
                             [1, 4, 5],
                             [0, 3, 4],
                             [3, 4, 7],
                             [2, 3, 7],
                             [2, 6, 7],
                             [1, 2, 5],
                             [2, 5, 6],
                             [4, 5, 6],
                             [4, 6, 7]])
        
        #creates the mesh colors
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
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

        return points, indecies, meshColors


    def get_rotor_points(self):
        radius = 1.8 * self.unit_length
        N = 10
        points = np.array([[0, 0, 0]])
        theta = 0
        while theta <= 2*np.pi:
            theta += 2 * np.pi / N
            new_point = np.array([[radius*np.cos(theta), radius*np.sin(theta), 0]])
            points = np.concatenate((points, new_point), axis=0)
        index = np.array([[0, 1, 2]])
        meshColors = np.empty((points.shape[0]-1, 3, 4))
        for i in range(1, (points.shape[0]-1)):
            new_mesh = np.array([[0, i, i+1]])
            index = np.concatenate((index, new_mesh), axis=0)
            meshColors[i] = self.mygrey4
        return points.T, index, meshColors
    
