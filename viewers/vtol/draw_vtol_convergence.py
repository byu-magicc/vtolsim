"""
class to draw vtol that looks like convergence RC aircraft
    - Beard & McLain, PUP, 2012
    - Update history:
        4/1/2019 - Randy Beard
        4/15/2019 - BGM
        5/3/2019 - Randy Beard
"""

import numpy as np
import pyqtgraph.opengl as gl
from tools.rotations import euler_to_rotation

class drawVtol():
    def __init__(self, state, window):
        """
        Draw the MAV.

        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.pn  # north position
            state.pe  # east position
            state.h   # altitude
            state.phi  # roll angle
            state.theta  # pitch angle
            state.psi  # yaw angle
        """
        self.unit_length = .1
        # get points that define the non-rotated, non-translated vtol and the mesh colors
        self.vtol_points, self.vtol_meshColors = self.get_fuselage_points()
        self.motor_points, self.motor_meshColors = self.get_motor_points()
        self.rotor_points, self.rotor_colors = self.get_rotor_points()
        vtol_position = np.array([[state.pn], [state.pe], [-state.h]])  # NED coordinates
        # attitude of vtol as a rotation matrix R from body to inertial
        R_bi = euler_to_rotation(state.phi, state.theta, state.psi)
        # convert North-East Down to East-North-Up for rendering
        R_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        # fuselage and wings
        rotated_body = self.rotate_points(self.vtol_points, R_bi)
        translated_body = self.translate_points(rotated_body, vtol_position)
        translated_body = R_ned @ translated_body
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        body_mesh = self.fuselage_points_to_mesh(translated_body)
        self.vtol_body = gl.GLMeshItem(vertexes=body_mesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=self.vtol_meshColors,  # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=False,  # speeds up rendering
                                      computeNormals=False)  # speeds up rendering
        window.addItem(self.vtol_body)  # add body to plot

        # right motor
        self.rmotor_pos = np.array([[-2.5 * self.unit_length],
                                    [2.0 * self.unit_length],
                                    [0]])
        R_rmotor = euler_to_rotation(0.0, state.right_rotor, 0.0)
        rotated_rmotor = self.rotate_points(self.motor_points, R_bi @ R_rmotor)
        rmotor_position = vtol_position + R_bi @ self.rmotor_pos
        translated_rmotor = self.translate_points(rotated_rmotor, rmotor_position)
        translated_rmotor = R_ned @ translated_rmotor
        rmotor_mesh = self.motor_points_to_mesh(translated_rmotor)
        self.vtol_rmotor = gl.GLMeshItem(vertexes=rmotor_mesh,
                                         vertexColors=self.motor_meshColors,
                                         drawEdges=True,
                                         smooth=False,
                                         computeNormals=False)
        window.addItem(self.vtol_rmotor)

        # left motor
        self.lmotor_pos = np.array([[-2.5 * self.unit_length],
                                    [-2.0 * self.unit_length],
                                    [0]])
        R_lmotor = euler_to_rotation(0.0, state.left_rotor, 0.0)
        rotated_lmotor = self.rotate_points(self.motor_points, R_bi @ R_lmotor)
        lmotor_position = vtol_position + R_bi @ self.lmotor_pos
        translated_lmotor = self.translate_points(rotated_lmotor, lmotor_position)
        translated_lmotor = R_ned @ translated_lmotor
        lmotor_mesh = self.motor_points_to_mesh(translated_lmotor)
        self.vtol_lmotor = gl.GLMeshItem(vertexes=lmotor_mesh,
                                         vertexColors=self.motor_meshColors,
                                         drawEdges=True,
                                         smooth=False,
                                         computeNormals=False)
        window.addItem(self.vtol_lmotor)

        # back rotor
        self.back_rotor_pos = np.array([[-4.5 * self.unit_length], [0], [0]])
        rotated_back_rotor = self.rotate_points(self.rotor_points, R_bi)
        back_rotor_position = vtol_position + R_bi @ self.back_rotor_pos
        translated_back_rotor = self.translate_points(rotated_back_rotor, back_rotor_position)
        translated_back_rotor = R_ned @ translated_back_rotor
        self.back_rotor =  gl.GLLinePlotItem(pos=translated_back_rotor.T,
                                             color=self.rotor_colors,
                                             width=2,
                                             antialias=True,
                                             mode='line_strip')
        window.addItem(self.back_rotor)

        # right rotor
        self.rotor_pos = np.array([[1.1 * self.unit_length],
                                   [0],
                                   [0]]) # with respect to motor
        self.R_rotor = euler_to_rotation(0.0, np.pi / 2, 0.0)
        rotated_right_rotor = self.rotate_points(self.rotor_points, R_bi @ R_rmotor @ self.R_rotor)
        right_rotor_position = vtol_position \
                               + R_bi @ self.rmotor_pos \
                               + R_bi @ R_rmotor @ self.rotor_pos
        translated_right_rotor = self.translate_points(rotated_right_rotor, right_rotor_position)
        translated_right_rotor = R_ned @ translated_right_rotor
        self.right_rotor =  gl.GLLinePlotItem(pos=translated_right_rotor.T,
                                              color=self.rotor_colors,
                                              width=2,
                                              antialias=True,
                                              mode='line_strip')
        window.addItem(self.right_rotor)

        # left rotor
        rotated_left_rotor = self.rotate_points(self.rotor_points, R_bi @ R_lmotor @ self.R_rotor)
        left_rotor_position = vtol_position \
                               + R_bi @ self.rmotor_pos \
                               + R_bi @ R_lmotor @ self.rotor_pos
        translated_left_rotor = self.translate_points(rotated_left_rotor, left_rotor_position)
        translated_left_rotor = R_ned @ translated_left_rotor
        self.left_rotor =  gl.GLLinePlotItem(pos=translated_left_rotor.T,
                                             color=self.rotor_colors,
                                             width=2,
                                             antialias=True,
                                             mode='line_strip')
        window.addItem(self.left_rotor)

    def update(self, state):
        # NED coordinates of vtol
        vtol_position = np.array([[state.pn], [state.pe], [-state.h]])
        # attitude of vtol as a rotation matrix R from body to inertial
        R_bi = euler_to_rotation(state.phi, state.theta, state.psi)
        # convert North-East Down to East-North-Up for rendering
        R_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        # fuselage and wings
        rotated_body = self.rotate_points(self.vtol_points, R_bi)
        translated_body = self.translate_points(rotated_body, vtol_position)
        translated_body = R_ned @ translated_body
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        body_mesh = self.fuselage_points_to_mesh(translated_body)
        # draw VTOL by resetting mesh using rotated and translated points
        self.vtol_body.setMeshData(vertexes=body_mesh, vertexColors=self.vtol_meshColors)

        # right motor
        R_rmotor =  euler_to_rotation(0.0, state.right_rotor, 0.0)
        rotated_rmotor = self.rotate_points(self.motor_points, R_bi @ R_rmotor)
        rmotor_position = vtol_position + R_bi @ self.rmotor_pos
        translated_rmotor = self.translate_points(rotated_rmotor, rmotor_position)
        translated_rmotor = R_ned @ translated_rmotor
        rmotor_mesh = self.motor_points_to_mesh(translated_rmotor)
        self.vtol_rmotor.setMeshData(vertexes=rmotor_mesh,
                                     vertexColors=self.motor_meshColors)
        # left motor
        R_lmotor = euler_to_rotation(0.0, state.left_rotor, 0.0)
        rotated_lmotor = self.rotate_points(self.motor_points, R_bi @ R_lmotor)
        lmotor_position = vtol_position + R_bi @ self.lmotor_pos
        translated_lmotor = self.translate_points(rotated_lmotor, lmotor_position)
        translated_lmotor = R_ned @ translated_lmotor
        lmotor_mesh = self.motor_points_to_mesh(translated_lmotor)
        self.vtol_lmotor.setMeshData(vertexes=lmotor_mesh,
                                     vertexColors=self.motor_meshColors)

        # back rotor
        rotated_back_rotor = self.rotate_points(self.rotor_points, R_bi)
        back_rotor_position = vtol_position + R_bi @ self.back_rotor_pos
        translated_back_rotor = self.translate_points(rotated_back_rotor, back_rotor_position)
        translated_back_rotor = R_ned @ translated_back_rotor
        self.back_rotor.setData(pos=translated_back_rotor.T)

        # right rotor
        rotated_right_rotor = self.rotate_points(self.rotor_points, R_bi @ R_rmotor @ self.R_rotor)
        right_rotor_position = vtol_position \
                               + R_bi @ self.rmotor_pos \
                               + R_bi @ R_rmotor @ self.rotor_pos
        translated_right_rotor = self.translate_points(rotated_right_rotor, right_rotor_position)
        translated_right_rotor = R_ned @ translated_right_rotor
        self.right_rotor.setData(pos=translated_right_rotor.T)

        # left rotor
        rotated_left_rotor = self.rotate_points(self.rotor_points, R_bi @ R_lmotor @ self.R_rotor)
        left_rotor_position = vtol_position \
                               + R_bi @ self.lmotor_pos \
                               + R_bi @ R_lmotor @ self.rotor_pos
        translated_left_rotor = self.translate_points(rotated_left_rotor, left_rotor_position)
        translated_left_rotor = R_ned @ translated_left_rotor
        self.left_rotor.setData(pos=translated_left_rotor.T)

    def rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points
        return rotated_points

    def translate_points(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(translation, np.ones([1, points.shape[1]]))
        return translated_points

    def get_fuselage_points(self):
        """"
            Points that define the vtol, and the colors of the triangular mesh
            Define the points on the aircraft following diagram in Figure C.3
        """
        # define MAV body parameters
        fuse_h = 1.0 * self.unit_length
        fuse_w = 1.0 * self.unit_length
        fuse_l1 = 3.0 * self.unit_length
        fuse_l2 = 1.0 * self.unit_length
        fuse_l3 = -3.0 * self.unit_length
        wing_l1 = 1.0 * self.unit_length
        wing_l2 = -4.0 * self.unit_length
        wing_l3 = -5.0 * self.unit_length
        wing_l4 = -3.5 * self.unit_length
        wing_w1 = 6.0 * self.unit_length
        wing_w2 = 2.0 * self.unit_length
        winglet_d = 1.0 * self.unit_length
        tail_h = 2.0 * self.unit_length
        tail_l = -2.0 * self.unit_length
        tail_pos_l = -4.5 * self.unit_length
        tail_pos_w = 1.5 * self.unit_length

        # points are in NED coordinates
        #   define the points on the aircraft following diagram Fig 2.14
        points = np.array([[fuse_l1, 0, 0],  # point [0] - nose-tip
                           [fuse_l2, fuse_w / 2.0, -fuse_h / 2.0],  # point [1] - nose cone
                           [fuse_l2, -fuse_w / 2.0, -fuse_h / 2.0],  # point [2] - nose cone
                           [fuse_l2, -fuse_w / 2.0, fuse_h / 2.0],  # point [3] - nose cone
                           [fuse_l2, fuse_w / 2.0, fuse_h / 2.0],  # point [4] - nose cone
                           [fuse_l3, 0, 0],  # point [5] - end of fuselage
                           [wing_l1, 0, 0],  # point [6] wing
                           [wing_l2, -wing_w1/2, 0], # point [7] wing
                           [wing_l3, -wing_w1/2, 0],  # point [8] wing
                           [wing_l3, -wing_w2/2, 0],  # point [9] wing
                           [(wing_l4+wing_l3)/2, -wing_w2/2, 0],  # point [10] wing
                           [wing_l4, -wing_w2/6, 0],  # point [11] wing
                           [wing_l4, wing_w2/6, 0],  # point [12] wing
                           [(wing_l4+wing_l3)/2, wing_w2/2, 0],  # point [13] wing
                           [wing_l3, wing_w2/2, 0],  # point [14] wing
                           [wing_l3, wing_w1/2, 0],  # point [15] wing
                           [wing_l2, wing_w1/2, 0], # point [16] wing
                           [(wing_l2+wing_l3)/2, -wing_w1/2, winglet_d], # point [17] left winglet
                           [wing_l3, -wing_w1/2, winglet_d],  # point [18] left winglet
                           [(wing_l2+wing_l3)/2, wing_w1/2, winglet_d], # point [19] right winglet
                           [wing_l3, wing_w1/2, winglet_d],  # point [20] right winglet
                           [tail_pos_l + tail_l, -tail_pos_w, 0],  # point [21] left tail
                           [tail_pos_l + tail_l, -tail_pos_w, -tail_h],  # point [22] left tail
                           [tail_pos_l + tail_l/2, -tail_pos_w, -tail_h],  # point [23] left tail
                           [tail_pos_l, -tail_pos_w, 0],  # point [24] left tail
                           [tail_pos_l + tail_l, tail_pos_w, 0],  # point [25] right tail
                           [tail_pos_l + tail_l, tail_pos_w, -tail_h],  # point [26] right tail
                           [tail_pos_l + tail_l/2, tail_pos_w, -tail_h],  # point [27] right tail
                           [tail_pos_l, tail_pos_w, 0],  # point [28] right tail
                           ]).T

        #   define the colors for each face of triangular mesh
        #red = np.array([1., 0., 0., 1])
        red = np.array([211, 68, 63, 256])/256
        #green = np.array([0., 1., 0., 1])
        green = np.array([63, 211, 105, 256])/256.
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        meshColors = np.empty((25, 3, 4), dtype=np.float32)
        meshColors[0] = blue  # nose-top
        meshColors[1] = blue  # nose-right
        meshColors[2] = red  # nose-bottom
        meshColors[3] = blue  # nose-left
        meshColors[4] = blue  # fuselage-left
        meshColors[5] = blue  # fuselage-top
        meshColors[6] = blue  # fuselage-right
        meshColors[7] = red  # fuselage-bottom
        meshColors[8] = green  # wing
        meshColors[9] = green  # wing
        meshColors[10] = green  # wing
        meshColors[11] = green  # wing
        meshColors[12] = green  # wing
        meshColors[13] = green  # wing
        meshColors[14] = green  # wing
        meshColors[15] = green  # wing
        meshColors[16] = green  # wing
        meshColors[17] = green  # winglet
        meshColors[18] = green  # winglet
        meshColors[19] = green  # winglet
        meshColors[20] = green  # winglet
        meshColors[21] = blue  # tail
        meshColors[22] = blue  # tail
        meshColors[23] = blue  # tail
        meshColors[24] = blue  # tail
        return points, meshColors

    def fuselage_points_to_mesh(self, points):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        points = points.T
        mesh = np.array([[points[0], points[1], points[2]],  # nose-top
                         [points[0], points[1], points[4]],  # nose-right
                         [points[0], points[3], points[4]],  # nose-bottom
                         [points[0], points[3], points[2]],  # nose-left
                         [points[5], points[2], points[3]],  # fuselage-left
                         [points[5], points[1], points[2]],  # fuselage-top
                         [points[5], points[1], points[4]],  # fuselage-right
                         [points[5], points[3], points[4]],  # fuselage-bottom
                         [points[6], points[7], points[11]],  # wing
                         [points[7], points[11], points[10]],  # wing
                         [points[7], points[10], points[9]],  # wing
                         [points[7], points[8], points[9]],  # wing
                         [points[6], points[11], points[12]],  # wing
                         [points[6], points[12], points[16]],  # wing
                         [points[12], points[13], points[16]],  # wing
                         [points[13], points[16], points[14]],  # wing
                         [points[14], points[15], points[16]],  # wing
                         [points[7], points[8], points[17]],  # winglet
                         [points[17], points[18], points[8]],  # winglet
                         [points[15], points[16], points[19]],  # winglet
                         [points[15], points[19], points[20]],  # winglet
                         [points[21], points[22], points[23]],  # tail
                         [points[21], points[23], points[24]],  # tail
                         [points[25], points[26], points[27]],  # tail
                         [points[25], points[27], points[28]],  # tail
                         ])
        return mesh

    def get_motor_points(self):
        """"
            Points that define the motor
        """
        # define MAV body parameters
        height = 0.5 * self.unit_length
        width = 0.5 * self.unit_length
        length = 1.0 * self.unit_length

        # points are in NED coordinates
        #   define the points on the aircraft following diagram Fig 2.14
        points = np.array([[0, width/2, height/2],  # [0]
                           [0, width/2, -height/2],  # [1]
                           [0, -width/2, -height/2],  # [2]
                           [0, -width/2, height/2],  # [3]
                           [length, width/2, height/2],  # [4]
                           [length, width/2, -height/2],  # [5]
                           [length, -width/2, -height/2],  # [6]
                           [length, -width/2, height/2],  # [7]
                           ]).T

        #   define the colors for each face of triangular mesh
        #red = np.array([1., 0., 0., 1])
        red = np.array([211, 68, 63, 256])/256
        green = np.array([0., 1., 0., 1])
        blue = np.array([66, 161, 244, 256])/256.
        yellow = np.array([1., 1., 0., 1])
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
        meshColors[0] = blue  # end
        meshColors[1] = blue  # end
        meshColors[2] = blue # top
        meshColors[3] = blue  # top
        meshColors[4] = blue  # right
        meshColors[5] = blue  # right
        meshColors[6] = red  # bottom
        meshColors[7] = red  # bottom
        meshColors[8] = blue  # left
        meshColors[9] = blue  # left
        meshColors[10] = red  # end
        meshColors[11] = red  # end
        return points, meshColors

    def motor_points_to_mesh(self, points):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        points = points.T
        mesh = np.array([[points[0], points[1], points[2]],  # end
                         [points[0], points[3], points[2]],  # end
                         [points[1], points[5], points[6]],  # top
                         [points[1], points[2], points[6]],  # top
                         [points[0], points[1], points[5]],  # right
                         [points[0], points[4], points[5]],  # right
                         [points[0], points[4], points[7]],  # bottom
                         [points[0], points[3], points[7]],  # bottom
                         [points[3], points[7], points[6]],  # left
                         [points[3], points[2], points[6]],  # left
                         [points[4], points[5], points[6]],  # end
                         [points[4], points[7], points[6]],  # end
                         ])
        return mesh

    def get_rotor_points(self):
        radius = 0.6 * self.unit_length
        N = 10
        theta = 0
        theta_list = [theta]
        while theta < 2*np.pi:
            theta += 0.1
            theta_list.append(theta)
        points = np.array([[radius, 0, 0]])
        for angle in theta_list:
            new_point = np.array([[radius * np.cos(angle),
                                   radius * np.sin(angle),
                                   0]])
            points = np.concatenate((points, new_point), axis=0)
        color = np.array([1., 1., 0., 1])
        path_color = np.tile(color, (points.shape[0], 1))
        return points.T, path_color

