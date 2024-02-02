"""
    - Update history:
        4/1/2019 - Randy Beard
        4/15/2019 - BGM
        5/3/2019 - Randy Beard
        11/16/2023 - R. Beard
"""
import numpy as np
import pyqtgraph.opengl as gl
from tools.rotations import euler_to_rotation


class DrawVtol():
    def __init__(self, state, window, scale=1):
        """
        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.p  # inertial position (NED)
            state.R  # rotation R_b^i
        """
        self.unit_length = scale
        vtol_position = state.pos  # NED coordinates
        R_bi = state.R  # body to inertial rotation
        # convert North-East Down to East-North-Up for rendering
        self.R_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        # body
        self.body_points, self.body_index, self.body_meshColors = self.get_body_points()
        self.vtol_body = self.add_object(
            self.body_points,
            self.body_index,
            self.body_meshColors,
            R_bi,
            vtol_position)
        window.addItem(self.vtol_body)  # add fuselage/wing to drawing
        # motors
        self.motor_points, self.motor_index, self.motor_meshColors = self.get_motor_points()
        # right motor
        self.right_motor_pos = np.array([[-2.5 * self.unit_length],
                                         [2.0 * self.unit_length],
                                         [0]])
        R_right_motor = euler_to_rotation(0.0, state.rotor_angle_right, 0.0)
        self.right_motor = self.add_object(
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ R_right_motor,
            vtol_position + R_bi @ self.right_motor_pos)
        window.addItem(self.right_motor)  # add right motor to drawing
        # left motor
        self.left_motor_pos = np.array([[-2.5 * self.unit_length],
                                        [-2.0 * self.unit_length],
                                        [0]])
        R_left_motor = euler_to_rotation(0.0, state.rotor_angle_left, 0.0)
        self.left_motor = self.add_object(
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ R_left_motor,
            vtol_position + R_bi @ self.left_motor_pos)
        window.addItem(self.left_motor)  # add left motor to drawing
        # rotors
        self.rotor_points, self.rotor_index, self.rotor_meshColors = self.get_rotor_points()
        # back rotor
        self.back_rotor_pos = np.array([[-4.5 * self.unit_length], [0], [0]])
        self.back_rotor = self.add_object(
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi,
            vtol_position + R_bi @ self.back_rotor_pos)
        window.addItem(self.back_rotor)
        # right rotor
        self.R_rotor = euler_to_rotation(0.0, np.pi/2, 0)
        self.rotor_pos = np.array([[1.3 * self.unit_length],
                                   [0],
                                   [0]]) # with respect to motor
        self.right_rotor = self.add_object(
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ R_right_motor @ self.R_rotor,
            vtol_position + R_bi @ self.right_motor_pos + R_bi @ R_right_motor @ self.rotor_pos)
        window.addItem(self.right_rotor)
        # left rotor
        #self.left_rotor_position = vtol_position + R_bi @ self.left_motor_pos 
        #R_left_rotor = euler_to_rotation(0.0, np.pi / 2, 0.0)
        self.left_rotor = self.add_object(
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ R_left_motor @ self.R_rotor,
            vtol_position + R_bi @ self.left_motor_pos + R_bi @ R_left_motor @ self.rotor_pos)
        window.addItem(self.left_rotor)

    def update(self, state):
        # NED coordinates of votl
        vtol_position = state.pos
        # attitude of vtol as a rotation matrix R from body to inertial
        R_bi = state.R
        self.vtol_body = self.update_object(
            self.vtol_body,
            self.body_points,
            self.body_index,
            self.body_meshColors,
            R_bi,
            vtol_position)
        # right motor
        R_right_motor = euler_to_rotation(0.0, state.rotor_angle_right, 0.0)
        self.right_motor = self.update_object(
            self.right_motor,
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ R_right_motor,
            vtol_position + R_bi @ self.right_motor_pos)
        # left motor
        R_left_motor = euler_to_rotation(0.0, state.rotor_angle_left, 0.0)
        self.left_motor = self.update_object(
            self.left_motor,
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ R_left_motor,
            vtol_position + R_bi @ self.left_motor_pos)
        # back rotor
        self.back_rotor = self.update_object(
            self.back_rotor,
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi,
            vtol_position + R_bi @ self.back_rotor_pos)
        # right rotor
        self.right_rotor = self.update_object(
            self.right_rotor,
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ R_right_motor @ self.R_rotor,
            vtol_position + R_bi @ (self.right_motor_pos + R_right_motor @ self.rotor_pos))
        # left rotor
        self.left_rotor = self.update_object(
            self.left_rotor,
            self.rotor_points,
            self.rotor_index,
            self.rotor_meshColors,
            R_bi @ R_left_motor @ self.R_rotor,
            vtol_position + R_bi @ (self.left_motor_pos + R_left_motor @ self.rotor_pos))

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


    def get_body_points(self):
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
        index = np.array([
            [0, 1, 2],  # nose-top
            [0, 1, 4],  # nose-right
            [0, 3, 4],  # nose-bottom
            [0, 3, 2],  # nose-left
            [5, 2, 3],  # fuselage-left
            [5, 1, 2],  # fuselage-top
            [5, 1, 4],  # fuselage-right
            [5, 3, 4],  # fuselage-bottom
            [6, 7, 11],  # wing
            [7, 11, 10],  # wing
            [7, 10, 9],  # wing
            [7, 8, 9],  # wing
            [6, 11, 12],  # wing
            [6, 12, 16],  # wing
            [12, 13, 16],  # wing
            [13, 16, 14],  # wing
            [14, 15, 16],  # wing
            [7, 8, 17],  # winglet
            [17, 18, 8],  # winglet
            [15, 16, 19],  # winglet
            [15, 19, 20],  # winglet
            [21, 22, 23],  # tail
            [21, 23, 24],  # tail
            [25, 26, 27],  # tail
            [25, 27, 28],  # tail
        ])
        #   define the colors for each face of triangular mesh
        #red = np.array([1., 0., 0., 1])
        red = np.array([211, 68, 63, 256])/256
        #green = np.array([0., 1., 0., 1])
        green = np.array([63, 211, 105, 256])/256.
        blue = np.array([0., 0., 1., 1])
        #yellow = np.array([1., 1., 0., 1])
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
        return points, index, meshColors

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
        index = np.array([
            [0, 1, 2],  # end
            [0, 3, 2],  # end
            [1, 5, 6],  # top
            [1, 2, 6],  # top
            [0, 1, 5],  # right
            [0, 4, 5],  # right
            [0, 4, 7],  # bottom
            [0, 3, 7],  # bottom
            [3, 7, 6],  # left
            [3, 2, 6],  # left
            [4, 5, 6],  # end
            [4, 7, 6],  # end
        ])
        #   define the colors for each face of triangular mesh
        # red = np.array([211, 68, 63, 256])/256
        # #green = np.array([0., 1., 0., 1])
        # blue = np.array([66, 161, 244, 256])/256.
        # #yellow = np.array([1., 1., 0., 1])
        # meshColors = np.empty((12, 3, 4), dtype=np.float32)
        # meshColors[0] = blue  # end
        # meshColors[1] = blue  # end
        # meshColors[2] = blue # top
        # meshColors[3] = blue  # top
        # meshColors[4] = blue  # right
        # meshColors[5] = blue  # right
        # meshColors[6] = red  # bottom
        # meshColors[7] = red  # bottom
        # meshColors[8] = blue  # left
        # meshColors[9] = blue  # left
        # meshColors[10] = red  # end
        # meshColors[11] = red  # end
        mygrey3 = np.array([0.5, 0.5, 0.5, 1])
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
        meshColors[0] = mygrey3  # end
        meshColors[1] = mygrey3  # end
        meshColors[2] = mygrey3 # top
        meshColors[3] = mygrey3  # top
        meshColors[4] = mygrey3  # right
        meshColors[5] = mygrey3  # right
        meshColors[6] = mygrey3  # bottom
        meshColors[7] = mygrey3  # bottom
        meshColors[8] = mygrey3  # left
        meshColors[9] = mygrey3  # left
        meshColors[10] = mygrey3  # end
        meshColors[11] = mygrey3  # end

        return points, index, meshColors

    def get_rotor_points(self):
        radius = 0.8 * self.unit_length
        N = 10
        points = np.array([[0, 0, 0]])
        theta = 0
        while theta <= 2*np.pi:
            theta += 2 * np.pi / N
            new_point = np.array([[radius*np.cos(theta), radius*np.sin(theta), 0]])
            points = np.concatenate((points, new_point), axis=0)
        mygrey4 = np.array([0.3, 0.3, 0.3, 1])
        index = np.array([[0, 1, 2]])
        meshColors = np.empty((points.shape[0]-1, 3, 4))
        for i in range(1, (points.shape[0]-1)):
            new_mesh = np.array([[0, i, i+1]])
            index = np.concatenate((index, new_mesh), axis=0)
            meshColors[i] = mygrey4
        return points.T, index, meshColors
    