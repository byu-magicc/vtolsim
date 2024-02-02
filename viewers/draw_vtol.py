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
    def __init__(self, state, window, scale=0.75):
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
        # colors
        self.mygrey1 = np.array([0.8, 0.8, 0.8, 1])  # light
        self.mygrey2 = np.array([0.6, 0.6, 0.6, 1])
        self.mygrey3 = np.array([0.5, 0.5, 0.5, 1])
        self.mygrey4 = np.array([0.3, 0.3, 0.3, 1])  # dark

        # body
        #self.body_backward_x = 1.2 * self.unit_length
        self.body_points, self.body_index, self.body_meshColors = self.get_body_points()
        self.vtol_body = self.add_object(
            self.body_points,
            self.body_index,
            self.body_meshColors,
            R_bi,
            vtol_position)
        window.addItem(self.vtol_body)  # add fuselage/wing to drawing
        # rod
        self.rod_points, self.rod_index, self.rod_meshColors = self.get_rod_points()
        self.rod_position = np.array([[-self.body_backward_x], [0.0], [0.0]])
        self.rod = self.add_object(
            self.rod_points,
            self.rod_index,
            self.rod_meshColors,
            R_bi,
            vtol_position + R_bi @ self.rod_position)
        window.addItem(self.rod)  # add fuselage/wing to drawing
        # right wing
        self.wing_points, self.wing_index, self.wing_meshColors = self.get_wing_points()
        self.right_wing_position = np.array([[self.body_forward_x / 2], 
                                             [self.body_y2], 
                                             [self.body_z1]])
        self.right_wing = self.add_object(
            self.wing_points,
            self.wing_index,
            self.wing_meshColors,
            R_bi,
            vtol_position + R_bi @ self.right_wing_position)
        window.addItem(self.right_wing)  # add right_wing to drawing
        # left wing
        self.left_wing_position = np.array([[self.body_forward_x / 2], 
                                             [-self.body_y2], 
                                             [self.body_z1]])
        self.left_wing = self.add_object(
            self.wing_points,
            self.wing_index,
            self.wing_meshColors,
            R_bi @ euler_to_rotation(np.pi, 0, 0),
            vtol_position + R_bi @ self.left_wing_position)
        window.addItem(self.left_wing)  # add left_wing to drawing
        # right tail
        self.tail_points, self.tail_index, self.tail_meshColors = self.get_tail_points()
        self.right_tail_position = np.array([[-self.body_backward_x-self.rod_length+self.tail_x1], 
                                             [self.rod_width], 
                                             [0]])
        self.right_tail = self.add_object(
            self.tail_points,
            self.tail_index,
            self.tail_meshColors,
            R_bi,
            vtol_position + R_bi @ self.right_tail_position)
        window.addItem(self.right_tail)  # add right_tail to drawing
        # left tail
        self.left_tail_position = np.array([[-self.body_backward_x-self.rod_length+self.tail_x1], 
                                             [-self.rod_width], 
                                             [self.tail_z1]])
        self.left_tail = self.add_object(
            self.tail_points,
            self.tail_index,
            self.tail_meshColors,
            R_bi @ euler_to_rotation(np.pi, 0, 0),
            vtol_position + R_bi @ self.left_tail_position)
        window.addItem(self.left_tail)  # add left_tail to drawing
        # vertical tail
        self.vertical_tail_position = np.array([[-self.body_backward_x-self.rod_length+self.tail_x1], 
                                             [0], 
                                             [-self.rod_height]])
        self.vertical_tail = self.add_object(
            self.tail_points,
            self.tail_index,
            self.tail_meshColors,
            R_bi @ euler_to_rotation(-np.pi/2, 0, 0),
            vtol_position + R_bi @ self.vertical_tail_position)
        window.addItem(self.vertical_tail)  # add vertical_tail to drawing
        # motors
        self.motor_points, self.motor_index, self.motor_meshColors = self.get_motor_points()
        # right motor
        self.right_motor_pos = np.array([[0.25 * self.unit_length],
                                         [4.0 * self.unit_length],
                                         [0]])
        R_right_motor = euler_to_rotation(0.0, state.motor_angle.item(0), 0.0)
        self.right_motor = self.add_object(
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ R_right_motor,
            vtol_position + R_bi @ self.right_motor_pos)
        window.addItem(self.right_motor)  # add right motor to drawing
        # left motor
        self.left_motor_pos = np.array([[0.25 * self.unit_length],
                                        [-4.0 * self.unit_length],
                                        [0]])
        R_left_motor = euler_to_rotation(0.0, state.motor_angle.item(1), 0.0)
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
        self.back_rotor_pos = np.array([[-3.5 * self.unit_length], [0], [-0.3]])
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
        R_bi = state.R
        self.vtol_body = self.update_object(
            self.vtol_body,
            self.body_points,
            self.body_index,
            self.body_meshColors,
            R_bi,
            vtol_position)
        # rod
        self.rod = self.update_object(
            self.rod,
            self.rod_points,
            self.rod_index,
            self.rod_meshColors,
            R_bi,
            vtol_position + R_bi @ self.rod_position)       
        # right wing
        self.right_wing = self.update_object(
            self.right_wing,
            self.wing_points,
            self.wing_index,
            self.wing_meshColors,
            R_bi,
            vtol_position + R_bi @ self.right_wing_position)
        # left wing
        self.left_wing = self.update_object(
            self.left_wing,
            self.wing_points,
            self.wing_index,
            self.wing_meshColors,
            R_bi @ euler_to_rotation(np.pi, 0, 0),
            vtol_position + R_bi @ self.left_wing_position)
        # right tail
        self.right_tail = self.update_object(
            self.right_tail,
            self.tail_points,
            self.tail_index,
            self.tail_meshColors,
            R_bi,
            vtol_position + R_bi @ self.right_tail_position)
        # left tail
        self.left_tail = self.update_object(
            self.left_tail,
            self.tail_points,
            self.tail_index,
            self.tail_meshColors,
            R_bi @ euler_to_rotation(np.pi, 0, 0),
            vtol_position + R_bi @ self.left_tail_position)
        # vertical tail
        self.vertical_tail = self.update_object(
            self.vertical_tail,
            self.tail_points,
            self.tail_index,
            self.tail_meshColors,
            R_bi @ euler_to_rotation(-np.pi/2, 0, 0),
            vtol_position + R_bi @ self.vertical_tail_position)
        # right motor
        R_right_motor = euler_to_rotation(0.0, state.motor_angle.item(0), 0.0)
        self.right_motor = self.update_object(
            self.right_motor,
            self.motor_points,
            self.motor_index,
            self.motor_meshColors,
            R_bi @ R_right_motor,
            vtol_position + R_bi @ self.right_motor_pos)
        # left motor
        R_left_motor = euler_to_rotation(0.0, state.motor_angle.item(1), 0.0)
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
            Points that define the vtol body, and the colors of the triangular mesh
            Define the points on the aircraft following diagram in Figure C.3
        """
        # define fuselage body parameters
        self.body_forward_x = 2.0 * self.unit_length
        self.body_backward_x = 1.2 * self.unit_length
        self.body_y1 = 0.8 * self.unit_length
        self.body_y2 = 1.0 * self.unit_length
        self.body_y3 = 0.4 * self.unit_length
        self.body_z1 = 0.0 * self.unit_length
        body_z2 = 0.4 * self.unit_length
        body_z3 = 0.4 * self.unit_length
        body_z4 = 0.2 * self.unit_length
        # points are in NED coordinates
        #   define the points on the aircraft following diagram Fig 2.14
        points = np.array([
            [self.body_forward_x, self.body_y1, self.body_z1],  # 0
            [self.body_forward_x, self.body_y1, body_z2],  # 1
            [self.body_forward_x / 2, self.body_y2, -body_z2],  # 2
            [self.body_forward_x / 2, self.body_y1, body_z2],  # 3
            [self.body_forward_x / 2, self.body_y1, body_z3],  # 4
            [- self.body_backward_x, self.body_y3, -body_z2],  # 5
            [- self.body_backward_x, self.body_y3, body_z4],  # 6
            [- self.body_backward_x, -self.body_y3, -body_z2],  # 7
            [- self.body_backward_x, -self.body_y3, body_z4],  # 8
            [self.body_forward_x / 2, -self.body_y2, -body_z2],  # 9
            [self.body_forward_x / 2, -self.body_y1, body_z2],  # 10
            [self.body_forward_x / 2, -self.body_y1, body_z3],  # 11
            [self.body_forward_x, -self.body_y1, self.body_z1],  # 12
            [self.body_forward_x, -self.body_y1, body_z2],  # 13
            ]).T
        # point index that defines the mesh
        index = np.array([
            [0, 2, 9],  # top
            [0, 12, 9],  # top
            [2, 5, 7], # top
            [2, 9, 7],  # top
            [0, 1, 3],  # right side
            [0, 2, 3],  # right side
            [2, 4, 6], # right side
            [2, 5, 6],  # right side
            [12, 13, 10], # left side
            [12, 9, 10],  # left side
            [9, 7, 8],  # left side
            [9, 11, 8],  # left side
            [7, 8, 6],  # back
            [7, 5, 6],  # back
            [0, 1, 13],  # front top
            [0, 12, 13],  # front top
            [3, 4, 11],  # front-middle
            [3, 10, 11],  # front - middle
            [1, 3, 10],  # bottom
            [1, 13, 10],  # bottom
            [4, 6, 8],  # bottom
            [4, 11, 8],  # bottom
            ])
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
        meshColors[14] = self.mygrey2  # front top
        meshColors[15] = self.mygrey2  # front top
        meshColors[16] = self.mygrey2  # front middle
        meshColors[17] = self.mygrey2  # front middle
        meshColors[18] = self.mygrey4  # bottom
        meshColors[19] = self.mygrey4  # bottom
        meshColors[20] = self.mygrey4  # bottom
        meshColors[21] = self.mygrey4  # bottom
        return points, index, meshColors

    def get_rod_points(self):
        # define rod
        self.rod_height = 0.2 * self.unit_length
        self.rod_width = 0.2 * self.unit_length
        self.rod_length = 5 * self.unit_length

        # points are in NED coordinates
        #   define the points on the aircraft following diagram Fig 2.14
        points = np.array([[0, self.rod_width/2, self.rod_height/2],  # [0]
                           [0, self.rod_width/2, -self.rod_height/2],  # [1]
                           [0, -self.rod_width/2, -self.rod_height/2],  # [2]
                           [0, -self.rod_width/2, self.rod_height/2],  # [3]
                           [-self.rod_length, self.rod_width/2, self.rod_height/2],  # [4]
                           [-self.rod_length, self.rod_width/2, -self.rod_height/2],  # [5]
                           [-self.rod_length, -self.rod_width/2, -self.rod_height/2],  # [6]
                           [-self.rod_length, -self.rod_width/2, self.rod_height/2],  # [7]
                           ]).T

        # point index that defines the mesh
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
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
        meshColors[0] = self.mygrey2  # end
        meshColors[1] = self.mygrey2  # end
        meshColors[2] = self.mygrey2 # top
        meshColors[3] = self.mygrey2  # top
        meshColors[4] = self.mygrey2  # right
        meshColors[5] = self.mygrey2  # right
        meshColors[6] = self.mygrey2  # bottom
        meshColors[7] = self.mygrey2  # bottom
        meshColors[8] = self.mygrey2  # left
        meshColors[9] = self.mygrey2  # left
        meshColors[10] = self.mygrey2  # end
        meshColors[11] = self.mygrey2  # end
        return points, index, meshColors

    def get_wing_points(self):
        # define wing lengths
        wing_x1 = 2.5 * self.unit_length
        wing_x2 = 1.5 * self.unit_length
        wing_x3 = 3.5 * self.unit_length
        wing_y1 = self.body_y3 - self.body_y2
        wing_y2 = 6.0 * self.unit_length
        wing_z1 = 0.3 * self.unit_length
        wing_z3 = -0.1 * self.unit_length
        #   define the points wing
        points = np.array([[0, 0, 0],  # [0]
                           [-wing_x1, wing_y1, wing_z1/2],  # [1]
                           [0, 0, wing_z1],  # [2]
                           [-wing_x2, wing_y2, wing_z3],  # [3]
                           [-wing_x3, wing_y2, wing_z3],  # [4]
                           ]).T
        # point index that defines the mesh
        index = np.array([
            [0, 1, 4],  # top
            [0, 3, 4],  # top
            [2, 1, 4],  # bottom
            [2, 3, 4],  # bottom
            [0, 2, 3],  # front
            ])
        #   define the colors for each face of triangular mesh
        meshColors = np.empty((5, 3, 4), dtype=np.float32)
        meshColors[0] = self.mygrey3 # top
        meshColors[1] = self.mygrey3  # top
        meshColors[2] = self.mygrey3  # bottom
        meshColors[3] = self.mygrey3  # bottom
        meshColors[4] = self.mygrey3  # front
        return points, index, meshColors

    def get_tail_points(self):
        # define tail lengths
        self.tail_x1 = 1.25 * self.unit_length
        tail_x2 = 0.75 * self.unit_length
        tail_x3 = 1.75 * self.unit_length
        tail_y1 = 0.0
        tail_y2 = 3.0 * self.unit_length
        self.tail_z1 = 0.15 * self.unit_length
        tail_z3 = -0.05 * self.unit_length
        #   define the points tail
        points = np.array([[0, 0, 0],  # [0]
                           [-self.tail_x1, tail_y1, self.tail_z1/2],  # [1]
                           [0, 0, self.tail_z1],  # [2]
                           [-tail_x2, tail_y2, tail_z3],  # [3]
                           [-tail_x3, tail_y2, tail_z3],  # [4]
                           ]).T
        # point index that defines the mesh
        index = np.array([
            [0, 1, 4],  # top
            [0, 3, 4],  # top
            [2, 1, 4],  # bottom
            [2, 3, 4],  # bottom
            [0, 2, 3],  # front
            ])
        #   define the colors for each face of triangular mesh
        meshColors = np.empty((5, 3, 4), dtype=np.float32)
        meshColors[0] = self.mygrey4 # top
        meshColors[1] = self.mygrey4  # top
        meshColors[2] = self.mygrey4  # bottom
        meshColors[3] = self.mygrey4  # bottom
        meshColors[4] = self.mygrey4  # front
        return points, index, meshColors

    def get_motor_points(self):
        """"
            Points that define the motor
        """
        # define MAV body parameters
        height = 0.5 * self.unit_length
        width = 0.5 * self.unit_length
        length = 0.75 * self.unit_length

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
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
        meshColors[0] = self.mygrey1  # end
        meshColors[1] = self.mygrey1  # end
        meshColors[2] = self.mygrey1 # top
        meshColors[3] = self.mygrey1  # top
        meshColors[4] = self.mygrey1  # right
        meshColors[5] = self.mygrey1  # right
        meshColors[6] = self.mygrey4  # bottom
        meshColors[7] = self.mygrey4  # bottom
        meshColors[8] = self.mygrey1  # left
        meshColors[9] = self.mygrey1  # left
        meshColors[10] = self.mygrey1  # end
        meshColors[11] = self.mygrey1  # end

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
        index = np.array([[0, 1, 2]])
        meshColors = np.empty((points.shape[0]-1, 3, 4))
        for i in range(1, (points.shape[0]-1)):
            new_mesh = np.array([[0, i, i+1]])
            index = np.concatenate((index, new_mesh), axis=0)
            meshColors[i] = self.mygrey4
        return points.T, index, meshColors
    