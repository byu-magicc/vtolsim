"""
    lqr controller for vtolsim
"""
import numpy as np
import scipy
from message_types.msg_convert import *
from controllers.lqr.lqr_dynamics import es_jacobians
from tools.quaternions import state_boxMinus, state_boxPlus

class LqrControl:
    def __init__(self, ts_control):
        self.Ts = ts_control
        self.u_prev = np.zeros((5,1))
        self.alpha = 0.5
        # step used for numeric integration/differentiation
        self.epsilon = .001
        # Q and R cost matrices
        self.Q = np.diag([
            1/10.,  # [0]  north position
            1/10.,  # [1]  east position
            1/10.,  # [2]  down position
            1/10.,  # [3]  velocity along body x-axis
            1/10.,  # [4]  velocity along body y-axis
            1/10.,  # [5]  velocity along body z-axis
            2.,     # [6]  q_tilde0
            2.,     # [7]  q_tilde1
            2.,    # [8]  q_tilde2
        ])
        self.R = np.diag([
            1.,   # ax
            1.,   # az
            10.,  # p
            10.,  # q
            150., # r
        ])

    def update(self, x, x_des, u_des, df_traj):
        """
        Control state = (p, v, q)
        Control Input = (ax, az, omega)
        """
        # find error state
        x_tilde = state_boxMinus(x_des, x)
        u_tilde = u_des - self.u_prev
        # print performance measure
        # J_x = (x_tilde.T @ Q).T * x_tilde
        # J_u = (u_tilde.T @ R).T * u_tilde
        # print("J_x = ", np.sum(J_x), " = ", J_x.T)
        # print("J_u = ", np.sum(J_u), " = ", J_u.T)
        x_tilde[0:2, 0] = np.clip(x_tilde[0:2, 0], -.5, .5)
        # find A and B matrices
        A, B = es_jacobians(x_tilde, x_des, u_tilde, u_des, df_traj) 
        # find K gain matrix
        K = self.find_K(A, B)
        u_star = K @ x_tilde
        u = self.alpha * self.u_prev + (1-self.alpha) * (u_des + u_star)
        self.u_prev = u
        # saturate controller
        f_mag = np.linalg.norm(u[0:2])
        if(f_mag > .9):
            u[0:2] = .9*u[0:2] / f_mag
        if(u.item(2) > 1.0):
            u[2] = 1.0
        if(u.item(3) > 1.0):
            u[3] = 1.0
        if(u.item(4) > 1.0):
            u[4] = 1.0
        if(u.item(0) < 0.0):
            u[0] = 0.0
        if(u.item(1) < 0.0):
            u[1] = 0.0
        if(u.item(2) < -1.0):
            u[2] = -1.0
        if(u.item(3) < -1.0):
            u[3] = -1.0
        if(u.item(4) < -1.0):
            u[4] = -1.0
        force = u[0:2]
        omega = u[2:5]
        return force, omega

    def find_K(self, A, B):
        # set up Hamiltonian matrix
        R_inv = np.linalg.inv(self.R)
        H = np.block([[A, -B @ R_inv @ B.T], [-self.Q, -A.T]])
        # find schur decomposition
        try:
            T, U, sdim = scipy.linalg.schur(H, sort='lhp')
        except np.linalg.LinAlgError as err:
            print("----- Error on Schur Decomp -----")
            print(err)
            print("\n\nA = \n", A)
            print("\n\nB = \n", B)
            print("\n\nH = \n", H)
            raise
        n = int(np.size(U, 0)/2)
        U_11 = U[0:n, 0:n]
        U_21 = U[n:, 0:n]
        # find P
        P = np.linalg.inv(U_11.T) @ U_21.T
        K = R_inv @ B.T @ P
        # print("K = ", K)
        return K







