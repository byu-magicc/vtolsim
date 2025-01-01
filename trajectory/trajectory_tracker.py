import numpy as np
import sys
sys.path.append('..')

import parameters.anaconda_parameters as QUAD
import parameters.geometric_control_parameters as GEO_CTRL
import parameters.control_parameters_airplane as AIR_CTRL
from copy import copy

from message_types.msg_state import MsgState
from message_types.msg_trajectory import MsgTrajectory

from tools.rotations import *
from tools.derivativesAndIntegration import DirtyDerivative

import parameters.simulation_parameters as SIM

from spatialmath.base import skewa, skew




#sets the simga
sigma = 0.05

#sets the dimensionality of the problem
numDimensions = 3

class TrajectoryTracker:

    #creates the initialization function

    #sets the two gain matrices,
    #K_p: the Proportional gain matrix, which maps the gain proportional to the 
    def __init__(self, K_p: np.ndarray, K_d: np.ndarray):
        #the vector that stores the positional errors
        self.pos_errs = []
        #stores the velocity errors
        self.vel_errs = []
        #stores the total desired accelerations
        self.accel_des = []
        #stores the desired Forces
        self.F_ds = []
        #stores the desired Rotations
        self.R = []
        #store the desired Moments
        self.M_ds = []

        #saves the two gain matrices
        self.K_p = K_p
        self.K_d = K_d

        #creates an matrix of dirty derivative instances
        self.RDerivativeMatrix = []
        for i in range(numDimensions):
            column = []
            for j in range(numDimensions):
                #creates an instance of the dirty derivative
                temp = DirtyDerivative(Ts=SIM.ts_simulation, sigma=sigma)
                column.append(temp)

            (self.RDerivativeMatrix).append(column)


        #creates a vector the omega derivative calculation
        self.omegaDerivativeVector = []
        for i in range(numDimensions):
            #creates a new instance of the derivative class
            temp = DirtyDerivative(Ts=SIM.ts_simulation, sigma=sigma)
            #appends the temp to the vector
            (self.omegaDerivativeVector).append(temp)
        


        #defines the x, y, and z hat basis vectors of the inertial space described in the inertial frame
        self.x_hat_inertial = np.array([[1],[0],[0]])
        self.y_hat_inertial = np.array([[0],[1],[0]])
        self.z_hat_inertial = np.array([[0],[0],[1]])


        #creates the counter for testing purposes
        self.counter = 0
    #creates the update function
    #arguments:
    #1. State: the vector low level state of the plane
    #2. trajectory: the current desired 
    #position, velocity, acceleration, and yaw from the B Spline generator
    def update(self, state: MsgState, trajectory: MsgTrajectory):

        #gets the current position of the body in the 
        #gets the actual state position

        #TODO figure out whether 
        pos_inertial = np.array([[state.north],
                                 [state.east],
                                 [-state.altitude]])
        
        #gets the actual current velocity in the body frame
        vel_body = np.array([[state.u],
                             [state.v],
                             [state.w]])
        
        
        #gets the rotation matrix to rotate from body back to inertial 
        Rotation_body_to_inertial = euler_to_rotation(phi=state.phi, theta=state.theta, psi=state.psi)

        #gets the actual velocity in the inertial frame
        vel_inertial = Rotation_body_to_inertial @ vel_body

        #gets the desired position, in the inertial frame
        pos_des_inertial = trajectory.pos_des_inertial
        #gets the desired velocity in the inertial frame
        vel_des_inertial = trajectory.vel_des_inertial
        #gets the desired acceleration in the inertial frame
        accel_des_inertial = trajectory.accel_des_inertial


        #gets the positional error in the inertial frame
        pos_error_inertial = pos_des_inertial - pos_inertial

        #gets the velocity error in the inertial frame
        vel_error_inertial = vel_des_inertial - vel_inertial

        #gets the desired acceleration in the inertial frame, summation and controlled
        accel_total_des = accel_des_inertial + self.K_p @ pos_error_inertial + self.K_d @ vel_error_inertial

        self.pos_errs.append(pos_error_inertial)
        self.vel_errs.append(vel_error_inertial)
        self.accel_des.append(accel_total_des)
        #gets the component vectors of the Rotation matrix, whcich are again expressed in the inertial frame
        i_hat_body_inertial = Rotation_body_to_inertial[:,0].reshape(numDimensions,1)
        j_hat_body_inertial = Rotation_body_to_inertial[:,1].reshape(numDimensions,1)
        k_hat_body_inertial = Rotation_body_to_inertial[:,2].reshape(numDimensions,1)

        #for this particular setup, we set the i_hat desired to the i hat body
        i_hat_desired_inertial = i_hat_body_inertial

        #Now that we have the i_hat desired, we can project the total acceleration vector onto it and obtain the net force in the X Direction
        F_x_des = QUAD.mass*np.transpose(i_hat_desired_inertial) @ accel_total_des

        #creates the alternative for finding the F_z desired vector
        F_z_des_vector = QUAD.mass * accel_total_des - F_x_des.item(0) * i_hat_desired_inertial

        #gets the alternative F_z_desired
        F_z_des = np.linalg.norm(F_z_des_vector)

        #gets the alternative k_hat_desired
        k_hat_desired_inertial = F_z_des_vector / F_z_des


        #creates the desired forces vector
        F_des = np.array([[F_x_des.item(0)],
                          [F_z_des]])
        
        #stores the desired forces
        self.F_ds.append(F_des)


        #gets the j_hat_desired_inertial, using the cross product rule in the appropriate order
        j_hat_desired_inertial = skew(k_hat_desired_inertial) @ i_hat_desired_inertial

        #puts all three together to get the appropriate Rotation matrix
        R_desired_inertial = np.concatenate((i_hat_desired_inertial, j_hat_desired_inertial, k_hat_desired_inertial), axis=1)

        #converts the desired Rotation matrix to convert to three euler angles
        phi_desired, theta_desired, psi_desired = rotation_to_euler(R=R_desired_inertial)

        if self.counter % 150 == 0:
            potato = 0

        #calls the function to get the derivative of the above matrix
        R_desired_inertial_dot = self.RotationDerivative(R_des_inert=R_desired_inertial)

        #omega input matrix
        omegaInputMatrix = np.transpose(R_desired_inertial) @ R_desired_inertial_dot
        #gets the omega vector from the skew symmetric matrix.
        omega = self.vee(M=omegaInputMatrix)

        #gets the omega_dot 
        omega_dot = (self.omegaDerivative(omega=omega))

        #finally, uses the equation given by Professor Beard to find the desired Moments
        #I believe that this moment vector is the desire moment about the Aircraft's Body Axes.
        #TODO Figure this out: whether this should actually be the case, or if it should be rotated into another frame
        #It could be in the desired frame, not the current body frame, so we may need to use the Rotation matrix we just found
        Moments_desired = QUAD.J @ omega_dot + skew(omega) @ (QUAD.J @ omega)

        #stores the rotation and moments vectors
        self.R.append(R_desired_inertial)
        self.M_ds.append(Moments_desired)


        #increments the counter by one
        self.counter += 1

        #returns the desired forces, the desired Rotation, and the desired moments in that order
        return F_des, R_desired_inertial, Moments_desired

    

    #function that gets the derivative of the rotation matrix, which is fortunately only
    #the numerical derivative of each individual component of the Matix. Nothing mathematically fancy or weird
    #Arguments:
    # 1. R_des_inert: the Desired Rotational inertia matrix, passed in, because it was just found
    # Returns:
    # 1. R_des_inert_dot: the component-wise derivative of the rotation derivative    
    def RotationDerivative(self, R_des_inert: np.ndarray):
        
        R_desired_inertial_dot = np.ndarray((numDimensions,numDimensions))
        for i in range(numDimensions):
            #gets the current column
            column = (self.RDerivativeMatrix)[i]
            for j in range(numDimensions):
                #gets the current derivative instance
                tempDerivativeFunction = column[j]
                #calls the update to get the current derivative 
                R_desired_inertial_dot[i,j] = tempDerivativeFunction.update(R_des_inert[i,j])
        
        #returns the R desired inertial dot
        return R_desired_inertial_dot
    
    #creates a function that gets the derivative of the 
    #Arguments:
    #1.
    def omegaDerivative(self, omega: np.ndarray):

        #creates the omega_dot vector initialized with nothing in it
        omega_dot = np.ndarray((numDimensions,1))

        #iterates through and updates the current derivative and saves it to the omega dot vector
        for i in range(numDimensions):
            #gets the current derivative function
            currentDerivativeFunction = (self.omegaDerivativeVector)[i]

            #calls the update function
            omega_dot[i,0] = currentDerivativeFunction.update(omega[i,0])
        

        #returns the omega dot vector
        return omega_dot


    #function that defines the vee operator, which corresponds to extracting the components of the
    #skew symmetric matrix
    #Arguments:
    #1. M - the input skew symmetric matrix
    #Returns:
    #2. v - the vector that was extracted
    def vee(self, M):
        #creates the vector v
        v = np.array([[M[2,1]],
                      [-M[2,0]],
                      [M[1,0]]])
        
        #returns the v vector
        return v
    

    #creates a function that gets all the histories of all the components as lists
    def getHistory(self):
        return self.pos_errs, self.vel_errs, self.accel_des, self.F_ds, self.R, self.M_ds


    #gets the position, velocity, accel desired hisotry
    def getTrajectoryHistory(self):
        return self.pos_errs, self.vel_errs, self.accel_des
    
    #gets the control history (the forces, Rotation  desired and moments desired vectors)
    def getControlHistory(self):
        return self.F_ds, self.R, self.M_ds
