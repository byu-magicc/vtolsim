import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[3]))
import numpy as np
import pandas as pd


import parameters.simulation_parameters as SIM
from models.quad_dynamics_control import QuadDynamicsControl
from controllers.autopilot_fixedWing import Autopilot as AutopilotFixedWing

import time

from viewers.view_manager import ViewManager
from message_types.msg_autopilot_fixedWing import MsgAutopilot as MsgAutopilotFixedWing
from tools.signals import Signals

#imports the controllers
from controllers.low_level_control import LowLevelControl_aircraftControl
from controllers.rate_control import RateControl
from trajectory.trajectory_tracker import TrajectoryTracker
from tools.rotations import quaternion_to_euler, rotation_to_quaternion, rotation_to_euler, quaternion_to_rotation

from controllers.forces_torques_derivatives import wrenchCalculation

from trajectory.plannedTrajectories.lineTrajectories import straight_line_flight
from tools.performanceMeasures import performanceMeasures


#imports the message types
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState

#imports the matplot library
import matplotlib.pyplot as plt

import parameters.anaconda_parameters as QUAD
from copy import copy




#creates the main function
def main():

    quadrotorsExist = True
    
    #creates the initial state
    initialState = QUAD.initState

    #creates the quad and instantiates it
    quad = QuadDynamicsControl(Ts=SIM.ts_simulation,
                               quadrotorsExist=quadrotorsExist,
                               initialState=initialState)
    
    # initialize elements of the architecture
    wind = np.array([[0],[0],[0],[0],[0],[0]])
    #creates the view manager
    viewers = ViewManager(animation=True, data=True)

    # INITIALIZE TRAJECTORIES
    traj = straight_line_flight

    # draw the trajectory
    '''
    SIM.end_time = traj.end_time
    #'''
    trajectory_position_points = traj.get_position_pts(.01)
    viewers.quad_view.addTrajectory(trajectory_position_points[:3,:])


    #instantiates the trajectory controller.
    traj_tracker = TrajectoryTracker(K_p=1*np.eye(3), K_d=1.0*np.eye(3))

    #initializes the low level control
    rate_control = RateControl(ts_control=SIM.ts_control)
    control_alloc = LowLevelControl_aircraftControl(torqueControl=True)

    #initializes the command message
    delta = MsgDelta()

    #creates the list to store the state through time for post analysis
    stateArray = []

    #stores the force desired vector
    Force_Desired = []

    #stores the actual forces on the quad
    forcesMomentsActualAll = []

    #sets the simulation time
    sim_time = SIM.start_time
    Ts = SIM.ts_simulation
    end_time = SIM.end_time

    #creates the main simulation loop
    while sim_time < end_time:

        #sets the estimated state to the true state
        estimated_state = quad._state

        true_state = quad.true_state

        #saves the current state vector
        stateArray.append(copy(quad._state))


        #creates the control start time
        ctrl_start_time = time.time()
        #gets the trajectory derivatives
        currentTrajectory = traj.traj_msg(sim_time)

        #gets the positional errors
        actualPosition = estimated_state[0:3,:]
        #commanded position
        commandedPosition = currentTrajectory.pos_des_inertial

        #gets the values from the geometric high level controller
        #gets  the Desired Force vector, the desired rotation matrix, and the desired Moment vector
        F_des, R_des_inert, M_des = traj_tracker.update(state=quad.true_state, trajectory=currentTrajectory)

        Force_Desired.append(F_des)
        #from the desired force and Moment, we can run the control allocation piece,
        #at least on the aircraft low level controls.
        delta = control_alloc.update(f_d=F_des,
                                     state=quad.true_state,
                                     wind=wind,
                                     tau_desired=M_des)
        
        #-------update physical system-------------
        #updates the quad based on the delta input and the current wind conditions (0)
        quad.update(delta=delta, wind=wind)

        #gets the actual forces and moments from the quadplane
        actualForcesMoments = quad._forces_moments(delta=delta)
        
        #stores the actual forces and moments in the vector
        forcesMomentsActualAll.append(actualForcesMoments)


        #-------update viewers-------------
        #gets the current desired position in the inertial frame
        position_desired_inertial = currentTrajectory.pos_des_inertial
        #gets the current desired airspeed
        Va_desired = np.linalg.norm(currentTrajectory.vel_des_inertial)
        #gets the current desired state
        commandedState = MsgState()
        #sets the three desired positions, north east and altitude
        commandedState.north = position_desired_inertial.item(0)
        commandedState.east = position_desired_inertial.item(1)
        commandedState.altitude = -position_desired_inertial.item(2)
        #sets the desired state Va
        commandedState.Va = Va_desired

        #gets the desired roll pitch and yaw from the Rotational matrix
        commandedState.phi, commandedState.theta, commandedState.psi = rotation_to_euler(R_des_inert)

        viewers.update(sim_time=sim_time,
                       true_state=quad.true_state,
                       estimated_state=quad.true_state,
                       commanded_state=commandedState,
                       delta=delta)
        
        #increments the time by Ts seconds
        sim_time += Ts

    #gets the trajectory history
    pos_errs, vel_errs, accel_des = traj_tracker.getTrajectoryHistory()
    #gets the force control historyu
    F_ds, R_ds, M_ds = traj_tracker.getControlHistory()

    #converts the lists to useful arrays
    pos_errs = np.array(pos_errs)[:,:,0].T
    vel_errs = np.array(vel_errs)[:,:,0].T
    accel_des = np.array(accel_des)[:,:,0].T

    F_ds = np.array(F_ds)[:,:,0].T
    M_ds = np.array(M_ds)[:,:,0].T

    #iterates through all of the Rotation matrices and converts them to euler angles
    rolls = []
    pitches = []
    yaws = []
    for i in range(len(R_ds)):
        roll, pitch, yaw = rotation_to_euler(R=R_ds[i])
        #appends all of them
        rolls.append(roll)
        pitches.append(pitch)
        yaws.append(yaw)

    #converts them to arrays
    rolls = np.array(rolls)
    pitches = np.array(pitches)
    yaws = np.array(yaws)

    #converts the actual forces Moments list into a numpy array
    forcesMomentsActualAll = np.array(forcesMomentsActualAll)[:,:,0].T


    #puts together the desired forces and moments
    wrenchDesired = np.concatenate((F_ds, M_ds), axis=0)
    #writes it all out to a csv file
    wrenchDataFrame = pd.DataFrame(forcesMomentsActualAll)

    path = os.path.abspath("launch_files/trajectoryFollower/aircraftLineFollower/referenceData")
    wrenchDataFrame.to_csv(path + "/GeometricControllerOutput.csv", header=False, index=False)


    #plots the positional errors
    plt.figure(0)
    plt.plot(pos_errs[0,:], label='x error')
    plt.plot(pos_errs[1,:], label='y error')
    plt.plot(pos_errs[2,:], label='z error')
    plt.legend()
    plt.title("Position Errors")
    plt.xlabel('Error (Meters)')
    plt.show()


    
    #plots the desired forces compared with the actual forces in the z direction
    #'''
    #plots the stuff from Fx
    plt.figure(1)
    plt.plot(F_ds[0,:], label='Fx Desired')
    plt.plot(forcesMomentsActualAll[0,:], label='Fx Actual')
    plt.legend()
    plt.title("x Forces")
    plt.show()


    #plots the stuff from Fy
    plt.figure(2)
    plt.plot(forcesMomentsActualAll[1,:], label='Fy Actual')
    plt.legend()
    plt.title("y Forces")
    plt.show()

    #plots the stuff from Fz
    plt.figure(3)
    plt.plot(F_ds[1,:], label='Fz Desired')
    plt.plot(forcesMomentsActualAll[2,:], label='Fz Actual')
    plt.legend()
    plt.title("z Forces")
    plt.show()



    #plots the stuff from the 
    plt.figure(4)
    plt.plot(M_ds[0,:], label='Mx Desired')
    plt.plot(forcesMomentsActualAll[3,:], label='Mx Actual')
    plt.legend()
    plt.title('x Moments')
    plt.show()


    plt.figure(5)
    plt.plot(M_ds[1,:], label='My Desired')
    plt.plot(forcesMomentsActualAll[3,:], label='My Actual')
    plt.legend()
    plt.title('y Moments')
    plt.show()


    plt.figure(6)
    plt.plot(M_ds[2,:], label='Mz Desired')
    plt.plot(forcesMomentsActualAll[3,:], label='Mz Actual')
    plt.legend()
    plt.title('z Moments')
    plt.show()

    
    
    #'''

    '''
    #plots the desired forces
    plt.figure(1)
    plt.plot(F_ds[0,:], label='Fx')
    plt.plot(F_ds[1,:], label='Fz')
    plt.title("Desired Forces`")
    plt.show()

    #plots the actual forces
    plt.figure(2)
    plt.plot(forcesMomentsActualAll[0,:], label='Fx')
    plt.plot(forcesMomentsActualAll[1,:], label='Fy')
    plt.plot(forcesMomentsActualAll[2,:], label='Fz')
    plt.title("Actual Forces")
    plt.show()
    #'''



    potato = 0
    
    '''
    #writes them to a csv file
    pos_df = pd.DataFrame(data=(pos_errs))
    vel_df = pd.DataFrame(data=(vel_errs))
    accel_df = pd.DataFrame(data=(accel_des))

    F_df = pd.DataFrame(data=(F_ds))
    R_df = pd.DataFrame(data=R_ds)
    M_df = pd.DataFrame(data=M_ds)

    pos_df.to_csv("/home/benjamin/Documents/vtolsim/launch_files/trajectoryFollower/aircraftLineFollower/outputs/pos.csv")
    vel_df.to_csv("/home/benjamin/Documents/vtolsim/launch_files/trajectoryFollower/aircraftLineFollower/outputs/vel.csv")
    accel_df.to_csv("/home/benjamin/Documents/vtolsim/launch_files/trajectoryFollower/aircraftLineFollower/outputs/accel.csv")

    F_df.to_csv("/home/benjamin/Documents/vtolsim/launch_files/trajectoryFollower/aircraftLineFollower/outputs/Forces.csv")
    R_df.to_csv("/home/benjamin/Documents/vtolsim/launch_files/trajectoryFollower/aircraftLineFollower/outputs/Rotations.csv")
    M_df.to_csv("/home/benjamin/Documents/vtolsim/launch_files/trajectoryFollower/aircraftLineFollower/outputs/.csv")
    #'''


#calls the main function
main()

