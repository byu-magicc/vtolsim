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
from controllers.low_level_control import LowLevelControl_simultaneousControl
from controllers.rate_control import RateControl


from trajectory.pitch_free_trajectory_tracker import PitchFreeTrajectoryTracker
from trajectory.pitch_control import PitchControl
from trajectory.attitude_control import AttitudeControl


from tools.rotations import *

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

    #instantiates the autopilot
    autopilot = AutopilotFixedWing(ts_control=SIM.ts_control)

    # draw the trajectory
    '''
    SIM.end_time = traj.end_time
    #'''
    trajectory_position_points = traj.get_position_pts(.01)
    viewers.quad_view.addTrajectory(trajectory_position_points[:3,:])


    #instantiates the trajectory controller.
    traj_tracker = PitchFreeTrajectoryTracker()
    attitude_ctrl = AttitudeControl()
    pitch_ctrl = PitchControl()

    #initializes the low level control
    rate_control = RateControl(ts_control=SIM.ts_control)
    control_alloc = LowLevelControl_simultaneousControl(ts=SIM.ts_control, torqueControl=False)

    msgAutopilot = MsgAutopilotFixedWing()

    #initializes the command message
    delta = MsgDelta()

    #creates the list to store the deltas through time for post analysis
    deltaList = []

    #creates the list to store the state through time for post analysis
    stateArray = []

    #creates the list to store the true state messages
    trueStateArray = []

    #stores the force desired vector
    Forces_Desired_Desired = []

    #stores the desired forces in the inertial frame
    Forces_Desired_inertial = []

    #stores the desired angular velocities
    omega_desired = []

    #stores the actual forces on the quad
    forcesMomentsActualAll = []

    #stores the trajectory data
    trajectoryData = []

    #sets the simulation time
    sim_time = SIM.start_time
    Ts = SIM.ts_simulation
    end_time = SIM.end_time

    #creates the counter variable
    counter = 0

    #creates the main simulation loop
    while sim_time < end_time:

        #sets the estimated state to the true state
        estimated_state = quad._state

        true_state = quad.true_state

        #saves the current state vector
        stateArray.append(copy(quad._state))
        trueStateArray.append(copy(true_state))



        #creates the control start time
        ctrl_start_time = time.time()
        #gets the trajectory derivatives
        currentTrajectory = traj.traj_msg(sim_time)

        #gets the positional errors
        actualPosition = estimated_state[0:3,:]
        #commanded position
        commandedPosition = currentTrajectory.pos_des_inertial

        #stores the current trajecotry in the list
        trajectoryData.append(currentTrajectory)


        #gets the altitude, airspeed, and course angle command.
        altitude_command = -currentTrajectory.pos_des_inertial.item(2)
        #gets the desired airspeed
        airspeed_command = np.linalg.norm(currentTrajectory.vel_des_inertial)
        eastVel = currentTrajectory.vel_des_inertial.item(1)
        northVel = currentTrajectory.vel_des_inertial.item(0)
        #gets the desired course angle
        course_command = np.arctan2(eastVel, northVel)
        #writes that to the autopilot message class
        msgAutopilot.airspeed_command = airspeed_command
        msgAutopilot.altitude_command = altitude_command
        msgAutopilot.course_command = course_command
        #updates the standard autopilot
        delta_autopilot, commandedState_autopilot = autopilot.update(msgAutopilot, state=true_state)

        #gets the force and rotation desired
        F_des_des, R_des2inert = traj_tracker.update(state=quad.true_state, trajectory=currentTrajectory)
        #puts that through the pitch control
        F_des_des, R_des2inert = pitch_ctrl.update(thrust_input=F_des_des, 
                                               R_d2i=R_des2inert, 
                                               v_body=np.array([[quad.true_state.u],
                                                                [quad.true_state.v],
                                                                [quad.true_state.w]]))
        #gets the commanded angular rates vector. In this version, we will use angular rates
        #instead of desired moments
        #gets the current body frame rotation
        R_body2inert = euler_to_rotation(phi=quad.true_state.phi,
                                         theta=quad.true_state.theta,
                                         psi=quad.true_state.psi)
        omega_c = attitude_ctrl.update(R_b2i=R_body2inert, R_d2i = R_des2inert)


        Forces_Desired_Desired.append(F_des_des)

        #gets the desired force in the inertial frame

        temp_F_des_des = np.array([[F_des_des.item(0)],
                                   [0],
                                   [F_des_des.item(1)]])
        
        #gets the F desired in the inertial frame
        F_des_inert = R_des2inert @ temp_F_des_des
        #appends that to the list
        Forces_Desired_inertial.append(F_des_inert)

        omega_desired.append(omega_c)
        #from the desired force and Moment, we can run the control allocation piece,
        #at least on the aircraft low level controls.
        delta = control_alloc.update(f_d=F_des_des,
                                     state=quad.true_state,
                                     wind=wind,
                                     omega_d=omega_c)

        #appends the delta to the deltas list
        deltaList.append(copy(delta))
        
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
        commandedState.phi, commandedState.theta, commandedState.psi = rotation_to_euler(R_des2inert)

        viewers.update(sim_time=sim_time,
                       true_state=quad.true_state,
                       estimated_state=quad.true_state,
                       commanded_state=commandedState,
                       delta=delta)
        

        if counter % 150 == 0:
            potato = 0

        counter += 1
        #increments the time by Ts seconds
        sim_time += Ts




    path = os.path.abspath("launch_files/trajectoryFollower/aircraftLineFollower/seperatedControlOutputs/convergenceTuning")

    #converts to an array
    stateArray = np.array(stateArray)[:,:,0].T

    testNumber = '_104.csv'

    #writes it out now
    stateArrayDataFrame = pd.DataFrame(stateArray)
    stateArrayDataFrame.to_csv(path + "/stateOutputArray" + testNumber, index=False, header=False)


    #'''
    #creates the full delta array
    deltaArray = np.ndarray((8,0))
    #converts the delta message to a 2d array
    for delta in deltaList:
        #converts the delta list to an array
        deltaTemp = delta.to_array()
        #concatenates it onto the delta array
        deltaArray = np.concatenate((deltaArray, deltaTemp), axis=1)

    #converts it to a data frame
    deltaDataFrame = pd.DataFrame(deltaArray)
    #writes it out to a csv
    deltaDataFrame.to_csv(path + "/deltaOutputArray" + testNumber, header=False, index=False)
    #'''

    #puts together the trajectory data into an array and saves it as a csv
    #creates the array
    trajectoryArray = np.ndarray((9,0))
    for trajectoryMessage in trajectoryData:
        temp = trajectoryMessage.pos_des_inertial
        temp = np.concatenate((temp, trajectoryMessage.vel_des_inertial), axis=0)
        temp = np.concatenate((temp, trajectoryMessage.accel_des_inertial),axis=0)

        #concatenates it onto the full array
        trajectoryArray = np.concatenate((trajectoryArray, temp), axis=1)

    trajectoryDataFrame = pd.DataFrame(trajectoryArray)
    trajectoryDataFrame.to_csv(path + '/trajectoryOutputArray' + testNumber, index=False, header=False)


    Forces_Desired_Desired = np.array(Forces_Desired_Desired)[:,:,0].T


    forceDesDesDataFrame = pd.DataFrame(Forces_Desired_Desired)
    forceDesDesDataFrame.to_csv(path + '/ForceDesired' + testNumber, index=False, header=False)
    #turns the omegas and the forces into arrays and sents them out to csv files
    omega_desired = np.array(omega_desired)[:,:,0].T

    omegaDataFrame = pd.DataFrame(omega_desired)
    omegaDataFrame.to_csv(path + '/OmegaDesired' + testNumber, index=False, header=False)



    Forces_Desired_inertial = np.array(Forces_Desired_inertial)[:,:,0].T
    #writes the forces in the inertial frame
    forceDesInertDataFrame = pd.DataFrame(Forces_Desired_inertial)
    forceDesInertDataFrame.to_csv(path + '/ForcesDesiredInertial' + testNumber, index=False, header=False)


    forcesMomentsActualAll = np.array(forcesMomentsActualAll)[:,:,0].T
    #saves the forces moments actual out to the files
    forcesMomentsActualFrame = pd.DataFrame(forcesMomentsActualAll)
    forcesMomentsActualFrame.to_csv(path + '/ForcesMomentsActual' + testNumber, index=False, header=False)

    tomato = 0


#calls the main function
main()


