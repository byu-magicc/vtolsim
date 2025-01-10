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
    control_alloc = LowLevelControl_aircraftControl(torqueControl=False)

    #initializes the command message
    delta = MsgDelta()

    #creates the list to store the deltas through time for post analysis
    deltaList = []

    #creates the list to store the state through time for post analysis
    stateArray = []

    #creates the list to store the true state messages
    trueStateArray = []

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
        trueStateArray.append(copy(true_state))


        #creates the control start time
        ctrl_start_time = time.time()
        #gets the trajectory derivatives
        currentTrajectory = traj.traj_msg(sim_time)

        #gets the positional errors
        actualPosition = estimated_state[0:3,:]
        #commanded position
        commandedPosition = currentTrajectory.pos_des_inertial

        #gets the force and rotation desired
        F_des, R_des2inert = traj_tracker.update(state=quad.true_state, trajectory=currentTrajectory)
        #puts that through the pitch control
        F_des, R_des2inert = pitch_ctrl.update(thrust_input=F_des, 
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


        Force_Desired.append(F_des)
        #from the desired force and Moment, we can run the control allocation piece,
        #at least on the aircraft low level controls.
        delta = control_alloc.update(f_d=F_des,
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
        
        #increments the time by Ts seconds
        sim_time += Ts




    path = os.path.abspath("launch_files/trajectoryFollower/aircraftLineFollower/seperatedControlOutputs/convergenceTuning")

    #converts to an array
    stateArray = np.array(stateArray)[:,:,0].T

    #writes it out now
    stateArrayDataFrame = pd.DataFrame(stateArray)
    stateArrayDataFrame.to_csv(path + "/verticalTest_3meters.csv", index=False, header=False)


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
    deltaDataFrame.to_csv(path + "/deltaOutputArray.csv", header=False, index=False)
    #'''





#calls the main function
main()

