import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[3]))
import numpy as np
import parameters.simulation_parameters as SIM
from models.quad_dynamics_control import QuadDynamicsControl
from controllers.autopilot_fixedWing import Autopilot

import time

#imports the simulation viewer manager
from viewers.view_manager import ViewManager
from message_types.msg_autopilot_fixedWing import MsgAutopilot
from tools.signals import Signals

#imports the controllers
from controllers.low_level_control import LowLevelControl_simultaneousControl
from controllers.low_level_control import LowLevelControl_successiveControl
from controllers.low_level_control import LowLevelControl_aircraftControl
from controllers.rate_control import RateControl
from trajectory.pitch_free_trajectory_tracker import PitchFreeTrajectoryTracker
from trajectory.pitch_control import PitchControl
from trajectory.attitude_control import AttitudeControl
from tools.rotations import quaternion_to_euler, rotation_to_quaternion, rotation_to_euler, quaternion_to_rotation

#gets the wrench calculation
from controllers.forces_torques_derivatives import wrenchCalculation

import pandas as pd

from trajectory.plannedTrajectories.lineTrajectories import straight_line_flight
from tools.performanceMeasures import performanceMeasures

#imports the message types
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState

import parameters.anaconda_parameters as QUAD
from copy import copy

#creates list to store the state


#creates the main function
def main():
    quadrotorsExist = True

    #creates the initial state
    initialState = QUAD.initState

    #instantiates the quad
    quad = QuadDynamicsControl(Ts=SIM.ts_simulation, 
                               quadrotorsExist=quadrotorsExist, 
                               initialState=initialState)

    # initialize elements of the architecture
    wind = np.array([[0],[0],[0],[0],[0],[0]])
    #creates the view manager
    viewers = ViewManager(animation=True, data=True)

    # INITIALIZE TRAJECTORIES
    traj = straight_line_flight
    
    ## ---------------------------------

    # draw the trajectory
    SIM.end_time = traj.end_time
    trajectory_position_points = traj.get_position_pts(.01)
    viewers.quad_view.addTrajectory(trajectory_position_points[:3,:])
    

    #creates the arrays to store the desired thrust and 


    # initialize geometric controller
    traj_tracker = PitchFreeTrajectoryTracker()
    att_ctrl = AttitudeControl()
    pitch_ctrl = PitchControl()

    #initialize low level control
    rate_control = RateControl(ts_control=SIM.ts_control)
    control_alloc = LowLevelControl_aircraftControl(torqueControl=False)

    #initializes the command message
    delta = MsgDelta()
    
    #creates the array to store the state
    stateArray = []



    stateArray = []

    #sets the sim time
    sim_time = SIM.start_time
    Ts = SIM.ts_simulation

    #creates the main simulation loop
    while sim_time < SIM.end_time:

        #sets the estimated state to the true state
        estimated_state = quad._state


        #saves the current state vector
        stateArray.append(copy(quad._state))


        #creates the control start time
        ctrl_start_time = time.time()
        #gets the trajectory derivatives
        traj_derivatives_at_t = traj.traj_msg(sim_time)

        #gets the positional errors
        actualPosition = estimated_state[0:3,:]
        #commanded position
        commandedPosition = traj_derivatives_at_t[0:3,0].reshape((3,1))


        #------- High Level controller-------------
        #from the trajectory tracker, based on our estimated state, we get a Thrust desired (Fx, Fz)
        #and desired Rotation Matrix R_d
        T_d, R_d = traj_tracker.update(estimated_state, traj_derivatives_at_t)
        T_d, R_d = pitch_ctrl.update(T_d, R_d, estimated_state[3:6])
        T_d = T_d.reshape(-1)
        #gets the commanded roll rates vector, based on the rotation matrix
        omega_c = att_ctrl.update(quaternion_to_rotation(estimated_state[6:10]), R_d)
        omega_c = omega_c.reshape(-1)

        #------- Low Level Controller -------------
        #gets the actual roll rates vector
        omega = estimated_state[10:13,0]
        #gets the commanded tau based on the commanded and actual roll rates
        tau_c = rate_control.update(omega_c, omega)
        #gets the delta commands from the control allocation
        delta = control_alloc.update(f_d=T_d, 
                                     tau_desired=tau_c, 
                                     state=quad.true_state,
                                     wind=wind)
        ctrl_end_time = time.time()


        #-------update physical system-------------
        quad.update(delta=delta, wind=wind)  # propagate the MAV dynamics

        #-------update viewers-------------
        pd_i = traj_derivatives_at_t[0:3,0]
        va_d = np.linalg.norm(traj_derivatives_at_t[0:3,1])
        desired_state = MsgState()
        desired_state.north = pd_i.item(0)
        desired_state.east = pd_i.item(1)
        desired_state.altitude = -pd_i.item(2)
        desired_state.Va = va_d
        desired_state.phi, desired_state.theta, desired_state.chi = rotation_to_euler(R_d)

        viewers.update(sim_time=sim_time,
                       true_state=quad.true_state,
                       estimated_state=quad.true_state,
                       commanded_state=quad.true_state,
                       delta=delta)
        
        #-------increment time-------------
        sim_time += Ts


    #saves the state information into an output file
    np.savez("/home/benjamin/Documents/vtolsim/launch_files/trajectoryFollower/aircraftLineFollower/outputs/state.npz", stateArray)
    #gets the vectors from the pitch free trajectory tracker, and then saves them
    pos_errs, vel_errs, F_ds, R = traj_tracker.getInfo()
    #saves themout
    np.savez("/home/benjamin/Documents/vtolsim/launch_files/trajectoryFollower/aircraftLineFollower/outputs/PitchFreeInfo.npz", pos_errs, vel_errs, F_ds, R)


#calls the main function
main()