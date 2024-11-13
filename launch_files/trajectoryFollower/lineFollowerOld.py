#This file is a carbon copy of Mason's Code, which implements a controller which follows a straight line trajectory through space

import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
import numpy as np
import time
import parameters.simulation_parameters as SIM

from viewers.view_manager import ViewManager
from models.quad_dynamics import QuadDynamics
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
from controllers.low_level_control import LowLevelControl_simultaneousControl
from controllers.rate_control import RateControl
from trajectory.pitch_free_trajectory_tracker import PitchFreeTrajectoryTracker
from trajectory.pitch_control import PitchControl
from trajectory.attitude_control import AttitudeControl
from tools.rotations import quaternion_to_euler, rotation_to_quaternion, rotation_to_euler, quaternion_to_rotation
import pandas as pd

from trajectory.plannedTrajectories.lineTrajectories import lineFlight

from tools.performanceMeasures import performanceMeasures

#creates the main function
def main():
    np.set_printoptions(precision=4, linewidth=200, suppress=True)
    # initialize viewers
    viewers = ViewManager(data=True, animation=True)

    # initialize elements of the architecture
    wind = np.array([[0],[0],[0]])
    quad = QuadDynamics(SIM.ts_simulation)

    # INITIALIZE TRAJECTORIES
    traj = lineFlight
    
    ## ---------------------------------

    # draw the trajectory
    SIM.end_time = traj.end_time
    trajectory_position_points = traj.get_position_pts(.01)

    viewers.addTrajectory(trajectory_position_points[:3,:])

    # initialize geometric controller
    traj_tracker = PitchFreeTrajectoryTracker()
    att_ctrl = AttitudeControl()
    pitch_ctrl = PitchControl()

    #initialize low level control
    rate_control = RateControl(ts_control=SIM.ts_control)
    control_alloc = LowLevelControl_simultaneousControl()




    #creates instance of the performance measures class
    performance = performanceMeasures(Ts=SIM.ts_simulation)

    # initialize command message
    delta = MsgDelta()

    #calculate_trim
    quad._update_true_state()

    # initialize the simulation time
    sim_time = SIM.start_time
    Ts = SIM.ts_simulation

    time_hist = []
    comp_time_hist = []


    # main simulation loop
    while sim_time < SIM.end_time:
        #-------observer-------------
        estimated_state = quad._state  # estimated state is current state

        ctrl_start_time = time.time()
        # ------ Trajectory follower
        traj_derivatives_at_t = traj.traj_msg(sim_time)

        #gets the positional errors
        actualPosition = estimated_state[0:3,:]
        #commanded position
        commandedPosition = traj_derivatives_at_t[0:3,0].reshape((3,1))
        #throws those into the metric equations
        performance.posErrorTracker.update(desiredPosition=commandedPosition,
                                           actualPosition=actualPosition)
      

        #------- High Level controller-------------
        #from the trajectory tracker, based on our estimated state, we get a Thrust desired (Fx, Fz)
        #and 
        T, R_d = traj_tracker.update(estimated_state, traj_derivatives_at_t)
        T, R_d = pitch_ctrl.update(T, R_d, estimated_state[3:6])
        T = T.reshape(-1)
        #gets the commanded roll rates vector, based on the rotation matrix
        omega_c = att_ctrl.update(quaternion_to_rotation(estimated_state[6:10]), R_d)
        omega_c = omega_c.reshape(-1)

        #------- Low Level Controller -------------
        #gets the actual roll rates vector
        omega = estimated_state[10:13,0]
        #gets the commanded tau based on the commanded and actual roll rates
        tau_c = rate_control.update(omega_c, omega)
        #gets the delta commands from the control allocation
        delta = control_alloc.update(T, tau_c, estimated_state, quad._Va)
        ctrl_end_time = time.time()

        #gets the motor electrical vectors
        V_in, I_in, P_in = quad.getMotorElectricals()

        #writes down the arrays in the performance matrix
        performance.energyTracker.update(V_in=V_in, I_in=I_in, P_in=P_in)


        #-------update physical system-------------
        quad.update(delta, np.array([[0.0,0.0,0.0,0.0,0.0,0.0]]).T)  # propagate the MAV dynamics

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
                       commanded_state=quad.true_state,
                       desired_state=quad.true_state,
                       delta=delta)

        time_hist.append(sim_time)
        comp_time_hist.append(ctrl_end_time - ctrl_start_time)

        #-------increment time-------------
        sim_time += Ts


#calls the main function
main()