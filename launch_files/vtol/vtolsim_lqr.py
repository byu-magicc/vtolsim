"""
vtolsim_lqr
    - simultion showing trajectory following using full state LQR
    - Update history:
        5/8/2019 - R.W. Beard
        2/1/2024 - RWB
        3/12/2024 - RWB
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
import parameters.simulation_parameters as SIM
import parameters.spline_parameters as SPLP
from message_types.msg_convert import *
from models.vtol_dynamics import VtolDynamics
from controllers.lqr.lqr_control import LqrControl
from controllers.low_level_control import LowLevelControl
from planners.trajectory.spline_trajectory import SplineTrajectory
from planners.trajectory.differential_flatness import DifferentialFlatness
from message_types.msg_delta import MsgDelta
from viewers.view_manager import ViewManager

# initialize elements of the architecture
wind = np.array([[0., 0., 0., 0., 0., 0.]]).T
vtol = VtolDynamics(SIM.ts_simulation)
viewers = ViewManager(animation=True, data=True)

# initialize trajectory
# df_traj = DifferentialFlatness(XYZSinusoid(150., 150., 75., 600., 300., 600., -np.pi/2, np.pi, 0.*np.pi/2))
# df_traj = DifferentialFlatness(HANDTraj())
df_traj = DifferentialFlatness(SplineTrajectory(SPLP.pts, SPLP.vels)) # points, max velocity
step =  0.*df_traj.traj.s_max/500
viewers.vtol_view.addTrajectory(df_traj.traj.getPList(df_traj.traj.getP, 0., df_traj.traj.s_max + step, df_traj.traj.s_max/500))

#initialize controllers
low_ctrl = LowLevelControl(M=0.5, Va0=2.0, ts=SIM.ts_simulation)
lqr_ctrl = LqrControl(SIM.ts_simulation)

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    #-------observer-------------
    measurements = vtol.sensors()  # get sensor measurements
    estimated_state = vtol._state  # estimated state is current state
    
    # ------ Trajectory follower
    desired_state, desired_input = df_traj.desiredState_fromX(estimated_state[0:10])
    
    #------- High Level controller-------------
    force_des, omega_des = lqr_ctrl.update(
        estimated_state[0:10], 
        desired_state, 
        desired_input, 
        df_traj)
    
    #------- Low Level Controller -------------
    delta = low_ctrl.update(force_des, omega_des, vtol.true_state)
    
    #-------update physical system-------------
    vtol.update(delta, wind)  # propagate the MAV dynamics

    #-------update viewers-------------
    viewers.update(
        sim_time,
        vtol.true_state,  # true states
        vtol.true_state,  # estimated states
        vtol.true_state,  # commanded states
        delta,  # inputs to aircraft
        None,  # measurements
    )
    
    #-------increment time-------------
    sim_time += SIM.ts_simulation
input("Press a key to exit")
